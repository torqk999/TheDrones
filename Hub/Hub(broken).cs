using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        // This file contains your actual script.
        //
        // You can either keep all your code here, or you can create separate
        // code files to make your program easier to navigate while coding.
        //
        // In order to add a new utility class, right-click on your project, 
        // select 'New' then 'Add Item...'. Now find the 'Space Engineers'
        // category under 'Visual C# Items' on the left hand side, and select
        // 'Utility Class' in the main area. Name it in the box below, and
        // press OK. This utility class will be merged in with your code when
        // deploying your final script.
        //
        // You can also simply create a new utility class manually, you don't
        // have to use the template if you don't want to. Just do so the first
        // time to see what a utility class looks like.
        // 
        // Go to:
        // https://github.com/malware-dev/MDK-SE/wiki/Quick-Introduction-to-Space-Engineers-Ingame-Scripts
        //
        // to learn more about ingame scripts.

        #region HUB

        const string HubPBName = "DRONE_HUB";
        const string ShipControlName = "HUB_CONTROL";
        const string SpherePBName = "SPHERE_GEN";
        const string DockIDtag = "[DOCK]";

        const string OutChannel = "DRONE_REMOTE";
        const string InChannel = "DRONE_HUB"; 
        const string SwarmChannel = "DRONE_SWARM";

        const char Split = ':';
        const float DockDisplacement = 5;

        IMyShipController Control;
        IMyBroadcastListener HubEar;
        IMyTextSurface Debug;
        IMyTerminalBlock SpherePB;
        IMyLaserAntenna[] POLES = new IMyLaserAntenna[2];

        Random RNG;
        TimeSpan Expiration = TimeSpan.FromMinutes(1);

        List<DroneRegistry> Registered = new List<DroneRegistry>();
        List<DroneRegistry> Available = new List<DroneRegistry>();
        
        List<DroneFormSlot> AllSlots = new List<DroneFormSlot>();
        List<DroneFormSlot> OpenSlots = new List<DroneFormSlot>();

        List<DockingPort> Docks = new List<DockingPort>();

        int FormationPointsCount = 0;

        bool bForwardV = false;
        bool bShunt = false;
        bool bSwarmState = false;
        bool bConfigured;
        DroneMode SwarmMode = DroneMode.STANDBY;
        public enum DroneRequest
        {
            REGISTER,
            REASSIGN,
            FORM,
            DOCK,
            RELEASE
        }
        public enum DroneMode
        {
            STANDBY,
            NAV,
            MIMIC,
            RESUPPLY
        }

        class DroneRegistry
        {
            public long ID;
            public DroneFormSlot Slot;
            public DockingPort Port;
            public DateTime Registration;
            public DateTime LastCall;
            //public DateTime LastOperation;

            public DroneRegistry(long id = 0, DroneFormSlot slot = null)//, int index = -1)
            {
                ID = id;
                Slot = slot;
                Port = null;
                Registration = DateTime.Now;
                LastCall = Registration;
            }

            public string Save()
            {
                return $"{ID}:{Slot.FormIndex}";
            }

            public int Load(string data, List<DroneFormSlot> slots)
            {
                string[] raw = data.Split(':');
                ID = long.Parse(raw[0]);
                int index = int.Parse(raw[1]);
                if (index > -1 && index < slots.Count)
                    Slot = slots[index];

                return index;
            }
        }
        class DroneFormSlot
        {
            public int FormIndex;
            //public int PullIndex;
            public bool Occupied;

            public DroneFormSlot(int form, bool occupied = false)
            {
                FormIndex = form;
                //PullIndex = pull;
                Occupied = occupied;
            }
        }
        class DockingPort
        {
            public bool Occupied;
            public IMyShipConnector Port;

            public bool DockIsFree()
            {
                if (Occupied == true)
                    return false;

                if (Port.Status == MyShipConnectorStatus.Connectable ||
                    Port.Status == MyShipConnectorStatus.Connected)
                    return false;

                return true;
            }

            public DockingPort(IMyShipConnector port)
            {
                Port = port;
                Occupied = false;
            }
        }

        DroneRegistry RegistrationRNG(long id)
        {
            DroneFormSlot slot = ProduceSlotRNG();
            DroneRegistry newDrone = new DroneRegistry(id, slot);//, index);
            ToggleDroneAssignment(newDrone, slot);
            Registered.Add(newDrone);
            return newDrone;
        }
        DroneFormSlot ProduceSlotRNG()
        {
            DroneFormSlot newRandomSlot = (OpenSlots.Count() > 0) ? OpenSlots[RNG.Next(0, OpenSlots.Count() - 1)] : null;
            if (newRandomSlot != null)
                OpenSlots.Remove(newRandomSlot);
            return newRandomSlot;
        }
        bool Assignment(DroneRegistry drone, DroneRequest mode)
        {
            //int offset = (bForwardV) ? drone.Index : 5 - drone.Index;   // Formation switching rough implementation

            drone.LastCall = DateTime.Now;

            switch (mode)
            {
                case DroneRequest.REASSIGN:

                    // DO DEES JU LAZY FACK

                    break;

                case DroneRequest.FORM:
                    if (drone.Slot == null)
                        drone.Slot = ProduceSlotRNG();

                    if (drone.Slot != null)
                    {
                        IGC.SendBroadcastMessage($"DRONE_{drone.ID}", GenerateCoordData(drone.Slot.FormIndex));
                        return true;
                    }
                    else
                    {
                        IGC.SendBroadcastMessage($"DRONE_{drone.ID}", "No spots sam...");
                        return false;
                    }

                case DroneRequest.DOCK:

                    if (drone.Slot != null)
                        ToggleDroneAssignment(drone);

                    if (drone.Port == null)
                        drone.Port = Docks.Find(x => x.DockIsFree());

                    if (drone.Port != null)
                    {
                        drone.Port.Occupied = true;
                        IGC.SendBroadcastMessage($"DRONE_{drone.ID}", GenerateDockData(drone.Port));
                        return true;
                    }
                    else
                    {
                        IGC.SendBroadcastMessage($"DRONE_{drone.ID}", GenerateLobbyData()); // Problemo
                        //IGC.SendBroadcastMessage($"DRONE_{drone.ID}", "No docks sam...");
                        return false;
                    }

                case DroneRequest.RELEASE:

                    if(drone.Port != null)
                    {
                        drone.Port.Occupied = false;
                        drone.Port = null;
                    }

                    DroneFormSlot newSlot = ProduceSlotRNG();
                    ToggleDroneAssignment(drone, newSlot);

                    break;
            }
            return false;
        }
        ImmutableArray<double> GenerateCoordData(int droneIndex)
        {
            string[] raw0 = SpherePB.CustomData.Split('\n')[droneIndex].Split(Split);
            double[] raw1 = new double[9];

            raw1[0] = double.Parse(raw0[0]);
            raw1[1] = double.Parse(raw0[1]);
            raw1[2] = double.Parse(raw0[2]);

            Vector3 pos = Control.GetPosition();
            raw1[3] = pos.X;
            raw1[4] = pos.Y;
            raw1[5] = pos.Z;

            Vector3 drift = Control.GetShipVelocities().LinearVelocity;
            raw1[6] = drift.X;
            raw1[7] = drift.Y;
            raw1[8] = drift.Z;

            return ImmutableArray.Create(raw1);
        }
        ImmutableArray<double> GenerateDockData(DockingPort port)
        {
            double[] raw1 = new double[10];

            Vector3 portPos = port.Port.GetPosition();
            Vector3 delta = port.Port.WorldMatrix.Forward * DockDisplacement;
            Vector3 alignPos = portPos + delta;

            raw1[0] = alignPos.X;
            raw1[1] = alignPos.Y;
            raw1[2] = alignPos.Z;

            raw1[3] = portPos.X;
            raw1[4] = portPos.Y;
            raw1[5] = portPos.Z;

            Vector3 drift = Control.GetShipVelocities().LinearVelocity;
            raw1[6] = drift.X;
            raw1[7] = drift.Y;
            raw1[8] = drift.Z;

            raw1[9] = 1; // trigger for docking to commence

            return ImmutableArray.Create(raw1);
        }
        ImmutableArray<double> GenerateLobbyData()
        {
            string[] raw0 = SpherePB.CustomData.Split('\n')[AllSlots.Count - 1].Split(Split);
            double[] raw1 = new double[9];
            for (int i = 0; i < 3; i++)             // Formation Deltas
                raw1[i] = double.Parse(raw0[i]);

            Vector3 pos = Control.GetPosition();
            raw1[3] = pos.X;
            raw1[4] = pos.Y;
            raw1[5] = pos.Z;

            Vector3 drift = Control.GetShipVelocities().LinearVelocity;
            raw1[6] = drift.X;
            raw1[7] = drift.Y;
            raw1[8] = drift.Z;

            return ImmutableArray.Create(raw1);
        }

        void SwarmSpeed(bool faster = true)
        {
            int BOOL = faster ? 8 : 7;
            IGC.SendBroadcastMessage(SwarmChannel, BOOL);
        }
        void SwarmFit(bool bigger = true)
        {
            int BOOL = bigger ? 6 : 5;
            IGC.SendBroadcastMessage(SwarmChannel, BOOL);
        }
        void SwarmToggle()
        {
            bSwarmState = !bSwarmState;
            int BOOL = (bSwarmState) ? 1 : 0;
            IGC.SendBroadcastMessage(SwarmChannel, BOOL);
        }
        void SwarmModeToggle()
        {
            SwarmMode = (SwarmMode == DroneMode.MIMIC) ? 0 : SwarmMode + 1;
            IGC.SendBroadcastMessage(SwarmChannel, (int)SwarmMode + 2);
        }
        void DroneCallHandler()
        {
            ImmutableArray<long> data = (ImmutableArray<long>)HubEar.AcceptMessage().Data;
            DroneRegistry drone = Registered.Find(x => x.ID == data.ItemRef(0));

            if (drone == null)
                drone = RegistrationRNG(data.ItemRef(0));

            Assignment(drone, (DroneRequest)data.ItemRef(1));
        }

        void ToggleSlot(DroneFormSlot slot, bool occupy = false)
        {
            if (slot == null)
                return;

            if (!occupy)
            {
                slot.Occupied = false;
                OpenSlots.Add(slot);
                return;
            }

            slot.Occupied = true;
            OpenSlots.Remove(slot);
        }
        void ToggleDroneAssignment(DroneRegistry reg, DroneFormSlot slot = null)
        {
            if (reg == null)
                return;

            if (reg.Slot != null)
                ToggleSlot(reg.Slot);

            reg.Slot = slot;

            if (slot == null)
            {
                Available.Add(reg);
            }
            else
            {
                Available.Remove(reg);
                ToggleSlot(slot, true);
            }
        }
        void CleanupPool()
        {
            Debug.WriteText("Cleaning Pool...\n", false);
            for (int i = Registered.Count - 1; i > -1; i--)
            {
                Debug.WriteText($"RegIndex: {i}\n", true);
                if (DateTime.Now - Registered[i].LastCall > Expiration)
                {
                    ToggleSlot(Registered[i].Slot);
                    if (Registered[i].Port != null)
                        Registered[i].Port.Occupied = false;
                    Registered.RemoveAt(i);
                }
            }
        }
        void TrickleFormation1()
        {
            string output = "Trickle\n";

            for (int i = 0; i < AllSlots.Count - 1; i++)
            {
                output += $"{i}:{AllSlots[i].Occupied}\n";

                if (AllSlots[i].Occupied ||
                    !AllSlots[i + 1].Occupied)
                    continue;

                DroneRegistry pullReg = Registered.Find(x => x.Slot == AllSlots[i+1]);

                if (pullReg == null)
                    continue;

                ToggleDroneAssignment(pullReg, AllSlots[i]);
            }

            //Debug.WriteText(output);
        }
        void TrickleFormation0()
        {
            /*
            for (int i = 0; i < AllSlots.Count; i++)
            {
                Debug.WriteText($"slot {i}:\n");

                if (AllSlots[i].PullIndex < 0 ||
                    AllSlots[i].Occupied)
                    continue;

                Debug.WriteText($"yo 0\n", true);

                DroneFormSlot pullSlot = AllSlots[AllSlots[i].PullIndex];

                Debug.WriteText($"yo 1\n", true);

                if (!pullSlot.Occupied)
                    continue;

                Debug.WriteText($"yo 2\n", true);

                DroneRegistry pullReg = Registered.Find(x => x.Slot == pullSlot);

                Debug.WriteText($"yo 3\n", true);

                if (pullReg == null)
                    continue;

                Debug.WriteText($"yo 4\n");

                ToggleDroneAssignment(pullReg, AllSlots[i]);
                ToggleSlot(pullSlot);
            }
            */
        }

        void GenerateDocks()
        {
            Docks.Clear();

            List<IMyShipConnector> blocks = new List<IMyShipConnector>();
            GridTerminalSystem.GetBlocksOfType(blocks);

            foreach (IMyShipConnector portBlock in blocks)
            {
                if (!portBlock.CustomName.Contains(DockIDtag))
                    continue;

                DockingPort newPort = new DockingPort(portBlock);
                Docks.Add(newPort);
            }
        }
        bool GeneratePool()
        {
            SpherePB = GridTerminalSystem.GetBlockWithName(SpherePBName);
            try
            {
                string[] raw0 = SpherePB.CustomData.Split('\n');
                FormationPointsCount = raw0.Length;     // / 2; // ROUGH HACK (divide by 2)!!!
                AllSlots.Clear();
                OpenSlots.Clear();
                for (int i = 0; i < FormationPointsCount; i++)
                {
                    DroneFormSlot newSlot = new DroneFormSlot(i);

                    AllSlots.Add(newSlot);
                    OpenSlots.Add(newSlot);
                }

                RNG = new Random();
                Registered.Clear();

                return true;
            }
            catch
            {
                return false;
            }
        }

        public Program()
        {
            Control = (IMyShipController)GridTerminalSystem.GetBlockWithName(ShipControlName);

            Debug = Me.GetSurface(0);
            Debug.ContentType = ContentType.TEXT_AND_IMAGE;
            Me.CustomName = HubPBName;

            bConfigured = GeneratePool();
            if (bConfigured)
                Load();
            GenerateDocks();

            Debug.WriteText($"Configuration successful: {bConfigured}\n" +
                $"Total Point Count: {FormationPointsCount}\n", false);

            HubEar = IGC.RegisterBroadcastListener(InChannel);
            HubEar.SetMessageCallback();
        }
        public void Main(string argument, UpdateType updateSource)
        {
            switch (argument)
            {
                case "SWARM":
                    SwarmToggle();
                    break;

                case "MODE":
                    SwarmModeToggle();
                    break;

                case "FIT_UP":
                    SwarmFit();
                    break;

                case "FIT_DOWN":
                    SwarmFit(false);
                    break;

                case "SPEED_UP":
                    SwarmSpeed();
                    break;

                case "SPEED_DOWN":
                    SwarmSpeed(false);
                    break;

                case "SHUNT":
                    bShunt = !bShunt;
                    break;

                case "FORMATION":
                    bForwardV = !bForwardV;
                    break;

                case "CLEAR":
                    Storage = string.Empty;
                    break;
            }

            CleanupPool();

            int count = 0;
            while (HubEar.HasPendingMessage)
            {
                DroneCallHandler();
                count++;
            }

            if (bShunt)
                TrickleFormation1();
            
            Debug.WriteText(
                $"Configuration successful: {bConfigured}\n" +
                $"Current Swarm Mode: {SwarmMode}\n" +
                $"Swarm Currently Running: {bSwarmState}\n" +
                $"Total FormationPoints Count: {FormationPointsCount}\n" +
                $"Total DockingPorts Count: {Docks.Count()}\n" +
                $"Total operations last stack: {count}\n" +
                $"Last operation: {DateTime.Now}\n", false);

            string extra = string.Empty;
            int i = 0;
            foreach (DockingPort port in Docks)
            {
                extra += $"{i}:{port.Occupied}\n";
                i++;
            }
            Debug.WriteText(extra, true);
            
        }
        public void Save()
        {
            Storage = string.Empty;
            foreach (DroneRegistry reg in Registered)
                Storage += reg.Save() + "\n";
        }
        public void Load()
        {
            try
            {
                string[] raw0 = Storage.Split('\n');

                for (int i = 0; i < raw0.Length; i++)
                {
                    DroneRegistry reg = new DroneRegistry();
                    Available.RemoveAt(reg.Load(raw0[i], AllSlots));
                    Registered.Add(reg);
                }
                Echo("Loaded");
            }
            catch
            {
                Echo("Nothing to load");
            }
        }

        #endregion


    }
}
