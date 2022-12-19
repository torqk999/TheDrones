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

        #region HUB

        const string HubPBName = "DRONE_HUB";
        const string ShipControlName = "HUB_CONTROL";
        const string SpherePBName = "SPHERE_GEN";
        const string DockIDtag = "[DOCK]";

        const string InChannel = "DRONE_HUB";
        const string OutChannel = "DRONE_REMOTE";
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
        List<DockingPort> Docks = new List<DockingPort>();
        List<int> OpenFormSpots = new List<int>();

        int FormationPointsCount = 0;

        bool bForwardV = false;
        bool bSwarmState = false;
        bool bConfigured;
        DroneMode SwarmMode = DroneMode.STANDBY;
        public enum DroneRequest
        {
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
            public int Index;
            public DateTime Registration;
            public DateTime LastCall;
            public DateTime LastOperation;

            public DroneRegistry(long id, int index)
            {
                ID = id;
                Index = index;
                Registration = DateTime.Now;
                LastCall = Registration;
            }
        }
        class DockingPort
        {
            public IMyShipConnector Port;
            public DroneRegistry Occupant;

            public bool DockIsFree()
            {
                if (Occupant != null)
                    return false;

                if (Port.Status == MyShipConnectorStatus.Connectable ||
                    Port.Status == MyShipConnectorStatus.Connected)
                    return false;

                return true;
            }

            public DockingPort(IMyShipConnector port)
            {
                Port = port;
            }
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
                drone = Registration(data.ItemRef(0));

            Assignment(drone, (DroneRequest)data.ItemRef(1));
        }
        DroneRegistry Registration(long id)
        {
            int slot = (OpenFormSpots.Count() > 0) ? RNG.Next(0, OpenFormSpots.Count() - 1) : -1;
            if (slot > -1)
                OpenFormSpots.RemoveAt(slot);

            DroneRegistry newDrone = new DroneRegistry(id, OpenFormSpots[slot]);
            Registered.Add(newDrone);
            return newDrone;
        }
        bool Assignment(DroneRegistry drone, DroneRequest mode)
        {
            //int offset = (bForwardV) ? drone.Index : 5 - drone.Index;   // Formation switching rough implementation

            drone.LastCall = DateTime.Now;
            DockingPort freeDock;

            switch (mode)
            {
                case DroneRequest.FORM:
                    if (drone.Index > -1)
                    {
                        IGC.SendBroadcastMessage($"DRONE_{drone.ID}", GenerateCoordData(drone.Index));
                        return true;
                    }
                    else
                    {
                        IGC.SendBroadcastMessage($"DRONE_{drone.ID}", "No spots sam...");
                        return false;
                    }

                case DroneRequest.DOCK:
                    freeDock = Docks.Find(x => x.Occupant == drone);
                    if (freeDock == null)
                        freeDock = Docks.Find(x => x.DockIsFree());
                    if (freeDock != null)
                    {
                        freeDock.Occupant = drone;
                        IGC.SendBroadcastMessage($"DRONE_{drone.ID}", GenerateDockData(freeDock));
                        return true;
                    }
                    else
                    {
                        IGC.SendBroadcastMessage($"DRONE_{drone.ID}", "No docks sam...");
                        return false;
                    }

                case DroneRequest.RELEASE:
                    freeDock = Docks.Find(x => x.Occupant == drone);
                    if (freeDock != null)
                        freeDock.Occupant = null;
                    break;

            }
            return false;
        }
        ImmutableArray<double> GenerateCoordData(int droneIndex)
        {
            string[] raw0 = SpherePB.CustomData.Split('\n')[droneIndex].Split(Split);
            double[] raw1 = new double[6];
            for (int i = 0; i < 3; i++)             // Formation Deltas
                raw1[i] = double.Parse(raw0[i]);

            Vector3 pos = Control.GetPosition();
            raw1[3] = pos.X;
            raw1[4] = pos.Y;
            raw1[5] = pos.Z;

            return ImmutableArray.Create(raw1);
        }
        ImmutableArray<double> GenerateDockData(DockingPort port)
        {
            double[] raw1 = new double[6];

            Vector3 portPos = port.Port.GetPosition();
            Vector3 delta = port.Port.WorldMatrix.Forward * DockDisplacement;
            Vector3 alignPos = portPos + delta;

            raw1[0] = alignPos.X;
            raw1[1] = alignPos.Y;
            raw1[2] = alignPos.Z;

            raw1[3] = portPos.X;
            raw1[4] = portPos.Y;
            raw1[5] = portPos.Z;

            return ImmutableArray.Create(raw1);
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
                FormationPointsCount = SpherePB.CustomData.Split('\n').Length;// / 2; // ROUGH HACK (divide by 2)!!!
                OpenFormSpots.Clear();
                for (int i = 0; i < FormationPointsCount; i++)
                    OpenFormSpots.Add(i);

                RNG = new Random();
                Registered.Clear();

                return true;
            }
            catch
            {
                return false;
            }
        }
        void CleanupPool()
        {
            for (int i = Registered.Count - 1; i > 0; i--)
            {
                if (DateTime.Now - Registered[i].LastCall > Expiration)
                {
                    Registered.RemoveAt(i);
                    OpenFormSpots.Add(i);
                }
            }
        }
        public Program()
        {
            Control = (IMyShipController)GridTerminalSystem.GetBlockWithName(ShipControlName);

            Debug = Me.GetSurface(0);
            Debug.ContentType = ContentType.TEXT_AND_IMAGE;
            Me.CustomName = HubPBName;

            bConfigured = GeneratePool();
            GenerateDocks();

            Debug.WriteText($"Configuration successful: {bConfigured}\n" +
                $"Total Point Count: {FormationPointsCount}\n", false);

            HubEar = IGC.RegisterBroadcastListener(InChannel);
            HubEar.SetMessageCallback();

            Runtime.UpdateFrequency = UpdateFrequency.Update100;
        }

        public void Main(string argument, UpdateType updateSource)
        {
            if (updateSource == UpdateType.Update100)
                CleanupPool();

            switch (argument)
            {
                case "SWARM":
                    SwarmToggle();
                    break;

                case "MODE":
                    SwarmModeToggle();
                    break;

                case "FORMATION":
                    bForwardV = !bForwardV;
                    break;
            }

            int count = 0;
            while (HubEar.HasPendingMessage)
            {
                DroneCallHandler();
                count++;
            }

            Debug.WriteText(
                $"Configuration successful: {bConfigured}\n" +
                $"Current Swarm Mode: {SwarmMode}\n" +
                $"Swarm Currently Running: {bSwarmState}\n" +
                $"Total FormationPoints Count: {FormationPointsCount}\n" +
                $"Total DockingPorts Count: {Docks.Count()}\n" +
                $"Total operations last stack: {count}\n" +
                $"Last operation: {DateTime.Now}", false);
        }

        #endregion

        public void Save()
        {

        }
    }
}
