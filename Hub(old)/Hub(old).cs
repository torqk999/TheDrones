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
        const string SpherePBName = "SPHERE_GEN";
        const string InChannel = "DRONE_HUB";
        const string OutChannel = "DRONE_REMOTE";
        const string SwarmChannel = "DRONE_SWARM";

        IMyBroadcastListener HubEar;
        IMyTextSurface Debug;
        IMyTerminalBlock SpherePB;
        IMyLaserAntenna[] POLES = new IMyLaserAntenna[2];

        Random RNG;

        List<DroneRegistry> Registered = new List<DroneRegistry>();
        List<int> Availble = new List<int>();
        int SpherePointsCount = 0;

        bool bForwardV = false;
        bool bSwarmState = false;
        bool bConfigured;
        DroneMode Mode = DroneMode.STANDBY;

        public enum DroneMode
        {
            STANDBY,
            NAV,
            MIMIC
        }

        class DroneRegistry
        {
            public long ID;
            public int Index;
            public DateTime Registration;
            public DateTime LastCall;

            public DroneRegistry(long id, int index)
            {
                ID = id;
                Index = index;
                Registration = DateTime.Now;
                LastCall = Registration;
            }
        }

        void SwarmToggle()
        {
            bSwarmState = !bSwarmState;
            int BOOL = (bSwarmState) ? 1 : 0;
            IGC.SendBroadcastMessage(SwarmChannel, BOOL);
        }

        void SwarmMode()
        {
            Mode = (Mode == DroneMode.MIMIC) ? 0 : Mode + 1;
            IGC.SendBroadcastMessage(SwarmChannel, (int)Mode + 2);
        }

        void DroneCallHandler()
        {
            long id = (long)HubEar.AcceptMessage().Data;
            DroneRegistry drone = Registered.Find(x => x.ID == id);

            if (drone == null)
                Registration(id);
            else
                Assignment(drone);
        }

        void Registration(long id)
        {
            if (Availble.Count() > 0)
            {
                int slot = RNG.Next(0, Availble.Count() - 1);
                DroneRegistry newDrone = new DroneRegistry(id, Availble[slot]);
                Registered.Add(newDrone);
                Availble.RemoveAt(slot);
                Assignment(newDrone);
            }
            else
                IGC.SendBroadcastMessage($"DRONE_{id}", "No room sam...");
        }

        void Assignment(DroneRegistry drone)
        {
            int offset = (bForwardV) ? drone.Index : 5 - drone.Index;   // Formation switching rough implementation

            string raw = SpherePB.CustomData.Split('\n')[offset];
            IGC.SendBroadcastMessage($"DRONE_{drone.ID}", raw);
            drone.LastCall = DateTime.Now;
        }

        bool GeneratePool()
        {
            SpherePB = GridTerminalSystem.GetBlockWithName(SpherePBName);
            try
            {
                SpherePointsCount = SpherePB.CustomData.Split('\n').Length / 2; // ROUGH HACK (divide by 2)!!!
                Availble.Clear();
                for (int i = 0; i < SpherePointsCount; i++)
                    Availble.Add(i);

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
            Debug = Me.GetSurface(0);
            Debug.ContentType = ContentType.TEXT_AND_IMAGE;
            Me.CustomName = HubPBName;

            bConfigured = GeneratePool();

            Debug.WriteText($"Configuration successful: {bConfigured}\n" +
                $"Total Point Count: {Availble.Count()}\n", false);

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
                    SwarmMode();
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

            Debug.WriteText($"Configuration successful: {bConfigured}\n" +
                $"Current Swarm Mode: {Mode}\n" +
                $"Vanguard in Forward Position: {bForwardV}\n" +
                $"Swarm Currently Running: {bSwarmState}\n" +
                $"Total Point Count: {Availble.Count()}\n" +
                $"Total operations last stack: {count}\n" +
                $"Last operation: {DateTime.Now}", false);

            //Debug.WriteText($"Total operations: {count}\nLast operation: {DateTime.Now}",true);
        }

        #endregion
        public void Save()
        {

        }

    }
}
