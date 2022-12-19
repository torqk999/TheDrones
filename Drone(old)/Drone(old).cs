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

        #region DRONE

        // User Defs //

        const string OutChannel = "DRONE_HUB";
        const string SwarmChannel = "DRONE_SWARM";
        const string PanelName = "DERP";
        const string ControlName = "CONTROL";
        const string PortName = "PORT";
        const string RadioName = "RADIO";
        const char Split = ':';
        const float PROXY = 1.5f;

        const float THRUST_SCALE = 1000f;

        //////////

        string InChannel;
        string DebugLog;
        bool bConfigured = false;
        bool bRunning = false;
        bool bTargetGood = false;
        DroneMode Mode = DroneMode.STANDBY;

        IMyBroadcastListener DroneEar;
        IMyBroadcastListener SwarmEar;
        IMyTurretControlBlock TurretControl;
        IMyTextPanel Panel;
        IMyTextSurface DroneSurface;
        IMyRemoteControl DroneRC;
        IMyShipConnector DronePort;
        IMyRadioAntenna DroneRadio;

        Vector3 Target;

        List<IMyThrust>[] ThrustGroups;

        public enum DroneMode
        {
            STANDBY,
            NAV,
            MIMIC
        }

        void PopulateThrusters()
        {
            ThrustGroups = new List<IMyThrust>[6];

            List<IMyThrust> thrusters = new List<IMyThrust>();
            GridTerminalSystem.GetBlocksOfType(thrusters);

            /*  0 - Forward
             *  1 - Backward
             *  2 - Left
             *  3 - Right
             *  4 - Up
             *  5 - Down
             */

            for (int i = 0; i < 6; i++)
            {
                ThrustGroups[i] = new List<IMyThrust>();

                foreach (IMyThrust thrust in thrusters)
                {
                    if (thrust.Orientation.Forward == (Base6Directions.Direction)i)
                        ThrustGroups[i].Add(thrust);
                }
            }
        }
        void Toggle(bool on)
        {
            bRunning = on;
            if (bRunning)
                Request();
        }
        void SwarmResponse(MyIGCMessage message)
        {
            try
            {
                int data = (int)message.Data;

                switch (data)
                {
                    case 0:
                        Toggle(false);
                        break;

                    case 1:
                        Toggle(true);
                        break;

                    case 2:
                        SwitchModes(DroneMode.STANDBY);
                        break;

                    case 3:
                        SwitchModes(DroneMode.NAV);
                        break;

                    case 4:
                        SwitchModes(DroneMode.MIMIC);
                        break;
                }
            }
            catch
            {
                DroneSurface.WriteText("Failed to interpret message. Not a string object!");
            }
        }
        void Request()
        {
            DebugLog += "Requesting...\n";
            IGC.SendBroadcastMessage(OutChannel, Me.EntityId);
        }
        bool Recieve(MyIGCMessage message)
        {
            string[] raw = ((string)message.Data).Split(Split);

            try
            {
                Target.X = float.Parse(raw[0]);
                Target.Y = float.Parse(raw[1]);
                Target.Z = float.Parse(raw[2]);
            }
            catch
            {
                return false;
            }

            DebugLog += "Recieved!\n";

            return true;
        }
        void Idle()
        {
            //DroneSurface.WriteText("FAILED TO RECIEVE TARGET!", false);
        }
        void Move()
        {
            DebugLog += $"Moving... \n";
            DroneRC.ClearWaypoints();
            DroneRC.AddWaypoint(Target, "TARGET");
            DroneRC.SetAutoPilotEnabled(true);
            bTargetGood = false;
        }
        bool ProxyCheck()
        {
            DebugLog += "ProxyCheck... \n";
            return !DroneRC.IsAutoPilotEnabled;
            //return (Vector3.Distance(Target, DroneRC.GetPosition()) < PROXY);
        }

        /////////////////////////////

        void foo0()
        {
            //TurretControl.GetTargetedEntity().Position
        }
        void foo1()
        {
            MyShipVelocities velocityPack = DroneRC.GetShipVelocities();

            //velocityPack.LinearVelocity
        }
        Vector3 NormalizeVectorRelative(MatrixD S, Vector3 D) // S = sourceBearing, D = WorldVectorDelta
        {
            /* X,Y,Z = Normalized Vector Unit Coefficients
             * x,y,z = Delta Target Vector (raw World GPS)
             * a,b,c = Normalized X vector components (relative x,y,z)
             * d,e,f = Normalized Y ''
             * g,h,i = Normalized Z ''
             * 
             * Keen Implementation:
             * Row 1: Right.x , Right.y , Right.z
             * Row 2: Up.x    , Up.y    , Up.z
             * Row 3: Back.x  , Back.y  , Back.z
            */

            Vector3D NV = new Vector3D(); // new NormalizedVector

            // Z = (d(bz - cy) + e(cx - az) + f(ay - bx)) / (d(bi - ch) + e(cg - ai) + f(ah - bg))

            // Z = (d     * ((b     * z)   - (c     * y))   + e     * ((c     * x)   - (a     * z))   + f     * ((a     * y)   - (b     * x)))   / (d     * ((b     * i)     - (c     * h))     + e     * ((c     * g)     - (a     * i))     + f     * ((a     * h)     - (b     * g)))
            NV.Z = (S.M21 * ((S.M12 * D.Z) - (S.M13 * D.Y)) + S.M22 * ((S.M13 * D.X) - (S.M11 * D.Z)) + S.M23 * ((S.M11 * D.Y) - (S.M12 * D.X))) / (S.M21 * ((S.M12 * S.M33) - (S.M13 * S.M32)) + S.M22 * ((S.M13 * S.M31) - (S.M11 * S.M33)) + S.M23 * ((S.M11 * S.M32) - (S.M12 * S.M31)));


            // Y = (Z(hc - ib) + zb - yc) / (fb - ec)
            // Y = (Z(gb - ha) + ya - xb) / (ea - db)

            // Y = (Z    * ((h     * c)     - (i     * b))     + (z   * b)     - (y   * c))     / ((f     * b)     - (e     * c))
            NV.Y = (NV.Z * ((S.M32 * S.M13) - (S.M33 * S.M12)) + (D.Z * S.M12) - (D.Y * S.M13)) / ((S.M23 * S.M12) - (S.M22 * S.M13));

            // X = (x - (Yd + Zg)) / a
            // X = (y - (Ye + Zh)) / b
            // X = (z - (Yf + Zi)) / c

            // X = (x   - ((Y    * d)     + (Z    * g)))     / a
            NV.X = (D.X - ((NV.Y * S.M21) + (NV.Z * S.M31))) / S.M11;

            return NV;
        }
        float[] GenerateThrustVectors()
        {
            /*  0 - Forward
             *  1 - Backward
             *  2 - Left
             *  3 - Right
             *  4 - Up
             *  5 - Down
             */

            float distance = Vector3.Distance(Target, DroneRC.GetPosition());
            Vector3 targetDelta = Target - DroneRC.GetPosition();
            Vector3 targetNormal = NormalizeVectorRelative(DroneRC.WorldMatrix, targetDelta);
            Vector3 driftNormal = NormalizeVectorRelative(DroneRC.WorldMatrix, DroneRC.GetShipVelocities().LinearVelocity);
            Vector3 netNormal = targetNormal - driftNormal;
            Vector3 thrustNormal = netNormal * THRUST_SCALE;

            DebugLog += $"distance: {distance}\n";
            DebugLog += $"targetDelta: {targetDelta}\n";
            DebugLog += $"targetNormal: {targetNormal}\n";
            DebugLog += $"driftNormal: {driftNormal}\n";
            DebugLog += $"netNormal: {netNormal}\n";

            float[] thrustMag = new float[6];

            thrustMag[0] = thrustNormal.Z;
            thrustMag[1] = -thrustNormal.Z;
            thrustMag[2] = thrustNormal.X;
            thrustMag[3] = -thrustNormal.X;
            thrustMag[4] = -thrustNormal.Y;
            thrustMag[5] = thrustNormal.Y;

            for (int i = 0; i < 6; i++)
                DebugLog += $"{(Base6Directions.Direction)i} thrust: {thrustMag[i]}\n";

            return thrustMag;
        }
        void ApplyThrust(bool kill = false)
        {
            DebugLog += "Applying Thrust...\n" +
                $"kill : {kill}\n";

            Vector3 delta = Target - DroneRC.GetPosition();

            DebugLog += $"Delta: {delta}\n";

            float[] thrustVectors = (kill) ? new float[6] : GenerateThrustVectors();
            for (int i = 0; i < 6; i++)
            {
                foreach (IMyThrust thrust in ThrustGroups[i])
                {
                    thrust.ThrustOverride = thrustVectors[i];
                }
            }
        }
        void DroneUpdate()
        {
            DebugLog += $"Mode: {Mode}\n";

            switch (Mode)
            {
                case DroneMode.STANDBY:
                    break;

                case DroneMode.NAV:
                    if (ProxyCheck())
                    {
                        if (bTargetGood)
                            Move();
                        else
                            Request();
                    }
                    break;

                case DroneMode.MIMIC:
                    Request();
                    if (bTargetGood)
                        ApplyThrust();
                    break;
            }
        }
        void SwitchModes(DroneMode mode)
        {
            Mode = mode;

            if (Mode != DroneMode.MIMIC)
                ApplyThrust(true);

            if (Mode != DroneMode.NAV)
                DroneRC.SetAutoPilotEnabled(false);
        }

        void SwitchModes()
        {
            Mode = (Mode == DroneMode.MIMIC) ? 0 : Mode + 1;

            if (Mode != DroneMode.MIMIC)
                ApplyThrust(true);

            if (Mode != DroneMode.NAV)
                DroneRC.SetAutoPilotEnabled(false);
        }

        public Program()
        {
            try
            {
                Panel = (IMyTextPanel)GridTerminalSystem.GetBlockWithName(PanelName);
                DroneSurface = Me.GetSurface(0);
                DroneSurface.ContentType = ContentType.TEXT_AND_IMAGE;
                DroneSurface.WriteText("", false);
                DebugLog = string.Empty;

                Me.CustomName = "DRONE_PB";
                Me.CubeGrid.CustomName = $"DRONE_{Me.EntityId}";
                InChannel = $"DRONE_{Me.EntityId}";
                SwarmEar = IGC.RegisterBroadcastListener(SwarmChannel);
                DroneEar = IGC.RegisterBroadcastListener(InChannel);
                DroneEar.SetMessageCallback(InChannel);

                PopulateThrusters();
                DroneRC = (IMyRemoteControl)GridTerminalSystem.GetBlockWithName(ControlName);
                DroneRC.FlightMode = FlightMode.OneWay;
                DroneRC.SetDockingMode(true);
                DroneRC.SetCollisionAvoidance(true);

                Runtime.UpdateFrequency = UpdateFrequency.Update10;
                bConfigured = true;
            }
            catch
            {
                bConfigured = false;
            }
        }

        public void Main(string argument, UpdateType updateSource)
        {
            DroneSurface.WriteText(DebugLog);
            DebugLog = string.Empty;

            switch (argument)
            {
                case "REQUEST":
                    Request();
                    break;

                case "TOGGLE":
                    Toggle(!bRunning);
                    break;

                case "TEST":
                    foo1();
                    break;

                case "MODE":
                    SwitchModes();
                    break;
            }

            while (SwarmEar.HasPendingMessage)
                SwarmResponse(SwarmEar.AcceptMessage());

            if (!bRunning || !bConfigured)
                return;

            while (DroneEar.HasPendingMessage)
                bTargetGood = Recieve(DroneEar.AcceptMessage());

            DroneUpdate();
        }

        #endregion

        public void Save()
        {

        }

    }
}
