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

        #region DRONE

        // User Defs //
        const string OutChannel = "DRONE_HUB";
        const string SwarmChannel = "DRONE_SWARM";
        const string PanelName = "PANEL";
        const string ControlName = "CONTROL";
        const string PortName = "PORT";

        const float THRUST_PROXY = 0.5f;
        const double GYRO_PROXY = 1.6;
        const float THRUST_SCALE = 1000f;
        const float GYRO_SCALE = 0.01f;
        const double RAD2PI = 180 / Math.PI;
        const int ALIGN_TIME = 500;
        const int EVENT_TIME = 20;

        string InChannel;
        string DebugLog;
        int AlignClock = 0;
        int EventClock = 0;
        bool bConfigured = false;
        bool bRunning = false;
        bool bTargetGood = false;
        bool bDocking = false;
        bool bDockingInitialized = false;
        DroneMode Mode = DroneMode.STANDBY;

        IMyBroadcastListener DroneEar;
        IMyBroadcastListener SwarmEar;
        IMyTextSurface DroneSurface;
        IMyRemoteControl DroneRC;
        IMyShipConnector DronePort;

        // WIP
        IMyTurretControlBlock TurretControl;
        IMyTextPanel Panel;
        //

        IMyTerminalBlock HEAD;
        Vector3 TARGET;
        //double GYRO_SUM;

        Vector3 DELTA_0;
        Vector3 DELTA_1;
        Vector3 Look;

        float[] ThrustRatios;
        List<IMyThrust>[] ThrustGroups;
        List<GyroMask> Gyros;
        Sequence DockingSequence;

        public enum Trigger
        {
            DELTA_0,
            DELTA_1,
            ALIGN,
            EVENT
        }
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
        public enum GyroAction
        {
            PITCH,
            YAW,
            ROLL
        }

        public struct ActionMask
        {
            public GyroAction Action;
            public int Sign;

            public ActionMask(GyroAction action, int sign = 1)
            {
                Action = action;
                Sign = sign;
            }
        }
        public struct Operation
        {
            public Trigger Trigger;
            public DroneMode Mode;

            public Operation(Trigger trigger, DroneMode mode)
            {
                Trigger = trigger;
                Mode = mode;
            }
        }
        public class Sequence
        {
            public int CurrentIndex;
            public Operation[] Operations;

            public Sequence(Operation[] operations)
            {
                Operations = operations;
                CurrentIndex = 0;
            }
        }
        public class GyroMask
        {
            public IMyGyro Gyro;
            public ActionMask[] RCactions;    //  0 = pitch, 1 = yaw, 2 = roll 
            public ActionMask[] DockActions;

            public GyroMask(IMyGyro gyro)
            {
                Gyro = gyro;
                RCactions = new ActionMask[3];
                DockActions = new ActionMask[3];
            }
        }

        GyroMask SetupGyroMask(IMyGyro gyro)
        {
            GyroMask newMask = new GyroMask(gyro);

            newMask.RCactions = SetupGyroActions(DroneRC, gyro);
            newMask.DockActions = SetupGyroActions(DronePort, gyro);
            return newMask;
        }
        void SetupRemoteControl()
        {
            DroneRC = (IMyRemoteControl)GridTerminalSystem.GetBlockWithName(ControlName);
            DroneRC.FlightMode = FlightMode.OneWay;
            DroneRC.SetDockingMode(true);
            DroneRC.SetCollisionAvoidance(true);
            DroneRC.SetAutoPilotEnabled(false);
            DroneRC.ClearWaypoints();
        }
        void PopulateGyros()
        {
            Gyros = new List<GyroMask>();
            List<IMyGyro> gyros = new List<IMyGyro>();
            GridTerminalSystem.GetBlocksOfType(gyros);

            foreach (IMyGyro gyro in gyros)
                Gyros.Add(SetupGyroMask(gyro));

            Echo("gyros populated");
        }
        void PopulateThrusters()
        {
            ThrustGroups = new List<IMyThrust>[6];
            ThrustRatios = new float[6];

            List<float> thrustTotals = new List<float>();

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
                float thrustTotal = 0;

                foreach (IMyThrust thrust in thrusters)
                {
                    if (thrust.Orientation.Forward != (Base6Directions.Direction)i)
                        continue;

                    ThrustGroups[i].Add(thrust);
                    thrustTotal += thrust.MaxThrust;
                }

                thrustTotals.Add(thrustTotal);
            }

            int weakestIndex = 0;

            for (int i = 1; i < 6; i++)
            {
                if (thrustTotals[i] < thrustTotals[weakestIndex])
                    weakestIndex = i;
            }

            for (int i = 0; i < 6; i++)
            {
                ThrustRatios[i] = thrustTotals[weakestIndex] / thrustTotals[i];
            }

            Echo("thrusters populated");
        }
        void Toggle(bool on)
        {
            bRunning = on;
            if (bRunning)
                Request((bDocking) ? DroneRequest.DOCK : DroneRequest.FORM);
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
        void Request(DroneRequest request)
        {
            DebugLog += "Requesting...\n";

            long[] raw = new long[] {Me.EntityId, (long)request};

            var data = ImmutableArray.Create(raw);

            IGC.SendBroadcastMessage(OutChannel, data);
        }
        bool Recieve(MyIGCMessage message)
        {
            if (message.Data is ImmutableArray<double>)
            {
                ImmutableArray<double> data = (ImmutableArray<double>)message.Data;

                DELTA_0 = new Vector3(data.ItemRef(0), data.ItemRef(1), data.ItemRef(2));
                DELTA_1 = new Vector3(data.ItemRef(3), data.ItemRef(4), data.ItemRef(5));

                if (bDocking)
                {
                    //bDockingInitialized = true;
                    Look = DELTA_1 - DELTA_0;
                    if (!bDockingInitialized)
                        Move();
                }

                else
                    Look = DELTA_0 - DELTA_1;

                DebugLog += "Vector data recieved!\n";
                return true;
            }

            if (message.Data is string)
            {
                // put something here? I dunno fuck...
                DebugLog += $"Message recieved: {message.Data}\n";
                return false;
            }

            DebugLog += "Couldn't translate!\n";

            return false;
        }
        void Idle()
        {
            //DroneSurface.WriteText("FAILED TO RECIEVE TARGET!", false);
        }
        void Move()
        {
            DebugLog += $"Moving... \n";
            DroneRC.ClearWaypoints();
            DroneRC.AddWaypoint(DELTA_0, "TARGET");
            DroneRC.SetAutoPilotEnabled(true);
            bTargetGood = false;
            bDockingInitialized = bDocking;
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

        int[] GenerateAxisData(IMyTerminalBlock myBoop)
        {
            // RUB (right, up, back)
            Vector3 pitchVector = -Base6Directions.Directions[(int)myBoop.Orientation.Left]; // take negative, need right vector
            Vector3 yawVector = Base6Directions.Directions[(int)myBoop.Orientation.Up];
            Vector3 rollVector = -Base6Directions.Directions[(int)myBoop.Orientation.Forward]; // take negative, need back vector

            int pitchAxis = 0;
            int yawAxis = 0;
            int rollAxis = 0;
            int pitchSign = 0;
            int yawSign = 0;
            int rollSign = 0;

            for (int i = 0; i < 3; i++)
            {
                if (pitchVector.GetDim(i) != 0)
                {
                    pitchAxis = i;
                    pitchSign = (int)pitchVector.GetDim(i);
                }
                if (yawVector.GetDim(i) != 0)
                {
                    yawAxis = i;
                    yawSign = (int)yawVector.GetDim(i);
                }
                if (rollVector.GetDim(i) != 0)
                {
                    rollAxis = i;
                    rollSign = (int)rollVector.GetDim(i);
                }
            }

            return new int[] { pitchAxis, yawAxis, rollAxis, pitchSign, yawSign, rollSign }; 
        }
        ActionMask[] SetupGyroActions(IMyTerminalBlock parent, IMyTerminalBlock child)
        {
            int[] childSet = GenerateAxisData(child);
            int[] parentSet = GenerateAxisData(parent);

            ActionMask[] newActions = new ActionMask[3];

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    if (childSet[j] == parentSet[i])
                    {
                        newActions[i].Action = (GyroAction)j;
                        newActions[i].Sign = parentSet[i + 3] * childSet[j + 3];
                    }
                }
            }

            return newActions;
        }
        Vector3 TransformVectorRelative(MatrixD S, Vector3 D) // S = sourceBearing, D = WorldVectorDelta
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
        float[] GenerateThrustVectors(IMyTerminalBlock head, Vector3 delta)
        {
            /*  0 - Forward
             *  1 - Backward
             *  2 - Left
             *  3 - Right
             *  4 - Up
             *  5 - Down
             */

            float distance = Vector3.Distance(delta, head.GetPosition());
            Vector3 targetDelta = delta - head.GetPosition();
            Vector3 targetNormal = TransformVectorRelative(head.CubeGrid.WorldMatrix, targetDelta);
            Vector3 driftNormal = TransformVectorRelative(head.CubeGrid.WorldMatrix, DroneRC.GetShipVelocities().LinearVelocity);
            Vector3 netNormal = targetNormal - driftNormal;
            Vector3 thrustNormal = netNormal * THRUST_SCALE;

           
            DebugLog += $"distance: {distance}\n";
            DebugLog += $"targetDelta: {targetDelta}\n";
            DebugLog += $"targetNormal: {targetNormal}\n";
            DebugLog += $"driftNormal: {driftNormal}\n";
            DebugLog += $"netNormal: {netNormal}\n";
            

            float[] thrustMag = new float[6];

            thrustMag[0] = thrustNormal.Z * ThrustRatios[0];
            thrustMag[1] = -thrustNormal.Z * ThrustRatios[1];
            thrustMag[2] = thrustNormal.X * ThrustRatios[2];
            thrustMag[3] = -thrustNormal.X * ThrustRatios[3];
            thrustMag[4] = -thrustNormal.Y * ThrustRatios[4];
            thrustMag[5] = thrustNormal.Y * ThrustRatios[5];

            //for (int i = 0; i < 6; i++)
            //    DebugLog += $"{(Base6Directions.Direction)i} thrust: {thrustMag[i]}\n";

            return thrustMag;
        }
        void UpdateBlockMeta()
        {
            if (bDocking)
            {
                HEAD = DronePort;
                Trigger trig = DockingSequence.Operations[DockingSequence.CurrentIndex].Trigger;
                TARGET = (trig == Trigger.DELTA_0 || trig == Trigger.ALIGN) ? DELTA_0 : DELTA_1;
            }
            else
            {
                HEAD = DroneRC;
                TARGET = DELTA_0;
            }

            DebugLog += $"Target: {TARGET}\n";
        }
        double[] GenerateGyroVectors()
        {
            double[] output = new double[3];

            IMyTerminalBlock head = (bDocking) ? (IMyTerminalBlock)DronePort : DroneRC;
            Vector3 normalLook = TransformVectorRelative(head.WorldMatrix, Look);

            double targetYaw = Math.Atan2(normalLook.X, -normalLook.Z);
            double targetPitch = Math.Atan2(normalLook.Y, -normalLook.Z);

            targetYaw = (targetYaw > Math.PI) ? -((2 * Math.PI) - targetYaw) : targetYaw;
            targetPitch = (targetPitch > Math.PI) ? -((2 * Math.PI) - targetPitch) : targetPitch;

            targetYaw *= RAD2PI;
            targetPitch *= RAD2PI;

            targetYaw *= GYRO_SCALE;
            targetPitch *= GYRO_SCALE;

            /*
            DebugLog += $"Target: {Target}\n";
            DebugLog += $"Look: {Look}\n";
            DebugLog += $"Normal: {normalLook}\n";
            DebugLog += $"Yaw: {targetYaw}\n";
            DebugLog += $"Pitch: {targetPitch}\n";
            */

            for (int i = 0; i < 3; i++)
                DebugLog += $"{Gyros[0].RCactions[i].Action} : {Gyros[0].RCactions[i].Sign}\n";

            output[0] = -targetPitch;
            output[1] = targetYaw;
            output[2] = 0; // Sans rrrroller pas exactement ci vous plais

            return output;
        }
        void ApplyGyros(bool kill = false)
        {
            double[] gyroVectors = (kill) ? new double[3] : GenerateGyroVectors();

            for (int i = 0; i < 3; i++)
            {
                foreach (GyroMask gyro in Gyros)
                {
                    ApplyGyroAction(gyro, i, gyroVectors[i]);
                }
            }
        }
        void ApplyGyroAction(GyroMask mask, int action, double value)
        {
            ActionMask[] actMask = (bDocking) ? mask.DockActions : mask.RCactions;

            switch (actMask[action].Action)
            {
                case GyroAction.YAW:
                    mask.Gyro.Yaw = (float)value * actMask[action].Sign;
                    break;

                case GyroAction.PITCH:
                    mask.Gyro.Pitch = (float)value * actMask[action].Sign;
                    break;

                case GyroAction.ROLL:
                    mask.Gyro.Roll = (float)value * actMask[action].Sign;
                    break;
            }
        }
        void ApplyThrust(bool kill = false)
        {
            DebugLog += "Applying Thrust...\n" +
                $"kill : {kill}\n";

            UpdateBlockMeta();
            float[] thrustVectors = (kill) ? new float[6] : GenerateThrustVectors(HEAD, TARGET);

            for (int i = 0; i < 6; i++)
            {
                foreach (IMyThrust thrust in ThrustGroups[i])
                {
                    thrust.ThrustOverride = thrustVectors[i];
                }
            }
        }
        void DockingUpdate()
        {
            if (!bDocking || !bDockingInitialized)
                return;

            switch(DockingSequence.Operations[DockingSequence.CurrentIndex].Trigger)
            {
                case Trigger.DELTA_0:
                    if (DockingSequence.CurrentIndex == 0 &&
                        ProxyCheck())
                    {
                        SwitchModes(DroneMode.MIMIC);
                        DockingSequence.CurrentIndex++;
                    }
                    if (DockingSequence.CurrentIndex == 4)
                    {
                        Request(DroneRequest.DOCK);
                        if (THRUST_PROXY >= Vector3.Distance(DELTA_0, HEAD.GetPosition()))
                        {
                            bDocking = false;
                            bDockingInitialized = false;
                            DockingSequence.CurrentIndex = 0;
                            SwitchModes(DroneMode.NAV);
                            //if (Mode == DroneMode.NAV)
                            Request(DroneRequest.RELEASE);
                            Request(DroneRequest.FORM);
                            DockingSequence.CurrentIndex++;
                        }
                    }
                        break;

                case Trigger.ALIGN:
                    Request(DroneRequest.DOCK);
                    AlignClock++;
                    DebugLog += $"AlignClock: {AlignClock}\n";
                    //if (GYRO_PROXY >= GYRO_SUM)
                    if (AlignClock >= ALIGN_TIME)
                    {
                        AlignClock = 0;
                        DockingSequence.CurrentIndex++;
                    }
                    break;

                case Trigger.DELTA_1:
                    Request(DroneRequest.DOCK);
                    if (DronePort.Status == MyShipConnectorStatus.Connectable)
                    {
                        DronePort.Connect();
                        SwitchModes(DroneMode.RESUPPLY);
                        DockingSequence.CurrentIndex++;
                    }
                    break;

                case Trigger.EVENT:
                    EventClock++;
                    DebugLog += $"AlignClock: {AlignClock}\n";
                    if (EventClock >= EVENT_TIME)
                    {
                        DronePort.Disconnect();
                        SwitchModes(DroneMode.MIMIC);
                        EventClock = 0;
                        DockingSequence.CurrentIndex++;
                    }
                    break;
            }

            DebugLog += $"DockingSequenceIndex: {DockingSequence.CurrentIndex}\n";
        }
        void DroneUpdate()
        {
            DebugLog += $"Mode: {Mode}\n";
            //DebugLog += $"Docking: {bDocking}\n";
            //DebugLog += $"D_Init: {bDockingInitialized}\n";
            //DroneRequest request = (bDocking) ? DroneRequest.DOCK : DroneRequest.FORM;

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
                            Request(DroneRequest.FORM);
                    }
                    break;

                    /*
                case DroneMode.MIMIC:
                    Request(DroneRequest.FORM);
                    if (bTargetGood)
                    {
                        ApplyThrust();
                        ApplyGyros();
                    }
                    else
                    {
                        ApplyThrust(true);
                        ApplyGyros(true);
                    }
                    break;
                    */
            }
        }
        void MimicMode()
        {
            if (Mode != DroneMode.MIMIC)
                return;

            Request((bDocking) ? DroneRequest.DOCK : DroneRequest.FORM);
            if (!bTargetGood)
            {
                ApplyThrust(true);
                ApplyGyros(true);
            }
            else
            {
                ApplyThrust();
                ApplyGyros();
            }
        }
        void ToggleGyros(bool toggle = false)
        {
            //int count = 0;
            //DroneSurface.WriteText($"oh rearry\n", true);

            foreach (GyroMask gyro in Gyros)
            {
                //DroneSurface.WriteText($"yo {count}\n", true);

                if (!toggle)
                {
                    gyro.Gyro.Yaw = 0;
                    gyro.Gyro.Pitch = 0;
                    gyro.Gyro.Roll = 0;
                }

                gyro.Gyro.GyroOverride = toggle;
            }
        }
        void SwitchModes(DroneMode mode)
        {
            Mode = mode;

            if (Mode == DroneMode.MIMIC)
                ToggleGyros(true);

            if (Mode != DroneMode.MIMIC)
            {
                ApplyThrust(true);
                ToggleGyros();
            }

            if (Mode != DroneMode.NAV)
                DroneRC.SetAutoPilotEnabled(false);

            if (Mode == DroneMode.NAV)
                bTargetGood = false;
        }
        void SwitchModes()
        {
            Mode = (Mode == DroneMode.MIMIC) ? 0 : Mode + 1;

            if (Mode == DroneMode.MIMIC)
                ToggleGyros(true);

            if (Mode != DroneMode.MIMIC)
            {
                ApplyThrust(true);
                ToggleGyros();
            }

            if (Mode != DroneMode.NAV)
                DroneRC.SetAutoPilotEnabled(false);
        }
        void DockingSequenceBuilder()
        {
            Operation[] operations = new Operation[5];

            operations[0] = new Operation(Trigger.DELTA_0, DroneMode.NAV);      // Head towards docking port
            operations[1] = new Operation(Trigger.ALIGN, DroneMode.MIMIC);      // Align to docking port
            operations[2] = new Operation(Trigger.DELTA_1, DroneMode.MIMIC);    // Approach docking port and connect
            operations[3] = new Operation(Trigger.EVENT, DroneMode.RESUPPLY);   // Resupply and disconnect
            operations[4] = new Operation(Trigger.DELTA_0, DroneMode.MIMIC);    // Clear away from docking port
            //operations[5] = new Operation(Trigger.DELTA_0, DroneMode.NAV);    // Return to pevious assignment

            DockingSequence = new Sequence(operations);
        }
        void InitiateDockingSequence()
        {
            bDocking = true;
            bDockingInitialized = false;
            bTargetGood = false;
            DockingSequence.CurrentIndex = 0;
            AlignClock = 0;
            SwitchModes(DroneMode.NAV);
            Request(DroneRequest.DOCK);
        }

        public Program()
        {
            try
            {
                Panel = (IMyTextPanel)GridTerminalSystem.GetBlockWithName(PanelName);
                DronePort = (IMyShipConnector)GridTerminalSystem.GetBlockWithName(PortName);

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

                DockingSequenceBuilder();
                SetupRemoteControl();
                PopulateThrusters();
                PopulateGyros();

                SwitchModes(DroneMode.STANDBY);

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
                case "TOGGLE":
                    Toggle(!bRunning);
                    break;

                case "TEST":
                    InitiateDockingSequence();
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

            MimicMode();

            if (bDocking)
                DockingUpdate();
            else
                DroneUpdate();  
        }

        #endregion

        public void Save()
        {

        }
    }
}
