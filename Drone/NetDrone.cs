using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using VRage.Game.GUI.TextPanel;
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

        /*
         
        Thought Bin:

        

        void RegisterBroadcastAndCallback(IMyBroadcastListener ear, string channel, string argument)
        {

        }
         */

        #region NetDrone

        #region LegacyDrone

        // User Defs //
        //const string OutChannel = "DRONE_HUB";
        const string SwarmChannel = "DRONE_SWARM";
        const string PanelName = "PANEL";
        const string ControlName = "CONTROL";
        const string PortName = "PORT";
        const string SensorName = "SENSOR";
        const string ShieldControlName = "[A] Shield Controller";

        //const float MAX_DRIFT_MIN = 10f;
        //const float MAX_DRIFT_MAX = 1000f;
        const float DOCK_PROXY = 12f;
        const float CHECK_PROXY = 1f;
        const float DRIFT_PROXY = 0.1f;
        const double GYRO_PROXY = 1.6;
        const float THRUST_SCALE = 100000f;
        const float GYRO_SCALE = 0.01f;
        const float COLL_THRUST_SCALE = 20f;
        const double RAD2PI = 180 / Math.PI;
        const int EVENT_TIME = 20;

        string InChannel;
        string DebugLog;
        int EventClock = 0;
        bool bConfigured = false;
        bool bShieldAvailable = false;
        bool bRunning = false;
        bool bTargetGood = false;

        bool bDocking = false;
        bool bDockingInitialized = false;
        bool bDockingProxy = false;

        DroneFlight Flight = DroneFlight.STANDBY;
        DroneFlight OldFlight;
        DroneMode Mode = DroneMode.IDLE;
        DroneMode OldMode;

        IMyTerminalBlock DroneShield;
        IMyBroadcastListener DroneEar;
        IMyBroadcastListener SwarmEar;
        IMyTextSurface DroneSurface;
        IMyRemoteControl DroneRC;
        IMyShipConnector DronePort;
        IMySensorBlock DroneSensor;

        List<IMySensorBlock> DroneSensors;

        // WIP
        //IMyTurretControlBlock TurretControl;
        //

        IMyTerminalBlock HEAD;

        Vector3 TARGET;
        Vector3 DELTA_0;
        Vector3 DELTA_1;
        Vector3 Look;
        Vector3 HUB_DRIFT;

        bool SWITCH_0; // distinguish between old coord and docking coord
        bool SWITCH_1; // 

        double MaxDrift = 50f;
        float[] ThrustRatios = new float[6];
        float[] ThrustVectors = new float[6];
        double[] GyroVectors = new double[3];
        long[] RawMessage = new long[2];

        List<IMyThrust>[] ThrustGroups;
        List<GyroMask> Gyros;
        Sequence DockingSequence;

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
            public DroneFlight Flight;

            public Operation(Trigger trigger, DroneFlight flight)
            {
                Trigger = trigger;
                Flight = flight;
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

        // Call this block !

        void DockingSequenceBuilder()
        {
            Operation[] operations = new Operation[4];

            operations[0] = new Operation(Trigger.DELTA_0, DroneFlight.NAV);      // Head towards docking port
            operations[1] = new Operation(Trigger.DELTA_1, DroneFlight.MIMIC);    // Approach docking port and connect
            operations[2] = new Operation(Trigger.EVENT, DroneFlight.MIMIC);   // Resupply and disconnect
            operations[3] = new Operation(Trigger.DELTA_0, DroneFlight.MIMIC);    // Clear away from docking port

            DockingSequence = new Sequence(operations);
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
        void SetupSensor(IMySensorBlock sensor)
        {
            if (sensor == null)
                return;

            sensor.DetectOwner = true;
            sensor.DetectFriendly = true;
            sensor.DetectEnemy = true;
            sensor.DetectNeutral = true;

            sensor.DetectStations = true;
            sensor.DetectLargeShips = true;
            sensor.DetectSmallShips = true;
            sensor.DetectSubgrids = true;
            sensor.DetectAsteroids = true;
            sensor.DetectPlayers = false;

            sensor.BackExtend = 50;
            sensor.BottomExtend = 50;
            sensor.FrontExtend = 50;
            sensor.LeftExtend = 50;
            sensor.RightExtend = 50;
            sensor.TopExtend = 50;
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

            float criticalAccel = thrustTotals[weakestIndex] / DroneRC.CalculateShipMass().TotalMass;
            float stopDistance = DroneSensor == null ? 10 : DroneSensor.MaxRange - (float)DroneRC.WorldAABB.HalfExtents.Z; // Subtract half the 'length' of the ship from the sensor radius
            MaxDrift = Math.Sqrt((2 * stopDistance) / criticalAccel) * criticalAccel;

            for (int i = 0; i < 6; i++)
            {
                ThrustRatios[i] = thrustTotals[weakestIndex] / thrustTotals[i];
            }

            Echo("thrusters populated");
        }

        ///////////////

        GyroMask SetupGyroMask(IMyGyro gyro)
        {
            GyroMask newMask = new GyroMask(gyro);

            newMask.RCactions = SetupGyroActions(DroneRC, gyro);
            newMask.DockActions = SetupGyroActions(DronePort, gyro);
            return newMask;
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

        void Toggle(bool on)
        {
            bRunning = on;
            if (bRunning)
                Request((bDocking) ? DroneRequest.DOCK : DroneRequest.FORM);
        }
        void Request(DroneRequest request) // Entry-Point needs amending
        {
            DebugLog += "Requesting...\n";

            RawMessage[1] = (long)request;

            var data = ImmutableArray.Create(RawMessage);

            //IGC.SendBroadcastMessage(OutChannel, data);
        }

        /* MessageReception
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

                    case 5:
                        AdjustShield(false);
                        break;

                    case 6:
                        AdjustShield(true);
                        break;

                    case 7:
                        AdjustSpeed(false);
                        break;

                    case 8:
                        AdjustSpeed(true);
                        break;

                    default:
                        DroneSurface.WriteText("Out of range!");
                        break;
                }
            }
            catch
            {
                DroneSurface.WriteText("Failed to interpret message. Not an integer!");
            }
        }
        bool Recieve(MyIGCMessage message)
        {
            if (message.Data is ImmutableArray<double>)
            {
                ImmutableArray<double> data = (ImmutableArray<double>)message.Data;

                DELTA_0.X = (float)data.ItemRef(0); DELTA_0.Y = (float)data.ItemRef(1); DELTA_0.Z = (float)data.ItemRef(2);
                DELTA_1.X = (float)data.ItemRef(3); DELTA_1.Y = (float)data.ItemRef(4); DELTA_1.Z = (float)data.ItemRef(5);
                HUB_DRIFT.X = (float)data.ItemRef(6); HUB_DRIFT.Y = (float)data.ItemRef(7); HUB_DRIFT.Z = (float)data.ItemRef(8);

                try
                {
                    SWITCH_0 = ((int)data.ItemRef(9)) < 0 ? false : true;
                }
                catch
                {
                    SWITCH_0 = false;
                }

                if (bDocking)
                {
                    Look = DELTA_1 - DELTA_0;

                    if (!bDockingInitialized &&
                        SWITCH_0 == true)
                    {
                        bDockingInitialized = true;
                    }
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
        */

        Vector3 GenerateCollisionAvoidanceVector(IMyTerminalBlock head, bool kill = false)
        {
            Vector3 output = new Vector3();

            if (kill == true)
                return output;

            if (DroneSensor == null &&
                DroneSensors.Count == 0)
                return output;

            List<MyDetectedEntityInfo> nextSet = new List<MyDetectedEntityInfo>();
            List<MyDetectedEntityInfo> detectedEntities = new List<MyDetectedEntityInfo>();
            DroneSensor.DetectedEntities(detectedEntities);

            if (detectedEntities.Count == 0)
                return output;

            foreach (IMySensorBlock sensor in DroneSensors)
            {
                sensor.DetectedEntities(nextSet);

                foreach (MyDetectedEntityInfo info in detectedEntities)
                    nextSet.Remove(info);

                detectedEntities.AddRange(nextSet);
            }

            foreach (MyDetectedEntityInfo entity in detectedEntities)
            {
                Vector3 delta;
                float distance;
                if (entity.HitPosition != null)
                {
                    delta = head.GetPosition() - entity.HitPosition.Value;
                    distance = Vector3.Distance(Vector3.Zero, delta);
                }
                else
                {
                    delta = head.GetPosition() - entity.Position;
                    distance = Vector3.Distance(Vector3.Zero, delta);

                    double maxExtents = entity.BoundingBox.HalfExtents.X > entity.BoundingBox.HalfExtents.Y ? entity.BoundingBox.HalfExtents.X : entity.BoundingBox.HalfExtents.Y;
                    maxExtents = entity.BoundingBox.HalfExtents.Z > maxExtents ? entity.BoundingBox.HalfExtents.Z : maxExtents;
                    distance -= (float)maxExtents;
                }

                float magnitude = COLL_THRUST_SCALE / (distance * distance);
                Vector3 normalDelta = TransformVectorRelative(head.CubeGrid.WorldMatrix, delta) * magnitude;
                output += normalDelta;
            }

            return output;
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
        Vector3 VelocityCapVector(Vector3 vector, double cap)
        {
            double mag = Vector3.Distance(vector, Vector3.Zero);
            DebugLog += $"mag: {mag}\n";

            if (mag == 0)
                return Vector3.Zero;

            Vector3 newVector = Vector3.Normalize(vector * 2);
            DebugLog += $"normal: {newVector}\n";

            mag = mag > cap ? cap : mag;
            DebugLog += $"capped: {mag}\n";

            newVector *= (float)mag;
            DebugLog += $"final: {newVector}\n";
            return newVector;
        }

        void GenerateThrustVectors(IMyTerminalBlock head, Vector3 delta, bool kill = false)
        {
            /*  0 - Forward
             *  1 - Backward
             *  2 - Left
             *  3 - Right
             *  4 - Up
             *  5 - Down
             */

            if (kill)
            {
                for (int i = 0; i < 6; i++)
                    ThrustVectors[i] = 0;
                return;
            }

            float distance = Vector3.Distance(delta, head.GetPosition());
            Vector3 targetDelta = delta - head.GetPosition();
            Vector3 targetTrans = TransformVectorRelative(head.CubeGrid.WorldMatrix, targetDelta);
            Vector3 hubDriftTrans = TransformVectorRelative(head.CubeGrid.WorldMatrix, HUB_DRIFT);
            targetTrans += hubDriftTrans;

            Vector3 driftTrans = TransformVectorRelative(head.CubeGrid.WorldMatrix, DroneRC.GetShipVelocities().LinearVelocity);
            Vector3 collAvoid = GenerateCollisionAvoidanceVector(head, bDockingProxy);

            float collMagRaw = Vector3.Distance(collAvoid, Vector3.Zero);

            /*
            collAvoid = Vector3.Normalize(collAvoid);
            collMagRaw = (collMagRaw > MaxDrift) ? MaxDrift : collMagRaw;
            collAvoid *= collMagRaw;
            */
            collAvoid = VelocityCapVector(collAvoid, MaxDrift);

            Vector3 thrustVector = Vector3.Normalize(targetTrans);
            float combinedMag = (distance) / (1 + collMagRaw);
            thrustVector *= combinedMag;
            thrustVector = VelocityCapVector(thrustVector, MaxDrift);

            /*
            combinedMag = (combinedMag > MaxDrift) ? MaxDrift : combinedMag;
            combinedNormal *= combinedMag;
            */

            //combinedNormal -= driftNormal;

            thrustVector += collAvoid - driftTrans;
            thrustVector *= THRUST_SCALE;
            //Vector3 thrustNormal = netVector * THRUST_SCALE;

            DebugLog += $"Maxrift: {MaxDrift}\n";
            DebugLog += $"distance: {distance}\n";
            DebugLog += $"targetDelta: {targetDelta}\n";
            DebugLog += $"collAvoid: {collAvoid}\n";
            DebugLog += $"targetNormal: {targetTrans}\n";
            DebugLog += $"driftNormal: {driftTrans}\n";
            DebugLog += $"netNormal: {thrustVector}\n";

            ThrustVectors[0] = thrustVector.Z * ThrustRatios[0];
            ThrustVectors[1] = -thrustVector.Z * ThrustRatios[1];
            ThrustVectors[2] = thrustVector.X * ThrustRatios[2];
            ThrustVectors[3] = -thrustVector.X * ThrustRatios[3];
            ThrustVectors[4] = -thrustVector.Y * ThrustRatios[4];
            ThrustVectors[5] = thrustVector.Y * ThrustRatios[5];
        }
        void UpdateBlockMeta()
        {
            if (bDocking)
            {
                HEAD = DronePort;
                Trigger trig = DockingSequence.Operations[DockingSequence.CurrentIndex].Trigger;
                TARGET = (trig == Trigger.DELTA_0) ? DELTA_0 : DELTA_1;
            }
            else
            {
                HEAD = DroneRC;
                TARGET = DELTA_0;
            }

            DebugLog += $"Target: {TARGET}\n";
        }
        void GenerateGyroVectors(bool kill = false)
        {
            if (kill)
            {
                GyroVectors[0] = 0;
                GyroVectors[1] = 0;
                GyroVectors[2] = 0;
                return;
            }

            IMyTerminalBlock head = (bDocking) ? (IMyTerminalBlock)DronePort : DroneRC;

            ////////////////

            Vector3 lookVector;


            //if (bDocking)
            lookVector = Look;
            //else
            //   lookVector = DroneRC.GetShipVelocities().LinearVelocity;


            Vector3 normalLook = TransformVectorRelative(head.WorldMatrix, lookVector);

            double targetYaw = Math.Atan2(normalLook.X, -normalLook.Z);
            double targetPitch = Math.Atan2(normalLook.Y, -normalLook.Z);
            //double targetRoll

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

            GyroVectors[0] = -targetPitch;
            GyroVectors[1] = targetYaw;
            GyroVectors[2] = 0; // Sans rrrroller pas exactement ci vous plais

        }
        void ApplyGyros(bool kill = false)
        {
            GenerateGyroVectors(kill);

            for (int i = 0; i < 3; i++)
            {
                foreach (GyroMask gyro in Gyros)
                {
                    ApplyGyroAction(gyro, i, GyroVectors[i]);
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
            GenerateThrustVectors(HEAD, TARGET, kill);

            for (int i = 0; i < 6; i++)
            {
                foreach (IMyThrust thrust in ThrustGroups[i])
                {
                    thrust.ThrustOverride = ThrustVectors[i];
                }
            }
        }

        void DockingUpdate()
        {
            if (!bDocking || !bDockingInitialized)
                return;

            if (!bDockingProxy)
                bDockingProxy = ProxyCheckDocking();

            switch (DockingSequence.Operations[DockingSequence.CurrentIndex].Trigger)
            {
                case Trigger.DELTA_0:

                    if (!ProxyCheckLiteral())
                        break;

                    if (DockingSequence.CurrentIndex == 0)
                    {
                        DronePort.Enabled = true;
                        DockingSequence.CurrentIndex++;
                    }

                    if (DockingSequence.CurrentIndex == 3)
                    {
                        Flight = OldFlight;
                        Mode = OldMode;
                        DronePort.Enabled = false;
                        bDocking = false;
                        bDockingInitialized = false;
                        bDockingProxy = false;
                        DockingSequence.CurrentIndex = 0;
                        Request(DroneRequest.RELEASE);
                        Request(DroneRequest.FORM);
                    }
                    break;

                case Trigger.DELTA_1:

                    if (DronePort.Status == MyShipConnectorStatus.Connectable)
                    {
                        DronePort.Connect();

                        //SwitchModes(FlightMode.RESUPPLY);
                        DockingSequence.CurrentIndex++;
                    }

                    break;

                case Trigger.EVENT:

                    EventClock++;
                    DebugLog += $"EventClock: {EventClock}\n";

                    if (EventClock >= EVENT_TIME)
                    {
                        DronePort.Disconnect();
                        // SwitchModes(DroneMode.MIMIC);
                        EventClock = 0;
                        DockingSequence.CurrentIndex++;
                    }
                    break;
            }

            DebugLog += $"DockingSequenceIndex: {DockingSequence.CurrentIndex}\n";
        }
        void NavMode()
        {
            if (Flight != DroneFlight.NAV ||
                !ProxyCheckNav())
                return;

            if (bTargetGood)
                Move();
            else
                Request(DroneRequest.FORM);
        }
        void MimicMode()
        {
            if (Flight != DroneFlight.MIMIC)
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
            //bDockingInitialized = bDocking;
        }
        bool ProxyCheckDocking()
        {
            return Vector3.Distance(HEAD.GetPosition(), DELTA_0) < DOCK_PROXY;
        }
        bool ProxyCheckNav()
        {
            return !DroneRC.IsAutoPilotEnabled;
        }
        bool ProxyCheckLiteral()
        {
            return Vector3.Distance(TARGET, HEAD.GetPosition()) < CHECK_PROXY &&
                Vector3.Distance(DroneRC.GetShipVelocities().LinearVelocity, HUB_DRIFT) < DRIFT_PROXY &&
                Math.Abs(GyroVectors[0] + GyroVectors[1] + GyroVectors[2]) < GYRO_PROXY;
        }

        void ToggleGyros(bool toggle = false)
        {
            foreach (GyroMask gyro in Gyros)
            {
                if (!toggle)
                {
                    gyro.Gyro.Yaw = 0;
                    gyro.Gyro.Pitch = 0;
                    gyro.Gyro.Roll = 0;
                }

                gyro.Gyro.GyroOverride = toggle;
            }
        }
        void SwitchFlight(DroneFlight flight)
        {
            Flight = flight;

            if (Flight == DroneFlight.MIMIC)
                ToggleGyros(true);

            if (Flight != DroneFlight.MIMIC)
            {
                ApplyThrust(true);
                ToggleGyros();
            }

            if (Flight != DroneFlight.NAV)
                DroneRC.SetAutoPilotEnabled(false);
        }

        void SwitchMode(DroneMode mode)
        {
            Mode = mode;

            // Do Stuff
        }

        /* Perriferals
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
        void AdjustSpeed(bool faster)
        {
            MaxDrift = MaxDrift < MAX_DRIFT_MAX && faster ? MaxDrift + 10 : MaxDrift;
            MaxDrift = MaxDrift > MAX_DRIFT_MIN && !faster ? MaxDrift - 10 : MaxDrift;
        }
        void AdjustShield(bool bigger)
        {
            if (!bShieldAvailable)
                return;

            try
            {
                float newVal = DroneShield.GetValueFloat("DS-CFit");
                newVal = newVal < 22 && bigger ? newVal + 1 : newVal;
                newVal = newVal > 1 && !bigger ? newVal - 1 : newVal;
                DroneShield.SetValueFloat("DS-CFit", newVal);
            }
            catch
            {
                return;
            }
        }
        */

        void InitiateDockingSequence()
        {
            OldFlight = Flight;
            OldMode = Mode;
            bDocking = true;
            bDockingInitialized = false;
            bTargetGood = false;
            DronePort.Enabled = false;
            DockingSequence.CurrentIndex = 0;
            SwitchFlight(DroneFlight.MIMIC);
            Request(DroneRequest.DOCK);
        }

        // Old Entry Points
        void LegacyDroneBuilder()
        {
            DroneShield = GridTerminalSystem.GetBlockWithName(ShieldControlName);
            bShieldAvailable = DroneShield != null;
            string shield = bShieldAvailable ? "ShieldCalibrated" : "ShieldMissing";
            Echo(shield);


            IMyBlockGroup group = GridTerminalSystem.GetBlockGroupWithName(SensorName);
            if (group != null)
            {
                DroneSensors = new List<IMySensorBlock>();
                List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
                group.GetBlocks(blocks);
                foreach (IMyTerminalBlock block in blocks)
                {
                    if (block is IMySensorBlock)
                    {
                        SetupSensor((IMySensorBlock)block);
                        DroneSensors.Add((IMySensorBlock)block);
                    }
                }
            }
            DroneSensor = (IMySensorBlock)GridTerminalSystem.GetBlockWithName(SensorName);
            SetupSensor(DroneSensor);
            string sensor = DroneSensor != null || DroneSensors != null || DroneSensors.Count > 0 ? "SensorCalibrated" : "SensorMissing";
            Echo(sensor);

            try
            {
                DronePort = (IMyShipConnector)GridTerminalSystem.GetBlockWithName(PortName);
                DronePort.Enabled = false;

                DroneSurface = Me.GetSurface(0);
                DroneSurface.ContentType = ContentType.TEXT_AND_IMAGE;
                DroneSurface.WriteText("");

                Panel = (IMyTextPanel)GridTerminalSystem.GetBlockWithName(PanelName);
                Panel.ContentType = ContentType.TEXT_AND_IMAGE;
                Panel.WriteText("");

                Panel.FontSize = 100;
                Panel.Alignment = TextAlignment.CENTER;

                DebugLog = string.Empty;

                Me.CustomName = "DRONE_PB";
                RawMessage[0] = Me.EntityId;
                Me.CubeGrid.CustomName = $"DRONE_{Me.EntityId}";
                InChannel = $"DRONE_{Me.EntityId}";
                SwarmEar = IGC.RegisterBroadcastListener(SwarmChannel);
                DroneEar = IGC.RegisterBroadcastListener(InChannel);
                DroneEar.SetMessageCallback(InChannel);

                DockingSequenceBuilder();
                SetupRemoteControl();
                PopulateThrusters();
                PopulateGyros();

                SwitchFlight(DroneFlight.STANDBY);

                Runtime.UpdateFrequency = UpdateFrequency.Update10;
                bConfigured = true;
            }
            catch
            {
                bConfigured = false;
            }
        }
        /*
        public void Main(string argument, UpdateType updateSource)
        {
            DroneSurface.WriteText(DebugLog);
            DebugLog = string.Empty; // DO NOT REMOVE THIS!!!!
            //DroneSurface.WriteText($"MaxDrift: {MaxDrift}");

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
                NavMode();
        }
        */

        #endregion

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Net Section

        // Constants
        const string ControlChannel = "SKY_NET1";
        const string RecruitChannel = "NEED_WORK";
        //const string StringChannel = "SKY_NETchar[]";
        const float MAX_BCR = 50000;
        const int CLUST_NAME_CHAR_CAP = 10;
        const byte MAX_MESSAGE_PROCESS_COUNT = 100;
        const byte MSG_TIMEOUT = 100;
        const byte MAX_STR_LNGTH = 10;

        // Channels
        IMyBroadcastListener ControlEar;
        IMyBroadcastListener RecruitEar;
        IMyBroadcastListener ClusterEar;
        IMyBroadcastListener IndexEar;
        IMyBroadcastListener IdEar;

        // Internals

        // Mechanical
        IMyTextPanel Panel;
        bool bActive;
        bool bClustered;
        bool bJoining;
        float MIN_BCR;
        float ServiceId;

        // Cluster
        byte ClusterMsgCount = 0;
        byte ClusterIndex = 0;
        byte CurrentClusterCount = 1;
        //string ClusterChannel;
        //string IndexChannel;
        //string ServiceChannel;

        // Clocks
        byte JoinClock;

        // Package
        float ClusterId;
        MatrixD Matrix;
        Vector3 Center;
        float Radius;
        float TemporalOffset;
        float PositionalOffset;

        //////////////

        // Legacy Elements
        public enum Trigger
        {
            DELTA_0,
            DELTA_1,
            EVENT
        }
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
            IDLE,
            RESUPPLY,
            LEADING,
            CLUSTERED,
            JOINING,
        }
        public enum DroneFlight
        {
            STANDBY,
            NAV,
            MIMIC,
        }

        // Op-Codes
        enum ControlCode
        {
            OFF,
            ON,
            PING,
            REG_SVC,
            CLUSTER,
            DE_CLUSTER,
        }
        enum IDcode
        {
            LEAD,
            REQ_DEBUG,
            LINK_SERVICE,
            RCV_PACKAGE
        }
        enum ClusterCode
        {
            //RCV_TIME
        }
        enum IndexCode
        {
            REQ_PCK,
            REC_PCK
        }

        // Helpers
        bool UnzipLong(ref ImmutableArray<byte> data, ref long source)
        {
            if (data.Length != 11)
                return false;

            source = 0;

            for (int i = 10; i > 0; i--)
            {
                source += data.ItemRef(i);
                if (i == 0)
                    break;
                source *= 100;
            }
            return true;
        }
        bool UnzipString(ref ImmutableArray<byte> data, ref string source)
        {
            if (data.Length > MAX_STR_LNGTH + 1)
                return false;

            source = "";

            for (int i = 1; i < data.Length; i++)
            {
                source += data.ItemRef(i);
            }
            return true;
        }
        ImmutableArray<byte> ZipLong(ref byte[] byteBuffer, long data, byte opCode)
        {
            byteBuffer[0] = opCode;

            for (int i = 1; i < 11; i++)
            {
                byteBuffer[i] = (byte)(data % 100);
                data /= 100;
            }

            return ImmutableArray.Create(byteBuffer);
        }
        ImmutableArray<byte> ZipString(ref byte[] byteBuffer, string data, byte opCode)
        {
            byteBuffer[0] = opCode;

            for (int i = 1; i < MAX_STR_LNGTH + 1; i++)
            {
                byteBuffer[i] = (byte)data[i];
            }

            return ImmutableArray.Create(byteBuffer);
        }

        // CaptainMethods
        void CreateCluster(ref ImmutableArray<float> package)
        {
            ImprintPackage(ref package);
            PromoteCaptain();
        }
        void PromoteCaptain()
        {
            SetIndex(0, ref ClusterId);
            RecruitEar = IGC.RegisterBroadcastListener(RecruitChannel);
            IndexEar = IGC.RegisterBroadcastListener($"{ClusterId}:{ClusterIndex}");
            ClusterEar = IGC.RegisterBroadcastListener($"CLUSTER_{ClusterId}");
        }
        void SetIndex(byte index, ref float clusterId)
        {
            ClusterIndex = index;
            ClusterId = clusterId;
            bClustered = !(ClusterId == 0);

            if (!bClustered)
                IGC.DisableBroadcastListener(IndexEar);

            else
                IndexEar = IGC.RegisterBroadcastListener($"{ClusterId}_{ClusterIndex}");
 
        }
        void ImprintPackage(ref ImmutableArray<float> package)
        {
            ClusterId = package.ItemRef(0);
            //ClusterChannel = $"CLUSTER_{ClusterId}";

            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    Matrix[j, i] = package.ItemRef(i + (j * 3) + 1);

            Center.X = package.ItemRef(10);
            Center.Y = package.ItemRef(11);
            Center.Z = package.ItemRef(12);

            /*
            Matrix.M11 = package.ItemRef(1);
            Matrix.M12 = package.ItemRef(2);
            Matrix.M13 = package.ItemRef(3);
            Matrix.M21 = package.ItemRef(4);
            Matrix.M22 = package.ItemRef(5);
            Matrix.M23 = package.ItemRef(6);
            Matrix.M31 = package.ItemRef(7);
            Matrix.M32 = package.ItemRef(8);
            Matrix.M33 = package.ItemRef(9);
            */


            bClustered = true;
        }
        void AttemptRecruit(ref ImmutableArray<byte> data)
        {
            long id = 0;
            if (!UnzipLong(ref data, ref id))
            {
                // failure! >:-(
                return;
            }

            
        }

        // DroneMethods
        void LinkService(ref ImmutableArray<float> data)
        {
            ServiceId = data.ItemRef(1);
        }
        void SendDebug()
        {
            if (ServiceId == 0)
                return;

            IGC.SendBroadcastMessage($"SERVICE_{ServiceId}", DebugLog);
        }
        void JoinCluster()
        {
            IGC.SendBroadcastMessage(RecruitChannel, Me.EntityId);
            //MsgSendIndex(IndexCode.REQ_JOIN, $"{RecievedStringBuffer}:{0}");
            Mode = DroneMode.JOINING;
        }
        void LeaveCluster()
        {
            // Clean up all elements pertaining to cluster
            IGC.DisableBroadcastListener(ClusterEar);
            ClusterId = 0;
            SetIndex(0, ref ClusterId);

            // Resolve drone internals
            Mode = DroneMode.IDLE;
        }
        void UpdateClusterPosition(ref ImmutableArray<byte> data)
        {

        }

        // Callers
        void MsgSendControl(ControlCode code)
        {
            switch(code)
            {
                case ControlCode.PING:
                    ImmutableArray<long> data = new ImmutableArray<long> {(long)ControlCode.PING, Me.EntityId };
                    IGC.SendBroadcastMessage(ControlChannel, Me.EntityId);
                    break;
            }
        }
        void MsgSendCluster(ClusterCode code)
        {

        }
        void MsgSendIndex(IndexCode code, string indexChannel)
        {
            switch(code)
            {
                case IndexCode.REQ_PCK:
                    break;

                case IndexCode.REC_PCK:
                    break;
            }

        }
        void MsgSendID(long id)
        {

        }

        // Listeners
        void MsgRcvControl(MyIGCMessage message)
        {
            try
            {
                ControlCode code = (ControlCode)(int)message.Data;

                switch(code)
                {
                    case ControlCode.OFF:
                        bActive = false;
                        break;

                    case ControlCode.ON:
                        bActive = true;
                        break;

                    case ControlCode.PING:
                        MsgSendControl(code);
                        break;

                    case ControlCode.CLUSTER:
                        JoinCluster();
                        break;

                    case ControlCode.DE_CLUSTER:
                        LeaveCluster();
                        break;


                    default:
                        DebugLog += "Nothing happened sam \n";
                        break;
                }
            }
            catch
            {
                DebugLog += "Invalid code type! Was expecting Imu<string>[] \n";
            }
        }
        void MsgRcvCluster(MyIGCMessage message)
        {
            try
            {

            }
            catch
            {

            }
        }
        void MsgRcvIndex(MyIGCMessage message)
        {
            try
            {
                ImmutableArray<byte> data = (ImmutableArray<byte>)message.Data;

                /*  [0] = Code
                 *  [1] = Sender's Index
                 *  [2] = Drone Count
                 *  [3] = Time (fauxMinute)
                 *  [4] = Time (fauxSecond)
                 */


                switch((IndexCode)data.ItemRef(0))
                {
                    case IndexCode.REQ_PCK:

                        break;

                    case IndexCode.REC_PCK:
                        UpdateClusterPosition(ref data);
                        break;
                }

            }
            catch
            {
                DebugLog += "Invalid data type! Was expecting Imu<float>[] \n";
            }
        }
        void MsgRcvID(MyIGCMessage message)
        {
            try
            {
                ImmutableArray<float> data = (ImmutableArray<float>)message.Data;

                switch((IDcode)data.ItemRef(0))
                {
                    case IDcode.LEAD:
                        CreateCluster(ref data);
                        break;

                    case IDcode.LINK_SERVICE:
                        LinkService(ref data);
                        break;

                    case IDcode.REQ_DEBUG:
                        SendDebug();
                        break;
                }
            }
            catch
            {

            }
        }
        void MsgRcvRecruit(MyIGCMessage message)
        {
            try
            {
                // Always expects a long
            }
            catch
            {

            }
        }
        /*
        void MsgRcvString(MyIGCMessage message)
        {
            try
            {
                RecievedStringBuffer = (string)message.Data;
            }
            catch
            {
                DebugLog += "Invalid message data! Was expecting a string \n";
            }
        }
        */

        // Time-outs (return false till time-out. Internal clocks will self-resolve)
        bool JoinTimeOut()
        {
            JoinClock++;
            JoinClock = JoinClock > MSG_TIMEOUT ? (byte)0 : JoinClock;
            return JoinClock == 0;
        }

        // Behaviours
        void PersonalBehaviourUpdate()
        {
            switch(Mode)
            {
                case DroneMode.JOINING:
                    Panel.WriteText("J");
                    Panel.FontColor = Color.Red;
                    if (JoinTimeOut())
                        JoinCluster();
                    break;

                case DroneMode.IDLE:
                    Panel.WriteText("I");
                    Panel.FontColor = Color.Yellow;
                    break;

                case DroneMode.LEADING:
                    Panel.WriteText("L");
                    Panel.FontColor = Color.Blue;
                    break;

                case DroneMode.CLUSTERED:
                    Panel.WriteText("C");
                    Panel.FontColor = Color.Green;
                    break;
            }
        }
        void Independant()
        {

        }
        void Lost()
        {

        }
        void Leader()
        {

        }
        void Drone()
        {

        }

        // Setup
        void EssentialDroneConfigurations()
        {
            // Permanent
            IdEar = IGC.RegisterBroadcastListener(Me.EntityId.ToString());
            ControlEar = IGC.RegisterBroadcastListener(ControlChannel);

            // Post-Load
        }

        // EntryPoints
        public Program()
        {
            LegacyDroneBuilder();
            Load();
            EssentialDroneConfigurations();
        }

        public void Main(string argument, UpdateType updateSource)
        {
            switch (argument)
            {

            }

            DroneSurface.WriteText(DebugLog);

            while (ControlEar.HasPendingMessage)
            {
                MsgRcvControl(ControlEar.AcceptMessage());
            }

            /*while (StringEar.HasPendingMessage)
            {
                MsgRcvString(StringEar.AcceptMessage());
            }*/

            while (IdEar.HasPendingMessage)
            {
                MsgRcvID(IdEar.AcceptMessage());
            }

            while (IndexEar.HasPendingMessage)
            {
                MsgRcvIndex(IndexEar.AcceptMessage());
            }

            DebugLog = ""; // DO NOT REMOVE!!!

            if (!bActive)
            {
                return;
            }

            int clstMsgCount = 0;
            while (ClusterEar.HasPendingMessage &&
                clstMsgCount < MAX_MESSAGE_PROCESS_COUNT)
            {
                MsgRcvCluster(ClusterEar.AcceptMessage());
                clstMsgCount++;
            }

            PersonalBehaviourUpdate();

            if (ClusterIndex < 0)
            {
                return;
            }
        }
        public void Save()
        {

        }

        void Load()
        {

        }

        #endregion
    }
}
