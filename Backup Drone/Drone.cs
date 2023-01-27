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

        const string RootName = "TEST_DRONE";
        const string LostChannel = "NEED_HUB";
        const string PortName = "PORT";
        const string ShieldControlName = "[A] Shield Control";

        #region AUTO_MINE
        const float MINE_DISTANCE = 20f; // 1000f
        const float MINE_LERP_RATE = .1f; // 0.0005f
        const float MINE_ROTATION = 1f;

        Vector3 MiningOrigin;
        Vector3 MiningEnd;
        float MiningProgress;
        #endregion

        static long lowEnd = (long)Math.Pow(2, 16) - 1;
        static long highEnd = (long)Math.Pow(2, 64) - 1 - lowEnd;

        const byte MIN_MAX_DRIFT = 10;
        const byte MAX_DRIFT = byte.MaxValue;
        const byte DEFAULT_DRIFT = 100;

        const float DOCK_PROXY = 15f;
        const float CHECK_PROXY = 1f;
        const float DRIFT_PROXY = 0.1f;
        const float SENSOR_RANGE = 50f;
        const float THRUST_SCALE = 100000f;
        const float COLL_THRUST_MAX = 200f;
        const float COLL_THRUST_MIN = 20f;
        const float GYRO_SCALE = 0.01f;

        const double GYRO_PROXY = 1.6;
        const double RAD2DEG = 180 / Math.PI;

        const int EVENT_TIME = 20;
        static TimeSpan EXP_TIME = TimeSpan.FromSeconds(5);

        int EventClock = 0;
        DateTime RecieveTime;
        StringBuilder DebugLog;

        bool DEBUG_BREAK = false;
        bool bConfigured = false;
        bool bCockpitAvailable = false;
        bool bRCavailable = false;

        bool bTargetGood = false;
        bool bDockingProxy = false;

        DroneSettings CurrentSettings;
        DroneSettings PreviousSettings;

        WorldFrame Status;
        WorldFrame Goal;

        FlightMode flightMode;
        RequestType Request;

        // Network
        IMyBroadcastListener DroneEar;
        IMyBroadcastListener SwarmEar;

        string HubChannel;
        string SwarmChannel;
        string InChannel;

        byte[] MSG_BYTE_BUFFER = new byte[5];
        float[] MSG_IX_BUFFER = new float[Enum.GetValues(typeof(MsgIx)).Length];
        string[] MSG_STR_BUFFER = new string[3];

        // Hardware
        IMyTextSurface DroneSurface;
        IMyTextSurface CockpitScreen;
        IMyRemoteControl DroneRC;
        IMyShipController DroneControl;
        IMyTerminalBlock PrimaryHead;
        IMyShipConnector DronePort;
        IMyTerminalBlock DroneShield;
        IMyShipDrill ForwardDrill;

        List<IMyCargoContainer> DroneCargos;
        List<IMySensorBlock> DroneSensors;
        List<IMyShipDrill> DroneDrills;
        //List<IMyTur>

        IMyTerminalBlock HEAD;

        double CollisionThrust = 100f;
        List<IMyGyro> Gyros = new List<IMyGyro>();
        List<IMyThrust>[] ThrustGroups;
        float[] ThrustRatios = new float[6];
        float[] ThrustVectors = new float[6];
        Vector3D GyroVectors;

        Sequence CurrentSequence;
        Sequence DockingSequence;

        public enum Trigger
        {
            ALIGN,
            DOCK,
            RESUPPLY
        }
        public enum DroneTask
        {
            IDLE,
            DOCK,
            LOBBY,
            FORM,
            HAUL,
            MINE
        }
        public enum RequestType
        {
            REGISTER,
            IDLE,
            BUSY,
            DOCK,
            UN_DOCK,
            TASK
        }
        public enum FlightMode
        {
            STANDBY,
            NAV,
            TAI,
            RESUPPLY,
            SKYNET
        }
        public enum MsgIx
        {
            ID_0 = 0,
            ID_1 = 1,
            ID_2 = 2,
            ID_3 = 3,
            M11 = 4,
            M12 = 5,
            M13 = 6,
            M14 = 7,
            M21 = 8,
            M22 = 9,
            M23 = 10,
            M24 = 11,
            M31 = 12,
            M32 = 13,
            M33 = 14,
            M34 = 15,
            POS_X = 16,
            POS_Y = 17,
            POS_Z = 18,
            M44 = 19,
            //POS_X = 20,
            //POS_Y = 21,
            //POS_Z = 22,
            VEL_X = 20,
            VEL_Y = 21,
            VEL_Z = 22,
            REQ = 23,
            R_IND = 24,
            FUEL = 25,
            POWER = 26,
            INTEG = 27,
            AMMO = 28
        }
        public enum SwarmRequest
        {
            TASK = 0,
            INDEX = 1,
            SPEED = 2,
            SHIELD_SIZE = 3,
            OPTIONS = 4,
        }
        public enum Options
        {
            SHIELD_ON = 0,
            FLIGHT_TRUE = 1,
            SPIN_ON = 2,
            SHUNT_ON = 3
        }

        public struct DroneSettings
        {
            public DroneTask Task;

            public byte TaskIndex;
            public byte Speed;
            public byte ShieldSize;

            public bool[] Options;

            public DroneSettings(DroneTask task, byte index = 0, byte speed = DEFAULT_DRIFT, byte shieldSize = 0, bool[] options = null)
            {
                Task = task;
                TaskIndex = index;
                Speed = speed;
                ShieldSize = shieldSize;
                Options = options == null || options.Length != 8 ? new bool[8] : options;
            }
            void GetByte(out byte bite, bool[] bits)
            {
                int result = 0;

                for (int i = 0; i < 8; i++)
                {
                    result += bits[i] ? 1 : 0;
                    result = i < 7 ? result << 1 : result;
                }
                bite = (byte)result;
            }

            void GetBools(byte bite, bool[] bits)
            {
                uint buffer = bite;

                for (int i = 0; i < 8; i++)
                {
                    bits[i] = ((buffer & 128) == 128);
                    buffer = buffer << 1;
                }
            }

            public void ReadFromStream(byte[] stream)
            {
                // Refresh? 0
                Task = (DroneTask)stream[(int)SwarmRequest.TASK];
                Speed = stream[(int)SwarmRequest.SPEED];
                ShieldSize = stream[(int)SwarmRequest.SHIELD_SIZE];

                GetBools(stream[(int)SwarmRequest.OPTIONS], Options);

                //ShieldON = stream[(int)SwarmRequest.SHIELD_ON];
                //FlightTRUE = stream[(int)SwarmRequest.FLIGHT_TRUE];
                //SpinON = stream[(int)SwarmRequest.SPIN_ON];
                //ShuntON = stream[(int)SwarmRequest.SHUNT_ON];
            }
            public void WriteToStream(byte[] stream)
            {
                // Refresh? 0
                stream[(int)SwarmRequest.TASK] = (byte)Task;
                stream[(int)SwarmRequest.SPEED] = Speed;
                stream[(int)SwarmRequest.SHIELD_SIZE] = ShieldSize;

                GetByte(out stream[(int)SwarmRequest.OPTIONS], Options);
            }
        }
        public struct WorldFrame
        {
            public static WorldFrame Zero = new WorldFrame(MatrixD.Zero, Vector3.Zero, Vector3.Zero);
            public DateTime TimeStamp;
            public Vector3 Velocity;
            public MatrixD Matrix;

            public WorldFrame(MatrixD mat, Vector3 pos, Vector3 vel)
            {
                TimeStamp = DateTime.Now;
                Velocity = vel;
                Matrix = mat;
            }
            public WorldFrame(IMyTerminalBlock block)
            {
                TimeStamp = DateTime.Now;
                Matrix = block.WorldMatrix;
                Velocity = Vector3.Zero;
            }
            public void Update(IMyTerminalBlock block)
            {
                if (block == null)
                    return;
                Vector3 oldPosition = Matrix.Translation;
                Matrix = block.WorldMatrix;
                Velocity = Matrix.Translation - oldPosition;
                TimeStamp = DateTime.Now;
            }
            public void ReadFromStream(float[] stream)
            {
                ExtractMatrixD(ref stream, ref Matrix, (int)MsgIx.M11);
                ExtractVector3(ref stream, ref Velocity, (int)MsgIx.VEL_X);
                TimeStamp = DateTime.Now;
            }
            public void WriteToStream(float[] stream)
            {
                InsertMatrixD(ref stream, ref Matrix, (int)MsgIx.M11);
                InsertVector3(ref stream, ref Velocity, (int)MsgIx.VEL_X);
            }
            public bool Expired()
            {
                return DateTime.Now - TimeStamp > EXP_TIME;
            }
        }
        public struct Operation
        {
            public Trigger Trigger;
            public FlightMode Mode;

            public Operation(Trigger trigger, FlightMode mode)
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

        void DockingSequenceBuilder()
        {
            Operation[] operations = new Operation[3];

            operations[0] = new Operation(Trigger.ALIGN, FlightMode.NAV);      // Head towards docking port
            operations[1] = new Operation(Trigger.DOCK, FlightMode.TAI);    // Approach docking port and connect
            operations[2] = new Operation(Trigger.RESUPPLY, FlightMode.RESUPPLY);   // Resupply and disconnect

            DockingSequence = new Sequence(operations);

            //operations[3] = new Operation(Trigger.ALIGN, DroneRequest.UN_DOCK, FlightMode.TAI);    // Clear away from docking port
        }
        void SetupShipControl()
        {
            List<IMyShipController> controllers = new List<IMyShipController>();
            GridTerminalSystem.GetBlocksOfType(controllers);

            List<IMyRemoteControl> remotes = new List<IMyRemoteControl>();
            GridTerminalSystem.GetBlocksOfType(remotes);

            if (remotes.Count > 0)
            {
                DroneRC = remotes[0];
                bRCavailable = true;
            }
            if (controllers.Count > 0)
            {
                DroneControl = controllers[0];
                bCockpitAvailable = true;
            }
            else if (bRCavailable)
            {
                DroneControl = DroneRC;
            }

            if (DroneControl != null)
            {
                PrimaryHead = DroneControl;
                if (DroneControl is IMyCockpit)
                {
                    CockpitScreen = ((IMyCockpit)DroneControl).GetSurface(0);
                    CockpitScreen.ContentType = ContentType.TEXT_AND_IMAGE;
                    CockpitScreen.WriteText("");
                }
            }
            else
                PrimaryHead = Me;

            SetupRemoteControl(DroneRC);
        }
        void SetupRemoteControl(IMyRemoteControl droneRC)
        {
            if (droneRC == null)
                return;

            droneRC = (IMyRemoteControl)DroneControl;
            droneRC.FlightMode = Sandbox.ModAPI.Ingame.FlightMode.OneWay;
            droneRC.SetDockingMode(true);
            droneRC.SetCollisionAvoidance(true);
            droneRC.SetAutoPilotEnabled(false);
            droneRC.ClearWaypoints();
        }
        void SetupDrills(List<IMyShipDrill> drills)
        {
            Dictionary<Base6Directions.Direction, int> orientations = new Dictionary<Base6Directions.Direction, int>();

            foreach (IMyShipDrill drill in drills)
            {
                drill.Enabled = true;
                Base6Directions.Direction key = drill.Orientation.Forward;

                int value;
                if (orientations.TryGetValue(key, out value))
                    orientations[key]++;
                else
                    orientations.Add(key, 1);
            }

            KeyValuePair<Base6Directions.Direction, int> highest = new KeyValuePair<Base6Directions.Direction, int>();
            foreach (var pair in orientations)
            {
                if (pair.Value > highest.Value)
                    highest = pair;
            }
            foreach (IMyShipDrill drill in DroneDrills)
                if (drill.Orientation.Forward == highest.Key)
                {
                    ForwardDrill = drill;
                    return;
                }
        }
        void SetupSensors(List<IMySensorBlock> sensors)
        {
            foreach (IMySensorBlock sensor in sensors)
            {
                if (sensor == null)
                    return;

                sensor.Enabled = true;

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

                sensor.BackExtend = SENSOR_RANGE;
                sensor.BottomExtend = SENSOR_RANGE;
                sensor.FrontExtend = SENSOR_RANGE;
                sensor.LeftExtend = SENSOR_RANGE;
                sensor.RightExtend = SENSOR_RANGE;
                sensor.TopExtend = SENSOR_RANGE;
            }
        }
        void PopulateGyros()
        {
            List<IMyGyro> gyros = new List<IMyGyro>();
            GridTerminalSystem.GetBlocksOfType(gyros);
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

            SetMaxDrift(DroneControl, thrustTotals[weakestIndex]);

            for (int i = 0; i < 6; i++)
            {
                ThrustRatios[i] = thrustTotals[weakestIndex] / thrustTotals[i];
            }

            Echo($"thrusters populated: {thrusters.Count}");
        }
        void SetMaxDrift(IMyShipController control, float fixedThrust)
        {
            if (control == null)
            {
                CurrentSettings.Speed = DEFAULT_DRIFT;
                return;
            }
            float criticalAccel = fixedThrust / control.CalculateShipMass().TotalMass;
            float stopDistance = SENSOR_RANGE - (float)control.WorldAABB.HalfExtents.Z; // Subtract half the 'length' of the ship from the sensor radius
            CurrentSettings.Speed = (byte)(Math.Sqrt((2 * stopDistance) / criticalAccel) * criticalAccel);
        }

        void HubSend(RequestType request)
        {
            DebugLog.Append("Requesting...\n");

            Status.WriteToStream(MSG_IX_BUFFER); DebugLog.Append("Status Buffered!\n");

            GenerateMyIDpacket(ref MSG_IX_BUFFER, Me.EntityId, (int)MsgIx.ID_0);
            MSG_IX_BUFFER[(int)MsgIx.REQ] = (float)request; DebugLog.Append("Request Buffered!\n");
            if (CurrentSequence != null)
            {
                MSG_IX_BUFFER[(int)MsgIx.R_IND] = CurrentSequence.CurrentIndex; DebugLog.Append("Sequence Index Buffered!\n");
            }

            IGC.SendBroadcastMessage(HubChannel, ImmutableArray.Create(MSG_IX_BUFFER));
            DebugLog.Append("Data shipped!\n");
        }
        void HubRecieve(MyIGCMessage msg)
        {
            DebugLog.Append("Recieved Hub message...\n");

            if (!ProcessMsgString(msg, ref MSG_STR_BUFFER, DebugLog))
            {
                DebugLog.Append("bad message!\n");
                //DEBUG_BREAK = true;
                return;
            }
            DebugLog.Append("msg good!\n");

            RequestType request;
            try
            {
                request = (RequestType)Enum.Parse(typeof(RequestType), MSG_STR_BUFFER[0]);
            }
            catch
            {
                DebugLog.Append("bad request!\n");
                //DEBUG_BREAK = true;
                return;
            }

            switch (request)
            {
                case RequestType.REGISTER:
                    RegisterHub();
                    break;
            }
        }
        void SwarmReceive(MyIGCMessage msg)
        {
            RecieveTime = DateTime.Now;
            DebugLog.Append("Recieved Swarm message...\n");

            if (!ProcessMsgByte(msg, ref MSG_BYTE_BUFFER))
            {
                DebugLog.Append("bad message!\n");
                //DEBUG_BREAK = true;
                return;
            }
            DebugLog.Append("msg good!\n");

            if (Request == RequestType.BUSY)
                PreviousSettings.ReadFromStream(MSG_BYTE_BUFFER);
            else
            {
                CurrentSettings.ReadFromStream(MSG_BYTE_BUFFER);
                SetTask(CurrentSettings.Task);
            }
            DebugLog.Append("stream read good!\n");
        }
        bool DroneRecieve(MyIGCMessage msg)
        {
            RecieveTime = DateTime.Now;

            if (msg.Data is ImmutableArray<float>)
            {
                return TaskRecieve(msg);
            }


            if (msg.Data is ImmutableArray<string>)
            {
                HubRecieve(msg);
                //DEBUG_BREAK = true;
                return true;
            }

            if (msg.Data is string) // Error message
            {
                DebugLog.Append(msg.Data);
                return false;
            }


            DebugLog.Append("Couldn't translate!\n");

            return false;
        }
        bool TaskRecieve(MyIGCMessage msg)
        {
            if (!ProcessMsgIx(msg, ref MSG_IX_BUFFER))
            {
                DebugLog.Append("MSG Processing failed!\n");
                return false;
            }
            Goal.ReadFromStream(MSG_IX_BUFFER);

            DebugLog.Append("MSG Processing success!\n");
            return true;
        }

        static bool GenerateMyIDpacket(ref float[] msgBuffer, long id, int index)
        {
            long buffer;
            try
            {
                for (int i = 0; i < 4; i++)
                {
                    buffer = id & lowEnd;
                    id = id & highEnd;
                    msgBuffer[index + i] = buffer;
                    id = id >> 16;
                }
                return true;
            }
            catch { return false; }
        }
        static bool ProcessMsgIx(MyIGCMessage msg, ref float[] buffer)
        {
            try
            {
                ImmutableArray<float> raw = ((ImmutableArray<float>)msg.Data);
                for (int i = 0; i < buffer.Length; i++)
                    buffer[i] = i < raw.Length ? raw[i] : 0;
                return true;
            }
            catch { return false; }
        }
        static bool ProcessMsgString(MyIGCMessage msg, ref string[] buffer, StringBuilder debug)
        {
            try
            {
                debug.Append($"rawStatus: {msg.Data is ImmutableArray<string>}\n");
                ImmutableArray<string> raw = ((ImmutableArray<string>)msg.Data);

                debug.Append($"Processing| raw.Length: {raw.Length} | buffer.Length: {buffer.Length}\n");
                for (int i = 0; i < buffer.Length; i++)
                {
                    buffer[i] = i < raw.Length ? raw[i] : "#";
                    debug.Append($"{i}:{buffer[i]}\n");
                }

                return true;
            }
            catch { return false; }
        }
        static bool ProcessMsgByte(MyIGCMessage msg, ref byte[] buffer)
        {
            try
            {
                ImmutableArray<byte> raw = ((ImmutableArray<byte>)msg.Data);
                for (int i = 0; i < buffer.Length; i++)
                    buffer[i] = i < raw.Length ? raw[i] : byte.MinValue;
                return true;
            }
            catch { return false; }
        }
        static void InsertVector3(ref float[] buffer, ref Vector3 input, int index)
        {
            for (int i = 0; i < 3; i++)
                buffer[index + i] = input.GetDim(i);
        }
        static void ExtractVector3(ref float[] dataStream, ref Vector3 output, int index)
        {
            for (int i = 0; i < 3; i++)
                output.SetDim(i, dataStream[index + i]);
        }
        static void InsertMatrixD(ref float[] buffer, ref MatrixD input, int index)
        {
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                    buffer[index + (r * 4) + c] = (float)input[r, c];
        }
        static void ExtractMatrixD(ref float[] dataStream, ref MatrixD output, int index)
        {
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                    output[r, c] = dataStream[index + (r * 4) + c];
        }

        #region FLIGHT CONTROL
        void GenerateThrustVectors(bool kill = false)
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

            DebugLog.Append("Thrust Vectoring...\n");

            float distance = Vector3.Distance(Goal.Matrix.Translation, HEAD.GetPosition());
            Vector3 targetDelta = Goal.Matrix.Translation - HEAD.GetPosition();
            Vector3 targetTrans = TransformVectorRelative(HEAD.CubeGrid.WorldMatrix, targetDelta);
            Vector3 hubDriftTrans = TransformVectorRelative(HEAD.CubeGrid.WorldMatrix, Goal.Velocity);
            targetTrans += hubDriftTrans;

            Vector3 driftTrans = TransformVectorRelative(HEAD.CubeGrid.WorldMatrix, Status.Velocity);
            Vector3 collAvoid = GenerateCollisionAvoidanceVector(HEAD, bDockingProxy);

            float collMagRaw = Vector3.Distance(collAvoid, Vector3.Zero);
            collAvoid = VelocityCapVector(collAvoid, CurrentSettings.Speed);

            Vector3 thrustVector = Vector3.Normalize(targetTrans);
            float combinedMag = (distance) / (1 + collMagRaw);
            thrustVector *= combinedMag;
            thrustVector = VelocityCapVector(thrustVector, CurrentSettings.Speed);
            thrustVector += collAvoid - driftTrans;
            thrustVector *= THRUST_SCALE;

            ThrustVectors[0] = thrustVector.Z * ThrustRatios[0];
            ThrustVectors[1] = -thrustVector.Z * ThrustRatios[1];
            ThrustVectors[2] = thrustVector.X * ThrustRatios[2];
            ThrustVectors[3] = -thrustVector.X * ThrustRatios[3];
            ThrustVectors[4] = -thrustVector.Y * ThrustRatios[4];
            ThrustVectors[5] = thrustVector.Y * ThrustRatios[5];

            DebugLog.Append("Thrust Vectoring Complete!\n");
        }
        void GenerateGyroVectors(bool kill = false)
        {
            if (kill)
            {
                GyroVectors = Vector3D.Zero;
                return;
            }

            MatrixD orient;

            if (CurrentSettings.Options[(int)Options.FLIGHT_TRUE])
                orient = MatrixD.CreateLookAt(
                    HEAD.GetPosition(),
                    HEAD.GetPosition() + Status.Velocity,
                    HEAD.WorldMatrix.Up);
            else
                orient = Goal.Matrix;

            MatrixD correct = GenerateCorrection(HEAD.WorldMatrix, ref orient);

            if (!MatrixD.GetEulerAnglesXYZ(ref correct, out GyroVectors))
            {
                GenerateGyroVectors(true);
                return;
            }

            GyroVectors *= RAD2DEG * GYRO_SCALE;
        }
        void ApplyThrust(bool kill = false)
        {
            for (int i = 0; i < 6; i++)
            {
                foreach (IMyThrust thrust in ThrustGroups[i])
                {
                    thrust.ThrustOverride = kill ? 0 : ThrustVectors[i];
                }
            }
        }
        void ApplyGyros()
        {
            foreach (IMyGyro gyro in Gyros)
                ApplyGyroVector(gyro, HEAD, GyroVectors);
        }
        void ToggleGyros(bool toggle = false)
        {
            foreach (IMyGyro gyro in Gyros)
            {
                if (!toggle)
                {
                    gyro.Yaw = 0;
                    gyro.Pitch = 0;
                    gyro.Roll = 0;
                }

                gyro.GyroOverride = toggle;
            }
        }

        Vector3 GenerateCollisionAvoidanceVector(IMyTerminalBlock head, bool kill = false)
        {
            Vector3 output = new Vector3();

            if (kill == true)
                return output;

            if (DroneSensors.Count == 0)
                return output;

            List<MyDetectedEntityInfo> nextSet = new List<MyDetectedEntityInfo>();
            List<MyDetectedEntityInfo> detectedEntities = new List<MyDetectedEntityInfo>();

            foreach (IMySensorBlock sensor in DroneSensors)
            {
                sensor.DetectedEntities(nextSet);

                foreach (MyDetectedEntityInfo info in detectedEntities)
                    nextSet.Remove(info);

                detectedEntities.AddRange(nextSet);
            }

            if (detectedEntities.Count == 0)
                return output;

            int count = 0;

            foreach (MyDetectedEntityInfo entity in detectedEntities)
            {
                DebugLog.Append($"detecting:{count}\n");
                count++;
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

                double magnitude = CollisionThrust / (distance * distance);
                Vector3 normalDelta = TransformVectorRelative(head.CubeGrid.WorldMatrix, delta) * magnitude;
                output += normalDelta;
            }

            return output;
        }
        Vector3D TransformVectorRelative(MatrixD S, Vector3 D) // S = sourceBearing, D = WorldVectorDelta
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
            if (mag == 0)
                return Vector3.Zero;
            Vector3 newVector = Vector3.Normalize(vector * 2);
            mag = mag > cap ? cap : mag;
            newVector *= (float)mag;
            return newVector;
        }
        static MatrixD GenerateCorrection(MatrixD current, ref MatrixD target)
        {
            return MatrixD.Multiply(MatrixD.Invert(target), current);

            //return MatrixD.GetEulerAnglesXYZ(ref current, out solution);
        }
        static void ApplyGyroVector(IMyGyro gyro, IMyTerminalBlock head, Vector3D correction)
        {
            OrientGyroVector(gyro, head.Orientation.Left, (float)correction.X);
            OrientGyroVector(gyro, head.Orientation.Up, (float)correction.Y);
            OrientGyroVector(gyro, head.Orientation.Forward, (float)correction.Z);
        }
        static void OrientGyroVector(IMyGyro gyro, Base6Directions.Direction dir, float cor)
        {
            if (dir == gyro.Orientation.Left)
                gyro.Pitch = cor;
            if (dir == Base6Directions.GetFlippedDirection(gyro.Orientation.Left))
                gyro.Pitch = -cor;

            if (dir == gyro.Orientation.Up)
                gyro.Yaw = cor;
            if (dir == Base6Directions.GetFlippedDirection(gyro.Orientation.Up))
                gyro.Yaw = -cor;

            if (dir == gyro.Orientation.Forward)
                gyro.Roll = cor;
            if (dir == Base6Directions.GetFlippedDirection(gyro.Orientation.Forward))
                gyro.Roll = -cor;
        }

        void NavMode()
        {
            if (!bRCavailable ||
                flightMode != FlightMode.NAV ||
                !ProxyCheckNav())
                return;

            if (bTargetGood && bRCavailable)
                Move();
            else
                HubSend(RequestType.BUSY);
        }
        void TorqkAI()
        {
            GenerateThrustVectors(!bTargetGood);
            GenerateGyroVectors(!bTargetGood);
            ApplyThrust();
            ApplyGyros();
        }
        void Idle()
        {
            //DroneSurface.WriteText("FAILED TO RECIEVE TARGET!", false);
        }
        void Move()
        {
            DebugLog.Append($"Moving... \n");
            DroneRC.ClearWaypoints();
            DroneRC.AddWaypoint(Goal.Matrix.Translation, "TARGET");
            DroneRC.SetAutoPilotEnabled(true);
            bTargetGood = false;
        }
        bool ProxyCheckDocking()
        {
            return Vector3.Distance(HEAD.GetPosition(), Goal.Matrix.Translation) < DOCK_PROXY;
        }
        bool ProxyCheckNav()
        {
            return !DroneRC.IsAutoPilotEnabled;
        }
        bool ProxyCheckLiteral()
        {
            return Vector3.Distance(Status.Matrix.Translation, Goal.Matrix.Translation) < CHECK_PROXY &&
                Vector3.Distance(Status.Velocity, Goal.Velocity) < DRIFT_PROXY &&
                Vector3D.Distance(Vector3D.Zero, GyroVectors) < GYRO_PROXY;
        }
        #endregion

        void SetTask(DroneTask task)
        {
            CurrentSettings.Task = task;
            HEAD = null;
            bool drillOn = false;
            bool portOn = false;

            DebugLog.Append("Setting Task...\n");

            switch (CurrentSettings.Task)
            {
                case DroneTask.DOCK:
                    if (DronePort == null)
                        break;
                    HEAD = DronePort;
                    flightMode = FlightMode.TAI;
                    Request = RequestType.DOCK;
                    portOn = true;
                    break;

                case DroneTask.FORM:
                    DebugLog.Append("FORM\n");
                    flightMode = FlightMode.TAI;
                    Request = RequestType.TASK;
                    break;

                case DroneTask.IDLE:
                    flightMode = FlightMode.STANDBY;
                    Request = RequestType.IDLE;
                    break;

                case DroneTask.LOBBY:
                    flightMode = FlightMode.NAV;
                    Request = RequestType.IDLE;
                    break;

                case DroneTask.HAUL:
                    if (DroneCargos.Count < 1)
                        break;
                    flightMode = FlightMode.NAV;
                    Request = RequestType.BUSY;
                    break;

                case DroneTask.MINE:
                    if (ForwardDrill == null)
                        break;
                    HEAD = ForwardDrill;
                    flightMode = FlightMode.TAI;
                    Request = RequestType.BUSY;
                    drillOn = true;
                    break;
            }

            if (HEAD == null)
                HEAD = PrimaryHead;

            if (DronePort != null)
                DronePort.Enabled = portOn;

            if (ForwardDrill != null)
                ForwardDrill.Enabled = drillOn;

            RefreshFlightHardware();
        }
        void RefreshFlightHardware()
        {
            if (flightMode == FlightMode.TAI)
                ToggleGyros(true);

            if (flightMode != FlightMode.TAI)
            {
                ApplyThrust(true);
                ToggleGyros();
            }
        }
        void InitiateDockingSequence()
        {
            if (DronePort == null)
                return;

            bTargetGood = false;
            DronePort.Enabled = false;
            //bResponsePending = false;

            PreviousSettings = CurrentSettings;

            CurrentSettings.Task = DroneTask.DOCK;

            LoadDockingOperation(0);
        }
        void LoadDockingOperation(int index = -1)
        {
            if (index == -1)
            {
                DockingSequence.CurrentIndex++;
                if (DockingSequence.CurrentIndex >= DockingSequence.Operations.Length)
                    return;
            }

            Operation op = DockingSequence.Operations[DockingSequence.CurrentIndex];
            flightMode = op.Mode;
            RefreshFlightHardware();
        }
        void DockingProcedure()
        {
            if (CurrentSettings.Task != DroneTask.DOCK || !bTargetGood)
                return;

            if (!bDockingProxy)
                bDockingProxy = ProxyCheckDocking();

            switch (DockingSequence.Operations[DockingSequence.CurrentIndex].Trigger)
            {
                case Trigger.ALIGN:
                    if (!ProxyCheckLiteral())
                        break;

                    if (DockingSequence.CurrentIndex == 0)
                    {
                        DronePort.Enabled = true;
                        LoadDockingOperation();
                    }

                    if (DockingSequence.CurrentIndex == 3)
                    {
                        CurrentSettings = PreviousSettings;
                        RefreshFlightHardware();

                        DronePort.Enabled = false;
                        bDockingProxy = false;

                        HubSend(RequestType.UN_DOCK);
                    }
                    break;

                case Trigger.DOCK:

                    if (DronePort.Status == MyShipConnectorStatus.Connectable)
                    {
                        DronePort.Connect();
                        LoadDockingOperation();
                    }

                    break;

                case Trigger.RESUPPLY:

                    EventClock++;
                    DebugLog.Append($"EventClock: {EventClock}\n");

                    if (EventClock >= EVENT_TIME)
                    {
                        DronePort.Disconnect();
                        EventClock = 0;
                        LoadDockingOperation();
                    }
                    break;
            }

            DebugLog.Append($"DockingSequenceIndex: {DockingSequence.CurrentIndex}\n");
        }
        void InitiateMining()
        {
            bTargetGood = true;

            PreviousSettings = CurrentSettings;

            SetTask(DroneTask.MINE);
            //RefreshFlightHardware();
            //CurrentSettings.Task = DroneTask.MINE;
            //CurrentSettings.Request = DroneRequest.BUSY;
            //CurrentSettings.FlightMode = FlightMode.TAI;


            MiningOrigin = ForwardDrill.GetPosition();
            MiningEnd = ForwardDrill.WorldMatrix.Forward * MINE_DISTANCE + MiningOrigin;
            Goal.Matrix.Translation = MiningOrigin;
            Goal.Matrix = MatrixD.CreateLookAt(MiningOrigin, MiningEnd, HEAD.WorldMatrix.Up);
        }
        void MiningLerp()
        {
            if (CurrentSettings.Task != DroneTask.MINE)
                return;

            if (ProxyCheckLiteral())
            {
                MiningProgress += MINE_LERP_RATE;
                Goal.Matrix.Translation = MiningOrigin + (MiningProgress * (Goal.Matrix.Forward));
            }
            if (MiningProgress == 1)
            {
                bTargetGood = false;

                CurrentSettings = PreviousSettings;
                RefreshFlightHardware();
            }
        }

        void RollBack()
        {
            CurrentSettings = PreviousSettings;
            RefreshFlightHardware();
        }
        void UnRegister()
        {
            RecieveTime = DateTime.Now;
            HubChannel = LostChannel;
            RegisterSwarm("");
            SetTask(DroneTask.IDLE);
            Request = RequestType.REGISTER;
            //DEBUG_BREAK = bConfigured;
        }
        void RegisterHub()
        {
            HubChannel = MSG_STR_BUFFER[1];
            DebugLog.Append("Hub Registered!\n");
            RegisterSwarm(MSG_STR_BUFFER[2]);
            Request = RequestType.IDLE;
        }
        void RegisterSwarm(string newSwarm)
        {
            SwarmChannel = newSwarm;
            Me.CustomName = $"{SwarmChannel}{RootName}";
            Me.CubeGrid.CustomName = $"{SwarmChannel}{RootName}";
            SwarmEar = IGC.RegisterBroadcastListener(SwarmChannel);
            SwarmEar.SetMessageCallback();
            DebugLog.Append("Swarm Registered!\n");
        }
        void SetupNetwork()
        {
            Me.CustomName = RootName;
            GenerateMyIDpacket(ref MSG_IX_BUFFER, Me.EntityId, (int)MsgIx.ID_0);
            Me.CubeGrid.CustomName = RootName;
            InChannel = $"DRONE_{Me.EntityId}";
            DroneEar = IGC.RegisterBroadcastListener(InChannel);
            UnRegister();
        }

        #region OPERATION
        void MessageHandling()
        {
            if (Request == RequestType.BUSY)
                return;

            try
            {
                DebugLog.Append("Msg Handler...\n");

                if (SwarmEar != null)
                    while (SwarmEar.HasPendingMessage)
                        SwarmReceive(SwarmEar.AcceptMessage());

                DebugLog.Append("Swarm Ear Emptied!\n");

                while (DroneEar.HasPendingMessage)
                    bTargetGood = DroneRecieve(DroneEar.AcceptMessage());

                HubSend(Request);

                if (DateTime.Now - RecieveTime > EXP_TIME)
                    UnRegister();
            }
            catch
            {
                DebugLog.Append("FAIL-POINT!\n");
                //bConfigured = false;
            }
        }
        public static string MatrixToString(MatrixD matrix, string digits)
        {
            return
                $"R:{matrix.Right.X.ToString(digits)}|{matrix.Right.Y.ToString(digits)}|{matrix.Right.Z.ToString(digits)}\n" +
                $"U:{matrix.Up.X.ToString(digits)}|{matrix.Up.Y.ToString(digits)}|{matrix.Up.Z.ToString(digits)}\n" +
                $"F:{matrix.Forward.X.ToString(digits)}|{matrix.Forward.Y.ToString(digits)}|{matrix.Forward.Z.ToString(digits)}\n" +
                $"T:{matrix.Translation.X.ToString(digits)}|{matrix.Translation.Y.ToString(digits)}|{matrix.Translation.Z.ToString(digits)}\n";
        }
        void Debugging()
        {
            DroneSurface.WriteText(DebugLog);

            if (CockpitScreen != null)
                CockpitScreen.WriteText(DebugLog);

            if (DEBUG_BREAK)
                return;

            DebugLog.Clear(); // DO NOT REMOVE THIS!!!!
            DebugLog.Append(
                $"MY_ID: {Me.EntityId}\n" +
                $"HEAD: {HEAD != null}\n" +
                $"DRIFT: {Goal.Velocity}\n" +
                $"TARGET: {MatrixToString(Goal.Matrix, "#.##")}" +
                $"LOOK: {Goal.Matrix.Forward}\n" +
                $"FLIGHT_MODE: {flightMode}\n" +
                $"HUB_CHANNEL: {HubChannel}\n" +
                $"CURRENT_POS: {Status.Matrix.Translation}\n" +
                $"CURRENT_TASK: {CurrentSettings.Task}\n" +
                $"CURRENT_REQUEST: {Request}\n" +
                $"TIME_OUT: {EXP_TIME - (DateTime.Now - RecieveTime)}\n" +
                $"===================\n");
        }
        void MyEcho()
        {
            Echo($"Shield Available: {DroneShield != null}\n" +
                $"Sensor Count: {DroneSensors.Count}\n" +
                $"Drill Count: {DroneDrills.Count}\n" +
                $"Configured: {bConfigured}");
        }
        void UserArgs(string argument)
        {
            switch (argument)
            {
                case "TEST":
                    InitiateMining();
                    break;

                case "DOCK":
                    InitiateDockingSequence();
                    break;

                case "CLEAR":
                    SetTask(DroneTask.IDLE);
                    break;
            }
        }
        void Run()
        {
            switch (CurrentSettings.Task)
            {
                case DroneTask.IDLE:
                    Idle();
                    break;

                case DroneTask.HAUL:
                    break;

                case DroneTask.FORM:
                    break;

                case DroneTask.LOBBY:
                    break;

                case DroneTask.DOCK:
                    DockingProcedure();
                    break;

                case DroneTask.MINE:
                    MiningLerp();
                    break;
            }

            switch (flightMode)
            {
                case FlightMode.STANDBY:
                    break;

                case FlightMode.NAV:
                    NavMode();
                    break;

                case FlightMode.TAI:
                    TorqkAI();
                    break;
            }
        }
        #endregion

        #region ENTRY-POINTS
        public Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Update10;
            DebugLog = new StringBuilder();

            DroneSurface = Me.GetSurface(0);
            DroneSurface.ContentType = ContentType.TEXT_AND_IMAGE;
            DroneSurface.WriteText("", false);

            try
            {
                DronePort = (IMyShipConnector)GridTerminalSystem.GetBlockWithName(PortName);

                if (DronePort != null)
                    DronePort.Enabled = false;

                DroneShield = GridTerminalSystem.GetBlockWithName(ShieldControlName);

                DroneSensors = new List<IMySensorBlock>();
                DroneDrills = new List<IMyShipDrill>();

                GridTerminalSystem.GetBlocksOfType(DroneSensors);
                GridTerminalSystem.GetBlocksOfType(DroneDrills);

                SetupSensors(DroneSensors);
                DebugLog.Append("Sensors Setup!\n");

                SetupDrills(DroneDrills);
                DebugLog.Append("Drills Setup!\n");

                DockingSequenceBuilder();
                DebugLog.Append("Docking Sequence Setup!\n");

                SetupShipControl();
                DebugLog.Append("RemoteControl Setup!\n");

                PopulateThrusters();
                DebugLog.Append("Thrusters Setup!\n");

                PopulateGyros();
                DebugLog.Append("Gyros Setup!\n");

                CurrentSettings = new DroneSettings(DroneTask.IDLE);
                SetupNetwork();
                DebugLog.Append("Network Setup!\n");

                //Load();

                DebugLog.Append("Load Complete!\n");
                bConfigured = true;
                DebugLog.Append("Setup Complete!\n");
            }
            catch
            {
                DebugLog.Append("FAIL-POINT!\n");
                bConfigured = false;
            }
            DroneSurface.WriteText(DebugLog);
            //DebugLog.Clear();
        }
        public void Main(string argument, UpdateType updateSource)
        {
            MyEcho();

            if (!bConfigured)
                return;

            Debugging();
            if (DEBUG_BREAK)
                return;

            DebugLog.Append("Status Updateing...\n");
            Status.Update(HEAD);
            Status.WriteToStream(MSG_IX_BUFFER);
            UserArgs(argument);
            MessageHandling();
            Run();
        }
        public void Save()
        {
            Storage = $"{(int)CurrentSettings.Task}:{(int)flightMode}:{RecieveTime}";
        }
        public void Load()
        {
            try
            {
                string[] raw = Storage.Split(':');
                //SetTask((DroneTask)int.Parse(raw[0]));
                //SetFlightMode((FlightMode)int.Parse(raw[1]));
                RecieveTime = DateTime.Parse(raw[2]);
                Echo("Loaded");
            }
            catch
            {
                Echo("Nothing to load");
            }
        }
        #endregion
        #endregion

    }
}
