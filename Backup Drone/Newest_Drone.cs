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
        #region MOTHBALL
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
        void VelocityCapVector(ref Vector3 vector, float cap)
        {
            float mag = Vector3.Distance(vector, Vector3.Zero);
            mag = mag > cap ? cap / mag : 1;
            vector *= mag;
        }
        public bool BufferedCollision(MyDetectedEntityInfo a, IMyCubeGrid b, float buffer, out float deltaMin, out float deltaMax)
        {
            /* Note: This can be easily scaled up to any number of dimensions, but for the sake of this assignment, I have hard coded two iterations (X,Z), and performed a single
             * range comparison, rather than carry a buffer of the "remaining plausible impact time". */

            float[,] deltas = new float[3, 2];
            deltaMin = float.NegativeInfinity;
            deltaMax = float.PositiveInfinity;

            for (int i = 0; i < 3; i++)
            {
                if (!DimensionCollisionCheck(
                    (float)a.Position.GetDim(i), a.Velocity.GetDim(i), (float)a.BoundingBox.HalfExtents.GetDim(i) + buffer,
                    (float)b.GetPosition().GetDim(i), b.LinearVelocity.GetDim(i), (float)b.WorldAABB.HalfExtents.GetDim(i) + buffer,
                    out deltas[i, 0], out deltas[i, 1]))
                {
                    deltaMin = float.NegativeInfinity;
                    deltaMax = float.NegativeInfinity;
                    return false;
                }

                DebugLog.Append($"Dimension {i} collision: {deltaMin}/{deltaMax}\n");

                //if (deltas[i,0] < 0)
                //{
                //    if (deltas[i, 1] < 0)
                //        return float.PositiveInfinity;
                //    deltas[i, 0] = 0;
                //}

                deltaMin = deltaMin > deltas[i, 0] ? deltaMin : deltas[i, 0];
                deltaMax = deltaMax < deltas[i, 1] ? deltaMax : deltas[i, 1];

                if (i == 0)
                    continue;

                if ((deltas[i, 0] > deltas[i - 1, 1] || deltas[i - 1, 0] > deltas[i, 1]) ||
                    (i == 2 && deltas[2, 0] > deltas[0, 1] || deltas[0, 0] > deltas[2, 1]))
                {
                    deltaMin = float.NegativeInfinity;
                    deltaMax = float.NegativeInfinity;
                    return false;
                }
            }

            return deltaMax > 0;
            //float xDeltaMin, xDeltaMax; // "start" and "end" points in time where the objects overlap in this dimension
            //DimensionCollisionCheck((float)a.GetPosition().X, a.LinearVelocity.X, (float)a.WorldAABB.HalfExtents.X, (float)b.GetPosition().X, b.LinearVelocity.X, (float)b.WorldAABB.HalfExtents.X, out xDeltaMin, out xDeltaMax);
            //
            //if (xDeltaMin < 0)
            //{
            //    if (xDeltaMax < 0) // leave early if there is no collision or it occured in the past
            //        return float.PositiveInfinity;
            //    xDeltaMin = 0; // clamp the min to zero (we aren't concerned with the past)
            //}
            //
            //float zDeltaMin, zDeltaMax;
            //DimensionCollisionCheck(a.Position.Z, a.LinearVelocity.Z, a.BoundingRadius, b.Position.Z, b.LinearVelocity.Z, b.BoundingRadius, out zDeltaMin, out zDeltaMax);
            //
            //if (zDeltaMin < 0)
            //{
            //    if (zDeltaMax < 0)
            //        return float.PositiveInfinity;
            //    zDeltaMin = 0;
            //}
            //
            //if (xDeltaMin > zDeltaMax || zDeltaMin > xDeltaMax) // check if dimension contact ranges overlap at all
            //    return float.PositiveInfinity;
            //
            //return xDeltaMin > zDeltaMin ? xDeltaMin : zDeltaMin; // return the greater of the two minimums for soonest collision (all dimensions need to have some collision)
        }

        /// <summary>
        /// Isolates the exact two points in time when the pairs of opposing side edges first touch/leave one another, depending on the net speed between both objects.
        /// This can return negative points in time, signifying a "point in the past" where they could have had initial/final contact assuming they existed back then with the same parameters.
        /// Returns the smaller of the two values as the min, and the greater as the max.
        /// </summary>
        /// <param name="positionA"></param>
        /// <param name="velocityA"></param>
        /// <param name="radiusA"></param>
        /// <param name="positionB"></param>
        /// <param name="velocityB"></param>
        /// <param name="radiusB"></param>
        /// <param name="deltaMin"></param>
        /// <param name="deltaMax"></param>
        public bool DimensionCollisionCheck(float positionA, float velocityA, float radiusA, float positionB, float velocityB, float radiusB, out float deltaMin, out float deltaMax)
        {
            deltaMin = float.NegativeInfinity;

            if (velocityA == velocityB) // Resolve edge case where they are travelling the same speed along the given dimension
            {
                if (positionA - radiusA <= positionB + radiusB && positionA + radiusA >= positionB - radiusB) // are they currently, and thus indefinitely, collided with one another?
                {
                    deltaMax = float.PositiveInfinity;
                    return true;
                }

                // They will never collide
                deltaMax = float.NegativeInfinity;
                return false;
            }

            // Generalize the point slope form for each object (position in dimension vs. time)
            // objA: y = mx + a
            // objB: y = nx + b

            // Substitute for x (we are only concerned with time of intersection)
            // x(m-n) = (c-b)
            // x = (b-a)/(m-n)

            // Match opposing max/min edge values of the two objects, determine the point in time where they intersect
            // x0 = (bMax - aMin) / (m - n)
            // x1 = (bMin - aMax) / (m - n)

            float aMin_bMax_delta = ((positionB + radiusB) - (positionA - radiusA)) / (velocityA - velocityB);
            float bMin_aMax_delta = ((positionB - radiusB) - (positionA + radiusA)) / (velocityA - velocityB);

            deltaMin = aMin_bMax_delta < bMin_aMax_delta ? aMin_bMax_delta : bMin_aMax_delta;
            deltaMax = aMin_bMax_delta > bMin_aMax_delta ? aMin_bMax_delta : bMin_aMax_delta;

            return true;
        }


        #endregion

        #region DRONE

        // User Defs //
        public static Program PROG { get; private set; }

        const string FollowEyeName = "EYEBALL";
        const string RootName = "TEST_DRONE";
        const string LostChannel = "NEED_HUB";
        const string Mechchannel = "MECH";
        const string PortName = "PORT";
        const string ShieldControlName = "[A] Shield Control";
        const string ShieldName = "[A] Shield";

        #region AUTO_MINE
        const double MINE_DISTANCE = 20f; // 1000f
        const double MINE_LERP_RATE = .1f; // 0.0005f
        const double MINE_ROTATION = 1f;

        const int SHIELD_OFFSET_SCALE = 5;

        Vector3 MiningOrigin;
        Vector3 MiningEnd;
        double MiningProgress;
        #endregion

        static long longlowEnd = (long)Math.Pow(2, 32) - 1;
        static long longhighEnd = (long)Math.Pow(2, 64) - 1 - longlowEnd;

        const byte DEFAULT_DRIFT = 100;

        const double MECH_SCALE = 0.1f;
        const double MECH_LOOK_THRESHOLD = 0.1f;
        const double MECH_LOOK_LIMIT = 5f;
        const double MECH_APPROACH_LIMIT = 15f;
        const double DOCK_DISTANCE = 1.8f;
        const double CHECK_PROXY = 1f;
        const double DRIFT_PROXY = 0.1f;
        const float SENSOR_RANGE = 50f;
        const float THRUST_SCALE = 10000f;
        const float ARRIVE_SCALE = 10f;
        const float ARRIVE_SQUARED = ARRIVE_SCALE * ARRIVE_SCALE;
        const float AVOID_SCALE = 40;
        const float AVOID_SQUARED = AVOID_SCALE * AVOID_SCALE;
        const double GYRO_SCALE = 0.01f;

        const double GYRO_PROXY = 1.6;
        const double RAD2DEG = 180 / Math.PI;

        const int EVENT_TIME = 20;

        
        static TimeSpan EXP_TIME = TimeSpan.FromSeconds(5);
        static StringBuilder DebugLog;

        int EventClock = 0;
        DateTime RecieveTime;

        double biggestSelfHalfExtent = 0;
        double CurrentGoalDistance = 0;
        double FurthestGoalDistance = 0;

        bool DEBUG_SYSTEM_INTERRUPT = false;
        bool DEBUG_BREAK = false;
        bool MANUAL_OPERATON = false;
        bool MECH_OPERATION = false;

        bool Configured = false;
        bool CockpitAvailable => DroneControl != null && DroneControl is IMyCockpit;
        IMyCockpit Cockpit => (IMyCockpit)DroneControl;
        bool RCavailable => DroneRC != null;

        bool TargetGood = false;

        bool Busy =>
            CurrentStatus.Task == DroneTask.DOCK ||
            CurrentStatus.Task == DroneTask.UN_DOCK;

        DroneStatus CurrentStatus = DroneStatus.Default;
        DroneStatus CachedStatus = DroneStatus.Default;

        SwarmSettings RequestedSettings = SwarmSettings.Default;
        //SwarmSettings CurrentSettings = SwarmSettings.Default;

        WorldFrame CurrentFrame;
        WorldFrame Goal;

        FlightMode CurrentFlightMode;

        // Network
        IMyBroadcastListener DroneEar;

        string HubChannel;
        string InChannel;

        // Hardware
        IMyTextSurface DroneSurface;
        IMyTextSurface CockpitScreen;
        IMyRemoteControl DroneRC;
        IMyShipController DroneControl;
        IMyTerminalBlock PrimaryHead;
        IMyShipConnector Port;
        IMyTerminalBlock FollowEye;
        IMyTerminalBlock Shield;
        IMyTerminalBlock ShieldController;
        IMyShipDrill ForwardDrill;
        IMyShipWelder Welder;

        List<IMyCargoContainer> Cargos;
        List<IMySensorBlock> Sensors;
        List<IMyShipDrill> Drills;
        List<IMyShipGrinder> Grinders;
        List<IMyLargeTurretBase> Weapons;
        List<IMyGasTank> FuelTanks;
        List<IMyBatteryBlock> Batteries;

        static IMyTerminalBlock HEAD;

        List<IMyGyro> Gyros = new List<IMyGyro>();
        List<IMyThrust> Thrusters = new List<IMyThrust>();

        static Vector3D GyroVectorsRadians = Vector3D.Zero;
        static Vector3D GyroVectorsResolved;
        Vector3D ThrustVector;
        Vector3D buffer;
        Vector2D WalkVector = Vector2D.Zero;
        MechAction MechActionBuffer;

        Sequence CurrentSequence;
        Sequence DockingSequence;

        public class Sequence
        {
            //public Program PROG;
            public int CurrentIndex;
            public Operation[] Operations;

            public Sequence(Operation[] operations)
            {
                Operations = operations;
                CurrentIndex = 0;
            }

            public Trigger CurrentTrigger()
            {
                if (Operations == null || Operations.Length < 1)
                    return Trigger.ERROR;

                return Operations[CurrentIndex].Trigger;
            }
            public void InitializeSequence()
            {
                DebugLog.Append("Initialize Sequence...\n");
                LoadOperation(0);
            }
            public void EndSequence()
            {
                PROG.RollBack();
            }
            public bool NextOperation()
            {
                return LoadOperation(CurrentIndex + 1);
            }
            bool LoadOperation(int index)
            {
                DebugLog.Append($"Loading Operation: {index}\n");

                if (Operations == null || Operations.Length < 1)
                    return false;

                if (index < 0 || index >= Operations.Length)
                {
                    DebugLog.Append($"Sequence Ended: {index}\n");
                    return true;
                }

                CurrentIndex = index;
                PROG.SetFlightMode(Operations[CurrentIndex].FlightMode);

                return true;
            }
        }

        #region ENUMS
        public enum Trigger
        {
            ERROR,
            ALIGN,
            DOCK,
            RESUPPLY
        }
        public enum FlightMode
        {
            STANDBY,
            NAV,
            FLIGHT_AI,
            MECH_AI,
            RESUPPLY,
            SKYNET
        }


        static double[] MY_ID_CACHE = new double[2];
        static string[] MSG_STR_BUFFER = new string[3];
        static byte[] MSG_BYTE_BUFFER = new byte[4];
        static double[] MECH_IX_BUFFER = new double[Enum.GetValues(typeof(MechIx)).Length];
        static double[] MSG_IX_BUFFER = new double[Enum.GetValues(typeof(MsgIx)).Length];

        public enum DroneTask
        {
            REGISTER,
            //BUSY,
            UN_DOCK,
            IDLE,
            DOCK,
            LOBBY,
            FORM,
            FOLLOW,
            HAUL,
            MINE,
            GRIND
        }
        public enum Option
        {
            NEW_REQUEST = 0,
            SHIELD_ON = 1,
            FLIGHT_TRUE = 2,
            SPIN_ON = 3,
            SHUNT_ON = 4
        }
        enum MechAction
        {
            DEFAULT,
            WAIT,
            ZERO_OUT,
            RELEASE,
        }
        enum MechIx
        {
            ACTION,
            MOVE_Z,
            MOVE_X,
            YAW,
            PITCH,
            ROLL,
        }
        enum MsgIx
        {
            ID_0 = 0,
            ID_1 = 1,

            MATRIX = 2,
            M12 = 3,
            M13 = 4,

            M21 = 5,
            M22 = 6,
            M23 = 7,

            M31 = 8,
            M32 = 9,
            M33 = 10,

            POS_X = 11,
            POS_Y = 12,
            POS_Z = 13,

            VEL_X = 14,
            VEL_Y = 15,
            VEL_Z = 16,

            STATUS = 17,
            SQUAD = 17,

            RESOURCES = 18
        }
        enum StatusIx
        {
            CURRENT,
            EQUIPMENT,
            SEQ_INDEX,
        }
        enum SquadIx
        {
            REQUEST,
            SPEED,
            SHIELD_SIZE,
            OPTIONS
        }
        enum ResourceIx
        {
            FUEL,
            POWER,
            INTEGRITY,
            CARGO,
        }
        enum Equipment
        {
            REMOTE,
            PORT,
            DRILLS,
            WELDER,
            GRINDERS,
            WEAPONS,
            CARGO,
            SHIELD
        }
        #endregion

        #region STRUCTS
        public struct Operation
        {
            public Trigger Trigger;
            public FlightMode FlightMode;

            public Operation(Trigger trigger, FlightMode mode)
            {
                Trigger = trigger;
                FlightMode = mode;
            }
        }

        public struct DroneStatus
        {
            public DroneTask Task;
            public byte SequenceIndex;
            public bool[] Equipment;

            public static DroneStatus Default = new DroneStatus(DroneTask.IDLE);

            public DroneStatus(DroneTask task, byte seqIndex = 0, bool[] equip = null)
            {
                Task = task;
                SequenceIndex = seqIndex;
                Equipment = equip != null ? equip : new bool[8];
            }
            public void ReadFromStream()
            {
                GetBytesFromDouble(MSG_IX_BUFFER[(int)MsgIx.STATUS]);

                Task = (DroneTask)MSG_BYTE_BUFFER[(int)StatusIx.CURRENT];
                SequenceIndex = MSG_BYTE_BUFFER[(int)StatusIx.SEQ_INDEX];
                GetBools(MSG_BYTE_BUFFER[(int)StatusIx.EQUIPMENT], Equipment);
            }
            public void WriteToStream()
            {
                MSG_BYTE_BUFFER[(int)StatusIx.CURRENT] = (byte)Task;
                MSG_BYTE_BUFFER[(int)StatusIx.SEQ_INDEX] = SequenceIndex;
                GetByteFromBools(out MSG_BYTE_BUFFER[(int)StatusIx.EQUIPMENT], Equipment);

                MSG_IX_BUFFER[(int)MsgIx.STATUS] = GetDoubleFromBytes();
            }
        }
        public struct SwarmSettings
        {
            public DroneTask Task;

            public byte MaxSpeed;
            public byte ShieldSize;

            public bool[] Options;

            public static SwarmSettings Default = new SwarmSettings(DroneTask.IDLE);

            public SwarmSettings(DroneTask task = DroneTask.IDLE, byte speed = DEFAULT_DRIFT, byte shieldSize = 0, bool[] options = null)
            {
                Task = task;
                MaxSpeed = speed;
                ShieldSize = shieldSize;
                Options = options == null || options.Length != 8 ? new bool[8] : options;
            }

            public bool this[Option option]
            {
                get { return Options[(int)option]; }
                set { Options[(int)option] = value; }
            }

            public void ReadFromStream()
            {
                GetBytesFromDouble(MSG_IX_BUFFER[(int)MsgIx.SQUAD]);

                Task = (DroneTask)MSG_BYTE_BUFFER[(int)SquadIx.REQUEST];
                MaxSpeed = MSG_BYTE_BUFFER[(int)SquadIx.SPEED];
                ShieldSize = MSG_BYTE_BUFFER[(int)SquadIx.SHIELD_SIZE];
                GetBools(MSG_BYTE_BUFFER[(int)SquadIx.OPTIONS], Options);
            }
            public void WriteToStream()
            {
                MSG_BYTE_BUFFER[(int)SquadIx.REQUEST] = (byte)Task;
                MSG_BYTE_BUFFER[(int)SquadIx.SPEED] = MaxSpeed;
                MSG_BYTE_BUFFER[(int)SquadIx.SHIELD_SIZE] = ShieldSize;
                GetByteFromBools(out MSG_BYTE_BUFFER[(int)SquadIx.OPTIONS], Options);

                MSG_IX_BUFFER[(int)MsgIx.SQUAD] = GetDoubleFromBytes();
            }
        }
        public struct WorldFrame
        {
            public static WorldFrame Zero = new WorldFrame(MatrixD.Zero, Vector3.Zero);
            public DateTime TimeStamp;
            public Vector3D Velocity;
            //public Vector3 velocity => Velocity;
            public MatrixD Matrix;

            public WorldFrame(MatrixD mat, Vector3 vel)
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
               
                Matrix = block.WorldMatrix;
                Velocity = block.CubeGrid.LinearVelocity;
                TimeStamp = DateTime.Now;
            }
            public void Update(MyDetectedEntityInfo info)
            {
                Matrix = info.Orientation;
                Matrix.Translation = info.Position;
                Velocity = info.Velocity;
                TimeStamp = DateTime.Now;
            }
            public void ReadFromStream()
            {
                ExtractMatrixD(ref Matrix);
                ExtractVector3D(ref Velocity, (int)MsgIx.VEL_X);
                TimeStamp = DateTime.Now;
            }
            public void WriteToStream()
            {
                InsertMatrixD(ref Matrix);
                InsertVector3D(ref Velocity, (int)MsgIx.VEL_X);
            }
            public bool Expired()
            {
                return DateTime.Now - TimeStamp > EXP_TIME;
            }
        }
        #endregion

        #region INITIALIZERS
        void DockingSequenceBuilder()
        {
            Operation[] operations = new Operation[3];

            operations[0] = new Operation(Trigger.ALIGN, FlightMode.FLIGHT_AI);      // Head towards docking port
            operations[1] = new Operation(Trigger.DOCK, FlightMode.FLIGHT_AI);    // Approach docking port and connect
            operations[2] = new Operation(Trigger.RESUPPLY, FlightMode.RESUPPLY);   // Resupply and disconnect

            DockingSequence = new Sequence(operations);

            //operations[3] = new Operation(Trigger.ALIGN, DroneRequest.UN_DOCK, FlightMode.TAI);    // Clear away from docking port
        }
        void GetBlocks<T>(ref List<T> list) where T : class
        {
            if (list != null)
                list.Clear();
            else
                list = new List<T>();
            GridTerminalSystem.GetBlocksOfType(list);
            DebugLog.Append($"{typeof(T)} : {list.Count}\n");
        }
        void GetBlock<T>(ref T field, string name) where T : class
        {
            field = (T)GridTerminalSystem.GetBlockWithName(name);
            DebugLog.Append($"{name} : {(field != null ? field.GetType().ToString() : "null")}\n");
        }
        void RefreshDroneEquipment()
        {
            GetBlock(ref Port, PortName);
            GetBlock(ref ShieldController, ShieldControlName);
            GetBlock(ref Shield, ShieldName);

            DebugLog.Append("Single Blocks Aquired\n");
            DebugLog.Append($"{(Port != null ? Port.GetType().ToString() : "null")} : {Port != null}\n");

            GetBlocks(ref Sensors);
            GetBlocks(ref Drills);
            GetBlocks(ref Grinders);
            GetBlocks(ref Weapons);
            GetBlocks(ref Cargos);

            DebugLog.Append("Multi-Blocks Aquired\n");

            if (Port != null)
                Port.Enabled = false;

            DebugLog.Append("Setting up Sensors...\n");
            SetupSensors();
            DebugLog.Append("Sensors Setup!\n");

            SetupDrills();
            DebugLog.Append("Drills Setup!\n");

            DockingSequenceBuilder();
            DebugLog.Append("Docking Sequence Setup!\n");

            SetupShipControl();
            DebugLog.Append("RemoteControl Setup!\n");

            PopulateThrusters();
            DebugLog.Append("Thrusters Setup!\n");

            PopulateGyros();
            DebugLog.Append("Gyros Setup!\n");

            MyAvailableHardWare();

            HEAD = PrimaryHead;
        }
        void SetupShipControl()
        {
            DebugLog.Append("yo0?\n");

            List<IMyShipController> controllers = new List<IMyShipController>();
            GridTerminalSystem.GetBlocksOfType(controllers);

            DebugLog.Append("yo1?\n");

            List<IMyRemoteControl> remotes = new List<IMyRemoteControl>();
            GridTerminalSystem.GetBlocksOfType(remotes);

            DebugLog.Append("yo2?\n");

            if (remotes.Count > 0)
            {
                SetupRemoteControl(remotes[0]);
            }
            DebugLog.Append("yo3?\n");
            if (controllers.Count > 0)
                DroneControl = controllers[0];
            else if (RCavailable)
                DroneControl = DroneRC;
            DebugLog.Append("yo4?\n");
            if (DroneControl != null)
            {
                DebugLog.Append("yo5?\n");
                PrimaryHead = DroneControl;
                if (CockpitAvailable)
                {
                    DebugLog.Append("Cockpit available!\n");
                    try { CockpitScreen = ((IMyCockpit)DroneControl).GetSurface(0); }
                    catch { }
                    CockpitScreen = CockpitScreen == null ? Me.GetSurface(0) : CockpitScreen;
                    CockpitScreen.ContentType = ContentType.TEXT_AND_IMAGE;
                    CockpitScreen.WriteText("");

                }
            }
            else
                PrimaryHead = Me;
            DebugLog.Append("yo6?\n");
        }
        /*void SetupCargos()
        {

        }
        void SetupGrinders()
        {

        }
        void SetupWeapons()
        {

        }*/
        void SetupRemoteControl(IMyRemoteControl droneRC)
        {
            DroneControl = droneRC;
            droneRC.FlightMode = Sandbox.ModAPI.Ingame.FlightMode.OneWay;
            droneRC.SetDockingMode(true);
            droneRC.SetCollisionAvoidance(true);
            droneRC.SetAutoPilotEnabled(false);
            droneRC.ClearWaypoints();
        }
        void SetupDrills()
        {
            if (Drills == null ||
                Drills.Count == 0)
                return;

            Dictionary<Base6Directions.Direction, int> orientations = new Dictionary<Base6Directions.Direction, int>();

            foreach (IMyShipDrill drill in Drills)
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
            foreach (IMyShipDrill drill in Drills)
                if (drill.Orientation.Forward == highest.Key)
                {
                    ForwardDrill = drill;
                    return;
                }
        }
        void SetupSensors()
        {
            if (Sensors == null ||
                Sensors.Count == 0)
                return;

            foreach (IMySensorBlock sensor in Sensors)
            {
                if (sensor == null)
                    return;

                DebugLog.Append($"{sensor.CustomName}\n");

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
                sensor.DetectPlayers = true;

                sensor.BackExtend = SENSOR_RANGE;
                sensor.BottomExtend = SENSOR_RANGE;
                sensor.FrontExtend = SENSOR_RANGE;
                sensor.LeftExtend = SENSOR_RANGE;
                sensor.RightExtend = SENSOR_RANGE;
                sensor.TopExtend = SENSOR_RANGE;

                DebugLog.Append("Sensor updated...\n");
            }
        }
        void PopulateGyros()
        {
            //List<IMyGyro> gyros = new List<IMyGyro>();
            Gyros.Clear();
            GridTerminalSystem.GetBlocksOfType(Gyros);
            Echo("gyros populated");
        }
        void PopulateThrusters()
        {
            Thrusters.Clear();
            GridTerminalSystem.GetBlocksOfType(Thrusters);

            Echo($"thrusters populated: {Thrusters.Count}");
        }
        //void SetMaxDrift(IMyShipController control, double fixedThrust)
        //{
        //    if (control == null)
        //    {
        //        CurrentSettings.MaxSpeed = DEFAULT_DRIFT;
        //        return;
        //    }
        //    double criticalAccel = fixedThrust / control.CalculateShipMass().TotalMass;
        //    double stopDistance = SENSOR_RANGE - control.WorldAABB.HalfExtents.Z; // Subtract half the 'length' of the ship from the sensor radius
        //    CurrentSettings.MaxSpeed = (byte)(Math.Sqrt((2 * stopDistance) / criticalAccel) * criticalAccel);
        //}
        #endregion

        #region COMMUNICATION
        void HubSend()
        {
            InsertMyIDpacket();
            InsertMyResources();

            CurrentFrame.WriteToStream();
            CurrentStatus.WriteToStream();

            IGC.SendBroadcastMessage(HubChannel, ImmutableArray.Create(MSG_IX_BUFFER));
        }
        bool DroneRecieve(MyIGCMessage msg)
        {
            if (msg.Data is ImmutableArray<double>)
                return TaskRecieve(msg);

            if (msg.Data is ImmutableArray<string>)
                return ActionRecieve(msg);

            if (msg.Data is string) // Error message
            {
                DebugLog.Append(msg.Data);
                return false;
            }

            DebugLog.Append("Couldn't translate!\n");

            return false;
        }
        bool ActionRecieve(MyIGCMessage msg)
        {
            DroneTask task;
            if (!ProcessMsgString(msg, ref MSG_STR_BUFFER, DebugLog) ||
                !Enum.TryParse(MSG_STR_BUFFER[0], out task))
            {
                return false;
            }

            switch (task)
            {
                case DroneTask.REGISTER:
                    RegisterHub();
                    break;

                case DroneTask.DOCK:
                    SetTask(DroneTask.DOCK);
                    break;
            }

            return true;
        }

        bool TaskRecieve(MyIGCMessage msg)
        {
            if (!ProcessMsgIx(msg))
            {
                DebugLog.Append("HandShake Processing failed!\n");
                return false;
            }

            RequestedSettings.ReadFromStream();

            if (RequestedSettings.Task != CurrentStatus.Task)
            {
                if (!Busy)
                    SetTask(RequestedSettings.Task);

                else if (!RequestedSettings[Option.NEW_REQUEST])
                {
                    DebugLog.Append("Old request!\n");
                    return false;
                }
            }

            Goal.ReadFromStream();

            if (CurrentStatus.Task == DroneTask.DOCK)
            {
                DebugLog.Append("Docking...\n");
                double mag = Vector3D.DistanceSquared(Goal.Matrix.Translation, CurrentFrame.Matrix.Translation) / ARRIVE_SQUARED;
                mag = mag > AVOID_SQUARED ? AVOID_SQUARED : mag;
                mag = DOCK_DISTANCE * (1+(mag / AVOID_SQUARED));
                Goal.Matrix.Translation += Goal.Matrix.Backward * mag;
            }

            DebugLog.Append("Processing Task...\n");

            return true;
        }

        void UnRegister()
        {
            HubChannel = LostChannel;
            SetTask(DroneTask.IDLE);
            CurrentStatus.Task = DroneTask.REGISTER;
        }
        void RegisterHub()
        {
            HubChannel = MSG_STR_BUFFER[1];
            DebugLog.Append("Hub Registered!\n");
            CurrentStatus.Task = DroneTask.IDLE;
        }
        void SetupNetwork()
        {
            Me.CustomName = RootName;
            GenerateMyIDpacket(Me.EntityId);
            Me.CubeGrid.CustomName = RootName;
            InChannel = $"DRONE_{Me.EntityId}";
            DroneEar = IGC.RegisterBroadcastListener(InChannel);
            UnRegister();
            DebugLog.Append("Network Setup!\n");
        }

        static void InsertMyIDpacket()
        {
            for (int i = 0; i < MY_ID_CACHE.Length; i++)
                MSG_IX_BUFFER[i] = MY_ID_CACHE[i];
        }
        void InsertMyResources()
        {
            int buffer = 0;

            for (int i = 0; i < 4; i++)
            {
                buffer |= ResourcePercentRemaining(i);
                buffer = buffer << 8;
            }

            MSG_IX_BUFFER[(int)MsgIx.RESOURCES] = buffer;
        }
        void MyAvailableHardWare()
        {
            for (int i = 0; i < Enum.GetNames(typeof(Equipment)).Length; i++)
            {
                switch ((Equipment)i)
                {
                    case Equipment.REMOTE:
                        CurrentStatus.Equipment[i] = DroneRC != null;
                        break;

                    case Equipment.PORT:
                        CurrentStatus.Equipment[i] = Port != null;
                        break;

                    case Equipment.SHIELD:
                        CurrentStatus.Equipment[i] = Shield != null &&
                            ShieldController != null;
                        break;

                    case Equipment.WELDER:
                        CurrentStatus.Equipment[i] = Welder != null;
                        break;

                    case Equipment.DRILLS:
                        CurrentStatus.Equipment[i] = Drills != null &&
                            Drills.Count > 0;
                        break;

                    case Equipment.GRINDERS:
                        CurrentStatus.Equipment[i] = Grinders != null &&
                            Grinders.Count > 0;
                        break;

                    case Equipment.WEAPONS:
                        CurrentStatus.Equipment[i] = Weapons != null &&
                            Weapons.Count > 0;
                        break;

                    case Equipment.CARGO:
                        CurrentStatus.Equipment[i] = Cargos != null &&
                            Cargos.Count > 0;
                        break;
                }
            }
        }
        void GenerateMyIDpacket(long id)
        {
            long buffer;

            for (int i = 0; i < MY_ID_CACHE.Length; i++)
            {
                buffer = id & longlowEnd;
                id = id & longhighEnd;
                MY_ID_CACHE[i] = buffer;
                id = id >> 32;
            }
        }

        static bool ProcessMsgIx(MyIGCMessage msg)
        {
            try
            {
                ImmutableArray<double> raw = ((ImmutableArray<double>)msg.Data);
                for (int i = 0; i < MSG_IX_BUFFER.Length; i++)
                    MSG_IX_BUFFER[i] = i < raw.Length ? raw[i] : 0;
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

        #endregion

        #region STREAM HELPERS
        static void InsertVector3D(ref Vector3D input, int index)
        {
            for (int i = 0; i < 3; i++)
                MSG_IX_BUFFER[index + i] = input.GetDim(i);
        }
        static void ExtractVector3D(ref Vector3D output, int index)
        {
            for (int i = 0; i < 3; i++)
                output.SetDim(i, MSG_IX_BUFFER[index + i]);
        }
        static void InsertMatrixD(ref MatrixD input)
        {
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 3; c++)
                    MSG_IX_BUFFER[(int)MsgIx.MATRIX + (r * 3) + c] = /*c == 3 ? 1 : */input[r, c];
        }
        static void ExtractMatrixD(ref MatrixD output)
        {
            output = MatrixD.Identity;
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 3; c++)
                    output[r, c] = MSG_IX_BUFFER[(int)MsgIx.MATRIX + (r * 3) + c];
            //for (int c = 0; c < 4; c++)
            //    output[r, c] = c == 3 ? 1 : MSG_IX_BUFFER[(int)MsgIx.MATRIX + (r * 3) + c];
        }
        static void GetBytesFromDouble(double input)
        {
            int buffer = (int)input;

            for (int i = MSG_BYTE_BUFFER.Length - 1; i > -1; i--)
            {
                MSG_BYTE_BUFFER[i] = (byte)(buffer & 255);
                buffer = buffer >> 8;
            }
        }
        static double GetDoubleFromBytes()
        {
            int buffer = 0;

            for (int i = 0; i < MSG_BYTE_BUFFER.Length; i++)
            {
                buffer = buffer << 8;
                buffer |= MSG_BYTE_BUFFER[i];
            }
            return (double)buffer;
        }
        static void GetByteFromBools(out byte bite, bool[] bits)
        {
            int result = 0;

            for (int i = 0; i < 8; i++)
            {
                result = result << 1;
                result += bits[i] ? 1 : 0;
            }
            bite = (byte)result;
        }
        static void GetBools(byte bite, bool[] bits)
        {
            uint buffer = bite;

            for (int i = 0; i < 8; i++)
            {
                bits[i] = (buffer & 128) == 128;
                buffer = buffer << 1;
            }
        }
        #endregion

        #region FLIGHT CONTROL
        void GenerateThrustVector()
        {

            double alpha, Alpha, Beta, Delta, dot;
            Vector3D perp, solution, normalDrift, normalTarget, avoidVector;
            double targetMag, avoidMag;//, goalMag;

            ThrustVector = Goal.Matrix.Translation - HEAD.GetPosition();
            GenerateCollisionAvoidanceVector(out avoidVector);

            Vector3D.DistanceSquared(ref ThrustVector, ref Vector3D.Zero, out targetMag);
            Vector3D.DistanceSquared(ref avoidVector, ref Vector3D.Zero, out avoidMag);

            double netMag = targetMag - avoidMag;
            netMag = netMag > 0 ? netMag : 0;
            netMag /= targetMag;
            ThrustVector *= netMag;

            ThrustVector += avoidVector;

            targetMag = (targetMag * RequestedSettings.MaxSpeed) / (targetMag + (CurrentStatus.Task == DroneTask.DOCK ? ARRIVE_SQUARED : ARRIVE_SCALE));

            Vector3D.Normalize(ref ThrustVector, out ThrustVector);
            ThrustVector *= targetMag;

            ThrustVector += Goal.Velocity;

            Vector3D.Distance(ref ThrustVector, ref Vector3D.Zero, out targetMag);
            Vector3D.Cross(ref ThrustVector, ref CurrentFrame.Velocity, out perp);
            if (perp != Vector3D.Zero)
            {
                //DebugLog.Append($"Perp: {perp}\n");
                Vector3D.Cross(ref ThrustVector, ref perp, out solution);
                Vector3D.Normalize(ref solution, out solution);
                //DebugLog.Append($"Step6: {solution}\n");
                Vector3D.Normalize(ref CurrentFrame.Velocity, out normalDrift);
                Vector3D.Normalize(ref ThrustVector, out normalTarget);
                Vector3D.Dot(ref normalTarget, ref normalDrift, out dot);
                Vector3D.Distance(ref CurrentFrame.Velocity, ref Vector3D.Zero, out Delta);
                Delta = Delta < DRIFT_PROXY ? 0 : Delta;

                alpha = Math.Acos(dot);
                Alpha = Math.Sin(alpha) * Delta;
                Beta = Math.Cos(alpha) * Delta;

                solution *= Alpha;
                //DebugLog.Append($"Step7: {solution}\n");
                solution += normalTarget * (targetMag - Beta);
                //DebugLog.Append($"Step8: {solution}\n");
                ThrustVector = solution;
            }

            ThrustVector *= THRUST_SCALE;

            DebugLog.Append($"Thrust Vectors Generated: {ThrustVector}\n");
        }

        static Vector3D up_buffer;

        Vector3D BestUpVector()
        {
            up_buffer = Vector3D.Zero;
            
            if (CockpitAvailable)
                up_buffer = -Cockpit.GetNaturalGravity();
            
            if (up_buffer == Vector3D.Zero)
                up_buffer = CurrentFlightMode == FlightMode.MECH_AI ? Goal.Matrix.Up : HEAD.WorldMatrix.Up;

            DebugLog.Append($"UpVector: {up_buffer}\n");

            return up_buffer;
        }

        void GenerateGyroVectors()
        {
            MatrixD orient;

            if (RequestedSettings.Options[(int)Option.FLIGHT_TRUE] ||
                CurrentFlightMode == FlightMode.MECH_AI)
            {
                if (CurrentFlightMode == FlightMode.MECH_AI &&
                    CurrentGoalDistance < MECH_LOOK_LIMIT)
                {
                    GyroVectorsRadians = Vector3D.Zero;
                    return;
                }

                Vector3D headPos = HEAD.GetPosition();
                Vector3D target = CurrentFlightMode == FlightMode.MECH_AI ? Goal.Matrix.Translation : headPos + CurrentFrame.Velocity;
                Vector3D upVector = BestUpVector();

                CreateLookAt(ref headPos, ref target, ref upVector, out orient);
            }

            else
            {
                //DebugLog.Append("Fixed Direction Orientation\n");
                orient = Goal.Matrix;
            }

            DebugLog.Append($"Step1:\n {MatrixToStringA(orient)}");

            MatrixD correct = HEAD.WorldMatrix;

            DebugLog.Append($"Step2:\n {MatrixToStringA(correct)}");

            orient.Translation = Vector3D.Zero;
            correct.Translation = Vector3D.Zero;

            correct = GenerateCorrection(ref correct, ref orient);

            DebugLog.Append($"Step3:\n {MatrixToStringA(correct)}");

            MatrixD.GetEulerAnglesXYZ(ref correct, out GyroVectorsRadians);

            DebugLog.Append($"Step4: {GyroVectorsRadians}\n");


            GyroVectorsResolved = GyroVectorsRadians * RAD2DEG * GYRO_SCALE;
            DebugLog.Append($"Gyro Corrections Generated: {GyroVectorsResolved}\n");
        }
        static Vector3D[] vector_buffer = new Vector3D[4];
        static void CreateLookAt(ref Vector3D camPos, ref Vector3D targetPos, ref Vector3D camUp, out MatrixD result)
        {
            vector_buffer[3] = camPos;                                                                      // translate result to cam
            vector_buffer[2] = camPos - targetPos;                                                          // set z to reverse delta                                                                     
            vector_buffer[2] = vector_buffer[2] == Vector3D.Zero ? Vector3D.Backward : vector_buffer[2];    // zero corner-case
            Vector3D.Normalize(ref camUp, out camUp);
            Vector3D.Normalize(ref vector_buffer[2], out vector_buffer[2]);
            Vector3D.Cross(ref camUp, ref vector_buffer[2], out vector_buffer[0]);                          // generate right                                                                        // z-flip
            Vector3D.Normalize(ref vector_buffer[0], out vector_buffer[0]);
            Vector3D.Cross(ref vector_buffer[2], ref vector_buffer[0], out vector_buffer[1]);               // correct up

            // populate matrix
            result = MatrixD.Identity;
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 3; c++)
                    result[r, c] = vector_buffer[r].GetDim(c);
        }
        void ApplyThrust(bool kill = false)
        {
            foreach (IMyThrust thrust in Thrusters)
                thrust.ThrustOverride = kill ? 0 : ModifiedThrustValue(thrust);

            DebugLog.Append("Thrust Complete!\n");
        }
        float ModifiedThrustValue(IMyThrust thruster)
        {
            double dot;
            buffer = thruster.WorldMatrix.Forward;
            Vector3D.Dot(ref buffer, ref ThrustVector, out dot);
            dot *= -1;
            dot = dot < 0 ? 0 : dot;
            return (float)dot;
        }
        void ApplyGyros(bool kill = false)
        {
            foreach (IMyGyro gyro in Gyros)
                ApplyGyroVector(gyro, kill);

            DebugLog.Append("Gyros Complete!\n");
        }
        void ToggleGyros(bool toggle = false)
        {
            DebugLog.Append("Toggling Gyros...\n");

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
        void GenerateCollisionAvoidanceVector(out Vector3D output)
        {
            output = Vector3.Zero;

            if (Sensors.Count == 0)
                return;

            List<MyDetectedEntityInfo> nextSet = new List<MyDetectedEntityInfo>();
            List<MyDetectedEntityInfo> detectedEntities = new List<MyDetectedEntityInfo>();

            foreach (IMySensorBlock sensor in Sensors)
            {
                sensor.DetectedEntities(nextSet);

                foreach (MyDetectedEntityInfo info in detectedEntities)
                    nextSet.Remove(info);

                detectedEntities.AddRange(nextSet);
            }

            if (detectedEntities.Count == 0)
                return;

            double goalDeltaSquared = Vector3D.DistanceSquared(ThrustVector, Vector3D.Zero);
            double arriveAdjust = goalDeltaSquared / ARRIVE_SQUARED;
            arriveAdjust = arriveAdjust < 1 ? arriveAdjust : 1;

            foreach (MyDetectedEntityInfo entity in detectedEntities)
            {
                Vector3D hitPos = entity.HitPosition.HasValue ? entity.HitPosition.Value : entity.Position;

                Vector3D targetDelta = hitPos - CurrentFrame.Matrix.Translation;
                Vector3D driftDelta = CurrentFrame.Velocity - entity.Velocity;
                double targetDeltaSquared = Vector3D.DistanceSquared(targetDelta, Vector3D.Zero);
                double driftDeltaSquared = Vector3D.DistanceSquared(driftDelta, Vector3D.Zero);

                if (targetDeltaSquared > goalDeltaSquared ||
                    driftDeltaSquared == 0)
                    continue;

                double biggestTargetHalfExtent = 1;
                for (int i = 0; i < 3; i++)
                    biggestTargetHalfExtent = biggestTargetHalfExtent > entity.BoundingBox.HalfExtents.GetDim(i) ? biggestTargetHalfExtent : entity.BoundingBox.HalfExtents.GetDim(i);
                double halfExtentSquared = Math.Pow(biggestSelfHalfExtent + biggestTargetHalfExtent, 2);

                Vector3D driftNormal = Vector3D.Normalize(driftDelta == Vector3D.Zero ? CurrentFrame.Matrix.Forward : driftDelta);
                Vector3D targetNormal = Vector3D.Normalize(targetDelta);

                if (targetDeltaSquared < halfExtentSquared)
                {
                    output += -targetNormal * AVOID_SQUARED;
                    continue;
                }

                float dot = Vector3.Dot(driftNormal, targetNormal);

                if (dot < 0)
                    continue;

                double alpha = Math.Acos(dot);

                double Beta = Math.Cos(alpha) * targetDeltaSquared;
                double Alpha = Math.Sin(alpha) * targetDeltaSquared;
                

                

                double avoidMag = halfExtentSquared * 2;//* 2 * arriveAdjust;
                double avoidLateral = avoidMag - Alpha;
                double avoidLinear = avoidMag - Beta;

                avoidLateral = avoidLateral < 0 ? 0 : avoidLateral;
                //avoidLateral *= arriveAdjust;
                avoidLinear = avoidLinear < 0 ? 0 : avoidLinear;

                double avoidTotal = avoidLateral * avoidLinear * driftDeltaSquared;

                Vector3 origin = CurrentFrame.Matrix.Translation + (driftNormal * Beta);
                Vector3 avoidDelta = Vector3D.Normalize(origin - hitPos) * Math.Pow(avoidTotal, 2);

                DebugLog.Append(
                    $"hitPos: {hitPos}\n" +
                    $"driftNormal: {driftNormal}\n" +
                    $"targetNormal: {targetNormal}\n" +
                    $"origin: {origin}\n" +
                    $"avoidDelta: {avoidDelta}\n\n");

                output += avoidDelta;

            }
            output *= arriveAdjust;
        }
        
        static MatrixD GenerateCorrection(ref MatrixD current, ref MatrixD target)
        {
            return MatrixD.Multiply(target, MatrixD.Invert(current));
        }

        static Vector3D Reduce(Vector4 input)
        {
            return new Vector3D(input.X, input.Y, input.Z);
        }
        static void ApplyGyroVector(IMyGyro gyro, bool kill = false)
        {
            Vector3D correction = kill ? Vector3D.Zero : GyroVectorsResolved;

            
            Vector3D finals = Vector3D.Zero;

            for (int i = 0; i < 3; i++) // Vectors
                for (int j = 0; j < 3; j++) // Application
                    finals.SetDim(j, finals.GetDim(j) + Vector3D.Dot(Reduce(HEAD.WorldMatrix.GetRow(i)), Reduce(gyro.WorldMatrix.GetRow(j))) * correction.GetDim(i));
                
                    

            DebugLog.Append($"PYR: {finals}\n");

            gyro.Pitch = PROG.DEBUG_SYSTEM_INTERRUPT ? 0 : (float)finals.X;
            gyro.Yaw = PROG.DEBUG_SYSTEM_INTERRUPT ? 0 : (float)finals.Y;
            gyro.Roll = PROG.DEBUG_SYSTEM_INTERRUPT ? 0 : (float)finals.Z;
        }

        void NavMode()
        {
            if (!RCavailable ||
                CurrentFlightMode != FlightMode.NAV)
                return;

            if (TargetGood && ProxyCheckNav())
                Move();
        }
        void FlightAI()
        {
            if (TargetGood)
            {
                GenerateGyroVectors();
                GenerateThrustVector();
            }

            ApplyThrust(!TargetGood);
            ApplyGyros(!TargetGood);
        }
        void MechAI()
        {
            if (!TargetGood)
                return;

            CurrentGoalDistance = Vector3D.Distance(HEAD.WorldMatrix.Translation, Goal.Matrix.Translation);
            FurthestGoalDistance = FurthestGoalDistance > CurrentGoalDistance ? FurthestGoalDistance : CurrentGoalDistance;

            DebugLog.Append($"Distances (Current/Furthest): {CurrentGoalDistance}/{FurthestGoalDistance}\n");

            GenerateGyroVectors();
            GenerateWalkValue();
            TransmitMechInstructions();
        }

        void TransmitMechInstructions()
        {
            for (int i = 0; i < 3; i++)
            {
                double value = GyroVectorsRadians.GetDim(i);
                value = Math.Abs(value) < MECH_LOOK_THRESHOLD ? 0 : value;
                GyroVectorsRadians.SetDim(i, value);
            }
                

            GyroVectorsRadians *= MECH_SCALE;
            MECH_IX_BUFFER[(int)MechIx.PITCH] = GyroVectorsRadians.X;
            MECH_IX_BUFFER[(int)MechIx.YAW] = GyroVectorsRadians.Y;
            MECH_IX_BUFFER[(int)MechIx.ROLL] = GyroVectorsRadians.Z;
            MECH_IX_BUFFER[(int)MechIx.MOVE_X] = WalkVector.X;
            MECH_IX_BUFFER[(int)MechIx.MOVE_Z] = WalkVector.Y;
            MECH_IX_BUFFER[(int)MechIx.ACTION] = (double)MechActionBuffer;
            MechActionBuffer = MechAction.DEFAULT;

            for (int i = 0; i < MECH_IX_BUFFER.Length; i++)
                DebugLog.Append($"{(MechIx)i} : {MECH_IX_BUFFER[i]}\n");

            IGC.SendBroadcastMessage(Mechchannel, ImmutableArray.Create(MECH_IX_BUFFER), TransmissionDistance.CurrentConstruct);
        }

        void GenerateWalkValue()
        {
            Vector3D goalDelta = Goal.Matrix.Translation - HEAD.WorldMatrix.Translation;
            if (CurrentGoalDistance < MECH_APPROACH_LIMIT)
            {
                WalkVector = Vector2D.Zero;
                return;
            }
            double dot = Vector3D.Dot(goalDelta.Normalized(), HEAD.WorldMatrix.Forward.Normalized());
            dot = Math.Abs(dot) < 0.9f ? 0 : dot;
            WalkVector.Y = Math.Sign(dot);
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
            TargetGood = false;
        }
        bool ProxyCheckNav()
        {
            return !DroneRC.IsAutoPilotEnabled;
        }
        bool ProxyCheckLiteral()
        {
            return TargetGood && 
                Vector3.DistanceSquared(CurrentFrame.Matrix.Translation, Goal.Matrix.Translation) < Math.Pow(CHECK_PROXY,2) &&
                Vector3.DistanceSquared(CurrentFrame.Velocity, Goal.Velocity) < Math.Pow(DRIFT_PROXY,2) &&
                Vector3D.DistanceSquared(Vector3D.Zero, GyroVectorsResolved) < Math.Pow(GYRO_PROXY,2);
        }
        #endregion

        #region OPERATION LOGIC
        byte ResourcePercentRemaining(int index)
        {
            return ResourcePercentRemaining((ResourceIx)index);
        }
        byte ResourcePercentRemaining(ResourceIx resType)
        {
            switch (resType)
            {
                case ResourceIx.FUEL:
                    return FuelRemaining();

                case ResourceIx.POWER:
                    return PowerRemaining();

                case ResourceIx.CARGO:
                    return CargoFilled();

                default:
                    return 0;
            }
        }
        byte FuelRemaining()
        {
            if (FuelTanks == null)
                return 0;

            double current = 0;
            double max = 0;

            foreach (IMyGasTank tank in FuelTanks)
            {
                current += tank.FilledRatio * tank.Capacity;
                max += tank.Capacity;
            }
            return (byte)((current / max) * 100);
        }
        byte PowerRemaining()
        {
            if (Batteries == null)
                return 0;

            double current = 0;
            double max = 0;

            foreach (IMyBatteryBlock battery in Batteries)
            {
                current += battery.CurrentStoredPower;
                max += battery.MaxStoredPower;
            }
            return (byte)((current / max) * 100);
        }
        byte CargoFilled()
        {
            if (Cargos == null)
                return 0;

            double current = 0;
            double max = 0;

            foreach (IMyCargoContainer cargo in Cargos)
            {
                current += (double)cargo.GetInventory().CurrentVolume / (double)cargo.GetInventory().MaxVolume;
                max += (double)cargo.GetInventory().MaxVolume;
            }
            return (byte)((current / max) * 100);
        }

        void SetTask(DroneTask task)
        {
            CachedStatus = CurrentStatus;
            DroneTask oldTask = CurrentStatus.Task;
            CurrentStatus.Task = task;
            TargetGood = false;
            HEAD = null;

            DebugLog.Append("Setting Task...\n");

            switch (CurrentStatus.Task)
            {
                case DroneTask.DOCK:
                    if (Port == null)
                    {
                        CurrentStatus.Task = oldTask;
                        break;
                    }
                    HEAD = Port;
                    DockingSequence.InitializeSequence();
                    break;

                case DroneTask.FORM:
                    SetFlightMode(FlightMode.FLIGHT_AI);
                    break;

                case DroneTask.FOLLOW:
                    HEAD = FollowEye;
                    break;

                case DroneTask.IDLE:
                    SetFlightMode(FlightMode.STANDBY);
                    break;

                case DroneTask.LOBBY:
                    SetFlightMode(FlightMode.NAV);
                    break;

                case DroneTask.HAUL:
                    if (Cargos.Count < 1)
                    {
                        CurrentStatus.Task = oldTask;
                        break;
                    }
                    SetFlightMode(FlightMode.NAV);
                    break;

                case DroneTask.MINE:
                    if (ForwardDrill == null)
                    {
                        CurrentStatus.Task = oldTask;
                        break;
                    }
                        
                    HEAD = ForwardDrill;
                    SetFlightMode(FlightMode.FLIGHT_AI);
                    break;
            }

            if (HEAD == null)
                HEAD = PrimaryHead;

            //RefreshShields();
        }
        void RefreshShields()
        {
            if (Shield == null)
                return;

            DebugLog.Append("Refreshing Shield!\n");

            ShieldController.SetValueFloat("DS-CFit", RequestedSettings.ShieldSize);
            //ShieldController.SetValueFloat("DS-C_OffsetWidthSlider", RequestedSettings.ShieldSize * SHIELD_OFFSET_SCALE);
            //ShieldController.SetValueFloat("DS-C_OffsetHeightSlider", RequestedSettings.ShieldSize * SHIELD_OFFSET_SCALE);
            //ShieldController.SetValueFloat("DS-C_OffsetDepthSlider", RequestedSettings.ShieldSize * SHIELD_OFFSET_SCALE);
        }
        void SetFlightMode(FlightMode mode)
        {
            DebugLog.Append($"Setting Flight Mode:{mode}\n");

            CurrentFlightMode = mode;

            if (CurrentFlightMode == FlightMode.FLIGHT_AI)
                ToggleGyros(true);

            else
            {
                ApplyThrust(true);
                ToggleGyros();
            }

            foreach (IMyThrust thrust in Thrusters)
                thrust.Enabled = CurrentFlightMode != FlightMode.MECH_AI;
        }
        void RollBack()
        {
            CurrentStatus = CachedStatus;
            SetTask(CurrentStatus.Task);
        }
        void DockingProcedure()
        {
            DebugLog.Append("Docking Procedure!\n");
            
            switch (DockingSequence.CurrentTrigger())
            {
                case Trigger.ALIGN:

                    if (ProxyCheckLiteral())
                    {
                        DebugLog.Append("Aligned!\n");
                        Port.Enabled = true;
                    }
                        
                    else
                    {
                        DebugLog.Append("Not Aligned Yet...\n");
                        break;
                    }
                        
                    if (Port.IsWorking)
                    {
                        DebugLog.Append("Port Online!\n");
                        DockingSequence.NextOperation();
                    }
                        
                    break;

                case Trigger.DOCK:

                    if (Port.Status == MyShipConnectorStatus.Connectable)
                        Port.Connect();

                    if (Port.Status == MyShipConnectorStatus.Connected)
                        DockingSequence.NextOperation();

                    break;

                case Trigger.RESUPPLY:

                    EventClock++;
                    DebugLog.Append($"EventClock: {EventClock}\n");

                    if (EventClock >= EVENT_TIME)
                    {
                        Port.Disconnect();
                        EventClock = 0;
                        Port.Enabled = false;
                        CurrentStatus.Task = DroneTask.UN_DOCK;
                        HubSend();
                        DockingSequence.NextOperation();
                    }
                    break;

                default:
                    DebugLog.Append("Docking Sequence Error!");
                    DockingSequence.EndSequence();
                    break;
            }

            DebugLog.Append($"DockingSequenceIndex: {DockingSequence.CurrentIndex}\n");
        }
        void FollowNearestPlayer()
        {
            DebugLog.Append("yo?\n");
            if (Sensors == null || Sensors.Count < 1)
            {
                TargetGood = false;
                return;
            }
            DebugLog.Append("yo?\n");
            List<MyDetectedEntityInfo> allEntities = new List<MyDetectedEntityInfo>();
            List<MyDetectedEntityInfo> entities = new List<MyDetectedEntityInfo>();

            foreach(IMySensorBlock sensor in Sensors)
            {
                //entities.Clear(); // ????
                sensor.DetectedEntities(entities);
                allEntities.AddRange(entities);
            }

            MyDetectedEntityInfo? nearestPlayer = null;
            double nearestDistance = double.MaxValue;

            foreach(MyDetectedEntityInfo entity in entities)
            {
                if (entity.Type != MyDetectedEntityType.CharacterHuman)
                    continue;

                double nextDistance = Vector3D.Distance(HEAD.WorldMatrix.Translation, entity.Position);

                if (!nearestPlayer.HasValue ||
                    nextDistance < nearestDistance)
                {
                    nearestPlayer = entity;
                    nearestDistance = nextDistance;
                }
            }

            if (!nearestPlayer.HasValue)
            {
                TargetGood = false;
                return;
            }
            DebugLog.Append("yo?\n");
            TargetGood = true;
            Goal.Update(nearestPlayer.Value);
            DebugLog.Append($"NearestPlayer: {nearestPlayer.Value.Name}\n");
        }

        void InitiateMining()
        {
            TargetGood = true;

            CachedStatus = CurrentStatus;

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
            if (CurrentStatus.Task != DroneTask.MINE)
                return;

            if (ProxyCheckLiteral())
            {
                MiningProgress += MINE_LERP_RATE;
                Goal.Matrix.Translation = MiningOrigin + (MiningProgress * (Goal.Matrix.Forward));
            }
            if (MiningProgress == 1)
            {
                TargetGood = false;

                RollBack();
                //CurrentSettings = PreviousSettings;
                //RefreshFlightHardware();
            }
        }

        #endregion

        #region RUN-TIME
        void MessageHandling()
        {
            if (MANUAL_OPERATON)
                return;

            try
            {
                DebugLog.Append($"HasMessage: {DroneEar.HasPendingMessage}\n");

                if (DroneEar.HasPendingMessage)
                {
                    TargetGood = false;

                    while (DroneEar.HasPendingMessage)
                    {
                        RecieveTime = DateTime.Now;
                        TargetGood = DroneRecieve(DroneEar.AcceptMessage()) || TargetGood;
                    }
                    HubSend();
                }

                if (DateTime.Now - RecieveTime > EXP_TIME)
                {
                    DebugLog.Append("Time-Out...\n");
                    RecieveTime = DateTime.Now;
                    TargetGood = false;
                    //Response = false;
                    UnRegister();
                    HubSend();
                }
            }
            catch
            {
                RecieveTime = DateTime.Now;
                TargetGood = false;
                DebugLog.Append("FAIL-POINT!\n");
            }
        }
        string MatrixToStringA(MatrixD matrix, string digits = "")
        {
            return 
                $"R: {matrix.M11.ToString(digits)} : {matrix.M12.ToString(digits)} : {matrix.M13.ToString(digits)}\n" +
                $"U: {matrix.M21.ToString(digits)} : {matrix.M22.ToString(digits)} : {matrix.M23.ToString(digits)}\n" +
                $"F: {matrix.M31.ToString(digits)} : {matrix.M32.ToString(digits)} : {matrix.M33.ToString(digits)}\n" +
                $"T: {matrix.M41.ToString(digits)} : {matrix.M42.ToString(digits)} : {matrix.M43.ToString(digits)}\n" +
                $"S: {matrix.M14.ToString(digits)} : {matrix.M24.ToString(digits)} : {matrix.M34.ToString(digits)}\n" ;
        }

        static string MatrixToStringB(MatrixD matrix, string digits = "#.##")
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
                $"HEAD: {(HEAD == null ? "No head": $"{HEAD.CustomName}")}\n" +
                $"FLIGHT_MODE: {CurrentFlightMode}\n" +
                $"GOOD_TARGET: {TargetGood}\n" +
                //$"MAT_TARGET:\n{MatrixToStringA(Goal.Matrix)}" +
                //$"MAT_CURRENT:\n{MatrixToStringA(CurrentFrame.Matrix)}\n" +
                $"HUB_DRIFT: {Goal.Velocity}\n" +
                $"HUB_CHANNEL: {HubChannel}\n" +
                $"TASK: {CurrentStatus.Task}\n" +
                $"SPEED/SHIELD: {RequestedSettings.MaxSpeed} / {RequestedSettings.ShieldSize}\n" +
                $"TIME_OUT: {EXP_TIME - (DateTime.Now - RecieveTime)}\n" +
                $"===================\n");
        }
        void MyEcho()
        {
            Echo($"Shield Available: {Shield != null}\n" +
                $"Sensor Count: {(Sensors != null ? Sensors.Count.ToString() : "None")}\n" +
                $"Drill Count: {(Drills != null ? Drills.Count.ToString() : "None")}\n" +
                $"Configured: {Configured}");
        }
        void UserArgs(string argument)
        {
            switch (argument)
            {
                case "DEBUG":
                    DEBUG_SYSTEM_INTERRUPT = !DEBUG_SYSTEM_INTERRUPT;
                    break;

                case "TEST":
                    MANUAL_OPERATON = true;
                    MECH_OPERATION = true;
                    SetTask(DroneTask.FOLLOW);
                    break;

                case "DOCK":
                    SetTask(DroneTask.DOCK);
                    break;

                case "CLEAR":
                    MANUAL_OPERATON = false;
                    MECH_OPERATION = false;
                    SetTask(DroneTask.IDLE);
                    break;
            }
        }
        void Run()
        {
            DebugLog.Append("Running Task...\n");

            switch (CurrentStatus.Task)
            {
                case DroneTask.IDLE:
                    Idle();
                    break;

                case DroneTask.HAUL:
                    break;

                case DroneTask.FORM:
                    break;

                case DroneTask.FOLLOW:
                    FollowNearestPlayer();
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

            DebugLog.Append("Running Flight...\n");

            switch (CurrentFlightMode)
            {
                case FlightMode.STANDBY:
                    break;

                case FlightMode.NAV:
                    NavMode();
                    break;

                case FlightMode.FLIGHT_AI:
                    FlightAI();
                    break;

                case FlightMode.MECH_AI:
                    MechAI();
                    break;
            }

            RefreshShields();
        }

        void MechStateHandler()
        {
            if (!MECH_OPERATION)
                return;

            bool gyroOn = false;

            foreach (IMyGyro gyro in Gyros)
                if (gyro.Enabled)
                    gyroOn = true;

            if (gyroOn && CurrentFlightMode != FlightMode.FLIGHT_AI)
                SetFlightMode(FlightMode.FLIGHT_AI);

            if (!gyroOn && CurrentFlightMode != FlightMode.MECH_AI)
                SetFlightMode(FlightMode.MECH_AI);
        }

        #endregion

        #region ENTRY-POINTS
        public Program()
        {
            PROG = this;
            Runtime.UpdateFrequency = UpdateFrequency.Update1;
            DebugLog = new StringBuilder();

            DroneSurface = Me.GetSurface(0);
            DroneSurface.ContentType = ContentType.TEXT_AND_IMAGE;
            DroneSurface.WriteText("", false);

            try
            {
                RefreshDroneEquipment();
                SetupNetwork();

                FollowEye = GridTerminalSystem.GetBlockWithName(FollowEyeName);

                biggestSelfHalfExtent = 1;
                for (int i = 0; i < 3; i++)
                    biggestSelfHalfExtent = biggestSelfHalfExtent > Me.WorldAABB.HalfExtents.GetDim(i) ? biggestSelfHalfExtent : Me.WorldAABB.HalfExtents.GetDim(i);
                //Load();

                DebugLog.Append("Load Complete!\n");
                Configured = true;
                DebugLog.Append("Setup Complete!\n");
            }
            catch
            {
                DebugLog.Append("FAIL-POINT!\n");
                Configured = false;
            }
            DroneSurface.WriteText(DebugLog);
        }
        public void Main(string argument, UpdateType updateSource)
        {
            if (!Configured)
                return;

            Debugging();
            if (DEBUG_BREAK)
                return;

            CurrentFrame.Update(HEAD);
            UserArgs(argument);
            MechStateHandler();
            MessageHandling();
            Run();
        }
        public void Save()
        {
            Storage = $"{(int)CurrentStatus.Task}:{(int)CurrentFlightMode}:{RecieveTime}";
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
