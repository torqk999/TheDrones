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

        #region COMBO

        #region DEFS

        //const string SpherePBName = "SPHERE_GEN";
        const string DockIDtag = "[DOCK]";
        //const string OutChannel = "DRONE_REMOTE";
        const string InChannel = "DRONE_HUB";
        const string SwarmChannel = "DRONE_SWARM";
        const string SourceName = "HUB_CONTROL";
        const string HubPBname = "HUB_PB";
        const string ShipControlName = "HUB_CONTROL";

        //const char Split = ':';
        const float DockDisplacement = 5;
        const float Radius = 210;
        const float Distance = 50;
        const int LiteralCount = 4;
        const int FORM_SCALE_LIMIT = 5;
        const int MAX_MSG_COUNT = 20;
        const int MAX_ASSIGN_COUNT = 10;
        //const double OFFSET_INCREMENT = 0.0004d;

        #endregion

        #region FORMATION

        bool Literal = false;
        bool Spin = false;
        //double OffsetAngle = 0;
        int FormationScalePow;  
        float FormationScaleVal;

        // SpinProfiling
        double Origin = 0;
        double AngleBuffer;
        DateTime ZeroPoint;
        TimeSpan Interval = TimeSpan.FromMinutes(3);

        IMyTextSurface PBscreen;
        Vector3[] EquatorDeltas;
        Vector3[] SphereDeltas;
        Point[][] SuperDeltas;
        Vector3[] VanguardDeltas;
        Vector3[] FormationLiterals;
        Vector3[] FormationBuffer = new Vector3[2]; // 0 = new, 1 = old

        IMyShipController Control;
        IMyTerminalBlock Source;

        public struct Point
        {
            public int PullIndex;
            public Vector3 Delta;

            public Point(int pullIndex, Vector3 delta)
            {
                PullIndex = pullIndex;
                Delta = delta;
            }
        }

        void GenerateLatitudeSphereDeltas(float radius, float distance, bool directional = false)
        {
            List<Vector3> points = new List<Vector3>();

            int ringCount = (int)((Math.PI * radius) / distance);
            string debug = string.Empty;

            for (int i = 0; i <= ringCount; i++)
            {
                double ringRadius = Math.Sin(((double)i / ringCount) * Math.PI) * radius;
                double deltaY = Math.Cos(((double)i / ringCount) * Math.PI) * radius;

                int pointCount = (int)((2 * Math.PI * ringRadius) / distance);
                pointCount = (pointCount < 1) ? 1 : pointCount;

                for (int j = 0; j < pointCount; j++)
                {
                    double deltaZ = Math.Sin(((double)j / pointCount) * (2 * Math.PI)) * ringRadius;
                    double deltaX = Math.Cos(((double)j / pointCount) * (2 * Math.PI)) * ringRadius;

                    if (directional)
                        points.Add(new Vector3(deltaX, deltaZ, -deltaY));
                    else
                        points.Add(new Vector3(deltaX, deltaY, deltaZ));
                }
            }

            SphereDeltas = new Vector3[points.Count];
            FormationLiterals = new Vector3[points.Count];
            points.CopyTo(SphereDeltas);
        }
        void GenerateSuperSphereDeltas(float radius, float distance, bool directional = false)
        {
            List<List<Vector3>> rings = new List<List<Vector3>>();

            int ringCount = (int)((Math.PI * radius) / distance);
            string debug = string.Empty;

            for (int i = 0; i <= ringCount; i++)
            {
                List<Vector3> points = new List<Vector3>();

                double ringRadius = Math.Sin(((double)i / ringCount) * Math.PI) * radius;
                double deltaY = Math.Cos(((double)i / ringCount) * Math.PI) * radius;

                int pointCount = (int)((2 * Math.PI * ringRadius) / distance);
                pointCount = (pointCount < 1) ? 1 : pointCount;

                for (int j = 0; j < pointCount; j++)
                {
                    double deltaZ = Math.Sin(((double)j / pointCount) * (2 * Math.PI)) * ringRadius;
                    double deltaX = Math.Cos(((double)j / pointCount) * (2 * Math.PI)) * ringRadius;

                    if (directional) // face forward
                        points.Add(new Vector3(deltaX, -deltaZ, deltaY));
                    else // face down
                        points.Add(new Vector3(deltaX, deltaY, deltaZ));
                }

                rings.Add(points);
            }


            SuperDeltas = new Point[rings.Count][];
            int literalIndexOffset = -1;

            for (int i = rings.Count - 1; i > -1; i--) // work backwards through rings
            {
                SuperDeltas[i] = new Point[rings[i].Count];
                for (int j = 0; j < rings[i].Count; j++)
                {
                    SuperDeltas[i][j].Delta = rings[i][j];
                    SuperDeltas[i][j].PullIndex = -1;

                    if (i >= rings.Count - 1)
                        continue;

                    int shortestPullIndex = literalIndexOffset;
                    float shortestPullDelta = Vector3.Distance(SuperDeltas[i][j].Delta, SuperDeltas[i + 1][0].Delta);

                    for (int k = 1; k < rings[i + 1].Count; k++)
                    {
                        float currentPullDelta = Vector3.Distance(SuperDeltas[i][j].Delta, SuperDeltas[i + 1][k].Delta);

                        if (currentPullDelta < shortestPullDelta)
                        {
                            shortestPullDelta = currentPullDelta;
                            shortestPullIndex = k + literalIndexOffset;
                        }
                    }
                    SuperDeltas[i][j].PullIndex = shortestPullIndex;
                }

                literalIndexOffset += rings[i].Count;
            }

        }
        void GenerateEquatorDeltas(float radius, float distance)
        {
            List<Vector3> points = new List<Vector3>();

            int pointCount = Literal ? LiteralCount : (int)((Math.PI * 2 * radius) / distance);

            for (int i = 0; i < pointCount; i++)
            {
                double deltaZ = Math.Sin(((double)i / pointCount) * (2 * Math.PI)) * radius;
                double deltaX = Math.Cos(((double)i / pointCount) * (2 * Math.PI)) * radius;

                points.Add(new Vector3(deltaX, 0, deltaZ));
            }

            EquatorDeltas = new Vector3[points.Count];
            points.CopyTo(EquatorDeltas);
        }
        void GenerateThreePointVanguardDeltas(float radius, float altitude)
        {
            VanguardDeltas = new Vector3[6];

            for (int i = 0; i < 6; i++)
            {
                double ratio = (((double)i / 3) + ((double)1 / 6));
                double deltaZ = Math.Sin(ratio * Math.PI) * radius;
                double deltaX = Math.Cos(ratio * Math.PI) * radius;

                VanguardDeltas[i] = new Vector3(deltaX, altitude, deltaZ);
            }
        }
        Vector3 DeTransformVectorRelative(MatrixD S, Vector3 N)
        {
            Vector3D DV = new Vector3D();

            /*
                x = Xa + Yd + Zg
                y = Xb + Ye + Zh
                z = Xc + Yf + Zi 
             */

            DV.X = (N.X * S.M11) + (N.Y * S.M21) + (N.Z * S.M31);

            DV.Y = (N.X * S.M12) + (N.Y * S.M22) + (N.Z * S.M32);

            DV.Z = (N.X * S.M13) + (N.Y * S.M23) + (N.Z * S.M33);

            return DV;
        }
        void GenerateFormationLiterals(ref IMyTerminalBlock source, ref Vector3[] formationDeltas, ref Vector3[] targetArray)
        {
            if (source == null)
                return;

            if (formationDeltas == null ||
                formationDeltas.Count() == 0)
                return;

            if (targetArray == null || targetArray.Count() != formationDeltas.Count())
                targetArray = new Vector3[formationDeltas.Count()];

            //string data = string.Empty;
            //Me.CustomData = string.Empty;

            for (int i = 0; i < formationDeltas.Length; i++)
            {
                //Vector3 newVector = Deltas[i] + source.GetPosition(); // Raw world space formation
                Vector3 scaledVector = new Vector3(formationDeltas[i].X * FormationScaleVal,
                                                   formationDeltas[i].Y * FormationScaleVal,
                                                   formationDeltas[i].Z * FormationScaleVal);

                AngleBuffer = Origin;
                if (Spin)
                    AngleBuffer += ReturnTimedAngle();

                Vector3 relativeVector = DeTransformVectorRelative(GenerateOffsetMatrix(ref AngleBuffer), scaledVector);
                //Vector3 relativeVector = DeTransformVectorRelative(source.WorldMatrix, scaledVector);
                Vector3 newVector = relativeVector + source.GetPosition();

                targetArray[i] = newVector;

                //Me.CustomData += $"{newVector.X}{Split}{newVector.Y}{Split}{newVector.Z}";
                //Me.CustomData += (i < formationDeltas.Length - 1) ? "\n" : "";
            }
            //Me.CustomData = data;
        }
        bool GenerateFormationLiteral(ref IMyTerminalBlock source, ref Vector3[] formationDeltas, int index, ref Vector3 target)
        {
            if (source == null)
                return false;

            if (formationDeltas == null ||
                formationDeltas.Count() == 0)
                return false;

            if (target == null)
                return false;

            Vector3 scaledVector = new Vector3(formationDeltas[index].X * FormationScaleVal,
                                               formationDeltas[index].Y * FormationScaleVal,
                                               formationDeltas[index].Z * FormationScaleVal);

            AngleBuffer = Origin;
            if (Spin)
                AngleBuffer += ReturnTimedAngle();

            Vector3 relativeVector = DeTransformVectorRelative(GenerateOffsetMatrix(ref AngleBuffer), scaledVector);
            target = relativeVector + source.GetPosition();

            return true;
        }

        void GenerateSuperLiterals(IMyTerminalBlock source, Point[][] formationDeltas)
        {
            //string data = string.Empty;

            for (int i = 0; i < SuperDeltas.Length; i++)
            {
                for (int j = 0; j < SuperDeltas[i].Length; j++)
                {
                    Vector3 scaledVector = new Vector3(formationDeltas[i][j].Delta.X * FormationScaleVal,
                                                       formationDeltas[i][j].Delta.Y * FormationScaleVal,
                                                       formationDeltas[i][j].Delta.Z * FormationScaleVal);
                    Vector3 relativeVector = DeTransformVectorRelative(source.WorldMatrix, scaledVector);
                    Vector3 newVector = relativeVector + source.GetPosition();

                    //data += $"{newVector.X}{Split}{newVector.Y}{Split}{newVector.Z}{Split}{formationDeltas[i][j].PullIndex}";
                    //data += (i < formationDeltas.Length - 1) ? "\n" : "";
                }
            }
            //Me.CustomData = data;
        }

        void ToggleSpin(bool spin = true)
        {
            Spin = spin;

            if (Spin)
                ZeroPoint = DateTime.Now;

            else
                Origin = ReturnTimedAngle();
        }

        void ScaleFormation(bool increase)
        {
            if (increase && FormationScalePow < FORM_SCALE_LIMIT)
                FormationScalePow++;
            if (!increase && FormationScalePow > 0)
                FormationScalePow--;
            SetScaleValue();
        }

        void SetScaleValue()
        {
            FormationScaleVal = (float)Math.Pow((double)1.1, FormationScalePow);
        }
        /*
        void UpdateSpinOffset()
        {
            if (!Spin)
                return;

            OffsetAngle += OFFSET_INCREMENT;
            OffsetAngle = (OffsetAngle >= (2 * Math.PI)) ? 0 : OffsetAngle;
        }
        */
        double ReturnTimedAngle()
        {
            DateTime current = DateTime.Now;
            TimeSpan length = (current - ZeroPoint);
            while (length >= Interval)
            {
                ZeroPoint += Interval;
                length = (current - ZeroPoint);
            }
            return (length.TotalSeconds / Interval.TotalSeconds) * (2 * Math.PI);
        }

        MatrixD GenerateOffsetMatrix(ref double radians)
        {
            /* 
             * Keen Implementation:
             * Row 1: Right.x , Right.y , Right.z
             * Row 2: Up.x    , Up.y    , Up.z
             * Row 3: Back.x  , Back.y  , Back.z
            */

            double deltaZ = Math.Sin(radians);
            double deltaX = Math.Cos(radians);

            MatrixD output = new MatrixD(
                deltaX, 0, deltaZ,
                0, 1, 0,
                -deltaZ, 0, deltaX);

            return output;
        }

        #endregion

        #region HUB

        IMyTextPanel HubDebug;
        IMyBroadcastListener HubEar;
        IMyLaserAntenna[] POLES = new IMyLaserAntenna[2];

        Random RNG;
        TimeSpan Expiration = TimeSpan.FromMinutes(1);
        double[] MSG_BUFFER = new double[9];



        List<PendingRequest> ResponseQue = new List<PendingRequest>();

        List<DroneRegistry> Registered = new List<DroneRegistry>();
        List<DroneRegistry> Available = new List<DroneRegistry>();

        List<DroneFormSlot> AllSlots = new List<DroneFormSlot>();
        List<DroneFormSlot> OpenSlots = new List<DroneFormSlot>();

        List<DockingPort> Docks = new List<DockingPort>();

        bool bForwardV = false;
        bool bShunt = false;
        bool bSwarmState = false;
        bool bTrueFlight = false;
        bool bConfigured;
        bool bInboxActive = false;

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

        public struct PendingRequest
        {
            public long ID;
            public DroneRequest Request;

            public PendingRequest(long iD, DroneRequest request)
            {
                ID = iD;
                Request = request;
            }
        }
        class DroneRegistry
        {
            public long ID;
            public DroneFormSlot Slot;
            public DockingPort Port;
            public DateTime Registration;
            public DateTime LastCall;
            public DateTime LastOperation;

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
                ToggleSlot(newRandomSlot, true);
            return newRandomSlot;
        }
        bool Assignment(PendingRequest pending)
        {
            //int offset = (bForwardV) ? drone.Index : 5 - drone.Index;   // Formation switching rough implementation

            DroneRegistry drone = Registered.Find(x => x.ID == pending.ID);
            if (drone == null)
                return false;

            drone.LastCall = DateTime.Now;

            switch (pending.Request)
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

                    if (drone.Port != null)
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
            //string[] raw0 = SpherePB.CustomData.Split('\n')[droneIndex].Split(Split);
            //double[] raw1 = new double[9];

            //MSG_BUFFER[0] = FormationLiterals[droneIndex].X;
            //MSG_BUFFER[1] = FormationLiterals[droneIndex].Y;
            //MSG_BUFFER[2] = FormationLiterals[droneIndex].Z;

            GenerateFormationLiteral(ref Source, ref SphereDeltas, droneIndex, ref FormationBuffer[0]);

            MSG_BUFFER[0] = FormationBuffer[0].X;
            MSG_BUFFER[1] = FormationBuffer[0].Y;
            MSG_BUFFER[2] = FormationBuffer[0].Z;

            Vector3 pos = Control.GetPosition();
            MSG_BUFFER[3] = pos.X;
            MSG_BUFFER[4] = pos.Y;
            MSG_BUFFER[5] = pos.Z;

            Vector3 drift = Control.GetShipVelocities().LinearVelocity;
            MSG_BUFFER[6] = drift.X;
            MSG_BUFFER[7] = drift.Y;
            MSG_BUFFER[8] = drift.Z;

            return ImmutableArray.Create(MSG_BUFFER);
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
            //string[] raw0 = SpherePB.CustomData.Split('\n')[AllSlots.Count - 1].Split(Split);
            //double[] raw1 = new double[9];

            //MSG_BUFFER[0] = FormationLiterals[AllSlots.Count - 1].X;
            //MSG_BUFFER[1] = FormationLiterals[AllSlots.Count - 1].Y;
            //MSG_BUFFER[2] = FormationLiterals[AllSlots.Count - 1].Z;

            GenerateFormationLiteral(ref Source, ref SphereDeltas, AllSlots.Count - 1, ref FormationBuffer[0]);

            MSG_BUFFER[0] = FormationBuffer[0].X;
            MSG_BUFFER[1] = FormationBuffer[0].Y;
            MSG_BUFFER[2] = FormationBuffer[0].Z;

            Vector3 pos = Control.GetPosition();
            MSG_BUFFER[3] = pos.X;
            MSG_BUFFER[4] = pos.Y;
            MSG_BUFFER[5] = pos.Z;

            Vector3 drift = Control.GetShipVelocities().LinearVelocity;
            MSG_BUFFER[6] = drift.X;
            MSG_BUFFER[7] = drift.Y;
            MSG_BUFFER[8] = drift.Z;

            return ImmutableArray.Create(MSG_BUFFER);
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
        void SwarmTrueFlightToggle()
        {
            bTrueFlight = !bTrueFlight;
            IGC.SendBroadcastMessage(SwarmChannel, bTrueFlight ? 9 : 10);
        }
        void DroneSingletonHandler()
        {
            if (!HubEar.HasPendingMessage)
                return;

            ImmutableArray<long> data = (ImmutableArray<long>)HubEar.AcceptMessage().Data;
            DroneRegistry drone = Registered.Find(x => x.ID == data.ItemRef(0));

            if (drone == null)
                drone = RegistrationRNG(data.ItemRef(0));

            Assignment(new PendingRequest(drone.ID, (DroneRequest)data.ItemRef(1)));
        }
        void DroneCallsHandler()
        {
            ImmutableArray<long> data = (ImmutableArray<long>)HubEar.AcceptMessage().Data;
            DroneRegistry drone = Registered.Find(x => x.ID == data.ItemRef(0));

            if (drone == null)
                drone = RegistrationRNG(data.ItemRef(0));

            int index = ResponseQue.FindIndex(x => x.ID == drone.ID);
            if (index < 0)
                ResponseQue.Add(new PendingRequest(drone.ID, (DroneRequest)data.ItemRef(1)));

            //Assignment(drone, (DroneRequest)data.ItemRef(1));
        }
        void DroneAssingmentsHandler()
        {
            for (int i = 0; i < MAX_ASSIGN_COUNT && i < ResponseQue.Count; i++)
            {
                Assignment(ResponseQue[0]);
                ResponseQue.RemoveAt(0);
            }
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
            //Debug.WriteText("Cleaning Pool...\n", false);
            for (int i = Registered.Count - 1; i > -1; i--)
            {
                //Debug.WriteText($"RegIndex: {i}\n", true);
                if (DateTime.Now - Registered[i].LastCall > Expiration)
                {
                    ToggleSlot(Registered[i].Slot);
                    if (Registered[i].Port != null)
                        Registered[i].Port.Occupied = false;
                    Registered.RemoveAt(i);
                }
            }
        }
        void TrickleFormation()
        {
            //string output = "Trickle\n";

            for (int i = 0; i < AllSlots.Count - 1; i++)
            {
                //output += $"{i}:{AllSlots[i].Occupied}\n";

                if (AllSlots[i].Occupied ||
                    !AllSlots[i + 1].Occupied)
                    continue;

                DroneRegistry pullReg = Registered.Find(x => x.Slot == AllSlots[i + 1]);

                if (pullReg == null)
                    continue;

                ToggleDroneAssignment(pullReg, AllSlots[i]);
            }

            //Debug.WriteText(output);
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
            //SpherePB = GridTerminalSystem.GetBlockWithName(SpherePBName);
            try
            {
                //string[] raw0 = SpherePB.CustomData.Split('\n');
                //FormationPointsCount = raw0.Length;     // / 2; // ROUGH HACK (divide by 2)!!!

                AllSlots.Clear();
                OpenSlots.Clear();
                //for (int i = 0; i < FormationPointsCount; i++)
                for (int i = 0; i < FormationLiterals.Length; i++)
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
        #endregion

        #region ENTRY-POINTS
        public Program()
        {
            Control = (IMyShipController)GridTerminalSystem.GetBlockWithName(ShipControlName);
            Source = GridTerminalSystem.GetBlockWithName(SourceName);

            HubDebug = (IMyTextPanel)GridTerminalSystem.GetBlockWithName("HubDebug");

            Me.CustomName = HubPBname;
            PBscreen = Me.GetSurface(0);
            PBscreen.ContentType = ContentType.TEXT_AND_IMAGE;
            PBscreen.WriteText("", false);

            SetScaleValue();
            //GenerateThreePointVanguardDeltas(50, -10);    // Migrate user constants!!!
            GenerateLatitudeSphereDeltas(Radius, Distance, true);
            //GenerateSuperSphereDeltas(Radius, Distance, true);
            //GenerateEquatorDeltas(Radius, Distance);
            bConfigured = GeneratePool();
            if (bConfigured)
                Load();
            GenerateDocks();
            HubEar = IGC.RegisterBroadcastListener(InChannel);
            HubEar.SetMessageCallback();
            //Runtime.UpdateFrequency = UpdateFrequency.Update10;
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
                    Registered.Clear();
                    ResponseQue.Clear();
                    Storage = string.Empty;
                    break;

                case "INCREASE":
                    ScaleFormation(true);
                    break;

                case "DECREASE":
                    ScaleFormation(false);
                    break;

                case "SPIN":
                    ToggleSpin(!Spin);
                    break;

                case "TRUE":
                    SwarmTrueFlightToggle();
                    break;
            }

            CleanupPool();

            DroneSingletonHandler();

            /*
            bInboxActive = false;
            int count = 0;
            while (HubEar.HasPendingMessage)
            {
                bInboxActive = true;
                DroneCallsHandler();
                count++;
            }

            if (bInboxActive)
                return;

            DroneAssingmentsHandler();
            */

            if (bShunt)
                TrickleFormation();

            if (Source != null)
            {
                //UpdateSpinOffset();
                //GenerateFormationLiterals(Source, ref SphereDeltas, ref FormationLiterals);
                PBscreen.WriteText($"Point Coint:{FormationLiterals.Count()}\n" +
                    $"Spin: {Spin}\n" +
                    //$"Offset: {OffsetAngle}\n" +
                    $"DroneSlots: {AllSlots.Count}\n" +
                    $"OpenSlots: {OpenSlots.Count}\n" +
                    $"Registered: {Registered.Count}\n" +
                    $"Configured?: {bConfigured}\n" +
                    $"State: {bSwarmState}\n" +
                    $"Mode: {SwarmMode}\n" +
                    $"TrueFlight: {bTrueFlight}\n" +
                    $"ResponseQueSize: {ResponseQue.Count}");
                //GenerateSuperLiterals(Control, SuperDeltas);
            }
        }
        public void Save()
        {
            Storage = $"{Spin}:{Origin}\n";
            Storage += $"{Registered.Count}\n";
            foreach (DroneRegistry reg in Registered)
                Storage += reg.Save() + "\n";
            Storage += $"{ResponseQue.Count}\n";
            foreach (PendingRequest request in ResponseQue)
                Storage += $"{request.ID}:{(int)request.Request}\n";

            HubDebug.WriteText(Storage);
        }       
        public void Load()
        {
            try
            {
                string[] raw0 = Storage.Split('\n');

                string[] raw1 = raw0[0].Split(':');
                Spin = bool.Parse(raw1[0]);
                Origin = double.Parse(raw1[1]);
                int regCount = int.Parse(raw0[1]);
                int pendCountIndex = regCount + 2;
                int pendCount = int.Parse(raw0[pendCountIndex]);

                for (int i = 2; i < (regCount + 2); i++)
                {
                    string[] raw2 = raw0[i].Split(':');
                    long id = long.Parse(raw2[0]);
                    int slotIndex = int.Parse(raw2[1]);
                    DroneRegistry reg = Registered.Find(x => x.ID == id);
                    if (reg==null)
                    {
                        reg = new DroneRegistry(id, AllSlots[slotIndex]);
                        Registered.Add(reg);
                    }
                    ToggleSlot(AllSlots.Find(x => x.FormIndex == slotIndex), true);
                    //OpenSlots.Remove(AllSlots.Find(x => x.FormIndex == slotIndex));
                }

                for (int i = pendCountIndex + 1; i < (pendCount + regCount + 3); i++)
                {
                    string[] raw2 = raw0[i].Split(':');
                    PendingRequest request = new PendingRequest(long.Parse(raw2[0]), (DroneRequest)int.Parse(raw2[1]));
                    ResponseQue.Add(request);
                }

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
