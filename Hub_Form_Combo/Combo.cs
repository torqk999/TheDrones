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
        /*#region TEST

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

        IMyTextSurface Surface;
        StringBuilder Builder = new StringBuilder();

        public Program()
        {
            Surface = Me.GetSurface(0);
            Surface.ContentType = ContentType.TEXT_AND_IMAGE;
            bool[] EX = new bool[]
            {
                true,
                false,
                false,
                true,
                true,
                false,
                false,
                true
            };
            byte ex = 0;
            for (int i = 0; i < EX.Length; i++)
                Builder.Append($"{i}:{EX[i]}\n");
            GetByte(out ex, EX);
            Builder.Append($"result byte: {ex}\n");
            GetBools(ex, EX);
            Builder.Append("Conversion:\n");
            for (int i = 0; i < EX.Length; i++)
                Builder.Append($"{i}:{EX[i]}\n");
            Surface.WriteText(Builder);
        }

        public void Main(string argument, UpdateType updateSource)
        {

        }

        #endregion*/




        #region COMBO

        #region DEFS

        const string DockIDtag = "[DOCK]";
        const string HubPanelGroup = "HUB_LCDS";
        const string LostChannel = "NEED_HUB";
        const string HubChannel = "[TORQK]";
        const string HubSwarmChannel = "[ALPHA]";
        const string HubPBname = "HUB_PB";
        const string ShipControlName = "HUB_CONTROL";

        const float DockDisplacement = 5;
        const float Radius = 30;
        const float Distance = 10;

        const int LiteralCount = 4;
        const int OUT_BOX_MAX = 16;
        const int FORM_SCALE_LIMIT = 5;
        const int MAX_MSG_COUNT = 20;
        const int MAX_ASSIGN_COUNT = 10;
        const int PROC_HOLD = 100;

        static TimeSpan EXP_TIME = TimeSpan.FromMinutes(1);
        #endregion

        #region GUI VARS

        public interface IGUIListing
        {
            string ReturnListing();
            int ReturnIndex();
            GUILayer ReturnLayer();
        }
        public enum Screen
        {
            BUTTONS = 0,
            GROUP = 3,
            SLOT = 2,
            DRONE = 1,
            SPLASH = 6,
            DEBUG_STATIC = 4,
            DEBUG_STREAM = 5
        }
        public enum GUILayer
        {
            GROUP = 0,
            SLOT = 1,
            DRONE = 2
        }
        public enum GUINav
        {
            SCROLL_UP,
            SCROLL_DOWN,
            UP,
            DOWN,
            BACK,
            SELECT
        }

        IMyTextSurface PBscreen;
        IMyTextSurface[] CockPitScreens = new IMyTextSurface[3];
        IMyTextSurface[] Screens = new IMyTextSurface[Enum.GetNames(typeof(Screen)).Length];

        string[] FORM_BUFF;
        string[] TICK_BUFF = new string[20];
        StringBuilder DisplayBuilder = new StringBuilder();
        StringBuilder DebugStream = new StringBuilder();
        StringBuilder DebugStatic = new StringBuilder();

        GUILayer CurrentGUILayer = GUILayer.GROUP;
        bool CapLines = true;
        int LineCount = 0;
        int CursorIndex = 0;
        int HalfScreenBufferSize = 7;
        //const char Split = ':';
        int[] CharBuffers = new int[Enum.GetValues(typeof(Screen)).Length];
        int[] SelObjIndex = new int[Enum.GetValues(typeof(GUILayer)).Length];
        int[] LastLibraryInput = new int[2];
        int[] LastLibraryClock = new int[2];

        static readonly string[] GroupActions = new string[]
        {
            "Idle",
            "Form"
        };
        static readonly string[] SlotActions = new string[]
        {

        };
        static readonly string[] DroneActions = new string[]
        {

        };
        static readonly string[] InputLabels =
        {
            "w - Scroll Up",
            "s - Scroll Down",
            "a - Back Step",
            "d - Enter"
        };
        static readonly string[] Cursor = { "  ", "->" };
        static readonly string[][] MenuButtons = new string[][]
        {
            GroupActions,
            SlotActions,
            DroneActions
        };
        #endregion

        #region GUI METHODS

        void Debugging()
        {
            if (HubControl == null)
                return;

            DebugStream.Append($"Point Coint:{SphereDeltas.Count()}\n" +
                $"DroneSlots: {Groups[SelObjIndex[0]].AllSlots.Count}\n" +
                $"OpenSlots: {Groups[SelObjIndex[0]].OpenSlots.Count}\n" +
                $"Registered: {Registered.Count}\n" +
                $"ResponseQueSize: {ResponseQue.Count}\n" +
                $"MSG_COUNTS: {MSG_COUNT} : {LOST_EAR_COUNT} : {DRONE_EAR_COUNT}\n");

            Screens[(int)Screen.DEBUG_STREAM].WriteText(DebugStream);
            DebugStream.Clear();
            DebugStream.Append(">>> DEBUG STREAM >>>\n");
        }
        void DebugTicker()
        {
            try
            {
                IMyTextSurface tick = Screens[(int)Screen.DEBUG_STATIC];
                tick.WriteText("");

                for (int i = 1; i < TICK_BUFF.Length; i++)
                {
                    TICK_BUFF[i - 1] = TICK_BUFF[i];
                    tick.WriteText(TICK_BUFF[i], true);
                }
                TICK_BUFF[TICK_BUFF.Length - 1] = DebugStatic.ToString();
                tick.WriteText(TICK_BUFF[TICK_BUFF.Length - 1], true);
                DebugStream.Append("Ticker Success!\n");
            }
            catch { DebugStream.Append("Ticker Fail!\n"); }

            DebugStatic.Clear();
        }
        void MenuInput()
        {
            if (HubControl == null)
                return;

            int z = (int)HubControl.MoveIndicator.Z;
            int x = (int)HubControl.MoveIndicator.X;

            DebugStream.Append($"input: {z},{x}\n" +
                $"last: {LastLibraryInput[0]},{LastLibraryInput[1]}\n" +
                $"CurrentLayer: {CurrentGUILayer}\n" +
                $"CursorIndex: {CursorIndex}\n" +
                $"StringCount: {LineCount}\n" +
                $"LineBufferSize: {HalfScreenBufferSize}\n" +
                $"AvailableCount: {Available.Count}\n");

            if (LastLibraryInput[0] == z)
                LastLibraryClock[0]++;

            if (LastLibraryClock[0] > PROC_HOLD ||
                LastLibraryInput[0] != z)
            {
                LastLibraryInput[0] = z;
                if (z != 0)
                {
                    GUINav nav = z > 0 ? GUINav.UP : GUINav.DOWN;
                    GUINavigation(nav);
                }
                else
                    LastLibraryClock[0] = 0;
            }

            if (LastLibraryClock[1] > PROC_HOLD ||
                LastLibraryInput[1] != x)
            {
                LastLibraryInput[1] = x;
                if (x != 0)
                {
                    GUINav nav = x < 0 ? GUINav.BACK : GUINav.SELECT;
                    GUINavigation(nav);
                }
                else
                    LastLibraryClock[1] = 0;
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
        void AppendLibraryItem(IGUIListing listing, Screen screen, bool newLine = true)
        {
            string entry = string.Empty;
            if (listing == null)
            {
                entry = "ERROR: NULL LISTING!";
            }
            else
            {
                int cursor;

                if (CurrentGUILayer == listing.ReturnLayer() &&
                    SelObjIndex[(int)listing.ReturnLayer()] == listing.ReturnIndex())
                {
                    CursorIndex = LineCount + 1; // -1 for indice, +2 for header lines
                    cursor = 1;
                }
                else
                    cursor = 0;

                entry = $"{Cursor[cursor]}| {listing.ReturnIndex()} | {listing.ReturnListing()}";
            }

            entry += newLine ? "\n" : " | ";
            DisplayBuilder.Append(entry);
            CharBuffers[(int)screen] = entry.Length > CharBuffers[(int)screen] ? entry.Length : CharBuffers[(int)screen];
            LineCount = newLine ? LineCount + 1 : LineCount;
        }
        void RefreshIndex(int layer, int count)
        {
            if (count < 1)
            {
                SelObjIndex[layer] = -1;
                return;
            }

            if (SelObjIndex[layer] < 0)
            {
                SelObjIndex[layer] = count - 1;
                return;
            }

            if (SelObjIndex[layer] >= count)
            {
                SelObjIndex[layer] = 0;
                return;
            }
        }
        /// <summary>
        /// Design This!
        /// </summary>
        void SplashBuilder()
        {
            try
            {
                DisplayBuilder.Clear();
                if (Groups.Count < 1)
                {
                    DisplayBuilder.Append("Nothing to Select!\n");
                    Screens[(int)Screen.SPLASH].WriteText(DisplayBuilder);
                    return;
                }

                DroneSlotGroup group = Groups[SelObjIndex[(int)GUILayer.GROUP]];
                DroneSlot slot = group.AllSlots.Count > 0 ? group.AllSlots[SelObjIndex[(int)GUILayer.SLOT]] : null;
                DroneRegistry drone = Registered.Count > 0 ? Registered[SelObjIndex[(int)GUILayer.DRONE]] : null;

                switch (CurrentGUILayer)
                {

                    case GUILayer.GROUP:
                        DisplayBuilder.Append($"{group.ReturnIndex()}:{group.ReturnListing()}\n");
                        DisplayBuilder.Append($"SlotCount: {group.AllSlots.Count} | MemberCount: {group.Members.Count}\n");
                        DisplayBuilder.Append(//$"Position: {group.CurrentOrigin.Position}\n" +
                            $"Matrix: {MatrixToString(group.CurrentOrigin.Matrix, "#.##")}");
                        if (group is DroneSwarm)
                        {
                            DroneSwarm swarm = (DroneSwarm)group;
                            DisplayBuilder.Append($"Current Swarm Task: {swarm.Settings.Task}\n");
                        }

                        break;

                    case GUILayer.SLOT:
                        if (slot == null)
                        {
                            DisplayBuilder.Append("Nothing to Select!\n");
                            break;
                        }
                        DisplayBuilder.Append($"{slot.ReturnIndex()}:{slot.ReturnListing()}\n");
                        // + $"Delta: {slot.Status.Position}\n");
                        DisplayBuilder.Append($"{(slot.Occupant == null ? "VACANT" : "OCCUPIED")}\n");
                        if (slot.Occupant != null)
                            DisplayBuilder.Append($"{slot.Occupant.ReturnListing()}\n");
                        break;

                    case GUILayer.DRONE:
                        if (drone == null)
                        {
                            DisplayBuilder.Append("Nothing to Select!\n");
                            break;
                        }
                        DisplayBuilder.Append($"| {drone.ReturnIndex()} |{drone.ReturnListing()}\n");
                        DisplayBuilder.Append($"{drone.LastReport.Matrix.Translation} || {drone.LastReport.TimeStamp}\n");
                        break;
                }

                Screens[(int)Screen.SPLASH].WriteText(DisplayBuilder);
            }
            catch { };
        }

        void GroupListBuilder()
        {
            CursorIndex = 0;
            LineCount = 0;
            DisplayBuilder.Clear();
            DisplayBuilder.Append($"Groups: ({SelObjIndex[(int)GUILayer.GROUP]}/{Groups.Count})\n================\n");

            foreach (DroneSlotGroup group in Groups)
                AppendLibraryItem(group, Screen.GROUP);

            FormattedStringBuilder();

            Screens[(int)Screen.GROUP].WriteText(DisplayBuilder);
        }
        void SlotListBuilder()
        {
            CursorIndex = 0;
            LineCount = 0;
            DisplayBuilder.Clear();
            int G = SelObjIndex[(int)GUILayer.GROUP];
            int count = Groups[G] != null ? Groups[G].AllSlots.Count : -1;
            string denom = count < 0 ? "NA" : count.ToString();
            DisplayBuilder.Append($"Slots: ({SelObjIndex[(int)GUILayer.SLOT]}/{denom})\n================\n");

            foreach (DroneSlot slot in Groups[G].AllSlots)
            {
                AppendLibraryItem(slot, Screen.SLOT, false);
                if (slot != null)
                {
                    if (slot.Occupant != null)
                        AppendLibraryItem(slot.Occupant, Screen.DRONE);
                    else
                    {
                        DisplayBuilder.Append("[VACANCY]\n");
                        LineCount++;
                    }

                }
            }

            FormattedStringBuilder();

            Screens[(int)Screen.SLOT].WriteText(DisplayBuilder);
        }
        void DroneListBuilder()
        {
            CursorIndex = 0;
            LineCount = 0;
            DisplayBuilder.Clear();
            int D = SelObjIndex[(int)GUILayer.DRONE];
            DisplayBuilder.Append($"Drones: ({SelObjIndex[(int)GUILayer.DRONE]}/{Registered.Count})\n================\n");

            foreach (DroneRegistry reg in Registered)
            {
                AppendLibraryItem(reg, Screen.DRONE, false);
                DisplayBuilder.Append($"{reg.LastReport.TimeStamp}\n");
                LineCount++;
            }
                

            FormattedStringBuilder();

            Screens[(int)Screen.DRONE].WriteText(DisplayBuilder);
        }
        void ButtonStringBuilder(string[] actions)
        {
            DisplayBuilder.Clear();

            for (int i = 0; i < InputLabels.Length; i++)
                DisplayBuilder.Append($"{InputLabels[i]}\n");

            for (int i = 0; i < actions.Length; i++)
                DisplayBuilder.Append($"{i + 1} - {actions[i]}\n");

            Screens[(int)Screen.BUTTONS].WriteText(DisplayBuilder);
        }
        void FormattedStringBuilder()
        {
            FORM_BUFF = DisplayBuilder.ToString().Split('\n');
            DisplayBuilder.Clear();
            int startIndex = CursorIndex - HalfScreenBufferSize < 0 ? 2 : CursorIndex - HalfScreenBufferSize;
            startIndex = CursorIndex + HalfScreenBufferSize > LineCount ? LineCount - (2 * (HalfScreenBufferSize - 1)) : startIndex;
            startIndex = startIndex < 2 ? 2 : startIndex;

            try { DisplayBuilder.Append($"{FORM_BUFF[0]}\n{FORM_BUFF[1]}\n"); }
            catch { return; }

            try
            {
                if (!CapLines)
                    for (int i = 2; i < FORM_BUFF.Length; i++)
                        DisplayBuilder.Append($"{FORM_BUFF[i]}\n");
                else
                    for (int i = startIndex; i < startIndex + (2 * HalfScreenBufferSize) && i < FORM_BUFF.Length; i++)
                    {
                        //DisplayBuilder.Append($"i:{i} | cursorIndex: {CursorIndex} | LineCoount:{StringCount}\n");
                        DisplayBuilder.Append($"{FORM_BUFF[i]}\n");
                    }
            }
            catch { DisplayBuilder.Append("FAIL POINT\n"); }
        }

        void ButtonPress(int button)
        {
            switch (CurrentGUILayer)
            {
                case GUILayer.GROUP:
                    GroupAction(button);
                    break;

                case GUILayer.SLOT:
                    SlotAction(button);
                    break;

                case GUILayer.DRONE:
                    DroneAction(button);
                    break;
            }
        }
        void GroupAction(int action)
        {
            switch (action)
            {
                case 1:
                    try
                    {
                        DroneSwarm swarm = (DroneSwarm)Groups[SelObjIndex[(int)GUILayer.GROUP]];
                        swarm.Members[0].SYNCED = false;
                        swarm.Settings.Task = DroneTask.IDLE;
                        swarm.UpdateSlotGroup();
                    }
                    catch { }
                    break;

                case 2:
                    try
                    {
                        DroneSwarm swarm = (DroneSwarm)Groups[SelObjIndex[(int)GUILayer.GROUP]];
                        swarm.Members[0].SYNCED = false;
                        swarm.Settings.Task = DroneTask.FORM;
                        swarm.UpdateSlotGroup();
                    }
                    catch { }
                    break;

                case 3:
                    break;

                case 4:
                    break;

                case 5:
                    break;

                case 6:
                    break;

                case 7:
                    break;

                case 8:
                    break;

                case 9:
                    break;
            }
        }
        void SlotAction(int action)
        {
            switch (action)
            {
                case 1:
                    break;

                case 2:
                    break;

                case 3:
                    break;

                case 4:
                    break;

                case 5:
                    break;

                case 6:
                    break;

                case 7:
                    break;

                case 8:
                    break;

                case 9:
                    break;
            }
        }
        void DroneAction(int action)
        {
            switch (action)
            {
                case 1:
                    break;

                case 2:
                    break;

                case 3:
                    break;

                case 4:
                    break;

                case 5:
                    break;

                case 6:
                    break;

                case 7:
                    break;

                case 8:
                    break;

                case 9:
                    break;
            }
        }
        void GUINavigation(GUINav dir)
        {
            switch (dir)
            {
                case GUINav.UP:
                    ScrollSelection(true);
                    break;

                case GUINav.DOWN:
                    ScrollSelection(false);
                    break;

                case GUINav.BACK:
                    ChangeGUILayer(false);
                    break;

                case GUINav.SELECT:
                    ChangeGUILayer(true);
                    break;
            }
            //GUIUpdate();
        }
        void ChangeGUILayer(bool up)
        {
            if (up && (int)CurrentGUILayer + 1 < Enum.GetValues(typeof(GUILayer)).Length)
                CurrentGUILayer++;
            if (!up && (int)CurrentGUILayer > 0)
                CurrentGUILayer--;
            RefreshSelection();
        }
        void ScrollSelection(bool up)
        {
            SelObjIndex[(int)CurrentGUILayer] += up ? 1 : -1;
            RefreshSelection();
        }
        
        void RefreshSelection()
        {
            RefreshIndex((int)GUILayer.GROUP, Groups.Count);
            int G = SelObjIndex[(int)GUILayer.GROUP];
            RefreshIndex((int)GUILayer.SLOT, Groups[G].AllSlots.Count);
            RefreshIndex((int)GUILayer.DRONE, Registered.Count);
        }
        void GUIUpdate()
        {
            ButtonStringBuilder(MenuButtons[(int)CurrentGUILayer]);
            GroupListBuilder();
            SlotListBuilder();
            DroneListBuilder();
            SplashBuilder();
        }

        bool UserInputString(ref string buffer)
        {
            try
            {
                StringBuilder myStringBuilder = new StringBuilder();
                CockPitScreens[2].ReadText(myStringBuilder);
                buffer = myStringBuilder.ToString();
                if (buffer == "")
                    buffer = null;

                return true;
            }
            catch
            {
                return false;
            }
        }
        bool UserInputFloat(ref float buffer)
        {
            try
            {
                StringBuilder myStringBuilder = new StringBuilder();
                CockPitScreens[2].ReadText(myStringBuilder);
                buffer = float.Parse(myStringBuilder.ToString());
                return true;
            }
            catch
            {
                return false;
            }
        }

        #endregion

        #region FORMATION

        // WIP //////////////////
        Vector3[] EquatorDeltas;
        Point[][] SuperDeltas;
        Vector3[] VanguardDeltas;
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
        /////////////////////////


        IMyShipController HubControl;
        WorldFrame[] SphereDeltas;
        Vector3 OldPosition;


        #region WIP
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
        void GenerateEquatorDeltas(float radius, float distance, bool Literal = false)
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
        void GenerateLatitudeSphereDeltas(float radius, float distance, bool directional = false)
        {
            List<Vector3> points = new List<Vector3>();

            int ringCount = (int)((Math.PI * radius) / distance);

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

            SphereDeltas = new WorldFrame[points.Count];
            MatrixD buffer;
            for (int i = 0; i < points.Count; i++)
            {
                buffer = MatrixD.CreateLookAt(Vector3D.Zero, points[i], Vector3D.Up);
                SphereDeltas[i] = new WorldFrame(buffer, points[i], Vector3.Zero);
            };
        }

        #endregion
        static Vector3 DeTransformVectorRelative(MatrixD S, Vector3 N)
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


        #endregion

        #region HUB

        IMyBroadcastListener HubEar;
        IMyBroadcastListener LostEar;

        string[] MSG_STR_BUFFER = new string[3]; // ??
        //byte[] MSG_BYTE_BUFFER = new byte[Enum.GetValues(typeof(SwarmRequest)).Length];
        float[] MSG_IX_BUFFER = new float[Enum.GetValues(typeof(MsgIx)).Length];

        bool bConfigured;
        public int MSG_COUNT = 0;
        public int DRONE_EAR_COUNT = 0;
        public int LOST_EAR_COUNT = 0;

        List<PendingRequest> ResponseQue = new List<PendingRequest>();
        List<DroneRegistry> Registered = new List<DroneRegistry>();
        List<DroneRegistry> Available = new List<DroneRegistry>();
        List<DroneSlotGroup> Groups = new List<DroneSlotGroup>();

        DockingCluster HubPier;
        DroneSwarm HubSwarm;

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

        public struct PendingRequest
        {
            public long ID;
            public RequestType Request;
            public int CurrentIndex;

            public PendingRequest(long iD, RequestType request, int index)
            {
                ID = iD;
                Request = request;
                CurrentIndex = index;
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
                Matrix.Translation = pos;
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
                InsertMatrixD(ref stream, Matrix, (int)MsgIx.M11);
                InsertVector3(ref stream, Velocity, (int)MsgIx.VEL_X);
            }
            public bool Expired()
            {
                return DateTime.Now - TimeStamp > EXP_TIME;
            }
        }
        public struct DroneSettings
        {
            public DroneTask Task;

            public byte TaskIndex;
            public byte Speed;
            public byte ShieldSize;

            public bool[] Options;

            public DroneSettings(DroneTask task, byte index, byte speed, byte shieldSize, bool[] options = null)
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
                TaskIndex = stream[(int)SwarmRequest.INDEX];
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
                stream[(int)SwarmRequest.INDEX] = TaskIndex;
                stream[(int)SwarmRequest.SPEED] = Speed;
                stream[(int)SwarmRequest.SHIELD_SIZE] = ShieldSize;

                GetByte(out stream[(int)SwarmRequest.OPTIONS], Options);
            }
        }

        public class DroneRegistry : IGUIListing
        {
            public long ID;
            public int RegisterIndex;
            public bool LINKED = false;
            public bool SYNCED = false;
            public DateTime Registration;
            public List<DroneTask> AvailableTasks = new List<DroneTask>(); // WIP

            public DroneSlotGroup Group;
            public DroneSlot Slot;

            public WorldFrame LastReport;

            public DroneRegistry(long id, WorldFrame report = new WorldFrame())//, int index = -1)
            {
                ID = id;
                LastReport = report;
                Registration = DateTime.Now;
                AvailableTasks.Add(DroneTask.IDLE);
                AvailableTasks.Add(DroneTask.FORM);
            }

            public void JoinFormation(DroneSwarm formation = null)
            {
                Group = formation;

                if (Group == null)
                    return;

                OccupySlot(Group.FindSlotRNG());
            }
            public bool OccupySlot(DroneSlot slot = null)
            {
                if (slot != null &&
                    (slot.Group == null ||
                     slot.Occupant != null))
                    return false;

                if (Slot != null)
                    Slot.Occupant = null;

                if (Group != null)
                    Group.Members.Remove(this);

                if (slot != null)
                {
                    slot.Occupant = this;
                    slot.Group.Members.Add(this);

                    Group = slot.Group;
                    Group.UpdateSlotGroup();
                }
                else
                {
                    Group = null;
                }

                Slot = slot;


                return true;
            }
            public string Save()
            {
                return $"{ID}:{Slot.ReturnIndex()}:{Slot is SwarmPosition}";
            }
            public int Load(string data, List<SwarmPosition> formation, List<DockingPort> docks)
            {
                string[] raw = data.Split(':');
                ID = long.Parse(raw[0]);
                int index = int.Parse(raw[1]);
                bool isForm = bool.Parse(raw[2]);
                if (index > -1)
                {
                    if (isForm && index < formation.Count)
                        OccupySlot(formation[index]);

                    if (!isForm && index < docks.Count)
                        OccupySlot(docks[index]);
                }
                return index;
            }

            public string ReturnListing()
            {
                string slot = Slot == null ? "NA" : Slot.ReturnIndex().ToString();
                string output = $"{slot} | [DRONE]:{ID}";//\nTasks:";

                return output;
            }
            public int ReturnIndex()
            {
                return RegisterIndex;
            }
            public GUILayer ReturnLayer()
            {
                return GUILayer.DRONE;
            }
        }
        public class DroneSlotGroup : IGUIListing
        {
            public List<DroneRegistry> Members = new List<DroneRegistry>();
            public List<DroneSlot> AllSlots = new List<DroneSlot>();
            public List<DroneSlot> OpenSlots = new List<DroneSlot>();
            public List<WorldFrame> FrameBuffer = new List<WorldFrame>();
            public WorldFrame OldOrigin;
            public WorldFrame CurrentOrigin;
            public IMyTerminalBlock CenterBlock;
            Random RNG = new Random();
            public int Index;
            public Program HUB;

            public DroneSlotGroup(Program hub, int index, IMyTerminalBlock block = null)
            {
                HUB = hub;
                Index = index;
                CenterBlock = block;
                ClearSlots();
            }
            public virtual void UpdateSlotGroup()
            {
                //UpdateOrigin();
                //foreach (DroneSlot slot in AllSlots)
                //slot.UpdateSlot();
            }
            public virtual void UpdateOrigin()
            {

            }
            /*public void CalculateSwarmCenter()
            {
                OldOrigin = CurrentOrigin;
                CurrentOrigin.Position = Vector3.Zero;
                for (int i = 0; i < FrameBuffer.Count; i++)
                {
                    CurrentOrigin.Position += FrameBuffer[i].Position;
                }
                CurrentOrigin.Position /= FrameBuffer.Count;
                CurrentOrigin.Velocity = CurrentOrigin.Position - OldOrigin.Position;
            }*/
            public void SortMembers()
            {
                Members.Clear();
                foreach (DroneSlot slot in AllSlots)
                    if (slot.Occupant != null)
                        Members.Add(slot.Occupant);
            }
            public void ClearSlots()
            {
                RNG = new Random();
                AllSlots.Clear();
                OpenSlots.Clear();
                Members.Clear();
            }

            public DroneSlot FindSlotRNG()
            {
                return OpenSlots.Count() > 0 ? OpenSlots[RNG.Next(0, OpenSlots.Count() - 1)] : null;
            }
            public DroneSlot GetSlot(int index)
            {
                if (index < 0 || index >= AllSlots.Count)
                    return null;
                return AllSlots[index];
            }

            public int ReturnIndex()
            {
                return Index;
            }
            public GUILayer ReturnLayer()
            {
                return GUILayer.GROUP;
            }

            public virtual string ReturnListing()
            {
                return $"";
            }
            public Vector3 ReturnPosition()
            {
                return CurrentOrigin.Matrix.Translation;
            }
        }
        public class DockingCluster : DroneSlotGroup
        {
            public DockingCluster(Program hub, int index, List<IMyShipConnector> ports) : base(hub, index)
            {
                SetupDocks(ports);
            }

            public override string ReturnListing()
            {
                return $"{HUB.Me.CubeGrid.CustomName}'s Docking Cluster";
            }
            void SetupDocks(List<IMyShipConnector> ports)
            {
                ClearSlots();

                for (int i = 0; i < ports.Count; i++)
                {
                    if (!ports[i].CustomName.Contains(DockIDtag))
                        continue;

                    DockingPort port = new DockingPort(ports[i], this, i);
                    AllSlots.Add(port);
                    OpenSlots.Add(port);
                }
            }
        }
        public class DroneSwarm : DroneSlotGroup
        {
            public string TAG;

            public bool ForcePull = true;
            public bool ForwardV = false;
            public bool TrueFlight = false;
            public bool ShuntFormation = false;
            public bool EvenSpacing = false;
            public bool Spin = false;

            public DroneSettings Settings;
            public WorldFrame Status;
            //public WorldFrame Goal;

            double OriginAngle = 0;
            double AngleBuffer;
            DateTime ZeroPoint;
            TimeSpan Interval = TimeSpan.FromMinutes(3);

            int FormationScalePow;
            float FormationScaleVal = 1;
            byte[] MSG_BYTE_BUFFER = new byte[5];

            public DroneSwarm(Program hub, int index, string tag, WorldFrame[] frames, DroneSettings init) : base(hub, index)
            {
                TAG = tag;
                Settings = init;
                SetupFormation(frames);
            }
            public void SetupFormation(WorldFrame[] frames)
            {
                ClearSlots();

                for (int i = 0; i < frames.Length; i++)
                {
                    SwarmPosition slot = new SwarmPosition(frames[i], this, i);
                    AllSlots.Add(slot);
                    OpenSlots.Add(slot);
                }
            }

            public override void UpdateOrigin()
            {
                CurrentOrigin.Update(HUB.Me);
                FrameBuffer.Clear();
                foreach (DroneSlot slot in AllSlots)
                {
                    slot.UpdateSlot();
                    FrameBuffer.Add(slot.Status);
                }
                //CalculateSwarmCenter();
            }
            public override void UpdateSlotGroup()
            {
                UpdateOrigin();
                base.UpdateSlotGroup();
                foreach (DroneRegistry reg in Members)
                    if (reg.LINKED && !reg.SYNCED)
                    {
                        //HUB.DebugStatic.Clear();
                        Settings.WriteToStream(MSG_BYTE_BUFFER);
                        //HUB.Screens[(int)Screen.DEBUG_STATIC].WriteText(HUB.DebugStatic);
                        HUB.IGC.SendBroadcastMessage(TAG, ImmutableArray.Create(MSG_BYTE_BUFFER));
                        foreach (DroneRegistry regg in Members)
                            regg.SYNCED = true;
                        break;
                    }
            }
            public void UpdateFormation()
            {
                TrickleFormation();
            }
            public void ToggleTrueFlight()
            {
                TrueFlight = !TrueFlight;
            }
            public void ToggleSpin()
            {
                SetSpin(!Spin);
            }
            public void SetSpin(bool spin)
            {
                Spin = spin;

                if (Spin)
                    ZeroPoint = DateTime.Now;

                else
                    OriginAngle = ReturnTimedAngle();
            }
            public void SetTask(DroneTask task)
            {

            }
            public void ScaleFormation(bool increase)
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
            public bool GenerateFormationLiteral(SwarmPosition slot, out WorldFrame output)
            {
                output = WorldFrame.Zero;
                if (slot == null)
                    return false;

                return GenerateFormationLiteral(slot.FormationDelta, out output);
            }
            /*public bool GenerateFormationLiteral(DroneRegistry drone, out WorldFrame output)
            {
                output = WorldFrame.Zero;
                if (drone == null)
                    return false;

                return GenerateFormationLiteral(drone.Status, out output);
            }
            public bool GenerateFormationLiteral(int index, out WorldFrame output)
            {
                output = WorldFrame.Zero;
                if (AllSlots == null ||
                    index < 0 ||
                    index >= AllSlots.Count)
                    return false;

                return GenerateFormationLiteral(((SwarmPosition)AllSlots[index]).CurrentDelta, out output);
            }
            public bool GenerateFormationLiteral(IMyTerminalBlock source, Vector3 velocity, out WorldFrame output)
            {
                output = WorldFrame.Zero;
                if (source == null)
                    return false;

                return GenerateFormationLiteral(new WorldFrame(source, velocity), out output);
            }*/
            /*public bool GenerateFormationLiteral(string gps, out WorldFrame output)
            {
                string[] data = gps.Split(':');
                try
                {
                    Vector3 newTarget = new Vector3(float.Parse(data[2]), float.Parse(data[3]), float.Parse(data[4]));
                    WorldFrame newFrame = new WorldFrame(CenterOfMass.Matrix, newTarget, CenterOfMass.Velocity)
                    return GenerateFormationLiteral(newTarget, ref formationDeltas, index, ref target);
                }
                catch
                {
                    return false;
                }
            }*/
            public bool GenerateFormationLiteral(WorldFrame source, out WorldFrame output)
            {
                //Vector3 positionBuffer = 
                //Vector3 scaledVector = source.Matrix.Translation * FormationScaleVal;

                AngleBuffer = OriginAngle;
                if (Spin)
                    AngleBuffer += ReturnTimedAngle();

                Vector3 relativeVector = DeTransformVectorRelative(GenerateOffsetMatrix(ref AngleBuffer), source.Matrix.Translation);
                output = new WorldFrame(
                    MatrixD.Multiply(CurrentOrigin.Matrix, source.Matrix),
                    relativeVector + CurrentOrigin.Matrix.Translation,
                    CurrentOrigin.Velocity);

                output.Matrix.Translation *= FormationScaleVal;

                return true;
            }
            double ReturnTimedAngle()
            {
                DateTime current = DateTime.Now;
                TimeSpan length = (current - ZeroPoint);

                return ((length.TotalSeconds % Interval.TotalSeconds) / Interval.TotalSeconds) * (2 * Math.PI);
            }
            void TrickleFormation()
            {
                if (!ShuntFormation)
                    return;

                for (int i = 0; i < AllSlots.Count - 1; i++)
                {
                    SwarmPosition target = (SwarmPosition)AllSlots[i];
                    SwarmPosition fill = (SwarmPosition)AllSlots[i + 1];

                    if (target == null ||
                        fill == null)
                        continue;

                    if (target.Occupant != null ||
                        fill.Occupant == null)
                        continue;

                    fill.Occupant.OccupySlot(target);
                }
            }
            public void ToggleShunt()
            {
                ShuntFormation = !ShuntFormation;
            }

            public override string ReturnListing()
            {
                return $"{TAG} Squadron";
            }
        }
        public class DroneSlot : IGUIListing //, IWorldListing
        {
            public int Index;
            public DroneSlotGroup Group;
            public DroneRegistry Occupant;
            //public WorldFrame OldDelta;
            public WorldFrame Status;

            public DroneSlot(DroneSlotGroup group, int index)
            {
                Group = group;
                Index = index;
            }
            public virtual void UpdateSlot()
            {
                //UpdateVelocity();
            }
            void UpdateVelocity()
            {
                //Status.Velocity = Status.Position - //OldDelta.Position;
            }
            public int ReturnIndex()
            {
                return Index;
            }
            public GUILayer ReturnLayer()
            {
                return GUILayer.SLOT;
            }
            public virtual string ReturnListing()
            {
                return $"[GENERIC]";
            }
            public virtual Vector3 ReturnVelocity()
            {
                return Status.Velocity;
            }
            public virtual MatrixD ReturnMatrix()
            {
                return Status.Matrix;
            }
            public virtual Vector3 ReturnPosition()
            {
                return Status.Matrix.Translation;
            }
        }
        public class SwarmPosition : DroneSlot
        {
            public WorldFrame LastOccupantReport;
            public WorldFrame FormationDelta;
            public SwarmPosition(WorldFrame frame, DroneSwarm group, int index) : base(group, index)
            {
                FormationDelta = frame;
            }

            public override void UpdateSlot()
            {
                ((DroneSwarm)Group).GenerateFormationLiteral(this, out Status);
                base.UpdateSlot();
            }

            public override string ReturnListing()
            {
                return $"[FORM]";
            }

            public override MatrixD ReturnMatrix()
            {
                return base.ReturnMatrix();
            }

            public override Vector3 ReturnPosition()
            {
                return base.ReturnPosition();
            }

            public override Vector3 ReturnVelocity()
            {
                return base.ReturnVelocity();
            }


        }
        public class DockingPort : DroneSlot
        {
            public IMyShipConnector Connector;

            public override void UpdateSlot()
            {
                Status.Update(Connector);
                base.UpdateSlot();
            }

            public bool DockIsFree()
            {
                if (Occupant != null)
                    return false;

                if (Connector.Status == MyShipConnectorStatus.Connectable ||
                    Connector.Status == MyShipConnectorStatus.Connected)
                    return false;

                return true;
            }

            public override string ReturnListing()
            {
                return $"[DOCK]";
            }

            public override MatrixD ReturnMatrix()
            {
                return Connector.WorldMatrix;
            }

            public override Vector3 ReturnPosition()
            {
                return Connector.GetPosition();
            }



            public override Vector3 ReturnVelocity()
            {
                return base.ReturnVelocity();
            }

            public DockingPort(IMyShipConnector port, DockingCluster group, int index) : base(group, index)
            {
                Connector = port;
            }
        }

        DroneRegistry Registration(long id)
        {
            DroneRegistry newDrone = new DroneRegistry(id);
            Registered.Add(newDrone);
            Available.Add(newDrone);
            ReIndexRegistries();
            return newDrone;
        }
        DroneRegistry ReturnCandidate(DroneTask task)
        {
            return Available.Find(x => x.AvailableTasks.IndexOf(task) > -1 && x.Slot == null);
        }
        void ReIndexRegistries()
        {
            for (int i = 0; i < Registered.Count; i++)
                Registered[i].RegisterIndex = i;
        }

        bool DroneResponse(PendingRequest pending)
        {
            DroneRegistry drone = Registered.Find(x => x.ID == pending.ID);
            DebugStatic.Append($"OUT: {DateTime.Now} | {(drone == null ? "NULL" : drone.ID.ToString())}\n");

            DockingPort port = null;
            if (drone.Slot is DockingPort)
                port = (DockingPort)drone.Slot;

            WorldFrame buffer = WorldFrame.Zero;

            switch (pending.Request)
            {
                case RequestType.REGISTER:
                    MSG_STR_BUFFER[0] = RequestType.REGISTER.ToString();
                    MSG_STR_BUFFER[1] = HubChannel;
                    MSG_STR_BUFFER[2] = HubSwarmChannel;

                    IGC.SendBroadcastMessage($"DRONE_{drone.ID}", ImmutableArray.Create(MSG_STR_BUFFER));

                    return true;

                case RequestType.IDLE:
                case RequestType.TASK:
                    if (drone.Group == null || !(drone.Group is DroneSwarm))
                    {
                        IGC.SendBroadcastMessage($"DRONE_{drone.ID}", "Missing Task Force!");
                        return false;
                    }
                    if (!(drone.Slot is SwarmPosition))
                    {
                        drone.OccupySlot(HubSwarm.FindSlotRNG());
                    }
                    if (drone.Slot == null)
                    {
                        IGC.SendBroadcastMessage($"DRONE_{drone.ID}", "No Slot available!");
                        return false;
                    }

                    buffer = drone.Slot.Status;
                    MSG_IX_BUFFER[(int)MsgIx.REQ] = (float)((DroneSwarm)drone.Group).Settings.Task;
                    break;

                case RequestType.DOCK:

                    if (!(drone.Slot is DockingPort))
                    {
                        drone.OccupySlot(HubPier.FindSlotRNG());
                    }

                    if (port == null)
                    {
                        IGC.SendBroadcastMessage($"DRONE_{drone.ID}", "No docks sam...");
                        return false;
                    }

                    /// MIGRATE DOCKING LOGIC!!! ///
                    buffer.Matrix = MatrixD.Multiply(port.Connector.WorldMatrix, MatrixD.CreateRotationY(Math.PI));
                    Vector3 portLocation = port.Connector.GetPosition();
                    Vector3 align = portLocation + (port.Connector.WorldMatrix.Forward * DockDisplacement);
                    if (pending.CurrentIndex == 0)
                    {
                        buffer.Matrix.Translation = align;
                        break;
                    }
                    if (pending.CurrentIndex == 1)
                    {
                        buffer.Matrix.Translation = portLocation;
                        break;
                    }
                    /////////////////////////////////
                    ///
                    return false;

                case RequestType.UN_DOCK:

                    if (drone.Slot is DockingPort)
                        drone.OccupySlot();

                    break;

                default:
                    return false;
            }

            buffer.WriteToStream(MSG_IX_BUFFER);
            IGC.SendBroadcastMessage($"DRONE_{drone.ID}", ImmutableArray.Create(MSG_IX_BUFFER));

            return true;
        }

        static void InsertVector3(ref float[] buffer, Vector3 input, int index)
        {
            for (int i = 0; i < 3; i++)
                buffer[index + i] = input.GetDim(i);
        }
        static void ExtractVector3(ref float[] dataStream, ref Vector3 output, int index)
        {
            for (int i = 0; i < 3; i++)
                output.SetDim(i, dataStream[index + i]);
        }
        static void InsertMatrixD(ref float[] buffer, MatrixD input, int index)
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
        static void ExtractID(ref float[] dataStream, ref long output, int index)
        {

            for (int i = 3; i > -1; i--)
            {
                output += (long)dataStream[index + i];
                output = i > 0 ? output << 16 : output;
            }
        }
        static MatrixD GenerateCorrection(MatrixD current, ref MatrixD target)
        {
            return MatrixD.Multiply(MatrixD.Invert(target), current);

            //return MatrixD.GetEulerAnglesXYZ(ref current, out solution);
        }

        void SwarmSpeed(bool faster = true)
        {
            int BOOL = faster ? 8 : 7;
            IGC.SendBroadcastMessage(HubSwarmChannel, BOOL);
        }
        void SwarmFit(bool bigger = true)
        {
            int BOOL = bigger ? 6 : 5;
            IGC.SendBroadcastMessage(HubSwarmChannel, BOOL);
        }

        void UpdateHubFrames()
        {
            HubPier.CurrentOrigin.Update(Me);
            //HubSwarm.CurrentOrigin.Update(Me);
            HubSwarm.UpdateSlotGroup();
        }
        void AssignmentHandler()
        {
            foreach (DroneSlotGroup group in Groups)
            {
                if (!(group is DroneSwarm))
                    continue;

                DroneSwarm swarm = (DroneSwarm)group;

                if (!swarm.ForcePull)
                    continue;

                DebugStream.Append("PullingDrone...\n");

                DroneSlot slot = swarm.FindSlotRNG();
                if (slot == null)
                    continue;

                DebugStream.Append("Slot Found! \n");

                DroneRegistry candidate = ReturnCandidate(swarm.Settings.Task);
                if (candidate == null)
                    continue;

                DebugStream.Append("Candidate Found!\n");

                AssignCandidate(candidate, slot);
            }
        }
        bool AssignCandidate(DroneRegistry reg, DroneSlot slot)
        {
            bool result = reg.OccupySlot(slot);
            if (result)
            {
                Available.Remove(reg);

            }

            return result;
        }
        void InboxHandler()
        {
            LOST_EAR_COUNT = 0;
            DRONE_EAR_COUNT = 0;
            while (LOST_EAR_COUNT < MAX_MSG_COUNT && LostEar.HasPendingMessage)
                if (ProcessNextMsg(LostEar, ref MSG_IX_BUFFER))
                {
                    DebugStatic.Append("Lost:\n");
                    AppendResponseQue();
                    LOST_EAR_COUNT++;
                }

            while (DRONE_EAR_COUNT < MAX_MSG_COUNT && HubEar.HasPendingMessage)
                if (ProcessNextMsg(HubEar, ref MSG_IX_BUFFER))
                {
                    DebugStatic.Append("Found:\n");
                    AppendResponseQue();
                    DRONE_EAR_COUNT++;
                }

        }
        void AppendResponseQue()
        {
            DroneRegistry droneReg = null;
            long id = 0;
            ExtractID(ref MSG_IX_BUFFER, ref id, (int)MsgIx.ID_0);

            for (int i = 0; i < Registered.Count; i++)
                if (Registered[i].ID == id)
                {
                    droneReg = Registered[i];
                    break;
                }

            if (droneReg == null)
                droneReg = Registration(id);

            DebugStatic.Append($"IN: {DateTime.Now} | {(droneReg == null ? "NULL" : droneReg.ID.ToString())}\n");

            if (!droneReg.LINKED)
                droneReg.LINKED = true;

            droneReg.LastReport.ReadFromStream(MSG_IX_BUFFER);
            RequestType request = (RequestType)MSG_IX_BUFFER[(int)MsgIx.REQ];
            int requestIndex = (int)MSG_IX_BUFFER[(int)MsgIx.R_IND];

            ResponseQue.Add(new PendingRequest(droneReg.ID, request, requestIndex));
        }
        void OutBoxHandler()
        {
            if (ResponseQue.Count < 1)
                return;

            for (int i = ResponseQue.Count - 1; i > - 1 && ResponseQue.Count - i < OUT_BOX_MAX; i--)
            {
                DroneResponse(ResponseQue[i]);
                ResponseQue.RemoveAt(i);
            }
        }
        bool ProcessNextMsg(IMyBroadcastListener ear, ref float[] buffer)
        {
            ImmutableArray<float> raw = ((ImmutableArray<float>)ear.AcceptMessage().Data);
            if (raw.Length != buffer.Length)
                return false;

            MSG_COUNT++;
            for (int i = 0; i < buffer.Length; i++)
                buffer[i] = i < raw.Length ? raw[i] : 0;

            return true;
        }
        void CleanupPool()
        {
            bool proc = false;
            for (int i = Registered.Count - 1; i > -1; i--)
            {
                if (Registered[i].LastReport.Expired())
                {
                    Registered[i].OccupySlot();
                    Registered.RemoveAt(i);
                    proc = true;
                }
            }
            if (proc)
                ReIndexRegistries();
        }
        void GenerateDocks()
        {
            List<IMyShipConnector> blocks = new List<IMyShipConnector>();
            GridTerminalSystem.GetBlocksOfType(blocks);
            HubPier = new DockingCluster(this, GenerateGroupIndex(), blocks);
            Groups.Add(HubPier);
        }
        void GenerateTestFormation()
        {
            GenerateLatitudeSphereDeltas(Radius, Distance);
            DroneSettings settings = new DroneSettings(DroneTask.IDLE, 0, 25, 20);
            HubSwarm = new DroneSwarm(this, GenerateGroupIndex(), HubSwarmChannel, SphereDeltas, settings);
            Groups.Add(HubSwarm);
        }
        int GROUP_INDEX = 0;
        int GenerateGroupIndex()
        {
            GROUP_INDEX++;
            return GROUP_INDEX - 1;
        }
        #endregion

        #region ENTRY-POINTS
        public Program()
        {
            Me.CustomName = HubPBname;
            PBscreen = Me.GetSurface(0);
            PBscreen.ContentType = ContentType.TEXT_AND_IMAGE;
            DebugStatic.Clear();

            for (int i = 0; i < TICK_BUFF.Length; i++)
                TICK_BUFF[i] = "\n";
            
            try
            {
                HubControl = (IMyShipController)GridTerminalSystem.GetBlockWithName(ShipControlName);
                if (HubControl != null && HubControl is IMyCockpit)
                    Screens[0] = ((IMyCockpit)HubControl).GetSurface(0);

                List<IMyTerminalBlock> panels = new List<IMyTerminalBlock>();
                IMyBlockGroup panelGroup = GridTerminalSystem.GetBlockGroupWithName(HubPanelGroup);
                if (panelGroup != null)
                    panelGroup.GetBlocks(panels);

                for (int i = 0; i + 1 < Screens.Length && i < panels.Count; i++)
                    if (panels[i] is IMyTextPanel)
                        Screens[i + 1] = (IMyTextSurface)panels[i];

                for (int i = 0; i < Screens.Length; i++)
                    if (Screens[i] != null)
                    {
                        Screens[i].ContentType = ContentType.TEXT_AND_IMAGE;
                        Screens[i].WriteText("");
                    }

                bConfigured = true;
            }
            catch { bConfigured = false; return; }


            GenerateDocks();
            GenerateLatitudeSphereDeltas(Radius, Distance, true);
            GenerateTestFormation();
            //Load();

            LostEar = IGC.RegisterBroadcastListener(LostChannel);
            HubEar = IGC.RegisterBroadcastListener(HubChannel);
            //HubEar.SetMessageCallback();
            Runtime.UpdateFrequency = UpdateFrequency.Update1;
        }
        public void Main(string argument, UpdateType updateSource)
        {
            RuntimeArguments(argument);
            MenuInput();
            GUIUpdate();
            Debugging();
            DebugTicker();

            if (!bConfigured)
                return;

            UpdateHubFrames();
            CleanupPool();
            AssignmentHandler();
            InboxHandler();
            OutBoxHandler();
        }
        void RuntimeArguments(string argument)
        {
            if (argument == string.Empty)
                return;


            switch (argument)
            {
                case "STANDBY":
                    //SwarmToggle();
                    break;

                case "MODE":

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
                    HubSwarm.ToggleShunt();
                    break;

                case "FORMATION":
                    //bForwardV = !bForwardV;
                    break;

                case "CLEAR":
                    Registered.Clear();
                    ResponseQue.Clear();
                    foreach (DroneSlotGroup group in Groups)
                        group.Members.Clear();
                    Storage = string.Empty;
                    Echo("Cleared!");
                    break;

                case "INCREASE":
                    HubSwarm.ScaleFormation(true);
                    break;

                case "DECREASE":
                    HubSwarm.ScaleFormation(false);
                    break;

                case "SPIN":
                    HubSwarm.ToggleSpin();
                    break;

                case "TRUE":
                    HubSwarm.ToggleTrueFlight();
                    break;

                default:
                    try
                    {
                        string[] button = argument.Split(':');
                        if (button[0] == "BUTTON")
                            ButtonPress(int.Parse(button[1]));
                    }
                    catch { }
                    break;
            }
        }
        public void Save()
        {
            try
            {
                //Storage = $"{Spin}:{Origin}\n";
                Storage += $"{Registered.Count}\n";
                foreach (DroneRegistry reg in Registered)
                    if (reg != null)
                        Storage += reg.Save() + "\n";
                Storage += $"{ResponseQue.Count}\n";
                foreach (PendingRequest request in ResponseQue)
                    Storage += $"{request.ID}:{(int)request.Request}:{request.CurrentIndex}\n";

                //Screens[(int)Screen.DEBUG_STATIC].WriteText(Storage);
            }
            catch { }
        }
        public void Load()
        {
            try
            {
                string[] raw0 = Storage.Split('\n');

                string[] raw1 = raw0[0].Split(':');
                //Spin = bool.Parse(raw1[0]);
                //Origin = double.Parse(raw1[1]);
                int regCount = int.Parse(raw0[1]);
                int pendCountIndex = regCount + 2;
                int pendCount = int.Parse(raw0[pendCountIndex]);

                for (int i = 2; i < (regCount + 2); i++)
                {
                    string[] raw2 = raw0[i].Split(':');
                    long id = long.Parse(raw2[0]);
                    int slotIndex = int.Parse(raw2[1]);
                    DroneRegistry reg = Registered.Find(x => x.ID == id);
                    if (reg == null)
                    {
                        reg = new DroneRegistry(id);
                        reg.OccupySlot(HubSwarm.AllSlots[slotIndex]);
                        Registered.Add(reg);
                    }
                }

                for (int i = pendCountIndex + 1; i < (pendCount + regCount /*+ 3*/); i++)
                {
                    string[] raw2 = raw0[i].Split(':');
                    PendingRequest request = new PendingRequest(long.Parse(raw2[0]), (RequestType)int.Parse(raw2[1]), int.Parse(raw2[2]));
                    ResponseQue.Add(request);
                }

                //Echo("Loaded");
            }
            catch
            {
                //Echo("Nothing to load");
            }
        }
        #endregion

        #endregion
    }
}
