using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Security.Cryptography;
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

        #region COMBO

        #region DEFS
        //const double DockDisplacement = 5;
        const double Radius = 50;
        const double Distance = 15;

        const string DockIDtag = "[DOCK]";
        const string HubPanelGroup = "HUB_LCDS";
        const string LostChannel = "NEED_HUB";
        const string HubChannel = "[TORQK]";
        const string HubPBname = "HUB_PB";
        const string ShipControlName = "HUB_CONTROL";

        const int LiteralCount = 4;
        const int OUT_BOX_MAX = 16;
        const int FORM_SCALE_MIN_LIMIT = 1;
        const int FORM_SCALE_MAX_LIMIT = 5;
        const byte SHIELD_FIT_MIN = 0;
        const byte SHIELD_FIT_MAX = 20;
        const int MAX_MSG_COUNT = 20;
        const int MAX_ASSIGN_COUNT = 10;
        const int PROC_HOLD = 100;

        public static Program PROG { get; private set; }
        static TimeSpan EXP_TIME = TimeSpan.FromMinutes(1);
        #endregion

        #region GUI VARS

        public interface IGUIListing
        {
            string ReturnListing();
            int ReturnIndex();
            GUILayer ReturnLayer();
        }

        public delegate void GUIAction();
        public class GUIButtonAction
        {
            public string Name;
            public GUIAction myAction;
            public GUIButtonPage myLayer;

            public GUIButtonAction(GUIButtonPage myLayer, GUIAction myAction, string name)
            {
                this.myLayer = myLayer;
                this.myAction = myAction;
                Name = name;
            }
        }

        public class GUIButtonPage
        {
            public string Name;
            public Dictionary<GUIEvent, GUIButtonAction> Actions;

            public GUIButtonPage(GUILayer layer)
            {
                Name = layer.ToString();
                Actions = new Dictionary<GUIEvent, GUIButtonAction>();
            }

            public GUIButtonPage(string name, Dictionary<GUIEvent, GUIButtonAction> actions)
            {
                Name = name;
                Actions = actions != null ? actions : new Dictionary<GUIEvent, GUIButtonAction>();
            }

            public void EventPress(GUIEvent press)
            {
                Actions[press].myAction();
            }
        }

        public Dictionary<GUILayer, GUIButtonPage> GUIPages;

        public void EventPress(GUIEvent press)
        {
            GUIPages[CurrentGUILayer].EventPress(press);
        }

        //void SetupGUI()
        //{
        //    GUIPages = new Dictionary<GUILayer, GUIButtonPage>();
        //
        //    GUIButtonPage groupPage = new GUIButtonPage(GUILayer.GROUP);
        //    groupPage.Actions.Add(GUIEvent.ALPHA_1, new GUIButtonAction(groupPage, ))
        //
        //    GUIPages.Add(GUILayer.GROUP, )
        //}

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
        public enum GUIEvent
        {
            ALPHA_1,
            ALPHA_2,
            ALPHA_3,
            ALPHA_4,
            ALPHA_5,
            ALPHA_6,
            ALPHA_7,
            ALPHA_8,
            ALPHA_9,

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
        static StringBuilder DisplayBuilder = new StringBuilder();
        static StringBuilder DebugStream = new StringBuilder();
        static StringBuilder DebugStatic = new StringBuilder();

        GUILayer CurrentGUILayer = GUILayer.GROUP;
        bool CapLines = true;
        int LineCount = 0;
        int CursorIndex = 0;
        int HalfScreenBufferSize = 7;

        int[] CharBuffers = new int[Enum.GetValues(typeof(Screen)).Length];
        int[] SelObjIndex = new int[Enum.GetValues(typeof(GUILayer)).Length];
        int[] LastLibraryInput = new int[2];
        int[] LastLibraryClock = new int[2];

        static readonly string[] GroupActions = new string[]
        {
            "Idle",
            "Form",
            "Focus",
            "Shunt",
            "Spin",
            "IncreaseForm",
            "DecreaseForm",
            "IncreaseFit",
            "DecreaseFit"
        };
        static readonly string[] SlotActions = new string[]
        {

        };
        static readonly string[] DroneActions = new string[]
        {
            "Manual Resupply",
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

            DebugStream.Append(
                $"Configured: {Configured}\n" +
                $"Focus: {HubSwarm.Focus}\n" +
                $"AiTarget: {(AiTarget.HasValue ? AiTarget.Value.Position.ToString() : "None")}\n" +
                $"Point Coint:{SphereDeltas.Count()}\n" +
                $"MaxSpeed: {Groups[SelObjIndex[0]].Settings.MaxSpeed}\n" +
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
                    GUIEvent nav = z > 0 ? GUIEvent.UP : GUIEvent.DOWN;
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
                    GUIEvent nav = x < 0 ? GUIEvent.BACK : GUIEvent.SELECT;
                    GUINavigation(nav);
                }
                else
                    LastLibraryClock[1] = 0;
            }


        }

        string MatrixToStringA(MatrixD matrix)
        {
            return
                $"position: {matrix.M41} : {matrix.M42} : {matrix.M43}\n" +
                $"Right: {matrix.M11} : {matrix.M12} : {matrix.M13}\n" +
                $"Up: {matrix.M21} : {matrix.M22} : {matrix.M23}\n" +
                $"Forward: {matrix.M31} : {matrix.M32} : {matrix.M33}";

        }

        public static string MatrixToStringB(MatrixD matrix, string digits = "#.##")
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
                            $"Matrix: {MatrixToStringA(group.CurrentOrigin.Matrix)}");
                        if (group is DroneSwarm)
                        {
                            DroneSwarm swarm = (DroneSwarm)group;
                            DisplayBuilder.Append($"Current Swarm Task: {swarm.Settings.RequestedTask}\n");
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
                        DisplayBuilder.Append($"{drone.LastReportedFrame.Matrix.Translation} || {drone.LastReportedFrame.TimeStamp}\n");
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
                DisplayBuilder.Append($"{reg.LastReportedFrame.TimeStamp}\n");
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
            DroneSlotGroup group = Groups[SelObjIndex[(int)GUILayer.GROUP]];
            if (group == null)
                return;
            switch (action)
            {
                case 1:
                        group.Settings.RequestedTask = DroneTask.IDLE;
                    break;

                case 2:
                        group.Settings.RequestedTask = DroneTask.FORM;
                    break;

                case 3:
                    if (!(group is DroneSwarm))
                        break;
                    ((DroneSwarm)group).ToggleFocus();
                    break;

                case 4:
                    if (!(group is DroneSwarm))
                        break;
                    ((DroneSwarm)group).ToggleShunt();
                    break;

                case 5:
                    if (!(group is DroneSwarm))
                        break;
                    ((DroneSwarm)group).ToggleSpin();
                    break;

                case 6:
                    if (!(group is DroneSwarm))
                        break;
                    ((DroneSwarm)group).ScaleFormation(true);
                    break;

                case 7:
                    if (!(group is DroneSwarm))
                        break;
                    ((DroneSwarm)group).ScaleFormation(false);
                    break;

                case 8:
                    group.AdjustShieldSize();
                    break;

                case 9:
                    group.AdjustShieldSize(false);
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
            DroneRegistry drone = Registered[SelObjIndex[(int)GUILayer.DRONE]];
            if (drone == null)
                return;

            switch (action)
            {
                case 1:
                    ManualDock(drone);
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
        void GUINavigation(GUIEvent dir)
        {
            switch (dir)
            {
                case GUIEvent.UP:
                    ScrollSelection(true);
                    break;

                case GUIEvent.DOWN:
                    ScrollSelection(false);
                    break;

                case GUIEvent.BACK:
                    ChangeGUILayer(false);
                    break;

                case GUIEvent.SELECT:
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
        bool UserInputDouble(ref double buffer)
        {
            try
            {
                StringBuilder myStringBuilder = new StringBuilder();
                CockPitScreens[2].ReadText(myStringBuilder);
                buffer = double.Parse(myStringBuilder.ToString());
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
        IMyLargeTurretBase Targetter;
        MyDetectedEntityInfo? AiTarget = null;

        WorldFrame[] SphereDeltas;


        #region WIP
        void GenerateSuperSphereDeltas(double radius, double distance, bool directional = false)
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
                    double shortestPullDelta = Vector3.Distance(SuperDeltas[i][j].Delta, SuperDeltas[i + 1][0].Delta);

                    for (int k = 1; k < rings[i + 1].Count; k++)
                    {
                        double currentPullDelta = Vector3.Distance(SuperDeltas[i][j].Delta, SuperDeltas[i + 1][k].Delta);

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
        void GenerateEquatorDeltas(double radius, double distance, bool Literal = false)
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
        void GenerateThreePointVanguardDeltas(double radius, double altitude)
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
        void GenerateLatitudeSphereDeltas(double radius, double distance, bool directional = false)
        {
            List<Vector3D> points = new List<Vector3D>();

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
            MatrixD mBuffer;
            Vector3D vBuffer;
            for (int i = 0; i < points.Count; i++)
            {
                vBuffer = points[i];
                CreateLookAt(ref Vector3D.Zero, ref  vBuffer, ref Vector3D.Up, out mBuffer);
                mBuffer.Translation = vBuffer;
                DebugStatic.Append($"Point{i}:\n{MatrixToStringA(mBuffer)}");
                SphereDeltas[i] = new WorldFrame(mBuffer, Vector3.Zero);
            };
        }

        #endregion
        static Vector3D[] vector_buffer = new Vector3D[4];
        static void CreateLookAt(ref Vector3D camPos, ref Vector3D targetPos, ref Vector3D camUp, out MatrixD result)
        {
            vector_buffer[3] = camPos;                                                                      // translate result to cam
            vector_buffer[2] = camPos - targetPos;                                                          // set z to reverse delta
            //vector_buffer[2].Z *= -1;                                                                       // z-flip
            vector_buffer[2] = vector_buffer[2] == Vector3D.Zero ? Vector3D.Backward : vector_buffer[2];    // zero corner-case
            Vector3D.Normalize(ref vector_buffer[2], out vector_buffer[2]);
            Vector3D.Cross(ref camUp, ref vector_buffer[2], out vector_buffer[0]);                          // generate right                                                                        // z-flip
            Vector3D.Normalize(ref vector_buffer[0], out vector_buffer[0]);
            Vector3D.Cross(ref vector_buffer[2], ref vector_buffer[0], out vector_buffer[1]);               // correct up
            Vector3D.Normalize(ref vector_buffer[1], out vector_buffer[1]);

            // populate matrix
            result = MatrixD.Identity;
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 3; c++)
                    result[r, c] = vector_buffer[r].GetDim(c);
        }
        static void DeTransformVectorRelative(ref MatrixD S, ref Vector3D N, out Vector3D DV)
        {
            //Vector3D DV = new Vector3D();

            /*
                x = Xa + Yd + Zg
                y = Xb + Ye + Zh
                z = Xc + Yf + Zi 
             */

            DV.X = (N.X * S.M11) + (N.Y * S.M21) + (N.Z * S.M31);

            DV.Y = (N.X * S.M12) + (N.Y * S.M22) + (N.Z * S.M32);

            DV.Z = (N.X * S.M13) + (N.Y * S.M23) + (N.Z * S.M33);

            //return DV;
        }
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
        static void GenerateCorrection(ref MatrixD current, ref MatrixD target, out MatrixD correction)
        {
            correction = MatrixD.Multiply(target, MatrixD.Invert(current));
        }
        static void GenerateOffsetMatrix(ref double radians, out MatrixD output)
        {
            /* 
             * Keen Implementation:
             * Row 1: Right.x , Right.y , Right.z
             * Row 2: Up.x    , Up.y    , Up.z
             * Row 3: Back.x  , Back.y  , Back.z
            */

            double deltaZ = Math.Sin(radians);
            double deltaX = Math.Cos(radians);

            output.M11 = deltaX;  output.M12 = 0; output.M13 = deltaZ;  output.M14 = 1;

            output.M21 = 0;       output.M22 = 1; output.M23 = 0;       output.M24 = 1;

            output.M31 = -deltaZ; output.M32 = 0; output.M33 = deltaX;  output.M34 = 1;

            output.M41 = 0;       output.M42 = 0; output.M43 = 0;       output.M44 = 1;

            /*MatrixD output = new MatrixD(
                deltaX, 0, deltaZ,
                0, 1, 0,
                -deltaZ, 0, deltaX);*/

            //return output;
        }

        void AquireAiTargetedEntity()
        {
            if (Targetter != null &&
                Targetter.AIEnabled &&
                Targetter.HasTarget)
            {
                AiTarget = Targetter.GetTargetedEntity();
                return;
            }
                
            AiTarget = null;
        }

        #endregion

        #region HUB

        IMyBroadcastListener HubEar;
        IMyBroadcastListener LostEar;

        static byte[] MSG_BYTE_BUFFER = new byte[4];
        static double[] MSG_IX_BUFFER = new double[Enum.GetValues(typeof(MsgIx)).Length];
        static string[] MSG_STR_BUFFER = new string[3];

        bool DEBUG_BREAK;
        bool Configured;
        int GROUP_INDEX = 0;
        public int MSG_COUNT = 0;
        public int DRONE_EAR_COUNT = 0;
        public int LOST_EAR_COUNT = 0;
        const byte DEFAULT_MAX_SPEED = 20;

        List<PendingRequest> ResponseQue = new List<PendingRequest>();
        static List<DroneRegistry> Registered = new List<DroneRegistry>();
        static List<DroneRegistry> Available = new List<DroneRegistry>();
        List<DroneSlotGroup> Groups = new List<DroneSlotGroup>();

        DockingCluster HubPier;
        DroneSwarm HubSwarm;

        #region ENUMS
        public enum MsgIx
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
        public enum DroneTask
        {
            REGISTER,
            //BUSY,
            UN_DOCK,
            IDLE,
            DOCK,
            LOBBY,
            FORM,
            HAUL,
            MINE,
            GRIND
        }
        public enum StatusIx
        {
            CURRENT,
            EQUIPMENT,
            SEQ_INDEX,
        }
        public enum SquadIx
        {
            REQUEST,
            SPEED,
            SHIELD_SIZE,
            OPTIONS
        }
        public enum ResourceIx
        {
            FUEL,
            POWER,
            INTEGRITY,
            CARGO,
        }
        public enum Option
        {
            NEW_REQUEST = 0,
            SHIELD_ON = 1,
            FLIGHT_TRUE = 2,
            SPIN_ON = 3,
            SHUNT_ON = 4
        }
        public enum Equipment
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
        public struct PendingRequest
        {
            public long ID;
            public DroneStatus Status;

            public PendingRequest(long iD, DroneStatus status)
            {
                ID = iD;
                Status = status;
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
            public DroneTask RequestedTask;

            public byte MaxSpeed;
            public byte ShieldSize;

            public bool[] Options;

            public static SwarmSettings Default = new SwarmSettings(DroneTask.IDLE);

            public SwarmSettings(DroneTask task = DroneTask.IDLE, byte speed = DEFAULT_MAX_SPEED, byte shieldSize = 0, bool[] options = null)
            {
                RequestedTask = task;
                MaxSpeed = speed;
                ShieldSize = shieldSize;
                Options = options == null || options.Length != 8 ? new bool[8] : options;
            }

            public void Toggle(Option option)
            {
                Options[(int)option] = !Options[(int)option];
            }
            public bool this[Option option]
            {
                get { return Options[(int)option]; }
                set { Options[(int)option] = value; }
            }
            public void ReadFromStream()
            {
                GetBytesFromDouble(MSG_IX_BUFFER[(int)MsgIx.SQUAD]);

                RequestedTask = (DroneTask)MSG_BYTE_BUFFER[(int)SquadIx.REQUEST];
                MaxSpeed = MSG_BYTE_BUFFER[(int)SquadIx.SPEED];
                ShieldSize = MSG_BYTE_BUFFER[(int)SquadIx.SHIELD_SIZE];
                GetBools(MSG_BYTE_BUFFER[(int)SquadIx.OPTIONS], Options);
            }
            public void WriteToStream()
            {
                MSG_BYTE_BUFFER[(int)SquadIx.REQUEST] = (byte)RequestedTask;
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
            public void Update(IMyTerminalBlock block, bool flip = false)
            {
                if (block == null)
                    return;
                
                Matrix = block.WorldMatrix;
                Matrix.Forward *= flip ? -1 : 1;
                Matrix.Right *= flip ? -1 : 1;
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

        #region CLASSES
        public class DroneRegistry : IGUIListing
        {
            public long ID;
            public int RegisterIndex;
            public DateTime Registration;
            public List<DroneTask> AvailableTasks = new List<DroneTask>(); // WIP

            public DroneSlotGroup Group;
            public DroneSlot Slot;

            public WorldFrame LastReportedFrame;
            public DroneStatus LastReportedStatus;

            public DroneRegistry(long id, WorldFrame frame = new WorldFrame())
            {
                ID = id;
                LastReportedFrame = frame;
                LastReportedStatus = DroneStatus.Default;
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
                    slot.Occupant != null)
                    return false;

                if (Slot != null)
                {
                    Slot.Group.OpenSlots.Add(Slot);
                    Slot.Occupant = null;
                }
                    
                if (Group != null)
                    Group.Members.Remove(this);

                if (slot != null)
                {
                    slot.Occupant = this;
                    slot.Group.Members.Add(this);
                    slot.Group.OpenSlots.Remove(slot);
                    Group = slot.Group;
                }
                else
                {
                    Group = null;
                }

                bool wasAvailable = Slot == null;
                bool isAvailable = slot == null;

                if (wasAvailable && !isAvailable)
                    Available.Remove(this);

                if (!wasAvailable && isAvailable)
                    Available.Add(this);

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
                string output = $"{(Slot == null ? "NA" : Slot.ReturnIndex().ToString())} | [DRONE]:{ID}";
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

            public WorldFrame OldOrigin;
            public WorldFrame CurrentOrigin;
            public SwarmSettings Settings;

            public IMyTerminalBlock CenterBlock;
            public IMyEntity CenterEntity;
            Random RNG = new Random();
            public int Index;

            public DroneSlotGroup(int index, IMyTerminalBlock block)
            {
                Index = index;
                CenterBlock = block;
                ClearSlots();
            }
            public virtual void UpdateSlotGroup()
            {
                UpdateOrigin();
                foreach (DroneSlot slot in AllSlots)
                    slot.UpdateSlot();
            }
            public virtual void UpdateOrigin()
            {
                CurrentOrigin.Update(CenterBlock);
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

            public void AdjustShieldSize(bool increase = true)
            {
                Settings.ShieldSize = increase? (byte)(Settings.ShieldSize + 1) : (byte)(Settings.ShieldSize - 1);
                Settings.ShieldSize = Settings.ShieldSize < SHIELD_FIT_MIN ? SHIELD_FIT_MIN : Settings.ShieldSize > SHIELD_FIT_MAX ? SHIELD_FIT_MAX : Settings.ShieldSize;
            }

            public Vector3 ReturnPosition()
            {
                return CurrentOrigin.Matrix.Translation;
            }
        }
        public class DockingCluster : DroneSlotGroup
        {
            public DockingCluster(int index, IMyTerminalBlock block, List<IMyShipConnector> ports) : base(index, block)
            {
                SetupDocks(ports);
            }

            public override string ReturnListing()
            {
                return $"{PROG.Me.CubeGrid.CustomName}'s Docking Cluster";
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
            //public bool TrueFlight = false;
            public bool ShuntFormation = false;
            public bool EvenSpacing = false;
            public bool Spin = false;
            public bool Focus = false;

            public WorldFrame Status;

            double OriginAngle = 0;
            double AngleBuffer;
            DateTime ZeroPoint;
            TimeSpan Interval = TimeSpan.FromMinutes(3);

            int FormationScalePow = FORM_SCALE_MIN_LIMIT;
            double FormationScaleValue = 1;

            public DroneSwarm(int index, IMyTerminalBlock block, string tag, WorldFrame[] frames, SwarmSettings init) : base(index, block)
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

            public override void UpdateSlotGroup()
            {
                base.UpdateSlotGroup();
                TrickleFormation();
            }
            public override void UpdateOrigin()
            {
                if (Focus && PROG.AiTarget.HasValue)
                    CurrentOrigin.Update(PROG.AiTarget.Value);
                else
                    CurrentOrigin.Update(CenterBlock);
            }
            public void ToggleTrueFlight()
            {
                Settings.Toggle(Option.FLIGHT_TRUE);
            }
            public void ToggleSpin()
            {
                SetSpin(!Spin);
            }
            public void ToggleFocus()
            {
                Focus = !Focus;
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
                if (increase && FormationScalePow < FORM_SCALE_MAX_LIMIT)
                    FormationScalePow++;
                if (!increase && FormationScalePow > FORM_SCALE_MIN_LIMIT)
                    FormationScalePow--;
                SetScaleValue();
            }
            void SetScaleValue()
            {
                FormationScaleValue = Math.Pow((double)1.1, FormationScalePow);
            }
            
            public bool GenerateFormationLiteral(SwarmPosition slot, out WorldFrame output)
            {
                if (slot == null)
                {
                    DebugStatic.Append("null slot\n");
                    output = WorldFrame.Zero;
                    return false;
                }
                    
                GenerateFormationLiteral(ref slot.FormationDelta, out output);
                return true;
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
            public void GenerateFormationLiteral(ref WorldFrame inDelta, out WorldFrame outLiteral)
            {
                //outLiteral = inDelta;
                //return;

                AngleBuffer = OriginAngle;
                if (Spin)
                    AngleBuffer += ReturnTimedAngle();


                Vector3D rawDelta = inDelta.Matrix.Translation;
                Vector3D rotatedDelta, resolvedDelta;
                MatrixD rotationMatrix = MatrixD.CreateFromYawPitchRoll(0, 0, AngleBuffer);
                DeTransformVectorRelative(ref rotationMatrix, ref rawDelta, out rotatedDelta);
                DeTransformVectorRelative(ref CurrentOrigin.Matrix, ref rotatedDelta, out resolvedDelta);
                resolvedDelta *= FormationScaleValue;
                MatrixD finalMatrix;
                MatrixD.Multiply(ref inDelta.Matrix, ref rotationMatrix, out finalMatrix);
                MatrixD.Multiply(ref finalMatrix, ref CurrentOrigin.Matrix, out finalMatrix);
                finalMatrix.Translation = CurrentOrigin.Matrix.Translation + resolvedDelta;
                outLiteral = new WorldFrame(finalMatrix, CurrentOrigin.Velocity);

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
        public class DroneSlot : IGUIListing
        {
            public int Index;
            public DroneSlotGroup Group;
            public DroneRegistry Occupant;
            public WorldFrame LiteralFrame;

            public DroneSlot(DroneSlotGroup group, int index)
            {
                Group = group;
                Index = index;
            }
            public virtual void UpdateSlot()
            {
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
                return LiteralFrame.Velocity;
            }
            public virtual MatrixD ReturnMatrix()
            {
                return LiteralFrame.Matrix;
            }
            public virtual Vector3 ReturnPosition()
            {
                return LiteralFrame.Matrix.Translation;
            }
        }
        public class SwarmPosition : DroneSlot
        {
            public WorldFrame LastOccupantReport;
            public WorldFrame FormationDelta;
            public DroneSwarm Swarm => Group == null ? null : (DroneSwarm)Group;
            public SwarmPosition(WorldFrame frame, DroneSwarm group, int index) : base(group, index)
            {
                FormationDelta = frame;
            }

            public override void UpdateSlot()
            {
                Swarm.GenerateFormationLiteral(this, out LiteralFrame);
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
                LiteralFrame.Update(Connector, true);
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
                    MSG_IX_BUFFER[(int)MsgIx.MATRIX + (r * 3) + c] = input[r, c];
        }
        static void ExtractMatrixD(ref MatrixD output)
        {
            output = MatrixD.Identity;
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 3; c++)
                    output[r, c] = MSG_IX_BUFFER[(int)MsgIx.MATRIX + (r * 3) + c];
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
        static void ExtractID(ref double[] dataStream, ref long output, int index)
        {
            for (int i = 1; i > -1; i--)
            {
                output += (long)dataStream[index + i];
                output = i > 0 ? output << 32 : output;
            }
        }
        #endregion

        #region MAIL HANDLING
        void InboxHandler()
        {
            LOST_EAR_COUNT = 0;
            DRONE_EAR_COUNT = 0;
            while (LOST_EAR_COUNT < MAX_MSG_COUNT && LostEar.HasPendingMessage)
                if (ProcessNextMsg(LostEar, ref MSG_IX_BUFFER))
                {
                    //DebugStatic.Append("Lost:\n");
                    AppendResponseQue();
                    LOST_EAR_COUNT++;
                }

            while (DRONE_EAR_COUNT < MAX_MSG_COUNT && HubEar.HasPendingMessage)
                if (ProcessNextMsg(HubEar, ref MSG_IX_BUFFER))
                {
                    //DebugStatic.Append("Found:\n");
                    AppendResponseQue();
                    DRONE_EAR_COUNT++;
                }

        }
        bool ProcessNextMsg(IMyBroadcastListener ear, ref double[] buffer)
        {
            ImmutableArray<double> raw = ((ImmutableArray<double>)ear.AcceptMessage().Data);
            if (raw.Length != buffer.Length)
                return false;

            MSG_COUNT++;
            for (int i = 0; i < buffer.Length; i++)
                buffer[i] = i < raw.Length ? raw[i] : 0;

            return true;
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

            

            droneReg.LastReportedFrame.ReadFromStream();
            droneReg.LastReportedStatus.ReadFromStream();

            DebugStatic.Append($"IN: {DateTime.Now} | {(droneReg == null ? "NULL" : $"{droneReg.LastReportedStatus.Task} | {droneReg.ID} \n {MatrixToStringA(droneReg.LastReportedFrame.Matrix)}")}\n");

            ResponseQue.Add(new PendingRequest(droneReg.ID, droneReg.LastReportedStatus));
        }

        void OutBoxHandler()
        {
            if (ResponseQue.Count < 1)
                return;

            //for (int i = ResponseQue.Count - 1; i > -1 && ResponseQue.Count - i < OUT_BOX_MAX; i--)
            for (int i = 0; i < ResponseQue.Count && i < OUT_BOX_MAX; i++)
            {
                DroneResponse(ResponseQue[0]);
                ResponseQue.RemoveAt(0);
            }
        }
        void ManualDock(DroneRegistry drone)
        {
            MSG_STR_BUFFER[0] = DroneTask.DOCK.ToString();
            MSG_STR_BUFFER[1] = HubChannel;

            IGC.SendBroadcastMessage($"DRONE_{drone.ID}", ImmutableArray.Create(MSG_STR_BUFFER));
        }

        bool DroneResponse(PendingRequest pending)
        {
            DroneRegistry drone = Registered.Find(x => x.ID == pending.ID);
            //DebugStatic.Append($"OUT: {DateTime.Now} | {pending.Status.Task} | {pending.ID}\n");

            switch (pending.Status.Task)
            {
                case DroneTask.REGISTER:
                    MSG_STR_BUFFER[0] = DroneTask.REGISTER.ToString();
                    MSG_STR_BUFFER[1] = HubChannel;

                    IGC.SendBroadcastMessage($"DRONE_{drone.ID}", ImmutableArray.Create(MSG_STR_BUFFER));

                    return true;

                case DroneTask.UN_DOCK:

                    if (drone.Slot is DockingPort)
                        drone.OccupySlot();

                    return true;

                case DroneTask.IDLE:
                case DroneTask.FORM:

                    if (!(drone.Slot is SwarmPosition))
                        drone.OccupySlot(HubSwarm.FindSlotRNG());
                    
                    if (drone.Slot == null)
                    {
                        IGC.SendBroadcastMessage($"DRONE_{drone.ID}", "No Slot available!");
                        return false;
                    }

                    break;

                case DroneTask.DOCK:

                    if (!(drone.Slot is DockingPort))
                        drone.OccupySlot(HubPier.FindSlotRNG());

                    if (drone.Slot == null)
                    {
                        IGC.SendBroadcastMessage($"DRONE_{drone.ID}", "No docks sam...");
                        return false;
                    }

                    break;

                default:
                    // Unkown request...
                    return false;
            }

            SwarmSettings buffer = drone.Group.Settings;
            buffer[Option.NEW_REQUEST] = buffer.RequestedTask != pending.Status.Task;
            buffer.WriteToStream();

            drone.Slot.LiteralFrame.WriteToStream();

            DebugStatic.Append($"Out Task Buffer [{pending.Status.Task}][Conflict?:{buffer[Option.NEW_REQUEST]}]:\n{MatrixToStringA(drone.Slot.LiteralFrame.Matrix)}\n");
            

            IGC.SendBroadcastMessage($"DRONE_{drone.ID}", ImmutableArray.Create(MSG_IX_BUFFER));

            return true;
        }
        #endregion

        #region GROUP API'S
        void SwarmSpeed(bool faster = true)
        {
            int BOOL = faster ? 8 : 7;
            //IGC.SendBroadcastMessage(HubSwarmChannel, BOOL);
        }
        void SwarmFit(bool bigger = true)
        {
            int BOOL = bigger ? 6 : 5;
            //IGC.SendBroadcastMessage(HubSwarmChannel, BOOL);
        }
        #endregion

        #region REGISTRY/ASSIGNMENT
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

                DroneRegistry candidate = ReturnCandidate(swarm.Settings.RequestedTask);
                if (candidate == null)
                    continue;

                DebugStream.Append("Candidate Found!\n");

                candidate.OccupySlot(slot);
            }
        }
        #endregion

        #region INITIALIZERS
        void GenerateDocks()
        {
            List<IMyShipConnector> blocks = new List<IMyShipConnector>();
            GridTerminalSystem.GetBlocksOfType(blocks);
            HubPier = new DockingCluster(GenerateGroupIndex(), Me, blocks);
            Groups.Add(HubPier);
        }
        void GenerateTestFormation()
        {
            GenerateLatitudeSphereDeltas(Radius, Distance, true);
            SwarmSettings settings = new SwarmSettings(DroneTask.IDLE, 10, 20);
            HubSwarm = new DroneSwarm(GenerateGroupIndex(), Me, HubChannel, SphereDeltas, settings);
            Groups.Add(HubSwarm);
        }
        int GenerateGroupIndex()
        {
            GROUP_INDEX++;
            return GROUP_INDEX - 1;
        }
        #endregion

        #region UPDATES
        void UpdateHubWorldFrames()
        {
            HubPier.UpdateSlotGroup();
            HubSwarm.UpdateSlotGroup();
        }
        void CleanupPool()
        {
            bool proc = false;
            for (int i = Registered.Count - 1; i > -1; i--)
            {
                if (Registered[i].LastReportedFrame.Expired())
                {
                    Registered[i].OccupySlot();
                    Registered.RemoveAt(i);
                    proc = true;
                }
            }
            if (proc)
                ReIndexRegistries();
        }
        void ReIndexRegistries()
        {
            for (int i = 0; i < Registered.Count; i++)
                Registered[i].RegisterIndex = i;
        }

        #endregion

        #endregion

        #region ENTRY-POINTS
        public Program()
        {
            Me.CustomName = HubPBname;
            PROG = this;
            PBscreen = Me.GetSurface(0);
            PBscreen.ContentType = ContentType.TEXT_AND_IMAGE;
            DebugStatic.Clear();

            for (int i = 0; i < TICK_BUFF.Length; i++)
                TICK_BUFF[i] = "\n";
            
            try
            {
                //IMyLargeTurretBase control;
                //
                //control.tar
                //
                //Echo("yo");
                //api = new WcPbApi();
                //Echo("yo");
                //api.Activate(Me);
                //Echo("yo");

                HubControl = (IMyShipController)GridTerminalSystem.GetBlockWithName(ShipControlName);

                if (HubControl != null && HubControl is IMyCockpit)
                    Screens[0] = ((IMyCockpit)HubControl).GetSurface(0);

                List<IMyTerminalBlock> panels = new List<IMyTerminalBlock>();
                IMyBlockGroup panelGroup = GridTerminalSystem.GetBlockGroupWithName(HubPanelGroup);

                List<IMyLargeTurretBase> turrets = new List<IMyLargeTurretBase>();
                GridTerminalSystem.GetBlocksOfType(turrets);

                if (turrets.Count > 0)
                    Targetter = turrets[0];

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

                Configured = true;
            }
            catch { Configured = false; return; }


            GenerateDocks();
            //GenerateLatitudeSphereDeltas(Radius, Distance, true);
            GenerateTestFormation();
            //Load();

            LostEar = IGC.RegisterBroadcastListener(LostChannel);
            HubEar = IGC.RegisterBroadcastListener(HubChannel);
            //HubEar.SetMessageCallback();
            Runtime.UpdateFrequency = UpdateFrequency.Update1;
        }
        public void Main(string argument, UpdateType updateSource)
        {
            if (DEBUG_BREAK)
                return;

            RuntimeArguments(argument);
            MenuInput();
            GUIUpdate();
            Debugging();
            DebugTicker();
            AquireAiTargetedEntity();

            if (!Configured)
                return;

            UpdateHubWorldFrames();
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

                case "FOCUS":
                    HubSwarm.ToggleFocus();
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
                //foreach (PendingRequest request in ResponseQue)
                    //Storage += $"{request.ID}:{(int)request.Request}:{request.CurrentIndex}\n";

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
                    //PendingRequest request = new PendingRequest(long.Parse(raw2[0]), (RequestType)int.Parse(raw2[1]), int.Parse(raw2[2]));
                    //ResponseQue.Add(request);
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
