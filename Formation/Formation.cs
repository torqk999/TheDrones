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

        //  a:b:c:d:e:f
        //  worldPos    = (a,b,c)
        //  delta       = (d,e,f)

        #region FORM

        const string SpherePBName = "SPHERE_GEN";
        const string SourceName = "Antenna Dish";
        const string HubPBname = "HUB_PB";
        const string ShipControlName = "HUB_CONTROL";
        const char Split = ':';
        const float Radius = 1500;
        const float Distance = 20;
        const int LiteralCount = 4;
        const int FORM_SCALE_LIMIT = 5;
        const double OFFSET_INCREMENT = 0.0004d;

        bool Literal = true;
        bool Spin = false;
        double OffsetAngle = 0;
        int FormationScalePow;
        float FormationScaleVal;

        IMyTextSurface Debug;
        Vector3[] EquatorDeltas;
        Vector3[] SphereDeltas;
        Point[][] SuperDeltas;
        Vector3[] VanguardDeltas;
        Vector3[] FormationLiterals;
        //IMyTerminalBlock Target;
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

                    if(directional)
                        points.Add(new Vector3(deltaX, deltaZ, -deltaY));
                    else
                        points.Add(new Vector3(deltaX, deltaY, deltaZ));
                }
            }

            SphereDeltas = new Vector3[points.Count];
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

            for (int i = rings.Count - 1; i > - 1; i--) // work backwards through rings
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

            int pointCount = Literal? LiteralCount : (int)((Math.PI * 2 * radius) / distance);

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
        void GenerateFormationLiterals(IMyTerminalBlock source, ref Vector3[] formationDeltas, ref Vector3[] targetArray)
        {
            if (formationDeltas == null ||
                formationDeltas.Count() == 0)
                return;
            
            if (targetArray == null || targetArray.Count() != formationDeltas.Count())
                targetArray = new Vector3[formationDeltas.Count()];

            //string data = string.Empty;
            Me.CustomData = string.Empty;

            for (int i = 0; i < formationDeltas.Length; i++)
            {
                //Vector3 newVector = Deltas[i] + source.GetPosition(); // Raw world space formation
                Vector3 scaledVector = new Vector3(formationDeltas[i].X * FormationScaleVal,
                                                   formationDeltas[i].Y * FormationScaleVal,
                                                   formationDeltas[i].Z * FormationScaleVal);
                Vector3 relativeVector = DeTransformVectorRelative(GenerateOffsetMatrix(), scaledVector);
                //Vector3 relativeVector = DeTransformVectorRelative(source.WorldMatrix, scaledVector);
                Vector3 newVector = relativeVector + source.GetPosition();

                targetArray[i] = newVector;

                Me.CustomData += $"{newVector.X}{Split}{newVector.Y}{Split}{newVector.Z}";
                Me.CustomData += (i < formationDeltas.Length - 1) ? "\n" : "";
            }
            //Me.CustomData = data;
        }
        void GenerateSuperLiterals(IMyTerminalBlock source, Point[][] formationDeltas)
        {
            string data = string.Empty;

            for (int i = 0; i < SuperDeltas.Length; i++)
            {
                for (int j = 0; j < SuperDeltas[i].Length; j++)
                {
                    Vector3 scaledVector = new Vector3(formationDeltas[i][j].Delta.X * FormationScaleVal,
                                                       formationDeltas[i][j].Delta.Y * FormationScaleVal,
                                                       formationDeltas[i][j].Delta.Z * FormationScaleVal);
                    Vector3 relativeVector = DeTransformVectorRelative(source.WorldMatrix, scaledVector);
                    Vector3 newVector = relativeVector + source.GetPosition();

                    data += $"{newVector.X}{Split}{newVector.Y}{Split}{newVector.Z}{Split}{formationDeltas[i][j].PullIndex}";
                    data += (i < formationDeltas.Length - 1) ? "\n" : "";
                }
            }
            Me.CustomData = data;
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
        void UpdateSpinOffset()
        {
            if (!Spin)
                return;

            OffsetAngle += OFFSET_INCREMENT;
            OffsetAngle = (OffsetAngle >= (2 * Math.PI)) ? 0 : OffsetAngle;
        }

        MatrixD GenerateOffsetMatrix()
        {
            /* 
             * Keen Implementation:
             * Row 1: Right.x , Right.y , Right.z
             * Row 2: Up.x    , Up.y    , Up.z
             * Row 3: Back.x  , Back.y  , Back.z
            */

            double deltaZ = Math.Sin(OffsetAngle);
            double deltaX = Math.Cos(OffsetAngle);

            MatrixD output = new MatrixD(
                deltaX, 0, deltaZ,
                0, 1, 0,
                -deltaZ, 0, deltaX);

            return output;
        }

        public Program()
        {
            Control = (IMyShipController)GridTerminalSystem.GetBlockWithName(ShipControlName);
            Source = GridTerminalSystem.GetBlockWithName(SourceName);

            Me.CustomName = SpherePBName;
            Debug = Me.GetSurface(0);
            Debug.ContentType = ContentType.TEXT_AND_IMAGE;
            Debug.WriteText("", false);

            SetScaleValue();
            //GenerateThreePointVanguardDeltas(50, -10);    // Migrate user constants!!!
            //GenerateLatitudeSphereDeltas(Radius, Distance, true);
            //GenerateSuperSphereDeltas(Radius, Distance, true);
            GenerateEquatorDeltas(Radius, Distance);
            Load();

            Runtime.UpdateFrequency = UpdateFrequency.Update10;
        }

        public void Main(string argument, UpdateType updateSource)
        {
            switch (argument)
            {
                case "INCREASE":
                    ScaleFormation(true);
                    break;

                case "DECREASE":
                    ScaleFormation(false);
                    break;

                case "SPIN":
                    Spin = !Spin;
                    break;
            }

            if (Source != null)
            {
                

                //Debug.WriteText($"Velocity: {Control.GetShipVelocities().LinearVelocity}\n");
                /*
                for (int i = 0; i < 6; i++)
                {
                    Debug.WriteText($"{(Base6Directions.Direction)i} : {Base6Directions.Directions[i]}\n", true);

                }*/


                /*
                Debug.WriteText($"{Control.WorldMatrix.M11}\n" +
                    $"{Control.WorldMatrix.M12}\n" +
                    $"{Control.WorldMatrix.M13}\n" +
                    $"{Control.WorldMatrix.M21}\n" +
                    $"{Control.WorldMatrix.M22}\n" +
                    $"{Control.WorldMatrix.M23}\n" +
                    $"{Control.WorldMatrix.M31}\n" +
                    $"{Control.WorldMatrix.M32}\n" +
                    $"{Control.WorldMatrix.M33}");
                */
                UpdateSpinOffset();
                GenerateFormationLiterals(Source,ref EquatorDeltas,ref FormationLiterals);
                Debug.WriteText($"Point Coint:{FormationLiterals.Count()}\n" +
                    $"Spin: {Spin}\n" +
                    $"Offset: {OffsetAngle}");
                //GenerateSuperLiterals(Control, SuperDeltas);
            }
        }
        public void Save()
        {
            Storage = $"{Spin}:{OffsetAngle}";
        }

        public void Load()
        {
            try
            {
                string[] raw = Storage.Split(':');
                Spin = bool.Parse(raw[0]);
                OffsetAngle = double.Parse(raw[1]);
                Echo("Loaded");
            }
            catch
            {
                Echo("Nothing to load");
            }
        }

        #endregion


    }
}
