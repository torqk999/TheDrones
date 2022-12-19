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

        #region SPHERE

        const string SpherePBName = "SPHERE_GEN";
        const string ShipControlName = "HUB_CONTROL";
        const char Split = ':';
        const float Radius = 20;
        const float Distance = 7;

        IMyTextSurface Debug;
        Vector3[] SphereDeltas;
        Vector3[] VanguardDeltas;
        IMyTerminalBlock Target;
        IMyShipController Control;

        Vector3[] GenerateLatitudeSphereDeltas(float radius, float distance)
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

                    points.Add(new Vector3(deltaX, deltaY, deltaZ));
                }
            }

            Vector3[] output = new Vector3[points.Count];
            points.CopyTo(output);
            return output;
        }

        Vector3[] GenerateThreePointVanguardDeltas(float radius, float altitude)
        {
            Vector3[] output = new Vector3[6];

            for (int i = 0; i < 6; i++)
            {
                double ratio = (((double)i / 3) + ((double)1 / 6));
                double deltaZ = Math.Sin(ratio * Math.PI) * radius;
                double deltaX = Math.Cos(ratio * Math.PI) * radius;

                //int index = (i == 2) ? 4 : i;   // Maaaaaaaaaaaybe?
                //index = (i == 4) ? 5 : i;
                //index = (i == 5) ? 2 : i;

                output[i] = new Vector3(deltaX, altitude, deltaZ);
            }
            return output;
        }

        Vector3 DeNormalizeVectorRelative(MatrixD S, Vector3 N)
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

        void GenerateFormationLiterals(IMyTerminalBlock source, Vector3[] formationDeltas)
        {
            string data = string.Empty;

            for (int i = 0; i < formationDeltas.Length; i++)
            {
                //Vector3 newVector = Deltas[i] + source.GetPosition(); // Raw world space formation

                Vector3 relativeVector = DeNormalizeVectorRelative(source.WorldMatrix, formationDeltas[i]);
                Vector3 newVector = relativeVector + source.GetPosition();

                data += $"{newVector.X}{Split}{newVector.Y}{Split}{newVector.Z}";
                data += (i < formationDeltas.Length - 1) ? "\n" : "";
            }
            Me.CustomData = data;
        }

        public Program()
        {
            Control = (IMyShipController)GridTerminalSystem.GetBlockWithName(ShipControlName);

            Me.CustomName = SpherePBName;
            Debug = Me.GetSurface(0);
            Debug.ContentType = ContentType.TEXT_AND_IMAGE;
            Debug.WriteText("", false);

            VanguardDeltas = GenerateThreePointVanguardDeltas(50, -10);    // Migrate user constants!!!
            SphereDeltas = GenerateLatitudeSphereDeltas(Radius, Distance);

            Target = Control; // << For now...

            Runtime.UpdateFrequency = UpdateFrequency.Update10;
        }

        public void Main(string argument, UpdateType updateSource)
        {
            Debug.WriteText($"Velocity: {Control.GetShipVelocities().LinearVelocity}");

            if (Target != null)
                GenerateFormationLiterals(Target, SphereDeltas);
        }

        #endregion

        public void Save()
        {

        }
    }
}
