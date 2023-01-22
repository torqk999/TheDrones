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
    class Junk : MyGridProgram
    {
        #region TODO
        /*  Automatic block detection and intuitive assumptions
         *  Rotational drift caused by Formation's matrix source
         *  GUI
         *  
         */

        #endregion

        #region TERMINOLOGY
        /*



         */
        #endregion

        #region OLD_SOURCE

        #region test
        List<IMyMotorBase> MotorBases;
        IMyCubeGrid TestGrid;

        public void WhatAreYou()
        {
            ScreenOut("PARTS", "", false);
            foreach (IMyMotorBase next in MotorBases)
            {
                ScreenOut("PARTS", next.CustomName + "\n", true);
            }

            ScreenOut("MATRIX", "", false);

            string[] matrix = TestGrid.WorldMatrix.ToString().Split(':');
            foreach (string next in matrix)
                ScreenOut("MATRIX", next + "\n", true);

        }
        public void DisplayInputValues(IMyShipController control)
        {
            ScreenOut("MOVE", "", false);   // Clear screen
            if (control != null)
            {
                ScreenOut("MOVE", control.CustomName + "\n", true);
                ScreenOut("MOVE", "Mouse-X: " + control.RotationIndicator.X + "\n", true);
                ScreenOut("MOVE", "Mouse-Y: " + control.RotationIndicator.Y + "\n", true);
                ScreenOut("MOVE", "Move-X: " + control.MoveIndicator.X + "\n", true);
                ScreenOut("MOVE", "Move-Y: " + control.MoveIndicator.Y + "\n", true);
                ScreenOut("MOVE", "Move-Z: " + control.MoveIndicator.Z + "\n", true);
            }
            else
                ScreenOut("MOVE", "Invalid Control Block!", true);
        }
        /*
        public void NormalizeGrid_M()
        {
            Vector3D Origin = BasisVectorBlocks[0].GetPosition();
            Vector3D Xpos = BasisVectorBlocks[1].GetPosition();
            Vector3D Ypos = BasisVectorBlocks[2].GetPosition();
            Vector3D Zpos = BasisVectorBlocks[3].GetPosition();

            TurretNormalMatrix[0] = VRageMath.Vector3D.Normalize(Origin - Xpos);
            TurretNormalMatrix[1] = VRageMath.Vector3D.Normalize(Origin - Ypos);
            TurretNormalMatrix[2] = VRageMath.Vector3D.Normalize(Origin - Zpos);
        }
        */
        Vector3D[] MatrixConversion(MatrixD matrix)
        {
            Vector3D[] result = new Vector3D[3];

            /*
             * Keen Implementation:
             * Row 1: Right.x , Right.y , Right.z
             * Row 2: Up.x    , Up.y    , Up.z
             * Row 3: Back.x  , Back.y  , Back.z
             * 
             * My Implementation:
             * Indice[0]: Right.x   , Right.y   , Right.z
             * Indice[1]: Forward.x , Forward.y , Forward.z
             * Indice[2]: Down.x    , Down.y    , Down.z      ?? Is this correct or did keen change there matrix layout  ??
             *                                                ?? Which Side is the pitch rotor on? Create logic for this ??
            */

            result[0] = new Vector3D(matrix.M11, matrix.M12, matrix.M13);
            result[1] = new Vector3D(-matrix.M31, -matrix.M32, -matrix.M33);
            result[2] = new Vector3D(-matrix.M21, -matrix.M22, -matrix.M23);

            return result;
        }
        public void DeltaTarget()
        {
            if (TurretYaw != null && TurretSensor != null)
            {
                Vector3D Origin = TurretPitch.GetPosition();
                Vector3D Target = TurretSensor.LastDetectedEntity.Position;
                TargetDeltaVector = Origin - Target;
            }
        }
        public void NormalizeTarget()
        {
            /* X,Y,Z = Normalized Vector Unit Coefficients
             * x,y,z = Delta Target Vector (raw World GPS)
             * a,b,c = Normalized X vector components (relative x,y,z)
             * d,e,f = Normalized Y ''
             * g,h,i = Normalized Z ''
            */

            Vector3D[] NM = TurretNormalMatrix;
            Vector3D NV = new Vector3D(); // new NormalizedVector
            Vector3D DV = TargetDeltaVector;

            // Z = (d(bz - cy) + e(cx - az) + f(ay - bx)) / (d(bi - ch) + e(cg - ai) + f(ah - bg))

            // Z = (d       * ((b       * z)    - (c       * y))    + e       * ((c       * x)    - (a       * z))    + f       * ((a       * y)    - (b       * x)))    / (d       * ((b       * i)       - (c       * h))       + e       * ((c       * g)       - (a       * i))       + f       * ((a       * h)       - (b       * g)))
            NV.Z = (NM[1].X * ((NM[0].Y * DV.Z) - (NM[0].Z * DV.Y)) + NM[1].Y * ((NM[0].Z * DV.X) - (NM[0].X * DV.Z)) + NM[1].Z * ((NM[0].X * DV.Y) - (NM[0].Y * DV.X))) / (NM[1].X * ((NM[0].Y * NM[2].Z) - (NM[0].Z * NM[2].Y)) + NM[1].Y * ((NM[0].Z * NM[2].X) - (NM[0].X * NM[2].Z)) + NM[1].Z * ((NM[0].X * NM[2].Y) - (NM[0].Y * NM[2].X)));

            // Y = (Z(hc - ib) + zb - yc) / (fb - ec)
            // Y = (Z(gb - ha) + ya - xb) / (ea - db)

            // Y = (Z    * ((h       * c)       - (i       * b))       + (z    * b)       - (y    * c))       / ((f       * b)       - (e       * c))
            NV.Y = (NV.Z * ((NM[2].Y * NM[0].Z) - (NM[2].Z * NM[0].Y)) + (DV.Z * NM[0].Y) - (DV.Y * NM[0].Z)) / ((NM[1].Z * NM[0].Y) - (NM[1].Y * NM[0].Z));

            // X = (x - (Yd + Zg)) / a
            // X = (y - (Ye + Zh)) / b
            // X = (z - (Yf + Zi)) / c

            // X = (x    - ((Y    * d)        + (Z    * g)))      / a
            NV.X = (DV.X - ((NV.Y * NM[1].X) + (NV.Z * NM[2].X))) / NM[0].X;

            TargetNormalizedVector = NV;
        }
        public void TargetBearings()
        {
            double yawAdjust = 0;
            if (TargetNormalizedVector.Y > 0)
            {
                if (TargetNormalizedVector.X < 0)
                    yawAdjust = 360;
            }
            else
                yawAdjust = 180;

            TargetRelativeYaw = (Math.Atan(TargetNormalizedVector.X / TargetNormalizedVector.Y) * RadToDeg) + yawAdjust;
            TargetRelativePitch = Math.Atan(TargetNormalizedVector.Z / Math.Sqrt(Math.Pow(TargetNormalizedVector.X, 2) + Math.Pow(TargetNormalizedVector.Y, 2))) * RadToDeg;
        }
        public void ResetBearings()
        {
            TurretYaw.SetValue("Velocity", 1);
        }
        public void YawRotationVelocity(IMyMotorStator rotor)
        {
            if (rotor != null)
            {
                double velocity = 0;
                double domainMin = 0;
                double domainMax = 0;
                double magnitude = 0;
                bool currentIsMin;
                int direction = 1;

                double currentAngle = rotor.Angle * RadToDeg;

                ScreenOut("TRACK", "RadToDeg: " + RadToDeg + "\n", true);
                ScreenOut("TRACK", "CurrentAng: " + currentAngle + "\n", true);

                if (currentAngle > 180)
                {
                    domainMax = currentAngle;
                    domainMin = currentAngle - 180;
                    currentIsMin = false;
                }
                else
                {
                    domainMax = currentAngle + 180;
                    domainMin = currentAngle;
                    currentIsMin = true;
                }

                if (TargetRelativeYaw > domainMin && TargetRelativeYaw <= domainMax)
                {
                    if (currentIsMin)
                    {
                        magnitude = TargetRelativeYaw - currentAngle;
                        direction = 1;
                    }
                    else
                    {
                        magnitude = currentAngle - TargetRelativeYaw;
                        direction = -1;
                    }
                }
                if (TargetRelativeYaw > domainMax)
                {
                    if (currentIsMin)
                    {
                        magnitude = (360 - TargetRelativeYaw) + currentAngle;
                        direction = -1;
                    }
                    else
                    {
                        magnitude = TargetRelativeYaw - currentAngle;
                        direction = 1;
                    }
                }
                if (TargetRelativeYaw <= domainMin)
                {
                    if (currentIsMin)
                    {
                        magnitude = currentAngle - TargetRelativeYaw;
                        direction = -1;
                    }
                    else
                    {
                        magnitude = (360 - currentAngle) + TargetRelativeYaw;
                        direction = 1;
                    }
                }


                velocity = Math.Pow(magnitude, VelocityAdjustPower) * direction;

                rotor.SetValueFloat("Velocity", (float)velocity);
            }
        }
        public void PitchRotationVelocity(IMyMotorStator rotor)
        {
            if (rotor != null)
            {
                double magnitude;
                double velocity;
                double adjustedPitch;
                double currentPitch;
                int direction = 1;

                currentPitch = rotor.Angle * RadToDeg;
                adjustedPitch = 90 - currentPitch;

                if (adjustedPitch < TargetRelativePitch)
                    direction = -1;

                magnitude = Math.Abs(adjustedPitch - TargetRelativePitch);
                velocity = Math.Pow(magnitude, VelocityAdjustPower) * direction;
                rotor.SetValueFloat("Velocity", (float)velocity);
            }
        }

        #endregion
        #region reference

        const string TurretSensorName = "SpotlightSensor";
        const string TurretYawName = "SpotlightYaw";
        const string TurretPitchName = "SpotlightPitch";
        const string TurretFocusName = "SpotlightFocus";
        const string TurretControlName = "SpotlightControl";

        IMySensorBlock TurretSensor;
        IMyMotorStator TurretYaw;
        IMyMotorStator TurretPitch;
        _Turret Sample;

        Vector3D[] TurretNormalMatrix = new Vector3D[3];
        Vector3D TargetDeltaVector = new Vector3D();
        Vector3D TargetNormalizedVector = new Vector3D();

        double TargetRelativeYaw;
        double TargetRelativePitch;
        double VelocityAdjustPower = 1;
        const double RadToDeg = 180 / Math.PI;

        public void ScreenOut(string target, string log, bool append)
        {
            foreach (IMyTextPanel nextPanel in Panels)
                if (nextPanel.CustomName.Contains(target))
                    nextPanel.WriteText(log, append);
        }
        #endregion
        #region automatic

        bool TrackingOn = true;
        public class _Turret
        {
            public string Name;
            public bool bHasTarget;

            public IMySensorBlock Sensor;
            public IMyMotorStator MotorYaw;
            public IMyMotorStator MotorPitch;
            public IMyTerminalBlock Focus;
            public IMyTerminalBlock Controls;
            public IMyTextSurface Debug;

            public Vector3D[] TurretNormalMatrix = new Vector3D[3];
            public Vector3D TargetDeltaVector = new Vector3D();
            public Vector3D TargetNormalizedVector = new Vector3D();

            public string Keyword = string.Empty;
            public MyDetectedEntityType EntityType;
            public MyDetectedEntityInfo Entity;

            public double TargetRelativeYaw = 0;
            public double TargetRelativePitch = 0;
            public double VelocityAdjustPower = 1;

            const double RadToDeg = 180 / Math.PI;

            public _Turret(IMyTextSurface debug, string name, MyDetectedEntityType entityType = MyDetectedEntityType.None)
            {
                Name = name;
                EntityType = entityType;
                Debug = debug;
            }
            public void UpdateTargetParamaters()
            {
                if (Controls == null)
                    return;

                string[] data = Controls.CustomData.Split('\n');

                /* Targeting Instructions:
                 * 
                 * @ = Keyword
                 * # = Type
                */

                foreach (string nextLine in data)
                {
                    char check = nextLine[0];
                    string label = nextLine.Remove(0, 1);

                    switch (check)
                    {
                        case '@':
                            Keyword = label;
                            break;

                        case '#':
                            foreach (MyDetectedEntityType type in Enum.GetValues(typeof(MyDetectedEntityType)))
                            {
                                if (type.ToString().Contains(label))
                                    EntityType = type;
                            }
                            break;
                    }
                }
            }
            public void TurretUpdate()
            {
                if (MotorPitch == null || MotorYaw == null || Sensor == null || Focus == null)
                    return;

                bHasTarget = CheckForTarget();
                ToggleFocus(bHasTarget);

                if (bHasTarget)
                {
                    MatrixConversion();
                    DeltaTarget();
                    NormalizeTarget();
                    //TargetBearings();
                }
                else
                    ResetBearings();

                YawRotationVelocity(MotorYaw);
                PitchRotationVelocity(MotorPitch);

            }
            void ToggleFocus(bool toggle)
            {
                try
                {
                    IMyFunctionalBlock function = (IMyFunctionalBlock)Focus;
                    function.Enabled = toggle;
                }
                catch
                {
                    // Error message here!
                }
            }
            void MatrixConversion()
            {
                /*
                 * Keen Implementation:
                 * Row 1: Right.x , Right.y , Right.z
                 * Row 2: Up.x    , Up.y    , Up.z
                 * Row 3: Back.x  , Back.y  , Back.z
                 * 
                 * My Implementation:
                 * Indice[0]: Right.x   , Right.y   , Right.z
                 * Indice[1]: Forward.x , Forward.y , Forward.z
                 * Indice[2]: Down.x    , Down.y    , Down.z      ?? Is this correct or did keen change there matrix layout ??
                 * 
                 * New Imp:
                 * Indice[0]: Right.x   , Right.y   , Right.z
                 * Indice[1]: Up.x      , Up.y      , Up.z
                 * Indice[2]: Forward.x , Forward.y , Forward.z
                */

                MatrixD matrix = MotorYaw.WorldMatrix;

                TurretNormalMatrix[0] = new Vector3D(matrix.M11, matrix.M12, matrix.M13);
                TurretNormalMatrix[1] = new Vector3D(matrix.M21, matrix.M22, matrix.M23);
                TurretNormalMatrix[2] = new Vector3D(-matrix.M31, -matrix.M32, -matrix.M33);

                //TurretNormalMatrix[0] = new Vector3D(matrix.M11, matrix.M12, matrix.M13);
                //TurretNormalMatrix[1] = new Vector3D(-matrix.M31, -matrix.M32, -matrix.M33);
                //TurretNormalMatrix[2] = new Vector3D(-matrix.M21, -matrix.M22, -matrix.M23);
            }
            void DeltaTarget()
            {
                Vector3D Origin = Focus.GetPosition();
                Vector3D Target = Entity.Position;
                TargetDeltaVector = Origin - Target;
            }
            void NormalizeTarget()
            {
                /* X,Y,Z = Normalized Vector Unit Coefficients
                 * x,y,z = Delta Target Vector (raw World GPS)
                 * a,b,c = Normalized X vector components (relative x,y,z)
                 * d,e,f = Normalized Y ''
                 * g,h,i = Normalized Z ''
                */

                Vector3D[] NM = TurretNormalMatrix;
                Vector3D NV = new Vector3D(); // new NormalizedVector
                Vector3D DV = TargetDeltaVector;

                // Z = (d(bz - cy) + e(cx - az) + f(ay - bx)) / (d(bi - ch) + e(cg - ai) + f(ah - bg))

                // Z = (d       * ((b       * z)    - (c       * y))    + e       * ((c       * x)    - (a       * z))    + f       * ((a       * y)    - (b       * x)))    / (d       * ((b       * i)       - (c       * h))       + e       * ((c       * g)       - (a       * i))       + f       * ((a       * h)       - (b       * g)))
                NV.Z = (NM[1].X * ((NM[0].Y * DV.Z) - (NM[0].Z * DV.Y)) + NM[1].Y * ((NM[0].Z * DV.X) - (NM[0].X * DV.Z)) + NM[1].Z * ((NM[0].X * DV.Y) - (NM[0].Y * DV.X))) / (NM[1].X * ((NM[0].Y * NM[2].Z) - (NM[0].Z * NM[2].Y)) + NM[1].Y * ((NM[0].Z * NM[2].X) - (NM[0].X * NM[2].Z)) + NM[1].Z * ((NM[0].X * NM[2].Y) - (NM[0].Y * NM[2].X)));

                // Y = (Z(hc - ib) + zb - yc) / (fb - ec)
                // Y = (Z(gb - ha) + ya - xb) / (ea - db)

                // Y = (Z    * ((h       * c)       - (i       * b))       + (z    * b)       - (y    * c))       / ((f       * b)       - (e       * c))
                NV.Y = (NV.Z * ((NM[2].Y * NM[0].Z) - (NM[2].Z * NM[0].Y)) + (DV.Z * NM[0].Y) - (DV.Y * NM[0].Z)) / ((NM[1].Z * NM[0].Y) - (NM[1].Y * NM[0].Z));

                // X = (x - (Yd + Zg)) / a
                // X = (y - (Ye + Zh)) / b
                // X = (z - (Yf + Zi)) / c

                // X = (x    - ((Y    * d)        + (Z    * g)))      / a
                NV.X = (DV.X - ((NV.Y * NM[1].X) + (NV.Z * NM[2].X))) / NM[0].X;

                TargetNormalizedVector = NV;
            }

            Vector3 NormalizeTarget2(MatrixD S, Vector3 D) // S = sourceBearing, D = targetWorldDelta
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

                //Vector3D[] NM = TurretNormalMatrix;
                Vector3D NV = new Vector3D(); // new NormalizedVector
                                              //Vector3D DV = TargetDeltaVector;

                // Z = (d(bz - cy) + e(cx - az) + f(ay - bx)) / (d(bi - ch) + e(cg - ai) + f(ah - bg))

                // Z = (d       * ((b       * z)    - (c       * y))    + e       * ((c       * x)    - (a       * z))    + f       * ((a       * y)    - (b       * x)))    / (d       * ((b       * i)       - (c       * h))       + e       * ((c       * g)       - (a       * i))       + f       * ((a       * h)       - (b       * g)))
                //NV.Z = (NM[1].X * ((NM[0].Y * DV.Z) - (NM[0].Z * DV.Y)) + NM[1].Y * ((NM[0].Z * DV.X) - (NM[0].X * DV.Z)) + NM[1].Z * ((NM[0].X * DV.Y) - (NM[0].Y * DV.X))) / (NM[1].X * ((NM[0].Y * NM[2].Z) - (NM[0].Z * NM[2].Y)) + NM[1].Y * ((NM[0].Z * NM[2].X) - (NM[0].X * NM[2].Z)) + NM[1].Z * ((NM[0].X * NM[2].Y) - (NM[0].Y * NM[2].X)));
                NV.Z = (S.M21 * ((S.M12 * D.Z) - (S.M13 * D.Y)) + S.M22 * ((S.M13 * D.X) - (S.M11 * D.Z)) + S.M23 * ((S.M11 * D.Y) - (S.M12 * D.X))) / (S.M21 * ((S.M12 * S.M33) - (S.M13 * S.M32)) + S.M22 * ((S.M13 * S.M31) - (S.M11 * S.M33)) + S.M23 * ((S.M11 * S.M32) - (S.M12 * S.M31)));


                // Y = (Z(hc - ib) + zb - yc) / (fb - ec)
                // Y = (Z(gb - ha) + ya - xb) / (ea - db)

                // Y = (Z    * ((h       * c)       - (i       * b))       + (z    * b)       - (y    * c))       / ((f       * b)       - (e       * c))
                //NV.Y = (NV.Z * ((NM[2].Y * NM[0].Z) - (NM[2].Z * NM[0].Y)) + (DV.Z * NM[0].Y) - (DV.Y * NM[0].Z)) / ((NM[1].Z * NM[0].Y) - (NM[1].Y * NM[0].Z));
                NV.Y = (NV.Z * ((S.M32 * S.M13) - (S.M33 * S.M12)) + (D.Z * S.M12) - (D.Y * S.M13)) / ((S.M23 * S.M12) - (S.M22 * S.M13));

                // X = (x - (Yd + Zg)) / a
                // X = (y - (Ye + Zh)) / b
                // X = (z - (Yf + Zi)) / c

                // X = (x    - ((Y    * d)        + (Z    * g)))      / a
                //NV.X = (DV.X - ((NV.Y * NM[1].X) + (NV.Z * NM[2].X))) / NM[0].X;
                NV.X = (D.X - ((NV.Y * S.M21) + (NV.Z * S.M31))) / S.M11;

                return NV;
            }
            void TargetBearingsOLD()
            {
                double yawAdjust = 0;
                if (TargetNormalizedVector.Y > 0)
                {
                    if (TargetNormalizedVector.X < 0)
                        yawAdjust = 360;
                }
                else
                    yawAdjust = 180;

                TargetRelativeYaw = (Math.Atan(TargetNormalizedVector.X / TargetNormalizedVector.Y) * RadToDeg) + yawAdjust;
                TargetRelativePitch = Math.Atan(TargetNormalizedVector.Z / Math.Sqrt(Math.Pow(TargetNormalizedVector.X, 2) + Math.Pow(TargetNormalizedVector.Y, 2))) * RadToDeg;
            }
            void ResetBearings()
            {
                TargetRelativeYaw = 0;
                TargetRelativePitch = 0;
            }
            void YawRotationVelocity(IMyMotorStator rotor)
            {
                if (rotor != null)
                {
                    double velocity = 0;
                    double domainMin = 0;
                    double domainMax = 0;
                    double magnitude = 0;
                    bool currentIsMin;
                    int direction = 1;

                    double currentAngle = rotor.Angle * RadToDeg;

                    //ScreenOut("TRACK", "RadToDeg: " + RadToDeg + "\n", true);
                    //ScreenOut("TRACK", "CurrentAng: " + currentAngle + "\n", true);

                    if (currentAngle > 180)
                    {
                        domainMax = currentAngle;
                        domainMin = currentAngle - 180;
                        currentIsMin = false;
                    }
                    else
                    {
                        domainMax = currentAngle + 180;
                        domainMin = currentAngle;
                        currentIsMin = true;
                    }

                    if (TargetRelativeYaw > domainMin && TargetRelativeYaw <= domainMax)
                    {
                        if (currentIsMin)
                        {
                            magnitude = TargetRelativeYaw - currentAngle;
                            direction = 1;
                        }
                        else
                        {
                            magnitude = currentAngle - TargetRelativeYaw;
                            direction = -1;
                        }
                    }
                    if (TargetRelativeYaw > domainMax)
                    {
                        if (currentIsMin)
                        {
                            magnitude = (360 - TargetRelativeYaw) + currentAngle;
                            direction = -1;
                        }
                        else
                        {
                            magnitude = TargetRelativeYaw - currentAngle;
                            direction = 1;
                        }
                    }
                    if (TargetRelativeYaw <= domainMin)
                    {
                        if (currentIsMin)
                        {
                            magnitude = currentAngle - TargetRelativeYaw;
                            direction = -1;
                        }
                        else
                        {
                            magnitude = (360 - currentAngle) + TargetRelativeYaw;
                            direction = 1;
                        }
                    }


                    velocity = Math.Pow(magnitude, VelocityAdjustPower) * direction;

                    rotor.SetValueFloat("Velocity", (float)velocity);
                }
            }
            void PitchRotationVelocity(IMyMotorStator rotor)
            {
                if (rotor != null)
                {
                    double magnitude;
                    double velocity;
                    double adjustedPitch;
                    double currentPitch;
                    int direction = 1;

                    currentPitch = rotor.Angle * RadToDeg;
                    adjustedPitch = 90 - currentPitch;

                    if (adjustedPitch < TargetRelativePitch)
                        direction = -1;

                    magnitude = Math.Abs(adjustedPitch - TargetRelativePitch);
                    velocity = Math.Pow(magnitude, VelocityAdjustPower) * direction;
                    rotor.SetValueFloat("Velocity", (float)velocity);
                }
            }
            bool CheckForTarget()
            {
                List<MyDetectedEntityInfo> entities = new List<MyDetectedEntityInfo>();
                Sensor.DetectedEntities(entities);
                //Entity = entities.Find(x => x.Type == EntityType && x.Name.Contains(Keyword));
                Entity = entities.Find(x => x.Name.Contains(Keyword));
                //int index = entities.FindIndex(x => x.Type == EntityType && x.Name.Contains(Keyword));
                int index = entities.FindIndex(x => x.Name.Contains(Keyword));
                Debug.WriteText("\nIndex: " + index, false);
                return (index > -1) ? true : false;
            }
        }

        public void BlockDetectionS()
        {
            Sample.Sensor = (IMySensorBlock)GetBlock(TurretSensorName);
            Sample.MotorPitch = (IMyMotorStator)GetBlock(TurretPitchName);
            Sample.MotorYaw = (IMyMotorStator)GetBlock(TurretYawName);
            Sample.Focus = GetBlock(TurretFocusName);
            Sample.Controls = GetBlock(TurretControlName);
        }
        public void AutoTrackTest()
        {
            //DisplayTrackingValues();

            if (TrackingOn)
                Sample.TurretUpdate();
        }
        public void DisplayTrackingValues()
        {
            ScreenOut("TRACK", "", false);   // Clear screen
            ScreenOut("TRACK", "Tracking: " + TrackingOn + "\n", true);
            ScreenOut("TRACK", "TargetIdentity: " + Sample.Sensor.LastDetectedEntity.Name + "\n", true);
            ScreenOut("TRACK", "TargetYaw: " + Sample.TargetRelativeYaw + "\n", true);
            ScreenOut("TRACK", "TargetPitch: " + Sample.TargetRelativePitch + "\n", true);
            ScreenOut("TRACK", "Normal-X: " + Sample.TargetNormalizedVector.X + "\n", true);
            ScreenOut("TRACK", "Normal-Y: " + Sample.TargetNormalizedVector.Y + "\n", true);
            ScreenOut("TRACK", "Normal-Z: " + Sample.TargetNormalizedVector.Z + "\n", true);
        }

        #endregion
        #region manual

        // TARGET VARIABLES //

        public string CONTROL = "Ex - Control";
        public string YAW = "Ex - Yaw";
        public string PITCH = "Ex - Pitch";
        public string ARM = "Ex - Arm";

        public float ARM_VELOCITY = 1;
        public int YAW_DIR = -1;
        public float YAW_MAX = 1;
        public int PITCH_DIR = -1;
        public float PITCH_MAX = 1;

        ///////////////////////////

        IMyShipController ArmControl;
        IMyMotorStator YawBase;
        IMyMotorStator PitchBase;
        IMyBlockGroup ArmGroup;

        List<IMyTextPanel> Panels = new List<IMyTextPanel>();
        List<IMyPistonBase> ArmPistons = new List<IMyPistonBase>();

        //IMyCubeGrid TestGrid;

        // METHODS //
        /// Helper Section

        IMyTerminalBlock GetBlock(string name)
        {
            return GridTerminalSystem.GetBlockWithName(name);
        }
        public float Clamp(float value, float min, float max)
        {
            float output;

            if (value < min)
                output = min;
            else if (value > max)
                output = max;
            else
                output = value;

            return output;
        }

        public void BlockDetectionM()
        {
            Panels.Clear();
            GridTerminalSystem.GetBlocksOfType(Panels);

            //TestGrid = Panels[0].CubeGrid;
        }
        public void BlockAssignmentM()
        {
            List<IMyShipController> controls = new List<IMyShipController>();
            GridTerminalSystem.GetBlocksOfType(controls);
            foreach (IMyShipController next in controls)
                next.IsMainCockpit = false;
            ArmControl = (IMyShipController)GetBlock(CONTROL);
            ArmControl.IsMainCockpit = true;
            YawBase = (IMyMotorStator)GetBlock(YAW);
            PitchBase = (IMyMotorStator)GetBlock(PITCH);
            ArmGroup = GridTerminalSystem.GetBlockGroupWithName(ARM);
            ArmPistons.Clear();

            List<IMyTerminalBlock> termBlocks = new List<IMyTerminalBlock>();
            ArmGroup.GetBlocks(termBlocks);

            foreach (IMyTerminalBlock next in termBlocks)
            {
                IMyPistonBase piston = (IMyPistonBase)next;
                if (piston != null)
                    ArmPistons.Add(piston);
            }
        }

        /// Main Section

        public void TestMotion(IMyShipController control)
        {
            if (control != null)
            {
                YawBase.SetValueFloat("Velocity", Clamp(control.RotationIndicator.Y * YAW_DIR, -YAW_MAX, YAW_MAX));
                PitchBase.SetValueFloat("Velocity", Clamp(control.MoveIndicator.X * PITCH_DIR, -PITCH_MAX, PITCH_MAX));

                foreach (IMyPistonBase next in ArmPistons)
                    next.SetValueFloat("Velocity", -(control.MoveIndicator.Z * ARM_VELOCITY));
            }
        }

        #endregion
        #region Main


        public void Main1(string argument, UpdateType updateSource)
        {
            //DisplayInputValues(ArmControl);
            //TestMotion(ArmControl);

            AutoTrackTest();
            Echo("Working:");
            Echo("\nCurrentTarget: " + Sample.Entity.Name);
            Echo("\nCurrentKeyword: " + Sample.Keyword);
            Echo("\nCurrentType: " + Sample.EntityType);

            switch (argument)
            {
                case "TRACK":
                    TrackingOn = !TrackingOn;
                    break;

                case "UPDATE":
                    Sample.UpdateTargetParamaters();
                    break;
            }

        }

        #endregion

        #endregion

        /*

         //SafeZoneTime();

            /*List<ITerminalAction> actions = new List<ITerminalAction>();
            List<ITerminalProperty> properties = new List<ITerminalProperty>();

            Zone.GetProperties(properties);
            Zone.GetActions(actions);

            Builder.Append("Actions:\n");
            foreach (ITerminalAction action in actions)
                Builder.Append($"{action.Name} : {action.Id}\n");

            Builder.Append("Properties:\n");
            foreach (ITerminalProperty property in properties)
                Builder.Append($"{property.TypeName} : {property.Id}\n");*/

        /*#region TEST


        0-3   : ID
        4-19  : MX
        20-22 : POS
        23-25 : LOOK
        26/27 : REQ/IND


        float[] MSG_BUFF = new float[28];

        Program()
        {
            StringBuilder builder = new StringBuilder();
            IMyTextSurface surface = Me.GetSurface(0);
            surface.ContentType = ContentType.TEXT_AND_IMAGE;
            long id = Me.EntityId;
            builder.Append(
                $"Packet encrypt: {GenerateMyIDpacket(ref builder, ref MSG_BUFF, id, 0)}\n" +
                $"================\n");

            builder.Append(
                $"Packet decrypt: {ExtractID(ref builder, ref MSG_BUFF, out id, 0)}\n" +
                $"================\n" +
                $"output: {id}");
            surface.WriteText(builder);
        }

        public void Main(string argument, UpdateType updateSource)
        {

        }

        static long lowEnd = (long)Math.Pow(2, 16) - 1;
        static long highEnd = (long)Math.Pow(2, 64) - 1 - lowEnd;

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
        static bool ExtractID(ref ImmutableArray<float> dataStream, out long output, int index)
        {
            output = 0;
            try
            {
                for (int i = 3; i > -1; i--)
                {
                    output += (long)dataStream[index + i];
                    output = i > 0 ? output << 16 : output;
                }
                return true;
            }
            catch { return false; }
        }

        #endregion*/





        /*#region TEST

        ///////////////////////////////////////
        string CargoName = "CargoNameGoesHere";
        ///////////////////////////////////////

        bool Configured = false;
        IMySafeZoneBlock Zone;
        IMyCargoContainer Cargo;
        IMyTextSurface Surface;
        StringBuilder Builder;

        static TimeSpan chipValue = new TimeSpan(2, 0, 0);
        string[] Buffer;
        string[] Checks = new string[]
        {
            "Next Upkeep in: ",
            "Zone Chips:"
        };

        public Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Update100;
            Builder = new StringBuilder();
            Surface = Me.GetSurface(0);
            Surface.ContentType = ContentType.TEXT_AND_IMAGE;

            Cargo = (IMyCargoContainer)GridTerminalSystem.GetBlockWithName(CargoName);
            if (Cargo == null)
                return;

            List<IMySafeZoneBlock> zones = new List<IMySafeZoneBlock>();
            GridTerminalSystem.GetBlocksOfType(zones);
            if (zones.Count < 1)
                return;

            Zone = zones[0];

            Configured = true;
        }

        private void SafeZoneTime(int chipCount)
        {
            Builder.Clear();
            Buffer = Zone.DetailedInfo.Split('\n');

            DateTime remainingCurrent = new DateTime(0);
            TimeSpan bulkChipTime = new TimeSpan(0);
            TimeSpan remainingChipTime = new TimeSpan(0);

            foreach (string nextLine in Buffer)
            {
                if (nextLine.Contains(Checks[0]))
                    remainingCurrent = DateTime.Parse(nextLine.Replace(Checks[0], ""));

                if (nextLine.Contains(Checks[1]))
                    bulkChipTime = new TimeSpan(chipValue.Ticks * chipCount);
            }

            remainingChipTime = bulkChipTime + remainingCurrent.TimeOfDay;

            Builder.Append(
                $"Remaining Chips: {chipCount}\n" +
                $"Remaining Days: {remainingChipTime.Days}\n" +
                $"Expiration Time: {DateTime.Now + remainingChipTime}");

            Surface.WriteText(Builder);
        }

        int ZoneChipCount()
        {
            List<MyInventoryItem> items = new List<MyInventoryItem>();
            Cargo.GetInventory().GetItems(items);
            MyFixedPoint count = 0;
            foreach (MyInventoryItem item in items)
                if (item.Type.ToString().Contains("Zone"))
                    count += item.Amount;
            return (int)count;
        }

        public void Main(string argument, UpdateType updateSource)
        {
            if (!Configured)
                return;

            SafeZoneTime(ZoneChipCount());
        }

        #endregion*/
    }



}
