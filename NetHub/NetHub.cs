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

        IMyTextSurface HubScreen;
        string debugBin = string.Empty;

        const string ControlChannel = "SKY_NET1";
        const string ServiceChannel = "Overlord";
        const string StringChannel = "SKY_NETchar[]";

        IMyBroadcastListener ControlEar;
        IMyBroadcastListener ServiceEar;
        IMyTextSurface DroneDebugScreen;
        const string DDSname = "DRONE_DEBUG";

        const int MAX_MESSAGE_PROCESS_COUNT = 100;
        //const int MSG_TIMEOUT = 100;

        // Op-Codes
        enum ControlCode
        {
            OFF,
            ON,
            PING,
            CLUSTER,
            DE_CLUSTER,
        }
        enum IDcode
        {
            LEAD,
            REQ_DEBUG,
            LINK_SERVICE,
        }
        enum ClusterCode
        {

        }
        enum IndexCode
        {
            /* REQ = request
             * APR = approved
             * DEN = denied
             */

            REQ_JOIN,
            APR_JOIN,
            DEN_JOIN,
            REQ_TAIL
        }
        /*
        public enum DroneCommand
        {

        }
        */
        bool bSetupComplete;
        bool bSpin = false;

        double[] Rotations =
        {
            0,
            0,
            0
        };
        const double Y_INCREMENT = 0.0001d;

        MatrixD FormationMatrix;
        Vector3 Center;
        Vector3 Root;

        float[] MessageBuffer = new float[16];

        // Callers
        void SendMsgDroneData(long id, ref float[] buffer)
        {
            IGC.SendBroadcastMessage($"DRONE_{id}", ImmutableArray.Create(buffer));
        }
        void SendMsgDroneString(long id, string buffer)
        {
            IGC.SendBroadcastMessage($"DRONE_{id}", buffer);
        }
        void SendMsgSwarm()
        {

        }

        // Listeners
        void RcvMsgControl(MyIGCMessage message)
        {

        }
        void RcvMsgService(MyIGCMessage message)
        {
            //short 
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

        void ConstructFormMtx(ref double[] rotations, ref MatrixD FM)
        {
            double SinX = Math.Sin(rotations[0]);
            double CosX = Math.Cos(rotations[0]);
            double SinY = Math.Sin(rotations[1]);
            double CosY = Math.Cos(rotations[1]);
            double SinZ = Math.Sin(rotations[2]);
            double CosZ = Math.Cos(rotations[2]);

            Vector3 iR = new Vector3(CosZ, -SinZ, 0); // Roll
            Vector3 iU = new Vector3(0, CosX, -SinX); // Pitch
            Vector3 iB = new Vector3(-SinY, 0, CosY); // Yaw

            MatrixD matrixPitch = new MatrixD
                (1, 0, 0,
                 0, CosX, -SinX,
                 0, SinX, CosX);

            MatrixD matrixYaw = new MatrixD
                (CosY, 0, SinY,
                 0, 1, 0,
                 -SinY, 0, CosX);

            MatrixD matrixRoll = new MatrixD
                (CosZ, -SinZ, 0,
                 SinZ, CosZ, 0,
                 0, 0, 1);

            iR = TransformVectorRelative(matrixYaw, TransformVectorRelative(matrixPitch, iR));
            iU = TransformVectorRelative(matrixRoll, TransformVectorRelative(matrixYaw, iU));
            iB = TransformVectorRelative(matrixPitch, TransformVectorRelative(matrixRoll, iB));

            FM.M11 = iR.X; FM.M12 = iR.Y; FM.M13 = iR.Z;
            FM.M21 = iU.X; FM.M22 = iU.Y; FM.M23 = iU.Z;
            FM.M31 = iB.X; FM.M32 = iB.Y; FM.M33 = iB.Z;
        }

        void UpdateYRotation()
        {
            if (Rotations == null || Rotations.Count() < 3)
                return;

            Rotations[1] += Y_INCREMENT;
            Rotations[1] = Rotations[1] >= (2 * Math.PI) ? 0 : Rotations[1];
        }

        void IDinstruction(string arg)
        {
            // CODE:ID:INS
            string[] raw = arg.Split(':');
            long id = long.Parse(raw[1]);

            switch (raw[2])
            {
                case "REQ_DEBUG":
                    break;
            }
        }

        bool Setup()
        {
            try
            {
                ControlEar = IGC.RegisterBroadcastListener(ControlChannel);
                ServiceEar = IGC.RegisterBroadcastListener(ServiceChannel);

                string[] raw0 = Me.CustomData.Split(':');
                //GPS:Name:1002598.06:996484.24:1000713.38:#FFF1BF75:
                Center = new Vector3(float.Parse(raw0[2]), float.Parse(raw0[3]), float.Parse(raw0[4]));
                debugBin += "Planet centre established...\n";
                return true;
            }
            catch
            {
                debugBin += "Failed to aquire centre Vector3!\n";
                return false;
            }
        }

        public Program()
        {
            HubScreen = Me.GetSurface(0);
            HubScreen.ContentType = ContentType.TEXT_AND_IMAGE;
            HubScreen.WriteText("");

            bSetupComplete = Setup();

            Runtime.UpdateFrequency = UpdateFrequency.Update10;
        }
        public void Main(string argument, UpdateType updateSource)
        {
            if (!bSetupComplete)
                return;

            int msgCount = 0;
            while (ControlEar.HasPendingMessage &&
                msgCount < MAX_MESSAGE_PROCESS_COUNT)
            {

                msgCount++;
            }

            if (bSpin)
                UpdateYRotation();

            ConstructFormMtx(ref Rotations, ref FormationMatrix);

            if (argument.Contains("ID:"))
                IDinstruction(argument);

            else
                switch (argument)
                {
                    // Primary
                    case "PING":
                        break;

                    // Extra-curricular
                    case "SPIN":
                        bSpin = !bSpin;
                        break;
                }
        }
        public void Save()
        {

        }
    }
}
