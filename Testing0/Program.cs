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

        
        public class DefenseShields
        {
            private IMyTerminalBlock _block;

            private readonly Func<IMyTerminalBlock, RayD, Vector3D?> _rayIntersectShield;
            private readonly Func<IMyTerminalBlock, LineD, Vector3D?> _lineIntersectShield;
            private readonly Func<IMyTerminalBlock, Vector3D, bool> _pointInShield;
            private readonly Func<IMyTerminalBlock, float> _getShieldPercent;
            private readonly Func<IMyTerminalBlock, int> _getShieldHeat;
            private readonly Func<IMyTerminalBlock, float> _getChargeRate;
            private readonly Func<IMyTerminalBlock, int> _hpToChargeRatio;
            private readonly Func<IMyTerminalBlock, float> _getMaxCharge;
            private readonly Func<IMyTerminalBlock, float> _getCharge;
            private readonly Func<IMyTerminalBlock, float> _getPowerUsed;
            private readonly Func<IMyTerminalBlock, float> _getPowerCap;
            private readonly Func<IMyTerminalBlock, float> _getMaxHpCap;
            private readonly Func<IMyTerminalBlock, bool> _isShieldUp;
            private readonly Func<IMyTerminalBlock, string> _shieldStatus;
            private readonly Func<IMyTerminalBlock, IMyEntity, bool, bool> _entityBypass;
            // Fields below do not require SetActiveShield to be defined first.
            private readonly Func<IMyCubeGrid, bool> _gridHasShield;
            private readonly Func<IMyCubeGrid, bool> _gridShieldOnline;
            private readonly Func<IMyEntity, bool> _protectedByShield;
            private readonly Func<IMyEntity, IMyTerminalBlock> _getShieldBlock;
            private readonly Func<IMyTerminalBlock, bool> _isShieldBlock;
            private readonly Func<Vector3D, IMyTerminalBlock> _getClosestShield;
            private readonly Func<IMyTerminalBlock, Vector3D, double> _getDistanceToShield;
            private readonly Func<IMyTerminalBlock, Vector3D, Vector3D?> _getClosestShieldPoint;

            public void SetActiveShield(IMyTerminalBlock block) => _block = block; // AutoSet to TapiFrontend(block) if shield exists on grid.

            public DefenseShields(IMyTerminalBlock block)
            {
                _block = block;
                var delegates = _block.GetProperty("DefenseSystemsPbAPI")?.As<Dictionary<string, Delegate>>().GetValue(_block);
                if (delegates == null) return;

                _rayIntersectShield = (Func<IMyTerminalBlock, RayD, Vector3D?>)delegates["RayIntersectShield"];
                _lineIntersectShield = (Func<IMyTerminalBlock, LineD, Vector3D?>)delegates["LineIntersectShield"];
                _pointInShield = (Func<IMyTerminalBlock, Vector3D, bool>)delegates["PointInShield"];
                _getShieldPercent = (Func<IMyTerminalBlock, float>)delegates["GetShieldPercent"];
                _getShieldHeat = (Func<IMyTerminalBlock, int>)delegates["GetShieldHeat"];
                _getChargeRate = (Func<IMyTerminalBlock, float>)delegates["GetChargeRate"];
                _hpToChargeRatio = (Func<IMyTerminalBlock, int>)delegates["HpToChargeRatio"];
                _getMaxCharge = (Func<IMyTerminalBlock, float>)delegates["GetMaxCharge"];
                _getCharge = (Func<IMyTerminalBlock, float>)delegates["GetCharge"];
                _getPowerUsed = (Func<IMyTerminalBlock, float>)delegates["GetPowerUsed"];
                _getPowerCap = (Func<IMyTerminalBlock, float>)delegates["GetPowerCap"];
                _getMaxHpCap = (Func<IMyTerminalBlock, float>)delegates["GetMaxHpCap"];
                _isShieldUp = (Func<IMyTerminalBlock, bool>)delegates["IsShieldUp"];
                _shieldStatus = (Func<IMyTerminalBlock, string>)delegates["ShieldStatus"];
                _entityBypass = (Func<IMyTerminalBlock, IMyEntity, bool, bool>)delegates["EntityBypass"];
                _gridHasShield = (Func<IMyCubeGrid, bool>)delegates["GridHasShield"];
                _gridShieldOnline = (Func<IMyCubeGrid, bool>)delegates["GridShieldOnline"];
                _protectedByShield = (Func<IMyEntity, bool>)delegates["ProtectedByShield"];
                _getShieldBlock = (Func<IMyEntity, IMyTerminalBlock>)delegates["GetShieldBlock"];
                _isShieldBlock = (Func<IMyTerminalBlock, bool>)delegates["IsShieldBlock"];
                _getClosestShield = (Func<Vector3D, IMyTerminalBlock>)delegates["GetClosestShield"];
                _getDistanceToShield = (Func<IMyTerminalBlock, Vector3D, double>)delegates["GetDistanceToShield"];
                _getClosestShieldPoint = (Func<IMyTerminalBlock, Vector3D, Vector3D?>)delegates["GetClosestShieldPoint"];

                if (!IsShieldBlock()) _block = GetShieldBlock(_block.CubeGrid) ?? _block;
            }
            public Vector3D? RayIntersectShield(RayD ray) => _rayIntersectShield?.Invoke(_block, ray) ?? null;
            public Vector3D? LineIntersectShield(LineD line) => _lineIntersectShield?.Invoke(_block, line) ?? null;
            public bool PointInShield(Vector3D pos) => _pointInShield?.Invoke(_block, pos) ?? false;
            public float GetShieldPercent() => _getShieldPercent?.Invoke(_block) ?? -1;
            public int GetShieldHeat() => _getShieldHeat?.Invoke(_block) ?? -1;
            public float GetChargeRate() => _getChargeRate?.Invoke(_block) ?? -1;
            public float HpToChargeRatio() => _hpToChargeRatio?.Invoke(_block) ?? -1;
            public float GetMaxCharge() => _getMaxCharge?.Invoke(_block) ?? -1;
            public float GetCharge() => _getCharge?.Invoke(_block) ?? -1;
            public float GetPowerUsed() => _getPowerUsed?.Invoke(_block) ?? -1;
            public float GetPowerCap() => _getPowerCap?.Invoke(_block) ?? -1;
            public float GetMaxHpCap() => _getMaxHpCap?.Invoke(_block) ?? -1;
            public bool IsShieldUp() => _isShieldUp?.Invoke(_block) ?? false;
            public string ShieldStatus() => _shieldStatus?.Invoke(_block) ?? string.Empty;
            public bool EntityBypass(IMyEntity entity, bool remove = false) => _entityBypass?.Invoke(_block, entity, remove) ?? false;
            public bool GridHasShield(IMyCubeGrid grid) => _gridHasShield?.Invoke(grid) ?? false;
            public bool GridShieldOnline(IMyCubeGrid grid) => _gridShieldOnline?.Invoke(grid) ?? false;
            public bool ProtectedByShield(IMyEntity entity) => _protectedByShield?.Invoke(entity) ?? false;
            public IMyTerminalBlock GetShieldBlock(IMyEntity entity) => _getShieldBlock?.Invoke(entity) ?? null;
            public bool IsShieldBlock() => _isShieldBlock?.Invoke(_block) ?? false;
            public IMyTerminalBlock GetClosestShield(Vector3D pos) => _getClosestShield?.Invoke(pos) ?? null;
            public double GetDistanceToShield(Vector3D pos) => _getDistanceToShield?.Invoke(_block, pos) ?? -1;
            public Vector3D? GetClosestShieldPoint(Vector3D pos) => _getClosestShieldPoint?.Invoke(_block, pos) ?? null;
        }
        

        #region TEST

        IMyTextSurface Screen;
        IMyTextSurface Output;
        IMyTerminalBlock Core;
        string YourBlockName = "T1 Deep Space Facility Core HQ";
        string YourBlockName1 = "T1 Monitor Seat";
        //IMyTerminalBlock Shield;
        //string ShieldName = "[A] SHIELD0";
        IMyLaserAntenna Laser;

        void foo1(IMyTerminalBlock block)
        {
            var actions = new List<ITerminalAction>();
            block.GetActions(actions);
            string output = string.Empty;

            foreach (ITerminalAction action in actions)
                output += $"{action.Name} : {action.Id}\n";

            Screen.WriteText(output);
        }

        void foo(IMyTerminalBlock block)
        {
            List<ITerminalProperty> props = new List<ITerminalProperty>();
            block.GetProperties(props);
            foreach (ITerminalProperty prop in props)
                Screen.WriteText($"{prop.TypeName} : {prop.Id}\n", true);
        }

        void foo2()
        {

            Laser.SetTargetCoords("GPS:Laser Antenna 3:1037799.04:994414.76:1020453.96:#FF75C9F1:");
        }

        // Single : DS-CFit //

        IMyTextPanel Panel;

        public Program()
        {
            Screen = Me.GetSurface(0);
            Screen.ContentType = ContentType.TEXT_AND_IMAGE;
            Screen.WriteText("");

            Output = ((IMyCockpit)GridTerminalSystem.GetBlockWithName(YourBlockName1)).GetSurface(0);
            Output.ContentType = ContentType.TEXT_AND_IMAGE;

            Core = GridTerminalSystem.GetBlockWithName(YourBlockName);

            Runtime.UpdateFrequency = UpdateFrequency.Update10;

            IMyTerminalBlock foobar = GridTerminalSystem.GetBlockWithName(YourBlockName);
            if (foobar != null)
                foo(foobar);
        }

        public void Main(string argument, UpdateType updateSource)
        {
            //Output.WriteText("test");
            Output.WriteText($"Current Core Broadcast Radius:\n{Core.GetValueFloat("Radius")}");
        }

        #endregion

        public void Save()
        {

        }
    }
}
