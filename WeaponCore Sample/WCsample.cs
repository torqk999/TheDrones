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
        /*#region AUTO_PROJ
        ////////////////////////////////////////////////////////////////
        const string GroupName = "PROJ_GROUP";
        ////////////////////////////////////////////////////////////////

        IMyBlockGroup Group;
        List<IMyProjector> Projectors = new List<IMyProjector>();
        IMyProjector ON;

        void MyEcho()
        {
            string projName = ON != null ? ON.CustomName : "NONE";
            Echo($"GroupFound: {Group != null}\n" +
                $"ProjectorCount: {Projectors.Count}\n" +
                $"CurrentlyActiveProjector: {projName}");
        }

        public Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Update10;
            Group = GridTerminalSystem.GetBlockGroupWithName(GroupName);
            if (Group == null)
                return;
            Group.GetBlocksOfType(Projectors);
            if (Projectors.Count() > 0)
                ON = Projectors[0];
        }

        public void Main(string argument, UpdateType updateSource)
        {
            MyEcho();

            IMyProjector candidate = null;

            foreach (IMyProjector proj in Projectors)
                if (proj != ON && proj.Enabled)
                {
                    candidate = proj;
                    break;
                }

            if (candidate != null)
            {
                ON = candidate;
                foreach (IMyProjector proj in Projectors)
                    if (proj != ON)
                        proj.Enabled = false;
            }
        }

        #endregion*/


        #region MAIN
        const string MyControllerCustomName = "PILOT";
        const string ShivaGroup = "shivas";
        IMyCockpit MyCockpit;
        IMyTextSurface[] MySurfaces = new IMyTextSurface[3];
        StringBuilder MyStringBuilder = new StringBuilder();
        bool Configured = false;
        bool Track = false;

        MyDetectedEntityInfo info;
        IMyTerminalBlock blk;
        IMyCameraBlock cam;

        void foo()
        {
            
        }

        enum Screen
        {
            WEAPONS = 0,
            TARGETS = 1
        }

        void CurrentTargets()
        {
            MyStringBuilder.Clear();
            bool ai = api.HasGridAi(Me.EntityId);
            MyStringBuilder.Append($"Checking for AI: {ai}\n");
            if (!ai)
            {
                MySurfaces[(int)Screen.TARGETS].WriteText(MyStringBuilder);
                return;
            }
                
            MyStringBuilder.Append($"Current Target: ");
            MyDetectedEntityInfo ? chk = api.GetAiFocus(Me.EntityId);

            if (chk == null)
            {
                MyStringBuilder.Append("Missing");
            }
            else
            {
                MyDetectedEntityInfo info = (MyDetectedEntityInfo)chk;
                MyStringBuilder.Append(
                    $"{info.Name} : {info.EntityId} \n" +
                    $"{info.HitPosition}\n" +
                    $"{info.Orientation}");
            }

            MySurfaces[(int)Screen.TARGETS].WriteText(MyStringBuilder);
        }


        void WeaponLoadOut()
        {
            //api.IsWeaponReadyToFire;
            //api.IsTargetValid;
            //api.IsTargetAligned;
            //api.IsInRange;
            //api.get
            
            MyStringBuilder.Clear();
            
            MyStringBuilder.Append(
                $"Weapons Load-out: {AllWeapons.Count} : {StaticWeapons.Count} : {TurretWeapons.Count}\n" +
                 "======================\n" +
                $"Static Definitions: {StaticWeapons.Count}\n");
            //MyStringBuilder.Append("|RDT|TV|TA|TIN|\n");
            for (int i = 0; i < StaticWeapons.Count; i++)
            {
                IMyTerminalBlock blk = StaticWeapons[i]; //= StaticWeapons.Find(x => x.BlockDefinition.SubtypeName == WeaponDefinitions[i].SubtypeName);
                MyDefinitionId def = StaticDefinitions.Find(x => x.SubtypeName == blk.BlockDefinition.SubtypeName);
                
                if (def == null)
                    continue;
                MyStringBuilder.Append($"{i}:{def.SubtypeName} =|= {blk.BlockDefinition}\n");
            }
            MyStringBuilder.Append("======================\n");
            MyStringBuilder.Append($"Turret Definitions: {TurretWeapons.Count}\n");
            for (int i = 0; i < TurretWeapons.Count; i++)
            {
                IMyTerminalBlock blk = TurretWeapons[i]; //= StaticWeapons.Find(x => x.BlockDefinition.SubtypeName == WeaponDefinitions[i].SubtypeName);
                MyDefinitionId def = StaticDefinitions.Find(x => x.SubtypeName == blk.BlockDefinition.SubtypeName);

                if (def == null)
                    continue;
                MyStringBuilder.Append($"{i}:{def.SubtypeName} =|= {blk.BlockDefinition}\n");
            }

            MySurfaces[(int)Screen.WEAPONS].WriteText(MyStringBuilder);
        }

        void BlockLoadOut()
        {
            MyStringBuilder.Clear();
            MyStringBuilder.Append("Static Blocks:\n");

            for (int i = 0; i < StaticWeapons.Count; i++)
            {
                IMyTerminalBlock blk = StaticWeapons[i];

                MyStringBuilder.Append($"{i}:{blk.CustomName}:{blk.EntityId}:{blk.BlockDefinition.GetHashCode()}\n");
            }
        }

        void TargetAssignment()
        {
            //api.SetTurretTargetTypes();
        }

        void ShivaTestFire()
        {
            List<IMyTerminalBlock> shivas = new List<IMyTerminalBlock>();
            GridTerminalSystem.GetBlockGroupWithName(ShivaGroup).GetBlocks(shivas);
            foreach (IMyTerminalBlock shiva in shivas)
                api.FireWeaponOnce(shiva);
        }

        void WeaponAssignment()
        {
            
            StaticWeapons.Clear();
            StaticDefinitions.Clear();
            //api.GetT
            api.GetAllCoreWeapons(AllDefinitions);
            //api.Get
            api.GetAllCoreStaticLaunchers(StaticDefinitions);
            api.GetAllCoreTurrets(TurretDefinitions);
            //api.GetAllCoreStaticLaunchers(WeaponDefinitions);
            definitionSubIds.Clear();
            AllDefinitions.ForEach(d => definitionSubIds.Add(d.SubtypeName));
            GridTerminalSystem.GetBlocksOfType<IMyTerminalBlock>(AllWeapons, b => definitionSubIds.Contains(b.BlockDefinition.SubtypeName));
            GridTerminalSystem.GetBlocksOfType<IMyTerminalBlock>(StaticWeapons, b => b.CubeGrid == Me.CubeGrid && definitionSubIds.Contains(b.BlockDefinition.SubtypeName));
            GridTerminalSystem.GetBlocksOfType<IMyTerminalBlock>(TurretWeapons, b => b.CubeGrid != Me.CubeGrid && definitionSubIds.Contains(b.BlockDefinition.SubtypeName));
            //StaticWeapons.ForEach(b => api.FireWeaponOnce(b));

        }

        void MyEcho()
        {
            MyStringBuilder.Clear();
            MyStringBuilder.Append($"Control: {MyCockpit != null}\n");
            if (MyCockpit == null)
                return;
            MyStringBuilder.Append($"WpnCoreApi: {api != null}\n");
            Echo(MyStringBuilder.ToString());
        }
        public Program()
        {
            MyCockpit = (IMyCockpit)GridTerminalSystem.GetBlockWithName(MyControllerCustomName);
            if (MyCockpit == null)
                return;

            api = new WcPbApi();
            api.Activate(Me);

            WeaponAssignment();

            for(int i = 0; i < 3; i++)
            {
                MySurfaces[i] = MyCockpit.GetSurface(i);
                MySurfaces[i].ContentType = ContentType.TEXT_AND_IMAGE;
            }
                
            Runtime.UpdateFrequency = UpdateFrequency.Update10;
            Configured = true;
        }

        public void Save()
        {
            // Called when the program needs to save its state. Use
            // this method to save your state to the Storage field
            // or some other means. 
            // 
            // This method is optional and can be removed if not
            // needed.
        }

        public static WcPbApi api;

        public List<IMyTerminalBlock> AllWeapons = new List<IMyTerminalBlock>();
        public List<IMyTerminalBlock> StaticWeapons = new List<IMyTerminalBlock>();
        public List<IMyTerminalBlock> TurretWeapons = new List<IMyTerminalBlock>();

        public List<MyDefinitionId> AllDefinitions = new List<MyDefinitionId>();
        public List<MyDefinitionId> StaticDefinitions = new List<MyDefinitionId>();
        public List<MyDefinitionId> TurretDefinitions = new List<MyDefinitionId>();

        public List<string> definitionSubIds = new List<string>();

        public void Main(string argument, UpdateType updateSource)
        {
            if (!Configured)
                return;

            UserCommands(argument);
            MyEcho();
            WeaponLoadOut();
            CurrentTargets();
        }

        private void UserCommands(string argument)
        {
            switch (argument)
            {
                case "SHIVA":
                    ShivaTestFire();
                    break;

                case "TRACK":
                    break;
            }
        }

        /*
         * WcPbAPI class. Reference: https://steamcommunity.com/sharedfiles/filedetails/?id=2178802013
         * It is highly recommended to delete unneeded api methods
         * Non-API functions:
         *  Activate(pbBlock)
         *  ApiAssign(delegates)
         *  AssignMethod(delegates,name,field)
         */
        public class WcPbApi
        {
            private Action<ICollection<MyDefinitionId>> _getCoreWeapons;
            private Action<ICollection<MyDefinitionId>> _getCoreStaticLaunchers;
            private Action<ICollection<MyDefinitionId>> _getCoreTurrets;
            private Func<IMyTerminalBlock, IDictionary<string, int>, bool> _getBlockWeaponMap;
            private Func<long, MyTuple<bool, int, int>> _getProjectilesLockedOn;
            private Action<IMyTerminalBlock, IDictionary<MyDetectedEntityInfo, float>> _getSortedThreats;
            private Func<long, int, MyDetectedEntityInfo> _getAiFocus;
            private Func<IMyTerminalBlock, long, int, bool> _setAiFocus;
            private Func<IMyTerminalBlock, int, MyDetectedEntityInfo> _getWeaponTarget;
            private Action<IMyTerminalBlock, long, int> _setWeaponTarget;
            private Action<IMyTerminalBlock, bool, int> _fireWeaponOnce;
            private Action<IMyTerminalBlock, bool, bool, int> _toggleWeaponFire;
            private Func<IMyTerminalBlock, int, bool, bool, bool> _isWeaponReadyToFire;
            private Func<IMyTerminalBlock, int, float> _getMaxWeaponRange;
            private Func<IMyTerminalBlock, ICollection<string>, int, bool> _getTurretTargetTypes;
            private Action<IMyTerminalBlock, ICollection<string>, int> _setTurretTargetTypes;
            private Action<IMyTerminalBlock, float> _setBlockTrackingRange;
            private Func<IMyTerminalBlock, long, int, bool> _isTargetAligned;
            private Func<IMyTerminalBlock, long, int, bool> _canShootTarget;
            private Func<IMyTerminalBlock, long, int, Vector3D?> _getPredictedTargetPos;
            private Func<IMyTerminalBlock, float> _getHeatLevel;
            private Func<IMyTerminalBlock, float> _currentPowerConsumption;
            private Func<MyDefinitionId, float> _getMaxPower;
            private Func<long, bool> _hasGridAi;
            private Func<IMyTerminalBlock, bool> _hasCoreWeapon;
            private Func<long, float> _getOptimalDps;
            private Func<IMyTerminalBlock, int, string> _getActiveAmmo;
            private Action<IMyTerminalBlock, int, string> _setActiveAmmo;
            private Action<Action<Vector3, float>> _registerProjectileAdded;
            private Action<Action<Vector3, float>> _unRegisterProjectileAdded;
            private Func<long, float> _getConstructEffectiveDps;
            private Func<IMyTerminalBlock, long> _getPlayerController;
            private Func<Sandbox.ModAPI.Ingame.IMyTerminalBlock, int, Matrix> _getWeaponAzimuthMatrix;
            private Func<Sandbox.ModAPI.Ingame.IMyTerminalBlock, int, Matrix> _getWeaponElevationMatrix;
            private Func<Sandbox.ModAPI.Ingame.IMyTerminalBlock, long, bool, bool, bool> _isTargetValid;
            private Func<Sandbox.ModAPI.Ingame.IMyTerminalBlock, int, MyTuple<Vector3D, Vector3D>> _getWeaponScope;
            private Func<Sandbox.ModAPI.Ingame.IMyTerminalBlock, MyTuple<bool, bool>> _isInRange;

            /*
             *  Tries to setup the Api if WC is loaded
             *  @param pbBlock: the block executing this script (WcPbAPI properties are stored in IMyProgrammableBlock)
             *  @throws Exception: thrown if WC is NOT loaded on function call. This should only be the case if WC broke
             *      or wasn't included in the world
             *  @return: ApiAssign()
             */
            public bool Activate(IMyTerminalBlock pbBlock)
            {
                var dict = pbBlock.GetProperty("WcPbAPI")?.As<IReadOnlyDictionary<string, Delegate>>().GetValue(pbBlock);
                if (dict == null) throw new Exception($"WcPbAPI failed to activate");
                return ApiAssign(dict);
            }

            /*
             *  Tries to assign the Api delegates to fields of this class
             *  @param delegates: read-only dictionary of delegates with string keys
             *  @return: true unless delegates is null
             */
            public bool ApiAssign(IReadOnlyDictionary<string, Delegate> delegates)
            {
                if (delegates == null)
                    return false;
                AssignMethod(delegates, "GetCoreWeapons", ref _getCoreWeapons);
                AssignMethod(delegates, "GetCoreStaticLaunchers", ref _getCoreStaticLaunchers);
                AssignMethod(delegates, "GetCoreTurrets", ref _getCoreTurrets);
                AssignMethod(delegates, "GetBlockWeaponMap", ref _getBlockWeaponMap);
                AssignMethod(delegates, "GetProjectilesLockedOn", ref _getProjectilesLockedOn);
                AssignMethod(delegates, "GetSortedThreats", ref _getSortedThreats);
                AssignMethod(delegates, "GetAiFocus", ref _getAiFocus);
                AssignMethod(delegates, "SetAiFocus", ref _setAiFocus);
                AssignMethod(delegates, "GetWeaponTarget", ref _getWeaponTarget);
                AssignMethod(delegates, "SetWeaponTarget", ref _setWeaponTarget);
                AssignMethod(delegates, "FireWeaponOnce", ref _fireWeaponOnce);
                AssignMethod(delegates, "ToggleWeaponFire", ref _toggleWeaponFire);
                AssignMethod(delegates, "IsWeaponReadyToFire", ref _isWeaponReadyToFire);
                AssignMethod(delegates, "GetMaxWeaponRange", ref _getMaxWeaponRange);
                AssignMethod(delegates, "GetTurretTargetTypes", ref _getTurretTargetTypes);
                AssignMethod(delegates, "SetTurretTargetTypes", ref _setTurretTargetTypes);
                AssignMethod(delegates, "SetBlockTrackingRange", ref _setBlockTrackingRange);
                AssignMethod(delegates, "IsTargetAligned", ref _isTargetAligned);
                AssignMethod(delegates, "CanShootTarget", ref _canShootTarget);
                AssignMethod(delegates, "GetPredictedTargetPosition", ref _getPredictedTargetPos);
                AssignMethod(delegates, "GetHeatLevel", ref _getHeatLevel);
                AssignMethod(delegates, "GetCurrentPower", ref _currentPowerConsumption);
                AssignMethod(delegates, "GetMaxPower", ref _getMaxPower);
                AssignMethod(delegates, "HasGridAi", ref _hasGridAi);
                AssignMethod(delegates, "HasCoreWeapon", ref _hasCoreWeapon);
                AssignMethod(delegates, "GetOptimalDps", ref _getOptimalDps);
                AssignMethod(delegates, "GetActiveAmmo", ref _getActiveAmmo);
                AssignMethod(delegates, "SetActiveAmmo", ref _setActiveAmmo);
                AssignMethod(delegates, "RegisterProjectileAdded", ref _registerProjectileAdded);
                AssignMethod(delegates, "UnRegisterProjectileAdded", ref _unRegisterProjectileAdded);
                AssignMethod(delegates, "GetConstructEffectiveDps", ref _getConstructEffectiveDps);
                AssignMethod(delegates, "GetPlayerController", ref _getPlayerController);
                AssignMethod(delegates, "GetWeaponAzimuthMatrix", ref _getWeaponAzimuthMatrix);
                AssignMethod(delegates, "GetWeaponElevationMatrix", ref _getWeaponElevationMatrix);
                AssignMethod(delegates, "IsTargetValid", ref _isTargetValid);
                AssignMethod(delegates, "GetWeaponScope", ref _getWeaponScope);
                AssignMethod(delegates, "IsInRange", ref _isInRange);
                return true;
            }

            /*
             *  Tries to assign delegate methods to fields while checking for identical types.
             *  @param delegates: read-only dictionary of delegates with string keys. If this is null field will be set to null
             *  @param name: name of the delegate to assign
             *  @param field: referenceto a field in this class, to assign the delegate to.
             *  @throws Exception: thrown if either name isn't pointing to a delegate in delegates or field and the delegate aren't of the same type
             */
            private void AssignMethod<T>(IReadOnlyDictionary<string, Delegate> delegates, string name, ref T field) where T : class
            {
                if (delegates == null)
                {
                    field = null;
                    return;
                }
                Delegate del;
                if (!delegates.TryGetValue(name, out del))
                    throw new Exception($"{GetType().Name} :: Couldn't find {name} delegate of type {typeof(T)}");
                field = del as T;
                if (field == null)
                    throw new Exception(
                        $"{GetType().Name} :: Delegate {name} is not type {typeof(T)}, instead it's: {del.GetType()}");
            }
            public void GetAllCoreWeapons(ICollection<MyDefinitionId> collection) => _getCoreWeapons?.Invoke(collection);
            public void GetAllCoreStaticLaunchers(ICollection<MyDefinitionId> collection) =>
                _getCoreStaticLaunchers?.Invoke(collection);
            public void GetAllCoreTurrets(ICollection<MyDefinitionId> collection) => _getCoreTurrets?.Invoke(collection);
            public bool GetBlockWeaponMap(IMyTerminalBlock weaponBlock, IDictionary<string, int> collection) =>
                _getBlockWeaponMap?.Invoke(weaponBlock, collection) ?? false;
            public MyTuple<bool, int, int> GetProjectilesLockedOn(long victim) =>
                _getProjectilesLockedOn?.Invoke(victim) ?? new MyTuple<bool, int, int>();
            public void GetSortedThreats(IMyTerminalBlock pbBlock, IDictionary<MyDetectedEntityInfo, float> collection) =>
                _getSortedThreats?.Invoke(pbBlock, collection);
            public MyDetectedEntityInfo? GetAiFocus(long shooter, int priority = 0) => _getAiFocus?.Invoke(shooter, priority);
            public bool SetAiFocus(IMyTerminalBlock pbBlock, long target, int priority = 0) =>
                _setAiFocus?.Invoke(pbBlock, target, priority) ?? false;
            public MyDetectedEntityInfo? GetWeaponTarget(IMyTerminalBlock weapon, int weaponId = 0) =>
                _getWeaponTarget?.Invoke(weapon, weaponId) ?? null;
            public void SetWeaponTarget(IMyTerminalBlock weapon, long target, int weaponId = 0) =>
                _setWeaponTarget?.Invoke(weapon, target, weaponId);
            public void FireWeaponOnce(IMyTerminalBlock weapon, bool allWeapons = true, int weaponId = 0) =>
                _fireWeaponOnce?.Invoke(weapon, allWeapons, weaponId);
            public void ToggleWeaponFire(IMyTerminalBlock weapon, bool on, bool allWeapons, int weaponId = 0) =>
                _toggleWeaponFire?.Invoke(weapon, on, allWeapons, weaponId);
            public bool IsWeaponReadyToFire(IMyTerminalBlock weapon, int weaponId = 0, bool anyWeaponReady = true,
                bool shootReady = false) =>
                _isWeaponReadyToFire?.Invoke(weapon, weaponId, anyWeaponReady, shootReady) ?? false;
            public float GetMaxWeaponRange(IMyTerminalBlock weapon, int weaponId) =>
                _getMaxWeaponRange?.Invoke(weapon, weaponId) ?? 0f;
            public bool GetTurretTargetTypes(IMyTerminalBlock weapon, IList<string> collection, int weaponId = 0) =>
                _getTurretTargetTypes?.Invoke(weapon, collection, weaponId) ?? false;
            public void SetTurretTargetTypes(IMyTerminalBlock weapon, IList<string> collection, int weaponId = 0) =>
                _setTurretTargetTypes?.Invoke(weapon, collection, weaponId);
            public void SetBlockTrackingRange(IMyTerminalBlock weapon, float range) =>
                _setBlockTrackingRange?.Invoke(weapon, range);
            public bool IsTargetAligned(IMyTerminalBlock weapon, long targetEnt, int weaponId) =>
                _isTargetAligned?.Invoke(weapon, targetEnt, weaponId) ?? false;
            public bool CanShootTarget(IMyTerminalBlock weapon, long targetEnt, int weaponId) =>
                _canShootTarget?.Invoke(weapon, targetEnt, weaponId) ?? false;
            public Vector3D? GetPredictedTargetPosition(IMyTerminalBlock weapon, long targetEnt, int weaponId) =>
                _getPredictedTargetPos?.Invoke(weapon, targetEnt, weaponId) ?? null;
            public float GetHeatLevel(IMyTerminalBlock weapon) => _getHeatLevel?.Invoke(weapon) ?? 0f;
            public float GetCurrentPower(IMyTerminalBlock weapon) => _currentPowerConsumption?.Invoke(weapon) ?? 0f;
            public float GetMaxPower(MyDefinitionId weaponDef) => _getMaxPower?.Invoke(weaponDef) ?? 0f;
            public bool HasGridAi(long entity) => _hasGridAi?.Invoke(entity) ?? false;
            public bool HasCoreWeapon(IMyTerminalBlock weapon) => _hasCoreWeapon?.Invoke(weapon) ?? false;
            public float GetOptimalDps(long entity) => _getOptimalDps?.Invoke(entity) ?? 0f;
            public string GetActiveAmmo(IMyTerminalBlock weapon, int weaponId) =>
                _getActiveAmmo?.Invoke(weapon, weaponId) ?? null;
            public void SetActiveAmmo(IMyTerminalBlock weapon, int weaponId, string ammoType) =>
                _setActiveAmmo?.Invoke(weapon, weaponId, ammoType);
            public void RegisterProjectileAddedCallback(Action<Vector3, float> action) =>
                _registerProjectileAdded?.Invoke(action);
            public void UnRegisterProjectileAddedCallback(Action<Vector3, float> action) =>
                _unRegisterProjectileAdded?.Invoke(action);
            public float GetConstructEffectiveDps(long entity) => _getConstructEffectiveDps?.Invoke(entity) ?? 0f;
            public long GetPlayerController(IMyTerminalBlock weapon) => _getPlayerController?.Invoke(weapon) ?? -1;
            public Matrix GetWeaponAzimuthMatrix(Sandbox.ModAPI.Ingame.IMyTerminalBlock weapon, int weaponId) =>
                _getWeaponAzimuthMatrix?.Invoke(weapon, weaponId) ?? Matrix.Zero;
            public Matrix GetWeaponElevationMatrix(Sandbox.ModAPI.Ingame.IMyTerminalBlock weapon, int weaponId) =>
                _getWeaponElevationMatrix?.Invoke(weapon, weaponId) ?? Matrix.Zero;
            public bool IsTargetValid(Sandbox.ModAPI.Ingame.IMyTerminalBlock weapon, long targetId, bool onlyThreats, bool checkRelations) =>
                _isTargetValid?.Invoke(weapon, targetId, onlyThreats, checkRelations) ?? false;
            public MyTuple<Vector3D, Vector3D> GetWeaponScope(Sandbox.ModAPI.Ingame.IMyTerminalBlock weapon, int weaponId) =>
                _getWeaponScope?.Invoke(weapon, weaponId) ?? new MyTuple<Vector3D, Vector3D>();
            public MyTuple<bool, bool> IsInRange(Sandbox.ModAPI.Ingame.IMyTerminalBlock block) =>
                _isInRange?.Invoke(block) ?? new MyTuple<bool, bool>();
        }
        #endregion
    }
}
