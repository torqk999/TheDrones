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

        /*
         TODO:

        Connector support
        Auto-poweroff assemblers
        reactor balancing
        production ingots?


         */

#region MAIN

        const string YourCoreName = "CoreNameGoesHere";
        const string YourLCDname = "LCDnameGoesHere";
        IMyTextSurface Surface;

        public Program()
        {
            Surface = Me.GetSurface(0);
            Surface.ContentType = ContentType.TEXT_AND_IMAGE;
            Surface.WriteText("");

            Runtime.UpdateFrequency = UpdateFrequency.Update10;
        }

        public void Main(string argument, UpdateType updateSource)
        {
            IMyTerminalBlock core = GridTerminalSystem.GetBlockWithName(YourCoreName);
            IMyTextPanel panel = (IMyTextPanel)GridTerminalSystem.GetBlockWithName(YourLCDname);

            if (core == null)
            {
                Echo("Didn't find core!");
                return;
            }

            if (panel == null)
            {
                Echo("Didn't find panel!");
                return;
            }

            panel.ContentType = ContentType.TEXT_AND_IMAGE;
            panel.Enabled = true;
            panel.Alignment = TextAlignment.CENTER;

            try
            {
                string output = $"Core Broadcast Radius:\n{core.GetValueFloat("Radius")}";
                Surface.WriteText(output);
                panel.WriteText(output);
                Echo("Updating...");
            }
            catch
            {
                Echo("Not a core block!");
            }            
        }

#endregion
    }
}
