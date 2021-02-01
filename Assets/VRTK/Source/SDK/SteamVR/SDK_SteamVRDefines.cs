// SteamVR Defines|SDK_SteamVR|001
// DISCLAIMER: the code changes herein (for compatability with SteamVR Plugin 2.2.x) were created by a third party (WildStyle69) outside of VRTK, and are unsupported. VRTK takes no responsibility for the usage of this code, nor will provide any official support via GitHub or Slack.
namespace VRTK
{
    using System;
    using System.Reflection;

    /// <summary>
    /// Handles all the scripting define symbols for the SteamVR SDK.
    /// </summary>
    public static class SDK_SteamVRDefines
    {
        /// <summary>
        /// The scripting define symbol for the SteamVR SDK.
        /// </summary>
        public const string ScriptingDefineSymbol = SDK_ScriptingDefineSymbolPredicateAttribute.RemovableSymbolPrefix + "SDK_STEAMVR";

        private const string BuildTargetGroupName = "Standalone";

        [SDK_ScriptingDefineSymbolPredicate(ScriptingDefineSymbol, BuildTargetGroupName)]
        [SDK_ScriptingDefineSymbolPredicate(SDK_ScriptingDefineSymbolPredicateAttribute.RemovableSymbolPrefix + "STEAMVR_PLUGIN_2_0_1_OR_NEWER", BuildTargetGroupName)]
        private static bool IsPluginVersion201OrNewer()
        {
            Type steamInputClass = VRTK_SharedMethods.GetTypeUnknownAssembly("Valve.VR.SteamVR_Input");
            if (steamInputClass == null)
            {
                return false;
            }

            return steamInputClass.GetMethod("IdentifyActionsFile", BindingFlags.Public | BindingFlags.Static) != null;
        }

        [SDK_ScriptingDefineSymbolPredicate(ScriptingDefineSymbol, BuildTargetGroupName)]
        [SDK_ScriptingDefineSymbolPredicate(SDK_ScriptingDefineSymbolPredicateAttribute.RemovableSymbolPrefix + "STEAMVR_PLUGIN_1_2_2_OR_NEWER", BuildTargetGroupName)]
        private static bool IsPluginVersion122OrNewer()
        {
            Type controllerManagerClass = VRTK_SharedMethods.GetTypeUnknownAssembly("Valve.VR.SteamVR_ControllerManager");
            if (controllerManagerClass == null)
            {
                return false;
            }

            return controllerManagerClass.GetMethod("SetUniqueObject", BindingFlags.NonPublic | BindingFlags.Instance) != null;
        }

        [SDK_ScriptingDefineSymbolPredicate(ScriptingDefineSymbol, BuildTargetGroupName)]
        [SDK_ScriptingDefineSymbolPredicate(SDK_ScriptingDefineSymbolPredicateAttribute.RemovableSymbolPrefix + "STEAMVR_PLUGIN_1_2_1_OR_NEWER", BuildTargetGroupName)]
        private static bool IsPluginVersion121OrNewer()
        {
            Type eventClass = VRTK_SharedMethods.GetTypeUnknownAssembly("Valve.VR.SteamVR_Events");
            if (eventClass == null)
            {
                return false;
            }

            MethodInfo systemMethod = eventClass.GetMethod("System", BindingFlags.Public | BindingFlags.Static);
            if (systemMethod == null)
            {
                return false;
            }

            ParameterInfo[] systemMethodParameters = systemMethod.GetParameters();
            if (systemMethodParameters.Length != 1)
            {
                return false;
            }

            return systemMethodParameters[0].ParameterType == VRTK_SharedMethods.GetTypeUnknownAssembly("Valve.VR.EVREventType");
        }

        [SDK_ScriptingDefineSymbolPredicate(ScriptingDefineSymbol, BuildTargetGroupName)]
        [SDK_ScriptingDefineSymbolPredicate(SDK_ScriptingDefineSymbolPredicateAttribute.RemovableSymbolPrefix + "STEAMVR_PLUGIN_1_2_0", BuildTargetGroupName)]
        private static bool IsPluginVersion120()
        {
            Type eventClass = VRTK_SharedMethods.GetTypeUnknownAssembly("Valve.VR.SteamVR_Events");
            if (eventClass == null)
            {
                return false;
            }

            MethodInfo systemMethod = eventClass.GetMethod("System", BindingFlags.Public | BindingFlags.Static);
            if (systemMethod == null)
            {
                return false;
            }

            ParameterInfo[] systemMethodParameters = systemMethod.GetParameters();
            if (systemMethodParameters.Length != 1)
            {
                return false;
            }

            return systemMethodParameters[0].ParameterType == typeof(string);
        }

        [SDK_ScriptingDefineSymbolPredicate(ScriptingDefineSymbol, BuildTargetGroupName)]
        [SDK_ScriptingDefineSymbolPredicate(SDK_ScriptingDefineSymbolPredicateAttribute.RemovableSymbolPrefix + "STEAMVR_PLUGIN_1_1_1_OR_OLDER", BuildTargetGroupName)]
        private static bool IsPluginVersion111OrOlder()
        {
            Type utilsClass = VRTK_SharedMethods.GetTypeUnknownAssembly("Valve.VR.SteamVR_Utils");
            if (utilsClass == null)
            {
                return false;
            }

            Type eventClass = VRTK_SharedMethods.GetNestedType(utilsClass, "Event");
            if (eventClass == null)
            {
                return false;
            }

            return eventClass.GetMethod("Listen", BindingFlags.Public | BindingFlags.Static) != null;
        }
    }
}