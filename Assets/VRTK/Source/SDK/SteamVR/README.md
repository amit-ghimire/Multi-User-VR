# SteamVR

## Instructions for using SteamVR

 * Import the [SteamVR Unity Plugin v2.2b4](https://github.com/ValveSoftware/steamvr_unity_plugin/releases/tag/2.2b4) from GitHub.
 * Follow the initial [Getting Started](/Assets/VRTK/Documentation/GETTING_STARTED.md) steps and then add the `[CameraRig]` prefab from the Unity project directory. [`\Assets\VRTK\Source\SDK\SteamVR\Prefabs\`] as a child of the SDK Setup GameObject.
 * Note that the `[CameraRig]` prefab from SteamVR has been customized to work with VRTK and SteamVR 2.2.x, and has different / additional scripts attached to the controllers (`SDK_SteamVRInputSource`) and camera (`SteamVR_Behaviour_Pose`) GameObject.
