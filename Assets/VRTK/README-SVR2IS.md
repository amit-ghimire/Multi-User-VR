# VRTK (3.3.0) and SteamVR (2.2.0) Input System

## Disclaimer / Important

* The code changes herein (for compatibility with SteamVR Plugin 2.2.0) were created by a third party (WildStyle69) outside of VRTK, and are unsupported.
* VRTK takes no responsibility for the usage of this code, nor will provide any official support via GitHub or Slack.
* Code changes were made with Unity version 2018.2.17f1 and tested with Unity versions 2018.2.17f1 and 2018.3.7f1.
* Be sure to read this document in full, as it contains important information, including details on installation and bugs / known issues.


## What is this?

* This is a hack and slash version of VRTK 3.3.0 that offers very basic support for the new SteamVR Input System.
* The goal was to get the existing VRTK code-base to communicate with the SteamVR Plugin 2.2.0 in an error free fashion.
* Existing VRTK button mappings and actions have been mapped to the new SteamVR 2.2.0 Input System via an intermediate `SDK_SteamVRInputSource` class.
* No additional functionality has been added at this time, that is outside of the scope / objectives for this initial conversion task.
* The main VRTK example scenes should mostly work, however not everything has been extensively tested.
* Legacy example scenes have not been updated / changed to add support, and are broken / will not work correctly at this time.
* If you decide to use this version of VRTK, please don't give the original VRTK authors, myself (WildStyle69) or (jimthegrim) a hard time, if something is bugged / broken.
* Reporting bugs / issues to me (WildStyle69) is appreciated, as then I can at least investigate and fix, if its in-scope.


## Overview on what's changed

 * Since the SteamVR Plugin hit version 2.0.1 compatibility with VRTK was broken, this was because the SteamVR controller / input architecture completely changed.
 * The new SteamVR Input System is based on actions, which are setup in JSON files and via the SteamVR Input window in Unity.
 * These JSON files are then used by the SteamVR Input System to generate C# code classes, which can be used to access your Action Sets via code.
 * In Unity, if SteamVR is open and you have controllers attached, you can open a binding UI, which displays a localhost web page for editing controller / button configurations.
 * Through this input binding web page, you edit and map actions to buttons, depending on the type of controller, i.e. Vive Wand or Knuckles controller.
 * There are additional debugging output windows provided by SteamVR which are useful when checking / debugging controller configurations and / or issues.
 * The code in this version of VRTK 3.3.0 already has included the required JSON files and generated action code classes, which should get you up and running.
 * Read more about the latest SteamVR Plugin changes on the Valve Software > GitHub release page and blog posts (linked from release notes).
   - [Valve Software > GitHub release page](https://github.com/ValveSoftware/steamvr_unity_plugin/releases)


## VRTK Installation

 * If an older version of VRTK is already installed, then delete the VRTK asset / folders, before attempting to install this new version.
 * If an older version of SteamVR Plugin is already installed, then delete the SteamVR asset / folders, before attempting to install the new version.
 * In your Unity project ensure that SteamVR 2.2.0 has been installed before installing this version of VRTK.
   - Download link for [SteamVR Plugin 2.2.0](https://github.com/ValveSoftware/steamvr_unity_plugin/releases/download/2.2.0/SteamVR_v220.unitypackage)
 * Next install VRTK, all you need to copy is the `VRTK` folder and contents, note this now contains additional binding files for the SteamVR Input System.
 * Important: the import bindings dialog window for SteamVR bindings will pop up multiple times during the VRTK import process, do not import at this time, just cancel.
 * Once VRTK has imported then Unity will recompile the project and related script / assemblies etc. Wait for this to complete.
 * Next, expand the VRTK folder and you should see a `SteamVR_VRTK_Actions` folder inside, right-click this folder and select `Reimport`.
 * The SteamVR Plugin import dialog window for SteamVR input bindings should display again, this time confirm and the import process should run.
 * If successful the process should import the VRTK partial bindings for the SteamVR Input System, create the related classes and copy the required files into your project.
 * You should then see the following project structure in Unity, using the files from this version of VRTK 3.3.0 in an empty Unity project, with SteamVR Plugin 2.2.0 installed.
 * Note that project root / is the parent folder that contains the 'Assets' folder, this is where the controller JSON files are stored by SteamVR.
   - `/actions.json`
   - `/bindings_holographic_controller.json`
   - `/bindings_knuckles.json`
   - `/bindings_oculus_touch.json`
   - `/bindings_vive_controller`
   - `/Assets/SteamVR`
   - `/Assets/SteamVR_Input`
   - `/Assets/VRTK`
* Additionally, there is a new prefab for [CameraRig], check the `/Assets/VRTK/Source/SDK/SteamVR/Prefabs/` folder, you'll need to use this if rebuilding the [VRTK_SDKSetups].


## Bugs / Known Issues

 * Currently there is a debug log entry in the `SteamVR_Action_Skeleton` class, around line 910, that needs to be disabled / commented out, as it throws once per controller on scene start (still investigating).
 * The `StartMenu` button binding in VRTK is not currently working with SteamVR Plugin 2.2.x and the new Input System. A developer @ Valve has confirmed this is expected, as access to this button has been depreciated (still investigating).
 * Oculus Touch controllers have a known mapping bug for buttons 1 and 2, this relates to the touch events not being bindable on the right controller from the SteamVR Input System (OpenVR bug - should get fixed).