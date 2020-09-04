// SteamVR InputSource|SDK_SteamVR|006
// DISCLAIMER: the code changes herein (for compatability with SteamVR Plugin 2.2.x) were created by a third party (WildStyle69) outside of VRTK, and are unsupported. VRTK takes no responsibility for the usage of this code, nor will provide any official support via GitHub or Slack.
namespace VRTK
{
    using UnityEngine;
    using Valve.VR;

    /// <summary>
    /// SDK SteamVR Input Source handles input commmunication with the SteamVR Input System. 
    /// </summary>
    ///<remarks>
    /// This class assumes the default action set and related SteamVR (SteamVR Plugin 2.2.x) Input System generated classes are available in a configuration that maps to the correct variable values (a default `actions.json` should be included in the project root).
    /// </remarks>
    public class SDK_SteamVRInputSource : MonoBehaviour
    {
        #region Variables

        // Variables
        // ----------
        public SteamVR_Input_Sources HandType;
        //
        // SteamVR Input Actions
        public SteamVR_Action_Vibration HapticAction;
        //
        public SteamVR_ActionSet ActionSet;
        //
        public SteamVR_Action_Skeleton Skeleton;
        //
        public SteamVR_Action_Boolean ButtonOneTouch;
        //
        public SteamVR_Action_Boolean ButtonOneClick;
        //
        public SteamVR_Action_Boolean ButtonTwoTouch;
        //
        public SteamVR_Action_Boolean ButtonTwoClick;
        //
        public SteamVR_Action_Boolean ButtonStartMenuClick;
        //
        public SteamVR_Action_Single Grip;
        //
        public SteamVR_Action_Boolean GripTouch;
        //
        public SteamVR_Action_Boolean GripButton;
        //
        public SteamVR_Action_Single Trigger;
        //
        public SteamVR_Action_Boolean TriggerTouch;
        //
        public SteamVR_Action_Boolean TriggerButton;
        //
        public SteamVR_Action_Boolean TouchpadButton;
        //
        public SteamVR_Action_Boolean TouchpadTouch;
        //
        public SteamVR_Action_Vector2 TouchpadPosition;
        //
        public SteamVR_Action_Boolean TouchpadTwoButton;
        //
        public SteamVR_Action_Boolean TouchpadTwoTouch;
        //
        public SteamVR_Action_Vector2 TouchpadTwoPosition;
        //
        // Hair Trigger
        public float HairTriggerDelta = 0.1f; // amount trigger must be pulled or released to change state
        //
        private float _hairTriggerLimit;
        private bool _hairTriggerState;
        private bool _hairTriggerPrevState;
        //
        // State
        private int _prevFrameCount = -1;
        private SDK_BaseController.ControllerType _currentControllerType = SDK_BaseController.ControllerType.Undefined;
        //>
        
        #endregion Variables

        #region Unity Methods

        /// <summary>
        /// On Enable
        /// </summary>
        void OnEnable()
        {
            // Activate action set
            ActionSet.Activate();
        }
        //-->

        /// <summary>
        /// Update
        /// </summary>
        void Update ()
        {
            if (Time.frameCount != _prevFrameCount)
            {
                _prevFrameCount = Time.frameCount;

                var system = OpenVR.System;
                if (system != null)
                {
                    UpdateHairTrigger();
                }
            }
        }
        //-->

        #endregion Unity Methods

        #region Properties

        /// <summary>
        /// Current Hand Type
        /// </summary>
        public SteamVR_Input_Sources CurrentHandType
        {
            get { return HandType; }
            set { HandType = value; }
        }
        //-->

        /// <summary>
        /// Current ControllerType
        /// </summary>
        public SDK_BaseController.ControllerType CurrentControllerType
        {
            get { return _currentControllerType; }
            set {_currentControllerType = value; }
        }
        //>

        /// <summary>
        /// Current Input Has Fingers
        /// </summary>
        public bool CurrentInputHasFingers {
            get
            {
                return (_currentControllerType == SDK_BaseController.ControllerType.SteamVR_ValveKnuckles);
            }
        }
        //>

        #endregion Properties

        #region Input Methods

        /// <summary>
        /// Get Axis Delta
        /// </summary>
        /// <param name="buttonType"></param>
        /// <returns></returns>
        public float GetAxisDelta(SDK_BaseController.ButtonTypes buttonType)
        {
            switch (buttonType)
            {
                case SDK_BaseController.ButtonTypes.Trigger:
                case SDK_BaseController.ButtonTypes.TriggerHairline:
                    return Trigger[HandType].delta;
            }

            return 0f;
        }
        //-->

        /// <summary>
        /// Get Button Sense Axis
        /// </summary>
        /// <param name="buttonType"></param>
        /// <returns></returns>
        public float GetButtonSenseAxis(SDK_BaseController.ButtonTypes buttonType)
        {
            switch (buttonType)
            {
                case SDK_BaseController.ButtonTypes.Grip:
                    return Grip[HandType].axis;

                case SDK_BaseController.ButtonTypes.Trigger:
                    return Trigger[HandType].axis;
            }

            return 0f;
        }
        //-->

        /// <summary>
        /// Get Finger Curl
        /// </summary>
        /// <param name="buttonType"></param>
        /// <returns></returns>
        public float GetFingerCurl(SDK_BaseController.ButtonTypes buttonType)
        {
            switch (buttonType)
            {
                case SDK_BaseController.ButtonTypes.MiddleFinger:
                    return Skeleton.middleCurl;

                case SDK_BaseController.ButtonTypes.RingFinger:
                    return Skeleton.ringCurl;

                case SDK_BaseController.ButtonTypes.PinkyFinger:
                    return Skeleton.pinkyCurl;
            }

            return 0f;
        }
        //-->

        /// <summary>
        /// Get Touch Pad Axis
        /// </summary>
        /// <param name="buttonType"></param>
        /// <returns></returns>
        public Vector2 GetTouchPadAxis(SDK_BaseController.ButtonTypes buttonType)
        {
            switch (buttonType)
            {
                case SDK_BaseController.ButtonTypes.Touchpad:
                    return TouchpadPosition != null ? TouchpadPosition[HandType].axis : Vector2.zero;
                    
                case SDK_BaseController.ButtonTypes.TouchpadTwo:
                    return TouchpadTwoPosition != null ? TouchpadTwoPosition[HandType].axis : Vector2.zero;

                default:
                    return Vector2.zero;
            }
        }
        //-->

        /// <summary>
        /// Get Button State
        /// </summary>
        /// <param name="buttonType"></param>
        /// <param name="pressType"></param>
        /// <returns></returns>
        public bool GetButtonState(SDK_BaseController.ButtonTypes buttonType, SDK_BaseController.ButtonPressTypes pressType)
        {
            SteamVR_Action_Boolean buttonAction = null;

            if (!IsButtonActionTouch(pressType))
            {
                // Button Press, Press Down, Press Up
                switch (buttonType)
                {
                    case SDK_BaseController.ButtonTypes.Trigger:
                        buttonAction = TriggerButton;
                        break;

                    case SDK_BaseController.ButtonTypes.Grip:
                        buttonAction = GripButton;
                        break;

                    case SDK_BaseController.ButtonTypes.Touchpad:
                        buttonAction = TouchpadButton;
                        break;

                    case SDK_BaseController.ButtonTypes.ButtonOne:
                        buttonAction = ButtonOneClick;
                        break;

                    case SDK_BaseController.ButtonTypes.ButtonTwo:
                        buttonAction = ButtonTwoClick;
                        break;
                    
                    case SDK_BaseController.ButtonTypes.StartMenu:
                        buttonAction = ButtonStartMenuClick;
                        break;

                    case SDK_BaseController.ButtonTypes.TouchpadTwo:
                        buttonAction = TouchpadTwoButton;
                        break;
                }
            }
            else
            {
                // Button Touch, Touch Down, Touch Up
                switch (buttonType)
                {
                    case SDK_BaseController.ButtonTypes.Trigger:
                        buttonAction = TriggerTouch;
                        break;

                    case SDK_BaseController.ButtonTypes.Grip:
                        buttonAction = GripTouch;
                        break;

                    case SDK_BaseController.ButtonTypes.Touchpad:
                        buttonAction = TouchpadTouch;
                        break;

                    case SDK_BaseController.ButtonTypes.ButtonOne:
                        buttonAction = ButtonOneTouch;
                        break;

                    case SDK_BaseController.ButtonTypes.ButtonTwo:
                        buttonAction = ButtonTwoTouch;
                        break;

                    case SDK_BaseController.ButtonTypes.TouchpadTwo:
                        buttonAction = TouchpadTwoTouch;
                        break;
                }
            }

            return buttonAction != null && GetBooleanButtonState(pressType, buttonAction);
        }
        //-->

        /// <summary>
        /// Get Boolean Button State
        /// </summary>
        /// <param name="pressType"></param>
        /// <param name="buttonAction"></param>
        /// <returns></returns>
        public bool GetBooleanButtonState(SDK_BaseController.ButtonPressTypes pressType, SteamVR_Action_Boolean buttonAction)
        {
            if (buttonAction == null)
            {
                return false;
            }

            switch (pressType)
            {
                case SDK_BaseController.ButtonPressTypes.Touch:
                case SDK_BaseController.ButtonPressTypes.Press:
                    return buttonAction[HandType].state;

                case SDK_BaseController.ButtonPressTypes.TouchDown:
                case SDK_BaseController.ButtonPressTypes.PressDown:
                    return buttonAction[HandType].stateDown;

                case SDK_BaseController.ButtonPressTypes.TouchUp:
                case SDK_BaseController.ButtonPressTypes.PressUp:
                    return buttonAction[HandType].stateUp;

                default:
                    return false;
            }
        }
        //-->

        /// <summary>
        /// Is Button Action Touch?
        /// </summary>
        /// <param name="pressType"></param>
        /// <returns></returns>
        private bool IsButtonActionTouch(SDK_BaseController.ButtonPressTypes pressType)
        {
            return (pressType != SDK_BaseController.ButtonPressTypes.Press)
                   && (pressType != SDK_BaseController.ButtonPressTypes.PressDown)
                   && (pressType != SDK_BaseController.ButtonPressTypes.PressUp);
        }
        //-->

        #endregion Input Methods

        #region Hair Trigger Methods

        /// <summary>
        /// Update Hair Trigger
        /// </summary>
        /// <remarks>Code adapted from <see href="https://github.com/ValveSoftware/openvr/blob/master/samples/unity_keyboard_sample/Assets/SteamVR/Scripts/SteamVR_Controller.cs">SteamVR_Controller</see> class.</remarks>
        private void UpdateHairTrigger()
        {
            _hairTriggerPrevState = _hairTriggerState;
            var value = Trigger[HandType].axis;
            if (_hairTriggerState)
            {
                if (value < _hairTriggerLimit - HairTriggerDelta || value <= 0.0f)
                    _hairTriggerState = false;
            }
            else
            {
                if (value > _hairTriggerLimit + HairTriggerDelta || value >= 1.0f)
                    _hairTriggerState = true;
            }
            _hairTriggerLimit = _hairTriggerState ? Mathf.Max(_hairTriggerLimit, value) : Mathf.Min(_hairTriggerLimit, value);
        }
        //-->

        /// <summary>
        /// Get Hair Trigger
        /// </summary>
        /// <returns></returns>
        public bool GetHairTrigger()
        {
            return _hairTriggerState;
        }
        //-->

        /// <summary>
        /// Get Hair Trigger Down
        /// </summary>
        /// <returns></returns>
        public bool GetHairTriggerDown()
        {
            return _hairTriggerState && !_hairTriggerPrevState;
        }
        //-->

        /// <summary>
        /// Get Hair Trigger Up
        /// </summary>
        /// <returns></returns>
        public bool GetHairTriggerUp()
        {
            return !_hairTriggerState && _hairTriggerPrevState;
        }
        //-->

        #endregion Hair Trigger Methods

        #region Haptic Methods

        /// <summary>
        /// Trigger Haptic Pulse
        /// </summary>
        /// <param name="amplitude"></param>
        /// <param name="duration"></param>
        /// <param name="frequency"></param>
        public void TriggerHapticPulse(float amplitude = 1f, float duration = 1f, float frequency = 150f)
        {
            HapticAction.Execute(0, duration, frequency, amplitude, HandType);
        }
        //-->

        #endregion Haptic Methods
    }
}
