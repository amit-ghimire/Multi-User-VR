// ----------------------------------------------------------------------------
// <copyright file="PushToTalkPrivateButton.cs" company="Exit Games GmbH">
// Photon Voice Demo for PUN- Copyright (C) Exit Games GmbH
// </copyright>
// <summary>
// Script that implements Push-To-Talk feature of Photon Voice for PUN
// </summary>
// <author>developer@photonengine.com</author>
// ----------------------------------------------------------------------------

#pragma warning disable 0649 // Field is never assigned to, and will always have its default value

namespace ExitGames.Demos.DemoPunVoice
{
    using Client.Photon.LoadBalancing;
    using UnityEngine;
    using UnityEngine.UI;

    [RequireComponent(typeof(Button))]
    [DisallowMultipleComponent]
    [SelectionBase]
    public class PushToTalkPrivateButton : MonoBehaviour
    {
        [SerializeField]
        private Button pushToTalkPrivateButton;
        [SerializeField]
        private Text buttonText;
        private PushToTalkScript pttScript;
        public int TargetActorNr;
        public byte AudioGroup;
        public bool Subscribed;
        private KeyCode keyCode;
        private KeyCode keyCode2;

        // Use this for initialization
        private void Start()
        {
            pttScript = FindObjectOfType<PushToTalkScript>();
            PhotonVoiceNetwork.Client.OnStateChangeAction += OnVoiceClientStateChanged;
        }


        /// <summary>Callback by the Voice Chat Client.</summary>
        /// <remarks>
        /// Unlike callbacks from PUN, this only updates you on the state of Voice.
        /// Voice will by default automatically enter a Voice room, when PUN joined one. That's why Joined state will happen.
        /// </remarks>
        /// <param name="state">The new state.</param>
        private void OnVoiceClientStateChanged(ClientState state)
        {
            if (pushToTalkPrivateButton != null)
            {
                switch (state)
                {
                    case ClientState.Joined:
                        pushToTalkPrivateButton.gameObject.SetActive(true);
                        Subscribed = Subscribed || Subscribe();
                        break;
                    default:
                        pushToTalkPrivateButton.gameObject.SetActive(false);
                        break;
                }
            }
        }

        public void SetAudioGroup(PhotonPlayer player)
        {
            if (!Subscribed)
            {
                TargetActorNr = player.ID;
                buttonText.text = string.Format("Talk-To-Player{0}", TargetActorNr);
                keyCode = KeyCode.Keypad0 + TargetActorNr;
                keyCode2 = KeyCode.Alpha0 + TargetActorNr;
                //Debug.LogWarningFormat("KeyCode:{0}({1})", keyCode, (int)keyCode); 
                int temp;
                if (PhotonNetwork.player.ID < TargetActorNr)
                {
                    temp = TargetActorNr + PhotonNetwork.player.ID * 10;
                }
                else if (PhotonNetwork.player.ID > TargetActorNr)
                {
                    temp = PhotonNetwork.player.ID + TargetActorNr * 10;
                }
                else
                {
                    return;
                }
                // this could happen if actor number reaches 25
                if (temp > byte.MaxValue)
                {
                    Debug.LogErrorFormat("Unsupprted AudioGroup value: {0}. Will not be able to talk to player number {1}", temp, player.ID);
                    Destroy(this);
                    return;
                }
                AudioGroup = (byte) temp;
                if (PhotonVoiceNetwork.ClientState == ClientState.Joined)
                {
                    Subscribed = Subscribe();
                }
            }
        }

        public void PushToTalkOn()
        {
            if (pttScript.CurrentTargetGroup != -1)
            {
                Debug.LogWarningFormat("Cannot talk to Player {0} as already talking to another player", TargetActorNr);
            }
            else if (Subscribed)
            {
                buttonText.text = string.Format("Talking-To-Player{0}", TargetActorNr);
                pttScript.PushToTalkOn(AudioGroup);
            }
            else
            {
                Debug.LogWarningFormat("Cannot talk to Player {0} as Voice client not subscribed to AudioGroup {1}", TargetActorNr, AudioGroup);
            }
        }

        /// <summary>Called per frame. Used to check key, which is used for Voice push to talk.</summary>
        public void Update()
        {
            if (Input.GetKeyDown(keyCode) || Input.GetKeyDown(keyCode2))
            {
                PushToTalkOn();
            }
            else if (Input.GetKeyUp(keyCode) || Input.GetKeyUp(keyCode2))
            {
                PushToTalkOff();
            }
        }

        public void PushToTalkOff()
        {
            buttonText.text = string.Format("Talk-To-Player{0}", TargetActorNr);
            pttScript.PushToTalkOff(AudioGroup);
        }

        public bool Subscribe()
        {
            return !Subscribed && PhotonVoiceNetwork.Client.ChangeAudioGroups(null, new byte[1] { AudioGroup });
        }
    }
}
