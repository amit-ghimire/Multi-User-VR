// ----------------------------------------------------------------------------
// <copyright file="PushToTalkScript.cs" company="Exit Games GmbH">
// Photon Voice Demo for PUN- Copyright (C) 2016 Exit Games GmbH
// </copyright>
// <summary>
// Script that implements Push-To-Talk feature of Photon Voice for PUN
// </summary>
// <author>developer@photonengine.com</author>
// ----------------------------------------------------------------------------

#pragma warning disable 0649 // Field is never assigned to, and will always have its default value

using System.Collections.Generic;

namespace ExitGames.Demos.DemoPunVoice
{
    using Client.Photon.LoadBalancing;
    using UnityEngine;
    using UnityEngine.UI;

    public class PushToTalkScript : MonoBehaviour
    {
        [SerializeField]
        private Button pushToTalkButton;

        /// <summary>The button's text, so we can change it.</summary>
        private Text pushToTalkText;

        /// <summary>This client's voice recorder.</summary>
        private PhotonVoiceRecorder rec;

        private Transform parent;
        [SerializeField]
        private GameObject buttonPrefab;

        private Dictionary<int, PushToTalkPrivateButton> buttons;

        public bool MuteOthersWhileTalking;

        /// <summary>
        /// Current AudioGroup that we transmit voice to if any.
        /// </summary>
        public int CurrentTargetGroup = -1;

        public void OnEnable()
        {
            CharacterInstantiation.CharacterInstantiated += CharacterInstantiation_CharacterInstantiated;
        }

        public void OnDisable()
        {
            CharacterInstantiation.CharacterInstantiated -= CharacterInstantiation_CharacterInstantiated;
        }
        
        /// <summary>Initializes the component.</summary>
        public void Start()
        {
            if (pushToTalkButton != null)
            {
                pushToTalkText = pushToTalkButton.GetComponentInChildren<Text>();
                if (pushToTalkText != null)
                {
                    pushToTalkText.text = "Talk-To-All";
                }
                else
                {
                    Debug.LogWarning("Could not find the button's text component.");
                }
                pushToTalkButton.gameObject.SetActive(false);   // if it's inactive, GetComponentInChildren won't find anything.
                parent = pushToTalkButton.transform.parent;
            }
            buttons = new Dictionary<int, PushToTalkPrivateButton>(4);
            PhotonVoiceNetwork.Client.OnStateChangeAction += OnVoiceClientStateChanged;
        }

        public void OnToggleValueChanged(bool v)
        {
            MuteOthersWhileTalking = v;
            if (MuteOthersWhileTalking && CurrentTargetGroup != -1)
            {
                KeepOnlyOneGroup((byte)CurrentTargetGroup);
            }
            else if (!MuteOthersWhileTalking)
            {
                if (CurrentTargetGroup != -1 && rec != null)
                {
                    rec.AudioGroup = (byte)CurrentTargetGroup;
                }
                SubscribeToAllPrivateGroups();
            }
        }


        /// <summary>
        /// A callback for CharacterInstantiation. We need the PhotonVoiceRecorder from our new character.
        /// </summary>
        /// <param name="character">This client's character.</param>
        private void CharacterInstantiation_CharacterInstantiated(GameObject character)
        {
            rec = character.GetComponent<PhotonVoiceRecorder>();
            if (rec != null)
            {
                rec.enabled = true;
            }
        }

        /// <summary>Callback by the Voice Chat Client.</summary>
        /// <remarks>
        /// Unlike callbacks from PUN, this only updates you on the state of Voice.
        /// Voice will by default automatically enter a Voice room, when PUN joined one. That's why Joined state will happen.
        /// </remarks>
        /// <param name="state">The new state.</param>
        private void OnVoiceClientStateChanged(ClientState state)
        {
            if (pushToTalkButton != null)
            {
                switch (state)
                {
                    case ClientState.Joined:
                        pushToTalkButton.gameObject.SetActive(true);
                        for (int i = 0; i < PhotonNetwork.otherPlayers.Length; i++)
                        {
                            OnPhotonPlayerConnected(PhotonNetwork.otherPlayers[i]);
                        }
                        break;
                    default:
                        pushToTalkButton.gameObject.SetActive(false);
                        break;
                }
            }
        }

        /// <summary>Activates Push-To-Talk feature for given group</summary>
        public void PushToTalkOn(int group)
        {
            if (CurrentTargetGroup != -1)
            {
                Debug.LogWarningFormat("Cannot start AudioGroup {0}, player already talking to AudioGroup {1}", group, CurrentTargetGroup);
                return;
            }
            if (MuteOthersWhileTalking)
            {
                KeepOnlyOneGroup((byte)group);
            }
            else if (rec != null)
            {
                SubscribeToAllPrivateGroups();
                rec.AudioGroup = (byte)group;
            }
            PushToTalk(true);
            CurrentTargetGroup = group;
            if (group == 0 && pushToTalkText != null)
            {
                pushToTalkText.text = "Talk now!";
            }
        }

        /// <summary>Deactivates Push-To-Talk feature for given group</summary>
        public void PushToTalkOff(int group)
        {
            if (CurrentTargetGroup == group)
            {
                PushToTalk(false);
                SubscribeToAllPrivateGroups();
            }
        }

        /// <summary>Deactivates Push-To-Talk feature for global audio group: 0</summary>
        public void PushToTalkOff()
        {
            PushToTalkOff(0);
            if (pushToTalkText != null) { pushToTalkText.text = "Talk-To-All"; }
        }

        /// <summary>Activates Push-To-Talk feature for global audio group: 0</summary>
        public void PushToTalkOn()
        {
            PushToTalkOn(0);
        }

        /// <summary>
        /// Toggles Push-To-Talk feature
        /// </summary>
        /// <param name="toggle">value to set PTT on/off</param>
        public void PushToTalk(bool toggle)
        {
            CurrentTargetGroup = -1;
            if (rec != null)
            {
                rec.Transmit = toggle;
            }
        }

        /// <summary>This is a callback by PUN. Makes sure recording is off when leaving the room.</summary>
        /// <remarks>Unlike callbacks from the Voice Client, this only updates you on the state of PUN. Use to control voice according to changes in PUN.</remarks>
        public void OnLeftRoom()
        {
            if (rec != null)
            {
                rec.enabled = false;
                rec = null;
            }
        }

        /// <summary>Called per frame. Used to check key "v", which is used for Voice push to talk.</summary>
        public void Update()
        {
            if (Input.GetKeyDown("v"))
            {
                PushToTalkOn();
            }
            else if (Input.GetKeyUp("v"))
            {
                PushToTalkOff();
            }
        }

        public void OnPhotonPlayerConnected(PhotonPlayer newPlayer)
        {
            if (buttons.ContainsKey(newPlayer.ID))
            {
                Debug.LogWarningFormat(buttons[newPlayer.ID].gameObject, "PTT Button already added for player number {0}", newPlayer.ID);
                return;
            }
            GameObject button = Instantiate(buttonPrefab);
            button.transform.SetParent(parent, false);
            button.transform.SetSiblingIndex(newPlayer.ID + 1);
            PushToTalkPrivateButton script = button.GetComponentInChildren<PushToTalkPrivateButton>();
            script.SetAudioGroup(newPlayer);
            buttons.Add(newPlayer.ID, script);
        }

        public void OnPhotonPlayerDisconnected(PhotonPlayer leavingPlayer)
        {
            PhotonVoiceNetwork.Client.ChangeAudioGroups(new byte[1] {buttons[leavingPlayer.ID].AudioGroup}, null);
            if (buttons.ContainsKey(leavingPlayer.ID))
            {
                Destroy(buttons[leavingPlayer.ID].gameObject);
                buttons.Remove(leavingPlayer.ID);
            }
            else
            {
                Debug.LogWarningFormat("PTT Button not found for leaving player number {0}", leavingPlayer.ID);
            }
        }

        private void SubscribeToAllPrivateGroups()
        {
            PhotonVoiceNetwork.Client.ChangeAudioGroups(null, new byte[0]);
            for (int i = 0; i < PhotonNetwork.otherPlayers.Length; i++)
            {
                // mark other AudioGroups as subscribed
                buttons[PhotonNetwork.otherPlayers[i].ID].Subscribed = true;
            }
        }

        private void KeepOnlyOneGroup(byte group)
        {
            for (int i = 0; i < PhotonNetwork.otherPlayers.Length; i++)
            {
                // mark other AudioGroups as unsubscribed
                if (buttons[PhotonNetwork.otherPlayers[i].ID].AudioGroup != group)
                {
                    buttons[PhotonNetwork.otherPlayers[i].ID].Subscribed = false;
                }
                else
                {
                    PhotonVoiceNetwork.Client.GlobalAudioGroup = group;
                    buttons[PhotonNetwork.otherPlayers[i].ID].Subscribed = true;
                }
            }
        }
    }
}
