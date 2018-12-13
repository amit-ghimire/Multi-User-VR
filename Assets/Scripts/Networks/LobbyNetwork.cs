using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

public class LobbyNetwork : MonoBehaviour {

    #region private variables
    private string GameVersion = "v1.0.0";
    #endregion

    #region MonoCallbacks
    void Start () {
        //XRSettings.enabled = false;
        print("Connecting to server");
        PhotonNetwork.ConnectUsingSettings(GameVersion);
	}
    #endregion

    #region Photon Callbacks
    private void OnConnectedToMaster()
    {
        print("Connected to master");
        PhotonNetwork.automaticallySyncScene = true;
        PhotonNetwork.playerName = PlayerNetwork.Instance.PlayerName;
        PhotonNetwork.JoinLobby(TypedLobby.Default);
    }

    private void OnJoinedLobby()
    {
        print("Joined Lobby");
        
        if(!PhotonNetwork.inRoom)
            MainCanvasManager.Instance.LobbyCanvas.transform.SetAsLastSibling();
    }
    #endregion
}
