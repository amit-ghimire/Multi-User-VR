using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class CreateRoom : MonoBehaviour {
    [SerializeField]
    private Text _roomName;
    private Text RoomName
    {
        get { return _roomName; }
    }

    public void OnClick_CreateRoom()
    {
        
        RoomOptions roomOptions = new RoomOptions() { IsVisible = true, IsOpen = true, MaxPlayers = 20 };

        if (PhotonNetwork.CreateRoom(RoomName.text,roomOptions,TypedLobby.Default))
        {
            print("create room succesfully sent");
        }
        else
        {
            print("create room failed to send");
        }
    }

    private void OnPhotonCreateRoomFailed(object[] codeAndMessage)
    {
        print("Create room failed: " + codeAndMessage[1]);
    }

    private void OnCreatedRoom()
    {
        print("Room Created successfully");
    }
	
}
