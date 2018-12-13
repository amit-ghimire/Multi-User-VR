using System.Collections.Generic;
using UnityEngine;

public class RoomLayoutGroup : MonoBehaviour {
    [SerializeField]
    private GameObject _roomListingPrefab;
    private GameObject RoomListingPrefab
    {
        get { return _roomListingPrefab; }
    }

    private List<RoomListing> _roomListinButtons = new List<RoomListing>();
    private List<RoomListing> RoomListingButtons
    {
        get { return _roomListinButtons; }
    }


    private void OnReceivedRoomListUpdate()
    {
        Debug.Log("new room updated");
        RoomInfo[] rooms = PhotonNetwork.GetRoomList();
        foreach (RoomInfo room in rooms)
        {
            RoomReceived(room);
        }

        RemoveOldRooms();
    }


    private void RoomReceived(RoomInfo room)
    {
        int index = RoomListingButtons.FindIndex(x => x.RoomName == room.Name);
        if (index == -1)
        {
            if (room.IsVisible && room.PlayerCount < room.MaxPlayers)
            {
                GameObject roomListingObj = Instantiate(RoomListingPrefab);
                roomListingObj.transform.SetParent(transform, false);

                RoomListing roomListing = roomListingObj.GetComponent<RoomListing>();
                RoomListingButtons.Add(roomListing);

                index = (RoomListingButtons.Count - 1);
            }
        }

        if (index != -1)
        {
            RoomListing roomListing = RoomListingButtons[index];
            roomListing.SetRoomNameText(room.Name);
            roomListing.Updated = true;
        }
    }


    private void RemoveOldRooms()
    {
        List<RoomListing> removeRooms = new List<RoomListing>();

        foreach (RoomListing rl in RoomListingButtons)
        {
            if (!rl.Updated)
            {
                removeRooms.Add(rl);
            }
            else
            {
                rl.Updated = false;
            }
        }

        foreach (RoomListing rl in removeRooms)
        {
            GameObject roomListingObj = rl.gameObject;
            RoomListingButtons.Remove(rl);
            Destroy(roomListingObj);
        }
    }
}
