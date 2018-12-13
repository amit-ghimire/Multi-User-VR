using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ChangeOwners : MonoBehaviour {

    private VRTK.VRTK_InteractableObject toss;
    private PhotonView photonView;

	// Use this for initialization
	void Start () {

        photonView = GetComponent<PhotonView>();

        toss = GetComponent<VRTK.VRTK_InteractableObject>();
        toss.InteractableObjectGrabbed += Toss_InteractableObjectGrabbed;
	}

    private void Toss_InteractableObjectGrabbed(object sender, VRTK.InteractableObjectEventArgs e)
    {
        toss.GetComponent<PhotonView>().TransferOwnership(PhotonNetwork.player.ID);
    }
}
