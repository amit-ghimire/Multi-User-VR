using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SetName : MonoBehaviour {

    PhotonView photonView;
    public TextMesh tMesh;
	
    // Use this for initialization
	void Start () {
        photonView = GetComponent<PhotonView>();
        tMesh.text = PhotonNetwork.playerName;
	}

}
