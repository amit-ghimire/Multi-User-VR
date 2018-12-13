using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NameTag : MonoBehaviour {

    private PhotonView pView;

	// Use this for initialization
	void Start () {
        pView = GetComponent<PhotonView>();
        GetComponent<TextMesh>().text = pView.owner.NickName;
	}
}
