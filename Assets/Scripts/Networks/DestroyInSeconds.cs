using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DestroyInSeconds : MonoBehaviour {

	// Use this for initialization
	void Start () {
        StartCoroutine("Destroy");
    }

    IEnumerator Destroy()
    {
        yield return new WaitForSeconds(10f);
        PhotonNetwork.Destroy(this.gameObject);
    }
}
