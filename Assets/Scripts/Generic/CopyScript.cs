using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CopyScript : Photon.MonoBehaviour {

    public int index = 1;
    	
	// Update is called once per frame
	void Update () {
        if (photonView.isMine)
        {
            switch (index)
            {
                case 1://head
                    transform.position = HMDManager.Instance.head.transform.position;
                    transform.rotation = HMDManager.Instance.head.transform.rotation;
                    break;
                case 2://left
                    transform.position = HMDManager.Instance.leftHand.transform.position;
                    transform.rotation = HMDManager.Instance.leftHand.transform.rotation;
                    transform.localScale = HMDManager.Instance.leftHand.transform.localScale;
                    break;
                case 3://right
                    transform.position = HMDManager.Instance.rightHand.transform.position;
                    transform.rotation = HMDManager.Instance.rightHand.transform.rotation;
                    transform.localScale = HMDManager.Instance.rightHand.transform.localScale;
                    break;
            }
           
        }
	}
}
