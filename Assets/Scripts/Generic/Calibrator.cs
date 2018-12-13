using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Calibrator : MonoBehaviour {

    public float dummyScale;
    public float multiplier;
    public GameObject eye;

    PhotonView photonView;

    private void Awake()
    {
        ChangeScale();
    }
    // Use this for initialization
    private void Start()
    {
        photonView = GetComponent<PhotonView>();
        ChangeScale();
     
    }

    void Update () {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            if (!photonView.isMine)
            {
                return;
            }
            else
            {
                ChangeScale();
            }
            
        }
	}

    void ChangeScale()
    {
        multiplier = 0.625f;
        eye = GameObject.Find("Camera (eye)");
        dummyScale = eye.transform.localPosition.y;
        transform.localScale = new Vector3(multiplier*dummyScale, multiplier*dummyScale, multiplier*dummyScale);
    }

}
