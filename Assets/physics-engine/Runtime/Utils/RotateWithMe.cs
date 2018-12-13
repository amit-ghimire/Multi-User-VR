using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RotateWithMe : MonoBehaviour {
    [SerializeField] GameObject satellite1 = null;
    [SerializeField] GameObject satellite2 = null;
    [SerializeField] bool turnTogther = false;

    private Quaternion centerRot, startRot1, startRot2;
    private Vector3 centerPos, startPos1, startPos2;

    private bool lastRotate = false;
	// Use this for initialization
	void Start () {
	}
	
	// Update is called once per frame
	void Update () {
		if (turnTogther != lastRotate)
        {
            if (turnTogther)
            {
                centerRot = transform.rotation;
                centerPos = transform.position;
                if (satellite1 != null)
                {
                    startPos1 = satellite1.transform.position;
                    startRot1 = satellite1.transform.rotation;
                }
                if (satellite2 != null)
                {
                    startPos2 = satellite2.transform.position;
                    startRot2 = satellite2.transform.rotation;
                }
            }
            lastRotate = turnTogther;
        }
        if (turnTogther)
        {
            Quaternion rotDelta = Quaternion.Inverse(centerRot) * transform.rotation;
            float rotAngle = rotDelta.eulerAngles.y;

            if (satellite1 != null)
            {
                satellite1.transform.position = startPos1 + transform.position - centerPos;
                satellite1.transform.rotation = startRot1;
                satellite1.transform.RotateAround(transform.position, Vector3.up, rotAngle);
            }
            if (satellite2 != null)
            {
                satellite2.transform.position = startPos2 + transform.position - centerPos; ;
                satellite2.transform.rotation = startRot2;
                satellite2.transform.RotateAround(transform.position, Vector3.up, rotAngle);
            }
        }
	}
}
