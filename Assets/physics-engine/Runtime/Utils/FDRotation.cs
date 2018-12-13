using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FDRotation : MonoBehaviour {
	public GameObject m_target;
	private tntPoseController m_poseController;
	private tntBallLink m_ballLink;
	// Use this for initialization
	void Start () {
		m_poseController = transform.parent.GetComponentInChildren<tntPoseController>();
		m_ballLink = GetComponentInParent<tntBallLink>();
	}
	
	// Update is called once per frame
	void Update ()
	{
		Quaternion localRotation = m_target.transform.localRotation;
		m_poseController.m_desiredPose.SetJointOrientationToQuaternion(m_ballLink.GetIndex(), localRotation);
	}
}
