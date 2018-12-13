using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.ComponentModel;
using System.Reflection;

using PhysicsAPI;

[System.Serializable]
public class tntTrackingChain
{
    public tntChildLink m_endEffector;
    public tntLink m_root;
    public GameObject m_target;
    public float[] m_targetPosRot = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
}

public class tntPoseController : tntController
{
    private const float defaultSolverToleranceThreshold = 0.001f;

	public tntReducedState m_desiredPose = null;
    // When set to false, the engine will allocate memory for the desired pose and C# side won't be able to access
    // the desired pose
    // When set to true, C# will allocate memory for the desired pose and pass the buffer to the engine. C# will have full
    // access to the desired pos
    // Normally this should be left as default value of True because C# need to dynamically change the desired pose for animation
    public bool m_reflectDesiredPose;
    // When the following set to True it will only reflect the buffer of a copy of the desired pose script object
    // to the engine to protect the on-disk script object from corruption
    public bool m_protectDesiredPoseScriptObject;
    public tntTrackingChain[] m_trackingChains;

    [SerializeField, HideInInspector]
    protected float m_solverToleranceThreshold = defaultSolverToleranceThreshold;

    // ------------------PD Control Parameters----------------------
    //the root is not directly controlled by any joint, so we will store its Kp, Kd and maxTorque separated here.
    //while the pose controller does not use these values, other types of controllers may need this information
    private PDParams m_rootPdParams;
    private tntBase m_root = null;

    public tntPoseController()
    {
        m_rootPdParams = new PDParams();
        m_reflectDesiredPose = true;
        m_protectDesiredPoseScriptObject = true;
    }

    // Articulated base and child links are pushed to the physics world during Awake(). So by now
    // The articulation rooted at the parent node of this node should have already existed in the
    // physics world.
    public override bool SerializeToEngine()
    {
        if (transform.parent == null)
            return false;
        Transform rootTrans = transform.parent;
        Transform containerTrans = rootTrans.parent;
        m_root = rootTrans.GetComponent<tntBase>();

        if (m_desiredPose == null || m_desiredPose.NumOfChildLinks != m_root.numLinks())
        {
            //Debug.Log("m_desiredPose.NumOfChildLinks = " + m_desiredPose.NumOfChildLinks + ", _root.numLinks() = " + m_root.numLinks());
            tntReducedState oldDesiredPoseTemp = m_desiredPose;
            m_desiredPose = m_root.CreateDefaultReducedState();
			//Debug.Log("m_desiredPose.NumOfChildLinks = " + m_desiredPose.NumOfChildLinks);
            m_desiredPose.Remap(oldDesiredPoseTemp);
        }
        else if (m_protectDesiredPoseScriptObject)
            m_desiredPose = m_desiredPose.Copy();

		m_controller = TNT.acCreatePoseController(m_root.GetWorld(), m_root.GetBase(), m_constraintPDFix);

		if (m_controller != IntPtr.Zero && m_desiredPose != null && m_reflectDesiredPose)
		unsafe
		{
			fixed (float* pDesiredPose = &m_desiredPose.m_values[0])
			{
			    TNT.acInstallCommonControllerSharedMemoryBuffers (m_controller, null, pDesiredPose, null);
			}
		}

		if (m_trackingChains != null) {
			for (int i = 0; i < m_trackingChains.Length; ++i) {
				unsafe {
					fixed (float* pTargetPos = &m_trackingChains[i].m_targetPosRot[0]) {
						TNT.acAddPoseControlChain (
							m_controller,
							m_trackingChains [i].m_endEffector.GetIndex (),
							m_trackingChains [i].m_root.GetIndex (),
							pTargetPos);
					}
				}
			}
		}

        TNT.acInitRootPdParams(m_controller, m_rootPdParams.controlled, m_rootPdParams.kp,
            m_rootPdParams.kd, m_rootPdParams.maxAbsTorque,
            m_rootPdParams.scale.x,
            m_rootPdParams.scale.y,
            m_rootPdParams.scale.z,
            m_rootPdParams.relToCharFrame);

        for (int i = 0; i < m_root.numLinks(); ++i)
        {
            tntChildLink childLink = m_root.getChildLink(i);

            TNT.acAddLinkPdParams(m_controller, childLink.m_kp != 0 || childLink.m_kd != 0,
                childLink.m_kp, childLink.m_kd, childLink.m_maxPDTorque,
                0, 0, 0, false);

            if (childLink.m_useSoftConstraint)
                TNT.acUseConstraintBasedPDControl(m_controller, childLink.GetIndex());
        }

        TNT.acSetPoseControllerToleranceThreshold(m_controller, m_solverToleranceThreshold);

        return true;
    }

    void Update()
    {
		if (m_trackingChains == null)
			return;
        for (int i = 0; i < m_trackingChains.Length; ++i)
        {
            Vector3 targetPos = m_trackingChains[i].m_target.transform.position;
            Quaternion targetRot = m_trackingChains[i].m_target.transform.rotation;
            m_trackingChains[i].m_targetPosRot[0] = targetPos.x;
            m_trackingChains[i].m_targetPosRot[1] = targetPos.y;
            m_trackingChains[i].m_targetPosRot[2] = targetPos.z;
            m_trackingChains[i].m_targetPosRot[3] = targetRot.x;
            m_trackingChains[i].m_targetPosRot[4] = targetRot.y;
            m_trackingChains[i].m_targetPosRot[5] = targetRot.z;
            m_trackingChains[i].m_targetPosRot[6] = targetRot.w;
        }
    }

    public float SolverToleranceThreshold
    {
        get { return m_solverToleranceThreshold; }
        set
        {
            if (value != m_solverToleranceThreshold)
            {
                m_solverToleranceThreshold = value;
                if (m_controller != IntPtr.Zero)
                {
                    TNT.acSetPoseControllerToleranceThreshold(m_controller, m_solverToleranceThreshold);
                }
            }
        }
    }
}

