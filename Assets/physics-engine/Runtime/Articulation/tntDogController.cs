using UnityEngine;
//using UnityEditor;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.ComponentModel;
using System.Reflection;
using System.Runtime.InteropServices;

using PhysicsAPI;

[System.Serializable]
// Assumptions:
// 1) stance is the first field
// 2) the structure layout matches the ACE internal structure
public class tntDogControllerState
{
    public float stridePhase;
    public float frontLeftStancePhase;
    public float frontLeftSwingPhase;
    public float frontRightStancePhase;
    public float frontRightSwingPhase;
    public float rearLeftStancePhase;
    public float rearLeftSwingPhase;
    public float rearRightStancePhase;
    public float rearRightSwingPhase;
    public bool grounded;
    public bool standStill;	   //Whether the character should stand still and suppress leg swing
    public bool dead;          //Whether the character is dead as a ragdoll  
};

public class tntDogController : tntController
{
    // Public Properties 
    public tntReducedState m_currentPose;
	// Normally the following flag should be left as default value: False
	// When set to true the engine will reflect the current pose of the monster back to
	// the ScriptObject slotted here on a frame to frame basis. This can be useful for
	// debugging if you want to keep a snapshot of the last monster pose before crash or
	// something and the snapshot is stored into the ScriptObject when the game is running
	// in Unity editor.
    public bool m_reflectCurrentPose;   
    public bool m_keepRootPosition;     // Keep the base position/orientation when setting initial pose to m_reducedState
    public tntReducedState m_desiredPose;
	// When set to false, the engine will allocate memory for the desired pose and C# side won't be able to access
	// the desired pose
	// When set to true, C# will allocate memory for the desired pose and pass the buffer to the engine. C# will have full
	// access to the desired pos
	// Normally this should be left as default value of True because C# need to dynamically change the desired pose for animation
	public bool m_reflectDesiredPose;
	// When the following set to True it will only reflect the buffer of a copy of the desired pose script object
	// to the engine to protect the on-disk script object from corruption
	public bool m_protectDesiredPoseScriptObject;
	// When the following set to True it will only reflect the buffer of a copy of the control parameter script object
	// to the engine to protect the on-disk script object from corruption
	public bool m_protectControlParamsScriptObject;
    public bool m_reflectControllerState;
    // When set to true, use blend space interpolation to automatically set control parameter values
    public bool m_useBlendSpace;
    // When set to true, a GRF solver will be used to optimize the ground reaction forces for balance control
    public bool m_disableGRFSolver;
    // When the following set to True it will only reflect the buffer of a copy of the PD parameter script object
    // to the engine to protect the on-disk script object from corruption
    public bool m_protectPDParamSetScriptObject;
    // This is the minimum magnitude by which blend space parameters much change change to incur an updated blend
    public float m_blendGranularity = 0.1f;

    // ------------------PD Control Parameters----------------------
	//the root is not directly controlled by any joint, so we will store its Kp, Kd and maxTorque separated here.
	//while the pose controller does not use these values, other types of controllers may need this information
	public PDParams m_rootPdParams;
	// The following two fields are obsolete
	public List<PDParams> m_pdParams;
	private	int[] m_pdParamsIndexMap;  // map joint index to the pdParams index
	
	//-------------------Dynamic Control Parameters----------------- 
	public PDParamSet m_pdParamSet;
    public tntDogControlParams m_controlParams;
    public tntDogBlendSample[] m_controlParamsBlendSamples;
    public tntDogControllerState controllerState;

	public Vector3 m_forward = new Vector3(-1, 0, 0);
	public Vector3 m_right = new Vector3(0, 0, 1);
	private tntBase m_root = null;

    public tntDogController()
    {
		m_keepRootPosition = true;
        m_reflectCurrentPose = false;
        m_reflectDesiredPose = true;
		m_protectDesiredPoseScriptObject = true;
		m_protectControlParamsScriptObject = true;
        m_protectPDParamSetScriptObject = true;
        m_reflectControllerState = true;

		//m_pdParams = new List<ControlParams>();
		m_currentPose = null;
        m_desiredPose = null;
        m_controlParams = null;
        m_controller = IntPtr.Zero;
        m_worldIndex = -1;
		controllerState = new tntDogControllerState();
	}

	// Articulated base and child links are pushed to the physics world during Awake(). So by now
    // The articulation rooted at the parent node of this node should have already existed in the
    // physics world.
    public override bool SerializeToEngine()
    {
        if (!transform.parent)
            return false;

        Transform rootTrans = transform.parent;
        Transform containerTrans = rootTrans.parent;
        m_root = rootTrans.GetComponent<tntBase>();

        if (!m_root || !containerTrans)
            return false;

        if (m_currentPose == null || m_currentPose.NumOfChildLinks != m_root.numLinks())
        {
#if true
            m_currentPose = m_root.CreateCurrentReducedState(m_currentPose);
#else
            tntReducedState oldCurrentPoseTemp = m_currentPose;
            m_currentPose = m_root.CreateDefaultReducedState();
            m_currentPose.Remap(oldCurrentPoseTemp);
#endif
        }

        if (m_desiredPose == null || m_desiredPose.NumOfChildLinks != m_root.numLinks())
        {
            tntReducedState oldDesiredPoseTemp = m_desiredPose;
            m_desiredPose = m_root.CreateDefaultReducedState();
            m_desiredPose.Remap(oldDesiredPoseTemp);
        }
        else if (m_protectDesiredPoseScriptObject)
            m_desiredPose = m_desiredPose.Copy();
        
        if (m_protectControlParamsScriptObject)
			m_controlParams = m_controlParams.Copy();

        if (m_limbs.m_upperBack == null)
        {
            Transform torsoTrans = containerTrans.Find("backJ5");
            if (torsoTrans != null)
                m_limbs.m_upperBack = torsoTrans.GetComponent<tntLink>();
        }
        if (m_limbs.m_lHand == null)
        {
            Transform lHandTrans = containerTrans.Find("lFingerJoint"); // conservative lookup based on T-Rex
            if (lHandTrans != null)
                m_limbs.m_lHand = lHandTrans.GetComponent<tntLink>();
        }
        if (m_limbs.m_rHand == null)
        {
            Transform rHandTrans = containerTrans.Find("rFingerJoint"); // conservative lookup based on T-Rex
            if (rHandTrans != null)
                m_limbs.m_rHand = rHandTrans.GetComponent<tntLink>();
        }
        if (m_limbs.m_lToes == null)
        {
            Transform lToesTrans = containerTrans.Find("lToeJoint");
			if (lToesTrans != null)
                m_limbs.m_lToes = lToesTrans.GetComponent<tntLink>();
        }
		if (m_limbs.m_rToes == null)
		{
			Transform rToesTrans = containerTrans.Find("rToeJoint");
			if (rToesTrans != null)
                m_limbs.m_rToes = rToesTrans.GetComponent<tntLink>();
		}

        if (m_limbs.m_neck == null)
        {
            Transform headTrans = containerTrans.Find("neckJ4");
			if (headTrans != null)
                m_limbs.m_neck = headTrans.GetComponent<tntLink>();
		} 

        if (m_limbs.m_tail == null)
        {
            Transform tail1Trans = containerTrans.Find("tailJ4");   // conservative lookup based on T-Rex
			if (tail1Trans != null)
                m_limbs.m_tail = tail1Trans.GetComponent<tntLink>();
        }

        if (m_limbs.m_upperBack != null && !m_root.ContainsLink(m_limbs.m_upperBack, false))
            m_limbs.m_upperBack = null;

        if (m_limbs.m_lHand != null && !m_root.ContainsLink(m_limbs.m_lHand, false))
            m_limbs.m_lHand = null;

        if (m_limbs.m_rHand != null && !m_root.ContainsLink(m_limbs.m_rHand, false))
            m_limbs.m_rHand = null;

        if (m_limbs.m_lToes != null && !m_root.ContainsLink(m_limbs.m_lToes, false))
            m_limbs.m_lToes = null;

        if (m_limbs.m_rToes != null && !m_root.ContainsLink(m_limbs.m_rToes, false))
            m_limbs.m_rToes = null;

        if (m_limbs.m_neck != null && !m_root.ContainsLink(m_limbs.m_neck, false))
            m_limbs.m_neck = null;

        if (m_limbs.m_tail != null && !m_root.ContainsLink(m_limbs.m_tail, false))
            m_limbs.m_tail = null;


        if (m_currentPose == null || m_currentPose.m_values.Length < m_root.numLinks())
        {
            Debug.LogError("Please assign the initial current pose to the controller!");
            return false;
        }

        if (m_desiredPose == null || m_desiredPose.m_values.Length < m_root.numLinks())
        {
            Debug.LogError("Please assign the initial desired pose to the controller!");
            return false;
        }

        if (m_controlParams == null)
        {
                Debug.LogError("Please assign control parameters to the controller!");
                return false;
        }

        if (m_pdParamSet == null)
        {
            PDParamSet paramSet = ScriptableObject.CreateInstance<PDParamSet>();
            paramSet.FillPDParamSet(transform.parent.parent.gameObject);
            m_pdParamSet = paramSet;
        }
        else if (m_protectPDParamSetScriptObject)
        {
            m_pdParamSet = m_pdParamSet.Copy();
        }

        if (m_useBlendSpace) {
            if (m_controlParamsBlendSamples == null || m_controlParamsBlendSamples.Length < 1) {
                Debug.LogError("Blendspace samples do not exist. Blendspace has been automatically disabled.");
                m_useBlendSpace = false;
            }
            for (int i = 0; i < m_controlParamsBlendSamples.Length; i++) {
                if (m_controlParamsBlendSamples[i] == null ||
                    m_controlParamsBlendSamples[i].m_paramsScriptObject == null) {
                    Debug.LogError("Blendspace samples do not exist. Blendspace has been automatically disabled.");
                    m_useBlendSpace = false;
                }
            }
        }

		unsafe
		{
			fixed(float* pControlParams = &m_controlParams.m_params[0])
			fixed(float* pCurrentPose = &m_currentPose.m_values[0])
            fixed(float* pDesiredPose = &m_desiredPose.m_values[0])
			fixed(void* pState = &controllerState.stridePhase)
			fixed(Vector3* pForward = &m_forward)
			fixed(Vector3* pRight = &m_right)
			{
                m_controller = TNT.acCreateDogController(m_root.GetWorld(), m_root.GetBase(),
                    m_limbs.m_upperBack ? m_limbs.m_upperBack.GetIndex() : -1,
                    m_limbs.m_lHand ? m_limbs.m_lHand.GetIndex() : -1,
                    m_limbs.m_rHand ? m_limbs.m_rHand.GetIndex() : -1,
                    m_limbs.m_lToes ? m_limbs.m_lToes.GetIndex() : -1,
                    m_limbs.m_rToes ? m_limbs.m_rToes.GetIndex() : -1,
                    m_limbs.m_neck ? m_limbs.m_neck.GetIndex() : -1,
                    m_limbs.m_tail ? m_limbs.m_tail.GetIndex() : -1,
                    m_limbs.m_limbTrackingKp,
                    m_limbs.m_limbTrackingKd,
                    m_limbs.m_limbMaxTrackingForce,
                    pControlParams, m_controlParams.m_params.Length,
                    pCurrentPose,
                    pDesiredPose,
                    m_blendGranularity,
                    m_keepRootPosition,
                    m_useBlendSpace,
                    !m_disableGRFSolver,
					m_constraintPDFix,
                    pForward,
				    pRight);

                if (m_controller != IntPtr.Zero)
                {
                    TNT.acInstallCommonControllerSharedMemoryBuffers(m_controller,
                        m_reflectCurrentPose ? pCurrentPose : null,
                        m_reflectDesiredPose ? pDesiredPose : null,
                        pControlParams);

                    TNT.acInstallDogControllerSharedMemoryBuffers(m_controller,
                        m_reflectControllerState ? pState : null);
                }
            }
		}

        if (m_useBlendSpace) {
            for (int i = 0; i < m_controlParamsBlendSamples.Length; i++) {
                if (m_controlParamsBlendSamples[i] == null) {
                    Debug.LogError("Blendspace samples do not exist.");
                    return false;
                }
                if (m_protectControlParamsScriptObject) {
                    m_controlParamsBlendSamples[i] = m_controlParamsBlendSamples[i].Copy();
                }
                unsafe
                {
                    fixed (float* pControlParams = &m_controlParamsBlendSamples[i].m_paramsScriptObject.m_params[0]) {
                        TNT.acAddHumanoidBlendSample(
                            m_controller,
                            pControlParams,
                            m_controlParamsBlendSamples[i].m_fwdVel,
                            m_controlParamsBlendSamples[i].m_sideVel,
                            m_controlParamsBlendSamples[i].m_turnVel);
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

		if (m_pdParams.Count > 0)
		{
			// Obsolete way to initialize PD parameters. Should migrate into each tntXXXLink
	 		m_pdParamsIndexMap = new int[m_root.numLinks()];
			// Walk through articulation links in order and add ControlParams to each of them
	        for (int i = 0; i < m_root.numLinks(); ++i)
				m_pdParamsIndexMap[i] = -1;
	        for (int i = 0; i < m_pdParams.Count; ++i)
	        {
	            int jointIndex = m_root.NameToIndex(m_pdParams[i].name);
                if (jointIndex == -2)
                    continue;
	            m_pdParamsIndexMap[jointIndex] = i;
	            Debug.Log(m_pdParams[i].name + " => " + m_root.NameToIndex(m_pdParams[i].name));
	        }
			for (int i = 0; i < m_root.numLinks(); ++i)
	        {
                PDParams param;
				if (m_pdParamsIndexMap[i] >= 0)
	            {
					param = m_pdParams[m_pdParamsIndexMap[i]];

	            } else
	            {
	                // Add a dumyy uncontrolled PDParam entry
	                param = new PDParams();
	                param.controlled = false;
	            }
	            TNT.acAddLinkPdParams(m_controller, param.controlled, param.kp,
	                                  param.kd, param.maxAbsTorque,
	                                  param.scale.x, param.scale.y, param.scale.z,
	                                  param.relToCharFrame);
	        }
		} else
		{
			// New way to store Kp/Kd inside tntXXXLinks
			for (int i = 0; i < m_root.numLinks(); ++i)
			{
				tntChildLink childLink = m_root.getChildLink(i);

				TNT.acAddLinkPdParams(m_controller, childLink.m_kp != 0 || childLink.m_kd != 0, 
					childLink.m_kp, childLink.m_kd, childLink.m_maxPDTorque,
					0, 0, 0, false);

                if (childLink.m_useSoftConstraint)
                    TNT.acUseConstraintBasedPDControl(m_controller, childLink.GetIndex());
			}
		}

        return true;
    }

	public void UpdateVelocity(float forwardVel, float sideVel, float verticalVel)
	{
		// Also update the virtual force velocity target
        if (m_controller != IntPtr.Zero)
		    TNT.acSetDesiredVelocities(m_controller, forwardVel, sideVel, 0);
	}
	
	public void UpdateHeading(float heading)
	{
		// TBD: check against cached heading and only pInvoke if changed
        if (m_controller != IntPtr.Zero)
		    TNT.acSetDesiredHeading(m_controller, heading);
	}

	public void UpdatePdParams(int jointIndex, float kp, float kd, float maxTorque)
	{
        if (jointIndex < 0 || jointIndex >= m_root.numLinks())
            return;
		if (m_pdParams.Count > 0) {
            PDParams param = m_pdParams [m_pdParamsIndexMap [jointIndex]];
			if (param.kp == kp && param.kd == kd && param.maxAbsTorque == maxTorque)
				return;
			param.kp = kp;
			param.kd = kd;
			param.maxAbsTorque = maxTorque;
			TNT.acUpdateLinkPdParams (m_controller, m_pdParamsIndexMap [jointIndex], kp, kd, maxTorque);
		} else
		{
			tntChildLink childLink = m_root.getChildLink(jointIndex);
			if (childLink.m_kp == kp && childLink.m_kd == kd && childLink.m_maxPDTorque == maxTorque)
				return;
			childLink.m_kp = kp;
			childLink.m_kd = kd;
			childLink.m_maxPDTorque = maxTorque;
        	m_pdParamSet.UpdatePdParams(jointIndex, kp, kd, maxTorque);
		}
	}
}
