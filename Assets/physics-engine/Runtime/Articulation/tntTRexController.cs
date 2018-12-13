using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.ComponentModel;
using System.Reflection;

using PhysicsAPI;

[System.Serializable]
// Assumptions:
// 1) stance is the first field
// 2) the structure layout matches the ACE internal structure
public class tntTRexControllerState
{			
	public float stridePhase;
	public float leftStancePhase;
	public float leftSwingPhase;
	public float rightStancePhase;
	public float rightSwingPhase;
	public bool grounded;
    public bool standStill;	   //Whether the character should stand still and suppress leg swing
    public bool dead;          //Whether the character is dead as a ragdoll  
};

public class tntTRexController : tntController
{
    // Public Properties 
    public tntReducedState m_currentPose;
    public bool m_reflectCurrentPose;   // dynamically update the currentPose ScriptableObject with engine's current pose
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

    // ------------------PD Control Parameters----------------------
	//the root is not directly controlled by any joint, so we will store its Kp, Kd and maxTorque separated here.
	//while the pose controller does not use these values, other types of controllers may need this information
	public PDParams m_rootPdParams;
	public List<PDParams> m_pdParams;
	private	int[] m_pdParamsIndexMap;  // map joint index to the pdParams index
	
	//-------------------Dynamic Control Parameters----------------- 
    public tntTRexControlParams m_controlParams;

    public tntTRexControllerState controllerState;

	//-------------------Debug Visualization ------------------------
	public GameObject m_leftFootTarget;
	public GameObject m_rightFootTarget;

	private Vector3 m_leftEEPos;
	private Vector3 m_rightEEPos;

    public tntTRexController()
    {
		m_keepRootPosition = true;
        m_reflectCurrentPose = false;
        m_reflectDesiredPose = true;
		m_protectDesiredPoseScriptObject = true;
		m_protectControlParamsScriptObject = true;
        m_reflectControllerState = true;

		m_pdParams = new List<PDParams>();
		m_currentPose = null;
        m_desiredPose = null;
        m_controlParams = null;
        m_controller = IntPtr.Zero;
        m_worldIndex = -1;
		controllerState = new tntTRexControllerState();
		m_leftFootTarget = m_rightFootTarget = null;
		m_pdParamsIndexMap = null;
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
        tntBase root = rootTrans.GetComponent<tntBase>();

        if (!containerTrans || !root)
            return false;

        if (m_currentPose == null || m_currentPose.NumOfChildLinks != root.numLinks())
        {
#if true
            m_currentPose = root.CreateCurrentReducedState(m_currentPose);
#else
            tntReducedState oldCurrentPoseTemp = m_currentPose;
            m_currentPose = root.CreateDefaultReducedState();
            m_currentPose.Remap(oldCurrentPoseTemp);
#endif
        }

        if (m_desiredPose == null || m_desiredPose.NumOfChildLinks != root.numLinks())
        {
            tntReducedState oldDesiredPoseTemp = m_desiredPose;
            m_desiredPose = root.CreateDefaultReducedState();
            m_desiredPose.Remap(oldDesiredPoseTemp);
        }
        else if (m_protectDesiredPoseScriptObject)
            m_desiredPose = m_desiredPose.Copy();

		if (m_protectControlParamsScriptObject)
			m_controlParams = m_controlParams.Copy();

        if (m_limbs.m_upperBack == null)
        {
            Transform torsoTrans = containerTrans.Find("lowerBack");
            if (torsoTrans != null)
                m_limbs.m_upperBack = torsoTrans.GetComponent<tntLink>();
        }
        if (m_limbs.m_lHand == null)
        {
            Transform lHandTrans = containerTrans.Find("lElbow"); // conservative lookup based on T-Rex
            if (lHandTrans != null)
                m_limbs.m_lHand = lHandTrans.GetComponent<tntLink>();
        }
        if (m_limbs.m_rHand == null)
        {
            Transform rHandTrans = containerTrans.Find("rElbow"); // conservative lookup based on T-Rex
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
            Transform headTrans = containerTrans.Find("neckJoint2");
			if (headTrans != null)
                m_limbs.m_neck = headTrans.GetComponent<tntLink>();
		} 

        if (m_limbs.m_tail == null)
        {
            Transform tail1Trans = containerTrans.Find("tailJ3");   // conservative lookup based on T-Rex
			if (tail1Trans != null)
                m_limbs.m_tail = tail1Trans.GetComponent<tntLink>();
        }

        if (m_limbs.m_upperBack != null && !root.ContainsLink(m_limbs.m_upperBack, false))
            m_limbs.m_upperBack = null;

        if (m_limbs.m_lHand != null && !root.ContainsLink(m_limbs.m_lHand, false))
            m_limbs.m_lHand = null;

        if (m_limbs.m_rHand != null && !root.ContainsLink(m_limbs.m_rHand, false))
            m_limbs.m_rHand = null;

        if (m_limbs.m_lToes != null && !root.ContainsLink(m_limbs.m_lToes, false))
            m_limbs.m_lToes = null;

        if (m_limbs.m_rToes != null && !root.ContainsLink(m_limbs.m_rToes, false))
            m_limbs.m_rToes = null;

        if (m_limbs.m_neck != null && !root.ContainsLink(m_limbs.m_neck, false))
            m_limbs.m_neck = null;

        if (m_limbs.m_tail != null && !root.ContainsLink(m_limbs.m_tail, false))
            m_limbs.m_tail = null;

        if (m_currentPose == null || m_currentPose.m_values.Length < root.numLinks())
        {
            Debug.LogError("Please assign the initial current pose to the controller!");
            return false;
        }

        if (m_desiredPose == null || m_desiredPose.m_values.Length < root.numLinks())
        {
            Debug.LogError("Please assign the initial desired pose to the controller!");
            return false;
        }

        if (m_controlParams == null)
        {
                Debug.LogError("Please assign control parameters to the controller!");
                return false;
        }

		unsafe
		{
            fixed (float* pControlParams = &m_controlParams.m_params[0])
			fixed(float* pCurrentPose = &m_currentPose.m_values[0])
            fixed(float* pDesiredPose = &m_desiredPose.m_values[0])
			fixed(Vector3* lEEPos = &m_leftEEPos)
			fixed(Vector3* rEEPos = &m_rightEEPos)

			fixed(void* pState = &controllerState.stridePhase)
			{
                m_controller = TNT.acCreateTRexController(root.GetWorld(), root.GetBase(),
                    m_limbs.m_upperBack ? m_limbs.m_upperBack.GetIndex() : -1,
                    m_limbs.m_lHand ? m_limbs.m_lHand.GetIndex() : -1,
                    m_limbs.m_rHand ? m_limbs.m_rHand.GetIndex() : -1,
                    m_limbs.m_lToes ? m_limbs.m_lToes.GetIndex() : -1,
                    m_limbs.m_rToes ? m_limbs.m_rToes.GetIndex() : -1,
                    m_limbs.m_neck ? m_limbs.m_neck.GetIndex() : -1,
                    m_limbs.m_tail ? m_limbs.m_tail.GetIndex() : -1,
                    pControlParams, m_controlParams.m_params.Length,
                    pCurrentPose,
                    pDesiredPose,
                    m_keepRootPosition,
					m_constraintPDFix
				    );

                if (m_controller != IntPtr.Zero)
                {
                    TNT.acInstallCommonControllerSharedMemoryBuffers(m_controller,
                        m_reflectCurrentPose ? pCurrentPose : null,
                        m_reflectDesiredPose ? pDesiredPose : null,
                        pControlParams);

                    TNT.acInstallTRexControllerSharedMemoryBuffers(m_controller,
                        m_reflectControllerState ? pState : null,
                        lEEPos, rEEPos);
                }       
            }
        }

        TNT.acInitRootPdParams(m_controller, m_rootPdParams.controlled, m_rootPdParams.kp,
		                            m_rootPdParams.kd, m_rootPdParams.maxAbsTorque,
		                            m_rootPdParams.scale.x,
		                            m_rootPdParams.scale.y,
		                            m_rootPdParams.scale.z,
		                            m_rootPdParams.relToCharFrame);

		m_pdParamsIndexMap = new int[root.numLinks()];
		// Walk through articultion links in order and add ControlParams to each of them
        for (int i = 0; i < root.numLinks(); ++i)
			m_pdParamsIndexMap[i] = -1;
        for (int i = 0; i < m_pdParams.Count; ++i)
        {
            int jointIndex = root.NameToIndex(m_pdParams[i].name);
            if (jointIndex == -2)
                continue;
            m_pdParamsIndexMap[jointIndex] = i;
            Debug.Log(m_pdParams[i].name + " => " + root.NameToIndex(m_pdParams[i].name));
        }
        for (int i = 0; i < root.numLinks(); ++i)
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

	public int JointToPdIndex(int jointIndex)
	{
		return m_pdParamsIndexMap[jointIndex];
	}

	public PDParams GetPdParams(int jointIndex)
	{
		return m_pdParams[m_pdParamsIndexMap[jointIndex]];
	}

    public void UpdatePdParams(int jointIndex, float kp, float kd, float maxTorque)
    {
        PDParams param = m_pdParams[m_pdParamsIndexMap[jointIndex]];
        if (param.kp == kp && param.kd == kd && param.maxAbsTorque == maxTorque)
            return;
        param.kp = kp;
        param.kd = kd;
        param.maxAbsTorque = maxTorque;
        TNT.acUpdateLinkPdParams(m_controller, jointIndex, kp, kd, maxTorque);
    }
 
	public override void UpdateDebugVisualizers()
	{
		if (m_leftFootTarget != null)
			m_leftFootTarget.transform.position = m_leftEEPos;
		if (m_rightFootTarget != null)
			m_rightFootTarget.transform.position = m_rightEEPos;
	}
}