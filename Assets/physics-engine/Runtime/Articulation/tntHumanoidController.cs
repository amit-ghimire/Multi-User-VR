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
public class tntHumanoidControllerState
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

public class tntHumanoidController : tntController
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
    public bool m_reflectControllerState;   // dynamically reflect any changes in controller state ScriptableObject in the engine
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

    //-------------------Dynamic Control Parameters----------------- 
    public PDParamSet m_pdParamSet;
    public tntHumanoidControlParams m_controlParams;
    public tntHumanoidBlendSample[] m_controlParamsBlendSamples;
    public tntHumanoidControllerState controllerState;

    public Vector3 m_forward = new Vector3(0, 0, 1);
    public Vector3 m_right = new Vector3(1, 0, 0);

    public tntBase m_root = null;

    public tntHumanoidController()
    {
        m_keepRootPosition = true;
        m_reflectDesiredPose = true;
        m_protectDesiredPoseScriptObject = true;
        m_protectControlParamsScriptObject = true;
        m_reflectControllerState = true;
        m_protectPDParamSetScriptObject = true;

        m_currentPose = null;
        m_desiredPose = null;
        m_controlParams = null;
        m_controller = IntPtr.Zero;
        m_worldIndex = -1;
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

        if (m_limbs.m_lowerBack != null && !m_root.ContainsLink(m_limbs.m_lowerBack, false))
            m_limbs.m_lowerBack = null;

        if (m_limbs.m_upperBack != null && !m_root.ContainsLink(m_limbs.m_upperBack, false))
            m_limbs.m_upperBack = null;

        if (m_limbs.m_neck != null && !m_root.ContainsLink(m_limbs.m_neck, false))
            m_limbs.m_neck = null;

        if (m_limbs.m_lShoulder != null && !m_root.ContainsLink(m_limbs.m_lShoulder, false))
            m_limbs.m_lShoulder = null;

        if (m_limbs.m_rShoulder != null && !m_root.ContainsLink(m_limbs.m_rShoulder, false))
            m_limbs.m_rShoulder = null;

        if (m_limbs.m_lToes != null && !m_root.ContainsLink(m_limbs.m_lToes, false))
            m_limbs.m_lToes = null;

        if (m_limbs.m_rHand != null && !m_root.ContainsLink(m_limbs.m_rHand, false))
            m_limbs.m_rHand = null;

        if (m_limbs.m_lHand != null && !m_root.ContainsLink(m_limbs.m_lHand, false))
            m_limbs.m_lHand = null;

        if (m_limbs.m_rToes != null && !m_root.ContainsLink(m_limbs.m_rToes, false))
            m_limbs.m_rToes = null;

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
            fixed (float* pControlParams = &m_controlParams.m_params[0])
            fixed (float* pCurrentPose = &m_currentPose.m_values[0])
            fixed (float* pDesiredPose = &m_desiredPose.m_values[0])
            fixed (Vector3* pForward = &m_forward)
            fixed (Vector3* pRight = &m_right)
            fixed (void* pState = &controllerState.stridePhase)
            {
                m_controller = TNT.acCreateHumanoidController(m_root.GetWorld(), m_root.GetBase(),
                    m_limbs.m_lowerBack ? m_limbs.m_lowerBack.GetIndex() : -1,
                    m_limbs.m_upperBack ? m_limbs.m_upperBack.GetIndex() : -1,
                    m_limbs.m_neck ? m_limbs.m_neck.GetIndex() : -1,
                    m_limbs.m_lShoulder ? m_limbs.m_lShoulder.GetIndex() : -1,
                    m_limbs.m_rShoulder ? m_limbs.m_rShoulder.GetIndex() : -1,
                    m_limbs.m_lToes ? m_limbs.m_lToes.GetIndex() : -1,
                    m_limbs.m_rToes ? m_limbs.m_rToes.GetIndex() : -1,
                    m_limbs.m_lHand ? m_limbs.m_lHand.GetIndex() : -1,
                    m_limbs.m_rHand ? m_limbs.m_rHand.GetIndex() : -1,
                    m_limbs.m_limbTrackingKp,
                    m_limbs.m_limbTrackingKd,
                    m_limbs.m_limbMaxTrackingForce,
                    pControlParams, m_controlParams.m_params.Length,
                    pCurrentPose,
                    pDesiredPose,
                    m_blendGranularity,
                    m_keepRootPosition,
                    m_limbs.m_antiLegCrossing,
                    m_useBlendSpace,
                    !m_disableGRFSolver,
                    m_limbs.m_stepRelativeToCOM,
                    m_limbs.m_stepRelativeToRoot,
					m_constraintPDFix,
                    pForward,
                    pRight);

                if (m_controller != IntPtr.Zero)
                {
                    TNT.acInstallCommonControllerSharedMemoryBuffers(m_controller,
                        m_reflectCurrentPose ? pCurrentPose : null,
                        m_reflectDesiredPose ? pDesiredPose : null,
                        pControlParams);

                    TNT.acInstallHumanoidControllerSharedMemoryBuffers(m_controller,
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

        for (int i = 0; i < m_root.numLinks(); ++i)
        {
            tntChildLink childLink = m_root.getChildLink(i);

            TNT.acAddLinkPdParams(m_controller, childLink.m_kp != 0 || childLink.m_kd != 0,
                                  childLink.m_kp, childLink.m_kd, childLink.m_maxPDTorque,
                                  0, 0, 0, false);

            if (childLink.m_useSoftConstraint)
                TNT.acUseConstraintBasedPDControl(m_controller, childLink.GetIndex());
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

    public void UpdatePdParams(int jointIndex, float kp, float kd, float maxTorque, float kpmod = 1, float kdmod =1)
    {
        if (jointIndex < 0 || jointIndex >= m_root.numLinks())
            return;
        tntChildLink childLink = m_root.getChildLink(jointIndex);
        if (childLink.m_kp == kp && childLink.m_kd == kd && childLink.m_maxPDTorque == maxTorque)
            return;
        childLink.m_kp = kp;
        childLink.m_kd = kd;
        childLink.m_maxPDTorque = maxTorque;
        m_pdParamSet.UpdatePdParams(jointIndex, kp, kd, maxTorque);
    }

    public void EnableLeftShoulderSwing(bool enabled)
    {
        TNT.acEnableLeftShoulderSwing(m_controller, enabled);
    }

    public void EnableRightShoulderSwing(bool enabled)
    {
        TNT.acEnableRightShoulderSwing(m_controller, enabled);
    }

    public bool CheckIfLinkExist(tntLink link)
    {
        return m_root.ContainsLink(link, false);
    }
}
