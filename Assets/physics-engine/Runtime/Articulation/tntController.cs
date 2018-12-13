using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.ComponentModel;
using System.Reflection;

using PhysicsAPI;

[Serializable]
public class LimbConfiguration
{
    // Static limb configuration
    public tntLink m_lowerBack;
    public tntLink m_upperBack;
    public tntLink m_neck;
    public tntLink m_lShoulder;
    public tntLink m_rShoulder;
    public tntLink m_lToes;
    public tntLink m_rToes;
    public tntLink m_lHand;
    public tntLink m_rHand;
    public tntLink m_tail;

    // Dynamic limb control parameters synced to kernel every frame
    public GameObject m_headTarget = null;
    public GameObject m_lHandTarget = null;
    public GameObject m_rHandTarget = null;
    public GameObject m_lFootTarget = null;
    public GameObject m_rFootTarget = null;
    public GameObject m_rootTarget = null;
    public float m_limbTrackingKp = 1000;
    public float m_limbTrackingKd = 100;
    public float m_limbMaxTrackingForce = 10000;
    public float m_deadThresholdSwingLeg = 1.0f;
    public float m_deadThresholdGRF = 1.0f;
    public bool m_kickIfDead = false;
    public bool m_antiLegCrossing = false;   // anti leg crossing
    public bool m_enableLean = false;
    // Step control only supports 3 possible body center
    // 1) Relative to COM: m_stepRelativeToCOM  = true
    // 2) Relative to root: m_stepRelativeToCOM  = false, m_m_stepRelativeToRoot = true;
    // 3) Relative to Body Frame: m_stepRelativeToCOM  = false, stepRelativeToRoot = false;
    public bool m_stepRelativeToCOM = false;
    public bool m_stepRelativeToRoot = false;
};

public class tntController : MonoBehaviour
{
    protected IntPtr m_controller = IntPtr.Zero;
    protected int m_worldIndex = -1;

    public LimbConfiguration m_limbs;
	public bool m_constraintPDFix;

    public tntController()
    {
		m_controller = IntPtr.Zero;
        m_worldIndex = -1;
		m_constraintPDFix = false;
    }

    public IntPtr GetEngineController() { return m_controller; }

    public void SetWorldIndex(int idx) { m_worldIndex = idx; }
    public int GetWorldIndex() { return m_worldIndex; }

    public static Transform FindInChildren(Transform thisOne, string name)
    {
        foreach(Transform t in thisOne.GetComponentsInChildren<Transform>())
        {
            if(t.name == name)
                return t;
        }

        return null;
    }

    public virtual void Cleanup()
    {
        if (m_controller != IntPtr.Zero) {
			TNT.acDeleteController(m_controller);
        }
        m_controller = IntPtr.Zero;
    }

	void OnEnable()
	{
		// TBD
	}

	void OnDisable()
	{
		// TBD
	}

	void OnDestroy()
	{
		Cleanup();
	}

	void OnApplicationQuit()
	{
		Cleanup();
	}

    public virtual bool SerializeToEngine() { return false; }
	public virtual void SyncWithEngine() {}
	public virtual void UpdateDebugVisualizers() {}

    public virtual void UpdateControllerState()
    {
		if (m_limbs == null)
			return;
        bool trackHead = (m_limbs.m_headTarget != null);
        bool tracklHand = (m_limbs.m_lHandTarget != null);
        bool trackrHand = (m_limbs.m_rHandTarget != null);
        bool tracklFoot = (m_limbs.m_lFootTarget != null);
        bool trackrFoot = (m_limbs.m_rFootTarget != null);
        bool trackRoot = (m_limbs.m_rootTarget != null);

        Vector3 headPos, lHandPos, rHandPos, lFootPos, rFootPos, rootPos;
        Quaternion headRot, lHandRot, rHandRot, lFootRot, rFootRot, rootRot;
        if (trackHead)
        {
            headPos = m_limbs.m_headTarget.transform.position;
            headRot = m_limbs.m_headTarget.transform.rotation;
        }
        if (tracklHand)
        {
            lHandPos = m_limbs.m_lHandTarget.transform.position;
            lHandRot = m_limbs.m_lHandTarget.transform.rotation;
        }
        if (trackrHand)
        {
            rHandPos = m_limbs.m_rHandTarget.transform.position;
            rHandRot = m_limbs.m_rHandTarget.transform.rotation;
        }
        if (tracklFoot)
        {
            lFootPos = m_limbs.m_lFootTarget.transform.position;
            lFootRot = m_limbs.m_lFootTarget.transform.rotation;
        }
        if (trackrFoot)
        {
            rFootPos = m_limbs.m_rFootTarget.transform.position;
            rFootRot = m_limbs.m_rFootTarget.transform.rotation;
        }
        if (trackRoot)
        {
            rootPos = m_limbs.m_rootTarget.transform.position;
            rootRot = m_limbs.m_rootTarget.transform.rotation;
        }
        unsafe
        {
            TNT.acUpdateControllerState(
                m_controller,
                trackHead ? &headPos : null,
                trackHead ? &headRot : null,
                tracklHand ? &lHandPos : null,
                tracklHand ? &lHandRot : null,
                trackrHand ? &rHandPos : null,
                trackrHand ? &rHandRot : null,
                tracklFoot ? &lFootPos : null,
                tracklFoot ? &lFootRot : null,
                trackrFoot ? &rFootPos : null,
                trackrFoot ? &rFootRot : null,
                trackRoot ? &rootPos : null,
                trackRoot ? &rootRot : null,
                m_limbs.m_limbTrackingKp,
                m_limbs.m_limbTrackingKd,
                m_limbs.m_limbMaxTrackingForce,
                m_limbs.m_deadThresholdSwingLeg,
                m_limbs.m_deadThresholdGRF,
                m_limbs.m_kickIfDead,
                m_limbs.m_antiLegCrossing,
                m_limbs.m_stepRelativeToCOM,
                m_limbs.m_stepRelativeToRoot
            );
        }
        m_limbs.m_kickIfDead = false;       // Edge trigger
        //Debug.Log("kicked=" + m_limbs.m_kickIfDead);
    }

    public void ActiviateSupportLeg(int legIndex, bool enabled)
    {
        TNT.acActiviateSupportLeg(m_controller, legIndex, enabled);
    }

    public void EnableNeckControl(bool enabled)
    {
        TNT.acEnableNeckControl(m_controller, enabled);
    }

    public void EnableTorsoControl(bool enabled)
	{
		TNT.acEnableTorsoControl(m_controller, enabled);
	}

	public void SetControllerDead(bool enabled)
    {
        TNT.acSetControllerDead(m_controller, enabled);
    }

    //create a wrapper function to make the call easier
    public Vector3 GetCOM()
    {
        Vector3 v;
        unsafe { TNT.acGetCOM(m_controller, &v); }
        return v;
    }

    public void SetDead(bool dead)
    {
        TNT.acSetControllerDead(m_controller, dead);
    }
}
