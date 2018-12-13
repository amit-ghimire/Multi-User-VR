using UnityEngine;
using System;
using PhysicsAPI;


public class tntIKActuator : MonoBehaviour
{
    private const float defaultSolverToleranceThreshold = 0.001f;

    public tntLink m_IKRoot;
    public tntChildLink m_endEffector;
    [SerializeField, HideInInspector]
    protected float m_solverToleranceThreshold = defaultSolverToleranceThreshold;

    public GameObject m_target;
	private float[] m_targetPos = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    public int m_Active;
    private IntPtr m_kernelIKActuator;
    private tntController m_controller;

    public tntIKActuator()
    {
        m_kernelIKActuator = IntPtr.Zero;
        m_target = null;
        m_Active = 0;
    }

    void Start()
    {
        m_controller = GetComponentInParent<tntController>();
        if (m_controller != null)
        {
            unsafe
            {
                fixed(float* pTargetPos = &m_targetPos[0])
                fixed(int* pActive = &m_Active)
                {
                    m_kernelIKActuator = TNT.acRegisterIKActuator(m_controller.GetEngineController(), m_endEffector.GetIndex(), m_IKRoot.GetIndex(), pTargetPos, pActive);
                    TNT.acSetIKActuatorToleranceThreshold(m_kernelIKActuator, m_solverToleranceThreshold);
                }
            }
        }
    }

    void Update()
    {
		if (m_target != null && m_kernelIKActuator != IntPtr.Zero)
        {
			SetTargetPos(m_target.transform.position);
			SetTargetRot(m_target.transform.rotation);
		}
    }

    void Stop()
    {
		if (m_controller != null && m_kernelIKActuator != IntPtr.Zero)
        {
            TNT.acUnregisterIKActuator(m_controller.GetEngineController(), m_kernelIKActuator);
			m_kernelIKActuator = IntPtr.Zero;
        }
    }

    public void SetTarget(GameObject go)
    {
        m_target = go;
    }

    public void SetTargetPos(Vector3 pos)
    {
        m_targetPos[0] = pos.x;
        m_targetPos[1] = pos.y;
        m_targetPos[2] = pos.z;
    }

	public void SetTargetRot(Quaternion rot)
	{
		m_targetPos[3] = rot.x;
		m_targetPos[4] = rot.y;
		m_targetPos[5] = rot.z;
		m_targetPos[6] = rot.w;
	}

    public void EnableControl()
    {
        m_Active = 1;
    }

    public void DisableControl()
    {
        m_Active = 0;
    }

    public float SolverToleranceThreshold
    {
        get { return m_solverToleranceThreshold; }
        set
        {
            if (value != m_solverToleranceThreshold)
            {
                m_solverToleranceThreshold = value;
                if (m_kernelIKActuator != IntPtr.Zero)
                {
                    TNT.acSetIKActuatorToleranceThreshold(m_kernelIKActuator, m_solverToleranceThreshold);
                }
            }
        }
    }
}


