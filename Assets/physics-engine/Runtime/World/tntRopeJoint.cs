using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using PhysicsAPI;

/**
 * @brief Utility class performing cubic Bezier interpolation used for tntRopeJoint rendering
 */
public class BezierCurve
{
    private Vector3 p0, p1, p2, p3;

    public BezierCurve(Vector3 _p0, Vector3 _p1, Vector3 _p2, Vector3 _p3)
    {
        p0 = _p0;
        p1 = _p1;
        p2 = _p2;
        p3 = _p3;
    }

    public Vector3 getAt(float t)
    {
        Vector3 pos = (1-t)*(1-t)*(1-t)*p0;
        pos += 3*(1-t)*(1-t)*t*p1;
        pos += 3*(1-t)*t*t*p2;
        pos += t*t*t*p3;
        return pos;
    }
}

/**
 * @brief Class representing a rope-like connection between two endpoints, A & B, where A and B can each be either a tntRigidBody or a tntLink
 */
public class tntRopeJoint : MonoBehaviour
{
    private IntPtr m_ropeJoint;
    private LineRenderer m_ropeRenderer;
	private float[] m_worldVel;		// sensor for world velocities of two pivot points: 6 floats

    public MonoBehaviour m_partA;
    public MonoBehaviour m_partB;
    public Vector3 m_pivotA;
    public Vector3 m_pivotB;
	public bool m_showJoint;
	public bool m_visualEditor;
    public float m_maxLength;
    public float m_maxForce;
    public float m_stiffness;
    public float m_damping;
	public int m_broken;

    private static HashSet<tntRopeJoint> ropeJoints = new HashSet<tntRopeJoint>();


    /**
     * Returns pointer to the internal rope joint object
     * @return native pointer to the internal rope joint object
     */
    public IntPtr GetRopeJoint()
    {
        return m_ropeJoint;
    }

    /**
     * Checks if this rope joint has already been deleted by the kernel
     * @return true if kernel has already deleted this rope joint, false otherwise
     */
    public bool GetBroken()
    {
        return (m_broken == 1);
    }

    /**
     * Removes the underlying internal rope joint
     */
    public void RemoveRope() // make kernel remove this rope joint
    {
        if (m_ropeJoint != IntPtr.Zero)
        {
            if (!GetBroken()) TNT.apRemoveRopeJoint(m_ropeJoint);
            m_ropeJoint = IntPtr.Zero;
        }
    }

    tntRopeJoint()
    {
        m_ropeJoint = IntPtr.Zero;
        m_broken = 0;
		m_worldVel = new float[6];
		m_worldVel[0] = m_worldVel[1] = m_worldVel[2] = m_worldVel[3] = m_worldVel[4] = m_worldVel[5] = 0;
    }

    /**
     * Creates a new rope internally and assigns it to this tntRopeJoint
     */
    public bool addRope()
    {
		tntRigidBody body1 = m_partA.gameObject.GetComponent<tntRigidBody>();
		tntRigidBody body2 = m_partB.gameObject.GetComponent<tntRigidBody>();
		tntLink link1 = m_partA.gameObject.GetComponent<tntLink>();
		tntLink link2 = m_partB.gameObject.GetComponent<tntLink>();
        if (body1 != null && body2 != null)
        {
			unsafe
			{
				fixed(float* pMaxLength = &m_maxLength)
				fixed(float* pMaxForce = &m_maxForce)
				fixed(int* pBroken = &m_broken)
				fixed(float* pWorldVel = &m_worldVel[0])
                fixed(Vector3* pivotA = &m_pivotA)
                fixed(Vector3* pivotB = &m_pivotB)
				{
                    m_ropeJoint = TNT.apAddRopeJointForRigidBodyAndRigidBody(body1.GetRigidBody(), pivotA, body2.GetRigidBody(), pivotB,
					                                                         m_stiffness, m_damping, 
					                                                         pMaxLength, pMaxForce, pBroken, pWorldVel);
					
				}
			}
        }
        else if (link1 != null && link2 != null)
        {
			unsafe
			{
				fixed(float* pMaxLength = &m_maxLength)
				fixed(float* pMaxForce = &m_maxForce)
				fixed(int* pBroken = &m_broken)
				fixed(float* pWorldVel = &m_worldVel[0])
                fixed(Vector3* pivotA = &m_pivotA)
                fixed(Vector3* pivotB = &m_pivotB)
				{
                    m_ropeJoint = TNT.apAddRopeJointForLinkAndLink(link1.GetBase(), link1.GetIndex(), pivotA, link2.GetBase(), link2.GetIndex(), pivotB,
					                                               m_stiffness, m_damping,
					                                               pMaxLength, pMaxForce, pBroken, pWorldVel);
					
				}
			}
        }
        else if (body1 != null && link2 != null)
        {
			unsafe
			{
				fixed(float* pMaxLength = &m_maxLength)
				fixed(float* pMaxForce = &m_maxForce)
				fixed(int* pBroken = &m_broken)
				fixed(float* pWorldVel = &m_worldVel[0])
                fixed(Vector3* pivotA = &m_pivotA)
                fixed(Vector3* pivotB = &m_pivotB)
				{
                    m_ropeJoint = TNT.apAddRopeJointForRigidBodyAndLink(body1.GetRigidBody(), pivotA, link2.GetBase(), link2.GetIndex(), pivotB,
					                                                    m_stiffness, m_damping,
					                                                    pMaxLength, pMaxForce, pBroken, pWorldVel);
					
				}
			}
        }
		else if (body2 != null && link1 != null)
		{
			Debug.LogError("PartA:tntLink & PartB:tntRigidBody are unsupported. Please swap to PartA:tntRigidBody and PartB:tntLink");
			return false;
		}
		else
		{
			Debug.LogError("Unsported PartA:" + m_partA.name + " PartB:" + m_partB.name);
			return false;
		}
        m_broken = 0;
		return true;
    }

    private bool isStarted = false;

    void OnEnable()
    {
        if (isStarted)
        {
            if (!addRope())
                return;
            m_ropeRenderer = gameObject.GetComponent<LineRenderer>();
            ropeJoints.Add(this);
        }
    }

    void Start()
    {
        isStarted = true;
        OnEnable();
    }

    void OnDisable()
    {
        RemoveRope();
        ropeJoints.Remove(this);

        //- No need to activate connected bodies. Rope doesn't let the bodies deactivate.
    }

    /**
     * Renders this rope using a heuristic approach based on cubic Bezier interpolation
     */
	public void RenderRope(LineRenderer renderer)
	{
		if (renderer == null)
			return;
		Vector3 pos1 = PivotAToWorld();
		Vector3 pos2 = PivotBToWorld();

		Vector3 v21 = pos1 - pos2;
		float v21Dist = v21.magnitude;
		if (v21Dist >= m_maxLength)
		{
            renderer.positionCount = 2;
			renderer.SetPosition(0, pos1);
			renderer.SetPosition(1, pos2);
		}
		else
		{
			float surplus = (m_maxLength - v21Dist) / m_maxLength;
			Vector3 pos1_t = pos1 + (new Vector3(0,-1,0)) * surplus;
			Vector3 pos2_t = pos2 + (new Vector3(0,-1,0)) * surplus;
			BezierCurve curve = new BezierCurve(pos1, pos1_t, pos2_t, pos2);
			int numChain = (int)(m_maxLength * 10);
			renderer.positionCount = numChain;
			for (int i = 0; i < numChain; i++)
			{
				renderer.SetPosition(i, curve.getAt((float)i/(numChain-1)));
			}
		}
	}
	
	void Update()
	{
		if (m_ropeJoint == IntPtr.Zero)
			return;
        if (GetBroken()) m_ropeJoint = IntPtr.Zero;
        if (GetBroken() && m_ropeRenderer)
		{
			m_ropeRenderer.positionCount = 1;
            return;
		}
		if (m_ropeRenderer) RenderRope(m_ropeRenderer);
	}

    /**
     * Transforms pivot/rope attachment location from endpoint A's local frame to world coordinates
     * @return endpoint A's pivot expressed in world coordinates
     */
	public Vector3 PivotAToWorld()
	{
		return MathUtils.PointToWorld(m_pivotA, m_partA.transform);
	}

    /**
     * Sets pivot/rope attachment location for endpoint A
     * @param pivotAWorld endpoint A's pivot expressed in world coordinates
     */
    public void PivotAFromWorld(Vector3 pivotAWorld)
	{
		m_pivotA = MathUtils.PointFromWorld (pivotAWorld, m_partA.transform);
	}

    /**
     * Transforms pivot/rope attachment location from endpoint B's local frame to world coordinates
     * @return endpoint B's pivot expressed in world coordinates
     */
	public Vector3 PivotBToWorld()
	{
		return MathUtils.PointToWorld(m_pivotB, m_partB.transform);
	}

    /**
     * Sets pivot/rope attachment location for endpoint B
     * @param pivotBWorld endpoint B's pivot expressed in world coordinates
     */
    public void PivotBFromWorld(Vector3 pivotBWorld)
	{
		m_pivotB = MathUtils.PointFromWorld (pivotBWorld, m_partB.transform);
	}

    public static void VerifyRopesHaveValidBodies()
    {
        List<tntRopeJoint> jointsToDisable = new List<tntRopeJoint>();
        foreach(var j in ropeJoints)
        {
            if (!j.m_partA || !j.m_partB) jointsToDisable.Add(j);
        }

        foreach(var j in jointsToDisable)
        {
            j.enabled = false;
        }
    }
}