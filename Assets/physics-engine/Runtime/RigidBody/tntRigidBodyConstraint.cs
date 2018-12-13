using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using PhysicsAPI;

/**
 * @brief Abstract class representing a rigid-body joint constraint
 */
public abstract class tntRigidBodyConstraint : MonoBehaviour
{
    private static List<tntRigidBodyConstraint> sConstraints = new List<tntRigidBodyConstraint> ();
    private static int sLastAddedToWorld = 0;

    protected tntRigidBodyConstraint(Type type) { this.type = type; }

    public enum Type
    {
        Invalid,
        Ball,
        Hinge,
        Fixed
    }

    /// Type of represented joint
    [HideInInspector]
    public Type type { get; protected set; }

    internal tntWorld m_world;
    private IntPtr worldInKernel { get { return m_world ? m_world.worldInKernel : IntPtr.Zero; } }

    [NonSerialized] internal IntPtr m_constraint = IntPtr.Zero;
	[NonSerialized] internal bool m_added = false;

	// Public Properties
	[SerializeField] public bool m_disableSelfCollision;

    // FIXME: ideally, we only need one rigid body as gameObject itself can have another one
    // but this become a bit problem for the existing prefabs/objects
    [SerializeField] tntRigidBody m_bodyA;
    [SerializeField] tntRigidBody m_bodyB;

#if UNITY_EDITOR
    [SerializeField] bool m_showJoint;
    [SerializeField] bool m_visualEditor;
#endif

    public tntRigidBody bodyA {
        get { return m_bodyA; }
    }

    public tntRigidBody bodyB {
        get { return m_bodyB; }
    }

    public Vector3 m_pivotA;
	public Vector3 m_pivotB;
	public Vector3 m_axisA;
	public Vector3 m_axisB;
	public float m_breakingImpulse = 1e+38f;
	public int m_overrideNumIterations = -1;
	public apJointFeedback m_feedback = new apJointFeedback {
        m_accumulatedImpulse = 0,
        m_appliedImpulse = 0,
        m_broken = false
    };

    static internal void UpdateToWorld (tntWorld world) {
        for (; sLastAddedToWorld < sConstraints.Count; ++sLastAddedToWorld) {
            sConstraints[sLastAddedToWorld].AddIfActive (world);
        }
    }

    static internal void WorldIsRemoved (tntWorld world) {
        for (int i = 0; i < sConstraints.Count; ++i) {
            if (sConstraints[i] != null) {
                world.RemoveConstraint (sConstraints[i]);
            }
        }
    }

    internal bool Added {
        get { return m_added; }
    }

#if UNITY_EDITOR
    void Reset () {
        // set it default to itself
        m_bodyA = GetComponent<tntRigidBody> ();
    }
#endif

    void Awake()
    {
        AddJointToBodies ();
    }

    void CreateConstraintInKernel (bool recreated = false)
    {
        if (recreated && m_constraint != IntPtr.Zero) {
            tntEntityAndJointFactory.DestroyConstraintInKernel(this);
        }

        if (m_constraint == IntPtr.Zero
            && AreBothRigidBodiesEnabled()) {
            tntEntityAndJointFactory.CreateConstraintInKernel(this);
        }
    }

    void OnDestroy()
	{
        tntEntityAndJointFactory.DestroyConstraintInKernel (this);

        if (m_bodyA != null) {
            m_bodyA.RemoveJoint (this);
        }
        if (m_bodyB != null) {
            m_bodyB.RemoveJoint (this);
        }

        m_bodyA = null;
        m_bodyB = null;
	}

    public void RemoveIfRigidBodyBecomeInactive (tntWorld world) {
        if (m_added && !AreBothRigidBodiesEnabled ()) {
            world.RemoveConstraint (this);
        }
    }

	void AddJointToBodies()
	{
        if (m_bodyA != null) {
            m_bodyA.AddJoint (this);
        }
        if (m_bodyB != null) {
            m_bodyB.AddJoint (this);
        }
    }

    void OnEnable () {
        if (m_world) AddIfActive (m_world, false);
    }

    void OnDisable () {
        if (m_world) m_world.RemoveConstraint (this);
    }

    internal bool AreBothRigidBodiesEnabled () {
        return (m_bodyA != null && m_bodyA.Added) &&
            (m_bodyB != null && m_bodyB.Added);
    }

    internal void AddIfActive (tntWorld world, bool recreated = false) {
        CreateConstraintInKernel (recreated);

        m_world = world; //FIXME: Once have the enable list, remove this

        if (enabled && gameObject.activeInHierarchy) {
            world.AddConstraint (this);
        }

    }
   
    /**
     * Computes pivot position on body B by transforming pivot on body A
     */
    public void AutoFillPivotB()
	{
		m_pivotB = MathUtils.TransformPivot(m_pivotA, m_bodyA.transform, 
		                                    m_bodyB.transform);
		MathUtils.RoundToGrid(ref m_pivotB);
	}

    /**
     * Computes pivot position on body A by transforming pivot on body B
     */
	public void AutoFillPivotA()
	{
		m_pivotA = MathUtils.TransformPivot(m_pivotB,
		                                    m_bodyB.transform,
		                                    m_bodyA.transform
		                                    );
		MathUtils.RoundToGrid(ref m_pivotA);
	}

    /**
     * Transforms pivot on body A from A's local frame to world coordinates
     * @return pivot on body A expressed in world coordinates
     */
    public Vector3 PivotAToWorld()
	{
		return MathUtils.PointToWorld(m_pivotA, m_bodyA.transform);
	}

    /**
     * Transforms pivot on body B from B's local frame to world coordinates
     * @return pivot on body B expressed in world coordinates
     */
	public Vector3 PivotBToWorld()
	{
		return MathUtils.PointToWorld(m_pivotB, m_bodyB.transform);
	}

    /**
     * Sets pivot on body A
     * @param pivotAWorld pivot on body A expressed in world coordinates
     */
    public void PivotAFromWorld(Vector3 pivotAWorld)
	{
		m_pivotA = MathUtils.PointFromWorld(pivotAWorld, m_bodyA.transform);
		MathUtils.RoundToGrid(ref m_pivotA);
	}

    /**
     * Set the threshold value for this joint's reaction impulse
     * @param breakingImpulse threshold on the allowable reaction impulse for this joint
     * @remark This joint will break if it needs to exert impulse equal to or larger than breakingImpulse
     */
    public void SetBreakingImpulse(float breakingImpulse)
	{
		m_breakingImpulse = breakingImpulse;
		if (m_constraint != IntPtr.Zero)
			TNT.apSetConstraintBreakingImpulse (m_constraint, m_breakingImpulse);
	}
}
