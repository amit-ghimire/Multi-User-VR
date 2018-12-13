using UnityEngine;
using System;
using PhysicsAPI;

/**
 * @brief Class representing a base for articulation constraints
 */
public abstract class tntArticulationConstraint : MonoBehaviour
{
    protected IntPtr m_constraint;
    protected bool m_added = false;

    // Public Properties
    public tntLink m_linkA;
    public tntLink m_linkB;
    public bool m_useBodyB;
    public tntRigidBody m_bodyB;
    public Vector3 m_pivotA;
    public Vector3 m_pivotB;
    public bool m_showJoint;
    public bool m_visualEditor;
    public float m_maxImpulse;
    public float m_breakingImpulse;
    public apJointFeedback m_feedback;

    public tntWorld m_world;

    public tntArticulationConstraint()
    {
        m_useBodyB = false;
        m_showJoint = false;
        m_visualEditor = false;
        m_maxImpulse = 1.0e+38f;
        m_breakingImpulse = 1.0e+38f;
        m_feedback.m_appliedImpulse = 0;
        m_feedback.m_accumulatedImpulse = 0;
        m_feedback.m_broken = false;
    }

    public abstract void AddConstraint();
    public abstract void RemoveConstraint();

    /**
     * Set the threshold value for this constraint's reaction impulse
     * @param breakingImpulse threshold on the allowable reaction impulse for this constraint
     * @remark This contraint will break if it needs to exert impulse equal to or larger than breakingImpulse
     */
    public abstract void SetBreakingImpulse(float breakingImpulse);

    /**
     * Computes pivot position on link A by transforming pivot on link B
     */
    public void AutoFillPivotA()
    {
        m_pivotA = MathUtils.TransformPivot(m_pivotB,
                                            m_useBodyB ? m_bodyB.transform : m_linkB.transform,
                                            m_linkA.transform
                                            );
        MathUtils.RoundToGrid(ref m_pivotA);
    }

    /**
     * Computes pivot position on link B by transforming pivot on link A
     */
    public void AutoFillPivotB()
    {
        m_pivotB = MathUtils.TransformPivot(m_pivotA, m_linkA.transform,
                                            m_useBodyB ? m_bodyB.transform : m_linkB.transform);
        MathUtils.RoundToGrid(ref m_pivotB);
    }

    void Awake()
    {
        if (m_world == null) { m_world = GameObject.FindObjectOfType<tntWorld>(); }
        m_world.EnsureDynamicWorld();
    }

    private bool isStarted = false;

    //- First AddArticulationP2PConstraint() must be called after Start is called already for attached tntRigidBody/tntLinks; therefore it's postponed in the first OnEnable() that happens before Start().
    //- When calling the first AddArticulationP2PConstraint() in Start() this magically works, although it might be expected not to, when the attached bodie's Start() methods are called after this constraint's Start().
    void OnEnable()
    {
        if (isStarted)
        {
            AddConstraint();
        }
    }

    void Start()
    {
        AddConstraint();
        isStarted = true;
    }

    void OnDisable()
    {
        RemoveConstraint();

        // Activate bodies that are now unconstrainted
        if (m_linkA && m_linkA.m_base != IntPtr.Zero) TNT.apActivateKinematicArticulation(m_linkA.m_base);
        if (m_useBodyB)
        {
            if (m_bodyB && m_bodyB.m_rigidBody != IntPtr.Zero) TNT.apActivateKinematicRigidBody(m_bodyB.m_rigidBody);
        }
        else
        {
            if (m_linkB && m_linkB.m_base != IntPtr.Zero) TNT.apActivateKinematicArticulation(m_linkB.m_base);
        }
    }

    void OnApplicationQuit()
    {
        RemoveConstraint();
    }

    /**
     * Transforms pivot on link A from A's local frame to world coordinates
     * @return pivot on link A expressed in world coordinates
     */
    public Vector3 PivotAToWorld()
    {
        return MathUtils.PointToWorld(m_pivotA, m_linkA.transform);
    }

    /**
     * Transforms pivot on link B from B's local frame to world coordinates
     * @return pivot on link B expressed in world coordinates
     */
    public Vector3 PivotBToWorld()
    {
        return MathUtils.PointToWorld(m_pivotB, m_useBodyB ? m_bodyB.transform : m_linkB.transform);
    }

    /**
     * Sets pivot on link A
     * @param pivotAWorld pivot on link A expressed in world coordinates
     */
    public void PivotAFromWorld(Vector3 pivotAWorld)
    {
        m_pivotA = MathUtils.PointFromWorld(pivotAWorld, m_linkA.transform);
        MathUtils.RoundToGrid(ref m_pivotA);
    }


}

