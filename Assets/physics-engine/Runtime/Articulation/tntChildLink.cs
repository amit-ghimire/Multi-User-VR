using UnityEngine;
using System;
using System.Linq;
using PhysicsAPI;

/**
 * @brief Joint's single degree-of-freedom info: DOF-type, motor, spring-damper and mobility limits settings
 */
[System.Serializable]
public class tntDofData
{
    public bool m_useMotor;         // If mobility motor is used
    public bool m_isPositionMotor;
    public float m_desiredPosition; // In degrees for rotational DOF
    public float m_desiredVelocity; // In degress/second for rotational DOF
    public float m_maxMotorForce;
    public float m_positionLockThreshold; // In degrees for rotational DOF
    public bool m_useAutomaticPositionLockThreshold;

    public float m_springStiffness;
    public float m_springDamping;
    public float m_neutralPoint;    // In degrees for rotational DOF

    public float m_continuousForce;

    public bool m_useLimit;
    public float m_limitLow;        // In degrees for rotational DOF
    public float m_limitHigh;       // In degrees for rotational DOF
    public float m_maxLimitForce;

    public bool m_rotational { get; private set; }

    public tntDofData(tntDofData data)
    {
        m_useMotor = data.m_useMotor;
        m_isPositionMotor = data.m_isPositionMotor;
        m_desiredPosition = data.m_desiredPosition;
        m_desiredVelocity = data.m_desiredVelocity;
        m_maxMotorForce = data.m_maxMotorForce;
        m_positionLockThreshold = data.m_positionLockThreshold;
        m_useAutomaticPositionLockThreshold = data.m_useAutomaticPositionLockThreshold;

        m_springStiffness = data.m_springStiffness;
        m_springDamping = data.m_springDamping;
        m_neutralPoint = data.m_neutralPoint;

        m_continuousForce = data.m_continuousForce;

        m_useLimit = data.m_useLimit;
        m_limitLow = data.m_limitLow;
        m_limitHigh = data.m_limitHigh;
        m_maxLimitForce = data.m_maxLimitForce;

        m_rotational = data.m_rotational;
    }

    public tntDofData(bool isRotational)
    {
        m_useMotor = false;
        m_isPositionMotor = false;
        m_desiredVelocity = 0f;
        m_desiredPosition = 0f;
        m_maxMotorForce = 0f;
        m_positionLockThreshold = 0f;
        m_useAutomaticPositionLockThreshold = true;

        m_springDamping = 0f;
        m_springStiffness = 0f;
        m_neutralPoint = 0f;

        m_continuousForce = 0f;

        m_useLimit = false;
        m_limitLow = 0f;
        m_limitHigh = 0f;
        m_maxLimitForce = 0f;

        m_rotational = isRotational;
    }
}

/**
 * @brief Abstract class representing a single non-base body/link in an articulation
 */
public abstract class tntChildLink : tntLink
{

	protected IntPtr m_mobilizer;

	// Public Properties
	public tntLink m_parent;
	public bool m_collideWithParent;

    public tntDofData[] m_dofData;

    public float m_breakingReactionImpulse = Mathf.Infinity;

    // Sensors
    protected float[] m_currentPosition;   // reflected from engine, in degrees for rotational DOFs
	public apJointFeedback m_feedback;	// reflected from engine: joint reaction forces

    // Joint mount points and axis
    public bool m_showJoint;
    public bool m_visualEditor;

    [SerializeField]
    private Vector3 m_pivotA;
    public virtual Vector3 PivotA
    {
        get { return m_pivotA; }  
        set { m_pivotA = value; }
    }

    [SerializeField]
    private Vector3 m_pivotB;
    public virtual Vector3 PivotB
    {
        get { return m_pivotB; }
        set { m_pivotB = value; }
    }

    // Only used for pose controllers
    public float m_kp = 0;
    public float m_kd = 0;
    public float m_maxPDTorque = 0;
    public bool m_useSoftConstraint = false;

    public float[] CurrentPosition
    {
        get { return m_currentPosition; }
    }

    public void SetAdded() { m_added = true; }

    public void SetMobilizer(IntPtr mobilizer) { m_mobilizer = mobilizer; }

    public tntChildLink() : base(Type.ChildLink)
    {
        m_collideWithParent = true;
    }

    public tntChildLink(int nDOF, bool[] dofRotational) : base(Type.ChildLink)
    {
        //- Unlike tntChildLink() this doesn't initialize m_collideWithParent

        m_mobilizer = IntPtr.Zero;
        m_showJoint = false;
		m_visualEditor = false;
		m_dofData = new tntDofData[nDOF];

        for (int i = 0; i < nDOF; ++i)
            m_dofData[i] = new tntDofData(dofRotational[i]);

        m_currentPosition = Enumerable.Repeat<float>(0f, nDOF).ToArray();
        m_currentVelocity = Enumerable.Repeat<float>(0f, nDOF).ToArray();

		m_feedback.m_appliedImpulse = 0;
		m_feedback.m_accumulatedImpulse = 0;
		m_feedback.m_broken = false;
    }

    // The tntBaseLink of the Articulation this tntLink belongs to; granted it's connected via active tntLinks
    public tntBase baseLink
    {
        get
        {
            tntLink link = this;
            while (link as tntChildLink)
            {
                tntLink ascendant = (link as tntChildLink).m_parent;
                if (ascendant && ascendant.enabled && ascendant.gameObject.activeInHierarchy)
                {
                    link = ascendant;
                }
                else
                {
                    break;
                }
            }
            return link as tntBase;
        }
    }

    public virtual bool ArePivotsMatching()
    {
        return PivotAToWorld() == PivotBToWorld();
    }

    /**
     * Computes pivot position on link B by transforming pivot on link A
     */
    public void AutoFillPivotB()
    {
        m_pivotB = MathUtils.TransformPivot(m_pivotA, m_parent.transform, transform);
        MathUtils.RoundToGrid(ref m_pivotB);
    }

    /**
     * Computes pivot position on link A by transforming pivot on link B
     */
    public void AutoFillPivotA()
    {
        m_pivotA = MathUtils.TransformPivot(m_pivotB, transform, m_parent.transform);
        MathUtils.RoundToGrid(ref m_pivotA);
    }

    /**
     * Transforms pivot on link A from A's local frame to world coordinates
     * @return pivot on link A expressed in world coordinates
     */
    public virtual Vector3 PivotAToWorld()
    {
        return MathUtils.PointToWorld(m_pivotA, m_parent.transform);
    }

    /**
     * Transforms pivot on link B from B's local frame to world coordinates
     * @return pivot on link B expressed in world coordinates
     */
    public virtual Vector3 PivotBToWorld()
    {
        return MathUtils.PointToWorld(m_pivotB, transform);
    }

    /**
     * Sets pivot on link A
     * @param pivotAWorld pivot on link A expressed in world coordinates
     */
    public void PivotAFromWorld(Vector3 pivotAWorld)
    {
        m_pivotA = MathUtils.PointFromWorld(pivotAWorld, m_parent.transform);
        MathUtils.RoundToGrid(ref m_pivotA);
    }

    /**
     * Adds a 1-degree-of-freedom motor to this link
     * @param dof degree of freedom the motor will drive
     * @param isPositionMotor indicates whether the added motor is positional or velocity-based
     * @param desiredVelocity target velocity for positional and velocity motors
     * @param desiredPosition target position/angle for positional motors
     * @param maxForce force threshold for the motor
     * @param positionLockThreshold sets the threshold for positional motor to snap to position from.
     * @param useAutomaticPositionLockThreshold makes the positionLockThreshold be calculated automatically, positionLockThreshold is ignored if this is true.
     * @remark angular values are expected to be provided in degrees
     */
    public void AddMotor(int dof, bool isPositionMotor, float desiredVelocity,
                         float desiredPosition, float maxForce,
                         float positionLockThreshold, bool useAutomaticPositionLockThreshold)
	{
        if (m_world != null)
        {
            // m_world is only set when the child link is added to the world in the engine
            if (m_dofData[dof].m_rotational)
            {
                desiredVelocity *= Mathf.Deg2Rad;
                desiredPosition *= Mathf.Deg2Rad;
                positionLockThreshold *= Mathf.Deg2Rad;
            }
            m_mobilizer = m_world.AddMotor(m_base, m_index, dof, isPositionMotor,
                                           desiredVelocity, desiredPosition, maxForce, positionLockThreshold, useAutomaticPositionLockThreshold);
        }
	}

    /**
     * Removes a 1-degree-of-freedom motor from this link
     * @param dof degree-of-freedom this motor drives; if set to -1, motors from all DoFs are removed
     */
    public void RemoveMotor(int dof)
	{
        if (dof >= m_dofData.Length)
            return;

        if (dof >= 0)
        {
            m_dofData[dof].m_useMotor = false;
            if (m_mobilizer != IntPtr.Zero)
                m_world.RemoveMotor(m_mobilizer, dof);

            return;
        }

        for (dof = 0; dof < m_dofData.Length; ++dof)
            RemoveMotor(dof);
	}

    /**
     * Set a 1-degree-of-freedom motor to be positional or velocity-based
     * @param dof degree of freedom the motor drives
     * @param positional indicates whether the added motor is positional or velocity-based, true means positional, false otherwise
     */
    public void SetMotorIsPositional(int dof, bool positional)
    {
        if (m_mobilizer != IntPtr.Zero)
        {
            m_dofData[dof].m_isPositionMotor = positional;
            TNT.apSetMotorIsPostional(m_mobilizer, dof, positional);
        }
    }

    // desiredSpeed:  In degrees/second for rotational DOF
    /**
     * Set a 1-degree-of-freedom motor desired velocity
     * @param dof degree of freedom the motor drives
     * @param desiredSpeed the desired velocity
     * @remark angular values are expected to be provided in degrees
     */
    public void SetMotorDesiredSpeed(int dof, float desiredSpeed)
	{ 
        if (m_mobilizer != IntPtr.Zero)
        {
            m_dofData[dof].m_desiredVelocity = desiredSpeed;
            if (m_dofData[dof].m_rotational)
            {
                desiredSpeed *= Mathf.Deg2Rad;
            }
            TNT.apSetMotorDesiredSpeed(m_mobilizer, dof, desiredSpeed);
        }
	}

    // desiredPosition:  In degrees for rotational DOF
    /**
     * Set a 1-degree-of-freedom positional motor desired position
     * @param dof degree of freedom the motor drives
     * @param desiredPosition the desired position
     * @remark angular values are expected to be provided in degrees
     */
    public void SetMotorDesiredPosition(int dof, float desiredPosition)
    { 
        if (m_mobilizer != IntPtr.Zero)
        {
            m_dofData[dof].m_desiredPosition = desiredPosition;
            if (m_dofData[dof].m_rotational)
            {
                desiredPosition *= Mathf.Deg2Rad;
            }
            TNT.apSetMotorDesiredPosition(m_mobilizer, dof, desiredPosition);
        }
    }

    /**
     * Disable a 1-degree-of-freedom motor for this link
     * @param dof degree of freedom the motor will drive
     */
	public void DisableMotor(int dof)
	{
		tntDofData dofData = m_dofData[dof];
		if (m_mobilizer == IntPtr.Zero || !dofData.m_useMotor)
			return;
		dofData.m_useMotor = false;
		RemoveMotor(dof);
	}

    /**
     * Enable a 1-degree-of-freedom motor for this link according to tntChildLink::m_dofData settings
     * @param dof degree of freedom the motor will drive
     */
    public void EnableMotor(int dof)
	{
		tntDofData dofData = m_dofData[dof];
		if (m_mobilizer == IntPtr.Zero || dofData.m_useMotor)
			return;
		dofData.m_useMotor = true;
		AddMotor(dof, dofData.m_isPositionMotor, dofData.m_desiredVelocity,
                dofData.m_desiredPosition, dofData.m_maxMotorForce, dofData.m_positionLockThreshold, dofData.m_useAutomaticPositionLockThreshold);
	}

    /**
  * Disable a 1-degree-of-freedom limit for this link
  * @param dof degree of freedom the limit will drive
  */
    public void DisableLimit(int dof)
    {
        tntDofData dofData = m_dofData[dof];
        if (m_mobilizer == IntPtr.Zero || !dofData.m_useLimit)
            return;
        dofData.m_useLimit = false;
        RemoveLimit(dof);
    }

    /**
     * Enable a 1-degree-of-freedom limit for this link according to tntChildLink::m_dofData settings
     * @param dof degree of freedom the limit will drive
     */
    public void EnableLimit(int dof)
    {
        tntDofData dofData = m_dofData[dof];
        if (m_mobilizer == IntPtr.Zero || dofData.m_useLimit)
            return;
        dofData.m_useLimit = true;
        AddLimits(dof, dofData.m_limitLow, dofData.m_limitHigh, dofData.m_maxLimitForce);
    }

    /**
     * Set a 1-degree-of-freedom motor force threshold
     * @param dof degree of freedom the motor drives
     * @param maxForce force threshold for the motor
     */
    public void SetMotorMaxForce(int dof, float maxForce)
	{ 
        if (m_mobilizer != IntPtr.Zero)
        {
            m_dofData[dof].m_maxMotorForce = maxForce;
            TNT.apSetMotorMaxForce(m_mobilizer, dof, maxForce);
        }
	}

    // positionLockThreshold:  In degrees for rotational DOF
    /**
     * Sets "position lock threshold" for this link's positional motor
     * @param dof degree of freedom the motor drives
     * @param positionLockThreshold position lock threshold for the motor
     * @remark use position lock threshold to achieve better stability around target position for your positional motors at the cost of constant motor speed
     * @remark angular values are expected to be provided in degrees
     */
    public void SetMotorPositionLockThreshold(int dof, float positionLockThreshold)
    {
        if (m_mobilizer != IntPtr.Zero)
        {
            m_dofData[dof].m_positionLockThreshold = positionLockThreshold;
            if (m_dofData[dof].m_rotational)
            {
                positionLockThreshold *= Mathf.Deg2Rad;
            }
            TNT.apSetMotorPositionLockThreshold(m_mobilizer, dof, positionLockThreshold);
        }
    }

    /**
     * Sets "should use automatic position lock threshold" for this link's positional motor
     * @param dof degree of freedom the motor drives
     * @param useAutomaticPositionLockThreshold toggles on/off the use of automatically calculated position lock threshold for the motor
     * @remark automatic PLT makes sure it's activated in the moment the desiredVelocity can't be used anymore. 
     */
    public void SetMotorUseAutomaticPositionLockThreshold(int dof, bool useAutomaticPositionLockThreshold)
    {
        if (m_mobilizer != IntPtr.Zero)
        {
            m_dofData[dof].m_useAutomaticPositionLockThreshold = useAutomaticPositionLockThreshold;
            TNT.apSetUseAutomaticPositionLockThreshold(m_mobilizer, dof, useAutomaticPositionLockThreshold);
        }
    }

    // limitLow, limitHigh: In degrees for rotational DOF
    /**
     * Adds a 1-degree-of-freedom joint limit to this link
     * @param dof degree of freedom the limit will restrain
     * @param limitLow lower limit on allowable position/angle range
     * @param limitHigh upper limit on allowable position/angle range
     * @param maxForce force threshold for the limit
     * @remark angular values are expected to be provided in degrees
     */
    public void AddLimits(int dof, float limitLow, float limitHigh, float maxForce)
    {
        if (m_world != null)
        {
            m_dofData[dof].m_limitLow = limitLow;
            m_dofData[dof].m_limitHigh = limitHigh;
            if (m_dofData[dof].m_rotational)
            {
                limitLow *= Mathf.Deg2Rad;
                limitHigh *= Mathf.Deg2Rad;
            }
            m_dofData[dof].m_maxLimitForce = maxForce;
            // m_world is only set when the child link is added to the world in the engine
            m_mobilizer = m_world.AddLimits(m_base, m_index, dof,
                                            limitLow, limitHigh, maxForce);
        }
    }

    /**
     * Removes a 1-degree-of-freedom joint limit from this link
     * @param dof degree-of-freedom this limit restrains; if set to -1, limits from all DoFs are removed
     */
    public void RemoveLimit(int dof)
    {
        if (dof >= m_dofData.Length)
            return;

        if (dof >= 0)
        {

            if (m_mobilizer != IntPtr.Zero)
                m_world.RemoveLimits(m_mobilizer, dof);

            return;
        }

        for (dof = 0; dof < m_dofData.Length; ++dof)
        {
            // This is the only case the caller need this function to flip the "m_useLimit"
            // flag. When caller invoke this API with >=0 dof index the caller always assumes
            // the responsibility of updating m_useLimit if necessary.
            m_dofData[dof].m_useLimit = false;
            RemoveLimit(dof);
        }               
    }

    // limitLow in degrees for rotational DOF
    /**
     * Sets a 1-degree-of-freedom joint lower mobility limit
     * @param dof degree of freedom the limit restrains
     * @param limitLow lower limit on allowable position/angle range
     * @remark angular values are expected to be provided in degrees
     */
    public void SetLimitLower(int dof, float limitLow)
    { 
        if (m_mobilizer != IntPtr.Zero)
        {
            m_dofData[dof].m_limitLow = limitLow;
            if (m_dofData[dof].m_rotational)
            {
                limitLow *= Mathf.Deg2Rad;
            }
            TNT.apSetJointLimitLo(m_mobilizer, dof, limitLow);
        }
    }
         
    // limitHigh in degrees for rotational DOF
    /**
     * Sets a 1-degree-of-freedom joint upper mobility limit
     * @param dof degree of freedom the limit restrains
     * @param limitHigh upper limit on allowable position/angle range
     * @remark angular values are expected to be provided in degrees
     */
    public void SetLimitUpper(int dof, float limitHigh)
    { 
        if (m_mobilizer != IntPtr.Zero)
        {
            m_dofData[dof].m_limitHigh = limitHigh;
            if (m_dofData[dof].m_rotational)
            {
                limitHigh *= Mathf.Deg2Rad;
            }
            TNT.apSetJointLimitHi(m_mobilizer, dof, limitHigh);
        }
    }

    /**
     * Set a 1-degree-of-freedom joint mobility limit force threshold
     * @param dof degree of freedom the limit restrains
     * @param maxForce force threshold for the limit
     */
    public void SetLimitMaxForce(int dof, float maxForce)
    { 
        if (m_mobilizer != IntPtr.Zero)
        {
            m_dofData[dof].m_maxLimitForce = maxForce;
            TNT.apSetJointLimitMaxForce(m_mobilizer, dof, maxForce);
        }
    }

    /**
     * Set a 1-degree-of-freedom spring stiffness
     * @param dof degree of freedom this spring affects
     * @param stiffness spring stiffness coefficient to be set
     */
    public void SetSpringStiffness(int dof, float stiffness)
    { 
        if (m_mobilizer != IntPtr.Zero)
        {
            m_dofData[dof].m_springStiffness = stiffness;
            TNT.apSetSpringStiffness(m_mobilizer, dof, stiffness);
        }
    }

    /**
     * Set a 1-degree-of-freedom damper coefficient
     * @param dof degree of freedom this damper affects
     * @param damping damper coefficient to be set
     */
    public void SetSpringDamping(int dof, float damping)
    { 
        if (m_mobilizer != IntPtr.Zero)
        {
            m_dofData[dof].m_springDamping = damping;
            TNT.apSetSpringDamping(m_mobilizer, dof, damping);
        }
    }

    // neutralPoint: In degrees for rotational DOF
    /**
     * Set a 1-degree-of-freedom spring neutral point (length or angle)
     * @param dof degree of freedom this spring affects
     * @param neutralPoint spring neutral point to be set
     * @remark angular values are expected to be provided in degrees
     */
    public void SetSpringNeutralPoint(int dof, float neutralPoint)
    { 
        if (m_mobilizer != IntPtr.Zero)
        {
            m_dofData[dof].m_neutralPoint = neutralPoint;
            if (m_dofData[dof].m_rotational)
            {
                neutralPoint *= Mathf.Deg2Rad;
            }
            TNT.apSetSpringNeutralPoint(m_mobilizer, dof, neutralPoint);
        }
    }

    /**
     * Set continuous force to be applied to the specified DOF
     * @param dof degree of freedom this force affects
     * @param continuousForce continuous force to be applied
     */
    public void SetContinuousForceActuator(int dof, float continuousForce)
    {
        if (m_mobilizer != IntPtr.Zero)
        {
            m_dofData[dof].m_continuousForce = continuousForce;
            TNT.apSetContinuousForceActuator(m_mobilizer, dof, continuousForce);
        }
    }

    // FIXME: setting zero mass to child links (when fixed, update doxygen comment as well)
    /**
     * Sets this link's mass
     * @param mass link mass to set
     * @remark Setting zero mass to a child link is not properly handled at this moment
     */
    public override void SetMass(float newMass)
    {
        if (newMass == 0)
        {
            Debug.LogError("tntChildLink '" + name + "' mass cannot be set to 0. Please set it to a different value.");
            return;
        }
        // previously anchored TNT rigid body is no de-anchored
        MeshCollider meshCollider = GetComponent<MeshCollider>();
        if (meshCollider != null && !meshCollider.convex)
        {
            Debug.LogError("Concave mesh cannot be unanchored now");
            return;
        }
        if (m_mobilizer != IntPtr.Zero)
            TNT.apSetLinkMass(m_mobilizer, newMass);

        m_mass = newMass;

    }

    /**
     * Detaches this link (together with a subtree rooted at it) for its articulation
     * @remark it will take at least one Update for this call to complete
     * @remark new game objects and components will be created as a result of this call
     */
    public void BreakOff()
    {
        if (m_mobilizer != IntPtr.Zero)
        {
            TNT.apBreakArticulationLinkOff(m_mobilizer);
        }
    }

	void OnDestroy()
	{
        RemoveAllCollisionListeners();

        RemoveArticulationLink();
	}
	
	void OnApplicationQuit()
	{
        RemoveAllCollisionListeners();

        RemoveArticulationLink();
	}

	public void RemoveArticulationLink()
	{
		// TBD:  Remove articulation links and motor constraints
		if (m_added && m_world)
		{
			m_world.RemoveArticulationLink(this);
			m_added = false;
		}
	}

    /**
     * Sets layer ID for this link
     * @param layerID layer ID to be set
     * @param resetCollisionCache Force deletion of existing collision caches (needed to update filtering on e.g. resting contact) and activate the object
     * @remark APE respects Unity's layer collision matrix so this operation may have impact on collision detection
     * @see http://docs.unity3d.com/Manual/LayerBasedCollision.html
     */
    public override void SetLayerID(int layerID, bool resetCollisionCache = false)
    {
		gameObject.layer = layerID;
        TNT.apSetLinkLayerID(m_base, m_index, layerID);
        if (resetCollisionCache)
        {
            TNT.apResetCollisionCacheForLink(GetWorld(), m_base, m_index);
        }
    }

    /**
     * Enables or disables collision detection for this link
     * @param collide true enables collision detection, false disables it
     */
    public override void SetCollidable(bool collide, bool resetCollisionCache = false)
    {
        if (!collide)
        {
			TNT.apSetLinkLayerID(m_base, m_index, -1);
        }
        else
        {
			TNT.apSetLinkLayerID(m_base, m_index, gameObject.layer);
        }
        m_collidable = collide;
        if (resetCollisionCache)
        {
            TNT.apResetCollisionCacheForLink(GetWorld(), m_base, m_index);
        }
    }

    internal bool m_isVelocityCacheValid;
    private Vector3 m_cachedCurrentVelocity;
    private Vector3 m_cachedCurrentAngularVelocity;

    /// Current linear velocity of this tntBaseLink
    public override Vector3 linearVelocity { get { if (!m_isVelocityCacheValid) ComputeWorldVelocity(out m_cachedCurrentVelocity, out m_cachedCurrentAngularVelocity, false); return m_cachedCurrentVelocity; } set { tntWorld.Assert(false, "tntChildLink.linearVelocity.set() is not supported"); } }

    /// Current angular velocity of this tntBaseLink
    public override Vector3 angularVelocity { get { if (!m_isVelocityCacheValid) ComputeWorldVelocity(out m_cachedCurrentVelocity, out m_cachedCurrentAngularVelocity, false); return m_cachedCurrentAngularVelocity; } set { tntWorld.Assert(false, "tntChildLink.angularVelocity.set() is not supported"); } }

    /**
     * Computes vectorial velocity of this link expressed in world coordinates.
     * @param linVel this link's linear velocity will be returned in it
     * @param angVel this link's angular velocity will be returned in it
     * @param useCache specifies if cached velocity vectors can be used which is faster but can return outdated values in certain cases
     * @remark angular values are expected to be provided in degrees
     * @remark this method uses API function which is O(n) if useCache is false (n is the number of links in the articulation)
     */
    public void ComputeWorldVelocity(out Vector3 linVel, out Vector3 angVel, bool useCache)
    {
        linVel = new Vector3();
        angVel = new Vector3();
        if (m_mobilizer == IntPtr.Zero)
            return;

        unsafe
        {
            fixed (Vector3 *pLinVel = &linVel)
            fixed (Vector3 *pAngVel = &angVel)
            {
                TNT.apComputeLinkWorldVelocity(m_mobilizer, pLinVel, pAngVel, useCache);
            }
        }
    }

    /**
     * Returns the number of this link's degrees of freedom
     * @return number of this link's degrees of freedom
     */
    public int GetDofCount()
    {
        if (m_mobilizer == IntPtr.Zero)
            return 0;

        return TNT.apGetLinkDofCount(m_mobilizer);
    }

    /**
     * Returns the number of entries in this link's RAW position vector
     * @return number of entries in this link's RAW position vector
     * @remark RAW position uses internal APE's format which can use different units or different number of parameters compared to Unity
     */
    public int GetRawPosVariablesCount()
    {
        if (m_mobilizer == IntPtr.Zero)
            return 0;

        return TNT.apGetLinkRawPosVarCount(m_mobilizer);
    }

    /**
     * Sets scalar RAW velocity variable to one of this link's degrees of freedom
     * @param dofIndex identifies the degree of freedom
     * @param vel RAW scalar velocity variable to be set
     * @remark this way of setting link velocity is useful mostly for initial state; using it during simulation may lead to issues
     * @remark RAW velocity uses internal APE's format which can use different units or different number of parameters compared to Unity
     */
    public void SetRawVelocityVariable(int dofIndex, float vel, bool updateArticulationTransformsAndSensors)
    {
        if (m_mobilizer != IntPtr.Zero)
        {
            TNT.apSetLinkRawVelocityVariable(m_mobilizer, dofIndex, vel, updateArticulationTransformsAndSensors);
        }
    }

    /**
     * Gets scalar RAW velocity variable of one of this link's degrees of freedom
     * @param dofIndex identifies the degree of freedom
     * @return RAW scalar velocity variable
     * @remark RAW velocity uses internal APE's format which can use different units or different number of parameters compared to Unity
     */
    public float GetRawVelocityVariable(int dofIndex)
    {
        if (m_mobilizer != IntPtr.Zero)
        {
            return TNT.apGetLinkRawVelocityVariable(m_mobilizer, dofIndex);
        }

        return 0f;
    }

    /**
     * Sets scalar RAW position variable to one of this link's position vector entries
     * @param posVarIndex identifies the position vector entry
     * @param posVar RAW scalar position variable to be set
     * @remark this way of setting link position is useful mostly for initial state; using it during simulation may lead to issues
     * @remark RAW position uses internal APE's format which can use different units or different number of parameters compared to Unity
     */
    public void SetRawPositionVariable(int posVarIndex, float posVar, bool updateArticulationTransformsAndSensors)
    {
        if (m_mobilizer != IntPtr.Zero)
        {
            TNT.apSetLinkRawPosVariable(m_mobilizer, posVarIndex, posVar, updateArticulationTransformsAndSensors);
        }
    }

    /**
     * Gets scalar RAW position variable to one of this link's position vector entries
     * @param posVarIndex identifies the position vector entry
     * @return RAW scalar position variable
     * @remark RAW position uses internal APE's format which can use different units or different number of parameters compared to Unity
     */
    public float GetRawPositionVariable(int posVarIndex)
    {
        if (m_mobilizer != IntPtr.Zero)
        {
            return TNT.apGetLinkRawPosVariable(m_mobilizer, posVarIndex);
        }

        return 0f;
    }

    // Note: RetrieveAxes, RetrievePivots and GetKernelJointType have all been extracted
    // from tntBase.AddArticulationBase with minimal modification
    // Polymorphic (per child link type) versions of those would be much more convenient but let's stick ..
    // to their original form for now in order to this to avoid introducing too many changes at once

    // TODO: polymorphic RetrieveAxes
    // code extracted from tntBase.AddArticulation
    public void RetrieveAxes(out Vector3 axisA, out Vector3 axisB)
    {
        axisA = Vector3.zero;
        axisB = Vector3.zero;
        //
        tntHingeLink hinge = this as tntHingeLink;
        tntUniversalLink universal = this as tntUniversalLink;
        tntSliderLink slider = this as tntSliderLink;
        tntPlaneLink plane = this as tntPlaneLink;
        tntBallLink ball = this as tntBallLink;
        //
        if (hinge != null)
        {
            axisA = hinge.m_axisA;
        }
        else if (universal != null)
        {
            axisA = universal.m_axisA;
            axisB = universal.m_axisB;

            Vector3 axisAInChild = Vector3.Normalize(Quaternion.Inverse(this.transform.rotation) * this.m_parent.transform.rotation * universal.m_axisA);
            Vector3 axisBInChild = Vector3.Normalize(universal.m_axisB);
            float dotValueAbs = Mathf.Abs(Vector3.Dot(axisAInChild, axisBInChild));

            // Orthogonalize axes if needed
            if (dotValueAbs > 0f)
            {
                if (dotValueAbs >= 1e-1)
                    Debug.LogWarningFormat("Your universal link's ({0}) axes are far from orthogonal.They will get auto-orthogonalized but you should probably check your setup.", this.name);

                Vector3.OrthoNormalize(ref axisAInChild, ref axisBInChild);

                axisA = Quaternion.Inverse(this.m_parent.transform.rotation) * this.transform.rotation * axisAInChild;
                axisB = axisBInChild;

                universal.m_axisA = axisA;
                universal.m_axisB = axisB;
            }
        }
        else if (slider != null)
        {
            axisA = slider.m_axisA;
        }
        else if (ball != null)
        { }
        else if (plane != null)
        {
            axisA = plane.m_axisA;
        }
    }

    // TODO: polymorphic RetrievePivots
    // code extracted from tntBase.AddArticulation
    public void RetrievePivots(out Vector3 pivotA, out Vector3 pivotB)
    {
        pivotA = Vector3.zero;
        pivotB = Vector3.zero;
        //
        tntHingeLink hinge = this as tntHingeLink;
        tntUniversalLink universal = this as tntUniversalLink;
        tntSliderLink slider = this as tntSliderLink;
        tntPlaneLink plane = this as tntPlaneLink;
        tntBallLink ball = this as tntBallLink;
        //
        if (hinge != null)
        {
            pivotA = hinge.m_pivotA;
            pivotB = hinge.m_pivotB;
        }
        else if (universal != null)
        {
            pivotA = universal.m_pivotA;
            pivotB = universal.m_pivotB;
        }
        else if (slider != null)
        { }
        else if (ball != null)
        {
            pivotA = ball.m_pivotA;
            pivotB = ball.m_pivotB;
        }
        else if (plane != null)
        { }
    }

    // TODO: polymorphic GetKernelJointType
    // code extracted from tntBase.AddArticulation
    public int GetKernelJointType()
    {
        int jointType = -1;

        tntHingeLink hinge = this as tntHingeLink;
        tntUniversalLink universal = this as tntUniversalLink;
        tntSliderLink slider = this as tntSliderLink;
        tntPlaneLink plane = this as tntPlaneLink;
        tntBallLink ball = this as tntBallLink;
        tntFixedLink fixedLink = this as tntFixedLink;

        if (hinge != null)
        {
            jointType = 0;
		}
        else if (universal != null)
		{
			jointType = 5;
		}
		else if (slider != null)
        {
            jointType = 1;
        }
        else if (ball != null)
        {
            jointType = 2;
        }
        else if (plane != null)
        {
            jointType = 3;
        }
        else if (fixedLink != null)
        {
            jointType = 4;
        }

        return jointType;
    }

    public override void SetBodyShapeToKernel(IntPtr shape)
    {
        if (m_mobilizer != IntPtr.Zero)
            TNT.apSetLinkCollisionShape(m_mobilizer, shape);
    }

    public override void ComputeMoIFromCollisionShape()
    {
        if (m_mobilizer != IntPtr.Zero)
        {
            Component mainCollider;
            bool isCompound;
            Component[] childColliders;

            float[] masses = null;
            float[] inertias = null;

            if (ColliderFactory.ExtractColliderDefinitions(this, out mainCollider, out isCompound, out childColliders))
            {
                if (isCompound)
                {
                    if (!ColliderFactory.ComputeChildColliderMasses(childColliders, out masses, out inertias))
                    {
                        masses = null;
                        inertias = null;
                    }
                }
                else
                {
                    masses = new float[1] { m_mass };
                    inertias = new float[3] { 0, 0, 0 };
                }
            }

            if (masses == null || inertias == null)
            {
                // TODO: log sth
                return;
            }

            Vector3 pivotA, pivotB, axisA, axisB;
            RetrievePivots(out pivotA, out pivotB);
            RetrieveAxes(out axisA, out axisB);

            unsafe
            {
                fixed (float* pMasses = masses)
                fixed (float* pInertias = inertias)
                fixed (float* pMass = &m_mass)
                fixed (Vector3* pMoI = &m_moi)
                {
                    TNT.apComputeLinkMoIFromCollisionShape(
                        m_mobilizer, pMasses, pInertias,
                        &pivotA, &axisA, &pivotB, &axisB,
                        pMass, pMoI);
                }
            }
        }
    }

    public override void AssignValidChildLinkIndices(bool forceUpdate = false)
    {
        if (m_parent && this != m_parent)
            m_parent.AssignValidChildLinkIndices(forceUpdate);
        else
            InvalidateIndex();
    }
}
