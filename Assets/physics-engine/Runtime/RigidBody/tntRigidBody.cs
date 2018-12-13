using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using PhysicsAPI;

/**
 * Class representing an unconstrained rigid body
 */
[DisallowMultipleComponent]
public class tntRigidBody : tntEntity
{
        /// Simple constructor to only specify the type
    tntRigidBody() : base(Type.RigidBody)
    {
        // Initialize velocity sensor array
        m_currentVelocity = new float[] { 0f, 0f, 0f, 0f, 0f, 0f };
    }


    public Vector3 m_initialLinearVelocity = Vector3.zero; // initial velocity will be set right after the rigidbody is created
    public Vector3 m_initialAngularVelocity = Vector3.zero; // initial velocity will be set right after the rigidbody is created

    private static List<tntRigidBody> sBodies = new List<tntRigidBody> ();
    private static int sLastAddedToWorld = 0;

    private static LinkedList<int> sChangedList = new LinkedList<int> ();
    private static LinkedList<Action<tntWorld>> sDeferedCommands = new LinkedList<Action<tntWorld>> ();

    [Flags]
    internal enum Flag {
        Enable = 1,
        Transform = 2,
    };

    [NonSerialized] internal IntPtr m_rigidBody = IntPtr.Zero;

    private IntPtr worldInKernel { get { return m_world ? m_world.worldInKernel : IntPtr.Zero; } }

    [NonSerialized] internal List<tntRigidBodyConstraint> m_joints = new List<tntRigidBodyConstraint>();

    [SerializeField]
    public bool m_IsKinematic = false;
    public bool kinematic {
        get { return m_IsKinematic; }
        set { SetKinematic (value); }
    }
    
    /**
     * Checks if this rigid body is static (i.e. immobile)
     * @return true if this body is static, false otherwise
     */
    public bool IsStatic() { return m_mass == 0;  }

    // Public Properties

    public bool m_collidable = true;
	
    // for applying drag

    [NonSerialized] internal bool m_isSetLocalScale = false;
    [NonSerialized] internal Flag m_flag = 0;

    internal bool Added {
        get { return m_added; }
    }

    public bool HasListeners {
        get { return hasCollisionListeners; }
    }

    public Vector3 position {
        get { return m_transform.position; }
        set {
            m_transform.position = value;
            m_transform.hasChanged = false;
            AddToChangeList (this, Flag.Transform);
        }
    }

    public Quaternion rotation {
        get { return m_transform.rotation; }
        set {
            m_transform.rotation = value;
            m_transform.hasChanged = false;
            AddToChangeList (this, Flag.Transform);
        }
    }
    
    public override Vector3 linearVelocity
    {
        get { return new Vector3(CurrentVelocity[0], CurrentVelocity[1], CurrentVelocity[2]); }
        set
        {
            for (int i = 0; i < 3; i++) CurrentVelocity[i] = value[i];
            // This can also be run from inspector before play
            if (m_rigidBody != IntPtr.Zero)
            {
                float[] vel = CurrentVelocity;
                TNT.apSetRigidBodyVelocity(m_rigidBody, vel[0], vel[1], vel[2], vel[3], vel[4], vel[5]);
                TNT.apActivateKinematicRigidBody(m_rigidBody);
            }
        }
    }

    public override Vector3 angularVelocity
    {
        get { return new Vector3(CurrentVelocity[3], CurrentVelocity[4], CurrentVelocity[5]) * Mathf.Rad2Deg; }
        set
        {
            for (int i = 0; i < 3; i++) CurrentVelocity[3 + i] = value[i] * Mathf.Deg2Rad;
            // This can also be run from inspector before play
            if (m_rigidBody != IntPtr.Zero)
            {
                float[] vel = CurrentVelocity;
                TNT.apSetRigidBodyVelocity(m_rigidBody, vel[0], vel[1], vel[2], vel[3], vel[4], vel[5]);
                TNT.apActivateKinematicRigidBody(m_rigidBody);
            }
        }
    }

    /**
     * Returns the internal representation of this rigid body
     * @return native pointer to the internal representation of this rigid body
     */
    public IntPtr GetRigidBody() { return m_rigidBody; }

	internal void AddJoint(tntRigidBodyConstraint joint) { m_joints.Add(joint); }
	internal void RemoveJoint(tntRigidBodyConstraint joint) { m_joints.Remove (joint); }

    static internal void AddToChangeList (tntRigidBody body, Flag flag) {
        if (body.m_flag == 0) {
            // if not been added already
            sChangedList.AddLast (body.m_worldIndex);
        }

        body.m_flag |= flag;
    }

    static internal tntRigidBody ByIndex (int index) {
        return sBodies [index];
    }

    static internal void UpdateToWorld (tntWorld world) {
        for (; sLastAddedToWorld < sBodies.Count; ++sLastAddedToWorld) {
            var body = sBodies [sLastAddedToWorld];
            if (body == null) {
                continue;
            }

            body.AddIfActive (world);
        }

        var changed = sChangedList.GetEnumerator ();
        while (changed.MoveNext ()) {
            var body = sBodies [changed.Current];
            if (body == null) {
                continue;
            }

            sBodies [changed.Current].SyncToKernel (world);
        }

        sChangedList.Clear ();

        var command = sDeferedCommands.GetEnumerator ();
        while (command.MoveNext ()) {
            command.Current (world);
        }
        sDeferedCommands.Clear ();
    }

    static internal void WorldIsRemoved (tntWorld world) {
        int nBodies = sBodies.Count;
        for (int i = 0; i < nBodies; ++i) {
            world.RemoveRigidBody(sBodies[i]);
        }
    }

    static internal void RemoveLast (int range) {
        if (range > 0) {
            sBodies.RemoveRange (sBodies.Count - range, range);
            sLastAddedToWorld -= range;
        }
                }

    // go thru the list of rigid bodies and update its transforms
    // return the number of null'd bodies
    // NOTE: the empty holes are already filled up, need to call
    // RemoveLast so there won't be duplicates
    static internal int FetchTransform (tntWorld world) {
        int nBodies = sBodies.Count;
        for (int i = 0; i < nBodies; ++i) {
            var body = sBodies [i];
            if (body == null) {
                // nBodies point to the last non-nil body
                while (i < nBodies && sBodies[--nBodies] == null)
                    ;

                if (i >= nBodies) 
                break;

                sBodies[i] = sBodies[nBodies];
                sBodies[i].m_worldIndex = i;
                body = sBodies [i];
            }

            if (!body.m_added) {
                continue;
            }

            if (body.m_position != body.m_transform.position
                || body.m_rotation != body.m_transform.rotation) {
                if (body.GetKinematic ()) {
                    body.m_position = body.m_transform.position;
                    body.m_rotation = body.m_transform.rotation;
                } else if (!body.IsStatic()) {
                    body.m_transform.position = body.m_position;
                    body.m_transform.rotation = body.m_rotation;
                }
            }
        }

        return sBodies.Count - nBodies;
    }
	
	/**
    * Returns this links collision shape
    * @return native pointer to this link's collision shape
    */
    public IntPtr GetBodyShape() { return m_bodyShape; }
    public tntShapeType GetShapeType() { return m_shapeType; }

    public void SetShapeType(tntShapeType shapeType) { m_shapeType = shapeType; }
    public void SetBodyShape(IntPtr shape) { m_bodyShape = shape; }

    internal void SyncToKernel (tntWorld world) {
        // this can happen when the component is disabled in the beginning
        // but it's still batched to update its position
        if (m_rigidBody == IntPtr.Zero) {
            m_flag = 0;
                return;
        }

        if ((m_flag & Flag.Transform) > 0) {
            var position = m_transform.position;
            var rotation = m_transform.rotation;
            TNT.apSetRigidBodyPosition (
                m_rigidBody, position.x, position.y, position.z,
                rotation.x, rotation.y, rotation.z, rotation.w);
        }

        m_flag = 0;
	}

	public void AddIfActive(tntWorld world)
    {
        m_world = world;

        if (enabled && gameObject.activeInHierarchy)
        {
            world.AddRigidBody(this);
        }
    }

    public void SetInitialVelocities()
    {
        // init velocities
        linearVelocity = m_initialLinearVelocity;
        angularVelocity = m_initialAngularVelocity;
    }

    void Awake()
    {
        m_transform = transform;

        if (!ContainedByCompoundShape())
        {
            m_worldIndex = sBodies.Count;
            sBodies.Add(this);
        }
    }

	public void AddToKernel()
	{
		// TODO: add to the change list
		if (m_world)
            m_world.AddRigidBody (this);
	}

    void OnEnable()
    {
		AddToKernel ();
    }

	public void RemoveFromKernel()
	{
		if (m_world)
			m_world.RemoveRigidBody(this);
	}

    void OnDisable()
	{
		RemoveFromKernel();
	}

	void OnDestroy()
    {
        tntEntityAndJointFactory.DestroyRigidBody(this);
    }

	public bool ContainedByCompoundShape()
			{
        Transform theTransform = m_transform ? m_transform : transform;
        return theTransform.parent != null
            && theTransform.parent.GetComponentInParent<tntCompoundCollider> () != null;
            }

    /**
     * Sets rigid body mass
     * @param mass the mass to set
     * @remark Setting zero mass makes the body immobile
     * @remark The actual application is deferred until the next simulation step
     */
    public override void SetMass(float newMass)
    {
#if I_WANT_PERFORMANCE_PENALTY
        if (newMass != 0)
        {
            // previously anchored TNT rigid body is now de-anchored
            MeshCollider meshCollider = GetComponent<MeshCollider>();
            if (meshCollider != null && !meshCollider.convex)
            {
                Debug.LogError("Concave mesh cannot be unanhcored now");
                return;
            }
        }
#endif
        if (m_added && worldInKernel != IntPtr.Zero) {
            sDeferedCommands.AddLast (
                (world) => TNT.apSetRigidBodyMass(world.GetDynamicWorld(), m_rigidBody, newMass));
		}
			m_mass = newMass;
		}

	// The collision layer of tntRigidBody has to be changed via this API
	// instead of via directly modifying gameObject.layer
    /**
     * Sets layer ID for this link
     * @param layerID layer ID to be set
     * @remark APE respects Unity's layer collision matrix so this operation may have impact on collision detection
     * @remark Collision layer of tntRigidBody has to be changed via this API instead of via directly modifying gameObject.layer
     * @see http://docs.unity3d.com/Manual/LayerBasedCollision.html
     */
    public override void SetLayerID(int layerID, bool resetCollisionCache = false)
    {
		gameObject.layer = layerID;
        if (m_rigidBody == IntPtr.Zero) {
			return;
		}

        if (m_added) {
            sDeferedCommands.AddLast (
                (world) => TNT.apSetRigidBodyLayerID(m_rigidBody, layerID));
            if (resetCollisionCache && m_collidable)
            {
                sDeferedCommands.AddLast(
                    (world) => TNT.apResetCollisionCacheForRigidBody(worldInKernel, m_rigidBody));
            }
        }
        else {
            TNT.apSetRigidBodyLayerID (m_rigidBody, layerID);
            if (resetCollisionCache && m_collidable)
            {
                TNT.apResetCollisionCacheForRigidBody(worldInKernel, m_rigidBody);
            }
        }
    }

	// Semantic:  collidable == TRUE - subject to its current collision layer stored in gameObject
	//            collidable == FALSE - absolutely not collidable with anything
    /**
     * Enables or disables collision detection for this rigid body
     * @param collide true enables collision detection, false disables it
     */
    public override void SetCollidable(bool collide, bool resetCollisionCache = false)
    {
        if (m_rigidBody != IntPtr.Zero) {
            TNT.apSetRigidBodyLayerID(m_rigidBody, collide ? gameObject.layer : -1);
        }
        m_collidable = collide;
        if (m_rigidBody != IntPtr.Zero)
        {
            if (resetCollisionCache)
            {
                TNT.apResetCollisionCacheForRigidBody(worldInKernel, m_rigidBody);
            }
        }
    }

    /**
     * Applies local scale stored in this tntRidigBody to the internal representation of this rigid body
     */
    public void SetLocalScale()
	{ 
        m_isSetLocalScale = true;
        if (m_bodyShape != IntPtr.Zero) {
            TNT.apSetScaling(m_bodyShape,
                m_transform.localScale.x,
                m_transform.localScale.y,
                m_transform.localScale.z);
        }
    }

    /**
     * Applies local scale to the internal representation of this rigid body.
     * The scale stored in this tntRidigBody is multiplied by the passed in factors.
     * @param x local scale factor along body-local X-axis
     * @param y local scale factor along body-local Y-axis
     * @param z local scale factor along body-local Z-axis
     */
    public void SetLocalScale(float x, float y, float z)
    {
        m_isSetLocalScale = true;
        if (m_bodyShape != IntPtr.Zero) {
        TNT.apSetScaling(m_bodyShape,
                        m_transform.localScale.x * x,
                        m_transform.localScale.y * y,
                        m_transform.localScale.z * z);
    }
    }

    /**
     * Sets this rigid body to be kinematic (i.e. mobile but not affected by physics sim) or not
     * @param kinematic specifies if the body is to become kinematic or not, true to make it kinematic, false otherwise
     * @remark This operation is ignored if it does not change internal settings
     * @see tntRigidBody::ForceSetKinematic
     */
    public void SetKinematic(bool kinematic)
    {
        if (m_IsKinematic != kinematic)
        {
            m_IsKinematic = kinematic;
            ForceSetKinematic();
        }
    }

    /**
     * Sets this rigid body to be kinematic (i.e. mobile but not affected by physics sim)
     * @see tntRigidBody::SetKinematic
     */
    public void ForceSetKinematic() {
        ForceSetKinematic (null);
            }

    internal void ForceSetKinematic(tntWorld to)
            {
        if (m_rigidBody == IntPtr.Zero)
            return;
        // FIXME: shouldn't this be rigid body state?

        // if already added, push to add to the world
        if (m_added) {
            sDeferedCommands.AddLast (
                (world) => TNT.apSetRigidBodyKinematic (
                    world.GetDynamicWorld (), m_rigidBody, m_IsKinematic, m_IsKinematic ? 0 : m_mass));
        } else if (to && to.worldInKernel != IntPtr.Zero) {
            // try to set in the given world if not added
            TNT.apSetRigidBodyKinematic (
                to.worldInKernel, m_rigidBody, m_IsKinematic, m_IsKinematic ? 0 : m_mass);
        }
    }

    /**
     * Checks if this rigid body is kinematic (i.e. mobile but not affected by physics sim)
     * @return true if this body is kinematic, false otherwise
     */
    public bool GetKinematic()
    {
        return m_IsKinematic;
    }

    /**
     * Obsolete
     */
    [Obsolete ("Use position/rotation property")]
    public void SetKinematicFromTransfrom()
    {
        if (m_IsKinematic)
        {
            m_transform.position = transform.position;
            m_transform.rotation = transform.rotation;
        }
    }

    /**
     * Obsolete
     */
    [Obsolete ("Use position/rotation property")]
    public void SetKinematicPosRot(Vector3 pos, Quaternion qt)
    {
        if (m_IsKinematic)
        {
            m_transform.position = pos;
            m_transform.rotation = qt;
        }
    }

    /**
     * Updates sync trigger setting for this tntRigidBody from Unity Collider/tntCollider assigned to it.
     * Trigger setting is also forwarded to internal representation of this rigid body.
     */
    public void SyncTriggerFlag()
    {
        if (m_colliderCached != null)
        {
            if (m_isTriggerShadowed != m_colliderCached.isTrigger)
            {
                m_isTriggerShadowed = m_colliderCached.isTrigger;
                TNT.apSetRigidBodyTriggerStatus(GetRigidBody(), m_colliderCached.isTrigger);
            }
        }
        else if (m_tntColliderCached != null)
        {
            if (m_isTriggerShadowed != m_tntColliderCached.IsTrigger)
            {
                m_isTriggerShadowed = m_tntColliderCached.IsTrigger;
                TNT.apSetRigidBodyTriggerStatus(GetRigidBody(), m_tntColliderCached.IsTrigger);
            }
        }
    }

    // Add force functionalities
    /* Force Modes:
     * Force            Add a continuous force to the rigidbody, using its mass.
     * Acceleration     Add a continuous acceleration to the rigidbody, ignoring its mass. NOT FULLY IMPLEMENTED!
     * Impulse           Add an instant force impulse to the rigidbody, using its mass.
     * VelocityChange    Add an instant velocity change to the rigidbody, ignoring its mass. NOT FULLY IMPLEMENTED!
     */

    // explosionPosition is in world space
    /* upwardsModifier : 
     * By default, the direction of the force is the line going from the explosion centre to the rigidbody's centre of mass. 
     * If you pass a non-zero value for the upwardsModifier parameter, the direction will be modified by subtracting that value 
     * from the Y component of the centre point. For example, if you pass a value of 2.0 for upwardsModifier, 
     * the explosion will appear to be centred 2.0 units below its actual position for purposes of calculating the force direction 
     * (ie, the centre and the radius of effect are not modified). 
     * Using this parameter, you can easily make the explosion appear to throw objects up into the air, 
     * which often gives a more dramatic effect than a simple outward force.
     */
    public void AddExplosionForce(float explosionForce, Vector3 explosionPosition, float explosionRadius, 
                                  float upwardsModifier = 0.0f, ForceMode mode = ForceMode.Force)
    {
        Vector3 forceDir = m_transform.position - explosionPosition;
        float dist = forceDir.magnitude;
        float force = explosionRadius == 0.0f ? explosionForce : dist < explosionRadius ? (1.0f - dist/explosionRadius) * explosionForce : 0;
        if (upwardsModifier != 0.0f)
        {
            Vector3 up = new Vector3(0,1,0);
            forceDir = m_transform.position + upwardsModifier*up - explosionPosition;
        }
        forceDir.Normalize();
        AddForce(force*forceDir, mode);
    }

    private void AddOneTimeForceToRigidBody(IntPtr rigidBody, Vector3 force, Vector3 posWorld, bool ignoreWorldPos)
    {
        unsafe
        {
            TNT.apAddOneTimeForceToRigidBody(rigidBody, &force, &posWorld, ignoreWorldPos);
        }
    }

    // force is in world space
    /**
     * Applies force to this rigid body at its center of mass
     * @param force vector expressed in world coordinates
     * @param mode indicates how the force vector should be interpreted
     * @remark The actual application is deferred until the next simulation step
     * @see http://docs.unity3d.com/ScriptReference/ForceMode.html
     */
    public void AddForce(Vector3 force, ForceMode mode = ForceMode.Force)
    {
        switch(mode)
        {
        case ForceMode.Force:
            sDeferedCommands.AddLast ((world) => m_accumulatedForce += force);
            break;
        case ForceMode.Impulse:
            sDeferedCommands.AddLast (
                (world) => AddOneTimeForceToRigidBody(m_rigidBody, force * world.m_numSteps, m_position, true));
            break;
        case ForceMode.Acceleration:
            force = force * m_mass;
            sDeferedCommands.AddLast ((world) => m_accumulatedForce += force);
            break;
        case ForceMode.VelocityChange:
            force = force * m_mass;
            sDeferedCommands.AddLast (
                (world) => AddOneTimeForceToRigidBody(m_rigidBody, force, m_position, true));
            break;
        }
    }

    // force is in world space, position is in world space
    /**
    * Applies force to this rigid body at the desired location
    * @param force vector expressed in world coordinates
    * @param position vector expressed in world coordinates
    * @param mode indicates how the force vector should be interpreted
    * @remark The actual application is deferred until the next simulation step
    * @see http://docs.unity3d.com/ScriptReference/ForceMode.html
    */
    public void AddForceAtPosition(Vector3 force, Vector3 position, ForceMode mode = ForceMode.Force)
    {
        switch(mode)
        {
        case ForceMode.Force:
            {
                m_accumulatedForce += force;
                Vector3 relPos = position - m_position;
            sDeferedCommands.AddLast ((world) => m_accumulatedTorque += Vector3.Cross(relPos, force));
            }
            break;
        case ForceMode.Impulse:
            sDeferedCommands.AddLast (
                (world) => AddOneTimeForceToRigidBody(m_rigidBody, force * world.m_numSteps, position, false));
            break;
        case ForceMode.Acceleration:
            Debug.LogError("Adding acceleration at position is not supported.");
            break;
        case ForceMode.VelocityChange:
            Debug.LogError("Adding velocity change at position is not supported.");
            break;
        }
    }

    // force is in local space
    /**
     * Applies force to this rigid body at its center of mass
     * @param force vector expressed in body-local coordinates
     * @param mode indicates how the force vector should be interpreted
     * @remark The actual application is deferred until the next simulation step
     * @see http://docs.unity3d.com/ScriptReference/ForceMode.html
     */
    public void AddRelativeForce(Vector3 force, ForceMode mode = ForceMode.Force)
    {
        Vector3 forceWorldSpace = m_transform.TransformVector(force);
        AddForce(forceWorldSpace, mode);
    }

    // torque is in local space
    /**
     * Applies torque to this rigid body
     * @param torque vector expressed in body-local coordinates
     * @param mode indicates how the torque vector should be interpreted
     * @remark The actual application is deferred until the next simulation step
     * @see http://docs.unity3d.com/ScriptReference/ForceMode.html
     */
    public void AddRelativeTorque(Vector3 torque, ForceMode mode = ForceMode.Force)
    {
        Vector3 torqueWorldSpace = m_transform.TransformVector(torque);
        AddTorque(torqueWorldSpace, mode);
    }

    private void AddOneTimeTorqueToRigidBody(IntPtr rigidBody, Vector3 torque)
    {
        unsafe
        {
            TNT.apAddOneTimeTorqueToRigidBody(rigidBody, &torque);
        }
    }

    // torque is in world space
    /**
     * Applies torque to this rigid body
     * @param torque vector expressed in world coordinates
     * @param mode indicates how the torque vector should be interpreted
     * @remark The actual application is deferred until the next simulation step
     * @see http://docs.unity3d.com/ScriptReference/ForceMode.html
     */
    public void AddTorque(Vector3 torque, ForceMode mode = ForceMode.Force)
    {
        switch(mode)
        {
            case ForceMode.Force:
            sDeferedCommands.AddLast ((world) => m_accumulatedTorque += torque);
                break;
            case ForceMode.Impulse:
            sDeferedCommands.AddLast (
                (world) => AddOneTimeTorqueToRigidBody(m_rigidBody, torque * world.m_numSteps));
                break;
            case ForceMode.Acceleration:
                Debug.LogError("Adding angurlar acceleration is not supported.");
                break;
            case ForceMode.VelocityChange:
                Debug.LogError("Adding angurlar velocity change is not supported.");
                break;
        }
    }

    public virtual void SetBodyShapeToKernel(IntPtr shape)
    {
        if (m_rigidBody != IntPtr.Zero)
            TNT.apSetRigidBodyCollisionShape(m_rigidBody, shape);
    }

    public virtual void ComputeMoIFromCollisionShape()
    {
        if (m_rigidBody != IntPtr.Zero)
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

            unsafe
            {
                fixed (float* pMasses = masses)
                fixed (float* pInertias = inertias)
                fixed (float* pMass = &m_mass)
                fixed (Vector3* pMoI = &m_moi)
                {
                    TNT.apComputeRigidBodyMoIFromCollisionShape(
                        m_rigidBody, pMasses, pInertias,
                        pMass, pMoI);
                }
            }
        }
    }
}
