using UnityEngine;
using System;
using PhysicsAPI;

/**
 * @brief Abstract class representing a single body/link in an articulation
 */
public abstract class tntLink : tntEntity
{
    protected struct LinkTPose
    {
        public Vector3 m_Pos;
        public Quaternion m_Rot;
        public bool m_WasSet;                   // defaults to false

        public void Reset()
        {
            m_Pos.Set(0f, 0f, 0f);
            m_Rot.Set(0f, 0f, 0f, 1f);
            m_WasSet = false;
        }
    }

    internal IntPtr m_base = IntPtr.Zero;
    [SerializeField]
    internal int m_index;		// -2 stands for unassigned index

    public tntLink m_mirroredLink = null;

    internal float m_friction;
    internal float m_rollingFriction;
    internal float m_restitution;

    internal PhysicMaterialCombine m_frictionCombineMode;
    internal PhysicMaterialCombine m_restitutionCombineMode;

    // Public Properties
    public byte[] m_asciiName;
    public bool m_collidable;
    public bool m_computeMoiFromColliders = false;
    public string m_mark;

    // transformation info
    protected LinkTPose m_TPose;

    /**
     * Checks if this link is assigned to any articulation
     * @return true if this links is a part of an articulation, false otherwise
     */
    public bool IsAssigned() { return m_index > -2; }
    /**
     * Returns index of this link which uniquely identifies it within an articulation
     * @return link index, -1 indicates a base link, 0 or greater indicates non-base links
     */
    public int GetIndex() { return m_index; }
    /**
     * Returns the base link in this link's articulation
     * @return a native pointer to the base link
     */
    public IntPtr GetBase() { return m_base; }
    /**
     * Return the dynamic world this link is a part of
     * @return a native pointer to the dynamic world
     */
    public IntPtr GetWorld() { return m_world.GetDynamicWorld(); }

    public void SetIndex(int index) { m_index = index; }
    public void InvalidateIndex() { m_index = -2;  }    
    public void SetBase(IntPtr root) { m_base = root; }
    public void SetWorld(tntWorld world) { m_world = world; }

    public abstract void AssignValidChildLinkIndices(bool forceUpdate = false);

    /**
     * Sets index identifying this link within tntWorld
     * @param idx this link's index to be set
     * @remark It has nothing to do with identification within dynamic world or an articulation
     * @see tntWorld::AddArticulationLink
     */
    public void SetWorldIndex(int idx) { m_worldIndex = idx; }

    /**
     * Returns index identifying this link within tntWorld
     * @return this link's index
     * @remark It has nothing to do with identification within dynamic world or an articulation
     * @see tntWorld::AddArticulationLink
     */
    public int GetWorldIndex() { return m_worldIndex; }

    /**
     * Returns native pointer to the index identifying this link within tntWorld
     * @return native pointer to this link's index
     * @remark It has nothing to do with identification within dynamic world or an articulation
     * @see tntLink::GetWorldInde
     * @see tntWorld::AddArticulationLink
     */
    public IntPtr GetWorldIndexRef() 
    {
        unsafe
        {
            fixed(int* pWorldIndex = &m_worldIndex)
            {
                return (IntPtr)pWorldIndex;
            }
        }
    }
    
    /**
     * Returns this links collision shape
     * @return native pointer to this link's collision shape
     */
    public IntPtr GetBodyShape() { return m_bodyShape; }
    public tntShapeType GetShapeType() { return m_shapeType; }
 
    public void SetShapeType(tntShapeType shapeType) { m_shapeType = shapeType; }
    public void SetBodyShape(IntPtr shape) { m_bodyShape = shape; }

    /**
     * Return friction coefficient for this link's material
     * @return friction coefficient
     */
    public float GetFriction() { return m_friction; }
    /**
     * Return rolling friction coefficient for this link's material
     * @return rolling friction coefficient
     */
    public float GetRollingFriction() { return m_rollingFriction; }
    /**
     * Return friction combine mode for this link's material
     * @return friction combine mode
     */
    public PhysicMaterialCombine GetFrictionCombineMode() { return m_frictionCombineMode; }
    /**
     * Return restitution coefficient for this link's material
     * @return restitution coefficient
     */
    public float GetRestitution() { return m_restitution; }
    /**
     * Return restitution combine mode for this link's material
     * @return restitution combine mode
     */
    public PhysicMaterialCombine GetRestitutionCombineMode() { return m_restitutionCombineMode; }


    /**
     * Link constructor, initializes members to their defaults
     */
    public tntLink(Type type) : base(type)
    {
        m_base = IntPtr.Zero;       
        m_index = -2;               // link index is unassigned
        m_worldIndex = -1; 			// not added to Unity world yet
        m_friction = 0.4f;			// Unity default
        m_rollingFriction = 0f;		// TNT default
        m_frictionCombineMode = PhysicMaterialCombine.Multiply; // old APE only mode
        m_restitution = 0f;
        m_restitutionCombineMode = PhysicMaterialCombine.Multiply; // old APE only mode
        m_bodyShape = IntPtr.Zero;
        m_shapeType = tntShapeType.INVALID;
        m_collidable = true;
        m_added = false;
        m_isTriggerShadowed = false; // since tntLink is always created non-trigger
        m_colliderCached = null;
        m_tntColliderCached = null;
        m_accumulatedForce.Set(0,0,0);
        m_accumulatedTorque.Set(0,0,0);
        m_drag = 0.0f;
        m_angularDrag = 0.0f;
    }

    /**
     * Sets physic material for this link
     * @param material physic material to set
     * @remark Physic material describes object's friction and restitution
     * @see http://docs.unity3d.com/ScriptReference/PhysicMaterial.html
     */
    public void SetMaterial(PhysicMaterial material)
    {
        m_material = material;
    }

    /**
     * Returns physic material for this link
     * @return physic material set for this link
     * @remark Physic material describes object's friction and restitution
     * @see http://docs.unity3d.com/ScriptReference/PhysicMaterial.html
     */
    public PhysicMaterial GetMaterial()
    {
        return m_material;
    }

    /**
     * Converts physic material to APE's internal material description
     * @see http://docs.unity3d.com/ScriptReference/PhysicMaterial.html
     */
    public void ParseMaterial()
	{
		if (m_material == null)
		{
			Collider collider = gameObject.GetComponent<Collider>();
			if (collider != null)
			{
				m_material = collider.sharedMaterial;
			}
		}
		
		if (m_material != null)
		{
			m_friction = m_material.staticFriction;
			// HACK: Use Unity's dynamicFriction as TNT's rolling friction
			m_rollingFriction = m_material.dynamicFriction;
            m_frictionCombineMode = m_material.frictionCombine;

			m_restitution = m_material.bounciness;
            m_restitutionCombineMode = m_material.bounceCombine;
		}
	}

    public virtual void Awake()
    {
        // Start is called after all components on the GameObject are initialized
        m_transform = transform;
    }

    public virtual void Start()
    {
        if (gameObject != null)
        {
            Collider collider = gameObject.GetComponent<Collider>();
            if (collider != null)
            {
                m_colliderCached = collider;
                if (m_isTriggerShadowed != m_colliderCached.isTrigger)
                {
                    m_isTriggerShadowed = m_colliderCached.isTrigger;
                    TNT.apSetLinkTriggerStatus(GetBase(), GetIndex(), m_colliderCached.isTrigger);
                }
            }
            else
            {
                tntCollider tntcollider = gameObject.GetComponent<tntCollider>();
                if (tntcollider != null)
                {
                    m_tntColliderCached = tntcollider;
                    if (m_isTriggerShadowed != m_tntColliderCached.IsTrigger)
                    {
                        m_isTriggerShadowed = m_tntColliderCached.IsTrigger;
                        TNT.apSetLinkTriggerStatus(GetBase(), GetIndex(), m_tntColliderCached.IsTrigger);
                    }
                }
            }
        }
    }

    public virtual void Update()
    {
        //SyncTriggerFlag();
    }

    /**
     * Updates sync trigger setting for this tntLink from Unity Collider/tntCollider assigned to it.
     * Trigger setting is also forwarded to internal representation of this link.
     */
    public void SyncTriggerFlag()
    {
        if (m_colliderCached != null)
        {
            if (m_isTriggerShadowed != m_colliderCached.isTrigger)
            {
                m_isTriggerShadowed = m_colliderCached.isTrigger;
                TNT.apSetLinkTriggerStatus(GetBase(), GetIndex(), m_colliderCached.isTrigger);
            }
        }
        else if (m_tntColliderCached != null)
        {
            if (m_isTriggerShadowed != m_tntColliderCached.IsTrigger)
            {
                m_isTriggerShadowed = m_tntColliderCached.IsTrigger;
                TNT.apSetLinkTriggerStatus(GetBase(), GetIndex(), m_tntColliderCached.IsTrigger);
            }
        }
    }

    /**
     * Applies local scale stored in this tntLink to the internal representation of this link
     */
	public void SetLocalScale()
	{ 
		TNT.apSetScaling(m_bodyShape, m_transform.localScale.x, m_transform.localScale.y, m_transform.localScale.z); 
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
                                  float upwardsModifier = 0.0F, ForceMode mode = ForceMode.Force)
    {
        Vector3 forceDir = m_transform.position - explosionPosition;
        float dist = forceDir.magnitude;
        forceDir.Normalize();
        Vector3 force = explosionRadius == 0.0f ? explosionForce * forceDir : dist < explosionRadius ? dist/explosionRadius * explosionForce * forceDir : 0 * forceDir;
        AddForce(force, mode);
        
    }

    private void AddOneTimeForceToMultiBody(IntPtr theBase, int linkIndex, Vector3 force, Vector3 posWorld, bool ignoreWorldPos)
    {
        unsafe
        {
            TNT.apAddOneTimeForceToMultiBody(theBase, linkIndex, &force, &posWorld, ignoreWorldPos);
        }
    }

    /**
     * Applies force to this link at its center of mass
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
                m_accumulatedForce += force;
                break;
            case ForceMode.Impulse:
                AddOneTimeForceToMultiBody(GetBase(), GetIndex(), force * m_world.m_numSteps, m_position, true);
                break;
            case ForceMode.Acceleration:
                m_accumulatedForce += force * m_mass;
                break;
            case ForceMode.VelocityChange:
                AddOneTimeForceToMultiBody(GetBase(), GetIndex(), force * m_mass, m_position, true);
                break;
        }
    }

    /**
    * Applies force to this link at the desired location
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
                m_accumulatedTorque += Vector3.Cross(relPos, force);
            }
                break;
            case ForceMode.Impulse:
                AddOneTimeForceToMultiBody(GetBase(), GetIndex(), force * m_world.m_numSteps, position, false);
                break;
            case ForceMode.Acceleration:
                Debug.LogError("Adding acceleration at position is not supported.");
                break;
            case ForceMode.VelocityChange:
                Debug.LogError("Adding velocity change at position is not supported.");
                break;
        }
    }

    /**
     * Applies force to this link at its center of mass
     * @param force vector expressed in link-local coordinates
     * @param mode indicates how the force vector should be interpreted
     * @remark The actual application is deferred until the next simulation step
     * @see http://docs.unity3d.com/ScriptReference/ForceMode.html
     */
    public void AddRelativeForce(Vector3 force, ForceMode mode = ForceMode.Force)
    {
        Vector3 forceWorldSpace = m_transform.TransformVector(force);
        AddForce(forceWorldSpace, mode);
    }

    /**
     * Applies torque to this link
     * @param torque vector expressed in link-local coordinates
     * @param mode indicates how the torque vector should be interpreted
     * @remark The actual application is deferred until the next simulation step
     * @see http://docs.unity3d.com/ScriptReference/ForceMode.html
     */
    public void AddRelativeTorque(Vector3 torque, ForceMode mode = ForceMode.Force)
    {
        Vector3 torqueWorldSpace = m_transform.TransformVector(torque);
        AddTorque(torqueWorldSpace, mode);
    }

    private void AddOneTimeTorqueToMultiBody(IntPtr theBase, int linkIndex, Vector3 torque)
    {
        unsafe
        {
            TNT.apAddOneTimeTorqueToMultiBody(theBase, linkIndex, &torque);
        }
    }

    /**
     * Applies torque to this link
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
                m_accumulatedTorque += torque;
                break;
            case ForceMode.Impulse:
                AddOneTimeTorqueToMultiBody(GetBase(), GetIndex(), torque * m_world.m_numSteps);
                break;
            case ForceMode.Acceleration:
                Debug.LogError("Adding angular acceleration is not supported.");
                break;
            case ForceMode.VelocityChange:
                Debug.LogError("Adding angular velocity change is not supported.");
                break;
        }
    }

    /**
     * Sets this link's T-Pose to current Unity transform position and orientation values.
     * Operation will not be performed unless T-Pose is in unset state of override flag is set.
     * @param override flag which determines whether T-Pose should be set if it's already been set
     * @param coordinateFrame optional coordinate frame relative to which T-Pose info is stored; can be null
     */
    public virtual void SetTPoseFromCurrentTransform(bool overwrite, Transform coordinateFrame)
    {
        if (m_TPose.m_WasSet && !overwrite)
            return;

        m_TPose.m_Pos = coordinateFrame ? coordinateFrame.InverseTransformPoint(transform.position) : transform.position;
        m_TPose.m_Rot = coordinateFrame ? Quaternion.Inverse(coordinateFrame.rotation) * transform.rotation : transform.rotation;
        m_TPose.m_WasSet = true;
    }

    /**
     * Sets this link's current Unity transform position and orientation to T-Pose values.
     * @param coordinateFrame optional coordinate frame relative to which T-Pose info is stored; can be null
    */
    public virtual void SetCurrentTransformFromTPose(Transform coordinateFrame)
    {
        transform.position = coordinateFrame ? coordinateFrame.TransformPoint(m_TPose.m_Pos) : m_TPose.m_Pos;
        transform.rotation = coordinateFrame ? coordinateFrame.rotation * m_TPose.m_Rot : m_TPose.m_Rot;
    }

    /**
     * Resets this link's T-Pose to defaults
     */
    public virtual void ResetTPose()
    {
        m_TPose.m_Pos.Set(0f, 0f, 0f);
        m_TPose.m_Rot.Set(0f, 0f, 0f, 1f);
        m_TPose.m_WasSet = false;
    }

    /**
     * Fills reducedState state vector with data pertaining to this link
     * @param reducedState state vector to be filled
     * @param offset offset into the vector at which this link is to put its data
     */
    public abstract void FillReducedStateVectorFromCurrentState(tntReducedState reducedState, int offset);

    public abstract void SetBodyShapeToKernel(IntPtr shape);
    public abstract void ComputeMoIFromCollisionShape();
}