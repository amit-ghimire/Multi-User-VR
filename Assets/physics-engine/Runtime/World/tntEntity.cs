using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using PhysicsAPI;

/**
 * This is the base class for tntRidbody and tntLink, where the latter is the building block of Articulations pipeline.
 * 
 * This holds common properties and methods used in both simulation pipelines. This is an abstract class.
 * 
 * @remark tntEntity also holds some members that belong to tntLink only. This is to allow handling of all entity types
 * in one place, and make it easier to keep code consistent for all entity types.
 */
public abstract class tntEntity : MonoBehaviour {

        /// The type of tntEntity
    public enum Type
    {
        Invalid, ///< Initial invalid value
        RigidBody, ///< tntRigidBody used in the rigidbody simulation pipeline
        Base, ///< tntBase, which serves as the root for Articulations
        ChildLink ///< tntChildLink, child links connected directly or indirectly to the tntBase in an Articulation
    }

        /// The type of this tntEntity; set at construction time
    [SerializeField]
    public Type type { get; protected set; }

        /// Require the constructor to have the tntEntity type parameter.
    protected tntEntity(Type type) { this.type = type; }

    internal tntWorld m_world;

    internal IntPtr m_bodyShape = IntPtr.Zero;
    internal tntShapeType m_shapeType = tntShapeType.INVALID;

    internal int m_worldIndex = -1;	// index in Unity physics world
    internal bool m_added = false;

    // Public properties
    public float m_mass;
    public float m_drag = 0.0f;
    public float m_angularDrag = 0.0f;
    public Vector3 m_moi;
    public PhysicMaterial m_material;
    public Vector3 m_accumulatedForce; //< used for adding force
    public Vector3 m_accumulatedTorque; //< used for adding force

    // cached component references for performance
    [NonSerialized] public Transform m_transform = null;
    [NonSerialized] public Collider m_colliderCached = null;
    [NonSerialized] public tntCollider m_tntColliderCached = null;

    // collision listener references
    internal List<ITntTriggerListener> m_triggerListeners = new List<ITntTriggerListener>();
    internal List<ITntTriggerExtendedListener> m_triggerExtendedListeners = new List<ITntTriggerExtendedListener>();
    internal List<ITntCollisionListener> m_collisionListeners = new List<ITntCollisionListener>();

    // Read-only sensors
    [NonSerialized] public Vector3 m_position = Vector3.zero; //< Read only position sensor
    [NonSerialized] public Quaternion m_rotation = Quaternion.identity; //< Read only rotation sensor    

    //
    // Runtime state
    //

        /// Linear velocity of this tntEntity
    public virtual Vector3 linearVelocity { get { return Vector3.zero; } set { } }

        /// Angular velocity of this tntEntity in deg/s
    public virtual Vector3 angularVelocity { get { return Vector3.zero; } set { } }


    // reflected from engine; 
    // - for tntRigidBody and tntBase this holds world space velocity (linear and angular), so 6 elements. 
    // - for tntChildLinks, one entry for each link's DoF; angular quantities in deg/s
    protected float[] m_currentVelocity; //< Serialized because it's accessed by Inspector

    // Other state
    internal bool m_isTriggerShadowed;

    // Properties
    internal bool hasCollisionListeners { get { return m_collisionListeners.Count + m_triggerListeners.Count + m_triggerExtendedListeners.Count > 0; } }


    /**
     * Checks if this tntEntity's collider is configured as a trigger
     * @return true if this tntEntity's collider is configured as a trigger, false otherwise.
     */
    public bool GetShadowedTriggerFlag()
    {
        return m_isTriggerShadowed;
    }

    public float[] CurrentVelocity
    {
        get { return m_currentVelocity; }
    }

    /**
     * This property is a hack-fix to replace use of `m_mass = value` followed by `SetMass(value)`.
     * @remark Unlike `SetMass(value)` itself, this property always force-sets the mass variable to the new value.
     */
    public float mass
    {
        get { return m_mass; }
        set { m_mass = value; SetMass(value);  }
    }

    /**
     * Sets this link's mass
     * @param mass link mass to set
     */
    public abstract void SetMass(float mass);
	/*{
		Debug.Assert(false, "SetMass not implemented");
	}*/

    /**
     * Sets layer ID for this link
     * @param layerID layer ID to be set
     * @param resetCollisionCache Force deletion of existing collision caches (needed to update filtering on e.g. resting contact) and activate the object
     * @remark APE respects Unity's layer collision matrix so this operation may have impact on collision detection
     * @remark Collision layer of tntLink has to be changed via this API instead of via directly modifying gameObject.layer
     * @see http://docs.unity3d.com/Manual/LayerBasedCollision.html
     */
    public abstract void SetLayerID(int layerID, bool resetCollisionCache = false);

    /**
     * Enables or disables collision detection for this link
     * @param collide true enables collision detection, false disables it
     */
    public abstract void SetCollidable(bool collide, bool resetCollisionCache = false);


    // todo: read & change
    // This is the folow of hooking up the collision/trigger callback
    //  1. the object you want it to have collision/trigger response needs to have a script
    //      that implements either ITntCollisionListener or ITntTriggerListener/ITntTriggerExtendedListener (or both) attached, like this
    //      public class onCollisionTest : MonoBehaviour, ITntCollisionListener, ITntTriggerListener/ITntTriggerExtendedListener 
    //  2. you need to call AddListener for tntRigidbody or tntLink
    //      this can be done in Start() like this example or anytime
    //  3. you could call RemoveListener to remove the listener when you need
    //  -Note-:
    //      adding or removing listener during the collision happens will not be effective immediately,
    //      it will when the collision happens again  

    // Add listener and, if this is the first listener register this body for collision info callbacks in kernel.
    public void AddListener(ITntCollisionListenerBase listener)
    {
        bool hadListeners = hasCollisionListeners;
        if (listener is ITntCollisionListener) m_collisionListeners.Add(listener as ITntCollisionListener);
        if (listener is ITntTriggerListener) m_triggerListeners.Add(listener as ITntTriggerListener);
        if (listener is ITntTriggerExtendedListener) m_triggerExtendedListeners.Add(listener as ITntTriggerExtendedListener);

        if (!hadListeners && hasCollisionListeners)
        {
            switch (type)
            {
                case Type.RigidBody: if ((this as tntRigidBody).m_rigidBody != IntPtr.Zero) TNT.apAddListenerRigidBody((this as tntRigidBody).m_rigidBody); break;
                case Type.Base: TNT.apAddListenerArticulation((this as tntBase).m_base); break;
                case Type.ChildLink: TNT.apAddListenerArticulationLink((this as tntChildLink).m_base, (this as tntChildLink).m_index); break;
            }
        }
    }

    // Remove listener and, if all listeners are removed unregister this body from collision info callbacks in kernel.
    public void RemoveListener(ITntCollisionListenerBase listener)
    {
        bool hadListeners = hasCollisionListeners;
        if (listener is ITntCollisionListener) m_collisionListeners.Remove(listener as ITntCollisionListener);
        if (listener is ITntTriggerListener) m_triggerListeners.Remove(listener as ITntTriggerListener);
        if (listener is ITntTriggerExtendedListener) m_triggerExtendedListeners.Remove(listener as ITntTriggerExtendedListener);

        if (hadListeners && !hasCollisionListeners)
        {
            switch (type)
            {
                // If tntRigidbody::Destroy() is called first the listener will have already been destroyed
                case Type.RigidBody: if ((this as tntRigidBody).m_rigidBody != IntPtr.Zero) TNT.apRemoveListenerRigidBody((this as tntRigidBody).m_rigidBody); break;
                case Type.Base: if ((this as tntBase).m_base != IntPtr.Zero) TNT.apRemoveListenerArticulation((this as tntBase).m_base); break;
                case Type.ChildLink: if ((this as tntChildLink).m_base != IntPtr.Zero) TNT.apRemoveListenerArticulationLink((this as tntChildLink).m_base, (this as tntChildLink).m_index); break;
            }
        }
    }

    /**
     * Reloads collision and trigger callbacks from components attached to this tntEntity
     */
    internal void AddCollisionListenersThatAreComponentsOfThisGameObject()
    {

        tntWorld.Assert((type == Type.RigidBody && (this as tntRigidBody).m_rigidBody != IntPtr.Zero) || (type != Type.RigidBody && (this as tntLink).m_base != IntPtr.Zero), "Legacy assert from old API (consider removing).");
        foreach (var l in GetComponents<ITntCollisionListenerBase>()) AddListener(l);
    }

    internal void RemoveAllCollisionListeners()
    {
        while (0 < m_collisionListeners.Count) RemoveListener(m_collisionListeners[0]);
        while (0 < m_triggerListeners.Count) RemoveListener(m_triggerListeners[0]);
        while (0 < m_triggerExtendedListeners.Count) RemoveListener(m_triggerExtendedListeners[0]);
    }

    /**
     * Reloads collision and trigger callbacks from components attached to this rigid body
     */
    public void UpdateCollisionListeners()
    {
        RemoveAllCollisionListeners();
        AddCollisionListenersThatAreComponentsOfThisGameObject();
    }

}
