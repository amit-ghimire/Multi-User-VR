using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;

using PhysicsAPI;

public static class tntEntityAndJointFactory
{
	public static bool CreateRigidBody(tntRigidBody rigidBody, bool recreated = false)
    {
        if (rigidBody.ContainedByCompoundShape())
            return true;

        if (rigidBody.m_rigidBody != IntPtr.Zero && !recreated)
            return true;

        if (rigidBody.m_rigidBody != IntPtr.Zero)
        {
            if (rigidBody.m_added)
            {
                // this will remove the joints as well, but the connection is
                // still valid until the rigid body is readded to the world
                rigidBody.m_world.RemoveRigidBody(rigidBody);
            }

            TNT.apDeleteRigidBody(rigidBody.m_rigidBody);
            rigidBody.m_rigidBody = IntPtr.Zero;
        }

        if (rigidBody.m_bodyShape != IntPtr.Zero)
        {
            TNT.apDeleteShape(rigidBody.m_bodyShape);
            rigidBody.m_bodyShape = IntPtr.Zero;
        }

        float[] masses = null;
        float[] inertias = null;
        Vector3 transformOffset;

        rigidBody.m_bodyShape = ColliderFactory.AddCollider(
            rigidBody, rigidBody.m_mass == 0, out rigidBody.m_shapeType, out masses, out inertias, out transformOffset);

        if (rigidBody.m_bodyShape == IntPtr.Zero)
        {
            Debug.LogError("Adding collider failed for tntRigidBody:" + rigidBody.transform.name);
            return false;
        }

        if ((rigidBody.m_shapeType == tntShapeType.HeightMap || rigidBody.m_shapeType == tntShapeType.ConcaveMeshStatic) && !rigidBody.IsStatic())
        {
            Debug.LogError("Configured static collision shapes on non-static tntRigidBody:" + rigidBody.transform.name);
            return false;
        }

        if (rigidBody.m_isSetLocalScale)
            rigidBody.SetLocalScale();

#if DEBUG
        // FIXME: this can be done via Unity helper attributes
        tntRigidBody[] bodies = rigidBody.transform.GetComponentsInChildren<tntRigidBody>(true);
        if (bodies.Length > 1 && rigidBody.transform.GetComponent<tntCompoundCollider>() == null)
        {
            Debug.LogError("More than one tntRigidBodies on:" + rigidBody.transform.name);
            return false;
        }

        tntLink[] links = rigidBody.transform.GetComponentsInChildren<tntLink>(true);
        if (links.Length > 0)
        {
            Debug.LogError("tntLink cannot be configured on tntRigidBody:" + rigidBody.transform.name, rigidBody);
            return false;
        }
#endif
        // sync only transform on creation because they are in Transform
        // and not serialized to this instance
        rigidBody.m_position = rigidBody.transform.position + transformOffset;
        rigidBody.m_rotation = rigidBody.transform.rotation;
        rigidBody.transform.hasChanged = false;

        // Default physics material
        float friction = 0.4f;			// Unity default
        float rollingFriction = 0f;		// TNT default
        float restitution = 0f;

        PhysicMaterialCombine frictionCombineMode = PhysicMaterialCombine.Multiply; // old APE only mode
        PhysicMaterialCombine restitutionCombineMode = PhysicMaterialCombine.Multiply; // old APE only mode

        if (rigidBody.m_material == null)
        {
            Collider[] colliders = rigidBody.gameObject.GetComponents<Collider>();
            if (colliders.Length > 1)
            {
                Debug.LogWarning("More than one collider is found, only the first material will be applied");
            }

            if (colliders.Length > 0)
            {
                rigidBody.m_material = colliders[0].sharedMaterial;
            }
        }

        if (rigidBody.m_material != null)
        {
            friction = rigidBody.m_material.staticFriction;
            // HACK: Use Unity's dynamicFriction as TNT's rolling friction
            rollingFriction = rigidBody.m_material.dynamicFriction;
            restitution = rigidBody.m_material.bounciness;

            frictionCombineMode = rigidBody.m_material.frictionCombine;
            restitutionCombineMode = rigidBody.m_material.bounceCombine;
        }

        if (masses == null)
        {
            masses = new float[1] { rigidBody.m_mass };
        }
        if (inertias == null)
        {
            inertias = new float[3] { rigidBody.m_moi.x, rigidBody.m_moi.y, rigidBody.m_moi.z };
        }

        unsafe
        {
            Vector3 linTmpBuffer = rigidBody.linearVelocity; // This is only input value parameter. no reference is saved in kernel
            Vector3 angTmpBuffer = rigidBody.angularVelocity * Mathf.Deg2Rad; // This is only input value parameter. no reference is saved in kernel

            fixed (Vector3* pos = &rigidBody.m_position)
            fixed (Quaternion* rot = &rigidBody.m_rotation)
            fixed (float* mass = &masses[0])
            fixed (float* inertia = &inertias[0])
            fixed (Vector3* moi = &rigidBody.m_moi)            
            {
                rigidBody.m_rigidBody = TNT.apCreateRigidBody(
                    rigidBody.m_mass == 0 ? null : mass, inertia, moi, rigidBody.m_bodyShape, pos, rot,
                    &linTmpBuffer, &angTmpBuffer, friction, rollingFriction, (int)frictionCombineMode, restitution, (int)restitutionCombineMode);
            }

            fixed (Vector3* pos = &rigidBody.m_position)
            fixed (Quaternion* rot = &rigidBody.m_rotation)
            fixed (Vector3* force = &rigidBody.m_accumulatedForce)
            fixed (Vector3* torque = &rigidBody.m_accumulatedTorque)
            fixed (int* worldIndex = &rigidBody.m_worldIndex)
            fixed (float* velocity = &rigidBody.CurrentVelocity[0])
            fixed (float* drag = &rigidBody.m_drag)
            fixed (float* angularDrag = &rigidBody.m_angularDrag)            
            {
                TNT.apInstallRigidBodySharedMemoryBuffers(
                    rigidBody.m_rigidBody,
                    pos, rot,
                    force, torque,
                    (IntPtr)worldIndex, velocity,
                    drag, angularDrag);
            }
        }

        rigidBody.SetCollidable(rigidBody.m_collidable);

        rigidBody.m_flag = 0;

        Collider collider = rigidBody.gameObject.GetComponent<Collider>();
        if (collider != null)
        {
            rigidBody.m_colliderCached = collider;
            if (rigidBody.m_isTriggerShadowed != rigidBody.m_colliderCached.isTrigger)
            {
                rigidBody.m_isTriggerShadowed = rigidBody.m_colliderCached.isTrigger;
                TNT.apSetRigidBodyTriggerStatus(rigidBody.GetRigidBody(), rigidBody.m_colliderCached.isTrigger);
            }
        }
        else
        {
            tntCollider tntcollider = rigidBody.gameObject.GetComponent<tntCollider>();
            if (tntcollider != null)
            {
                rigidBody.m_tntColliderCached = tntcollider;
                if (rigidBody.m_isTriggerShadowed != rigidBody.m_tntColliderCached.IsTrigger)
                {
                    rigidBody.m_isTriggerShadowed = rigidBody.m_tntColliderCached.IsTrigger;
                    TNT.apSetRigidBodyTriggerStatus(rigidBody.GetRigidBody(), rigidBody.m_tntColliderCached.IsTrigger);
                }
            }
        }

        if (rigidBody.hasCollisionListeners)
        {
            TNT.apAddListenerRigidBody(rigidBody.m_rigidBody);
        }
        return true;
    }

    public static void DestroyRigidBody(tntRigidBody rigidBody)
    {
        {
            if (rigidBody.m_rigidBody != IntPtr.Zero)
            {
                TNT.apDeleteRigidBody(rigidBody.m_rigidBody);
                rigidBody.m_rigidBody = IntPtr.Zero;
            }

            if (rigidBody.m_bodyShape != IntPtr.Zero)
            {
                // Collision shape sharing among multiple bodies is currently not supported. See ColliderFactory.AddColider remark.
                // We can safely delete collider.
                ColliderFactory.DeleteCollider(rigidBody.m_bodyShape, rigidBody.m_shapeType);
                rigidBody.m_bodyShape = IntPtr.Zero;
            }

            // joints has been removed from the world already
        }
    }

    public static bool CreateConstraintInKernel(tntRigidBodyConstraint joint)
    {
        tntWorld.Assert(!joint.m_added && joint.m_constraint == IntPtr.Zero);

        if (joint.AreBothRigidBodiesEnabled())
        {
            unsafe
            {
                fixed (Vector3* pivotA = &joint.m_pivotA)
                fixed (Vector3* pivotB = &joint.m_pivotB)
                fixed (Vector3* axisA = &joint.m_axisA)
                fixed (Vector3* axisB = &joint.m_axisB)
                fixed (void* pFB = &joint.m_feedback.m_appliedImpulse)
                {
                    switch (joint.type)
                    {
                        case tntRigidBodyConstraint.Type.Ball:
                            joint.m_constraint = TNT.apCreateP2PConstraint(
                                joint.bodyA.m_rigidBody, joint.bodyB.m_rigidBody, pivotA, pivotB,
                                joint.m_breakingImpulse, joint.m_overrideNumIterations, pFB);
                            break;
                        case tntRigidBodyConstraint.Type.Fixed:
                            joint.m_constraint = TNT.apCreateFixedConstraint(
                                joint.bodyA.m_rigidBody, joint.bodyB.m_rigidBody, pivotA, pivotB,
                                joint.m_breakingImpulse, joint.m_overrideNumIterations, pFB);
                            break;
                        case tntRigidBodyConstraint.Type.Hinge:
                            joint.m_constraint = TNT.apCreateHingeConstraint(
                                 joint.bodyA.m_rigidBody, joint.bodyB.m_rigidBody,
                                 pivotA, pivotB, axisA, axisB,
                                 joint.m_breakingImpulse, joint.m_overrideNumIterations, pFB);
                            break;
                    }
                }
            }
        }
        return joint.m_constraint != IntPtr.Zero;
    }

    public static void DestroyConstraintInKernel(tntRigidBodyConstraint joint)
    {
        if (joint.m_constraint == IntPtr.Zero)
        {
            return;
        }

        joint.m_world.RemoveConstraint(joint);

        TNT.apDeleteConstraint(joint.m_constraint);
        joint.m_constraint = IntPtr.Zero;
    }

    /**
     * Creates the multibody to the physics kernel: this includes creating the base and all of its child links.
     * @remark Assumption: All articulation links are child game objects of the articulation base's parent game object (they're on the same scene graph branch).
     * @remark The link indices allocation algorithm guarantees that parents always have smaller indices than children
     */
    public static void CreateArticulationBase(tntBase baseLink)
    {
        if (!baseLink.m_world || baseLink.m_base != IntPtr.Zero || baseLink.m_articulationCreated)
        {
            tntWorld.Assert(false, "CreateArticulationBase() called in invalid moment.");
            return; // tmp
        }

        float[] masses = null;
        float[] inertias = null;
        Vector3 transformOffset;
        baseLink.m_bodyShape = ColliderFactory.AddCollider(baseLink, baseLink.m_mass == 0, out baseLink.m_shapeType, out masses, out inertias, out transformOffset);
        if (baseLink.m_bodyShape == IntPtr.Zero)
        {
            Debug.LogErrorFormat("Failed to add collider to base link, creation of articulation {0} terminated", baseLink.name);
            return;
        }
        baseLink.SetTPoseFromCurrentTransform(false, null);
        baseLink.SetCurrentTransformFromTPose(null);

        baseLink.m_position = baseLink.transform.position + transformOffset;
        baseLink.m_rotation = baseLink.transform.rotation;

        baseLink.ParseMaterial();
        baseLink.m_world.AddArticulationLink(baseLink); //< This is only registration done upon creation

        // Assumption: All articulation links are child game objects of the articulation base game object
        // The link indices allocatino algorithm guarantees that parents always have smaller indices than
        // children
        tntChildLink[] links = null;
        bool isMultiDOF = true;
        if (baseLink.transform.parent != null)
        {
            tntChildLink[] allLinks = baseLink.transform.parent.GetComponentsInChildren<tntChildLink>(false);
            List<tntChildLink> childList = new List<tntChildLink>();
            foreach (tntChildLink cl in allLinks)
            {
                if (!cl.m_parent)
                {
                    Debug.LogError("tntChildLink '" + cl.name + "' doesn't have a parent link, please fix that.");
                    return;
                }
                if (cl.enabled && cl.m_parent && cl.baseLink == baseLink)
                    childList.Add(cl);
            }
            links = childList.ToArray();

        }

        if (baseLink.m_computeMoiFromColliders)
            baseLink.m_moi.Set(0.0f, 0.0f, 0.0f);

        if (masses == null)
            masses = new float[1] { baseLink.m_mass };
        if (inertias == null)
            inertias = new float[3] { baseLink.m_moi.x, baseLink.m_moi.y, baseLink.m_moi.z }; // dummy
        unsafe
        {
            fixed (Vector3* pos = &baseLink.m_position)
            fixed (Quaternion* rot = &baseLink.m_rotation)
            fixed (float* massPtr = &masses[0])
            fixed (float* inertia = &inertias[0])
            fixed (Vector3* moi = &baseLink.m_moi)            
            {
                baseLink.m_base = TNT.apCreateArticulationBase(links != null ? links.Length : 0,
                                                      baseLink.m_mass == 0 ? null : massPtr,
                                                      inertia,
                                                      moi,
                                                      baseLink.m_bodyShape, true, isMultiDOF,
                                                      baseLink.m_enableSelfCollision,
                                                      baseLink.m_highDefIntegrator,
                                                      baseLink.m_useGlobalVel,
                                                      baseLink.m_simulationFrequencyMultiplier,
                                                      baseLink.m_linearDamping,
                                                      baseLink.m_angularDamping,
                                                      baseLink.m_maxAppliedImpulse,
                                                      baseLink.m_maxCoordinateVelocity,
                                                      baseLink.m_friction, baseLink.m_rollingFriction, (int)baseLink.m_frictionCombineMode,
                                                      baseLink.m_restitution, (int)baseLink.m_restitutionCombineMode,
                                                      pos, rot);
            }

            fixed (Vector3* pos = &baseLink.m_position)
            fixed (Quaternion* rot = &baseLink.m_rotation)
            fixed (int* pWorldIndex = &baseLink.m_worldIndex)
            fixed (Vector3* force = &baseLink.m_accumulatedForce)
            fixed (Vector3* torque = &baseLink.m_accumulatedTorque)            
            fixed (float* velocity = &baseLink.CurrentVelocity[0])
            fixed (float* drag = &baseLink.m_drag)
            fixed (float* angularDrag = &baseLink.m_angularDrag)
            {
                TNT.apInstallArticulationBaseSharedMemoryBuffers(
                    baseLink.m_base,
                    pos, rot,
                    force, torque,
                    (IntPtr)pWorldIndex, velocity,
                    drag, angularDrag);
            }
        }

        if (links != null)
        {
            baseLink.AssignValidChildLinkIndices();
            baseLink.m_childLinks = new List<tntChildLink>(links);
            baseLink.m_childLinks.Sort((tntChildLink link1, tntChildLink link2) => link1.GetIndex() - link2.GetIndex());
            
            baseLink.m_nameToLink = new Dictionary<string, tntChildLink>();

            foreach (tntChildLink childLink in baseLink.m_childLinks)
            {
                if (baseLink.m_nameToLink.ContainsKey(childLink.gameObject.name))
                    Debug.LogWarning("Duplicate link name : " + childLink.gameObject.name);
                else 
                    baseLink.m_nameToLink.Add(childLink.gameObject.name, childLink);
            {
                    // non-null m_parent assumed
                    childLink.SetTPoseFromCurrentTransform(false, childLink.m_parent.transform);
                    childLink.SetCurrentTransformFromTPose(childLink.m_parent.transform);

                    childLink.SetWorld(baseLink.m_world);
                    childLink.m_position = childLink.transform.position;
                    childLink.m_rotation = childLink.transform.rotation;
                    childLink.ParseMaterial();
                    tntShapeType shapeType;

                    masses = null;
                    inertias = null;
                    IntPtr childShape = ColliderFactory.AddCollider(childLink, childLink.m_mass == 0,
                                                                      out shapeType, out masses, out inertias,
                                                                    out transformOffset);
                    if (childShape == IntPtr.Zero)
                    {
                        Debug.LogErrorFormat("Failed to add collider to child link {0}, creation of articulation {1} terminated", childLink.name, baseLink.name);
                        return;
                    }
                    childLink.SetBodyShape(childShape);

                    if (childLink.m_computeMoiFromColliders)
                        childLink.m_moi.Set(0.0f, 0.0f, 0.0f);

                    if (masses == null)
                        masses = new float[1] { childLink.m_mass };
                    if (inertias == null)
                        inertias = new float[3] { childLink.m_moi.x, childLink.m_moi.y, childLink.m_moi.z }; // dummy

                    childLink.SetShapeType(shapeType);
                    childLink.SetBase(baseLink.m_base);

                    IntPtr mobilizer = IntPtr.Zero;

                    baseLink.m_world.AddArticulationLink(childLink);

                    childLink.m_asciiName = Encoding.ASCII.GetBytes(childLink.name);
                    if (childLink as tntFixedLink)
                    {
                        unsafe
                        {
                            fixed (Vector3* pos = &childLink.m_position)         // in/out 
                            fixed (Quaternion* rot = &childLink.m_rotation)      // in/out
                            fixed (void* pName = &childLink.m_asciiName[0])          // in
                            fixed (float* massPtr = &masses[0])                     // in
                            fixed (float* inertia = &inertias[0])                // in
                            fixed (Vector3* moi = &childLink.m_moi)             // in
                            fixed (void* feedback = &childLink.m_feedback)      // out
                            fixed (Vector3* force = &childLink.m_accumulatedForce)   // in
                            fixed (Vector3* torque = &childLink.m_accumulatedTorque) // in
                            fixed (float* drag = &childLink.m_drag)                  // in
                            fixed (float* angularDrag = &childLink.m_angularDrag)    // in
                            {
                                TNT.apSetupFixedLink(
                                    baseLink.m_base,
                                    childLink.m_collideWithParent,
                                    childLink.GetIndex(),
                                    childLink.m_parent.GetIndex(),
                                    childLink.m_mirroredLink ? childLink.m_mirroredLink.GetIndex() : -1,
                                    childLink.m_mass == 0 ? null : massPtr,
                                    inertia,
                                    moi,
                                    childLink.GetBodyShape(),
                                    childLink.GetFriction(),
                                    childLink.GetRollingFriction(), 
                                    (int)childLink.GetFrictionCombineMode(),
                                    childLink.GetRestitution(), 
                                    (int)childLink.GetRestitutionCombineMode(),
                                    pos, rot
                                );
                            }

                            fixed (Vector3* pos = &childLink.m_position)
                            fixed (Quaternion* rot = &childLink.m_rotation)
                            fixed (Vector3* force = &childLink.m_accumulatedForce)
                            fixed (Vector3* torque = &childLink.m_accumulatedTorque)
                            fixed (float* drag = &childLink.m_drag)
                            fixed (float* angularDrag = &childLink.m_angularDrag)
                            fixed (void* feedback = &childLink.m_feedback)
                            {
                                TNT.apInstallArticulationLinkSharedMemoryBuffers(
                                    baseLink.m_base,
                                    childLink.GetIndex(),
                                    pos, rot,
                                    force, torque,
                                    childLink.GetWorldIndexRef(),
                                    null,
                                    null,
                                    drag,
                                    angularDrag,
                                    feedback);
                            }
                        }
                    }
                    else
                    {
                        float[] springStiffness = new float[childLink.m_dofData.Length];
                        float[] springDamping = new float[childLink.m_dofData.Length];
                        for (int k = 0; k < childLink.m_dofData.Length; ++k)
                        {
                            springStiffness[k] = childLink.m_dofData[k].m_springStiffness;
                            springDamping[k] = childLink.m_dofData[k].m_springDamping;
                        }

                        Vector3 pivotA, pivotB, axisA, axisB;
                        int jointType = -1;

                        tntHingeLink hinge = childLink as tntHingeLink;
                        tntUniversalLink universal = childLink as tntUniversalLink;
                        tntSliderLink slider = childLink as tntSliderLink;
                        tntPlaneLink plane = childLink as tntPlaneLink;
                        tntBallLink ball = childLink as tntBallLink;
                        if (hinge != null)
                        {
                            pivotA = hinge.PivotA;
                            pivotB = hinge.PivotB;
                            axisA = hinge.m_axisA;
                            jointType = 0;
                        }
                        else if (universal != null)
                        {
                            pivotA = universal.PivotA;
                            pivotB = universal.PivotB;
                            axisA = universal.m_axisA;
                            axisB = universal.m_axisB;
                            jointType = 5;

                            Vector3 axisAInChild = Vector3.Normalize(Quaternion.Inverse(childLink.transform.rotation) * childLink.m_parent.transform.rotation * universal.m_axisA);
                            Vector3 axisBInChild = Vector3.Normalize(universal.m_axisB);
                            float dotValueAbs = Mathf.Abs(Vector3.Dot(axisAInChild, axisBInChild));

                            // Orthogonalize axes if needed
                            if (dotValueAbs > 0f)
                            {
                                if (dotValueAbs >= 1e-1)
                                    Debug.LogWarningFormat("Your universal link's ({0}) axes are far from orthogonal.They will get auto-orthogonalized but you should probably check your setup.", childLink.name);

                                Vector3.OrthoNormalize(ref axisAInChild, ref axisBInChild);

                                axisA = Quaternion.Inverse(childLink.m_parent.transform.rotation) * childLink.transform.rotation * axisAInChild;
                                axisB = axisBInChild;

                                universal.m_axisA = axisA;
                                universal.m_axisB = axisB;
                            }
                        }
                        else if (slider != null)
                        {
                            axisA = slider.m_axisA;
                            jointType = 1;
                        }
                        else if (ball != null)
                        {
                            pivotA = ball.PivotA;
                            pivotB = ball.PivotB;
                            jointType = 2;
                        }
                        else if (plane != null)
                        {
                            axisA = plane.m_axisA;
                            jointType = 3;
                        }
                        else
                        {
                            continue;
                        }

                        unsafe
                        {
                            fixed (Vector3* pos = &childLink.m_position)            // in/out 
                            fixed (Quaternion* rot = &childLink.m_rotation)     // in/out
                            fixed (float* stiffness = &springStiffness[0])      // in
                            fixed (float* damping = &springDamping[0])          // in
                            fixed (float* massPtr = &masses[0])                     // in
                            fixed (float* inertia = &inertias[0])                // in
                            fixed (Vector3* moi = &childLink.m_moi)             // in                            
                            {
                                mobilizer = TNT.apSetupArticulationLink(
                                    baseLink.m_base,
                                    jointType,
                                    childLink.m_collideWithParent,
                                    childLink.GetIndex(), childLink.m_parent.GetIndex(), childLink.m_mirroredLink ? childLink.m_mirroredLink.GetIndex() : -1,
                                    childLink.m_mass == 0 ? null : massPtr, inertia, moi,
                                    childLink.GetBodyShape(),
                                    childLink.GetFriction(),
                                    childLink.GetRollingFriction(),
                                    (int)childLink.GetFrictionCombineMode(),
                                    childLink.GetRestitution(),
                                    (int)childLink.GetRestitutionCombineMode(),
                                    pos, rot,
                                    &pivotA, &axisA, &pivotB, &axisB,
                                    stiffness,
                                    damping
                                );
                            }

                            {
                                for (int k = 0; k < childLink.m_dofData.Length; ++k)
                                {
                                    TNT.apSetContinuousForceActuator(mobilizer, k, childLink.m_dofData[k].m_continuousForce);
                                }
                            }

                            fixed (Vector3* pos = &childLink.m_position)
                            fixed (Quaternion* rot = &childLink.m_rotation)                            
                            fixed (Vector3* force = &childLink.m_accumulatedForce)
                            fixed (Vector3* torque = &childLink.m_accumulatedTorque)
                            fixed (float* curPos = &childLink.CurrentPosition[0])
                            fixed (float* curVel = &childLink.CurrentVelocity[0])
                            fixed (float* drag = &childLink.m_drag)
                            fixed (float* angularDrag = &childLink.m_angularDrag)
                            fixed (void* feedback = &childLink.m_feedback)
                            {
                                TNT.apInstallArticulationLinkSharedMemoryBuffers(
                                    baseLink.m_base,
                                    childLink.GetIndex(),
                                    pos, rot,
                                    force, torque,
                                    childLink.GetWorldIndexRef(),
                                    curPos,
                                    curVel,
                                    drag,
                                    angularDrag,
                                    feedback);
                            }

                            {
                                float breakImpulseRaw = childLink.m_breakingReactionImpulse;
                                float breakImpulse =
                                    breakImpulseRaw < 0.0f || float.IsInfinity(breakImpulseRaw) || float.IsNaN(breakImpulseRaw)
                                    ? -1.0f
                                    : breakImpulseRaw;
                                TNT.apSetLinkBreakingImpulseThreshold(mobilizer, breakImpulse);
                            }
                        }
                    }

                    childLink.SetWorld(baseLink.m_world);
                    childLink.SetAdded();
                    childLink.SetMobilizer(mobilizer);
                }
            }
        }

        baseLink.m_articulationCreated = true;
    }

    /**
     * Destroys the articulation in kernel
     * @param multiBody base representing the multibody to be removed
     */
	public static void DestroyArticulationBase(tntBase baseLink)
    {
        tntWorld.Assert(!baseLink.m_added, "Articulation must have been removed from world when calling DestroyArticulationBase()");
        // TBD:  Remove articulation links and motor constraints
        if (baseLink.m_articulationCreated && baseLink.m_base != IntPtr.Zero)
        {
            if (baseLink.transform.parent != null)
            {
                tntChildLink[] links = null;
                links = baseLink.transform.parent.GetComponentsInChildren<tntChildLink>(false);
                if (links != null)
                {
                    for (int i = 0; i < links.Length; ++i)
                    {
                        baseLink.m_world.RemoveArticulationLink(links[i]);
                        if (links[i].m_bodyShape != IntPtr.Zero)
                        {
                            // Collision shape sharing among multiple bodies is currently not supported. See ColliderFactory.AddColider remark.
                            // We can safely delete collider.
                            ColliderFactory.DeleteCollider(links[i].m_bodyShape, links[i].m_shapeType);
                            links[i].m_bodyShape = IntPtr.Zero;
                        }
                    }
                }
            }
            baseLink.m_world.RemoveArticulationLink(baseLink); // Removes ArticluationLink(this)

            TNT.apDeleteArticulation(baseLink.GetBase());
            baseLink.SetBase(IntPtr.Zero);
            for (int linkIndex = 0; linkIndex < baseLink.numLinks(); ++linkIndex)
                baseLink.getChildLink(linkIndex).SetMobilizer(IntPtr.Zero);

            baseLink.m_base = IntPtr.Zero;
            baseLink.m_nameToLink = null;
            baseLink.m_articulationCreated = false;

            if (baseLink.m_bodyShape != IntPtr.Zero)
            {
                // Collision shape sharing among multiple bodies is currently not supported. See ColliderFactory.AddColider remark.
                // We can safely delete collider.
                ColliderFactory.DeleteCollider(baseLink.m_bodyShape, baseLink.m_shapeType);
                baseLink.m_bodyShape = IntPtr.Zero;
            }
        }

    }

}