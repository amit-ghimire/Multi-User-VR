using System.Collections.Generic;
using UnityEngine;
using System;

public static class RigidBodyGrabbingUtil
{
    // Map holding info on re-parented transforms:
    //  changedParenthoodLookUp[aTransf] = bTransf means that the original parent of ..
    //  .. aTransf is bTransf but was changed due to grabbing
    private static Dictionary<Transform, Transform> changedParenthoodLookUp = new Dictionary<Transform, Transform>();

    public static void GrabRigidBody(MonoBehaviour grabbing, tntRigidBody grabbed)
    {
        if (!grabbing || !grabbed || grabbed.ContainedByCompoundShape() || grabbed.GetKinematic())
            return;

        if (!(grabbing is tntLink) && !(grabbing is tntRigidBody))
            return;

        tntShapeType grabbingBodyShapeType;
        IntPtr grabbingBodyShape = GetGrabbingBodyShape(grabbing, out grabbingBodyShapeType);
        Transform grabbingTransform = grabbing.transform;
        float grabbingMass = GetGrabbingMass(grabbing);

        // TODO: consider adding a compound collider to the grabbing link if it's not there
        if (grabbingBodyShapeType != tntShapeType.CompoundShape)
        {
            Debug.LogError("This link does not have a compound collider!");
            return;
        }

        // define child collider masses of the body being grabbed
        {
            Component mainCollider;
            bool isCompound;
            Component[] childColliders;
            if (ColliderFactory.ExtractColliderDefinitions(grabbed, out mainCollider, out isCompound, out childColliders))
            {
                if (!isCompound)
                    ColliderFactory.TrySetColliderAssignedMassAndMoI(mainCollider, grabbed.m_mass, grabbed.m_moi);
                else
                {
                    int numChildColliders = childColliders.Length;
                    float childColliderMass;
                    Vector3 childColliderMoI;

                    foreach (Component childCollider in childColliders)
                    {
                        if (ColliderFactory.ComputeChildColliderMass(childCollider, out childColliderMass, out childColliderMoI, numChildColliders))
                            ColliderFactory.TrySetColliderAssignedMassAndMoI(childCollider, childColliderMass, childColliderMoI);
                        else
                        {
                            Debug.LogWarning("tntRigidBodyGrabber.Grab failed due to ColliderFactory.ComputeChildColliderMass");
                            return;
                        }
                    }
                }
            }
            else
            {
                Debug.LogWarning("tntRigidBodyGrabber.Grab failed due to ColliderFactory.ExtractColliderDefinitions");
                return;
            }
        }

        // re-parent (Unity) all children *colliders* of the body being grabbed
        {
            int numChildChildren = grabbed.transform.childCount;
            int currChildIdx = 0;
            for (int i = 0; i < numChildChildren; ++i)
            {
                Transform currChild = grabbed.transform.GetChild(currChildIdx);

                if (
                    currChild.GetComponent<Collider>() == null &&
                    currChild.GetComponent<tntCollider>() == null
                    )
                {
                    ++currChildIdx;
                    continue;
                }

                // store the original parent
                changedParenthoodLookUp[currChild] = grabbed.transform;

                // re-parent
                currChild.SetParent(grabbingTransform);
            }
        }

        // re-parent (Unity) the body being grabbed itself
        {
            // store the original parent
            changedParenthoodLookUp[grabbed.transform] = grabbed.transform.parent;

            // re-parent
            grabbed.transform.SetParent(grabbingTransform);
        }

        // re-create grabbing object's collision geometry
        {
            if (grabbingBodyShape != IntPtr.Zero)
            {
                ColliderFactory.DeleteCollider(grabbingBodyShape, grabbingBodyShapeType);
                SetGrabbingBodyShape(grabbing, IntPtr.Zero, tntShapeType.INVALID);
            }

            float[] masses = null;
            float[] inertias = null;
            tntShapeType shapeType;
            Vector3 transformOffset;
            IntPtr newCompoundShape = ColliderFactory.AddCollider(grabbing, grabbingMass == 0, out shapeType, out masses, out inertias, out transformOffset);

            SetGrabbingBodyShape(grabbing, newCompoundShape, shapeType, true);
            ComputeGrabbingMoIFromCollisionShape(grabbing);
        }

        // disable what's left of the body being grabbed
        {
            // this will effectively remove the body being grabbed from the world
            grabbed.enabled = false;
        }


    }

    public static void ReleaseRigidBody(MonoBehaviour grabbing, tntRigidBody grabbed)
    {
        if (!grabbing || !grabbed)
            return;

        if (!(grabbing is tntLink) && !(grabbing is tntRigidBody))
            return;

        tntShapeType grabbingBodyShapeType;
        IntPtr grabbingBodyShape = GetGrabbingBodyShape(grabbing, out grabbingBodyShapeType);
        Transform grabbingTransform = grabbing.transform;
        float grabbingMass = GetGrabbingMass(grabbing);

        // TODO: consider adding a compound collider to the grabbing link if it's not there
        if (grabbingBodyShapeType != tntShapeType.CompoundShape)
        {
            Debug.LogError("This link does not have a compound collider!");
            return;
        }

        IntPtr grabbedShape = grabbed.GetBodyShape();
        if (grabbedShape == IntPtr.Zero)
        {
            Debug.LogError("The child object has no linked collider!");
            return;
        }

        // retrieve grabbing/releasing-to-grabbed frame offset, post-release grabbed body's pos, ori and vel
        Vector3 grabbingToGrabbedFrameOffset, postReleaseGrabbedPos;
        Quaternion postReleaseGrabbedOri;
        Vector3 postReleaseGrabbedLinVel, postReleaseGrabbedAngVel;
        {
            // compute grabbing/releasing link's velocity
            Vector3 parentLinkLinearVel, parentLinkAngularVel;
            ComputeGrabbingVelocity(grabbing, out parentLinkLinearVel, out parentLinkAngularVel);

            // retrieve grabbing/releasing-to-grabbed frame offset
            grabbingToGrabbedFrameOffset = grabbing.transform.rotation * grabbed.transform.localPosition;

            // compute post-release grabbed object's pos, ori and vel
            postReleaseGrabbedPos = grabbed.transform.position;
            postReleaseGrabbedOri = grabbed.transform.rotation;
            postReleaseGrabbedLinVel = parentLinkLinearVel + Vector3.Cross(parentLinkAngularVel * Mathf.Deg2Rad, grabbingToGrabbedFrameOffset);
            postReleaseGrabbedAngVel = parentLinkAngularVel;
        }

        // re-parent (Unity) all former children of the grabbed body
        {
            List<Transform> toRemove = new List<Transform>();
            foreach (var p in changedParenthoodLookUp)
            {
                if (p.Key.parent == grabbingTransform && p.Value == grabbed.transform)
                {
                    p.Key.SetParent(grabbed.transform);
                    toRemove.Add(p.Key);
                }
            }

            foreach (Transform t in toRemove)
                changedParenthoodLookUp.Remove(t);
        }

        // re-parent (Unity) the grabbed body itself
        {
            // store the original parent
            grabbed.transform.SetParent(changedParenthoodLookUp[grabbed.transform]);

            changedParenthoodLookUp.Remove(grabbed.transform);
        }

        // re-create grabbing object's collision geometry
        {
            if (grabbingBodyShape != IntPtr.Zero)
            {
                ColliderFactory.DeleteCollider(grabbingBodyShape, grabbingBodyShapeType);
                SetGrabbingBodyShape(grabbing, IntPtr.Zero, tntShapeType.INVALID);
            }

            float[] masses = null;
            float[] inertias = null;
            tntShapeType shapeType;
            Vector3 transformOffset;
            IntPtr newCompoundShape = ColliderFactory.AddCollider(grabbing, grabbingMass == 0, out shapeType, out masses, out inertias, out transformOffset);

            SetGrabbingBodyShape(grabbing, newCompoundShape, shapeType, true);
            ComputeGrabbingMoIFromCollisionShape(grabbing);
        }

        // re-introduce the grabbed rigid body (--> release it)
        {
            // pass velocity from the releasing to the released
            {
                grabbed.linearVelocity = postReleaseGrabbedLinVel;
                grabbed.angularVelocity = postReleaseGrabbedAngVel;
            }

            // pass transformation from the releasing to the released
            {
                grabbed.position = postReleaseGrabbedPos;
                grabbed.rotation = postReleaseGrabbedOri;
            }

            // reset MoI to make sure it's recomputed by kernel
            {
                grabbed.m_moi.Set(0, 0, 0);
            }

            // enable the body
            {
                // this will effectively add the grabbed body to the world ..
                // .. and, due to how tntRigidNody is implemented, actually recreate it first
                grabbed.enabled = true;
            }
        }
    }

    public static void ReparentChildren(MonoBehaviour grabbing, tntRigidBody grabbed)
    {
        if (!grabbing || !grabbed)
            return;

        if (!(grabbing is tntLink) && !(grabbing is tntRigidBody))
            return;

        Transform grabbingTransform = grabbing.transform;

        // re-parent (Unity) all former children of the grabbed body
        {
            List<Transform> toRemove = new List<Transform>();
            foreach (var p in changedParenthoodLookUp)
            {
                if (p.Key.parent == grabbingTransform && p.Value == grabbed.transform)
                {
                    p.Key.SetParent(grabbed.transform);
                    toRemove.Add(p.Key);
                }
            }

            foreach (Transform t in toRemove)
                changedParenthoodLookUp.Remove(t);
        }
    }

    #region auxiliary methods

    // note: most of those won't be needed ..
    // .. once we have a common base for links and rigid bodies

    private static float GetGrabbingMass(MonoBehaviour grabbing)
    {
        if (!grabbing)
            return 0.0f;

        tntLink grabbingLink = grabbing as tntLink;
        tntRigidBody grabbingBody = grabbing as tntRigidBody;

        if (grabbingLink)
            return grabbingLink.m_mass;
        else if (grabbingBody)
            return grabbingBody.m_mass;

        return 0.0f;
    }

    private static IntPtr GetGrabbingBodyShape(MonoBehaviour grabbing, out tntShapeType shapeType)
    {
        shapeType = tntShapeType.INVALID;

        if (!grabbing)
            return IntPtr.Zero;

        tntLink grabbingLink = grabbing as tntLink;
        tntRigidBody grabbingBody = grabbing as tntRigidBody;

        if (grabbingLink)
        {
            shapeType = grabbingLink.GetShapeType();
            return grabbingLink.GetBodyShape();
        }
        else if (grabbingBody)
        {
            shapeType = grabbingBody.GetShapeType();
            return grabbingBody.GetBodyShape();
        }

        return IntPtr.Zero;
    }

    private static void SetGrabbingBodyShape(
        MonoBehaviour grabbing, IntPtr bodyShape, tntShapeType shapeType, bool toKernelToo = false)
    {
        if (!grabbing)
            return;

        tntLink grabbingLink = grabbing as tntLink;
        tntRigidBody grabbingBody = grabbing as tntRigidBody;

        if (grabbingLink)
        {
            grabbingLink.SetBodyShape(bodyShape);
            grabbingLink.SetShapeType(shapeType);
            if (toKernelToo)
                grabbingLink.SetBodyShapeToKernel(bodyShape);
        }
        else if (grabbingBody)
        {
            grabbingBody.SetBodyShape(bodyShape);
            grabbingBody.SetShapeType(shapeType);
            if (toKernelToo)
                grabbingBody.SetBodyShapeToKernel(bodyShape);
        }
    }

    private static void ComputeGrabbingMoIFromCollisionShape(MonoBehaviour grabbing)
    {
        if (!grabbing)
            return;

        tntLink grabbingLink = grabbing as tntLink;
        tntRigidBody grabbingBody = grabbing as tntRigidBody;

        if (grabbingLink)
        {
            grabbingLink.ComputeMoIFromCollisionShape();

            tntBase theBase = grabbingLink as tntBase;
            theBase = theBase != null ?
                theBase : grabbingLink.transform.parent.GetComponentInChildren<tntBase>();

            List<tntChildLink> childLinks = new List<tntChildLink>();
            for (int i = 0; i < theBase.numLinks(); ++i)
            {
                if (theBase.getChildLink(i).m_parent == grabbingLink)
                    childLinks.Add(theBase.getChildLink(i));
            }

            foreach (tntChildLink childLink in childLinks)
                childLink.ComputeMoIFromCollisionShape();
        }
        else if (grabbingBody)
            grabbingBody.ComputeMoIFromCollisionShape();
    }

    private static void ComputeGrabbingVelocity(MonoBehaviour grabbing, out Vector3 linVel, out Vector3 angVel)
    {
        linVel = Vector3.zero;
        angVel = Vector3.zero;

        if (!grabbing)
            return;

        tntBase grabbingBase = grabbing as tntBase;
        tntChildLink grabbingChildLink = grabbing as tntChildLink;
        tntRigidBody grabbingBody = grabbing as tntRigidBody;

        if (grabbingBase)
        {
            float[] currVel = grabbingBase.CurrentVelocity;
            linVel = new Vector3(currVel[1], currVel[0], currVel[2]);
            angVel = new Vector3(currVel[3], currVel[4], currVel[5]);
        }
        else if (grabbingChildLink)
        {
            grabbingChildLink.ComputeWorldVelocity(out linVel, out angVel, true);
        }
        else if (grabbingBody)
        {
            linVel = grabbingBody.linearVelocity;
            angVel = grabbingBody.angularVelocity;
        }
    }

    #endregion
}
