// optional code is guarded by the following define:
//   #define MIDAS_USE_CUSTOM_TAG_WHEN_SPLITTING_ARTICULATIONS

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PhysicsAPI;

public partial class tntWorld : MonoBehaviour
{

    protected void UpdateBrokenJoints(bool splitOnlyFirst)
    {
        int nLinks = links.Count;
        int linkIndex = 0;
        while (linkIndex < nLinks)
        {
            tntChildLink childLink = links[linkIndex] as tntChildLink;
            if (childLink != null && childLink.m_feedback.m_broken)
            {
                GameObject multibody = childLink.transform.parent.gameObject;

                SplitMultibodyIntoTwo(multibody, childLink.GetIndex(), splitOnlyFirst);

                if (splitOnlyFirst)
                    return;

                linkIndex = 0;
            }
            else
                ++linkIndex;
        }
    }

    private class ChildLinkAndItsStateDesc
    {
        public tntChildLink m_childLink;
        public int m_dofCount, m_dofOffset;
        public int m_posVarCount, m_posVarOffset;
    };

    protected void SplitMultibodyIntoTwo(GameObject multibody, int splitJointIndex, bool cachedLinkVelsCanBeUsed)
    {
        GameObject oldMB = multibody;

        if (oldMB == null)
            return;

        tntBase oldBase = oldMB.GetComponentInChildren<tntBase>();

        if (oldBase == null)
            return;

        int oldNumChildLinks = oldBase.numLinks();

        if (oldNumChildLinks == 0)
            return;

        tntChildLink splitChildLink = oldBase.getChildLink(splitJointIndex);
        GameObject splitChildLinkGameObject = splitChildLink.gameObject;

        // find all multibody constraints on the old mb so that we can update them after splitting
        // note: we don't seem to support any mb constraints other than p2p
        tntArticulationP2PConstraint[] p2pMbConstraints = FindObjectsOfType<tntArticulationP2PConstraint>();
        tntArticulationP2PConstraint[] p2pMbConstraintsOfTheOldMb = System.Array.FindAll<tntArticulationP2PConstraint>(p2pMbConstraints, p2p =>
            p2p.enabled && (
            (p2p.m_linkA != null && p2p.m_linkA.GetBase() == oldBase.GetBase()) ||
            (p2p.m_linkB != null && p2p.m_linkB.GetBase() == oldBase.GetBase()))
            );
        tntArticulationP2PConstraint[] p2pMbConstraintsOfSplitLink = System.Array.FindAll<tntArticulationP2PConstraint>(p2pMbConstraintsOfTheOldMb, p2p =>
            p2p.m_linkA == splitChildLink ||
            p2p.m_linkB == splitChildLink
            );

        // read the full raw state of the old multibody
        Vector3 oldBaseRawPos, oldBaseRawLinVel, oldBaseRawAngVel;
        Quaternion oldBaseRawOri;
        float[] oldStateRawPosVars, oldStateRawVelVars;
        oldBase.ReadFullRawState(out oldBaseRawPos, out oldBaseRawOri, out oldBaseRawLinVel, out oldBaseRawAngVel, out oldStateRawPosVars, out oldStateRawVelVars);

        // divide old base's links into those which will stay in it/those which will be moved..
        // .. and gather information about their state data location inside raw state vectors
        List<ChildLinkAndItsStateDesc> linksToStay = new List<ChildLinkAndItsStateDesc>();
        List<ChildLinkAndItsStateDesc> linksToBeMoved = new List<ChildLinkAndItsStateDesc>();
        int dofOffset = 0, posVarOffset = 0;
        for (int linkIdx = 0; linkIdx < oldNumChildLinks; ++linkIdx)
        {
            tntChildLink childLink = oldBase.getChildLink(linkIdx);

            int posVarCount = childLink.GetRawPosVariablesCount();
            int dofCount = childLink.GetDofCount();

            List<ChildLinkAndItsStateDesc> tempListAlias =
                IsChildLinkInSubtree(oldBase, childLink, splitChildLink) ? linksToBeMoved : linksToStay;
            tempListAlias.Add(new ChildLinkAndItsStateDesc
            {
                m_childLink = childLink,
                m_dofCount = dofCount,
                m_dofOffset = dofOffset,
                m_posVarCount = posVarCount,
                m_posVarOffset = posVarOffset
            });

            dofOffset += dofCount;
            posVarOffset += posVarCount;
        }

        // get/compute the world space vectorial linear and angular velocity of the split child link (i.e. new base)
        Vector3 newBaseLinVel, newBaseAngVel;
        splitChildLink.ComputeWorldVelocity(out newBaseLinVel, out newBaseAngVel, cachedLinkVelsCanBeUsed);
        Vector3 newBaseRawLinVel = newBaseLinVel;
        Vector3 newBaseRawAngVel = newBaseAngVel * Mathf.Deg2Rad;    // the only difference between vectorial raw and non-raw velocities is that the former is in rad/s

        // remove native engine side constraints and multibody
        foreach (tntArticulationP2PConstraint p2p in p2pMbConstraintsOfTheOldMb)
            p2p.RemoveConstraint();

        // Remove collision listeners for tntBase and all tntChildLinks
        oldBase.RemoveAllCollisionListeners();
        for (int i = 0; i < oldBase.numLinks(); i++) oldBase.getChildLink(i).RemoveAllCollisionListeners();

        RemoveArticulationBase(oldBase);
        tntEntityAndJointFactory.DestroyArticulationBase(oldBase);

        // create the new multibody game object
        GameObject newMB = new GameObject(oldMB.name + "-split-at-" + splitChildLink.name);
        // reuse the split link's game object for the new base go
        GameObject newBaseGameObject = splitChildLinkGameObject;
        newBaseGameObject.name = splitChildLinkGameObject.name + "-based";
        // disable newBaseGameObject so that the incoming AddComponent of a tntBase..
        // ..does not launch tntBase's Awake which would create native multibody
        newBaseGameObject.SetActive(false);
        // instantiate the new base component
        tntBase newBase = newBaseGameObject.AddComponent<tntBase>();

        // copy settings from the link that's to become a base now
        // we set those directly even if there are setters since we don't want to talk to the engine, just configure Unity side
        {
            newBase.m_collidable = splitChildLink.m_collidable;
            newBase.m_mass = splitChildLink.m_mass;
            newBase.m_moi = splitChildLink.m_moi;
            newBase.m_material = splitChildLink.m_material;
            newBase.m_mark = splitChildLink.m_mark;
            newBase.m_drag = splitChildLink.m_drag;
            newBase.m_angularDrag = splitChildLink.m_angularDrag;
            newBase.transform.position = splitChildLink.transform.position;
            newBase.transform.rotation = splitChildLink.transform.rotation;
            //
            newBase.m_simulationFrequencyMultiplier = oldBase.m_simulationFrequencyMultiplier;
            newBase.RequiredSolverFidelityIndex = oldBase.RequiredSolverFidelityIndex;
            newBase.m_useWorldSolverFidelityIndex = oldBase.m_useWorldSolverFidelityIndex;
        }

        // move base from old to new mb
        {
            // assign new base's parent
            newBaseGameObject.transform.parent = newMB.transform;

            // re-assign all p2p constraints from the soon-to-be-destroyed link component to new base component
            foreach (tntArticulationP2PConstraint p2p in p2pMbConstraintsOfSplitLink)
            {
                p2p.m_linkA = p2p.m_linkA == splitChildLink ? newBase : p2p.m_linkA;
                p2p.m_linkB = p2p.m_linkB == splitChildLink ? newBase : p2p.m_linkB;
            }

            // disable and mark for destruction the surplus old tntChildLink component from the new base
            splitChildLink.enabled = false;
            Destroy(splitChildLink);
        }

        // invalidate indices of old mb links
        foreach (ChildLinkAndItsStateDesc childLinkDesc in linksToStay)
            childLinkDesc.m_childLink.InvalidateIndex();

        // move child links from old to new mb and invalidate their indices
        foreach (ChildLinkAndItsStateDesc childLinkAndDesc in linksToBeMoved)
        {
            tntChildLink childLink = childLinkAndDesc.m_childLink;
            childLink.InvalidateIndex();
            childLink.gameObject.transform.parent = newMB.transform;
            if (childLink.m_parent == splitChildLink)
                childLink.m_parent = newBase;
            if (m_removeMotorsAndLimitsOfBrokenOffLinks)
            {
                childLink.RemoveLimit(-1);
                childLink.RemoveMotor(-1);
            }
        }

        //Put the new multi body into the battleground scene so that it can be unloaded along with the playground scene when needed
        //UnityEngine.SceneManagement.Scene battleScene = UnityEngine.SceneManagement.SceneManager.GetSceneAt(1);
        //if (battleScene != null) { UnityEngine.SceneManagement.SceneManager.MoveGameObjectToScene(newMB, battleScene); }

        // recreate the native engine side of the old multibody
        tntEntityAndJointFactory.CreateArticulationBase(oldBase);
        AddArticulationBase(oldBase);
        if (oldBase.m_IsKinematic)
            oldBase.ForceSetKinematic();

        // Readd collision listeners
        oldBase.UpdateCollisionListeners();
        for (int i = 0; i < oldBase.numLinks(); i++) oldBase.getChildLink(i).UpdateCollisionListeners();

        // recreate old multibody's state vector
        float[] oldBaseNewQs = new float[oldBase.GetTotalRawPosVarCountExcludingBase()];
        float[] oldBaseNewQds = new float[oldBase.GetTotalDofCountExcludingBase()];
        dofOffset = 0;
        posVarOffset = 0;
        for (int linkIndex = 0; linkIndex < oldBase.numLinks(); ++linkIndex)
        {
            tntChildLink childLink = oldBase.getChildLink(linkIndex);
            ChildLinkAndItsStateDesc childLinkAndDesc = linksToStay.Find(desc => desc.m_childLink == childLink);

            Array.Copy(oldStateRawPosVars, childLinkAndDesc.m_posVarOffset, oldBaseNewQs, posVarOffset, childLinkAndDesc.m_posVarCount);
            Array.Copy(oldStateRawVelVars, childLinkAndDesc.m_dofOffset, oldBaseNewQds, dofOffset, childLinkAndDesc.m_dofCount);

            posVarOffset += childLinkAndDesc.m_posVarCount;
            dofOffset += childLinkAndDesc.m_dofCount;
        }
        oldBase.SetFullRawState(oldBaseRawPos, oldBaseRawOri, oldBaseRawLinVel, oldBaseRawAngVel, oldBaseNewQs, oldBaseNewQds, true);
        oldBase.SynchronizeTransformsWithKernel();

#if MIDAS_USE_ACE_WHEN_SPLITTING_ARTICULATIONS
        // FIXME: animator reset should be done in a more anonymous way (event-driven probably) but we'd need some refactoring..
        // ..to achieve that so it needs to be postponed
        if (oldBase.transform.parent)
        {
            HumanoidAnimator animator = oldBase.transform.parent.GetComponentInChildren<HumanoidAnimator>();
            if (animator && animator.enabled)
                animator.Reset(false);
        }
#endif

        // create the native engine side of the new multibody
        newBaseGameObject.SetActive(true);
        AddArticulationBase(newBase);
        // create new multibody's state vector
        float[] newBaseNewQs = new float[newBase.GetTotalRawPosVarCountExcludingBase()];
        float[] newBaseNewQds = new float[newBase.GetTotalDofCountExcludingBase()];
        dofOffset = 0;
        posVarOffset = 0;
        for (int linkIndex = 0; linkIndex < newBase.numLinks(); ++linkIndex)
        {
            tntChildLink childLink = newBase.getChildLink(linkIndex);
            ChildLinkAndItsStateDesc childLinkAndDesc = linksToBeMoved.Find(desc => desc.m_childLink == childLink);

            Array.Copy(oldStateRawPosVars, childLinkAndDesc.m_posVarOffset, newBaseNewQs, posVarOffset, childLinkAndDesc.m_posVarCount);
            Array.Copy(oldStateRawVelVars, childLinkAndDesc.m_dofOffset, newBaseNewQds, dofOffset, childLinkAndDesc.m_dofCount);

            posVarOffset += childLinkAndDesc.m_posVarCount;
            dofOffset += childLinkAndDesc.m_dofCount;
        }
        newBase.SetFullRawState(newBase.transform.position, newBase.transform.rotation, newBaseRawLinVel, newBaseRawAngVel, newBaseNewQs, newBaseNewQds, true);
        newBase.SynchronizeTransformsWithKernel();

        // recreate native engine side of mb constraints
        if (p2pMbConstraintsOfTheOldMb != null)
            foreach (tntArticulationP2PConstraint p2p in p2pMbConstraintsOfTheOldMb)
                p2p.AddConstraint();

#if MIDAS_USE_CUSTOM_TAG_WHEN_SPLITTING_ARTICULATIONS
        //Give a tag, make it capable of being identified later for various operation
        foreach (Transform t in newMB.transform)
        {
            CustomTag ct = t.gameObject.GetComponent<CustomTag>();
            if (ct == null) { ct = t.gameObject.AddComponent<CustomTag>(); }
            ct.m_tagName = "Detached Joint";
        }
#endif

        return;
    }

    /**
     * Checks if treeLink child link is a part of the subtree rooted at subtreeRoot link
     * @param baseLink tntBase of the articulation checked
     * @param treeLink child link to be checked
     * @param subtreeRoot root link of the subtree to be checked
     * @return true if treeLink is in the subtree rooted at subtreeRoot; false otherwise
     */
    public static bool IsChildLinkInSubtree(tntBase baseLink, tntChildLink treeLink, tntLink subtreeRoot)
    {
        if (treeLink.GetIndex() < subtreeRoot.GetIndex())       // guaranteed by the way links indices are assigned in tntBase.AddArticulationBase
            return false;

        tntLink treeRoot = baseLink;

        while (treeLink && treeLink.m_parent && treeLink.m_parent != subtreeRoot && treeLink.m_parent != treeRoot)
            treeLink = treeLink.m_parent as tntChildLink;

        return treeLink && treeLink.m_parent == subtreeRoot;
    }

    /**
     * Adds a point-to-point articulation constraint between a link-local point and a world location
     * @param hitTntBase native pointer to a base of the multibody
     * @param hitTntLinkIndex index of a link we want to add constraint to
     * @param pivotA link-local location
     * @param hitLocation world location
     * @param force maximal impulse this constraint is allowed to apply
     * @return native pointer to the underlying constraint
     */
    public unsafe IntPtr AddP2PLinkAsMouseSpring(IntPtr hitTntBase, int hitTntLinkIndex,
                                                 Vector3 pivotA, Vector3 hitLocation, float force)
    {
        return TNT.apAddArticulationRigidBodyP2PLink(
            dynamicsWorld,
            hitTntBase, hitTntLinkIndex,
            IntPtr.Zero,
            &pivotA, &hitLocation, force, 1e+38f,
            null
           );
    }

    /**
     * Adds a point-to-point rigid-body constraint between a body-local point and a world location
     * @param hitBody native pointer to the rigid body
     * @param pivotA body-local location
     * @param force maximal impulse this constraint is allowed to apply
     * @return native pointer to the underlying constraint
     */
    public unsafe IntPtr AddP2PJointAsMouseSpring(IntPtr hitBody,
                                                 Vector3 pivotA, float force)
    {
        IntPtr p2p = TNT.apCreateP2PConstraint(hitBody, IntPtr.Zero, &pivotA, null, force, -1, null);
        TNT.apAddConstraint(dynamicsWorld, p2p, true);
        return p2p;
    }

    public unsafe IntPtr AddFixedJointAsMouseSpring(IntPtr hitBody,
                                                     Vector3 pivotB, float force)
    {
        IntPtr pFixed = TNT.apCreateFixedConstraint(IntPtr.Zero, hitBody, null, &pivotB, force, -1, null);
        TNT.apAddConstraint(dynamicsWorld, pFixed, true);
        return pFixed;
    }
}