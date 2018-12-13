using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
using PhysicsAPI;

/**
 * @brief Class describing an articulation base body/link. It also represents the articulation as a whole
 */

public class tntBase : tntLink
{
	public bool m_enableSelfCollision;
    public bool m_highDefIntegrator;
    public bool m_useGlobalVel;
        /// Changes the frequency this Articulation uses for simulation.
        /// 
        /// Use this for simulations requiring higher accuracy than the rest of the scene.
        /// 4 is a good value for high-frequency simulation.
    public int m_simulationFrequencyMultiplier;
    [SerializeField]
    public bool m_useWorldSolverFidelityIndex = true;
    [SerializeField]
    protected SolverFidelityIndex m_solverFidelityIndex = SolverFidelityIndex.Default;
    public float m_linearDamping;
    public float m_angularDamping;
    public float m_maxAppliedImpulse;
    public float m_maxCoordinateVelocity;
	
    internal Dictionary<string, tntChildLink> m_nameToLink;
	internal List<tntChildLink> m_childLinks;
    // TODO: consider having a 'created' flag like this for all entities ..
    // .. note: tntBase is special though - it represents itself AND all child links
    internal bool m_articulationCreated = false;

    public bool m_IsKinematic;

    public bool m_isOutOfSim = false;

    /**
     * Base body constructor, initializes members to their defaults
     */
    tntBase() : base(Type.Base)
    {
        m_index = -1;
        m_enableSelfCollision = true;
        m_highDefIntegrator = false;
        m_simulationFrequencyMultiplier = 1;
        m_useGlobalVel = false;
        m_linearDamping = 0.04f;
        m_angularDamping = 0.04f;
        m_maxCoordinateVelocity = 100.0f;
        m_IsKinematic = false;

        // Initialize velocity sensor array
        m_currentVelocity = new float[] { 0f, 0f, 0f, 0f, 0f, 0f };
    }

    /**
     * Checks if the base is static, i.e. immobile
     * @return true if static, false otherwise
     */
    public bool IsStatic() {  return m_mass == 0;  }
    /**
     * Maps the passed in link name to a corresponding index
     * @param name link's name
     * @return link index
     */
    public int NameToIndex(string name) { return m_nameToLink.ContainsKey(name) ? m_nameToLink[name].GetIndex() : -2; }
    /**
     * Returns the number of links in this articulation
     * @return number of links in this articulation
     */
    public int numLinks() { return m_childLinks != null ? m_childLinks.Count : 0; }
    /**
     * Returns articulation link corresponding to number the passed in index parameter
     * @param index link's index
     */
	public tntChildLink getChildLink(int index) { return m_childLinks[index]; }

	public override void Awake()
	{
		base.Awake();
		if (m_world == null)
		{
			m_world = GameObject.FindObjectOfType<tntWorld>();
		}
		m_world.EnsureDynamicWorld();
        tntEntityAndJointFactory.CreateArticulationBase(this);
        SetRequiredSolverFidelityIndexToKernel(m_solverFidelityIndex);

        UpdateCollisionListeners();
        if (m_childLinks != null) foreach (var l in m_childLinks) l.UpdateCollisionListeners();
    }

    void OnEnable()
    {
        m_world.AddArticulationBase(this);
        if (m_IsKinematic)
            ForceSetKinematic();
    }

    void OnDisable() { m_world.RemoveArticulationBase(this); }

    void OnDestroy()
	{
        // Remove collision listeners for tntBase and all tntChildLinks
        RemoveAllCollisionListeners();
        if (m_childLinks != null) foreach (var l in m_childLinks) l.RemoveAllCollisionListeners();

        tntEntityAndJointFactory.DestroyArticulationBase(this);
    }

	void OnApplicationQuit()
	{
        // Remove collision listeners for tntBase and all tntChildLinks
        RemoveAllCollisionListeners();
        if (m_childLinks != null) foreach (var l in m_childLinks) l.RemoveAllCollisionListeners();

        m_world.RemoveArticulationBase(this);
        tntEntityAndJointFactory.DestroyArticulationBase(this);
    }

    protected int FindLinkTopologyDepth(tntChildLink link)
    {
        int depth = 0;

        tntLink tmpLink = link;
        while (tmpLink && tmpLink as tntChildLink)
        {
            tmpLink = (tmpLink as tntChildLink).m_parent;
            depth++;
        }

        depth = tmpLink ? depth : Int32.MaxValue;

        return depth;
    }

    protected int CompareChildLinksDepth(tntChildLink link1, tntChildLink link2)
    {
        int depth1 = FindLinkTopologyDepth(link1);
        int depth2 = FindLinkTopologyDepth(link2);

        return depth1 - depth2;
    }

    protected int CompareChildLinksForIndexing(tntChildLink link1, tntChildLink link2)
    {
        int depthCmp = CompareChildLinksDepth(link1, link2);
        if (depthCmp == 0)
            return String.Compare(link1.name, link2.name);      // if equal depth, then sort by name (to minimize the impact of Unity)
        return depthCmp;
    }

    protected static bool AreChildLinkIndicesValidAndUnique(tntChildLink[] childLinks)
    {
        if (childLinks == null)
            return true;

        foreach (var childLink in childLinks)
        {
            if (childLink == null || !childLink.IsAssigned() || childLink.m_parent == null)
                return false;

            bool isIndexValid =
                    childLink.GetIndex() < childLinks.Length
                &&  childLink.GetIndex() >= 0
                &&  childLink.GetIndex() > childLink.m_parent.GetIndex();

            bool isIndexUnique = !isIndexValid
                ? true      // no point checking if unique if it's invalid (minor opt)
                : Array.Find<tntChildLink>(childLinks, c => c != childLink && c.GetIndex() == childLink.GetIndex()) == null;

            if (!isIndexValid || !isIndexUnique)
            {
                Debug.LogWarning(
                    string.Format("Link's '{0}' index ({1}) is {2}, it's probably saved this way." +
                    " All link indices will be recomputed for the current run." +
                    " You can do it permanently yourself by using the 'Recompute link indices' button in tntBase inspector when in the Edit mode.",
                    childLink.name, childLink.GetIndex(), (isIndexValid ? "not unique" : "invalid")));
                return false;
            }
        }

        return true;
    }

    public override void AssignValidChildLinkIndices(bool forceUpdate = false)
    {
        // depth-first index assignment
        tntChildLink[] allLinks = transform.parent.GetComponentsInChildren<tntChildLink>(false);
        tntChildLink[] links = System.Array.FindAll<tntChildLink>(allLinks, comp => comp.enabled && comp.m_parent != null);

        if (!forceUpdate && tntBase.AreChildLinkIndicesValidAndUnique(links))
            return;

        System.Array.Sort<tntChildLink>(links, (link1, link2) => CompareChildLinksForIndexing(link1, link2));

        int nextFreeIndex = 0;
        foreach (tntChildLink childLink in links)
            childLink.SetIndex(nextFreeIndex++);
    }

    /**
     * Sets the base body mass
     * @param mass the mass to set
     * @remark Setting zero mass makes the base immobile
     */
    public override void SetMass(float newMass)
	{
        if (newMass != 0)
        {
            // previously anchored TNT base is no de-anchored
            MeshCollider meshCollider = GetComponent<MeshCollider>();
            if (meshCollider != null && !meshCollider.convex)
            {
                Debug.LogError("Concave mesh cannot be unanhcored now");
                return;
            }
        }
		if (m_base != IntPtr.Zero)
			TNT.apSetBaseMass(m_base, newMass);

		m_mass = newMass;
	}

    public override Vector3 linearVelocity
    {
        get { return new Vector3(CurrentVelocity[0], CurrentVelocity[1], CurrentVelocity[2]); }
        set
        {
            // This can also be run from inspector before play
            if (m_base != IntPtr.Zero && !m_IsKinematic && !IsStatic())
            {
                for (int i = 0; i < 3; i++) CurrentVelocity[i] = value[i];

                TNT.apActivateKinematicArticulation(m_base);
                Vector3 basePos, baseVel, baseAngVel;
                Quaternion baseOri;
                float[] qs, qds;
                ReadFullRawState(out basePos, out baseOri, out baseVel, out baseAngVel, out qs, out qds);
                SetFullRawState(basePos, baseOri, value, baseAngVel, qs, qds, true);
            }
        }
    }

    /// Current angular velocity of this tntBaseLink
    public override Vector3 angularVelocity
    {
        get { return new Vector3(CurrentVelocity[3], CurrentVelocity[4], CurrentVelocity[5]) * Mathf.Rad2Deg; }
        set
        {
            // This can also be run from inspector before play
            if (m_base != IntPtr.Zero && !m_IsKinematic && !IsStatic())
            {
                for (int i = 0; i < 3; i++) CurrentVelocity[3 + i] = value[i] * Mathf.Deg2Rad;

                TNT.apActivateKinematicArticulation(m_base);
                Vector3 basePos, baseVel, baseAngVel;
                Quaternion baseOri;
                float[] qs, qds;
                ReadFullRawState(out basePos, out baseOri, out baseVel, out baseAngVel, out qs, out qds);
                SetFullRawState(basePos, baseOri, baseVel, value * Mathf.Deg2Rad, qs, qds, true);
            }
        }
    }
    /**
     * Returns the current linear velocity vector of the base body
     * @return linear velocity vector of the base body
     */
    public Vector3 GetLinearVelocity() { return linearVelocity; }
    
    /**
     * Returns the current linear angular vector of the base body
     * @return linear angular vector of the base body
     */
    public Vector3 GetAngularVelocity() { return angularVelocity; }

     /**
     * Sets layer ID for the base body
     * @param layerID layer ID to be set
     * @param resetCollisionCache Force deletion of existing collision caches (needed to update filtering on e.g. resting contact) and activate the object
     * @remark Child links of this base are not influenced
     * @remark APE respects Unity's layer collision matrix so this operation may have impact on collision detection
     * @see http://docs.unity3d.com/Manual/LayerBasedCollision.html
     */
    public override void SetLayerID(int layerID, bool resetCollisionCache = false)
    {
		gameObject.layer = layerID;
        TNT.apSetArticulationLayerID(m_base, layerID);
        if (resetCollisionCache)
        {
            TNT.apResetCollisionCacheForArticulation(GetWorld(), m_base);
        }
    }

    /**
     * Enables or disables collision detection for the base body
     * @param collide true enables collision detection, false disables it
     * @remark Child links of this base are not influenced
     * @see tntChildLink::SetCollidable
     */
    public override void SetCollidable(bool collide, bool resetCollisionCache = false)
    {
        if (!collide)
        {
            TNT.apSetArticulationLayerID(m_base, -1);
        }
        else
        {
			TNT.apSetArticulationLayerID(m_base, gameObject.layer);
        }
        m_collidable = collide;
        if (resetCollisionCache)
        {
            TNT.apResetCollisionCacheForArticulation(GetWorld(), m_base);
        }
    }

    /**
     * Sets this base body to be kinematic (i.e. mobile but not affected by physics sim) or not
     * @param kinematic specifies if the body is to become kinematic or not, true to make it kinematic, false otherwise
     * @remark This operation is ignored if it does not change internal settings
     * @see tntBase::ForceSetKinematic
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
     * Sets this base body to be kinematic (i.e. mobile but not affected by physics sim)
     * @see tntBase::SetKinematic
     */
    public void ForceSetKinematic()
    {
		if (m_world != null && m_added)
		{
			if (m_IsKinematic)
			{
				TNT.apSetArticulationKinematic(GetBase(), m_IsKinematic, 0);
			}
			else
			{
				TNT.apSetArticulationKinematic(GetBase(), m_IsKinematic, m_mass);
			}
		}
    }

    /**
     * Checks if this base body is kinematic (i.e. mobile but not affected by physics sim)
     * @return true if this body is kinematic, false otherwise
     */
    public bool GetKinematic()
    {
        return m_IsKinematic;
    }

    public void SetKinematicFromTransfrom()
    {
        if (m_IsKinematic)
        {
            m_transform.position = transform.position;
            m_transform.rotation = transform.rotation;
        }
    }

    public void SetKinematicPosRot(Vector3 pos, Quaternion qt)
    {
        if (m_IsKinematic)
        {
            m_transform.position = pos;
            m_transform.rotation = qt;
        }
    }

    public bool ContainsLink(tntLink theLink, bool amongChildrenOnly)
    {
        return (!amongChildrenOnly && theLink == this) || (m_childLinks.Find(link => link == theLink) != null);
    }

    public tntReducedState CreateCurrentReducedState(tntReducedState templateState)
    {
        tntReducedState currState = ScriptableObject.CreateInstance<tntReducedState>();
        currState.NumOfChildLinks = this.numLinks();
        currState.AllocArrays();

        // copy global settings from the template
        if (templateState)
        {
            currState.m_values[0] = templateState.m_values[0];
            currState.m_values[1] = templateState.m_values[1];
            currState.m_values[2] = templateState.m_values[2];
            currState.m_values[3] = templateState.m_values[3];
        }

        FillReducedStateVectorFromCurrentState(currState, 0);
        int offset = 17;

        for (int i = 0; i < currState.NumOfChildLinks; ++i)
        {
            tntChildLink childLink = this.getChildLink(i);
            currState.m_names[i] = childLink.name;
            currState.m_editAsEulers[i] = false;

            childLink.FillReducedStateVectorFromCurrentState(currState, offset);
            offset += 7;
            
        }
        return currState;
    }

    public tntReducedState CreateDefaultReducedState()
    {
        tntReducedState asset = ScriptableObject.CreateInstance<tntReducedState>();
        asset.NumOfChildLinks = this.numLinks();
        asset.AllocArrays();
        for (int i = 0; i < asset.NumOfChildLinks; ++i)
        {
            tntChildLink childLink = this.getChildLink(i);
            asset.m_names[i] = childLink.name;
            asset.m_editAsEulers[i] = false;

			// Set the initial pose to match the underlying articulation
			Quaternion parentRot = getChildLink(i).m_parent.transform.rotation;
			Quaternion childRot = getChildLink(i).transform.rotation;
			asset.SetJointOrientationToQuaternion(i, Quaternion.Inverse(parentRot) * childRot);
        }
        return asset;
    }

    /**
     * Returns the total number of this articulation's raw position variables base's positional state
     * @return total number of this articulation's raw position variables base's positional state
     * @remark RAW position uses internal APE's format which can use different units or different number of parameters compared to Unity
     */
    public int GetTotalRawPosVarCountExcludingBase()
    {
        if (m_base == IntPtr.Zero)
            return 0;

        return TNT.apGetArticulationTotalRawPosVarCountExcludingBase(m_base);
    }

    /**
     * Returns the total number of this articulation's degrees of freedom excluding base's velocity
     * @return total number of this articulation's degrees of freedom excluding base's DoFs
     * @remark RAW velocity uses internal APE's format which can use different units or different number of parameters compared to Unity
     */
    public int GetTotalDofCountExcludingBase()
    {
        if (m_base == IntPtr.Zero)
            return 0;

        return TNT.apGetArticulationTotalDofCountExcludingBase(m_base);
    }

    public float GetTotalMass()
    {
        float sum = 0;
		if (transform.parent == null)
			return m_mass;
        foreach (tntEntity ent in transform.parent.GetComponentsInChildren<tntEntity>())
        {
            if (ent as tntChildLink)
                sum += ent.m_mass;
        }
        sum += m_mass;
        return sum;
    }

    public void SetFullRawState(Vector3 basePos, Quaternion baseOri, Vector3 baseVel, Vector3 baseAngVel,
                                float[] qs, float[] qds, bool updateArticulationTransformAndSenors)
    {
        if (m_base == IntPtr.Zero)
            return;

        unsafe
        {
            fixed (float* pQs = qs)
            fixed (float* pQds = qds)
            {
                TNT.apSetArticulationFullRawState(m_base, &basePos, &baseOri,
                                                    &baseVel, &baseAngVel,
                                                    pQs, pQds, updateArticulationTransformAndSenors);
            }
        }
    }

    public void ReadFullRawState(out Vector3 basePos, out Quaternion baseOri, out Vector3 baseVel,
                                out Vector3 baseAngVel,out float[] qs, out float[] qds)
    {
        basePos = new Vector3();
        baseOri = new Quaternion();
        baseVel = new Vector3();
        baseAngVel = new Vector3();
        qs = null;
        qds = null;

        if (m_base == IntPtr.Zero)
            return;

        int totalPosVarCount = GetTotalRawPosVarCountExcludingBase();
        int totalDofCount = GetTotalDofCountExcludingBase();

        qs = new float[totalPosVarCount];
        qds = new float[totalDofCount];

        unsafe
        {
            fixed (Vector3* pBasePos = &basePos)
            fixed (Quaternion* pBaseOri = &baseOri)
            fixed (Vector3* pBaseVel = &baseVel)
            fixed (Vector3* pBaseAngVel = &baseAngVel)
            fixed (float* pQs = qs)
            fixed (float* pQds = qds)
            {
                TNT.apGetArticulationFullRawState(m_base, pBasePos, pBaseOri,
                                                   pBaseVel, pBaseAngVel,
                                                   pQs, pQds);
            }
        }
    }

    /**
     * Synchronizes this articulation's links' positions and rotations between Unity and dynamic world internals
     */
    public void SynchronizeTransformsWithKernel()
    {
        PushNeededTransformsToKernel();
        PullNeededTransformsFromKernel();
    }

    public void PushNeededTransformsToKernel()
    {
        if (GetKinematic())
        {
            PushTransformToKernel();
        }
    }

    public void PullNeededTransformsFromKernel()
    {
        if (!GetKinematic())
        {
            PullTransformFromKernel();
        }

        PullChildLinksTransformsFromKernel();
    }

    private void PushTransformToKernel()
    {
        Transform activeTransform = m_transform ? m_transform : transform;

        if (m_position != activeTransform.position || m_rotation != activeTransform.rotation)
        {
            m_position = activeTransform.position;
            m_rotation = activeTransform.rotation;
        }
    }

    private void PullTransformFromKernel()
    {
        Transform activeTransform = m_transform ? m_transform : transform;

        activeTransform.position = m_position;
        activeTransform.rotation = m_rotation;
    }

    private void PullChildLinksTransformsFromKernel()
    {
        if (m_childLinks != null)
            foreach (tntChildLink link in m_childLinks)
            {
                Transform activeTransform = link.m_transform ? link.m_transform : link.transform;

                activeTransform.position = link.m_position;
                activeTransform.rotation = link.m_rotation;
            }
    }

    /**
     * Checks if treeLink child link is a part of the subtree rooted at subtreeRoot link
     * @param treeLink child link to be checked
     * @param subtreeRoot root link of the subtree to be checked
     * @return true if treeLink is in the subtree rooted at subtreeRoot; false otherwise
     */
    public bool IsChildLinkInSubtree(tntChildLink treeLink, tntLink subtreeRoot)
    {
        if (treeLink.GetIndex() < subtreeRoot.GetIndex())       // guaranteed by the way links indices are assigned in tntBase.AddArticulationBase
            return false;

        tntLink treeRoot = this;

        while (treeLink && treeLink.m_parent && treeLink.m_parent != subtreeRoot && treeLink.m_parent != treeRoot)
            treeLink = treeLink.m_parent as tntChildLink;
            
        return treeLink && treeLink.m_parent == subtreeRoot;
    }

    /**
     * Fills reducedState state vector with data pertaining to this base
     * @param reducedState state vector to be filled
     * @param offset offset into the vector at which this base is to put its data
     */
    public override void FillReducedStateVectorFromCurrentState(tntReducedState reducedState, int offset)
    {
        reducedState.m_values[offset + 4] = m_position.x;
        reducedState.m_values[offset + 5] = m_position.y;
        reducedState.m_values[offset + 6] = m_position.z;
        //
        reducedState.m_values[offset + 7] = m_rotation.x;
        reducedState.m_values[offset + 8] = m_rotation.y;
        reducedState.m_values[offset + 9] = m_rotation.z;
        reducedState.m_values[offset + 10] = m_rotation.w;
        //
        reducedState.m_values[offset + 11] = CurrentVelocity[0];
        reducedState.m_values[offset + 12] = CurrentVelocity[1];
        reducedState.m_values[offset + 13] = CurrentVelocity[2];
        reducedState.m_values[offset + 14] = CurrentVelocity[3] * Mathf.Deg2Rad;
        reducedState.m_values[offset + 15] = CurrentVelocity[4] * Mathf.Deg2Rad;
        reducedState.m_values[offset + 16] = CurrentVelocity[5] * Mathf.Deg2Rad;
    }

    public override void SetBodyShapeToKernel(IntPtr shape)
    {
        if (m_base != IntPtr.Zero)
            TNT.apSetBaseCollisionShape(m_base, shape);
    }

    public override void ComputeMoIFromCollisionShape()
    {
        if (m_base != IntPtr.Zero)
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
                    TNT.apComputeBaseMoIFromCollisionShape(
                        m_base, pMasses, pInertias,
                        pMass, pMoI);
                }
            }
        }
    }

    public Vector3 ComputeArticulationCoMPosition()
    {
        Vector3 comPos = Vector3.zero;
        if (m_base != IntPtr.Zero)
        {
            unsafe
            {
                TNT.apComputeArticulationCoMPosition(m_base, &comPos);
            }
        }

        return comPos;
    }

    public Vector3 ComputeArticulationCoMVelocity()
    {
        Vector3 comVel = Vector3.zero;
        if (m_base != IntPtr.Zero)
        {
            unsafe
            {
                TNT.apComputeArticulationCoMVelocity(m_base, &comVel);
            }
        }

        return comVel;
    }

    // This is for temporarily disable the joint limits without affecting the configuration
    // so that those limits can be resumed when the disabling period is over
    public void SuspendJointLimits()
    {
        for (int i = 0; i < m_childLinks.Count; ++i)
        {
            tntChildLink link = m_childLinks[i];
            for (int j = 0; j < link.m_dofData.Length; ++j)
            {
                tntDofData dof = link.m_dofData[j];
                if (dof.m_useLimit)
                    link.RemoveLimit(j);
            }
        }
    }

    // Resume the previously temporarily disable the joint limits based on the configuration
    public void ResumeJointLimits()
    {
        for (int i = 0; i < m_childLinks.Count; ++i)
        {
            tntChildLink link = m_childLinks[i];
            for (int j = 0; j < link.m_dofData.Length; ++j)
            {
                tntDofData dof = link.m_dofData[j];
                if (dof.m_useLimit)
                    link.AddLimits(j, dof.m_limitLow, dof.m_limitHigh, dof.m_maxLimitForce);
            }
        }
    }

    // This is for temporarily disable the joint motors without affecting the configuration
    // so that those motors can be resumed when the disabling period is over
    public void SuspendMotors()
    {
        for (int i = 0; i < m_childLinks.Count; ++i)
        {
            tntChildLink link = m_childLinks[i];
            for (int j = 0; j < link.m_dofData.Length; ++j)
            {
                tntDofData dof = link.m_dofData[j];
                if (dof.m_useMotor)
                    link.RemoveMotor(j);
            }
        }
    }

    public void ResumeMotors()
    {
        for (int i = 0; i < m_childLinks.Count; ++i)
        {
            tntChildLink link = m_childLinks[i];
            for (int j = 0; j < link.m_dofData.Length; ++j)
            {
                tntDofData dofData = link.m_dofData[j];
                if (dofData.m_useMotor)
                {
                    link.AddMotor(j, dofData.m_isPositionMotor, dofData.m_desiredVelocity,
                         dofData.m_desiredPosition, dofData.m_maxMotorForce, dofData.m_positionLockThreshold, dofData.m_useAutomaticPositionLockThreshold);
                }
            }
        }
    }

    public SolverFidelityIndex RequiredSolverFidelityIndex
    {
        get { return m_solverFidelityIndex; }
        set
        {
            if (value != m_solverFidelityIndex)
            {
                m_solverFidelityIndex = value;
                SetRequiredSolverFidelityIndexToKernel(m_solverFidelityIndex);
            }
        }
    }

    protected void SetRequiredSolverFidelityIndexToKernel(SolverFidelityIndex fidx)
    {
        if (m_base != IntPtr.Zero)
            TNT.apSetArticulationRequiredSolverFidelityIndex(m_base, (int)(m_useWorldSolverFidelityIndex ? m_world.DefaultSolverFidelityIndex : fidx) );
    }
}
