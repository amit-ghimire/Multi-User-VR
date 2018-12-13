using UnityEngine;
using System;
using System.Collections;

using PhysicsAPI;

/**
 * @brief Class representing momentary state of a articulation
 * 
 * tntReducedState encapsulates 3d orientations and angular velocities of all links in a articulation.
 * Additionally, articulation as a whole is described by UP direction, heading angle, base link position and linear velocity.
 * Articulation motion can be discretely represented by a time-sequence of tntReducedState objects.
 */
[System.Serializable]
public class tntReducedState : ScriptableObject
{
    public int m_numOfChildLinks;
    public string[] m_names;  // optional name labels for the joints
    public bool[] m_editAsEulers;
    public Vector3[] m_eulers;
    public float[] m_values;

    public static int GetProperValuesVectorSize(int numChildLinks)
    {
        // TODO: we should ask kernel for meta-data when preparing this since we're using raw m_values vectors to send poses to it
        // sth like: acGetRawPoseVectorSize
        // let's keep it here for now: it's now explicit and re-usable by others but still imperfect
        return 17 + 7 * numChildLinks;
    }

    public static float [] CreateProperlySizedValuesVector(int numChildLinks, bool initialize)
    {
        // TODO: we should ask kernel for meta-data when preparing this since we're using raw m_values vectors to send poses to it
        // sth like: acGetRawPoseVectorStructure
        // let's keep it here for now: it's now explicit and re-usable by others but still imperfect
        float[] values = new float[tntReducedState.GetProperValuesVectorSize(numChildLinks)];

        if (initialize)
        {
            values[0] = Vector3.up.x;
            values[1] = Vector3.up.y;
            values[2] = Vector3.up.z;
            values[3] = 0;    // heading

            values[4] = 0;    // root x
            values[5] = 0;    // root y
            values[6] = 0;    // root z
            values[7] = 0;    // rotation.x
            values[8] = 0;    // rotation.y
            values[9] = 0;    // rotation.z
            values[10] = 1;   // rotation.w

            values[11] = 0;   // velocity x;
            values[12] = 0;   // velocity y;
            values[13] = 0;   // velocity z;
            values[14] = 0;   // ang velocity x;
            values[15] = 0;   // ang velocity y;
            values[16] = 0;   // ang velocity z;

            for (int i = 0; i < numChildLinks; ++i)
            {
                values[17 + 7 * i] = 0;   // rel orientation x
                values[18 + 7 * i] = 0;   // rel orientation y
                values[19 + 7 * i] = 0;   // rel orientation z
                values[20 + 7 * i] = 1;   // rel orientation w
                values[21 + 7 * i] = 0;   // rel ang velocity x
                values[22 + 7 * i] = 0;   // rel ang velocity y
                values[23 + 7 * i] = 0;   // rel ang velocity z
            }
        }

        return values;
    }

    public int NumOfChildLinks
    {
        get { return m_numOfChildLinks; }
        set
        {
            if (value != m_numOfChildLinks)
            {
                m_numOfChildLinks = value;
                m_values = CreateProperlySizedValuesVector(m_numOfChildLinks, true);                                
            }
        }
    }

    /**
     * Creates a deep copy of this tntReducedState
     */
	public tntReducedState Copy() {
		tntReducedState ret = ScriptableObject.CreateInstance<tntReducedState>();

		ret.m_names = new string[m_names.Length];
		m_names.CopyTo (ret.m_names, 0);

		ret.m_editAsEulers = new bool[m_editAsEulers.Length];
		m_editAsEulers.CopyTo (ret.m_editAsEulers, 0);

		ret.m_eulers = new Vector3[m_eulers.Length];
		m_eulers.CopyTo (ret.m_eulers, 0);

		ret.NumOfChildLinks = m_names.Length;
		if (ret.NumOfChildLinks > 0) {
			m_values.CopyTo (ret.m_values, 0);
		}

		return ret;
	}

    /**
     * Performs a per-link name-based copy from the passed in state into this state
     * @param fromState tntReducedState object top copy/map from
     */
    public void Remap(tntReducedState fromState)
    {
        if (!fromState)
            return;

        // copy global and base variables
        for (int i = 0; i < 17; ++i)
            m_values[i] = fromState.m_values[i];

        // remap link variables using names
        for (int linkIndex = 0; linkIndex < m_numOfChildLinks; ++linkIndex)
        {
            int matchedLinkIndex = Array.FindIndex<string>(fromState.m_names,
                childLinkName => childLinkName == m_names[linkIndex]);     // TBD: find a better/more efficient way of matching links
            if (matchedLinkIndex < 0)
                continue;

            for (int i = 0; i < 7; ++i)
                m_values[17 + 7 * linkIndex + i] = fromState.m_values[17 + 7 * matchedLinkIndex + i];
            m_eulers[linkIndex] = fromState.m_eulers[matchedLinkIndex];
        }
    }

    /**
     * Allocates memory for internal arrays based on the number of multibody links
     */
    public void AllocArrays()
    {
        m_names = new string[m_numOfChildLinks];
        m_editAsEulers = new bool[m_numOfChildLinks];
        m_eulers = new Vector3[m_numOfChildLinks];
    }

    /**
     * Returns the relative orientation of a single link connected to the multibody by the specified joint
     * @param jointIndex index of the joint
     * @return quaternion representing relative orientation of the link
     */
    public Quaternion GetJointOrientation(int jointIndex)
    {
        if (jointIndex < 0 || jointIndex >= m_numOfChildLinks)
            return Quaternion.identity;
        int index = 17 + 7 * jointIndex;
        return new Quaternion(m_values[index], m_values[index + 1], m_values[index + 2], m_values[index + 3]);
    }

    /**
     * Sets the relative orientation of a single link connected to the multibody by the specified joint
     * @param jointIndex index of the joint
     * @param eulers relative orientation to be set expressed using Euler angles
     */
    public void SetJointOrientationToEulers(int jointIndex, Vector3 eulers)
    {
        if (jointIndex < 0 || jointIndex >= m_numOfChildLinks)
            return;
        int index = 17 + 7 * jointIndex;
        Quaternion quat = Quaternion.Euler(eulers);
        m_values[index]     = quat.x;
        m_values[index + 1] = quat.y;
        m_values[index + 2] = quat.z;
        m_values[index + 3] = quat.w;

        if (m_editAsEulers[jointIndex])
            m_eulers[jointIndex] = eulers;
    }

    /**
     * Sets the relative orientation of a single link connected to the multibody by the specified joint
     * @param jointIndex index of the joint
     * @param quat relative orientation to be set expressed using a quaternion
     */
    public void SetJointOrientationToQuaternion(int jointIndex, Quaternion quat)
    {
        if (jointIndex < 0 || jointIndex >= m_numOfChildLinks)
            return;
        int index = 17 + 7 * jointIndex;
        m_values[index] = quat.x;
        m_values[index + 1] = quat.y;
        m_values[index + 2] = quat.z;
        m_values[index + 3] = quat.w;

        if (m_editAsEulers[jointIndex])
            m_eulers[jointIndex] = quat.eulerAngles;
    }

    public void SetRootTransform(Vector3 rootPos, Quaternion rootOri)
    {
        m_values[4] = rootPos.x;    // root x
        m_values[5] = rootPos.y;    // root y
        m_values[6] = rootPos.z;    // root z
        m_values[7] = rootOri.x;    // rotation.x
        m_values[8] = rootOri.y;    // rotation.y
        m_values[9] = rootOri.z;    // rotation.z
        m_values[10] = rootOri.w;   // rotation.w
    }

    public void GetRootTransform(out Vector3 rootPos, out Quaternion rootOri)
    {
        rootPos = new Vector3(m_values[4], m_values[5], m_values[6]);
        rootOri = new Quaternion(m_values[7], m_values[8], m_values[9], m_values[10]);
    }

    public void GetRootVelocity(out Vector3 rootVel, out Vector3 rootAngVel)
    {
        rootVel = new Vector3(m_values[11], m_values[12], m_values[13]);
        rootAngVel = new Vector3(m_values[14], m_values[15], m_values[16]);
    }

    public string GetJsonText()
    {
        byte[] bytes = new byte[m_values.Length * sizeof(float)];
        Buffer.BlockCopy(m_values, 0, bytes, 0, bytes.Length);
        return "{\r\n    \"numLinks\" : " + m_numOfChildLinks + ",\r\n    \"values\" : \"" + System.Convert.ToBase64String(bytes) + "\"\r\n}";
    }
}
