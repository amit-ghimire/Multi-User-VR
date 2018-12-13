using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using PhysicsAPI;

/**
 * @brief Class representing a single body/link connected to an articulation with a universal joint
 */
public class tntUniversalLink : tntChildLink
{	
	// Public Properties
	public Vector3 m_axisA;
	public Vector3 m_axisB;

    public tntUniversalLink() : base(2, new bool[2]{true, true})
    {}

    /**
     * Returns the first rotational axis of this joint expressed in world coordinates
     * return the first rotational axis expressed in world coordinates
     */
	public Vector3 AxisAToWorld()
	{
		return m_parent.transform.TransformDirection(m_axisA);
	}

    /**
     * Sets the first rotational axis of this joint
     * @param axisWorld axis expressed in world coordinates
     */
    public void AxisAFromWorld(Vector3 axisWorld)
    {
        m_axisA = m_parent.transform.InverseTransformDirection(axisWorld);
        MathUtils.RoundToGrid(ref m_axisA);
    }

    /**
     * Returns the second rotational axis of this joint expressed in world coordinates
     * return the second rotational axis expressed in world coordinates
     */
    public Vector3 AxisBToWorld()
	{
		return transform.TransformDirection(m_axisB);
	}

    /**
     * Sets the second rotational axis of this joint
     * @param axisWorld axis expressed in world coordinates
     */
	public void AxisBFromWorld(Vector3 axisWorld)
	{
		m_axisB = transform.InverseTransformDirection(axisWorld);
		MathUtils.RoundToGrid(ref m_axisB);
	}

    /**
     * Fills reducedState state vector with data pertaining to this link
     * @param reducedState state vector to be filled
     * @param offset offset into the vector at which this link is to put its data
     */
    public override void FillReducedStateVectorFromCurrentState(tntReducedState reducedState, int offset)
    {
        float q1 = GetRawPositionVariable(0);
        float q2 = GetRawPositionVariable(1);
        Vector3 axisAInChild = transform.InverseTransformDirection(AxisAToWorld()).normalized;
        Quaternion q3d = Quaternion.AngleAxis(q2 * Mathf.Rad2Deg, m_axisB) * Quaternion.AngleAxis(q1 * Mathf.Rad2Deg, axisAInChild);
        reducedState.m_values[offset + 0] = q3d.x;
        reducedState.m_values[offset + 1] = q3d.y;
        reducedState.m_values[offset + 2] = q3d.z;
        reducedState.m_values[offset + 3] = q3d.w;
        //
        float qd1 = CurrentVelocity[0];
        float qd2 = CurrentVelocity[1];
        Vector3 qd3dInChild = (axisAInChild * qd1 + m_axisB * qd2) * Mathf.Deg2Rad;
        Vector3 qd3dInParent = transform.InverseTransformVector(transform.TransformVector(qd3dInChild));
        reducedState.m_values[offset + 4] = qd3dInParent.x;
        reducedState.m_values[offset + 5] = qd3dInParent.y;
        reducedState.m_values[offset + 6] = qd3dInParent.z;
    }
}
