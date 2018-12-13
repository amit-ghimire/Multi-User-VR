using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using PhysicsAPI;

/**
 * @brief Class representing a single body/link connected to an articulation with a hinge joint
 */
public class tntHingeLink : tntChildLink
{	
	// Public Properties
	public Vector3 m_axisA;

    public tntHingeLink() : base(1, new bool[1]{true})
    {
        m_axisA = new Vector3(0, 0, 1);
    }

    /**
     * Returns rotation axis of this link's inboard hinge joint
     * @return joint rotation axis expressed in world coordinates
     */
    public Vector3 AxisAToWorld()
    {
        return m_parent.transform.TransformDirection(m_axisA);
    }

    /**
     * Sets rotation axis of this link's inboard hinge joint
     * @param axisWorld joint rotation axis expressed in world coordinates
     */
    public void AxisAFromWorld(Vector3 axisWorld)
    {
        m_axisA = m_parent.transform.InverseTransformDirection(axisWorld);
        MathUtils.RoundToGrid(ref m_axisA);
    }

    /**
     * Fills reducedState state vector with data pertaining to this link
     * @param reducedState state vector to be filled
     * @param offset offset into the vector at which this link is to put its data
     */
    public override void FillReducedStateVectorFromCurrentState(tntReducedState reducedState, int offset)
    {       
        float q = GetRawPositionVariable(0);
        Vector3 axisAInChild = transform.InverseTransformDirection(AxisAToWorld()).normalized;
        Quaternion q3d = Quaternion.AngleAxis(q * Mathf.Rad2Deg, axisAInChild);
        reducedState.m_values[offset + 0] = q3d.x;
        reducedState.m_values[offset + 1] = q3d.y;
        reducedState.m_values[offset + 2] = q3d.z;
        reducedState.m_values[offset + 3] = q3d.w;
        //
        float qd = CurrentVelocity[0];
        Vector3 qd3dInChild = axisAInChild * qd * Mathf.Deg2Rad;
        Vector3 qd3dInParent = transform.InverseTransformVector(transform.TransformVector(qd3dInChild));
        reducedState.m_values[offset + 4] = qd3dInParent.x;
        reducedState.m_values[offset + 5] = qd3dInParent.y;
        reducedState.m_values[offset + 6] = qd3dInParent.z;
    }
}
