using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using PhysicsAPI;

/**
 * @brief Class representing a single body/link connected to an articulation with a slider joint
 */
public class tntSliderLink : tntChildLink
{	
	// Public Properties
	public Vector3 m_axisA;

    public tntSliderLink() : base(1, new bool[1]{false})
    {
        m_axisA = new Vector3(0, 0, 1);
    }

    public override Vector3 PivotA
    {
        get { return Vector3.zero; }
        set {/*Intentionally empty - this makes no sense for a slider link*/}
    }

    public override Vector3 PivotB
    {
        get { return Vector3.zero; }
        set { /*Intentionally empty - this makes no sense for a slider link*/}
    }

    public override bool ArePivotsMatching()
    {
        return true;
    }

    /**
     * Returns translation axis of this link's inboard slider joint
     * @return joint translation axis expressed in world coordinates
     */
    public Vector3 AxisAToWorld()
    {
        return m_parent.transform.TransformDirection(m_axisA);
    }

    /**
     * Sets translation axis of this link's inboard slider joint
     * @param axisWorld translation rotation axis expressed in world coordinates
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
     * @remark it simply sets zero orientation and angular velocity for tntChildLink
     */
    public override void FillReducedStateVectorFromCurrentState(tntReducedState reducedState, int offset)
    {        
        reducedState.m_values[offset + 0] = 0f;
        reducedState.m_values[offset + 1] = 0f;
        reducedState.m_values[offset + 2] = 0f;
        reducedState.m_values[offset + 3] = 1f;
        //
        reducedState.m_values[offset + 4] = 0f;
        reducedState.m_values[offset + 5] = 0f;
        reducedState.m_values[offset + 6] = 0f;
    }
}
