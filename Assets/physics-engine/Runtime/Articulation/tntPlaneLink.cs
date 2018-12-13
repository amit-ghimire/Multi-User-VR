using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using PhysicsAPI;

/**
 * @brief Class representing a single body/link connected to an articulation with a planar joint
 */
public class tntPlaneLink : tntChildLink
{	
	// Public Properties
	public Vector3 m_axisA;

    public tntPlaneLink() : base(3, new bool[3]{true, false, false})
    {
        m_axisA = new Vector3(0, 0, 1);
    }

    public override Vector3 PivotA
    {
        get { return Vector3.zero; }
        set {/*Intentionally empty - this makes no sense for a plane link*/}
    }

    public override Vector3 PivotB
    {
        get { return Vector3.zero; }
        set { /*Intentionally empty - this makes no sense for a plane link*/}
    }

    public override bool ArePivotsMatching()
    {
        return true;
    }

    /**
     * Returns rotation axis of this link's inboard planar joint
     * @return joint rotation axis expressed in world coordinates
     */
    public Vector3 AxisAToWorld()
    {
        return m_parent.transform.TransformDirection(m_axisA);
    }

    /**
     * Sets rotation axis of this link's inboard planar joint
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
     * @remark implementation is missing for tntPlaneLink
     */
    public override void FillReducedStateVectorFromCurrentState(tntReducedState reducedState, int offset)
    {
        throw new Exception("tntPlaneLink.FillReducedStateVectorFromCurrentState is not implemented!");
    }
}
