using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using PhysicsAPI;

/**
 * @brief Class representing a single body/link connected to an articulation with a ball-and-socket joint
 */
public class tntBallLink : tntChildLink
{
    public tntBallLink() : base(3, new bool[3]{true, true, true})
    {
        m_added = true;
    }

    /**
     * Fills reducedState state vector with data pertaining to this link
     * @param reducedState state vector to be filled
     * @param offset offset into the vector at which this link is to put its data
     */
    public override void FillReducedStateVectorFromCurrentState(tntReducedState reducedState, int offset)
    {
        reducedState.m_values[offset + 0] = GetRawPositionVariable(0);
        reducedState.m_values[offset + 1] = GetRawPositionVariable(1);
        reducedState.m_values[offset + 2] = GetRawPositionVariable(2);
        reducedState.m_values[offset + 3] = GetRawPositionVariable(3);
        //
        float qd1 = CurrentVelocity[0];
        float qd2 = CurrentVelocity[1];
        float qd3 = CurrentVelocity[2];
        Vector3 qd3dInChild = new Vector3(qd1, qd2, qd3) * Mathf.Deg2Rad;
        Vector3 qd3dInParent = transform.InverseTransformVector(transform.TransformVector(qd3dInChild));
        reducedState.m_values[offset + 4] = qd3dInParent.x;
        reducedState.m_values[offset + 5] = qd3dInParent.y;
        reducedState.m_values[offset + 6] = qd3dInParent.z;
    }
}
