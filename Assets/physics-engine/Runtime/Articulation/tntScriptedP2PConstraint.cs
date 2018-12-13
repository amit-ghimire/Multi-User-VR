using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using PhysicsAPI;

/**
 * @brief Class representing a point-to-point constraint in which user should dictate the exact behavior with a script instead of using a link.
 * NOTE: This might use only Rigid Body + Script in some cases, so it's not really "ArticulationConstraint" per se. 
 */
public class tntScriptedP2PConstraint : tntArticulationConstraint
{

    tntScriptedP2PConstraint() : base(){ }

    public override void AddConstraint()
	{
		if (!m_added && m_world)
		{
			m_constraint = m_world.AddScriptedP2PConstraint(this);
			m_added = true;
		}
	}

	public override void RemoveConstraint()
	{
		if (m_added && m_constraint!= IntPtr.Zero)
        {
            if (m_useBodyB)
            {
                m_world.RemoveConstraint(m_constraint);
            }
            else
            {
                m_world.RemoveArticulationConstraint(m_constraint);
            }
            m_added = false;
        }
    }

    public override void SetBreakingImpulse(float breakingImpulse)
	{
		m_breakingImpulse = breakingImpulse;
        if (m_constraint != IntPtr.Zero)
        {
            if (m_useBodyB)
            {
                TNT.apSetConstraintBreakingImpulse(m_constraint, breakingImpulse);
            }
            else
            {
                TNT.apSetP2PLinkBreakingImpulse(m_constraint, breakingImpulse);
            }
        }
    }

    public void SetPivot(Vector3 pivot)
    {

        if (m_useBodyB)
        {
            TNT.apSetP2PConstraintPivotB(m_constraint, pivot.x, pivot.y, pivot.z);
        }
        else
        {
            TNT.apSetP2PlinkPivotB(m_constraint, pivot.x, pivot.y, pivot.z);
        }
    }
}
