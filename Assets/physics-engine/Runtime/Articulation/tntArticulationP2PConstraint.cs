using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using PhysicsAPI;

/**
 * @brief Class representing a point-to-point articulation constraint
 */
public class tntArticulationP2PConstraint : tntArticulationConstraint
{
    
    tntArticulationP2PConstraint() : base(){ }

    public override void AddConstraint()
	{
		if (!m_added && m_world)
		{
			m_constraint = m_world.AddArticulationP2PConstraint(this);
			m_added = true;
		}
	}

	public override void RemoveConstraint()
	{
		if (m_added && m_constraint!= IntPtr.Zero)
		{
			m_world.RemoveArticulationConstraint(m_constraint);
		    m_added = false;
		}
	}
        
	public override void SetBreakingImpulse(float breakingImpulse)
	{
		m_breakingImpulse = breakingImpulse;
		if (m_constraint != IntPtr.Zero)
			TNT.apSetP2PLinkBreakingImpulse(m_constraint, breakingImpulse);
	}
}
