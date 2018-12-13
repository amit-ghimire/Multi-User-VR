using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using PhysicsAPI;

/**
 * @brief Class representing a rigid-body ball joint (which is equivalent to point-to-point constraint) 
 */
public class tntBallJoint : tntRigidBodyConstraint
{
    tntBallJoint() : base(Type.Ball) { }
}
