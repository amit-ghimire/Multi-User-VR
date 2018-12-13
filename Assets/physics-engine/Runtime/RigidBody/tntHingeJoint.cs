using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using PhysicsAPI;

/**
 * @brief Class representing a rigid-body hinge joint constraint
 */
public class tntHingeJoint : tntRigidBodyConstraint
{
    tntHingeJoint() : base(Type.Hinge) { }
}
