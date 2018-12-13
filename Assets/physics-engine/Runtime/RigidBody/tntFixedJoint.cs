using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using PhysicsAPI;

/**
 * @brief Class representing a rigid-body "fixed" joint constraint
 */
public class tntFixedJoint : tntRigidBodyConstraint
{
    tntFixedJoint() : base(Type.Fixed) { }
 }
