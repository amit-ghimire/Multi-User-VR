/**
    This class is used as a container for the properties needed by a PD controller
*/
using UnityEngine;

[System.Serializable]
public class PDParams
{
    public string name;
    //this variable is set to true if the current joint is being actively controlled, false otherwise
    public bool controlled;
    //these two variables are the proporitonal and derivative gains for the PD controller used to compute the torque
    public float kp, kd;
    //this is the maximum absolute value torque allowed at this joint
    public float maxAbsTorque;
    //the torques about the about the x, y and z axis will be scaled differently to account for the potentially different principal moments of inertia
    //of the child
    public Vector3 scale;

    public float strength;

    //this variable, if true, indicates that the desired orientation is specified in the character coordinate frame
    public bool relToCharFrame;

    /**
        This constructor initializes the variables to some safe, default values
    */
	public PDParams()
	{
		controlled = true;
		kp = kd = 0;
		maxAbsTorque = 0;
		scale = Vector3.zero;
		strength = 1;
		relToCharFrame = false;
	}
}