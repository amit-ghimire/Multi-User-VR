using UnityEngine;
using System.Collections;

using PhysicsAPI;

[System.Serializable]
public class tntHumanoidControlParams : ScriptableObject
{
	//-------------------Dynamic Control Parameters----------------- 

	// Enums	
	public const int P_STRIDE_DURATION = 0;
	public const int P_DESIRED_HEIGHT = 1;
	public const int P_DESIRED_HEIGHT_TRAJ1 = 2;
	public const int P_DESIRED_HEIGHT_TRAJ2 = 3;
	public const int P_DESIRED_HEIGHT_TRAJ3 = 4;
	public const int P_DESIRED_HEIGHT_TRAJ4 = 5;
	public const int P_DESIRED_HEIGHT_TRAJ5 = 6;
	public const int P_DESIRED_HEIGHT_TRAJ6 = 7;
	public const int P_VFORCE_KP_SAGITTAL = 8;
	public const int P_VFORCE_KD_SAGITTAL = 9;
	public const int P_VFORCE_KP_CORONAL = 10;
	public const int P_VFORCE_KD_CORONAL = 11;
	public const int P_VFORCE_KP_VERTICAL = 12;
	public const int P_VFORCE_KD_VERTICAL = 13;
	public const int P_VFORCE_FF_VERTICAL_WEIGHT_PERCENTAGE = 14;
	public const int P_BODYFRAME_TORQUE_KP = 15;
	public const int P_BODYFRAME_TORQUE_KD = 16;
	public const int P_BFRAME_LEAN_FORWARD = 17;
	public const int P_BFRAME_LEAN_SIDEWAYS = 18;
	public const int P_BFRAME_LEAN_FORWARD_TRAJ1 = 19;
	public const int P_BFRAME_LEAN_FORWARD_TRAJ2 = 20;
	public const int P_BFRAME_LEAN_FORWARD_TRAJ3 = 21;
	public const int P_BFRAME_LEAN_FORWARD_TRAJ4 = 22;
	public const int P_BFRAME_LEAN_FORWARD_TRAJ5 = 23;
	public const int P_BFRAME_LEAN_FORWARD_TRAJ6 = 24;
	public const int P_BFRAME_LEAN_SIDEWAYS_TRAJ1 = 25;
	public const int P_BFRAME_LEAN_SIDEWAYS_TRAJ2 = 26;
	public const int P_BFRAME_LEAN_SIDEWAYS_TRAJ3 = 27;
	public const int P_BFRAME_LEAN_SIDEWAYS_TRAJ4 = 28;
	public const int P_BFRAME_LEAN_SIDEWAYS_TRAJ5 = 29;
	public const int P_BFRAME_LEAN_SIDEWAYS_TRAJ6 = 30;
	public const int P_SPINE_TWIST = 31;
	public const int P_SPINE_SLOUCH_FORWARD = 32;
	public const int P_SPINE_SLOUCH_SIDEWAYS = 33;
	public const int P_SPINE_TWIST_TRAJ1 = 34;
	public const int P_SPINE_TWIST_TRAJ2 = 35;
	public const int P_SPINE_TWIST_TRAJ3 = 36;
	public const int P_SPINE_TWIST_TRAJ4 = 37;
	public const int P_SPINE_TWIST_TRAJ5 = 38;
	public const int P_SPINE_TWIST_TRAJ6 = 39;
	public const int P_SPINE_SLOUCH_FORWARD_TRAJ1 = 40;
	public const int P_SPINE_SLOUCH_FORWARD_TRAJ2 = 41;
	public const int P_SPINE_SLOUCH_FORWARD_TRAJ3 = 42;
	public const int P_SPINE_SLOUCH_FORWARD_TRAJ4 = 43;
	public const int P_SPINE_SLOUCH_FORWARD_TRAJ5 = 44;
	public const int P_SPINE_SLOUCH_FORWARD_TRAJ6 = 45;
	public const int P_SPINE_SLOUCH_SIDEWAYS_TRAJ1 = 46;
	public const int P_SPINE_SLOUCH_SIDEWAYS_TRAJ2 = 47;
	public const int P_SPINE_SLOUCH_SIDEWAYS_TRAJ3 = 48;
	public const int P_SPINE_SLOUCH_SIDEWAYS_TRAJ4 = 49;
	public const int P_SPINE_SLOUCH_SIDEWAYS_TRAJ5 = 50;
	public const int P_SPINE_SLOUCH_SIDEWAYS_TRAJ6 = 51;
	public const int P_LEG_PLANE_ANGLE_LEFT = 52;
	public const int P_LEG_PLANE_ANGLE_RIGHT = 53;
	public const int P_SWING_FOOT_HEIGHT_TRAJ1 = 54;
	public const int P_SWING_FOOT_HEIGHT_TRAJ2 = 55;
	public const int P_SWING_FOOT_HEIGHT_TRAJ3 = 56;
	public const int P_SWING_FOOT_HEIGHT_TRAJ4 = 57;
	public const int P_SWING_FOOT_HEIGHT_TRAJ5 = 58;
	public const int P_SWING_FOOT_HEIGHT_TRAJ6 = 59;
    public const int P_SWING_ANKLE_ROT1 = 60;
    public const int P_SWING_ANKLE_ROT2 = 61;
    public const int P_SWING_ANKLE_ROT3 = 62;
    public const int P_SWING_ANKLE_ROT4 = 63;
    public const int P_SWING_ANKLE_ROT5 = 64;
    public const int P_SWING_ANKLE_ROT6 = 65;
    public const int P_SWING_TOE_ROT1 = 66;
    public const int P_SWING_TOE_ROT2 = 67;
    public const int P_SWING_TOE_ROT3 = 68;
    public const int P_SWING_TOE_ROT4 = 69;
    public const int P_SWING_TOE_ROT5 = 70;
    public const int P_SWING_TOE_ROT6 = 71;
    public const int P_STANCE_ANKLE_ROT1 = 72;
    public const int P_STANCE_ANKLE_ROT2 = 73;
    public const int P_STANCE_ANKLE_ROT3 = 74;
    public const int P_STANCE_ANKLE_ROT4 = 75;
    public const int P_STANCE_ANKLE_ROT5 = 76;
    public const int P_STANCE_ANKLE_ROT6 = 77;
    public const int P_STANCE_TOE_ROT1 = 78;
    public const int P_STANCE_TOE_ROT2 = 79;
    public const int P_STANCE_TOE_ROT3 = 80;
    public const int P_STANCE_TOE_ROT4 = 81;
    public const int P_STANCE_TOE_ROT5 = 82;
    public const int P_STANCE_TOE_ROT6 = 83;

    public const int P_STEP_WIDTH = 84;
    public const int P_STEP_FORWARD_OFFSET = 85;
	
	public const int P_SWING_FOOT_VFORCE_KP = 86;
	public const int P_SWING_FOOT_VFORCE_KD = 87;
	public const int P_SWING_FOOT_MAX_FORCE = 88;
	
	public const int P_LEFT_LEG_SWING_START = 89;
	public const int P_LEFT_LEG_SWING_END = 90;
	public const int P_RIGHT_LEG_SWING_START = 91;
	public const int P_RIGHT_LEG_SWING_END = 92;
	
	public const int P_STEP_TARGET_INTERPOLATION_FUNCTION1 = 93;
	public const int P_STEP_TARGET_INTERPOLATION_FUNCTION2 = 94;
	public const int P_STEP_TARGET_INTERPOLATION_FUNCTION3 = 95;
	public const int P_STEP_TARGET_INTERPOLATION_FUNCTION4 = 96;
	public const int P_STEP_TARGET_INTERPOLATION_FUNCTION5 = 97;
	public const int P_STEP_TARGET_INTERPOLATION_FUNCTION6 = 98;

    public const int P_GRF_REGULARIZER = 99;
    public const int P_GRF_TORQUE_TO_FORCE_OBJECTIVE_RATIO = 100;

	public const int P_MAX_CONTACT_POINTS_PER_FOOT = 101;
	public const int P_ROTATION_FRICTION_KD = 102;
	public const int P_GYRO_RATIO = 103;
	public const int P_STAND_STILL_THRESHOLD = 104;
    public const int P_EARLY_SWING_TERMINATE = 105;
    public const int P_LATE_SWING_TERMINATE = 106;
	public float[] m_params = new float[P_LATE_SWING_TERMINATE + 1];

    public tntHumanoidControlParams Copy() {
        tntHumanoidControlParams ret = ScriptableObject.CreateInstance<tntHumanoidControlParams>();

        m_params.CopyTo (ret.m_params, 0);
		
		return ret;
	}
}
