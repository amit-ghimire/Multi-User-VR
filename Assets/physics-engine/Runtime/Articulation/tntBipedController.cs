using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.ComponentModel;
using System.Reflection;

using PhysicsAPI;

public enum StanceOrientation
{
    LEFT_STANCE = 0,
    RIGHT_STANCE = 1
}

public class BipedBoneMarkEnumUtil
{
    public enum BipedBoneMark // https://sites.google.com/a/midastouchgame.com/develop/physicsengine/biped-skeleton-naming-convention
    {
        [DescriptionAttribute("lHip")]LeftHip, 
        [DescriptionAttribute("rHip")]RightHip, 
        [DescriptionAttribute("lKnee")]LeftKnee, 
        [DescriptionAttribute("rKnee")]RightKnee, 
        [DescriptionAttribute("lFoot")]LeftFoot, 
        [DescriptionAttribute("rFoot")]RightFoot, 
        [DescriptionAttribute("lowerBack")]LowerBack,
        [DescriptionAttribute("torso")]Torso,
        [DescriptionAttribute("head")]Head,
        [DescriptionAttribute("lAnkle")]LeftAnkle,
        [DescriptionAttribute("rAnkle")]RightAnkle
    }

    public static string GetMarkString<T>(T mark) // make this generic to handle multiple enumerations
    { 
        FieldInfo fi = mark.GetType().GetField(mark.ToString()); 
        DescriptionAttribute[] attributes = (DescriptionAttribute[])fi.GetCustomAttributes(typeof(DescriptionAttribute), false); 
        if (attributes.Length > 0) 
        { 
            return attributes[0].Description; 
        } 
        else
        { 
            return mark.ToString(); 
        }
    }

    public static bool CheckMark<T>(tntLink joint, T mark)
    {
        if (joint != null)
            return joint.m_mark.Equals(GetMarkString(mark), StringComparison.InvariantCultureIgnoreCase);
        else
            return false;
    }
}

/**
    This class is used to represent generic trajectories. 
    We'll define a trajectory that can be parameterized by a one-d parameter (called t).
    Based on a set of knots ( tuples <t, T>), we can evaluate the 
    trajectory at any t, through interpolation. This is not used for extrapolation.
    Outside the range of the knots, the closest known value is returned instead.
*/
[System.Serializable]
public class Trajectory1D
{
    public List<float> tValues;
    public List<float> values;

    // A caching variable to optimize searching
    int m_lastIndex;
    IntPtr m_engineHandle;	// handle in the engine
    bool m_dirty;		// whether synchronized with engine

    public Trajectory1D()
    {
        tValues = new List<float>();
        values = new List<float>();
        m_dirty = true;
    }

    /**
        This method returns the index of the first knot whose value is larger than the parameter value t.
        If no such index exists (t is larger than any
        of the values stored), then values.size() is returned.
    */
    int getFirstLargerIndex(float t)
    {
        int size = tValues.Count;
        if( size == 0 ) 
            return 0;
        if( t < tValues[(m_lastIndex+size-1) % size] )
            m_lastIndex = 0;
        for (int i = 0; i<size;i++)
        {
            int index = (i + m_lastIndex) % size;
            if (t < tValues[index])
            {
                m_lastIndex = index;
                return index;
            }
        }
        return size;
    }

    /**
        This method is used to insert a new knot in the current trajectory
    */
    public void addKnot(float t, float val)
    {
        //first we need to know where to insert it, based on the t-values
        int index = getFirstLargerIndex(t);
        
        tValues.Insert(index, t);
        values.Insert(index, val);
    }

    /**
        Returns the value of the ith knot. It is assumed that i is within the correct range.
    */
    public float getKnotValue(int i){
        return values[i];
    }
    
    /**
        Returns the position of the ith knot. It is assumed that i is within the correct range.
    */
    public float getKnotPosition(int i){
        return tValues[i];
    }

    /**
        returns the number of knots in this trajectory
    */
    public int getKnotCount()
    {
        return tValues.Count;
    }

    // Assumptions:  
    // 1) Before calling this API the trajectory is already parsed out of .ace file
    // 2) The engine counterpart has been allocated for this trajectory
    // 3) Only call this once during initialization. Incremental updates please use updateKnot()
    //
    public void serializeToEngine(IntPtr engineHandle)
    {
        m_engineHandle = engineHandle;
        for (int i = 0; i < getKnotCount(); ++i)
            TNT.acAddTrajectory1DElement(m_engineHandle, tValues[i], values[i]);
        m_dirty = false;
    }

    // Assumption: serializeToEngine() was invoked already to seed the engine handle.
    // Call this prior to invoking the engine simulation step
    // 
    public void syncWithEngine()
    {
        if (!m_dirty)
            return;
        if (getKnotCount() < 1 || getKnotCount() > 64)
            return;         // not supported
        float[] timesBuf = new float[64];
        float[] valuesBuf = new float[64];
        for (int i = 0; i < getKnotCount(); ++i)
        {
            timesBuf[i] = tValues[i];
            valuesBuf[i] = values[i];
        }
        unsafe
        {
            fixed (float* pTime = &timesBuf[0])
            fixed (float* pValue = &valuesBuf[0])
            {
                TNT.acUpdateTrajectory1D(m_engineHandle, getKnotCount(), pTime, pValue);
            }
        }
        m_dirty = false;
    }

    public bool updateKnot(int index, float tValue, float value)
    {
        if (tValues[index] != tValue)
        {
            tValues[index] = tValue;
            m_dirty = true;
        }
        if (values[index] != value)
        {
            values[index] = value;
            m_dirty = true;
        }
		return m_dirty;
    }

    public void scaleValues(float scalar)
    {
        for (int i = 0; i < getKnotCount(); ++i)
        {
            values[i] *= scalar;
        }
        m_dirty = true;
    }

}

/**
    This generic interface for classes that provide balance feedback for controllers for physically simulated characters.
*/
public interface BalanceFeedback
{
};

/**
    This class applies feedback that is linear in d and v - i.e. the original simbicon feedback formulation
*/
[System.Serializable]
public class LinearBalanceFeedback : BalanceFeedback
{
    //This vector, dotted with d or v, gives the quantities that should be used in the feedback formula
    public Vector3 feedbackProjectionAxis;
    //these are the two feedback gains
    public float cd;
    public float cv;
    
    public float vMin, vMax, dMin, dMax;

    public LinearBalanceFeedback()
    {
        feedbackProjectionAxis = new Vector3();
        cd = cv = 0;
        vMin = dMin = -1000;
        vMax = dMax = 1000;
    }
}

/**
 *  This helper class is used to hold information regarding one component of a state trajectory.
 *  This includes (mainly): the base trajectory, 
 *  a data member that specifies the feedback law to be used,
 *  and the axis about which it represents a rotation, 
 */
[System.Serializable]
public class TrajectoryComponent
{
    //this is the array of basis functions that specify the trajectories for the sagittal plane.
    public Trajectory1D baseTraj;
    //if this variable is set to true, then when the stance of the character is the left side, the 
    //static target provided by this trajectory should be negated
    public bool reverseAngleOnLeftStance;
    //if this variable is set to true, then when the stance of the character is the right side, the 
    //static target provided by this trajectory should be negated
    public bool reverseAngleOnRightStance;
    //this is the rotation axis that the angles obtained from the trajectory represent rotations about
    public Vector3 rotationAxis;
    
    //this is the balance feedback that is to be used with this trajectory
    public bool feedbackEnabled;
    public LinearBalanceFeedback balanceFeedback;

    //
    /**
        default constructor
    */
    public TrajectoryComponent()
    {
        baseTraj = new Trajectory1D();
        rotationAxis = new Vector3();
        reverseAngleOnLeftStance = false;
        reverseAngleOnRightStance = false;
        feedbackEnabled = false;
        balanceFeedback = null;
    }
}

/**
 *  This helper class is used to hold information regarding one state trajectory. This includes: a sequence of components, 
 *  the index of the joint that this trajectory applies to, the coordinate frame in which the final orientation is expressed, etc.
 */
[System.Serializable]
public class Trajectory
{
	//if the biped that is controlled is in a left-sideed stance, then this is the index of the joint that
	//the trajectory is used to control - it is assumed that if this is -1, then the trajectory applies
	//to the torso, and not to a joint
	public int leftStanceIndex {get; set;}
	//and this is the index of the joint that the trajectory is associated to if the biped is in a
	//right-side stance
	public int rightStanceIndex {get; set;}

	//we'll keep the joint name, for debugging purposes
	public string jName;
    //these are the components that define the current trajectory
    public List<TrajectoryComponent> components;
    
    //if this variable is set to true, then the desired orientation here is expressed in character
    //coordinates, otherwise it is relative
    //to the parent
    public bool relToCharFrame;
    
    //this is the trajectory for the strength of the joint.
    public Trajectory1D strengthTraj;
    
    /**
        default constructor
    */
    public Trajectory()
    {
        leftStanceIndex = rightStanceIndex = -1;
        jName = "NoNameJoint";
        strengthTraj = null;
        relToCharFrame = false;
        components = new List<TrajectoryComponent>();
    }
}

/**
 * This helper class is used to hold information regarding the external force trajectories for one joint. 
 * This includes forces along X, Y, Z axis and torques around X, Y, Z.
 */
[System.Serializable]
public class ExternalForce
{
	//if the biped that is controlled is in a left-sideed stance, then this is the index of the joint that
	//the trajectory is used to apply external forces  - it is assumed that if this is -1, then the 
	//trajectory applies to the root
	public int leftStanceIndex {get; set;}
	//and this is the index of the joint that the trajectory is associated to if the biped is in a
	//right-side stance
	public int rightStanceIndex {get; set;}

	public string jName;
	public Trajectory1D forceX;
	public Trajectory1D forceY;
	public Trajectory1D forceZ;
	public Trajectory1D torqueX;
	public Trajectory1D torqueY;
	public Trajectory1D torqueZ;

	private bool m_dirty;

	/**
        default constructor
    */
	public ExternalForce()
	{
		jName = "NoNameJoint";
		leftStanceIndex = rightStanceIndex = -1;
		forceX = forceY = forceZ = null;
		torqueX = torqueY = torqueZ = null;
		m_dirty = true;
	}

	public void MarkDirty() {m_dirty = true;}

	public void serializeToEngine(IntPtr engineHandle)
	{
		forceX.serializeToEngine(TNT.acGetForceXTraj(engineHandle));
		forceY.serializeToEngine(TNT.acGetForceYTraj(engineHandle));
		forceZ.serializeToEngine(TNT.acGetForceZTraj(engineHandle));
		torqueX.serializeToEngine(TNT.acGetTorqueXTraj(engineHandle));
		torqueY.serializeToEngine(TNT.acGetTorqueYTraj(engineHandle));
		torqueZ.serializeToEngine(TNT.acGetTorqueZTraj(engineHandle));
		m_dirty = false;
	}

	public void syncWithEngine()
	{
		if (m_dirty)
		{
			forceX.syncWithEngine();
			forceY.syncWithEngine();
			forceZ.syncWithEngine();
			torqueX.syncWithEngine();
			torqueX.syncWithEngine();
			torqueX.syncWithEngine();
			m_dirty = false;
		}
	}

    public void scale(float scalar)
    {
        forceX.scaleValues(scalar);
        forceY.scaleValues(scalar);
        forceZ.scaleValues(scalar);
        torqueX.scaleValues(scalar);
        torqueY.scaleValues(scalar);
        torqueZ.scaleValues(scalar);
        m_dirty = true;
    }
}

[System.Serializable]
public class BipedConState
{
	public List<ExternalForce> sExternalForces;
    public List<Trajectory> sTraj;
    //this is a description of this state, for debugging purposes
    public string description;
    //this is the number of the state that we should transition to in the controller's
    // finite state machine
    public int nextStateIndex;
    //this is the ammount of time that it is expected the biped will spend in this state
    public float stateTime;
    public float maxGyro;
    
    //upon a transition to a new FSM state, it is assumed that the stance of the character
    //either will be given stance, it will be reverseed , or keept the same.
    //if a state is designed for a certain stance, it is given by this variable
    //for generic states, this variable is used to determine if the stance should be reversed
    //(as opposed to set to left or right), or stay the same.
    public bool reverseStance;
    //and if this is the same, then upon entering this FSM state, the stance will remain the same
    public bool keepStance;
    //if both keepStance and reverseStance are set to false, then this is the state that
    //the character is asumed to take
    public StanceOrientation stateStance;
    
    //if this variable is set to true, it indicates that the transition to the new state
    //should occur when the swing foot contacts the ground
    //if this variable is false, then it will happen when the time of the controller goes up
    public bool transitionOnFootContact;
    //if we are to allow a transition on foot contact, we need to take care of the
    //possibility that it will occur early. In this case, we may still want to switch.
    //If phi is at least this, then it is assumed that we can transition;
    public float minPhiBeforeTransitionOnFootContact;
    //also, in order to make sure that we don't transition tooooo early, we expect a minimum
    //force applied on the swing foot before it should register as a contact
    public float minSwingFootForceForContact;
    
    //this is the trajectory for the zero value of  the feedback d
    public Trajectory1D dTrajX;
    public Trajectory1D dTrajZ;

    //this is the trajectory for the zero value of the feedback v
    public Trajectory1D vTrajX;
    public Trajectory1D vTrajZ;

    public BipedConState()
    {
        description = "Uninitialized state";
        nextStateIndex = -1;
        stateTime = 0;
        maxGyro = 0;
        transitionOnFootContact = true;
        minPhiBeforeTransitionOnFootContact = 0.5f;
        minSwingFootForceForContact = 20.0f;
        reverseStance = false;
        keepStance = false;

		sExternalForces = new List<ExternalForce>();
        sTraj = new List<Trajectory>();
        dTrajX = null;
        dTrajZ = null;
        vTrajX = null;
        vTrajZ = null;
    }
}

[System.Serializable]
// Assumptions:
// 1) stance is the first field
// 2) the structure layout matches the ACE internal structure
public class BipedControllerState
{
	public int stance;				
	public float phi;
	public int FSMStateIndex;
	public bool bodyGroundContact;
	public bool feetInTheAir;
};


public class tntBipedController : tntController
{
    // Public Properties 
	public bool isIKVMOn;
	public bool isInvertedPendulumOn;		// only used when IKVM is on
	public bool isGravityCompensationOn;	// only used when IKVM is on
	public float stepWidth;					// only used when IKVM is on
	public bool isRagdollOn;
    public bool useExplicitPDControllers;
    public bool useImplicitPositionError;
    public bool useMOIAboutJointPosition;

    public float stanceHipDamping;
    public float stanceHipMaxVelocity;
    public float rootPredictiveTorqueScale;
    public float maxGyro;
    public int startingState;
    public StanceOrientation startingStance;
 
    //the root is not directly controlled by any joint, so we will store its Kp, Kd and maxTorque separated here.
    //while the pose controller does not use these values, other types of controllers may need this information
    public PDParams rootControlParams;

    //this is the array of joint properties used to specify the 
    public List<PDParams> controlParams;
 
    //this is a collection of the states that are used in the controller
    public List<BipedConState> states;

	public BipedControllerState controllerState;

    public tntBipedController()
    { 
        controlParams = new List<PDParams>();
        states = new List<BipedConState>();
		controllerState = new BipedControllerState();
        m_controller = IntPtr.Zero;
        m_worldIndex = -1;
        isIKVMOn = false;
        isInvertedPendulumOn = false;
        isGravityCompensationOn = true;
        stepWidth = 0;
        isRagdollOn = true;
        maxGyro = 0;
        useExplicitPDControllers = true;
        useImplicitPositionError = false;
        useMOIAboutJointPosition = false;
    }

	private void ResolveJointIndex(string jName, tntBase root,
								   out int leftStanceIndex,
	                               out int rightStanceIndex)
	{
		string tmpName;

		if (jName == "root")
		{
			leftStanceIndex = rightStanceIndex = -1;
			return;
		}
		//deal with the SWING_XXX' case
		if (jName.StartsWith("SWING_"))
		{
			tmpName = "r" + jName.Substring(6);
			leftStanceIndex = root.NameToIndex(tmpName);
			if (leftStanceIndex < 0)
				Debug.LogError("Cannot find joint : " + tmpName);
			tmpName = "l" + jName.Substring(6);
			rightStanceIndex = root.NameToIndex(tmpName);
			if (rightStanceIndex < 0)
				Debug.LogError("Cannot find joint : " + tmpName);
			return;
		}
		
		//deal with the STANCE_XXX' case
		if (jName.StartsWith("STANCE_"))
		{
			tmpName = "l" + jName.Substring(7);
			leftStanceIndex = root.NameToIndex(tmpName);
			if (leftStanceIndex < 0)
				Debug.LogError("Cannot find joint : " + tmpName);
			tmpName = "r" + jName.Substring(7);
			rightStanceIndex = root.NameToIndex(tmpName);
			if (rightStanceIndex < 0)
				Debug.LogError("Cannot find joint : " + tmpName);
			return;
		}
		
		//if we get here, it means it is just the name...
		leftStanceIndex = root.NameToIndex(jName);
		if (leftStanceIndex < 0)
			Debug.LogError("Cannot find joint : " + jName);
		rightStanceIndex = leftStanceIndex;
	}

    private void ResolveJoints(BipedConState state, tntBase root)
    {
		int leftIndex;
		int rightIndex;
		for (int i = 0; i < state.sExternalForces.Count; ++i)
		{
			ExternalForce force = state.sExternalForces[i];
			ResolveJointIndex(force.jName, root, out leftIndex, out rightIndex);
			force.leftStanceIndex = leftIndex;
			force.rightStanceIndex = rightIndex;
		}

        for (int i = 0; i < state.sTraj.Count; ++i)
        {
            Trajectory jt = state.sTraj[i];
			ResolveJointIndex(jt.jName, root, out leftIndex, out rightIndex);
			jt.leftStanceIndex = leftIndex;
			jt.rightStanceIndex = rightIndex;
        }
    }

	// Articulated base and child links are pushed to the physics world during Awake(). So by now
    // The articulation rooted at the parent node of this node should have already existed in the
    // physics world.
    public override bool SerializeToEngine()
    {
        if (transform.parent == null)
            return false;
        Transform rootTrans = transform.parent;
        Transform containerTrans = rootTrans.parent;
        tntBase root = rootTrans.GetComponent<tntBase>();

        tntLink lHip = null;
        tntLink rHip = null;
        tntLink lKnee = null;
        tntLink rKnee = null;
        tntLink lowerBack = null;
        tntLink torso = null;
        tntLink head = null;
        tntLink lFoot = null;
        tntLink rFoot = null;
        tntLink lAnkle = null;
        tntLink rAnkle = null;

        // detect tntLink using mark tags first
        tntLink[] joints = containerTrans.GetComponentsInChildren<tntLink>();
        foreach (tntLink joint in joints)
        {
            if (BipedBoneMarkEnumUtil.CheckMark(joint, BipedBoneMarkEnumUtil.BipedBoneMark.LeftFoot))
            {
                lFoot = joint;
            } 
            else if (BipedBoneMarkEnumUtil.CheckMark(joint, BipedBoneMarkEnumUtil.BipedBoneMark.RightFoot))
            {
                rFoot = joint;
            }
            else if (BipedBoneMarkEnumUtil.CheckMark(joint, BipedBoneMarkEnumUtil.BipedBoneMark.LeftHip))
            {
                lHip = joint;
            }
            else if (BipedBoneMarkEnumUtil.CheckMark(joint, BipedBoneMarkEnumUtil.BipedBoneMark.RightHip))
            {
                rHip = joint;
            }
            else if (BipedBoneMarkEnumUtil.CheckMark(joint, BipedBoneMarkEnumUtil.BipedBoneMark.LeftKnee))
            {
                lKnee = joint;
            }
            else if (BipedBoneMarkEnumUtil.CheckMark(joint, BipedBoneMarkEnumUtil.BipedBoneMark.RightKnee))
            {
                rKnee = joint;
            }
            else if (BipedBoneMarkEnumUtil.CheckMark(joint, BipedBoneMarkEnumUtil.BipedBoneMark.LowerBack))
            {
                lowerBack = joint;
            }
            else if (BipedBoneMarkEnumUtil.CheckMark(joint, BipedBoneMarkEnumUtil.BipedBoneMark.Torso))
            {
                torso = joint;
            }
            else if (BipedBoneMarkEnumUtil.CheckMark(joint, BipedBoneMarkEnumUtil.BipedBoneMark.Head))
            {
                head = joint;
            }
            else if (BipedBoneMarkEnumUtil.CheckMark(joint, BipedBoneMarkEnumUtil.BipedBoneMark.LeftAnkle))
            {
                lAnkle = joint;
            }
            else if (BipedBoneMarkEnumUtil.CheckMark(joint, BipedBoneMarkEnumUtil.BipedBoneMark.RightAnkle))
            {
                rAnkle = joint;
            }
        }

        // if above mark tags fail, fall back to search for game objects with certain names
        if (lHip == null)
        {
            Transform lHipTrans = containerTrans.Find("lHip");
            if (lHipTrans != null)
                lHip = lHipTrans.GetComponent<tntLink>();
        }
        if (rHip == null)
        {
            Transform rHipTrans = containerTrans.Find("rHip");
            if (rHipTrans != null)
                rHip = rHipTrans.GetComponent<tntLink>();
        }
        if (lKnee == null)
        {
            Transform lKneeTrans = containerTrans.Find("lKnee");
            if (lKneeTrans != null)
                lKnee = lKneeTrans.GetComponent<tntLink>();
        }
        if (rKnee == null)
        {
            Transform rKneeTrans = containerTrans.Find("rKnee");
            if (rKneeTrans != null)
                rKnee = rKneeTrans.GetComponent<tntLink>();
        }
        if (lowerBack == null)
        {
            Transform lowerBackTrans = containerTrans.Find("pelvis_lowerback");
            if (lowerBackTrans != null)
                lowerBack = lowerBackTrans.GetComponent<tntLink>();
        }
        if (torso == null)
        {
            Transform torsoTrans = containerTrans.Find("lowerback_torso");
            if (torsoTrans != null)
                torso = torsoTrans.GetComponent<tntLink>();
        }
        if (head == null)
        {
            Transform headTrans = containerTrans.Find("torso_head");
            if (headTrans != null)
                head = headTrans.GetComponent<tntLink>();
        }
        if (lAnkle == null)
        {
            Transform lAnkleTrans = containerTrans.Find("lAnkle");
            if (lAnkleTrans != null)
                lAnkle = lAnkleTrans.GetComponent<tntLink>();
        }
        if (rAnkle == null)
        {
            Transform rAnkleTrans = containerTrans.Find("rAnkle");
            if (rAnkleTrans != null)
                rAnkle = rAnkleTrans.GetComponent<tntLink>();
        }
        
        // Note: Due to inversed coordinates handness between Unity and APE
        // we swapped left and right foot's rigid body and meshes
        // To-do? use ArticulatedBodyImporter.flipLeftRightMeshes to determine if need to swap?
        if (lFoot == null)
        {
            Transform lFootTrans = FindInChildren(containerTrans, "rfoot");
            if (lFootTrans == null)
                lFootTrans = FindInChildren(containerTrans, "rFoot");
            if (lFootTrans != null)
                lFoot = lFootTrans.parent.GetComponent<tntLink>();
            if (lFoot == null)
                lFoot = lAnkle;
        }
        if (rFoot == null)
        {
            Transform rFootTrans = FindInChildren(containerTrans, "lfoot");
            if (rFootTrans == null)
                rFootTrans = FindInChildren(containerTrans, "lFoot");
            if (rFootTrans != null)
                rFoot = rFootTrans.parent.GetComponent<tntLink>();
            if (rFoot == null)
                rFoot = rAnkle;
        }

        // Ankle bones are connecting the knees and the feet. These are optional joints between the knee joints
        // and the feet joints. In some cases (e.g. BigBird) we already consider the ankle bones the "foot" bones
        // and we should leave these ankles as null. In some cases (e.g. Raptor) we consider the toe bones as the
        // "foot" bones and we have separate ankle bones specified here so that the engines know that there is
        // an extra hop between the feet and the knee.
        if (lAnkle == lFoot)
            lAnkle = null;
        if (rAnkle == rFoot)
            rAnkle = null;

        m_controller = TNT.acCreateBipedController(root.GetWorld(), root.GetBase(),
                                                   lFoot.GetIndex(), rFoot.GetIndex(),
                                                   lAnkle == null ? -1 : lAnkle.GetIndex(),
                                                   rAnkle == null ? -1 : rAnkle.GetIndex(),
                                                   lKnee == null ? -1 : lKnee.GetIndex(),
                                                   rKnee == null ? -1 : rKnee.GetIndex(),
                                                   lHip.GetIndex(), rHip.GetIndex(),
                                                   lowerBack == null ? -1 : lowerBack.GetIndex(),
                                                   torso == null ? -1 : torso.GetIndex(),
                                                   head == null ? -1 : head.GetIndex(),
                                                   stanceHipDamping,
                                                   stanceHipMaxVelocity,
                                                   rootPredictiveTorqueScale,
                                                   startingState,
                                                   (int)startingStance,
                                                   stepWidth,
                                                   isIKVMOn
                                                   );

        TNT.acSetMaxGyro(m_controller, maxGyro);
        TNT.acEnableRagdoll(m_controller, isRagdollOn);
        if (useExplicitPDControllers)
            TNT.acUseExplicitPDControl(m_controller);
        else
            TNT.acUseImplicitPDControl(m_controller);
        TNT.acUseImplicitPositionError(m_controller, useImplicitPositionError);
        TNT.acUseMOIAboutJointPosition(m_controller, useMOIAboutJointPosition);

        if (isIKVMOn)
            TNT.acSetIKVMMode(m_controller, isInvertedPendulumOn, isGravityCompensationOn);

        unsafe
        {
            fixed(void* pState = &controllerState.stance)
            {
                TNT.acInstallStateSensor(m_controller, pState);       
            }
        }

        TNT.acInitRootPdParams(m_controller, rootControlParams.controlled, rootControlParams.kp,
                                    rootControlParams.kd, rootControlParams.maxAbsTorque,
                                    rootControlParams.scale.x,
                                    rootControlParams.scale.y,
                                    rootControlParams.scale.z,
                                    rootControlParams.relToCharFrame);

        // Walk through articultion links in order and add ControlParams to each of them
        int[] indexMap = new int[root.numLinks()];      // Key: tntLink index  Value: PDParams index
        for (int i = 0; i < root.numLinks(); ++i)
            indexMap[i] = -1;
        for (int i = 0; i < controlParams.Count; ++i)
        {
            indexMap[root.NameToIndex(controlParams[i].name)] = i;
            Debug.Log(controlParams[i].name + " => " + root.NameToIndex(controlParams[i].name));
        }
        for (int i = 0; i < root.numLinks(); ++i)
        {
            PDParams param;
            if (indexMap[i] >= 0)
            {
                param = controlParams[indexMap[i]];

            } else
            {
                // Add a dumyy uncontrolled PDParam entry
                param = new PDParams();
                param.controlled = false;
            }
            TNT.acAddLinkPdParams(m_controller, param.controlled, param.kp,
                                   param.kd, param.maxAbsTorque,
                                   param.scale.x, param.scale.y, param.scale.z,
                                   param.relToCharFrame);
        }

        // Serialize Controller States
        for (int i = 0; i < states.Count; ++i)
        {
            BipedConState state = states[i];
            ResolveJoints(state, root);
            IntPtr engineState = TNT.acAddSimBiConState(
                m_controller, state.nextStateIndex,
                state.stateTime, state.maxGyro, state.reverseStance,
                state.keepStance, state.stateStance == StanceOrientation.LEFT_STANCE ? 0 : 1,
                state.transitionOnFootContact,
                state.minPhiBeforeTransitionOnFootContact,
                state.minSwingFootForceForContact);
            // serialize sExternalForces
            for (int j = 0; j < state.sExternalForces.Count; ++j)
            {
                ExternalForce force = state.sExternalForces[j];
                IntPtr externalForce = TNT.acAddExternalForce(engineState,
                                                              force.leftStanceIndex,
                                                              force.rightStanceIndex);
                force.serializeToEngine(externalForce);
            }

            // serialize sTraj
            for (int j = 0; j < state.sTraj.Count; ++j)
            {
                Trajectory traj = state.sTraj[j];
                IntPtr engineTraj = TNT.acAddTrajectory(engineState, traj.leftStanceIndex,
                                                        traj.rightStanceIndex,
                                                        traj.relToCharFrame);

                // serilize components
                for (int k = 0; k < traj.components.Count; ++k)
                {
                    TrajectoryComponent comp = traj.components[k];
                    IntPtr endgineFeedback = IntPtr.Zero;
                    LinearBalanceFeedback fb = comp.balanceFeedback;
                    if (comp.feedbackEnabled)
                        endgineFeedback = TNT.acCreateLinearBalanceFeedback(fb.feedbackProjectionAxis.x,
                                                                            fb.feedbackProjectionAxis.y,
                                                                            fb.feedbackProjectionAxis.z,
                                                                            fb.cd,
                                                                            fb.cv,
                                                                            fb.vMin,
                                                                            fb.vMax,
                                                                            fb.dMin,
                                                                            fb.dMax);

                    IntPtr compBaseTraj = TNT.acAddTrajectoryComponent(engineTraj,
                                                                     comp.reverseAngleOnLeftStance,
                                                                     comp.reverseAngleOnRightStance,
                                                                     comp.rotationAxis.x,
                                                                     comp.rotationAxis.y,
                                                                     comp.rotationAxis.z,
                                                                     endgineFeedback);
                    comp.baseTraj.serializeToEngine(compBaseTraj);
                }

                // serialize strengthTraj
                traj.strengthTraj.serializeToEngine(TNT.acGetStrengthTraj(engineTraj));
            }

            // serialize dTrajX, dTrajZ, vTrajX, vTrajZ
            state.dTrajX.serializeToEngine(TNT.acGetDTrajX(engineState));
            state.dTrajZ.serializeToEngine(TNT.acGetDTrajZ(engineState));
            state.vTrajX.serializeToEngine(TNT.acGetVTrajX(engineState));
            state.vTrajZ.serializeToEngine(TNT.acGetVTrajZ(engineState));
        }
        return true;
    }

    public override void SyncWithEngine()
    {
        for (int i = 0; i < states.Count; ++i)
        {
            BipedConState state = states[i];
			for (int j = 0; j < state.sExternalForces.Count; j++)
			    state.sExternalForces[j].syncWithEngine();
            state.vTrajX.syncWithEngine();
            state.vTrajZ.syncWithEngine();
        }
    }

 
    public void UpdateVelocity(int stateIndex, float forwardVel, float sideVel, float verticalVel)
    {
        //Debug.Log("Update velocity to:" + velocity);
        if (stateIndex >= states.Count)
            return;
        if (m_controller == IntPtr.Zero)
        {
            return;
        }
        Trajectory1D trajZ = states[stateIndex].vTrajZ;
        if (trajZ.getKnotCount () == 1)		// expect only a single knot velocity trajectory in the model
            trajZ.updateKnot(0, 0, forwardVel);

        Trajectory1D trajX = states[stateIndex].vTrajX;
        if (trajX.getKnotCount () == 1)     // expect only a single knot velocity trajectory in the model
            trajX.updateKnot(0, 0, sideVel);

        if (isIKVMOn)
        {
            // Also update the virtual force velocity target
            TNT.acSetDesiredVelocities(m_controller, forwardVel, sideVel, verticalVel);
        }
    }

    public void UpdateHeading(float heading)
    {
        // TBD: check against cached heading and only pInvoke if changed
        if (m_controller != IntPtr.Zero)
            TNT.acSetDesiredHeading(m_controller, heading);
    }

    public void EnableRagdoll(bool enable)
    {
        if (isRagdollOn != enable && m_controller != IntPtr.Zero)
        {
            isRagdollOn = enable;
            TNT.acEnableRagdoll(m_controller, enable);
        }
    }

    public void SetMaxGyro(float gyro)
    {
        if (maxGyro != gyro && m_controller != IntPtr.Zero)
        {
            maxGyro = gyro;
            TNT.acSetMaxGyro(m_controller, gyro);
        }
    }

    public void UseExplicitPDControllers(bool use)
    {
        if (useExplicitPDControllers != use && m_controller != IntPtr.Zero)
        {
            useExplicitPDControllers = use;
            if (useExplicitPDControllers)
                TNT.acUseExplicitPDControl(m_controller);
            else
                TNT.acUseImplicitPDControl(m_controller);
        }
    }

    public void UseImplicitPositionError(bool use)
    {
        if (useImplicitPositionError != use && m_controller != IntPtr.Zero)
        {
            useImplicitPositionError = use;
            TNT.acUseImplicitPositionError(m_controller, use);
        }
    }

    public void UseMOIAboutJointPosition(bool use)
    {
        if (useMOIAboutJointPosition != use || m_controller != IntPtr.Zero)
        {
            useMOIAboutJointPosition = use;
            TNT.acUseMOIAboutJointPosition(m_controller, use);
        }
    }

	public int getStatesCount()
	{
		return states.Count;
	}

	public int getCurrentState()
    {
        return controllerState.FSMStateIndex;
    }

    public int getCurrentNextState()
    {
        return states[controllerState.FSMStateIndex].nextStateIndex;
    }

    public void setCurrentNextState(int index, bool immediate)
    {
        if (index != getCurrentNextState() && m_controller != IntPtr.Zero)
        {
            TNT.acSetCurrentNextState(m_controller, index, immediate);
            states[controllerState.FSMStateIndex].nextStateIndex = index;
        }
    }

    public void setNextState(int curState, int nextState, bool immediate)
    {
        if (states[curState].nextStateIndex != nextState && m_controller != IntPtr.Zero)
        {
            TNT.acSetNextState(m_controller, curState, nextState, immediate);
            states[curState].nextStateIndex = nextState;
        }
    }

    public float GetForwardVelocity(int stateIndex)
    {
        if (stateIndex >= states.Count)
            return 0;
        Trajectory1D trajZ = states[stateIndex].vTrajZ;
        if (trajZ.getKnotCount () != 1)     // expect only a single knot velocity trajectory in the model
            return 0;
        return trajZ.getKnotValue(0);
    }

	// Assumptions on the Biped Controller under the control of this API: 
	//
	// 1) You have at least one SExternalForces element in the state of the controller where you want
	// to exert external forces ( you can manually add an SExernalForces element in Unity editor under the 
	// state of Biped controller )
	//
	// 2) if you are updating one of the 6 components of the external forces the trajectory of that 
	// component must have at least 1 element in the model. For example: if you want to apply
	// non-zero force.x you need to insert at least one element to the ExternalForce::forceX trajectory of the model.
	// You can do this in Unity inspector manually by entering "1" into the "Size" of the "TValues" and "Values"
	// array under the "Force X" component of the "Element 0" of "SExternal Forces")
	//
	public void UpdateExternalForce(int stateIndex, Vector3 force, Vector3 torque)
	{
		if (stateIndex >= states.Count)
			return;
		List<ExternalForce> forces = states[stateIndex].sExternalForces;
		if (forces == null || forces.Count < 1)
			return;
		ExternalForce externalForce = forces[0];
		if (externalForce.forceX.getKnotCount() > 0 && externalForce.forceX.updateKnot(0, 0, force.x))
			externalForce.MarkDirty();
		if (externalForce.forceY.getKnotCount() > 0 && externalForce.forceY.updateKnot(0, 0, force.y))
			externalForce.MarkDirty();
		if (externalForce.forceZ.getKnotCount() > 0 && externalForce.forceZ.updateKnot(0, 0, force.z))
			externalForce.MarkDirty();
		if (externalForce.torqueX.getKnotCount() > 0 && externalForce.torqueX.updateKnot(0, 0, torque.x))
			externalForce.MarkDirty();
		if (externalForce.torqueY.getKnotCount() > 0 && externalForce.torqueY.updateKnot(0, 0, torque.y))
			externalForce.MarkDirty();
		if (externalForce.torqueZ.getKnotCount() > 0 && externalForce.torqueZ.updateKnot(0, 0, torque.z))
			externalForce.MarkDirty();
	}

	public void GetExternalForce(int stateIndex, out Vector3 force, out Vector3 torque)
	{
		force = Vector3.zero;
		torque = Vector3.zero;
		if (stateIndex >= states.Count)
			return;
		List<ExternalForce> forces = states[stateIndex].sExternalForces;
		if (forces == null || forces.Count < 1)
			return;
		ExternalForce externalForce = forces[0];
		if (externalForce.forceX.getKnotCount() > 0)
			force.x = externalForce.forceX.getKnotValue(0);
		if (externalForce.forceY.getKnotCount() > 0)
			force.y = externalForce.forceY.getKnotValue(0);
		if (externalForce.forceZ.getKnotCount() > 0)
			force.z = externalForce.forceZ.getKnotValue(0);
		if (externalForce.torqueX.getKnotCount() > 0)
			torque.x = externalForce.torqueX.getKnotValue(0);
		if (externalForce.torqueY.getKnotCount() > 0)
			torque.y = externalForce.torqueY.getKnotValue(0);
		if (externalForce.torqueZ.getKnotCount() > 0)
			torque.z = externalForce.torqueZ.getKnotValue(0);
	}
}
