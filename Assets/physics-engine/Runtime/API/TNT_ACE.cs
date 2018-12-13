// ------------------------------------------------------------------------------
//  TNT_ACE.cs
//      This the C# API to the Articulated Physics Engine
// ------------------------------------------------------------------------------
using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace PhysicsAPI
{
	public partial class TNT       // FIXME: ugly but I need it to be public "for now" to access TNT.SamconStateVerifierMode
	{
        // Experiment shows that IL2CPP 64-bit pInvoke only pass the first 20 parameters correctly
        [DllImport (KernalLibraryName)] public static extern 
            IntPtr acCreateBipedController(IntPtr world, IntPtr character,
                                           int lFoot, int rFoot,
			                               int lAnkle, int rAnkle,
                                           int lKnee, int rKnee,
                                           int lHip, int rHip,
                                           int lowerBack, int torso, int head,
                                           float stanceHipDamping,
                                           float stanceHipMaxVelocity,
                                           float rootPredictiveTorqueScale,
                                           int staringState,
                                           int startingStance,
                                           float stepWidth,
                                           [MarshalAs(UnmanagedType.I1)]bool isIKVM
                                           );

		[DllImport (KernalLibraryName)] public unsafe static extern 
			IntPtr acCreateTRexController(IntPtr world,
		                                  IntPtr character,
                                          int torso, int lHand, int rHand, int lToes, int rToes, int head, int tail,
										  float* controlParams, int numControlParams,	// Note: this must match the scalar type in Engine
			                              float* pStartPose,
                                          float* pInitDesiredPose,
                                          [MarshalAs(UnmanagedType.I1)]bool keepRootPos,
                                          [MarshalAs(UnmanagedType.I1)]bool constraintPDFix
										  );

        [DllImport(KernalLibraryName)]
        public unsafe static extern
            IntPtr acCreateHumanoidController(IntPtr world,
                                              IntPtr character,
                                              int lowerBack, int upperBack, int neck, int lShoulder, int rShoulder,
                                              int lToes, int rToes, int lHand, int rHand,
                                              float limbTrackingKp, float limbTrackingKd, float limbTrackingMaxForce,
                                              float* controlParams, int numControlParams, // Note: this must match the scalar type in Engine
                                              float* pStartPose,
                                              float* pInitDesiredPose,
                                              float blendGranularity,
                                              [MarshalAs(UnmanagedType.I1)]bool keepRootPos,
                                              [MarshalAs(UnmanagedType.I1)]bool antiLegCrossing,
                                              [MarshalAs(UnmanagedType.I1)]bool useBlendSpace,
                                              [MarshalAs(UnmanagedType.I1)]bool enableGRFSolver,
                                              [MarshalAs(UnmanagedType.I1)]bool stepRelativeToCOM,
                                              [MarshalAs(UnmanagedType.I1)]bool stepRelativeToRoot,
                                              [MarshalAs(UnmanagedType.I1)]bool constraintPDFix,
                                              Vector3* forward,
                                              Vector3* right
                                            );
        [DllImport(KernalLibraryName)] public unsafe static extern
        IntPtr acCreatePoseController(IntPtr world, IntPtr articulation,          
                                      [MarshalAs(UnmanagedType.I1)]bool constraintPDFix);

        [DllImport(KernalLibraryName)] public unsafe static extern
        void acAddPoseControlChain(IntPtr poseController, int endEffector, int root, float* targetPosRot);

        [DllImport(KernalLibraryName)]
        public unsafe static extern
            IntPtr acCreateDogController(IntPtr world,
                                          IntPtr character,
                                          int torso, int lHand, int rHand, int lToes, int rToes, int head, int tail,
                                          float limbTrackingKp, float limbTrackingKd, float limbTrackingMaxforce,
                                          float* controlParams, int numControlParams, // Note: this must match the scalar type in Engine
                                          float* pStartPose,
                                          float* pInitDesiredPose,
                                          float blendGranularity,
                                          [MarshalAs(UnmanagedType.I1)]bool keepRootPos,
                                          [MarshalAs(UnmanagedType.I1)]bool useBlendSpace,
                                          [MarshalAs(UnmanagedType.I1)]bool enableGRFSolver,                                       [MarshalAs(UnmanagedType.I1)]bool constraintPDFix,
			                              Vector3* forward,
			                              Vector3* right
                                          );

        [DllImport(KernalLibraryName)] public unsafe static extern
        void acInstallCommonControllerSharedMemoryBuffers(IntPtr controller,
                                        float* currentPose,
                                        float* desiredPose,
                                        float* controlParams);

        [DllImport(KernalLibraryName)]
        public unsafe static extern
        void acInstallTRexControllerSharedMemoryBuffers(IntPtr controller,
                                void* pTRexStateSensor,
                                Vector3* lEEPos, Vector3* rEEPos);

        [DllImport(KernalLibraryName)] public unsafe static extern
        void acInstallHumanoidControllerSharedMemoryBuffers(IntPtr controller,
                                void* pHumanoidStateSensor);

        [DllImport(KernalLibraryName)]
        public unsafe static extern
        void acInstallDogControllerSharedMemoryBuffers(IntPtr controller,
                                void* pDogStateSensor);

        [DllImport (KernalLibraryName)] public static extern
            void acSetIKVMMode(IntPtr controller, [MarshalAs(UnmanagedType.I1)]bool isInvertedPendulumOn, [MarshalAs(UnmanagedType.I1)]bool isGravityCompensationOn);

        [DllImport (KernalLibraryName)] public static extern 
            void acRegisterController(IntPtr controller);
        [DllImport (KernalLibraryName)] public static extern 
            void acUnregisterController(IntPtr controller);
        [DllImport (KernalLibraryName)] public static extern 
            void acDeleteController(IntPtr controller);

        [DllImport (KernalLibraryName)] public static extern
            float acGetDesiredHeading(IntPtr controller);

        [DllImport (KernalLibraryName)] public static extern
            void acSetDesiredHeading(IntPtr controller, float desiredHeading);

		[DllImport (KernalLibraryName)] public static extern
			void acSetDesiredVelocities(IntPtr controller, float forwardVel, float sideVel, float upVel);

        [DllImport(KernalLibraryName)] public static extern
            void acEnableNeckControl(IntPtr controller, [MarshalAs(UnmanagedType.I1)]bool enabled);

		[DllImport(KernalLibraryName)] public static extern
			void acEnableTorsoControl(IntPtr controller, [MarshalAs(UnmanagedType.I1)]bool enabled);

        [DllImport(KernalLibraryName)] public static extern
            void acEnableRagdoll(IntPtr controller, [MarshalAs(UnmanagedType.I1)]bool enable);

        [DllImport(KernalLibraryName)] public static extern
            void acActiviateSupportLeg(IntPtr controller, int legIndex, [MarshalAs(UnmanagedType.I1)]bool enabled);

        [DllImport(KernalLibraryName)] public static extern
            void acEnableLeftShoulderSwing(IntPtr controller, [MarshalAs(UnmanagedType.I1)]bool enabled);

        [DllImport(KernalLibraryName)] public static extern
            void acEnableRightShoulderSwing(IntPtr controller, [MarshalAs(UnmanagedType.I1)]bool enabled);

        [DllImport (KernalLibraryName)] public static extern
            void acSetMaxGyro(IntPtr controller, float gyro);

        [DllImport (KernalLibraryName)] public static extern
            void acUseExplicitPDControl(IntPtr controller, int linkIdx = -1);

        [DllImport(KernalLibraryName)] public static extern
            void acUseImplicitPDControl(IntPtr controller, int linkIdx = -1);

        [DllImport(KernalLibraryName)] public static extern
            void acUseConstraintBasedPDControl(IntPtr controller, int linkIdx = -1);

        [DllImport(KernalLibraryName)] public static extern
            void acUseExplicitWithConstrainedBasedDampingPDControl(IntPtr controller, int linkIdx = -1);

        [DllImport (KernalLibraryName)] public static extern
            void acUseImplicitPositionError(IntPtr controller, [MarshalAs(UnmanagedType.I1)]bool use);

        [DllImport (KernalLibraryName)] public static extern
            void acUseMOIAboutJointPosition(IntPtr controller, [MarshalAs(UnmanagedType.I1)]bool use);

        [DllImport (KernalLibraryName)] public unsafe static extern
            void acInstallStateSensor(IntPtr controller, void* pState);

        [DllImport (KernalLibraryName)] public static extern
            int acGetCurrentState(IntPtr controller);

        [DllImport (KernalLibraryName)] public static extern
            void acSetNextState(IntPtr controller, int curState, int nextState, [MarshalAs(UnmanagedType.I1)]bool immediate);
        
        [DllImport (KernalLibraryName)] public static extern
            void acSetCurrentNextState(IntPtr controller, int nextState, [MarshalAs(UnmanagedType.I1)]bool immeidate);

        [DllImport(KernalLibraryName)]
        public unsafe static extern
            void acAddHumanoidBlendSample(IntPtr controller,
                                          float* controlParams,
                                          float fwdVel,
                                          float sideVel,
                                          float turnVel
                                          );

        [DllImport (KernalLibraryName)] public static extern
            void  acInitRootPdParams(  IntPtr controller,
	                                    [MarshalAs(UnmanagedType.I1)]bool controlled,
	                                    float kp, float kd,
	                                    float maxAbsTorque,
	                                    float scaleX, float scaleY, float scaleZ,
	                                    [MarshalAs(UnmanagedType.I1)]bool relToCharFrame
	                                    );
        
        [DllImport (KernalLibraryName)] public static extern
            void  acAddLinkPdParams(   IntPtr controller,
                                    [MarshalAs(UnmanagedType.I1)]bool controlled,
                                    float kp, float kd,
                                    float maxAbsTorque,
                                    float scaleX, float scaleY, float scaleZ,        
                                    [MarshalAs(UnmanagedType.I1)]bool relToCharFrame
                                	);
                                    
        [DllImport (KernalLibraryName)] public unsafe static extern
            void acAddPdParamSet (IntPtr controller, 
                                  float* pdParamSet, 
                                  int numLinks
                                  );

		[DllImport (KernalLibraryName)] public static extern
			void acUpdateLinkPdParams( IntPtr controller,
									   int index,
								       float kp, float kd,
									   float maxAbsTorque
			                          );

        [DllImport (KernalLibraryName)] public static extern
            IntPtr acAddSimBiConState(
                IntPtr controller,
                int nextStateIndex,
                float stateTime,
                float maxGyro,
                [MarshalAs(UnmanagedType.I1)]bool reverseStance,
                [MarshalAs(UnmanagedType.I1)]bool keepStance,
                int stateStance,
                [MarshalAs(UnmanagedType.I1)]bool transitionOnFootContact,
                float minPhi,
                float minForce
            );

        [DllImport (KernalLibraryName)] public static extern
            IntPtr acAddTrajectory(IntPtr state, int leftStanceIndex, int rightStanceIndex, [MarshalAs(UnmanagedType.I1)]bool relToCharFrame);

        [DllImport (KernalLibraryName)] public static extern
            IntPtr acCreateLinearBalanceFeedback(float axisX, float axisY, float axisZ,
                float cd, float cv, float vMin, float vMax, float dMin, float dMax);

        [DllImport (KernalLibraryName)] public static extern
            IntPtr acAddTrajectoryComponent(IntPtr trajectory, [MarshalAs(UnmanagedType.I1)]bool reverseAngleOnLeftStance,
                [MarshalAs(UnmanagedType.I1)]bool reverseAngleOnRightStance, float axisX, float axisY, float axisZ,
                IntPtr bFeedback);

        [DllImport (KernalLibraryName)] public static extern
            IntPtr acGetStrengthTraj(IntPtr trajectory);

        [DllImport (KernalLibraryName)] public static extern
            IntPtr acGetDTrajX(IntPtr state);
        [DllImport (KernalLibraryName)] public static extern
            IntPtr acGetDTrajZ(IntPtr state);
        [DllImport (KernalLibraryName)] public static extern
            IntPtr acGetVTrajX(IntPtr state);
        [DllImport (KernalLibraryName)] public static extern
            IntPtr acGetVTrajZ(IntPtr state);

		[DllImport (KernalLibraryName)] public static extern
			IntPtr acAddExternalForce(IntPtr state, int leftStanceIndex, int rightStanceIndex);

		[DllImport (KernalLibraryName)] public static extern
			IntPtr acGetForceXTraj(IntPtr force);
		[DllImport (KernalLibraryName)] public static extern
			IntPtr acGetForceYTraj(IntPtr force);
		[DllImport (KernalLibraryName)] public static extern
			IntPtr acGetForceZTraj(IntPtr force);
		[DllImport (KernalLibraryName)] public static extern
			IntPtr acGetTorqueXTraj(IntPtr force);
		[DllImport (KernalLibraryName)] public static extern
			IntPtr acGetTorqueYTraj(IntPtr force);
		[DllImport (KernalLibraryName)] public static extern
			IntPtr acGetTorqueZTraj(IntPtr force);

		[DllImport (KernalLibraryName)] public static extern
			void acAddTrajectory1DElement(IntPtr traj, float t, float value);

        [DllImport (KernalLibraryName)] public unsafe static extern
            void acUpdateTrajectory1D(IntPtr handle, int nKnot, float* tValue, float* vValue);

        [DllImport (KernalLibraryName)] public unsafe static extern
            void acSetDesiredPose(IntPtr controller, float* q);
        [DllImport (KernalLibraryName)] public unsafe static extern
            void acSetCurrentPose(IntPtr controller, float* q);
        [DllImport (KernalLibraryName)] public unsafe static extern
            void acSetReducedVirtualAgentState(IntPtr controller, float* q, [MarshalAs(UnmanagedType.I1)]bool keepRootPosAndOri);
        [DllImport (KernalLibraryName)] public unsafe static extern
            void acSetMirroredReducedVirtualAgentState(IntPtr controller, float* q, [MarshalAs(UnmanagedType.I1)]bool keepRootPosAndOri);

	    [DllImport (KernalLibraryName)] public unsafe static extern
            void acGetCOM(IntPtr controller, Vector3 *com);

        [DllImport(KernalLibraryName)] public static extern
        void acSetControllerDead(IntPtr controller, [MarshalAs(UnmanagedType.I1)]bool dead);

        //[DllImport (KernalLibraryName)] public unsafe static extern
        //    void acTestIK(IntPtr controller, int endEffector, int IKRoot, Vector3* targetPosWS, Vector3* outputPosWS, int* pNum);

        [DllImport (KernalLibraryName)] public unsafe static extern
            IntPtr acRegisterIKActuator(IntPtr controller, int endEffector, int IKRoot, float* targetPosWS, int* status);
        [DllImport (KernalLibraryName)] public unsafe static extern
            void acUnregisterIKActuator(IntPtr controller, IntPtr act);
        [DllImport (KernalLibraryName)] public unsafe static extern
            void acSetIKActuatorToleranceThreshold(IntPtr actuator, float tolerance);

        //Samcon
        [DllImport (KernalLibraryName)] public unsafe static extern
            IntPtr acLoadBVH(void* bvhPath, int* numJoints, int* numFrames, float* frameTime);

        [DllImport (KernalLibraryName)] public unsafe static extern
            IntPtr acLoadBVHFromMemory(void* content, int* numJoints, int* numFrames, float* frameTime);

        [DllImport (KernalLibraryName)] public unsafe static extern
            void acGetBasicBVHInfo(IntPtr bvhHandle, int* numJoints, int* numFrames, float* frameTime);

        [DllImport (KernalLibraryName)] public static extern
            void acMoveBVHFrameTransformationsToSkeleton(IntPtr bvhHandle, int frame);

        [DllImport(KernalLibraryName)] public static extern
            void acDeleteBVH(IntPtr bvhHandle);

        [DllImport(KernalLibraryName)] public static extern
            uint acGetBVHJointNameSize(IntPtr bvhHandle, int jointIndex);

        [DllImport(KernalLibraryName)] public unsafe static extern
            void acGetBVHJointName(IntPtr bvhHandle, int jointIndex, void* nameBuffer);

        [DllImport(KernalLibraryName)] public unsafe static extern
            void acGetBVHSkeletonTransforms(IntPtr bvhHandle, float* outputPos, float* outputRot);

        [DllImport(KernalLibraryName)] public unsafe static extern
            void acGetBVHFrameOrdered(IntPtr bvhHandle, float frame, int local, float* outputPos, float* outputRot);

        [DllImport(KernalLibraryName)] public unsafe static extern
            void acGetBVHJointHierachy(IntPtr bvhHandle, int* joints);

#if NOT_USED
        [DllImport(KernalLibraryName)] public static extern
            IntPtr acControllerUsePoseMotorsForImplicitDamping(IntPtr controller, [MarshalAs(UnmanagedType.I1)]bool use);
#endif

        [DllImport(KernalLibraryName)] public static extern
            void acResetControllerTorques(IntPtr controller);

        public unsafe delegate int EndEffectorDelegate(int linkIndex, Vector3* pEELinkRelativePositions, Quaternion* pEELinkRelativeOrientations);  // Unity fills in, kernel manages buffers' mem, kernel ises contents
        public unsafe delegate void PerDoFSamplingWindowDelegate(int linkIndex, Vector3* pWnd);
        public unsafe delegate bool StateVerificationDelegate(float *pToBeBeckedReducedStateVectorValues, float *pRefReducedStateVectorValues);     // kernel fills in, kernel manages buffer mem, Unity uses contents
        public unsafe delegate bool PostSingleSampleCallbackDelegate(int trialIndex, int trialIterationIndex, int sampleIndex, float sampleCost, float sampleTotalSubCost);
        public unsafe delegate bool PostSingleIterationCallbackDelegate(int trialIndex, int trialIterationIndex, float bestSampleCost, float bestSampleTotalSubCost);

        public enum SamconRandomizerType : int
        {
            Uniform = 0,
            Normal = 1,
            None = 2
        }

        public enum SamconCharacterStartPoseMode : int
        {
            ExplicitlyProvidedPose = 0,
            FirstReferenceClipFrame = 1,
            FirstTargetsClipFrame = 2
        }

        public enum SamconStateVerifierMode : int
        {
            RootOrientation_Internal = 0,
            CoMHeight_Internal = 1,
            AlwaysPass_Internal = 2,
            AlwaysFail_Internal = 3,
            External = 4
        }

        [DllImport(KernalLibraryName)][return: MarshalAs(UnmanagedType.I1)] public static extern
            bool acApplyAveragingProcessorToMotionClip(
            int numOfRuns, [MarshalAs(UnmanagedType.I1)]bool preserveFrameDuration,
            IntPtr inputClip, IntPtr outputClip);

        [DllImport(KernalLibraryName)][return: MarshalAs(UnmanagedType.I1)] public static extern
            bool acApplyFwdDiffProcessorToMotionClip(
            IntPtr inputClip, IntPtr outputClip);
        
        [DllImport(KernalLibraryName)][return: MarshalAs(UnmanagedType.I1)] public static extern
            bool acApplyInverseDynamicsProcessorToMotionClip(
            IntPtr world, IntPtr character, IntPtr controller, IntPtr inputClip, IntPtr outputClip,
            [MarshalAs(UnmanagedType.LPStr)] string configStr
            );

        [DllImport(KernalLibraryName)][return: MarshalAs(UnmanagedType.I1)] public static extern unsafe
            bool acApplyLoopingProcessorToMotionClip(
            IntPtr inputClip, IntPtr outputClip,
            int numReRuns, int numExtraFrameToReRun,
            [MarshalAs(UnmanagedType.I1)]bool withMirror, int* parentIds, int* mirrorIds
            );

        [DllImport(KernalLibraryName)][return: MarshalAs(UnmanagedType.I1)] public static extern
            bool acApplyResamplingProcessorToMotionClip(
            IntPtr inputClip, IntPtr outputClip,
            int frequency, [MarshalAs(UnmanagedType.I1)]bool preserveClipDuration, [MarshalAs(UnmanagedType.I1)]bool preserveLastFrame);
        
        [DllImport(KernalLibraryName)][return: MarshalAs(UnmanagedType.I1)] public static extern
            bool acApplyClampingProcessorToMotionClip(
            IntPtr inputClip, IntPtr outputClip,
            int startIdx, int endIdx);


        [DllImport(KernalLibraryName)][return: MarshalAs(UnmanagedType.I1)] public static extern
            bool acApplyFilteringProcessorToMotionClip(IntPtr inputClip, IntPtr outputClip, [MarshalAs(UnmanagedType.LPStr)]string configStr);

        [DllImport(KernalLibraryName)] public unsafe static extern
            void acSetArticulationState(IntPtr baseHandle, float* state);

        // 3 Point Tracking
        [DllImport(KernalLibraryName)] public unsafe static extern
        void acUpdateControllerState(
            IntPtr controller,
            Vector3* headPos, Quaternion* headRotation,
            Vector3* lHandPos, Quaternion* lHandRotation,
            Vector3* rHandPos, Quaternion* rHandRotation,
            Vector3* lFootPos, Quaternion* lFootRotation,
            Vector3* rFootPos, Quaternion* rFootRotation,
            Vector3* rootPos, Quaternion* rootRotation,
            float limbTrackingKp,
            float limbTrackingKd,
            float limbMaxTrackingForce,
            float deathThresholdLegSwing,
            float deathThresholdGRF,
            [MarshalAs(UnmanagedType.I1)]bool isDead,
            [MarshalAs(UnmanagedType.I1)]bool antiLegCrossing,
            [MarshalAs(UnmanagedType.I1)]bool stepRelativeToCOM,
            [MarshalAs(UnmanagedType.I1)]bool stepRelativeToRoot
        );

        [DllImport(KernalLibraryName)] public unsafe static extern
            void acSetPoseControllerToleranceThreshold(IntPtr controller, float tolerance);
    }
}
