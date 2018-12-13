// ------------------------------------------------------------------------------
//  TNT_APE.cs
//      This the C# API to the Articulated Physics Engine
// ------------------------------------------------------------------------------
using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace PhysicsAPI
{
	[System.Serializable]
	public struct apJointFeedback
	{
		public float m_appliedImpulse;
		public float m_accumulatedImpulse;
		public bool m_broken;
	}    

	partial class TNT
	{
#if (UNITY_IOS || UNITY_WEBGL) && !UNITY_EDITOR
        // On iOS and Xbox 360 plugins are statically linked into
		// the executable, so we have to use __Internal as the
		// library name.
		public const bool UseVeneer = false;
        public const string KernalLibraryName = "__Internal";
#else
        // Other platforms load plugins dynamically, so pass the name
        // of the plugin's dynamic library.
        public const bool UseVeneer = false;
        public const string KernalLibraryName = UseVeneer ? "libArticulatedPhysicsVeneer64" : "libArticulatedPhysics";
#endif
        // SDK
        [DllImport (KernalLibraryName)] public static extern
            IntPtr apGetPhysicsSdk();
        [DllImport (KernalLibraryName)] public static extern
            IntPtr apNewPhysicsSdk();
		[DllImport (KernalLibraryName)] public static extern
			void apDeletePhysicsSdk(IntPtr physicsSDK);

        // math

        [DllImport(KernalLibraryName)]
        public static extern unsafe
        void apTransformInverse(Vector3* inPos, Quaternion* inRotation, Vector3* outPos, Quaternion* outRotation);

        [DllImport(KernalLibraryName)]
        public static extern unsafe
        void apTransformTimesTransform(Vector3* inPos1, Quaternion* inRotation1, Vector3* inPos2, Quaternion* inRotation2, Vector3* outPos, Quaternion* outRotation);

        [DllImport(KernalLibraryName)]
        public static extern unsafe
        void apTransformInverseTimesTransform(Vector3* inPos1, Quaternion* inRotation1, Vector3* inPos2, Quaternion* inRotation2, Vector3* outPos, Quaternion* outRotation);

        [DllImport(KernalLibraryName)]
        public static extern unsafe
        void apTransformTimesTransformInverse(Vector3* inPos1, Quaternion* inRotation1, Vector3* inPos2, Quaternion* inRotation2, Vector3* outPos, Quaternion* outRotation);


        // World
        [DllImport (KernalLibraryName)] public static extern
            IntPtr apCreateDynamicsWorld(IntPtr physicsSdk, int numIterations, float staticConvexDistanceMargin, float defaultErp, bool disableRandomness);
		[DllImport (KernalLibraryName)] public static extern
			void apDeleteDynamicsWorld(IntPtr world);

        [DllImport (KernalLibraryName)] public static extern
			void apResetDynamicsWorldCaches(IntPtr world);

        [DllImport (KernalLibraryName)] public static extern
			void apStepSimulation(IntPtr world, float timeStep, int maxSubSteps, float fixedTimeStep);
        [DllImport (KernalLibraryName)] public unsafe static extern
            bool apRayCast(IntPtr world, Vector3 *rayFrom, Vector3 *rayTo,
                           IntPtr* hitBody, IntPtr* hitTntBase, int* hitTntLinkIndex,
                           Vector3* hit, Vector3* hitNormal);
        [DllImport (KernalLibraryName)] public unsafe static extern
            bool apRayCast2(IntPtr world, Vector3 *rayFrom, Vector3 *direction,
                           float maxDistance, int layerMask,
                            Vector3* hit, Vector3* hitNormal, Vector3* hitBaryCentric, int* triangleIndex,
                            int* index, bool* isRigidBody, IntPtr* hitBody, IntPtr* hitTntBase, int* hitTntLinkIndex);

		[DllImport (KernalLibraryName)] public unsafe static extern
			void apSetCollisionInfoBuffers(CollisionInfo* pCIBuffer, int size1, ContactPointInfo* pCPIBuffer, int size2);
		[DllImport (KernalLibraryName)] public static extern
            void apAddListenerRigidBody(IntPtr rigidbody);
        [DllImport (KernalLibraryName)] public static extern
            void apRemoveListenerRigidBody(IntPtr rigidbody);
        [DllImport (KernalLibraryName)] public static extern
            void apAddListenerArticulation(IntPtr theBase);
        [DllImport(KernalLibraryName)] public static extern
            void apAddListenerArticulationLink(IntPtr theBase, int linkIndex);
        [DllImport (KernalLibraryName)] public static extern
            void apRemoveListenerArticulation(IntPtr theBase);
        [DllImport (KernalLibraryName)] public static extern
            void apRemoveListenerArticulationLink(IntPtr theBase, int linkIndex);
        [DllImport (KernalLibraryName)] public static extern
            void apSetRigidBodyLayerID(IntPtr rigidBody, int layer);
        [DllImport (KernalLibraryName)] public static extern
            void apSetArticulationLayerID(IntPtr theBase, int layer);
        [DllImport (KernalLibraryName)] public static extern
			void apSetLinkLayerID(IntPtr theBase, int linkIndex, int layer);
		[DllImport (KernalLibraryName)] public static extern
            void apResetCollisionCacheForRigidBody(IntPtr world, IntPtr rigidBody);
		[DllImport (KernalLibraryName)] public static extern
            void apResetCollisionCacheForArticulation(IntPtr world, IntPtr articulationBase);
		[DllImport (KernalLibraryName)] public static extern
            void apResetCollisionCacheForLink(IntPtr world, IntPtr articulationBase, int linkIndex);
		[DllImport (KernalLibraryName)] public static extern
			void apSetLayerCollision(int layer0, int layer1, [MarshalAs(UnmanagedType.I1)]bool collide);

		// Collision Shape
		[DllImport (KernalLibraryName)] public static extern
			IntPtr apNewSphereShape(float radius);
		[DllImport (KernalLibraryName)] public static extern
			IntPtr apNewBoxShape(float x, float y, float z);
		[DllImport (KernalLibraryName)] public static extern
			IntPtr apNewCapsuleShape(float radius, float height);
        [DllImport (KernalLibraryName)] public static extern
            IntPtr apNewCapsuleShapeX(float radius, float height); 
        [DllImport (KernalLibraryName)] public static extern
            IntPtr apNewCapsuleShapeZ(float radius, float height); 
		[DllImport (KernalLibraryName)] public static extern
			IntPtr apNewConeShape(float radius, float height);
        [DllImport (KernalLibraryName)] public static extern
            IntPtr apNewConeShapeX(float radius, float height);
        [DllImport (KernalLibraryName)] public static extern
            IntPtr apNewConeShapeZ(float radius, float height);
		[DllImport (KernalLibraryName)] public static extern
			IntPtr apNewCylinderShape(float radius, float height);
        [DllImport (KernalLibraryName)] public static extern
            IntPtr apNewCylinderShapeX(float radius, float height);
        [DllImport (KernalLibraryName)] public static extern
            IntPtr apNewCylinderShapeZ(float radius, float height);
		[DllImport (KernalLibraryName)] public static extern
			void apDeleteShape(IntPtr shape);

		[DllImport (KernalLibraryName)] public static extern
			IntPtr apNewTriangleMesh();
		[DllImport (KernalLibraryName)] public unsafe static extern
			void apAddTriangle(IntPtr mesh, Vector3* v0, Vector3* v1, Vector3* v2);
		[DllImport (KernalLibraryName)] public static extern
			IntPtr apNewConcaveMeshShape(IntPtr mesh, [MarshalAs(UnmanagedType.I1)]bool anchored);	
		[DllImport (KernalLibraryName)] public static extern
			void apDeleteConcaveMeshShape(IntPtr shape, [MarshalAs(UnmanagedType.I1)]bool anchored);
        [DllImport (KernalLibraryName)] public unsafe static extern
            IntPtr apNewHeightMapShape(int widthSamples, int lengthSamples, float* heightMap, float maxHeight);
		[DllImport (KernalLibraryName)] public static extern
			IntPtr apNewConvexHullShape();
		[DllImport (KernalLibraryName)] public static extern
			void apAddVertex(IntPtr convexHull, float x, float y, float z);

		[DllImport (KernalLibraryName)] public static extern
			IntPtr apNewCompoundShape();
        [DllImport (KernalLibraryName)] public unsafe static extern
            void apAddChildShape(IntPtr compoundShape, IntPtr childShape, Vector3* childPos, Quaternion* childOrn);
        [DllImport (KernalLibraryName)] public unsafe static extern
            void apRemoveChildShape(IntPtr compoundShape, IntPtr childShape);
        [DllImport (KernalLibraryName)] public static extern
			void apDeleteCompoundShape(IntPtr shape);

		[DllImport (KernalLibraryName)] public static extern
			void apSetScaling(IntPtr shape, float x, float y, float z);
		
		// Rigid Body
		[DllImport (KernalLibraryName)] public unsafe static extern
            IntPtr apCreateRigidBody(float* masses, float* inertias, Vector3* moi,
			                         IntPtr cshape, Vector3* pos, Quaternion *rotation,
			                         Vector3* linearVelocity, Vector3* angularVelocity,
                                     float friction, float rollingFriction, int frictionCombineMode,
                                     float restitution, int restitutionCombineMode);

        [DllImport (KernalLibraryName)] public static extern
            void apSetRigidBodyVelocity(IntPtr body, float x, float y, float z,
	                                    float wx, float wy, float wz);

        [DllImport (KernalLibraryName)] public static extern
            void apSetRigidBodyPosition(IntPtr body, float x, float y, float z,
			                            float qx, float qy, float qz, float qw);
        [DllImport(KernalLibraryName)]
        public unsafe static extern void apSetRigidBodyTransform(IntPtr body, Vector3* position, Quaternion* orientation);

        [DllImport(KernalLibraryName)]
        public unsafe static extern void apGetRigidBodyPosition(IntPtr body, Vector3* position);
        [DllImport(KernalLibraryName)]
        public unsafe static extern void apGetRigidBodyOrientation(IntPtr body, Quaternion* orientation);
        [DllImport(KernalLibraryName)]
        public unsafe static extern void apGetRigidBodyTransform(IntPtr body, Vector3* position, Quaternion* orientation);

        [DllImport (KernalLibraryName)] public static extern
			void apDeleteRigidBody(IntPtr body);

		[DllImport (KernalLibraryName)] public static extern
            void apAddRigidBody(IntPtr world, IntPtr rigidBody, [MarshalAs(UnmanagedType.I1)]bool collidable, int layerID);
		[DllImport (KernalLibraryName)] public static extern
			void apRemoveRigidBody(IntPtr world, IntPtr rigidBody);

		// Joint
		[DllImport (KernalLibraryName)] public unsafe static extern
			IntPtr apCreateHingeConstraint(IntPtr bodyA, IntPtr bodyB, Vector3* pivotA, Vector3* pivotB,
			                               Vector3* axisA, Vector3* axisB, float breakingImpulse, int overrideIterations,
			                               void* feedback);
		[DllImport (KernalLibraryName)] public unsafe static extern
			IntPtr apCreateP2PConstraint(IntPtr bodyA, IntPtr bodyB, Vector3* pivotA, Vector3* pivotB,
			                             float breakingImpulse, int overrideIterations,
			                             void* feedback);
		[DllImport (KernalLibraryName)] public unsafe static extern
			IntPtr apCreateFixedConstraint(IntPtr bodyA, IntPtr bodyB, Vector3* pivotA, Vector3* pivotB,
			                               float breakingImpulse, int overrideIterations, void* feedback);
		[DllImport (KernalLibraryName)] public static extern
			void apSetFixedConstraintPivotA(IntPtr constraint, float x, float y, float z);
		[DllImport (KernalLibraryName)] public static extern
			void apSetP2PConstraintPivotB(IntPtr constraint, float x, float y, float z);
		[DllImport (KernalLibraryName)] public unsafe static extern
			void apSetConstraintBreakingImpulse(IntPtr constraint, float breakingImpulse);
		[DllImport (KernalLibraryName)] public static extern
			void apDeleteConstraint(IntPtr constraint);
		
		[DllImport (KernalLibraryName)] public static extern
            void apAddConstraint(IntPtr world, IntPtr constraint, [MarshalAs(UnmanagedType.I1)]bool disableSelfCollision);
		[DllImport (KernalLibraryName)] public static extern
			void apRemoveConstraint(IntPtr world, IntPtr constraint);

		/** 
		 * @remark maxAppliedImpulse is kept for backward compatibility of the API and currently doesn't do anything
		 */
		[DllImport (KernalLibraryName)] public unsafe static extern 
            IntPtr apCreateArticulationBase(int numLinks, float* masses, float* inertias, Vector3* moi,
			                                IntPtr cshape, [MarshalAs(UnmanagedType.I1)]bool canSleep, [MarshalAs(UnmanagedType.I1)]bool multiDof,
                                            [MarshalAs(UnmanagedType.I1)]bool selfCollision, [MarshalAs(UnmanagedType.I1)]bool highDefIntegrator,
			                                [MarshalAs(UnmanagedType.I1)]bool useGlobalVel, int simulationFreqMultiplier,
                                            float linearDamping, float angularDamping,
                                            float maxAppliedImpulse, float maxCoordinateVelocity,
		                                    float friction, float rollingFriction, int frictionCombineMode, 
                                            float restitution, int restitutionCombineMode,
			                                Vector3* position, Quaternion *rotation);

        [DllImport(KernalLibraryName)] public unsafe static extern
            void apGetArticulationFullRawState(IntPtr baseLink, Vector3* basePos, Quaternion* baseOri,
                                            Vector3* baseVel, Vector3* baseAngVel,
                                            float* q, float* qd);

        [DllImport(KernalLibraryName)] public unsafe static extern 
            void apSetArticulationFullRawState(IntPtr baseLink, Vector3* basePos, Quaternion* baseOri,
                                            Vector3* baseVel, Vector3* baseAngVel,
                                            float* q, float* qd, [MarshalAs(UnmanagedType.I1)]bool updateArticulationTransformAndSenors);

        [DllImport(KernalLibraryName)] public static extern 
        void apUpdateArticulationTransformsAndSensors(IntPtr baseLink);

        [DllImport(KernalLibraryName)] public static extern
            int apGetArticulationTotalDofCountExcludingBase(IntPtr baseLink);

        [DllImport(KernalLibraryName)] public static extern
            int apGetArticulationTotalRawPosVarCountExcludingBase(IntPtr baseLink);

		[DllImport (KernalLibraryName)] public static extern
			void apSetBaseMass(IntPtr baseLink, float mass);

        [DllImport(KernalLibraryName)] public static extern
            void apSetLinkMass(IntPtr mobilizer, float mass);

        [DllImport(KernalLibraryName)] public static extern
            void apSetLinkCollisionShape(IntPtr mobilizer, IntPtr cshape);

        [DllImport(KernalLibraryName)] public unsafe static extern
        	void apComputeLinkMoIFromCollisionShape(
		        IntPtr mobilizer,
		        float *masses, float* inertias,
		        Vector3* pivotA, Vector3* axisA,
		        Vector3* pivotB, Vector3* axisB,
		        float *mass, Vector3* moi);

        [DllImport(KernalLibraryName)] public static extern
            void apSetBaseCollisionShape(IntPtr baseLink, IntPtr cshape);

        [DllImport(KernalLibraryName)] public unsafe static extern
            void apComputeBaseMoIFromCollisionShape(
                IntPtr baseLink,
                float* masses, float* inertias,
                float* mass, Vector3* moi);

        [DllImport(KernalLibraryName)] public static extern
            void apSetLinkRawVelocityVariable(IntPtr mobilizer, int dofIndex, float vel,
                                            [MarshalAs(UnmanagedType.I1)]bool updateArticulationTransformAndSenors);

        [DllImport(KernalLibraryName)] public static extern
            float apGetLinkRawVelocityVariable(IntPtr mobilizer, int dofIndex);

        [DllImport(KernalLibraryName)] public static extern
            void apSetLinkRawPosVariable(IntPtr mobilizer, int posVarIndex, float posVar,
                                        [MarshalAs(UnmanagedType.I1)]bool updateArticulationTransformAndSenors);

        [DllImport(KernalLibraryName)] public static extern
            float apGetLinkRawPosVariable(IntPtr mobilizer, int posVarIndex);

        [DllImport(KernalLibraryName)] public static extern
            void apSetLinkBreakingImpulseThreshold(IntPtr mobilizer, float breakingImpulseThreshold);        

        // This is a O(n) API where n is the number of links in the articulation (if !useCached)
        [DllImport(KernalLibraryName)] public unsafe static extern
            void apComputeLinkWorldVelocity(IntPtr mobilierData, 
                                            Vector3* linVel, Vector3* angVel,
                                            [MarshalAs(UnmanagedType.I1)]bool useCached);

        [DllImport(KernalLibraryName)] public static extern
            int apGetLinkDofCount(IntPtr mobilierData);
        [DllImport(KernalLibraryName)] public static extern
            int apGetLinkRawPosVarCount(IntPtr mobilierData);

        [DllImport(KernalLibraryName)] public static extern
            void apSetRigidBodyMass(IntPtr world, IntPtr rigidBody, float mass);

        [DllImport(KernalLibraryName)] public static extern
            void apSetRigidBodyCollisionShape(IntPtr rigidBody, IntPtr cshape);

        [DllImport(KernalLibraryName)] public unsafe static extern
            void apComputeRigidBodyMoIFromCollisionShape(
                IntPtr rigidBody,
                float* masses, float* inertias,
                float* mass, Vector3* moi);

        // Based on our experiments IL2CPP 64-bit pInvoke API only supports up to 20 parameters reliably
        // as of Unity 4.6.5
        [DllImport (KernalLibraryName)] public unsafe static extern 
            IntPtr apSetupArticulationLink(IntPtr articulatedBase, int jointType,
                                            [MarshalAs(UnmanagedType.I1)]bool parentCollision,
			                               	int myIndex, int parentIndex, int mirrorIndex,
                                            float* masses, float* inertias, Vector3* moi,
			                               	IntPtr cshape,
                                            float friction, float rollingFriction, int frictionCombineMode,
                                            float restitution, int restitutionCombineMode,
                                            Vector3* position, Quaternion* rotation,
                                            Vector3* pivotA, Vector3* axisA,
                                            Vector3* pivotB, Vector3* axisB,
                                            float* springStiffness, float* springDamping);

        [DllImport(KernalLibraryName)] public static extern 
            void apBreakArticulationLinkOff(IntPtr mobilizer);

        [DllImport (KernalLibraryName)] public unsafe static extern 
            void apInstallArticulationLinkSharedMemoryBuffers(
            IntPtr articulatedBase,
            int linkIndex,
            Vector3* position, Quaternion* rotation,
            Vector3* accumulatedForce, Vector3* accumulatedTorque,
            IntPtr worldIndex, float* currentPosition, float* currentVelocity,
            float* drag, float* angularDrag,
            void* feedback);
        [DllImport (KernalLibraryName)] public unsafe static extern 
            void apInstallRigidBodySharedMemoryBuffers(IntPtr rigidbody,
                                            Vector3* position, Quaternion* rotation,
                                            Vector3* accumulatedForce, Vector3* accumulatedTorque,
                                            IntPtr worldIndex, float* currentVelocity,                                     
                                            float* drag, float* angularDrag);
        [DllImport (KernalLibraryName)] public unsafe static extern 
            void apInstallArticulationBaseSharedMemoryBuffers(
            IntPtr linkBase,
            Vector3* position, Quaternion* rotation,
            Vector3* accumulatedForce, Vector3* accumulatedTorque,
            IntPtr worldIndex, float* currentVelocity,
            float* drag, float* angularDrag);
        
        [DllImport (KernalLibraryName)] public unsafe static extern 
            void apSetupFixedLink(IntPtr articulationBase,
                                  [MarshalAs(UnmanagedType.I1)]bool parentCollision, int myIndex, int parentIndex, int mirrorIndex,
                                  float *masses, float* inertias, Vector3* moi,
                                  IntPtr cshape, float friction, float rollingFriction, int frictionCombineMode,
                                  float restitution, int restitutionCombineMode, Vector3 *position, Quaternion *rotation);

		[DllImport (KernalLibraryName)] public static extern
			void apDeleteArticulation(IntPtr articulatedBase);
		[DllImport (KernalLibraryName)] public unsafe static extern
            void apAddArticulation(IntPtr world, IntPtr articulatedBase, bool* collidable, int layerID);
        [DllImport (KernalLibraryName)] public static extern
            void apAddOutOfSimArticulation(IntPtr articulatedBase);
		[DllImport (KernalLibraryName)] public static extern
			void apRemoveArticulation(IntPtr world, IntPtr articulatedBase);
        [DllImport (KernalLibraryName)] public unsafe static extern
            void apLinkPosToWorld(IntPtr articulationBase, int link, Vector3* localPos, 
                                  Vector3* worldPos);
        [DllImport (KernalLibraryName)] public unsafe static extern
            void apLinkDirToWorld(IntPtr articulationBase, int link,Vector3* localDir,
                                  Vector3* worldDir);
        [DllImport (KernalLibraryName)] public unsafe static extern
            void apWorldPosToLink(IntPtr articulationBase, int link, Vector3* worldPos,
                                  Vector3* localPos);
        [DllImport (KernalLibraryName)] public unsafe static extern
            void apWorldDirToLink(IntPtr articulationBase, int link, Vector3* worldDir,
                                  Vector3* localDir);
        
        [DllImport (KernalLibraryName)] public unsafe static extern
            void apComputeArticulationCoMPosition(IntPtr articulationBase, Vector3* comPos);
        [DllImport (KernalLibraryName)] public unsafe static extern
            void apComputeArticulationCoMVelocity(IntPtr articulationBase, Vector3* comVel);
        [DllImport (KernalLibraryName)] public unsafe static extern
            void apComputeArticulationAngularMomentum(IntPtr articulationBase, Vector3* angMom);
        [DllImport (KernalLibraryName)] public unsafe static extern
            void apComputeArticulationHeadingTransform(IntPtr articulationBase, Vector3* rootPlannarLocation, Quaternion* rootHeadingRotation);

        [DllImport (KernalLibraryName)] public static extern
            IntPtr apAddMotor(IntPtr world, IntPtr root, int link, int dof,
                                float desiredVelocity, float desiredPostion, float maxMotorForce,
                                [MarshalAs(UnmanagedType.I1)]bool positional,
                                float positionLockThreshold, [MarshalAs(UnmanagedType.I1)]bool useAutomaticPositionLockThreshold);
        [DllImport (KernalLibraryName)] public static extern
            void apDeleteMotor(IntPtr world, IntPtr motor, int dof);
        [DllImport (KernalLibraryName)] public static extern
            void apSetMotorIsPostional(IntPtr mobilizer, int dof, [MarshalAs(UnmanagedType.I1)]bool positional);
        [DllImport (KernalLibraryName)] public static extern
            void apSetMotorDesiredSpeed(IntPtr mobilizer, int dof, float speed);
        [DllImport (KernalLibraryName)] public static extern
            void apSetMotorDesiredPosition(IntPtr mobilizer, int dof, float position);
        [DllImport (KernalLibraryName)] public static extern
            void apSetMotorMaxForce(IntPtr mobilizer, int dof, float maxForce);
        [DllImport (KernalLibraryName)] public static extern
            void apSetMotorPositionLockThreshold(IntPtr mobilizer, int dof, float positionLockThreshold);
        [DllImport(KernalLibraryName)] public static extern
            void apSetUseAutomaticPositionLockThreshold(IntPtr mobilizer, int dof, [MarshalAs(UnmanagedType.I1)]bool useAutomaticPositionLockThreshold);

        [DllImport (KernalLibraryName)] public static extern
            IntPtr apAddJointLimits(IntPtr world, IntPtr root, int link, int dof, float limitLo,
                                    float limitHi, float maxForce);
        [DllImport (KernalLibraryName)] public static extern
            void apDeleteJointLimits(IntPtr world, IntPtr limits, int dof);
        [DllImport (KernalLibraryName)] public static extern
            void apSetJointLimitLo(IntPtr mobilierData, int dof, float limitLo);
        [DllImport (KernalLibraryName)] public static extern
            void apSetJointLimitHi(IntPtr mobilierData, int dof, float limitHi);
        [DllImport (KernalLibraryName)] public static extern
            void apSetJointLimitMaxForce(IntPtr mobilierData, int dof, float maxForce);
        [DllImport (KernalLibraryName)] public static extern
            void apSetSpringStiffness(IntPtr mobilizer, int dof, float stiffness);
        [DllImport (KernalLibraryName)] public static extern
            void apSetSpringDamping(IntPtr mobilizer, int dof, float damping);
        [DllImport (KernalLibraryName)] public static extern
            void apSetSpringNeutralPoint(IntPtr mobilizer, int dof, float neutralPoint);

        [DllImport(KernalLibraryName)] public static extern
            void apSetContinuousForceActuator(IntPtr mobilizer, int dof, float force);

        [DllImport (KernalLibraryName)] public unsafe static extern
            IntPtr apAddArticulationP2PLink(IntPtr world, IntPtr bodyA, int linkA, IntPtr bodyB,
                                            int linkB, Vector3* pivotInA, Vector3* pivotInB,
                                            float maxImpulse, float breakingImpulse,
			                                void* feedback);
        [DllImport (KernalLibraryName)] public unsafe static extern
            IntPtr apAddArticulationRigidBodyP2PLink(IntPtr world, IntPtr bodyA, int linkA,
                                                     IntPtr bodyB, Vector3* pivotInA,
                                                     Vector3* pivotInB, float maxImpulse,
			                                         float breakingImpulse,
			                                         void* feedback);
        [DllImport (KernalLibraryName)] public static extern
            void apSetP2PlinkPivotB(IntPtr p2pLink, float x, float y, float z);

		[DllImport (KernalLibraryName)] public static extern
			void apSetP2PLinkBreakingImpulse(IntPtr p2pLink, float breakingImpulse);

        [DllImport (KernalLibraryName)] public unsafe static extern
            IntPtr apAddArticulationFixedConstraint(IntPtr world, IntPtr bodyA, int linkA, IntPtr bodyB,
                                            int linkB, Vector3* pivotInA, Vector3* pivotInB,
                                            float maxImpulse, float breakingImpulse,
                                            void* feedback);
        [DllImport (KernalLibraryName)] public unsafe static extern
            IntPtr apAddArticulationRigidBodyFixedConstraint(IntPtr world, IntPtr bodyA, int linkA,
                                                     IntPtr bodyB, Vector3* pivotInA,
                                                     Vector3* pivotInB, float maxImpulse,
                                                     float breakingImpulse,
                                                     void* feedback);

        [DllImport (KernalLibraryName)] public static extern
            void apSetFixedConstraintBreakingImpulse(IntPtr fixedConstraint, float breakingImpulse);

        [DllImport (KernalLibraryName)] public static extern
			void apRemoveArticulationConstraint(IntPtr world, IntPtr constraint);

        [DllImport(KernalLibraryName)]
            public static extern void apDeleteArticulationConstraint(IntPtr world, IntPtr constraint);

        [DllImport (KernalLibraryName)] public static extern 
            void apSetRigidBodyKinematic(IntPtr world, IntPtr rigidbody, [MarshalAs(UnmanagedType.I1)]bool flag, float mass);
        [DllImport (KernalLibraryName)] public static extern 
            void apSetArticulationKinematic(IntPtr linkBase, [MarshalAs(UnmanagedType.I1)]bool flag, float mass);

        [DllImport (KernalLibraryName)] public unsafe static extern
			IntPtr apAddRopeJointForRigidBodyAndRigidBody(IntPtr bodyA, Vector3* pivotA,
			                                              IntPtr bodyB, Vector3* pivotB,
			                                              float stiffness, float damping,
			                                              float* maxLength, float* maxForce, int* broken, float* pWorldVel);
        [DllImport (KernalLibraryName)] public unsafe static extern
			IntPtr apAddRopeJointForRigidBodyAndLink(IntPtr body, Vector3* pivotA,
			                                         IntPtr theBase, int linkIndex, Vector3* pivotB,
			                                         float stiffness, float damping,
			                                         float* maxLength, float* maxForce, int* broken, float* pWorldVel); // negative linkIndex indicates this is base
        [DllImport (KernalLibraryName)] public unsafe static extern
			IntPtr apAddRopeJointForLinkAndLink(IntPtr baseA, int linkIndexA, Vector3* pivotA,
			                                    IntPtr baseB, int linkIndexB, Vector3* pivotB,
			                                    float stiffness, float damping,
			                                    float* maxLength, float* maxForce, int* broken, float* pWorldVel); // negative linkIndex indicates this is base
        [DllImport (KernalLibraryName)] public static extern
            void apRemoveRopeJoint(IntPtr ropeJoint);

        // trigger status
        [DllImport (KernalLibraryName)] public static extern
            void apSetRigidBodyTriggerStatus(IntPtr rigidBody, [MarshalAs(UnmanagedType.I1)]bool flag);
        [DllImport (KernalLibraryName)] public static extern
            void apSetLinkTriggerStatus(IntPtr a_Base, int linkIndex, [MarshalAs(UnmanagedType.I1)]bool flag);

        // overlap sphere
        [DllImport (KernalLibraryName)] public unsafe static extern
            int apOverlapSphere(IntPtr world, Vector3* origin, float radius, int filterLayer, int maxNumColliders, int* overlappingColliderWorldIndexArray);

        // add force
        [DllImport (KernalLibraryName)] public unsafe static extern
            void apAddOneTimeForceToRigidBody(IntPtr rigidBody, Vector3* force, Vector3* posWorld, [MarshalAs(UnmanagedType.I1)]bool ignoreWorldPos);
        [DllImport (KernalLibraryName)] public unsafe static extern
            void apAddOneTimeForceToMultiBody(IntPtr theBase, int linkIndex, Vector3* force, Vector3* posWorld, [MarshalAs(UnmanagedType.I1)]bool ignoreWorldPos);
        [DllImport (KernalLibraryName)] public unsafe static extern
            void apAddOneTimeTorqueToRigidBody(IntPtr rigidBody, Vector3* torque);
        [DllImport (KernalLibraryName)] public unsafe static extern
            void apAddOneTimeTorqueToMultiBody(IntPtr theBase, int linkIndex, Vector3* torque);

        // statistics
        [DllImport (KernalLibraryName)] public unsafe static extern
            void apSetStatsBuffer(PhysicsStatsInfo* pStatsInfo);

        // gravity
        [DllImport (KernalLibraryName)] public unsafe static extern
            void apSetGravity(IntPtr world, Vector3* gravity);
        [DllImport (KernalLibraryName)] public unsafe static extern
            void apGetGravity(IntPtr world, Vector3* gravity);

        [DllImport(KernalLibraryName)] public static extern
            void apSetDynamicsWorldSolverIterationsNumber(IntPtr world, int numIterations);

        // added by adrian (and available in kernel api)
        [DllImport(KernalLibraryName)]
        public static extern
            void apActivateKinematicRigidBody(IntPtr rigidBody);

        // added by adrian (and available in kernel api)
        [DllImport(KernalLibraryName)]
        public static extern
            void apActivateKinematicArticulation(IntPtr baseLink);

        [DllImport(KernalLibraryName), Obsolete("apSetDefaultSolverFidelityIndex should now be used")]
        public static extern
            void apEnableDirectSolver(IntPtr worldHandle,  [MarshalAs(UnmanagedType.I1)]bool enable);

        [DllImport(KernalLibraryName)]
        public static extern
            void apSetDefaultSolverFidelityIndex(IntPtr worldHandle, int fidelityIndex);

        [DllImport(KernalLibraryName)]
        public static extern
            void apSetArticulationRequiredSolverFidelityIndex(IntPtr baseHandle, int fidelityIndex);

        [DllImport(KernalLibraryName)]
        public static extern
            int apGetNumberOfAvailableFidelityIndices();
    }
}
