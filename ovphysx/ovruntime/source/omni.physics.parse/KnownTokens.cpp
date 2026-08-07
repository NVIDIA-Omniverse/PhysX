// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-CORE-002
 * @covers AC-2
 *
 * @implements REQ-PARSE-JOINT-002
 * @covers AC-2
 */

#include <omni/physics/parse/KnownTokens.h>
#include <omni/physics/parse/IPhysicsSource.h>

namespace omni::physics::parse
{

void KnownTokens::intern(const IPhysicsSource& source)
{
    physicsMaterialAPI = source.internToken("PhysicsMaterialAPI");
    physxMaterialAPI = source.internToken("PhysxMaterialAPI");
    staticFriction = source.internToken("physics:staticFriction");
    dynamicFriction = source.internToken("physics:dynamicFriction");
    restitution = source.internToken("physics:restitution");
    density = source.internToken("physics:density");
    frictionCombineMode = source.internToken("physxMaterial:frictionCombineMode");
    restitutionCombineMode = source.internToken("physxMaterial:restitutionCombineMode");
    dampingCombineMode = source.internToken("physxMaterial:dampingCombineMode");
    compliantContactAccelerationSpring = source.internToken("physxMaterial:compliantContactAccelerationSpring");
    compliantContactStiffness = source.internToken("physxMaterial:compliantContactStiffness");
    compliantContactDamping = source.internToken("physxMaterial:compliantContactDamping");

    average = source.internToken("average");
    min = source.internToken("min");
    max = source.internToken("max");
    multiply = source.internToken("multiply");

    physicsScene = source.internToken("PhysicsScene");
    gravityDirection = source.internToken("physics:gravityDirection");
    gravityMagnitude = source.internToken("physics:gravityMagnitude");

    // PhysxSceneAPI extensions
    physxSceneAPI = source.internToken("PhysxSceneAPI");
    physxSceneUpdateType = source.internToken("physxScene:updateType");
    physxSceneBounceThreshold = source.internToken("physxScene:bounceThreshold");
    physxSceneFrictionOffsetThreshold = source.internToken("physxScene:frictionOffsetThreshold");
    physxSceneFrictionCorrelationDistance = source.internToken("physxScene:frictionCorrelationDistance");
    physxSceneMaxBiasCoefficient = source.internToken("physxScene:maxBiasCoefficient");
    physxSceneTimeStepsPerSecond = source.internToken("physxScene:timeStepsPerSecond");
    physxSceneMinPositionIterationCount = source.internToken("physxScene:minPositionIterationCount");
    physxSceneMaxPositionIterationCount = source.internToken("physxScene:maxPositionIterationCount");
    physxSceneMinVelocityIterationCount = source.internToken("physxScene:minVelocityIterationCount");
    physxSceneMaxVelocityIterationCount = source.internToken("physxScene:maxVelocityIterationCount");
    physxSceneEnableCCD = source.internToken("physxScene:enableCCD");
    physxSceneEnableStabilization = source.internToken("physxScene:enableStabilization");
    physxSceneEnableGPUDynamics = source.internToken("physxScene:enableGPUDynamics");
    physxSceneEnableEnhancedDeterminism = source.internToken("physxScene:enableEnhancedDeterminism");
    physxSceneEnableExternalForcesEveryIteration = source.internToken("physxScene:enableExternalForcesEveryIteration");
    physxSceneInvertCollisionGroupFilter = source.internToken("physxScene:invertCollisionGroupFilter");
    physxSceneReportKinematicKinematicPairs = source.internToken("physxScene:reportKinematicKinematicPairs");
    physxSceneReportKinematicStaticPairs = source.internToken("physxScene:reportKinematicStaticPairs");
    physxSceneEnableSceneQuerySupport = source.internToken("physxScene:enableSceneQuerySupport");
    physxSceneSolveArticulationContactLast = source.internToken("physxScene:solveArticulationContactLast");
    physxSceneDisableSleeping = source.internToken("physxScene:disableSleeping");
    physxSceneCollisionSystem = source.internToken("physxScene:collisionSystem");
    physxSceneSolverType = source.internToken("physxScene:solverType");
    physxSceneBroadphaseType = source.internToken("physxScene:broadphaseType");
    physxSceneFrictionType = source.internToken("physxScene:frictionType");
    physxSceneGpuTempBufferCapacity = source.internToken("physxScene:gpuTempBufferCapacity");
    physxSceneGpuMaxRigidContactCount = source.internToken("physxScene:gpuMaxRigidContactCount");
    physxSceneGpuMaxRigidPatchCount = source.internToken("physxScene:gpuMaxRigidPatchCount");
    physxSceneGpuHeapCapacity = source.internToken("physxScene:gpuHeapCapacity");
    physxSceneGpuFoundLostPairsCapacity = source.internToken("physxScene:gpuFoundLostPairsCapacity");
    physxSceneGpuFoundLostAggregatePairsCapacity = source.internToken("physxScene:gpuFoundLostAggregatePairsCapacity");
    physxSceneGpuTotalAggregatePairsCapacity = source.internToken("physxScene:gpuTotalAggregatePairsCapacity");
    physxSceneGpuMaxDeformableVolumeContacts = source.internToken("physxScene:gpuMaxDeformableVolumeContacts");
    physxSceneGpuMaxDeformableSurfaceContacts = source.internToken("physxScene:gpuMaxDeformableSurfaceContacts");
    physxSceneGpuMaxParticleContacts = source.internToken("physxScene:gpuMaxParticleContacts");
    physxSceneGpuCollisionStackSize = source.internToken("physxScene:gpuCollisionStackSize");
    physxSceneGpuMaxNumPartitions = source.internToken("physxScene:gpuMaxNumPartitions");
    physxSceneEnvIdInBoundsBitCount = source.internToken("physxScene:envIdInBoundsBitCount");

    sceneUpdateSynchronous = source.internToken("Synchronous");
    sceneUpdateAsynchronous = source.internToken("Asynchronous");
    sceneUpdateDisabled = source.internToken("Disabled");
    collisionSystemPCM = source.internToken("PCM");
    collisionSystemSAT = source.internToken("SAT");
    solverTypeTGS = source.internToken("TGS");
    solverTypePGS = source.internToken("PGS");
    broadphaseTypeMBP = source.internToken("MBP");
    broadphaseTypeSAP = source.internToken("SAP");
    broadphaseTypeGPU = source.internToken("GPU");

    physxSceneQuasistaticAPI = source.internToken("PhysxSceneQuasistaticAPI");
    physxSceneQuasistaticEnableQuasistatic = source.internToken("physxSceneQuasistatic:enableQuasistatic");

    newtonTimeStepsPerSecond = source.internToken("newton:timeStepsPerSecond");
    newtonGravityEnabled = source.internToken("newton:gravityEnabled");

    physicsRigidBodyAPI = source.internToken("PhysicsRigidBodyAPI");
    physicsCollisionAPI = source.internToken("PhysicsCollisionAPI");
    physicsMassAPI = source.internToken("PhysicsMassAPI");
    physicsMass = source.internToken("physics:mass");
    physicsDensity = source.internToken("physics:density");
    physicsCenterOfMass = source.internToken("physics:centerOfMass");
    physicsDiagonalInertia = source.internToken("physics:diagonalInertia");
    physicsPrincipalAxes = source.internToken("physics:principalAxes");
    physicsVelocity = source.internToken("physics:velocity");
    physicsAngularVelocity = source.internToken("physics:angularVelocity");
    physicsRigidBodyEnabled = source.internToken("physics:rigidBodyEnabled");
    physicsStartsAsleep = source.internToken("physics:startsAsleep");
    physicsKinematicEnabled = source.internToken("physics:kinematicEnabled");

    physicsJoint = source.internToken("PhysicsJoint");
    physicsBody0 = source.internToken("physics:body0");
    physicsBody1 = source.internToken("physics:body1");
    physicsLocalPos0 = source.internToken("physics:localPos0");
    physicsLocalPos1 = source.internToken("physics:localPos1");
    physicsLocalRot0 = source.internToken("physics:localRot0");
    physicsLocalRot1 = source.internToken("physics:localRot1");
    physicsJointEnabled = source.internToken("physics:jointEnabled");
    physicsBreakForce = source.internToken("physics:breakForce");
    physicsBreakTorque = source.internToken("physics:breakTorque");

    // PhysxJointAPI extensions
    physxJointAPI = source.internToken("PhysxJointAPI");
    physxJointJointFriction = source.internToken("physxJoint:jointFriction");
    physxJointArmature = source.internToken("physxJoint:armature");
    physxJointMaxJointVelocity = source.internToken("physxJoint:maxJointVelocity");

    // PhysxCollisionAPI extensions (subset routed through parse library)
    physxCollisionAPI = source.internToken("PhysxCollisionAPI");
    physxCollisionTorsionalPatchRadius = source.internToken("physxCollision:torsionalPatchRadius");
    physxCollisionMinTorsionalPatchRadius = source.internToken("physxCollision:minTorsionalPatchRadius");
    physxCollisionContactOffset = source.internToken("physxCollision:contactOffset");
    physxCollisionRestOffset = source.internToken("physxCollision:restOffset");

    physxConvexGeometryMargin = source.internToken("physxConvexGeometry:margin");

    physxCharacterControllerSlopeLimit = source.internToken("physxCharacterController:slopeLimit");

    usdPhysicsMeshCollisionAPI = source.internToken("PhysicsMeshCollisionAPI");
    physicsApproximation = source.internToken("physics:approximation");
    approximationNone               = source.internToken("none");
    approximationConvexHull         = source.internToken("convexHull");
    approximationBoundingSphere     = source.internToken("boundingSphere");
    approximationBoundingCube       = source.internToken("boundingCube");
    approximationMeshSimplification = source.internToken("meshSimplification");
    approximationConvexDecomposition= source.internToken("convexDecomposition");
    approximationSphereFill         = source.internToken("sphereFill");
    approximationSdf                = source.internToken("sdf");

    // PhysxTriggerAPI / PhysxTriggerStateAPI — single-apply schemas.
    physxTriggerAPI = source.internToken("PhysxTriggerAPI");
    physxTriggerStateAPI = source.internToken("PhysxTriggerStateAPI");

    // PhysxConvexHullCollisionAPI cooking-knob extensions.
    physxConvexHullCollisionAPI = source.internToken("PhysxConvexHullCollisionAPI");
    physxConvexHullCollisionHullVertexLimit = source.internToken("physxConvexHullCollision:hullVertexLimit");
    physxConvexHullCollisionMinThickness = source.internToken("physxConvexHullCollision:minThickness");
    newtonMaxHullVertices = source.internToken("newton:maxHullVertices");
    newtonContactMargin = source.internToken("newton:contactMargin");
    newtonContactGap = source.internToken("newton:contactGap");

    // PhysxConvexDecompositionCollisionAPI cooking-knob extensions.
    physxConvexDecompositionCollisionAPI = source.internToken("PhysxConvexDecompositionCollisionAPI");
    physxConvexDecompositionCollisionMinThickness = source.internToken("physxConvexDecompositionCollision:minThickness");
    physxConvexDecompositionCollisionMaxConvexHulls = source.internToken("physxConvexDecompositionCollision:maxConvexHulls");
    physxConvexDecompositionCollisionHullVertexLimit = source.internToken("physxConvexDecompositionCollision:hullVertexLimit");
    physxConvexDecompositionCollisionVoxelResolution = source.internToken("physxConvexDecompositionCollision:voxelResolution");
    physxConvexDecompositionCollisionErrorPercentage = source.internToken("physxConvexDecompositionCollision:errorPercentage");
    physxConvexDecompositionCollisionShrinkWrap = source.internToken("physxConvexDecompositionCollision:shrinkWrap");

    // PhysxSphereFillCollisionAPI cooking-knob extensions.
    physxSphereFillCollisionAPI = source.internToken("PhysxSphereFillCollisionAPI");
    physxSphereFillCollisionMaxSpheres = source.internToken("physxSphereFillCollision:maxSpheres");
    physxSphereFillCollisionSeedCount = source.internToken("physxSphereFillCollision:seedCount");
    physxSphereFillCollisionVoxelResolution = source.internToken("physxSphereFillCollision:voxelResolution");
    physxSphereFillCollisionFillMode = source.internToken("physxSphereFillCollision:fillMode");
    fillModeFlood = source.internToken("flood");
    fillModeRaycast = source.internToken("raycast");
    fillModeSurface = source.internToken("surface");

    // PhysxTriangleMeshCollisionAPI / PhysxTriangleMeshSimplificationCollisionAPI
    physxTriangleMeshCollisionAPI = source.internToken("PhysxTriangleMeshCollisionAPI");
    physxTriangleMeshCollisionWeldTolerance = source.internToken("physxTriangleMeshCollision:weldTolerance");
    physxTriangleMeshSimplificationCollisionAPI = source.internToken("PhysxTriangleMeshSimplificationCollisionAPI");
    physxTriangleMeshSimplificationCollisionMetric = source.internToken("physxTriangleMeshSimplificationCollision:metric");
    physxTriangleMeshSimplificationCollisionWeldTolerance = source.internToken("physxTriangleMeshSimplificationCollision:weldTolerance");

    // PhysxSDFMeshCollisionAPI cooking-knob extensions.
    physxSDFMeshCollisionAPI = source.internToken("PhysxSDFMeshCollisionAPI");
    physxSDFMeshCollisionSdfResolution = source.internToken("physxSDFMeshCollision:sdfResolution");
    physxSDFMeshCollisionSdfSubgridResolution = source.internToken("physxSDFMeshCollision:sdfSubgridResolution");
    physxSDFMeshCollisionSdfBitsPerSubgridPixel = source.internToken("physxSDFMeshCollision:sdfBitsPerSubgridPixel");
    physxSDFMeshCollisionSdfNarrowBandThickness = source.internToken("physxSDFMeshCollision:sdfNarrowBandThickness");
    physxSDFMeshCollisionSdfMargin = source.internToken("physxSDFMeshCollision:sdfMargin");
    physxSDFMeshCollisionSdfEnableRemeshing = source.internToken("physxSDFMeshCollision:sdfEnableRemeshing");
    physxSDFMeshCollisionSdfTriangleCountReductionFactor = source.internToken("physxSDFMeshCollision:sdfTriangleCountReductionFactor");
    bitsPerPixel8 = source.internToken("BitsPerPixel8");
    bitsPerPixel16 = source.internToken("BitsPerPixel16");
    bitsPerPixel32 = source.internToken("BitsPerPixel32");

    // PhysxArticulationAPI extensions
    physxArticulationAPI = source.internToken("PhysxArticulationAPI");
    physxArticulationEnabled = source.internToken("physxArticulation:articulationEnabled");
    physxArticulationSleepThreshold = source.internToken("physxArticulation:sleepThreshold");
    physxArticulationStabilizationThreshold = source.internToken("physxArticulation:stabilizationThreshold");
    physxArticulationSolverPositionIterationCount = source.internToken("physxArticulation:solverPositionIterationCount");
    physxArticulationSolverVelocityIterationCount = source.internToken("physxArticulation:solverVelocityIterationCount");
    physxArticulationEnabledSelfCollisions = source.internToken("physxArticulation:enabledSelfCollisions");
    newtonSelfCollisionEnabled = source.internToken("newton:selfCollisionEnabled");

    // PhysicsFilteredPairsAPI — applied API + the filteredPairs relationship.
    physicsFilteredPairsAPI = source.internToken("PhysicsFilteredPairsAPI");
    physicsFilteredPairs    = source.internToken("physics:filteredPairs");

    // UsdPhysicsCollisionGroup — filteredGroups rel + the "colliders"
    // collection name on the collision-group prim.
    physicsFilteredGroups   = source.internToken("physics:filteredGroups");
    collidersCollectionName = source.internToken("colliders");

    // OmniPhysicsDeformable attachments + element collision filters.
    omniphysicsAttachmentEnabled = source.internToken("omniphysics:attachmentEnabled");
    omniphysicsFilterEnabled     = source.internToken("omniphysics:filterEnabled");
    omniphysicsDamping           = source.internToken("omniphysics:damping");
    omniphysicsStiffness         = source.internToken("omniphysics:stiffness");
    omniphysicsSrc0              = source.internToken("omniphysics:src0");
    omniphysicsSrc1              = source.internToken("omniphysics:src1");

    // OmniPhysicsDeformable body schemas (BodyAPI + DeformableBodyAPI).
    omniphysicsBodyAPI                = source.internToken("OmniPhysicsBodyAPI");
    omniphysicsDeformableBodyAPI      = source.internToken("OmniPhysicsDeformableBodyAPI");
    omniphysicsDeformableBodyEnabled  = source.internToken("omniphysics:deformableBodyEnabled");
    omniphysicsMass                   = source.internToken("omniphysics:mass");
    omniphysicsKinematicEnabled       = source.internToken("omniphysics:kinematicEnabled");
    omniphysicsStartsAsleep           = source.internToken("omniphysics:startsAsleep");
    omniphysicsSimulationOwner        = source.internToken("omniphysics:simulationOwner");

    // PhysxRigidBodyAPI
    physxRigidBodyAPI = source.internToken("PhysxRigidBodyAPI");
    physxRigidBodyLinearDamping = source.internToken("physxRigidBody:linearDamping");
    physxRigidBodyAngularDamping = source.internToken("physxRigidBody:angularDamping");
    physxRigidBodyMaxLinearVelocity = source.internToken("physxRigidBody:maxLinearVelocity");
    physxRigidBodyMaxAngularVelocity = source.internToken("physxRigidBody:maxAngularVelocity");
    physxRigidBodySleepThreshold = source.internToken("physxRigidBody:sleepThreshold");
    physxRigidBodyStabilizationThreshold = source.internToken("physxRigidBody:stabilizationThreshold");
    physxRigidBodyMaxDepenetrationVelocity = source.internToken("physxRigidBody:maxDepenetrationVelocity");
    physxRigidBodyContactSlopCoefficient = source.internToken("physxRigidBody:contactSlopCoefficient");
    physxRigidBodyMaxContactImpulse = source.internToken("physxRigidBody:maxContactImpulse");
    physxRigidBodyCfmScale = source.internToken("physxRigidBody:cfmScale");
    physxRigidBodySolverPositionIterationCount = source.internToken("physxRigidBody:solverPositionIterationCount");
    physxRigidBodySolverVelocityIterationCount = source.internToken("physxRigidBody:solverVelocityIterationCount");
    physxRigidBodyEnableCCD = source.internToken("physxRigidBody:enableCCD");
    physxRigidBodyEnableSpeculativeCCD = source.internToken("physxRigidBody:enableSpeculativeCCD");
    physxRigidBodyDisableGravity = source.internToken("physxRigidBody:disableGravity");
    physxRigidBodyRetainAccelerations = source.internToken("physxRigidBody:retainAccelerations");
    physxRigidBodyEnableGyroscopicForces = source.internToken("physxRigidBody:enableGyroscopicForces");
    physxRigidBodySolveContact = source.internToken("physxRigidBody:solveContact");
    physxRigidBodyLockedPosAxis = source.internToken("physxRigidBody:lockedPosAxis");
    physxRigidBodyLockedRotAxis = source.internToken("physxRigidBody:lockedRotAxis");

    // PhysxSurfaceVelocityAPI
    physxSurfaceVelocityAPI = source.internToken("PhysxSurfaceVelocityAPI");
    physxSurfaceVelocityEnabled = source.internToken("physxSurfaceVelocity:surfaceVelocityEnabled");
    physxSurfaceVelocityLocalSpace = source.internToken("physxSurfaceVelocity:surfaceVelocityLocalSpace");
    physxSurfaceVelocity = source.internToken("physxSurfaceVelocity:surfaceVelocity");
    physxSurfaceAngularVelocity = source.internToken("physxSurfaceVelocity:surfaceAngularVelocity");

    // PhysxSplinesSurfaceVelocityAPI
    physxSplinesSurfaceVelocityAPI = source.internToken("PhysxSplinesSurfaceVelocityAPI");
    physxSplinesSurfaceVelocityEnabled = source.internToken("physxSplinesSurfaceVelocity:surfaceVelocityEnabled");
    physxSplinesSurfaceVelocityMagnitude = source.internToken("physxSplinesSurfaceVelocity:surfaceVelocityMagnitude");
    physxSplinesSurfaceVelocityCurve = source.internToken("physxSplinesSurfaceVelocity:surfaceVelocityCurve");
    basisCurvesType = source.internToken("BasisCurves");

    // Prim metadata key (custom-data-dict key) used to force
    // local-space velocity output on a rigid body.
    metadataLocalSpaceVelocities = source.internToken("physics:localSpaceVelocities");
}

} // namespace omni::physics::parse
