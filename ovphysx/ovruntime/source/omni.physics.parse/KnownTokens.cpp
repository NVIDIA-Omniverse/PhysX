// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CORE-002
 * @covers AC-2
 *
 * @implements REQ-SPLINE-CURVE-001
 * @covers AC-6
 *
 * @implements REQ-PARSE-JOINT-002
 * @covers AC-2
 *
 * @implements REQ-PARSE-JOINT-005
 * @covers AC-5
 *
 * @implements REQ-SIM-SCENEQUERY-001
 * @covers AC-3
 */

#include <omni/physics/parse/KnownTokens.h>
#include <omni/physics/parse/IPhysicsSource.h>

#include <atomic>

namespace omni::physics::parse
{

namespace
{
std::atomic<uint64_t> gInternCallCount{ 0 };
}

uint64_t internCallCount()
{
    return gInternCallCount.load(std::memory_order_relaxed);
}

void KnownTokens::intern(const IPhysicsSource& source)
{
    gInternCallCount.fetch_add(1, std::memory_order_relaxed);

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
    newtonVelocityLimit = source.internToken("newton:velocityLimit");

    // PhysxCollisionAPI extensions (subset routed through parse library)
    physxCollisionAPI = source.internToken("PhysxCollisionAPI");
    physxCollisionTorsionalPatchRadius = source.internToken("physxCollision:torsionalPatchRadius");
    physxCollisionMinTorsionalPatchRadius = source.internToken("physxCollision:minTorsionalPatchRadius");
    physxCollisionContactOffset = source.internToken("physxCollision:contactOffset");
    physxCollisionRestOffset = source.internToken("physxCollision:restOffset");

    physxConvexGeometryMargin = source.internToken("physxConvexGeometry:margin");

    physxCharacterControllerAPI = source.internToken("PhysxCharacterControllerAPI");
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

    // PhysxMeshMergeCollisionAPI: implicit mesh merging for collision geometry.
    physxMeshMergeCollisionAPI = source.internToken("PhysxMeshMergeCollisionAPI");

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

    // PhysxSchemaTokens attributes (schema-generated PhysX token table)
    acceleration = source.internToken("acceleration");
    brakes0 = source.internToken("brakes0");
    brakes1 = source.internToken("brakes1");
    constrained = source.internToken("constrained");
    contactOffset = source.internToken("contactOffset");
    defaultFrictionValue = source.internToken("defaultFrictionValue");
    drive = source.internToken("drive");
    easy = source.internToken("easy");
    enableCCD = source.internToken("enableCCD");
    fluidRestOffset = source.internToken("fluidRestOffset");
    frictionValues = source.internToken("frictionValues");
    groundMaterials = source.internToken("groundMaterials");
    maxDepenetrationVelocity = source.internToken("maxDepenetrationVelocity");
    maxNeighborhood = source.internToken("maxNeighborhood");
    maxVelocity = source.internToken("maxVelocity");
    neighborhoodScale = source.internToken("neighborhoodScale");
    particleContactOffset = source.internToken("particleContactOffset");
    particleSystemEnabled = source.internToken("particleSystemEnabled");
    physicsBody0Indices = source.internToken("physics:body0Indices");
    physicsBody0s = source.internToken("physics:body0s");
    physicsBody1Indices = source.internToken("physics:body1Indices");
    physicsBody1s = source.internToken("physics:body1s");
    physicsGearRatio = source.internToken("physics:gearRatio");
    physicsHinge = source.internToken("physics:hinge");
    physicsHinge0 = source.internToken("physics:hinge0");
    physicsHinge1 = source.internToken("physics:hinge1");
    physicsLocalPos0s = source.internToken("physics:localPos0s");
    physicsLocalPos1s = source.internToken("physics:localPos1s");
    physicsLocalRot0s = source.internToken("physics:localRot0s");
    physicsLocalRot1s = source.internToken("physics:localRot1s");
    physicsPrismatic = source.internToken("physics:prismatic");
    physicsProtoIndices = source.internToken("physics:protoIndices");
    physicsPrototypes = source.internToken("physics:prototypes");
    physicsRatio = source.internToken("physics:ratio");
    PhysxAutoDeformableAttachmentAPI = source.internToken("PhysxAutoDeformableAttachmentAPI");
    physxAutoDeformableAttachmentAttachable0 = source.internToken("physxAutoDeformableAttachment:attachable0");
    physxAutoDeformableAttachmentAttachable1 = source.internToken("physxAutoDeformableAttachment:attachable1");
    physxAutoDeformableAttachmentCollisionFilteringOffset = source.internToken("physxAutoDeformableAttachment:collisionFilteringOffset");
    physxAutoDeformableAttachmentDeformableVertexOverlapOffset = source.internToken("physxAutoDeformableAttachment:deformableVertexOverlapOffset");
    physxAutoDeformableAttachmentEnableCollisionFiltering = source.internToken("physxAutoDeformableAttachment:enableCollisionFiltering");
    physxAutoDeformableAttachmentEnableDeformableFilteringPairs = source.internToken("physxAutoDeformableAttachment:enableDeformableFilteringPairs");
    physxAutoDeformableAttachmentEnableDeformableVertexAttachments = source.internToken("physxAutoDeformableAttachment:enableDeformableVertexAttachments");
    physxAutoDeformableAttachmentEnableRigidSurfaceAttachments = source.internToken("physxAutoDeformableAttachment:enableRigidSurfaceAttachments");
    physxAutoDeformableAttachmentMaskShapes = source.internToken("physxAutoDeformableAttachment:maskShapes");
    physxAutoDeformableAttachmentRigidSurfaceSamplingDistance = source.internToken("physxAutoDeformableAttachment:rigidSurfaceSamplingDistance");
    PhysxAutoDeformableBodyAPI = source.internToken("PhysxAutoDeformableBodyAPI");
    PhysxAutoDeformableHexahedralMeshAPI = source.internToken("PhysxAutoDeformableHexahedralMeshAPI");
    PhysxAutoDeformableMeshSimplificationAPI = source.internToken("PhysxAutoDeformableMeshSimplificationAPI");
    physxCharacterControllerClimbingMode = source.internToken("physxCharacterController:climbingMode");
    physxCharacterControllerContactOffset = source.internToken("physxCharacterController:contactOffset");
    physxCharacterControllerInvisibleWallHeight = source.internToken("physxCharacterController:invisibleWallHeight");
    physxCharacterControllerMaxJumpHeight = source.internToken("physxCharacterController:maxJumpHeight");
    physxCharacterControllerNonWalkableMode = source.internToken("physxCharacterController:nonWalkableMode");
    physxCharacterControllerScaleCoeff = source.internToken("physxCharacterController:scaleCoeff");
    physxCharacterControllerStepOffset = source.internToken("physxCharacterController:stepOffset");
    physxCharacterControllerUpAxis = source.internToken("physxCharacterController:upAxis");
    physxCharacterControllerVolumeGrowth = source.internToken("physxCharacterController:volumeGrowth");
    physxContactReportAPI = source.internToken("PhysxContactReportAPI");
    physxContactReportReportPairs = source.internToken("physxContactReport:reportPairs");
    physxContactReportThreshold = source.internToken("physxContactReport:threshold");
    physxDeformableBodyAutoDeformableBodyEnabled = source.internToken("physxDeformableBody:autoDeformableBodyEnabled");
    physxDeformableBodyAutoDeformableMeshSimplificationEnabled = source.internToken("physxDeformableBody:autoDeformableMeshSimplificationEnabled");
    physxDeformableBodyCollisionIterationMultiplier = source.internToken("physxDeformableBody:collisionIterationMultiplier");
    physxDeformableBodyCollisionPairUpdateFrequency = source.internToken("physxDeformableBody:collisionPairUpdateFrequency");
    physxDeformableBodyDisableGravity = source.internToken("physxDeformableBody:disableGravity");
    physxDeformableBodyEnableSpeculativeCCD = source.internToken("physxDeformableBody:enableSpeculativeCCD");
    physxDeformableBodyForceConforming = source.internToken("physxDeformableBody:forceConforming");
    physxDeformableBodyLinearDamping = source.internToken("physxDeformableBody:linearDamping");
    physxDeformableBodyMaxDepenetrationVelocity = source.internToken("physxDeformableBody:maxDepenetrationVelocity");
    physxDeformableBodyMaxLinearVelocity = source.internToken("physxDeformableBody:maxLinearVelocity");
    physxDeformableBodyRemeshingEnabled = source.internToken("physxDeformableBody:remeshingEnabled");
    physxDeformableBodyRemeshingResolution = source.internToken("physxDeformableBody:remeshingResolution");
    physxDeformableBodyResolution = source.internToken("physxDeformableBody:resolution");
    physxDeformableBodySelfCollision = source.internToken("physxDeformableBody:selfCollision");
    physxDeformableBodySelfCollisionFilterDistance = source.internToken("physxDeformableBody:selfCollisionFilterDistance");
    physxDeformableBodySettlingDamping = source.internToken("physxDeformableBody:settlingDamping");
    physxDeformableBodySettlingThreshold = source.internToken("physxDeformableBody:settlingThreshold");
    physxDeformableBodySleepThreshold = source.internToken("physxDeformableBody:sleepThreshold");
    physxDeformableBodySolverPositionIterationCount = source.internToken("physxDeformableBody:solverPositionIterationCount");
    physxDeformableBodyTargetTriangleCount = source.internToken("physxDeformableBody:targetTriangleCount");
    physxDeformableMaterialBendDamping = source.internToken("physxDeformableMaterial:bendDamping");
    physxDeformableMaterialElasticityDamping = source.internToken("physxDeformableMaterial:elasticityDamping");
    physxDiffuseParticlesAPI = source.internToken("PhysxDiffuseParticlesAPI");
    physxDiffuseParticlesAirDrag = source.internToken("physxDiffuseParticles:airDrag");
    physxDiffuseParticlesBubbleDrag = source.internToken("physxDiffuseParticles:bubbleDrag");
    physxDiffuseParticlesBuoyancy = source.internToken("physxDiffuseParticles:buoyancy");
    physxDiffuseParticlesCollisionDecay = source.internToken("physxDiffuseParticles:collisionDecay");
    physxDiffuseParticlesDiffuseParticlesEnabled = source.internToken("physxDiffuseParticles:diffuseParticlesEnabled");
    physxDiffuseParticlesDivergenceWeight = source.internToken("physxDiffuseParticles:divergenceWeight");
    physxDiffuseParticlesKineticEnergyWeight = source.internToken("physxDiffuseParticles:kineticEnergyWeight");
    physxDiffuseParticlesLifetime = source.internToken("physxDiffuseParticles:lifetime");
    physxDiffuseParticlesMaxDiffuseParticleMultiplier = source.internToken("physxDiffuseParticles:maxDiffuseParticleMultiplier");
    physxDiffuseParticlesPressureWeight = source.internToken("physxDiffuseParticles:pressureWeight");
    physxDiffuseParticlesThreshold = source.internToken("physxDiffuseParticles:threshold");
    PhysxDrivePerformanceEnvelopeAPI = source.internToken("PhysxDrivePerformanceEnvelopeAPI");
    physxForceAPI = source.internToken("PhysxForceAPI");
    physxForceForce = source.internToken("physxForce:force");
    physxForceForceEnabled = source.internToken("physxForce:forceEnabled");
    physxForceMode = source.internToken("physxForce:mode");
    physxForceTorque = source.internToken("physxForce:torque");
    physxForceWorldFrameEnabled = source.internToken("physxForce:worldFrameEnabled");
    PhysxJointAxisAPI = source.internToken("PhysxJointAxisAPI");
    physxMimicJoint_MultipleApplyTemplate_DampingRatio = source.internToken("physxMimicJoint:__INSTANCE_NAME__:dampingRatio");
    physxMimicJoint_MultipleApplyTemplate_Gearing = source.internToken("physxMimicJoint:__INSTANCE_NAME__:gearing");
    physxMimicJoint_MultipleApplyTemplate_NaturalFrequency = source.internToken("physxMimicJoint:__INSTANCE_NAME__:naturalFrequency");
    physxMimicJoint_MultipleApplyTemplate_Offset = source.internToken("physxMimicJoint:__INSTANCE_NAME__:offset");
    physxMimicJoint_MultipleApplyTemplate_ReferenceJoint = source.internToken("physxMimicJoint:__INSTANCE_NAME__:referenceJoint");
    physxMimicJoint_MultipleApplyTemplate_ReferenceJointAxis = source.internToken("physxMimicJoint:__INSTANCE_NAME__:referenceJointAxis");
    physxParticleAnisotropyAPI = source.internToken("PhysxParticleAnisotropyAPI");
    physxParticleAnisotropyParticleAnisotropyEnabled = source.internToken("physxParticleAnisotropy:particleAnisotropyEnabled");
    physxParticleAPI = source.internToken("PhysxParticleAPI");
    physxParticleFluid = source.internToken("physxParticle:fluid");
    physxParticleIsosurfaceAPI = source.internToken("PhysxParticleIsosurfaceAPI");
    physxParticleIsosurfaceGridFilteringPasses = source.internToken("physxParticleIsosurface:gridFilteringPasses");
    physxParticleIsosurfaceGridSmoothingRadius = source.internToken("physxParticleIsosurface:gridSmoothingRadius");
    physxParticleIsosurfaceIsosurfaceEnabled = source.internToken("physxParticleIsosurface:isosurfaceEnabled");
    physxParticleIsosurfaceNumMeshNormalSmoothingPasses = source.internToken("physxParticleIsosurface:numMeshNormalSmoothingPasses");
    physxParticleIsosurfaceNumMeshSmoothingPasses = source.internToken("physxParticleIsosurface:numMeshSmoothingPasses");
    physxParticleIsosurfaceSurfaceDistance = source.internToken("physxParticleIsosurface:surfaceDistance");
    physxParticleParticleEnabled = source.internToken("physxParticle:particleEnabled");
    physxParticleParticleGroup = source.internToken("physxParticle:particleGroup");
    physxParticleSamplingAPI = source.internToken("PhysxParticleSamplingAPI");
    physxParticleSelfCollision = source.internToken("physxParticle:selfCollision");
    physxParticleSetAPI = source.internToken("PhysxParticleSetAPI");
    physxParticleSimulationPoints = source.internToken("physxParticle:simulationPoints");
    physxParticleSmoothingAPI = source.internToken("PhysxParticleSmoothingAPI");
    physxParticleSmoothingParticleSmoothingEnabled = source.internToken("physxParticleSmoothing:particleSmoothingEnabled");
    physxPBDMaterialAdhesion = source.internToken("physxPBDMaterial:adhesion");
    physxPBDMaterialAdhesionOffsetScale = source.internToken("physxPBDMaterial:adhesionOffsetScale");
    physxPBDMaterialCflCoefficient = source.internToken("physxPBDMaterial:cflCoefficient");
    physxPBDMaterialCohesion = source.internToken("physxPBDMaterial:cohesion");
    physxPBDMaterialDamping = source.internToken("physxPBDMaterial:damping");
    physxPBDMaterialDensity = source.internToken("physxPBDMaterial:density");
    physxPBDMaterialFriction = source.internToken("physxPBDMaterial:friction");
    physxPBDMaterialGravityScale = source.internToken("physxPBDMaterial:gravityScale");
    physxPBDMaterialParticleAdhesionScale = source.internToken("physxPBDMaterial:particleAdhesionScale");
    physxPBDMaterialParticleFrictionScale = source.internToken("physxPBDMaterial:particleFrictionScale");
    physxPBDMaterialSurfaceTension = source.internToken("physxPBDMaterial:surfaceTension");
    physxPBDMaterialViscosity = source.internToken("physxPBDMaterial:viscosity");
    physxPBDMaterialVorticityConfinement = source.internToken("physxPBDMaterial:vorticityConfinement");
    physxPhysicsDistanceJointSpringDamping = source.internToken("physxPhysicsDistanceJoint:springDamping");
    physxPhysicsDistanceJointSpringEnabled = source.internToken("physxPhysicsDistanceJoint:springEnabled");
    physxPhysicsDistanceJointSpringStiffness = source.internToken("physxPhysicsDistanceJoint:springStiffness");
    schemaQuasistaticEnableQuasistatic = source.internToken("physxScene:quasistaticEnableQuasistatic");
    physxTendonAttachmentAPI = source.internToken("PhysxTendonAttachmentAPI");
    physxTendonAttachmentLeafAPI = source.internToken("PhysxTendonAttachmentLeafAPI");
    physxTendonAttachmentRootAPI = source.internToken("PhysxTendonAttachmentRootAPI");
    physxTendonAxisAPI = source.internToken("PhysxTendonAxisAPI");
    physxTendonAxisRootAPI = source.internToken("PhysxTendonAxisRootAPI");
    physxVehicleAPI = source.internToken("PhysxVehicleAPI");
    physxVehicleAckermannSteeringAPI = source.internToken("PhysxVehicleAckermannSteeringAPI");
    physxVehicleAckermannSteeringMaxSteerAngle = source.internToken("physxVehicleAckermannSteering:maxSteerAngle");
    physxVehicleAckermannSteeringStrength = source.internToken("physxVehicleAckermannSteering:strength");
    physxVehicleAckermannSteeringTrackWidth = source.internToken("physxVehicleAckermannSteering:trackWidth");
    physxVehicleAckermannSteeringWheel0 = source.internToken("physxVehicleAckermannSteering:wheel0");
    physxVehicleAckermannSteeringWheel1 = source.internToken("physxVehicleAckermannSteering:wheel1");
    physxVehicleAckermannSteeringWheelBase = source.internToken("physxVehicleAckermannSteering:wheelBase");
    physxVehicleBrakesAPI = source.internToken("PhysxVehicleBrakesAPI");
    physxVehicleBrakes_MultipleApplyTemplate_MaxBrakeTorque = source.internToken("physxVehicleBrakes:__INSTANCE_NAME__:maxBrakeTorque");
    physxVehicleBrakes_MultipleApplyTemplate_TorqueMultipliers = source.internToken("physxVehicleBrakes:__INSTANCE_NAME__:torqueMultipliers");
    physxVehicleBrakes_MultipleApplyTemplate_Wheels = source.internToken("physxVehicleBrakes:__INSTANCE_NAME__:wheels");
    physxVehicleContextAPI = source.internToken("PhysxVehicleContextAPI");
    physxVehicleContextLongitudinalAxis = source.internToken("physxVehicleContext:longitudinalAxis");
    physxVehicleContextUpdateMode = source.internToken("physxVehicleContext:updateMode");
    physxVehicleContextVerticalAxis = source.internToken("physxVehicleContext:verticalAxis");
    physxVehicleControllerAPI = source.internToken("PhysxVehicleControllerAPI");
    physxVehicleControllerAccelerator = source.internToken("physxVehicleController:accelerator");
    physxVehicleControllerBrake = source.internToken("physxVehicleController:brake");
    physxVehicleControllerBrake0 = source.internToken("physxVehicleController:brake0");
    physxVehicleControllerBrake1 = source.internToken("physxVehicleController:brake1");
    physxVehicleControllerHandbrake = source.internToken("physxVehicleController:handbrake");
    physxVehicleControllerSteer = source.internToken("physxVehicleController:steer");
    physxVehicleControllerSteerLeft = source.internToken("physxVehicleController:steerLeft");
    physxVehicleControllerSteerRight = source.internToken("physxVehicleController:steerRight");
    physxVehicleControllerTargetGear = source.internToken("physxVehicleController:targetGear");
    physxVehicleDrive = source.internToken("physxVehicle:drive");
    physxVehicleDriveBasicAPI = source.internToken("PhysxVehicleDriveBasicAPI");
    physxVehicleDriveBasicPeakTorque = source.internToken("physxVehicleDriveBasic:peakTorque");
    physxVehicleDriveStandardAPI = source.internToken("PhysxVehicleDriveStandardAPI");
    physxVehicleEngineDampingRateFullThrottle = source.internToken("physxVehicleEngine:dampingRateFullThrottle");
    physxVehicleEngineDampingRateZeroThrottleClutchDisengaged = source.internToken("physxVehicleEngine:dampingRateZeroThrottleClutchDisengaged");
    physxVehicleEngineDampingRateZeroThrottleClutchEngaged = source.internToken("physxVehicleEngine:dampingRateZeroThrottleClutchEngaged");
    physxVehicleEngineIdleRotationSpeed = source.internToken("physxVehicleEngine:idleRotationSpeed");
    physxVehicleEngineMaxRotationSpeed = source.internToken("physxVehicleEngine:maxRotationSpeed");
    physxVehicleEngineMoi = source.internToken("physxVehicleEngine:moi");
    physxVehicleEnginePeakTorque = source.internToken("physxVehicleEngine:peakTorque");
    physxVehicleEngineTorqueCurve = source.internToken("physxVehicleEngine:torqueCurve");
    physxVehicleLateralStickyTireDamping = source.internToken("physxVehicleLateralStickyTire:damping");
    physxVehicleLateralStickyTireThresholdSpeed = source.internToken("physxVehicleLateralStickyTire:thresholdSpeed");
    physxVehicleLateralStickyTireThresholdTime = source.internToken("physxVehicleLateralStickyTire:thresholdTime");
    physxVehicleLimitSuspensionExpansionVelocity = source.internToken("physxVehicle:limitSuspensionExpansionVelocity");
    physxVehicleLongitudinalStickyTireDamping = source.internToken("physxVehicleLongitudinalStickyTire:damping");
    physxVehicleLongitudinalStickyTireThresholdSpeed = source.internToken("physxVehicleLongitudinalStickyTire:thresholdSpeed");
    physxVehicleLongitudinalStickyTireThresholdTime = source.internToken("physxVehicleLongitudinalStickyTire:thresholdTime");
    physxVehicleMinActiveLongitudinalSlipDenominator = source.internToken("physxVehicle:minActiveLongitudinalSlipDenominator");
    physxVehicleMinLateralSlipDenominator = source.internToken("physxVehicle:minLateralSlipDenominator");
    physxVehicleMinPassiveLongitudinalSlipDenominator = source.internToken("physxVehicle:minPassiveLongitudinalSlipDenominator");
    physxVehicleMultiWheelDifferentialAPI = source.internToken("PhysxVehicleMultiWheelDifferentialAPI");
    physxVehicleMultiWheelDifferentialAverageWheelSpeedRatios = source.internToken("physxVehicleMultiWheelDifferential:averageWheelSpeedRatios");
    physxVehicleMultiWheelDifferentialTorqueRatios = source.internToken("physxVehicleMultiWheelDifferential:torqueRatios");
    physxVehicleMultiWheelDifferentialWheels = source.internToken("physxVehicleMultiWheelDifferential:wheels");
    physxVehicleNCR_MultipleApplyTemplate_CommandValues = source.internToken("physxVehicleNCR:__INSTANCE_NAME__:commandValues");
    physxVehicleNCR_MultipleApplyTemplate_SpeedResponses = source.internToken("physxVehicleNCR:__INSTANCE_NAME__:speedResponses");
    physxVehicleNCR_MultipleApplyTemplate_SpeedResponsesPerCommandValue = source.internToken("physxVehicleNCR:__INSTANCE_NAME__:speedResponsesPerCommandValue");
    physxVehicleSteeringAPI = source.internToken("PhysxVehicleSteeringAPI");
    physxVehicleSteeringAngleMultipliers = source.internToken("physxVehicleSteering:angleMultipliers");
    physxVehicleSteeringMaxSteerAngle = source.internToken("physxVehicleSteering:maxSteerAngle");
    physxVehicleSteeringWheels = source.internToken("physxVehicleSteering:wheels");
    physxVehicleSuspensionCamberAtMaxCompression = source.internToken("physxVehicleSuspension:camberAtMaxCompression");
    physxVehicleSuspensionCamberAtMaxDroop = source.internToken("physxVehicleSuspension:camberAtMaxDroop");
    physxVehicleSuspensionCamberAtRest = source.internToken("physxVehicleSuspension:camberAtRest");
    physxVehicleSuspensionComplianceSuspensionForceAppPoint = source.internToken("physxVehicleSuspensionCompliance:suspensionForceAppPoint");
    physxVehicleSuspensionComplianceTireForceAppPoint = source.internToken("physxVehicleSuspensionCompliance:tireForceAppPoint");
    physxVehicleSuspensionComplianceWheelCamberAngle = source.internToken("physxVehicleSuspensionCompliance:wheelCamberAngle");
    physxVehicleSuspensionComplianceWheelToeAngle = source.internToken("physxVehicleSuspensionCompliance:wheelToeAngle");
    physxVehicleSuspensionMaxCompression = source.internToken("physxVehicleSuspension:maxCompression");
    physxVehicleSuspensionMaxDroop = source.internToken("physxVehicleSuspension:maxDroop");
    physxVehicleSuspensionSpringDamperRate = source.internToken("physxVehicleSuspension:springDamperRate");
    physxVehicleSuspensionSpringStrength = source.internToken("physxVehicleSuspension:springStrength");
    physxVehicleSuspensionSprungMass = source.internToken("physxVehicleSuspension:sprungMass");
    physxVehicleSuspensionTravelDistance = source.internToken("physxVehicleSuspension:travelDistance");
    physxVehicleTankControllerAPI = source.internToken("PhysxVehicleTankControllerAPI");
    physxVehicleTankControllerThrust0 = source.internToken("physxVehicleTankController:thrust0");
    physxVehicleTankControllerThrust1 = source.internToken("physxVehicleTankController:thrust1");
    physxVehicleTankDifferentialAPI = source.internToken("PhysxVehicleTankDifferentialAPI");
    physxVehicleTankDifferentialNumberOfWheelsPerTrack = source.internToken("physxVehicleTankDifferential:numberOfWheelsPerTrack");
    physxVehicleTankDifferentialThrustIndexPerTrack = source.internToken("physxVehicleTankDifferential:thrustIndexPerTrack");
    physxVehicleTankDifferentialTrackToWheelIndices = source.internToken("physxVehicleTankDifferential:trackToWheelIndices");
    physxVehicleTankDifferentialWheelIndicesInTrackOrder = source.internToken("physxVehicleTankDifferential:wheelIndicesInTrackOrder");
    physxVehicleTireCamberStiffness = source.internToken("physxVehicleTire:camberStiffness");
    physxVehicleTireCamberStiffnessPerUnitGravity = source.internToken("physxVehicleTire:camberStiffnessPerUnitGravity");
    physxVehicleTireFrictionTable = source.internToken("physxVehicleTire:frictionTable");
    physxVehicleTireFrictionVsSlipGraph = source.internToken("physxVehicleTire:frictionVsSlipGraph");
    physxVehicleTireLateralStiffnessGraph = source.internToken("physxVehicleTire:lateralStiffnessGraph");
    physxVehicleTireLatStiffX = source.internToken("physxVehicleTire:latStiffX");
    physxVehicleTireLatStiffY = source.internToken("physxVehicleTire:latStiffY");
    physxVehicleTireLongitudinalStiffness = source.internToken("physxVehicleTire:longitudinalStiffness");
    physxVehicleTireLongitudinalStiffnessPerUnitGravity = source.internToken("physxVehicleTire:longitudinalStiffnessPerUnitGravity");
    physxVehicleTireRestLoad = source.internToken("physxVehicleTire:restLoad");
    physxVehicleVehicleEnabled = source.internToken("physxVehicle:vehicleEnabled");
    physxVehicleWheelAttachmentCollisionGroup = source.internToken("physxVehicleWheelAttachment:collisionGroup");
    physxVehicleWheelAttachmentDriven = source.internToken("physxVehicleWheelAttachment:driven");
    physxVehicleWheelAttachmentIndex = source.internToken("physxVehicleWheelAttachment:index");
    physxVehicleWheelAttachmentSuspension = source.internToken("physxVehicleWheelAttachment:suspension");
    physxVehicleWheelAttachmentSuspensionForceAppPointOffset = source.internToken("physxVehicleWheelAttachment:suspensionForceAppPointOffset");
    physxVehicleWheelAttachmentSuspensionFrameOrientation = source.internToken("physxVehicleWheelAttachment:suspensionFrameOrientation");
    physxVehicleWheelAttachmentSuspensionFramePosition = source.internToken("physxVehicleWheelAttachment:suspensionFramePosition");
    physxVehicleWheelAttachmentSuspensionTravelDirection = source.internToken("physxVehicleWheelAttachment:suspensionTravelDirection");
    physxVehicleWheelAttachmentTire = source.internToken("physxVehicleWheelAttachment:tire");
    physxVehicleWheelAttachmentTireForceAppPointOffset = source.internToken("physxVehicleWheelAttachment:tireForceAppPointOffset");
    physxVehicleWheelAttachmentWheel = source.internToken("physxVehicleWheelAttachment:wheel");
    physxVehicleWheelAttachmentWheelCenterOfMassOffset = source.internToken("physxVehicleWheelAttachment:wheelCenterOfMassOffset");
    physxVehicleWheelAttachmentWheelFrameOrientation = source.internToken("physxVehicleWheelAttachment:wheelFrameOrientation");
    physxVehicleWheelAttachmentWheelFramePosition = source.internToken("physxVehicleWheelAttachment:wheelFramePosition");
    physxVehicleWheelControllerAPI = source.internToken("PhysxVehicleWheelControllerAPI");
    physxVehicleWheelControllerBrakeTorque = source.internToken("physxVehicleWheelController:brakeTorque");
    physxVehicleWheelControllerDriveTorque = source.internToken("physxVehicleWheelController:driveTorque");
    physxVehicleWheelControllerSteerAngle = source.internToken("physxVehicleWheelController:steerAngle");
    physxVehicleWheelDampingRate = source.internToken("physxVehicleWheel:dampingRate");
    physxVehicleWheelMass = source.internToken("physxVehicleWheel:mass");
    physxVehicleWheelMaxBrakeTorque = source.internToken("physxVehicleWheel:maxBrakeTorque");
    physxVehicleWheelMaxHandBrakeTorque = source.internToken("physxVehicleWheel:maxHandBrakeTorque");
    physxVehicleWheelMaxSteerAngle = source.internToken("physxVehicleWheel:maxSteerAngle");
    physxVehicleWheelMoi = source.internToken("physxVehicleWheel:moi");
    physxVehicleWheelRadius = source.internToken("physxVehicleWheel:radius");
    physxVehicleWheelToeAngle = source.internToken("physxVehicleWheel:toeAngle");
    physxVehicleWheelWidth = source.internToken("physxVehicleWheel:width");
    preventClimbing = source.internToken("preventClimbing");
    preventClimbingForceSliding = source.internToken("preventClimbingForceSliding");
    quasistaticactors = source.internToken("physxScene:quasistaticActors");
    restOffset = source.internToken("restOffset");
    rotX = source.internToken("rotX");
    rotY = source.internToken("rotY");
    rotZ = source.internToken("rotZ");
    solidRestOffset = source.internToken("solidRestOffset");
    solverPositionIterationCount = source.internToken("solverPositionIterationCount");
    steer = source.internToken("steer");

    // UsdGeomTokens attributes (schema-generated UsdGeom token table)
    angularVelocities = source.internToken("angularVelocities");
    axis = source.internToken("axis");
    basis = source.internToken("basis");
    bezier = source.internToken("bezier");
    catmullRom = source.internToken("catmullRom");
    curveVertexCounts = source.internToken("curveVertexCounts");
    extent = source.internToken("extent");
    face = source.internToken("face");
    faceVertexCounts = source.internToken("faceVertexCounts");
    faceVertexIndices = source.internToken("faceVertexIndices");
    height = source.internToken("height");
    holeIndices = source.internToken("holeIndices");
    ids = source.internToken("ids");
    inactiveIds = source.internToken("inactiveIds");
    indices = source.internToken("indices");
    invisible = source.internToken("invisible");
    nonperiodic = source.internToken("nonperiodic");
    orientation = source.internToken("orientation");
    orientations = source.internToken("orientations");
    periodic = source.internToken("periodic");
    pinned = source.internToken("pinned");
    points = source.internToken("points");
    positions = source.internToken("positions");
    protoIndices = source.internToken("protoIndices");
    prototypes = source.internToken("prototypes");
    proxy = source.internToken("proxy");
    radius = source.internToken("radius");
    scales = source.internToken("scales");
    size = source.internToken("size");
    surfaceFaceVertexIndices = source.internToken("surfaceFaceVertexIndices");
    tetVertexIndices = source.internToken("tetVertexIndices");
    type = source.internToken("type");
    velocities = source.internToken("velocities");
    vertex = source.internToken("vertex");
    visibility = source.internToken("visibility");
    wrap = source.internToken("wrap");
    xformOpOrder = source.internToken("xformOpOrder");

    // Matches PhysxTokens.h's gWorldMatrixTokenString literal (omni.physx is
    // downstream of this library, so the string is duplicated here rather than
    // depending on it).
    omniXform = source.internToken("omni:xform");
    omniResetXformStack = source.internToken("omni:resetXformStack");
    omniFabricLocalMatrix = source.internToken("omni:fabric:localMatrix");
    omniFabricWorldMatrix = source.internToken("omni:fabric:worldMatrix");

    // OmniUsdPhysicsDeformableSchemaTokens attributes (schema-generated OmniPhysics deformable token table)
    deformablePose_MultipleApplyTemplate_OmniphysicsPoints = source.internToken("deformablePose:__INSTANCE_NAME__:omniphysics:points");
    deformablePose_MultipleApplyTemplate_OmniphysicsPurposes = source.internToken("deformablePose:__INSTANCE_NAME__:omniphysics:purposes");
    flatDefault = source.internToken("flatDefault");
    OmniPhysicsDeformablePoseAPI = source.internToken("OmniPhysicsDeformablePoseAPI");
    omniphysicsDensity = source.internToken("omniphysics:density");
    omniphysicsDynamicFriction = source.internToken("omniphysics:dynamicFriction");
    OmniPhysicsElementCollisionFilter = source.internToken("OmniPhysicsElementCollisionFilter");
    omniphysicsGroupElemCounts0 = source.internToken("omniphysics:groupElemCounts0");
    omniphysicsGroupElemCounts1 = source.internToken("omniphysics:groupElemCounts1");
    omniphysicsGroupElemIndices0 = source.internToken("omniphysics:groupElemIndices0");
    omniphysicsGroupElemIndices1 = source.internToken("omniphysics:groupElemIndices1");
    omniphysicsLocalPositionsSrc1 = source.internToken("omniphysics:localPositionsSrc1");
    omniphysicsPoissonsRatio = source.internToken("omniphysics:poissonsRatio");
    omniphysicsRestBendAnglesDefault = source.internToken("omniphysics:restBendAnglesDefault");
    omniphysicsRestShapePoints = source.internToken("omniphysics:restShapePoints");
    omniphysicsRestTetVtxIndices = source.internToken("omniphysics:restTetVtxIndices");
    omniphysicsRestTriVtxIndices = source.internToken("omniphysics:restTriVtxIndices");
    omniphysicsStaticFriction = source.internToken("omniphysics:staticFriction");
    omniphysicsSurfaceBendStiffness = source.internToken("omniphysics:surfaceBendStiffness");
    OmniPhysicsSurfaceDeformableSimAPI = source.internToken("OmniPhysicsSurfaceDeformableSimAPI");
    omniphysicsSurfaceShearStiffness = source.internToken("omniphysics:surfaceShearStiffness");
    omniphysicsSurfaceStretchStiffness = source.internToken("omniphysics:surfaceStretchStiffness");
    omniphysicsSurfaceThickness = source.internToken("omniphysics:surfaceThickness");
    omniphysicsTetCoordsSrc0 = source.internToken("omniphysics:tetCoordsSrc0");
    omniphysicsTetCoordsSrc1 = source.internToken("omniphysics:tetCoordsSrc1");
    omniphysicsTetIndicesSrc0 = source.internToken("omniphysics:tetIndicesSrc0");
    omniphysicsTetIndicesSrc1 = source.internToken("omniphysics:tetIndicesSrc1");
    OmniPhysicsTetXformAttachment = source.internToken("OmniPhysicsTetXformAttachment");
    omniphysicsTriCoordsSrc1 = source.internToken("omniphysics:triCoordsSrc1");
    omniphysicsTriIndicesSrc1 = source.internToken("omniphysics:triIndicesSrc1");
    OmniPhysicsVolumeDeformableSimAPI = source.internToken("OmniPhysicsVolumeDeformableSimAPI");
    omniphysicsVtxIndicesSrc0 = source.internToken("omniphysics:vtxIndicesSrc0");
    omniphysicsVtxIndicesSrc1 = source.internToken("omniphysics:vtxIndicesSrc1");
    OmniPhysicsVtxTetAttachment = source.internToken("OmniPhysicsVtxTetAttachment");
    OmniPhysicsVtxTriAttachment = source.internToken("OmniPhysicsVtxTriAttachment");
    OmniPhysicsVtxVtxAttachment = source.internToken("OmniPhysicsVtxVtxAttachment");
    OmniPhysicsVtxXformAttachment = source.internToken("OmniPhysicsVtxXformAttachment");
    omniphysicsYoungsModulus = source.internToken("omniphysics:youngsModulus");

    // UsdPhysicsTokens attributes (schema-generated UsdPhysics token table)
    angular = source.internToken("angular");
    linear = source.internToken("linear");
    physicsCollisionEnabled = source.internToken("physics:collisionEnabled");
    physicsConeAngle0Limit = source.internToken("physics:coneAngle0Limit");
    physicsConeAngle1Limit = source.internToken("physics:coneAngle1Limit");
    physicsLowerLimit = source.internToken("physics:lowerLimit");
    physicsMaxDistance = source.internToken("physics:maxDistance");
    physicsMinDistance = source.internToken("physics:minDistance");
    physicsSimulationOwner = source.internToken("physics:simulationOwner");
    physicsUpperLimit = source.internToken("physics:upperLimit");
    transX = source.internToken("transX");
    transY = source.internToken("transY");
    transZ = source.internToken("transZ");
    x = source.internToken("X");
    y = source.internToken("Y");
    z = source.internToken("Z");

    // PhysxAxisInstanceTokens attributes (schema-generated per-axis PhysxJointAxisAPI/PhysxDrivePerformanceEnvelopeAPI instance token table)
    armatureAngular = source.internToken("physxJointAxis:angular:armature");
    armatureLinear = source.internToken("physxJointAxis:linear:armature");
    armatureRotX = source.internToken("physxJointAxis:rotX:armature");
    armatureRotY = source.internToken("physxJointAxis:rotY:armature");
    armatureRotZ = source.internToken("physxJointAxis:rotZ:armature");
    dynamicFrictionEffortAngular = source.internToken("physxJointAxis:angular:dynamicFrictionEffort");
    dynamicFrictionEffortLinear = source.internToken("physxJointAxis:linear:dynamicFrictionEffort");
    dynamicFrictionEffortRotX = source.internToken("physxJointAxis:rotX:dynamicFrictionEffort");
    dynamicFrictionEffortRotY = source.internToken("physxJointAxis:rotY:dynamicFrictionEffort");
    dynamicFrictionEffortRotZ = source.internToken("physxJointAxis:rotZ:dynamicFrictionEffort");
    maxActuatorVelocityAngular = source.internToken("physxDrivePerformanceEnvelope:angular:maxActuatorVelocity");
    maxActuatorVelocityLinear = source.internToken("physxDrivePerformanceEnvelope:linear:maxActuatorVelocity");
    maxActuatorVelocityRotX = source.internToken("physxDrivePerformanceEnvelope:rotX:maxActuatorVelocity");
    maxActuatorVelocityRotY = source.internToken("physxDrivePerformanceEnvelope:rotY:maxActuatorVelocity");
    maxActuatorVelocityRotZ = source.internToken("physxDrivePerformanceEnvelope:rotZ:maxActuatorVelocity");
    maxJointVelocityAngular = source.internToken("physxJointAxis:angular:maxJointVelocity");
    maxJointVelocityLinear = source.internToken("physxJointAxis:linear:maxJointVelocity");
    maxJointVelocityRotX = source.internToken("physxJointAxis:rotX:maxJointVelocity");
    maxJointVelocityRotY = source.internToken("physxJointAxis:rotY:maxJointVelocity");
    maxJointVelocityRotZ = source.internToken("physxJointAxis:rotZ:maxJointVelocity");
    speedEffortGradientAngular = source.internToken("physxDrivePerformanceEnvelope:angular:speedEffortGradient");
    speedEffortGradientLinear = source.internToken("physxDrivePerformanceEnvelope:linear:speedEffortGradient");
    speedEffortGradientRotX = source.internToken("physxDrivePerformanceEnvelope:rotX:speedEffortGradient");
    speedEffortGradientRotY = source.internToken("physxDrivePerformanceEnvelope:rotY:speedEffortGradient");
    speedEffortGradientRotZ = source.internToken("physxDrivePerformanceEnvelope:rotZ:speedEffortGradient");
    staticFrictionEffortAngular = source.internToken("physxJointAxis:angular:staticFrictionEffort");
    staticFrictionEffortLinear = source.internToken("physxJointAxis:linear:staticFrictionEffort");
    staticFrictionEffortRotX = source.internToken("physxJointAxis:rotX:staticFrictionEffort");
    staticFrictionEffortRotY = source.internToken("physxJointAxis:rotY:staticFrictionEffort");
    staticFrictionEffortRotZ = source.internToken("physxJointAxis:rotZ:staticFrictionEffort");
    velocityDependentResistanceAngular = source.internToken("physxDrivePerformanceEnvelope:angular:velocityDependentResistance");
    velocityDependentResistanceLinear = source.internToken("physxDrivePerformanceEnvelope:linear:velocityDependentResistance");
    velocityDependentResistanceRotX = source.internToken("physxDrivePerformanceEnvelope:rotX:velocityDependentResistance");
    velocityDependentResistanceRotY = source.internToken("physxDrivePerformanceEnvelope:rotY:velocityDependentResistance");
    velocityDependentResistanceRotZ = source.internToken("physxDrivePerformanceEnvelope:rotZ:velocityDependentResistance");
    viscousFrictionCoefficientAngular = source.internToken("physxJointAxis:angular:viscousFrictionCoefficient");
    viscousFrictionCoefficientLinear = source.internToken("physxJointAxis:linear:viscousFrictionCoefficient");
    viscousFrictionCoefficientRotX = source.internToken("physxJointAxis:rotX:viscousFrictionCoefficient");
    viscousFrictionCoefficientRotY = source.internToken("physxJointAxis:rotY:viscousFrictionCoefficient");
    viscousFrictionCoefficientRotZ = source.internToken("physxJointAxis:rotZ:viscousFrictionCoefficient");

    // Instance-qualified applied-schema names ("<baseSchema>:<instance>") for
    // PhysxMimicJointAPI / PhysxDrivePerformanceEnvelopeAPI / PhysxJointAxisAPI,
    // matched against the raw apiSchemas metadata list (structural add/remove
    // detection), not read as attributes.
    physxMimicJointAPIRotX = source.internToken("PhysxMimicJointAPI:rotX");
    physxMimicJointAPIRotY = source.internToken("PhysxMimicJointAPI:rotY");
    physxMimicJointAPIRotZ = source.internToken("PhysxMimicJointAPI:rotZ");
    physxDrivePerformanceEnvelopeAPIAngular = source.internToken("PhysxDrivePerformanceEnvelopeAPI:angular");
    physxDrivePerformanceEnvelopeAPILinear = source.internToken("PhysxDrivePerformanceEnvelopeAPI:linear");
    physxDrivePerformanceEnvelopeAPIRotX = source.internToken("PhysxDrivePerformanceEnvelopeAPI:rotX");
    physxDrivePerformanceEnvelopeAPIRotY = source.internToken("PhysxDrivePerformanceEnvelopeAPI:rotY");
    physxDrivePerformanceEnvelopeAPIRotZ = source.internToken("PhysxDrivePerformanceEnvelopeAPI:rotZ");
    physxJointAxisAPIAngular = source.internToken("PhysxJointAxisAPI:angular");
    physxJointAxisAPILinear = source.internToken("PhysxJointAxisAPI:linear");
    physxJointAxisAPIRotX = source.internToken("PhysxJointAxisAPI:rotX");
    physxJointAxisAPIRotY = source.internToken("PhysxJointAxisAPI:rotY");
    physxJointAxisAPIRotZ = source.internToken("PhysxJointAxisAPI:rotZ");
    physicsDriveAPIAngular = source.internToken("PhysicsDriveAPI:angular");
    physicsDriveAPILinear = source.internToken("PhysicsDriveAPI:linear");
    physicsDriveAPIRotX = source.internToken("PhysicsDriveAPI:rotX");
    physicsDriveAPIRotY = source.internToken("PhysicsDriveAPI:rotY");
    physicsDriveAPIRotZ = source.internToken("PhysicsDriveAPI:rotZ");
    driveMaxForceAngular = source.internToken("drive:angular:physics:maxForce");
    driveMaxForceLinear = source.internToken("drive:linear:physics:maxForce");
    driveMaxForceRotX = source.internToken("drive:rotX:physics:maxForce");
    driveMaxForceRotY = source.internToken("drive:rotY:physics:maxForce");
    driveMaxForceRotZ = source.internToken("drive:rotZ:physics:maxForce");

    // Non-D6 joint gate tokens (revolute/prismatic/spherical/distance) so the per-joint
    // hasSchema gate reads a cached TokenId; D6 per-axis instances intern at runtime.
    physxLimitAPIAngular = source.internToken("PhysxLimitAPI:angular");
    physxLimitAPILinear = source.internToken("PhysxLimitAPI:linear");
    physxLimitAPICone = source.internToken("PhysxLimitAPI:cone");
    physxLimitAPIDistance = source.internToken("PhysxLimitAPI:distance");
    physicsJointStateAPIAngular = source.internToken("PhysicsJointStateAPI:angular");
    physicsJointStateAPILinear = source.internToken("PhysicsJointStateAPI:linear");
    physxPhysicsDistanceJointAPI = source.internToken("PhysxPhysicsDistanceJointAPI");

    // NewtonSchemaTokens attributes (schema-generated Newton token table)
    NewtonMimicAPI = source.internToken("NewtonMimicAPI");
    newtonMimicCoef0 = source.internToken("newton:mimicCoef0");
    newtonMimicCoef1 = source.internToken("newton:mimicCoef1");
    newtonMimicEnabled = source.internToken("newton:mimicEnabled");
    newtonMimicJoint = source.internToken("newton:mimicJoint");

    // UsdTokens attributes (schema-generated core Usd token table)
    apiSchemas = source.internToken("apiSchemas");

    // Prim/schema type-name tokens (ADR-0018), for isA()/hasSchema() gates that
    // previously went through PhysXTools.h's schemaTypeToken<T>()/isAType<T>().
    meshType = source.internToken("Mesh");
    capsuleType = source.internToken("Capsule");
    cylinderType = source.internToken("Cylinder");
    coneType = source.internToken("Cone");
    cubeType = source.internToken("Cube");
    sphereType = source.internToken("Sphere");
    gprimType = source.internToken("Gprim");
    xformableType = source.internToken("Xformable");
    xformType = source.internToken("Xform");
    pointBasedType = source.internToken("PointBased");
    pointsType = source.internToken("Points");
    pointInstancerType = source.internToken("PointInstancer");
    tetMeshType = source.internToken("TetMesh");
    physicsCollisionGroupType = source.internToken("PhysicsCollisionGroup");
    physicsJointStateAPI = source.internToken("PhysicsJointStateAPI");
    physxParticleSystemType = source.internToken("PhysxParticleSystem");
    physxPhysicsJointInstancerType = source.internToken("PhysxPhysicsJointInstancer");
    physxVehicleTireFrictionTableType = source.internToken("PhysxVehicleTireFrictionTable");
}

} // namespace omni::physics::parse
