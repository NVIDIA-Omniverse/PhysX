// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-CORE-001
 * @covers AC-1
 *
 * @implements REQ-PARSE-CORE-002
 * @covers AC-2
 *
 * @implements REQ-PARSE-JOINT-002
 * @covers AC-2
 */

#pragma once

#include "Handles.h"

namespace omni::physics::parse
{

class IPhysicsSource;

// ---------------------------------------------------------------------------
// KnownTokens — well-known schema attribute names, batch-interned at attach
// time so parsers do integer-compare lookups with zero string ops on hot paths.
//
// Populated by KnownTokens::intern() from the source's token table.
// ---------------------------------------------------------------------------

struct KnownTokens
{
    // Material attributes
    TokenId physicsMaterialAPI;
    TokenId physxMaterialAPI;
    TokenId staticFriction;
    TokenId dynamicFriction;
    TokenId restitution;
    TokenId density;
    TokenId frictionCombineMode;
    TokenId restitutionCombineMode;
    TokenId dampingCombineMode;
    TokenId compliantContactAccelerationSpring;
    TokenId compliantContactStiffness;
    TokenId compliantContactDamping;

    // Combine mode values
    TokenId average;
    TokenId min;
    TokenId max;
    TokenId multiply;

    // Scene attributes
    TokenId physicsScene;
    TokenId gravityDirection;
    TokenId gravityMagnitude;

    // PhysxSceneAPI extension attributes
    TokenId physxSceneAPI;
    TokenId physxSceneUpdateType;
    TokenId physxSceneBounceThreshold;
    TokenId physxSceneFrictionOffsetThreshold;
    TokenId physxSceneFrictionCorrelationDistance;
    TokenId physxSceneMaxBiasCoefficient;
    TokenId physxSceneTimeStepsPerSecond;
    TokenId physxSceneMinPositionIterationCount;
    TokenId physxSceneMaxPositionIterationCount;
    TokenId physxSceneMinVelocityIterationCount;
    TokenId physxSceneMaxVelocityIterationCount;
    TokenId physxSceneEnableCCD;
    TokenId physxSceneEnableStabilization;
    TokenId physxSceneEnableGPUDynamics;
    TokenId physxSceneEnableEnhancedDeterminism;
    TokenId physxSceneEnableExternalForcesEveryIteration;
    TokenId physxSceneInvertCollisionGroupFilter;
    TokenId physxSceneReportKinematicKinematicPairs;
    TokenId physxSceneReportKinematicStaticPairs;
    TokenId physxSceneEnableSceneQuerySupport;
    TokenId physxSceneSolveArticulationContactLast;
    TokenId physxSceneDisableSleeping;
    TokenId physxSceneCollisionSystem;
    TokenId physxSceneSolverType;
    TokenId physxSceneBroadphaseType;
    TokenId physxSceneFrictionType;
    TokenId physxSceneGpuTempBufferCapacity;
    TokenId physxSceneGpuMaxRigidContactCount;
    TokenId physxSceneGpuMaxRigidPatchCount;
    TokenId physxSceneGpuHeapCapacity;
    TokenId physxSceneGpuFoundLostPairsCapacity;
    TokenId physxSceneGpuFoundLostAggregatePairsCapacity;
    TokenId physxSceneGpuTotalAggregatePairsCapacity;
    TokenId physxSceneGpuMaxDeformableVolumeContacts;
    TokenId physxSceneGpuMaxDeformableSurfaceContacts;
    TokenId physxSceneGpuMaxParticleContacts;
    TokenId physxSceneGpuCollisionStackSize;
    TokenId physxSceneGpuMaxNumPartitions;
    TokenId physxSceneEnvIdInBoundsBitCount;

    // PhysxSceneAPI updateType token enum (Synchronous / Asynchronous / Disabled).
    TokenId sceneUpdateSynchronous;
    TokenId sceneUpdateAsynchronous;
    TokenId sceneUpdateDisabled;
    // collisionSystem token enum (PCM / SAT).
    TokenId collisionSystemPCM;
    TokenId collisionSystemSAT;
    // solverType token enum (TGS / PGS).
    TokenId solverTypeTGS;
    TokenId solverTypePGS;
    // broadphaseType token enum (MBP / SAP / GPU).
    TokenId broadphaseTypeMBP;
    TokenId broadphaseTypeSAP;
    TokenId broadphaseTypeGPU;

    // PhysxSceneQuasistaticAPI extension attributes
    TokenId physxSceneQuasistaticAPI;
    TokenId physxSceneQuasistaticEnableQuasistatic;

    // Newton scene fallbacks (PhysxSceneAPI fallbacks).
    TokenId newtonTimeStepsPerSecond;
    TokenId newtonGravityEnabled;

    // Body attributes
    TokenId physicsRigidBodyAPI;
    TokenId physicsCollisionAPI;
    TokenId physicsMassAPI;
    TokenId physicsMass;
    TokenId physicsDensity;
    TokenId physicsCenterOfMass;
    TokenId physicsDiagonalInertia;
    TokenId physicsPrincipalAxes;
    TokenId physicsVelocity;
    TokenId physicsAngularVelocity;
    TokenId physicsRigidBodyEnabled;
    TokenId physicsStartsAsleep;
    TokenId physicsKinematicEnabled;

    // PhysxRigidBodyAPI extension attributes
    TokenId physxRigidBodyAPI;
    TokenId physxRigidBodyLinearDamping;
    TokenId physxRigidBodyAngularDamping;
    TokenId physxRigidBodyMaxLinearVelocity;
    TokenId physxRigidBodyMaxAngularVelocity;
    TokenId physxRigidBodySleepThreshold;
    TokenId physxRigidBodyStabilizationThreshold;
    TokenId physxRigidBodyMaxDepenetrationVelocity;
    TokenId physxRigidBodyContactSlopCoefficient;
    TokenId physxRigidBodyMaxContactImpulse;
    TokenId physxRigidBodyCfmScale;
    TokenId physxRigidBodySolverPositionIterationCount;
    TokenId physxRigidBodySolverVelocityIterationCount;
    TokenId physxRigidBodyEnableCCD;
    TokenId physxRigidBodyEnableSpeculativeCCD;
    TokenId physxRigidBodyDisableGravity;
    TokenId physxRigidBodyRetainAccelerations;
    TokenId physxRigidBodyEnableGyroscopicForces;
    TokenId physxRigidBodySolveContact;
    TokenId physxRigidBodyLockedPosAxis;
    TokenId physxRigidBodyLockedRotAxis;

    // PhysxSurfaceVelocityAPI
    TokenId physxSurfaceVelocityAPI;
    TokenId physxSurfaceVelocityEnabled;
    TokenId physxSurfaceVelocityLocalSpace;
    TokenId physxSurfaceVelocity;
    TokenId physxSurfaceAngularVelocity;

    // PhysxSplinesSurfaceVelocityAPI
    TokenId physxSplinesSurfaceVelocityAPI;
    TokenId physxSplinesSurfaceVelocityEnabled;
    TokenId physxSplinesSurfaceVelocityMagnitude;
    TokenId physxSplinesSurfaceVelocityCurve;
    // UsdGeom typeName for the splines curve target validation.
    TokenId basisCurvesType;

    // Prim metadata used for body parsing
    TokenId metadataLocalSpaceVelocities;

    // Joint attributes
    TokenId physicsJoint;
    TokenId physicsBody0;
    TokenId physicsBody1;
    TokenId physicsLocalPos0;
    TokenId physicsLocalPos1;
    TokenId physicsLocalRot0;
    TokenId physicsLocalRot1;
    TokenId physicsJointEnabled;
    TokenId physicsBreakForce;
    TokenId physicsBreakTorque;

    // PhysxJointAPI extensions
    TokenId physxJointAPI;
    TokenId physxJointJointFriction;
    // Fallback fields read from PhysxJointAPI when PhysxJointAxisAPI is
    // not applied for the current axis.
    TokenId physxJointArmature;
    TokenId physxJointMaxJointVelocity;

    // PhysxCollisionAPI extensions (subset routed through parse library)
    TokenId physxCollisionAPI;
    TokenId physxCollisionTorsionalPatchRadius;
    TokenId physxCollisionMinTorsionalPatchRadius;
    TokenId physxCollisionContactOffset;
    TokenId physxCollisionRestOffset;

    // physxConvexGeometry:margin (Cylinder/Cone shapes)
    TokenId physxConvexGeometryMargin;

    // PhysxCharacterControllerAPI: physxCharacterController:slopeLimit
    // (read by parseCct).
    TokenId physxCharacterControllerSlopeLimit;

    // UsdPhysicsMeshCollisionAPI: physics:approximation
    TokenId usdPhysicsMeshCollisionAPI;
    TokenId physicsApproximation;
    // Approximation token enum values. "none"/"convexHull"/"boundingSphere"/
    // "boundingCube"/"meshSimplification" come from UsdPhysicsTokens; the
    // remaining three (convexDecomposition/sphereFill/sdf) come from
    // PhysxSchemaTokens. All are interned by string; parsers compare by id.
    TokenId approximationNone;
    TokenId approximationConvexHull;
    TokenId approximationBoundingSphere;
    TokenId approximationBoundingCube;
    TokenId approximationMeshSimplification;
    TokenId approximationConvexDecomposition;
    TokenId approximationSphereFill;
    TokenId approximationSdf;

    // PhysxTriggerAPI / PhysxTriggerStateAPI — single-apply schemas.
    // Their presence flips PhysxShapeDesc::isTrigger / isTriggerUsdOutput.
    TokenId physxTriggerAPI;
    TokenId physxTriggerStateAPI;

    // PhysxConvexHullCollisionAPI cooking-knob extensions.
    TokenId physxConvexHullCollisionAPI;
    TokenId physxConvexHullCollisionHullVertexLimit;
    TokenId physxConvexHullCollisionMinThickness;
    // Newton fallback for hullVertexLimit when the PhysX value is not
    // authored. Only applies when the int attribute resolves to > 0.
    TokenId newtonMaxHullVertices;
    // Newton fallbacks for contactOffset/restOffset. Apply when the
    // corresponding PhysX attribute is unauthored.
    TokenId newtonContactMargin;
    TokenId newtonContactGap;

    // PhysxConvexDecompositionCollisionAPI cooking-knob extensions.
    TokenId physxConvexDecompositionCollisionAPI;
    TokenId physxConvexDecompositionCollisionMinThickness;
    TokenId physxConvexDecompositionCollisionMaxConvexHulls;
    TokenId physxConvexDecompositionCollisionHullVertexLimit;
    TokenId physxConvexDecompositionCollisionVoxelResolution;
    TokenId physxConvexDecompositionCollisionErrorPercentage;
    TokenId physxConvexDecompositionCollisionShrinkWrap;

    // PhysxSphereFillCollisionAPI cooking-knob extensions.
    TokenId physxSphereFillCollisionAPI;
    TokenId physxSphereFillCollisionMaxSpheres;
    TokenId physxSphereFillCollisionSeedCount;
    TokenId physxSphereFillCollisionVoxelResolution;
    TokenId physxSphereFillCollisionFillMode;
    // SphereFillMode token enum values: "flood" / "raycast" / "surface".
    TokenId fillModeFlood;
    TokenId fillModeRaycast;
    TokenId fillModeSurface;

    // PhysxTriangleMeshCollisionAPI / PhysxTriangleMeshSimplificationCollisionAPI
    // cooking-knob extensions. Both write into TriangleMeshCookingParams.
    TokenId physxTriangleMeshCollisionAPI;
    TokenId physxTriangleMeshCollisionWeldTolerance;
    TokenId physxTriangleMeshSimplificationCollisionAPI;
    TokenId physxTriangleMeshSimplificationCollisionMetric;
    TokenId physxTriangleMeshSimplificationCollisionWeldTolerance;

    // PhysxSDFMeshCollisionAPI cooking-knob extensions.
    TokenId physxSDFMeshCollisionAPI;
    TokenId physxSDFMeshCollisionSdfResolution;
    TokenId physxSDFMeshCollisionSdfSubgridResolution;
    TokenId physxSDFMeshCollisionSdfBitsPerSubgridPixel;
    TokenId physxSDFMeshCollisionSdfNarrowBandThickness;
    TokenId physxSDFMeshCollisionSdfMargin;
    TokenId physxSDFMeshCollisionSdfEnableRemeshing;
    TokenId physxSDFMeshCollisionSdfTriangleCountReductionFactor;
    // sdfBitsPerSubgridPixel token enum values: "BitsPerPixel8" /
    // "BitsPerPixel16" / "BitsPerPixel32".
    TokenId bitsPerPixel8;
    TokenId bitsPerPixel16;
    TokenId bitsPerPixel32;

    // PhysxArticulationAPI extensions
    TokenId physxArticulationAPI;
    TokenId physxArticulationEnabled;
    TokenId physxArticulationSleepThreshold;
    TokenId physxArticulationStabilizationThreshold;
    TokenId physxArticulationSolverPositionIterationCount;
    TokenId physxArticulationSolverVelocityIterationCount;
    TokenId physxArticulationEnabledSelfCollisions;
    // Newton fallback (when PhysxArticulationAPI's selfCollisions isn't authored).
    TokenId newtonSelfCollisionEnabled;

    // PhysicsFilteredPairsAPI — applied API + the filteredPairs relationship.
    // Shared by rigid-body / collision-shape / articulation / deformable-body
    // descriptors via a per-prim filter list.
    TokenId physicsFilteredPairsAPI;
    TokenId physicsFilteredPairs;

    // UsdPhysicsCollisionGroup — the filteredGroups relationship and the
    // "colliders" collection name.
    TokenId physicsFilteredGroups;
    TokenId collidersCollectionName;

    // OmniPhysicsDeformable attachments + element collision filters.
    // Shared between all 7 attachment subtypes and the element-collision-
    // filter prim.
    TokenId omniphysicsAttachmentEnabled;
    TokenId omniphysicsFilterEnabled;
    TokenId omniphysicsDamping;
    TokenId omniphysicsStiffness;
    TokenId omniphysicsSrc0;
    TokenId omniphysicsSrc1;

    // OmniPhysicsDeformable body schemas (BodyAPI + DeformableBodyAPI).
    TokenId omniphysicsBodyAPI;
    TokenId omniphysicsDeformableBodyAPI;
    TokenId omniphysicsDeformableBodyEnabled;
    TokenId omniphysicsMass;
    TokenId omniphysicsKinematicEnabled;
    TokenId omniphysicsStartsAsleep;
    TokenId omniphysicsSimulationOwner;

    void intern(const IPhysicsSource& source);
};

} // namespace omni::physics::parse
