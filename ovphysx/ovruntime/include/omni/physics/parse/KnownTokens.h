// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CORE-001
 * @covers AC-1
 *
 * @implements REQ-SPLINE-CURVE-001
 * @covers AC-6
 *
 * @implements REQ-PARSE-CORE-002
 * @covers AC-2
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

#pragma once

#include "Handles.h"

namespace omni::physics::parse
{

class IPhysicsSource;

// ---------------------------------------------------------------------------
// KnownTokens - well-known schema attribute names, populated as one batch and
// reused by its owner. AttachedStage owners populate the batch at source attach;
// ParseContext owners populate it lazily on first use. Parsers then do
// integer-compare lookups with zero string operations on hot paths.
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
    // NewtonJointAPI fallback for physxJoint:maxJointVelocity.
    TokenId newtonVelocityLimit;

    // PhysxCollisionAPI extensions (subset routed through parse library)
    TokenId physxCollisionAPI;
    TokenId physxCollisionTorsionalPatchRadius;
    TokenId physxCollisionMinTorsionalPatchRadius;
    TokenId physxCollisionContactOffset;
    TokenId physxCollisionRestOffset;

    // physxConvexGeometry:margin (Cylinder/Cone shapes)
    TokenId physxConvexGeometryMargin;

    // PhysxCharacterControllerAPI applied-schema name (hasSchema gate) and
    // physxCharacterController:slopeLimit (read by parseCct).
    TokenId physxCharacterControllerAPI;
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

    // PhysxMeshMergeCollisionAPI: implicit mesh merging for collision geometry.
    TokenId physxMeshMergeCollisionAPI;

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

    // PhysxSchemaTokens attributes (schema-generated PhysX token table)
    TokenId acceleration;
    TokenId brakes0;
    TokenId brakes1;
    TokenId constrained;
    TokenId contactOffset;
    TokenId defaultFrictionValue;
    TokenId drive;
    TokenId easy;
    TokenId enableCCD;
    TokenId fluidRestOffset;
    TokenId frictionValues;
    TokenId groundMaterials;
    TokenId maxDepenetrationVelocity;
    TokenId maxNeighborhood;
    TokenId maxVelocity;
    TokenId neighborhoodScale;
    TokenId particleContactOffset;
    TokenId particleSystemEnabled;
    TokenId physicsBody0Indices;
    TokenId physicsBody0s;
    TokenId physicsBody1Indices;
    TokenId physicsBody1s;
    TokenId physicsGearRatio;
    TokenId physicsHinge;
    TokenId physicsHinge0;
    TokenId physicsHinge1;
    TokenId physicsLocalPos0s;
    TokenId physicsLocalPos1s;
    TokenId physicsLocalRot0s;
    TokenId physicsLocalRot1s;
    TokenId physicsPrismatic;
    TokenId physicsProtoIndices;
    TokenId physicsPrototypes;
    TokenId physicsRatio;
    TokenId PhysxAutoDeformableAttachmentAPI;
    TokenId physxAutoDeformableAttachmentAttachable0;
    TokenId physxAutoDeformableAttachmentAttachable1;
    TokenId physxAutoDeformableAttachmentCollisionFilteringOffset;
    TokenId physxAutoDeformableAttachmentDeformableVertexOverlapOffset;
    TokenId physxAutoDeformableAttachmentEnableCollisionFiltering;
    TokenId physxAutoDeformableAttachmentEnableDeformableFilteringPairs;
    TokenId physxAutoDeformableAttachmentEnableDeformableVertexAttachments;
    TokenId physxAutoDeformableAttachmentEnableRigidSurfaceAttachments;
    TokenId physxAutoDeformableAttachmentMaskShapes;
    TokenId physxAutoDeformableAttachmentRigidSurfaceSamplingDistance;
    TokenId PhysxAutoDeformableBodyAPI;
    TokenId PhysxAutoDeformableHexahedralMeshAPI;
    TokenId PhysxAutoDeformableMeshSimplificationAPI;
    TokenId physxCharacterControllerClimbingMode;
    TokenId physxCharacterControllerContactOffset;
    TokenId physxCharacterControllerInvisibleWallHeight;
    TokenId physxCharacterControllerMaxJumpHeight;
    TokenId physxCharacterControllerNonWalkableMode;
    TokenId physxCharacterControllerScaleCoeff;
    TokenId physxCharacterControllerStepOffset;
    TokenId physxCharacterControllerUpAxis;
    TokenId physxCharacterControllerVolumeGrowth;
    TokenId physxContactReportAPI;
    TokenId physxContactReportReportPairs;
    TokenId physxContactReportThreshold;
    TokenId physxDeformableBodyAutoDeformableBodyEnabled;
    TokenId physxDeformableBodyAutoDeformableMeshSimplificationEnabled;
    TokenId physxDeformableBodyCollisionIterationMultiplier;
    TokenId physxDeformableBodyCollisionPairUpdateFrequency;
    TokenId physxDeformableBodyDisableGravity;
    TokenId physxDeformableBodyEnableSpeculativeCCD;
    TokenId physxDeformableBodyForceConforming;
    TokenId physxDeformableBodyLinearDamping;
    TokenId physxDeformableBodyMaxDepenetrationVelocity;
    TokenId physxDeformableBodyMaxLinearVelocity;
    TokenId physxDeformableBodyRemeshingEnabled;
    TokenId physxDeformableBodyRemeshingResolution;
    TokenId physxDeformableBodyResolution;
    TokenId physxDeformableBodySelfCollision;
    TokenId physxDeformableBodySelfCollisionFilterDistance;
    TokenId physxDeformableBodySettlingDamping;
    TokenId physxDeformableBodySettlingThreshold;
    TokenId physxDeformableBodySleepThreshold;
    TokenId physxDeformableBodySolverPositionIterationCount;
    TokenId physxDeformableBodyTargetTriangleCount;
    TokenId physxDeformableMaterialBendDamping;
    TokenId physxDeformableMaterialElasticityDamping;
    TokenId physxDiffuseParticlesAPI;
    TokenId physxDiffuseParticlesAirDrag;
    TokenId physxDiffuseParticlesBubbleDrag;
    TokenId physxDiffuseParticlesBuoyancy;
    TokenId physxDiffuseParticlesCollisionDecay;
    TokenId physxDiffuseParticlesDiffuseParticlesEnabled;
    TokenId physxDiffuseParticlesDivergenceWeight;
    TokenId physxDiffuseParticlesKineticEnergyWeight;
    TokenId physxDiffuseParticlesLifetime;
    TokenId physxDiffuseParticlesMaxDiffuseParticleMultiplier;
    TokenId physxDiffuseParticlesPressureWeight;
    TokenId physxDiffuseParticlesThreshold;
    TokenId PhysxDrivePerformanceEnvelopeAPI;
    TokenId physxForceAPI;
    TokenId physxForceForce;
    TokenId physxForceForceEnabled;
    TokenId physxForceMode;
    TokenId physxForceTorque;
    TokenId physxForceWorldFrameEnabled;
    TokenId PhysxJointAxisAPI;
    TokenId physxMimicJoint_MultipleApplyTemplate_DampingRatio;
    TokenId physxMimicJoint_MultipleApplyTemplate_Gearing;
    TokenId physxMimicJoint_MultipleApplyTemplate_NaturalFrequency;
    TokenId physxMimicJoint_MultipleApplyTemplate_Offset;
    TokenId physxMimicJoint_MultipleApplyTemplate_ReferenceJoint;
    TokenId physxMimicJoint_MultipleApplyTemplate_ReferenceJointAxis;
    TokenId physxParticleAnisotropyAPI;
    TokenId physxParticleAnisotropyParticleAnisotropyEnabled;
    // PhysxParticleAPI applied-schema name (hasSchema gate).
    TokenId physxParticleAPI;
    TokenId physxParticleFluid;
    TokenId physxParticleIsosurfaceAPI;
    TokenId physxParticleIsosurfaceGridFilteringPasses;
    TokenId physxParticleIsosurfaceGridSmoothingRadius;
    TokenId physxParticleIsosurfaceIsosurfaceEnabled;
    TokenId physxParticleIsosurfaceNumMeshNormalSmoothingPasses;
    TokenId physxParticleIsosurfaceNumMeshSmoothingPasses;
    TokenId physxParticleIsosurfaceSurfaceDistance;
    TokenId physxParticleParticleEnabled;
    TokenId physxParticleParticleGroup;
    TokenId physxParticleSamplingAPI;
    TokenId physxParticleSelfCollision;
    TokenId physxParticleSetAPI;
    TokenId physxParticleSimulationPoints;
    TokenId physxParticleSmoothingAPI;
    TokenId physxParticleSmoothingParticleSmoothingEnabled;
    TokenId physxPBDMaterialAdhesion;
    TokenId physxPBDMaterialAdhesionOffsetScale;
    TokenId physxPBDMaterialCflCoefficient;
    TokenId physxPBDMaterialCohesion;
    TokenId physxPBDMaterialDamping;
    TokenId physxPBDMaterialDensity;
    TokenId physxPBDMaterialFriction;
    TokenId physxPBDMaterialGravityScale;
    TokenId physxPBDMaterialParticleAdhesionScale;
    TokenId physxPBDMaterialParticleFrictionScale;
    TokenId physxPBDMaterialSurfaceTension;
    TokenId physxPBDMaterialViscosity;
    TokenId physxPBDMaterialVorticityConfinement;
    TokenId physxPhysicsDistanceJointSpringDamping;
    TokenId physxPhysicsDistanceJointSpringEnabled;
    TokenId physxPhysicsDistanceJointSpringStiffness;
    TokenId schemaQuasistaticEnableQuasistatic;
    // PhysxTendonAttachmentAPI/PhysxTendonAttachmentLeafAPI/PhysxTendonAttachmentRootAPI/
    // PhysxTendonAxisAPI/PhysxTendonAxisRootAPI applied-schema names (hasSchema gate; multi-apply --
    // hasSchema matches if at least one instance is applied).
    TokenId physxTendonAttachmentAPI;
    TokenId physxTendonAttachmentLeafAPI;
    TokenId physxTendonAttachmentRootAPI;
    TokenId physxTendonAxisAPI;
    TokenId physxTendonAxisRootAPI;
    TokenId physxVehicleAPI;
    TokenId physxVehicleAckermannSteeringAPI;
    TokenId physxVehicleAckermannSteeringMaxSteerAngle;
    TokenId physxVehicleAckermannSteeringStrength;
    TokenId physxVehicleAckermannSteeringTrackWidth;
    TokenId physxVehicleAckermannSteeringWheel0;
    TokenId physxVehicleAckermannSteeringWheel1;
    TokenId physxVehicleAckermannSteeringWheelBase;
    TokenId physxVehicleBrakesAPI;
    TokenId physxVehicleBrakes_MultipleApplyTemplate_MaxBrakeTorque;
    TokenId physxVehicleBrakes_MultipleApplyTemplate_TorqueMultipliers;
    TokenId physxVehicleBrakes_MultipleApplyTemplate_Wheels;
    TokenId physxVehicleContextAPI;
    TokenId physxVehicleContextLongitudinalAxis;
    TokenId physxVehicleContextUpdateMode;
    TokenId physxVehicleContextVerticalAxis;
    TokenId physxVehicleControllerAPI;
    TokenId physxVehicleControllerAccelerator;
    TokenId physxVehicleControllerBrake;
    TokenId physxVehicleControllerBrake0;
    TokenId physxVehicleControllerBrake1;
    TokenId physxVehicleControllerHandbrake;
    TokenId physxVehicleControllerSteer;
    TokenId physxVehicleControllerSteerLeft;
    TokenId physxVehicleControllerSteerRight;
    TokenId physxVehicleControllerTargetGear;
    TokenId physxVehicleDrive;
    TokenId physxVehicleDriveBasicAPI;
    TokenId physxVehicleDriveBasicPeakTorque;
    TokenId physxVehicleDriveStandardAPI;
    TokenId physxVehicleEngineDampingRateFullThrottle;
    TokenId physxVehicleEngineDampingRateZeroThrottleClutchDisengaged;
    TokenId physxVehicleEngineDampingRateZeroThrottleClutchEngaged;
    TokenId physxVehicleEngineIdleRotationSpeed;
    TokenId physxVehicleEngineMaxRotationSpeed;
    TokenId physxVehicleEngineMoi;
    TokenId physxVehicleEnginePeakTorque;
    TokenId physxVehicleEngineTorqueCurve;
    TokenId physxVehicleLateralStickyTireDamping;
    TokenId physxVehicleLateralStickyTireThresholdSpeed;
    TokenId physxVehicleLateralStickyTireThresholdTime;
    TokenId physxVehicleLimitSuspensionExpansionVelocity;
    TokenId physxVehicleLongitudinalStickyTireDamping;
    TokenId physxVehicleLongitudinalStickyTireThresholdSpeed;
    TokenId physxVehicleLongitudinalStickyTireThresholdTime;
    TokenId physxVehicleMinActiveLongitudinalSlipDenominator;
    TokenId physxVehicleMinLateralSlipDenominator;
    TokenId physxVehicleMinPassiveLongitudinalSlipDenominator;
    TokenId physxVehicleMultiWheelDifferentialAPI;
    TokenId physxVehicleMultiWheelDifferentialAverageWheelSpeedRatios;
    TokenId physxVehicleMultiWheelDifferentialTorqueRatios;
    TokenId physxVehicleMultiWheelDifferentialWheels;
    TokenId physxVehicleNCR_MultipleApplyTemplate_CommandValues;
    TokenId physxVehicleNCR_MultipleApplyTemplate_SpeedResponses;
    TokenId physxVehicleNCR_MultipleApplyTemplate_SpeedResponsesPerCommandValue;
    TokenId physxVehicleSteeringAPI;
    TokenId physxVehicleSteeringAngleMultipliers;
    TokenId physxVehicleSteeringMaxSteerAngle;
    TokenId physxVehicleSteeringWheels;
    TokenId physxVehicleSuspensionCamberAtMaxCompression;
    TokenId physxVehicleSuspensionCamberAtMaxDroop;
    TokenId physxVehicleSuspensionCamberAtRest;
    TokenId physxVehicleSuspensionComplianceSuspensionForceAppPoint;
    TokenId physxVehicleSuspensionComplianceTireForceAppPoint;
    TokenId physxVehicleSuspensionComplianceWheelCamberAngle;
    TokenId physxVehicleSuspensionComplianceWheelToeAngle;
    TokenId physxVehicleSuspensionMaxCompression;
    TokenId physxVehicleSuspensionMaxDroop;
    TokenId physxVehicleSuspensionSpringDamperRate;
    TokenId physxVehicleSuspensionSpringStrength;
    TokenId physxVehicleSuspensionSprungMass;
    TokenId physxVehicleSuspensionTravelDistance;
    TokenId physxVehicleTankControllerAPI;
    TokenId physxVehicleTankControllerThrust0;
    TokenId physxVehicleTankControllerThrust1;
    TokenId physxVehicleTankDifferentialAPI;
    TokenId physxVehicleTankDifferentialNumberOfWheelsPerTrack;
    TokenId physxVehicleTankDifferentialThrustIndexPerTrack;
    TokenId physxVehicleTankDifferentialTrackToWheelIndices;
    TokenId physxVehicleTankDifferentialWheelIndicesInTrackOrder;
    TokenId physxVehicleTireCamberStiffness;
    TokenId physxVehicleTireCamberStiffnessPerUnitGravity;
    TokenId physxVehicleTireFrictionTable;
    TokenId physxVehicleTireFrictionVsSlipGraph;
    TokenId physxVehicleTireLateralStiffnessGraph;
    TokenId physxVehicleTireLatStiffX;
    TokenId physxVehicleTireLatStiffY;
    TokenId physxVehicleTireLongitudinalStiffness;
    TokenId physxVehicleTireLongitudinalStiffnessPerUnitGravity;
    TokenId physxVehicleTireRestLoad;
    TokenId physxVehicleVehicleEnabled;
    TokenId physxVehicleWheelAttachmentCollisionGroup;
    TokenId physxVehicleWheelAttachmentDriven;
    TokenId physxVehicleWheelAttachmentIndex;
    TokenId physxVehicleWheelAttachmentSuspension;
    TokenId physxVehicleWheelAttachmentSuspensionForceAppPointOffset;
    TokenId physxVehicleWheelAttachmentSuspensionFrameOrientation;
    TokenId physxVehicleWheelAttachmentSuspensionFramePosition;
    TokenId physxVehicleWheelAttachmentSuspensionTravelDirection;
    TokenId physxVehicleWheelAttachmentTire;
    TokenId physxVehicleWheelAttachmentTireForceAppPointOffset;
    TokenId physxVehicleWheelAttachmentWheel;
    TokenId physxVehicleWheelAttachmentWheelCenterOfMassOffset;
    TokenId physxVehicleWheelAttachmentWheelFrameOrientation;
    TokenId physxVehicleWheelAttachmentWheelFramePosition;
    TokenId physxVehicleWheelControllerAPI;
    TokenId physxVehicleWheelControllerBrakeTorque;
    TokenId physxVehicleWheelControllerDriveTorque;
    TokenId physxVehicleWheelControllerSteerAngle;
    TokenId physxVehicleWheelDampingRate;
    TokenId physxVehicleWheelMass;
    TokenId physxVehicleWheelMaxBrakeTorque;
    TokenId physxVehicleWheelMaxHandBrakeTorque;
    TokenId physxVehicleWheelMaxSteerAngle;
    TokenId physxVehicleWheelMoi;
    TokenId physxVehicleWheelRadius;
    TokenId physxVehicleWheelToeAngle;
    TokenId physxVehicleWheelWidth;
    TokenId preventClimbing;
    TokenId preventClimbingForceSliding;
    TokenId quasistaticactors;
    TokenId restOffset;
    TokenId rotX;
    TokenId rotY;
    TokenId rotZ;
    TokenId solidRestOffset;
    TokenId solverPositionIterationCount;
    TokenId steer;

    // UsdGeomTokens attributes (schema-generated UsdGeom token table)
    TokenId angularVelocities;
    TokenId axis;
    TokenId basis;
    TokenId bezier;
    TokenId catmullRom;
    TokenId curveVertexCounts;
    TokenId extent;
    TokenId face;
    TokenId faceVertexCounts;
    TokenId faceVertexIndices;
    TokenId height;
    TokenId holeIndices;
    TokenId ids;
    TokenId inactiveIds;
    TokenId indices;
    TokenId invisible;
    TokenId nonperiodic;
    TokenId orientation;
    TokenId orientations;
    TokenId periodic;
    TokenId pinned;
    TokenId points;
    TokenId positions;
    TokenId protoIndices;
    TokenId prototypes;
    TokenId proxy;
    TokenId radius;
    TokenId scales;
    TokenId size;
    TokenId surfaceFaceVertexIndices;
    TokenId tetVertexIndices;
    TokenId type;
    TokenId velocities;
    TokenId vertex;
    TokenId visibility;
    TokenId wrap;
    TokenId xformOpOrder;

    // Fabric/ovstage transform-change property names, matched against a
    // changed attribute in PrimChangeMap::checkPrimChange's transform special
    // case (xformOp:* itself is a prefix match, done via tokenToString, not a
    // single token here).
    TokenId omniXform;
    TokenId omniResetXformStack;
    TokenId omniFabricLocalMatrix;
    TokenId omniFabricWorldMatrix;

    // OmniUsdPhysicsDeformableSchemaTokens attributes (schema-generated OmniPhysics deformable token table)
    TokenId deformablePose_MultipleApplyTemplate_OmniphysicsPoints;
    TokenId deformablePose_MultipleApplyTemplate_OmniphysicsPurposes;
    TokenId flatDefault;
    TokenId OmniPhysicsDeformablePoseAPI;
    TokenId omniphysicsDensity;
    TokenId omniphysicsDynamicFriction;
    TokenId OmniPhysicsElementCollisionFilter;
    TokenId omniphysicsGroupElemCounts0;
    TokenId omniphysicsGroupElemCounts1;
    TokenId omniphysicsGroupElemIndices0;
    TokenId omniphysicsGroupElemIndices1;
    TokenId omniphysicsLocalPositionsSrc1;
    TokenId omniphysicsPoissonsRatio;
    TokenId omniphysicsRestBendAnglesDefault;
    TokenId omniphysicsRestShapePoints;
    TokenId omniphysicsRestTetVtxIndices;
    TokenId omniphysicsRestTriVtxIndices;
    TokenId omniphysicsStaticFriction;
    TokenId omniphysicsSurfaceBendStiffness;
    TokenId OmniPhysicsSurfaceDeformableSimAPI;
    TokenId omniphysicsSurfaceShearStiffness;
    TokenId omniphysicsSurfaceStretchStiffness;
    TokenId omniphysicsSurfaceThickness;
    TokenId omniphysicsTetCoordsSrc0;
    TokenId omniphysicsTetCoordsSrc1;
    TokenId omniphysicsTetIndicesSrc0;
    TokenId omniphysicsTetIndicesSrc1;
    TokenId OmniPhysicsTetXformAttachment;
    TokenId omniphysicsTriCoordsSrc1;
    TokenId omniphysicsTriIndicesSrc1;
    TokenId OmniPhysicsVolumeDeformableSimAPI;
    TokenId omniphysicsVtxIndicesSrc0;
    TokenId omniphysicsVtxIndicesSrc1;
    TokenId OmniPhysicsVtxTetAttachment;
    TokenId OmniPhysicsVtxTriAttachment;
    TokenId OmniPhysicsVtxVtxAttachment;
    TokenId OmniPhysicsVtxXformAttachment;
    TokenId omniphysicsYoungsModulus;

    // UsdPhysicsTokens attributes (schema-generated UsdPhysics token table)
    TokenId angular;
    TokenId linear;
    TokenId physicsCollisionEnabled;
    TokenId physicsConeAngle0Limit;
    TokenId physicsConeAngle1Limit;
    TokenId physicsLowerLimit;
    TokenId physicsMaxDistance;
    TokenId physicsMinDistance;
    TokenId physicsSimulationOwner;
    TokenId physicsUpperLimit;
    TokenId transX;
    TokenId transY;
    TokenId transZ;
    TokenId x;
    TokenId y;
    TokenId z;

    // PhysxAxisInstanceTokens attributes (schema-generated per-axis PhysxJointAxisAPI/PhysxDrivePerformanceEnvelopeAPI instance token table)
    TokenId armatureAngular;
    TokenId armatureLinear;
    TokenId armatureRotX;
    TokenId armatureRotY;
    TokenId armatureRotZ;
    TokenId dynamicFrictionEffortAngular;
    TokenId dynamicFrictionEffortLinear;
    TokenId dynamicFrictionEffortRotX;
    TokenId dynamicFrictionEffortRotY;
    TokenId dynamicFrictionEffortRotZ;
    TokenId maxActuatorVelocityAngular;
    TokenId maxActuatorVelocityLinear;
    TokenId maxActuatorVelocityRotX;
    TokenId maxActuatorVelocityRotY;
    TokenId maxActuatorVelocityRotZ;
    TokenId maxJointVelocityAngular;
    TokenId maxJointVelocityLinear;
    TokenId maxJointVelocityRotX;
    TokenId maxJointVelocityRotY;
    TokenId maxJointVelocityRotZ;
    TokenId speedEffortGradientAngular;
    TokenId speedEffortGradientLinear;
    TokenId speedEffortGradientRotX;
    TokenId speedEffortGradientRotY;
    TokenId speedEffortGradientRotZ;
    TokenId staticFrictionEffortAngular;
    TokenId staticFrictionEffortLinear;
    TokenId staticFrictionEffortRotX;
    TokenId staticFrictionEffortRotY;
    TokenId staticFrictionEffortRotZ;
    TokenId velocityDependentResistanceAngular;
    TokenId velocityDependentResistanceLinear;
    TokenId velocityDependentResistanceRotX;
    TokenId velocityDependentResistanceRotY;
    TokenId velocityDependentResistanceRotZ;
    TokenId viscousFrictionCoefficientAngular;
    TokenId viscousFrictionCoefficientLinear;
    TokenId viscousFrictionCoefficientRotX;
    TokenId viscousFrictionCoefficientRotY;
    TokenId viscousFrictionCoefficientRotZ;

    // Instance-qualified applied-schema names ("<baseSchema>:<instance>") for
    // PhysxMimicJointAPI / PhysxDrivePerformanceEnvelopeAPI / PhysxJointAxisAPI,
    // matched against the raw apiSchemas metadata list (structural add/remove
    // detection), not read as attributes.
    TokenId physxMimicJointAPIRotX;
    TokenId physxMimicJointAPIRotY;
    TokenId physxMimicJointAPIRotZ;
    TokenId physxDrivePerformanceEnvelopeAPIAngular;
    TokenId physxDrivePerformanceEnvelopeAPILinear;
    TokenId physxDrivePerformanceEnvelopeAPIRotX;
    TokenId physxDrivePerformanceEnvelopeAPIRotY;
    TokenId physxDrivePerformanceEnvelopeAPIRotZ;
    TokenId physxJointAxisAPIAngular;
    TokenId physxJointAxisAPILinear;
    TokenId physxJointAxisAPIRotX;
    TokenId physxJointAxisAPIRotY;
    TokenId physxJointAxisAPIRotZ;
    // Non-D6 joint hot-path gate tokens (see KnownTokens.cpp).
    TokenId physxLimitAPIAngular;
    TokenId physxLimitAPILinear;
    TokenId physxLimitAPICone;
    TokenId physxLimitAPIDistance;
    TokenId physicsJointStateAPIAngular;
    TokenId physicsJointStateAPILinear;
    TokenId physxPhysicsDistanceJointAPI;

    // Instance-qualified UsdPhysicsDriveAPI ("PhysicsDriveAPI:<instance>"), matched
    // the same way as the PhysxJointAxisAPI/PhysxDrivePerformanceEnvelopeAPI fields
    // above; plus the composite "drive:<instance>:physics:maxForce" attribute name
    // each per-axis drive reads. Used by usdInterface/UsdInterface.cpp's
    // changeSchemaAPI/updateDrivePerformanceEnvelope.
    TokenId physicsDriveAPIAngular;
    TokenId physicsDriveAPILinear;
    TokenId physicsDriveAPIRotX;
    TokenId physicsDriveAPIRotY;
    TokenId physicsDriveAPIRotZ;
    TokenId driveMaxForceAngular;
    TokenId driveMaxForceLinear;
    TokenId driveMaxForceRotX;
    TokenId driveMaxForceRotY;
    TokenId driveMaxForceRotZ;

    // NewtonSchemaTokens attributes (schema-generated Newton token table)
    TokenId NewtonMimicAPI;
    TokenId newtonMimicCoef0;
    TokenId newtonMimicCoef1;
    TokenId newtonMimicEnabled;
    TokenId newtonMimicJoint;

    // UsdTokens attributes (schema-generated core Usd token table)
    TokenId apiSchemas;

    // Prim/schema type-name tokens (ADR-0018), for isA()/hasSchema() gates that
    // previously went through PhysXTools.h's schemaTypeToken<T>()/isAType<T>().
    // A "Type" suffix disambiguates from a same-named attribute token above
    // (e.g. points/pointsType); the pre-existing type-name fields
    // (physicsScene, physicsJoint, basisCurvesType) keep their unsuffixed names.
    TokenId meshType;
    TokenId capsuleType;
    TokenId cylinderType;
    TokenId coneType;
    TokenId cubeType;
    TokenId sphereType;
    TokenId gprimType;
    TokenId xformableType;
    TokenId xformType;
    TokenId pointBasedType;
    TokenId pointsType;
    TokenId pointInstancerType;
    TokenId tetMeshType;
    TokenId physicsCollisionGroupType;
    TokenId physicsJointStateAPI;
    TokenId physxParticleSystemType;
    TokenId physxPhysicsJointInstancerType;
    TokenId physxVehicleTireFrictionTableType;

    void intern(const IPhysicsSource& source);
};

// Test-only instrumentation: process-wide count of KnownTokens::intern() calls.
// Cheap relaxed atomic increment (negligible next to intern()'s own ~659
// internToken() calls) -- not gated behind a build macro. Regression tests
// snapshot this before/after a hot-path call to assert it re-interns at most
// once per query/update instead of once per candidate (REQ-SIM-SCENEQUERY-001).
uint64_t internCallCount();

} // namespace omni::physics::parse
