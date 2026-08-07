// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// GENERATED (header-only) by tools/gen_tokens.py from schema.usda via usdGenSchema's
// GatherTokens. DO NOT EDIT. Codeless tokens: PhysxSchemaTokens is defined inline (no lib).
/// \file physxSchema/tokens.h
#ifndef PHYSXSCHEMA_TOKENS_H
#define PHYSXSCHEMA_TOKENS_H

#include <pxr/pxr.h>
#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/token.h>
#include <vector>

PXR_NAMESPACE_OPEN_SCOPE

/// \class PhysxSchemaTokensType
///
/// \link PhysxSchemaTokens \endlink provides static, efficient
/// \link TfToken TfTokens\endlink for use in all public USD API.
///
/// These tokens are auto-generated from the module's schema, representing
/// property names, for when you need to fetch an attribute or relationship
/// directly by name, e.g. UsdPrim::GetAttribute(), in the most efficient
/// manner, and allow the compiler to verify that you spelled the name
/// correctly.
///
/// PhysxSchemaTokens also contains all of the \em allowedTokens values
/// declared for schema builtin attributes of 'token' scene description type.
/// Use PhysxSchemaTokens like so:
///
/// \code
///     gprim.GetMyTokenValuedAttr().Set(PhysxSchemaTokens->acceleration);
/// \endcode
struct PhysxSchemaTokensType {
    PhysxSchemaTokensType();
    /// \brief "acceleration"
    ///
    /// Fallback value for PhysxSchemaPhysxForceAPI::GetModeAttr(), Possible value for PhysxSchemaPhysxVehicleContextAPI::GetUpdateModeAttr()
    const TfToken acceleration;
    /// \brief "alwaysUpdateEnabled"
    ///
    /// PhysxSchemaPhysxCameraAPI
    const TfToken alwaysUpdateEnabled;
    /// \brief "Asynchronous"
    ///
    /// Possible value for PhysxSchemaPhysxSceneAPI::GetUpdateTypeAttr()
    const TfToken Asynchronous;
    /// \brief "average"
    ///
    /// Fallback value for PhysxSchemaPhysxMaterialAPI::GetDampingCombineModeAttr(), Fallback value for PhysxSchemaPhysxMaterialAPI::GetFrictionCombineModeAttr(), Fallback value for PhysxSchemaPhysxMaterialAPI::GetRestitutionCombineModeAttr()
    const TfToken average;
    /// \brief "BitsPerPixel16"
    ///
    /// Fallback value for PhysxSchemaPhysxSDFMeshCollisionAPI::GetSdfBitsPerSubgridPixelAttr()
    const TfToken BitsPerPixel16;
    /// \brief "BitsPerPixel32"
    ///
    /// Possible value for PhysxSchemaPhysxSDFMeshCollisionAPI::GetSdfBitsPerSubgridPixelAttr()
    const TfToken BitsPerPixel32;
    /// \brief "BitsPerPixel8"
    ///
    /// Possible value for PhysxSchemaPhysxSDFMeshCollisionAPI::GetSdfBitsPerSubgridPixelAttr()
    const TfToken BitsPerPixel8;
    /// \brief "brakes0"
    ///
    ///  This token represents braking system 0 used in PhysxVehicleBrakesAPI. It also holds the instance name to use for PhysxVehicleNonlinearCommandResponseAPI when applied to braking system 0.
    const TfToken brakes0;
    /// \brief "brakes1"
    ///
    ///  This token represents braking system 1 used in PhysxVehicleBrakesAPI. It also holds the instance name to use for PhysxVehicleNonlinearCommandResponseAPI when applied to braking system 1.
    const TfToken brakes1;
    /// \brief "collisionmeshes"
    ///
    ///  This token defines the PhysxMeshMergeCollisionAPI collection that gathers the collision meshes.
    const TfToken collisionmeshes;
    /// \brief "constrained"
    ///
    /// Possible value for PhysxSchemaPhysxCharacterControllerAPI::GetClimbingModeAttr()
    const TfToken constrained;
    /// \brief "contactOffset"
    ///
    /// PhysxSchemaPhysxParticleSystem
    const TfToken contactOffset;
    /// \brief "convexDecomposition"
    ///
    ///  This token represents the collection name to use with PhysxCookedDataAPI to represent cooked data of a convexDecomposition.
    const TfToken convexDecomposition;
    /// \brief "convexHull"
    ///
    ///  This token represents the collection name to use with PhysxCookedDataAPI to represent cooked data of a convexhull.
    const TfToken convexHull;
    /// \brief "defaultFrictionValue"
    ///
    /// PhysxSchemaPhysxVehicleTireFrictionTable
    const TfToken defaultFrictionValue;
    /// \brief "Disabled"
    ///
    /// Possible value for PhysxSchemaPhysxSceneAPI::GetUpdateTypeAttr()
    const TfToken Disabled;
    /// \brief "drive"
    ///
    ///  This token holds the instance name to use for PhysxVehicleNonlinearCommandResponseAPI when applying it to a basic drive.
    const TfToken drive;
    /// \brief "easy"
    ///
    /// Fallback value for PhysxSchemaPhysxCharacterControllerAPI::GetClimbingModeAttr()
    const TfToken easy;
    /// \brief "enableCCD"
    ///
    /// PhysxSchemaPhysxParticleSystem
    const TfToken enableCCD;
    /// \brief "flood"
    ///
    /// Fallback value for PhysxSchemaPhysxSphereFillCollisionAPI::GetFillModeAttr()
    const TfToken flood;
    /// \brief "fluidRestOffset"
    ///
    /// PhysxSchemaPhysxParticleSystem
    const TfToken fluidRestOffset;
    /// \brief "force"
    ///
    /// Possible value for PhysxSchemaPhysxForceAPI::GetModeAttr()
    const TfToken force;
    /// \brief "frictionValues"
    ///
    /// PhysxSchemaPhysxVehicleTireFrictionTable
    const TfToken frictionValues;
    /// \brief "globalSelfCollisionEnabled"
    ///
    /// PhysxSchemaPhysxParticleSystem
    const TfToken globalSelfCollisionEnabled;
    /// \brief "GPU"
    ///
    /// Fallback value for PhysxSchemaPhysxSceneAPI::GetBroadphaseTypeAttr()
    const TfToken GPU;
    /// \brief "groundMaterials"
    ///
    /// PhysxSchemaPhysxVehicleTireFrictionTable
    const TfToken groundMaterials;
    /// \brief "max"
    ///
    /// Possible value for PhysxSchemaPhysxMaterialAPI::GetDampingCombineModeAttr(), Possible value for PhysxSchemaPhysxMaterialAPI::GetFrictionCombineModeAttr(), Possible value for PhysxSchemaPhysxMaterialAPI::GetRestitutionCombineModeAttr()
    const TfToken max;
    /// \brief "maxDepenetrationVelocity"
    ///
    /// PhysxSchemaPhysxParticleSystem
    const TfToken maxDepenetrationVelocity;
    /// \brief "maxNeighborhood"
    ///
    /// PhysxSchemaPhysxParticleSystem
    const TfToken maxNeighborhood;
    /// \brief "maxVelocity"
    ///
    /// PhysxSchemaPhysxParticleSystem
    const TfToken maxVelocity;
    /// \brief "MBP"
    ///
    /// Possible value for PhysxSchemaPhysxSceneAPI::GetBroadphaseTypeAttr()
    const TfToken MBP;
    /// \brief "min"
    ///
    /// Possible value for PhysxSchemaPhysxMaterialAPI::GetDampingCombineModeAttr(), Possible value for PhysxSchemaPhysxMaterialAPI::GetFrictionCombineModeAttr(), Possible value for PhysxSchemaPhysxMaterialAPI::GetRestitutionCombineModeAttr()
    const TfToken min;
    /// \brief "multiply"
    ///
    /// Possible value for PhysxSchemaPhysxMaterialAPI::GetDampingCombineModeAttr(), Possible value for PhysxSchemaPhysxMaterialAPI::GetFrictionCombineModeAttr(), Possible value for PhysxSchemaPhysxMaterialAPI::GetRestitutionCombineModeAttr()
    const TfToken multiply;
    /// \brief "negX"
    ///
    /// Possible value for PhysxSchemaPhysxVehicleContextAPI::GetLongitudinalAxisAttr(), Possible value for PhysxSchemaPhysxVehicleContextAPI::GetVerticalAxisAttr()
    const TfToken negX;
    /// \brief "negY"
    ///
    /// Possible value for PhysxSchemaPhysxVehicleContextAPI::GetLongitudinalAxisAttr(), Possible value for PhysxSchemaPhysxVehicleContextAPI::GetVerticalAxisAttr()
    const TfToken negY;
    /// \brief "negZ"
    ///
    /// Possible value for PhysxSchemaPhysxVehicleContextAPI::GetLongitudinalAxisAttr(), Possible value for PhysxSchemaPhysxVehicleContextAPI::GetVerticalAxisAttr()
    const TfToken negZ;
    /// \brief "neighborhoodScale"
    ///
    /// PhysxSchemaPhysxParticleSystem
    const TfToken neighborhoodScale;
    /// \brief "nonParticleCollisionEnabled"
    ///
    /// PhysxSchemaPhysxParticleSystem
    const TfToken nonParticleCollisionEnabled;
    /// \brief "particleContactOffset"
    ///
    /// PhysxSchemaPhysxParticleSystem
    const TfToken particleContactOffset;
    /// \brief "particleSystemEnabled"
    ///
    /// PhysxSchemaPhysxParticleSystem
    const TfToken particleSystemEnabled;
    /// \brief "patch"
    ///
    /// Fallback value for PhysxSchemaPhysxSceneAPI::GetFrictionTypeAttr()
    const TfToken patch;
    /// \brief "PCM"
    ///
    /// Fallback value for PhysxSchemaPhysxSceneAPI::GetCollisionSystemAttr()
    const TfToken PCM;
    /// \brief "PGS"
    ///
    /// Possible value for PhysxSchemaPhysxSceneAPI::GetSolverTypeAttr()
    const TfToken PGS;
    /// \brief "physics:body0Indices"
    ///
    /// PhysxSchemaPhysxPhysicsJointInstancer
    const TfToken physicsBody0Indices;
    /// \brief "physics:body0s"
    ///
    /// PhysxSchemaPhysxPhysicsJointInstancer
    const TfToken physicsBody0s;
    /// \brief "physics:body1Indices"
    ///
    /// PhysxSchemaPhysxPhysicsJointInstancer
    const TfToken physicsBody1Indices;
    /// \brief "physics:body1s"
    ///
    /// PhysxSchemaPhysxPhysicsJointInstancer
    const TfToken physicsBody1s;
    /// \brief "physics:gearRatio"
    ///
    /// PhysxSchemaPhysxPhysicsGearJoint
    const TfToken physicsGearRatio;
    /// \brief "physics:hinge"
    ///
    /// PhysxSchemaPhysxPhysicsRackAndPinionJoint
    const TfToken physicsHinge;
    /// \brief "physics:hinge0"
    ///
    /// PhysxSchemaPhysxPhysicsGearJoint
    const TfToken physicsHinge0;
    /// \brief "physics:hinge1"
    ///
    /// PhysxSchemaPhysxPhysicsGearJoint
    const TfToken physicsHinge1;
    /// \brief "physics:localPos0s"
    ///
    /// PhysxSchemaPhysxPhysicsJointInstancer
    const TfToken physicsLocalPos0s;
    /// \brief "physics:localPos1s"
    ///
    /// PhysxSchemaPhysxPhysicsJointInstancer
    const TfToken physicsLocalPos1s;
    /// \brief "physics:localRot0s"
    ///
    /// PhysxSchemaPhysxPhysicsJointInstancer
    const TfToken physicsLocalRot0s;
    /// \brief "physics:localRot1s"
    ///
    /// PhysxSchemaPhysxPhysicsJointInstancer
    const TfToken physicsLocalRot1s;
    /// \brief "physics:prismatic"
    ///
    /// PhysxSchemaPhysxPhysicsRackAndPinionJoint
    const TfToken physicsPrismatic;
    /// \brief "physics:protoIndices"
    ///
    /// PhysxSchemaPhysxPhysicsInstancer
    const TfToken physicsProtoIndices;
    /// \brief "physics:prototypes"
    ///
    /// PhysxSchemaPhysxPhysicsInstancer
    const TfToken physicsPrototypes;
    /// \brief "physics:ratio"
    ///
    /// PhysxSchemaPhysxPhysicsRackAndPinionJoint
    const TfToken physicsRatio;
    /// \brief "physxArticulation:articulationEnabled"
    ///
    /// PhysxSchemaPhysxArticulationAPI
    const TfToken physxArticulationArticulationEnabled;
    /// \brief "physxArticulation:enabledSelfCollisions"
    ///
    /// PhysxSchemaPhysxArticulationAPI
    const TfToken physxArticulationEnabledSelfCollisions;
    /// \brief "physxArticulation:sleepThreshold"
    ///
    /// PhysxSchemaPhysxArticulationAPI
    const TfToken physxArticulationSleepThreshold;
    /// \brief "physxArticulation:solverPositionIterationCount"
    ///
    /// PhysxSchemaPhysxArticulationAPI
    const TfToken physxArticulationSolverPositionIterationCount;
    /// \brief "physxArticulation:solverVelocityIterationCount"
    ///
    /// PhysxSchemaPhysxArticulationAPI
    const TfToken physxArticulationSolverVelocityIterationCount;
    /// \brief "physxArticulation:stabilizationThreshold"
    ///
    /// PhysxSchemaPhysxArticulationAPI
    const TfToken physxArticulationStabilizationThreshold;
    /// \brief "physxAutoDeformableAttachment:attachable0"
    ///
    /// PhysxSchemaPhysxAutoDeformableAttachmentAPI
    const TfToken physxAutoDeformableAttachmentAttachable0;
    /// \brief "physxAutoDeformableAttachment:attachable1"
    ///
    /// PhysxSchemaPhysxAutoDeformableAttachmentAPI
    const TfToken physxAutoDeformableAttachmentAttachable1;
    /// \brief "physxAutoDeformableAttachment:collisionFilteringOffset"
    ///
    /// PhysxSchemaPhysxAutoDeformableAttachmentAPI
    const TfToken physxAutoDeformableAttachmentCollisionFilteringOffset;
    /// \brief "physxAutoDeformableAttachment:deformableVertexOverlapOffset"
    ///
    /// PhysxSchemaPhysxAutoDeformableAttachmentAPI
    const TfToken physxAutoDeformableAttachmentDeformableVertexOverlapOffset;
    /// \brief "physxAutoDeformableAttachment:enableCollisionFiltering"
    ///
    /// PhysxSchemaPhysxAutoDeformableAttachmentAPI
    const TfToken physxAutoDeformableAttachmentEnableCollisionFiltering;
    /// \brief "physxAutoDeformableAttachment:enableDeformableFilteringPairs"
    ///
    /// PhysxSchemaPhysxAutoDeformableAttachmentAPI
    const TfToken physxAutoDeformableAttachmentEnableDeformableFilteringPairs;
    /// \brief "physxAutoDeformableAttachment:enableDeformableVertexAttachments"
    ///
    /// PhysxSchemaPhysxAutoDeformableAttachmentAPI
    const TfToken physxAutoDeformableAttachmentEnableDeformableVertexAttachments;
    /// \brief "physxAutoDeformableAttachment:enableRigidSurfaceAttachments"
    ///
    /// PhysxSchemaPhysxAutoDeformableAttachmentAPI
    const TfToken physxAutoDeformableAttachmentEnableRigidSurfaceAttachments;
    /// \brief "physxAutoDeformableAttachment:maskShapes"
    ///
    /// PhysxSchemaPhysxAutoDeformableAttachmentAPI
    const TfToken physxAutoDeformableAttachmentMaskShapes;
    /// \brief "physxAutoDeformableAttachment:rigidSurfaceSamplingDistance"
    ///
    /// PhysxSchemaPhysxAutoDeformableAttachmentAPI
    const TfToken physxAutoDeformableAttachmentRigidSurfaceSamplingDistance;
    /// \brief "physxCamera:subject"
    ///
    /// PhysxSchemaPhysxCameraAPI
    const TfToken physxCameraSubject;
    /// \brief "physxCharacterController:climbingMode"
    ///
    /// PhysxSchemaPhysxCharacterControllerAPI
    const TfToken physxCharacterControllerClimbingMode;
    /// \brief "physxCharacterController:contactOffset"
    ///
    /// PhysxSchemaPhysxCharacterControllerAPI
    const TfToken physxCharacterControllerContactOffset;
    /// \brief "physxCharacterController:invisibleWallHeight"
    ///
    /// PhysxSchemaPhysxCharacterControllerAPI
    const TfToken physxCharacterControllerInvisibleWallHeight;
    /// \brief "physxCharacterController:maxJumpHeight"
    ///
    /// PhysxSchemaPhysxCharacterControllerAPI
    const TfToken physxCharacterControllerMaxJumpHeight;
    /// \brief "physxCharacterController:moveTarget"
    ///
    /// PhysxSchemaPhysxCharacterControllerAPI
    const TfToken physxCharacterControllerMoveTarget;
    /// \brief "physxCharacterController:nonWalkableMode"
    ///
    /// PhysxSchemaPhysxCharacterControllerAPI
    const TfToken physxCharacterControllerNonWalkableMode;
    /// \brief "physxCharacterController:scaleCoeff"
    ///
    /// PhysxSchemaPhysxCharacterControllerAPI
    const TfToken physxCharacterControllerScaleCoeff;
    /// \brief "physxCharacterController:simulationOwner"
    ///
    /// PhysxSchemaPhysxCharacterControllerAPI
    const TfToken physxCharacterControllerSimulationOwner;
    /// \brief "physxCharacterController:slopeLimit"
    ///
    /// PhysxSchemaPhysxCharacterControllerAPI
    const TfToken physxCharacterControllerSlopeLimit;
    /// \brief "physxCharacterController:stepOffset"
    ///
    /// PhysxSchemaPhysxCharacterControllerAPI
    const TfToken physxCharacterControllerStepOffset;
    /// \brief "physxCharacterController:upAxis"
    ///
    /// PhysxSchemaPhysxCharacterControllerAPI
    const TfToken physxCharacterControllerUpAxis;
    /// \brief "physxCharacterController:volumeGrowth"
    ///
    /// PhysxSchemaPhysxCharacterControllerAPI
    const TfToken physxCharacterControllerVolumeGrowth;
    /// \brief "physxCollision:contactOffset"
    ///
    /// PhysxSchemaPhysxCollisionAPI
    const TfToken physxCollisionContactOffset;
    /// \brief "physxCollisionCustomGeometry"
    ///
    ///  This token represents the custom geometry option for cones and cylinders. The simulation will not use a convex approximation but a custom geometry instead.
    const TfToken physxCollisionCustomGeometry;
    /// \brief "physxCollision:minTorsionalPatchRadius"
    ///
    /// PhysxSchemaPhysxCollisionAPI
    const TfToken physxCollisionMinTorsionalPatchRadius;
    /// \brief "physxCollision:restOffset"
    ///
    /// PhysxSchemaPhysxCollisionAPI
    const TfToken physxCollisionRestOffset;
    /// \brief "physxCollision:torsionalPatchRadius"
    ///
    /// PhysxSchemaPhysxCollisionAPI
    const TfToken physxCollisionTorsionalPatchRadius;
    /// \brief "physxContactReport:reportPairs"
    ///
    /// PhysxSchemaPhysxContactReportAPI
    const TfToken physxContactReportReportPairs;
    /// \brief "physxContactReport:threshold"
    ///
    /// PhysxSchemaPhysxContactReportAPI
    const TfToken physxContactReportThreshold;
    /// \brief "physxConvexDecompositionCollision:errorPercentage"
    ///
    /// PhysxSchemaPhysxConvexDecompositionCollisionAPI
    const TfToken physxConvexDecompositionCollisionErrorPercentage;
    /// \brief "physxConvexDecompositionCollision:hullVertexLimit"
    ///
    /// PhysxSchemaPhysxConvexDecompositionCollisionAPI
    const TfToken physxConvexDecompositionCollisionHullVertexLimit;
    /// \brief "physxConvexDecompositionCollision:maxConvexHulls"
    ///
    /// PhysxSchemaPhysxConvexDecompositionCollisionAPI
    const TfToken physxConvexDecompositionCollisionMaxConvexHulls;
    /// \brief "physxConvexDecompositionCollision:minThickness"
    ///
    /// PhysxSchemaPhysxConvexDecompositionCollisionAPI
    const TfToken physxConvexDecompositionCollisionMinThickness;
    /// \brief "physxConvexDecompositionCollision:shrinkWrap"
    ///
    /// PhysxSchemaPhysxConvexDecompositionCollisionAPI
    const TfToken physxConvexDecompositionCollisionShrinkWrap;
    /// \brief "physxConvexDecompositionCollision:voxelResolution"
    ///
    /// PhysxSchemaPhysxConvexDecompositionCollisionAPI
    const TfToken physxConvexDecompositionCollisionVoxelResolution;
    /// \brief "physxConvexHullCollision:hullVertexLimit"
    ///
    /// PhysxSchemaPhysxConvexHullCollisionAPI
    const TfToken physxConvexHullCollisionHullVertexLimit;
    /// \brief "physxConvexHullCollision:minThickness"
    ///
    /// PhysxSchemaPhysxConvexHullCollisionAPI
    const TfToken physxConvexHullCollisionMinThickness;
    /// \brief "physxCookedData"
    ///
    /// Property namespace prefix for the PhysxSchemaPhysxCookedDataAPI schema.
    const TfToken physxCookedData;
    /// \brief "physxCookedData:__INSTANCE_NAME__:buffer"
    ///
    /// PhysxSchemaPhysxCookedDataAPI
    const TfToken physxCookedData_MultipleApplyTemplate_Buffer;
    /// \brief "physxDeformableBody:autoDeformableBodyEnabled"
    ///
    /// PhysxSchemaPhysxAutoDeformableBodyAPI
    const TfToken physxDeformableBodyAutoDeformableBodyEnabled;
    /// \brief "physxDeformableBody:autoDeformableMeshSimplificationEnabled"
    ///
    /// PhysxSchemaPhysxAutoDeformableMeshSimplificationAPI
    const TfToken physxDeformableBodyAutoDeformableMeshSimplificationEnabled;
    /// \brief "physxDeformableBody:collisionIterationMultiplier"
    ///
    /// PhysxSchemaPhysxSurfaceDeformableBodyAPI
    const TfToken physxDeformableBodyCollisionIterationMultiplier;
    /// \brief "physxDeformableBody:collisionPairUpdateFrequency"
    ///
    /// PhysxSchemaPhysxSurfaceDeformableBodyAPI
    const TfToken physxDeformableBodyCollisionPairUpdateFrequency;
    /// \brief "physxDeformableBody:cookingSourceMesh"
    ///
    /// PhysxSchemaPhysxAutoDeformableBodyAPI
    const TfToken physxDeformableBodyCookingSourceMesh;
    /// \brief "physxDeformableBody:disableGravity"
    ///
    /// PhysxSchemaPhysxBaseDeformableBodyAPI
    const TfToken physxDeformableBodyDisableGravity;
    /// \brief "physxDeformableBody:enableSpeculativeCCD"
    ///
    /// PhysxSchemaPhysxBaseDeformableBodyAPI
    const TfToken physxDeformableBodyEnableSpeculativeCCD;
    /// \brief "physxDeformableBody:forceConforming"
    ///
    /// PhysxSchemaPhysxAutoDeformableMeshSimplificationAPI
    const TfToken physxDeformableBodyForceConforming;
    /// \brief "physxDeformableBody:linearDamping"
    ///
    /// PhysxSchemaPhysxBaseDeformableBodyAPI
    const TfToken physxDeformableBodyLinearDamping;
    /// \brief "physxDeformableBody:maxDepenetrationVelocity"
    ///
    /// PhysxSchemaPhysxBaseDeformableBodyAPI
    const TfToken physxDeformableBodyMaxDepenetrationVelocity;
    /// \brief "physxDeformableBody:maxLinearVelocity"
    ///
    /// PhysxSchemaPhysxBaseDeformableBodyAPI
    const TfToken physxDeformableBodyMaxLinearVelocity;
    /// \brief "physxDeformableBody:remeshingEnabled"
    ///
    /// PhysxSchemaPhysxAutoDeformableMeshSimplificationAPI
    const TfToken physxDeformableBodyRemeshingEnabled;
    /// \brief "physxDeformableBody:remeshingResolution"
    ///
    /// PhysxSchemaPhysxAutoDeformableMeshSimplificationAPI
    const TfToken physxDeformableBodyRemeshingResolution;
    /// \brief "physxDeformableBody:resolution"
    ///
    /// PhysxSchemaPhysxAutoDeformableHexahedralMeshAPI
    const TfToken physxDeformableBodyResolution;
    /// \brief "physxDeformableBody:selfCollision"
    ///
    /// PhysxSchemaPhysxBaseDeformableBodyAPI
    const TfToken physxDeformableBodySelfCollision;
    /// \brief "physxDeformableBody:selfCollisionFilterDistance"
    ///
    /// PhysxSchemaPhysxBaseDeformableBodyAPI
    const TfToken physxDeformableBodySelfCollisionFilterDistance;
    /// \brief "physxDeformableBody:settlingDamping"
    ///
    /// PhysxSchemaPhysxBaseDeformableBodyAPI
    const TfToken physxDeformableBodySettlingDamping;
    /// \brief "physxDeformableBody:settlingThreshold"
    ///
    /// PhysxSchemaPhysxBaseDeformableBodyAPI
    const TfToken physxDeformableBodySettlingThreshold;
    /// \brief "physxDeformableBody:sleepThreshold"
    ///
    /// PhysxSchemaPhysxBaseDeformableBodyAPI
    const TfToken physxDeformableBodySleepThreshold;
    /// \brief "physxDeformableBody:solverPositionIterationCount"
    ///
    /// PhysxSchemaPhysxBaseDeformableBodyAPI
    const TfToken physxDeformableBodySolverPositionIterationCount;
    /// \brief "physxDeformableBody:targetTriangleCount"
    ///
    /// PhysxSchemaPhysxAutoDeformableMeshSimplificationAPI
    const TfToken physxDeformableBodyTargetTriangleCount;
    /// \brief "physxDeformableMaterial:bendDamping"
    ///
    /// PhysxSchemaPhysxSurfaceDeformableMaterialAPI
    const TfToken physxDeformableMaterialBendDamping;
    /// \brief "physxDeformableMaterial:elasticityDamping"
    ///
    /// PhysxSchemaPhysxDeformableMaterialAPI
    const TfToken physxDeformableMaterialElasticityDamping;
    /// \brief "physxDiffuseParticles:airDrag"
    ///
    /// PhysxSchemaPhysxDiffuseParticlesAPI
    const TfToken physxDiffuseParticlesAirDrag;
    /// \brief "physxDiffuseParticles:bubbleDrag"
    ///
    /// PhysxSchemaPhysxDiffuseParticlesAPI
    const TfToken physxDiffuseParticlesBubbleDrag;
    /// \brief "physxDiffuseParticles:buoyancy"
    ///
    /// PhysxSchemaPhysxDiffuseParticlesAPI
    const TfToken physxDiffuseParticlesBuoyancy;
    /// \brief "physxDiffuseParticles:collisionDecay"
    ///
    /// PhysxSchemaPhysxDiffuseParticlesAPI
    const TfToken physxDiffuseParticlesCollisionDecay;
    /// \brief "physxDiffuseParticles:diffuseParticlesEnabled"
    ///
    /// PhysxSchemaPhysxDiffuseParticlesAPI
    const TfToken physxDiffuseParticlesDiffuseParticlesEnabled;
    /// \brief "physxDiffuseParticles:divergenceWeight"
    ///
    /// PhysxSchemaPhysxDiffuseParticlesAPI
    const TfToken physxDiffuseParticlesDivergenceWeight;
    /// \brief "physxDiffuseParticles:kineticEnergyWeight"
    ///
    /// PhysxSchemaPhysxDiffuseParticlesAPI
    const TfToken physxDiffuseParticlesKineticEnergyWeight;
    /// \brief "physxDiffuseParticles:lifetime"
    ///
    /// PhysxSchemaPhysxDiffuseParticlesAPI
    const TfToken physxDiffuseParticlesLifetime;
    /// \brief "physxDiffuseParticles:maxDiffuseParticleMultiplier"
    ///
    /// PhysxSchemaPhysxDiffuseParticlesAPI
    const TfToken physxDiffuseParticlesMaxDiffuseParticleMultiplier;
    /// \brief "physxDiffuseParticles:pressureWeight"
    ///
    /// PhysxSchemaPhysxDiffuseParticlesAPI
    const TfToken physxDiffuseParticlesPressureWeight;
    /// \brief "physxDiffuseParticles:threshold"
    ///
    /// PhysxSchemaPhysxDiffuseParticlesAPI
    const TfToken physxDiffuseParticlesThreshold;
    /// \brief "physxDiffuseParticles:useAccurateVelocity"
    ///
    /// PhysxSchemaPhysxDiffuseParticlesAPI
    const TfToken physxDiffuseParticlesUseAccurateVelocity;
    /// \brief "physxDrivePerformanceEnvelope"
    ///
    /// Property namespace prefix for the PhysxSchemaPhysxDrivePerformanceEnvelopeAPI schema.
    const TfToken physxDrivePerformanceEnvelope;
    /// \brief "physxDrivePerformanceEnvelope:__INSTANCE_NAME__:maxActuatorVelocity"
    ///
    /// PhysxSchemaPhysxDrivePerformanceEnvelopeAPI
    const TfToken physxDrivePerformanceEnvelope_MultipleApplyTemplate_MaxActuatorVelocity;
    /// \brief "physxDrivePerformanceEnvelope:__INSTANCE_NAME__:speedEffortGradient"
    ///
    /// PhysxSchemaPhysxDrivePerformanceEnvelopeAPI
    const TfToken physxDrivePerformanceEnvelope_MultipleApplyTemplate_SpeedEffortGradient;
    /// \brief "physxDrivePerformanceEnvelope:__INSTANCE_NAME__:velocityDependentResistance"
    ///
    /// PhysxSchemaPhysxDrivePerformanceEnvelopeAPI
    const TfToken physxDrivePerformanceEnvelope_MultipleApplyTemplate_VelocityDependentResistance;
    /// \brief "physxDroneCamera:feedForwardVelocityGain"
    ///
    /// PhysxSchemaPhysxCameraDroneAPI
    const TfToken physxDroneCameraFeedForwardVelocityGain;
    /// \brief "physxDroneCamera:followDistance"
    ///
    /// PhysxSchemaPhysxCameraDroneAPI
    const TfToken physxDroneCameraFollowDistance;
    /// \brief "physxDroneCamera:followHeight"
    ///
    /// PhysxSchemaPhysxCameraDroneAPI
    const TfToken physxDroneCameraFollowHeight;
    /// \brief "physxDroneCamera:horizontalVelocityGain"
    ///
    /// PhysxSchemaPhysxCameraDroneAPI
    const TfToken physxDroneCameraHorizontalVelocityGain;
    /// \brief "physxDroneCamera:maxDistance"
    ///
    /// PhysxSchemaPhysxCameraDroneAPI
    const TfToken physxDroneCameraMaxDistance;
    /// \brief "physxDroneCamera:maxSpeed"
    ///
    /// PhysxSchemaPhysxCameraDroneAPI
    const TfToken physxDroneCameraMaxSpeed;
    /// \brief "physxDroneCamera:positionOffset"
    ///
    /// PhysxSchemaPhysxCameraDroneAPI
    const TfToken physxDroneCameraPositionOffset;
    /// \brief "physxDroneCamera:rotationFilterTimeConstant"
    ///
    /// PhysxSchemaPhysxCameraDroneAPI
    const TfToken physxDroneCameraRotationFilterTimeConstant;
    /// \brief "physxDroneCamera:velocityFilterTimeConstant"
    ///
    /// PhysxSchemaPhysxCameraDroneAPI
    const TfToken physxDroneCameraVelocityFilterTimeConstant;
    /// \brief "physxDroneCamera:verticalVelocityGain"
    ///
    /// PhysxSchemaPhysxCameraDroneAPI
    const TfToken physxDroneCameraVerticalVelocityGain;
    /// \brief "physxFollowCamera:cameraPositionTimeConstant"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowCameraCameraPositionTimeConstant;
    /// \brief "physxFollowCamera:followMaxDistance"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowCameraFollowMaxDistance;
    /// \brief "physxFollowCamera:followMaxSpeed"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowCameraFollowMaxSpeed;
    /// \brief "physxFollowCamera:followMinDistance"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowCameraFollowMinDistance;
    /// \brief "physxFollowCamera:followMinSpeed"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowCameraFollowMinSpeed;
    /// \brief "physxFollowCamera:followTurnRateGain"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowCameraFollowTurnRateGain;
    /// \brief "physxFollowCamera:lookAheadMaxSpeed"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowCameraLookAheadMaxSpeed;
    /// \brief "physxFollowCamera:lookAheadMinDistance"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowCameraLookAheadMinDistance;
    /// \brief "physxFollowCamera:lookAheadMinSpeed"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowCameraLookAheadMinSpeed;
    /// \brief "physxFollowCamera:lookAheadTurnRateGain"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowCameraLookAheadTurnRateGain;
    /// \brief "physxFollowCamera:lookPositionHeight"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowCameraLookPositionHeight;
    /// \brief "physxFollowCamera:lookPositionTimeConstant"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowCameraLookPositionTimeConstant;
    /// \brief "physxFollowCamera:pitchAngle"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowCameraPitchAngle;
    /// \brief "physxFollowCamera:pitchAngleTimeConstant"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowCameraPitchAngleTimeConstant;
    /// \brief "physxFollowCamera:positionOffset"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowCameraPositionOffset;
    /// \brief "physxFollowCamera:slowPitchAngleSpeed"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowCameraSlowPitchAngleSpeed;
    /// \brief "physxFollowCamera:slowSpeedPitchAngleScale"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowCameraSlowSpeedPitchAngleScale;
    /// \brief "physxFollowCamera:velocityNormalMinSpeed"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowCameraVelocityNormalMinSpeed;
    /// \brief "physxFollowCamera:yawAngle"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowCameraYawAngle;
    /// \brief "physxFollowCamera:yawRateTimeConstant"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowCameraYawRateTimeConstant;
    /// \brief "physxFollowFollowCamera:lookAheadMaxDistance"
    ///
    /// PhysxSchemaPhysxCameraFollowAPI
    const TfToken physxFollowFollowCameraLookAheadMaxDistance;
    /// \brief "physxFollowLookCamera:downHillGroundAngle"
    ///
    /// PhysxSchemaPhysxCameraFollowLookAPI
    const TfToken physxFollowLookCameraDownHillGroundAngle;
    /// \brief "physxFollowLookCamera:downHillGroundPitch"
    ///
    /// PhysxSchemaPhysxCameraFollowLookAPI
    const TfToken physxFollowLookCameraDownHillGroundPitch;
    /// \brief "physxFollowLookCamera:followReverseDistance"
    ///
    /// PhysxSchemaPhysxCameraFollowLookAPI
    const TfToken physxFollowLookCameraFollowReverseDistance;
    /// \brief "physxFollowLookCamera:followReverseSpeed"
    ///
    /// PhysxSchemaPhysxCameraFollowLookAPI
    const TfToken physxFollowLookCameraFollowReverseSpeed;
    /// \brief "physxFollowLookCamera:upHillGroundAngle"
    ///
    /// PhysxSchemaPhysxCameraFollowLookAPI
    const TfToken physxFollowLookCameraUpHillGroundAngle;
    /// \brief "physxFollowLookCamera:upHillGroundPitch"
    ///
    /// PhysxSchemaPhysxCameraFollowLookAPI
    const TfToken physxFollowLookCameraUpHillGroundPitch;
    /// \brief "physxFollowLookCamera:velocityBlendTimeConstant"
    ///
    /// PhysxSchemaPhysxCameraFollowLookAPI
    const TfToken physxFollowLookCameraVelocityBlendTimeConstant;
    /// \brief "physxForce:force"
    ///
    /// PhysxSchemaPhysxForceAPI
    const TfToken physxForceForce;
    /// \brief "physxForce:forceEnabled"
    ///
    /// PhysxSchemaPhysxForceAPI
    const TfToken physxForceForceEnabled;
    /// \brief "physxForce:mode"
    ///
    /// PhysxSchemaPhysxForceAPI
    const TfToken physxForceMode;
    /// \brief "physxForce:torque"
    ///
    /// PhysxSchemaPhysxForceAPI
    const TfToken physxForceTorque;
    /// \brief "physxForce:worldFrameEnabled"
    ///
    /// PhysxSchemaPhysxForceAPI
    const TfToken physxForceWorldFrameEnabled;
    /// \brief "physxJoint:armature"
    ///
    /// PhysxSchemaPhysxJointAPI
    const TfToken physxJointArmature;
    /// \brief "physxJointAxis"
    ///
    /// Property namespace prefix for the PhysxSchemaPhysxJointAxisAPI schema.
    const TfToken physxJointAxis;
    /// \brief "physxJointAxis:__INSTANCE_NAME__:armature"
    ///
    /// PhysxSchemaPhysxJointAxisAPI
    const TfToken physxJointAxis_MultipleApplyTemplate_Armature;
    /// \brief "physxJointAxis:__INSTANCE_NAME__:dynamicFrictionEffort"
    ///
    /// PhysxSchemaPhysxJointAxisAPI
    const TfToken physxJointAxis_MultipleApplyTemplate_DynamicFrictionEffort;
    /// \brief "physxJointAxis:__INSTANCE_NAME__:maxJointVelocity"
    ///
    /// PhysxSchemaPhysxJointAxisAPI
    const TfToken physxJointAxis_MultipleApplyTemplate_MaxJointVelocity;
    /// \brief "physxJointAxis:__INSTANCE_NAME__:staticFrictionEffort"
    ///
    /// PhysxSchemaPhysxJointAxisAPI
    const TfToken physxJointAxis_MultipleApplyTemplate_StaticFrictionEffort;
    /// \brief "physxJointAxis:__INSTANCE_NAME__:viscousFrictionCoefficient"
    ///
    /// PhysxSchemaPhysxJointAxisAPI
    const TfToken physxJointAxis_MultipleApplyTemplate_ViscousFrictionCoefficient;
    /// \brief "physxJoint:jointFriction"
    ///
    /// PhysxSchemaPhysxJointAPI
    const TfToken physxJointJointFriction;
    /// \brief "physxJoint:maxJointVelocity"
    ///
    /// PhysxSchemaPhysxJointAPI
    const TfToken physxJointMaxJointVelocity;
    /// \brief "physxLimit"
    ///
    /// Property namespace prefix for the PhysxSchemaPhysxLimitAPI schema.
    const TfToken physxLimit;
    /// \brief "physxLimit:__INSTANCE_NAME__:bounceThreshold"
    ///
    /// PhysxSchemaPhysxLimitAPI
    const TfToken physxLimit_MultipleApplyTemplate_BounceThreshold;
    /// \brief "physxLimit:__INSTANCE_NAME__:damping"
    ///
    /// PhysxSchemaPhysxLimitAPI
    const TfToken physxLimit_MultipleApplyTemplate_Damping;
    /// \brief "physxLimit:__INSTANCE_NAME__:restitution"
    ///
    /// PhysxSchemaPhysxLimitAPI
    const TfToken physxLimit_MultipleApplyTemplate_Restitution;
    /// \brief "physxLimit:__INSTANCE_NAME__:stiffness"
    ///
    /// PhysxSchemaPhysxLimitAPI
    const TfToken physxLimit_MultipleApplyTemplate_Stiffness;
    /// \brief "physxMaterial:compliantContactAccelerationSpring"
    ///
    /// PhysxSchemaPhysxMaterialAPI
    const TfToken physxMaterialCompliantContactAccelerationSpring;
    /// \brief "physxMaterial:compliantContactDamping"
    ///
    /// PhysxSchemaPhysxMaterialAPI
    const TfToken physxMaterialCompliantContactDamping;
    /// \brief "physxMaterial:compliantContactStiffness"
    ///
    /// PhysxSchemaPhysxMaterialAPI
    const TfToken physxMaterialCompliantContactStiffness;
    /// \brief "physxMaterial:dampingCombineMode"
    ///
    /// PhysxSchemaPhysxMaterialAPI
    const TfToken physxMaterialDampingCombineMode;
    /// \brief "physxMaterial:frictionCombineMode"
    ///
    /// PhysxSchemaPhysxMaterialAPI
    const TfToken physxMaterialFrictionCombineMode;
    /// \brief "physxMaterial:restitutionCombineMode"
    ///
    /// PhysxSchemaPhysxMaterialAPI
    const TfToken physxMaterialRestitutionCombineMode;
    /// \brief "physxMimicJoint"
    ///
    /// Property namespace prefix for the PhysxSchemaPhysxMimicJointAPI schema.
    const TfToken physxMimicJoint;
    /// \brief "physxMimicJoint:__INSTANCE_NAME__:dampingRatio"
    ///
    /// PhysxSchemaPhysxMimicJointAPI
    const TfToken physxMimicJoint_MultipleApplyTemplate_DampingRatio;
    /// \brief "physxMimicJoint:__INSTANCE_NAME__:gearing"
    ///
    /// PhysxSchemaPhysxMimicJointAPI
    const TfToken physxMimicJoint_MultipleApplyTemplate_Gearing;
    /// \brief "physxMimicJoint:__INSTANCE_NAME__:naturalFrequency"
    ///
    /// PhysxSchemaPhysxMimicJointAPI
    const TfToken physxMimicJoint_MultipleApplyTemplate_NaturalFrequency;
    /// \brief "physxMimicJoint:__INSTANCE_NAME__:offset"
    ///
    /// PhysxSchemaPhysxMimicJointAPI
    const TfToken physxMimicJoint_MultipleApplyTemplate_Offset;
    /// \brief "physxMimicJoint:__INSTANCE_NAME__:referenceJoint"
    ///
    /// PhysxSchemaPhysxMimicJointAPI
    const TfToken physxMimicJoint_MultipleApplyTemplate_ReferenceJoint;
    /// \brief "physxMimicJoint:__INSTANCE_NAME__:referenceJointAxis"
    ///
    /// PhysxSchemaPhysxMimicJointAPI
    const TfToken physxMimicJoint_MultipleApplyTemplate_ReferenceJointAxis;
    /// \brief "physxParticleAnisotropy:max"
    ///
    /// PhysxSchemaPhysxParticleAnisotropyAPI
    const TfToken physxParticleAnisotropyMax;
    /// \brief "physxParticleAnisotropy:min"
    ///
    /// PhysxSchemaPhysxParticleAnisotropyAPI
    const TfToken physxParticleAnisotropyMin;
    /// \brief "physxParticleAnisotropy:particleAnisotropyEnabled"
    ///
    /// PhysxSchemaPhysxParticleAnisotropyAPI
    const TfToken physxParticleAnisotropyParticleAnisotropyEnabled;
    /// \brief "physxParticleAnisotropy:scale"
    ///
    /// PhysxSchemaPhysxParticleAnisotropyAPI
    const TfToken physxParticleAnisotropyScale;
    /// \brief "physxParticle:fluid"
    ///
    /// PhysxSchemaPhysxParticleSetAPI
    const TfToken physxParticleFluid;
    /// \brief "physxParticleIsosurface:gridFilteringPasses"
    ///
    /// PhysxSchemaPhysxParticleIsosurfaceAPI
    const TfToken physxParticleIsosurfaceGridFilteringPasses;
    /// \brief "physxParticleIsosurface:gridSmoothingRadius"
    ///
    /// PhysxSchemaPhysxParticleIsosurfaceAPI
    const TfToken physxParticleIsosurfaceGridSmoothingRadius;
    /// \brief "physxParticleIsosurface:gridSpacing"
    ///
    /// PhysxSchemaPhysxParticleIsosurfaceAPI
    const TfToken physxParticleIsosurfaceGridSpacing;
    /// \brief "physxParticleIsosurface:isosurfaceEnabled"
    ///
    /// PhysxSchemaPhysxParticleIsosurfaceAPI
    const TfToken physxParticleIsosurfaceIsosurfaceEnabled;
    /// \brief "physxParticleIsosurface:maxSubgrids"
    ///
    /// PhysxSchemaPhysxParticleIsosurfaceAPI
    const TfToken physxParticleIsosurfaceMaxSubgrids;
    /// \brief "physxParticleIsosurface:maxTriangles"
    ///
    /// PhysxSchemaPhysxParticleIsosurfaceAPI
    const TfToken physxParticleIsosurfaceMaxTriangles;
    /// \brief "physxParticleIsosurface:maxVertices"
    ///
    /// PhysxSchemaPhysxParticleIsosurfaceAPI
    const TfToken physxParticleIsosurfaceMaxVertices;
    /// \brief "physxParticleIsosurface:numMeshNormalSmoothingPasses"
    ///
    /// PhysxSchemaPhysxParticleIsosurfaceAPI
    const TfToken physxParticleIsosurfaceNumMeshNormalSmoothingPasses;
    /// \brief "physxParticleIsosurface:numMeshSmoothingPasses"
    ///
    /// PhysxSchemaPhysxParticleIsosurfaceAPI
    const TfToken physxParticleIsosurfaceNumMeshSmoothingPasses;
    /// \brief "physxParticleIsosurface:surfaceDistance"
    ///
    /// PhysxSchemaPhysxParticleIsosurfaceAPI
    const TfToken physxParticleIsosurfaceSurfaceDistance;
    /// \brief "physxParticle:particleEnabled"
    ///
    /// PhysxSchemaPhysxParticleAPI
    const TfToken physxParticleParticleEnabled;
    /// \brief "physxParticle:particleGroup"
    ///
    /// PhysxSchemaPhysxParticleAPI
    const TfToken physxParticleParticleGroup;
    /// \brief "physxParticle:particleSystem"
    ///
    /// PhysxSchemaPhysxParticleAPI
    const TfToken physxParticleParticleSystem;
    /// \brief "physxParticleSampling:maxSamples"
    ///
    /// PhysxSchemaPhysxParticleSamplingAPI
    const TfToken physxParticleSamplingMaxSamples;
    /// \brief "physxParticleSampling:particles"
    ///
    /// PhysxSchemaPhysxParticleSamplingAPI
    const TfToken physxParticleSamplingParticles;
    /// \brief "physxParticleSampling:samplingDistance"
    ///
    /// PhysxSchemaPhysxParticleSamplingAPI
    const TfToken physxParticleSamplingSamplingDistance;
    /// \brief "physxParticleSampling:volume"
    ///
    /// PhysxSchemaPhysxParticleSamplingAPI
    const TfToken physxParticleSamplingVolume;
    /// \brief "physxParticle:selfCollision"
    ///
    /// PhysxSchemaPhysxParticleAPI
    const TfToken physxParticleSelfCollision;
    /// \brief "physxParticle:simulationPoints"
    ///
    /// PhysxSchemaPhysxParticleSetAPI
    const TfToken physxParticleSimulationPoints;
    /// \brief "physxParticleSmoothing:particleSmoothingEnabled"
    ///
    /// PhysxSchemaPhysxParticleSmoothingAPI
    const TfToken physxParticleSmoothingParticleSmoothingEnabled;
    /// \brief "physxParticleSmoothing:strength"
    ///
    /// PhysxSchemaPhysxParticleSmoothingAPI
    const TfToken physxParticleSmoothingStrength;
    /// \brief "physxPBDMaterial:adhesion"
    ///
    /// PhysxSchemaPhysxPBDMaterialAPI
    const TfToken physxPBDMaterialAdhesion;
    /// \brief "physxPBDMaterial:adhesionOffsetScale"
    ///
    /// PhysxSchemaPhysxPBDMaterialAPI
    const TfToken physxPBDMaterialAdhesionOffsetScale;
    /// \brief "physxPBDMaterial:cflCoefficient"
    ///
    /// PhysxSchemaPhysxPBDMaterialAPI
    const TfToken physxPBDMaterialCflCoefficient;
    /// \brief "physxPBDMaterial:cohesion"
    ///
    /// PhysxSchemaPhysxPBDMaterialAPI
    const TfToken physxPBDMaterialCohesion;
    /// \brief "physxPBDMaterial:damping"
    ///
    /// PhysxSchemaPhysxPBDMaterialAPI
    const TfToken physxPBDMaterialDamping;
    /// \brief "physxPBDMaterial:density"
    ///
    /// PhysxSchemaPhysxPBDMaterialAPI
    const TfToken physxPBDMaterialDensity;
    /// \brief "physxPBDMaterial:friction"
    ///
    /// PhysxSchemaPhysxPBDMaterialAPI
    const TfToken physxPBDMaterialFriction;
    /// \brief "physxPBDMaterial:gravityScale"
    ///
    /// PhysxSchemaPhysxPBDMaterialAPI
    const TfToken physxPBDMaterialGravityScale;
    /// \brief "physxPBDMaterial:particleAdhesionScale"
    ///
    /// PhysxSchemaPhysxPBDMaterialAPI
    const TfToken physxPBDMaterialParticleAdhesionScale;
    /// \brief "physxPBDMaterial:particleFrictionScale"
    ///
    /// PhysxSchemaPhysxPBDMaterialAPI
    const TfToken physxPBDMaterialParticleFrictionScale;
    /// \brief "physxPBDMaterial:surfaceTension"
    ///
    /// PhysxSchemaPhysxPBDMaterialAPI
    const TfToken physxPBDMaterialSurfaceTension;
    /// \brief "physxPBDMaterial:viscosity"
    ///
    /// PhysxSchemaPhysxPBDMaterialAPI
    const TfToken physxPBDMaterialViscosity;
    /// \brief "physxPBDMaterial:vorticityConfinement"
    ///
    /// PhysxSchemaPhysxPBDMaterialAPI
    const TfToken physxPBDMaterialVorticityConfinement;
    /// \brief "physxPhysicsDistanceJoint:springDamping"
    ///
    /// PhysxSchemaPhysxPhysicsDistanceJointAPI
    const TfToken physxPhysicsDistanceJointSpringDamping;
    /// \brief "physxPhysicsDistanceJoint:springEnabled"
    ///
    /// PhysxSchemaPhysxPhysicsDistanceJointAPI
    const TfToken physxPhysicsDistanceJointSpringEnabled;
    /// \brief "physxPhysicsDistanceJoint:springStiffness"
    ///
    /// PhysxSchemaPhysxPhysicsDistanceJointAPI
    const TfToken physxPhysicsDistanceJointSpringStiffness;
    /// \brief "physxRigidBody:angularDamping"
    ///
    /// PhysxSchemaPhysxRigidBodyAPI
    const TfToken physxRigidBodyAngularDamping;
    /// \brief "physxRigidBody:cfmScale"
    ///
    /// PhysxSchemaPhysxRigidBodyAPI
    const TfToken physxRigidBodyCfmScale;
    /// \brief "physxRigidBody:contactSlopCoefficient"
    ///
    /// PhysxSchemaPhysxRigidBodyAPI
    const TfToken physxRigidBodyContactSlopCoefficient;
    /// \brief "physxRigidBody:disableGravity"
    ///
    /// PhysxSchemaPhysxRigidBodyAPI
    const TfToken physxRigidBodyDisableGravity;
    /// \brief "physxRigidBody:enableCCD"
    ///
    /// PhysxSchemaPhysxRigidBodyAPI
    const TfToken physxRigidBodyEnableCCD;
    /// \brief "physxRigidBody:enableGyroscopicForces"
    ///
    /// PhysxSchemaPhysxRigidBodyAPI
    const TfToken physxRigidBodyEnableGyroscopicForces;
    /// \brief "physxRigidBody:enableSpeculativeCCD"
    ///
    /// PhysxSchemaPhysxRigidBodyAPI
    const TfToken physxRigidBodyEnableSpeculativeCCD;
    /// \brief "physxRigidBody:linearDamping"
    ///
    /// PhysxSchemaPhysxRigidBodyAPI
    const TfToken physxRigidBodyLinearDamping;
    /// \brief "physxRigidBody:lockedPosAxis"
    ///
    /// PhysxSchemaPhysxRigidBodyAPI
    const TfToken physxRigidBodyLockedPosAxis;
    /// \brief "physxRigidBody:lockedRotAxis"
    ///
    /// PhysxSchemaPhysxRigidBodyAPI
    const TfToken physxRigidBodyLockedRotAxis;
    /// \brief "physxRigidBody:maxAngularVelocity"
    ///
    /// PhysxSchemaPhysxRigidBodyAPI
    const TfToken physxRigidBodyMaxAngularVelocity;
    /// \brief "physxRigidBody:maxContactImpulse"
    ///
    /// PhysxSchemaPhysxRigidBodyAPI
    const TfToken physxRigidBodyMaxContactImpulse;
    /// \brief "physxRigidBody:maxDepenetrationVelocity"
    ///
    /// PhysxSchemaPhysxRigidBodyAPI
    const TfToken physxRigidBodyMaxDepenetrationVelocity;
    /// \brief "physxRigidBody:maxLinearVelocity"
    ///
    /// PhysxSchemaPhysxRigidBodyAPI
    const TfToken physxRigidBodyMaxLinearVelocity;
    /// \brief "physxRigidBody:retainAccelerations"
    ///
    /// PhysxSchemaPhysxRigidBodyAPI
    const TfToken physxRigidBodyRetainAccelerations;
    /// \brief "physxRigidBody:sleepThreshold"
    ///
    /// PhysxSchemaPhysxRigidBodyAPI
    const TfToken physxRigidBodySleepThreshold;
    /// \brief "physxRigidBody:solveContact"
    ///
    /// PhysxSchemaPhysxRigidBodyAPI
    const TfToken physxRigidBodySolveContact;
    /// \brief "physxRigidBody:solverPositionIterationCount"
    ///
    /// PhysxSchemaPhysxRigidBodyAPI
    const TfToken physxRigidBodySolverPositionIterationCount;
    /// \brief "physxRigidBody:solverVelocityIterationCount"
    ///
    /// PhysxSchemaPhysxRigidBodyAPI
    const TfToken physxRigidBodySolverVelocityIterationCount;
    /// \brief "physxRigidBody:stabilizationThreshold"
    ///
    /// PhysxSchemaPhysxRigidBodyAPI
    const TfToken physxRigidBodyStabilizationThreshold;
    /// \brief "physxScene:bounceThreshold"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneBounceThreshold;
    /// \brief "physxScene:broadphaseType"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneBroadphaseType;
    /// \brief "physxScene:collisionSystem"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneCollisionSystem;
    /// \brief "physxScene:disableSleeping"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneDisableSleeping;
    /// \brief "physxScene:enableCCD"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneEnableCCD;
    /// \brief "physxScene:enableEnhancedDeterminism"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneEnableEnhancedDeterminism;
    /// \brief "physxScene:enableExternalForcesEveryIteration"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneEnableExternalForcesEveryIteration;
    /// \brief "physxScene:enableGPUDynamics"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneEnableGPUDynamics;
    /// \brief "physxScene:enableSceneQuerySupport"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneEnableSceneQuerySupport;
    /// \brief "physxScene:enableStabilization"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneEnableStabilization;
    /// \brief "physxScene:frictionCorrelationDistance"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneFrictionCorrelationDistance;
    /// \brief "physxScene:frictionOffsetThreshold"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneFrictionOffsetThreshold;
    /// \brief "physxScene:frictionType"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneFrictionType;
    /// \brief "physxScene:gpuCollisionStackSize"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneGpuCollisionStackSize;
    /// \brief "physxScene:gpuFoundLostAggregatePairsCapacity"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneGpuFoundLostAggregatePairsCapacity;
    /// \brief "physxScene:gpuFoundLostPairsCapacity"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneGpuFoundLostPairsCapacity;
    /// \brief "physxScene:gpuHeapCapacity"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneGpuHeapCapacity;
    /// \brief "physxScene:gpuMaxDeformableSurfaceContacts"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneGpuMaxDeformableSurfaceContacts;
    /// \brief "physxScene:gpuMaxDeformableVolumeContacts"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneGpuMaxDeformableVolumeContacts;
    /// \brief "physxScene:gpuMaxNumPartitions"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneGpuMaxNumPartitions;
    /// \brief "physxScene:gpuMaxParticleContacts"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneGpuMaxParticleContacts;
    /// \brief "physxScene:gpuMaxRigidContactCount"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneGpuMaxRigidContactCount;
    /// \brief "physxScene:gpuMaxRigidPatchCount"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneGpuMaxRigidPatchCount;
    /// \brief "physxScene:gpuTempBufferCapacity"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneGpuTempBufferCapacity;
    /// \brief "physxScene:gpuTotalAggregatePairsCapacity"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneGpuTotalAggregatePairsCapacity;
    /// \brief "physxScene:invertCollisionGroupFilter"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneInvertCollisionGroupFilter;
    /// \brief "physxScene:maxBiasCoefficient"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneMaxBiasCoefficient;
    /// \brief "physxScene:maxPositionIterationCount"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneMaxPositionIterationCount;
    /// \brief "physxScene:maxVelocityIterationCount"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneMaxVelocityIterationCount;
    /// \brief "physxScene:minPositionIterationCount"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneMinPositionIterationCount;
    /// \brief "physxScene:minVelocityIterationCount"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneMinVelocityIterationCount;
    /// \brief "physxSceneQuasistatic:enableQuasistatic"
    ///
    /// PhysxSchemaPhysxSceneQuasistaticAPI
    const TfToken physxSceneQuasistaticEnableQuasistatic;
    /// \brief "physxScene:reportKinematicKinematicPairs"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneReportKinematicKinematicPairs;
    /// \brief "physxScene:reportKinematicStaticPairs"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneReportKinematicStaticPairs;
    /// \brief "physxScene:solveArticulationContactLast"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneSolveArticulationContactLast;
    /// \brief "physxScene:solverType"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneSolverType;
    /// \brief "physxScene:timeStepsPerSecond"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneTimeStepsPerSecond;
    /// \brief "physxScene:updateType"
    ///
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneUpdateType;
    /// \brief "physxSDFMeshCollision:sdfBitsPerSubgridPixel"
    ///
    /// PhysxSchemaPhysxSDFMeshCollisionAPI
    const TfToken physxSDFMeshCollisionSdfBitsPerSubgridPixel;
    /// \brief "physxSDFMeshCollision:sdfEnableRemeshing"
    ///
    /// PhysxSchemaPhysxSDFMeshCollisionAPI
    const TfToken physxSDFMeshCollisionSdfEnableRemeshing;
    /// \brief "physxSDFMeshCollision:sdfMargin"
    ///
    /// PhysxSchemaPhysxSDFMeshCollisionAPI
    const TfToken physxSDFMeshCollisionSdfMargin;
    /// \brief "physxSDFMeshCollision:sdfNarrowBandThickness"
    ///
    /// PhysxSchemaPhysxSDFMeshCollisionAPI
    const TfToken physxSDFMeshCollisionSdfNarrowBandThickness;
    /// \brief "physxSDFMeshCollision:sdfResolution"
    ///
    /// PhysxSchemaPhysxSDFMeshCollisionAPI
    const TfToken physxSDFMeshCollisionSdfResolution;
    /// \brief "physxSDFMeshCollision:sdfSubgridResolution"
    ///
    /// PhysxSchemaPhysxSDFMeshCollisionAPI
    const TfToken physxSDFMeshCollisionSdfSubgridResolution;
    /// \brief "physxSDFMeshCollision:sdfTriangleCountReductionFactor"
    ///
    /// PhysxSchemaPhysxSDFMeshCollisionAPI
    const TfToken physxSDFMeshCollisionSdfTriangleCountReductionFactor;
    /// \brief "physxSphereFillCollision:fillMode"
    ///
    /// PhysxSchemaPhysxSphereFillCollisionAPI
    const TfToken physxSphereFillCollisionFillMode;
    /// \brief "physxSphereFillCollision:maxSpheres"
    ///
    /// PhysxSchemaPhysxSphereFillCollisionAPI
    const TfToken physxSphereFillCollisionMaxSpheres;
    /// \brief "physxSphereFillCollision:seedCount"
    ///
    /// PhysxSchemaPhysxSphereFillCollisionAPI
    const TfToken physxSphereFillCollisionSeedCount;
    /// \brief "physxSphereFillCollision:voxelResolution"
    ///
    /// PhysxSchemaPhysxSphereFillCollisionAPI
    const TfToken physxSphereFillCollisionVoxelResolution;
    /// \brief "physxSplinesSurfaceVelocity:surfaceVelocityCurve"
    ///
    /// PhysxSchemaPhysxSplinesSurfaceVelocityAPI
    const TfToken physxSplinesSurfaceVelocitySurfaceVelocityCurve;
    /// \brief "physxSplinesSurfaceVelocity:surfaceVelocityEnabled"
    ///
    /// PhysxSchemaPhysxSplinesSurfaceVelocityAPI
    const TfToken physxSplinesSurfaceVelocitySurfaceVelocityEnabled;
    /// \brief "physxSplinesSurfaceVelocity:surfaceVelocityForceBased"
    ///
    /// PhysxSchemaPhysxSplinesSurfaceVelocityAPI
    const TfToken physxSplinesSurfaceVelocitySurfaceVelocityForceBased;
    /// \brief "physxSplinesSurfaceVelocity:surfaceVelocityMagnitude"
    ///
    /// PhysxSchemaPhysxSplinesSurfaceVelocityAPI
    const TfToken physxSplinesSurfaceVelocitySurfaceVelocityMagnitude;
    /// \brief "physxSurfaceVelocity:surfaceAngularVelocity"
    ///
    /// PhysxSchemaPhysxSurfaceVelocityAPI
    const TfToken physxSurfaceVelocitySurfaceAngularVelocity;
    /// \brief "physxSurfaceVelocity:surfaceVelocity"
    ///
    /// PhysxSchemaPhysxSurfaceVelocityAPI
    const TfToken physxSurfaceVelocitySurfaceVelocity;
    /// \brief "physxSurfaceVelocity:surfaceVelocityEnabled"
    ///
    /// PhysxSchemaPhysxSurfaceVelocityAPI
    const TfToken physxSurfaceVelocitySurfaceVelocityEnabled;
    /// \brief "physxSurfaceVelocity:surfaceVelocityLocalSpace"
    ///
    /// PhysxSchemaPhysxSurfaceVelocityAPI
    const TfToken physxSurfaceVelocitySurfaceVelocityLocalSpace;
    /// \brief "physxTendon"
    ///
    /// Property namespace prefix for the PhysxSchemaPhysxTendonAxisAPI schema., Property namespace prefix for the PhysxSchemaPhysxTendonAxisRootAPI schema., Property namespace prefix for the PhysxSchemaPhysxTendonAttachmentAPI schema., Property namespace prefix for the PhysxSchemaPhysxTendonAttachmentRootAPI schema., Property namespace prefix for the PhysxSchemaPhysxTendonAttachmentLeafAPI schema.
    const TfToken physxTendon;
    /// \brief "physxTendon:__INSTANCE_NAME__:damping"
    ///
    /// PhysxSchemaPhysxTendonAxisRootAPI, PhysxSchemaPhysxTendonAttachmentRootAPI
    const TfToken physxTendon_MultipleApplyTemplate_Damping;
    /// \brief "physxTendon:__INSTANCE_NAME__:forceCoefficient"
    ///
    /// PhysxSchemaPhysxTendonAxisAPI
    const TfToken physxTendon_MultipleApplyTemplate_ForceCoefficient;
    /// \brief "physxTendon:__INSTANCE_NAME__:gearing"
    ///
    /// PhysxSchemaPhysxTendonAxisAPI, PhysxSchemaPhysxTendonAttachmentAPI
    const TfToken physxTendon_MultipleApplyTemplate_Gearing;
    /// \brief "physxTendon:__INSTANCE_NAME__:jointAxis"
    ///
    /// PhysxSchemaPhysxTendonAxisAPI
    const TfToken physxTendon_MultipleApplyTemplate_JointAxis;
    /// \brief "physxTendon:__INSTANCE_NAME__:limitStiffness"
    ///
    /// PhysxSchemaPhysxTendonAxisRootAPI, PhysxSchemaPhysxTendonAttachmentRootAPI
    const TfToken physxTendon_MultipleApplyTemplate_LimitStiffness;
    /// \brief "physxTendon:__INSTANCE_NAME__:localPos"
    ///
    /// PhysxSchemaPhysxTendonAttachmentAPI
    const TfToken physxTendon_MultipleApplyTemplate_LocalPos;
    /// \brief "physxTendon:__INSTANCE_NAME__:lowerLimit"
    ///
    /// PhysxSchemaPhysxTendonAxisRootAPI, PhysxSchemaPhysxTendonAttachmentLeafAPI
    const TfToken physxTendon_MultipleApplyTemplate_LowerLimit;
    /// \brief "physxTendon:__INSTANCE_NAME__:offset"
    ///
    /// PhysxSchemaPhysxTendonAxisRootAPI, PhysxSchemaPhysxTendonAttachmentRootAPI
    const TfToken physxTendon_MultipleApplyTemplate_Offset;
    /// \brief "physxTendon:__INSTANCE_NAME__:parentAttachment"
    ///
    /// PhysxSchemaPhysxTendonAttachmentAPI
    const TfToken physxTendon_MultipleApplyTemplate_ParentAttachment;
    /// \brief "physxTendon:__INSTANCE_NAME__:parentLink"
    ///
    /// PhysxSchemaPhysxTendonAttachmentAPI
    const TfToken physxTendon_MultipleApplyTemplate_ParentLink;
    /// \brief "physxTendon:__INSTANCE_NAME__:restLength"
    ///
    /// PhysxSchemaPhysxTendonAxisRootAPI, PhysxSchemaPhysxTendonAttachmentLeafAPI
    const TfToken physxTendon_MultipleApplyTemplate_RestLength;
    /// \brief "physxTendon:__INSTANCE_NAME__:stiffness"
    ///
    /// PhysxSchemaPhysxTendonAxisRootAPI, PhysxSchemaPhysxTendonAttachmentRootAPI
    const TfToken physxTendon_MultipleApplyTemplate_Stiffness;
    /// \brief "physxTendon:__INSTANCE_NAME__:tendonEnabled"
    ///
    /// PhysxSchemaPhysxTendonAxisRootAPI, PhysxSchemaPhysxTendonAttachmentRootAPI
    const TfToken physxTendon_MultipleApplyTemplate_TendonEnabled;
    /// \brief "physxTendon:__INSTANCE_NAME__:upperLimit"
    ///
    /// PhysxSchemaPhysxTendonAxisRootAPI, PhysxSchemaPhysxTendonAttachmentLeafAPI
    const TfToken physxTendon_MultipleApplyTemplate_UpperLimit;
    /// \brief "physxTriangleMeshCollision:weldTolerance"
    ///
    /// PhysxSchemaPhysxTriangleMeshCollisionAPI
    const TfToken physxTriangleMeshCollisionWeldTolerance;
    /// \brief "physxTriangleMeshSimplificationCollision:metric"
    ///
    /// PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI
    const TfToken physxTriangleMeshSimplificationCollisionMetric;
    /// \brief "physxTriangleMeshSimplificationCollision:weldTolerance"
    ///
    /// PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI
    const TfToken physxTriangleMeshSimplificationCollisionWeldTolerance;
    /// \brief "physxTrigger:triggeredCollisions"
    ///
    /// PhysxSchemaPhysxTriggerStateAPI
    const TfToken physxTriggerTriggeredCollisions;
    /// \brief "physxVehicleAckermannSteering:maxSteerAngle"
    ///
    /// PhysxSchemaPhysxVehicleAckermannSteeringAPI
    const TfToken physxVehicleAckermannSteeringMaxSteerAngle;
    /// \brief "physxVehicleAckermannSteering:strength"
    ///
    /// PhysxSchemaPhysxVehicleAckermannSteeringAPI
    const TfToken physxVehicleAckermannSteeringStrength;
    /// \brief "physxVehicleAckermannSteering:trackWidth"
    ///
    /// PhysxSchemaPhysxVehicleAckermannSteeringAPI
    const TfToken physxVehicleAckermannSteeringTrackWidth;
    /// \brief "physxVehicleAckermannSteering:wheel0"
    ///
    /// PhysxSchemaPhysxVehicleAckermannSteeringAPI
    const TfToken physxVehicleAckermannSteeringWheel0;
    /// \brief "physxVehicleAckermannSteering:wheel1"
    ///
    /// PhysxSchemaPhysxVehicleAckermannSteeringAPI
    const TfToken physxVehicleAckermannSteeringWheel1;
    /// \brief "physxVehicleAckermannSteering:wheelBase"
    ///
    /// PhysxSchemaPhysxVehicleAckermannSteeringAPI
    const TfToken physxVehicleAckermannSteeringWheelBase;
    /// \brief "physxVehicleAutoGearBox:downRatios"
    ///
    /// PhysxSchemaPhysxVehicleAutoGearBoxAPI
    const TfToken physxVehicleAutoGearBoxDownRatios;
    /// \brief "physxVehicleAutoGearBox:latency"
    ///
    /// PhysxSchemaPhysxVehicleAutoGearBoxAPI
    const TfToken physxVehicleAutoGearBoxLatency;
    /// \brief "physxVehicleAutoGearBox:upRatios"
    ///
    /// PhysxSchemaPhysxVehicleAutoGearBoxAPI
    const TfToken physxVehicleAutoGearBoxUpRatios;
    /// \brief "physxVehicleBrakes"
    ///
    /// Property namespace prefix for the PhysxSchemaPhysxVehicleBrakesAPI schema.
    const TfToken physxVehicleBrakes;
    /// \brief "physxVehicleBrakes:__INSTANCE_NAME__:maxBrakeTorque"
    ///
    /// PhysxSchemaPhysxVehicleBrakesAPI
    const TfToken physxVehicleBrakes_MultipleApplyTemplate_MaxBrakeTorque;
    /// \brief "physxVehicleBrakes:__INSTANCE_NAME__:torqueMultipliers"
    ///
    /// PhysxSchemaPhysxVehicleBrakesAPI
    const TfToken physxVehicleBrakes_MultipleApplyTemplate_TorqueMultipliers;
    /// \brief "physxVehicleBrakes:__INSTANCE_NAME__:wheels"
    ///
    /// PhysxSchemaPhysxVehicleBrakesAPI
    const TfToken physxVehicleBrakes_MultipleApplyTemplate_Wheels;
    /// \brief "physxVehicleClutch:strength"
    ///
    /// PhysxSchemaPhysxVehicleClutchAPI
    const TfToken physxVehicleClutchStrength;
    /// \brief "physxVehicleContext:forwardAxis"
    ///
    /// PhysxSchemaPhysxVehicleContextAPI
    const TfToken physxVehicleContextForwardAxis;
    /// \brief "physxVehicleContext:longitudinalAxis"
    ///
    /// PhysxSchemaPhysxVehicleContextAPI
    const TfToken physxVehicleContextLongitudinalAxis;
    /// \brief "physxVehicleContext:upAxis"
    ///
    /// PhysxSchemaPhysxVehicleContextAPI
    const TfToken physxVehicleContextUpAxis;
    /// \brief "physxVehicleContext:updateMode"
    ///
    /// PhysxSchemaPhysxVehicleContextAPI
    const TfToken physxVehicleContextUpdateMode;
    /// \brief "physxVehicleContext:verticalAxis"
    ///
    /// PhysxSchemaPhysxVehicleContextAPI
    const TfToken physxVehicleContextVerticalAxis;
    /// \brief "physxVehicleController:accelerator"
    ///
    /// PhysxSchemaPhysxVehicleControllerAPI
    const TfToken physxVehicleControllerAccelerator;
    /// \brief "physxVehicleController:brake"
    ///
    /// PhysxSchemaPhysxVehicleControllerAPI
    const TfToken physxVehicleControllerBrake;
    /// \brief "physxVehicleController:brake0"
    ///
    /// PhysxSchemaPhysxVehicleControllerAPI
    const TfToken physxVehicleControllerBrake0;
    /// \brief "physxVehicleController:brake1"
    ///
    /// PhysxSchemaPhysxVehicleControllerAPI
    const TfToken physxVehicleControllerBrake1;
    /// \brief "physxVehicleController:handbrake"
    ///
    /// PhysxSchemaPhysxVehicleControllerAPI
    const TfToken physxVehicleControllerHandbrake;
    /// \brief "physxVehicleController:steer"
    ///
    /// PhysxSchemaPhysxVehicleControllerAPI
    const TfToken physxVehicleControllerSteer;
    /// \brief "physxVehicleController:steerLeft"
    ///
    /// PhysxSchemaPhysxVehicleControllerAPI
    const TfToken physxVehicleControllerSteerLeft;
    /// \brief "physxVehicleController:steerRight"
    ///
    /// PhysxSchemaPhysxVehicleControllerAPI
    const TfToken physxVehicleControllerSteerRight;
    /// \brief "physxVehicleController:targetGear"
    ///
    /// PhysxSchemaPhysxVehicleControllerAPI
    const TfToken physxVehicleControllerTargetGear;
    /// \brief "physxVehicle:drive"
    ///
    /// PhysxSchemaPhysxVehicleAPI
    const TfToken physxVehicleDrive;
    /// \brief "physxVehicleDriveBasic:peakTorque"
    ///
    /// PhysxSchemaPhysxVehicleDriveBasicAPI
    const TfToken physxVehicleDriveBasicPeakTorque;
    /// \brief "physxVehicleDriveStandard:autoGearBox"
    ///
    /// PhysxSchemaPhysxVehicleDriveStandardAPI
    const TfToken physxVehicleDriveStandardAutoGearBox;
    /// \brief "physxVehicleDriveStandard:clutch"
    ///
    /// PhysxSchemaPhysxVehicleDriveStandardAPI
    const TfToken physxVehicleDriveStandardClutch;
    /// \brief "physxVehicleDriveStandard:engine"
    ///
    /// PhysxSchemaPhysxVehicleDriveStandardAPI
    const TfToken physxVehicleDriveStandardEngine;
    /// \brief "physxVehicleDriveStandard:gears"
    ///
    /// PhysxSchemaPhysxVehicleDriveStandardAPI
    const TfToken physxVehicleDriveStandardGears;
    /// \brief "physxVehicleEngine:dampingRateFullThrottle"
    ///
    /// PhysxSchemaPhysxVehicleEngineAPI
    const TfToken physxVehicleEngineDampingRateFullThrottle;
    /// \brief "physxVehicleEngine:dampingRateZeroThrottleClutchDisengaged"
    ///
    /// PhysxSchemaPhysxVehicleEngineAPI
    const TfToken physxVehicleEngineDampingRateZeroThrottleClutchDisengaged;
    /// \brief "physxVehicleEngine:dampingRateZeroThrottleClutchEngaged"
    ///
    /// PhysxSchemaPhysxVehicleEngineAPI
    const TfToken physxVehicleEngineDampingRateZeroThrottleClutchEngaged;
    /// \brief "physxVehicleEngine:idleRotationSpeed"
    ///
    /// PhysxSchemaPhysxVehicleEngineAPI
    const TfToken physxVehicleEngineIdleRotationSpeed;
    /// \brief "physxVehicleEngine:maxRotationSpeed"
    ///
    /// PhysxSchemaPhysxVehicleEngineAPI
    const TfToken physxVehicleEngineMaxRotationSpeed;
    /// \brief "physxVehicleEngine:moi"
    ///
    /// PhysxSchemaPhysxVehicleEngineAPI
    const TfToken physxVehicleEngineMoi;
    /// \brief "physxVehicleEngine:peakTorque"
    ///
    /// PhysxSchemaPhysxVehicleEngineAPI
    const TfToken physxVehicleEnginePeakTorque;
    /// \brief "physxVehicleEngine:torqueCurve"
    ///
    /// PhysxSchemaPhysxVehicleEngineAPI
    const TfToken physxVehicleEngineTorqueCurve;
    /// \brief "physxVehicleGears:ratios"
    ///
    /// PhysxSchemaPhysxVehicleGearsAPI
    const TfToken physxVehicleGearsRatios;
    /// \brief "physxVehicleGears:ratioScale"
    ///
    /// PhysxSchemaPhysxVehicleGearsAPI
    const TfToken physxVehicleGearsRatioScale;
    /// \brief "physxVehicleGears:switchTime"
    ///
    /// PhysxSchemaPhysxVehicleGearsAPI
    const TfToken physxVehicleGearsSwitchTime;
    /// \brief "physxVehicle:highForwardSpeedSubStepCount"
    ///
    /// PhysxSchemaPhysxVehicleAPI
    const TfToken physxVehicleHighForwardSpeedSubStepCount;
    /// \brief "physxVehicle:lateralStickyTireDamping"
    ///
    /// PhysxSchemaPhysxVehicleAPI
    const TfToken physxVehicleLateralStickyTireDamping;
    /// \brief "physxVehicle:lateralStickyTireThresholdSpeed"
    ///
    /// PhysxSchemaPhysxVehicleAPI
    const TfToken physxVehicleLateralStickyTireThresholdSpeed;
    /// \brief "physxVehicle:lateralStickyTireThresholdTime"
    ///
    /// PhysxSchemaPhysxVehicleAPI
    const TfToken physxVehicleLateralStickyTireThresholdTime;
    /// \brief "physxVehicle:limitSuspensionExpansionVelocity"
    ///
    /// PhysxSchemaPhysxVehicleAPI
    const TfToken physxVehicleLimitSuspensionExpansionVelocity;
    /// \brief "physxVehicle:longitudinalStickyTireDamping"
    ///
    /// PhysxSchemaPhysxVehicleAPI
    const TfToken physxVehicleLongitudinalStickyTireDamping;
    /// \brief "physxVehicle:longitudinalStickyTireThresholdSpeed"
    ///
    /// PhysxSchemaPhysxVehicleAPI
    const TfToken physxVehicleLongitudinalStickyTireThresholdSpeed;
    /// \brief "physxVehicle:longitudinalStickyTireThresholdTime"
    ///
    /// PhysxSchemaPhysxVehicleAPI
    const TfToken physxVehicleLongitudinalStickyTireThresholdTime;
    /// \brief "physxVehicle:lowForwardSpeedSubStepCount"
    ///
    /// PhysxSchemaPhysxVehicleAPI
    const TfToken physxVehicleLowForwardSpeedSubStepCount;
    /// \brief "physxVehicle:minActiveLongitudinalSlipDenominator"
    ///
    /// PhysxSchemaPhysxVehicleAPI
    const TfToken physxVehicleMinActiveLongitudinalSlipDenominator;
    /// \brief "physxVehicle:minLateralSlipDenominator"
    ///
    /// PhysxSchemaPhysxVehicleAPI
    const TfToken physxVehicleMinLateralSlipDenominator;
    /// \brief "physxVehicle:minLongitudinalSlipDenominator"
    ///
    /// PhysxSchemaPhysxVehicleAPI
    const TfToken physxVehicleMinLongitudinalSlipDenominator;
    /// \brief "physxVehicle:minPassiveLongitudinalSlipDenominator"
    ///
    /// PhysxSchemaPhysxVehicleAPI
    const TfToken physxVehicleMinPassiveLongitudinalSlipDenominator;
    /// \brief "physxVehicleMultiWheelDifferential:averageWheelSpeedRatios"
    ///
    /// PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI
    const TfToken physxVehicleMultiWheelDifferentialAverageWheelSpeedRatios;
    /// \brief "physxVehicleMultiWheelDifferential:torqueRatios"
    ///
    /// PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI
    const TfToken physxVehicleMultiWheelDifferentialTorqueRatios;
    /// \brief "physxVehicleMultiWheelDifferential:wheels"
    ///
    /// PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI
    const TfToken physxVehicleMultiWheelDifferentialWheels;
    /// \brief "physxVehicleNCR"
    ///
    /// Property namespace prefix for the PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI schema.
    const TfToken physxVehicleNCR;
    /// \brief "physxVehicleNCR:__INSTANCE_NAME__:commandValues"
    ///
    /// PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI
    const TfToken physxVehicleNCR_MultipleApplyTemplate_CommandValues;
    /// \brief "physxVehicleNCR:__INSTANCE_NAME__:speedResponses"
    ///
    /// PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI
    const TfToken physxVehicleNCR_MultipleApplyTemplate_SpeedResponses;
    /// \brief "physxVehicleNCR:__INSTANCE_NAME__:speedResponsesPerCommandValue"
    ///
    /// PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI
    const TfToken physxVehicleNCR_MultipleApplyTemplate_SpeedResponsesPerCommandValue;
    /// \brief "physxVehicleSteering:angleMultipliers"
    ///
    /// PhysxSchemaPhysxVehicleSteeringAPI
    const TfToken physxVehicleSteeringAngleMultipliers;
    /// \brief "physxVehicleSteering:maxSteerAngle"
    ///
    /// PhysxSchemaPhysxVehicleSteeringAPI
    const TfToken physxVehicleSteeringMaxSteerAngle;
    /// \brief "physxVehicleSteering:wheels"
    ///
    /// PhysxSchemaPhysxVehicleSteeringAPI
    const TfToken physxVehicleSteeringWheels;
    /// \brief "physxVehicle:subStepThresholdLongitudinalSpeed"
    ///
    /// PhysxSchemaPhysxVehicleAPI
    const TfToken physxVehicleSubStepThresholdLongitudinalSpeed;
    /// \brief "physxVehicleSuspension:camberAtMaxCompression"
    ///
    /// PhysxSchemaPhysxVehicleSuspensionAPI
    const TfToken physxVehicleSuspensionCamberAtMaxCompression;
    /// \brief "physxVehicleSuspension:camberAtMaxDroop"
    ///
    /// PhysxSchemaPhysxVehicleSuspensionAPI
    const TfToken physxVehicleSuspensionCamberAtMaxDroop;
    /// \brief "physxVehicleSuspension:camberAtRest"
    ///
    /// PhysxSchemaPhysxVehicleSuspensionAPI
    const TfToken physxVehicleSuspensionCamberAtRest;
    /// \brief "physxVehicleSuspensionCompliance:suspensionForceAppPoint"
    ///
    /// PhysxSchemaPhysxVehicleSuspensionComplianceAPI
    const TfToken physxVehicleSuspensionComplianceSuspensionForceAppPoint;
    /// \brief "physxVehicleSuspensionCompliance:tireForceAppPoint"
    ///
    /// PhysxSchemaPhysxVehicleSuspensionComplianceAPI
    const TfToken physxVehicleSuspensionComplianceTireForceAppPoint;
    /// \brief "physxVehicleSuspensionCompliance:wheelCamberAngle"
    ///
    /// PhysxSchemaPhysxVehicleSuspensionComplianceAPI
    const TfToken physxVehicleSuspensionComplianceWheelCamberAngle;
    /// \brief "physxVehicleSuspensionCompliance:wheelToeAngle"
    ///
    /// PhysxSchemaPhysxVehicleSuspensionComplianceAPI
    const TfToken physxVehicleSuspensionComplianceWheelToeAngle;
    /// \brief "physxVehicle:suspensionLineQueryType"
    ///
    /// PhysxSchemaPhysxVehicleAPI
    const TfToken physxVehicleSuspensionLineQueryType;
    /// \brief "physxVehicleSuspension:maxCompression"
    ///
    /// PhysxSchemaPhysxVehicleSuspensionAPI
    const TfToken physxVehicleSuspensionMaxCompression;
    /// \brief "physxVehicleSuspension:maxDroop"
    ///
    /// PhysxSchemaPhysxVehicleSuspensionAPI
    const TfToken physxVehicleSuspensionMaxDroop;
    /// \brief "physxVehicleSuspension:springDamperRate"
    ///
    /// PhysxSchemaPhysxVehicleSuspensionAPI
    const TfToken physxVehicleSuspensionSpringDamperRate;
    /// \brief "physxVehicleSuspension:springStrength"
    ///
    /// PhysxSchemaPhysxVehicleSuspensionAPI
    const TfToken physxVehicleSuspensionSpringStrength;
    /// \brief "physxVehicleSuspension:sprungMass"
    ///
    /// PhysxSchemaPhysxVehicleSuspensionAPI
    const TfToken physxVehicleSuspensionSprungMass;
    /// \brief "physxVehicleSuspension:travelDistance"
    ///
    /// PhysxSchemaPhysxVehicleSuspensionAPI
    const TfToken physxVehicleSuspensionTravelDistance;
    /// \brief "physxVehicleTankController:thrust0"
    ///
    /// PhysxSchemaPhysxVehicleTankControllerAPI
    const TfToken physxVehicleTankControllerThrust0;
    /// \brief "physxVehicleTankController:thrust1"
    ///
    /// PhysxSchemaPhysxVehicleTankControllerAPI
    const TfToken physxVehicleTankControllerThrust1;
    /// \brief "physxVehicleTankDifferential:numberOfWheelsPerTrack"
    ///
    /// PhysxSchemaPhysxVehicleTankDifferentialAPI
    const TfToken physxVehicleTankDifferentialNumberOfWheelsPerTrack;
    /// \brief "physxVehicleTankDifferential:thrustIndexPerTrack"
    ///
    /// PhysxSchemaPhysxVehicleTankDifferentialAPI
    const TfToken physxVehicleTankDifferentialThrustIndexPerTrack;
    /// \brief "physxVehicleTankDifferential:trackToWheelIndices"
    ///
    /// PhysxSchemaPhysxVehicleTankDifferentialAPI
    const TfToken physxVehicleTankDifferentialTrackToWheelIndices;
    /// \brief "physxVehicleTankDifferential:wheelIndicesInTrackOrder"
    ///
    /// PhysxSchemaPhysxVehicleTankDifferentialAPI
    const TfToken physxVehicleTankDifferentialWheelIndicesInTrackOrder;
    /// \brief "physxVehicleTire:camberStiffness"
    ///
    /// PhysxSchemaPhysxVehicleTireAPI
    const TfToken physxVehicleTireCamberStiffness;
    /// \brief "physxVehicleTire:camberStiffnessPerUnitGravity"
    ///
    /// PhysxSchemaPhysxVehicleTireAPI
    const TfToken physxVehicleTireCamberStiffnessPerUnitGravity;
    /// \brief "physxVehicleTire:frictionTable"
    ///
    /// PhysxSchemaPhysxVehicleTireAPI
    const TfToken physxVehicleTireFrictionTable;
    /// \brief "physxVehicleTire:frictionVsSlipGraph"
    ///
    /// PhysxSchemaPhysxVehicleTireAPI
    const TfToken physxVehicleTireFrictionVsSlipGraph;
    /// \brief "physxVehicleTire:lateralStiffnessGraph"
    ///
    /// PhysxSchemaPhysxVehicleTireAPI
    const TfToken physxVehicleTireLateralStiffnessGraph;
    /// \brief "physxVehicleTire:latStiffX"
    ///
    /// PhysxSchemaPhysxVehicleTireAPI
    const TfToken physxVehicleTireLatStiffX;
    /// \brief "physxVehicleTire:latStiffY"
    ///
    /// PhysxSchemaPhysxVehicleTireAPI
    const TfToken physxVehicleTireLatStiffY;
    /// \brief "physxVehicleTire:longitudinalStiffness"
    ///
    /// PhysxSchemaPhysxVehicleTireAPI
    const TfToken physxVehicleTireLongitudinalStiffness;
    /// \brief "physxVehicleTire:longitudinalStiffnessPerUnitGravity"
    ///
    /// PhysxSchemaPhysxVehicleTireAPI
    const TfToken physxVehicleTireLongitudinalStiffnessPerUnitGravity;
    /// \brief "physxVehicleTire:restLoad"
    ///
    /// PhysxSchemaPhysxVehicleTireAPI
    const TfToken physxVehicleTireRestLoad;
    /// \brief "physxVehicle:vehicleEnabled"
    ///
    /// PhysxSchemaPhysxVehicleAPI
    const TfToken physxVehicleVehicleEnabled;
    /// \brief "physxVehicleWheelAttachment:collisionGroup"
    ///
    /// PhysxSchemaPhysxVehicleWheelAttachmentAPI
    const TfToken physxVehicleWheelAttachmentCollisionGroup;
    /// \brief "physxVehicleWheelAttachment:driven"
    ///
    /// PhysxSchemaPhysxVehicleWheelAttachmentAPI
    const TfToken physxVehicleWheelAttachmentDriven;
    /// \brief "physxVehicleWheelAttachment:index"
    ///
    /// PhysxSchemaPhysxVehicleWheelAttachmentAPI
    const TfToken physxVehicleWheelAttachmentIndex;
    /// \brief "physxVehicleWheelAttachment:suspension"
    ///
    /// PhysxSchemaPhysxVehicleWheelAttachmentAPI
    const TfToken physxVehicleWheelAttachmentSuspension;
    /// \brief "physxVehicleWheelAttachment:suspensionForceAppPointOffset"
    ///
    /// PhysxSchemaPhysxVehicleWheelAttachmentAPI
    const TfToken physxVehicleWheelAttachmentSuspensionForceAppPointOffset;
    /// \brief "physxVehicleWheelAttachment:suspensionFrameOrientation"
    ///
    /// PhysxSchemaPhysxVehicleWheelAttachmentAPI
    const TfToken physxVehicleWheelAttachmentSuspensionFrameOrientation;
    /// \brief "physxVehicleWheelAttachment:suspensionFramePosition"
    ///
    /// PhysxSchemaPhysxVehicleWheelAttachmentAPI
    const TfToken physxVehicleWheelAttachmentSuspensionFramePosition;
    /// \brief "physxVehicleWheelAttachment:suspensionTravelDirection"
    ///
    /// PhysxSchemaPhysxVehicleWheelAttachmentAPI
    const TfToken physxVehicleWheelAttachmentSuspensionTravelDirection;
    /// \brief "physxVehicleWheelAttachment:tire"
    ///
    /// PhysxSchemaPhysxVehicleWheelAttachmentAPI
    const TfToken physxVehicleWheelAttachmentTire;
    /// \brief "physxVehicleWheelAttachment:tireForceAppPointOffset"
    ///
    /// PhysxSchemaPhysxVehicleWheelAttachmentAPI
    const TfToken physxVehicleWheelAttachmentTireForceAppPointOffset;
    /// \brief "physxVehicleWheelAttachment:wheel"
    ///
    /// PhysxSchemaPhysxVehicleWheelAttachmentAPI
    const TfToken physxVehicleWheelAttachmentWheel;
    /// \brief "physxVehicleWheelAttachment:wheelCenterOfMassOffset"
    ///
    /// PhysxSchemaPhysxVehicleWheelAttachmentAPI
    const TfToken physxVehicleWheelAttachmentWheelCenterOfMassOffset;
    /// \brief "physxVehicleWheelAttachment:wheelFrameOrientation"
    ///
    /// PhysxSchemaPhysxVehicleWheelAttachmentAPI
    const TfToken physxVehicleWheelAttachmentWheelFrameOrientation;
    /// \brief "physxVehicleWheelAttachment:wheelFramePosition"
    ///
    /// PhysxSchemaPhysxVehicleWheelAttachmentAPI
    const TfToken physxVehicleWheelAttachmentWheelFramePosition;
    /// \brief "physxVehicleWheelController:brakeTorque"
    ///
    /// PhysxSchemaPhysxVehicleWheelControllerAPI
    const TfToken physxVehicleWheelControllerBrakeTorque;
    /// \brief "physxVehicleWheelController:driveTorque"
    ///
    /// PhysxSchemaPhysxVehicleWheelControllerAPI
    const TfToken physxVehicleWheelControllerDriveTorque;
    /// \brief "physxVehicleWheelController:steerAngle"
    ///
    /// PhysxSchemaPhysxVehicleWheelControllerAPI
    const TfToken physxVehicleWheelControllerSteerAngle;
    /// \brief "physxVehicleWheel:dampingRate"
    ///
    /// PhysxSchemaPhysxVehicleWheelAPI
    const TfToken physxVehicleWheelDampingRate;
    /// \brief "physxVehicleWheel:mass"
    ///
    /// PhysxSchemaPhysxVehicleWheelAPI
    const TfToken physxVehicleWheelMass;
    /// \brief "physxVehicleWheel:maxBrakeTorque"
    ///
    /// PhysxSchemaPhysxVehicleWheelAPI
    const TfToken physxVehicleWheelMaxBrakeTorque;
    /// \brief "physxVehicleWheel:maxHandBrakeTorque"
    ///
    /// PhysxSchemaPhysxVehicleWheelAPI
    const TfToken physxVehicleWheelMaxHandBrakeTorque;
    /// \brief "physxVehicleWheel:maxSteerAngle"
    ///
    /// PhysxSchemaPhysxVehicleWheelAPI
    const TfToken physxVehicleWheelMaxSteerAngle;
    /// \brief "physxVehicleWheel:moi"
    ///
    /// PhysxSchemaPhysxVehicleWheelAPI
    const TfToken physxVehicleWheelMoi;
    /// \brief "physxVehicleWheel:radius"
    ///
    /// PhysxSchemaPhysxVehicleWheelAPI
    const TfToken physxVehicleWheelRadius;
    /// \brief "physxVehicleWheel:toeAngle"
    ///
    /// PhysxSchemaPhysxVehicleWheelAPI
    const TfToken physxVehicleWheelToeAngle;
    /// \brief "physxVehicleWheel:width"
    ///
    /// PhysxSchemaPhysxVehicleWheelAPI
    const TfToken physxVehicleWheelWidth;
    /// \brief "posX"
    ///
    /// Possible value for PhysxSchemaPhysxVehicleContextAPI::GetLongitudinalAxisAttr(), Possible value for PhysxSchemaPhysxVehicleContextAPI::GetVerticalAxisAttr()
    const TfToken posX;
    /// \brief "posY"
    ///
    /// Possible value for PhysxSchemaPhysxVehicleContextAPI::GetLongitudinalAxisAttr(), Possible value for PhysxSchemaPhysxVehicleContextAPI::GetVerticalAxisAttr()
    const TfToken posY;
    /// \brief "posZ"
    ///
    /// Possible value for PhysxSchemaPhysxVehicleContextAPI::GetLongitudinalAxisAttr(), Possible value for PhysxSchemaPhysxVehicleContextAPI::GetVerticalAxisAttr()
    const TfToken posZ;
    /// \brief "preventClimbing"
    ///
    /// Fallback value for PhysxSchemaPhysxCharacterControllerAPI::GetNonWalkableModeAttr()
    const TfToken preventClimbing;
    /// \brief "preventClimbingForceSliding"
    ///
    /// Possible value for PhysxSchemaPhysxCharacterControllerAPI::GetNonWalkableModeAttr()
    const TfToken preventClimbingForceSliding;
    /// \brief "quasistaticactors"
    ///
    ///  This token defines the PhysxSceneQuasistaticAPI collection that gathers the quasistatic actors.
    const TfToken quasistaticactors;
    /// \brief "raycast"
    ///
    /// Possible value for PhysxSchemaPhysxSphereFillCollisionAPI::GetFillModeAttr(), Fallback value for PhysxSchemaPhysxVehicleAPI::GetSuspensionLineQueryTypeAttr()
    const TfToken raycast;
    /// \brief "physxVehicle:referenceFrameIsCenterOfMass"
    ///
    ///  This token represents a boolean custom metadata attribute that defines whether some vehicle wheel attachment properties are defined relative to the vehicle prim coordinate frame or relative to the vehicle center of mass coordinate frame. The affected properties are: suspensionTravelDirection, suspensionFramePosition, suspensionFrameOrientation, suspensionForceAppPointOffset, wheelCenterOfMassOffset and tireForceAppPointOffset. This custom metadata can be set on the prim that has PhysxVehicleAPI applied. Note that using the center of mass frame as reference (=True) is deprecated and will not be supported for much longer.
    const TfToken referenceFrameIsCenterOfMass;
    /// \brief "restOffset"
    ///
    /// PhysxSchemaPhysxParticleSystem
    const TfToken restOffset;
    /// \brief "rotX"
    ///
    /// Possible value for PhysxSchemaPhysxTendonAxisAPI::GetJointAxisAttr(), Fallback value for PhysxSchemaPhysxMimicJointAPI::GetReferenceJointAxisAttr()
    const TfToken rotX;
    /// \brief "rotY"
    ///
    /// Possible value for PhysxSchemaPhysxTendonAxisAPI::GetJointAxisAttr(), Possible value for PhysxSchemaPhysxMimicJointAPI::GetReferenceJointAxisAttr()
    const TfToken rotY;
    /// \brief "rotZ"
    ///
    /// Possible value for PhysxSchemaPhysxTendonAxisAPI::GetJointAxisAttr(), Possible value for PhysxSchemaPhysxMimicJointAPI::GetReferenceJointAxisAttr()
    const TfToken rotZ;
    /// \brief "SAP"
    ///
    /// Possible value for PhysxSchemaPhysxSceneAPI::GetBroadphaseTypeAttr()
    const TfToken SAP;
    /// \brief "SAT"
    ///
    /// Possible value for PhysxSchemaPhysxSceneAPI::GetCollisionSystemAttr()
    const TfToken SAT;
    /// \brief "sdf"
    ///
    ///  This token represents the SDF triangle mesh approximation.
    const TfToken sdf;
    /// \brief "simulationOwner"
    ///
    /// PhysxSchemaPhysxParticleSystem
    const TfToken simulationOwner;
    /// \brief "solidRestOffset"
    ///
    /// PhysxSchemaPhysxParticleSystem
    const TfToken solidRestOffset;
    /// \brief "solverPositionIterationCount"
    ///
    /// PhysxSchemaPhysxParticleSystem
    const TfToken solverPositionIterationCount;
    /// \brief "sphereFill"
    ///
    ///  This token represents sphere fill approximation.
    const TfToken sphereFill;
    /// \brief "state"
    ///
    /// Property namespace prefix for the PhysxSchemaJointStateAPI schema.
    const TfToken state;
    /// \brief "state:__INSTANCE_NAME__:physics:position"
    ///
    /// PhysxSchemaJointStateAPI
    const TfToken state_MultipleApplyTemplate_PhysicsPosition;
    /// \brief "state:__INSTANCE_NAME__:physics:velocity"
    ///
    /// PhysxSchemaJointStateAPI
    const TfToken state_MultipleApplyTemplate_PhysicsVelocity;
    /// \brief "steer"
    ///
    ///  This token holds the instance name to use for PhysxVehicleNonlinearCommandResponseAPI when applying it to the steering system.
    const TfToken steer;
    /// \brief "surface"
    ///
    /// Possible value for PhysxSchemaPhysxSphereFillCollisionAPI::GetFillModeAttr()
    const TfToken surface;
    /// \brief "sweep"
    ///
    /// Possible value for PhysxSchemaPhysxVehicleAPI::GetSuspensionLineQueryTypeAttr()
    const TfToken sweep;
    /// \brief "Synchronous"
    ///
    /// Fallback value for PhysxSchemaPhysxSceneAPI::GetUpdateTypeAttr()
    const TfToken Synchronous;
    /// \brief "TGS"
    ///
    /// Fallback value for PhysxSchemaPhysxSceneAPI::GetSolverTypeAttr()
    const TfToken TGS;
    /// \brief "transX"
    ///
    /// Possible value for PhysxSchemaPhysxTendonAxisAPI::GetJointAxisAttr()
    const TfToken transX;
    /// \brief "transY"
    ///
    /// Possible value for PhysxSchemaPhysxTendonAxisAPI::GetJointAxisAttr()
    const TfToken transY;
    /// \brief "transZ"
    ///
    /// Possible value for PhysxSchemaPhysxTendonAxisAPI::GetJointAxisAttr()
    const TfToken transZ;
    /// \brief "triangleMesh"
    ///
    ///  This token represents the collection name to use with PhysxCookedDataAPI to represent cooked data of a TriangleMesh.
    const TfToken triangleMesh;
    /// \brief "undefined"
    ///
    /// Fallback value for PhysxSchemaPhysxVehicleContextAPI::GetLongitudinalAxisAttr(), Fallback value for PhysxSchemaPhysxVehicleContextAPI::GetVerticalAxisAttr()
    const TfToken undefined;
    /// \brief "velocityChange"
    ///
    /// Fallback value for PhysxSchemaPhysxVehicleContextAPI::GetUpdateModeAttr()
    const TfToken velocityChange;
    /// \brief "wind"
    ///
    /// PhysxSchemaPhysxParticleSystem
    const TfToken wind;
    /// \brief "X"
    ///
    /// Possible value for PhysxSchemaPhysxCharacterControllerAPI::GetUpAxisAttr()
    const TfToken X;
    /// \brief "Y"
    ///
    /// Possible value for PhysxSchemaPhysxCharacterControllerAPI::GetUpAxisAttr()
    const TfToken Y;
    /// \brief "Z"
    ///
    /// Fallback value for PhysxSchemaPhysxCharacterControllerAPI::GetUpAxisAttr()
    const TfToken Z;
    /// \brief "PhysicsJointStateAPI"
    ///
    /// Schema identifer and family for PhysxSchemaJointStateAPI
    const TfToken PhysicsJointStateAPI;
    /// \brief "PhysxArticulationAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxArticulationAPI
    const TfToken PhysxArticulationAPI;
    /// \brief "PhysxAutoDeformableAttachmentAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxAutoDeformableAttachmentAPI
    const TfToken PhysxAutoDeformableAttachmentAPI;
    /// \brief "PhysxAutoDeformableBodyAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxAutoDeformableBodyAPI
    const TfToken PhysxAutoDeformableBodyAPI;
    /// \brief "PhysxAutoDeformableHexahedralMeshAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxAutoDeformableHexahedralMeshAPI
    const TfToken PhysxAutoDeformableHexahedralMeshAPI;
    /// \brief "PhysxAutoDeformableMeshSimplificationAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxAutoDeformableMeshSimplificationAPI
    const TfToken PhysxAutoDeformableMeshSimplificationAPI;
    /// \brief "PhysxBaseDeformableBodyAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxBaseDeformableBodyAPI
    const TfToken PhysxBaseDeformableBodyAPI;
    /// \brief "PhysxCameraAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxCameraAPI
    const TfToken PhysxCameraAPI;
    /// \brief "PhysxCameraDroneAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxCameraDroneAPI
    const TfToken PhysxCameraDroneAPI;
    /// \brief "PhysxCameraFollowAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxCameraFollowAPI
    const TfToken PhysxCameraFollowAPI;
    /// \brief "PhysxCameraFollowLookAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxCameraFollowLookAPI
    const TfToken PhysxCameraFollowLookAPI;
    /// \brief "PhysxCameraFollowVelocityAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxCameraFollowVelocityAPI
    const TfToken PhysxCameraFollowVelocityAPI;
    /// \brief "PhysxCharacterControllerAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxCharacterControllerAPI
    const TfToken PhysxCharacterControllerAPI;
    /// \brief "PhysxCollisionAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxCollisionAPI
    const TfToken PhysxCollisionAPI;
    /// \brief "PhysxContactReportAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxContactReportAPI
    const TfToken PhysxContactReportAPI;
    /// \brief "PhysxConvexDecompositionCollisionAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxConvexDecompositionCollisionAPI
    const TfToken PhysxConvexDecompositionCollisionAPI;
    /// \brief "PhysxConvexHullCollisionAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxConvexHullCollisionAPI
    const TfToken PhysxConvexHullCollisionAPI;
    /// \brief "PhysxCookedDataAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxCookedDataAPI
    const TfToken PhysxCookedDataAPI;
    /// \brief "PhysxDeformableMaterialAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxDeformableMaterialAPI
    const TfToken PhysxDeformableMaterialAPI;
    /// \brief "PhysxDiffuseParticlesAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxDiffuseParticlesAPI
    const TfToken PhysxDiffuseParticlesAPI;
    /// \brief "PhysxDrivePerformanceEnvelopeAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxDrivePerformanceEnvelopeAPI
    const TfToken PhysxDrivePerformanceEnvelopeAPI;
    /// \brief "PhysxForceAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxForceAPI
    const TfToken PhysxForceAPI;
    /// \brief "PhysxJointAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxJointAPI
    const TfToken PhysxJointAPI;
    /// \brief "PhysxJointAxisAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxJointAxisAPI
    const TfToken PhysxJointAxisAPI;
    /// \brief "PhysxLimitAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxLimitAPI
    const TfToken PhysxLimitAPI;
    /// \brief "PhysxMaterialAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxMaterialAPI
    const TfToken PhysxMaterialAPI;
    /// \brief "PhysxMeshMergeCollisionAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxMeshMergeCollisionAPI
    const TfToken PhysxMeshMergeCollisionAPI;
    /// \brief "PhysxMimicJointAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxMimicJointAPI
    const TfToken PhysxMimicJointAPI;
    /// \brief "PhysxParticleAnisotropyAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxParticleAnisotropyAPI
    const TfToken PhysxParticleAnisotropyAPI;
    /// \brief "PhysxParticleAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxParticleAPI
    const TfToken PhysxParticleAPI;
    /// \brief "PhysxParticleIsosurfaceAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxParticleIsosurfaceAPI
    const TfToken PhysxParticleIsosurfaceAPI;
    /// \brief "PhysxParticleSamplingAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxParticleSamplingAPI
    const TfToken PhysxParticleSamplingAPI;
    /// \brief "PhysxParticleSetAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxParticleSetAPI
    const TfToken PhysxParticleSetAPI;
    /// \brief "PhysxParticleSmoothingAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxParticleSmoothingAPI
    const TfToken PhysxParticleSmoothingAPI;
    /// \brief "PhysxParticleSystem"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxParticleSystem
    const TfToken PhysxParticleSystem;
    /// \brief "PhysxPBDMaterialAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxPBDMaterialAPI
    const TfToken PhysxPBDMaterialAPI;
    /// \brief "PhysxPhysicsDistanceJointAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxPhysicsDistanceJointAPI
    const TfToken PhysxPhysicsDistanceJointAPI;
    /// \brief "PhysxPhysicsGearJoint"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxPhysicsGearJoint
    const TfToken PhysxPhysicsGearJoint;
    /// \brief "PhysxPhysicsInstancer"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxPhysicsInstancer
    const TfToken PhysxPhysicsInstancer;
    /// \brief "PhysxPhysicsJointInstancer"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxPhysicsJointInstancer
    const TfToken PhysxPhysicsJointInstancer;
    /// \brief "PhysxPhysicsRackAndPinionJoint"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxPhysicsRackAndPinionJoint
    const TfToken PhysxPhysicsRackAndPinionJoint;
    /// \brief "PhysxRigidBodyAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxRigidBodyAPI
    const TfToken PhysxRigidBodyAPI;
    /// \brief "PhysxSceneAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxSceneAPI
    const TfToken PhysxSceneAPI;
    /// \brief "PhysxSceneQuasistaticAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxSceneQuasistaticAPI
    const TfToken PhysxSceneQuasistaticAPI;
    /// \brief "PhysxSDFMeshCollisionAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxSDFMeshCollisionAPI
    const TfToken PhysxSDFMeshCollisionAPI;
    /// \brief "PhysxSphereFillCollisionAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxSphereFillCollisionAPI
    const TfToken PhysxSphereFillCollisionAPI;
    /// \brief "PhysxSplinesSurfaceVelocityAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxSplinesSurfaceVelocityAPI
    const TfToken PhysxSplinesSurfaceVelocityAPI;
    /// \brief "PhysxSurfaceDeformableBodyAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxSurfaceDeformableBodyAPI
    const TfToken PhysxSurfaceDeformableBodyAPI;
    /// \brief "PhysxSurfaceDeformableMaterialAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxSurfaceDeformableMaterialAPI
    const TfToken PhysxSurfaceDeformableMaterialAPI;
    /// \brief "PhysxSurfaceVelocityAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxSurfaceVelocityAPI
    const TfToken PhysxSurfaceVelocityAPI;
    /// \brief "PhysxTendonAttachmentAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxTendonAttachmentAPI
    const TfToken PhysxTendonAttachmentAPI;
    /// \brief "PhysxTendonAttachmentLeafAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxTendonAttachmentLeafAPI
    const TfToken PhysxTendonAttachmentLeafAPI;
    /// \brief "PhysxTendonAttachmentRootAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxTendonAttachmentRootAPI
    const TfToken PhysxTendonAttachmentRootAPI;
    /// \brief "PhysxTendonAxisAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxTendonAxisAPI
    const TfToken PhysxTendonAxisAPI;
    /// \brief "PhysxTendonAxisRootAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxTendonAxisRootAPI
    const TfToken PhysxTendonAxisRootAPI;
    /// \brief "PhysxTriangleMeshCollisionAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxTriangleMeshCollisionAPI
    const TfToken PhysxTriangleMeshCollisionAPI;
    /// \brief "PhysxTriangleMeshSimplificationCollisionAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI
    const TfToken PhysxTriangleMeshSimplificationCollisionAPI;
    /// \brief "PhysxTriggerAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxTriggerAPI
    const TfToken PhysxTriggerAPI;
    /// \brief "PhysxTriggerStateAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxTriggerStateAPI
    const TfToken PhysxTriggerStateAPI;
    /// \brief "PhysxVehicleAckermannSteeringAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleAckermannSteeringAPI
    const TfToken PhysxVehicleAckermannSteeringAPI;
    /// \brief "PhysxVehicleAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleAPI
    const TfToken PhysxVehicleAPI;
    /// \brief "PhysxVehicleAutoGearBoxAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleAutoGearBoxAPI
    const TfToken PhysxVehicleAutoGearBoxAPI;
    /// \brief "PhysxVehicleBrakesAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleBrakesAPI
    const TfToken PhysxVehicleBrakesAPI;
    /// \brief "PhysxVehicleClutchAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleClutchAPI
    const TfToken PhysxVehicleClutchAPI;
    /// \brief "PhysxVehicleContextAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleContextAPI
    const TfToken PhysxVehicleContextAPI;
    /// \brief "PhysxVehicleControllerAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleControllerAPI
    const TfToken PhysxVehicleControllerAPI;
    /// \brief "PhysxVehicleDriveBasicAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleDriveBasicAPI
    const TfToken PhysxVehicleDriveBasicAPI;
    /// \brief "PhysxVehicleDriveStandardAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleDriveStandardAPI
    const TfToken PhysxVehicleDriveStandardAPI;
    /// \brief "PhysxVehicleEngineAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleEngineAPI
    const TfToken PhysxVehicleEngineAPI;
    /// \brief "PhysxVehicleGearsAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleGearsAPI
    const TfToken PhysxVehicleGearsAPI;
    /// \brief "PhysxVehicleMultiWheelDifferentialAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI
    const TfToken PhysxVehicleMultiWheelDifferentialAPI;
    /// \brief "PhysxVehicleNonlinearCommandResponseAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI
    const TfToken PhysxVehicleNonlinearCommandResponseAPI;
    /// \brief "PhysxVehicleSteeringAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleSteeringAPI
    const TfToken PhysxVehicleSteeringAPI;
    /// \brief "PhysxVehicleSuspensionAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleSuspensionAPI
    const TfToken PhysxVehicleSuspensionAPI;
    /// \brief "PhysxVehicleSuspensionComplianceAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleSuspensionComplianceAPI
    const TfToken PhysxVehicleSuspensionComplianceAPI;
    /// \brief "PhysxVehicleTankControllerAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleTankControllerAPI
    const TfToken PhysxVehicleTankControllerAPI;
    /// \brief "PhysxVehicleTankDifferentialAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleTankDifferentialAPI
    const TfToken PhysxVehicleTankDifferentialAPI;
    /// \brief "PhysxVehicleTireAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleTireAPI
    const TfToken PhysxVehicleTireAPI;
    /// \brief "PhysxVehicleTireFrictionTable"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleTireFrictionTable
    const TfToken PhysxVehicleTireFrictionTable;
    /// \brief "PhysxVehicleWheelAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleWheelAPI
    const TfToken PhysxVehicleWheelAPI;
    /// \brief "PhysxVehicleWheelAttachmentAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleWheelAttachmentAPI
    const TfToken PhysxVehicleWheelAttachmentAPI;
    /// \brief "PhysxVehicleWheelControllerAPI"
    ///
    /// Schema identifer and family for PhysxSchemaPhysxVehicleWheelControllerAPI
    const TfToken PhysxVehicleWheelControllerAPI;
    /// A vector of all of the tokens listed above.
    const std::vector<TfToken> allTokens;
};

inline PhysxSchemaTokensType::PhysxSchemaTokensType() :
    acceleration("acceleration", TfToken::Immortal),
    alwaysUpdateEnabled("alwaysUpdateEnabled", TfToken::Immortal),
    Asynchronous("Asynchronous", TfToken::Immortal),
    average("average", TfToken::Immortal),
    BitsPerPixel16("BitsPerPixel16", TfToken::Immortal),
    BitsPerPixel32("BitsPerPixel32", TfToken::Immortal),
    BitsPerPixel8("BitsPerPixel8", TfToken::Immortal),
    brakes0("brakes0", TfToken::Immortal),
    brakes1("brakes1", TfToken::Immortal),
    collisionmeshes("collisionmeshes", TfToken::Immortal),
    constrained("constrained", TfToken::Immortal),
    contactOffset("contactOffset", TfToken::Immortal),
    convexDecomposition("convexDecomposition", TfToken::Immortal),
    convexHull("convexHull", TfToken::Immortal),
    defaultFrictionValue("defaultFrictionValue", TfToken::Immortal),
    Disabled("Disabled", TfToken::Immortal),
    drive("drive", TfToken::Immortal),
    easy("easy", TfToken::Immortal),
    enableCCD("enableCCD", TfToken::Immortal),
    flood("flood", TfToken::Immortal),
    fluidRestOffset("fluidRestOffset", TfToken::Immortal),
    force("force", TfToken::Immortal),
    frictionValues("frictionValues", TfToken::Immortal),
    globalSelfCollisionEnabled("globalSelfCollisionEnabled", TfToken::Immortal),
    GPU("GPU", TfToken::Immortal),
    groundMaterials("groundMaterials", TfToken::Immortal),
    max("max", TfToken::Immortal),
    maxDepenetrationVelocity("maxDepenetrationVelocity", TfToken::Immortal),
    maxNeighborhood("maxNeighborhood", TfToken::Immortal),
    maxVelocity("maxVelocity", TfToken::Immortal),
    MBP("MBP", TfToken::Immortal),
    min("min", TfToken::Immortal),
    multiply("multiply", TfToken::Immortal),
    negX("negX", TfToken::Immortal),
    negY("negY", TfToken::Immortal),
    negZ("negZ", TfToken::Immortal),
    neighborhoodScale("neighborhoodScale", TfToken::Immortal),
    nonParticleCollisionEnabled("nonParticleCollisionEnabled", TfToken::Immortal),
    particleContactOffset("particleContactOffset", TfToken::Immortal),
    particleSystemEnabled("particleSystemEnabled", TfToken::Immortal),
    patch("patch", TfToken::Immortal),
    PCM("PCM", TfToken::Immortal),
    PGS("PGS", TfToken::Immortal),
    physicsBody0Indices("physics:body0Indices", TfToken::Immortal),
    physicsBody0s("physics:body0s", TfToken::Immortal),
    physicsBody1Indices("physics:body1Indices", TfToken::Immortal),
    physicsBody1s("physics:body1s", TfToken::Immortal),
    physicsGearRatio("physics:gearRatio", TfToken::Immortal),
    physicsHinge("physics:hinge", TfToken::Immortal),
    physicsHinge0("physics:hinge0", TfToken::Immortal),
    physicsHinge1("physics:hinge1", TfToken::Immortal),
    physicsLocalPos0s("physics:localPos0s", TfToken::Immortal),
    physicsLocalPos1s("physics:localPos1s", TfToken::Immortal),
    physicsLocalRot0s("physics:localRot0s", TfToken::Immortal),
    physicsLocalRot1s("physics:localRot1s", TfToken::Immortal),
    physicsPrismatic("physics:prismatic", TfToken::Immortal),
    physicsProtoIndices("physics:protoIndices", TfToken::Immortal),
    physicsPrototypes("physics:prototypes", TfToken::Immortal),
    physicsRatio("physics:ratio", TfToken::Immortal),
    physxArticulationArticulationEnabled("physxArticulation:articulationEnabled", TfToken::Immortal),
    physxArticulationEnabledSelfCollisions("physxArticulation:enabledSelfCollisions", TfToken::Immortal),
    physxArticulationSleepThreshold("physxArticulation:sleepThreshold", TfToken::Immortal),
    physxArticulationSolverPositionIterationCount("physxArticulation:solverPositionIterationCount", TfToken::Immortal),
    physxArticulationSolverVelocityIterationCount("physxArticulation:solverVelocityIterationCount", TfToken::Immortal),
    physxArticulationStabilizationThreshold("physxArticulation:stabilizationThreshold", TfToken::Immortal),
    physxAutoDeformableAttachmentAttachable0("physxAutoDeformableAttachment:attachable0", TfToken::Immortal),
    physxAutoDeformableAttachmentAttachable1("physxAutoDeformableAttachment:attachable1", TfToken::Immortal),
    physxAutoDeformableAttachmentCollisionFilteringOffset("physxAutoDeformableAttachment:collisionFilteringOffset", TfToken::Immortal),
    physxAutoDeformableAttachmentDeformableVertexOverlapOffset("physxAutoDeformableAttachment:deformableVertexOverlapOffset", TfToken::Immortal),
    physxAutoDeformableAttachmentEnableCollisionFiltering("physxAutoDeformableAttachment:enableCollisionFiltering", TfToken::Immortal),
    physxAutoDeformableAttachmentEnableDeformableFilteringPairs("physxAutoDeformableAttachment:enableDeformableFilteringPairs", TfToken::Immortal),
    physxAutoDeformableAttachmentEnableDeformableVertexAttachments("physxAutoDeformableAttachment:enableDeformableVertexAttachments", TfToken::Immortal),
    physxAutoDeformableAttachmentEnableRigidSurfaceAttachments("physxAutoDeformableAttachment:enableRigidSurfaceAttachments", TfToken::Immortal),
    physxAutoDeformableAttachmentMaskShapes("physxAutoDeformableAttachment:maskShapes", TfToken::Immortal),
    physxAutoDeformableAttachmentRigidSurfaceSamplingDistance("physxAutoDeformableAttachment:rigidSurfaceSamplingDistance", TfToken::Immortal),
    physxCameraSubject("physxCamera:subject", TfToken::Immortal),
    physxCharacterControllerClimbingMode("physxCharacterController:climbingMode", TfToken::Immortal),
    physxCharacterControllerContactOffset("physxCharacterController:contactOffset", TfToken::Immortal),
    physxCharacterControllerInvisibleWallHeight("physxCharacterController:invisibleWallHeight", TfToken::Immortal),
    physxCharacterControllerMaxJumpHeight("physxCharacterController:maxJumpHeight", TfToken::Immortal),
    physxCharacterControllerMoveTarget("physxCharacterController:moveTarget", TfToken::Immortal),
    physxCharacterControllerNonWalkableMode("physxCharacterController:nonWalkableMode", TfToken::Immortal),
    physxCharacterControllerScaleCoeff("physxCharacterController:scaleCoeff", TfToken::Immortal),
    physxCharacterControllerSimulationOwner("physxCharacterController:simulationOwner", TfToken::Immortal),
    physxCharacterControllerSlopeLimit("physxCharacterController:slopeLimit", TfToken::Immortal),
    physxCharacterControllerStepOffset("physxCharacterController:stepOffset", TfToken::Immortal),
    physxCharacterControllerUpAxis("physxCharacterController:upAxis", TfToken::Immortal),
    physxCharacterControllerVolumeGrowth("physxCharacterController:volumeGrowth", TfToken::Immortal),
    physxCollisionContactOffset("physxCollision:contactOffset", TfToken::Immortal),
    physxCollisionCustomGeometry("physxCollisionCustomGeometry", TfToken::Immortal),
    physxCollisionMinTorsionalPatchRadius("physxCollision:minTorsionalPatchRadius", TfToken::Immortal),
    physxCollisionRestOffset("physxCollision:restOffset", TfToken::Immortal),
    physxCollisionTorsionalPatchRadius("physxCollision:torsionalPatchRadius", TfToken::Immortal),
    physxContactReportReportPairs("physxContactReport:reportPairs", TfToken::Immortal),
    physxContactReportThreshold("physxContactReport:threshold", TfToken::Immortal),
    physxConvexDecompositionCollisionErrorPercentage("physxConvexDecompositionCollision:errorPercentage", TfToken::Immortal),
    physxConvexDecompositionCollisionHullVertexLimit("physxConvexDecompositionCollision:hullVertexLimit", TfToken::Immortal),
    physxConvexDecompositionCollisionMaxConvexHulls("physxConvexDecompositionCollision:maxConvexHulls", TfToken::Immortal),
    physxConvexDecompositionCollisionMinThickness("physxConvexDecompositionCollision:minThickness", TfToken::Immortal),
    physxConvexDecompositionCollisionShrinkWrap("physxConvexDecompositionCollision:shrinkWrap", TfToken::Immortal),
    physxConvexDecompositionCollisionVoxelResolution("physxConvexDecompositionCollision:voxelResolution", TfToken::Immortal),
    physxConvexHullCollisionHullVertexLimit("physxConvexHullCollision:hullVertexLimit", TfToken::Immortal),
    physxConvexHullCollisionMinThickness("physxConvexHullCollision:minThickness", TfToken::Immortal),
    physxCookedData("physxCookedData", TfToken::Immortal),
    physxCookedData_MultipleApplyTemplate_Buffer("physxCookedData:__INSTANCE_NAME__:buffer", TfToken::Immortal),
    physxDeformableBodyAutoDeformableBodyEnabled("physxDeformableBody:autoDeformableBodyEnabled", TfToken::Immortal),
    physxDeformableBodyAutoDeformableMeshSimplificationEnabled("physxDeformableBody:autoDeformableMeshSimplificationEnabled", TfToken::Immortal),
    physxDeformableBodyCollisionIterationMultiplier("physxDeformableBody:collisionIterationMultiplier", TfToken::Immortal),
    physxDeformableBodyCollisionPairUpdateFrequency("physxDeformableBody:collisionPairUpdateFrequency", TfToken::Immortal),
    physxDeformableBodyCookingSourceMesh("physxDeformableBody:cookingSourceMesh", TfToken::Immortal),
    physxDeformableBodyDisableGravity("physxDeformableBody:disableGravity", TfToken::Immortal),
    physxDeformableBodyEnableSpeculativeCCD("physxDeformableBody:enableSpeculativeCCD", TfToken::Immortal),
    physxDeformableBodyForceConforming("physxDeformableBody:forceConforming", TfToken::Immortal),
    physxDeformableBodyLinearDamping("physxDeformableBody:linearDamping", TfToken::Immortal),
    physxDeformableBodyMaxDepenetrationVelocity("physxDeformableBody:maxDepenetrationVelocity", TfToken::Immortal),
    physxDeformableBodyMaxLinearVelocity("physxDeformableBody:maxLinearVelocity", TfToken::Immortal),
    physxDeformableBodyRemeshingEnabled("physxDeformableBody:remeshingEnabled", TfToken::Immortal),
    physxDeformableBodyRemeshingResolution("physxDeformableBody:remeshingResolution", TfToken::Immortal),
    physxDeformableBodyResolution("physxDeformableBody:resolution", TfToken::Immortal),
    physxDeformableBodySelfCollision("physxDeformableBody:selfCollision", TfToken::Immortal),
    physxDeformableBodySelfCollisionFilterDistance("physxDeformableBody:selfCollisionFilterDistance", TfToken::Immortal),
    physxDeformableBodySettlingDamping("physxDeformableBody:settlingDamping", TfToken::Immortal),
    physxDeformableBodySettlingThreshold("physxDeformableBody:settlingThreshold", TfToken::Immortal),
    physxDeformableBodySleepThreshold("physxDeformableBody:sleepThreshold", TfToken::Immortal),
    physxDeformableBodySolverPositionIterationCount("physxDeformableBody:solverPositionIterationCount", TfToken::Immortal),
    physxDeformableBodyTargetTriangleCount("physxDeformableBody:targetTriangleCount", TfToken::Immortal),
    physxDeformableMaterialBendDamping("physxDeformableMaterial:bendDamping", TfToken::Immortal),
    physxDeformableMaterialElasticityDamping("physxDeformableMaterial:elasticityDamping", TfToken::Immortal),
    physxDiffuseParticlesAirDrag("physxDiffuseParticles:airDrag", TfToken::Immortal),
    physxDiffuseParticlesBubbleDrag("physxDiffuseParticles:bubbleDrag", TfToken::Immortal),
    physxDiffuseParticlesBuoyancy("physxDiffuseParticles:buoyancy", TfToken::Immortal),
    physxDiffuseParticlesCollisionDecay("physxDiffuseParticles:collisionDecay", TfToken::Immortal),
    physxDiffuseParticlesDiffuseParticlesEnabled("physxDiffuseParticles:diffuseParticlesEnabled", TfToken::Immortal),
    physxDiffuseParticlesDivergenceWeight("physxDiffuseParticles:divergenceWeight", TfToken::Immortal),
    physxDiffuseParticlesKineticEnergyWeight("physxDiffuseParticles:kineticEnergyWeight", TfToken::Immortal),
    physxDiffuseParticlesLifetime("physxDiffuseParticles:lifetime", TfToken::Immortal),
    physxDiffuseParticlesMaxDiffuseParticleMultiplier("physxDiffuseParticles:maxDiffuseParticleMultiplier", TfToken::Immortal),
    physxDiffuseParticlesPressureWeight("physxDiffuseParticles:pressureWeight", TfToken::Immortal),
    physxDiffuseParticlesThreshold("physxDiffuseParticles:threshold", TfToken::Immortal),
    physxDiffuseParticlesUseAccurateVelocity("physxDiffuseParticles:useAccurateVelocity", TfToken::Immortal),
    physxDrivePerformanceEnvelope("physxDrivePerformanceEnvelope", TfToken::Immortal),
    physxDrivePerformanceEnvelope_MultipleApplyTemplate_MaxActuatorVelocity("physxDrivePerformanceEnvelope:__INSTANCE_NAME__:maxActuatorVelocity", TfToken::Immortal),
    physxDrivePerformanceEnvelope_MultipleApplyTemplate_SpeedEffortGradient("physxDrivePerformanceEnvelope:__INSTANCE_NAME__:speedEffortGradient", TfToken::Immortal),
    physxDrivePerformanceEnvelope_MultipleApplyTemplate_VelocityDependentResistance("physxDrivePerformanceEnvelope:__INSTANCE_NAME__:velocityDependentResistance", TfToken::Immortal),
    physxDroneCameraFeedForwardVelocityGain("physxDroneCamera:feedForwardVelocityGain", TfToken::Immortal),
    physxDroneCameraFollowDistance("physxDroneCamera:followDistance", TfToken::Immortal),
    physxDroneCameraFollowHeight("physxDroneCamera:followHeight", TfToken::Immortal),
    physxDroneCameraHorizontalVelocityGain("physxDroneCamera:horizontalVelocityGain", TfToken::Immortal),
    physxDroneCameraMaxDistance("physxDroneCamera:maxDistance", TfToken::Immortal),
    physxDroneCameraMaxSpeed("physxDroneCamera:maxSpeed", TfToken::Immortal),
    physxDroneCameraPositionOffset("physxDroneCamera:positionOffset", TfToken::Immortal),
    physxDroneCameraRotationFilterTimeConstant("physxDroneCamera:rotationFilterTimeConstant", TfToken::Immortal),
    physxDroneCameraVelocityFilterTimeConstant("physxDroneCamera:velocityFilterTimeConstant", TfToken::Immortal),
    physxDroneCameraVerticalVelocityGain("physxDroneCamera:verticalVelocityGain", TfToken::Immortal),
    physxFollowCameraCameraPositionTimeConstant("physxFollowCamera:cameraPositionTimeConstant", TfToken::Immortal),
    physxFollowCameraFollowMaxDistance("physxFollowCamera:followMaxDistance", TfToken::Immortal),
    physxFollowCameraFollowMaxSpeed("physxFollowCamera:followMaxSpeed", TfToken::Immortal),
    physxFollowCameraFollowMinDistance("physxFollowCamera:followMinDistance", TfToken::Immortal),
    physxFollowCameraFollowMinSpeed("physxFollowCamera:followMinSpeed", TfToken::Immortal),
    physxFollowCameraFollowTurnRateGain("physxFollowCamera:followTurnRateGain", TfToken::Immortal),
    physxFollowCameraLookAheadMaxSpeed("physxFollowCamera:lookAheadMaxSpeed", TfToken::Immortal),
    physxFollowCameraLookAheadMinDistance("physxFollowCamera:lookAheadMinDistance", TfToken::Immortal),
    physxFollowCameraLookAheadMinSpeed("physxFollowCamera:lookAheadMinSpeed", TfToken::Immortal),
    physxFollowCameraLookAheadTurnRateGain("physxFollowCamera:lookAheadTurnRateGain", TfToken::Immortal),
    physxFollowCameraLookPositionHeight("physxFollowCamera:lookPositionHeight", TfToken::Immortal),
    physxFollowCameraLookPositionTimeConstant("physxFollowCamera:lookPositionTimeConstant", TfToken::Immortal),
    physxFollowCameraPitchAngle("physxFollowCamera:pitchAngle", TfToken::Immortal),
    physxFollowCameraPitchAngleTimeConstant("physxFollowCamera:pitchAngleTimeConstant", TfToken::Immortal),
    physxFollowCameraPositionOffset("physxFollowCamera:positionOffset", TfToken::Immortal),
    physxFollowCameraSlowPitchAngleSpeed("physxFollowCamera:slowPitchAngleSpeed", TfToken::Immortal),
    physxFollowCameraSlowSpeedPitchAngleScale("physxFollowCamera:slowSpeedPitchAngleScale", TfToken::Immortal),
    physxFollowCameraVelocityNormalMinSpeed("physxFollowCamera:velocityNormalMinSpeed", TfToken::Immortal),
    physxFollowCameraYawAngle("physxFollowCamera:yawAngle", TfToken::Immortal),
    physxFollowCameraYawRateTimeConstant("physxFollowCamera:yawRateTimeConstant", TfToken::Immortal),
    physxFollowFollowCameraLookAheadMaxDistance("physxFollowFollowCamera:lookAheadMaxDistance", TfToken::Immortal),
    physxFollowLookCameraDownHillGroundAngle("physxFollowLookCamera:downHillGroundAngle", TfToken::Immortal),
    physxFollowLookCameraDownHillGroundPitch("physxFollowLookCamera:downHillGroundPitch", TfToken::Immortal),
    physxFollowLookCameraFollowReverseDistance("physxFollowLookCamera:followReverseDistance", TfToken::Immortal),
    physxFollowLookCameraFollowReverseSpeed("physxFollowLookCamera:followReverseSpeed", TfToken::Immortal),
    physxFollowLookCameraUpHillGroundAngle("physxFollowLookCamera:upHillGroundAngle", TfToken::Immortal),
    physxFollowLookCameraUpHillGroundPitch("physxFollowLookCamera:upHillGroundPitch", TfToken::Immortal),
    physxFollowLookCameraVelocityBlendTimeConstant("physxFollowLookCamera:velocityBlendTimeConstant", TfToken::Immortal),
    physxForceForce("physxForce:force", TfToken::Immortal),
    physxForceForceEnabled("physxForce:forceEnabled", TfToken::Immortal),
    physxForceMode("physxForce:mode", TfToken::Immortal),
    physxForceTorque("physxForce:torque", TfToken::Immortal),
    physxForceWorldFrameEnabled("physxForce:worldFrameEnabled", TfToken::Immortal),
    physxJointArmature("physxJoint:armature", TfToken::Immortal),
    physxJointAxis("physxJointAxis", TfToken::Immortal),
    physxJointAxis_MultipleApplyTemplate_Armature("physxJointAxis:__INSTANCE_NAME__:armature", TfToken::Immortal),
    physxJointAxis_MultipleApplyTemplate_DynamicFrictionEffort("physxJointAxis:__INSTANCE_NAME__:dynamicFrictionEffort", TfToken::Immortal),
    physxJointAxis_MultipleApplyTemplate_MaxJointVelocity("physxJointAxis:__INSTANCE_NAME__:maxJointVelocity", TfToken::Immortal),
    physxJointAxis_MultipleApplyTemplate_StaticFrictionEffort("physxJointAxis:__INSTANCE_NAME__:staticFrictionEffort", TfToken::Immortal),
    physxJointAxis_MultipleApplyTemplate_ViscousFrictionCoefficient("physxJointAxis:__INSTANCE_NAME__:viscousFrictionCoefficient", TfToken::Immortal),
    physxJointJointFriction("physxJoint:jointFriction", TfToken::Immortal),
    physxJointMaxJointVelocity("physxJoint:maxJointVelocity", TfToken::Immortal),
    physxLimit("physxLimit", TfToken::Immortal),
    physxLimit_MultipleApplyTemplate_BounceThreshold("physxLimit:__INSTANCE_NAME__:bounceThreshold", TfToken::Immortal),
    physxLimit_MultipleApplyTemplate_Damping("physxLimit:__INSTANCE_NAME__:damping", TfToken::Immortal),
    physxLimit_MultipleApplyTemplate_Restitution("physxLimit:__INSTANCE_NAME__:restitution", TfToken::Immortal),
    physxLimit_MultipleApplyTemplate_Stiffness("physxLimit:__INSTANCE_NAME__:stiffness", TfToken::Immortal),
    physxMaterialCompliantContactAccelerationSpring("physxMaterial:compliantContactAccelerationSpring", TfToken::Immortal),
    physxMaterialCompliantContactDamping("physxMaterial:compliantContactDamping", TfToken::Immortal),
    physxMaterialCompliantContactStiffness("physxMaterial:compliantContactStiffness", TfToken::Immortal),
    physxMaterialDampingCombineMode("physxMaterial:dampingCombineMode", TfToken::Immortal),
    physxMaterialFrictionCombineMode("physxMaterial:frictionCombineMode", TfToken::Immortal),
    physxMaterialRestitutionCombineMode("physxMaterial:restitutionCombineMode", TfToken::Immortal),
    physxMimicJoint("physxMimicJoint", TfToken::Immortal),
    physxMimicJoint_MultipleApplyTemplate_DampingRatio("physxMimicJoint:__INSTANCE_NAME__:dampingRatio", TfToken::Immortal),
    physxMimicJoint_MultipleApplyTemplate_Gearing("physxMimicJoint:__INSTANCE_NAME__:gearing", TfToken::Immortal),
    physxMimicJoint_MultipleApplyTemplate_NaturalFrequency("physxMimicJoint:__INSTANCE_NAME__:naturalFrequency", TfToken::Immortal),
    physxMimicJoint_MultipleApplyTemplate_Offset("physxMimicJoint:__INSTANCE_NAME__:offset", TfToken::Immortal),
    physxMimicJoint_MultipleApplyTemplate_ReferenceJoint("physxMimicJoint:__INSTANCE_NAME__:referenceJoint", TfToken::Immortal),
    physxMimicJoint_MultipleApplyTemplate_ReferenceJointAxis("physxMimicJoint:__INSTANCE_NAME__:referenceJointAxis", TfToken::Immortal),
    physxParticleAnisotropyMax("physxParticleAnisotropy:max", TfToken::Immortal),
    physxParticleAnisotropyMin("physxParticleAnisotropy:min", TfToken::Immortal),
    physxParticleAnisotropyParticleAnisotropyEnabled("physxParticleAnisotropy:particleAnisotropyEnabled", TfToken::Immortal),
    physxParticleAnisotropyScale("physxParticleAnisotropy:scale", TfToken::Immortal),
    physxParticleFluid("physxParticle:fluid", TfToken::Immortal),
    physxParticleIsosurfaceGridFilteringPasses("physxParticleIsosurface:gridFilteringPasses", TfToken::Immortal),
    physxParticleIsosurfaceGridSmoothingRadius("physxParticleIsosurface:gridSmoothingRadius", TfToken::Immortal),
    physxParticleIsosurfaceGridSpacing("physxParticleIsosurface:gridSpacing", TfToken::Immortal),
    physxParticleIsosurfaceIsosurfaceEnabled("physxParticleIsosurface:isosurfaceEnabled", TfToken::Immortal),
    physxParticleIsosurfaceMaxSubgrids("physxParticleIsosurface:maxSubgrids", TfToken::Immortal),
    physxParticleIsosurfaceMaxTriangles("physxParticleIsosurface:maxTriangles", TfToken::Immortal),
    physxParticleIsosurfaceMaxVertices("physxParticleIsosurface:maxVertices", TfToken::Immortal),
    physxParticleIsosurfaceNumMeshNormalSmoothingPasses("physxParticleIsosurface:numMeshNormalSmoothingPasses", TfToken::Immortal),
    physxParticleIsosurfaceNumMeshSmoothingPasses("physxParticleIsosurface:numMeshSmoothingPasses", TfToken::Immortal),
    physxParticleIsosurfaceSurfaceDistance("physxParticleIsosurface:surfaceDistance", TfToken::Immortal),
    physxParticleParticleEnabled("physxParticle:particleEnabled", TfToken::Immortal),
    physxParticleParticleGroup("physxParticle:particleGroup", TfToken::Immortal),
    physxParticleParticleSystem("physxParticle:particleSystem", TfToken::Immortal),
    physxParticleSamplingMaxSamples("physxParticleSampling:maxSamples", TfToken::Immortal),
    physxParticleSamplingParticles("physxParticleSampling:particles", TfToken::Immortal),
    physxParticleSamplingSamplingDistance("physxParticleSampling:samplingDistance", TfToken::Immortal),
    physxParticleSamplingVolume("physxParticleSampling:volume", TfToken::Immortal),
    physxParticleSelfCollision("physxParticle:selfCollision", TfToken::Immortal),
    physxParticleSimulationPoints("physxParticle:simulationPoints", TfToken::Immortal),
    physxParticleSmoothingParticleSmoothingEnabled("physxParticleSmoothing:particleSmoothingEnabled", TfToken::Immortal),
    physxParticleSmoothingStrength("physxParticleSmoothing:strength", TfToken::Immortal),
    physxPBDMaterialAdhesion("physxPBDMaterial:adhesion", TfToken::Immortal),
    physxPBDMaterialAdhesionOffsetScale("physxPBDMaterial:adhesionOffsetScale", TfToken::Immortal),
    physxPBDMaterialCflCoefficient("physxPBDMaterial:cflCoefficient", TfToken::Immortal),
    physxPBDMaterialCohesion("physxPBDMaterial:cohesion", TfToken::Immortal),
    physxPBDMaterialDamping("physxPBDMaterial:damping", TfToken::Immortal),
    physxPBDMaterialDensity("physxPBDMaterial:density", TfToken::Immortal),
    physxPBDMaterialFriction("physxPBDMaterial:friction", TfToken::Immortal),
    physxPBDMaterialGravityScale("physxPBDMaterial:gravityScale", TfToken::Immortal),
    physxPBDMaterialParticleAdhesionScale("physxPBDMaterial:particleAdhesionScale", TfToken::Immortal),
    physxPBDMaterialParticleFrictionScale("physxPBDMaterial:particleFrictionScale", TfToken::Immortal),
    physxPBDMaterialSurfaceTension("physxPBDMaterial:surfaceTension", TfToken::Immortal),
    physxPBDMaterialViscosity("physxPBDMaterial:viscosity", TfToken::Immortal),
    physxPBDMaterialVorticityConfinement("physxPBDMaterial:vorticityConfinement", TfToken::Immortal),
    physxPhysicsDistanceJointSpringDamping("physxPhysicsDistanceJoint:springDamping", TfToken::Immortal),
    physxPhysicsDistanceJointSpringEnabled("physxPhysicsDistanceJoint:springEnabled", TfToken::Immortal),
    physxPhysicsDistanceJointSpringStiffness("physxPhysicsDistanceJoint:springStiffness", TfToken::Immortal),
    physxRigidBodyAngularDamping("physxRigidBody:angularDamping", TfToken::Immortal),
    physxRigidBodyCfmScale("physxRigidBody:cfmScale", TfToken::Immortal),
    physxRigidBodyContactSlopCoefficient("physxRigidBody:contactSlopCoefficient", TfToken::Immortal),
    physxRigidBodyDisableGravity("physxRigidBody:disableGravity", TfToken::Immortal),
    physxRigidBodyEnableCCD("physxRigidBody:enableCCD", TfToken::Immortal),
    physxRigidBodyEnableGyroscopicForces("physxRigidBody:enableGyroscopicForces", TfToken::Immortal),
    physxRigidBodyEnableSpeculativeCCD("physxRigidBody:enableSpeculativeCCD", TfToken::Immortal),
    physxRigidBodyLinearDamping("physxRigidBody:linearDamping", TfToken::Immortal),
    physxRigidBodyLockedPosAxis("physxRigidBody:lockedPosAxis", TfToken::Immortal),
    physxRigidBodyLockedRotAxis("physxRigidBody:lockedRotAxis", TfToken::Immortal),
    physxRigidBodyMaxAngularVelocity("physxRigidBody:maxAngularVelocity", TfToken::Immortal),
    physxRigidBodyMaxContactImpulse("physxRigidBody:maxContactImpulse", TfToken::Immortal),
    physxRigidBodyMaxDepenetrationVelocity("physxRigidBody:maxDepenetrationVelocity", TfToken::Immortal),
    physxRigidBodyMaxLinearVelocity("physxRigidBody:maxLinearVelocity", TfToken::Immortal),
    physxRigidBodyRetainAccelerations("physxRigidBody:retainAccelerations", TfToken::Immortal),
    physxRigidBodySleepThreshold("physxRigidBody:sleepThreshold", TfToken::Immortal),
    physxRigidBodySolveContact("physxRigidBody:solveContact", TfToken::Immortal),
    physxRigidBodySolverPositionIterationCount("physxRigidBody:solverPositionIterationCount", TfToken::Immortal),
    physxRigidBodySolverVelocityIterationCount("physxRigidBody:solverVelocityIterationCount", TfToken::Immortal),
    physxRigidBodyStabilizationThreshold("physxRigidBody:stabilizationThreshold", TfToken::Immortal),
    physxSceneBounceThreshold("physxScene:bounceThreshold", TfToken::Immortal),
    physxSceneBroadphaseType("physxScene:broadphaseType", TfToken::Immortal),
    physxSceneCollisionSystem("physxScene:collisionSystem", TfToken::Immortal),
    physxSceneDisableSleeping("physxScene:disableSleeping", TfToken::Immortal),
    physxSceneEnableCCD("physxScene:enableCCD", TfToken::Immortal),
    physxSceneEnableEnhancedDeterminism("physxScene:enableEnhancedDeterminism", TfToken::Immortal),
    physxSceneEnableExternalForcesEveryIteration("physxScene:enableExternalForcesEveryIteration", TfToken::Immortal),
    physxSceneEnableGPUDynamics("physxScene:enableGPUDynamics", TfToken::Immortal),
    physxSceneEnableSceneQuerySupport("physxScene:enableSceneQuerySupport", TfToken::Immortal),
    physxSceneEnableStabilization("physxScene:enableStabilization", TfToken::Immortal),
    physxSceneFrictionCorrelationDistance("physxScene:frictionCorrelationDistance", TfToken::Immortal),
    physxSceneFrictionOffsetThreshold("physxScene:frictionOffsetThreshold", TfToken::Immortal),
    physxSceneFrictionType("physxScene:frictionType", TfToken::Immortal),
    physxSceneGpuCollisionStackSize("physxScene:gpuCollisionStackSize", TfToken::Immortal),
    physxSceneGpuFoundLostAggregatePairsCapacity("physxScene:gpuFoundLostAggregatePairsCapacity", TfToken::Immortal),
    physxSceneGpuFoundLostPairsCapacity("physxScene:gpuFoundLostPairsCapacity", TfToken::Immortal),
    physxSceneGpuHeapCapacity("physxScene:gpuHeapCapacity", TfToken::Immortal),
    physxSceneGpuMaxDeformableSurfaceContacts("physxScene:gpuMaxDeformableSurfaceContacts", TfToken::Immortal),
    physxSceneGpuMaxDeformableVolumeContacts("physxScene:gpuMaxDeformableVolumeContacts", TfToken::Immortal),
    physxSceneGpuMaxNumPartitions("physxScene:gpuMaxNumPartitions", TfToken::Immortal),
    physxSceneGpuMaxParticleContacts("physxScene:gpuMaxParticleContacts", TfToken::Immortal),
    physxSceneGpuMaxRigidContactCount("physxScene:gpuMaxRigidContactCount", TfToken::Immortal),
    physxSceneGpuMaxRigidPatchCount("physxScene:gpuMaxRigidPatchCount", TfToken::Immortal),
    physxSceneGpuTempBufferCapacity("physxScene:gpuTempBufferCapacity", TfToken::Immortal),
    physxSceneGpuTotalAggregatePairsCapacity("physxScene:gpuTotalAggregatePairsCapacity", TfToken::Immortal),
    physxSceneInvertCollisionGroupFilter("physxScene:invertCollisionGroupFilter", TfToken::Immortal),
    physxSceneMaxBiasCoefficient("physxScene:maxBiasCoefficient", TfToken::Immortal),
    physxSceneMaxPositionIterationCount("physxScene:maxPositionIterationCount", TfToken::Immortal),
    physxSceneMaxVelocityIterationCount("physxScene:maxVelocityIterationCount", TfToken::Immortal),
    physxSceneMinPositionIterationCount("physxScene:minPositionIterationCount", TfToken::Immortal),
    physxSceneMinVelocityIterationCount("physxScene:minVelocityIterationCount", TfToken::Immortal),
    physxSceneQuasistaticEnableQuasistatic("physxSceneQuasistatic:enableQuasistatic", TfToken::Immortal),
    physxSceneReportKinematicKinematicPairs("physxScene:reportKinematicKinematicPairs", TfToken::Immortal),
    physxSceneReportKinematicStaticPairs("physxScene:reportKinematicStaticPairs", TfToken::Immortal),
    physxSceneSolveArticulationContactLast("physxScene:solveArticulationContactLast", TfToken::Immortal),
    physxSceneSolverType("physxScene:solverType", TfToken::Immortal),
    physxSceneTimeStepsPerSecond("physxScene:timeStepsPerSecond", TfToken::Immortal),
    physxSceneUpdateType("physxScene:updateType", TfToken::Immortal),
    physxSDFMeshCollisionSdfBitsPerSubgridPixel("physxSDFMeshCollision:sdfBitsPerSubgridPixel", TfToken::Immortal),
    physxSDFMeshCollisionSdfEnableRemeshing("physxSDFMeshCollision:sdfEnableRemeshing", TfToken::Immortal),
    physxSDFMeshCollisionSdfMargin("physxSDFMeshCollision:sdfMargin", TfToken::Immortal),
    physxSDFMeshCollisionSdfNarrowBandThickness("physxSDFMeshCollision:sdfNarrowBandThickness", TfToken::Immortal),
    physxSDFMeshCollisionSdfResolution("physxSDFMeshCollision:sdfResolution", TfToken::Immortal),
    physxSDFMeshCollisionSdfSubgridResolution("physxSDFMeshCollision:sdfSubgridResolution", TfToken::Immortal),
    physxSDFMeshCollisionSdfTriangleCountReductionFactor("physxSDFMeshCollision:sdfTriangleCountReductionFactor", TfToken::Immortal),
    physxSphereFillCollisionFillMode("physxSphereFillCollision:fillMode", TfToken::Immortal),
    physxSphereFillCollisionMaxSpheres("physxSphereFillCollision:maxSpheres", TfToken::Immortal),
    physxSphereFillCollisionSeedCount("physxSphereFillCollision:seedCount", TfToken::Immortal),
    physxSphereFillCollisionVoxelResolution("physxSphereFillCollision:voxelResolution", TfToken::Immortal),
    physxSplinesSurfaceVelocitySurfaceVelocityCurve("physxSplinesSurfaceVelocity:surfaceVelocityCurve", TfToken::Immortal),
    physxSplinesSurfaceVelocitySurfaceVelocityEnabled("physxSplinesSurfaceVelocity:surfaceVelocityEnabled", TfToken::Immortal),
    physxSplinesSurfaceVelocitySurfaceVelocityForceBased("physxSplinesSurfaceVelocity:surfaceVelocityForceBased", TfToken::Immortal),
    physxSplinesSurfaceVelocitySurfaceVelocityMagnitude("physxSplinesSurfaceVelocity:surfaceVelocityMagnitude", TfToken::Immortal),
    physxSurfaceVelocitySurfaceAngularVelocity("physxSurfaceVelocity:surfaceAngularVelocity", TfToken::Immortal),
    physxSurfaceVelocitySurfaceVelocity("physxSurfaceVelocity:surfaceVelocity", TfToken::Immortal),
    physxSurfaceVelocitySurfaceVelocityEnabled("physxSurfaceVelocity:surfaceVelocityEnabled", TfToken::Immortal),
    physxSurfaceVelocitySurfaceVelocityLocalSpace("physxSurfaceVelocity:surfaceVelocityLocalSpace", TfToken::Immortal),
    physxTendon("physxTendon", TfToken::Immortal),
    physxTendon_MultipleApplyTemplate_Damping("physxTendon:__INSTANCE_NAME__:damping", TfToken::Immortal),
    physxTendon_MultipleApplyTemplate_ForceCoefficient("physxTendon:__INSTANCE_NAME__:forceCoefficient", TfToken::Immortal),
    physxTendon_MultipleApplyTemplate_Gearing("physxTendon:__INSTANCE_NAME__:gearing", TfToken::Immortal),
    physxTendon_MultipleApplyTemplate_JointAxis("physxTendon:__INSTANCE_NAME__:jointAxis", TfToken::Immortal),
    physxTendon_MultipleApplyTemplate_LimitStiffness("physxTendon:__INSTANCE_NAME__:limitStiffness", TfToken::Immortal),
    physxTendon_MultipleApplyTemplate_LocalPos("physxTendon:__INSTANCE_NAME__:localPos", TfToken::Immortal),
    physxTendon_MultipleApplyTemplate_LowerLimit("physxTendon:__INSTANCE_NAME__:lowerLimit", TfToken::Immortal),
    physxTendon_MultipleApplyTemplate_Offset("physxTendon:__INSTANCE_NAME__:offset", TfToken::Immortal),
    physxTendon_MultipleApplyTemplate_ParentAttachment("physxTendon:__INSTANCE_NAME__:parentAttachment", TfToken::Immortal),
    physxTendon_MultipleApplyTemplate_ParentLink("physxTendon:__INSTANCE_NAME__:parentLink", TfToken::Immortal),
    physxTendon_MultipleApplyTemplate_RestLength("physxTendon:__INSTANCE_NAME__:restLength", TfToken::Immortal),
    physxTendon_MultipleApplyTemplate_Stiffness("physxTendon:__INSTANCE_NAME__:stiffness", TfToken::Immortal),
    physxTendon_MultipleApplyTemplate_TendonEnabled("physxTendon:__INSTANCE_NAME__:tendonEnabled", TfToken::Immortal),
    physxTendon_MultipleApplyTemplate_UpperLimit("physxTendon:__INSTANCE_NAME__:upperLimit", TfToken::Immortal),
    physxTriangleMeshCollisionWeldTolerance("physxTriangleMeshCollision:weldTolerance", TfToken::Immortal),
    physxTriangleMeshSimplificationCollisionMetric("physxTriangleMeshSimplificationCollision:metric", TfToken::Immortal),
    physxTriangleMeshSimplificationCollisionWeldTolerance("physxTriangleMeshSimplificationCollision:weldTolerance", TfToken::Immortal),
    physxTriggerTriggeredCollisions("physxTrigger:triggeredCollisions", TfToken::Immortal),
    physxVehicleAckermannSteeringMaxSteerAngle("physxVehicleAckermannSteering:maxSteerAngle", TfToken::Immortal),
    physxVehicleAckermannSteeringStrength("physxVehicleAckermannSteering:strength", TfToken::Immortal),
    physxVehicleAckermannSteeringTrackWidth("physxVehicleAckermannSteering:trackWidth", TfToken::Immortal),
    physxVehicleAckermannSteeringWheel0("physxVehicleAckermannSteering:wheel0", TfToken::Immortal),
    physxVehicleAckermannSteeringWheel1("physxVehicleAckermannSteering:wheel1", TfToken::Immortal),
    physxVehicleAckermannSteeringWheelBase("physxVehicleAckermannSteering:wheelBase", TfToken::Immortal),
    physxVehicleAutoGearBoxDownRatios("physxVehicleAutoGearBox:downRatios", TfToken::Immortal),
    physxVehicleAutoGearBoxLatency("physxVehicleAutoGearBox:latency", TfToken::Immortal),
    physxVehicleAutoGearBoxUpRatios("physxVehicleAutoGearBox:upRatios", TfToken::Immortal),
    physxVehicleBrakes("physxVehicleBrakes", TfToken::Immortal),
    physxVehicleBrakes_MultipleApplyTemplate_MaxBrakeTorque("physxVehicleBrakes:__INSTANCE_NAME__:maxBrakeTorque", TfToken::Immortal),
    physxVehicleBrakes_MultipleApplyTemplate_TorqueMultipliers("physxVehicleBrakes:__INSTANCE_NAME__:torqueMultipliers", TfToken::Immortal),
    physxVehicleBrakes_MultipleApplyTemplate_Wheels("physxVehicleBrakes:__INSTANCE_NAME__:wheels", TfToken::Immortal),
    physxVehicleClutchStrength("physxVehicleClutch:strength", TfToken::Immortal),
    physxVehicleContextForwardAxis("physxVehicleContext:forwardAxis", TfToken::Immortal),
    physxVehicleContextLongitudinalAxis("physxVehicleContext:longitudinalAxis", TfToken::Immortal),
    physxVehicleContextUpAxis("physxVehicleContext:upAxis", TfToken::Immortal),
    physxVehicleContextUpdateMode("physxVehicleContext:updateMode", TfToken::Immortal),
    physxVehicleContextVerticalAxis("physxVehicleContext:verticalAxis", TfToken::Immortal),
    physxVehicleControllerAccelerator("physxVehicleController:accelerator", TfToken::Immortal),
    physxVehicleControllerBrake("physxVehicleController:brake", TfToken::Immortal),
    physxVehicleControllerBrake0("physxVehicleController:brake0", TfToken::Immortal),
    physxVehicleControllerBrake1("physxVehicleController:brake1", TfToken::Immortal),
    physxVehicleControllerHandbrake("physxVehicleController:handbrake", TfToken::Immortal),
    physxVehicleControllerSteer("physxVehicleController:steer", TfToken::Immortal),
    physxVehicleControllerSteerLeft("physxVehicleController:steerLeft", TfToken::Immortal),
    physxVehicleControllerSteerRight("physxVehicleController:steerRight", TfToken::Immortal),
    physxVehicleControllerTargetGear("physxVehicleController:targetGear", TfToken::Immortal),
    physxVehicleDrive("physxVehicle:drive", TfToken::Immortal),
    physxVehicleDriveBasicPeakTorque("physxVehicleDriveBasic:peakTorque", TfToken::Immortal),
    physxVehicleDriveStandardAutoGearBox("physxVehicleDriveStandard:autoGearBox", TfToken::Immortal),
    physxVehicleDriveStandardClutch("physxVehicleDriveStandard:clutch", TfToken::Immortal),
    physxVehicleDriveStandardEngine("physxVehicleDriveStandard:engine", TfToken::Immortal),
    physxVehicleDriveStandardGears("physxVehicleDriveStandard:gears", TfToken::Immortal),
    physxVehicleEngineDampingRateFullThrottle("physxVehicleEngine:dampingRateFullThrottle", TfToken::Immortal),
    physxVehicleEngineDampingRateZeroThrottleClutchDisengaged("physxVehicleEngine:dampingRateZeroThrottleClutchDisengaged", TfToken::Immortal),
    physxVehicleEngineDampingRateZeroThrottleClutchEngaged("physxVehicleEngine:dampingRateZeroThrottleClutchEngaged", TfToken::Immortal),
    physxVehicleEngineIdleRotationSpeed("physxVehicleEngine:idleRotationSpeed", TfToken::Immortal),
    physxVehicleEngineMaxRotationSpeed("physxVehicleEngine:maxRotationSpeed", TfToken::Immortal),
    physxVehicleEngineMoi("physxVehicleEngine:moi", TfToken::Immortal),
    physxVehicleEnginePeakTorque("physxVehicleEngine:peakTorque", TfToken::Immortal),
    physxVehicleEngineTorqueCurve("physxVehicleEngine:torqueCurve", TfToken::Immortal),
    physxVehicleGearsRatios("physxVehicleGears:ratios", TfToken::Immortal),
    physxVehicleGearsRatioScale("physxVehicleGears:ratioScale", TfToken::Immortal),
    physxVehicleGearsSwitchTime("physxVehicleGears:switchTime", TfToken::Immortal),
    physxVehicleHighForwardSpeedSubStepCount("physxVehicle:highForwardSpeedSubStepCount", TfToken::Immortal),
    physxVehicleLateralStickyTireDamping("physxVehicle:lateralStickyTireDamping", TfToken::Immortal),
    physxVehicleLateralStickyTireThresholdSpeed("physxVehicle:lateralStickyTireThresholdSpeed", TfToken::Immortal),
    physxVehicleLateralStickyTireThresholdTime("physxVehicle:lateralStickyTireThresholdTime", TfToken::Immortal),
    physxVehicleLimitSuspensionExpansionVelocity("physxVehicle:limitSuspensionExpansionVelocity", TfToken::Immortal),
    physxVehicleLongitudinalStickyTireDamping("physxVehicle:longitudinalStickyTireDamping", TfToken::Immortal),
    physxVehicleLongitudinalStickyTireThresholdSpeed("physxVehicle:longitudinalStickyTireThresholdSpeed", TfToken::Immortal),
    physxVehicleLongitudinalStickyTireThresholdTime("physxVehicle:longitudinalStickyTireThresholdTime", TfToken::Immortal),
    physxVehicleLowForwardSpeedSubStepCount("physxVehicle:lowForwardSpeedSubStepCount", TfToken::Immortal),
    physxVehicleMinActiveLongitudinalSlipDenominator("physxVehicle:minActiveLongitudinalSlipDenominator", TfToken::Immortal),
    physxVehicleMinLateralSlipDenominator("physxVehicle:minLateralSlipDenominator", TfToken::Immortal),
    physxVehicleMinLongitudinalSlipDenominator("physxVehicle:minLongitudinalSlipDenominator", TfToken::Immortal),
    physxVehicleMinPassiveLongitudinalSlipDenominator("physxVehicle:minPassiveLongitudinalSlipDenominator", TfToken::Immortal),
    physxVehicleMultiWheelDifferentialAverageWheelSpeedRatios("physxVehicleMultiWheelDifferential:averageWheelSpeedRatios", TfToken::Immortal),
    physxVehicleMultiWheelDifferentialTorqueRatios("physxVehicleMultiWheelDifferential:torqueRatios", TfToken::Immortal),
    physxVehicleMultiWheelDifferentialWheels("physxVehicleMultiWheelDifferential:wheels", TfToken::Immortal),
    physxVehicleNCR("physxVehicleNCR", TfToken::Immortal),
    physxVehicleNCR_MultipleApplyTemplate_CommandValues("physxVehicleNCR:__INSTANCE_NAME__:commandValues", TfToken::Immortal),
    physxVehicleNCR_MultipleApplyTemplate_SpeedResponses("physxVehicleNCR:__INSTANCE_NAME__:speedResponses", TfToken::Immortal),
    physxVehicleNCR_MultipleApplyTemplate_SpeedResponsesPerCommandValue("physxVehicleNCR:__INSTANCE_NAME__:speedResponsesPerCommandValue", TfToken::Immortal),
    physxVehicleSteeringAngleMultipliers("physxVehicleSteering:angleMultipliers", TfToken::Immortal),
    physxVehicleSteeringMaxSteerAngle("physxVehicleSteering:maxSteerAngle", TfToken::Immortal),
    physxVehicleSteeringWheels("physxVehicleSteering:wheels", TfToken::Immortal),
    physxVehicleSubStepThresholdLongitudinalSpeed("physxVehicle:subStepThresholdLongitudinalSpeed", TfToken::Immortal),
    physxVehicleSuspensionCamberAtMaxCompression("physxVehicleSuspension:camberAtMaxCompression", TfToken::Immortal),
    physxVehicleSuspensionCamberAtMaxDroop("physxVehicleSuspension:camberAtMaxDroop", TfToken::Immortal),
    physxVehicleSuspensionCamberAtRest("physxVehicleSuspension:camberAtRest", TfToken::Immortal),
    physxVehicleSuspensionComplianceSuspensionForceAppPoint("physxVehicleSuspensionCompliance:suspensionForceAppPoint", TfToken::Immortal),
    physxVehicleSuspensionComplianceTireForceAppPoint("physxVehicleSuspensionCompliance:tireForceAppPoint", TfToken::Immortal),
    physxVehicleSuspensionComplianceWheelCamberAngle("physxVehicleSuspensionCompliance:wheelCamberAngle", TfToken::Immortal),
    physxVehicleSuspensionComplianceWheelToeAngle("physxVehicleSuspensionCompliance:wheelToeAngle", TfToken::Immortal),
    physxVehicleSuspensionLineQueryType("physxVehicle:suspensionLineQueryType", TfToken::Immortal),
    physxVehicleSuspensionMaxCompression("physxVehicleSuspension:maxCompression", TfToken::Immortal),
    physxVehicleSuspensionMaxDroop("physxVehicleSuspension:maxDroop", TfToken::Immortal),
    physxVehicleSuspensionSpringDamperRate("physxVehicleSuspension:springDamperRate", TfToken::Immortal),
    physxVehicleSuspensionSpringStrength("physxVehicleSuspension:springStrength", TfToken::Immortal),
    physxVehicleSuspensionSprungMass("physxVehicleSuspension:sprungMass", TfToken::Immortal),
    physxVehicleSuspensionTravelDistance("physxVehicleSuspension:travelDistance", TfToken::Immortal),
    physxVehicleTankControllerThrust0("physxVehicleTankController:thrust0", TfToken::Immortal),
    physxVehicleTankControllerThrust1("physxVehicleTankController:thrust1", TfToken::Immortal),
    physxVehicleTankDifferentialNumberOfWheelsPerTrack("physxVehicleTankDifferential:numberOfWheelsPerTrack", TfToken::Immortal),
    physxVehicleTankDifferentialThrustIndexPerTrack("physxVehicleTankDifferential:thrustIndexPerTrack", TfToken::Immortal),
    physxVehicleTankDifferentialTrackToWheelIndices("physxVehicleTankDifferential:trackToWheelIndices", TfToken::Immortal),
    physxVehicleTankDifferentialWheelIndicesInTrackOrder("physxVehicleTankDifferential:wheelIndicesInTrackOrder", TfToken::Immortal),
    physxVehicleTireCamberStiffness("physxVehicleTire:camberStiffness", TfToken::Immortal),
    physxVehicleTireCamberStiffnessPerUnitGravity("physxVehicleTire:camberStiffnessPerUnitGravity", TfToken::Immortal),
    physxVehicleTireFrictionTable("physxVehicleTire:frictionTable", TfToken::Immortal),
    physxVehicleTireFrictionVsSlipGraph("physxVehicleTire:frictionVsSlipGraph", TfToken::Immortal),
    physxVehicleTireLateralStiffnessGraph("physxVehicleTire:lateralStiffnessGraph", TfToken::Immortal),
    physxVehicleTireLatStiffX("physxVehicleTire:latStiffX", TfToken::Immortal),
    physxVehicleTireLatStiffY("physxVehicleTire:latStiffY", TfToken::Immortal),
    physxVehicleTireLongitudinalStiffness("physxVehicleTire:longitudinalStiffness", TfToken::Immortal),
    physxVehicleTireLongitudinalStiffnessPerUnitGravity("physxVehicleTire:longitudinalStiffnessPerUnitGravity", TfToken::Immortal),
    physxVehicleTireRestLoad("physxVehicleTire:restLoad", TfToken::Immortal),
    physxVehicleVehicleEnabled("physxVehicle:vehicleEnabled", TfToken::Immortal),
    physxVehicleWheelAttachmentCollisionGroup("physxVehicleWheelAttachment:collisionGroup", TfToken::Immortal),
    physxVehicleWheelAttachmentDriven("physxVehicleWheelAttachment:driven", TfToken::Immortal),
    physxVehicleWheelAttachmentIndex("physxVehicleWheelAttachment:index", TfToken::Immortal),
    physxVehicleWheelAttachmentSuspension("physxVehicleWheelAttachment:suspension", TfToken::Immortal),
    physxVehicleWheelAttachmentSuspensionForceAppPointOffset("physxVehicleWheelAttachment:suspensionForceAppPointOffset", TfToken::Immortal),
    physxVehicleWheelAttachmentSuspensionFrameOrientation("physxVehicleWheelAttachment:suspensionFrameOrientation", TfToken::Immortal),
    physxVehicleWheelAttachmentSuspensionFramePosition("physxVehicleWheelAttachment:suspensionFramePosition", TfToken::Immortal),
    physxVehicleWheelAttachmentSuspensionTravelDirection("physxVehicleWheelAttachment:suspensionTravelDirection", TfToken::Immortal),
    physxVehicleWheelAttachmentTire("physxVehicleWheelAttachment:tire", TfToken::Immortal),
    physxVehicleWheelAttachmentTireForceAppPointOffset("physxVehicleWheelAttachment:tireForceAppPointOffset", TfToken::Immortal),
    physxVehicleWheelAttachmentWheel("physxVehicleWheelAttachment:wheel", TfToken::Immortal),
    physxVehicleWheelAttachmentWheelCenterOfMassOffset("physxVehicleWheelAttachment:wheelCenterOfMassOffset", TfToken::Immortal),
    physxVehicleWheelAttachmentWheelFrameOrientation("physxVehicleWheelAttachment:wheelFrameOrientation", TfToken::Immortal),
    physxVehicleWheelAttachmentWheelFramePosition("physxVehicleWheelAttachment:wheelFramePosition", TfToken::Immortal),
    physxVehicleWheelControllerBrakeTorque("physxVehicleWheelController:brakeTorque", TfToken::Immortal),
    physxVehicleWheelControllerDriveTorque("physxVehicleWheelController:driveTorque", TfToken::Immortal),
    physxVehicleWheelControllerSteerAngle("physxVehicleWheelController:steerAngle", TfToken::Immortal),
    physxVehicleWheelDampingRate("physxVehicleWheel:dampingRate", TfToken::Immortal),
    physxVehicleWheelMass("physxVehicleWheel:mass", TfToken::Immortal),
    physxVehicleWheelMaxBrakeTorque("physxVehicleWheel:maxBrakeTorque", TfToken::Immortal),
    physxVehicleWheelMaxHandBrakeTorque("physxVehicleWheel:maxHandBrakeTorque", TfToken::Immortal),
    physxVehicleWheelMaxSteerAngle("physxVehicleWheel:maxSteerAngle", TfToken::Immortal),
    physxVehicleWheelMoi("physxVehicleWheel:moi", TfToken::Immortal),
    physxVehicleWheelRadius("physxVehicleWheel:radius", TfToken::Immortal),
    physxVehicleWheelToeAngle("physxVehicleWheel:toeAngle", TfToken::Immortal),
    physxVehicleWheelWidth("physxVehicleWheel:width", TfToken::Immortal),
    posX("posX", TfToken::Immortal),
    posY("posY", TfToken::Immortal),
    posZ("posZ", TfToken::Immortal),
    preventClimbing("preventClimbing", TfToken::Immortal),
    preventClimbingForceSliding("preventClimbingForceSliding", TfToken::Immortal),
    quasistaticactors("quasistaticactors", TfToken::Immortal),
    raycast("raycast", TfToken::Immortal),
    referenceFrameIsCenterOfMass("physxVehicle:referenceFrameIsCenterOfMass", TfToken::Immortal),
    restOffset("restOffset", TfToken::Immortal),
    rotX("rotX", TfToken::Immortal),
    rotY("rotY", TfToken::Immortal),
    rotZ("rotZ", TfToken::Immortal),
    SAP("SAP", TfToken::Immortal),
    SAT("SAT", TfToken::Immortal),
    sdf("sdf", TfToken::Immortal),
    simulationOwner("simulationOwner", TfToken::Immortal),
    solidRestOffset("solidRestOffset", TfToken::Immortal),
    solverPositionIterationCount("solverPositionIterationCount", TfToken::Immortal),
    sphereFill("sphereFill", TfToken::Immortal),
    state("state", TfToken::Immortal),
    state_MultipleApplyTemplate_PhysicsPosition("state:__INSTANCE_NAME__:physics:position", TfToken::Immortal),
    state_MultipleApplyTemplate_PhysicsVelocity("state:__INSTANCE_NAME__:physics:velocity", TfToken::Immortal),
    steer("steer", TfToken::Immortal),
    surface("surface", TfToken::Immortal),
    sweep("sweep", TfToken::Immortal),
    Synchronous("Synchronous", TfToken::Immortal),
    TGS("TGS", TfToken::Immortal),
    transX("transX", TfToken::Immortal),
    transY("transY", TfToken::Immortal),
    transZ("transZ", TfToken::Immortal),
    triangleMesh("triangleMesh", TfToken::Immortal),
    undefined("undefined", TfToken::Immortal),
    velocityChange("velocityChange", TfToken::Immortal),
    wind("wind", TfToken::Immortal),
    X("X", TfToken::Immortal),
    Y("Y", TfToken::Immortal),
    Z("Z", TfToken::Immortal),
    PhysicsJointStateAPI("PhysicsJointStateAPI", TfToken::Immortal),
    PhysxArticulationAPI("PhysxArticulationAPI", TfToken::Immortal),
    PhysxAutoDeformableAttachmentAPI("PhysxAutoDeformableAttachmentAPI", TfToken::Immortal),
    PhysxAutoDeformableBodyAPI("PhysxAutoDeformableBodyAPI", TfToken::Immortal),
    PhysxAutoDeformableHexahedralMeshAPI("PhysxAutoDeformableHexahedralMeshAPI", TfToken::Immortal),
    PhysxAutoDeformableMeshSimplificationAPI("PhysxAutoDeformableMeshSimplificationAPI", TfToken::Immortal),
    PhysxBaseDeformableBodyAPI("PhysxBaseDeformableBodyAPI", TfToken::Immortal),
    PhysxCameraAPI("PhysxCameraAPI", TfToken::Immortal),
    PhysxCameraDroneAPI("PhysxCameraDroneAPI", TfToken::Immortal),
    PhysxCameraFollowAPI("PhysxCameraFollowAPI", TfToken::Immortal),
    PhysxCameraFollowLookAPI("PhysxCameraFollowLookAPI", TfToken::Immortal),
    PhysxCameraFollowVelocityAPI("PhysxCameraFollowVelocityAPI", TfToken::Immortal),
    PhysxCharacterControllerAPI("PhysxCharacterControllerAPI", TfToken::Immortal),
    PhysxCollisionAPI("PhysxCollisionAPI", TfToken::Immortal),
    PhysxContactReportAPI("PhysxContactReportAPI", TfToken::Immortal),
    PhysxConvexDecompositionCollisionAPI("PhysxConvexDecompositionCollisionAPI", TfToken::Immortal),
    PhysxConvexHullCollisionAPI("PhysxConvexHullCollisionAPI", TfToken::Immortal),
    PhysxCookedDataAPI("PhysxCookedDataAPI", TfToken::Immortal),
    PhysxDeformableMaterialAPI("PhysxDeformableMaterialAPI", TfToken::Immortal),
    PhysxDiffuseParticlesAPI("PhysxDiffuseParticlesAPI", TfToken::Immortal),
    PhysxDrivePerformanceEnvelopeAPI("PhysxDrivePerformanceEnvelopeAPI", TfToken::Immortal),
    PhysxForceAPI("PhysxForceAPI", TfToken::Immortal),
    PhysxJointAPI("PhysxJointAPI", TfToken::Immortal),
    PhysxJointAxisAPI("PhysxJointAxisAPI", TfToken::Immortal),
    PhysxLimitAPI("PhysxLimitAPI", TfToken::Immortal),
    PhysxMaterialAPI("PhysxMaterialAPI", TfToken::Immortal),
    PhysxMeshMergeCollisionAPI("PhysxMeshMergeCollisionAPI", TfToken::Immortal),
    PhysxMimicJointAPI("PhysxMimicJointAPI", TfToken::Immortal),
    PhysxParticleAnisotropyAPI("PhysxParticleAnisotropyAPI", TfToken::Immortal),
    PhysxParticleAPI("PhysxParticleAPI", TfToken::Immortal),
    PhysxParticleIsosurfaceAPI("PhysxParticleIsosurfaceAPI", TfToken::Immortal),
    PhysxParticleSamplingAPI("PhysxParticleSamplingAPI", TfToken::Immortal),
    PhysxParticleSetAPI("PhysxParticleSetAPI", TfToken::Immortal),
    PhysxParticleSmoothingAPI("PhysxParticleSmoothingAPI", TfToken::Immortal),
    PhysxParticleSystem("PhysxParticleSystem", TfToken::Immortal),
    PhysxPBDMaterialAPI("PhysxPBDMaterialAPI", TfToken::Immortal),
    PhysxPhysicsDistanceJointAPI("PhysxPhysicsDistanceJointAPI", TfToken::Immortal),
    PhysxPhysicsGearJoint("PhysxPhysicsGearJoint", TfToken::Immortal),
    PhysxPhysicsInstancer("PhysxPhysicsInstancer", TfToken::Immortal),
    PhysxPhysicsJointInstancer("PhysxPhysicsJointInstancer", TfToken::Immortal),
    PhysxPhysicsRackAndPinionJoint("PhysxPhysicsRackAndPinionJoint", TfToken::Immortal),
    PhysxRigidBodyAPI("PhysxRigidBodyAPI", TfToken::Immortal),
    PhysxSceneAPI("PhysxSceneAPI", TfToken::Immortal),
    PhysxSceneQuasistaticAPI("PhysxSceneQuasistaticAPI", TfToken::Immortal),
    PhysxSDFMeshCollisionAPI("PhysxSDFMeshCollisionAPI", TfToken::Immortal),
    PhysxSphereFillCollisionAPI("PhysxSphereFillCollisionAPI", TfToken::Immortal),
    PhysxSplinesSurfaceVelocityAPI("PhysxSplinesSurfaceVelocityAPI", TfToken::Immortal),
    PhysxSurfaceDeformableBodyAPI("PhysxSurfaceDeformableBodyAPI", TfToken::Immortal),
    PhysxSurfaceDeformableMaterialAPI("PhysxSurfaceDeformableMaterialAPI", TfToken::Immortal),
    PhysxSurfaceVelocityAPI("PhysxSurfaceVelocityAPI", TfToken::Immortal),
    PhysxTendonAttachmentAPI("PhysxTendonAttachmentAPI", TfToken::Immortal),
    PhysxTendonAttachmentLeafAPI("PhysxTendonAttachmentLeafAPI", TfToken::Immortal),
    PhysxTendonAttachmentRootAPI("PhysxTendonAttachmentRootAPI", TfToken::Immortal),
    PhysxTendonAxisAPI("PhysxTendonAxisAPI", TfToken::Immortal),
    PhysxTendonAxisRootAPI("PhysxTendonAxisRootAPI", TfToken::Immortal),
    PhysxTriangleMeshCollisionAPI("PhysxTriangleMeshCollisionAPI", TfToken::Immortal),
    PhysxTriangleMeshSimplificationCollisionAPI("PhysxTriangleMeshSimplificationCollisionAPI", TfToken::Immortal),
    PhysxTriggerAPI("PhysxTriggerAPI", TfToken::Immortal),
    PhysxTriggerStateAPI("PhysxTriggerStateAPI", TfToken::Immortal),
    PhysxVehicleAckermannSteeringAPI("PhysxVehicleAckermannSteeringAPI", TfToken::Immortal),
    PhysxVehicleAPI("PhysxVehicleAPI", TfToken::Immortal),
    PhysxVehicleAutoGearBoxAPI("PhysxVehicleAutoGearBoxAPI", TfToken::Immortal),
    PhysxVehicleBrakesAPI("PhysxVehicleBrakesAPI", TfToken::Immortal),
    PhysxVehicleClutchAPI("PhysxVehicleClutchAPI", TfToken::Immortal),
    PhysxVehicleContextAPI("PhysxVehicleContextAPI", TfToken::Immortal),
    PhysxVehicleControllerAPI("PhysxVehicleControllerAPI", TfToken::Immortal),
    PhysxVehicleDriveBasicAPI("PhysxVehicleDriveBasicAPI", TfToken::Immortal),
    PhysxVehicleDriveStandardAPI("PhysxVehicleDriveStandardAPI", TfToken::Immortal),
    PhysxVehicleEngineAPI("PhysxVehicleEngineAPI", TfToken::Immortal),
    PhysxVehicleGearsAPI("PhysxVehicleGearsAPI", TfToken::Immortal),
    PhysxVehicleMultiWheelDifferentialAPI("PhysxVehicleMultiWheelDifferentialAPI", TfToken::Immortal),
    PhysxVehicleNonlinearCommandResponseAPI("PhysxVehicleNonlinearCommandResponseAPI", TfToken::Immortal),
    PhysxVehicleSteeringAPI("PhysxVehicleSteeringAPI", TfToken::Immortal),
    PhysxVehicleSuspensionAPI("PhysxVehicleSuspensionAPI", TfToken::Immortal),
    PhysxVehicleSuspensionComplianceAPI("PhysxVehicleSuspensionComplianceAPI", TfToken::Immortal),
    PhysxVehicleTankControllerAPI("PhysxVehicleTankControllerAPI", TfToken::Immortal),
    PhysxVehicleTankDifferentialAPI("PhysxVehicleTankDifferentialAPI", TfToken::Immortal),
    PhysxVehicleTireAPI("PhysxVehicleTireAPI", TfToken::Immortal),
    PhysxVehicleTireFrictionTable("PhysxVehicleTireFrictionTable", TfToken::Immortal),
    PhysxVehicleWheelAPI("PhysxVehicleWheelAPI", TfToken::Immortal),
    PhysxVehicleWheelAttachmentAPI("PhysxVehicleWheelAttachmentAPI", TfToken::Immortal),
    PhysxVehicleWheelControllerAPI("PhysxVehicleWheelControllerAPI", TfToken::Immortal),
    allTokens({
        acceleration,
        alwaysUpdateEnabled,
        Asynchronous,
        average,
        BitsPerPixel16,
        BitsPerPixel32,
        BitsPerPixel8,
        brakes0,
        brakes1,
        collisionmeshes,
        constrained,
        contactOffset,
        convexDecomposition,
        convexHull,
        defaultFrictionValue,
        Disabled,
        drive,
        easy,
        enableCCD,
        flood,
        fluidRestOffset,
        force,
        frictionValues,
        globalSelfCollisionEnabled,
        GPU,
        groundMaterials,
        max,
        maxDepenetrationVelocity,
        maxNeighborhood,
        maxVelocity,
        MBP,
        min,
        multiply,
        negX,
        negY,
        negZ,
        neighborhoodScale,
        nonParticleCollisionEnabled,
        particleContactOffset,
        particleSystemEnabled,
        patch,
        PCM,
        PGS,
        physicsBody0Indices,
        physicsBody0s,
        physicsBody1Indices,
        physicsBody1s,
        physicsGearRatio,
        physicsHinge,
        physicsHinge0,
        physicsHinge1,
        physicsLocalPos0s,
        physicsLocalPos1s,
        physicsLocalRot0s,
        physicsLocalRot1s,
        physicsPrismatic,
        physicsProtoIndices,
        physicsPrototypes,
        physicsRatio,
        physxArticulationArticulationEnabled,
        physxArticulationEnabledSelfCollisions,
        physxArticulationSleepThreshold,
        physxArticulationSolverPositionIterationCount,
        physxArticulationSolverVelocityIterationCount,
        physxArticulationStabilizationThreshold,
        physxAutoDeformableAttachmentAttachable0,
        physxAutoDeformableAttachmentAttachable1,
        physxAutoDeformableAttachmentCollisionFilteringOffset,
        physxAutoDeformableAttachmentDeformableVertexOverlapOffset,
        physxAutoDeformableAttachmentEnableCollisionFiltering,
        physxAutoDeformableAttachmentEnableDeformableFilteringPairs,
        physxAutoDeformableAttachmentEnableDeformableVertexAttachments,
        physxAutoDeformableAttachmentEnableRigidSurfaceAttachments,
        physxAutoDeformableAttachmentMaskShapes,
        physxAutoDeformableAttachmentRigidSurfaceSamplingDistance,
        physxCameraSubject,
        physxCharacterControllerClimbingMode,
        physxCharacterControllerContactOffset,
        physxCharacterControllerInvisibleWallHeight,
        physxCharacterControllerMaxJumpHeight,
        physxCharacterControllerMoveTarget,
        physxCharacterControllerNonWalkableMode,
        physxCharacterControllerScaleCoeff,
        physxCharacterControllerSimulationOwner,
        physxCharacterControllerSlopeLimit,
        physxCharacterControllerStepOffset,
        physxCharacterControllerUpAxis,
        physxCharacterControllerVolumeGrowth,
        physxCollisionContactOffset,
        physxCollisionCustomGeometry,
        physxCollisionMinTorsionalPatchRadius,
        physxCollisionRestOffset,
        physxCollisionTorsionalPatchRadius,
        physxContactReportReportPairs,
        physxContactReportThreshold,
        physxConvexDecompositionCollisionErrorPercentage,
        physxConvexDecompositionCollisionHullVertexLimit,
        physxConvexDecompositionCollisionMaxConvexHulls,
        physxConvexDecompositionCollisionMinThickness,
        physxConvexDecompositionCollisionShrinkWrap,
        physxConvexDecompositionCollisionVoxelResolution,
        physxConvexHullCollisionHullVertexLimit,
        physxConvexHullCollisionMinThickness,
        physxCookedData,
        physxCookedData_MultipleApplyTemplate_Buffer,
        physxDeformableBodyAutoDeformableBodyEnabled,
        physxDeformableBodyAutoDeformableMeshSimplificationEnabled,
        physxDeformableBodyCollisionIterationMultiplier,
        physxDeformableBodyCollisionPairUpdateFrequency,
        physxDeformableBodyCookingSourceMesh,
        physxDeformableBodyDisableGravity,
        physxDeformableBodyEnableSpeculativeCCD,
        physxDeformableBodyForceConforming,
        physxDeformableBodyLinearDamping,
        physxDeformableBodyMaxDepenetrationVelocity,
        physxDeformableBodyMaxLinearVelocity,
        physxDeformableBodyRemeshingEnabled,
        physxDeformableBodyRemeshingResolution,
        physxDeformableBodyResolution,
        physxDeformableBodySelfCollision,
        physxDeformableBodySelfCollisionFilterDistance,
        physxDeformableBodySettlingDamping,
        physxDeformableBodySettlingThreshold,
        physxDeformableBodySleepThreshold,
        physxDeformableBodySolverPositionIterationCount,
        physxDeformableBodyTargetTriangleCount,
        physxDeformableMaterialBendDamping,
        physxDeformableMaterialElasticityDamping,
        physxDiffuseParticlesAirDrag,
        physxDiffuseParticlesBubbleDrag,
        physxDiffuseParticlesBuoyancy,
        physxDiffuseParticlesCollisionDecay,
        physxDiffuseParticlesDiffuseParticlesEnabled,
        physxDiffuseParticlesDivergenceWeight,
        physxDiffuseParticlesKineticEnergyWeight,
        physxDiffuseParticlesLifetime,
        physxDiffuseParticlesMaxDiffuseParticleMultiplier,
        physxDiffuseParticlesPressureWeight,
        physxDiffuseParticlesThreshold,
        physxDiffuseParticlesUseAccurateVelocity,
        physxDrivePerformanceEnvelope,
        physxDrivePerformanceEnvelope_MultipleApplyTemplate_MaxActuatorVelocity,
        physxDrivePerformanceEnvelope_MultipleApplyTemplate_SpeedEffortGradient,
        physxDrivePerformanceEnvelope_MultipleApplyTemplate_VelocityDependentResistance,
        physxDroneCameraFeedForwardVelocityGain,
        physxDroneCameraFollowDistance,
        physxDroneCameraFollowHeight,
        physxDroneCameraHorizontalVelocityGain,
        physxDroneCameraMaxDistance,
        physxDroneCameraMaxSpeed,
        physxDroneCameraPositionOffset,
        physxDroneCameraRotationFilterTimeConstant,
        physxDroneCameraVelocityFilterTimeConstant,
        physxDroneCameraVerticalVelocityGain,
        physxFollowCameraCameraPositionTimeConstant,
        physxFollowCameraFollowMaxDistance,
        physxFollowCameraFollowMaxSpeed,
        physxFollowCameraFollowMinDistance,
        physxFollowCameraFollowMinSpeed,
        physxFollowCameraFollowTurnRateGain,
        physxFollowCameraLookAheadMaxSpeed,
        physxFollowCameraLookAheadMinDistance,
        physxFollowCameraLookAheadMinSpeed,
        physxFollowCameraLookAheadTurnRateGain,
        physxFollowCameraLookPositionHeight,
        physxFollowCameraLookPositionTimeConstant,
        physxFollowCameraPitchAngle,
        physxFollowCameraPitchAngleTimeConstant,
        physxFollowCameraPositionOffset,
        physxFollowCameraSlowPitchAngleSpeed,
        physxFollowCameraSlowSpeedPitchAngleScale,
        physxFollowCameraVelocityNormalMinSpeed,
        physxFollowCameraYawAngle,
        physxFollowCameraYawRateTimeConstant,
        physxFollowFollowCameraLookAheadMaxDistance,
        physxFollowLookCameraDownHillGroundAngle,
        physxFollowLookCameraDownHillGroundPitch,
        physxFollowLookCameraFollowReverseDistance,
        physxFollowLookCameraFollowReverseSpeed,
        physxFollowLookCameraUpHillGroundAngle,
        physxFollowLookCameraUpHillGroundPitch,
        physxFollowLookCameraVelocityBlendTimeConstant,
        physxForceForce,
        physxForceForceEnabled,
        physxForceMode,
        physxForceTorque,
        physxForceWorldFrameEnabled,
        physxJointArmature,
        physxJointAxis,
        physxJointAxis_MultipleApplyTemplate_Armature,
        physxJointAxis_MultipleApplyTemplate_DynamicFrictionEffort,
        physxJointAxis_MultipleApplyTemplate_MaxJointVelocity,
        physxJointAxis_MultipleApplyTemplate_StaticFrictionEffort,
        physxJointAxis_MultipleApplyTemplate_ViscousFrictionCoefficient,
        physxJointJointFriction,
        physxJointMaxJointVelocity,
        physxLimit,
        physxLimit_MultipleApplyTemplate_BounceThreshold,
        physxLimit_MultipleApplyTemplate_Damping,
        physxLimit_MultipleApplyTemplate_Restitution,
        physxLimit_MultipleApplyTemplate_Stiffness,
        physxMaterialCompliantContactAccelerationSpring,
        physxMaterialCompliantContactDamping,
        physxMaterialCompliantContactStiffness,
        physxMaterialDampingCombineMode,
        physxMaterialFrictionCombineMode,
        physxMaterialRestitutionCombineMode,
        physxMimicJoint,
        physxMimicJoint_MultipleApplyTemplate_DampingRatio,
        physxMimicJoint_MultipleApplyTemplate_Gearing,
        physxMimicJoint_MultipleApplyTemplate_NaturalFrequency,
        physxMimicJoint_MultipleApplyTemplate_Offset,
        physxMimicJoint_MultipleApplyTemplate_ReferenceJoint,
        physxMimicJoint_MultipleApplyTemplate_ReferenceJointAxis,
        physxParticleAnisotropyMax,
        physxParticleAnisotropyMin,
        physxParticleAnisotropyParticleAnisotropyEnabled,
        physxParticleAnisotropyScale,
        physxParticleFluid,
        physxParticleIsosurfaceGridFilteringPasses,
        physxParticleIsosurfaceGridSmoothingRadius,
        physxParticleIsosurfaceGridSpacing,
        physxParticleIsosurfaceIsosurfaceEnabled,
        physxParticleIsosurfaceMaxSubgrids,
        physxParticleIsosurfaceMaxTriangles,
        physxParticleIsosurfaceMaxVertices,
        physxParticleIsosurfaceNumMeshNormalSmoothingPasses,
        physxParticleIsosurfaceNumMeshSmoothingPasses,
        physxParticleIsosurfaceSurfaceDistance,
        physxParticleParticleEnabled,
        physxParticleParticleGroup,
        physxParticleParticleSystem,
        physxParticleSamplingMaxSamples,
        physxParticleSamplingParticles,
        physxParticleSamplingSamplingDistance,
        physxParticleSamplingVolume,
        physxParticleSelfCollision,
        physxParticleSimulationPoints,
        physxParticleSmoothingParticleSmoothingEnabled,
        physxParticleSmoothingStrength,
        physxPBDMaterialAdhesion,
        physxPBDMaterialAdhesionOffsetScale,
        physxPBDMaterialCflCoefficient,
        physxPBDMaterialCohesion,
        physxPBDMaterialDamping,
        physxPBDMaterialDensity,
        physxPBDMaterialFriction,
        physxPBDMaterialGravityScale,
        physxPBDMaterialParticleAdhesionScale,
        physxPBDMaterialParticleFrictionScale,
        physxPBDMaterialSurfaceTension,
        physxPBDMaterialViscosity,
        physxPBDMaterialVorticityConfinement,
        physxPhysicsDistanceJointSpringDamping,
        physxPhysicsDistanceJointSpringEnabled,
        physxPhysicsDistanceJointSpringStiffness,
        physxRigidBodyAngularDamping,
        physxRigidBodyCfmScale,
        physxRigidBodyContactSlopCoefficient,
        physxRigidBodyDisableGravity,
        physxRigidBodyEnableCCD,
        physxRigidBodyEnableGyroscopicForces,
        physxRigidBodyEnableSpeculativeCCD,
        physxRigidBodyLinearDamping,
        physxRigidBodyLockedPosAxis,
        physxRigidBodyLockedRotAxis,
        physxRigidBodyMaxAngularVelocity,
        physxRigidBodyMaxContactImpulse,
        physxRigidBodyMaxDepenetrationVelocity,
        physxRigidBodyMaxLinearVelocity,
        physxRigidBodyRetainAccelerations,
        physxRigidBodySleepThreshold,
        physxRigidBodySolveContact,
        physxRigidBodySolverPositionIterationCount,
        physxRigidBodySolverVelocityIterationCount,
        physxRigidBodyStabilizationThreshold,
        physxSceneBounceThreshold,
        physxSceneBroadphaseType,
        physxSceneCollisionSystem,
        physxSceneDisableSleeping,
        physxSceneEnableCCD,
        physxSceneEnableEnhancedDeterminism,
        physxSceneEnableExternalForcesEveryIteration,
        physxSceneEnableGPUDynamics,
        physxSceneEnableSceneQuerySupport,
        physxSceneEnableStabilization,
        physxSceneFrictionCorrelationDistance,
        physxSceneFrictionOffsetThreshold,
        physxSceneFrictionType,
        physxSceneGpuCollisionStackSize,
        physxSceneGpuFoundLostAggregatePairsCapacity,
        physxSceneGpuFoundLostPairsCapacity,
        physxSceneGpuHeapCapacity,
        physxSceneGpuMaxDeformableSurfaceContacts,
        physxSceneGpuMaxDeformableVolumeContacts,
        physxSceneGpuMaxNumPartitions,
        physxSceneGpuMaxParticleContacts,
        physxSceneGpuMaxRigidContactCount,
        physxSceneGpuMaxRigidPatchCount,
        physxSceneGpuTempBufferCapacity,
        physxSceneGpuTotalAggregatePairsCapacity,
        physxSceneInvertCollisionGroupFilter,
        physxSceneMaxBiasCoefficient,
        physxSceneMaxPositionIterationCount,
        physxSceneMaxVelocityIterationCount,
        physxSceneMinPositionIterationCount,
        physxSceneMinVelocityIterationCount,
        physxSceneQuasistaticEnableQuasistatic,
        physxSceneReportKinematicKinematicPairs,
        physxSceneReportKinematicStaticPairs,
        physxSceneSolveArticulationContactLast,
        physxSceneSolverType,
        physxSceneTimeStepsPerSecond,
        physxSceneUpdateType,
        physxSDFMeshCollisionSdfBitsPerSubgridPixel,
        physxSDFMeshCollisionSdfEnableRemeshing,
        physxSDFMeshCollisionSdfMargin,
        physxSDFMeshCollisionSdfNarrowBandThickness,
        physxSDFMeshCollisionSdfResolution,
        physxSDFMeshCollisionSdfSubgridResolution,
        physxSDFMeshCollisionSdfTriangleCountReductionFactor,
        physxSphereFillCollisionFillMode,
        physxSphereFillCollisionMaxSpheres,
        physxSphereFillCollisionSeedCount,
        physxSphereFillCollisionVoxelResolution,
        physxSplinesSurfaceVelocitySurfaceVelocityCurve,
        physxSplinesSurfaceVelocitySurfaceVelocityEnabled,
        physxSplinesSurfaceVelocitySurfaceVelocityForceBased,
        physxSplinesSurfaceVelocitySurfaceVelocityMagnitude,
        physxSurfaceVelocitySurfaceAngularVelocity,
        physxSurfaceVelocitySurfaceVelocity,
        physxSurfaceVelocitySurfaceVelocityEnabled,
        physxSurfaceVelocitySurfaceVelocityLocalSpace,
        physxTendon,
        physxTendon_MultipleApplyTemplate_Damping,
        physxTendon_MultipleApplyTemplate_ForceCoefficient,
        physxTendon_MultipleApplyTemplate_Gearing,
        physxTendon_MultipleApplyTemplate_JointAxis,
        physxTendon_MultipleApplyTemplate_LimitStiffness,
        physxTendon_MultipleApplyTemplate_LocalPos,
        physxTendon_MultipleApplyTemplate_LowerLimit,
        physxTendon_MultipleApplyTemplate_Offset,
        physxTendon_MultipleApplyTemplate_ParentAttachment,
        physxTendon_MultipleApplyTemplate_ParentLink,
        physxTendon_MultipleApplyTemplate_RestLength,
        physxTendon_MultipleApplyTemplate_Stiffness,
        physxTendon_MultipleApplyTemplate_TendonEnabled,
        physxTendon_MultipleApplyTemplate_UpperLimit,
        physxTriangleMeshCollisionWeldTolerance,
        physxTriangleMeshSimplificationCollisionMetric,
        physxTriangleMeshSimplificationCollisionWeldTolerance,
        physxTriggerTriggeredCollisions,
        physxVehicleAckermannSteeringMaxSteerAngle,
        physxVehicleAckermannSteeringStrength,
        physxVehicleAckermannSteeringTrackWidth,
        physxVehicleAckermannSteeringWheel0,
        physxVehicleAckermannSteeringWheel1,
        physxVehicleAckermannSteeringWheelBase,
        physxVehicleAutoGearBoxDownRatios,
        physxVehicleAutoGearBoxLatency,
        physxVehicleAutoGearBoxUpRatios,
        physxVehicleBrakes,
        physxVehicleBrakes_MultipleApplyTemplate_MaxBrakeTorque,
        physxVehicleBrakes_MultipleApplyTemplate_TorqueMultipliers,
        physxVehicleBrakes_MultipleApplyTemplate_Wheels,
        physxVehicleClutchStrength,
        physxVehicleContextForwardAxis,
        physxVehicleContextLongitudinalAxis,
        physxVehicleContextUpAxis,
        physxVehicleContextUpdateMode,
        physxVehicleContextVerticalAxis,
        physxVehicleControllerAccelerator,
        physxVehicleControllerBrake,
        physxVehicleControllerBrake0,
        physxVehicleControllerBrake1,
        physxVehicleControllerHandbrake,
        physxVehicleControllerSteer,
        physxVehicleControllerSteerLeft,
        physxVehicleControllerSteerRight,
        physxVehicleControllerTargetGear,
        physxVehicleDrive,
        physxVehicleDriveBasicPeakTorque,
        physxVehicleDriveStandardAutoGearBox,
        physxVehicleDriveStandardClutch,
        physxVehicleDriveStandardEngine,
        physxVehicleDriveStandardGears,
        physxVehicleEngineDampingRateFullThrottle,
        physxVehicleEngineDampingRateZeroThrottleClutchDisengaged,
        physxVehicleEngineDampingRateZeroThrottleClutchEngaged,
        physxVehicleEngineIdleRotationSpeed,
        physxVehicleEngineMaxRotationSpeed,
        physxVehicleEngineMoi,
        physxVehicleEnginePeakTorque,
        physxVehicleEngineTorqueCurve,
        physxVehicleGearsRatios,
        physxVehicleGearsRatioScale,
        physxVehicleGearsSwitchTime,
        physxVehicleHighForwardSpeedSubStepCount,
        physxVehicleLateralStickyTireDamping,
        physxVehicleLateralStickyTireThresholdSpeed,
        physxVehicleLateralStickyTireThresholdTime,
        physxVehicleLimitSuspensionExpansionVelocity,
        physxVehicleLongitudinalStickyTireDamping,
        physxVehicleLongitudinalStickyTireThresholdSpeed,
        physxVehicleLongitudinalStickyTireThresholdTime,
        physxVehicleLowForwardSpeedSubStepCount,
        physxVehicleMinActiveLongitudinalSlipDenominator,
        physxVehicleMinLateralSlipDenominator,
        physxVehicleMinLongitudinalSlipDenominator,
        physxVehicleMinPassiveLongitudinalSlipDenominator,
        physxVehicleMultiWheelDifferentialAverageWheelSpeedRatios,
        physxVehicleMultiWheelDifferentialTorqueRatios,
        physxVehicleMultiWheelDifferentialWheels,
        physxVehicleNCR,
        physxVehicleNCR_MultipleApplyTemplate_CommandValues,
        physxVehicleNCR_MultipleApplyTemplate_SpeedResponses,
        physxVehicleNCR_MultipleApplyTemplate_SpeedResponsesPerCommandValue,
        physxVehicleSteeringAngleMultipliers,
        physxVehicleSteeringMaxSteerAngle,
        physxVehicleSteeringWheels,
        physxVehicleSubStepThresholdLongitudinalSpeed,
        physxVehicleSuspensionCamberAtMaxCompression,
        physxVehicleSuspensionCamberAtMaxDroop,
        physxVehicleSuspensionCamberAtRest,
        physxVehicleSuspensionComplianceSuspensionForceAppPoint,
        physxVehicleSuspensionComplianceTireForceAppPoint,
        physxVehicleSuspensionComplianceWheelCamberAngle,
        physxVehicleSuspensionComplianceWheelToeAngle,
        physxVehicleSuspensionLineQueryType,
        physxVehicleSuspensionMaxCompression,
        physxVehicleSuspensionMaxDroop,
        physxVehicleSuspensionSpringDamperRate,
        physxVehicleSuspensionSpringStrength,
        physxVehicleSuspensionSprungMass,
        physxVehicleSuspensionTravelDistance,
        physxVehicleTankControllerThrust0,
        physxVehicleTankControllerThrust1,
        physxVehicleTankDifferentialNumberOfWheelsPerTrack,
        physxVehicleTankDifferentialThrustIndexPerTrack,
        physxVehicleTankDifferentialTrackToWheelIndices,
        physxVehicleTankDifferentialWheelIndicesInTrackOrder,
        physxVehicleTireCamberStiffness,
        physxVehicleTireCamberStiffnessPerUnitGravity,
        physxVehicleTireFrictionTable,
        physxVehicleTireFrictionVsSlipGraph,
        physxVehicleTireLateralStiffnessGraph,
        physxVehicleTireLatStiffX,
        physxVehicleTireLatStiffY,
        physxVehicleTireLongitudinalStiffness,
        physxVehicleTireLongitudinalStiffnessPerUnitGravity,
        physxVehicleTireRestLoad,
        physxVehicleVehicleEnabled,
        physxVehicleWheelAttachmentCollisionGroup,
        physxVehicleWheelAttachmentDriven,
        physxVehicleWheelAttachmentIndex,
        physxVehicleWheelAttachmentSuspension,
        physxVehicleWheelAttachmentSuspensionForceAppPointOffset,
        physxVehicleWheelAttachmentSuspensionFrameOrientation,
        physxVehicleWheelAttachmentSuspensionFramePosition,
        physxVehicleWheelAttachmentSuspensionTravelDirection,
        physxVehicleWheelAttachmentTire,
        physxVehicleWheelAttachmentTireForceAppPointOffset,
        physxVehicleWheelAttachmentWheel,
        physxVehicleWheelAttachmentWheelCenterOfMassOffset,
        physxVehicleWheelAttachmentWheelFrameOrientation,
        physxVehicleWheelAttachmentWheelFramePosition,
        physxVehicleWheelControllerBrakeTorque,
        physxVehicleWheelControllerDriveTorque,
        physxVehicleWheelControllerSteerAngle,
        physxVehicleWheelDampingRate,
        physxVehicleWheelMass,
        physxVehicleWheelMaxBrakeTorque,
        physxVehicleWheelMaxHandBrakeTorque,
        physxVehicleWheelMaxSteerAngle,
        physxVehicleWheelMoi,
        physxVehicleWheelRadius,
        physxVehicleWheelToeAngle,
        physxVehicleWheelWidth,
        posX,
        posY,
        posZ,
        preventClimbing,
        preventClimbingForceSliding,
        quasistaticactors,
        raycast,
        referenceFrameIsCenterOfMass,
        restOffset,
        rotX,
        rotY,
        rotZ,
        SAP,
        SAT,
        sdf,
        simulationOwner,
        solidRestOffset,
        solverPositionIterationCount,
        sphereFill,
        state,
        state_MultipleApplyTemplate_PhysicsPosition,
        state_MultipleApplyTemplate_PhysicsVelocity,
        steer,
        surface,
        sweep,
        Synchronous,
        TGS,
        transX,
        transY,
        transZ,
        triangleMesh,
        undefined,
        velocityChange,
        wind,
        X,
        Y,
        Z,
        PhysicsJointStateAPI,
        PhysxArticulationAPI,
        PhysxAutoDeformableAttachmentAPI,
        PhysxAutoDeformableBodyAPI,
        PhysxAutoDeformableHexahedralMeshAPI,
        PhysxAutoDeformableMeshSimplificationAPI,
        PhysxBaseDeformableBodyAPI,
        PhysxCameraAPI,
        PhysxCameraDroneAPI,
        PhysxCameraFollowAPI,
        PhysxCameraFollowLookAPI,
        PhysxCameraFollowVelocityAPI,
        PhysxCharacterControllerAPI,
        PhysxCollisionAPI,
        PhysxContactReportAPI,
        PhysxConvexDecompositionCollisionAPI,
        PhysxConvexHullCollisionAPI,
        PhysxCookedDataAPI,
        PhysxDeformableMaterialAPI,
        PhysxDiffuseParticlesAPI,
        PhysxDrivePerformanceEnvelopeAPI,
        PhysxForceAPI,
        PhysxJointAPI,
        PhysxJointAxisAPI,
        PhysxLimitAPI,
        PhysxMaterialAPI,
        PhysxMeshMergeCollisionAPI,
        PhysxMimicJointAPI,
        PhysxParticleAnisotropyAPI,
        PhysxParticleAPI,
        PhysxParticleIsosurfaceAPI,
        PhysxParticleSamplingAPI,
        PhysxParticleSetAPI,
        PhysxParticleSmoothingAPI,
        PhysxParticleSystem,
        PhysxPBDMaterialAPI,
        PhysxPhysicsDistanceJointAPI,
        PhysxPhysicsGearJoint,
        PhysxPhysicsInstancer,
        PhysxPhysicsJointInstancer,
        PhysxPhysicsRackAndPinionJoint,
        PhysxRigidBodyAPI,
        PhysxSceneAPI,
        PhysxSceneQuasistaticAPI,
        PhysxSDFMeshCollisionAPI,
        PhysxSphereFillCollisionAPI,
        PhysxSplinesSurfaceVelocityAPI,
        PhysxSurfaceDeformableBodyAPI,
        PhysxSurfaceDeformableMaterialAPI,
        PhysxSurfaceVelocityAPI,
        PhysxTendonAttachmentAPI,
        PhysxTendonAttachmentLeafAPI,
        PhysxTendonAttachmentRootAPI,
        PhysxTendonAxisAPI,
        PhysxTendonAxisRootAPI,
        PhysxTriangleMeshCollisionAPI,
        PhysxTriangleMeshSimplificationCollisionAPI,
        PhysxTriggerAPI,
        PhysxTriggerStateAPI,
        PhysxVehicleAckermannSteeringAPI,
        PhysxVehicleAPI,
        PhysxVehicleAutoGearBoxAPI,
        PhysxVehicleBrakesAPI,
        PhysxVehicleClutchAPI,
        PhysxVehicleContextAPI,
        PhysxVehicleControllerAPI,
        PhysxVehicleDriveBasicAPI,
        PhysxVehicleDriveStandardAPI,
        PhysxVehicleEngineAPI,
        PhysxVehicleGearsAPI,
        PhysxVehicleMultiWheelDifferentialAPI,
        PhysxVehicleNonlinearCommandResponseAPI,
        PhysxVehicleSteeringAPI,
        PhysxVehicleSuspensionAPI,
        PhysxVehicleSuspensionComplianceAPI,
        PhysxVehicleTankControllerAPI,
        PhysxVehicleTankDifferentialAPI,
        PhysxVehicleTireAPI,
        PhysxVehicleTireFrictionTable,
        PhysxVehicleWheelAPI,
        PhysxVehicleWheelAttachmentAPI,
        PhysxVehicleWheelControllerAPI,
    })
{
}

/// \var PhysxSchemaTokens
///
/// A global variable with static, efficient \link TfToken TfTokens\endlink
/// for use in all public USD API.  \sa PhysxSchemaTokensType
///
/// Codeless: defined inline (header-only) -- no compiled tokens.cpp / library.
inline TfStaticData<PhysxSchemaTokensType> PhysxSchemaTokens;

PXR_NAMESPACE_CLOSE_SCOPE

#endif
