//
// Copyright 2016 Pixar
//
// Licensed under the Apache License, Version 2.0 (the "Apache License")
// with the following modification; you may not use this file except in
// compliance with the Apache License and the following modification to it:
// Section 6. Trademarks. is deleted and replaced with:
//
// 6. Trademarks. This License does not grant permission to use the trade
//    names, trademarks, service marks, or product names of the Licensor
//    and its affiliates, except as required to comply with Section 4(c) of
//    the License and to reproduce the content of the NOTICE file.
//
// You may obtain a copy of the Apache License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the Apache License with the above modification is
// distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied. See the Apache License for the specific
// language governing permissions and limitations under the Apache License.
//
#ifndef PHYSXSCHEMA_TOKENS_H
#define PHYSXSCHEMA_TOKENS_H

/// \file physxSchema/tokens.h

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// 
// This is an automatically generated file (by usdGenSchema.py).
// Do not hand-edit!
// 
// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

#include "pxr/pxr.h"
#include ".//api.h"
#include "pxr/base/tf/staticData.h"
#include "pxr/base/tf/token.h"
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
    PHYSXSCHEMA_API PhysxSchemaTokensType();
    /// \brief "acceleration"
    /// 
    /// Fallback value for PhysxSchemaPhysxForceAPI::GetModeAttr(), Possible value for PhysxSchemaPhysxVehicleContextAPI::GetUpdateModeAttr()
    const TfToken acceleration;
    /// \brief "actor0"
    /// 
    /// PhysxSchemaPhysxPhysicsAttachment
    const TfToken actor0;
    /// \brief "actor1"
    /// 
    /// PhysxSchemaPhysxPhysicsAttachment
    const TfToken actor1;
    /// \brief "alwaysUpdateEnabled"
    /// 
    /// PhysxSchemaPhysxCameraAPI
    const TfToken alwaysUpdateEnabled;
    /// \brief "Asynchronous"
    /// 
    /// Possible value for PhysxSchemaPhysxSceneAPI::GetUpdateTypeAttr()
    const TfToken Asynchronous;
    /// \brief "attachmentEnabled"
    /// 
    /// PhysxSchemaPhysxPhysicsAttachment
    const TfToken attachmentEnabled;
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
    /// \brief "clothConstaint"
    /// 
    ///  This token represents the collection name to use with PhysxCookedDataAPI to represent cooked data of a clothConstaint. 
    const TfToken clothConstaint;
    /// \brief "collisionFilterIndices0"
    /// 
    /// PhysxSchemaPhysxPhysicsAttachment
    const TfToken collisionFilterIndices0;
    /// \brief "collisionFilterIndices1"
    /// 
    /// PhysxSchemaPhysxPhysicsAttachment
    const TfToken collisionFilterIndices1;
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
    /// \brief "filterType0"
    /// 
    /// PhysxSchemaPhysxPhysicsAttachment
    const TfToken filterType0;
    /// \brief "filterType1"
    /// 
    /// PhysxSchemaPhysxPhysicsAttachment
    const TfToken filterType1;
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
    /// \brief "Geometry"
    /// 
    /// Possible value for PhysxSchemaPhysxPhysicsAttachment::GetFilterType0Attr(), Possible value for PhysxSchemaPhysxPhysicsAttachment::GetFilterType1Attr()
    const TfToken Geometry;
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
    /// \brief "indices"
    /// 
    /// PhysxSchemaTetrahedralMesh
    const TfToken indices;
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
    /// \brief "physxAutoAttachment:collisionFilteringOffset"
    /// 
    /// PhysxSchemaPhysxAutoAttachmentAPI
    const TfToken physxAutoAttachmentCollisionFilteringOffset;
    /// \brief "physxAutoAttachment:deformableVertexOverlapOffset"
    /// 
    /// PhysxSchemaPhysxAutoAttachmentAPI
    const TfToken physxAutoAttachmentDeformableVertexOverlapOffset;
    /// \brief "physxAutoAttachment:enableCollisionFiltering"
    /// 
    /// PhysxSchemaPhysxAutoAttachmentAPI
    const TfToken physxAutoAttachmentEnableCollisionFiltering;
    /// \brief "physxAutoAttachment:enableDeformableFilteringPairs"
    /// 
    /// PhysxSchemaPhysxAutoAttachmentAPI
    const TfToken physxAutoAttachmentEnableDeformableFilteringPairs;
    /// \brief "physxAutoAttachment:enableDeformableVertexAttachments"
    /// 
    /// PhysxSchemaPhysxAutoAttachmentAPI
    const TfToken physxAutoAttachmentEnableDeformableVertexAttachments;
    /// \brief "physxAutoAttachment:enableRigidSurfaceAttachments"
    /// 
    /// PhysxSchemaPhysxAutoAttachmentAPI
    const TfToken physxAutoAttachmentEnableRigidSurfaceAttachments;
    /// \brief "physxAutoAttachment:rigidSurfaceSamplingDistance"
    /// 
    /// PhysxSchemaPhysxAutoAttachmentAPI
    const TfToken physxAutoAttachmentRigidSurfaceSamplingDistance;
    /// \brief "physxAutoParticleCloth:disableMeshWelding"
    /// 
    /// PhysxSchemaPhysxAutoParticleClothAPI
    const TfToken physxAutoParticleClothDisableMeshWelding;
    /// \brief "physxAutoParticleCloth:springBendStiffness"
    /// 
    /// PhysxSchemaPhysxAutoParticleClothAPI
    const TfToken physxAutoParticleClothSpringBendStiffness;
    /// \brief "physxAutoParticleCloth:springDamping"
    /// 
    /// PhysxSchemaPhysxAutoParticleClothAPI
    const TfToken physxAutoParticleClothSpringDamping;
    /// \brief "physxAutoParticleCloth:springShearStiffness"
    /// 
    /// PhysxSchemaPhysxAutoParticleClothAPI
    const TfToken physxAutoParticleClothSpringShearStiffness;
    /// \brief "physxAutoParticleCloth:springStretchStiffness"
    /// 
    /// PhysxSchemaPhysxAutoParticleClothAPI
    const TfToken physxAutoParticleClothSpringStretchStiffness;
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
    /// \brief "physxDeformableBodyMaterial:dampingScale"
    /// 
    /// PhysxSchemaPhysxDeformableBodyMaterialAPI
    const TfToken physxDeformableBodyMaterialDampingScale;
    /// \brief "physxDeformableBodyMaterial:density"
    /// 
    /// PhysxSchemaPhysxDeformableBodyMaterialAPI
    const TfToken physxDeformableBodyMaterialDensity;
    /// \brief "physxDeformableBodyMaterial:dynamicFriction"
    /// 
    /// PhysxSchemaPhysxDeformableBodyMaterialAPI
    const TfToken physxDeformableBodyMaterialDynamicFriction;
    /// \brief "physxDeformableBodyMaterial:elasticityDamping"
    /// 
    /// PhysxSchemaPhysxDeformableBodyMaterialAPI
    const TfToken physxDeformableBodyMaterialElasticityDamping;
    /// \brief "physxDeformableBodyMaterial:poissonsRatio"
    /// 
    /// PhysxSchemaPhysxDeformableBodyMaterialAPI
    const TfToken physxDeformableBodyMaterialPoissonsRatio;
    /// \brief "physxDeformableBodyMaterial:youngsModulus"
    /// 
    /// PhysxSchemaPhysxDeformableBodyMaterialAPI
    const TfToken physxDeformableBodyMaterialYoungsModulus;
    /// \brief "physxDeformable:collisionIndices"
    /// 
    /// PhysxSchemaPhysxDeformableBodyAPI
    const TfToken physxDeformableCollisionIndices;
    /// \brief "physxDeformable:collisionPoints"
    /// 
    /// PhysxSchemaPhysxDeformableBodyAPI
    const TfToken physxDeformableCollisionPoints;
    /// \brief "physxDeformable:collisionRestPoints"
    /// 
    /// PhysxSchemaPhysxDeformableBodyAPI
    const TfToken physxDeformableCollisionRestPoints;
    /// \brief "physxDeformable:deformableEnabled"
    /// 
    /// PhysxSchemaPhysxDeformableAPI
    const TfToken physxDeformableDeformableEnabled;
    /// \brief "physxDeformable:disableGravity"
    /// 
    /// PhysxSchemaPhysxDeformableBodyAPI
    const TfToken physxDeformableDisableGravity;
    /// \brief "physxDeformable:enableCCD"
    /// 
    /// PhysxSchemaPhysxDeformableAPI
    const TfToken physxDeformableEnableCCD;
    /// \brief "physxDeformable:maxDepenetrationVelocity"
    /// 
    /// PhysxSchemaPhysxDeformableAPI
    const TfToken physxDeformableMaxDepenetrationVelocity;
    /// \brief "physxDeformable:restPoints"
    /// 
    /// PhysxSchemaPhysxDeformableAPI
    const TfToken physxDeformableRestPoints;
    /// \brief "physxDeformable:selfCollision"
    /// 
    /// PhysxSchemaPhysxDeformableAPI
    const TfToken physxDeformableSelfCollision;
    /// \brief "physxDeformable:selfCollisionFilterDistance"
    /// 
    /// PhysxSchemaPhysxDeformableAPI
    const TfToken physxDeformableSelfCollisionFilterDistance;
    /// \brief "physxDeformable:settlingThreshold"
    /// 
    /// PhysxSchemaPhysxDeformableAPI
    const TfToken physxDeformableSettlingThreshold;
    /// \brief "physxDeformable:simulationIndices"
    /// 
    /// PhysxSchemaPhysxDeformableAPI
    const TfToken physxDeformableSimulationIndices;
    /// \brief "physxDeformable:simulationOwner"
    /// 
    /// PhysxSchemaPhysxDeformableAPI
    const TfToken physxDeformableSimulationOwner;
    /// \brief "physxDeformable:simulationPoints"
    /// 
    /// PhysxSchemaPhysxDeformableBodyAPI
    const TfToken physxDeformableSimulationPoints;
    /// \brief "physxDeformable:simulationRestPoints"
    /// 
    /// PhysxSchemaPhysxDeformableBodyAPI
    const TfToken physxDeformableSimulationRestPoints;
    /// \brief "physxDeformable:simulationVelocities"
    /// 
    /// PhysxSchemaPhysxDeformableAPI
    const TfToken physxDeformableSimulationVelocities;
    /// \brief "physxDeformable:sleepDamping"
    /// 
    /// PhysxSchemaPhysxDeformableAPI
    const TfToken physxDeformableSleepDamping;
    /// \brief "physxDeformable:sleepThreshold"
    /// 
    /// PhysxSchemaPhysxDeformableAPI
    const TfToken physxDeformableSleepThreshold;
    /// \brief "physxDeformable:solverPositionIterationCount"
    /// 
    /// PhysxSchemaPhysxDeformableAPI
    const TfToken physxDeformableSolverPositionIterationCount;
    /// \brief "physxDeformableSurface:bendingStiffnessScale"
    /// 
    /// PhysxSchemaPhysxDeformableSurfaceAPI
    const TfToken physxDeformableSurfaceBendingStiffnessScale;
    /// \brief "physxDeformableSurface:collisionIterationMultiplier"
    /// 
    /// PhysxSchemaPhysxDeformableSurfaceAPI
    const TfToken physxDeformableSurfaceCollisionIterationMultiplier;
    /// \brief "physxDeformableSurface:collisionPairUpdateFrequency"
    /// 
    /// PhysxSchemaPhysxDeformableSurfaceAPI
    const TfToken physxDeformableSurfaceCollisionPairUpdateFrequency;
    /// \brief "physxDeformableSurface:flatteningEnabled"
    /// 
    /// PhysxSchemaPhysxDeformableSurfaceAPI
    const TfToken physxDeformableSurfaceFlatteningEnabled;
    /// \brief "physxDeformableSurfaceMaterial:density"
    /// 
    /// PhysxSchemaPhysxDeformableSurfaceMaterialAPI
    const TfToken physxDeformableSurfaceMaterialDensity;
    /// \brief "physxDeformableSurfaceMaterial:dynamicFriction"
    /// 
    /// PhysxSchemaPhysxDeformableSurfaceMaterialAPI
    const TfToken physxDeformableSurfaceMaterialDynamicFriction;
    /// \brief "physxDeformableSurfaceMaterial:poissonsRatio"
    /// 
    /// PhysxSchemaPhysxDeformableSurfaceMaterialAPI
    const TfToken physxDeformableSurfaceMaterialPoissonsRatio;
    /// \brief "physxDeformableSurfaceMaterial:thickness"
    /// 
    /// PhysxSchemaPhysxDeformableSurfaceMaterialAPI
    const TfToken physxDeformableSurfaceMaterialThickness;
    /// \brief "physxDeformableSurfaceMaterial:youngsModulus"
    /// 
    /// PhysxSchemaPhysxDeformableSurfaceMaterialAPI
    const TfToken physxDeformableSurfaceMaterialYoungsModulus;
    /// \brief "physxDeformableSurface:maxVelocity"
    /// 
    /// PhysxSchemaPhysxDeformableSurfaceAPI
    const TfToken physxDeformableSurfaceMaxVelocity;
    /// \brief "physxDeformable:vertexVelocityDamping"
    /// 
    /// PhysxSchemaPhysxDeformableAPI
    const TfToken physxDeformableVertexVelocityDamping;
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
    /// \brief "physxMimicJoint:__INSTANCE_NAME__:gearing"
    /// 
    /// PhysxSchemaPhysxMimicJointAPI
    const TfToken physxMimicJoint_MultipleApplyTemplate_Gearing;
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
    /// \brief "physxParticle:pressure"
    /// 
    /// PhysxSchemaPhysxParticleClothAPI
    const TfToken physxParticlePressure;
    /// \brief "physxParticle:restPoints"
    /// 
    /// PhysxSchemaPhysxParticleClothAPI
    const TfToken physxParticleRestPoints;
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
    /// \brief "physxParticle:selfCollisionFilter"
    /// 
    /// PhysxSchemaPhysxParticleClothAPI
    const TfToken physxParticleSelfCollisionFilter;
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
    /// \brief "physxParticle:springDampings"
    /// 
    /// PhysxSchemaPhysxParticleClothAPI
    const TfToken physxParticleSpringDampings;
    /// \brief "physxParticle:springIndices"
    /// 
    /// PhysxSchemaPhysxParticleClothAPI
    const TfToken physxParticleSpringIndices;
    /// \brief "physxParticle:springRestLengths"
    /// 
    /// PhysxSchemaPhysxParticleClothAPI
    const TfToken physxParticleSpringRestLengths;
    /// \brief "physxParticle:springStiffnesses"
    /// 
    /// PhysxSchemaPhysxParticleClothAPI
    const TfToken physxParticleSpringStiffnesses;
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
    /// \brief "physxPBDMaterial:drag"
    /// 
    /// PhysxSchemaPhysxPBDMaterialAPI
    const TfToken physxPBDMaterialDrag;
    /// \brief "physxPBDMaterial:friction"
    /// 
    /// PhysxSchemaPhysxPBDMaterialAPI
    const TfToken physxPBDMaterialFriction;
    /// \brief "physxPBDMaterial:gravityScale"
    /// 
    /// PhysxSchemaPhysxPBDMaterialAPI
    const TfToken physxPBDMaterialGravityScale;
    /// \brief "physxPBDMaterial:lift"
    /// 
    /// PhysxSchemaPhysxPBDMaterialAPI
    const TfToken physxPBDMaterialLift;
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
    /// \brief "physxResidualReporting:maxResidualPositionIteration"
    /// 
    /// PhysxSchemaPhysxResidualReportingAPI
    const TfToken physxResidualReportingMaxResidualPositionIteration;
    /// \brief "physxResidualReporting:maxResidualVelocityIteration"
    /// 
    /// PhysxSchemaPhysxResidualReportingAPI
    const TfToken physxResidualReportingMaxResidualVelocityIteration;
    /// \brief "physxResidualReporting:rmsResidualPositionIteration"
    /// 
    /// PhysxSchemaPhysxResidualReportingAPI
    const TfToken physxResidualReportingRmsResidualPositionIteration;
    /// \brief "physxResidualReporting:rmsResidualVelocityIteration"
    /// 
    /// PhysxSchemaPhysxResidualReportingAPI
    const TfToken physxResidualReportingRmsResidualVelocityIteration;
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
    /// \brief "physxScene:enableResidualReporting"
    /// 
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneEnableResidualReporting;
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
    /// \brief "physxScene:gpuMaxSoftBodyContacts"
    /// 
    /// PhysxSchemaPhysxSceneAPI
    const TfToken physxSceneGpuMaxSoftBodyContacts;
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
    /// \brief "physxTrigger:enterScriptType"
    /// 
    /// PhysxSchemaPhysxTriggerAPI
    const TfToken physxTriggerEnterScriptType;
    /// \brief "physxTrigger:leaveScriptType"
    /// 
    /// PhysxSchemaPhysxTriggerAPI
    const TfToken physxTriggerLeaveScriptType;
    /// \brief "physxTrigger:onEnterScript"
    /// 
    /// PhysxSchemaPhysxTriggerAPI
    const TfToken physxTriggerOnEnterScript;
    /// \brief "physxTrigger:onLeaveScript"
    /// 
    /// PhysxSchemaPhysxTriggerAPI
    const TfToken physxTriggerOnLeaveScript;
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
    /// \brief "points0"
    /// 
    /// PhysxSchemaPhysxPhysicsAttachment
    const TfToken points0;
    /// \brief "points1"
    /// 
    /// PhysxSchemaPhysxPhysicsAttachment
    const TfToken points1;
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
    /// \brief "scriptFile"
    /// 
    /// Fallback value for PhysxSchemaPhysxTriggerAPI::GetEnterScriptTypeAttr(), Fallback value for PhysxSchemaPhysxTriggerAPI::GetLeaveScriptTypeAttr()
    const TfToken scriptFile;
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
    /// \brief "Vertices"
    /// 
    /// Possible value for PhysxSchemaPhysxPhysicsAttachment::GetFilterType0Attr(), Possible value for PhysxSchemaPhysxPhysicsAttachment::GetFilterType1Attr()
    const TfToken Vertices;
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
    /// \brief "PhysxAutoAttachmentAPI"
    /// 
    /// Schema identifer and family for PhysxSchemaPhysxAutoAttachmentAPI
    const TfToken PhysxAutoAttachmentAPI;
    /// \brief "PhysxAutoParticleClothAPI"
    /// 
    /// Schema identifer and family for PhysxSchemaPhysxAutoParticleClothAPI
    const TfToken PhysxAutoParticleClothAPI;
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
    /// \brief "PhysxDeformableAPI"
    /// 
    /// Schema identifer and family for PhysxSchemaPhysxDeformableAPI
    const TfToken PhysxDeformableAPI;
    /// \brief "PhysxDeformableBodyAPI"
    /// 
    /// Schema identifer and family for PhysxSchemaPhysxDeformableBodyAPI
    const TfToken PhysxDeformableBodyAPI;
    /// \brief "PhysxDeformableBodyMaterialAPI"
    /// 
    /// Schema identifer and family for PhysxSchemaPhysxDeformableBodyMaterialAPI
    const TfToken PhysxDeformableBodyMaterialAPI;
    /// \brief "PhysxDeformableSurfaceAPI"
    /// 
    /// Schema identifer and family for PhysxSchemaPhysxDeformableSurfaceAPI
    const TfToken PhysxDeformableSurfaceAPI;
    /// \brief "PhysxDeformableSurfaceMaterialAPI"
    /// 
    /// Schema identifer and family for PhysxSchemaPhysxDeformableSurfaceMaterialAPI
    const TfToken PhysxDeformableSurfaceMaterialAPI;
    /// \brief "PhysxDiffuseParticlesAPI"
    /// 
    /// Schema identifer and family for PhysxSchemaPhysxDiffuseParticlesAPI
    const TfToken PhysxDiffuseParticlesAPI;
    /// \brief "PhysxForceAPI"
    /// 
    /// Schema identifer and family for PhysxSchemaPhysxForceAPI
    const TfToken PhysxForceAPI;
    /// \brief "PhysxJointAPI"
    /// 
    /// Schema identifer and family for PhysxSchemaPhysxJointAPI
    const TfToken PhysxJointAPI;
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
    /// \brief "PhysxParticleClothAPI"
    /// 
    /// Schema identifer and family for PhysxSchemaPhysxParticleClothAPI
    const TfToken PhysxParticleClothAPI;
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
    /// \brief "PhysxPhysicsAttachment"
    /// 
    /// Schema identifer and family for PhysxSchemaPhysxPhysicsAttachment
    const TfToken PhysxPhysicsAttachment;
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
    /// \brief "PhysxResidualReportingAPI"
    /// 
    /// Schema identifer and family for PhysxSchemaPhysxResidualReportingAPI
    const TfToken PhysxResidualReportingAPI;
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
    /// \brief "TetrahedralMesh"
    /// 
    /// Schema identifer and family for PhysxSchemaTetrahedralMesh
    const TfToken TetrahedralMesh;
    /// A vector of all of the tokens listed above.
    const std::vector<TfToken> allTokens;
};

/// \var PhysxSchemaTokens
///
/// A global variable with static, efficient \link TfToken TfTokens\endlink
/// for use in all public USD API.  \sa PhysxSchemaTokensType
extern PHYSXSCHEMA_API TfStaticData<PhysxSchemaTokensType> PhysxSchemaTokens;

PXR_NAMESPACE_CLOSE_SCOPE

#endif
