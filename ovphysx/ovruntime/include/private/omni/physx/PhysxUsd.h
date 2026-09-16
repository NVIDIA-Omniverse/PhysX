// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-PARSE-SHAPE-002
 * @covers AC-2 AC-3
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27 AC-28
 */

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>
#include <carb/extras/Hash.h>

#include <omni/physx/PhysxCookingParams.h>
#include <omni/physx/MeshKey.h>
#include <omni/physx/ObjectId.h>

#include <foundation/PxMat44.h>              // PxMat44d

#include <omni/physics/parse/Handles.h>      // ObjectKey, TokenId, BufferHandle
#include <omni/physics/parse/Math.h>         // Matrix4d
#include <omni/physics/parse/Descriptors.h>  // ObjectType, ObjectCategory, PhysxObjectDesc, body descs

#include <map>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>
// Is this really the way we include PI in carbonite...
#define _USE_MATH_DEFINES
#include <math.h>


#if !defined FLT_MAX
#    define FLT_MAX (1000000.0f)
#endif

// `omni::physics::parse` (in `Descriptors.h`) is the canonical home for
// descriptor types. This header re-exports the parse-lib types into
// `omni::physx::usdparser::` via `using` aliases so legacy consumer code
// keeps compiling, and additionally defines the descriptor variants that
// still differ from their parse-lib mirror — almost always because the
// consumer-side bookkeeping keys by `PXR_NS::SdfPath` where parse-lib
// uses `ObjectKey`. Collapsing those into `using` aliases tracks under
// ADR-0008. New code should include
// `omni/physics/parse/Descriptors.h` directly and use the parse-lib
// namespace.
namespace omni
{
namespace physx
{
namespace usdparser
{
inline carb::Float3 scaleToSignScale(const carb::Float3& scale)
{
    return carb::Float3{ scale.x >= 0 ? 1.f : -1.f, scale.y >= 0 ? 1.f : -1.f, scale.z >= 0 ? 1.f : -1.f };
}
using ErrorCode = ::omni::physics::parse::ErrorCode;

using ObjectIdPair = std::pair<ObjectId, ObjectId>;
using ObjectIdPairVector = std::vector<ObjectIdPair>;

// ObjectType / ObjectCategory live in omni::physics::parse and are re-exported
// here so existing usdparser::* code keeps compiling.  Per-value using-decls
// bring the enumerators (eShape, eDynamicBody, ...) into usdparser:: scope so
// `desc.type == eDynamicBody` works whether the caller writes
// `usdparser::eDynamicBody` or just `eDynamicBody` from inside the namespace.
using ObjectType = ::omni::physics::parse::ObjectType;
using ObjectCategory = ::omni::physics::parse::ObjectCategory;

using ::omni::physics::parse::eUndefined;
using ::omni::physics::parse::eCategoryMask;

using ::omni::physics::parse::eShape;
using ::omni::physics::parse::eSphereShape;
using ::omni::physics::parse::eBoxShape;
using ::omni::physics::parse::eCapsuleShape;
using ::omni::physics::parse::eCylinderShape;
using ::omni::physics::parse::eConeShape;
using ::omni::physics::parse::eConvexMeshShape;
using ::omni::physics::parse::eConvexMeshDecompositionShape;
using ::omni::physics::parse::eBoundingSphereShape;
using ::omni::physics::parse::eBoundingBoxShape;
using ::omni::physics::parse::eTriangleMeshShape;
using ::omni::physics::parse::ePlaneShape;
using ::omni::physics::parse::eCustomShape;
using ::omni::physics::parse::eSpherePointsShape;

using ::omni::physics::parse::eBody;
using ::omni::physics::parse::eDynamicBody;
using ::omni::physics::parse::eStaticBody;

using ::omni::physics::parse::eJoint;
using ::omni::physics::parse::eJointFixed;
using ::omni::physics::parse::eJointRevolute;
using ::omni::physics::parse::eJointPrismatic;
using ::omni::physics::parse::eJointSpherical;
using ::omni::physics::parse::eJointDistance;
using ::omni::physics::parse::eJointD6;
using ::omni::physics::parse::eJointGear;
using ::omni::physics::parse::eJointRackAndPinion;
using ::omni::physics::parse::eJointCustom;

using ::omni::physics::parse::eScene;
using ::omni::physics::parse::eMaterial;

using ::omni::physics::parse::eArticulationLink;
using ::omni::physics::parse::eArticulation;
using ::omni::physics::parse::eArticulationJoint;
using ::omni::physics::parse::eArticulationRootJoint;

using ::omni::physics::parse::eFilteredPair;
using ::omni::physics::parse::eCollisionGroup;

using ::omni::physics::parse::eCct;
using ::omni::physics::parse::eCapsuleCct;

using ::omni::physics::parse::eVehicle;
using ::omni::physics::parse::eVehicleContext;
using ::omni::physics::parse::eVehicleTireFrictionTable;
using ::omni::physics::parse::eVehicleController;
using ::omni::physics::parse::eVehicleControllerStandard;
using ::omni::physics::parse::eVehicleControllerTank;
using ::omni::physics::parse::eVehicleEngine;
using ::omni::physics::parse::eVehicleSuspension;
using ::omni::physics::parse::eVehicleTire;
using ::omni::physics::parse::eVehicleWheel;
using ::omni::physics::parse::eVehicleWheelAttachment;
using ::omni::physics::parse::eVehicleWheelController;

using ::omni::physics::parse::eVehicleDrive;
using ::omni::physics::parse::eVehicleDriveBasic;
using ::omni::physics::parse::eVehicleDriveStandard;

using ::omni::physics::parse::eParticleSystem;
using ::omni::physics::parse::eParticleSet;
using ::omni::physics::parse::ePointInstancedBody;
using ::omni::physics::parse::eInfiniteVoxelMap;

using ::omni::physics::parse::ePBDMaterial;
using ::omni::physics::parse::ePhysxForce;

using ::omni::physics::parse::eTendons;
using ::omni::physics::parse::eTendonFixed;
using ::omni::physics::parse::eTendonAxis;
using ::omni::physics::parse::eTendonAxisUI;
using ::omni::physics::parse::eTendonAttachment;
using ::omni::physics::parse::eTendonAttachmentLeaf;
using ::omni::physics::parse::eTendonAttachmentRoot;
using ::omni::physics::parse::eTendonAttachmentUI;

using ::omni::physics::parse::eMimicJointRotX;
using ::omni::physics::parse::eMimicJointRotY;
using ::omni::physics::parse::eMimicJointRotZ;

using ::omni::physics::parse::eDeformableMaterial;
using ::omni::physics::parse::eSurfaceDeformableMaterial;

using ::omni::physics::parse::eDeformableBody;
using ::omni::physics::parse::eVolumeDeformableBody;
using ::omni::physics::parse::eSurfaceDeformableBody;

using ::omni::physics::parse::eDeformableAttachment;
using ::omni::physics::parse::eAttachmentVtxVtx;
using ::omni::physics::parse::eAttachmentVtxTri;
using ::omni::physics::parse::eAttachmentVtxTet;
using ::omni::physics::parse::eAttachmentVtxXform;
using ::omni::physics::parse::eAttachmentTetXform;

using ::omni::physics::parse::eDeformableCollisionFilter;
using ::omni::physics::parse::eXformActor;

using ::omni::physics::parse::eNewtonMimicJoint;

using BodyType = ::omni::physics::parse::BodyType;
using ::omni::physics::parse::eDynamic;
using ::omni::physics::parse::eStatic;
using ::omni::physics::parse::eKinematic;

using CctShapeType = ::omni::physics::parse::CctShapeType;
using ::omni::physics::parse::eBox;
using ::omni::physics::parse::eCapsule;

using Axis = ::omni::physics::parse::Axis;
using ::omni::physics::parse::eX;
using ::omni::physics::parse::eY;
using ::omni::physics::parse::eZ;

using JointAxis = ::omni::physics::parse::JointAxis;
using ::omni::physics::parse::eDistance;
using ::omni::physics::parse::eTransX;
using ::omni::physics::parse::eTransY;
using ::omni::physics::parse::eTransZ;
using ::omni::physics::parse::eRotX;
using ::omni::physics::parse::eRotY;
using ::omni::physics::parse::eRotZ;

using ArticulationJointType = ::omni::physics::parse::ArticulationJointType;
using ::omni::physics::parse::eStandardJoint;
using ::omni::physics::parse::eMaximalJoint;

using VehicleUpdateMode = ::omni::physics::parse::VehicleUpdateMode;
using ::omni::physics::parse::eVelocityChange;
using ::omni::physics::parse::eAcceleration;

using CollisionSystem = ::omni::physics::parse::CollisionSystem;
using ::omni::physics::parse::ePCM;
using ::omni::physics::parse::eSAT;

using SolverType = ::omni::physics::parse::SolverType;
using ::omni::physics::parse::ePGS;
using ::omni::physics::parse::eTGS;

using BroadphaseType = ::omni::physics::parse::BroadphaseType;
using ::omni::physics::parse::eMBP;
using ::omni::physics::parse::eSAP;
using ::omni::physics::parse::eGPU;

using SceneUpdateType = ::omni::physics::parse::SceneUpdateType;
using ::omni::physics::parse::eSynchronous;
using ::omni::physics::parse::eAsynchronous;
using ::omni::physics::parse::eDisabled;

using CombineMode = ::omni::physics::parse::CombineMode;
using ::omni::physics::parse::eAverage;
using ::omni::physics::parse::eMin;
using ::omni::physics::parse::eMultiply;
using ::omni::physics::parse::eMax;

// ObjectInstance: instancerKey/protoKey are ObjectKey (were instancerPath/
// protoPath, SdfPath); runtime sites needing SdfPath go through
// AttachedStage::pathFor().
using ObjectInstance = ::omni::physics::parse::ObjectInstance;

// PhysxObjectDesc is defined once in omni::physics::parse and re-exported here.
// Other usdparser:: descriptor structs (Material, Scene, Joint, ...) inherit
// from this alias which now resolves to parse::PhysxObjectDesc.
using PhysxObjectDesc = ::omni::physics::parse::PhysxObjectDesc;

// PhysxMaterialDesc: `materialPath` is an ObjectKey; runtime sites resolve
// it to an SdfPath via AttachedStage::pathFor().
using PhysxMaterialDesc = ::omni::physics::parse::PhysxMaterialDesc;

// Deformable + PBD material descriptors: materialPath is ObjectKey; runtime
// sites that need an SdfPath go through AttachedStage::pathFor().
using PhysxDeformableMaterialDesc        = ::omni::physics::parse::PhysxDeformableMaterialDesc;
using PhysxSurfaceDeformableMaterialDesc = ::omni::physics::parse::PhysxSurfaceDeformableMaterialDesc;
using PBDMaterialDesc                    = ::omni::physics::parse::PBDMaterialDesc;

// PhysxSceneDesc: `quasistaticActors` is `unordered_set<ObjectKey>`; runtime
// sites go through AttachedStage::pathFor() when they need an SdfPath. The
// nested default-material descriptors (PBDMaterialDesc,
// PhysxDeformableMaterialDesc, PhysxSurfaceDeformableMaterialDesc) stay
// legacy structs since Scene only uses their fields, not materialPath.
using PhysxSceneDesc = ::omni::physics::parse::PhysxSceneDesc;

struct FilteredPairDesc : PhysxObjectDesc
{
    FilteredPairDesc()
    {
        type = eFilteredPair;
    }

    ObjectIdPairVector pairs;
};

// CollisionGroupDesc: groupId/filteredGroups (ObjectId-typed) are unchanged;
// the parse-lib twin additionally carries primKey/sourceFilteredGroups/
// sourceMembers (ObjectKey, source-side provenance from scanStage) that this
// consumer-side population path (setupCollisionGroups) does not populate.
using CollisionGroupDesc = ::omni::physics::parse::CollisionGroupDesc;

// Existing legacy code that derives from `PhysxShapeDesc`
// (CustomPhysxShapeDesc, MergeMeshPhysxShapeDesc, etc.) inherits
// transparently through the alias.
using PhysxShapeDesc = ::omni::physics::parse::PhysxShapeDesc;

// `customGeometryToken` (TfToken) became `customGeometryTokenHash` (size_t)
// -- the same content hash `PhysXCustomGeometryManager`'s map keys on.
using CustomPhysxShapeDesc = ::omni::physics::parse::CustomPhysxShapeDesc;

using SpherePhysxShapeDesc = ::omni::physics::parse::SpherePhysxShapeDesc;

// Storage is `std::vector<carb::Float3>` for points and
// `std::vector<int32_t>` for the index / face / hole vectors (not VtArray).
// `processMeshesToMerge` writes points via `GfVec3ToFloat3`; the cooking
// consumer reads `data() + size()` directly, no cast needed.
using MergeMeshDesc = ::omni::physics::parse::MergeMeshDesc;

// ConvexMeshPhysxShapeDesc / TriangleMeshPhysxShapeDesc /
// ConvexMeshDecompositionPhysxShapeDesc / SpherePointsPhysxShapeDesc remain
// legacy structs because their `meshPath` is still SdfPath; unifying them
// requires the same keyFor/pathFor boundary work done for `rigidBody` /
// `sourceGprim`.
using MergeMeshPhysxShapeDesc       = ::omni::physics::parse::MergeMeshPhysxShapeDesc;
using BoundingSpherePhysxShapeDesc  = ::omni::physics::parse::BoundingSpherePhysxShapeDesc;
using BoundingBoxPhysxShapeDesc     = ::omni::physics::parse::BoundingBoxPhysxShapeDesc;

using CapsulePhysxShapeDesc  = ::omni::physics::parse::CapsulePhysxShapeDesc;
using CylinderPhysxShapeDesc = ::omni::physics::parse::CylinderPhysxShapeDesc;
using ConePhysxShapeDesc     = ::omni::physics::parse::ConePhysxShapeDesc;
using BoxPhysxShapeDesc      = ::omni::physics::parse::BoxPhysxShapeDesc;

// InfiniteVoxelMapDesc: rootPrim is ObjectKey (was SdfPath); construct
// directly from the already-known ObjectKey at the scan-time call site.
using InfiniteVoxelMapDesc = ::omni::physics::parse::InfiniteVoxelMapDesc;

// meshPath is ObjectKey here (unlike the legacy shape descs above).
using ConvexMeshPhysxShapeDesc              = ::omni::physics::parse::ConvexMeshPhysxShapeDesc;
using TriangleMeshPhysxShapeDesc            = ::omni::physics::parse::TriangleMeshPhysxShapeDesc;
using ConvexMeshDecompositionPhysxShapeDesc = ::omni::physics::parse::ConvexMeshDecompositionPhysxShapeDesc;
using SpherePhysxPoint                      = ::omni::physics::parse::SpherePhysxPoint;
using SpherePointsPhysxShapeDesc            = ::omni::physics::parse::SpherePointsPhysxShapeDesc;

using PlanePhysxShapeDesc = ::omni::physics::parse::PlanePhysxShapeDesc;

// Rigid body descriptors are defined once in omni::physics::parse and
// re-exported here. Resolving the prim references (sourceGPrimPath /
// splinesCurvePrimPath, both ObjectKey) to SdfPath happens at the runtime
// boundary via AttachedStage::pathFor.
using PhysxRigidBodyDesc       = ::omni::physics::parse::PhysxRigidBodyDesc;
using StaticPhysxRigidBodyDesc = ::omni::physics::parse::StaticPhysxRigidBodyDesc;
using DynamicPhysxRigidBodyDesc = ::omni::physics::parse::DynamicPhysxRigidBodyDesc;

struct PointInstancedBodyDesc : PhysxObjectDesc
{
    PointInstancedBodyDesc()
    {
        type = ePointInstancedBody;
    }
};

struct PhysxForceDesc : PhysxObjectDesc
{
    PhysxForceDesc()
    {
        type = ePhysxForce;
    }

    carb::Float3 force;
    carb::Float3 torque;

    bool worldFrame;
    bool accelerationMode;

    bool enabled;

    carb::Float3 worldPos;

    carb::Float4 localRot;

    ObjectId body;
    ObjectId scene;
};

using PhysxJointLimit          = ::omni::physics::parse::PhysxJointLimit;
using PhysxJointDrive          = ::omni::physics::parse::PhysxJointDrive;
using PhysxJointAxisProperties = ::omni::physics::parse::PhysxJointAxisProperties;
using PhysicsJointState        = ::omni::physics::parse::PhysicsJointState;


// PhysxArticulationDesc: rootPrim / staticRootBodyPrim / fixBasePath are
// ObjectKey; articulatedJoints / articulatedBodies are
// unordered_set<ObjectKey>. Runtime sites needing SdfPath go through
// AttachedStage::pathFor().
using PhysxArticulationDesc = ::omni::physics::parse::PhysxArticulationDesc;

// Tendon descriptor family: jointPath -> jointKey, link0/link1, parentPath ->
// parentKey, linkPath -> linkKey (all ObjectKey); instanceToken/parentToken
// are TokenId (were TfToken). Runtime sites needing SdfPath/TfToken go
// through AttachedStage::pathFor()/tfTokenFor().
using PhysxTendonAxisDesc           = ::omni::physics::parse::PhysxTendonAxisDesc;
using PhysxTendonFixedDesc          = ::omni::physics::parse::PhysxTendonFixedDesc;
using PhysxTendonAttachmentDesc     = ::omni::physics::parse::PhysxTendonAttachmentDesc;
using PhysxTendonSpatialDesc        = ::omni::physics::parse::PhysxTendonSpatialDesc;
using PhysxTendonAttachmentLeafDesc = ::omni::physics::parse::PhysxTendonAttachmentLeafDesc;

// MimicJointDesc: mimicJointPath/referenceJointPath -> mimicJointKey/
// referenceJointKey (ObjectKey).
using MimicJointDesc = ::omni::physics::parse::MimicJointDesc;

typedef std::vector<std::pair<JointAxis, PhysxJointLimit>> JointLimits;
// Joint descriptor family: path-typed members (jointPrimPath, rel0/1,
// body0/1) are ObjectKey; runtime sites needing SdfPath go through
// AttachedStage::pathFor().
using JointLimits             = ::omni::physics::parse::JointLimits;
using JointDrives             = ::omni::physics::parse::JointDrives;
using JointStates             = ::omni::physics::parse::JointStates;
using JointAxisProperties     = ::omni::physics::parse::JointAxisProperties;
using PhysxJointDesc          = ::omni::physics::parse::PhysxJointDesc;
using FixedPhysxJointDesc     = ::omni::physics::parse::FixedPhysxJointDesc;
using D6PhysxJointDesc        = ::omni::physics::parse::D6PhysxJointDesc;
using PrismaticPhysxJointDesc = ::omni::physics::parse::PrismaticPhysxJointDesc;
using SphericalPhysxJointDesc = ::omni::physics::parse::SphericalPhysxJointDesc;
using RevolutePhysxJointDesc  = ::omni::physics::parse::RevolutePhysxJointDesc;
using DistancePhysxJointDesc  = ::omni::physics::parse::DistancePhysxJointDesc;

// GearPhysxJointDesc: hingePrimPath0/1 are ObjectKey. RackPhysxJointDesc:
// hingePrimPath -> hingePrimKey, prismaticPrimPath -> prismaticPrimKey
// (ObjectKey). CustomPhysxJointDesc: customJointToken is TokenId (was
// TfToken).
using GearPhysxJointDesc   = ::omni::physics::parse::GearPhysxJointDesc;
using RackPhysxJointDesc   = ::omni::physics::parse::RackPhysxJointDesc;
using CustomPhysxJointDesc = ::omni::physics::parse::CustomPhysxJointDesc;


struct PhysxArticulationLinkDesc : DynamicPhysxRigidBodyDesc
{
    PhysxArticulationLinkDesc()
        : DynamicPhysxRigidBodyDesc(),
          articulation(kInvalidObjectId),
          parent(kInvalidObjectId),
          articulationJointType(eStandardJoint),
          articulationJoint(nullptr)
    {
        type = eArticulationLink;
    }

    ObjectId articulation;
    ObjectId parent;
    ArticulationJointType articulationJointType;
    const PhysxJointDesc* articulationJoint;
};

// PhysxDeformableBodyDesc: simMeshPath -> simMeshKey, collisionMeshPath ->
// collisionMeshKey, skinGeomPaths (element type ObjectKey, same field name),
// cookingSrcMeshPath -> cookingSrcMeshKey (all ObjectKey); the *BindPoseToken
// fields and skinGeomBindPoseTokens are TokenId (were TfToken). Runtime sites
// needing SdfPath/TfToken go through AttachedStage::pathFor()/tfTokenFor().
using PhysxDeformableBodyDesc        = ::omni::physics::parse::PhysxDeformableBodyDesc;
using PhysxVolumeDeformableBodyDesc  = ::omni::physics::parse::PhysxVolumeDeformableBodyDesc;
using PhysxSurfaceDeformableBodyDesc = ::omni::physics::parse::PhysxSurfaceDeformableBodyDesc;

// TireFrictionTableDesc: path -> key, materialPaths element type is ObjectKey
// (was SdfPath).
using TireFrictionTableDesc = ::omni::physics::parse::TireFrictionTableDesc;

// Vehicle descriptor family: `path` -> `key` throughout; WheelAttachmentDesc's
// `collisionGroupPath`/`shapePath` -> `collisionGroupKey`/`shapeKey`;
// TireDesc's `frictionTablePath` -> `frictionTableKey` (all ObjectKey). The
// vehicle consumer's own bookkeeping (e.g. `Vehicle::mWheelAttachmentByPath`)
// is updated in lockstep at its call sites. Fields tagged `// deprecated` on
// the parse-lib twin refer to the corresponding USD attribute being
// deprecated -- Vehicle.cpp's `parseWheelAttachment` logs a warning when the
// deprecated attribute is authored; the field itself is still populated and
// acted on for back-compat with older USD content (see the `eHAS_*` state
// bits raised in Vehicle.cpp and read in VehicleGenerator.cpp).
using WheelDesc            = ::omni::physics::parse::WheelDesc;
using TireDesc              = ::omni::physics::parse::TireDesc;
using SuspensionDesc        = ::omni::physics::parse::SuspensionDesc;
using SuspensionComplianceDesc = ::omni::physics::parse::SuspensionComplianceDesc;
using WheelAttachmentDesc   = ::omni::physics::parse::WheelAttachmentDesc;
using WheelControllerDesc   = ::omni::physics::parse::WheelControllerDesc;
using EngineDesc             = ::omni::physics::parse::EngineDesc;

using GearsDesc = ::omni::physics::parse::GearsDesc;

using AutoGearBoxDesc = ::omni::physics::parse::AutoGearBoxDesc;

using ClutchDesc = ::omni::physics::parse::ClutchDesc;

using NonlinearCmdResponseDesc = ::omni::physics::parse::NonlinearCmdResponseDesc;

// DriveDesc: collapsed alongside DriveBasicDesc so the inheritance chain
// stays sound -- DriveStandardDesc (below, not itself retyped: it has no
// path-typed fields) derives from this same name.
using DriveDesc = ::omni::physics::parse::DriveDesc;

// DriveBasicDesc: path -> key (ObjectKey).
using DriveBasicDesc = ::omni::physics::parse::DriveBasicDesc;

struct DriveStandardDesc : public DriveDesc
{
    DriveStandardDesc()
    {
        type = eVehicleDriveStandard;
    }

    // note: both object IDs and descriptors are provided though one would suffice -- IDs are
    //       needed to fit into the USD prim update listener logic, which passes objects into
    //       create calls.

    EngineDesc* engine;
    ObjectId engineId;

    const GearsDesc* gears;
    const AutoGearBoxDesc* autoGearBox;
    const ClutchDesc* clutch;
};

using DifferentialDesc = ::omni::physics::parse::DifferentialDesc;

using MultiWheelDifferentialDesc = ::omni::physics::parse::MultiWheelDifferentialDesc;

using TankDifferentialDesc = ::omni::physics::parse::TankDifferentialDesc;

using BrakesDesc = ::omni::physics::parse::BrakesDesc;

struct SteeringDesc
{
    enum Type
    {
        eBasic,
        eAckermann
    };

    NonlinearCmdResponseDesc* nonlinearCmdResponse;
    Type type;
};

struct SteeringBasicDesc : SteeringDesc
{
    SteeringBasicDesc()
    {
        type = eBasic;
    }

    std::vector<int> wheels;
    std::vector<float> angleMultipliers;
    float maxSteerAngle;
};

struct SteeringAckermannDesc : SteeringDesc
{
    SteeringAckermannDesc()
    {
        type = eAckermann;
    }

    int wheel0;
    int wheel1;
    float maxSteerAngle;
    float wheelBase;
    float trackWidth;
    float strength;
};

// VehicleContextDesc: scenePath -> sceneKey (ObjectKey).
using VehicleContextDesc = ::omni::physics::parse::VehicleContextDesc;

struct VehicleDesc : public PhysxObjectDesc
{
    enum QueryType
    {
        eRAYCAST,
        eSWEEP
    };

    VehicleDesc()
    {
        type = eVehicle;
    }

    // Computed or internal data.
    ObjectId bodyId;

    std::vector<WheelAttachmentDesc> wheelAttachments;
    std::vector<WheelControllerDesc> wheelControllers;

    std::vector<const BrakesDesc*> brakes;

    SteeringDesc* steering;

    DriveDesc* drive;
    MultiWheelDifferentialDesc* differential;

    carb::Float3 scale; // total scale

    float subStepThresholdLongitudinalSpeed;
    int lowForwardSpeedSubStepCount;
    int highForwardSpeedSubStepCount;

    float minLongitudinalSlipDenominator; // deprecated
    float minPassiveLongitudinalSlipDenominator;
    float minActiveLongitudinalSlipDenominator;
    float minLateralSlipDenominator;

    float longitudinalStickyTireThresholdSpeed;
    float longitudinalStickyTireThresholdTime;
    float longitudinalStickyTireDamping;
    float lateralStickyTireThresholdSpeed;
    float lateralStickyTireThresholdTime;
    float lateralStickyTireDamping;

    bool enabled;

    uint8_t queryType;

    bool hasUserDefinedSprungMassValues;
    bool hasUserDefinedMaxDroopValues; // deprecated
    bool hasUserDefinedRestLoadValues;
    bool isUsingDeprecatedLatStiffY; // deprecated
    bool referenceFrameIsCenterOfMass; // deprecated
    bool limitSuspensionExpansionVelocity;

    static constexpr uint32_t maxNumberOfWheels = 20;
};

struct VehicleControllerDesc : public PhysxObjectDesc
{
    VehicleControllerDesc()
    {
        type = eVehicleControllerStandard;
    }

    float accelerator;
    float brake0;
    float brake1;
    float brake; // deprecated
    float handbrake; // deprecated
    float steer;
    float steerLeft; // deprecated
    float steerRight; // deprecated
    int targetGear;

    static constexpr int automaticGearValue = 0xff;
};

struct VehicleTankControllerDesc : public VehicleControllerDesc
{
    VehicleTankControllerDesc()
    {
        type = eVehicleControllerTank;
    }

    float thrust0;
    float thrust1;
};

// Particle descriptor family: scenePath -> sceneKey, systemPath -> systemKey,
// primPath -> primKey, particleSystemPath -> particleSystemKey,
// particleSetPath -> particleSetKey, filteredCollisions element type is
// ObjectKey (all were SdfPath). Runtime sites needing SdfPath go through
// AttachedStage::pathFor().
using ParticleSystemDesc    = ::omni::physics::parse::ParticleSystemDesc;
using ParticleDesc          = ::omni::physics::parse::ParticleDesc;
using ParticleSetDesc       = ::omni::physics::parse::ParticleSetDesc;
using ParticleSamplingDesc  = ::omni::physics::parse::ParticleSamplingDesc;
using ParticleIsosurfaceDesc = ::omni::physics::parse::ParticleIsosurfaceDesc;
using ParticleSmoothingDesc  = ::omni::physics::parse::ParticleSmoothingDesc;
using ParticleAnisotropyDesc = ::omni::physics::parse::ParticleAnisotropyDesc;

// PhysxDeformableAttachmentDesc / PhysxDeformableCollisionFilterDesc: src0/
// src1 are ObjectKey (were SdfPath); the parse-lib twins additionally carry
// primKey (the attachment/filter prim's own source-side identity).
using PhysxDeformableAttachmentDesc      = ::omni::physics::parse::PhysxDeformableAttachmentDesc;
using PhysxDeformableCollisionFilterDesc = ::omni::physics::parse::PhysxDeformableCollisionFilterDesc;

// The CCT parser lives in omni.physics.parse / ScannedStage::ccts; the
// engine integration in usdInterface/UsdInterface.cpp consumes the same
// descriptor type.
using CctDesc        = ::omni::physics::parse::CctDesc;
using CapsuleCctDesc = ::omni::physics::parse::CapsuleCctDesc;

} // namespace usdparser
} // namespace physx
} // namespace omni
