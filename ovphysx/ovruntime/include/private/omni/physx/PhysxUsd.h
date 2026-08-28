// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-PARSE-SHAPE-002
 * @covers AC-2 AC-3
 */

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>
#include <carb/extras/Hash.h>

#include <omni/physx/PhysxCookingParams.h>
#include <omni/physx/MeshKey.h>
#include <omni/physx/ObjectId.h>

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

struct ObjectInstance
{
    PXR_NS::SdfPath instancerPath;
    uint32_t index;
    PXR_NS::SdfPath protoPath;
    bool isExclusive;
    ::omni::physics::parse::Matrix4d protoTransformInverse;
    bool hasProtoTransformInverse = false;
};

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

struct CollisionGroupDesc : PhysxObjectDesc
{
    CollisionGroupDesc()
    {
        type = eCollisionGroup;
    }

    ObjectId groupId;
    std::vector<ObjectId> filteredGroups;
};

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

struct InfiniteVoxelMapDesc : PhysxObjectDesc
{
    InfiniteVoxelMapDesc(const PXR_NS::SdfPath path) : PhysxObjectDesc(), rootPrim{ path }
    {
        type = eInfiniteVoxelMap;
    };

    const PXR_NS::SdfPath rootPrim;
};

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

struct PhysxTendonAxisDesc : PhysxObjectDesc
{
    PhysxTendonAxisDesc()
        : gearings(1u), forceCoefficients(1u), axes(1u), parentAxisId(kInvalidObjectId), wasVisited(false)
    {
        type = eTendonAxis;
        forceCoefficients[0] = 1.0f;
    }

    PXR_NS::TfToken instanceToken;
    PXR_NS::SdfPath jointPath;

    PXR_NS::SdfPath link0;
    PXR_NS::SdfPath link1;

    std::vector<float> gearings;
    std::vector<float> forceCoefficients;
    std::vector<JointAxis> axes;

    ObjectId parentAxisId;

    bool wasVisited;
};

struct PhysxTendonFixedDesc : PhysxObjectDesc
{
    PhysxTendonFixedDesc()
        : stiffness(0.f),
          damping(0.f),
          restLength(0.f),
          offset(0.f),
          limitStiffness(0.f),
          lowLimit(-FLT_MAX),
          highLimit(FLT_MAX),
          isEnabled(true),
          rootAxis(nullptr)
    {
        type = eTendonFixed;
    }

    PXR_NS::TfToken instanceToken;
    PXR_NS::SdfPath jointPath;

    float stiffness;
    float damping;
    float restLength;
    float offset;

    float limitStiffness;
    float lowLimit;
    float highLimit;

    bool isEnabled;
    PhysxTendonAxisDesc* rootAxis;
};

struct PhysxTendonAttachmentDesc : PhysxObjectDesc
{
    PhysxTendonAttachmentDesc() : gearing(1.0f), localPos({ 0.f, 0.f, 0.f }), parentId(kInvalidObjectId)
    {
        type = eTendonAttachment;
    }

    float gearing;
    carb::Float3 localPos;
    PXR_NS::SdfPath parentPath;
    PXR_NS::SdfPath linkPath;
    PXR_NS::TfToken parentToken;
    PXR_NS::TfToken instanceToken;
    ObjectId parentId;
};

struct PhysxTendonSpatialDesc : PhysxTendonAttachmentDesc
{
    PhysxTendonSpatialDesc() : stiffness(0.f), damping(0.f), limitStiffness(0.f), offset(0.f), isEnabled(true)
    {
        type = eTendonAttachmentRoot;
    }

    float stiffness;
    float damping;
    float limitStiffness;
    float offset;
    bool isEnabled;
};

struct PhysxTendonAttachmentLeafDesc : PhysxTendonAttachmentDesc
{
    PhysxTendonAttachmentLeafDesc() : lowLimit(-FLT_MAX), highLimit(FLT_MAX), restLength(-FLT_MAX)
    {
        type = eTendonAttachmentLeaf;
    }
    float lowLimit;
    float highLimit;
    float restLength;
};

struct MimicJointDesc : PhysxObjectDesc
{
    MimicJointDesc()
    {
        type = eUndefined;
        // the type depends on the axis it should operate on and that will only be known at parsing time.
    }

    // the joint that should mimic another joint
    PXR_NS::SdfPath mimicJointPath;
    ObjectId mimicJointId;
    int mimicJointAxis; // eDEFAULT_AXIS for revolute or prismatic, else the enum JointAxis

    // the joint that should get mimicked
    PXR_NS::SdfPath referenceJointPath;
    ObjectId referenceJointId;
    int referenceJointAxis; // see mimicJointAxis

    float gearing;
    float offset;
    float naturalFrequency;
    float dampingRatio;

    static const int eDEFAULT_AXIS = -1;
};

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

struct GearPhysxJointDesc : public PhysxJointDesc
{
    GearPhysxJointDesc() : gearRatio(0.0f)
    {
        type = eJointGear;
    }

    PXR_NS::SdfPath hingePrimPath0;
    PXR_NS::SdfPath hingePrimPath1;
    float gearRatio;
};

struct RackPhysxJointDesc : public PhysxJointDesc
{
    RackPhysxJointDesc() : ratio(0.0f)
    {
        type = eJointRackAndPinion;
    }

    PXR_NS::SdfPath hingePrimPath;
    PXR_NS::SdfPath prismaticPrimPath;
    float ratio;
};

struct CustomPhysxJointDesc : public PhysxJointDesc
{
    CustomPhysxJointDesc()
    {
        type = eJointCustom;
    }

    PXR_NS::TfToken customJointToken;
};


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

struct PhysxDeformableBodyDesc : public PhysxObjectDesc
{
    PhysxDeformableBodyDesc()
        : bodyEnabled(false),
          kinematicBody(false),
          startsAsleep(false),
          sceneId(kInvalidObjectId),
          transform(1.0f),
          mass(-1.0f),
          simMeshMaterial(kInvalidObjectId)
    {
    }

    // as opposed to rigid bodies, we need to pass on the bodyEnabled flag
    // so we can support live update. Rigid bodies have a dynamic and a static type instead.
    bool bodyEnabled;
    bool kinematicBody;
    bool startsAsleep;
    bool enableSpeculativeCCD;
    bool selfCollision;
    bool disableGravity;
    float sleepThreshold;
    float linearDamping;
    float maxLinearVelocity;
    float settlingThreshold;
    float settlingDamping;
    float maxDepenetrationVelocity;
    float contactOffset;
    float restOffset;
    float selfCollisionFilterDistance;
    uint32_t solverPositionIterationCount; // TODO switch schema to int for consistency with rigid bodies.
    ObjectId sceneId;

    // mesh generation parameter
    bool isAutoMeshSimplificationEnabled;
    bool isAutoRemeshingEnabled;
    bool hasAutoForceConforming;
    uint32_t autoRemeshingResolution;
    uint32_t autoTriangleTargetCount;

    PXR_NS::GfMatrix4d transform;
    float mass;

    PXR_NS::SdfPath simMeshPath;
    ObjectId simMeshMaterial;
    PXR_NS::TfToken simMeshBindPoseToken;
    bool simMeshLeftHandedOrientation;

    PXR_NS::SdfPath collisionMeshPath;
    PXR_NS::TfToken collisionMeshBindPoseToken;
    bool collisionMeshLeftHandedOrientation;
    ObjectId collisionGroup;

    PXR_NS::SdfPathVector skinGeomPaths;
    PXR_NS::TfTokenVector skinGeomBindPoseTokens; // same size as skinGeomPaths

    PXR_NS::SdfPath cookingSrcMeshPath;
    PXR_NS::TfToken cookingSrcMeshBindPoseToken;

    bool hasAutoAPI;
};

struct PhysxVolumeDeformableBodyDesc : public PhysxDeformableBodyDesc
{
    PhysxVolumeDeformableBodyDesc()
    {
        type = eVolumeDeformableBody;
    }

    // mesh generation parameter
    bool isAutoHexahedralMeshEnabled;
    uint32_t autoHexahedralResolution;
};

struct PhysxSurfaceDeformableBodyDesc : public PhysxDeformableBodyDesc
{
    PhysxSurfaceDeformableBodyDesc()
    {
        type = eSurfaceDeformableBody;
    }

    // bending
    PXR_NS::TfToken restBendAnglesDefault;

    // collision substepping
    uint32_t collisionPairUpdateFrequency;
    uint32_t collisionIterationMultiplier;
};

struct TireFrictionTableDesc : public PhysxObjectDesc
{
    TireFrictionTableDesc()
    {
        type = eVehicleTireFrictionTable;
    }

    PXR_NS::SdfPath path;
    std::vector<PXR_NS::SdfPath> materialPaths;
    std::vector<ObjectId> materialIds;
    std::vector<float> frictionValues;
    float defaultFrictionValue;
};

struct WheelDesc : public PhysxObjectDesc
{
    WheelDesc()
    {
        type = eVehicleWheel;
    }

    PXR_NS::SdfPath path;

    float radius;
    float width;
    float mass;
    float moi;
    float dampingRate;
    float maxBrakeTorque; // deprecated
    float maxHandBrakeTorque; // deprecated
    float maxSteerAngle; // deprecated
    float toeAngle; // deprecated
};

struct TireDesc : public PhysxObjectDesc
{
    TireDesc()
    {
        type = eVehicleTire;
    }

    PXR_NS::SdfPath path;

    float latStiffX; // deprecated
    float latStiffY; // deprecated
    carb::Float2 lateralStiffnessGraph;
    float longitudinalStiffnessPerUnitGravity; // deprecated
    float longitudinalStiffness;
    float camberStiffnessPerUnitGravity; // deprecated
    float camberStiffness;
    carb::Float2 frictionVsSlipGraph[3];
    ObjectId frictionTableId;
    PXR_NS::SdfPath frictionTablePath;
    float restLoad;
};

struct SuspensionDesc : public PhysxObjectDesc
{
    SuspensionDesc()
    {
        type = eVehicleSuspension;
    }

    PXR_NS::SdfPath path;

    float springStrength;
    float springDamperRate;
    float travelDistance;
    float maxCompression; // deprecated
    float maxDroop; // deprecated
    float camberAtRest; // deprecated
    float camberAtMaxCompression; // deprecated
    float camberAtMaxDroop; // deprecated
    float sprungMass;
};

using SuspensionComplianceDesc = ::omni::physics::parse::SuspensionComplianceDesc;

// Consumer-side mirror of `omni::physics::parse::WheelAttachmentDesc`
// (in `Descriptors.h`). Both structs carry the same fields in the same
// order; the divergence is:
//
//   * `path`, `collisionGroupPath`, `shapePath` are `PXR_NS::SdfPath`
//     here and correspond to `key`, `collisionGroupKey`, `shapeKey`
//     (`parse::ObjectKey`) in the parse-lib version. The vehicle
//     consumer's bookkeeping (e.g. `Vehicle::mWheelAttachmentByPath`)
//     still keys by SdfPath; flipping it to ObjectKey so this struct
//     can collapse into a `using` alias of the parse-lib version is
//     tracked separately (ADR-0008 target).
//
//   * Fields tagged `// deprecated` refer to the corresponding USD
//     attribute being deprecated — Vehicle.cpp's `parseWheelAttachment`
//     logs a warning when the deprecated attribute is authored. The
//     field itself is still populated and acted on for back-compat
//     with older USD content (see the `eHAS_*` state bits raised in
//     Vehicle.cpp and read in VehicleGenerator.cpp). The parse-lib
//     mirror carries the same fields without the annotation so the
//     layout stays in lockstep until both can be dropped together.
struct WheelAttachmentDesc : public PhysxObjectDesc
{
    enum State
    {
        eMANAGE_TRANSFORMS = (1 << 0),
        // if the wheel attachment is a UsdGeomXformable, the code will set the corresponding
        // transforms based on the wheel simulation. The root transform to modify is defined by the
        // prim pointed to in the member "path"

        eHAS_SHAPE = (1 << 1),
        eHAS_WHEEL_COM_OFFSET = (1 << 2),
        eHAS_SUSP_FORCE_APP_POINT = (1 << 3),
        eHAS_TIRE_FORCE_APP_POINT = (1 << 4),
        eHAS_SUSPENSION_FRAME = (1 << 5)
    };

    WheelAttachmentDesc()
    {
        type = eVehicleWheelAttachment;
    }

    PXR_NS::SdfPath path;
    ObjectId id;

    WheelDesc* wheel;
    ObjectId wheelId;

    TireDesc* tire;
    ObjectId tireId;

    SuspensionDesc* suspension;
    ObjectId suspensionId;

    SuspensionComplianceDesc* suspensionCompliance;

    carb::Float3 suspensionTravelDirection;
    carb::Float3 suspensionForceAppPointOffset; // deprecated
    carb::Float3 wheelCenterOfMassOffset; // deprecated
    carb::Float3 tireForceAppPointOffset; // deprecated
    carb::Float3 suspensionFramePosition;
    carb::Float4 suspensionFrameOrientation;
    carb::Float3 wheelFramePosition;
    carb::Float4 wheelFrameOrientation;

    int index;

    bool driven; // deprecated

    ObjectId collisionGroupId;
    PXR_NS::SdfPath collisionGroupPath;

    PXR_NS::SdfPath shapePath;
    ObjectId shapeId;
    uint8_t state;
};

struct WheelControllerDesc : public PhysxObjectDesc
{
    WheelControllerDesc()
    {
        type = eVehicleWheelController;
    }

    PXR_NS::SdfPath path;
    ObjectId id;

    float driveTorque;
    float brakeTorque;
    float steerAngle;
};

struct EngineDesc : public PhysxObjectDesc
{
    EngineDesc()
    {
        type = eVehicleEngine;
    }

    static constexpr uint32_t maxNumberOfTorqueCurvePoints = 8;

    PXR_NS::SdfPath path;

    float moi;
    float peakTorque;
    float maxRotationSpeed;
    float idleRotationSpeed;
    carb::Float2 torqueCurve[maxNumberOfTorqueCurvePoints];
    unsigned int torqueCurvePointCount;
    float dampingRateFullThrottle;
    float dampingRateZeroThrottleClutchEngaged;
    float dampingRateZeroThrottleClutchDisengaged;
};

using GearsDesc = ::omni::physics::parse::GearsDesc;

using AutoGearBoxDesc = ::omni::physics::parse::AutoGearBoxDesc;

using ClutchDesc = ::omni::physics::parse::ClutchDesc;

using NonlinearCmdResponseDesc = ::omni::physics::parse::NonlinearCmdResponseDesc;

struct DriveDesc : public PhysxObjectDesc
{
};

struct DriveBasicDesc : public DriveDesc
{
    DriveBasicDesc()
    {
        type = eVehicleDriveBasic;
    }

    NonlinearCmdResponseDesc* nonlinearCmdResponse;

    PXR_NS::SdfPath path;
    ObjectId id;

    float peakTorque;
};

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

struct VehicleContextDesc : public PhysxObjectDesc
{
    enum AxisDir
    {
        ePosX,
        eNegX,
        ePosY,
        eNegY,
        ePosZ,
        eNegZ,
        eUndefined
    };

    VehicleContextDesc()
    {
        type = eVehicleContext;
    }

    void setDefaultValues()
    {
        vehicleUpdateMode = VehicleUpdateMode::eVelocityChange;
        upAxis.x = 0.0f;
        upAxis.y = 1.0f;
        upAxis.z = 0.0f;
        forwardAxis.x = 0.0f;
        forwardAxis.y = 0.0f;
        forwardAxis.z = 1.0f;
        verticalAxis = ePosY;
        longitudinalAxis = ePosZ;
    }

    PXR_NS::SdfPath scenePath;

    VehicleUpdateMode vehicleUpdateMode;

    carb::Float3 upAxis; // deprecated
    carb::Float3 forwardAxis; // deprecated
    AxisDir verticalAxis;
    AxisDir longitudinalAxis;
};

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

struct ParticleSystemDesc : public PhysxObjectDesc
{
    ParticleSystemDesc()
    {
        type = eParticleSystem;
    }
    bool enableParticleSystem;
    bool enableCCD;
    float restOffset;
    float contactOffset;
    float particleContactOffset;
    float solidRestOffset;
    float fluidRestOffset;
    float maxDepenetrationVelocity;
    float maxVelocity;
    float fluidBoundaryDensityScale;

    bool enableSmoothing;
    bool enableAnisotropy;
    bool enableIsosurface;
    int solverPositionIterations;
    carb::Float3 wind;
    int maxNeighborhood;
    float neighborhoodScale;
    int lockedAxis;

    ObjectId material;

    ObjectId collisionGroup;
    PXR_NS::SdfPathVector filteredCollisions;

    PXR_NS::SdfPath scenePath;
    PXR_NS::SdfPath systemPath;
};

struct ParticleDesc : public PhysxObjectDesc
{
    ParticleDesc()
    {
        type = eUndefined; // this should not be instantiated
    }
    int numParticles;

    bool enabled;
    bool selfCollision; // each set/cloth can set selfCollision flag independently
    int particleGroup; // each set/cloth can set particleGroup flag independently

    std::vector<carb::Float3> points; // Position for each particle
    std::vector<carb::Float3> velocities; // Velocity for each particle

    PXR_NS::SdfPath primPath;
    PXR_NS::SdfPath particleSystemPath;
    PXR_NS::SdfPath scenePath;

    // mass properties.
    float mass; // from massAPI, and overrides all
    float density; // from massAPI, and overrides any material densities
};

// rigid or fluid particle set defined via a UsdGeomPointInstancer or UsdGeomPoints
struct ParticleSetDesc : public ParticleDesc
{
    ParticleSetDesc()
    {
        type = eParticleSet;
        mass = -1.0f; // default invalid mass value that triggers MassAPI density or material or default density use
        density = -1.0f; // default invalid density that triggers material or default density use
    }

    bool fluid; // Are the particles in this prototype simulated as fluid or not
    float solidRestOffset;
    float fluidRestOffset;

    bool enableDiffuseParticles;
    float maxDiffuseParticleMultiplier;
    float diffuseParticlesThreshold;
    float diffuseParticlesLifetime;
    float diffuseParticlesAirDrag;
    float diffuseParticlesBubbleDrag;
    float diffuseParticlesBuoyancy;
    float diffuseParticlesKineticEnergyWeight;
    float diffuseParticlesPressureWeight;
    float diffuseParticlesDivergenceWeight;
    float diffuseParticlesCollisionDecay;

    std::vector<carb::Float3> simulationPoints; // Optional simulation positions, if positions are used for smoothed
                                                // positions

    int maxParticles;
};

struct ParticleSamplingDesc
{
    // sampling API
    float samplingDistance;
    bool sampleVolume;
    PXR_NS::SdfPath particleSetPath;
    int maxSamples;

    // particles
    float pointWidth;
};

struct ParticleIsosurfaceDesc : public PhysxObjectDesc
{
    struct GridFilteringPass
    {
        enum Enum
        {
            eSmooth,
            eGrow,
            eReduce,
            eNone
        };
    };

    bool enableIsosurface;

    int maxIsosurfaceVertices;
    int maxIsosurfaceTriangles;
    int maxNumIsosurfaceSubgrids;
    float gridSpacing;

    float surfaceDistance;
    std::vector<GridFilteringPass::Enum> gridFilteringPasses;
    float gridSmoothingRadius;

    int numMeshSmoothingPasses;
    int numMeshNormalSmoothingPasses;

    PXR_NS::SdfPath systemPath;
};

struct ParticleSmoothingDesc : public PhysxObjectDesc
{
    bool enableSmoothing;
    float strength;
    PXR_NS::SdfPath systemPath;
};

struct ParticleAnisotropyDesc : public PhysxObjectDesc
{
    bool enableAnisotropy;
    float scale;
    float min;
    float max;
    PXR_NS::SdfPath systemPath;
};

struct PhysxDeformableAttachmentDesc : public PhysxObjectDesc
{
    PhysxDeformableAttachmentDesc() : enabled(true)
    {
    }

    bool enabled;
    PXR_NS::SdfPath src0;
    PXR_NS::SdfPath src1;
    float stiffness;
    float damping;
};

struct PhysxDeformableCollisionFilterDesc : public PhysxObjectDesc
{
    PhysxDeformableCollisionFilterDesc() : enabled(true)
    {
        type = eDeformableCollisionFilter;
    }

    bool enabled;
    PXR_NS::SdfPath src0;
    PXR_NS::SdfPath src1;
};

// The CCT parser lives in omni.physics.parse / ScannedStage::ccts; the
// engine integration in usdInterface/UsdInterface.cpp consumes the same
// descriptor type.
using CctDesc        = ::omni::physics::parse::CctDesc;
using CapsuleCctDesc = ::omni::physics::parse::CapsuleCctDesc;

} // namespace usdparser
} // namespace physx
} // namespace omni
