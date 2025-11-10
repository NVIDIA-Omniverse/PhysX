// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>
#include <carb/extras/Hash.h>

#include <omni/physx/PhysxCookingParams.h>
#include <omni/physx/MeshKey.h>
#include <omni/physx/ObjectId.h>

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
struct ErrorCode
{
    enum Enum
    {
        eError,
        eWarning,
        eInfo
    };
};

using ObjectIdPair = std::pair<ObjectId, ObjectId>;
using ObjectIdPairVector = std::vector<ObjectIdPair>;

enum ObjectType : uint32_t
{
    eUndefined = 0,

    eCategoryMask = 0xffffu << 16,

    eShape = 1 << 16,
    eSphereShape,
    eBoxShape,
    eCapsuleShape,
    eCylinderShape,
    eConeShape,
    eConvexMeshShape,
    eConvexMeshDecompositionShape,
    eBoundingSphereShape,
    eBoundingBoxShape,
    eTriangleMeshShape,
    ePlaneShape,
    eCustomShape,
    eSpherePointsShape,

    eBody = 2 << 16,
    eDynamicBody,
    eStaticBody,

    eJoint = 3 << 16,
    eJointFixed,
    eJointRevolute,
    eJointPrismatic,
    eJointSpherical,
    eJointDistance,
    eJointD6,
    eJointGear,
    eJointRackAndPinion,
    eJointCustom,

    eScene = 4 << 16,

    eMaterial = 5 << 16,

    eArticulationLink = 6 << 16,

    eArticulation = 7 << 16,
    eArticulationJoint,
    eArticulationRootJoint,

    eFilteredPair = 8 << 16,
    eCollisionGroup = 9 << 16,

    eCct = 10 << 16,
    eCapsuleCct,

    eVehicle = 11 << 16,
    eVehicleContext = 12 << 16,
    eVehicleTireFrictionTable = 13 << 16,
    eVehicleController = 14 << 16,
    eVehicleControllerStandard,
    eVehicleControllerTank,
    eVehicleEngine = 15 << 16,
    eVehicleSuspension = 16 << 16,
    eVehicleTire = 17 << 16,
    eVehicleWheel = 18 << 16,
    eVehicleWheelAttachment = 19 << 16,
    eVehicleWheelController = 20 << 16,

    eVehicleDrive = 21 << 16,
    eVehicleDriveBasic,
    eVehicleDriveStandard,

    // physx specific types
    eParticleSystem = 22 << 16,
    eParticleSet = 23 << 16,
    eParticleCloth = 24 << 16, // DEPRECATED, Will be replaced by new deformable implementation in future release.

    eSoftBodyMaterial = 25 << 16, // DEPRECATED, Will be replaced by new deformable implementation in future release.
    eFEMClothMaterial = 26 << 16, // DEPRECATED, Will be replaced by new deformable implementation in future release.
    eSoftBody = 27 << 16, // DEPRECATED, Will be replaced by new deformable implementation in future release.
    eFEMCloth = 28 << 16, // DEPRECATED, Will be replaced by new deformable implementation in future release.

    ePhysxAttachment = 29 << 16, // DEPRECATED, Will be replaced by new deformable implementation in future release.
    ePhysXAttachmentTargetWorld = 30 << 16, // DEPRECATED, Will be replaced by new deformable implementation in future
                                            // release.

    ePointInstancedBody = 31 << 16,
    eInfiniteVoxelMap = 32 << 16,

    ePBDMaterial = 33 << 16,

    ePhysxForce = 34 << 16,

    eTendons = 35 << 16,
    eTendonFixed,
    eTendonAxis,
    eTendonAxisUI,
    eTendonAttachment,
    eTendonAttachmentLeaf,
    eTendonAttachmentRoot,
    eTendonAttachmentUI,

    eMimicJointRotX = 38 << 16,
    eMimicJointRotY = 39 << 16,
    eMimicJointRotZ = 40 << 16,
    // note: these can not use a common category because the same prim can have
    //       multiple mimic joints

    eDeformableMaterial = 41 << 16,
    eSurfaceDeformableMaterial,

    eDeformableBody = 42 << 16,
    eVolumeDeformableBody,
    eSurfaceDeformableBody,

    eDeformableAttachment = 43 << 16,
    eAttachmentVtxVtx,
    eAttachmentVtxTri,
    eAttachmentVtxTet,
    eAttachmentVtxXform,
    eAttachmentTetXform,

    eDeformableCollisionFilter = 44 << 16,
    eXformActor = 45 << 16,
};

struct ObjectCategory
{
    ObjectCategory() : data(eUndefined)
    {
    }
    ObjectCategory(ObjectType type) : data(type & eCategoryMask)
    {
    }

    bool operator==(const ObjectType& category) const
    {
        return data == category;
    }

    bool operator==(const ObjectCategory& categories) const
    {
        return data == categories.data;
    }

    bool operator<(const ObjectCategory& categories) const
    {
        return data < categories.data;
    }

    uint32_t getData() const
    {
        return data;
    }

private:
    uint32_t data;
};

enum BodyType
{
    eDynamic,
    eStatic,
    eKinematic
};

enum CctShapeType
{
    eBox,
    eCapsule
};

enum Axis
{
    eX,
    eY,
    eZ
};

enum JointAxis
{
    eDistance,
    eTransX,
    eTransY,
    eTransZ,
    eRotX,
    eRotY,
    eRotZ
};

enum ArticulationJointType
{
    eStandardJoint,
    eMaximalJoint,
};

enum VehicleUpdateMode
{
    eVelocityChange,
    eAcceleration,
};

enum CollisionSystem
{
    ePCM,
    eSAT
};

enum SolverType
{
    ePGS,
    eTGS
};

enum BroadphaseType
{
    eMBP,
    eSAP,
    eGPU
};

enum SceneUpdateType
{
    Synchronous,
    Asynchronous,
    Disabled
};

enum CombineMode
{
    eAverage = 0,
    eMin,
    eMultiply,
    eMax
};

struct ObjectInstance
{
    pxr::SdfPath instancerPath;
    uint32_t index;
    pxr::SdfPath protoPath;
    bool isExclusive;
};

struct PhysxObjectDesc
{
    PhysxObjectDesc() : type(eUndefined)
    {
    }

    ObjectType type;
};

struct PhysxMaterialDesc : PhysxObjectDesc
{
    PhysxMaterialDesc()
        : staticFriction(0.0f),
          dynamicFriction(0.0f),
          restitution(0.0f),
          density(-1.0f),
          frictionCombineMode(CombineMode::eAverage),
          restitutionCombineMode(CombineMode::eAverage),
          dampingCombineMode(CombineMode::eAverage),
          materialPath(pxr::SdfPath()),
          compliantAccelerationSpring(false),
          compliantStiffness(0.f),
          compliantDamping(0.f)
    {
        type = eMaterial;
    }

    float staticFriction;
    float dynamicFriction;
    float restitution;
    float density;

    CombineMode frictionCombineMode;
    CombineMode restitutionCombineMode;
    CombineMode dampingCombineMode;

    pxr::SdfPath materialPath;

    // compliant contacts:
    bool compliantAccelerationSpring;
    float compliantStiffness;
    float compliantDamping;
};

struct PhysxDeformableMaterialDesc : PhysxObjectDesc
{
    PhysxDeformableMaterialDesc()
        : staticFriction(0.0f),
          dynamicFriction(0.0f),
          density(-1.0f),
          youngsModulus(0.0f),
          poissonsRatio(0.0f),
          elasticityDamping(0.0f),
          materialPath(pxr::SdfPath())
    {
        type = eDeformableMaterial;
    }

    float staticFriction;
    float dynamicFriction;
    float density;

    float youngsModulus;
    float poissonsRatio;

    float elasticityDamping;

    pxr::SdfPath materialPath;
};

struct PhysxSurfaceDeformableMaterialDesc : PhysxDeformableMaterialDesc
{
    PhysxSurfaceDeformableMaterialDesc()
        : surfaceThickness(0.0f),
          surfaceStretchStiffness(0.0f),
          surfaceShearStiffness(0.0f),
          surfaceBendStiffness(0.0f),
          bendDamping(0.0f)
    {
        type = eSurfaceDeformableMaterial;
    }

    float surfaceThickness;
    float surfaceStretchStiffness;
    float surfaceShearStiffness;
    float surfaceBendStiffness;
    float bendDamping;
};

struct PBDMaterialDesc : public PhysxObjectDesc
{
    PBDMaterialDesc()
    {
        type = ePBDMaterial;
    }
    float friction;
    float particleFrictionScale;
    float damping;
    float viscosity;
    float vorticityConfinement;
    float surfaceTension;
    float cohesion;
    float adhesion;
    float particleAdhesionScale;
    float adhesionOffsetScale;
    float lift; // DEPRECATED, particle cloth will be replaced by new deformable implementation in future release.
    float drag; // DEPRECATED, particle cloth Will be replaced by new deformable implementation in future release.
    float gravityScale;
    float cflCoefficient;
    float density;

    pxr::SdfPath materialPath;
};

// DEPRECATED, Will be replaced by new deformable implementation in future release.
struct FemMaterialDesc : public PhysxObjectDesc
{
    FemMaterialDesc()
    {
        type = eMaterial;

        density = 0.0f;
        youngs = 0.0f;
        poissons = 0.0f;
        dynamicFriction = 0.0f;
        bendDamping = 0.0f;
        elasticityDamping = 0.0f;
        bendStiffness = 0.0f;
        materialPath = pxr::SdfPath();
    }
    // common material parameters
    float density;
    float youngs;
    float poissons;
    float dynamicFriction;
    float bendDamping;
    float elasticityDamping;
    float bendStiffness;

    pxr::SdfPath materialPath;
};

// DEPRECATED, Will be replaced by new deformable implementation in future release.
struct FemSoftBodyMaterialDesc : public FemMaterialDesc
{
    FemSoftBodyMaterialDesc()
    {
        type = eSoftBodyMaterial;

        damping = 0.0f;
        dampingScale = 1.0f;
        deformThreshold = FLT_MAX;
        deformLowLimitRatio = 1.0f;
        deformHighLimitRatio = 1.0f;
    }

    // softbody material parameters
    float damping;
    float dampingScale;

    float deformThreshold;
    float deformLowLimitRatio;
    float deformHighLimitRatio;
};

// DEPRECATED, Will be replaced by new deformable implementation in future release.
struct FemClothMaterialDesc : public FemMaterialDesc
{
    FemClothMaterialDesc()
    {
        type = eFEMClothMaterial;
        thickness = 0.0f;
    }

    // FEM cloth material parameters
    float thickness;
};

struct PhysxSceneDesc : PhysxObjectDesc
{
    PhysxSceneDesc()
    {
        type = eScene;
    }

    carb::Float3 gravityDirection;
    float gravityMagnitude;

    uint32_t timeStepsPerSecond;

    float bounceThreshold;
    float frictionOffsetThreshold;
    float frictionCorrelationDistance;
    float maxBiasCoefficient;
    CollisionSystem collisionSystem;
    SolverType solverType;
    BroadphaseType broadphaseType;
    bool enableCCD;
    bool enableStabilization;
    bool enableGPUDynamics;
    bool enableEnhancedDeterminism;
    bool enableExternalForcesEveryIteration;
    bool invertedFiltering;
    bool reportKineKine;
    bool reportKineStatic;
    SceneUpdateType sceneUpdateType;
    bool supportSceneQueries;
    bool enableResidualReporting;
    bool reportResiduals;
    bool solveArticulationContactLast;

    bool enableQuasistatic;
    pxr::SdfPathSet quasistaticActors;

    uint64_t gpuTempBufferCapacity;
    uint32_t gpuMaxRigidContactCount;
    uint32_t gpuMaxRigidPatchCount;
    uint32_t gpuHeapCapacity;
    uint32_t gpuFoundLostPairsCapacity;
    uint32_t gpuFoundLostAggregatePairsCapacity;
    uint32_t gpuTotalAggregatePairsCapacity;
    uint32_t gpuMaxDeformableVolumeContacts;
    uint32_t gpuMaxDeformableSurfaceContacts;
    uint32_t gpuMaxParticleContacts;
    uint32_t gpuCollisionStackSize;
    uint32_t gpuMaxNumPartitions;

    uint32_t minPosIterationCount;
    uint32_t maxPosIterationCount;
    uint32_t minVelIterationCount;
    uint32_t maxVelIterationCount;

    int32_t envIdInBoundsBitCount;

    PhysxMaterialDesc defaultMaterialDesc;
    PBDMaterialDesc defaultPBDMaterialDesc;
    FemSoftBodyMaterialDesc defaultSoftBodyMaterialDesc; // DEPRECATED, will be replaced by new deformable
                                                         // implementation in future release.
    FemClothMaterialDesc defaultFemClothMaterialDesc; // DEPRECATED, will be replaced by new deformable implementation
                                                      // in future release.

    PhysxDeformableMaterialDesc defaultDeformableMaterialDesc;
    PhysxSurfaceDeformableMaterialDesc defaultSurfaceDeformableMaterialDesc;
};

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

struct PhysxShapeDesc : PhysxObjectDesc
{
    PhysxShapeDesc()
        : localPos({ 0.0f, 0.0f, 0.0f }),
          localRot({ 0.0f, 0.0f, 0.0f, 1.0f }),
          localScale({ 1.0f, 1.0f, 1.0f }),
          collisionGroup(kInvalidObjectId),
          collisionEnabled(true)
    {
    }

    carb::Float3 localPos;
    carb::Float4 localRot;
    carb::Float3 localScale;
    std::vector<ObjectId> materials;
    pxr::SdfPath rigidBody;
    ObjectId collisionGroup;
    bool collisionEnabled;

    float contactOffset;
    float restOffset;
    float torsionalPatchRadius;
    float minTorsionalPatchRadius;
    bool isTrigger;
    bool isTriggerUsdOutput;

    std::vector<ObjectId> sceneIds;
};

struct CustomPhysxShapeDesc : PhysxShapeDesc
{
    CustomPhysxShapeDesc()
    {
        type = eCustomShape;
    }

    pxr::TfToken customGeometryToken;
};

struct SpherePhysxShapeDesc : PhysxShapeDesc
{
    SpherePhysxShapeDesc(float inRadius = 0.0f) : PhysxShapeDesc(), radius(inRadius)
    {
        type = eSphereShape;
    }

    float radius;
};

struct MergeMeshDesc
{
    pxr::VtArray<pxr::GfVec3f> points;
    pxr::VtArray<int> indices;
    pxr::VtArray<int> faces;
    pxr::VtArray<int> holes;
};

struct MergeMeshPhysxShapeDesc : PhysxShapeDesc
{
    MergeMeshPhysxShapeDesc() : PhysxShapeDesc(), mergedMesh(nullptr)
    {
    }

    MergeMeshDesc* mergedMesh;
};

struct BoundingSpherePhysxShapeDesc : MergeMeshPhysxShapeDesc
{
    BoundingSpherePhysxShapeDesc() : MergeMeshPhysxShapeDesc()
    {
        type = eBoundingSphereShape;
    }

    carb::Float3 positionOffset;
    float radius;
};

struct BoundingBoxPhysxShapeDesc : MergeMeshPhysxShapeDesc
{
    BoundingBoxPhysxShapeDesc() : MergeMeshPhysxShapeDesc()
    {
        type = eBoundingBoxShape;
    }

    carb::Float3 positionOffset;
    carb::Float4 rotationOffset;
    carb::Float3 halfExtents;
};

struct CapsulePhysxShapeDesc : PhysxShapeDesc
{
    CapsulePhysxShapeDesc(float inRadius = 0.0f, float half_height = 0.0f, Axis cap_axis = eX)
        : PhysxShapeDesc(), radius(inRadius), halfHeight(half_height), axis(cap_axis)
    {
        type = eCapsuleShape;
    }

    float radius;
    float halfHeight;
    Axis axis;
};

struct CylinderPhysxShapeDesc : PhysxShapeDesc
{
    CylinderPhysxShapeDesc(float inRadius = 0.0f, float half_height = 0.0f, Axis cap_axis = eX, float inMargin = 0.0f)
        : PhysxShapeDesc(), radius(inRadius), halfHeight(half_height), axis(cap_axis), margin(inMargin)
    {
        type = eCylinderShape;
    }

    float radius;
    float halfHeight;
    Axis axis;
    float margin;
};

struct ConePhysxShapeDesc : PhysxShapeDesc
{
    ConePhysxShapeDesc(float inRadius = 0.0f, float half_height = 0.0f, Axis cap_axis = eX, float inMargin = 0.0f)
        : PhysxShapeDesc(), radius(inRadius), halfHeight(half_height), axis(cap_axis), margin(inMargin)
    {
        type = eConeShape;
    }

    float radius;
    float halfHeight;
    Axis axis;
    float margin;
};

struct BoxPhysxShapeDesc : PhysxShapeDesc
{
    BoxPhysxShapeDesc(float x = 0.0f, float y = 0.0f, float z = 0.0f) : PhysxShapeDesc(), halfExtents({ x, y, z })
    {
        type = eBoxShape;
    }
    BoxPhysxShapeDesc(const carb::Float3& inHalfExtents) : PhysxShapeDesc(), halfExtents(inHalfExtents)
    {
        type = eBoxShape;
    }

    carb::Float3 halfExtents;
};

struct InfiniteVoxelMapDesc : PhysxObjectDesc
{
    InfiniteVoxelMapDesc(const pxr::SdfPath path) : PhysxObjectDesc(), rootPrim{ path }
    {
        type = eInfiniteVoxelMap;
    };

    const pxr::SdfPath rootPrim;
};

struct ConvexMeshPhysxShapeDesc : MergeMeshPhysxShapeDesc
{
    ConvexMeshPhysxShapeDesc(size_t nbVerts = 0, const carb::Float3* verts = nullptr, MeshKey inCrc = MeshKey())
        : MergeMeshPhysxShapeDesc(), crc(inCrc)
    {
        type = eConvexMeshShape;
    }

    pxr::SdfPath meshPath;
    carb::Float3 meshScale;
    MeshKey crc;
    MeshKey meshKey;
    uint32_t numVerts{ 0 };

    ConvexMeshCookingParams convexCookingParams;
};


struct TriangleMeshPhysxShapeDesc : MergeMeshPhysxShapeDesc
{
    TriangleMeshPhysxShapeDesc(MeshKey inCrc = MeshKey())
        : MergeMeshPhysxShapeDesc(), meshScale({ 1.f, 1.f, 1.f }), crc(inCrc), doubleSided(false)
    {
        type = eTriangleMeshShape;
    }

    pxr::SdfPath meshPath;
    carb::Float3 meshScale;
    MeshKey crc;
    MeshKey meshKey; // this is the hash of the source USD mesh data
    bool doubleSided;

    SdfMeshCookingParams sdfMeshCookingParams;
    TriangleMeshCookingParams triangleMeshCookingParams;
};

struct ConvexMeshDecompositionPhysxShapeDesc : TriangleMeshPhysxShapeDesc
{
    ConvexMeshDecompositionPhysxShapeDesc(MeshKey inCrc = MeshKey()) : TriangleMeshPhysxShapeDesc(inCrc)
    {
        type = eConvexMeshDecompositionShape;
    }

    // convex decomposition parameters
    ConvexDecompositionCookingParams convexDecompositionCookingParams;
};

struct SpherePhysxPoint
{
    carb::Float3 position;
    float radius;
};

struct SpherePointsPhysxShapeDesc : TriangleMeshPhysxShapeDesc
{
    SpherePointsPhysxShapeDesc(void) : TriangleMeshPhysxShapeDesc()
    {
        type = eSpherePointsShape;
    }

    ~SpherePointsPhysxShapeDesc(void)
    {
    }

    SphereFillCookingParams sphereFillCookingParams;
    std::vector<SpherePhysxPoint> spheres;
};


struct PlanePhysxShapeDesc : PhysxShapeDesc
{
    PlanePhysxShapeDesc() : PhysxShapeDesc(), axis(eX)
    {
        type = ePlaneShape;
    }

    Axis axis;
};

struct PhysxRigidBodyDesc : PhysxObjectDesc
{
    PhysxRigidBodyDesc()
        : position({ 0.0f, 0.0f, 0.0f }), rotation({ 0.0f, 0.0f, 0.0f, 1.0f }), scale({ 1.0f, 1.0f, 1.0f })
    {
    }

    std::vector<ObjectId> shapes;
    std::vector<ObjectId> collisionBlocks;
    carb::Float3 position;
    carb::Float4 rotation;
    carb::Float3 scale;

    std::vector<ObjectId> sceneIds;
};

struct PointInstancedBodyDesc : PhysxObjectDesc
{
    PointInstancedBodyDesc()
    {
        type = ePointInstancedBody;
    }
};

struct StaticPhysxRigidBodyDesc : PhysxRigidBodyDesc
{
    StaticPhysxRigidBodyDesc() : PhysxRigidBodyDesc()
    {
        type = eStaticBody;
    }
};

struct DynamicPhysxRigidBodyDesc : PhysxRigidBodyDesc
{
    DynamicPhysxRigidBodyDesc()
        : PhysxRigidBodyDesc(),
          kinematicBody(false),
          hasTimeSampledXform(false),
          linearVelocity({ 0.0f, 0.0f, 0.0f }),
          angularVelocity({ 0.0f, 0.0f, 0.0f }),
          startsAsleep(false)
    {
        type = eDynamicBody;
    }

    bool kinematicBody;
    bool hasTimeSampledXform;
    carb::Float3 linearVelocity;
    carb::Float3 angularVelocity;
    bool startsAsleep;

    float linearDamping;
    float angularDamping;
    float maxLinearVelocity;
    float maxAngularVelocity;
    float sleepThreshold;
    float stabilizationThreshold;
    float maxDepenetrationVelocity;
    float maxContactImpulse;
    float contactSlopCoefficient;
    float cfmScale;
    int solverPositionIterationCount;
    int solverVelocityIterationCount;

    bool enableCCD;
    bool enableSpeculativeCCD;
    bool disableGravity;
    bool retainAccelerations;
    bool enableGyroscopicForces;
    bool localSpaceVelocities;
    bool solveContacts;

    int lockedPosAxis;
    int lockedRotAxis;

    bool surfaceVelocityEnabled;
    bool surfaceVelocityLocalSpace;
    carb::Float3 surfaceLinearVelocity;
    carb::Float3 surfaceAngularVelocity;

    bool splinesSurfaceVelocityEnabled;
    float splinesSurfaceVelocityMagnitude;
    pxr::SdfPath splinesCurvePrimPath;
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

struct PhysxJointLimit
{
    PhysxJointLimit() : enabled(false), angle0((float)M_PI_2), angle1((float)-M_PI_2)
    {
    }

    bool enabled;
    union
    {
        float angle0;
        float lower;
        float minDist;
    };
    union
    {
        float angle1;
        float upper;
        float maxDist;
    };

    float restitution;
    float bounceThreshold;
    float stiffness;
    float damping;
};

struct PhysxJointDrive
{
    PhysxJointDrive()
        : enabled(false),
          targetPosition(0.0f),
          targetVelocity(0.0f),
          forceLimit(FLT_MAX),
          stiffness(0.0f),
          damping(0.0f),
          acceleration(false),
          isEnvelopeUsed(false),
          maxActuatorVelocity(FLT_MAX),
          velocityDependentResistance(0),
          speedEffortGradient(0)
    {
    }

    bool enabled;
    float targetPosition;
    float targetVelocity;
    float forceLimit;
    float stiffness;
    float damping;
    bool acceleration;
    bool isEnvelopeUsed;
    float maxActuatorVelocity;
    float velocityDependentResistance;
    float speedEffortGradient;
};

struct PhysxJointAxisProperties
{
    PhysxJointAxisProperties()
    {}
    float maxJointVelocity;
    float armature;
    float staticFrictionEffort;
    float dynamicFrictionEffort;
    float viscousFrictionCoefficient;
};

struct PhysicsJointState
{
    PhysicsJointState() : enabled(false), position(0.0f), velocity(0.0f)
    {
    }
    bool enabled;
    float position;
    float velocity;
};


struct PhysxArticulationDesc : PhysxObjectDesc
{
    PhysxArticulationDesc() : fixBase(false), sceneId(kInvalidObjectId)
    {
        type = eArticulation;
    }

    pxr::SdfPath rootPrim;
    pxr::SdfPath staticRootBodyPrim;
    bool fixBase;
    pxr::SdfPath fixBasePath;
    int solverPositionIterationCount;
    int solverVelocityIterationCount;
    float sleepThreshold;
    float stabilizationThreshold;
    bool selfCollision;
    bool reportResiduals;
    std::unordered_set<pxr::SdfPath, pxr::SdfPath::Hash> articulatedJoints;
    std::unordered_set<pxr::SdfPath, pxr::SdfPath::Hash> articulatedBodies;
    ObjectId sceneId;
};

struct PhysxTendonAxisDesc : PhysxObjectDesc
{
    PhysxTendonAxisDesc()
        : gearings(1u), forceCoefficients(1u), axes(1u), parentAxisId(kInvalidObjectId), wasVisited(false)
    {
        type = eTendonAxis;
        forceCoefficients[0] = 1.0f;
    }

    pxr::TfToken instanceToken;
    pxr::SdfPath jointPath;

    pxr::SdfPath link0;
    pxr::SdfPath link1;

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

    pxr::TfToken instanceToken;
    pxr::SdfPath jointPath;

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

struct PhysxTendonAxisHierarchyDesc : PhysxObjectDesc
{
    PhysxTendonAxisHierarchyDesc() : axes(0), children(0)
    {
        type = eTendonAxisUI;
    }

    pxr::SdfPath jointPath;
    pxr::TfToken instanceToken;

    pxr::SdfPath link0;
    pxr::SdfPath link1;

    std::vector<JointAxis> axes;
    std::vector<PhysxTendonAxisHierarchyDesc*> children;
};

struct PhysxTendonAttachmentDesc : PhysxObjectDesc
{
    PhysxTendonAttachmentDesc() : gearing(1.0f), localPos({ 0.f, 0.f, 0.f }), parentId(kInvalidObjectId)
    {
        type = eTendonAttachment;
    }

    float gearing;
    carb::Float3 localPos;
    pxr::SdfPath parentPath;
    pxr::SdfPath linkPath;
    pxr::TfToken parentToken;
    pxr::TfToken instanceToken;
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

struct PhysxTendonAttachmentHierarchyDesc : PhysxObjectDesc
{
    PhysxTendonAttachmentHierarchyDesc() : localPos({ 0.f, 0.f, 0.f }), children(0)
    {
        type = eTendonAttachmentUI;
    }

    carb::Float3 localPos;
    pxr::SdfPath linkPath;
    std::vector<PhysxTendonAttachmentHierarchyDesc*> children;
};

struct MimicJointDesc : PhysxObjectDesc
{
    MimicJointDesc()
    {
        type = eUndefined;
        // the type depends on the axis it should operate on and that will only be known at parsing time.
    }

    // the joint that should mimic another joint
    pxr::SdfPath mimicJointPath;
    ObjectId mimicJointId;
    int mimicJointAxis; // eDEFAULT_AXIS for revolute or prismatic, else the enum JointAxis

    // the joint that should get mimicked
    pxr::SdfPath referenceJointPath;
    ObjectId referenceJointId;
    int referenceJointAxis; // see mimicJointAxis

    float gearing;
    float offset;
    float naturalFrequency;
    float dampingRatio;

    static const int eDEFAULT_AXIS = -1;
};

typedef std::vector<std::pair<JointAxis, PhysxJointLimit>> JointLimits;
typedef std::vector<std::pair<JointAxis, PhysxJointDrive>> JointDrives;
typedef std::vector<std::pair<JointAxis, PhysicsJointState>> JointStates;
typedef std::vector<std::pair<JointAxis, PhysxJointAxisProperties>> JointAxisProperties;

struct PhysxJointDesc : public PhysxObjectDesc
{
    PhysxJointDesc()
        : localPose0Position({ 0.0f, 0.0f, 0.0f }),
          localPose0Orientation({ 1.0f, 0.0f, 0.0f, 0.0f }),
          localPose1Position({ 0.0f, 0.0f, 0.0f }),
          localPose1Orientation({ 1.0f, 0.0f, 0.0f, 0.0f }),
          jointEnabled(true),
          breakForce(FLT_MAX), // USD default is none, which is not a float...
          breakTorque(FLT_MAX),
          jointFriction(0.0f)
    {
    }

    pxr::SdfPath jointPrimPath;
    pxr::SdfPath rel0;
    pxr::SdfPath rel1;
    pxr::SdfPath body0;
    pxr::SdfPath body1;
    carb::Float3 localPose0Position;
    carb::Float4 localPose0Orientation;
    carb::Float3 localPose1Position;
    carb::Float4 localPose1Orientation;
    bool jointEnabled;
    float breakForce;
    float breakTorque;
    float jointFriction;
    bool enableCollision;
    bool excludedFromArticulation;
    bool validBodyTransformations;

    bool enableResidualReporting;
};

struct FixedPhysxJointDesc : public PhysxJointDesc
{
    FixedPhysxJointDesc()
    {
        type = eJointFixed;
    }
};

struct D6PhysxJointDesc : public PhysxJointDesc
{
    D6PhysxJointDesc()
    {
        type = eJointD6;
    }

    JointLimits jointLimits;
    JointDrives jointDrives;
    JointStates jointStates;
    JointAxisProperties jointProperties;
};

struct PrismaticPhysxJointDesc : public PhysxJointDesc
{
    PrismaticPhysxJointDesc() : axis(eX)
    {
        type = eJointPrismatic;
    }

    Axis axis;
    PhysxJointLimit limit;
    PhysxJointDrive drive;
    PhysicsJointState state;
    PhysxJointAxisProperties properties;
};

struct SphericalPhysxJointDesc : public PhysxJointDesc
{
    SphericalPhysxJointDesc() : axis(eX)
    {
        type = eJointSpherical;
    }

    Axis axis;
    PhysxJointLimit limit;
    PhysicsJointState state;
    JointAxisProperties jointProperties;
};

struct RevolutePhysxJointDesc : public PhysxJointDesc
{
    RevolutePhysxJointDesc() : axis(eX)
    {
        type = eJointRevolute;
    }

    Axis axis;
    PhysxJointLimit limit;
    PhysxJointDrive drive;
    PhysicsJointState state;
    PhysxJointAxisProperties properties;
};

struct DistancePhysxJointDesc : public PhysxJointDesc
{
    DistancePhysxJointDesc() : minEnabled(false), maxEnabled(false), springEnabled(false), damping(0.0f), stiffness(0.0)
    {
        type = eJointDistance;
    }

    bool minEnabled;
    bool maxEnabled;
    PhysxJointLimit limit;
    PhysicsJointState state;

    bool springEnabled;
    float damping;
    float stiffness;
};

struct GearPhysxJointDesc : public PhysxJointDesc
{
    GearPhysxJointDesc() : gearRatio(0.0f)
    {
        type = eJointGear;
    }

    pxr::SdfPath hingePrimPath0;
    pxr::SdfPath hingePrimPath1;
    float gearRatio;
};

struct RackPhysxJointDesc : public PhysxJointDesc
{
    RackPhysxJointDesc() : ratio(0.0f)
    {
        type = eJointRackAndPinion;
    }

    pxr::SdfPath hingePrimPath;
    pxr::SdfPath prismaticPrimPath;
    float ratio;
};

struct CustomPhysxJointDesc : public PhysxJointDesc
{
    CustomPhysxJointDesc()
    {
        type = eJointCustom;
    }

    pxr::TfToken customJointToken;
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
    uint32_t solverPositionIterationCount; // TOTO switch schema to int for consistency with rigid bodies.
    ObjectId sceneId;

    // mesh generation parameter
    bool isAutoMeshSimplificationEnabled;
    bool isAutoRemeshingEnabled;
    bool hasAutoForceConforming;
    uint32_t autoRemeshingResolution;
    uint32_t autoTriangleTargetCount;

    pxr::GfMatrix4d transform;
    float mass;

    pxr::SdfPath simMeshPath;
    ObjectId simMeshMaterial;
    pxr::TfToken simMeshBindPoseToken;

    pxr::SdfPath collisionMeshPath;
    pxr::TfToken collisionMeshBindPoseToken;
    ObjectId collisionGroup;

    pxr::SdfPathVector skinGeomPaths;
    pxr::TfTokenVector skinGeomBindPoseTokens; // same size as skinGeomPaths

    pxr::SdfPath cookingSrcMeshPath;
    pxr::TfToken cookingSrcMeshBindPoseToken;

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
    pxr::TfToken restBendAnglesDefault;

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

    pxr::SdfPath path;
    std::vector<pxr::SdfPath> materialPaths;
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

    pxr::SdfPath path;

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

    pxr::SdfPath path;

    float latStiffX; // deprecated
    float latStiffY; // deprecated
    carb::Float2 lateralStiffnessGraph;
    float longitudinalStiffnessPerUnitGravity; // deprecated
    float longitudinalStiffness;
    float camberStiffnessPerUnitGravity; // deprecated
    float camberStiffness;
    carb::Float2 frictionVsSlipGraph[3];
    ObjectId frictionTableId;
    pxr::SdfPath frictionTablePath;
    float restLoad;
};

struct SuspensionDesc : public PhysxObjectDesc
{
    SuspensionDesc()
    {
        type = eVehicleSuspension;
    }

    pxr::SdfPath path;

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

struct SuspensionComplianceDesc
{
    std::vector<carb::Float2> wheelToeAngleList;
    std::vector<carb::Float2> wheelCamberAngleList;
    std::vector<carb::Float4> suspensionForceAppPointList;
    std::vector<carb::Float4> tireForceAppPointList;
};

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

    pxr::SdfPath path;
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
    pxr::SdfPath collisionGroupPath;

    pxr::SdfPath shapePath;
    ObjectId shapeId;
    uint8_t state;
};

struct WheelControllerDesc : public PhysxObjectDesc
{
    WheelControllerDesc()
    {
        type = eVehicleWheelController;
    }

    pxr::SdfPath path;
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

    pxr::SdfPath path;

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

struct GearsDesc
{
    std::vector<float> ratios;
    float ratioScale;
    float switchTime;

    static constexpr uint32_t maxNumberOfGears = 32; // including reverse and neutral
};

struct AutoGearBoxDesc
{
    std::vector<float> upRatios;
    std::vector<float> downRatios;
    float latency;
};

struct ClutchDesc
{
    float strength;
};

struct NonlinearCmdResponseDesc
{
    std::vector<float> commandValues;
    std::vector<int> speedResponsesPerCommandValue;
    std::vector<carb::Float2> speedResponses;

    static constexpr uint32_t maxNumberOfCommandValues = 8;
    static constexpr uint32_t maxNumberOfSpeedResponses = 64;
};

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

    pxr::SdfPath path;
    ObjectId id;

    float peakTorque;
};

struct DriveStandardDesc : public DriveDesc
{
    DriveStandardDesc()
    {
        type = eVehicleDriveStandard;
    }

    // note: object IDs and descriptors are provided both even though one or the other would be enough.
    //       Initially, only descriptors were used but to better fit into the USD prim update
    //       listener logic, the objects are now actually passed into create calls and thus IDs
    //       are available.

    EngineDesc* engine;
    ObjectId engineId;

    const GearsDesc* gears;
    const AutoGearBoxDesc* autoGearBox;
    const ClutchDesc* clutch;
};

struct DifferentialDesc
{
    enum Type
    {
        eMultiWheel,
        eTank
    };

    Type type;
};

struct MultiWheelDifferentialDesc : public DifferentialDesc
{
    MultiWheelDifferentialDesc()
    {
        type = eMultiWheel;
    }

    std::vector<int> wheels;
    std::vector<float> torqueRatios;
    std::vector<float> averageWheelSpeedRatios;
};

struct TankDifferentialDesc : public MultiWheelDifferentialDesc
{
    TankDifferentialDesc()
    {
        type = eTank;
    }

    std::vector<int> numberOfWheelsPerTrack;
    std::vector<int> thrustIndexPerTrack;
    std::vector<int> trackToWheelIndices;
    std::vector<int> wheelIndicesInTrackOrder;
};

struct BrakesDesc
{
    NonlinearCmdResponseDesc* nonlinearCmdResponse;
    std::vector<int> wheels;
    std::vector<float> torqueMultipliers;
    float maxBrakeTorque;
    uint8_t brakesIndex;
};

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

    pxr::SdfPath scenePath;

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
    pxr::SdfPathVector filteredCollisions;

    pxr::SdfPath scenePath;
    pxr::SdfPath systemPath;
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

    pxr::SdfPath primPath;
    pxr::SdfPath particleSystemPath; // particle system's path
    pxr::SdfPath scenePath; // scene path

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

    // Fabric
    bool useFabric;
};

// DEPRECATED, will be replaced with new deformable implementation in future release.
struct ParticleClothDesc : public ParticleDesc
{
    ParticleClothDesc()
    {
        type = eParticleCloth;
        mass = -1.0f; // default invalid mass value that triggers MassAPI density or material or default density use
        density = -1.0f; // default invalid density that triggers material or default density use
    }

    bool selfCollisionFilter;
    float restOffset;

    std::vector<carb::Float3> restPoints;
    std::vector<carb::Float3> scaledRestPoints;

    std::vector<carb::Int2> springIndices;
    std::vector<float> springStiffnesses;
    std::vector<float> springDampings;
    std::vector<float> springRestLengths;

    std::vector<uint32_t> triangleIndices; // triangle mesh indices

    // Welded mesh
    bool isWelded; // true if welding is enabled and welded tri indices is not empty. Welded info is pre-processed in
                   // the parser
    std::vector<uint32_t> verticesRemapToWeld; // remap table from original vertex indices to welded vertex indices
    std::vector<uint32_t> verticesRemapToOrig; // remap table from welded vertex indices to original vertex indices (1:1
                                               // mapping, welded indice always refer to its closest original points)

    float pressure; // If set to a value larger than zero, the object will behave like an inflatable
    float inflatableVolume;

    bool isWatertight;

    pxr::SdfPath instancePath;
};

struct ParticleSamplingDesc
{
    // sampling API
    float samplingDistance;
    bool sampleVolume;
    pxr::SdfPath particleSetPath;
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

    pxr::SdfPath systemPath;
};

struct ParticleSmoothingDesc : public PhysxObjectDesc
{
    bool enableSmoothing;
    float strength;
    pxr::SdfPath systemPath;
};

struct ParticleAnisotropyDesc : public PhysxObjectDesc
{
    bool enableAnisotropy;
    float scale;
    float min;
    float max;
    pxr::SdfPath systemPath;
};

// DEPRECATED, will be replaced with new deformable implementation in future release.
struct DeformableDesc : public PhysxObjectDesc
{
    DeformableDesc()
    {
        mass = -1.0f; // default invalid mass value that triggers MassAPI density or material or default density use
        density = -1.0f; // default invalid density that triggers material or default density use
        deformableEnabled = true;
    }

    bool deformableEnabled;
    uint32_t solverPositionIterations;

    float velocityDamping;
    float sleepDamping;
    float sleepThreshold;
    float settlingThreshold;

    bool selfCollision;
    float selfCollisionFilterDistance;

    bool enableCCD;

    float collisionRestOffset;
    float collisionContactOffset;
    ObjectId collisionGroup;
    pxr::SdfPathVector filteredCollisions;

    std::vector<carb::Float3> points;
    std::vector<carb::Float3> restPoints;
    std::vector<carb::Float3> simulationVelocities;
    std::vector<uint32_t> simulationIndices;

    // material and mass properties.
    std::vector<ObjectId> materials; // from a material binding / multi-material setup
    float mass; // from massAPI, and overrides all
    float density; // from massAPI, and overrides any material densities

    pxr::SdfPath scenePath;
};

// DEPRECATED, will be replaced with new deformable implementation in future release.
struct SoftBodyDesc : public DeformableDesc
{
    SoftBodyDesc() : DeformableDesc()
    {
        type = eSoftBody;
        simulationHexahedralResolution = 0;
        kinematicBody = false;
        numberOfTetsPerHex = 6;
    }

    bool kinematicBody;
    pxr::SdfPath softBodyPath;

    // collision mesh related parameter
    std::vector<uint32_t> collisionIndices;
    std::vector<carb::Float3> collisionRestPoints;
    std::vector<carb::Float3> collisionPoints;

    // simulation mesh related parameter (some are shared in DeformableDesc)
    std::vector<carb::Float3> simulationRestPoints;
    std::vector<carb::Float3> simulationPoints;
    uint32_t simulationHexahedralResolution;
    uint32_t numberOfTetsPerHex;

    std::vector<uint32_t> collisionVertexToSimulationTetIndices;
    std::vector<carb::Uint3> collisionVertexToSkinTriVertexIndices;
    std::vector<carb::Float3> collisionVertexToSkinTriBarycentrics;

    // inv mass scale
    std::vector<float> invMassScale;
};

// DEPRECATED, Will be replaced by new deformable implementation in future release.
struct FEMClothDesc : public DeformableDesc
{
    FEMClothDesc() : DeformableDesc()
    {
        // PhysxObjectDesc
        type = eFEMCloth;

        // FEMClothDesc
        flatteningEnabled = false;
        collisionPairUpdateFrequency = 1;
        collisionIterationMultiplier = 1;
        maxVelocity = 0.0f;
        maxDepenetrationVelocity = 0.0f;

        // currently unused parameters
        lift = 0.0f;
        drag = 0.0f;
        wind = { 0.0f, 0.0f, 0.0f };
        useAnisotropicCloth = false;
    }

    pxr::SdfPath femClothPath;

    // bending
    bool flatteningEnabled;

    // collision substepping
    uint32_t collisionPairUpdateFrequency;
    uint32_t collisionIterationMultiplier;

    // velocity clamping
    float maxVelocity;
    float maxDepenetrationVelocity;

    // currently unused parameters
    float lift;
    float drag;
    carb::Float3 wind;
    bool useAnisotropicCloth;
};

// DEPRECATED, will be replaced with new deformable implementation in future release.
enum PhysxAttachmentFilterType
{
    eNONE = 0,
    eVERTICES = 1,
    eGEOMETRY = 2
};

// DEPRECATED, will be replaced with new deformable implementation in future release.
struct PhysxAttachmentDesc : public PhysxObjectDesc
{
    PhysxAttachmentDesc()
    {
        type = ePhysxAttachment;
        actor[0].filterType = PhysxAttachmentFilterType::eNONE;
        actor[1].filterType = PhysxAttachmentFilterType::eNONE;

        actor[0].id = 0;
        actor[1].id = 1;

        attachmentEnabled = true;
    }

    struct AttachmentActorDesc
    {
        pxr::SdfPath path;
        std::vector<carb::Float3> points;
        PhysxAttachmentFilterType filterType;
        std::vector<uint32_t> filterIndices;
        ObjectId objId;
        ObjectType objType;
        uint32_t id;
    };

    AttachmentActorDesc actor[2];

    std::vector<carb::Float3> distanceAxes;

    bool attachmentEnabled;
};

struct PhysxDeformableAttachmentDesc : public PhysxObjectDesc
{
    PhysxDeformableAttachmentDesc() : enabled(true)
    {
    }

    bool enabled;
    pxr::SdfPath src0;
    pxr::SdfPath src1;
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
    pxr::SdfPath src0;
    pxr::SdfPath src1;
};

struct CctDesc : public PhysxObjectDesc
{
    CctDesc() : sceneId(kInvalidObjectId)
    {
    }

    float slopeLimit;

    carb::Float3 pos;
    carb::Float3 scale;

    ObjectId sceneId;
};

struct CapsuleCctDesc : public CctDesc
{
    CapsuleCctDesc(float inRadius = 0.1f, float half_height = 0.1f) : CctDesc(), height(half_height), radius(inRadius)
    {
        type = eCapsuleCct;
    };

    float height;
    float radius;
};

} // namespace usdparser
} // namespace physx
} // namespace omni
