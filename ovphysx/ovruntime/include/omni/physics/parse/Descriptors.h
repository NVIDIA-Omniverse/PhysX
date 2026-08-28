// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-CORE-001
 * @covers AC-1
 *
 * @implements REQ-PARSE-CORE-004
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-PARSE-SHAPE-001
 * @covers AC-1
 *
 * @implements REQ-PARSE-SCAN-001
 * @covers AC-7 AC-12
 */

#pragma once

#include "Handles.h"
#include "Math.h"

#include <cstdint>
#include <unordered_set>
#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>
#include <cfloat>

#include <carb/Types.h>

#include <omni/physx/MeshKey.h>
#include <omni/physx/ObjectId.h>
#include <omni/physx/PhysxCookingParams.h>

namespace omni::physics::parse
{

// Consumer-side (engine) identity. Descriptors carry an `ObjectId` for
// each cross-reference that names a runtime engine entity; these are
// populated post-parse by the consumer when it registers parsed objects
// with the simulation backend. See `ObjectKey` (Handles.h) for the
// parse-time / source-side counterpart, and `omni/physx/ObjectId.h`
// for the full lifecycle doc.
using omni::physx::usdparser::ObjectId;
using omni::physx::usdparser::kInvalidObjectId;
using omni::physx::usdparser::MeshKey;
using omni::physx::usdparser::MeshKeyHash;
using omni::physx::ConvexMeshCookingParams;
using omni::physx::TriangleMeshCookingParams;
using omni::physx::SdfMeshCookingParams;
using omni::physx::ConvexDecompositionCookingParams;
using omni::physx::SphereFillCookingParams;

// Matrix4d lives in Math.h; both Descriptors.h and IPhysicsSource.h include it.

// ---------------------------------------------------------------------------
// Helper: sign-scale extraction — returns ±1 per component matching the
// sign of `scale`.
// ---------------------------------------------------------------------------

inline carb::Float3 scaleToSignScale(const carb::Float3& scale)
{
    return carb::Float3{ scale.x >= 0 ? 1.f : -1.f, scale.y >= 0 ? 1.f : -1.f, scale.z >= 0 ? 1.f : -1.f };
}

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

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

    eParticleSystem = 22 << 16,
    eParticleSet = 23 << 16,
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

    eDeformableMaterial = 41 << 16,
    eSurfaceDeformableMaterial,

    eDeformableBody = 42 << 16,
    eVolumeDeformableBody,
    eSurfaceDeformableBody,

    eDeformableAttachment = 43 << 16,
    eAttachmentVtxVtx,
    eAttachmentVtxTri,
    eAttachmentVtxTet,
    eAttachmentVtxCrv,
    eAttachmentVtxXform,
    eAttachmentTetXform,
    eAttachmentTriTri,

    eDeformableCollisionFilter = 44 << 16,
    eXformActor = 45 << 16,

    eNewtonMimicJoint = 46 << 16,
};

struct ObjectCategory
{
    ObjectCategory() : data(eUndefined) {}
    ObjectCategory(ObjectType type) : data(type & eCategoryMask) {}

    bool operator==(const ObjectType& category) const { return data == category; }
    bool operator==(const ObjectCategory& c) const { return data == c.data; }
    bool operator<(const ObjectCategory& c) const { return data < c.data; }

    uint32_t getData() const { return data; }

private:
    uint32_t data;
};

enum BodyType { eDynamic, eStatic, eKinematic };
enum CctShapeType { eBox, eCapsule };
enum Axis { eX, eY, eZ };
enum JointAxis { eDistance, eTransX, eTransY, eTransZ, eRotX, eRotY, eRotZ };
enum ArticulationJointType { eStandardJoint, eMaximalJoint };
enum VehicleUpdateMode { eVelocityChange, eAcceleration };
enum CollisionSystem { ePCM, eSAT };
enum SolverType { ePGS, eTGS };
enum BroadphaseType { eMBP, eSAP, eGPU };
enum SceneUpdateType { eSynchronous, eAsynchronous, eDisabled };
enum CombineMode { eAverage = 0, eMin, eMultiply, eMax };

// ---------------------------------------------------------------------------
// Instancing
// ---------------------------------------------------------------------------

struct ObjectInstance
{
    ObjectKey instancerKey;
    uint32_t index = 0;
    ObjectKey protoKey;
    bool isExclusive = false;
};

// ---------------------------------------------------------------------------
// Base descriptor
//
// Defaulting convention:
//   - Every field's stage-independent default is expressed as an in-class
//     default-initialiser on the member itself (or via the type's own
//     default ctor, e.g. `Matrix4d` → identity). A default-constructed
//     descriptor is therefore in a fully-defined, sensible state — no
//     uninitialised members, no UB on read.
//   - Descriptors whose defaults depend on stage units (metersPerUnit,
//     up-axis) get a free function `setToDefault(<Desc>&, const SourceUnits&)`
//     in the corresponding `Parse*.cpp` that *overlays* the unit-scaled
//     fields on top of the in-class defaults. These functions only touch
//     the units-aware members — they do not re-set what the in-class
//     initialisers already cover. Parsers MUST call `setToDefault` before
//     reading units-aware fields; the in-class defaults for those fields
//     are deliberately zero / safe-but-not-meaningful values.
//   - Descriptors with no units-dependent fields (e.g. PhysxMaterialDesc)
//     have no `setToDefault` and are construction-complete.
// ---------------------------------------------------------------------------

struct PhysxObjectDesc
{
    PhysxObjectDesc() : type(eUndefined) {}
    ObjectType type;
};

// ---------------------------------------------------------------------------
// Materials
// ---------------------------------------------------------------------------

struct PhysxMaterialDesc : PhysxObjectDesc
{
    // Constructor-complete: no units-aware fields, so no companion
    // `setToDefault` function. Friction defaults are 0.5 (PhysX
    // convention), not 0.0.
    PhysxMaterialDesc()
        : staticFriction(0.5f), dynamicFriction(0.5f), restitution(0.0f), density(-1.0f),
          frictionCombineMode(CombineMode::eAverage), restitutionCombineMode(CombineMode::eAverage),
          dampingCombineMode(CombineMode::eAverage), compliantAccelerationSpring(false),
          compliantStiffness(0.f), compliantDamping(0.f)
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

    ObjectKey materialKey;

    bool compliantAccelerationSpring;
    float compliantStiffness;
    float compliantDamping;
};

struct PhysxDeformableMaterialDesc : PhysxObjectDesc
{
    PhysxDeformableMaterialDesc()
        : staticFriction(0.0f), dynamicFriction(0.0f), density(-1.0f),
          youngsModulus(0.0f), poissonsRatio(0.0f), elasticityDamping(0.0f)
    {
        type = eDeformableMaterial;
    }

    float staticFriction;
    float dynamicFriction;
    float density;
    float youngsModulus;
    float poissonsRatio;
    float elasticityDamping;

    ObjectKey materialKey;
};

struct PhysxSurfaceDeformableMaterialDesc : PhysxDeformableMaterialDesc
{
    PhysxSurfaceDeformableMaterialDesc()
        : surfaceThickness(0.0f), surfaceStretchStiffness(0.0f),
          surfaceShearStiffness(0.0f), surfaceBendStiffness(0.0f), bendDamping(0.0f)
    {
        type = eSurfaceDeformableMaterial;
    }

    float surfaceThickness;
    float surfaceStretchStiffness;
    float surfaceShearStiffness;
    float surfaceBendStiffness;
    float bendDamping;
};

struct PBDMaterialDesc : PhysxObjectDesc
{
    // Explicit non-zero defaults: a default-constructed `PBDMaterialDesc`
    // is used as `PhysxSceneDesc::defaultPBDMaterialDesc` when the scene has
    // no material binding. PhysX's PBD particle simulation needs physically
    // meaningful `friction` / `density` / `*Scale` values to run -- zero would
    // either be rejected by PhysX setters or produce a degenerate simulation
    // (e.g. zero density makes particles massless). Values below match the
    // PhysX schema defaults documented for PBDMaterialAPI.
    PBDMaterialDesc()
        : friction(0.2f), particleFrictionScale(1.0f), damping(0.0f),
          viscosity(0.0f), vorticityConfinement(0.0f), surfaceTension(0.0f),
          cohesion(0.0f), adhesion(0.0f), particleAdhesionScale(1.0f),
          adhesionOffsetScale(0.0f), gravityScale(1.0f), cflCoefficient(1.0f),
          density(1000.0f)
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
    float gravityScale;
    float cflCoefficient;
    float density;

    ObjectKey materialKey;
};

// ---------------------------------------------------------------------------
// Scene
// ---------------------------------------------------------------------------

struct PhysxSceneDesc : PhysxObjectDesc
{
    PhysxSceneDesc() { type = eScene; }

    // Scene's own source-side identity. Populated by `scanStage`
    // so consumers iterating ScannedStage::scenes can resolve `primKey`
    // back to its source object via `ScannedStage::pathFor`.
    ObjectKey primKey;

    // True for a default scene synthesized when the source authored none (see
    // makeDefaultSceneDesc). The descriptor carries no backing prim, so the
    // consumer must skip source existence / ownership gates for it.
    bool synthetic = false;

    // Units-aware — set by setToDefault(PhysxSceneDesc&, SourceUnits&).
    carb::Float3 gravityDirection = { 0.0f, 0.0f, -1.0f };
    float gravityMagnitude = 9.81f;
    float frictionOffsetThreshold = 0.04f;
    float frictionCorrelationDistance = 0.025f;

    // Units-agnostic defaults below.
    uint32_t timeStepsPerSecond = 60;

    float bounceThreshold = 1e-06f;
    float maxBiasCoefficient = FLT_MAX;
    CollisionSystem collisionSystem = ePCM;
    SolverType solverType = eTGS;
    BroadphaseType broadphaseType = eGPU;
    bool enableCCD = false;
    bool enableStabilization = false;
    bool enableGPUDynamics = true;
    bool enableEnhancedDeterminism = false;
    bool enableExternalForcesEveryIteration = false;
    bool invertedFiltering = false;
    bool reportKineKine = false;
    bool reportKineStatic = false;
    SceneUpdateType sceneUpdateType = eSynchronous;
    bool supportSceneQueries = true;
    bool solveArticulationContactLast = false;

    bool enableQuasistatic = false;
    std::unordered_set<ObjectKey, ObjectKey::Hash> quasistaticActors;

    bool disableSleeping = false;

    uint64_t gpuTempBufferCapacity = (16ull * 1024ull * 1024ull);
    uint32_t gpuMaxRigidContactCount = (1024u * 512u);
    uint32_t gpuMaxRigidPatchCount = (1024u * 80u);
    uint32_t gpuHeapCapacity = (64u * 1024u * 1024u);
    uint32_t gpuFoundLostPairsCapacity = (256u * 1024u);
    uint32_t gpuFoundLostAggregatePairsCapacity = 1024u;
    uint32_t gpuTotalAggregatePairsCapacity = 1024u;
    uint32_t gpuMaxDeformableVolumeContacts = (1u * 1024u * 1024u);
    uint32_t gpuMaxDeformableSurfaceContacts = (1u * 1024u * 1024u);
    uint32_t gpuMaxParticleContacts = (1u * 1024u * 1024u);
    uint32_t gpuCollisionStackSize = (64u * 1024u * 1024u);
    uint32_t gpuMaxNumPartitions = 8;

    uint32_t minPosIterationCount = 0;
    uint32_t maxPosIterationCount = 255;
    uint32_t minVelIterationCount = 0;
    uint32_t maxVelIterationCount = 255;

    int32_t envIdInBoundsBitCount = -1;

    PhysxMaterialDesc defaultMaterialDesc;
    PBDMaterialDesc defaultPBDMaterialDesc;
    PhysxDeformableMaterialDesc defaultDeformableMaterialDesc;
    PhysxSurfaceDeformableMaterialDesc defaultSurfaceDeformableMaterialDesc;
};

// ---------------------------------------------------------------------------
// Collision groups and filtered pairs
// ---------------------------------------------------------------------------

struct FilteredPairDesc : PhysxObjectDesc
{
    FilteredPairDesc() { type = eFilteredPair; }
    ObjectIdPairVector pairs;
};

struct CollisionGroupDesc : PhysxObjectDesc
{
    CollisionGroupDesc() { type = eCollisionGroup; }

    // Source-side identity of the collision-group object. Populated
    // by `scanStage` so consumers can resolve back via
    // `ScannedStage::pathFor(primKey)`.
    ObjectKey primKey;

    // Source-side filtered-groups list — `ObjectKey`s naming the OTHER
    // collision-group objects this group filters contacts against.
    // Populated by `scanStage` from parse::parseCollisionGroup's
    // resolved output. Consumers translate keys to runtime
    // ObjectIds via their object database.
    std::vector<ObjectKey> sourceFilteredGroups;

    // Source-side member list — `ObjectKey`s of the objects that belong
    // to this group via the `colliders` collection. Populated by
    // `scanStage` from parse::parseCollisionGroup.
    std::vector<ObjectKey> sourceMembers;

    ObjectId groupId = kInvalidObjectId;
    std::vector<ObjectId> filteredGroups;
};

// ---------------------------------------------------------------------------
// Shapes
// ---------------------------------------------------------------------------

struct PhysxShapeDesc : PhysxObjectDesc
{
    PhysxShapeDesc()
        : localPos({ 0.0f, 0.0f, 0.0f }), localRot({ 0.0f, 0.0f, 0.0f, 1.0f }),
          localScale({ 1.0f, 1.0f, 1.0f }), collisionGroup(kInvalidObjectId), collisionEnabled(true)
    {
    }

    carb::Float3 localPos;
    carb::Float4 localRot;
    carb::Float3 localScale;

    // Runtime cross-references — populated by the consumer when the
    // shape is registered with the simulation engine.
    std::vector<ObjectId> materials;
    ObjectId collisionGroup;
    std::vector<ObjectId> sceneIds;

    // Source-side cross-references — populated by `scanStage`.
    // Mirror the schema ShapeDesc's materials / simulationOwners /
    // filteredCollisions cross-refs. Consumers iterating
    // ScannedStage::shapes translate these `ObjectKey`s to runtime
    // `ObjectId`s via their object database.
    std::vector<ObjectKey> sourceMaterials;
    std::vector<ObjectKey> sourceSimulationOwners;
    std::vector<ObjectKey> sourceFilteredCollisions;

    // The shape's own source-side key (the object where
    // UsdPhysicsCollisionAPI is applied). Distinct from `sourceGprim`,
    // which points to the geometry object that owns the points / mesh
    // attributes — they coincide for simple shapes (sphere / box / etc.)
    // and differ for mesh shapes whose collider sits on a parent of the
    // gprim. Consumers cross-referencing from
    // `PhysxRigidBodyDesc::sourceCollisions` resolve through this field,
    // not `sourceGprim`.
    ObjectKey primKey;
    ObjectKey rigidBody;
    ObjectKey sourceGprim;
    bool collisionEnabled;

    // Units-aware — set by setToDefault(PhysxShapeDesc&, SourceUnits&).
    // Sentinel `-1.0f` for non-plane shapes means "recompute in PhysX layer".
    float contactOffset = -1.0f;

    // Units-agnostic defaults below: parsers may override via schema reads.
    float restOffset = 0.0f;
    float torsionalPatchRadius = 0.0f;
    float minTorsionalPatchRadius = 0.0f;
    bool isTrigger = false;
    bool isTriggerUsdOutput = false;
};

struct CustomPhysxShapeDesc : PhysxShapeDesc
{
    CustomPhysxShapeDesc() : customGeometryTokenHash(0) { type = eCustomShape; }
    // Stable content hash of the custom-geometry schema-API token (e.g.
    // "PhysxMeshMergeCollisionAPI"). USD callers compute this via
    // `omni::physx::computeCustomGeometryHash(TfToken)`; non-USD callers
    // use the equivalent string overload. Matches the key type
    // `PhysXCustomGeometryManager::mCustomGeometryTypeMap` uses internally.
    size_t customGeometryTokenHash;
};

struct SpherePhysxShapeDesc : PhysxShapeDesc
{
    SpherePhysxShapeDesc(float inRadius = 0.0f) : PhysxShapeDesc(), radius(inRadius) { type = eSphereShape; }
    float radius;
};

// Owned-storage merged-mesh data. Populated by the merged-mesh
// pre-process (which gathers child UsdGeomMesh prims under a
// `PhysxMeshMergeCollisionAPI` parent and concatenates their points /
// indices / faces / holes into a single buffer set). Stored as
// `std::vector<T>` so the cooking layer can read raw `data() + size()`
// without needing IPhysicsSource access — points are already
// `carb::Float3` (3 packed floats, identical layout to USD's GfVec3f).
struct MergeMeshDesc
{
    std::vector<carb::Float3> points;
    std::vector<int32_t>      indices;
    std::vector<int32_t>      faces;
    std::vector<int32_t>      holes;
};

struct MergeMeshPhysxShapeDesc : PhysxShapeDesc
{
    MergeMeshPhysxShapeDesc() : PhysxShapeDesc(), mergedMesh(nullptr) {}
    MergeMeshDesc* mergedMesh;
};

struct BoundingSpherePhysxShapeDesc : MergeMeshPhysxShapeDesc
{
    BoundingSpherePhysxShapeDesc() : MergeMeshPhysxShapeDesc() { type = eBoundingSphereShape; }
    carb::Float3 positionOffset = { 0.0f, 0.0f, 0.0f };
    float radius = 0.0f;
};

struct BoundingBoxPhysxShapeDesc : MergeMeshPhysxShapeDesc
{
    BoundingBoxPhysxShapeDesc() : MergeMeshPhysxShapeDesc() { type = eBoundingBoxShape; }
    carb::Float3 positionOffset = { 0.0f, 0.0f, 0.0f };
    carb::Float4 rotationOffset = { 0.0f, 0.0f, 0.0f, 1.0f };
    carb::Float3 halfExtents = { 0.0f, 0.0f, 0.0f };
};

struct CapsulePhysxShapeDesc : PhysxShapeDesc
{
    CapsulePhysxShapeDesc(float inRadius = 0.0f, float half_height = 0.0f, Axis cap_axis = eX)
        : PhysxShapeDesc(), radius(inRadius), halfHeight(half_height), axis(cap_axis) { type = eCapsuleShape; }
    float radius;
    float halfHeight;
    Axis axis;
};

struct CylinderPhysxShapeDesc : PhysxShapeDesc
{
    CylinderPhysxShapeDesc(float inRadius = 0.0f, float half_height = 0.0f, Axis cap_axis = eX, float inMargin = 0.0f)
        : PhysxShapeDesc(), radius(inRadius), halfHeight(half_height), axis(cap_axis), margin(inMargin) { type = eCylinderShape; }
    float radius;
    float halfHeight;
    Axis axis;
    float margin;
};

struct ConePhysxShapeDesc : PhysxShapeDesc
{
    ConePhysxShapeDesc(float inRadius = 0.0f, float half_height = 0.0f, Axis cap_axis = eX, float inMargin = 0.0f)
        : PhysxShapeDesc(), radius(inRadius), halfHeight(half_height), axis(cap_axis), margin(inMargin) { type = eConeShape; }
    float radius;
    float halfHeight;
    Axis axis;
    float margin;
};

struct BoxPhysxShapeDesc : PhysxShapeDesc
{
    BoxPhysxShapeDesc(float x = 0.0f, float y = 0.0f, float z = 0.0f) : PhysxShapeDesc(), halfExtents({ x, y, z }) { type = eBoxShape; }
    BoxPhysxShapeDesc(const carb::Float3& inHalfExtents) : PhysxShapeDesc(), halfExtents(inHalfExtents) { type = eBoxShape; }
    carb::Float3 halfExtents;
};

struct InfiniteVoxelMapDesc : PhysxObjectDesc
{
    InfiniteVoxelMapDesc(ObjectKey key = {}) : PhysxObjectDesc(), rootPrim(key) { type = eInfiniteVoxelMap; }
    ObjectKey rootPrim;
};

struct ConvexMeshPhysxShapeDesc : MergeMeshPhysxShapeDesc
{
    ConvexMeshPhysxShapeDesc(size_t = 0, const carb::Float3* = nullptr, MeshKey inCrc = MeshKey())
        : MergeMeshPhysxShapeDesc(), crc(inCrc) { type = eConvexMeshShape; }

    // Source-side ObjectKey for the mesh prim. Named `meshPrimKey` to
    // disambiguate from the cooking-system `meshKey` (content hash, type
    // `omni::physx::MeshKey`) declared below.
    ObjectKey meshPrimKey;
    carb::Float3 meshScale = { 1.0f, 1.0f, 1.0f };
    MeshKey crc;
    MeshKey meshKey;
    uint32_t numVerts{ 0 };
    ConvexMeshCookingParams convexCookingParams;
};

struct TriangleMeshPhysxShapeDesc : MergeMeshPhysxShapeDesc
{
    TriangleMeshPhysxShapeDesc(MeshKey inCrc = MeshKey())
        : MergeMeshPhysxShapeDesc(), meshScale({ 1.f, 1.f, 1.f }), crc(inCrc), doubleSided(false) { type = eTriangleMeshShape; }

    // Source-side ObjectKey for the mesh prim. See the ConvexMeshPhysxShapeDesc
    // note above for the naming rationale.
    ObjectKey meshPrimKey;
    carb::Float3 meshScale;
    MeshKey crc;
    MeshKey meshKey;
    bool doubleSided;
    SdfMeshCookingParams sdfMeshCookingParams;
    TriangleMeshCookingParams triangleMeshCookingParams;
};

struct ConvexMeshDecompositionPhysxShapeDesc : TriangleMeshPhysxShapeDesc
{
    ConvexMeshDecompositionPhysxShapeDesc(MeshKey inCrc = MeshKey()) : TriangleMeshPhysxShapeDesc(inCrc) { type = eConvexMeshDecompositionShape; }
    ConvexDecompositionCookingParams convexDecompositionCookingParams;
};

struct SpherePhysxPoint
{
    carb::Float3 position = { 0.0f, 0.0f, 0.0f };
    float radius = 0.0f;
};

struct SpherePointsPhysxShapeDesc : TriangleMeshPhysxShapeDesc
{
    SpherePointsPhysxShapeDesc() : TriangleMeshPhysxShapeDesc() { type = eSpherePointsShape; }
    SphereFillCookingParams sphereFillCookingParams;
    std::vector<SpherePhysxPoint> spheres;
};

struct PlanePhysxShapeDesc : PhysxShapeDesc
{
    PlanePhysxShapeDesc() : PhysxShapeDesc(), axis(eX) { type = ePlaneShape; }
    Axis axis;
};

// ---------------------------------------------------------------------------
// Rigid bodies
// ---------------------------------------------------------------------------

struct PhysxRigidBodyDesc : PhysxObjectDesc
{
    PhysxRigidBodyDesc()
        : position({ 0.0f, 0.0f, 0.0f }), rotation({ 0.0f, 0.0f, 0.0f, 1.0f }), scale({ 1.0f, 1.0f, 1.0f }) {}

    // Body's own source-side identity. Populated by `scanStage` so
    // consumers iterating the typed list can resolve `primKey` back
    // to the source via `ScannedStage::pathFor` — mirrors
    // `materialKey` on PhysxMaterialDesc, `jointPrimKey` on
    // PhysxJointDesc, `rootPrim` on PhysxArticulationDesc.
    ObjectKey primKey;

    // Runtime cross-references — populated by the consumer when the
    // body is registered with the simulation engine. Empty at parse
    // time; the source-side equivalents below carry the parse-time
    // identities consumers translate from.
    std::vector<ObjectId> shapes;
    std::vector<ObjectId> collisionBlocks;
    std::vector<ObjectId> sceneIds;

    // Source-side cross-references — populated by `scanStage` from
    // the schema parser's resolved `RigidBodyDesc` (or by non-USD
    // sources via the equivalent natively). Consumers translate
    // these `ObjectKey`s to runtime `ObjectId`s via their object
    // database during the post-scan iteration.
    //
    // `sourceCollisions` mirrors the schema desc's collision set —
    // `ObjectKey`s of the collision shapes the body owns.
    // `sourceSimulationOwners` — `ObjectKey`s of the scene objects the
    // body simulates under.
    // `sourceFilteredCollisions` — per-body filter pairs from
    // PhysicsFilteredPairsAPI.
    std::unordered_set<ObjectKey, ObjectKey::Hash> sourceCollisions;
    std::vector<ObjectKey> sourceSimulationOwners;
    std::vector<ObjectKey> sourceFilteredCollisions;

    carb::Float3 position;
    carb::Float4 rotation;
    carb::Float3 scale;
};

struct PointInstancedBodyDesc : PhysxObjectDesc
{
    PointInstancedBodyDesc() { type = ePointInstancedBody; }
};

struct StaticPhysxRigidBodyDesc : PhysxRigidBodyDesc
{
    StaticPhysxRigidBodyDesc() : PhysxRigidBodyDesc() { type = eStaticBody; }
    ObjectKey sourceGPrimKey;
};

struct DynamicPhysxRigidBodyDesc : PhysxRigidBodyDesc
{
    DynamicPhysxRigidBodyDesc() { type = eDynamicBody; }

    bool kinematicBody = false;
    bool hasTimeSampledXform = false;
    carb::Float3 linearVelocity = { 0.0f, 0.0f, 0.0f };
    carb::Float3 angularVelocity = { 0.0f, 0.0f, 0.0f };
    bool startsAsleep = false;

    // Units-agnostic defaults below: parsers override via schema reads.
    float linearDamping = 0.0f;
    float angularDamping = 0.05f;
    float maxLinearVelocity = 1e16f;       // SQRT_FLT_MAX — PhysX velocity-squared cap
    float maxAngularVelocity = 100.0f;     // rad/s, already converted
    float maxContactImpulse = FLT_MAX;
    float contactSlopCoefficient = 0.0f;
    float cfmScale = 0.025f;
    int solverPositionIterationCount = 16;
    int solverVelocityIterationCount = 1;

    // Units-aware — set by setToDefault(DynamicPhysxRigidBodyDesc&, SourceUnits&).
    float sleepThreshold = 0.0f;
    float stabilizationThreshold = 0.0f;
    float maxDepenetrationVelocity = 0.0f;

    bool enableCCD = false;
    bool enableSpeculativeCCD = false;
    bool disableGravity = false;
    bool retainAccelerations = false;
    bool enableGyroscopicForces = true;
    bool localSpaceVelocities = false;
    bool solveContacts = true;

    int lockedPosAxis = 0;
    int lockedRotAxis = 0;

    bool surfaceVelocityEnabled = false;
    bool surfaceVelocityLocalSpace = false;
    carb::Float3 surfaceLinearVelocity = { 0.0f, 0.0f, 0.0f };
    carb::Float3 surfaceAngularVelocity = { 0.0f, 0.0f, 0.0f };

    bool splinesSurfaceVelocityEnabled = false;
    float splinesSurfaceVelocityMagnitude = 0.0f;
    ObjectKey splinesCurvePrimKey;
};

// ---------------------------------------------------------------------------
// Force
// ---------------------------------------------------------------------------

struct PhysxForceDesc : PhysxObjectDesc
{
    PhysxForceDesc() { type = ePhysxForce; }

    carb::Float3 force = { 0.0f, 0.0f, 0.0f };
    carb::Float3 torque = { 0.0f, 0.0f, 0.0f };
    bool worldFrame = false;
    bool accelerationMode = false;
    bool enabled = false;
    carb::Float3 worldPos = { 0.0f, 0.0f, 0.0f };
    carb::Float4 localRot = { 0.0f, 0.0f, 0.0f, 1.0f };
    ObjectId body = kInvalidObjectId;
    ObjectId scene = kInvalidObjectId;
};

// ---------------------------------------------------------------------------
// Joint sub-types
// ---------------------------------------------------------------------------

struct PhysxJointLimit
{
    bool enabled = false;
    union { float angle0 = (float)M_PI_2; float lower; float minDist; };
    union { float angle1 = (float)-M_PI_2; float upper; float maxDist; };
    float restitution = 0.0f;
    float bounceThreshold = 0.0f;
    float stiffness = 0.0f;
    float damping = 0.0f;
};

struct PhysxJointDrive
{
    PhysxJointDrive()
        : enabled(false), targetPosition(0.0f), targetVelocity(0.0f), forceLimit(FLT_MAX),
          stiffness(0.0f), damping(0.0f), acceleration(false), isEnvelopeUsed(false),
          maxActuatorVelocity(FLT_MAX), velocityDependentResistance(0), speedEffortGradient(0)
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
    float maxJointVelocity = 0.0f;
    float armature = 0.0f;
    float staticFrictionEffort = 0.0f;
    float dynamicFrictionEffort = 0.0f;
    float viscousFrictionCoefficient = 0.0f;
};

struct PhysicsJointState
{
    PhysicsJointState() : enabled(false), position(0.0f), velocity(0.0f) {}
    bool enabled;
    float position;
    float velocity;
};

using JointLimits = std::vector<std::pair<JointAxis, PhysxJointLimit>>;
using JointDrives = std::vector<std::pair<JointAxis, PhysxJointDrive>>;
using JointStates = std::vector<std::pair<JointAxis, PhysicsJointState>>;
using JointAxisProperties = std::vector<std::pair<JointAxis, PhysxJointAxisProperties>>;

// ---------------------------------------------------------------------------
// Joints
// ---------------------------------------------------------------------------

struct PhysxJointDesc : PhysxObjectDesc
{
    PhysxJointDesc()
        : localPose0Position({ 0.0f, 0.0f, 0.0f }), localPose0Orientation({ 1.0f, 0.0f, 0.0f, 0.0f }),
          localPose1Position({ 0.0f, 0.0f, 0.0f }), localPose1Orientation({ 1.0f, 0.0f, 0.0f, 0.0f }),
          jointEnabled(true), breakForce(FLT_MAX), breakTorque(FLT_MAX), jointFriction(0.0f)
    {
    }

    ObjectKey jointPrimKey;
    ObjectKey rel0;
    ObjectKey rel1;
    ObjectKey body0;
    ObjectKey body1;
    carb::Float3 localPose0Position;
    carb::Float4 localPose0Orientation;
    carb::Float3 localPose1Position;
    carb::Float4 localPose1Orientation;
    bool jointEnabled;
    float breakForce;
    float breakTorque;
    float jointFriction;
    bool enableCollision = false;            // USD `physics:collisionEnabled` default
    bool excludedFromArticulation = false;   // USD `physics:excludeFromArticulation` default
    bool validBodyTransformations = true;    // runtime-side validity flag; parsers default to true
};

struct FixedPhysxJointDesc : PhysxJointDesc
{
    FixedPhysxJointDesc() { type = eJointFixed; }
};

struct D6PhysxJointDesc : PhysxJointDesc
{
    D6PhysxJointDesc() { type = eJointD6; }
    JointLimits jointLimits;
    JointDrives jointDrives;
    JointStates jointStates;
    JointAxisProperties jointProperties;
};

struct PrismaticPhysxJointDesc : PhysxJointDesc
{
    PrismaticPhysxJointDesc() : axis(eX) { type = eJointPrismatic; }
    Axis axis;
    PhysxJointLimit limit;
    PhysxJointDrive drive;
    PhysicsJointState state;
    PhysxJointAxisProperties properties;
};

struct SphericalPhysxJointDesc : PhysxJointDesc
{
    SphericalPhysxJointDesc() : axis(eX) { type = eJointSpherical; }
    Axis axis;
    PhysxJointLimit limit;
    PhysicsJointState state;
    JointAxisProperties jointProperties;
};

struct RevolutePhysxJointDesc : PhysxJointDesc
{
    RevolutePhysxJointDesc() : axis(eX) { type = eJointRevolute; }
    Axis axis;
    PhysxJointLimit limit;
    PhysxJointDrive drive;
    PhysicsJointState state;
    PhysxJointAxisProperties properties;
};

struct DistancePhysxJointDesc : PhysxJointDesc
{
    DistancePhysxJointDesc() : minEnabled(false), maxEnabled(false), springEnabled(false), damping(0.0f), stiffness(0.0)
    { type = eJointDistance; }

    bool minEnabled;
    bool maxEnabled;
    PhysxJointLimit limit;
    PhysicsJointState state;
    bool springEnabled;
    float damping;
    float stiffness;
};

struct GearPhysxJointDesc : PhysxJointDesc
{
    GearPhysxJointDesc() : gearRatio(0.0f) { type = eJointGear; }
    ObjectKey hingePrimPath0;
    ObjectKey hingePrimPath1;
    float gearRatio;
};

struct RackPhysxJointDesc : PhysxJointDesc
{
    RackPhysxJointDesc() : ratio(0.0f) { type = eJointRackAndPinion; }
    ObjectKey hingePrimKey;
    ObjectKey prismaticPrimKey;
    float ratio;
};

struct CustomPhysxJointDesc : PhysxJointDesc
{
    CustomPhysxJointDesc() { type = eJointCustom; }
    TokenId customJointToken;
};

// ---------------------------------------------------------------------------
// Articulations
// ---------------------------------------------------------------------------

struct PhysxArticulationDesc : PhysxObjectDesc
{
    PhysxArticulationDesc() : fixBase(false), sceneId(kInvalidObjectId) { type = eArticulation; }

    // The source-side prim that carries `UsdPhysicsArticulationRootAPI` —
    // i.e. the articulation "owner" prim, as opposed to the per-root
    // descriptors that all share the same owner. Consumers key
    // articulation maps by this so multi-root articulations group under
    // one entry.
    ObjectKey articulationPrim;

    ObjectKey rootPrim;
    ObjectKey staticRootBodyPrim;
    bool fixBase;
    ObjectKey fixBaseKey;
    // Defaults mirror ArticulationFields (ParseApi.h). sleepThreshold /
    // stabilizationThreshold are units-aware — `setToDefault(ArticulationFields,
    // SourceUnits)` overlays the metersPerUnit-scaled values; the in-class
    // defaults here are deliberately zero / safe-but-not-meaningful per the
    // descriptor defaulting convention above.
    int solverPositionIterationCount = 32;
    int solverVelocityIterationCount = 1;
    float sleepThreshold = 0.0f;
    float stabilizationThreshold = 0.0f;
    bool selfCollision = true;
    std::unordered_set<ObjectKey, ObjectKey::Hash> articulatedJoints;
    std::unordered_set<ObjectKey, ObjectKey::Hash> articulatedBodies;

    // Source-side cross-references — articulation-level filtered
    // collision pairs (`ObjectKey`s from PhysicsFilteredPairsAPI on the
    // articulation root). Mirrors `schema::ArticulationDesc::filteredCollisions`.
    std::vector<ObjectKey> sourceFilteredCollisions;

    ObjectId sceneId;
};

struct PhysxArticulationLinkDesc : DynamicPhysxRigidBodyDesc
{
    PhysxArticulationLinkDesc()
        : DynamicPhysxRigidBodyDesc(), articulation(kInvalidObjectId), parent(kInvalidObjectId),
          articulationJointType(eStandardJoint), articulationJoint(nullptr)
    {
        type = eArticulationLink;
    }

    ObjectId articulation;
    ObjectId parent;
    ArticulationJointType articulationJointType;
    const PhysxJointDesc* articulationJoint;
};

// ---------------------------------------------------------------------------
// Tendons
// ---------------------------------------------------------------------------

struct PhysxTendonAxisDesc : PhysxObjectDesc
{
    PhysxTendonAxisDesc()
        : gearings(1u), forceCoefficients(1u), axes(1u), parentAxisId(kInvalidObjectId), wasVisited(false)
    {
        type = eTendonAxis;
        forceCoefficients[0] = 1.0f;
    }

    TokenId instanceToken;
    ObjectKey jointKey;
    ObjectKey link0;
    ObjectKey link1;

    std::vector<float> gearings;
    std::vector<float> forceCoefficients;
    std::vector<JointAxis> axes;

    ObjectId parentAxisId;
    bool wasVisited;
};

struct PhysxTendonFixedDesc : PhysxObjectDesc
{
    PhysxTendonFixedDesc()
        : stiffness(0.f), damping(0.f), restLength(0.f), offset(0.f),
          limitStiffness(0.f), lowLimit(-FLT_MAX), highLimit(FLT_MAX),
          isEnabled(true), rootAxis(nullptr)
    {
        type = eTendonFixed;
    }

    TokenId instanceToken;
    ObjectKey jointKey;

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
    { type = eTendonAttachment; }

    float gearing;
    carb::Float3 localPos;
    ObjectKey parentKey;
    ObjectKey linkKey;
    TokenId parentToken;
    TokenId instanceToken;
    ObjectId parentId;
};

struct PhysxTendonSpatialDesc : PhysxTendonAttachmentDesc
{
    PhysxTendonSpatialDesc() : stiffness(0.f), damping(0.f), limitStiffness(0.f), offset(0.f), isEnabled(true)
    { type = eTendonAttachmentRoot; }

    float stiffness;
    float damping;
    float limitStiffness;
    float offset;
    bool isEnabled;
};

struct PhysxTendonAttachmentLeafDesc : PhysxTendonAttachmentDesc
{
    PhysxTendonAttachmentLeafDesc() : lowLimit(-FLT_MAX), highLimit(FLT_MAX), restLength(-FLT_MAX)
    { type = eTendonAttachmentLeaf; }

    float lowLimit;
    float highLimit;
    float restLength;
};

// ---------------------------------------------------------------------------
// Mimic joints
// ---------------------------------------------------------------------------

struct MimicJointDesc : PhysxObjectDesc
{
    MimicJointDesc() { type = eUndefined; }

    ObjectKey mimicJointKey;
    ObjectId mimicJointId = kInvalidObjectId;
    int mimicJointAxis = -1;  // eDEFAULT_AXIS

    ObjectKey referenceJointKey;
    ObjectId referenceJointId = kInvalidObjectId;
    int referenceJointAxis = -1;  // eDEFAULT_AXIS

    float gearing = 0.0f;
    float offset = 0.0f;
    float naturalFrequency = 0.0f;
    float dampingRatio = 0.0f;

    static constexpr int eDEFAULT_AXIS = -1;
};

// ---------------------------------------------------------------------------
// Deformable bodies
// ---------------------------------------------------------------------------

struct PhysxDeformableBodyDesc : PhysxObjectDesc
{
    PhysxDeformableBodyDesc() = default;

    // Body's own source-side identity. Populated by `scanStage` so
    // consumers iterating ScannedStage::deformables can resolve
    // `primKey` back to its source via `ScannedStage::pathFor`.
    ObjectKey primKey;

    bool bodyEnabled = false;
    bool kinematicBody = false;
    bool startsAsleep = false;
    bool enableSpeculativeCCD = false;
    bool selfCollision = false;
    bool disableGravity = false;

    // Units-aware — set by setToDefault(PhysxDeformableBodyDesc&, SourceUnits&).
    float sleepThreshold = 0.0f;
    float settlingThreshold = 0.0f;
    float maxDepenetrationVelocity = 0.0f;
    float restOffset = 0.0f;

    // Units-agnostic defaults below.
    float linearDamping = 0.005f;
    // sqrtf(FLT_MAX) ≈ 1.844674e19f — PhysX velocity cap for deformables.
    float maxLinearVelocity = 1.844674e19f;
    float settlingDamping = 10.0f;
    float contactOffset = -1.0f;            // sentinel: recompute in PhysX layer
    float selfCollisionFilterDistance = -1.0f;
    uint32_t solverPositionIterationCount = 16;
    ObjectId sceneId = kInvalidObjectId;

    bool isAutoMeshSimplificationEnabled = false;
    bool isAutoRemeshingEnabled = false;
    bool hasAutoForceConforming = false;
    uint32_t autoRemeshingResolution = 0;
    uint32_t autoTriangleTargetCount = 0;

    Matrix4d transform;                     // defaults to identity (see Math.h)
    float mass = -1.0f;

    ObjectKey simMeshKey;
    ObjectId simMeshMaterial = kInvalidObjectId;
    TokenId simMeshBindPoseToken;
    bool simMeshLeftHandedOrientation = false;

    ObjectKey collisionMeshKey;
    TokenId collisionMeshBindPoseToken;
    bool collisionMeshLeftHandedOrientation = false;
    ObjectId collisionGroup = kInvalidObjectId;

    std::vector<ObjectKey> skinGeomPaths;
    std::vector<TokenId> skinGeomBindPoseTokens;

    ObjectKey cookingSrcMeshKey;
    TokenId cookingSrcMeshBindPoseToken;

    bool hasAutoAPI = false;

    // Source-side cross-references — populated by `scanStage` from the
    // body's OmniPhysicsBodyAPI simulationOwner rel and from
    // PhysicsFilteredPairsAPI on the body prim. Consumers translate
    // these `ObjectKey`s to runtime `ObjectId`s post-scan. Mirrors the
    // matching fields on PhysxRigidBodyDesc.
    std::vector<ObjectKey> sourceSimulationOwners;
    std::vector<ObjectKey> sourceFilteredCollisions;
};

struct PhysxVolumeDeformableBodyDesc : PhysxDeformableBodyDesc
{
    PhysxVolumeDeformableBodyDesc() { type = eVolumeDeformableBody; }
    bool isAutoHexahedralMeshEnabled = false;
    uint32_t autoHexahedralResolution = 0;
};

struct PhysxSurfaceDeformableBodyDesc : PhysxDeformableBodyDesc
{
    PhysxSurfaceDeformableBodyDesc() { type = eSurfaceDeformableBody; }
    TokenId restBendAnglesDefault;
    uint32_t collisionPairUpdateFrequency = 0;
    uint32_t collisionIterationMultiplier = 0;
};

// ---------------------------------------------------------------------------
// Vehicle
// ---------------------------------------------------------------------------

struct TireFrictionTableDesc : PhysxObjectDesc
{
    TireFrictionTableDesc() { type = eVehicleTireFrictionTable; }

    ObjectKey key;
    std::vector<ObjectKey> materialPaths;
    std::vector<ObjectId> materialIds;
    std::vector<float> frictionValues;
    float defaultFrictionValue = 0.0f;
};

struct WheelDesc : PhysxObjectDesc
{
    WheelDesc() { type = eVehicleWheel; }
    ObjectKey key;
    float radius = 0.0f;
    float width = 0.0f;
    float mass = 0.0f;
    float moi = 0.0f;
    float dampingRate = 0.0f;
    float maxBrakeTorque = 0.0f;
    float maxHandBrakeTorque = 0.0f;
    float maxSteerAngle = 0.0f;
    float toeAngle = 0.0f;
};

struct TireDesc : PhysxObjectDesc
{
    TireDesc() { type = eVehicleTire; }
    ObjectKey key;
    float latStiffX = 0.0f;
    float latStiffY = 0.0f;
    carb::Float2 lateralStiffnessGraph = { 0.0f, 0.0f };
    float longitudinalStiffnessPerUnitGravity = 0.0f;
    float longitudinalStiffness = 0.0f;
    float camberStiffnessPerUnitGravity = 0.0f;
    float camberStiffness = 0.0f;
    carb::Float2 frictionVsSlipGraph[3] = {};
    ObjectId frictionTableId = kInvalidObjectId;
    ObjectKey frictionTableKey;
    float restLoad = 0.0f;
};

struct SuspensionDesc : PhysxObjectDesc
{
    SuspensionDesc() { type = eVehicleSuspension; }
    ObjectKey key;
    float springStrength = 0.0f;
    float springDamperRate = 0.0f;
    float travelDistance = 0.0f;
    float maxCompression = 0.0f;
    float maxDroop = 0.0f;
    float camberAtRest = 0.0f;
    float camberAtMaxCompression = 0.0f;
    float camberAtMaxDroop = 0.0f;
    float sprungMass = 0.0f;
};

struct SuspensionComplianceDesc
{
    std::vector<carb::Float2> wheelToeAngleList;
    std::vector<carb::Float2> wheelCamberAngleList;
    std::vector<carb::Float4> suspensionForceAppPointList;
    std::vector<carb::Float4> tireForceAppPointList;
};

// Parse-lib mirror of the consumer-side `WheelAttachmentDesc` in
// `private/omni/physx/PhysxUsd.h`. Same field layout; the only divergence
// is that `key` / `collisionGroupKey` / `shapeKey` here correspond to
// `path` / `collisionGroupPath` / `shapePath` (all USD `SdfPath`-typed)
// in the legacy mirror. Keep the field set in lockstep -- dedup into a
// single `using` alias is tracked by ADR-0008
// once the vehicle consumer's SdfPath-keyed bookkeeping is flipped to
// ObjectKey.
struct WheelAttachmentDesc : PhysxObjectDesc
{
    enum State
    {
        eMANAGE_TRANSFORMS = (1 << 0),
        eHAS_SHAPE = (1 << 1),
        eHAS_WHEEL_COM_OFFSET = (1 << 2),
        eHAS_SUSP_FORCE_APP_POINT = (1 << 3),
        eHAS_TIRE_FORCE_APP_POINT = (1 << 4),
        eHAS_SUSPENSION_FRAME = (1 << 5)
    };

    WheelAttachmentDesc() { type = eVehicleWheelAttachment; }

    ObjectKey key;
    ObjectId id = kInvalidObjectId;

    WheelDesc* wheel = nullptr;
    ObjectId wheelId = kInvalidObjectId;
    TireDesc* tire = nullptr;
    ObjectId tireId = kInvalidObjectId;
    SuspensionDesc* suspension = nullptr;
    ObjectId suspensionId = kInvalidObjectId;
    SuspensionComplianceDesc* suspensionCompliance = nullptr;

    carb::Float3 suspensionTravelDirection = { 0.0f, 0.0f, 0.0f };
    carb::Float3 suspensionForceAppPointOffset = { 0.0f, 0.0f, 0.0f };
    carb::Float3 wheelCenterOfMassOffset = { 0.0f, 0.0f, 0.0f };
    carb::Float3 tireForceAppPointOffset = { 0.0f, 0.0f, 0.0f };
    carb::Float3 suspensionFramePosition = { 0.0f, 0.0f, 0.0f };
    carb::Float4 suspensionFrameOrientation = { 0.0f, 0.0f, 0.0f, 1.0f };
    carb::Float3 wheelFramePosition = { 0.0f, 0.0f, 0.0f };
    carb::Float4 wheelFrameOrientation = { 0.0f, 0.0f, 0.0f, 1.0f };

    int index = 0;
    bool driven = false;

    ObjectId collisionGroupId = kInvalidObjectId;
    ObjectKey collisionGroupKey;
    ObjectKey shapeKey;
    ObjectId shapeId = kInvalidObjectId;
    uint8_t state = 0;
};

struct WheelControllerDesc : PhysxObjectDesc
{
    WheelControllerDesc() { type = eVehicleWheelController; }
    ObjectKey key;
    ObjectId id = kInvalidObjectId;
    float driveTorque = 0.0f;
    float brakeTorque = 0.0f;
    float steerAngle = 0.0f;
};

struct EngineDesc : PhysxObjectDesc
{
    EngineDesc() { type = eVehicleEngine; }
    static constexpr uint32_t maxNumberOfTorqueCurvePoints = 8;

    ObjectKey key;
    float moi = 0.0f;
    float peakTorque = 0.0f;
    float maxRotationSpeed = 0.0f;
    float idleRotationSpeed = 0.0f;
    carb::Float2 torqueCurve[maxNumberOfTorqueCurvePoints] = {};
    unsigned int torqueCurvePointCount = 0;
    float dampingRateFullThrottle = 0.0f;
    float dampingRateZeroThrottleClutchEngaged = 0.0f;
    float dampingRateZeroThrottleClutchDisengaged = 0.0f;
};

struct GearsDesc
{
    std::vector<float> ratios;
    float ratioScale = 0.0f;
    float switchTime = 0.0f;
    static constexpr uint32_t maxNumberOfGears = 32;
};

struct AutoGearBoxDesc
{
    std::vector<float> upRatios;
    std::vector<float> downRatios;
    float latency = 0.0f;
};

struct ClutchDesc
{
    float strength = 0.0f;
};

struct NonlinearCmdResponseDesc
{
    std::vector<float> commandValues;
    std::vector<int> speedResponsesPerCommandValue;
    std::vector<carb::Float2> speedResponses;
    static constexpr uint32_t maxNumberOfCommandValues = 8;
    static constexpr uint32_t maxNumberOfSpeedResponses = 64;
};

struct DriveDesc : PhysxObjectDesc {};

struct DriveBasicDesc : DriveDesc
{
    DriveBasicDesc() { type = eVehicleDriveBasic; }
    NonlinearCmdResponseDesc* nonlinearCmdResponse = nullptr;
    ObjectKey key;
    ObjectId id = kInvalidObjectId;
    float peakTorque = 0.0f;
};

struct DriveStandardDesc : DriveDesc
{
    DriveStandardDesc() { type = eVehicleDriveStandard; }
    EngineDesc* engine = nullptr;
    ObjectId engineId = kInvalidObjectId;
    const GearsDesc* gears = nullptr;
    const AutoGearBoxDesc* autoGearBox = nullptr;
    const ClutchDesc* clutch = nullptr;
};

struct DifferentialDesc
{
    enum Type { eMultiWheel, eTank };
    Type type = eMultiWheel;
};

struct MultiWheelDifferentialDesc : DifferentialDesc
{
    MultiWheelDifferentialDesc() { type = eMultiWheel; }
    std::vector<int> wheels;
    std::vector<float> torqueRatios;
    std::vector<float> averageWheelSpeedRatios;
};

struct TankDifferentialDesc : MultiWheelDifferentialDesc
{
    TankDifferentialDesc() { type = eTank; }
    std::vector<int> numberOfWheelsPerTrack;
    std::vector<int> thrustIndexPerTrack;
    std::vector<int> trackToWheelIndices;
    std::vector<int> wheelIndicesInTrackOrder;
};

struct BrakesDesc
{
    NonlinearCmdResponseDesc* nonlinearCmdResponse = nullptr;
    std::vector<int> wheels;
    std::vector<float> torqueMultipliers;
    float maxBrakeTorque = 0.0f;
    uint8_t brakesIndex = 0;
};

struct SteeringDesc
{
    enum Type { eBasic, eAckermann };
    NonlinearCmdResponseDesc* nonlinearCmdResponse = nullptr;
    Type type = eBasic;
};

struct SteeringBasicDesc : SteeringDesc
{
    SteeringBasicDesc() { type = eBasic; }
    std::vector<int> wheels;
    std::vector<float> angleMultipliers;
    float maxSteerAngle = 0.0f;
};

struct SteeringAckermannDesc : SteeringDesc
{
    SteeringAckermannDesc() { type = eAckermann; }
    int wheel0 = 0;
    int wheel1 = 0;
    float maxSteerAngle = 0.0f;
    float wheelBase = 0.0f;
    float trackWidth = 0.0f;
    float strength = 0.0f;
};

struct VehicleContextDesc : PhysxObjectDesc
{
    enum AxisDir { ePosX, eNegX, ePosY, eNegY, ePosZ, eNegZ, eUndefined };

    VehicleContextDesc() { type = eVehicleContext; }

    // Resets a VehicleContextDesc to factory defaults (equivalent to the
    // in-class initialisers below) without re-assigning the descriptor.
    void setDefaultValues()
    {
        vehicleUpdateMode = VehicleUpdateMode::eVelocityChange;
        upAxis = { 0.0f, 1.0f, 0.0f };
        forwardAxis = { 0.0f, 0.0f, 1.0f };
        verticalAxis = ePosY;
        longitudinalAxis = ePosZ;
    }

    ObjectKey sceneKey;
    VehicleUpdateMode vehicleUpdateMode = VehicleUpdateMode::eVelocityChange;
    carb::Float3 upAxis = { 0.0f, 1.0f, 0.0f };
    carb::Float3 forwardAxis = { 0.0f, 0.0f, 1.0f };
    AxisDir verticalAxis = ePosY;
    AxisDir longitudinalAxis = ePosZ;
};

struct VehicleDesc : PhysxObjectDesc
{
    enum QueryType { eRAYCAST, eSWEEP };

    VehicleDesc() { type = eVehicle; }

    ObjectId bodyId = kInvalidObjectId;

    std::vector<WheelAttachmentDesc> wheelAttachments;
    std::vector<WheelControllerDesc> wheelControllers;
    std::vector<const BrakesDesc*> brakes;
    SteeringDesc* steering = nullptr;
    DriveDesc* drive = nullptr;
    MultiWheelDifferentialDesc* differential = nullptr;

    carb::Float3 scale = { 1.0f, 1.0f, 1.0f };

    float subStepThresholdLongitudinalSpeed = 0.0f;
    int lowForwardSpeedSubStepCount = 0;
    int highForwardSpeedSubStepCount = 0;

    float minLongitudinalSlipDenominator = 0.0f;
    float minPassiveLongitudinalSlipDenominator = 0.0f;
    float minActiveLongitudinalSlipDenominator = 0.0f;
    float minLateralSlipDenominator = 0.0f;

    float longitudinalStickyTireThresholdSpeed = 0.0f;
    float longitudinalStickyTireThresholdTime = 0.0f;
    float longitudinalStickyTireDamping = 0.0f;
    float lateralStickyTireThresholdSpeed = 0.0f;
    float lateralStickyTireThresholdTime = 0.0f;
    float lateralStickyTireDamping = 0.0f;

    bool enabled = true;
    uint8_t queryType = 0;

    bool hasUserDefinedSprungMassValues = false;
    bool hasUserDefinedMaxDroopValues = false;
    bool hasUserDefinedRestLoadValues = false;
    bool isUsingDeprecatedLatStiffY = false;
    bool referenceFrameIsCenterOfMass = false;
    bool limitSuspensionExpansionVelocity = false;

    static constexpr uint32_t maxNumberOfWheels = 20;
};

struct VehicleControllerDesc : PhysxObjectDesc
{
    VehicleControllerDesc() { type = eVehicleControllerStandard; }
    float accelerator = 0.0f;
    float brake0 = 0.0f;
    float brake1 = 0.0f;
    float brake = 0.0f;
    float handbrake = 0.0f;
    float steer = 0.0f;
    float steerLeft = 0.0f;
    float steerRight = 0.0f;
    int targetGear = 0;
    static constexpr int automaticGearValue = 0xff;
};

struct VehicleTankControllerDesc : VehicleControllerDesc
{
    VehicleTankControllerDesc() { type = eVehicleControllerTank; }
    float thrust0 = 0.0f;
    float thrust1 = 0.0f;
};

// ---------------------------------------------------------------------------
// Particles
// ---------------------------------------------------------------------------

struct ParticleSystemDesc : PhysxObjectDesc
{
    ParticleSystemDesc() { type = eParticleSystem; }

    bool enableParticleSystem = true;
    bool enableCCD = false;
    float restOffset = 0.0f;
    float contactOffset = 0.0f;
    float particleContactOffset = 0.0f;
    float solidRestOffset = 0.0f;
    float fluidRestOffset = 0.0f;
    float maxDepenetrationVelocity = 0.0f;
    float maxVelocity = 0.0f;
    float fluidBoundaryDensityScale = 0.0f;

    bool enableSmoothing = false;
    bool enableAnisotropy = false;
    bool enableIsosurface = false;
    int solverPositionIterations = 0;
    carb::Float3 wind = { 0.0f, 0.0f, 0.0f };
    int maxNeighborhood = 0;
    float neighborhoodScale = 0.0f;
    int lockedAxis = 0;

    ObjectId material = kInvalidObjectId;
    ObjectId collisionGroup = kInvalidObjectId;
    std::vector<ObjectKey> filteredCollisions;

    ObjectKey sceneKey;
    ObjectKey systemKey;
};

struct ParticleDesc : PhysxObjectDesc
{
    ParticleDesc() { type = eUndefined; }

    int numParticles = 0;
    bool enabled = true;
    bool selfCollision = false;
    int particleGroup = 0;
    std::vector<carb::Float3> points;
    std::vector<carb::Float3> velocities;

    ObjectKey primKey;
    ObjectKey particleSystemKey;
    ObjectKey sceneKey;

    float mass = 0.0f;
    float density = 0.0f;
};

struct ParticleSetDesc : ParticleDesc
{
    ParticleSetDesc()
    {
        type = eParticleSet;
        mass = -1.0f;
        density = -1.0f;
    }

    bool fluid = false;
    float solidRestOffset = 0.0f;
    float fluidRestOffset = 0.0f;

    bool enableDiffuseParticles = false;
    float maxDiffuseParticleMultiplier = 0.0f;
    float diffuseParticlesThreshold = 0.0f;
    float diffuseParticlesLifetime = 0.0f;
    float diffuseParticlesAirDrag = 0.0f;
    float diffuseParticlesBubbleDrag = 0.0f;
    float diffuseParticlesBuoyancy = 0.0f;
    float diffuseParticlesKineticEnergyWeight = 0.0f;
    float diffuseParticlesPressureWeight = 0.0f;
    float diffuseParticlesDivergenceWeight = 0.0f;
    float diffuseParticlesCollisionDecay = 0.0f;

    std::vector<carb::Float3> simulationPoints;
    int maxParticles = 0;
};

struct ParticleSamplingDesc
{
    float samplingDistance = 0.0f;
    bool sampleVolume = false;
    ObjectKey particleSetKey;
    int maxSamples = 0;
    float pointWidth = 0.0f;
};

struct ParticleIsosurfaceDesc : PhysxObjectDesc
{
    struct GridFilteringPass
    {
        enum Enum { eSmooth, eGrow, eReduce, eNone };
    };

    bool enableIsosurface = false;
    int maxIsosurfaceVertices = 0;
    int maxIsosurfaceTriangles = 0;
    int maxNumIsosurfaceSubgrids = 0;
    float gridSpacing = 0.0f;
    float surfaceDistance = 0.0f;
    std::vector<GridFilteringPass::Enum> gridFilteringPasses;
    float gridSmoothingRadius = 0.0f;
    int numMeshSmoothingPasses = 0;
    int numMeshNormalSmoothingPasses = 0;
    ObjectKey systemKey;
};

struct ParticleSmoothingDesc : PhysxObjectDesc
{
    bool enableSmoothing = false;
    float strength = 0.0f;
    ObjectKey systemKey;
};

struct ParticleAnisotropyDesc : PhysxObjectDesc
{
    bool enableAnisotropy = false;
    float scale = 0.0f;
    float min = 0.0f;
    float max = 0.0f;
    ObjectKey systemKey;
};

// ---------------------------------------------------------------------------
// Deformable attachments and collision filters
// ---------------------------------------------------------------------------

struct PhysxDeformableAttachmentDesc : PhysxObjectDesc
{
    PhysxDeformableAttachmentDesc() = default;

    // Attachment prim's own source-side identity.
    ObjectKey primKey;

    bool enabled = true;
    ObjectKey src0;
    ObjectKey src1;
    float stiffness = 0.0f;
    float damping = 0.0f;
};

struct PhysxDeformableCollisionFilterDesc : PhysxObjectDesc
{
    PhysxDeformableCollisionFilterDesc() : enabled(true) { type = eDeformableCollisionFilter; }

    // Collision-filter prim's own source-side identity.
    ObjectKey primKey;

    bool enabled;
    ObjectKey src0;
    ObjectKey src1;
};

// ---------------------------------------------------------------------------
// Character controller
// ---------------------------------------------------------------------------

struct CctDesc : PhysxObjectDesc
{
    CctDesc() = default;

    // CCT object's own source-side identity — set by the walker so the
    // engine integration can resolve `primKey` back to its source via
    // `ScannedStage::pathFor`.
    ObjectKey primKey;

    // Source-side `physics:simulationOwner` target. Singular: at most
    // one target is allowed (multi-target inputs log an error and use
    // the first). The consumer resolves this to `sceneId` post-scan,
    // matching the rigid-body pattern.
    ObjectKey sourceSimulationOwner;

    float slopeLimit = 0.0f;
    carb::Float3 pos = { 0.0f, 0.0f, 0.0f };
    carb::Float3 scale = { 1.0f, 1.0f, 1.0f };
    ObjectId sceneId = kInvalidObjectId;
};

struct CapsuleCctDesc : CctDesc
{
    CapsuleCctDesc(float inRadius = 0.1f, float half_height = 0.1f) : CctDesc(), height(half_height), radius(inRadius)
    { type = eCapsuleCct; }
    float height;
    float radius;
};

} // namespace omni::physics::parse
