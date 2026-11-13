// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-CACHE-RUNTIME-001
 * @covers AC-10 AC-9 AC-8 AC-7 AC-6 AC-1 AC-2 AC-3 AC-4 AC-5 AC-14
 */
#pragma once
#include <omni/physics/parse/CacheArchive.h>
#include <array>
#include <string>
#include <utility>
#include <vector>

namespace omni::physx::cache
{
using SerialId = uint64_t;
// Archive-local, 1-based index into paths. Zero denotes an absent optional path.
using PathId = uint32_t;
// These are wire records, not binary images of InternalActor/InternalJoint.
// IDs name objects in the accompanying PhysX collection. A loader must mint
// fresh runtime IDs/keys/owner epochs and fix up userData after decoding.
struct ObjectIdentity
{
    SerialId id = 0;
    PathId path = 0;
};
struct ActorRecord
{
    ObjectIdentity object;
    bool articulationLink = false;
    bool inboundJointHasState = false;
    std::array<float, 3> scale{ 1, 1, 1 };
    uint32_t flags = 0;
    PathId sourceGeometryPath = 0;
    std::array<float, 3> initialVelocity{};
    std::array<float, 3> initialAngularVelocity{};
    bool velocityWritten = false;
    bool angularVelocityWritten = false;
    bool surfaceVelocityEnabled = false, surfaceVelocityLocalSpace = true, contactSolveEnabled = true;
    std::array<float, 3> surfaceVelocity{}, surfaceVelocityAuthored{}, surfaceAngularVelocity{}, surfacePivotPosition{};
    std::array<float, 4> surfacePivotRotation{ 0, 0, 0, 1 };
    // PxRigidDynamicLockFlags. PhysX binary deserialization resets them; hydration restores them.
    uint32_t lockFlags = 0;
    // How the ordinary load placed a dynamic body: it created the body at the insertion pose
    // and added it to the scene with the center of mass at the actor origin, then its mass
    // update passed the applied center of mass to PhysX. Hydration replays both through
    // PhysX's setters, which normalize as creation does; the first simulation step reads the
    // result. Capture archives them only for a body that still holds the center of mass
    // PhysX stored for the applied one.
    bool hasInsertionPose = false;
    std::array<float, 3> insertionPosition{};
    std::array<float, 4> insertionRotation{ 0, 0, 0, 1 };
    std::array<float, 3> appliedCenterOfMassPosition{};
    std::array<float, 4> appliedCenterOfMassRotation{ 0, 0, 0, 1 };
};
struct MaterialRecord
{
    ObjectIdentity object;
    float density = 0;
    // PxMaterialFlags. PhysX binary deserialization resets them; hydration restores them.
    uint32_t flags = 0;
};
struct ShapeRecord
{
    ObjectIdentity object;
    PathId sourceGeometryPath = 0; // geometry used for material binding, possibly below the collider
    SerialId material = 0; // runtime density binding; zero means implicit
    std::vector<SerialId> materialSlots; // native table; zero selects the scene default
    std::vector<SerialId> compoundShapes; // empty for a single shape; first ID equals object.id
    std::array<float, 3> scale{ 1, 1, 1 };
    uint32_t axis = 0, instanceIndex = UINT32_MAX;
    float volume = 0;
    std::array<float, 9> inertia{};
    std::array<float, 3> centerOfMass{}, localPosition{}, aabbMin{}, aabbMax{}, geometryPosition{};
    std::array<float, 4> localRotation{ 0, 0, 0, 1 }, geometryRotation{ 0, 0, 0, 1 };
};
struct ArticulationRecord
{
    ObjectIdentity object;
    bool selfCollision = true;
    PathId staticRootBodyPath = 0;
    PathId fixedBaseJointPath = 0;
};
struct DriveRecord
{
    bool enabled = false;
    bool acceleration = false;
    float stiffness = 0, damping = 0, maxForce = 0, targetPosition = 0, targetVelocity = 0;
    bool envelopeUsed = false;
    float maxActuatorVelocity = 0, velocityDependentResistance = 0, speedEffortGradient = 0;
};
struct JointStateRecord
{
    std::string token;
    bool enabled = false, convertToDegrees = false;
    uint32_t physxAxis = 0;
    float initialPosition = 0, initialVelocity = 0;
};
enum class JointKind : uint32_t
{
    eFixed,
    eRevolute,
    ePrismatic,
    eSpherical,
    eDistance,
    eD6,
    eGear,
    eRackAndPinion
};
struct JointRecord
{
    ObjectIdentity object;
    JointKind kind = JointKind::eFixed;
    bool articulationJoint = false, body0IsParent = true;
    uint32_t axis = 0, axisIndex = 0, d6Drive = 0;
    DriveRecord drive;
    std::array<DriveRecord, 6> drives;
    std::array<JointStateRecord, 6> states;
};
struct JointInstance
{
    ObjectIdentity object;
    uint32_t settings = 0;
};
enum class TendonKind : uint32_t
{
    eFixedAxis,
    eSpatialAttachment
};
struct TendonRecord
{
    ObjectIdentity object;
    TendonKind kind = TendonKind::eFixedAxis;
    std::string instanceName;
    bool root = false, leaf = false;
    std::array<float, 3> globalPosition{};
    float initialLength = 0;
};
struct MimicRecord
{
    ObjectIdentity object;
    uint32_t axis = 0; // runtime mimic category rotX, rotY or rotZ
    float gearingSign = 1, offsetSign = 1;
};
struct InstancedActorRecord
{
    SerialId actor = 0;
    PathId instancer = 0;
    uint32_t index = 0;
    std::array<double, 16> prototypeInverse{};
};
struct PointInstancerRecord
{
    PathId path = 0;
    bool initialArrays = false;
    std::vector<std::array<float, 3>> positions, scales, velocities, angularVelocities;
    std::vector<std::array<float, 4>> orientations;
};
struct ForceRecord
{
    PathId path = 0;
    SerialId actor = 0;
    bool worldFrame = true, accelerationMode = false, enabled = true, centerOfMass = true, bodyPrimDifferent = false;
    std::array<float, 3> force{}, torque{}, localPosition{};
    std::array<float, 4> localRotation{ 0, 0, 0, 1 };
};
struct CollisionGroupRecord
{
    PathId path = 0;
};
struct CollisionMembershipRecord
{
    PathId member = 0, group = 0;
};
struct CollisionGroupPairRecord
{
    PathId first = 0, second = 0;
};
struct ShapeFilterRecord
{
    SerialId shape = 0;
    PathId simulationGroup = 0, queryGroup = 0;
};
struct FilteredPairRecord
{
    PathId path = 0;
    std::vector<std::pair<SerialId, SerialId>> pairs;
};
struct ContactReportRecord
{
    SerialId actor = 0, target = 0; // zero target means wildcard unless pendingPath is set
    PathId pendingPath = 0;
    float threshold = 0;
};
struct SchemaRecord
{
    PathId path = 0;
    uint64_t flags = 0;
};
struct ObjectBindingRecord
{
    PathId path = 0;
    SerialId id = 0;
    uint32_t category = 0;
    bool fixedBaseAlias = false;
};
// The USD face of each input triangle of a single shape's triangle mesh cooked
// from original triangles: the runtime's side table, which PhysX does not
// serialize. PxTriangleMesh::getTrianglesRemap() indexes it; it ends at the
// largest input triangle the remap names.
struct FaceMapRecord
{
    SerialId shape = 0;
    std::vector<uint32_t> faces;
};
struct RuntimeArchive
{
    std::vector<std::string> paths; // unique strings; never ovstage handles or ObjectKeys
    // The scene is parsed from the stage on restore, which the identity covers; its
    // settings and default materials are not archived.
    PathId scenePath = 0;
    std::vector<PathId> physicsPaths; // complete initial change-feed membership
    std::vector<ActorRecord> actors;
    std::vector<MaterialRecord> materials;
    std::vector<ShapeRecord> shapes;
    std::vector<ArticulationRecord> articulations;
    std::vector<JointRecord> joints;
    // Optional compact decoded view: shared joint settings plus per-object instances.
    // An archive uses either expanded joints or templates/instances, never both.
    std::vector<JointRecord> jointTemplates;
    std::vector<JointInstance> jointInstances;
    size_t jointCount() const
    {
        return joints.size() + jointInstances.size();
    }
    template <typename Fn>
    bool forEachJoint(Fn&& fn) const
    {
        for (const auto& joint : joints)
            if (!fn(joint.object, joint))
                return false;
        for (const auto& joint : jointInstances)
            if (joint.settings >= jointTemplates.size() || !fn(joint.object, jointTemplates[joint.settings]))
                return false;
        return true;
    }
    std::vector<TendonRecord> tendons;
    std::vector<MimicRecord> mimics;
    std::vector<ForceRecord> forces;
    std::vector<InstancedActorRecord> instancedActors;
    std::vector<PointInstancerRecord> pointInstancers;
    std::vector<CollisionGroupRecord> collisionGroups;
    std::vector<CollisionMembershipRecord> collisionMemberships;
    std::vector<CollisionGroupPairRecord> collisionGroupPairs;
    std::vector<ShapeFilterRecord> shapeFilters;
    std::vector<FilteredPairRecord> filteredPairs;
    std::vector<ContactReportRecord> contactReports;
    std::vector<SchemaRecord> schemas;
    std::vector<ObjectBindingRecord> bindings;
    std::vector<PathId> animatedKinematics;
    std::vector<FaceMapRecord> faceMaps;
    const std::string& path(PathId id) const
    {
        return paths.at(size_t(id) - 1);
    }
};
// Both directions reject duplicate IDs/paths, invalid material references,
// unknown enum values, and invalid paths. Output is unchanged on failure.
bool encodeRuntimeArchive(const RuntimeArchive& archive, omni::physics::parse::cache::Bytes& output);
bool decodeRuntimeArchive(const omni::physics::parse::cache::Bytes& bytes,
                          RuntimeArchive& output,
                          bool compactJoints = false);
// Input bytes are borrowed only during this call; output owns all metadata.
bool decodeRuntimeArchive(const uint8_t* bytes, size_t size, RuntimeArchive& output, bool compactJoints = false);
} // namespace omni::physx::cache
