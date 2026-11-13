// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-CACHE-RUNTIME-001
 * @covers AC-13 AC-12 AC-11 AC-10 AC-9 AC-8 AC-7 AC-6 AC-1 AC-2 AC-3 AC-4 AC-5
 */
#include <private/omni/physx/RuntimeArchive.h>
#include <omni/physics/parse/Descriptors.h>
#include "../utils/PrimPathGrammar.h"
#include <set>
#include <algorithm>
#include <cstring>
#include <iterator>
#include <memory_resource>
#include <string_view>
#include <unordered_set>
#include <unordered_map>

namespace omni::physx::cache
{
namespace
{
using omni::physics::parse::cache::Bytes;
using omni::physics::parse::cache::Reader;
using omni::physics::parse::cache::Writer;
constexpr uint32_t kRuntimeArchiveVersion = 16;
constexpr uint64_t kMaxJointRecords = 512 * 1024 * 1024 / sizeof(JointRecord);
// Captured IDs are dense and index a flag array; other IDs use a hash set.
class ObjectIds
{
public:
    explicit ObjectIds(size_t count) : dense(count + 1, 0)
    {
    }
    bool insert(SerialId id)
    {
        if (id >= dense.size())
            return sparse.insert(id).second;
        const bool fresh = !dense[size_t(id)];
        dense[size_t(id)] = 1;
        return fresh;
    }
    bool count(SerialId id) const
    {
        return id < dense.size() ? dense[size_t(id)] != 0 : sparse.count(id) != 0;
    }
    size_t denseSize() const
    {
        return dense.size();
    }

private:
    std::vector<uint8_t> dense;
    std::unordered_set<SerialId> sparse;
};
template <typename T>
bool identities(const std::vector<T>& records,
                ObjectIds& ids,
                size_t pathCount,
                const std::unordered_set<SerialId>& instances = {},
                bool repeatedPaths = false)
{
    std::vector<bool> paths(pathCount + 1);
    for (const T& r : records)
        if (!r.object.id || (!r.object.path || r.object.path > pathCount) || !ids.insert(r.object.id) ||
            (!repeatedPaths && !instances.count(r.object.id) && paths[r.object.path]))
            return false;
        else if (!instances.count(r.object.id))
            paths[r.object.path] = true;
    return true;
}
// An absolute prim path, an insertion unit's local name (see kUnitRoot), or a
// native-instancing prototype path under its stable name
// `/__Prototype(<archived path>)/...` (see canonicalPrototypePaths).
bool archivedPathValid(std::string_view path, int depth = 0)
{
    using omni::physics::parse::cache::kUnitRoot;
    if (depth == 0 && path.compare(0, kUnitRoot.size(), kUnitRoot) == 0)
        return path.size() == kUnitRoot.size() || looksLikeAbsolutePrimPath(path.substr(kUnitRoot.size()), false);
    constexpr std::string_view prefix = "/__Prototype(";
    if (path.compare(0, prefix.size(), prefix) != 0)
        return looksLikeAbsolutePrimPath(path, true);
    if (depth > omni::physics::parse::cache::kMaxPrototypeNesting)
        return false;
    size_t end = prefix.size();
    int open = 1;
    for (; end < path.size() && open; ++end)
        open += path[end] == '(' ? 1 : path[end] == ')' ? -1 : 0;
    if (open || !archivedPathValid(path.substr(prefix.size(), end - 1 - prefix.size()), depth + 1))
        return false;
    const std::string_view rest = path.substr(end);
    return rest.empty() || looksLikeAbsolutePrimPath(rest, true);
}
bool valid(const RuntimeArchive& a)
{
    if (a.paths.size() > UINT32_MAX || (!a.joints.empty() && (!a.jointTemplates.empty() || !a.jointInstances.empty())))
        return false;
    // Validation nodes never escape this call. Allocate them in blocks and
    // release the blocks together instead of visiting the heap for each path.
    std::pmr::monotonic_buffer_resource pathStorage;
    std::pmr::unordered_set<std::string_view> uniquePaths(&pathStorage);
    uniquePaths.reserve(a.paths.size());
    for (const auto& path : a.paths)
        if (!archivedPathValid(path) || !uniquePaths.insert(path).second)
            return false;
    const auto pathValid = [&](PathId id) { return id && id <= a.paths.size(); };
    if (a.scenePath && !pathValid(a.scenePath))
        return false;
    ObjectIds ids(a.actors.size() + a.materials.size() + a.shapes.size() + a.articulations.size() + a.jointCount() +
                  a.tendons.size() + a.mimics.size());
    std::unordered_set<SerialId> materials;
    materials.reserve(a.materials.size());
    std::vector<bool> physicsPaths(a.paths.size() + 1);
    for (PathId path : a.physicsPaths)
        if (!pathValid(path) || physicsPaths[path])
            return false;
        else
            physicsPaths[path] = true;
    std::unordered_set<SerialId> instanceActors, instanceShapes;
    std::set<std::tuple<PathId, uint32_t, PathId>> instanceIdentities;
    std::unordered_set<PathId> instancers;
    for (const auto& p : a.pointInstancers)
        if (!pathValid(p.path) || !instancers.insert(p.path).second)
            return false;
    std::unordered_map<SerialId, PathId> actorPaths;
    if (!a.instancedActors.empty())
        for (const auto& actor : a.actors)
            actorPaths.emplace(actor.object.id, actor.object.path);
    for (const auto& i : a.instancedActors)
    {
        if (!instancers.count(i.instancer) || i.index == UINT32_MAX || !instanceActors.insert(i.actor).second)
            return false;
        const auto actor = actorPaths.find(i.actor);
        if (actor == actorPaths.end() || !instanceIdentities.emplace(i.instancer, i.index, actor->second).second)
            return false;
    }
    for (const auto& shape : a.shapes)
        if (shape.instanceIndex != UINT32_MAX)
        {
            if (instancers.empty())
                return false;
            instanceShapes.insert(shape.object.id);
        }
    if (!identities(a.actors, ids, a.paths.size(), instanceActors) || !identities(a.materials, ids, a.paths.size()) ||
        !identities(a.shapes, ids, a.paths.size(), instanceShapes) ||
        !identities(a.articulations, ids, a.paths.size()) || !identities(a.joints, ids, a.paths.size(), {}, true) ||
        !identities(a.jointInstances, ids, a.paths.size(), {}, true))
        return false;
    // Multiple applied tendon/mimic instances can share a prim path.
    std::set<std::tuple<PathId, uint32_t, std::string, bool>> tendonNames;
    for (const TendonRecord& t : a.tendons)
        if (!t.object.id || !ids.insert(t.object.id) || !pathValid(t.object.path) || uint32_t(t.kind) > 1 ||
            t.instanceName.empty() || (t.root && t.leaf) || (t.kind == TendonKind::eFixedAxis && t.leaf) ||
            !tendonNames.emplace(t.object.path, uint32_t(t.kind), t.instanceName, t.root).second)
            return false;
    // Mimic axes are offsets from eMimicJointRotX in type units: rotX..rotZ, or the
    // single-apply Newton mimic.
    constexpr uint32_t kNewtonMimicAxis =
        (uint32_t(omni::physics::parse::eNewtonMimicJoint) - uint32_t(omni::physics::parse::eMimicJointRotX)) >> 16;
    std::set<std::pair<PathId, uint32_t>> mimicAxes;
    for (const MimicRecord& m : a.mimics)
        if (!m.object.id || !ids.insert(m.object.id) || !pathValid(m.object.path) ||
            (m.axis > 2 && m.axis != kNewtonMimicAxis) || (m.gearingSign != 1 && m.gearingSign != -1) ||
            (m.offsetSign != 1 && m.offsetSign != -1) || !mimicAxes.emplace(m.object.path, m.axis).second)
            return false;
    for (const MaterialRecord& m : a.materials)
        materials.insert(m.object.id);
    std::unordered_set<SerialId> compoundIds;
    for (const ShapeRecord& s : a.shapes)
    {
        if (s.sourceGeometryPath && !pathValid(s.sourceGeometryPath))
            return false;
        if ((s.material && !materials.count(s.material)) || s.axis > 2 || s.materialSlots.size() > 65535)
            return false;
        for (SerialId slot : s.materialSlots)
            if (slot && !materials.count(slot))
                return false;
        if (!s.compoundShapes.empty() && s.compoundShapes.front() != s.object.id)
            return false;
        for (SerialId id : s.compoundShapes)
            if (!id || (id != s.object.id && ids.count(id)) || !compoundIds.insert(id).second)
                return false;
    }
    for (const ActorRecord& r : a.actors)
        if ((r.sourceGeometryPath && !pathValid(r.sourceGeometryPath)) || r.lockFlags > 0x3f)
            return false;
    for (const MaterialRecord& m : a.materials)
        if (m.flags > 0xffff)
            return false;
    for (const ArticulationRecord& r : a.articulations)
        if ((r.staticRootBodyPath && !pathValid(r.staticRootBodyPath)) ||
            (r.fixedBaseJointPath && !pathValid(r.fixedBaseJointPath)))
            return false;
    auto validJoint = [](const JointRecord& j) {
        // A joint state names its PhysicsJointStateAPI instance, or none.
        static constexpr std::string_view kStates[] = { "",     "angular", "linear", "rotX",  "rotY",
                                                        "rotZ", "transX",  "transY", "transZ" };
        if (uint32_t(j.kind) > uint32_t(JointKind::eRackAndPinion) || j.axis > 2 || j.axisIndex > 5 || j.d6Drive > 5)
            return false;
        for (const JointStateRecord& state : j.states)
            if (state.physxAxis > 5 ||
                std::find(std::begin(kStates), std::end(kStates), state.token) == std::end(kStates))
                return false;
        return true;
    };
    for (const auto& joint : a.joints)
        if (!validJoint(joint))
            return false;
    for (const auto& joint : a.jointTemplates)
        if (!validJoint(joint))
            return false;
    for (const auto& joint : a.jointInstances)
        if (joint.settings >= a.jointTemplates.size())
            return false;
    std::vector<bool> schemaPaths(a.paths.size() + 1);
    for (const SchemaRecord& s : a.schemas)
        if (!pathValid(s.path) || schemaPaths[s.path])
            return false;
        else
            schemaPaths[s.path] = true;
    // An articulation whose root API sits on its fixed root joint binds that prim
    // twice in one category: to the articulation and to its fixed-base alias.
    struct BindingKey
    {
        PathId path;
        uint32_t category;
        SerialId id;
        bool fixedBaseAlias;
        bool operator==(const BindingKey& b) const
        {
            return path == b.path && category == b.category && id == b.id && fixedBaseAlias == b.fixedBaseAlias;
        }
    };
    struct BindingHash
    {
        size_t operator()(const BindingKey& b) const
        {
            // Path and object IDs often have the same numeric value. Plain XOR
            // cancels those fields and makes validation quadratic for large stages.
            size_t value = std::hash<PathId>{}(b.path);
            value ^= std::hash<SerialId>{}(b.id) + size_t(0x9e3779b9u) + (value << 6) + (value >> 2);
            return value ^ (size_t(b.category) * size_t(0x9e3779b9u)) ^ size_t(b.fixedBaseAlias);
        }
    };
    std::vector<const ObjectBindingRecord*> firstBindings(ids.denseSize(), nullptr);
    std::unordered_set<BindingKey, BindingHash> aliases;
    for (const ObjectBindingRecord& b : a.bindings)
    {
        if (!pathValid(b.path) || !ids.count(b.id))
            return false;
        if (b.id < firstBindings.size())
        {
            auto*& first = firstBindings[size_t(b.id)];
            if (!first)
            {
                first = &b;
                continue;
            }
            if (first->path == b.path && first->category == b.category && first->fixedBaseAlias == b.fixedBaseAlias)
                return false;
        }
        // Multiple bindings for an object and sparse IDs retain exact duplicate
        // validation; the common one-binding-per-dense-ID path needs no hash node.
        if (!aliases.insert({ b.path, b.category, b.id, b.fixedBaseAlias }).second)
            return false;
    }
    std::unordered_set<PathId> groups, filters;
    for (const auto& group : a.collisionGroups)
        if (!pathValid(group.path) || !groups.insert(group.path).second)
            return false;
    for (const auto& member : a.collisionMemberships)
        if (!pathValid(member.member) || !groups.count(member.group))
            return false;
    for (const auto& pair : a.collisionGroupPairs)
        if (!groups.count(pair.first) || !groups.count(pair.second))
            return false;
    std::unordered_set<SerialId> shapeIds, filteredShapes;
    if (!a.shapeFilters.empty())
        for (const auto& shape : a.shapes)
        {
            shapeIds.insert(shape.object.id);
            shapeIds.insert(shape.compoundShapes.begin(), shape.compoundShapes.end());
        }
    for (const auto& filter : a.shapeFilters)
        if (!shapeIds.count(filter.shape) || !filteredShapes.insert(filter.shape).second ||
            (filter.simulationGroup && !groups.count(filter.simulationGroup)) ||
            (filter.queryGroup && !groups.count(filter.queryGroup)))
            return false;
    for (const auto& filter : a.filteredPairs)
    {
        if (!pathValid(filter.path) || !filters.insert(filter.path).second)
            return false;
        for (const auto& pair : filter.pairs)
            if (!ids.count(pair.first) || !ids.count(pair.second))
                return false;
    }
    std::unordered_set<SerialId> actors;
    if (!a.forces.empty() || !a.contactReports.empty())
        for (const auto& actor : a.actors)
            actors.insert(actor.object.id);
    std::unordered_set<PathId> forcePaths;
    for (const auto& force : a.forces)
        if (!actors.count(force.actor) || !pathValid(force.path) || !forcePaths.insert(force.path).second)
            return false;
    for (const auto& report : a.contactReports)
        if (!actors.count(report.actor) || (report.target && !actors.count(report.target)) ||
            (report.pendingPath && (!pathValid(report.pendingPath) || report.target)))
            return false;
    for (PathId path : a.animatedKinematics)
        if (!pathValid(path))
            return false;
    // Hydration checks that the shape has a triangle mesh the map covers.
    if (!a.faceMaps.empty())
    {
        std::unordered_set<SerialId> shapes, mapped;
        for (const ShapeRecord& s : a.shapes)
            shapes.insert(s.object.id);
        for (const FaceMapRecord& m : a.faceMaps)
            if (m.faces.empty() || !shapes.count(m.shape) || !mapped.insert(m.shape).second)
                return false;
    }
    return true;
}
template <size_t N>
void floats(Writer& w, const std::array<float, N>& v)
{
    for (float f : v)
        w.f32(f);
}
template <size_t N>
void floats(Reader& r, std::array<float, N>& v)
{
    for (float& f : v)
        f = r.f32();
}
// Every codec below binds all members of its record by name, in declaration
// order: a member added to a record does not compile until both directions name
// it. Naming a member does not write or read it; the round-trip test sets every
// member and compares every decoded member with its source.
void identity(Writer& w, const ObjectIdentity& o)
{
    const auto& [id, path] = o;
    w.u64(id);
    w.u32(path);
}
void identity(Reader& r, ObjectIdentity& o)
{
    auto& [id, path] = o;
    id = r.u64();
    path = r.u32();
}
bool boolean(Reader& r, bool& value)
{
    const uint8_t byte = r.u8();
    value = byte != 0;
    return byte <= 1 && r.good();
}
void drive(Writer& w, const DriveRecord& d)
{
    const auto& [enabled, acceleration, stiffness, damping, maxForce, targetPosition, targetVelocity, envelopeUsed,
                 maxActuatorVelocity, velocityDependentResistance, speedEffortGradient] = d;
    w.u8(enabled);
    w.u8(acceleration);
    w.f32(stiffness);
    w.f32(damping);
    w.f32(maxForce);
    w.f32(targetPosition);
    w.f32(targetVelocity);
    w.u8(envelopeUsed);
    w.f32(maxActuatorVelocity);
    w.f32(velocityDependentResistance);
    w.f32(speedEffortGradient);
}
bool drive(Reader& r, DriveRecord& d)
{
    auto& [enabled, acceleration, stiffness, damping, maxForce, targetPosition, targetVelocity, envelopeUsed,
           maxActuatorVelocity, velocityDependentResistance, speedEffortGradient] = d;
    if (!boolean(r, enabled) || !boolean(r, acceleration))
        return false;
    stiffness = r.f32();
    damping = r.f32();
    maxForce = r.f32();
    targetPosition = r.f32();
    targetVelocity = r.f32();
    if (!boolean(r, envelopeUsed))
        return false;
    maxActuatorVelocity = r.f32();
    velocityDependentResistance = r.f32();
    speedEffortGradient = r.f32();
    return r.good();
}
void record(Writer& w, const ActorRecord& a)
{
    const auto& [object, articulationLink, inboundJointHasState, scale, flags, sourceGeometryPath, initialVelocity,
                 initialAngularVelocity, velocityWritten, angularVelocityWritten, surfaceVelocityEnabled,
                 surfaceVelocityLocalSpace, contactSolveEnabled, surfaceVelocity, surfaceVelocityAuthored,
                 surfaceAngularVelocity, surfacePivotPosition, surfacePivotRotation, lockFlags, hasInsertionPose,
                 insertionPosition, insertionRotation, appliedCenterOfMassPosition, appliedCenterOfMassRotation] = a;
    identity(w, object);
    w.u8(articulationLink);
    w.u8(inboundJointHasState);
    floats(w, scale);
    w.u32(flags);
    w.u32(sourceGeometryPath);
    floats(w, initialVelocity);
    floats(w, initialAngularVelocity);
    w.u8(velocityWritten);
    w.u8(angularVelocityWritten);
    w.u8(surfaceVelocityEnabled);
    w.u8(surfaceVelocityLocalSpace);
    w.u8(contactSolveEnabled);
    floats(w, surfaceVelocity);
    floats(w, surfaceVelocityAuthored);
    floats(w, surfaceAngularVelocity);
    floats(w, surfacePivotPosition);
    floats(w, surfacePivotRotation);
    w.u32(lockFlags);
    w.u8(hasInsertionPose);
    floats(w, insertionPosition);
    floats(w, insertionRotation);
    floats(w, appliedCenterOfMassPosition);
    floats(w, appliedCenterOfMassRotation);
}
bool record(Reader& r, ActorRecord& a)
{
    auto& [object, articulationLink, inboundJointHasState, scale, flags, sourceGeometryPath, initialVelocity,
           initialAngularVelocity, velocityWritten, angularVelocityWritten, surfaceVelocityEnabled,
           surfaceVelocityLocalSpace, contactSolveEnabled, surfaceVelocity, surfaceVelocityAuthored,
           surfaceAngularVelocity, surfacePivotPosition, surfacePivotRotation, lockFlags, hasInsertionPose,
           insertionPosition, insertionRotation, appliedCenterOfMassPosition, appliedCenterOfMassRotation] = a;
    identity(r, object);
    if (!boolean(r, articulationLink) || !boolean(r, inboundJointHasState))
        return false;
    floats(r, scale);
    flags = r.u32();
    sourceGeometryPath = r.u32();
    floats(r, initialVelocity);
    floats(r, initialAngularVelocity);
    if (!boolean(r, velocityWritten) || !boolean(r, angularVelocityWritten) || !boolean(r, surfaceVelocityEnabled) ||
        !boolean(r, surfaceVelocityLocalSpace) || !boolean(r, contactSolveEnabled))
        return false;
    floats(r, surfaceVelocity);
    floats(r, surfaceVelocityAuthored);
    floats(r, surfaceAngularVelocity);
    floats(r, surfacePivotPosition);
    floats(r, surfacePivotRotation);
    lockFlags = r.u32();
    if (!boolean(r, hasInsertionPose))
        return false;
    floats(r, insertionPosition);
    floats(r, insertionRotation);
    floats(r, appliedCenterOfMassPosition);
    floats(r, appliedCenterOfMassRotation);
    return r.good();
}
template <size_t N>
void vectors(Writer& w, const std::vector<std::array<float, N>>& values)
{
    w.u64(values.size());
    for (const auto& v : values)
        floats(w, v);
}
template <size_t N>
bool vectors(Reader& r, std::vector<std::array<float, N>>& values)
{
    const auto count = r.u64();
    if (!r.good() || count > r.remaining() / (N * 4))
        return false;
    values.resize(count);
    for (auto& v : values)
        floats(r, v);
    return r.good();
}
void record(Writer& w, const InstancedActorRecord& a)
{
    const auto& [actor, instancer, index, prototypeInverse] = a;
    w.u64(actor);
    w.u32(instancer);
    w.u32(index);
    for (double v : prototypeInverse)
    {
        uint64_t bits;
        std::memcpy(&bits, &v, 8);
        w.u64(bits);
    }
}
bool record(Reader& r, InstancedActorRecord& a)
{
    auto& [actor, instancer, index, prototypeInverse] = a;
    actor = r.u64();
    instancer = r.u32();
    index = r.u32();
    for (auto& v : prototypeInverse)
    {
        const auto bits = r.u64();
        std::memcpy(&v, &bits, 8);
    }
    return r.good();
}
void record(Writer& w, const PointInstancerRecord& a)
{
    const auto& [path, initialArrays, positions, scales, velocities, angularVelocities, orientations] = a;
    w.u32(path);
    w.u8(initialArrays);
    vectors(w, positions);
    vectors(w, orientations);
    vectors(w, scales);
    vectors(w, velocities);
    vectors(w, angularVelocities);
}
bool record(Reader& r, PointInstancerRecord& a)
{
    auto& [path, initialArrays, positions, scales, velocities, angularVelocities, orientations] = a;
    path = r.u32();
    return boolean(r, initialArrays) && vectors(r, positions) && vectors(r, orientations) && vectors(r, scales) &&
           vectors(r, velocities) && vectors(r, angularVelocities);
}
void record(Writer& w, const ForceRecord& a)
{
    const auto& [path, actor, worldFrame, accelerationMode, enabled, centerOfMass, bodyPrimDifferent, force, torque,
                 localPosition, localRotation] = a;
    w.u32(path);
    w.u64(actor);
    w.u8(worldFrame);
    w.u8(accelerationMode);
    w.u8(enabled);
    w.u8(centerOfMass);
    w.u8(bodyPrimDifferent);
    floats(w, force);
    floats(w, torque);
    floats(w, localPosition);
    floats(w, localRotation);
}
bool record(Reader& r, ForceRecord& a)
{
    auto& [path, actor, worldFrame, accelerationMode, enabled, centerOfMass, bodyPrimDifferent, force, torque,
           localPosition, localRotation] = a;
    path = r.u32();
    actor = r.u64();
    if (!boolean(r, worldFrame) || !boolean(r, accelerationMode) || !boolean(r, enabled) || !boolean(r, centerOfMass) ||
        !boolean(r, bodyPrimDifferent))
        return false;
    floats(r, force);
    floats(r, torque);
    floats(r, localPosition);
    floats(r, localRotation);
    return r.good();
}
void record(Writer& w, const CollisionGroupRecord& a)
{
    const auto& [path] = a;
    w.u32(path);
}
bool record(Reader& r, CollisionGroupRecord& a)
{
    auto& [path] = a;
    path = r.u32();
    return r.good();
}
void record(Writer& w, const CollisionMembershipRecord& a)
{
    const auto& [member, group] = a;
    w.u32(member);
    w.u32(group);
}
bool record(Reader& r, CollisionMembershipRecord& a)
{
    auto& [member, group] = a;
    member = r.u32();
    group = r.u32();
    return r.good();
}
void record(Writer& w, const CollisionGroupPairRecord& a)
{
    const auto& [first, second] = a;
    w.u32(first);
    w.u32(second);
}
bool record(Reader& r, CollisionGroupPairRecord& a)
{
    auto& [first, second] = a;
    first = r.u32();
    second = r.u32();
    return r.good();
}
void record(Writer& w, const ShapeFilterRecord& a)
{
    const auto& [shape, simulationGroup, queryGroup] = a;
    w.u64(shape);
    w.u32(simulationGroup);
    w.u32(queryGroup);
}
bool record(Reader& r, ShapeFilterRecord& a)
{
    auto& [shape, simulationGroup, queryGroup] = a;
    shape = r.u64();
    simulationGroup = r.u32();
    queryGroup = r.u32();
    return r.good();
}
void record(Writer& w, const FilteredPairRecord& a)
{
    const auto& [path, pairs] = a;
    w.u32(path);
    w.u64(pairs.size());
    for (const auto& p : pairs)
    {
        w.u64(p.first);
        w.u64(p.second);
    }
}
bool record(Reader& r, FilteredPairRecord& a)
{
    auto& [path, pairs] = a;
    path = r.u32();
    const auto count = r.u64();
    if (!r.good() || count > r.remaining() / 16)
        return false;
    for (uint64_t i = 0; i < count; ++i)
    {
        const auto first = r.u64(), second = r.u64();
        pairs.emplace_back(first, second);
    }
    return r.good();
}
void record(Writer& w, const ContactReportRecord& a)
{
    const auto& [actor, target, pendingPath, threshold] = a;
    w.u64(actor);
    w.u64(target);
    w.u32(pendingPath);
    w.f32(threshold);
}
bool record(Reader& r, ContactReportRecord& a)
{
    auto& [actor, target, pendingPath, threshold] = a;
    actor = r.u64();
    target = r.u64();
    pendingPath = r.u32();
    threshold = r.f32();
    return r.good();
}
void record(Writer& w, const MaterialRecord& a)
{
    const auto& [object, density, flags] = a;
    identity(w, object);
    w.f32(density);
    w.u32(flags);
}
bool record(Reader& r, MaterialRecord& a)
{
    auto& [object, density, flags] = a;
    identity(r, object);
    density = r.f32();
    flags = r.u32();
    return r.good();
}
void record(Writer& w, const ShapeRecord& a)
{
    const auto& [object, sourceGeometryPath, material, materialSlots, compoundShapes, scale, axis, instanceIndex,
                 volume, inertia, centerOfMass, localPosition, aabbMin, aabbMax, geometryPosition, localRotation,
                 geometryRotation] = a;
    identity(w, object);
    w.u32(sourceGeometryPath);
    w.u64(material);
    w.u64(materialSlots.size());
    for (SerialId id : materialSlots)
        w.u64(id);
    w.u64(compoundShapes.size());
    for (SerialId id : compoundShapes)
        w.u64(id);
    floats(w, scale);
    w.u32(axis);
    w.u32(instanceIndex);
    w.f32(volume);
    floats(w, inertia);
    floats(w, centerOfMass);
    floats(w, localPosition);
    floats(w, localRotation);
    floats(w, aabbMin);
    floats(w, aabbMax);
    floats(w, geometryPosition);
    floats(w, geometryRotation);
}
bool record(Reader& r, ShapeRecord& a)
{
    auto& [object, sourceGeometryPath, material, materialSlots, compoundShapes, scale, axis, instanceIndex, volume,
           inertia, centerOfMass, localPosition, aabbMin, aabbMax, geometryPosition, localRotation, geometryRotation] = a;
    identity(r, object);
    sourceGeometryPath = r.u32();
    material = r.u64();
    const uint64_t slots = r.u64();
    if (!r.good() || slots > 65535 || slots > r.remaining() / 8)
        return false;
    for (uint64_t i = 0; i < slots; ++i)
        materialSlots.push_back(r.u64());
    const uint64_t shapes = r.u64();
    if (!r.good() || shapes > r.remaining() / 8)
        return false;
    for (uint64_t i = 0; i < shapes; ++i)
        compoundShapes.push_back(r.u64());
    floats(r, scale);
    axis = r.u32();
    instanceIndex = r.u32();
    volume = r.f32();
    floats(r, inertia);
    floats(r, centerOfMass);
    floats(r, localPosition);
    floats(r, localRotation);
    floats(r, aabbMin);
    floats(r, aabbMax);
    floats(r, geometryPosition);
    floats(r, geometryRotation);
    return r.good();
}
void record(Writer& w, const ArticulationRecord& a)
{
    const auto& [object, selfCollision, staticRootBodyPath, fixedBaseJointPath] = a;
    identity(w, object);
    w.u8(selfCollision);
    w.u32(staticRootBodyPath);
    w.u32(fixedBaseJointPath);
}
bool record(Reader& r, ArticulationRecord& a)
{
    auto& [object, selfCollision, staticRootBodyPath, fixedBaseJointPath] = a;
    identity(r, object);
    if (!boolean(r, selfCollision))
        return false;
    staticRootBodyPath = r.u32();
    fixedBaseJointPath = r.u32();
    return r.good();
}
void record(Writer& w, const TendonRecord& a)
{
    const auto& [object, kind, instanceName, root, leaf, globalPosition, initialLength] = a;
    identity(w, object);
    w.u32(uint32_t(kind));
    w.string(instanceName);
    w.u8(root);
    w.u8(leaf);
    floats(w, globalPosition);
    w.f32(initialLength);
}
bool record(Reader& r, TendonRecord& a)
{
    auto& [object, kind, instanceName, root, leaf, globalPosition, initialLength] = a;
    identity(r, object);
    kind = TendonKind(r.u32());
    instanceName = r.string();
    if (!boolean(r, root) || !boolean(r, leaf))
        return false;
    floats(r, globalPosition);
    initialLength = r.f32();
    return r.good();
}
void record(Writer& w, const MimicRecord& a)
{
    const auto& [object, axis, gearingSign, offsetSign] = a;
    identity(w, object);
    w.u32(axis);
    w.f32(gearingSign);
    w.f32(offsetSign);
}
bool record(Reader& r, MimicRecord& a)
{
    auto& [object, axis, gearingSign, offsetSign] = a;
    identity(r, object);
    axis = r.u32();
    gearingSign = r.f32();
    offsetSign = r.f32();
    return r.good();
}
void jointState(Writer& w, const JointStateRecord& s)
{
    const auto& [token, enabled, convertToDegrees, physxAxis, initialPosition, initialVelocity] = s;
    w.string(token);
    w.u8(enabled);
    w.u8(convertToDegrees);
    w.u32(physxAxis);
    w.f32(initialPosition);
    w.f32(initialVelocity);
}
bool jointState(Reader& r, JointStateRecord& s)
{
    auto& [token, enabled, convertToDegrees, physxAxis, initialPosition, initialVelocity] = s;
    token = r.string();
    if (!boolean(r, enabled) || !boolean(r, convertToDegrees))
        return false;
    physxAxis = r.u32();
    initialPosition = r.f32();
    initialVelocity = r.f32();
    return r.good();
}
// A joint's settings; its identity is written beside them (see jointRecords).
void jointData(Writer& w, const JointRecord& a)
{
    const auto& [object, kind, articulationJoint, body0IsParent, axis, axisIndex, d6Drive, primaryDrive, axisDrives,
                 states] = a;
    (void)object;
    w.u32(uint32_t(kind));
    w.u8(articulationJoint);
    w.u8(body0IsParent);
    w.u32(axis);
    w.u32(axisIndex);
    w.u32(d6Drive);
    drive(w, primaryDrive);
    for (const DriveRecord& d : axisDrives)
        drive(w, d);
    for (const JointStateRecord& s : states)
        jointState(w, s);
}
bool jointData(Reader& r, JointRecord& a)
{
    auto& [object, kind, articulationJoint, body0IsParent, axis, axisIndex, d6Drive, primaryDrive, axisDrives, states] =
        a;
    (void)object;
    kind = JointKind(r.u32());
    if (!boolean(r, articulationJoint) || !boolean(r, body0IsParent))
        return false;
    axis = r.u32();
    axisIndex = r.u32();
    d6Drive = r.u32();
    if (!drive(r, primaryDrive))
        return false;
    for (DriveRecord& d : axisDrives)
        if (!drive(r, d))
            return false;
    for (JointStateRecord& s : states)
        if (!jointState(r, s))
            return false;
    return r.good();
}
// Repeated joint settings (including disabled fields and exact float bits) are
// stored once. Each object still gets independent mutable metadata on restore.
// The reader's bound: templates plus rows within kMaxJointRecords. The writer
// refuses what its reader would, so an oversized stage loads ordinarily instead
// of storing a bundle that every later lookup rejects.
bool jointRecordsFit(uint64_t templates, uint64_t rows)
{
    return templates <= kMaxJointRecords && rows <= kMaxJointRecords - templates;
}
bool jointRecords(Writer& w, const RuntimeArchive& archive)
{
    if (!archive.jointTemplates.empty() || !archive.jointInstances.empty())
    {
        if (!jointRecordsFit(archive.jointTemplates.size(), archive.jointInstances.size()))
            return false;
        w.u64(archive.jointTemplates.size());
        Writer scratch;
        for (const auto& value : archive.jointTemplates)
        {
            scratch.clear();
            jointData(scratch, value);
            w.u64(scratch.data().size());
            w.bytes(scratch.data().data(), scratch.data().size());
        }
        w.u64(archive.jointInstances.size());
        for (const auto& value : archive.jointInstances)
        {
            const auto& [object, settings] = value;
            identity(w, object);
            w.u32(settings);
        }
        return true;
    }
    const auto& joints = archive.joints;
    std::unordered_map<std::string, uint32_t> templates;
    std::vector<Bytes> data;
    std::vector<uint32_t> indices;
    indices.reserve(joints.size());
    Writer scratch;
    for (const JointRecord& joint : joints)
    {
        scratch.clear();
        jointData(scratch, joint);
        const auto& bytes = scratch.data();
        std::string key(reinterpret_cast<const char*>(bytes.data()), bytes.size());
        const auto found = templates.find(key);
        if (found != templates.end())
            indices.push_back(found->second);
        else
        {
            const uint32_t index = uint32_t(data.size());
            templates.emplace(std::move(key), index);
            data.push_back(bytes);
            indices.push_back(index);
        }
    }
    if (!jointRecordsFit(data.size(), joints.size()))
        return false;
    w.u64(data.size());
    for (const Bytes& bytes : data)
    {
        w.u64(bytes.size());
        w.bytes(bytes.data(), bytes.size());
    }
    w.u64(joints.size());
    for (size_t i = 0; i < joints.size(); ++i)
    {
        identity(w, joints[i].object);
        w.u32(indices[i]);
    }
    return true;
}
bool jointRecords(Reader& r, RuntimeArchive& archive, bool compact)
{
    const uint64_t count = r.u64();
    if (!r.good() || count > r.remaining() / 8 || !jointRecordsFit(count, 0))
        return false;
    std::vector<JointRecord> templates;
    for (uint64_t i = 0; i < count; ++i)
    {
        const uint64_t size = r.u64();
        if (!r.good() || size > r.remaining())
            return false;
        Bytes bytes(size);
        if (!r.bytes(bytes.data(), bytes.size()))
            return false;
        Reader body(bytes.data(), bytes.size());
        JointRecord value;
        if (!jointData(body, value) || !body.done())
            return false;
        templates.push_back(std::move(value));
    }
    const uint64_t rows = r.u64();
    if (!r.good() || rows > r.remaining() / 16 || !jointRecordsFit(count, rows))
        return false;
    if (compact)
        archive.jointInstances.reserve(size_t(rows));
    else
        archive.joints.reserve(size_t(rows));
    for (uint64_t i = 0; i < rows; ++i)
    {
        ObjectIdentity object;
        identity(r, object);
        const uint32_t index = r.u32();
        if (!r.good() || index >= templates.size())
            return false;
        if (compact)
            archive.jointInstances.push_back({ object, index });
        else
        {
            archive.joints.push_back(templates[index]);
            archive.joints.back().object = object;
        }
    }
    if (compact)
        archive.jointTemplates = std::move(templates);
    return true;
}

void record(Writer& w, const SchemaRecord& a)
{
    const auto& [path, flags] = a;
    w.u32(path);
    w.u64(flags);
}
bool record(Reader& r, SchemaRecord& a)
{
    auto& [path, flags] = a;
    path = r.u32();
    flags = r.u64();
    return r.good();
}
void record(Writer& w, const ObjectBindingRecord& a)
{
    const auto& [path, id, category, fixedBaseAlias] = a;
    w.u32(path);
    w.u64(id);
    w.u32(category);
    w.u8(fixedBaseAlias);
}
bool record(Reader& r, ObjectBindingRecord& a)
{
    auto& [path, id, category, fixedBaseAlias] = a;
    path = r.u32();
    id = r.u64();
    category = r.u32();
    return boolean(r, fixedBaseAlias);
}
void record(Writer& w, const FaceMapRecord& a)
{
    const auto& [shape, faces] = a;
    w.u64(shape);
    w.u64(faces.size());
    for (uint32_t face : faces)
        w.u32(face);
}
bool record(Reader& r, FaceMapRecord& a)
{
    auto& [shape, faces] = a;
    shape = r.u64();
    const uint64_t count = r.u64();
    if (!r.good() || count > r.remaining() / 4)
        return false;
    faces.resize(size_t(count));
    for (uint32_t& face : faces)
        face = r.u32();
    return r.good();
}
void record(Writer& w, PathId a)
{
    w.u32(a);
}
bool record(Reader& r, PathId& a)
{
    a = r.u32();
    return r.good();
}
void record(Writer& w, const std::string& a)
{
    w.string(a);
}
bool record(Reader& r, std::string& a)
{
    a = r.string();
    return r.good();
}
template <typename T>
void records(Writer& w, const std::vector<T>& values)
{
    w.u64(values.size());
    for (const T& value : values)
        record(w, value);
}
template <typename T>
bool records(Reader& r, std::vector<T>& values)
{
    const uint64_t count = r.u64();
    // Every record has at least a PathId. Check before entering a loop.
    if (!r.good() || count > r.remaining() / 4)
        return false;
    // Reserve without trusting a corrupt count to amplify memory beyond the
    // bytes still available. Large records can grow only as they are decoded.
    values.reserve(size_t(std::min<uint64_t>(count, r.remaining() / sizeof(T))));
    for (uint64_t i = 0; i < count; ++i)
    {
        T value;
        if (!record(r, value))
            return false;
        values.push_back(std::move(value));
    }
    return true;
}
} // namespace
bool encodeRuntimeArchive(const RuntimeArchive& archive, Bytes& output)
{
    if (!valid(archive))
        return false;
    const auto& [paths, scenePath, physicsPaths, actors, materials, shapes, articulations, joints, jointTemplates,
                 jointInstances, tendons, mimics, forces, instancedActors, pointInstancers, collisionGroups,
                 collisionMemberships, collisionGroupPairs, shapeFilters, filteredPairs, contactReports, schemas,
                 bindings, animatedKinematics, faceMaps] = archive;
    // Expanded joints or their compact templates and instances, see jointRecords.
    (void)joints, (void)jointTemplates, (void)jointInstances;
    Writer w;
    w.u32(kRuntimeArchiveVersion);
    records(w, paths);
    w.u32(scenePath);
    records(w, physicsPaths);
    records(w, actors);
    records(w, materials);
    records(w, shapes);
    records(w, articulations);
    if (!jointRecords(w, archive))
        return false;
    records(w, schemas);
    records(w, bindings);
    records(w, animatedKinematics);
    records(w, tendons);
    records(w, mimics);
    records(w, contactReports);
    records(w, collisionGroups);
    records(w, collisionMemberships);
    records(w, collisionGroupPairs);
    records(w, shapeFilters);
    records(w, filteredPairs);
    records(w, forces);
    records(w, instancedActors);
    records(w, pointInstancers);
    records(w, faceMaps);
    output = w.take();
    return true;
}
bool decodeRuntimeArchive(const Bytes& bytes, RuntimeArchive& output, bool compactJoints)
{
    return decodeRuntimeArchive(bytes.data(), bytes.size(), output, compactJoints);
}
bool decodeRuntimeArchive(const uint8_t* bytes, size_t size, RuntimeArchive& output, bool compactJoints)
{
    Reader r(bytes, size);
    if (r.u32() != kRuntimeArchiveVersion)
        return false;
    RuntimeArchive candidate;
    auto& [paths, scenePath, physicsPaths, actors, materials, shapes, articulations, joints, jointTemplates,
           jointInstances, tendons, mimics, forces, instancedActors, pointInstancers, collisionGroups,
           collisionMemberships, collisionGroupPairs, shapeFilters, filteredPairs, contactReports, schemas, bindings,
           animatedKinematics, faceMaps] = candidate;
    (void)joints, (void)jointTemplates, (void)jointInstances;
    if (!records(r, paths))
        return false;
    scenePath = r.u32();
    if (!records(r, physicsPaths) || !records(r, actors) || !records(r, materials) || !records(r, shapes) ||
        !records(r, articulations) || !jointRecords(r, candidate, compactJoints) || !records(r, schemas) ||
        !records(r, bindings) || !records(r, animatedKinematics) || !records(r, tendons) || !records(r, mimics) ||
        !records(r, contactReports) || !records(r, collisionGroups) || !records(r, collisionMemberships) ||
        !records(r, collisionGroupPairs) || !records(r, shapeFilters) || !records(r, filteredPairs) ||
        !records(r, forces) || !records(r, instancedActors) || !records(r, pointInstancers) || !records(r, faceMaps) ||
        !r.done() || !valid(candidate))
        return false;
    output = std::move(candidate);
    return true;
}
} // namespace omni::physx::cache
