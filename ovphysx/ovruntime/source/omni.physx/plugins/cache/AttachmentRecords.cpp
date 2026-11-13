// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-CACHE-ATTACH-001
 * @covers AC-20 AC-19 AC-18 AC-11 AC-2 AC-3 AC-4 AC-7 AC-8 AC-9 AC-10
 * @implements REQ-CACHE-UPDATE-001
 * @covers AC-6 AC-7 AC-8 AC-12 AC-13 AC-14
 * @implements REQ-CACHE-INITIAL-001
 * @covers AC-3 AC-6 AC-7
 */
#include "AttachmentCache.h"
#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>
#include <unordered_set>
#include <future>
#include "../../../omni.physics.ovstage/OvstageChangeFeed.h"
#include "../../../omni.physics.ovstage/OvstageSource.h"
#include <omni/physics/ovstage/StageFingerprint.h>
#include "../usdLoad/AttachedStage.h"
#include "../usdLoad/LoadStage.h"
#include "../internal/InternalPhysXDatabase.h"
#include "../internal/InternalScene.h"
#include "../internal/InternalMimicJoint.h"
#include "../PhysXScene.h"
#include "../ContactReport.h"
#include "../internal/InternalFilteredPairs.h"
#include "../ChangeRegister.h"
#include "../usdLoad/Material.h"
#include "../usdInterface/UsdInterface.h"
#include "../MeshCache.h"

#include <algorithm>
#include <cstring>
#include <functional>
#include <iterator>
#include <unordered_map>
#include <string_view>
#include <atomic>
#include <new>
#include <optional>
#include <stdexcept>

namespace omni::physx::cache
{
namespace
{
using namespace ::physx;
using namespace internal;
using namespace usdparser;
using omni::physics::parse::ObjectKey;
using Record = InternalDatabase::Record;
enum class InjectedFailure : int
{
    eNone,
    eBundleDefect,
    eOutOfMemory
};
std::atomic<InjectedFailure> failInsertionBeforeCommit{ InjectedFailure::eNone };
std::atomic<InjectedFailure> failRestoreAfterChange{ InjectedFailure::eNone };
std::atomic<InjectedFailure> failRestoreBeforeCommit{ InjectedFailure::eNone };
// Records a failed hydration released one at a time.
std::atomic<size_t> rolledBackRecordCount{ 0 };
// Throw the one-shot injected failure, if any.
void injectFailure(std::atomic<InjectedFailure>& armed, const char* what)
{
    switch (armed.exchange(InjectedFailure::eNone, std::memory_order_relaxed))
    {
    case InjectedFailure::eBundleDefect:
        throw BundleDefect(what);
    case InjectedFailure::eOutOfMemory:
        throw std::bad_alloc();
    default:
        break;
    }
}
// A stored ID that its table does not hold. Decoding checked every ID against the
// bundle's own sections, so the bundle contradicts itself: say so, rather than
// with the std::out_of_range of at(), which is not blamed on the bundle.
template <typename T>
T& stored(std::vector<T>& table, size_t index)
{
    if (index >= table.size())
        throw BundleDefect("a stored ID outside its table");
    return table[index];
}
template <typename Map>
typename Map::mapped_type& stored(Map& table, const typename Map::key_type& key)
{
    const auto found = table.find(key);
    if (found == table.end())
        throw BundleDefect("a stored ID outside its table");
    return found->second;
}
std::atomic<size_t> parallelCaptureCount{ 0 };
std::atomic<size_t> captureCandidateRecordCount{ 0 };
std::array<float, 3> wire(const PxVec3& value)
{
    return { value.x, value.y, value.z };
}
std::array<float, 4> wire(const PxQuat& value)
{
    return { value.x, value.y, value.z, value.w };
}
std::array<float, 3> wire(const carb::Float3& value)
{
    return { value.x, value.y, value.z };
}
std::array<float, 4> wire(const carb::Float4& value)
{
    return { value.x, value.y, value.z, value.w };
}
carb::Float3 live(const std::array<float, 3>& value)
{
    return { value[0], value[1], value[2] };
}
carb::Float4 live(const std::array<float, 4>& value)
{
    return { value[0], value[1], value[2], value[3] };
}
PxTransform transform(const std::array<float, 3>& position, const std::array<float, 4>& rotation)
{
    return PxTransform(
        PxVec3(position[0], position[1], position[2]), PxQuat(rotation[0], rotation[1], rotation[2], rotation[3]));
}
// How the ordinary load placed a restored dynamic body; see ActorRecord.
struct Insertion
{
    PxRigidDynamic* body;
    PxTransform pose;
    PxTransform appliedCenterOfMass;
};
// Adds restored scene objects in the order and state in which the ordinary load added
// the originals; the first simulation step depends on both. PhysX numbers objects in
// insertion order and keeps each shape's world transform from its insertion until the
// body moves. The load adds rigid actors as it creates them, each dynamic body at its
// insertion pose with its center of mass at the actor origin; PhysX normalizes the
// rotation on creation as its pose setter does, so replaying the pose is exact. It then
// creates the joints, listed here in that order, whose constraints join the scene as they
// are created, and applies the centers of mass in its mass update. It adds articulations,
// each in its aggregate, when it finishes setup; a static root body leaves the scene for
// its aggregate then.
void addAsLoaded(PxScene& scene,
                 const PxCollection& objects,
                 const std::vector<Insertion>& insertions,
                 const std::vector<PxJoint*>& joints)
{
    std::vector<PxArticulationReducedCoordinate*> articulations;
    std::vector<PxAggregate*> aggregates;
    std::unordered_set<PxAggregate*> articulationAggregates;
    for (PxU32 i = 0; i < objects.getNbObjects(); ++i)
    {
        PxBase& object = objects.getObject(i);
        if (auto* art = object.is<PxArticulationReducedCoordinate>())
        {
            articulations.push_back(art);
            if (PxAggregate* aggregate = art->getAggregate())
                articulationAggregates.insert(aggregate);
        }
        else if (auto* aggregate = object.is<PxAggregate>())
            aggregates.push_back(aggregate);
    }
    std::vector<PxActor*> actors;
    std::unordered_map<PxAggregate*, std::vector<PxRigidStatic*>> staticRoots;
    for (PxU32 i = 0; i < objects.getNbObjects(); ++i)
        if (auto* actor = objects.getObject(i).is<PxRigidActor>())
        {
            PxAggregate* aggregate = actor->getAggregate();
            if (aggregate && (!actor->is<PxRigidStatic>() || !articulationAggregates.count(aggregate)))
                continue;
            if (aggregate)
            {
                staticRoots[aggregate].push_back(actor->is<PxRigidStatic>());
                aggregate->removeActor(*actor);
            }
            actors.push_back(actor);
        }
    std::vector<const Insertion*> replayed;
    for (const Insertion& insertion : insertions)
    {
        // Replay only a center of mass that the load moved. Capture archived the insertion
        // only for a body that held the center of mass as PhysX stored it for the load.
        const PxTransform center = insertion.body->getCMassLocalPose();
        if (center.p.isZero() && center.q.isIdentity())
            continue;
        insertion.body->setCMassLocalPose(PxTransform(PxIdentity));
        insertion.body->setGlobalPose(insertion.pose);
        replayed.push_back(&insertion);
    }
    // PhysX adds a restored constraint when the second of its actors joins the scene, and
    // the solver's results depend on constraint order. Each joint between rigid actors
    // waits on a placeholder actor that never joins the scene, so that its constraint
    // joins in creation order. A joint on an articulation link joins with the link, as in
    // ordinary loading. PhysX finds a joint among its actor's connectors by a linear
    // search, so each placeholder holds a bounded number of joints: one placeholder for
    // every joint would make moving them back quadratic in the joint count.
    constexpr size_t kJointsPerPlaceholder = 64;
    std::vector<std::pair<PxJoint*, std::array<PxRigidActor*, 2>>> held;
    std::vector<PxRigidDynamic*> placeholders;
    for (PxJoint* joint : joints)
    {
        std::array<PxRigidActor*, 2> bodies{};
        joint->getActors(bodies[0], bodies[1]);
        if ((bodies[0] && bodies[0]->is<PxArticulationLink>()) || (bodies[1] && bodies[1]->is<PxArticulationLink>()))
            continue;
        if (held.size() % kJointsPerPlaceholder == 0)
        {
            PxRigidDynamic* placeholder = scene.getPhysics().createRigidDynamic(PxTransform(PxIdentity));
            if (!placeholder)
                break;
            placeholders.push_back(placeholder);
        }
        joint->setActors(placeholders.back(), nullptr);
        held.emplace_back(joint, bodies);
    }
    if (!actors.empty())
        scene.addActors(actors.data(), PxU32(actors.size()));
    for (const auto& [joint, bodies] : held)
        joint->setActors(bodies[0], bodies[1]);
    for (PxRigidDynamic* placeholder : placeholders)
        placeholder->release();
    for (const Insertion* insertion : replayed)
        insertion->body->setCMassLocalPose(insertion->appliedCenterOfMass);
    std::unordered_set<PxAggregate*> added;
    for (PxArticulationReducedCoordinate* art : articulations)
    {
        PxAggregate* aggregate = art->getAggregate();
        if (!aggregate)
        {
            scene.addArticulation(*art);
            continue;
        }
        if (!added.insert(aggregate).second)
            continue;
        const auto roots = staticRoots.find(aggregate);
        if (roots != staticRoots.end())
            for (PxRigidStatic* root : roots->second)
            {
                scene.removeActor(*root);
                aggregate->addActor(*root);
            }
        scene.addAggregate(*aggregate);
    }
    for (PxAggregate* aggregate : aggregates)
        if (added.insert(aggregate).second)
            scene.addAggregate(*aggregate);
}
DriveRecord wire(const PhysxJointDrive& value)
{
    DriveRecord result;
    result.enabled = value.enabled;
    result.acceleration = value.acceleration;
    result.stiffness = value.stiffness;
    result.damping = value.damping;
    result.maxForce = value.forceLimit;
    result.targetPosition = value.targetPosition;
    result.targetVelocity = value.targetVelocity;
    result.envelopeUsed = value.isEnvelopeUsed;
    result.maxActuatorVelocity = value.maxActuatorVelocity;
    result.velocityDependentResistance = value.velocityDependentResistance;
    result.speedEffortGradient = value.speedEffortGradient;
    return result;
}
PhysxJointDrive live(const DriveRecord& value)
{
    PhysxJointDrive result;
    result.enabled = value.enabled;
    result.acceleration = value.acceleration;
    result.stiffness = value.stiffness;
    result.damping = value.damping;
    result.forceLimit = value.maxForce;
    result.targetPosition = value.targetPosition;
    result.targetVelocity = value.targetVelocity;
    result.isEnvelopeUsed = value.envelopeUsed;
    result.maxActuatorVelocity = value.maxActuatorVelocity;
    result.velocityDependentResistance = value.velocityDependentResistance;
    result.speedEffortGradient = value.speedEffortGradient;
    return result;
}
// Indexed by JointKind.
constexpr ObjectType kJointTypes[] = { eJointFixed,    eJointRevolute, eJointPrismatic, eJointSpherical,
                                       eJointDistance, eJointD6,       eJointGear,      eJointRackAndPinion };
static_assert(std::size(kJointTypes) == size_t(JointKind::eRackAndPinion) + 1);
bool jointKind(ObjectType type, JointKind& kind)
{
    const auto found = std::find(std::begin(kJointTypes), std::end(kJointTypes), type);
    if (found == std::end(kJointTypes))
        return false;
    kind = JointKind(found - std::begin(kJointTypes));
    return true;
}
ObjectType jointType(JointKind kind)
{
    return size_t(kind) < std::size(kJointTypes) ? kJointTypes[size_t(kind)] : eUndefined;
}
PxBase* nativeObject(const Record& record)
{
    switch (record.mType)
    {
    case ePTActor:
    case ePTLink:
        return static_cast<PxRigidActor*>(record.mPtr);
    case ePTShape:
        return static_cast<PxShape*>(record.mPtr);
    case ePTCompoundShape: {
        const auto& shapes = static_cast<CompoundShape*>(record.mPtr)->getShapes();
        return shapes.empty() ? nullptr : shapes.front();
    }
    case ePTMaterial:
        return static_cast<PxMaterial*>(record.mPtr);
    case ePTJoint:
        return static_cast<PxJoint*>(record.mPtr);
    case ePTFixedTendonAxis:
        return static_cast<PxArticulationTendonJoint*>(record.mPtr);
    case ePTTendonAttachment:
        return static_cast<PxArticulationAttachment*>(record.mPtr);
    case ePTMimicJoint:
        return static_cast<PxArticulationMimicJoint*>(record.mPtr);
    case ePTLinkJoint:
        return static_cast<PxArticulationJointReducedCoordinate*>(record.mPtr);
    case ePTArticulation:
    case ePTArticulationFixedBase:
        return static_cast<PxArticulationReducedCoordinate*>(record.mPtr);
    default:
        return nullptr;
    }
}
// Owner bookkeeping records with no native SDK object of their own.
bool metadataOnly(PhysXType type)
{
    return type == ePTCollisionGroup || type == ePTFilteredPair || type == ePTForce || type == ePTPointInstancer;
}
// Room for `extra` more values, so no later append allocates. Growth is
// geometric: an exact reserve would copy the whole vector at every insertion.
template <typename T>
void reserveAhead(std::vector<T>& values, size_t extra)
{
    if (values.capacity() - values.size() < extra)
        values.reserve(std::max(values.size() + extra, values.capacity() * 2));
}
} // namespace

bool captureAttachmentData(usdparser::AttachedStage& owner,
                           RuntimeArchive& archive,
                           ::physx::PxCollection& roots,
                           std::string_view subtree,
                           const std::function<bool()>& nativeWork,
                           omni::physics::parse::cache::Bytes* runtimeBytes,
                           const InsertionCaptureSelection* selection,
                           const omni::physics::ovstage::PhysicsInputSnapshot* inputs)
{
    CARB_PROFILE_ZONE(0, "Compiled cache: capture attachment");
    using namespace ::physx;
    using namespace internal;
    using namespace usdparser;
    if (!runtimeBytes || roots.getNbObjects() != 0 || owner.getPhysXScenes().size() != 1 ||
        !owner.getTimeSampleMap().empty())
        return false;
    PhysXScene* scene = owner.getPhysXScenes().begin()->second;
    InternalPhysXDatabase& database = owner.getInternalPhysXDatabase();
    // Source-owned path text, resolved once per key.
    std::unordered_map<ObjectKey, std::string_view, ObjectKey::Hash> pathViews;
    pathViews.reserve(64);
    auto textFor = [&](ObjectKey key) {
        const auto found = pathViews.find(key);
        if (found != pathViews.end())
            return found->second;
        const auto text = owner.textViewFor(key);
        pathViews.emplace(key, text);
        return text;
    };
    auto included = [&](std::string_view path) {
        return subtree.empty() || path == subtree ||
               (path.size() > subtree.size() && path.substr(0, subtree.size()) == subtree && path[subtree.size()] == '/');
    };
    // Record membership was produced by the ordinary load. Whole attachment
    // capture retains its full scan; an insertion visits only its creation journal.
    std::vector<size_t> recordIndices;
    if (selection)
    {
        if (subtree.empty() || !selection->paths)
            return false;
        recordIndices = selection->records;
        std::sort(recordIndices.begin(), recordIndices.end());
        recordIndices.erase(std::unique(recordIndices.begin(), recordIndices.end()), recordIndices.end());
        if (!recordIndices.empty() && recordIndices.back() >= database.getRecords().size())
            return false;
    }
    const size_t recordCount = selection ? recordIndices.size() : database.getRecords().size();
    captureCandidateRecordCount.store(recordCount, std::memory_order_relaxed);
    const auto recordIndex = [&](size_t offset) { return selection ? recordIndices[offset] : offset; };
    struct Row
    {
        const Record* record;
        size_t index;
        SerialId id = 0;
        size_t shapeIndex = 0;
        // Rigid dynamic lock flags or material flags, read before the metadata
        // worker starts: PhysX binary deserialization resets both.
        uint32_t nativeFlags = 0;
        // A dynamic body whose insertion a hit replays, also decided before the worker
        // starts: one the load placed that still holds the center of mass PhysX stored
        // for the load's mass update.
        bool replayInsertion = false;
    };
    std::vector<Row> rows;
    rows.reserve(recordCount);
    RuntimeArchive candidate;
    candidate.paths.reserve(recordCount);
    // The archive's path table owns one copy of each path, found again by hash;
    // no caller's view is retained. Keys resolve their path once.
    std::unordered_multimap<size_t, PathId> pathIndex;
    pathIndex.reserve(recordCount);
    auto pathId = [&](std::string_view path) {
        if (path.empty())
            return PathId(0);
        const size_t hash = std::hash<std::string_view>{}(path);
        for (auto [it, end] = pathIndex.equal_range(hash); it != end; ++it)
            if (candidate.paths[size_t(it->second) - 1] == path)
                return it->second;
        candidate.paths.emplace_back(path);
        const auto id = PathId(candidate.paths.size());
        pathIndex.emplace(hash, id);
        return id;
    };
    std::unordered_map<ObjectKey, PathId, ObjectKey::Hash> keyPaths;
    keyPaths.reserve(recordCount);
    auto keyPath = [&](ObjectKey key) {
        const auto [it, inserted] = keyPaths.try_emplace(key, PathId(0));
        if (inserted)
            it->second = pathId(textFor(key));
        return it->second;
    };
    if (selection)
        candidate.scenePath = keyPath(scene->getInternalScene()->getSceneDesc().primKey);
    // Fixed-base binding alias record per articulation; the last one wins.
    std::unordered_map<const void*, size_t> fixedBaseAliases;
    for (size_t offset = 0; offset < recordCount; ++offset)
    {
        const size_t i = recordIndex(offset);
        const Record& record = database.getRecords()[i];
        if (record.mType == ePTRemoved)
            continue;
        if (record.mType == ePTScene)
        {
            candidate.scenePath = keyPath(record.mKey);
            continue;
        }
        if (record.mType == ePTArticulationFixedBase)
        {
            fixedBaseAliases[record.mPtr] = i;
            continue;
        }
        if (!included(textFor(record.mKey)) || metadataOnly(record.mType))
            continue;
        if (!record.mPtr || !record.mInternalPtr || !nativeObject(record))
            return false;
        rows.push_back({ &record, i });
    }
    // Rows stay in record creation order: hydration recreates records and adds
    // native objects in this order, matching the ordinary load's scene order.
    // The collection already owns the object-to-ID index. Freeze it before
    // finishCapture so the metadata worker can share it with native encoding,
    // which completes and sorts its own copy of the collection.
    const auto nativeId = [&](const PxBase* object) -> SerialId {
        return object ? roots.getId(*object) : PX_SERIAL_OBJECT_ID_INVALID;
    };
    const auto recordId = [&](size_t index) -> SerialId {
        if (index >= database.getRecords().size())
            return PX_SERIAL_OBJECT_ID_INVALID;
        const auto& record = database.getRecords()[index];
        const auto id = record.mPtr ? nativeId(nativeObject(record)) : PX_SERIAL_OBJECT_ID_INVALID;
        // An alias record (for example the fixed-base articulation binding) is
        // not itself a captured row, even when it points at an included object.
        return id && id <= rows.size() && rows[size_t(id) - 1].index == index ? id : PX_SERIAL_OBJECT_ID_INVALID;
    };
    size_t joints = 0, actors = 0, shapes = 0;
    for (const auto& row : rows)
    {
        joints += row.record->mType == ePTJoint || row.record->mType == ePTLinkJoint;
        actors += row.record->mType == ePTActor || row.record->mType == ePTLink;
        shapes += row.record->mType == ePTShape || row.record->mType == ePTCompoundShape;
    }
    candidate.joints.reserve(joints);
    candidate.actors.reserve(actors);
    candidate.shapes.reserve(shapes);
    candidate.bindings.reserve(rows.size());
    candidate.schemas.reserve(selection ? selection->keys.size() : owner.getObjectDatabase()->getSchemaAPIMap().size());
    for (size_t i = 0; i < rows.size(); ++i)
    {
        PxBase* object = nativeObject(*rows[i].record);
        if (roots.contains(*object))
            return false;
        rows[i].id = i + 1;
        roots.add(*object, rows[i].id);
    }
    SerialId nextDependency = rows.size() + 1;
    for (const Row& row : rows)
        if (row.record->mType == ePTCompoundShape)
            for (PxShape* shape : static_cast<CompoundShape*>(row.record->mPtr)->getShapes())
                if (!nativeId(shape))
                    roots.add(*shape, nextDependency++);
    // Read all SDK-owned data and finish the native closure before launching
    // owner-private metadata work. The worker reads only runtime state and the
    // frozen collection ID index, never native object state, while the caller
    // serializes the quiescent SDK objects through a separate collection copy.
    std::vector<PxMaterial*> materialSlots;
    std::unordered_map<size_t, bool> tendonRoots;
    const PxMaterial* defaultMaterial = scene->getDefaultMaterial();
    // A mesh cooked from original triangles has a runtime face map; archive it
    // once, with the first single shape that uses the mesh.
    std::unordered_set<const PxTriangleMesh*> faceMapMeshes;
    const auto captureFaceMap = [&](const PxShape& shape, SerialId id) {
        if (shape.getGeometry().getType() != PxGeometryType::eTRIANGLEMESH)
            return;
        const PxTriangleMesh* mesh = static_cast<const PxTriangleMeshGeometry&>(shape.getGeometry()).triangleMesh;
        const uint32_t* faces = mesh ? getMeshCache()->getTriangleMeshFaceMap(mesh) : nullptr;
        const PxU32* remap = faces ? mesh->getTrianglesRemap() : nullptr;
        if (!remap || !faceMapMeshes.insert(mesh).second)
            return;
        uint32_t count = 0;
        for (PxU32 t = 0; t < mesh->getNbTriangles(); ++t)
            count = std::max(count, remap[t] + 1);
        if (count)
            candidate.faceMaps.push_back({ id, std::vector<uint32_t>(faces, faces + count) });
    };
    for (Row& row : rows)
    {
        const Record& record = *row.record;
        if (record.mType == ePTShape || record.mType == ePTCompoundShape)
        {
            auto* shape = static_cast<PxShape*>(nativeObject(record));
            materialSlots.resize(shape->getNbMaterials());
            shape->getMaterials(materialSlots.data(), PxU32(materialSlots.size()));
            row.shapeIndex = candidate.shapes.size();
            candidate.shapes.emplace_back();
            auto& materials = candidate.shapes.back().materialSlots;
            materials.reserve(materialSlots.size());
            for (PxMaterial* material : materialSlots)
            {
                const auto id = nativeId(material);
                if (!id && material != defaultMaterial)
                    return false;
                materials.push_back(id);
            }
            if (record.mType == ePTShape)
                captureFaceMap(*shape, row.id);
        }
        else if (record.mType == ePTArticulation)
        {
            const auto& art = *static_cast<const InternalArticulation*>(record.mInternalPtr);
            if (art.mAggregate)
                roots.add(*art.mAggregate);
        }
        else if (record.mType == ePTActor)
        {
            if (const auto* dynamic = nativeObject(record)->is<PxRigidDynamic>())
            {
                row.nativeFlags = uint8_t(dynamic->getRigidDynamicLockFlags());
                // Compare with what PhysX stored, bit for bit; normalizing the applied
                // center of mass here could round differently from the SDK.
                const auto& actor = *static_cast<const InternalActor*>(record.mInternalPtr);
                const PxTransform center = dynamic->getCMassLocalPose();
                row.replayInsertion =
                    actor.mInsertionPose && std::memcmp(&center, &actor.mNormalizedCenterOfMass, sizeof(center)) == 0;
            }
        }
        else if (record.mType == ePTMaterial)
            row.nativeFlags = uint16_t(static_cast<PxMaterial*>(nativeObject(record))->getFlags());
        else if (record.mType == ePTFixedTendonAxis)
            tendonRoots.emplace(row.index, !static_cast<PxArticulationTendonJoint*>(record.mPtr)->getParent());
        else if (record.mType == ePTTendonAttachment)
            tendonRoots.emplace(row.index, !static_cast<PxArticulationAttachment*>(record.mPtr)->getParent());
    }
    // Collision group IDs and pair pointer hashes in native filter words belong
    // to the original owner. Archive logical references, then rebuild those words.
    auto groupPath = [&](uint32_t id, PathId& path) {
        if (!id)
        {
            path = 0;
            return true;
        }
        if (id >= database.getRecords().size())
            return false;
        const auto& group = database.getRecords()[id];
        if (group.mType != ePTCollisionGroup || !included(textFor(group.mKey)))
            return false;
        path = keyPath(group.mKey);
        return true;
    };
    for (const auto& row : rows)
    {
        if (row.record->mType != ePTShape && row.record->mType != ePTCompoundShape)
            continue;
        const auto addFilter = [&](PxShape* shape) {
            ShapeFilterRecord filter;
            filter.shape = nativeId(shape);
            if (!groupPath(shape->getSimulationFilterData().word2, filter.simulationGroup) ||
                !groupPath(shape->getQueryFilterData().word2, filter.queryGroup))
                return false;
            if (filter.simulationGroup || filter.queryGroup)
                candidate.shapeFilters.push_back(filter);
            return true;
        };
        if (row.record->mType == ePTShape)
        {
            if (!addFilter(static_cast<PxShape*>(row.record->mPtr)))
                return false;
        }
        else
            for (auto* shape : static_cast<CompoundShape*>(row.record->mPtr)->getShapes())
                if (!addFilter(shape))
                    return false;
    }
    for (size_t offset = 0; offset < recordCount; ++offset)
    {
        const size_t index = recordIndex(offset);
        const auto& record = database.getRecords()[index];
        if (record.mType == ePTRemoved || !included(textFor(record.mKey)))
            continue;
        if (record.mType == ePTCollisionGroup)
            candidate.collisionGroups.push_back({ keyPath(record.mKey) });
        if (record.mType == ePTFilteredPair)
        {
            const auto* pairs = static_cast<InternalFilteredPairs*>(record.mInternalPtr);
            if (!pairs)
                return false;
            FilteredPairRecord filter;
            filter.path = keyPath(record.mKey);
            for (const auto& pair : pairs->mPairs)
            {
                if (pair.first >= database.getRecords().size() || pair.second >= database.getRecords().size())
                    return false;
                // The ordinary collector includes every binding on each prim,
                // including force/group registrations. These provide no filter
                // objects in collectFilteredObjects and need no native reference.
                if (metadataOnly(database.getRecords()[pair.first].mType) ||
                    metadataOnly(database.getRecords()[pair.second].mType))
                    continue;
                const auto a = recordId(pair.first), b = recordId(pair.second);
                if (!a || !b)
                    return false;
                filter.pairs.emplace_back(a, b);
            }
            candidate.filteredPairs.push_back(std::move(filter));
        }
    }
    for (const auto& pair : owner.getCollisionGroupFilteredPairs())
    {
        if (pair.first() >= database.getRecords().size() || pair.second() >= database.getRecords().size())
            return false;
        if (!included(textFor(database.getRecords()[pair.first()].mKey)) &&
            !included(textFor(database.getRecords()[pair.second()].mKey)))
            continue;
        CollisionGroupPairRecord out;
        if (!groupPath(pair.first(), out.first) || !groupPath(pair.second(), out.second))
            return false;
        candidate.collisionGroupPairs.push_back(out);
    }
    auto members = [&](const CollisionGroupsMap& map) {
        for (const auto& member : map)
        {
            if (!included(textFor(member.first)))
                continue;
            for (const auto& group : member.second)
            {
                if (!included(textFor(group)))
                    return false;
                candidate.collisionMemberships.push_back({ keyPath(member.first), keyPath(group) });
            }
        }
        return true;
    };
    if (!members(owner.getCollisionGroupMap()))
        return false;
    for (const auto& map : owner.getAdditionalCollisionGroupMaps())
        if (!members(map))
            return false;
    // Report registration is owner state, not a native SDK object's userData.
    // Persist resolved actor IDs and preserve unresolved logical targets verbatim.
    for (const auto& pair : scene->getContactReport()->getContactPairsMap())
    {
        const auto actor = nativeId(pair.first);
        if (!actor)
            continue; // another insertion unit
        const auto target = nativeId(pair.second.first);
        if (pair.second.first && !target)
            return false;
        candidate.contactReports.push_back({ actor, target, 0, pair.second.second });
    }
    for (const auto& pair : scene->getContactReport()->getUnresolvedContactPairsMap())
    {
        const auto actor = nativeId(pair.first);
        if (!actor)
            continue;
        if (pair.second.first.valid() && !included(textFor(pair.second.first)))
            return false;
        candidate.contactReports.push_back(
            { actor, 0, pair.second.first.valid() ? keyPath(pair.second.first) : 0, pair.second.second });
    }
    auto finishCapture = [&]() {
        for (const Row& row : rows)
        {
            const Record& record = *row.record;
            const ObjectIdentity identity{ row.id, keyPath(row.record->mKey) };
            switch (record.mType)
            {
            case ePTActor:
            case ePTLink: {
                const InternalActor& actor = *static_cast<const InternalActor*>(record.mInternalPtr);
                if (actor.mSplinesCurve || !actor.mMirrors.empty())
                    return false;
                if (actor.mInstanceKey.valid())
                {
                    if (!included(textFor(actor.mInstanceKey)))
                        return false;
                    InstancedActorRecord instance;
                    instance.actor = identity.id;
                    instance.instancer = keyPath(actor.mInstanceKey);
                    instance.index = actor.mInstanceIndex;
                    std::copy(actor.mProtoTransformInverse.front(), actor.mProtoTransformInverse.front() + 16,
                              instance.prototypeInverse.begin());
                    candidate.instancedActors.push_back(instance);
                }
                ActorRecord out;
                out.object = identity;
                out.articulationLink = record.mType == ePTLink;
                if (out.articulationLink)
                    out.inboundJointHasState = static_cast<const InternalLink&>(actor).hasInboundJointWithStateAPI;
                out.scale = wire(actor.mScale);
                out.flags = actor.mFlags;
                out.surfaceVelocityEnabled = actor.surfaceVelocityEnabled();
                out.contactSolveEnabled = actor.contactSolveEnabled();
                out.surfaceVelocityLocalSpace = actor.mSurfaceVelocityLocalSpace;
                out.surfaceVelocity = wire(actor.mSurfaceVelocity);
                out.surfaceVelocityAuthored = wire(actor.mSurfaceVelocityAuthored);
                out.surfaceAngularVelocity = wire(actor.mSurfaceAngularVelocity);
                out.surfacePivotPosition = wire(actor.mSurfaceAngularVelocityPivot.p);
                out.surfacePivotRotation = wire(actor.mSurfaceAngularVelocityPivot.q);
                if (actor.mSourceGPrimKey.valid())
                    out.sourceGeometryPath = keyPath(actor.mSourceGPrimKey);
                const auto initial = database.mInitialActorDataMap.find(record.mKey);
                if (initial != database.mInitialActorDataMap.end())
                {
                    out.initialVelocity = wire(initial->second.velocity);
                    out.initialAngularVelocity = wire(initial->second.angularVelocity);
                    out.velocityWritten = initial->second.velocityWritten;
                    out.angularVelocityWritten = initial->second.angularVelocityWritten;
                }
                out.lockFlags = row.nativeFlags;
                if (row.replayInsertion)
                {
                    out.hasInsertionPose = true;
                    out.insertionPosition = wire(actor.mInsertionPose->p);
                    out.insertionRotation = wire(actor.mInsertionPose->q);
                    out.appliedCenterOfMassPosition = wire(actor.mAppliedCenterOfMass.p);
                    out.appliedCenterOfMassRotation = wire(actor.mAppliedCenterOfMass.q);
                }
                candidate.actors.push_back(std::move(out));
                break;
            }
            case ePTMaterial:
                candidate.materials.push_back(
                    { identity, static_cast<const InternalMaterial*>(record.mInternalPtr)->mDensity, row.nativeFlags });
                break;
            case ePTShape:
            case ePTCompoundShape: {
                const InternalShape& shape = *static_cast<const InternalShape*>(record.mInternalPtr);
                if (shape.mDetached)
                    return false;
                // Material slots were captured directly into this record before
                // native serialization; finish only the runtime-owned fields here.
                ShapeRecord& out = candidate.shapes[row.shapeIndex];
                if (shape.mSourceGprim.valid())
                {
                    const auto geometry = textFor(shape.mSourceGprim);
                    if (!included(geometry))
                        return false;
                    out.sourceGeometryPath = pathId(geometry);
                }
                out.object = identity;
                out.scale = wire(shape.mScale);
                out.instanceIndex = shape.mInstanceIndex;
                out.axis = shape.mAxis;
                const CompoundShape* compound =
                    record.mType == ePTCompoundShape ? static_cast<const CompoundShape*>(record.mPtr) : nullptr;
                if (compound)
                    for (PxShape* child : compound->getShapes())
                        out.compoundShapes.push_back(nativeId(child));
                if (shape.mMaterialId != kInvalidObjectId)
                {
                    const auto material = recordId(shape.mMaterialId);
                    if (!material)
                        return false;
                    out.material = material;
                }
                const PhysXUsdPhysicsInterface::MassInformation& mass = compound ? compound->mMassInfo : shape.mMassInfo;
                out.volume = mass.volume;
                std::copy(std::begin(mass.inertia), std::end(mass.inertia), out.inertia.begin());
                out.centerOfMass = wire(mass.centerOfMass);
                out.localPosition = wire(mass.localPos);
                out.localRotation = wire(mass.localRot);
                out.aabbMin = wire(mass.aabbLocalMin);
                out.aabbMax = wire(mass.aabbLocalMax);
                out.geometryPosition = wire(mass.geometryToSourcePos);
                out.geometryRotation = wire(mass.geometryToSourceRot);
                break;
            }
            case ePTArticulation: {
                const InternalArticulation& art = *static_cast<const InternalArticulation*>(record.mInternalPtr);
                ArticulationRecord out;
                out.object = identity;
                out.selfCollision = art.mEnableSelfCollision;
                if (art.mStaticRootBodyKey.valid())
                    out.staticRootBodyPath = keyPath(art.mStaticRootBodyKey);
                if (const auto alias = fixedBaseAliases.find(record.mPtr); alias != fixedBaseAliases.end())
                    out.fixedBaseJointPath = keyPath(database.getRecords()[alias->second].mKey);
                candidate.articulations.push_back(std::move(out));
                break;
            }
            case ePTFixedTendonAxis:
            case ePTTendonAttachment: {
                TendonRecord out;
                out.object = identity;
                if (record.mType == ePTFixedTendonAxis)
                {
                    out.instanceName = static_cast<const InternalTendonAxis*>(record.mInternalPtr)->instanceName;
                    out.root = tendonRoots.at(row.index);
                }
                else
                {
                    const auto& meta = *static_cast<const InternalTendonAttachment*>(record.mInternalPtr);
                    out.kind = TendonKind::eSpatialAttachment;
                    out.instanceName = meta.instanceName;
                    out.root = tendonRoots.at(row.index);
                    out.leaf = owner.getSource()->hasSchema(
                        record.mKey, owner.getSource()->internToken("PhysxTendonAttachmentLeafAPI:" + out.instanceName));
                    out.globalPosition = { meta.globalPos.x, meta.globalPos.y, meta.globalPos.z };
                    out.initialLength = meta.initLength;
                }
                candidate.tendons.push_back(std::move(out));
                break;
            }
            case ePTMimicJoint: {
                const auto& meta = *static_cast<const InternalMimicJoint*>(record.mInternalPtr);
                const uint32_t axis = (uint32_t(meta.getObjectType()) - uint32_t(eMimicJointRotX)) >> 16;
                candidate.mimics.push_back({ identity, axis, meta.getGearingSign(), meta.getOffsetSign() });
                break;
            }
            case ePTJoint:
            case ePTLinkJoint: {
                const InternalJoint& joint = *static_cast<const InternalJoint*>(record.mInternalPtr);
                JointRecord out;
                if (!jointKind(joint.mJointType, out.kind))
                    return false;
                out.object = identity;
                out.articulationJoint = record.mType == ePTLinkJoint;
                out.body0IsParent = joint.mBody0IsParentLink;
                out.axis = uint32_t(joint.mAxis);
                // Ordinary loading sets the drive axis of every maximal-coordinate
                // revolute or prismatic joint, driven or not: a drive added later uses
                // it. Other kinds and articulation joints leave these uninitialized.
                out.drive = wire(joint.mJointDrive);
                if (!out.articulationJoint && (out.kind == JointKind::eRevolute || out.kind == JointKind::ePrismatic))
                {
                    out.axisIndex = joint.mAxisIndex;
                    out.d6Drive = uint32_t(joint.mD6JointDrive);
                }
                for (size_t i = 0; i < 6; ++i)
                {
                    out.drives[i] = wire(joint.mJointDrives[i]);
                    const InternalJoint::InternalJointState& state = joint.mJointStates[i];
                    out.states[i] = { state.usdToken,
                                      state.enabled,
                                      state.convertToDegrees,
                                      uint32_t(state.physxAxis),
                                      state.initialState.position,
                                      state.initialState.velocity };
                }
                candidate.joints.push_back(std::move(out));
                break;
            }
            default:
                return false;
            }
        }
        const auto& keyMap = owner.getObjectDatabase()->getKeyMap();
        std::vector<ObjectKey> bindingKeys;
        if (selection)
        {
            bindingKeys = selection->keys;
            // Point instances and other synthesized bindings need not be authored
            // prims. Their creation records supply those keys without a source walk.
            for (const size_t index : recordIndices)
                if (database.getRecords()[index].mType != ePTRemoved)
                    bindingKeys.push_back(database.getRecords()[index].mKey);
            std::sort(
                bindingKeys.begin(), bindingKeys.end(), [](ObjectKey a, ObjectKey b) { return a.handle < b.handle; });
            bindingKeys.erase(std::unique(bindingKeys.begin(), bindingKeys.end()), bindingKeys.end());
        }
        const auto captureBindings = [&](ObjectKey key, const auto& entries) {
            for (const auto& entry : entries)
            {
                if (entry.second >= database.getRecords().size())
                    return false;
                const Record& record = database.getRecords()[entry.second];
                if (record.mType == ePTScene || !included(textFor(key)) || metadataOnly(record.mType))
                    continue;
                const auto id = nativeId(nativeObject(record));
                if (!id)
                    return false;
                candidate.bindings.push_back(
                    { keyPath(key), id, entry.first.getData(), record.mType == ePTArticulationFixedBase });
            }
            return true;
        };
        if (selection)
        {
            for (const auto key : bindingKeys)
            {
                const auto found = keyMap.find(key);
                if (found != keyMap.end() && !captureBindings(key, found->second))
                    return false;
            }
        }
        else
            for (const auto& entry : keyMap)
                if (!captureBindings(entry.first, entry.second))
                    return false;
        // Schema metadata comes from the runtime database, not from ovstage.
        if (selection)
        {
            for (const auto key : bindingKeys)
            {
                const auto schemas = owner.getObjectDatabase()->getSchemaAPIs(key);
                if (schemas && included(textFor(key)))
                    candidate.schemas.push_back({ keyPath(key), schemas });
            }
        }
        else
            for (const auto& entry : owner.getObjectDatabase()->getSchemaAPIMap())
                if (entry.second && included(textFor(entry.first)))
                    candidate.schemas.push_back({ keyPath(entry.first), entry.second });
        for (ObjectKey key : owner.getAnimatedKinematicBodies())
            if (included(textFor(key)))
                candidate.animatedKinematics.push_back(keyPath(key));
        for (size_t offset = 0; offset < recordCount; ++offset)
        {
            const size_t index = recordIndex(offset);
            const auto& record = database.getRecords()[index];
            if (record.mType != ePTForce || !included(textFor(record.mKey)))
                continue;
            const auto* force = static_cast<const InternalForce*>(record.mInternalPtr);
            if (!force)
                return false;
            const auto actor = nativeId(force->mRigidActor);
            if (!actor)
                return false;
            ForceRecord out;
            out.path = keyPath(record.mKey);
            out.actor = actor;
            out.worldFrame = force->mWorldFrame;
            out.accelerationMode = force->mAccelerationMode;
            out.enabled = force->mEnabled;
            out.centerOfMass = force->mCoMApplied;
            out.bodyPrimDifferent = force->mBodyPrimDifferent;
            out.force = wire(force->getForce());
            out.torque = wire(force->getTorque());
            out.localPosition = wire(force->mLocalPos);
            out.localRotation = wire(force->mLocalRot);
            candidate.forces.push_back(out);
        }
        for (size_t offset = 0; offset < recordCount; ++offset)
        {
            const size_t index = recordIndex(offset);
            const auto& record = database.getRecords()[index];
            if (record.mType != ePTPointInstancer || !included(textFor(record.mKey)))
                continue;
            PointInstancerRecord out;
            out.path = keyPath(record.mKey);
            const auto initial = database.mInitialPointInstancerTransforms.find(record.mKey);
            if (initial != database.mInitialPointInstancerTransforms.end())
            {
                out.initialArrays = true;
                for (const auto& v : initial->second.positions)
                    out.positions.push_back(wire(v));
                for (const auto& v : initial->second.orientations)
                    out.orientations.push_back(wire(v));
                for (const auto& v : initial->second.scales)
                    out.scales.push_back(wire(v));
                for (const auto& v : initial->second.velocities)
                    out.velocities.push_back(wire(v));
                for (const auto& v : initial->second.angularVelocities)
                    out.angularVelocities.push_back(wire(v));
            }
            candidate.pointInstancers.push_back(std::move(out));
        }
        auto* feed = dynamic_cast<omni::physics::ovstage::OvstageChangeFeed*>(owner.getChangeFeed());
        const auto addBaselinePath = [&](std::string_view path) {
            if (included(path))
                candidate.physicsPaths.push_back(pathId(path));
        };
        if (!feed || !(selection ? feed->forEachBaselinePath(*selection->paths, addBaselinePath) :
                                   feed->forEachBaselinePath(addBaselinePath)))
            return false;
        // Sort by path ID for restore locality; repeated bindings of one path keep their order.
        std::sort(candidate.physicsPaths.begin(), candidate.physicsPaths.end());
        std::stable_sort(candidate.bindings.begin(), candidate.bindings.end(),
                         [](const auto& a, const auto& b) { return a.path < b.path; });
        std::sort(candidate.schemas.begin(), candidate.schemas.end(),
                  [](const auto& a, const auto& b) { return a.path < b.path; });
        if (!subtree.empty())
        {
            const std::string scenePath(candidate.path(candidate.scenePath));
            for (auto& path : candidate.paths)
            {
                if (included(path))
                    path = std::string(omni::physics::parse::cache::kUnitRoot).append(path, subtree.size());
                else if (path != scenePath)
                    return false;
            }
        }
        // Generated native-instancing prototype names change between populations; store
        // geometry sources under prototypes by their stable instance-root names.
        const auto* ovstageSource = dynamic_cast<const omni::physics::ovstage::OvstageSource*>(owner.getSource());
        if (!omni::physics::ovstage::canonicalPrototypePaths(
                ovstageSource ? ovstageSource->instance() : nullptr, candidate.paths, inputs))
            return false;
        if (!encodeRuntimeArchive(candidate, *runtimeBytes))
            return false;
        archive = std::move(candidate);
        return true;
    };
    if (nativeWork && rows.size() >= (subtree.empty() ? 128 : 4096))
    {
        parallelCaptureCount.fetch_add(1, std::memory_order_relaxed);
        auto runtime = std::async(std::launch::async, finishCapture);
        const bool nativeValid = nativeWork();
        const bool runtimeValid = runtime.get();
        return nativeValid && runtimeValid;
    }
    if (!finishCapture())
        return false;
    return !nativeWork || nativeWork();
}
size_t getCompiledCaptureCandidateRecordCountForTest()
{
    return captureCandidateRecordCount.load(std::memory_order_relaxed);
}
void resetParallelCompiledCaptureCountForTest()
{
    parallelCaptureCount.store(0, std::memory_order_relaxed);
}
size_t getParallelCompiledCaptureCountForTest()
{
    return parallelCaptureCount.load(std::memory_order_relaxed);
}
bool captureAttachment(usdparser::AttachedStage& owner, RuntimeArchive& archive, ::physx::PxCollection& roots)
{
    omni::physics::parse::cache::Bytes encoded;
    return captureAttachmentData(owner, archive, roots, {}, {}, &encoded);
}

void initializeChangeFeedBaseline(usdparser::AttachedStage& owner)
{
    if (auto* feed = dynamic_cast<omni::physics::ovstage::OvstageChangeFeed*>(owner.getChangeFeed()))
        feed->initializeBaseline();
}

Hydration hydrateAttachment(usdparser::AttachedStage& owner,
                            const RuntimeArchive& archive,
                            std::unique_ptr<DecodedCollection> decoded,
                            bool insertion,
                            bool* committed,
                            std::vector<ObjectId>* created,
                            bool* changing)
{
    CARB_PROFILE_ZONE(0, "Compiled cache: hydrate attachment");
    using namespace ::physx;
    using namespace internal;
    using namespace usdparser;
    // Every check that can reject the bundle runs before the owner changes.
    if (!decoded || !archive.scenePath || archive.physicsPaths.empty())
        return Hydration::eRejected;
    // This worker builds only detached metadata from validated archive values.
    // Source/database mutation and native ownership transfer remain on the caller.
    auto jointMetadata = std::async(archive.jointCount() > 4096 ? std::launch::async : std::launch::deferred, [&]() {
        auto storage = std::make_shared<std::vector<InternalJoint>>(archive.jointCount());
        size_t index = 0;
        if (!archive.forEachJoint([&](const ObjectIdentity&, const JointRecord& r) {
                auto* meta = &(*storage)[index++];
                meta->mOwnsMemory = false;
                meta->mJointType = jointType(r.kind);
                meta->mAxis = Axis(r.axis);
                meta->mBody0IsParentLink = r.body0IsParent;
                meta->mAxisIndex = r.axisIndex;
                meta->mD6JointDrive = PxD6Drive::Enum(r.d6Drive);
                meta->mJointDrive = live(r.drive);
                for (size_t i = 0; i < 6; ++i)
                {
                    meta->mJointDrives[i] = live(r.drives[i]);
                    const JointStateRecord& s = r.states[i];
                    meta->mJointStates[i] = { s.token,
                                              s.enabled,
                                              s.convertToDegrees,
                                              PxArticulationAxis::Enum(s.physxAxis),
                                              { s.initialPosition, s.initialVelocity } };
                }
                return true;
            }))
            return decltype(storage){};
        return storage;
    });
    auto* feed = dynamic_cast<omni::physics::ovstage::OvstageChangeFeed*>(owner.getChangeFeed());
    if (!feed || (!insertion && feed->baselineInitialized()))
        return Hydration::eRejected;
    // A whole stage's change-feed baseline comes from the archived physics paths alone. Build it
    // while the bundle is checked; the attachment adopts it below.
    using PreparedBaseline = omni::physics::ovstage::OvstageChangeFeed::PreparedBaseline;
    std::future<PreparedBaseline> preparedBaseline;
    if (!insertion)
        preparedBaseline =
            std::async(archive.physicsPaths.size() > 4096 ? std::launch::async : std::launch::deferred, [&archive]() {
                std::vector<std::string_view> baseline;
                baseline.reserve(archive.physicsPaths.size());
                for (PathId id : archive.physicsPaths)
                    baseline.push_back(archive.path(id));
                return omni::physics::ovstage::OvstageChangeFeed::prepareBaseline(baseline);
            });
    PxCollection& remaining = decoded->collection();
    // Stored bytes are checksummed, not authenticated: refuse any native object the
    // encoder never writes, such as a custom geometry or a constraint that no joint
    // owns, whose callbacks would come straight from the bytes.
    for (PxU32 i = 0; i < remaining.getNbObjects(); ++i)
        if (!supportsNativeArchive(remaining.getObject(i)))
            return Hydration::eRejected;
    if (!constraintsOwnedByJoints(remaining))
        return Hydration::eRejected;
    const size_t objectCount = archive.actors.size() + archive.materials.size() + archive.shapes.size() +
                               archive.articulations.size() + archive.jointCount() + archive.tendons.size() +
                               archive.mimics.size();
    // Capture assigns dense serial IDs 1..N; out-of-range wire IDs are rejected.
    std::vector<PxBase*> native(objectCount + 1, nullptr);
    std::vector<PhysXType> types(objectCount + 1);
    std::vector<PathId> objectPaths(objectCount + 1);
    std::vector<std::string_view> paths;
    paths.reserve(archive.paths.size());
    for (const auto& path : archive.paths)
        paths.push_back(path);
    std::vector<ObjectKey> minted;
    auto* source = dynamic_cast<omni::physics::ovstage::OvstageSource*>(owner.getSource());
    if (!source)
        return Hydration::eRejected;
    if (insertion)
    {
        // The change feed and selected-input traversal already established the
        // destination keys; reuse them.
        minted.reserve(paths.size());
        for (const auto path : paths)
        {
            const auto key = source->findByPath(path);
            if (!key.valid())
                return Hydration::eRejected;
            minted.push_back(key);
        }
        if (owner.getPhysXScenes().size() != 1)
            return Hydration::eRejected;
        auto* scene = owner.getPhysXScenes().begin()->second;
        if (owner.textViewFor(scene->getInternalScene()->getSceneDesc().primKey) != archive.path(archive.scenePath))
            return Hydration::eRejected;
        for (const auto& binding : archive.bindings)
        {
            const auto* existing = owner.getObjectDatabase()->getEntries(stored(minted, binding.path - 1));
            if (existing && !existing->empty())
                return Hydration::eRejected;
        }
    }
    // An insertion's caller rebinds its unit-local names to the destination root, and a
    // whole stage has none, so a path that is still not absolute is a defect.
    if (std::any_of(paths.begin(), paths.end(), [](std::string_view path) { return path.front() != '/'; }))
        return Hydration::eRejected;
    auto resolve = [&](PathId path) { return path ? stored(minted, size_t(path) - 1) : ObjectKey{}; };
    auto validate = [&](const ObjectIdentity& identity, PhysXType type) {
        if (!identity.id || identity.id > objectCount || native[identity.id])
            return false;
        PxBase* object = remaining.find(identity.id);
        if (!object)
            return false;
        bool matches = false;
        switch (type)
        {
        case ePTActor:
            matches = object->is<PxRigidDynamic>() || object->is<PxRigidStatic>();
            break;
        case ePTLink:
            matches = object->is<PxArticulationLink>();
            break;
        case ePTMaterial:
            matches = object->is<PxMaterial>();
            break;
        case ePTShape:
        case ePTCompoundShape:
            matches = object->is<PxShape>();
            break;
        case ePTArticulation:
            matches = object->is<PxArticulationReducedCoordinate>();
            break;
        case ePTJoint:
            matches = object->is<PxJoint>();
            break;
        case ePTFixedTendonAxis:
            matches = (object->getConcreteType() == PxConcreteType::eARTICULATION_TENDON_JOINT);
            break;
        case ePTTendonAttachment:
            matches = (object->getConcreteType() == PxConcreteType::eARTICULATION_ATTACHMENT);
            break;
        case ePTMimicJoint:
            matches = (object->getConcreteType() == PxConcreteType::eARTICULATION_MIMIC_JOINT);
            break;
        case ePTLinkJoint:
            matches = object->is<PxArticulationJointReducedCoordinate>();
            break;
        default:
            break;
        }
        if (!matches)
            return false;
        native[identity.id] = object;
        types[identity.id] = type;
        objectPaths[identity.id] = identity.path;
        return true;
    };
    for (const ActorRecord& r : archive.actors)
        if (!validate(r.object, r.articulationLink ? ePTLink : ePTActor) ||
            (r.hasInsertionPose && (!native[r.object.id]->is<PxRigidDynamic>() ||
                                    !transform(r.insertionPosition, r.insertionRotation).isSane() ||
                                    !transform(r.appliedCenterOfMassPosition, r.appliedCenterOfMassRotation).isSane())))
            return Hydration::eRejected;
    for (const MaterialRecord& r : archive.materials)
        if (!validate(r.object, ePTMaterial))
            return Hydration::eRejected;
    for (const ShapeRecord& r : archive.shapes)
        if (r.materialSlots.empty() || !validate(r.object, r.compoundShapes.empty() ? ePTShape : ePTCompoundShape))
            return Hydration::eRejected;
    for (const ArticulationRecord& r : archive.articulations)
        if (!validate(r.object, ePTArticulation))
            return Hydration::eRejected;
    if (!archive.forEachJoint([&](const ObjectIdentity& object, const JointRecord& r) {
            if (!validate(object, r.articulationJoint ? ePTLinkJoint : ePTJoint))
                return false;
            if (r.articulationJoint)
            {
                if (r.kind == JointKind::eGear || r.kind == JointKind::eRackAndPinion)
                    return false;
            }
            else
            {
                PxBase* joint = native[object.id];
                bool matches = false;
                switch (r.kind)
                {
                case JointKind::eFixed:
                    matches = joint->is<PxFixedJoint>() || joint->is<PxD6Joint>();
                    break;
                case JointKind::eRevolute:
                    matches = joint->is<PxRevoluteJoint>() || joint->is<PxD6Joint>();
                    break;
                case JointKind::ePrismatic:
                    matches = joint->is<PxPrismaticJoint>() || joint->is<PxD6Joint>();
                    break;
                case JointKind::eSpherical:
                    matches = joint->is<PxSphericalJoint>() || joint->is<PxD6Joint>();
                    break;
                case JointKind::eDistance:
                    matches = joint->is<PxDistanceJoint>();
                    break;
                case JointKind::eD6:
                    matches = joint->is<PxD6Joint>();
                    break;
                case JointKind::eGear:
                    matches = joint->is<PxGearJoint>();
                    break;
                case JointKind::eRackAndPinion:
                    matches = joint->is<PxRackAndPinionJoint>();
                    break;
                }
                if (!matches)
                    return false;
            }
            return true;
        }))
        return Hydration::eRejected;
    for (const TendonRecord& r : archive.tendons)
    {
        if (!validate(r.object, r.kind == TendonKind::eFixedAxis ? ePTFixedTendonAxis : ePTTendonAttachment))
            return Hydration::eRejected;
        const bool root = r.kind == TendonKind::eFixedAxis ?
                              !static_cast<PxArticulationTendonJoint*>(native[r.object.id])->getParent() :
                              !static_cast<PxArticulationAttachment*>(native[r.object.id])->getParent();
        if (r.root != root)
            return Hydration::eRejected;
    }
    for (const MimicRecord& r : archive.mimics)
        if (!validate(r.object, ePTMimicJoint))
            return Hydration::eRejected;
    std::unordered_set<const PxBase*> compoundChildren;
    std::unordered_map<SerialId, PxShape*> compoundShapes;
    for (const ShapeRecord& r : archive.shapes)
        for (SerialId id : r.compoundShapes)
        {
            PxBase* shape = remaining.find(id);
            if (!shape || !shape->is<PxShape>() || !compoundChildren.insert(shape).second)
                return Hydration::eRejected;
            compoundShapes.emplace(id, static_cast<PxShape*>(shape));
        }
    // Liveness follows the matched complete source identity and stable read
    // ordinal; cached identities are not re-queried here.
    // All live bodies, shapes and joints need matching runtime metadata. Only
    // dependency materials, cooked meshes, constraints and automatic aggregates
    // may lack rows. Mesh application references remain with decoded until all
    // restored shapes have been released, before its aligned storage is freed.
    for (PxU32 i = 0; i < remaining.getNbObjects(); ++i)
    {
        PxBase& object = remaining.getObject(i);
        const auto serial = remaining.getId(object);
        if (serial < native.size() && native[serial])
            continue;
        if (compoundChildren.count(&object))
            continue;
        if (object.is<PxMaterial>() || object.is<PxConstraint>() || object.is<PxAggregate>())
            continue;
        if (object.is<PxConvexMesh>() || object.is<PxTriangleMesh>() || object.is<PxHeightField>())
            continue;
        if (object.getConcreteType() == PxConcreteType::eARTICULATION_FIXED_TENDON ||
            object.getConcreteType() == PxConcreteType::eARTICULATION_SPATIAL_TENDON)
            continue;
        // Dummy fixed-tendon root axes have no independent source record.
        if (auto* axis = (object.getConcreteType() == PxConcreteType::eARTICULATION_TENDON_JOINT ?
                              static_cast<PxArticulationTendonJoint*>(&object) :
                              nullptr))
            if (!axis->getParent())
                continue;
        // The fixed inbound joint of an otherwise un-authored articulation link
        // has no runtime record; its lifetime belongs to its articulation.
        if (object.is<PxArticulationJointReducedCoordinate>())
            continue;
        return Hydration::eRejected;
    }
    for (const ObjectBindingRecord& b : archive.bindings)
        if (!b.id || b.id >= native.size() || !native[b.id] ||
            (b.fixedBaseAlias && stored(types, b.id) != ePTArticulation))
            return Hydration::eRejected;
    std::unordered_set<SerialId> fixedBases;
    for (const ArticulationRecord& r : archive.articulations)
        if (r.fixedBaseJointPath)
            fixedBases.insert(r.object.id);
    for (const ObjectBindingRecord& b : archive.bindings)
        if (b.fixedBaseAlias && !fixedBases.count(b.id))
            return Hydration::eRejected;
    // Scene queries and contact reports resolve a triangle's USD face through the
    // runtime's face map, which PhysX does not serialize. Register each restored
    // mesh's map; the decoded collection forgets it before releasing the mesh,
    // since the table is keyed by address.
    for (const FaceMapRecord& r : archive.faceMaps)
    {
        const PxShape* shape = r.shape < native.size() && native[r.shape] ? native[r.shape]->is<PxShape>() : nullptr;
        if (!shape || shape->getGeometry().getType() != PxGeometryType::eTRIANGLEMESH)
            return Hydration::eRejected;
        const PxTriangleMesh* mesh = static_cast<const PxTriangleMeshGeometry&>(shape->getGeometry()).triangleMesh;
        const PxU32* remap = mesh ? mesh->getTrianglesRemap() : nullptr;
        if (!remap)
            return Hydration::eRejected;
        for (PxU32 t = 0; t < mesh->getNbTriangles(); ++t)
            if (remap[t] >= r.faces.size())
                return Hydration::eRejected;
        decoded->beforeRelease([mesh]() { getMeshCache()->forgetTriangleMeshFaceMap(mesh); });
        getMeshCache()->restoreTriangleMeshFaceMap(mesh, r.faces.data(), r.faces.size());
    }

    struct Release
    {
        void operator()(PxCollection* p) const
        {
            if (p)
                p->release();
        }
    };
    std::unique_ptr<PxCollection, Release> sceneObjects(PxCreateCollection());
    // Out of memory is this process's failure, not a defect of the bundle.
    if (!sceneObjects)
        throw std::bad_alloc();
    auto jointStorage = jointMetadata.get();
    if (!jointStorage)
        return Hydration::eRejected;
    // Shared shapes are writable only with their single application reference.
    // The decoded collection owns that reference throughout this operation;
    // temporarily detach actor references, then restore the topology. Only
    // decoded objects change here, which a rejected bundle releases.
    std::vector<std::pair<PxRigidActor*, PxShape*>> sharedAttachments;
    std::unordered_set<PxShape*> sharedShapes;
    const auto noteShared = [&sharedShapes](PxShape* shape) {
        if (shape->isExclusive())
            return true;
        if (!shape->getBaseFlags().isSet(PxBaseFlag::eIS_RELEASABLE))
            return false;
        sharedShapes.insert(shape);
        return true;
    };
    for (const auto& r : archive.shapes)
    {
        // Every child of a compound is written below, not only the first one,
        // which stands for the compound in the archive.
        if (!r.compoundShapes.empty())
        {
            for (SerialId serial : r.compoundShapes)
                if (!noteShared(stored(compoundShapes, serial)))
                    return Hydration::eRejected;
        }
        else if (!noteShared(static_cast<PxShape*>(stored(native, r.object.id))))
            return Hydration::eRejected;
    }
    if (!sharedShapes.empty())
        for (const auto& r : archive.actors)
        {
            auto* actor = static_cast<PxRigidActor*>(stored(native, r.object.id));
            std::vector<PxShape*> shapes(actor->getNbShapes());
            actor->getShapes(shapes.data(), PxU32(shapes.size()));
            for (auto* shape : shapes)
                if (sharedShapes.count(shape))
                {
                    sharedAttachments.emplace_back(actor, shape);
                    actor->detachShape(*shape, false);
                }
        }
    for (auto* shape : sharedShapes)
        if (shape->getReferenceCount() != 1)
            return Hydration::eRejected;
    for (const auto& filter : archive.shapeFilters)
    {
        PxBase* shape = remaining.find(filter.shape);
        if (!shape || !shape->is<PxShape>())
            return Hydration::eRejected;
    }
    // This hydration owns the aligned allocation until commit. On a failure the
    // registration guard retires the transferred objects before the allocation is
    // released, or hands it to a whole stage's candidate, which outlives them.
    DecodedCollection* lifetime = decoded.get();
    // Nothing above changed the owner. From here on a failure makes the caller
    // discard it and load ordinarily; only an insertion's registrations roll back.
    if (changing)
        *changing = true;
    if (!insertion)
        injectFailure(failRestoreAfterChange, "injected restore failure after the attachment changed");
    std::vector<ObjectId> sceneRecords;
    if (!insertion)
    {
        // Mint instance-local keys for the archive's paths, then let the change feed adopt
        // the archived membership prepared above; neither reads the other's state.
        const bool keysRestored = source->restoreKeysForPaths(paths, minted);
        const bool baselineRestored = feed->restoreBaseline(preparedBaseline.get());
        // The live source and change feed failed, not the decoded bundle.
        if (!baselineRestored || !keysRestored)
            return Hydration::eInterrupted;
        // The identity covers the scene's inputs: the ordinary parse rebuilds the
        // scene the bundle was compiled in. Its objects are announced with the
        // restored ones after hydration, so a throwing subscriber is never taken
        // for a failed restore.
        struct Suppress
        {
            bool& flag;
            bool previous;
            explicit Suppress(bool& value) : flag(value), previous(value)
            {
                flag = true;
            }
            ~Suppress()
            {
                flag = previous;
            }
        } suppress(owner.objectNotificationState().suppressed);
        // The attach may have created the scene while it captured the inputs.
        if (owner.preparedScene)
        {
            if (owner.textViewFor(owner.preparedScene->key) != archive.path(archive.scenePath))
                return Hydration::eInterrupted;
            sceneRecords = std::move(owner.preparedScene->records);
            owner.preparedScene.reset();
        }
        else
        {
            const size_t firstSceneRecord = owner.getInternalPhysXDatabase().getRecords().size();
            if (!loadSceneFromStage(owner, archive.path(archive.scenePath)))
                return Hydration::eInterrupted;
            for (size_t id = firstSceneRecord; id < owner.getInternalPhysXDatabase().getRecords().size(); ++id)
                sceneRecords.push_back(ObjectId(id));
        }
    }
    std::vector<ObjectKey> keys(objectCount + 1);
    for (size_t id = 1; id <= objectCount; ++id)
        if (native[id])
        {
            keys[id] = resolve(objectPaths[id]);
            if (!keys[id].valid())
                return Hydration::eFailed;
        }
    if (owner.getPhysXScenes().size() != 1)
        return Hydration::eFailed;
    PhysXScene* scene = owner.getPhysXScenes().begin()->second;
    InternalScene& internalScene = *scene->getInternalScene();
    InternalPhysXDatabase& db = owner.getInternalPhysXDatabase();
    const size_t count = native.size() + archive.articulations.size() + archive.collisionGroups.size() +
                         archive.filteredPairs.size() + archive.forces.size() + archive.pointInstancers.size();
    reserveAhead(db.getRecords(), count);
    reserveAhead(internalScene.mActors, archive.actors.size());
    reserveAhead(internalScene.mArticulations, archive.articulations.size());
    std::vector<ObjectId> ids(objectCount + 1, kInvalidObjectId), aliases(objectCount + 1, kInvalidObjectId);
    // Serialized userData belongs to the producer. In particular, articulation
    // teardown must never interpret an unregistered child's old ID in this owner.
    const auto invalid = reinterpret_cast<void*>(kInvalidObjectId);
    for (PxU32 i = 0; i < remaining.getNbObjects(); ++i)
    {
        PxBase& object = remaining.getObject(i);
        if (auto* actor = object.is<PxActor>())
            actor->userData = invalid;
        if (auto* shape = object.is<PxShape>())
            shape->userData = invalid;
        if (auto* joint = object.is<PxJoint>())
            joint->userData = invalid;
        if (auto* art = object.is<PxArticulationReducedCoordinate>())
            art->userData = invalid;
        // PhysX serializes no name for a link joint: its name pointer is the producer's.
        if (auto* joint = object.is<PxArticulationJointReducedCoordinate>())
        {
            joint->userData = invalid;
            joint->setName(nullptr);
        }
        switch (object.getConcreteType())
        {
        // addAsLoaded inserts only these roots. Constraints follow actors;
        // links/joints follow their articulation; meshes/materials remain
        // dependencies. Ownership transfers in 'remaining' stay unchanged.
        case PxConcreteType::eRIGID_DYNAMIC:
        case PxConcreteType::eRIGID_STATIC:
        case PxConcreteType::eARTICULATION_REDUCED_COORDINATE:
        case PxConcreteType::eAGGREGATE:
            sceneObjects->add(object);
            break;
        case PxConcreteType::eARTICULATION_FIXED_TENDON:
            static_cast<PxArticulationFixedTendon&>(object).userData = invalid;
            break;
        case PxConcreteType::eARTICULATION_SPATIAL_TENDON:
            static_cast<PxArticulationSpatialTendon&>(object).userData = invalid;
            break;
        // Articulation teardown finds tendon records through these.
        case PxConcreteType::eARTICULATION_TENDON_JOINT:
            static_cast<PxArticulationTendonJoint&>(object).userData = invalid;
            break;
        case PxConcreteType::eARTICULATION_ATTACHMENT:
            static_cast<PxArticulationAttachment&>(object).userData = invalid;
            break;
        default:
            break;
        }
    }
    // The owner keeps the decoded storage for as long as the registered objects in
    // it live. Restored materials and shapes can gain references from other owners;
    // the retained collection keeps one of its own until it proves them unused.
    const auto retainCollection = [&owner, &db, &decoded, lifetime](const std::vector<ObjectId>& records) {
        std::vector<PxRefCounted*> shared;
        for (ObjectId id : records)
        {
            const auto& record = db.getRecords()[id];
            if (!record.mPtr)
                continue;
            if (record.mType == ePTMaterial && !lifetime->owns(record.mPtr))
                continue; // the scene's default material
            if (record.mType == ePTMaterial)
                shared.push_back(static_cast<PxMaterial*>(record.mPtr));
            else if (record.mType == ePTShape)
                shared.push_back(static_cast<PxShape*>(record.mPtr));
            else if (record.mType == ePTCompoundShape)
                for (PxShape* shape : static_cast<CompoundShape*>(record.mPtr)->getShapes())
                    shared.push_back(shape);
        }
        owner.retainCompiledCollection(decoded, records, std::move(shared));
    };
    struct RegistrationRollback
    {
        AttachedStage& owner;
        bool notifications;
        bool committed = false;
        std::vector<ObjectId> added;
        std::vector<ObjectKey> bindings;
        std::vector<ObjectKey> collisionMembers;
        PxCollection* untransferred = nullptr; // objects still owned by the decoded collection
        // Set for a whole stage, whose caller discards the candidate after a failure.
        // The candidate then keeps the decoded storage, and its teardown releases what
        // was registered in one pass. Releasing record by record would scan every
        // record for each joint.
        std::function<void(const std::vector<ObjectId>&)> discard;
        RegistrationRollback(AttachedStage& stage, size_t count, size_t paths)
            : owner(stage), notifications(stage.getPhysXPhysicsInterface()->objectChangeNotificationsEnabled(stage))
        {
            added.reserve(count);
            bindings.reserve(paths);
            owner.getPhysXPhysicsInterface()->enableObjectChangeNotifications(owner, false);
        }
        ~RegistrationRollback()
        {
            if (!committed && discard)
                try
                {
                    discard(added);
                    committed = true;
                }
                catch (...)
                {
                    // Allocation failed before the owner took the storage; release
                    // the records here instead.
                }
            if (!committed)
                try
                {
                    struct Suppress
                    {
                        bool& flag;
                        bool previous;
                        explicit Suppress(bool& value) : flag(value), previous(value)
                        {
                            flag = true;
                        }
                        ~Suppress()
                        {
                            flag = previous;
                        }
                    } suppress(owner.objectNotificationState().suppressed);
                    auto& db = owner.getInternalPhysXDatabase();
                    // Retire pair registrations before releasing their referenced shapes.
                    for (ObjectId id : added)
                    {
                        const auto& record = db.getRecords()[id];
                        if (record.mType == ePTFilteredPair)
                            owner.getPhysXPhysicsInterface()->releaseObject(owner, record.mKey, id);
                    }
                    // Joints and shapes first, then articulations (which own links),
                    // remaining actor metadata and materials. All IDs are this unit's.
                    for (int phase = 0; phase < 4; ++phase)
                        for (ObjectId id : added)
                        {
                            auto& record = db.getRecords()[id];
                            const auto type = record.mType;
                            if (type == ePTRemoved)
                                continue;
                            const int order = type == ePTArticulation ? 1 :
                                              (type == ePTActor || type == ePTLink || type == ePTArticulationFixedBase) ?
                                                                        2 :
                                              type == ePTMaterial ? 3 :
                                                                    0;
                            if (order != phase)
                                continue;
                            if (type == ePTCollisionGroup)
                            {
                                removeFilteredObject<uint32_t>(uint32_t(id), owner.getCollisionGroupFilteredPairs());
                                delete static_cast<PxFilterData*>(record.mPtr);
                                record.mPtr = nullptr;
                                record.setRemoved();
                                continue;
                            }
                            db.mInitialActorDataMap.erase(record.mKey);
                            db.mInitialPointInstancerTransforms.erase(record.mKey);
                            // A fixed-base alias has no object of its own; releaseObject ignores it.
                            if (type == ePTPointInstancer || type == ePTArticulationFixedBase)
                            {
                                record.setRemoved();
                                continue;
                            }
                            owner.getPhysXPhysicsInterface()->releaseObject(owner, record.mKey, id);
                            rolledBackRecordCount.fetch_add(1, std::memory_order_relaxed);
                        }
                    for (auto key : collisionMembers)
                        owner.getCollisionGroupMap().erase(key);
                    for (auto key : bindings)
                    {
                        owner.getObjectDatabase()->removeEntries(key);
                        owner.getObjectDatabase()->removeSchemaAPIs(key);
                        owner.getAnimatedKinematicBodies().erase(key);
                    }
                    // Releasing an actor records its attached joints for later
                    // reattachment. Joints not yet transferred die with the decoded
                    // collection after this rollback, so the owner must forget them.
                    if (untransferred)
                        for (PxU32 i = 0; i < untransferred->getNbObjects(); ++i)
                            if (auto* joint = untransferred->getObject(i).is<PxJoint>())
                                db.removePxJoint(joint);
                }
                catch (...)
                {
                    // A destructor must not throw; the unit stays partially released
                    // and the caller's ordinary fallback reports the failure.
                    CARB_LOG_ERROR("Compiled cache: rolling back an unpublished unit failed");
                }
            owner.getPhysXPhysicsInterface()->enableObjectChangeNotifications(owner, notifications);
        }
    } rollback(owner, count, archive.bindings.size() + archive.schemas.size() + archive.animatedKinematics.size());
    rollback.untransferred = &remaining;
    if (!insertion)
        rollback.discard = [&decoded, &retainCollection](const std::vector<ObjectId>& records) {
            if (decoded) // commit may have handed it over before failing
                retainCollection(records);
        };
    rollback.bindings.reserve(rollback.bindings.capacity() + archive.collisionGroups.size() +
                              archive.filteredPairs.size() + archive.forces.size() + archive.pointInstancers.size());
    rollback.collisionMembers.reserve(archive.collisionMemberships.size());
    std::unordered_map<PathId, ObjectId> groupIds;
    for (const auto& group : archive.collisionGroups)
    {
        const auto key = resolve(group.path);
        std::unique_ptr<PxFilterData> data(new PxFilterData);
        const auto id = db.addRecord(ePTCollisionGroup, data.get(), nullptr, key);
        rollback.added.push_back(id);
        data.release();
        convertCollisionGroupToPxFilterData(
            convertToCollisionGroup(id), *static_cast<PxFilterData*>(db.getRecords()[id].mPtr));
        groupIds.emplace(group.path, id);
        rollback.bindings.push_back(key);
        owner.getObjectDatabase()->findOrCreateEntry(key, archive.path(group.path), eCollisionGroup, id);
    }
    for (const auto& pair : archive.collisionGroupPairs)
        owner.getCollisionGroupFilteredPairs().emplace(
            uint32_t(stored(groupIds, pair.first)), uint32_t(stored(groupIds, pair.second)));
    for (const auto& member : archive.collisionMemberships)
    {
        const auto key = resolve(member.member);
        rollback.collisionMembers.push_back(key);
        owner.getCollisionGroupMap()[key].push_back(resolve(member.group));
    }
    // A whole stage records its bindings' paths (hierarchy rows and path texts, the costly half
    // of an entry) on a worker while this thread wires the native objects, which use no ObjectDb.
    // The entries follow once their object ids exist. An insertion records each path with its
    // entry, so that its rollback, which removes entries with their rows, leaves none behind.
    std::vector<bool> pathRecorded(archive.paths.size() + 1);
    std::future<void> recordedPaths;
    if (!insertion)
    {
        owner.getObjectDatabase()->reserveAdditional(archive.paths.size(), archive.schemas.size());
        recordedPaths = std::async(std::launch::async, [&]() {
            ObjectDb& objects = *owner.getObjectDatabase();
            for (const ObjectBindingRecord& b : archive.bindings)
                if (b.path < pathRecorded.size() && !pathRecorded[b.path])
                {
                    pathRecorded[b.path] = true;
                    objects.recordPathText(resolve(b.path), archive.path(b.path));
                }
        });
    }
    // Clear original pointer hashes even on shapes whose pair was subsequently removed.
    for (PxU32 i = 0; i < remaining.getNbObjects(); ++i)
        if (auto* shape = remaining.getObject(i).is<PxShape>())
        {
            auto simulation = shape->getSimulationFilterData(), query = shape->getQueryFilterData();
            simulation.word1 = 0;
            simulation.word2 = 0;
            query.word2 = 0;
            shape->setSimulationFilterData(simulation);
            shape->setQueryFilterData(query);
        }
    for (const auto& filter : archive.shapeFilters)
    {
        auto* shape = remaining.find(filter.shape)->is<PxShape>();
        auto simulation = shape->getSimulationFilterData(), query = shape->getQueryFilterData();
        simulation.word2 = filter.simulationGroup ? uint32_t(stored(groupIds, filter.simulationGroup)) : 0;
        query.word2 = filter.queryGroup ? uint32_t(stored(groupIds, filter.queryGroup)) : 0;
        shape->setSimulationFilterData(simulation);
        shape->setQueryFilterData(query);
    }
    auto transfer = [&](SerialId serial, void* pointer, void* metadata) {
        const ObjectId id = db.addRecord(stored(types, serial), pointer, metadata, stored(keys, serial));
        rollback.added.push_back(id); // capacity reserved before publication
        // No throwing operations between registration and ownership transfer.
        if (remaining.contains(*stored(native, serial)))
            remaining.remove(*stored(native, serial));
        return id;
    };
    std::unordered_map<PxArticulationReducedCoordinate*, std::vector<PxBase*>> articulationChildren;
    for (PxU32 i = 0; i < remaining.getNbObjects(); ++i)
    {
        PxBase& object = remaining.getObject(i);
        PxArticulationReducedCoordinate* art = nullptr;
        switch (object.getConcreteType())
        {
        case PxConcreteType::eARTICULATION_FIXED_TENDON:
            art = static_cast<PxArticulationFixedTendon&>(object).getArticulation();
            break;
        case PxConcreteType::eARTICULATION_SPATIAL_TENDON:
            art = static_cast<PxArticulationSpatialTendon&>(object).getArticulation();
            break;
        case PxConcreteType::eARTICULATION_TENDON_JOINT:
            art = static_cast<PxArticulationTendonJoint&>(object).getTendon()->getArticulation();
            break;
        case PxConcreteType::eARTICULATION_ATTACHMENT:
            art = static_cast<PxArticulationAttachment&>(object).getTendon()->getArticulation();
            break;
        case PxConcreteType::eARTICULATION_MIMIC_JOINT:
            art = &static_cast<PxArticulationMimicJoint&>(object).getArticulation();
            break;
        default:
            break;
        }
        if (art)
            articulationChildren[art].push_back(&object);
    }
    for (const ArticulationRecord& r : archive.articulations)
    {
        PxArticulationReducedCoordinate* art = static_cast<PxArticulationReducedCoordinate*>(stored(native, r.object.id));
        // Runtime metadata derives from Allocateable, whose plain operator new goes
        // through Allocator::malloc while operator delete frees as from-new; a Debug
        // build asserts on that pairing. ICE_NEW records the allocation as from-new,
        // as ordinary loading does (UsdInterface.cpp).
        std::unique_ptr<InternalArticulation> meta(ICE_NEW(InternalArticulation)(scene));
        meta->mEnableSelfCollision = r.selfCollision;
        if (r.staticRootBodyPath)
            meta->mStaticRootBodyKey = resolve(r.staticRootBodyPath);
        meta->mAggregate = art->getAggregate();
        // Environment ids follow this attachment, as when ordinary loading creates the
        // aggregate; they can only be set before the collection joins the scene.
        if (meta->mAggregate)
            meta->mAggregate->setEnvironmentID(creationEnvironmentId(owner, stored(keys, r.object.id)));
        std::vector<PxArticulationLink*> links(art->getNbLinks());
        art->getLinks(links.data(), PxU32(links.size()));
        // Look up before the transfer: inserting an empty entry could fail after it.
        const auto children = articulationChildren.find(art);
        internalScene.mArticulations.push_back(art);
        const ObjectId id = transfer(r.object.id, art, meta.get());
        meta.release();
        if (children != articulationChildren.end())
            for (PxBase* child : children->second)
                remaining.remove(*child);
        for (PxArticulationLink* link : links)
        {
            if (remaining.contains(*link))
                remaining.remove(*link);
            PxArticulationJointReducedCoordinate* joint = link->getInboundJoint();
            if (joint && remaining.contains(*joint))
                remaining.remove(*joint);
        }
        if (art->getAggregate() && remaining.contains(*art->getAggregate()))
            remaining.remove(*art->getAggregate());
        art->userData = reinterpret_cast<void*>(id);
        art->setName(owner.getParseOptions().exposePrimNames ? owner.textFor(stored(keys, r.object.id)) : nullptr);
        ids[r.object.id] = id;
        if (r.fixedBaseJointPath)
        {
            aliases[r.object.id] = db.addRecord(ePTArticulationFixedBase, art, nullptr, resolve(r.fixedBaseJointPath));
            rollback.added.push_back(aliases[r.object.id]);
        }
    }
    std::vector<Insertion> insertions;
    // Siblings share their nested-rigid-body answer; the source read is stable here. That
    // check reads a few ancestors' schemas: their own rows, not a stage-wide membership query.
    NestedBodyMemo nestedBodies;
    std::optional<omni::physics::ovstage::OvstageSource::SinglePrimSchemaScope> ancestorSchemas;
    ancestorSchemas.emplace(*source);
    db.mInitialActorDataMap.reserve(db.mInitialActorDataMap.size() + archive.actors.size());
    for (const ActorRecord& r : archive.actors)
    {
        PxRigidActor* actor = static_cast<PxRigidActor*>(stored(native, r.object.id));
        const ObjectKey key = stored(keys, r.object.id);
        PxRigidDynamic* dynamic = actor->is<PxRigidDynamic>();
        if (dynamic)
            dynamic->setRigidDynamicLockFlags(PxRigidDynamicLockFlags(PxU8(r.lockFlags)));
        // Statics keep no environment id, and an aggregated actor takes its aggregate's.
        if (dynamic && !dynamic->getAggregate())
            dynamic->setEnvironmentID(creationEnvironmentId(owner, key));
        std::unique_ptr<InternalActor> meta;
        if (r.articulationLink)
        {
            std::unique_ptr<InternalLink> link(ICE_NEW(InternalLink)(scene, nullptr, key));
            link->hasInboundJointWithStateAPI = r.inboundJointHasState;
            meta = std::move(link);
        }
        else
            meta.reset(ICE_NEW(InternalActor)(scene,
                                              dynamic && !(dynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC),
                                              nullptr, (r.flags & InternalActorFlag::eLOCALSPACE_VELOCITIES) != 0, key,
                                              &nestedBodies));
        if (r.hasInsertionPose)
        {
            meta->mInsertionPose = transform(r.insertionPosition, r.insertionRotation);
            meta->mAppliedCenterOfMass = transform(r.appliedCenterOfMassPosition, r.appliedCenterOfMassRotation);
            meta->mNormalizedCenterOfMass = dynamic->getCMassLocalPose();
            insertions.push_back({ dynamic, *meta->mInsertionPose, meta->mAppliedCenterOfMass });
        }
        meta->mActor = actor;
        meta->mScale = live(r.scale);
        meta->mFlags = r.flags;
        if (r.sourceGeometryPath)
            meta->mSourceGPrimKey = resolve(r.sourceGeometryPath);
        db.mInitialActorDataMap[key] = { live(r.initialVelocity), live(r.initialAngularVelocity), r.velocityWritten,
                                         r.angularVelocityWritten };
        db.mInitialTransformsStored = true;
        internalScene.mActors.push_back(meta.get());
        const ObjectId id = transfer(r.object.id, actor, meta.get());
        meta.release();
        actor->userData = reinterpret_cast<void*>(id);
        actor->setName(owner.getParseOptions().exposePrimNames ? owner.textFor(key) : nullptr);
        ids[r.object.id] = id;
    }
    ancestorSchemas.reset();
    // As in ordinary loading, the scene's default-material prim is the scene's own
    // default material, so later colliders and edits share that one object. Its
    // deserialized copy stays an unused dependency of the collection.
    std::unordered_map<SerialId, PxMaterial*> sceneDefaultMaterials;
    for (const MaterialRecord& r : archive.materials)
    {
        PxMaterial* material = static_cast<PxMaterial*>(stored(native, r.object.id));
        std::unique_ptr<InternalMaterial> meta(ICE_NEW(InternalMaterial)(r.density));
        const ObjectKey key = stored(keys, r.object.id);
        ObjectId id = kInvalidObjectId;
        if (key.valid() && key == scene->getDefaultMaterialPath() && scene->getDefaultMaterial())
        {
            material = scene->getDefaultMaterial();
            id = db.addRecord(ePTMaterial, material, meta.get(), key);
            rollback.added.push_back(id); // capacity reserved before publication
            material->acquireReference(); // the record owns a reference independently of the scene
            sceneDefaultMaterials.emplace(r.object.id, material);
        }
        else
        {
            material->setFlags(PxMaterialFlags(PxU16(r.flags)));
            id = transfer(r.object.id, material, meta.get());
        }
        meta.release();
        material->userData = reinterpret_cast<void*>(id);
        ids[r.object.id] = id;
    }
    std::vector<PxMaterial*> slots;
    for (const ShapeRecord& r : archive.shapes)
    {
        PxShape* shape = static_cast<PxShape*>(stored(native, r.object.id));
        std::unique_ptr<InternalShape> meta(ICE_NEW(InternalShape)(scene, live(r.scale)));
        meta->mSourceGprim = resolve(r.sourceGeometryPath);
        meta->mAxis = r.axis;
        meta->mInstanceIndex = r.instanceIndex;
        if (r.material)
            meta->mMaterialId = stored(ids, r.material);
        slots.clear();
        for (SerialId id : r.materialSlots)
        {
            const auto sceneDefault = sceneDefaultMaterials.find(id);
            slots.push_back(!id                                         ? scene->getDefaultMaterial() :
                            sceneDefault != sceneDefaultMaterials.end() ? sceneDefault->second :
                                                                          static_cast<PxMaterial*>(stored(native, id)));
        }
        PhysXUsdPhysicsInterface::MassInformation& mass = meta->mMassInfo;
        mass.volume = r.volume;
        std::copy(r.inertia.begin(), r.inertia.end(), std::begin(mass.inertia));
        mass.centerOfMass = live(r.centerOfMass);
        mass.localPos = live(r.localPosition);
        mass.localRot = live(r.localRotation);
        mass.aabbLocalMin = live(r.aabbMin);
        mass.aabbLocalMax = live(r.aabbMax);
        mass.geometryToSourcePos = live(r.geometryPosition);
        mass.geometryToSourceRot = live(r.geometryRotation);
        std::unique_ptr<CompoundShape> compound;
        if (!r.compoundShapes.empty())
        {
            compound.reset(new CompoundShape(scene));
            compound->mMaterialId = meta->mMaterialId;
            compound->mMassInfo = mass;
            auto& children = const_cast<std::vector<PxShape*>&>(compound->getShapes());
            for (SerialId serial : r.compoundShapes)
                children.push_back(stored(compoundShapes, serial));
            // The ordinary compound release path owns the wrapper and native
            // children; keep its auxiliary metadata in the decoded arena.
            auto storage = std::shared_ptr<InternalShape>(meta.release());
            storage->mOwnsMemory = false;
            lifetime->retainRuntimeStorage(storage);
            const ObjectId id = transfer(r.object.id, compound.get(), storage.get());
            compound.release();
            ids[r.object.id] = id;
            for (SerialId serial : r.compoundShapes)
            {
                auto* child = stored(compoundShapes, serial);
                if (remaining.contains(*child))
                    remaining.remove(*child);
                child->setMaterials(slots.data(), PxU16(slots.size()));
                child->userData = reinterpret_cast<void*>(id);
                child->setName(owner.getParseOptions().exposePrimNames ? owner.textFor(stored(keys, r.object.id)) :
                                                                         nullptr);
            }
        }
        else
        {
            const ObjectId id = transfer(r.object.id, shape, meta.get());
            meta.release();
            ids[r.object.id] = id;
            shape->setMaterials(slots.data(), PxU16(slots.size()));
            shape->userData = reinterpret_cast<void*>(id);
            shape->setName(owner.getParseOptions().exposePrimNames ? owner.textFor(stored(keys, r.object.id)) : nullptr);
        }
        if (r.material)
            static_cast<InternalMaterial*>(db.getRecords()[stored(ids, r.material)].mInternalPtr)
                ->addShapeId(ids[r.object.id]);
    }
    if (recordedPaths.valid())
        recordedPaths.get();
    for (const auto& r : archive.pointInstancers)
    {
        const auto key = resolve(r.path);
        const auto id = db.addRecord(ePTPointInstancer, nullptr, nullptr, key);
        rollback.added.push_back(id);
        rollback.bindings.push_back(key);
        owner.getObjectDatabase()->findOrCreateEntry(key, archive.path(r.path), ePointInstancedBody, id);
        if (r.initialArrays)
        {
            auto& initial = db.mInitialPointInstancerTransforms[key];
            for (const auto& v : r.positions)
                initial.positions.push_back(live(v));
            for (const auto& v : r.orientations)
                initial.orientations.push_back(live(v));
            for (const auto& v : r.scales)
                initial.scales.push_back(live(v));
            for (const auto& v : r.velocities)
                initial.velocities.push_back(live(v));
            for (const auto& v : r.angularVelocities)
                initial.angularVelocities.push_back(live(v));
        }
    }
    for (const auto& r : archive.instancedActors)
    {
        auto* actor = static_cast<InternalActor*>(db.getRecords()[stored(ids, r.actor)].mInternalPtr);
        actor->mInstanceKey = resolve(r.instancer);
        actor->mInstanceIndex = r.index;
        for (uint32_t c = 0; c < 4; ++c)
            for (uint32_t row = 0; row < 4; ++row)
                actor->mProtoTransformInverse(row, c) = r.prototypeInverse[c * 4 + row];
    }
    for (const auto& r : archive.actors)
    {
        auto* actor = static_cast<PxRigidActor*>(stored(native, r.object.id));
        auto* meta = static_cast<InternalActor*>(db.getRecords()[stored(ids, r.object.id)].mInternalPtr);
        meta->mSurfaceVelocityLocalSpace = r.surfaceVelocityLocalSpace;
        meta->mSurfaceVelocity = PxVec3(r.surfaceVelocity[0], r.surfaceVelocity[1], r.surfaceVelocity[2]);
        meta->mSurfaceVelocityAuthored =
            PxVec3(r.surfaceVelocityAuthored[0], r.surfaceVelocityAuthored[1], r.surfaceVelocityAuthored[2]);
        meta->mSurfaceAngularVelocity =
            PxVec3(r.surfaceAngularVelocity[0], r.surfaceAngularVelocity[1], r.surfaceAngularVelocity[2]);
        meta->mSurfaceAngularVelocityPivot =
            PxTransform(PxVec3(r.surfacePivotPosition[0], r.surfacePivotPosition[1], r.surfacePivotPosition[2]),
                        PxQuat(r.surfacePivotRotation[0], r.surfacePivotRotation[1], r.surfacePivotRotation[2],
                               r.surfacePivotRotation[3]));
        meta->enableSurfaceVelocity(r.surfaceVelocityEnabled, *actor);
        meta->enableContactSolve(r.contactSolveEnabled, actor);
    }
    // Use the same non-owning-record protocol as replication, with an arena
    // that also runs string destructors when the compiled owner retires.
    lifetime->retainRuntimeStorage(jointStorage);
    size_t jointIndex = 0;
    std::vector<PxJoint*> joints; // in creation order, as captured
    if (!archive.forEachJoint([&](const ObjectIdentity& identity, const JointRecord& r) {
            InternalJoint* meta = &(*jointStorage)[jointIndex++];
            PxBase* object = stored(native, identity.id);
            const ObjectId id = transfer(identity.id, object, meta);
            if (r.articulationJoint)
            {
                PxArticulationJointReducedCoordinate* joint = static_cast<PxArticulationJointReducedCoordinate*>(object);
                joint->userData = reinterpret_cast<void*>(id);
                // Ordinary loading names every link joint, whatever exposePrimNames says.
                joint->setName(owner.textFor(stored(keys, identity.id)));
            }
            else
            {
                PxJoint* joint = static_cast<PxJoint*>(object);
                joint->userData = reinterpret_cast<void*>(id);
                if (joint->getConstraint() && remaining.contains(*joint->getConstraint()))
                    remaining.remove(*joint->getConstraint());
                joint->setName(owner.getParseOptions().exposePrimNames ? owner.textFor(stored(keys, identity.id)) :
                                                                         nullptr);
                joints.push_back(joint);
            }
            ids[identity.id] = id;
            return true;
        }))
        return Hydration::eFailed;
    for (const TendonRecord& r : archive.tendons)
    {
        if (r.kind == TendonKind::eFixedAxis)
        {
            auto* axis = static_cast<PxArticulationTendonJoint*>(stored(native, r.object.id));
            std::unique_ptr<InternalTendonAxis> meta(ICE_NEW(InternalTendonAxis));
            meta->instanceName = r.instanceName;
            if (r.root)
                registerFixedTendonChangeParams(owner, r.instanceName);
            else
                registerTendonAxisChangeParam(owner, r.instanceName);
            const ObjectId id = transfer(r.object.id, axis, meta.get());
            meta.release();
            axis->userData = reinterpret_cast<void*>(id);
            ids[r.object.id] = id;
            if (!axis->getParent())
                axis->getTendon()->userData = reinterpret_cast<void*>(id);
        }
        else
        {
            auto* attachment = static_cast<PxArticulationAttachment*>(stored(native, r.object.id));
            std::unique_ptr<InternalTendonAttachment> meta(ICE_NEW(InternalTendonAttachment));
            meta->instanceName = r.instanceName;
            meta->globalPos = PxVec3(r.globalPosition[0], r.globalPosition[1], r.globalPosition[2]);
            meta->initLength = r.initialLength;
            registerTendonAttachmentChangeParams(owner, r.instanceName);
            if (r.root)
                registerSpatialTendonChangeParams(owner, r.instanceName);
            if (r.leaf)
                registerTendonAttachmentLeafChangeParams(owner, r.instanceName);
            const ObjectId id = transfer(r.object.id, attachment, meta.get());
            meta.release();
            attachment->userData = reinterpret_cast<void*>(id);
            ids[r.object.id] = id;
            if (!attachment->getParent())
                attachment->getTendon()->userData = reinterpret_cast<void*>(id);
        }
    }
    for (const MimicRecord& r : archive.mimics)
    {
        auto* joint = static_cast<PxArticulationMimicJoint*>(stored(native, r.object.id));
        auto releaseMetadata = [](InternalMimicJoint* meta) {
            if (meta)
                meta->release(false, false);
        };
        std::unique_ptr<InternalMimicJoint, decltype(releaseMetadata)> meta(
            ICE_NEW(InternalMimicJoint)(internalScene, *joint, ObjectType(uint32_t(eMimicJointRotX) + (r.axis << 16)),
                                        r.gearingSign, r.offsetSign),
            releaseMetadata);
        const ObjectId id = transfer(r.object.id, joint, meta.get());
        InternalMimicJoint* registered = meta.release();
        joint->userData = reinterpret_cast<void*>(id);
        ids[r.object.id] = id;
        internalScene.addMimicJoint(*registered);
    }
    if (insertion)
        owner.getObjectDatabase()->reserveAdditional(archive.paths.size(), archive.schemas.size());
    // A path recorded already, by the worker or by a prim's earlier binding (a body's, for its
    // collider), only gains the entry.
    for (const ObjectBindingRecord& b : archive.bindings)
    {
        const auto id = b.fixedBaseAlias ? stored(aliases, b.id) : stored(ids, b.id);
        if (id == kInvalidObjectId)
            return Hydration::eFailed;
        const ObjectKey key = resolve(b.path);
        rollback.bindings.push_back(key);
        const auto category = ObjectCategory(ObjectType(b.category));
        if (b.path < pathRecorded.size() && pathRecorded[b.path])
            owner.getObjectDatabase()->findOrCreateEntryWithoutHierarchyStorage(key, category, id);
        else
        {
            if (b.path < pathRecorded.size())
                pathRecorded[b.path] = true;
            owner.getObjectDatabase()->findOrCreateEntry(key, archive.path(b.path), category, id);
        }
    }
    for (const SchemaRecord& s : archive.schemas)
    {
        rollback.bindings.push_back(resolve(s.path));
        owner.getObjectDatabase()->setSchemaAPI(resolve(s.path), s.flags);
    }
    for (PathId path : archive.animatedKinematics)
    {
        rollback.bindings.push_back(resolve(path));
        owner.getAnimatedKinematicBodies().insert(resolve(path));
    }
    internalScene.mForces.reserve(internalScene.mForces.size() + archive.forces.size());
    for (const auto& r : archive.forces)
    {
        std::unique_ptr<InternalForce> force(ICE_NEW(InternalForce));
        force->mWorldFrame = r.worldFrame;
        force->mAccelerationMode = r.accelerationMode;
        force->mEnabled = r.enabled;
        force->mCoMApplied = r.centerOfMass;
        force->mBodyPrimDifferent = r.bodyPrimDifferent;
        force->setForce(PxVec3(r.force[0], r.force[1], r.force[2]));
        force->setTorque(PxVec3(r.torque[0], r.torque[1], r.torque[2]));
        force->mLocalPos = PxVec3(r.localPosition[0], r.localPosition[1], r.localPosition[2]);
        force->mLocalRot = PxQuat(r.localRotation[0], r.localRotation[1], r.localRotation[2], r.localRotation[3]);
        force->mRigidActor = static_cast<PxRigidActor*>(stored(native, r.actor));
        force->mPhysXScene = scene;
        const auto key = resolve(r.path);
        const auto id = db.addRecord(ePTForce, nullptr, force.get(), key);
        rollback.added.push_back(id);
        auto* registered = force.release();
        internalScene.mForces.push_back(registered);
        rollback.bindings.push_back(key);
        owner.getObjectDatabase()->findOrCreateEntry(key, archive.path(r.path), ePhysxForce, id);
    }
    for (const auto& attachment : sharedAttachments)
        if (!attachment.first->attachShape(*attachment.second))
            return Hydration::eFailed;
    for (const auto& filter : archive.filteredPairs)
    {
        const auto key = resolve(filter.path);
        std::unique_ptr<InternalFilteredPairs> pairs(ICE_NEW(InternalFilteredPairs)(owner));
        for (const auto& pair : filter.pairs)
            pairs->mPairs.emplace_back(stored(ids, pair.first), stored(ids, pair.second));
        const auto id = db.addRecord(ePTFilteredPair, nullptr, pairs.get(), key);
        rollback.added.push_back(id);
        auto* registered = pairs.release();
        rollback.bindings.push_back(key);
        owner.getObjectDatabase()->findOrCreateEntry(key, archive.path(filter.path), eFilteredPair, id);
        registered->createFilteredPairs();
    }
    for (const auto& report : archive.contactReports)
    {
        auto* actor = static_cast<PxRigidActor*>(stored(native, report.actor));
        if (report.pendingPath)
            scene->getContactReport()->addActorPair(actor, resolve(report.pendingPath), report.threshold);
        else
            scene->getContactReport()->getContactPairsMap().emplace(
                actor, std::make_pair(report.target ? static_cast<PxRigidActor*>(stored(native, report.target)) : nullptr,
                                      report.threshold));
    }
    if (insertion)
        injectFailure(failInsertionBeforeCommit, "injected insertion failure before commit");
    else
        injectFailure(failRestoreBeforeCommit, "injected restore failure before commit");
    retainCollection(rollback.added);
    addAsLoaded(*scene->getScene(), *sceneObjects, insertions, joints);
    rollback.committed = true;
    if (committed)
        *committed = true;
    // Pending reports of other bodies may target the restored actors. Resolve them
    // only after commit: a rollback would erase them along with those actors.
    scene->getContactReport()->resolvePairs();
    if (created)
    {
        *created = std::move(sceneRecords);
        created->insert(created->end(), rollback.added.begin(), rollback.added.end());
    }
    return Hydration::eHydrated;
}
void notifyRestoredObjects(AttachedStage& owner, const std::vector<ObjectId>& created)
{
    if (!owner.getPhysXPhysicsInterface()->objectCreationNotificationsWanted(owner))
        return;
    const auto& records = owner.getInternalPhysXDatabase().getRecords();
    std::unordered_set<const void*> fixedBases;
    for (const ObjectId id : created)
        if (records[id].mType == ePTArticulationFixedBase)
            fixedBases.insert(records[id].mPtr);
    for (const ObjectId id : created)
    {
        const auto& record = records[id];
        if (record.mType == ePTLinkJoint || record.mType == ePTArticulationFixedBase || record.mType == ePTRemoved)
            continue;
        const PhysXType type =
            record.mType == ePTArticulation && fixedBases.count(record.mPtr) ? ePTArticulationFixedBase : record.mType;
        owner.getPhysXPhysicsInterface()->sendObjectCreationNotification(owner, record.mKey, id, type);
    }
}
// One-shot fault injection exercises rollback after all runtime bindings exist:
// a fault in the bundle, or running out of memory. False withdraws it.
void failNextCompiledInsertionForTest(bool enabled, bool outOfMemory)
{
    failInsertionBeforeCommit.store(!enabled    ? InjectedFailure::eNone :
                                    outOfMemory ? InjectedFailure::eOutOfMemory :
                                                  InjectedFailure::eBundleDefect,
                                    std::memory_order_relaxed);
}
// One-shot fault injection: the next whole-stage restore throws right after its
// checks pass, once it may change the attachment, or with beforeCommit once all
// its runtime bindings exist. False withdraws it.
void failNextCompiledRestoreForTest(bool enabled, bool outOfMemory, bool beforeCommit)
{
    const InjectedFailure failure = !enabled    ? InjectedFailure::eNone :
                                    outOfMemory ? InjectedFailure::eOutOfMemory :
                                                  InjectedFailure::eBundleDefect;
    failRestoreAfterChange.store(beforeCommit ? InjectedFailure::eNone : failure, std::memory_order_relaxed);
    failRestoreBeforeCommit.store(beforeCommit ? failure : InjectedFailure::eNone, std::memory_order_relaxed);
}
size_t getCompiledRolledBackRecordCountForTest()
{
    return rolledBackRecordCount.load(std::memory_order_relaxed);
}
} // namespace omni::physx::cache
