// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-CACHE-ATTACH-001
 * @covers AC-9 AC-14
 */

/** @implements REQ-PARSE-FEED-006
 * @covers AC-1 AC-2 AC-3 AC-4 AC-8
 */

/**
 * @implements REQ-CUSTOM-LIFETIME-001
 * @covers AC-6
 *
 * @implements REQ-PARSE-FEED-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-PARSE-CORE-003
 * @covers AC-10
 *
 * @implements REQ-PUBLICAPI-003
 * @covers AC-4
 *
 * @implements REQ-SIM-OVSTAGE-WRITEAPPLY-001
 * @covers AC-8 AC-14 AC-15 AC-16 AC-17 AC-20
 *
 * @implements REQ-PARSE-FEED-002
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-PARSE-FEED-003
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-11 AC-13 AC-14 AC-16
 *
 * @implements REQ-PARSE-FEED-004
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-PARSE-XFORM-001
 * @covers AC-5
 *
 * @implements REQ-PARSE-FEED-005
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-8 AC-9
 */

#include "OvstageChangeFeed.h"
#include <omni/physics/parse/CustomTokens.h>

#include "OvstageSource.h"
#include "ReadGroupUtils.h"
#include "ReadColumn.h"
#include <ovstage/ovstage_api/experimental/ovstage_placement_extension.h> // official ovstage.placement device read

#include <ovstage/ovstage_api/ovstage_tensor_utils.h>

#include <carb/profiler/Profile.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <future>
#include <initializer_list>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

namespace omni::physics::ovstage
{
namespace
{

ovx_string_t ovxStr(const char* s)
{
    return { s, std::string_view(s).size() };
}

// Test-only count of live initial-baseline scans (seedKnownPhysicsPaths).
std::atomic_size_t initialBaselineScans{ 0 };

// True when ovstage exposes the device-placement read ("ovstage.placement"); an ovstage without it
// returns not-OK here, gating the feed onto the host read path.
bool deviceReadAvailable(ovstage_instance_t* instance)
{
    const ovstage_placement_extension_v1_t* ext = nullptr;
    return instance && ovstage_query_placement_extension(instance, &ext) == OVSTAGE_OK;
}

// Test-only count of structural drains that dropped the hierarchy cache instead of
// patching it (tombstones, failed delta read, rejected delta, every whole-stage rescan). A
// fallback storm on a fast producer would silently restore the whole-stage rebuild per spawn;
// this makes it visible.
std::atomic_size_t& hierarchyFallbackCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

// Test-only failure after an incremental add has been classified.
std::atomic_int& addedKeyResolutionFault()
{
    static std::atomic_int count{ 0 };
    return count;
}

// Test-only count of whole-stage rescans of the known physics sets. Steady-state spawning
// must take the incremental path; this shows a rescan storm before the drift fit does.
std::atomic_size_t& structuralRescanCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

// Test-only count of structural rows the whole-stage growth probe walked. A value-only drain
// must walk none: every row it returns is stringified and patched into the source's memos.
std::atomic_size_t& structuralRowsReadCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

// Test-only: the next N collectKnownPathsFromFilter scans report failure (a partial seed /
// rescan), the shape mKnownSetAuthoritative exists for.
std::atomic_int& knownPathScanFaultCounter()
{
    static std::atomic_int counter{ 0 };
    return counter;
}

bool consumeFault(std::atomic_int& counter)
{
    int n = counter.load(std::memory_order_relaxed);
    while (n > 0)
    {
        if (counter.compare_exchange_weak(n, n - 1, std::memory_order_relaxed))
            return true;
    }
    return false;
}

bool consumeKnownPathScanFaultForTest()
{
    return consumeFault(knownPathScanFaultCounter());
}

// Test-only: the next N readStructuralRows reads fail (the growth probe cannot run).
std::atomic_int& structuralRowsReadFaultCounter()
{
    static std::atomic_int counter{ 0 };
    return counter;
}

void waitAndRelease(ovstage_instance_t* inst, ovstage_enqueue_result_t r)
{
    if (r.status != OVSTAGE_OK || r.op_index == OVSTAGE_INVALID_OP_ID)
        return;
    ovstage_wait_op(inst, r.op_index, OVSTAGE_TIMEOUT_INFINITE, nullptr);
    ovstage_release_op(inst, r.op_index);
}

int64_t totalElements(const DLTensor& t)
{
    int64_t n = 1;
    for (int i = 0; i < t.ndim; ++i)
        n *= t.shape[i];
    return n * (t.dtype.lanes > 0 ? t.dtype.lanes : 1);
}

bool isTokenIdTensor(const DLTensor& t)
{
    if (t.ndim < 0 || (t.ndim > 0 && !t.shape) || t.dtype.code != kDLUInt ||
        (t.dtype.bits != 64 && t.dtype.bits != 32))
        return false;
    // Empty schema rows are complete deltas even when the producer omits storage.
    return t.data || totalElements(t) == 0;
}

uint64_t tokenIdAt(const DLTensor& t, int64_t i)
{
    const auto* bytes = static_cast<const uint8_t*>(t.data) + t.byte_offset;
    return t.dtype.bits == 64 ? reinterpret_cast<const uint64_t*>(bytes)[i] :
                                static_cast<uint64_t>(reinterpret_cast<const uint32_t*>(bytes)[i]);
}

// usd-prim-type row: fixed token column, one id per prim (OvstageSource::buildSchemaCache's
// fixed-column decode). False when the payload is not that shape.
bool decodeTypeRow(const ReadGroupColumn& g, uint32_t row, uint64_t& value)
{
    if (g.is_array || g.data.tensor_count == 0 || !g.data.tensors)
        return false;
    const DLTensor& t = g.data.tensors[0];
    if (!isTokenIdTensor(t))
        return false;
    const int64_t total = totalElements(t);
    const int64_t storedRows = (t.ndim > 0 && t.shape[0] > 0) ? t.shape[0] : 1;
    if (total <= 0 || total % storedRows != 0 || total / storedRows != 1)
        return false;
    const uint32_t dataRow = g.data.index_map ? g.data.index_map[row] : row;
    if (dataRow >= static_cast<uint32_t>(storedRows))
        return false;
    value = tokenIdAt(t, dataRow);
    return true;
}

// usd-schemas row: token-id array, one tensor per prim. A packed single-tensor group with
// several prims fails the tensor_count bound and is reported undecodable.
template <typename Fn>
bool forEachSchemaToken(const ReadGroupColumn& g, uint32_t row, Fn&& fn)
{
    if (!g.is_array || !g.data.tensors)
        return false;
    const uint32_t tensorIndex = g.data.index_map ? g.data.index_map[row] : row;
    if (tensorIndex >= g.data.tensor_count)
        return false;
    const DLTensor& t = g.data.tensors[tensorIndex];
    if (!isTokenIdTensor(t))
        return false;
    const int64_t count = totalElements(t);
    if (count < 0)
        return false;
    for (int64_t i = 0; i < count; ++i)
    {
        const uint64_t value = tokenIdAt(t, i);
        if (value != 0)
            fn(value);
    }
    return true;
}

// Map a per-column DLTensor (dtype + per-prim component count) onto a ColumnView
// element type. Returns ColumnType::eNone for shapes the feed does not deliver
// (e.g. double vectors / 4x4 matrices - physics-attribute scope first; see the
// transform note in the header).
ColumnType columnTypeOf(const DLDataType& dt, int64_t comps)
{
    if (dt.code == kDLFloat && dt.bits == 32)
    {
        if (comps <= 1) return ColumnType::eFloat;
        if (comps == 2) return ColumnType::eFloat2;
        if (comps == 3) return ColumnType::eFloat3;
        if (comps == 4) return ColumnType::eFloat4;
        return ColumnType::eNone;
    }
    if (dt.code == kDLFloat && dt.bits == 64)
        return comps <= 1 ? ColumnType::eDouble : ColumnType::eNone;
    if (dt.code == kDLInt && dt.bits == 32) return comps <= 1 ? ColumnType::eInt32 : ColumnType::eNone;
    if (dt.code == kDLInt && dt.bits == 64) return comps <= 1 ? ColumnType::eInt64 : ColumnType::eNone;
    if (dt.code == kDLUInt && dt.bits == 32) return comps <= 1 ? ColumnType::eInt32 : ColumnType::eNone;
    if (dt.code == kDLUInt && dt.bits == 64) return comps <= 1 ? ColumnType::eInt64 : ColumnType::eNone;
    if (dt.code == kDLBool) return ColumnType::eBool;
    return ColumnType::eNone;
}

// Value-feed attribute filter. Fabric-internal computed columns and the built-in
// usd-* metadata (usd-path / usd-schemas / usd-prim-type) belong to structural
// discovery, not the value feed. Populated USD metadata is the exception: it lives
// under the reserved `usd-metadata:` prefix and is ordinary parse data (e.g. a
// PointInstancer's inactive-id set), so an edit to it must reach the value feed.
bool isValueAttributeName(std::string_view name)
{
    if (name.empty() || name[0] == '_' || name == conv::kUsdTypeName)
        return false;
    const bool builtInUsdMetadata = name.rfind("usd-", 0) == 0 && name.rfind(conv::kMetadataPrefix, 0) != 0;
    return !builtInUsdMetadata;
}

// The families of prim that make a path "known physics" to this feed: an attribute
// change on any other path is not delivered to the runtime at all. Both the initial
// seed and the structural refresh consult these, and they used to be two duplicated
// literal arrays -- a duplicated fact is a defect smell, and it cost a real gap
// (PhysxCharacterControllerAPI was missing, so CCT property updates never reached the
// runtime on any non-USD source). Keep them here, in one place.
const ovx_string_t kKnownPhysicsSchemas[] = {
    ovxStr("PhysicsRigidBodyAPI"), ovxStr("PhysicsCollisionAPI"), ovxStr("PhysicsMaterialAPI"), ovxStr("PhysxForceAPI"),
    ovxStr("PhysxSurfaceVelocityAPI"), ovxStr("PhysxContactReportAPI"), ovxStr("PhysicsFilteredPairsAPI"),
    ovxStr("PhysicsArticulationRootAPI"), ovxStr("PhysxCharacterControllerAPI"), ovxStr("PhysxVehicleTireAPI"),
    ovxStr("PhysxVehicleWheelAttachmentAPI"),
    // Particle sets carry per-particle arrays the drain applies through the write backend. Deformable
    // bodies (volume + surface) are also queried, but their per-vertex `deformablePose` change is NOT
    // drained -- it is the sim mesh's bind pose, which falls back to the parse-path resync/re-cook; the
    // schemas are here so that change is DELIVERED and the resync fires. That per-vertex state lives on the
    // SIM-MESH prim, which under a hierarchical layout is a child of the body prim and carries the sim API,
    // NOT the body API -- so its schemas must be selected too, or the child's changes are silently dropped
    // while the ordinal advances and the resync never runs.
    ovxStr("OmniPhysicsDeformableBodyAPI"), ovxStr("PhysxParticleSetAPI"), ovxStr("OmniPhysicsVolumeDeformableSimAPI"),
    ovxStr("OmniPhysicsSurfaceDeformableSimAPI")
};
const ovx_string_t kKnownPhysicsPrimTypes[] = {
    ovxStr("PhysicsScene"),          ovxStr("PhysicsFixedJoint"),     ovxStr("PhysicsRevoluteJoint"),
    ovxStr("PhysicsPrismaticJoint"), ovxStr("PhysicsSphericalJoint"), ovxStr("PhysicsDistanceJoint"),
    ovxStr("PhysicsJoint"),          ovxStr("PhysxPhysicsGearJoint"), ovxStr("PhysxPhysicsRackAndPinionJoint"),
    ovxStr("PhysicsCollisionGroup"), ovxStr("PointInstancer"),        ovxStr("PhysxPhysicsJointInstancer")
};

} // namespace

// Not declared in the public header; test translation units forward-declare these
// (same pattern as the OvstageSource.cpp counters).
void resetOvstageInitialBaselineScanCountForTest()
{
    initialBaselineScans.store(0, std::memory_order_relaxed);
}
size_t getOvstageInitialBaselineScanCountForTest()
{
    return initialBaselineScans.load(std::memory_order_relaxed);
}

void resetOvstageHierarchyFallbackCountForTest()
{
    hierarchyFallbackCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstageHierarchyFallbackCountForTest()
{
    return hierarchyFallbackCounter().load(std::memory_order_relaxed);
}

namespace
{
// Value ChangeBatches dispatched by readValueChanges (coalesced and per-group alike).
std::atomic_size_t& valueBatchDispatchCounter()
{
    static std::atomic_size_t count{ 0 };
    return count;
}
} // namespace

void resetOvstageValueBatchDispatchCountForTest()
{
    valueBatchDispatchCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstageValueBatchDispatchCountForTest()
{
    return valueBatchDispatchCounter().load(std::memory_order_relaxed);
}

void resetOvstageStructuralRescanCountForTest()
{
    structuralRescanCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstageStructuralRescanCountForTest()
{
    return structuralRescanCounter().load(std::memory_order_relaxed);
}

void resetOvstageStructuralRowsReadCountForTest()
{
    structuralRowsReadCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstageStructuralRowsReadCountForTest()
{
    return structuralRowsReadCounter().load(std::memory_order_relaxed);
}

void setOvstageKnownPathScanFaultForTest(int count)
{
    knownPathScanFaultCounter().store(count, std::memory_order_relaxed);
}

void setOvstageAddedKeyResolutionFaultForTest(int count)
{
    addedKeyResolutionFault().store(count, std::memory_order_relaxed);
}

void setOvstageStructuralRowsReadFaultForTest(int count)
{
    structuralRowsReadFaultCounter().store(count, std::memory_order_relaxed);
}

void OvstageChangeFeed::refreshCustomFamilies()
{
    const uint64_t generation=customTokenGeneration();
    if (generation==mCustomTokenGeneration && !mPhysicsTypeNames.empty()) return;
    // Retain discovered names through this feed's lifetime: unregistering code
    // must not hide pending tombstones. Plugin retirement independently gates
    // live native state. Rebuild string views only after storage has settled.
    std::vector<std::string> types = mPhysicsTypeNames;
    std::vector<std::string> schemas = mPhysicsSchemaNames;
    if (types.empty())
        for (const ovx_string_t & name : kKnownPhysicsPrimTypes)
            types.emplace_back(name.ptr, name.length);
    if (schemas.empty())
        for (const ovx_string_t & name : kKnownPhysicsSchemas)
            schemas.emplace_back(name.ptr, name.length);
    for (const omni::physics::parse::CustomTokenKind kind : { CustomTokenKind::eJoint, CustomTokenKind::eShape,
                            CustomTokenKind::ePhysicsInstancer })
        for (const std::string & name : customTokens(kind))
        {
            types.push_back(name);
            if (kind == CustomTokenKind::eShape)
                schemas.push_back(name);
        }
    void (*const unique)(std::vector<std::string>&) = [](std::vector<std::string>& names) {
        std::sort(names.begin(), names.end());
        names.erase(std::unique(names.begin(), names.end()), names.end());
    };
    unique(types); unique(schemas);
    if (types == mPhysicsTypeNames && schemas == mPhysicsSchemaNames) {mCustomTokenGeneration=generation;return;}
    std::vector<ovx_string_t> primTypes,schemaNames;
    primTypes.reserve(types.size());schemaNames.reserve(schemas.size());
    for (const std::string& name:types) primTypes.push_back({name.data(),name.size()});
    for (const std::string& name:schemas) schemaNames.push_back({name.data(),name.size()});
    mPhysicsTypeNames=std::move(types);mPhysicsSchemaNames=std::move(schemas);
    mPhysicsPrimTypes=std::move(primTypes);mPhysicsSchemas=std::move(schemaNames);
    mKnownFamilyTokensBuilt = false;
    mKnownSetAuthoritative = false;
    dirtyCachedFamilyQueries();
    mCustomTokenGeneration=generation;
}

OvstageChangeFeed::OvstageChangeFeed(OvstageSource& source,
                                     ovstage_instance_t* instance,
                                     ovx_path_dictionary_t* dict,
                                     bool deferBaseline)
    : mSource(source), mInstance(instance), mDict(dict), mLastDrainedOrd1(source.readOrdinal())
{
    refreshCustomFamilies();
    if (!deferBaseline)
        initializeBaseline();
}

OvstageChangeFeed::~OvstageChangeFeed()
{
    releaseCachedQuery(mKnownPhysicsQuery);
    releaseCachedQuery(mKnownTransformQuery);
    releaseCachedQuery(mSchemaFamilyQuery);
    releaseCachedQuery(mTypeFamilyQuery);
}

void OvstageChangeFeed::registerInterest(
    ObjectKey objectType, TokenId prop, int /*device*/, OnChangeFn cb, uint64_t userData)
{
    // ovstage range reads are host-resident here; the requested device is ignored.
    mRegistrations.push_back({ objectType, prop, std::move(cb), userData });
    mRegisteredAttributeTokensDirty = true;
    mValueReadAttrsDirty = true;
}

void OvstageChangeFeed::registerGroupComplete(OnGroupCompleteFn cb)
{
    mGroupComplete = std::move(cb);
}

bool OvstageChangeFeed::drain()
{
    // ovstage has no implicit "since last" window - changes are pulled over an
    // explicit ordinal range via drainRange (engine: updateFromOvStage).
    return true;
}

void OvstageChangeFeed::setEnabled(bool enabled)
{
    mEnabled = enabled;
}

void OvstageChangeFeed::setDeviceOrdinal(int ordinal)
{
    mDeviceOrdinal = ordinal;
}

bool OvstageChangeFeed::dispatch(const ChangeBatch& batch) const
{
    bool ok = true;
    for (const Registration& reg : mRegistrations)
    {
        // Wildcard (invalid objectType + invalid prop) matches every batch;
        // otherwise match on the property the registration declared interest in
        // (mirrors UsdChangeFeed::dispatch).
        const bool wildcard = !reg.objectType.valid() && !reg.prop.valid();
        if (!wildcard && reg.prop != batch.property)
            continue;
        if (reg.cb)
        {
            ChangeBatch b = batch;
            b.userData = reg.userData;
            // A false return means the consumer committed part of the batch and then failed: the
            // range must be held and redelivered, so the whole drain fails closed.
            ok = reg.cb(b) && ok;
        }
    }
    return ok;
}

void OvstageChangeFeed::initializeBaseline()
{
    if (mBaselineInitialized)
        return;
    seedKnownPhysicsPaths();
    mBaselineInitialized = true;
}
bool OvstageChangeFeed::forEachBaselinePath(const std::function<void(std::string_view)>& visit) const
{
    if (!mBaselineInitialized || !mKnownSetAuthoritative)
        return false;
    // Membership is a set; the archive orders its own copies.
    for (const auto& path : mKnownPhysicsPaths)
        visit(path);
    return true;
}
bool OvstageChangeFeed::forEachBaselinePath(const std::vector<std::string>& selected,
                                            const std::function<void(std::string_view)>& visit) const
{
    if (!mBaselineInitialized || !mKnownSetAuthoritative)
        return false;
    for (const auto& path : selected)
        if (mKnownPhysicsPaths.count(path))
            visit(path);
    return true;
}
bool OvstageChangeFeed::restoreBaseline(const std::vector<std::string_view>& paths)
{
    if (mBaselineInitialized)
        return false;
    return restoreBaseline(prepareBaseline(paths));
}

template <typename Paths>
void OvstageChangeFeed::countTransformPaths(const Paths& physicsPaths, std::unordered_map<std::string, size_t>& counts)
{
    for (const auto& path : physicsPaths)
    {
        std::string current(path);
        while (!current.empty())
        {
            ++counts[current];
            if (current == "/")
                break;
            const size_t slash = current.find_last_of('/');
            if (slash == std::string::npos || slash == 0)
                current = "/";
            else
                current.resize(slash);
        }
    }
}

OvstageChangeFeed::PreparedBaseline OvstageChangeFeed::prepareBaseline(const std::vector<std::string_view>& paths)
{
    // The three sets depend on the paths alone; a large membership builds them side by side.
    // A path named twice invalidates the baseline whatever the other two hold.
    const auto launch = paths.size() > 4096 ? std::launch::async : std::launch::deferred;
    auto transforms = std::async(launch, [&paths]() {
        std::unordered_set<std::string> result;
        result.reserve(paths.size());
        for (const auto path : paths)
        {
            // This local set is ancestor-closed. An existing ancestor proves that
            // its entire chain is already present, so shared prefixes stop early.
            std::string ancestor(path);
            while (!ancestor.empty() && result.insert(ancestor).second)
            {
                if (ancestor == "/")
                    break;
                const auto slash = ancestor.find_last_of('/');
                if (slash == std::string::npos || slash == 0)
                    ancestor = "/";
                else
                    ancestor.resize(slash);
            }
        }
        return result;
    });
    auto counts = std::async(launch, [&paths]() {
        std::unordered_map<std::string, size_t> result;
        result.reserve(paths.size());
        countTransformPaths(paths, result);
        return result;
    });
    PreparedBaseline baseline;
    baseline.physics.reserve(paths.size());
    for (const auto path : paths)
        if (!baseline.physics.emplace(path).second)
            return baseline;
    baseline.transforms = transforms.get();
    baseline.transformRefCounts = counts.get();
    baseline.valid = true;
    return baseline;
}

bool OvstageChangeFeed::restoreBaseline(PreparedBaseline&& baseline)
{
    if (mBaselineInitialized || !baseline.valid)
        return false;
    mKnownPhysicsPaths = std::move(baseline.physics);
    mKnownTransformPaths = std::move(baseline.transforms);
    mTransformPathRefCounts = std::move(baseline.transformRefCounts);
    mKnownPhysicsQuery.noteReplaced();
    mKnownTransformQuery.noteReplaced();
    // Family queries are rebuilt lazily on the first update. Their initial
    // counts are deliberately not used as evidence about the cached baseline.
    mKnownSetAuthoritative = true;
    mBaselineInitialized = true;
    return true;
}

void OvstageChangeFeed::seedKnownPhysicsPaths()
{
    initialBaselineScans.fetch_add(1, std::memory_order_relaxed);
    if (!mInstance || !mDict)
        return;

    std::unordered_set<std::string> physicsPaths;
    std::unordered_set<std::string> transformPaths;
    const bool schemaOk = collectKnownPathsFromFilter("usd-schemas", OVSTAGE_FILTER_OP_CONTAINS, mPhysicsSchemas.data(),
                                                      mPhysicsSchemas.size(), mSchemaFamilyQuery,
                                                      physicsPaths, transformPaths);
    const bool typeOk = collectKnownPathsFromTypes(physicsPaths, transformPaths);

    mKnownPhysicsPaths = std::move(physicsPaths);
    mKnownTransformPaths = std::move(transformPaths);
    rebuildTransformPathRefCounts();
    mKnownPhysicsQuery.noteReplaced();
    mKnownTransformQuery.noteReplaced();
    // A failed scan leaves the sets partial with no other record; the next drain must then
    // rescan before its value and transform reads rather than extend them.
    mKnownSetAuthoritative = schemaOk && typeOk;
}

bool OvstageChangeFeed::collectKnownPathsFromTypes(std::unordered_set<std::string>& physicsPaths,
                                                  std::unordered_set<std::string>& transformPaths,
                                                  std::vector<ObjectKey>* newKeys,
                                                  const std::unordered_set<std::string>* oldPhysicsPaths)
{
    const std::optional<bool> authored = mSource.hasAuthoredTypeColumn();
    if (!authored.has_value())
        return false;
    // Recovery reads the latest sealed families. Initial seeding instead shares the
    // source's type index with the following descriptor scan, avoiding a second
    // enumeration of the same prims on OVStage 0.4.
    if (!*authored && oldPhysicsPaths)
        return collectKnownPathsFromFilter(conv::kUsdPrimType, OVSTAGE_FILTER_OP_IN,
            mPhysicsPrimTypes.data(), mPhysicsPrimTypes.size(), mTypeFamilyQuery,
            physicsPaths, transformPaths, newKeys, oldPhysicsPaths);

    // A built-in type filter cannot see an authored-type override, so use the same merged
    // type index as the walker. A recovery rescan needs a fresh snapshot because its range
    // may have skipped structural rows.
    if (*authored)
        mFamilyPrimCountReliable = false;
    else if (consumeKnownPathScanFaultForTest())
        return false;
    OvstageSource latest(mInstance, mDict, mSource.readOrdinal());
    const OvstageSource& types = oldPhysicsPaths ? latest : mSource;
    size_t indexedTypeCount = 0;
    for (const std::string& type : mPhysicsTypeNames)
    {
        std::vector<ObjectKey> keys;
        if (!types.collectPrimTypeKeys(type, keys))
            return false;
        indexedTypeCount += keys.size();
        for (const ObjectKey key : keys)
        {
            const std::string path(types.sourceKeyToString(key));
            if (path.empty())
                return false;
            const bool inserted = physicsPaths.insert(path).second;
            rememberTransformPathAndAncestors(transformPaths, path);
            if (inserted && newKeys && oldPhysicsPaths && !oldPhysicsPaths->count(path))
                newKeys->push_back(mSource.findByPath(path));
        }
    }
    if (!*authored)
    {
        // Keep the frozen family query and discovery used by later drains, but do
        // not read its payload again. Trust its count only after a successful fetch
        // agrees with the indexed baseline; otherwise the first drain must recover.
        const ovstage_query_handle_t query = queryForFilter(conv::kUsdPrimType, OVSTAGE_FILTER_OP_IN,
            mPhysicsPrimTypes.data(), mPhysicsPrimTypes.size(), mTypeFamilyQuery);
        if (query == OVSTAGE_INVALID_QUERY_HANDLE)
            return false;
        ovstage_query_result_t result{};
        if (ovstage_fetch_query_result(mInstance, query, OVSTAGE_TIMEOUT_INFINITE, &result) != OVSTAGE_OK)
            return false;
        const bool matches = result.total_prim_count == indexedTypeCount;
        mTypeFamilyQuery.primCount = result.total_prim_count;
        ovstage_release_query_result(mInstance, &result);
        if (!matches)
            return false;
    }
    return true;
}

bool OvstageChangeFeed::collectKnownPathsFromFilter(const char* attrName,
                                                    ovstage_filter_op_t op,
                                                    const ovx_string_t* values,
                                                    size_t valueCount,
                                                    CachedFilterQuery& query,
                                                    std::unordered_set<std::string>& physicsPaths,
                                                    std::unordered_set<std::string>& transformPaths,
                                                    std::vector<ObjectKey>* newKeys,
                                                    const std::unordered_set<std::string>* oldPhysicsPaths)
{
    if (!mInstance || !mDict || !attrName)
        return false;

    ovstage_query_handle_t q = queryForFilter(attrName, op, values, valueCount, query);
    if (q == OVSTAGE_INVALID_QUERY_HANDLE)
        return false;

    const ovx_token_t attrTok = internOvxToken(attrName);
    if (attrTok == OVX_INVALID_TOKEN)
        return false;
    if (consumeKnownPathScanFaultForTest())
        return false;

    ovstage_ordinal_range_t range{};
    range.end_ordinal = OVSTAGE_ORDINAL_MAX;
    range.has_start_ordinal = false;

    bool ok = true;
    size_t rowsSeen = 0;
    ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
    const ovstage_enqueue_result_t re = ovstage_read_attributes(mInstance, q, &attrTok, 1, range, &rh);
    if (re.status == OVSTAGE_OK)
    {
        waitAndRelease(mInstance, re);
        ReadListMemo listMemo(mDict);
        ReadColumnCursor cursor(mInstance, rh);
        ReadGroupColumn g{};
        ovstage_api_status_t fetchErr;
        while ((fetchErr = cursor.next(g)) == OVSTAGE_OK)
        {
            if (!g.is_delete)
            {
                rowsSeen += g.prims.count;
                std::vector<ObjectKey> groupNewKeys =
                    collectPathsFromGroup(g, listMemo, physicsPaths, transformPaths, oldPhysicsPaths);
                if (newKeys)
                    newKeys->insert(newKeys->end(), groupNewKeys.begin(), groupNewKeys.end());
            }
        }
        if (fetchErr != OVSTAGE_ERROR_END_OF_ITERATION)
            ok = false;
        waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
    }
    else
    {
        ok = false;
    }

    // The matched-prim count is the structural count check's only signal; distrust it
    // for good if the read shows prims the count denies.
    if (ok && rowsSeen > 0 && query.primCount == 0)
        mFamilyPrimCountReliable = false;

    return ok;
}

void OvstageChangeFeed::releaseCachedQuery(ovstage_query_handle_t& query)
{
    if (query == OVSTAGE_INVALID_QUERY_HANDLE || !mInstance)
        return;
    waitAndRelease(mInstance, ovstage_release_query(mInstance, query));
    query = OVSTAGE_INVALID_QUERY_HANDLE;
}

void OvstageChangeFeed::releaseCachedQuery(CachedFilterQuery& query)
{
    releaseCachedQuery(query.query);
    query.dirty = true;
    query.discoveredValueAttributes.clear();
    query.primCount = 0;
    mValueReadAttrsDirty = true;
}

void OvstageChangeFeed::releaseCachedQuery(PathListHandle& handle)
{
    releaseCachedQuery(handle.query);
    if (handle.list != OVX_INVALID_PRIMPATH_LIST && mDict)
        ovx_path_dictionary_destroy_path_list(mDict, handle.list);
    handle.list = OVX_INVALID_PRIMPATH_LIST;
    handle.paths.clear();
}

void OvstageChangeFeed::releaseCachedQuery(PathListQuery& query)
{
    releaseCachedQuery(query.base);
    releaseCachedQuery(query.delta);
    for (auto& segment : query.segments)
        releaseCachedQuery(segment);
    query.segments.clear();
    query.noteReplaced();
}

void OvstageChangeFeed::dirtyCachedFamilyQueries()
{
    releaseCachedQuery(mSchemaFamilyQuery);
    releaseCachedQuery(mTypeFamilyQuery);
}

bool OvstageChangeFeed::setDiscoveredValueAttributes(std::vector<ovx_token_t>& target,
                                                     const ovx_token_t* attrs,
                                                     size_t count)
{
    bool resolved = true;
    std::vector<ovx_token_t> fresh;
    fresh.reserve(count);
    std::unordered_set<ovx_token_t> seen;
    for (size_t i = 0; i < count; ++i)
    {
        ovx_string_t s{};
        if (ovx_path_dictionary_token_to_string(mDict, attrs[i], &s) != OVX_OK || !s.ptr || s.length == 0)
        {
            resolved = false;
            continue;
        }
        const std::string_view name(s.ptr, s.length);
        if (!isValueAttributeName(name))
            continue;
        if (seen.insert(attrs[i]).second)
            fresh.push_back(attrs[i]);
    }

    // A token that failed to resolve leaves `fresh` incomplete; never swap a partial set in. Leave the last
    // fully-resolved set standing and report failure, so no caller -- including one already on path-list
    // discovery -- classifies against an unverified set.
    if (!resolved)
        return false;

    // The refresh can run every drain; only a changed set may dirty the read list.
    bool same = fresh.size() == target.size();
    for (size_t i = 0; same && i < target.size(); ++i)
        same = seen.count(target[i]) != 0;
    if (same)
        return true;
    target.swap(fresh);
    mValueReadAttrsDirty = true;
    return true;
}

ovstage_query_handle_t OvstageChangeFeed::queryForFilter(const char* attrName,
                                                         ovstage_filter_op_t op,
                                                         const ovx_string_t* values,
                                                         size_t valueCount,
                                                         CachedFilterQuery& query)
{
    if (!query.dirty && query.query != OVSTAGE_INVALID_QUERY_HANDLE)
        return query.query;

    releaseCachedQuery(query.query);
    query.discoveredValueAttributes.clear();
    query.primCount = 0;
    query.dirty = false;
    mValueReadAttrsDirty = true;

    if (!mInstance || !mDict || !attrName)
        return OVSTAGE_INVALID_QUERY_HANDLE;

    ovstage_predicate_t pred{};
    pred.attribute.token = ovx_token_t{0};
    pred.attribute.string = ovxStr(attrName);
    pred.op = op;
    pred.values = values;
    pred.value_count = valueCount;

    ovstage_filter_t filter{};
    filter.predicates = &pred;
    filter.count = 1;

    ovstage_query_handle_t newQuery = OVSTAGE_INVALID_QUERY_HANDLE;
    const ovstage_enqueue_result_t qe = ovstage_query(mInstance, &filter, nullptr, 0, &newQuery);
    if (qe.status != OVSTAGE_OK || newQuery == OVSTAGE_INVALID_QUERY_HANDLE)
    {
        query.dirty = true;
        return OVSTAGE_INVALID_QUERY_HANDLE;
    }
    waitAndRelease(mInstance, qe);
    query.query = newQuery;

    ovstage_query_result_t qr{};
    if (ovstage_fetch_query_result(mInstance, query.query, OVSTAGE_TIMEOUT_INFINITE, &qr) == OVSTAGE_OK)
    {
        setDiscoveredValueAttributes(query.discoveredValueAttributes, qr.attributes, qr.attribute_count);
        query.primCount = qr.total_prim_count;
        ovstage_release_query_result(mInstance, &qr);
    }

    return query.query;
}

bool OvstageChangeFeed::executeFamilyQuery(const char* attrName,
                                           ovstage_filter_op_t op,
                                           const ovx_string_t* values,
                                           size_t valueCount,
                                           CachedFilterQuery& query,
                                           bool refreshDiscovery,
                                           size_t& primCount)
{
    primCount = 0;
    if (!mInstance || !mDict || !attrName)
        return false;

    ovstage_predicate_t pred{};
    pred.attribute.token = ovx_token_t{0};
    pred.attribute.string = ovxStr(attrName);
    pred.op = op;
    pred.values = values;
    pred.value_count = valueCount;

    ovstage_filter_t filter{};
    filter.predicates = &pred;
    filter.count = 1;

    // When only the count is wanted, scope discovery to one built-in column so the
    // query does not pay for enumerating every attribute of every matched prim.
    const ovx_token_t scope = internOvxToken(conv::kUsdPrimType);
    const bool scoped = !refreshDiscovery && scope != OVX_INVALID_TOKEN;

    ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
    const ovstage_enqueue_result_t qe =
        ovstage_query(mInstance, &filter, scoped ? &scope : nullptr, scoped ? 1 : 0, &q);
    if (qe.status != OVSTAGE_OK || q == OVSTAGE_INVALID_QUERY_HANDLE)
        return false;
    waitAndRelease(mInstance, qe);

    ovstage_query_result_t qr{};
    const bool ok = ovstage_fetch_query_result(mInstance, q, OVSTAGE_TIMEOUT_INFINITE, &qr) == OVSTAGE_OK;
    if (ok)
    {
        primCount = qr.total_prim_count;
        if (refreshDiscovery)
            setDiscoveredValueAttributes(query.discoveredValueAttributes, qr.attributes, qr.attribute_count);
        ovstage_release_query_result(mInstance, &qr);
    }
    releaseCachedQuery(q);
    return ok;
}

bool OvstageChangeFeed::familyMembershipUnchanged(bool& unchanged)
{
    unchanged = false;
    if (!mFamilyPrimCountReliable || !mFamilyCountsCurrent ||
        mSchemaFamilyQuery.query == OVSTAGE_INVALID_QUERY_HANDLE ||
        mTypeFamilyQuery.query == OVSTAGE_INVALID_QUERY_HANDLE)
        return true;

    size_t count = 0;
    // Count only: attribute discovery is refreshed per drain from the known-physics
    // path-list query instead (readValueChanges), which is far cheaper than
    // enumerating every attribute of every matched prim here.
    if (!executeFamilyQuery("usd-schemas", OVSTAGE_FILTER_OP_CONTAINS, mPhysicsSchemas.data(), mPhysicsSchemas.size(),
                            mSchemaFamilyQuery, /*refreshDiscovery=*/false, count))
        return false;
    const bool schemaSame = count == mSchemaFamilyQuery.primCount;
    if (!executeFamilyQuery(conv::kUsdPrimType, OVSTAGE_FILTER_OP_IN, mPhysicsPrimTypes.data(), mPhysicsPrimTypes.size(),
                            mTypeFamilyQuery, /*refreshDiscovery=*/false, count))
        return false;
    unchanged = schemaSame && count == mTypeFamilyQuery.primCount;
    return true;
}

bool OvstageChangeFeed::refreshDiscoveryFromQuery(const PathListQuery& query)
{
    // Union all immutable segments. A fetch failure leaves the previous
    // discovery standing rather than replacing it with a partial vocabulary.
    std::vector<ovx_token_t> attrs;
    for (const PathListHandle* h : query.handles())
    {
        if (!h->valid())
            continue;
        ovstage_query_result_t qr{};
        if (ovstage_fetch_query_result(mInstance, h->query, OVSTAGE_TIMEOUT_INFINITE, &qr) != OVSTAGE_OK)
            return false;
        if (qr.attribute_count > 0 && qr.attributes)
            attrs.insert(attrs.end(), qr.attributes, qr.attributes + qr.attribute_count);
        ovstage_release_query_result(mInstance, &qr);
    }
    if (attrs.empty())
        return false;
    // Dictionary tokens have immutable names. Reuse classification only after every
    // token resolved successfully, and only for the identical fresh metadata sequence.
    if (mUsePathListDiscovery && attrs == mLastPathListAttributes)
        return true;
    // A token that failed to resolve makes setDiscoveredValueAttributes return false WITHOUT touching
    // mPathListDiscoveredAttrs, so an attachment already on path-list discovery keeps classifying against the
    // last fully-resolved set. Drop our sequence record and report failure.
    if (!setDiscoveredValueAttributes(mPathListDiscoveredAttrs, attrs.data(), attrs.size()))
    {
        mLastPathListAttributes.clear();
        return false;
    }
    mLastPathListAttributes = std::move(attrs);
    if (!mUsePathListDiscovery)
    {
        mUsePathListDiscovery = true;
        mValueReadAttrsDirty = true;
    }
    return true;
}

OvstageChangeFeed::PathListHandle OvstageChangeFeed::buildPathListQuery(const std::vector<const std::string*>& paths)
{
    std::vector<ovx_string_t> views;
    views.reserve(paths.size());
    for (const std::string* p : paths)
        views.push_back(ovx_string_t{ p->data(), p->size() }); // views into set-owned strings

    PathListHandle handle;
    if (ovx_path_dictionary_create_path_list_from_strings(mDict, views.data(), views.size(), &handle.list) != OVX_OK)
        return {};
    if (ovstage_query_from_path_list(mInstance, handle.list, &handle.query) != OVSTAGE_OK)
    {
        releaseCachedQuery(handle);
        return {};
    }
    // Captured once: the list is immutable, and every group the query returns over it
    // addresses rows by index into exactly this sequence.
    const ovx_primpath_t* listPaths = nullptr;
    size_t listCount = 0;
    if (ovx_path_dictionary_get_paths(mDict, handle.list, &listPaths, &listCount) == OVX_OK && listPaths)
        handle.paths.assign(listPaths, listPaths + listCount);
    return handle;
}

bool OvstageChangeFeed::syncPathListQuery(const std::unordered_set<std::string>& paths, PathListQuery& query, bool allowDeferredAdds)
{
    if (!query.dirty)
        return true;
    if (!mInstance || !mDict)
        return false;
    // Readers return before syncing an empty set, so `paths` is non-empty here.

    const size_t threshold = std::max<size_t>(8, static_cast<size_t>(std::sqrt(2.0 * paths.size())));

    // A full structural parse owns initial values for accepted adds. Avoid constructing
    // an unused delta when the base still addresses every older path. Keep the pending
    // paths dirty: the next structural read must materialize them, even after deletion.
    // This applies to large batches too: folding them now would rebuild the entire
    // watched set and read initial values that the structural load already owns.
    if (allowDeferredAdds && mAddedPropertiesCovered && !query.fold && query.base.valid() && !query.deltaPaths.empty())
    {
        const std::unordered_set<std::string>& added =
            &query == &mKnownPhysicsQuery ? mAddedPhysicsPaths : mAddedTransformPaths;
        if (std::all_of(query.deltaPaths.begin(), query.deltaPaths.end(),
                        [&](const std::string& path) { return added.count(path) != 0; }))
        {
            releaseCachedQuery(query.delta);
            return true;
        }
    }

    // Small additions share a cheap delta, then fold into the base. Retaining
    // small segments saves path-list construction but adds read calls on every
    // later drain. Only larger batches amortize that cost through segmentation.
    // A delta-only removal preserves the older immutable segments.
    if (query.fold || (query.deltaPaths.size() > threshold &&
                      (query.deltaPaths.size() < PathListQuery::kMinSegmentPaths ||
                       query.segments.size() >= PathListQuery::kMaxSegments)))
    {
        query.fold = true; // a failed fold must retry the complete surviving set
        for (auto& segment : query.segments)
            releaseCachedQuery(segment);
        query.segments.clear();
        releaseCachedQuery(query.base);
        releaseCachedQuery(query.delta);
        std::vector<const std::string*> all;
        all.reserve(paths.size());
        for (const std::string& p : paths)
            all.push_back(&p);
        query.base = buildPathListQuery(all);
        if (!query.base.valid())
            return false; // stays dirty and folding: retried next drain, this drain fails
        query.deltaPaths.clear();
        query.fold = false;
        query.dirty = false;
        return true;
    }

    // Older segments stay intact: rebuild only the pending additions.
    releaseCachedQuery(query.delta);
    if (!query.deltaPaths.empty())
    {
        std::vector<const std::string*> added;
        added.reserve(query.deltaPaths.size());
        for (const std::string& p : query.deltaPaths)
            added.push_back(&p);
        query.delta = buildPathListQuery(added);
        if (!query.delta.valid())
            return false; // stays dirty
    }
    if (query.deltaPaths.size() > threshold)
    {
        // Keep a large immutable addition batch instead of rebuilding every
        // older path at each spawn. The cap bounds read fanout; removals and
        // replacements still fold the complete surviving set above.
        query.segments.push_back(std::move(query.delta));
        query.delta = {};
        query.deltaPaths.clear();
    }
    query.dirty = false;
    return true;
}

template <class ReadFn>
bool OvstageChangeFeed::readPathListQuery(const PathListQuery& query,
                                          const std::vector<ovx_token_t>& attrs,
                                          uint64_t ord0,
                                          ReadFn read, bool omitAdded)
{
    std::vector<ovx_token_t> sealed;
    bool sealedKnown = false;
    std::string path; // reused, so a lookup allocates only for a longer path
    for (const PathListHandle* h : query.handles())
    {
        if (!h->valid())
            continue;
        if (omitAdded && mAddedPropertiesCovered)
        {
            const std::unordered_set<std::string>& added =
                &query == &mKnownPhysicsQuery ? mAddedPhysicsPaths : mAddedTransformPaths;
            bool allAdded = !h->paths.empty() && !added.empty();
            for (ovx_primpath_t raw : h->paths)
            {
                if (!allAdded)
                    break;
                path.assign(mSource.sourceKeyToString(mSource.internKey(raw.raw_id)));
                if (added.count(path) == 0)
                {
                    allAdded = false;
                    break;
                }
            }
            if (allAdded)
                continue;
        }
        ovstage_api_status_t err = OVSTAGE_OK;
        if (read(*h, attrs, err))
            continue;
        // A synchronous read failure must fail the drain: otherwise updateFromOvStage()
        // advances the external cursor past changes that were never read.
        if (err != OVSTAGE_ERROR_WRITE_FLOOR_VIOLATION && err != OVSTAGE_ERROR_OP_FAILED)
            return false;
        // A column written in the range but not sealed vetoes this handle. Retry it with
        // the columns the producer has actually published (floor at or past the range);
        // handles that already delivered are not re-read.
        if (!sealedKnown)
        {
            for (ovx_token_t tok : attrs)
                if (hasAttributeFloorAtOrAfter(tok, ord0))
                    sealed.push_back(tok);
            sealedKnown = true;
        }
        if (sealed.empty())
            continue;
        if (!read(*h, sealed, err))
            return false;
    }
    return true;
}

ovx_token_t OvstageChangeFeed::internOvxToken(std::string_view attrName) const
{
    if (!mDict || attrName.empty())
        return OVX_INVALID_TOKEN;
    ovx_token_t tok = OVX_INVALID_TOKEN;
    const ovx_string_t s{ attrName.data(), attrName.size() };
    if (ovx_path_dictionary_intern_token(mDict, s, &tok) != OVX_OK)
        return OVX_INVALID_TOKEN;
    return tok;
}

ObjectKey OvstageChangeFeed::resolveGroupPathKey(ovx_primpath_t raw)
{
    if (raw == OVX_INVALID_PRIMPATH)
        return {};
    const auto it = mResolvedPathKeys.find(raw.raw_id);
    if (it != mResolvedPathKeys.end())
        return it->second;

    ObjectKey key = mSource.canonicalKey(mSource.internKey(raw.raw_id));
    if (!key.valid())
        key = mSource.internKey(raw.raw_id);
    mResolvedPathKeys[raw.raw_id] = key;
    if (key.valid())
    {
        // Also index by the canonical RAW handle (not key.handle, which is now
        // the packed generation-tagged form -- ADR-0021) so a later call already
        // holding that raw value hits this same cache entry.
        if (const uint64_t canonicalRaw = mSource.canonicalPath(key).raw_id)
            mResolvedPathKeys[canonicalRaw] = key;
    }
    return key;
}

TokenId OvstageChangeFeed::propertyTokenFor(ovx_token_t attr)
{
    const auto it = mPropertyTokens.find(attr);
    if (it != mPropertyTokens.end())
        return it->second;

    TokenId property{};
    ovx_string_t attrStr{};
    if (ovx_path_dictionary_token_to_string(mDict, attr, &attrStr) == OVX_OK && attrStr.ptr)
        property = mSource.internToken(conv::toUsdAttributeName(std::string_view(attrStr.ptr, attrStr.length)));
    mPropertyTokens.emplace(attr, property);
    return property;
}

void OvstageChangeFeed::collectGroupKeys(const ReadGroupColumn& group,
                                         std::vector<ObjectKey>& keys,
                                         const PathListHandle* handle,
                                         bool skipMaskedRows,
                                         std::vector<uint32_t>* keptRows)
{
    keys.clear();
    if (keptRows)
        keptRows->clear();
    mGroupListPaths = nullptr;
    mGroupListCount = 0;
    const ovx_primpath_t* gpaths = nullptr;
    size_t gcount = 0;
    if (handle && group.prims.list == handle->list)
    {
        gpaths = handle->paths.data();
        gcount = handle->paths.size();
    }
    else
    {
        // Every group of a change-range read shares ovstage's own copy of the query list,
        // entry for entry identical to `handle->paths`, so a list of the same size is indexed
        // through our captured entries; any other list is fetched from the dictionary once per
        // read. Either answer is memoised per list handle until the held groups are released.
        auto memo = mListMemo.find(group.prims.list);
        if (memo == mListMemo.end())
        {
            ListMemo entry;
            size_t listCount = 0;
            const bool counted = ovstage_compat::status(mDict, path_dictionary_get_num_paths_from_path_list(
                                                                   mDict, group.prims.list, &listCount)) == OVX_API_SUCCESS;
            if (handle && counted && listCount == handle->paths.size())
            {
                entry.sameAsHandle = true;
            }
            else
            {
                if (ovx_path_dictionary_get_paths(mDict, group.prims.list, &gpaths, &gcount) != OVX_OK || !gpaths)
                    return;
                entry.paths.assign(gpaths, gpaths + gcount);
            }
            memo = mListMemo.emplace(group.prims.list, std::move(entry)).first;
        }
        if (memo->second.sameAsHandle)
        {
            gpaths = handle->paths.data();
            gcount = handle->paths.size();
        }
        else
        {
            gpaths = memo->second.paths.data();
            gcount = memo->second.paths.size();
        }
    }
    mGroupListPaths = gpaths;
    mGroupListCount = gcount;

    keys.reserve(group.prims.count);
    const uint64_t* const mask = skipMaskedRows ? group.data.mask : nullptr;
    for (uint32_t i = 0; i < group.prims.count; ++i)
    {
        // Same bit layout as ReadColumn::present: one bit per logical row, set when the row carries a value.
        if (mask && (mask[i / 64] & (uint64_t(1) << (i % 64))) == 0)
            continue;
        const uint32_t idx = group.prims.index_map ? group.prims.index_map[i] : (group.prims.offset + i);
        // Skip out-of-range and null-primpath rows symmetrically with
        // seedBucketFromReadGroup, which skips raw==0 before advancing its key
        // ordinal; pushing a key for a raw==0 row here would misalign every
        // subsequent key with the seeded tensor row (issue #8).
        if (idx >= gcount || gpaths[idx] == OVX_INVALID_PRIMPATH)
            continue;
        keys.push_back(resolveGroupPathKey(gpaths[idx]));
        if (keptRows)
            keptRows->push_back(i);
    }
}

void OvstageChangeFeed::rebuildRegisteredAttributeTokens()
{
    if (!mRegisteredAttributeTokensDirty)
        return;

    mRegisteredAttributeTokens.clear();
    mHasWildcardRegistration = false;
    if (std::any_of(mRegistrations.begin(), mRegistrations.end(),
                    [](const Registration& reg) { return !reg.objectType.valid() && !reg.prop.valid(); }))
    {
        mHasWildcardRegistration = true;
        mRegisteredAttributeTokensDirty = false;
        return;
    }

    std::unordered_set<ovx_token_t> seen;
    for (const Registration& reg : mRegistrations)
    {
        if (!reg.prop.valid())
            continue;
        const std::string_view propName = mSource.tokenToString(reg.prop);
        if (propName.empty())
            continue;
        const ovx_token_t propTok = internOvxToken(propName);
        if (propTok != OVX_INVALID_TOKEN && seen.insert(propTok).second)
            mRegisteredAttributeTokens.push_back(propTok);
    }

    mRegisteredAttributeTokensDirty = false;
}

void OvstageChangeFeed::rebuildValueReadAttributes()
{
    rebuildRegisteredAttributeTokens();

    std::unordered_set<ovx_token_t> registered;
    if (!mHasWildcardRegistration)
        registered.insert(mRegisteredAttributeTokens.begin(), mRegisteredAttributeTokens.end());
    // Transform columns belong to readKnownTransformChanges, whose path set is a
    // superset of the physics prims; reading them here too delivered each moved body twice.
    std::unordered_set<ovx_token_t> seen(transformAttrTokens().begin(), transformAttrTokens().end());
    // The resolved world matrix is derived hierarchy output, never a value input.
    seen.insert(internOvxToken(conv::kFabricWorldMatrix));
    mValueReadAttrs.clear();
    auto addAttr = [&](ovx_token_t tok)
    {
        if (tok != OVX_INVALID_TOKEN && (mHasWildcardRegistration || registered.count(tok)) && seen.insert(tok).second)
            mValueReadAttrs.push_back(tok);
    };

    // Union of every discovery source: the family-filter discovery (documented,
    // populated when a family query is built) plus the per-drain path-list discovery
    // (catches a column first authored at runtime). addAttr dedups. A superset of the
    // trunk read set, so no consumer-relied attribute is ever dropped.
    for (ovx_token_t tok : mSchemaFamilyQuery.discoveredValueAttributes)
        addAttr(tok);
    for (ovx_token_t tok : mTypeFamilyQuery.discoveredValueAttributes)
        addAttr(tok);
    for (ovx_token_t tok : mPathListDiscoveredAttrs)
        addAttr(tok);

    mValueReadAttrsDirty = false;
}

std::vector<ObjectKey> OvstageChangeFeed::collectPathsFromGroup(
    const ReadGroupColumn& group,
    ReadListMemo& listMemo,
    std::unordered_set<std::string>& physicsPaths,
    std::unordered_set<std::string>& transformPaths,
    const std::unordered_set<std::string>* oldPhysicsPaths)
{
    std::vector<ObjectKey> newKeys;
    const ovx_primpath_t* paths = nullptr;
    size_t pathCount = 0;
    if (!listMemo.paths(group.prims.list, &paths, &pathCount))
        return newKeys;

    for (uint32_t i = 0; i < group.prims.count; ++i)
    {
        const uint32_t idx = group.prims.index_map ? group.prims.index_map[i] : (group.prims.offset + i);
        if (idx >= pathCount)
            continue;
        const ovx_primpath_t raw = paths[idx];
        const std::string_view path = mSource.sourceKeyToString(mSource.internKey(raw.raw_id));
        if (path.empty())
            continue;

        std::string pathString(path);
        const bool inserted = physicsPaths.insert(pathString).second;
        rememberTransformPathAndAncestors(transformPaths, pathString);
        if (inserted && oldPhysicsPaths && oldPhysicsPaths->find(pathString) == oldPhysicsPaths->end())
            newKeys.push_back(resolveGroupPathKey(raw));
    }
    return newKeys;
}

bool OvstageChangeFeed::hasAttributeFloorAtOrAfter(ovx_token_t attr, uint64_t ordinal) const
{
    if (attr == OVX_INVALID_TOKEN || !mInstance)
        return true;

    ovstage_ordinal_query_handle_t q = OVSTAGE_INVALID_ORDINAL_QUERY_HANDLE;
    const ovstage_enqueue_result_t re = ovstage_get_attribute_write_floor(mInstance, ovx_token_or_string_t{ attr, {} }, &q);
    if (re.status != OVSTAGE_OK || q == OVSTAGE_INVALID_ORDINAL_QUERY_HANDLE)
        return true;
    waitAndRelease(mInstance, re);

    ovstage_ordinal_t floor = 0;
    bool ok = ovstage_fetch_ordinal(mInstance, q, OVSTAGE_TIMEOUT_INFINITE, &floor) == OVSTAGE_OK;
    waitAndRelease(mInstance, ovstage_release_ordinal_query(mInstance, q));
    // Compare as ovstage ordinals: the floor of an attribute no seal reached yet is
    // OVSTAGE_ORDINAL_MIN, which an unsigned comparison would read as the largest ordinal.
    return !ok || floor >= static_cast<ovstage_ordinal_t>(ordinal);
}

bool OvstageChangeFeed::rangeWithinRetainedHistory(uint64_t ord0) const
{
    if (!mInstance)
        return false;
    ovstage_ordinal_query_handle_t q = OVSTAGE_INVALID_ORDINAL_QUERY_HANDLE;
    const ovstage_enqueue_result_t re = ovstage_get_oldest_preserved_ordinal(mInstance, &q);
    if (re.status != OVSTAGE_OK || q == OVSTAGE_INVALID_ORDINAL_QUERY_HANDLE)
        return false;
    waitAndRelease(mInstance, re);

    ovstage_ordinal_t oldest = 0;
    const bool ok = ovstage_fetch_ordinal(mInstance, q, OVSTAGE_TIMEOUT_INFINITE, &oldest) == OVSTAGE_OK;
    waitAndRelease(mInstance, ovstage_release_ordinal_query(mInstance, q));
    // Until retention discards history the frontier is OVSTAGE_ORDINAL_MIN: compare as ovstage
    // ordinals, or the unsigned comparison fails every drain closed.
    return ok && static_cast<ovstage_ordinal_t>(ord0) >= oldest;
}

bool OvstageChangeFeed::structuralAttributesChanged(uint64_t ordinal)
{
    const ovx_token_t schemasTok = internOvxToken(conv::kUsdSchemas);
    const ovx_token_t primTypeTok = internOvxToken(conv::kUsdPrimType);
    const ovx_token_t authoredTypeTok = internOvxToken(conv::kUsdTypeName);
    return (authoredTypeTok != OVX_INVALID_TOKEN && hasAttributeFloorAtOrAfter(authoredTypeTok, ordinal)) ||
           (schemasTok != OVX_INVALID_TOKEN && hasAttributeFloorAtOrAfter(schemasTok, ordinal)) ||
           (primTypeTok != OVX_INVALID_TOKEN && hasAttributeFloorAtOrAfter(primTypeTok, ordinal));
}

void OvstageChangeFeed::rebuildTransformPathRefCounts()
{
    mTransformPathRefCounts.clear();
    countTransformPaths(mKnownPhysicsPaths, mTransformPathRefCounts);
}

bool OvstageChangeFeed::rememberTransformPathAndAncestors(std::string_view path)
{
    bool changed = false;
    std::string current(path);
    while (!current.empty())
    {
        if (++mTransformPathRefCounts[current] == 1)
        {
            mKnownTransformPaths.insert(current);
            mKnownTransformQuery.noteAdded(current);
            if (mCollectAddedPaths)
                mAddedTransformPaths.insert(current);
            changed = true;
        }
        if (current == "/")
            break;
        const size_t slash = current.find_last_of('/');
        if (slash == std::string::npos || slash == 0)
            current = "/";
        else
            current.resize(slash);
    }
    return changed;
}

bool OvstageChangeFeed::rememberTransformPathAndAncestors(std::unordered_set<std::string>& paths,
                                                          std::string_view path)
{
    if (path.empty())
        return false;

    bool changed = false;
    std::string cur(path);
    while (!cur.empty())
    {
        if (paths.insert(cur).second)
        {
            changed = true;
        }
        if (cur == "/")
            break;

        const size_t slash = cur.find_last_of('/');
        if (slash == std::string::npos || slash == 0)
            cur = "/";
        else
            cur.resize(slash);
    }
    return changed;
}

bool OvstageChangeFeed::buildKnownFamilyTokens()
{
    if (mKnownFamilyTokensBuilt)
        return true;
    mKnownSchemaTokens.clear();
    mKnownTypeTokens.clear();
    for (size_t i = 0; i < mPhysicsSchemas.size(); ++i)
    {
        const ovx_token_t tok = internOvxToken({ mPhysicsSchemas[i].ptr, mPhysicsSchemas[i].length });
        if (tok != OVX_INVALID_TOKEN)
            mKnownSchemaTokens.push_back(tok);
    }
    for (size_t i = 0; i < mPhysicsPrimTypes.size(); ++i)
    {
        const ovx_token_t tok = internOvxToken({ mPhysicsPrimTypes[i].ptr, mPhysicsPrimTypes[i].length });
        if (tok != OVX_INVALID_TOKEN)
            mKnownTypeTokens.push_back(tok);
    }
    std::sort(mKnownSchemaTokens.begin(), mKnownSchemaTokens.end());
    std::sort(mKnownTypeTokens.begin(), mKnownTypeTokens.end());
    mKnownFamilyTokensBuilt =
        mKnownSchemaTokens.size() == mPhysicsSchemas.size() && mKnownTypeTokens.size() == mPhysicsPrimTypes.size();
    return mKnownFamilyTokensBuilt;
}

void OvstageChangeFeed::classifyStructuralGroup(const ReadGroupColumn& g,
                                                ReadListMemo& listMemo,
                                                ovx_token_t schemasTok,
                                                ovx_token_t primTypeTok,
                                                std::unordered_map<std::string, size_t>& rowIndex,
                                                StructuralRows& out,
                                                bool knownRowsExpected)
{
    const bool authoredType = g.attribute == internOvxToken(conv::kUsdTypeName);
    const ovx_primpath_t* gpaths = nullptr;
    size_t gcount = 0;
    // Filter reads often return a separate list for each one-row type group.
    // The group already pins its list: read its slice directly, avoiding a
    // second pin, allocation and full-list fetch for every individual prim.
    std::vector<ovx_primpath_t> slice;
    ovx_primpath_t single = OVX_INVALID_PRIMPATH;
    if (!g.prims.index_map)
    {
        if (g.prims.count > 1)
            slice.resize(g.prims.count);
        auto* destination = g.prims.count == 1 ? &single : slice.data();
        ovstage_compat::pathListFetchCounter().fetch_add(1, std::memory_order_relaxed);
        if (ovstage_compat::status(
                mDict, path_dictionary_get_paths_from_path_list(
                           mDict, g.prims.list, g.prims.offset, g.prims.count, destination, &gcount)) != OVX_OK ||
            gcount != g.prims.count)
        {
            out.exact = false;
            return;
        }
        gpaths = destination;
    }
    else if (!listMemo.paths(g.prims.list, &gpaths, &gcount))
    {
        out.exact = false;
        return;
    }

    auto pathAt = [&](uint32_t row, ovx_primpath_t& raw) -> std::string_view {
        const uint32_t idx = g.prims.index_map ? g.prims.index_map[row] : row;
        raw = idx < gcount ? gpaths[idx] : OVX_INVALID_PRIMPATH;
        if (idx >= gcount)
            out.exact = false;
        return raw != OVX_INVALID_PRIMPATH ? mSource.sourceKeyToString(mSource.internKey(raw.raw_id)) : std::string_view{};
    };

    // A tombstone on a known path contradicts readKnownStructuralChanges having seen nothing
    // (unless that read saw it: known-rows mode records it for the removal patch); one on an
    // unfamiliar prim is not a family change (as before), but its schemas are gone (prim or
    // column deleted), which the schema patch must learn. The type is forgotten only when its
    // own column is the tombstoned one: the type index (collectPrimTypeKeys) is trusted as
    // complete, so forgetting it for another column's tombstone would drop a live typed prim
    // from every later scan. A whole-prim delete of a known prim is removed exactly through
    // `tombstoned` (applyPrimRemoval) instead.
    if (g.is_delete)
    {
        for (uint32_t row = 0; row < g.prims.count; ++row)
        {
            ovx_primpath_t raw = OVX_INVALID_PRIMPATH;
            const std::string_view path = pathAt(row, raw);
            if (path.empty())
                continue;
            const bool known = mKnownPhysicsPaths.count(std::string(path)) != 0;
            // Removing an override can reveal a physics type on a previously
            // unwatched prim. Rebuild membership from the final effective types.
            if (authoredType && !known)
                out.exact = false;
            if (known && !knownRowsExpected)
            {
                out.exact = false;
                continue;
            }
            if (known)
                out.tombstoned.push_back(raw);
            if (g.attribute != primTypeTok)
                out.schemas.push_back({ raw, {} });
            if (g.attribute == primTypeTok)
                out.types.push_back({ raw, 0, authoredType });
        }
        return;
    }

    // Both columns are requested, so any other attribute is unexpected; its rows still
    // count for the hierarchy delta but cannot be classified.
    const bool knownColumn = g.attribute == schemasTok || g.attribute == primTypeTok;
    const bool decodable = knownColumn && g.data.mask == nullptr && g.data.tensor_count > 0 && g.data.tensors &&
                           !(g.data.index_map && g.data.count < g.prims.count);
    for (uint32_t row = 0; row < g.prims.count; ++row)
    {
        ovx_primpath_t raw = OVX_INVALID_PRIMPATH;
        const std::string_view path = pathAt(row, raw);
        if (raw == OVX_INVALID_PRIMPATH)
            continue;
        out.added.push_back(raw);
        if (path.empty())
        {
            out.exact = false;
            continue;
        }
        std::string pathString(path);
        const bool known = mKnownPhysicsPaths.count(pathString) != 0;
        if (known && !knownRowsExpected)
        {
            out.exact = false; // the known-path read reported no change for this prim
            continue;
        }
        if (!decodable)
        {
            out.exact = false;
            continue;
        }

        bool hit = false;
        bool ok;
        if (g.attribute == primTypeTok)
        {
            uint64_t value = 0;
            ok = decodeTypeRow(g, row, value);
            if (ok)
                out.types.push_back({ raw, value, authoredType });
            if (ok && !authoredType && mSource.physicsInputSharingEnabled() &&
                g.meta.attribute_write_floor_ordinal >= mSource.readOrdinal())
            {
                ReadColumn column;
                // Preserve the exact fingerprint representation, not the effective
                // type cache (which may contain an authored typeName override).
                if (readColumn(g, column) && column.components == 1 && column.tensor->dtype.code == kDLUInt &&
                    column.tensor->dtype.bits == 64 && column.tensor->dtype.lanes == 1)
                    out.sharedTypes.push_back({ raw.raw_id, value, g.semantic });
            }
            hit = ok && std::binary_search(mKnownTypeTokens.begin(), mKnownTypeTokens.end(), ovx_token_t{value});
        }
        else
        {
            SchemaRowDelta& schemaRow = out.schemas.emplace_back();
            schemaRow.raw = raw;
            ok = forEachSchemaToken(g, row,
                                    [&](uint64_t value)
                                    {
                                        schemaRow.schemaTokens.push_back(value);
                                        hit = hit ||
                                              std::binary_search(mKnownSchemaTokens.begin(), mKnownSchemaTokens.end(), ovx_token_t{value});
                                    });
        }
        if (!ok)
        {
            out.exact = false;
            continue;
        }
        if (known)
            continue; // a memo patch input only; the known sets already hold the prim

        const auto inserted = rowIndex.emplace(pathString, out.unfamiliar.size());
        if (inserted.second)
        {
            StructuralRows::Row& fresh = out.unfamiliar.emplace_back();
            fresh.raw = raw;
            fresh.path = std::move(pathString);
        }
        StructuralRows::Row& r = out.unfamiliar[inserted.first->second];
        if (g.attribute == primTypeTok)
        {
            if (authoredType || !r.hasAuthoredType)
                r.byType = hit;
            r.hasAuthoredType = r.hasAuthoredType || authoredType;
        }
        else
            r.bySchemas = r.bySchemas || hit;
    }
}

bool OvstageChangeFeed::readStructuralRows(uint64_t ord0, uint64_t ord1, StructuralRows& out, bool knownRowsExpected)
{
    out.clear();
    if (!mInstance || !mDict || !buildKnownFamilyTokens())
        return false;
    if (consumeFault(structuralRowsReadFaultCounter()))
        return false;

    std::array<ovx_token_t, 3> attrToks{};
    size_t attrCount = 0;
    const ovx_token_t schemasTok = internOvxToken(conv::kUsdSchemas);
    const ovx_token_t primTypeTok = internOvxToken(conv::kUsdPrimType);
    if (schemasTok != OVX_INVALID_TOKEN)
        attrToks[attrCount++] = schemasTok;
    if (primTypeTok != OVX_INVALID_TOKEN)
        attrToks[attrCount++] = primTypeTok;
    const ovx_token_t authoredTypeTok = internOvxToken(conv::kUsdTypeName);
    if (authoredTypeTok != OVX_INVALID_TOKEN)
        attrToks[attrCount++] = authoredTypeTok;
    if (attrCount == 0)
        return false;

    // Every prim carries the auto-maintained usd-path column, so HAS on it matches the whole
    // stage. HAS, not PREFIX "/": a range read over a value-predicate query returns every prim's
    // rows, not the range's (1000 rows stringified per value-only drain on a 1000-body stage).
    // Rebuilt each call: a filter query is frozen at build, so a cached one would never contain
    // the prims we are here to detect.
    ovstage_predicate_t pred{};
    pred.attribute.token = ovx_token_t{0};
    pred.attribute.string = ovxStr(conv::kUsdPath);
    pred.op = OVSTAGE_FILTER_OP_HAS;
    pred.values = nullptr;
    pred.value_count = 0;
    ovstage_filter_t filter{};
    const bool prepareInputs = mSource.physicsInputSharingEnabled();
    // Unfiltered membership also supports exact ordinal ranges, and supplies
    // the complete vocabulary (HAS discovery omits relationship columns).
    // Cache-enabled insertion shares this query's discovery with fingerprinting.
    if (!prepareInputs)
    {
        filter.predicates = &pred;
        filter.count = 1;
    }

    ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
    const ovstage_enqueue_result_t qe = ovstage_query(mInstance, &filter, nullptr, 0, &q);
    if (qe.status != OVSTAGE_OK || q == OVSTAGE_INVALID_QUERY_HANDLE)
        return false;
    waitAndRelease(mInstance, qe);
    ovstage_ordinal_range_t range{};
    range.start_ordinal = ord0;
    range.end_ordinal = ord1;
    range.has_start_ordinal = true;

    bool ok = true;
    ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
    const ovstage_enqueue_result_t re = ovstage_read_attributes(mInstance, q, attrToks.data(), attrCount, range, &rh);
    if (re.status == OVSTAGE_OK)
    {
        waitAndRelease(mInstance, re);
        // Every live row is a hierarchy-delta member; the ones on unfamiliar paths are
        // classified against the family token sets (known-prim structural changes were
        // already seen by readKnownStructuralChanges). One release per group, whatever
        // the classifier decided.
        std::unordered_map<std::string, size_t> rowIndex;
        ReadListMemo listMemo(mDict);
        ReadColumnCursor cursor(mInstance, rh);
        ReadGroupColumn g{};
        ovstage_api_status_t fetchErr;
        while ((fetchErr = cursor.next(g)) == OVSTAGE_OK)
        {
            out.rowCount += g.prims.count;
            structuralRowsReadCounter().fetch_add(g.prims.count, std::memory_order_relaxed);
            classifyStructuralGroup(g, listMemo, schemasTok, g.attribute == authoredTypeTok ? authoredTypeTok : primTypeTok, rowIndex, out, knownRowsExpected);
        }
        if (fetchErr != OVSTAGE_ERROR_END_OF_ITERATION)
            ok = false;
        waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
    }
    else
    {
        ok = false;
    }
    if (prepareInputs && ok && out.exact && !out.unfamiliar.empty() && !out.sharedTypes.empty())
    {
        ovstage_query_result_t result{};
        if (ovstage_fetch_query_result(mInstance, q, OVSTAGE_TIMEOUT_INFINITE, &result) == OVSTAGE_OK)
        {
            bool complete = result.attribute_count == 0 || result.attributes;
            std::vector<std::string> vocabulary;
            vocabulary.reserve(result.attribute_count);
            for (size_t i = 0; i < result.attribute_count && result.attributes; ++i)
            {
                ovx_string_t text{};
                if (ovx_path_dictionary_token_to_string(mDict, result.attributes[i], &text) == OVX_OK && text.ptr &&
                    text.length)
                    vocabulary.emplace_back(text.ptr, text.length);
                else
                    complete = false;
            }
            ovstage_release_query_result(mInstance, &result);
            if (complete)
                mSource.sharePhysicsInputVocabulary(vocabulary);
        }
    }

    releaseCachedQuery(q);
    return ok;
}

bool OvstageChangeFeed::structuralGrowthProbe(uint64_t ord0, uint64_t ord1)
{
    mStructuralRowsValid = readStructuralRows(ord0, ord1, mStructuralRows);
    return mStructuralRowsValid;
}

void OvstageChangeFeed::refreshHierarchyCache(uint64_t ord0,
                                              uint64_t ord1,
                                              const std::unordered_set<std::string>& knownDeletedPaths,
                                              bool knownRowsExpected)
{
    // Nothing cached: the full drop is free here (and still clears the existence memo) and
    // the next consumer builds at the latest ordinal, so there is no delta worth reading.
    if (!mSource.hierarchyCacheBuilt())
    {
        mSource.invalidateHierarchyCache();
        return;
    }
    // Known tombstones the source did not remove exactly (applyPrimRemoval) keep the full
    // rebuild: the live-prim rows below cannot show a delete.
    bool patched = knownDeletedPaths.empty();
    if (patched && !mStructuralRowsValid)
        patched = mStructuralRowsValid = readStructuralRows(ord0, ord1, mStructuralRows, knownRowsExpected);
    if (patched)
        patched = mSource.applyHierarchyDelta(mStructuralRows.added);
    if (!patched)
    {
        hierarchyFallbackCounter().fetch_add(1, std::memory_order_relaxed);
        mSource.invalidateHierarchyCache();
    }
}

bool OvstageChangeFeed::reconcileStructuralChanges(uint64_t ord0, uint64_t ord1)
{
    mStructuralRowsValid = false;
    mStructuralRows.clear();
    std::unordered_set<std::string> deletedPaths;
    bool sawKnownStructuralChange = false;
    bool cachesInvalidated = false;
    bool exactPatch = false;
    bool pureRemoval = false;
    if (!readKnownStructuralChanges(ord0, ord1, &sawKnownStructuralChange, deletedPaths, &cachesInvalidated,
                                    &exactPatch, &pureRemoval))
        return false;

    if (sawKnownStructuralChange)
    {
        // The known-path read proved every affected prim dead and patched its source caches.
        // Also require an empty structural probe over live prims: a simultaneous spawn,
        // recreation or column tombstone on a surviving prim still needs reconciliation.
        if (pureRemoval && exactPatch && mKnownSetAuthoritative && !mRangeGapSinceDrop)
        {
            if (!mStructuralRowsValid)
                mStructuralRowsValid = readStructuralRows(ord0, ord1, mStructuralRows, /*knownRowsExpected=*/true);
            if (mStructuralRowsValid && mStructuralRows.exact && mStructuralRows.rowCount == 0 &&
                ++mIncrementalReconciles < kStructuralSelfHealPeriod)
                return applyIncrementalRemoval(deletedPaths);
        }
        return rescanKnownPaths(ord0, ord1, deletedPaths, cachesInvalidated, exactPatch);
    }
    // A partial known set (a failed seed or rescan scan) would make the value and transform
    // reads below watch a subset: repair it now, whatever this range carries.
    if (!mKnownSetAuthoritative)
        return rescanKnownPaths(ord0, ord1, deletedPaths, cachesInvalidated, false);
    if (!structuralAttributesChanged(ord0))
    {
        // No structural column was sealed, no known prim changed, and the known set is authoritative:
        // the prim set is provably identical to the previous drain; nothing to reconcile.
        return true;
    }

    // The floors say a structural column was sealed but no known prim changed: a prim
    // joined a family, or a SCOPE_ALL seal advanced every floor with no real structural
    // write. The change-range read distinguishes the two exactly (a floor advance is not
    // a write) and classifies each unfamiliar row; the family-count compare is the fallback.
    bool decided;
    decided = structuralGrowthProbe(ord0, ord1);
    if (!decided)
    {
        bool unchanged = false;
        if (familyMembershipUnchanged(unchanged) && unchanged)
        {
            // No family grew, but the range wrote structural rows nobody read: a change no
            // drained range showed, which the memos cannot be patched for.
            dropSourceCaches();
            return true;
        }
        return rescanKnownPaths(ord0, ord1, deletedPaths, cachesInvalidated, false);
    }
    if (mStructuralRows.exact && mStructuralRows.unfamiliar.empty())
    {
        // No physics family grew, but the rows (a non-physics prim's type / schemas, a column
        // deleted on one) still change what the source memos mirror.
        if (!mStructuralRows.added.empty() || !mStructuralRows.schemas.empty() || !mStructuralRows.types.empty())
            patchSourceFromStructuralRows(ord0, ord1);
        return true;
    }
    if (!mStructuralRows.exact || !mKnownSetAuthoritative || ++mIncrementalReconciles >= kStructuralSelfHealPeriod)
        return rescanKnownPaths(ord0, ord1, deletedPaths, cachesInvalidated, false);

    return applyIncrementalGrowth(ord0, ord1);
}

void OvstageChangeFeed::dropSourceCaches()
{
    mSource.clearSchemaCache();
    hierarchyFallbackCounter().fetch_add(1, std::memory_order_relaxed);
    mSource.invalidateHierarchyCache();
    mRangeGapSinceDrop = false; // the rebuild reads latest: the skipped ordinals are in it
}

void OvstageChangeFeed::patchSourceFromStructuralRows(uint64_t ord0, uint64_t ord1)
{
    // Any unfamiliar structural write changes the stage the source caches mirror, physics
    // row or not. The rows in hand patch the schema, type and hierarchy memos (a drop would
    // send the flush back to whole-stage queries per spawn); the instancing latches have no
    // delta and are re-armed. The new prims are dirtied into the path-list queries by the
    // caller, so the value read's per-drain discovery covers their columns.
    if (mSource.applySchemaDelta(mStructuralRows.schemas) && mSource.applyPrimTypeDelta(mStructuralRows.types))
        mSource.resetInstancingCaches();
    else
        mSource.clearSchemaCache();
    refreshHierarchyCache(ord0, ord1, {});
    mResolvedPathKeys.clear();
    // The typeName dispatch asks exists() for every new prim; the rows already prove them live.
    mSource.noteLivePrims(mStructuralRows.added);
}

bool OvstageChangeFeed::dispatchStructuralAdds(const std::vector<ObjectKey>& newKeys)
{
    if (newKeys.empty())
        return true;
    const TokenId typeNameField = mSource.internToken("typeName");
    ChangeBatch batch;
    batch.isDelete = false;
    batch.numChanges = newKeys.size();
    batch.keys = ColumnView{ ColumnType::eObjectKey, -1, newKeys.size(), newKeys.data() };
    batch.property = TokenId{};
    batch.values = ColumnView{ ColumnType::eToken, -1, 1, &typeNameField };
    // The consumer's add handling walks each new prim's subtree (PrimUpdateMap::addPrim). The
    // hierarchy cache was patched or dropped for exactly these rows, so a bulk-read window lets
    // it certify the new leaves instead of firing a whole-stage prefix query per leaf.
    struct ScopedBulkRead
    {
        const OvstageSource& source;
        explicit ScopedBulkRead(const OvstageSource& s) : source(s) { source.beginHierarchyBulkRead(); }
        ~ScopedBulkRead() { source.endHierarchyBulkRead(); }
    } scopedBulkRead(mSource);
    return dispatch(batch);
}

bool OvstageChangeFeed::applyIncrementalGrowth(uint64_t ord0, uint64_t ord1)
{
    patchSourceFromStructuralRows(ord0, ord1);
    mSource.sharePhysicsInputTypes(
        std::move(mStructuralRows.sharedTypes), mRangeGapSinceDrop ? nullptr : &mKnownPhysicsPaths);

    std::vector<ObjectKey> newKeys;
    mCollectAddedPaths = mAddedPropertiesCovered;
    bool allKeysResolved = true;
    size_t schemaMatches = 0;
    size_t typeMatches = 0;
    for (const StructuralRows::Row& r : mStructuralRows.unfamiliar)
    {
        if (!r.physics())
            continue;
        if (mKnownPhysicsPaths.insert(r.path).second)
        {
            mKnownPhysicsQuery.noteAdded(r.path);
            if (mAddedPropertiesCovered)
                mAddedPhysicsPaths.insert(r.path);
            rememberTransformPathAndAncestors(r.path);
        }
        schemaMatches += r.bySchemas ? 1 : 0;
        typeMatches += r.byType ? 1 : 0;
        ObjectKey key = resolveGroupPathKey(r.raw);
        if (consumeFault(addedKeyResolutionFault()))
        {
            key = {};
        }
        if (key.valid())
            newKeys.push_back(key);
        else
            allKeysResolved = false;
    }
    // The frozen family queries stay; keep their recorded counts exact so the count-compare
    // fallback does not read every incremental add as growth (or fail a drain under a
    // write-floor violation it would otherwise have survived).
    if (mSchemaFamilyQuery.query != OVSTAGE_INVALID_QUERY_HANDLE)
        mSchemaFamilyQuery.primCount += schemaMatches;
    if (mTypeFamilyQuery.query != OVSTAGE_INVALID_QUERY_HANDLE)
        mTypeFamilyQuery.primCount += typeMatches;

    mCollectAddedPaths = false;
    const bool accepted = dispatchStructuralAdds(newKeys);
    if (!accepted || !allKeysResolved)
    {
        mAddedPhysicsPaths.clear();
        mAddedTransformPaths.clear();
    }
    return accepted;
}

bool OvstageChangeFeed::applyIncrementalRemoval(const std::unordered_set<std::string>& deletedPaths)
{
    // Keep the old membership until the consumer accepts the tombstones. On rejection the
    // cursor holds, and the next rescan must still find these paths to redeliver the delete.
    mKnownSetAuthoritative = false;
    std::vector<ObjectKey> deleteKeys;
    deleteKeys.reserve(deletedPaths.size());
    for (const std::string& path : deletedPaths)
    {
        const ObjectKey key = mSource.findByPath(path);
        if (!key.valid())
            return false;
        deleteKeys.push_back(key);
    }
    ChangeBatch batch;
    batch.isDelete = true;
    batch.numChanges = deleteKeys.size();
    batch.keys = ColumnView{ ColumnType::eObjectKey, -1, deleteKeys.size(), deleteKeys.data() };
    if (!dispatch(batch))
        return false;

    for (const std::string& path : deletedPaths)
    {
        if (mKnownPhysicsPaths.erase(path) == 0)
            continue;
        mKnownPhysicsQuery.noteRemoved(path);
        // Visit only the deleted path's ancestor chain. Shared ancestors remain
        // watched until the last surviving physics path releases them.
        std::string current(path);
        while (!current.empty())
        {
            const auto ref = mTransformPathRefCounts.find(current);
            if (ref != mTransformPathRefCounts.end() && --ref->second == 0)
            {
                mTransformPathRefCounts.erase(ref);
                mKnownTransformPaths.erase(current);
                mKnownTransformQuery.noteRemoved(current);
            }
            if (current == "/")
                break;
            const size_t slash = current.find_last_of('/');
            if (slash == std::string::npos || slash == 0)
                current = "/";
            else
                current.resize(slash);
        }
    }
    mResolvedPathKeys.clear();
    mFamilyCountsCurrent = false;
    mKnownSetAuthoritative = true;
    return true;
}

bool OvstageChangeFeed::rescanKnownPaths(uint64_t ord0,
                                         uint64_t ord1,
                                         std::unordered_set<std::string>& deletedPaths,
                                         bool cachesInvalidated,
                                         bool exactPatch)
{
    structuralRescanCounter().fetch_add(1, std::memory_order_relaxed);
    mIncrementalReconciles = 0;
    mKnownSetAuthoritative = false;
    // The tombstones the exact known-path read saw; `deletedPaths` is reconciled below.
    const std::unordered_set<std::string> tombstoned = deletedPaths;
    // A rescan is for structural writes no drained range showed (a skipped ordinal, a classifier
    // miss), so unless it proves the drained rows told the whole story (below), both source caches
    // are dropped: only a rebuild from latest topology can be trusted.
    mResolvedPathKeys.clear();
    // Filter queries are frozen at build: a cached family query would never contain
    // the prims the rescan is here to find.
    dirtyCachedFamilyQueries();

    // The refresh only inserts into the fresh sets below, so it can borrow the
    // previous set instead of copying every path string.
    const std::unordered_set<std::string>& oldPhysicsPaths = mKnownPhysicsPaths;

    std::unordered_set<std::string> freshPhysicsPaths;
    std::unordered_set<std::string> freshTransformPaths;
    std::vector<ObjectKey> newKeys;
    const bool schemaScanOk = collectKnownPathsFromFilter(
        "usd-schemas", OVSTAGE_FILTER_OP_CONTAINS, mPhysicsSchemas.data(), mPhysicsSchemas.size(),
        mSchemaFamilyQuery, freshPhysicsPaths, freshTransformPaths, &newKeys, &oldPhysicsPaths);
    const bool typeScanOk = collectKnownPathsFromTypes(
        freshPhysicsPaths, freshTransformPaths, &newKeys, &oldPhysicsPaths);
    bool ok = schemaScanOk && typeScanOk; // folds in the structural-batch dispatch results below

    // The final structural snapshot is authoritative. The ranged read only
    // addresses oldPhysicsPaths, so erase its tombstone when the same path was
    // recreated through another structural channel within this drain.
    if (schemaScanOk && typeScanOk)
    {
        for (const std::string& oldPath : oldPhysicsPaths)
        {
            if (freshPhysicsPaths.find(oldPath) == freshPhysicsPaths.end())
                deletedPaths.insert(oldPath);
            else
                deletedPaths.erase(oldPath);
        }
        // A rejected add can already exist in a subscriber. If the producer
        // removed it before retry, neither snapshot contains it, but it still
        // needs a tombstone. A live/recreated path must not be removed.
        for (const std::string& path : mPendingRescanAddedPaths)
        {
            if (freshPhysicsPaths.find(path) == freshPhysicsPaths.end())
                deletedPaths.insert(path);
            else
                deletedPaths.erase(path);
        }
    }
    else
    {
        // Do not turn a failed/partial refresh into synthetic removals or
        // replace the last known-good path sets with incomplete data.
        dropSourceCaches();
        return false;
    }

    // Keep the exactly patched caches only when the rescan found nothing the drained rows did
    // not show: every removed known path was a tombstone the known-path read saw, and every
    // new key is a live row of the range (a same-range re-create or reparent). Anything else
    // is a change no drained range showed, which no delta can contain -- as is any in-place
    // edit of a known prim in an ordinal skipped since the last drop (mRangeGapSinceDrop): the
    // family sets agree on it, and the ranged known-path read never saw its row.
    bool drop = !cachesInvalidated || !exactPatch || mRangeGapSinceDrop;
    for (auto it = deletedPaths.begin(); !drop && it != deletedPaths.end(); ++it)
        drop = tombstoned.count(*it) == 0;
    // The range's rows are also the memo patch input for everything unfamiliar in it (a spawn
    // paired with the delete, a non-physics prim's type or schemas), so they are needed even
    // when the rescan found no new key.
    if (!drop && !mStructuralRowsValid)
        mStructuralRowsValid = readStructuralRows(ord0, ord1, mStructuralRows, /*knownRowsExpected=*/true);
    drop = drop || !mStructuralRowsValid || !mStructuralRows.exact;
    if (!drop && !newKeys.empty())
    {
        std::unordered_set<std::string> livePaths;
        for (const ovx_primpath_t raw : mStructuralRows.added)
            livePaths.emplace(mSource.sourceKeyToString(mSource.internKey(raw.raw_id)));
        for (const ObjectKey key : newKeys)
            drop = drop || livePaths.count(std::string(mSource.sourceKeyToString(key))) == 0;
    }
    if (!drop)
        drop = !(mSource.applySchemaDelta(mStructuralRows.schemas) && mSource.applyPrimTypeDelta(mStructuralRows.types));
    if (drop)
        dropSourceCaches();
    else
    {
        mSource.resetInstancingCaches();
        mSource.noteLivePrims(mStructuralRows.added);
    }

    std::vector<ObjectKey> deleteKeys;
    deleteKeys.reserve(deletedPaths.size());
    for (const std::string& deletedPath : deletedPaths)
    {
        const ObjectKey key = mSource.findByPath(deletedPath);
        if (key.valid())
            deleteKeys.push_back(key);
    }

    if (!deleteKeys.empty())
    {
        ChangeBatch batch;
        batch.isDelete = true;
        batch.numChanges = deleteKeys.size();
        batch.keys = ColumnView{ ColumnType::eObjectKey, -1, deleteKeys.size(), deleteKeys.data() };
        ok = dispatch(batch) && ok;
    }

    ok = dispatchStructuralAdds(newKeys) && ok;
    // A rejected batch holds the drain cursor. Keep the previous membership so
    // every rescan retry can discover and redeliver the pending deletions.
    if (ok)
    {
        mKnownPhysicsPaths = std::move(freshPhysicsPaths);
        mKnownTransformPaths = std::move(freshTransformPaths);
        rebuildTransformPathRefCounts();
        mKnownPhysicsQuery.noteReplaced();
        mKnownTransformQuery.noteReplaced();
        mPendingRescanAddedPaths.clear();
    }
    else
    {
        for (const ObjectKey key : newKeys)
            mPendingRescanAddedPaths.emplace(mSource.sourceKeyToString(key));
    }
    mKnownSetAuthoritative = ok;
    mFamilyCountsCurrent = ok;
    return ok;
}

bool OvstageChangeFeed::readKnownStructuralChanges(uint64_t ord0,
                                                   uint64_t ord1,
                                                   bool* sawChange,
                                                   std::unordered_set<std::string>& deletedPaths,
                                                   bool* outInvalidatedCaches,
                                                   bool* outExactPatch,
                                                   bool* outPureRemoval)
{
    if (outInvalidatedCaches)
        *outInvalidatedCaches = false;
    if (outExactPatch)
        *outExactPatch = false;
    if (outPureRemoval)
        *outPureRemoval = false;
    if (sawChange)
        *sawChange = false;
    deletedPaths.clear();
    if (mKnownPhysicsPaths.empty())
        return true;

    std::array<ovx_token_t, 3> attrToks{};
    size_t attrCount = 0;
    const ovx_token_t schemasTok = internOvxToken(conv::kUsdSchemas);
    const ovx_token_t primTypeTok = internOvxToken(conv::kUsdPrimType);
    if (schemasTok != OVX_INVALID_TOKEN)
        attrToks[attrCount++] = schemasTok;
    if (primTypeTok != OVX_INVALID_TOKEN)
        attrToks[attrCount++] = primTypeTok;
    const ovx_token_t authoredTypeTok = internOvxToken(conv::kUsdTypeName);
    if (authoredTypeTok != OVX_INVALID_TOKEN)
        attrToks[attrCount++] = authoredTypeTok;
    if (attrCount == 0)
        return false;

    // Non-empty path set, so no handle after a successful sync is a construction failure.
    if (!syncPathListQuery(mKnownPhysicsPaths, mKnownPhysicsQuery) || !mKnownPhysicsQuery.valid())
        return false;

    bool ok = true;
    // Collect apiSchemas/typeName change batches during the read, then invalidate the
    // source's schema/hierarchy caches, then dispatch — so the classification callbacks
    // (forEachAppliedSchema etc.) observe post-change membership rather than the stale
    // pre-change cache (issue #8, discover -> invalidate -> dispatch).
    struct DeferredStructural
    {
        std::vector<ObjectKey> keys;
        bool isSchemas;
        bool isAuthoredType = false;
    };
    std::vector<DeferredStructural> deferred;
    // The same rows as memo patch inputs: complete post-write lists / types of the known prims,
    // and the tombstoned known prims. Any row the decoders reject makes the patch inexact.
    std::vector<SchemaRowDelta> knownSchemas;
    std::vector<PrimTypeRowDelta> knownTypes;
    std::vector<ovx_primpath_t> tombstoneRaws;
    bool allRowsDecodable = true;
    for (const PathListHandle* h : mKnownPhysicsQuery.handles())
    {
        if (!h->valid())
            continue;
        const ovstage_query_handle_t q = h->query;
        ovstage_ordinal_range_t range{};
        range.start_ordinal = ord0;
        range.end_ordinal = ord1;
        range.has_start_ordinal = true;

        ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
        const ovstage_enqueue_result_t re =
            ovstage_read_attributes(mInstance, q, attrToks.data(), attrCount, range, &rh);
        if (re.status == OVSTAGE_OK)
        {
            waitAndRelease(mInstance, re);
            ReadListMemo listMemo(mDict);
            ReadColumnCursor cursor(mInstance, rh);
            ReadGroupColumn g{};
            ovstage_api_status_t fetchErr;
            while ((fetchErr = cursor.next(g)) == OVSTAGE_OK)
            {
                if (sawChange)
                    *sawChange = true;
                // The query's own list was captured with the handle; any other list once per read.
                const ovx_primpath_t* groupPaths = nullptr;
                size_t groupPathCount = 0;
                if (g.prims.list == h->list && !h->paths.empty())
                {
                    groupPaths = h->paths.data();
                    groupPathCount = h->paths.size();
                }
                else if (!listMemo.paths(g.prims.list, &groupPaths, &groupPathCount))
                    groupPaths = nullptr;

                if (g.is_delete)
                {
                    if (groupPaths)
                    {
                        for (uint32_t i = 0; i < g.prims.count; ++i)
                        {
                            const uint32_t idx = g.prims.index_map ? g.prims.index_map[i] : (g.prims.offset + i);
                            if (idx >= groupPathCount)
                            {
                                allRowsDecodable = false;
                                continue;
                            }
                            const std::string_view path = mSource.sourceKeyToString(mSource.internKey(groupPaths[idx].raw_id));
                            if (path.empty())
                            {
                                allRowsDecodable = false;
                                continue;
                            }
                            if (deletedPaths.insert(std::string(path)).second)
                                tombstoneRaws.push_back(groupPaths[idx]);
                            if (g.attribute == authoredTypeTok)
                                knownTypes.push_back({ groupPaths[idx], 0, true });
                        }
                    }
                    else
                        allRowsDecodable = false;
                }
                else if (groupPaths && (g.attribute == schemasTok || g.attribute == primTypeTok || g.attribute == authoredTypeTok))
                {
                    const bool decodable = g.data.mask == nullptr && g.data.tensor_count > 0 && g.data.tensors &&
                                           !(g.data.index_map && g.data.count < g.prims.count);
                    std::vector<ObjectKey> keys;
                    keys.reserve(g.prims.count);
                    for (uint32_t i = 0; i < g.prims.count; ++i)
                    {
                        const uint32_t idx = g.prims.index_map ? g.prims.index_map[i] : (g.prims.offset + i);
                        if (idx >= groupPathCount)
                            continue;
                        const ovx_primpath_t raw = groupPaths[idx];
                        keys.push_back(resolveGroupPathKey(raw));
                        if (!decodable)
                        {
                            allRowsDecodable = false;
                        }
                        else if (g.attribute == schemasTok)
                        {
                            SchemaRowDelta& row = knownSchemas.emplace_back();
                            row.raw = raw;
                            if (!forEachSchemaToken(g, i, [&](uint64_t value) { row.schemaTokens.push_back(value); }))
                                allRowsDecodable = false;
                        }
                        else
                        {
                            uint64_t value = 0;
                            if (decodeTypeRow(g, i, value))
                                knownTypes.push_back({ raw, value, g.attribute == authoredTypeTok });
                            else
                                allRowsDecodable = false;
                        }
                    }
                    if (!keys.empty())
                        deferred.push_back({ std::move(keys), g.attribute == schemasTok, g.attribute == authoredTypeTok });
                }
                else
                    allRowsDecodable = false;
            }
            if (fetchErr != OVSTAGE_ERROR_END_OF_ITERATION)
                ok = false;
            waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
        }
        else
        {
            ok = false;
        }
        if (!ok)
            break; // the drain already fails; the second round trip buys nothing
    }

    // Discover -> patch -> dispatch: all structural groups are now read, so bring the source's
    // schema, type, existence and hierarchy memos to post-change state before replaying the
    // apiSchemas/typeName batches (dispatching first let the classification callbacks read
    // pre-change membership, so an API add/remove could be classified from the stale cache
    // and never replayed). The rows are complete post-write state, so the memos are patched
    // exactly: dead prims leave with their cached subtree (a tombstoned column on a prim that
    // is still live only empties its list), live known rows replace their entries. Anything
    // the source cannot apply exactly drops the caches, as before. A truncated read fails the
    // drain and the range is redelivered, so its partial buffer is not applied.
    if (ok && (!deferred.empty() || !deletedPaths.empty()))
    {
        bool exact = allRowsDecodable;
        std::vector<ovx_primpath_t> deadRaws;
        if (exact && !tombstoneRaws.empty())
        {
            std::vector<ObjectKey> keys;
            keys.reserve(tombstoneRaws.size());
            for (const ovx_primpath_t raw : tombstoneRaws)
                keys.push_back(mSource.internKey(raw.raw_id));
            std::vector<bool> live;
            exact = mSource.existsBatchChecked(keys, live);
            for (size_t i = 0; exact && i < tombstoneRaws.size(); ++i)
                if (!live[i])
                    deadRaws.push_back(tombstoneRaws[i]);
        }
        // Whole-prim tombstones are handled by removal. Only an authored-type
        // tombstone on a surviving prim needs to invalidate its fallback type.
        const std::unordered_set<ovx_primpath_t> deadTypes(deadRaws.begin(), deadRaws.end());
        knownTypes.erase(std::remove_if(knownTypes.begin(), knownTypes.end(),
                                       [&](const PrimTypeRowDelta& row) { return deadTypes.count(row.raw) != 0; }),
                         knownTypes.end());
        for (const PrimTypeRowDelta& row : knownTypes)
            if (row.authored && row.typeToken == 0)
                deferred.push_back({ { resolveGroupPathKey(row.raw) }, false, true });
        std::unordered_set<uint64_t> changedTypes;
        exact = exact && mSource.applyPrimRemoval(deadRaws) && mSource.applySchemaDelta(knownSchemas) &&
                mSource.applyPrimTypeDelta(knownTypes, &changedTypes);
        mPendingAuthoredTypeChanges.insert(changedTypes.begin(), changedTypes.end());
        if (exact)
            mSource.resetInstancingCaches();
        else
            mSource.clearSchemaCache();
        refreshHierarchyCache(ord0, ord1, exact ? std::unordered_set<std::string>{} : deletedPaths,
                              /*knownRowsExpected=*/true);
        if (mStructuralRowsValid)
            mSource.noteLivePrims(mStructuralRows.added);
        if (outInvalidatedCaches)
            *outInvalidatedCaches = true;
        if (outExactPatch)
            *outExactPatch = exact;
        if (outPureRemoval)
            *outPureRemoval = exact && deferred.empty() && !deletedPaths.empty() &&
                              deadRaws.size() == deletedPaths.size();

        const TokenId apiSchemasField = mSource.internToken("apiSchemas");
        const TokenId typeNameField = mSource.internToken("typeName");
        for (DeferredStructural& d : deferred)
        {
            // Population refreshes parent authored metadata without replacing
            // the prim. Built-in structural writes still signal repopulation:
            // a range snapshot can fold same-type delete/recreate into one row.
            if (exact && d.isAuthoredType)
                d.keys.erase(std::remove_if(d.keys.begin(), d.keys.end(), [&](ObjectKey key)
                {
                    return mPendingAuthoredTypeChanges.count(mSource.rawHandle(key)) == 0;
                }), d.keys.end());
            if (d.keys.empty())
                continue;
            const TokenId& field = d.isSchemas ? apiSchemasField : typeNameField;
            ChangeBatch batch;
            batch.isDelete = false;
            batch.numChanges = d.keys.size();
            batch.keys = ColumnView{ ColumnType::eObjectKey, -1, d.keys.size(), d.keys.data() };
            batch.property = TokenId{};
            batch.values = ColumnView{ ColumnType::eToken, -1, 1, &field };
            ok = dispatch(batch) && ok;
        }
    }

    return ok;
}

const std::vector<ovx_token_t>& OvstageChangeFeed::transformAttrTokens()
{
    if (mTransformAttrToks.empty())
    {
        std::unordered_set<ovx_token_t> seen;
        for (const char* attrName :
             { conv::kFabricLocalMatrix, conv::kLocalTransform, conv::kResetXformStack })
        {
            const ovx_token_t tok = internOvxToken(attrName);
            if (tok != OVX_INVALID_TOKEN && seen.insert(tok).second)
                mTransformAttrToks.push_back(tok);
        }
    }
    return mTransformAttrToks;
}

bool OvstageChangeFeed::readKnownTransformChanges(uint64_t ord0, uint64_t ord1)
{
    if (mKnownTransformPaths.empty())
        return true;

    if (transformAttrTokens().empty())
        return true;

    // mKnownTransformPaths is non-empty here (guarded above), so a failed sync or no
    // handle is a construction/API failure, not an empty match set: fail the drain
    // rather than silently skipping the family and advancing the external cursor.
    if (!syncPathListQuery(mKnownTransformPaths, mKnownTransformQuery, true) || !mKnownTransformQuery.valid())
        return false;

    const TokenId worldMatrixProperty = mSource.internToken(conv::kFabricWorldMatrix);
    // Local frames and reset-stack edits drive notifications. Read derived world
    // matrices alongside them only to cache resolved poses for the transform flush.
    std::vector<ovx_token_t> readAttrs = mTransformAttrToks;
    readAttrs.push_back(internOvxToken(conv::kFabricWorldMatrix));

    // One bucket for the whole drain: the transform flush at group-complete reads
    // every changed body through it, so each world-matrix group appends to the bucket
    // instead of replacing its predecessor (a replaced bucket sent every body but the
    // last to a live per-prim ovstage round trip).
    mSource.clearBucket();

    // A consumer returning false (partial commit) fails the drain closed so the cursor holds.
    bool ok = true;

    // A change-range read that selects nothing is a successful zero-group read, so it
    // doubles as the "did anything move" probe without per-attribute floor queries.
    auto runRead = [&](const PathListHandle& h, const std::vector<ovx_token_t>& attrs, ovstage_api_status_t& err) -> bool
    {
        ovstage_ordinal_range_t range{};
        range.start_ordinal = ord0;
        range.end_ordinal = ord1;
        range.has_start_ordinal = true;

        ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
        const ovstage_enqueue_result_t re =
            ovstage_read_attributes(mInstance, h.query, attrs.data(), attrs.size(), range, &rh);
        err = re.status;
        if (re.status != OVSTAGE_OK)
            return false;
        waitAndRelease(mInstance, re);

        uint64_t groupCount = 0;
        ReadColumnCursor cursor(mInstance, rh);
        ReadGroupColumn g{};
        while ((err = cursor.next(g)) == OVSTAGE_OK)
        {
            ++groupCount;
            cursor.keep(mDeferredGroups);
            const TokenId property = propertyTokenFor(g.attribute);
            collectGroupKeys(g, mGroupKeys, &h);
            if (g.is_delete)
            {
                if (property == worldMatrixProperty)
                    continue;
                // Removing a local/reset input changes the resolved pose of a
                // surviving prim. Whole-prim removal stays on the structural path.
                mGroupKeys.erase(std::remove_if(mGroupKeys.begin(), mGroupKeys.end(),
                    [&](ObjectKey key) { return !mSource.exists(key); }), mGroupKeys.end());
                if (mGroupKeys.empty())
                    continue;
            }
            if (property == worldMatrixProperty)
            {
                mSource.seedBucketFromReadGroup(property, g, mGroupKeys.empty() ? nullptr : mGroupKeys.data(),
                                                mGroupKeys.size(), /*append=*/true, mGroupListPaths, mGroupListCount);
                // Hierarchy evaluation can rewrite unrelated world rows. Those are
                // not authored pose edits and must never teleport simulated bodies.
                continue;
            }

            ChangeBatch batch;
            batch.isDelete = false;
            batch.numChanges = mGroupKeys.size();
            batch.keys = ColumnView{ ColumnType::eObjectKey, -1, mGroupKeys.size(),
                                     mGroupKeys.empty() ? nullptr : mGroupKeys.data() };
            batch.property = property;
            ok = dispatch(batch) && ok;
        }
        releaseDeferredGroups();
        waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
        return err == OVSTAGE_ERROR_END_OF_ITERATION;
    };

    // Device POSE drain. On a DirectGPU sim, apply the authored poses straight from GPU: device-read the
    // RESOLVED world matrix for EXACTLY the prims whose local frame / reset stack changed this range, then
    // dispatch it as a device column the write side decomposes (pos + quat) and scatters with no host copy.
    // The gating is the whole correctness point: omni:fabric:worldMatrix is a read-only OUTPUT column that
    // hierarchy evaluation rewrites for unrelated bodies, so reading it over the WHOLE superset would
    // teleport a body nobody re-posed (REQ-PARSE-XFORM-001 AC-5). Restricting the device
    // read to the authored-local-frame path set -- the exact set the host read dispatches -- reads only the
    // poses the producer actually authored. The host read below is INERT for a DirectGPU rigid (updateTransform is
    // a no-op for a DirectGPU actor) but NOT for every prim (it resyncs a child collider), so rows are routed by
    // whether they carry a world matrix:
    //  - A row WITH a world matrix goes to the device path. applyPoseBatch SPLITS: it poses the serviceable bodies
    //    on device and reports the rest for the per-prim path -- harmless, since a row it cannot serve is not a
    //    created, posable body.
    //  - An authored key with NO world-matrix row (masked out, or omitted from the read) is a local-only prim
    //    (REQ-PARSE-XFORM-001 AC-5); it is routed through the host read, which handles it per prim. So is a delete.
    //  - The device read refuses a query holding a local-only prim (NOT_SUPPORTED: it is all-or-nothing), so that
    //    refusal is answered by device-reading just the prims that carry a world matrix; the rest go to the host
    //    read. Any OTHER failure that leaves world-matrix data unapplied -- a setup/fetch error, or a row whose
    //    value is present but not a usable CUDA column -- would be DROPPED by the inert host read, so it FAILS the
    //    drain (retryable).
    const bool poseDeviceAvail = mDeviceOrdinal >= 0 && deviceReadAvailable(mInstance);
    bool anyTransformFloor = false;
    if (poseDeviceAvail)
        for (ovx_token_t tok : mTransformAttrToks)
            if (hasAttributeFloorAtOrAfter(tok, ord0))
            {
                anyTransformFloor = true;
                break;
            }

    if (poseDeviceAvail && anyTransformFloor)
    {
        // Phase 1: collect the authored-local-frame set (keys + owned path strings) from a change-range read
        // of the LOCAL-FRAME attrs only -- only rows that carry a value (a masked-out row did not change, and
        // reading its world matrix in Phase 2 would teleport an unposed body) and only prims that still exist (a
        // whole-prim removal is structural, reconciled before this read, as the host transform read drops a key
        // whose prim is gone). A delete of a local/reset input re-poses a surviving prim and is reconciled by the
        // host read below (sawDelete), without standing down the device serve for the authored poses.
        std::vector<ObjectKey> authoredKeys;
        std::vector<std::string> authoredPaths; // owns the strings buildPathListQuery views into
        std::unordered_set<ObjectKey, ObjectKey::Hash> seen;
        bool sawDelete = false;
        auto collectAuthored = [&](const PathListHandle& h, const std::vector<ovx_token_t>& attrs,
                                   ovstage_api_status_t& err) -> bool
        {
            ovstage_ordinal_range_t range{};
            range.start_ordinal = ord0;
            range.end_ordinal = ord1;
            range.has_start_ordinal = true;

            ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
            const ovstage_enqueue_result_t re =
                ovstage_read_attributes(mInstance, h.query, attrs.data(), attrs.size(), range, &rh);
            err = re.status;
            if (re.status != OVSTAGE_OK)
                return false;
            waitAndRelease(mInstance, re);

            ReadColumnCursor cursor(mInstance, rh);
            ReadGroupColumn g{};
            while ((err = cursor.next(g)) == OVSTAGE_OK)
            {
                if (g.is_delete)
                    sawDelete = true;
                else
                {
                    collectGroupKeys(g, mGroupKeys, &h, /*skipMaskedRows=*/true);
                    for (ObjectKey key : mGroupKeys)
                        if (mSource.exists(key) && seen.insert(key).second)
                        {
                            authoredKeys.push_back(key);
                            authoredPaths.emplace_back(mSource.sourceKeyToString(key));
                        }
                }
                cursor.keep(mDeferredGroups);
            }
            releaseDeferredGroups();
            waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
            return err == OVSTAGE_ERROR_END_OF_ITERATION;
        };

        const bool collectOk =
            readPathListQuery(mKnownTransformQuery, mTransformAttrToks, ord0, collectAuthored, true);

        // Phase 2: device-read the resolved world matrix for the AUTHORED (non-delete) paths and dispatch it as a
        // device column. A delete in the range (sawDelete) does NOT disable the device serve for the surviving
        // authored poses -- that would send them to the host transform path, inert on a DirectGPU scene, and drop
        // them. The deletes themselves are reconciled by the host read below, which `needsHost = sawDelete` forces.
        // A failed authored-set collection (read enqueue/fetch or query-sync error) must FAIL the drain, not
        // fall through to the host read: inside this block the attach is all-DirectGPU (mDeviceOrdinal >= 0),
        // where the host read's per-prim transform path is inert (updateTransform is a no-op for a DirectGPU
        // actor, UsdInterface.cpp), so a "successful" host read would advance the ordinal applying nothing.
        // These setup failures are transient -- retry can succeed -- so holding the cursor recovers the poses.
        // An EMPTY authored set is NOT a failure: no local frame resolved to a serviceable key (deletes-only or
        // a no-op floor), which the host read reconciles.
        if (!collectOk)
            ok = false;
        else if (!authoredKeys.empty())
        {
            std::vector<const std::string*> pathPtrs;
            pathPtrs.reserve(authoredPaths.size());
            for (const std::string& p : authoredPaths)
                pathPtrs.push_back(&p);
            PathListHandle sub = buildPathListQuery(pathPtrs);
            if (sub.valid())
            {
                bool served = false;          // at least one group was dispatched on device
                bool needsHost = sawDelete;   // a delete, or an authored key with no world-matrix row, still needs
                                              // the host read (see the coverage check below)
                std::unordered_set<ObjectKey, ObjectKey::Hash> covered; // authored keys handed to the device path
                std::vector<uint32_t> keptRows; // per present key: its logical row in the group
                std::vector<uint32_t> rowMap;   // per present key: its storage row (the gather the write side runs)
                // A LATEST snapshot read (no start ordinal), not a change-range read: the authored set is already known
                // (Phase 1), so what is wanted is each authored prim's CURRENT resolved world matrix. The snapshot
                // reflects the authored local edit this range, so the pose applied equals the authored pose.
                ovstage_ordinal_range_t range{};
                range.end_ordinal = ord1;
                range.has_start_ordinal = false;
                const ovx_token_t worldMatrixTok = internOvxToken(conv::kFabricWorldMatrix);

                // Device-read `q`'s world matrices and dispatch every row that carries one. Returns the enqueue status;
                // a fetch error fails the drain in place.
                const auto serveOnDevice = [&](const PathListHandle& q) -> ovstage_api_status_t
                {
                    ovstage_placement_t placement = OVSTAGE_PLACEMENT_INIT;
                    placement.device.device_type = kDLCUDA;
                    placement.device.device_id = mDeviceOrdinal;
                    ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
                    const ovstage_enqueue_result_t re = ovstage_read_attributes_on_device(
                        mInstance, q.query, &worldMatrixTok, 1, range, &placement, &rh);
                    if (re.status != OVSTAGE_OK)
                        return re.status;
                    waitAndRelease(mInstance, re);
                    ReadColumnCursor cursor(mInstance, rh);
                    ReadGroupColumn g{};
                    ovstage_api_status_t fetchErr;
                    while ((fetchErr = cursor.next(g)) == OVSTAGE_OK)
                    {
                        // Route by WORLD-MATRIX PRESENCE. A row carrying a world matrix is the device path's: it is
                        // served here (applyPoseBatch poses the serviceable bodies and reports the rest for the
                        // per-prim path). A row with NO world-matrix value -- masked out of the group, or omitted
                        // from the read altogether -- is a LOCAL-ONLY prim (REQ-PARSE-XFORM-001 AC-5), e.g. a child
                        // collider. The host transform read handles that per prim (handleTransformChange resyncs
                        // a collider's body; it is inert only for a DirectGPU rigid), so it is left uncovered and
                        // routed there by the coverage check after the loop. A delete goes to the host read too.
                        if (g.is_delete)
                        {
                            needsHost = true;
                            cursor.keep(mDeferredGroups);
                            continue;
                        }
                        collectGroupKeys(g, mGroupKeys, &q, /*skipMaskedRows=*/true, &keptRows);
                        if (!mGroupKeys.empty())
                        {
                            // The column comes back dense or index-mapped (both observed: a 1-of-N subset whose other
                            // prims' world matrices also changed is index-mapped; a subset whose storage compacts is
                            // dense), and may be masked. A present key's matrix is at storage row index_map[row] when
                            // index-mapped, else at its own logical row. Each matrix is 16 float64 -- or 16 float32,
                            // a representation the host source widens too (OvstageSource decodeMatrixAt) -- so the
                            // storage row count the map indexes into is totalElements/16 -- not g.data.count (the
                            // LOGICAL count).
                            const DLTensor* t = (g.data.tensor_count > 0 && g.data.tensors && g.data.tensors[0].data &&
                                                 !g.is_array && g.data.tensors[0].device.device_type == kDLCUDA) ?
                                                    &g.data.tensors[0] :
                                                    nullptr;
                            const bool f32 = t && t->dtype.code == kDLFloat && t->dtype.bits == 32;
                            const bool f64 = t && t->dtype.code == kDLFloat && t->dtype.bits == 64;
                            const int64_t storageRows =
                                ((f32 || f64) && totalElements(*t) % 16 == 0) ? totalElements(*t) / 16 : 0;
                            rowMap.clear();
                            bool inStorage = storageRows > 0;
                            bool identity = true;
                            for (size_t k = 0; k < keptRows.size(); ++k)
                            {
                                const uint32_t s = g.data.index_map ? g.data.index_map[keptRows[k]] : keptRows[k];
                                inStorage = inStorage && int64_t(s) < storageRows;
                                identity = identity && s == k;
                                rowMap.push_back(s);
                            }
                            if (t && inStorage)
                            {
                                ChangeBatch batch;
                                batch.isDelete = false;
                                batch.numChanges = mGroupKeys.size();
                                batch.keys =
                                    ColumnView{ ColumnType::eObjectKey, -1, mGroupKeys.size(), mGroupKeys.data() };
                                batch.property = worldMatrixProperty;
                                // eDouble / eFloat is the element type; the property tells the consumer each row is
                                // a 16-element matrix. Carry ovstage's producer event so the device decompose
                                // waits on it (ColumnView is CUDA-free, so the handles pass as opaque uintptr).
                                batch.values = ColumnView{ f32 ? ColumnType::eFloat : ColumnType::eDouble,
                                                           mDeviceOrdinal, mGroupKeys.size(),
                                                           static_cast<const uint8_t*>(t->data) + t->byte_offset,
                                                           static_cast<uint64_t>(g.data.cuda_sync.stream),
                                                           static_cast<uint64_t>(g.data.cuda_sync.wait_event) };
                                // Row map (host storage-row indices the write side gathers through); null when the
                                // present rows ARE the storage rows in order -- a dense, unmasked column.
                                batch.values.rowMap =
                                    (identity && int64_t(rowMap.size()) == storageRows) ? nullptr : rowMap.data();
                                ok = dispatch(batch) && ok;
                                served = true;
                                covered.insert(mGroupKeys.begin(), mGroupKeys.end());
                            }
                            else
                                // A world-matrix value IS present but is not a usable CUDA column (host-resident,
                                // malformed storage) -- a contract violation. The host transform read cannot apply
                                // it (inert for a DirectGPU rigid), so FAIL the drain: cursor held, range retried.
                                ok = false;
                        }
                        cursor.keep(mDeferredGroups);
                    }
                    // A real fetch ERROR (anything other than end-of-iteration) can leave authored groups unread
                    // after some were dispatched on device. FAIL the drain so the range stays retryable: forcing the
                    // host read would not recover them -- the per-prim transform path it feeds is inert on a
                    // DirectGPU scene (updateTransform is a no-op for a DirectGPU actor), so a "successful" host read
                    // would advance the ordinal while applying nothing. `ok = false` matches the host and value
                    // fetch loops elsewhere in this file, which fail the drain the same way on a non-end error.
                    if (fetchErr != OVSTAGE_ERROR_END_OF_ITERATION)
                        ok = false;
                    releaseDeferredGroups();
                    waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
                    return OVSTAGE_OK;
                };

                ovstage_api_status_t status = serveOnDevice(sub);
                if (status == OVSTAGE_ERROR_NOT_SUPPORTED)
                {
                    // The device read is all-or-nothing: it refuses a query in which some prim carries NO world-matrix
                    // column -- a local-only prim (REQ-PARSE-XFORM-001 AC-5), e.g. a child collider. That is not
                    // transient (the same range is refused again on retry), so find which authored prims DO carry one
                    // with a HOST snapshot read -- a host read tolerates an absent column -- and device-read just
                    // those. The rest stay uncovered and go to the host read. Paid only when such a prim is in the
                    // authored set.
                    std::unordered_set<ObjectKey, ObjectKey::Hash> withWorld;
                    ovstage_read_handle_t ph = OVSTAGE_INVALID_READ_HANDLE;
                    const ovstage_enqueue_result_t pe =
                        ovstage_read_attributes(mInstance, sub.query, &worldMatrixTok, 1, range, &ph);
                    status = pe.status;
                    if (pe.status == OVSTAGE_OK)
                    {
                        waitAndRelease(mInstance, pe);
                        ReadColumnCursor probe(mInstance, ph);
                        ReadGroupColumn pg{};
                        ovstage_api_status_t probeErr;
                        while ((probeErr = probe.next(pg)) == OVSTAGE_OK)
                        {
                            if (!pg.is_delete)
                            {
                                collectGroupKeys(pg, mGroupKeys, &sub, /*skipMaskedRows=*/true);
                                withWorld.insert(mGroupKeys.begin(), mGroupKeys.end());
                            }
                            probe.keep(mDeferredGroups);
                        }
                        releaseDeferredGroups();
                        waitAndRelease(mInstance, ovstage_release_read(mInstance, ph));
                        status = probeErr == OVSTAGE_ERROR_END_OF_ITERATION ? OVSTAGE_OK : probeErr;
                    }
                    if (status == OVSTAGE_OK && !withWorld.empty())
                    {
                        std::vector<const std::string*> worldPaths;
                        for (size_t i = 0; i < authoredKeys.size(); ++i)
                            if (withWorld.count(authoredKeys[i]) != 0)
                                worldPaths.push_back(&authoredPaths[i]);
                        PathListHandle worldSub = buildPathListQuery(worldPaths);
                        status = worldSub.valid() ? serveOnDevice(worldSub) : OVSTAGE_ERROR_INTERNAL;
                        if (worldSub.valid())
                            releaseCachedQuery(worldSub);
                    }
                }
                // Any other enqueue failure -- or a failed probe or retry -- leaves world-matrix data unapplied that
                // the host read cannot recover (it is inert for a DirectGPU rigid): FAIL the drain, cursor held, range
                // retried.
                if (status != OVSTAGE_OK)
                    ok = false;
                // Coverage: "some group was served" does not mean every authored key was. A key no served group carried
                // has no world-matrix row (a local-only prim); route it through the host read.
                if (!needsHost)
                    for (ObjectKey key : authoredKeys)
                        if (covered.count(key) == 0)
                        {
                            needsHost = true;
                            break;
                        }
                releaseCachedQuery(sub);
                // Skip the host transform read ONLY when every authored key went to the device path and there is no
                // delete to reconcile; otherwise fall through -- the host read routes the uncovered (local-only) keys
                // per prim, and re-applying the device-served rows there is idempotent and harmless.
                if (served && !needsHost)
                    return ok;
            }
            else
                // Sub-query construction failed (buildPathListQuery). Same fail-closed reasoning: an all-
                // DirectGPU attach cannot recover these poses on the inert host read.
                ok = false;
        }
    }

    return readPathListQuery(mKnownTransformQuery, readAttrs, ord0, runRead, true) && ok;
}

bool OvstageChangeFeed::readValueChanges(uint64_t ord0, uint64_t ord1)
{
    if (mKnownPhysicsPaths.empty())
        return true;

    // Non-empty path set, so a failed sync or no handle is a construction/API failure:
    // fail the drain instead of silently skipping every value change.
    if (!syncPathListQuery(mKnownPhysicsPaths, mKnownPhysicsQuery, true) || !mKnownPhysicsQuery.valid())
        return false;

    // The attribute set of the known prims can grow any frame (a column first authored at
    // runtime), so it is re-read from the path-list query every drain. On failure the family
    // filters' discovery from the last (re)build stands in.
    refreshDiscoveryFromQuery(mKnownPhysicsQuery);

    if (mValueReadAttrsDirty || mRegisteredAttributeTokensDirty)
        rebuildValueReadAttributes();
    if (mValueReadAttrs.empty())
        return true;

    // A consumer returning false (partial commit) fails the drain closed so the cursor holds.
    bool ok = true;

    // The change-range read selects exactly the changed keys. A latest-
    // snapshot read would be dense, but its groups carry the attribute's latest ordinal, not
    // each row's, so it cannot tell changed rows from unchanged ones and would re-publish
    // stale values to prims not written in the range.
    auto runRead = [&](const PathListHandle& h, const std::vector<ovx_token_t>& attrs, ovstage_api_status_t& err) -> bool
    {
        ovstage_ordinal_range_t range{};
        range.start_ordinal = ord0;
        range.end_ordinal = ord1;
        range.has_start_ordinal = true;

        ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
        ovstage_enqueue_result_t re{};
        re = ovstage_read_attributes(mInstance, h.query, attrs.data(), attrs.size(), range, &rh);
        err = re.status;
        if (re.status != OVSTAGE_OK)
            return false;
        waitAndRelease(mInstance, re);

        // A change-range read returns one group per changed key, so a bulk write over N prims
        // arrives as N one-row groups. Consumers pay per batch (the write backend plans and
        // scatters each ChangeBatch), so the fixed-width host groups of one attribute are
        // gathered into a single contiguous column and dispatched once per attribute after the
        // loop. Indexed columns select data rows independently of the prim-list indexes;
        // masked-out rows are omitted. Ragged and device groups keep the per-group path below.
        struct CoalescedColumn
        {
            ovx_token_t attr = OVX_INVALID_TOKEN;
            TokenId property;
            ColumnType ct = ColumnType::eNone;
            int64_t comps = 0;
            DLDataType dtype{};
            size_t rowBytes = 0;
            std::vector<ObjectKey> keys;
            std::vector<uint64_t> raws;
            std::vector<uint8_t> bytes;
        };
        std::vector<CoalescedColumn> coalesced;

        auto tryCoalesce = [&](const ReadGroupColumn& grp, TokenId property) -> bool
        {
            ReadColumn column;
            if (!readColumn(grp, column) || mGroupKeys.size() != grp.prims.count || !mGroupListPaths)
                return false;
            const DLTensor& t = *column.tensor;
            const uint32_t rows = grp.prims.count;
            const int64_t comps = column.components;
            const ColumnType ct = columnTypeOf(t.dtype, comps);
            const size_t rowBytes = column.rowBytes;
            if (ct == ColumnType::eNone || comps < 1 || comps > 4 || rowBytes == 0)
                return false;

            CoalescedColumn* col = nullptr;
            for (CoalescedColumn& c : coalesced)
                if (c.attr == grp.attribute)
                    col = &c;
            if (!col)
            {
                coalesced.push_back({});
                col = &coalesced.back();
                col->attr = grp.attribute;
                col->property = property;
                col->ct = ct;
                col->comps = comps;
                col->dtype = t.dtype;
                col->dtype.lanes = 1; // rows are [count, comps] in the gathered view
                col->rowBytes = rowBytes;
            }
            else if (col->ct != ct || col->comps != comps || col->dtype.code != t.dtype.code ||
                     col->dtype.bits != t.dtype.bits)
            {
                return false; // a differently shaped group of the same column takes the per-group path
            }
            if (!column.indexes && !column.mask)
            {
                col->keys.insert(col->keys.end(), mGroupKeys.begin(), mGroupKeys.end());
                for (uint32_t i = 0; i < rows; ++i)
                {
                    const uint32_t idx = grp.prims.index_map ? grp.prims.index_map[i] : (grp.prims.offset + i);
                    col->raws.push_back(idx < mGroupListCount ? mGroupListPaths[idx].raw_id : 0);
                }
                const uint8_t* src = column.data(0);
                col->bytes.insert(col->bytes.end(), src, src + static_cast<size_t>(rows) * rowBytes);
                return true;
            }
            for (uint32_t i = 0; i < rows; ++i)
            {
                if (!column.present(i))
                    continue;
                col->keys.push_back(mGroupKeys[i]);
                const uint32_t idx = grp.prims.index_map ? grp.prims.index_map[i] : (grp.prims.offset + i);
                col->raws.push_back(idx < mGroupListCount ? mGroupListPaths[idx].raw_id : 0);
                const uint8_t* src = column.data(i);
                col->bytes.insert(col->bytes.end(), src, src + rowBytes);
            }
            return true;
        };

        auto flushCoalesced = [&]()
        {
            for (CoalescedColumn& c : coalesced)
            {
                if (c.keys.empty())
                    continue;
                const size_t n = c.keys.size();
                int64_t shape[2] = { static_cast<int64_t>(n), c.comps };
                DLTensor view{};
                view.data = c.bytes.data();
                view.device = { kDLCPU, 0 };
                view.ndim = 2;
                view.dtype = c.dtype;
                view.shape = shape;

                ChangeBatch batch;
                batch.isDelete = false;
                batch.numChanges = n;
                batch.keys = ColumnView{ ColumnType::eObjectKey, -1, n, c.keys.data() };
                batch.property = c.property;
                batch.values = ColumnView{ c.ct, -1, n, c.bytes.data() };
                mSource.seedBucketFromColumn(c.property, c.raws.data(), c.keys.data(), n, view);
                valueBatchDispatchCounter().fetch_add(1, std::memory_order_relaxed);
                ok = dispatch(batch) && ok;
                mSource.clearBucket();
            }
            coalesced.clear();
        };

        uint64_t groupCount = 0;
        ReadColumnCursor cursor(mInstance, rh);
        ReadGroupColumn g{};
        for (;;)
        {
            err = cursor.next(g);
            if (err != OVSTAGE_OK)
                break;
            ++groupCount;
            if (!g.is_delete)
            {
                const TokenId property = propertyTokenFor(g.attribute);
                collectGroupKeys(g, mGroupKeys, &h);
                if (g.attribute == internOvxToken("prototypes") &&
                    mSource.stageHasAttributeColumn(conv::kUsdPrototypes) != std::optional<bool>(false))
                {
                    // Only prims with their own preserved row ignore renderer
                    // remaps. Direct-authored prims in the same group still edit
                    // the ordinary relationship. Dispatch keys for source reads;
                    // the original payload no longer aligns after filtering.
                    const TokenId preserved = mSource.internToken(conv::kUsdPrototypes);
                    mGroupKeys.erase(std::remove_if(mGroupKeys.begin(), mGroupKeys.end(), [&](ObjectKey key)
                    {
                        return mSource.hasRelationship(key, preserved);
                    }), mGroupKeys.end());
                    if (!mGroupKeys.empty())
                    {
                        ChangeBatch batch;
                        batch.numChanges = mGroupKeys.size();
                        batch.keys = { ColumnType::eObjectKey, -1, mGroupKeys.size(), mGroupKeys.data() };
                        batch.property = property;
                        valueBatchDispatchCounter().fetch_add(1, std::memory_order_relaxed);
                        ok = dispatch(batch) && ok;
                    }
                    cursor.keep(mDeferredGroups);
                    continue;
                }
                if (tryCoalesce(g, property))
                {
                    cursor.keep(mDeferredGroups);
                    continue;
                }

                ChangeBatch batch;
                batch.isDelete = false;
                batch.numChanges = mGroupKeys.size();
                batch.keys = ColumnView{ ColumnType::eObjectKey, -1, mGroupKeys.size(),
                                         mGroupKeys.empty() ? nullptr : mGroupKeys.data() };
                batch.property = property;

                // Gather storage for the ragged array path below; must outlive dispatch(batch).
                std::vector<uint8_t> raggedBuf;
                std::vector<uint32_t> raggedOffsets;

                // Attach a dense value column only when every prim row produced a key
                // (keys.size() == prims.count). The value view is a dense pointer with one
                // row per prim row, so batch.numChanges (== keys.size()) must equal the
                // row count; if any row was skipped above (out-of-range / null primpath)
                // the shorter key array would pair values[i] with the wrong prim, so fall
                // back to per-key reads instead (issue #8).
                ReadColumn denseColumn;
                if (readColumn(g, denseColumn) && !denseColumn.indexes && !denseColumn.mask &&
                    mGroupKeys.size() == g.prims.count)
                {
                    const DLTensor& t = *denseColumn.tensor;
                    const uint32_t rows = g.prims.count;
                    const int64_t comps = denseColumn.components;
                    const ColumnType ct = columnTypeOf(t.dtype, comps);
                    // Host columns only, as the ragged path below already requires. A device column
                    // carries a producer-ordering event (`g.data.cuda_sync`) a consumer MUST wait on before
                    // reading it, but `ChangeBatch` is source-agnostic and cannot carry an ovstage sync -- so
                    // the drain would borrow and scatter the column before OVStage finished producing it
                    // (`scatterGroup(slot, {})`), and would also mistake any non-CPU DLPack device kind for a
                    // CUDA ordinal. Deliver only a host column; a device column falls through to the per-key
                    // read path. Enabling the zero-copy device borrow first needs that producer sync plumbed
                    // to `scatterGroup` (a source-agnostic device-sync on `ChangeBatch`, or a feed-side wait).
                    if (ct != ColumnType::eNone && t.device.device_type == kDLCPU)
                        batch.values =
                            ColumnView{ ct, -1, rows, static_cast<const uint8_t*>(t.data) + t.byte_offset };
                }
                // Ragged array path: one host float tensor PER PRIM (tensor_count == prims.count), each
                // prim's own element count in its shape. Gather them into one contiguous CSR column plus
                // per-prim offsets, so a per-vertex / per-particle consumer (deformable / particle) reads
                // a flat column with variable-length rows. Host, non-sparse, aligned keys only -- exactly
                // the conditions the dense path needs, plus is_array. (ovstage arrays are host today; a
                // device array would need a D2H gather this does not do.)
                else if (g.is_array && g.data.tensor_count == g.prims.count && g.data.tensors &&
                         g.data.index_map == nullptr && g.data.mask == nullptr &&
                         mGroupKeys.size() == g.prims.count && g.prims.count > 0 &&
                         g.data.tensors[0].dtype.code == kDLFloat && g.data.tensors[0].dtype.bits == 32 &&
                         g.data.tensors[0].device.device_type == kDLCPU)
                {
                    // Typed reads put tuple components in trailing shape axes, not dtype.lanes.
                    // Normalize only the borrowed descriptor; the payload is still gathered below.
                    DLTensor first = g.data.tensors[0];
                    int64_t firstLayout[2]{};
                    bool ragOk = first.dtype.lanes != 1 ||
                                 ovstage_tensor_to_write(&g.data.tensors[0], &first, firstLayout) == OVSTAGE_OK;
                    const uint32_t lanes = first.dtype.lanes;
                    const ColumnType ct = columnTypeOf(first.dtype, lanes);
                    ragOk = ragOk && ct != ColumnType::eNone;
                    raggedOffsets.reserve(g.prims.count + 1);
                    raggedOffsets.push_back(0);
                    for (uint32_t p = 0; ragOk && p < g.prims.count; ++p)
                    {
                        DLTensor t = g.data.tensors[p];
                        int64_t layout[2]{};
                        if (t.dtype.lanes == 1 &&
                            ovstage_tensor_to_write(&g.data.tensors[p], &t, layout) != OVSTAGE_OK)
                        {
                            ragOk = false;
                            break;
                        }
                        // Uniform float32 element width across prims, host-resident, and COMPACT (row-major
                        // packed): the gather below copies each row as elems*lanes*sizeof(float) bytes. The
                        // entry check above inspected only tensors[0], but a wildcard family query can surface
                        // the same custom attribute authored as different USD types on different prims (e.g.
                        // float3[] and double3[]) in one group -- so re-check dtype.code/bits per row, not just
                        // lanes: a double3[] row otherwise passes and has half its 8-byte payload copied and
                        // read as float32, silent corruption the correct CSR offset would not flag. Even
                        // after normalizing tuple axes, a non-compact or shape-malformed row cannot be
                        // copied as packed. Explicit compact
                        // strides are valid and required by newer DLPack read producers.
                        if (t.dtype.code != kDLFloat || t.dtype.bits != 32 ||
                            t.dtype.lanes != lanes || t.device.device_type != kDLCPU ||
                            !detail::isCompactReadTensor(t))
                        {
                            ragOk = false;
                            break;
                        }
                        const int64_t elems = lanes > 0 ? totalElements(t) / static_cast<int64_t>(lanes) : 0;
                        // A malformed tensor (a negative shape dimension) makes elems negative; the
                        // static_cast<size_t> below would then wrap `bytes` to an enormous value and the
                        // insert would read far past the buffer. A count past UINT32_MAX would also overflow
                        // the uint32_t CSR offset. Reject either to the parse-layer fallback.
                        if (elems < 0 || elems > static_cast<int64_t>(UINT32_MAX) || (elems > 0 && !t.data))
                        {
                            ragOk = false;
                            break;
                        }
                        // Bounding each row is not enough: the CUMULATIVE CSR offset is a uint32_t and the
                        // running total can still wrap. Reject before `back() + elems` overflows it.
                        if (static_cast<int64_t>(raggedOffsets.back()) + elems > static_cast<int64_t>(UINT32_MAX))
                        {
                            ragOk = false;
                            break;
                        }
                        const size_t bytes = static_cast<size_t>(elems) * lanes * sizeof(float);
                        if (bytes != 0)
                        {
                            const uint8_t* srcBytes = static_cast<const uint8_t*>(t.data) + t.byte_offset;
                            raggedBuf.insert(raggedBuf.end(), srcBytes, srcBytes + bytes);
                        }
                        raggedOffsets.push_back(raggedOffsets.back() + static_cast<uint32_t>(elems));
                    }
                    if (ragOk)
                    {
                        batch.values = ColumnView{ ct, -1, raggedOffsets.back(), raggedBuf.data() };
                        batch.valueRowOffsets = raggedOffsets.data();
                    }
                }

                mSource.seedBucketFromReadGroup(property, g, mGroupKeys.empty() ? nullptr : mGroupKeys.data(),
                                                mGroupKeys.size(), /*append=*/false, mGroupListPaths,
                                                mGroupListCount);
                valueBatchDispatchCounter().fetch_add(1, std::memory_order_relaxed);
                // A consumer that committed part of the batch and then failed returns false; fail
                // the drain closed so updateFromOvStage holds the cursor and redelivers the range.
                ok = dispatch(batch) && ok;
                mSource.clearBucket();
            }
            cursor.keep(mDeferredGroups);
        }
        // A truncated read fails the drain and the range is redelivered: do not dispatch
        // the partial coalesced columns.
        if (err == OVSTAGE_ERROR_END_OF_ITERATION)
            flushCoalesced();
        releaseDeferredGroups();
        waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
        return err == OVSTAGE_ERROR_END_OF_ITERATION;
    };

    // Device value drain. On a DirectGPU sim, read the ovstage-authored velocity columns ON DEVICE (via the
    // ovstage.placement device read) over the drained ordinal range and deliver each as a device column the
    // scatter applies with no host copy: linear velocity borrows in place, angular velocity is deg->rad
    // scaled on device. The read is a change-range read (AC-16) -- only the rows written this range -- and
    // the device path engages for the dense pattern: a DirectGPU sim that authors every body each drain
    // rewrites every row of the column. Every other attribute stays on the host change-range path below.
    const bool deviceReadAvail = mDeviceOrdinal >= 0 && deviceReadAvailable(mInstance);
    // The attributes served on device on a DirectGPU sim: rigid-body velocity + angular velocity (fixed-size
    // vec3 columns) and the single-DOF articulation joint drive/state DOF scalars. The ovstage device read
    // serves all of them; a token the read returns index-mapped or on the host (a sparse multi-axis set that
    // does not cover every queried prim, or a partial rewrite) stays on the host change-range path below, like
    // any partial rigid column, and drains through applyArticulationBatch's host scatter -- never dropped.
    std::vector<ovx_token_t> deviceEligibleTokens;
    if (deviceReadAvail)
        deviceEligibleTokens = { internOvxToken("physics:velocity"),
                                 internOvxToken("physics:angularVelocity"),
                                 internOvxToken("drive:linear:physics:targetPosition"),
                                 internOvxToken("drive:angular:physics:targetPosition"),
                                 internOvxToken("drive:linear:physics:targetVelocity"),
                                 internOvxToken("drive:angular:physics:targetVelocity"),
                                 internOvxToken("state:linear:physics:position"),
                                 internOvxToken("state:angular:physics:position"),
                                 internOvxToken("state:linear:physics:velocity"),
                                 internOvxToken("state:angular:physics:velocity") };
    const auto authoredValueAttr = [&](ovx_token_t tok) {
        return tok != OVX_INVALID_TOKEN &&
               std::find(mValueReadAttrs.begin(), mValueReadAttrs.end(), tok) != mValueReadAttrs.end();
    };

    // Per-token device-serve outcome (reset before each token's read). An attribute is served on device
    // -- and so removed from the host change-range read below -- ONLY when the device read actually
    // delivered it as a dense device (kDLCUDA) column. The placement read is permissive: it materializes a
    // host-authored dense column onto CUDA, so host authoring alone does NOT keep a device-eligible column
    // on the host read. A group comes back undispatched (kDLCPU or non-dense) only when it is index-mapped
    // or masked (a partial rewrite) or hits ovstage's single-prim placement downgrade -- that must stay on
    // the host path, not be silently dropped between the two.
    bool tokDeviceServed = false;
    bool tokNeedsHostRead = false;

    auto deviceRead = [&](const PathListHandle& h, const std::vector<ovx_token_t>& attrs,
                          ovstage_api_status_t& err) -> bool
    {
        // Change-range [ord0, ord1], like the host read: only the rows actually written in this range,
        // so a body nobody rewrote this drain keeps its simulated state instead of being overwritten with
        // its stale last-authored value (a latest/whole-superset read re-applied every row every drain).
        ovstage_ordinal_range_t range{};
        range.start_ordinal = ord0;
        range.end_ordinal = ord1;
        range.has_start_ordinal = true;

        ovstage_placement_t placement = OVSTAGE_PLACEMENT_INIT;
        placement.device.device_type = kDLCUDA;
        placement.device.device_id = mDeviceOrdinal;
        ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
        ovstage_enqueue_result_t re = ovstage_read_attributes_on_device(
            mInstance, h.query, attrs.data(), attrs.size(), range, &placement, &rh);
        err = re.status;
        if (re.status != OVSTAGE_OK)
            return false;
        waitAndRelease(mInstance, re);

        ReadColumnCursor cursor(mInstance, rh);
        ReadGroupColumn g{};
        for (;;)
        {
            err = cursor.next(g);
            if (err != OVSTAGE_OK)
                break;
            const bool dense = !g.is_delete && g.data.tensor_count > 0 && g.data.tensors &&
                               g.data.tensors[0].data && !g.is_array && g.data.index_map == nullptr &&
                               g.data.mask == nullptr && g.prims.count > 0;
            bool dispatched = false;
            if (dense && g.data.tensors[0].device.device_type == kDLCUDA)
            {
                collectGroupKeys(g, mGroupKeys, &h);
                if (mGroupKeys.size() == g.prims.count)
                {
                    const DLTensor& t = g.data.tensors[0];
                    const uint32_t rows = g.prims.count;
                    const int64_t comps = totalElements(t) / static_cast<int64_t>(rows);
                    const ColumnType ct = columnTypeOf(t.dtype, comps);
                    if (ct != ColumnType::eNone)
                    {
                        ChangeBatch batch;
                        batch.isDelete = false;
                        batch.numChanges = mGroupKeys.size();
                        batch.keys = ColumnView{ ColumnType::eObjectKey, -1, mGroupKeys.size(), mGroupKeys.data() };
                        batch.property = propertyTokenFor(g.attribute);
                        // Borrow the device column in place; carry ovstage's producer event so the
                        // scatter waits on it before reading (ColumnView is CUDA-free, so the sync
                        // handles pass as opaque uintptr_t values). No bucket seed: a device tensor
                        // cannot be read host-side, and this dense group's borrow covers every row it
                        // carries, so those rows need no per-key host fallback.
                        batch.values = ColumnView{ ct, mDeviceOrdinal, rows,
                                                   static_cast<const uint8_t*>(t.data) + t.byte_offset,
                                                   static_cast<uint64_t>(g.data.cuda_sync.stream),
                                                   static_cast<uint64_t>(g.data.cuda_sync.wait_event) };
                        ok = dispatch(batch) && ok;
                        dispatched = true; // a device column reached the consumer for these rows
                    }
                }
            }
            // Any group NOT dispatched on device -- a host (kDLCPU) column, an index-mapped or masked
            // chunk, a delete, a key-count mismatch, or an unmapped dtype -- keeps the WHOLE attribute on
            // the host change-range read below so its rows are never dropped between the two paths. The
            // read runs per handle (base + delta) and can split into a dispatched group plus an undispatched
            // one for the same attribute; the host read then re-applies the device-served rows idempotently.
            if (dispatched)
                tokDeviceServed = true;
            else
                tokNeedsHostRead = true;
            cursor.keep(mDeferredGroups);
        }
        releaseDeferredGroups();
        waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
        return err == OVSTAGE_ERROR_END_OF_ITERATION;
    };

    // Serve each device-eligible attribute on device when possible; a device read that cannot fully
    // cover (partial column / hook rejection) leaves that attribute on the host change-range path below.
    // Re-applying the same authored value is idempotent, so a partial device serve is harmless.
    std::vector<ovx_token_t> servedOnDevice;
    if (deviceReadAvail)
        for (ovx_token_t tok : deviceEligibleTokens)
        {
            // Change-driven: read + scatter this attribute ONLY when it actually changed in this range
            // (its write floor reached ord0). An attribute nobody wrote this drain is skipped entirely --
            // no re-read, no re-scatter of state PhysX already holds. The read below is a change-range read
            // (AC-16): only the rows written in [ord0, ord1] come back, so a body nobody rewrote this range
            // is not re-applied and keeps its simulated state.
            if (!authoredValueAttr(tok) || !hasAttributeFloorAtOrAfter(tok, ord0))
                continue;
            tokDeviceServed = false;
            tokNeedsHostRead = false;
            const bool readOk =
                readPathListQuery(mKnownPhysicsQuery, std::vector<ovx_token_t>{ tok }, ord0, deviceRead);
            // Removed from the host read only when a device column was served AND no group needed the host
            // read; otherwise it stays in the host read set (a host-authored, partial, or mixed column is
            // not dropped -- the delta handle that came back index-mapped is the case this guards).
            if (readOk && tokDeviceServed && !tokNeedsHostRead)
                servedOnDevice.push_back(tok);
        }

    // The device serve above is the whole value apply for the device-resident attributes. Every other
    // authored attribute (host-authored, or a device-eligible column the read returned on the host) goes
    // through the host change-range read below so a value that changed this range is never dropped.
    std::vector<ovx_token_t> hostAttrs;
    for (ovx_token_t tok : mValueReadAttrs)
        if (std::find(servedOnDevice.begin(), servedOnDevice.end(), tok) == servedOnDevice.end())
            hostAttrs.push_back(tok);

    bool readOk = true;
    if (!hostAttrs.empty())
        readOk = readPathListQuery(mKnownPhysicsQuery, hostAttrs, ord0, runRead, true);
    return readOk && ok;
}

// Release cost depends on order: releasing each group while later ones are still
// pending costs O(remaining) per call (31 ms for 4096 groups), releasing them all after the
// loop last-fetched-first costs ~1.6 us each. Groups stay valid until released, so nothing
// read from them (bucket tensors included) dangles before the flush that follows.
void OvstageChangeFeed::releaseDeferredGroups()
{
    std::reverse(mDeferredGroups.begin(), mDeferredGroups.end());
    ovstage_release_groups(mInstance, mDeferredGroups.data(), mDeferredGroups.size());
    mDeferredGroups.clear();
    mListMemo.clear(); // list handles may be recycled once the groups are gone
}

bool OvstageChangeFeed::drainRange(uint64_t ord0, uint64_t ord1)
{
    if (!mBaselineInitialized)
        return false;
    if (!mEnabled || !mInstance || !mDict)
        return true;

    refreshCustomFamilies();
    mAddedPhysicsPaths.clear();
    mAddedTransformPaths.clear();
    mCollectAddedPaths = false;

    // The source's ordinal-gated reads (child cache under a load cache, whole-stage schema
    // reads) target the range being drained, not the attach ordinal.
    mSource.setReadOrdinal(static_cast<ovstage_ordinal_t>(ord1));
    struct SharedInputGuard
    {
        OvstageSource& source;
        bool keep = false;
        ~SharedInputGuard()
        {
            if (!keep)
                source.clearSharedPhysicsInputs();
        }
    } sharedInputGuard{ mSource };

    // Existence answers are exact as of the previous drain only. A prim deleted in this range
    // appears in no structural read (query membership is live prims), so nothing below could
    // forget it: start from an empty memo. The records this drain seeds survive to the flush.
    mSource.clearExistsMemo();
    // Likewise the stage-wide column vocabulary: a column first authored in this range must be
    // seen by the relationship reads the drain triggers (rebuilt once, on first use).
    mSource.invalidateAttributeVocabulary();

    // ovstage only guarantees exact change membership at or above the retained-history
    // frontier; below it, history may be coalesced or discarded. Fail closed if `ord0`
    // predates the frontier (or it can't be determined) rather than delivering a partial
    // diff and letting updateFromOvStage advance the cursor past lost changes.
    if (!rangeWithinRetainedHistory(ord0))
        return false;
    // A skipped ordinal may carry known-prim edits no read of this drain covers (the known-path
    // read is ranged): the patched memos are suspect until a rescan drops them.
    if (ord0 != mLastDrainedOrd1 + 1)
        mRangeGapSinceDrop = true;

    bool ok = true;
    {
        CARB_PROFILE_ZONE(0, "OvstageChangeFeed::structuralChangeCheck");
        ok = reconcileStructuralChanges(ord0, ord1) && ok;
    }

    {
        CARB_PROFILE_ZONE(0, "OvstageChangeFeed::cachedChangeRead");
        // The family queries feed the growth classification and the discovery fallback.
        queryForFilter("usd-schemas", OVSTAGE_FILTER_OP_CONTAINS, mPhysicsSchemas.data(), mPhysicsSchemas.size(),
                       mSchemaFamilyQuery);
        queryForFilter(conv::kUsdPrimType, OVSTAGE_FILTER_OP_IN, mPhysicsPrimTypes.data(), mPhysicsPrimTypes.size(),
                       mTypeFamilyQuery);
        ok = readValueChanges(ord0, ord1) && ok;
        ok = readKnownTransformChanges(ord0, ord1) && ok;
    }

    if (mGroupComplete)
        mGroupComplete();
    mSource.clearBucket();

    // A failed drain is redelivered from ord0; whatever it half-patched is suspect either way.
    if (ok)
    {
        mLastDrainedOrd1 = ord1;
        mPendingAuthoredTypeChanges.clear();
        sharedInputGuard.keep = true;
    }
    else
        mRangeGapSinceDrop = true;
    return ok;
}
} // namespace omni::physics::ovstage
