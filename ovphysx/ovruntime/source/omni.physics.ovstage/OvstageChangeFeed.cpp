// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include "OvstageChangeFeed.h"

#include "OvstageSource.h"

#include <carb/profiler/Profile.h>

#include <array>
#include <cstring>
#include <string>
#include <string_view>
#include <unordered_set>

namespace omni::physics::ovstage
{
namespace
{

ovx_string_t ovxStr(const char* s)
{
    return { s, std::string_view(s).size() };
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

} // namespace

OvstageChangeFeed::OvstageChangeFeed(OvstageSource& source, ovstage_instance_t* instance, ovx_path_dictionary_t* dict)
    : mSource(source), mInstance(instance), mDict(dict)
{
    seedKnownPhysicsPaths();
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

void OvstageChangeFeed::dispatch(const ChangeBatch& batch) const
{
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
            reg.cb(b);
        }
    }
}

void OvstageChangeFeed::seedKnownPhysicsPaths()
{
    if (!mInstance || !mDict)
        return;

    const ovx_string_t schemaVals[] = { ovxStr("PhysicsRigidBodyAPI"), ovxStr("PhysicsCollisionAPI"),
                                        ovxStr("PhysicsMaterialAPI"), ovxStr("PhysicsArticulationRootAPI") };
    const ovx_string_t typeVals[] = { ovxStr("PhysicsScene"),         ovxStr("PhysicsFixedJoint"),
                                      ovxStr("PhysicsRevoluteJoint"), ovxStr("PhysicsPrismaticJoint"),
                                      ovxStr("PhysicsSphericalJoint"), ovxStr("PhysicsDistanceJoint"),
                                      ovxStr("PhysicsJoint"),          ovxStr("PhysxPhysicsGearJoint"),
                                      ovxStr("PhysxPhysicsRackAndPinionJoint"), ovxStr("PhysicsCollisionGroup"),
                                      ovxStr("PointInstancer") };

    std::unordered_set<std::string> physicsPaths;
    std::unordered_set<std::string> transformPaths;
    collectKnownPathsFromFilter("usd-schemas", OVSTAGE_FILTER_OP_CONTAINS, schemaVals,
                                sizeof(schemaVals) / sizeof(schemaVals[0]), mSchemaFamilyQuery,
                                physicsPaths, transformPaths);
    collectKnownPathsFromFilter("usd-prim-type", OVSTAGE_FILTER_OP_IN, typeVals,
                                sizeof(typeVals) / sizeof(typeVals[0]), mTypeFamilyQuery,
                                physicsPaths, transformPaths);

    mKnownPhysicsPaths = std::move(physicsPaths);
    mKnownTransformPaths = std::move(transformPaths);
    mKnownPhysicsQueryDirty = true;
    mKnownTransformQueryDirty = true;
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

    ovstage_ordinal_range_t range{};
    range.end_ordinal = ~ovstage_ordinal_t(0);
    range.has_start_ordinal = false;

    bool ok = true;
    ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
    const ovstage_enqueue_result_t re = ovstage_read_attributes(mInstance, q, &attrTok, 1, range, &rh);
    if (re.status == OVSTAGE_OK)
    {
        waitAndRelease(mInstance, re);
        ovstage_read_group_t g{};
        ovstage_api_status_t fetchErr;
        while ((fetchErr = ovstage_fetch_read_next(mInstance, rh, OVSTAGE_TIMEOUT_INFINITE, &g)) == OVSTAGE_OK)
        {
            if (!g.is_delete)
            {
                std::vector<ObjectKey> groupNewKeys =
                    collectPathsFromGroup(g, physicsPaths, transformPaths, oldPhysicsPaths);
                if (newKeys)
                    newKeys->insert(newKeys->end(), groupNewKeys.begin(), groupNewKeys.end());
            }
            ovstage_release_group(mInstance, &g);
        }
        if (fetchErr != OVSTAGE_ERROR_END_OF_ITERATION)
            ok = false;
        waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
    }
    else
    {
        ok = false;
    }

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
}

void OvstageChangeFeed::dirtyCachedFamilyQueries()
{
    releaseCachedQuery(mSchemaFamilyQuery);
    releaseCachedQuery(mTypeFamilyQuery);
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
    query.dirty = false;

    if (!mInstance || !mDict || !attrName)
        return OVSTAGE_INVALID_QUERY_HANDLE;

    ovstage_predicate_t pred{};
    pred.attribute.token = 0;
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

    std::unordered_set<ovx_token_t> discovered;
    ovstage_query_result_t qr{};
    if (ovstage_fetch_query_result(mInstance, query.query, OVSTAGE_TIMEOUT_INFINITE, &qr) == OVSTAGE_OK)
    {
        for (size_t i = 0; i < qr.attribute_count; ++i)
        {
            ovx_string_t s{};
            if (ovx_path_dictionary_token_to_string(mDict, qr.attributes[i], &s) != OVX_OK || !s.ptr || s.length == 0)
                continue;
            if (s.ptr[0] == '_' || (s.length >= 4 && std::strncmp(s.ptr, "usd-", 4) == 0))
                continue;
            if (discovered.insert(qr.attributes[i]).second)
                query.discoveredValueAttributes.push_back(qr.attributes[i]);
        }
        ovstage_release_query_result(mInstance, &qr);
    }

    return query.query;
}

ovstage_query_handle_t OvstageChangeFeed::queryForPaths(const std::unordered_set<std::string>& paths,
                                                        ovstage_query_handle_t& query,
                                                        bool& dirty)
{
    if (!dirty && query != OVSTAGE_INVALID_QUERY_HANDLE)
        return query;

    releaseCachedQuery(query);
    dirty = false;

    if (paths.empty() || !mInstance || !mDict)
        return OVSTAGE_INVALID_QUERY_HANDLE;

    std::vector<std::string> pathStrings;
    pathStrings.reserve(paths.size());
    for (const std::string& path : paths)
        pathStrings.push_back(path);

    std::vector<ovx_string_t> pathViews;
    pathViews.reserve(pathStrings.size());
    for (const std::string& path : pathStrings)
        pathViews.push_back(ovx_string_t{ path.data(), path.size() });

    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
    if (ovx_path_dictionary_create_path_list_from_strings(mDict, pathViews.data(), pathViews.size(), &list) != OVX_OK)
        return OVSTAGE_INVALID_QUERY_HANDLE;

    ovstage_query_handle_t newQuery = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(mInstance, list, &newQuery) == OVSTAGE_OK)
        query = newQuery;

    ovx_path_dictionary_destroy_path_list(mDict, list);
    return query;
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
    if (!raw)
        return {};
    const auto it = mResolvedPathKeys.find(raw);
    if (it != mResolvedPathKeys.end())
        return it->second;

    ObjectKey key = mSource.canonicalKey(ObjectKey{ raw });
    if (!key.valid())
        key = ObjectKey{ raw };
    mResolvedPathKeys[raw] = key;
    if (key.valid())
        mResolvedPathKeys[key.handle] = key;
    return key;
}

void OvstageChangeFeed::rebuildRegisteredAttributeTokens()
{
    if (!mRegisteredAttributeTokensDirty)
        return;

    mRegisteredAttributeTokens.clear();
    mHasWildcardRegistration = false;

    std::unordered_set<ovx_token_t> seen;
    for (const Registration& reg : mRegistrations)
    {
        if (!reg.objectType.valid() && !reg.prop.valid())
            mHasWildcardRegistration = true;

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

std::vector<ovx_token_t> OvstageChangeFeed::buildReadAttributes(const CachedFilterQuery& query)
{
    rebuildRegisteredAttributeTokens();

    std::vector<ovx_token_t> attrs;
    attrs.reserve(mRegisteredAttributeTokens.size() +
                  (mHasWildcardRegistration ? query.discoveredValueAttributes.size() : 0));
    std::unordered_set<ovx_token_t> seen;
    auto addAttr = [&](ovx_token_t tok)
    {
        if (tok != OVX_INVALID_TOKEN && seen.insert(tok).second)
            attrs.push_back(tok);
    };

    for (ovx_token_t tok : mRegisteredAttributeTokens)
        addAttr(tok);
    if (mHasWildcardRegistration)
        for (ovx_token_t tok : query.discoveredValueAttributes)
            addAttr(tok);

    return attrs;
}

std::vector<ObjectKey> OvstageChangeFeed::collectPathsFromGroup(
    const ovstage_read_group_t& group,
    std::unordered_set<std::string>& physicsPaths,
    std::unordered_set<std::string>& transformPaths,
    const std::unordered_set<std::string>* oldPhysicsPaths)
{
    std::vector<ObjectKey> newKeys;
    const ovx_primpath_t* paths = nullptr;
    size_t pathCount = 0;
    if (ovx_path_dictionary_get_paths(mDict, group.prims.list, &paths, &pathCount) != OVX_OK || !paths)
        return newKeys;

    for (uint32_t i = 0; i < group.prims.count; ++i)
    {
        const uint32_t idx = group.prims.index_map ? group.prims.index_map[i] : (group.prims.offset + i);
        if (idx >= pathCount)
            continue;
        const ovx_primpath_t raw = paths[idx];
        const std::string_view path = mSource.sourceKeyToString(ObjectKey{ raw });
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
    const ovstage_enqueue_result_t re = ovstage_get_attribute_write_floor(mInstance, ovx_string_or_token_t{ attr, {} }, &q);
    if (re.status != OVSTAGE_OK || q == OVSTAGE_INVALID_ORDINAL_QUERY_HANDLE)
        return true;
    waitAndRelease(mInstance, re);

    ovstage_ordinal_t floor = 0;
    bool ok = ovstage_fetch_ordinal(mInstance, q, OVSTAGE_TIMEOUT_INFINITE, &floor) == OVSTAGE_OK;
    waitAndRelease(mInstance, ovstage_release_ordinal_query(mInstance, q));
    return !ok || floor >= ordinal;
}

bool OvstageChangeFeed::anyAttributeFloorAtOrAfter(const std::vector<ovx_token_t>& attrs, uint64_t ordinal) const
{
    for (ovx_token_t attr : attrs)
    {
        if (hasAttributeFloorAtOrAfter(attr, ordinal))
            return true;
    }
    return false;
}

bool OvstageChangeFeed::structuralAttributesChanged(uint64_t ordinal)
{
    const ovx_token_t schemasTok = internOvxToken(conv::kUsdSchemas);
    const ovx_token_t primTypeTok = internOvxToken(conv::kUsdPrimType);
    return (schemasTok != OVX_INVALID_TOKEN && hasAttributeFloorAtOrAfter(schemasTok, ordinal)) ||
           (primTypeTok != OVX_INVALID_TOKEN && hasAttributeFloorAtOrAfter(primTypeTok, ordinal));
}

bool OvstageChangeFeed::rememberTransformPathAndAncestors(std::string_view path)
{
    return rememberTransformPathAndAncestors(mKnownTransformPaths, path);
}

bool OvstageChangeFeed::rememberTransformPathAndAncestors(std::unordered_set<std::string>& paths, std::string_view path)
{
    if (path.empty())
        return false;

    bool changed = false;
    std::string cur(path);
    while (!cur.empty())
    {
        changed = paths.insert(cur).second || changed;
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

void OvstageChangeFeed::forgetPathsFromGroup(const ovstage_read_group_t& group)
{
    const ovx_primpath_t* paths = nullptr;
    size_t pathCount = 0;
    if (ovx_path_dictionary_get_paths(mDict, group.prims.list, &paths, &pathCount) != OVX_OK || !paths)
        return;

    for (uint32_t i = 0; i < group.prims.count; ++i)
    {
        const uint32_t idx = group.prims.index_map ? group.prims.index_map[i] : (group.prims.offset + i);
        if (idx >= pathCount)
            continue;
        const std::string_view path = mSource.sourceKeyToString(ObjectKey{ paths[idx] });
        if (!path.empty() && mKnownPhysicsPaths.erase(std::string(path)) != 0)
            mKnownPhysicsQueryDirty = true;
    }
}

bool OvstageChangeFeed::reconcileStructuralChanges(uint64_t ord0, uint64_t ord1)
{
    std::unordered_set<std::string> deletedPaths;
    bool sawKnownStructuralChange = false;
    if (!readKnownStructuralChanges(ord0, ord1, &sawKnownStructuralChange, deletedPaths))
        return false;

    const bool structuralChange = sawKnownStructuralChange || structuralAttributesChanged(ord0);
    if (!structuralChange)
        return true;

    mSource.clearSchemaCache();
    mResolvedPathKeys.clear();

    dirtyCachedFamilyQueries();

    const ovx_string_t schemaVals[] = { ovxStr("PhysicsRigidBodyAPI"), ovxStr("PhysicsCollisionAPI"),
                                        ovxStr("PhysicsMaterialAPI"), ovxStr("PhysicsArticulationRootAPI") };
    const ovx_string_t typeVals[] = { ovxStr("PhysicsScene"),         ovxStr("PhysicsFixedJoint"),
                                      ovxStr("PhysicsRevoluteJoint"), ovxStr("PhysicsPrismaticJoint"),
                                      ovxStr("PhysicsSphericalJoint"), ovxStr("PhysicsDistanceJoint"),
                                      ovxStr("PhysicsJoint"),          ovxStr("PhysxPhysicsGearJoint"),
                                      ovxStr("PhysxPhysicsRackAndPinionJoint"), ovxStr("PhysicsCollisionGroup"),
                                      ovxStr("PointInstancer") };

    // The refresh only inserts into the fresh sets below, so it can borrow the
    // previous set instead of copying every path string.
    const std::unordered_set<std::string>& oldPhysicsPaths = mKnownPhysicsPaths;

    std::unordered_set<std::string> freshPhysicsPaths;
    std::unordered_set<std::string> freshTransformPaths;
    std::vector<ObjectKey> newKeys;
    const bool schemaScanOk = collectKnownPathsFromFilter(
        "usd-schemas", OVSTAGE_FILTER_OP_CONTAINS, schemaVals, sizeof(schemaVals) / sizeof(schemaVals[0]),
        mSchemaFamilyQuery, freshPhysicsPaths, freshTransformPaths, &newKeys, &oldPhysicsPaths);
    const bool typeScanOk = collectKnownPathsFromFilter(
        "usd-prim-type", OVSTAGE_FILTER_OP_IN, typeVals, sizeof(typeVals) / sizeof(typeVals[0]), mTypeFamilyQuery,
        freshPhysicsPaths, freshTransformPaths, &newKeys, &oldPhysicsPaths);
    const bool ok = schemaScanOk && typeScanOk;

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
    }
    else
    {
        // Do not turn a failed/partial refresh into synthetic removals or
        // replace the last known-good path sets with incomplete data.
        return false;
    }

    std::vector<ObjectKey> deleteKeys;
    deleteKeys.reserve(deletedPaths.size());
    for (const std::string& deletedPath : deletedPaths)
    {
        const ObjectKey key = mSource.findByPath(deletedPath);
        if (key.valid())
            deleteKeys.push_back(key);
    }

    mKnownPhysicsPaths = std::move(freshPhysicsPaths);
    mKnownTransformPaths = std::move(freshTransformPaths);
    mKnownPhysicsQueryDirty = true;
    mKnownTransformQueryDirty = true;

    if (!deleteKeys.empty())
    {
        ChangeBatch batch;
        batch.isDelete = true;
        batch.numChanges = deleteKeys.size();
        batch.keys = ColumnView{ ColumnType::eObjectKey, -1, deleteKeys.size(), deleteKeys.data() };
        dispatch(batch);
    }

    if (!newKeys.empty())
    {
        const TokenId typeNameField = mSource.internToken("typeName");
        ChangeBatch batch;
        batch.isDelete = false;
        batch.numChanges = newKeys.size();
        batch.keys = ColumnView{ ColumnType::eObjectKey, -1, newKeys.size(), newKeys.data() };
        batch.property = TokenId{};
        batch.values = ColumnView{ ColumnType::eToken, -1, 1, &typeNameField };
        dispatch(batch);
    }

    return ok;
}

bool OvstageChangeFeed::readKnownStructuralChanges(uint64_t ord0,
                                                   uint64_t ord1,
                                                   bool* sawChange,
                                                   std::unordered_set<std::string>& deletedPaths)
{
    if (sawChange)
        *sawChange = false;
    deletedPaths.clear();
    if (mKnownPhysicsPaths.empty())
        return true;

    std::array<ovx_token_t, 2> attrToks{};
    size_t attrCount = 0;
    const ovx_token_t schemasTok = internOvxToken(conv::kUsdSchemas);
    const ovx_token_t primTypeTok = internOvxToken(conv::kUsdPrimType);
    if (schemasTok != OVX_INVALID_TOKEN)
        attrToks[attrCount++] = schemasTok;
    if (primTypeTok != OVX_INVALID_TOKEN)
        attrToks[attrCount++] = primTypeTok;
    if (attrCount == 0)
        return false;

    ovstage_query_handle_t q = queryForPaths(mKnownPhysicsPaths, mKnownPhysicsQuery, mKnownPhysicsQueryDirty);
    if (q == OVSTAGE_INVALID_QUERY_HANDLE)
        return false;

    bool ok = true;
    if (attrCount != 0)
    {
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
            ovstage_read_group_t g{};
            ovstage_api_status_t fetchErr;
            while ((fetchErr = ovstage_fetch_read_next(mInstance, rh, OVSTAGE_TIMEOUT_INFINITE, &g)) == OVSTAGE_OK)
            {
                if (sawChange)
                    *sawChange = true;
                const ovx_primpath_t* groupPaths = nullptr;
                size_t groupPathCount = 0;
                if (ovx_path_dictionary_get_paths(mDict, g.prims.list, &groupPaths, &groupPathCount) != OVX_OK)
                    groupPaths = nullptr;

                if (g.is_delete)
                {
                    if (groupPaths)
                    {
                        for (uint32_t i = 0; i < g.prims.count; ++i)
                        {
                            const uint32_t idx = g.prims.index_map ? g.prims.index_map[i] : (g.prims.offset + i);
                            if (idx >= groupPathCount)
                                continue;
                            const std::string_view path = mSource.sourceKeyToString(ObjectKey{ groupPaths[idx] });
                            if (path.empty())
                                continue;
                            deletedPaths.insert(std::string(path));
                        }
                    }
                }
                else if (groupPaths && (g.attribute == schemasTok || g.attribute == primTypeTok))
                {
                    std::vector<ObjectKey> keys;
                    keys.reserve(g.prims.count);
                    for (uint32_t i = 0; i < g.prims.count; ++i)
                    {
                        const uint32_t idx = g.prims.index_map ? g.prims.index_map[i] : (g.prims.offset + i);
                        if (idx >= groupPathCount)
                            continue;
                        keys.push_back(resolveGroupPathKey(groupPaths[idx]));
                    }
                    if (!keys.empty())
                    {
                        const TokenId apiSchemasField = mSource.internToken("apiSchemas");
                        const TokenId typeNameField = mSource.internToken("typeName");
                        const TokenId& field = (g.attribute == schemasTok) ? apiSchemasField : typeNameField;

                        ChangeBatch batch;
                        batch.isDelete = false;
                        batch.numChanges = keys.size();
                        batch.keys = ColumnView{ ColumnType::eObjectKey, -1, keys.size(), keys.data() };
                        batch.property = TokenId{};
                        batch.values = ColumnView{ ColumnType::eToken, -1, 1, &field };
                        dispatch(batch);
                    }
                }
                ovstage_release_group(mInstance, &g);
            }
            if (fetchErr != OVSTAGE_ERROR_END_OF_ITERATION)
                ok = false;
            waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
        }
        else
        {
            ok = false;
        }
    }

    return ok;
}

bool OvstageChangeFeed::readKnownTransformChanges(uint64_t ord0, uint64_t ord1)
{
    if (mKnownTransformPaths.empty())
        return true;

    std::vector<ovx_token_t> attrToks;
    std::unordered_set<ovx_token_t> attrTokSet;
    auto addAttr = [&](const char* attrName)
    {
        ovx_token_t tok = OVX_INVALID_TOKEN;
        const ovx_string_t s = ovxStr(attrName);
        if (ovx_path_dictionary_intern_token(mDict, s, &tok) == OVX_OK && tok != OVX_INVALID_TOKEN &&
            attrTokSet.insert(tok).second)
        {
            attrToks.push_back(tok);
        }
    };
    addAttr(conv::kFabricWorldMatrix);
    addAttr(conv::kFabricLocalMatrix);
    addAttr(conv::kLocalTransform);
    addAttr(conv::kResetXformStack);
    if (attrToks.empty() || !anyAttributeFloorAtOrAfter(attrToks, ord0))
        return true;

    ovstage_query_handle_t q = queryForPaths(mKnownTransformPaths, mKnownTransformQuery, mKnownTransformQueryDirty);
    if (q == OVSTAGE_INVALID_QUERY_HANDLE)
        return true;

    bool ok = true;
    if (!attrToks.empty())
    {
        ovstage_ordinal_range_t range{};
        range.start_ordinal = ord0;
        range.end_ordinal = ord1;
        range.has_start_ordinal = true;

        ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
        const ovstage_enqueue_result_t re =
            ovstage_read_attributes(mInstance, q, attrToks.data(), attrToks.size(), range, &rh);
        if (re.status == OVSTAGE_OK)
        {
            waitAndRelease(mInstance, re);
            ovstage_read_group_t g{};
            ovstage_api_status_t fetchErr;
            while ((fetchErr = ovstage_fetch_read_next(mInstance, rh, OVSTAGE_TIMEOUT_INFINITE, &g)) == OVSTAGE_OK)
            {
                ovx_string_t attrStr{};
                TokenId property{};
                if (ovx_path_dictionary_token_to_string(mDict, g.attribute, &attrStr) == OVX_OK && attrStr.ptr)
                    property = mSource.internToken(std::string_view(attrStr.ptr, attrStr.length));

                std::vector<ObjectKey> keys;
                const ovx_primpath_t* gpaths = nullptr;
                size_t gcount = 0;
                if (ovx_path_dictionary_get_paths(mDict, g.prims.list, &gpaths, &gcount) == OVX_OK && gpaths)
                {
                    keys.reserve(g.prims.count);
                    for (uint32_t i = 0; i < g.prims.count; ++i)
                    {
                        const uint32_t idx = g.prims.index_map ? g.prims.index_map[i] : (g.prims.offset + i);
                        if (idx >= gcount)
                            continue;
                        keys.push_back(resolveGroupPathKey(gpaths[idx]));
                    }
                }

                if (g.is_delete)
                {
                    ovstage_release_group(mInstance, &g);
                    continue;
                }

                if (property == mSource.internToken(conv::kFabricWorldMatrix))
                {
                    mSource.seedBucketFromReadGroup(property, g, keys.empty() ? nullptr : keys.data(), keys.size());
                }

                ChangeBatch batch;
                batch.isDelete = false;
                batch.numChanges = keys.size();
                batch.keys = ColumnView{ ColumnType::eObjectKey, -1, keys.size(),
                                         keys.empty() ? nullptr : keys.data() };
                batch.property = property;
                dispatch(batch);
                ovstage_release_group(mInstance, &g);
            }
            if (fetchErr != OVSTAGE_ERROR_END_OF_ITERATION)
                ok = false;
            waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
        }
    }

    return ok;
}

bool OvstageChangeFeed::readCachedFamilyChanges(CachedFilterQuery& query,
                                                uint64_t ord0,
                                                uint64_t ord1,
                                                std::unordered_map<ovx_token_t, bool>& attributeFloorCache)
{
    if (query.query == OVSTAGE_INVALID_QUERY_HANDLE)
        return true;

    std::vector<ovx_token_t> attrToks = buildReadAttributes(query);
    if (!attrToks.empty())
    {
        std::vector<ovx_token_t> advancedAttrToks;
        advancedAttrToks.reserve(attrToks.size());
        for (ovx_token_t tok : attrToks)
        {
            auto floorIt = attributeFloorCache.find(tok);
            if (floorIt == attributeFloorCache.end())
            {
                bool advanced = hasAttributeFloorAtOrAfter(tok, ord0);
                floorIt = attributeFloorCache.emplace(tok, advanced).first;
            }
            if (floorIt->second)
                advancedAttrToks.push_back(tok);
        }
        attrToks.swap(advancedAttrToks);
    }
    if (attrToks.empty())
        return true;

    ovstage_ordinal_range_t range{};
    range.start_ordinal = ord0;
    range.end_ordinal = ord1;
    range.has_start_ordinal = true;

    bool ok = true;
    ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
    ovstage_enqueue_result_t re{};
    re = ovstage_read_attributes(mInstance, query.query, attrToks.data(), attrToks.size(), range, &rh);
    if (re.status == OVSTAGE_OK)
        waitAndRelease(mInstance, re);
    if (re.status == OVSTAGE_OK)
    {
        ovstage_read_group_t g{};
        ovstage_api_status_t fetchErr;
        while ((fetchErr = ovstage_fetch_read_next(mInstance, rh, OVSTAGE_TIMEOUT_INFINITE, &g)) == OVSTAGE_OK)
        {
            ovx_string_t attrStr{};
            TokenId property{};
            if (ovx_path_dictionary_token_to_string(mDict, g.attribute, &attrStr) == OVX_OK && attrStr.ptr)
                property = mSource.internToken(std::string_view(attrStr.ptr, attrStr.length));

            std::vector<ObjectKey> keys;
            const ovx_primpath_t* gpaths = nullptr;
            size_t gcount = 0;
            if (ovx_path_dictionary_get_paths(mDict, g.prims.list, &gpaths, &gcount) == OVX_OK && gpaths)
            {
                keys.reserve(g.prims.count);
                for (uint32_t i = 0; i < g.prims.count; ++i)
                {
                    const uint32_t idx = g.prims.index_map ? g.prims.index_map[i] : (g.prims.offset + i);
                    if (idx >= gcount)
                        continue;
                    keys.push_back(resolveGroupPathKey(gpaths[idx]));
                }
            }

            if (!g.is_delete)
            {
                ChangeBatch batch;
                batch.isDelete = false;
                batch.numChanges = keys.size();
                batch.keys = ColumnView{ ColumnType::eObjectKey, -1, keys.size(),
                                         keys.empty() ? nullptr : keys.data() };
                batch.property = property;

                if (g.data.tensor_count > 0 && g.data.tensors && g.data.tensors[0].data &&
                    !g.is_array && g.data.index_map == nullptr && g.data.mask == nullptr)
                {
                    const DLTensor& t = g.data.tensors[0];
                    const uint32_t rows = g.prims.count;
                    const int64_t comps = rows > 0 ? totalElements(t) / static_cast<int64_t>(rows) : totalElements(t);
                    const ColumnType ct = columnTypeOf(t.dtype, comps);
                    if (ct != ColumnType::eNone)
                    {
                        const int device = (t.device.device_type == kDLCPU) ? -1 : t.device.device_id;
                        batch.values = ColumnView{ ct, device, rows,
                                                   static_cast<const uint8_t*>(t.data) + t.byte_offset };
                    }
                }

                mSource.seedBucketFromReadGroup(property, g, keys.empty() ? nullptr : keys.data(), keys.size());
                dispatch(batch);
                mSource.clearBucket();
            }
            ovstage_release_group(mInstance, &g);
        }
        if (fetchErr != OVSTAGE_ERROR_END_OF_ITERATION)
            ok = false;
        waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
    }

    return ok;
}

bool OvstageChangeFeed::drainRange(uint64_t ord0, uint64_t ord1)
{
    if (!mEnabled || !mInstance || !mDict)
        return true;

    const ovx_string_t schemaVals[] = { ovxStr("PhysicsRigidBodyAPI"), ovxStr("PhysicsCollisionAPI"),
                                        ovxStr("PhysicsMaterialAPI"), ovxStr("PhysicsArticulationRootAPI") };
    const ovx_string_t typeVals[] = { ovxStr("PhysicsScene"),         ovxStr("PhysicsFixedJoint"),
                                      ovxStr("PhysicsRevoluteJoint"), ovxStr("PhysicsPrismaticJoint"),
                                      ovxStr("PhysicsSphericalJoint"), ovxStr("PhysicsDistanceJoint"),
                                      ovxStr("PhysicsJoint"),          ovxStr("PhysxPhysicsGearJoint"),
                                      ovxStr("PhysxPhysicsRackAndPinionJoint"), ovxStr("PhysicsCollisionGroup"),
                                      ovxStr("PointInstancer") };

    bool ok = true;
    {
        CARB_PROFILE_ZONE(0, "OvstageChangeFeed::structuralChangeCheck");
        ok = reconcileStructuralChanges(ord0, ord1) && ok;
    }
    {
        CARB_PROFILE_ZONE(0, "OvstageChangeFeed::cachedChangeRead");
        std::unordered_map<ovx_token_t, bool> attributeFloorCache;
        queryForFilter("usd-schemas", OVSTAGE_FILTER_OP_CONTAINS, schemaVals,
                       sizeof(schemaVals) / sizeof(schemaVals[0]), mSchemaFamilyQuery);
        queryForFilter("usd-prim-type", OVSTAGE_FILTER_OP_IN, typeVals,
                       sizeof(typeVals) / sizeof(typeVals[0]), mTypeFamilyQuery);
        ok = readCachedFamilyChanges(mSchemaFamilyQuery, ord0, ord1, attributeFloorCache) && ok;
        ok = readCachedFamilyChanges(mTypeFamilyQuery, ord0, ord1, attributeFloorCache) && ok;
        ok = readKnownTransformChanges(ord0, ord1) && ok;
    }

    if (mGroupComplete)
    {
        mGroupComplete();
    }
    mSource.clearBucket();
    return ok;
}
} // namespace omni::physics::ovstage
