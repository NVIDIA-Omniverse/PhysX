// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-003
 * @covers AC-1 AC-2 AC-3 AC-4 AC-6
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-40 AC-45
 *
 * @implements REQ-PARSE-FEED-002
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-PARSE-FEED-003
 * @covers AC-7 AC-8 AC-9 AC-10 AC-11 AC-12 AC-14
 *
 * @implements REQ-PARSE-FEED-005
 * @covers AC-7
 *
 * @implements REQ-PARSE-COL-005
 * @covers AC-1
 *
 * @implements REQ-LOAD-TOKENS-001
 * @covers AC-5
 */
#pragma once

#include <omni/physics/parse/IPhysicsSource.h>

// Flat ovstage C API (ovstage.h) + the ovx path dictionary. NOTE: ovstage.h and
// ovstage_api/ovstage_api.h define the SAME types and must not both be included
// in one TU — the flat header is the one with exported symbols, so we use it.
#include <ovstage/ovstage.h>
#include <ovstage/ovx_path_dictionary.h>

#include <array>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace omni::physics::ovstage
{
using namespace omni::physics::parse;

// Backing storage for OvstageSource::mInternTable. Defined in OvstageSource.cpp
// (not here) so its members stay private to the implementation; see that
// member's doc comment for why it is shared, refcounted (via shared_ptr), and
// scoped to the underlying ovstage_instance_t rather than to the C++ object.
struct OvstageInternTable;

// Attribute-name conventions shared between OvstageSource (reader), the
// OvstageWalker (enumeration probe), and the test OvstagePopulator (writer).
// Stage units match the real ovpopulation data model so the source reads both a
// controlled (populator-built) instance and a Fabric-backed one populated via
// ovpopulation:
//   - stage units → ovstage 374259+ populates them onto the root prim "/", under the
//     reserved "usd-metadata:" column prefix (usd-metadata:metersPerUnit, etc.), and only
//     when a populate desc's stage_metadata_paths asked for them. Controlled/older instances
//     author them unprefixed on the "/__ovstage_population_stage_info__" prim (legacy
//     "/__ovpopulation_stage_info__" also accepted). loadUnits reads all of these.
//     metersPerUnit / kilogramsPerUnit are doubles and upAxis is a token id ("Y"/"Z").
//   - usd-path/usd-schemas/usd-prim-type are built-in metadata.
// Fabric-backed ovstage authors `omni:fabric:worldMatrix` as the resolved
// local-to-world transform. Older/controlled test data may only carry local
// xform data (`omni:xform` / `omni:fabric:localMatrix` plus resetXformStack);
// the source uses that local composition only as a compatibility fallback.
namespace conv
{
inline constexpr const char* kLocalTransform = "omni:xform"; //!< double[16] local xform matrix on xformable prims
inline constexpr const char* kResetXformStack = "omni:resetXformStack"; //!< bool reset-xform-stack flag
inline constexpr const char* kFabricLocalMatrix = "omni:fabric:localMatrix"; //!< Fabric local xform matrix
inline constexpr const char* kFabricWorldMatrix = "omni:fabric:worldMatrix"; //!< Fabric resolved local-to-world matrix
inline constexpr const char* kUsdPath = "usd-path";         //!< prim path (built-in, auto-maintained)
inline constexpr const char* kUsdParent = "usd-parent";     //!< parent prim path (built-in; IN = direct children)
inline constexpr const char* kUsdChildren = "usd-children"; //!< child prim paths (built-in metadata)
inline constexpr const char* kUsdSchemas = "usd-schemas";   //!< applied schemas (built-in metadata)
inline constexpr const char* kUsdPrimType = "usd-prim-type"; //!< prim type (built-in metadata)
inline constexpr const char* kRootPrimPath = "/"; //!< root prim; ovstage 374259+ populates stage metadata here (usd-metadata: prefix)
inline constexpr const char* kStageInfoPath = "/__ovstage_population_stage_info__"; //!< legacy prim holding stage units
inline constexpr const char* kLegacyStageInfoPath = "/__ovpopulation_stage_info__"; //!< legacy prim holding stage units
inline constexpr const char* kDefaultScenePath = "/__defaultPhysicsScene__"; //!< synthetic default-scene identity (no prim authored)
inline constexpr const char* kMetersPerUnit = "metersPerUnit"; //!< double on kStageInfoPath
inline constexpr const char* kKilogramsPerUnit = "kilogramsPerUnit"; //!< double on kStageInfoPath
inline constexpr const char* kUpAxis = "upAxis"; //!< token id ("Y"/"Z") on kStageInfoPath
inline constexpr const char* kIds = "ids"; //!< int64[] PointInstancer per-instance ids that kInactiveIds names

// Populated USD *metadata* lives under a reserved `usd-metadata:` column prefix,
// where it cannot collide with an attribute or relationship of the same name:
// prim metadata is `usd-metadata:<field>` and property metadata is
// `<property>:usd-metadata:<field>`, with a metadatum nested in a dictionary
// extending `<field>` by the `.`-joined key path.
inline constexpr const char* kMetadataPrefix = "usd-metadata:";
inline constexpr const char* kInactiveIds = "usd-metadata:inactiveIds"; //!< int64[] PointInstancer inactive-id metadata

// Consumers above the source name a metadatum by its USD field: KnownTokens
// interns "physics:localSpaceVelocities" because the USD source reads it from
// customData under that name, and the runtime's change handling compares against
// UsdGeomTokens->inactiveIds. Only the ovstage column carries the prefix, so the
// translation is confined to the ovstage token boundary and every cache, token id,
// and property comparison keeps working off the USD-facing name.
inline constexpr const char* kInactiveIdsUsdField = "inactiveIds";
inline constexpr const char* kLocalSpaceVelocitiesUsdField = "physics:localSpaceVelocities";
// `:` separates every level of a metadata path, including into a dictionary, because that is
// how USD splits a key path. `SetCustomDataByKey(TfToken("physics:localSpaceVelocities"))` does
// NOT create one key of that name — USD splits on the colon and stores customData → physics →
// localSpaceVelocities, two levels — so the column names each level in turn. The earlier
// `customData.physics:localSpaceVelocities` spelling addressed a single flat key containing a
// colon, which is a key USD can hold but cannot address, so nothing ever matched.
inline constexpr const char* kLocalSpaceVelocitiesColumn =
    "usd-metadata:customData:physics:localSpaceVelocities";

//! USD-facing attribute name → the ovstage column publishing it (identity for all
//! but the metadata-backed knobs above).
inline std::string_view toOvstageColumn(std::string_view usdName)
{
    if (usdName == kLocalSpaceVelocitiesUsdField)
        return kLocalSpaceVelocitiesColumn;
    if (usdName == kInactiveIdsUsdField)
        return kInactiveIds;
    return usdName;
}

//! The inverse of toOvstageColumn, for change groups tagged by ovstage column.
inline std::string_view toUsdAttributeName(std::string_view column)
{
    if (column == kLocalSpaceVelocitiesColumn)
        return kLocalSpaceVelocitiesUsdField;
    if (column == kInactiveIds)
        return kInactiveIdsUsdField;
    return column;
}
} // namespace conv

// One prim's complete `usd-schemas` list after a structural write, as the token ids the column
// carries (path-dictionary token space); empty when the column was tombstoned. Input of
// OvstageSource::applySchemaDelta().
struct SchemaRowDelta
{
    ovx_primpath_t raw = 0;
    std::vector<uint64_t> schemaTokens;
};

// One prim's `usd-prim-type` after a structural write, as the dictionary token id the column
// carries; 0 when the column was tombstoned. Input of OvstageSource::applyPrimTypeDelta().
struct PrimTypeRowDelta
{
    ovx_primpath_t raw = 0;
    uint64_t typeToken = 0;
};

// The prim lists behind one read's groups, each fetched from the dictionary once and reused for
// every group that addresses it. A whole-stage read returns one group per prim, all over the
// query's list, so resolving it per group costs O(groups x list) -- quadratic in stage size.
// Scope one memo to one fetch loop; each memoised list is pinned until the memo dies, so a
// released group cannot recycle its handle under the memo.
class ReadListMemo
{
public:
    explicit ReadListMemo(ovx_path_dictionary_t* dict) : mDict(dict)
    {
    }
    ~ReadListMemo();
    ReadListMemo(const ReadListMemo&) = delete;
    ReadListMemo& operator=(const ReadListMemo&) = delete;

    // The list's entries (valid until the memo dies). False, with a null `outPaths`, when the
    // list is empty or cannot be resolved -- the same outcome ovx_path_dictionary_get_paths
    // reports for those, and a failed fetch is not memoised.
    bool paths(ovx_primpath_list_t list, const ovx_primpath_t** outPaths, size_t* outCount);

private:
    struct Entry
    {
        bool pinned = false;
        std::vector<ovx_primpath_t> paths;
    };
    ovx_path_dictionary_t* mDict = nullptr;
    // A read may hand every group its own list (one per prim on a filter query), so the lookup is
    // O(1): consecutive groups mostly share a list (the last hit), the rest go through the map.
    std::unordered_map<ovx_primpath_list_t, Entry> mEntries;
    ovx_primpath_list_t mLastList = OVX_INVALID_PRIMPATH_LIST;
    const Entry* mLastEntry = nullptr;
};

// ---------------------------------------------------------------------------
// OvstageSource - IPhysicsSource over an ovstage instance (ADR-0002).
//
// Implements scalar and array attributes, relationships, schema/type queries,
// path traversal, transforms, stage units, material bindings, and mesh buffers.
// The walker seeds columnar bulk-read caches; uncached reads use the same ovstage
// data plane one prim at a time. Scene-graph instance proxies keep logical keys
// while type and geometry reads can resolve to prototype backing storage.
//
// `mInstance` / `mDict` are non-owning: the scene owner (a test OvstagePopulator,
// or ovpopulation in production) creates and destroys them. When either is null
// the source degrades to an empty scan.
// ---------------------------------------------------------------------------

class OvstageSource final : public IPhysicsSource
{
public:
    // `readOrdinal` is the sealed ordinal default-time reads target (the ordinal
    // the producer wrote + advanced the watermark to).
    OvstageSource(ovstage_instance_t* instance,
                  ovx_path_dictionary_t* dict,
                  ovstage_ordinal_t readOrdinal = 1,
                  uint64_t usdStageId = 0);
    ~OvstageSource() override;

    // --- IPhysicsSource overrides (the pure-virtual contract) ---

    std::string_view sourceKeyToString(ObjectKey key) const override;
    TokenId internToken(std::string_view token) const override;
    std::string_view tokenToString(TokenId id) const override;
    // The one KnownTokens batch for this source, interned lazily on first call and
    // never invalidated (token ids are permanent per source). Every ParseContext and
    // the attach-scoped cache reference it instead of interning their own.
    const KnownTokens* knownTokens() const override;

    // Intern a string in the ovstage path dictionary's token space. Memoized per
    // string: dictionary tokens are permanent for the dictionary's lifetime, so a
    // repeated call never re-enters ovx_path_dictionary_intern_token.
    ovx_token_t ovxToken(std::string_view s) const;

    ObjectKey getRootKey() const override;
    bool exists(ObjectKey key) const override;
    // One `usd-path IN [...]` round trip for every cold key in `keys`, instead of
    // exists()'s one round trip per cold key. See IPhysicsSource::existsBatch.
    void existsBatch(const std::vector<ObjectKey>& keys, std::vector<bool>& outExists) const override;
    // existsBatch() that reports whether every cold key was actually resolved (false: the live
    // query failed, so a `false` answer is not proof of absence).
    bool existsBatchChecked(const std::vector<ObjectKey>& keys, std::vector<bool>& outExists) const;
    bool isA(ObjectKey key, TokenId typeToken) const override;
    void forEachDescendantPruned(ObjectKey root,
                                 std::function<bool(ObjectKey)> visit,
                                 DescendantScope scope = DescendantScope::eAll) const override;
    void forEachChild(ObjectKey parent, std::function<void(ObjectKey)> cb) const override;
    ObjectKey findByPath(std::string_view path) const override;
    ObjectKey getParent(ObjectKey key) const override;
    // enumerate/get_paths and intern_path can hand back distinct handles for the
    // same path; canonicalise to the interned handle so cross-ref / graph matching
    // is reliable (see canonicalPath).
    ObjectKey canonicalKey(ObjectKey key) const override
    {
        return internKey(canonicalPath(key));
    }

    bool hasSchema(ObjectKey key, TokenId schemaToken) const override;
    // Exact authored type name, not schema-hierarchy membership. ovstage
    // publishes it as the `usd-prim-type` token column, so it is a plain
    // attribute read. Consumers key custom-joint dispatch off it
    // (`LoadStage.cpp`'s eJointCustom branch), which is why the base's
    // invalid-token default is not good enough here.
    TokenId getTypeName(ObjectKey key) const override;
    void forEachAppliedSchema(ObjectKey key, std::function<void(TokenId)> cb) const override;
    void forEachMultiApplyInstance(
        ObjectKey key,
        std::string_view baseSchema,
        std::function<void(std::string_view instance)> cb) const override;

    AttrValue getAttribute(ObjectKey key, TokenId attr) const override;
    AttrValue getAttributeAtTime(ObjectKey key, TokenId attr, ReadTime time) const override;
    using IPhysicsSource::getAttribute; // bring in typed overloads
    // ovpopulation stores USD token attrs as a uint64 token-id column (decoded as
    // eInt, not eToken), so the base eToken-only typed overload never resolves
    // them. Override to resolve the id through the path dictionary and re-intern
    // into the source token space (joint axis, round-shape axis, drive type, …).
    bool getAttribute(ObjectKey key, TokenId attr, TokenId& out) const override;
    bool hasAuthoredAttribute(ObjectKey key, TokenId attr) const override;
    bool isAttributeTimeSampled(ObjectKey key, TokenId attr) const override;

    void getRelationshipTargets(ObjectKey key, TokenId rel, std::vector<ObjectKey>& out) const override;
    bool hasRelationship(ObjectKey key, TokenId rel) const override;
    void getInactiveInstanceIds(ObjectKey key, std::vector<int64_t>& out) const override;

    void getLocalToWorldTransform(ObjectKey key, Matrix4d& outMatrix) const override;
    void getLocalTransform(ObjectKey key, ReadTime time, Matrix4d& outMatrix,
                           bool& outResetsXformStack) const override;
    // Shared implementation behind getLocalTransform: reads the object-local transform from
    // the ovstage data plane / load cache. There is no USD-stage fallback -- the data plane is
    // a single ordinal snapshot, so `time` and `usdAtEarliestTime` are unused; they remain in
    // the signature for parity with the sibling backends' time-aware read entry points.
    void getLocalTransformImpl(ObjectKey key, ReadTime time, bool usdAtEarliestTime,
                               Matrix4d& outMatrix, bool& outResetsXformStack) const;
    // Compose a prim's world transform from its parent chain of locals, memoizing
    // each ancestor's composed world in mLoadCacheComposedWorld. Only used while
    // mLoadCacheActive (the initial load walk), where the data-plane snapshot is
    // static so memoized results cannot go stale.
    void composeWorldFromLocalsLoadCached(ObjectKey key, Matrix4d& out) const;
    void getLocalToWorldRotationAndScale(ObjectKey key,
                                         Matrix3d& outRotation,
                                         carb::Float3& outScale) const override;

    const void* resolveBuffer(BufferHandle handle, size_t& byteCount) const override;
    void releaseBuffer(BufferHandle handle) const override;
    MeshGeometry getMeshAttributes(ObjectKey key, bool includeFaceMaterials = true) const override;
    BufferHandle getArrayAttribute(ObjectKey key, TokenId attr, ReadTime time) const override;
    BufferHandle readArrayAttribute(ObjectKey key, std::string_view attr, BufferElemType type, int comps) const;
    SourceUnits getSourceUnits() const override;

    // The resident USD stage this ovstage session mirrors, classified once at
    // attach time and handed in through AttachTarget::residentBackingStageId —
    // never re-queried from the payload here, so a nonresident or erroring id
    // cannot become a local stage lookup after the attach has begun. 0 for an
    // ovstage-only (stageless) attach, which is the common production case.
    //
    // No code reads a USD stage through this id any more (see
    // mLoadCacheMayBeStale). It is kept, and this override kept returning it
    // unchanged, only because it is still the backend-neutral value
    // IPhysicsSource::residentUsdStageId() promises to callers such as
    // AttachedStage -- the interface contract, not this class' own behaviour,
    // is what requires it.
    uint64_t residentUsdStageId() const override
    {
        return mLoadCacheMayBeStale;
    }

    void resolveCollection(ObjectKey primKey,
                           TokenId collectionName,
                           std::vector<ObjectKey>& members) const override;
    ObjectKey getMaterialBinding(ObjectKey primKey) const override;

    // Vends the pull-based ovstage change feed (ADR-0003 M3). Null when the
    // source is unconfigured (no instance/dict → nothing to observe).
    std::unique_ptr<IChangeFeed> createChangeFeed() override;

    // --- Columnar bulk read (ADR-0002 "speed-of-light" path) -------------------
    //
    // These are OvstageSource-specific (not part of IPhysicsSource): the walker
    // holds a concrete OvstageSource and calls them around a concept's parse loop.
    // Parsers still see only the IPhysicsSource facade — getAttribute /
    // getLocalToWorldTransform transparently serve from the cache when a bucket is
    // active, so no parser churn.
    //
    // prefetchBucket issues ONE read_attributes over a path-list query of all
    // `keys` for all `attrNames` (the bucket's columns), decodes each prim's row
    // out of the returned DLTensor columns, and caches them. While the bucket is
    // active:
    //   - scalar/vector reads for a bucket key hit the cache; a clean-bucket miss
    //     resolves to "unauthored" (empty) with no per-prim round trip;
    //   - transform reads hit cached world/local transform rows when ovpopulation
    //     provides them, falling back to composition only for uncached paths;
    //   - reads for keys outside the bucket always fall back to the per-prim path,
    //     so behaviour outside the active bucket is unchanged.
    // clearBucket drops the cache. Buckets do not nest; a second prefetchBucket
    // or seedBucketFromReadGroup replaces the first.
    void prefetchBucket(const std::vector<ObjectKey>& keys,
                        const std::vector<std::string>& attrNames,
                        bool sealMissing = true) const;
    // Deduped root-to-parent ancestor chain of `keys`, path-based (no ovstage query).
    // Shared by the walker's ancestor prefetch and AttachedStage's mass-update prefetch.
    std::vector<ObjectKey> collectAncestors(const std::vector<ObjectKey>& keys) const;
    // Bulk-prefetch material:binding[:physics] and physics:simulationOwner for `keys` and
    // their ancestors into the load cache. No-op unless a beginLoadCache() window is open.
    void prefetchRelationshipAncestors(const std::vector<ObjectKey>& keys) const;
    // The relationship columns prefetchRelationshipAncestors() reads, for a caller folding
    // them into one wider prefetchBucket() read.
    static const std::vector<std::string>& relationshipPrefetchAttrs();
    // Keep bucket-prefetched values alive across the whole ovstage load scan.
    // The scanner still uses narrow buckets for concept-local parsing, but this
    // load cache keeps ovstage read groups alive and remembers row references plus
    // covered misses so later concepts do not re-query columns already read.
    // A prefetchBucket() that includes `usd-path` under an open window also seeds the
    // existence memo for every bucket key from the returned rows (a live prim always has one).
    void beginLoadCache() const;
    void clearLoadCache() const;
    // Whether a beginLoadCache() window is open. A consumer that opens one around a whole
    // incremental load lets the scan and every follow-up read share it (scanOvstage joins an
    // open window instead of replacing it).
    bool loadCacheActive() const;
    // Bulk hierarchy-read window: while open, forEachChild() trusts a complete child
    // cache's "no children" answer instead of a per-prim prefix query (O(prims^2) for a
    // pass that asks about every leaf). Separate from beginLoadCache() on purpose: this
    // never arms the attribute covered-miss machinery, so it cannot seal an attribute miss.
    void beginHierarchyBulkRead() const;
    void endHierarchyBulkRead() const;
    // Seed existence checks with keys proven live by a just-completed scan.
    // Kept separate from the attribute bucket so exists() can be O(1) without
    // changing getAttribute cache semantics.
    void seedKnownKeys(const std::vector<ObjectKey>& keys) const;
    void clearKnownKeys() const;
    // Seed the same scalar bucket from a change-feed read group that has already
    // been fetched. This lets update callbacks use getAttribute/getValue without
    // causing a second ovstage read for the same changed column. With `append` the
    // group's rows join the current bucket instead of replacing it; only the world
    // matrix column supports this (it is decoded eagerly, so the released group is
    // never dereferenced later).
    // `listPaths`/`listCount`, when given, are the entries of `group.prims.list` (caller-verified
    // by handle identity) and replace the per-call dictionary fetch of the whole list.
    void seedBucketFromReadGroup(TokenId attr,
                                 const ovstage_read_group_t& group,
                                 const ObjectKey* keys,
                                 size_t keyCount,
                                 bool append = false,
                                 const ovx_primpath_t* listPaths = nullptr,
                                 size_t listCount = 0) const;
    // Seed the bucket from a feed-owned dense column: row i of `tensor` ([count, comps], host,
    // packed) is the value of `rawHandles[i]` / `keys[i]`. Replaces the bucket; `tensor` must
    // stay alive until clearBucket().
    void seedBucketFromColumn(
        TokenId attr, const uint64_t* rawHandles, const ObjectKey* keys, size_t count, const DLTensor& tensor) const;
    void clearBucket() const;
    // Whether a prefetchBucket() / seed* bucket is active.
    bool bucketActive() const;
    // Move the active bucket aside so a nested scan on this source can prefetch and clear its
    // own without clobbering the caller's; resumeBucket() puts it back in place of whatever the
    // nested user left. Null when no bucket is active. The rows a snapshot references (a feed's
    // read-group tensor) stay the caller's to keep alive, as for the live bucket.
    struct BucketSnapshot;
    struct BucketSnapshotDeleter
    {
        void operator()(BucketSnapshot* snapshot) const;
    };
    using BucketSnapshotPtr = std::unique_ptr<BucketSnapshot, BucketSnapshotDeleter>;
    BucketSnapshotPtr suspendBucket() const;
    void resumeBucket(BucketSnapshotPtr snapshot) const;
    void clearSchemaCache() const;
    // Patch the schema-membership memo for prims whose usd-schemas column was written since
    // the memo was filled, instead of dropping it: every memoised schema (and multi-apply base)
    // gains or loses each row's prim exactly; a schema nothing asked about yet needs no entry
    // (its first query sees the prim). Returns false when a row cannot be applied exactly
    // (unresolvable handle or token) -- the caller must then clearSchemaCache().
    bool applySchemaDelta(const std::vector<SchemaRowDelta>& rows) const;
    // The instancing / material latches of clearSchemaCache() alone (rebuilt lazily by the next
    // consumer); leaves the schema-membership and existence memos intact.
    void resetInstancingCaches() const;
    // Record prims a structural read just proved live, so exists() answers them from the memo.
    void noteLivePrims(const std::vector<ovx_primpath_t>& raws) const;
    // Drop every memoised existence answer. The change feed calls it as a drain begins: a prim
    // deleted in the range shows up in no structural read, so only a fresh memo can read it absent.
    void clearExistsMemo() const;
    // Full drop of the child / parent / descendant hierarchy caches; the next traversal
    // re-derives them. The change feed uses it only as the fallback for applyHierarchyDelta()
    // (tombstones in range, failed delta read).
    void invalidateHierarchyCache() const;
    // True once buildChildCache() has run (complete or not). While nothing has built the
    // cache the change feed skips the hierarchy delta entirely.
    bool hierarchyCacheBuilt() const;
    // Patch the child/parent caches for prims whose structural columns were written since
    // the cache was built, instead of dropping it. `addedRaw` holds raw ovx_primpath_t
    // handles as returned by ovx_path_dictionary_get_paths on a read group; aliases are
    // canonicalised here. Returns false when the delta cannot be applied exactly (partial
    // build, unresolvable handle, no source) -- the caller must then
    // invalidateHierarchyCache(). No-op returning true when the cache is not built.
    bool applyHierarchyDelta(const std::vector<ovx_primpath_t>& addedRaw) const;
    // Patch the prim-type memo isA()/getTypeName() answer from: raw handles as the feed read
    // them, `typeToken` the usd-prim-type dictionary id (0 = tombstone -> forget). False on an
    // unresolvable handle -- the caller must then clearSchemaCache().
    bool applyPrimTypeDelta(const std::vector<PrimTypeRowDelta>& rows) const;
    // Exact removal of tombstoned prims and their cached subtrees from the hierarchy,
    // membership, per-prim schema, type and existence memos. Assumes a tombstoned prim's
    // descendants are tombstoned too (USD / population semantics). True when applied exactly;
    // false when the hierarchy cache is not built or not complete (the subtree is unknown) --
    // the caller must then drop both caches.
    bool applyPrimRemoval(const std::vector<ovx_primpath_t>& removedRaw) const;
    // The sealed ordinal the ordinal-gated reads (child cache under a load cache, whole-stage
    // schema reads) target; the change feed advances it to each drained range's end.
    void setReadOrdinal(ovstage_ordinal_t ord);
    ovstage_ordinal_t readOrdinal() const;
    ovstage_instance_t* instance() const
    {
        return mInstance;
    }
    bool collectSchemaKeys(TokenId schemaToken, std::vector<ObjectKey>& out) const;
    // Keys whose usd-prim-type is `typeName`, from a whole-stage type index built once per source
    // (one read at the read ordinal) and kept exact by applyPrimTypeDelta() / applyPrimRemoval();
    // dropped by clearSchemaCache(). False when the index cannot be built: enumerate live instead.
    bool collectPrimTypeKeys(std::string_view typeName, std::vector<ObjectKey>& out) const;
    // Drop the stage-wide attribute-column vocabulary behind stageHasAttributeWithPrefix() /
    // stageHasAttributeColumn(), and the instance-material cache whose empty answer it gated.
    // The change feed calls it as a drain begins: a column first authored in the range must be
    // seen by the reads the drain triggers, value-only ranges included.
    void invalidateAttributeVocabulary() const;
    bool collectMultiApplySchemaKeys(TokenId baseSchemaToken, std::vector<ObjectKey>& out) const;
    // Iterative, mChildCache-backed enumeration of root + all descendants
    // (memoized in mDescendantCache). Public so the load-time known-key seed can
    // gather the whole subtree without the recursive forEachDescendantPruned walk
    // (per-node pathOf/mutex/std::function overhead). Visits the same key set.
    void collectDescendantKeys(ObjectKey root, std::vector<ObjectKey>& out) const;

    // --- Output read support (ADR-0007) ----------------------------------------
    // The shared path dictionary this source reads through. Consumers building an
    // ovx_primpath_list_t (e.g. the rigid-body output read) need it to create the
    // list and later destroy it.
    ovx_path_dictionary_t* dictionary() const { return mDict; }

    // Canonical interned prim-path handle for `key` (intern_path of its resolved
    // path string). Use this when handing a key's path to ovstage APIs that
    // require a dictionary-interned handle (e.g. ovx_path_dictionary_create_path_list):
    // walker/enumerate handles can differ from the interned handle for the same path.
    // Returns OVX_INVALID_PRIMPATH (0) for an unresolvable key.
    ovx_primpath_t canonicalPath(ObjectKey key) const { return canonicalHandleRaw(rawHandle(key)); }

    // Resolve the storage prim that owns geometry authored for a logical scene-
    // graph instance proxy. Non-instanced keys resolve to themselves. Callers
    // must keep descriptor identity, transforms, materials, and relationships on
    // the logical key; this backing key is only for type and geometry reads.
    ObjectKey geometryBackingKey(ObjectKey key) const;
    // Resolve selected collision attributes through the nearest strict
    // scene-graph instance-root ancestor. Returns invalid when there is no such
    // ancestor, so a top-level instance root's value blocks remain authoritative.
    ObjectKey collisionAttributeBackingKey(ObjectKey key) const;
    // True for the private prototype root or any descendant below it. Those
    // keys provide backing data and must never become physics descriptors.
    bool isPrototypeBackingKey(ObjectKey key) const;

    // --- ObjectKey generation tagging (ADR-0021 Alternative A) -----------------
    //
    // ObjectKey.handle on this backend used to BE the externally-owned
    // ovx_primpath_t minted by the vendored ovstage path-dictionary dependency,
    // with no attach identity encoded: a stale key captured before a detach
    // could numerically alias a live object minted by a later, unrelated
    // attach, because ovx_primpath_t carries no generation/epoch concept of its
    // own (checked ovx_path_dictionary.h/ovstage.h) and this class does not own
    // its bit layout (unlike UsdSource's private invented index -- see
    // UsdSource::packKey/decodeLocalIndex, the pattern this mirrors).
    //
    // The fix: OvstageSource owns a private ObjectKey <-> ovx_primpath_t intern
    // table (mInternTable below), and every ObjectKey value crossing this
    // class's public surface (IPhysicsSource overrides, plus the
    // OvstageSource-specific public helpers OvstageWalker/OvstageOutput/
    // OvstageChangeFeed call directly) is the packed, generation-tagged form:
    // (generation << 32) | (1-based local index into the table). mInternTable
    // is scoped to the underlying ovstage_instance_t and refcounted, not
    // per-C++-object: production code builds exactly one OvstageSource per
    // attach, but test/probe code legitimately builds several OvstageSource
    // wrappers over ONE shared, concurrently-live instance and exchanges
    // ObjectKeys between them, so sibling wrappers over the same live
    // instance share one table/generation (see OvstageInternTable's doc
    // comment in OvstageSource.cpp for the full mechanism, and why a naive
    // per-object counter -- mirroring UsdSource::mGeneration too literally --
    // broke that case). A stale cross-attach collision is still detectable:
    // rawHandle() rejects a decode whose generation does not match this
    // instance's table, instead of silently resolving to whatever raw handle
    // a later, unrelated attach's table happens to have interned at the same
    // local index.
    //
    // Internal ovstage C-API calls and this class's own raw-handle-keyed caches
    // (mPathStringCache, mChildCache, mCanonicalHandleCache, mBucketKeys, ...)
    // continue to operate on the TRUE raw ovx_primpath_t throughout -- translate
    // only at the two functions below, which are this class's only packed-form
    // mint/decode choke points.
public:
    // Mint (or look up) the packed ObjectKey for a raw ovx_primpath_t. Returns
    // the invalid sentinel for raw == OVX_INVALID_PRIMPATH (0). Idempotent: the
    // same raw handle always packs to the same ObjectKey for the lifetime of
    // this instance.
    ObjectKey internKey(uint64_t raw) const;
    // Decode a packed ObjectKey minted by *this* instance back to its true raw
    // ovx_primpath_t. Returns 0 (OVX_INVALID_PRIMPATH, always an invalid
    // decode) when `key` is the invalid sentinel, was minted by a different
    // (stale or foreign) OvstageSource instance, or is otherwise out of range --
    // this is the actual generation check that closes the aliasing gap.
    uint64_t rawHandle(ObjectKey key) const;

private:
    struct CachedTensorRow
    {
        const DLTensor* tensor = nullptr;
        int64_t comps = 0;
        uint32_t row = 0;
    };

    // One (prim, attribute) cell of the load-cache table. A null tensor means "no row
    // stored" (the store paths never record a null tensor), so no separate presence set is
    // needed; `covered` is the sealed-miss mark prefetchBucket() sets for every bucket key
    // of a cleanly read column.
    struct LoadCacheCell
    {
        CachedTensorRow scalar;
        const DLTensor* relationship = nullptr;
        bool covered = false;
    };
    // Per-prim record of the load-cache table. `cells` is indexed by the per-window attribute
    // index (mLoadCacheAttrIndex[TokenId.id]); the three transform rows have fixed slots.
    // `usdPathSerial` == mPrefetchSerial marks a prim whose usd-path row the current
    // prefetchBucket() read returned (the per-call existence evidence).
    struct LoadCachePrim
    {
        std::vector<LoadCacheCell> cells;
        CachedTensorRow world;
        CachedTensorRow local;
        CachedTensorRow reset;
        uint32_t usdPathSerial = 0;
    };
    static constexpr uint32_t kNoLoadCacheIndex = ~uint32_t(0);

    // Load-cache table access. The insert forms map a raw handle AND its canonical alias to
    // one prim record (resolved once per prim, not per cell) and grow the per-window
    // attribute index; the const forms never insert and answer "absent" for a handle or
    // attribute the window has not seen.
    uint32_t loadCachePrimIndexFor(uint64_t raw) const;
    const LoadCachePrim* loadCachePrim(uint64_t handle) const;
    uint32_t loadCacheAttrIndexFor(uint32_t attrId) const;
    LoadCacheCell& loadCacheCellFor(uint32_t primIndex, uint32_t attrIndex) const;
    const LoadCacheCell* loadCacheCell(uint64_t handle, uint32_t attrId) const;
    bool loadCacheCovers(uint64_t handle, uint32_t attrId) const;
    // Drop every row and coverage mark of the window, keeping the containers' capacity.
    void resetLoadCacheTable() const;

    TokenId doInternToken(std::string_view token) const;

    // Resolve a raw ovx_primpath_t to its path string. Purely internal -- never
    // sees a packed ObjectKey; callers holding a packed key must go through
    // rawHandle() first (see the mint/decode choke points above).
    std::string pathOfRaw(uint64_t raw) const;
    // Normalise a raw handle through its path string: intern_path (findByPath)
    // and get_paths (read-group prim lists) can hand back DISTINCT ovx_primpath_t
    // for the same path. Returns 0 for an unresolvable handle.
    uint64_t canonicalHandleRaw(uint64_t raw) const;
    // Raw-space counterpart of findByPath() (see public override below): the
    // actual C-API-facing implementation, reused internally by callers already
    // holding a raw handle (canonicalHandleRaw, getParentRaw, ...) so they don't
    // pay a pack/decode round trip just to call themselves back through the
    // packed public surface.
    uint64_t findByPathRaw(std::string_view path) const;
    uint64_t getParentRaw(uint64_t raw) const;
    bool existsRaw(uint64_t raw) const;
    uint64_t geometryBackingKeyRaw(uint64_t raw) const;
    AttrValue getAttributeAtTimeRaw(uint64_t raw, TokenId attr, ReadTime time) const;
    void collectDescendantKeysRaw(uint64_t root, std::vector<uint64_t>& out) const;
    bool collectSchemaKeysRaw(TokenId schemaToken, std::vector<uint64_t>& out) const;
    void forEachChildRaw(uint64_t parent, const std::function<void(uint64_t)>& cb) const;
    // Scoped counterpart of forEachChildRaw(): reuses the eager cache if
    // already built, otherwise queries only `parent`'s own direct children
    // instead of triggering buildChildCache(). Used for a leaf/non-leaf check
    // on a handful of physics-relevant keys. Returns true when the read was
    // authoritative (either the eager cache served it, or the live query
    // completed a clean pass over usd-path); false when the source could not be
    // queried or the enumeration was cut short, so a caller must NOT read
    // "no children emitted" as proof of a leaf.
    bool forEachChildScopedRaw(uint64_t parent, const std::function<void(uint64_t)>& cb) const;
    // Body of the live, single-parent, prefix-scoped ovstage query shared by
    // forEachChildRaw()'s cache-miss fallback and forEachChildScopedRaw().
    // Emits `parent`'s direct children, sorted by path then handle, via `emit`.
    // Returns true when the enumeration completed a clean, authoritative pass
    // (usd-path is populated on every queryable row, so it is the authoritative
    // enumerator); false on query failure or a truncated read.
    bool liveQueryDirectChildrenRaw(uint64_t parent, const std::string& parentPath,
                                     const std::function<void(uint64_t)>& emit) const;

    bool buildPrototypeRootCache() const;
    bool buildPhysicsInstancingCache() const;
    bool buildInstanceMaterialCache() const;
    void buildChildCache() const;
    // Insert `child` under `parent` keeping the (pathOfRaw, handle) order buildChildCache
    // imposes. Returns false when the edge (or an alias of it) is already present.
    bool insertChildEdgeSorted(uint64_t parent, uint64_t child) const;
    bool buildSchemaCache() const;
    // Per-schema membership via one targeted, memoized `usd-schemas CONTAINS <name>` query;
    // never traverses the stage. Exact for concrete schemas and "Base:instance" names.
    const std::unordered_set<uint64_t>* schemaMembershipFromQuery(TokenId schemaToken) const;
    // Membership for an unqualified multi-apply base ("PhysxLimitAPI"): union of per-instance
    // queries over the fixed joint-DOF axis set, memoized. Returns nullptr for anything that
    // is not a known joint-axis base so callers can fall back.
    const std::unordered_set<uint64_t>* multiApplyBaseMembership(TokenId baseToken) const;
    // Read one prim's usd-schemas name list: from the per-prim memo when the whole-stage build
    // or a structural delta filled it, else one single-prim read (no whole-stage parse).
    // Backs forEachAppliedSchema()/forEachMultiApplyInstance().
    void readPrimSchemaNames(uint64_t raw, const std::function<void(std::string_view)>& fn) const;
    // The prim's own usd-prim-type as a source TokenId: memo first, else a read (memoised).
    // False when the prim has no type column.
    bool primTypeTokenRaw(uint64_t raw, TokenId& out) const;
    // Purge one handle from every membership / per-prim memo (applyPrimRemoval helper).
    void purgeHandleFromMemos(uint64_t handle) const;

public:
    // Whether any prim authors an attribute starting with `prefix`, from the stage's
    // attribute-column vocabulary (one query, no traversal, memoized). nullopt when the
    // vocabulary could not be read; callers keep their fail-open default.
    std::optional<bool> stageHasAttributeWithPrefix(std::string_view prefix) const;
    // Whether any prim authors the attribute column `name` exactly (same vocabulary).
    std::optional<bool> stageHasAttributeColumn(std::string_view name) const;

    // Whether any prim applies one of the named schemas. Exact for concrete schemas and
    // "Base:instance" names (one count-only CONTAINS query); an unqualified multi-apply
    // base matches when any instance of it is applied. Holds when every attribute is
    // unauthored. nullopt when the answer could not be determined.
    std::optional<bool> stageHasAnySchema(const char* const* schemaNames, size_t nameCount) const;

private:
    // One whole-stage read of the token-id column `column` at the read ordinal: fn(raw,
    // canonical, tokenValue) per element of every prim's row. False unless the read completed
    // cleanly.
    bool forEachStageTokenValue(std::string_view column,
                                const std::function<void(uint64_t, uint64_t, uint64_t)>& fn) const;
    // forEachStageTokenValue over usd-schemas: one call per applied schema element.
    bool forEachStageSchemaValue(const std::function<void(uint64_t, uint64_t, uint64_t)>& fn) const;
    // The whole-stage usd-prim-type index behind collectPrimTypeKeys(); fills mPrimTypeByRaw from
    // the same rows. Latched: a failed build is not retried until clearSchemaCache().
    bool buildPrimTypeIndex() const;
    // Build the attribute-column vocabulary if needed; false when it could not be fetched.
    bool ensureAttributeVocabulary() const;
    // Prims applying any instance of each multi-apply base, keyed by base TokenId; one
    // forEachStageSchemaValue pass, memoized. Backs unqualified-base hasSchema() and
    // stageHasAnySchema() for bases whose instance names are arbitrary.
    bool ensureSchemaVocabulary() const;
    mutable bool mSchemaVocabularyBuilt = false;
    mutable bool mSchemaVocabularyComplete = false;
    mutable std::unordered_map<uint32_t, std::unordered_set<uint64_t>> mBaseInstanceMembership;


    // Read attribute `attrName` for the single prim `raw` at the read ordinal and
    // invoke `fn` with the first read group's DLTensor + prim count. Returns
    // false if the source is unconfigured or the attribute has no data. Handles
    // are all released before returning.
    bool withAttributeTensor(uint64_t raw,
                             std::string_view attrName,
                             const std::function<void(const DLTensor&, uint32_t primCount)>& fn) const;
    void loadUnits();

    // Mint a BufferHandle backed by an owned byte copy of `data` (the cooking
    // consumer resolves it via resolveBuffer). Returns an invalid handle for an
    // empty payload. `elemCount` is the logical element count (vertices/indices),
    // `type` the element interpretation the MeshGeometry contract documents.
    BufferHandle registerMeshBuffer(const void* data, size_t byteCount, uint32_t elemCount, BufferElemType type) const;
    // Read a per-prim array column (points/faceVertexIndices/…) and register it as
    // a buffer. `comps` is the components-per-element (3 for Vec3, 1 for scalars);
    // int columns are normalized to Int32 so the bytes match the MeshGeometry contract.
    BufferHandle readArrayBuffer(ObjectKey key, std::string_view attr, BufferElemType type, int comps,
                                  ReadTime time = ReadTime::defaultTime()) const;

    ovstage_instance_t* mInstance = nullptr;
    ovx_path_dictionary_t* mDict = nullptr;
    ovstage_ordinal_t mReadOrdinal = 1;

    // Despite the type, this no longer means "the resident USD stage id" to
    // this class' own logic -- no code here reads a USD stage. What it still
    // gates (four covered-miss sites in OvstageSource.cpp) is "there may be a
    // live authoring source whose edits post-date the load cache, so a
    // covered-miss cannot be trusted as authoritative-empty and must fall
    // through to a live read instead." A stageless attach necessarily passes
    // 0 here and gets "covered-miss is authoritative"; an attach that happens
    // to carry a resident stage id gets the live re-read. The value itself
    // (not just its zero-ness) is still exposed verbatim through
    // residentUsdStageId(), which is the actual reason it stays a uint64_t
    // rather than becoming a bool.
    uint64_t mLoadCacheMayBeStale = 0;

    // String<->uint32 token table. parse TokenId and ovx_token_t stay distinct
    // because their widths are part of separate API contracts.
    // Guards dictionary calls plus mutable token/cache/buffer state. The source is normally
    // used single-threaded, but replicator clone workers read transforms concurrently.
    mutable std::recursive_mutex mMutex;

    mutable std::unordered_map<std::string, TokenId> mStringToToken;
    mutable std::deque<std::string> mTokenToString;
    // See knownTokens(). unique_ptr keeps KnownTokens.h out of this header; the
    // destructor is out of line. Non-copyable along with the rest of the source
    // (mMutex), so the batch can never leak onto a source with another token table.
    mutable std::unique_ptr<KnownTokens> mKnownTokens;
    // ovxToken() memo. Dictionary tokens are dict-lifetime and equal strings yield equal
    // tokens (path_dictionary.h), so every successful intern result is permanent.
    // std::less<> gives allocation-free string_view lookups.
    mutable std::map<std::string, ovx_token_t, std::less<>> mOvxTokenMemo;

    // The ObjectKey <-> ovx_primpath_t intern table (generation + the raw<->
    // local-index mapping), shared with every other OvstageSource wrapping the
    // same live ovstage_instance_t (see OvstageInternTable's doc comment on
    // this class's public section above, and its full definition in
    // OvstageSource.cpp). internKey()/rawHandle() are the only readers/writers.
    std::shared_ptr<OvstageInternTable> mInternTable;

    SourceUnits mUnits{ 1.0f, 1.0f, UpAxis::eZ };

    // --- Bucket cache (populated by prefetchBucket / change feed) --------------
    // Keyed by raw and canonical prim-path handles (ObjectKey::handle ==
    // ovx_primpath_t). Keeping both aliases avoids dictionary round trips on hot
    // cache hits while still matching callers that use canonicalized keys.
    mutable bool mBucketActive = false;
    // false once a group came back sparse/array/unsupported and was skipped: then a
    // scalar cache miss for a bucket key cannot be trusted as "unauthored", so it
    // falls back to a per-prim read instead of resolving to empty.
    mutable bool mBucketScalarsComplete = true;
    mutable bool mBucketTransformsComplete = true;
    mutable std::unordered_set<uint64_t> mBucketKeys;
    mutable std::unordered_map<uint64_t, uint32_t> mBucketRows;
    mutable std::unordered_set<uint32_t> mBucketAttributeIds;
    mutable std::unordered_map<uint64_t, std::unordered_map<uint32_t, AttrValue>> mBucketScalars;
    mutable std::unordered_map<uint64_t, Matrix4d> mBucketWorldTransforms;
    mutable std::unordered_map<uint64_t, Matrix4d> mBucketLocalTransforms;
    mutable std::unordered_map<uint64_t, bool> mBucketResetXformStack;
    mutable TokenId mBucketReadGroupAttr;
    mutable const DLTensor* mBucketReadGroupTensor = nullptr;
    mutable int64_t mBucketReadGroupComps = 0;
    // Interned once: seedBucketFromReadGroup runs per read group on the drain hot path.
    mutable bool mBucketAttrTokensValid = false;
    mutable TokenId mBucketLocalTransformAttr;
    mutable TokenId mBucketFabricLocalTransformAttr;
    mutable TokenId mBucketWorldTransformAttr;
    mutable TokenId mBucketResetXformStackAttr;

    // Load-time cache. Unlike the current bucket this survives clearBucket()
    // calls during scanOvstage(), but is explicitly cleared once the load scan is
    // done. Coverage is tracked per (prim, attr) so misses are only trusted when
    // a columnar read proved the attr absent for that prim.
    mutable bool mLoadCacheActive = false;
    // Enables forEachChildRaw()'s complete-child-cache leaf fast path outside a load-cache
    // window. Hierarchy-only: unlike mLoadCacheActive it never seals an attribute miss.
    mutable bool mHierarchyBulkRead = false;
    mutable std::deque<ovstage_read_group_t> mLoadCacheGroups;
    // The load-cache table (REQ-PARSE-FEED-005 AC-7): one flat record per prim instead of a
    // hash node per (handle, attribute) cell. mLoadCachePrimIndex maps both the raw handle
    // and its canonical alias to the same record; mLoadCacheAttrIndex maps a TokenId.id
    // (dense, an index into mTokenToString) to the window's attribute column. The records
    // beyond mLoadCachePrimCount are dormant and reused by the next window, so a steady-state
    // prefetch allocates nothing per cell.
    mutable std::unordered_map<uint64_t, uint32_t> mLoadCachePrimIndex;
    mutable std::vector<LoadCachePrim> mLoadCachePrims;
    mutable uint32_t mLoadCachePrimCount = 0;
    mutable std::vector<uint32_t> mLoadCacheAttrIndex;
    mutable uint32_t mLoadCacheAttrCount = 0;
    mutable uint32_t mPrefetchSerial = 0;
    // Composed world transforms memoized during the load walk. Unlike
    // LoadCachePrim::world (raw data-plane omni:fabric:worldMatrix rows), this
    // holds the result of composing the parent chain of locals for prims whose
    // world matrix is NOT resolved on the data plane. Reusing each ancestor's
    // composed world (UsdGeomXformCache-style) turns whole-scan world resolution
    // from O(prims * depth) into O(prims).
    mutable std::unordered_map<uint64_t, Matrix4d> mLoadCacheComposedWorld;

    // Existence-only cache populated by initial attach from the scan result.
    // It avoids per-prim usd-path queries without affecting bucketed reads.
    mutable std::unordered_set<uint64_t> mKnownKeys;

    // Per-key existence answers filled by exists()/existsBatch() and noteLivePrims(), for keys
    // that neither the bucket nor mKnownKeys covers. Without it every such answer costs
    // a blocking usd-path round trip held under mMutex, so a caller probing thousands of
    // keys (replication clone paths and their ancestors) serializes into a convoy. The
    // bucket / known-key sets are still consulted first, and the memo is dropped by
    // invalidateHierarchyCache(), applyHierarchyDelta(), clearSchemaCache() and
    // clearKnownKeys() -- the structural points -- and by clearExistsMemo() at the start of
    // every drain, so no answer predates the last drained range. clearBucket() leaves it
    // alone: the bucket is an attribute-value cache whose lifetime says nothing about prim
    // existence, and the drain's records must survive it to the flush.
    mutable std::unordered_map<uint64_t, bool> mExistsMemo;

    // Built from a broad read of usd-schemas. Membership answers and multi-apply
    // instance enumeration are local token lookups; schema queries are only a
    // compatibility fallback if the metadata column cannot be read. The change feed keeps the
    // membership maps and the per-prim schema list current across incremental structural
    // drains (applySchemaDelta() / applyPrimRemoval()); readPrimSchemaNames() serves a prim
    // from mSchemasByPrimCache when it has an entry, else from a single-prim read.
    mutable bool mSchemaCacheBuilt = false;
    mutable bool mSchemaCacheComplete = false;
    mutable std::unordered_map<uint32_t, bool> mSchemaMayExistCache;
    mutable std::unordered_map<uint32_t, std::unordered_set<uint64_t>> mSchemaMembershipCache;
    mutable std::unordered_map<uint64_t, std::vector<uint32_t>> mSchemasByPrimCache;
    mutable std::unordered_map<uint32_t, std::unordered_set<uint64_t>> mMultiApplyMembershipCache;
    mutable std::unordered_map<uint64_t, std::unordered_map<uint32_t, std::vector<std::string>>> mMultiApplyInstancesByPrimCache;
    // raw / canonical handle -> usd-prim-type dictionary token id, filled by isA()/getTypeName()
    // reads and the feed's applyPrimTypeDelta(); dropped by clearSchemaCache().
    mutable std::unordered_map<uint64_t, uint64_t> mPrimTypeByRaw;
    // usd-prim-type dictionary token -> handles (raw and canonical alias) carrying it, from one
    // whole-stage read (buildPrimTypeIndex); patched by applyPrimTypeDelta() / applyPrimRemoval(),
    // dropped by clearSchemaCache(). Complete or unusable: an empty entry means no prim has the type.
    mutable bool mPrimTypeIndexBuilt = false;
    mutable bool mPrimTypeIndexComplete = false;
    mutable std::unordered_map<uint64_t, std::unordered_set<uint64_t>> mPrimTypeIndex;

    // Distinct attribute-column vocabulary of the stage, read once from a query result's
    // discovered attributes (no traversal). Backs stageHasAttributeWithPrefix() /
    // stageHasAttributeColumn(); dropped by invalidateAttributeVocabulary() (every drain start)
    // and clearSchemaCache(). Only ever a superset of the columns readable at the read ordinal, so
    // a "no such column" answer is exact.
    mutable bool mAttributeVocabularyBuilt = false;
    mutable std::unordered_set<std::string> mAttributeVocabulary;

    // Built once per source so collection expansion can walk ovstage hierarchy
    // without issuing a prefix query per visited prim; kept current across structural
    // drains by applyHierarchyDelta() so a spawn does not force a whole-stage rebuild.
    mutable bool mChildCacheBuilt = false;
    // True only when buildChildCache()'s authoritative usd-path enumeration
    // completed cleanly (ended at END_OF_ITERATION with rows), i.e. the parent/
    // child edge set is trustworthy and complete. A transient/partial read leaves
    // this false. Gated together with mLoadCacheActive, it lets forEachChild()
    // treat a known childless prim as a leaf (returning empty) during the initial
    // load walk instead of firing the per-call full-stage prefix query that made
    // whole-stage scans O(prims^2). Outside the load walk, or when the build was
    // incomplete, forEachChild() falls back to that live query so post-attach
    // structural edits and retries are still observed.
    mutable bool mChildCacheComplete = false;
    mutable std::unordered_map<uint64_t, std::vector<uint64_t>> mChildCache;
    mutable std::unordered_map<uint64_t, std::string> mPathStringCache;
    mutable std::unordered_map<std::string, uint64_t> mPathToHandleCache;
    mutable std::unordered_map<uint64_t, uint64_t> mCanonicalHandleCache;
    mutable std::unordered_map<uint64_t, uint64_t> mParentHandleCache;
    mutable std::unordered_map<uint64_t, std::vector<uint64_t>> mDescendantCache;

    // Keep the cheap complete prototype-root list for suppressing private backing
    // prims. Normally expand only roots that back physics collision shapes; an
    // ambiguous non-leaf collider outside a prototype conservatively retains the
    // old complete expansion. Each expansion currently rebuilds the complete
    // instancing graph. Attempted/valid latches prevent a failed authoritative
    // read from being retried once per scanned prim; clearSchemaCache() resets
    // both latches for the next structural state.
    mutable bool mPrototypeRootCacheInitialized = false;
    mutable bool mPrototypeRootCacheValid = false;
    mutable bool mPhysicsInstancingCacheInitialized = false;
    mutable bool mPhysicsInstancingCacheValid = false;
    mutable std::unordered_set<std::string> mPrototypeRootPaths;
    mutable std::unordered_map<std::string, std::string> mPrototypeRootByInstanceRoot;
    mutable std::unordered_map<uint64_t, uint64_t> mGeometryBackingCache;

    // Scene-graph proxy material bindings are resolved by ovpopulation into a
    // scalar path column rather than a USD relationship. Cache that column
    // through the public read API so scan-time and consumer-time lookups agree.
    mutable bool mInstanceMaterialCacheBuilt = false;
    mutable bool mInstanceMaterialCacheValid = false;
    mutable std::unordered_map<uint64_t, uint64_t> mInstanceMaterialByPrim;

    // --- Mesh-geometry buffer store (getMeshAttributes / resolveBuffer) --------
    // Owns byte copies of read array columns; keys are monotonic ids (0 = invalid).
    mutable std::unordered_map<uint64_t, std::vector<uint8_t>> mBuffers;
    mutable uint64_t mNextBufferId = 1;
};

} // namespace omni::physics::ovstage
