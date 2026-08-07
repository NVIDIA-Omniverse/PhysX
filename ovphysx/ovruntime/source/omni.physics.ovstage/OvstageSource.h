// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace omni::physics::ovstage
{
using namespace omni::physics::parse;

// Attribute-name conventions shared between OvstageSource (reader), the
// OvstageWalker (enumeration probe), and the test OvstagePopulator (writer).
// Stage units match the real ovpopulation data model so the source reads both a
// controlled (populator-built) instance and a Fabric-backed one populated via
// ovpopulation:
//   - stage units → on the "/__ovstage_population_stage_info__" prim, named
//     metersPerUnit / kilogramsPerUnit (doubles) and upAxis (a token id "Y"/"Z").
//     The legacy "/__ovpopulation_stage_info__" path is still accepted for older populated instances.
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
inline constexpr const char* kStageInfoPath = "/__ovstage_population_stage_info__"; //!< prim holding stage units
inline constexpr const char* kLegacyStageInfoPath = "/__ovpopulation_stage_info__"; //!< legacy prim holding stage units
inline constexpr const char* kDefaultScenePath = "/__defaultPhysicsScene__"; //!< synthetic default-scene identity (no prim authored)
inline constexpr const char* kMetersPerUnit = "metersPerUnit"; //!< double on kStageInfoPath
inline constexpr const char* kKilogramsPerUnit = "kilogramsPerUnit"; //!< double on kStageInfoPath
inline constexpr const char* kUpAxis = "upAxis"; //!< token id ("Y"/"Z") on kStageInfoPath
inline constexpr const char* kInactiveIds = "inactiveIds"; //!< int64[] PointInstancer inactive-id metadata
} // namespace conv

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

    ObjectKey getRootKey() const override;
    bool exists(ObjectKey key) const override;
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
        return ObjectKey{ canonicalPath(key) };
    }

    bool hasSchema(ObjectKey key, TokenId schemaToken) const override;
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
    // Shared implementation behind getLocalTransform: reads the object-local transform,
    // resolving any USD-stage fallback at `time` (ovstage data-plane reads are a single ordinal
    // snapshot and ignore it). When `usdAtEarliestTime` is set the USD fallback reads at
    // UsdTimeCode::EarliestTime() instead of `time`; the no-arg getLocalToWorldTransform uses
    // this for its manual parent-chain fallback so it matches the attached-stage
    // UsdGeomXformCache path (and UsdSource's static no-arg overload). The pxr timecode is
    // resolved in the .cpp to keep this header pxr-free.
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
    MeshGeometry getMeshAttributes(ObjectKey key) const override;
    BufferHandle getArrayAttribute(ObjectKey key, TokenId attr, ReadTime time) const override;
    BufferHandle readArrayAttribute(ObjectKey key, std::string_view attr, BufferElemType type, int comps) const;
    SourceUnits getSourceUnits() const override;
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
    // Keep bucket-prefetched values alive across the whole ovstage load scan.
    // The scanner still uses narrow buckets for concept-local parsing, but this
    // load cache keeps ovstage read groups alive and remembers row references plus
    // covered misses so later concepts do not re-query columns already read.
    void beginLoadCache() const;
    void clearLoadCache() const;
    // Seed existence checks with keys proven live by a just-completed scan.
    // Kept separate from the attribute bucket so exists() can be O(1) without
    // changing getAttribute cache semantics.
    void seedKnownKeys(const std::vector<ObjectKey>& keys) const;
    void clearKnownKeys() const;
    // Seed the same scalar bucket from a change-feed read group that has already
    // been fetched. This lets update callbacks use getAttribute/getValue without
    // causing a second ovstage read for the same changed column.
    void seedBucketFromReadGroup(TokenId attr,
                                 const ovstage_read_group_t& group,
                                 const ObjectKey* keys,
                                 size_t keyCount) const;
    void clearBucket() const;
    void clearSchemaCache() const;
    bool collectSchemaKeys(TokenId schemaToken, std::vector<ObjectKey>& out) const;
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
    ovx_primpath_t canonicalPath(ObjectKey key) const { return canonicalHandle(key); }

    // Resolve the storage prim that owns geometry authored for a logical scene-
    // graph instance proxy. Non-instanced keys resolve to themselves. Callers
    // must keep descriptor identity, transforms, materials, and relationships on
    // the logical key; this backing key is only for type and geometry reads.
    ObjectKey geometryBackingKey(ObjectKey key) const;
    // True for the private prototype root or any descendant below it. Those
    // keys provide backing data and must never become physics descriptors.
    bool isPrototypeBackingKey(ObjectKey key) const;

private:
    struct CachedTensorRow
    {
        const DLTensor* tensor = nullptr;
        int64_t comps = 0;
        uint32_t row = 0;
    };

    struct CachedArrayRow
    {
        const DLTensor* tensor = nullptr;
    };

    TokenId doInternToken(std::string_view token) const;

    // Resolve an ObjectKey (== ovx_primpath_t) to its path string.
    std::string pathOf(ObjectKey key) const;
    // Normalise a key's prim-path handle through its path string: intern_path
    // (findByPath) and get_paths (read-group prim lists) can hand back DISTINCT
    // ovx_primpath_t for the same path. Returns 0 for an unresolvable key.
    uint64_t canonicalHandle(ObjectKey key) const;
    bool buildPrototypeRootCache() const;
    bool buildPhysicsInstancingCache() const;
    bool buildInstanceMaterialCache() const;
    // Intern a string in the ovstage path dictionary's token space.
    ovx_token_t ovxToken(std::string_view s) const;
    void buildChildCache() const;
    bool buildSchemaCache() const;
    const std::unordered_set<uint64_t>* schemaMembershipFromQuery(TokenId schemaToken) const;

    // Read attribute `attrName` for the single prim `key` at the read ordinal and
    // invoke `fn` with the first read group's DLTensor + prim count. Returns
    // false if the source is unconfigured or the attribute has no data. Handles
    // are all released before returning.
    bool withAttributeTensor(ObjectKey key,
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
    uint64_t mUsdStageId = 0;

    // String<->uint32 token table. parse TokenId and ovx_token_t stay distinct
    // because their widths are part of separate API contracts.
    // Guards dictionary calls plus mutable token/cache/buffer state. The source is normally
    // used single-threaded, but replicator clone workers read transforms concurrently.
    mutable std::recursive_mutex mMutex;

    mutable std::unordered_map<std::string, TokenId> mStringToToken;
    mutable std::deque<std::string> mTokenToString;

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

    // Load-time cache. Unlike the current bucket this survives clearBucket()
    // calls during scanOvstage(), but is explicitly cleared once the load scan is
    // done. Coverage is tracked per (prim, attr) so misses are only trusted when
    // a columnar read proved the attr absent for that prim.
    mutable bool mLoadCacheActive = false;
    mutable std::deque<ovstage_read_group_t> mLoadCacheGroups;
    mutable std::unordered_map<uint64_t, std::unordered_set<uint32_t>> mLoadCacheCoveredAttrs;
    mutable std::unordered_map<uint64_t, std::unordered_map<uint32_t, CachedTensorRow>> mLoadCacheScalars;
    mutable std::unordered_map<uint64_t, std::unordered_map<uint32_t, CachedArrayRow>> mLoadCacheRelationships;
    mutable std::unordered_map<uint64_t, CachedTensorRow> mLoadCacheWorldTransforms;
    mutable std::unordered_map<uint64_t, CachedTensorRow> mLoadCacheLocalTransforms;
    mutable std::unordered_map<uint64_t, CachedTensorRow> mLoadCacheResetXformStack;
    // Composed world transforms memoized during the load walk. Unlike
    // mLoadCacheWorldTransforms (raw data-plane omni:fabric:worldMatrix rows), this
    // holds the result of composing the parent chain of locals for prims whose
    // world matrix is NOT resolved on the data plane. Reusing each ancestor's
    // composed world (UsdGeomXformCache-style) turns whole-scan world resolution
    // from O(prims * depth) into O(prims).
    mutable std::unordered_map<uint64_t, Matrix4d> mLoadCacheComposedWorld;

    // Existence-only cache populated by initial attach from the scan result.
    // It avoids per-prim usd-path queries without affecting bucketed reads.
    mutable std::unordered_set<uint64_t> mKnownKeys;

    // Built from a broad read of usd-schemas. Membership answers and multi-apply
    // instance enumeration are local token lookups; schema queries are only a
    // compatibility fallback if the metadata column cannot be read.
    mutable bool mSchemaCacheBuilt = false;
    mutable bool mSchemaCacheComplete = false;
    mutable std::unordered_map<uint32_t, bool> mSchemaMayExistCache;
    mutable std::unordered_map<uint32_t, std::unordered_set<uint64_t>> mSchemaMembershipCache;
    mutable std::unordered_map<uint64_t, std::vector<uint32_t>> mSchemasByPrimCache;
    mutable std::unordered_map<uint32_t, std::unordered_set<uint64_t>> mMultiApplyMembershipCache;
    mutable std::unordered_map<uint64_t, std::unordered_map<uint32_t, std::vector<std::string>>> mMultiApplyInstancesByPrimCache;

    // Built once per source so collection expansion can walk ovstage hierarchy
    // without issuing a prefix query per visited prim.
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
