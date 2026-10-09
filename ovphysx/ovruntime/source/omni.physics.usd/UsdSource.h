// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <omni/physics/parse/IPhysicsSource.h>

#include <carb/extras/Hash.h>
#include <carb/Warning.h>

#include <pxr/base/tf/token.h>
#include <pxr/base/vt/array.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xformCache.h>

#include <any>
#include <array>
#include <deque>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace omni::physics::usd
{
using namespace omni::physics::parse;

// The UsdUtilsStageCache id for `stage`, or 0 when there is no stage. The
// single place the parse layer maps a live stage to its cache id; the guarded
// form is required because an invalid UsdStageCache::Id does not round-trip
// to 0 (REQ-PARSE-BACKEND-001 AC-6).
long usdStageCacheId(PXR_NS::UsdStageWeakPtr stage);

class UsdSource final : public IPhysicsSource
{
public:
    explicit UsdSource(PXR_NS::UsdStageWeakPtr stage);
    ~UsdSource() override;

    // IPhysicsSource overrides
    std::string_view sourceKeyToString(ObjectKey key) const override;
    TokenId internToken(std::string_view token) const override;
    std::string_view tokenToString(TokenId id) const override;

    ObjectKey getRootKey() const override;
    void forEachChild(ObjectKey parent, std::function<void(ObjectKey)> cb) const override;
    void forEachDescendant(ObjectKey root, std::function<void(ObjectKey)> cb) const override;
    void forEachDescendantPruned(ObjectKey root, std::function<bool(ObjectKey)> visit,
                                 DescendantScope scope = DescendantScope::eAll) const override;
    ObjectKey findByPath(std::string_view path) const override;
    // Existence-independent: interns the SdfPath directly (keyFor), so a not-yet-authored
    // path (e.g. a runtime clone target) still mints a stable key -- unlike findByPath.
    ObjectKey mintKeyForPath(std::string_view path) const override;
    std::vector<ObjectKey> remapKeysFrom(const IPhysicsSource& source,
                                       const std::vector<ObjectKey>& keys) const override;
    ObjectKey replacePathPrefix(ObjectKey key, ObjectKey prefix, ObjectKey replacement) const override;

    bool hasSchema(ObjectKey key, TokenId schemaToken) const override;
    bool isA(ObjectKey key, TokenId typeToken) const override;
    bool exists(ObjectKey key) const override;
    bool isPrototype(ObjectKey key) const override;
    bool isInstanceProxy(ObjectKey key) const override;
    bool isInstance(ObjectKey key) const override;
    bool isInPrototype(ObjectKey key) const override;
    TokenId getTypeName(ObjectKey key) const override;

    AttrValue getAttribute(ObjectKey key, TokenId attr) const override;
    // Pull in the typed `getAttribute(..., T& out)` overloads from the base
    // class so callers can use them without explicit qualification (without
    // the using-decl, the AttrValue-returning override above would hide them).
    using IPhysicsSource::getAttribute;

    AttrValue getAttributeAtTime(ObjectKey key, TokenId attr, ReadTime time) const override;

    std::unique_ptr<IChangeFeed> createChangeFeed() override;

    bool hasAuthoredAttribute(ObjectKey key, TokenId attr) const override;

    bool isAttributeTimeSampled(ObjectKey key, TokenId attr) const override;
    bool mightBeTimeVarying(ObjectKey key, TokenId attr) const override;

    void getLocalToWorldTransform(ObjectKey key, Matrix4d& outMatrix) const override;
    void getLocalToWorldTransform(ObjectKey key, ReadTime time, Matrix4d& outMatrix) const override;

    void getLocalToWorldRotationAndScale(ObjectKey key,
                                         Matrix3d& outRotation,
                                         carb::Float3& outScale) const override;

    void getLocalTransform(ObjectKey key, ReadTime time, Matrix4d& outMatrix,
                           bool& outResetsXformStack) const override;

    bool mightWorldTransformBeTimeVarying(ObjectKey key) const override;

    ObjectKey getParent(ObjectKey key) const override;

    void getRelationshipTargets(ObjectKey key, TokenId rel, std::vector<ObjectKey>& out) const override;
    bool hasRelationship(ObjectKey key, TokenId rel) const override;
    void getInactiveInstanceIds(ObjectKey key, std::vector<int64_t>& out) const override;

    const void* resolveBuffer(BufferHandle handle, size_t& byteCount) const override;

    MeshGeometry getMeshAttributes(ObjectKey key, bool includeFaceMaterials = true) const override;

    BufferHandle getArrayAttribute(ObjectKey key, TokenId attr, ReadTime time) const override;
    void releaseBuffer(BufferHandle handle) const override;

    SourceUnits getSourceUnits() const override;

    uint64_t residentUsdStageId() const override
    {
        return static_cast<uint64_t>(mStageId);
    }

    void resolveCollection(ObjectKey primKey,
                           TokenId collectionName,
                           std::vector<ObjectKey>& members) const override;

    void forEachMultiApplyInstance(
        ObjectKey key,
        std::string_view baseSchema,
        std::function<void(std::string_view instance)> cb) const override;

    void forEachAppliedSchema(ObjectKey key, std::function<void(TokenId)> cb) const override;

    ObjectKey getMaterialBinding(ObjectKey primKey) const override;

    // USD-specific helpers — used by the USD backend and change-tracking code

    // The stage's UsdUtilsStageCache id, snapshotted at construction (0 when
    // there is no stage). Snapshotted rather than recomputed on every call so a
    // stage that is later erased from the cache does not silently change this
    // attach's id: the id is the attach registry key, and a key that shifts
    // under a live attach would strand the registration. Typed `long` to match
    // UsdStageCache::Id::ToLongInt; residentUsdStageId() is the backend-neutral
    // spelling of the same value.
    long getStageId() const
    {
        return mStageId;
    }

    bool isForStage(PXR_NS::UsdStageWeakPtr stage) const
    {
        return mStage == stage;
    }
    // A scan shares this owner's identity table, but owns fresh token, transform
    // and buffer state. It can retain its snapshot without retaining runtime state.
    std::unique_ptr<UsdSource> makeScanSource() const;

    ObjectKey keyFor(const PXR_NS::SdfPath& path) const;
    PXR_NS::SdfPath pathFor(ObjectKey key) const;

    TokenId tokenFor(const PXR_NS::TfToken& token) const;
    PXR_NS::TfToken tfTokenFor(TokenId id) const;

    // Mint a BufferHandle from a VtArray. Stores the array (refcount bump)
    // to keep the cdata() pointer alive, computes a 128-bit fnv hash of the
    // raw bytes (same hash function the cooking service uses internally —
    // `carb::extras::fnv128hash`, exposed via MeshKey::computeVerticesHash),
    // and assigns a monotonic id. Empty arrays return an invalid handle.
    //
    // The buffer lives until the UsdSource is destroyed; there is no explicit
    // release in Sh1. Callers that need narrower lifetime can call
    // releaseBuffers() to clear all registered buffers.
    template <typename T>
    BufferHandle registerBuffer(const PXR_NS::VtArray<T>& array, BufferElemType type) const;

    // Drop all currently-registered buffers. Existing BufferHandles become
    // unresolvable. Useful at parse-run boundaries.
    void releaseBuffers() const;

private:
    struct ScanIdentity {};
    UsdSource(const UsdSource& owner, ScanIdentity);
    static constexpr uint32_t kInternShardBits = 5;
    static constexpr uint32_t kInternShardCount = 1u << kInternShardBits;
    // Cache-line padding is intentional so adjacent shard locks do not share a line.
    CARB_IGNOREWARNING_MSC_WITH_PUSH(4324)
    struct alignas(64) InternShard
    {
        mutable std::shared_mutex mutex;
        std::unordered_map<PXR_NS::SdfPath, ObjectKey, PXR_NS::SdfPath::Hash> pathToKey;
        std::deque<PXR_NS::SdfPath> keyToPath;
    };
    CARB_IGNOREWARNING_MSC_POP
    static uint32_t shardForPath(const PXR_NS::SdfPath& path)
    {
        return static_cast<uint32_t>(path.GetHash()) & (kInternShardCount - 1);
    }
    // Caller holds this shard exclusively; path must be nonempty.
    ObjectKey keyForLocked(InternShard& shard, uint32_t shardIndex, const PXR_NS::SdfPath& path) const;

    // High 32 bits retain the source generation. Low bits encode a shard and
    // its 1-based index. Slot zero in every shard is the invalid sentinel.
    ObjectKey packKey(uint32_t shard, uint32_t localIndex) const
    {
        return ObjectKey{(static_cast<uint64_t>(mGeneration) << 32) |
                         (static_cast<uint64_t>(localIndex) << kInternShardBits) | shard};
    }
    uint32_t decodeLocalIndex(ObjectKey key) const
    {
        if (key.handle == 0 || static_cast<uint32_t>(key.handle >> 32) != mGeneration)
            return 0;
        return static_cast<uint32_t>(key.handle) >> kInternShardBits;
    }

    struct CachedPath
    {
        ObjectKey key;
        PXR_NS::SdfPath path;
        const std::string* text = nullptr;
    };
    static CachedPath& cacheFor(ObjectKey key);

    struct BufferEntry
    {
        const void* ptr = nullptr;
        size_t byteCount = 0;
        std::any keepalive; // holds the VtArray copy keeping `ptr` alive
    };
    PXR_NS::UsdStageWeakPtr mStage;

    // Stage-cache id of mStage, resolved once in the constructor — see
    // getStageId() for why it is a snapshot rather than a live lookup.
    long mStageId = 0;

    // Per-owner identity folded into the high 32 bits of every ObjectKey this
    // Source mints (see keyFor()/pathFor()/sourceKeyToString()). A fresh UsdSource
    // is constructed on every attach/reattach (AttachedStage::rebuildSource), and
    // its own per-shard path tables restart numbering from 1 each time
    // -- so the Nth path interned by one UsdSource instance and the Nth path
    // interned by the next instance would otherwise mint the SAME raw ObjectKey.
    // A stale key held across a detach/reattach could then silently resolve
    // against whichever live object the new instance happens to have put at that
    // same slot. mGeneration makes that collision detectable: it is assigned from
    // nextObjectKeyGeneration() (Handles.h), a counter shared process-wide with
    // OvstageSource, so no two UsdSource instances (in this process) ever share
    // one -- and, since the counter is shared rather than a per-class statics,
    // a key minted by a fresh UsdSource can't alias one minted by a fresh
    // OvstageSource either, closing the cross-backend case of the same gap
    // (a stale key retained across a USD<->ovstage source switch). A key
    // minted by a previous instance of either backend decodes to a generation
    // that will never match this one's. This mirrors the disambiguation trick
    // AttachHandle already uses process-wide for the same class of problem
    // (ADR-0013 / ADR-0016) -- scoped down to a plain shared counter here
    // because UsdSource is constructed before its owning AttachedStage's
    // AttachHandle is minted (LoadUsd.cpp loadAttachedStage()), so the real
    // AttachHandle is not yet available at this point.
    // Owned scan contexts inherit their attachment's identity namespace while
    // keeping transient token/transform/buffer state private.
    const uint32_t mGeneration;

    // Independent path shards avoid serializing collection and clone workers
    // on one table. Each shard synchronizes reads and insertion; retained path
    // nodes keep identities and diagnostic text stable for this source's life.
    std::shared_ptr<std::array<InternShard, kInternShardCount>> mInternShards;

    // Bidirectional TfToken <-> TokenId intern table
    mutable std::unordered_map<PXR_NS::TfToken, TokenId, PXR_NS::TfToken::HashFunctor> mTokenToId;
    mutable std::vector<PXR_NS::TfToken> mIdToToken;

    // Lazily-built xform cache for getLocalToWorldTransform. mutable because
    // the cache populates on read but the source itself is logically const.
    mutable std::unique_ptr<PXR_NS::UsdGeomXformCache> mXformCache;

    // BufferHandle registry. Keys are monotonic ids (0 reserved for invalid).
    mutable std::unordered_map<uint64_t, BufferEntry> mBuffers;
    mutable uint64_t mNextBufferId = 1;
};

// Template definition — kept in the header so callers can instantiate any
// VtArray<T> they please without explicit per-T entry points.
template <typename T>
BufferHandle UsdSource::registerBuffer(const PXR_NS::VtArray<T>& array, BufferElemType type) const
{
    if (array.empty())
        return BufferHandle{};

    const size_t byteCount = array.size() * sizeof(T);
    const auto fullHash = carb::extras::fnv128hash(
        reinterpret_cast<const uint8_t*>(array.cdata()), byteCount);

    BufferHandle h;
    h.id = mNextBufferId++;
    h.elemCount = static_cast<uint32_t>(array.size());
    h.type = type;
    h.contentHash[0] = fullHash.d[0];
    h.contentHash[1] = fullHash.d[1];

    BufferEntry entry;
    entry.ptr = static_cast<const void*>(array.cdata());
    entry.byteCount = byteCount;
    entry.keepalive = array; // VtArray COW: refcount bump, no allocation
    mBuffers.emplace(h.id, std::move(entry));
    return h;
}

// On-demand down-cast from the backend-neutral source to the concrete USD
// source (ADR-0005). Returns null when the active backend is not USD, so
// USD-specific consumers can probe and degrade gracefully.
inline UsdSource* asUsdSource(IPhysicsSource* source)
{
    return dynamic_cast<UsdSource*>(source);
}

inline const UsdSource* asUsdSource(const IPhysicsSource* source)
{
    return dynamic_cast<const UsdSource*>(source);
}

} // namespace omni::physics::usd
