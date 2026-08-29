// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <omni/physics/parse/IPhysicsSource.h>

#include <carb/extras/Hash.h>

#include <pxr/base/tf/token.h>
#include <pxr/base/vt/array.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xformCache.h>

#include <any>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace omni::physics::usd
{
using namespace omni::physics::parse;

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

    bool hasSchema(ObjectKey key, TokenId schemaToken) const override;
    bool isA(ObjectKey key, TokenId typeToken) const override;
    bool exists(ObjectKey key) const override;
    bool isPrototype(ObjectKey key) const override;
    bool isInstanceProxy(ObjectKey key) const override;
    bool isInstance(ObjectKey key) const override;
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

    MeshGeometry getMeshAttributes(ObjectKey key) const override;

    BufferHandle getArrayAttribute(ObjectKey key, TokenId attr, ReadTime time) const override;
    void releaseBuffer(BufferHandle handle) const override;

    SourceUnits getSourceUnits() const override;

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
    struct BufferEntry
    {
        const void* ptr = nullptr;
        size_t byteCount = 0;
        std::any keepalive; // holds the VtArray copy keeping `ptr` alive
    };
    PXR_NS::UsdStageWeakPtr mStage;

    // Bidirectional SdfPath <-> ObjectKey intern table
    mutable std::unordered_map<PXR_NS::SdfPath, ObjectKey, PXR_NS::SdfPath::Hash> mPathToKey;
    mutable std::vector<PXR_NS::SdfPath> mKeyToPath;

    // Bidirectional TfToken <-> TokenId intern table
    mutable std::unordered_map<PXR_NS::TfToken, TokenId, PXR_NS::TfToken::HashFunctor> mTokenToId;
    mutable std::vector<PXR_NS::TfToken> mIdToToken;

    // Cached string representations for sourceKeyToString return stability
    mutable std::vector<std::string> mKeyStrings;

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
