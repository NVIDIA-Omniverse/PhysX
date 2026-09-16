// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-WRITE-CORE-001
 * @covers AC-3 AC-4 AC-5 AC-8 AC-9 AC-10 AC-11
 *
 * @implements REQ-WRITE-AUTHORING-001
 * @covers AC-4
 *
 * @implements REQ-WRITE-TRANSFORM-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-WRITE-DATA-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-WRITE-ARRAY-001
 * @covers AC-3 AC-4 AC-5
 *
 * @implements REQ-WRITE-LOCALXFORM-001
 * @covers AC-1 AC-2
 */
#pragma once

#include <pxr/usd/sdf/changeBlock.h>
#include <pxr/usd/usd/editContext.h>
#include <pxr/usd/usdGeom/xformCache.h>

#include <omni/physics/parse/IPhysicsDataWrite.h>

#include "XformOpResetStorage.h"

#include <carb/tasking/TaskingUtils.h>

#include <memory>
#include <unordered_map>

namespace omni::physics::usd
{

// Forward declaration: we hold a raw pointer to the UsdSource owned by the
// AttachedStage (same lifetime), so this header doesn't pull in its header.
class UsdSource;
} // namespace omni::physics::usd

namespace omni::physics::parse
{
class IPhysicsSource;
}

namespace omni::physics::usd
{

// USD-backed implementation of the source-agnostic physics output sink. Owned
// by AttachedStage alongside the UsdSource it resolves keys through (rebuilt
// together in AttachedStage::rebuildUsdSource), so it never outlives the stage.
//
// USD-backend-only ObjectKey -> SdfPath (via the UsdSource intern table)
// -> UsdPrim resolution. Writes are read-only
// w.r.t. the intern table (pathFor only, never keyFor), so they are safe on
// the lock-free side of the replicator concurrency contract -- the sole
// exception is resolveNearestXformableAncestor, whose only caller runs once
// at vehicle attach time, off the per-frame parallel write path.
class UsdPhysicsDataWrite final : public omni::physics::parse::IPhysicsDataWrite
{
public:
    // `fallbackSource` resolves keys/tokens when `source` is null: the sink then authors into
    // `stage` on behalf of an attach whose active source is not USD (ovstage with a resident
    // backing stage), resolving ObjectKey/TokenId through that source's string identity.
    UsdPhysicsDataWrite(PXR_NS::UsdStageWeakPtr stage,
                        const omni::physics::usd::UsdSource* source,
                        const omni::physics::parse::IPhysicsSource* fallbackSource = nullptr);
    ~UsdPhysicsDataWrite() override;

    // When true, prepareTransformWrite leaves XformCommonAPI-compatible prims in
    // XformCommonAPI form and writeTransforms authors via SetXformVectors.
    void setUpdateToUsdUsingXformCommonAPI(bool value) override
    {
        mUpdateUsingXformCommonAPI = value;
    }

    void beginWrite() override;
    void endWrite() override;

    void beginFrameWrite(void* destinationOverride) override;
    void endFrameWrite() override;

    bool createDefaultPhysicsScene(omni::physics::parse::ObjectKey sceneKey) override;
    void removeDefaultPhysicsScene(omni::physics::parse::ObjectKey sceneKey) override;

    // destinationOverride: same non-owning pointer-sized SdfLayerRefPtr-bytes
    // convention as beginFrameWrite's -- scopes the one-time xform-op
    // normalization's edits (new xformOp attributes / xformOpOrder rewrite) to
    // that layer instead of the stage's current edit target, wrapped in a
    // UsdEditContext for the duration of the call.
    void prepareTransformWrite(const omni::physics::parse::ObjectKey* keys,
                               size_t count,
                               bool* outEligible,
                               void* destinationOverride = nullptr) override;
    void releaseTransformWrite(const omni::physics::parse::ObjectKey* keys, size_t count) override;
    omni::physics::parse::WheelScaleReadResult readWheelLocalScale(omni::physics::parse::ObjectKey key,
                                                                    carb::Float3& outScale) override;
    omni::physics::parse::ObjectKey resolveNearestXformableAncestor(omni::physics::parse::ObjectKey key) override;

    void storeXformOpReset(omni::physics::parse::ObjectKey key) override;
    void restoreXformOpReset(omni::physics::parse::ObjectKey key, bool purgeOrphanedTranslateOrientScale) override;

    void writeTransforms(const omni::physics::parse::ObjectKey* keys,
                         size_t count,
                         const omni::physics::parse::DataWriteView& positions,
                         const omni::physics::parse::DataWriteView& orientations,
                         const omni::physics::parse::DataWriteView& scales) override;

    void writeData(const omni::physics::parse::ObjectKey* keys,
                   size_t count,
                   omni::physics::parse::TokenId attr,
                   const omni::physics::parse::DataWriteView& data) override;

    void writeArray(omni::physics::parse::ObjectKey key,
                    omni::physics::parse::TokenId attr,
                    const omni::physics::parse::DataWriteView& data) override;

    bool promoteEarliestSampleToDefault(omni::physics::parse::ObjectKey key,
                                        omni::physics::parse::TokenId attr) override;

    void clearArray(omni::physics::parse::ObjectKey key, omni::physics::parse::TokenId attr) override;

    void writeProxyPurpose(omni::physics::parse::ObjectKey key, bool proxy) override;

    void defineAnisotropyPrimvars(omni::physics::parse::ObjectKey key) override;

    void setIsosurfaceMeshEnabled(omni::physics::parse::ObjectKey particleSystemKey, bool enabled) override;

    void setDiffuseParticleRenderingEnabled(omni::physics::parse::ObjectKey particleSystemKey, bool enabled) override;

    void writeIsosurfaceMesh(omni::physics::parse::ObjectKey particleSystemKey,
                             const carb::Float3* points,
                             size_t numPoints,
                             const carb::Float3* normals,
                             size_t numNormals,
                             const int32_t* faceVertexCounts,
                             size_t numFaces,
                             const int32_t* faceVertexIndices,
                             size_t numIndices) override;

    void writeDiffuseParticlePoints(omni::physics::parse::ObjectKey particleSystemKey,
                                    const carb::Float3* points,
                                    size_t numPoints,
                                    const carb::Float3* colors,
                                    size_t numColors) override;

    void prepareTriggerWrite(const omni::physics::parse::ObjectKey* keys, size_t count, bool* outEligible) override;
    void writeTriggerCollisions(omni::physics::parse::ObjectKey trigger,
                                const omni::physics::parse::ObjectKey* targets,
                                size_t count) override;
    void releaseTriggerWrite(const omni::physics::parse::ObjectKey* keys, size_t count) override;

    void writeByteArrayAttribute(omni::physics::parse::ObjectKey key,
                                 std::string_view attrName,
                                 const uint8_t* data,
                                 size_t count) override;

    void writeUIntAttribute(omni::physics::parse::ObjectKey key, std::string_view attrName, uint32_t value) override;
    void removeAttribute(omni::physics::parse::ObjectKey key, std::string_view attrName) override;

    void writeVtxTetAttachment(omni::physics::parse::ObjectKey key,
                               const std::vector<int32_t>& vtxIndicesSrc0,
                               const std::vector<int32_t>& tetIndicesSrc1,
                               const std::vector<carb::Float3>& tetCoordsSrc1,
                               bool enabled) override;

    void writeVtxXformAttachment(omni::physics::parse::ObjectKey key,
                                 const std::vector<int32_t>& vtxIndicesSrc0,
                                 const std::vector<carb::Float3>& localPositionsSrc1,
                                 bool enabled) override;

    void writeElementCollisionFilter(omni::physics::parse::ObjectKey key,
                                     const std::vector<uint32_t>& groupElemCounts0,
                                     const std::vector<uint32_t>& groupElemIndices0,
                                     const std::vector<uint32_t>& groupElemCounts1,
                                     const std::vector<uint32_t>& groupElemIndices1,
                                     bool enabled) override;

    void removeAppliedAPI(omni::physics::parse::ObjectKey key,
                          std::string_view apiName,
                          std::string_view instanceName) override;

    void writeFloatAttribute(omni::physics::parse::ObjectKey key, std::string_view attrName, float value) override;
    void writeIntAttribute(omni::physics::parse::ObjectKey key, std::string_view attrName, int32_t value) override;
    void writeBoolAttribute(omni::physics::parse::ObjectKey key, std::string_view attrName, bool value) override;

    // One-shot LOCAL-space pose authoring from a full 4x4, for the attach/reset
    // writes that are not part of the per-step output path (vehicle wheel and
    // wheel-shape initial transforms, the wheel-shape identity reset, and the
    // per-step wheel pose whose world->local solve the caller already did).
    //
    // Takes a matrix rather than a decomposed (position, orientation, scale)
    // because these callers hand over a product of two arbitrary affine
    // transforms (`affineInverse(parentWorld) * world`), which can carry shear
    // that a TRS triple cannot represent. Decomposing at the call site would
    // silently drop it. The matrix is LOCAL to the prim's parent -- no parent
    // frame is resolved here, unlike writeTransforms.
    void writeLocalTransformMatrix(omni::physics::parse::ObjectKey key,
                                   const omni::physics::parse::Matrix4d& localMatrix,
                                   bool setScale) override;

    // USD-backend-specific attribute authoring used by the cooking write-back
    // (deliberately NOT part of IPhysicsDataWrite — attribute creation with a USD
    // value type is a USD concept a backend-neutral interface should not carry).
    // Unlike writeData/writeArray (which only write existing attributes and infer
    // shape from the destination), these CREATE the attribute with the given USD
    // value type if absent. Keyed by ObjectKey, resolved to a prim internally.
    void writeUIntAttribute(omni::physics::parse::ObjectKey key, const PXR_NS::TfToken& attr, uint32_t value);
    void writeUCharArrayAttribute(omni::physics::parse::ObjectKey key,
                                  const PXR_NS::TfToken& attr,
                                  const uint8_t* data,
                                  size_t count);
    void removeAttribute(omni::physics::parse::ObjectKey key, const PXR_NS::TfToken& attr);

    // Create-or-set a bool attribute (mirrors writeUIntAttribute). Used for ad
    // hoc cosmetic markers (e.g. the particle-sampling hydra "skip while
    // rewriting" hint) that are not part of any schema token vocabulary.
    void writeBoolAttribute(omni::physics::parse::ObjectKey key, const PXR_NS::TfToken& attr, bool value);

    // Author UsdGeomImageable visibility (Inherited/Invisible).
    void writeVisibility(omni::physics::parse::ObjectKey key, bool visible) override;

    // Set the radius of a UsdGeomPointInstancer's first prototype (a
    // UsdGeomSphere), used by the particle-sampling instancer path to size
    // the rendered point sprite from the sampled point width.
    void writeInstancerProtoRadius(omni::physics::parse::ObjectKey instancerKey, float radius) override;

    // USD-only write boundary for legacy schema APIs that still require UsdPrim.
    PXR_NS::UsdPrim usdPrimForWrite(omni::physics::parse::ObjectKey key) const;

private:
    // Per-object setup for one key; loops in the batched prepareTransformWrite.
    bool prepareTransformWriteOne(omni::physics::parse::ObjectKey key);
    PXR_NS::UsdPrim primFor(omni::physics::parse::ObjectKey key) const;
    PXR_NS::TfToken tfTokenFor(omni::physics::parse::TokenId id) const;

    // Per-object state cached by prepareTransformWrite, applied per write.
    // The residual "extra" transform captures any xform ops that couldn't be
    // normalized to scale/orient/translate; it is folded into the authored
    // local pose exactly as InternalActor used to do via processExtraTransforms.
    // (The parent xformable is resolved per-frame in writeTransforms, so a
    // reparented or moving parent is always picked up.)
    struct TransformWriteState
    {
        bool hasExtraTransform = false;
        bool extraTransformPreMultiply = false;
        PXR_NS::GfMatrix4d extraTransformInverse{ 1.0 };
        // When set, the prim's xform-op stack is left in XformCommonAPI form
        // (not normalized to scale/orient/translate); the local pose is authored
        // via UsdGeomXformCommonAPI::SetXformVectors honoring its rotation order.
        bool useXformCommonAPI = false;
    };

    PXR_NS::UsdStageWeakPtr mStage;
    const omni::physics::usd::UsdSource* mSource;
    // Used only when mSource is null; see the constructor comment.
    const omni::physics::parse::IPhysicsSource* mFallbackSource = nullptr;

    // Mirror of the consumer's `updateToUsdUsingXformCommonAPI` setting, pushed
    // via setUpdateToUsdUsingXformCommonAPI before prepareTransformWrite. Read
    // only at prepare time (cached per object in TransformWriteState).
    bool mUpdateUsingXformCommonAPI = false;

    // Resolves parent-frame world transforms for the world->local conversion.
    // Reset at each beginWrite() so a moving parent is re-read each frame.
    PXR_NS::UsdGeomXformCache mXformCache;

    std::unordered_map<omni::physics::parse::ObjectKey, TransformWriteState, omni::physics::parse::ObjectKey::Hash>
        mTransformState;

    // storeXformOpReset/restoreXformOpReset snapshot state, keyed by the object
    // that was snapshotted. A snapshot is store/consume/erase: restoreXformOpReset
    // removes its entry once applied.
    std::unordered_map<omni::physics::parse::ObjectKey, XformOpResetStorage, omni::physics::parse::ObjectKey::Hash>
        mXformOpResetState;

    // Guards mTransformState writes only. prepareTransformWrite/releaseTransformWrite
    // run at object setup, which on the replicator clone path happens across
    // parallel carb-tasking fibers (hence the carb fiber-aware mutex, not
    // std::mutex). The per-frame writeTransforms reads the map lock-free: it is
    // temporally disjoint from setup (no objects are prepared mid-update).
    carb::tasking::MutexWrapper mTransformStateMutex;

    // Open between beginWrite()/endWrite() to batch authoring into one change
    // notice (and one undo step) the way the per-frame write loop expects.
    std::unique_ptr<PXR_NS::SdfChangeBlock> mChangeBlock;

    // Open between beginFrameWrite()/endFrameWrite() -- a coarser, whole-frame
    // scope distinct from mChangeBlock/beginWrite() above (deliberately separate
    // members: a beginWrite()/endWrite() pair issued while a frame write is open
    // nests safely, since SdfChangeBlock's open/close is a global count, not
    // owned by a single instance). mFrameEditContext is only engaged when
    // beginFrameWrite() is given a non-null destination override.
    std::unique_ptr<PXR_NS::UsdEditContext> mFrameEditContext;
    std::unique_ptr<PXR_NS::SdfChangeBlock> mFrameChangeBlock;
};

// On-demand down-cast from the backend-neutral sink to the concrete USD sink
// (ADR-0005). Returns null when the active backend is not USD, so consumers of
// the USD-specific authoring API (writeUCharArrayAttribute,
// setUpdateToUsdUsingXformCommonAPI, ...) can probe and degrade gracefully.
inline UsdPhysicsDataWrite* asUsdDataWrite(omni::physics::parse::IPhysicsDataWrite* write)
{
    return dynamic_cast<UsdPhysicsDataWrite*>(write);
}

inline const UsdPhysicsDataWrite* asUsdDataWrite(const omni::physics::parse::IPhysicsDataWrite* write)
{
    return dynamic_cast<const UsdPhysicsDataWrite*>(write);
}

} // namespace omni::physics::usd
