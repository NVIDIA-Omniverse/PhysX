// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-WRITE-CORE-001
 * @covers AC-3 AC-4 AC-5 AC-8
 *
 * @implements REQ-WRITE-TRANSFORM-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-WRITE-DATA-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-WRITE-ARRAY-001
 * @covers AC-3 AC-4 AC-5
 */
#pragma once

#include <pxr/usd/usdGeom/xformCache.h>

#include <omni/physics/parse/IPhysicsDataWrite.h>

#include <carb/tasking/TaskingUtils.h>

#include <memory>
#include <unordered_map>

namespace omni::physics::usd
{

// Forward declaration: we hold a raw pointer to the UsdSource owned by the
// AttachedStage (same lifetime), so this header doesn't pull in its header.
class UsdSource;

// USD-backed implementation of the source-agnostic physics output sink. Owned
// by AttachedStage alongside the UsdSource it resolves keys through (rebuilt
// together in AttachedStage::rebuildUsdSource), so it never outlives the stage.
//
// USD-backend-only ObjectKey -> SdfPath (via the UsdSource intern table)
// -> UsdPrim resolution. Writes are read-only
// w.r.t. the intern table (pathFor only, never keyFor), so they are safe on
// the lock-free side of the replicator concurrency contract.
class UsdPhysicsDataWrite final : public omni::physics::parse::IPhysicsDataWrite
{
public:
    UsdPhysicsDataWrite(PXR_NS::UsdStageWeakPtr stage, const omni::physics::usd::UsdSource* source);
    ~UsdPhysicsDataWrite() override;

    // USD-backend-specific authoring mode (not part of IPhysicsDataWrite): when
    // true, prepareTransformWrite leaves XformCommonAPI-compatible prims in
    // XformCommonAPI form and writeTransforms authors via SetXformVectors. The
    // consumer pushes the live `updateToUsdUsingXformCommonAPI` setting here
    // before preparing an object, so the sink need not reach into OmniPhysX.
    void setUpdateToUsdUsingXformCommonAPI(bool value)
    {
        mUpdateUsingXformCommonAPI = value;
    }

    void beginWrite() override;
    void endWrite() override;

    void prepareTransformWrite(const omni::physics::parse::ObjectKey* keys,
                               size_t count,
                               bool* outEligible) override;
    void releaseTransformWrite(const omni::physics::parse::ObjectKey* keys, size_t count) override;

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

    // Mirror of the consumer's `updateToUsdUsingXformCommonAPI` setting, pushed
    // via setUpdateToUsdUsingXformCommonAPI before prepareTransformWrite. Read
    // only at prepare time (cached per object in TransformWriteState).
    bool mUpdateUsingXformCommonAPI = false;

    // Resolves parent-frame world transforms for the world->local conversion.
    // Reset at each beginWrite() so a moving parent is re-read each frame.
    PXR_NS::UsdGeomXformCache mXformCache;

    std::unordered_map<omni::physics::parse::ObjectKey, TransformWriteState, omni::physics::parse::ObjectKey::Hash>
        mTransformState;

    // Guards mTransformState writes only. prepareTransformWrite/releaseTransformWrite
    // run at object setup, which on the replicator clone path happens across
    // parallel carb-tasking fibers (hence the carb fiber-aware mutex, not
    // std::mutex). The per-frame writeTransforms reads the map lock-free: it is
    // temporally disjoint from setup (no objects are prepared mid-update).
    carb::tasking::MutexWrapper mTransformStateMutex;

    // Open between beginWrite()/endWrite() to batch authoring into one change
    // notice (and one undo step) the way the per-frame write loop expects.
    std::unique_ptr<PXR_NS::SdfChangeBlock> mChangeBlock;
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
