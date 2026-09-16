// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-COVERAGE-001
 * @covers AC-4c, AC-4d
 *
 * The read's own device view over a scene's point sets -- a deformable body's simulation mesh
 * or a particle set. Serves both AC-4c (deformables) and AC-4d (particles).
 *
 * The LAYOUT contract lives here: columnFloats() and setOffsetFloats() define the compact addressing
 * a caller allocates and indexes against. Device residency and the single fused pass are behaviour,
 * so REQ-READ-DEVICE-001 is claimed by the .cpp that implements them, not here.
 */
#pragma once

#include "tensors/PointSetReframe.h"

#include <foundation/PxVec4.h>
#include <cstdint>
#include <vector>

namespace physx
{
class PxCudaContextManager;
}

namespace omni
{
namespace physx
{
namespace tensors
{

// What one POINT SET contributes to a read -- a deformable body's simulation mesh, or a particle set.
// The caller resolves the two device buffers because every kind names them differently
// (getSimPositionInvMassBufferD on a volume deformable, getPositionInvMassBufferD on a surface one,
// PxParticleBuffer::getPositionInvMasses on a particle set); that indifference is what lets one view
// serve both object types.
//
// Kept out of CommonTypes.h beside the other *Entry structs: every entry there carries a
// PXR_NS::SdfPath, and this header's whole value is not needing pxr.
struct PointSetReadEntry
{
    const ::physx::PxVec4* positions = nullptr;
    const ::physx::PxVec4* velocities = nullptr;
    ::physx::PxU32 numPoints = 0;
};

// The ovstage output read's own device view over a scene's point sets.
//
// Named for the SHAPE rather than the object type, because two types have it: a deformable body's
// simulation mesh and a particle set are both "N prims, each a ragged run of PxVec4, one column
// reframed per prim and one not".
//
// NOT GpuVolumeDeformableBodyView: that view is the tensor binding's, and constructing one sets
// PxDeformableVolumeFlag::ePARTIALLY_KINEMATIC on every body it covers -- a solver behaviour change a
// caller asking for `points` must not trigger. It also demands USD-derived element indices and rest
// positions that a read never looks at.
//
// So this holds only what is read-shaped: per point set a pair of device source pointers, a point
// count, and where that set's values start in the destination column. It owns no engine data --
// `src` points straight into PhysX's own buffers.
//
// Unlike the instancer view it is NOT a child of GpuSimulationView: the sources are PhysX buffers that
// outlive any view and the device scratch is its own, so a rebuilt simulation view cannot dangle it
// and the read's cache has one invalidation condition (the object-lifetime epoch) rather than two.
class GpuPointSetReadView
{
public:
    enum class Column
    {
        ePoints,     // reframed on the device: sim-mesh-local for a deformable, prim-local for particles
        eVelocities, // world frame, no reframe -- matching what the host path published
    };

    GpuPointSetReadView(::physx::PxCudaContextManager* ctxMgr, const std::vector<PointSetReadEntry>& entries);
    ~GpuPointSetReadView();

    // This view owns three cudaMalloc'd pointers and frees them in its destructor, so a copy would
    // double-free.
    GpuPointSetReadView(const GpuPointSetReadView&) = delete;
    GpuPointSetReadView& operator=(const GpuPointSetReadView&) = delete;

    // False when the view cannot serve a read at all -- an empty point set, or a device allocation
    // that failed while building. One flag for both, so a failed allocation cannot merely LOOK empty
    // and have every column read answer "nothing to do, success" over zero-filled groups.
    bool isUsable() const
    {
        return mUsable;
    }

    ::physx::PxU32 getCount() const
    {
        return mNumSets;
    }

    // Total floats one column occupies, which is the destination the caller must allocate: exactly
    // sum(numPoints * 3). The layout is COMPACT even though the launch that fills it is rectangular
    // over (set, point) -- every thread addresses its destination through the record's own
    // `dstOffsetFloats`, so padding to the widest set would cost memory and buy nothing.
    ::physx::PxU32 columnFloats() const
    {
        return mColumnFloats;
    }

    // Where set `i`'s values start in that column, in floats. The caller needs these to point one
    // array-group tensor per prim at its own slice.
    //
    // `i` must be < getCount(); this and pointCountFor() below are unchecked on purpose, since both
    // feed pointer arithmetic in the caller and a defaulted answer would silently aim a tensor at set
    // 0's data. The contract is on the INDEX, not a loop shape: OvxPhysicsRead reaches here through
    // `rows[r]`, which holds only because `rows` carries this view's own row indices.
    ::physx::PxU32 setOffsetFloats(::physx::PxU32 i) const
    {
        return mOffsets[i];
    }

    // How many points set `i` has -- the length of its row, and the kernel's per-record bound.
    // Same index contract as setOffsetFloats() above.
    ::physx::PxU32 pointCountFor(::physx::PxU32 i) const
    {
        return mPointCounts[i];
    }

    // The per-prim reframe matrices for the next points read: world-to-sim-mesh for a deformable,
    // world-to-prim-local for a particle set. Resolved by the caller every read and never cached here:
    // moving the prim changes this while creating and retiring nothing, so the object-lifetime epoch
    // that guards everything else cannot see it.
    //
    // Synchronous, and now only the FALLBACK: the read normally uploads these through
    // setReframeMatricesAsync from pinned staging, and reaches this form only when a pinned buffer
    // cannot be acquired -- trading the host block for an upload that is still guaranteed to land.
    bool setReframeMatrices(const PointSetTransform* transforms, ::physx::PxU32 count);

    // Same upload, issued ASYNCHRONOUSLY on the null stream from a PINNED source -- the read's normal
    // path. The sync form above blocks the host for the ~32 KB copy; this one enqueues it and lets the
    // read's release-time completion wait order it, like every other device copy on the read path. Two
    // caller obligations make it safe: `pinnedTransforms` must be pinned host memory (a pageable source
    // makes cudaMemcpyAsync synchronous again, buying nothing), and it must stay alive until the copy
    // executes -- the read session owns it and frees it only past the same completion event. Device-side
    // ordering is unchanged from the sync form: the copy and the gather that reads mTransformsDev are
    // both on the null stream, so the copy still precedes the gather. Returns the ENQUEUE result.
    bool setReframeMatricesAsync(const PointSetTransform* pinnedTransforms, ::physx::PxU32 count);

    // Fill `dstDev` -- one device column, columnFloats() long -- in a single launch.
    bool getColumnOvStage(Column column, void* dstDev) const;

private:
    ::physx::PxCudaContextManager* mCtxMgr = nullptr;
    bool mUsable = false;

    ::physx::PxU32 mNumSets = 0;
    ::physx::PxU32 mMaxPoints = 0; // launch shape only; see columnFloats()
    ::physx::PxU32 mColumnFloats = 0;
    std::vector<::physx::PxU32> mPointCounts;
    std::vector<::physx::PxU32> mOffsets;

    // One record array per column kind, differing only in which PhysX buffer `src` names. Uploaded
    // once at construction: `src`, `numPoints` and `dstOffsetFloats` are fixed for as long as the
    // membership is, and a membership change is what retires this view.
    void* mPointRecordsDev = nullptr;
    void* mVelocityRecordsDev = nullptr;

    // Rewritten by every points read, so NOT safe for two concurrent reads of the same scene. It holds
    // only because the read API serialises on g_mutex and both the upload and the gather run on the
    // null stream; moving either to a non-default stream makes this per-read scratch, not view state.
    PointSetTransform* mTransformsDev = nullptr;
};

} // namespace tensors
} // namespace physx
} // namespace omni
