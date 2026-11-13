// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-COVERAGE-001
 * @covers AC-4c, AC-4d
 *
 * @implements REQ-READ-SKIN-001
 * @covers AC-2
 *
 * The read's own device view over a scene's point sets: a deformable body's simulation mesh,
 * a particle set, or a deformable's skinned render mesh. Serves AC-4c (deformables), AC-4d
 * (particles) and REQ-READ-SKIN-001 AC-2 (skin meshes, a PxVec3 source).
 *
 * The LAYOUT contract lives here: columnFloats() and setOffsetFloats() define the compact addressing
 * a caller allocates and indexes against. Device residency and the single fused pass are behaviour,
 * so REQ-READ-DEVICE-001 is claimed by the .cpp that implements them, not here.
 */
#pragma once

#include "tensors/PointSetReframe.h"
#include "tensors/gpu/OvStageDeviceArgs.h"

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

// How a view's source buffers are laid out. One layout covers the whole view, since every set a view
// covers comes from the same kind of buffer.
enum class PointSetSourceLayout : uint32_t
{
    eVec4, // PhysX sim-mesh and particle buffers: xyz plus inverse mass, which the gather drops
    eVec3, // the runtime's skinned-vertex buffer of a deformable skin mesh: packed xyz, positions only
};

// What one POINT SET contributes to a read: a deformable body's simulation mesh, a particle set, or one
// skin mesh of a deformable. The caller resolves the device buffers because every kind names them
// differently, from getSimPositionInvMassBufferD to a slice of InternalDeformableBody::mAllSkinnedVerticesD,
// and that indifference lets one view serve all of them. The view's PointSetSourceLayout types the
// pointers. `velocities` may be null where the source has none, as for a skin mesh, and that column then
// gathers nothing for the set.
//
// Kept out of CommonTypes.h beside the other *Entry structs: every entry there carries a
// PXR_NS::SdfPath, and this header's whole value is not needing pxr.
struct PointSetReadEntry
{
    const void* positions = nullptr;
    const void* velocities = nullptr;
    ::physx::PxU32 numPoints = 0;
};

// The ovstage output read's own device view over a scene's point sets.
//
// Named for the SHAPE rather than the object type, because three types have it: a deformable body's
// simulation mesh, a particle set and a deformable's skin mesh are all "N prims, each a ragged run of
// points, one column reframed per prim and one not". The first two read PxVec4 buffers, and the skin
// mesh reads the PxVec3 buffer the runtime's skinning writes.
//
// NOT GpuVolumeDeformableBodyView: that view is the tensor binding's, and constructing one sets
// PxDeformableVolumeFlag::ePARTIALLY_KINEMATIC on every body it covers -- a solver behaviour change a
// caller asking for `points` must not trigger. It also demands USD-derived element indices and rest
// positions that a read never looks at.
//
// So this holds only what is read-shaped: per point set a pair of device source pointers, a point
// count, and where that set's values start in the destination column. It owns no engine data, and
// `src` points straight into PhysX's buffers or the runtime's skinned-vertex buffer.
//
// Unlike the instancer view it is NOT a child of GpuSimulationView: the sources are engine buffers that
// outlive any view and the device scratch is its own, so a rebuilt simulation view cannot dangle it
// and the read's cache has one invalidation condition (the object-lifetime epoch) rather than two.
class GpuPointSetReadView
{
public:
    enum class Column
    {
        ePoints,     // reframed on the device: sim-mesh-local for a deformable, prim-local for particles,
                     // skin-mesh-local for a skin mesh
        eVelocities, // world frame, no reframe -- matching what the host path published
    };

    GpuPointSetReadView(::physx::PxCudaContextManager* ctxMgr,
                        const std::vector<PointSetReadEntry>& entries,
                        PointSetSourceLayout layout);
    ~GpuPointSetReadView();

    // This view owns two cudaMalloc'd pointers and its reads' events, and frees them in its destructor, so a copy
    // would double-free.
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

    // Fill `dstDev` -- one device column, columnFloats() long -- in a single launch on `args.stream` (ADR-0055),
    // reframing points through the getCount() transforms at `transformsDev`: world-to-sim-mesh for a deformable,
    // world-to-prim-local for a particle set. The read resolves them every time and holds them in its own
    // buffer, since moving the prim changes them while creating and retiring nothing. Outside a capture the view
    // then records that the stream reads its records, which its destruction waits for.
    bool readColumnOvStage(Column column,
                           void* dstDev,
                           const PointSetTransform* transformsDev,
                           const OvStageDeviceReadArgs& args) const;

private:
    ::physx::PxCudaContextManager* mCtxMgr = nullptr;
    PointSetSourceLayout mLayout = PointSetSourceLayout::eVec4;
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


    // One event per stream-ordered read still possibly running, recorded after its launch on a stream nothing
    // else orders before this view frees its records; reads on several streams can overlap. Completed ones are
    // kept for reuse.
    mutable std::vector<uintptr_t> mStreamReads;
    mutable std::vector<uintptr_t> mIdleStreamReads;
};

} // namespace tensors
} // namespace physx
} // namespace omni
