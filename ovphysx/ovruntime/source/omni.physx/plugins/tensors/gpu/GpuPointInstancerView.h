// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "tensors/CommonTypes.h"
#include "tensors/base/BasePointInstancerView.h"
#include "tensors/gpu/GpuSimulationData.h"

#include <vector>

namespace omni
{
namespace physx
{
namespace tensors
{

class GpuSimulationView;
class GpuRigidBodyView;

// The device instancer view: fills ovstage columns on the GPU, reframing against the pose the engine
// already has there rather than pulling it to the host first. It holds no engine data of its own --
// instanced bodies are rows of the scene's superset rigid view, which issues the bulk read; only the
// instancer-shaped data (per-instance records, per-instancer world inverses, offsets) lives here.
class GpuPointInstancerView : public BasePointInstancerView
{
public:
    GpuPointInstancerView(GpuSimulationView* sim, const std::vector<PointInstancerEntry>& entries);
    ~GpuPointInstancerView() override;

    bool setWorldInverses(const ::physx::PxMat44d* inverses, ::physx::PxU32 numInstancers) override;

    // Refresh the sibling superset's DirectGPU indices after an instance is disabled or re-enabled,
    // including actor-flag changes that did not pass through a tensor view.
    bool refreshDisabledRowsOvStage() const;

    bool getInstancerColumnsOvStage(InstancerColumn column,
                                    void* dst,
                                    const ::physx::PxU32* offsets,
                                    ::physx::PxU32 numInstancers) const override;
    // The device write (ADR-0012). One instancer per call, launched over that instancer's record
    // range, so the packed pair handed to PhysX is exactly its live instances -- count known on the
    // host, no compaction and no host block.
    bool setInstancerColumnOvStage(InstancerColumn column,
                                   ::physx::PxU32 instancerIndex,
                                   const void* src,
                                   ::physx::PxU32 arrayLength) override;

    // False when this view cannot serve reads at all: an instance whose body has no row in the scene's
    // superset rigid view, or a device allocation that failed during construction. The caller must drop
    // it and rebuild -- an unusable view looks empty, and an empty view answers every column read with
    // "nothing to do, success".
    bool isUsable() const
    {
        return mUsable;
    }

private:
    // Push the staged world inverses to the device. Called from the column read rather than from
    // setWorldInverses, so the upload happens inside the CUDA context guard the read already takes,
    // and it is skipped unless an instancer actually moved.
    bool uploadWorldInverses() const;
    // Per-instancer destination starts, in FLOATS -- not "elements", which in this codebase means the
    // dtype.lanes tuple. The array depends only on the instancer lengths and the column width, and five
    // of InstancerColumn's six values are 3 floats wide (only eLocalOrientation is 4), so consecutive
    // reads usually ask for identical offsets; what is already resident is compared, not re-uploaded.
    bool uploadOffsets(const ::physx::PxU32* offsets, ::physx::PxU32 numInstancers) const;

    GpuSimulationDataPtr mGpuSimData;

    // The scene's superset rigid view, resolved once at construction. Not owned: it is a sibling
    // child of the same GpuSimulationView, which tears both down together.
    GpuRigidBodyView* mRigidView = nullptr;
    bool mUsable = true;

    GpuPointInstancerRecord* mRecordsDev = nullptr;
    // The FORWARD instancer transforms, for the write (ADR-0012). Beside mWorldInversesDev and
    // refreshed in the same upload, since both derive from the entries' worldInverse -- keeping them
    // apart would let one go stale while the other did not.
    InstancerAffine* mWorldForwardsDev = nullptr;

    // Where each instancer's records live in the flat array. Built at construction because the
    // record loop groups by instancer anyway; it is what lets a write launch over ONE instancer.
    struct InstancerRange
    {
        ::physx::PxU32 begin = 0;
        ::physx::PxU32 count = 0;
    };
    std::vector<InstancerRange> mRanges;

    // The record rows, host-side, for the write's packed index build. Filled lazily on the first
    // write and never invalidated: the records it mirrors are immutable for this view's lifetime, and
    // a rebuild takes a new view.
    std::vector<::physx::PxU32> mWriteRows;
    // One row-cache token per instancer range, minted from the shared ovstage counter when mWriteRows
    // is built. Stored so repeated writes to the same range reuse the device copy, yet a rebuilt view
    // gets fresh tokens -- unlike an address-derived token, these never alias a freed view reused at
    // the old pointer, which the superset row cache would otherwise serve as this range's rows.
    std::vector<uint64_t> mWriteRowsTokens;
    ::physx::PxU32 mNumInstances = 0;

    // Mutable for the same reason the rigid view's caches are: the ovstage reads are const by the
    // convention that "const" means the VIEW is unchanged, not the device scratch behind it.
    mutable InstancerAffine* mWorldInversesDev = nullptr;
    mutable bool mWorldInversesDirty = true;
    // The inverses PACKED, which is what the device wants: `worldInverse` is a strided field of a
    // larger entry, so `mEntries` is not an array of inverses. Filled in setWorldInverses so the
    // upload reads it directly.
    mutable std::vector<InstancerAffine> mWorldInversesHost;
    mutable ::physx::PxU32* mOffsetsDev = nullptr;
    mutable ::physx::PxU32 mOffsetsCapacity = 0;
    // What is currently on the device, so an unchanged offsets array is not pushed again.
    mutable std::vector<::physx::PxU32> mOffsetsHost;
};

} // namespace tensors
} // namespace physx
} // namespace omni
