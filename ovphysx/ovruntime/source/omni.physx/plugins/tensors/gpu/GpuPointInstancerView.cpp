// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-INSTANCER-001
 * @covers AC-1, AC-5
 *
 * @implements REQ-READ-CORE-001
 * @covers AC-8
 *
 * @implements REQ-INPUT-DEVICE-001
 * @covers AC-1 AC-2 AC-14
 *
 * @implements REQ-INPUT-CORE-001
 * @covers AC-10
 */

// clang-format off
// clang-format on

#include "tensors/gpu/GpuPointInstancerView.h"
#include "tensors/OvStageRowsVersion.h"
#include "tensors/gpu/CudaKernels.h"
#include "tensors/gpu/GpuRigidBodyView.h"
#include "tensors/gpu/GpuSimulationView.h"

#include <common/foundation/MatrixTools.h> // affineInverse -- these transforms carry scale
#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>

#include <cstring>

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

GpuPointInstancerView::GpuPointInstancerView(GpuSimulationView* const sim,
                                             const std::vector<PointInstancerEntry>& entries)
    : BasePointInstancerView(sim, entries)
{
    if (!sim)
        return;
    mGpuSimData = sim->getGpuSimulationData();

    // Every instance resolves to a row of the scene's superset rigid view, which is where its pose
    // is read from. An unresolved one is not patched over: the caller is told to rebuild, because a
    // missing row means this view no longer describes the scene.
    const std::unordered_map<const PxRigidBody*, PxU32>* rowMap = nullptr;
    mRigidView = sim->supersetRigidView(&rowMap);
    if (!mRigidView || !rowMap)
    {
        mUsable = false;
        return;
    }

    std::vector<GpuPointInstancerRecord> records;
    for (const PointInstancerEntry& e : mEntries)
        records.reserve(records.size() + e.instances.size());

    // Per-instancer [begin, count) into `records`, which the loop below fills GROUPED by instancer.
    // That grouping is what lets the WRITE address one instancer with a launch over its own range,
    // instead of filtering every instance and uploading a per-instance slot map on every commit.
    mRanges.assign(mEntries.size(), InstancerRange{});
    for (size_t i = 0; i < mEntries.size(); ++i)
    {
        const PointInstancerEntry& e = mEntries[i];
        mRanges[i].begin = PxU32(records.size());
        for (const PointInstance& inst : e.instances)
        {
            if (!inst.body || inst.index >= e.arrayLength)
                continue;
            std::unordered_map<const PxRigidBody*, PxU32>::const_iterator it = rowMap->find(inst.body);
            if (it == rowMap->end())
            {
                mUsable = false;
                return;
            }
            GpuPointInstancerRecord rec;
            rec.rbRow = it->second;
            rec.instancerIdx = PxU32(i);
            rec.slot = inst.index;
            rec.protoInverse = inst.protoInverse;
            // Both directions stored, so the write does not invert in a kernel. See the record.
            rec.proto = omni::physx::affineInverse(inst.protoInverse);
            records.push_back(rec);
        }
        // Holes were already dropped by the guard above, so a range covers only LIVE instances --
        // which is also why its count is the packed write's output count, known on the host.
        mRanges[i].count = PxU32(records.size()) - mRanges[i].begin;
    }

    mNumInstances = PxU32(records.size());
    if (mNumInstances == 0)
        return;

    // The record rows the write's packed index build reads, in range order, which is the order the kernel writes
    // its packed slots in; with one token per range, minted from the shared ovstage row-cache counter. Distinct per
    // range because ranges can share an instance count, which alone would false-hit in the superset cache; stable
    // for this view's lifetime so repeated writes reuse the device copy. Not derived from `this`: a freed view
    // re-created at the same address would alias its predecessor's slot and scatter into the wrong bodies.
    mWriteRows.reserve(mNumInstances);
    for (const GpuPointInstancerRecord& rec : records)
        mWriteRows.push_back(rec.rbRow);
    mWriteRowsTokens.resize(mRanges.size());
    for (uint64_t& token : mWriteRowsTokens)
        token = nextOvStageRowsVersion();

    // An allocation failure marks the view unusable rather than empty. Empty is a legitimate state
    // that reads succeed against; this is not one.
    PhysxCudaContextGuard ctxGuard(mGpuSimData ? mGpuSimData->mCudaContextManager : nullptr);
    if (!prepareDeviceData((void**)&mRecordsDev, records.data(), records.size() * sizeof(GpuPointInstancerRecord),
                           "mRecordsDev"))
    {
        mUsable = false;
        return;
    }
    if (!prepareDeviceData((void**)&mWorldForwardsDev, nullptr, mEntries.size() * sizeof(InstancerAffine),
                           "mWorldForwardsDev"))
    {
        // mUsable, like every sibling allocation here: a view that failed to allocate but still
        // looked usable would answer reads as "empty, success" -- the exact bug isUsable() documents.
        mUsable = false;
        return;
    }
    if (!prepareDeviceData((void**)&mWorldInversesDev, nullptr, mEntries.size() * sizeof(InstancerAffine),
                           "mWorldInversesDev"))
    {
        mUsable = false;
    }
}

GpuPointInstancerView::~GpuPointInstancerView()
{
    PhysxCudaContextGuard ctxGuard(mGpuSimData ? mGpuSimData->mCudaContextManager : nullptr);
    // Drain before freeing: a read or write over this view's records and frames returns without synchronizing, so a
    // view destroyed in the same frame as its last use can free what a kernel is still reading. cudaFree has
    // historically synchronized implicitly; that is not a guarantee to rely on. Work on the null stream is drained;
    // a stream-ordered read or commit reads them on its caller's stream instead, which is waited for as well.
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    if (mGpuSimData)
        mGpuSimData->waitForTableReaders();

    CHECK_CUDA(cudaFree(mRecordsDev));
    CHECK_CUDA(cudaFree(mWorldInversesDev));
    CHECK_CUDA(cudaFree(mWorldForwardsDev)); // allocated beside the inverses, so freed beside them
}

bool GpuPointInstancerView::setWorldInverses(const ::physx::PxMat44d* const inverses, const PxU32 numInstancers)
{
    if (!BasePointInstancerView::setWorldInverses(inverses, numInstancers))
        return false;
    // Only a real move dirties the upload: the reader re-resolves the inverses on every read whether
    // or not any instancer moved, so dirtying on the call itself would cost a blocking copy per read.
    if (mWorldInversesChanged || mWorldInversesHost.size() != numInstancers)
    {
        mWorldInversesHost.assign(inverses, inverses + numInstancers);
        mWorldInversesDirty = true;
    }
    // Uploaded lazily, at the next column read: doing it here would need a CUDA context guard on a
    // call that may be nowhere near one.
    return true;
}

bool GpuPointInstancerView::uploadWorldInverses() const
{
    if (!mWorldInversesDirty)
        return true;
    if (!mWorldInversesDev || mEntries.empty())
        return false;

    // Seeds the staging array for a read that precedes any setWorldInverses call: the constructor
    // already filled `mEntries` from the enumeration, so those are the right values.
    if (mWorldInversesHost.size() != mEntries.size())
    {
        mWorldInversesHost.resize(mEntries.size());
        for (size_t i = 0; i < mEntries.size(); ++i)
            mWorldInversesHost[i] = mEntries[i].worldInverse;
    }

    // The FORWARD instancer transforms too, for the write (ADR-0012). Derived from the staging array
    // above rather than from mEntries, so a refresh through setWorldInverses is reflected here as
    // well -- the two go stale together because both come from the same worldInverse.
    std::vector<InstancerAffine> hostFwd(mWorldInversesHost.size());
    for (size_t i = 0; i < mWorldInversesHost.size(); ++i)
    {
        hostFwd[i] = omni::physx::affineInverse(mWorldInversesHost[i]);
    }

    if (!mGpuSimData || !mGpuSimData->rewriteTable(mWorldInversesDev, mWorldInversesHost.data(),
                                                    mWorldInversesHost.size() * sizeof(InstancerAffine)))
    {
        return false;
    }
    if (mWorldForwardsDev &&
        !mGpuSimData->rewriteTable(mWorldForwardsDev, hostFwd.data(), hostFwd.size() * sizeof(InstancerAffine)))
    {
        return false;
    }
    mWorldInversesDirty = false;
    return true;
}

bool GpuPointInstancerView::refreshDisabledRowsOvStage() const
{
    if (!mRigidView)
        return false;
    // Instanced bodies are rows of the sibling superset; that view probes its own entries.
    return mRigidView->refreshDisabledRowsOvStage();
}

namespace
{
GpuRigidBodyView::OvStageRigidRead rigidReadOf(const BasePointInstancerView::InstancerColumn column)
{
    using Column = BasePointInstancerView::InstancerColumn;
    using Read = GpuRigidBodyView::OvStageRigidRead;
    switch (column)
    {
    case Column::eLocalPosition:
        return Read::ePosition;
    case Column::eLocalOrientation:
        return Read::eOrientation;
    case Column::eLinearVelocity:
        return Read::eLinearVelocity;
    case Column::eAngularVelocity:
        return Read::eAngularVelocity;
    case Column::eLinearAcceleration:
        return Read::eLinearAcceleration;
    case Column::eAngularAcceleration:
        return Read::eAngularAcceleration;
    }
    return Read::eNone;
}
} // namespace

bool GpuPointInstancerView::planOvStageRead(const InstancerColumn* columns,
                                            const PxU32* const* offsets,
                                            const PxU32 columnCount,
                                            const bool capturing,
                                            std::vector<uint8_t>& head,
                                            size_t& scratchBytes) const
{
    head.clear();
    scratchBytes = 0;
    if (!check() || !mRigidView || !mUsable || !columns || !offsets || columnCount == 0)
        return false;
    // A view with no live instance gathers nothing (readOvStageColumns), so neither its frames nor the GPU index map
    // has to be current; it allocated no frames to upload. Only the local pose columns read the frames: velocities
    // are published in world space.
    bool wantsFrames = false;
    for (PxU32 c = 0; c < columnCount; ++c)
        wantsFrames = wantsFrames || columns[c] == InstancerColumn::eLocalPosition ||
                      columns[c] == InstancerColumn::eLocalOrientation;
    if (mNumInstances != 0 && capturing)
    {
        if ((wantsFrames && mWorldInversesDirty) || mRigidView->rdIndicesRefreshPending())
        {
            CARB_LOG_ERROR("ovstage instancer read: an instancer moved, or an instance was enabled or disabled, "
                           "since the query's last read outside the capture, and a captured read cannot upload "
                           "the refreshed frames or GPU index map");
            return false;
        }
    }
    else if (mNumInstances != 0)
    {
        PhysxCudaContextGuard ctxGuard(mGpuSimData ? mGpuSimData->mCudaContextManager : nullptr);
        if ((wantsFrames && !uploadWorldInverses()) || !mRigidView->refreshDisabledRowsOvStage())
            return false;
    }

    const PxU32 numInstancers = PxU32(mEntries.size());
    OvStageScratch layout(nullptr);
    std::vector<size_t> at(columnCount);
    for (PxU32 c = 0; c < columnCount; ++c)
    {
        at[c] = layout.used();
        layout.take<PxU32>(numInstancers);
    }
    head.assign(layout.used(), 0);
    for (PxU32 c = 0; c < columnCount; ++c)
        std::memcpy(head.data() + at[c], offsets[c], size_t(numInstancers) * sizeof(PxU32));
    std::vector<GpuRigidBodyView::OvStageInstancerReadColumn> reads(columnCount);
    for (PxU32 c = 0; c < columnCount; ++c)
        reads[c].kind = rigidReadOf(columns[c]);
    scratchBytes = layout.used() + mRigidView->ovStageInstancerReadScratchBytes(reads.data(), columnCount);
    return true;
}

bool GpuPointInstancerView::readOvStageColumns(const OvStageDeviceReadArgs& args,
                                               const InstancerColumn* columns,
                                               float* const* dsts,
                                               const PxU32 columnCount) const
{
    if (!check() || !mRigidView || !mUsable || !args.scratch || !columns || !dsts)
        return false;
    if (mNumInstances == 0)
        return true;
    const PxU32 numInstancers = PxU32(mEntries.size());
    OvStageScratch scratch(args.scratch);
    std::vector<GpuRigidBodyView::OvStageInstancerReadColumn> reads(columnCount);
    for (PxU32 c = 0; c < columnCount; ++c)
    {
        reads[c].kind = rigidReadOf(columns[c]);
        reads[c].dst = dsts[c];
        reads[c].offsetsDev = scratch.take<PxU32>(numInstancers);
    }
    // The rigid view's copies follow the offsets in the same scratch.
    OvStageDeviceReadArgs copies = args;
    copies.scratch = static_cast<char*>(args.scratch) + scratch.used();
    copies.scratchBytes = args.scratchBytes > scratch.used() ? args.scratchBytes - scratch.used() : 0;
    return mRigidView->readOvStageInstancerColumns(copies, reads.data(), columnCount, mRecordsDev, mWorldInversesDev,
                                                   mNumInstances);
}

bool GpuPointInstancerView::prepareInstancerWriteOvStage(const PxU32 instancerIndex,
                                                         const PxU32*& outRows,
                                                         PxU32& outCount)
{
    outRows = nullptr;
    outCount = 0;
    if (!check() || !mUsable || !mRigidView || !mRecordsDev)
        return false;
    if (instancerIndex >= mEntries.size() || instancerIndex >= mRanges.size())
        return false;
    PhysxCudaContextGuard ctxGuard(mGpuSimData ? mGpuSimData->mCudaContextManager : nullptr);
    if (!uploadWorldInverses())
        return false;
    const InstancerRange& range = mRanges[instancerIndex];
    outRows = mWriteRows.data() + range.begin;
    outCount = range.count;
    return true;
}

bool GpuPointInstancerView::setInstancerColumnOvStage(const InstancerColumn column,
                                                      const ::physx::PxU32 instancerIndex,
                                                      const void* const src,
                                                      const ::physx::PxU32 arrayLength,
                                                      const uint8_t*,
                                                      const OvStageDeviceWriteArgs* const sessionDeviceArgs)
{
    if (!check() || !mUsable || !mRigidView || !mRecordsDev)
        return false;
    if (!refreshDisabledRowsOvStage())
        return false;
    // Disabling an individual instancer INSTANCE is not a supported operation -- disableSimulation is
    // not an instancer-writable column and there is no per-instance tensor route -- so a disabled
    // instance is only reachable through the undefined raw-pointer path. Records are built once and
    // hold every live instance, so such an instance keeps its sentinel row here; the DirectGPU write
    // below refuses the whole instancer group on that sentinel (shared ovStageWriteBuffers) rather
    // than resolving it to the wrong instance (REQ-INPUT-CORE-001 AC-10). The read, by contrast, keeps
    // the instancer available and zero-fills the slot -- it has no wrong-write hazard to guard.
    if (!src)
    {
        CARB_LOG_ERROR("%s: null source", __FUNCTION__);
        return false;
    }
    if (instancerIndex >= mEntries.size() || instancerIndex >= mRanges.size())
    {
        CARB_LOG_ERROR("%s: instancer %u of %zu", __FUNCTION__, instancerIndex, mEntries.size());
        return false;
    }
    // The caller sized its column from the array length this view reported. Disagreement means the
    // view was rebuilt underneath the group, and writing anyway would place instances at indices the
    // caller never meant.
    if (arrayLength != mEntries[instancerIndex].arrayLength)
    {
        CARB_LOG_ERROR("%s: column carries %u slots, instancer %u now has %u", __FUNCTION__, arrayLength,
                       instancerIndex, mEntries[instancerIndex].arrayLength);
        return false;
    }

    const InstancerRange& range = mRanges[instancerIndex];
    if (range.count == 0)
        return true; // every instance of this instancer is a hole -- nothing to write, not a failure

    const OvStageDeviceWriteArgs deviceArgs = sessionDeviceArgs ? *sessionDeviceArgs : OvStageDeviceWriteArgs();
    PhysxCudaContextGuard ctxGuard(mGpuSimData ? mGpuSimData->mCudaContextManager : nullptr);
    // A session prepared both at open; anything still to upload here means the instancer changed since.
    if (deviceArgs.capturing && mWorldInversesDirty)
    {
        CARB_LOG_ERROR("%s: instancer %u changed since this write was opened, and a captured commit cannot upload "
                       "its transforms. Reopen the write and capture again.",
                       __FUNCTION__, instancerIndex);
        return false;
    }
    const PxU32* const rbRows = mWriteRows.data() + range.begin;
    const uint64_t rowsToken = mWriteRowsTokens[instancerIndex];

    const float* const srcDev = static_cast<const float*>(src);
    switch (column)
    {
    case InstancerColumn::eLocalPosition:
    case InstancerColumn::eLocalOrientation:
    {
        if (!uploadWorldInverses())
            return false;
        const bool wantOrientation = (column == InstancerColumn::eLocalOrientation);
        return mRigidView->setInstancerPoseColumnOvStage(wantOrientation, srcDev, mRecordsDev + range.begin,
                                                         mWorldInversesDev, mWorldForwardsDev, rbRows,
                                                         range.count, instancerIndex, rowsToken, deviceArgs);
    }
    case InstancerColumn::eLinearVelocity:
    case InstancerColumn::eAngularVelocity:
    {
        const bool wantAngular = (column == InstancerColumn::eAngularVelocity);
        return mRigidView->setInstancerVelocityColumnOvStage(wantAngular, srcDev, mRecordsDev + range.begin,
                                                             rbRows, range.count, rowsToken, deviceArgs);
    }
    }
    return false;
}

} // namespace tensors
} // namespace physx
} // namespace omni
