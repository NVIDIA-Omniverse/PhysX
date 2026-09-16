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
 * @covers AC-1 AC-2
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
    // Drain before freeing: an ovstage gather is launched on the null stream and returns without
    // synchronizing, so a view destroyed in the same frame as its last read can free records the
    // kernel is still reading. cudaFree has historically synchronized implicitly; that is not a
    // guarantee to rely on. Same reason GpuArticulationView drains before its selection buffer.
    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    CHECK_CUDA(cudaFree(mRecordsDev));
    CHECK_CUDA(cudaFree(mWorldInversesDev));
    CHECK_CUDA(cudaFree(mWorldForwardsDev)); // allocated beside the inverses, so freed beside them
    CHECK_CUDA(cudaFree(mOffsetsDev));
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

    if (!CHECK_CUDA(cudaMemcpy(mWorldInversesDev, mWorldInversesHost.data(),
                               mWorldInversesHost.size() * sizeof(InstancerAffine), cudaMemcpyHostToDevice)))
    {
        return false;
    }
    if (mWorldForwardsDev &&
        !CHECK_CUDA(cudaMemcpy(mWorldForwardsDev, hostFwd.data(), hostFwd.size() * sizeof(InstancerAffine),
                               cudaMemcpyHostToDevice)))
    {
        return false;
    }
    mWorldInversesDirty = false;
    return true;
}

bool GpuPointInstancerView::uploadOffsets(const PxU32* const offsets, const PxU32 numInstancers) const
{
    if (numInstancers > mOffsetsCapacity)
    {
        CHECK_CUDA(cudaFree(mOffsetsDev));
        mOffsetsDev = nullptr;
        mOffsetsCapacity = 0;
        mOffsetsHost.clear(); // the device array is gone; nothing is known to be resident
        if (!prepareDeviceData((void**)&mOffsetsDev, nullptr, numInstancers * sizeof(PxU32), "mOffsetsDev"))
            return false;
        mOffsetsCapacity = numInstancers;
    }
    else if (mOffsetsHost.size() == numInstancers &&
             std::memcmp(mOffsetsHost.data(), offsets, numInstancers * sizeof(PxU32)) == 0)
    {
        return true; // this exact array is already on the device
    }
    if (!CHECK_CUDA(
            cudaMemcpy(mOffsetsDev, offsets, numInstancers * sizeof(PxU32), cudaMemcpyHostToDevice)))
    {
        return false;
    }
    mOffsetsHost.assign(offsets, offsets + numInstancers);
    return true;
}

bool GpuPointInstancerView::refreshDisabledRowsOvStage() const
{
    if (!mRigidView)
        return false;
    // Instanced bodies are rows of the sibling superset; that view probes its own entries.
    return mRigidView->refreshDisabledRowsOvStage();
}

bool GpuPointInstancerView::getInstancerColumnsOvStage(const InstancerColumn column,
                                                       void* const dst,
                                                       const PxU32* const offsets,
                                                       const PxU32 numInstancers) const
{
    // Same split as the CPU view: an unusable view or a missing sibling is a decline the reader
    // handles, the other two are caller errors that must not look like one.
    if (!check() || !mRigidView || !mUsable)
        return false;
    if (!dst || !offsets)
    {
        CARB_LOG_ERROR("%s: null destination or offsets", __FUNCTION__);
        return false;
    }
    if (numInstancers != mEntries.size())
    {
        CARB_LOG_ERROR("%s: called with %u instancers, view holds %zu", __FUNCTION__, numInstancers,
                       mEntries.size());
        return false;
    }
    if (mNumInstances == 0)
        return true;

    PhysxCudaContextGuard ctxGuard(mGpuSimData ? mGpuSimData->mCudaContextManager : nullptr);
    if (!uploadOffsets(offsets, numInstancers))
        return false;

    float* const dstDev = static_cast<float*>(dst);
    switch (column)
    {
    case InstancerColumn::eLocalPosition:
    case InstancerColumn::eLocalOrientation:
    {
        // Only the pose columns need the instancer frame; the velocity ones are published in world
        // space, so they never pay for this upload.
        if (!uploadWorldInverses())
            return false;
        const bool wantOrientation = (column == InstancerColumn::eLocalOrientation);
        return mRigidView->getInstancerPoseColumnOvStage(wantOrientation, dstDev, mRecordsDev, mWorldInversesDev,
                                                         mOffsetsDev, mNumInstances);
    }
    case InstancerColumn::eLinearVelocity:
    case InstancerColumn::eAngularVelocity:
    case InstancerColumn::eLinearAcceleration:
    case InstancerColumn::eAngularAcceleration:
    {
        // All four are world frame, so none pays for the instancer-frame upload above; acceleration
        // differs from velocity only in the DirectGPU read type it asks for.
        const bool wantAngular = (column == InstancerColumn::eAngularVelocity ||
                                  column == InstancerColumn::eAngularAcceleration);
        const bool wantAcceleration = (column == InstancerColumn::eLinearAcceleration ||
                                       column == InstancerColumn::eAngularAcceleration);
        return mRigidView->getInstancerVelocityColumnOvStage(wantAngular, wantAcceleration, dstDev,
                                                             mRecordsDev, mOffsetsDev, mNumInstances);
    }
    }
    return false;
}


bool GpuPointInstancerView::setInstancerColumnOvStage(const InstancerColumn column,
                                                      const ::physx::PxU32 instancerIndex,
                                                      const void* const src,
                                                      const ::physx::PxU32 arrayLength)
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

    PhysxCudaContextGuard ctxGuard(mGpuSimData ? mGpuSimData->mCudaContextManager : nullptr);

    // The record-row list for THIS instancer's live instances, in range order, which is the order the
    // kernel writes its packed slots in. Held per view and rebuilt only when the records are, since
    // the ranges and rbRows change together with them.
    if (mWriteRows.empty())
    {
        std::vector<GpuPointInstancerRecord> hostRecords(mNumInstances);
        if (!CHECK_CUDA(cudaMemcpy(hostRecords.data(), mRecordsDev,
                                   size_t(mNumInstances) * sizeof(GpuPointInstancerRecord),
                                   cudaMemcpyDeviceToHost)))
        {
            return false;
        }
        mWriteRows.resize(mNumInstances);
        for (PxU32 i = 0; i < mNumInstances; ++i)
            mWriteRows[i] = hostRecords[i].rbRow;
        // A per-range token minted from the shared ovstage row-cache counter, once, with the list it
        // identifies. Distinct per range because ranges can share an instance count, which alone would
        // false-hit in the superset cache; stable for this view's lifetime so repeated writes reuse the
        // device copy. Not derived from `this`: a freed view re-created at the same address would alias
        // its predecessor's slot and scatter into the wrong bodies.
        mWriteRowsTokens.resize(mRanges.size());
        for (uint64_t& token : mWriteRowsTokens)
            token = nextOvStageRowsVersion();
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
                                                         range.count, instancerIndex, rowsToken);
    }
    case InstancerColumn::eLinearVelocity:
    case InstancerColumn::eAngularVelocity:
    {
        const bool wantAngular = (column == InstancerColumn::eAngularVelocity);
        return mRigidView->setInstancerVelocityColumnOvStage(wantAngular, srcDev, mRecordsDev + range.begin,
                                                             rbRows, range.count, rowsToken);
    }
    }
    return false;
}

} // namespace tensors
} // namespace physx
} // namespace omni
