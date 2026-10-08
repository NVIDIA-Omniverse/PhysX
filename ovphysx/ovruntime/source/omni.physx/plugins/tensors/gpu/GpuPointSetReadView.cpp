// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-COVERAGE-001
 * @covers AC-4c, AC-4d
 *
 * @implements REQ-READ-DEVICE-001
 * @covers AC-1, AC-2
 */

#include "tensors/gpu/GpuPointSetReadView.h"
#include "tensors/gpu/CudaKernels.h"
#include "tensors/gpu/GpuSimulationData.h"

#include <PxPhysicsAPI.h>
#include <cudamanager/PxCudaContextManager.h>

#include <carb/logging/Log.h>

#include <cuda_runtime_api.h>

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{
namespace
{

// Allocate and fill in one step, so a half-built record array cannot be mistaken for a usable one.
bool uploadRecords(void*& devOut, const std::vector<GpuPointSetReadRecord>& host)
{
    const size_t bytes = host.size() * sizeof(GpuPointSetReadRecord);
    if (!CHECK_CUDA(cudaMalloc(&devOut, bytes)))
    {
        devOut = nullptr;
        return false;
    }
    if (!CHECK_CUDA(cudaMemcpy(devOut, host.data(), bytes, cudaMemcpyHostToDevice)))
    {
        CHECK_CUDA(cudaFree(devOut));
        devOut = nullptr;
        return false;
    }
    return true;
}

} // namespace

GpuPointSetReadView::GpuPointSetReadView(PxCudaContextManager* const ctxMgr,
                                         const std::vector<PointSetReadEntry>& entries)
    : mCtxMgr(ctxMgr)
{
    if (!ctxMgr || entries.empty())
        return;

    PhysxCudaContextGuard ctxGuard(ctxMgr);

    mNumSets = static_cast<PxU32>(entries.size());
    mPointCounts.resize(entries.size());
    for (size_t i = 0; i < entries.size(); ++i)
    {
        mPointCounts[i] = entries[i].numPoints;
        mMaxPoints = PxMax(mMaxPoints, entries[i].numPoints);
    }
    if (mMaxPoints == 0)
        return; // every set is empty; there is no column to serve

    // Compact running offset rather than `i * mMaxPoints * 3`: padding every set out to the widest
    // one would only inflate the allocation and the memset that clears it.
    std::vector<GpuPointSetReadRecord> points(entries.size());
    std::vector<GpuPointSetReadRecord> velocities(entries.size());
    uint64_t offset = 0;
    mOffsets.resize(entries.size());
    for (size_t i = 0; i < entries.size(); ++i)
    {
        mOffsets[i] = static_cast<PxU32>(offset);
        points[i].src = entries[i].positions;
        points[i].numPoints = entries[i].numPoints;
        points[i].dstOffsetFloats = static_cast<PxU32>(offset);
        velocities[i].src = entries[i].velocities;
        velocities[i].numPoints = entries[i].numPoints;
        velocities[i].dstOffsetFloats = static_cast<PxU32>(offset);
        offset += uint64_t(entries[i].numPoints) * 3u;
    }
    // Accumulated in 64 bits and checked once, because `dstOffsetFloats` is a PxU32: a column that
    // does not fit has to refuse the read rather than wrap an offset and scatter one set's values
    // over another's.
    if (offset > 0xFFFFFFFFull)
    {
        CARB_LOG_ERROR("ovphysx read: point-set column needs %llu floats, which overflows a 32-bit offset.",
                       static_cast<unsigned long long>(offset));
        return;
    }
    // Non-zero by construction: the mMaxPoints guard above already returned unless some set has
    // points, so the running sum is at least 3.
    mColumnFloats = static_cast<PxU32>(offset);

    // The same refusal for the LAUNCH extent, which the compact check above does not cover: that one
    // bounds sum(numPoints) * 3, this one numSets * mMaxPoints, and a ragged set separates them --
    // many small sets beside one large one keep the sum small while the product grows. A product of
    // exactly 2^32 wraps to ZERO, giving a zero-block grid that writes nothing and publishes the
    // memset zeros under a successful read.
    //
    // >=, not >: UINT32_MAX itself is representable as a thread count but not as a grid. The launch
    // rounds up by 255 in 32-bit, so exactly UINT32_MAX wraps to 254 and yields the same silent
    // zero-block grid one past the value this guard was written to catch.
    if (uint64_t(mNumSets) * uint64_t(mMaxPoints) >= 0xFFFFFFFFull)
    {
        CARB_LOG_ERROR("ovphysx read: point-set gather needs %llu threads for %u sets of up to %u points, "
                       "which overflows a 32-bit launch extent.",
                       static_cast<unsigned long long>(uint64_t(mNumSets) * uint64_t(mMaxPoints)), mNumSets,
                       mMaxPoints);
        return;
    }

    if (!uploadRecords(mPointRecordsDev, points) || !uploadRecords(mVelocityRecordsDev, velocities))
        return;

    if (!CHECK_CUDA(cudaMalloc(&mTransformsDev, entries.size() * sizeof(PointSetTransform))))
    {
        mTransformsDev = nullptr;
        return;
    }

    mUsable = true;
}

GpuPointSetReadView::~GpuPointSetReadView()
{
    PhysxCudaContextGuard ctxGuard(mCtxMgr);
    // Drain before freeing: an ovstage gather is launched on the null stream and returns without
    // synchronizing, so a view destroyed in the same frame as its last read can free records the
    // kernel is still reading. cudaFree has historically synchronized implicitly; that is not a
    // guarantee to rely on. Same reason GpuArticulationView drains before its selection buffer.
    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    CHECK_CUDA(cudaFree(mPointRecordsDev));
    CHECK_CUDA(cudaFree(mVelocityRecordsDev));
    CHECK_CUDA(cudaFree(mTransformsDev));
}

bool GpuPointSetReadView::setReframeMatrices(const PointSetTransform* const transforms, const PxU32 count)
{
    if (!mUsable || !transforms || count != mNumSets)
        return false;
    // Guarded like the constructor and destructor that bracket it: this copy targets device memory
    // this view owns, and a caller on a thread with a different current context would otherwise copy
    // into the wrong one.
    PhysxCudaContextGuard ctxGuard(mCtxMgr);
    return CHECK_CUDA(cudaMemcpy(mTransformsDev, transforms, size_t(count) * sizeof(PointSetTransform),
                                 cudaMemcpyHostToDevice));
}

bool GpuPointSetReadView::setReframeMatricesAsync(const PointSetTransform* const pinnedTransforms,
                                                  const PxU32 count)
{
    if (!mUsable || !pinnedTransforms || count != mNumSets)
        return false;
    // Guarded like the sync form; the copy targets this view's own device memory. Stream 0, so it is
    // ordered before the gather that reads mTransformsDev without the host blocking on it -- the caller
    // owns the pinned source and keeps it alive until the read's completion event has been waited.
    PhysxCudaContextGuard ctxGuard(mCtxMgr);
    return CHECK_CUDA(cudaMemcpyAsync(mTransformsDev, pinnedTransforms,
                                      size_t(count) * sizeof(PointSetTransform), cudaMemcpyHostToDevice, 0));
}

bool GpuPointSetReadView::getColumnOvStage(const Column column, void* const dstDev) const
{
    if (!mUsable || !dstDev)
        return false;

    const bool reframe = (column == Column::ePoints);
    const void* records = reframe ? mPointRecordsDev : mVelocityRecordsDev;
    return fetchPointSetColumnOvStage(static_cast<float*>(dstDev),
                                      static_cast<const GpuPointSetReadRecord*>(records), mTransformsDev,
                                      mNumSets, mMaxPoints, reframe);
}

} // namespace tensors
} // namespace physx
} // namespace omni
