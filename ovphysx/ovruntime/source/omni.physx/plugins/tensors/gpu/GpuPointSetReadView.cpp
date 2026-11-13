// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-COVERAGE-001
 * @covers AC-4c, AC-4d
 *
 * @implements REQ-READ-DEVICE-001
 * @covers AC-1, AC-2
 *
 * @implements REQ-READ-SKIN-001
 * @covers AC-2
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

// Allocate and fill in one step, so a half-built record array cannot be mistaken for a usable one. A copy from
// pageable memory may return before it reaches the device, and a read on a non-blocking stream does not wait for the
// legacy default stream it runs on, so the upload is waited for.
bool uploadRecords(void*& devOut, const std::vector<GpuPointSetReadRecord>& host)
{
    const size_t bytes = host.size() * sizeof(GpuPointSetReadRecord);
    if (!CHECK_CUDA(cudaMalloc(&devOut, bytes)))
    {
        devOut = nullptr;
        return false;
    }
    if (!CHECK_CUDA(cudaMemcpy(devOut, host.data(), bytes, cudaMemcpyHostToDevice)) ||
        !CHECK_CUDA(cudaStreamSynchronize(nullptr)))
    {
        CHECK_CUDA(cudaFree(devOut));
        devOut = nullptr;
        return false;
    }
    return true;
}

} // namespace

GpuPointSetReadView::GpuPointSetReadView(PxCudaContextManager* const ctxMgr,
                                         const std::vector<PointSetReadEntry>& entries,
                                         const PointSetSourceLayout layout)
    : mCtxMgr(ctxMgr), mLayout(layout)
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

    mUsable = true;
}

GpuPointSetReadView::~GpuPointSetReadView()
{
    PhysxCudaContextGuard ctxGuard(mCtxMgr);
    // Drain before freeing: an ovstage gather launched on the null stream returns without synchronizing, so a
    // view destroyed in the same frame as its last read can free records the kernel is still reading. cudaFree
    // has historically synchronized implicitly; that is not a guarantee to rely on. A stream-ordered read runs on
    // its caller's stream instead, which the drain does not cover: it is waited for as well.
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    for (const uintptr_t read : mStreamReads)
        CHECK_CU(getCudaShim()->eventSynchronize(read, nullptr));
    for (const std::vector<uintptr_t>* events : { &mStreamReads, &mIdleStreamReads })
        for (const uintptr_t event : *events)
            CHECK_CU(getCudaShim()->eventDestroy(event, nullptr));

    CHECK_CUDA(cudaFree(mPointRecordsDev));
    CHECK_CUDA(cudaFree(mVelocityRecordsDev));
}

bool GpuPointSetReadView::readColumnOvStage(const Column column,
                                            void* const dstDev,
                                            const PointSetTransform* const transformsDev,
                                            const OvStageDeviceReadArgs& args) const
{
    const bool reframe = (column == Column::ePoints);
    if (!mUsable || !dstDev || (reframe && !transformsDev))
        return false;
    PhysxCudaContextGuard ctxGuard(mCtxMgr);
    const void* records = reframe ? mPointRecordsDev : mVelocityRecordsDev;
    const bool launched =
        fetchPointSetColumnOvStage(static_cast<float*>(dstDev), static_cast<const GpuPointSetReadRecord*>(records),
                                   transformsDev, mNumSets, mMaxPoints, reframe, mLayout, args.stream);
    if (args.capturing)
        return launched;
    // A read whose event has completed no longer reads the records.
    for (size_t i = 0; i < mStreamReads.size();)
    {
        if (cudaEventQuery(reinterpret_cast<cudaEvent_t>(mStreamReads[i])) == cudaSuccess)
        {
            mIdleStreamReads.push_back(mStreamReads[i]);
            mStreamReads[i] = mStreamReads.back();
            mStreamReads.pop_back();
        }
        else
        {
            ++i;
        }
    }
    IOptionalCuda* const cuda = getCudaShim();
    uintptr_t read = 0;
    if (!mIdleStreamReads.empty())
    {
        read = mIdleStreamReads.back();
        mIdleStreamReads.pop_back();
    }
    int status = 0;
    if ((read || cuda->eventCreate(&read, CU_EVENT_DISABLE_TIMING, &status)) &&
        cuda->eventRecord(read, reinterpret_cast<uintptr_t>(args.stream), &status))
    {
        mStreamReads.push_back(read);
        return launched;
    }
    if (read)
        mIdleStreamReads.push_back(read);
    // Without the record the destructor cannot tell when this read stops reading the records, so wait for it now.
    CARB_LOG_ERROR_ONCE("Recording a stream-ordered point-set read failed (CUresult %d); waiting for its stream "
                        "instead, which stalls the host.",
                        status);
    return CHECK_CU(cuda->streamSynchronize(reinterpret_cast<uintptr_t>(args.stream), nullptr)) && launched;
}

} // namespace tensors
} // namespace physx
} // namespace omni
