// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "GpuSdfShapeView.h"
#include "GpuSimulationView.h"

#include <PxPhysicsAPI.h>
#include "CudaKernels.h"

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>

#include <omni/physics/tensors/TensorUtils.h>

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

GpuSdfShapeView::GpuSdfShapeView(GpuSimulationView* sim, const std::vector<SdfShapeEntry>& entries, int device)
    : BaseSdfShapeView(sim, entries), mDevice(device)
{
    if (sim)
    {
        mGpuSimData = sim->getGpuSimulationData();
    }

    PxU32 numObjects = getCount();
    samplePointCounts.resize(numObjects);
    mShapeIndices.resize(numObjects);
    mSdfRecords.resize(numObjects);

    for (PxU32 i = 0; i < numObjects; i++)
    {
        GpuSdfShapeRecord sdfRecord;
        sdfRecord.globalIndex = mEntries[i].shape->getGPUIndex();
        // having the global indices, look up the shape index in the object record
        mShapeToViewIndexMap[sdfRecord.globalIndex] = i;
        // TODO: change later for variable number of sample points
        sdfRecord.numSamplePoints = mEntries[i].numSamplePoints;
        mMaxNumPoints = PxMax(mMaxNumPoints, sdfRecord.numSamplePoints);

        if (mEntries[i].subspace)
        {
            const carb::Float3& origin = mEntries[i].subspace->origin;
            sdfRecord.origin = { origin.x, origin.y, origin.z };
        }
        else
        {
            sdfRecord.origin = { 0.0f, 0.0f, 0.0f };
        }

        samplePointCounts[i] = sdfRecord.numSamplePoints;
        mShapeIndices[i] = sdfRecord.globalIndex;
        mSdfRecords[i] = sdfRecord;
    }

    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    // shape indices
    if (!CHECK_CUDA(cudaMalloc(&mShapeIndicesD, mShapeIndices.size() * sizeof(PxU32))))
    {
        return;
    }
    if (!CHECK_CUDA(cudaMemcpy(
            mShapeIndicesD, mShapeIndices.data(), mShapeIndices.size() * sizeof(PxShapeGPUIndex), cudaMemcpyHostToDevice)))
    {
        return;
    }

    // sample point counts
    if (!CHECK_CUDA(cudaMalloc(&samplePointCountsD, numObjects * sizeof(PxU32))))
    {
        return;
    }
    if (!CHECK_CUDA(cudaMemcpy(
            samplePointCountsD, samplePointCounts.data(), numObjects * sizeof(PxU32), cudaMemcpyHostToDevice)))
    {
        return;
    }

    // sdf records
    if (!CHECK_CUDA(cudaMalloc(&mSdfRecordsDev, numObjects * sizeof(GpuSdfShapeRecord))))
    {
        return;
    }
    if (!CHECK_CUDA(cudaMemcpy(
            mSdfRecordsDev, mSdfRecords.data(), numObjects * sizeof(GpuSdfShapeRecord), cudaMemcpyHostToDevice)))
    {
        return;
    }

    // input points
    if (!CHECK_CUDA(cudaMalloc(&samplePointsPerShape, numObjects * mMaxNumPoints * sizeof(PxVec4))))
    {
        return;
    }
    if (!CHECK_CUDA(cudaMemset(samplePointsPerShape, 0, numObjects * mMaxNumPoints * sizeof(PxVec4))))
    {
        return;
    }
}

GpuSdfShapeView::~GpuSdfShapeView()
{
    CHECK_CUDA(cudaFree(mShapeIndicesD));
    CHECK_CUDA(cudaFree(mSdfRecordsDev));
    CHECK_CUDA(cudaFree(samplePointCountsD));
    CHECK_CUDA(cudaFree(samplePointsPerShape));
}


bool GpuSdfShapeView::getSdfAndGradients(const TensorDesc* dstTensor,
                                         const TensorDesc* srcPointTensor) const
{
    GPUAPI_CHECK_READY(mGpuSimData, false);
    if (!mGpuSimData)
    {
        return false;
    }


    if (!dstTensor || !dstTensor->data || !srcPointTensor || !srcPointTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "SDF and Gradient", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "SDF and Gradient", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxNumPoints * 4u, "SDF and Gradient", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*srcPointTensor, mDevice, "sample points", __FUNCTION__) ||
        !checkTensorFloat32(*srcPointTensor, "sample points", __FUNCTION__) ||
        !checkTensorSizeExact(*srcPointTensor, getCount() * mMaxNumPoints * 3u, "sample points", __FUNCTION__))
    {
        return false;
    }

    PxVec4* dstSDFVals = static_cast<PxVec4*>(dstTensor->data);
    PxVec3* srcPoints = static_cast<PxVec3*>(srcPointTensor->data);

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    // This step could have been avoided if we didn't have to re-arrange srcPointTensor data to PxVec4 
    // i.e. receiving srcPointTensor as tensor of PxVec4 and could just return the
    if (!submitSdfQueryPoints(samplePointsPerShape, srcPoints, mSdfRecordsDev, getCount(), mMaxNumPoints))
    {
        CARB_LOG_ERROR("Failed to submit SDF query points");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));


    CUevent signalEvent = mGpuSimData->mCopyEvents[CopyEvent::eSdfData];
    CHECK_CU(cuStreamWaitEvent(nullptr, signalEvent, 0));
    mGpuSimData->mScene->getDirectGPUAPI().evaluateSDFDistances(dstSDFVals, mShapeIndicesD, samplePointsPerShape, samplePointCountsD, (PxU32)mShapeIndices.size(), mMaxNumPoints, NULL, signalEvent);
    CHECK_CU(cuEventSynchronize(signalEvent));
    // CHECK_CUDA(cudaStreamSynchronize(nullptr));
    SYNCHRONIZE_CUDA();
    return true;
}

}
}
}
