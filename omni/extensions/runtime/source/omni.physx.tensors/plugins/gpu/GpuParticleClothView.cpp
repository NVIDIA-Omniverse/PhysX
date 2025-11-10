// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "CudaKernels.h"
#include "GpuParticleClothView.h"
#include "GpuSimulationView.h"

#include "../GlobalsAreBad.h"
#include "../CommonTypes.h"
#include "../SimulationBackend.h"

#include <PxPhysicsAPI.h>

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

GpuParticleClothView::GpuParticleClothView(GpuSimulationView* sim, const std::vector<ParticleClothEntry>& entries, int device)
    : BaseParticleClothView(sim, entries), mDevice(device)
{
    if (sim)
    {
        mGpuSimData = sim->getGpuSimulationData();
    }
    if (!mGpuSimData)
    {
        return;
    }

    PxU32 numCloths = getCount();

    std::vector<GpuParticleClothRecord> pcRecords(numCloths);
    std::vector<PxU32> pcIndices(numCloths);

    // helper data for GpuSimData update if needed.
    std::vector<PxGpuParticleBufferIndexPair> indexPairs;

    for (PxU32 i = 0; i < numCloths; i++)
    {
        GpuParticleClothRecord& pc = pcRecords[i];
        pcIndices[i] = i;

        if (mEntries[i].subspace)
        {
            const carb::Float3& origin = mEntries[i].subspace->origin;
            pc.origin = { origin.x, origin.y, origin.z };
        }
        else
        {
            pc.origin = { 0.0f, 0.0f, 0.0f };
        }

        // AD we assume this pointer doesn't change at runtime.
        PxParticleClothBuffer* clothBuffer = mEntries[i].cloth;
        pc.positions = clothBuffer->getPositionInvMasses();
        pc.numParticles = clothBuffer->getNbActiveParticles();
        pc.velocities = clothBuffer->getVelocities();
        
        // global index maps to systemIndex, bufferIndex pair for each cloth.
        pc.globalIndex = 0xffffffff;
        pc.index.bufferIndex = clothBuffer->bufferUniqueId;
        pc.index.systemIndex = mEntries[i].particleSystem->getGpuParticleSystemIndex();

        PxU64 combinedIndex = (uint64_t) pc.index.systemIndex << 32 | pc.index.bufferIndex;
        auto it = mIndexPair2ClothMap.find(combinedIndex);
        if (it !=mIndexPair2ClothMap.end())
        {
            pc.globalIndex = it->second;            
        }
        else // new cloth, fill the global list as we arrive here.
        {
            PxGpuParticleBufferIndexPair indexPair{ pc.index.systemIndex, pc.index.bufferIndex };
            mIndexPair2ClothMap[combinedIndex] = i;
            indexPairs.push_back(indexPair);
            pc.globalIndex = i;
        }

        // get the number of springs
        pc.numSprings = clothBuffer->getNbSprings();
        pc.springs = clothBuffer->getSprings();

        mMaxParticlesPerCloth = PxMax(mMaxParticlesPerCloth, pc.numParticles);
        mMaxSpringsPerCloth = PxMax(mMaxSpringsPerCloth, pc.numSprings);
    }

    mNumCloths = numCloths;

    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    // upload the per-view data to the GPU
    if (!CHECK_CUDA(cudaMalloc(&mPcRecordsDev, numCloths * sizeof(GpuParticleClothRecord))))
    {
        return;
    }
    if (!CHECK_CUDA(cudaMemcpy(mPcRecordsDev, pcRecords.data(), numCloths * sizeof(GpuParticleClothRecord), cudaMemcpyHostToDevice)))
    {
        return;
    }

    // cloth indices
    if (!CHECK_CUDA(cudaMalloc(&mClothIndicesDev, numCloths * sizeof(PxU32))))
    {
        return;
    }
    if (!CHECK_CUDA(cudaMemcpy(mClothIndicesDev, pcIndices.data(), numCloths * sizeof(PxU32), cudaMemcpyHostToDevice)))
    {
        return;
    }

    if (!CHECK_CUDA(cudaMalloc(&mDirtyPcIndicesDev, mNumCloths * sizeof(PxU32))))
    {
        return;
    }
    if (!CHECK_CUDA(cudaMemset(mDirtyPcIndicesDev, 0, mNumCloths * sizeof(PxU32))))
    {
        return;
    }
    if (!CHECK_CUDA(cudaMalloc(&mPcDirtyFlagsDev, mNumCloths * sizeof(PxParticleBufferFlags))))
    {
        return;
    }
    if (!CHECK_CUDA(cudaMemset(mPcDirtyFlagsDev, 0, mNumCloths * sizeof(PxParticleBufferFlags))))
    {
        return;
    }
    if (!CHECK_CUDA(cudaMalloc(&mPcIndexPairsDev, mNumCloths * sizeof(PxGpuParticleBufferIndexPair))))
    {
        return;
    }
    if (!CHECK_CUDA(cudaMemcpy(mPcIndexPairsDev, indexPairs.data(), mNumCloths * sizeof(PxGpuParticleBufferIndexPair),
                               cudaMemcpyHostToDevice)))
    {
        return;
    }

}

GpuParticleClothView::~GpuParticleClothView()
{
    if (mGpuSimData)
    {
        CudaContextGuard ctxGuard(mGpuSimData->mCtx);

        CHECK_CUDA(cudaFree(mPcRecordsDev));
        CHECK_CUDA(cudaFree(mClothIndicesDev));
        CHECK_CUDA(cudaFree(mPcDirtyFlagsDev));
        CHECK_CUDA(cudaFree(mDirtyPcIndicesDev));
        CHECK_CUDA(cudaFree(mPcIndexPairsDev));
    }
}

bool GpuParticleClothView::getPositions(const TensorDesc* dstTensor) const
{
        GPUAPI_CHECK_READY(mGpuSimData, false);
    
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }
    
    if (!checkTensorDevice(*dstTensor, mDevice, "positions", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "positions", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getMaxParticlesPerCloth() * getCount() * 3u, "positions", __FUNCTION__))
    {
        return false;
    }

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    SYNCHRONIZE_CUDA();

    if (!fetchParticleClothPositions(static_cast<PxVec3*>(dstTensor->data), mPcRecordsDev, getCount(), getMaxParticlesPerCloth()))
    {
        CARB_LOG_ERROR("failed to fetch particle cloth positions");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

// AD some notes:
// - we use the position buffer of the userBuffer as a staging buffer
// - we use the same dirty bits as the PxParticleBuffers, but maintain a staging list indexed by the global cloth index 
//   and mark the buffers dirty during submit. (mGpuSimData->mPcDirtyFlagsDev)
// - flush calls applyData, which will run an update kernel that copies the dirty buffers directly into the particle system.

bool GpuParticleClothView::applyParticleCloth()
{
    PxU32 numDirtyIndices =
        getPcDirtyIndices(mPcIndexSingleAllocPolicy, mDirtyPcIndicesDev, mPcDirtyFlagsDev, mNumCloths);
    CUevent waitEvent = mGpuSimData->mApplyWaitEvents[ApplyEvent::ePcBuffers];
    CUevent signalEvent = mGpuSimData->mApplySignalEvents[ApplyEvent::ePcBuffers];
    CHECK_CU(cuEventRecord(waitEvent, nullptr));
    CHECK_CU(cuEventSynchronize(waitEvent));
    mGpuSimData->mScene->applyParticleBufferData(
        mDirtyPcIndicesDev, mPcIndexPairsDev, mPcDirtyFlagsDev, numDirtyIndices, waitEvent, signalEvent);
    CHECK_CU(cuStreamWaitEvent(nullptr, (CUevent)signalEvent, 0));
    if (!CHECK_CUDA(cudaMemset(mPcDirtyFlagsDev, 0, mNumCloths * sizeof(PxParticleBufferFlags))))
    {
        return false;
    }
    if (!CHECK_CUDA(cudaMemset(mDirtyPcIndicesDev, 0, mNumCloths * sizeof(PxU32))))
    {
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    return true;
}

bool GpuParticleClothView::setPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, mDevice, "positions", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "positions", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getMaxParticlesPerCloth() * getCount() * 3u, "positions", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, mDevice, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mClothIndicesDev;
        numIndices = getCount();
    }

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    if (!submitParticleClothPositions(static_cast<PxVec3*>(srcTensor->data), indices, numIndices, mPcDirtyFlagsDev, mPcRecordsDev, getMaxParticlesPerCloth()))
    {
        CARB_LOG_ERROR("Failed to submit particle cloth positions");
        return false;
    }
    
    return applyParticleCloth();

}

bool GpuParticleClothView::getVelocities(const TensorDesc* dstTensor) const
{
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "velocities", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "velocities", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getMaxParticlesPerCloth() * getCount() * 3u, "velocities", __FUNCTION__))
    {
        return false;
    }

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    SYNCHRONIZE_CUDA();

    if (!fetchParticleClothVelocities(static_cast<PxVec3*>(dstTensor->data), mPcRecordsDev, getCount(), getMaxParticlesPerCloth()))
    {
        CARB_LOG_ERROR("failed to fetch particle cloth positions");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

bool GpuParticleClothView::setVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, mDevice, "velocities", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "velocities", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getMaxParticlesPerCloth() * getCount() * 3u, "velocities", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, mDevice, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mClothIndicesDev;
        numIndices = getCount();
    }

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    if (!submitParticleClothVelocities(static_cast<PxVec3*>(srcTensor->data), indices, numIndices, mPcDirtyFlagsDev, mPcRecordsDev, getMaxParticlesPerCloth()))
    {
        CARB_LOG_ERROR("Failed to submit particle cloth positions");
        return false;
    }

    return applyParticleCloth();

}

bool GpuParticleClothView::getMasses(const TensorDesc* dstTensor) const
{
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "masses", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "masses", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getMaxParticlesPerCloth() * getCount() * 1u, "masses", __FUNCTION__))
    {
        return false;
    }

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    SYNCHRONIZE_CUDA();

    if (!fetchParticleClothMasses(static_cast<float*>(dstTensor->data), mPcRecordsDev, getCount(), getMaxParticlesPerCloth()))
    {
        CARB_LOG_ERROR("failed to fetch particle cloth masses");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

bool GpuParticleClothView::setMasses(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, mDevice, "masses", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "masses", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getMaxParticlesPerCloth() * getCount() * 1u, "masses", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, mDevice, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mClothIndicesDev;
        numIndices = getCount();
    }

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    if (!submitParticleClothMasses(static_cast<float*>(srcTensor->data), indices, numIndices, mPcDirtyFlagsDev, mPcRecordsDev, getMaxParticlesPerCloth()))
    {
        CARB_LOG_ERROR("Failed to submit particle cloth positions");
        return false;
    }

    return applyParticleCloth();
}

bool GpuParticleClothView::getSpringDamping(const TensorDesc* dstTensor) const
{
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "spring_damping", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "spring_damping", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getMaxSpringsPerCloth() * getCount() * 1u, "spring_damping", __FUNCTION__))
    {
        return false;
    }

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    SYNCHRONIZE_CUDA();

    // we load directly from the PhysX spring pointer because that one is exposed.
    if (!fetchParticleClothSpringDamping(static_cast<float*>(dstTensor->data), mPcRecordsDev, getCount(), getMaxSpringsPerCloth()))
    {
        CARB_LOG_ERROR("failed to fetch particle cloth spring damping");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

bool GpuParticleClothView::setSpringDamping(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, mDevice, "spring_damping", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "spring_damping", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getMaxSpringsPerCloth() * getCount() * 1u, "spring_damping", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, mDevice, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mClothIndicesDev;
        numIndices = getCount();
    }

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    if (!submitParticleClothSpringDamping(static_cast<float*>(srcTensor->data), indices, numIndices, mPcRecordsDev, getMaxSpringsPerCloth()))
    {
        CARB_LOG_ERROR("Failed to submit particle cloth spring damping");
        return false;
    }

    return applyParticleCloth();
}

bool GpuParticleClothView::getSpringStiffness(const TensorDesc* dstTensor) const
{
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "spring_stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "spring_stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getMaxSpringsPerCloth() * getCount() * 1u, "spring_stiffness", __FUNCTION__))
    {
        return false;
    }

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    SYNCHRONIZE_CUDA();

    // we load directly from the PhysX spring pointer because that one is exposed.
    if (!fetchParticleClothSpringStiffness(static_cast<float*>(dstTensor->data), mPcRecordsDev, getCount(), getMaxSpringsPerCloth()))
    {
        CARB_LOG_ERROR("failed to fetch particle cloth spring stiffness");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

bool GpuParticleClothView::setSpringStiffness(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, mDevice, "spring_stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "spring_stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getMaxSpringsPerCloth() * getCount() * 1u, "spring_stiffness", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, mDevice, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mClothIndicesDev;
        numIndices = getCount();
    }

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    if (!submitParticleClothSpringStiffness(static_cast<float*>(srcTensor->data), indices, numIndices, mPcRecordsDev, getMaxSpringsPerCloth()))
    {
        CARB_LOG_ERROR("Failed to submit particle cloth spring stiffness");
        return false;
    }

    return applyParticleCloth();
}

}
}
}
