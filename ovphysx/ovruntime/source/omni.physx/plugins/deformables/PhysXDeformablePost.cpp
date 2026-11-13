// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-DEFORM-SKIN-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-READ-SKIN-001
 * @covers AC-4
 */

#include "PhysXDeformablePost.h"
#include "PhysXDefines.h"
#include "gpu/PxPhysicsGpu.h"
#include "extensions/PxCudaHelpersExt.h"

#include <cuda.h>

using namespace carb;
using namespace ::physx;
using namespace physx::Ext;
using namespace omni::physx;
using namespace omni::physx::deformables;

namespace
{
    size_t findSkinningData(const std::vector<VolumeDeformableSkinningData>& skinningData, const PxDeformableVolume* deformableVolume)
    {
        size_t index = skinningData.size();
        for (size_t i = 0; i < skinningData.size(); ++i)
        {
            if (skinningData[i].mDeformableVolume == deformableVolume)
            {
                index = i;
                break;
            }
        }

        return index;
    }

    size_t findSkinningData(const std::vector<SurfaceDeformableSkinningData>& skinningData, const PxDeformableSurface* deformableSurface)
    {
        size_t index = skinningData.size();
        for (size_t i = 0; i < skinningData.size(); ++i)
        {
            if (skinningData[i].mDeformableSurface == deformableSurface)
            {
                index = i;
                break;
            }
        }

        return index;
    }
}

void VolumeDeformableSkinningData::packageSkinningData(PxTetmeshSkinningGpuData& packagedData)
{
    if (!mDeformableVolume)
        return;

    packagedData.guideVerticesD.data = reinterpret_cast<PxVec3*>(mDeformableVolume->getSimPositionInvMassBufferD());
    packagedData.guideVerticesD.stride = sizeof(PxVec4);
    const PxTetrahedronMesh* simulationMesh = mDeformableVolume->getSimulationMesh();
    packagedData.guideVerticesD.count = simulationMesh ? simulationMesh->getNbVertices() : 0;
    packagedData.guideTetrahedraD = mGuideTetrahedraD;
    packagedData.skinningInfoPerVertexD = mSkinningEmbeddingInfoD;
    packagedData.skinnedVerticesD.count = mNumSkinnedVertices;
    packagedData.skinnedVerticesD.stride = sizeof(PxVec3);
    packagedData.skinnedVerticesD.data = mAllSkinnedVerticesD;
}

void SurfaceDeformableSkinningData::packageSkinningData(PxTrimeshSkinningGpuData& packagedData)
{
    if (!mDeformableSurface)
        return;

    packagedData.guideVerticesD.data = reinterpret_cast<PxVec3*>(mDeformableSurface->getPositionInvMassBufferD());
    packagedData.guideVerticesD.stride = sizeof(PxVec4);
    packagedData.guideVerticesD.count = mNumGuideVertices;

    packagedData.guideNormalsD = mGuideNormalsD;
    packagedData.guideTrianglesD = mGuideTrianglesD;
    packagedData.skinningInfoPerVertexD = mSkinningEmbeddingInfoD;
    packagedData.skinnedVerticesD.data = mSkinnedVerticesD;
    packagedData.skinnedVerticesD.stride = sizeof(PxVec3);
    packagedData.skinnedVerticesD.count = mNumSkinnedVertices;

    packagedData.halfSurfaceThickness = mHalfThickness;
    packagedData.nbGuideTriangles = mNumGuideTriangles;
}

DeformablePostSolveCallback::DeformablePostSolveCallback(CUstream stream, PxCudaContextManager* cudaContextManager, ::physx::PxScene* scene)
    : mSkinningStream(stream), mSkinningDoneEvent(nullptr), mCudaContextManager(cudaContextManager), mScene(scene), mCurrentSize(0), mNeedResize(false), isRegistered(false)
{
    mSkinning = PxGetPhysicsGpu()->createDeformableSkinning(cudaContextManager);

    // Created once with the callback, so consumers keep the handle across launches. Timing is disabled
    // because the event only orders work.
    if (mCudaContextManager && mCudaContextManager->getCudaContext())
    {
        PxScopedCudaLock _lock(*mCudaContextManager);
        PxCudaContext* cudaContext = mCudaContextManager->getCudaContext();
        if (cudaContext->eventCreate(&mSkinningDoneEvent, CU_EVENT_DISABLE_TIMING) != CUDA_SUCCESS)
        {
            mSkinningDoneEvent = nullptr;
            CARB_LOG_WARN("DeformablePostSolveCallback: could not create the skinning ordering event, so the "
                          "output read refuses to gather skin-mesh points.");
        }
    }
}

DeformablePostSolveCallback::~DeformablePostSolveCallback()
{
    // The skinning object must be released before the context manager, as the SDK snippets do with PX_DELETE.
    // InternalScene::release destroys the copy stream before it deletes the callbacks, so synchronize() must
    // not run here. No wait is needed, because cuEventDestroy defers until pending work completes and the
    // skinning object holds no stream.
    if (mCudaContextManager && mCudaContextManager->getCudaContext())
    {
        PxScopedCudaLock _lock(*mCudaContextManager);
        PxCudaContext* cudaContext = mCudaContextManager->getCudaContext();
        if (mSkinningDoneEvent)
            cudaContext->eventDestroy(mSkinningDoneEvent);
        for (const std::vector<CUevent>* events : { &mConsumerDoneEvents, &mIdleConsumerDoneEvents })
            for (CUevent event : *events)
                cudaContext->eventDestroy(event);
        PX_DELETE(mSkinning);
    }
    mSkinningDoneEvent = nullptr;
    mConsumerDoneEvents.clear();
    mIdleConsumerDoneEvents.clear();
}

void DeformablePostSolveCallback::recordSkinningDone(PxCudaContext* cudaContext)
{
    if (mSkinningDoneEvent)
        cudaContext->eventRecord(mSkinningDoneEvent, mSkinningStream);
}

bool DeformablePostSolveCallback::recordConsumerDone(PxCudaContext* cudaContext, CUstream consumerStream)
{
    // The host waits for an in-flight step before a read, so this never overlaps onPostSolve and needs no
    // lock. Completed events return to the pool here, because a callback without skinning data of its kind
    // is never registered and its onPostSolve never drains the list.
    for (size_t i = 0; i < mConsumerDoneEvents.size();)
    {
        if (cudaContext->eventQuery(mConsumerDoneEvents[i]) == CUDA_SUCCESS)
        {
            mIdleConsumerDoneEvents.push_back(mConsumerDoneEvents[i]);
            mConsumerDoneEvents[i] = mConsumerDoneEvents.back();
            mConsumerDoneEvents.pop_back();
        }
        else
        {
            ++i;
        }
    }

    CUevent event = nullptr;
    if (!mIdleConsumerDoneEvents.empty())
    {
        event = mIdleConsumerDoneEvents.back();
        mIdleConsumerDoneEvents.pop_back();
    }
    else if (cudaContext->eventCreate(&event, CU_EVENT_DISABLE_TIMING) != CUDA_SUCCESS)
    {
        return false;
    }

    if (cudaContext->eventRecord(event, consumerStream) != CUDA_SUCCESS)
    {
        mIdleConsumerDoneEvents.push_back(event);
        return false;
    }
    mConsumerDoneEvents.push_back(event);
    return true;
}

void DeformablePostSolveCallback::waitForConsumers(PxCudaContext* cudaContext)
{
    // Keeps the skinning launch from rewriting the skinned vertices during a consumer's gather. The wait
    // captures each record as it stands, so the event returns to the pool at once.
    for (CUevent event : mConsumerDoneEvents)
    {
        const PxCUresult result = cudaContext->streamWaitEvent(mSkinningStream, event);
        if (result != CUDA_SUCCESS)
            CARB_LOG_WARN_ONCE("DeformablePostSolveCallback: the skinning could not wait for a skin-mesh read "
                               "(CUresult %d), so it may rewrite the points while that read gathers them.",
                               static_cast<int>(result.value));
        mIdleConsumerDoneEvents.push_back(event);
    }
    mConsumerDoneEvents.clear();
}

void DeformablePostSolveCallback::synchronize()
{
    if (!mCudaContextManager || !mCudaContextManager->getCudaContext())
    {
        CARB_LOG_WARN_ONCE("DeformablePostSolveCallback::synchronize: CUDA context unavailable, skipping.");
        return;
    }

    mCudaContextManager->getCudaContext()->streamSynchronize(mSkinningStream);
}

VolumeDeformablePostSolveCallback::VolumeDeformablePostSolveCallback(CUstream stream, PxCudaContextManager* cudaContextManager, ::physx::PxScene* scene)
    : DeformablePostSolveCallback(stream, cudaContextManager, scene), mPackagedSkinningDataH(nullptr), mPackagedSkinningDataD(nullptr) 
{
}

VolumeDeformablePostSolveCallback::~VolumeDeformablePostSolveCallback()
{
    releaseMemory();
}

void VolumeDeformablePostSolveCallback::copySkinnedVerticesDtoHAsync(PxDeformableVolume* deformableVolume, PxVec3* allSkinnedVerticesH)
{
    size_t index = findSkinningData(mSkinningData, deformableVolume);

    // TODO: Potential optimization to use the copy stream instead of mSkinningStream
    if (!allSkinnedVerticesH || index >= mSkinningData.size() || !mSkinningData[index].mAllSkinnedVerticesD)
        return;

    PxCudaHelpersExt::copyDToHAsync(*mCudaContextManager, allSkinnedVerticesH, mSkinningData[index].mAllSkinnedVerticesD, mSkinningData[index].mNumSkinnedVertices, mSkinningStream);
}

void VolumeDeformablePostSolveCallback::onPostSolve(CUevent startEvent)
{
    if (!mCudaContextManager || !mCudaContextManager->getCudaContext())
    {
        CARB_LOG_WARN_ONCE("VolumeDeformablePostSolveCallback::onPostSolve: CUDA context unavailable, skipping.");
        return;
    }
    PxCudaContext* cudaContext = mCudaContextManager->getCudaContext();

    cudaContext->streamWaitEvent(mSkinningStream, startEvent);
    waitForConsumers(cudaContext);

    if (mNeedResize)
    {
        const size_t newSize = mSkinningData.size();
        if (newSize != mCurrentSize) {
            resizeMemory(newSize);
        }

        mNeedResize = false;
    }

    const PxU32 skinningDataSize = (PxU32)mSkinningData.size();
    if (mPackagedSkinningDataH && mPackagedSkinningDataD)
    {
        for (PxU32 i = 0; i < skinningDataSize; ++i)
        {
            mSkinningData[i].packageSkinningData(mPackagedSkinningDataH[i]);
        }

        PxCudaHelpersExt::copyHToDAsync(*mCudaContextManager, mPackagedSkinningDataD, mPackagedSkinningDataH, skinningDataSize, mSkinningStream);

        mSkinning->evaluateVerticesEmbeddedIntoVolume(mPackagedSkinningDataD, skinningDataSize, mSkinningStream);

        cudaContext->eventRecord(startEvent, mSkinningStream);
        recordSkinningDone(cudaContext);
    }
}

void VolumeDeformablePostSolveCallback::addVolumeDeformableSkinningData(const VolumeDeformableSkinningData& skinningData)
{
    mSkinningData.push_back(skinningData);
    mNeedResize = true;

    if (!isRegistered)
    {
        mScene->setDeformableVolumeGpuPostSolveCallback(this);
        isRegistered = true;
    }
}

void VolumeDeformablePostSolveCallback::removeVolumeDeformableSkinningData(const PxDeformableVolume* deformableVolume)
{
    size_t index = findSkinningData(mSkinningData, deformableVolume);

    if (index < mSkinningData.size())
    {
        mSkinningData[index] = mSkinningData.back();
        mSkinningData.pop_back();
    }

    mNeedResize = true;
}

void VolumeDeformablePostSolveCallback::resizeMemory(const size_t newSize)
{
    releaseMemory();

    if (newSize > 0)
    {
        mPackagedSkinningDataH = PxCudaHelpersExt::allocPinnedHostBuffer<PxTetmeshSkinningGpuData>(*mCudaContextManager, newSize);
        mPackagedSkinningDataD = PxCudaHelpersExt::allocDeviceBuffer<PxTetmeshSkinningGpuData>(*mCudaContextManager, newSize);

        if (!mPackagedSkinningDataH || !mPackagedSkinningDataD)
        {
            releaseMemory();
            mCurrentSize = 0;
            return;
        }
    }

    mCurrentSize = newSize;
}

void VolumeDeformablePostSolveCallback::releaseMemory()
{
    if (mPackagedSkinningDataH)
    {
        PxCudaHelpersExt::freePinnedHostBuffer(*mCudaContextManager, mPackagedSkinningDataH);
        mPackagedSkinningDataH = nullptr;
    }


    if (mPackagedSkinningDataD)
    {
        PxCudaHelpersExt::freeDeviceBuffer(*mCudaContextManager, mPackagedSkinningDataD);
        mPackagedSkinningDataD = nullptr;
    }
}

SurfaceDeformablePostSolveCallback::SurfaceDeformablePostSolveCallback(CUstream stream, PxCudaContextManager* cudaContextManager, ::physx::PxScene* scene)
    : DeformablePostSolveCallback(stream, cudaContextManager, scene), mPackagedSkinningDataH(nullptr), mPackagedSkinningDataD(nullptr)
{
}

SurfaceDeformablePostSolveCallback::~SurfaceDeformablePostSolveCallback()
{
    releaseMemory();
}

void SurfaceDeformablePostSolveCallback::copySkinnedVerticesDtoHAsync(PxDeformableSurface* deformableSurface, PxVec3* allSkinnedVerticesH)
{
    size_t index = findSkinningData(mSkinningData, deformableSurface);

    // TODO: Potential optimization to use the copy stream instead of mSkinningStream
    if (!allSkinnedVerticesH || index >= mSkinningData.size() || !mSkinningData[index].mSkinnedVerticesD)
        return;

    PxCudaHelpersExt::copyDToHAsync(*mCudaContextManager, allSkinnedVerticesH, mSkinningData[index].mSkinnedVerticesD, mSkinningData[index].mNumSkinnedVertices, mSkinningStream);
}

void SurfaceDeformablePostSolveCallback::onPostSolve(CUevent startEvent)
{
    if (!mCudaContextManager || !mCudaContextManager->getCudaContext())
    {
        CARB_LOG_WARN_ONCE("SurfaceDeformablePostSolveCallback::onPostSolve: CUDA context unavailable, skipping.");
        return;
    }
    PxCudaContext* cudaContext = mCudaContextManager->getCudaContext();

    cudaContext->streamWaitEvent(mSkinningStream, startEvent);
    waitForConsumers(cudaContext);

    if (mNeedResize)
    {
        size_t newSize = mSkinningData.size();
        if (newSize != mCurrentSize) {
            resizeMemory(newSize);
        }

        mNeedResize = false;
    }

    const PxU32 skinningDataSize = (PxU32)mSkinningData.size();
    if (mPackagedSkinningDataH && mPackagedSkinningDataD)
    {
        for (PxU32 i = 0; i < skinningDataSize; ++i)
        {
            mSkinningData[i].packageSkinningData(mPackagedSkinningDataH[i]);
        }

        PxCudaHelpersExt::copyHToDAsync(*mCudaContextManager, mPackagedSkinningDataD, mPackagedSkinningDataH, skinningDataSize, mSkinningStream);

        mSkinning->computeNormalVectors(mPackagedSkinningDataD, skinningDataSize, mSkinningStream);
        mSkinning->evaluateVerticesEmbeddedIntoSurface(mPackagedSkinningDataD, skinningDataSize, mSkinningStream);

        cudaContext->eventRecord(startEvent, mSkinningStream);
        recordSkinningDone(cudaContext);
    }
}

void SurfaceDeformablePostSolveCallback::addSurfaceDeformableSkinningData(const SurfaceDeformableSkinningData& skinningData)
{
    mSkinningData.push_back(skinningData);
    mNeedResize = true;

    if (!isRegistered)
    {
        mScene->setDeformableSurfaceGpuPostSolveCallback(this);
        isRegistered = true;
    }
}

void SurfaceDeformablePostSolveCallback::removeSurfaceDeformableSkinningData(const PxDeformableSurface* deformableSurface)
{
    size_t index = findSkinningData(mSkinningData, deformableSurface);

    if (index < mSkinningData.size())
    {
        mSkinningData[index] = mSkinningData.back();
        mSkinningData.pop_back();
    }

    mNeedResize = true;
}

void SurfaceDeformablePostSolveCallback::resizeMemory(const size_t newSize)
{
    releaseMemory();

    if (newSize > 0)
    {
        mPackagedSkinningDataH = PxCudaHelpersExt::allocPinnedHostBuffer<PxTrimeshSkinningGpuData>(*mCudaContextManager, newSize);
        mPackagedSkinningDataD = PxCudaHelpersExt::allocDeviceBuffer<PxTrimeshSkinningGpuData>(*mCudaContextManager, newSize);

        if (!mPackagedSkinningDataH || !mPackagedSkinningDataD)
        {
            releaseMemory();
            mCurrentSize = 0;
            return;
        }
    }

    mCurrentSize = newSize;
}

void SurfaceDeformablePostSolveCallback::releaseMemory()
{
    if (mPackagedSkinningDataH)
    {
        PxCudaHelpersExt::freePinnedHostBuffer(*mCudaContextManager, mPackagedSkinningDataH);
        mPackagedSkinningDataH = nullptr;
    }


    if (mPackagedSkinningDataD)
    {
        PxCudaHelpersExt::freeDeviceBuffer(*mCudaContextManager, mPackagedSkinningDataD);
        mPackagedSkinningDataD = nullptr;
    }
}
