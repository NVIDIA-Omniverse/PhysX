// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXDeformablePost.h"
#include "PhysXDefines.h"
#include "gpu/PxPhysicsGpu.h"
#include "extensions/PxCudaHelpersExt.h"

using namespace pxr;
using namespace carb;
using namespace ::physx;
using namespace physx::Ext;
using namespace omni::physx;
using namespace omni::physx::deformables;

void VolumeDeformableSkinningData::packageSkinningData(PxTetmeshSkinningGpuData& packagedData)
{
    if (!mDeformableVolume)
        return;

    packagedData.guideVerticesD.data = reinterpret_cast<PxVec3*>(mDeformableVolume->getSimPositionInvMassBufferD());
    packagedData.guideVerticesD.stride = sizeof(PxVec4);
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
    : mSkinningStream(stream), mCudaContextManager(cudaContextManager), mScene(scene), mCurrentSize(0), mNeedResize(false), isRegistered(false)
{
    mSkinning = PxGetPhysicsGpu()->createDeformableSkinning(cudaContextManager);
}

void DeformablePostSolveCallback::synchronize()
{
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

void VolumeDeformablePostSolveCallback::copySkinnedVerticesDtoHAsync(size_t deformableIndex, PxVec3* mAllSkinnedVerticesH)
{
    if (!mAllSkinnedVerticesH || deformableIndex >= mSkinningData.size() || !mSkinningData[deformableIndex].mAllSkinnedVerticesD)
        return;

    PxCudaHelpersExt::copyDToHAsync(*mCudaContextManager, mAllSkinnedVerticesH, mSkinningData[deformableIndex].mAllSkinnedVerticesD, mSkinningData[deformableIndex].mNumSkinnedVertices, mSkinningStream);
}

void VolumeDeformablePostSolveCallback::onPostSolve(CUevent startEvent)
{
    mCudaContextManager->getCudaContext()->streamWaitEvent(mSkinningStream, startEvent);

    if (mNeedResize)
    {
        // Check the current size of mSkinningData
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

        PxCudaHelpersExt::copyHToD(*mCudaContextManager, mPackagedSkinningDataD, mPackagedSkinningDataH, skinningDataSize);

        mSkinning->evaluateVerticesEmbeddedIntoVolume(mPackagedSkinningDataD, skinningDataSize, mSkinningStream);
    }
}

void VolumeDeformablePostSolveCallback::addVolumeDeformableSkinningData(const VolumeDeformableSkinningData& skinningData)
{
    mSkinningData.push_back(skinningData);
    mNeedResize = true;

    // Register the callback with the scene if it wasn't done already
    if (!isRegistered)
    {
        mScene->setDeformableVolumeGpuPostSolveCallback(this);
        isRegistered = true;
    }
}

void VolumeDeformablePostSolveCallback::removeVolumeDeformableSkinningData(const size_t index)
{
    auto it = mSkinningData.begin() + index;
    if (it != mSkinningData.end())
    {
        *it = mSkinningData.back();
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

        // Handle allocation failure
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

void SurfaceDeformablePostSolveCallback::copySkinnedVerticesDtoHAsync(size_t deformableIndex, PxVec3* mAllSkinnedVerticesH)
{
    if (!mAllSkinnedVerticesH || deformableIndex >= mSkinningData.size() || !mSkinningData[deformableIndex].mSkinnedVerticesD)
        return;

    PxCudaHelpersExt::copyDToHAsync(*mCudaContextManager, mAllSkinnedVerticesH, mSkinningData[deformableIndex].mSkinnedVerticesD, mSkinningData[deformableIndex].mNumSkinnedVertices, mSkinningStream);
}

void SurfaceDeformablePostSolveCallback::onPostSolve(CUevent startEvent)
{
    mCudaContextManager->getCudaContext()->streamWaitEvent(mSkinningStream, startEvent);

    if (mNeedResize)
    {
        // Check the current size of mSkinningData
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

        PxCudaHelpersExt::copyHToD(*mCudaContextManager, mPackagedSkinningDataD, mPackagedSkinningDataH, skinningDataSize);

        mSkinning->computeNormalVectors(mPackagedSkinningDataD, skinningDataSize, mSkinningStream);
        mSkinning->evaluateVerticesEmbeddedIntoSurface(mPackagedSkinningDataD, skinningDataSize, mSkinningStream);
    }
}

void SurfaceDeformablePostSolveCallback::addSurfaceDeformableSkinningData(const SurfaceDeformableSkinningData& skinningData)
{
    mSkinningData.push_back(skinningData);
    mNeedResize = true;

    // Register the callback with the scene if it wasn't done already
    if (!isRegistered)
    {
        mScene->setDeformableSurfaceGpuPostSolveCallback(this);
        isRegistered = true;
    }
}

void SurfaceDeformablePostSolveCallback::removeSurfaceDeformableSkinningData(const size_t index)
{
    auto it = mSkinningData.begin() + index;
    if (it != mSkinningData.end())
    {
        *it = mSkinningData.back();
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

        // Handle allocation failure
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
