// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

        PxCudaHelpersExt::copyHToDAsync(*mCudaContextManager, mPackagedSkinningDataD, mPackagedSkinningDataH, skinningDataSize, mSkinningStream);

        mSkinning->evaluateVerticesEmbeddedIntoVolume(mPackagedSkinningDataD, skinningDataSize, mSkinningStream);

        mCudaContextManager->getCudaContext()->eventRecord(startEvent, mSkinningStream);
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

        PxCudaHelpersExt::copyHToDAsync(*mCudaContextManager, mPackagedSkinningDataD, mPackagedSkinningDataH, skinningDataSize, mSkinningStream);

        mSkinning->computeNormalVectors(mPackagedSkinningDataD, skinningDataSize, mSkinningStream);
        mSkinning->evaluateVerticesEmbeddedIntoSurface(mPackagedSkinningDataD, skinningDataSize, mSkinningStream);

        mCudaContextManager->getCudaContext()->eventRecord(startEvent, mSkinningStream);
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
