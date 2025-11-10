// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <OmniPhysX.h>

#include "FabricParticles.h"
#include "PhysXTools.h"
#include "internal/InternalParticle.h"

#include <carb/logging/Log.h>
#include <usdLoad/LoadUsd.h>

#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSettings.h>
#include <omni/physx/IPhysxSimulation.h>

#include <cuda.h>

#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"

using namespace pxr;
using namespace carb;
using namespace omni::fabric;
using namespace ::physx;

namespace omni
{
namespace physx
{

FabricParticles::FabricParticles()
{
    mStageId.id = 0;

    mFabric = nullptr;
    mIStageReaderWriter = nullptr;

    omni::fabric::Token::iToken = carb::getCachedInterface<omni::fabric::IToken>();
    omni::fabric::Path::iPath = carb::getCachedInterface<omni::fabric::IPath>();

    mAppliedSchemaType = omni::fabric::Type(omni::fabric::BaseDataType::eTag, 1, 0, omni::fabric::AttributeRole::eAppliedSchema);
    mFloat4ArrayType = omni::fabric::Type(omni::fabric::BaseDataType::eFloat, 4, 1, omni::fabric::AttributeRole::eNone);

    mParticleSetApiToken = omni::fabric::Token("PhysxParticleSetAPI");
    mPositionInvMassesToken = omni::fabric::Token("_positionInvMasses");
    mVelocitiesFloat4Token = omni::fabric::Token("_velocitiesFloat4");
}

FabricParticles::~FabricParticles()
{
}

void FabricParticles::attachStage(long stageId)
{
    carb::Framework* framework = carb::getFramework();

    mFabric = framework->tryAcquireInterface<omni::fabric::IFabric>();
    mIStageReaderWriter = framework->tryAcquireInterface<omni::fabric::IStageReaderWriter>();

    mStageId.id = stageId;

    mCudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
}

void FabricParticles::detachStage()
{
    mStageId.id = 0;

    mFabric = nullptr;
    mIStageReaderWriter = nullptr;

    mCudaContextManager = nullptr;
}

void FabricParticles::update()
{
#if 0
    // This PhysX Fabric is normally updated by the PhysX Fabric extension, but not
    // when running a simulation manually from Python.
    if (mIPhysxFabric)
    {
        // The current time and time step are not used in the update function.
        mIPhysxFabric->update(currentTime, elapsedSecs);
    }
#endif

    if (!mParticleSets.empty())
    {
        updateFabric();

#if 0
        // Enable this to render the fabric particles. Useful for debugging.
        saveToUsd();
#endif
    }
}

void FabricParticles::reset()
{
    omni::fabric::StageReaderWriterId sipId = mIStageReaderWriter->get(mStageId);

    for (auto particleSet : mParticleSets)
    {
        omni::fabric::PathC primPath = omni::fabric::asInt(particleSet.first);

        mIStageReaderWriter->destroyAttribute2(sipId, primPath, mPositionInvMassesToken);
        mIStageReaderWriter->destroyAttribute2(sipId, primPath, mVelocitiesFloat4Token);
    }

    mParticleSets.clear();
}

void FabricParticles::initializeParticleSet(internal::InternalParticleSet* particleSet)
{
    // pause change tracking so that we dont get transformations back
    const bool changeTrackingPaused = usdparser::UsdLoad::getUsdLoad()->isChangeTrackingPaused(mStageId.id);
    usdparser::UsdLoad::getUsdLoad()->pauseChangeTracking(mStageId.id, true);

    omni::fabric::StageReaderWriterId sipId = mIStageReaderWriter->get(mStageId);

    omni::fabric::PathC primPath = omni::fabric::asInt(particleSet->mPrim.GetPrimPath());
    mIStageReaderWriter->prefetchPrim(mStageId, primPath);

    mIStageReaderWriter->createAttribute(sipId, primPath, mPositionInvMassesToken, { (omni::fabric::TypeC)mFloat4ArrayType });
    mIStageReaderWriter->createAttribute(sipId, primPath, mVelocitiesFloat4Token, (omni::fabric::TypeC)mFloat4ArrayType);

    mIStageReaderWriter->setArrayAttributeSize(sipId, primPath, mPositionInvMassesToken, particleSet->mNumParticles);
    mIStageReaderWriter->setArrayAttributeSize(sipId, primPath, mVelocitiesFloat4Token, particleSet->mNumParticles);

    SpanWithTypeC positionsGpu = mIStageReaderWriter->getAttributeWrGpu(sipId, primPath, mPositionInvMassesToken);
    SpanWithTypeC velocitiesGpu = mIStageReaderWriter->getAttributeWrGpu(sipId, primPath, mVelocitiesFloat4Token);

    particleSet->mFabricPositions = (*reinterpret_cast<carb::Float4**>(positionsGpu.ptr));
    particleSet->mFabricVelocities = (*reinterpret_cast<carb::Float4**>(velocitiesGpu.ptr));

    if (particleSet->mNumParticles)
    {
        if (mCudaContextManager)
        {
            PxCudaContext* cudaContext = mCudaContextManager->getCudaContext();

            cudaContext->memcpyHtoD(CUdeviceptr(*reinterpret_cast<carb::Float4**>(positionsGpu.ptr)), particleSet->mPositions, particleSet->mNumParticles * sizeof(carb::Float4));
            cudaContext->memcpyHtoD(CUdeviceptr(*reinterpret_cast<carb::Float4**>(velocitiesGpu.ptr)), particleSet->mVelocities, particleSet->mNumParticles * sizeof(carb::Float4));

            mCudaContextManager->releaseContext();
        }
    }

    mParticleSets[particleSet->mPrim.GetPrimPath()] = particleSet;

    usdparser::UsdLoad::getUsdLoad()->pauseChangeTracking(mStageId.id, changeTrackingPaused);
}

void FabricParticles::updateParticles(const PrimBucketList& primBucketList, size_t primBucketListIndex, const Token& token)
{
    omni::fabric::StageReaderWriterId sipId = mIStageReaderWriter->get(mStageId);
    StageReaderWriter srw = mIStageReaderWriter->get(mStageId);

    gsl::span<const omni::fabric::Path> paths = srw.getPathArray(primBucketList, primBucketListIndex);
    for (const omni::fabric::Path& path : paths)
    {
        pxr::SdfPath usdPath = omni::fabric::toSdfPath(path);
        internal::InternalParticleSet* particleSet = mParticleSets[usdPath];

        if (particleSet)
        {
            if (token == mPositionInvMassesToken)
            {
                ConstSpanWithTypeC positions = mIStageReaderWriter->getArrayAttributeRd(sipId, path, mPositionInvMassesToken);

                if (positions.elementCount != particleSet->mNumParticles)
                {
                    particleSet->resize(uint32_t(positions.elementCount));
                }

#if ENABLE_BATCH_PROCESSING
                SpanC positionsGpu = mIStageReaderWriter->getAttributeWrGpu(sipId, path, mPositionsToken);

                modifiedData data;
                data.fabricPtr = (*reinterpret_cast<carb::Float3**>(positionsGpu.ptr));
                data.cudaPtr = particleSet->mParticleBuffer->getPositionInvMasses();
                data.size = positions.elementCount;
                mPositionsChanged.push_back(data);
#else
                SpanWithTypeC positionsGpu = mIStageReaderWriter->getAttributeWrGpu(sipId, path, mPositionInvMassesToken);
                particleSet->mFabricPositions = (*reinterpret_cast<carb::Float4**>(positionsGpu.ptr));

                particleSet->mUploadDirtyFlags |= internal::ParticleBufferFlags::ePOSITIONS;
#endif
            }

            if (token == mVelocitiesFloat4Token)
            {
                ConstSpanWithTypeC velocities = mIStageReaderWriter->getArrayAttributeRd(sipId, path, mVelocitiesFloat4Token);

                if (velocities.elementCount != particleSet->mNumParticles)
                {
                    particleSet->resize(uint32_t(velocities.elementCount));
                }

#if ENABLE_BATCH_PROCESSING
                SpanC velocitiesGpu = mIStageReaderWriter->getAttributeWrGpu(sipId, path, mVelocitiesToken);

                modifiedData data;
                data.fabricPtr = (*reinterpret_cast<carb::Float3**>(velocitiesGpu.ptr));
                data.cudaPtr = particleSet->mParticleBuffer->getVelocities();
                data.size = velocities.elementCount;
                mVelocitiesChanged.push_back(data);
#else
                SpanWithTypeC velocitiesGpu = mIStageReaderWriter->getAttributeWrGpu(sipId, path, mVelocitiesFloat4Token);
                particleSet->mFabricVelocities = (*reinterpret_cast<carb::Float4**>(velocitiesGpu.ptr));

                particleSet->mUploadDirtyFlags |= internal::ParticleBufferFlags::eVELOCITIES;
#endif
            }
        }
    }
}

void FabricParticles::updateParticles()
{
    static const omni::fabric::set<AttrNameAndType_v2> requiredAll = { AttrNameAndType_v2(mAppliedSchemaType, mParticleSetApiToken) };
    static const omni::fabric::set<AttrNameAndType_v2> requiredAny = { AttrNameAndType_v2(mFloat4ArrayType, mPositionInvMassesToken),
                                                                    AttrNameAndType_v2(mFloat4ArrayType, mVelocitiesFloat4Token) };

    StageReaderWriter srw = mIStageReaderWriter->get(mStageId);
    PrimBucketList primBuckets = srw.findPrims(requiredAll, requiredAny);
    size_t bucketCount = primBuckets.bucketCount();

    for (size_t i = 0; i != bucketCount; i++)
    {
        updateParticles(primBuckets, i, mPositionInvMassesToken);
        updateParticles(primBuckets, i, mVelocitiesFloat4Token);
    }

#if ENABLE_BATCH_PROCESSING
    if (mCudaContextManager)
    {
        mCudaContextManager->acquireContext();
        PxCudaContext* cudaContext = mCudaContextManager->getCudaContext();

        size_t maxBufferSize = mParticleSets.size();

        cudaContext->memAlloc(&mInPtrDev, sizeof(CUdeviceptr) * maxBufferSize);
        cudaContext->memAlloc(&mOutPtrDev, sizeof(CUdeviceptr) * maxBufferSize);
        cudaContext->memAlloc(&mSizePtrDev, sizeof(int) * maxBufferSize);
        cudaContext->memHostAlloc((void**)&mInPtrHost, sizeof(void*) * maxBufferSize, CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
        cudaContext->memHostAlloc((void**)&mOutPtrHost, sizeof(void*) * maxBufferSize, CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
        cudaContext->memHostAlloc((void**)&mSizePtrHost, sizeof(int) * maxBufferSize, CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);

        int maxSize = 0;

        for (size_t i = 0; i < mPositionsChanged.size(); i++)
        {
            mInPtrHost[i] = mPositionsChanged[i].fabricPtr;
            mOutPtrHost[i] = mPositionsChanged[i].cudaPtr;
            mSizePtrHost[i] = (int)mPositionsChanged[i].size;
            maxSize = PxMax(maxSize, mSizePtrHost[i]);
        }

        for (size_t i = 0; i < mVelocitiesChanged.size(); i++)
        {
            mInPtrHost[i] = mVelocitiesChanged[i].fabricPtr;
            mOutPtrHost[i] = mVelocitiesChanged[i].cudaPtr;
            mSizePtrHost[i] = (int)mVelocitiesChanged[i].size;
            maxSize = PxMax(maxSize, mSizePtrHost[i]);
        }

        mPositionsChanged.clear();
        mVelocitiesChanged.clear();
    }
#endif
}

void FabricParticles::updateFabric()
{
    // pause change tracking so that we dont get transformations back
    const bool changeTrackingPaused = usdparser::UsdLoad::getUsdLoad()->isChangeTrackingPaused(mStageId.id);
    usdparser::UsdLoad::getUsdLoad()->pauseChangeTracking(mStageId.id, true);

    static const omni::fabric::set<AttrNameAndType_v2> requiredAll = { AttrNameAndType_v2(mAppliedSchemaType, mParticleSetApiToken) };
    static const omni::fabric::set<AttrNameAndType_v2> requiredAny = { AttrNameAndType_v2(mFloat4ArrayType, mPositionInvMassesToken),
                                                                    AttrNameAndType_v2(mFloat4ArrayType, mVelocitiesFloat4Token) };

    omni::fabric::StageReaderWriterId sipId = mIStageReaderWriter->get(mStageId);
    StageReaderWriter srw = mIStageReaderWriter->get(mStageId);
    PrimBucketList primBuckets = srw.findPrims(requiredAll, requiredAny);
    size_t bucketCount = primBuckets.bucketCount();

    for (size_t i = 0; i != bucketCount; i++)
    {
        gsl::span<const omni::fabric::Path> paths = srw.getPathArray(primBuckets, i);
        for (const omni::fabric::Path& path : paths)
        {
            pxr::SdfPath usdPath = omni::fabric::toSdfPath(path);
            internal::InternalParticleSet* particleSet = mParticleSets[usdPath];

            if (particleSet)
            {
                // Kick off fabric event notification that data is ready to be read
                omni::fabric::PathC primPath = omni::fabric::asInt(particleSet->mPrim.GetPrimPath());
                SpanWithTypeC positionsGpu = mIStageReaderWriter->getAttributeWrGpu(sipId, path, mPositionInvMassesToken);
                SpanWithTypeC velocitiesGpu = mIStageReaderWriter->getAttributeWrGpu(sipId, path, mVelocitiesFloat4Token);
            }
        }
    }

    usdparser::UsdLoad::getUsdLoad()->pauseChangeTracking(mStageId.id, changeTrackingPaused);
}

void FabricParticles::saveToUsd()
{
    // pause change tracking so that we dont get transformations back
    const bool changeTrackingPaused = usdparser::UsdLoad::getUsdLoad()->isChangeTrackingPaused(mStageId.id);
    usdparser::UsdLoad::getUsdLoad()->pauseChangeTracking(mStageId.id, true);

    usdparser::UsdLoad::getUsdLoad()->blockUSDUpdate(true);

    static const omni::fabric::set<AttrNameAndType_v2> requiredAll = { AttrNameAndType_v2(mAppliedSchemaType, mParticleSetApiToken) };
    static const omni::fabric::set<AttrNameAndType_v2> requiredAny = { AttrNameAndType_v2(mFloat4ArrayType, mPositionInvMassesToken),
                                                                    AttrNameAndType_v2(mFloat4ArrayType, mVelocitiesFloat4Token) };

    omni::fabric::StageReaderWriterId sipId = mIStageReaderWriter->get(mStageId);
    StageReaderWriter srw = mIStageReaderWriter->get(mStageId);
    PrimBucketList primBuckets = srw.findPrims(requiredAll, requiredAny);
    size_t bucketCount = primBuckets.bucketCount();

    for (size_t i = 0; i != bucketCount; i++)
    {
        gsl::span<const omni::fabric::Path> paths = srw.getPathArray(primBuckets, i);
        for (const omni::fabric::Path& path : paths)
        {
            pxr::SdfPath usdPath = omni::fabric::toSdfPath(path);
            internal::InternalParticleSet* particleSet = mParticleSets[usdPath];

            if (particleSet)
            {
                SpanWithTypeC positionsGpu = mIStageReaderWriter->getAttributeWrGpu(sipId, path, mPositionInvMassesToken);
                SpanWithTypeC velocitiesGpu = mIStageReaderWriter->getAttributeWrGpu(sipId, path, mVelocitiesFloat4Token);

                if (particleSet->mNumParticles)
                {
                    if (mCudaContextManager)
                    {
                        PxCudaContext* cudaContext = mCudaContextManager->getCudaContext();

                        cudaContext->memcpyDtoH(particleSet->mPositions, CUdeviceptr(*reinterpret_cast<carb::Float4**>(positionsGpu.ptr)), particleSet->mNumParticles * sizeof(carb::Float4));
                        cudaContext->memcpyDtoH(particleSet->mVelocities, CUdeviceptr(*reinterpret_cast<carb::Float4**>(velocitiesGpu.ptr)), particleSet->mNumParticles * sizeof(carb::Float4));

                        mCudaContextManager->releaseContext();
                    }
                }

                // transform particles from world space back to prim local space
                pxr::UsdGeomXform xform(particleSet->mPrim);
                pxr::GfMatrix4f worldToLocal = pxr::GfMatrix4f(xform.ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default()).GetInverse());

                pxr::UsdGeomPointBased pointBased{ particleSet->mPrim };
                pxr::UsdGeomPointInstancer pointInstancer{ particleSet->mPrim };

                {
                    pxr::UsdAttribute pointsAttr = pointBased ? pointBased.CreatePointsAttr() : pointInstancer.CreatePositionsAttr();
                    pxr::VtArray<GfVec3f> tmpPoints;
                    copyBuffer(tmpPoints, (carb::Float4*)particleSet->mPositions, particleSet->mNumParticles, worldToLocal);
                    pointsAttr.Set(tmpPoints);
                }
                {
                    pxr::UsdAttribute velocityAttr = pointBased ? pointBased.CreateVelocitiesAttr() : pointInstancer.CreateVelocitiesAttr();
                    pxr::VtArray<GfVec3f> tmpVelocities;
                    copyBuffer(tmpVelocities, (carb::Float4*)particleSet->mVelocities, particleSet->mNumParticles);
                    velocityAttr.Set(tmpVelocities);
                }
            }
        }
    }

    usdparser::UsdLoad::getUsdLoad()->blockUSDUpdate(false);
    usdparser::UsdLoad::getUsdLoad()->pauseChangeTracking(mStageId.id, changeTrackingPaused);
}

} // namespace physx
} // namespace omni
