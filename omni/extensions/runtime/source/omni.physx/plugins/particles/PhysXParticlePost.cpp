// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXParticlePost.h"

#include <PhysXTools.h>
#include <common/utilities/PrimUtilities.h>
#include <common/utilities/MemoryMacros.h>

#include "../internal/InternalParticle.h"

#include <usdLoad/LoadUsd.h>
#include <usdLoad/Particles.h>
#include <usdLoad/FabricSync.h>

#if USE_PHYSX_GPU
#include "PxIsosurfaceExtraction.h"
#include "PxAnisotropy.h"
#include "PxSmoothing.h"
#include "PxParticleNeighborhoodProvider.h"
#include "gpu/PxPhysicsGpu.h"
#endif

using namespace pxr;
using namespace carb;
using namespace ::physx;
using namespace omni::physx;
using namespace omni::physx::particles;
using namespace omni::physx::usdparser;

#if USE_PHYSX_GPU

void PostProcessCallback::onPostSolve(const PxGpuMirroredPointer<PxGpuParticleSystem>& gpuParticleSystem, CUstream stream)
{
    PxGpuParticleSystem& p = *gpuParticleSystem.mHostPtr;

    if (mResultNumParticles != nullptr)
    {
        *mResultNumParticles = p.mCommonData.mMaxParticles;
    }

    if (!mParticleSystem->mParticleDataAvailable)
    {
        return;
    }

    if (mResultNumParticles != nullptr &&  p.mCommonData.mMaxParticles > 0)
    {
        if (mAnisotropyGenerator) 
        {
            mAnisotropyGenerator->generateAnisotropy(gpuParticleSystem.mDevicePtr, p.mCommonData.mMaxParticles, stream);
        }

        if (mSmoothedPositionGenerator)
        {
            mSmoothedPositionGenerator->generateSmoothedPositions(gpuParticleSystem.mDevicePtr, p.mCommonData.mMaxParticles, stream);
        }

        if (mIsosurfaceExtractor)
        {
            PxVec4* srcPositionsDevice = (PxVec4*)p.mUnsortedPositions_InvMass;
            PxVec4* srcAnisotropy1Device = nullptr;
            PxVec4* srcAnisotropy2Device = nullptr;
            PxVec4* srcAnisotropy3Device = nullptr;

            if (mSmoothedPositionGenerator)
            {
                srcPositionsDevice = mSmoothedPositionGenerator->getSmoothedPositionsDevicePointer();
            }

            if (mAnisotropyGenerator)
            {
                srcAnisotropy1Device = mAnisotropyGenerator->getAnisotropy1DevicePointer();
                srcAnisotropy2Device = mAnisotropyGenerator->getAnisotropy2DevicePointer();
                srcAnisotropy3Device = mAnisotropyGenerator->getAnisotropy3DevicePointer();
            }

            mIsosurfaceExtractor->extractIsosurface(srcPositionsDevice, p.mCommonData.mNumParticles, stream, p.mUnsortedPhaseArray, PxParticlePhaseFlag::eParticlePhaseFluid,
                /*p.mActiveArray*/ nullptr, srcAnisotropy1Device, srcAnisotropy2Device, srcAnisotropy3Device, p.mCommonData.mParticleContactDistance);
        }

        // this would potentially be the place for Vec4->Vec3 and transforms on GPU.
    }

    // kickoff the particle download
    if (mParticleSystem)
    {
        mParticleSystem->fetchParticles(stream);
    }

    // sync on this stream is called in updateParticleTransforms.
}

void PostProcessCallback::onBegin(const PxGpuMirroredPointer<PxGpuParticleSystem>& gpuParticleSystem, CUstream stream)
{
    if (mParticleSystem)
    {
        mParticleSystem->uploadParticles(stream);
    }
}
        
#endif

namespace
{
    bool isFluidParticleSet(const UsdPrim& particleSetPrim)
    {
        PhysxSchemaPhysxParticleSetAPI particleSetApi(particleSetPrim);
        if (!particleSetApi)
        {
            return false;
        }
        bool isFluid;
        particleSetApi.GetFluidAttr().Get(&isFluid);
        return isFluid;
    }

    UsdAttribute getParticleSetPointsAttr(const UsdPrim& particleSetPrim)
    {
        UsdAttribute pointsAttr;
        UsdGeomPointBased pointBased = UsdGeomPointBased(particleSetPrim);
        if (pointBased)
        {
            pointsAttr = pointBased.GetPointsAttr();
        }

        UsdGeomPointInstancer pointInstancer = UsdGeomPointInstancer(particleSetPrim);
        if (pointInstancer)
        {
            pointsAttr = pointInstancer.GetPositionsAttr();
        }
        return pointsAttr;
    }

    VtArray<GfVec3f> getParticleSetPoints(const UsdPrim& particleSetPrim)
    {
        VtArray<GfVec3f> dstPoints;
        PhysxSchemaPhysxParticleSetAPI particleSetApi(particleSetPrim);
        if (!particleSetApi)
        {
            return dstPoints;
        }

        UsdAttribute simPointsAttr = particleSetApi.GetSimulationPointsAttr();
        if (simPointsAttr.HasAuthoredValue())
        {
            simPointsAttr.Get(&dstPoints);
        }
        else
        {
            UsdAttribute pointsAttr = getParticleSetPointsAttr(particleSetPrim);
            pointsAttr.Get(&dstPoints);
        }
        return dstPoints;
    }

    void appendParticleSetPoints(std::vector<PxVec4>& dstPoints, UsdPrim& particleSetPrim)
    {
        VtArray<GfVec3f> srcPoints = getParticleSetPoints(particleSetPrim);
        GfMatrix4d localToWorld = UsdGeomXform(particleSetPrim).ComputeLocalToWorldTransform(UsdTimeCode::Default());
        dstPoints.reserve(dstPoints.size() + srcPoints.size());
        for (size_t p = 0; p < srcPoints.size(); ++p)
        {
            const GfVec3f& src = localToWorld.Transform(srcPoints[p]);
            dstPoints.push_back({src[0], src[1], src[2], 0.0f});
        }
    }

    void updatePointInstancerScaleOrient(UsdGeomPointInstancer& pointInstancer, const PxVec4* anisotropyQ1, const PxVec4* anisotropyQ2, const PxVec4* anisotropyQ3, const size_t numPoints, const float contactDistanceInv)
    {
        VtArray<GfVec3f> dstScales(numPoints);
        VtArray<GfQuath> dstOrientations(numPoints);
        for (size_t i = 0; i < numPoints; ++i)
        {
            PxVec4 q1 = anisotropyQ1[i];
            PxVec4 q2 = anisotropyQ2[i];
            PxVec4 q3 = anisotropyQ3[i];
            // AD: I have no idea where this 4x factor is coming from, but it seems to be important.
            dstScales[i] = { 4.0f * contactDistanceInv * q1[3], 4.0f * contactDistanceInv * q2[3], 4.0f * contactDistanceInv * q3[3]};
            PxQuat q = PxQuat(PxMat33(q1.getXYZ(), q2.getXYZ(), q3.getXYZ()));
            dstOrientations[i] = GfQuath(q.w, q.x, q.y, q.z);
        }
        pointInstancer.GetScalesAttr().Set(dstScales);
        pointInstancer.GetOrientationsAttr().Set(dstOrientations);
    }

    void restorePointInstancerScaleOrient(UsdGeomPointInstancer& pointInstancer)
    {
        UsdLoad::getUsdLoad()->blockUSDUpdate(true);
        UsdLoad::getUsdLoad()->pauseChangeTracking(OmniPhysX::getInstance().getStageId(), true);

        VtArray<GfVec3f> scales;
        VtArray<GfQuath> orientations;
        pointInstancer.GetScalesAttr().Get(&scales);
        pointInstancer.GetOrientationsAttr().Get(&orientations);
        for (size_t i = 0; i < scales.size(); ++i)
        {
            scales[i] = GfVec3f(1.0f);
        }
        for (size_t i = 0; i < orientations.size(); ++i)
        {
            orientations[i] = GfQuath::GetIdentity();
        }
        pointInstancer.GetScalesAttr().Set(scales);
        pointInstancer.GetOrientationsAttr().Set(orientations);

        UsdLoad::getUsdLoad()->blockUSDUpdate(false);
        UsdLoad::getUsdLoad()->pauseChangeTracking(OmniPhysX::getInstance().getStageId(), false);
    }

    void restoreSmoothedPoints(PhysxSchemaPhysxParticleSetAPI& particleSetAPI)
    {
        UsdLoad::getUsdLoad()->blockUSDUpdate(true);
        UsdLoad::getUsdLoad()->pauseChangeTracking(OmniPhysX::getInstance().getStageId(), true);

        UsdPrim usdPrim = particleSetAPI.GetPrim();
        UsdAttribute simPointsAttr = particleSetAPI.GetSimulationPointsAttr();
        if (simPointsAttr.HasAuthoredValue())
        {
            VtArray<GfVec3f> srcPoints;
            simPointsAttr.Get(&srcPoints);

            UsdAttribute dstPointsAttr;
            if (usdPrim.IsA<UsdGeomPoints>())
            {
                dstPointsAttr = UsdGeomPoints(usdPrim).GetPointsAttr();
            }
            else if (usdPrim.IsA<UsdGeomPointInstancer>())
            {
                dstPointsAttr = UsdGeomPointInstancer(usdPrim).GetPositionsAttr();
            }
            dstPointsAttr.Set(srcPoints);
            simPointsAttr.Clear();
        }

        UsdLoad::getUsdLoad()->blockUSDUpdate(false);
        UsdLoad::getUsdLoad()->pauseChangeTracking(OmniPhysX::getInstance().getStageId(), false);
    }

    void updateParticleSetPoints(VtArray<GfVec3f>& points, const PxVec4* positions, const GfMatrix4d& transform, const size_t numPoints)
    {
        points.resize(numPoints);
        for (size_t i = 0; i < numPoints; ++i)
        {
            const PxVec4& src = positions[i];
            points[i] = transform.Transform(GfVec3f(src.x, src.y, src.z));
        }
    }
}

ParticlePostprocess::ParticlePostprocess(const SdfPath particleSystemPath)
    : mParticleSystemPath(particleSystemPath)
#if USE_PHYSX_GPU
    , mParticlesPreview(NULL)
    , mAnisotropyQ1(NULL)
    , mAnisotropyQ2(NULL)
    , mAnisotropyQ3(NULL)
    , mSmoothedPositions(NULL)
    , mIsosurfaceVertices(NULL)
    , mIsosurfaceTriangleIndices(NULL)
    , mIsosurfaceNormals(NULL)
#endif
{
}

ParticlePostprocess::~ParticlePostprocess()
{
    //disable all postprocesses
    setPostprocessFlags(0);
    setParent(nullptr);

#if USE_PHYSX_GPU
    if (mParticlesPreview)
    {
        PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
        if (cudaContextManager)
        {
            PX_DEVICE_FREE(cudaContextManager, mParticlesPreview);
        }
    }
    mMaxParticlesPreview = 0;
#endif
}

void ParticlePostprocess::setPostprocessFlags(uint32_t newFlags)
{
#if USE_PHYSX_GPU
    const uint32_t oldFlags = mFlags;
    const uint32_t diffFlags = oldFlags ^ newFlags;
    const uint32_t loweredFlags = diffFlags & oldFlags;
    const uint32_t raisedFlags = diffFlags & ~oldFlags;

    if (loweredFlags & ParticlePostFlag::eAnisotropy)
    {
        restoreScaleOrient();
        releaseAnisotropyGenerator();
    }
    else if (raisedFlags & ParticlePostFlag::eAnisotropy)
    {
    }

    if (loweredFlags & ParticlePostFlag::eSmoothing)
    {
        restorePoints();
        releaseSmoothedPositionGenerator();
    }
    else if (raisedFlags & ParticlePostFlag::eSmoothing)
    {
    }

    if (loweredFlags & ParticlePostFlag::eIsosurface)
    {
        releaseSessionIsosurfaceMesh();
        releaseIsosurfaceExtractor();
    }
    else if (raisedFlags & ParticlePostFlag::eIsosurface)
    {
        createSessionIsosurfaceMesh();
    }

    mFlags = newFlags;
#endif
}

void ParticlePostprocess::updateGeneratorsCallback()
{
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    if (stage && mParent)
    {
        // update max particles
        mMaxParticlesCallback = 0;
        if (mParent)
        {
            for (size_t i = 0; i < mParent->mCloths.size(); ++i)
            {
                mMaxParticlesCallback += mParent->mCloths[i]->mNumParticles;
            }

            for (size_t i = 0; i < mParent->mParticleSets.size(); ++i)
            {
                mMaxParticlesCallback += mParent->mParticleSets[i]->mMaxParticles;
            }
        }

        UsdPrim particleSystemPrim = stage->GetPrimAtPath(mParticleSystemPath);
        if (hasAnisotropy())
        {
            updateAnisotropyGenerator(mMaxParticlesCallback);
        }

        if (hasSmoothing())
        {
            updateSmoothedPositionGenerator(mMaxParticlesCallback);
        }

        if (hasIsosurface())
        {
            updateIsosurfaceExtractor(mMaxParticlesCallback);
        }

#if USE_PHYSX_GPU
        PostProcessCallback* callback = mParent->mCallback;
        callback->initialize(mAnisotropyGenerator, mSmoothedPositionGenerator, mIsosurfaceExtractor, &mResultNumParticles);
#endif
    }
}

void ParticlePostprocess::setParent(omni::physx::internal::InternalPbdParticleSystem* parent)
{
#if USE_PHYSX_GPU
    if (mParent)
    {
        PostProcessCallback* callback = mParent->mCallback;
        callback->initialize(nullptr, nullptr, nullptr, nullptr);
        mParent = nullptr;
    }
    mParent = parent;
#endif
}

bool ParticlePostprocess::parseParticleContactOffset(float& particleContactOffset)
{
    UsdPrim particleSystemPrim = OmniPhysX::getInstance().getStage()->GetPrimAtPath(mParticleSystemPath);
    if (particleSystemPrim)
    {
        AttachedStage attachedStage(OmniPhysX::getInstance().getStage());
        ParticleSystemDesc* particleSystemDesc = ParseParticleSystem(attachedStage, particleSystemPrim);
        if (particleSystemDesc)
        {
            particleContactOffset = particleSystemDesc->particleContactOffset;
            ICE_FREE(particleSystemDesc);
            return true;
        }
    }
    return false;
}

void ParticlePostprocess::createNeighborhoodProvider(float cellSize, const PxU32 maxParticles)
{
#if USE_PHYSX_GPU
    if (!mNeighborhoodProvider && maxParticles > 0)
    {
        PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
        if (cudaContextManager)
        {
            mNeighborhoodProvider = PxGetPhysicsGpu()->createParticleNeighborhoodProvider(cudaContextManager, maxParticles, cellSize);
        }
    }
#endif
}

void ParticlePostprocess::updateAnisotropyGenerator(const PxU32 maxParticles)
{
#if USE_PHYSX_GPU
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    UsdPrim prim = stage->GetPrimAtPath(mParticleSystemPath);
    if (!prim)
    {
        return;
    }

    PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
    if (!cudaContextManager)
    {
        return;
    }

    ParticleAnisotropyDesc* desc = ParseParticleAnisotropy(prim);
    if (!desc)
    {
        return;
    }

    if (mAnisotropyGenerator && (maxParticles == 0))
    {
        releaseAnisotropyGenerator();
    }

    bool needNew = !mAnisotropyGenerator && maxParticles > 0;
    if (needNew)
    {
        mAnisotropyGenerator = PxGetPhysicsGpu()->createAnisotropyGenerator(cudaContextManager, maxParticles, desc->scale, desc->min, desc->max);
        if (mParent)
        {
            PostProcessCallback* callback = mParent->mCallback;
            callback->initialize(mAnisotropyGenerator, mSmoothedPositionGenerator, mIsosurfaceExtractor, &mResultNumParticles);
        }
    }

    if (mAnisotropyGenerator)
    {
        mAnisotropyGenerator->setAnisotropyScale(desc->scale);
        mAnisotropyGenerator->setAnisotropyMin(desc->min);
        mAnisotropyGenerator->setAnisotropyMax(desc->max);
        mAnisotropyGenerator->setEnabled(desc->enableAnisotropy);

        PxU32 generatorMaxParticles = mAnisotropyGenerator->getMaxParticles();
        if (needNew || maxParticles != generatorMaxParticles)
        {
            PX_PINNED_HOST_FREE(cudaContextManager, mAnisotropyQ1);
            PX_PINNED_HOST_FREE(cudaContextManager, mAnisotropyQ2);
            PX_PINNED_HOST_FREE(cudaContextManager, mAnisotropyQ3);
            mAnisotropyQ1 = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, maxParticles);
            mAnisotropyQ2 = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, maxParticles);
            mAnisotropyQ3 = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, maxParticles);
            mAnisotropyGenerator->setResultBufferHost(mAnisotropyQ1, mAnisotropyQ2, mAnisotropyQ3);
            mAnisotropyGenerator->setMaxParticles(maxParticles);
        }
    }

    ICE_FREE(desc);
#endif
}

void ParticlePostprocess::releaseAnisotropyGenerator()
{
#if USE_PHYSX_GPU
    if (mAnisotropyGenerator)
    {
        PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
        if (cudaContextManager)
        {
            if (mAnisotropyQ1 || mAnisotropyQ2 || mAnisotropyQ3)
            {
                PX_PINNED_HOST_FREE(cudaContextManager, mAnisotropyQ1);
                PX_PINNED_HOST_FREE(cudaContextManager, mAnisotropyQ2);
                PX_PINNED_HOST_FREE(cudaContextManager, mAnisotropyQ3);
            }
        }
        SAFE_RELEASE(mAnisotropyGenerator);

        if (mParent)
        {
            PostProcessCallback* callback = mParent->mCallback;
            callback->initialize(mAnisotropyGenerator, mSmoothedPositionGenerator, mIsosurfaceExtractor, &mResultNumParticles);
        }
    }

    if (!mSmoothedPositionGenerator)
    {
        SAFE_RELEASE(mNeighborhoodProvider);
    }
#endif
}

void ParticlePostprocess::updateSmoothedPositionGenerator(const PxU32 maxParticles)
{
#if USE_PHYSX_GPU
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    UsdPrim prim = stage->GetPrimAtPath(mParticleSystemPath);
    if (!prim)
    {
        return;
    }

    PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
    if (!cudaContextManager)
    {
        return;
    }

    ParticleSmoothingDesc* desc = ParseParticleSmoothing(prim);
    if (!desc)
    {
        return;
    }

    if (mSmoothedPositionGenerator && (maxParticles == 0))
    {
        releaseSmoothedPositionGenerator();
    }

    bool needNew = !mSmoothedPositionGenerator && maxParticles > 0;
    if (needNew)
    {
        mSmoothedPositionGenerator = PxGetPhysicsGpu()->createSmoothedPositionGenerator(cudaContextManager, maxParticles, desc->strength);
        if (mParent)
        {
            PostProcessCallback* callback = mParent->mCallback;
            callback->initialize(mAnisotropyGenerator, mSmoothedPositionGenerator, mIsosurfaceExtractor, &mResultNumParticles);
        }
    }

    if (mSmoothedPositionGenerator)
    {
        mSmoothedPositionGenerator->setSmoothing(desc->strength);
        mSmoothedPositionGenerator->setEnabled(desc->enableSmoothing);

        PxU32 generatorMaxParticles = mSmoothedPositionGenerator->getMaxParticles();
        if (needNew || maxParticles != generatorMaxParticles)
        {
            PX_PINNED_HOST_FREE(cudaContextManager, mSmoothedPositions);
            mSmoothedPositions = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, maxParticles);
            mSmoothedPositionGenerator->setResultBufferHost(mSmoothedPositions);
            mSmoothedPositionGenerator->setMaxParticles(maxParticles);
        }
    }

    ICE_FREE(desc);
#endif
}

void ParticlePostprocess::releaseSmoothedPositionGenerator()
{
#if USE_PHYSX_GPU
    if (mSmoothedPositionGenerator)
    {
        PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
        if (cudaContextManager)
        {
            PX_PINNED_HOST_FREE(cudaContextManager, mSmoothedPositions);
        }
        SAFE_RELEASE(mSmoothedPositionGenerator);

        if (mParent)
        {
            PostProcessCallback* callback = mParent->mCallback;
            callback->initialize(mAnisotropyGenerator, mSmoothedPositionGenerator, mIsosurfaceExtractor, &mResultNumParticles);
        }
    }

    if (!mAnisotropyGenerator)
    {
        SAFE_RELEASE(mNeighborhoodProvider);
    }
#endif
}

void ParticlePostprocess::updateIsosurfaceExtractor(const PxU32 maxParticles)
{
#if USE_PHYSX_GPU
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    UsdPrim prim = stage->GetPrimAtPath(mParticleSystemPath);
    if (!prim)
    {
        return;
    }

    PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
    if (!cudaContextManager)
    {
        return;
    }

    ParticleIsosurfaceDesc* desc = ParseParticleIsosurface(prim);
    if (!desc)
    {
        return;
    }

    if (mIsosurfaceExtractor)
    {
        if (mMaxNumIsosurfaceSubgrids != desc->maxNumIsosurfaceSubgrids ||
            mIsosurfaceGridSpacing != desc->gridSpacing ||
            maxParticles == 0)
        {
            releaseIsosurfaceExtractor();
        }
    }

    bool needNew = !mIsosurfaceExtractor && maxParticles > 0;
    if (needNew)
    {
        PxSparseGridParams sgIsosurfaceParams;
        sgIsosurfaceParams.subgridSizeX = 16;
        sgIsosurfaceParams.subgridSizeY = 16;
        sgIsosurfaceParams.subgridSizeZ = 16;
        sgIsosurfaceParams.maxNumSubgrids = desc->maxNumIsosurfaceSubgrids;
        sgIsosurfaceParams.gridSpacing = desc->gridSpacing;

        PxIsosurfaceParams isosurfaceParams;
        isosurfaceParams.particleCenterToIsosurfaceDistance = desc->surfaceDistance;
        setIsosurfaceGridFilteringPasses(isosurfaceParams, desc->gridFilteringPasses);
        isosurfaceParams.gridSmoothingRadius = desc->gridSmoothingRadius;
        isosurfaceParams.numMeshSmoothingPasses = desc->numMeshSmoothingPasses;
        isosurfaceParams.numMeshNormalSmoothingPasses = desc->numMeshNormalSmoothingPasses;

        mIsosurfaceExtractor = PxGetPhysicsGpu()->createSparseGridIsosurfaceExtractor(cudaContextManager, sgIsosurfaceParams, isosurfaceParams,
            maxParticles, desc->maxIsosurfaceVertices, desc->maxIsosurfaceTriangles);

        mMaxNumIsosurfaceSubgrids = desc->maxNumIsosurfaceSubgrids;
        mIsosurfaceGridSpacing = desc->gridSpacing;

        mIsosurfaceExtractor->setEnabled(desc->enableIsosurface);
        if (mParent)
        {
            PostProcessCallback* callback = mParent->mCallback;
            callback->initialize(mAnisotropyGenerator, mSmoothedPositionGenerator, mIsosurfaceExtractor, &mResultNumParticles);
        }
    }

    if (mIsosurfaceExtractor)
    {
        PxIsosurfaceParams isosurfaceParams;
        isosurfaceParams.particleCenterToIsosurfaceDistance = desc->surfaceDistance;
        setIsosurfaceGridFilteringPasses(isosurfaceParams, desc->gridFilteringPasses);
        isosurfaceParams.gridSmoothingRadius = desc->gridSmoothingRadius;
        isosurfaceParams.numMeshSmoothingPasses = desc->numMeshSmoothingPasses;
        isosurfaceParams.numMeshNormalSmoothingPasses = desc->numMeshNormalSmoothingPasses;
        mIsosurfaceExtractor->setIsosurfaceParams(isosurfaceParams);
        mIsosurfaceExtractor->setEnabled(desc->enableIsosurface);

        PxU32 extractorMaxParticles = mIsosurfaceExtractor->getMaxParticles();
        if (maxParticles > extractorMaxParticles)
        {
            mIsosurfaceExtractor->setMaxParticles(maxParticles);
        }

        PxU32 extractorMaxVertices = mIsosurfaceExtractor->getMaxVertices();
        PxU32 extractorMaxTriangles = mIsosurfaceExtractor->getMaxTriangles();
        if (needNew || extractorMaxVertices < PxU32(desc->maxIsosurfaceVertices) || extractorMaxTriangles < PxU32(desc->maxIsosurfaceTriangles))
        {
            PX_PINNED_HOST_FREE(cudaContextManager, mIsosurfaceVertices);
            PX_PINNED_HOST_FREE(cudaContextManager, mIsosurfaceTriangleIndices);
            PX_PINNED_HOST_FREE(cudaContextManager, mIsosurfaceNormals);
            mIsosurfaceVertices = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, desc->maxIsosurfaceVertices);
            mIsosurfaceTriangleIndices = PX_PINNED_HOST_ALLOC_T(PxU32, cudaContextManager, desc->maxIsosurfaceTriangles * 3);
            mIsosurfaceNormals = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, desc->maxIsosurfaceVertices);
            mIsosurfaceExtractor->setResultBufferHost(mIsosurfaceVertices, mIsosurfaceTriangleIndices, mIsosurfaceNormals);
            mIsosurfaceExtractor->setMaxVerticesAndTriangles(desc->maxIsosurfaceVertices, desc->maxIsosurfaceTriangles);
        }
    }

    ICE_FREE(desc);
#endif
}

void ParticlePostprocess::releaseIsosurfaceExtractor()
{
#if USE_PHYSX_GPU
    if (mIsosurfaceExtractor)
    {
        if (mIsosurfaceVertices)
        {
            PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
            if (cudaContextManager)
            {
                PX_PINNED_HOST_FREE(cudaContextManager, mIsosurfaceVertices);
                PX_PINNED_HOST_FREE(cudaContextManager, mIsosurfaceTriangleIndices);
                PX_PINNED_HOST_FREE(cudaContextManager, mIsosurfaceNormals);
            }
        }
        SAFE_RELEASE(mIsosurfaceExtractor);

        if (mParent)
        {
            PostProcessCallback* callback = mParent->mCallback;
            callback->initialize(mAnisotropyGenerator, mSmoothedPositionGenerator, mIsosurfaceExtractor, &mResultNumParticles);
        }
    }
#endif
}

void ParticlePostprocess::updateFromPoints(const PxVec4* points, PxU32 numPoints)
{
#if USE_PHYSX_GPU
    CARB_ASSERT(numPoints > 0);

    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    UsdPrim usdPrim = stage->GetPrimAtPath(mParticleSystemPath);
    if (!usdPrim)
    {
        return;
    }

    PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
    if (!cudaContextManager)
    {
        return;
    }

    PxU32 maxParticles = (mMaxParticlesCallback > numPoints) ? mMaxParticlesCallback : numPoints;

    // get particle contact offset from particle system
    float particleContactOffset;
    if (!parseParticleContactOffset(particleContactOffset))
    {
        return;
    }

    if (hasAnisotropy())
    {
        updateAnisotropyGenerator(maxParticles);
        if (!mAnisotropyGenerator)
        {
            return;
        }
    }

    if (hasSmoothing())
    {
        updateSmoothedPositionGenerator(maxParticles);
        if (!mSmoothedPositionGenerator)
        {
            return;
        }
    }

    if (hasIsosurface())
    {
        updateIsosurfaceExtractor(maxParticles);
        if (!mIsosurfaceExtractor)
        {
            return;
        }
    }

    CUstream defaultStream = 0;
    cudaContextManager->acquireContext();

    // adjust preview particle capacity
    if (mMaxParticlesPreview != maxParticles)
    {
        PX_DEVICE_FREE(cudaContextManager, mParticlesPreview);
        mParticlesPreview = PX_DEVICE_ALLOC_T(PxVec4, cudaContextManager, maxParticles);
        mMaxParticlesPreview = maxParticles;
    }

    // upload preview particles
    cudaContextManager->getCudaContext()->memcpyHtoDAsync(CUdeviceptr(mParticlesPreview), points, numPoints * sizeof(PxVec4), defaultStream);

    if (hasAnisotropy() || hasSmoothing())
    {
        float cellSize = 2.0f*particleContactOffset;

        if (!mNeighborhoodProvider)
        {
            createNeighborhoodProvider(cellSize, maxParticles);
        }

        if (mNeighborhoodProvider)
        {
            PxU32 providerMaxParticles = mNeighborhoodProvider->getMaxParticles();
            if (maxParticles != providerMaxParticles)
            {
                mNeighborhoodProvider->setMaxParticles(maxParticles);
            }

            float providerCellSize = mNeighborhoodProvider->getCellSize();
            if (cellSize != providerCellSize)
            {
                mNeighborhoodProvider->setCellProperties(mNeighborhoodProvider->getMaxGridCells(), cellSize);
            }

            mNeighborhoodProvider->buildNeighborhood(mParticlesPreview, numPoints, defaultStream);
        }
    }

    if (mAnisotropyGenerator && mNeighborhoodProvider)
    {
        mAnisotropyGenerator->generateAnisotropy(mParticlesPreview, *mNeighborhoodProvider, numPoints, particleContactOffset, defaultStream);
    }

    if (mSmoothedPositionGenerator && mNeighborhoodProvider)
    {
        mSmoothedPositionGenerator->generateSmoothedPositions(mParticlesPreview, *mNeighborhoodProvider, numPoints, particleContactOffset, defaultStream);
    }

    if (mIsosurfaceExtractor)
    {
        PxVec4* srcPositionsDevice = mParticlesPreview;
        PxVec4* srcAnisotropy1Device = nullptr;
        PxVec4* srcAnisotropy2Device = nullptr;
        PxVec4* srcAnisotropy3Device = nullptr;
        float anisoFactor = 1.0f;

        if (hasSmoothing())
        {
            srcPositionsDevice = mSmoothedPositionGenerator->getSmoothedPositionsDevicePointer();
        }

        if (hasAnisotropy())
        {
            srcAnisotropy1Device = mAnisotropyGenerator->getAnisotropy1DevicePointer();
            srcAnisotropy2Device = mAnisotropyGenerator->getAnisotropy2DevicePointer();
            srcAnisotropy3Device = mAnisotropyGenerator->getAnisotropy3DevicePointer();
            anisoFactor = 2.0f * particleContactOffset;
        }

        mIsosurfaceExtractor->extractIsosurface(srcPositionsDevice, numPoints, defaultStream,
            nullptr, 4194304U, nullptr, srcAnisotropy1Device, srcAnisotropy2Device, srcAnisotropy3Device, anisoFactor);
    }

    cudaContextManager->getCudaContext()->streamSynchronize(defaultStream);
    cudaContextManager->releaseContext();
#endif
}

void ParticlePostprocess::addParticleSet(const SdfPath& particleSetPath)
{
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    UsdPrim particleSetPrim = stage->GetPrimAtPath(particleSetPath);

    {
        if (isFluidParticleSet(particleSetPrim))
        {
            mFluidParticleSetsPreview.insert(particleSetPath);
            if (hasIsosurface())
            {
                // make particle set invisible by setting purpose = proxy on session layer
                // we don't need to store this to file, and retains visibility attribute for user (e.g. to selectively allow for disabling of particle visualization)
                ScopedLayerEdit scopedSessionLayerEdit(stage, stage->GetSessionLayer());
                UsdGeomImageable geomImageable(particleSetPrim);
                geomImageable.GetPurposeAttr().Set(UsdGeomTokens.Get()->proxy);
            }
        }
    }
}

void ParticlePostprocess::removeParticleSet(const SdfPath& particleSetPath)
{
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    auto fit = mFluidParticleSetsPreview.find(particleSetPath);
    if (fit != mFluidParticleSetsPreview.end())
    {
        if (stage)
        {
            if (hasIsosurface())
            {
                // make particles visible again by removing proxy purpose attribute from session layer
                ScopedLayerEdit scopedSessionLayerEdit(stage, stage->GetSessionLayer());
                UsdGeomImageable geomImageable(stage->GetPrimAtPath(particleSetPath));
                geomImageable.GetPurposeAttr().Clear();
            }

            if (hasAnisotropy())
            {
                UsdGeomPointInstancer pointInstancer(stage->GetPrimAtPath(particleSetPath));
                if (pointInstancer)
                {
                    restorePointInstancerScaleOrient(pointInstancer);
                }
            }

            if (hasSmoothing())
            {
                PhysxSchemaPhysxParticleSetAPI particleSetAPI = PhysxSchemaPhysxParticleSetAPI::Get(stage, particleSetPath);
                if (particleSetAPI)
                {
                    restoreSmoothedPoints(particleSetAPI);
                }
            }
        }
        mFluidParticleSetsPreview.erase(fit);
    }
}

std::vector<ParticlePostprocess::ParticleSet>& ParticlePostprocess::getTmpParticleSets()
{
    mTmpParticleSets.clear();
    if (mParent)
    {
        for (size_t s = 0; s < mParent->mParticleSets.size(); ++s)
        {
            internal::InternalParticleSet* internalParticleSet = mParent->mParticleSets[s];
            if (internalParticleSet->mFluid)
            {
                ParticleSet particleSet;
                particleSet.internal = internalParticleSet;
                particleSet.usdPrim = internalParticleSet->mPrim;
                particleSet.srcOffset = internalParticleSet->mParticleBuffer->getFlatListStartIndex();
                particleSet.srcCount = internalParticleSet->mNumParticles;
                mTmpParticleSets.push_back(particleSet);
            }
        }
    }
    else
    {
        UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
        if (stage)
        {
            uint32_t srcOffset = 0;
            for (SdfPath setPath : mFluidParticleSetsPreview)
            {
                UsdPrim usdPrim = stage->GetPrimAtPath(setPath);
                if (usdPrim)
                {
                    UsdAttribute dstPointsAttr = getParticleSetPointsAttr(usdPrim);
                    VtArray<GfVec3f> dstPoints;
                    dstPointsAttr.Get(&dstPoints);
                    uint32_t count = uint32_t(dstPoints.size());

                    ParticleSet particleSet;
                    particleSet.internal = nullptr;
                    particleSet.usdPrim = usdPrim;
                    particleSet.srcOffset = srcOffset;
                    particleSet.srcCount = count;
                    mTmpParticleSets.push_back(particleSet);

                    srcOffset += count;
                }
            }
        }
    }

    return mTmpParticleSets;
}

void ParticlePostprocess::updatePreview()
{
#if USE_PHYSX_GPU
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    std::vector<PxVec4> points;
    std::vector<ParticleSet>& particleSets = getTmpParticleSets();

    for (size_t s = 0; s < particleSets.size(); ++s)
    {
        ParticleSet& particleSet = particleSets[s];
        if (mParent)
        {
            size_t prevSize = points.size();
            points.resize(prevSize + particleSet.internal->mNumParticles);
            PxVec4* dst = points.data() + prevSize;
            std::memcpy(dst, particleSet.internal->mPositions, sizeof(PxVec4)*particleSet.internal->mNumParticles);
        }
        else
        {
            appendParticleSetPoints(points, particleSet.usdPrim);
        }
    }

    mResultNumParticles =  PxU32(points.size());
    if (mResultNumParticles > 0)
    {
        updateFromPoints(points.data(), mResultNumParticles);
    }

    //update the geometry
    {
        UsdLoad::getUsdLoad()->blockUSDUpdate(true);
        UsdLoad::getUsdLoad()->pauseChangeTracking(OmniPhysX::getInstance().getStageId(), true);

        if (hasIsosurface())
        {
            updateMesh();
        }
        else
        {
            if (hasAnisotropy())
            {
                updateScaleOrient();
            }

            if (hasSmoothing())
            {
                updatePoints();
            }
        }

        UsdLoad::getUsdLoad()->blockUSDUpdate(false);
        UsdLoad::getUsdLoad()->pauseChangeTracking(OmniPhysX::getInstance().getStageId(), false);
    }
#endif
}

void ParticlePostprocess::syncPostSolveWork()
{
    if (mParent && mParent->mPS)
    {
        PxScene* scene = mParent->mPS->getScene();
        if (scene)
        {
            scene->fetchResultsParticleSystem();
        }
    }
}

void ParticlePostprocess::getAnisotropy(::physx::PxVec4*& anisotropyQ1, ::physx::PxVec4*& anisotropyQ2, ::physx::PxVec4*& anisotropyQ3)
{
#if USE_PHYSX_GPU
    if (hasAnisotropy())
    {
        anisotropyQ1 = mAnisotropyQ1;
        anisotropyQ2 = mAnisotropyQ2;
        anisotropyQ3 = mAnisotropyQ3;
    }
    else
#endif
    {
        anisotropyQ1 = nullptr;
        anisotropyQ2 = nullptr;
        anisotropyQ3 = nullptr;
    }
}

::physx::PxVec4* ParticlePostprocess::getSmoothedPositions()
{
#if USE_PHYSX_GPU
    if (hasSmoothing())
    {
        return mSmoothedPositions;
    }
    else
#endif
    {
        return nullptr;
    }
}

void ParticlePostprocess::updateMesh()
{
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    ScopedLayerEdit scopedSessionLayerEdit(stage, stage->GetSessionLayer());

    UsdGeomMesh mesh(stage->GetPrimAtPath(mIsosurfaceMeshPath));
    if (!mesh)
    {
        return;
    }

#if USE_PHYSX_GPU
    PxU32 numVertices = 0;
    PxU32 numTris = 0;

    if (mIsosurfaceExtractor && mResultNumParticles > 0)
    {
        numVertices = mIsosurfaceExtractor->getNumVertices();
        numTris = mIsosurfaceExtractor->getNumTriangles();
    }

    if (numVertices > 0)
    {
        VtArray<int> tmpVertexCounts;
        mesh.GetFaceVertexCountsAttr().Get(&tmpVertexCounts);
        const size_t oldSize = tmpVertexCounts.size();
        tmpVertexCounts.resize(numTris);
        for (size_t i = oldSize; i < numTris; ++i)
        {
            tmpVertexCounts[i] = 3;
        }

        VtArray<int> tmpVertexIndices(3 * numTris);
        PxU32* tris = mIsosurfaceTriangleIndices;
        for (size_t i = 0; i < 3 * numTris; ++i)
        {
            tmpVertexIndices[i] = tris[i];
        }

        VtArray<GfVec3f> tmpPoints(numVertices);
        PxVec4* verts = mIsosurfaceVertices;
        for (PxU32 i = 0; i < numVertices; ++i)
        {
            const PxVec4& v = verts[i];
            tmpPoints[i] = GfVec3f(v.x, v.y, v.z);
        }

        VtArray<GfVec3f> tmpNormals(numVertices);
        PxVec4* normals = mIsosurfaceNormals;
        for (PxU32 i = 0; i < numVertices; ++i)
        {
            const PxVec4& n = normals[i];
            tmpNormals[i] = GfVec3f(n.x, n.y, n.z);
        }

        mesh.GetPointsAttr().Set(tmpPoints);
        mesh.GetNormalsAttr().Set(tmpNormals);
        mesh.GetFaceVertexCountsAttr().Set(tmpVertexCounts);
        mesh.GetFaceVertexIndicesAttr().Set(tmpVertexIndices);
    }
    else
#endif
    {
        VtArray<int> tmpVertexCounts = { 3 };
        VtArray<int> tmpVertexIndices = { 0, 0, 0 };
        VtArray<GfVec3f> tmpVec3 = { GfVec3f(0.0f) };
        mesh.GetPointsAttr().Set(tmpVec3);
        mesh.GetNormalsAttr().Set(tmpVec3);
        mesh.GetFaceVertexCountsAttr().Set(tmpVertexCounts);
        mesh.GetFaceVertexIndicesAttr().Set(tmpVertexIndices);
    }

    {
        flushUsdToFabric(stage, false);
    }
}


void ParticlePostprocess::updateScaleOrient()
{
#if USE_PHYSX_GPU
    const PxVec4* srcAnisotropyQ1 = mAnisotropyQ1;
    const PxVec4* srcAnisotropyQ2 = mAnisotropyQ2;
    const PxVec4* srcAnisotropyQ3 = mAnisotropyQ3;
    bool hasResults = srcAnisotropyQ1 && srcAnisotropyQ2 && srcAnisotropyQ3 && mResultNumParticles > 0;
    if (!hasResults)
    {
        restoreScaleOrient();
        return;
    }

    float particleContactOffset;
    if (mParent)
    {
        particleContactOffset = mParent->mPS->getParticleContactOffset();
    }
    else
    {
        if (!parseParticleContactOffset(particleContactOffset))
        {
            return;
        }
    }
    // codepath to handle anisotropy values being absolute while the point instancer needs a scale.
    float particleContactDistanceInv = 1.0f / (particleContactOffset * 2.0f);

    std::vector<ParticleSet>& particleSets = getTmpParticleSets();
    for (size_t s = 0; s < particleSets.size(); ++s)
    {
        ParticleSet& particleSet = particleSets[s];
        UsdGeomPointInstancer pointInstancer(particleSet.usdPrim);
        if (pointInstancer)
        {
            updatePointInstancerScaleOrient(pointInstancer,
                srcAnisotropyQ1 + particleSet.srcOffset,
                srcAnisotropyQ2 + particleSet.srcOffset,
                srcAnisotropyQ3 + particleSet.srcOffset,
                particleSet.srcCount, particleContactDistanceInv);
        }
    }

    {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        flushUsdToFabric(omniPhysX.getStage(), false);
    }
#endif
}

void ParticlePostprocess::updatePoints()
{
#if USE_PHYSX_GPU
    const PxVec4* srcPositions = mSmoothedPositions;
    if (!srcPositions || mResultNumParticles == 0)
    {
        restorePoints();
        return;
    }

    std::vector<ParticleSet>& particleSets = getTmpParticleSets();
    for (size_t s = 0; s < particleSets.size(); ++s)
    {
        ParticleSet& particleSet = particleSets[s];

        if (!mParent)
        {
            //backup simulation points
            UsdAttribute simPointsAttr = PhysxSchemaPhysxParticleSetAPI(particleSet.usdPrim).GetSimulationPointsAttr();
            if (!simPointsAttr.HasAuthoredValue())
            {
                VtArray<GfVec3f> points = getParticleSetPoints(particleSet.usdPrim);
                simPointsAttr.Set(points);
            }
        }

        // A.B. use xformcache?
        GfMatrix4d localToWorld = UsdGeomXform(particleSet.usdPrim).ComputeLocalToWorldTransform(UsdTimeCode::Default());
        GfMatrix4d worldToLocal = localToWorld.GetInverse();
        UsdAttribute dstPointsAttr = getParticleSetPointsAttr(particleSet.usdPrim);
        VtArray<GfVec3f> dstPoints;
        dstPointsAttr.Get(&dstPoints);
        updateParticleSetPoints(dstPoints, srcPositions + particleSet.srcOffset, worldToLocal, particleSet.srcCount);
        dstPointsAttr.Set(dstPoints);
    }

    {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        flushUsdToFabric(omniPhysX.getStage(), false);
    }
#endif
}

void ParticlePostprocess::restorePoints()
{
    std::vector<ParticleSet>& particleSets = getTmpParticleSets();
    for (size_t s = 0; s < particleSets.size(); ++s)
    {
        ParticleSet& particleSet = particleSets[s];
        PhysxSchemaPhysxParticleSetAPI particleSetAPI(particleSet.usdPrim);
        if (particleSetAPI)
        {
            restoreSmoothedPoints(particleSetAPI);
        }
    }

    {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        flushUsdToFabric(omniPhysX.getStage(), false);
    }
}

void ParticlePostprocess::restoreScaleOrient()
{
    std::vector<ParticleSet>& particleSets = getTmpParticleSets();
    for (size_t s = 0; s < particleSets.size(); ++s)
    {
        ParticleSet& particleSet = particleSets[s];
        UsdGeomPointInstancer pointInstancer(particleSet.usdPrim);
        if (pointInstancer)
        {
            restorePointInstancerScaleOrient(pointInstancer);
        }
    }

    {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        flushUsdToFabric(omniPhysX.getStage(), false);
    }
}

void ParticlePostprocess::createSessionIsosurfaceMesh()
{
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    if (stage)
    {
        ScopedLayerEdit scopedSessionLayerEdit(stage, stage->GetSessionLayer());
        mIsosurfaceMeshPath = mParticleSystemPath.AppendElementString("Isosurface");
        UsdGeomMesh mesh = UsdGeomMesh::Define(stage, mIsosurfaceMeshPath);
        primutils::setNoDelete(mesh.GetPrim(), true);
        primutils::setHideInStageWindow(mesh.GetPrim(), true);

        // make particle set invisible by setting purpose = proxy on session layer
        // we don't need to store this to file, and retains visibility attribute for user (e.g. to selectively allow for disabling of particle visualization)
        std::vector<ParticleSet>& particleSets = getTmpParticleSets();
        for (size_t s = 0; s < particleSets.size(); ++s)
        {
            ParticleSet& particleSet = particleSets[s];
            UsdGeomImageable geomImageable(particleSet.usdPrim);
            if (geomImageable)
            {
                geomImageable.GetPurposeAttr().Set(UsdGeomTokens.Get()->proxy);
            }
        }
    }
}

void ParticlePostprocess::releaseSessionIsosurfaceMesh()
{
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    if (stage)
    {
        ScopedLayerEdit scopedSessionLayerEdit(stage, stage->GetSessionLayer());
        UsdPrim meshPrim = stage->GetPrimAtPath(mIsosurfaceMeshPath);
        if (meshPrim)
        {
            stage->RemovePrim(mIsosurfaceMeshPath);
        }

        // make particles visible again by removing proxy purpose attribute from session layer
        std::vector<ParticleSet>& particleSets = getTmpParticleSets();
        for (size_t s = 0; s < particleSets.size(); ++s)
        {
            ParticleSet& particleSet = particleSets[s];
            UsdGeomImageable geomImageable(particleSet.usdPrim);
            if (geomImageable)
            {
                geomImageable.GetPurposeAttr().Clear();
            }
        }
    }
    mIsosurfaceMeshPath = SdfPath();
}

namespace
{
    struct PostprocessRef
    {
        uint32_t refCount = 0;
        ParticlePostprocess* postprocess = nullptr;
    };

    typedef std::map<SdfPath, PostprocessRef> PostprocessRefMap;
    PostprocessRefMap gPostprocessRefMap;

    PostprocessRef* getPostprocessRef(const SdfPath& particleSystemPath)
    {
        auto it = gPostprocessRefMap.find(particleSystemPath);
        return (it != gPostprocessRefMap.end()) ? &it->second : nullptr;
    }
}

namespace omni
{
namespace physx
{
namespace particles
{
#if USE_PHYSX_GPU
    void setIsosurfaceGridFilteringPasses(PxIsosurfaceParams& isosurfaceParams, const std::vector<ParticleIsosurfaceDesc::GridFilteringPass::Enum>& passes)
    {
        isosurfaceParams.clearFilteringPasses();
        for (size_t i = 0; i < passes.size(); ++i)
        {
            PxIsosurfaceGridFilteringType::Enum dstType = PxIsosurfaceGridFilteringType::eNONE;
            if (passes[i] == ParticleIsosurfaceDesc::GridFilteringPass::eSmooth)
            {
                dstType = PxIsosurfaceGridFilteringType::eSMOOTH;
            }
            else if (passes[i] == ParticleIsosurfaceDesc::GridFilteringPass::eGrow)
            {
                dstType = PxIsosurfaceGridFilteringType::eGROW;
            }
            else if (passes[i] == ParticleIsosurfaceDesc::GridFilteringPass::eReduce)
            {
                dstType = PxIsosurfaceGridFilteringType::eSHRINK;
            }
            isosurfaceParams.addGridFilteringPass(dstType);
        }
    }
#endif

    void createPostprocess(const SdfPath& particleSystemPath, uint32_t particlePostFlags, omni::physx::internal::InternalPbdParticleSystem* parent)
    {
        PostprocessRef newRef = {0, nullptr};
        auto res = gPostprocessRefMap.insert({particleSystemPath, newRef});
        if (res.second)
        {
            ParticlePostprocess* newPostprocess = ICE_NEW(ParticlePostprocess)(particleSystemPath);
            newPostprocess->setPostprocessFlags(particlePostFlags);
            // with the current setup, parent needs to be set after enabling stages.
            if (parent)
            {
                newPostprocess->setParent(parent);
            }
            res.first->second.postprocess = newPostprocess;
        }
        else
        {
            ParticlePostprocess* oldPostprocess = res.first->second.postprocess;
            uint32_t currentPostFlags = oldPostprocess->getPostprocessFlags();
            if (particlePostFlags != currentPostFlags)
            {
                oldPostprocess->setPostprocessFlags(particlePostFlags);
            }

            if (parent)
            {
                oldPostprocess->setParent(parent);
            }
        }
        res.first->second.refCount++;
    }

    void releasePostprocess(const SdfPath& particleSystemPath, omni::physx::internal::InternalPbdParticleSystem* parent)
    {
        PostprocessRef* postprocessRef = getPostprocessRef(particleSystemPath);
        if (postprocessRef && postprocessRef->postprocess)
        {
            if (parent)
            {
                postprocessRef->postprocess->setParent(nullptr);
            }

            CARB_ASSERT(postprocessRef->refCount > 0);
            postprocessRef->refCount--;

            if (postprocessRef->refCount == 0)
            {
                SAFE_DELETE_SINGLE(postprocessRef->postprocess);
                gPostprocessRefMap.erase(particleSystemPath);
            }
        }
    }

    void getAnisotropy(::physx::PxVec4*& anisotropyQ1, ::physx::PxVec4*& anisotropyQ2, ::physx::PxVec4*& anisotropyQ3, const SdfPath& particleSystemPath)
    {
        PostprocessRef* postprocessRef = getPostprocessRef(particleSystemPath);
        if (postprocessRef && postprocessRef->postprocess)
        {
            postprocessRef->postprocess->getAnisotropy(anisotropyQ1, anisotropyQ2, anisotropyQ3);
        }
        else
        {
            anisotropyQ1 = anisotropyQ2 = anisotropyQ3 = nullptr;
        }
    }

    ::physx::PxVec4* getSmoothedPositions(const SdfPath& particleSystemPath)
    {
        PostprocessRef* postprocessRef = getPostprocessRef(particleSystemPath);
        if (postprocessRef && postprocessRef->postprocess)
        {
            return postprocessRef->postprocess->getSmoothedPositions();
        }
        return nullptr;
    }

    void updateIsosurfaceMesh(const SdfPath& particleSystemPath)
    {
        PostprocessRef* postprocessRef = getPostprocessRef(particleSystemPath);
        if (postprocessRef && postprocessRef->postprocess)
        {
            postprocessRef->postprocess->updateMesh();
        }
    }

    void notifyParticleSystemResize(const SdfPath& particleSystemPath)
    {
        PostprocessRef* postprocessRef = getPostprocessRef(particleSystemPath);
        if (postprocessRef && postprocessRef->postprocess)
        {
            postprocessRef->postprocess->updateGeneratorsCallback();
        }
    }

    void notifyPhysXRelease()
    {
        for (auto it = gPostprocessRefMap.begin(); it != gPostprocessRefMap.end(); ++it)
        {
            PostprocessRef& postprocessRef = it->second;
            if (postprocessRef.postprocess)
            {
                //disable all postprocesses
                postprocessRef.postprocess->setPostprocessFlags(0);
            }
        }
    }

    ///////////////////////////////
    /// IPhysXParticles interface
    ///////////////////////////////

    void createPostprocess(const SdfPath& particleSystemPath, uint32_t particlePostFlags)
    {
        createPostprocess(particleSystemPath, particlePostFlags, nullptr);
    }

    void releasePostprocess(const SdfPath& particleSystemPath)
    {
        releasePostprocess(particleSystemPath, nullptr);
    }

    uint32_t getPostprocessStages(const SdfPath& particleSystemPath)
    {
        PostprocessRef* postprocessRef = getPostprocessRef(particleSystemPath);
        if (postprocessRef && postprocessRef->postprocess)
        {
            return postprocessRef->postprocess->getPostprocessFlags();
        }
        return ParticlePostFlag::eNone;
    }

    void setPostprocessStages(const SdfPath& particleSystemPath, uint32_t particlePostFlags)
    {
        PostprocessRef* postprocessRef = getPostprocessRef(particleSystemPath);
        if (postprocessRef && postprocessRef->postprocess)
        {
            postprocessRef->postprocess->setPostprocessFlags(particlePostFlags);
        }
    }

    void addPostprocessParticleSet(const SdfPath& particleSystemPath, const SdfPath& particleSetPath)
    {
        PostprocessRef* postprocessRef = getPostprocessRef(particleSystemPath);
        if (postprocessRef && postprocessRef->postprocess)
        {
            postprocessRef->postprocess->addParticleSet(particleSetPath);
        }
    }

    void removePostprocessParticleSet(const SdfPath& particleSystemPath, const SdfPath& particleSetPath)
    {
        PostprocessRef* postprocessRef = getPostprocessRef(particleSystemPath);
        if (postprocessRef && postprocessRef->postprocess)
        {
            postprocessRef->postprocess->removeParticleSet(particleSetPath);
        }
    }


    void updatePostprocess(const SdfPath& particleSystemPath)
    {
        PostprocessRef* postprocessRef = getPostprocessRef(particleSystemPath);
        if (postprocessRef && postprocessRef->postprocess)
        {
            postprocessRef->postprocess->updatePreview();
        }
    }

} // namespace particles
} // namespace physx
} // namespace omni
