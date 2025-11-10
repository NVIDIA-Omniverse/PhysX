// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <common/utilities/PrimUtilities.h>
#include <common/utilities/MemoryMacros.h>

#include "InternalParticle.h"
#include "InternalTools.h"
#include "particles/PhysXParticlePost.h"

#include <PhysXTools.h>
#include <PhysXSettings.h>

#include <usdLoad/LoadUsd.h>
#include <usdLoad/Particles.h>


#if USE_PHYSX_GPU
#include "extensions/PxParticleExt.h"
#endif

using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace pxr;
using namespace carb;
using namespace ::physx;
using namespace ::physx::ExtGpu;

extern ObjectId getObjectId(const pxr::SdfPath& path, PhysXType type);

InternalDiffuseParticles::~InternalDiffuseParticles()
{
    if (mGeo)
    {
        UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
        ScopedLayerEdit scopedSessionLayerEdit(stage, stage->GetSessionLayer());
        stage->RemovePrim(mGeo.GetPath());
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void InternalParticleSet::enableParticleSet(bool enable)
{
    if (enable == mEnabled)
        return;

    bool hasBufferAndPsEnabled = mParticleBuffer && mParentParticleSystem && mParentParticleSystem->mEnabled;
    if (!hasBufferAndPsEnabled)
        return;

    if (enable)
    {
        mParentParticleSystem->mPS->addParticleBuffer(mParticleBuffer);
    }
    else
    {
        mParentParticleSystem->mPS->removeParticleBuffer(mParticleBuffer);
    }
    mEnabled = enable;
}

void InternalParticleSet::releasePinnedBuffers()
{
#if USE_PHYSX_GPU
    PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
    if (cudaContextManager)
    {
        if (mPositions)
        {
            PX_PINNED_HOST_FREE(cudaContextManager, mPositions);
        }

        if (mVelocities)
        {
            PX_PINNED_HOST_FREE(cudaContextManager, mVelocities);
        }

        if (mPhases)
        {
            PX_PINNED_HOST_FREE(cudaContextManager, mPhases);
        }

        if (mDiffuseParticlePositions)
        {
            PX_PINNED_HOST_FREE(cudaContextManager, mDiffuseParticlePositions);
        }
    }
#endif
}

InternalParticleSet::InternalParticleSet(InternalPbdParticleSystem& particleSystem)
    : InternalParticle(particleSystem)
    , mPositions(nullptr)
    , mVelocities(nullptr)
    , mPhases(nullptr)
    , mDiffuseParticlePositions(nullptr)
    , mNumDiffuseParticles(0)
    , mDiffuseParticleParams(::physx::PxDiffuseParticleParams())
    , mMaxDiffuseParticleMultiplier(0.0f)
    , mDiffuseParticlesEnabled(false)
    , mDownloadDirtyFlags(0)
    , mParticleBuffer(nullptr)
    , mUploadDirtyFlags(0)
    , mReplacementBuffer(nullptr)
    , mMaxParticles(0)
    , mNumVolumes(0)
    , mFluid(false)
    , mParticleInvMass(0.0f)
    , mPhase(0)
    , mFabric(false)
{
}

InternalParticleSet::~InternalParticleSet()
{
    releasePinnedBuffers();

    if (mParticleBuffer)
    {
        SAFE_RELEASE(mParticleBuffer); // will remove from particle system
    }

    // Remove material reference
    InternalPBDParticleMaterial* internalMaterial = getInternalPtr<InternalPBDParticleMaterial>(ePTPBDMaterial, mMaterialId);
    if (internalMaterial)
    {
        usdparser::ObjectId id = getObjectId(mPrim.GetPath(), ePTParticleSet);
        internalMaterial->removeParticleId(id);
    }
}

uint32_t InternalParticleSet::getMaxDiffuseParticles(uint32_t maxParticles,
    bool diffuseEnabled, float maxDiffuseParticleMultiplier)
{
    return diffuseEnabled ? static_cast<uint32_t>(maxParticles * maxDiffuseParticleMultiplier) : 0;
}

PxParticleAndDiffuseBuffer* InternalParticleSet::createUserBuffer(uint32_t maxParticles, uint32_t numParticles,
    bool diffuseEnabled, float maxDiffuseParticleMultiplier, PxDiffuseParticleParams& diffuseParams, PhysXSetup& physXSetup)
{
    PxParticleAndDiffuseBuffer* newBuffer = nullptr;
    PxCudaContextManager* cudaContextManager = physXSetup.getCudaContextManager();
    if (cudaContextManager)
    {
        PxPhysics* physics = physXSetup.getPhysics();
        uint32_t maxDiffuseParticles = InternalParticleSet::getMaxDiffuseParticles(maxParticles, diffuseEnabled, maxDiffuseParticleMultiplier);
        uint32_t maxDiffuseParticlesBuffer = std::max<uint32_t>(maxDiffuseParticles, 1);
        uint32_t maxParticlesBuffer = std::max<uint32_t>(maxParticles, 1);
        newBuffer = physics->createParticleAndDiffuseBuffer(maxParticlesBuffer, 0, maxDiffuseParticlesBuffer, cudaContextManager);

        newBuffer->setNbActiveParticles(numParticles);
        newBuffer->setMaxActiveDiffuseParticles(maxDiffuseParticles);
        newBuffer->setNbParticleVolumes(0);
        newBuffer->setDiffuseParticleParams(diffuseParams);
    }

    return newBuffer;
}

void InternalParticleSet::createSharedDiffuseParticles()
{
    if (!mParentParticleSystem)
        return; // TODO should this spit out a warning?

    // creates a diffuse particle instance - makes things ready to roll
    if (!mParentParticleSystem->mDiffuseParticleInstance)
    {
        UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
        ScopedLayerEdit scopedSessionLayerEdit(stage, stage->GetSessionLayer());
            
        mParentParticleSystem->mDiffuseParticleInstance = ICE_NEW(InternalDiffuseParticles);
        InternalDiffuseParticles* internalDiffuseParticles = mParentParticleSystem->mDiffuseParticleInstance;

        std::string geomPath = mParentParticleSystem->mPath.GetString() + "/DiffuseParticles";
        pxr::UsdPrim geoPrim = stage->DefinePrim(SdfPath(geomPath), pxr::TfToken("Points"));

        internalDiffuseParticles->mGeo = pxr::UsdGeomPoints(geoPrim);

        primutils::setNoDelete(internalDiffuseParticles->mGeo.GetPrim(), true);
        primutils::setHideInStageWindow(internalDiffuseParticles->mGeo.GetPrim(), true);

        internalDiffuseParticles->mGeo.CreatePointsAttr();
        internalDiffuseParticles->mGeo.GetPointsAttr().Clear();

        internalDiffuseParticles->mGeo.CreateDisplayColorPrimvar();
        internalDiffuseParticles->mGeo.GetDisplayColorPrimvar().GetAttr().Clear();

        internalDiffuseParticles->mPrim = geoPrim;

        internalDiffuseParticles->mGeo.MakeInvisible();

        //Connect a flow emitter
        bool success = false;
        UsdPrim psPrim = stage->GetPrimAtPath(mParentParticleSystem->mPath);
        for (auto c : psPrim.GetChildren()) //Search in particle system's children for a flow emitter
        {
            if (c.GetTypeName() == TfToken("FlowEmitterPoint"))
            {
                const TfToken pointsPrimRel("pointsPrim");
                SdfPathVector target;
                target.push_back(SdfPath(geomPath));
                c.CreateRelationship(pointsPrimRel, true).SetTargets(target);
                success = true;
                break;
            }
        }
        if (!success)
        {
            for (auto c : psPrim.GetParent().GetChildren()) //Search in children of particle system's parent for a flow emitter
            {
                if (c.GetTypeName() == TfToken("FlowEmitterPoint"))
                {
                    const TfToken pointsPrimRel("pointsPrim");
                    SdfPathVector target;
                    target.push_back(SdfPath(geomPath));
                    c.CreateRelationship(pointsPrimRel, true).SetTargets(target);
                    success = true;
                    break;
                }
            }
        }
    }

    if (!mSharedDiffuseParticles)
    {
        mParentParticleSystem->mDiffuseParticleInstanceRefCount++;
        mSharedDiffuseParticles = mParentParticleSystem->mDiffuseParticleInstance;
    }
}

void InternalParticleSet::releaseSharedDiffuseParticles()
{
    if (mSharedDiffuseParticles && mParentParticleSystem)
    {
        if (mParentParticleSystem->mDiffuseParticleInstanceRefCount == 1)
        {
            // release shared InternalDiffuseParticles (geometry)
            SAFE_DELETE_SINGLE(mParentParticleSystem->mDiffuseParticleInstance);
        }
        mParentParticleSystem->mDiffuseParticleInstanceRefCount--;
    }
}

void InternalParticleSet::changeDiffuseParticles(bool remove)
{
#if USE_PHYSX_GPU
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    PxCudaContextManager* cudaContextManager = physxSetup.getCudaContextManager();
    if (!cudaContextManager)
        return;

    const uint64_t stageId = OmniPhysX::getInstance().getStageId();
    usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (!attachedStage)
        return;

    if (remove)
    {
        mDiffuseParticlesEnabled = false;
        mDiffuseParticleParams = PxDiffuseParticleParams();
        releaseSharedDiffuseParticles();
    }
    else
    {
        ParticleSetDesc* particleSetDesc = ParseParticleSet(*attachedStage, mPrim);
        if (particleSetDesc)
        {
            mDiffuseParticlesEnabled = particleSetDesc->enableDiffuseParticles;
            mMaxDiffuseParticleMultiplier = particleSetDesc->maxDiffuseParticleMultiplier;
            mDiffuseParticleParams.threshold = particleSetDesc->diffuseParticlesThreshold;
            mDiffuseParticleParams.lifetime = particleSetDesc->diffuseParticlesLifetime;
            mDiffuseParticleParams.airDrag = particleSetDesc->diffuseParticlesAirDrag;
            mDiffuseParticleParams.bubbleDrag = particleSetDesc->diffuseParticlesBubbleDrag;
            mDiffuseParticleParams.buoyancy = particleSetDesc->diffuseParticlesBuoyancy;
            mDiffuseParticleParams.kineticEnergyWeight = particleSetDesc->diffuseParticlesKineticEnergyWeight;
            mDiffuseParticleParams.pressureWeight = particleSetDesc->diffuseParticlesPressureWeight;
            mDiffuseParticleParams.divergenceWeight = particleSetDesc->diffuseParticlesDivergenceWeight;
            mDiffuseParticleParams.collisionDecay = particleSetDesc->diffuseParticlesCollisionDecay;
            ICE_FREE(particleSetDesc);
        }
    }

    if (mDiffuseParticlePositions)
    {
        PX_PINNED_HOST_FREE(cudaContextManager, mDiffuseParticlePositions);
    }

    uint32_t maxDiffuseParticles = getMaxDiffuseParticles(mMaxParticles, mDiffuseParticlesEnabled, mMaxDiffuseParticleMultiplier);
    if (maxDiffuseParticles > 0)
    {
        mDiffuseParticlePositions = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, maxDiffuseParticles);
        createSharedDiffuseParticles();
    }

    mNumDiffuseParticles = 0;

    mReplacementBuffer = createUserBuffer(mMaxParticles, mNumParticles, mDiffuseParticlesEnabled, mMaxDiffuseParticleMultiplier, mDiffuseParticleParams, physxSetup);
#endif
}

void InternalParticleSet::enableDiffuseParticles(bool enabled)
{
    mDiffuseParticlesEnabled = enabled;
    uint32_t maxDiffuseParticles = getMaxDiffuseParticles(mMaxParticles, mDiffuseParticlesEnabled, mMaxDiffuseParticleMultiplier);
    mParticleBuffer->setMaxActiveDiffuseParticles(maxDiffuseParticles);
}

void InternalParticleSet::setDiffuseParticleParams()
{
    mParticleBuffer->setDiffuseParticleParams(mDiffuseParticleParams);
}

void InternalParticleSet::resize(uint32_t newNumParticles)
{
    if (newNumParticles == mNumParticles)
    {
        return;
    }

    if (newNumParticles <= mMaxParticles)
    {
        //update active particles and initialize phases if necessary
        if (newNumParticles > mNumParticles)
        {
            for (uint32_t i = mNumParticles; i < newNumParticles; ++i)
                mPhases[i] = mPhase;

            mUploadDirtyFlags |= ParticleBufferFlags::ePHASES;
        }
        mParticleBuffer->setNbActiveParticles(newNumParticles);
        mNumParticles = newNumParticles;
        return;
    }

    //now we need to deal with the case where buffers need to be resized
    static float growthFactor = 1.5f;
    uint32_t newMaxParticles = std::max(uint32_t(growthFactor*mMaxParticles), newNumParticles);

#if USE_PHYSX_GPU
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    PxPhysics* physics = physxSetup.getPhysics();
    PxCudaContextManager* cudaContextManager = physxSetup.getCudaContextManager();
    if (!cudaContextManager)
        return;

    // resize all the pinned buffers: we skip copying the old data because this will be loaded from USD anyway.
    if (mPositions)
    {
        PX_PINNED_HOST_FREE(cudaContextManager, mPositions);
        mPositions = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, newMaxParticles);
    }

    if (mVelocities)
    {
        PX_PINNED_HOST_FREE(cudaContextManager, mVelocities);
        mVelocities = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, newMaxParticles);
    }

    if (mPhases)
    { 
        PxU32* newPhases = PX_PINNED_HOST_ALLOC_T(PxU32, cudaContextManager, newMaxParticles);

        //copy and initialize surplus phases
        std::memcpy(newPhases, mPhases, mNumParticles * sizeof(PxU32));

        for (PxU32 i = mNumParticles; i < newNumParticles; ++i)
            newPhases[i] = mPhase;

        PX_PINNED_HOST_FREE(cudaContextManager, mPhases);
        mPhases = newPhases;
        mUploadDirtyFlags |= ParticleBufferFlags::ePHASES;
    }

    if (mDiffuseParticlePositions)
    {
        uint32_t maxDiffuseParticles = getMaxDiffuseParticles(mMaxParticles, mDiffuseParticlesEnabled, mMaxDiffuseParticleMultiplier);
        PX_PINNED_HOST_FREE(cudaContextManager, mDiffuseParticlePositions);
        mDiffuseParticlePositions = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, maxDiffuseParticles);
    }

    mMaxParticles = newMaxParticles;
    mNumParticles = newNumParticles;
    PxParticleAndDiffuseBuffer* newBuffer = createUserBuffer(mMaxParticles, mNumParticles, mDiffuseParticlesEnabled,
        mMaxDiffuseParticleMultiplier, mDiffuseParticleParams, physxSetup);
    newBuffer->raiseFlags(PxParticleBufferFlag::eALL);

    mParticleBuffer->release(); // will also remove buffer form particle system
    mParticleBuffer = newBuffer;

    if (mEnabled && mParentParticleSystem && mParentParticleSystem->mEnabled)
        mParentParticleSystem->mPS->addParticleBuffer(mParticleBuffer);

    //notify resizing to postprocess
    particles::notifyParticleSystemResize(mParentParticleSystem->mPath);

    // need to udpate the Physx pointer for the particle set.
    {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();
        usdparser::ObjectId id = getObjectId(mPrim.GetPath(), ePTParticleSet);
        InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTParticleSet, id);
        if (objectRecord)
        {
            objectRecord->mPtr = mParticleBuffer;
        }
    }
#endif
}

void InternalParticleSet::uploadParticles(CUstream stream)
{
    PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
    if (!cudaContextManager)
        return;

    PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

    // if we need to update/switch out the particle buffer
    if (mReplacementBuffer)
    {
        cudaContextManager->acquireContext();
        cudaContext->memcpyDtoDAsync(CUdeviceptr(mReplacementBuffer->getPositionInvMasses()), CUdeviceptr(mParticleBuffer->getPositionInvMasses()), mNumParticles * sizeof(PxVec4), stream);
        cudaContext->memcpyDtoDAsync(CUdeviceptr(mReplacementBuffer->getVelocities()), CUdeviceptr(mParticleBuffer->getVelocities()), mNumParticles * sizeof(PxVec4), stream);
        cudaContext->memcpyDtoDAsync(CUdeviceptr(mReplacementBuffer->getPhases()), CUdeviceptr(mParticleBuffer->getPhases()), mNumParticles * sizeof(PxU32), stream);
        cudaContextManager->releaseContext();

        mReplacementBuffer->raiseFlags(PxParticleBufferFlag::eALL);
        mReplacementBuffer->setNbActiveParticles(mParticleBuffer->getNbActiveParticles());

        mParticleBuffer->release(); // removes buffer from particle system

        mParticleBuffer = mReplacementBuffer;
        mReplacementBuffer = nullptr;

        if (mEnabled && mParentParticleSystem && mParentParticleSystem->mEnabled)
            mParentParticleSystem->mPS->addParticleBuffer(mParticleBuffer);

        // need to udpate the Physx pointer for the particle set.
        {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

            PhysXType internalType = ePTRemoved;
            usdparser::ObjectId id = getObjectId(mPrim.GetPath(), ePTParticleSet);
            InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, id);
            if (objectRecord)
            {
                objectRecord->mPtr = mParticleBuffer;
            }
        }
    }

    if (!mEnabled)
        return;

    if (mUploadDirtyFlags)
    {
        cudaContextManager->acquireContext();

        if ((mUploadDirtyFlags & ParticleBufferFlags::ePOSITIONS) && mPositions)
        {
            if (!mFabric)
                cudaContext->memcpyHtoDAsync(CUdeviceptr(mParticleBuffer->getPositionInvMasses()), mPositions, mNumParticles * sizeof(PxVec4), stream);
            else
                cudaContext->memcpyDtoDAsync(CUdeviceptr(mParticleBuffer->getPositionInvMasses()), CUdeviceptr(mFabricPositions), mNumParticles * sizeof(PxVec4), stream);

            mParticleBuffer->raiseFlags(PxParticleBufferFlag::eUPDATE_POSITION);
        }

        if ((mUploadDirtyFlags & ParticleBufferFlags::eVELOCITIES) && mVelocities)
        {
            if (!mFabric)
                cudaContext->memcpyHtoDAsync(CUdeviceptr(mParticleBuffer->getVelocities()), mVelocities, mNumParticles * sizeof(PxVec4), stream);
            else
                cudaContext->memcpyDtoDAsync(CUdeviceptr(mParticleBuffer->getVelocities()), CUdeviceptr(mFabricVelocities), mNumParticles * sizeof(PxVec4), stream);

            mParticleBuffer->raiseFlags(PxParticleBufferFlag::eUPDATE_VELOCITY);
        }

        if ((mUploadDirtyFlags & ParticleBufferFlags::ePHASES) && mPhases)
        {
    		cudaContext->memcpyHtoDAsync(CUdeviceptr(mParticleBuffer->getPhases()), mPhases, mNumParticles * sizeof(PxU32), stream);
            mParticleBuffer->raiseFlags(PxParticleBufferFlag::eUPDATE_PHASE);
        }

        cudaContextManager->releaseContext();

        mUploadDirtyFlags = 0;
    }
}

void InternalParticleSet::fetchParticles(CUstream stream, bool hasIsosurface, bool hasSmoothing,
    bool hasAnisotropy, bool updateToUsd, bool updateParticlesToUsd, bool updateVelocities, bool debugVizOn)
{
    if (mParticleBuffer && mEnabled)
    {
        PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
        if (!cudaContextManager)
            return;

        PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

        // calculate download flags and set them for transform updates.
        mDownloadDirtyFlags = 0;

        // TODO: how does this change with vec4-vec3 + transforms on gpu?
        // TODO: how does this change with fabric/direct gpu interop?

        // AD some comments about this:
        //
        // These download flags are theoretically independent of all the updateToUsd stuff. However, we need to respect
        // those settings in here as well.
        //
        // The intention of those flags is to 
        //
        // 1) figure out what has been calculated on the GPU
        // 2) communicate to the transform update function what data has been fetched from the CPU
        //
        // this has the effect the the fine-grained updateToUsd logic in the transform updates needs to
        // be respected there as well
        //
        // This means that these flags are also relevant in case updateToUsd is off, but the GPU-CPU transfer
        // is still needed for fabric support.
        //

        // Truth table:
        // updateToUsd just downloads everything that has been calculated.
        // updateParticlesToUsd - always download
        // debugViz - always download
        // not a fluid - always download as not impacted by postprocessing
        // fluid, no postpocessing - download
        // fluid, surface - don't download
        // fluid, smoothing - don't download
        // fluid, aniso - download
        // fluid, surface + smoothing - no download
        // fluid, surface + aniso - no download
        // fluid, smoothing + aniso - no download
        // fluid, smoothing + aniso + surface - no download
        if ((updateParticlesToUsd || debugVizOn || !mFluid || (!hasIsosurface && !hasSmoothing && !hasAnisotropy) ||
            (hasAnisotropy && !hasSmoothing && !hasIsosurface)) &&
            (updateToUsd || updateParticlesToUsd))
            mDownloadDirtyFlags |= ParticleDirtyFlags::ePOSITION_INVMASS;

        if (updateVelocities && (updateToUsd || updateParticlesToUsd))
            mDownloadDirtyFlags |= ParticleDirtyFlags::eVELOCITY;

        if ((mNumVolumes > 0 && mVolumes))
            mDownloadDirtyFlags |= ParticleDirtyFlags::eVOLUME;

        mNumDiffuseParticles = mParticleBuffer->getNbActiveDiffuseParticles();
        if ((mNumDiffuseParticles > 0 && mDiffuseParticlePositions))
            mDownloadDirtyFlags |= ParticleDirtyFlags::eDIFFUSE_PARTICLES;

        if (hasSmoothing && mFluid && (!hasIsosurface))
        {
            mDownloadDirtyFlags |= ParticleDirtyFlags::eSMOOTHED_POSITIONS;
        }

        if (hasAnisotropy && mFluid && (!hasIsosurface))
            mDownloadDirtyFlags |= ParticleDirtyFlags::eANISOTROPY;

        if (mFabric)
        {
            mDownloadDirtyFlags |= ParticleDirtyFlags::ePOSITION_INVMASS;
            mDownloadDirtyFlags |= ParticleDirtyFlags::eVELOCITY;
        }

        if (mDownloadDirtyFlags)
        {
            cudaContextManager->acquireContext();

            if (mDownloadDirtyFlags & ParticleDirtyFlags::ePOSITION_INVMASS)
            {
                if (!mFabric)
                    cudaContext->memcpyDtoHAsync(mPositions, CUdeviceptr(mParticleBuffer->getPositionInvMasses()), mNumParticles * sizeof(PxVec4), stream);
                else
                    cudaContext->memcpyDtoDAsync(CUdeviceptr(mFabricPositions), CUdeviceptr(mParticleBuffer->getPositionInvMasses()), mNumParticles * sizeof(PxVec4), stream);
            }

            if (mDownloadDirtyFlags & ParticleDirtyFlags::eVELOCITY)
            {
                if (!mFabric)
                    cudaContext->memcpyDtoHAsync(mVelocities, CUdeviceptr(mParticleBuffer->getVelocities()), mNumParticles * sizeof(PxVec4), stream);
                else
                    cudaContext->memcpyDtoDAsync(CUdeviceptr(mFabricVelocities), CUdeviceptr(mParticleBuffer->getVelocities()), mNumParticles * sizeof(PxVec4), stream);
            }

            if (mDownloadDirtyFlags & ParticleDirtyFlags::eVOLUME)
                cudaContext->memcpyDtoHAsync(mVolumes, CUdeviceptr(mParticleBuffer->getParticleVolumes()), mNumVolumes * sizeof(PxParticleVolume), stream);

            if (mDownloadDirtyFlags & ParticleDirtyFlags::eDIFFUSE_PARTICLES)
                cudaContext->memcpyDtoHAsync(mDiffuseParticlePositions, CUdeviceptr(mParticleBuffer->getDiffusePositionLifeTime()), mNumDiffuseParticles * sizeof(PxVec4), stream);

            cudaContextManager->releaseContext();

            // TODO: make sure smoothing & aniso are only downloaded when needed.
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

InternalPbdParticleSystem::~InternalPbdParticleSystem()
{
    //InternalParticleClothDeprecated and InternalParticleSet instances need to be removed from PhysX DB
    //outside of destructor, because here we can't tell whether DB is
    //- still alive (direct particle system removal)
    //- or not (internal scene release)
    for (uint32_t i = 0; i < mParticleSets.size(); i++)
    {
        InternalParticleSet* current = mParticleSets[i];
        SAFE_DELETE_SINGLE(current);
    }
    mParticleSets.clear();
    mParticleSets.shrink_to_fit();

    for (uint32_t i = 0; i < mCloths.size(); i++)
    {
        InternalParticleClothDeprecated* current = mCloths[i];
        SAFE_DELETE_SINGLE(current);
    }
    mCloths.clear();
    mCloths.shrink_to_fit();

    setPost(ParticlePostFlag::eNone);

    SAFE_DELETE_SINGLE(mDiffuseParticleInstance);

#if USE_PHYSX_GPU
    SAFE_DELETE_SINGLE(mCallback);
#endif

    if (mPS)
    {
        if (mPS->getScene())
        {
            mPS->getScene()->removeActor(*mPS, false);
        }
        SAFE_RELEASE(mPS);
    }
}

void InternalPbdParticleSystem::enableParticleSystem(bool enable)
{
    if (enable == mEnabled)
        return;

    if (enable)
    {
        mPhysXScene->getScene()->addActor(*mPS);

        for (uint32_t i = 0; i < mParticleSets.size(); ++i)
        {
            if (mParticleSets[i]->mEnabled)
            {
                mPS->addParticleBuffer(mParticleSets[i]->mParticleBuffer);
            }
        }

        for (uint32_t i = 0; i < mCloths.size(); ++i)
        {
            if (mCloths[i]->mEnabled)
            {
                mPS->addParticleBuffer(mCloths[i]->mClothBuffer);
            }
        }
    }
    else
    {
        for (uint32_t i = 0; i < mParticleSets.size(); ++i)
        {
            if (mParticleSets[i]->mEnabled)
            {
                mPS->removeParticleBuffer(mParticleSets[i]->mParticleBuffer);
            }
        }

        for (uint32_t i = 0; i < mCloths.size(); ++i)
        {
            if (mCloths[i]->mEnabled)
            {
                mPS->removeParticleBuffer(mCloths[i]->mClothBuffer);
            }
        }

        mPS->getScene()->removeActor(*mPS);
    }
    mEnabled = enable;
}

void InternalPbdParticleSystem::removeParticleObjectsFromDB()
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    for (uint32_t i = 0; i < mParticleSets.size(); i++)
    {
        usdparser::ObjectId id = getObjectId(mParticleSets[i]->mPrim.GetPath(), ePTParticleSet);
        InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTParticleSet, id);
        if (objectRecord)
        {
            objectRecord->setRemoved();
        }
    }

    for (uint32_t i = 0; i < mCloths.size(); i++)
    {
        usdparser::ObjectId id = getObjectId(mCloths[i]->mPrim.GetPath(), ePTParticleClothDeprecated);
        InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTParticleClothDeprecated, id);
        if (objectRecord)
        {
            objectRecord->setRemoved();
        }
    }
}

void InternalPbdParticleSystem::removeParticleSet(InternalParticleSet* internalParticleSet)
{
    auto it = std::find(mParticleSets.begin(), mParticleSets.end(), internalParticleSet);
    if (it != mParticleSets.end())
    {
        std::swap(*it, mParticleSets.back());
        mParticleSets.pop_back();
    }
}

void InternalPbdParticleSystem::removeParticleCloth(InternalParticleClothDeprecated* internalParticleCloth)
{
    auto it = std::find(mCloths.begin(), mCloths.end(), internalParticleCloth);
    if (it != mCloths.end())
    {
        std::swap(*it, mCloths.back());
        mCloths.pop_back();
    }
}

void InternalPbdParticleSystem::uploadParticles(CUstream stream)
{
    for (int i = 0; i < mParticleSets.size(); ++i)
    {
        mParticleSets[i]->uploadParticles(stream);
    }

    for (int i = 0; i < mCloths.size(); ++i)
    {
        mCloths[i]->uploadParticles(stream);
    }
}

void InternalPbdParticleSystem::fetchParticles(CUstream stream)
{
    if (!mParticleDataAvailable || !mEnabled || mPhysXScene->isReadbackSuppressed())
        return;

    uint32_t postFlags = particles::getPostprocessStages(mPath);
    bool hasIsosurface = postFlags & ParticlePostFlag::eIsosurface;
    bool hasSmoothing = postFlags & ParticlePostFlag::eSmoothing;
    bool hasAnisotropy = postFlags & ParticlePostFlag::eAnisotropy;

    carb::settings::ISettings* settings = OmniPhysX::getInstance().getISettings();
    bool debugVizOn = settings->getAsInt(kSettingDisplayParticles) > 0; // needs everything in USD
    bool updateParticlesToUsd = settings->getAsBool(kSettingUpdateParticlesToUsd); // force read to save simulation state
    bool updateVelocities = settings->getAsBool(kSettingUpdateVelocitiesToUsd);
    bool updateToUsd = settings->getAsBool(kSettingUpdateToUsd);

    for (int i = 0; i < mParticleSets.size(); ++i)
    {
        mParticleSets[i]->fetchParticles(stream, hasIsosurface, hasSmoothing, hasAnisotropy, updateToUsd,
            updateParticlesToUsd, updateVelocities, debugVizOn);
    }

    for (int i = 0; i < mCloths.size(); ++i)
    {
        mCloths[i]->fetchParticles(stream, updateToUsd, updateVelocities, debugVizOn);
    }
}

void InternalPbdParticleSystem::setPost(const uint32_t postFlags)
{
    if (postFlags == ParticlePostFlag::eNone)
    {
        if (mHasPost)
        {
            particles::releasePostprocess(mPath, this);
            mHasPost = false;
        }
    }
    else if (mHasPost)
    {
        particles::setPostprocessStages(mPath, postFlags);
    }
    else
    {
        particles::createPostprocess(mPath, postFlags, this);
        mHasPost = true;
    }
}

void InternalPbdParticleSystem::enablePost(omni::physx::ParticlePostFlag::Enum flag, const bool enable)
{
    uint32_t currentPostFlags = particles::getPostprocessStages(mPath);
    uint32_t newPostFlags = enable ? (currentPostFlags | flag) : (currentPostFlags & ~flag);
    setPost(newPostFlags);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

InternalParticleClothDeprecated::InternalParticleClothDeprecated(pxr::UsdPrim& prim, InternalPbdParticleSystem& particleSystem)
    : InternalParticle(particleSystem)
    , mPositions(nullptr)
    , mVelocities(nullptr)
    , mPhases(nullptr)
    , mClothBuffer(nullptr)
#if USE_PHYSX_GPU    
    , mAttachments(nullptr)
#endif
    , mUploadDirtyFlags(0)
    , mDownloadDirtyFlags(0)
{
    mPrim = prim;

    // store original user-provided xform ops
    mXformOpStorage.store(pxr::UsdGeomXformable(prim));

    mSkipUpdateTransform = false;
    const bool updateUSD = OmniPhysX::getInstance().getISettings()->getAsBool(kSettingUpdateToUsd);
    if (updateUSD)
    {
        if (OmniPhysX::getInstance().getSimulationLayer())
        {
            pxr::UsdEditContext editContext(OmniPhysX::getInstance().getStage(), UsdEditTarget(OmniPhysX::getInstance().getSimulationLayer()));
            // if we fail to setup transformations skip the prim for update
            if (!setupTransformOpsAsScaleOrientTranslate(prim))
            {
                CARB_LOG_WARN("Failed to create transformation stack (position, quat, scale) on a particle cloth: %s, ParticleCloth won't get transformation updates.",
                    prim.GetPrimPath().GetText());
                mSkipUpdateTransform = true;
            }
        }
        else
        {
            // if we fail to setup transformations skip the prim for update
            if (!setupTransformOpsAsScaleOrientTranslate(prim))
            {
                CARB_LOG_WARN("Failed to create transformation stack (position, quat, scale) on a particle cloth: %s", prim.GetPrimPath().GetText());
                mSkipUpdateTransform = true;
            }
        }
    }
}

InternalParticleClothDeprecated::~InternalParticleClothDeprecated()
{
#if USE_PHYSX_GPU
    mParticleClothBuffer->release();
    mVolumeBuffer->release();
    SAFE_DELETE_SINGLE(mAttachments);
#endif

    releasePinnedBuffers();

    if (mClothBuffer)
    {
        SAFE_RELEASE(mClothBuffer); // will remove from particle system
    }

    // Remove material reference
    InternalPBDParticleMaterial* internalMaterial = getInternalPtr<InternalPBDParticleMaterial>(ePTPBDMaterial, mMaterialId);
    if (internalMaterial)
    {
        ObjectId id = getObjectId(mPrim.GetPath(), ePTParticleClothDeprecated);
        internalMaterial->removeParticleId(id);
    }
}

void InternalParticleClothDeprecated::addRigidAttachment(PxRigidActor* actor, PxU32 particleId, PxVec3 actorSpacePose)
{
#if USE_PHYSX_GPU
    if (!mAttachments && mParentParticleSystem)
        mAttachments = PxCreateParticleAttachmentBuffer(*mClothBuffer, *mParentParticleSystem->mPS);
    
    mAttachments->addRigidAttachment(actor, particleId, actorSpacePose);
#endif
}

void InternalParticleClothDeprecated::removeRigidAttachment(PxRigidActor* actor, PxU32 particleId)
{
#if USE_PHYSX_GPU
    if (mAttachments)
    {
        mAttachments->removeRigidAttachment(actor, particleId);
    }
#endif
}

void InternalParticleClothDeprecated::addRigidFilter(PxRigidActor* rigidActor, const PxU32 particleID)
{
#if USE_PHYSX_GPU
    if (!mAttachments && mParentParticleSystem)
        mAttachments = PxCreateParticleAttachmentBuffer(*mClothBuffer, *mParentParticleSystem->mPS);

    mAttachments->addRigidFilter(rigidActor, particleID);
#endif
}

void InternalParticleClothDeprecated::removeRigidFilter(PxRigidActor* rigidActor, const PxU32 particleID)
{
#if USE_PHYSX_GPU
    if (mAttachments)
    {
        mAttachments->removeRigidFilter(rigidActor, particleID);
    }
#endif
}

void InternalParticleClothDeprecated::enableCloth(bool enable)
{
    if (enable == mEnabled)
        return;

    bool hasBufferAndPsEnabled = mClothBuffer && mParentParticleSystem && mParentParticleSystem->mEnabled;
    if (!hasBufferAndPsEnabled)
        return;

    if (enable)
    {
        mParentParticleSystem->mPS->addParticleBuffer(mClothBuffer);
    }
    else
    {
        mParentParticleSystem->mPS->removeParticleBuffer(mClothBuffer);
    }
    mEnabled = enable;
}

void InternalParticleClothDeprecated::uploadParticles(CUstream stream)
{
     if (!mEnabled)
        return;

    // TODO: move initial upload to async as well.
    // TODO: add a dirtyTopology flag and copy directly into the particle system if topology doesn't change.

    if (mUploadDirtyFlags)
    {
        PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
        if (!cudaContextManager)
            return;

        PxCudaContext* cudaContext = cudaContextManager->getCudaContext();
        cudaContextManager->acquireContext();

        if ((mUploadDirtyFlags & ParticleBufferFlags::ePOSITIONS) && mPositions)
        {
            cudaContext->memcpyHtoDAsync(CUdeviceptr(mClothBuffer->getPositionInvMasses()), mPositions, mNumParticles * sizeof(PxVec4), stream);
            mClothBuffer->raiseFlags(PxParticleBufferFlag::eUPDATE_POSITION);
        }
        if ((mUploadDirtyFlags & ParticleBufferFlags::eVELOCITIES) && mVelocities)
        {
            cudaContext->memcpyHtoDAsync(CUdeviceptr(mClothBuffer->getVelocities()), mVelocities, mNumParticles * sizeof(PxVec4), stream);
            mClothBuffer->raiseFlags(PxParticleBufferFlag::eUPDATE_VELOCITY);
        }
        if ((mUploadDirtyFlags & ParticleBufferFlags::ePHASES) && mPhases)
        {
            cudaContext->memcpyHtoDAsync(CUdeviceptr(mClothBuffer->getPhases()), mPhases, mNumParticles * sizeof(PxU32), stream);
            mClothBuffer->raiseFlags(PxParticleBufferFlag::eUPDATE_PHASE);
        }

        cudaContextManager->releaseContext();

        mUploadDirtyFlags = 0;
    }
}

void InternalParticleClothDeprecated::fetchParticles(CUstream stream, bool updateToUsd,
        bool updateVelocities, bool debugVizOn)
{
    if (mClothBuffer && mEnabled)
    {
        mDownloadDirtyFlags = 0;

        // AD: Because particles only work with USD now, we use the updateToUsd setting to initiate DtoH transfers
        // this needs to be changed for fabric support, where the dma logic needs to be decoupled slightly from these
        // flags. Although we still want to figure out what we need to download.

        // luckily for cloth this is not super complicated, we basically always need the positions, and need to figure
        // out whether we want velocities or not

        // we will always need the volumes for picking.

        if (updateToUsd || debugVizOn)
            mDownloadDirtyFlags |= ParticleDirtyFlags::ePOSITION_INVMASS;

        if (updateVelocities && updateToUsd)
            mDownloadDirtyFlags |= ParticleDirtyFlags::eVELOCITY;

#if USE_PHYSX_GPU
        if (mNumVolumes && mVolumeBuffer)
            mDownloadDirtyFlags |= ParticleDirtyFlags::eVOLUME;
#endif

        if (mDownloadDirtyFlags)
        {
            PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
            if (!cudaContextManager)
                return;

            PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

            cudaContextManager->acquireContext();

            if (mDownloadDirtyFlags & ParticleDirtyFlags::ePOSITION_INVMASS)
                cudaContext->memcpyDtoHAsync(mPositions, CUdeviceptr(mClothBuffer->getPositionInvMasses()), mNumParticles * sizeof(PxVec4), stream);

            if (mDownloadDirtyFlags & ParticleDirtyFlags::eVELOCITY)
                cudaContext->memcpyDtoHAsync(mVelocities, CUdeviceptr(mClothBuffer->getVelocities()), mNumParticles * sizeof(PxVec4), stream);

#if USE_PHYSX_GPU
            if (mDownloadDirtyFlags & ParticleDirtyFlags::eVOLUME)
                cudaContext->memcpyDtoHAsync(mVolumeBuffer->getParticleVolumes(), CUdeviceptr(mClothBuffer->getParticleVolumes()), mNumVolumes * sizeof(PxParticleVolume), stream);
#endif

            cudaContextManager->releaseContext();
        }
    }
}

void InternalParticleClothDeprecated::releasePinnedBuffers()
{
#if USE_PHYSX_GPU
    PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
    if (cudaContextManager)
    {
        PX_PINNED_HOST_FREE(cudaContextManager, mPositions);
        PX_PINNED_HOST_FREE(cudaContextManager, mVelocities);
        PX_PINNED_HOST_FREE(cudaContextManager, mPhases);
    }
#endif
}
