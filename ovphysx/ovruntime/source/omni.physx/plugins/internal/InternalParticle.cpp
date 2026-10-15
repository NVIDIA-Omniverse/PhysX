// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-SIM-PARTICLESET-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-SIM-PARTICLESET-002
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6
 */

// PhysXParticlePost.h defines PostProcessCallback, which mCallback below needs as a
// complete type for SAFE_DELETE_SINGLE to route through Allocateable::operator delete --
// deleting it as an incomplete type silently calls plain global operator delete on the
// raw pointer instead (skipping the allocator's own header offset), corrupting the heap
// on free. The header is itself pxr-free (ParticlePostprocess authors through the write sink).
#include "particles/PhysXParticlePost.h"
#include <common/utilities/MemoryMacros.h>

#include "InternalParticle.h"

#include <PhysXTools.h>
#include <extensions/PxCudaHelpersExt.h>
#include <PhysXSettings.h>

#include <usdLoad/LoadUsd.h>
#include <usdLoad/Particles.h>
#include <usdLoad/IceDescriptorAllocator.h>

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>
#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/IPhysicsDataWrite.h>


#if USE_PHYSX_GPU
#include "extensions/PxParticleExt.h"
#endif

using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace carb;
using namespace ::physx;
using namespace ::physx::Ext;


namespace
{
// The native particle system supplies the source owner. A missing write sink
// makes this rendering update a no-op.
void setDiffuseParticleRenderingEnabledInSink(usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey particleSystemKey, bool enabled)
{
    usdparser::AttachedStage* as = &owner;
    omni::physics::parse::IPhysicsDataWrite* dw = as ? as->getDataWrite() : nullptr;
    if (dw)
        dw->setDiffuseParticleRenderingEnabled(particleSystemKey, enabled);
}
} // namespace

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
    PxCudaContextManager* cudaContextManager = mPhysXScene ? mPhysXScene->getScene()->getCudaContextManager() : nullptr;
    if (cudaContextManager)
    {
        if (mPositions)
        {
            PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mPositions);
        }

        if (mVelocities)
        {
            PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mVelocities);
        }

        if (mPhases)
        {
            PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mPhases);
        }

        if (mDiffuseParticlePositions)
        {
            PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mDiffuseParticlePositions);
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
    , mFluid(false)
    , mParticleInvMass(0.0f)
    , mPhase(0)
{
}

InternalParticleSet::~InternalParticleSet()
{
    releasePinnedBuffers();

    SAFE_RELEASE(mParticleBuffer); // will remove from particle system
    SAFE_RELEASE(mReplacementBuffer);

    // Remove material reference
    InternalPBDParticleMaterial* internalMaterial = getInternalPtr<InternalPBDParticleMaterial>(mPhysXScene->getAttachedStage().getInternalPhysXDatabase(), ePTPBDMaterial, mMaterialId);
    if (internalMaterial)
    {
        usdparser::ObjectId id = getObjectDataOrID<ObjectDataQueryType::eOBJECT_ID>(mKey, ePTParticleSet, mPhysXScene->getAttachedStage().getInternalPhysXDatabase(), mPhysXScene->getAttachedStage());
        internalMaterial->removeParticleId(id);
    }
}

uint32_t InternalParticleSet::getMaxDiffuseParticles(uint32_t maxParticles,
    bool diffuseEnabled, float maxDiffuseParticleMultiplier)
{
    return diffuseEnabled ? static_cast<uint32_t>(maxParticles * maxDiffuseParticleMultiplier) : 0;
}

PxParticleAndDiffuseBuffer* InternalParticleSet::createUserBuffer(uint32_t maxParticles, uint32_t numParticles,
    bool diffuseEnabled, float maxDiffuseParticleMultiplier, PxDiffuseParticleParams& diffuseParams, PxCudaContextManager* cudaContextManager)
{
    PxParticleAndDiffuseBuffer* newBuffer = nullptr;
    if (cudaContextManager)
    {
        PxPhysics* physics = OmniPhysX::getInstance().getPhysXSetup().getPhysics();
        uint32_t maxDiffuseParticles = InternalParticleSet::getMaxDiffuseParticles(maxParticles, diffuseEnabled, maxDiffuseParticleMultiplier);
        uint32_t maxDiffuseParticlesBuffer = std::max<uint32_t>(maxDiffuseParticles, 1);
        uint32_t maxParticlesBuffer = std::max<uint32_t>(maxParticles, 1);
        newBuffer = physics->createParticleAndDiffuseBuffer(maxParticlesBuffer, maxDiffuseParticlesBuffer, cudaContextManager);

        newBuffer->setNbActiveParticles(numParticles);
        newBuffer->setMaxActiveDiffuseParticles(maxDiffuseParticles);
        newBuffer->setDiffuseParticleParams(diffuseParams);
    }

    return newBuffer;
}

void InternalParticleSet::createSharedDiffuseParticles()
{
    if (!mParentParticleSystem)
        return; // TODO should this spit out a warning?

    if (!mParentParticleSystem->mDiffuseParticleRenderingEnabled)
    {
        setDiffuseParticleRenderingEnabledInSink(mParentParticleSystem->mPhysXScene->getAttachedStage(), mParentParticleSystem->mKey, true);
        mParentParticleSystem->mDiffuseParticleRenderingEnabled = true;
    }

    if (!mHasSharedDiffuseParticles)
    {
        mParentParticleSystem->mDiffuseParticleInstanceRefCount++;
        mHasSharedDiffuseParticles = true;
    }
}

void InternalParticleSet::releaseSharedDiffuseParticles()
{
    if (mHasSharedDiffuseParticles && mParentParticleSystem)
    {
        if (mParentParticleSystem->mDiffuseParticleInstanceRefCount == 1)
        {
            setDiffuseParticleRenderingEnabledInSink(mParentParticleSystem->mPhysXScene->getAttachedStage(), mParentParticleSystem->mKey, false);
            mParentParticleSystem->mDiffuseParticleRenderingEnabled = false;
        }
        mParentParticleSystem->mDiffuseParticleInstanceRefCount--;
        mHasSharedDiffuseParticles = false;
    }
}

void InternalParticleSet::changeDiffuseParticles(bool remove)
{
#if USE_PHYSX_GPU
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    PxCudaContextManager* cudaContextManager = mPhysXScene->getScene()->getCudaContextManager();
    if (!cudaContextManager)
        return;

    usdparser::AttachedStage* attachedStage = &mParentParticleSystem->mPhysXScene->getAttachedStage();
    if (!attachedStage)
        return;

    PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mDiffuseParticlePositions);

    if (remove)
    {
        mDiffuseParticlesEnabled = false;
        mDiffuseParticleParams = PxDiffuseParticleParams();
        releaseSharedDiffuseParticles();
    }
    else
    {
        // Re-read the set's diffuse params through the parse library
        // (source-backed, no direct USD).
        const omni::physics::parse::IPhysicsSource* src = attachedStage->getSource();
        if (src)
        {
            omni::physics::parse::ParseContext ctx(const_cast<omni::physics::parse::IPhysicsSource&>(*src), usdparser::iceDescriptorAllocator());
            if (omni::physics::parse::DescPtr<omni::physics::parse::ParticleSetDesc> particleSetDesc =
                    omni::physics::parse::parseParticleSet(ctx, mKey))
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
            }
        }

        const uint32_t maxDiffuseParticles = getMaxDiffuseParticles(mMaxParticles, mDiffuseParticlesEnabled, mMaxDiffuseParticleMultiplier);
        if (maxDiffuseParticles > 0)
        {
            mDiffuseParticlePositions = PxCudaHelpersExt::allocPinnedHostBuffer<PxVec4>(*cudaContextManager, maxDiffuseParticles);
            createSharedDiffuseParticles();
            // as set creation does, so that a later removal of the API tears this down
            attachedStage->getObjectDatabase()->addSchemaAPI(mKey, SchemaAPIFlag::eDiffuseParticlesAPI);
        }
    }

    mNumDiffuseParticles = 0;

    SAFE_RELEASE(mReplacementBuffer);
    mReplacementBuffer = createUserBuffer(mMaxParticles, mNumParticles, mDiffuseParticlesEnabled, mMaxDiffuseParticleMultiplier, mDiffuseParticleParams, cudaContextManager);
#endif
}

void InternalParticleSet::enableDiffuseParticles(bool enabled)
{
    mDiffuseParticlesEnabled = enabled;
    uint32_t maxDiffuseParticles = getMaxDiffuseParticles(mMaxParticles, mDiffuseParticlesEnabled, mMaxDiffuseParticleMultiplier);

    // If diffuse particles were disabled when the set's buffers were last built, the PhysX buffer has room for
    // only one diffuse particle and mDiffuseParticlePositions was not allocated. Enabling them then rebuilds
    // the PhysX buffer and allocates mDiffuseParticlePositions, both for the enabled count.
    const bool physxBufferTooSmall = maxDiffuseParticles > mParticleBuffer->getMaxDiffuseParticles();
    const bool hostBufferMissing = maxDiffuseParticles > 0 && !mDiffuseParticlePositions;
    if (physxBufferTooSmall || hostBufferMissing)
    {
        changeDiffuseParticles(false);
    }
    else
    {
        mParticleBuffer->setMaxActiveDiffuseParticles(maxDiffuseParticles);
    }
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

    // buffers need to be resized
    static float growthFactor = 1.5f;
    uint32_t newMaxParticles = std::max(uint32_t(growthFactor*mMaxParticles), newNumParticles);

#if USE_PHYSX_GPU
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    PxPhysics* physics = physxSetup.getPhysics();
    PxCudaContextManager* cudaContextManager = mPhysXScene->getScene()->getCudaContextManager();
    if (!cudaContextManager)
        return;

    // resize all the pinned buffers: we skip copying the old data because this will be loaded from USD anyway.
    // A set created without points has none yet, so they are allocated here rather than only regrown.
    PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mPositions);
    mPositions = PxCudaHelpersExt::allocPinnedHostBuffer<PxVec4>(*cudaContextManager, newMaxParticles);

    PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mVelocities);
    mVelocities = PxCudaHelpersExt::allocPinnedHostBuffer<PxVec4>(*cudaContextManager, newMaxParticles);

    PxU32* newPhases = PxCudaHelpersExt::allocPinnedHostBuffer<PxU32>(*cudaContextManager, newMaxParticles);

    //copy and initialize surplus phases
    if (mNumParticles > 0)
        std::memcpy(newPhases, mPhases, mNumParticles * sizeof(PxU32));

    for (PxU32 i = mNumParticles; i < newNumParticles; ++i)
        newPhases[i] = mPhase;

    PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mPhases);
    mPhases = newPhases;
    mUploadDirtyFlags |= ParticleBufferFlags::ePHASES;

    // Sized for the new capacity, like the buffer createUserBuffer() makes below. A set created without points
    // has no diffuse host buffer yet, so growing it sets up what creation does for a set with diffuse capacity.
    const uint32_t maxDiffuseParticles = getMaxDiffuseParticles(newMaxParticles, mDiffuseParticlesEnabled, mMaxDiffuseParticleMultiplier);
    if (mDiffuseParticlePositions)
    {
        PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mDiffuseParticlePositions);
        mDiffuseParticlePositions = PxCudaHelpersExt::allocPinnedHostBuffer<PxVec4>(*cudaContextManager, maxDiffuseParticles);
    }
    else if (maxDiffuseParticles > 0)
    {
        mDiffuseParticlePositions = PxCudaHelpersExt::allocPinnedHostBuffer<PxVec4>(*cudaContextManager, maxDiffuseParticles);
        createSharedDiffuseParticles();
        mPhysXScene->getAttachedStage().getObjectDatabase()->addSchemaAPI(mKey, SchemaAPIFlag::eDiffuseParticlesAPI);
    }

    mMaxParticles = newMaxParticles;
    mNumParticles = newNumParticles;
    PxParticleAndDiffuseBuffer* newBuffer = createUserBuffer(mMaxParticles, mNumParticles, mDiffuseParticlesEnabled,
        mMaxDiffuseParticleMultiplier, mDiffuseParticleParams, cudaContextManager);
    newBuffer->raiseFlags(PxParticleBufferFlag::eALL);

    mParticleBuffer->release(); // will also remove buffer form particle system
    mParticleBuffer = newBuffer;

    // a replacement queued by changeDiffuseParticles() has the old capacity; the new buffer already has its settings
    SAFE_RELEASE(mReplacementBuffer);

    if (mEnabled && mParentParticleSystem && mParentParticleSystem->mEnabled)
        mParentParticleSystem->mPS->addParticleBuffer(mParticleBuffer);

    // Postprocess-generator cache invalidation only; particles::notifyParticleSystemResize is
    // ObjectKey-keyed (a no-op here: the postprocess registry it queries is always empty).
    particles::notifyParticleSystemResize(mPhysXScene->getAttachedStage(), mParentParticleSystem->mKey);

    // need to udpate the Physx pointer for the particle set.
    {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        internal::InternalPhysXDatabase& db = mPhysXScene->getAttachedStage().getInternalPhysXDatabase();
        usdparser::ObjectId id = getObjectDataOrID<ObjectDataQueryType::eOBJECT_ID>(mKey, ePTParticleSet, mPhysXScene->getAttachedStage().getInternalPhysXDatabase(), mPhysXScene->getAttachedStage());
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
    PxCudaContextManager* cudaContextManager = mPhysXScene ? mPhysXScene->getScene()->getCudaContextManager() : nullptr;
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
        // Diffuse edits made while the replacement was pending only reached the old buffer.
        const uint32_t maxDiffuseParticles = getMaxDiffuseParticles(mMaxParticles, mDiffuseParticlesEnabled, mMaxDiffuseParticleMultiplier);
        mReplacementBuffer->setMaxActiveDiffuseParticles(std::min(maxDiffuseParticles, mReplacementBuffer->getMaxDiffuseParticles()));
        mReplacementBuffer->setDiffuseParticleParams(mDiffuseParticleParams);

        mParticleBuffer->release(); // removes buffer from particle system

        mParticleBuffer = mReplacementBuffer;
        mReplacementBuffer = nullptr;

        if (mEnabled && mParentParticleSystem && mParentParticleSystem->mEnabled)
            mParentParticleSystem->mPS->addParticleBuffer(mParticleBuffer);

        // need to udpate the Physx pointer for the particle set.
        {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            internal::InternalPhysXDatabase& db = mPhysXScene->getAttachedStage().getInternalPhysXDatabase();

            PhysXType internalType = ePTRemoved;
            usdparser::ObjectId id = getObjectDataOrID<ObjectDataQueryType::eOBJECT_ID>(mKey, ePTParticleSet, mPhysXScene->getAttachedStage().getInternalPhysXDatabase(), mPhysXScene->getAttachedStage());
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
            cudaContext->memcpyHtoDAsync(CUdeviceptr(mParticleBuffer->getPositionInvMasses()), mPositions, mNumParticles * sizeof(PxVec4), stream);

            mParticleBuffer->raiseFlags(PxParticleBufferFlag::eUPDATE_POSITION);
        }

        if ((mUploadDirtyFlags & ParticleBufferFlags::eVELOCITIES) && mVelocities)
        {
            cudaContext->memcpyHtoDAsync(CUdeviceptr(mParticleBuffer->getVelocities()), mVelocities, mNumParticles * sizeof(PxVec4), stream);

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
        PxCudaContextManager* cudaContextManager = mPhysXScene ? mPhysXScene->getScene()->getCudaContextManager() : nullptr;
        if (!cudaContextManager)
            return;

        PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

        // calculate download flags and set them for transform updates.
        mDownloadDirtyFlags = 0;

        // TODO: how does this change with vec4-vec3 + transforms on gpu?
        // TODO: how does this change with direct gpu interop?

        // These download flags are conceptually independent of updateToUsd, but must still respect those settings.
        // Purpose: 1) track what has been calculated on the GPU, 2) tell the transform-update function what data
        // was fetched from the GPU, so its fine-grained updateToUsd logic applies correctly. They matter even when
        // updateToUsd is off, if the GPU->CPU transfer is still needed.

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

        mNumDiffuseParticles = mParticleBuffer->getNbActiveDiffuseParticles();
        if ((mNumDiffuseParticles > 0 && mDiffuseParticlePositions))
            mDownloadDirtyFlags |= ParticleDirtyFlags::eDIFFUSE_PARTICLES;

        if (hasSmoothing && mFluid && (!hasIsosurface))
        {
            mDownloadDirtyFlags |= ParticleDirtyFlags::eSMOOTHED_POSITIONS;
        }

        if (hasAnisotropy && mFluid && (!hasIsosurface))
            mDownloadDirtyFlags |= ParticleDirtyFlags::eANISOTROPY;

        if (mDownloadDirtyFlags)
        {
            cudaContextManager->acquireContext();

            if (mDownloadDirtyFlags & ParticleDirtyFlags::ePOSITION_INVMASS)
            {
                cudaContext->memcpyDtoHAsync(mPositions, CUdeviceptr(mParticleBuffer->getPositionInvMasses()), mNumParticles * sizeof(PxVec4), stream);
            }

            if (mDownloadDirtyFlags & ParticleDirtyFlags::eVELOCITY)
            {
                cudaContext->memcpyDtoHAsync(mVelocities, CUdeviceptr(mParticleBuffer->getVelocities()), mNumParticles * sizeof(PxVec4), stream);
            }

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
    // InternalParticleSet instances need to be removed from PhysX DB
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

    setPost(ParticlePostFlag::eNone);

    if (mDiffuseParticleRenderingEnabled)
    {
        setDiffuseParticleRenderingEnabledInSink(mPhysXScene->getAttachedStage(), mKey, false);
        mDiffuseParticleRenderingEnabled = false;
    }

    if (mPS)
    {
        // Detach the actor (and its callback registration) from the scene, and
        // explicitly clear the callback pointer, before mCallback is freed below --
        // defensive ordering so the actor can't invoke/hold a reference to mCallback
        // after it is deleted. (The heap-corruption crash once seen here traced to a
        // separate bug -- an incomplete PostProcessCallback type at the delete site, see
        // the #include note at the top of this file -- so this reordering alone did not
        // fix it, but is kept as good practice.)
        if (mPS->getScene())
        {
            mPS->getScene()->removeActor(*mPS, false);
        }
#if USE_PHYSX_GPU
        mPS->setParticleSystemCallback(nullptr);
#endif
        SAFE_RELEASE(mPS);
    }

#if USE_PHYSX_GPU
    SAFE_DELETE_SINGLE(mCallback);
#endif
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

        mPS->getScene()->removeActor(*mPS);
    }
    mEnabled = enable;
}

void InternalPbdParticleSystem::removeParticleObjectsFromDB()
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    internal::InternalPhysXDatabase& db = mPhysXScene->getAttachedStage().getInternalPhysXDatabase();

    for (uint32_t i = 0; i < mParticleSets.size(); i++)
    {
        usdparser::ObjectId id = getObjectDataOrID<ObjectDataQueryType::eOBJECT_ID>(mParticleSets[i]->mKey, ePTParticleSet, mPhysXScene->getAttachedStage().getInternalPhysXDatabase(), mPhysXScene->getAttachedStage());
        InternalDatabase::Record* objectRecord = db.getFullTypedRecord(ePTParticleSet, id);
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

void InternalPbdParticleSystem::uploadParticles(CUstream stream)
{
    for (int i = 0; i < mParticleSets.size(); ++i)
    {
        mParticleSets[i]->uploadParticles(stream);
    }
}

void InternalPbdParticleSystem::fetchParticles(CUstream stream)
{
    if (!mParticleDataAvailable || !mEnabled || mPhysXScene->isReadbackSuppressed())
        return;

    // particles::getPostprocessStages is ObjectKey-keyed: the postprocess registry it
    // queries is always empty in OvruntimePhysX (see setPost() below), so postprocessing
    // is always disabled.
    uint32_t postFlags = particles::getPostprocessStages(mPhysXScene->getAttachedStage(), mKey);
    bool hasIsosurface = postFlags & ParticlePostFlag::eIsosurface;
    bool hasSmoothing = postFlags & ParticlePostFlag::eSmoothing;
    bool hasAnisotropy = postFlags & ParticlePostFlag::eAnisotropy;

    const OmniCachedSettings& settings = mPhysXScene->getAttachedStage().getCachedSettings();
    const bool debugVizOn = settings.visualizationDisplayParticles > 0;
    const bool updateParticlesToUsd = settings.updateParticlesToUsd;
    const bool updateVelocities = settings.updateVelocitiesToUsd;
    const bool updateToUsd = settings.updateToUsd;

    for (int i = 0; i < mParticleSets.size(); ++i)
    {
        mParticleSets[i]->fetchParticles(stream, hasIsosurface, hasSmoothing, hasAnisotropy, updateToUsd,
            updateParticlesToUsd, updateVelocities, debugVizOn);
    }
}

void InternalPbdParticleSystem::setPost(const uint32_t postFlags)
{
    // The particles:: postprocess subsystem (isosurface/smoothing/anisotropy Hydra rendering,
    // GPU generator setup aside) publishes through the write sink and no-ops there with none
    // (no attached stage, or an ovstage attach with no write sink), so this bookkeeping is
    // harmless either way (mHasPost still tracks it consistently, just never against a live
    // postprocess when createPostprocess found no attached stage to register against).
    if (postFlags == ParticlePostFlag::eNone)
    {
        if (mHasPost)
        {
            particles::releasePostprocess(mPhysXScene->getAttachedStage(), mKey, this);
            mHasPost = false;
        }
    }
    else if (mHasPost)
    {
        particles::setPostprocessStages(mPhysXScene->getAttachedStage(), mKey, postFlags);
    }
    else
    {
        particles::createPostprocess(mPhysXScene->getAttachedStage(), mKey, postFlags, this);
        mHasPost = true;
    }
}

void InternalPbdParticleSystem::enablePost(omni::physx::ParticlePostFlag::Enum flag, const bool enable)
{
    uint32_t currentPostFlags = particles::getPostprocessStages(mPhysXScene->getAttachedStage(), mKey);
    uint32_t newPostFlags = enable ? (currentPostFlags | flag) : (currentPostFlags & ~flag);
    setPost(newPostFlags);
}
