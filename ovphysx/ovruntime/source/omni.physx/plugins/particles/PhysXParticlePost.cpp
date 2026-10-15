// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-9
 *
 * Three stage-dependent sites, three deliberately different outcomes:
 *   - `updateFromPoints` skips a prim-existence PRECONDITION whose subject it never
 *     uses again, and continues -- gating the function would silently disable the
 *     GPU post-processing below it;
 *   - `addParticleSet` REFUSES with a diagnostic, because its work is the source
 *     read the caller asked for;
 *   - `updateMesh` publishes unconditionally; the sink itself no-ops the write when
 *     the session-layer mesh it targets was never defined (see
 *     UsdPhysicsDataWrite::writeIsosurfaceMesh);
 *   - `updatePoints` RETURNS before its stage read, because that read
 *     (getWorldTransform) only feeds sink writes that already no-op without a stage.
 */

// @implements REQ-ATTACH-IO-001
// @covers AC-3

#include "ScopedNoticeLock.h"
#include "PhysXParticlePost.h"

#include <PhysXTools.h>
#include <extensions/PxCudaHelpersExt.h>
#include <common/foundation/MatrixTools.h>
#include <common/utilities/MemoryMacros.h>

#include "../internal/InternalParticle.h"

#include <usdLoad/LoadUsd.h>
#include <usdLoad/Particles.h>
#include <usdLoad/IceDescriptorAllocator.h>

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>
#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/IPhysicsDataWrite.h>
#include <omni/physics/parse/KnownTokens.h>

#if USE_PHYSX_GPU
#include "PxIsosurfaceExtraction.h"
#include "PxAnisotropy.h"
#include "PxSmoothing.h"
#include "PxParticleNeighborhoodProvider.h"
#include "gpu/PxPhysicsGpu.h"
#endif

#include <unordered_map>

using namespace carb;
using namespace ::physx;
using namespace ::physx::Ext;
using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::particles;
using namespace omni::physx::usdparser;

namespace
{
using omni::physics::parse::DataType;
using omni::physics::parse::DataWriteView;
using omni::physics::parse::IPhysicsDataWrite;
using omni::physics::parse::IPhysicsSource;
using omni::physics::parse::KnownTokens;
using omni::physics::parse::ObjectKey;
using omni::physics::parse::ReadTime;

// Toggle the purpose=proxy visibility hint the isosurface pipeline uses to
// hide raw fluid particles behind its substitute render.
void writeProxyPurposeToSink(AttachedStage& owner, ObjectKey key, bool proxy)
{
    AttachedStage* as = &owner;
    IPhysicsDataWrite* dw = as ? as->getDataWrite() : nullptr;
    if (dw)
        dw->writeProxyPurpose(key, proxy);
}

void setIsosurfaceMeshEnabledInSink(AttachedStage& owner, ObjectKey particleSystemKey, bool enabled)
{
    AttachedStage* as = &owner;
    IPhysicsDataWrite* dw = as ? as->getDataWrite() : nullptr;
    if (dw)
        dw->setIsosurfaceMeshEnabled(particleSystemKey, enabled);
}

void writeIsosurfaceMeshToSink(AttachedStage& owner, ObjectKey particleSystemKey,
                               const carb::Float3* points, size_t numPoints,
                               const carb::Float3* normals, size_t numNormals,
                               const int32_t* faceVertexCounts, size_t numFaces,
                               const int32_t* faceVertexIndices, size_t numIndices)
{
    AttachedStage* as = &owner;
    IPhysicsDataWrite* dw = as ? as->getDataWrite() : nullptr;
    if (dw)
        dw->writeIsosurfaceMesh(particleSystemKey, points, numPoints, normals, numNormals,
                                faceVertexCounts, numFaces, faceVertexIndices, numIndices);
}

// Writes a particle set's whole points/positions array (point-based vs.
// point-instancer resolved from the destination prim's type).
void writePointsToSink(AttachedStage& owner, ObjectKey key, const carb::Float3* data, size_t count)
{
    AttachedStage* as = &owner;
    const IPhysicsSource* src = as ? as->getSource() : nullptr;
    IPhysicsDataWrite* dw = as ? as->getDataWrite() : nullptr;
    if (!src || !dw)
        return;
    KnownTokens tok;
    tok.intern(*src);
    const bool isInstancer = src->isA(key, tok.pointInstancerType);
    dw->writeArray(key, isInstancer ? tok.positions : tok.points,
                   DataWriteView{ data, count, 0, -1, DataType::e32Bit });
}

// Backs up a particle set's current points/positions into the
// physxParticle:simulationPoints attribute, once (skipped once a backup is
// already authored -- matches HasAuthoredValue()'s "already backed up" gate).
void backupSimulationPointsToSink(AttachedStage& owner, ObjectKey key)
{
    AttachedStage* as = &owner;
    const IPhysicsSource* src = as ? as->getSource() : nullptr;
    IPhysicsDataWrite* dw = as ? as->getDataWrite() : nullptr;
    if (!src || !dw)
        return;
    KnownTokens tok;
    tok.intern(*src);
    if (src->hasAuthoredAttribute(key, tok.physxParticleSimulationPoints))
        return;
    const bool isInstancer = src->isA(key, tok.pointInstancerType);
    std::vector<carb::Float3> points;
    getArrayValue(*as, key, isInstancer ? tok.positions : tok.points, ReadTime::defaultTime(), points);
    dw->writeArray(key, tok.physxParticleSimulationPoints,
                   DataWriteView{ points.data(), points.size(), 0, -1, DataType::e32Bit });
}

// Restores a particle set's points/positions from its physxParticle:simulationPoints
// backup (if any) and drops the backup opinion, so a later backup call sees
// "not yet backed up" again.
void restorePointsToSink(AttachedStage& owner, ObjectKey key)
{
    AttachedStage* as = &owner;
    const IPhysicsSource* src = as ? as->getSource() : nullptr;
    IPhysicsDataWrite* dw = as ? as->getDataWrite() : nullptr;
    if (!src || !dw)
        return;
    KnownTokens tok;
    tok.intern(*src);
    if (!src->hasSchema(key, tok.physxParticleSetAPI))
        return;
    if (!src->hasAuthoredAttribute(key, tok.physxParticleSimulationPoints))
        return;

    { ScopedNoticeBlock blockNotices(owner, true, false);

    std::vector<carb::Float3> srcPoints;
    getArrayValue(*as, key, tok.physxParticleSimulationPoints, ReadTime::defaultTime(), srcPoints);

    const bool isInstancer = src->isA(key, tok.pointInstancerType);
    dw->writeArray(key, isInstancer ? tok.positions : tok.points,
                   DataWriteView{ srcPoints.data(), srcPoints.size(), 0, -1, DataType::e32Bit });
    dw->clearArray(key, tok.physxParticleSimulationPoints);

    }
}

// Writes anisotropy-derived scale/orientation for a fluid particle set's
// point instancer prototypes; no-op on a non-instancer particle set.
void writePointInstancerScaleOrientToSink(AttachedStage& owner, ObjectKey key,
                                          const PxVec4* anisotropyQ1,
                                          const PxVec4* anisotropyQ2,
                                          const PxVec4* anisotropyQ3,
                                          size_t count,
                                          float contactDistanceInv)
{
    AttachedStage* as = &owner;
    const IPhysicsSource* src = as ? as->getSource() : nullptr;
    IPhysicsDataWrite* dw = as ? as->getDataWrite() : nullptr;
    if (!src || !dw)
        return;
    KnownTokens tok;
    tok.intern(*src);
    if (!src->isA(key, tok.pointInstancerType))
        return;

    std::vector<carb::Float3> scales(count);
    std::vector<carb::Float4> orientations(count);
    for (size_t i = 0; i < count; ++i)
    {
        const PxVec4& q1 = anisotropyQ1[i];
        const PxVec4& q2 = anisotropyQ2[i];
        const PxVec4& q3 = anisotropyQ3[i];
        // AD: I have no idea where this 4x factor is coming from, but it seems to be important.
        scales[i] = { 4.0f * contactDistanceInv * q1[3], 4.0f * contactDistanceInv * q2[3],
                     4.0f * contactDistanceInv * q3[3] };
        const PxQuat q = PxQuat(PxMat33(q1.getXYZ(), q2.getXYZ(), q3.getXYZ()));
        orientations[i] = { q.x, q.y, q.z, q.w };
    }

    dw->writeArray(key, tok.scales, DataWriteView{ scales.data(), scales.size(), 0, -1, DataType::e32Bit });
    dw->writeArray(key, tok.orientations, DataWriteView{ orientations.data(), orientations.size(), 0, -1, DataType::e32Bit });
}

// Resets a fluid particle set's point-instancer scale/orientation to
// identity, at whatever count is currently authored; no-op on a
// non-instancer particle set.
void restorePointInstancerScaleOrientToSink(AttachedStage& owner, ObjectKey key)
{
    AttachedStage* as = &owner;
    const IPhysicsSource* src = as ? as->getSource() : nullptr;
    IPhysicsDataWrite* dw = as ? as->getDataWrite() : nullptr;
    if (!src || !dw)
        return;
    KnownTokens tok;
    tok.intern(*src);
    if (!src->isA(key, tok.pointInstancerType))
        return;

    std::vector<carb::Float3> scales;
    getArrayValue(*as, key, tok.scales, ReadTime::defaultTime(), scales);
    std::fill(scales.begin(), scales.end(), carb::Float3{ 1.0f, 1.0f, 1.0f });

    std::vector<carb::Float4> orientations;
    getArrayValue(*as, key, tok.orientations, ReadTime::defaultTime(), orientations);
    std::fill(orientations.begin(), orientations.end(), carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f });

    dw->writeArray(key, tok.scales, DataWriteView{ scales.data(), scales.size(), 0, -1, DataType::e32Bit });
    dw->writeArray(key, tok.orientations, DataWriteView{ orientations.data(), orientations.size(), 0, -1, DataType::e32Bit });
}

// Reads a particle set's current points (preferring an authored
// physxParticle:simulationPoints backup), world-transforms them, and appends
// to dstPoints -- the "no live simulation" preview data source.
void appendParticleSetPointsFromSink(AttachedStage& owner, std::vector<PxVec4>& dstPoints, ObjectKey key)
{
    AttachedStage* as = &owner;
    const IPhysicsSource* src = as ? as->getSource() : nullptr;
    if (!src)
        return;
    KnownTokens tok;
    tok.intern(*src);

    std::vector<carb::Float3> srcPoints;
    if (src->hasAuthoredAttribute(key, tok.physxParticleSimulationPoints))
    {
        getArrayValue(*as, key, tok.physxParticleSimulationPoints, ReadTime::defaultTime(), srcPoints);
    }
    else
    {
        const bool isInstancer = src->isA(key, tok.pointInstancerType);
        getArrayValue(*as, key, isInstancer ? tok.positions : tok.points, ReadTime::defaultTime(), srcPoints);
    }

    const PxMat44d localToWorld = getWorldTransform(*as, key, ReadTime::defaultTime());
    dstPoints.reserve(dstPoints.size() + srcPoints.size());
    for (size_t p = 0; p < srcPoints.size(); ++p)
    {
        const carb::Float3& local = srcPoints[p];
        const PxVec3d worldPos = localToWorld.transform(PxVec3d(local.x, local.y, local.z));
        dstPoints.push_back({ float(worldPos.x), float(worldPos.y), float(worldPos.z), 0.0f });
    }
}
} // namespace

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

ParticlePostprocess::ParticlePostprocess(AttachedStage& owner, omni::physics::parse::ObjectKey particleSystemKey)
    : mAttachedStage(owner), mParticleSystemKey(particleSystemKey)
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
        PxCudaContextManager* cudaContextManager = mCudaContextManager;
        if (cudaContextManager)
        {
            PxCudaHelpersExt::freeDeviceBuffer(*cudaContextManager, mParticlesPreview);
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
    if (mParent)
    {
        // update max particles
        mMaxParticlesCallback = 0;
        if (mParent)
        {
            for (size_t i = 0; i < mParent->mParticleSets.size(); ++i)
            {
                mMaxParticlesCallback += mParent->mParticleSets[i]->mMaxParticles;
            }
        }

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
    if (parent && parent->mPhysXScene)
        mCudaContextManager = parent->mPhysXScene->getScene()->getCudaContextManager();
#endif
}

bool ParticlePostprocess::parseParticleContactOffset(float& particleContactOffset)
{
    // Post-processing has no scanned stage; re-read the system descriptor on
    // demand through the parse library (source-backed, no direct USD).
    AttachedStage* attachedStage = &mAttachedStage;
    const omni::physics::parse::IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr;
    if (!src)
        return false;
    if (!src->exists(mParticleSystemKey))
        return false;
    omni::physics::parse::ParseContext ctx(const_cast<omni::physics::parse::IPhysicsSource&>(*src), iceDescriptorAllocator());
    if (omni::physics::parse::DescPtr<omni::physics::parse::ParticleSystemDesc> desc = omni::physics::parse::parseParticleSystem(ctx, mParticleSystemKey))
    {
        particleContactOffset = desc->particleContactOffset;
        return true;
    }
    return false;
}

void ParticlePostprocess::createNeighborhoodProvider(float cellSize, const PxU32 maxParticles)
{
#if USE_PHYSX_GPU
    if (!mNeighborhoodProvider && maxParticles > 0)
    {
        PxCudaContextManager* cudaContextManager = mCudaContextManager;
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
    AttachedStage* attachedStage = &mAttachedStage;
    const omni::physics::parse::IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr;
    if (!src)
    {
        return;
    }

    PxCudaContextManager* cudaContextManager = mCudaContextManager;
    if (!cudaContextManager)
    {
        return;
    }

    omni::physics::parse::ParseContext ctx(const_cast<omni::physics::parse::IPhysicsSource&>(*src), iceDescriptorAllocator());
    omni::physics::parse::DescPtr<omni::physics::parse::ParticleAnisotropyDesc> desc =
        omni::physics::parse::parseParticleAnisotropy(ctx, mParticleSystemKey);
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
            PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mAnisotropyQ1);
            PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mAnisotropyQ2);
            PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mAnisotropyQ3);
            mAnisotropyQ1 = PxCudaHelpersExt::allocPinnedHostBuffer<PxVec4>(*cudaContextManager, maxParticles);
            mAnisotropyQ2 = PxCudaHelpersExt::allocPinnedHostBuffer<PxVec4>(*cudaContextManager, maxParticles);
            mAnisotropyQ3 = PxCudaHelpersExt::allocPinnedHostBuffer<PxVec4>(*cudaContextManager, maxParticles);
            mAnisotropyGenerator->setResultBufferHost(mAnisotropyQ1, mAnisotropyQ2, mAnisotropyQ3);
            mAnisotropyGenerator->setMaxParticles(maxParticles);
        }
    }
#endif
}

void ParticlePostprocess::releaseAnisotropyGenerator()
{
#if USE_PHYSX_GPU
    if (mAnisotropyGenerator)
    {
        PxCudaContextManager* cudaContextManager = mCudaContextManager;
        if (cudaContextManager)
        {
            if (mAnisotropyQ1 || mAnisotropyQ2 || mAnisotropyQ3)
            {
                PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mAnisotropyQ1);
                PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mAnisotropyQ2);
                PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mAnisotropyQ3);
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
    AttachedStage* attachedStage = &mAttachedStage;
    const omni::physics::parse::IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr;
    if (!src)
    {
        return;
    }

    PxCudaContextManager* cudaContextManager = mCudaContextManager;
    if (!cudaContextManager)
    {
        return;
    }

    omni::physics::parse::ParseContext ctx(const_cast<omni::physics::parse::IPhysicsSource&>(*src), iceDescriptorAllocator());
    omni::physics::parse::DescPtr<omni::physics::parse::ParticleSmoothingDesc> desc =
        omni::physics::parse::parseParticleSmoothing(ctx, mParticleSystemKey);
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
            PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mSmoothedPositions);
            mSmoothedPositions = PxCudaHelpersExt::allocPinnedHostBuffer<PxVec4>(*cudaContextManager, maxParticles);
            mSmoothedPositionGenerator->setResultBufferHost(mSmoothedPositions);
            mSmoothedPositionGenerator->setMaxParticles(maxParticles);
        }
    }
#endif
}

void ParticlePostprocess::releaseSmoothedPositionGenerator()
{
#if USE_PHYSX_GPU
    if (mSmoothedPositionGenerator)
    {
        PxCudaContextManager* cudaContextManager = mCudaContextManager;
        if (cudaContextManager)
        {
            PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mSmoothedPositions);
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
    AttachedStage* attachedStage = &mAttachedStage;
    const omni::physics::parse::IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr;
    if (!src)
    {
        return;
    }

    PxCudaContextManager* cudaContextManager = mCudaContextManager;
    if (!cudaContextManager)
    {
        return;
    }

    omni::physics::parse::ParseContext ctx(const_cast<omni::physics::parse::IPhysicsSource&>(*src), iceDescriptorAllocator());
    // Isosurface grid autocompute keys off the system's resolved fluidRestOffset.
    float fluidRestOffset = 0.0f;
    if (omni::physics::parse::DescPtr<omni::physics::parse::ParticleSystemDesc> sysDesc =
            omni::physics::parse::parseParticleSystem(ctx, mParticleSystemKey))
        fluidRestOffset = sysDesc->fluidRestOffset;
    omni::physics::parse::DescPtr<omni::physics::parse::ParticleIsosurfaceDesc> desc =
        omni::physics::parse::parseParticleIsosurface(ctx, mParticleSystemKey, fluidRestOffset);
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
            PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mIsosurfaceVertices);
            PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mIsosurfaceTriangleIndices);
            PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mIsosurfaceNormals);
            mIsosurfaceVertices = PxCudaHelpersExt::allocPinnedHostBuffer<PxVec4>(*cudaContextManager, desc->maxIsosurfaceVertices);
            mIsosurfaceTriangleIndices = PxCudaHelpersExt::allocPinnedHostBuffer<PxU32>(*cudaContextManager, desc->maxIsosurfaceTriangles * 3);
            mIsosurfaceNormals = PxCudaHelpersExt::allocPinnedHostBuffer<PxVec4>(*cudaContextManager, desc->maxIsosurfaceVertices);
            mIsosurfaceExtractor->setResultBufferHost(mIsosurfaceVertices, mIsosurfaceTriangleIndices, mIsosurfaceNormals);
            mIsosurfaceExtractor->setMaxVerticesAndTriangles(desc->maxIsosurfaceVertices, desc->maxIsosurfaceTriangles);
        }
    }
#endif
}

void ParticlePostprocess::releaseIsosurfaceExtractor()
{
#if USE_PHYSX_GPU
    if (mIsosurfaceExtractor)
    {
        if (mIsosurfaceVertices)
        {
            PxCudaContextManager* cudaContextManager = mCudaContextManager;
            if (cudaContextManager)
            {
                PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mIsosurfaceVertices);
                PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mIsosurfaceTriangleIndices);
                PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mIsosurfaceNormals);
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

    // Existence precondition only -- the object is not used past this point -- so with
    // no backing source there is nothing to check and the post-processing below still
    // runs. Skipping it here would silently disable anisotropy, smoothing and
    // isosurface extraction rather than protect anything.
    AttachedStage* attachedStage = &mAttachedStage;
    const omni::physics::parse::IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr;
    if (src && !src->exists(mParticleSystemKey))
    {
        return;
    }

    PxCudaContextManager* cudaContextManager = mCudaContextManager;
    if (!cudaContextManager)
    {
        return;
    }

    PxU32 maxParticles = (mMaxParticlesCallback > numPoints) ? mMaxParticlesCallback : numPoints;

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
        PxCudaHelpersExt::freeDeviceBuffer(*cudaContextManager, mParticlesPreview);
        mParticlesPreview = PxCudaHelpersExt::allocDeviceBuffer<PxVec4>(*cudaContextManager, maxParticles);
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

void ParticlePostprocess::addParticleSet(omni::physics::parse::ObjectKey particleSetKey)
{
    AttachedStage* attachedStage = &mAttachedStage;
    const omni::physics::parse::IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr;
    if (!src)
    {
        // The fluid classification and the proxy-purpose edit are both source
        // reads/writes, so refuse rather than register a set we could not classify.
        CARB_LOG_ERROR("addPostprocessParticleSet requires an active physics source, ignoring particle set: %s",
                       attachedStage ? attachedStage->textFor(particleSetKey) : "<no stage>");
        return;
    }

    omni::physics::parse::ParseContext ctx(const_cast<omni::physics::parse::IPhysicsSource&>(*src), iceDescriptorAllocator());
    omni::physics::parse::DescPtr<omni::physics::parse::ParticleSetDesc> desc =
        omni::physics::parse::parseParticleSet(ctx, particleSetKey);

    if (desc && desc->fluid)
    {
        mFluidParticleSetsPreview.insert(particleSetKey);
        if (hasIsosurface())
        {
            // make particle set invisible on the session layer -- we don't need to
            // store this to file, and retain visibility attribute for user (e.g. to
            // selectively allow for disabling of particle visualization)
            writeProxyPurposeToSink(mAttachedStage, particleSetKey, true);
        }
    }
}

void ParticlePostprocess::removeParticleSet(omni::physics::parse::ObjectKey particleSetKey)
{
    auto fit = mFluidParticleSetsPreview.find(particleSetKey);
    if (fit != mFluidParticleSetsPreview.end())
    {
        if (hasIsosurface())
        {
            // make particles visible again by removing proxy purpose on the session layer
            writeProxyPurposeToSink(mAttachedStage, particleSetKey, false);
        }

        if (hasAnisotropy())
        {
            restorePointInstancerScaleOrientToSink(mAttachedStage, particleSetKey);
        }

        if (hasSmoothing())
        {
            restorePointsToSink(mAttachedStage, particleSetKey);
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
                particleSet.key = internalParticleSet->mKey;
                particleSet.srcOffset = internalParticleSet->mParticleBuffer->getFlatListStartIndex();
                particleSet.srcCount = internalParticleSet->mNumParticles;
                mTmpParticleSets.push_back(particleSet);
            }
        }
    }
    else
    {
        AttachedStage* attachedStage = &mAttachedStage;
        const omni::physics::parse::IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr;
        if (src)
        {
            KnownTokens tok;
            tok.intern(*src);
            uint32_t srcOffset = 0;
            for (const omni::physics::parse::ObjectKey& setKey : mFluidParticleSetsPreview)
            {
                if (!src->exists(setKey))
                {
                    continue;
                }
                const bool isInstancer = src->isA(setKey, tok.pointInstancerType);
                std::vector<carb::Float3> dstPoints;
                getArrayValue(*attachedStage, setKey, isInstancer ? tok.positions : tok.points,
                              ReadTime::defaultTime(), dstPoints);
                uint32_t count = uint32_t(dstPoints.size());

                ParticleSet particleSet;
                particleSet.internal = nullptr;
                particleSet.key = setKey;
                particleSet.srcOffset = srcOffset;
                particleSet.srcCount = count;
                mTmpParticleSets.push_back(particleSet);

                srcOffset += count;
            }
        }
    }

    return mTmpParticleSets;
}

void ParticlePostprocess::updatePreview()
{
#if USE_PHYSX_GPU
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
            appendParticleSetPointsFromSink(mAttachedStage, points, particleSet.key);
        }
    }

    mResultNumParticles =  PxU32(points.size());
    if (mResultNumParticles > 0)
    {
        updateFromPoints(points.data(), mResultNumParticles);
    }

    //update the geometry
    {
        { ScopedNoticeBlock blockNotices(mAttachedStage, true, false);

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

        }
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
    // Publishes unconditionally; the sink no-ops the write when the session-layer
    // mesh createSessionIsosurfaceMesh would have defined isn't there (same gate
    // as there).
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
        std::vector<carb::Float3> tmpPoints(numVertices);
        const PxVec4* verts = mIsosurfaceVertices;
        for (PxU32 i = 0; i < numVertices; ++i)
        {
            tmpPoints[i] = { verts[i].x, verts[i].y, verts[i].z };
        }

        std::vector<carb::Float3> tmpNormals(numVertices);
        const PxVec4* normals = mIsosurfaceNormals;
        for (PxU32 i = 0; i < numVertices; ++i)
        {
            tmpNormals[i] = { normals[i].x, normals[i].y, normals[i].z };
        }

        const std::vector<int32_t> tmpVertexCounts(numTris, 3);

        std::vector<int32_t> tmpVertexIndices(3 * size_t(numTris));
        const PxU32* tris = mIsosurfaceTriangleIndices;
        for (size_t i = 0; i < tmpVertexIndices.size(); ++i)
        {
            tmpVertexIndices[i] = int32_t(tris[i]);
        }

        writeIsosurfaceMeshToSink(mAttachedStage, mParticleSystemKey, tmpPoints.data(), tmpPoints.size(), tmpNormals.data(),
                                  tmpNormals.size(), tmpVertexCounts.data(), tmpVertexCounts.size(),
                                  tmpVertexIndices.data(), tmpVertexIndices.size());
        return;
    }
#endif
    {
        const std::vector<int32_t> tmpVertexCounts = { 3 };
        const std::vector<int32_t> tmpVertexIndices = { 0, 0, 0 };
        const std::vector<carb::Float3> tmpVec3 = { carb::Float3{ 0.0f, 0.0f, 0.0f } };
        writeIsosurfaceMeshToSink(mAttachedStage, mParticleSystemKey, tmpVec3.data(), tmpVec3.size(), tmpVec3.data(), tmpVec3.size(),
                                  tmpVertexCounts.data(), tmpVertexCounts.size(), tmpVertexIndices.data(),
                                  tmpVertexIndices.size());
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
        writePointInstancerScaleOrientToSink(mAttachedStage, particleSet.key,
            srcAnisotropyQ1 + particleSet.srcOffset,
            srcAnisotropyQ2 + particleSet.srcOffset,
            srcAnisotropyQ3 + particleSet.srcOffset,
            particleSet.srcCount, particleContactDistanceInv);
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

    // Every write below goes through a sink helper that already no-ops without an
    // attached stage, so with no stage there is nothing to publish -- return before
    // the getWorldTransform() read rather than dereferencing a null stage.
    AttachedStage* as = &mAttachedStage;
    if (!as)
        return;

    for (size_t s = 0; s < particleSets.size(); ++s)
    {
        ParticleSet& particleSet = particleSets[s];

        if (!mParent)
        {
            //backup simulation points
            backupSimulationPointsToSink(mAttachedStage, particleSet.key);
        }

        const omni::physics::parse::ObjectKey particleSetKey = particleSet.internal
            ? particleSet.internal->mKey
            : particleSet.key;
        const PxMat44d localToWorld = getWorldTransform(*as, particleSetKey, ReadTime::defaultTime());
        const PxMat44d worldToLocal = omni::physx::affineInverse(localToWorld);

        std::vector<carb::Float3> dstPoints(particleSet.srcCount);
        const PxVec4* srcSlice = srcPositions + particleSet.srcOffset;
        for (uint32_t i = 0; i < particleSet.srcCount; ++i)
        {
            const PxVec3d dst = worldToLocal.transform(PxVec3d(srcSlice[i].x, srcSlice[i].y, srcSlice[i].z));
            dstPoints[i] = { float(dst.x), float(dst.y), float(dst.z) };
        }

        writePointsToSink(mAttachedStage, particleSet.key, dstPoints.data(), dstPoints.size());
    }

#endif
}

void ParticlePostprocess::restorePoints()
{
    std::vector<ParticleSet>& particleSets = getTmpParticleSets();
    for (size_t s = 0; s < particleSets.size(); ++s)
    {
        restorePointsToSink(mAttachedStage, particleSets[s].key);
    }
}

void ParticlePostprocess::restoreScaleOrient()
{
    std::vector<ParticleSet>& particleSets = getTmpParticleSets();
    for (size_t s = 0; s < particleSets.size(); ++s)
    {
        restorePointInstancerScaleOrientToSink(mAttachedStage, particleSets[s].key);
    }
}

void ParticlePostprocess::createSessionIsosurfaceMesh()
{
    setIsosurfaceMeshEnabledInSink(mAttachedStage, mParticleSystemKey, true);

    // make particle sets invisible by setting purpose = proxy on the session
    // layer -- we don't need to store this to file, and retain visibility
    // attribute for user (e.g. to selectively allow for disabling of particle
    // visualization)
    std::vector<ParticleSet>& particleSets = getTmpParticleSets();
    for (size_t s = 0; s < particleSets.size(); ++s)
    {
        writeProxyPurposeToSink(mAttachedStage, particleSets[s].key, true);
    }
}

void ParticlePostprocess::releaseSessionIsosurfaceMesh()
{
    setIsosurfaceMeshEnabledInSink(mAttachedStage, mParticleSystemKey, false);

    // make particles visible again by removing proxy purpose from the session layer
    std::vector<ParticleSet>& particleSets = getTmpParticleSets();
    for (size_t s = 0; s < particleSets.size(); ++s)
    {
        writeProxyPurposeToSink(mAttachedStage, particleSets[s].key, false);
    }
}

namespace
{
using omni::physx::particles::PostprocessRef;
using omni::physx::particles::PostprocessRefMap;

PostprocessRef* getPostprocessRef(const AttachedStage& owner, omni::physics::parse::ObjectKey particleSystemKey)
{
    PostprocessRefMap& refs = owner.particlePostprocess;
    auto it = refs.find(particleSystemKey);
    return (it != refs.end()) ? &it->second : nullptr;
}
} // namespace

namespace omni
{
namespace physx
{
namespace particles
{
#if USE_PHYSX_GPU
    void setIsosurfaceGridFilteringPasses(PxIsosurfaceParams& isosurfaceParams, const std::vector<omni::physics::parse::ParticleIsosurfaceDesc::GridFilteringPass::Enum>& passes)
    {
        using GridFilteringPass = omni::physics::parse::ParticleIsosurfaceDesc::GridFilteringPass;
        isosurfaceParams.clearFilteringPasses();
        for (size_t i = 0; i < passes.size(); ++i)
        {
            PxIsosurfaceGridFilteringType::Enum dstType = PxIsosurfaceGridFilteringType::eNONE;
            if (passes[i] == GridFilteringPass::eSmooth)
            {
                dstType = PxIsosurfaceGridFilteringType::eSMOOTH;
            }
            else if (passes[i] == GridFilteringPass::eGrow)
            {
                dstType = PxIsosurfaceGridFilteringType::eGROW;
            }
            else if (passes[i] == GridFilteringPass::eReduce)
            {
                dstType = PxIsosurfaceGridFilteringType::eSHRINK;
            }
            isosurfaceParams.addGridFilteringPass(dstType);
        }
    }
#endif

    // createPostprocess binds the supplied attachment: the GPU generator setup is
    // source-backed and runs regardless of write-sink availability. Only the final
    // USD-authoring writes are sink-gated, each no-op-ing internally with no live sink.

    void createPostprocess(AttachedStage& owner, omni::physics::parse::ObjectKey particleSystemKey, uint32_t particlePostFlags, omni::physx::internal::InternalPbdParticleSystem* parent)
    {
        PostprocessRefMap& registry = owner.particlePostprocess;
        auto entry = registry.find(particleSystemKey);
        if (entry == registry.end())
        {
            std::unique_ptr<ParticlePostprocess> pending(ICE_NEW(ParticlePostprocess)(owner, particleSystemKey));
            pending->setPostprocessFlags(particlePostFlags);
            if (parent)
                pending->setParent(parent);
            entry = registry.emplace(particleSystemKey, PostprocessRef{ 0, pending.get() }).first;
            pending.release();
        }
        else
        {
            ParticlePostprocess* oldPostprocess = entry->second.postprocess;
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
        entry->second.refCount++;
    }

    void releasePostprocess(AttachedStage& owner, omni::physics::parse::ObjectKey particleSystemKey, omni::physx::internal::InternalPbdParticleSystem* parent)
    {
        PostprocessRef* postprocessRef = getPostprocessRef(owner, particleSystemKey);
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
                owner.particlePostprocess.erase(particleSystemKey);
            }
        }
    }

    void getAnisotropy(AttachedStage& owner, ::physx::PxVec4*& anisotropyQ1, ::physx::PxVec4*& anisotropyQ2, ::physx::PxVec4*& anisotropyQ3, omni::physics::parse::ObjectKey particleSystemKey)
    {
        PostprocessRef* postprocessRef = getPostprocessRef(owner, particleSystemKey);
        if (postprocessRef && postprocessRef->postprocess)
        {
            postprocessRef->postprocess->getAnisotropy(anisotropyQ1, anisotropyQ2, anisotropyQ3);
            return;
        }
        anisotropyQ1 = anisotropyQ2 = anisotropyQ3 = nullptr;
    }

    ::physx::PxVec4* getSmoothedPositions(AttachedStage& owner, omni::physics::parse::ObjectKey particleSystemKey)
    {
        PostprocessRef* postprocessRef = getPostprocessRef(owner, particleSystemKey);
        if (postprocessRef && postprocessRef->postprocess)
        {
            return postprocessRef->postprocess->getSmoothedPositions();
        }
        return nullptr;
    }

    void updateIsosurfaceMesh(AttachedStage& owner, omni::physics::parse::ObjectKey particleSystemKey)
    {
        PostprocessRef* postprocessRef = getPostprocessRef(owner, particleSystemKey);
        if (postprocessRef && postprocessRef->postprocess)
        {
            postprocessRef->postprocess->updateMesh();
        }
    }

    void notifyParticleSystemResize(const AttachedStage& owner, omni::physics::parse::ObjectKey particleSystemKey)
    {
        PostprocessRef* postprocessRef = getPostprocessRef(owner, particleSystemKey);
        if (postprocessRef && postprocessRef->postprocess)
        {
            postprocessRef->postprocess->updateGeneratorsCallback();
        }
    }

    void notifyPhysXRelease(AttachedStage& owner)
    {
        for (PostprocessRefMap::reference item : owner.particlePostprocess)
            if (item.second.postprocess)
                item.second.postprocess->setPostprocessFlags(0);
    }

    void releaseAttachmentPostprocess(AttachedStage& owner)
    {
        for (PostprocessRefMap::reference item : owner.particlePostprocess)
            SAFE_DELETE_SINGLE(item.second.postprocess);
        owner.particlePostprocess.clear();
    }

    // Explicit process shutdown operation. Local teardown uses the overload above.
    void notifyAllPhysXRelease()
    {
        for (const usdparser::AttachedStageMap::value_type& entry : usdparser::UsdLoad::getUsdLoad()->getAttachedStages())
            notifyPhysXRelease(*entry.second);
    }

    ///////////////////////////////
    /// IPhysXParticles interface
    ///////////////////////////////

    void createPostprocess(AttachedStage& owner, omni::physics::parse::ObjectKey particleSystemKey, uint32_t particlePostFlags)
    {
        createPostprocess(owner, particleSystemKey, particlePostFlags, nullptr);
    }

    void releasePostprocess(AttachedStage& owner, omni::physics::parse::ObjectKey particleSystemKey)
    {
        releasePostprocess(owner, particleSystemKey, nullptr);
    }

    uint32_t getPostprocessStages(AttachedStage& owner, omni::physics::parse::ObjectKey particleSystemKey)
    {
        PostprocessRef* postprocessRef = getPostprocessRef(owner, particleSystemKey);
        if (postprocessRef && postprocessRef->postprocess)
        {
            return postprocessRef->postprocess->getPostprocessFlags();
        }
        return ParticlePostFlag::eNone;
    }

    void setPostprocessStages(AttachedStage& owner, omni::physics::parse::ObjectKey particleSystemKey, uint32_t particlePostFlags)
    {
        PostprocessRef* postprocessRef = getPostprocessRef(owner, particleSystemKey);
        if (postprocessRef && postprocessRef->postprocess)
        {
            postprocessRef->postprocess->setPostprocessFlags(particlePostFlags);
        }
    }

    void addPostprocessParticleSet(AttachedStage& owner, omni::physics::parse::ObjectKey particleSystemKey, omni::physics::parse::ObjectKey particleSetKey)
    {
        PostprocessRef* postprocessRef = getPostprocessRef(owner, particleSystemKey);
        if (postprocessRef && postprocessRef->postprocess)
        {
            postprocessRef->postprocess->addParticleSet(particleSetKey);
        }
    }

    void removePostprocessParticleSet(AttachedStage& owner, omni::physics::parse::ObjectKey particleSystemKey, omni::physics::parse::ObjectKey particleSetKey)
    {
        PostprocessRef* postprocessRef = getPostprocessRef(owner, particleSystemKey);
        if (postprocessRef && postprocessRef->postprocess)
        {
            postprocessRef->postprocess->removeParticleSet(particleSetKey);
        }
    }

    void updatePostprocess(AttachedStage& owner, omni::physics::parse::ObjectKey particleSystemKey)
    {
        PostprocessRef* postprocessRef = getPostprocessRef(owner, particleSystemKey);
        if (postprocessRef && postprocessRef->postprocess)
        {
            postprocessRef->postprocess->updatePreview();
        }
    }

} // namespace particles
} // namespace physx
} // namespace omni
