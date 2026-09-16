// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

// ParticlePostprocess drives the GPU anisotropy/smoothing/isosurface post-process pipeline
// and publishes its results through the source-agnostic write sink (IPhysicsDataWrite):
// session-layer mesh/point/orientation output, Hydra-consumed -- genuine Kit-viewport-
// rendering-only functionality (see UsdInterface.cpp's finishSetup() comment), with no
// ovstage-native consumer anywhere in the runtime today. Everything that touches the PhysX
// SDK generators or OmniPhysX internals (GPU compute, CUDA buffers, InternalPbdParticleSystem)
// stays here, pxr-free; only the final "author this into USD" step crosses the sink boundary
// (writeArray/writeProxyPurpose/writeIsosurfaceMesh/...), exactly like the rest of the
// IPhysicsDataWrite-backed write path. The generator setup itself is source-backed (needs
// only an active attached stage, not a live write sink) and runs unconditionally. With no
// live sink (getDataWrite() == nullptr) every write call below no-ops internally, so
// nothing is published -- without disabling the generator work an ovstage-native attach
// may still want.
#include <PxPhysicsAPI.h>
#include <PhysXDefines.h>

#include <internal/Internal.h>

#include <private/omni/physx/ParticlePostFlag.h>

#include <omni/physics/parse/Descriptors.h>
#include <omni/physics/parse/Handles.h>

#include <unordered_set>
#include <vector>

namespace physx
{
class PxGeometry;

class PxParticleNeighborhoodProvider;
class PxAnisotropyGenerator;
class PxSmoothedPositionGenerator;
class PxIsosurfaceExtractor;
struct PxIsosurfaceParams;
} // namespace physx

namespace omni
{
namespace physx
{

namespace internal
{
class InternalPbdParticleSystem;
class InternalParticleSet;
} // namespace internal

namespace particles
{

class PostProcessCallback : public ::physx::PxParticleSystemCallback, public Allocateable
{
public:
    PostProcessCallback(internal::InternalPbdParticleSystem* particleSystem)
        : mParticleSystem(particleSystem),
          mAnisotropyGenerator(nullptr),
          mSmoothedPositionGenerator(nullptr),
          mIsosurfaceExtractor(nullptr),
          mResultNumParticles(nullptr)
    {
    }

    void initialize(::physx::PxAnisotropyGenerator* anisotropyGenerator,
                    ::physx::PxSmoothedPositionGenerator* smoothedPositionGenerator,
                    ::physx::PxIsosurfaceExtractor* isosurfaceExtractor,
                    uint32_t* resultNumParticles)
    {
        mAnisotropyGenerator = anisotropyGenerator;
        mSmoothedPositionGenerator = smoothedPositionGenerator;
        mIsosurfaceExtractor = isosurfaceExtractor;
        mResultNumParticles = resultNumParticles;
    }

    virtual void onPostSolve(const ::physx::PxGpuMirroredPointer<::physx::PxGpuParticleSystem>& gpuParticleSystem,
                             CUstream stream);
    virtual void onBegin(const ::physx::PxGpuMirroredPointer<::physx::PxGpuParticleSystem>& gpuParticleSystem,
                         CUstream stream);
    virtual void onAdvance(const ::physx::PxGpuMirroredPointer<::physx::PxGpuParticleSystem>& /*gpuParticleSystem*/,
                           CUstream /*stream*/)
    {
    }
    virtual ~PostProcessCallback()
    {
    }

    internal::InternalPbdParticleSystem* mParticleSystem;
    ::physx::PxAnisotropyGenerator* mAnisotropyGenerator;
    ::physx::PxSmoothedPositionGenerator* mSmoothedPositionGenerator;
    ::physx::PxIsosurfaceExtractor* mIsosurfaceExtractor;
    uint32_t* mResultNumParticles;
};

class ParticlePostprocess : public Allocateable
{
public:
    ParticlePostprocess(omni::physics::parse::ObjectKey particleSystemKey);
    ~ParticlePostprocess();

    uint32_t getPostprocessFlags() const
    {
        return mFlags;
    }
    void setPostprocessFlags(uint32_t newFlags);
    void setParent(internal::InternalPbdParticleSystem* parent);
    void addParticleSet(omni::physics::parse::ObjectKey particleSetKey);
    void removeParticleSet(omni::physics::parse::ObjectKey particleSetKey);
    void updateGeneratorsCallback();

    void updatePreview();

    // retrieve/process results
    void syncPostSolveWork();
    void updateMesh();
    void getAnisotropy(::physx::PxVec4*& anisotropyQ1, ::physx::PxVec4*& anisotropyQ2, ::physx::PxVec4*& anisotropyQ3);
    ::physx::PxVec4* getSmoothedPositions();

private:
    struct ParticleSet
    {
        internal::InternalParticleSet* internal = nullptr;
        omni::physics::parse::ObjectKey key;
        uint32_t srcOffset;
        uint32_t srcCount;
    };

    bool parseParticleContactOffset(float& particleContactOffset);

    void createNeighborhoodProvider(float particleRestOffset, const ::physx::PxU32 maxParticles);

    void updateAnisotropyGenerator(const ::physx::PxU32 maxParticles);
    void releaseAnisotropyGenerator();

    void updateSmoothedPositionGenerator(const ::physx::PxU32 maxParticles);
    void releaseSmoothedPositionGenerator();

    void updateIsosurfaceExtractor(const ::physx::PxU32 maxParticles);
    void releaseIsosurfaceExtractor();

    void updateFromPoints(const ::physx::PxVec4* points, const ::physx::PxU32 numPoints);
    void updateScaleOrient();
    void updatePoints();
    void restorePoints();
    void restoreScaleOrient();
    void createSessionIsosurfaceMesh();
    void releaseSessionIsosurfaceMesh();

    bool hasAnisotropy() const
    {
        return mFlags & omni::physx::ParticlePostFlag::eAnisotropy;
    }
    bool hasSmoothing() const
    {
        return mFlags & omni::physx::ParticlePostFlag::eSmoothing;
    }
    bool hasIsosurface() const
    {
        return mFlags & omni::physx::ParticlePostFlag::eIsosurface;
    }

    std::vector<ParticleSet>& getTmpParticleSets();

    omni::physics::parse::ObjectKey mParticleSystemKey;
    std::unordered_set<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash> mFluidParticleSetsPreview;
    uint32_t mMaxParticlesCallback = 0;
    uint32_t mFlags = ParticlePostFlag::eNone;
    internal::InternalPbdParticleSystem* mParent = nullptr;
    std::vector<ParticleSet> mTmpParticleSets;
    uint32_t mResultNumParticles = 0;

#if USE_PHYSX_GPU
    ::physx::PxParticleNeighborhoodProvider* mNeighborhoodProvider = nullptr;
    ::physx::PxAnisotropyGenerator* mAnisotropyGenerator = nullptr;
    ::physx::PxSmoothedPositionGenerator* mSmoothedPositionGenerator = nullptr;
    ::physx::PxIsosurfaceExtractor* mIsosurfaceExtractor = nullptr;
    int32_t mMaxNumIsosurfaceSubgrids = 0;
    float mIsosurfaceGridSpacing = 0.0f;

    ::physx::PxVec4* mParticlesPreview;
    uint32_t mMaxParticlesPreview = 0;

    ::physx::PxVec4* mAnisotropyQ1;
    ::physx::PxVec4* mAnisotropyQ2;
    ::physx::PxVec4* mAnisotropyQ3;

    ::physx::PxVec4* mSmoothedPositions;

    ::physx::PxVec4* mIsosurfaceVertices;
    ::physx::PxU32* mIsosurfaceTriangleIndices;
    ::physx::PxVec4* mIsosurfaceNormals;
#endif
};

#if USE_PHYSX_GPU
void setIsosurfaceGridFilteringPasses(
    ::physx::PxIsosurfaceParams& isosurfaceParams,
    const std::vector<omni::physics::parse::ParticleIsosurfaceDesc::GridFilteringPass::Enum>& passes);
#endif

void createPostprocess(omni::physics::parse::ObjectKey particleSystemKey,
                       uint32_t particlePostFlags,
                       internal::InternalPbdParticleSystem* parent);
void releasePostprocess(omni::physics::parse::ObjectKey particleSystemKey,
                        internal::InternalPbdParticleSystem* parent);

void updateIsosurfaceMesh(omni::physics::parse::ObjectKey particleSystemKey);
void getAnisotropy(::physx::PxVec4*& anisotropyQ1,
                   ::physx::PxVec4*& anisotropyQ2,
                   ::physx::PxVec4*& anisotropyQ3,
                   omni::physics::parse::ObjectKey particleSystemKey);
::physx::PxVec4* getSmoothedPositions(omni::physics::parse::ObjectKey particleSystemKey);

void notifyParticleSystemResize(omni::physics::parse::ObjectKey particleSystemKey);
void notifyPhysXRelease();

// IPhysXParticles API -- ObjectKey-typed and USD-postprocess-registry-backed like the
// rest of this file (see the top-of-file comment); getPostprocessStages/
// setPostprocessStages are the two overloads with live callers today, the rest are
// unreferenced leftovers of the deleted IPhysxParticlesPrivate ABI mirror (see
// include/private/omni/physx/IPhysxParticlesPrivate.h's own note) -- kept ObjectKey-typed
// uniformly rather than singled out.
void createPostprocess(omni::physics::parse::ObjectKey particleSystemKey, uint32_t particlePostFlags);
void releasePostprocess(omni::physics::parse::ObjectKey particleSystemKey);
void setPostprocessStages(omni::physics::parse::ObjectKey particleSystemKey, uint32_t particlePostFlags);
uint32_t getPostprocessStages(omni::physics::parse::ObjectKey particleSystemKey);
void addPostprocessParticleSet(omni::physics::parse::ObjectKey particleSystemKey,
                               omni::physics::parse::ObjectKey particleSetKey);
void removePostprocessParticleSet(omni::physics::parse::ObjectKey particleSystemKey,
                                  omni::physics::parse::ObjectKey particleSetKey);
void updatePostprocess(omni::physics::parse::ObjectKey particleSystemKey);


} // namespace particles
} // namespace physx
} // namespace omni
