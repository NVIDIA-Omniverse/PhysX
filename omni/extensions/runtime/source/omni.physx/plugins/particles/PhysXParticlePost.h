// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include <PxPhysicsAPI.h>
#include <PhysXDefines.h>

#include <internal/Internal.h>

#include <private/omni/physx/IPhysxParticlesPrivate.h>

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
    ParticlePostprocess(const pxr::SdfPath particleSystemPath);
    ~ParticlePostprocess();

    uint32_t getPostprocessFlags() const
    {
        return mFlags;
    }
    void setPostprocessFlags(uint32_t newFlags);
    void setParent(internal::InternalPbdParticleSystem* parent);
    void addParticleSet(const pxr::SdfPath& particleSetPath);
    void removeParticleSet(const pxr::SdfPath& particleSetPath);
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
        pxr::UsdPrim usdPrim;
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

    pxr::SdfPath mParticleSystemPath;
    pxr::SdfPathSet mFluidParticleSetsPreview;
    uint32_t mMaxParticlesCallback = 0;
    uint32_t mFlags = ParticlePostFlag::eNone;
    pxr::SdfPath mIsosurfaceMeshPath;
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
    const std::vector<omni::physx::usdparser::ParticleIsosurfaceDesc::GridFilteringPass::Enum>& passes);
#endif

void createPostprocess(const pxr::SdfPath& particleSystemPath,
                       uint32_t particlePostFlags,
                       internal::InternalPbdParticleSystem* parent);
void releasePostprocess(const pxr::SdfPath& particleSystemPath, internal::InternalPbdParticleSystem* parent);

void updateIsosurfaceMesh(const pxr::SdfPath& particleSystemPath);
void getAnisotropy(::physx::PxVec4*& anisotropyQ1,
                   ::physx::PxVec4*& anisotropyQ2,
                   ::physx::PxVec4*& anisotropyQ3,
                   const pxr::SdfPath& particleSystemPath);
::physx::PxVec4* getSmoothedPositions(const pxr::SdfPath& particleSystemPath);

void notifyParticleSystemResize(const pxr::SdfPath& particleSystemPath);
void notifyPhysXRelease();

// IPhysXParticles API
void createPostprocess(const pxr::SdfPath& particleSystemPath, uint32_t particlePostFlags);
void releasePostprocess(const pxr::SdfPath& particleSystemPath);
void setPostprocessStages(const pxr::SdfPath& particleSystemPath, uint32_t particlePostFlags);
uint32_t getPostprocessStages(const pxr::SdfPath& particleSystemPath);
void addPostprocessParticleSet(const pxr::SdfPath& particleSystemPath, const pxr::SdfPath& particleSetPath);
void removePostprocessParticleSet(const pxr::SdfPath& particleSystemPath, const pxr::SdfPath& particleSetPath);
void updatePostprocess(const pxr::SdfPath& particleSystemPath);


} // namespace particles
} // namespace physx
} // namespace omni
