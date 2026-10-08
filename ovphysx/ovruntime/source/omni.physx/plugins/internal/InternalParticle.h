// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#ifdef _MSC_VER
#    pragma warning(push)
#    ifndef NOMINMAX
#        define NOMINMAX // Make sure nobody #defines min or max
#    endif
#endif

#ifdef __linux__
#    define __forceinline __attribute__((always_inline))
#endif

#include <PxPhysicsAPI.h>
#include <PhysXDefines.h>

#include <private/omni/physx/ParticlePostFlag.h>
// PhysxUsd.h is itself pxr-free (re-exports parse-lib ObjectId/etc. into usdparser::) -- pulled
// in unconditionally for usdparser::ObjectId/kInvalidObjectId below.
#include <private/omni/physx/PhysxUsd.h>
#include <common/foundation/Allocator.h>

#include <omni/physics/parse/Handles.h>

namespace omni
{
namespace physx
{

class PhysXSetup;
class PhysXScene;

namespace particles
{
class PostProcessCallback;
}

namespace internal
{

struct ParticleBufferFlags
{
    enum Enum
    {
        ePOSITIONS = 1 << 0,
        eVELOCITIES = 1 << 1,
        ePHASES = 1 << 2,
        eALL = (ePOSITIONS | eVELOCITIES | ePHASES),
    };
};

struct ParticleDirtyFlags
{
    enum Enum
    {
        ePOSITION_INVMASS = 1 << 0,
        eVELOCITY = 1 << 1,
        eDIFFUSE_PARTICLES = 1 << 2,
        eSMOOTHED_POSITIONS = 1 << 3,
        eANISOTROPY = 1 << 4,
    };
};

class InternalPbdParticleSystem;

class InternalParticle : public Allocateable
{
public:
    InternalParticle(InternalPbdParticleSystem& particleSystem)
        : mParentParticleSystem(&particleSystem),
          mNumParticles(0),
          mEnabled(false),
          mMaterialId(usdparser::kInvalidObjectId),
          mFlags(0)
    {
    }

    // Source-agnostic handle for this particle (set)'s prim; resolved to a
    // prim/path on demand via the AttachedStage (no stored UsdPrim).
    omni::physics::parse::ObjectKey mKey;
    InternalPbdParticleSystem* mParentParticleSystem;
    uint32_t mNumParticles;
    bool mEnabled;

    usdparser::ObjectId mMaterialId;
    uint32_t mFlags;
    PhysXScene* mPhysXScene;
};

class InternalParticleSet : public InternalParticle
{
public:
    InternalParticleSet(InternalPbdParticleSystem& particleSystem);
    ~InternalParticleSet();

    // pinned host buffers
    ::physx::PxVec4* mPositions;
    ::physx::PxVec4* mVelocities;
    ::physx::PxU32* mPhases;

    // diffuse particles
    ::physx::PxVec4* mDiffuseParticlePositions;
    ::physx::PxU32 mNumDiffuseParticles;
    ::physx::PxDiffuseParticleParams mDiffuseParticleParams;
    float mMaxDiffuseParticleMultiplier;
    bool mDiffuseParticlesEnabled;
    // Whether this particle set has counted itself in the parent particle
    // system's shared diffuse-particle-rendering refcount (mDiffuseParticleInstanceRefCount).
    bool mHasSharedDiffuseParticles = false;

    // dirty flags for DtoH transfers
    uint32_t mDownloadDirtyFlags;

    // particle User buffer
    ::physx::PxParticleAndDiffuseBuffer* mParticleBuffer;
    uint32_t mUploadDirtyFlags;
    ::physx::PxParticleAndDiffuseBuffer* mReplacementBuffer;

    // restore
    std::vector<carb::Float3> mPositionSaveRestoreBuf;
    std::vector<carb::Float3> mVelocitySaveRestoreBuf;

    uint32_t mMaxParticles;

    ::physx::PxMat44d mWorldToLocal{ ::physx::PxIdentity };

    bool mFluid;
    float mParticleInvMass;
    uint32_t mPhase;

    static uint32_t getMaxDiffuseParticles(uint32_t maxParticles, bool diffuseEnabled, float maxDiffuseParticleMultiplier);

    static ::physx::PxParticleAndDiffuseBuffer* createUserBuffer(uint32_t maxParticles,
                                                                 uint32_t numParticles,
                                                                 bool diffuseEnabled,
                                                                 float maxDiffuseParticleMultiplier,
                                                                 ::physx::PxDiffuseParticleParams& diffuseParams,
                                                                 PhysXSetup& physXSetup);

    void uploadParticles(CUstream stream);
    void fetchParticles(CUstream stream,
                        bool hasIsosurface,
                        bool hasSmoothing,
                        bool hasAnisotropy,
                        bool updateToUsd,
                        bool updateParticlesToUsd,
                        bool updateVelocities,
                        bool debugVizOn);

    void resize(uint32_t newNumParticles);

    void enableParticleSet(bool enabled);
    void releasePinnedBuffers();

    void createSharedDiffuseParticles();
    void releaseSharedDiffuseParticles();

    void changeDiffuseParticles(bool remove);
    void setDiffuseParticleParams();
    void enableDiffuseParticles(bool enabled);
};

class InternalPBDParticleMaterial : public Allocateable
{
public:
    InternalPBDParticleMaterial(float density) : mDensity(density)
    {
    }

    ~InternalPBDParticleMaterial() = default;

    void addParticleId(usdparser::ObjectId id)
    {
        mParticleIds.push_back(id);
    }

    void removeParticleId(usdparser::ObjectId id)
    {
        for (size_t i = mParticleIds.size(); i--;)
        {
            if (mParticleIds[i] == id)
            {
                mParticleIds[i] = mParticleIds.back();
                mParticleIds.pop_back();
                break;
            }
        }
    }

    float mDensity;
    std::vector<usdparser::ObjectId> mParticleIds;
};

class InternalPbdParticleSystem : public Allocateable
{
public:
    InternalPbdParticleSystem(PhysXScene* scene)
        : mPS(nullptr),
          mPhysXScene(scene),
          mMaterialId(usdparser::kInvalidObjectId),
          mEnabled(false),
          mParticleDataAvailable(false),
          mAsyncSim(false),
          mHasPost(false)
#if USE_PHSYX_GPU
          ,
          mCallback(nullptr)
#endif
          ,
          mDiffuseParticleRenderingEnabled(false),
          mDiffuseParticleInstanceRefCount(0)
    {
    }

    ~InternalPbdParticleSystem();

    ::physx::PxPBDParticleSystem* mPS;
    PhysXScene* mPhysXScene;

    usdparser::ObjectId mMaterialId;
    // Source-agnostic handle for the particle-system prim; the particles:: post-process
    // registry (particles/PhysXParticlePost.h) and the diffuse-particle-rendering sink
    // (IPhysicsDataWrite::setDiffuseParticleRenderingEnabled/writeDiffuseParticlePoints)
    // are both ObjectKey-keyed and take mKey directly.
    omni::physics::parse::ObjectKey mKey;

    bool mEnabled;
    bool mParticleDataAvailable;
    bool mAsyncSim;

    std::vector<InternalParticleSet*> mParticleSets;

    bool mHasPost;
#if USE_PHYSX_GPU
    omni::physx::particles::PostProcessCallback* mCallback;
#endif

    // Whether the shared "DiffuseParticles" session-layer render prim is currently
    // defined (see IPhysicsDataWrite::setDiffuseParticleRenderingEnabled).
    bool mDiffuseParticleRenderingEnabled;
    uint32_t mDiffuseParticleInstanceRefCount;

    std::unordered_map<int, unsigned int> mPhaseMap;

    void uploadParticles(CUstream stream);
    void fetchParticles(CUstream stream);

    void enableParticleSystem(bool enable);
    void removeParticleObjectsFromDB();
    void removeParticleSet(InternalParticleSet* internalParticleSet);

    void setPost(const uint32_t postFlags);
    void enablePost(omni::physx::ParticlePostFlag::Enum flag, const bool enable);
};

} // namespace internal
} // namespace physx
} // namespace omni

#ifdef _MSC_VER
#    pragma warning(pop)
#endif
