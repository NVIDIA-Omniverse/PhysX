// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#ifdef _MSC_VER
#    pragma warning(push)
#    define NOMINMAX // Make sure nobody #defines min or max
#endif

#ifdef __linux__
#    define __forceinline __attribute__((always_inline))
#endif

#include "UsdPCH.h"

#include <PxPhysicsAPI.h>
#include <PhysXDefines.h>

#include <internal/InternalXformOpResetStorage.h>
#include <private/omni/physx/IPhysxParticlesPrivate.h>
#include <common/foundation/Allocator.h>

namespace physx
{
namespace ExtGpu
{
class PxParticleAttachmentBuffer;
class PxParticleClothBufferHelper;
class PxParticleVolumeBufferHelper;
} // namespace ExtGpu
} // namespace physx

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
        eVOLUME = 1 << 2,
        eDIFFUSE_PARTICLES = 1 << 3,
        eSMOOTHED_POSITIONS = 1 << 4,
        eANISOTROPY = 1 << 5,
    };
};

class InternalPbdParticleSystem;

class InternalPointCloud : public Allocateable
{
public:
    pxr::UsdPrim mPrim;
    pxr::UsdGeomPoints mGeo;
};

class InternalDiffuseParticles : public InternalPointCloud
{
public:
    InternalDiffuseParticles()
    {
    }
    ~InternalDiffuseParticles();
};

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

    pxr::UsdPrim mPrim;
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

    ::physx::PxParticleVolume* mVolumes;

    // diffuse particles
    ::physx::PxVec4* mDiffuseParticlePositions;
    ::physx::PxU32 mNumDiffuseParticles;
    ::physx::PxDiffuseParticleParams mDiffuseParticleParams;
    float mMaxDiffuseParticleMultiplier;
    bool mDiffuseParticlesEnabled;
    InternalDiffuseParticles* mSharedDiffuseParticles = nullptr;

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
    uint32_t mNumVolumes;

    pxr::GfMatrix4d mWorldToLocal;

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

    // Fabric
    bool mFabric;
    void* mFabricPositions;
    void* mFabricVelocities;
};

class InternalParticleClothDeprecated : public InternalParticle
{
public:
    InternalParticleClothDeprecated(pxr::UsdPrim& prim, InternalPbdParticleSystem& particleSystem);
    ~InternalParticleClothDeprecated();

    // pinned host buffers - may be outdated
    ::physx::PxVec4* mPositions;
    ::physx::PxVec4* mVelocities;
    ::physx::PxU32* mPhases;

    ::physx::PxParticleClothBuffer* mClothBuffer;
    uint32_t mUploadDirtyFlags;
    uint32_t mDownloadDirtyFlags;

    uint32_t mNumVolumes;

#if USE_PHYSX_GPU
    ::physx::ExtGpu::PxParticleClothBufferHelper* mParticleClothBuffer;
    ::physx::ExtGpu::PxParticleVolumeBufferHelper* mVolumeBuffer;
    ::physx::ExtGpu::PxParticleAttachmentBuffer* mAttachments;
#endif

    std::vector<carb::Float3> mPositionSaveRestoreBuf;
    std::vector<carb::Float3> mVelocitySaveRestoreBuf;
    pxr::VtVec3fArray mExtentSaveRestoreBuf;

    bool mIsWelded;
    std::vector<uint32_t> mVerticesRemapToWeld;
    std::vector<uint32_t> mVerticesRemapToOrig;

    pxr::UsdGeomMesh mGeo;
    pxr::GfMatrix4d mLocalToWorld;
    pxr::GfMatrix4d mInitialLocalToWorld;

    XformOpResetStorage mXformOpStorage;
    bool mSkipUpdateTransform;

    void addRigidAttachment(::physx::PxRigidActor* actor, ::physx::PxU32 particleId, ::physx::PxVec3 actorSpacePose);
    void removeRigidAttachment(::physx::PxRigidActor* actor, ::physx::PxU32 particleId);
    void addRigidFilter(::physx::PxRigidActor* actor, const ::physx::PxU32 particleId);
    void removeRigidFilter(::physx::PxRigidActor* actor, const ::physx::PxU32 particleId);

    void enableCloth(bool enabled);
    void uploadParticles(CUstream stream);
    void fetchParticles(CUstream stream, bool updateToUsd, bool updateVelocities, bool debugVizOn);
    void releasePinnedBuffers();
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
          mPath(pxr::SdfPath()),
          mEnabled(false),
          mParticleDataAvailable(false),
          mAsyncSim(false),
          mHasPost(false)
#if USE_PHSYX_GPU
          ,
          mCallback(nullptr)
#endif
          ,
          mDiffuseParticleInstance(nullptr),
          mDiffuseParticleInstanceRefCount(0)
    {
    }

    ~InternalPbdParticleSystem();

    ::physx::PxPBDParticleSystem* mPS;
    PhysXScene* mPhysXScene;

    usdparser::ObjectId mMaterialId;
    pxr::SdfPath mPath;

    bool mEnabled;
    bool mParticleDataAvailable;
    bool mAsyncSim;

    std::vector<InternalParticleSet*> mParticleSets;
    std::vector<InternalParticleClothDeprecated*> mCloths;

    bool mHasPost;
#if USE_PHYSX_GPU
    omni::physx::particles::PostProcessCallback* mCallback;
#endif

    InternalDiffuseParticles* mDiffuseParticleInstance;
    uint32_t mDiffuseParticleInstanceRefCount;

    std::unordered_map<int, unsigned int> mPhaseMap;

    void uploadParticles(CUstream stream);
    void fetchParticles(CUstream stream);

    void enableParticleSystem(bool enable);
    void removeParticleObjectsFromDB();
    void removeParticleSet(InternalParticleSet* internalParticleSet);
    void removeParticleCloth(InternalParticleClothDeprecated* internalParticleCloth);

    void setPost(const uint32_t postFlags);
    void enablePost(omni::physx::ParticlePostFlag::Enum flag, const bool enable);
};

} // namespace internal
} // namespace physx
} // namespace omni

#ifdef _MSC_VER
#    pragma warning(pop)
#endif
