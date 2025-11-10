// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include <private/omni/physx/IPhysxParticlesPrivate.h>
#include <common/foundation/Allocator.h>

#include <PxPhysicsAPI.h>

namespace omni
{
namespace physx
{
namespace particles
{

class PhysxParticleSampler;
typedef std::map<pxr::SdfPath, PhysxParticleSampler*> PathToSamplerMap;

/*
 *  Particle Factory: object that holds a ref and coordinates all the samplers going into the same
 *  particle prim.
 */
class PhysxParticleFactory : public Allocateable
{
public:
    PhysxParticleFactory(pxr::SdfPath particlePath);
    ~PhysxParticleFactory();

    void addSampler(pxr::SdfPath samplerPath);
    bool updateSampler(pxr::SdfPath samplerPath, bool forceResampling);
    void removeSampler(pxr::SdfPath samplerPath);
    void processParticleSamplingResults(pxr::SdfPath samplerPath,
                                        const pxr::GfVec3f* positions,
                                        size_t numPoints,
                                        float pointWidth,
                                        const pxr::GfMatrix4d& rigidTransform,
                                        const pxr::GfMatrix3d& shearScaleTransform,
                                        bool registerOriginalCount);
    bool empty()
    {
        return mSamplers.empty();
    }
    PhysxParticleSampler* getParticleSampler(pxr::SdfPath samplerPath);

    // static - callback for samplingResults
    static void processSamplingResults(pxr::SdfPath samplerPath,
                                       pxr::SdfPath particleSetPath,
                                       const pxr::GfVec3f* positions,
                                       size_t numPoints,
                                       float pointWidth,
                                       const pxr::GfMatrix4d& rigidTransform,
                                       const pxr::GfMatrix3d& shearScaleTransform,
                                       bool registerOriginalCount);

    static bool getDecomposedTransform(pxr::SdfPath samplerPath,
                                       pxr::SdfPath particleSetPath,
                                       pxr::GfMatrix4d& rigidTransform,
                                       pxr::GfMatrix3d& shearScaleTransform);

private:
    void moveStartIndices(int firstChangedIndex, int correction);
    bool checkTargetCountsAndSizes(bool registerOriginalCount);
    void resetStartAndCountForAllSamplers();
    void saveTotalCount();

    pxr::SdfPath mPath;
    PathToSamplerMap mSamplers;
    bool mInitialized;
    size_t mTotalParticleCount;
};

/*
 *  Particle Sampler: object that holds all the information for a particle sampler.
 *  1-to-1 mapping between mesh with sampler API and particle sampler object.
 *
 *  if mStartIndex and mParticleCount are 0, that means the sampler has not been
 *  initialized or something was off with the total particle count of the prim we're
 *  writing the results to has been compromised. In that case we start from scratch,
 *  but the other samplers sampling into that prim have to be resampled manually.
 *
 */
class PhysxParticleSampler : public Allocateable
{
public:
    PhysxParticleSampler(pxr::SdfPath path, pxr::SdfPath target);
    ~PhysxParticleSampler();

    bool update(bool forceResampling);
    bool getDecomposedTransform(pxr::GfMatrix4d& rigidTransform, pxr::GfMatrix3d& shearScaleTransform) const;
    bool checkTransforms(bool& resample, pxr::GfMatrix4d& newRigidTransform);

    bool processSamplingResults(const pxr::GfVec3f* positions,
                                size_t numPoints,
                                float pointWidth,
                                const pxr::GfMatrix4d& rigidTransform,
                                const pxr::GfMatrix3d& shearScaleTransform,
                                bool factoryInitialized,
                                bool recreate,
                                int& firstChangedIndex,
                                int& shiftValue);

    bool processSamplingRegistration(size_t numPoints,
                                     size_t totalRegisteredPoints,
                                     const pxr::GfMatrix4d& rigidTransform,
                                     const pxr::GfMatrix3d& shearScaleTransform);

    void moveStartIndex(int firstChangedIndex, int correction);
    uint32_t getStartIndex()
    {
        return mStartIndex;
    }
    void setStartIndex(uint32_t index)
    {
        mStartIndex = index;
    }
    uint32_t getParticleCount()
    {
        return mParticleCount;
    }
    void setParticleCount(uint32_t count)
    {
        mParticleCount = count;
    }
    pxr::SdfPath getTarget()
    {
        return mTarget;
    }

private:
    void transformPoints(pxr::GfMatrix4d& newRigidTransform);

    pxr::SdfPath mSamplerPath;
    pxr::SdfPath mTarget;
    uint32_t mStartIndex;
    uint32_t mParticleCount;
    pxr::GfMatrix4d mRigidTransform;
    pxr::GfMatrix3d mShearScaleTransform;
};

// IPhysxParticlesPrivate API
void createParticleSampler(pxr::SdfPath path, pxr::SdfPath particlePrimPath);
void updateParticleSampler(pxr::SdfPath path, pxr::SdfPath particlePrimPath, bool forceResampling);
void removeParticleSampler(pxr::SdfPath path, pxr::SdfPath particlePrimPath);

} // namespace particles
} // namespace physx
} // namespace omni
