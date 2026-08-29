// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

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
typedef std::map<PXR_NS::SdfPath, PhysxParticleSampler*> PathToSamplerMap;

/*
 *  Particle Factory: object that holds a ref and coordinates all the samplers going into the same
 *  particle prim.
 */
class PhysxParticleFactory : public Allocateable
{
public:
    PhysxParticleFactory(PXR_NS::SdfPath particlePath);
    ~PhysxParticleFactory();

    void addSampler(PXR_NS::SdfPath samplerPath);
    bool updateSampler(PXR_NS::SdfPath samplerPath, bool forceResampling);
    void removeSampler(PXR_NS::SdfPath samplerPath);
    void processParticleSamplingResults(PXR_NS::SdfPath samplerPath,
                                        const PXR_NS::GfVec3f* positions,
                                        size_t numPoints,
                                        float pointWidth,
                                        const PXR_NS::GfMatrix4d& rigidTransform,
                                        const PXR_NS::GfMatrix3d& shearScaleTransform,
                                        bool registerOriginalCount);
    bool empty()
    {
        return mSamplers.empty();
    }
    PhysxParticleSampler* getParticleSampler(PXR_NS::SdfPath samplerPath);

    // static - callback for samplingResults
    static void processSamplingResults(PXR_NS::SdfPath samplerPath,
                                       PXR_NS::SdfPath particleSetPath,
                                       const PXR_NS::GfVec3f* positions,
                                       size_t numPoints,
                                       float pointWidth,
                                       const PXR_NS::GfMatrix4d& rigidTransform,
                                       const PXR_NS::GfMatrix3d& shearScaleTransform,
                                       bool registerOriginalCount);

    static bool getDecomposedTransform(PXR_NS::SdfPath samplerPath,
                                       PXR_NS::SdfPath particleSetPath,
                                       PXR_NS::GfMatrix4d& rigidTransform,
                                       PXR_NS::GfMatrix3d& shearScaleTransform);

private:
    void moveStartIndices(int firstChangedIndex, int correction);
    void applyTotalCountDelta(int shiftValue);
    bool canRegisterOriginalCount(size_t numPoints);
    bool checkTargetCountsAndSizes(bool registerOriginalCount);
    void resetStartAndCountForAllSamplers();
    void saveTotalCount();

    PXR_NS::SdfPath mPath;
    PathToSamplerMap mSamplers;
    bool mInitialized;
    bool mHasAuthoredSamplingResults;
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
    PhysxParticleSampler(PXR_NS::SdfPath path, PXR_NS::SdfPath target);
    ~PhysxParticleSampler();

    bool update(bool forceResampling);
    bool getDecomposedTransform(PXR_NS::GfMatrix4d& rigidTransform, PXR_NS::GfMatrix3d& shearScaleTransform) const;
    bool checkTransforms(bool& resample, PXR_NS::GfMatrix4d& newRigidTransform);

    bool processSamplingResults(const PXR_NS::GfVec3f* positions,
                                size_t numPoints,
                                float pointWidth,
                                const PXR_NS::GfMatrix4d& rigidTransform,
                                const PXR_NS::GfMatrix3d& shearScaleTransform,
                                bool factoryInitialized,
                                bool recreate,
                                int& firstChangedIndex,
                                int& shiftValue,
                                bool& topologyChange);

    bool processSamplingRegistration(size_t numPoints,
                                     size_t totalRegisteredPoints,
                                     const PXR_NS::GfMatrix4d& rigidTransform,
                                     const PXR_NS::GfMatrix3d& shearScaleTransform);

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
    PXR_NS::SdfPath getTarget()
    {
        return mTarget;
    }

private:
    void transformPoints(PXR_NS::GfMatrix4d& newRigidTransform);

    PXR_NS::SdfPath mSamplerPath;
    PXR_NS::SdfPath mTarget;
    uint32_t mStartIndex;
    uint32_t mParticleCount;
    PXR_NS::GfMatrix4d mRigidTransform;
    PXR_NS::GfMatrix3d mShearScaleTransform;
};

// IPhysxParticlesPrivate API
void createParticleSampler(PXR_NS::SdfPath path, PXR_NS::SdfPath particlePrimPath);
void updateParticleSampler(PXR_NS::SdfPath path, PXR_NS::SdfPath particlePrimPath, bool forceResampling);
void removeParticleSampler(PXR_NS::SdfPath path, PXR_NS::SdfPath particlePrimPath);

} // namespace particles
} // namespace physx
} // namespace omni
