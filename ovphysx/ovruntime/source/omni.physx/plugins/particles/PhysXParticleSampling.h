// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-MATH-001
 * @covers AC-8 AC-9
 */

#pragma once

// PhysxParticleFactory/PhysxParticleSampler and the create/update/removeParticleSampler
// free functions implement the mesh-sampling particle-emission workflow. Split per the
// IPhysicsDataWrite pattern (ADR-0004/ADR-0005): everything here reads through
// IPhysicsSource and writes through IPhysicsDataWrite, keyed by ObjectKey -- no pxr
// dependency. A handful of genuinely USD-only cosmetic
// side effects (hydra "skip" hint, sampler-mesh visibility, CRC-property removal, the
// instancer prototype sphere radius) have no ovstage-native equivalent yet and stay
// narrowly fenced at their call sites in the .cpp (matching CookingDataAsync.cpp's
// storeVolumeDeformableBodyDataToUsd precedent), not declared here.
// decomposeSamplerTransform is pure PxMat44d/PxMat33d math with no pxr dependency, and
// TestMatrixTools.cpp pins its exact output bytes directly.

#include <private/omni/physx/ParticlePostFlag.h>
#include <common/foundation/Allocator.h>

#include <omni/physics/parse/Handles.h>

#include <carb/Types.h>
#include <PxPhysicsAPI.h>

#include <unordered_map>

// Convention for the sampler transforms below: they are element copies of the Gf
// matrices they replaced (same doubles, same linear order), so each holds the
// TRANSPOSE linear map of its Gf original - the tree-wide PhysX convention under
// which Gf `A*B` becomes PhysX `B*A`. `shearScaleTransform`'s nine doubles are
// memcpy'd verbatim into ParticlePoissonSamplingCookingParams::shearScale, which
// is CRC'd into the cooking cache key and reinterpret_cast back to a PxMat33d by
// omni.physx.cooking; the element-copy convention is exactly what keeps those
// bytes unchanged.

namespace omni
{
namespace physx
{
namespace particles
{

class PhysxParticleSampler;
typedef std::unordered_map<omni::physics::parse::ObjectKey, PhysxParticleSampler*, omni::physics::parse::ObjectKey::Hash>
    PathToSamplerMap;

/*
 *  Particle Factory: object that holds a ref and coordinates all the samplers going into the same
 *  particle prim.
 */
class PhysxParticleFactory : public Allocateable
{
public:
    PhysxParticleFactory(omni::physics::parse::ObjectKey particleKey);
    ~PhysxParticleFactory();

    void addSampler(omni::physics::parse::ObjectKey samplerKey);
    bool updateSampler(omni::physics::parse::ObjectKey samplerKey, bool forceResampling);
    void removeSampler(omni::physics::parse::ObjectKey samplerKey);
    void processParticleSamplingResults(omni::physics::parse::ObjectKey samplerKey,
                                        const carb::Float3* positions,
                                        size_t numPoints,
                                        float pointWidth,
                                        const ::physx::PxMat44d& rigidTransform,
                                        const ::physx::PxMat33d& shearScaleTransform,
                                        bool registerOriginalCount);
    bool empty()
    {
        return mSamplers.empty();
    }
    PhysxParticleSampler* getParticleSampler(omni::physics::parse::ObjectKey samplerKey);

    // static - callback for samplingResults
    static void processSamplingResults(omni::physics::parse::ObjectKey samplerKey,
                                       omni::physics::parse::ObjectKey particleSetKey,
                                       const carb::Float3* positions,
                                       size_t numPoints,
                                       float pointWidth,
                                       const ::physx::PxMat44d& rigidTransform,
                                       const ::physx::PxMat33d& shearScaleTransform,
                                       bool registerOriginalCount);

    static bool getDecomposedTransform(omni::physics::parse::ObjectKey samplerKey,
                                       omni::physics::parse::ObjectKey particleSetKey,
                                       ::physx::PxMat44d& rigidTransform,
                                       ::physx::PxMat33d& shearScaleTransform);

private:
    void moveStartIndices(int firstChangedIndex, int correction);
    void applyTotalCountDelta(int shiftValue);
    bool canRegisterOriginalCount(size_t numPoints);
    bool checkTargetCountsAndSizes(bool registerOriginalCount);
    void resetStartAndCountForAllSamplers();
    void saveTotalCount();

    omni::physics::parse::ObjectKey mParticleKey;
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
    PhysxParticleSampler(omni::physics::parse::ObjectKey key, omni::physics::parse::ObjectKey target);
    ~PhysxParticleSampler();

    bool update(bool forceResampling);
    bool getDecomposedTransform(::physx::PxMat44d& rigidTransform, ::physx::PxMat33d& shearScaleTransform) const;
    bool checkTransforms(bool& resample, ::physx::PxMat44d& newRigidTransform);

    bool processSamplingResults(const carb::Float3* positions,
                                size_t numPoints,
                                float pointWidth,
                                const ::physx::PxMat44d& rigidTransform,
                                const ::physx::PxMat33d& shearScaleTransform,
                                bool factoryInitialized,
                                bool recreate,
                                int& firstChangedIndex,
                                int& shiftValue,
                                bool& topologyChange);

    bool processSamplingRegistration(size_t numPoints,
                                     size_t totalRegisteredPoints,
                                     const ::physx::PxMat44d& rigidTransform,
                                     const ::physx::PxMat33d& shearScaleTransform);

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
    omni::physics::parse::ObjectKey getTarget()
    {
        return mTargetKey;
    }

private:
    void transformPoints(::physx::PxMat44d& newRigidTransform);

    omni::physics::parse::ObjectKey mSamplerKey;
    omni::physics::parse::ObjectKey mTargetKey;
    uint32_t mStartIndex;
    uint32_t mParticleCount;
    ::physx::PxMat44d mRigidTransform;
    ::physx::PxMat33d mShearScaleTransform;
};

// Split the sampler's local-to-world into the rigid part the sampled points are
// placed with and the shear/scale remainder that is hashed into the poisson
// sampling cache key. Exposed (rather than file-local) so the exact-bits parity
// pin in TestMatrixTools.cpp can compare the production bytes against Gf.
bool decomposeSamplerTransform(const ::physx::PxMat44d& l2w,
                               ::physx::PxMat44d& rigidTransform,
                               ::physx::PxMat33d& shearScaleTransform);

// Particle-sampler entry points (internal, ObjectKey-keyed)
void createParticleSampler(omni::physics::parse::ObjectKey samplerKey, omni::physics::parse::ObjectKey particlePrimKey);
void updateParticleSampler(omni::physics::parse::ObjectKey samplerKey,
                           omni::physics::parse::ObjectKey particlePrimKey,
                           bool forceResampling);
void removeParticleSampler(omni::physics::parse::ObjectKey samplerKey, omni::physics::parse::ObjectKey particlePrimKey);

} // namespace particles
} // namespace physx
} // namespace omni
