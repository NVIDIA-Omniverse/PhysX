// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

namespace omni
{
namespace physx
{

// particle post process types
struct ParticlePostFlag
{
    enum Enum
    {
        eNone = 0,
        eAnisotropy = (1 << 0), //< compute particle anisotropy based on neighborhood
        eSmoothing = (1 << 1), //< compute smoothed particle positions based on neighborhood
        eIsosurface = (1 << 2), //< compute particle isosurface mesh
    };
};

/// omni.physx particles interface
struct IPhysxParticlesPrivate
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxParticlesPrivate", 0, 1)

    /// Create postprocess for anisotropy, smoothing or isosurface, or fetch existing one
    ///
    ///\param[in] particleSystemPath Path to primitive of type PhysxSchemaPhysxParticleSystem
    ///\param[in] particlePostFlags Set of ParticlePostFlag
    void(CARB_ABI* createPostprocess)(const pxr::SdfPath& particleSystemPath, uint32_t particlePostFlags);

    /// Release postprocess
    ///
    ///\param[in] particleSystemPath Path to primitive of type PhysxSchemaPhysxParticleSystem
    void(CARB_ABI* releasePostprocess)(const pxr::SdfPath& particleSystemPath);

    /// Return enabled stages of postprocess
    ///
    ///\param[in] particleSystemPath Path to primitive of type PhysxSchemaPhysxParticleSystem
    ///\return Enabled ParticlePostFlag set
    uint32_t(CARB_ABI* getPostprocessStages)(const pxr::SdfPath& particleSystemPath);

    /// Set stages of postprocess
    ///
    ///\param[in] particleSystemPath Path to primitive of type PhysxSchemaPhysxParticleSystem
    ///\param[in] particlePostFlags Set of ParticlePostFlag
    void(CARB_ABI* setPostprocessStages)(const pxr::SdfPath& particleSystemPath, uint32_t particlePostFlags);

    /// Register particle set for postprocessing
    ///
    ///\param[in] particleSystemPath Path to primitive of type PhysxSchemaPhysxParticleSystem
    ///\param[in] particleSetPath Path to primitive with PhysxSchemaPhysxParticleSetAPI
    void(CARB_ABI* addPostprocessParticleSet)(const pxr::SdfPath& particleSystemPath,
                                              const pxr::SdfPath& particleSetPath);

    /// Unregister particle set for postprocessing
    ///
    ///\param[in] particleSystemPath Path to primitive of type PhysxSchemaPhysxParticleSystem
    ///\param[in] particleSetPath Path to primitive with PhysxSchemaPhysxParticleSetAPI
    void(CARB_ABI* removePostprocessParticleSet)(const pxr::SdfPath& particleSystemPath,
                                                 const pxr::SdfPath& particleSetPath);

    /// Update post process, including associated geometry i.e. UsdGeomPointInstancer, UsdGeomPoints, UsdGeomMesh
    ///
    ///\param[in] particleSystemPath Path to primitive of type PhysxSchemaPhysxParticleSystem
    void(CARB_ABI* updatePostprocess)(const pxr::SdfPath& particleSystemPath);


    /// Create a particle sampler at path pointing to the particle prim at particlePrimPath
    ///
    ///\param[in] path Path to a primitive with PhysxParticlePoissonSamplingAPI
    ///\param[in] particlePrimPath Path to the prim specified in the particles relationship of the sampling API
    void(CARB_ABI* createParticleSampler)(pxr::SdfPath path, pxr::SdfPath particlePrimPath);

    /// Update a particle sampler at path pointing to the particle prim at particlePrimPath
    ///
    ///\param[in] path Path to a primitive with PhysxParticlePoissonSamplingAPI
    ///\param[in] particlePrimPath Path to the prim specified in the particles relationship of the sampling API
    ///\param[in] forceResampling bool, force a resampling of this prim if true.
    void(CARB_ABI* updateParticleSampler)(pxr::SdfPath path, pxr::SdfPath particlePrimPath, bool forceResampling);

    /// remove a particle sampler at path pointing to the particle prim at particlePrimPath
    ///
    ///\param[in] path Path to a primitive with PhysxParticlePoissonSamplingAPI
    ///\param[in] particlePrimPath Path to the prim specified in the particles relationship of the sampling API
    void(CARB_ABI* removeParticleSampler)(pxr::SdfPath path, pxr::SdfPath particlePrimPath);
};

} // namespace physx
} // namespace omni
