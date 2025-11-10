// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

using namespace carb;

namespace omni
{

namespace physx
{

struct IPhysxVisualizationPrivate
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxVisualizationPrivate", 1, 0)


    /// get the connected vertices of this mesh - 2 vertices form 1 line for debug viz.
    ///
    /// DEPRECATED, Will be replaced by new deformable implementation in future release.
    ///
    /// \param[in] usdPath              path to geometry mesh
    /// \param[in] ParticleClothDesc    parsed Physx particle cloth
    /// \param[in] invalidateCache      set to true to invalidate the cache
    /// \param[out] points              uint32_t array holding vertex indices
    void(CARB_ABI* getParticleClothDebugDraw)(const pxr::SdfPath& usdPath,
                                              usdparser::ParticleClothDesc* desc,
                                              pxr::VtArray<uint32_t>& points,
                                              bool invalidateCache);

    /// get the connected vertices of this mesh
    ///
    /// DEPRECATED, Will be replaced by new deformable implementation in future release.
    ///
    /// \param[out] pointIndices        uint32_t array holding vertex indices
    /// \param[in] primPath             path to geometry mesh
    void(CARB_ABI* getFEMClothDebugDraw)(pxr::VtArray<uint32_t>& pointIndices, const pxr::SdfPath& primPath);
};

} // namespace physx
} // namespace omni
