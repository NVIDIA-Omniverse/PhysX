// SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

namespace omni
{

namespace physx
{

struct IPhysxAssetValidator
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxAssetValidator", 0, 1)

    /// Check if Joint States for given articulation root api are coherent with body transforms for the articulation.
    /// Stage must be already parsed and PhysX objects already created prior to calling this function (use
    /// IPhysX::forceLoadPhysicsFromUsd or equivalent)
    ///
    /// \param[in] stageId Stage Id containing the USD Path to check
    /// \param[in] primId USD Path of prim with an applied ArticulationRootAPI
    /// \return  True if the joint states are coherent with body transformations of the given ArticulationRootAPI
    bool(CARB_ABI* jointStateIsValid)(uint64_t stageId, uint64_t primId);


    /// Modifies bodies of the articulations to make them match what's specified by applied Joint States APIs
    /// Stage must be already parsed and PhysX objects already created prior to calling this function (use
    /// IPhysX::forceLoadPhysicsFromUsd or equivalent)
    ///
    /// \param[in] stageId Stage Id containing the USD Path to check
    /// \param[in] primId USD Path of prim with an applied ArticulationRootAPI
    void(CARB_ABI* jointStateApplyFix)(uint64_t stageId, uint64_t primId);

    /// Check if the stage is backward compatible with the current version of the asset validator.
    /// \param[in] stageId Stage Id containing the USD Path to check
    /// \return  True if the stage is backward compatible
    bool(CARB_ABI* backwardCompatibilityCheck)(uint64_t stageId);

    /// Get the log of the backward compatibility check.
    /// \return  The log of the backward compatibility check
    const char*(CARB_ABI* getBackwardCompatibilityLog)();

    /// Run backward compatibility check on the stage.
    /// \param[in] stageId Stage Id containing the USD Path to check
    void(CARB_ABI* runBackwardCompatibilityCheck)(uint64_t stageId);

    /// Check if resulting convex mesh is GPU compatible for given prim.
    /// Stage must be already parsed and PhysX objects already created prior to calling this function (use
    /// IPhysX::forceLoadPhysicsFromUsd or equivalent)
    ///
    /// \param[in] stageId Stage Id containing the USD Path to check
    /// \param[in] primId USD Path of prim with an applied Convex Hull approximation
    bool(CARB_ABI* convexGPUCompatibilityIsValid)(uint64_t stageId, uint64_t primId);

};


} // namespace physx
} // namespace omni
