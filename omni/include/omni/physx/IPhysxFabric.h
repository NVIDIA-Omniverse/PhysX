// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

namespace omni
{
namespace physx
{

static constexpr char kSettingFabricUpdateTransformations[] = "/physics/fabricUpdateTransformations";
static constexpr char kSettingFabricUpdateVelocities[] = "/physics/fabricUpdateVelocities";
static constexpr char kSettingFabricUpdateJointStates[] = "/physics/fabricUpdateJointStates";
static constexpr char kSettingFabricUpdatePoints[] = "/physics/fabricUpdatePoints";
static constexpr char kSettingFabricUseGPUInterop[] = "/physics/fabricUseGPUInterop";
static constexpr char kSettingFabricUpdateResiduals[] = "/physics/fabricUpdateResiduals";

struct IPhysxFabric
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxFabric", 1, 0)

    /// Attach USD stage.
    /// Note: previous stage will be detached.
    ///
    /// \param[in] id USD stageId (can be retrieved from a stagePtr -
    /// pxr::UsdUtilsStageCache::Get().GetId(stagePtr).ToLongInt()) \return True if stage was successfully attached.
    bool(CARB_ABI* attachStage)(long id);

    /// Detach USD stage, this will remove all objects from the PhysX SDK
    ///
    void(CARB_ABI* detachStage)();

    // Update fabric
    void(CARB_ABI* update)(float currentTime, float elapsedSecs);

    /// Save to USD
    /// Save current fabric data back to USD
    ///
    void(CARB_ABI* saveToUsd)();

    // Update fabric from PhysX data even if simulation did not run
    void(CARB_ABI* forceUpdate)(float currentTime, float elapsedSecs);

    // Enable update of Fabric transformations for kinematic bodies
    // \note For USD updates, the update for kinematic body transformations
    // cannot happen as this would break the time samples for example. Physics 
    // simulation should not be responsible for updating kinematic body transformations
    // as physics is not changing the transformation, the user is. However
    // in case of Fabric the update can be complex, hence omni.physx.fabric
    // offers a way how to update both dynamic and kinematic bodies, though 
    // strictly speaking this is not standard.
    // \note Changing this while simulation is already initialized will not have any impact
    // \note As this is helping to resolve issues in Isaac, its enabled by default
    /// \param[in] enable Enable update for kinematic body transformation in Fabric
    void(CARB_ABI* enableKinematicBodyTransformationUpdate)(bool enable);

    // Checks whether update of Fabric transformations for kinematic bodies is enabled
    // \return True if enabled
    bool(CARB_ABI* isKinematicBodyTransformationUpdateEnabled)();

};

} // namespace physx
} // namespace omni
