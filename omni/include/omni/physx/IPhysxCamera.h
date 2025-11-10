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

struct IPhysxCamera
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxCamera", 0, 1)

    /// Add a camera that follows the specified subject from a chase subject perspective. USD settings can be tuned to
    /// affect the behavior of the camera.
    ///
    /// \param[in] subjectPath The USD path of the subject to add a camera to follow.
    /// \param[in] cameraPath  The USD path for the camera to create.
    /// \return True if successful, else false (for example, cameraPath pointing to existing prim).
    bool(CARB_ABI* addFollowLookCamera)(const char* subjectPath, const char* cameraPath);

    /// Add a camera that follows the specified subject from a chase subject perspective. USD settings can be tuned to
    /// affect the behavior of the camera.
    ///
    /// \param[in] subjectPath The USD path of the subject to add a camera to follow.
    /// \param[in] cameraPath  The USD path for the camera to create.
    /// \return True if successful, else false (for example, cameraPath pointing to existing prim).
    bool(CARB_ABI* addFollowVelocityCamera)(const char* subjectPath, const char* cameraPath);

    /// Add a camera that follows the specified subject from an aerial drone perspective. USD settings can be tuned to
    /// affect the behavior of the camera.
    ///
    /// \param[in] subjectPath The USD path of the subject to add a camera to follow.
    /// \param[in] cameraPath  The USD path for the camera to create.
    /// \return True if successful, else false (for example, cameraPath pointing to existing prim).
    bool(CARB_ABI* addDroneCamera)(const char* subjectPath, const char* cameraPath);

    /// Attach to a USD stage explicitly. This will traverse the prims of the stage and create controllers
    /// as needed. This method should only be used if the controllers are going to be updated explicitly
    /// (see update()) instead of being tirggered by the default stage update loop.
    ///
    /// \note: the currently attached stage will be detached.
    ///
    /// \param[in] id                         USD stageId (can be retrieved from a stagePtr -
    ///                                       pxr::UsdUtilsStageCache::Get().GetId(stagePtr).ToLongInt())
    /// \param[in] unregisterFromStageUpdate  If true, the subject extension will not listen to stage update
    ///                                       events any longer.
    /// \return True if stage was successfully attached.
    bool(CARB_ABI* attachStage)(long id, bool unregisterFromStageUpdate);

    /// Detach from the USD stage. This will remove all subject related controller objects.
    ///
    /// \param[in] registerToStageUpdate  If true, the detach operation will be followed by a re-attach
    ///                                   and the subject extension will listen to stage update events again.
    void(CARB_ABI* detachStage)(bool registerToStageUpdate);

    /// Explicit pre-update step for cameras tracking both physics and animated cameras when not using the Play
    /// feature in Kit or when manually stepping the physics simulation.
    void(CARB_ABI* preUpdate)();

    /// Explicit update step for the animated cameras when not using the Play feature in Kit.
    /// Cameras that are tracking rigid body objects will get updated by the simulation event callback system.
    ///
    /// \param[in] elapsedSeconds      Elapsed time in seconds.
    void(CARB_ABI* update)(float elapsedSeconds);

    /// Explicit call to process buffered USD changes (for example, adding vehicle camera after simulation has started).
    ///
    /// This method should only be used in combination with update(), when explicitly stepping
    /// the simulation. The required order of operation is:
    ///
    /// 1. preUpdate() and/or update()
    /// 2. simulate/fetchResults() or updateSimulation() (omni.physx)
    /// 3. processPendingUSDChanges()
    void(CARB_ABI* processPendingUSDChanges)();
};


struct IPhysxCameraTesting
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxCameraTesting", 0, 1)
};

} // namespace physx
} // namespace omni
