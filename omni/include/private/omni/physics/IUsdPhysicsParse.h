// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include <private/omni/physics/schema/IUsdPhysicsListener.h>

namespace omni
{
namespace physics
{
namespace usdparser
{

struct IUsdPhysicsParse
{
    CARB_PLUGIN_INTERFACE("omni::physics::usdparser::IUsdPhysicsParse", 1, 0)

    /// Parse joint desc from a given Usd path
    ///
    /// \param[in] stageId StageId for the given path
    /// \param[in] path Joint usd path
    /// \return Joint descriptor, note that memory is owned by omni.physics and it must be released with
    /// IUsdPhysicsParse::releaseDesc
    schema::JointDesc*(CARB_ABI* parseJoint)(uint64_t stageId, const pxr::SdfPath& path);

    /// Release memory allocated by any of the IUsdPhysicsParse::parse* methods
    ///
    /// \param[in] desc allocated by this interface to be released
    void(CARB_ABI* releaseDesc)(schema::ObjectDesc* desc);

    /// Parse attachment desc from a given Usd path
    ///
    /// \param[in] stageId StageId for the given path
    /// \param[in] path Attachment usd path
    /// \return Attachment descriptor, note that memory is owned by omni.physics and it must be released with
    /// IUsdPhysicsParse::releaseDesc
    schema::AttachmentDesc*(CARB_ABI* parseAttachment)(uint64_t stageId, const pxr::SdfPath& path);

    /// Parse element collision filter desc from a given Usd path
    ///
    /// \param[in] stageId StageId for the given path
    /// \param[in] path Element collision filter usd path
    /// \return Element collision filter descriptor, note that memory is owned by omni.physics and it must be released
    /// with IUsdPhysicsParse::releaseDesc
    schema::ElementCollisionFilterDesc*(CARB_ABI* parseCollisionFilter)(uint64_t stageId, const pxr::SdfPath& path);
};

} // namespace usdparser
} // namespace physics
} // namespace omni
