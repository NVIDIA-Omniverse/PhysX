// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>
#include <omni/physx/ObjectId.h>

namespace omni
{
namespace physx
{

/// Joint State data output by IPhysxJoint::getJointStateData
struct JointStateData
{
    bool body0IsParentLink; //!< True if in articulation first joint body is parent of second
    bool enabled[6]; //!< True if Joint State api is applied on corresponding DOF
    int physxAxis[6]; //!< The physx::PxArticulationAxis for this DOF
    bool convertToDegrees[6]; //!< True if this is a rotational DOF
    float initialPosition[6]; //!< Initial Joint position at simulation start (already converted to degrees if
                              //!< necessary)
    float initialVelocity[6]; //!< Initial Joint velocity at simulation start (already converted to degrees if
                              //!< necessary)
    uint64_t fabricTokenC[6]; //!< The corresponding axis usd token
};


/// Expose physx specific joint data. 
struct IPhysxJoint
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxJoint", 1, 0)

    /// Get internal joint state data for given object id
    ///
    /// \param[in] ids      Joint object ID (see getObjectId() to get the ID from a USD path)
    /// \param[out] data    Output joint state object
    void(CARB_ABI* getJointStateData)(usdparser::ObjectId id, JointStateData* data);

    /// Get the rotation that was applied to the joint frame during USD parsing
    ///
    /// This is the rotation that is applied to the USD joint frame during USD parsing
    /// This rotation can be used to rotate SDK outputs to USD frame again.
    ///
    /// \param[in] id The object id of the queried joint
    /// \return The rotation as a carb::Float4 in PxQuat layout: (x, y, z, w)
    carb::Float4(CARB_ABI* getJointFramePhysxToUsdQuat)(usdparser::ObjectId id);
};

} // namespace physx
} // namespace omni
