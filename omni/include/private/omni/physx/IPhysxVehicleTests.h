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

struct IPhysxVehicleTesting
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxVehicleTesting", 0, 2)

    /// Check whether a vehicle was created successfully.
    ///
    /// \note: should only be called after the simulation has been started.
    ///
    /// \param[in] path            Vehicle USD path.
    /// \return True if the vehicle was created successfully, else false.
    bool(CARB_ABI* doesVehicleExist)(const char* path);
};

} // namespace physx
} // namespace omni
