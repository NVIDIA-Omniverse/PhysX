// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#pragma once
#include <carb/Defines.h>
#include <carb/Types.h>

namespace omni
{
namespace physx
{

struct IPhysxGpu
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxGpu", 1, 0)
    // No API — plugin exists to load PhysXGpu_64 as a side effect.
};

} // namespace physx
} // namespace omni
