// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

namespace omni
{
namespace physx
{

/**
\brief Physics debug-visualization mode
*/
enum class SceneMultiGPUMode : int
{
    eDisabled = 0, //!< Disabled.
    eAll = 1, //!< Assign all CUDA devices to scenes in a rotating fashion.
    eSkipFirst = 2 //!< Assign all CUDA devices to scenes in a rotating fashion except the first device.
};

} // namespace physx
} // namespace omni
