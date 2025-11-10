// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

namespace omni
{
namespace physx
{
namespace ui
{

/**
\brief Physics debug-visualization mode
*/
enum class VisualizerMode : int
{
    eNone = 0, //!< Debug visualization disabled
    eSelected = 1, //!< Debug visualization enabled for selected objects
    eAll = 2 //!< Debug visualization enabled for all objects
};

} // namespace ui
} // namespace physx
} // namespace omni
