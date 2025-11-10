// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "UsdPCH.h"
#include <omni/physx/IPhysxVisualization.h>

namespace omni
{
namespace physx
{
namespace ui
{
/// returns debug draw simplex lines for unit AABB Debug Draw primitive
///
/// \param[in] color             color
/// \param[out] numLines         number of lines created
const DebugLine* getUnitAABBDebugDraw(uint32_t color, uint32_t& numLines);

/// returns debug draw simplex lines for "corner" unit AABB Debug Draw primitive
///
/// \param[in] color             color
/// \param[out] numLines         number of lines created
const DebugLine* getUnitCornerAABBDebugDraw(uint32_t color, uint32_t& numLines);

} // namespace ui
} // namespace physx
} // namespace omni
