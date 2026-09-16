// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2
 *
 * @implements REQ-BUILD-BRIDGE-001
 * @covers AC-3
 */

#pragma once

#include <omni/physics/parse/OpaquePxrHandleOps.h>

namespace omni::physics::usd
{

// The USD-built operation table for the attached-stage handle (UsdStageWeakPtr).
// The installer hands this to parse::setAttachedStageUsdHandleOps.
const omni::physics::parse::OpaquePxrHandleOps* attachedStageUsdHandleOps();

// The USD-built operation table for the simulation-layer handle (SdfLayerRefPtr).
// The installer hands this to parse::setSimulationLayerHandleOps.
const omni::physics::parse::OpaquePxrHandleOps* simulationLayerHandleOps();

} // namespace omni::physics::usd
