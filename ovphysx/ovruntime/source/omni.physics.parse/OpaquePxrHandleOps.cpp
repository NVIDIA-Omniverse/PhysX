// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2 AC-3
 *
 * @implements REQ-BUILD-BRIDGE-001
 * @covers AC-3
 */

#include <omni/physics/parse/OpaquePxrHandleOps.h>

namespace omni::physics::parse
{
namespace
{
// File-static slots, null until the USD library installs its tables (ScanBackend.cpp shape).
const OpaquePxrHandleOps* sStageOps = nullptr;
const OpaquePxrHandleOps* sLayerOps = nullptr;
} // namespace

void setAttachedStageUsdHandleOps(const OpaquePxrHandleOps* ops) noexcept
{
    sStageOps = ops;
}

const OpaquePxrHandleOps* attachedStageUsdHandleOps() noexcept
{
    return sStageOps;
}

void setSimulationLayerHandleOps(const OpaquePxrHandleOps* ops) noexcept
{
    sLayerOps = ops;
}

const OpaquePxrHandleOps* simulationLayerHandleOps() noexcept
{
    return sLayerOps;
}

} // namespace omni::physics::parse
