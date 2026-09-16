// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2
 */

#pragma once

#include <omni/physics/parse/UsdStageLifecycle.h> // omni::physics::parse::IUsdStageLifecycle

#include <memory>

namespace omni::physics::usd
{

// Factory for the USD stage-lifecycle backend (ADR-0027 bridge collapse). Builds the concrete
// IUsdStageLifecycle that drives UsdStage::Open / CreateNew, the process UsdUtilsStageCache
// insert/erase, and SdfLayer::Find on behalf of the USD-free runtime-entry bridge. The loader
// installs the result via parse::setUsdStageLifecycle; the USD-free omni.physx side drives it
// through the pxr-free IUsdStageLifecycle interface only.
std::unique_ptr<omni::physics::parse::IUsdStageLifecycle> makeUsdStageLifecycle();

} // namespace omni::physics::usd
