// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2
 */

#pragma once

#include <omni/physics/parse/UsdBackingAuthor.h> // omni::physics::parse::IUsdBackingAuthor

#include <memory>

namespace omni::physics::usd
{

// Factory for the USD backing-stage author (ADR-0027 bridge collapse). Builds the concrete
// IUsdBackingAuthor that reads the live UsdStageWeakPtr out of the opaque handle storage and
// reuses the USD backend authoring (createDefaultPhysicsScene / removeDefaultPhysicsScene /
// UsdPhysicsDataWrite). The loader installs the result via parse::setUsdBackingAuthor; the
// USD-free omni.physx side drives it through the pxr-free IUsdBackingAuthor interface only.
std::unique_ptr<omni::physics::parse::IUsdBackingAuthor> makeUsdBackingAuthor();

} // namespace omni::physics::usd
