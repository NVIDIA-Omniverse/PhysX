// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2
 */

#pragma once

#include <omni/physics/parse/UsdReparse.h> // omni::physics::parse::IUsdReparse

#include <memory>

namespace omni::physics::usd
{

// Factory for the USD reparse backend (ADR-0027 seam #2). Builds the concrete
// IUsdReparse whose per-stage sessions own a live UsdStageWeakPtr plus the USD
// IPhysicsSource built by the USD parse backend, so the session is the ObjectKey
// minting authority for anything reparsed off that stage. The loader installs
// the result via parse::setUsdReparse; the USD-free omni.physx side drives it
// through the pxr-free IUsdReparse interface only.
std::unique_ptr<omni::physics::parse::IUsdReparse> makeUsdReparse();

} // namespace omni::physics::usd
