// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-SCAN-001
 * @covers AC-1 AC-2 AC-3
 */
#pragma once

// Native-USD scan backend (ADR-0018 scanStage backend-registration gap).
// Stateless counterpart to UsdParseBackend.h's makeUsdParseBackend(): once
// registered, every omni::physics::parse::scanStage(AttachTarget, ...) caller
// gets the same native pxr walk omni::physics::usd::scanStage(AttachTarget,
// ...) already falls back to when no backend is registered, without a
// per-call usd::scanStage()/parse::scanStage() fork.
//
// Factory only -- the concrete backend type stays private to the
// omni.physics.usd library. UsdLoad::attach() installs it via
// omni::physics::parse::setScanBackend(makeUsdScanBackend()) for the
// lifetime of the plain (non-ovstage) attaches sharing it.

#include <memory>

namespace omni { namespace physics { namespace parse { class IScanBackend; } } }

namespace omni
{
namespace physics
{
namespace usd
{

// Create the native-USD scan backend. The returned backend interprets
// AttachTarget::nativeStage as a const PXR_NS::UsdStageWeakPtr*, exactly like
// makeUsdParseBackend()'s backend does for createSource().
std::unique_ptr<omni::physics::parse::IScanBackend> makeUsdScanBackend();

} // namespace usd
} // namespace physics
} // namespace omni
