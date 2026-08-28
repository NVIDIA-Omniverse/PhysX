// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-2 AC-3
 */
#pragma once

// USD parse backend (ADR-0005). The default backend installed by the runtime:
// it builds a UsdSource + UsdPhysicsDataWrite + UsdChangeFeed for a USD stage.
//
// Factory only — the concrete backend type stays private to the
// omni.physics.usd library. The runtime installs it via
// `omni::physics::parse::setParseBackend(makeUsdParseBackend())`.

#include <memory>

namespace omni { namespace physics { namespace parse { class IParseBackend; } } }

namespace omni
{
namespace physics
{
namespace usd
{

// Create the USD parse backend. The returned backend interprets
// `AttachTarget::nativeStage` as a `const PXR_NS::UsdStageWeakPtr*`.
std::unique_ptr<omni::physics::parse::IParseBackend> makeUsdParseBackend();

} // namespace usd
} // namespace physics
} // namespace omni
