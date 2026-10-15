// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-LOG-001
 * @covers AC-1 AC-3 AC-6
 *
 * @implements REQ-CAPI-CACHE-001
 * @covers AC-1 AC-2 AC-5
 *
 * @implements REQ-CAPI-PROFILER-002
 * @covers AC-4 AC-11 AC-12
 */

#pragma once

#include "internal/CarboniteLoaderState.hpp"
#include "internal/LogManagerState.hpp"

namespace ovphysx
{

// Internal process-wide storage. Each group retains its own synchronization
// and explicit resource cleanup; constructing this object starts no services.
struct GlobalState
{
    LogManagerState logging;
    CarboniteLoaderState carboniteLoader;
};

// Constructed eagerly with the library and retained through process teardown.
// Do not call from another static initializer or from a member constructor.
GlobalState& globalState() noexcept;

} // namespace ovphysx
