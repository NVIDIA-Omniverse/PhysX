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

#include "internal/GlobalState.hpp"

#include <cassert>
#include <new>
#include <type_traits>

namespace ovphysx
{
namespace
{

// Constant-initialized so the debug assertion catches an accessor call before
// the passive state has been constructed.
bool g_constructed = false;

struct GlobalStateStorage
{
    alignas(GlobalState) unsigned char bytes[sizeof(GlobalState)];

    GlobalStateStorage()
    {
        ::new (static_cast<void*>(bytes)) GlobalState;
        g_constructed = true;
    }
};

// The holder owns storage, but deliberately does not register a destructor for
// its payload. Late Carbonite/atexit callbacks still need its locks and state.
// Existing explicit shutdown and last-loader paths release live resources.
static_assert(std::is_trivially_destructible<GlobalStateStorage>::value,
              "Global state storage must remain available during process teardown");
GlobalStateStorage g_storage;

} // namespace

GlobalState& globalState() noexcept
{
    assert(g_constructed);
    return *std::launder(reinterpret_cast<GlobalState*>(g_storage.bytes));
}

} // namespace ovphysx
