// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <atomic>
#include <cstdint>
#include <limits>

namespace ovphysx
{
namespace internal
{

// Hand out the next serial of an opaque-object handle sequence. The counter is
// taken by reference so this stays a pure function over caller-owned state and
// stays unit-testable; the process-wide instance lives in ovphysx.cpp.
//
// Invariants: values are strictly increasing and never reused, so a serial
// identifies one object for the life of the sequence even after that object is
// destroyed. Zero is both the public invalid-handle sentinel and the sticky
// exhausted state: after handing out UINT64_MAX the counter latches at zero and
// every later call fails instead of wrapping back onto live handles.
//
// Relaxed ordering is sufficient: the atomic modification order alone
// guarantees unique values, and the object maps' mutexes publish the
// corresponding payloads.
inline uint64_t allocateOpaqueObjectHandle(std::atomic<uint64_t>& nextHandle) noexcept
{
    uint64_t current = nextHandle.load(std::memory_order_relaxed);
    for (;;)
    {
        if (current == 0)
            return 0;

        const uint64_t next = current == std::numeric_limits<uint64_t>::max() ? 0 : current + 1;
        if (nextHandle.compare_exchange_weak(current, next, std::memory_order_relaxed, std::memory_order_relaxed))
        {
            return current;
        }
    }
}

// Process-wide allocator for ovphysx-owned opaque objects: instances, tensor
// bindings, contact bindings and SDF views. The single backing atomic is
// defined in ovphysx.cpp, next to the rationale for making it process-wide, so
// every caller inside libovphysx draws from one namespace. Operation indices
// and backend-owned query/read handles are not object handles and deliberately
// do not use this allocator.
uint64_t allocateOpaqueObjectHandle() noexcept;

} // namespace internal
} // namespace ovphysx
