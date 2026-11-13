// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-INPUT-CORE-001
// @covers AC-15

#pragma once

#include <atomic>
#include <cstdint>

namespace omni
{
namespace physx
{
namespace tensors
{

// Advanced whenever the runtime changes the centre of mass of a rigid body or link, through any path: the stage, a
// write session or a tensor binding. A view's centre-of-mass table records the value it was built at and is stale
// once it differs.
inline std::atomic<uint64_t>& centerOfMassEpoch()
{
    static std::atomic<uint64_t> epoch{ 0 };
    return epoch;
}

inline void centerOfMassChanged()
{
    centerOfMassEpoch().fetch_add(1, std::memory_order_relaxed);
}

} // namespace tensors
} // namespace physx
} // namespace omni
