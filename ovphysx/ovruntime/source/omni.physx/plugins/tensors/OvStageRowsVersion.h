// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-INSTANCER-001
 * @covers AC-1
 */

#pragma once

#include <atomic>
#include <cstdint>

namespace omni
{
namespace physx
{
namespace tensors
{

// Monotonic identity for an ovstage row list uploaded through GpuRigidBodyView::ovStageRowsDevice.
// The device copy is re-uploaded exactly when this value changes, so a producer must mint a NEW one
// when the row CONTENTS change and reuse the SAME one while they do not.
//
// Homed here, next to the row cache it identifies, so every producer -- the ovstage reads, the
// loose-rigid write, and the point-instancer write -- mints from ONE counter into ONE keyspace. Two
// counters would eventually hand out the same value for different lists, and the shared view would
// then keep one list's rows and index another list's data through them: wrong values, silently. A
// token derived from an object address is the same hazard once the address is reused (a freed view
// re-created at the old pointer aliases its predecessor's slot). The atomic starts at 1 because a
// cache slot reads token 0 as empty. Inline so the static is a single instance across the plugin.
inline uint64_t nextOvStageRowsVersion()
{
    static std::atomic<uint64_t> sVersion{ 0 };
    return ++sVersion;
}

} // namespace tensors
} // namespace physx
} // namespace omni
