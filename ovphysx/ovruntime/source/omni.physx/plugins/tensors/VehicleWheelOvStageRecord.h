// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-VEHICLE-001
 * @covers AC-1, AC-2
 */

#pragma once

#include <cstdint>

namespace omni
{
namespace physx
{
namespace tensors
{

// Per-output-slot record for the ovstage vehicle wheel read.
//
// The view is indexed by (vehicle, wheel) -- rows are [N, W, ...] as the rest of the tensor API
// indexes links under articulations -- while the read emits a FLAT list, one row per wheel prim
// across every queried vehicle. Each output slot therefore names its own source:
//   dst[i] = <transform>(vehicle[viewVehicleIdx].wheel[wheelIdx]).
//
// Host-only: a vehicle cannot be attached to a DirectGPU scene at all, so no part of its state ever
// lives on the device.
struct VehicleWheelOvStageRecord
{
    uint32_t viewVehicleIdx = 0xffffffff; // vehicle's row in the view
    uint32_t wheelIdx = 0xffffffff;       // wheel's index within that vehicle
};

// Which part of a wheel's world transform an output column carries. Composing the transform is the
// expensive half of the read, so the getter takes this rather than emitting a 7-wide column the
// caller would have to split.
enum class VehicleWheelComponent : uint32_t
{
    ePosition = 0,   // 3 floats
    eOrientation = 1 // 4 floats, xyzw
};

inline uint32_t vehicleWheelComponents(VehicleWheelComponent c)
{
    return c == VehicleWheelComponent::eOrientation ? 4u : 3u;
}

} // namespace tensors
} // namespace physx
} // namespace omni
