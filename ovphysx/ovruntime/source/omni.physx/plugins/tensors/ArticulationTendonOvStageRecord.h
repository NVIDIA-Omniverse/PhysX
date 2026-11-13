// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-TENDON-001
 * @covers AC-2, AC-3
 *
 * @implements REQ-SIM-OVSTAGE-WRITEAPPLY-001
 * @covers AC-21
 */

#pragma once

#include <cstdint>

namespace omni
{
namespace physx
{
namespace tensors
{

// Per-output-slot record for the ovstage tendon read. Device-agnostic and dependency-free, so the
// same declaration serves the CUDA gather and the host one.
//
// The read emits a FLAT list of tendons -- one row per tendon prim, across every queried
// articulation -- while both the view and the DirectGPU buffer are indexed by (articulation,
// tendon slot). Each output slot therefore names its own source:
//   dst[i] = <property>(tendon[viewArtiIdx][tendonIdx]).
//
// No unit scale here, even though a fixed tendon's length is the weighted sum of its axes' joint
// positions, which are RADIANS on a revolute axis: updateTendonAxisSingleGearing already folds
// rad2deg into the gearing coefficient, and parse, setters and dense getters are all passthrough,
// so converting here would break the round trip.
struct ArticulationTendonOvStageRecord
{
    uint32_t viewArtiIdx = 0xffffffff; // articulation's row in the view
    uint32_t tendonIdx = 0xffffffff;   // tendon's index within that articulation
};

// Which property of a tendon an output column carries.
//
// The values are FLOAT OFFSETS into PxGpuSpatialTendonData / PxGpuFixedTendonData, which are plain
// float structs (stiffness, damping, limitStiffness, offset [, lowLimit, highLimit, restLength]).
// That is what lets one gather kernel serve every attribute: only the offset and the component
// count vary. The host path switches on the same enum to pick a getter, so the two backends cannot
// disagree on what an attribute means. 5 is deliberately absent: it is highLimit, read as eLimit's
// SECOND component rather than named on its own, which is why eRestLength is 6.
// TestOvstageOutputReadTendons' GPU case is what pins these offsets to the structs.
enum class TendonProperty : uint32_t
{
    eStiffness = 0,
    eDamping = 1,
    eLimitStiffness = 2,
    eOffset = 3,
    eLimit = 4,     // 2 components (low, high) -- fixed tendons only
    eRestLength = 6 // fixed tendons only
};

// Component width of a property; only the fixed tendon's limit is a pair. Declared with the enum so
// the two backends and the reader that sizes the column cannot disagree about a width.
inline uint32_t tendonPropertyComponents(TendonProperty prop)
{
    return prop == TendonProperty::eLimit ? 2u : 1u;
}

// How a written value lands in its property. The ovstage drain writes one side of a fixed tendon's limit at a
// time and moves the other only when the two would cross, as the parse path's limit updaters do with the current
// pair; the pair lives on the device, hence a mode the scatter applies rather than a host-side merge.
enum class TendonFieldWrite : uint32_t
{
    eSet,         // the property's components = the values
    eLowPushHigh, // eLimit, one value per row: low = v, high = max(high, v)
    eHighPushLow  // eLimit, one value per row: high = v, low = min(low, v)
};

} // namespace tensors
} // namespace physx
} // namespace omni
