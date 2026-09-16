// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-ATTRS-001
 * @covers AC-17
 *
 * @implements REQ-READ-COVERAGE-001
 * @covers AC-5
 */

#pragma once

#include <cstdint>

namespace omni
{
namespace physx
{
namespace tensors
{

// Per-output-slot record for the ovstage articulation-link read; dependency-free so the CUDA gather and the
// host path share one declaration. The read emits a flat list of link prims while the view, the cache and the
// DirectGPU buffer are indexed by (articulation, link slot), so each slot names its own source:
//   dst[i] = <attribute>(link[viewArtiIdx][physxLinkIdx]).
//
// `physxLinkIdx` is the index WITHIN the articulation (PxArticulationLink::getLinkIndex). It cannot be
// pre-multiplied into a flat offset because the view's maxLinks stride and the scene's differ.
//
// Carries no scale or sign: linkIncomingJointForce is a spatial force whose frame (USD joint-pose rotation
// and body-order swap included) is resolved as it is read, so folding it here would apply it twice.
struct ArticulationLinkOvStageRecord
{
    uint32_t viewArtiIdx = 0xffffffff;  // articulation's row in the view
    uint32_t physxLinkIdx = 0xffffffff; // link's index within that articulation
};

} // namespace tensors
} // namespace physx
} // namespace omni
