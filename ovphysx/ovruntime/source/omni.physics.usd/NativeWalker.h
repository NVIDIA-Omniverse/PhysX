// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * Native USD prim walker for `scanStage`.
 *
 * Walks a USD stage (or sub-range) directly and emits parse-library
 * descriptors via the existing `parse::parse*` entry points.
 * Internal-only — callable from `StageScan.cpp`.
 *
 * @implements REQ-PARSE-SCAN-001
 * @covers AC-1 AC-2 AC-3
 */

#pragma once

#include <omni/physics/usd/StageScan.h>

#include <pxr/usd/usd/stage.h>

namespace omni::physics::schema
{
class PrimIteratorBase;
}

namespace omni::physics::usd
{
using namespace omni::physics::parse;

// Same contract as the public `scanStage(stage, primIterator)` —
// returns a `ScannedStage` populated by walking `stage` natively.
// `allocator` is propagated to every descriptor allocation produced
// by the scan (parse-lib parsers + walker-local allocations both
// route through `ParseContext::descriptorAllocator()`).
ScannedStage scanStageNative(PXR_NS::UsdStageWeakPtr stage,
                             omni::physics::schema::PrimIteratorBase& primIterator,
                             parse::IDescriptorAllocator& allocator);

// Resolve a prim's bound `UsdShadeMaterial` path for the "physics" purpose.
// Mirrors `usdmaterialutils::getMaterialBinding` (common/) verbatim so the USD
// backend stays free of a common/ dependency; the cooking service's
// `TriangulateUSDPrim::fillFaceMaterials` resolves subset materials through the
// very same `getMaterialBinding`, so the per-face material-index scheme is
// guaranteed identical between the cached and prim-id cooking paths.
PXR_NS::SdfPath getMaterialBindingPath(const PXR_NS::UsdPrim& usdPrim);

} // namespace omni::physics::usd
