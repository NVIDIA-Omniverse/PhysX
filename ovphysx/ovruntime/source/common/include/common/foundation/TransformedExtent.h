// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-MATH-001
 * @covers AC-11
 */

#pragma once

#include <carb/Types.h>
#include <foundation/PxBounds3.h>
#include <foundation/PxMat44.h>

namespace omni
{
namespace physx
{

// Axis-aligned bounds of a world-space AABB after `m`, written into the two
// elements of a USD `extent` array.
//
// This is the element-for-element transcription of Arvo's method as
// implemented by `GfBBox3d(range, m).ComputeAlignedBox()` / `ComputeAlignedRange()`
// (OpenUSD `pxr/base/gf/bbox3d.cpp`, Copyright 2016 Pixar, Tomorrow Open
// Source Technology License 1.0 -- an Apache-2.0 variant; full text
// `ovphysx/tools/internal-licenses/OpenUSD-LICENSE.txt`, same provenance
// convention as the larger transcription in common/foundation/MatrixTools.h):
// Gf and PxMat44d hold the same sixteen values (Gf row j == PhysX column j),
// so `m[j][i]` indexes the same number in both spellings and no transpose is
// involved. The accumulation stays in double and is narrowed once, at the
// end, exactly as the Gf version's GfRange3d -> GfVec3f step did. An empty
// input range is passed through untransformed, matching
// GfBBox3d::ComputeAlignedRange(). This file is Apache-2.0 NVIDIA code;
// this comment is the modified-work notice/attribution Section 4(b)/(c) of
// the upstream license require.
//
// NOT bit-exact with GfBBox3d, however: TestTransformedExtent.cpp measured a
// worst-case divergence of 66 ULP (4.73e-6 relative) over a 2000-sample sweep
// -- this function's direct min/max accumulation and GfBBox3d's internal
// homogeneous-transform path round the same double arithmetic differently at
// the margins. REQ-MATH-001 AC-11 states the tolerance (1e-4 relative) that
// contract is held to.
//
// Moved out of omni.physx/plugins/internal/InternalScene.cpp's anonymous
// namespace (PLAN-gf-math-removal.md's bucket-A precedent) so a test file can
// pin the production function itself, not a copy of it.
void computeTransformedExtent(const ::physx::PxBounds3& worldBounds, const ::physx::PxMat44d& m,
                              carb::Float3 (&outExtent)[2]);

} // namespace physx
} // namespace omni
