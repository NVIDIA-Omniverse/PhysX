// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-MATH-001
 * @covers AC-8 AC-9
 */

#pragma once

#include <carb/Types.h>
#include <foundation/PxMat44.h>

#include <cstddef>

namespace omni
{
namespace physx
{

// Construction of the deformable cooking space, formerly "GF ISLAND #1" in
// omni.physx/plugins/CookingDataAsync.cpp.
//
// The cooking space is built so that it is invariant to translation, rotation
// and uniform scale of the deformable, and so that `boundsFitPoints` (in sim
// space) map into the unit cube around the origin.
//
// CACHE KEY. `simToCookingTransform`'s sixteen doubles are memcpy'd verbatim
// into three `*CookingParams` structs and hashed into the persistent **Ujitso**
// deformable cooking cache key (omni.physx.cooking/.../CookingHashing.h). The
// body is therefore written entirely in `omni::physx::gfmath` -- the bit-exact
// pxr transcriptions -- and not in the PhysX-semantics helpers that sit beside
// them in MatrixTools.h. `gfmath::inverse` is NOT `affineInverse`, and
// `gfmath::removeScaleShearGf` is NOT `omni::physx::removeScaleShear`; swapping
// either in produces a mathematically equivalent, numerically different matrix,
// which invalidates every cached entry for every user. Read the gfmath banner in
// MatrixTools.h before touching the implementation.
//
// LAYOUT. Every PxMat44d here holds the same sixteen doubles in the same flat
// order a GfMatrix4d would (the element-copy convention in TypeCast.h), so it
// carries the transpose linear map and Gf's `a * b` is `gfmath::multiply(a, b)`.
//
// The exact-bits pin is TestMatrixTools.cpp, which replays this function against
// `deformableutility::computeDeformableCookingTransform` -- an independent,
// still-Gf implementation of the same algorithm in the test tree that is
// deliberately NOT ported, because it is the only oracle not derived from this
// code.
//
// `cookingToWorldTransform` and `cookingToWorldScale` are optional and are NOT
// cache inputs (all three cooking call sites pass null for the transform); they
// feed the CUDA deformable skinning setup in usdInterface/UsdInterfaceDeformable.cpp.
//
// Returns false when `simToWorld` fails to orthonormalize; the outputs are then
// untouched.
bool computeDeformableCookingTransform(::physx::PxMat44d* simToCookingTransform,
                                       ::physx::PxMat44d* cookingToWorldTransform,
                                       double* cookingToWorldScale,
                                       const ::physx::PxMat44d& simToWorld,
                                       const carb::Float3* boundsFitPoints,
                                       size_t boundsFitPointCount);

} // namespace physx
} // namespace omni
