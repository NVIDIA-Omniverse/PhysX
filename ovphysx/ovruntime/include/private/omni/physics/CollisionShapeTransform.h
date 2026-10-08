// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-SHAPE-003
 * @covers AC-1 AC-2
 *
 * @implements REQ-MATH-001
 * @covers AC-13
 */

#pragma once

#include <common/foundation/MatrixTools.h>

#include <carb/Types.h>

#include <foundation/PxMat44.h>
#include <foundation/PxTransform.h>
#include <foundation/PxVec3.h>

namespace omni::physics
{

// Shared descriptor math; callers provide matrices from their own source.
//
// Matrices are ::physx::PxMat44d -- the same sixteen doubles a GfMatrix4d holds,
// read column-major (see common/foundation/MatrixTools.h). This header is
// deliberately Gf-free: it is included by both the USD and the ovstage walker,
// and the ovstage backend must not pull in pxr. A USD caller converts at the
// call boundary with omni.physics.usd/TypeCast.h's toPhysX(GfMatrix4d).
//
// `shapeToBody` is a relative transform between a collision gprim and its
// body, so a non-uniform scale on an ancestor plus a rotation between the two
// prims produces sheared input as a matter of course. `bodyToWorld` is
// equally shear-prone (the same ancestor pattern one level up), so both legs
// go through the same decomposition. The PhysX-native `decomposeMatrix` and
// Gf's `GfTransform` decomposition both drop the shear but do not agree on a
// sheared matrix (Gf factors out a pivot orientation PhysX has no spelling
// for -- see MatrixTools.h), so this uses the bit-exact Gf transcription
// (`gfmath::decomposeWithPivot`) throughout to keep collision-shape local
// pose/scale numerically unchanged from the pre-port behaviour instead of
// switching to the polar-style factor.
inline void decomposeCollisionShapeLocalTransform(const ::physx::PxMat44d& shapeToBody,
                                                  const ::physx::PxMat44d& bodyToWorld,
                                                  carb::Float3& outLocalPos,
                                                  carb::Float4& outLocalRot,
                                                  carb::Float3& outLocalScale)
{
    const omni::physx::gfmath::PivotTransform shapeLocal = omni::physx::gfmath::decomposeWithPivot(shapeToBody);
    const ::physx::PxQuatd shapeLocalRot = omni::physx::gfmath::getQuat(shapeLocal.rotation);

    // PhysX actors carry no body scale, so bake it into the child offset.
    // `bodyToWorld` can carry shear (non-uniform ancestor scale plus a
    // rotation below it in the chain to the body), so this must go through
    // the same pivot decomposition as the shape-local part above, not
    // `omni::physx::getScale` (MatrixTools.h documents ~40% scale divergence
    // between the two on sheared input).
    const ::physx::PxVec3d bodyScaleD = omni::physx::gfmath::decomposeWithPivot(bodyToWorld).scale;
    const ::physx::PxVec3 bodyScale(float(bodyScaleD.x), float(bodyScaleD.y), float(bodyScaleD.z));

    outLocalPos = { float(shapeLocal.translation.x * bodyScale.x), float(shapeLocal.translation.y * bodyScale.y),
                    float(shapeLocal.translation.z * bodyScale.z) };
    outLocalRot = { float(shapeLocalRot.x), float(shapeLocalRot.y), float(shapeLocalRot.z), float(shapeLocalRot.w) };
    outLocalScale = { float(shapeLocal.scale.x), float(shapeLocal.scale.y), float(shapeLocal.scale.z) };
}

} // namespace omni::physics
