// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-JOINT-001
 * @covers AC-2 AC-6
 */

#pragma once

#include <common/foundation/MatrixTools.h>

#include <carb/Types.h>

#include <foundation/PxMat44.h>
#include <foundation/PxQuat.h>
#include <foundation/PxTransform.h>
#include <foundation/PxVec3.h>

namespace omni::physics
{

// Convert a joint frame authored relative to a relationship target into the
// resolved body's local frame. PhysX actors carry no body scale, so the body's
// world scale is baked into the resulting translation.
//
// Matrices are ::physx::PxMat44d -- the same sixteen doubles a GfMatrix4d holds,
// read column-major (see common/foundation/MatrixTools.h). Products are written
// in PhysX operand order, i.e. the Gf `A * B` of the previous revision is
// `B * A` here. This header is Gf-free because it is shared by the USD and the
// ovstage walker; a USD caller converts with TypeCast.h at the call boundary.
//
// CAVEAT -- SHEAR. `bodyAnchor` (`inverse(bodyToWorld) * worldAnchor`, i.e.
// `worldAnchor * bodyToWorld.GetInverse()` in Gf order) genuinely contains
// shear when the relationship target and the body carry non-uniform scales
// under different rotations. Gf's RemoveScaleShear is NOT an
// orthonormalization: it factors out a pivot orientation (a five-component
// GfTransform decomposition) that PhysX has no spelling for, and the
// PhysX-native polar factor disagrees with it on sheared input (see
// MatrixTools.h). This uses the bit-exact Gf transcription
// (`gfmath::removeScaleShearGf`) to keep joint-frame local pose numerically
// unchanged from the pre-port behaviour.
inline void transformJointFrameToBody(const ::physx::PxMat44d& relationshipToWorld,
                                      const ::physx::PxMat44d& bodyToWorld,
                                      bool relationshipTargetsBody,
                                      carb::Float3& inOutLocalPosition,
                                      carb::Float4& inOutLocalOrientation)
{
    if (!relationshipTargetsBody)
    {
        // GfMatrix4d::SetTranslate + SetRotateOnly, i.e. a rigid pose matrix.
        const ::physx::PxMat44d localAnchor = omni::physx::makeMatrix(::physx::PxTransform(
            ::physx::PxVec3(inOutLocalPosition.x, inOutLocalPosition.y, inOutLocalPosition.z),
            ::physx::PxQuat(inOutLocalOrientation.x, inOutLocalOrientation.y, inOutLocalOrientation.z,
                            inOutLocalOrientation.w)));

        // Gf: localAnchor * relationshipToWorld  -> operands swap in PhysX order.
        const ::physx::PxMat44d worldAnchor = relationshipToWorld * localAnchor;
        // Gf: worldAnchor * bodyToWorld.GetInverse(), then RemoveScaleShear().
        // toTransform() is the rigid part of that product, which is exactly what
        // ExtractTranslation()/ExtractRotationQuat() read back off the
        // scale-and-shear-removed matrix.
        const ::physx::PxMat44d bodyAnchor = omni::physx::affineInverse(bodyToWorld) * worldAnchor;
        // Bit-exact Gf::RemoveScaleShear() first, then extract the rigid pose
        // from the now shear-free result -- decomposeMatrix/toTransform agrees
        // with Gf's ExtractTranslation/ExtractRotationQuat to float precision
        // once shear is gone, so this reproduces the pre-port answer instead of
        // the PhysX-native polar factor on the (possibly sheared) input.
        const ::physx::PxMat44d bodyAnchorNoShear = omni::physx::gfmath::removeScaleShearGf(bodyAnchor);
        ::physx::PxTransform bodyLocalAnchor = omni::physx::toTransform(bodyAnchorNoShear);
        bodyLocalAnchor.q.normalize();

        inOutLocalPosition = { bodyLocalAnchor.p.x, bodyLocalAnchor.p.y, bodyLocalAnchor.p.z };
        inOutLocalOrientation = { bodyLocalAnchor.q.x, bodyLocalAnchor.q.y, bodyLocalAnchor.q.z,
                                  bodyLocalAnchor.q.w };
    }

    // `bodyToWorld` can carry shear (non-uniform ancestor scale plus a
    // rotation below it in the chain to the body), so this must go through
    // the bit-exact Gf transcription (`gfmath::decomposeWithPivot`), not
    // `omni::physx::getScale` (MatrixTools.h documents ~40% scale divergence
    // between the two on sheared input).
    const ::physx::PxVec3d bodyScaleD = omni::physx::gfmath::decomposeWithPivot(bodyToWorld).scale;
    const ::physx::PxVec3 bodyScale(float(bodyScaleD.x), float(bodyScaleD.y), float(bodyScaleD.z));
    inOutLocalPosition = { inOutLocalPosition.x * bodyScale.x, inOutLocalPosition.y * bodyScale.y,
                           inOutLocalPosition.z * bodyScale.z };
}

} // namespace omni::physics
