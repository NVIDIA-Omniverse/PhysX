// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-MATH-001
 * @covers AC-8 AC-9
 */

#include "common/foundation/DeformableCookingTransform.h"

#include "common/foundation/MatrixTools.h"

#include <algorithm>
#include <cmath>

using ::physx::PxMat44d;
using ::physx::PxVec3d;

namespace omni
{
namespace physx
{
namespace
{

namespace gfm = ::omni::physx::gfmath;

// The 1e-7 quantization the cooking space applies to the shear/scale
// decomposition, so that a small edit to `simToWorld` does not produce a new
// cache key. Every step below is the Gf spelling it replaced, element for
// element: `GfVec3d::GetNormalized()` clamps at GF_MIN_VECTOR_LENGTH rather than
// returning zero, and `GfRotation` is stored axis-angle in DEGREES, so the angle
// is quantized as a fraction of a turn and multiplied back by 360.
PxVec3d quantizedDir(const PxVec3d& dir)
{
    const double eps = 1e-7;
    const double epsInv = 1.0 / eps;
    const PxVec3d dirQuant(
        std::round(dir.x * epsInv) * eps, std::round(dir.y * epsInv) * eps, std::round(dir.z * epsInv) * eps);
    return gfm::getNormalized(dirQuant);
}

gfm::Rotation quantizedRotation(const gfm::Rotation& rotation)
{
    const double eps = 1e-7;
    const double epsInv = 1.0 / eps;
    const double angleUnit = rotation.angle / 360.0;
    const double angleQuant = std::round(angleUnit * epsInv) * eps * 360.0;
    return gfm::makeRotation(quantizedDir(rotation.axis), angleQuant);
}

// The quantized shear/scale transform, with the (normalized) scale magnitude
// factored out into `scaleAbs`. Translation and pivot position are dropped --
// the cooking space is translation-invariant by construction.
gfm::PivotTransform quantizedSkewTransform(double& scaleAbs, const gfm::PivotTransform& transformSkew)
{
    PxVec3d scaleNormalized = transformSkew.scale;
    scaleAbs = gfm::normalize(scaleNormalized);

    return gfm::makePivotTransform(PxVec3d(0.0, 0.0, 0.0), quantizedRotation(transformSkew.rotation),
                                   quantizedDir(scaleNormalized), PxVec3d(0.0, 0.0, 0.0),
                                   quantizedRotation(transformSkew.pivotOrientation));
}

// Midpoint and largest extent of the fit points after `transform`. The
// accumulator is gfmath::Range3d rather than a hand-written min/max because
// GfRange3d has three bit traps the obvious spelling gets wrong: the empty seed
// is FLT_MAX in a double range, the midpoint is `0.5*min + 0.5*max`, and the
// min/max update runs x, y, z in that order. An empty point set is reachable and
// must keep producing a zero midpoint and the 1e-7 scale floor.
void computeFitBounds(PxVec3d& translation,
                      double& scale,
                      const carb::Float3* points,
                      size_t pointCount,
                      const gfm::PivotTransform& transform)
{
    const PxMat44d m = gfm::composeWithPivot(transform);

    gfm::Range3d bounds = gfm::emptyRange();
    for (size_t i = 0; i < pointCount; ++i)
    {
        // Elementwise float -> double widening, which is exact and is what the
        // Gf version's `for (const GfVec3d& p : VtArray<GfVec3f>)` did.
        const carb::Float3& p = points[i];
        gfm::unionWith(bounds, gfm::transformPoint(m, PxVec3d(double(p.x), double(p.y), double(p.z))));
    }
    const PxVec3d dims = gfm::getSize(bounds);
    const double dimMax = std::max(std::max(dims.x, dims.y), dims.z);

    translation = gfm::getMidpoint(bounds);
    scale = std::max(1e-7, dimMax);
}

} // namespace

bool computeDeformableCookingTransform(PxMat44d* simToCookingTransform,
                                       PxMat44d* cookingToWorldTransform,
                                       double* cookingToWorldScale,
                                       const PxMat44d& simToWorld,
                                       const carb::Float3* boundsFitPoints,
                                       size_t boundsFitPointCount)
{
    PxMat44d simToWorldOrtho = simToWorld;
    if (!gfm::orthonormalize(simToWorldOrtho))
    {
        return false;
    }

    // Gf's `simToWorld * simToWorldOrtho.GetInverse()`.
    const PxMat44d simToWorldSkew = gfm::multiply(simToWorld, gfm::inverse(simToWorldOrtho));

    double scaleAbs;
    const gfm::PivotTransform skewQuant = quantizedSkewTransform(scaleAbs, gfm::decomposeWithPivot(simToWorldSkew));

    PxVec3d fbTrans;
    double fbScale;
    computeFitBounds(fbTrans, fbScale, boundsFitPoints, boundsFitPointCount, skewQuant);

    const gfm::Rotation pivotOrient = skewQuant.pivotOrientation;
    const gfm::Rotation rotation = skewQuant.rotation;
    const PxVec3d scale = skewQuant.scale;

    if (simToCookingTransform)
    {
        const PxMat44d matFbScaleInv = gfm::setScale(1.0 / fbScale);
        const PxMat44d matFbTransInv = gfm::setTranslate(PxVec3d(-fbTrans.x, -fbTrans.y, -fbTrans.z));
        const PxMat44d matScale = gfm::setScale(scale);
        const PxMat44d matOrientInv = gfm::setTransform(gfm::inverse(pivotOrient), PxVec3d(0.0, 0.0, 0.0));
        // Gf's `matOrientInv * matScale * matFbTransInv * matFbScaleInv`, kept
        // left-associated: reversing a product to PhysX operand order would
        // change the accumulation order, and these bytes are the cache key.
        *simToCookingTransform =
            gfm::multiply(gfm::multiply(gfm::multiply(matOrientInv, matScale), matFbTransInv), matFbScaleInv);
    }

    if (cookingToWorldTransform)
    {
        // The corresponding cooking space -> world space transform, ignoring the
        // pre-scale factor scaleAbs*fbScale.
        const PxMat44d matRot = gfm::setTransform(gfm::multiply(pivotOrient, rotation), PxVec3d(0.0, 0.0, 0.0));
        const PxMat44d matTrans = gfm::setTransform(
            gfm::makeRotation(PxVec3d(1.0, 0.0, 0.0), 0.0),
            PxVec3d(fbTrans.x * scaleAbs, fbTrans.y * scaleAbs, fbTrans.z * scaleAbs));
        *cookingToWorldTransform = gfm::multiply(gfm::multiply(matTrans, matRot), simToWorldOrtho);
    }

    if (cookingToWorldScale)
    {
        *cookingToWorldScale = scaleAbs * fbScale;
    }
    return true;
}

} // namespace physx
} // namespace omni
