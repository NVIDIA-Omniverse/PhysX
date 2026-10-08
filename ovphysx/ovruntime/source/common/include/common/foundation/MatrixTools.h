// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-MATH-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-8
 */

#pragma once

#include <foundation/PxMat33.h>
#include <foundation/PxMat44.h>
#include <foundation/PxQuat.h>
#include <foundation/PxTransform.h>
#include <foundation/PxVec3.h>

// Double-precision 4x4 affine helpers on ::physx::PxMat44d, replacing the
// GfMatrix4d operations the runtime used to depend on. ADR-0001 §8: internal
// math is PhysX foundation math; Gf* belongs only at the USD boundary.
//
// LAYOUT. PxMat44d and GfMatrix4d hold the same sixteen doubles for the same
// transform: the first three groups of four are the X/Y/Z basis vectors and
// [12..14] is the translation. What differs is the reading -- USD reads that as
// row-major with a row-vector convention (v' = v*M), PhysX as column-major with
// a column-vector convention (v' = M*v). Consequences when porting:
//
//   Gf                                PhysX
//   ---------------------------------------------------------------
//   m.Transform(p)                    m.transform(p)
//   m.TransformDir(v)                 m.rotate(v)
//   A * B                             B * A            <-- operands SWAP
//   m.GetInverse()                    affineInverse(m)
//   m.GetOrthonormalized() /
//     m.RemoveScaleShear()            removeScaleShear(m)
//   m.ExtractTranslation()            m.getPosition()
//   m.ExtractRotationQuat()           toTransform(m).q
//   GfTransform(m).GetScale()         getScale(m)
//   m.SetIdentity()                   m = PxMat44d(::physx::PxIdentity)
//
// The operand swap is the trap: element-copying between the two types is
// correct and needs no transpose, but a *product* written in Gf order computes
// the transpose of what you want in PhysX order. See the sibling note on
// PxMat33(q) vs GfMatrix3f(q) -- those also hold identical elements.
//
// THE SWAP IS MATRIX-ONLY. A GfQuat product ports UNCHANGED, same operand order:
// GfQuat and PxQuat use the same Hamilton product with the same meaning, and the
// converters preserve components. In Gf, M(q1*q2) == M(q2)*M(q1), which cancels
// the matrix swap exactly. Measured over 5000 random rotation pairs: same order
// agrees to 7.8e-8, swapped is off by 9.99e-1 -- i.e. applying the matrix rule to
// a quaternion inverts the composition outright.

namespace omni
{
namespace physx
{

// Factor a 4x4 affine matrix into a rigid pose plus a per-axis scale.
//
// `m` is 16 doubles in the shared layout described above; the decomposition is
// correct under either reading. Shear is dropped into the rotation and scale
// stays a pure diagonal, matching GfTransform::GetScale(); a reflection in `m`
// is absorbed into a fully negative scale so the returned quaternion is always a
// proper rotation. Degenerate and non-finite input yields identity rather than NaNs.
//
// Ported from the clash-detection implementation
// (clash/extensions/clashdetection/.../plugins/Matrix.cpp), which is the version
// validated against OMPE-45979 (meshes with scale ~1e-4). Do not replace it with
// a normalize-the-basis-vectors shortcut -- that is what OMPE-45979 was.
//
// Measured against GfTransform over 30k random matrices (see the migration
// notes), three regimes:
//   * ordinary and mirrored transforms -- agrees with GfTransform to float
//     precision (quat 3.9e-8, scale 5.9e-8). Mirrors report all three scale
//     components negative, matching Gf.
//   * scales around 1e-4 -- Gf collapses (scale clamped to 1e-10, rotation lost);
//     this recomposes to 5.9e-8. That is OMPE-45979 and the divergence is the
//     point of using this instead.
//   * SHEARED matrices -- both drop the shear, but not identically (quat differs
//     by ~2e-2, scale by ~40%). A pose+diagonal-scale pair cannot represent
//     shear at all. Do NOT convert a site whose matrix can carry shear on the
//     strength of this function; report it instead.
void decomposeMatrix(::physx::PxTransform& pose, ::physx::PxVec3& scale, const double* m);

inline void decomposeMatrix(::physx::PxTransform& pose, ::physx::PxVec3& scale, const ::physx::PxMat44d& m)
{
    decomposeMatrix(pose, scale, m.front());
}

// Rigid part of `m`; any scale is computed and discarded.
// Gf equivalent: GfTransform(m).GetRotation() + m.ExtractTranslation().
inline ::physx::PxTransform toTransform(const ::physx::PxMat44d& m)
{
    ::physx::PxTransform pose;
    ::physx::PxVec3 scale;
    decomposeMatrix(pose, scale, m);
    return pose;
}

// Per-axis scale of `m`, shear dropped. Gf equivalent: GfTransform(m).GetScale().
// Note this is signed: a mirrored transform reports all three components
// negative, exactly as GfTransform does. Callers feeding PhysX geometry must
// take PxAbs.
inline ::physx::PxVec3 getScale(const ::physx::PxMat44d& m)
{
    ::physx::PxTransform pose;
    ::physx::PxVec3 scale;
    decomposeMatrix(pose, scale, m);
    return scale;
}

// Compose a matrix from a rigid pose and a per-axis scale (scale applied first,
// in the object's own frame). Gf equivalent: GfMatrix4d().SetTransform(rot, pos)
// pre-multiplied by a scale matrix.
::physx::PxMat44d makeMatrix(const ::physx::PxTransform& pose,
                             const ::physx::PxVec3& scale = ::physx::PxVec3(1.0f, 1.0f, 1.0f));

// `m` with scale and shear removed, translation preserved. Equivalent to
// GfMatrix4d::RemoveScaleShear() for any matrix WITHOUT shear (measured to
// 1e-4 elementwise over 500 random scaled matrices).
//
// On a SHEARED matrix it is not equivalent, and not merely by a rounding
// margin: Gf's RemoveScaleShear is not an orthonormalization. For the 3x3
// {{2,0,0},{0.5,3,0},{0,0,1}} it returns a frame rotated ~5.7 degrees about z,
// where both Gram-Schmidt and the polar factor return identity (max elementwise
// disagreement 6.7e-1 over random sheared matrices). Gf is factoring out a
// pivot orientation, which is a five-component decomposition PhysX has no
// spelling for. Sites that depend on that -- the deformable cooking transform
// and the particle Poisson sampler both do, deliberately -- must stay on
// GfTransform and convert at their boundary instead.
inline ::physx::PxMat44d removeScaleShear(const ::physx::PxMat44d& m)
{
    return makeMatrix(toTransform(m));
}

// General affine inverse of `m`, valid in the presence of non-uniform scale and
// shear (PxMat44::inverseRT() is not -- it assumes a rigid transform).
// Gf equivalent: m.GetInverse(). Returns identity for a singular matrix, which
// is what GfMatrix4d does for a zero determinant.
//
// Inversion commutes with transposition, so this is layout-agnostic: the same
// sixteen doubles in, the correct inverse out, under either reading.
::physx::PxMat44d affineInverse(const ::physx::PxMat44d& m);

// The upper-left 3x3 of `m` as a float PxMat33 (basis vectors preserved as
// columns, i.e. the PhysX reading). Gf equivalent: GfMatrix4d::ExtractRotationMatrix()
// when the matrix carries no scale, or the raw 3x3 block when it does.
inline ::physx::PxMat33 getBasis(const ::physx::PxMat44d& m)
{
    const double* d = m.front();
    return ::physx::PxMat33(::physx::PxVec3(float(d[0]), float(d[1]), float(d[2])),
                            ::physx::PxVec3(float(d[4]), float(d[5]), float(d[6])),
                            ::physx::PxVec3(float(d[8]), float(d[9]), float(d[10])));
}

// ===========================================================================
// gfmath -- BIT-EXACT transcriptions of the pxr Gf routines that have no PhysX
// spelling, so the runtime can drop its `libusd_gf` link (PLAN-gf-math-removal
// bucket A, DECIDED 2026-08-12 in favour of the verbatim port).
//
// READ THIS BEFORE TOUCHING ANYTHING BELOW.
//
// These are NOT reimplementations and must never become reimplementations.
// Their outputs are memcpy'd into cooking parameters that are hashed into a
// **Ujitso** cache key (CookingDataAsync.cpp `simToCookingTransform`,
// PhysXParticleSampling.cpp `shearScale`). Ujitso is a persistent, potentially
// shared asset cache, so a numerically different -- even if mathematically
// equivalent -- result is a cache INVALIDATION, not a refactor.
//
// The reason a "cleaner" version is not equivalent: `decomposeWithPivot` gets
// its pivot orientation from `factor`, whose rotation frame comes from
// `_jacobi3`, an UNSORTED cyclic Jacobi eigensolver. For rigid or uniformly
// scaled input -- the common case -- the stretch is a near-multiple of the
// identity, its eigenvectors are degenerate, and the frame that comes back is
// decided purely by floating-point rounding. Any independently-derived
// eigensolver returns an equally valid but DIFFERENT frame.
//
// So: preserve operation order, comparison order, loop iteration order, sweep
// and convergence thresholds, and every normalization step exactly as pxr does
// them. In particular note the two different division conventions pxr uses --
// GfVec3d/GfQuaternion `operator/=` multiplies by the reciprocal while GfQuatd
// `operator/=` divides the real part for real; both are reproduced verbatim and
// they do not round the same way.
//
// Transcribed from OpenUSD v25.11 (the version pinned in
// `_build/target-deps/usd`), Copyright 2016 Pixar. The dependency package
// ships this under the Tomorrow Open Source Technology License 1.0, an
// Apache-2.0 variant with a modified Section 6 (Trademarks); Section 4
// (Redistribution), the operative clause here, is unchanged from stock
// Apache-2.0. Full text: `ovphysx/tools/internal-licenses/OpenUSD-LICENSE.txt`
// (picked up by ovphysx's third-party license packaging). This file itself
// is Apache-2.0 NVIDIA code (see the SPDX header above); the routines
// below are the derivative work Section 4(b)/(c) require notice and
// attribution for -- this comment block, and the per-function file:line
// provenance list, are that notice. What changed from upstream: retyped from
// `Gf`/`pxr` types to `PxMat44d`/`PxVec3d`/`PxQuatd`, no `Tf`-based error
// reporting (`orthonormalize`'s convergence flag replaces `TF_WARN`), and
// each routine renamed per the mapping below.
// Sources, file:line:
//   pxr/base/gf/matrix4d.cpp:450  GfMatrix4d::Orthonormalize   -> orthonormalize
//   pxr/base/gf/matrix4d.cpp:304  GfMatrix4d::GetInverse       -> inverse
//   pxr/base/gf/matrix4d.cpp:568  GfMatrix4d::operator*=       -> multiply
//   pxr/base/gf/matrix4d.cpp:864  GfMatrix4d::Factor           -> factor
//   pxr/base/gf/matrix4d.cpp:921  GfMatrix4d::_Jacobi3         -> factor's kernel
//   pxr/base/gf/matrix4d.cpp:1023 GfMatrix4d::RemoveScaleShear -> removeScaleShearGf
//   pxr/base/gf/matrix4d.cpp:1040 GfMatrix4d::ExtractRotationQuat
//   pxr/base/gf/transform.cpp:41  GfTransform::SetMatrix       -> decomposeWithPivot
//   pxr/base/gf/transform.cpp:89  GfTransform::GetMatrix       -> composeWithPivot
//   pxr/base/gf/rotation.cpp:29   GfRotation::SetQuat
//   pxr/base/gf/rotation.cpp:76   GfRotation::GetQuat          -> getQuat
//   pxr/base/gf/rotation.cpp:629  GfRotation::operator*=       -> multiply(Rotation,Rotation)
//   pxr/base/gf/vec3d.cpp:94      GfOrthogonalizeBasis         -> orthonormalize's kernel
//
// LAYOUT. Everything here takes and returns `PxMat44d` holding the SAME sixteen
// doubles a `GfMatrix4d` would, in the same flat order -- the element-copy
// convention documented at the top of this file. That means the PxMat44d holds
// the TRANSPOSE linear map of the GfMatrix4d, which is exactly what keeps the
// cache bytes stable, and it means the operand swap applies: `gfmath::multiply(a, b)`
// computes Gf's `a * b` on those bytes, i.e. PhysX's `b * a`. Do not "fix" it.
// ===========================================================================
// SPDX-SnippetBegin
// SPDX-SnippetCopyrightText: Copyright 2016 Pixar
// SPDX-License-Identifier: Apache-2.0
// Modified from OpenUSD v25.11 pxr/base/gf/{matrix4d,transform,rotation,vec3d}.cpp
// (Tomorrow Open Source Technology License 1.0, an Apache-2.0 variant -- see
// ovphysx/tools/internal-licenses/OpenUSD-LICENSE.txt). See the file:line
// provenance table above for the per-routine source mapping and what changed.
namespace gfmath
{

// GfRotation: an axis (normalized on construction) and an angle in DEGREES.
// Deliberately not a PxQuat -- Gf carries the axis through unchanged for a zero
// angle, and quantization in the deformable cooking path reads axis and angle
// separately, so a quaternion round-trip would move bits.
struct Rotation
{
    ::physx::PxVec3d axis{ 1.0, 0.0, 0.0 };
    double angle{ 0.0 };
};

// GfTransform: the five-component decomposition, in Gf's own field order.
// M = -P * -O * S * O * R * P * T, with P = pivotPosition, O = pivotOrientation,
// S = scale, R = rotation, T = translation (all in Gf's row-vector order).
struct PivotTransform
{
    ::physx::PxVec3d scale{ 1.0, 1.0, 1.0 };
    Rotation pivotOrientation{};
    Rotation rotation{};
    ::physx::PxVec3d pivotPosition{ 0.0, 0.0, 0.0 };
    ::physx::PxVec3d translation{ 0.0, 0.0, 0.0 };
};

// GfRange3d, for the deformable cooking `computeFitBounds` accumulator. Three
// bit traps, all reproduced below: an empty range is seeded from FLOAT max
// (`FLT_MAX` in a *double* range, so ~3.4e38, not DBL_MAX); the midpoint is
// `0.5*min + 0.5*max`, NOT `(min+max)*0.5`; and the x/y/z comparison order in
// the min/max update is the order pxr writes.
struct Range3d
{
    ::physx::PxVec3d min;
    ::physx::PxVec3d max;
};

Range3d emptyRange();                                          // GfRange3d()
void unionWith(Range3d& r, const ::physx::PxVec3d& p);         // GfRange3d::UnionWith
::physx::PxVec3d getSize(const Range3d& r);                    // GfRange3d::GetSize
::physx::PxVec3d getMidpoint(const Range3d& r);                // GfRange3d::GetMidpoint

// --- the headline routine -------------------------------------------------

// GfTransform::SetMatrix(m), bit-exact, starting from an identity transform
// (i.e. `GfTransform(m)`), except that `pivotPosition` is taken from the caller
// exactly as GfTransform preserves its current one. Both bucket-A islands pass
// the default zero.
//
// The returned `pivotOrientation` is the component PhysX has no spelling for and
// the reason this function exists: it is left identity when the scale is exactly
// (1,1,1), matching Gf's `if (_scale != GfVec3d(1.0, 1.0, 1.0))` guard.
PivotTransform decomposeWithPivot(const ::physx::PxMat44d& m,
                                  const ::physx::PxVec3d& pivotPosition = ::physx::PxVec3d(0.0, 0.0, 0.0));

// GfTransform::GetMatrix(), bit-exact -- including the `_GF_ACCUM` skip logic,
// which is load-bearing: whether a component matrix is multiplied in at all
// depends on exact equality tests against identity, so a "harmless" always-
// multiply version returns different bits.
//
// This is `decomposeWithPivot`'s twin and MUST ship with it: the deformable
// cooking path quantizes the five components and then rebuilds a matrix from
// them, so without the composition direction the island cannot be retired.
::physx::PxMat44d composeWithPivot(const PivotTransform& t);

// The five-component constructor, in the argument order of GfTransform's "3x"
// constructor (`GfTransform(translation, rotation, scale, pivotPosition,
// pivotOrientation)`), which is the one the deformable cooking path uses. It is
// a plain field assignment -- Gf's `Set` does no arithmetic -- but it is spelled
// out so a port cannot silently transpose two same-typed arguments.
PivotTransform makePivotTransform(const ::physx::PxVec3d& translation,
                                  const Rotation& rotation,
                                  const ::physx::PxVec3d& scale,
                                  const ::physx::PxVec3d& pivotPosition,
                                  const Rotation& pivotOrientation);

// --- the primitives the islands also need ---------------------------------

// GfMatrix4d::Factor(). Any output pointer except `s`/`t`/`r`/`u`/`p` may be
// null; all five are required, matching the Gf signature. Returns false when the
// matrix is singular (Gf still fills the outputs in that case, and both callers
// use them regardless).
bool factor(const ::physx::PxMat44d& m,
            ::physx::PxMat44d* r,
            ::physx::PxVec3d* s,
            ::physx::PxMat44d* u,
            ::physx::PxVec3d* t,
            ::physx::PxMat44d* p,
            double eps = 1e-10);

// GfMatrix4d::Orthonormalize(issueWarning=false), bit-exact. Modifies `m` in
// place and returns the convergence flag (GF ISLAND #1 branches on it). The
// warning is dropped because the runtime has no TF_WARN.
bool orthonormalize(::physx::PxMat44d& m);

// GfMatrix4d::RemoveScaleShear(), bit-exact. NOT an orthonormalization -- see
// the note on the PhysX-semantics `omni::physx::removeScaleShear` above, which
// is a different function with a different answer on sheared input.
//
// The `Gf` suffix is deliberate and is the reason this is not spelled
// `removeScaleShear`: the two took identical `(const PxMat44d&) -> PxMat44d`
// signatures, so an unqualified call from inside `namespace omni::physx` bound
// silently to the PhysX-semantics twin -- no warning, no ADL rescue, and the
// wrong answer lands in a hashed cooking-cache key. Unqualified calls to the
// PhysX one are live today (InternalActor.cpp, PointInstancer.cpp), so the
// collision was reachable. Keep the names distinct.
::physx::PxMat44d removeScaleShearGf(const ::physx::PxMat44d& m);

// GfMatrix4d::GetInverse(), bit-exact -- a cofactor expansion, NOT the pivoted
// Gauss-Jordan that `omni::physx::affineInverse` uses. The two agree to ~1e-9
// but not to the bit, and the islands feed the result into a hashed transform.
::physx::PxMat44d inverse(const ::physx::PxMat44d& m, double eps = 0.0);

// Gf's `a * b` on the shared bytes (== PhysX's `b * a`). Present so a caller
// transcribing an island keeps Gf's accumulation order verbatim.
::physx::PxMat44d multiply(const ::physx::PxMat44d& a, const ::physx::PxMat44d& b);

// GfMatrix4d::Transform(v) -- Gf's row-vector transform with the homogeneous
// divide, on the shared bytes.
::physx::PxVec3d transformPoint(const ::physx::PxMat44d& m, const ::physx::PxVec3d& v);

// GfMatrix4d::ExtractRotation() / ExtractRotationQuat().
Rotation extractRotation(const ::physx::PxMat44d& m);

// Matrix builders, each the corresponding GfMatrix4d::SetXxx() applied to a
// default-constructed (all-zero) GfMatrix4d, so the untouched elements match.
::physx::PxMat44d setScale(const ::physx::PxVec3d& s);
::physx::PxMat44d setScale(double s);
::physx::PxMat44d setTranslate(const ::physx::PxVec3d& t);
::physx::PxMat44d setRotate(const Rotation& r);
// GfMatrix4d(rotate, translate) == SetRotate(r) then SetTranslateOnly(t).
::physx::PxMat44d setTransform(const Rotation& r, const ::physx::PxVec3d& t);

// GfRotation operations.
Rotation makeRotation(const ::physx::PxVec3d& axis, double angleInDegrees); // GfRotation(axis, angle)
Rotation identityRotation();                                                // GfRotation().SetIdentity()
Rotation inverse(const Rotation& r);                                        // GfRotation::GetInverse()
Rotation multiply(const Rotation& a, const Rotation& b);                    // Gf `a * b`
::physx::PxQuatd getQuat(const Rotation& r);                                // GfRotation::GetQuat()

// GfVec3d::Normalize() -- multiplies by the reciprocal and clamps at
// GF_MIN_VECTOR_LENGTH, returning the length BEFORE normalization. Exposed
// because the deformable cooking path uses the returned length as `scaleAbs`.
double normalize(::physx::PxVec3d& v);
::physx::PxVec3d getNormalized(const ::physx::PxVec3d& v); // GfVec3d::GetNormalized()

} // namespace gfmath
// SPDX-SnippetEnd

} // namespace physx
} // namespace omni
