// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-MATH-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-8
 */

#include "common/foundation/MatrixTools.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>

using namespace physx;

namespace omni
{
namespace physx
{
namespace
{

static PxQuat quatFromColumnMatrix(double n00, double n01, double n02,
                                   double n10, double n11, double n12,
                                   double n20, double n21, double n22)
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
    const double trace = n00 + n11 + n22;
    if (trace > 0.0)
    {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        w = 0.25 * s;
        x = (n21 - n12) / s;
        y = (n02 - n20) / s;
        z = (n10 - n01) / s;
    }
    else if (n00 > n11 && n00 > n22)
    {
        const double s = std::sqrt(1.0 + n00 - n11 - n22) * 2.0;
        w = (n21 - n12) / s;
        x = 0.25 * s;
        y = (n01 + n10) / s;
        z = (n02 + n20) / s;
    }
    else if (n11 > n22)
    {
        const double s = std::sqrt(1.0 + n11 - n00 - n22) * 2.0;
        w = (n02 - n20) / s;
        x = (n01 + n10) / s;
        y = 0.25 * s;
        z = (n12 + n21) / s;
    }
    else
    {
        const double s = std::sqrt(1.0 + n22 - n00 - n11) * 2.0;
        w = (n10 - n01) / s;
        x = (n02 + n20) / s;
        y = (n12 + n21) / s;
        z = 0.25 * s;
    }

    PxQuat q{ float(x), float(y), float(z), float(w) };
    const float mag2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (mag2 > 1.0e-12f && std::isfinite(mag2))
    {
        q.normalize();
        return q;
    }
    return PxQuat(PxIdentity);
}

} // anonymous namespace


// Robust TRS extraction: factor M = R * S with R orthogonal, S symmetric, via
// Gram-Schmidt (initial estimate, handles shear by projection) followed by
// Newton polar iteration  R_{k+1} = 0.5 * (R_k + R_k^-T)  which converges
// quadratically to the true polar factor. Scale is read off as diag(R^T * M)
// (matches GfTransform: shear is dropped into the rotation, scale stays a pure
// diagonal), and a reflection in M is absorbed into a fully negative scale (see
// the note at the det<0 branch). The flat layout
// is identical for USD GfMatrix4d (rows = basis vectors) and PxMat44d (columns
// = basis vectors), so this works for both callers.
void decomposeMatrix(PxTransform& pose, PxVec3& scale, const double* m)
{
    // Translation is copied straight from `m` with no arithmetic of its own
    // (unlike rotation/scale below, which are derived from the 3x3 block and
    // therefore inherit that block's isfinite guards), so it needs its own
    // up-front finiteness check to honour the header's "non-finite input
    // yields identity" contract. The 3x3 block cannot leak a NaN into this
    // translation: pose.p is set directly from m[12..14] and never touches
    // M/R/dot3/det3, so guarding the three translation components here is
    // sufficient -- a full 16-element check is not needed.
    if (!std::isfinite(m[12]) || !std::isfinite(m[13]) || !std::isfinite(m[14]))
    {
        pose = PxTransform(PxIdentity);
        scale = PxVec3(1.0f, 1.0f, 1.0f);
        return;
    }

    // Build the 3x3 affine block as three column vectors of a column-vector
    // rotation matrix. M[c] is column c, M[c][r] is row r within that column.
    const double M[3][3] = {
        { m[0],  m[1],  m[2]  },  // X basis
        { m[4],  m[5],  m[6]  },  // Y basis
        { m[8],  m[9],  m[10] },  // Z basis
    };
    pose.p = PxVec3(float(m[12]), float(m[13]), float(m[14]));

    auto dot3 = [](const double* a, const double* b) -> double {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    };
    auto det3 = [](const double R[3][3]) -> double {
        return R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
             - R[1][0] * (R[0][1] * R[2][2] - R[0][2] * R[2][1])
             + R[2][0] * (R[0][1] * R[1][2] - R[0][2] * R[1][1]);
    };

    // Phase 1: Gram-Schmidt for an initial near-orthogonal estimate.
    double R[3][3];
    for (int c = 0; c < 3; ++c)
        for (int r = 0; r < 3; ++r)
            R[c][r] = M[c][r];

    auto safeNormalize = [&](double* v, int fallbackAxis) {
        const double L = std::sqrt(dot3(v, v));
        if (L > 1.0e-12 && std::isfinite(L))
        {
            const double inv = 1.0 / L;
            v[0] *= inv; v[1] *= inv; v[2] *= inv;
        }
        else
        {
            v[0] = (fallbackAxis == 0) ? 1.0 : 0.0;
            v[1] = (fallbackAxis == 1) ? 1.0 : 0.0;
            v[2] = (fallbackAxis == 2) ? 1.0 : 0.0;
        }
    };

    safeNormalize(R[0], 0);
    const double p01 = dot3(R[0], R[1]);
    R[1][0] -= p01 * R[0][0]; R[1][1] -= p01 * R[0][1]; R[1][2] -= p01 * R[0][2];
    safeNormalize(R[1], 1);
    const double p02 = dot3(R[0], R[2]);
    const double p12 = dot3(R[1], R[2]);
    R[2][0] -= p02 * R[0][0] + p12 * R[1][0];
    R[2][1] -= p02 * R[0][1] + p12 * R[1][1];
    R[2][2] -= p02 * R[0][2] + p12 * R[1][2];
    safeNormalize(R[2], 2);

    // Phase 2: Newton polar refinement. From a Gram-Schmidt seed this typically
    // converges in 1-2 passes; the cap matches USD's robustness budget for
    // ill-conditioned matrices.
    for (int iter = 0; iter < 16; ++iter)
    {
        const double d = det3(R);
        if (!std::isfinite(d) || std::fabs(d) < 1.0e-20)
            break;
        const double invD = 1.0 / d;

        // Columns of R^-T are the cofactor columns of R divided by det(R).
        const double inv0[3] = {
            (R[1][1] * R[2][2] - R[1][2] * R[2][1]) * invD,
            (R[1][2] * R[2][0] - R[1][0] * R[2][2]) * invD,
            (R[1][0] * R[2][1] - R[1][1] * R[2][0]) * invD,
        };
        const double inv1[3] = {
            (R[2][1] * R[0][2] - R[2][2] * R[0][1]) * invD,
            (R[2][2] * R[0][0] - R[2][0] * R[0][2]) * invD,
            (R[2][0] * R[0][1] - R[2][1] * R[0][0]) * invD,
        };
        const double inv2[3] = {
            (R[0][1] * R[1][2] - R[0][2] * R[1][1]) * invD,
            (R[0][2] * R[1][0] - R[0][0] * R[1][2]) * invD,
            (R[0][0] * R[1][1] - R[0][1] * R[1][0]) * invD,
        };

        double maxDelta = 0.0;
        for (int r = 0; r < 3; ++r)
        {
            const double a0 = 0.5 * (R[0][r] + inv0[r]);
            const double a1 = 0.5 * (R[1][r] + inv1[r]);
            const double a2 = 0.5 * (R[2][r] + inv2[r]);
            const double d0 = std::fabs(R[0][r] - a0);
            const double d1 = std::fabs(R[1][r] - a1);
            const double d2 = std::fabs(R[2][r] - a2);
            if (d0 > maxDelta) maxDelta = d0;
            if (d1 > maxDelta) maxDelta = d1;
            if (d2 > maxDelta) maxDelta = d2;
            R[0][r] = a0; R[1][r] = a1; R[2][r] = a2;
        }
        if (maxDelta < 1.0e-12)
            break;
    }

    // Scale = diag(R^T * M_original): signed projection of each source basis
    // vector onto the orthonormal column. Off-diagonal stretch (shear) is
    // dropped — same convention as GfTransform::GetScale().
    float sx = float(dot3(R[0], M[0]));
    float sy = float(dot3(R[1], M[1]));
    float sz = float(dot3(R[2], M[2]));

    // Reflection: if R is improper (det < 0) negate ALL THREE columns, which
    // makes det positive again (negating an odd-sized basis flips its sign) and
    // pushes the reflection into a fully negative scale.
    //
    // This deliberately diverges from the clash-detection original, which flips
    // only the first column and negates only scale.x. Measured against
    // GfTransform::GetScale() over mirrored matrices: Gf reports all three
    // components negative, 300/300, for a mirror authored on any single axis.
    // Since this replaces GfTransform in the runtime, it has to reproduce that
    // convention -- a caller reading scale.y off a mirrored prim must keep
    // seeing what it saw before. Both conventions recompose to the same matrix.
    if (det3(R) < 0.0)
    {
        for (int c = 0; c < 3; ++c)
        {
            R[c][0] = -R[c][0]; R[c][1] = -R[c][1]; R[c][2] = -R[c][2];
        }
        sx = -sx;
        sy = -sy;
        sz = -sz;
    }

    if (!std::isfinite(sx)) sx = 1.0f;
    if (!std::isfinite(sy)) sy = 1.0f;
    if (!std::isfinite(sz)) sz = 1.0f;
    scale = PxVec3(sx, sy, sz);

    pose.q = quatFromColumnMatrix(
        R[0][0], R[1][0], R[2][0],
        R[0][1], R[1][1], R[2][1],
        R[0][2], R[1][2], R[2][2]);
}

// Compose scale-then-rotate-then-translate. The basis vectors are the rotated
// axes scaled component-wise, which is R * diag(scale) in PhysX column order.
PxMat44d makeMatrix(const PxTransform& pose, const PxVec3& scale)
{
    const PxMat33 basis(pose.q);
    double d[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
    d[0] = double(basis.column0.x) * double(scale.x);
    d[1] = double(basis.column0.y) * double(scale.x);
    d[2] = double(basis.column0.z) * double(scale.x);
    d[4] = double(basis.column1.x) * double(scale.y);
    d[5] = double(basis.column1.y) * double(scale.y);
    d[6] = double(basis.column1.z) * double(scale.y);
    d[8] = double(basis.column2.x) * double(scale.z);
    d[9] = double(basis.column2.y) * double(scale.z);
    d[10] = double(basis.column2.z) * double(scale.z);
    d[12] = double(pose.p.x);
    d[13] = double(pose.p.y);
    d[14] = double(pose.p.z);
    return PxMat44d(d);
}

// Gauss-Jordan with partial pivoting on the flat 4x4. Chosen over a cofactor
// expansion because the runtime inverts matrices with scales spanning several
// orders of magnitude (see OMPE-45979) and pivoting keeps those conditioned.
PxMat44d affineInverse(const PxMat44d& m)
{
    double a[4][8];
    const double* src = m.front();
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            a[i][j] = src[i * 4 + j];
            a[i][4 + j] = (i == j) ? 1.0 : 0.0;
        }
    }

    for (int col = 0; col < 4; ++col)
    {
        int pivot = col;
        double best = std::fabs(a[col][col]);
        for (int r = col + 1; r < 4; ++r)
        {
            const double v = std::fabs(a[r][col]);
            if (v > best)
            {
                best = v;
                pivot = r;
            }
        }
        if (!(best > 1.0e-300) || !std::isfinite(best))
            return PxMat44d(PxIdentity); // singular: GfMatrix4d also yields a fallback

        if (pivot != col)
            for (int j = 0; j < 8; ++j)
                std::swap(a[col][j], a[pivot][j]);

        const double inv = 1.0 / a[col][col];
        for (int j = 0; j < 8; ++j)
            a[col][j] *= inv;

        for (int r = 0; r < 4; ++r)
        {
            if (r == col)
                continue;
            const double f = a[r][col];
            if (f == 0.0)
                continue;
            for (int j = 0; j < 8; ++j)
                a[r][j] -= f * a[col][j];
        }
    }

    double dst[16];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            dst[i * 4 + j] = a[i][4 + j];

    if (!std::isfinite(dst[0]) || !std::isfinite(dst[15]))
        return PxMat44d(PxIdentity);
    return PxMat44d(dst);
}

// ===========================================================================
// gfmath -- bit-exact transcription of pxr Gf. See the long banner in
// MatrixTools.h for WHY this is a transcription and not an implementation.
//
// Ground rules followed throughout, all of them observable in the output bits:
//   * operation order, comparison order and loop iteration order are pxr's;
//   * pxr's two division conventions are kept apart. GfVec3d::operator/= and
//     GfQuaternion::operator/= multiply by the reciprocal; GfQuatd::operator/=
//     divides the real part and multiplies the imaginary part by the reciprocal
//     (because it delegates to GfVec3d). These do not round the same way;
//   * summation order in the two length-squared spellings differs between
//     GfQuatd (imaginary first) and GfQuaternion (real first) and is preserved;
//   * the working type is Gf's `double _mtx[4][4]`, indexed [row][col]. A
//     PxMat44d converts to it by a flat sixteen-double copy, which is the
//     element-copy convention -- so Gf row i is PhysX column i, and every
//     product written here in Gf order is the PhysX-order product reversed.
// ===========================================================================
// SPDX-SnippetBegin
// SPDX-SnippetCopyrightText: Copyright 2016 Pixar
// SPDX-License-Identifier: Apache-2.0
// Modified from OpenUSD v25.11 pxr/base/gf/{matrix4d,transform,rotation,vec3d}.cpp
// (Tomorrow Open Source Technology License 1.0, an Apache-2.0 variant -- see
// ovphysx/tools/internal-licenses/OpenUSD-LICENSE.txt). See MatrixTools.h's
// file:line provenance table for the per-routine source mapping and what changed.
namespace gfmath
{
namespace
{

// GF_MIN_VECTOR_LENGTH (pxr/base/gf/limits.h:17)
constexpr double kMinVectorLength = 1e-10;
// GF_MIN_ORTHO_TOLERANCE (pxr/base/gf/limits.h:22)
constexpr double kMinOrthoTolerance = 1e-6;
// The value of glibc's M_PI, which is what GfRadiansToDegrees /
// GfDegreesToRadians multiply by. Spelled out rather than included because M_PI
// is not portably visible from <cmath>; it is the exact double 0x400921FB54442D18.
constexpr double kPi = 3.14159265358979323846;

struct V3
{
    double v[3];
};

struct M4
{
    double m[4][4];
};

// GfQuaternion / GfQuatd share this shape; the two types differ only in which
// operators they expose, and those differences are spelled out at each use.
struct Q
{
    double r;
    V3 i;
};

inline M4 toM4(const PxMat44d& p)
{
    M4 a;
    const double* d = p.front();
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            a.m[i][j] = d[i * 4 + j];
    return a;
}

inline PxMat44d toPx(const M4& a)
{
    double d[16];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            d[i * 4 + j] = a.m[i][j];
    return PxMat44d(d);
}

inline V3 toV3(const PxVec3d& p)
{
    return V3{ { p.x, p.y, p.z } };
}

inline PxVec3d toPx(const V3& v)
{
    return PxVec3d(v.v[0], v.v[1], v.v[2]);
}

// --- GfVec3d ---------------------------------------------------------------

inline double dot(const V3& a, const V3& b)
{
    return a.v[0] * b.v[0] + a.v[1] * b.v[1] + a.v[2] * b.v[2];
}

inline double lengthSq(const V3& a)
{
    return dot(a, a);
}

inline double length(const V3& a)
{
    return std::sqrt(lengthSq(a));
}

inline V3 add(const V3& a, const V3& b)
{
    return V3{ { a.v[0] + b.v[0], a.v[1] + b.v[1], a.v[2] + b.v[2] } };
}

inline V3 sub(const V3& a, const V3& b)
{
    return V3{ { a.v[0] - b.v[0], a.v[1] - b.v[1], a.v[2] - b.v[2] } };
}

inline V3 scaled(const V3& a, double s)
{
    return V3{ { a.v[0] * s, a.v[1] * s, a.v[2] * s } };
}

// GfVec3d::operator/= (vec3d.h:202) -- multiplies by the reciprocal. Its own
// comment concedes this is less stable than dividing; do not "fix" it.
inline V3 divided(const V3& a, double s)
{
    return scaled(a, 1.0 / s);
}

// GfVec3d::Normalize (vec3d.h:252). Returns the length BEFORE normalizing, and
// divides by eps rather than by the length when the vector is shorter than eps.
inline double normalizeV3(V3& v, double eps = kMinVectorLength)
{
    const double len = length(v);
    v = divided(v, (len > eps) ? len : eps);
    return len;
}

// GfIsClose(GfVec3d, GfVec3d, tol) (vec3d.h:404) -- squared, and <=, not <.
inline bool isCloseV3(const V3& a, const V3& b, double tolerance)
{
    return lengthSq(sub(a, b)) <= tolerance * tolerance;
}

// GfIsClose(double, double, eps) (math.h:25) -- strict <, and no square.
inline bool isCloseD(double a, double b, double epsilon)
{
    return std::fabs(a - b) < epsilon;
}

inline double clampD(double value, double lo, double hi)
{
    if (value < lo)
        return lo;
    if (value > hi)
        return hi;
    return value;
}

inline double radiansToDegrees(double radians)
{
    return radians * (180.0 / kPi);
}

inline double degreesToRadians(double degrees)
{
    return degrees * (kPi / 180.0);
}

// GfSinCos -> ArchSinCos (arch/math.h:98). On Linux that is glibc's sincos(),
// which is NOT guaranteed to return the same bits as sin() and cos() called
// separately, so the platform split is reproduced rather than simplified.
inline void sinCos(double v, double* s, double* c)
{
#if defined(__linux__)
    ::sincos(v, s, c);
#else
    *s = std::sin(v);
    *c = std::cos(v);
#endif
}

// --- GfOrthogonalizeBasis (vec3d.cpp:94), normalize == true ----------------

bool orthogonalizeBasis(V3& tx, V3& ty, V3& tz, bool normalize, double eps = kMinOrthoTolerance)
{
    V3 ax, bx, cx, ay, by, cy, az, bz, cz;

    if (normalize)
    {
        normalizeV3(tx);
        normalizeV3(ty);
        normalizeV3(tz);
        ax = tx;
        ay = ty;
        az = tz;
    }
    else
    {
        ax = tx;
        ay = ty;
        az = tz;
        normalizeV3(ax);
        normalizeV3(ay);
        normalizeV3(az);
    }

    // Colinearity quick-out. It is not an optimization: without it the error
    // test below reads zero for colinear input and reports convergence.
    if (isCloseV3(ax, ay, eps) || isCloseV3(ax, az, eps) || isCloseV3(ay, az, eps))
        return false;

    const int MAX_ITERS = 20;
    int iter;
    for (iter = 0; iter < MAX_ITERS; ++iter)
    {
        bx = tx;
        by = ty;
        bz = tz;

        // Each projection uses the value updated by the previous line.
        bx = sub(bx, scaled(ay, dot(ay, bx)));
        bx = sub(bx, scaled(az, dot(az, bx)));

        by = sub(by, scaled(ax, dot(ax, by)));
        by = sub(by, scaled(az, dot(az, by)));

        bz = sub(bz, scaled(ax, dot(ax, bz)));
        bz = sub(bz, scaled(ay, dot(ay, bz)));

        cx = scaled(add(tx, bx), 0.5);
        cy = scaled(add(ty, by), 0.5);
        cz = scaled(add(tz, bz), 0.5);

        if (normalize)
        {
            normalizeV3(cx);
            normalizeV3(cy);
            normalizeV3(cz);
        }

        const V3 xDiff = sub(tx, cx);
        const V3 yDiff = sub(ty, cy);
        const V3 zDiff = sub(tz, cz);

        const double error = dot(xDiff, xDiff) + dot(yDiff, yDiff) + dot(zDiff, zDiff);

        // error is squared, so compare against the squared tolerance
        if (error < eps * eps)
            break;

        tx = cx;
        ty = cy;
        tz = cz;

        ax = tx;
        ay = ty;
        az = tz;

        if (!normalize)
        {
            normalizeV3(ax);
            normalizeV3(ay);
            normalizeV3(az);
        }
    }

    return iter < MAX_ITERS;
}

// --- GfMatrix4d ------------------------------------------------------------

// GfMatrix4d::operator*= (matrix4d.cpp:568), i.e. Gf's `a * b`.
M4 mul(const M4& a, const M4& b)
{
    M4 o;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            o.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j] + a.m[i][3] * b.m[3][j];
    return o;
}

M4 transpose(const M4& a)
{
    M4 o;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            o.m[i][j] = a.m[j][i];
    return o;
}

// GfMatrix4d::SetDiagonal(double) (matrix4d.cpp:167)
M4 diagonal(double s)
{
    M4 o;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            o.m[i][j] = (i == j) ? s : 0.0;
    return o;
}

// GfMatrix4d::_GetDeterminant3(0,1,2,0,1,2) (matrix4d.cpp:419). The term order
// is pxr's; GetHandedness() relies on it being computed exactly this way.
double determinant3(const M4& a)
{
    return (a.m[0][0] * a.m[1][1] * a.m[2][2] + a.m[0][1] * a.m[1][2] * a.m[2][0] +
            a.m[0][2] * a.m[1][0] * a.m[2][1] - a.m[0][0] * a.m[1][2] * a.m[2][1] -
            a.m[0][1] * a.m[1][0] * a.m[2][2] - a.m[0][2] * a.m[1][1] * a.m[2][0]);
}

// GfMatrix4d::SetScale(const GfVec3d&) (matrix4d.cpp:777)
M4 setScaleM(const V3& s)
{
    M4 o;
    o.m[0][0] = s.v[0]; o.m[0][1] = 0.0;    o.m[0][2] = 0.0;    o.m[0][3] = 0.0;
    o.m[1][0] = 0.0;    o.m[1][1] = s.v[1]; o.m[1][2] = 0.0;    o.m[1][3] = 0.0;
    o.m[2][0] = 0.0;    o.m[2][1] = 0.0;    o.m[2][2] = s.v[2]; o.m[2][3] = 0.0;
    o.m[3][0] = 0.0;    o.m[3][1] = 0.0;    o.m[3][2] = 0.0;    o.m[3][3] = 1.0;
    return o;
}

// GfMatrix4d::SetScale(double) (matrix4d.cpp:657)
M4 setScaleM(double s)
{
    M4 o;
    o.m[0][0] = s;   o.m[0][1] = 0.0; o.m[0][2] = 0.0; o.m[0][3] = 0.0;
    o.m[1][0] = 0.0; o.m[1][1] = s;   o.m[1][2] = 0.0; o.m[1][3] = 0.0;
    o.m[2][0] = 0.0; o.m[2][1] = 0.0; o.m[2][2] = s;   o.m[2][3] = 0.0;
    o.m[3][0] = 0.0; o.m[3][1] = 0.0; o.m[3][2] = 0.0; o.m[3][3] = 1.0;
    return o;
}

// GfMatrix4d::SetTranslate (matrix4d.cpp:788)
M4 setTranslateM(const V3& t)
{
    M4 o;
    o.m[0][0] = 1.0;    o.m[0][1] = 0.0;    o.m[0][2] = 0.0;    o.m[0][3] = 0.0;
    o.m[1][0] = 0.0;    o.m[1][1] = 1.0;    o.m[1][2] = 0.0;    o.m[1][3] = 0.0;
    o.m[2][0] = 0.0;    o.m[2][1] = 0.0;    o.m[2][2] = 1.0;    o.m[2][3] = 0.0;
    o.m[3][0] = t.v[0]; o.m[3][1] = t.v[1]; o.m[3][2] = t.v[2]; o.m[3][3] = 1.0;
    return o;
}

// GfMatrix4d::_SetRotateFromQuat (matrix4d.cpp:668)
void setRotateFromQuat(M4& o, double r, const V3& i)
{
    o.m[0][0] = 1.0 - 2.0 * (i.v[1] * i.v[1] + i.v[2] * i.v[2]);
    o.m[0][1] =       2.0 * (i.v[0] * i.v[1] + i.v[2] * r);
    o.m[0][2] =       2.0 * (i.v[2] * i.v[0] - i.v[1] * r);

    o.m[1][0] =       2.0 * (i.v[0] * i.v[1] - i.v[2] * r);
    o.m[1][1] = 1.0 - 2.0 * (i.v[2] * i.v[2] + i.v[0] * i.v[0]);
    o.m[1][2] =       2.0 * (i.v[1] * i.v[2] + i.v[0] * r);

    o.m[2][0] =       2.0 * (i.v[2] * i.v[0] + i.v[1] * r);
    o.m[2][1] =       2.0 * (i.v[1] * i.v[2] - i.v[0] * r);
    o.m[2][2] = 1.0 - 2.0 * (i.v[1] * i.v[1] + i.v[0] * i.v[0]);
}

// GfMatrix4d::GetInverse (matrix4d.cpp:304). A cofactor expansion, kept
// variable-for-variable so the accumulation order survives.
M4 inverseM(const M4& a, double eps)
{
    double x00, x01, x02, x03;
    double x10, x11, x12, x13;
    double x20, x21, x22, x23;
    double x30, x31, x32, x33;
    double y01, y02, y03, y12, y13, y23;
    double z00, z10, z20, z30;
    double z01, z11, z21, z31;
    double z02, z03, z12, z13, z22, z23, z32, z33;

    x00 = a.m[0][0];
    x01 = a.m[0][1];
    x10 = a.m[1][0];
    x11 = a.m[1][1];
    x20 = a.m[2][0];
    x21 = a.m[2][1];
    x30 = a.m[3][0];
    x31 = a.m[3][1];

    y01 = x00 * x11 - x10 * x01;
    y02 = x00 * x21 - x20 * x01;
    y03 = x00 * x31 - x30 * x01;
    y12 = x10 * x21 - x20 * x11;
    y13 = x10 * x31 - x30 * x11;
    y23 = x20 * x31 - x30 * x21;

    x02 = a.m[0][2];
    x03 = a.m[0][3];
    x12 = a.m[1][2];
    x13 = a.m[1][3];
    x22 = a.m[2][2];
    x23 = a.m[2][3];
    x32 = a.m[3][2];
    x33 = a.m[3][3];

    z33 = x02 * y12 - x12 * y02 + x22 * y01;
    z23 = x12 * y03 - x32 * y01 - x02 * y13;
    z13 = x02 * y23 - x22 * y03 + x32 * y02;
    z03 = x22 * y13 - x32 * y12 - x12 * y23;
    z32 = x13 * y02 - x23 * y01 - x03 * y12;
    z22 = x03 * y13 - x13 * y03 + x33 * y01;
    z12 = x23 * y03 - x33 * y02 - x03 * y23;
    z02 = x13 * y23 - x23 * y13 + x33 * y12;

    y01 = x02 * x13 - x12 * x03;
    y02 = x02 * x23 - x22 * x03;
    y03 = x02 * x33 - x32 * x03;
    y12 = x12 * x23 - x22 * x13;
    y13 = x12 * x33 - x32 * x13;
    y23 = x22 * x33 - x32 * x23;

    z30 = x11 * y02 - x21 * y01 - x01 * y12;
    z20 = x01 * y13 - x11 * y03 + x31 * y01;
    z10 = x21 * y03 - x31 * y02 - x01 * y23;
    z00 = x11 * y23 - x21 * y13 + x31 * y12;
    z31 = x00 * y12 - x10 * y02 + x20 * y01;
    z21 = x10 * y03 - x30 * y01 - x00 * y13;
    z11 = x00 * y23 - x20 * y03 + x30 * y02;
    z01 = x20 * y13 - x30 * y12 - x10 * y23;

    const double det = x30 * z30 + x20 * z20 + x10 * z10 + x00 * z00;

    M4 o;
    if (std::fabs(det) > eps)
    {
        const double rcp = 1.0 / det;
        o.m[0][0] = z00 * rcp;
        o.m[0][1] = z10 * rcp;
        o.m[1][0] = z01 * rcp;
        o.m[0][2] = z20 * rcp;
        o.m[2][0] = z02 * rcp;
        o.m[0][3] = z30 * rcp;
        o.m[3][0] = z03 * rcp;
        o.m[1][1] = z11 * rcp;
        o.m[1][2] = z21 * rcp;
        o.m[2][1] = z12 * rcp;
        o.m[1][3] = z31 * rcp;
        o.m[3][1] = z13 * rcp;
        o.m[2][2] = z22 * rcp;
        o.m[2][3] = z32 * rcp;
        o.m[3][2] = z23 * rcp;
        o.m[3][3] = z33 * rcp;
    }
    else
    {
        // Gf's singular fallback really is a uniform scale by FLT_MAX, widened
        // to double. Not identity, not NaN.
        o = setScaleM(double(FLT_MAX));
    }
    return o;
}

// GfMatrix4d::_Jacobi3 (matrix4d.cpp:921). UNSORTED cyclic Jacobi: the frame it
// returns for a degenerate (near-identity) stretch is decided by rounding, and
// reproducing that exact frame is the entire point of this file.
void jacobi3(const M4& self, V3& eigenvalues, V3 eigenvectors[3])
{
    eigenvalues = V3{ { self.m[0][0], self.m[1][1], self.m[2][2] } };
    eigenvectors[0] = V3{ { 1.0, 0.0, 0.0 } };
    eigenvectors[1] = V3{ { 0.0, 1.0, 0.0 } };
    eigenvectors[2] = V3{ { 0.0, 0.0, 1.0 } };

    M4 a = self;
    V3 b = eigenvalues;
    V3 z{ { 0.0, 0.0, 0.0 } };

    for (int i = 0; i < 50; i++)
    {
        double sm = 0.0;
        for (int p = 0; p < 2; p++)
            for (int q = p + 1; q < 3; q++)
                sm += std::fabs(a.m[p][q]);

        if (sm == 0.0)
            return;

        const double thresh = (i < 3 ? (.2 * sm / (3 * 3)) : 0.0);

        for (int p = 0; p < 3; p++)
        {
            for (int q = p + 1; q < 3; q++)
            {
                double g = 100.0 * std::fabs(a.m[p][q]);

                // The `x + g == x` tests are deliberate floating-point
                // underflow probes, not comparisons that can be simplified.
                if (i > 3 && (std::fabs(eigenvalues.v[p]) + g == std::fabs(eigenvalues.v[p])) &&
                    (std::fabs(eigenvalues.v[q]) + g == std::fabs(eigenvalues.v[q])))
                {
                    a.m[p][q] = 0.0;
                }
                else if (std::fabs(a.m[p][q]) > thresh)
                {
                    double h = eigenvalues.v[q] - eigenvalues.v[p];
                    double t;

                    if (std::fabs(h) + g == std::fabs(h))
                    {
                        t = a.m[p][q] / h;
                    }
                    else
                    {
                        const double theta = 0.5 * h / a.m[p][q];
                        t = 1.0 / (std::fabs(theta) + std::sqrt(1.0 + theta * theta));
                        if (theta < 0.0)
                            t = -t;
                    }

                    const double c = 1.0 / std::sqrt(1.0 + t * t);
                    const double s = t * c;
                    const double tau = s / (1.0 + c);
                    h = t * a.m[p][q];
                    z.v[p] -= h;
                    z.v[q] += h;
                    eigenvalues.v[p] -= h;
                    eigenvalues.v[q] += h;
                    a.m[p][q] = 0.0;

                    for (int j = 0; j < p; j++)
                    {
                        g = a.m[j][p];
                        h = a.m[j][q];
                        a.m[j][p] = g - s * (h + g * tau);
                        a.m[j][q] = h + s * (g - h * tau);
                    }

                    for (int j = p + 1; j < q; j++)
                    {
                        g = a.m[p][j];
                        h = a.m[j][q];
                        a.m[p][j] = g - s * (h + g * tau);
                        a.m[j][q] = h + s * (g - h * tau);
                    }

                    for (int j = q + 1; j < 3; j++)
                    {
                        g = a.m[p][j];
                        h = a.m[q][j];
                        a.m[p][j] = g - s * (h + g * tau);
                        a.m[q][j] = h + s * (g - h * tau);
                    }

                    for (int j = 0; j < 3; j++)
                    {
                        g = eigenvectors[j].v[p];
                        h = eigenvectors[j].v[q];
                        eigenvectors[j].v[p] = g - s * (h + g * tau);
                        eigenvectors[j].v[q] = h + s * (g - h * tau);
                    }
                }
            }
        }
        for (int p = 0; p < 3; p++)
        {
            eigenvalues.v[p] = b.v[p] += z.v[p];
            z.v[p] = 0;
        }
    }
}

// GfMatrix4d::Factor (matrix4d.cpp:864)
bool factorM(const M4& self, M4& r, V3& s, M4& u, V3& t, M4& p, double eps)
{
    p = diagonal(1.0);

    M4 a;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
            a.m[i][j] = self.m[i][j];
        a.m[3][i] = a.m[i][3] = 0.0;
        t.v[i] = self.m[3][i];
    }
    a.m[3][3] = 1.0;

    const double det = determinant3(a);
    const double detSign = (det < 0.0 ? -1.0 : 1.0);
    const bool isSingular = det * detSign < eps;

    const M4 b = mul(a, transpose(a));
    V3 eigenvalues;
    V3 eigenvectors[3];
    jacobi3(b, eigenvalues, eigenvectors);

    // Rows of `r` are the eigenvectors, in the order Jacobi produced them --
    // unsorted, unsigned, unnormalized beyond what the sweeps did.
    r.m[0][0] = eigenvectors[0].v[0]; r.m[0][1] = eigenvectors[0].v[1]; r.m[0][2] = eigenvectors[0].v[2]; r.m[0][3] = 0.0;
    r.m[1][0] = eigenvectors[1].v[0]; r.m[1][1] = eigenvectors[1].v[1]; r.m[1][2] = eigenvectors[1].v[2]; r.m[1][3] = 0.0;
    r.m[2][0] = eigenvectors[2].v[0]; r.m[2][1] = eigenvectors[2].v[1]; r.m[2][2] = eigenvectors[2].v[2]; r.m[2][3] = 0.0;
    r.m[3][0] = 0.0; r.m[3][1] = 0.0; r.m[3][2] = 0.0; r.m[3][3] = 1.0;

    M4 sInv = diagonal(1.0);
    for (int i = 0; i < 3; i++)
    {
        if (eigenvalues.v[i] < eps)
            s.v[i] = detSign * eps;
        else
            s.v[i] = detSign * std::sqrt(eigenvalues.v[i]);
        sInv.m[i][i] = 1.0 / s.v[i];
    }

    // U = R S^-1 R^T A, associated left to right exactly as Gf writes it.
    u = mul(mul(mul(r, sInv), transpose(r)), a);

    return !isSingular;
}

// GfMatrix4d::Orthonormalize (matrix4d.cpp:450), issueWarning dropped.
bool orthonormalizeM(M4& a)
{
    V3 r0{ { a.m[0][0], a.m[0][1], a.m[0][2] } };
    V3 r1{ { a.m[1][0], a.m[1][1], a.m[1][2] } };
    V3 r2{ { a.m[2][0], a.m[2][1], a.m[2][2] } };
    const bool result = orthogonalizeBasis(r0, r1, r2, true);
    a.m[0][0] = r0.v[0]; a.m[0][1] = r0.v[1]; a.m[0][2] = r0.v[2];
    a.m[1][0] = r1.v[0]; a.m[1][1] = r1.v[1]; a.m[1][2] = r1.v[2];
    a.m[2][0] = r2.v[0]; a.m[2][1] = r2.v[1]; a.m[2][2] = r2.v[2];

    // Divide out a homogeneous coordinate -- unless it is zero.
    if (a.m[3][3] != 1.0 && !isCloseD(a.m[3][3], 0.0, kMinVectorLength))
    {
        a.m[3][0] /= a.m[3][3];
        a.m[3][1] /= a.m[3][3];
        a.m[3][2] /= a.m[3][3];
        a.m[3][3] = 1.0;
    }

    return result;
}

// GfMatrix4d::ExtractRotationQuat (matrix4d.cpp:1040)
Q extractRotationQuatM(const M4& a)
{
    int i;
    if (a.m[0][0] > a.m[1][1])
        i = (a.m[0][0] > a.m[2][2] ? 0 : 2);
    else
        i = (a.m[1][1] > a.m[2][2] ? 1 : 2);

    V3 im{ { 0.0, 0.0, 0.0 } };
    double r;

    if (a.m[0][0] + a.m[1][1] + a.m[2][2] > a.m[i][i])
    {
        r = 0.5 * std::sqrt(a.m[0][0] + a.m[1][1] + a.m[2][2] + a.m[3][3]);
        im = V3{ { (a.m[1][2] - a.m[2][1]) / (4.0 * r), (a.m[2][0] - a.m[0][2]) / (4.0 * r),
                   (a.m[0][1] - a.m[1][0]) / (4.0 * r) } };
    }
    else
    {
        const int j = (i + 1) % 3;
        const int k = (i + 2) % 3;
        const double q = 0.5 * std::sqrt(a.m[i][i] - a.m[j][j] - a.m[k][k] + a.m[3][3]);

        im.v[i] = q;
        im.v[j] = (a.m[i][j] + a.m[j][i]) / (4 * q);
        im.v[k] = (a.m[k][i] + a.m[i][k]) / (4 * q);
        r = (a.m[j][k] - a.m[k][j]) / (4 * q);
    }

    return Q{ clampD(r, -1.0, 1.0), im };
}

// --- GfRotation ------------------------------------------------------------

// GfRotation::SetAxisAngle (rotation.h:70) -- note the normalize is CONDITIONAL
// on the axis not already being unit length to within 1e-10.
Rotation setAxisAngle(const V3& axis, double angle)
{
    Rotation out;
    V3 a = axis;
    if (!isCloseD(dot(a, a), 1.0, 1e-10))
        normalizeV3(a);
    out.axis = toPx(a);
    out.angle = angle;
    return out;
}

Rotation identityRot()
{
    Rotation out;
    out.axis = PxVec3d(1.0, 0.0, 0.0);
    out.angle = 0.0;
    return out;
}

// GfRotation::SetQuat (rotation.cpp:29), taking a GfQuatd.
Rotation setQuat(const Q& quat)
{
    const double len = length(quat.i);
    if (len > kMinVectorLength)
    {
        const double x = std::acos(clampD(quat.r, -1.0, 1.0));
        return setAxisAngle(divided(quat.i, len), 2.0 * radiansToDegrees(x));
    }
    return identityRot();
}

// GfRotation::GetQuat (rotation.cpp:76). The final GetNormalized is GfQuatd's,
// which divides the real part and multiplies the imaginary part by 1/length.
Q getQuatR(const Rotation& r)
{
    const double radians = degreesToRadians(r.angle) / 2.0;
    double sinR, cosR;
    sinCos(radians, &sinR, &cosR);
    const V3 axis = scaled(toV3(r.axis), sinR);

    Q q{ cosR, axis };
    // GfQuatd::_GetLengthSquared -> GfDot(GfQuatd, GfQuatd): IMAGINARY FIRST.
    const double lenSq = dot(q.i, q.i) + q.r * q.r;
    const double len = std::sqrt(lenSq);
    if (len < kMinVectorLength)
        return Q{ 1.0, V3{ { 0.0, 0.0, 0.0 } } };
    // GfQuatd::operator/=: real divides, imaginary goes through GfVec3d's
    // reciprocal multiply. The asymmetry is real and is preserved.
    q.r /= len;
    q.i = divided(q.i, len);
    return q;
}

// GfQuaternion::operator*= (quaternion.cpp:62)
Q quaternionMul(const Q& lhs, const Q& rhs)
{
    const double r1 = lhs.r;
    const double r2 = rhs.r;
    const V3& i1 = lhs.i;
    const V3& i2 = rhs.i;

    const double r = r1 * r2 - dot(i1, i2);
    const V3 i{ { r1 * i2.v[0] + r2 * i1.v[0] + (i1.v[1] * i2.v[2] - i1.v[2] * i2.v[1]),
                  r1 * i2.v[1] + r2 * i1.v[1] + (i1.v[2] * i2.v[0] - i1.v[0] * i2.v[2]),
                  r1 * i2.v[2] + r2 * i1.v[2] + (i1.v[0] * i2.v[1] - i1.v[1] * i2.v[0]) } };
    return Q{ r, i };
}

// GfQuaternion::GetNormalized (quaternion.cpp:32). NOTE this is the GfQuaternion
// spelling, not GfQuatd's: length-squared sums REAL FIRST, and the division is a
// reciprocal multiply on both parts.
Q quaternionNormalized(const Q& q, double eps = kMinVectorLength)
{
    const double lenSq = q.r * q.r + dot(q.i, q.i);
    const double len = std::sqrt(lenSq);
    if (len < eps)
        return Q{ 1.0, V3{ { 0.0, 0.0, 0.0 } } };
    const double inv = 1.0 / len;
    return Q{ q.r * inv, scaled(q.i, inv) };
}

// GfRotation::operator*= (rotation.cpp:629), i.e. Gf's `a * b`.
Rotation rotationMul(const Rotation& a, const Rotation& b)
{
    Rotation out = a;
    // Gf converts both to GfQuaternion and multiplies REVERSED: (b_q * a_q).
    const Q q = quaternionNormalized(quaternionMul(getQuatR(b), getQuatR(a)));

    const double len = length(q.i);
    if (len > kMinVectorLength)
    {
        out.axis = toPx(divided(q.i, len));
        // No clamp here, unlike SetQuat -- Gf passes GetReal() to acos raw.
        out.angle = 2.0 * radiansToDegrees(std::acos(q.r));
    }
    else
    {
        // Axis is left as it was; only the angle is zeroed.
        out.angle = 0.0;
    }
    return out;
}

// GfMatrix4d::SetRotate(const GfRotation&) (matrix4d.cpp:708)
M4 setRotateM(const Rotation& r)
{
    // SetRotateOnly goes through GfRotation::GetQuaternion(), which is GetQuat()
    // re-wrapped -- same numbers, so GetQuat() directly is exact here.
    const Q q = getQuatR(r);
    M4 o;
    setRotateFromQuat(o, q.r, q.i);
    o.m[0][3] = 0.0;
    o.m[1][3] = 0.0;
    o.m[2][3] = 0.0;
    o.m[3][0] = 0.0;
    o.m[3][1] = 0.0;
    o.m[3][2] = 0.0;
    o.m[3][3] = 1.0;
    return o;
}

inline bool equalsV3(const V3& a, double x, double y, double z)
{
    return a.v[0] == x && a.v[1] == y && a.v[2] == z;
}

} // anonymous namespace

// --- public surface --------------------------------------------------------

PxMat44d multiply(const PxMat44d& a, const PxMat44d& b)
{
    return toPx(mul(toM4(a), toM4(b)));
}

PxMat44d inverse(const PxMat44d& m, double eps)
{
    return toPx(inverseM(toM4(m), eps));
}

bool orthonormalize(PxMat44d& m)
{
    M4 a = toM4(m);
    const bool result = orthonormalizeM(a);
    m = toPx(a);
    return result;
}

bool factor(const PxMat44d& m, PxMat44d* r, PxVec3d* s, PxMat44d* u, PxVec3d* t, PxMat44d* p, double eps)
{
    M4 rm, um, pm;
    V3 sv, tv;
    const bool nonSingular = factorM(toM4(m), rm, sv, um, tv, pm, eps);
    if (r)
        *r = toPx(rm);
    if (s)
        *s = toPx(sv);
    if (u)
        *u = toPx(um);
    if (t)
        *t = toPx(tv);
    if (p)
        *p = toPx(pm);
    return nonSingular;
}

// GfMatrix4d::RemoveScaleShear (matrix4d.cpp:1023)
PxMat44d removeScaleShearGf(const PxMat44d& m)
{
    const M4 self = toM4(m);
    M4 scaleOrientMat, factoredRotMat, perspMat;
    V3 scale, translation;
    if (!factorM(self, scaleOrientMat, scale, factoredRotMat, translation, perspMat, 1e-10))
    {
        // Unable to decompose: Gf returns the matrix unchanged.
        return m;
    }

    orthonormalizeM(factoredRotMat);
    return toPx(mul(factoredRotMat, setTranslateM(translation)));
}

PxVec3d transformPoint(const PxMat44d& m, const PxVec3d& v)
{
    const M4 a = toM4(m);
    // GfMatrix4d::Transform (matrix4d.h:642) -> GfProject (homogeneous.h:55),
    // which multiplies by the reciprocal of w and passes w == 0 through as 1.
    const double c0 = v.x * a.m[0][0] + v.y * a.m[1][0] + v.z * a.m[2][0] + a.m[3][0];
    const double c1 = v.x * a.m[0][1] + v.y * a.m[1][1] + v.z * a.m[2][1] + a.m[3][1];
    const double c2 = v.x * a.m[0][2] + v.y * a.m[1][2] + v.z * a.m[2][2] + a.m[3][2];
    const double c3 = v.x * a.m[0][3] + v.y * a.m[1][3] + v.z * a.m[2][3] + a.m[3][3];
    const double inv = (c3 != 0.0) ? 1.0 / c3 : 1.0;
    return PxVec3d(inv * c0, inv * c1, inv * c2);
}

Rotation extractRotation(const PxMat44d& m)
{
    return setQuat(extractRotationQuatM(toM4(m)));
}

PxMat44d setScale(const PxVec3d& s)
{
    return toPx(setScaleM(toV3(s)));
}

PxMat44d setScale(double s)
{
    return toPx(setScaleM(s));
}

PxMat44d setTranslate(const PxVec3d& t)
{
    return toPx(setTranslateM(toV3(t)));
}

PxMat44d setRotate(const Rotation& r)
{
    return toPx(setRotateM(r));
}

PxMat44d setTransform(const Rotation& r, const PxVec3d& t)
{
    // GfMatrix4d(rotate, translate) -> SetTransform -> SetRotate then
    // SetTranslateOnly, which only touches the bottom row.
    M4 o = setRotateM(r);
    o.m[3][0] = t.x;
    o.m[3][1] = t.y;
    o.m[3][2] = t.z;
    o.m[3][3] = 1.0;
    return toPx(o);
}

Rotation makeRotation(const PxVec3d& axis, double angleInDegrees)
{
    return setAxisAngle(toV3(axis), angleInDegrees);
}

Rotation identityRotation()
{
    return identityRot();
}

Rotation inverse(const Rotation& r)
{
    // GfRotation::GetInverse (rotation.h:124) -- GfRotation(_axis, -_angle),
    // which re-runs SetAxisAngle and therefore may re-normalize the axis.
    return setAxisAngle(toV3(r.axis), -r.angle);
}

Rotation multiply(const Rotation& a, const Rotation& b)
{
    return rotationMul(a, b);
}

PxQuatd getQuat(const Rotation& r)
{
    const Q q = getQuatR(r);
    return PxQuatd(q.i.v[0], q.i.v[1], q.i.v[2], q.r);
}

double normalize(PxVec3d& v)
{
    V3 a = toV3(v);
    const double len = normalizeV3(a);
    v = toPx(a);
    return len;
}

PxVec3d getNormalized(const PxVec3d& v)
{
    PxVec3d out = v;
    normalize(out);
    return out;
}

// --- GfRange3d (range3d.h, header-inline upstream) -------------------------

Range3d emptyRange()
{
    // GfRange3d::SetEmpty (range3d.h:58) seeds from FLT_MAX, not DBL_MAX, even
    // though the range is double. Widening FLT_MAX to double is exact, so the
    // seed really is 3.4028234663852886e+38.
    Range3d r;
    r.min = PxVec3d(double(FLT_MAX), double(FLT_MAX), double(FLT_MAX));
    r.max = PxVec3d(-double(FLT_MAX), -double(FLT_MAX), -double(FLT_MAX));
    return r;
}

void unionWith(Range3d& r, const PxVec3d& p)
{
    // GfRange3d::_FindMin then _FindMax (range3d.h:331/338), x then y then z.
    if (p.x < r.min.x)
        r.min.x = p.x;
    if (p.y < r.min.y)
        r.min.y = p.y;
    if (p.z < r.min.z)
        r.min.z = p.z;
    if (p.x > r.max.x)
        r.max.x = p.x;
    if (p.y > r.max.y)
        r.max.y = p.y;
    if (p.z > r.max.z)
        r.max.z = p.z;
}

PxVec3d getSize(const Range3d& r)
{
    return PxVec3d(r.max.x - r.min.x, r.max.y - r.min.y, r.max.z - r.min.z);
}

PxVec3d getMidpoint(const Range3d& r)
{
    // 0.5*min + 0.5*max, NOT (min+max)*0.5. The two round differently and this
    // value ends up in a hashed cooking transform.
    return PxVec3d(0.5 * r.min.x + 0.5 * r.max.x, 0.5 * r.min.y + 0.5 * r.max.y, 0.5 * r.min.z + 0.5 * r.max.z);
}

PivotTransform makePivotTransform(const PxVec3d& translation,
                                  const Rotation& rotation,
                                  const PxVec3d& scale,
                                  const PxVec3d& pivotPosition,
                                  const Rotation& pivotOrientation)
{
    PivotTransform out;
    out.scale = scale;
    out.pivotOrientation = pivotOrientation;
    out.rotation = rotation;
    out.pivotPosition = pivotPosition;
    out.translation = translation;
    return out;
}

// GfTransform::SetMatrix (transform.cpp:41), on a transform whose other fields
// are the identity ones GfTransform's matrix constructor starts from.
PivotTransform decomposeWithPivot(const PxMat44d& m, const PxVec3d& pivotPosition)
{
    PivotTransform out;
    out.pivotPosition = pivotPosition;

    const V3 pivot = toV3(pivotPosition);
    const M4 mPivotPos = setTranslateM(pivot);
    const M4 mPivotPosInv = setTranslateM(V3{ { -pivot.v[0], -pivot.v[1], -pivot.v[2] } });
    // Left-to-right: (mPivotPos * m) * mPivotPosInv.
    const M4 mNoPivot = mul(mul(mPivotPos, toM4(m)), mPivotPosInv);

    M4 shearRotMat, rotMat, projMat;
    V3 scale, translation;
    // Factor's return value is deliberately ignored: a singular matrix (a zero
    // scale, say) still produces a usable transform and Gf uses it regardless.
    factorM(mNoPivot, shearRotMat, scale, rotMat, translation, projMat, 1e-10);
    out.scale = toPx(scale);
    out.translation = toPx(translation);

    out.rotation = setQuat(extractRotationQuatM(rotMat));

    // Exact equality against (1,1,1), not a tolerance. A scale that differs in
    // the last bit takes the other branch, and that is the behaviour being
    // reproduced -- the identity pivot orientation is a different cache key
    // from a numerically-tiny one.
    if (!equalsV3(scale, 1.0, 1.0, 1.0))
        out.pivotOrientation = setQuat(extractRotationQuatM(transpose(shearRotMat)));
    else
        out.pivotOrientation = identityRot();

    return out;
}

// GfTransform::GetMatrix (transform.cpp:89), including the accumulation skips.
PxMat44d composeWithPivot(const PivotTransform& t)
{
    const V3 pivot = toV3(t.pivotPosition);
    const V3 scale = toV3(t.scale);
    const V3 translation = toV3(t.translation);

    const bool doPivot = !equalsV3(pivot, 0.0, 0.0, 0.0);
    const bool doScale = !equalsV3(scale, 1.0, 1.0, 1.0);
    const bool doScaleOrient = (t.pivotOrientation.angle != 0.0);
    const bool doRotation = (t.rotation.angle != 0.0);
    const bool doTranslation = !equalsV3(translation, 0.0, 0.0, 0.0);
    bool anySet = false;
    M4 mtx = diagonal(1.0);

    // Gf post-multiplies in application order, so `accum` appends on the right.
    auto accum = [&](const M4& tmp) {
        if (anySet)
            mtx = mul(mtx, tmp);
        else
        {
            mtx = tmp;
            anySet = true;
        }
    };

    if (doPivot)
        accum(setTranslateM(V3{ { -pivot.v[0], -pivot.v[1], -pivot.v[2] } }));

    if (doScale)
    {
        if (doScaleOrient)
            accum(setRotateM(inverse(t.pivotOrientation)));

        accum(setScaleM(scale));

        if (doScaleOrient)
            accum(setRotateM(t.pivotOrientation));
    }

    if (doRotation)
        accum(setRotateM(t.rotation));

    if (doPivot)
        accum(setTranslateM(pivot));

    if (doTranslation)
        accum(setTranslateM(translation));

    if (!anySet)
        mtx = diagonal(1.0);

    return toPx(mtx);
}

} // namespace gfmath
// SPDX-SnippetEnd

} // namespace physx
} // namespace omni
