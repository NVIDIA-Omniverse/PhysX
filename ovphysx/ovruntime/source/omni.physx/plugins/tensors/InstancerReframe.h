// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @implements REQ-READ-INSTANCER-001
 * @covers AC-2, AC-3, AC-4
 */

// The point-instancer reframe -- world pose to instancer-local instance-array space -- in a form both
// the host view and the CUDA kernel compile, so the CPU and GPU reads are one implementation and
// cannot drift. Both reads and the write path (InternalScene::updateRigidBodyTransforms) share ONE
// rotation factorisation; see instancerPolarRotation. TestInstancerReframe pins this against the
// expression the write path composes, so the device math is verified on the host, without a GPU.
//
// No pxr: this is included from a .cu and USD headers do not build under nvcc. PhysX foundation math
// is PX_CUDA_CALLABLE throughout, including operator*, and ADR-0001 section 8 puts internal math on it.

#if defined(__CUDACC__)
#    define OVX_INSTANCER_HD __host__ __device__
#else
#    define OVX_INSTANCER_HD
#endif

#include <math.h>

#include <foundation/PxMat44.h>
#include <foundation/PxVec4.h>

namespace omni
{
namespace physx
{
namespace tensors
{

// PhysX's COLUMN-VECTOR convention (v' = M * v), the same one PxMat44d carries everywhere else in the
// runtime. Converting from USD's row-vector convention is not a transpose: the two hold the SAME
// sixteen doubles for the same transform, and what differs is the ORDER OF PRODUCTS -- Gf `A * B` is
// PhysX `B * A`. See common/foundation/MatrixTools.h.
using InstancerAffine = ::physx::PxMat44T<double>;

// Double throughout: world-to-instancer is where a far-from-origin instancer loses precision -- large
// world coordinates in, a small local one out, so float composition rounds away the digits the
// subtraction is about to need. Inputs and outputs stay float; only the composition is widened, so
// PxMat44 (float) here would silently undo it.
//
// `out = b * a` and not `a * b`: this composes what the Gf form wrote as `a * b`, and the operands
// reverse between the conventions.
OVX_INSTANCER_HD inline void instancerComposeAffine(const InstancerAffine& a,
                                                    const InstancerAffine& b,
                                                    InstancerAffine& out)
{
    out = b * a;
}

// Rigid transform (quaternion xyzw + translation) to the affine above.
//
// Built element-wise in double rather than through PxMat44d(PxTransform): that constructor takes a
// float PxTransform, which would throw away the precision this whole header exists to keep.
OVX_INSTANCER_HD inline void instancerAffineFromPose(const double* q, const double* p, InstancerAffine& out)
{
    const double xx = q[0] * q[0], yy = q[1] * q[1], zz = q[2] * q[2];
    const double xy = q[0] * q[1], xz = q[0] * q[2], yz = q[1] * q[2];
    const double wx = q[3] * q[0], wy = q[3] * q[1], wz = q[3] * q[2];

    // Columns are the basis vectors and the translation.
    out.column0 = ::physx::PxVec4T<double>(1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz), 2.0 * (xz - wy), 0.0);
    out.column1 = ::physx::PxVec4T<double>(2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx), 0.0);
    out.column2 = ::physx::PxVec4T<double>(2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy), 0.0);
    out.column3 = ::physx::PxVec4T<double>(p[0], p[1], p[2], 1.0);
}

// Quaternion from rotation columns, by the SAME factorisation the write path uses: this is
// common/foundation/MatrixTools.cpp's decomposeMatrix, rotation half only, made PX_CUDA_CALLABLE
// (decomposeMatrix itself is a non-inline free function with no device marking). It is a second copy
// of that math, pinned by TestInstancerReframe's "read vs write path" section, which asserts
// agreement with omni::physx::toTransform() to 1e-6 scaled and unscaled.
//
// Float narrowing is part of the contract: the host builds the quaternion in double, narrows to
// PxQuat (float) and normalises in float. Reproduced step for step, because "agrees to 1e-6" is only
// true if the rounding matches.
OVX_INSTANCER_HD inline void instancerQuatFromColumns(double n00, double n01, double n02,
                                                     double n10, double n11, double n12,
                                                     double n20, double n21, double n22,
                                                     double* outQuat)
{
    double x = 0.0, y = 0.0, z = 0.0, w = 1.0;
    const double trace = n00 + n11 + n22;
    if (trace > 0.0)
    {
        const double sc = sqrt(trace + 1.0) * 2.0;
        w = 0.25 * sc;
        x = (n21 - n12) / sc;
        y = (n02 - n20) / sc;
        z = (n10 - n01) / sc;
    }
    else if (n00 > n11 && n00 > n22)
    {
        const double sc = sqrt(1.0 + n00 - n11 - n22) * 2.0;
        w = (n21 - n12) / sc;
        x = 0.25 * sc;
        y = (n01 + n10) / sc;
        z = (n02 + n20) / sc;
    }
    else if (n11 > n22)
    {
        const double sc = sqrt(1.0 + n11 - n00 - n22) * 2.0;
        w = (n02 - n20) / sc;
        x = (n01 + n10) / sc;
        y = 0.25 * sc;
        z = (n12 + n21) / sc;
    }
    else
    {
        const double sc = sqrt(1.0 + n22 - n00 - n11) * 2.0;
        w = (n10 - n01) / sc;
        x = (n02 + n20) / sc;
        y = (n12 + n21) / sc;
        z = 0.25 * sc;
    }

    // Narrow, then normalise in float -- the host's PxQuat does both.
    float fx = float(x), fy = float(y), fz = float(z), fw = float(w);
    const float mag2 = fx * fx + fy * fy + fz * fz + fw * fw;
    if (mag2 > 1.0e-12f && isfinite(mag2))
    {
        const float inv = 1.0f / sqrtf(mag2);
        fx *= inv; fy *= inv; fz *= inv; fw *= inv;
    }
    else
    {
        fx = 0.0f; fy = 0.0f; fz = 0.0f; fw = 1.0f;
    }
    outQuat[0] = double(fx);
    outQuat[1] = double(fy);
    outQuat[2] = double(fz);
    outQuat[3] = double(fw);
}

// Orthogonal factor of `m`'s 3x3 block as a quaternion: Gram-Schmidt for a seed, then Newton polar
// refinement R <- 0.5 * (R + R^-T), which converges quadratically. A reflection (det < 0) is
// absorbed by negating all three columns, which is the convention GfTransform reports and therefore
// the one MatrixTools reproduces.
OVX_INSTANCER_HD inline void instancerPolarRotation(const InstancerAffine& m, double* outQuat)
{
    // Columns are the basis vectors, as PxMat44d stores them.
    double R[3][3] = {
        { m.column0.x, m.column0.y, m.column0.z },
        { m.column1.x, m.column1.y, m.column1.z },
        { m.column2.x, m.column2.y, m.column2.z },
    };
    const double M[3][3] = { { R[0][0], R[0][1], R[0][2] },
                             { R[1][0], R[1][1], R[1][2] },
                             { R[2][0], R[2][1], R[2][2] } };

    for (int c = 0; c < 3; ++c)
    {
        // Gram-Schmidt: project out the columns already fixed, then normalise.
        for (int prev = 0; prev < c; ++prev)
        {
            const double d = R[prev][0] * R[c][0] + R[prev][1] * R[c][1] + R[prev][2] * R[c][2];
            R[c][0] -= d * R[prev][0];
            R[c][1] -= d * R[prev][1];
            R[c][2] -= d * R[prev][2];
        }
        const double len = sqrt(R[c][0] * R[c][0] + R[c][1] * R[c][1] + R[c][2] * R[c][2]);
        if (len > 1.0e-12 && isfinite(len))
        {
            const double inv = 1.0 / len;
            R[c][0] *= inv; R[c][1] *= inv; R[c][2] *= inv;
        }
        else
        {
            // Degenerate column: fall back to the matching axis, as the host does.
            R[c][0] = (c == 0) ? 1.0 : 0.0;
            R[c][1] = (c == 1) ? 1.0 : 0.0;
            R[c][2] = (c == 2) ? 1.0 : 0.0;
        }
    }

    for (int iter = 0; iter < 16; ++iter)
    {
        const double det = R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
                         - R[1][0] * (R[0][1] * R[2][2] - R[0][2] * R[2][1])
                         + R[2][0] * (R[0][1] * R[1][2] - R[0][2] * R[1][1]);
        if (!isfinite(det) || fabs(det) < 1.0e-20)
            break;
        const double invD = 1.0 / det;

        // Cofactor columns over the determinant are the columns of R^-T.
        const double inv0[3] = { (R[1][1] * R[2][2] - R[1][2] * R[2][1]) * invD,
                                 (R[1][2] * R[2][0] - R[1][0] * R[2][2]) * invD,
                                 (R[1][0] * R[2][1] - R[1][1] * R[2][0]) * invD };
        const double inv1[3] = { (R[2][1] * R[0][2] - R[2][2] * R[0][1]) * invD,
                                 (R[2][2] * R[0][0] - R[2][0] * R[0][2]) * invD,
                                 (R[2][0] * R[0][1] - R[2][1] * R[0][0]) * invD };
        const double inv2[3] = { (R[0][1] * R[1][2] - R[0][2] * R[1][1]) * invD,
                                 (R[0][2] * R[1][0] - R[0][0] * R[1][2]) * invD,
                                 (R[0][0] * R[1][1] - R[0][1] * R[1][0]) * invD };

        double maxDelta = 0.0;
        for (int r = 0; r < 3; ++r)
        {
            const double a0 = 0.5 * (R[0][r] + inv0[r]);
            const double a1 = 0.5 * (R[1][r] + inv1[r]);
            const double a2 = 0.5 * (R[2][r] + inv2[r]);
            const double d0 = fabs(R[0][r] - a0);
            const double d1 = fabs(R[1][r] - a1);
            const double d2 = fabs(R[2][r] - a2);
            if (d0 > maxDelta) maxDelta = d0;
            if (d1 > maxDelta) maxDelta = d1;
            if (d2 > maxDelta) maxDelta = d2;
            R[0][r] = a0; R[1][r] = a1; R[2][r] = a2;
        }
        if (maxDelta < 1.0e-12)
            break;
    }

    const double det = R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
                     - R[1][0] * (R[0][1] * R[2][2] - R[0][2] * R[2][1])
                     + R[2][0] * (R[0][1] * R[1][2] - R[0][2] * R[1][1]);
    if (det < 0.0)
    {
        for (int c = 0; c < 3; ++c)
        {
            R[c][0] = -R[c][0]; R[c][1] = -R[c][1]; R[c][2] = -R[c][2];
        }
    }
    (void)M; // the scale half of the host decomposition; the reframe wants rotation only

    instancerQuatFromColumns(R[0][0], R[1][0], R[2][0],
                             R[0][1], R[1][1], R[2][1],
                             R[0][2], R[1][2], R[2][2], outQuat);
}

// The Gf expression `protoInverse * worldPose * instancerWorldInverse` that
// InternalScene::updateRigidBodyTransforms composes, written in PhysX order, where the operands
// reverse to `instancerWorldInverse * worldPose * protoInverse`. Composition only: a caller reading
// the POSITION column takes three reads off the result and skips the rotation's double sqrt and
// divides.
OVX_INSTANCER_HD inline void instancerComposeLocal(const InstancerAffine& protoInverse,
                                                   const InstancerAffine& instancerWorldInverse,
                                                   const double* worldQuat,
                                                   const double* worldPos,
                                                   InstancerAffine& outLocal)
{
    InstancerAffine world;
    instancerAffineFromPose(worldQuat, worldPos, world);

    outLocal = instancerWorldInverse * world * protoInverse;
}

OVX_INSTANCER_HD inline void instancerReframe(const InstancerAffine& protoInverse,
                                              const InstancerAffine& instancerWorldInverse,
                                              const double* worldQuat,
                                              const double* worldPos,
                                              double* outPos,
                                              double* outQuat)
{
    InstancerAffine local;
    instancerComposeLocal(protoInverse, instancerWorldInverse, worldQuat, worldPos, local);

    outPos[0] = local.column3.x;
    outPos[1] = local.column3.y;
    outPos[2] = local.column3.z;

    instancerPolarRotation(local, outQuat);
}

// The INVERSE of instancerReframe: instancer-local instance-array space back to a world pose
// (ADR-0012). What the ovstage WRITE needs, where the read needs the forward direction.
//
// The algebra, with the same composition operator instancerComposeAffine implements:
//
//     read     local = protoInverse . world . instancerWorldInverse
//     write    world = proto . local . instancerWorld
//
// which follows by composing protoInverse away on the left and instancerWorldInverse on the right.
// It takes the FORWARD transforms where the read takes inverses, so a caller holding
// InternalInstance::mProtoTransformInverse has to invert it back -- there is no forward copy stored.
//
// Kept in this header beside the forward direction, not written out at the call site, for the reason
// the forward one is here: the CPU and GPU write paths must agree bit-for-bit on what an instance's
// world pose IS, and two transcriptions of this expression is the shape where they drift with
// nothing to notice.
OVX_INSTANCER_HD inline void instancerUnreframe(const InstancerAffine& proto,
                                                const InstancerAffine& instancerWorld,
                                                const double* localQuat,
                                                const double* localPos,
                                                double* outPos,
                                                double* outQuat)
{
    InstancerAffine local;
    instancerAffineFromPose(localQuat, localPos, local);
    InstancerAffine protoTimesLocal;
    instancerComposeAffine(proto, local, protoTimesLocal);
    InstancerAffine world;
    instancerComposeAffine(protoTimesLocal, instancerWorld, world);

    outPos[0] = world.column3.x;
    outPos[1] = world.column3.y;
    outPos[2] = world.column3.z;

    instancerPolarRotation(world, outQuat);
}

} // namespace tensors
} // namespace physx
} // namespace omni

// Scoped to this header: every use is above, and leaving it defined would put a bare OVX_* macro in
// every translation unit that includes GpuSimulationData.h.
#undef OVX_INSTANCER_HD
