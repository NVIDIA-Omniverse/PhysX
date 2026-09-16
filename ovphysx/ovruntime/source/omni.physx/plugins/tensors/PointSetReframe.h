// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-COVERAGE-001
 * @covers AC-4c
 *
 * @implements REQ-READ-DEVICE-001
 * @covers AC-2
 */
#pragma once

// The point-set reframe -- a world position into the owning prim's frame, sim-mesh-local for a
// deformable and prim-local for a particle set -- in a form both the host and the CUDA kernel
// compile, so the two backends cannot drift.
//
// Reproduces PxMat44T<Type>::transform(PxVec3) exactly: COLUMN-major storage, PhysX's COLUMN-vector
// convention (v' = M*v), no divide -- the source matrices (InternalDeformableBody::mWorldToSimMesh,
// InternalParticleSet::mWorldToLocal) are affine by construction. The matrix stays double until the
// host/device boundary; only the store here narrows.
//
// No pxr and no PhysX includes: this header is pulled into a .cu.

#if defined(__CUDACC__)
#    define OVX_POINTSET_HD __host__ __device__
#else
#    define OVX_POINTSET_HD
#endif

namespace omni
{
namespace physx
{
namespace tensors
{

// A 4x4 in PxMat44T's COLUMN-major storage and PhysX's COLUMN-VECTOR convention (v' = M*v),
// narrowed to float. m[c * 4 + r] is column c, row r -- PxMat44T::front()'s own layout.
struct PointSetTransform
{
    float m[16] = {};
};

// v' = column0*x + column1*y + column2*z + column3 -- PxMat44T<Type>::transform(PxVec3), affine, no divide.
OVX_POINTSET_HD inline void pointSetTransformPoint(const PointSetTransform& a, const float* p, float* out)
{
    const float x = p[0], y = p[1], z = p[2];
    out[0] = x * a.m[0] + y * a.m[4] + z * a.m[8] + a.m[12];
    out[1] = x * a.m[1] + y * a.m[5] + z * a.m[9] + a.m[13];
    out[2] = x * a.m[2] + y * a.m[6] + z * a.m[10] + a.m[14];
}

} // namespace tensors
} // namespace physx
} // namespace omni

// Scoped to this header: every use is above, and leaving it defined would put a bare OVX_* macro in
// every translation unit that includes this one.
#undef OVX_POINTSET_HD
