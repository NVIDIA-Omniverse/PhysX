// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <carb/Types.h>

#include <common/foundation/CarbPhysXCast.h>

#include <pxr/base/gf/half.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/matrix4f.h>
#include <pxr/base/gf/quatd.h>
#include <pxr/base/gf/quaternion.h>
#include <pxr/base/gf/quatf.h>
#include <pxr/base/gf/quath.h>
#include <pxr/base/gf/transform.h>
#include <pxr/base/gf/vec2d.h>
#include <pxr/base/gf/vec2f.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/vec3h.h>
#include <pxr/base/gf/vec3i.h>
#include <pxr/base/gf/vec4d.h>
#include <pxr/base/gf/vec4f.h>
#include <pxr/base/gf/vec4i.h>

namespace omni
{
namespace physx
{

// physx to pxr
inline PXR_NS::GfVec2f toVec2f(const ::physx::PxVec2& v) { return PXR_NS::GfVec2f{ v.x, v.y }; }

inline PXR_NS::GfVec2d toVec2d(const ::physx::PxVec2& v) { return PXR_NS::GfVec2d{ (double) v.x, (double) v.y }; }

inline PXR_NS::GfVec3f toVec3f(const ::physx::PxVec3& v) { return PXR_NS::GfVec3f{ v.x, v.y, v.z }; }

inline PXR_NS::GfVec3d toVec3d(const ::physx::PxVec3& v) { return PXR_NS::GfVec3d{ (double) v.x, (double) v.y, (double) v.z }; }

inline PXR_NS::GfVec4f toVec4f(const ::physx::PxVec4& v) { return PXR_NS::GfVec4f{ v.x, v.y, v.z, v.w }; }

inline PXR_NS::GfVec4d toVec4d(const ::physx::PxVec4& v) { return PXR_NS::GfVec4d{ (double) v.x, (double) v.y, (double) v.z, (double) v.w }; }

// pxr to carb
inline carb::Float4 toFloat4(const PXR_NS::GfQuaternion& v) { return carb::Float4{ float(v.GetImaginary()[0]), float(v.GetImaginary()[1]), float(v.GetImaginary()[2]), float(v.GetReal()) }; }

inline carb::Double4 toDouble4(const PXR_NS::GfQuaternion& v) { return carb::Double4{ v.GetImaginary()[0], v.GetImaginary()[1], v.GetImaginary()[2], v.GetReal() }; }

inline carb::Float4 toFloat4(const PXR_NS::GfQuatf& v) { return carb::Float4{ v.GetImaginary()[0], v.GetImaginary()[1], v.GetImaginary()[2], v.GetReal() }; }

inline carb::Double4 toDouble4(const PXR_NS::GfQuatf& v) { return carb::Double4{ (double) v.GetImaginary()[0], (double) v.GetImaginary()[1], (double) v.GetImaginary()[2], (double) v.GetReal() }; }

inline carb::Float4 toFloat4(const PXR_NS::GfQuatd& v) { return carb::Float4{ float(v.GetImaginary()[0]), float(v.GetImaginary()[1]), float(v.GetImaginary()[2]), float(v.GetReal()) }; }

inline carb::Double4 toDouble4(const PXR_NS::GfQuatd& v) { return carb::Double4{ v.GetImaginary()[0], v.GetImaginary()[1], v.GetImaginary()[2], v.GetReal() }; }

inline carb::Float2 toFloat2(const PXR_NS::GfVec2f& v) { return carb::Float2{ v[0], v[1] }; }

inline carb::Float2 toFloat2(const PXR_NS::GfVec2d& v) { return carb::Float2{ (float) v[0], (float) v[1] }; }

inline carb::Double2 toDouble2(const PXR_NS::GfVec2d& v) { return carb::Double2{ v[0], v[1] }; }

inline carb::Double2 toDouble2(const PXR_NS::GfVec2f& v) { return carb::Double2{ (double) v[0], (double) v[1] }; }

inline carb::Float3 toFloat3(const PXR_NS::GfVec3f& v) { return carb::Float3{ v[0], v[1], v[2] }; }

inline carb::Float3 toFloat3(const PXR_NS::GfVec3d& v) { return carb::Float3{ (float) v[0], (float) v[1], (float) v[2] }; }

inline carb::Double3 toDouble3(const PXR_NS::GfVec3d& v) { return carb::Double3{ v[0], v[1], v[2] }; }

inline carb::Double3 toDouble3(const PXR_NS::GfVec3f& v) { return carb::Double3{  (double) v[0],  (double) v[1], (double) v[2] }; }

inline carb::Float4 toFloat4(const PXR_NS::GfVec4f& v) { return carb::Float4{ v[0], v[1], v[2], v[3] }; }

inline carb::Float4 toFloat4(const PXR_NS::GfVec4d& v) { return carb::Float4{ (float) v[0], (float) v[1], (float) v[2], (float) v[3] }; }

inline carb::Double4 toDouble4(const PXR_NS::GfVec4d& v) { return carb::Double4{ v[0], v[1], v[2], v[3] }; }

inline carb::Double4 toDouble4(const PXR_NS::GfVec4f& v) { return carb::Double4{ (double) v[0], (double) v[1], (double) v[2], (double) v[3] }; }

// carb to pxr
inline PXR_NS::GfVec2f toVec2f(const carb::Float2& v) { return PXR_NS::GfVec2f{ v.x, v.y}; }

inline PXR_NS::GfVec2f toVec2f(const carb::Double2& v) { return PXR_NS::GfVec2f{ (float) v.x, (float) v.y}; }

inline PXR_NS::GfVec2d toVec2d(const carb::Float2& v) { return PXR_NS::GfVec2d{ (double) v.x,  (double) v.y}; }

inline PXR_NS::GfVec2d toVec2d(const carb::Double2& v) { return PXR_NS::GfVec2d{ v.x, v.y}; }

inline PXR_NS::GfVec3f toVec3f(const carb::Float3& v) { return PXR_NS::GfVec3f{ v.x, v.y, v.z}; }

inline PXR_NS::GfVec3f toVec3f(const carb::Double3& v) { return PXR_NS::GfVec3f{ (float) v.x, (float) v.y, (float) v.z}; }

inline PXR_NS::GfVec3d toVec3d(const carb::Float3& v) { return PXR_NS::GfVec3d{ (double) v.x,  (double) v.y, (double) v.z}; }

inline PXR_NS::GfVec3d toVec3d(const carb::Double3& v) { return PXR_NS::GfVec3d{ v.x, v.y, v.z}; }

inline PXR_NS::GfVec4f toVec4f(const carb::Float4& v) { return PXR_NS::GfVec4f{ v.x, v.y, v.z, v.w}; }

inline PXR_NS::GfVec4f toVec4f(const carb::Double4& v) { return PXR_NS::GfVec4f{ (float) v.x, (float) v.y, (float) v.z, (float) v.w}; }

inline PXR_NS::GfQuatf toQuatf(const carb::Float4& v) { return PXR_NS::GfQuatf{ v.w, PXR_NS::GfVec3f { v.x, v.y, v.z } }; }

inline PXR_NS::GfQuatf toQuatf(const carb::Double4& v) { return PXR_NS::GfQuatf{ (float) v.w, PXR_NS::GfVec3f { (float) v.x, (float) v.y, (float) v.z } }; }

inline PXR_NS::GfVec4d toVec4d(const carb::Float4& v) { return PXR_NS::GfVec4d{ (double) v.x,  (double) v.y, (double) v.z, (double) v.w}; }

inline PXR_NS::GfVec4d toVec4d(const carb::Double4& v) { return PXR_NS::GfVec4d{ v.x, v.y, v.z, v.w}; }

inline PXR_NS::GfQuaternion toQuaternion(const carb::Float4& v) { return PXR_NS::GfQuaternion{ (double) v.w, PXR_NS::GfVec3d { (double) v.x,  (double) v.y, (double) v.z } }; }

inline PXR_NS::GfQuaternion toQuaternion(const carb::Double4& v) { return PXR_NS::GfQuaternion{ v.w, PXR_NS::GfVec3d { v.x, v.y, v.z } }; }

inline PXR_NS::GfQuatd toQuatd(const carb::Float4& v) { return PXR_NS::GfQuatd{ (double) v.w, PXR_NS::GfVec3d { (double) v.x,  (double) v.y, (double) v.z } }; }

inline PXR_NS::GfQuatd toQuatd(const carb::Double4& v) { return PXR_NS::GfQuatd{ v.w, PXR_NS::GfVec3d { v.x, v.y, v.z } }; }

// pxr to physx
inline ::physx::PxQuat toPhysX(const PXR_NS::GfQuatf& q)
{
    return ::physx::PxQuat{ q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2], q.GetReal() };
}

inline ::physx::PxQuat toPhysX(const PXR_NS::GfQuaternion& q)
{
    return ::physx::PxQuat{ float(q.GetImaginary()[0]), float(q.GetImaginary()[1]), float(q.GetImaginary()[2]), float(q.GetReal()) };
}

inline ::physx::PxQuat toPhysX(const PXR_NS::GfQuatd& q)
{
    return ::physx::PxQuat{ float(q.GetImaginary()[0]), float(q.GetImaginary()[1]), float(q.GetImaginary()[2]), float(q.GetReal()) };
}

inline ::physx::PxVec2 toPhysX(const PXR_NS::GfVec2f& v) { return ::physx::PxVec2{ v[0], v[1] }; }

inline ::physx::PxVec2 toPhysX(const PXR_NS::GfVec2d& v) { return ::physx::PxVec2{ (float)v[0], (float)v[1] }; }

inline ::physx::PxVec3 toPhysX(const PXR_NS::GfVec3f& v) { return ::physx::PxVec3{ v[0], v[1], v[2] }; }

inline ::physx::PxVec3 toPhysX(const PXR_NS::GfVec3d& v) { return ::physx::PxVec3{ (float)v[0], (float)v[1], (float)v[2] }; }

inline ::physx::PxVec4 toPhysX(const PXR_NS::GfVec4f& v) { return ::physx::PxVec4{ v[0], v[1], v[2], v[3] }; }

inline ::physx::PxVec4 toPhysX(const PXR_NS::GfVec4d& v) { return ::physx::PxVec4{ (float)v[0], (float)v[1], (float)v[2], (float)v[3] }; }

inline ::physx::PxQuat toPhysXQuat(const PXR_NS::GfVec4f& v) { return ::physx::PxQuat{ v[0], v[1], v[2], v[3] }; }

inline ::physx::PxQuat toPhysXQuat(const PXR_NS::GfVec4d& v) { return ::physx::PxQuat{ (float)v[0], (float)v[1], (float)v[2], (float)v[3] }; }

inline ::physx::PxTransform toPhysX(const PXR_NS::GfTransform& usd)
{
    const PXR_NS::GfVec3d& usdPos = usd.GetTranslation();
    const PXR_NS::GfQuatd usdRot = usd.GetRotation().GetQuat();

    return ::physx::PxTransform(::physx::PxVec3((float)usdPos[0], (float)usdPos[1], (float)usdPos[2]),
                                ::physx::PxQuat((float)usdRot.GetImaginary()[0], (float)usdRot.GetImaginary()[1],
                                                (float)usdRot.GetImaginary()[2], (float)usdRot.GetReal()));
}

inline ::physx::PxTransform toPhysX(const PXR_NS::GfVec3d& pos, const PXR_NS::GfQuatd& rot)
{
    return ::physx::PxTransform(toPhysX(pos), toPhysX(rot));
}

// ---------------------------------------------------------------------------
// Gf <-> PhysX 4x4 bridges, for the shrinking set of sites that still have to
// hand a matrix to USD (xform-op authoring) or take one from it.
//
// These are element copies, NOT transposes. GfMatrix4d and PxMat44d hold the
// same sixteen doubles for the same transform -- USD reads them row-major with
// a row-vector convention (v' = v*M), PhysX column-major with a column-vector
// convention (v' = M*v). What differs is the product order, so a Gf `A * B`
// becomes a PhysX `B * A`. See common/foundation/MatrixTools.h for the rest of
// the mapping and for the operations PhysX does not ship (affine inverse, TRS
// decomposition).
// ---------------------------------------------------------------------------

inline ::physx::PxMat44d toPhysX(const PXR_NS::GfMatrix4d& m)
{
    double values[16];
    for (int i = 0; i < 16; ++i)
        values[i] = m.GetArray()[i];
    return ::physx::PxMat44d(values);
}

inline PXR_NS::GfMatrix4d toMatrix4d(const ::physx::PxMat44d& m)
{
    const double* v = m.front();
    return PXR_NS::GfMatrix4d(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7],
                              v[8], v[9], v[10], v[11], v[12], v[13], v[14], v[15]);
}

inline PXR_NS::GfMatrix4f toMatrix4f(const ::physx::PxMat44d& m)
{
    const double* v = m.front();
    return PXR_NS::GfMatrix4f(float(v[0]), float(v[1]), float(v[2]), float(v[3]),
                              float(v[4]), float(v[5]), float(v[6]), float(v[7]),
                              float(v[8]), float(v[9]), float(v[10]), float(v[11]),
                              float(v[12]), float(v[13]), float(v[14]), float(v[15]));
}

// Widening conversions. PxVec3T<double> has no converting constructor from
// PxVec3T<float>, and the toPhysX overloads above all land on the float PxVec3,
// so transforming float mesh points by a double world matrix needs these --
// otherwise the composition silently drops to float.

inline ::physx::PxVec3d toPhysXd(const PXR_NS::GfVec3f& v)
{
    return ::physx::PxVec3d(double(v[0]), double(v[1]), double(v[2]));
}

inline ::physx::PxVec3d toPhysXd(const PXR_NS::GfVec3d& v)
{
    return ::physx::PxVec3d(v[0], v[1], v[2]);
}

inline PXR_NS::GfVec3f toVec3f(const ::physx::PxVec3d& v)
{
    return PXR_NS::GfVec3f(float(v.x), float(v.y), float(v.z));
}


// ---------------------------------------------------------------------------
// Integer-lane bridges. Mesh/tet topology rides in GfVec3i / GfVec4i on the USD
// side; carb::Int3 / carb::Int4 are the source-neutral equivalents and are
// layout-compatible (3 resp. 4 packed int32), so a bulk `memcpy` between the two
// is legal -- these are for the element-at-a-time sites.
// ---------------------------------------------------------------------------

inline carb::Int3 toInt3(const PXR_NS::GfVec3i& v) { return carb::Int3{ v[0], v[1], v[2] }; }

inline carb::Int4 toInt4(const PXR_NS::GfVec4i& v) { return carb::Int4{ v[0], v[1], v[2], v[3] }; }

inline PXR_NS::GfVec3i toVec3i(const carb::Int3& v) { return PXR_NS::GfVec3i{ v.x, v.y, v.z }; }

inline PXR_NS::GfVec4i toVec4i(const carb::Int4& v) { return PXR_NS::GfVec4i{ v.x, v.y, v.z, v.w }; }

// ---------------------------------------------------------------------------
// Half-precision quaternion bridge (point-instancer / particle orientations).
//
// GfQuath holds FOUR 16-BIT HALVES, so it is NOT layout-compatible with
// carb::Float4 -- a memcpy or a reinterpret_cast between the two is wrong and
// silently produces garbage. Always go through these, which round-trip via
// GetReal()/GetImaginary(). Note the lane order flip: carb::Float4 is xyzw with
// w == the real part, matching the existing toFloat4(GfQuatf) overload.
// ---------------------------------------------------------------------------

inline carb::Float4 toFloat4(const PXR_NS::GfQuath& v)
{
    return carb::Float4{ float(v.GetImaginary()[0]), float(v.GetImaginary()[1]),
                         float(v.GetImaginary()[2]), float(v.GetReal()) };
}

inline PXR_NS::GfQuath toQuath(const carb::Float4& v)
{
    return PXR_NS::GfQuath{ PXR_NS::GfHalf(v.w),
                            PXR_NS::GfVec3h{ PXR_NS::GfHalf(v.x), PXR_NS::GfHalf(v.y), PXR_NS::GfHalf(v.z) } };
}

// (toQuatd(carb::Float4) / toQuatd(carb::Double4) already exist above.)

// PhysX quaternion <-> carb, so a site that drops Gf does not have to route
// through a Gf type to get between the two.

inline ::physx::PxQuat toPhysXQuat(const PXR_NS::GfQuath& v)
{
    return ::physx::PxQuat{ float(v.GetImaginary()[0]), float(v.GetImaginary()[1]),
                            float(v.GetImaginary()[2]), float(v.GetReal()) };
}


}
}
