// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <carb/Types.h>

namespace omni
{
namespace physx
{

// carb to physx
inline ::physx::PxVec2 toPhysX(const carb::Float2& v) { return ::physx::PxVec2{ v.x, v.y}; }

inline ::physx::PxVec2 toPhysX(const carb::Double2& v) { return ::physx::PxVec2{ (float)v.x, (float)v.y}; }

inline ::physx::PxVec3 toPhysX(const carb::Float3& v) { return ::physx::PxVec3{ v.x, v.y, v.z }; }

inline ::physx::PxVec3 toPhysX(const carb::Double3& v) { return ::physx::PxVec3{ (float)v.x, (float)v.y, (float)v.z }; }

inline ::physx::PxVec4 toPhysX(const carb::Float4& v) { return ::physx::PxVec4{ v.x, v.y, v.z, v.w}; }

inline ::physx::PxVec4 toPhysX(const carb::Double4& v) { return ::physx::PxVec4{ (float)v.x, (float)v.y, (float)v.z, (float)v.w }; }

inline ::physx::PxQuat toPhysXQuat(const carb::Float4& v) { return ::physx::PxQuat{ v.x, v.y, v.z, v.w }; }

inline ::physx::PxQuat toPhysXQuat(const carb::Double4& v) { return ::physx::PxQuat{ (float)v.x, (float)v.y, (float)v.z, (float)v.w }; }

// physx to pxr
inline PXR_NS::GfVec2f toVec2f(const ::physx::PxVec2& v) { return PXR_NS::GfVec2f{ v.x, v.y }; }

inline PXR_NS::GfVec2d toVec2d(const ::physx::PxVec2& v) { return PXR_NS::GfVec2d{ (double) v.x, (double) v.y }; }

inline PXR_NS::GfVec3f toVec3f(const ::physx::PxVec3& v) { return PXR_NS::GfVec3f{ v.x, v.y, v.z }; }

inline PXR_NS::GfVec3d toVec3d(const ::physx::PxVec3& v) { return PXR_NS::GfVec3d{ (double) v.x, (double) v.y, (double) v.z }; }

inline PXR_NS::GfVec4f toVec4f(const ::physx::PxVec4& v) { return PXR_NS::GfVec4f{ v.x, v.y, v.z, v.w }; }

inline PXR_NS::GfVec4d toVec4d(const ::physx::PxVec4& v) { return PXR_NS::GfVec4d{ (double) v.x, (double) v.y, (double) v.z, (double) v.w }; }

// physx to carb
inline carb::Float4 toFloat4(const ::physx::PxQuat& q) { return carb::Float4{ q.x, q.y, q.z, q.w }; }

inline carb::Double4 toDouble4(const ::physx::PxQuat& q) { return carb::Double4{ (double) q.x, (double)q.y, (double)q.z, (double)q.w }; }

inline carb::Float4 fromPhysX(const ::physx::PxQuat& q) { return toFloat4(q); }

inline carb::Float2 toFloat2(const ::physx::PxVec2& v) { return carb::Float2{ v.x, v.y }; }

inline carb::Double2 toDouble2(const ::physx::PxVec2& v) { return carb::Double2{ (double) v.x, (double) v.y }; }

inline carb::Float2 fromPhysX(const ::physx::PxVec2& v) { return toFloat2(v); }

inline carb::Float3 toFloat3(const ::physx::PxVec3& v) { return carb::Float3{ v.x, v.y, v.z }; }

inline carb::Double3 toDouble3(const ::physx::PxVec3& v) { return carb::Double3{ (double) v.x, (double) v.y, (double) v.z };  }

inline carb::Float3 fromPhysX(const ::physx::PxVec3& v) { return toFloat3(v); }

inline carb::Float4 toFloat4(const ::physx::PxVec4& v) { return carb::Float4{ v.x, v.y, v.z, v.w }; }

inline carb::Double4 toDouble4(const ::physx::PxVec4& v) { return carb::Double4{ (double) v.x, (double) v.y, (double) v.z, (double) v.w }; }

inline carb::Float4 fromPhysX(const ::physx::PxVec4& v) { return toFloat4(v); }

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

inline const ::physx::PxVec3& asPhysX(const carb::Float3& v) { return (const ::physx::PxVec3&)v; }

inline const carb::Float3& asFloat3(const ::physx::PxVec3& v) { return (const carb::Float3&)v; }

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

inline ::physx::PxTransform toPhysX(const carb::Float3& pos, const carb::Float4& rot)
{
    return ::physx::PxTransform(asPhysX(pos), toPhysXQuat(rot));
}

inline ::physx::PxTransform toPhysX(const carb::Double3& pos, const carb::Double4& rot)
{
    return ::physx::PxTransform(toPhysX(pos), toPhysXQuat(rot));
}

inline void toPhysX(::physx::PxTransform& pxtransform, ::physx::PxVec3& pxscale, const PXR_NS::GfMatrix4d& transform)
{
    pxtransform.p = toPhysX(transform.ExtractTranslation());
    pxtransform.q = toPhysX(transform.RemoveScaleShear().ExtractRotationQuat());
    pxscale = toPhysX(PXR_NS::GfTransform(transform).GetScale());
}

}
}
