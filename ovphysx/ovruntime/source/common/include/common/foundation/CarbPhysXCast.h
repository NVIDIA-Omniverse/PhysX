// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <carb/Types.h>

// Relies on the includer having already brought in PhysX's foundation/math headers
// (PxVec2/PxVec3/PxVec4/PxQuat/PxTransform, typically via PxPhysicsAPI.h), matching
// TypeCast.h's own convention -- neither header includes PhysX itself.
//
// pxr-free half of TypeCast.h: the carb <-> PhysX conversions that never name a Gf type, split out
// so USD-free consumers (e.g. omni.physx.cooking, ADR-0018) can convert without dragging in
// TypeCast.h's pxr/base/gf includes. TypeCast.h includes this header too, so existing direct
// includers of TypeCast.h are unaffected -- the Gf-typed overloads of the same function names
// (toPhysX(GfVec3f), toFloat3(GfVec3d), etc.) still live there, alongside these.

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

inline const ::physx::PxVec3& asPhysX(const carb::Float3& v) { return (const ::physx::PxVec3&)v; }

inline const carb::Float3& asFloat3(const ::physx::PxVec3& v) { return (const carb::Float3&)v; }

inline ::physx::PxTransform toPhysX(const carb::Float3& pos, const carb::Float4& rot)
{
    return ::physx::PxTransform(asPhysX(pos), toPhysXQuat(rot));
}

inline ::physx::PxTransform toPhysX(const carb::Double3& pos, const carb::Double4& rot)
{
    return ::physx::PxTransform(toPhysX(pos), toPhysXQuat(rot));
}

// Widening conversions. PxVec3T<double> has no converting constructor from
// PxVec3T<float>, and the toPhysX overloads above all land on the float PxVec3,
// so transforming float mesh points by a double world matrix needs these --
// otherwise the composition silently drops to float.

inline ::physx::PxVec3d toPhysXd(const carb::Float3& v)
{
    return ::physx::PxVec3d(double(v.x), double(v.y), double(v.z));
}

inline ::physx::PxVec3d toPhysXd(const ::physx::PxVec3& v)
{
    return ::physx::PxVec3d(double(v.x), double(v.y), double(v.z));
}

inline ::physx::PxVec3 toPhysXf(const ::physx::PxVec3d& v)
{
    return ::physx::PxVec3(float(v.x), float(v.y), float(v.z));
}

inline carb::Float3 toFloat3(const ::physx::PxVec3d& v)
{
    return carb::Float3{ float(v.x), float(v.y), float(v.z) };
}

}
}
