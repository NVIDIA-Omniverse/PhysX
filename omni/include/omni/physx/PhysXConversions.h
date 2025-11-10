// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

// Utility functions for carbon/physX conversions

#include <pxr/pxr.h>
#include <PxPhysicsAPI.h>

namespace omni
{
namespace physx
{

inline ::physx::PxVec3 toPhysX(const pxr::GfVec3d& usd)
{
    return ::physx::PxVec3(float(usd[0]), float(usd[1]), float(usd[2]));
}

inline ::physx::PxVec3 toPhysX(const pxr::GfVec3f& usd)
{
    return ::physx::PxVec3(usd[0], usd[1], usd[2]);
}

inline const ::physx::PxVec3& toPhysX(const carb::Float3& v)
{
    return (const ::physx::PxVec3&)v;
}

inline const carb::Float3& fromPhysX(const ::physx::PxVec3& v)
{
    return (const carb::Float3&)v;
}

inline ::physx::PxVec3 toPhysX(const carb::Double3& v)
{
    return ::physx::PxVec3(float(v.x), float(v.y), float(v.z));
}

inline ::physx::PxQuat toPhysX(const carb::Float4& v)
{
    return ::physx::PxQuat(v.x, v.y, v.z, v.w);
}

inline carb::Float4 fromPhysX(const ::physx::PxQuat& v)
{
    return carb::Float4{ v.x, v.y, v.z, v.w };
}

inline ::physx::PxQuat toPhysX(const carb::Double4& v)
{
    return ::physx::PxQuat(float(v.x), float(v.y), float(v.z), float(v.w));
}


inline ::physx::PxQuat toPhysX(const pxr::GfQuatd& v)
{
    return ::physx::PxQuat(
        float(v.GetImaginary()[0]), float(v.GetImaginary()[1]), float(v.GetImaginary()[2]), float(v.GetReal()));
}

inline ::physx::PxTransform toPhysX(const pxr::GfTransform& usd)
{
    const pxr::GfVec3d& usdPos = usd.GetTranslation();
    const pxr::GfQuatd usdRot = usd.GetRotation().GetQuat();

    return ::physx::PxTransform(::physx::PxVec3((float)usdPos[0], (float)usdPos[1], (float)usdPos[2]),
                                ::physx::PxQuat((float)usdRot.GetImaginary()[0], (float)usdRot.GetImaginary()[1],
                                                (float)usdRot.GetImaginary()[2], (float)usdRot.GetReal()));
}

inline ::physx::PxTransform toPhysX(const carb::Float3& pos, const carb::Float4& rot)
{
    return ::physx::PxTransform(toPhysX(pos), toPhysX(rot));
}

inline ::physx::PxTransform toPhysX(const carb::Double3& pos, const carb::Double4& rot)
{
    return ::physx::PxTransform(toPhysX(pos), toPhysX(rot));
}

inline void toPhysX(::physx::PxTransform& pxtransform, ::physx::PxVec3& pxscale, const pxr::GfMatrix4d& transform)
{
    pxtransform.p = toPhysX(transform.ExtractTranslation());
    pxtransform.q = toPhysX(transform.RemoveScaleShear().ExtractRotationQuat());
    pxscale = toPhysX(pxr::GfTransform(transform).GetScale());
}

inline ::physx::PxArticulationDrive toPhysX(const omni::physx::usdparser::PhysxJointDrive& drive)
{
    if (drive.isEnvelopeUsed)
        return ::physx::PxArticulationDrive(drive.stiffness, drive.damping, ::physx::PxPerformanceEnvelope(drive.forceLimit, drive.maxActuatorVelocity, drive.velocityDependentResistance, drive.speedEffortGradient),
            drive.acceleration ? ::physx::PxArticulationDriveType::eACCELERATION :
                                ::physx::PxArticulationDriveType::eFORCE);
    return ::physx::PxArticulationDrive(drive.stiffness, drive.damping, drive.forceLimit,
                                        drive.acceleration ? ::physx::PxArticulationDriveType::eACCELERATION :
                                                             ::physx::PxArticulationDriveType::eFORCE);
}

inline float degToRad(const float a)
{
    return 0.01745329251994329547f * a;
}

inline float radToDeg(const float a)
{
    return 57.29577951308232286465f * a;
}

inline pxr::GfVec3f degToRad(const pxr::GfVec3f& a)
{
    return pxr::GfVec3f(0.01745329251994329547f * a);
}

inline pxr::GfVec3f radToDeg(const pxr::GfVec3f& a)
{
    return pxr::GfVec3f(57.29577951308232286465f * a);
}

inline ::physx::PxVec3 radToDeg(const ::physx::PxVec3& a)
{
    return ::physx::PxVec3(57.29577951308232286465f * a);
}

inline void copyBuffer(std::vector<carb::Float4>& dst, ::physx::PxVec4* src, unsigned int numElements)
{
    const carb::Float4* srcValues = (const carb::Float4*)src;
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; ++i)
    {
        dst[i] = srcValues[i];
    }
}

inline void copyBuffer(std::vector<carb::Float3>& dst, ::physx::PxVec4* src, unsigned int numElements)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; ++i)
    {
        const ::physx::PxVec4& srcValue = src[i];
        dst[i] = { srcValue.x, srcValue.y, srcValue.z };
    }
}

inline void copyBuffer(::physx::PxVec4* dst, const carb::Float3* src, unsigned int numElements)
{
    for (unsigned int i = 0; i < numElements; ++i)
    {
        ::physx::PxVec4& dstValue = dst[i];
        const carb::Float3& srcValue = src[i];
        dstValue = { srcValue.x, srcValue.y, srcValue.z, dstValue.w };
    }
}

inline void copyBuffer(::physx::PxVec4* dst,
                       const carb::Float3* src,
                       unsigned int numElements,
                       const pxr::GfMatrix4f& transform)
{
    for (unsigned int i = 0; i < numElements; ++i)
    {
        const carb::Float3& srcValue = src[i];
        const pxr::GfVec3f& srcValueTransformed = transform.Transform(pxr::GfVec3f(srcValue.x, srcValue.y, srcValue.z));
        ::physx::PxVec4& dstValue = dst[i];
        dstValue = { srcValueTransformed[0], srcValueTransformed[1], srcValueTransformed[2], dstValue.w };
    }
}

template <typename SrcVecT>
void copyBuffer(pxr::VtArray<pxr::GfVec3f>& dst, const SrcVecT* src, unsigned int numElements)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; i++)
    {
        const SrcVecT& srcValue = src[i];
        dst[i] = pxr::GfVec3f(srcValue.x, srcValue.y, srcValue.z);
    }
}

template <typename SrcVecT>
void copyBuffer(pxr::VtArray<pxr::GfVec3f>& dst,
                const SrcVecT* src,
                unsigned int numElements,
                const pxr::GfMatrix4f& transform)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; i++)
    {
        const SrcVecT& srcValue = src[i];
        dst[i] = transform.Transform(pxr::GfVec3f(srcValue.x, srcValue.y, srcValue.z));
    }
}

template <typename SrcVecT>
void copyBuffer(pxr::VtArray<pxr::GfVec4f>& dst, const SrcVecT* src, unsigned int numElements)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; i++)
    {
        const SrcVecT& srcValue = src[i];
        dst[i] = pxr::GfVec4f(srcValue.x, srcValue.y, srcValue.z, srcValue.w);
    }
}

template <typename SrcVecT>
void copyBuffer(pxr::VtArray<pxr::GfVec2i>& dst, const SrcVecT* src, unsigned int numElements)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; i++)
    {
        const SrcVecT& srcValue = src[i];
        dst[i] = pxr::GfVec2i(srcValue.x, srcValue.y);
    }
}

inline void copyBuffer(pxr::VtArray<pxr::GfVec3f>& dst, ::physx::PxVec4* src, unsigned int numElements)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; i++)
    {
        const ::physx::PxVec4& srcValue = src[i];
        dst[i] = { srcValue.x, srcValue.y, srcValue.z };
    }
}

inline void copyBuffer(pxr::VtArray<pxr::GfVec3f>& dst,
                       ::physx::PxVec4* src,
                       unsigned int numElements,
                       const pxr::GfMatrix4f& transform)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; i++)
    {
        const ::physx::PxVec4& srcValue = src[i];
        dst[i] = transform.Transform(pxr::GfVec3f(srcValue.x, srcValue.y, srcValue.z));
    }
}

inline void copyBuffer(pxr::VtArray<carb::Float3>& dst, const pxr::GfVec3f* src, unsigned int numElements)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; i++)
    {
        const pxr::GfVec3f& srcValue = src[i];
        dst[i] = { srcValue[0], srcValue[1], srcValue[2] };
    }
}

inline void copyBuffer(float3* dst, const ::physx::PxVec4* src, unsigned int numElements, const pxr::GfMatrix4f& transform)
{
    for (unsigned int i = 0; i < numElements; i++)
    {
        const ::physx::PxVec4& srcValue = src[i];
        pxr::GfVec3f dstValue = transform.Transform(pxr::GfVec3f(srcValue.x, srcValue.y, srcValue.z));
        dst[i] = { dstValue[0], dstValue[1], dstValue[2] };
    }
}

inline void copyBuffer(float3* dst, const ::physx::PxVec3* src, unsigned int numElements, const pxr::GfMatrix4f& transform)
{
    for (unsigned int i = 0; i < numElements; i++)
    {
        const ::physx::PxVec3& srcValue = src[i];
        pxr::GfVec3f dstValue = transform.Transform(pxr::GfVec3f(srcValue.x, srcValue.y, srcValue.z));
        dst[i] = { dstValue[0], dstValue[1], dstValue[2] };
    }
}

inline void copyBuffer(float3* dst,
                       const ::physx::PxVec4* src,
                       unsigned int numElements,
                       const pxr::GfMatrix4f& transform,
                       uint32_t* mapping)
{
    for (unsigned int i = 0; i < numElements; i++)
    {
        const ::physx::PxVec4& srcValue = src[mapping[i]];
        pxr::GfVec3f dstValue = transform.Transform(pxr::GfVec3f(srcValue.x, srcValue.y, srcValue.z));
        dst[i] = { dstValue[0], dstValue[1], dstValue[2] };
    }
}

inline void copyBuffer(float3* dst,
                       const ::physx::PxVec3* src,
                       unsigned int numElements,
                       const pxr::GfMatrix4f& transform,
                       uint32_t* mapping)
{
    for (unsigned int i = 0; i < numElements; i++)
    {
        const ::physx::PxVec3& srcValue = src[mapping[i]];
        pxr::GfVec3f dstValue = transform.Transform(pxr::GfVec3f(srcValue.x, srcValue.y, srcValue.z));
        dst[i] = { dstValue[0], dstValue[1], dstValue[2] };
    }
}

template <typename SrcVecT>
void scatterBuffer(pxr::VtArray<pxr::GfVec3f>& dst, const SrcVecT* src, const pxr::VtArray<uint32_t>& indexMap)
{
    for (size_t i = 0; i < indexMap.size(); i++)
    {
        const SrcVecT& p = src[i];
        uint32_t index = indexMap[i];
        dst[index] = pxr::GfVec3f(p.x, p.y, p.z);
    }
}

template <typename SrcVecT>
void scatterBuffer(pxr::VtArray<pxr::GfQuath>& dst, const SrcVecT* src, const pxr::VtArray<uint32_t>& indexMap)
{
    for (size_t i = 0; i < indexMap.size(); i++)
    {
        const SrcVecT& p = src[i];
        uint32_t index = indexMap[i];
        dst[index] = p;
    }
}

template <typename SrcVecT>
void scatterBuffer(pxr::VtArray<pxr::GfVec3f>& dst,
                   const SrcVecT* src,
                   const pxr::VtArray<uint32_t>& indexMap,
                   const pxr::GfMatrix4f& transform)
{
    for (size_t i = 0; i < indexMap.size(); i++)
    {
        const SrcVecT& p = src[i];
        uint32_t index = indexMap[i];
        dst[index] = transform.Transform(pxr::GfVec3f(p.x, p.y, p.z));
    }
}

template <typename DstT>
void scatterBuffer(pxr::VtArray<DstT>& dst, const DstT& src, const pxr::VtArray<uint32_t>& indexMap)
{
    for (size_t i = 0; i < indexMap.size(); i++)
    {
        uint32_t index = indexMap[i];
        dst[index] = src;
    }
}

} // namespace physx
} // namespace omni
