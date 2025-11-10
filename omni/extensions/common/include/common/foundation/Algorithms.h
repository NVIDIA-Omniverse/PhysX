// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "TypeCast.h"


#if defined(__SSE__) || defined(_M_AMD64) || defined(_M_X64)
#    if !defined(OMNI_PHYSX_OPTIMIZE_SSE)
#        define OMNI_PHYSX_OPTIMIZE_SSE 1
#    endif
#endif


namespace omni
{
namespace physx
{

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

inline void copyBuffer(std::vector<carb::Float3>& dst, const pxr::GfVec3f* src, unsigned int numElements)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; ++i)
    {
        const pxr::GfVec3f& srcValue = src[i];
        dst[i] = { srcValue[0], srcValue[1], srcValue[2] };
    }
}

inline void copyBuffer(std::vector<::physx::PxVec3>& dst, const pxr::GfVec3f* src, unsigned int numElements)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; ++i)
    {
        const pxr::GfVec3f& srcValue = src[i];
        dst[i] = ::physx::PxVec3(srcValue[0], srcValue[1], srcValue[2]);
    }
}

inline void copyBuffer(std::vector<carb::Float4>& dst, const pxr::GfVec3f* src, unsigned int numElements, ::physx::PxVec3 scale)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; ++i)
    {
        ::physx::PxVec3 srcValue = toPhysX(src[i]).multiply(scale);
        dst[i] = { srcValue.x, srcValue.y, srcValue.z, 0.0f };
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

inline void copyBuffer(::physx::PxVec4* dst, const carb::Float3* src, unsigned int numElements, const pxr::GfMatrix4f& transform)
{
    for (unsigned int i = 0; i < numElements; ++i)
    {
        const carb::Float3& srcValue = src[i];
        const pxr::GfVec3f& srcValueTransformed = transform.Transform(pxr::GfVec3f(srcValue.x, srcValue.y, srcValue.z));
        ::physx::PxVec4& dstValue = dst[i];
        dstValue = { srcValueTransformed[0], srcValueTransformed[1], srcValueTransformed[2], dstValue.w };
    }
}

inline void copyBuffer(::physx::PxVec3* dst, const carb::Float3* src, unsigned int numElements, const pxr::GfMatrix4f& transform)
{
    for (unsigned int i = 0; i < numElements; ++i)
    {
        const carb::Float3& srcValue = src[i];
        const pxr::GfVec3f& srcValueTransformed = transform.Transform(pxr::GfVec3f(srcValue.x, srcValue.y, srcValue.z));
        ::physx::PxVec3& dstValue = dst[i];
        dstValue = { srcValueTransformed[0], srcValueTransformed[1], srcValueTransformed[2] };
    }
}

template<typename SrcVecT>
void copyBuffer(pxr::VtArray<pxr::GfVec3f>& dst, const SrcVecT* src, unsigned int numElements)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; i++)
    {
        const SrcVecT& srcValue = src[i];
        dst[i] = pxr::GfVec3f(srcValue.x, srcValue.y, srcValue.z);
    }
}

template<typename SrcVecT>
void copyBuffer(pxr::VtArray<pxr::GfVec3f>& dst, const SrcVecT* src, unsigned int numElements, const pxr::GfMatrix4f& transform)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; i++)
    {
        const SrcVecT& srcValue = src[i];
        dst[i] = transform.Transform(pxr::GfVec3f(srcValue.x, srcValue.y, srcValue.z));
    }
}

template<typename SrcVecT>
void copyBuffer(pxr::VtArray<pxr::GfVec3f>& dst, const SrcVecT* src, unsigned int numElements, const std::vector<uint32_t>& mapping)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < dst.size(); i++)
        dst[i] = pxr::GfVec3f(0.0f);
    for (unsigned int i = 0; i < numElements; i++)
    {
        const SrcVecT& srcValue = src[mapping[i]];
        dst[i] = pxr::GfVec3f(srcValue.x, srcValue.y, srcValue.z);
    }
}

template<typename SrcVecT>
void copyBuffer(pxr::VtArray<pxr::GfVec3f>& dst, const SrcVecT* src, unsigned int numElements, const pxr::GfMatrix4f& transform, const std::vector<uint32_t>& mapping)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < dst.size(); i++)
        dst[i] = pxr::GfVec3f(0.0f);
    for (unsigned int i = 0; i < numElements; i++)
    {
        const SrcVecT& srcValue = src[mapping[i]];
        dst[i] = transform.Transform(pxr::GfVec3f(srcValue.x, srcValue.y, srcValue.z));
    }
}

#if OMNI_PHYSX_OPTIMIZE_SSE
inline bool isAligned16(const void* a)
{
    return (0 == (size_t(a) & 0x0f));
}

template<>
inline void copyBuffer<::physx::PxVec4>(pxr::VtArray<pxr::GfVec3f>& dst,
                                        const ::physx::PxVec4* src,
                                        unsigned int numElements,
                                        const pxr::GfMatrix4f& transform)
{
    if (!(isAligned16(dst.data()) && isAligned16(src) && isAligned16(transform.data())))
    {
        dst.resize(numElements);
        for (unsigned int i = 0; i < numElements; i++)
        {
            const ::physx::PxVec4& srcValue = src[i];
            dst[i] = transform.Transform(pxr::GfVec3f(srcValue.x, srcValue.y, srcValue.z));
        }
        return;
    }

    auto kNoInit = [](...) {};
    dst.resize(numElements, kNoInit);
    auto Transform = [](__m128 const v, __m128 const l0, __m128 const l1, __m128 const l2, __m128 const l3) {
        __m128 r0 = _mm_mul_ps(_mm_shuffle_ps(v, v, _MM_SHUFFLE(0, 0, 0, 0)), l0);
        __m128 r1 = _mm_mul_ps(_mm_shuffle_ps(v, v, _MM_SHUFFLE(1, 1, 1, 1)), l1);
        __m128 r2 = _mm_mul_ps(_mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 2, 2, 2)), l2);
        __m128 r = _mm_add_ps(_mm_add_ps(r2, _mm_add_ps(r1, r0)), l3);

        // Normalize
        __m128 z = _mm_shuffle_ps(r, r, _MM_SHUFFLE(3, 3, 3, 3));
        __m128 eqz = _mm_cmpeq_ps(z, _mm_set1_ps(0));
#   if defined(__AVX__) || defined(__SSE4_1__)
        __m128 nrm = _mm_blendv_ps(_mm_mul_ps(r, _mm_rcp_ps(z)), _mm_set1_ps(1.0f), eqz); // z == 0 ? 1.0f : r / z
#   else
        __m128 nrm = _mm_or_ps(
                        _mm_and_ps(eqz, _mm_set1_ps(1.0f)),
                        _mm_andnot_ps(eqz, _mm_div_ps(r, z)));
#   endif
        return nrm;
    };

    const float* mat = transform.data();
    const __m128 l0 = _mm_load_ps(mat + 0 * 4);
    const __m128 l1 = _mm_load_ps(mat + 1 * 4);
    const __m128 l2 = _mm_load_ps(mat + 2 * 4);
    const __m128 l3 = _mm_load_ps(mat + 3 * 4);

    pxr::GfVec3f* vdest = dst.data();
    for (unsigned int i = numElements / 4; i != 0; --i)
    {
        auto s0 = Transform(_mm_load_ps(reinterpret_cast<const float*>(src + 0)), l0, l1, l2, l3);
        auto s1 = Transform(_mm_load_ps(reinterpret_cast<const float*>(src + 1)), l0, l1, l2, l3);
        auto s2 = Transform(_mm_load_ps(reinterpret_cast<const float*>(src + 2)), l0, l1, l2, l3);
        auto s3 = Transform(_mm_load_ps(reinterpret_cast<const float*>(src + 3)), l0, l1, l2, l3);


#    if defined(__AVX__) || defined(__SSE4_1__)
        auto d0 = _mm_insert_ps(s0, s1, 0x30); // s0.x s0.y s0.z s1.x
#    else
        auto d0 = _mm_shuffle_ps(
                    s0,
                    _mm_shuffle_ps(s0, s1, _MM_SHUFFLE(0, 1, 2, 3)), // s0.w s0.z s1.y s1.x
                    _MM_SHUFFLE(3, 1, 1, 0)); // s0.x s0.y s0.z s1.x
#    endif
        auto d1 = _mm_shuffle_ps(s1, s2, _MM_SHUFFLE(1, 0, 2, 1));
        auto d2 = _mm_shuffle_ps(_mm_shuffle_ps(s2, s3, 2), s3, _MM_SHUFFLE(2, 1, 2, 0));

        _mm_store_ps(reinterpret_cast<float*>(vdest) + 0 * 4, d0);
        _mm_store_ps(reinterpret_cast<float*>(vdest) + 1 * 4, d1);
        _mm_store_ps(reinterpret_cast<float*>(vdest) + 2 * 4, d2);
        vdest += 4;
        src += 4;
    }
    for (unsigned int i = numElements & 3; i != 0; i--)
    {
        const ::physx::PxVec4& srcValue = *src++;
        *vdest++ = transform.Transform(pxr::GfVec3f(srcValue.x, srcValue.y, srcValue.z));
    }
}
#endif // OMNI_PHYSX_OPTIMIZE_SSE


template<typename SrcVecT>
void copyBuffer(pxr::VtArray<pxr::GfVec4f>& dst, const SrcVecT* src, unsigned int numElements)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; i++)
    {
        const SrcVecT& srcValue = src[i];
        dst[i] = pxr::GfVec4f(srcValue.x, srcValue.y, srcValue.z, srcValue.w);
    }
}

template<typename SrcVecT>
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

inline void copyBuffer(pxr::VtArray<pxr::GfVec3f>& dst, ::physx::PxVec4* src, unsigned int numElements, const pxr::GfMatrix4f& transform)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; i++)
    {
        const ::physx::PxVec4& srcValue = src[i];
        dst[i] = transform.Transform(pxr::GfVec3f(srcValue.x, srcValue.y, srcValue.z));
    }
}

template <typename SrcIndexT>
void copyBuffer(pxr::VtArray<pxr::GfVec3i>& dst, const SrcIndexT* src, unsigned int numSrcElements)
{
    PX_COMPILE_TIME_ASSERT(3 * sizeof(SrcIndexT) == sizeof(pxr::GfVec3i));
    uint32_t numDstElements = numSrcElements / 3;
    dst.resize(numDstElements);
    std::memcpy(dst.data(), src, numDstElements * sizeof(pxr::GfVec3i));
}

template<typename SrcIndexT>
void copyBuffer(pxr::VtArray<pxr::GfVec4i>& dst, const SrcIndexT* src, unsigned int numSrcElements)
{
    PX_COMPILE_TIME_ASSERT(4 * sizeof(SrcIndexT) == sizeof(pxr::GfVec4i));
    uint32_t numDstElements = numSrcElements / 4;
    dst.resize(numDstElements);
    std::memcpy(dst.data(), src, numDstElements * sizeof(pxr::GfVec4i));
}

template<typename SrcVecT>
void scatterBuffer(pxr::VtArray<pxr::GfVec3f>& dst, const SrcVecT* src, const pxr::VtArray<uint32_t>& indexMap)
{
    for (size_t i = 0; i < indexMap.size(); i++)
    {
        const SrcVecT& p = src[i];
        uint32_t index = indexMap[i];
        dst[index] = pxr::GfVec3f(p.x, p.y, p.z);
    }
}

template<typename SrcVecT>
void scatterBuffer(pxr::VtArray<pxr::GfQuath>& dst, const SrcVecT* src, const pxr::VtArray<uint32_t>& indexMap)
{
    for (size_t i = 0; i < indexMap.size(); i++)
    {
        const SrcVecT& p = src[i];
        uint32_t index = indexMap[i];
        dst[index] = p;
    }
}

template<typename SrcVecT>
void scatterBuffer(pxr::VtArray<pxr::GfVec3f>& dst, const SrcVecT* src, const pxr::VtArray<uint32_t>& indexMap, const pxr::GfMatrix4f& transform)
{
    for (size_t i = 0; i < indexMap.size(); i++)
    {
        const SrcVecT& p = src[i];
        uint32_t index = indexMap[i];
        dst[index] = transform.Transform(pxr::GfVec3f(p.x, p.y, p.z));
    }
}

template<typename DstT>
void scatterBuffer(pxr::VtArray<DstT>& dst, const DstT& src, const pxr::VtArray<uint32_t>& indexMap)
{
    for (size_t i = 0; i < indexMap.size(); i++)
    {
        uint32_t index = indexMap[i];
        dst[index] = src;
    }
}


}
}
