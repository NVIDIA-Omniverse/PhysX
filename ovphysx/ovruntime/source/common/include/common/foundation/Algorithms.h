// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-MATH-001
 * @covers AC-10
 *
 * @implements REQ-ALGO-001
 * @covers AC-1
 */

#pragma once

// PxVec2/PxVec3/PxVec4/PxQuat/PxTransform/PxMat44 etc. must come before
// "TypeCast.h": both TypeCast.h and CarbPhysXCast.h are documented to rely on
// the includer having already brought in PhysX's foundation/math headers
// (neither includes PhysX itself). Every existing includer of this file
// happened to satisfy that transitively; a TU that includes Algorithms.h
// first (e.g. a test) will not.
#include <foundation/PxAssert.h>
#include <foundation/PxMat44.h>
#include <foundation/PxQuat.h>
#include <foundation/PxTransform.h>
#include <foundation/PxVec2.h>
#include <foundation/PxVec3.h>
#include <foundation/PxVec4.h>

// carb::Float3/Float4 and the toPhysX/toPhysXd carb<->PhysX conversions used
// below arrive from these two. They used to come in transitively via TypeCast.h,
// which was otherwise vestigial here (no PXR_NS:: type appears below). TypeCast.h
// is the pxr-typed PhysX<->Gf bridge and now lives in source/omni.physics.usd/,
// so it is gone from here rather than fenced; CarbPhysXCast.h is its pxr-free
// half and stays. A TU that needs the Gf conversions includes <TypeCast.h>
// directly.
#include <carb/Types.h>

#include "CarbPhysXCast.h"

#include <cstring>
#include <vector>


#if defined(__SSE__) || defined(_M_AMD64) || defined(_M_X64)
#    if !defined(OMNI_PHYSX_OPTIMIZE_SSE)
#        define OMNI_PHYSX_OPTIMIZE_SSE 1
#    endif
#endif

#if OMNI_PHYSX_OPTIMIZE_SSE
// __m128 and the _mm_* intrinsics used below. Previously left to whatever the
// including translation unit happened to drag in transitively -- true for
// every existing includer, but not guaranteed, and not true for a TU (this
// file's own test) that includes Algorithms.h without also happening to
// include something else that pulls in SSE intrinsics.
#    include <xmmintrin.h> // __m128, _mm_load_ps/_mm_store_ps/_mm_shuffle_ps/...
#    if defined(__AVX__) || defined(__SSE4_1__)
#        include <smmintrin.h> // _mm_blendv_ps, _mm_insert_ps
#    endif
#endif

// Buffer copy/scatter helpers. These are source-neutral: the element types are
// carb::Float2/3/4, carb::Int2/3/4 and PhysX vectors, and the transform is a
// ::physx::PxMat44d (ADR-0001 section 8 -- internal math is PhysX foundation
// math, Gf* belongs only at the USD boundary).
//
// TRANSFORM CONVENTION. PxMat44d and GfMatrix4f/GfMatrix4d hold the same
// sixteen values for the same transform; USD reads them row-major with a
// row-vector convention (v' = v*M), PhysX column-major with a column-vector
// convention (v' = M*v). So the Gf `transform.Transform(p)` these helpers used
// to do is spelled `transform.transform(p)` here, with no transpose and no
// element reordering -- see common/foundation/MatrixTools.h for the full
// mapping table. (Note that GfMatrix4f::Transform also divides by the resulting
// w; PxMat44::transform does not. Every matrix fed to these helpers is an
// affine world/local transform whose last column is (0,0,0,1), so w == 1 and
// the divide is a no-op.)

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

inline void copyBuffer(std::vector<::physx::PxVec3>& dst, const carb::Float3* src, unsigned int numElements)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; ++i)
    {
        const carb::Float3& srcValue = src[i];
        dst[i] = ::physx::PxVec3(srcValue.x, srcValue.y, srcValue.z);
    }
}

inline void copyBuffer(std::vector<carb::Float4>& dst, const carb::Float3* src, unsigned int numElements, ::physx::PxVec3 scale)
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

inline void copyBuffer(::physx::PxVec4* dst, const carb::Float3* src, unsigned int numElements, const ::physx::PxMat44d& transform)
{
    for (unsigned int i = 0; i < numElements; ++i)
    {
        const carb::Float3& srcValue = src[i];
        const ::physx::PxVec3d srcValueTransformed = transform.transform(toPhysXd(srcValue));
        ::physx::PxVec4& dstValue = dst[i];
        dstValue = { float(srcValueTransformed.x), float(srcValueTransformed.y), float(srcValueTransformed.z), dstValue.w };
    }
}

inline void copyBuffer(::physx::PxVec3* dst, const carb::Float3* src, unsigned int numElements, const ::physx::PxMat44d& transform)
{
    for (unsigned int i = 0; i < numElements; ++i)
    {
        const carb::Float3& srcValue = src[i];
        const ::physx::PxVec3d srcValueTransformed = transform.transform(toPhysXd(srcValue));
        ::physx::PxVec3& dstValue = dst[i];
        dstValue = { float(srcValueTransformed.x), float(srcValueTransformed.y), float(srcValueTransformed.z) };
    }
}

template<typename SrcVecT>
void copyBuffer(std::vector<carb::Float3>& dst, const SrcVecT* src, unsigned int numElements)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; i++)
    {
        const SrcVecT& srcValue = src[i];
        dst[i] = carb::Float3{ float(srcValue.x), float(srcValue.y), float(srcValue.z) };
    }
}

template<typename SrcVecT>
void copyBuffer(std::vector<carb::Float3>& dst, const SrcVecT* src, unsigned int numElements, const ::physx::PxMat44d& transform)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; i++)
    {
        const SrcVecT& srcValue = src[i];
        const ::physx::PxVec3d p = transform.transform(::physx::PxVec3d(double(srcValue.x), double(srcValue.y), double(srcValue.z)));
        dst[i] = carb::Float3{ float(p.x), float(p.y), float(p.z) };
    }
}

template<typename SrcVecT>
void copyBuffer(std::vector<carb::Float3>& dst, const SrcVecT* src, unsigned int numElements, const std::vector<uint32_t>& mapping)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < dst.size(); i++)
        dst[i] = carb::Float3{ 0.0f, 0.0f, 0.0f };
    for (unsigned int i = 0; i < numElements; i++)
    {
        const SrcVecT& srcValue = src[mapping[i]];
        dst[i] = carb::Float3{ float(srcValue.x), float(srcValue.y), float(srcValue.z) };
    }
}

template<typename SrcVecT>
void copyBuffer(std::vector<carb::Float3>& dst, const SrcVecT* src, unsigned int numElements, const ::physx::PxMat44d& transform, const std::vector<uint32_t>& mapping)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < dst.size(); i++)
        dst[i] = carb::Float3{ 0.0f, 0.0f, 0.0f };
    for (unsigned int i = 0; i < numElements; i++)
    {
        const SrcVecT& srcValue = src[mapping[i]];
        const ::physx::PxVec3d p = transform.transform(::physx::PxVec3d(double(srcValue.x), double(srcValue.y), double(srcValue.z)));
        dst[i] = carb::Float3{ float(p.x), float(p.y), float(p.z) };
    }
}

#if OMNI_PHYSX_OPTIMIZE_SSE
inline bool isAligned16(const void* a)
{
    return (0 == (size_t(a) & 0x0f));
}

template<>
inline void copyBuffer<::physx::PxVec4>(std::vector<carb::Float3>& dst,
                                        const ::physx::PxVec4* src,
                                        unsigned int numElements,
                                        const ::physx::PxMat44d& transform)
{
    // The SSE kernel is a float kernel: source points and destination are float,
    // so only the matrix is narrowed here. The sixteen values are laid out
    // exactly as the GfMatrix4f version loaded them (Gf row i == PhysX column i),
    // and the local is over-aligned so the matrix never forces the scalar path.
    alignas(16) float mat[16];
    {
        const double* m = transform.front();
        for (int i = 0; i < 16; ++i)
            mat[i] = float(m[i]);
    }

    // The SSE kernel's "Normalize" step below divides by the transformed w, unlike
    // PxMat44d::transform() (see the file-header TRANSFORM CONVENTION note), so the two
    // branches of this function only agree when the last row of `mat` is exactly
    // (0,0,0,1) -- every caller is documented to pass an affine world/local transform,
    // never a projective one. Assert it here, once, on the narrowed matrix both branches
    // share, rather than on every caller: this is a safety net for a future caller that
    // violates the contract, not a functional change for any input that already upholds
    // it.
    PX_ASSERT(mat[3] == 0.0f && mat[7] == 0.0f && mat[11] == 0.0f && mat[15] == 1.0f);

    // Resize before the gate: every caller passes a freshly-declared vector, whose
    // data() is null until it allocates. Testing that null would pass the alignment
    // check vacuously and let the SSE kernel _mm_store_ps into whatever the resize
    // then hands back, unchecked.
    //
    // std::vector::resize value-initializes the new range, so this zeroes the
    // whole buffer before every element below overwrites it -- unlike the old
    // VtArray::resize(n, kNoInit) this replaced. std::vector has no no-init
    // resize; avoiding it needs a custom allocator, which is out of scope for
    // this call site alone. Accepted: one linear memset is small next to the
    // per-element transform cost below, and every caller already owns a
    // freshly-declared (small-buffer) vector rather than a reused large one.
    dst.resize(numElements);

    // Narrowed-to-float matrix shared by the unaligned scalar path, the SIMD
    // main loop's remainder tail, and (via `mat`/l0..l3 above) the SIMD lanes
    // themselves, so all three agree bit-for-bit on identical input.
    const ::physx::PxMat44 matF(mat);

    if (!(isAligned16(dst.data()) && isAligned16(src)))
    {
        for (unsigned int i = 0; i < numElements; i++)
        {
            const ::physx::PxVec4& srcValue = src[i];
            const ::physx::PxVec3 p = matF.transform(::physx::PxVec3(srcValue.x, srcValue.y, srcValue.z));
            dst[i] = carb::Float3{ p.x, p.y, p.z };
        }
        return;
    }

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

    const __m128 l0 = _mm_load_ps(mat + 0 * 4);
    const __m128 l1 = _mm_load_ps(mat + 1 * 4);
    const __m128 l2 = _mm_load_ps(mat + 2 * 4);
    const __m128 l3 = _mm_load_ps(mat + 3 * 4);

    carb::Float3* vdest = dst.data();
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
        const ::physx::PxVec3 p = matF.transform(::physx::PxVec3(srcValue.x, srcValue.y, srcValue.z));
        *vdest++ = carb::Float3{ p.x, p.y, p.z };
    }
}
#endif // OMNI_PHYSX_OPTIMIZE_SSE


template<typename SrcVecT>
void copyBuffer(std::vector<carb::Float4>& dst, const SrcVecT* src, unsigned int numElements)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; i++)
    {
        const SrcVecT& srcValue = src[i];
        dst[i] = carb::Float4{ float(srcValue.x), float(srcValue.y), float(srcValue.z), float(srcValue.w) };
    }
}

template<typename SrcVecT>
void copyBuffer(std::vector<carb::Int2>& dst, const SrcVecT* src, unsigned int numElements)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; i++)
    {
        const SrcVecT& srcValue = src[i];
        dst[i] = carb::Int2{ int32_t(srcValue.x), int32_t(srcValue.y) };
    }
}

inline void copyBuffer(std::vector<carb::Float3>& dst, ::physx::PxVec4* src, unsigned int numElements, const ::physx::PxMat44d& transform)
{
    dst.resize(numElements);
    for (unsigned int i = 0; i < numElements; i++)
    {
        const ::physx::PxVec4& srcValue = src[i];
        const ::physx::PxVec3d p = transform.transform(::physx::PxVec3d(double(srcValue.x), double(srcValue.y), double(srcValue.z)));
        dst[i] = carb::Float3{ float(p.x), float(p.y), float(p.z) };
    }
}

template <typename SrcIndexT>
void copyBuffer(std::vector<carb::Int3>& dst, const SrcIndexT* src, unsigned int numSrcElements)
{
    PX_COMPILE_TIME_ASSERT(3 * sizeof(SrcIndexT) == sizeof(carb::Int3));
    uint32_t numDstElements = numSrcElements / 3;
    dst.resize(numDstElements);
    std::memcpy(dst.data(), src, numDstElements * sizeof(carb::Int3));
}

template<typename SrcIndexT>
void copyBuffer(std::vector<carb::Int4>& dst, const SrcIndexT* src, unsigned int numSrcElements)
{
    PX_COMPILE_TIME_ASSERT(4 * sizeof(SrcIndexT) == sizeof(carb::Int4));
    uint32_t numDstElements = numSrcElements / 4;
    dst.resize(numDstElements);
    std::memcpy(dst.data(), src, numDstElements * sizeof(carb::Int4));
}

template<typename SrcVecT>
void scatterBuffer(std::vector<carb::Float3>& dst, const SrcVecT* src, const std::vector<uint32_t>& indexMap)
{
    for (size_t i = 0; i < indexMap.size(); i++)
    {
        const SrcVecT& p = src[i];
        uint32_t index = indexMap[i];
        dst[index] = carb::Float3{ float(p.x), float(p.y), float(p.z) };
    }
}

// Orientations. The destination lanes are x,y,z,w with w == the real part, the
// same convention as toFloat4(GfQuatf)/toFloat4(GfQuath) in TypeCast.h. Note
// that the Gf version of this took a VtArray<GfQuath> -- half precision -- so a
// caller that still has to publish halves must convert with toQuath() at its own
// USD boundary; carb::Float4 is NOT layout-compatible with GfQuath.
template<typename SrcQuatT>
void scatterBuffer(std::vector<carb::Float4>& dst, const SrcQuatT* src, const std::vector<uint32_t>& indexMap)
{
    for (size_t i = 0; i < indexMap.size(); i++)
    {
        const SrcQuatT& p = src[i];
        uint32_t index = indexMap[i];
        dst[index] = carb::Float4{ float(p.x), float(p.y), float(p.z), float(p.w) };
    }
}

template<typename SrcVecT>
void scatterBuffer(std::vector<carb::Float3>& dst, const SrcVecT* src, const std::vector<uint32_t>& indexMap, const ::physx::PxMat44d& transform)
{
    for (size_t i = 0; i < indexMap.size(); i++)
    {
        const SrcVecT& p = src[i];
        uint32_t index = indexMap[i];
        const ::physx::PxVec3d t = transform.transform(::physx::PxVec3d(double(p.x), double(p.y), double(p.z)));
        dst[index] = carb::Float3{ float(t.x), float(t.y), float(t.z) };
    }
}

template<typename DstT>
void scatterBuffer(std::vector<DstT>& dst, const DstT& src, const std::vector<uint32_t>& indexMap)
{
    for (size_t i = 0; i < indexMap.size(); i++)
    {
        uint32_t index = indexMap[i];
        dst[index] = src;
    }
}


}
}
