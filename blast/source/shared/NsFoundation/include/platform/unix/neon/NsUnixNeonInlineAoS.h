// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.

#ifndef PS_UNIX_NEON_INLINE_AOS_H
#define PS_UNIX_NEON_INLINE_AOS_H

#if !COMPILE_VECTOR_INTRINSICS
#error Vector intrinsics should not be included when using scalar implementation.
#endif

// improved estimates
#define VRECIPEQ        recipq_newton<1>
#define VRECIPE         recip_newton<1>
#define VRECIPSQRTEQ    rsqrtq_newton<1>
#define VRECIPSQRTE     rsqrt_newton<1>

// "exact"
#define VRECIPQ         recipq_newton<4>
#define VRECIP          recip_newton<4>
#define VRECIPSQRTQ     rsqrtq_newton<4>
#define VRECIPSQRT      rsqrt_newton<4>

#define VECMATH_AOS_EPSILON (1e-3f)

//Remove this define when all platforms use simd solver.
#define NV_SUPPORT_SIMD

namespace internalUnitNeonSimd
{
    NV_FORCE_INLINE uint32_t BAllTrue4_R(const BoolV a)
    {
        const uint16x4_t dHigh = vget_high_u16(vreinterpretq_u16_u32(a));
        const uint16x4_t dLow = vmovn_u32(a);
        uint16x8_t combined = vcombine_u16(dLow, dHigh);
        const uint32x2_t finalReduce = vreinterpret_u32_u8(vmovn_u16(combined));
        return uint32_t(vget_lane_u32(finalReduce, 0) == 0xffffFFFF);
    }

    NV_FORCE_INLINE uint32_t BAnyTrue4_R(const BoolV a)
    {
        const uint16x4_t dHigh = vget_high_u16(vreinterpretq_u16_u32(a));
        const uint16x4_t dLow = vmovn_u32(a);
        uint16x8_t combined = vcombine_u16(dLow, dHigh);
        const uint32x2_t finalReduce = vreinterpret_u32_u8(vmovn_u16(combined));
        return uint32_t(vget_lane_u32(finalReduce, 0) != 0x0);
    }

    NV_FORCE_INLINE uint32_t BAllTrue3_R(const BoolV a)
    {
        const uint16x4_t dHigh = vget_high_u16(vreinterpretq_u16_u32(a));
        const uint16x4_t dLow = vmovn_u32(a);
        uint16x8_t combined = vcombine_u16(dLow, dHigh);
        const uint32x2_t finalReduce = vreinterpret_u32_u8(vmovn_u16(combined));
        return uint32_t((vget_lane_u32(finalReduce, 0) & 0xffFFff) == 0xffFFff);
    }

    NV_FORCE_INLINE uint32_t BAnyTrue3_R(const BoolV a)
    {
        const uint16x4_t dHigh = vget_high_u16(vreinterpretq_u16_u32(a));
        const uint16x4_t dLow = vmovn_u32(a);
        uint16x8_t combined = vcombine_u16(dLow, dHigh);
        const uint32x2_t finalReduce = vreinterpret_u32_u8(vmovn_u16(combined));
        return uint32_t((vget_lane_u32(finalReduce, 0) & 0xffFFff) != 0);
    }
}

namespace _VecMathTests
{
    NV_FORCE_INLINE bool allElementsEqualFloatV(const FloatV a, const FloatV b)
    {
        VECMATHAOS_ASSERT(isValidFloatV(a));
        VECMATHAOS_ASSERT(isValidFloatV(b));
        return vget_lane_u32(vceq_f32(a, b), 0) != 0;
    }

    NV_FORCE_INLINE bool allElementsEqualVec3V(const Vec3V a, const Vec3V b)
    {
        VECMATHAOS_ASSERT(isValidVec3V(a));
        VECMATHAOS_ASSERT(isValidVec3V(b));
        return V3AllEq(a, b) != 0;
    }

    NV_FORCE_INLINE bool allElementsEqualVec4V(const Vec4V a, const Vec4V b)
    {
        return V4AllEq(a, b) != 0;
    }

    NV_FORCE_INLINE bool allElementsEqualBoolV(const BoolV a, const BoolV b)
    {
        return internalUnitNeonSimd::BAllTrue4_R(vceqq_u32(a, b)) != 0;
    }

    NV_FORCE_INLINE uint32_t V4U32AllEq(const VecU32V a, const VecU32V b)
    {
        return internalUnitNeonSimd::BAllTrue4_R(V4IsEqU32(a, b));
    }

    NV_FORCE_INLINE bool allElementsEqualVecU32V(const VecU32V a, const VecU32V b)
    {
        return V4U32AllEq(a, b) != 0;
    }
    
    NV_FORCE_INLINE BoolV V4IsEqI32(const VecI32V a, const VecI32V b)
    {
        return vceqq_s32(a, b);
    }
    NV_FORCE_INLINE uint32_t V4I32AllEq(const VecI32V a, const VecI32V b)
    {
        return internalUnitNeonSimd::BAllTrue4_R(V4IsEqI32(a, b));
    }
    
    NV_FORCE_INLINE bool allElementsEqualVecI32V(const VecI32V a, const VecI32V b)
    {
        return V4I32AllEq(a, b) != 0;
    }



    NV_FORCE_INLINE bool allElementsNearEqualFloatV(const FloatV a, const FloatV b)
    {
        VECMATHAOS_ASSERT(isValidFloatV(a));
        VECMATHAOS_ASSERT(isValidFloatV(b));

        const float32x2_t c = vsub_f32(a, b);
        const float32x2_t error = vdup_n_f32(VECMATH_AOS_EPSILON);
        // absolute compare abs(error) > abs(c)
#if NV_WINRT     
        const uint32x2_t greater = vacgt_f32(error, c);
#else
        const uint32x2_t greater = vcagt_f32(error, c);
#endif
        const uint32x2_t min = vpmin_u32(greater, greater);
        return vget_lane_u32(min, 0) != 0x0;
    }

    NV_FORCE_INLINE bool allElementsNearEqualVec3V(const Vec3V a, const Vec3V b)
    {
        VECMATHAOS_ASSERT(isValidVec3V(a));
        VECMATHAOS_ASSERT(isValidVec3V(b));
        const float32x4_t c = vsubq_f32(a, b);
        const float32x4_t error = vdupq_n_f32(VECMATH_AOS_EPSILON);
        // absolute compare abs(error) > abs(c)
#if NV_WINRT
        const uint32x4_t greater = vacgtq_f32(error, c);
#else
        const uint32x4_t greater = vcagtq_f32(error, c);
#endif
        return internalUnitNeonSimd::BAllTrue3_R(greater) != 0;
    }

    NV_FORCE_INLINE bool allElementsNearEqualVec4V(const Vec4V a, const Vec4V b)
    {
        const float32x4_t c = vsubq_f32(a, b);
        const float32x4_t error = vdupq_n_f32(VECMATH_AOS_EPSILON);
        // absolute compare abs(error) > abs(c)
#if NV_WINRT
        const uint32x4_t greater = vacgtq_f32(error, c);
#else
        const uint32x4_t greater = vcagtq_f32(error, c);
#endif
        return internalUnitNeonSimd::BAllTrue4_R(greater) != 0x0;
    }
}


#if 0 // debugging printfs
#include <stdio.h>
NV_FORCE_INLINE void printVec(const float32x4_t& v, const char* name)
{
    NV_ALIGN(16, float32_t) data[4];
    vst1q_f32(data, v);
    printf("%s: (%f, %f, %f, %f)\n", name, data[0], data[1], data[2], data[3]);
}

NV_FORCE_INLINE void printVec(const float32x2_t& v, const char* name)
{
    NV_ALIGN(16, float32_t) data[2];
    vst1_f32(data, v);
    printf("%s: (%f, %f)\n", name, data[0], data[1]);
}

NV_FORCE_INLINE void printVec(const uint32x4_t& v, const char* name)
{
    NV_ALIGN(16, uint32_t) data[4];
    vst1q_u32(data, v);
    printf("%s: (0x%x, 0x%x, 0x%x, 0x%x)\n", name, data[0], data[1], data[2], data[3]);
}

NV_FORCE_INLINE void printVec(const uint16x8_t& v, const char* name)
{
    NV_ALIGN(16, uint16_t) data[8];
    vst1q_u16(data, v);
    printf("%s: (0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n", name, data[0], data[1], data[2], data[3],
        data[4], data[5], data[6], data[7]);
}

NV_FORCE_INLINE void printVec(const int32x4_t& v, const char* name)
{
    NV_ALIGN(16, int32_t) data[4];
    vst1q_s32(data, v);
    printf("%s: (0x%x, 0x%x, 0x%x, 0x%x)\n", name, data[0], data[1], data[2], data[3]);
}

NV_FORCE_INLINE void printVec(const int16x8_t& v, const char* name)
{
    NV_ALIGN(16, int16_t) data[8];
    vst1q_s16(data, v);
    printf("%s: (0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n", name, data[0], data[1], data[2], data[3],
        data[4], data[5], data[6], data[7]);
}

NV_FORCE_INLINE void printVec(const uint16x4_t& v, const char* name)
{
    NV_ALIGN(16, uint16_t) data[4];
    vst1_u16(data, v);
    printf("%s: (0x%x, 0x%x, 0x%x, 0x%x)\n", name, data[0], data[1], data[2], data[3]);
}

NV_FORCE_INLINE void printVec(const uint32x2_t& v, const char* name)
{
    NV_ALIGN(16, uint32_t) data[2];
    vst1_u32(data, v);
    printf("%s: (0x%x, 0x%x)\n", name, data[0], data[1]);
}

NV_FORCE_INLINE void printVar(const uint32_t v, const char* name)
{
    printf("%s: 0x%x\n", name, v);
}

NV_FORCE_INLINE void printVar(const float v, const char* name)
{
    printf("%s: %f\n", name, v);
}

#define PRINT_VAR(X) printVar((X), #X)
#define PRINT_VEC(X) printVec((X), #X)
#define PRINT_VEC_TITLE(TITLE, X) printVec((X), TITLE #X)
#endif // debugging printf



/////////////////////////////////////////////////////////////////////
////FUNCTIONS USED ONLY FOR ASSERTS IN VECTORISED IMPLEMENTATIONS
/////////////////////////////////////////////////////////////////////
NV_FORCE_INLINE bool isValidFloatV(const FloatV a)
{
    NV_ALIGN(16,float) data[4];
    vst1_f32((float32_t*)data, a);
    if(isFiniteFloatV(a))
        return data[0] == data[1];
    else
    {
        uint32_t* intData = (uint32_t*)data;
        return intData[0] == intData[1];
    }
}

NV_FORCE_INLINE bool isValidVec3V(const Vec3V a)
{
    const float32_t w = vgetq_lane_f32(a, 3);

    if(isFiniteVec3V(a))
        return w == 0.0f;
    else
    {

        NV_ALIGN(16,float) data[4];
        vst1q_f32((float32_t*)data, a);
        uint32_t* intData = (uint32_t*)data;
        return !intData[3] || ((intData[0] == intData[1]) && (intData[0] == intData[2]) && (intData[0] == intData[3]));
    }
}


NV_FORCE_INLINE bool isFiniteFloatV(const FloatV a)
{
    NV_ALIGN(16,float) data[4];
    vst1_f32((float32_t*)data, a);
    return NvIsFinite(data[0]) && NvIsFinite(data[1]);
}

NV_FORCE_INLINE bool isFiniteVec3V(const Vec3V a)
{
    NV_ALIGN(16,float) data[4];
    vst1q_f32((float32_t*)data, a);
    return NvIsFinite(data[0]) && NvIsFinite(data[1]) && NvIsFinite(data[2]);
}

NV_FORCE_INLINE bool isFiniteVec4V(const Vec4V a)
{
    NV_ALIGN(16,float) data[4];
    vst1q_f32((float32_t*)data, a);
    return NvIsFinite(data[0]) && NvIsFinite(data[1]) && NvIsFinite(data[2]) && NvIsFinite(data[3]);
}

NV_FORCE_INLINE bool hasZeroElementinFloatV(const FloatV a)
{
    VECMATHAOS_ASSERT(isValidFloatV(a));
    return vget_lane_u32(vreinterpret_u32_f32(a), 0) == 0;
}

NV_FORCE_INLINE bool hasZeroElementInVec3V(const Vec3V a)
{
    const uint32x2_t dLow = vget_low_u32(vreinterpretq_u32_f32(a));
    const uint32x2_t dMin = vpmin_u32(dLow, dLow);

    return vget_lane_u32(dMin, 0) == 0 || vgetq_lane_u32(vreinterpretq_u32_f32(a), 2) == 0;
}

NV_FORCE_INLINE bool hasZeroElementInVec4V(const Vec4V a)
{
    const uint32x2_t dHigh = vget_high_u32(vreinterpretq_u32_f32(a));
    const uint32x2_t dLow = vget_low_u32(vreinterpretq_u32_f32(a));

    const uint32x2_t dMin = vmin_u32(dHigh, dLow);
    const uint32x2_t pairMin = vpmin_u32(dMin, dMin);
    return vget_lane_u32(pairMin, 0) == 0;
}

/////////////////////////////////////////////////////////////////////
////VECTORISED FUNCTION IMPLEMENTATIONS
/////////////////////////////////////////////////////////////////////

NV_FORCE_INLINE FloatV FLoad(const float f)
{
    return vdup_n_f32(reinterpret_cast<const float32_t&>(f));
}

NV_FORCE_INLINE FloatV FLoadA(const float* const f)
{
    VECMATHAOS_ASSERT(0 == ((uint64_t)f & 0x0f));
    return vld1_f32((const float32_t*)f);
}

NV_FORCE_INLINE Vec3V V3Load(const float f)
{
    NV_ALIGN(16, float) data[4] = {f, f, f, 0.0f};
    return V4LoadA(data);
}

NV_FORCE_INLINE Vec4V V4Load(const float f)
{
    return vdupq_n_f32(reinterpret_cast<const float32_t&>(f));
}

NV_FORCE_INLINE BoolV BLoad(const bool f)
{
    const uint32_t i=uint32_t(-(int32_t)f);
    return vdupq_n_u32(i);
}

NV_FORCE_INLINE Vec3V V3LoadA(const NvVec3& f)
{
    VECMATHAOS_ASSERT(0 == ((size_t)&f & 0x0f));
    NV_ALIGN(16, float) data[4] = {f.x, f.y, f.z, 0.0f};
    return V4LoadA(data);
}

NV_FORCE_INLINE Vec3V V3LoadU(const NvVec3& f)
{
    NV_ALIGN(16, float) data[4] = {f.x, f.y, f.z, 0.0f};
    return V4LoadA(data);
}

NV_FORCE_INLINE Vec3V V3LoadUnsafeA(const NvVec3& f)
{
    NV_ALIGN(16, float) data[4] = {f.x, f.y, f.z, 0.0f};
    return V4LoadA(data);
}

NV_FORCE_INLINE Vec3V V3LoadA(const float* f)
{
    VECMATHAOS_ASSERT(0 == ((size_t)&f & 0x0f));
    NV_ALIGN(16, float) data[4] = {f[0], f[1], f[2], 0.0f};
    return V4LoadA(data);
}

NV_FORCE_INLINE Vec3V V3LoadU(const float* f)
{
    NV_ALIGN(16, float) data[4] = {f[0], f[1], f[2], 0.0f};
    return V4LoadA(data);
}

NV_FORCE_INLINE Vec3V Vec3V_From_Vec4V(Vec4V v)
{
    return vsetq_lane_f32(0.0f, v, 3);
}

NV_FORCE_INLINE Vec3V Vec3V_From_Vec4V_WUndefined(Vec4V v)
{
    return v;
}



NV_FORCE_INLINE Vec4V Vec4V_From_Vec3V(Vec3V f)
{
    return f;   //ok if it is implemented as the same type.
}

NV_FORCE_INLINE Vec4V Vec4V_From_FloatV(FloatV f)
{
    return vcombine_f32(f, f);
}

NV_FORCE_INLINE Vec3V Vec3V_From_FloatV(FloatV f)
{
    return Vec3V_From_Vec4V(Vec4V_From_FloatV(f));
}

NV_FORCE_INLINE Vec3V Vec3V_From_FloatV_WUndefined(FloatV f)
{
    return Vec3V_From_Vec4V_WUndefined(Vec4V_From_FloatV(f));
}

NV_FORCE_INLINE Vec4V Vec4V_From_NvVec3_WUndefined(const NvVec3& f)
{
    NV_ALIGN(16, float) data[4] = {f.x, f.y, f.z, 0.0f};
    return V4LoadA(data);
}

NV_FORCE_INLINE Mat33V Mat33V_From_NvMat33(const NvMat33 &m)
{
    return Mat33V(V3LoadU(m.column0),
        V3LoadU(m.column1),
        V3LoadU(m.column2));
}

NV_FORCE_INLINE void NvMat33_From_Mat33V(const Mat33V &m, NvMat33 &out)
{
    NV_ASSERT((size_t(&out)&15)==0);
    V3StoreU(m.col0, out.column0);
    V3StoreU(m.col1, out.column1);
    V3StoreU(m.col2, out.column2);
}


NV_FORCE_INLINE Vec4V V4LoadA(const float* const f)
{
    VECMATHAOS_ASSERT(0 == ((uint64_t)f & 0x0f));
    return vld1q_f32((const float32_t*)f);
}

NV_FORCE_INLINE void V4StoreA(Vec4V a, float* f)
{
    VECMATHAOS_ASSERT(0 == ((uint64_t)f & 0x0f));
    vst1q_f32((float32_t*)f,a);
}

NV_FORCE_INLINE void V4StoreU(const Vec4V a, float* f)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(0 == ((int)&a & 0x0F));
    NV_ALIGN(16,float) f2[4];
    vst1q_f32((float32_t*)f2, a);
    f[0] = f2[0];
    f[1] = f2[1];
    f[2] = f2[2];
    f[3] = f2[3];
}

NV_FORCE_INLINE void BStoreA(const BoolV a, uint32_t* u)
{
    VECMATHAOS_ASSERT(0 == ((uint64_t)u & 0x0f));
    vst1q_u32((uint32_t*)u,a);
}

NV_FORCE_INLINE void U4StoreA(const VecU32V uv, uint32_t* u)
{
    VECMATHAOS_ASSERT(0 == ((uint64_t)u & 0x0f));
    vst1q_u32((uint32_t*)u,uv);
}

NV_FORCE_INLINE void I4StoreA(const VecI32V iv, int32_t* i)
{
    VECMATHAOS_ASSERT(0 == ((uint64_t)i & 0x0f));
    vst1q_s32((int32_t*)i,iv);
}

NV_FORCE_INLINE Vec4V V4LoadU(const float* const f)
{
    return vld1q_f32((const float32_t*)f);
}

NV_FORCE_INLINE BoolV BLoad(const bool* const f)
{
    const NV_ALIGN(16, uint32_t) b[4]={(uint32_t)(-(int32_t)f[0]), (uint32_t)(-(int32_t)f[1]), (uint32_t)(-(int32_t)f[2]), (uint32_t)(-(int32_t)f[3])};
    return vld1q_u32(b);
}

NV_FORCE_INLINE float FStore(const FloatV a)
{
    VECMATHAOS_ASSERT(isValidFloatV(a));
    float f = vget_lane_f32(a, 0);
    return f;
}

NV_FORCE_INLINE void FStore(const FloatV a, float* NV_RESTRICT f)
{
    VECMATHAOS_ASSERT(isValidFloatV(a));
    //vst1q_lane_f32(f, a, 0); // causes vst1 alignment bug
    *f = vget_lane_f32(a, 0);
}

NV_FORCE_INLINE void V3StoreA(const Vec3V a, NvVec3& f)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(0 == ((int)&a & 0x0F));
    VECMATHAOS_ASSERT(0 == ((int)&f & 0x0F));
    NV_ALIGN(16,float) f2[4];
    vst1q_f32((float32_t*)f2, a);
    f = NvVec3(f2[0], f2[1], f2[2]);
}

NV_FORCE_INLINE void V3StoreU(const Vec3V a, NvVec3& f)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(0 == ((int)&a & 0x0F));
    NV_ALIGN(16,float) f2[4];
    vst1q_f32((float32_t*)f2, a);
    f = NvVec3(f2[0], f2[1], f2[2]);
}



//////////////////////////////////
//FLOATV
//////////////////////////////////

NV_FORCE_INLINE FloatV FZero()
{
    return FLoad(0.0f);
}

NV_FORCE_INLINE FloatV FOne()
{
    return FLoad(1.0f);
}

NV_FORCE_INLINE FloatV FHalf()
{
    return FLoad(0.5f);
}

NV_FORCE_INLINE FloatV FEps()
{
    return FLoad(NV_EPS_REAL);
}

NV_FORCE_INLINE FloatV FEps6()
{
    return FLoad(1e-6f);
}

NV_FORCE_INLINE FloatV FMax()
{
    return FLoad(NV_MAX_REAL);
}

NV_FORCE_INLINE FloatV FNegMax()
{
    return FLoad(-NV_MAX_REAL);
}

NV_FORCE_INLINE FloatV IZero()
{
    return vreinterpret_f32_u32(vdup_n_u32(0));
}

NV_FORCE_INLINE FloatV IOne()
{
    return vreinterpret_f32_u32(vdup_n_u32(1));
}

NV_FORCE_INLINE FloatV ITwo()
{
    return vreinterpret_f32_u32(vdup_n_u32(2));
}

NV_FORCE_INLINE FloatV IThree()
{
    return vreinterpret_f32_u32(vdup_n_u32(3));
}

NV_FORCE_INLINE FloatV IFour()
{
    return vreinterpret_f32_u32(vdup_n_u32(4));
}

NV_FORCE_INLINE FloatV FNeg(const FloatV f)
{
    VECMATHAOS_ASSERT(isValidFloatV(f));
    return vneg_f32(f);
}

NV_FORCE_INLINE FloatV FAdd(const FloatV a, const FloatV b)
{
    VECMATHAOS_ASSERT(isValidFloatV(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    return vadd_f32(a, b);
}

NV_FORCE_INLINE FloatV FSub(const FloatV a, const FloatV b)
{
    VECMATHAOS_ASSERT(isValidFloatV(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    return vsub_f32(a, b);
}

NV_FORCE_INLINE FloatV FMul(const FloatV a, const FloatV b)
{
    VECMATHAOS_ASSERT(isValidFloatV(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    return vmul_f32(a, b);
}

template <int n>
NV_FORCE_INLINE float32x2_t recip_newton(const float32x2_t& in)
{
    float32x2_t recip = vrecpe_f32(in);
    for(int i=0; i<n; ++i)
        recip = vmul_f32(recip, vrecps_f32(in, recip));
    return recip;
}

template <int n>
NV_FORCE_INLINE float32x4_t recipq_newton(const float32x4_t& in)
{
    float32x4_t recip = vrecpeq_f32(in);
    for(int i=0; i<n; ++i)
        recip = vmulq_f32(recip, vrecpsq_f32(recip, in));
    return recip;
}

template <int n>
NV_FORCE_INLINE float32x2_t rsqrt_newton(const float32x2_t& in)
{
    float32x2_t rsqrt = vrsqrte_f32(in);
    for(int i=0; i<n; ++i)
        rsqrt = vmul_f32(rsqrt, vrsqrts_f32(vmul_f32(rsqrt, rsqrt), in));
    return rsqrt;
}

template <int n>
NV_FORCE_INLINE float32x4_t rsqrtq_newton(const float32x4_t& in)
{
    float32x4_t rsqrt = vrsqrteq_f32(in);
    for(int i=0; i<n; ++i)
        rsqrt = vmulq_f32(rsqrt, vrsqrtsq_f32(vmulq_f32(rsqrt, rsqrt), in));
    return rsqrt;
}

NV_FORCE_INLINE FloatV FDiv(const FloatV a, const FloatV b)
{
    VECMATHAOS_ASSERT(isValidFloatV(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    return  vmul_f32(a, VRECIP(b));
}

NV_FORCE_INLINE FloatV FDivFast(const FloatV a, const FloatV b)
{
    VECMATHAOS_ASSERT(isValidFloatV(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    return  vmul_f32(a, VRECIPE(b));
}

NV_FORCE_INLINE FloatV FRecip(const FloatV a)
{
    return VRECIP(a);
}

NV_FORCE_INLINE FloatV FRecipFast(const FloatV a)
{
    return VRECIPE(a);
}

NV_FORCE_INLINE FloatV FRsqrt(const FloatV a)
{
    return VRECIPSQRT(a);
}

NV_FORCE_INLINE FloatV FSqrt(const FloatV a)
{
    return vmul_f32(a, VRECIPSQRT(a));
}

NV_FORCE_INLINE FloatV FRsqrtFast(const FloatV a)
{
    return VRECIPSQRTE(a);
}

NV_FORCE_INLINE FloatV FScaleAdd(const FloatV a, const FloatV b, const FloatV c)
{
    VECMATHAOS_ASSERT(isValidFloatV(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    VECMATHAOS_ASSERT(isValidFloatV(c));
    return vmla_f32(c, a, b);
}

NV_FORCE_INLINE FloatV FNegScaleSub(const FloatV a, const FloatV b, const FloatV c)
{
    VECMATHAOS_ASSERT(isValidFloatV(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    VECMATHAOS_ASSERT(isValidFloatV(c));
    return vmls_f32(c, a, b);
}

NV_FORCE_INLINE FloatV FAbs(const FloatV a)
{
    VECMATHAOS_ASSERT(isValidFloatV(a));
    return vabs_f32(a);
}

NV_FORCE_INLINE FloatV FSel(const BoolV c, const FloatV a, const FloatV b)
{
    VECMATHAOS_ASSERT(_VecMathTests::allElementsEqualBoolV(c,BTTTT()) || _VecMathTests::allElementsEqualBoolV(c,BFFFF()));
    VECMATHAOS_ASSERT(isValidFloatV(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    return vbsl_f32(vget_low_u32(c), a, b);
}

NV_FORCE_INLINE BoolV FIsGrtr(const FloatV a, const FloatV b)
{
    VECMATHAOS_ASSERT(isValidFloatV(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    return vdupq_lane_u32(vcgt_f32(a, b), 0);
}

NV_FORCE_INLINE BoolV FIsGrtrOrEq(const FloatV a, const FloatV b)
{
    VECMATHAOS_ASSERT(isValidFloatV(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    return vdupq_lane_u32(vcge_f32(a, b), 0);
}

NV_FORCE_INLINE BoolV FIsEq(const FloatV a, const FloatV b)
{
    VECMATHAOS_ASSERT(isValidFloatV(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    return vdupq_lane_u32(vceq_f32(a, b), 0);
}

NV_FORCE_INLINE FloatV FMax(const FloatV a, const FloatV b)
{
    VECMATHAOS_ASSERT(isValidFloatV(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    return vmax_f32(a, b);
}

NV_FORCE_INLINE FloatV FMin(const FloatV a, const FloatV b)
{
    VECMATHAOS_ASSERT(isValidFloatV(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    return vmin_f32(a, b);
}

NV_FORCE_INLINE FloatV FClamp(const FloatV a, const FloatV minV, const FloatV maxV)
{
    VECMATHAOS_ASSERT(isValidFloatV(a));
    VECMATHAOS_ASSERT(isValidFloatV(minV));
    VECMATHAOS_ASSERT(isValidFloatV(maxV));
    return FMax(FMin(a,maxV),minV);
}

NV_FORCE_INLINE uint32_t FAllGrtr(const FloatV a, const FloatV b)
{
    VECMATHAOS_ASSERT(isValidFloatV(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    return vget_lane_u32(vcgt_f32(a, b), 0);
}

NV_FORCE_INLINE uint32_t FAllGrtrOrEq(const FloatV a, const FloatV b)
{
    VECMATHAOS_ASSERT(isValidFloatV(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    return vget_lane_u32(vcge_f32(a, b), 0);
}

NV_FORCE_INLINE uint32_t FAllEq(const FloatV a, const FloatV b)
{
    VECMATHAOS_ASSERT(isValidFloatV(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    return vget_lane_u32(vceq_f32(a, b), 0);
}

NV_FORCE_INLINE FloatV FRound(const FloatV a)
{
    //truncate(a + (0.5f - sign(a)))
    const float32x2_t half = vdup_n_f32(0.5f);
    const float32x2_t sign = vcvt_f32_u32((vshr_n_u32(vreinterpret_u32_f32(a), 31)));
    const float32x2_t aPlusHalf = vadd_f32(a, half);
    const float32x2_t aRound = vsub_f32(aPlusHalf, sign);
    int32x2_t tmp = vcvt_s32_f32(aRound);
    return vcvt_f32_s32(tmp);
}


NV_FORCE_INLINE FloatV FSin(const FloatV a)
{
    //Vec4V V1, V2, V3, V5, V7, V9, V11, V13, V15, V17, V19, V21, V23;
    //Vec4V S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11;
    FloatV Result;

    // Modulo the range of the given angles such that -XM_2PI <= Angles < XM_2PI
    const FloatV recipTwoPi = FLoadA(g_NVReciprocalTwoPi.f);
    const FloatV twoPi = FLoadA(g_NVTwoPi.f);
    const FloatV tmp = FMul(a, recipTwoPi);
    const FloatV b = FRound(tmp);
    const FloatV V1 = FNegMulSub(twoPi, b, a);

    // sin(V) ~= V - V^3 / 3! + V^5 / 5! - V^7 / 7! + V^9 / 9! - V^11 / 11! + V^13 / 13! -
    //           V^15 / 15! + V^17 / 17! - V^19 / 19! + V^21 / 21! - V^23 / 23! (for -PI <= V < PI)
    const FloatV V2  = FMul(V1, V1);
    const FloatV V3  = FMul(V2, V1);
    const FloatV V5  = FMul(V3, V2);
    const FloatV V7  = FMul(V5, V2);
    const FloatV V9  = FMul(V7, V2);
    const FloatV V11 = FMul(V9, V2);
    const FloatV V13 = FMul(V11, V2);
    const FloatV V15 = FMul(V13, V2);
    const FloatV V17 = FMul(V15, V2);
    const FloatV V19 = FMul(V17, V2);
    const FloatV V21 = FMul(V19, V2);
    const FloatV V23 = FMul(V21, V2);

    const Vec4V sinCoefficients0 = V4LoadA(g_NVSinCoefficients0.f);
    const Vec4V sinCoefficients1 = V4LoadA(g_NVSinCoefficients1.f);
    const Vec4V sinCoefficients2 = V4LoadA(g_NVSinCoefficients2.f);

    const FloatV S1  = V4GetY(sinCoefficients0);
    const FloatV S2  = V4GetZ(sinCoefficients0);
    const FloatV S3  = V4GetW(sinCoefficients0);
    const FloatV S4  = V4GetX(sinCoefficients1);
    const FloatV S5  = V4GetY(sinCoefficients1);
    const FloatV S6  = V4GetZ(sinCoefficients1);
    const FloatV S7  = V4GetW(sinCoefficients1);
    const FloatV S8  = V4GetX(sinCoefficients2);
    const FloatV S9  = V4GetY(sinCoefficients2);
    const FloatV S10 = V4GetZ(sinCoefficients2);
    const FloatV S11 = V4GetW(sinCoefficients2);

    Result = FMulAdd(S1, V3, V1);
    Result = FMulAdd(S2, V5, Result);
    Result = FMulAdd(S3, V7, Result);
    Result = FMulAdd(S4, V9, Result);
    Result = FMulAdd(S5, V11, Result);
    Result = FMulAdd(S6, V13, Result);
    Result = FMulAdd(S7, V15, Result);
    Result = FMulAdd(S8, V17, Result);
    Result = FMulAdd(S9, V19, Result);
    Result = FMulAdd(S10, V21, Result);
    Result = FMulAdd(S11, V23, Result);

    return Result;

}

NV_FORCE_INLINE FloatV FCos(const FloatV a)
{
    //XMVECTOR V1, V2, V4, V6, V8, V10, V12, V14, V16, V18, V20, V22;
    //XMVECTOR C1, C2, C3, C4, C5, C6, C7, C8, C9, C10, C11;
    FloatV Result;

    // Modulo the range of the given angles such that -XM_2PI <= Angles < XM_2PI
    const FloatV recipTwoPi = FLoadA(g_NVReciprocalTwoPi.f);
    const FloatV twoPi = FLoadA(g_NVTwoPi.f);
    const FloatV tmp = FMul(a, recipTwoPi);
    const FloatV b = FRound(tmp);
    const FloatV V1 = FNegMulSub(twoPi, b, a);

    // cos(V) ~= 1 - V^2 / 2! + V^4 / 4! - V^6 / 6! + V^8 / 8! - V^10 / 10! + V^12 / 12! -
    //           V^14 / 14! + V^16 / 16! - V^18 / 18! + V^20 / 20! - V^22 / 22! (for -PI <= V < PI)
    const FloatV V2  = FMul(V1, V1);
    const FloatV V4  = FMul(V2, V2);
    const FloatV V6  = FMul(V4, V2);
    const FloatV V8  = FMul(V4, V4);
    const FloatV V10 = FMul(V6, V4);
    const FloatV V12 = FMul(V6, V6);
    const FloatV V14 = FMul(V8, V6);
    const FloatV V16 = FMul(V8, V8);
    const FloatV V18 = FMul(V10, V8);
    const FloatV V20 = FMul(V10, V10);
    const FloatV V22 = FMul(V12, V10);

    const Vec4V cosCoefficients0 = V4LoadA(g_NVCosCoefficients0.f);
    const Vec4V cosCoefficients1 = V4LoadA(g_NVCosCoefficients1.f);
    const Vec4V cosCoefficients2 = V4LoadA(g_NVCosCoefficients2.f);

    const FloatV C1  = V4GetY(cosCoefficients0);
    const FloatV C2  = V4GetZ(cosCoefficients0);
    const FloatV C3  = V4GetW(cosCoefficients0);
    const FloatV C4  = V4GetX(cosCoefficients1);
    const FloatV C5  = V4GetY(cosCoefficients1);
    const FloatV C6  = V4GetZ(cosCoefficients1);
    const FloatV C7  = V4GetW(cosCoefficients1);
    const FloatV C8  = V4GetX(cosCoefficients2);
    const FloatV C9  = V4GetY(cosCoefficients2);
    const FloatV C10 = V4GetZ(cosCoefficients2);
    const FloatV C11 = V4GetW(cosCoefficients2);

    Result = FMulAdd(C1, V2, FOne());
    Result = FMulAdd(C2, V4, Result);
    Result = FMulAdd(C3, V6, Result);
    Result = FMulAdd(C4, V8, Result);
    Result = FMulAdd(C5, V10, Result);
    Result = FMulAdd(C6, V12, Result);
    Result = FMulAdd(C7, V14, Result);
    Result = FMulAdd(C8, V16, Result);
    Result = FMulAdd(C9, V18, Result);
    Result = FMulAdd(C10, V20, Result);
    Result = FMulAdd(C11, V22, Result);

    return Result;

}

NV_FORCE_INLINE uint32_t FOutOfBounds(const FloatV a, const FloatV min, const FloatV max)
{
    const BoolV ffff = BFFFF();
    const BoolV c = BOr(FIsGrtr(a, max), FIsGrtr(min, a));
    return uint32_t(!BAllEq(c, ffff));
}

NV_FORCE_INLINE uint32_t FInBounds(const FloatV a, const FloatV min, const FloatV max)
{
    const BoolV tttt = BTTTT();
    const BoolV c = BAnd(FIsGrtrOrEq(a, min), FIsGrtrOrEq(max, a));
    return uint32_t(BAllEq(c, tttt));
}

NV_FORCE_INLINE uint32_t FOutOfBounds(const FloatV a, const FloatV bounds)
{
#if NV_WINRT
    const uint32x2_t greater = vacgt_f32(a, bounds);
#else
    const uint32x2_t greater = vcagt_f32(a, bounds);
#endif
    return vget_lane_u32(greater, 0);
}

NV_FORCE_INLINE uint32_t FInBounds(const FloatV a, const FloatV bounds)
{
#if NV_WINRT
    const uint32x2_t geq = vacge_f32(bounds, a);
#else
    const uint32x2_t geq = vcage_f32(bounds, a);
#endif
    return vget_lane_u32(geq, 0);
}



//////////////////////////////////
//VEC3V
//////////////////////////////////

NV_FORCE_INLINE Vec3V V3Splat(const FloatV f)
{
    VECMATHAOS_ASSERT(isValidFloatV(f));

#if NV_WINRT
    const uint32x2_t mask = { 0x00000000ffffFFFFULL };
#else
    const uint32x2_t mask = {0xffffFFFF, 0x0};
#endif

    const uint32x2_t uHigh = vreinterpret_u32_f32(f);
    const float32x2_t dHigh = vreinterpret_f32_u32(vand_u32(uHigh, mask));

    return vcombine_f32(f, dHigh);
}

NV_FORCE_INLINE Vec3V V3Merge(const FloatVArg x, const FloatVArg y, const FloatVArg z)
{
    VECMATHAOS_ASSERT(isValidFloatV(x));
    VECMATHAOS_ASSERT(isValidFloatV(y));
    VECMATHAOS_ASSERT(isValidFloatV(z));

#if NV_WINRT
    const uint32x2_t mask = { 0x00000000ffffFFFFULL };
#else
    const uint32x2_t mask = {0xffffFFFF, 0x0};
#endif

    const uint32x2_t dHigh = vand_u32(vreinterpret_u32_f32(z), mask);
    const uint32x2_t dLow = vext_u32(vreinterpret_u32_f32(x), vreinterpret_u32_f32(y), 1);
    return vreinterpretq_f32_u32(vcombine_u32(dLow, dHigh));
}

NV_FORCE_INLINE Vec3V V3UnitX()
{
#if NV_WINRT
    const float32x4_t x = { 0x000000003f800000ULL, 0x0ULL};
#else
    const float32x4_t x = { 1.0f, 0.0f, 0.0f, 0.0f};
#endif // NV_WINRT
    return x;
}

NV_FORCE_INLINE Vec3V V3UnitY()
{
#if NV_WINRT
    const float32x4_t y = { 0x3f80000000000000ULL, 0x0ULL};
#else
    const float32x4_t y = {0, 1.0f, 0, 0};
#endif
    return y;
}

NV_FORCE_INLINE Vec3V V3UnitZ()
{
#if NV_WINRT
    const float32x4_t z = { 0x0ULL, 0x000000003f800000ULL };
#else
    const float32x4_t z = {0, 0, 1.0f, 0};
#endif
    return z;
}

NV_FORCE_INLINE FloatV V3GetX(const Vec3V f)
{
    const float32x2_t fLow = vget_low_f32(f);
    return vdup_lane_f32(fLow, 0);
}

NV_FORCE_INLINE FloatV V3GetY(const Vec3V f)
{
    const float32x2_t fLow = vget_low_f32(f);
    return vdup_lane_f32(fLow, 1);
}

NV_FORCE_INLINE FloatV V3GetZ(const Vec3V f)
{
    const float32x2_t fhigh = vget_high_f32(f);
    return vdup_lane_f32(fhigh, 0);
}

NV_FORCE_INLINE Vec3V V3SetX(const Vec3V v, const FloatV f)
{
    VECMATHAOS_ASSERT(isValidVec3V(v));
    VECMATHAOS_ASSERT(isValidFloatV(f));
    return V3Sel(BFTTT(),v,  vcombine_f32(f, f));
}

NV_FORCE_INLINE Vec3V V3SetY(const Vec3V v, const FloatV f)
{
    VECMATHAOS_ASSERT(isValidVec3V(v));
    VECMATHAOS_ASSERT(isValidFloatV(f));
    return V3Sel(BTFTT(),v,vcombine_f32(f, f));
}

NV_FORCE_INLINE Vec3V V3SetZ(const Vec3V v, const FloatV f)
{
    VECMATHAOS_ASSERT(isValidVec3V(v));
    VECMATHAOS_ASSERT(isValidFloatV(f));
    return V3Sel(BTTFT(),v,vcombine_f32(f, f));
}

NV_FORCE_INLINE Vec3V V3ColX(const Vec3V a, const Vec3V b, const Vec3V c)
{
    const float32x2_t aLow = vget_low_f32(a);
    const float32x2_t bLow = vget_low_f32(b);
    const float32x2_t cLow = vget_low_f32(c);
    const float32x2_t zero = vdup_n_f32(0.0f);

    const float32x2x2_t zipL = vzip_f32(aLow, bLow);
    const float32x2x2_t zipH = vzip_f32(cLow, zero);

    return vcombine_f32(zipL.val[0], zipH.val[0]);
}

NV_FORCE_INLINE Vec3V V3ColY(const Vec3V a, const Vec3V b, const Vec3V c)
{
    const float32x2_t aLow = vget_low_f32(a);
    const float32x2_t bLow = vget_low_f32(b);
    const float32x2_t cLow = vget_low_f32(c);
    const float32x2_t zero = vdup_n_f32(0.0f);

    const float32x2x2_t zipL = vzip_f32(aLow, bLow);
    const float32x2x2_t zipH = vzip_f32(cLow, zero);

    return vcombine_f32(zipL.val[1], zipH.val[1]);
}

NV_FORCE_INLINE Vec3V V3ColZ(const Vec3V a, const Vec3V b, const Vec3V c)
{
    const float32x2_t aHi = vget_high_f32(a);
    const float32x2_t bHi = vget_high_f32(b);
    const float32x2_t cHi = vget_high_f32(c);

    const float32x2x2_t zipL = vzip_f32(aHi, bHi);

    return vcombine_f32(zipL.val[0], cHi);
}

NV_FORCE_INLINE Vec3V V3Zero()
{
    return vdupq_n_f32(0.0f);
}

NV_FORCE_INLINE Vec3V V3Eps()
{
    return V3Load(NV_EPS_REAL);
}

NV_FORCE_INLINE Vec3V V3One()
{
    return V3Load(1.0f);
}

NV_FORCE_INLINE Vec3V V3Neg(const Vec3V f)
{
    VECMATHAOS_ASSERT(isValidVec3V(f));
    return vnegq_f32(f);
}

NV_FORCE_INLINE Vec3V V3Add(const Vec3V a, const Vec3V b)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    return vaddq_f32(a, b);
}

NV_FORCE_INLINE Vec3V V3Add(const Vec3V a, const FloatV b)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidVec3V(b));
    return vaddq_f32(a, Vec3V_From_FloatV(b));
}

NV_FORCE_INLINE Vec3V V3Sub(const Vec3V a, const Vec3V b)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidVec3V(b));
    return vsubq_f32(a, b);
}

NV_FORCE_INLINE Vec3V V3Sub(const Vec3V a, const FloatV b)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    return vsubq_f32(a, Vec3V_From_FloatV(b));
}

NV_FORCE_INLINE Vec3V V3Scale(const Vec3V a, const FloatV b)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    return vmulq_lane_f32(a, b, 0);
}

NV_FORCE_INLINE Vec3V V3Mul(const Vec3V a, const Vec3V b)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidVec3V(b));
    return vmulq_f32(a, b);
}

NV_FORCE_INLINE Vec3V V3ScaleInv(const Vec3V a, const FloatV b)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));

    float32x2_t invB = VRECIP(b);
    return  vsetq_lane_f32(0.0f, vmulq_lane_f32(a, invB, 0), 3);
}

NV_FORCE_INLINE Vec3V V3Div(const Vec3V a, const Vec3V b)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidVec3V(b));

    float32x4_t invB = VRECIPQ(b);
    invB = vsetq_lane_f32(0.0f, invB, 3);
    return vmulq_f32(a, invB);
}

NV_FORCE_INLINE Vec3V V3ScaleInvFast(const Vec3V a, const FloatV b)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));

    const float32x2_t invB = VRECIPE(b);
    return vmulq_lane_f32(a, invB, 0);
}

NV_FORCE_INLINE Vec3V V3DivFast(const Vec3V a, const Vec3V b)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidVec3V(b));

    float32x4_t invB = VRECIPEQ(b);
    invB = vsetq_lane_f32(0.0f, invB, 3);
    return vmulq_f32(a, invB);
}

NV_FORCE_INLINE Vec3V V3Recip(const Vec3V a)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));

    const float32x4_t recipA = VRECIPQ(a);
    return vsetq_lane_f32(0.0f, recipA, 3);
}

NV_FORCE_INLINE Vec3V V3RecipFast(const Vec3V a)
{
    const float32x4_t recipA = VRECIPEQ(a);
    return vsetq_lane_f32(0.0f, recipA, 3);
}

NV_FORCE_INLINE Vec3V V3Rsqrt(const Vec3V a)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));

    const float32x4_t rSqrA = VRECIPSQRTQ(a);
    return vsetq_lane_f32(0.0f, rSqrA, 3);
}

NV_FORCE_INLINE Vec3V V3RsqrtFast(const Vec3V a)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));

    const float32x4_t rSqrA = VRECIPSQRTEQ(a);
    return vsetq_lane_f32(0.0f, rSqrA, 3);
}

NV_FORCE_INLINE Vec3V V3ScaleAdd(const Vec3V a, const FloatV b, const Vec3V c)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    VECMATHAOS_ASSERT(isValidVec3V(c));

    return vmlaq_lane_f32(c, a, b, 0);
}

NV_FORCE_INLINE Vec3V V3NegScaleSub(const Vec3V a, const FloatV b, const Vec3V c)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidFloatV(b));
    VECMATHAOS_ASSERT(isValidVec3V(c));

    return vmlsq_lane_f32(c, a, b, 0);
}

NV_FORCE_INLINE Vec3V V3MulAdd(const Vec3V a, const Vec3V b, const Vec3V c)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidVec3V(b));
    VECMATHAOS_ASSERT(isValidVec3V(c));

    return vmlaq_f32(c, a, b);
}

NV_FORCE_INLINE Vec3V V3NegMulSub(const Vec3V a, const Vec3V b, const Vec3V c)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidVec3V(b));
    VECMATHAOS_ASSERT(isValidVec3V(c));

    return vmlsq_f32(c, a, b);
}

NV_FORCE_INLINE Vec3V V3Abs(const Vec3V a)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));

    return vabsq_f32(a);
}

NV_FORCE_INLINE FloatV V3Dot(const Vec3V a, const Vec3V b)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidVec3V(b));
    NV_ASSERT(isValidVec3V(a));
    NV_ASSERT(isValidVec3V(b));

    //  const uint32x2_t mask = {0xffffFFFF, 0x0};
    const float32x4_t tmp = vmulq_f32(a, b);

    const float32x2_t low = vget_low_f32(tmp);
    const float32x2_t high = vget_high_f32(tmp);
    //  const float32x2_t high = vreinterpret_f32_u32(vand_u32(vreinterpret_u32_f32(high_), mask));

    const float32x2_t sumTmp = vpadd_f32(low, high); // = {0+z, x+y}
    const float32x2_t sum0ZYX = vpadd_f32(sumTmp, sumTmp); // = {x+y+z, x+y+z}

    return sum0ZYX;
}

NV_FORCE_INLINE Vec3V V3Cross(const Vec3V a, const Vec3V b)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidVec3V(b));

#if NV_WINRT
    const uint32x2_t TF = { 0x00000000ffffFFFFULL };
#else
    const uint32x2_t TF = {0xffffFFFF, 0x0};
#endif
    const float32x2_t ay_ax = vget_low_f32(a); // d2
    const float32x2_t aw_az = vget_high_f32(a); // d3
    const float32x2_t by_bx = vget_low_f32(b); // d4
    const float32x2_t bw_bz = vget_high_f32(b); // d5
    // Hi, Lo
    const float32x2_t bz_by = vext_f32(by_bx, bw_bz, 1);                                    // bz, by
    const float32x2_t az_ay = vext_f32(ay_ax, aw_az, 1);                                    // az, ay

    const float32x2_t azbx = vmul_f32(aw_az, by_bx);                                        // 0, az*bx
    const float32x2_t aybz_axby = vmul_f32(ay_ax, bz_by);                                   // ay*bz, ax*by

    const float32x2_t azbxSUBaxbz = vmls_f32(azbx, bw_bz, ay_ax);                           // 0, az*bx-ax*bz
    const float32x2_t aybzSUBazby_axbySUBaybx = vmls_f32(aybz_axby, by_bx, az_ay);          // ay*bz-az*by, ax*by-ay*bx

    const float32x2_t retLow = vext_f32(aybzSUBazby_axbySUBaybx, azbxSUBaxbz, 1);           // az*bx-ax*bz, ay*bz-az*by
    const uint32x2_t retHigh = vand_u32(TF, vreinterpret_u32_f32(aybzSUBazby_axbySUBaybx)); // 0, ax*by-ay*bx

    return vcombine_f32(retLow, vreinterpret_f32_u32(retHigh));

}

NV_FORCE_INLINE VecCrossV V3PrepareCross(const Vec3V a)
{
    return a;
}


NV_FORCE_INLINE FloatV V3Length(const Vec3V a)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    NV_ASSERT(isValidVec3V(a));

    //  const uint32x2_t mask = {0xffffFFFF, 0x0};

    const float32x4_t tmp = vmulq_f32(a, a);
    const float32x2_t low = vget_low_f32(tmp);
    const float32x2_t high = vget_high_f32(tmp);
    //  const float32x2_t high = vreinterpret_f32_u32(vand_u32(vreinterpret_u32_f32(high_), mask));

    const float32x2_t sumTmp = vpadd_f32(low, high);        // = {0+z, x+y}
    const float32x2_t sum0ZYX = vpadd_f32(sumTmp, sumTmp);  // = {x+y+z, x+y+z}

    const float32x2_t len = vmul_f32(VRECIPSQRTE(sum0ZYX), sum0ZYX);

    return len;
}

NV_FORCE_INLINE FloatV V3LengthSq(const Vec3V a)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    return V3Dot(a,a);
}

NV_FORCE_INLINE Vec3V V3Normalize(const Vec3V a)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    return V3ScaleInv(a, V3Length(a));
}

NV_FORCE_INLINE Vec3V V3NormalizeFast(const Vec3V a)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    return V3Scale(a, VRECIPSQRTE(V3Dot(a,a)));
}

NV_FORCE_INLINE Vec3V V3NormalizeSafe(const Vec3V a)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    const FloatV zero = vdup_n_f32(0.0f);
    const FloatV length = V3Length(a);
    const uint32x4_t isGreaterThanZero = FIsGrtr(length, zero);
    return V3Sel(isGreaterThanZero, V3ScaleInv(a, length),  vdupq_lane_f32(zero, 0));
}

NV_FORCE_INLINE Vec3V V3Sel(const BoolV c, const Vec3V a, const Vec3V b)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidVec3V(b));
    return vbslq_f32(c, a, b);
}

NV_FORCE_INLINE BoolV V3IsGrtr(const Vec3V a, const Vec3V b)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidVec3V(b));
    return vcgtq_f32(a, b);
}

NV_FORCE_INLINE BoolV V3IsGrtrOrEq(const Vec3V a, const Vec3V b)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidVec3V(b));
    return vcgeq_f32(a, b);
}

NV_FORCE_INLINE BoolV V3IsEq(const Vec3V a, const Vec3V b)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidVec3V(b));
    return vceqq_f32(a, b);
}

NV_FORCE_INLINE Vec3V V3Max(const Vec3V a, const Vec3V b)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidVec3V(b));
    return vmaxq_f32(a, b);
}

NV_FORCE_INLINE Vec3V V3Min(const Vec3V a, const Vec3V b)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidVec3V(b));
    return vminq_f32(a, b);
}

//Extract the maximum value from a
NV_FORCE_INLINE FloatV V3ExtractMax(const Vec3V a)
{
    const float32x2_t low = vget_low_f32(a);
    const float32x2_t high = vget_high_f32(a);

    const float32x2_t   zz = vdup_lane_f32(high, 0);
    const float32x2_t max0 = vpmax_f32(zz, low);
    const float32x2_t max1 = vpmax_f32(max0, max0);

    return max1;
}

//Extract the maximum value from a
NV_FORCE_INLINE FloatV V3ExtractMin(const Vec3V a)
{
    const float32x2_t low = vget_low_f32(a);
    const float32x2_t high = vget_high_f32(a);

    const float32x2_t   zz = vdup_lane_f32(high, 0);
    const float32x2_t min0 = vpmin_f32(zz, low);
    const float32x2_t min1 = vpmin_f32(min0, min0);

    return min1;
}

//return (a >= 0.0f) ? 1.0f : -1.0f;
NV_FORCE_INLINE Vec3V V3Sign(const Vec3V a)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    const Vec3V zero = V3Zero();
    const Vec3V one = V3One();
    const Vec3V none = V3Neg(one);
    return V3Sel(V3IsGrtrOrEq(a, zero), one,  none);
}

NV_FORCE_INLINE Vec3V V3Clamp(const Vec3V a, const Vec3V minV, const Vec3V maxV)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    VECMATHAOS_ASSERT(isValidVec3V(minV));
    VECMATHAOS_ASSERT(isValidVec3V(maxV));
    return V3Max(V3Min(a,maxV),minV);
}

NV_FORCE_INLINE uint32_t V3AllGrtr(const Vec3V a, const Vec3V b)
{
    return internalUnitNeonSimd::BAllTrue3_R(V4IsGrtr(a, b));
}


NV_FORCE_INLINE uint32_t V3AllGrtrOrEq(const Vec3V a, const Vec3V b)
{
    return internalUnitNeonSimd::BAllTrue3_R(V4IsGrtrOrEq(a, b));
}

NV_FORCE_INLINE uint32_t V3AllEq(const Vec3V a, const Vec3V b)
{
    return internalUnitNeonSimd::BAllTrue3_R(V4IsEq(a, b));
}

NV_FORCE_INLINE Vec3V V3Round(const Vec3V a)
{
    //truncate(a + (0.5f - sign(a)))
    const Vec3V     half = V3Load(0.5f);
    const float32x4_t sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(a), 31)));
    const Vec3V aPlusHalf = V3Add(a, half);
    const Vec3V aRound = V3Sub(aPlusHalf, sign);
    return vcvtq_f32_s32(vcvtq_s32_f32(aRound));
}


NV_FORCE_INLINE Vec3V V3Sin(const Vec3V a)
{
    //Vec4V V1, V2, V3, V5, V7, V9, V11, V13, V15, V17, V19, V21, V23;
    //Vec4V S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11;
    Vec3V Result;

    // Modulo the range of the given angles such that -XM_2PI <= Angles < XM_2PI
    const Vec3V recipTwoPi = V4LoadA(g_NVReciprocalTwoPi.f);
    const Vec3V twoPi = V4LoadA(g_NVTwoPi.f);
    const Vec3V tmp = V3Mul(a, recipTwoPi);
    const Vec3V b = V3Round(tmp);
    const Vec3V V1 = V3NegMulSub(twoPi, b, a);

    // sin(V) ~= V - V^3 / 3! + V^5 / 5! - V^7 / 7! + V^9 / 9! - V^11 / 11! + V^13 / 13! -
    //           V^15 / 15! + V^17 / 17! - V^19 / 19! + V^21 / 21! - V^23 / 23! (for -PI <= V < PI)
    const Vec3V V2  = V3Mul(V1, V1);
    const Vec3V V3  = V3Mul(V2, V1);
    const Vec3V V5  = V3Mul(V3, V2);
    const Vec3V V7  = V3Mul(V5, V2);
    const Vec3V V9  = V3Mul(V7, V2);
    const Vec3V V11 = V3Mul(V9, V2);
    const Vec3V V13 = V3Mul(V11, V2);
    const Vec3V V15 = V3Mul(V13, V2);
    const Vec3V V17 = V3Mul(V15, V2);
    const Vec3V V19 = V3Mul(V17, V2);
    const Vec3V V21 = V3Mul(V19, V2);
    const Vec3V V23 = V3Mul(V21, V2);

    const Vec4V sinCoefficients0 = V4LoadA(g_NVSinCoefficients0.f);
    const Vec4V sinCoefficients1 = V4LoadA(g_NVSinCoefficients1.f);
    const Vec4V sinCoefficients2 = V4LoadA(g_NVSinCoefficients2.f);

    const FloatV S1  = V4GetY(sinCoefficients0);
    const FloatV S2  = V4GetZ(sinCoefficients0);
    const FloatV S3  = V4GetW(sinCoefficients0);
    const FloatV S4  = V4GetX(sinCoefficients1);
    const FloatV S5  = V4GetY(sinCoefficients1);
    const FloatV S6  = V4GetZ(sinCoefficients1);
    const FloatV S7  = V4GetW(sinCoefficients1);
    const FloatV S8  = V4GetX(sinCoefficients2);
    const FloatV S9  = V4GetY(sinCoefficients2);
    const FloatV S10 = V4GetZ(sinCoefficients2);
    const FloatV S11 = V4GetW(sinCoefficients2);

    Result = V3ScaleAdd(V3,  S1, V1);
    Result = V3ScaleAdd(V5,  S2, Result);
    Result = V3ScaleAdd(V7,  S3, Result);
    Result = V3ScaleAdd(V9,  S4, Result);
    Result = V3ScaleAdd(V11, S5, Result);
    Result = V3ScaleAdd(V13, S6, Result);
    Result = V3ScaleAdd(V15, S7, Result);
    Result = V3ScaleAdd(V17, S8, Result);
    Result = V3ScaleAdd(V19, S9, Result);
    Result = V3ScaleAdd(V21, S10,Result);
    Result = V3ScaleAdd(V23, S11,Result);

    return Result;

}

NV_FORCE_INLINE Vec3V V3Cos(const Vec3V a)
{
    //XMVECTOR V1, V2, V4, V6, V8, V10, V12, V14, V16, V18, V20, V22;
    //XMVECTOR C1, C2, C3, C4, C5, C6, C7, C8, C9, C10, C11;
    Vec3V Result;

    // Modulo the range of the given angles such that -XM_2PI <= Angles < XM_2PI
    const Vec3V recipTwoPi = V4LoadA(g_NVReciprocalTwoPi.f);
    const Vec3V twoPi = V4LoadA(g_NVTwoPi.f);
    const Vec3V tmp = V3Mul(a, recipTwoPi);
    const Vec3V b = V3Round(tmp);
    const Vec3V V1 = V3NegMulSub(twoPi, b, a);

    // cos(V) ~= 1 - V^2 / 2! + V^4 / 4! - V^6 / 6! + V^8 / 8! - V^10 / 10! + V^12 / 12! -
    //           V^14 / 14! + V^16 / 16! - V^18 / 18! + V^20 / 20! - V^22 / 22! (for -PI <= V < PI)
    const Vec3V V2 = V3Mul(V1, V1);
    const Vec3V V4 = V3Mul(V2, V2);
    const Vec3V V6 = V3Mul(V4, V2);
    const Vec3V V8 = V3Mul(V4, V4);
    const Vec3V V10 = V3Mul(V6, V4);
    const Vec3V V12 = V3Mul(V6, V6);
    const Vec3V V14 = V3Mul(V8, V6);
    const Vec3V V16 = V3Mul(V8, V8);
    const Vec3V V18 = V3Mul(V10, V8);
    const Vec3V V20 = V3Mul(V10, V10);
    const Vec3V V22 = V3Mul(V12, V10);

    const Vec4V cosCoefficients0 = V4LoadA(g_NVCosCoefficients0.f);
    const Vec4V cosCoefficients1 = V4LoadA(g_NVCosCoefficients1.f);
    const Vec4V cosCoefficients2 = V4LoadA(g_NVCosCoefficients2.f);

    const FloatV C1  = V4GetY(cosCoefficients0);
    const FloatV C2  = V4GetZ(cosCoefficients0);
    const FloatV C3  = V4GetW(cosCoefficients0);
    const FloatV C4  = V4GetX(cosCoefficients1);
    const FloatV C5  = V4GetY(cosCoefficients1);
    const FloatV C6  = V4GetZ(cosCoefficients1);
    const FloatV C7  = V4GetW(cosCoefficients1);
    const FloatV C8  = V4GetX(cosCoefficients2);
    const FloatV C9  = V4GetY(cosCoefficients2);
    const FloatV C10 = V4GetZ(cosCoefficients2);
    const FloatV C11 = V4GetW(cosCoefficients2);

    Result = V3ScaleAdd(V2,  C1, V4One());
    Result = V3ScaleAdd(V4,  C2, Result);
    Result = V3ScaleAdd(V6,  C3, Result);
    Result = V3ScaleAdd(V8,  C4, Result);
    Result = V3ScaleAdd(V10, C5, Result);
    Result = V3ScaleAdd(V12, C6, Result);
    Result = V3ScaleAdd(V14, C7, Result);
    Result = V3ScaleAdd(V16, C8, Result);
    Result = V3ScaleAdd(V18, C9, Result);
    Result = V3ScaleAdd(V20, C10,Result);
    Result = V3ScaleAdd(V22, C11,Result);

    return Result;
}

NV_FORCE_INLINE Vec3V V3PermYZZ(const Vec3V a)
{
    const float32x2_t xy = vget_low_f32(a);
    const float32x2_t zw = vget_high_f32(a);
    const float32x2_t yz = vext_f32(xy, zw, 1);
    return vcombine_f32(yz, zw);
}

NV_FORCE_INLINE Vec3V V3PermXYX(const Vec3V a)
{
#if NV_WINRT
    const uint32x2_t mask = { 0x00000000ffffFFFFULL };
#else
    const uint32x2_t mask = {0xffffFFFF, 0x0};
#endif

    const uint32x2_t xy = vget_low_u32(vreinterpretq_u32_f32(a));
    const uint32x2_t xw = vand_u32(xy, mask);
    return vreinterpretq_f32_u32(vcombine_u32(xy, xw));
}

NV_FORCE_INLINE Vec3V V3PermYZX(const Vec3V a)
{
#if NV_WINRT
    const uint32x2_t mask = { 0x00000000ffffFFFFULL };
#else
    const uint32x2_t mask = {0xffffFFFF, 0x0};
#endif

    const uint32x2_t xy = vget_low_u32(vreinterpretq_u32_f32(a));
    const uint32x2_t zw = vget_high_u32(vreinterpretq_u32_f32(a));
    const uint32x2_t yz = vext_u32(xy, zw, 1);
    const uint32x2_t xw = vand_u32(xy, mask);
    return vreinterpretq_f32_u32(vcombine_u32(yz, xw));
}

NV_FORCE_INLINE Vec3V V3PermZXY(const Vec3V a)
{
    const uint32x2_t xy = vget_low_u32(vreinterpretq_u32_f32(a));
    const uint32x2_t zw = vget_high_u32(vreinterpretq_u32_f32(a));
    const uint32x2_t wz = vrev64_u32(zw);

    const uint32x2_t zx = vext_u32(wz, xy, 1);
    const uint32x2_t yw = vext_u32(xy, wz, 1);

    return vreinterpretq_f32_u32(vcombine_u32(zx, yw));
}

NV_FORCE_INLINE Vec3V V3PermZZY(const Vec3V a)
{
    const uint32x2_t xy = vget_low_u32(vreinterpretq_u32_f32(a));
    const uint32x2_t zw = vget_high_u32(vreinterpretq_u32_f32(a));

    const uint32x2_t wz = vrev64_u32(zw);
    const uint32x2_t yw = vext_u32(xy, wz, 1);
    const uint32x2_t zz = vdup_lane_u32(wz, 1);

    return vreinterpretq_f32_u32(vcombine_u32(zz, yw));
}

NV_FORCE_INLINE Vec3V V3PermYXX(const Vec3V a)
{
#if NV_WINRT
    const uint32x2_t mask = { 0x00000000ffffFFFFULL };
#else
    const uint32x2_t mask = {0xffffFFFF, 0x0};
#endif

    const uint32x2_t xy = vget_low_u32(vreinterpretq_u32_f32(a));
    const uint32x2_t yx = vrev64_u32(xy);
    const uint32x2_t xw = vand_u32(xy, mask);
    return vreinterpretq_f32_u32(vcombine_u32(yx, xw));
}

NV_FORCE_INLINE Vec3V V3Perm_Zero_1Z_0Y(const Vec3V v0, const Vec3V v1)
{
    const uint32x2_t xy = vget_low_u32(vreinterpretq_u32_f32(v0));
    const uint32x2_t zw = vget_high_u32(vreinterpretq_u32_f32(v1));
    const uint32x2_t wz = vrev64_u32(zw);
    const uint32x2_t yw = vext_u32(xy, wz, 1);

    return vreinterpretq_f32_u32(vcombine_u32(wz, yw));
}

NV_FORCE_INLINE Vec3V V3Perm_0Z_Zero_1X(const Vec3V v0, const Vec3V v1)
{
#if NV_WINRT
    const uint32x2_t mask = { 0x00000000ffffFFFFULL };
#else
    const uint32x2_t mask = {0xffffFFFF, 0x0};
#endif

    const uint32x2_t zw = vget_high_u32(vreinterpretq_u32_f32(v0));
    const uint32x2_t xy = vget_low_u32(vreinterpretq_u32_f32(v1));
    const uint32x2_t xw = vand_u32(xy, mask);

    return vreinterpretq_f32_u32(vcombine_u32(zw, xw));
}

NV_FORCE_INLINE Vec3V V3Perm_1Y_0X_Zero(const Vec3V v0, const Vec3V v1)
{
    const uint32x2_t axy = vget_low_u32(vreinterpretq_u32_f32(v0));
    const uint32x2_t bxy = vget_low_u32(vreinterpretq_u32_f32(v1));
    const uint32x2_t byax = vext_u32(bxy, axy, 1);
    const uint32x2_t ww = vdup_n_u32(0);

    return vreinterpretq_f32_u32(vcombine_u32(byax, ww));
}

NV_FORCE_INLINE FloatV V3SumElems(const Vec3V a)
{
    VECMATHAOS_ASSERT(isValidVec3V(a));
    NV_ASSERT(isValidVec3V(a));

    //const uint32x2_t mask = {0xffffFFFF, 0x0};

    const float32x2_t low = vget_low_f32(a);
    const float32x2_t high = vget_high_f32(a);
    //const float32x2_t high = vreinterpret_f32_u32(vand_u32(vreinterpret_u32_f32(high_), mask));

    const float32x2_t sumTmp = vpadd_f32(low, high); // = {0+z, x+y}
    const float32x2_t sum0ZYX = vpadd_f32(sumTmp, sumTmp); // = {x+y+z, x+y+z}

    return sum0ZYX;
}

NV_FORCE_INLINE uint32_t V3OutOfBounds(const Vec3V a, const Vec3V min, const Vec3V max)
{
    const BoolV c = BOr(V3IsGrtr(a, max), V3IsGrtr(min, a));
    return internalUnitNeonSimd::BAnyTrue3_R(c);
}

NV_FORCE_INLINE uint32_t V3InBounds(const Vec3V a, const Vec3V min, const Vec3V max)
{
    const BoolV c = BAnd(V3IsGrtrOrEq(a, min), V3IsGrtrOrEq(max, a));
    return internalUnitNeonSimd::BAllTrue4_R(c);
}

NV_FORCE_INLINE uint32_t V3OutOfBounds(const Vec3V a, const Vec3V bounds)
{
#if NV_WINRT
    const uint32x4_t greater = vacgtq_f32(a, bounds);
#else
    const uint32x4_t greater = vcagtq_f32(a, bounds);
#endif
    return internalUnitNeonSimd::BAnyTrue3_R(greater);
}

NV_FORCE_INLINE uint32_t V3InBounds(const Vec3V a, const Vec3V bounds)
{
#if NV_WINRT
    const uint32x4_t geq = vacgeq_f32(bounds, a);
#else
    const uint32x4_t geq = vcageq_f32(bounds, a);
#endif
    return internalUnitNeonSimd::BAllTrue4_R(geq);
}




//////////////////////////////////
//VEC4V
//////////////////////////////////

NV_FORCE_INLINE Vec4V V4Splat(const FloatV f)
{
    VECMATHAOS_ASSERT(isValidFloatV(f));
    return vcombine_f32(f, f);
}

NV_FORCE_INLINE Vec4V V4Merge(const FloatV* const floatVArray)
{
    VECMATHAOS_ASSERT(isValidFloatV(floatVArray[0]));
    VECMATHAOS_ASSERT(isValidFloatV(floatVArray[1]));
    VECMATHAOS_ASSERT(isValidFloatV(floatVArray[2]));
    VECMATHAOS_ASSERT(isValidFloatV(floatVArray[3]));

    const uint32x2_t xLow = vreinterpret_u32_f32(floatVArray[0]);
    const uint32x2_t yLow = vreinterpret_u32_f32(floatVArray[1]);
    const uint32x2_t zLow = vreinterpret_u32_f32(floatVArray[2]);
    const uint32x2_t wLow = vreinterpret_u32_f32(floatVArray[3]);

    const uint32x2_t dLow = vext_u32(xLow, yLow, 1);
    const uint32x2_t dHigh = vext_u32(zLow, wLow, 1);

    return vreinterpretq_f32_u32(vcombine_u32(dLow, dHigh));
}


NV_FORCE_INLINE Vec4V V4Merge(const FloatVArg x, const FloatVArg y, const FloatVArg z, const FloatVArg w)
{
    VECMATHAOS_ASSERT(isValidFloatV(x));
    VECMATHAOS_ASSERT(isValidFloatV(y));
    VECMATHAOS_ASSERT(isValidFloatV(z));
    VECMATHAOS_ASSERT(isValidFloatV(w));

    const uint32x2_t xLow = vreinterpret_u32_f32(x);
    const uint32x2_t yLow = vreinterpret_u32_f32(y);
    const uint32x2_t zLow = vreinterpret_u32_f32(z);
    const uint32x2_t wLow = vreinterpret_u32_f32(w);

    const uint32x2_t dLow = vext_u32(xLow, yLow, 1);
    const uint32x2_t dHigh = vext_u32(zLow, wLow, 1);

    return vreinterpretq_f32_u32(vcombine_u32(dLow, dHigh));
}

NV_FORCE_INLINE Vec4V V4MergeW(const Vec4VArg x, const Vec4VArg y, const Vec4VArg z, const Vec4VArg w)
{
    const float32x2_t xx = vget_high_f32(x);
    const float32x2_t yy = vget_high_f32(y);
    const float32x2_t zz = vget_high_f32(z);
    const float32x2_t ww = vget_high_f32(w);

    const float32x2x2_t zipL = vzip_f32(xx, yy);
    const float32x2x2_t zipH = vzip_f32(zz, ww);

    return vcombine_f32(zipL.val[1], zipH.val[1]);
}


NV_FORCE_INLINE Vec4V V4MergeZ(const Vec4VArg x, const Vec4VArg y, const Vec4VArg z, const Vec4VArg w)
{
    const float32x2_t xx = vget_high_f32(x);
    const float32x2_t yy = vget_high_f32(y);
    const float32x2_t zz = vget_high_f32(z);
    const float32x2_t ww = vget_high_f32(w);

    const float32x2x2_t zipL = vzip_f32(xx, yy);
    const float32x2x2_t zipH = vzip_f32(zz, ww);

    return vcombine_f32(zipL.val[0], zipH.val[0]);
}

NV_FORCE_INLINE Vec4V V4MergeY(const Vec4VArg x, const Vec4VArg y, const Vec4VArg z, const Vec4VArg w)
{
    const float32x2_t xx = vget_low_f32(x);
    const float32x2_t yy = vget_low_f32(y);
    const float32x2_t zz = vget_low_f32(z);
    const float32x2_t ww = vget_low_f32(w);

    const float32x2x2_t zipL = vzip_f32(xx, yy);
    const float32x2x2_t zipH = vzip_f32(zz, ww);

    return vcombine_f32(zipL.val[1], zipH.val[1]);
}

NV_FORCE_INLINE Vec4V V4MergeX(const Vec4VArg x, const Vec4VArg y, const Vec4VArg z, const Vec4VArg w)
{
    const float32x2_t xx = vget_low_f32(x);
    const float32x2_t yy = vget_low_f32(y);
    const float32x2_t zz = vget_low_f32(z);
    const float32x2_t ww = vget_low_f32(w);

    const float32x2x2_t zipL = vzip_f32(xx, yy);
    const float32x2x2_t zipH = vzip_f32(zz, ww);

    return vcombine_f32(zipL.val[0], zipH.val[0]);
}

NV_FORCE_INLINE Vec4V V4UnpackXY(const Vec4VArg a, const Vec4VArg b)
{
    return vzipq_f32(a, b).val[0];
}

NV_FORCE_INLINE Vec4V V4UnpackZW(const Vec4VArg a, const Vec4VArg b)
{
    return vzipq_f32(a, b).val[1];
}

NV_FORCE_INLINE Vec4V V4UnitW()
{
    const float32x2_t zeros = vreinterpret_f32_u32(vmov_n_u32(0));
    const float32x2_t ones = vmov_n_f32(1.0f);
    const float32x2_t zo = vext_f32(zeros, ones, 1);
    return vcombine_f32(zeros, zo);
}

NV_FORCE_INLINE Vec4V V4UnitX()
{
    const float32x2_t zeros = vreinterpret_f32_u32(vmov_n_u32(0));
    const float32x2_t ones = vmov_n_f32(1.0f);
    const float32x2_t oz = vext_f32(ones, zeros, 1);
    return vcombine_f32(oz, zeros);
}

NV_FORCE_INLINE Vec4V V4UnitY()
{
    const float32x2_t zeros = vreinterpret_f32_u32(vmov_n_u32(0));
    const float32x2_t ones = vmov_n_f32(1.0f);
    const float32x2_t zo = vext_f32(zeros, ones, 1);
    return vcombine_f32(zo, zeros);
}

NV_FORCE_INLINE Vec4V V4UnitZ()
{
    const float32x2_t zeros = vreinterpret_f32_u32(vmov_n_u32(0));
    const float32x2_t ones = vmov_n_f32(1.0f);
    const float32x2_t oz = vext_f32(ones, zeros, 1);
    return vcombine_f32(zeros, oz);
}

NV_FORCE_INLINE FloatV V4GetW(const Vec4V f)
{
    const float32x2_t fhigh = vget_high_f32(f);
    return vdup_lane_f32(fhigh, 1);
}

NV_FORCE_INLINE FloatV V4GetX(const Vec4V f)
{
    const float32x2_t fLow = vget_low_f32(f);
    return vdup_lane_f32(fLow, 0);
}

NV_FORCE_INLINE FloatV V4GetY(const Vec4V f)
{
    const float32x2_t fLow = vget_low_f32(f);
    return vdup_lane_f32(fLow, 1);
}

NV_FORCE_INLINE FloatV V4GetZ(const Vec4V f)
{
    const float32x2_t fhigh = vget_high_f32(f);
    return vdup_lane_f32(fhigh, 0);
}

NV_FORCE_INLINE Vec4V V4SetW(const Vec4V v, const FloatV f)
{
    VECMATHAOS_ASSERT(isValidFloatV(f));
    return V4Sel(BTTTF(), v, vcombine_f32(f, f));
}

NV_FORCE_INLINE Vec4V V4SetX(const Vec4V v, const FloatV f)
{
    VECMATHAOS_ASSERT(isValidFloatV(f));
    return V4Sel(BFTTT(), v, vcombine_f32(f, f));
}

NV_FORCE_INLINE Vec4V V4SetY(const Vec4V v, const FloatV f)
{
    VECMATHAOS_ASSERT(isValidFloatV(f));
    return V4Sel(BTFTT(), v, vcombine_f32(f, f));
}

NV_FORCE_INLINE Vec4V V4SetZ(const Vec4V v, const FloatV f)
{
    VECMATHAOS_ASSERT(isValidVec3V(v));
    VECMATHAOS_ASSERT(isValidFloatV(f));
    return V4Sel(BTTFT(), v, vcombine_f32(f, f));
}

NV_FORCE_INLINE Vec4V V4ClearW(const Vec4V v)
{
    VECMATHAOS_ASSERT(isValidVec3V(v));
    return V4Sel(BTTTF(), v, V4Zero());
}

NV_FORCE_INLINE Vec4V V4Perm_YXWZ(const Vec4V a)
{
    const float32x2_t xy = vget_low_f32(a);
    const float32x2_t zw = vget_high_f32(a);
    const float32x2_t yx = vext_f32(xy, xy, 1);
    const float32x2_t wz = vext_f32(zw, zw, 1);
    return vcombine_f32(yx, wz);
}

NV_FORCE_INLINE Vec4V V4Perm_XZXZ(const Vec4V a)
{
    const float32x2_t xy = vget_low_f32(a);
    const float32x2_t zw = vget_high_f32(a);
    const float32x2x2_t xzyw = vzip_f32(xy, zw);
    return vcombine_f32(xzyw.val[0], xzyw.val[0]);
}

NV_FORCE_INLINE Vec4V V4Perm_YWYW(const Vec4V a)
{
    const float32x2_t xy = vget_low_f32(a);
    const float32x2_t zw = vget_high_f32(a);
    const float32x2x2_t xzyw = vzip_f32(xy, zw);
    return vcombine_f32(xzyw.val[1], xzyw.val[1]);
}

template<uint8_t E0, uint8_t E1, uint8_t E2, uint8_t E3> NV_FORCE_INLINE Vec4V V4Perm(const Vec4V V)
{
    static const uint32_t ControlElement[ 4 ] =
    {
#if 1
        0x03020100, // XM_SWIZZLE_X
        0x07060504, // XM_SWIZZLE_Y
        0x0B0A0908, // XM_SWIZZLE_Z
        0x0F0E0D0C, // XM_SWIZZLE_W
#else
        0x00010203, // XM_SWIZZLE_X
        0x04050607, // XM_SWIZZLE_Y
        0x08090A0B, // XM_SWIZZLE_Z
        0x0C0D0E0F, // XM_SWIZZLE_W
#endif
    };

    uint8x8x2_t tbl;
    tbl.val[0] = vreinterpret_u8_f32(vget_low_f32(V));
    tbl.val[1] = vreinterpret_u8_f32(vget_high_f32(V));

    uint8x8_t idx = vcreate_u8( ((uint64_t)ControlElement[E0]) | (((uint64_t)ControlElement[E1]) << 32) );
    const uint8x8_t rL = vtbl2_u8( tbl, idx );
    idx = vcreate_u8( ((uint64_t)ControlElement[E2]) | (((uint64_t)ControlElement[E3]) << 32) );
    const uint8x8_t rH = vtbl2_u8( tbl, idx );

    return vreinterpretq_f32_u8(vcombine_u8( rL, rH ));
}

NV_FORCE_INLINE Vec4V V4Zero()
{
    return vreinterpretq_f32_u32(vmovq_n_u32(0));
}

NV_FORCE_INLINE Vec4V V4One()
{
    return vmovq_n_f32(1.0f);
}

NV_FORCE_INLINE Vec4V V4Eps()
{
    return V4Load(NV_EPS_REAL);
}

NV_FORCE_INLINE Vec4V V4Neg(const Vec4V f)
{
    return vnegq_f32(f);
}

NV_FORCE_INLINE Vec4V V4Add(const Vec4V a, const Vec4V b)
{
    return vaddq_f32(a, b);
}

NV_FORCE_INLINE Vec4V V4Sub(const Vec4V a, const Vec4V b)
{
    return vsubq_f32(a, b);
}

NV_FORCE_INLINE Vec4V V4Scale(const Vec4V a, const FloatV b)
{
    return vmulq_lane_f32(a, b, 0);
}

NV_FORCE_INLINE Vec4V V4Mul(const Vec4V a, const Vec4V b)
{
    return vmulq_f32(a, b);
}

NV_FORCE_INLINE Vec4V V4ScaleInv(const Vec4V a, const FloatV b)
{
    VECMATHAOS_ASSERT(isValidFloatV(b));
    const float32x2_t invB = VRECIP(b);
    return vmulq_lane_f32(a, invB, 0);
}

NV_FORCE_INLINE Vec4V V4Div(const Vec4V a, const Vec4V b)
{
    const float32x4_t invB = VRECIPQ(b);
    return vmulq_f32(a, invB);
}

NV_FORCE_INLINE Vec4V V4ScaleInvFast(const Vec4V a, const FloatV b)
{
    VECMATHAOS_ASSERT(isValidFloatV(b));
    const float32x2_t invB = VRECIPE(b);
    return vmulq_lane_f32(a, invB, 0);
}

NV_FORCE_INLINE Vec4V V4DivFast(const Vec4V a, const Vec4V b)
{
    const float32x4_t invB = VRECIPEQ(b);
    return vmulq_f32(a, invB);
}

NV_FORCE_INLINE Vec4V V4Recip(const Vec4V a)
{
    return VRECIPQ(a);
}

NV_FORCE_INLINE Vec4V V4RecipFast(const Vec4V a)
{
    return VRECIPEQ(a);
}

NV_FORCE_INLINE Vec4V V4Rsqrt(const Vec4V a)
{
    return VRECIPSQRTQ(a);
}

NV_FORCE_INLINE Vec4V V4RsqrtFast(const Vec4V a)
{
    return VRECIPSQRTEQ(a);
}

NV_FORCE_INLINE Vec4V V4Sqrt(const Vec4V a)
{
    return V4Mul(a, VRECIPSQRTQ(a));
}

NV_FORCE_INLINE Vec4V V4ScaleAdd(const Vec4V a, const FloatV b, const Vec4V c)
{
    VECMATHAOS_ASSERT(isValidFloatV(b));
    return vmlaq_lane_f32(c, a, b, 0);
}

NV_FORCE_INLINE Vec4V V4NegScaleSub(const Vec4V a, const FloatV b, const Vec4V c)
{
    VECMATHAOS_ASSERT(isValidFloatV(b));
    return vmlsq_lane_f32(c, a, b, 0);
}

NV_FORCE_INLINE Vec4V V4MulAdd(const Vec4V a, const Vec4V b, const Vec4V c)
{
    return vmlaq_f32(c, a, b);
}

NV_FORCE_INLINE Vec4V V4NegMulSub(const Vec4V a, const Vec4V b, const Vec4V c)
{
    return vmlsq_f32(c, a, b);
}

NV_FORCE_INLINE Vec4V V4Abs(const Vec4V a)
{
    return vabsq_f32(a);
}

NV_FORCE_INLINE FloatV V4SumElements(const Vec4V a)
{
    const Vec4V xy = V4UnpackXY(a, a);  //x,x,y,y
    const Vec4V zw = V4UnpackZW(a, a);  //z,z,w,w
    const Vec4V xz_yw = V4Add(xy, zw);  //x+z,x+z,y+w,y+w
    const FloatV xz = V4GetX(xz_yw);    //x+z
    const FloatV yw = V4GetZ(xz_yw);    //y+w
    return FAdd(xz, yw);                //sum
}

NV_FORCE_INLINE FloatV V4Dot(const Vec4V a, const Vec4V b)
{
    const float32x4_t tmp = vmulq_f32(a, b);
    const float32x2_t low = vget_low_f32(tmp);
    const float32x2_t high = vget_high_f32(tmp);

    const float32x2_t sumTmp = vpadd_f32(low, high); // = {z+w, x+y}
    const float32x2_t sumWZYX = vpadd_f32(sumTmp, sumTmp); // = {x+y+z+w, x+y+z+w}
    return sumWZYX;
}

NV_FORCE_INLINE FloatV V4Length(const Vec4V a)
{
    const float32x4_t tmp = vmulq_f32(a, a);
    const float32x2_t low = vget_low_f32(tmp);
    const float32x2_t high = vget_high_f32(tmp);

    const float32x2_t sumTmp = vpadd_f32(low, high);        // = {0+z, x+y}
    const float32x2_t sumWZYX = vpadd_f32(sumTmp, sumTmp);  // = {x+y+z, x+y+z}
    const float32x2_t len = vmul_f32(VRECIPSQRTE(sumWZYX), sumWZYX);

    return len;
}

NV_FORCE_INLINE FloatV V4LengthSq(const Vec4V a)
{
    return V4Dot(a,a);
}

NV_FORCE_INLINE Vec4V V4Normalize(const Vec4V a)
{
    return V4ScaleInv(a, V4Length(a));
}

NV_FORCE_INLINE Vec4V V4NormalizeFast(const Vec4V a)
{
    return V4Scale(a, FRsqrtFast(V4Dot(a,a)));
}

NV_FORCE_INLINE Vec4V V4NormalizeSafe(const Vec4V a)
{
    const FloatV zero = FZero();
    const FloatV length = V4Length(a);
    const uint32x4_t isGreaterThanZero = FIsGrtr(length, zero);
    return V4Sel(isGreaterThanZero, V4ScaleInv(a, length), vcombine_f32(zero, zero));
}

NV_FORCE_INLINE BoolV V4IsEqU32(const VecU32V a, const VecU32V b)
{
    return vceqq_u32(a, b);
}

NV_FORCE_INLINE Vec4V V4Sel(const BoolV c, const Vec4V a, const Vec4V b)
{
    return vbslq_f32(c, a, b);
}

NV_FORCE_INLINE BoolV V4IsGrtr(const Vec4V a, const Vec4V b)
{
    return vcgtq_f32(a, b);
}

NV_FORCE_INLINE BoolV V4IsGrtrOrEq(const Vec4V a, const Vec4V b)
{
    return vcgeq_f32(a, b);
}

NV_FORCE_INLINE BoolV V4IsEq(const Vec4V a, const Vec4V b)
{
    return vceqq_f32(a, b);
}

NV_FORCE_INLINE Vec4V V4Max(const Vec4V a, const Vec4V b)
{
    return vmaxq_f32(a, b);
}

NV_FORCE_INLINE Vec4V V4Min(const Vec4V a, const Vec4V b)
{
    return vminq_f32(a, b);
}

NV_FORCE_INLINE FloatV V4ExtractMax(const Vec4V a)
{
    const float32x2_t low = vget_low_f32(a);
    const float32x2_t high = vget_high_f32(a);

    const float32x2_t max0 = vpmax_f32(high, low);
    const float32x2_t max1 = vpmax_f32(max0, max0);

    return max1;
}

NV_FORCE_INLINE FloatV V4ExtractMin(const Vec4V a)
{
    const float32x2_t low = vget_low_f32(a);
    const float32x2_t high = vget_high_f32(a);

    const float32x2_t min0 = vpmin_f32(high, low);
    const float32x2_t min1 = vpmin_f32(min0, min0);

    return min1;
}

NV_FORCE_INLINE Vec4V V4Clamp(const Vec4V a, const Vec4V minV, const Vec4V maxV)
{
    return V4Max(V4Min(a,maxV),minV);
}

NV_FORCE_INLINE uint32_t V4AllGrtr(const Vec4V a, const Vec4V b)
{
    return internalUnitNeonSimd::BAllTrue4_R(V4IsGrtr(a, b));
}


NV_FORCE_INLINE uint32_t V4AllGrtrOrEq(const Vec4V a, const Vec4V b)
{
    return internalUnitNeonSimd::BAllTrue4_R(V4IsGrtrOrEq(a, b));
}

NV_FORCE_INLINE uint32_t V4AllEq(const Vec4V a, const Vec4V b)
{
    return internalUnitNeonSimd::BAllTrue4_R(V4IsEq(a, b));
}


NV_FORCE_INLINE Vec4V V4Round(const Vec4V a)
{
    //truncate(a + (0.5f - sign(a)))
    const Vec4V half = V4Load(0.5f);
    const float32x4_t sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(a), 31)));
    const Vec4V aPlusHalf = V4Add(a, half);
    const Vec4V aRound = V4Sub(aPlusHalf, sign);
    return vcvtq_f32_s32(vcvtq_s32_f32(aRound));
}


NV_FORCE_INLINE Vec4V V4Sin(const Vec4V a)
{
    //Vec4V V1, V2, V3, V5, V7, V9, V11, V13, V15, V17, V19, V21, V23;
    //Vec4V S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11;
    Vec4V Result;

    const Vec4V recipTwoPi = V4LoadA(g_NVReciprocalTwoPi.f);
    const Vec4V twoPi = V4LoadA(g_NVTwoPi.f);
    const Vec4V tmp = V4Mul(a, recipTwoPi);
    const Vec4V b = V4Round(tmp);
    const Vec4V V1 = V4NegMulSub(twoPi, b, a);

    // sin(V) ~= V - V^3 / 3! + V^5 / 5! - V^7 / 7! + V^9 / 9! - V^11 / 11! + V^13 / 13! -
    //           V^15 / 15! + V^17 / 17! - V^19 / 19! + V^21 / 21! - V^23 / 23! (for -PI <= V < PI)
    const Vec4V V2  = V4Mul(V1, V1);
    const Vec4V V3  = V4Mul(V2, V1);
    const Vec4V V5  = V4Mul(V3, V2);
    const Vec4V V7  = V4Mul(V5, V2);
    const Vec4V V9  = V4Mul(V7, V2);
    const Vec4V V11 = V4Mul(V9, V2);
    const Vec4V V13 = V4Mul(V11, V2);
    const Vec4V V15 = V4Mul(V13, V2);
    const Vec4V V17 = V4Mul(V15, V2);
    const Vec4V V19 = V4Mul(V17, V2);
    const Vec4V V21 = V4Mul(V19, V2);
    const Vec4V V23 = V4Mul(V21, V2);

    const Vec4V sinCoefficients0 = V4LoadA(g_NVSinCoefficients0.f);
    const Vec4V sinCoefficients1 = V4LoadA(g_NVSinCoefficients1.f);
    const Vec4V sinCoefficients2 = V4LoadA(g_NVSinCoefficients2.f);

    const FloatV S1  = V4GetY(sinCoefficients0);
    const FloatV S2  = V4GetZ(sinCoefficients0);
    const FloatV S3  = V4GetW(sinCoefficients0);
    const FloatV S4  = V4GetX(sinCoefficients1);
    const FloatV S5  = V4GetY(sinCoefficients1);
    const FloatV S6  = V4GetZ(sinCoefficients1);
    const FloatV S7  = V4GetW(sinCoefficients1);
    const FloatV S8  = V4GetX(sinCoefficients2);
    const FloatV S9  = V4GetY(sinCoefficients2);
    const FloatV S10 = V4GetZ(sinCoefficients2);
    const FloatV S11 = V4GetW(sinCoefficients2);

    Result = V4ScaleAdd(V3,  S1, V1);
    Result = V4ScaleAdd(V5,  S2, Result);
    Result = V4ScaleAdd(V7,  S3, Result);
    Result = V4ScaleAdd(V9,  S4, Result);
    Result = V4ScaleAdd(V11, S5, Result);
    Result = V4ScaleAdd(V13, S6, Result);
    Result = V4ScaleAdd(V15, S7, Result);
    Result = V4ScaleAdd(V17, S8, Result);
    Result = V4ScaleAdd(V19, S9, Result);
    Result = V4ScaleAdd(V21, S10,Result);
    Result = V4ScaleAdd(V23, S11,Result);

    return Result;
}

NV_FORCE_INLINE Vec4V V4Cos(const Vec4V a)
{
    //XMVECTOR V1, V2, V4, V6, V8, V10, V12, V14, V16, V18, V20, V22;
    //XMVECTOR C1, C2, C3, C4, C5, C6, C7, C8, C9, C10, C11;
    Vec4V Result;

    const Vec4V recipTwoPi = V4LoadA(g_NVReciprocalTwoPi.f);
    const Vec4V twoPi = V4LoadA(g_NVTwoPi.f);
    const Vec4V tmp = V4Mul(a, recipTwoPi);
    const Vec4V b = V4Round(tmp);
    const Vec4V V1 = V4NegMulSub(twoPi, b, a);

    // cos(V) ~= 1 - V^2 / 2! + V^4 / 4! - V^6 / 6! + V^8 / 8! - V^10 / 10! + V^12 / 12! -
    //           V^14 / 14! + V^16 / 16! - V^18 / 18! + V^20 / 20! - V^22 / 22! (for -PI <= V < PI)
    const Vec4V V2 = V4Mul(V1, V1);
    const Vec4V V4 = V4Mul(V2, V2);
    const Vec4V V6 = V4Mul(V4, V2);
    const Vec4V V8 = V4Mul(V4, V4);
    const Vec4V V10 = V4Mul(V6, V4);
    const Vec4V V12 = V4Mul(V6, V6);
    const Vec4V V14 = V4Mul(V8, V6);
    const Vec4V V16 = V4Mul(V8, V8);
    const Vec4V V18 = V4Mul(V10, V8);
    const Vec4V V20 = V4Mul(V10, V10);
    const Vec4V V22 = V4Mul(V12, V10);

    const Vec4V cosCoefficients0 = V4LoadA(g_NVCosCoefficients0.f);
    const Vec4V cosCoefficients1 = V4LoadA(g_NVCosCoefficients1.f);
    const Vec4V cosCoefficients2 = V4LoadA(g_NVCosCoefficients2.f);

    const FloatV C1  = V4GetY(cosCoefficients0);
    const FloatV C2  = V4GetZ(cosCoefficients0);
    const FloatV C3  = V4GetW(cosCoefficients0);
    const FloatV C4  = V4GetX(cosCoefficients1);
    const FloatV C5  = V4GetY(cosCoefficients1);
    const FloatV C6  = V4GetZ(cosCoefficients1);
    const FloatV C7  = V4GetW(cosCoefficients1);
    const FloatV C8  = V4GetX(cosCoefficients2);
    const FloatV C9  = V4GetY(cosCoefficients2);
    const FloatV C10 = V4GetZ(cosCoefficients2);
    const FloatV C11 = V4GetW(cosCoefficients2);

    Result = V4ScaleAdd(V2,  C1, V4One());
    Result = V4ScaleAdd(V4,  C2, Result);
    Result = V4ScaleAdd(V6,  C3, Result);
    Result = V4ScaleAdd(V8,  C4, Result);
    Result = V4ScaleAdd(V10, C5, Result);
    Result = V4ScaleAdd(V12, C6, Result);
    Result = V4ScaleAdd(V14, C7, Result);
    Result = V4ScaleAdd(V16, C8, Result);
    Result = V4ScaleAdd(V18, C9, Result);
    Result = V4ScaleAdd(V20, C10,Result);
    Result = V4ScaleAdd(V22, C11,Result);

    return Result;
}

NV_FORCE_INLINE void V4Transpose(Vec4V& col0, Vec4V& col1, Vec4V& col2, Vec4V& col3)
{
    const float32x4x2_t v0v1 = vzipq_f32(col0, col2);
    const float32x4x2_t v2v3 = vzipq_f32(col1, col3);
    const float32x4x2_t zip0 = vzipq_f32(v0v1.val[0], v2v3.val[0]);
    const float32x4x2_t zip1 = vzipq_f32(v0v1.val[1], v2v3.val[1]);
    col0 = zip0.val[0];
    col1 = zip0.val[1];
    col2 = zip1.val[0];
    col3 = zip1.val[1];
}


//////////////////////////////////
//VEC4V
//////////////////////////////////

NV_FORCE_INLINE BoolV BFFFF()
{
    return vmovq_n_u32(0);
}

NV_FORCE_INLINE BoolV BFFFT()
{
    const uint32x2_t zeros = vmov_n_u32(0);
    const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
    const uint32x2_t zo = vext_u32(zeros, ones, 1);
    return vcombine_u32(zeros, zo);
}

NV_FORCE_INLINE BoolV BFFTF()
{
    const uint32x2_t zeros = vmov_n_u32(0);
    const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
    const uint32x2_t oz = vext_u32(ones, zeros, 1);
    return vcombine_u32(zeros, oz);
}

NV_FORCE_INLINE BoolV BFFTT()
{
    const uint32x2_t zeros = vmov_n_u32(0);
    const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
    return vcombine_u32(zeros, ones);
}

NV_FORCE_INLINE BoolV BFTFF()
{
    const uint32x2_t zeros = vmov_n_u32(0);
    const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
    const uint32x2_t zo = vext_u32(zeros, ones, 1);
    return vcombine_u32(zo, zeros);
}

NV_FORCE_INLINE BoolV BFTFT()
{
    const uint32x2_t zeros = vmov_n_u32(0);
    const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
    const uint32x2_t zo = vext_u32(zeros, ones, 1);
    return vcombine_u32(zo, zo);
}

NV_FORCE_INLINE BoolV BFTTF()
{
    const uint32x2_t zeros = vmov_n_u32(0);
    const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
    const uint32x2_t zo = vext_u32(zeros, ones, 1);
    const uint32x2_t oz = vext_u32(ones, zeros, 1);
    return vcombine_u32(zo, oz);
}

NV_FORCE_INLINE BoolV BFTTT()
{
    const uint32x2_t zeros = vmov_n_u32(0);
    const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
    const uint32x2_t zo = vext_u32(zeros, ones, 1);
    return vcombine_u32(zo, ones);
}

NV_FORCE_INLINE BoolV BTFFF()
{
    const uint32x2_t zeros = vmov_n_u32(0);
    const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
    //const uint32x2_t zo = vext_u32(zeros, ones, 1);
    const uint32x2_t oz = vext_u32(ones, zeros, 1);
    return vcombine_u32(oz, zeros);
}

NV_FORCE_INLINE BoolV BTFFT()
{
    const uint32x2_t zeros = vmov_n_u32(0);
    const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
    const uint32x2_t zo = vext_u32(zeros, ones, 1);
    const uint32x2_t oz = vext_u32(ones, zeros, 1);
    return vcombine_u32(oz, zo);
}

NV_FORCE_INLINE BoolV BTFTF()
{
    const uint32x2_t zeros = vmov_n_u32(0);
    const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
    const uint32x2_t oz = vext_u32(ones, zeros, 1);
    return vcombine_u32(oz, oz);
}

NV_FORCE_INLINE BoolV BTFTT()
{
    const uint32x2_t zeros = vmov_n_u32(0);
    const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
    const uint32x2_t oz = vext_u32(ones, zeros, 1);
    return vcombine_u32(oz, ones);
}

NV_FORCE_INLINE BoolV BTTFF()
{
    const uint32x2_t zeros = vmov_n_u32(0);
    const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
    return vcombine_u32(ones, zeros);
}

NV_FORCE_INLINE BoolV BTTFT()
{
    const uint32x2_t zeros = vmov_n_u32(0);
    const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
    const uint32x2_t zo = vext_u32(zeros, ones, 1);
    return vcombine_u32(ones, zo);
}

NV_FORCE_INLINE BoolV BTTTF()
{
    const uint32x2_t zeros = vmov_n_u32(0);
    const uint32x2_t ones = vmov_n_u32(0xffffFFFF);
    const uint32x2_t oz = vext_u32(ones, zeros, 1);
    return vcombine_u32(ones, oz);
}

NV_FORCE_INLINE BoolV BTTTT()
{
    return vmovq_n_u32(0xffffFFFF);
}

NV_FORCE_INLINE BoolV BXMask()
{
    return BTFFF();
}

NV_FORCE_INLINE BoolV BYMask()
{
    return BFTFF();
}

NV_FORCE_INLINE BoolV BZMask()
{
    return BFFTF();
}

NV_FORCE_INLINE BoolV BWMask()
{
    return BFFFT();
}

NV_FORCE_INLINE BoolV BGetX(const BoolV f)
{
    const uint32x2_t fLow = vget_low_u32(f);
    return vdupq_lane_u32(fLow, 0);
}

NV_FORCE_INLINE BoolV BGetY(const BoolV f)
{
    const uint32x2_t fLow = vget_low_u32(f);
    return vdupq_lane_u32(fLow, 1);
}

NV_FORCE_INLINE BoolV BGetZ(const BoolV f)
{
    const uint32x2_t fHigh = vget_high_u32(f);
    return vdupq_lane_u32(fHigh, 0);
}

NV_FORCE_INLINE BoolV BGetW(const BoolV f)
{
    const uint32x2_t fHigh = vget_high_u32(f);
    return vdupq_lane_u32(fHigh, 1);
}

NV_FORCE_INLINE BoolV BSetX(const BoolV v, const BoolV f)
{
    return vbslq_u32(BFTTT(), v, f);
}

NV_FORCE_INLINE BoolV BSetY(const BoolV v, const BoolV f)
{
    return vbslq_u32(BTFTT(), v, f);
}

NV_FORCE_INLINE BoolV BSetZ(const BoolV v, const BoolV f)
{
    return vbslq_u32(BTTFT(), v, f);
}

NV_FORCE_INLINE BoolV BSetW(const BoolV v, const BoolV f)
{
    return vbslq_u32(BTTTF(), v, f);
}


NV_FORCE_INLINE BoolV BAnd(const BoolV a, const BoolV b)
{
    return vandq_u32(a, b);
}

NV_FORCE_INLINE BoolV BNot(const BoolV a)
{
    return vmvnq_u32(a);
}

NV_FORCE_INLINE BoolV BAndNot(const BoolV a, const BoolV b)
{
    //return vbicq_u32(a, b);
    return vandq_u32(a, vmvnq_u32(b));
}

NV_FORCE_INLINE BoolV BOr(const BoolV a, const BoolV b)
{
    return vorrq_u32(a, b);
}

NV_FORCE_INLINE BoolV BAllTrue4(const BoolV a)
{
    const uint32x2_t allTrue = vmov_n_u32(0xffffFFFF);
    const uint16x4_t dHigh = vget_high_u16(vreinterpretq_u16_u32(a));
    const uint16x4_t dLow = vmovn_u32(a);
    uint16x8_t combined = vcombine_u16(dLow, dHigh);
    const uint32x2_t finalReduce = vreinterpret_u32_u8(vmovn_u16(combined));
    const uint32x2_t result =  vceq_u32(finalReduce, allTrue);
    return vdupq_lane_u32(result, 0);
}

NV_FORCE_INLINE BoolV BAnyTrue4(const BoolV a)
{
    const uint32x2_t allTrue = vmov_n_u32(0xffffFFFF);
    const uint16x4_t dHigh = vget_high_u16(vreinterpretq_u16_u32(a));
    const uint16x4_t dLow = vmovn_u32(a);
    uint16x8_t combined = vcombine_u16(dLow, dHigh);
    const uint32x2_t finalReduce = vreinterpret_u32_u8(vmovn_u16(combined));
    const uint32x2_t result = vtst_u32(finalReduce, allTrue);
    return vdupq_lane_u32(result, 0);
}

NV_FORCE_INLINE BoolV BAllTrue3(const BoolV a)
{
    const uint32x2_t allTrue3 = vmov_n_u32(0x00ffFFFF);
    const uint16x4_t dHigh = vget_high_u16(vreinterpretq_u16_u32(a));
    const uint16x4_t dLow = vmovn_u32(a);
    uint16x8_t combined = vcombine_u16(dLow, dHigh);
    const uint32x2_t finalReduce = vreinterpret_u32_u8(vmovn_u16(combined));
    const uint32x2_t result =  vceq_u32(vand_u32(finalReduce,allTrue3), allTrue3);
    return vdupq_lane_u32(result, 0);
}

NV_FORCE_INLINE BoolV BAnyTrue3(const BoolV a)
{
    const uint32x2_t allTrue3 = vmov_n_u32(0x00ffFFFF);
    const uint16x4_t dHigh = vget_high_u16(vreinterpretq_u16_u32(a));
    const uint16x4_t dLow = vmovn_u32(a);
    uint16x8_t combined = vcombine_u16(dLow, dHigh);
    const uint32x2_t finalReduce = vreinterpret_u32_u8(vmovn_u16(combined));
    const uint32x2_t result = vtst_u32(vand_u32(finalReduce,allTrue3), allTrue3);
    return vdupq_lane_u32(result, 0);
}

NV_FORCE_INLINE uint32_t BAllEq(const BoolV a, const BoolV b)
{
    const BoolV bTest = vceqq_u32(a, b);
    return internalUnitNeonSimd::BAllTrue4_R(bTest);
}

NV_FORCE_INLINE uint32_t BAllEqTTTT(const BoolV a)
{
    return BAllEq(a, BTTTT());
}

NV_FORCE_INLINE uint32_t BAllEqFFFF(const BoolV a)
{
    return BAllEq(a, BFFFF());
}

NV_FORCE_INLINE uint32_t BGetBitMask(const BoolV a)
{
    static NV_ALIGN(16, const uint32_t) bitMaskData[4] = { 1, 2, 4, 8 };
    const uint32x4_t bitMask = *(reinterpret_cast<const uint32x4_t*>(bitMaskData));
    const uint32x4_t t0 = vandq_u32(a, bitMask);
    const uint32x2_t t1 = vpadd_u32(vget_low_u32(t0), vget_high_u32(t0)); // Pairwise add (0 + 1), (2 + 3)
    return uint32_t(vget_lane_u32(vpadd_u32(t1, t1), 0));
}

//////////////////////////////////
//MAT33V
//////////////////////////////////

NV_FORCE_INLINE Vec3V M33MulV3(const Mat33V& a, const Vec3V b)
{
    const FloatV x=V3GetX(b);
    const FloatV y=V3GetY(b);
    const FloatV z=V3GetZ(b);
    const Vec3V v0=V3Scale(a.col0,x);
    const Vec3V v1=V3Scale(a.col1,y);
    const Vec3V v2=V3Scale(a.col2,z);
    const Vec3V v0PlusV1=V3Add(v0,v1);
    return V3Add(v0PlusV1,v2);
}

NV_FORCE_INLINE Vec3V M33TrnspsMulV3(const Mat33V& a, const Vec3V b)
{
    const FloatV x=V3Dot(a.col0,b);
    const FloatV y=V3Dot(a.col1,b);
    const FloatV z=V3Dot(a.col2,b);
    return V3Merge(x,y,z);
}

NV_FORCE_INLINE Vec3V M33MulV3AddV3(const Mat33V& A, const Vec3V b, const Vec3V c)
{
    const FloatV x=V3GetX(b);
    const FloatV y=V3GetY(b);
    const FloatV z=V3GetZ(b);
    Vec3V result = V3ScaleAdd(A.col0, x, c);
    result = V3ScaleAdd(A.col1, y, result);
    return V3ScaleAdd(A.col2, z, result);
}

NV_FORCE_INLINE Mat33V M33MulM33(const Mat33V& a, const Mat33V& b)
{
    return Mat33V(M33MulV3(a,b.col0),M33MulV3(a,b.col1),M33MulV3(a,b.col2));
}

NV_FORCE_INLINE Mat33V M33Add(const Mat33V& a, const Mat33V& b)
{
    return Mat33V(V3Add(a.col0,b.col0),V3Add(a.col1,b.col1),V3Add(a.col2,b.col2));
}

NV_FORCE_INLINE Mat33V M33Scale(const Mat33V& a, const FloatV& b)
{
    return Mat33V(V3Scale(a.col0,b),V3Scale(a.col1,b),V3Scale(a.col2,b));
}


NV_FORCE_INLINE Mat33V M33Inverse(const Mat33V& a)
{
    const float32x2_t zeros = vreinterpret_f32_u32(vmov_n_u32(0));
    const BoolV btttf = BTTTF();

    const Vec3V cross01 = V3Cross(a.col0,a.col1);
    const Vec3V cross12 = V3Cross(a.col1,a.col2);
    const Vec3V cross20 = V3Cross(a.col2,a.col0);
    const FloatV dot = V3Dot(cross01,a.col2);
    const FloatV invDet = FRecipFast(dot);

    const float32x4x2_t merge = vzipq_f32(cross12, cross01);
    const float32x4_t mergeh = merge.val[0];
    const float32x4_t mergel = merge.val[1];

    //const Vec3V colInv0 = XMVectorPermute(mergeh,cross20,NvPermuteControl(0,4,1,7));
    const float32x4_t colInv0_xxyy = vzipq_f32(mergeh, cross20).val[0];
    const float32x4_t colInv0 = vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(colInv0_xxyy), btttf));

    //const Vec3V colInv1 = XMVectorPermute(mergeh,cross20,NvPermuteControl(2,5,3,7));
    const float32x2_t zw0 = vget_high_f32(mergeh);
    const float32x2_t xy1 = vget_low_f32(cross20);
    const float32x2_t yzero1 = vext_f32(xy1, zeros, 1);
    const float32x2x2_t merge1 = vzip_f32(zw0, yzero1);
    const float32x4_t colInv1 = vcombine_f32(merge1.val[0], merge1.val[1]);

    //const Vec3V colInv2 = XMVectorPermute(mergel,cross20,NvPermuteControl(0,6,1,7));
    const float32x2_t x0y0 = vget_low_f32(mergel);
    const float32x2_t z1w1 = vget_high_f32(cross20);
    const float32x2x2_t merge2 = vzip_f32(x0y0, z1w1);
    const float32x4_t colInv2 = vcombine_f32(merge2.val[0], merge2.val[1]);

    return Mat33V
        (
        vmulq_lane_f32(colInv0, invDet, 0),
        vmulq_lane_f32(colInv1, invDet, 0),
        vmulq_lane_f32(colInv2, invDet, 0)
        );
}

NV_FORCE_INLINE Mat33V M33Trnsps(const Mat33V& a)
{
    return Mat33V
        (
        V3Merge(V3GetX(a.col0),V3GetX(a.col1),V3GetX(a.col2)),
        V3Merge(V3GetY(a.col0),V3GetY(a.col1),V3GetY(a.col2)),
        V3Merge(V3GetZ(a.col0),V3GetZ(a.col1),V3GetZ(a.col2))
        );
}

NV_FORCE_INLINE Mat33V M33Identity()
{
    return Mat33V
        (
        V3UnitX(),
        V3UnitY(),
        V3UnitZ()
        );
}

NV_FORCE_INLINE Mat33V M33Sub(const Mat33V& a, const Mat33V& b)
{
    return Mat33V(V3Sub(a.col0,b.col0),V3Sub(a.col1,b.col1),V3Sub(a.col2,b.col2));
}

NV_FORCE_INLINE Mat33V M33Neg(const Mat33V& a)
{
    return Mat33V(V3Neg(a.col0),V3Neg(a.col1),V3Neg(a.col2));
}

NV_FORCE_INLINE Mat33V M33Abs(const Mat33V& a)
{
    return Mat33V(V3Abs(a.col0),V3Abs(a.col1),V3Abs(a.col2));
}

NV_FORCE_INLINE Mat33V PromoteVec3V(const Vec3V v)
{
    const BoolV bTFFF = BTFFF();
    const BoolV bFTFF = BFTFF();
    const BoolV bFFTF = BTFTF();

    const Vec3V zero = V3Zero();

    return Mat33V(  V3Sel(bTFFF, v, zero),
        V3Sel(bFTFF, v, zero),
        V3Sel(bFFTF, v, zero));
}

NV_FORCE_INLINE Mat33V M33Diagonal(const Vec3VArg d)
{
    const Vec3V x = V3Mul(V3UnitX(), d);
    const Vec3V y = V3Mul(V3UnitY(), d);
    const Vec3V z = V3Mul(V3UnitZ(), d);
    return Mat33V(x, y, z);
}

//////////////////////////////////
//MAT34V
//////////////////////////////////

NV_FORCE_INLINE Vec3V M34MulV3(const Mat34V& a, const Vec3V b)
{
    const FloatV x=V3GetX(b);
    const FloatV y=V3GetY(b);
    const FloatV z=V3GetZ(b);
    const Vec3V v0=V3Scale(a.col0,x);
    const Vec3V v1=V3Scale(a.col1,y);
    const Vec3V v2=V3Scale(a.col2,z);
    const Vec3V v0PlusV1=V3Add(v0,v1);
    const Vec3V v0PlusV1Plusv2=V3Add(v0PlusV1,v2);
    return (V3Add(v0PlusV1Plusv2,a.col3));
}

NV_FORCE_INLINE Vec3V M34Mul33V3(const Mat34V& a, const Vec3V b)
{
    const FloatV x=V3GetX(b);
    const FloatV y=V3GetY(b);
    const FloatV z=V3GetZ(b);
    const Vec3V v0=V3Scale(a.col0,x);
    const Vec3V v1=V3Scale(a.col1,y);
    const Vec3V v2=V3Scale(a.col2,z);
    const Vec3V v0PlusV1=V3Add(v0,v1);
    return V3Add(v0PlusV1,v2);
}

NV_FORCE_INLINE Vec3V M34TrnspsMul33V3(const Mat34V& a, const Vec3V b)
{
    const FloatV x=V3Dot(a.col0,b);
    const FloatV y=V3Dot(a.col1,b);
    const FloatV z=V3Dot(a.col2,b);
    return V3Merge(x,y,z);
}

NV_FORCE_INLINE Mat34V M34MulM34(const Mat34V& a, const Mat34V& b)
{
    return Mat34V(M34Mul33V3(a,b.col0), M34Mul33V3(a,b.col1),M34Mul33V3(a,b.col2),M34MulV3(a,b.col3));
}

NV_FORCE_INLINE Mat33V M34MulM33(const Mat34V& a, const Mat33V& b)
{
    return Mat33V(M34Mul33V3(a,b.col0),M34Mul33V3(a,b.col1),M34Mul33V3(a,b.col2));
}

NV_FORCE_INLINE Mat33V M34Mul33MM34(const Mat34V& a, const Mat34V& b)
{
    return Mat33V(M34Mul33V3(a,b.col0),M34Mul33V3(a,b.col1),M34Mul33V3(a,b.col2));
}

NV_FORCE_INLINE Mat34V M34Add(const Mat34V& a, const Mat34V& b)
{
    return Mat34V(V3Add(a.col0,b.col0),V3Add(a.col1,b.col1),V3Add(a.col2,b.col2),V3Add(a.col3,b.col3));
}

NV_FORCE_INLINE Mat33V M34Trnsps33(const Mat34V& a)
{
    return Mat33V
        (
        V3Merge(V3GetX(a.col0),V3GetX(a.col1),V3GetX(a.col2)),
        V3Merge(V3GetY(a.col0),V3GetY(a.col1),V3GetY(a.col2)),
        V3Merge(V3GetZ(a.col0),V3GetZ(a.col1),V3GetZ(a.col2))
        );
}


//////////////////////////////////
//MAT44V
//////////////////////////////////

NV_FORCE_INLINE Vec4V M44MulV4(const Mat44V& a, const Vec4V b)
{
    const FloatV x=V4GetX(b);
    const FloatV y=V4GetY(b);
    const FloatV z=V4GetZ(b);
    const FloatV w=V4GetW(b);

    const Vec4V v0=V4Scale(a.col0,x);
    const Vec4V v1=V4Scale(a.col1,y);
    const Vec4V v2=V4Scale(a.col2,z);
    const Vec4V v3=V4Scale(a.col3,w);
    const Vec4V v0PlusV1=V4Add(v0,v1);
    const Vec4V v0PlusV1Plusv2=V4Add(v0PlusV1,v2);
    return (V4Add(v0PlusV1Plusv2,v3));
}

NV_FORCE_INLINE Vec4V M44TrnspsMulV4(const Mat44V& a, const Vec4V b)
{
    return V4Merge(V4Dot(a.col0,b), V4Dot(a.col1,b), V4Dot(a.col2,b), V4Dot(a.col3,b));
}

NV_FORCE_INLINE Mat44V M44MulM44(const Mat44V& a, const Mat44V& b)
{
    return Mat44V(M44MulV4(a,b.col0),M44MulV4(a,b.col1),M44MulV4(a,b.col2),M44MulV4(a,b.col3));
}

NV_FORCE_INLINE Mat44V M44Add(const Mat44V& a, const Mat44V& b)
{
    return Mat44V(V4Add(a.col0,b.col0),V4Add(a.col1,b.col1),V4Add(a.col2,b.col2),V4Add(a.col3,b.col3));
}

NV_FORCE_INLINE Mat44V M44Trnsps(const Mat44V& a)
{
    // asm volatile(
    // "vzip.f32 %q0, %q2 \n\t"
    // "vzip.f32 %q1, %q3 \n\t"
    // "vzip.f32 %q0, %q1 \n\t"
    // "vzip.f32 %q2, %q3 \n\t"
    // : "+w" (a.col0), "+w" (a.col1), "+w" (a.col2), "+w" a.col3));

    const float32x4x2_t v0v1 = vzipq_f32(a.col0, a.col2);
    const float32x4x2_t v2v3 = vzipq_f32(a.col1, a.col3);
    const float32x4x2_t zip0 = vzipq_f32(v0v1.val[0], v2v3.val[0]);
    const float32x4x2_t zip1 = vzipq_f32(v0v1.val[1], v2v3.val[1]);

    return Mat44V(zip0.val[0], zip0.val[1], zip1.val[0], zip1.val[1]);
}

NV_FORCE_INLINE Mat44V M44Inverse(const Mat44V& a)
{
    float32x4_t minor0, minor1, minor2, minor3;
    float32x4_t row0, row1, row2, row3;
    float32x4_t det, tmp1;

    tmp1 = vmovq_n_f32(0.0f);
    row1 = vmovq_n_f32(0.0f);
    row3 = vmovq_n_f32(0.0f);

    row0 = a.col0;
    row1 = vextq_f32(a.col1, a.col1, 2);
    row2 = a.col2;
    row3 = vextq_f32(a.col3, a.col3, 2);

    tmp1 = vmulq_f32(row2, row3);
    tmp1 = vrev64q_f32(tmp1);
    minor0 = vmulq_f32(row1, tmp1);
    minor1 = vmulq_f32(row0, tmp1);
    tmp1 = vextq_f32(tmp1, tmp1, 2);
    minor0 = vsubq_f32(vmulq_f32(row1, tmp1), minor0);
    minor1 = vsubq_f32(vmulq_f32(row0, tmp1), minor1);
    minor1 = vextq_f32(minor1, minor1, 2);

    tmp1 = vmulq_f32(row1, row2);
    tmp1 = vrev64q_f32(tmp1);
    minor0 = vaddq_f32(vmulq_f32(row3, tmp1), minor0);
    minor3 = vmulq_f32(row0, tmp1);
    tmp1 = vextq_f32(tmp1, tmp1, 2);
    minor0 = vsubq_f32(minor0, vmulq_f32(row3, tmp1));
    minor3 = vsubq_f32(vmulq_f32(row0, tmp1), minor3);
    minor3 = vextq_f32(minor3, minor3, 2);

    tmp1 = vmulq_f32(vextq_f32(row1, row1, 2), row3);
    tmp1 = vrev64q_f32(tmp1);
    row2 = vextq_f32(row2, row2, 2);
    minor0 = vaddq_f32(vmulq_f32(row2, tmp1), minor0);
    minor2 = vmulq_f32(row0, tmp1);
    tmp1 = vextq_f32(tmp1, tmp1, 2);
    minor0 = vsubq_f32(minor0, vmulq_f32(row2, tmp1));
    minor2 = vsubq_f32(vmulq_f32(row0, tmp1), minor2);
    minor2 = vextq_f32(minor2, minor2, 2);

    tmp1 = vmulq_f32(row0, row1);
    tmp1 = vrev64q_f32(tmp1);
    minor2 = vaddq_f32(vmulq_f32(row3, tmp1), minor2);
    minor3 = vsubq_f32(vmulq_f32(row2, tmp1), minor3);
    tmp1 = vextq_f32(tmp1, tmp1, 2);
    minor2 = vsubq_f32(vmulq_f32(row3, tmp1), minor2);
    minor3 = vsubq_f32(minor3, vmulq_f32(row2, tmp1));

    tmp1 = vmulq_f32(row0, row3);
    tmp1 = vrev64q_f32(tmp1);
    minor1 = vsubq_f32(minor1, vmulq_f32(row2, tmp1));
    minor2 = vaddq_f32(vmulq_f32(row1, tmp1), minor2);
    tmp1 = vextq_f32(tmp1, tmp1, 2);
    minor1 = vaddq_f32(vmulq_f32(row2, tmp1), minor1);
    minor2 = vsubq_f32(minor2, vmulq_f32(row1, tmp1));

    tmp1 = vmulq_f32(row0, row2);
    tmp1 = vrev64q_f32(tmp1);
    minor1 = vaddq_f32(vmulq_f32(row3, tmp1), minor1);
    minor3 = vsubq_f32(minor3, vmulq_f32(row1, tmp1));
    tmp1 = vextq_f32(tmp1, tmp1, 2);
    minor1 = vsubq_f32(minor1, vmulq_f32(row3, tmp1));
    minor3 = vaddq_f32(vmulq_f32(row1, tmp1), minor3);

    det = vmulq_f32(row0, minor0);
    det = vaddq_f32(vextq_f32(det, det, 2), det);
    det = vaddq_f32(vrev64q_f32(det), det);
    det = vdupq_lane_f32(VRECIPE(vget_low_f32(det)), 0);

    minor0 = vmulq_f32(det, minor0);
    minor1 = vmulq_f32(det, minor1);
    minor2 = vmulq_f32(det, minor2);
    minor3 = vmulq_f32(det, minor3);
    Mat44V invTrans(minor0,minor1,minor2,minor3);
    return M44Trnsps(invTrans);
}


NV_FORCE_INLINE Vec4V V4LoadXYZW(const float& x, const float& y, const float& z, const float& w)
{
#if NV_WINRT
    NV_ALIGN(16,float) r[4] = {x, y, z ,w};
    return vld1q_f32((const float32_t*)r);
#else
    const float32x4_t ret = {x, y, z, w};
    return ret;
#endif // NV_WINRT
}

/*
NV_FORCE_INLINE VecU16V V4U32PK(VecU32V a, VecU32V b)
{
    return vcombine_u16(vqmovn_u32(a), vqmovn_u32(b));
}
*/

NV_FORCE_INLINE VecU32V V4U32Sel(const BoolV c, const VecU32V a, const VecU32V b)
{
    return vbslq_u32(c, a, b);
}

NV_FORCE_INLINE VecU32V V4U32or(VecU32V a, VecU32V b)
{
    return vorrq_u32(a, b);
}

NV_FORCE_INLINE VecU32V V4U32and(VecU32V a, VecU32V b)
{
    return vandq_u32(a, b);
}

NV_FORCE_INLINE VecU32V V4U32Andc(VecU32V a, VecU32V b)
{
    //return vbicq_u32(a, b); // creates gcc compiler bug in RTreeQueries.cpp
    return vandq_u32(a, vmvnq_u32(b));
}

/*
NV_FORCE_INLINE VecU16V V4U16Or(VecU16V a, VecU16V b)
{
    return vorrq_u16(a, b);
}
*/

/*
NV_FORCE_INLINE VecU16V V4U16And(VecU16V a, VecU16V b)
{
    return vandq_u16(a, b);
}
*/
/*
NV_FORCE_INLINE VecU16V V4U16Andc(VecU16V a, VecU16V b)
{
    return vbicq_u16(a, b);
}
*/

NV_FORCE_INLINE VecI32V I4Load(const int32_t i)
{
    return vdupq_n_s32(i);
}

NV_FORCE_INLINE VecI32V I4LoadU(const int32_t* i)
{
    return vld1q_s32(i);
}

NV_FORCE_INLINE VecI32V I4LoadA(const int32_t* i)
{
    return vld1q_s32(i);
}

NV_FORCE_INLINE VecI32V VecI32V_Add(const VecI32VArg a, const VecI32VArg b)
{
    return vaddq_s32(a, b);
}

NV_FORCE_INLINE VecI32V VecI32V_Sub(const VecI32VArg a, const VecI32VArg b)
{
    return vsubq_s32(a, b);
}

NV_FORCE_INLINE BoolV VecI32V_IsGrtr(const VecI32VArg a, const VecI32VArg b)
{
    return vcgtq_s32(a, b);
}

NV_FORCE_INLINE BoolV VecI32V_IsEq(const VecI32VArg a, const VecI32VArg b)
{
    return vceqq_s32(a, b);
}

NV_FORCE_INLINE VecI32V V4I32Sel(const BoolV c, const VecI32V a, const VecI32V b)
{
    return vbslq_s32(c, a, b);
}

NV_FORCE_INLINE VecI32V VecI32V_Zero()
{
    return vdupq_n_s32(0);
}

NV_FORCE_INLINE VecI32V VecI32V_One()
{
    return vdupq_n_s32(1);
}

NV_FORCE_INLINE VecI32V VecI32V_Two()
{
    return vdupq_n_s32(2);
}

NV_FORCE_INLINE VecI32V VecI32V_MinusOne()
{
    return vdupq_n_s32(-1);
}


NV_FORCE_INLINE VecU32V U4Zero()
{
    return U4Load(0);
}

NV_FORCE_INLINE VecU32V U4One()
{
    return U4Load(1);
}

NV_FORCE_INLINE VecU32V U4Two()
{
    return U4Load(2);
}

NV_FORCE_INLINE VecShiftV VecI32V_PrepareShift(const VecI32VArg shift)
{
    return shift;
}

NV_FORCE_INLINE VecI32V VecI32V_LeftShift(const VecI32VArg a, const VecShiftVArg count)
{
    return vshlq_s32(a, count);
}

NV_FORCE_INLINE VecI32V VecI32V_RightShift(const VecI32VArg a, const VecShiftVArg count)
{   
    return vshlq_s32(a, VecI32V_Sub(I4Load(0), count));
}

NV_FORCE_INLINE VecI32V VecI32V_And(const VecI32VArg a, const VecI32VArg b)
{
    return vandq_s32(a, b);
}

NV_FORCE_INLINE VecI32V VecI32V_Or(const VecI32VArg a, const VecI32VArg b)
{
    return vorrq_s32(a, b);
}

NV_FORCE_INLINE VecI32V VecI32V_GetX(const VecI32VArg f)
{
    const int32x2_t fLow = vget_low_s32(f);
    return vdupq_lane_s32(fLow, 0);
}

NV_FORCE_INLINE VecI32V VecI32V_GetY(const VecI32VArg f)
{
    const int32x2_t fLow = vget_low_s32(f);
    return vdupq_lane_s32(fLow, 1);
}

NV_FORCE_INLINE VecI32V VecI32V_GetZ(const VecI32VArg f)
{
    const int32x2_t fHigh = vget_high_s32(f);
    return vdupq_lane_s32(fHigh, 0);
}

NV_FORCE_INLINE VecI32V VecI32V_GetW(const VecI32VArg f)
{
    const int32x2_t fHigh = vget_high_s32(f);
    return vdupq_lane_s32(fHigh, 1);
}

NV_FORCE_INLINE VecI32V VecI32V_Sel(const BoolV c, const VecI32VArg a, const VecI32VArg b)
{
    VECMATHAOS_ASSERT(_VecMathTests::allElementsEqualBoolV(c,BTTTT()) || _VecMathTests::allElementsEqualBoolV(c,BFFFF()));
    return vbslq_s32(c, a, b);
}

NV_FORCE_INLINE void NvI32_From_VecI32V(const VecI32VArg a, int32_t* i)
{
    *i = vgetq_lane_s32(a, 0);
}

NV_FORCE_INLINE VecI32V VecI32V_Merge(const VecI32VArg a, const VecI32VArg b, const VecI32VArg c, const VecI32VArg d)
{
    const int32x2_t aLow = vget_low_s32(a);
    const int32x2_t bLow = vget_low_s32(b);
    const int32x2_t cLow = vget_low_s32(c);
    const int32x2_t dLow = vget_low_s32(d);

    const int32x2_t low = vext_s32(aLow, bLow, 1);
    const int32x2_t high = vext_s32(cLow, dLow, 1);

    return vcombine_s32(low, high);
}

NV_FORCE_INLINE VecI32V VecI32V_From_BoolV(const BoolVArg a)
{
    return reinterpret_cast<const int32x4_t&>(a);
}

NV_FORCE_INLINE VecU32V VecU32V_From_BoolV(const BoolVArg a)
{
    return reinterpret_cast<const uint32x4_t&>(a);
}

/*
template<int a> NV_FORCE_INLINE VecI32V V4ISplat()
{
    return vdupq_n_s32(a);
}

template<uint32_t a> NV_FORCE_INLINE VecU32V V4USplat()
{
    return vdupq_n_u32(a);
}
*/

/*
NV_FORCE_INLINE void V4U16StoreAligned(VecU16V val, VecU16V* address)
{
    vst1q_u16((uint16_t*)address, val);
}
*/

NV_FORCE_INLINE void V4U32StoreAligned(VecU32V val, VecU32V* address)
{
    vst1q_u32((uint32_t*)address, val);
}

NV_FORCE_INLINE Vec4V V4LoadAligned(Vec4V* addr)
{
    return vld1q_f32((float32_t*)addr);
}

NV_FORCE_INLINE Vec4V V4LoadUnaligned(Vec4V* addr)
{
    return vld1q_f32((float32_t*)addr);
}

NV_FORCE_INLINE Vec4V V4Andc(const Vec4V a, const VecU32V b)
{
    return vreinterpretq_f32_u32(V4U32Andc(vreinterpretq_u32_f32(a), b));
}

NV_FORCE_INLINE VecU32V V4IsGrtrV32u(const Vec4V a, const Vec4V b)
{
    return V4IsGrtr(a, b);
}

NV_FORCE_INLINE VecU16V V4U16LoadAligned(VecU16V* addr)
{
    return vld1q_u16((uint16_t*)addr);
}

NV_FORCE_INLINE VecU16V V4U16LoadUnaligned(VecU16V* addr)
{
    return vld1q_u16((uint16_t*)addr);
}

NV_FORCE_INLINE VecU16V V4U16CompareGt(VecU16V a, VecU16V b)
{
    return vcgtq_u16(a, b);
}

NV_FORCE_INLINE VecU16V V4I16CompareGt(VecU16V a, VecU16V b)
{
    return vcgtq_s16((VecI16V&)a, (VecI16V&)b);
}

NV_FORCE_INLINE Vec4V Vec4V_From_VecU32V(VecU32V a)
{
    return vcvtq_f32_u32(a);
}

NV_FORCE_INLINE Vec4V Vec4V_From_VecI32V(VecI32V a)
{
    return vcvtq_f32_s32(a);
}

NV_FORCE_INLINE VecI32V VecI32V_From_Vec4V(Vec4V a)
{
    return vcvtq_s32_f32(a);
}

NV_FORCE_INLINE Vec4V Vec4V_ReinterpretFrom_VecU32V(VecU32V a)
{
    return vreinterpretq_f32_u32(a);
}

NV_FORCE_INLINE Vec4V Vec4V_ReinterpretFrom_VecI32V(VecI32V a)
{
    return vreinterpretq_f32_s32(a);
}

NV_FORCE_INLINE VecU32V VecU32V_ReinterpretFrom_Vec4V(Vec4V a)
{
    return vreinterpretq_u32_f32(a);
}

NV_FORCE_INLINE VecI32V VecI32V_ReinterpretFrom_Vec4V(Vec4V a)
{
    return vreinterpretq_s32_f32(a);
}

template<int index> NV_FORCE_INLINE BoolV BSplatElement(BoolV a)
{   
#if NV_WINRT
    if(index == 0)
    {
        return vdupq_lane_u32(vget_low_u32(a), 0);
    }
    else if (index == 1)
    {
        return vdupq_lane_u32(vget_low_u32(a), 1);
    }
#else
    if(index < 2)
    {
        return vdupq_lane_u32(vget_low_u32(a), index);
    }
#endif
    else if(index == 2)
    {
        return vdupq_lane_u32(vget_high_u32(a), 0);
    }
    else if(index == 3)
    {
        return vdupq_lane_u32(vget_high_u32(a), 1);
    }
}


template<int index> NV_FORCE_INLINE VecU32V V4U32SplatElement(VecU32V a)
{
    const int highIndex = index-2;
#if NV_WINRT
    if(index == 0)
    {
        return vdupq_lane_u32(vget_low_u32(a), 0);
    }
    else if (index == 1)
    {
        return vdupq_lane_u32(vget_low_u32(a), 1);
    }
#else
    if(index < 2)
    {
        return vdupq_lane_u32(vget_low_u32(a), index);
    }
#endif
    else if(index == 2)
    {
        return vdupq_lane_u32(vget_high_u32(a), 0);
    }
    else if(index == 3)
    {
        return vdupq_lane_u32(vget_high_u32(a), 1);
    }
}

template<int index> NV_FORCE_INLINE Vec4V V4SplatElement(Vec4V a)
{
#if NV_WINRT
    if(index == 0)
    {
        return vdupq_lane_f32(vget_low_f32(a), 0);
    }
    else if (index == 1)
    {
        return vdupq_lane_f32(vget_low_f32(a), 1);
    }
#else
    if(index < 2)
    {
        return vdupq_lane_f32(vget_low_f32(a), index);
    }
#endif
    else if(index == 2)
    {
        return vdupq_lane_f32(vget_high_f32(a), 0);
    }
    else if(index == 3)
    {
        return vdupq_lane_f32(vget_high_f32(a), 1);
    }
}

template<int index> NV_FORCE_INLINE VecU16V V4U16SplatElement(VecU16V a)
{
#if NV_WINRT
    if(index == 0)
    {
        return vdupq_lane_u16(vget_low_u16(a), 0);
    }
    else if(index == 1)
    {
        return vdupq_lane_u16(vget_low_u16(a), 1);
    }
    else if(index == 2)
    {
        return vdupq_lane_u16(vget_low_u16(a), 2);
    }
    else if(index == 3)
    {
        return vdupq_lane_u16(vget_low_u16(a), 3);
    }
#else
    if(index < 4)
    {
        return vdupq_lane_u16(vget_low_u16(a),index);
    }
#endif
    else if(index == 4)
    {
        return vdupq_lane_u16(vget_high_u16(a), 0);
    }
    else if(index == 5)
    {
        return vdupq_lane_u16(vget_high_u16(a), 1);
    }
    else if(index == 6)
    {
        return vdupq_lane_u16(vget_high_u16(a), 2);
    }
    else if(index == 7)
    {
        return vdupq_lane_u16(vget_high_u16(a), 3);
    }
}

template<int imm> NV_FORCE_INLINE VecI16V V4I16SplatImmediate()
{
    return vdupq_n_s16(imm);
}

template<uint16_t imm> NV_FORCE_INLINE VecU16V V4U16SplatImmediate()
{
    return vdupq_n_u16(imm);
}

NV_FORCE_INLINE VecU16V V4U16SubtractModulo(VecU16V a, VecU16V b)
{
    return vsubq_u16(a, b);
}

NV_FORCE_INLINE VecU16V V4U16AddModulo(VecU16V a, VecU16V b)
{
    return vaddq_u16(a, b);
}

NV_FORCE_INLINE VecU32V V4U16GetLo16(VecU16V a)
{
    const uint16x4x2_t ret = vuzp_u16(vget_low_u16(a), vget_high_u16(a));
    return vmovl_u16(ret.val[0]);
}

NV_FORCE_INLINE VecU32V V4U16GetHi16(VecU16V a)
{
    const uint16x4x2_t ret = vuzp_u16(vget_low_u16(a), vget_high_u16(a));
    return vmovl_u16(ret.val[1]);
}

NV_FORCE_INLINE VecU32V VecU32VLoadXYZW(uint32_t x, uint32_t y, uint32_t z, uint32_t w)
{
#if NV_WINRT
    NV_ALIGN(16,uint32_t) r[4] = {x, y, z ,w};
    return vld1q_u32((const uint32_t*)r);
#else
    const uint32x4_t ret = {x, y, z, w};
    return ret;
#endif
}

NV_FORCE_INLINE VecU32V U4Load(const uint32_t i)
{
    return vdupq_n_u32(i);
}

NV_FORCE_INLINE VecU32V U4LoadU(const uint32_t* i)
{
    return vld1q_u32(i);
}

NV_FORCE_INLINE VecU32V U4LoadA(const uint32_t* i)
{
    return vld1q_u32(i);
}

NV_FORCE_INLINE Vec4V V4Ceil(const Vec4V in)
{
    const float32x4_t ones = vdupq_n_f32(1.0f);
    const float32x4_t rdToZero = vcvtq_f32_s32(vcvtq_s32_f32(in));
    const float32x4_t rdToZeroPlusOne = vaddq_f32(rdToZero, ones);
    const uint32x4_t gt = vcgtq_f32(in, rdToZero);
    return vbslq_f32(gt, rdToZeroPlusOne, rdToZero);
}

NV_FORCE_INLINE Vec4V V4Floor(const Vec4V in)
{
    const float32x4_t ones = vdupq_n_f32(1.0f);
    const float32x4_t rdToZero = vcvtq_f32_s32(vcvtq_s32_f32(in));
    const float32x4_t rdToZeroMinusOne = vsubq_f32(rdToZero, ones);
    const uint32x4_t lt = vcltq_f32(in, rdToZero);
    return vbslq_f32(lt, rdToZeroMinusOne, rdToZero);
}

NV_FORCE_INLINE VecU32V V4ConvertToU32VSaturate(const Vec4V in, uint32_t power)
{
    NV_ASSERT(power == 0 && "Non-zero power not supported in convertToU32VSaturate");
    NV_UNUSED(power); // prevent warning in release builds

    return vcvtq_u32_f32(in);
}

#endif //PS_UNIX_NEON_INLINE_AOS_H

