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

#ifndef PS_UNIX_SSE2_AOS_H
#define PS_UNIX_SSE2_AOS_H

// no includes here! this file should be included from NvcVecMath.h only!!!

#if !COMPILE_VECTOR_INTRINSICS
#error Vector intrinsics should not be included when using scalar implementation.
#endif

typedef union UnionM128
{
    UnionM128(){}
    UnionM128(__m128 in)
    {
        m128 = in;
    }
    
    UnionM128(__m128i in)
    {
        m128i = in;
    }
    
    operator __m128()
    {
        return m128;
    }
    
    operator const __m128() const
    {
        return m128;
    }
    
    
    float       m128_f32[4];
    __int8_t    m128_i8[16];
    __int16_t   m128_i16[8];
    __int32_t   m128_i32[4];
    __int64_t   m128_i64[2];
    __uint16_t  m128_u16[8];
    __uint32_t  m128_u32[4];
    __uint64_t  m128_u64[2];
    __m128      m128;
    __m128i     m128i;
} UnionM128;


typedef __m128 FloatV;
typedef __m128 Vec3V;
typedef __m128 Vec4V;
typedef __m128 BoolV;
typedef __m128 QuatV;
//typedef __m128 VecU32V;
//typedef __m128 VecI32V;
//typedef __m128 VecU16V;
//typedef __m128 VecI16V;
//typedef __m128 VecU8V;
typedef UnionM128 VecU32V;
typedef UnionM128 VecI32V;
typedef UnionM128 VecU16V;
typedef UnionM128 VecI16V;
typedef UnionM128 VecU8V;

#define FloatVArg   FloatV&
#define Vec3VArg    Vec3V&
#define Vec4VArg    Vec4V&
#define BoolVArg    BoolV&
#define VecU32VArg  VecU32V&
#define VecI32VArg  VecI32V&
#define VecU16VArg  VecU16V&
#define VecI16VArg  VecI16V&
#define VecU8VArg   VecU8V&
#define QuatVArg    QuatV&

//Optimization for situations in which you cross product multiple vectors with the same vector.
//Avoids 2X shuffles per product
struct VecCrossV
{
    Vec3V mL1;
    Vec3V mR1;
};

struct VecShiftV
{
    VecI32V shift;
};
#define VecShiftVArg VecShiftV&

NV_ALIGN_PREFIX(16)
struct Mat33V
{
    Mat33V(){}
    Mat33V(const Vec3V& c0, const Vec3V& c1, const Vec3V& c2)
        : col0(c0),
          col1(c1),
          col2(c2)
    {
    }
    Vec3V NV_ALIGN(16,col0);
    Vec3V NV_ALIGN(16,col1);
    Vec3V NV_ALIGN(16,col2);
}NV_ALIGN_SUFFIX(16);

NV_ALIGN_PREFIX(16)
struct Mat34V
{
    Mat34V(){}
    Mat34V(const Vec3V& c0, const Vec3V& c1, const Vec3V& c2, const Vec3V& c3)
        : col0(c0),
          col1(c1),
          col2(c2),
          col3(c3)
    {
    }
    Vec3V NV_ALIGN(16,col0);
    Vec3V NV_ALIGN(16,col1);
    Vec3V NV_ALIGN(16,col2);
    Vec3V NV_ALIGN(16,col3);
}NV_ALIGN_SUFFIX(16);

NV_ALIGN_PREFIX(16)
struct Mat43V
{
    Mat43V(){}
    Mat43V(const Vec4V& c0, const Vec4V& c1, const Vec4V& c2)
        : col0(c0),
          col1(c1),
          col2(c2)
    {
    }
    Vec4V NV_ALIGN(16,col0);
    Vec4V NV_ALIGN(16,col1);
    Vec4V NV_ALIGN(16,col2);
}NV_ALIGN_SUFFIX(16);

NV_ALIGN_PREFIX(16)
struct Mat44V
{
    Mat44V(){}
    Mat44V(const Vec4V& c0, const Vec4V& c1, const Vec4V& c2, const Vec4V& c3)
        : col0(c0),
          col1(c1),
          col2(c2),
          col3(c3)
    {
    }
    Vec4V NV_ALIGN(16,col0);
    Vec4V NV_ALIGN(16,col1);
    Vec4V NV_ALIGN(16,col2);
    Vec4V NV_ALIGN(16,col3);
}NV_ALIGN_SUFFIX(16);

#endif //PS_UNIX_SSE2_AOS_H



