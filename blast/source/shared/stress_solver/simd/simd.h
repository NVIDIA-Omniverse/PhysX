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
// Copyright (c) 2022-2024 NVIDIA Corporation. All rights reserved.

#pragma once

#include <xmmintrin.h>
#include <emmintrin.h>
#include <immintrin.h>

#if defined(__GNUC__)   // missing with gcc
#define _mm256_set_m128(vh, vl) _mm256_insertf128_ps(_mm256_castps128_ps256(vl), (vh), 1)
#endif


#define SIMD_ALIGN_16(code) NV_ALIGN_PREFIX(16) code NV_ALIGN_SUFFIX(16)
#define SIMD_ALIGN_32(code) NV_ALIGN_PREFIX(32) code NV_ALIGN_SUFFIX(32)

inline __m128   add(const __m128& a, const __m128& b)   { return _mm_add_ps(a, b); }
inline __m128   add(float a, const __m128& b)           { return _mm_add_ps(_mm_load1_ps(&a), b); }
inline __m128   add(const __m128& a, float b)           { return _mm_add_ps(a, _mm_load1_ps(&b)); }
inline float    add(float a, float b)                   { return a + b; }

inline __m128   sub(const __m128& a, const __m128& b)   { return _mm_sub_ps(a, b); }
inline __m128   sub(float a, const __m128& b)           { return _mm_sub_ps(_mm_load1_ps(&a), b); }
inline __m128   sub(const __m128& a, float b)           { return _mm_sub_ps(a, _mm_load1_ps(&b)); }
inline float    sub(float a, float b)                   { return a - b; }

inline __m128   mul(const __m128& a, const __m128& b)   { return _mm_mul_ps(a, b); }
inline __m128   mul(float a, const __m128& b)           { return _mm_mul_ps(_mm_load1_ps(&a), b); }
inline __m128   mul(const __m128& a, float b)           { return _mm_mul_ps(a, _mm_load1_ps(&b)); }
inline float    mul(float a, float b)                   { return a * b; }

inline __m128   div(const __m128& a, const __m128& b)   { return _mm_div_ps(a, b); }
inline __m128   div(float a, const __m128& b)           { return _mm_div_ps(_mm_load1_ps(&a), b); }
inline __m128   div(const __m128& a, float b)           { return _mm_div_ps(a, _mm_load1_ps(&b)); }
inline float    div(float a, float b)                   { return a / b; }

inline bool     lt(const __m128& a, const __m128& b)    { return !!_mm_comilt_ss(a, b); }
inline bool     gt(const __m128& a, const __m128& b)    { return !!_mm_comigt_ss(a, b); }
inline bool     le(const __m128& a, const __m128& b)    { return !!_mm_comile_ss(a, b); }
inline bool     ge(const __m128& a, const __m128& b)    { return !!_mm_comige_ss(a, b); }
inline bool     eq(const __m128& a, const __m128& b)    { return !!_mm_comieq_ss(a, b); }
inline bool     ne(const __m128& a, const __m128& b)    { return !!_mm_comineq_ss(a, b); }

inline bool     lt(const float a, const float b)        { return a < b; }
inline bool     gt(const float a, const float b)        { return a > b; }
inline bool     le(const float a, const float b)        { return a <= b; }
inline bool     ge(const float a, const float b)        { return a >= b; }
inline bool     eq(const float a, const float b)        { return a == b; }
inline bool     ne(const float a, const float b)        { return a != b; }

inline float to_float(const __m128& x) { float f; _mm_store_ss(&f, x); return f; }
inline float to_float(float x) { return x; }

inline void from_float(__m128& x, float y) { x = _mm_load1_ps(&y); }
inline void from_float(float& x, float y) { x = y; }

inline void set_zero(__m128& x) { x = _mm_setzero_ps(); }
inline void set_zero(float& x) { x = 0.0f; }

inline void store_float(float* mem, const __m128& f) { _mm_store_ps(mem, f); }
inline void store_float(float* mem, float f) { *mem = f; }

inline void load_float(__m128& f, const float* mem) { f = _mm_load_ps(mem); }
inline void load_float(float& f, const float* mem) { f = *mem; }

inline __m128 prep_cross3(const __m128& v) { return _mm_shuffle_ps(v, v, 0xc9); } // w z y x -> w x z y

inline __m128
cross3(const __m128& v0, const __m128& v1)
{
    __m128 prep0 = prep_cross3(v0);
    __m128 prep1 = prep_cross3(v1);
    __m128 res_shuffled = _mm_sub_ps(_mm_mul_ps(v0, prep1), _mm_mul_ps(prep0, v1));
    return _mm_shuffle_ps(res_shuffled, res_shuffled, 0xc9);
}

inline __m128
cross3_prep0(const __m128& v0, const __m128& prep0, const __m128& v1)
{
    __m128 prep1 = prep_cross3(v1);
    __m128 res_shuffled = _mm_sub_ps(_mm_mul_ps(v0, prep1), _mm_mul_ps(prep0, v1));
    return _mm_shuffle_ps(res_shuffled, res_shuffled, 0xc9);
}

inline __m128
cross3_prep1(const __m128& v0, const __m128& v1, const __m128& prep1)
{
    __m128 prep0 = prep_cross3(v0);
    __m128 res_shuffled = _mm_sub_ps(_mm_mul_ps(v0, prep1), _mm_mul_ps(prep0, v1));
    return _mm_shuffle_ps(res_shuffled, res_shuffled, 0xc9);
}
