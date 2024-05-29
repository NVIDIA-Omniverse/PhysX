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

#include "solver_types.h"

#include "NvCMath.h"


/**
 * Holds an inertia component and a mass component.
 * The inertial component is represented by a TensorType, which may be a float (representing a multiple of
 * the unit matrix), an NvcVec3 (representing the non-zero components of a diagonal inertia tensor), or a
 * 3x3 symmetric matrix representing a general inertia tensor.
 * 
 * This structure might also be used to store reciprocals, or powers (e.g. square roots) of these quantities.
 */
template <typename TensorType>
struct Inertia
{
    TensorType  I;  
    float       m;
};

typedef Inertia<float>      InertiaS;
typedef Inertia<NvcVec3>    InertiaD;
typedef Inertia<NvcMat33>   InertiaG;


template<typename Scalar = Float_Scalar>
struct InertiaMatrixOps
{
    /**
     * Matrix-vector multiply y = I*x.
     * 
     * Apply a block-diagonal inertia matrix I to a vector of AngLin6 elements.
     * x and y may be the same vector.
     * 
     * \param[out]  y   Resulting column vector of length N.
     * \param[in]   I   Input inertia matrix representation.
     * \param[in]   x   Input column vector of length N.
     * \param[in]   N   Number of columns in x and y, and the square size of I.
     *
     * x and y may be the same vector.
     */
    inline void
    mul(AngLin6* y, const InertiaS* I, const AngLin6* x, uint32_t N)
    {
        for (uint32_t i = 0; i < N; ++i)
        {
            const InertiaS& I_i = I[i];
            const AngLin6& x_i = x[i];
            AngLin6& y_i = y[i];
            y_i.ang = I_i.I*x_i.ang;
            y_i.lin = I_i.m*x_i.lin;
        }
    }
};

template<>
struct InertiaMatrixOps<SIMD_Scalar>
{
    /**
     * Matrix-vector multiply y = I*x.
     * 
     * Apply a block-diagonal inertia matrix I to a vector of AngLin6 elements.
     * 
     * \param[out]  y   Resulting column vector of length N.
     * \param[in]   I   Input inertia matrix representation.
     * \param[in]   x   Input column vector of length N.
     * \param[in]   N   Number of columns in x and y, and the square size of I.
     *
     * x and y may be the same vector.
     */
    inline void
    mul(AngLin6* y, const InertiaS* I, const AngLin6* x, uint32_t N)
    {
        for (uint32_t i = 0; i < N; ++i)
        {
            const InertiaS& I_i = I[i];
            const AngLin6& x_i = x[i];
            AngLin6& y_i = y[i];

            __m256 _x = _mm256_load_ps(&x_i.ang.x);
            __m128 _Il = _mm_load1_ps(&I_i.I);
            __m128 _Ih = _mm_load1_ps(&I_i.m);
            __m256 _I = _mm256_set_m128(_Ih,_Il);
            __m256 _y = _mm256_mul_ps(_I, _x);
            _mm256_store_ps(&y_i.ang.x, _y);
        }
    }
};
