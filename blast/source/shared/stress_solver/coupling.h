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
#include "anglin6.h"

#include "NvCMath.h"


/**
 * Bond coupling data used as a representation of a block column of a "coupling matrix" C,
 * which has exactly two non-zero blocks.  The non-zero blocks are of the form
 * 
 *               /   1  ~r_ij \
 *   C_ij = s_ij |            |.
 *               \   0    1   /
 * 
 * This represents the coupling of node i by bond j.  The scalar s_ij is +/-1, and for each
 * bond (column j of C) s_ij must take on both signs.  The matrix factor is again composed
 * of blocks, each element a 3x3 matrix.  The 0 and 1's are just multiples of the unit matrix,
 * and ~r_ij is the 3x3 antisymmetric matrix representing "crossing with the vector r_ij on the
 * left" (i.e. (~u)*v = (u) x (v)).  The vector r_ij represents the displacement from node i's
 * CoM to bond j's centroid.
 */

SIMD_ALIGN_32
(
struct Coupling
{
    NvcVec3 offset0;
    uint32_t node0;
    NvcVec3 offset1;
    uint32_t node1;
}
);


template <typename Elem, typename Scalar = Float_Scalar>
struct CouplingMatrixOps
{
    /**
     * Sparse matrix-vector multiply y = C*x, where C is a "coupling matrix" represented by columns
     * of type Coupling (see the comments for Coupling).
     *
     * \param[out]  y   Resulting column Elem vector of length M.
     * \param[in]   C   Input M x N coupling matrix.
     * \param[in]   x   Input column Elem vector of length N.
     * \param[in]   M   The number of rows in y and C.
     * \param[in]   N   The number of rows in x and columns in C.
     */
    inline void
    rmul(Elem* y, const Coupling* C, const Elem* x, uint32_t M, uint32_t N)
    {
        memset(y, 0, sizeof(AngLin6)*M);
        for (uint32_t j = 0 ; j < N; ++j)
        {
            const Coupling& c = C[j];
            const AngLin6& x_j = x[j];
            AngLin6& y0 = y[c.node0];
            AngLin6& y1 = y[c.node1];
            y0.ang += x_j.ang - (c.offset0^x_j.lin);
            y0.lin += x_j.lin;
            y1.ang -= x_j.ang - (c.offset1^x_j.lin);
            y1.lin -= x_j.lin;
        }
    }

    /**
     * Sparse matrix-vector multiply y = x*C, where C is a "coupling matrix" represented by columns
     * of type Coupling (see the comments for Coupling).
     *
     * \param[out]  y   Resulting row Elem vector of length N.
     * \param[in]   x   Input row Elem vector, must be long enough to be indexed by all values in B's representation.
     * \param[in]   C   Input M x N couping matrix.
     * \param[in]   M   The number of columns in x and rows in C.
     * \param[in]   N   The number of columns in y and C.
     */
    inline void
    lmul(Elem* y, const Elem* x, const Coupling* C, uint32_t M, uint32_t N)
    {
        NV_UNUSED(M);
        for (uint32_t j = 0; j < N; ++j)
        {
            const Coupling& c = C[j];
            const AngLin6& x0 = x[c.node0];
            const AngLin6& x1 = x[c.node1];
            AngLin6& y_j = y[j];
            y_j.ang = x0.ang - x1.ang;
            y_j.lin = x0.lin - x1.lin + (c.offset0^x0.ang) - (c.offset1^x1.ang);
        }
    }
};

template <typename Elem>
struct CouplingMatrixOps<Elem, SIMD_Scalar>
{
    /**
     * Sparse matrix-vector multiply y = C*x, where C is a "coupling matrix" represented by columns
     * of type Coupling (see the comments for Coupling).
     *
     * \param[out]  y   Resulting column Elem vector of length M.
     * \param[in]   C   Input M x N coupling matrix.
     * \param[in]   x   Input column Elem vector of length N.
     * \param[in]   M   The number of rows in y and C.
     * \param[in]   N   The number of rows in x and columns in C.
     */
    inline void
    rmul(Elem* y, const Coupling* C, const Elem* x, uint32_t M, uint32_t N)
    {
        memset(y, 0, sizeof(AngLin6)*M);
        for (uint32_t j = 0 ; j < N; ++j)
        {
            const Coupling& c = C[j];
            const AngLin6& x_j = x[j];
            AngLin6& y0 = y[c.node0];
            AngLin6& y1 = y[c.node1];

            __m256 _x = _mm256_load_ps(&x_j.ang.x);
            __m256 _y0 = _mm256_load_ps(&y0.ang.x);
            __m256 _y1 = _mm256_load_ps(&y1.ang.x);
            __m256 _c = _mm256_load_ps(&c.offset0.x);

            _y0 = _mm256_add_ps(_y0, _x);
            _y1 = _mm256_sub_ps(_y1, _x);

            __m128 _xl = _mm256_extractf128_ps(_x, 1);
            __m256 _a = pair_cross3(_mm256_set_m128(_xl, _xl), _c);
            _y0 = _mm256_add_ps(_y0, _mm256_set_m128(_mm_setzero_ps(), _mm256_castps256_ps128(_a)));
            _y1 = _mm256_sub_ps(_y1, _mm256_set_m128(_mm_setzero_ps(), _mm256_extractf128_ps(_a, 1)));

            _mm256_store_ps(&y0.ang.x, _y0);
            _mm256_store_ps(&y1.ang.x, _y1);
        }
    }

    /**
     * Sparse matrix-vector multiply y = x*C, where C is a "coupling matrix" represented by columns
     * of type Coupling (see the comments for Coupling).
     *
     * \param[out]  y   Resulting row Elem vector of length N.
     * \param[in]   x   Input row Elem vector, must be long enough to be indexed by all values in B's representation.
     * \param[in]   C   Input M x N couping matrix.
     * \param[in]   M   The number of columns in x and rows in C.
     * \param[in]   N   The number of columns in y and C.
     */
    inline void
    lmul(Elem* y, const Elem* x, const Coupling* C, uint32_t M, uint32_t N)
    {
        NV_UNUSED(M);
        for (uint32_t j = 0; j < N; ++j)
        {
            const Coupling& c = C[j];
            const AngLin6& x0 = x[c.node0];
            const AngLin6& x1 = x[c.node1];
            AngLin6& y_j = y[j];

            __m256 _x0 = _mm256_load_ps(&x0.ang.x);
            __m256 _x1 = _mm256_load_ps(&x1.ang.x);
            __m256 _c = _mm256_load_ps(&c.offset0.x);

            __m256 _y = _mm256_sub_ps(_x0, _x1);

            __m256 _a = pair_cross3(_c, _mm256_set_m128(_mm256_castps256_ps128(_x1), _mm256_castps256_ps128(_x0)));
            _y = _mm256_add_ps(_y, _mm256_set_m128(_mm_sub_ps(_mm256_castps256_ps128(_a), _mm256_extractf128_ps(_a, 1)), _mm_setzero_ps()));

            _mm256_store_ps(&y_j.ang.x, _y);
        }
    }

private:
    inline __m256
    pair_cross3(const __m256& v0, const __m256& v1)
    {
        __m256 prep0 = _mm256_shuffle_ps(v0, v0, 0xc9);
        __m256 prep1 = _mm256_shuffle_ps(v1, v1, 0xc9);
        __m256 res_shuffled = _mm256_fmsub_ps(v0, prep1, _mm256_mul_ps(prep0, v1));
        return _mm256_shuffle_ps(res_shuffled, res_shuffled, 0xc9);
    }
};
