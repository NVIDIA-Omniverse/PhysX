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

#include "coupling.h"
#include "inertia.h"
#include "anglin6.h"


/**
 * BondMatrix
 * 
 * Given a BondMatrix B, when (B^T)*B is applied to a vector of bond impulses, the result
 * is a vector of the differences between the the resulting accelerations of the nodes
 * joined by each bond.
 * 
 * This is done in block form, so a vector is composed of vector elements.  Each element
 * is a 6-dimensional vector, composed of a linear part followed by an angular part.
 * Matrix blocks are likewise 6x6.
 * 
 * This matrix is composed of two sparse matrices:
 *    An M x M block diagonal matrix I, where the i^th diagonal block is the 6x6 matrix:
 * 
 *             / I_i  0  \
 *      I_ii = |         |
 *             \  0  m_i /
 * 
 *    Except for possibly I_i, each "element" in I_ii is a multiple of the 3x3 unit matrix.  I_i is a
 *    3x3 symmetric inertia tensor.  See the definition of Inertia<TensorType> for its representation.
 * 
 *    The second component is the coupling matrix C, see documentation for Coupling.
 * 
 * The matrix represented by this object is (I^-1/2)*C, an M x N matrix.
 * 
 * NOTE: I and C are _not_ stored as described above, for efficiency.
 */
template <typename TensorType>
struct BondMatrix
{
    /** Constructor clears member data. */
    BondMatrix() : C(nullptr), sqrt_I_inv(nullptr), scratch(nullptr), M(0), N(0) {}

    /**
     * Set fields (shallow pointer copy).
     * 
     * \param[in]   _C          Coupling matrix, see the documentation for Coupling.
     * \param[in]   _sqrt_I_inv The inverse of the square root of the diagonal mass and inertia tensor, represented by a
     *                          vector of _M Inertia structs for the diagonal values.  The i^th element is the reciprocal
     *                          of the square root of the mass and inertia tensor of node i.
     * \param[in]   _scratch    Scratch memory required to carry out a multiply.  Must be at least _M*sizeof(AngLin6) bytes.
     * \param[in]   _M          The number of nodes.
     * \param[in]   _N          The number of bonds.
     */
    void
    set(const Coupling* _C, const Inertia<TensorType>* _sqrt_I_inv, void* _scratch, uint32_t _M, uint32_t _N)
    {
        C = _C;
        sqrt_I_inv = _sqrt_I_inv;
        scratch = _scratch;
        M = _M;
        N = _N;
    }

    const Coupling* C;
    const Inertia<TensorType>* sqrt_I_inv;
    void* scratch;
    uint32_t M, N;
};

typedef BondMatrix<float>       BondMatrixS;
typedef BondMatrix<NvcVec3>     BondMatrixD;
typedef BondMatrix<NvcMat33>    BondMatrixG;


template<typename TensorType, typename Scalar>
struct BondMatrixOps
{
    /**
     * Matrix-vector multiply y = B*x.
     * 
     * \param[out]  y   Resulting column vector of length N.
     * \param[in]   B   Input MxN matrix representation.
     * \param[in]   x   Input column vector of length M.
     * \param[in]   M   Number of rows in B.
     * \param[in]   N   Number of columns in B.
     */
    inline void
    rmul(AngLin6* y, const BondMatrix<TensorType>& B, const AngLin6* x, uint32_t M, uint32_t N) const
    {
        NV_UNUSED(M);   // BondMatrix stores these
        NV_UNUSED(N);

        // Calculate y = C*x (apply C)
        CouplingMatrixOps<AngLin6, Scalar>().rmul(y, B.C, x, B.M, B.N);

        // Calculate y = (I^-1/2)*C*x (apply I^-1/2)
        InertiaMatrixOps<Scalar>().mul(y, B.sqrt_I_inv, y, B.M);
    }

    /**
     * Matrix-vector multiply y = x*B.
     * 
     * \param[out]  y   Resulting row vector of length B.N.
     * \param[in]   x   Input row vector of length B.N.
     * \param[in]   B   Input matrix representation.
     * \param[in]   M   Number of rows in B.
     * \param[in]   N   Number of columns in B.
     */
    inline void
    lmul(AngLin6* y, const AngLin6* x, const BondMatrix<TensorType>& B, uint32_t M, uint32_t N) const
    {
        NV_UNUSED(M);   // BondMatrix stores these
        NV_UNUSED(N);

        AngLin6* s = (AngLin6*)B.scratch; // M-sized scratch s

        // Calculate s = (I^-1/2)*x (apply I^-1/2)
        InertiaMatrixOps<Scalar>().mul(s, B.sqrt_I_inv, x, B.M);

        // Calculate y = (C^T)*(I^-1/2)*x (apply C^T)
        CouplingMatrixOps<AngLin6, Scalar>().lmul(y, s, B.C, B.M, B.N);
    }
};

template<typename Scalar>
using BondMatrixOpsS = BondMatrixOps<float, Scalar>;

template<typename Scalar>
using BondMatrixOpsD = BondMatrixOps<float, NvcVec3>;

template<typename Scalar>
using BondMatrixOpsG = BondMatrixOps<float, NvcMat33>;
