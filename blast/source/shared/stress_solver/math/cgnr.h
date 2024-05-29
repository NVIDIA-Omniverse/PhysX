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

#include <stdint.h>
#include <cstring>  // for memcpy, memset

#include "simd/simd.h"


template<typename Elem, typename ElemOps, typename Mat, typename MatOps, typename Scalar = float, typename Error = float>
struct CGNR
{
    /**
     * Conjugate Gradient Normal Equation Residual (CGNR) solver for systems of M equations and N unknowns.
     * 
     * Based on Matrix Computations (4th ed.) by Golub and Van Loan, section 11.3.9.
     * 
     * Solves A*x = b.
     * 
     * Template arguments:
     *  Elem: the type of element used in the vectors x and b, and (implicitly) in the matrix A.
     * 
     *  ElemOps: a class which defines various functions on Elem type and vectors of Elem type.
     * 
     *  Mat: the explicit type used to represent the matrix, allowing e.g. for sparse representations.
     * 
     *  MatOps: a class which defines the functions rmul and lmul, which multiply a matrix of type Mat
     *      by an Elem-typed vector on the right and left, respectively.  The function signatures must be:
     * 
     *      void rmul(Elem* y, const Mat& A, const Elem* x, uint32_t M, uint32_t N);    // y = A*x
     *      void lmul(Elem* y, const Elem* x, const Mat& A, uint32_t M, uint32_t N);    // y = x*A
     * 
     *  Scalar: set to float by default.  May be used to keep all operations in a particular representation, e.g. SIMD registers.
     * 
     * \param[out]  x           User-supplied Elem vector of length N, filled with the solution upon exit (if successful).
     * \param[in]   A           System M x N matrix of type Mat.
     * \param[in]   b           Right hand side of equation to be solved, an Elem vector of length M.
     * \param[in]   M           The number of rows in A and elements in b.
     * \param[in]   N           The number of columns in A and elements in x.
     * \param[in]   cache       Cache memory provided by the user, must be at least required_cache_size(M, N) bytes, and sizeof(Elem)-byte aligned.
     * \param[out]  error_ptr   If not null, returns the square magnitude error calculated from residual.
     * \param[in]   tol         (Optional) relative convergence threshold for |(A^T)*(A*x-b)|/|b|.  Default value is 10^-6.
     * \param[in]   max_it      (Optional) the maximum number of internal iterations.  If set to 0, the maximum is N.  Default value is 0.
     * \param[in]   warmth      (Optional) valid values are 0, 1, and 2.  0 => cold, clears the x vector and ignores the cache.
     *                          1 => warm, uses the x vector as a starting solution, but still ignores the cache.  2 => hot, uses the x
     *                          vector as a starting solution, and the cache must be valid.  Default value is 0.
     *                          N.B. if warmth == 2, then this function must have been called previously, and the equation values
     *                          (x, A, b, M, and N) as well as the cache must not have been changed since the last call.
     * 
     * return the number of iterations taken to converge, if it converges.  Otherwise, returns minus the number of iterations before exiting.
     */
    int
    solve
    (
        Elem* x,
        const Mat& A,
        const Elem* b,
        uint32_t M,
        uint32_t N,
        void* cache,
        Error* error_ptr = nullptr,
        float tol = 1.e-6f,
        uint32_t max_it = 0,
        unsigned warmth = 0
    )
    {
        // Cache and temporary storage
        static_assert(sizeof(Elem) >= sizeof(Scalar), "sizeof(Elem) must be at least as great as sizeof(Scalar).");
        float* z_last_sq_mem = (float*)cache; cache = (Elem*)z_last_sq_mem + 1; // Elem-sized storage
        float* delta_sq_mem = (float*)cache; cache = (Elem*)delta_sq_mem + 1;   // Elem-sized storage
        Elem* z = (Elem*)cache; cache = z + N;  // Array of length N
        Elem* p = (Elem*)cache; cache = p + N;  // Array of length N
        Elem* r = (Elem*)cache; cache = r + M;  // Array of length M
        Elem* s = (Elem*)cache;                 // Array of length M

        Scalar z_last_sq, delta_sq;
        load_float(z_last_sq, z_last_sq_mem);
        load_float(delta_sq, delta_sq_mem);

        if (warmth < 2)                                         // Not hot
        {
            delta_sq = mul(tol*tol, ElemOps().length_sq(b, M)); // Calculate allowed residual length squared and cache it
            store_float(delta_sq_mem, delta_sq);
            memcpy(r, b, sizeof(Elem)*M);                       // Initialize residual r = b
            if (warmth)                                         // Warm start, r = b - A*x
            {
                MatOps().rmul(s, A, x, M, N);
                ElemOps().vsub(r, r, s, M);
            }
            else memset(x, 0, sizeof(Elem)*N);                  // Cold start, x = 0 so r = b
            warmth = 0;                                         // This lets p be initialized in the loop below
        }

        Error error;

        // Iterate
        if (!max_it) max_it = N;                                                        // Default to a maximum of N iterations
        uint32_t it = 0;
        do
        {
            MatOps().lmul(z, r, A, M, N);                                               // Set z = (A^T)*r
            const Scalar z_sq = ElemOps().calculate_error(error, z, N);                 // Calculate residual (of modified equation) length squared
            if (le(z_sq, delta_sq)) break;                                              // Terminate (convergence) if within tolerance
            if (warmth || warmth++) ElemOps().vmadd(p, div(z_sq, z_last_sq), p, z, N);  // If not cold set p = z + (|z|^2/|z_last|^2)*p, and make warm hereafter
            else memcpy(p, z, sizeof(Elem)*N);                                          // If cold set p = z
            z_last_sq = z_sq;
            MatOps().rmul(s, A, p, M, N);                                               // Calculate s = A*p
            const Scalar mu = div(z_sq, ElemOps().length_sq(s, M));                     // mu = |z|^2 / |A*p|^2
            ElemOps().vmadd(x, mu, p, x, N);                                            // x += mu*p
            ElemOps().vnmadd(r, mu, s, r, M);                                           // r -= mu*s
        } while (++it < max_it);

        // Store off remainder of state (the rest was maintained in memory with array operations)
        store_float(z_last_sq_mem, z_last_sq);

        // Store off the error if requested
        if (error_ptr) *error_ptr = error;

        // Return the number of iterations used if successful.  Otherwise return minus the number of iterations performed
        return it < max_it ? (int)it : -(int)it;
    }

    /**
     * \param[in]   M   See solve(...) for a description.
     * \param[in]   N   See solve(...) for a description.
     * 
     * \return the required cache size (in bytes) for the given values of M and N.
     */
    size_t  required_cache_size(uint32_t M, uint32_t N) { return 2*(M+N+1)*sizeof(Elem); }
};
