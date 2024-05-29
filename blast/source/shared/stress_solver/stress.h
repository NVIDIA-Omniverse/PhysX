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

#include "bond.h"
#include "buffer.h"


class StressProcessor
{
public:
    /** Constructor clears member data. */
    StressProcessor() : m_mass_scale(0.0f), m_length_scale(0.0f), m_can_resume(false) {}

    /** Parameters controlling the data preparation. */
    struct DataParams
    {
        bool        equalizeMasses  = false;        // Use the geometric mean of the nodes' masses instead of the individual masses.
        bool        centerBonds     = false;        // Place the bond position halfway between adjoining nodes' CoMs.
    };

    /** Parameters controlling the solver behavior. */
    struct SolverParams
    {
        uint32_t    maxIter         = 0;        // The maximum number of iterations.  If 0, use CGNR for default value.
        float       tolerance       = 1.e-6f;   // The relative tolerance threshold for convergence.  Iteration will stop when this is reached.
        bool        warmStart       = false;    // Whether or not to use the solve function's 'impulses' parameter as a starting input vector.
    };

    /**
     * Build the internal representation of the stress network from nodes and bonds.
     * This only needs to be called initially, and any time the nodes or bonds change.
     * 
     * \param[in]   nodes   Array of SolverNodeS (scalar inertia).
     * \param[in]   N_nodes Number of elements in the nodes array.
     * \param[in]   bonds   Array of SolverBond.  The node indices in each bond entry correspond to the ordering of the nodes array.
     * \param[in]   N_bonds Number of elements in the bonds array.
     * \param[in]   params  Parameters affecting the processing of the input data (see DataParams).
     */
    void        prepare(const SolverNodeS* nodes, uint32_t N_nodes, const SolverBond* bonds, uint32_t N_bonds, const DataParams& params);

    /**
     * Solve for the bond impulses given the velocities of each node.  The function prepare(...) must be called
     * before this can be used, but then solve(...) may be called multiple times.
     * 
     * The vector elements (impulses and velocities) hold linear and angular parts.
     * 
     * \param[out]  impulses    Output array of impulses exerted by each bond.  For a warm or hot start, this is also used as an input.
     *                          Must be of length N_bonds passed into the prepare(...) function.
     * \param[in]   velocities  Input array of external velocities on each node.  Must be of length N_nodes passed into the prepare(...) function.
     * \param[in]   params      Parameters affecting the solver characteristics (see SolverParams).
     * \param[out]  error_sq    (Optional) If not NULL, *error_sq will be filled with the angular and linear square errors (solver residuals).  Default = NULL.
     * \param[in]   resume      (Optional) Set to true if impulses and velocities have not changed since last call, to resume solving.  Default = false.
     * 
     * \return the number of iterations taken to converge, if it converges.  Otherwise, returns minus the number of iterations before exiting.
     */
    int         solve(AngLin6* impulses, const AngLin6* velocities, const SolverParams& params, AngLin6ErrorSq* error_sq = nullptr, bool resume = false);

    /**
     * Removes the indexed bond from the solver.
     * 
     * \param[in]   bondIndex   The index of the bond to remove.  Must be less than getBondCount().
     * 
     * \return true iff successful.
     */
    bool        removeBond(uint32_t bondIndex);

    /**
     * \return the number of nodes in the stress network.  (Set by prepare(...).)
     */
    uint32_t    getNodeCount() const { return (uint32_t)m_recip_sqrt_I.size(); }

    /**
     * \return the number of bonds in the stress network.  (Set by prepare(...), possibly reduced by removeBond(...).)
     */
    uint32_t    getBondCount() const { return (uint32_t)m_couplings.size(); }

    /**
     * \return whether or not the solver uses SIMD.  If the device and OS support SSE, AVX, and FMA instruction sets, SIMD is used. 
     */
    static bool usingSIMD() { return s_use_simd; }

protected:
    float                   m_mass_scale;
    float                   m_length_scale;
    POD_Buffer<InertiaS>    m_recip_sqrt_I;
    POD_Buffer<Coupling>    m_couplings;
    BondMatrixS             m_B;
    POD_Buffer<AngLin6>     m_rhs;
    POD_Buffer<AngLin6>     m_B_scratch;
    POD_Buffer<AngLin6>     m_solver_cache;
    bool                    m_can_resume;

    static const bool       s_use_simd;
};
