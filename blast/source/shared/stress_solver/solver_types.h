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

#include "NvCTypes.h"

#include "simd/simd.h"


/**
 * Scalar types for SIMD and non-SIMD calculations.
 * Currently also used as a template argument to distinguish code paths.  May need a different
 * scheme if two codepaths use the same scalar type.
 */
typedef __m128  SIMD_Scalar;
typedef float   Float_Scalar;


/**
 * Holds the components of a rigid body description that are necessary for the stress solver.
 */
template<typename InertiaType>
struct SolverNode
{
    NvcVec3     CoM;
    float       mass;
    InertiaType inertia;
};

typedef SolverNode<float>       SolverNodeS;
typedef SolverNode<NvcVec3>     SolverNodeD;
typedef SolverNode<NvcMat33>    SolverNodeG;


/**
 * Holds the components of a rigid body bond description that are necessary for the stress solver.
 */
struct SolverBond
{
    NvcVec3     centroid;
    uint32_t    nodes[2];   // Index into accompanying SolverNode<InertiaType> array.
};
