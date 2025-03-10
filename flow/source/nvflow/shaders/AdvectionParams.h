// SPDX-FileCopyrightText: Copyright (c) 2014-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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

#include "NvFlowShaderTypes.h"

struct AdvectionCombustionParams
{
    NvFlowFloat3 gravity;
    float burnPerFuel;

    float ignitionTemp;
    float burnPerTemp;
    float fuelPerBurn;
    float tempPerBurn;

    float smokePerBurn;
    float divergencePerBurn;
    float buoyancyPerTemp;
    float coolingRate;

    float buoyancyPerSmoke;
    float buoyancyMaxSmoke;
    NvFlowUint combustionEnabled;
    float pad3;
};

struct SemiLagrangianParams
{
    NvFlowFloat3 valueToVelocityBlockScale;
    NvFlowUint globalFetch;
    NvFlowFloat3 cellSizeInv;
    float deltaTime;
    NvFlowFloat3 maxDisplacement;
    float pad0;
};

struct MacCormackParams
{
    SemiLagrangianParams base;
    NvFlowFloat4 blendThreshold;
    NvFlowFloat4 blendFactor;
    NvFlowFloat4 dampingRate;
    NvFlowFloat4 fadeRate;
};

struct AdvectionDensityCS_GlobalParams
{
    NvFlowUint blockIdxOffset;
    NvFlowUint pad1;
    NvFlowUint pad2;
    NvFlowUint pad3;
    NvFlowSparseLevelParams table;
    NvFlowSparseLevelParams tableVelocity;
};

struct AdvectionDensityCS_LayerParams
{
    MacCormackParams advectParams;
    AdvectionCombustionParams combustParams;
};

struct AdvectionSimpleCS_GlobalParams
{
    NvFlowUint blockIdxOffset;
    NvFlowUint pad1;
    NvFlowUint pad2;
    NvFlowUint pad3;
    NvFlowSparseLevelParams table;
};

struct AdvectionSimpleCS_LayerParams
{
    SemiLagrangianParams advectParams;
    NvFlowFloat4 dampingRate;
};

struct AdvectionVelocityCS_GlobalParams
{
    NvFlowUint blockIdxOffset;
    NvFlowUint pad1;
    NvFlowUint pad2;
    NvFlowUint pad3;
    NvFlowSparseLevelParams table;
    NvFlowSparseLevelParams tableDensity;
};

struct AdvectionVelocityCS_LayerParams
{
    MacCormackParams advectParams;
    AdvectionCombustionParams combustParams;
    NvFlowFloat3 velocityToDensityBlockScale;
    float pad0;
};
