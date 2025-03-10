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

struct SparseShaderParams
{
    NvFlowUint uploadBlockIdxOffset;
    NvFlowUint sparseBlockIdxOffset;
    NvFlowUint uploadSizeInBytes;
    NvFlowUint pad3;
    NvFlowSparseLevelParams tableParams;
    NvFlowSparseLevelParams blockLevel[16u];
};

struct SparseClearTextureParams
{
    NvFlowInt3 clearOffset;
    int pad0;
    NvFlowInt3 clearExtent;
    int pad1;
};

struct SparseClearNewParams
{
    NvFlowUint blockIdxOffset;
    NvFlowUint pad1;
    NvFlowUint pad2;
    NvFlowUint pad3;
    NvFlowSparseLevelParams tableParams;
};

struct SparseRescaleGlobalParams
{
    NvFlowUint blockIdxOffset;
    NvFlowUint pad1;
    NvFlowUint pad2;
    NvFlowUint pad3;
    NvFlowSparseLevelParams tableParams;
    NvFlowSparseLevelParams tableParamsOld;
};

struct SparseRescaleLayerParams
{
    NvFlowFloat3 blockSizeWorld;
    NvFlowBool32 shouldClear;
    NvFlowFloat3 blockSizeWorldOld;
    float pad2;
};

struct SparseNanoVdbComputeStatsParams
{
    NvFlowUint blockIdxOffset0;
    NvFlowUint blockIdxOffset1;
    NvFlowUint blockIdxOffset2;
    NvFlowUint blockIdxOffset3;
    NvFlowSparseNanoVdbParams nanoVdb;
};

struct SparseNanoVdbExportParams
{
    NvFlowUint blockIdxOffset;
    NvFlowUint pad1;
    NvFlowUint pad2;
    NvFlowUint pad3;
    NvFlowSparseLevelParams tableVelocity;
    NvFlowSparseLevelParams tableDensity;
    NvFlowSparseNanoVdbParams velocityFloat;
    NvFlowSparseNanoVdbParams velocityVec3;
    NvFlowSparseNanoVdbParams densityFloat;
    NvFlowSparseNanoVdbParams rgba;
    NvFlowSparseNanoVdbParams rgb;
    NvFlowUint temperatureEnabled;
    NvFlowUint fuelEnabled;
    NvFlowUint burnEnabled;
    NvFlowUint smokeEnabled;
    NvFlowUint velocityEnabled;
    NvFlowUint divergenceEnabled;
    NvFlowUint rgbaEnabled;
    NvFlowUint rgbEnabled;
};
