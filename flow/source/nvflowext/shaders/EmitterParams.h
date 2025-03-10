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

#define EMITTER_MAX_SUBSTEPS 32

struct EmitterSphereCS_Params
{
    NvFlowUint blockIdxOffset;
    NvFlowUint pad1;
    NvFlowUint pad2;
    NvFlowUint pad3;

    NvFlowSparseLevelParams table;

    NvFlowFloat4 targetValue;
    NvFlowFloat4 coupleRate;

    NvFlowFloat3 vidxToWorld;
    float pad4;

    NvFlowFloat4x4 worldToLocals[EMITTER_MAX_SUBSTEPS];
    NvFlowFloat4x4 localToWorldOlds[EMITTER_MAX_SUBSTEPS];

    float physicsVelocityScale;
    float physicsDeltaTimeInv;
    NvFlowUint multisample;
    NvFlowUint numSubSteps;

    float numSubStepsInv;
    float radius;
    float radius2;
    float deltaTime;

    NvFlowFloat3 position;
    float pad5;

    NvFlowInt4 locationOffset;
    NvFlowUint4 locationExtent;

    NvFlowSparseLevelParams velocityTable;

    NvFlowFloat3 cellSizeInv;
    NvFlowUint numTraceSamples;
    NvFlowFloat3 valueToVelocityBlockScale;
    float traceDeltaTime;
};

struct EmitterBoxCS_Params
{
    NvFlowUint blockIdxOffset;
    NvFlowUint pad1;
    NvFlowUint instanceCount;
    float deltaTime;

    NvFlowSparseLevelParams table;
};

struct EmitterBoxCS_InstanceParams
{
    NvFlowFloat4 targetValue;
    NvFlowFloat4 coupleRate;

    NvFlowFloat3 vidxToWorld;
    int layerAndLevel;

    NvFlowFloat4x4 worldToLocal;
    NvFlowFloat4x4 localToWorldOld;

    NvFlowFloat3 halfSize;
    NvFlowUint enabled;

    NvFlowFloat3 position;
    NvFlowUint clippingPlaneCountCount;

    float physicsVelocityScale;
    float physicsDeltaTimeInv;
    NvFlowUint multisample;
    NvFlowUint clippingPlaneCount;

    NvFlowUint clippingPlanesOffset;
    NvFlowUint clippingPlaneCountsOffset;
    float pad1;
    float pad2;
};

struct EmitterPointCS_Params
{
    NvFlowUint pointBlockIdxOffset;
    NvFlowUint reduce2BlockIdxOffset;
    NvFlowUint reduce3BlockIdxOffset;
    NvFlowUint velocityIsWorldSpace;

    NvFlowSparseLevelParams table;
    NvFlowSparseLevelParams tableCoarse;

    NvFlowFloat4x4 localToWorld;
    NvFlowFloat4x4 localToWorldVelocity;

    NvFlowFloat4 defaultTargetValue;
    NvFlowFloat4 defaultCoupleRate;
    NvFlowFloat4 targetValueScale;

    int layerAndLevel;
    NvFlowUint defaultAllocateMask;
    NvFlowUint allocateMaskCount;
    NvFlowUint needsSrgbConversion;

    NvFlowFloat3 vidxToWorld;
    float deltaTime;

    NvFlowFloat3 worldToVidx;
    NvFlowUint isVelocity;

    NvFlowUint2 range_positions;
    NvFlowUint2 range_velocities;
    NvFlowUint2 range_divergences;
    NvFlowUint2 range_temperatures;

    NvFlowUint2 range_fuels;
    NvFlowUint2 range_burns;
    NvFlowUint2 range_smokes;
    NvFlowUint2 range_coupleRateVelocities;

    NvFlowUint2 range_coupleRateDivergences;
    NvFlowUint2 range_coupleRateTemperatures;
    NvFlowUint2 range_coupleRateFuels;
    NvFlowUint2 range_coupleRateBurns;

    NvFlowUint2 range_coupleRateSmokes;
    NvFlowUint2 range_colors;

    NvFlowFloat3 velocityToDensityBlockScale;
    NvFlowUint anyStreamingClearEnabled;

    NvFlowUint reduceCount1;
    NvFlowUint reduceCount2;
    NvFlowUint reduceCount3;
    NvFlowUint enableInterpolation;

    NvFlowUint voxelsPerPoint;
    NvFlowUint pad1;
    NvFlowUint pad2;
    NvFlowUint pad3;
};

struct EmitterPointMarkCS_Params
{
    NvFlowSparseLevelParams table;
    NvFlowSparseLevelParams tableCoarse;
    NvFlowFloat3 velocityToDensityBlockScale;
    NvFlowUint blockIdxOffset;
};

struct EmitterPointLayerClearCS_Params
{
    NvFlowUint blockIdxOffset;
    int layerAndLevel;
    NvFlowUint pad0;
    NvFlowUint pad1;

    NvFlowSparseLevelParams tableParams;
};

struct EmitterPointCS_SubStepParams
{
    NvFlowUint subStepIdx;
    NvFlowUint numSubSteps;
    float subStepDeltaTime;
    float subStepAccumDeltaTime;

    NvFlowFloat3 defaultVelocity;
    float totalDeltaTime;

    float velocityScale;
    float pad1;
    float pad2;
    float pad3;
};

struct EmitterTextureCS_Params
{
    NvFlowUint blockIdxOffset;
    NvFlowUint isVelocity;
    NvFlowUint textureFirstElement;
    NvFlowUint needsSrgbConversion;

    NvFlowUint textureWidth;
    NvFlowUint textureHeight;
    NvFlowUint textureDepth;
    float deltaTime;

    NvFlowSparseLevelParams table;

    NvFlowFloat4 targetValue;
    NvFlowFloat4 coupleRate;
    NvFlowFloat4 targetValueScale;

    NvFlowFloat3 vidxToWorld;
    NvFlowUint velocityIsWorldSpace;

    NvFlowFloat4x4 worldToLocal;
    NvFlowFloat4x4 localToWorldVelocity;

    NvFlowFloat3 halfSizeInv;
    float pad2;

    NvFlowFloat3 halfSize;
    float pad3;

    NvFlowFloat3 position;
    NvFlowUint textureIsColumnMajor;

    NvFlowInt4 locationOffset;
    NvFlowUint4 locationExtent;

    float minDistance;
    float maxDistance;
    NvFlowUint2 range_distances;

    NvFlowUint2 range_velocities;
    NvFlowUint2 range_divergences;
    NvFlowUint2 range_temperatures;
    NvFlowUint2 range_fuels;

    NvFlowUint2 range_burns;
    NvFlowUint2 range_smokes;
    NvFlowUint2 range_coupleRateVelocities;
    NvFlowUint2 range_coupleRateDivergences;

    NvFlowUint2 range_coupleRateTemperatures;
    NvFlowUint2 range_coupleRateFuels;
    NvFlowUint2 range_coupleRateBurns;
    NvFlowUint2 range_coupleRateSmokes;
};

struct EmitterNanoVdbCS_Params
{
    NvFlowUint blockIdxOffset;
    NvFlowUint needsSrgbConversion;
    NvFlowUint velocityIsWorldSpace;
    NvFlowUint sourceIsVelocity;

    NvFlowSparseLevelParams table;

    NvFlowFloat4 targetValue;
    NvFlowFloat4 coupleRate;
    NvFlowFloat4 targetValueScale;

    NvFlowFloat3 vidxToWorld;
    NvFlowUint isVelocity;

    NvFlowFloat3 position;
    float minDistance;

    NvFlowFloat3 halfSizeInv;
    float maxDistance;

    NvFlowFloat3 halfSize;
    float deltaTime;

    NvFlowUint anyStreamingClearEnabled;
    NvFlowUint absoluteValue;
    float minSmoke;
    float maxSmoke;

    NvFlowUint computeSpeed;
    NvFlowUint pad1;
    NvFlowUint pad2;
    NvFlowUint pad3;

    NvFlowFloat4x4 worldToLocal;
    NvFlowFloat4x4 localToWorldVelocity;

    NvFlowInt4 locationOffset;
    NvFlowUint4 locationExtent;

    NvFlowUint4 range_velocities;
    NvFlowUint4 range_divergences;
    NvFlowUint4 range_temperatures;
    NvFlowUint4 range_fuels;

    NvFlowUint4 range_burns;
    NvFlowUint4 range_smokes;
    NvFlowUint4 range_coupleRateVelocities;
    NvFlowUint4 range_coupleRateDivergences;

    NvFlowUint4 range_coupleRateTemperatures;
    NvFlowUint4 range_coupleRateFuels;
    NvFlowUint4 range_coupleRateBurns;
    NvFlowUint4 range_coupleRateSmokes;

    NvFlowUint4 range_distances;
    NvFlowUint4 range_rgba8s;
};

struct EmitterMeshCS_Params
{
    NvFlowUint blockIdxOffset;
    NvFlowUint faceBlockIdx1Offset;
    NvFlowUint faceBlockIdx2Offset;
    NvFlowUint faceBlockIdx3Offset;

    NvFlowUint face1Count;
    NvFlowUint face2Count;
    NvFlowUint face3Count;
    NvFlowUint velocityIsWorldSpace;

    float rasterThickness;
    float minDistance;
    float maxDistance;
    NvFlowBool32 orientationLeftHanded;

    NvFlowFloat4 emptyMin;
    NvFlowFloat4 emptyMax;

    NvFlowUint dispatchBoundsLocations;
    int layerAndLevel;
    NvFlowUint needsSrgbConversion;
    NvFlowUint pad1;

    NvFlowInt4 locationOffset;
    NvFlowUint4 locationExtent;

    NvFlowSparseLevelParams table;

    NvFlowFloat4x4 localToWorld;
    NvFlowFloat4x4 localToWorldVelocity;
    NvFlowFloat4x4 worldToLocal;
    NvFlowFloat4x4 localToWorldOld;

    NvFlowFloat4 defaultTargetValue;
    NvFlowFloat4 defaultCoupleRate;
    NvFlowFloat4 targetValueScale;

    NvFlowFloat3 vidxToWorld;
    float deltaTime;

    NvFlowFloat3 worldToVidx;
    NvFlowUint isVelocity;

    NvFlowUint2 range_positions;
    NvFlowUint2 range_faceVertexIndices;
    NvFlowUint2 range_faceVertexCounts;
    NvFlowUint2 range_faceVertexStarts;
    NvFlowUint2 range_velocities;
    float physicsVelocityScale;
    float physicsDeltaTimeInv;

    NvFlowUint2 range_divergences;
    NvFlowUint2 range_temperatures;

    NvFlowUint2 range_fuels;
    NvFlowUint2 range_burns;
    NvFlowUint2 range_smokes;
    NvFlowUint2 range_coupleRateVelocities;

    NvFlowUint2 range_coupleRateDivergences;
    NvFlowUint2 range_coupleRateTemperatures;
    NvFlowUint2 range_coupleRateFuels;
    NvFlowUint2 range_coupleRateBurns;

    NvFlowUint2 range_coupleRateSmokes;
    NvFlowUint2 range_colors;
};

struct EmitterMeshCS_SubStepParams
{
    NvFlowUint subStepIdx;
    NvFlowUint numSubSteps;
    float subStepDeltaTime;
    float subStepAccumDeltaTime;

    NvFlowFloat3 defaultVelocity;
    float totalDeltaTime;

    float velocityScale;
    float weight;
    float pad2;
    float pad3;
};
