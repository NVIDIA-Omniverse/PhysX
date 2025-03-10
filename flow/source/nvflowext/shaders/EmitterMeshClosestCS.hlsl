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

#include "NvFlowShader.hlsli"

#include "EmitterParams.h"

ConstantBuffer<EmitterMeshCS_Params> gParams;
ConstantBuffer<EmitterMeshCS_SubStepParams> gSubStepParams;

StructuredBuffer<uint> gTable;
StructuredBuffer<uint> arrayValuesIn;

StructuredBuffer<uint> sortValIn;
StructuredBuffer<uint4> bounds1In;
StructuredBuffer<uint4> bounds2In;
StructuredBuffer<uint4> bounds3In;
StructuredBuffer<uint4> bounds4In;

RWTexture3D<uint> closestOut;

bool rayTriangleNearest(float3 o, float3 d, float3 p0, float3 p1, float3 p2, out float3 uvt)
{
    // ray triangle intersection
    float3 e1 = p1 - p0;
    float3 e2 = p2 - p0;
    float3 q = cross(d, e2);
    float a = dot(e1, q);
    if (a == 0.f)
    {
        uvt = float3(0.f, 0.f, 0.f);
        return false;
    }
    float f = 1.f / a;
    float3 s = o - p0;
    float u = f * dot(s, q);
    float3 r = cross(s, e1);
    float v = f * dot(d, r);
    float t = f * dot(e2, r);

    // clamping to triangle
    float3 p = o + d * t;
    if (1.f - u - v < 0.f)
    {
        float edge = dot(p - p1, p2 - p1) / dot(p2 - p1, p2 - p1);
        edge = clamp(edge, 0.f, 1.f);
        u = 1.f - edge;
        v = edge;
    }
    else if (u < 0.f)
    {
        float edge = dot(p - p2, p0 - p2) / dot(p0 - p2, p0 - p2);
        edge = clamp(edge, 0.f, 1.f);
        u = 0.f;
        v = 1.f - edge;
    }
    else if (v < 0.f)
    {
        float edge = dot(p - p0, p1 - p0) / dot(p1 - p0, p1 - p0);
        edge = clamp(edge, 0.f, 1.f);
        u = edge;
        v = 0.f;
    }

    uvt = float3(u, v, t);
    return true;
}

float signedDistTriangle(float3 v1, float3 v2, float3 v3, float3 p)
{
    float3 n = cross(v2 - v1, v3 - v1);
    float3 uvt;
    bool valid = rayTriangleNearest(p, n, v1, v2, v3, uvt);
    if (!valid)
    {
        return gParams.emptyMin.x;
    }
    float3 nearest = (1.f - uvt.x - uvt.y) * v1 + uvt.x * v2 + uvt.y * v3;
    float3 diff = p - nearest;
    float dist = length(diff);
    float distSigned = sign(-uvt.z) * dist;
    if (gParams.orientationLeftHanded != 0u)
    {
        distSigned = -distSigned;
    }
    return distSigned;
}

float4 fetchWorldPos(uint faceVertexIdx)
{
    float4 posLocal = float4(
        asfloat(arrayValuesIn[gParams.range_positions.x + 3u * faceVertexIdx + 0u]),
        asfloat(arrayValuesIn[gParams.range_positions.x + 3u * faceVertexIdx + 1u]),
        asfloat(arrayValuesIn[gParams.range_positions.x + 3u * faceVertexIdx + 2u]),
        1.f);
    float4 posWorld = mul(posLocal, gParams.localToWorld);
    if (gSubStepParams.subStepIdx >= 1)
    {
        posWorld *= (1.f - gSubStepParams.weight);
        posWorld += gSubStepParams.weight * mul(posLocal, gParams.localToWorldOld);
    }
    if (gSubStepParams.subStepIdx >= 1)
    {
        float4 vel4 = float4(gSubStepParams.defaultVelocity, 0.f);
        if (faceVertexIdx < gParams.range_velocities.y)
        {
            vel4.x = asfloat(arrayValuesIn[gParams.range_velocities.x + 3 * faceVertexIdx + 0]);
            vel4.y = asfloat(arrayValuesIn[gParams.range_velocities.x + 3 * faceVertexIdx + 1]);
            vel4.z = asfloat(arrayValuesIn[gParams.range_velocities.x + 3 * faceVertexIdx + 2]);
        }
        if (gParams.velocityIsWorldSpace == 0u)
        {
            vel4 = mul(vel4, gParams.localToWorldVelocity);
        }
        vel4.xyz *= gSubStepParams.velocityScale;
        // displace back in time
        posWorld.xyz -= gSubStepParams.subStepAccumDeltaTime * vel4.xyz;
    }
    return posWorld;
}

bool overlapTest(float3 minA, float3 maxA, float3 minB, float3 maxB)
{
    return !(
        minA.x > maxB.x || minB.x > maxA.x ||
        minA.y > maxB.y || minB.y > maxA.y ||
        minA.z > maxB.z || minB.z > maxA.z
    );
}

groupshared uint sdata0[256u];
groupshared uint sdata1[256u];

groupshared uint slist3[256u];
groupshared uint slist2[256u];
groupshared uint slist1[256u];

uint blockScan(uint threadIdx, uint val, out uint totalCount)
{
    uint localVal = val;
    sdata0[threadIdx] = localVal;

    GroupMemoryBarrierWithGroupSync();

    if (threadIdx >= 1) localVal += sdata0[threadIdx - 1];
    if (threadIdx >= 2) localVal += sdata0[threadIdx - 2];
    if (threadIdx >= 3) localVal += sdata0[threadIdx - 3];
    sdata1[threadIdx] = localVal;

    GroupMemoryBarrierWithGroupSync();

    if (threadIdx >= 4) localVal += sdata1[threadIdx - 4];
    if (threadIdx >= 8) localVal += sdata1[threadIdx - 8];
    if (threadIdx >= 12) localVal += sdata1[threadIdx - 12];
    sdata0[threadIdx] = localVal;

    GroupMemoryBarrierWithGroupSync();

    if (threadIdx >= 16) localVal += sdata0[threadIdx - 16];
    if (threadIdx >= 32) localVal += sdata0[threadIdx - 32];
    if (threadIdx >= 48) localVal += sdata0[threadIdx - 48];
    sdata1[threadIdx] = localVal;

    GroupMemoryBarrierWithGroupSync();

    if (threadIdx >= 64) localVal += sdata1[threadIdx - 64];
    if (threadIdx >= 128) localVal += sdata1[threadIdx - 128];
    if (threadIdx >= 192) localVal += sdata1[threadIdx - 192];

    totalCount = sdata1[63] + sdata1[127] + sdata1[191] + sdata1[255];

    return localVal;
}

int3 computeThreadIdx(NvFlowSparseLevelParams tableParams, uint threadIdx1D)
{
    // do 8x8x4 workgroup
    int3 minorIdx = int3(
        (threadIdx1D) & 7,
        (threadIdx1D >> 3) & 7,
        (threadIdx1D >> 6) & 3
    );
    int3 majorIdx = int3(
        int(int(threadIdx1D >> 8u) & int(tableParams.blockDimLessOne.x >> 3)),
        int((int(threadIdx1D >> 8u) >> int(tableParams.blockDimBits.x - 3)) & int(tableParams.blockDimLessOne.y >> 3)),
        int((int(threadIdx1D >> 8u) >> int(tableParams.blockDimBits.x + tableParams.blockDimBits.y - 6)) & int(tableParams.blockDimLessOne.z >> 2))
    );
    return (majorIdx << int3(3, 3, 2)) | minorIdx;
}

[numthreads(256, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID, uint3 groupID : SV_GroupID, uint3 groupThreadID : SV_GroupThreadID)
{
    uint sthreadIdx = groupThreadID.x;

    int3 threadIdx = computeThreadIdx(gParams.table, dispatchThreadID.x);
    uint blockIdx = groupID.y + gParams.blockIdxOffset;
    if (gParams.dispatchBoundsLocations != 0u)
    {
        // intepret blockIdx as locationIdx in bounds instead
        uint locationIdx = blockIdx;
        int4 location = int4(
            int(locationIdx % gParams.locationExtent.x) + gParams.locationOffset.x,
            int((locationIdx / gParams.locationExtent.x) % gParams.locationExtent.y) + gParams.locationOffset.y,
            int(locationIdx / (gParams.locationExtent.x * gParams.locationExtent.y)) + gParams.locationOffset.z,
            gParams.locationOffset.w
            );
        blockIdx = NvFlowLocationToBlockIdx(gTable, gParams.table, location);
    }

    int4 location = int4(0, 0, 0, 0);
    if (!NvFlowBlockIdxToLocation(gTable, gParams.table, blockIdx, location))
    {
        return;
    }

    float closestDistance = gParams.emptyMin.x;
    uint closestFaceIdx = ~0u;
    uint testCount = 0u;

    int3 vidx = (location.xyz << int3(gParams.table.blockDimBits.xyz)) + threadIdx;
    float3 vidxf = float3(vidx)+float3(0.5f, 0.5f, 0.5f);
    float3 cellPosWorld = vidxf * gParams.vidxToWorld;

    // compute world min/max for this thread group
    int3 threadIdxMin = computeThreadIdx(gParams.table, (groupID.x << 8u) + 0u);
    int3 threadIdxMax = computeThreadIdx(gParams.table, (groupID.x << 8u) + 255u);
    int3 vidxMin = (location.xyz << int3(gParams.table.blockDimBits.xyz)) + threadIdxMin;
    int3 vidxMax = (location.xyz << int3(gParams.table.blockDimBits.xyz)) + threadIdxMax;
    float3 groupWorldMin = gParams.vidxToWorld * float3(vidxMin);
    float3 groupWorldMax = gParams.vidxToWorld * float3(vidxMax + int3(1, 1, 1));

    // inflate group bounds by thickness
    groupWorldMin -= gParams.rasterThickness;
    groupWorldMax += gParams.rasterThickness;

    // check overlap against level3, build list of overlaps
    uint bounds3Idx = sthreadIdx;
    float4 bounds3Min = bounds3Idx < gParams.face3Count ? asfloat(bounds3In[2u * bounds3Idx + 0u]) : gParams.emptyMin;
    float4 bounds3Max = bounds3Idx < gParams.face3Count ? asfloat(bounds3In[2u * bounds3Idx + 1u]) : gParams.emptyMax;
    uint overlap3 = overlapTest(groupWorldMin, groupWorldMax, bounds3Min.xyz, bounds3Max.xyz) ? 1u : 0u;
    uint total3 = 0u;
    uint scan3 = blockScan(sthreadIdx, overlap3, total3) - overlap3;
    if (overlap3 != 0u)
    {
        slist3[scan3] = bounds3Idx;
    }
    GroupMemoryBarrierWithGroupSync();

    // interate list of level3 overlaps
    for (uint slistIdx3 = 0u; slistIdx3 < total3; slistIdx3++)
    {
        uint bounds2Idx = (slist3[slistIdx3] << 8u) + sthreadIdx;
        float4 bounds2Min = bounds2Idx < gParams.face2Count ? asfloat(bounds2In[2u * bounds2Idx + 0u]) : gParams.emptyMin;
        float4 bounds2Max = bounds2Idx < gParams.face2Count ? asfloat(bounds2In[2u * bounds2Idx + 1u]) : gParams.emptyMax;
        uint overlap2 = overlapTest(groupWorldMin, groupWorldMax, bounds2Min.xyz, bounds2Max.xyz) ? 1u : 0u;
        uint total2 = 0u;
        uint scan2 = blockScan(sthreadIdx, overlap2, total2) - overlap2;
        if (overlap2 != 0u)
        {
            slist2[scan2] = bounds2Idx;
        }
        GroupMemoryBarrierWithGroupSync();

        // interate list of level2 overlaps
        for (uint slistIdx2 = 0u; slistIdx2 < total2; slistIdx2++)
        {
            uint bounds1Idx = (slist2[slistIdx2] << 8u) + sthreadIdx;
            float4 bounds1Min = bounds1Idx < gParams.face1Count ? asfloat(bounds1In[2u * bounds1Idx + 0u]) : gParams.emptyMin;
            float4 bounds1Max = bounds1Idx < gParams.face1Count ? asfloat(bounds1In[2u * bounds1Idx + 1u]) : gParams.emptyMax;
            uint overlap1 = overlapTest(groupWorldMin, groupWorldMax, bounds1Min.xyz, bounds1Max.xyz) ? 1u : 0u;
            uint total1 = 0u;
            uint scan1 = blockScan(sthreadIdx, overlap1, total1) - overlap1;
            if (overlap1 != 0u)
            {
                slist1[scan1] = bounds1Idx;
            }
            GroupMemoryBarrierWithGroupSync();

            // iterate faces
            for (uint slistIdx1 = 0u; slistIdx1 < total1; slistIdx1++)
            {
                uint sortedFaceIdx = slist1[slistIdx1];
                if (sortedFaceIdx < gParams.face1Count)
                {
                    uint faceIdx = sortValIn[sortedFaceIdx];

                    testCount++;

                    uint faceVertexCount = arrayValuesIn[gParams.range_faceVertexCounts.x + faceIdx];
                    uint faceVertexStart = arrayValuesIn[gParams.range_faceVertexStarts.x + faceIdx];
                    if (faceVertexCount >= 3u)
                    {
                        uint faceVertexIdx0 = arrayValuesIn[gParams.range_faceVertexIndices.x + faceVertexStart];
                        float4 posWorld0 = fetchWorldPos(faceVertexIdx0);

                        for (uint triangleIdx = 0u; triangleIdx < (faceVertexCount - 2u); triangleIdx++)
                        {
                            uint faceVertexIdx1 = arrayValuesIn[gParams.range_faceVertexIndices.x + faceVertexStart + triangleIdx + 1u];
                            uint faceVertexIdx2 = arrayValuesIn[gParams.range_faceVertexIndices.x + faceVertexStart + triangleIdx + 2u];
                            float4 posWorld1 = fetchWorldPos(faceVertexIdx1);
                            float4 posWorld2 = fetchWorldPos(faceVertexIdx2);

                            float dist = signedDistTriangle(posWorld0.xyz, posWorld1.xyz, posWorld2.xyz, cellPosWorld);
                            if (abs(dist) < abs(closestDistance))
                            {
                                closestDistance = dist;
                                closestFaceIdx = faceIdx;
                            }
                        }
                    }
                }
            }
        }
    }

    uint closestVal = ~0u;
    if (closestFaceIdx != ~0u && closestDistance >= gParams.minDistance && closestDistance <= gParams.maxDistance)
    {
        closestVal = closestFaceIdx;
    }
    NvFlowLocalWrite1ui(closestOut, gTable, gParams.table, blockIdx, threadIdx, closestVal);
}
