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

#include "shaders/EllipsoidRasterParams.h"

#include "NvFlowExt.h"

#include "NvFlowArray.h"
#include "NvFlowMath.h"
#include "NvFlowUploadBuffer.h"
#include "NvFlowArrayBuffer.h"
#include "NvFlowReadbackBuffer.h"
#include "NvFlowTimer.h"

#include "shaders/EllipsoidRasterSmoothCS.hlsl.h"
#include "shaders/EllipsoidRasterCS.hlsl.h"

#include "NvFlowLocationHashTable.h"
#include <atomic>

namespace
{
    enum Array1
    {
        eArray1_positionFloat3s = 0,

        eArray1_count = 1
    };
    enum Array4
    {
        eArray4_positions = 0,
        eArray4_anisotropyE1s = 1,
        eArray4_anisotropyE2s = 2,
        eArray4_anisotropyE3s = 3,

        eArray4_count = 4
    };
}

namespace
{
    struct EllipsoidRasterAllocateTaskParams
    {
        NvFlowLocationHashTable locationHash;
        const NvFlowEllipsoidRasterParams* layerParamsIn;
        NvFlowFloat3 blockSizeWorld;
        NvFlowFloat3 blockSizeWorldInv;
        NvFlowUint64 positionCount;
        int layerAndLevel;
        float allocationScale;
        float allocationOffset;
    };

    struct EllipsoidRasterAllocate
    {
        NvFlowContextInterface contextInterface = {};

        NvFlowArray<NvFlowInt4> locations;

        NvFlowArray<EllipsoidRasterAllocateTaskParams> taskParams;
    };

    EllipsoidRasterAllocate* EllipsoidRasterAllocate_create(const NvFlowOpInterface* opInterface, const NvFlowEllipsoidRasterAllocatePinsIn* in, NvFlowEllipsoidRasterAllocatePinsOut* out)
    {
        auto ptr = new EllipsoidRasterAllocate();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        return ptr;
    }

    void EllipsoidRasterAllocate_destroy(EllipsoidRasterAllocate* ptr, const NvFlowEllipsoidRasterAllocatePinsIn* in, NvFlowEllipsoidRasterAllocatePinsOut* out)
    {
        delete ptr;
    }

    void EllipsoidRasterAllocate_execute(EllipsoidRasterAllocate* ptr, const NvFlowEllipsoidRasterAllocatePinsIn* in, NvFlowEllipsoidRasterAllocatePinsOut* out)
    {
        using namespace NvFlowMath;

        out->feedback.data = ptr;

        ptr->locations.size = 0u;
        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < in->paramCount; layerParamIdx++)
        {
            auto layerParamsIn = in->params[layerParamIdx];

            int layerAndLevel = in->sparseParams.layers[layerParamIdx].layerAndLevel;

            NvFlowFloat3 blockSizeWorld = in->sparseParams.layers[layerParamIdx].blockSizeWorld;

            NvFlowFloat3 blockSizeWorldInv = {
                1.f / blockSizeWorld.x,
                1.f / blockSizeWorld.y,
                1.f / blockSizeWorld.z
            };

            float allocationScale = layerParamsIn->allocationScale * layerParamsIn->scale;
            float allocationOffset = layerParamsIn->allocationOffset;

            NvFlowUint64 positionCount = layerParamsIn->positionCount + layerParamsIn->positionFloat3Count;

            static const NvFlowUint64 pointsPerTask = 256u;
            NvFlowUint64 taskCount = ((positionCount + pointsPerTask - 1u) / pointsPerTask);

            ptr->taskParams.reserve(taskCount);
            ptr->taskParams.size = taskCount;

            for (NvFlowUint taskIdx = 0u; taskIdx < taskCount; taskIdx++)
            {
                ptr->taskParams[taskIdx].locationHash.reset();
                ptr->taskParams[taskIdx].layerParamsIn = layerParamsIn;
                ptr->taskParams[taskIdx].blockSizeWorld = blockSizeWorld;
                ptr->taskParams[taskIdx].blockSizeWorldInv = blockSizeWorldInv;
                ptr->taskParams[taskIdx].positionCount = positionCount;
                ptr->taskParams[taskIdx].layerAndLevel = layerAndLevel;
                ptr->taskParams[taskIdx].allocationScale = allocationScale;
                ptr->taskParams[taskIdx].allocationOffset = allocationOffset;
            }

            auto task = [](NvFlowUint taskIdx, NvFlowUint threadIdx, void* sharedMem, void* userdata)
            {
                auto ptr = (EllipsoidRasterAllocate*)userdata;

                auto& taskParams = ptr->taskParams[taskIdx];
                const auto& layerParamsIn = taskParams.layerParamsIn;

                NvFlowUint64 particleBeginIdx = taskIdx * pointsPerTask;
                NvFlowUint64 particleEndIdx = particleBeginIdx + pointsPerTask;
                if (particleEndIdx > taskParams.positionCount)
                {
                    particleEndIdx = taskParams.positionCount;
                }

                for (NvFlowUint64 particleIdx = particleBeginIdx; particleIdx < particleEndIdx; particleIdx++)
                {
                    NvFlowFloat4 position = { 0.f, 0.f, 0.f, 1.f };
                    if (particleIdx < layerParamsIn->positionCount)
                    {
                        position = layerParamsIn->positions[particleIdx];
                    }
                    else
                    {
                        NvFlowFloat3 pos3 = layerParamsIn->positionFloat3s[particleIdx - layerParamsIn->positionCount];
                        position.x = pos3.x;
                        position.y = pos3.y;
                        position.z = pos3.z;
                    }
                    NvFlowFloat4 e1 = particleIdx < layerParamsIn->anisotropyE1Count ? layerParamsIn->anisotropyE1s[particleIdx] : NvFlowFloat4{ 1.f, 0.f, 0.f, 1.f };
                    NvFlowFloat4 e2 = particleIdx < layerParamsIn->anisotropyE2Count ? layerParamsIn->anisotropyE2s[particleIdx] : NvFlowFloat4{ 0.f, 1.f, 0.f, 1.f };
                    NvFlowFloat4 e3 = particleIdx < layerParamsIn->anisotropyE3Count ? layerParamsIn->anisotropyE3s[particleIdx] : NvFlowFloat4{ 0.f, 0.f, 1.f, 1.f };
                    NvFlowFloat4 xTransform = { e1.x * e1.w, e2.x * e2.w, e3.x * e3.w, position.x };
                    NvFlowFloat4 yTransform = { e1.y * e1.w, e2.y * e2.w, e3.y * e3.w, position.y };
                    NvFlowFloat4 zTransform = { e1.z * e1.w, e2.z * e2.w, e3.z * e3.w, position.z };

                    NvFlowFloat4 minWorld = {};
                    NvFlowFloat4 maxWorld = {};
                    for (int k = -1; k <= +1; k += 2)
                    {
                        for (int j = -1; j <= +1; j += 2)
                        {
                            for (int i = -1; i <= +1; i += 2)
                            {
                                NvFlowFloat4 boxCoord = {
                                    taskParams.allocationScale * float(i),
                                    taskParams.allocationScale * float(j),
                                    taskParams.allocationScale * float(k),
                                    1.f
                                };
                                NvFlowFloat4 worldCoord = {
                                    vector4Dot(xTransform, boxCoord).x,
                                    vector4Dot(yTransform, boxCoord).x,
                                    vector4Dot(zTransform, boxCoord).x,
                                    1.f
                                };
                                if (minWorld.w == 0.f)
                                {
                                    minWorld = worldCoord;
                                    maxWorld = worldCoord;
                                }
                                else
                                {
                                    minWorld = vectorMin(minWorld, worldCoord);
                                    maxWorld = vectorMax(maxWorld, worldCoord);
                                }
                            }
                        }
                    }
                    minWorld.x -= taskParams.allocationOffset;
                    minWorld.y -= taskParams.allocationOffset;
                    minWorld.z -= taskParams.allocationOffset;
                    maxWorld.x += taskParams.allocationOffset;
                    maxWorld.y += taskParams.allocationOffset;
                    maxWorld.z += taskParams.allocationOffset;

                    NvFlowInt4 locationMin = {
                        int(floorf(minWorld.x * taskParams.blockSizeWorldInv.x)),
                        int(floorf(minWorld.y * taskParams.blockSizeWorldInv.y)),
                        int(floorf(minWorld.z * taskParams.blockSizeWorldInv.z)),
                        taskParams.layerAndLevel
                    };
                    NvFlowInt4 locationMax = {
                        int(floorf(maxWorld.x * taskParams.blockSizeWorldInv.x)),
                        int(floorf(maxWorld.y * taskParams.blockSizeWorldInv.y)),
                        int(floorf(maxWorld.z * taskParams.blockSizeWorldInv.z)),
                        taskParams.layerAndLevel
                    };
                    for (int k = locationMin.z; k <= locationMax.z; k++)
                    {
                        for (int j = locationMin.y; j <= locationMax.y; j++)
                        {
                            for (int i = locationMin.x; i <= locationMax.x; i++)
                            {
                                taskParams.locationHash.push(NvFlowInt4{ i, j, k, locationMin.w }, 0u);
                            }
                        }
                    }
                }
            };

            ptr->contextInterface.executeTasks(in->context, (NvFlowUint)taskCount, taskCount < 8u ? 8u : 1u, task, ptr);

            for (NvFlowUint taskIdx = 0u; taskIdx < taskCount; taskIdx++)
            {
                auto& taskParams = ptr->taskParams[taskIdx];
                for (NvFlowUint locationIdx = 0u; locationIdx < taskParams.locationHash.locations.size; locationIdx++)
                {
                    NvFlowInt4 entry_location = taskParams.locationHash.locations[locationIdx];
                    //NvFlowUint entry_mask = taskParams.locationHash.masks[locationIdx];

                    ptr->locations.pushBack(entry_location);
                }
            }
        }
        out->locations = ptr->locations.data;
        out->locationCount = ptr->locations.size;
    }
} // end namespace

NV_FLOW_OP_IMPL(NvFlowEllipsoidRasterAllocate, EllipsoidRasterAllocate)

namespace
{
    struct EllipsoidRasterTaskParams
    {
        NvFlowLocationHashTable locationHash;
        const NvFlowSparseParams* sparseParams;
        const NvFlowEllipsoidRasterParams* layerParamsIn;
        NvFlowFloat3 blockSizeWorld;
        NvFlowFloat3 blockSizeWorldInv;
        NvFlowUint64 positionCount;
        int layerAndLevel;
        float allocationScale;
        float allocationOffset;

        NvFlowArray<NvFlowUint2> perBlockInstanceBlockIdx;
    };

    struct EllipsoidAtomic
    {
        std::atomic_uint32_t v;
        EllipsoidAtomic() {}
        EllipsoidAtomic(const EllipsoidAtomic& rhs) { v.store(rhs.v.load()); }
    };

    struct EllipsoidRaster
    {
        NvFlowContextInterface contextInterface = {};

        EllipsoidRasterCS_Pipeline ellipsoidRasterCS = {};
        EllipsoidRasterSmoothCS_Pipeline ellipsoidRasterSmoothCS = {};

        NvFlowUploadBuffer globalBuffer = {};
        NvFlowUploadBuffer layerBuffer = {};
        NvFlowArrayBuffer arrayBuffer1 = {};
        NvFlowArrayBuffer arrayBuffer4 = {};
        NvFlowUploadBuffer blockIdxBuffer = {};
        NvFlowUploadBuffer rangeBuffer = {};
        NvFlowUploadBuffer instanceIdBuffer = {};

        NvFlowArray<NvFlowArrayBufferData> arrayBufferDatas1;
        NvFlowArray<NvFlowUint64> arrayFirstElements1;
        NvFlowArray<NvFlowArrayBufferData> arrayBufferDatas4;
        NvFlowArray<NvFlowUint64> arrayFirstElements4;

        NvFlowArray<EllipsoidAtomic> perBlockCount;
        NvFlowArray<NvFlowUint> perBlockIndex;
        NvFlowArray<NvFlowUint2> perBlockInstanceBlockIdx;

        NvFlowArray<NvFlowUint> blockListBlockIdx;
        NvFlowArray<NvFlowUint2> blockListRange;
        NvFlowArray<NvFlowUint> blockListInstanceId;

        NvFlowArray<NvFlowInt4> locations;

        NvFlowArray<EllipsoidRasterTaskParams> taskParams;
    };

    EllipsoidRaster* EllipsoidRaster_create(const NvFlowOpInterface* opInterface, const NvFlowEllipsoidRasterPinsIn* in, NvFlowEllipsoidRasterPinsOut* out)
    {
        auto ptr = new EllipsoidRaster();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        EllipsoidRasterCS_init(&ptr->contextInterface, in->context, &ptr->ellipsoidRasterCS);
        EllipsoidRasterSmoothCS_init(&ptr->contextInterface, in->context, &ptr->ellipsoidRasterSmoothCS);

        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->globalBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->layerBuffer, eNvFlowBufferUsage_constantBuffer | eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(EllipsoidRasterCS_LayerParams));
        NvFlowArrayBuffer_init(&ptr->contextInterface, in->context, &ptr->arrayBuffer1, eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_bufferCopySrc, eNvFlowFormat_unknown, sizeof(float));
        NvFlowArrayBuffer_init(&ptr->contextInterface, in->context, &ptr->arrayBuffer4, eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_bufferCopySrc, eNvFlowFormat_unknown, sizeof(NvFlowFloat4));
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->blockIdxBuffer, eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->rangeBuffer, eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint2));
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->instanceIdBuffer, eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(NvFlowUint));

        return ptr;
    }

    void EllipsoidRaster_destroy(EllipsoidRaster* ptr, const NvFlowEllipsoidRasterPinsIn* in, NvFlowEllipsoidRasterPinsOut* out)
    {
        NvFlowUploadBuffer_destroy(in->context, &ptr->globalBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->layerBuffer);
        NvFlowArrayBuffer_destroy(in->context, &ptr->arrayBuffer1);
        NvFlowArrayBuffer_destroy(in->context, &ptr->arrayBuffer4);
        NvFlowUploadBuffer_destroy(in->context, &ptr->blockIdxBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->rangeBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->instanceIdBuffer);

        EllipsoidRasterCS_destroy(in->context, &ptr->ellipsoidRasterCS);
        EllipsoidRasterSmoothCS_destroy(in->context, &ptr->ellipsoidRasterSmoothCS);

        delete ptr;
    }

    void EllipsoidRaster_execute(EllipsoidRaster* ptr, const NvFlowEllipsoidRasterPinsIn* in, NvFlowEllipsoidRasterPinsOut* out)
    {
        using namespace NvFlowMath;

        NV_FLOW_PROFILE_BEGIN(60, 15)
        NV_FLOW_PROFILE_TIMESTAMP("EllipsoidRasterBegin")

        NvFlowUint numLayers = in->layout.sparseParams.layerCount;

        const NvFlowSparseParams* sparseParams = &in->layout.sparseParams;
        const NvFlowSparseLevelParams* levelParams = &sparseParams->levels[in->layout.levelIdx];

        // count total particles
        NvFlowUint64 totalPositionCount = 0u;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < numLayers; layerParamIdx++)
        {
            auto layerParamsIn = in->params[layerParamIdx];
            totalPositionCount += layerParamsIn->positionCount + layerParamsIn->positionFloat3Count;
        }

        if (totalPositionCount == 0u)
        {
            NvFlowSparseTexture_passThrough(&out->value, &in->layout);
            return;
        }

        ptr->arrayBufferDatas1.reserve(numLayers * eArray1_count);
        ptr->arrayBufferDatas1.size = numLayers * eArray1_count;
        ptr->arrayBufferDatas4.reserve(numLayers * eArray4_count);
        ptr->arrayBufferDatas4.size = numLayers * eArray4_count;

        ptr->arrayFirstElements1.reserve(numLayers * eArray1_count);
        ptr->arrayFirstElements1.size = numLayers * eArray1_count;
        ptr->arrayFirstElements4.reserve(numLayers * eArray4_count);
        ptr->arrayFirstElements4.size = numLayers * eArray4_count;

        ptr->perBlockCount.size = 0u;
        ptr->perBlockIndex.size = 0u;
        ptr->perBlockInstanceBlockIdx.size = 0u;

        ptr->perBlockCount.reserve(sparseParams->locationCount);
        ptr->perBlockCount.size = sparseParams->locationCount;
        ptr->perBlockIndex.reserve(sparseParams->locationCount);
        ptr->perBlockIndex.size = sparseParams->locationCount;
        for (NvFlowUint64 idx = 0u; idx < ptr->perBlockCount.size; idx++)
        {
            ptr->perBlockCount[idx].v = 0u;
            ptr->perBlockIndex[idx] = 0u;
        }

        NvFlowUint64 positionsTmpSize = 0u;
        NvFlowUint64 positionsTmpWriteIdx = 0u;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < numLayers; layerParamIdx++)
        {
            auto layerParamsIn = in->params[layerParamIdx];
            if (layerParamsIn->positionFloat3Count > 0u)
            {
                positionsTmpSize += layerParamsIn->positionCount + layerParamsIn->positionFloat3Count;
            }
        }

        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < numLayers; layerParamIdx++)
        {
            auto layerParamsIn = in->params[layerParamIdx];

            ptr->arrayBufferDatas1[eArray1_count * layerParamIdx + eArray1_positionFloat3s] = { layerParamsIn->positionFloat3s, 3u * layerParamsIn->positionFloat3Count, 0llu };

            ptr->arrayBufferDatas4[eArray4_count * layerParamIdx + eArray4_positions] = { layerParamsIn->positions, layerParamsIn->positionCount, 0llu    };
            ptr->arrayBufferDatas4[eArray4_count * layerParamIdx + eArray4_anisotropyE1s] = { layerParamsIn->anisotropyE1s, layerParamsIn->anisotropyE1Count, 0llu    };
            ptr->arrayBufferDatas4[eArray4_count * layerParamIdx + eArray4_anisotropyE2s] = { layerParamsIn->anisotropyE2s, layerParamsIn->anisotropyE2Count, 0llu };
            ptr->arrayBufferDatas4[eArray4_count * layerParamIdx + eArray4_anisotropyE3s] = { layerParamsIn->anisotropyE3s, layerParamsIn->anisotropyE3Count, 0llu    };

            int layerAndLevel = sparseParams->layers[layerParamIdx].layerAndLevel;
            NvFlowFloat3 blockSizeWorld = sparseParams->layers[layerParamIdx].blockSizeWorld;
            NvFlowFloat3 blockSizeWorldInv = {
                1.f / blockSizeWorld.x,
                1.f / blockSizeWorld.y,
                1.f / blockSizeWorld.z
            };

            float allocationScale = layerParamsIn->allocationScale * layerParamsIn->scale;
            float allocationOffset = layerParamsIn->allocationOffset;

            NvFlowUint64 positionCount = layerParamsIn->positionCount + layerParamsIn->positionFloat3Count;

            static const NvFlowUint64 pointsPerTask = 256u;
            NvFlowUint64 taskCount = ((positionCount + pointsPerTask - 1u) / pointsPerTask);

            ptr->taskParams.reserve(taskCount);
            ptr->taskParams.size = taskCount;

            for (NvFlowUint taskIdx = 0u; taskIdx < taskCount; taskIdx++)
            {
                ptr->taskParams[taskIdx].locationHash.reset();
                ptr->taskParams[taskIdx].sparseParams = sparseParams;
                ptr->taskParams[taskIdx].layerParamsIn = layerParamsIn;
                ptr->taskParams[taskIdx].blockSizeWorld = blockSizeWorld;
                ptr->taskParams[taskIdx].blockSizeWorldInv = blockSizeWorldInv;
                ptr->taskParams[taskIdx].positionCount = positionCount;
                ptr->taskParams[taskIdx].layerAndLevel = layerAndLevel;
                ptr->taskParams[taskIdx].allocationScale = allocationScale;
                ptr->taskParams[taskIdx].allocationOffset = allocationOffset;

                ptr->taskParams[taskIdx].perBlockInstanceBlockIdx.size = 0u;
            }

            auto task = [](NvFlowUint taskIdx, NvFlowUint threadIdx, void* sharedMem, void* userdata)
            {
                auto ptr = (EllipsoidRaster*)userdata;

                auto& taskParams = ptr->taskParams[taskIdx];
                const auto& layerParamsIn = taskParams.layerParamsIn;

                NvFlowUint64 particleBeginIdx = taskIdx * pointsPerTask;
                NvFlowUint64 particleEndIdx = particleBeginIdx + pointsPerTask;
                if (particleEndIdx > taskParams.positionCount)
                {
                    particleEndIdx = taskParams.positionCount;
                }

                for (NvFlowUint64 particleIdx = particleBeginIdx; particleIdx < particleEndIdx; particleIdx++)
                {
                    NvFlowFloat4 position = { 0.f, 0.f, 0.f, 1.f };
                    if (particleIdx < layerParamsIn->positionCount)
                    {
                        position = layerParamsIn->positions[particleIdx];
                    }
                    else
                    {
                        NvFlowFloat3 pos3 = layerParamsIn->positionFloat3s[particleIdx - layerParamsIn->positionCount];
                        position.x = pos3.x;
                        position.y = pos3.y;
                        position.z = pos3.z;
                    }
                    NvFlowFloat4 e1 = particleIdx < layerParamsIn->anisotropyE1Count ? layerParamsIn->anisotropyE1s[particleIdx] : NvFlowFloat4{ 1.f, 0.f, 0.f, 1.f };
                    NvFlowFloat4 e2 = particleIdx < layerParamsIn->anisotropyE2Count ? layerParamsIn->anisotropyE2s[particleIdx] : NvFlowFloat4{ 0.f, 1.f, 0.f, 1.f };
                    NvFlowFloat4 e3 = particleIdx < layerParamsIn->anisotropyE3Count ? layerParamsIn->anisotropyE3s[particleIdx] : NvFlowFloat4{ 0.f, 0.f, 1.f, 1.f };
                    NvFlowFloat4 xTransform = { e1.x * e1.w, e2.x * e2.w, e3.x * e3.w, position.x };
                    NvFlowFloat4 yTransform = { e1.y * e1.w, e2.y * e2.w, e3.y * e3.w, position.y };
                    NvFlowFloat4 zTransform = { e1.z * e1.w, e2.z * e2.w, e3.z * e3.w, position.z };

                    NvFlowFloat4 minWorld = {};
                    NvFlowFloat4 maxWorld = {};
                    for (int k = -1; k <= +1; k += 2)
                    {
                        for (int j = -1; j <= +1; j += 2)
                        {
                            for (int i = -1; i <= +1; i += 2)
                            {
                                NvFlowFloat4 boxCoord = {
                                    taskParams.allocationScale * float(i),
                                    taskParams.allocationScale * float(j),
                                    taskParams.allocationScale * float(k),
                                    1.f
                                };
                                NvFlowFloat4 worldCoord = {
                                    vector4Dot(xTransform, boxCoord).x,
                                    vector4Dot(yTransform, boxCoord).x,
                                    vector4Dot(zTransform, boxCoord).x,
                                    1.f
                                };
                                if (minWorld.w == 0.f)
                                {
                                    minWorld = worldCoord;
                                    maxWorld = worldCoord;
                                }
                                else
                                {
                                    minWorld = vectorMin(minWorld, worldCoord);
                                    maxWorld = vectorMax(maxWorld, worldCoord);
                                }
                            }
                        }
                    }
                    minWorld.x -= taskParams.allocationOffset;
                    minWorld.y -= taskParams.allocationOffset;
                    minWorld.z -= taskParams.allocationOffset;
                    maxWorld.x += taskParams.allocationOffset;
                    maxWorld.y += taskParams.allocationOffset;
                    maxWorld.z += taskParams.allocationOffset;

                    NvFlowInt4 locationMin = {
                        int(floorf(minWorld.x * taskParams.blockSizeWorldInv.x)),
                        int(floorf(minWorld.y * taskParams.blockSizeWorldInv.y)),
                        int(floorf(minWorld.z * taskParams.blockSizeWorldInv.z)),
                        taskParams.layerAndLevel
                    };
                    NvFlowInt4 locationMax = {
                        int(floorf(maxWorld.x * taskParams.blockSizeWorldInv.x)),
                        int(floorf(maxWorld.y * taskParams.blockSizeWorldInv.y)),
                        int(floorf(maxWorld.z * taskParams.blockSizeWorldInv.z)),
                        taskParams.layerAndLevel
                    };
                    for (int k = locationMin.z; k <= locationMax.z; k++)
                    {
                        for (int j = locationMin.y; j <= locationMax.y; j++)
                        {
                            for (int i = locationMin.x; i <= locationMax.x; i++)
                            {
                                NvFlowInt4 location = { i, j, k, locationMin.w };
                                NvFlowUint blockIdx = NvFlowSparseParams_locationToBlockIdx(taskParams.sparseParams, location);
                                if (blockIdx != ~0u)
                                {
                                    ptr->perBlockCount[blockIdx].v.fetch_add(1u, std::memory_order_relaxed);
                                    taskParams.perBlockInstanceBlockIdx.pushBack(NvFlowUint2{ (NvFlowUint)particleIdx, blockIdx });
                                }
                            }
                        }
                    }
                }
            };

            ptr->contextInterface.executeTasks(in->context, (NvFlowUint)taskCount, taskCount < 8u ? 8u : 1u, task, ptr);

            NV_FLOW_PROFILE_TIMESTAMP("EllipsoidBoundsTasks")

            for (NvFlowUint taskIdx = 0u; taskIdx < taskCount; taskIdx++)
            {
                auto& taskParams = ptr->taskParams[taskIdx];
                for (NvFlowUint64 idx = 0u; idx < taskParams.perBlockInstanceBlockIdx.size; idx++)
                {
                    ptr->perBlockInstanceBlockIdx.pushBack(taskParams.perBlockInstanceBlockIdx[idx]);
                }
            }

            NV_FLOW_PROFILE_TIMESTAMP("EllipsoidBoundsMerge")
        }

        NvFlowUint64 arrayBufferSizeInBytes1 = 0llu;
        NvFlowUint64 arrayBufferSizeInBytes4 = 0llu;
        NvFlowBufferTransient* arraysTransient1 = NvFlowArrayBuffer_update(in->context, &ptr->arrayBuffer1, 0llu, ptr->arrayBufferDatas1.data, ptr->arrayFirstElements1.data, ptr->arrayBufferDatas1.size, &arrayBufferSizeInBytes1, "EllipsoidRasterUpload1");
        NvFlowBufferTransient* arraysTransient4 = NvFlowArrayBuffer_update(in->context, &ptr->arrayBuffer4, 0llu, ptr->arrayBufferDatas4.data, ptr->arrayFirstElements4.data, ptr->arrayBufferDatas4.size, &arrayBufferSizeInBytes4, "EllipsoidRasterUpload4");

        ptr->blockListBlockIdx.size = 0u;
        ptr->blockListRange.size = 0u;
        NvFlowUint perBlockScan = 0u;
        for (NvFlowUint64 blockIdx = 0u; blockIdx < ptr->perBlockCount.size; blockIdx++)
        {
            NvFlowUint perBlockCount = ptr->perBlockCount[blockIdx].v;
            ptr->perBlockIndex[blockIdx] = perBlockScan;
            if (perBlockCount > 0u)
            {
                ptr->blockListBlockIdx.pushBack((NvFlowUint)blockIdx);
                ptr->blockListRange.pushBack(NvFlowUint2{perBlockScan, perBlockScan + perBlockCount});
            }
            perBlockScan += perBlockCount;
        }

        NV_FLOW_PROFILE_TIMESTAMP("EllipsoidBoundsScan")

        // scatter
        ptr->blockListInstanceId.size = 0u;
        ptr->blockListInstanceId.reserve(ptr->perBlockInstanceBlockIdx.size);
        ptr->blockListInstanceId.size = ptr->perBlockInstanceBlockIdx.size;
        for (NvFlowUint64 idx = 0u; idx < ptr->perBlockInstanceBlockIdx.size; idx++)
        {
            NvFlowUint2 instanceBlockIdx = ptr->perBlockInstanceBlockIdx[idx];
            NvFlowUint writeIdx = ptr->perBlockIndex[instanceBlockIdx.y];
            ptr->perBlockIndex[instanceBlockIdx.y]++;
            ptr->blockListInstanceId[writeIdx] = instanceBlockIdx.x;
        }

        NV_FLOW_PROFILE_TIMESTAMP("EllipsoidBoundsScatter")

        auto mappedBlockIdx = (NvFlowUint*)NvFlowUploadBuffer_map(in->context, &ptr->blockIdxBuffer, ptr->blockListBlockIdx.size * sizeof(NvFlowUint));
        for (NvFlowUint64 idx = 0u; idx < ptr->blockListBlockIdx.size; idx++)
        {
            mappedBlockIdx[idx] = ptr->blockListBlockIdx[idx];
        }
        NvFlowBufferTransient* blockIdxTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->blockIdxBuffer);
        auto mappedRange = (NvFlowUint2*)NvFlowUploadBuffer_map(in->context, &ptr->rangeBuffer, ptr->blockListRange.size * sizeof(NvFlowUint2));
        for (NvFlowUint64 idx = 0u; idx < ptr->blockListRange.size; idx++)
        {
            mappedRange[idx] = ptr->blockListRange[idx];
        }
        NvFlowBufferTransient* rangeTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->rangeBuffer);
        auto mappedInstanceId = (NvFlowUint*)NvFlowUploadBuffer_map(in->context, &ptr->instanceIdBuffer, ptr->blockListInstanceId.size * sizeof(NvFlowUint));
        for (NvFlowUint64 idx = 0u; idx < ptr->blockListInstanceId.size; idx++)
        {
            mappedInstanceId[idx] = ptr->blockListInstanceId[idx];
        }
        NvFlowBufferTransient* instanceIdTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->instanceIdBuffer);

        NvFlowUint batchCount = (NvFlowUint)ptr->blockListBlockIdx.size;

        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init(&batches, batchCount);
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            auto mappedGlobal = (EllipsoidRasterCS_GlobalParams*)NvFlowUploadBuffer_map(in->context, &ptr->globalBuffer, sizeof(EllipsoidRasterCS_GlobalParams));

            mappedGlobal->blockIdxOffset = batches[batchIdx].blockIdxOffset;
            mappedGlobal->totalParticleCount = (NvFlowUint)totalPositionCount;
            mappedGlobal->pad1 = 0u;
            mappedGlobal->pad2 = 0u;
            mappedGlobal->table = *levelParams;

            NvFlowBufferTransient* globalTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->globalBuffer);

            batches[batchIdx].globalTransient = globalTransient;
        }

        auto mappedLayer = (EllipsoidRasterCS_LayerParams*)NvFlowUploadBuffer_map(in->context, &ptr->layerBuffer, numLayers * sizeof(EllipsoidRasterCS_LayerParams));

        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < in->paramCount; layerParamIdx++)
        {
            auto layerParamsIn = in->params[layerParamIdx];

            NvFlowFloat3 blockSizeWorld = in->layout.sparseParams.layers[layerParamIdx].blockSizeWorld;
            NvFlowFloat3 cellSizeWorld = NvFlowFloat3{
                blockSizeWorld.x / float(levelParams->blockDimLessOne.x + 1u),
                blockSizeWorld.y / float(levelParams->blockDimLessOne.y + 1u),
                blockSizeWorld.z / float(levelParams->blockDimLessOne.z + 1u)
            };

            int layerAndLevel = in->layout.sparseParams.layers[layerParamIdx].layerAndLevel;

            NvFlowFloat3 vidxToWorld = {
                1.f / (float(levelParams->blockDimLessOne.x + 1u) / blockSizeWorld.x),
                1.f / (float(levelParams->blockDimLessOne.y + 1u) / blockSizeWorld.y),
                1.f / (float(levelParams->blockDimLessOne.z + 1u) / blockSizeWorld.z)
            };

            mappedLayer[layerParamIdx].particleCount = (NvFlowUint)(layerParamsIn->positionCount + layerParamsIn->positionFloat3Count);
            mappedLayer[layerParamIdx].layerAndLevel = layerAndLevel;
            mappedLayer[layerParamIdx].density = layerParamsIn->density;
            mappedLayer[layerParamIdx].pad1 = 0.f;

            mappedLayer[layerParamIdx].vidxToWorld = vidxToWorld;
            mappedLayer[layerParamIdx].rasterRadiusScale = layerParamsIn->scale;

            mappedLayer[layerParamIdx].blockSizeWorld = blockSizeWorld;
            mappedLayer[layerParamIdx].sdfMode = layerParamsIn->sdfMode;

            mappedLayer[layerParamIdx].cellSizeWorld = cellSizeWorld;
            mappedLayer[layerParamIdx].pad3 = 0.f;

            mappedLayer[layerParamIdx].range_positions = { (NvFlowUint)ptr->arrayFirstElements4[eArray4_count * layerParamIdx + eArray4_positions], (NvFlowUint)(layerParamsIn->positionCount) };
            mappedLayer[layerParamIdx].range_anisotropyE1s = { (NvFlowUint)ptr->arrayFirstElements4[eArray4_count * layerParamIdx + eArray4_anisotropyE1s], (NvFlowUint)(layerParamsIn->anisotropyE1Count) };
            mappedLayer[layerParamIdx].range_anisotropyE2s = { (NvFlowUint)ptr->arrayFirstElements4[eArray4_count * layerParamIdx + eArray4_anisotropyE2s], (NvFlowUint)(layerParamsIn->anisotropyE2Count) };
            mappedLayer[layerParamIdx].range_anisotropyE3s = { (NvFlowUint)ptr->arrayFirstElements4[eArray4_count * layerParamIdx + eArray4_anisotropyE3s], (NvFlowUint)(layerParamsIn->anisotropyE3Count) };

            mappedLayer[layerParamIdx].range_positionFloat3s = { (NvFlowUint)ptr->arrayFirstElements1[eArray1_count * layerParamIdx + eArray1_positionFloat3s], (NvFlowUint)(layerParamsIn->positionFloat3Count) };
            mappedLayer[layerParamIdx].pad4 = 0.f;
            mappedLayer[layerParamIdx].pad5 = 0.f;
        }

        NvFlowBufferTransient* layerTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->layerBuffer);

        NvFlowSparseTexture valueFront = {};
        NvFlowSparseTexture valueBack = {};
        NvFlowSparseTexture_duplicateWithFormat(&ptr->contextInterface, in->context, &valueFront, &in->layout, eNvFlowFormat_r16_float);
        NvFlowSparseTexture_duplicateWithFormat(&ptr->contextInterface, in->context, &valueBack, &in->layout, eNvFlowFormat_r16_float);

        // raster particles directly to texture
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            EllipsoidRasterCS_PassParams params = {};
            params.globalParamsIn = batches[batchIdx].globalTransient;
            params.layerParamsIn = layerTransient;
            params.tableIn = in->layout.sparseBuffer;
            params.arrayValuesIn1 = arraysTransient1;
            params.arrayValuesIn4 = arraysTransient4;
            params.blockIdxIn = blockIdxTransient;
            params.rangeIn = rangeTransient;
            params.instanceIdIn = instanceIdTransient;
            params.valueOut = valueFront.textureTransient;

            NvFlowUint3 gridDim = {};
            gridDim.x = (levelParams->threadsPerBlock + 255u) / 256u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            EllipsoidRasterCS_addPassCompute(in->context, &ptr->ellipsoidRasterCS, gridDim, &params);
        }

        // TODO : support mix iterations counts
        NvFlowUint maxIterations = 0u;
        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < in->paramCount; layerParamIdx++)
        {
            auto layerParamsIn = in->params[layerParamIdx];
            if (layerParamsIn->smoothIterations > maxIterations)
            {
                maxIterations = layerParamsIn->smoothIterations;
            }
        }

        // smooth
        for (NvFlowUint iteration = 0u; iteration < maxIterations; iteration++)
        {
            for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
            {
                EllipsoidRasterSmoothCS_PassParams params = {};
                params.paramsIn = batches[batchIdx].globalTransient;
                params.tableIn = in->layout.sparseBuffer;
                params.blockIdxIn = blockIdxTransient;
                params.rangeIn = rangeTransient;
                params.instanceIdIn = instanceIdTransient;
                params.valueIn = valueFront.textureTransient;
                params.valueOut = valueBack.textureTransient;

                NvFlowUint3 gridDim = {};
                gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
                gridDim.y = batches[batchIdx].blockCount;
                gridDim.z = 1u;

                EllipsoidRasterSmoothCS_addPassCompute(in->context, &ptr->ellipsoidRasterSmoothCS, gridDim, &params);
            }

            auto valueTemp = valueFront;
            valueFront = valueBack;
            valueBack = valueTemp;
        }

        out->value = valueFront;

        NV_FLOW_PROFILE_TIMESTAMP("EllipsoidRasterEnd")
        NV_FLOW_PROFILE_FLUSH("EllipsoidRaster", ptr->contextInterface.getLogPrint(in->context))
    }
}

NV_FLOW_OP_IMPL(NvFlowEllipsoidRaster, EllipsoidRaster)
