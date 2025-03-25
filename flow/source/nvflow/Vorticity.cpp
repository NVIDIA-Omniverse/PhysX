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

#include "shaders/VorticityParams.h"

#include "NvFlowContext.h"

#include "NvFlowArray.h"
#include "NvFlowMath.h"
#include "NvFlowUploadBuffer.h"

#include "NvFlow.h"

#include "shaders/Vorticity1CS.hlsl.h"
#include "shaders/Vorticity2CS.hlsl.h"

namespace
{
    struct Vorticity
    {
        NvFlowContextInterface contextInterface = {};

        Vorticity1CS_Pipeline vorticity1CS = {};
        Vorticity2CS_Pipeline vorticity2CS = {};

        NvFlowUploadBuffer globalBuffer = {};
        NvFlowUploadBuffer layerBuffer = {};
    };

    Vorticity* Vorticity_create(const NvFlowOpInterface* opInterface, const NvFlowVorticityPinsIn* in, NvFlowVorticityPinsOut* out)
    {
        auto ptr = new Vorticity();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        Vorticity1CS_init(&ptr->contextInterface, in->context, &ptr->vorticity1CS);
        Vorticity2CS_init(&ptr->contextInterface, in->context, &ptr->vorticity2CS);

        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->globalBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);
        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->layerBuffer, eNvFlowBufferUsage_constantBuffer | eNvFlowBufferUsage_structuredBuffer, eNvFlowFormat_unknown, sizeof(VorticityCS_LayerParams));

        return ptr;
    }

    void Vorticity_destroy(Vorticity* ptr, const NvFlowVorticityPinsIn* in, NvFlowVorticityPinsOut* out)
    {
        NvFlowUploadBuffer_destroy(in->context, &ptr->globalBuffer);
        NvFlowUploadBuffer_destroy(in->context, &ptr->layerBuffer);

        Vorticity1CS_destroy(in->context, &ptr->vorticity1CS);
        Vorticity2CS_destroy(in->context, &ptr->vorticity2CS);

        delete ptr;
    }

    void Vorticity_execute(Vorticity* ptr, const NvFlowVorticityPinsIn* in, NvFlowVorticityPinsOut* out)
    {
        NvFlowSparseLevelParams* velocityLevelParams = &in->velocity.sparseParams.levels[in->velocity.levelIdx];

        if (velocityLevelParams->numLocations == 0u)
        {
            NvFlowSparseTexture_passThrough(&out->velocity, &in->velocity);
            return;
        }

        NvFlowUint numLayers = in->velocity.sparseParams.layerCount;

        // if all layers are disabled, can do passthrough
        NvFlowBool32 allDisabled = NV_FLOW_TRUE;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < numLayers; layerParamIdx++)
        {
            auto layerParamsIn = in->params[layerParamIdx];
            if (layerParamsIn->enabled && !in->velocity.sparseParams.layers[layerParamIdx].forceDisableCoreSimulation)
            {
                allDisabled = NV_FLOW_FALSE;
            }
        }
        if (allDisabled)
        {
            NvFlowSparseTexture_passThrough(&out->velocity, &in->velocity);
            return;
        }

        NvFlowSparseTexture curlTransient = {};
        NvFlowSparseTexture lowpassTransient = {};
        NvFlowSparseTexture_duplicate(&ptr->contextInterface, in->context, &curlTransient, &in->velocity);
        NvFlowSparseTexture_duplicate(&ptr->contextInterface, in->context, &lowpassTransient, &in->velocity);

        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init(&batches, velocityLevelParams->numLocations);

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            auto mappedGlobal = (VorticityCS_GlobalParams*)NvFlowUploadBuffer_map(in->context, &ptr->globalBuffer, sizeof(VorticityCS_GlobalParams));

            mappedGlobal->blockIdxOffset = batches[batchIdx].blockIdxOffset;
            mappedGlobal->pad1 = 0u;
            mappedGlobal->pad2 = 0u;
            mappedGlobal->pad3 = 0u;
            mappedGlobal->table = *velocityLevelParams;

            NvFlowBufferTransient* globalTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->globalBuffer);

            batches[batchIdx].globalTransient = globalTransient;
        }

        auto mappedLayer = (VorticityCS_LayerParams*)NvFlowUploadBuffer_map(in->context, &ptr->layerBuffer, numLayers * sizeof(VorticityCS_LayerParams));

        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < in->paramCount; layerParamIdx++)
        {
            auto layerParamsIn = in->params[layerParamIdx];

            mappedLayer[layerParamIdx].forceScale = layerParamsIn->forceScale * in->deltaTime;
            mappedLayer[layerParamIdx].velocityLogScale = fmaxf(layerParamsIn->velocityLogScale, 0.f);
            mappedLayer[layerParamIdx].velocityLogMask = layerParamsIn->velocityMask;
            mappedLayer[layerParamIdx].velocityLinearMask = layerParamsIn->velocityLinearMask;

            mappedLayer[layerParamIdx].constantMask = layerParamsIn->constantMask;
            mappedLayer[layerParamIdx].densityMask = layerParamsIn->densityMask;
            mappedLayer[layerParamIdx].pad2 = 0.f;
            mappedLayer[layerParamIdx].pad3 = 0.f;

            mappedLayer[layerParamIdx].temperatureMask = layerParamsIn->temperatureMask;
            mappedLayer[layerParamIdx].fuelMask = layerParamsIn->fuelMask;
            mappedLayer[layerParamIdx].burnMask = layerParamsIn->burnMask;
            mappedLayer[layerParamIdx].smokeMask = layerParamsIn->smokeMask;
        }

        NvFlowBufferTransient* layerTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->layerBuffer);

        // part 1
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            Vorticity1CS_PassParams params = {};
            params.globalParamsIn = batches[batchIdx].globalTransient;
            params.layerParamsIn = layerTransient;
            params.tableIn = in->velocity.sparseBuffer;
            params.velocityIn = in->velocity.textureTransient;
            params.curlOut = curlTransient.textureTransient;
            params.lowpassOut = lowpassTransient.textureTransient;

            NvFlowUint3 gridDim = {};
            gridDim.x = (velocityLevelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            Vorticity1CS_addPassCompute(in->context, &ptr->vorticity1CS, gridDim, &params);
        }

        NvFlowSparseTexture_duplicate(&ptr->contextInterface, in->context, &out->velocity, &in->velocity);

        // part 2
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            Vorticity2CS_PassParams params = {};
            params.globalParamsIn = batches[batchIdx].globalTransient;
            params.layerParamsIn = layerTransient;
            params.tableIn = in->velocity.sparseBuffer;
            params.velocityIn = in->velocity.textureTransient;
            params.curlIn = curlTransient.textureTransient;
            params.lowpassIn = lowpassTransient.textureTransient;
            params.coarseDensityIn = in->coarseDensity.textureTransient;
            params.velocityOut = out->velocity.textureTransient;

            NvFlowUint3 gridDim = {};
            gridDim.x = (velocityLevelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            Vorticity2CS_addPassCompute(in->context, &ptr->vorticity2CS, gridDim, &params);
        }
    }
}

NV_FLOW_OP_IMPL(NvFlowVorticity, Vorticity)
