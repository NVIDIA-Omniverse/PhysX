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

#include "shaders/PressureParams.h"

#include "NvFlowContext.h"

#include "NvFlowArray.h"
#include "NvFlowMath.h"
#include "NvFlowUploadBuffer.h"

#include "NvFlow.h"

#include "shaders/PressureDivergenceCS.hlsl.h"
#include "shaders/PressureJacobiCS.hlsl.h"
#include "shaders/PressureProlongCS.hlsl.h"
#include "shaders/PressureResidualCS.hlsl.h"
#include "shaders/PressureRestrictCS.hlsl.h"
#include "shaders/PressureSubtractCS.hlsl.h"

namespace
{
    struct Pressure
    {
        NvFlowContextInterface contextInterface = {};

        PressureDivergenceCS_Pipeline divergenceCS;
        PressureJacobiCS_Pipeline jacobiCS;
        PressureProlongCS_Pipeline prolongCS;
        PressureResidualCS_Pipeline residualCS;
        PressureRestrictCS_Pipeline restrictCS;
        PressureSubtractCS_Pipeline subtractCS;

        NvFlowSampler* samplerLinear = nullptr;

        NvFlowUploadBuffer constantBuffer = {};

        NvFlowArray<NvFlowTextureTransient*, 8u> pressureLevels;
        NvFlowArray<NvFlowTextureDesc, 8u> textureDescLevels;
    };

    Pressure* Pressure_create(const NvFlowOpInterface* opInterface, const NvFlowPressurePinsIn* in, NvFlowPressurePinsOut* out)
    {
        auto ptr = new Pressure();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        PressureDivergenceCS_init(&ptr->contextInterface, in->context, &ptr->divergenceCS);
        PressureJacobiCS_init(&ptr->contextInterface, in->context, &ptr->jacobiCS);
        PressureProlongCS_init(&ptr->contextInterface, in->context, &ptr->prolongCS);
        PressureResidualCS_init(&ptr->contextInterface, in->context, &ptr->residualCS);
        PressureRestrictCS_init(&ptr->contextInterface, in->context, &ptr->restrictCS);
        PressureSubtractCS_init(&ptr->contextInterface, in->context, &ptr->subtractCS);

        NvFlowSamplerDesc samplerDesc = {};
        samplerDesc.filterMode = eNvFlowSamplerFilterMode_linear;
        samplerDesc.addressModeU = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeV = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeW = eNvFlowSamplerAddressMode_border;

        ptr->samplerLinear = ptr->contextInterface.createSampler(in->context, &samplerDesc);

        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->constantBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);

        return ptr;
    }

    void Pressure_destroy(Pressure* ptr, const NvFlowPressurePinsIn* in, NvFlowPressurePinsOut* out)
    {
        NvFlowUploadBuffer_destroy(in->context, &ptr->constantBuffer);

        ptr->contextInterface.destroySampler(in->context, ptr->samplerLinear);

        PressureDivergenceCS_destroy(in->context, &ptr->divergenceCS);
        PressureJacobiCS_destroy(in->context, &ptr->jacobiCS);
        PressureProlongCS_destroy(in->context, &ptr->prolongCS);
        PressureResidualCS_destroy(in->context, &ptr->residualCS);
        PressureRestrictCS_destroy(in->context, &ptr->restrictCS);
        PressureSubtractCS_destroy(in->context, &ptr->subtractCS);

        delete ptr;
    }

    void addDivergence(
        NvFlowContext* context,
        Pressure* ptr,
        NvFlowSparseLevelParams* levelParams,
        NvFlowBufferTransient* sparseBuffer,
        NvFlowTextureTransient* velocityIn,
        NvFlowTextureTransient* pressureOut
    )
    {
        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init(&batches, levelParams->numLocations);
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            auto mapped = (PressureDivergenceParams*)NvFlowUploadBuffer_map(context, &ptr->constantBuffer, sizeof(PressureDivergenceParams));

            mapped->blockIdxOffset = batches[batchIdx].blockIdxOffset;
            mapped->dextScale = 1.f;
            mapped->pad1 = 0u;
            mapped->pad2 = 0u;
            mapped->table = *levelParams;

            NvFlowBufferTransient* constantTransient = NvFlowUploadBuffer_unmap(context, &ptr->constantBuffer);

            PressureDivergenceCS_PassParams params = {};
            params.gParams = constantTransient;
            params.gTable = sparseBuffer;
            params.velocityIn = velocityIn;
            params.pressureOut = pressureOut;

            NvFlowUint3 gridDim = {};
            gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            PressureDivergenceCS_addPassCompute(context, &ptr->divergenceCS, gridDim, &params);
        }
    }

    void addSubtract(
        NvFlowContext* context,
        Pressure* ptr,
        NvFlowSparseLevelParams* levelParams,
        NvFlowBufferTransient* sparseBuffer,
        NvFlowTextureTransient* pressureIn,
        NvFlowTextureTransient* velocityIn,
        NvFlowTextureTransient* velocityOut
    )
    {
        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init(&batches, levelParams->numLocations);
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            auto mapped = (PressureSubtractParams*)NvFlowUploadBuffer_map(context, &ptr->constantBuffer, sizeof(PressureSubtractParams));

            mapped->blockIdxOffset = batches[batchIdx].blockIdxOffset;
            mapped->pad1 = 0u;
            mapped->pad2 = 0u;
            mapped->pad3 = 0u;
            mapped->table = *levelParams;

            NvFlowBufferTransient* constantTransient = NvFlowUploadBuffer_unmap(context, &ptr->constantBuffer);

            PressureSubtractCS_PassParams params = {};
            params.gParams = constantTransient;
            params.gTable = sparseBuffer;
            params.velocityIn = velocityIn;
            params.pressureIn = pressureIn;
            params.velocityOut = velocityOut;

            NvFlowUint3 gridDim = {};
            gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            PressureSubtractCS_addPassCompute(context, &ptr->subtractCS, gridDim, &params);
        }
    }

    void addSmooth(
        NvFlowContext* context,
        Pressure* ptr,
        float dx2,
        NvFlowSparseLevelParams* levelParams,
        NvFlowBufferTransient* sparseBuffer,
        NvFlowTextureTransient* pressureIn,
        NvFlowTextureTransient* pressureOut
    )
    {
        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init(&batches, levelParams->numLocations);
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            auto mapped = (PressureJacobiParams*)NvFlowUploadBuffer_map(context, &ptr->constantBuffer, sizeof(PressureJacobiParams));

            mapped->blockIdxOffset = batches[batchIdx].blockIdxOffset;
            mapped->dx2 = dx2;
            mapped->pad1 = 0u;
            mapped->pad2 = 0u;
            mapped->table = *levelParams;

            NvFlowBufferTransient* constantTransient = NvFlowUploadBuffer_unmap(context, &ptr->constantBuffer);

            PressureJacobiCS_PassParams params = {};
            params.gParams = constantTransient;
            params.gTable = sparseBuffer;
            params.pressureIn = pressureIn;
            params.pressureOut = pressureOut;

            NvFlowUint3 gridDim = {};
            gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            PressureJacobiCS_addPassCompute(context, &ptr->jacobiCS, gridDim, &params);
        }
    }

    void addResidual(
        NvFlowContext* context,
        Pressure* ptr,
        float dx2,
        NvFlowSparseLevelParams* levelParams,
        NvFlowBufferTransient* sparseBuffer,
        NvFlowTextureTransient* pressureIn,
        NvFlowTextureTransient* residualOut
    )
    {
        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init(&batches, levelParams->numLocations);
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            auto mapped = (PressureResidualParams*)NvFlowUploadBuffer_map(context, &ptr->constantBuffer, sizeof(PressureResidualParams));

            mapped->blockIdxOffset = batches[batchIdx].blockIdxOffset;
            mapped->dx2Inv = 1.f / dx2;
            mapped->pad1 = 0u;
            mapped->pad2 = 0u;
            mapped->table = *levelParams;

            NvFlowBufferTransient* constantTransient = NvFlowUploadBuffer_unmap(context, &ptr->constantBuffer);

            PressureResidualCS_PassParams params = {};
            params.gParams = constantTransient;
            params.gTable = sparseBuffer;
            params.pressureIn = pressureIn;
            params.residualOut = residualOut;

            NvFlowUint3 gridDim = {};
            gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            PressureResidualCS_addPassCompute(context, &ptr->residualCS, gridDim, &params);
        }
    }

    void addRestrict(
        NvFlowContext* context,
        Pressure* ptr,
        NvFlowSparseLevelParams* fineLevelParams,
        NvFlowSparseLevelParams* coarseLevelParams,
        NvFlowBufferTransient* sparseBuffer,
        NvFlowTextureTransient* fineResidualIn,
        NvFlowTextureTransient* coarsePressureOut
    )
    {
        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init(&batches, coarseLevelParams->numLocations);
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            auto mapped = (PressureRestrictParams*)NvFlowUploadBuffer_map(context, &ptr->constantBuffer, sizeof(PressureRestrictParams));

            mapped->blockIdxOffset = batches[batchIdx].blockIdxOffset;
            mapped->pad1 = 0u;
            mapped->pad2 = 0u;
            mapped->pad3 = 0u;
            mapped->tableFine = *fineLevelParams;
            mapped->tableCoarse = *coarseLevelParams;

            NvFlowBufferTransient* constantTransient = NvFlowUploadBuffer_unmap(context, &ptr->constantBuffer);

            PressureRestrictCS_PassParams params = {};
            params.gParams = constantTransient;
            params.gTable = sparseBuffer;
            params.residualIn = fineResidualIn;
            params.valueSampler = ptr->samplerLinear;
            params.pressureOut = coarsePressureOut;

            NvFlowUint3 gridDim = {};
            gridDim.x = (coarseLevelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            PressureRestrictCS_addPassCompute(context, &ptr->restrictCS, gridDim, &params);
        }
    }

    void addProlong(
        NvFlowContext* context,
        Pressure* ptr,
        NvFlowSparseLevelParams* fineLevelParams,
        NvFlowSparseLevelParams* coarseLevelParams,
        NvFlowBufferTransient* sparseBuffer,
        NvFlowTextureTransient* finePressureIn,
        NvFlowTextureTransient* coarsePressureIn,
        NvFlowTextureTransient* finePressureOut
    )
    {
        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init(&batches, fineLevelParams->numLocations);
        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            auto mapped = (PressureProlongParams*)NvFlowUploadBuffer_map(context, &ptr->constantBuffer, sizeof(PressureProlongParams));

            mapped->blockIdxOffset = batches[batchIdx].blockIdxOffset;
            mapped->pad1 = 0u;
            mapped->pad2 = 0u;
            mapped->pad3 = 0u;
            mapped->tableFine = *fineLevelParams;
            mapped->tableCoarse = *coarseLevelParams;

            NvFlowBufferTransient* constantTransient = NvFlowUploadBuffer_unmap(context, &ptr->constantBuffer);

            PressureProlongCS_PassParams params = {};
            params.gParams = constantTransient;
            params.gTable = sparseBuffer;
            params.pressureFineIn = finePressureIn;
            params.pressureCoarseIn = coarsePressureIn;
            params.valueSampler = ptr->samplerLinear;
            params.pressureFineOut = finePressureOut;

            NvFlowUint3 gridDim = {};
            gridDim.x = (fineLevelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            PressureProlongCS_addPassCompute(context, &ptr->prolongCS, gridDim, &params);
        }
    }

    void swap(NvFlowTextureTransient** pA, NvFlowTextureTransient** pB)
    {
        NvFlowTextureTransient* temp = *pA;
        *pA = *pB;
        *pB = temp;
    }

    void addPassesInternal(
        NvFlowContext* context,
        Pressure* ptr,
        NvFlowSparseParams* sparseParams,
        NvFlowBufferTransient* sparseBuffer,
        NvFlowTextureTransient* velocityIn,
        NvFlowSparseTexture* pVelocityOut
    )
    {
        bool enableMultigrid = true;
        NvFlowUint numLevels = enableMultigrid ? sparseParams->levelCount : 1u;

        ptr->textureDescLevels.reserve(numLevels);
        ptr->textureDescLevels.size = numLevels;

        for (NvFlowUint levelIdx = 0u; levelIdx < numLevels; levelIdx++)
        {
            bool isLowPrecision = pVelocityOut->format == eNvFlowFormat_r8g8b8a8_unorm;
            bool isHighPrecision = pVelocityOut->format == eNvFlowFormat_r32g32b32a32_float;
            NvFlowFormat pressure_format =
                isHighPrecision ? eNvFlowFormat_r32g32_float : (
                isLowPrecision ? eNvFlowFormat_r8g8_unorm : eNvFlowFormat_r16g16_float);

            NvFlowTextureDesc pressureTexDesc = {};
            pressureTexDesc.textureType = eNvFlowTextureType_3d;
            pressureTexDesc.usageFlags = eNvFlowTextureUsage_rwTexture | eNvFlowTextureUsage_texture;
            pressureTexDesc.format = pressure_format;
            pressureTexDesc.width = sparseParams->levels[levelIdx].dim.x;
            pressureTexDesc.height = sparseParams->levels[levelIdx].dim.y;
            pressureTexDesc.depth = sparseParams->levels[levelIdx].dim.z;
            pressureTexDesc.mipLevels = 1u;

            ptr->textureDescLevels[levelIdx] = pressureTexDesc;
        }

        // allocate pressure levels
        ptr->pressureLevels.reserve(numLevels);
        ptr->pressureLevels.size = numLevels;
        for (NvFlowUint levelIdx = 0u; levelIdx < numLevels; levelIdx++)
        {
            ptr->pressureLevels[levelIdx] = ptr->contextInterface.getTextureTransient(context, &ptr->textureDescLevels[levelIdx]);
        }

        NvFlowTextureDesc velocityTexDesc = ptr->textureDescLevels[0u];
        velocityTexDesc.format = pVelocityOut->format;

        pVelocityOut->textureTransient = ptr->contextInterface.getTextureTransient(context, &velocityTexDesc);

        addDivergence(context, ptr, &sparseParams->levels[0], sparseBuffer, velocityIn, ptr->pressureLevels[0u]);

        if (numLevels > 1)
        {
            NvFlowUint fineIterations = 1u;
            NvFlowUint coarseIterations = 4u;

            // For N - 1 levels, fine smooth and restrict
            for (NvFlowUint fineLevelIdx = 0u; fineLevelIdx < (numLevels - 1u); fineLevelIdx++)
            {
                float dx2 = float(1 << (2u * fineLevelIdx));

                NvFlowTextureTransient* pressureTemp = ptr->contextInterface.getTextureTransient(context, &ptr->textureDescLevels[fineLevelIdx]);

                // smooth
                for (NvFlowUint iteration = 0u; iteration < fineIterations; iteration++)
                {
                    addSmooth(
                        context,
                        ptr,
                        dx2,
                        &sparseParams->levels[fineLevelIdx],
                        sparseBuffer,
                        ptr->pressureLevels[fineLevelIdx],
                        pressureTemp
                    );

                    swap(&pressureTemp, &ptr->pressureLevels[fineLevelIdx]);
                }

                // recycle
                NvFlowTextureTransient* residual = pressureTemp;

                addResidual(
                    context,
                    ptr,
                    dx2,
                    &sparseParams->levels[fineLevelIdx],
                    sparseBuffer,
                    ptr->pressureLevels[fineLevelIdx],
                    residual
                );

                addRestrict(
                    context,
                    ptr,
                    &sparseParams->levels[fineLevelIdx],
                    &sparseParams->levels[fineLevelIdx + 1u],
                    sparseBuffer,
                    residual,
                    ptr->pressureLevels[fineLevelIdx + 1u]
                );
            }

            // At N - 1 level, coarse smooth
            {
                NvFlowUint levelIdx = numLevels - 1u;

                float dx2 = float(1 << (2u * levelIdx));

                NvFlowTextureTransient* pressureTemp = ptr->contextInterface.getTextureTransient(context, &ptr->textureDescLevels[levelIdx]);

                // smooth
                for (NvFlowUint iteration = 0u; iteration < coarseIterations; iteration++)
                {
                    addSmooth(
                        context,
                        ptr,
                        dx2,
                        &sparseParams->levels[levelIdx],
                        sparseBuffer,
                        ptr->pressureLevels[levelIdx],
                        pressureTemp
                    );

                    swap(&pressureTemp, &ptr->pressureLevels[levelIdx]);
                }
            }

            // For N - 1 levels, prolong and smooth
            for (NvFlowUint fineLevelIdx = numLevels - 2u; fineLevelIdx < numLevels; fineLevelIdx--)
            {
                float dx2 = float(1 << (2u * fineLevelIdx));

                NvFlowTextureTransient* pressureTemp = ptr->contextInterface.getTextureTransient(context, &ptr->textureDescLevels[fineLevelIdx]);

                // prolong
                addProlong(
                    context,
                    ptr,
                    &sparseParams->levels[fineLevelIdx],
                    &sparseParams->levels[fineLevelIdx + 1u],
                    sparseBuffer,
                    ptr->pressureLevels[fineLevelIdx],
                    ptr->pressureLevels[fineLevelIdx + 1u],
                    pressureTemp
                );

                swap(&pressureTemp, &ptr->pressureLevels[fineLevelIdx]);

                // smooth
                for (NvFlowUint iteration = 0u; iteration < fineIterations; iteration++)
                {
                    addSmooth(
                        context,
                        ptr,
                        dx2,
                        &sparseParams->levels[fineLevelIdx],
                        sparseBuffer,
                        ptr->pressureLevels[fineLevelIdx],
                        pressureTemp
                    );

                    swap(&pressureTemp, &ptr->pressureLevels[fineLevelIdx]);
                }
            }
        }
        else
        {
            float dx2 = 1.f;

            NvFlowTextureTransient* pressureTemp = ptr->contextInterface.getTextureTransient(context, &ptr->textureDescLevels[0u]);

            // jacobi iteration
            for (NvFlowUint idx = 0u; idx < 40u; idx++)
            {
                addSmooth(context, ptr, dx2, &sparseParams->levels[0], sparseBuffer, ptr->pressureLevels[0u], pressureTemp);

                swap(&pressureTemp, &ptr->pressureLevels[0u]);
            }
        }

        addSubtract(context, ptr, &sparseParams->levels[0], sparseBuffer, ptr->pressureLevels[0u], velocityIn, pVelocityOut->textureTransient);
    }

    void Pressure_execute(Pressure* ptr, const NvFlowPressurePinsIn* in, NvFlowPressurePinsOut* out)
    {
        if (in->velocity.sparseParams.levels[in->velocity.levelIdx].numLocations == 0u)
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

        // addPassInternal will override velocityOut
        NvFlowSparseTexture_passThrough(&out->velocity, &in->velocity);

        NvFlowSparseParams sparseParams = {};
        sparseParams.levelCount = in->velocity.sparseParams.levelCount - in->velocity.levelIdx;
        sparseParams.levels = in->velocity.sparseParams.levels + in->velocity.levelIdx;

        NvFlowBufferTransient* sparseBuffer = in->velocity.sparseBuffer;

        addPassesInternal(in->context, ptr, &sparseParams, sparseBuffer, in->velocity.textureTransient, &out->velocity);
    }
}

NV_FLOW_OP_IMPL(NvFlowPressure, Pressure)
