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

#include "shaders/EmitterParams.h"

#include "EmitterCommon.h"

#include "shaders/EmitterSimpleCS.hlsl.h"

namespace
{
    struct EmitterSphere
    {
        NvFlowContextInterface contextInterface = {};

        EmitterSimpleCS_Pipeline emitterSimpleCS = {};

        NvFlowSampler* samplerLinear = nullptr;

        NvFlowUploadBuffer constantBuffer = {};

        NvFlowArray<NvFlowFloat4x4> oldLocalToWorlds;
    };

    NvFlowUint EmitterSphere_clampSubSteps(NvFlowUint numSubSteps)
    {
        if (numSubSteps == 0u)
        {
            numSubSteps = 1u;
        }
        if (numSubSteps > EMITTER_MAX_SUBSTEPS)
        {
            numSubSteps = EMITTER_MAX_SUBSTEPS;
        }
        return numSubSteps;
    }

    NV_FLOW_INLINE NvFlowFloat4x4 EmitterSphere_matrixInterpolate(const NvFlowFloat4x4& a, const NvFlowFloat4x4& b, float t)
    {
        return NvFlowMath::matrixInterpolateTranslation(a, b, t);
    }

    NV_FLOW_INLINE NvFlowFloat4x4 EmitterSphere_getLocalToWorld(const NvFlowEmitterSphereParams* params)
    {
        if (params->radiusIsWorldSpace)
        {
            return NvFlowFloat4x4{
                1.f, 0.f, 0.f, 0.f,
                0.f, 1.f, 0.f, 0.f,
                0.f, 0.f, 1.f, 0.f,
                NvFlowMath::vector4Transform(NvFlowMath::make_float4(params->position, 1.f), params->localToWorld)
            };
        }
        return params->localToWorld;
    }

    NV_FLOW_INLINE NvFlowFloat3 EmitterSphere_getPosition(const NvFlowEmitterSphereParams* params)
    {
        if (params->radiusIsWorldSpace)
        {
            return NvFlowFloat3{ 0.f, 0.f, 0.f };
        }
        return params->position;
    }

    NV_FLOW_INLINE NvFlowFloat3 EmitterSphere_getVelocity(const NvFlowEmitterSphereParams* params)
    {
        if (params->velocityIsWorldSpace)
        {
            return params->velocity;
        }
        NvFlowFloat4 vel = { params->velocity.x, params->velocity.y, params->velocity.z, 0.f };
        vel = NvFlowMath::vector4Transform(vel, params->localToWorldVelocity);
        return NvFlowFloat3{ vel.x, vel.y, vel.z };
    }

    void EmitterSphere_computeFootprint(
        NvFlowInt4* pLocationMin,
        NvFlowInt4* pLocationMax,
        const NvFlowEmitterSphereParams* params,
        const NvFlowSparseLayerParams* layerParams,
        const NvFlowFloat4x4 oldLocalToWorld,
        const NvFlowFloat4x4 localToWorld,
        bool scaleAlloc
    )
    {
        using namespace NvFlowMath;

        (*pLocationMin) = NvFlowInt4{ 0, 0, 0, 0 };
        (*pLocationMax) = NvFlowInt4{ 0, 0, 0, 0 };

        NvFlowFloat3 halfSize = NvFlowFloat3{ params->radius, params->radius, params->radius };
        if (scaleAlloc)
        {
            halfSize.x *= params->allocationScale;
            halfSize.y *= params->allocationScale;
            halfSize.z *= params->allocationScale;
        }

        NvFlowUint numSubSteps = EmitterSphere_clampSubSteps(params->numSubSteps);
        float numSubStepsInv = 1.f / float(numSubSteps);
        for (NvFlowUint subStepIdx = 0u; subStepIdx < numSubSteps; subStepIdx++)
        {
            float w = float(subStepIdx + 1u) * numSubStepsInv;

            NvFlowFloat4x4 localToWorldInterp = EmitterSphere_matrixInterpolate(oldLocalToWorld, localToWorld, w);

            NvFlowInt4 locationMinLocal = {};
            NvFlowInt4 locationMaxLocal = {};
            computeEmitterBoxBounds(
                localToWorldInterp,
                params->layer,
                params->level,
                EmitterSphere_getPosition(params),
                halfSize,
                layerParams->blockSizeWorld,
                &locationMinLocal,
                &locationMaxLocal
            );
            if (subStepIdx == 0u)
            {
                (*pLocationMin) = locationMinLocal;
                (*pLocationMax) = locationMaxLocal;
            }
            (*pLocationMin) = emitterLocationMin((*pLocationMin), locationMinLocal);
            (*pLocationMax) = emitterLocationMax((*pLocationMax), locationMaxLocal);
        }
    }

    EmitterSphere* EmitterSphere_create(const NvFlowOpInterface* opInterface, const NvFlowEmitterSpherePinsIn* in, NvFlowEmitterSpherePinsOut* out)
    {
        auto ptr = new EmitterSphere();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, in->contextInterface);

        EmitterSimpleCS_init(&ptr->contextInterface, in->context, &ptr->emitterSimpleCS);

        NvFlowSamplerDesc samplerDesc = {};
        samplerDesc.filterMode = eNvFlowSamplerFilterMode_linear;
        samplerDesc.addressModeU = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeV = eNvFlowSamplerAddressMode_border;
        samplerDesc.addressModeW = eNvFlowSamplerAddressMode_border;

        ptr->samplerLinear = ptr->contextInterface.createSampler(in->context, &samplerDesc);

        NvFlowUploadBuffer_init(&ptr->contextInterface, in->context, &ptr->constantBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);

        return ptr;
    }

    void EmitterSphere_destroy(EmitterSphere* ptr, const NvFlowEmitterSpherePinsIn* in, NvFlowEmitterSpherePinsOut* out)
    {
        NvFlowUploadBuffer_destroy(in->context, &ptr->constantBuffer);

        ptr->contextInterface.destroySampler(in->context, ptr->samplerLinear);

        EmitterSimpleCS_destroy(in->context, &ptr->emitterSimpleCS);

        delete ptr;
    }

    void EmitterSphere_executeSingleEmitter(
        EmitterSphere* ptr,
        NvFlowUint emitterIdx,
        const NvFlowEmitterSpherePinsIn* in,
        NvFlowEmitterSpherePinsOut* out,
        NvFlowSparseLevelParams* levelParams,
        NvFlowSparseLevelParams* velocityLevelParams,
        NvFlowSparseTexture* valueTemp,
        const NvFlowEmitterSphereParams* params,
        NvFlowBool32 isVelocity
    )
    {
        if (!params->enabled)
        {
            return;
        }
        // No impact if zero active blocks
        if (levelParams->numLocations == 0u)
        {
            return;
        }
        // Apply post pressure if requested
        if (isVelocity)
        {
            if (in->isPostPressure && !params->applyPostPressure)
            {
                return;
            }
            if (!in->isPostPressure && params->applyPostPressure)
            {
                return;
            }
        }

        NvFlowUint layerParamIdx = NvFlowSparseParams_layerToLayerParamIdx(&in->value.sparseParams, params->layer, params->level);
        // early out if layer is not active
        if (layerParamIdx == ~0u)
        {
            return;
        }
        const NvFlowSparseLayerParams* layerParams = &in->value.sparseParams.layers[layerParamIdx];
        if (layerParams->forceDisableEmitters)
        {
            return;
        }

        // Early out if geometry has no volume
        if (params->radius <= 0.f)
        {
            return;
        }
        // Early out if couple rates leave emitter having zero effect
        if (!isVelocity && params->coupleRateTemperature <= 0.f && params->coupleRateFuel <= 0.f && params->coupleRateBurn <= 0.f && params->coupleRateSmoke <= 0.f)
        {
            return;
        }
        if (isVelocity && params->coupleRateVelocity <= 0.f && params->coupleRateDivergence <= 0.f)
        {
            return;
        }

        using namespace NvFlowMath;

        NvFlowFloat3 blockSizeWorld = layerParams->blockSizeWorld;

        NvFlowFloat3 vidxToWorld = {
            blockSizeWorld.x / float(levelParams->blockDimLessOne.x + 1u),
            blockSizeWorld.y / float(levelParams->blockDimLessOne.y + 1u),
            blockSizeWorld.z / float(levelParams->blockDimLessOne.z + 1u)
        };

        float radius = params->radius;
        float radius2 = radius * radius;

        NvFlowFloat4x4 localToWorld = EmitterSphere_getLocalToWorld(params);
        NvFlowFloat4x4 oldLocalToWorld = localToWorld;
        if (emitterIdx < ptr->oldLocalToWorlds.size)
        {
            oldLocalToWorld = ptr->oldLocalToWorlds[emitterIdx];
        }

        NvFlowInt4 locationMin = {};
        NvFlowInt4 locationMax = {};
        bool scaleAlloc = params->numTraceSamples != 0u && params->allocationScale > 1.f;
        EmitterSphere_computeFootprint(&locationMin, &locationMax, params, layerParams, oldLocalToWorld, localToWorld, scaleAlloc);

        NvFlowUint numLocations = (locationMax.x - locationMin.x) * (locationMax.y - locationMin.y) * (locationMax.z - locationMin.z);

        NvFlowFloat3 valueBlockDimf = {
            float(levelParams->blockDimLessOne.x + 1u),
            float(levelParams->blockDimLessOne.y + 1u),
            float(levelParams->blockDimLessOne.z + 1u)
        };
        NvFlowFloat3 velocityBlockDimf = {
            float(velocityLevelParams->blockDimLessOne.x + 1u),
            float(velocityLevelParams->blockDimLessOne.y + 1u),
            float(velocityLevelParams->blockDimLessOne.z + 1u)
        };

        NvFlowDispatchBatches batches;
        NvFlowDispatchBatches_init(&batches, numLocations);

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            auto mapped = (EmitterSphereCS_Params*)NvFlowUploadBuffer_map(in->context, &ptr->constantBuffer, sizeof(EmitterSphereCS_Params));

            mapped->blockIdxOffset = batches[batchIdx].blockIdxOffset;
            mapped->pad1 = 0u;
            mapped->pad2 = 0u;
            mapped->pad3 = 0u;

            mapped->table = *levelParams;

            if (isVelocity)
            {
                NvFlowFloat3 velocity = EmitterSphere_getVelocity(params);
                mapped->targetValue = NvFlowFloat4{
                    velocity.x,
                    velocity.y,
                    velocity.z,
                    params->divergence
                };
                mapped->coupleRate = NvFlowFloat4{
                    params->coupleRateVelocity,
                    params->coupleRateVelocity,
                    params->coupleRateVelocity,
                    params->coupleRateDivergence
                };
            }
            else
            {
                mapped->targetValue = NvFlowFloat4{
                    params->temperature,
                    params->fuel,
                    params->burn,
                    params->smoke
                };
                mapped->coupleRate = NvFlowFloat4{
                    params->coupleRateTemperature,
                    params->coupleRateFuel,
                    params->coupleRateBurn,
                    params->coupleRateSmoke
                };
            }

            mapped->vidxToWorld = vidxToWorld;
            mapped->pad4 = 0.f;

            NvFlowUint numSubSteps = EmitterSphere_clampSubSteps(params->numSubSteps);
            float numSubStepsInv = 1.f / float(numSubSteps);
            for (NvFlowUint subStepIdx = 0u; subStepIdx < numSubSteps; subStepIdx++)
            {
                float w = float(subStepIdx + 1u) * numSubStepsInv;
                float wOld = float(subStepIdx) * numSubStepsInv;

                NvFlowFloat4x4 localToWorldInterp = EmitterSphere_matrixInterpolate(oldLocalToWorld, localToWorld, w);
                NvFlowFloat4x4 oldLocalToWorldInterp = EmitterSphere_matrixInterpolate(oldLocalToWorld, localToWorld, wOld);

                mapped->worldToLocals[subStepIdx] = matrixTranspose(matrixInverse(localToWorldInterp));
                mapped->localToWorldOlds[subStepIdx] = matrixTranspose(oldLocalToWorldInterp);
            }

            mapped->physicsVelocityScale = isVelocity ? params->physicsVelocityScale : 0.f;
            mapped->physicsDeltaTimeInv = 1.f / (in->deltaTime);
            mapped->multisample = params->multisample;
            mapped->numSubSteps = EmitterSphere_clampSubSteps(params->numSubSteps);

            mapped->numSubStepsInv = 1.f / float(EmitterSphere_clampSubSteps(params->numSubSteps));
            mapped->radius = radius;
            mapped->radius2 = radius2;
            mapped->deltaTime = 0.5f * in->deltaTime;

            mapped->position = EmitterSphere_getPosition(params);
            mapped->pad5 = 0.f;

            mapped->locationOffset = locationMin;
            mapped->locationExtent = NvFlowUint4{
                NvFlowUint(locationMax.x - locationMin.x),
                NvFlowUint(locationMax.y - locationMin.y),
                NvFlowUint(locationMax.z - locationMin.z),
                NvFlowUint(locationMax.w - locationMin.w)
            };

            mapped->velocityTable = *velocityLevelParams;

            mapped->cellSizeInv.x = 1.f / vidxToWorld.x;
            mapped->cellSizeInv.y = 1.f / vidxToWorld.y;
            mapped->cellSizeInv.z = 1.f / vidxToWorld.z;
            mapped->numTraceSamples = params->numTraceSamples;
            mapped->valueToVelocityBlockScale.x = velocityBlockDimf.x / valueBlockDimf.x;
            mapped->valueToVelocityBlockScale.y = velocityBlockDimf.y / valueBlockDimf.y;
            mapped->valueToVelocityBlockScale.z = velocityBlockDimf.z / valueBlockDimf.z;
            mapped->traceDeltaTime = params->traceDeltaTime;

            NvFlowBufferTransient* constantTransient = NvFlowUploadBuffer_unmap(in->context, &ptr->constantBuffer);

            batches[batchIdx].globalTransient = constantTransient;
        }

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            EmitterSimpleCS_PassParams passParams = {};
            passParams.gParams = batches[batchIdx].globalTransient;
            passParams.gTable = in->value.sparseBuffer;
            passParams.valueIn = in->value.textureTransient;
            passParams.velocityIn = in->velocity.textureTransient;
            passParams.samplerIn = ptr->samplerLinear;
            passParams.valueOut = valueTemp->textureTransient;

            NvFlowUint3 gridDim = {};
            gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            EmitterSimpleCS_addPassCompute(in->context, &ptr->emitterSimpleCS, gridDim, &passParams);
        }

        for (NvFlowUint64 batchIdx = 0u; batchIdx < batches.size; batchIdx++)
        {
            EmitterSimpleCS_PassParams passParams = {};
            passParams.gParams = batches[batchIdx].globalTransient;
            passParams.gTable = in->value.sparseBuffer;
            passParams.valueIn = valueTemp->textureTransient;
            passParams.velocityIn = in->velocity.textureTransient;
            passParams.samplerIn = ptr->samplerLinear;
            passParams.valueOut = out->value.textureTransient;

            NvFlowUint3 gridDim = {};
            gridDim.x = (levelParams->threadsPerBlock + 127u) / 128u;
            gridDim.y = batches[batchIdx].blockCount;
            gridDim.z = 1u;

            EmitterSimpleCS_addPassCompute(in->context, &ptr->emitterSimpleCS, gridDim, &passParams);
        }
    }

    void EmitterSphere_execute(EmitterSphere* ptr, const NvFlowEmitterSpherePinsIn* in, NvFlowEmitterSpherePinsOut* out)
    {
        NvFlowSparseLevelParams* levelParams = &in->value.sparseParams.levels[in->value.levelIdx];
        NvFlowSparseLevelParams* velocityLevelParams = &in->velocity.sparseParams.levels[in->velocity.levelIdx];

        // passthrough, since input is mutable
        NvFlowSparseTexture_passThrough(&out->value, &in->value);

        NvFlowSparseTexture valueTemp = in->valueTemp;
        if (!valueTemp.textureTransient)
        {
            NvFlowSparseTexture_duplicate(&ptr->contextInterface, in->context, &valueTemp, &in->value);
        }

        for (NvFlowUint velocityIdx = 0u; velocityIdx < in->velocityParamCount; velocityIdx++)
        {
            const NvFlowEmitterSphereParams* params = in->velocityParams[velocityIdx];
            EmitterSphere_executeSingleEmitter(ptr, velocityIdx, in, out, levelParams, velocityLevelParams, &valueTemp, params, NV_FLOW_TRUE);
        }
        for (NvFlowUint densityIdx = 0u; densityIdx < in->densityParamCount; densityIdx++)
        {
            const NvFlowEmitterSphereParams* params = in->densityParams[densityIdx];
            EmitterSphere_executeSingleEmitter(ptr, densityIdx, in, out, levelParams, velocityLevelParams, &valueTemp, params, NV_FLOW_FALSE);
        }

        // Capture old positions after post pressure emitter
        if (in->isPostPressure && in->velocityParamCount > 0u)
        {
            ptr->oldLocalToWorlds.size = 0u;
            for (NvFlowUint velocityIdx = 0u; velocityIdx < in->velocityParamCount; velocityIdx++)
            {
                const NvFlowEmitterSphereParams* params = in->velocityParams[velocityIdx];
                ptr->oldLocalToWorlds.pushBack(EmitterSphere_getLocalToWorld(params));
            }
        }
    }
}

NV_FLOW_OP_IMPL(NvFlowEmitterSphere, EmitterSphere)

namespace
{
    struct EmitterSphereAllocate
    {
        NvFlowArray<NvFlowInt4> locationsTmp;
        NvFlowArray<NvFlowFloat3> oldPositions;
        NvFlowArray<NvFlowFloat4x4> oldLocalToWorlds;

        NvFlowUint cachedMaxLocations = 0u;
    };

    EmitterSphereAllocate* EmitterSphereAllocate_create(const NvFlowOpInterface* opInterface, const NvFlowEmitterSphereAllocatePinsIn* in, NvFlowEmitterSphereAllocatePinsOut* out)
    {
        auto ptr = new EmitterSphereAllocate();
        return ptr;
    }

    void EmitterSphereAllocate_destroy(EmitterSphereAllocate* ptr, const NvFlowEmitterSphereAllocatePinsIn* in, NvFlowEmitterSphereAllocatePinsOut* out)
    {
        delete ptr;
    }

    void EmitterSphereAllocate_execute(EmitterSphereAllocate* ptr, const NvFlowEmitterSphereAllocatePinsIn* in, NvFlowEmitterSphereAllocatePinsOut* out)
    {
        const NvFlowSparseParams* in_sparseParams = &in->sparseParams;

        NvFlowUint maxLocations = 0u;
        if (in_sparseParams->levelCount > 0u)
        {
            maxLocations = in_sparseParams->levels[0u].maxLocations;
        }
        ptr->cachedMaxLocations = maxLocations;

        ptr->locationsTmp.size = 0u;
        for (NvFlowUint paramIdx = 0u; paramIdx < in->paramCount; paramIdx++)
        {
            const NvFlowEmitterSphereParams* params = in->params[paramIdx];
            if (!params->enabled)
            {
                continue;
            }
            NvFlowUint layerParamIdx = NvFlowSparseParams_layerToLayerParamIdx(&in->sparseParams, params->layer, params->level);
            if (layerParamIdx == ~0u)
            {
                continue;
            }
            const NvFlowSparseLayerParams* layerParams = &in->sparseParams.layers[layerParamIdx];
            if (layerParams->forceDisableEmitters)
            {
                continue;
            }

            if (params->allocationScale > 0.f && params->radius > 0.f)
            {
                NvFlowFloat3 blockSizeWorld = layerParams->blockSizeWorld;

                NvFlowFloat4x4 localToWorld = EmitterSphere_getLocalToWorld(params);
                NvFlowFloat4x4 oldLocalToWorld = localToWorld;
                if (paramIdx < ptr->oldLocalToWorlds.size)
                {
                    oldLocalToWorld = ptr->oldLocalToWorlds[paramIdx];
                }

                NvFlowInt4 locationMin = {};
                NvFlowInt4 locationMax = {};
                EmitterSphere_computeFootprint(&locationMin, &locationMax, params, layerParams, oldLocalToWorld, localToWorld, true);

                NvFlowUint locationPushCount = 0u;
                for (int k = locationMin.z; k < locationMax.z; k++)
                {
                    for (int j = locationMin.y; j < locationMax.y; j++)
                    {
                        for (int i = locationMin.x; i < locationMax.x; i++)
                        {
                            ptr->locationsTmp.pushBack(NvFlowInt4{ i, j, k, locationMin.w });
                            locationPushCount++;
                            if (locationPushCount >= ptr->cachedMaxLocations)
                            {
                                k = locationMax.z;
                                j = locationMax.y;
                                i = locationMax.x;
                            }
                        }
                    }
                }
            }
        }
        out->locations = ptr->locationsTmp.data;
        out->locationCount = ptr->locationsTmp.size;

        if (in->paramCount > 0u)
        {
            ptr->oldLocalToWorlds.size = 0u;
            for (NvFlowUint paramIdx = 0u; paramIdx < in->paramCount; paramIdx++)
            {
                const NvFlowEmitterSphereParams* params = in->params[paramIdx];
                ptr->oldLocalToWorlds.pushBack(EmitterSphere_getLocalToWorld(params));
            }
        }
    }
}

NV_FLOW_OP_IMPL(NvFlowEmitterSphereAllocate, EmitterSphereAllocate)
