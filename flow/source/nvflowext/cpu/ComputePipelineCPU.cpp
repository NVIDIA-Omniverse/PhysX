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

#include "CommonCPU.h"

namespace NvFlowCPU
{


NvFlowComputePipeline* createComputePipeline(NvFlowContext* contextIn, const NvFlowComputePipelineDesc* desc)
{
    auto context = cast(contextIn);
    auto ptr = new ComputePipeline();

    ptr->desc = *desc;

    ptr->resources.reserve(desc->numBindingDescs);
    ptr->resources.size = desc->numBindingDescs;

    return cast(ptr);
}

void destroyComputePipeline(NvFlowContext* contextIn, NvFlowComputePipeline* pipeline)
{
    auto context = cast(contextIn);
    auto ptr = cast(pipeline);

    delete ptr;
}

typedef void(*computePipeline_mainBlock_t)(void* smemPool, NvFlowCPU_Uint3 groupID, NvFlowCPU_Uint numDescriptorWrites, const NvFlowDescriptorWrite* descriptorWrites, NvFlowCPU_Resource** resources);

struct ComputePipelineTask
{
    computePipeline_mainBlock_t mainBlock;
    const NvFlowPassComputeParams* params;
    NvFlowCPU_Resource** resources;
};

void computePipeline_task(NvFlowUint taskIdx, NvFlowUint threadIdx, void* sharedMem, void* userdata)
{
    ComputePipelineTask* task = (ComputePipelineTask*)userdata;

    NvFlowCPU_Uint3 blockIdx(
        taskIdx % (task->params->gridDim.x),
        (taskIdx / task->params->gridDim.x) % (task->params->gridDim.y),
        taskIdx / (task->params->gridDim.x * task->params->gridDim.y)
    );

    task->mainBlock(
        sharedMem,
        blockIdx,
        task->params->numDescriptorWrites,
        task->params->descriptorWrites,
        task->resources
    );
}

void computePipeline_dispatch(Context* context, const NvFlowPassComputeParams* params)
{
    ComputePipeline* ptr = cast(params->pipeline);

    ptr->resources.reserve(params->numDescriptorWrites);
    ptr->resources.size = params->numDescriptorWrites;
    for (NvFlowUint idx = 0u; idx < params->numDescriptorWrites; idx++)
    {
        auto& descWrite = params->descriptorWrites[idx];
        auto& srcResource = params->resources[idx];
        NvFlowCPU_Resource* dstResource = nullptr;
        if (srcResource.bufferTransient)
        {
            dstResource = &(cast(srcResource.bufferTransient)->buffer->resource);
        }
        if (srcResource.textureTransient)
        {
            dstResource = &(cast(srcResource.textureTransient)->texture->resource);
        }
        if (srcResource.sampler)
        {
            dstResource = &(cast(srcResource.sampler)->resource);
        }
        ptr->resources[idx] = dstResource;
    }

    computePipeline_mainBlock_t mainBlock = (computePipeline_mainBlock_t)ptr->desc.bytecode.data;

    NvFlowUint totalBlocks = params->gridDim.x * params->gridDim.y * params->gridDim.z;

    ComputePipelineTask taskData = {};
    taskData.mainBlock = mainBlock;
    taskData.params = params;
    taskData.resources = ptr->resources.data;

    NvFlowUint targetBatchesPerThread = 32u;
    NvFlowUint threadCount = context->threadPoolInterface->getThreadCount(context->threadPool);
    NvFlowUint aveBlocksPerThread = totalBlocks / threadCount;
    NvFlowUint granularity = aveBlocksPerThread / targetBatchesPerThread;
    if (granularity == 0u)
    {
        granularity = 1u;
    }

    context->threadPoolInterface->execute(context->threadPool, totalBlocks, granularity, computePipeline_task, &taskData);
}

} // end namespace
