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

#include "CommonVulkan.h"

namespace NvFlowVulkan
{

NvFlowUint computePipeline_computeNumSets(NvFlowUint descriptorsPerSet)
{
    static const NvFlowUint targetCount = 4096u; // 65536u;
    NvFlowUint numSets = 1u;
    if (descriptorsPerSet > 0)
    {
        numSets = targetCount / (descriptorsPerSet);
    }
    if (numSets == 0u)
    {
        numSets = 1u;
    }
    return numSets;
}

NvFlowComputePipeline* createComputePipeline(NvFlowContext* contextIn, const NvFlowComputePipelineDesc* desc)
{
    auto context = cast(contextIn);
    auto ptr = new ComputePipeline();

    auto loader = &context->deviceQueue->device->loader;
    auto vulkanDevice = context->deviceQueue->device->vulkanDevice;

    ptr->desc = *desc;

    ptr->flowDescriptorType_to_vkDescriptorType[eNvFlowDescriptorType_constantBuffer] = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    ptr->flowDescriptorType_to_vkDescriptorType[eNvFlowDescriptorType_structuredBuffer] = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    ptr->flowDescriptorType_to_vkDescriptorType[eNvFlowDescriptorType_buffer] = VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER;
    ptr->flowDescriptorType_to_vkDescriptorType[eNvFlowDescriptorType_texture] = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
    ptr->flowDescriptorType_to_vkDescriptorType[eNvFlowDescriptorType_sampler] = VK_DESCRIPTOR_TYPE_SAMPLER;
    ptr->flowDescriptorType_to_vkDescriptorType[eNvFlowDescriptorType_rwStructuredBuffer] = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    ptr->flowDescriptorType_to_vkDescriptorType[eNvFlowDescriptorType_rwBuffer] = VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER;
    ptr->flowDescriptorType_to_vkDescriptorType[eNvFlowDescriptorType_rwTexture] = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;

    ptr->flowDescriptorType_to_vkDescriptorType[eNvFlowDescriptorType_textureSampler] = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;

    // process bytecode
    if (ptr->desc.bytecode.sizeInBytes > 0u)
    {
        VkShaderModuleCreateInfo shaderModuleCreateInfo = {};
        shaderModuleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        shaderModuleCreateInfo.codeSize = ptr->desc.bytecode.sizeInBytes;
        shaderModuleCreateInfo.pCode = (const uint32_t*)ptr->desc.bytecode.data;
        loader->vkCreateShaderModule(vulkanDevice, &shaderModuleCreateInfo, nullptr, &ptr->module);
    }
    else
    {
        ptr->module = nullptr;
    }

    // create descriptor layout
    {
        ptr->bindings.size = 0u;
        for (NvFlowUint idx = 0u; idx < desc->numBindingDescs; idx++)
        {
            auto bindingDesc = &desc->bindingDescs[idx];

            VkDescriptorSetLayoutBinding binding = {};
            binding.binding = bindingDesc->bindingDesc.vulkan.binding;
            binding.descriptorType = ptr->flowDescriptorType_to_vkDescriptorType[bindingDesc->type];
            binding.descriptorCount = bindingDesc->bindingDesc.vulkan.descriptorCount;
            binding.stageFlags = VK_SHADER_STAGE_ALL;
            binding.pImmutableSamplers = nullptr;

            // count descriptors
            ptr->descriptorCounts[bindingDesc->type]++;
            ptr->totalDescriptors += bindingDesc->bindingDesc.vulkan.descriptorCount;

            ptr->bindings.pushBack(binding);
        }

        VkDescriptorSetLayoutCreateInfo descriptorSetLayoutInfo = {};
        descriptorSetLayoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        descriptorSetLayoutInfo.bindingCount = (uint32_t)ptr->bindings.size;
        descriptorSetLayoutInfo.pBindings = ptr->bindings.data;

        loader->vkCreateDescriptorSetLayout(vulkanDevice, &descriptorSetLayoutInfo, nullptr, &ptr->descriptorSetLayout);

        // precompute desriptor count for pool allocation
        {
            ptr->setsPerPool = computePipeline_computeNumSets(ptr->totalDescriptors);

            auto pushPool = [&](NvFlowDescriptorType type)
            {
                if (ptr->descriptorCounts[type] > 0u)
                {
                    VkDescriptorType vkType = ptr->flowDescriptorType_to_vkDescriptorType[type];
                    NvFlowUint descriptorCount = ptr->setsPerPool * ptr->descriptorCounts[type];
                    // check for match
                    NvFlowUint idx = 0u;
                    for (; idx < ptr->poolSizeCount; idx++)
                    {
                        if (ptr->poolSizes[idx].type == vkType)
                        {
                            ptr->poolSizes[idx].descriptorCount += descriptorCount;
                            break;
                        }
                    }
                    if (idx == ptr->poolSizeCount)
                    {
                        ptr->poolSizes[ptr->poolSizeCount].type = vkType;
                        ptr->poolSizes[ptr->poolSizeCount].descriptorCount = descriptorCount;
                        ptr->poolSizeCount++;
                    }
                }
            };

            pushPool(eNvFlowDescriptorType_constantBuffer);
            pushPool(eNvFlowDescriptorType_structuredBuffer);
            pushPool(eNvFlowDescriptorType_buffer);
            pushPool(eNvFlowDescriptorType_texture);
            pushPool(eNvFlowDescriptorType_sampler);
            pushPool(eNvFlowDescriptorType_rwStructuredBuffer);
            pushPool(eNvFlowDescriptorType_rwBuffer);
            pushPool(eNvFlowDescriptorType_rwTexture);
            pushPool(eNvFlowDescriptorType_textureSampler);
        }
    }

    // create pipeline layout
    {
        VkDescriptorSetLayout layouts[1u] = {
            ptr->descriptorSetLayout
        };

        VkPipelineLayoutCreateInfo pipelineLayoutInfo = {};
        pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        pipelineLayoutInfo.setLayoutCount = 1u;
        pipelineLayoutInfo.pSetLayouts = layouts;
        pipelineLayoutInfo.pushConstantRangeCount = 0u;
        pipelineLayoutInfo.pPushConstantRanges = nullptr;

        loader->vkCreatePipelineLayout(vulkanDevice, &pipelineLayoutInfo, nullptr, &ptr->pipelineLayout);
    }

    // create pipeline
    {
        VkPipelineShaderStageCreateInfo stage = {};
        stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stage.pNext = nullptr;
        stage.flags = 0u;
        stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
        stage.pName = "main";
        stage.pSpecializationInfo = nullptr;

        VkComputePipelineCreateInfo createInfo = {};
        createInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;

        stage.module = ptr->module;
        createInfo.stage = stage;
        createInfo.layout = ptr->pipelineLayout;

        loader->vkCreateComputePipelines(
            vulkanDevice,
            VK_NULL_HANDLE,
            1u,
            &createInfo,
            nullptr,
            &ptr->pipeline
        );
    }

    return cast(ptr);
}

void destroyComputePipeline(NvFlowContext* contextIn, NvFlowComputePipeline* pipeline)
{
    auto context = cast(contextIn);
    auto ptr = cast(pipeline);
    auto loader = &context->deviceQueue->device->loader;
    auto vulkanDevice = context->deviceQueue->device->vulkanDevice;

    for (NvFlowUint idx = 0u; idx < ptr->pools.size; idx++)
    {
        if (ptr->pools[idx].pool)
        {
            loader->vkDestroyDescriptorPool(loader->device, ptr->pools[idx].pool, nullptr);
            ptr->pools[idx].pool = VK_NULL_HANDLE;
        }
    }

    loader->vkDestroyDescriptorSetLayout(loader->device, ptr->descriptorSetLayout, nullptr);
    loader->vkDestroyPipelineLayout(loader->device, ptr->pipelineLayout, nullptr);
    loader->vkDestroyPipeline(loader->device, ptr->pipeline, nullptr);

    loader->vkDestroyShaderModule(loader->device, ptr->module, nullptr);

    delete ptr;
}

VkDescriptorSet computePipeline_allocate(Context* context, ComputePipeline* ptr)
{
    auto loader = &context->deviceQueue->device->loader;
    auto vulkanDevice = context->deviceQueue->device->vulkanDevice;

    VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
    // allocate descriptor set
    VkDescriptorSetAllocateInfo descriptorAllocInfo = {};
    descriptorAllocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    descriptorAllocInfo.descriptorPool = VK_NULL_HANDLE;
    descriptorAllocInfo.descriptorSetCount = 1u;
    descriptorAllocInfo.pSetLayouts = &ptr->descriptorSetLayout;

    if (ptr->frontIdx < ptr->pools.size)
    {
        auto& pool = ptr->pools[ptr->frontIdx];
        if (pool.allocSetIdx < ptr->setsPerPool && pool.fenceValue == context->deviceQueue->nextFenceValue)
        {
            pool.allocSetIdx++;
            descriptorAllocInfo.descriptorPool = pool.pool;
            auto ret = loader->vkAllocateDescriptorSets(vulkanDevice, &descriptorAllocInfo, &descriptorSet);
            if (ret < 0)
            {
                descriptorSet = VK_NULL_HANDLE;
            }
        }
    }
    if (descriptorSet == VK_NULL_HANDLE)
    {
        NvFlowUint64 poolIdx = 0u;
        for (; poolIdx < ptr->pools.size; poolIdx++)
        {
            auto& pool = ptr->pools[poolIdx];
            if (pool.fenceValue <= context->deviceQueue->lastFenceCompleted)
            {
                loader->vkResetDescriptorPool(vulkanDevice, pool.pool, 0u);

                pool.allocSetIdx = 0u;
                pool.fenceValue = context->deviceQueue->nextFenceValue;

                pool.allocSetIdx++;
                descriptorAllocInfo.descriptorPool = pool.pool;
                loader->vkAllocateDescriptorSets(vulkanDevice, &descriptorAllocInfo, &descriptorSet);

                break;
            }
        }
        if (poolIdx == ptr->pools.size)
        {
            poolIdx = ptr->pools.allocateBack();

            auto& pool = ptr->pools[poolIdx];

            VkDescriptorPoolCreateInfo descriptorPoolInfo = {};
            descriptorPoolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
            descriptorPoolInfo.flags = 0u;
            descriptorPoolInfo.maxSets = ptr->setsPerPool;
            descriptorPoolInfo.poolSizeCount = ptr->poolSizeCount;
            descriptorPoolInfo.pPoolSizes = ptr->poolSizes;

            loader->vkCreateDescriptorPool(vulkanDevice, &descriptorPoolInfo, nullptr, &pool.pool);

            pool.allocSetIdx = 0u;
            pool.fenceValue = context->deviceQueue->nextFenceValue;

            pool.allocSetIdx++;
            descriptorAllocInfo.descriptorPool = pool.pool;
            loader->vkAllocateDescriptorSets(vulkanDevice, &descriptorAllocInfo, &descriptorSet);
        }
        ptr->frontIdx = poolIdx;
    }
    return descriptorSet;
}

void computePipeline_updateDescriptorSet(Context* context, ComputePipeline* ptr, VkDescriptorSet descriptorSet, const NvFlowPassComputeParams* params)
{
    auto loader = &context->deviceQueue->device->loader;
    auto vulkanDevice = context->deviceQueue->device->vulkanDevice;

    ptr->descriptorWrites.reserve(params->numDescriptorWrites);
    ptr->descriptorWrites.size = 0u;

    ptr->bufferInfos.reserve(params->numDescriptorWrites);
    ptr->bufferViews.reserve(params->numDescriptorWrites);
    ptr->imageInfos.reserve(params->numDescriptorWrites);

    ptr->bufferInfos.size = 0u;
    ptr->bufferViews.size = 0u;
    ptr->imageInfos.size = 0u;
    for (NvFlowUint idx = 0u; idx < params->numDescriptorWrites; idx++)
    {
        auto descriptorWrite = &params->descriptorWrites[idx];
        auto resource = &params->resources[idx];

        VkWriteDescriptorSet output = {};
        output.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        output.pNext = nullptr;
        output.dstSet = descriptorSet;
        output.dstBinding = descriptorWrite->write.vulkan.binding;
        output.dstArrayElement = descriptorWrite->write.vulkan.arrayIndex;
        output.descriptorCount = 1u;
        output.descriptorType = ptr->flowDescriptorType_to_vkDescriptorType[descriptorWrite->type];

        if (descriptorWrite->type == eNvFlowDescriptorType_constantBuffer ||
            descriptorWrite->type == eNvFlowDescriptorType_structuredBuffer ||
            descriptorWrite->type == eNvFlowDescriptorType_rwStructuredBuffer)
        {
            Buffer* buffer = cast(resource->bufferTransient)->buffer;

            VkDescriptorBufferInfo bufferInfo = {};
            bufferInfo.buffer = buffer->bufferVk;
            bufferInfo.offset = 0llu;
            bufferInfo.range = VK_WHOLE_SIZE;

            output.pBufferInfo = ptr->bufferInfos.data + ptr->bufferInfos.size;
            ptr->bufferInfos.pushBack(bufferInfo);
        }
        else if (descriptorWrite->type == eNvFlowDescriptorType_buffer ||
            descriptorWrite->type == eNvFlowDescriptorType_rwBuffer)
        {
            Buffer* buffer = cast(resource->bufferTransient)->buffer;
            VkBufferView bufferView = buffer_getBufferView(
                context,
                buffer,
                cast(resource->bufferTransient)->aliasFormat
            );
            output.pTexelBufferView = ptr->bufferViews.data + ptr->bufferViews.size;
            ptr->bufferViews.pushBack(bufferView);
        }
        else if (descriptorWrite->type == eNvFlowDescriptorType_texture ||
            descriptorWrite->type == eNvFlowDescriptorType_sampler ||
            descriptorWrite->type == eNvFlowDescriptorType_rwTexture ||
            descriptorWrite->type == eNvFlowDescriptorType_textureSampler)
        {
            VkDescriptorImageInfo imageInfo = {};
            if (descriptorWrite->type == eNvFlowDescriptorType_rwTexture)
            {
                imageInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
            }
            else
            {
                imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            }
            if (descriptorWrite->type == eNvFlowDescriptorType_sampler)
            {
                Sampler* sampler = cast(resource->sampler);
                imageInfo.sampler = sampler->sampler;
            }
            else if (descriptorWrite->type == eNvFlowDescriptorType_textureSampler)
            {
                Texture* texture = cast(resource->textureTransient)->texture;
                Sampler* sampler = cast(resource->sampler);
                imageInfo.sampler = sampler->sampler;
                imageInfo.imageView = texture_getImageViewAll(
                    context,
                    texture,
                    cast(resource->textureTransient)->aliasFormat
                );
            }
            else if (descriptorWrite->type == eNvFlowDescriptorType_texture)
            {
                Texture* texture = cast(resource->textureTransient)->texture;
                imageInfo.imageView = texture_getImageViewAll(
                    context,
                    texture,
                    cast(resource->textureTransient)->aliasFormat
                );
            }
            else if (descriptorWrite->type == eNvFlowDescriptorType_rwTexture)
            {
                Texture* texture = cast(resource->textureTransient)->texture;
                imageInfo.imageView = texture_getImageViewMipLevel(
                    context,
                    texture,
                    cast(resource->textureTransient)->aliasFormat
                );
            }

            output.pImageInfo = ptr->imageInfos.data + ptr->imageInfos.size;
            ptr->imageInfos.pushBack(imageInfo);
        }

        ptr->descriptorWrites.pushBack(output);
    }

    loader->vkUpdateDescriptorSets(vulkanDevice, (uint32_t)ptr->descriptorWrites.size, ptr->descriptorWrites.data, 0u, nullptr);

}

void computePipeline_dispatch(Context* context, const NvFlowPassComputeParams* params)
{
    ComputePipeline* ptr = cast(params->pipeline);

    auto loader = &context->deviceQueue->device->loader;
    auto vulkanDevice = context->deviceQueue->device->vulkanDevice;

    NvFlowUint3 gridDim = params->gridDim;

    VkDescriptorSet descriptorSet = computePipeline_allocate(context, ptr);

    computePipeline_updateDescriptorSet(context, ptr, descriptorSet, params);

    loader->vkCmdBindDescriptorSets(context->deviceQueue->commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE,
        ptr->pipelineLayout, 0u, 1u, &descriptorSet, 0u, nullptr);

    loader->vkCmdBindPipeline(context->deviceQueue->commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, ptr->pipeline);

    if (gridDim.x > 65535 || gridDim.y > 65535 || gridDim.z > 65535)
    {
        context->logPrint(eNvFlowLogLevel_warning, "Dispatch gridDim of (%d, %d, %d) exceeds maximum gridDim of 65535.",
            gridDim.x, gridDim.y, gridDim.z);
    }

    if (gridDim.x > 0 && gridDim.y > 0 && gridDim.z > 0)
    {
        loader->vkCmdDispatch(context->deviceQueue->commandBuffer, gridDim.x, gridDim.y, gridDim.z);
    }
}

} // end namespace
