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

#if defined(_WIN32)
// Windows.h already included
#else
#include <time.h>
#endif

namespace NvFlowVulkan
{

Context* context_create(DeviceQueue* deviceQueue)
{
    auto ptr = new Context();

    ptr->logPrint = deviceQueue->device->logPrint;

    ptr->deviceQueue = deviceQueue;

    NvFlowSamplerDesc samplerDesc = {};
    samplerDesc.addressModeU = eNvFlowSamplerAddressMode_border;
    samplerDesc.addressModeV = eNvFlowSamplerAddressMode_border;
    samplerDesc.addressModeW = eNvFlowSamplerAddressMode_border;
    samplerDesc.filterMode = eNvFlowSamplerFilterMode_point;

    createSampler(cast(ptr), &samplerDesc);

    ptr->profiler = profiler_create(ptr);

    NvFlowThreadPoolInterface_duplicate(&ptr->threadPoolInterface, NvFlowGetThreadPoolInterface());
    ptr->threadCount = ptr->threadPoolInterface.getDefaultThreadCount();
    ptr->threadPool = ptr->threadPoolInterface.create(ptr->threadCount, 0llu);

    return ptr;
}

void context_destroy(Context* ptr)
{
    if (ptr->threadPool)
    {
        ptr->threadPoolInterface.destroy(ptr->threadPool);
        ptr->threadPool = nullptr;
    }

    profiler_destroy(ptr, ptr->profiler);

    context_destroyBuffers(ptr);
    context_destroyTextures(ptr);
    context_destroySamplers(ptr);

    delete ptr;
}

void context_resetNodes(Context* context)
{
    context->nodes.size = 0u;
}

void context_resetNode(ContextNode* node)
{
    node->type = eContextNodeType_unknown;
    node->params.memory = ContextNodeMemoryParams{};
    node->descriptorWrites.size = 0u;
    node->resources.size = 0u;

    node->bufferBarriers.size = 0u;
    node->imageBarriers.size = 0u;
}

void addPassCompute(NvFlowContext* contextIn, const NvFlowPassComputeParams* params)
{
    auto context = cast(contextIn);
    ContextNode* node = &context->nodes[context->nodes.allocateBack()];
    context_resetNode(node);

    node->type = eContextNodeType_compute;
    node->params.compute = *params;

    for (NvFlowUint descriptorIdx = 0u; descriptorIdx < params->numDescriptorWrites; descriptorIdx++)
    {
        node->descriptorWrites.pushBack(params->descriptorWrites[descriptorIdx]);
        node->resources.pushBack(params->resources[descriptorIdx]);
    }
    node->params.compute.descriptorWrites = node->descriptorWrites.data;
    node->params.compute.resources = node->resources.data;
}

void addPassCopyBuffer(NvFlowContext* contextIn, const NvFlowPassCopyBufferParams* params)
{
    auto context = cast(contextIn);
    ContextNode* node = &context->nodes[context->nodes.allocateBack()];
    context_resetNode(node);

    node->type = eContextNodeType_copyBuffer;
    node->params.copyBuffer = *params;

    NvFlowResource src = {};
    NvFlowResource dst = {};
    src.bufferTransient = params->src;
    dst.bufferTransient = params->dst;

    node->descriptorWrites.pushBack(NvFlowDescriptorWrite{ eNvFlowDescriptorType_bufferCopySrc });
    node->resources.pushBack(src);
    node->descriptorWrites.pushBack(NvFlowDescriptorWrite{ eNvFlowDescriptorType_bufferCopyDst });
    node->resources.pushBack(dst);
}

void addPassCopyBufferToTexture(NvFlowContext* contextIn, const NvFlowPassCopyBufferToTextureParams* params)
{
    auto context = cast(contextIn);
    ContextNode* node = &context->nodes[context->nodes.allocateBack()];
    context_resetNode(node);

    node->type = eContextNodeType_copyBufferToTexture;
    node->params.copyBufferToTexture = *params;

    NvFlowResource src = {};
    NvFlowResource dst = {};
    src.bufferTransient = params->src;
    dst.textureTransient = params->dst;

    node->descriptorWrites.pushBack(NvFlowDescriptorWrite{ eNvFlowDescriptorType_bufferCopySrc });
    node->resources.pushBack(src);
    node->descriptorWrites.pushBack(NvFlowDescriptorWrite{ eNvFlowDescriptorType_textureCopyDst });
    node->resources.pushBack(dst);
}

void addPassCopyTextureToBuffer(NvFlowContext* contextIn, const NvFlowPassCopyTextureToBufferParams* params)
{
    auto context = cast(contextIn);
    ContextNode* node = &context->nodes[context->nodes.allocateBack()];
    context_resetNode(node);

    node->type = eContextNodeType_copyTextureToBuffer;
    node->params.copyTextureToBuffer = *params;

    NvFlowResource src = {};
    NvFlowResource dst = {};
    src.textureTransient = params->src;
    dst.bufferTransient = params->dst;

    node->descriptorWrites.pushBack(NvFlowDescriptorWrite{ eNvFlowDescriptorType_textureCopySrc });
    node->resources.pushBack(src);
    node->descriptorWrites.pushBack(NvFlowDescriptorWrite{ eNvFlowDescriptorType_bufferCopyDst });
    node->resources.pushBack(dst);
}

void addPassCopyTexture(NvFlowContext* contextIn, const NvFlowPassCopyTextureParams* params)
{
    auto context = cast(contextIn);
    ContextNode* node = &context->nodes[context->nodes.allocateBack()];
    context_resetNode(node);

    node->type = eContextNodeType_copyTexture;
    node->params.copyTexture = *params;

    NvFlowResource src = {};
    NvFlowResource dst = {};
    src.textureTransient = params->src;
    dst.textureTransient = params->dst;

    node->descriptorWrites.pushBack(NvFlowDescriptorWrite{ eNvFlowDescriptorType_textureCopySrc });
    node->resources.pushBack(src);
    node->descriptorWrites.pushBack(NvFlowDescriptorWrite{ eNvFlowDescriptorType_textureCopyDst });
    node->resources.pushBack(dst);
}

void context_flushNodes(Context* context)
{
    auto loader = &context->deviceQueue->device->loader;
    auto vulkanDevice = context->deviceQueue->device->vulkanDevice;

    // reset lifetimes
    for (NvFlowUint idx = 0u; idx < context->bufferTransients.size; idx++)
    {
        context->bufferTransients[idx]->nodeBegin = int(context->nodes.size);
        context->bufferTransients[idx]->nodeEnd = -1;
    }
    for (NvFlowUint idx = 0u; idx < context->textureTransients.size; idx++)
    {
        context->textureTransients[idx]->nodeBegin = int(context->nodes.size);
        context->textureTransients[idx]->nodeEnd = -1;
    }

    // already allocated resources begin life at -1, and if active live the whole frame
    for (NvFlowUint idx = 0u; idx < context->bufferTransients.size; idx++)
    {
        auto transient = context->bufferTransients[idx];
        if (transient->buffer)
        {
            transient->nodeBegin = -1;
            if (transient->buffer->refCount > 0)
            {
                transient->nodeEnd = int(context->nodes.size);
            }
        }
    }
    for (NvFlowUint idx = 0u; idx < context->textureTransients.size; idx++)
    {
        auto transient = context->textureTransients[idx];
        if (transient->texture)
        {
            transient->nodeBegin = -1;
            if (transient->texture->refCount > 0)
            {
                transient->nodeEnd = int(context->nodes.size);
            }
        }
    }
    // compute transient lifetimes
    for (NvFlowUint nodeIdx = 0u; nodeIdx < context->nodes.size; nodeIdx++)
    {
        ContextNode* node = &context->nodes[nodeIdx];
        for (NvFlowUint descriptorIdx = 0u; descriptorIdx < node->descriptorWrites.size; descriptorIdx++)
        {
            NvFlowResource* resource = &node->resources[descriptorIdx];
            if (resource->bufferTransient)
            {
                BufferTransient* bufferTransient = cast(resource->bufferTransient);
                if (int(nodeIdx) < bufferTransient->nodeBegin)
                {
                    bufferTransient->nodeBegin = int(nodeIdx);
                }
                if (int(nodeIdx) > bufferTransient->nodeEnd)
                {
                    bufferTransient->nodeEnd = int(nodeIdx);
                }
            }
            if (resource->textureTransient)
            {
                TextureTransient* textureTransient = cast(resource->textureTransient);
                if (int(nodeIdx) < textureTransient->nodeBegin)
                {
                    textureTransient->nodeBegin = int(nodeIdx);
                }
                if (int(nodeIdx) > textureTransient->nodeEnd)
                {
                    textureTransient->nodeEnd = int(nodeIdx);
                }
            }
        }
    }
    // extend lifetime for acquired
    for (NvFlowUint idx = 0u; idx < context->bufferAcquires.size; idx++)
    {
        if (context->bufferAcquires[idx]->bufferTransient)
        {
            context->bufferAcquires[idx]->bufferTransient->nodeEnd = int(context->nodes.size);
        }
    }
    for (NvFlowUint idx = 0u; idx < context->textureAcquires.size; idx++)
    {
        if (context->textureAcquires[idx]->textureTransient)
        {
            context->textureAcquires[idx]->textureTransient->nodeEnd = int(context->nodes.size);
        }
    }

    // reset per node transient arrays
    for (NvFlowUint nodeIdx = 0u; nodeIdx < context->nodes.size; nodeIdx++)
    {
        ContextNode* node = &context->nodes[nodeIdx];
        node->bufferTransientsCreate.size = 0u;
        node->textureTransientsCreate.size = 0u;
        node->bufferTransientsDestroy.size = 0u;
        node->textureTransientsDestroy.size = 0u;
    }
    // scatter transients to per node arrays
    for (NvFlowUint idx = 0u; idx < context->bufferTransients.size; idx++)
    {
        auto transient = context->bufferTransients[idx];
        if (transient->nodeBegin >= 0 && transient->nodeBegin < int(context->nodes.size))
        {
            context->nodes[transient->nodeBegin].bufferTransientsCreate.pushBack(transient);
        }
        if (transient->nodeEnd >= 0 && transient->nodeEnd < int(context->nodes.size))
        {
            context->nodes[transient->nodeEnd].bufferTransientsDestroy.pushBack(transient);
        }
    }
    for (NvFlowUint idx = 0u; idx < context->textureTransients.size; idx++)
    {
        auto transient = context->textureTransients[idx];
        if (transient->nodeBegin >= 0 && transient->nodeBegin < int(context->nodes.size))
        {
            context->nodes[transient->nodeBegin].textureTransientsCreate.pushBack(transient);
        }
        if (transient->nodeEnd >= 0 && transient->nodeEnd < int(context->nodes.size))
        {
            context->nodes[transient->nodeEnd].textureTransientsDestroy.pushBack(transient);
        }
    }

    // for node -1, revive already allocated resources if referenced in a different node
    for (NvFlowUint idx = 0u; idx < context->bufferTransients.size; idx++)
    {
        auto transient = context->bufferTransients[idx];
        if (transient->buffer && transient->buffer->refCount == 0 && transient->nodeEnd != -1)
        {
            transient->buffer->refCount = 1;
            transient->buffer->lastActive = context->deviceQueue->nextFenceValue;
        }
    }
    for (NvFlowUint idx = 0u; idx < context->textureTransients.size; idx++)
    {
        auto transient = context->textureTransients[idx];
        if (transient->texture && transient->texture->refCount == 0 && transient->nodeEnd != -1)
        {
            transient->texture->refCount = 1;
            transient->texture->lastActive = context->deviceQueue->nextFenceValue;
        }
    }
    // resolve transient resources
    for (NvFlowUint nodeIdx = 0u; nodeIdx < context->nodes.size; nodeIdx++)
    {
        ContextNode* node = &context->nodes[nodeIdx];

        for (NvFlowUint idx = 0u; idx < node->bufferTransientsCreate.size; idx++)
        {
            auto transient = node->bufferTransientsCreate[idx];
            if (transient->aliasBuffer)
            {
                transient->buffer = transient->aliasBuffer->buffer;
                transient->buffer->refCount++;
            }
            else
            {
                if (transient->buffer)
                {
                    context->logPrint(eNvFlowLogLevel_error, "NvFlowContext::BufferTransient double create");
                }
                transient->buffer = cast(createBuffer(cast(context), eNvFlowMemoryType_device, &transient->desc));
            }
        }
        for (NvFlowUint idx = 0u; idx < node->textureTransientsCreate.size; idx++)
        {
            auto transient = node->textureTransientsCreate[idx];
            if (transient->aliasTexture)
            {
                transient->texture = transient->aliasTexture->texture;
                transient->texture->refCount++;
            }
            else
            {
                if (transient->texture)
                {
                    context->logPrint(eNvFlowLogLevel_error, "NvFlowContext::TextureTransient double create");
                }
                transient->texture = cast(createTexture(cast(context), &transient->desc));
            }
        }

        for (NvFlowUint idx = 0u; idx < node->bufferTransientsDestroy.size; idx++)
        {
            auto transient = node->bufferTransientsDestroy[idx];
            destroyBuffer(cast(context), cast(transient->buffer));
        }
        for (NvFlowUint idx = 0u; idx < node->textureTransientsDestroy.size; idx++)
        {
            auto transient = node->textureTransientsDestroy[idx];
            destroyTexture(cast(context), cast(transient->texture));
        }
    }
    // for the final node, allocate resources needed for capture
    for (NvFlowUint idx = 0u; idx < context->bufferTransients.size; idx++)
    {
        auto transient = context->bufferTransients[idx];
        if (transient->nodeBegin == int(context->nodes.size) && transient->nodeEnd == int(context->nodes.size))
        {
            transient->buffer = cast(createBuffer(cast(context), eNvFlowMemoryType_device, &transient->desc));
        }
    }
    for (NvFlowUint idx = 0u; idx < context->textureTransients.size; idx++)
    {
        auto transient = context->textureTransients[idx];
        if (transient->nodeBegin == int(context->nodes.size) && transient->nodeEnd == int(context->nodes.size))
        {
            transient->texture = cast(createTexture(cast(context), &transient->desc));
        }
    }

    // precompute barriers
    for (NvFlowUint nodeIdx = 0u; nodeIdx < context->nodes.size; nodeIdx++)
    {
        ContextNode* node = &context->nodes[nodeIdx];
        for (NvFlowUint descriptorIdx = 0u; descriptorIdx < node->descriptorWrites.size; descriptorIdx++)
        {
            NvFlowDescriptorWrite* descriptorWrite = &node->descriptorWrites[descriptorIdx];
            NvFlowResource* resource = &node->resources[descriptorIdx];

            if (resource->bufferTransient)
            {
                Buffer* buffer = cast(resource->bufferTransient)->buffer;

                VkBufferMemoryBarrier bufferBarrier = buffer->currentBarrier;

                // new becomes old
                bufferBarrier.srcAccessMask = bufferBarrier.dstAccessMask;
                bufferBarrier.srcQueueFamilyIndex = bufferBarrier.dstQueueFamilyIndex;

                // establish new
                if (descriptorWrite->type == eNvFlowDescriptorType_constantBuffer ||
                    descriptorWrite->type == eNvFlowDescriptorType_structuredBuffer ||
                    descriptorWrite->type == eNvFlowDescriptorType_buffer)
                {
                    bufferBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
                }
                else if (descriptorWrite->type == eNvFlowDescriptorType_rwStructuredBuffer ||
                         descriptorWrite->type == eNvFlowDescriptorType_rwBuffer)
                {
                    bufferBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
                }
                else if (descriptorWrite->type == eNvFlowDescriptorType_indirectBuffer)
                {
                    bufferBarrier.dstAccessMask = VK_ACCESS_INDIRECT_COMMAND_READ_BIT;
                }
                else if (descriptorWrite->type == eNvFlowDescriptorType_bufferCopySrc)
                {
                    bufferBarrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
                }
                else if (descriptorWrite->type == eNvFlowDescriptorType_bufferCopyDst)
                {
                    bufferBarrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
                }
                bufferBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

                // capture barrier
                node->bufferBarriers.pushBack(bufferBarrier);

                buffer->currentBarrier = bufferBarrier;
            }
            if (resource->textureTransient)
            {
                Texture* texture = cast(resource->textureTransient)->texture;

                VkImageMemoryBarrier imageBarrier = texture->currentBarrier;

                // new becomes old
                imageBarrier.srcAccessMask = imageBarrier.dstAccessMask;
                imageBarrier.oldLayout = imageBarrier.newLayout;
                imageBarrier.srcQueueFamilyIndex = imageBarrier.dstQueueFamilyIndex;

                // establish new
                if (descriptorWrite->type == eNvFlowDescriptorType_texture || descriptorWrite->type == eNvFlowDescriptorType_textureSampler)
                {
                    imageBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
                    imageBarrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                }
                else if (descriptorWrite->type == eNvFlowDescriptorType_rwTexture)
                {
                    imageBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
                    imageBarrier.newLayout = VK_IMAGE_LAYOUT_GENERAL;
                }
                else if (descriptorWrite->type == eNvFlowDescriptorType_textureCopySrc)
                {
                    imageBarrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
                    imageBarrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
                }
                else if (descriptorWrite->type == eNvFlowDescriptorType_textureCopyDst)
                {
                    imageBarrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
                    imageBarrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
                }
                imageBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

                // commit barrier, except if multiple reads detected
                NvFlowBool32 shouldCommit = NV_FLOW_TRUE;
                if (imageBarrier.newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL &&
                    imageBarrier.newLayout == imageBarrier.oldLayout &&
                    imageBarrier.dstAccessMask == imageBarrier.srcAccessMask)
                {
                    shouldCommit = NV_FLOW_FALSE;
                }
                if (shouldCommit)
                {
                    node->imageBarriers.pushBack(imageBarrier);

                    texture->currentBarrier = imageBarrier;
                }
            }
        }
    }

    profiler_beginCapture(context, context->profiler, context->nodes.size);

    // execute nodes
    for (NvFlowUint nodeIdx = 0u; nodeIdx < context->nodes.size; nodeIdx++)
    {
        ContextNode* node = &context->nodes[nodeIdx];

        if (node->bufferBarriers.size > 0u ||
            node->imageBarriers.size > 0u)
        {
            loader->vkCmdPipelineBarrier(
                context->deviceQueue->commandBuffer,
                VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
                VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
                0,
                0u, nullptr,
                (uint32_t)node->bufferBarriers.size, node->bufferBarriers.data,
                (uint32_t)node->imageBarriers.size, node->imageBarriers.data
            );
        }

        if (node->type == eContextNodeType_compute)
        {
            const auto& params = node->params.compute;
            node->label = params.debugLabel;

            computePipeline_dispatch(context, &params);
        }
        else if (node->type == eContextNodeType_copyBuffer)
        {
            const auto& params = node->params.copyBuffer;
            node->label = params.debugLabel;

            VkBufferCopy region = {};
            region.srcOffset = params.srcOffset;
            region.dstOffset = params.dstOffset;
            region.size = params.numBytes;

            loader->vkCmdCopyBuffer(
                context->deviceQueue->commandBuffer,
                cast(params.src)->buffer->bufferVk,
                cast(params.dst)->buffer->bufferVk,
                1u, &region
            );
        }
        else if (node->type == eContextNodeType_copyBufferToTexture)
        {
            const auto& params = node->params.copyBufferToTexture;
            node->label = params.debugLabel;

            auto buffer = cast(params.src)->buffer;
            auto texture = cast(params.dst)->texture;
            NvFlowUint formatSize = formatConverter_getFormatSizeInBytes(
                context->deviceQueue->device->formatConverter,
                texture->desc.format);

            VkBufferImageCopy region = {};
            region.bufferOffset = params.bufferOffset;
            region.bufferRowLength = params.bufferRowPitch / formatSize;
            region.bufferImageHeight = params.bufferDepthPitch / params.bufferRowPitch;
            region.imageSubresource.aspectMask = texture->imageAspect;
            region.imageSubresource.mipLevel = params.textureMipLevel;
            region.imageSubresource.baseArrayLayer = 0u;
            region.imageSubresource.layerCount = 1u;
            region.imageOffset.x = params.textureOffset.x;
            region.imageOffset.y = params.textureOffset.y;
            region.imageOffset.z = params.textureOffset.z;
            region.imageExtent.width = params.textureExtent.x;
            region.imageExtent.height = params.textureExtent.y;
            region.imageExtent.depth = params.textureExtent.z;

            loader->vkCmdCopyBufferToImage(
                context->deviceQueue->commandBuffer,
                buffer->bufferVk,
                texture->imageVk,
                VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                1u, &region
            );
        }
        else if (node->type == eContextNodeType_copyTextureToBuffer)
        {
            const auto& params = node->params.copyTextureToBuffer;
            node->label = params.debugLabel;

            auto buffer = cast(params.dst)->buffer;
            auto texture = cast(params.src)->texture;
            NvFlowUint formatSize = formatConverter_getFormatSizeInBytes(
                context->deviceQueue->device->formatConverter,
                texture->desc.format);

            VkBufferImageCopy region = {};
            region.bufferOffset = params.bufferOffset;
            region.bufferRowLength = params.bufferRowPitch / formatSize;
            region.bufferImageHeight = params.bufferDepthPitch / params.bufferRowPitch;
            region.imageSubresource.aspectMask = texture->imageAspect;
            region.imageSubresource.mipLevel = params.textureMipLevel;
            region.imageSubresource.baseArrayLayer = 0u;
            region.imageSubresource.layerCount = 1u;
            region.imageOffset.x = params.textureOffset.x;
            region.imageOffset.y = params.textureOffset.y;
            region.imageOffset.z = params.textureOffset.z;
            region.imageExtent.width = params.textureExtent.x;
            region.imageExtent.height = params.textureExtent.y;
            region.imageExtent.depth = params.textureExtent.z;

            loader->vkCmdCopyImageToBuffer(
                context->deviceQueue->commandBuffer,
                texture->imageVk,
                VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                buffer->bufferVk,
                1u, &region
            );
        }
        else if (node->type == eContextNodeType_copyTexture)
        {
            const auto& params = node->params.copyTexture;
            node->label = params.debugLabel;

            auto src = cast(params.src)->texture;
            auto dst = cast(params.dst)->texture;

            VkImageCopy region = {};
            region.srcSubresource.aspectMask = src->imageAspect;
            region.srcSubresource.mipLevel = params.srcMipLevel;
            region.srcSubresource.baseArrayLayer = 0u;
            region.srcSubresource.layerCount = 1u;
            region.srcOffset.x = params.srcOffset.x;
            region.srcOffset.y = params.srcOffset.y;
            region.srcOffset.z = params.srcOffset.z;
            region.dstSubresource.aspectMask = dst->imageAspect;
            region.dstSubresource.mipLevel = params.dstMipLevel;
            region.dstSubresource.baseArrayLayer = 0u;
            region.dstSubresource.layerCount = 1u;
            region.dstOffset.x = params.dstOffset.x;
            region.dstOffset.y = params.dstOffset.y;
            region.dstOffset.z = params.dstOffset.z;
            region.extent.width = params.extent.x;
            region.extent.height = params.extent.y;
            region.extent.depth = params.extent.z;

            loader->vkCmdCopyImage(
                context->deviceQueue->commandBuffer,
                src->imageVk, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                dst->imageVk, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                1u, &region
            );
        }

        profiler_timestamp(context, context->profiler, node->label);
    }

    profiler_endCapture(context, context->profiler);

    profiler_processCaptures(context, context->profiler);

    // restore resource states
    context->restore_bufferBarriers.size = 0u;
    context->restore_imageBarriers.size = 0u;
    for (NvFlowUint idx = 0u; idx < context->bufferTransients.size; idx++)
    {
        auto transient = context->bufferTransients[idx];
        if (transient->buffer)
        {
            auto buffer = transient->buffer;

            VkBufferMemoryBarrier bufferBarrier = buffer->currentBarrier;

            // new becomes old
            bufferBarrier.srcAccessMask = bufferBarrier.dstAccessMask;
            bufferBarrier.srcQueueFamilyIndex = bufferBarrier.dstQueueFamilyIndex;

            // restore state
            bufferBarrier.dstAccessMask = buffer->restoreBarrier.dstAccessMask;
            bufferBarrier.dstQueueFamilyIndex = buffer->restoreBarrier.dstQueueFamilyIndex;

            // capture
            context->restore_bufferBarriers.pushBack(bufferBarrier);

            buffer->currentBarrier = bufferBarrier;
        }
    }
    for (NvFlowUint idx = 0u; idx < context->textureTransients.size; idx++)
    {
        auto transient = context->textureTransients[idx];
        if (transient->texture)
        {
            auto texture = transient->texture;

            VkImageMemoryBarrier imageBarrier = texture->currentBarrier;

            // new becomes old
            imageBarrier.srcAccessMask = imageBarrier.dstAccessMask;
            imageBarrier.oldLayout = imageBarrier.newLayout;
            imageBarrier.srcQueueFamilyIndex = imageBarrier.dstQueueFamilyIndex;

            // restore state
            imageBarrier.dstAccessMask = texture->restoreBarrier.dstAccessMask;
            imageBarrier.newLayout = texture->restoreBarrier.newLayout;
            imageBarrier.dstQueueFamilyIndex = texture->restoreBarrier.dstQueueFamilyIndex;

            // commit barrier, except if multiple reads detected
            NvFlowBool32 shouldCommit = NV_FLOW_TRUE;
            if (imageBarrier.newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL &&
                imageBarrier.newLayout == imageBarrier.oldLayout &&
                imageBarrier.dstAccessMask == imageBarrier.srcAccessMask)
            {
                shouldCommit = NV_FLOW_FALSE;
            }
            if (shouldCommit)
            {
                context->restore_imageBarriers.pushBack(imageBarrier);

                texture->currentBarrier = imageBarrier;
            }
        }
    }
    if (context->restore_bufferBarriers.size > 0u ||
        context->restore_imageBarriers.size > 0u )
    {
        loader->vkCmdPipelineBarrier(
            context->deviceQueue->commandBuffer,
            VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
            VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
            0,
            0u, nullptr,
            (uint32_t)context->restore_bufferBarriers.size, context->restore_bufferBarriers.data,
            (uint32_t)context->restore_imageBarriers.size, context->restore_imageBarriers.data
        );
    }

    // global barrier
    {
        VkMemoryBarrier barriers[3u] = {
            { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_MEMORY_WRITE_BIT, VK_ACCESS_MEMORY_READ_BIT},
            { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_MEMORY_READ_BIT, VK_ACCESS_MEMORY_WRITE_BIT},
            { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_MEMORY_WRITE_BIT, VK_ACCESS_MEMORY_WRITE_BIT},
        };

        loader->vkCmdPipelineBarrier(
            context->deviceQueue->commandBuffer,
            VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
            VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
            0,
            3u, barriers,
            0u, nullptr,
            0u, nullptr
        );
    }

    // process buffer acquires
    {
        for (NvFlowUint idx = 0u; idx < context->bufferAcquires.size; idx++)
        {
            auto bufferAcquire = context->bufferAcquires[idx];
            if (!bufferAcquire->buffer)
            {
                bufferAcquire->buffer = bufferAcquire->bufferTransient->buffer;
                bufferAcquire->bufferTransient = nullptr;
                if (bufferAcquire->buffer)
                {
                    bufferAcquire->buffer->refCount++;
                }
            }
        }
    }
    // process texture acquires
    {
        for (NvFlowUint idx = 0u; idx < context->textureAcquires.size; idx++)
        {
            auto textureAcquire = context->textureAcquires[idx];
            if (!textureAcquire->texture)
            {
                textureAcquire->texture = textureAcquire->textureTransient->texture;
                textureAcquire->textureTransient = nullptr;
                if (textureAcquire->texture)
                {
                    textureAcquire->texture->refCount++;
                }
            }
        }
    }

    // reset transient arrays
    context->bufferTransients.size = 0u;
    context->textureTransients.size = 0u;

    // clean up unused resources
    for (NvFlowUint idx = 0u; idx < context->pool_buffers.size; idx++)
    {
        auto& ptr = context->pool_buffers[idx];
        if (ptr->refCount == 0 && (ptr->lastActive + context->minLifetime) <= context->deviceQueue->lastFenceCompleted)
        {
            buffer_destroy(context, ptr);
            ptr = nullptr;

            context->logPrint(eNvFlowLogLevel_info, "Vulkan destroy pool buffer %d", idx);

            context->pool_buffers.removeSwapPointerAtIndex(idx);
            idx--;
        }
    }
    for (NvFlowUint idx = 0u; idx < context->pool_textures.size; idx++)
    {
        auto& ptr = context->pool_textures[idx];
        if (ptr->refCount == 0 && (ptr->lastActive + context->minLifetime) <= context->deviceQueue->lastFenceCompleted)
        {
            texture_destroy(context, ptr);
            ptr = nullptr;

            context->logPrint(eNvFlowLogLevel_info, "Vulkan destroy pool texture %d", idx);

            context->pool_textures.removeSwapPointerAtIndex(idx);
            idx--;
        }
    }
}

/// ***************************** TimerHeap *****************************************************

void profilerCapture_init(Context* context, ProfilerCapture* ptr, NvFlowUint64 capacity)
{
    auto loader = &context->deviceQueue->device->loader;
    auto vulkanDevice = context->deviceQueue->device->vulkanDevice;

    VkQueryPoolCreateInfo queryPoolInfo = {};
    queryPoolInfo.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
    queryPoolInfo.queryType = VK_QUERY_TYPE_TIMESTAMP;
    queryPoolInfo.queryCount = (uint32_t)capacity;

    loader->vkCreateQueryPool(vulkanDevice, &queryPoolInfo, nullptr, &ptr->queryPool);

    VkBufferCreateInfo bufCreateInfo = {};
    bufCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufCreateInfo.size = capacity * sizeof(NvFlowUint64);
    bufCreateInfo.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    bufCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    loader->vkCreateBuffer(vulkanDevice, &bufCreateInfo, nullptr, &ptr->queryBuffer);

    VkMemoryRequirements bufMemReq = {};
    loader->vkGetBufferMemoryRequirements(vulkanDevice, ptr->queryBuffer, &bufMemReq);

    uint32_t bufMemType = context_getMemoryType(context, bufMemReq.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT | VK_MEMORY_PROPERTY_HOST_CACHED_BIT);
    if (bufMemType == ~0u)
    {
        bufMemType = context_getMemoryType(context, bufMemReq.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    }

    VkMemoryAllocateInfo bufMemAllocInfo = {};
    bufMemAllocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    bufMemAllocInfo.allocationSize = bufMemReq.size;
    bufMemAllocInfo.memoryTypeIndex = bufMemType;

    loader->vkAllocateMemory(vulkanDevice, &bufMemAllocInfo, nullptr, &ptr->queryMemory);

    loader->vkBindBufferMemory(vulkanDevice, ptr->queryBuffer, ptr->queryMemory, 0u);

    loader->vkMapMemory(vulkanDevice, ptr->queryMemory, 0u, VK_WHOLE_SIZE, 0u, (void**)&ptr->queryMapped);

    ptr->capacity = capacity;

    ptr->entries.reserve(capacity);
    ptr->entries.size = 0u;
}

void profilerCapture_destroy(Context* context, ProfilerCapture* ptr)
{
    auto loader = &context->deviceQueue->device->loader;
    auto vulkanDevice = context->deviceQueue->device->vulkanDevice;

    if (ptr->queryPool)
    {
        loader->vkDestroyQueryPool(loader->device, ptr->queryPool, nullptr);
        loader->vkDestroyBuffer(loader->device, ptr->queryBuffer, nullptr);
        loader->vkFreeMemory(loader->device, ptr->queryMemory, nullptr);
    }

    ptr->queryPool = nullptr;
    ptr->queryBuffer = nullptr;
    ptr->queryMemory = nullptr;
    ptr->queryMapped = nullptr;
    ptr->queryFrequency = 0u;
    ptr->queryReadbackFenceVal = ~0llu;

    ptr->state = 0u;
    ptr->captureID = 0llu;
    ptr->cpuFreq = 0llu;
    ptr->capacity = 0u;

    ptr->entries.size = 0u;
}

void profilerCapture_reset(Context* context, ProfilerCapture* ptr, NvFlowUint64 minCapacity, NvFlowUint64 captureID)
{
    if (ptr->state == 0u)
    {
        auto loader = &context->deviceQueue->device->loader;

        if (ptr->capacity == 0u || ptr->capacity < minCapacity)
        {
            profilerCapture_destroy(context, ptr);

            NvFlowUint64 newCapacity = 128u;
            while (newCapacity < minCapacity)
            {
                newCapacity *= 2u;
            }

            profilerCapture_init(context, ptr, newCapacity);
        }

        ptr->queryFrequency = (NvFlowUint64)(double(1.0E9) / double(context->deviceQueue->device->physicalDeviceProperties.limits.timestampPeriod));
        loader->vkCmdResetQueryPool(context->deviceQueue->commandBuffer, ptr->queryPool, 0u, (uint32_t)ptr->capacity);

#if defined(_WIN32)
        LARGE_INTEGER tmpCpuFreq = {};
        QueryPerformanceFrequency(&tmpCpuFreq);
        ptr->cpuFreq = tmpCpuFreq.QuadPart;
#else
        ptr->cpuFreq = 1E9;
#endif
        ptr->entries.size = 0u;

        ptr->captureID = captureID;
        ptr->state = 1u;
    }
}

void profilerCapture_timestamp(Context* context, ProfilerCapture* ptr, const char* label)
{
    if (ptr->state == 1u && ptr->entries.size < ptr->capacity)
    {
        auto loader = &context->deviceQueue->device->loader;

        NvFlowUint64 entryIdx = ptr->entries.allocateBack();

        auto& entry = ptr->entries[entryIdx];

        entry.label = label;
        entry.cpuValue = 0llu;
        entry.gpuValue = 0llu;

        loader->vkCmdWriteTimestamp(context->deviceQueue->commandBuffer, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, ptr->queryPool, (uint32_t)entryIdx);

#if defined(_WIN32)
        LARGE_INTEGER tmpCpuTime = {};
        QueryPerformanceCounter(&tmpCpuTime);
        entry.cpuValue = tmpCpuTime.QuadPart;
#else
        timespec timeValue = {};
        clock_gettime(CLOCK_MONOTONIC, &timeValue);
        entry.cpuValue = 1E9 * NvFlowUint64(timeValue.tv_sec) + NvFlowUint64(timeValue.tv_nsec);
#endif
    }
}

void profilerCapture_download(Context* context, ProfilerCapture* ptr)
{
    if (ptr->state == 1u)
    {
        auto loader = &context->deviceQueue->device->loader;

        loader->vkCmdCopyQueryPoolResults(context->deviceQueue->commandBuffer, ptr->queryPool, 0u, (uint32_t)ptr->entries.size,
            ptr->queryBuffer, 0llu, sizeof(NvFlowUint64), VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT);

        ptr->queryReadbackFenceVal = context->deviceQueue->nextFenceValue;

        ptr->state = 2u;
    }
}

NvFlowBool32 profilerCapture_mapResults(Context* context, ProfilerCapture* ptr, NvFlowUint64* pNumEntries, NvFlowProfilerEntry** pEntries)
{
    if (ptr->state == 2u)
    {
        if (ptr->queryReadbackFenceVal > context->deviceQueue->lastFenceCompleted)
        {
            return NV_FLOW_FALSE;
        }

        for (NvFlowUint idx = 0u; idx < ptr->entries.size; idx++)
        {
            ptr->entries[idx].gpuValue = ptr->queryMapped[idx];
        }

        ptr->deltaEntries.reserve(ptr->entries.size);
        ptr->deltaEntries.size = ptr->entries.size;

        // compute time deltas
        ProfilerEntry prevEntry = {};
        if (ptr->entries.size > 0u)
        {
            prevEntry = ptr->entries[0u];
        }
        for (NvFlowUint idx = 0u; idx < ptr->entries.size; idx++)
        {
            auto entry = &ptr->entries[idx];
            auto deltaEntry = &ptr->deltaEntries[idx];

            deltaEntry->label = entry->label;
            deltaEntry->cpuDeltaTime = (float)(((double)(entry->cpuValue - prevEntry.cpuValue) / (double)(ptr->cpuFreq)));
            deltaEntry->gpuDeltaTime = (float)(((double)(entry->gpuValue - prevEntry.gpuValue) / (double)(ptr->queryFrequency)));

            prevEntry = *entry;
        }

        // map results
        if (pNumEntries)
        {
            *pNumEntries = ptr->entries.size;
        }
        if (pEntries)
        {
            *pEntries = ptr->deltaEntries.data;
        }

        ptr->state = 3u;
        return NV_FLOW_TRUE;
    }
    return NV_FLOW_FALSE;
}

void profilerCapture_unmapResults(Context* context, ProfilerCapture* ptr)
{
    if (ptr->state == 3u)
    {
        ptr->state = 0u;
    }
}

/// ***************************** Profiler *****************************************************

Profiler* profiler_create(Context* context)
{
    auto ptr = new Profiler();

    return ptr;
}

void profiler_destroy(Context* context, Profiler* ptr)
{
    for (NvFlowUint captureIndex = 0u; captureIndex < ptr->captures.size; captureIndex++)
    {
        profilerCapture_destroy(context, &ptr->captures[captureIndex]);
    }
    ptr->captures.size = 0u;

    delete ptr;
}

void profiler_beginCapture(Context* context, Profiler* ptr, NvFlowUint64 numEntries)
{
    if (!ptr->reportEntries)
    {
        return;
    }

    // account for implicit begin/end
    numEntries += 2u;

    NvFlowUint64 captureIndex = 0u;
    for (; captureIndex < ptr->captures.size; captureIndex++)
    {
        if (ptr->captures[captureIndex].state == 0u)
        {
            break;
        }
    }
    if (captureIndex == ptr->captures.size)
    {
        captureIndex = ptr->captures.allocateBack();

        profilerCapture_init(context, &ptr->captures[captureIndex], numEntries);
    }
    ptr->currentCaptureIndex = captureIndex;

    auto capture = &ptr->captures[captureIndex];

    ptr->currentCaptureID++;

    profilerCapture_reset(context, capture, numEntries, ptr->currentCaptureID);

    profilerCapture_timestamp(context, capture, "BeginCapture");
}

void profiler_endCapture(Context* context, Profiler* ptr)
{
    if (!ptr->reportEntries)
    {
        return;
    }

    if (ptr->currentCaptureIndex < ptr->captures.size)
    {
        auto capture = &ptr->captures[ptr->currentCaptureIndex];

        profilerCapture_timestamp(context, capture, "EndCapture");

        profilerCapture_download(context, capture);
    }
}

void profiler_processCaptures(Context* context, Profiler* ptr)
{
    if (!ptr->reportEntries)
    {
        return;
    }

    for (NvFlowUint64 captureIndex = 0u; captureIndex < ptr->captures.size; captureIndex++)
    {
        auto capture = &ptr->captures[captureIndex];

        NvFlowUint64 numEntries = 0u;
        NvFlowProfilerEntry* entries = nullptr;
        if (profilerCapture_mapResults(context, capture, &numEntries, &entries))
        {
            ptr->reportEntries(ptr->userdata, capture->captureID, (NvFlowUint)numEntries, entries);

            profilerCapture_unmapResults(context, capture);
        }
    }
}

void profiler_timestamp(Context* context, Profiler* ptr, const char* label)
{
    if (!ptr->reportEntries)
    {
        return;
    }

    if (ptr->currentCaptureIndex < ptr->captures.size)
    {
        auto capture = &ptr->captures[ptr->currentCaptureIndex];

        profilerCapture_timestamp(context, capture, label);
    }
}

void enableProfiler(NvFlowContext* contextIn, void* userdata, void(NV_FLOW_ABI* reportEntries)(void* userdata, NvFlowUint64 captureID, NvFlowUint numEntries, NvFlowProfilerEntry* entries))
{
    auto context = cast(contextIn);

    context->profiler->userdata = userdata;
    context->profiler->reportEntries = reportEntries;
}

void disableProfiler(NvFlowContext* contextIn)
{
    auto context = cast(contextIn);

    context->profiler->reportEntries = nullptr;
}

/// ***************************** Resource Register *****************************************************

NvFlowUint64 registerResourceId(NvFlowContext* context, NvFlowBuffer* buffer, NvFlowTexture* texture)
{
    auto ctx = cast(context);

    // check for existing
    for (NvFlowUint idx = 0u; idx < ctx->registeredResources.size; idx++)
    {
        if (ctx->registeredResources[idx].buffer == buffer && ctx->registeredResources[idx].texture == texture)
        {
            return ctx->registeredResources[idx].uid;
        }
    }

    ctx->registeredResourceCounter++;

    RegisteredResource resource = {};
    resource.buffer = buffer;
    resource.texture = texture;
    resource.uid = ctx->registeredResourceCounter;
    ctx->registeredResources.pushBack(resource);

    return resource.uid;
}

void unregisterResourceId(NvFlowContext* context, NvFlowUint64 resourceId)
{
    auto ctx = cast(context);
    NvFlowUint dstIdx = 0u;
    for (NvFlowUint srcIdx = 0u; srcIdx < ctx->registeredResources.size; srcIdx++)
    {
        if (ctx->registeredResources[srcIdx].uid != resourceId)
        {
            if (dstIdx != srcIdx)
            {
                ctx->registeredResources[dstIdx] = ctx->registeredResources[srcIdx];
            }
            dstIdx++;
        }
    }
    ctx->registeredResources.size = dstIdx;
}

NvFlowUint64 registerBufferId(NvFlowContext* context, NvFlowBuffer* buffer)
{
    return registerResourceId(context, buffer, nullptr);
}

NvFlowUint64 registerTextureId(NvFlowContext* context, NvFlowTexture* texture)
{
    return registerResourceId(context, nullptr, texture);
}

void unregisterBufferId(NvFlowContext* context, NvFlowUint64 bufferId)
{
    unregisterResourceId(context, bufferId);
}

void unregisterTextureId(NvFlowContext* context, NvFlowUint64 textureId)
{
    unregisterResourceId(context, textureId);
}

void setResourceMinLifetime(NvFlowContext* context, NvFlowUint64 minLifetime)
{
    auto ctx = cast(context);
    ctx->minLifetime = minLifetime;
}

} // end namespace
