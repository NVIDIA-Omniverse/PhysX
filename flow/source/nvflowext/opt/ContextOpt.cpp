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

#include "NvFlowExt.h"

#include "NvFlowArray.h"
#include "NvFlowString.h"

//#define DEBUG_LOG_PRINT

#ifdef DEBUG_LOG_PRINT
#include <stdio.h>
void debugLogPrint(NvFlowLogLevel level, const char* format, ...)
{
    va_list args;
    va_start(args, format);

    const char* prefix = "Unknown";
    if (level == eNvFlowLogLevel_error)
    {
        prefix = "Error";
    }
    else if (level == eNvFlowLogLevel_warning)
    {
        prefix = "Warning";
    }
    else if (level == eNvFlowLogLevel_info)
    {
        prefix = "Info";
    }
    printf("%s: ", prefix);
    vprintf(format, args);
    printf("\n");

    va_end(args);
}
#endif

namespace
{
    struct BufferBackend
    {
        NvFlowBool32 isActive = NV_FLOW_FALSE;
        NvFlowUint64 lastActive = 0llu;
        NvFlowMemoryType memoryType = eNvFlowMemoryType_device;
        NvFlowBufferDesc desc = {};
        NvFlowBuffer* buffer = nullptr;
        NvFlowBufferTransient* bufferTransient = nullptr;
        NvFlowUint64 transientFrame = ~0llu;
    };

    struct BufferVirtual
    {
        int refCount = 0;
        NvFlowMemoryType memoryType = eNvFlowMemoryType_device;
        NvFlowBufferDesc desc = {};
        BufferBackend* backend = nullptr;
    };

    struct Buffer
    {
        NvFlowMemoryType memoryType = eNvFlowMemoryType_device;
        NvFlowBufferDesc desc = {};
        BufferVirtual* bufferVirtual = nullptr;
    };
    NV_FLOW_CAST_PAIR(NvFlowBuffer, Buffer)

    struct BufferTransient
    {
        BufferVirtual* bufferVirtual = nullptr;
        NvFlowBufferTransient* bufferTransientExternal = nullptr;
        NvFlowFormat aliasFormat = eNvFlowFormat_unknown;
        NvFlowUint aliasStructureStride = ~0u;
        int commandEnd = -1;
    };
    NV_FLOW_CAST_PAIR(NvFlowBufferTransient, BufferTransient)

    struct BufferAcquire
    {
        BufferVirtual* bufferVirtual = nullptr;
    };
    NV_FLOW_CAST_PAIR(NvFlowBufferAcquire, BufferAcquire)

    struct TextureBackend
    {
        NvFlowBool32 isActive = NV_FLOW_FALSE;
        NvFlowUint64 lastActive = 0llu;
        NvFlowTextureDesc desc = {};
        NvFlowTexture* texture = nullptr;
        NvFlowTextureTransient* textureTransient = nullptr;
        NvFlowUint64 transientFrame = ~0llu;
    };
    struct TextureVirtual
    {
        int refCount = 0;
        NvFlowTextureDesc desc = {};
        TextureBackend* backend = nullptr;
    };

    struct Texture
    {
        TextureVirtual* textureVirtual = nullptr;
    };
    NV_FLOW_CAST_PAIR(NvFlowTexture, Texture)

    struct TextureTransient
    {
        TextureVirtual* textureVirtual = nullptr;
        NvFlowTextureTransient* textureTransientExternal = nullptr;
        NvFlowFormat aliasFormat = eNvFlowFormat_unknown;
        int commandEnd = -1;
    };
    NV_FLOW_CAST_PAIR(NvFlowTextureTransient, TextureTransient)

    struct TextureAcquire
    {
        TextureVirtual* textureVirtual = nullptr;
    };
    NV_FLOW_CAST_PAIR(NvFlowTextureAcquire, TextureAcquire)

    struct SamplerBackend
    {
        int refCount = 0;
        NvFlowSamplerDesc desc = {};
        NvFlowSampler* sampler = nullptr;
    };

    struct Sampler
    {
        SamplerBackend* backend;
    };
    NV_FLOW_CAST_PAIR(NvFlowSampler, Sampler)

    struct ComputePipelineBackend
    {
        NvFlowComputePipeline* pipeline = nullptr;
    };

    struct ComputePipeline
    {
        ComputePipelineBackend* backend = nullptr;
    };
    NV_FLOW_CAST_PAIR(NvFlowComputePipeline, ComputePipeline)

    struct ContextOpt;

    struct Command
    {
        const char* debugName;
        void* cmd;
        void (NV_FLOW_ABI* execute)(ContextOpt* contextOpt, NvFlowUint commandIdx, void* cmdIn);
    };

    struct FrameMapping
    {
        NvFlowUint64 optFrame;
        NvFlowUint64 backendFrame;
    };

    struct ContextOpt
    {
        NvFlowStringPool* stringPool = nullptr;

        NvFlowThreadPoolInterface threadPoolInterface = {};
        NvFlowThreadPool* threadPool = nullptr;
        NvFlowUint threadCount = 0u;

        NvFlowUint64 currentFrame = 2llu;
        NvFlowUint64 lastFrameCompleted = 1llu;
        NvFlowArray<FrameMapping> frameMappings;

        NvFlowUint64 minLifetime = 240u;

        NvFlowContextInterface backendContextInterface = {};
        NvFlowContext* backendContext = nullptr;

        NvFlowArrayPointer<BufferVirtual*> bufferVirtuals;
        NvFlowArrayPointer<TextureVirtual*> textureVirtuals;

        NvFlowArrayPointer<BufferBackend*> bufferBackends;
        NvFlowArrayPointer<TextureBackend*> textureBackends;
        NvFlowArrayPointer<SamplerBackend*> samplerBackends;
        NvFlowArrayPointer<ComputePipelineBackend*> computePipelineBackends;

        NvFlowArrayPointer<Buffer*> buffers;
        NvFlowArrayPointer<BufferTransient*> bufferTransients;
        NvFlowArrayPointer<BufferAcquire*> bufferAcquires;

        NvFlowArrayPointer<Texture*> textures;
        NvFlowArrayPointer<TextureTransient*> textureTransients;
        NvFlowArrayPointer<TextureAcquire*> textureAcquires;

        NvFlowArrayPointer<Sampler*> samplers;

        NvFlowArrayPointer<ComputePipeline*> computePipelines;

        NvFlowArray<Command> commands;

        NvFlowArray<BufferVirtual*> exportBufferVirtuals;
        NvFlowArray<TextureVirtual*> exportTextureVirtuals;

        NvFlowSampler* defaultSampler = nullptr;
    };

    NV_FLOW_CAST_PAIR(NvFlowContextOpt, ContextOpt)
    NV_FLOW_CAST_PAIR_NAMED(context, NvFlowContext, ContextOpt)

    template <class T>
    T* stringPoolAllocate(ContextOpt* ctx)
    {
        return (T*)NvFlowStringPoolAllocate(ctx->stringPool, int(sizeof(T)));
    }

    template <class T>
    T* stringPoolAllocateN(ContextOpt* ctx, NvFlowUint64 count)
    {
        return (T*)NvFlowStringPoolAllocate(ctx->stringPool, int(count * sizeof(T)));
    }

    NvFlowContextInterface* getContextInterfaceOpt();

    void destroySampler(NvFlowContext* context, NvFlowSampler* sampler);

    NvFlowContextOpt* create(NvFlowContextInterface* backendContextInterface, NvFlowContext* backendContext)
    {
        auto ptr = new ContextOpt();
        NvFlowContextInterface_duplicate(&ptr->backendContextInterface, backendContextInterface);
        ptr->backendContext = backendContext;

        ptr->stringPool = NvFlowStringPoolCreate();

        NvFlowThreadPoolInterface_duplicate(&ptr->threadPoolInterface, NvFlowGetThreadPoolInterface());
        ptr->threadPool = nullptr;
        ptr->threadCount = 0u;

        return cast(ptr);
    }

    void flush(NvFlowContextOpt* contextOpt);

    void destroy(NvFlowContextOpt* contextOpt)
    {
        auto ctx = cast(contextOpt);

        if (ctx->defaultSampler)
        {
            destroySampler(context_cast(ctx), ctx->defaultSampler);
            ctx->defaultSampler = nullptr;
        }

        flush(contextOpt);

        for (NvFlowUint idx = 0u; idx < ctx->bufferBackends.size; idx++)
        {
            auto bufferBackend = ctx->bufferBackends[idx];
            if (bufferBackend->buffer)
            {
                ctx->backendContextInterface.destroyBuffer(ctx->backendContext, bufferBackend->buffer);
                bufferBackend->buffer = nullptr;
            }
        }
        for (NvFlowUint idx = 0u; idx < ctx->textureBackends.size; idx++)
        {
            auto textureBackend = ctx->textureBackends[idx];
            if (textureBackend->texture)
            {
                ctx->backendContextInterface.destroyTexture(ctx->backendContext, textureBackend->texture);
                textureBackend->texture = nullptr;
            }
        }
        for (NvFlowUint idx = 0u; idx < ctx->samplerBackends.size; idx++)
        {
            auto samplerBackend = ctx->samplerBackends[idx];
            if (samplerBackend->sampler)
            {
                ctx->backendContextInterface.destroySampler(ctx->backendContext, samplerBackend->sampler);
                samplerBackend->sampler = nullptr;
            }
        }
        for (NvFlowUint idx = 0u; idx < ctx->computePipelineBackends.size; idx++)
        {
            auto computePipelineBackend = ctx->computePipelineBackends[idx];
            if (computePipelineBackend->pipeline)
            {
                ctx->backendContextInterface.destroyComputePipeline(ctx->backendContext, computePipelineBackend->pipeline);
                computePipelineBackend->pipeline = nullptr;
            }
        }

        if (ctx->threadPool)
        {
            ctx->threadPoolInterface.destroy(ctx->threadPool);
            ctx->threadPool = nullptr;
        }

        NvFlowStringPoolDestroy(ctx->stringPool);

        delete ctx;
    }

    void getContext(NvFlowContextOpt* contextOpt, NvFlowContextInterface** pContextInterface, NvFlowContext** pContext)
    {
        auto ptr = cast(contextOpt);
        if (pContextInterface)
        {
            *pContextInterface = getContextInterfaceOpt();
        }
        if (pContext)
        {
            *pContext = context_cast(ptr);
        }
    }

    void accumBufferTransientLifetime(BufferTransient* ptr, NvFlowUint64 commandIdx)
    {
        if (int(commandIdx) > ptr->commandEnd)
        {
            ptr->commandEnd = int(commandIdx);
        }
    }
    void accumTextureTransientLifetime(TextureTransient* ptr, NvFlowUint64 commandIdx)
    {
        if (int(commandIdx) > ptr->commandEnd)
        {
            ptr->commandEnd = int(commandIdx);
        }
    }

    BufferVirtual* getBufferVirtual(ContextOpt* ctx, NvFlowMemoryType memoryType, const NvFlowBufferDesc* desc)
    {
        auto bufferVirtual = ctx->bufferVirtuals.allocateBackPointer();
        bufferVirtual->refCount = 1;
        bufferVirtual->desc = *desc;
        bufferVirtual->memoryType = memoryType;
        bufferVirtual->backend = nullptr;
        return bufferVirtual;
    }
    NvFlowBufferTransient* getBufferBackendInterop(ContextOpt* ctx, BufferVirtual* bufferVirtual, NvFlowFormat aliasFormat, NvFlowUint aliasStructureStride, NvFlowBuffer* interopBuffer)
    {
        if (!bufferVirtual->backend && !interopBuffer)    // do not recycle for interop
        {
            // try to find match
            for (NvFlowUint idx = 0u; idx < ctx->bufferBackends.size; idx++)
            {
                auto bufferBackend = ctx->bufferBackends[idx];
                if (!bufferBackend->isActive)
                {
                    if (bufferBackend->desc.usageFlags == bufferVirtual->desc.usageFlags &&
                        bufferBackend->desc.format == bufferVirtual->desc.format &&
                        bufferBackend->desc.structureStride == bufferVirtual->desc.structureStride &&
                        bufferBackend->desc.sizeInBytes == bufferVirtual->desc.sizeInBytes &&
                        bufferBackend->memoryType == bufferVirtual->memoryType)
                    {
                        bufferBackend->isActive = NV_FLOW_TRUE;
                        bufferBackend->lastActive = ctx->currentFrame;
                        bufferVirtual->backend = bufferBackend;
                        break;
                    }
                }
            }
        }
        if (!bufferVirtual->backend)
        {
            auto bufferBackend = ctx->bufferBackends.allocateBackPointer();
            bufferBackend->isActive = NV_FLOW_TRUE;
            bufferBackend->lastActive = ctx->currentFrame;
            bufferBackend->desc = bufferVirtual->desc;
            bufferBackend->memoryType = bufferVirtual->memoryType;
            bufferBackend->buffer = nullptr;
            bufferBackend->bufferTransient = nullptr;
            bufferBackend->transientFrame = ~0llu;
            bufferVirtual->backend = bufferBackend;
        }
        auto bufferBackend = bufferVirtual->backend;
        if (!bufferBackend->buffer)
        {
            if (interopBuffer)    // source from interop if available
            {
                bufferBackend->buffer = interopBuffer;
            }
            else
            {
                bufferBackend->buffer = ctx->backendContextInterface.createBuffer(
                    ctx->backendContext, bufferBackend->memoryType, &bufferBackend->desc);
            }
            bufferBackend->bufferTransient = nullptr;
            bufferBackend->transientFrame = ~0llu;
        }
        // refresh transient
        NvFlowUint64 currentFrame = ctx->backendContextInterface.getCurrentFrame(ctx->backendContext);
        if (bufferBackend->transientFrame != currentFrame)
        {
            bufferBackend->bufferTransient = ctx->backendContextInterface.registerBufferAsTransient(ctx->backendContext, bufferBackend->buffer);
            bufferBackend->transientFrame = currentFrame;
        }
        NvFlowBufferTransient* backendTransient = bufferBackend->bufferTransient;
        if (aliasFormat != eNvFlowFormat_unknown || aliasStructureStride != ~0u)
        {
            backendTransient = ctx->backendContextInterface.aliasBufferTransient(
                ctx->backendContext, backendTransient, aliasFormat, aliasStructureStride);
        }
        return backendTransient;
    }
    NvFlowBufferTransient* getBufferBackend(ContextOpt* ctx, BufferVirtual* bufferVirtual, NvFlowFormat aliasFormat, NvFlowUint aliasStructureStride)
    {
        return getBufferBackendInterop(ctx, bufferVirtual, aliasFormat, aliasStructureStride, nullptr);
    }
    void addRefBufferVirtual(ContextOpt* ctx, BufferVirtual* bufferVirtual)
    {
        bufferVirtual->refCount++;
    }
    void releaseBufferVirtual(ContextOpt* ctx, BufferVirtual* bufferVirtual)
    {
        bufferVirtual->refCount--;
        if (bufferVirtual->refCount == 0)
        {
            if (bufferVirtual->backend)
            {
                bufferVirtual->backend->isActive = NV_FLOW_FALSE;
                bufferVirtual->backend->lastActive = ctx->currentFrame;
                bufferVirtual->backend = nullptr;
            }
        }
    }
    void releaseBufferTransientConditional(ContextOpt* ctx, BufferTransient* bufferTransient, NvFlowUint commandIdx)
    {
        if (bufferTransient->bufferVirtual && bufferTransient->commandEnd == int(commandIdx))
        {
            releaseBufferVirtual(ctx, bufferTransient->bufferVirtual);
            bufferTransient->bufferVirtual = nullptr;
        }
    }
    TextureVirtual* getTextureVirtual(ContextOpt* ctx, const NvFlowTextureDesc* descIn)
    {
        // normalize desc
        NvFlowTextureDesc desc = *descIn;
        if (desc.mipLevels == 0u)
        {
            desc.mipLevels = 1u;
        }
        if (desc.textureType == eNvFlowTextureType_1d)
        {
            desc.height = 1u;
            desc.depth = 1u;
        }
        if (desc.textureType == eNvFlowTextureType_2d)
        {
            desc.depth = 1u;
        }
        auto textureVirtual = ctx->textureVirtuals.allocateBackPointer();
        textureVirtual->refCount = 1;
        textureVirtual->desc = desc;
        textureVirtual->backend = nullptr;
        return textureVirtual;
    }
    NvFlowTextureTransient* getTextureBackend(ContextOpt* ctx, TextureVirtual* textureVirtual, NvFlowFormat aliasFormat)
    {
        if (!textureVirtual->backend)
        {
            // try to find match
            for (NvFlowUint idx = 0u; idx < ctx->textureBackends.size; idx++)
            {
                auto textureBackend = ctx->textureBackends[idx];
                if (!textureBackend->isActive)
                {
                    if (textureBackend->desc.textureType == textureVirtual->desc.textureType &&
                        textureBackend->desc.usageFlags == textureVirtual->desc.usageFlags &&
                        textureBackend->desc.format == textureVirtual->desc.format &&
                        textureBackend->desc.width == textureVirtual->desc.width &&
                        textureBackend->desc.height == textureVirtual->desc.height &&
                        textureBackend->desc.depth == textureVirtual->desc.depth &&
                        textureBackend->desc.mipLevels == textureVirtual->desc.mipLevels &&
                        textureBackend->desc.optimizedClearValue.x == textureVirtual->desc.optimizedClearValue.x &&
                        textureBackend->desc.optimizedClearValue.y == textureVirtual->desc.optimizedClearValue.y &&
                        textureBackend->desc.optimizedClearValue.z == textureVirtual->desc.optimizedClearValue.z &&
                        textureBackend->desc.optimizedClearValue.w == textureVirtual->desc.optimizedClearValue.w)
                    {
                        textureBackend->isActive = NV_FLOW_TRUE;
                        textureBackend->lastActive = ctx->currentFrame;
                        textureVirtual->backend = textureBackend;
                        break;
                    }
                }
            }
        }
        if (!textureVirtual->backend)
        {
            auto textureBackend = ctx->textureBackends.allocateBackPointer();
            textureBackend->isActive = NV_FLOW_TRUE;
            textureBackend->lastActive = ctx->currentFrame;
            textureBackend->desc = textureVirtual->desc;
            textureBackend->texture = nullptr;
            textureBackend->textureTransient = nullptr;
            textureBackend->transientFrame = ~0llu;
            textureVirtual->backend = textureBackend;
        }
        auto textureBackend = textureVirtual->backend;
        if (!textureBackend->texture)
        {
            textureBackend->texture = ctx->backendContextInterface.createTexture(ctx->backendContext, &textureBackend->desc);
            textureBackend->textureTransient = nullptr;
            textureBackend->transientFrame = ~0llu;
        }
        // refresh transient
        NvFlowUint64 currentFrame = ctx->backendContextInterface.getCurrentFrame(ctx->backendContext);
        if (textureBackend->transientFrame != currentFrame)
        {
            textureBackend->textureTransient = ctx->backendContextInterface.registerTextureAsTransient(ctx->backendContext, textureBackend->texture);
            textureBackend->transientFrame = currentFrame;
        }
        NvFlowTextureTransient* backendTransient = textureBackend->textureTransient;
        if (aliasFormat != eNvFlowFormat_unknown)
        {
            backendTransient = ctx->backendContextInterface.aliasTextureTransient(
                ctx->backendContext, backendTransient, aliasFormat);
        }
        return backendTransient;
    }
    void addRefTextureVirtual(ContextOpt* ctx, TextureVirtual* textureVirtual)
    {
        textureVirtual->refCount++;
    }
    void releaseTextureVirtual(ContextOpt* ctx, TextureVirtual* textureVirtual)
    {
        textureVirtual->refCount--;
        if (textureVirtual->refCount == 0)
        {
            if (textureVirtual->backend)
            {
                textureVirtual->backend->isActive = NV_FLOW_FALSE;
                textureVirtual->backend->lastActive = ctx->currentFrame;
                textureVirtual->backend = nullptr;
            }
        }
    }
    void releaseTextureTransientConditional(ContextOpt* ctx, TextureTransient* textureTransient, NvFlowUint commandIdx)
    {
        if (textureTransient->textureVirtual && textureTransient->commandEnd == int(commandIdx))
        {
            releaseTextureVirtual(ctx, textureTransient->textureVirtual);
            textureTransient->textureVirtual = nullptr;
        }
    }
    SamplerBackend* getSamplerBackend(ContextOpt* ctx, const NvFlowSamplerDesc* desc)
    {
        // try to find match
        for (NvFlowUint idx = 0u; idx < ctx->samplerBackends.size; idx++)
        {
            auto samplerBackend = ctx->samplerBackends[idx];
            if (samplerBackend->desc.addressModeU == desc->addressModeU &&
                samplerBackend->desc.addressModeV == desc->addressModeV &&
                samplerBackend->desc.addressModeW == desc->addressModeW &&
                samplerBackend->desc.filterMode == desc->filterMode)
            {
                samplerBackend->refCount++;
                return samplerBackend;
            }
        }
        auto samplerBackend = ctx->samplerBackends.allocateBackPointer();
        samplerBackend->desc = *desc;
        samplerBackend->sampler = ctx->backendContextInterface.createSampler(ctx->backendContext, desc);
        return samplerBackend;
    }
    ComputePipelineBackend* getComputePipelineBackend(ContextOpt* ctx, const NvFlowComputePipelineDesc* desc)
    {
        auto computePipelineBackend = ctx->computePipelineBackends.allocateBackPointer();
        computePipelineBackend->pipeline = ctx->backendContextInterface.createComputePipeline(ctx->backendContext, desc);
        return computePipelineBackend;
    }

    void flush(NvFlowContextOpt* contextOpt)
    {
        auto ctx = cast(contextOpt);

        // release buffer exports
        for (NvFlowUint idx = 0u; idx < ctx->exportBufferVirtuals.size; idx++)
        {
            releaseBufferVirtual(ctx, ctx->exportBufferVirtuals[idx]);
        }
        ctx->exportBufferVirtuals.size = 0u;
        // release texture exports
        for (NvFlowUint idx = 0u; idx < ctx->exportTextureVirtuals.size; idx++)
        {
            releaseTextureVirtual(ctx, ctx->exportTextureVirtuals[idx]);
        }
        ctx->exportTextureVirtuals.size = 0u;

        for (NvFlowUint commandIdx = 0u; commandIdx < ctx->commands.size; commandIdx++)
        {
            ctx->commands[commandIdx].execute(ctx, commandIdx, ctx->commands[commandIdx].cmd);
        }

        // reset all commands
        ctx->commands.size = 0u;

        // reset transient handles
        ctx->bufferTransients.size = 0u;
        ctx->textureTransients.size = 0u;

        // compact virtuals
        {
            NvFlowUint idx = 0u;
            while (idx < ctx->bufferVirtuals.size)
            {
                if (!ctx->bufferVirtuals[idx]->backend && ctx->bufferVirtuals[idx]->refCount == 0)
                {
                    ctx->bufferVirtuals.removeSwapPointerAtIndex(idx);
                }
                else
                {
                    idx++;
                }
            }
            idx = 0u;
            while (idx < ctx->textureVirtuals.size)
            {
                if (!ctx->textureVirtuals[idx]->backend && ctx->textureVirtuals[idx]->refCount == 0)
                {
                    ctx->textureVirtuals.removeSwapPointerAtIndex(idx);
                }
                else
                {
                    idx++;
                }
            }
        }
        // compact acquires
        {
            NvFlowUint idx = 0u;
            while (idx < ctx->bufferAcquires.size)
            {
                if (!ctx->bufferAcquires[idx]->bufferVirtual)
                {
                    ctx->bufferAcquires.removeSwapPointerAtIndex(idx);
                }
                else
                {
                    idx++;
                }
            }
            idx = 0u;
            while (idx < ctx->textureAcquires.size)
            {
                if (!ctx->textureAcquires[idx]->textureVirtual)
                {
                    ctx->textureAcquires.removeSwapPointerAtIndex(idx);
                }
                else
                {
                    idx++;
                }
            }
        }
        // compact buffers/textures/samplers
        {
            NvFlowUint idx = 0u;
            while (idx < ctx->buffers.size)
            {
                if (!ctx->buffers[idx]->bufferVirtual)
                {
                    ctx->buffers.removeSwapPointerAtIndex(idx);
                }
                else
                {
                    idx++;
                }
            }
            idx = 0u;
            while (idx < ctx->textures.size)
            {
                if (!ctx->textures[idx]->textureVirtual)
                {
                    ctx->textures.removeSwapPointerAtIndex(idx);
                }
                else
                {
                    idx++;
                }
            }
            idx = 0u;
            while (idx < ctx->samplers.size)
            {
                if (!ctx->samplers[idx]->backend)
                {
                    ctx->samplers.removeSwapPointerAtIndex(idx);
                }
                else
                {
                    idx++;
                }
            }
        }

        // reset string pool
        NvFlowStringPoolReset(ctx->stringPool);

        // add frame mapping
        {
            FrameMapping currentMapping = {};
            currentMapping.optFrame = ctx->currentFrame;
            currentMapping.backendFrame = ctx->backendContextInterface.getCurrentFrame(ctx->backendContext);
            ctx->frameMappings.pushBack(currentMapping);
        }

        // process frame mappings to update lastFrameCompleted
        {
            NvFlowUint64 backendLastFrameCompleted = ctx->backendContextInterface.getLastFrameCompleted(ctx->backendContext);
            NvFlowUint dstIdx = 0u;
            NvFlowUint srcIdx = 0u;
            for (; srcIdx < ctx->frameMappings.size; srcIdx++)
            {
                if (ctx->frameMappings[srcIdx].backendFrame <= backendLastFrameCompleted)
                {
                    ctx->lastFrameCompleted = ctx->frameMappings[srcIdx].optFrame;
                }
                else
                {
                    ctx->frameMappings[dstIdx] = ctx->frameMappings[srcIdx];
                    dstIdx++;
                }
            }
            ctx->frameMappings.size = dstIdx;
        }

        // clean up unused resources
        auto logPrint = ctx->backendContextInterface.getLogPrint(ctx->backendContext);
        for (NvFlowUint idx = 0u; idx < ctx->bufferBackends.size; idx++)
        {
            auto bufferBackend = ctx->bufferBackends[idx];
            if (bufferBackend->buffer && !bufferBackend->isActive && (bufferBackend->lastActive + ctx->minLifetime) <= ctx->lastFrameCompleted)
            {
                if (logPrint)
                {
                    logPrint(eNvFlowLogLevel_info, "ContextOpt : freeing buffer %d", idx);
                }
                ctx->backendContextInterface.destroyBuffer(ctx->backendContext, bufferBackend->buffer);
                bufferBackend->buffer = nullptr;

                *bufferBackend = BufferBackend{};
                ctx->bufferBackends.removeSwapPointerAtIndex(idx);
                idx--;
            }
        }
        for (NvFlowUint idx = 0u; idx < ctx->textureBackends.size; idx++)
        {
            auto textureBackend = ctx->textureBackends[idx];
            if (textureBackend->texture && !textureBackend->isActive && (textureBackend->lastActive + ctx->minLifetime) <= ctx->lastFrameCompleted)
            {
                if (logPrint)
                {
                    logPrint(eNvFlowLogLevel_info, "ContextOpt : freeing texture %d", idx);
                }
                ctx->backendContextInterface.destroyTexture(ctx->backendContext, textureBackend->texture);
                textureBackend->texture = nullptr;

                *textureBackend = TextureBackend{};
                ctx->textureBackends.removeSwapPointerAtIndex(idx);
                idx--;
            }
        }

        // start new frame
        ctx->currentFrame++;
    }

    void getContextConfig(NvFlowContext* context, NvFlowContextConfig* config)
    {
        auto ptr = context_cast(context);
        ptr->backendContextInterface.getContextConfig(ptr->backendContext, config);
    }

    NvFlowBool32 isFeatureSupported(NvFlowContext* context, NvFlowContextFeature feature)
    {
        auto ptr = context_cast(context);
        if (ptr->backendContextInterface.isFeatureSupported)
        {
            return ptr->backendContextInterface.isFeatureSupported(ptr->backendContext, feature);
        }
        return NV_FLOW_FALSE;
    }

    NvFlowUint64 getCurrentFrame(NvFlowContext* context)
    {
        auto ctx = context_cast(context);
        return ctx->currentFrame;
    }

    NvFlowUint64 getLastFrameCompleted(NvFlowContext* context)
    {
        auto ctx = context_cast(context);
        return ctx->lastFrameCompleted;
    }

    NvFlowUint64 getCurrentGlobalFrame(NvFlowContext* context)
    {
        auto ctx = context_cast(context);
        return ctx->backendContextInterface.getCurrentFrame(ctx->backendContext);
    }

    NvFlowUint64 getLastGlobalFrameCompleted(NvFlowContext* context)
    {
        auto ctx = context_cast(context);
        return ctx->backendContextInterface.getLastFrameCompleted(ctx->backendContext);
    }



    NvFlowLogPrint_t getLogPrint(NvFlowContext* context)
    {
        auto ptr = context_cast(context);
#ifdef DEBUG_LOG_PRINT
        return debugLogPrint;
#else
        return ptr->backendContextInterface.getLogPrint(ptr->backendContext);
#endif
    }

    void executeTasks(NvFlowContext* context, NvFlowUint taskCount, NvFlowUint taskGranularity, NvFlowContextThreadPoolTask_t task, void* userdata)
    {
        auto ptr = context_cast(context);
        if (ptr->backendContextInterface.executeTasks)
        {
            ptr->backendContextInterface.executeTasks(ptr->backendContext, taskCount, taskGranularity, task, userdata);
        }
        else
        {
            if (!ptr->threadPool)
            {
                ptr->threadCount = ptr->threadPoolInterface.getDefaultThreadCount();
                ptr->threadPool = ptr->threadPoolInterface.create(ptr->threadCount, 0llu);
            }
            ptr->threadPoolInterface.execute(ptr->threadPool, taskCount, taskGranularity, task, userdata);
        }
    }

    Buffer* createBufferHandle(ContextOpt* ctx)
    {
        Buffer* ret = ctx->buffers.allocateBackPointer();
        ret->bufferVirtual = nullptr;
        return ret;
    }

    BufferTransient* createBufferTransientHandle(ContextOpt* ctx)
    {
        BufferTransient* ret = ctx->bufferTransients.allocateBackPointer();
        ret->bufferVirtual = nullptr;
        ret->bufferTransientExternal = nullptr;
        ret->aliasFormat = eNvFlowFormat_unknown;
        ret->aliasStructureStride = ~0u;
        ret->commandEnd = -1;
        return ret;
    }

    BufferAcquire* createBufferAcquireHandle(ContextOpt* ctx)
    {
        BufferAcquire* ret = ctx->bufferAcquires.allocateBackPointer();
        ret->bufferVirtual = nullptr;
        return ret;
    }

    NvFlowBufferTransient* importBackendBufferTransient(NvFlowContextOpt* contextOpt, NvFlowBufferTransient* backendBufferTransient)
    {
        auto ctx = cast(contextOpt);

        if (!backendBufferTransient)
        {
            return nullptr;
        }

        auto bufferTransient = createBufferTransientHandle(ctx);

        // check for import of an export
        for (NvFlowUint idx = 0u; idx < ctx->exportBufferVirtuals.size; idx++)
        {
            if (ctx->exportBufferVirtuals[idx]->backend->bufferTransient == backendBufferTransient)
            {
                addRefBufferVirtual(ctx, ctx->exportBufferVirtuals[idx]);
                bufferTransient->bufferVirtual = ctx->exportBufferVirtuals[idx];
            }
        }
        if (!bufferTransient->bufferVirtual)
        {
            bufferTransient->bufferTransientExternal = backendBufferTransient;
        }
        return cast(bufferTransient);
    }

    void exportBufferTransient(NvFlowContextOpt* contextOpt, NvFlowBufferTransient* bufferTransient, NvFlowBufferTransient** pBackendBufferTransient)
    {
        auto ctx = cast(contextOpt);

        if (!bufferTransient)
        {
            *pBackendBufferTransient = nullptr;
            return;
        }

        struct CreateBufferCmd
        {
            BufferTransient* bufferTransient;
            NvFlowBufferTransient** pBackendBufferTransient;
        };
        auto cmd = stringPoolAllocate<CreateBufferCmd>(ctx);

        cmd->bufferTransient = cast(bufferTransient);
        cmd->pBackendBufferTransient = pBackendBufferTransient;

        accumBufferTransientLifetime(cmd->bufferTransient, ctx->commands.size);

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (CreateBufferCmd*)cmdIn;
            if (cmd->bufferTransient->bufferTransientExternal)
            {
                *cmd->pBackendBufferTransient = cmd->bufferTransient->bufferTransientExternal;
            }
            else
            {
                *cmd->pBackendBufferTransient = getBufferBackend(ctx, cmd->bufferTransient->bufferVirtual,
                    cmd->bufferTransient->aliasFormat, cmd->bufferTransient->aliasStructureStride);

                addRefBufferVirtual(ctx, cmd->bufferTransient->bufferVirtual);
                ctx->exportBufferVirtuals.pushBack(cmd->bufferTransient->bufferVirtual);
            }
            releaseBufferTransientConditional(ctx, cmd->bufferTransient, commandIdx);
        };
        ctx->commands.pushBack(Command{"exportBufferTransient", cmd, execute });
    }

    NvFlowBuffer* createBuffer(NvFlowContext* context, NvFlowMemoryType memoryType, const NvFlowBufferDesc* desc)
    {
        auto ctx = context_cast(context);

        struct CreateBufferCmd
        {
            Buffer* buffer;
        };
        auto cmd = stringPoolAllocate<CreateBufferCmd>(ctx);

        cmd->buffer = createBufferHandle(ctx);
        cmd->buffer->desc = *desc;
        cmd->buffer->memoryType = memoryType;

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (CreateBufferCmd*)cmdIn;
            // due to forwarding on mapBuffer, it is possible this is already created
            if (!cmd->buffer->bufferVirtual)
            {
                cmd->buffer->bufferVirtual = getBufferVirtual(ctx, cmd->buffer->memoryType, &cmd->buffer->desc);
            }
        };
        ctx->commands.pushBack(Command{"createBuffer", cmd, execute});

        return cast(cmd->buffer);
    }

    void destroyBuffer(NvFlowContext* context, NvFlowBuffer* buffer)
    {
        auto ctx = context_cast(context);

        struct DestroyBufferParams
        {
            Buffer* buffer;
        };
        auto cmd = stringPoolAllocate<DestroyBufferParams>(ctx);

        cmd->buffer = cast(buffer);

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (DestroyBufferParams*)cmdIn;
            releaseBufferVirtual(ctx, cmd->buffer->bufferVirtual);
            cmd->buffer->bufferVirtual = nullptr;
        };
        ctx->commands.pushBack(Command{"destroyBuffer", cmd, execute });
    }

    NvFlowBufferTransient* getBufferTransient(NvFlowContext* context, const NvFlowBufferDesc* desc)
    {
        auto ctx = context_cast(context);

        struct GetBufferTransientParams
        {
            BufferTransient* bufferTransient;
            NvFlowBufferDesc desc;
        };
        auto cmd = stringPoolAllocate<GetBufferTransientParams>(ctx);

        cmd->bufferTransient = createBufferTransientHandle(ctx);
        cmd->desc = *desc;

        accumBufferTransientLifetime(cmd->bufferTransient, ctx->commands.size);

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (GetBufferTransientParams*)cmdIn;
            cmd->bufferTransient->bufferVirtual = getBufferVirtual(ctx, eNvFlowMemoryType_device, &cmd->desc);

            releaseBufferTransientConditional(ctx, cmd->bufferTransient, commandIdx);
        };
        ctx->commands.pushBack(Command{"getBufferTransient", cmd, execute });

        return cast(cmd->bufferTransient);
    }

    NvFlowBufferTransient* registerBufferAsTransient(NvFlowContext* context, NvFlowBuffer* buffer)
    {
        auto ctx = context_cast(context);

        struct RegisterBufferAsTransientParams
        {
            BufferTransient* bufferTransient;
            Buffer* buffer;
        };
        auto cmd = stringPoolAllocate<RegisterBufferAsTransientParams>(ctx);

        cmd->bufferTransient = createBufferTransientHandle(ctx);
        cmd->buffer = cast(buffer);

        accumBufferTransientLifetime(cmd->bufferTransient, ctx->commands.size);

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (RegisterBufferAsTransientParams*)cmdIn;
            cmd->bufferTransient->bufferVirtual = cmd->buffer->bufferVirtual;
            addRefBufferVirtual(ctx, cmd->bufferTransient->bufferVirtual);

            releaseBufferTransientConditional(ctx, cmd->bufferTransient, commandIdx);
        };
        ctx->commands.pushBack(Command{"registerBufferAsTransient", cmd, execute });

        return cast(cmd->bufferTransient);
    }

    NvFlowBufferTransient* aliasBufferTransient(NvFlowContext* context, NvFlowBufferTransient* buffer, NvFlowFormat format, NvFlowUint structureStride)
    {
        auto ctx = context_cast(context);

        struct AliasBufferTransientParams
        {
            BufferTransient* bufferDst;
            BufferTransient* bufferSrc;
            NvFlowFormat format;
            NvFlowUint structureStride;
        };
        auto cmd = stringPoolAllocate<AliasBufferTransientParams>(ctx);

        cmd->bufferDst = createBufferTransientHandle(ctx);
        cmd->bufferSrc = cast(buffer);
        cmd->format = format;
        cmd->structureStride = structureStride;

        accumBufferTransientLifetime(cmd->bufferDst, ctx->commands.size);
        accumBufferTransientLifetime(cmd->bufferSrc, ctx->commands.size);

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (AliasBufferTransientParams*)cmdIn;
            cmd->bufferDst->bufferVirtual = cmd->bufferSrc->bufferVirtual;
            addRefBufferVirtual(ctx, cmd->bufferDst->bufferVirtual);

            cmd->bufferDst->aliasFormat = cmd->format;
            cmd->bufferDst->aliasStructureStride = cmd->structureStride;

            releaseBufferTransientConditional(ctx, cmd->bufferDst, commandIdx);
            releaseBufferTransientConditional(ctx, cmd->bufferSrc, commandIdx);
        };
        ctx->commands.pushBack(Command{ "aliasBufferTransient", cmd, execute });

        return cast(cmd->bufferDst);
    }

    NvFlowBufferAcquire* enqueueAcquireBuffer(NvFlowContext* context, NvFlowBufferTransient* buffer)
    {
        auto ctx = context_cast(context);

        struct EnqueueAcquireBufferParams
        {
            BufferAcquire* bufferAcquire;
            BufferTransient* bufferTransient;
        };
        auto cmd = stringPoolAllocate<EnqueueAcquireBufferParams>(ctx);

        cmd->bufferAcquire = createBufferAcquireHandle(ctx);
        cmd->bufferTransient = cast(buffer);

        accumBufferTransientLifetime(cmd->bufferTransient, ctx->commands.size);

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (EnqueueAcquireBufferParams*)cmdIn;
            cmd->bufferAcquire->bufferVirtual = cmd->bufferTransient->bufferVirtual;
            addRefBufferVirtual(ctx, cmd->bufferAcquire->bufferVirtual);
            releaseBufferTransientConditional(ctx, cmd->bufferTransient, commandIdx);
        };
        ctx->commands.pushBack(Command{"enqueueAcquireBuffer", cmd, execute });

        return cast(cmd->bufferAcquire);
    }

    NvFlowBool32 getAcquiredBuffer(NvFlowContext* context, NvFlowBufferAcquire* acquire, NvFlowBuffer** outBuffer)
    {
        auto ctx = context_cast(context);

        struct GetAcquiredBufferParams
        {
            BufferAcquire* bufferAcquire;
            Buffer* buffer;
        };
        auto cmd = stringPoolAllocate<GetAcquiredBufferParams>(ctx);

        cmd->bufferAcquire = cast(acquire);
        cmd->buffer = createBufferHandle(ctx);

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (GetAcquiredBufferParams*)cmdIn;
            cmd->buffer->bufferVirtual = cmd->bufferAcquire->bufferVirtual;
            cmd->bufferAcquire->bufferVirtual = nullptr;
        };
        ctx->commands.pushBack(Command{"getAcquiredBuffer", cmd, execute });

        *outBuffer = cast(cmd->buffer);

        return NV_FLOW_TRUE;
    }

    void* mapBuffer(NvFlowContext* context, NvFlowBuffer* buffer)
    {
        auto ctx = context_cast(context);
        auto ptr = cast(buffer);
        if (!ptr->bufferVirtual)
        {
            ptr->bufferVirtual = getBufferVirtual(ctx, ptr->memoryType, &ptr->desc);
        }
        if (ptr->bufferVirtual)
        {
            getBufferBackend(ctx, ptr->bufferVirtual, eNvFlowFormat_unknown, ~0u);
            auto backend = ptr->bufferVirtual->backend;
            return ctx->backendContextInterface.mapBuffer(ctx->backendContext, backend->buffer);
        }
        return nullptr;
    }

    void unmapBuffer(NvFlowContext* context, NvFlowBuffer* buffer)
    {
        auto ctx = context_cast(context);
        auto ptr = cast(buffer);
        if (ptr->bufferVirtual)
        {
            getBufferBackend(ctx, ptr->bufferVirtual, eNvFlowFormat_unknown, ~0u);
            auto backend = ptr->bufferVirtual->backend;
            ctx->backendContextInterface.unmapBuffer(ctx->backendContext, backend->buffer);
        }
    }

    NvFlowBufferTransient* getBufferTransientById(NvFlowContext* context, NvFlowUint64 bufferId)
    {
        auto ctx = context_cast(context);

        NvFlowBufferTransient* backendBuffer = ctx->backendContextInterface.getBufferTransientById(ctx->backendContext, bufferId);

        return importBackendBufferTransient(cast(ctx), backendBuffer);
    }

    void getBufferExternalHandle(NvFlowContext* context, NvFlowBuffer* buffer, NvFlowInteropHandle* dstHandle)
    {
        auto ctx = context_cast(context);
        auto ptr = cast(buffer);
        if (!ptr->bufferVirtual)
        {
            ptr->bufferVirtual = getBufferVirtual(ctx, ptr->memoryType, &ptr->desc);
        }
        if (ptr->bufferVirtual)
        {
            getBufferBackend(ctx, ptr->bufferVirtual, eNvFlowFormat_unknown, ~0u);
            auto backend = ptr->bufferVirtual->backend;
            ctx->backendContextInterface.getBufferExternalHandle(ctx->backendContext, backend->buffer, dstHandle);
        }
        else if (dstHandle)
        {
            NvFlowInteropHandle nullHandle = {};
            *dstHandle = nullHandle;
        }
    }

    void closeBufferExternalHandle(NvFlowContext* context, NvFlowBuffer* buffer, const NvFlowInteropHandle* srcHandle)
    {
        auto ctx = context_cast(context);
        auto ptr = cast(buffer);
        if (ptr->bufferVirtual)
        {
            getBufferBackend(ctx, ptr->bufferVirtual, eNvFlowFormat_unknown, ~0u);
            auto backend = ptr->bufferVirtual->backend;

            ctx->backendContextInterface.closeBufferExternalHandle(ctx->backendContext, backend->buffer, srcHandle);
        }
    }

    NvFlowBuffer* createBufferFromExternalHandle(NvFlowContext* context, const NvFlowBufferDesc* desc, const NvFlowInteropHandle* interopHandle)
    {
        auto ctx = context_cast(context);

        if (!ctx->backendContextInterface.createBufferFromExternalHandle)
        {
            return nullptr;
        }

        NvFlowBuffer* backendBuffer = ctx->backendContextInterface.createBufferFromExternalHandle(ctx->backendContext, desc, interopHandle);
        if (!backendBuffer)
        {
            return nullptr;
        }

        Buffer* ptr = createBufferHandle(ctx);
        ptr->desc = *desc;
        ptr->memoryType = eNvFlowMemoryType_device; // assume device for now

        if (!ptr->bufferVirtual)
        {
            ptr->bufferVirtual = getBufferVirtual(ctx, ptr->memoryType, &ptr->desc);
        }
        if (ptr->bufferVirtual)
        {
            getBufferBackendInterop(ctx, ptr->bufferVirtual, eNvFlowFormat_unknown, ~0u, backendBuffer);
        }

        return cast(ptr);
    }


    Texture* createTextureHandle(ContextOpt* ctx)
    {
        Texture* ret = ctx->textures.allocateBackPointer();
        ret->textureVirtual = nullptr;
        return ret;
    }

    TextureTransient* createTextureTransientHandle(ContextOpt* ctx)
    {
        TextureTransient* ret = ctx->textureTransients.allocateBackPointer();
        ret->textureVirtual = nullptr;
        ret->textureTransientExternal = nullptr;
        ret->aliasFormat = eNvFlowFormat_unknown;
        ret->commandEnd = -1;
        return ret;
    }

    TextureAcquire* createTextureAcquireHandle(ContextOpt* ctx)
    {
        TextureAcquire* ret = ctx->textureAcquires.allocateBackPointer();
        ret->textureVirtual = nullptr;
        return ret;
    }

    NvFlowTextureTransient* importBackendTextureTransient(NvFlowContextOpt* contextOpt, NvFlowTextureTransient* backendTextureTransient)
    {
        auto ctx = cast(contextOpt);

        if (!backendTextureTransient)
        {
            return nullptr;
        }

        auto texture = createTextureTransientHandle(ctx);

        // check for import of an export
        for (NvFlowUint idx = 0u; idx < ctx->exportTextureVirtuals.size; idx++)
        {
            if (ctx->exportTextureVirtuals[idx]->backend->textureTransient == backendTextureTransient)
            {
                addRefTextureVirtual(ctx, ctx->exportTextureVirtuals[idx]);
                texture->textureVirtual = ctx->exportTextureVirtuals[idx];
            }
        }
        if (!texture->textureVirtual)
        {
            texture->textureTransientExternal = backendTextureTransient;
        }
        return cast(texture);
    }

    void exportTextureTransient(NvFlowContextOpt* contextOpt, NvFlowTextureTransient* textureTransient, NvFlowTextureTransient** pBackendTextureTransient)
    {
        auto ctx = cast(contextOpt);

        if (!textureTransient)
        {
            *pBackendTextureTransient = nullptr;
            return;
        }

        struct CreateTextureCmd
        {
            TextureTransient* textureTransient;
            NvFlowTextureTransient** pBackendTextureTransient;
        };
        auto cmd = stringPoolAllocate<CreateTextureCmd>(ctx);

        cmd->textureTransient = cast(textureTransient);
        cmd->pBackendTextureTransient = pBackendTextureTransient;

        accumTextureTransientLifetime(cmd->textureTransient, ctx->commands.size);

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (CreateTextureCmd*)cmdIn;
            if (cmd->textureTransient->textureTransientExternal)
            {
                *cmd->pBackendTextureTransient = cmd->textureTransient->textureTransientExternal;
            }
            else
            {
                *cmd->pBackendTextureTransient = getTextureBackend(ctx, cmd->textureTransient->textureVirtual, cmd->textureTransient->aliasFormat);

                addRefTextureVirtual(ctx, cmd->textureTransient->textureVirtual);
                ctx->exportTextureVirtuals.pushBack(cmd->textureTransient->textureVirtual);
            }
            releaseTextureTransientConditional(ctx, cmd->textureTransient, commandIdx);
        };
        ctx->commands.pushBack(Command{"exportTextureTransient", cmd, execute });
    }

    NvFlowTexture* createTexture(NvFlowContext* context, const NvFlowTextureDesc* desc)
    {
        auto ctx = context_cast(context);

        struct CreateTextureParams
        {
            Texture* texture;
            NvFlowTextureDesc desc;
        };
        auto cmd = stringPoolAllocate<CreateTextureParams>(ctx);

        cmd->texture = createTextureHandle(ctx);
        cmd->desc = *desc;

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (CreateTextureParams*)cmdIn;
            cmd->texture->textureVirtual = getTextureVirtual(ctx, &cmd->desc);
        };
        ctx->commands.pushBack(Command{"createTexture", cmd, execute });

        return cast(cmd->texture);
    }

    void destroyTexture(NvFlowContext* context, NvFlowTexture* texture)
    {
        auto ctx = context_cast(context);

        struct DestroyTextureParams
        {
            Texture* texture;
        };
        auto cmd = stringPoolAllocate<DestroyTextureParams>(ctx);

        cmd->texture = cast(texture);

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (DestroyTextureParams*)cmdIn;
            releaseTextureVirtual(ctx, cmd->texture->textureVirtual);
            cmd->texture->textureVirtual = nullptr;
        };
        ctx->commands.pushBack(Command{"destroyTexture", cmd, execute });
    }

    NvFlowTextureTransient* getTextureTransient(NvFlowContext* context, const NvFlowTextureDesc* desc)
    {
        auto ctx = context_cast(context);

        struct GetTextureTransientParams
        {
            TextureTransient* textureTransient;
            NvFlowTextureDesc desc;
        };
        auto cmd = stringPoolAllocate<GetTextureTransientParams>(ctx);

        cmd->textureTransient = createTextureTransientHandle(ctx);
        cmd->desc = *desc;

        accumTextureTransientLifetime(cmd->textureTransient, ctx->commands.size);

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (GetTextureTransientParams*)cmdIn;
            cmd->textureTransient->textureVirtual = getTextureVirtual(ctx, &cmd->desc);

            releaseTextureTransientConditional(ctx, cmd->textureTransient, commandIdx);
        };
        ctx->commands.pushBack(Command{"getTextureTransient", cmd, execute });

        return cast(cmd->textureTransient);
    }

    NvFlowTextureTransient* registerTextureAsTransient(NvFlowContext* context, NvFlowTexture* texture)
    {
        auto ctx = context_cast(context);

        struct RegisterTextureAsTransientParams
        {
            TextureTransient* textureTransient;
            Texture* texture;
        };
        auto cmd = stringPoolAllocate<RegisterTextureAsTransientParams>(ctx);

        cmd->textureTransient = createTextureTransientHandle(ctx);
        cmd->texture = cast(texture);

        accumTextureTransientLifetime(cmd->textureTransient, ctx->commands.size);

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (RegisterTextureAsTransientParams*)cmdIn;
            cmd->textureTransient->textureVirtual = cmd->texture->textureVirtual;
            addRefTextureVirtual(ctx, cmd->textureTransient->textureVirtual);

            releaseTextureTransientConditional(ctx, cmd->textureTransient, commandIdx);
        };
        ctx->commands.pushBack(Command{"registerTextureAsTransient", cmd, execute });

        return cast(cmd->textureTransient);
    }

    NvFlowTextureTransient* aliasTextureTransient(NvFlowContext* context, NvFlowTextureTransient* texture, NvFlowFormat format)
    {
        auto ctx = context_cast(context);

        struct AliasTextureTransientParams
        {
            TextureTransient* textureDst;
            TextureTransient* textureSrc;
            NvFlowFormat format;
        };
        auto cmd = stringPoolAllocate<AliasTextureTransientParams>(ctx);

        cmd->textureDst = createTextureTransientHandle(ctx);
        cmd->textureSrc = cast(texture);
        cmd->format = format;

        accumTextureTransientLifetime(cmd->textureDst, ctx->commands.size);
        accumTextureTransientLifetime(cmd->textureSrc, ctx->commands.size);

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (AliasTextureTransientParams*)cmdIn;
            cmd->textureDst->textureVirtual = cmd->textureSrc->textureVirtual;
            addRefTextureVirtual(ctx, cmd->textureDst->textureVirtual);

            cmd->textureDst->aliasFormat = cmd->format;

            releaseTextureTransientConditional(ctx, cmd->textureDst, commandIdx);
            releaseTextureTransientConditional(ctx, cmd->textureSrc, commandIdx);
        };
        ctx->commands.pushBack(Command{ "aliasTextureTransient", cmd, execute });

        return cast(cmd->textureDst);
    }

    NvFlowTextureAcquire* enqueueAcquireTexture(NvFlowContext* context, NvFlowTextureTransient* texture)
    {
        auto ctx = context_cast(context);

        struct EnqueueAcquireTextureParams
        {
            TextureAcquire* textureAcquire;
            TextureTransient* textureTransient;
        };
        auto cmd = stringPoolAllocate<EnqueueAcquireTextureParams>(ctx);

        cmd->textureAcquire = createTextureAcquireHandle(ctx);
        cmd->textureTransient = cast(texture);

        accumTextureTransientLifetime(cmd->textureTransient, ctx->commands.size);

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (EnqueueAcquireTextureParams*)cmdIn;
            cmd->textureAcquire->textureVirtual = cmd->textureTransient->textureVirtual;
            addRefTextureVirtual(ctx, cmd->textureAcquire->textureVirtual);
            releaseTextureTransientConditional(ctx, cmd->textureTransient, commandIdx);
        };
        ctx->commands.pushBack(Command{"enqueueAcquireTexture", cmd, execute });

        return cast(cmd->textureAcquire);
    }

    NvFlowBool32 getAcquiredTexture(NvFlowContext* context, NvFlowTextureAcquire* acquire, NvFlowTexture** outTexture)
    {
        auto ctx = context_cast(context);

        struct GetAcquiredTextureParams
        {
            TextureAcquire* textureAcquire;
            Texture* texture;
        };
        auto cmd = stringPoolAllocate<GetAcquiredTextureParams>(ctx);

        cmd->textureAcquire = cast(acquire);
        cmd->texture = createTextureHandle(ctx);

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (GetAcquiredTextureParams*)cmdIn;
            cmd->texture->textureVirtual = cmd->textureAcquire->textureVirtual;
            cmd->textureAcquire->textureVirtual = nullptr;
        };
        ctx->commands.pushBack(Command{"getAcquiredTexture", cmd, execute });

        *outTexture = cast(cmd->texture);

        return NV_FLOW_TRUE;
    }

    NvFlowTextureTransient* getTextureTransientById(NvFlowContext* context, NvFlowUint64 textureId)
    {
        auto ctx = context_cast(context);

        NvFlowTextureTransient* backendTexture = ctx->backendContextInterface.getTextureTransientById(ctx->backendContext, textureId);

        return importBackendTextureTransient(cast(ctx), backendTexture);
    }


    Sampler* createSamplerHandle(ContextOpt* ctx)
    {
        Sampler* ret = ctx->samplers.allocateBackPointer();
        ret->backend = nullptr;
        return ret;
    }

    NvFlowSampler* createSampler(NvFlowContext* context, const NvFlowSamplerDesc* desc)
    {
        auto ctx = context_cast(context);

        struct CreateSamplerParams
        {
            Sampler* sampler;
            NvFlowSamplerDesc desc;
        };
        auto cmd = stringPoolAllocate<CreateSamplerParams>(ctx);

        cmd->sampler = createSamplerHandle(ctx);
        cmd->desc = *desc;

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (CreateSamplerParams*)cmdIn;
            cmd->sampler->backend = getSamplerBackend(ctx, &cmd->desc);
        };
        ctx->commands.pushBack(Command{"createSampler", cmd, execute });

        return cast(cmd->sampler);
    }

    NvFlowSampler* getDefaultSampler(NvFlowContext* context)
    {
        auto ctx = context_cast(context);

        if (!ctx->defaultSampler)
        {
            NvFlowSamplerDesc samplerDesc = {};
            samplerDesc.addressModeU = eNvFlowSamplerAddressMode_border;
            samplerDesc.addressModeV = eNvFlowSamplerAddressMode_border;
            samplerDesc.addressModeW = eNvFlowSamplerAddressMode_border;
            samplerDesc.filterMode = eNvFlowSamplerFilterMode_point;

            ctx->defaultSampler = createSampler(context, &samplerDesc);
        }

        return ctx->defaultSampler;
    }

    void destroySampler(NvFlowContext* context, NvFlowSampler* sampler)
    {
        auto ctx = context_cast(context);

        struct DestroySamplerParams
        {
            Sampler* sampler;
        };
        auto cmd = stringPoolAllocate<DestroySamplerParams>(ctx);

        cmd->sampler = cast(sampler);

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (DestroySamplerParams*)cmdIn;
            cmd->sampler->backend->refCount--;
            cmd->sampler->backend = nullptr;
        };
        ctx->commands.pushBack(Command{"destroySampler", cmd, execute });
    }


    ComputePipeline* createComputePipelineHandle(ContextOpt* ctx)
    {
        ComputePipeline* ret = ctx->computePipelines.allocateBackPointer();
        ret->backend = nullptr;
        return ret;
    }

    NvFlowComputePipeline* createComputePipeline(NvFlowContext* context, const NvFlowComputePipelineDesc* desc)
    {
        auto ctx = context_cast(context);

        struct CreateComputePipelineParams
        {
            ComputePipeline* computePipeline;
            NvFlowComputePipelineDesc desc;
        };
        auto cmd = stringPoolAllocate<CreateComputePipelineParams>(ctx);

        cmd->computePipeline = createComputePipelineHandle(ctx);
        cmd->desc = *desc;

        cmd->desc.bindingDescs = stringPoolAllocateN<NvFlowBindingDesc>(ctx, cmd->desc.numBindingDescs);
        for (NvFlowUint idx = 0u; idx < desc->numBindingDescs; idx++)
        {
            cmd->desc.bindingDescs[idx] = desc->bindingDescs[idx];
        }

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (CreateComputePipelineParams*)cmdIn;
            cmd->computePipeline->backend = getComputePipelineBackend(ctx, &cmd->desc);
        };
        ctx->commands.pushBack(Command{"createComputePipeline", cmd, execute });

        return cast(cmd->computePipeline);
    }

    void destroyComputePipeline(NvFlowContext* context, NvFlowComputePipeline* computePipeline)
    {
        auto ctx = context_cast(context);

        struct DestroyComputePipelineParams
        {
            ComputePipeline* computePipeline;
        };
        auto cmd = stringPoolAllocate<DestroyComputePipelineParams>(ctx);

        cmd->computePipeline = cast(computePipeline);

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (DestroyComputePipelineParams*)cmdIn;
            cmd->computePipeline->backend = nullptr;
        };
        ctx->commands.pushBack(Command{"destroyComputePipeline", cmd, execute });
    }


    void addPassCompute(NvFlowContext* context, const NvFlowPassComputeParams* paramsIn)
    {
        auto ctx = context_cast(context);

        struct AddPassComputeCmd
        {
            NvFlowPassComputeParams params;
        };
        auto cmd = stringPoolAllocate<AddPassComputeCmd>(ctx);

        cmd->params = *paramsIn;

        auto descriptorWrites = stringPoolAllocateN<NvFlowDescriptorWrite>(ctx, paramsIn->numDescriptorWrites);
        for (NvFlowUint idx = 0u; idx < paramsIn->numDescriptorWrites; idx++)
        {
            descriptorWrites[idx] = paramsIn->descriptorWrites[idx];
        }
        cmd->params.descriptorWrites = descriptorWrites;

        auto resources = stringPoolAllocateN<NvFlowResource>(ctx, paramsIn->numDescriptorWrites);
        for (NvFlowUint idx = 0u; idx < paramsIn->numDescriptorWrites; idx++)
        {
            resources[idx] = paramsIn->resources[idx];
        }
        cmd->params.resources = resources;

        for (NvFlowUint descriptorIdx = 0u; descriptorIdx < cmd->params.numDescriptorWrites; descriptorIdx++)
        {
            auto bufferTransient = cast(cmd->params.resources[descriptorIdx].bufferTransient);
            auto textureTransient = cast(cmd->params.resources[descriptorIdx].textureTransient);
            if (bufferTransient)
            {
                accumBufferTransientLifetime(bufferTransient, ctx->commands.size);
            }
            if (textureTransient)
            {
                accumTextureTransientLifetime(textureTransient, ctx->commands.size);
            }
        }

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (AddPassComputeCmd*)cmdIn;

            auto resourceTemp = stringPoolAllocateN<NvFlowResource>(ctx, cmd->params.numDescriptorWrites);

            for (NvFlowUint descriptorIdx = 0u; descriptorIdx < cmd->params.numDescriptorWrites; descriptorIdx++)
            {
                const NvFlowResource* srcResource = &cmd->params.resources[descriptorIdx];
                NvFlowResource dstResource = {};
                if (srcResource->bufferTransient)
                {
                    BufferTransient* bufferTransient = cast(srcResource->bufferTransient);
                    if (bufferTransient->bufferTransientExternal)
                    {
                        dstResource.bufferTransient = bufferTransient->bufferTransientExternal;
                    }
                    else
                    {
                        dstResource.bufferTransient = getBufferBackend(ctx, bufferTransient->bufferVirtual,
                            bufferTransient->aliasFormat, bufferTransient->aliasStructureStride);
                    }
                }
                if (srcResource->textureTransient)
                {
                    TextureTransient* textureTransient = cast(srcResource->textureTransient);
                    if (textureTransient->textureTransientExternal)
                    {
                        dstResource.textureTransient = textureTransient->textureTransientExternal;
                    }
                    else
                    {
                        dstResource.textureTransient = getTextureBackend(ctx, textureTransient->textureVirtual, textureTransient->aliasFormat);
                    }
                }
                if (srcResource->sampler)
                {
                    dstResource.sampler = cast(srcResource->sampler)->backend->sampler;
                }
                resourceTemp[descriptorIdx] = dstResource;
            }
            auto paramsBackend = cmd->params;
            paramsBackend.pipeline = cast(cmd->params.pipeline)->backend->pipeline;
            paramsBackend.resources = resourceTemp;
            ctx->backendContextInterface.addPassCompute(ctx->backendContext, &paramsBackend);
            for (NvFlowUint descriptorIdx = 0u; descriptorIdx < cmd->params.numDescriptorWrites; descriptorIdx++)
            {
                const NvFlowResource* srcResource = &cmd->params.resources[descriptorIdx];
                if (srcResource->bufferTransient)
                {
                    releaseBufferTransientConditional(ctx, cast(srcResource->bufferTransient), commandIdx);
                }
                if (srcResource->textureTransient)
                {
                    releaseTextureTransientConditional(ctx, cast(srcResource->textureTransient), commandIdx);
                }
            }
        };
        ctx->commands.pushBack(Command{"addPassCompute", cmd, execute });
    }

    void addPassCopyBuffer(NvFlowContext* context, const NvFlowPassCopyBufferParams* paramsIn)
    {
        auto ctx = context_cast(context);

        struct AddPassCopyBufferParams
        {
            NvFlowPassCopyBufferParams params;
        };
        auto cmd = stringPoolAllocate<AddPassCopyBufferParams>(ctx);

        cmd->params = *paramsIn;

        accumBufferTransientLifetime(cast(cmd->params.src), ctx->commands.size);
        accumBufferTransientLifetime(cast(cmd->params.dst), ctx->commands.size);

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (AddPassCopyBufferParams*)cmdIn;
            auto paramsBackend = cmd->params;
            if (cast(cmd->params.src)->bufferTransientExternal)
            {
                paramsBackend.src = cast(cmd->params.src)->bufferTransientExternal;
            }
            else
            {
                paramsBackend.src = getBufferBackend(ctx, cast(cmd->params.src)->bufferVirtual,
                    cast(cmd->params.src)->aliasFormat, cast(cmd->params.src)->aliasStructureStride);
            }
            if (cast(cmd->params.dst)->bufferTransientExternal)
            {
                paramsBackend.dst = cast(cmd->params.dst)->bufferTransientExternal;
            }
            else
            {
                paramsBackend.dst = getBufferBackend(ctx, cast(cmd->params.dst)->bufferVirtual,
                    cast(cmd->params.dst)->aliasFormat, cast(cmd->params.dst)->aliasStructureStride);
            }
            ctx->backendContextInterface.addPassCopyBuffer(ctx->backendContext, &paramsBackend);
            releaseBufferTransientConditional(ctx, cast(cmd->params.src), commandIdx);
            releaseBufferTransientConditional(ctx, cast(cmd->params.dst), commandIdx);
        };
        ctx->commands.pushBack(Command{"addPassCopyBuffer", cmd, execute });
    }

    void addPassCopyBufferToTexture(NvFlowContext* context, const NvFlowPassCopyBufferToTextureParams* paramsIn)
    {
        auto ctx = context_cast(context);

        struct AddPassCopyBufferToTextureParams
        {
            NvFlowPassCopyBufferToTextureParams params;
        };
        auto cmd = stringPoolAllocate<AddPassCopyBufferToTextureParams>(ctx);

        cmd->params = *paramsIn;

        accumBufferTransientLifetime(cast(cmd->params.src), ctx->commands.size);
        accumTextureTransientLifetime(cast(cmd->params.dst), ctx->commands.size);

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (AddPassCopyBufferToTextureParams*)cmdIn;
            auto paramsBackend = cmd->params;
            if (cast(cmd->params.src)->bufferTransientExternal)
            {
                paramsBackend.src = cast(cmd->params.src)->bufferTransientExternal;
            }
            else
            {
                paramsBackend.src = getBufferBackend(ctx, cast(cmd->params.src)->bufferVirtual,
                    cast(cmd->params.src)->aliasFormat, cast(cmd->params.src)->aliasStructureStride);
            }
            if (cast(cmd->params.dst)->textureTransientExternal)
            {
                paramsBackend.dst = cast(cmd->params.dst)->textureTransientExternal;
            }
            else
            {
                paramsBackend.dst = getTextureBackend(ctx, cast(cmd->params.dst)->textureVirtual, cast(cmd->params.dst)->aliasFormat);
            }
            ctx->backendContextInterface.addPassCopyBufferToTexture(ctx->backendContext, &paramsBackend);
            releaseBufferTransientConditional(ctx, cast(cmd->params.src), commandIdx);
            releaseTextureTransientConditional(ctx, cast(cmd->params.dst), commandIdx);
        };
        ctx->commands.pushBack(Command{"addPassCopyBufferToTexture", cmd, execute });
    }

    void addPassCopyTextureToBuffer(NvFlowContext* context, const NvFlowPassCopyTextureToBufferParams* paramsIn)
    {
        auto ctx = context_cast(context);

        struct AddPassCopyTextureToBufferParams
        {
            NvFlowPassCopyTextureToBufferParams params;
        };
        auto cmd = stringPoolAllocate<AddPassCopyTextureToBufferParams>(ctx);

        cmd->params = *paramsIn;

        accumTextureTransientLifetime(cast(cmd->params.src), ctx->commands.size);
        accumBufferTransientLifetime(cast(cmd->params.dst), ctx->commands.size);

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (AddPassCopyTextureToBufferParams*)cmdIn;
            auto paramsBackend = cmd->params;
            if (cast(cmd->params.src)->textureTransientExternal)
            {
                paramsBackend.src = cast(cmd->params.src)->textureTransientExternal;
            }
            else
            {
                paramsBackend.src = getTextureBackend(ctx, cast(cmd->params.src)->textureVirtual, cast(cmd->params.src)->aliasFormat);
            }
            if (cast(cmd->params.dst)->bufferTransientExternal)
            {
                paramsBackend.dst = cast(cmd->params.dst)->bufferTransientExternal;
            }
            else
            {
                paramsBackend.dst = getBufferBackend(ctx, cast(cmd->params.dst)->bufferVirtual,
                    cast(cmd->params.dst)->aliasFormat, cast(cmd->params.dst)->aliasStructureStride);
            }
            ctx->backendContextInterface.addPassCopyTextureToBuffer(ctx->backendContext, &paramsBackend);
            releaseTextureTransientConditional(ctx, cast(cmd->params.src), commandIdx);
            releaseBufferTransientConditional(ctx, cast(cmd->params.dst), commandIdx);
        };
        ctx->commands.pushBack(Command{"addPassCopyTextureToBuffer", cmd, execute });
    }

    void addPassCopyTexture(NvFlowContext* context, const NvFlowPassCopyTextureParams* paramsIn)
    {
        auto ctx = context_cast(context);

        struct AddPassCopyTextureParams
        {
            NvFlowPassCopyTextureParams params;
        };
        auto cmd = stringPoolAllocate<AddPassCopyTextureParams>(ctx);

        cmd->params = *paramsIn;

        accumTextureTransientLifetime(cast(cmd->params.src), ctx->commands.size);
        accumTextureTransientLifetime(cast(cmd->params.dst), ctx->commands.size);

        auto execute = [](ContextOpt* ctx, NvFlowUint commandIdx, void* cmdIn)
        {
            auto cmd = (AddPassCopyTextureParams*)cmdIn;
            auto paramsBackend = cmd->params;
            if (cast(cmd->params.src)->textureTransientExternal)
            {
                paramsBackend.src = cast(cmd->params.src)->textureTransientExternal;
            }
            else
            {
                paramsBackend.src = getTextureBackend(ctx, cast(cmd->params.src)->textureVirtual, cast(cmd->params.src)->aliasFormat);
            }
            if (cast(cmd->params.dst)->textureTransientExternal)
            {
                paramsBackend.dst = cast(cmd->params.dst)->textureTransientExternal;
            }
            else
            {
                paramsBackend.dst = getTextureBackend(ctx, cast(cmd->params.dst)->textureVirtual, cast(cmd->params.dst)->aliasFormat);
            }
            ctx->backendContextInterface.addPassCopyTexture(ctx->backendContext, &paramsBackend);
            releaseTextureTransientConditional(ctx, cast(cmd->params.src), commandIdx);
            releaseTextureTransientConditional(ctx, cast(cmd->params.dst), commandIdx);
        };
        ctx->commands.pushBack(Command{"addPassCopyTexture", cmd, execute });
    }

    void setResourceMinLifetime(NvFlowContextOpt* contextOpt, NvFlowUint64 minLifetime)
    {
        auto ctx = cast(contextOpt);

        ctx->minLifetime = minLifetime;
    }

    NvFlowContextInterface* getContextInterfaceOpt()
    {
        static NvFlowContextInterface iface = { NV_FLOW_REFLECT_INTERFACE_INIT(NvFlowContextInterface) };

        iface.getContextConfig = getContextConfig;
        iface.isFeatureSupported = isFeatureSupported;
        iface.getCurrentFrame = getCurrentFrame;
        iface.getLastFrameCompleted = getLastFrameCompleted;
        iface.getCurrentGlobalFrame = getCurrentGlobalFrame;
        iface.getLastGlobalFrameCompleted = getLastGlobalFrameCompleted;
        iface.getLogPrint = getLogPrint;

        iface.executeTasks = executeTasks;

        iface.createBuffer = createBuffer;
        iface.destroyBuffer = destroyBuffer;
        iface.getBufferTransient = getBufferTransient;
        iface.registerBufferAsTransient = registerBufferAsTransient;
        iface.aliasBufferTransient = aliasBufferTransient;
        iface.enqueueAcquireBuffer = enqueueAcquireBuffer;
        iface.getAcquiredBuffer = getAcquiredBuffer;
        iface.mapBuffer = mapBuffer;
        iface.unmapBuffer = unmapBuffer;
        iface.getBufferTransientById = getBufferTransientById;
        iface.getBufferExternalHandle = getBufferExternalHandle;
        iface.closeBufferExternalHandle = closeBufferExternalHandle;
        iface.createBufferFromExternalHandle = createBufferFromExternalHandle;

        iface.createTexture = createTexture;
        iface.destroyTexture = destroyTexture;
        iface.getTextureTransient = getTextureTransient;
        iface.registerTextureAsTransient = registerTextureAsTransient;
        iface.aliasTextureTransient = aliasTextureTransient;
        iface.enqueueAcquireTexture = enqueueAcquireTexture;
        iface.getAcquiredTexture = getAcquiredTexture;
        iface.getTextureTransientById = getTextureTransientById;

        iface.createSampler = createSampler;
        iface.getDefaultSampler = getDefaultSampler;
        iface.destroySampler = destroySampler;

        iface.createComputePipeline = createComputePipeline;
        iface.destroyComputePipeline = destroyComputePipeline;

        iface.addPassCompute = addPassCompute;
        iface.addPassCopyBuffer = addPassCopyBuffer;
        iface.addPassCopyBufferToTexture = addPassCopyBufferToTexture;
        iface.addPassCopyTextureToBuffer = addPassCopyTextureToBuffer;
        iface.addPassCopyTexture = addPassCopyTexture;

        return &iface;
    }
}

NvFlowContextOptInterface* NvFlowGetContextOptInterface()
{
    static NvFlowContextOptInterface iface = { NV_FLOW_REFLECT_INTERFACE_INIT(NvFlowContextOptInterface) };

    iface.create = create;
    iface.destroy = destroy;
    iface.getContext = getContext;
    iface.flush = flush;
    iface.importBackendBufferTransient = importBackendBufferTransient;
    iface.importBackendTextureTransient = importBackendTextureTransient;
    iface.exportBufferTransient = exportBufferTransient;
    iface.exportTextureTransient = exportTextureTransient;
    iface.setResourceMinLifetime = setResourceMinLifetime;

    return &iface;
}
