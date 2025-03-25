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

NvFlowUint texture_getFormatSizeInBytes(NvFlowFormat format)
{
    switch (format)
    {
    case eNvFlowFormat_unknown:
        return 0u;
    case eNvFlowFormat_r32g32b32a32_float:
    case eNvFlowFormat_r32g32b32a32_uint:
    case eNvFlowFormat_r32g32b32a32_sint:
        return sizeof(NvFlowFloat4);
    case eNvFlowFormat_r32g32b32_float:
    case eNvFlowFormat_r32g32b32_uint:
    case eNvFlowFormat_r32g32b32_sint:
        return sizeof(NvFlowFloat3);
    case eNvFlowFormat_r16g16b16a16_float:
    case eNvFlowFormat_r16g16b16a16_unorm:
    case eNvFlowFormat_r16g16b16a16_uint:
    case eNvFlowFormat_r16g16b16a16_snorm:
    case eNvFlowFormat_r16g16b16a16_sint:
        return sizeof(NvFlowFloat4);                // Note: FP32 for now
    case eNvFlowFormat_r32g32_float:
    case eNvFlowFormat_r32g32_uint:
    case eNvFlowFormat_r32g32_sint:
        return sizeof(NvFlowFloat2);
    case eNvFlowFormat_r10g10b10a2_unorm:
    case eNvFlowFormat_r10g10b10a2_uint:
    case eNvFlowFormat_r11g11b10_float:
        return sizeof(NvFlowFloat4);                // Note: FP32 for now
    case eNvFlowFormat_r8g8b8a8_unorm:
    case eNvFlowFormat_r8g8b8a8_unorm_srgb:
    case eNvFlowFormat_r8g8b8a8_uint:
    case eNvFlowFormat_r8g8b8a8_snorm:
    case eNvFlowFormat_r8g8b8a8_sint:
        return sizeof(NvFlowFloat4);                // Note: FP32 for now
    case eNvFlowFormat_r16g16_float:
    case eNvFlowFormat_r16g16_unorm:
    case eNvFlowFormat_r16g16_uint:
    case eNvFlowFormat_r16g16_snorm:
    case eNvFlowFormat_r16g16_sint:
        return sizeof(NvFlowFloat2);                // Note: FP32 for now
    case eNvFlowFormat_r32_float:
    case eNvFlowFormat_r32_uint:
    case eNvFlowFormat_r32_sint:
        return sizeof(float);
    case eNvFlowFormat_r8g8_unorm:
    case eNvFlowFormat_r8g8_uint:
    case eNvFlowFormat_r8g8_snorm:
    case eNvFlowFormat_r8g8_sint:
        return sizeof(NvFlowFloat2);                // Note: FP32 for now
    case eNvFlowFormat_r16_float:
    case eNvFlowFormat_r16_unorm:
    case eNvFlowFormat_r16_uint:
    case eNvFlowFormat_r16_snorm:
    case eNvFlowFormat_r16_sint:
        return sizeof(float);                        // Note: FP32 for now
    case eNvFlowFormat_r8_unorm:
    case eNvFlowFormat_r8_uint:
    case eNvFlowFormat_r8_snorm:
    case eNvFlowFormat_r8_sint:
        return sizeof(float);                        // Note: FP32 for now
    case eNvFlowFormat_b8g8r8a8_unorm:
    case eNvFlowFormat_b8g8r8a8_unorm_srgb:
        return sizeof(NvFlowFloat4);                // Note: FP32 for now
    default:
        return 0u;
    }
}

void texture_descClamping(Texture* ptr)
{
    if (ptr->desc.mipLevels == 0u)
    {
        ptr->desc.mipLevels = 1u;
    }

    if (ptr->desc.textureType == eNvFlowTextureType_1d)
    {
        ptr->desc.height = 1u;
        ptr->desc.depth = 1u;
    }
    else if (ptr->desc.textureType == eNvFlowTextureType_2d)
    {
        ptr->desc.depth = 1u;
    }
}

Texture* texture_create(Context* context, const NvFlowTextureDesc* desc)
{
    auto ptr = new Texture();

    ptr->desc = *desc;

    texture_descClamping(ptr);

    NvFlowUint64 formatNumBytes = texture_getFormatSizeInBytes(ptr->desc.format);
    NvFlowUint64 numBytes = ptr->desc.width * ptr->desc.height * ptr->desc.depth * formatNumBytes;

    ptr->resource.data = malloc(numBytes);
    ptr->resource.sizeInBytes = numBytes;
    ptr->resource.elementSizeInBytes = NvFlowUint(formatNumBytes);
    ptr->resource.elementCount = NvFlowUint(numBytes / formatNumBytes);
    ptr->resource.format = ptr->desc.format;
    ptr->resource.width = ptr->desc.width;
    ptr->resource.height = ptr->desc.height;
    ptr->resource.depth = ptr->desc.depth;

    return ptr;
}

void texture_destroy(Context* context, Texture* ptr)
{
    free(ptr->resource.data);
    ptr->resource.data = nullptr;

    delete ptr;
}

NvFlowBool32 textureDesc_compare(const NvFlowTextureDesc* a, const NvFlowTextureDesc* b)
{
    if (a->textureType == b->textureType &&
        a->usageFlags == b->usageFlags &&
        a->format == b->format &&
        a->width == b->width &&
        a->height == b->height &&
        a->depth == b->depth &&
        a->mipLevels == b->mipLevels &&
        a->optimizedClearValue.x == b->optimizedClearValue.x &&
        a->optimizedClearValue.y == b->optimizedClearValue.y &&
        a->optimizedClearValue.z == b->optimizedClearValue.z &&
        a->optimizedClearValue.w == b->optimizedClearValue.w )
    {
        return NV_FLOW_TRUE;
    }
    return NV_FLOW_FALSE;
}

NvFlowTexture* createTexture(NvFlowContext* contextIn, const NvFlowTextureDesc* desc)
{
    auto context = cast(contextIn);

    for (NvFlowUint idx = 0u; idx < context->pool_textures.size; idx++)
    {
        auto ptr = context->pool_textures[idx];
        if (ptr && !ptr->activeMask && textureDesc_compare(&ptr->desc, desc))
        {
            ptr->activeMask = 1u;
            return cast(ptr);
        }
    }

    auto ptr = texture_create(context, desc);

    ptr->activeMask = 1u;
    context->pool_textures.pushBack(ptr);

    return cast(ptr);
}

void destroyTexture(NvFlowContext* contextIn, NvFlowTexture* texture)
{
    auto context = cast(contextIn);
    auto ptr = cast(texture);

    if (!ptr->activeMask)
    {
        context->logPrint(eNvFlowLogLevel_error, "NvFlowContext::destroyTexture() called on inactive texture %p", texture);
    }

    ptr->activeMask &= ~1u;
}

void context_destroyTextures(Context* context)
{
    context->textureTransients.deletePointers();
    context->textureAcquires.deletePointers();

    for (NvFlowUint idx = 0u; idx < context->pool_textures.size; idx++)
    {
        auto& ptr = context->pool_textures[idx];
        texture_destroy(context, ptr);
        ptr = nullptr;
    }
}

NvFlowTextureTransient* getTextureTransient(NvFlowContext* contextIn, const NvFlowTextureDesc* desc)
{
    auto context = cast(contextIn);
    auto ptr = context->textureTransients.allocateBackPointer();
    ptr->desc = *desc;
    ptr->texture = cast(createTexture(contextIn, &ptr->desc));
    ptr->texture->activeMask = 2u;
    return cast(ptr);
}

NvFlowTextureTransient* registerTextureAsTransient(NvFlowContext* contextIn, NvFlowTexture* texture)
{
    auto context = cast(contextIn);
    auto ptr = context->textureTransients.allocateBackPointer();
    ptr->desc = cast(texture)->desc;
    ptr->texture = cast(texture);
    ptr->texture->activeMask |= 2u;
    return cast(ptr);
}

NvFlowTextureAcquire* enqueueAcquireTexture(NvFlowContext* contextIn, NvFlowTextureTransient* textureTransient)
{
    auto context = cast(contextIn);
    auto ptr = context->textureAcquires.allocateBackPointer();
    ptr->textureTransient = cast(textureTransient);
    ptr->texture = nullptr;
    return cast(ptr);
}

NvFlowBool32 getAcquiredTexture(NvFlowContext* contextIn, NvFlowTextureAcquire* acquire, NvFlowTexture** outTexture)
{
    auto context = cast(contextIn);
    auto ptr = cast(acquire);
    if (ptr->texture)
    {
        *outTexture = cast(ptr->texture);

        // remove from acquire array
        context->textureAcquires.removeSwapPointer(ptr);

        return NV_FLOW_TRUE;
    }
    return NV_FLOW_FALSE;
}

NvFlowTextureTransient* getTextureTransientById(NvFlowContext* context, NvFlowUint64 textureId)
{
    auto ctx = cast(context);
    for (NvFlowUint idx = 0u; idx < ctx->registeredResources.size; idx++)
    {
        if (ctx->registeredResources[idx].uid == textureId)
        {
            if (ctx->registeredResources[idx].texture)
            {
                return registerTextureAsTransient(context, ctx->registeredResources[idx].texture);
            }
        }
    }
    return nullptr;
}

/// ***************************** Samplers ********************************************

Sampler* sampler_create(Context* context, const NvFlowSamplerDesc* desc)
{
    auto ptr = new Sampler();

    ptr->desc = *desc;
    ptr->resource.samplerDesc = ptr->desc;

    return ptr;
}

void sampler_destroy(Context* context, Sampler* ptr)
{
    delete ptr;
}

NvFlowSampler* createSampler(NvFlowContext* contextIn, const NvFlowSamplerDesc* desc)
{
    auto context = cast(contextIn);
    auto ptr = sampler_create(context, desc);

    ptr->isActive = NV_FLOW_TRUE;
    context->pool_samplers.pushBack(ptr);

    return cast(ptr);
}

NvFlowSampler* getDefaultSampler(NvFlowContext* contextIn)
{
    auto context = cast(contextIn);
    return cast(context->pool_samplers[0u]);
}

void destroySampler(NvFlowContext* contextIn, NvFlowSampler* sampler)
{
    auto context = cast(contextIn);
    auto ptr = cast(sampler);

    ptr->isActive = NV_FLOW_FALSE;
}

void context_destroySamplers(Context* context)
{
    for (NvFlowUint idx = 0u; idx < context->pool_samplers.size; idx++)
    {
        auto& ptr = context->pool_samplers[idx];
        sampler_destroy(context, ptr);
        ptr = nullptr;
    }
}

} // end namespace
