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

#include "imgui.h"
#include "NvFlowLoader.h"

#include "ImguiRenderer.h"

#include "NvFlowUploadBuffer.h"
#include "NvFlowDynamicBuffer.h"

#include "shaders/ImguiParams.h"
#include "shaders/ImguiCS.hlsl.h"
#include "shaders/ImguiBuildCS.hlsl.h"
#include "shaders/ImguiTileCS.hlsl.h"
#include "shaders/ImguiTileCountCS.hlsl.h"
#include "shaders/ImguiTileScanCS.hlsl.h"
#include "shaders/ImguiTextureUploadCS.hlsl.h"

namespace NvFlowImguiRendererDefault
{
    struct TextureTile
    {
        NvFlowUint8 data[18u * 18u * 4u];
    };

    struct Texture
    {
        NvFlowArray<TextureTile> tiles;
        int tileGridWidth;
        int tileGridHeight;
        int texWidth;
        int texHeight;
        NvFlowUint textureId;
    };

    NV_FLOW_CAST_PAIR(NvFlowImguiTexture, Texture)

    struct Renderer
    {
        NvFlowContextInterface contextInterface = {};

        ImguiCS_Pipeline imguiCS = {};
        ImguiBuildCS_Pipeline imguiBuildCS = {};
        ImguiTileCS_Pipeline imguiTileCS = {};
        ImguiTileCountCS_Pipeline imguiTileCountCS = {};
        ImguiTileScanCS_Pipeline imguiTileScanCS = {};
        ImguiTextureUploadCS_Pipeline imguiTextureUploadCS = {};

        NvFlowUploadBuffer vertexPosTexCoordBuffer = {};
        NvFlowUploadBuffer vertexColorBuffer = {};
        NvFlowUploadBuffer indicesBuffer = {};
        NvFlowUploadBuffer drawCmdsBuffer = {};
        NvFlowUploadBuffer constantBuffer = {};

        NvFlowUploadBuffer textureUpload = {};
        NvFlowUploadBuffer textureTableUpload = {};
        NvFlowTexture* textureDevice = nullptr;
        NvFlowSampler* samplerLinear = nullptr;
        NvFlowUint textureWidth = 0u;
        NvFlowUint textureHeight = 0u;

        NvFlowDynamicBuffer treeBuffer = {};
        NvFlowDynamicBuffer tileCountBuffer = {};
        NvFlowDynamicBuffer triangleBuffer = {};
        NvFlowDynamicBuffer triangleRangeBuffer = {};

        NvFlowBuffer* totalCountBuffer = nullptr;

        NvFlowArrayPointer<Texture*> textures;
        NvFlowUint textureIdCounter = 0u;
        bool textureDirty = true;

        NvFlowArray<NvFlowUint> textureTable;
        NvFlowArray<NvFlowUint> textureData;
    };

    NV_FLOW_CAST_PAIR(NvFlowImguiRenderer, Renderer)

    NvFlowImguiTexture* createTexture(
        NvFlowContext* context,
        NvFlowImguiRenderer* renderer,
        unsigned char* pixels,
        int texWidth,
        int texHeight
    );

    NvFlowImguiRenderer* create(
        NvFlowContextInterface* contextInterface,
        NvFlowContext* context,
        unsigned char* pixels,
        int texWidth,
        int texHeight
    )
    {
        auto ptr = new Renderer();

        NvFlowContextInterface_duplicate(&ptr->contextInterface, contextInterface);

        ImguiCS_init(&ptr->contextInterface, context, &ptr->imguiCS);
        ImguiBuildCS_init(&ptr->contextInterface, context, &ptr->imguiBuildCS);
        ImguiTileCS_init(&ptr->contextInterface, context, &ptr->imguiTileCS);
        ImguiTileCountCS_init(&ptr->contextInterface, context, &ptr->imguiTileCountCS);
        ImguiTileScanCS_init(&ptr->contextInterface, context, &ptr->imguiTileScanCS);
        ImguiTextureUploadCS_init(&ptr->contextInterface, context, &ptr->imguiTextureUploadCS);

        NvFlowBufferUsageFlags bufferUsage = eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_bufferCopySrc;

        NvFlowUploadBuffer_init(&ptr->contextInterface, context, &ptr->vertexPosTexCoordBuffer, bufferUsage, eNvFlowFormat_unknown, sizeof(NvFlowFloat4));
        NvFlowUploadBuffer_init(&ptr->contextInterface, context, &ptr->vertexColorBuffer, bufferUsage, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowUploadBuffer_init(&ptr->contextInterface, context, &ptr->indicesBuffer, bufferUsage, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowUploadBuffer_init(&ptr->contextInterface, context, &ptr->drawCmdsBuffer, bufferUsage, eNvFlowFormat_unknown, sizeof(ImguiRendererDrawCmd));
        NvFlowUploadBuffer_init(&ptr->contextInterface, context, &ptr->constantBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);

        NvFlowUploadBuffer_init(&ptr->contextInterface, context, &ptr->textureUpload, eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_bufferCopySrc, eNvFlowFormat_unknown, 4u);
        NvFlowUploadBuffer_init(&ptr->contextInterface, context, &ptr->textureTableUpload, eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_bufferCopySrc, eNvFlowFormat_unknown, 4u);

        NvFlowSamplerDesc samplerDesc = {};
        samplerDesc.filterMode = eNvFlowSamplerFilterMode_linear;
        samplerDesc.addressModeU = eNvFlowSamplerAddressMode_wrap;
        samplerDesc.addressModeV = eNvFlowSamplerAddressMode_wrap;
        samplerDesc.addressModeW = eNvFlowSamplerAddressMode_wrap;

        ptr->samplerLinear = ptr->contextInterface.createSampler(context, &samplerDesc);

        NvFlowBufferUsageFlags deviceBufUsage = eNvFlowBufferUsage_rwStructuredBuffer | eNvFlowBufferUsage_structuredBuffer;

        NvFlowDynamicBuffer_init(&ptr->contextInterface, context, &ptr->treeBuffer, deviceBufUsage, eNvFlowFormat_unknown, sizeof(NvFlowInt4));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, context, &ptr->tileCountBuffer, deviceBufUsage, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, context, &ptr->triangleBuffer, deviceBufUsage, eNvFlowFormat_unknown, sizeof(NvFlowUint));
        NvFlowDynamicBuffer_init(&ptr->contextInterface, context, &ptr->triangleRangeBuffer, deviceBufUsage, eNvFlowFormat_unknown, sizeof(NvFlowUint2));

        NvFlowBufferDesc totalCountDesc = {};
        totalCountDesc.usageFlags = deviceBufUsage;
        totalCountDesc.format = eNvFlowFormat_unknown;
        totalCountDesc.structureStride = sizeof(NvFlowUint);
        totalCountDesc.sizeInBytes = 1024u * sizeof(NvFlowUint);

        ptr->totalCountBuffer = ptr->contextInterface.createBuffer(context, eNvFlowMemoryType_readback, &totalCountDesc);

        createTexture(context, cast(ptr), pixels, texWidth, texHeight);

        return cast(ptr);
    }

    void destroy(NvFlowContext* context, NvFlowImguiRenderer* renderer)
    {
        auto ptr = cast(renderer);

        ptr->textures.deletePointers();

        NvFlowUploadBuffer_destroy(context, &ptr->vertexPosTexCoordBuffer);
        NvFlowUploadBuffer_destroy(context, &ptr->vertexColorBuffer);
        NvFlowUploadBuffer_destroy(context, &ptr->indicesBuffer);
        NvFlowUploadBuffer_destroy(context, &ptr->drawCmdsBuffer);
        NvFlowUploadBuffer_destroy(context, &ptr->constantBuffer);

        NvFlowUploadBuffer_destroy(context, &ptr->textureUpload);
        NvFlowUploadBuffer_destroy(context, &ptr->textureTableUpload);
        if (ptr->textureDevice)
        {
            ptr->contextInterface.destroyTexture(context, ptr->textureDevice);
            ptr->textureDevice = nullptr;
        }
        ptr->contextInterface.destroySampler(context, ptr->samplerLinear);

        NvFlowDynamicBuffer_destroy(context, &ptr->treeBuffer);
        NvFlowDynamicBuffer_destroy(context, &ptr->tileCountBuffer);
        NvFlowDynamicBuffer_destroy(context, &ptr->triangleBuffer);
        NvFlowDynamicBuffer_destroy(context, &ptr->triangleRangeBuffer);

        ptr->contextInterface.destroyBuffer(context, ptr->totalCountBuffer);

        ImguiCS_destroy(context, &ptr->imguiCS);
        ImguiBuildCS_destroy(context, &ptr->imguiBuildCS);
        ImguiTileCS_destroy(context, &ptr->imguiTileCS);
        ImguiTileCountCS_destroy(context, &ptr->imguiTileCountCS);
        ImguiTileScanCS_destroy(context, &ptr->imguiTileScanCS);
        ImguiTextureUploadCS_destroy(context, &ptr->imguiTextureUploadCS);

        delete ptr;
    }

    void render(NvFlowContext* context, NvFlowImguiRenderer* renderer, ImDrawData* drawData, NvFlowUint width, NvFlowUint height, NvFlowTextureTransient* colorIn, NvFlowTextureTransient* colorOut)
    {
        auto ptr = cast(renderer);

        if (ptr->textureDirty)
        {
            ptr->textureDirty = false;

            ptr->textureTable.size = 0u;
            ptr->textureTable.pushBack((NvFlowUint)ptr->textures.size); // tableSize
            ptr->textureTable.pushBack(0u); // atlasGridWidthBits
            ptr->textureTable.pushBack(0u); // atlasWidthInv
            ptr->textureTable.pushBack(0u); // atlasHeightInv
            for (NvFlowUint64 textureIdx = 0u; textureIdx < ptr->textures.size; textureIdx++)
            {
                ptr->textureTable.pushBack(ptr->textures[textureIdx]->textureId);
            }
            NvFlowUint tileGridOffset = 0u;
            for (NvFlowUint64 textureIdx = 0u; textureIdx < ptr->textures.size; textureIdx++)
            {
                ptr->textureTable.pushBack(ptr->textures[textureIdx]->texWidth);
                ptr->textureTable.pushBack(ptr->textures[textureIdx]->texHeight);
                ptr->textureTable.pushBack(ptr->textures[textureIdx]->tileGridWidth);
                ptr->textureTable.pushBack(tileGridOffset);

                tileGridOffset += ptr->textures[textureIdx]->tileGridWidth *
                    ptr->textures[textureIdx]->tileGridHeight;
            }

            NvFlowUint atlasGridWidth = tileGridOffset;
            NvFlowUint atlasGridHeight = 1u;
            if (tileGridOffset > 512)
            {
                atlasGridWidth = 512;
                atlasGridHeight = (tileGridOffset + 511) / 512;
            }
            NvFlowUint atlasWidth = 18u * atlasGridWidth;
            NvFlowUint atlasHeight = 18u * atlasGridHeight;

            ptr->textureTable[1u] = 9u; // atlasGridWidthBits
            *((float*)&ptr->textureTable[2u]) = 1.f / float(atlasWidth); // atlasWidthInv
            *((float*)&ptr->textureTable[3u]) = 1.f / float(atlasHeight); // atlasHeightInv

            auto mappedTable = (NvFlowUint*)NvFlowUploadBuffer_map(context, &ptr->textureTableUpload, ptr->textureTable.size * 4u);
            for (NvFlowUint64 idx = 0u; idx < ptr->textureTable.size; idx++)
            {
                mappedTable[idx] = ptr->textureTable[idx];
            }
            NvFlowUploadBuffer_unmapDevice(context,
                &ptr->textureTableUpload, 0llu, ptr->textureTable.size * 4u, "ImguiTextureTableUpload");

            NvFlowUint64 uploadSize = tileGridOffset * 18u * 18u * 4u;
            auto mapped = (unsigned char*)NvFlowUploadBuffer_map(context, &ptr->textureUpload, uploadSize);
            NvFlowUint64 uploadOffset = 0u;
            for (NvFlowUint64 textureIdx = 0u; textureIdx < ptr->textures.size; textureIdx++)
            {
                for (NvFlowUint64 tileIdx = 0u; tileIdx < ptr->textures[textureIdx]->tiles.size; tileIdx++)
                {
                    memcpy(mapped + uploadOffset * 18u * 18u * 4u,
                        ptr->textures[textureIdx]->tiles[tileIdx].data,
                        18u * 18u * 4u);
                    uploadOffset++;
                }
            }
            NvFlowBufferTransient* uploadTransient = NvFlowUploadBuffer_unmap(context, &ptr->textureUpload);

            if (ptr->textureDevice && (ptr->textureWidth != atlasWidth || ptr->textureHeight != atlasHeight))
            {
                ptr->contextInterface.destroyTexture(context, ptr->textureDevice);
                ptr->textureDevice = nullptr;
                ptr->textureWidth = 0u;
                ptr->textureHeight = 0u;
            }
            if (!ptr->textureDevice)
            {
                ptr->textureWidth = atlasWidth;
                ptr->textureHeight = atlasHeight;

                NvFlowTextureDesc texDesc = {};
                texDesc.textureType = eNvFlowTextureType_2d;
                texDesc.usageFlags = eNvFlowTextureUsage_rwTexture | eNvFlowTextureUsage_texture;
                texDesc.format = eNvFlowFormat_r8g8b8a8_unorm;
                texDesc.width = ptr->textureWidth;
                texDesc.height = ptr->textureHeight;
                texDesc.depth = 1u;
                texDesc.mipLevels = 1u;
                texDesc.optimizedClearValue = NvFlowFloat4{ 0.f, 0.f, 0.f, 0.f };

                ptr->textureDevice = ptr->contextInterface.createTexture(context, &texDesc);
            }

            NvFlowTextureTransient* textureTransient = ptr->contextInterface.registerTextureAsTransient(context, ptr->textureDevice);

            {
                NvFlowUint3 gridDim = {
                    tileGridOffset,
                    1u,
                    1u
                };

                ImguiTextureUploadCS_PassParams passParams = {};
                passParams.uploadIn = uploadTransient;
                passParams.colorOut = textureTransient;

                ImguiTextureUploadCS_addPassCompute(context, &ptr->imguiTextureUploadCS, gridDim, &passParams);
            }
        }

        NvFlowUint numVertices = drawData->TotalVtxCount;
        NvFlowUint numIndices = drawData->TotalIdxCount;
        NvFlowUint numDrawCmds = 0u;
        for (int listIdx = 0; listIdx < drawData->CmdListsCount; listIdx++)
        {
            numDrawCmds += drawData->CmdLists[listIdx]->CmdBuffer.Size;
        }

        NvFlowUint numTriangles = numIndices / 3u;

        NvFlowUint trianglesPerBlock = 256u;
        NvFlowUint numBlocks = (numTriangles + trianglesPerBlock - 1u) / trianglesPerBlock;
        NvFlowUint64 treeNumBytes = numBlocks * (1u + 4u + 16u + 64u + 256u) * sizeof(NvFlowInt4);

        NvFlowDynamicBuffer_resize(context, &ptr->treeBuffer, treeNumBytes);

        NvFlowUint tileDimBits = 4u;
        NvFlowUint tileDim = 1u << tileDimBits;
        NvFlowUint tileGridDim_x = (width + tileDim - 1u) / tileDim;
        NvFlowUint tileGridDim_y = (height + tileDim - 1u) / tileDim;
        NvFlowUint tileGridDim_xy = tileGridDim_x * tileGridDim_y;
        NvFlowUint numTileBuckets = (tileGridDim_xy + 255u) / 256u;
        NvFlowUint numTileBucketPasses = (numTileBuckets + 255u) / 256u;

        NvFlowUint64 tileCountNumBytes = tileGridDim_x * tileGridDim_y * 3u * sizeof(NvFlowUint);

        NvFlowDynamicBuffer_resize(context, &ptr->tileCountBuffer, tileCountNumBytes);

        NvFlowUint maxTriangles = 4u * 256u * 1024u;
        NvFlowUint64 triangleBufferNumBytes = maxTriangles * sizeof(NvFlowUint);

        NvFlowDynamicBuffer_resize(context, &ptr->triangleBuffer, triangleBufferNumBytes);

        NvFlowUint64 triangleRangeBufferNumBytes = tileGridDim_xy * sizeof(NvFlowUint2);

        NvFlowDynamicBuffer_resize(context, &ptr->triangleRangeBuffer, triangleRangeBufferNumBytes);

        NvFlowUint64 numBytesPosTex = (numVertices + 1u) * sizeof(NvFlowFloat4);
        NvFlowUint64 numBytesColor = (numVertices + 1u) * sizeof(NvFlowUint);
        NvFlowUint64 numBytesIndices = (numIndices + 1u) * sizeof(NvFlowUint);
        NvFlowUint64 numBytesDrawCmds = (numDrawCmds + 1u) * sizeof(ImguiRendererDrawCmd);

        auto mappedPosTex = (NvFlowFloat4*)NvFlowUploadBuffer_map(context, &ptr->vertexPosTexCoordBuffer, numBytesPosTex);
        auto mappedColor = (NvFlowUint*)NvFlowUploadBuffer_map(context, &ptr->vertexColorBuffer, numBytesColor);
        auto mappedIndices = (NvFlowUint*)NvFlowUploadBuffer_map(context, &ptr->indicesBuffer, numBytesIndices);
        auto mappedDrawCmds = (ImguiRendererDrawCmd*)NvFlowUploadBuffer_map(context, &ptr->drawCmdsBuffer, numBytesDrawCmds);
        auto mapped = (ImguiRendererParams*)NvFlowUploadBuffer_map(context, &ptr->constantBuffer, sizeof(ImguiRendererParams));

        NvFlowUint vertexOffset = 0u;
        NvFlowUint indexOffset = 0u;
        NvFlowUint drawCmdOffset = 0u;

        for (int cmdListIdx = 0u; cmdListIdx < drawData->CmdListsCount; cmdListIdx++)
        {
            ImDrawList* cmdList = drawData->CmdLists[cmdListIdx];

            // copy vertices
            for (int vertIdx = 0; vertIdx < cmdList->VtxBuffer.Size; vertIdx++)
            {
                NvFlowUint writeIdx = vertIdx + vertexOffset;
                mappedPosTex[writeIdx].x = cmdList->VtxBuffer[vertIdx].pos.x;
                mappedPosTex[writeIdx].y = cmdList->VtxBuffer[vertIdx].pos.y;
                mappedPosTex[writeIdx].z = cmdList->VtxBuffer[vertIdx].uv.x;
                mappedPosTex[writeIdx].w = cmdList->VtxBuffer[vertIdx].uv.y;
                mappedColor[writeIdx] = cmdList->VtxBuffer[vertIdx].col;
            }

            // copy indices
            for (int indexIdx = 0; indexIdx < cmdList->IdxBuffer.Size; indexIdx++)
            {
                NvFlowUint writeIdx = indexIdx + indexOffset;
                mappedIndices[writeIdx] = cmdList->IdxBuffer[indexIdx] + vertexOffset;        // apply vertex offset on CPU
            }

            // copy drawCmds
            NvFlowUint indexOffsetLocal = indexOffset;
            for (int drawCmdIdx = 0; drawCmdIdx < cmdList->CmdBuffer.Size; drawCmdIdx++)
            {
                NvFlowUint writeIdx = drawCmdIdx + drawCmdOffset;
                auto& dst = mappedDrawCmds[writeIdx];
                auto& src = cmdList->CmdBuffer[drawCmdIdx];
                dst.clipRect.x = src.ClipRect.x;
                dst.clipRect.y = src.ClipRect.y;
                dst.clipRect.z = src.ClipRect.z;
                dst.clipRect.w = src.ClipRect.w;
                dst.elemCount = src.ElemCount;
                dst.userTexture = src.TextureId ? cast((NvFlowImguiTexture*)src.TextureId)->textureId : 0u;
                dst.vertexOffset = 0u;                                                    // vertex offset already applied
                dst.indexOffset = indexOffsetLocal;

                indexOffsetLocal += src.ElemCount;
            }

            vertexOffset += NvFlowUint(cmdList->VtxBuffer.Size);
            indexOffset += NvFlowUint(cmdList->IdxBuffer.Size);
            drawCmdOffset += NvFlowUint(cmdList->CmdBuffer.Size);
        }

        mapped->numVertices = numVertices;
        mapped->numIndices = numIndices;
        mapped->numDrawCmds = numDrawCmds;
        mapped->numBlocks = numBlocks;

        mapped->width = float(width);
        mapped->height = float(height);
        mapped->widthInv = 1.f / float(width);
        mapped->heightInv = 1.f / float(height);

        mapped->tileGridDim_x = tileGridDim_x;
        mapped->tileGridDim_y = tileGridDim_y;
        mapped->tileGridDim_xy = tileGridDim_xy;
        mapped->tileDimBits = tileDimBits;

        mapped->maxTriangles = maxTriangles;
        mapped->tileNumTrianglesOffset = 0u;
        mapped->tileLocalScanOffset = tileGridDim_xy;
        mapped->tileLocalTotalOffset = 2u * tileGridDim_xy;

        mapped->tileGlobalScanOffset = 2u * tileGridDim_xy + numTileBuckets;
        mapped->numTileBuckets = numTileBuckets;
        mapped->numTileBucketPasses = numTileBucketPasses;
        mapped->pad3 = 0u;

        //NvFlowBufferTransient* vertexPosTexCoordTransient = NvFlowUploadBuffer_unmapDevice(context, &ptr->vertexPosTexCoordBuffer, 0llu, numBytesPosTex);
        //NvFlowBufferTransient* vertexColorTransient = NvFlowUploadBuffer_unmapDevice(context, &ptr->vertexColorBuffer, 0llu, numBytesColor);
        //NvFlowBufferTransient* indicesTransient = NvFlowUploadBuffer_unmapDevice(context, &ptr->indicesBuffer, 0llu, numBytesIndices);
        //NvFlowBufferTransient* drawCmdsInTransient = NvFlowUploadBuffer_unmapDevice(context, &ptr->drawCmdsBuffer, 0llu, numBytesDrawCmds);
        //NvFlowBufferTransient* paramsInTransient = NvFlowUploadBuffer_unmap(context, &ptr->constantBuffer);

        NvFlowBufferTransient* vertexPosTexCoordTransient = NvFlowUploadBuffer_unmap(context, &ptr->vertexPosTexCoordBuffer);
        NvFlowBufferTransient* vertexColorTransient = NvFlowUploadBuffer_unmap(context, &ptr->vertexColorBuffer);
        NvFlowBufferTransient* indicesTransient = NvFlowUploadBuffer_unmap(context, &ptr->indicesBuffer);
        NvFlowBufferTransient* drawCmdsInTransient = NvFlowUploadBuffer_unmap(context, &ptr->drawCmdsBuffer);
        NvFlowBufferTransient* paramsInTransient = NvFlowUploadBuffer_unmap(context, &ptr->constantBuffer);

        NvFlowTextureTransient* textureTransient = ptr->contextInterface.registerTextureAsTransient(context, ptr->textureDevice);
        NvFlowBufferTransient* textureTableTransient = ptr->contextInterface.registerBufferAsTransient(context, ptr->textureTableUpload.deviceBuffer);
        NvFlowBufferTransient* treeTransient = NvFlowDynamicBuffer_getTransient(context, &ptr->treeBuffer);
        NvFlowBufferTransient* tileCountTransient = NvFlowDynamicBuffer_getTransient(context, &ptr->tileCountBuffer);
        NvFlowBufferTransient* triangleTransient = NvFlowDynamicBuffer_getTransient(context, &ptr->triangleBuffer);
        NvFlowBufferTransient* triangleRangeTransient = NvFlowDynamicBuffer_getTransient(context, &ptr->triangleRangeBuffer);

        auto totalCountMapped = (NvFlowUint*)ptr->contextInterface.mapBuffer(context, ptr->totalCountBuffer);

        ptr->contextInterface.unmapBuffer(context, ptr->totalCountBuffer);

        NvFlowBufferTransient* totalCountTransient = ptr->contextInterface.registerBufferAsTransient(context, ptr->totalCountBuffer);

        // build acceleration structure
        {
            NvFlowUint3 gridDim = {
                numBlocks,
                1u,
                1u
            };

            ImguiBuildCS_PassParams passParams = {};
            passParams.paramsIn = paramsInTransient;
            passParams.vertexPosTexCoordIn = vertexPosTexCoordTransient;
            passParams.vertexColorIn = vertexColorTransient;
            passParams.indicesIn = indicesTransient;
            passParams.drawCmdsIn = drawCmdsInTransient;
            passParams.treeOut = treeTransient;

            ImguiBuildCS_addPassCompute(context, &ptr->imguiBuildCS, gridDim, &passParams);
        }

        // count triangles per tile
        {
            NvFlowUint3 gridDim = {
                (tileGridDim_xy + 255u) / 256u,
                1u,
                1u
            };

            ImguiTileCountCS_PassParams passParams = {};
            passParams.paramsIn = paramsInTransient;
            passParams.treeIn = treeTransient;
            passParams.tileCountOut = tileCountTransient;

            ImguiTileCountCS_addPassCompute(context, &ptr->imguiTileCountCS, gridDim, &passParams);
        }

        // scan buckets
        {
            NvFlowUint3 gridDim = {
                1u,
                1u,
                1u
            };

            ImguiTileScanCS_PassParams passParams = {};
            passParams.paramsIn = paramsInTransient;
            passParams.tileCountOut = tileCountTransient;
            passParams.totalCountOut = totalCountTransient;

            ImguiTileScanCS_addPassCompute(context, &ptr->imguiTileScanCS, gridDim, &passParams);
        }

        // generate tile data
        {
            NvFlowUint3 gridDim = {
                (tileGridDim_xy + 255u) / 256u,
                1u,
                1u
            };

            ImguiTileCS_PassParams passParams = {};
            passParams.paramsIn = paramsInTransient;
            passParams.treeIn = treeTransient;
            passParams.tileCountIn = tileCountTransient;
            passParams.drawCmdsIn = drawCmdsInTransient;
            passParams.triangleOut = triangleTransient;
            passParams.triangleRangeOut = triangleRangeTransient;

            ImguiTileCS_addPassCompute(context, &ptr->imguiTileCS, gridDim, &passParams);
        }

        // render
        {
            NvFlowUint3 gridDim = {
                (width + 7u) / 8u,
                (height + 7u) / 8u,
                1u
            };

            ImguiCS_PassParams passParams = {};
            passParams.paramsIn = paramsInTransient;
            passParams.vertexPosTexCoordIn = vertexPosTexCoordTransient;
            passParams.vertexColorIn = vertexColorTransient;
            passParams.indicesIn = indicesTransient;
            passParams.drawCmdsIn = drawCmdsInTransient;
            passParams.textureIn = textureTransient;
            passParams.textureTableIn = textureTableTransient;
            passParams.samplerIn = ptr->samplerLinear;
            passParams.triangleIn = triangleTransient;
            passParams.triangleRangeIn = triangleRangeTransient;
            passParams.colorIn = colorIn;
            passParams.colorOut = colorOut;

            ImguiCS_addPassCompute(context, &ptr->imguiCS, gridDim, &passParams);
        }
    }

    void updateTexture(
        NvFlowContext* context,
        NvFlowImguiRenderer* renderer,
        NvFlowImguiTexture* texture,
        unsigned char* pixels,
        int texWidth,
        int texHeight
    )
    {
        auto ptr = cast(renderer);
        auto tex = cast(texture);

        ptr->textureDirty = true;

        tex->texWidth = texWidth;
        tex->texHeight = texHeight;
        tex->tileGridWidth = (tex->texWidth + 15u) / 16u;
        tex->tileGridHeight = (tex->texHeight + 15u) / 16u;

        tex->tiles.reserve(tex->tileGridWidth * tex->tileGridHeight);
        tex->tiles.size = tex->tileGridWidth * tex->tileGridHeight;

        for (int tj = 0; tj < tex->tileGridHeight; tj++)
        {
            for (int ti = 0; ti < tex->tileGridWidth; ti++)
            {
                int tile_idx = tj * tex->tileGridWidth + ti;
                for (int j = -1; j < 17; j++)
                {
                    for (int i = -1; i < 17; i++)
                    {
                        unsigned char r = 0;
                        unsigned char g = 0;
                        unsigned char b = 0;
                        unsigned char a = 0;

                        int src_i = (int)((NvFlowUint)((ti << 4u) + i) % (NvFlowUint)tex->texWidth);
                        int src_j = (int)((NvFlowUint)((tj << 4u) + j) % (NvFlowUint)tex->texHeight);
                        int src_idx = src_j * tex->texWidth + src_i;
                        r = pixels[4u * src_idx + 0];
                        g = pixels[4u * src_idx + 1];
                        b = pixels[4u * src_idx + 2];
                        a = pixels[4u * src_idx + 3];

                        int tile_subidx = (j + 1) * 18 + (i + 1);
                        tex->tiles[tile_idx].data[tile_subidx * 4 + 0] = r;
                        tex->tiles[tile_idx].data[tile_subidx * 4 + 1] = g;
                        tex->tiles[tile_idx].data[tile_subidx * 4 + 2] = b;
                        tex->tiles[tile_idx].data[tile_subidx * 4 + 3] = a;
                    }
                }
            }
        }
    }

    NvFlowImguiTexture* createTexture(
        NvFlowContext* context,
        NvFlowImguiRenderer* renderer,
        unsigned char* pixels,
        int texWidth,
        int texHeight
    )
    {
        auto ptr = cast(renderer);
        auto tex = ptr->textures.allocateBackPointer();

        updateTexture(context, renderer, cast(tex), pixels, texWidth, texHeight);

        tex->textureId = ptr->textureIdCounter;
        ptr->textureIdCounter++;

        return cast(tex);
    }

    void destroyTexture(
        NvFlowContext* context,
        NvFlowImguiRenderer* renderer,
        NvFlowImguiTexture* texture
    )
    {
        auto ptr = cast(renderer);
        auto tex = cast(texture);

        tex->tiles.release();

        ptr->textures.removeSwapPointer(tex);

        ptr->textureDirty = true;
    }
}

NvFlowImguiRendererInterface* NvFlowGetImguiRendererInterface()
{
    using namespace NvFlowImguiRendererDefault;
    static NvFlowImguiRendererInterface iface = { NV_FLOW_REFLECT_INTERFACE_INIT(NvFlowImguiRendererInterface) };
    iface.create = create;
    iface.destroy = destroy;
    iface.render = render;
    iface.createTexture = createTexture;
    iface.updateTexture = updateTexture;
    iface.destroyTexture = destroyTexture;
    return &iface;
}
