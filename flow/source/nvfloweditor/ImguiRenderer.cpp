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
//
// Copyright (c) 2014-2022 NVIDIA Corporation. All rights reserved.

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

namespace NvFlowImguiRendererDefault
{
	struct Renderer
	{
		NvFlowContextInterface contextInterface = {};

		ImguiCS_Pipeline imguiCS = {};
		ImguiBuildCS_Pipeline imguiBuildCS = {};
		ImguiTileCS_Pipeline imguiTileCS = {};
		ImguiTileCountCS_Pipeline imguiTileCountCS = {};
		ImguiTileScanCS_Pipeline imguiTileScanCS = {};

		NvFlowUploadBuffer vertexPosTexCoordBuffer = {};
		NvFlowUploadBuffer vertexColorBuffer = {};
		NvFlowUploadBuffer indicesBuffer = {};
		NvFlowUploadBuffer drawCmdsBuffer = {};
		NvFlowUploadBuffer constantBuffer = {};

		NvFlowUploadBuffer textureUpload = {};
		NvFlowTexture* textureDevice = nullptr;
		NvFlowSampler* samplerLinear = nullptr;

		NvFlowDynamicBuffer treeBuffer = {};
		NvFlowDynamicBuffer tileCountBuffer = {};
		NvFlowDynamicBuffer triangleBuffer = {};
		NvFlowDynamicBuffer triangleRangeBuffer = {};

		NvFlowBuffer* totalCountBuffer = nullptr;
	};

	NV_FLOW_CAST_PAIR(NvFlowImguiRenderer, Renderer)

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

		NvFlowBufferUsageFlags bufferUsage = eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_bufferCopySrc;

		NvFlowUploadBuffer_init(&ptr->contextInterface, context, &ptr->vertexPosTexCoordBuffer, bufferUsage, eNvFlowFormat_unknown, sizeof(NvFlowFloat4));
		NvFlowUploadBuffer_init(&ptr->contextInterface, context, &ptr->vertexColorBuffer, bufferUsage, eNvFlowFormat_unknown, sizeof(NvFlowUint));
		NvFlowUploadBuffer_init(&ptr->contextInterface, context, &ptr->indicesBuffer, bufferUsage, eNvFlowFormat_unknown, sizeof(NvFlowUint));
		NvFlowUploadBuffer_init(&ptr->contextInterface, context, &ptr->drawCmdsBuffer, bufferUsage, eNvFlowFormat_unknown, sizeof(ImguiRendererDrawCmd));
		NvFlowUploadBuffer_init(&ptr->contextInterface, context, &ptr->constantBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);

		NvFlowUploadBuffer_init(&ptr->contextInterface, context, &ptr->textureUpload, eNvFlowBufferUsage_bufferCopySrc, eNvFlowFormat_unknown, 0u);

		NvFlowUint numBytes = NvFlowUint(texWidth * texHeight * 4u * sizeof(unsigned char));
		auto mapped = (unsigned char*)NvFlowUploadBuffer_map(context, &ptr->textureUpload, numBytes);
		for (NvFlowUint idx = 0u; idx < numBytes; idx++)
		{
			mapped[idx] = pixels[idx];
		}
		NvFlowBufferTransient* bufferTransient = NvFlowUploadBuffer_unmap(context, &ptr->textureUpload);

		NvFlowTextureDesc texDesc = {};
		texDesc.textureType = eNvFlowTextureType_2d;
		texDesc.usageFlags = eNvFlowTextureUsage_textureCopyDst | eNvFlowTextureUsage_texture;
		texDesc.format = eNvFlowFormat_r8g8b8a8_unorm;
		texDesc.width = texWidth;
		texDesc.height = texHeight;
		texDesc.depth = 1u;
		texDesc.mipLevels = 1u;
		texDesc.optimizedClearValue = NvFlowFloat4{0.f, 0.f, 0.f, 0.f};

		ptr->textureDevice = ptr->contextInterface.createTexture(context, &texDesc);

		NvFlowSamplerDesc samplerDesc = {};
		samplerDesc.filterMode = eNvFlowSamplerFilterMode_linear;
		samplerDesc.addressModeU = eNvFlowSamplerAddressMode_wrap;
		samplerDesc.addressModeV = eNvFlowSamplerAddressMode_wrap;
		samplerDesc.addressModeW = eNvFlowSamplerAddressMode_wrap;

		ptr->samplerLinear = ptr->contextInterface.createSampler(context, &samplerDesc);

		NvFlowTextureTransient* textureTransient = ptr->contextInterface.registerTextureAsTransient(context, ptr->textureDevice);

		NvFlowPassCopyBufferToTextureParams copyParams = {};
		copyParams.bufferOffset = 0llu;
		copyParams.bufferRowPitch = texWidth * 4u * sizeof(unsigned char);
		copyParams.bufferDepthPitch = numBytes;

		copyParams.textureMipLevel = 0u;
		copyParams.textureOffset = NvFlowUint3{0u, 0u, 0u};
		copyParams.textureExtent = NvFlowUint3{NvFlowUint(texWidth), NvFlowUint(texHeight), 1u};

		copyParams.src = bufferTransient;
		copyParams.dst = textureTransient;

		copyParams.debugLabel = "ImguiUploadTexture";

		ptr->contextInterface.addPassCopyBufferToTexture(context, &copyParams);

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

		return cast(ptr);
	}

	void destroy(NvFlowContext* context, NvFlowImguiRenderer* renderer)
	{
		auto ptr = cast(renderer);

		NvFlowUploadBuffer_destroy(context, &ptr->vertexPosTexCoordBuffer);
		NvFlowUploadBuffer_destroy(context, &ptr->vertexColorBuffer);
		NvFlowUploadBuffer_destroy(context, &ptr->indicesBuffer);
		NvFlowUploadBuffer_destroy(context, &ptr->drawCmdsBuffer);
		NvFlowUploadBuffer_destroy(context, &ptr->constantBuffer);

		NvFlowUploadBuffer_destroy(context, &ptr->textureUpload);
		ptr->contextInterface.destroyTexture(context, ptr->textureDevice);
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

		delete ptr;
	}

	void render(NvFlowContext* context, NvFlowImguiRenderer* renderer, ImDrawData* drawData, NvFlowUint width, NvFlowUint height, NvFlowTextureTransient* colorIn, NvFlowTextureTransient* colorOut)
	{
		auto ptr = cast(renderer);

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
				mappedIndices[writeIdx] = cmdList->IdxBuffer[indexIdx] + vertexOffset;		// apply vertex offset on CPU
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
				dst.userTexture = *((NvFlowUint*)(&src.TextureId));
				dst.vertexOffset = 0u;													// vertex offset already applied
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
			passParams.samplerIn = ptr->samplerLinear;
			passParams.triangleIn = triangleTransient;
			passParams.triangleRangeIn = triangleRangeTransient;
			passParams.colorIn = colorIn;
			passParams.colorOut = colorOut;

			ImguiCS_addPassCompute(context, &ptr->imguiCS, gridDim, &passParams);
		}
	}
}

NvFlowImguiRendererInterface* NvFlowGetImguiRendererInterface()
{
	using namespace NvFlowImguiRendererDefault;
	static NvFlowImguiRendererInterface iface = { NV_FLOW_REFLECT_INTERFACE_INIT(NvFlowImguiRendererInterface) };
	iface.create = create;
	iface.destroy = destroy;
	iface.render = render;
	return &iface;
}