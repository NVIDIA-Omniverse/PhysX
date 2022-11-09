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

#include "FrameCapture.h"

#include "NvFlowDynamicBuffer.h"

#include <stdio.h>

#if !defined(_WIN32)
NV_FLOW_INLINE void fopen_s(FILE** streamptr, const char* filename, const char* mode)
{
	*streamptr = fopen(filename, mode);
}
#endif

namespace NvFlowFrameCaptureDefault
{
	struct BitmapHeader
	{
		char headerField0, headerField1;
		unsigned int size;
		unsigned short reserved1;
		unsigned short reserved2;
		unsigned int offset;
		unsigned int headerSize;
		unsigned int width;
		unsigned int height;
		unsigned short colorPlanes;
		unsigned short bitsPerPixel;
		unsigned int compressionMethod;
		unsigned int imageSize;
		unsigned int hRes;
		unsigned int vRes;
		unsigned int numColors;
		unsigned int numImportantColors;
	};

	struct FrameCaptureBuffer
	{
		NvFlowBool32 isActive;
		NvFlowBuffer* buffer = nullptr;
		NvFlowUint64 bufferSize = 0llu;
		NvFlowUint64 rowPitch = 0llu;
		NvFlowUint width;
		NvFlowUint height;
		NvFlowUint64 completedFence = 0llu;
	};

	struct FrameCapture
	{
		NvFlowContextInterface contextInterface = {};

		NvFlowArray<FrameCaptureBuffer> buffers;

		NvFlowUint64 captureFrameID = 0llu;
	};

	NV_FLOW_CAST_PAIR(NvFlowFrameCapture, FrameCapture)

	NvFlowFrameCapture* create(NvFlowContextInterface* contextInterface, NvFlowContext* context)
	{
		auto ptr = new FrameCapture();

		NvFlowContextInterface_duplicate(&ptr->contextInterface, contextInterface);

		return cast(ptr);
	}

	void destroy(NvFlowContext* context, NvFlowFrameCapture* renderer)
	{
		auto ptr = cast(renderer);

		for (NvFlowUint idx = 0u; idx < ptr->buffers.size; idx++) 
		{
			ptr->contextInterface.destroyBuffer(context, ptr->buffers[idx].buffer);
			ptr->buffers[idx].buffer = nullptr;
		}

		delete ptr;
	}

	void capture(NvFlowContext* context, NvFlowFrameCapture* frameCapture, NvFlowUint width, NvFlowUint height, NvFlowTextureTransient* texture)
	{
		auto ptr = cast(frameCapture);

		// Note: assume bgra8
		NvFlowUint rowPitch = width * sizeof(NvFlowUint);
		rowPitch = 256u * ((rowPitch + 255u) / 256u);
		NvFlowUint64 minBufferSize = rowPitch * height;

		FrameCaptureBuffer* captureBuffer = nullptr;
		for (NvFlowUint idx = 0u; idx < ptr->buffers.size; idx++)
		{
			if (!ptr->buffers[idx].isActive)
			{
				captureBuffer = &ptr->buffers[idx];
				break;
			}
		}
		if (!captureBuffer)
		{
			captureBuffer = &ptr->buffers[ptr->buffers.allocateBack()];
		}

		if (captureBuffer->bufferSize < minBufferSize)
		{
			if (captureBuffer->buffer)
			{
				ptr->contextInterface.destroyBuffer(context, captureBuffer->buffer);
				captureBuffer->buffer = nullptr;
				captureBuffer->bufferSize = 0llu;
			}

			NvFlowBufferDesc bufDesc = {};
			bufDesc.format = eNvFlowFormat_unknown;
			bufDesc.usageFlags = eNvFlowBufferUsage_bufferCopyDst;
			bufDesc.structureStride = 0u;
			bufDesc.sizeInBytes = 65536u;
			while (bufDesc.sizeInBytes < minBufferSize)
			{
				bufDesc.sizeInBytes *= 2u;
			}

			captureBuffer->bufferSize = bufDesc.sizeInBytes;
			captureBuffer->buffer = ptr->contextInterface.createBuffer(context, eNvFlowMemoryType_readback, &bufDesc);
		}

		captureBuffer->rowPitch = rowPitch;
		captureBuffer->width = width;
		captureBuffer->height = height;

		NvFlowPassCopyTextureToBufferParams copyParams = {};
		copyParams.bufferOffset = 0llu;
		copyParams.bufferRowPitch = rowPitch;
		copyParams.bufferDepthPitch = height * rowPitch;

		copyParams.textureMipLevel = 0;
		copyParams.textureOffset = NvFlowUint3{ 0u, 0u, 0u };
		copyParams.textureExtent = NvFlowUint3{ width, height, 1u };

		copyParams.src = texture;
		copyParams.dst = ptr->contextInterface.registerBufferAsTransient(context, captureBuffer->buffer);

		copyParams.debugLabel = "FrameCapture";

		ptr->contextInterface.addPassCopyTextureToBuffer(context, &copyParams);

		captureBuffer->isActive = NV_FLOW_TRUE;
		captureBuffer->completedFence = ptr->contextInterface.getCurrentFrame(context);
	}

	void update(NvFlowContext* context, NvFlowFrameCapture* frameCapture)
	{
		auto ptr = cast(frameCapture);

		NvFlowUint64 minFence = ~0llu;
		FrameCaptureBuffer* captureBuffer = nullptr;
		for (NvFlowUint idx = 0u; idx < ptr->buffers.size; idx++)
		{
			if (ptr->buffers[idx].isActive)
			{
				if (ptr->buffers[idx].completedFence < minFence)
				{
					minFence = ptr->buffers[idx].completedFence;
					captureBuffer = &ptr->buffers[idx];
				}
			}
		}
		if (!captureBuffer)
		{
			return;
		}
		if (minFence > ptr->contextInterface.getLastFrameCompleted(context))
		{
			return;
		}

		NvFlowUint8* mappedData = (NvFlowUint8*)ptr->contextInterface.mapBuffer(context, captureBuffer->buffer);

		char buf[80] = {};
		snprintf(buf, 80, "capture%lld.bmp", ptr->captureFrameID);

		FILE* file = nullptr;
		fopen_s(&file, buf, "wb");
		if (file)
		{
			BitmapHeader header = {};

			const NvFlowUint bitsPerPixel = 32;
			const NvFlowUint bytesPerPixel = bitsPerPixel / 8;
			const NvFlowUint imagesize = captureBuffer->width * captureBuffer->height * bytesPerPixel;
			header.headerField0 = 'B';
			header.headerField1 = 'M';
			header.size = 54 + imagesize;
			header.reserved1 = 0;
			header.reserved2 = 0;
			header.offset = 54;
			header.headerSize = 40;
			header.width = captureBuffer->width;
			header.height = captureBuffer->height;
			header.colorPlanes = 1;
			header.bitsPerPixel = bitsPerPixel;
			header.compressionMethod = 0;
			header.imageSize = imagesize;
			header.hRes = 2000;
			header.vRes = 2000;
			header.numColors = 0;
			header.numImportantColors = 0;

			fwrite(&header.headerField0, 1, 1, file);
			fwrite(&header.headerField1, 1, 1, file);
			fwrite(&header.size, 4, 1, file);
			fwrite(&header.reserved1, 2, 1, file);
			fwrite(&header.reserved2, 2, 1, file);
			fwrite(&header.offset, 4, 1, file);
			fwrite(&header.headerSize, 4, 1, file);
			fwrite(&header.width, 4, 1, file);
			fwrite(&header.height, 4, 1, file);
			fwrite(&header.colorPlanes, 2, 1, file);
			fwrite(&header.bitsPerPixel, 2, 1, file);
			fwrite(&header.compressionMethod, 4, 1, file);
			fwrite(&header.imageSize, 4, 1, file);
			fwrite(&header.hRes, 4, 1, file);
			fwrite(&header.vRes, 4, 1, file);
			fwrite(&header.numColors, 4, 1, file);
			fwrite(&header.numImportantColors, 4, 1, file);

			if (header.compressionMethod == 0)
			{
				for (NvFlowUint rowIdx = 0u; rowIdx < header.height; rowIdx++)
				{
					unsigned char* srcData = mappedData + (header.height - 1u - rowIdx) * captureBuffer->rowPitch;
					fwrite(srcData, 1, header.width * bytesPerPixel,  file);
				}
			}

			fclose(file);
		}

		ptr->contextInterface.unmapBuffer(context, captureBuffer->buffer);

		captureBuffer->isActive = NV_FLOW_FALSE;
		captureBuffer->completedFence = ~0llu;

		ptr->captureFrameID++;
	}
}

NvFlowFrameCaptureInterface* NvFlowGetFrameCaptureInterface()
{
	using namespace NvFlowFrameCaptureDefault;
	static NvFlowFrameCaptureInterface iface = { NV_FLOW_REFLECT_INTERFACE_INIT(NvFlowFrameCaptureInterface) };
	iface.create = create;
	iface.destroy = destroy;
	iface.capture = capture;
	iface.update = update;
	return &iface;
}