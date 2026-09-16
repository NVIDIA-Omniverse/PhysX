// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxPvdUserRenderImpl.h"
#include "PxPvdInternalByteStreams.h"
#include "PxPvdBits.h"
#include <stdarg.h>

using namespace physx;
using namespace physx::pvdsdk;

namespace
{

template <typename TStreamType>
struct RenderWriter : public RenderSerializer
{
	TStreamType& mStream;
	RenderWriter(TStreamType& stream) : mStream(stream)
	{
	}
	template <typename TDataType>
	void write(const TDataType* val, uint32_t count)
	{
		uint32_t numBytes = count * sizeof(TDataType);
		mStream.write(reinterpret_cast<const uint8_t*>(val), numBytes);
	}
	template <typename TDataType>
	void write(const TDataType& val)
	{
		write(&val, 1);
	}

	template <typename TDataType>
	void writeRef(DataRef<TDataType>& val)
	{
		uint32_t amount = val.size();
		write(amount);
		if(amount)
			write(val.begin(), amount);
	}

	virtual void streamify(uint64_t& val) PX_OVERRIDE
	{
		write(val);
	}
	virtual void streamify(uint32_t& val) PX_OVERRIDE
	{
		write(val);
	}
	virtual void streamify(float& val) PX_OVERRIDE
	{
		write(val);
	}
	virtual void streamify(uint8_t& val) PX_OVERRIDE
	{
		write(val);
	}
	virtual void streamify(DataRef<uint8_t>& val) PX_OVERRIDE
	{
		writeRef(val);
	}

	virtual void streamify(PxDebugText& val) PX_OVERRIDE
	{
		write(val.color);
		write(val.position);
		write(val.size);

		uint32_t amount = static_cast<uint32_t>(strnlen(val.string, UINT32_MAX - 1)) + 1;
		write(amount);
		if(amount)
			write(val.string, amount);
	}

	virtual void streamify(DataRef<PxDebugPoint>& val) PX_OVERRIDE
	{
		writeRef(val);
	}
	virtual void streamify(DataRef<PxDebugLine>& val) PX_OVERRIDE
	{
		writeRef(val);
	}
	virtual void streamify(DataRef<PxDebugTriangle>& val) PX_OVERRIDE
	{
		writeRef(val);
	}

	virtual uint32_t hasData() PX_OVERRIDE
	{
		return false;
	}
	virtual bool isGood() PX_OVERRIDE
	{
		return true;
	}

  private:
	RenderWriter& operator=(const RenderWriter&);
};

struct UserRenderer : public PvdUserRenderer
{
	ForwardingMemoryBuffer mBuffer;
	uint32_t mBufferCapacity;
	RendererEventClient* mClient;

	UserRenderer(uint32_t bufferFullAmount)
	: mBuffer("UserRenderBuffer"), mBufferCapacity(bufferFullAmount), mClient(NULL)
	{
	}
	virtual ~UserRenderer()
	{
	}
	virtual void release() PX_OVERRIDE
	{
		PVD_DELETE(this);
	}

	template <typename TEventType>
	void handleEvent(TEventType evt)
	{
		RenderWriter<ForwardingMemoryBuffer> _writer(mBuffer);
		RenderSerializer& writer(_writer);

		PvdUserRenderTypes::Enum evtType(getPvdRenderTypeFromType<TEventType>());
		writer.streamify(evtType);
		evt.serialize(writer);
		if(mBuffer.size() >= mBufferCapacity)
			flushRenderEvents();
	}
	virtual void setInstanceId(const void* iid) PX_OVERRIDE
	{
		handleEvent(SetInstanceIdRenderEvent(PVD_POINTER_TO_U64(iid)));
	}
	// Draw these points associated with this instance
	virtual void drawPoints(const PxDebugPoint* points, uint32_t count) PX_OVERRIDE
	{
		handleEvent(PointsRenderEvent(points, count));
	}
	// Draw these lines associated with this instance
	virtual void drawLines(const PxDebugLine* lines, uint32_t count) PX_OVERRIDE
	{
		handleEvent(LinesRenderEvent(lines, count));
	}
	// Draw these triangles associated with this instance
	virtual void drawTriangles(const PxDebugTriangle* triangles, uint32_t count) PX_OVERRIDE
	{
		handleEvent(TrianglesRenderEvent(triangles, count));
	}

	virtual void drawText(const PxDebugText& text) PX_OVERRIDE
	{
		handleEvent(TextRenderEvent(text));
	}

	virtual void drawRenderbuffer(const PxDebugPoint* pointData, uint32_t pointCount, const PxDebugLine* lineData,
	                              uint32_t lineCount, const PxDebugTriangle* triangleData, uint32_t triangleCount) PX_OVERRIDE
	{
		handleEvent(DebugRenderEvent(pointData, pointCount, lineData, lineCount, triangleData, triangleCount));
	}

	// Constraint visualization routines
	virtual void visualizeJointFrames(const PxTransform& parent, const PxTransform& child)	PX_OVERRIDE
	{
		handleEvent(JointFramesRenderEvent(parent, child));
	}
	virtual void visualizeLinearLimit(const PxTransform& t0, const PxTransform& t1, float value)	PX_OVERRIDE
	{
		handleEvent(LinearLimitRenderEvent(t0, t1, value, true));
	}
	virtual void visualizeAngularLimit(const PxTransform& t0, float lower, float upper)	PX_OVERRIDE
	{
		handleEvent(AngularLimitRenderEvent(t0, lower, upper, true));
	}
	virtual void visualizeLimitCone(const PxTransform& t, float tanQSwingY, float tanQSwingZ)	PX_OVERRIDE
	{
		handleEvent(LimitConeRenderEvent(t, tanQSwingY, tanQSwingZ, true));
	}
	virtual void visualizeDoubleCone(const PxTransform& t, float angle)	PX_OVERRIDE
	{
		handleEvent(DoubleConeRenderEvent(t, angle, true));
	}
	// Clear the immedate buffer.
	virtual void flushRenderEvents() PX_OVERRIDE
	{
		if(mClient)
			mClient->handleBufferFlush(mBuffer.begin(), mBuffer.size());
		mBuffer.clear();
	}

	virtual void setClient(RendererEventClient* client) PX_OVERRIDE
	{
		mClient = client;
	}

  private:
	UserRenderer& operator=(const UserRenderer&);
};

template <bool swapBytes>
struct RenderReader : public RenderSerializer
{
	MemPvdInputStream mStream;
	ForwardingMemoryBuffer& mBuffer;

	RenderReader(ForwardingMemoryBuffer& buf) : mBuffer(buf)
	{
	}
	void setData(DataRef<const uint8_t> data)
	{
		mStream.setup(const_cast<uint8_t*>(data.begin()), const_cast<uint8_t*>(data.end()));
	}
	virtual void streamify(uint32_t& val) PX_OVERRIDE
	{
		mStream >> val;
	}
	virtual void streamify(uint64_t& val) PX_OVERRIDE
	{
		mStream >> val;
	}
	virtual void streamify(float& val) PX_OVERRIDE
	{
		mStream >> val;
	}
	virtual void streamify(uint8_t& val) PX_OVERRIDE
	{
		mStream >> val;
	}
	template <typename TDataType>
	void readRef(DataRef<TDataType>& val)
	{
		uint32_t count;
		mStream >> count;
		uint32_t numBytes = sizeof(TDataType) * count;

		TDataType* dataPtr = reinterpret_cast<TDataType*>(mBuffer.growBuf(numBytes));
		mStream.read(reinterpret_cast<uint8_t*>(dataPtr), numBytes);
		val = DataRef<TDataType>(dataPtr, count);
	}

	virtual void streamify(DataRef<PxDebugPoint>& val) PX_OVERRIDE
	{
		readRef(val);
	}
	virtual void streamify(DataRef<PxDebugLine>& val) PX_OVERRIDE
	{
		readRef(val);
	}
	virtual void streamify(DataRef<PxDebugTriangle>& val) PX_OVERRIDE
	{
		readRef(val);
	}
	virtual void streamify(PxDebugText& val) PX_OVERRIDE
	{
		mStream >> val.color;
		mStream >> val.position;
		mStream >> val.size;

		uint32_t len = 0;
		mStream >> len;

		uint8_t* dataPtr = mBuffer.growBuf(len);
		mStream.read(dataPtr, len);
		val.string = reinterpret_cast<const char*>(dataPtr);
	}
	virtual void streamify(DataRef<uint8_t>& val) PX_OVERRIDE
	{
		readRef(val);
	}
	virtual bool isGood() PX_OVERRIDE
	{
		return mStream.isGood();
	}
	virtual uint32_t hasData() PX_OVERRIDE
	{
		return uint32_t(mStream.size() > 0);
	}

  private:
	RenderReader& operator=(const RenderReader&);
};

template <>
struct RenderReader<true> : public RenderSerializer
{
	MemPvdInputStream mStream;
	ForwardingMemoryBuffer& mBuffer;
	RenderReader(ForwardingMemoryBuffer& buf) : mBuffer(buf)
	{
	}
	void setData(DataRef<const uint8_t> data)
	{
		mStream.setup(const_cast<uint8_t*>(data.begin()), const_cast<uint8_t*>(data.end()));
	}

	template <typename TDataType>
	void read(TDataType& val)
	{
		mStream >> val;
		swapBytes(val);
	}
	virtual void streamify(uint64_t& val) PX_OVERRIDE
	{
		read(val);
	}
	virtual void streamify(uint32_t& val) PX_OVERRIDE
	{
		read(val);
	}
	virtual void streamify(float& val) PX_OVERRIDE
	{
		read(val);
	}
	virtual void streamify(uint8_t& val) PX_OVERRIDE
	{
		read(val);
	}
	template <typename TDataType>
	void readRef(DataRef<TDataType>& val)
	{
		uint32_t count;
		mStream >> count;
		swapBytes(count);
		uint32_t numBytes = sizeof(TDataType) * count;

		TDataType* dataPtr = reinterpret_cast<TDataType*>(mBuffer.growBuf(numBytes));
		PVD_FOREACH(idx, count)
		RenderSerializerMap<TDataType>().serialize(*this, dataPtr[idx]);
		val = DataRef<TDataType>(dataPtr, count);
	}

	virtual void streamify(DataRef<PxDebugPoint>& val) PX_OVERRIDE
	{
		readRef(val);
	}
	virtual void streamify(DataRef<PxDebugLine>& val) PX_OVERRIDE
	{
		readRef(val);
	}
	virtual void streamify(DataRef<PxDebugTriangle>& val) PX_OVERRIDE
	{
		readRef(val);
	}
	virtual void streamify(PxDebugText& val) PX_OVERRIDE
	{
		mStream >> val.color;
		mStream >> val.position;
		mStream >> val.size;

		uint32_t len = 0;
		mStream >> len;

		uint8_t* dataPtr = mBuffer.growBuf(len);
		mStream.read(dataPtr, len);
		val.string = reinterpret_cast<const char*>(dataPtr);
	}
	virtual void streamify(DataRef<uint8_t>& val) PX_OVERRIDE
	{
		readRef(val);
	}
	virtual bool isGood() PX_OVERRIDE
	{
		return mStream.isGood();
	}
	virtual uint32_t hasData() PX_OVERRIDE
	{
		return uint32_t(mStream.size() > 0);
	}

  private:
	RenderReader& operator=(const RenderReader&);
};

}

PvdUserRenderer* PvdUserRenderer::create(uint32_t bufferSize)
{
	return PVD_NEW(UserRenderer)(bufferSize);
}

