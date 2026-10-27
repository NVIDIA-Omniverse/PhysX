// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PVD_COMM_STREAM_TYPES_H
#define PX_PVD_COMM_STREAM_TYPES_H

#include "foundation/PxErrorCallback.h"
#include "common/PxRenderBuffer.h"
#include "pvd/PxPvdTransport.h"

#include "PxPvdObjectModelBaseTypes.h"
#include "PxPvdCommStreamEvents.h"
#include "PxPvdDataStream.h"
#include "foundation/PxMutex.h"

namespace physx
{
namespace profile
{
class PxProfileZone;
class PxProfileMemoryEventBuffer;
}
namespace pvdsdk
{
struct PvdErrorMessage;
class PvdObjectModelMetaData;

DEFINE_PVD_TYPE_NAME_MAP(profile::PxProfileZone, "_debugger_", "PxProfileZone")
DEFINE_PVD_TYPE_NAME_MAP(profile::PxProfileMemoryEventBuffer, "_debugger_", "PxProfileMemoryEventBuffer")
DEFINE_PVD_TYPE_NAME_MAP(PvdErrorMessage, "_debugger_", "PvdErrorMessage")
// All event streams are on the 'events' property of objects of these types
static inline NamespacedName getMemoryEventTotalsClassName()
{
	return NamespacedName("_debugger", "MemoryEventTotals");
}

class PvdOMMetaDataProvider
{
  protected:
	virtual ~PvdOMMetaDataProvider()
	{
	}

  public:
	virtual void addRef() = 0;
	virtual void release() = 0;
	virtual PvdObjectModelMetaData& lock() = 0;
	virtual void unlock() = 0;
	virtual bool createInstance(const NamespacedName& clsName, const void* instance) = 0;
	virtual bool isInstanceValid(const void* instance) = 0;
	virtual void destroyInstance(const void* instance) = 0;
	virtual int32_t getInstanceClassType(const void* instance) = 0;
};

class PvdCommStreamEmbeddedTypes
{
  public:
	static const char* getProfileEventStreamSemantic()
	{
		return "profile event stream";
	}
	static const char* getMemoryEventStreamSemantic()
	{
		return "memory event stream";
	}
	static const char* getRendererEventStreamSemantic()
	{
		return "render event stream";
	}
};

class PvdCommStreamEventBufferClient;

template <typename TStreamType>
struct EventStreamifier : public PvdEventSerializer
{
	TStreamType& mBuffer;
	EventStreamifier(TStreamType& buf) : mBuffer(buf)
	{
	}

	template <typename TDataType>
	void write(const TDataType& type)
	{
		mBuffer.write(reinterpret_cast<const uint8_t*>(&type), sizeof(TDataType));
	}
	template <typename TDataType>
	void write(const TDataType* type, uint32_t count)
	{
		mBuffer.write(reinterpret_cast<const uint8_t*>(type), count * sizeof(TDataType));
	}

	void writeRef(DataRef<const uint8_t> data)
	{
		uint32_t amount = static_cast<uint32_t>(data.size());
		write(amount);
		write(data.begin(), amount);
	}
	void writeRef(DataRef<StringHandle> data)
	{
		uint32_t amount = static_cast<uint32_t>(data.size());
		write(amount);
		write(data.begin(), amount);
	}
	template <typename TDataType>
	void writeRef(DataRef<TDataType> data)
	{
		uint32_t amount = static_cast<uint32_t>(data.size());
		write(amount);
		for(uint32_t idx = 0; idx < amount; ++idx)
		{
			TDataType& dtype(const_cast<TDataType&>(data[idx]));
			dtype.serialize(*this);
		}
	}

	virtual void streamify(uint16_t& val) PX_OVERRIDE
	{
		write(val);
	}
	virtual void streamify(uint8_t& val) PX_OVERRIDE
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
	virtual void streamify(uint64_t& val) PX_OVERRIDE
	{
		write(val);
	}
	virtual void streamify(PxDebugText& val)
	{
		write(val.color);
		write(val.position);
		write(val.size);
		streamify(val.string);
	}

	virtual void streamify(String& val) PX_OVERRIDE
	{
		uint32_t len = 0;
		String temp = nonNull(val);
		if(*temp)
			len = static_cast<uint32_t>(strnlen(temp, UINT32_MAX - 1) + 1);
		write(len);
		write(val, len);
	}
	virtual void streamify(DataRef<const uint8_t>& val) PX_OVERRIDE
	{
		writeRef(val);
	}
	virtual void streamify(DataRef<NameHandleValue>& val) PX_OVERRIDE
	{
		writeRef(val);
	}
	virtual void streamify(DataRef<StreamPropMessageArg>& val) PX_OVERRIDE
	{
		writeRef(val);
	}
	virtual void streamify(DataRef<StringHandle>& val) PX_OVERRIDE
	{
		writeRef(val);
	}

  private:
	EventStreamifier& operator=(const EventStreamifier&);
};

struct MeasureStream
{
	uint32_t mSize;
	MeasureStream() : mSize(0)
	{
	}
	template <typename TDataType>
	void write(const TDataType& val)
	{
		mSize += sizeof(val);
	}
	template <typename TDataType>
	void write(const TDataType*, uint32_t count)
	{
		mSize += sizeof(TDataType) * count;
	}
};

struct DataStreamState
{
	enum Enum
	{
		Open,
		SetPropertyValue,
		PropertyMessageGroup
	};
};

} // pvdsdk
} // physx
#endif

