// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PVD_BYTE_STREAMS_H
#define PX_PVD_BYTE_STREAMS_H

#include "PxPvdObjectModelBaseTypes.h"

namespace physx
{
namespace pvdsdk
{

static inline uint32_t strLen(const char* inStr)
{
	uint32_t len = 0;
	if(inStr)
	{
		while(*inStr)
		{
			++len;
			++inStr;
		}
	}
	return len;
}

class PvdInputStream
{
  protected:
	virtual ~PvdInputStream()
	{
	}

  public:
	// Return false if you can't write the number of bytes requested
	// But make an absolute best effort to read the data...
	virtual bool read(uint8_t* buffer, uint32_t& len) = 0;

	template <typename TDataType>
	bool read(TDataType* buffer, uint32_t numItems)
	{
		uint32_t expected = numItems;
		uint32_t amountToRead = numItems * sizeof(TDataType);
		read(reinterpret_cast<uint8_t*>(buffer), amountToRead);
		numItems = amountToRead / sizeof(TDataType);
		PX_ASSERT(numItems == expected);
		return expected == numItems;
	}

	template <typename TDataType>
	PvdInputStream& operator>>(TDataType& data)
	{
		uint32_t dataSize = static_cast<uint32_t>(sizeof(TDataType));
		bool success = read(reinterpret_cast<uint8_t*>(&data), dataSize);
		// PX_ASSERT( success );
		// PX_ASSERT( dataSize == sizeof( data ) );
		(void)success;
		return *this;
	}
};

class PvdOutputStream
{
  protected:
	virtual ~PvdOutputStream()
	{
	}

  public:
	// Return false if you can't write the number of bytes requested
	// But make an absolute best effort to write the data...
	virtual bool write(const uint8_t* buffer, uint32_t len) = 0;
	virtual bool directCopy(PvdInputStream& inStream, uint32_t len) = 0;

	template <typename TDataType>
	bool write(const TDataType* buffer, uint32_t numItems)
	{
		return write(reinterpret_cast<const uint8_t*>(buffer), numItems * sizeof(TDataType));
	}

	template <typename TDataType>
	PvdOutputStream& operator<<(const TDataType& data)
	{
		bool success = write(reinterpret_cast<const uint8_t*>(&data), sizeof(data));
		PX_ASSERT(success);
		(void)success;
		return *this;
	}

	PvdOutputStream& operator<<(const char* inString)
	{
		if(inString && *inString)
		{
			uint32_t len(strLen(inString));
			write(inString, len);
		}
		return *this;
	}
};
}
}
#endif
