// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PVD_INTERNAL_BYTE_STREAMS_H
#define PX_PVD_INTERNAL_BYTE_STREAMS_H

#include "PxPvdByteStreams.h"
#include "PxPvdFoundation.h"

namespace physx
{
namespace pvdsdk
{
struct MemPvdInputStream : public PvdInputStream
{
	const uint8_t* mBegin;
	const uint8_t* mEnd;
	bool mGood;

	MemPvdInputStream(const uint8_t* beg = NULL, const uint8_t* end = NULL)
	{
		mBegin = beg;
		mEnd = end;
		mGood = true;
	}

	uint32_t size() const
	{
		return mGood ? static_cast<uint32_t>(mEnd - mBegin) : 0;
	}
	bool isGood() const
	{
		return mGood;
	}

	void setup(uint8_t* start, uint8_t* stop)
	{
		mBegin = start;
		mEnd = stop;
	}

	void nocopyRead(uint8_t*& buffer, uint32_t& len)
	{
		if(len == 0 || mGood == false)
		{
			len = 0;
			buffer = NULL;
			return;
		}
		uint32_t original = len;
		len = PxMin(len, size());
		if(mGood && len != original)
			mGood = false;
		buffer = const_cast<uint8_t*>(mBegin);
		mBegin += len;
	}

	virtual bool read(uint8_t* buffer, uint32_t& len) PX_OVERRIDE
	{
		if(len == 0)
			return true;
		uint32_t original = len;
		len = PxMin(len, size());

		physx::intrinsics::memCopy(buffer, mBegin, len);
		mBegin += len;
		if(len < original)
			physx::intrinsics::memZero(buffer + len, original - len);
		mGood = mGood && len == original;
		return mGood;
	}
};
}
}
#endif

