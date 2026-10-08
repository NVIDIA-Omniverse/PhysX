// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxPreprocessor.h"
#include "foundation/PxAssert.h"
#include "foundation/PxAllocatorCallback.h"
#include "foundation/PxMemory.h"
#include "foundation/PxBitUtils.h"
#include "extensions/PxDefaultStreams.h"

#include "SnFile.h"
#include "foundation/PxUtilities.h"

#include <errno.h>

using namespace physx;

PxDefaultMemoryOutputStream::PxDefaultMemoryOutputStream(PxAllocatorCallback &allocator) 
:	mAllocator	(allocator)	
,	mData		(NULL)
,	mSize		(0)
,	mCapacity	(0)
{
}

PxDefaultMemoryOutputStream::~PxDefaultMemoryOutputStream()
{
	if(mData)
		mAllocator.deallocate(mData);
}

PxU64 PxDefaultMemoryOutputStream::write(const void* src, PxU64 size)
{
	PxU64 expectedSize = mSize + size;
	if(expectedSize > mCapacity)
	{
		mCapacity = PxMax(PxNextPowerOfTwo(expectedSize), uint64_t(4096));

		PxU8* newData = reinterpret_cast<PxU8*>(mAllocator.allocate(mCapacity,"PxDefaultMemoryOutputStream",__FILE__,__LINE__));
		PX_ASSERT(newData!=NULL);

		PxMemCopy(newData, mData, mSize);
		if(mData)
			mAllocator.deallocate(mData);

		mData = newData;
	}
	PxMemCopy(mData+mSize, src, size);
	mSize += size;
	return size;
}

///////////////////////////////////////////////////////////////////////////////

PxDefaultMemoryInputData::PxDefaultMemoryInputData(const PxU8* data, PxU64 length) :
	mSize	(length),
	mData	(data),
	mPos	(0)
{
}

PxU64 PxDefaultMemoryInputData::read(void* dest, PxU64 count)
{
	PxU64 length = PxMin<PxU64>(count, mSize-mPos);
	PxMemCopy(dest, mData+mPos, length);
	mPos += length;
	return length;
}

PxU64 PxDefaultMemoryInputData::getLength() const
{
	return mSize;
}

void PxDefaultMemoryInputData::seek(PxU64 offset)
{
	mPos = PxMin<PxU64>(mSize, offset);
}

PxU64 PxDefaultMemoryInputData::tell() const
{
	return mPos;
}

PxDefaultFileOutputStream::PxDefaultFileOutputStream(const char* filename)
{
	mFile = NULL;
	sn::fopen_s(&mFile, filename, "wb");
	// PT: when this fails, check that:
	// - the path is correct
	// - the file does not already exist. If it does, check that it is not write protected.
	if(NULL == mFile)
	{
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, 
			"Unable to open file %s, errno 0x%x\n",filename,errno);
	}
	PX_ASSERT(mFile);
}

PxDefaultFileOutputStream::~PxDefaultFileOutputStream()
{
	if(mFile)
		fclose(mFile);
	mFile = NULL;
}

PxU64 PxDefaultFileOutputStream::write(const void* src, PxU64 count)
{
	return mFile ? fwrite(src, 1, count, mFile) : 0;
}

bool PxDefaultFileOutputStream::isValid()
{
	return mFile != NULL;
}

///////////////////////////////////////////////////////////////////////////////

PxDefaultFileInputData::PxDefaultFileInputData(const char* filename)
{
	mFile = NULL;
	sn::fopen_s(&mFile, filename, "rb");

	if(mFile)
	{
		fseek(mFile, 0, SEEK_END);
		mLength = ftell(mFile);
		fseek(mFile, 0, SEEK_SET);
	}
	else
	{
		mLength = 0;
	}
}

PxDefaultFileInputData::~PxDefaultFileInputData()
{
	if(mFile)
		fclose(mFile);
}

PxU64 PxDefaultFileInputData::read(void* dest, PxU64 count)
{
	PX_ASSERT(mFile);
	const size_t size = fread(dest, 1, count, mFile);
	// there should be no assert here since by spec of PxInputStream we can read fewer bytes than expected
	return size;
}

PxU64 PxDefaultFileInputData::getLength() const
{
	return mLength;
}

void PxDefaultFileInputData::seek(PxU64 pos)
{
	fseek(mFile, long(pos), SEEK_SET);
}

PxU64 PxDefaultFileInputData::tell() const
{
	return ftell(mFile);
}

bool PxDefaultFileInputData::isValid() const
{
	return mFile != NULL;
}
