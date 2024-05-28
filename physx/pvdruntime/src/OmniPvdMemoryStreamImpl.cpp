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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "OmniPvdMemoryStreamImpl.h"
#include "OmniPvdMemoryReadStreamImpl.h"
#include "OmniPvdMemoryWriteStreamImpl.h"
#include <string.h>

OmniPvdMemoryStreamImpl::OmniPvdMemoryStreamImpl()
{
	mReadStream = 0;
	mWriteStream = 0;

	mBuffer = 0;
	mBufferLength = 0;
	mWrittenBytes = 0;
	mWritePosition = 0;
	mReadPosition = 0;

	mReadStream = new OmniPvdMemoryReadStreamImpl();
	mReadStream->mMemoryStream = this;

	mWriteStream = new OmniPvdMemoryWriteStreamImpl();
	mWriteStream->mMemoryStream = this;
}

OmniPvdMemoryStreamImpl::~OmniPvdMemoryStreamImpl()
{
	delete[] mBuffer;
	delete mReadStream;
	delete mWriteStream;
}

OmniPvdReadStream* OMNI_PVD_CALL OmniPvdMemoryStreamImpl::getReadStream()
{
	return mReadStream;
}

OmniPvdWriteStream* OMNI_PVD_CALL OmniPvdMemoryStreamImpl::getWriteStream()
{
	return mWriteStream;
}

uint64_t OMNI_PVD_CALL OmniPvdMemoryStreamImpl::setBufferSize(uint64_t bufferLength)
{
	if (bufferLength < mBufferLength)
	{
		return mBufferLength;
	}
	delete[] mBuffer;
	mBuffer = new uint8_t[bufferLength];
	mBufferLength = bufferLength;
	mWrittenBytes = 0;
	mWritePosition = 0;
	mReadPosition = 0;
	return bufferLength;
}

uint64_t OMNI_PVD_CALL OmniPvdMemoryStreamImpl::readBytes(uint8_t* destination, uint64_t nbrBytes)
{
	if (mWrittenBytes < 1) return 0;

	////////////////////////////////////////////////////////////////////////////////
	// Limit the number of bytes requested to requestedReadBytes
	////////////////////////////////////////////////////////////////////////////////
	uint64_t requestedReadBytes = nbrBytes;
	if (requestedReadBytes > mBufferLength)
	{
		requestedReadBytes = mBufferLength;
	}
	if (requestedReadBytes > mWrittenBytes)
	{
		return 0;
	}

	////////////////////////////////////////////////////////////////////////////////
	// Separate the reading of bytes into two operations:
	//  tail bytes : bytes from mReadPosition until maximum the end of the buffer
	//  head bytes : bytes from start of the buffer until maximum the mReadPosition-1
	////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////////////////
	// Tail bytes
	////////////////////////////////////////////////////////////////////////////////
	uint64_t tailBytes = mBufferLength - mReadPosition;
	if (tailBytes > requestedReadBytes)
	{
		tailBytes = requestedReadBytes;
	}
	if (destination)
	{
		memcpy(destination, mBuffer + mReadPosition, tailBytes);
	}
	////////////////////////////////////////////////////////////////////////////////
	// Head bytes
	////////////////////////////////////////////////////////////////////////////////
	uint64_t headBytes = requestedReadBytes - tailBytes;
	if (destination)
	{
		memcpy(destination + tailBytes, mBuffer, headBytes);
	}
	////////////////////////////////////////////////////////////////////////////////
	// Update the internal parameters : mReadPosition and mWrittenBytes
	////////////////////////////////////////////////////////////////////////////////
	mReadPosition += requestedReadBytes;
	if (mReadPosition >= mBufferLength)
	{
		mReadPosition -= mBufferLength;
	}
	mWrittenBytes -= requestedReadBytes;
	return requestedReadBytes;
}

uint64_t OmniPvdMemoryStreamImpl::skipBytes(uint64_t nbrBytes)
{
	return readBytes(0, nbrBytes);
}

uint64_t OmniPvdMemoryStreamImpl::writeBytes(const uint8_t* source, uint64_t nbrBytes)
{
	if (mWrittenBytes >= mBufferLength)
	{
		return 0;
	}

	////////////////////////////////////////////////////////////////////////////////
	// Limit the number of bytes requested to requestedWriteBytes
	////////////////////////////////////////////////////////////////////////////////
	uint64_t requestedWriteBytes = nbrBytes;
	if (requestedWriteBytes > mBufferLength)
	{
		requestedWriteBytes = mBufferLength;
	}
	uint64_t writeBytesLeft = mBufferLength - mWrittenBytes;
	if (requestedWriteBytes > writeBytesLeft)
	{
		return 0;
		//requestedWriteBytes = writeBytesLeft;
	}

	////////////////////////////////////////////////////////////////////////////////
	// Tail bytes
	////////////////////////////////////////////////////////////////////////////////
	uint64_t tailBytes = mBufferLength - mWritePosition;
	if (tailBytes > requestedWriteBytes)
	{
		tailBytes = requestedWriteBytes;
	}
	if (source)
	{
		memcpy(mBuffer + mWritePosition, source, tailBytes);
	}
	////////////////////////////////////////////////////////////////////////////////
	// Head bytes
	////////////////////////////////////////////////////////////////////////////////
	uint64_t headBytes = requestedWriteBytes - tailBytes;
	if (source)
	{
		memcpy(mBuffer, source + tailBytes, headBytes);
	}
	////////////////////////////////////////////////////////////////////////////////
	// Update the internal parameters : mReadPosition and mWrittenBytes
	////////////////////////////////////////////////////////////////////////////////
	mWritePosition += requestedWriteBytes;
	if (mWritePosition >= mBufferLength)
	{
		mWritePosition -= mBufferLength;
	}
	mWrittenBytes += requestedWriteBytes;
	return requestedWriteBytes;
}

bool OmniPvdMemoryStreamImpl::flush()
{
	return true;
}

