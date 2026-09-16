// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_MEMORY_STREAM_IMPL_H
#define OMNI_PVD_MEMORY_STREAM_IMPL_H

#include "OmniPvdMemoryStream.h"

class OmniPvdMemoryReadStreamImpl;
class OmniPvdMemoryWriteStreamImpl;

class OmniPvdMemoryStreamImpl : public OmniPvdMemoryStream
{
public:
	OmniPvdMemoryStreamImpl();
	~OmniPvdMemoryStreamImpl();

	OmniPvdReadStream* OMNI_PVD_CALL getReadStream();
	OmniPvdWriteStream* OMNI_PVD_CALL getWriteStream();
	uint64_t OMNI_PVD_CALL setBufferSize(uint64_t bufferLength);
	bool hasBuffer() const;

	////////////////////////////////////////////////////////////////////////////////
	// Read part
	////////////////////////////////////////////////////////////////////////////////
	uint64_t readBytes(uint8_t* destination, uint64_t nbrBytes);
	uint64_t skipBytes(uint64_t nbrBytes);
	
	////////////////////////////////////////////////////////////////////////////////
	// Write part
	////////////////////////////////////////////////////////////////////////////////
	uint64_t writeBytes(const uint8_t* source, uint64_t nbrBytes);
	bool flush();

	////////////////////////////////////////////////////////////////////////////////
	// Read/write streams
	////////////////////////////////////////////////////////////////////////////////
	OmniPvdMemoryReadStreamImpl *mReadStream;
	OmniPvdMemoryWriteStreamImpl *mWriteStream;

	////////////////////////////////////////////////////////////////////////////////
	// Round robin buffer
	////////////////////////////////////////////////////////////////////////////////
	uint8_t *mBuffer;
	uint64_t mBufferLength;
	uint64_t mWrittenBytes;
	uint64_t mWritePosition;
	uint64_t mReadPosition;
};

#endif
