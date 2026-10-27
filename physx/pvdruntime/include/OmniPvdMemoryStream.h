// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_MEMORY_STREAM_H
#define OMNI_PVD_MEMORY_STREAM_H

#include "OmniPvdReadStream.h"
#include "OmniPvdWriteStream.h"

/**
 * \brief Used to abstract a memory read/write stream
 *
 * Used to get the read and write streams. 
 */
class OmniPvdMemoryStream
{
public:
	virtual ~OmniPvdMemoryStream()
	{
	}
	/**
	 * \brief Used to get the read stream
	 *
	 * \return The read stream
	 */
	virtual OmniPvdReadStream* OMNI_PVD_CALL getReadStream() = 0;

	/**
	 * \brief Used to get the write stream
	 *
	 * \return The write stream
	 */
	virtual OmniPvdWriteStream* OMNI_PVD_CALL getWriteStream() = 0;

	/**
	 * \brief Sets the buffer size in bytes of the memory streams
	 *
	 * \return The actually allocated length of the memory stream
	 */
	virtual uint64_t OMNI_PVD_CALL setBufferSize(uint64_t bufferLength) = 0;
};

#endif
