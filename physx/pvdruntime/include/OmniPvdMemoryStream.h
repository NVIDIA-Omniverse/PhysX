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
 * Used to get independently opened and closed read and write views of a shared FIFO. A newly
 * created wrapper has no backing storage, so neither view opens until a nonzero setBufferSize()
 * request succeeds. A failed open leaves that view closed and can be retried after storage exists.
 * Closing the write view does not discard bytes that are waiting for the read view; this permits
 * the normal writer-close/read-open handoff for an in-memory recording.
 * The wrapper owns both views: do not destroy them separately or use them after destroying the
 * OmniPvdMemoryStream.
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
	 * \return A non-null borrowed read-stream view owned by this wrapper
	 */
	virtual OmniPvdReadStream* OMNI_PVD_CALL getReadStream() = 0;

	/**
	 * \brief Used to get the write stream
	 *
	 * \return A non-null borrowed write-stream view owned by this wrapper
	 */
	virtual OmniPvdWriteStream* OMNI_PVD_CALL getWriteStream() = 0;

	/**
	 * \brief Sets the buffer size in bytes of the memory streams
	 *
	 * A request is accepted only while both stream views are closed and the requested size is
	 * nonzero and at least the current size. Every accepted equal-size or growth request
	 * destructively discards all queued bytes and resets both the read and write cursors to byte
	 * zero. A zero-size request or a request made while either view is open fails and returns zero.
	 * A nonzero request to shrink the buffer returns its current size. Every rejected zero-size,
	 * open-view, or shrink request leaves the existing storage, queued bytes, and both cursors intact.
	 *
	 * \return The allocated length after an accepted request, zero for a zero-size request or if
	 * either view is open, or the unchanged current length for a rejected nonzero shrink request.
	 */
	virtual uint64_t OMNI_PVD_CALL setBufferSize(uint64_t bufferLength) = 0;
};

#endif
