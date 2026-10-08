// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_READ_STREAM_H
#define OMNI_PVD_READ_STREAM_H

#include "OmniPvdDefines.h"

/**
 * \brief Abstract byte-oriented read endpoint for an OmniPVD transport.
 *
 * A read stream starts closed. openStream() and closeStream() are explicit, idempotent
 * lifecycle operations; a failed open leaves the stream closed and may be retried. Reopening
 * after a successful close is supported, with cursor/session behavior documented by the
 * concrete transport.
 *
 * readBytes() and skipBytes() never open the stream. Both return zero while it is closed, and
 * short counts are valid transport results. The interface makes no general thread-safety
 * guarantee, so lifecycle and data access must be externally serialized unless a concrete
 * transport documents otherwise.
 * Production implementations close an open transport from their destructor as a safety net;
 * explicit close before destruction or the matching release remains the normal lifecycle.
 *
 * OmniPvdReader only borrows a bound stream: it never closes or destroys it. Binding performs
 * no I/O; reader activation calls openStream() and may block for transports such as TCP. The
 * caller must keep the stream alive until reader access has quiesced or the reader has been
 * rebound to another live stream, then close and destroy/release it through its owner.
 */
class OmniPvdReadStream
{
public:
	virtual ~OmniPvdReadStream()
	{
	}

	/**
	 * \brief Reads bytes from the open stream.
	 *
	 * This operation does not implicitly open the stream. It returns zero while closed.
	 *
	 * \param bytes Destination for the bytes read
	 * \param nbrBytes The requested number of bytes to read
	 * \return The actual number of bytes read, which may be less than nbrBytes
	 */
	virtual uint64_t OMNI_PVD_CALL readBytes(uint8_t* bytes, uint64_t nbrBytes) = 0;

	/**
	 * \brief Skips bytes in the open stream.
	 *
	 * This operation does not implicitly open the stream. It returns zero while closed.
	 *
	 * \param nbrBytes The requested number of bytes to skip
	 * \return The actual number of bytes skipped, which may be less than nbrBytes
	 */
	virtual uint64_t OMNI_PVD_CALL skipBytes(uint64_t nbrBytes) = 0;
	
	/**
	 * \brief Opens the read stream.
	 *
	 * Calling this on an already-open stream succeeds without resetting the current session.
	 * A failed open leaves the stream closed and retryable. Opening may block according to the
	 * concrete transport.
	 *
	 * \return True if the stream is open, false if opening failed
	 */
	virtual bool OMNI_PVD_CALL openStream() = 0;

	/**
	 * \brief Closes the read stream.
	 *
	 * Calling this on an already-closed stream succeeds. Closing does not destroy the stream;
	 * cursor/session behavior on a later open is transport-specific.
	 *
	 * \return True if the stream is closed
	 */
	virtual bool OMNI_PVD_CALL closeStream() = 0;
};

#endif
