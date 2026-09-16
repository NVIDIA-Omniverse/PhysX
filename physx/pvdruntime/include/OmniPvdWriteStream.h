// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_WRITE_STREAM_H
#define OMNI_PVD_WRITE_STREAM_H

#include "OmniPvdDefines.h"

/**
 * \brief Abstract byte-oriented write endpoint for an OmniPVD transport.
 *
 * A write stream starts closed. openStream() and closeStream() are explicit, idempotent
 * lifecycle operations; a failed direct open leaves the stream closed and may be retried by
 * another direct call. Reopening after a successful close is supported, with target/session
 * behavior documented by the concrete transport.
 *
 * writeBytes() and flush() never open the stream. A closed write returns zero and a closed
 * flush returns false. Short writes are valid transport results. The interface makes no
 * general thread-safety guarantee, so lifecycle and data access must be externally serialized
 * unless a concrete transport documents otherwise.
 * Production implementations close an open transport from their destructor as a safety net;
 * explicit close before destruction or the matching release remains the normal lifecycle.
 *
 * OmniPvdWriter only borrows a bound stream: it never closes or destroys it. The writer lazily
 * calls openStream() before its first command. If that lazy open fails, the writer suppresses later
 * attempts until OmniPvdWriter::clearStatus() or OmniPvdWriter::setWriteStream() explicitly starts
 * a new retry epoch; the endpoint remains independently retryable through direct openStream() calls.
 * The caller must keep the stream alive until all writes have quiesced or the writer has been
 * rebound to another live stream, then close and destroy/release it through its owner. Closing and
 * reopening alone does not reset writer state; rebind with OmniPvdWriter::setWriteStream() to reset
 * the writer session. The next lazy open still follows the endpoint's reopen policy: for example, a
 * closed file writer truncates, while an open stream appends a versioned segment at its current
 * position. Preserve that boundary for positioned decoding. For one standalone recording, use a
 * new or reset transport, or reopen a file writer so it truncates.
 */
class OmniPvdWriteStream
{
public:
	virtual ~OmniPvdWriteStream()
	{
	}

	/**
	 * \brief Writes bytes to the open stream.
	 *
	 * This operation does not implicitly open the stream. It returns zero while closed.
	 *
	 * \param bytes Pointer to the bytes to write
	 * \param nbrBytes The requested number of bytes to write
	 * \return The actual number of bytes written, which may be less than nbrBytes
	 */
	virtual uint64_t OMNI_PVD_CALL writeBytes(const uint8_t* bytes, uint64_t nbrBytes) = 0;

	/**
	 * \brief Flushes buffered writes on the open stream.
	 *
	 * This operation does not implicitly open the stream and returns false while closed.
	 *
	 * \return True if the flush succeeded
	 */
	virtual bool OMNI_PVD_CALL flush() = 0;

	/**
	 * \brief Opens the write stream.
	 *
	 * Calling this on an already-open stream succeeds without resetting the current session.
	 * A failed direct open leaves the stream closed and retryable by another direct call. A bound
	 * OmniPvdWriter requires clearStatus() or setWriteStream() before it retries a latched lazy-open
	 * failure. Opening may block according to the concrete transport.
	 *
	 * \return True if the stream is open, false if opening failed
	 */
	virtual bool OMNI_PVD_CALL openStream() = 0;

	/**
	 * \brief Closes the write stream.
	 *
	 * Calling this on an already-closed stream succeeds. Closing does not destroy the stream;
	 * target/session behavior on a later open is transport-specific. A false result can report a
	 * transport-specific final flush or close failure even though the endpoint has transitioned to
	 * closed, and buffered bytes may have been lost. Callers should close explicitly and check the
	 * result rather than rely on destructor cleanup.
	 *
	 * \return True if finalization succeeded or the stream was already closed, false if finalization failed
	 */
	virtual bool OMNI_PVD_CALL closeStream() = 0;
};

#endif
