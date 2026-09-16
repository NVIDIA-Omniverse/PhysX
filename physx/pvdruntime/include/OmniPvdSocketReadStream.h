// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_SOCKET_READ_STREAM_H
#define OMNI_PVD_SOCKET_READ_STREAM_H

#include "OmniPvdReadStream.h"

/**
 * \brief A TCP socket read stream: receives the OmniPVD command stream from a connected peer.
 *
 * This stream is the TCP SERVER. It listens and accepts the producer when openStream() is
 * called, either explicitly or by OmniPvdReader::startReading(). OmniPvdReader::setReadStream()
 * only binds the stream and does not perform I/O. Because this endpoint is the server,
 * activation must begin before the producer connects so it is already listening when the
 * producer's write stream connects on its first write. This open-timing asymmetry against the
 * write stream (which is a client that connects on first write) is intentional. readBytes
 * blocks until the requested bytes arrive; a peer disconnect ends the read loop like a file
 * EOF.
 *
 * Opening an already connected stream is a no-op. closeStream() is idempotent, and the same
 * object can be opened again after close to accept a new connection on its fixed port. A failed
 * listen, accept, or handshake leaves the stream closed. After a failed openStream() returns, it
 * can be retried. Before a successful open and after close, readBytes() and skipBytes() return zero.
 *
 * Calls on the same stream must be externally serialized. In particular, closeStream() cannot
 * be called concurrently to cancel a thread blocked in openStream(), readBytes(), or skipBytes().
 *
 * The listening port is fixed at creation: it is passed to the factory function
 * createOmniPvdSocketReadStream(port) and cannot be changed afterwards. This abstract class
 * has no public constructor of its own; obtain an instance via that factory.
 *
 * The listen/accept and the receive are blocking and run on the calling thread (the thread
 * that drives the OmniPvdReader); this stream does not spawn any threads of its own.
 */
class OmniPvdSocketReadStream : public OmniPvdReadStream
{
public:
	virtual ~OmniPvdSocketReadStream()
	{
	}

	/**
	 * \brief Opens the read stream: binds and listens on the port fixed at creation, accepts
	 * the producer (this stream is the TCP server) and consumes the live-stream handshake.
	 *
	 * Call this function directly, or call OmniPvdReader::startReading(), before the producer's
	 * first write so the listener is already active when the producer connects. Blocks on the
	 * calling thread (the thread driving the OmniPvdReader); spawns no threads of its own.
	 * Idempotent: a second call while already open is a no-op that returns true. After failure or
	 * closeStream(), a later call starts a fresh listen/accept and handshake on the fixed port. This
	 * call blocks through both accept and the handshake and cannot be cancelled by calling
	 * closeStream() from another thread. Arrange for the peer to connect and complete or close the
	 * handshake, then wait for this call to return before closing, reopening, releasing, or destroying
	 * the stream.
	 *
	 * \return True if the listen/accept and handshake succeeded.
	 */
	virtual bool OMNI_PVD_CALL openStream() = 0;

	/**
	 * \brief Closes the read stream and the underlying socket. Idempotent.
	 *
	 * Calls on the same stream must be externally serialized. Do not call closeStream() while another
	 * thread is blocked in openStream(), readBytes(), or skipBytes(); concurrent close is unsupported
	 * and does not provide a cancellation mechanism. Wait for the in-progress operation to return
	 * before closing, reopening, releasing, or destroying the stream. A connected read can be ended by
	 * closing or disconnecting the peer's write stream, which is observed as end-of-stream.
	 *
	 * \return True (close always succeeds).
	 */
	virtual bool OMNI_PVD_CALL closeStream() = 0;

protected:
	// Factory-created interface: only the derived Impl constructs it (via
	// createOmniPvdSocketReadStream); callers cannot instantiate or copy it directly.
	OmniPvdSocketReadStream()
	{
	}

private:
	OmniPvdSocketReadStream(const OmniPvdSocketReadStream&);
	OmniPvdSocketReadStream& operator=(const OmniPvdSocketReadStream&);
};

#endif
