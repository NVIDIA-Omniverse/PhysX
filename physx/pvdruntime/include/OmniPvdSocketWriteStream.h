// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_SOCKET_WRITE_STREAM_H
#define OMNI_PVD_SOCKET_WRITE_STREAM_H

#include "OmniPvdWriteStream.h"

/**
 * \brief A TCP socket write stream: streams the OmniPVD command stream to a connected peer.
 *
 * This stream is the TCP client. openStream() connects to a listening reader and sends the
 * live-stream handshake before any OmniPVD bytes; the first writeBytes then goes out on the
 * already-open connection. The endpoint is fixed at creation and cannot be changed afterwards.
 * Opening an already connected stream is a no-op. closeStream() is idempotent, and opening the
 * same object after close reconnects to that fixed endpoint. A failed connect or handshake leaves
 * the stream closed so openStream() can be retried. Before a successful open and after close,
 * writeBytes() returns zero and flush() returns false.
 *
 * The host, port and sendTimeout are arguments of the factory function
 * createOmniPvdSocketWriteStream(address, port, sendTimeout), not of this abstract class
 * constructor; obtain an instance via that factory.
 */
class OmniPvdSocketWriteStream : public OmniPvdWriteStream
{
public:
	virtual ~OmniPvdSocketWriteStream()
	{
	}

	/**
	 * \brief Opens the write stream: connects to the listening reader at the endpoint fixed at
	 * creation (this stream is the TCP client) and sends the live-stream handshake before the
	 * first OmniPVD bytes go out.
	 *
	 * Unlike the read stream (a server that must listen first), the client connects on open,
	 * so opening late is fine -- the reader just needs to be listening by then. Connection attempts
	 * are nonblocking internally and share a retry window of sendTimeout milliseconds. A zero
	 * sendTimeout uses the default 3000 millisecond connect window while leaving the blocked-send
	 * timeout at the OS default. DNS resolution and the handshake are outside this retry window.
	 * openStream() blocks the calling thread through connection and handshake; it spawns no threads
	 * of its own. Idempotent: a second call while already open is a no-op that returns true. A failed
	 * open may be retried, and re-opening after closeStream() clears the dead-peer flag and reconnects.
	 *
	 * \return True if the connect and handshake succeeded.
	 */
	virtual bool OMNI_PVD_CALL openStream() = 0;

	/**
	 * \brief Closes the write stream and the underlying socket. Idempotent.
	 *
	 * Closing disconnects the peer, which a connected reader observes as a clean end-of-stream that
	 * ends its read loop like a file EOF. This is the supported way to unblock a reader parked in
	 * readBytes()/skipBytes() (see OmniPvdSocketReadStream::closeStream()).
	 *
	 * \return True (close always succeeds).
	 */
	virtual bool OMNI_PVD_CALL closeStream() = 0;

protected:
	// Factory-created interface: only the derived Impl constructs it (via
	// createOmniPvdSocketWriteStream); callers cannot instantiate or copy it directly.
	OmniPvdSocketWriteStream()
	{
	}

private:
	OmniPvdSocketWriteStream(const OmniPvdSocketWriteStream&);
	OmniPvdSocketWriteStream& operator=(const OmniPvdSocketWriteStream&);
};

#endif
