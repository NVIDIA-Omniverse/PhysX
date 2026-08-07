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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef OMNI_PVD_SOCKET_READ_STREAM_H
#define OMNI_PVD_SOCKET_READ_STREAM_H

#include "OmniPvdReadStream.h"

/**
 * \brief A TCP socket read stream: receives the OmniPVD command stream from a connected peer.
 *
 * This stream is the TCP SERVER. It listens and accepts the producer when it is opened
 * (inside openStream(), which OmniPvdReader::setReadStream calls, or which the caller may
 * call directly). Because it is the server, it must be opened EARLY -- before the producer
 * connects -- so it is already listening when the producer's write stream connects on its
 * first write. This open-timing asymmetry against the write stream (which is a client that
 * connects on first write) is intentional: a server must be listening before its client can
 * connect. readBytes blocks until the requested bytes arrive; a peer disconnect ends the
 * read loop like a file EOF.
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
	 * Must be opened EARLY -- before the producer connects -- so the listener is already up
	 * when the producer's write stream connects on its first write. Blocks on the calling
	 * thread (the thread driving the OmniPvdReader); spawns no threads of its own. Idempotent:
	 * a second call while already open is a no-op that returns true.
	 *
	 * \return True if the listen/accept and handshake succeeded.
	 */
	virtual bool OMNI_PVD_CALL openStream() = 0;

	/**
	 * \brief Closes the read stream and the underlying socket. Idempotent.
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
