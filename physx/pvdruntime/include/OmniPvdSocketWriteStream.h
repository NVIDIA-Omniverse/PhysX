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

#ifndef OMNI_PVD_SOCKET_WRITE_STREAM_H
#define OMNI_PVD_SOCKET_WRITE_STREAM_H

#include "OmniPvdWriteStream.h"

/**
 * \brief A TCP socket write stream: streams the OmniPVD command stream to a connected peer.
 *
 * This stream is the TCP client. openStream() connects to a listening reader and sends the
 * live-stream handshake before any OmniPVD bytes; the first writeBytes then goes out on the
 * already-open connection. The endpoint is fixed at creation and cannot be changed afterwards.
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
	 * so opening late is fine -- the reader just needs to be listening by then. Blocks on the
	 * calling thread; spawns no threads of its own. Idempotent: a second call while already
	 * open is a no-op that returns true, and re-opening after closeStream() clears the
	 * dead-peer flag and reconnects.
	 *
	 * \return True if the connect and handshake succeeded.
	 */
	virtual bool OMNI_PVD_CALL openStream() = 0;

	/**
	 * \brief Closes the write stream and the underlying socket. Idempotent.
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
