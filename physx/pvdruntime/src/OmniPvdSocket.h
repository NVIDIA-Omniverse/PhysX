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

#ifndef OMNI_PVD_SOCKET_H
#define OMNI_PVD_SOCKET_H

#include "OmniPvdDefines.h"
#include <stdint.h>

//
// A tiny blocking TCP socket wrapper for the OmniPVD live stream. It has NO
// dependency on PhysXFoundation / PxSocket on purpose, so it can live inside the
// standalone pvdruntime shared library. It is pure byte transport and knows
// nothing about the OmniPVD protocol or the live-stream handshake.
//
// The native socket handle is stored in a platform-neutral integer so this header
// never needs to include winsock2.h (which would leak into every consumer of the
// socket stream impls). The actual socket type is cast inside the .cpp.
//
class OmniPvdSocket
{
public:
	OmniPvdSocket();
	~OmniPvdSocket();

	// Server side: bind to INADDR_ANY:port, listen, and accept exactly one client
	// (all blocking). Returns true once a client is connected.
	bool listenAndAccept(uint16_t port);

	// Split, non-blocking-friendly server primitives: beginListen() does the bind+listen
	// and returns immediately; acceptOne() does the blocking accept. Threading contract:
	// close() from another thread is the ONLY sanctioned concurrent call -- it closes the
	// listen socket so a thread blocked in acceptOne() is woken (the OS aborts the blocked
	// accept) and acceptOne() then returns false. No other method may run concurrently on the
	// same OmniPvdSocket; the handle members are not internally synchronized, so callers must
	// not race acceptOne()/connect()/send()/recv() against each other. (The read stream
	// layered on top runs acceptOne() on its own thread; that thread-creation aspect is
	// documented in OmniPvdSocketReadStream.)
	bool beginListen(uint16_t port);
	bool acceptOne();

	// Close only the accepted data connection, keeping the listen socket open so a new
	// client can be accepted (used for detach / re-attach).
	void closeDataOnly();

	// Client side: resolve address:port and connect (blocking). Returns true on success.
	bool connect(const char* address, uint16_t port);

	// Sets the bounded SO_SNDTIMEO (the timeout is in milliseconds) applied to the data
	// socket once it is connected/accepted, so a wedged/non-reading peer cannot block a
	// blocking send (and thus the simulation thread) indefinitely. Must be set before
	// connect()/accept().
	void setSendTimeout(uint32_t sendTimeout);

	// One send / one recv. The caller loops for a full transfer.
	// send: returns the number of bytes sent (> 0), or <= 0 on error / dead peer.
	int64_t send(const uint8_t* bytes, uint64_t nbrBytes);
	// recv: returns the number of bytes read (> 0), 0 on an orderly peer close,
	//       or < 0 on error.
	int64_t recv(uint8_t* bytes, uint64_t nbrBytes);

	// Closes the data connection and any listening socket. If nothing is open, a second
	// call is a no-op.
	void close();

	// True while a data connection is established.
	bool isOpen() const;

private:
	OmniPvdSocket(const OmniPvdSocket&);
	OmniPvdSocket& operator=(const OmniPvdSocket&);

	// Apply a bounded SO_SNDTIMEO to the data socket so a wedged/non-reading peer cannot
	// block a blocking send (and thus the simulation thread) indefinitely.
	void applySendTimeout();

	uint32_t mSendTimeoutMs; // SO_SNDTIMEO applied to the data socket (0 = leave at the OS default)

#if defined(OMNI_PVD_WIN)
	uint64_t mListenSocket; // SOCKET (UINT_PTR); INVALID_SOCKET sentinel
	uint64_t mDataSocket;
#else
	int      mListenSocket; // file descriptor; -1 sentinel
	int      mDataSocket;
#endif
};

#endif
