// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_SOCKET_H
#define OMNI_PVD_SOCKET_H

#include "OmniPvdDefines.h"
#include <stdint.h>

//
// A tiny blocking TCP socket wrapper for the OmniPVD live stream. It has NO
// dependency on PhysXFoundation / PxSocket on purpose, so it can live inside the
// standalone runtime implementation. It is pure byte transport and knows
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

	// Split server primitives: beginListen() does the bind+listen and returns immediately;
	// acceptOne() does the blocking accept. This wrapper is not internally synchronized. The
	// caller must serialize every operation on one OmniPvdSocket, including close(); close()
	// cannot be used from another thread to cancel a blocked acceptOne().
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
