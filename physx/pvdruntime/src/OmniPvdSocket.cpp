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

#include "OmniPvdSocket.h"

#include <stdio.h>
#include <string.h>

#if defined(OMNI_PVD_WIN)
	#include <winsock2.h>
	#include <ws2tcpip.h>
	#define OMNI_PVD_INVALID_SOCKET INVALID_SOCKET
	typedef int OmniPvdSockLen;
	// Native socket-handle type for the winsock calls. mListenSocket / mDataSocket are
	// stored as uint64_t in the header (to keep winsock2.h out of it); the handle must be
	// cast back to SOCKET (UINT_PTR), NOT int, or the upper 32 bits are lost on Win64 -- a
	// SOCKET is an opaque kernel handle that can legitimately exceed INT_MAX.
	typedef SOCKET OmniPvdNativeSocket;
#else
	#include <sys/types.h>
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <netinet/tcp.h>
	#include <arpa/inet.h>
	#include <netdb.h>
	#include <unistd.h>
	#include <errno.h>
	#define OMNI_PVD_INVALID_SOCKET (-1)
	typedef socklen_t OmniPvdSockLen;
	typedef int OmniPvdNativeSocket;
#endif

// Winsock has no MSG_NOSIGNAL (and never raises SIGPIPE); fall back to 0 there.
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

static void omniPvdSleepMs(unsigned ms)
{
#if defined(OMNI_PVD_WIN)
	Sleep(ms);
#else
	usleep(ms * 1000);
#endif
}

#if defined(OMNI_PVD_WIN)
// Winsock is process-wide. A per-socket WSACleanup would tear it down underneath other live
// sockets in the same process (and can cancel another thread's in-flight blocking calls), so
// Winsock is started exactly once via the guard below. The guard's destructor runs at static
// teardown -- including DLL_PROCESS_DETACH, i.e. when PVDRuntime is unloaded while the host
// process keeps running (e.g. a Kit extension disable) -- and pairs the single WSAStartup with
// one WSACleanup, so a load/use/unload cycle does not leak a Winsock reference, yet no per-stream
// cleanup can cancel another stream's pending calls. The C++11 thread-safe function-local static
// makes the one-time startup race-free even if two threads create their first socket at once.
namespace
{
class OmniPvdWinsockGuard
{
public:
	OmniPvdWinsockGuard() : mStarted(false)
	{
		WSADATA wsaData;
		mStarted = (WSAStartup(MAKEWORD(2, 2), &wsaData) == 0);
	}
	~OmniPvdWinsockGuard()
	{
		if (mStarted)
			WSACleanup();
	}
	bool started() const { return mStarted; }
private:
	bool mStarted;
	OmniPvdWinsockGuard(const OmniPvdWinsockGuard&);
	OmniPvdWinsockGuard& operator=(const OmniPvdWinsockGuard&);
};
} // anonymous namespace

static bool omniPvdEnsureWinsock()
{
	static OmniPvdWinsockGuard s_winsock;
	return s_winsock.started();
}
#endif

OmniPvdSocket::OmniPvdSocket()
{
	mSendTimeoutMs = 3000;
	mListenSocket = OMNI_PVD_INVALID_SOCKET;
	mDataSocket = OMNI_PVD_INVALID_SOCKET;
}

void OmniPvdSocket::setSendTimeout(uint32_t sendTimeout)
{
	mSendTimeoutMs = sendTimeout;
}

OmniPvdSocket::~OmniPvdSocket()
{
	close();
}

bool OmniPvdSocket::isOpen() const
{
	return mDataSocket != OMNI_PVD_INVALID_SOCKET;
}

bool OmniPvdSocket::beginListen(uint16_t port)
{
#if defined(OMNI_PVD_WIN)
	if (!omniPvdEnsureWinsock())
		return false;
#endif
	mListenSocket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (mListenSocket == OMNI_PVD_INVALID_SOCKET)
		return false;

	int reuse = 1;
	::setsockopt((OmniPvdNativeSocket)mListenSocket, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));

	sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(port);

	if (::bind((OmniPvdNativeSocket)mListenSocket, (sockaddr*)&addr, sizeof(addr)) != 0 ||
		::listen((OmniPvdNativeSocket)mListenSocket, 1) != 0)
	{
		close();
		return false;
	}
	return true;
}

bool OmniPvdSocket::acceptOne()
{
	if (mListenSocket == OMNI_PVD_INVALID_SOCKET)
		return false;
	// Blocks the CALLING thread until a client connects, or returns false when the
	// listen socket is closed by close() (the way a background acceptor is woken to exit).
	for (;;)
	{
		mDataSocket = ::accept((OmniPvdNativeSocket)mListenSocket, NULL, NULL);
#if !defined(OMNI_PVD_WIN)
		// A signal delivered to the accepting thread interrupts the blocking accept with EINTR;
		// that is not a real failure, so retry (matching the send/recv EINTR loops). A close()
		// from another thread closes the listen socket and yields a different error (EBADF/
		// EINVAL), which falls through to return false -- the intended wake-to-exit path.
		if (mDataSocket == OMNI_PVD_INVALID_SOCKET && errno == EINTR)
			continue;
#endif
		break;
	}
	if (mDataSocket == OMNI_PVD_INVALID_SOCKET)
		return false;
	applySendTimeout();
	return true;
}

void OmniPvdSocket::applySendTimeout()
{
	if (mDataSocket == OMNI_PVD_INVALID_SOCKET || mSendTimeoutMs == 0)
		return; // 0 = leave the send timeout at the OS default
#if defined(OMNI_PVD_WIN)
	DWORD ms = (DWORD)mSendTimeoutMs;
	::setsockopt((SOCKET)mDataSocket, SOL_SOCKET, SO_SNDTIMEO, (const char*)&ms, sizeof(ms));
#else
	struct timeval tv;
	tv.tv_sec = (time_t)(mSendTimeoutMs / 1000u);
	tv.tv_usec = (suseconds_t)((mSendTimeoutMs % 1000u) * 1000u);
	::setsockopt((int)mDataSocket, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv, sizeof(tv));
#endif
}

bool OmniPvdSocket::listenAndAccept(uint16_t port)
{
	return beginListen(port) && acceptOne();
}

void OmniPvdSocket::closeDataOnly()
{
	if (mDataSocket != OMNI_PVD_INVALID_SOCKET)
	{
#if defined(OMNI_PVD_WIN)
		closesocket(mDataSocket);
#else
		::close(mDataSocket);
#endif
		mDataSocket = OMNI_PVD_INVALID_SOCKET;
	}
}

bool OmniPvdSocket::connect(const char* address, uint16_t port)
{
#if defined(OMNI_PVD_WIN)
	if (!omniPvdEnsureWinsock())
		return false;
#endif
	char portStr[16];
	snprintf(portStr, sizeof(portStr), "%u", (unsigned)port);

	addrinfo hints;
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	addrinfo* result = NULL;
	if (getaddrinfo(address ? address : "127.0.0.1", portStr, &hints, &result) != 0 || result == NULL)
		return false;

	// Retry so either side tolerates starting before the other is listening. The 40 x 50ms sleeps
	// bound only the backoff to ~2s; this assumes each ::connect() returns fast (e.g. ECONNREFUSED
	// against a local listener that is not up yet). Against a filtered/unroutable host where connect()
	// itself blocks for the OS connect timeout, a single attempt can dominate and the total runs longer.
	const int maxAttempts = 40;
	bool connected = false;
	for (int attempt = 0; attempt < maxAttempts && !connected; ++attempt)
	{
		for (addrinfo* ai = result; ai != NULL; ai = ai->ai_next)
		{
			mDataSocket = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
			if (mDataSocket == OMNI_PVD_INVALID_SOCKET)
				continue;
			if (::connect((OmniPvdNativeSocket)mDataSocket, ai->ai_addr, (OmniPvdSockLen)ai->ai_addrlen) == 0)
			{
				connected = true;
				break;
			}
#if defined(OMNI_PVD_WIN)
			closesocket(mDataSocket);
#else
			::close(mDataSocket);
#endif
			mDataSocket = OMNI_PVD_INVALID_SOCKET;
		}
		if (!connected)
			omniPvdSleepMs(50);
	}
	freeaddrinfo(result);
	if (!connected)
	{
		close();
		return false;
	}
	applySendTimeout();
	return true;
}

int64_t OmniPvdSocket::send(const uint8_t* bytes, uint64_t nbrBytes)
{
	if (mDataSocket == OMNI_PVD_INVALID_SOCKET)
		return -1;
	// ::send takes an int length on Windows / size_t on POSIX; clamp to INT_MAX so a large
	// nbrBytes can never truncate or wrap negative. The caller loops for the remainder.
	const int len = (nbrBytes > 0x7fffffffu) ? 0x7fffffff : (int)nbrBytes;
	for (;;)
	{
		int64_t sent = (int64_t)::send((OmniPvdNativeSocket)mDataSocket, (const char*)bytes, len, MSG_NOSIGNAL);
#if !defined(OMNI_PVD_WIN)
		if (sent < 0 && errno == EINTR)
			continue;
#endif
		return sent;
	}
}

int64_t OmniPvdSocket::recv(uint8_t* bytes, uint64_t nbrBytes)
{
	if (mDataSocket == OMNI_PVD_INVALID_SOCKET)
		return -1;
	// ::recv takes an int length on Windows / size_t on POSIX; clamp to INT_MAX so a large
	// nbrBytes can never truncate or wrap negative. The caller loops for the remainder.
	const int len = (nbrBytes > 0x7fffffffu) ? 0x7fffffff : (int)nbrBytes;
	for (;;)
	{
		int64_t got = (int64_t)::recv((OmniPvdNativeSocket)mDataSocket, (char*)bytes, len, 0);
#if !defined(OMNI_PVD_WIN)
		if (got < 0 && errno == EINTR)
			continue;
#endif
		return got;
	}
}

void OmniPvdSocket::close()
{
#if defined(OMNI_PVD_WIN)
	if (mDataSocket != OMNI_PVD_INVALID_SOCKET)
	{
		closesocket(mDataSocket);
		mDataSocket = OMNI_PVD_INVALID_SOCKET;
	}
	if (mListenSocket != OMNI_PVD_INVALID_SOCKET)
	{
		closesocket(mListenSocket);
		mListenSocket = OMNI_PVD_INVALID_SOCKET;
	}
#else
	if (mDataSocket != OMNI_PVD_INVALID_SOCKET)
	{
		::close(mDataSocket);
		mDataSocket = OMNI_PVD_INVALID_SOCKET;
	}
	if (mListenSocket != OMNI_PVD_INVALID_SOCKET)
	{
		::close(mListenSocket);
		mListenSocket = OMNI_PVD_INVALID_SOCKET;
	}
#endif
}
