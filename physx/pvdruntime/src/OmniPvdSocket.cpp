// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OmniPvdSocket.h"

#include <chrono>
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
	#include <fcntl.h>
	#include <poll.h>
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

// Connect budget used when no send timeout is configured (mSendTimeoutMs == 0); matches the ctor default.
static const uint32_t OMNI_PVD_DEFAULT_SOCKET_TIMEOUT_MS = 3000;

// Poll granularity for the connect deadline; caps both the writability wait and the backoff sleep.
static const uint32_t OMNI_PVD_CONNECT_POLL_INTERVAL_MS = 50;

static void omniPvdSleepMs(unsigned ms)
{
#if defined(OMNI_PVD_WIN)
	Sleep(ms);
#else
	usleep(ms * 1000);
#endif
}

static void omniPvdCloseNativeSocket(OmniPvdNativeSocket socket)
{
	if (socket == OMNI_PVD_INVALID_SOCKET)
		return;
#if defined(OMNI_PVD_WIN)
	closesocket(socket);
#else
	::close(socket);
#endif
}

// Toggle blocking mode. connect() drives each attempt nonblocking to honor the shared deadline, then
// restores blocking before the socket is handed to send()/recv().
static bool omniPvdSetNonBlocking(OmniPvdNativeSocket socket, bool nonBlocking)
{
#if defined(OMNI_PVD_WIN)
	u_long mode = nonBlocking ? 1 : 0;
	return ::ioctlsocket(socket, FIONBIO, &mode) == 0;
#else
	const int flags = ::fcntl(socket, F_GETFL, 0);
	if (flags < 0)
		return false;
	const int requestedFlags = nonBlocking ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK);
	return ::fcntl(socket, F_SETFL, requestedFlags) == 0;
#endif
}

// True while a nonblocking ::connect() is still in progress (not failed): EINPROGRESS/EALREADY on
// POSIX, WSAEWOULDBLOCK/WSAEINPROGRESS/WSAEALREADY on Windows.
static bool omniPvdConnectIsPending()
{
#if defined(OMNI_PVD_WIN)
	const int error = WSAGetLastError();
	return error == WSAEWOULDBLOCK || error == WSAEINPROGRESS || error == WSAEALREADY;
#else
	return errno == EINPROGRESS || errno == EALREADY;
#endif
}

static bool omniPvdWaitWasInterrupted()
{
#if defined(OMNI_PVD_WIN)
	return WSAGetLastError() == WSAEINTR;
#else
	return errno == EINTR;
#endif
}

// Wait up to waitMilliseconds for a pending connect to resolve. Returns >0 when the socket is ready
// to probe via SO_ERROR, 0 on timeout, <0 on error.
static int omniPvdWaitForConnect(OmniPvdNativeSocket socket, int waitMilliseconds)
{
#if defined(OMNI_PVD_WIN)
	fd_set writable;
	fd_set failed;
	FD_ZERO(&writable);
	FD_ZERO(&failed);
	FD_SET(socket, &writable);
	FD_SET(socket, &failed);
	timeval timeout;
	timeout.tv_sec = waitMilliseconds / 1000;
	timeout.tv_usec = (waitMilliseconds % 1000) * 1000;
	return ::select(0, NULL, &writable, &failed, &timeout);
#else
	pollfd descriptor;
	descriptor.fd = socket;
	descriptor.events = POLLOUT;
	descriptor.revents = 0;
	return ::poll(&descriptor, 1, waitMilliseconds);
#endif
}

#if defined(OMNI_PVD_WIN)
// Winsock is process-wide. A per-socket WSACleanup would tear it down underneath other live
// sockets in the same process (and can cancel another thread's in-flight blocking calls), so
// each linked copy of the runtime starts Winsock exactly once via the guard below. The guard's
// destructor runs when that copy's owning module is unloaded and pairs that copy's single
// WSAStartup with one WSACleanup. Winsock's process-wide reference counting makes multiple runtime
// copies safe, while no per-stream cleanup can cancel another stream's pending calls. The C++11
// thread-safe function-local static makes each copy's one-time startup race-free even if two
// threads create their first socket at once.
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
	mSendTimeoutMs = OMNI_PVD_DEFAULT_SOCKET_TIMEOUT_MS;
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
	// Blocks the calling thread until a client connects or the native accept fails.
	for (;;)
	{
		mDataSocket = ::accept((OmniPvdNativeSocket)mListenSocket, NULL, NULL);
#if !defined(OMNI_PVD_WIN)
		// A signal delivered to the accepting thread interrupts the blocking accept with EINTR;
		// that is not a real failure, so retry (matching the send/recv EINTR loops).
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

	// Retry so either side tolerates starting before the other is listening, but bound the whole
	// connect by a single deadline derived from the send timeout. Each attempt is nonblocking
	// (::connect() then a short select()/poll() slice for writability), so an unreachable or
	// SYN-dropping host can not multiply the OS connect timeout by the retry count; a refused attempt
	// still retries within the remaining budget. The connect should then return near the timeout budget.
	typedef std::chrono::steady_clock ConnectClock;
	const uint32_t connectTimeoutMs = mSendTimeoutMs ? mSendTimeoutMs : OMNI_PVD_DEFAULT_SOCKET_TIMEOUT_MS;
	const ConnectClock::time_point connectDeadline =
		ConnectClock::now() + std::chrono::milliseconds(connectTimeoutMs);
	bool connected = false;
	OmniPvdNativeSocket connectedSocket = OMNI_PVD_INVALID_SOCKET;
	while (!connected && ConnectClock::now() < connectDeadline)
	{
		for (addrinfo* ai = result; ai != NULL; ai = ai->ai_next)
		{
			OmniPvdNativeSocket dataSocket = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
			if (dataSocket == OMNI_PVD_INVALID_SOCKET)
				continue;
			if (!omniPvdSetNonBlocking(dataSocket, true))
			{
				omniPvdCloseNativeSocket(dataSocket);
				continue;
			}

			const int connectResult =
				::connect(dataSocket, ai->ai_addr, (OmniPvdSockLen)ai->ai_addrlen);
			bool connectPending = connectResult != 0 && omniPvdConnectIsPending();
			connected = connectResult == 0;
			while (connectPending && ConnectClock::now() < connectDeadline)
			{
				const double remainingMs = std::chrono::duration<double, std::milli>(
					connectDeadline - ConnectClock::now()).count();
				if (remainingMs <= 0)
					break;
				const int waitMs = remainingMs < static_cast<double>(OMNI_PVD_CONNECT_POLL_INTERVAL_MS)
						? static_cast<int>(remainingMs) : static_cast<int>(OMNI_PVD_CONNECT_POLL_INTERVAL_MS);
				const int selected = omniPvdWaitForConnect(dataSocket, waitMs);
				if (selected > 0)
				{
					// Writable != success: a failed nonblocking connect also selects writable,
					// so read the real result from SO_ERROR.
					int completionCode = 0;
					OmniPvdSockLen completionCodeLength = sizeof(completionCode);
					connected = ::getsockopt(dataSocket, SOL_SOCKET, SO_ERROR,
						reinterpret_cast<char*>(&completionCode), &completionCodeLength) == 0 &&
						completionCode == 0;
					connectPending = false;
				}
				else if (selected < 0 && !omniPvdWaitWasInterrupted())
				{
					connectPending = false; // genuine select()/poll() error: fail this attempt
				}
				// selected == 0 (timeout) or EINTR: re-arm within the remaining budget.
			}

			// Restore blocking before the socket is used for send()/recv().
			if (connected && omniPvdSetNonBlocking(dataSocket, false))
			{
				connectedSocket = dataSocket;
				break;
			}
			connected = false;
			omniPvdCloseNativeSocket(dataSocket);
			if (ConnectClock::now() >= connectDeadline)
				break;
		}
		if (!connected && ConnectClock::now() < connectDeadline)
		{
			const double remainingMs = std::chrono::duration<double, std::milli>(
				connectDeadline - ConnectClock::now()).count();
			if (remainingMs > 0)
				omniPvdSleepMs(remainingMs < static_cast<double>(OMNI_PVD_CONNECT_POLL_INTERVAL_MS)
					? static_cast<unsigned>(remainingMs) : static_cast<unsigned>(OMNI_PVD_CONNECT_POLL_INTERVAL_MS));
		}
	}
	freeaddrinfo(result);
	if (!connected)
	{
		close();
		return false;
	}
	mDataSocket = connectedSocket;
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
