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

#include "OmniPvdSocketWriteStreamImpl.h"
#include "OmniPvdSocketProtocol.h"

#include <string.h>

OmniPvdSocketWriteStreamImpl::OmniPvdSocketWriteStreamImpl(const char* address, uint16_t port, uint32_t sendTimeout)
{
	mAddress = NULL;
	if (address)
	{
		const size_t maxAddressLen = 256;
		const size_t len = strnlen(address, maxAddressLen);
		mAddress = new char[len + 1];
		memcpy(mAddress, address, len);
		mAddress[len] = '\0';
	}
	mPort = port;
	mSendTimeoutMs = sendTimeout;
	mDead = false;
	mSocket.setSendTimeout(sendTimeout);
}

OmniPvdSocketWriteStreamImpl::~OmniPvdSocketWriteStreamImpl()
{
	closeStream();
	delete[] mAddress;
	mAddress = NULL;
}

bool OmniPvdSocketWriteStreamImpl::sendAll(const uint8_t* bytes, uint64_t nbrBytes)
{
	uint64_t sent = 0;
	while (sent < nbrBytes)
	{
		int64_t r = mSocket.send(bytes + sent, nbrBytes - sent);
		if (r <= 0)
			return false;
		sent += (uint64_t)r;
	}
	return true;
}

bool OmniPvdSocketWriteStreamImpl::sendHandshake()
{
	// The handshake fields are initialized by the OmniPvdSocketStreamHandshake constructor.
	OmniPvdSocketStreamHandshake handshake;
	return sendAll((const uint8_t*)&handshake, sizeof(handshake));
}

uint64_t OMNI_PVD_CALL OmniPvdSocketWriteStreamImpl::writeBytes(const uint8_t* bytes, uint64_t nbrBytes)
{
	// Called while the OmniPvdWriter holds its exclusive lock, so this is serialized.
	if (mDead)
		return 0; // dead peer -> short count -> writer trips eSTREAM_WRITE_FAILURE
	uint64_t sent = 0;
	while (sent < nbrBytes)
	{
		int64_t r = mSocket.send(bytes + sent, nbrBytes - sent);
		if (r <= 0)
		{
			// Short count -> OmniPvdWriterImpl::writeWithStatus trips eSTREAM_WRITE_FAILURE.
			// (send also returns <= 0 on the SO_SNDTIMEO timeout, so a wedged peer cannot
			// block the simulation thread indefinitely.)
			mDead = true;
			return sent;
		}
		sent += (uint64_t)r;
	}
	return sent; // == nbrBytes
}

bool OMNI_PVD_CALL OmniPvdSocketWriteStreamImpl::flush()
{
	// Direct-send transport: bytes are already on the wire by the time writeBytes returns.
	return !mDead;
}

bool OMNI_PVD_CALL OmniPvdSocketWriteStreamImpl::openStream()
{
	if (mSocket.isOpen())
		return true; // already open, so nothing to do

	// Fresh open: clear the dead-peer flag so a closeStream() followed by openStream() on
	// the same stream object reconnects live (the constructor only clears mDead on the very
	// first open; without this reset a reconnect would stay dead and silently drop writes).
	mDead = false;

	// Blocking client connect to the listening reader (with the socket's internal retry).
	if (!mSocket.connect(mAddress, mPort))
		return false;

	// Send the live-stream handshake below the OmniPVD byte layer, before the
	// writer emits the 12-byte version header on its first write.
	if (!sendHandshake())
	{
		mSocket.close();
		return false;
	}
	return true;
}

bool OMNI_PVD_CALL OmniPvdSocketWriteStreamImpl::closeStream()
{
	mSocket.close();
	return true;
}
