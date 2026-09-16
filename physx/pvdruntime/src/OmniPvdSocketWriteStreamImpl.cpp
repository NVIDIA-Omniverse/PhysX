// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
	return mSocket.isOpen() && !mDead;
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
