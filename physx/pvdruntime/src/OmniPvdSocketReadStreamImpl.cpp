// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OmniPvdSocketReadStreamImpl.h"
#include "OmniPvdSocketProtocol.h"

OmniPvdSocketReadStreamImpl::OmniPvdSocketReadStreamImpl(uint16_t port)
{
	mPort = port;
}

OmniPvdSocketReadStreamImpl::~OmniPvdSocketReadStreamImpl()
{
	closeStream();
}

bool OmniPvdSocketReadStreamImpl::recvAll(uint8_t* bytes, uint64_t nbrBytes)
{
	uint64_t got = 0;
	while (got < nbrBytes)
	{
		int64_t r = mSocket.recv(bytes + got, nbrBytes - got);
		if (r <= 0)
			return false;
		got += (uint64_t)r;
	}
	return true;
}

bool OmniPvdSocketReadStreamImpl::recvAndValidateHandshake()
{
	OmniPvdSocketStreamHandshake handshake;
	if (!recvAll((uint8_t*)&handshake, sizeof(handshake)))
		return false;
	// The magic + proto validate only this socket transport's on-wire framing at connect time.
	// OVD format/version compatibility is a separate, general concern carried by the 12-byte
	// OmniPVD version header that every stream type emits and that OmniPvdReader::startReading
	// validates one frame later -- the socket layer is deliberately oblivious to it.
	if (handshake.magic != OMNI_PVD_HANDSHAKE_MAGIC)
		return false;
	if (handshake.proto > OMNI_PVD_LIVE_PROTO)
		return false; // producer speaks a newer socket protocol than we understand
	return true;
}

uint64_t OMNI_PVD_CALL OmniPvdSocketReadStreamImpl::readBytes(uint8_t* bytes, uint64_t nbrBytes)
{
	uint64_t got = 0;
	while (got < nbrBytes)
	{
		int64_t r = mSocket.recv(bytes + got, nbrBytes - got);
		if (r <= 0)
			return got; // 0 = orderly peer close, < 0 = error -> EOF-like short read
		got += (uint64_t)r;
	}
	return got; // == nbrBytes
}

uint64_t OMNI_PVD_CALL OmniPvdSocketReadStreamImpl::skipBytes(uint64_t nbrBytes)
{
	uint8_t discard[4096];
	uint64_t got = 0;
	while (got < nbrBytes)
	{
		uint64_t chunk = nbrBytes - got;
		if (chunk > sizeof(discard))
			chunk = sizeof(discard);
		int64_t r = mSocket.recv(discard, chunk);
		if (r <= 0)
			return got;
		got += (uint64_t)r;
	}
	return got;
}

bool OMNI_PVD_CALL OmniPvdSocketReadStreamImpl::openStream()
{
	if (mSocket.isOpen())
		return true; // already open, so nothing to do

	// Blocking server listen + accept of the connecting producer on the fixed port.
	if (!mSocket.listenAndAccept(mPort))
	{
		mSocket.close();
		return false;
	}

	if (!recvAndValidateHandshake())
	{
		mSocket.close();
		return false;
	}
	return true;
}

bool OMNI_PVD_CALL OmniPvdSocketReadStreamImpl::closeStream()
{
	mSocket.close();
	return true;
}
