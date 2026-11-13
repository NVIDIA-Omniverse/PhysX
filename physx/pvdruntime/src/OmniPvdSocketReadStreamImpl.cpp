// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OmniPvdSocketReadStreamImpl.h"
#include "OmniPvdSocketProtocol.h"
#include <vector>

OmniPvdSocketReadStreamImpl::OmniPvdSocketReadStreamImpl(uint16_t port)
{
	mPort = port;
	mControlCapabilities = 0;
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
	mControlCapabilities = static_cast<uint16_t>(handshake.flags & (OMNI_PVD_LIVE_FILTER_CONTROL | OMNI_PVD_LIVE_CAPTURE_CATALOG));
	return true;
}

bool OMNI_PVD_CALL OmniPvdSocketReadStreamImpl::requestCapture(const OmniPvdCaptureRequest& request)
{
	const uint64_t count = uint64_t(request.nbScenes) + request.nbObjects;
	if (!request.requestId || request.catalogOnly || count > OMNI_PVD_CAPTURE_MAX_SELECTORS ||
		(request.nbScenes && !request.scenes) || (request.nbObjects && !request.objects))
		return false;
	std::lock_guard<std::mutex> lock(mControlSendMutex);
	if (!(mControlCapabilities & OMNI_PVD_LIVE_FILTER_CONTROL) || !mSocket.isOpen()) return false;
	std::vector<uint8_t> message(OMNI_PVD_CAPTURE_HEADER_BYTES + static_cast<size_t>(count) * 8);
	omniPvdEncodeControlInteger(message.data(), OMNI_PVD_CAPTURE_MAGIC, 4);
	omniPvdEncodeControlInteger(message.data() + 4, message.size(), 4);
	omniPvdEncodeControlInteger(message.data() + 8, request.requestId, 8);
	omniPvdEncodeControlInteger(message.data() + 16, request.recordSecondaryData ? 1u : 0u, 4);
	omniPvdEncodeControlInteger(message.data() + 20, request.nbScenes, 4);
	omniPvdEncodeControlInteger(message.data() + 24, request.nbObjects, 4);
	size_t offset = OMNI_PVD_CAPTURE_HEADER_BYTES;
	const OmniPvdObjectHandle* lists[] = { request.scenes, request.objects };
	const uint32_t counts[] = { request.nbScenes, request.nbObjects };
	for (uint32_t list = 0; list < 2; ++list)
		for (uint32_t i = 0; i < counts[list]; ++i, offset += 8)
			omniPvdEncodeControlInteger(message.data() + offset, lists[list][i], 8);
	return sendControlBytes(message.data(), message.size());
}

bool OMNI_PVD_CALL OmniPvdSocketReadStreamImpl::requestCaptureCatalog(uint64_t requestId)
{
	if (!requestId) return false;
	std::lock_guard<std::mutex> lock(mControlSendMutex);
	if (!(mControlCapabilities & OMNI_PVD_LIVE_CAPTURE_CATALOG) || !mSocket.isOpen()) return false;
	uint8_t message[OMNI_PVD_CAPTURE_HEADER_BYTES] = {};
	omniPvdEncodeControlInteger(message, OMNI_PVD_CAPTURE_CATALOG_MAGIC, 4);
	omniPvdEncodeControlInteger(message + 4, sizeof(message), 4);
	omniPvdEncodeControlInteger(message + 8, requestId, 8);
	return sendControlBytes(message, sizeof(message));
}

bool OmniPvdSocketReadStreamImpl::sendControlBytes(const uint8_t* bytes, uint64_t count)
{
	uint64_t sent = 0;
	while (sent < count)
	{
		const int64_t written = mSocket.send(bytes + sent, count - sent);
		if (written <= 0)
		{
			// A partial frame cannot be retried. Disable control until the caller closes this session.
			mControlCapabilities = 0;
			return false;
		}
		sent += static_cast<uint64_t>(written);
	}
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
	if (!mSocket.beginListen(mPort))
	{
		mSocket.close();
		return false;
	}
	return acceptAndValidate();
}

bool OmniPvdSocketReadStreamImpl::acceptAndValidate()
{
	if (!mSocket.acceptOne() || !recvAndValidateHandshake())
	{
		mSocket.close();
		return false;
	}
	return true;
}

bool OMNI_PVD_CALL OmniPvdSocketReadStreamImpl::closeStream()
{
	mControlCapabilities = 0;
	mSocket.close();
	return true;
}
