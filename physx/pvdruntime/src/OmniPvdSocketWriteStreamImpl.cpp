// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OmniPvdSocketWriteStreamImpl.h"
#include "OmniPvdSocketProtocol.h"

#include <string.h>
#include <atomic>

static std::atomic<uint64_t> gCaptureConnectionId(0);

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
	mControlClosed = false;
	mCaptureSessionId = 0;
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
	handshake.flags = OMNI_PVD_LIVE_FILTER_CONTROL | OMNI_PVD_LIVE_CAPTURE_CATALOG;
	return sendAll((const uint8_t*)&handshake, sizeof(handshake));
}

OmniPvdCaptureRequestResult::Enum OmniPvdSocketWriteStreamImpl::decodeCaptureRequest(OmniPvdCaptureRequest& request)
{
	if (mControlBytes.size() < OMNI_PVD_CAPTURE_HEADER_BYTES) return OmniPvdCaptureRequestResult::eNONE;
	const uint8_t* header = mControlBytes.data();
	const uint64_t magic = omniPvdDecodeControlInteger(header, 4);
	const bool catalogOnly = magic == OMNI_PVD_CAPTURE_CATALOG_MAGIC;
	if (!catalogOnly && magic != OMNI_PVD_CAPTURE_MAGIC)
		return OmniPvdCaptureRequestResult::eERROR;
	const uint32_t bytes = static_cast<uint32_t>(omniPvdDecodeControlInteger(header + 4, 4));
	const uint64_t requestId = omniPvdDecodeControlInteger(header + 8, 8);
	const uint32_t flags = static_cast<uint32_t>(omniPvdDecodeControlInteger(header + 16, 4));
	const uint32_t scenes = static_cast<uint32_t>(omniPvdDecodeControlInteger(header + 20, 4));
	const uint32_t objects = static_cast<uint32_t>(omniPvdDecodeControlInteger(header + 24, 4));
	const uint64_t count = uint64_t(scenes) + objects;
	if (catalogOnly && (!requestId || flags || count || bytes != OMNI_PVD_CAPTURE_HEADER_BYTES))
		return OmniPvdCaptureRequestResult::eERROR;
	if (count > OMNI_PVD_CAPTURE_MAX_SELECTORS || bytes != OMNI_PVD_CAPTURE_HEADER_BYTES + count * 8)
		return OmniPvdCaptureRequestResult::eERROR;
	if (mControlBytes.size() < bytes) return OmniPvdCaptureRequestResult::eNONE;
	request = OmniPvdCaptureRequest();
	request.requestId = requestId;
	request.recordSecondaryData = (flags & 1u) != 0;
	request.catalogOnly = catalogOnly;
	request.nbScenes = scenes; request.nbObjects = objects;
	mControlSelectors.resize(static_cast<size_t>(count));
	for (size_t i = 0; i < mControlSelectors.size(); ++i)
		mControlSelectors[i] = omniPvdDecodeControlInteger(header + OMNI_PVD_CAPTURE_HEADER_BYTES + 8 * i, 8);
	request.scenes = scenes ? mControlSelectors.data() : NULL;
	request.objects = objects ? mControlSelectors.data() + scenes : NULL;
	mControlBytes.erase(mControlBytes.begin(), mControlBytes.begin() + bytes);
	return OmniPvdCaptureRequestResult::eREQUEST;
}

OmniPvdCaptureRequestResult::Enum OMNI_PVD_CALL OmniPvdSocketWriteStreamImpl::pollCaptureRequest(OmniPvdCaptureRequest& request)
{
	if (mDead) return OmniPvdCaptureRequestResult::eERROR;
	if (!mSocket.isOpen() || mControlClosed) return OmniPvdCaptureRequestResult::eNONE;
	OmniPvdCaptureRequestResult::Enum result = decodeCaptureRequest(request);
	if (result == OmniPvdCaptureRequestResult::eNONE)
	{
		// At most 64 KiB of input per call, regardless of sender rate or fragmentation.
		const size_t remaining = OMNI_PVD_CAPTURE_MAX_BYTES - mControlBytes.size();
		const uint32_t capacity = static_cast<uint32_t>(remaining < mControlScratch.size() ? remaining : mControlScratch.size());
		const int64_t received = capacity ? mSocket.recvAvailable(mControlScratch.data(), capacity) : -1;
		if (received == OmniPvdSocket::eRECEIVE_CLOSED)
		{
			mControlClosed = true;
			// An incomplete request cannot be interpreted safely after EOF.
			if (!mControlBytes.empty()) result = OmniPvdCaptureRequestResult::eERROR;
		}
		else if (received < 0) result = OmniPvdCaptureRequestResult::eERROR;
		else if (received > 0)
		{
			mControlBytes.insert(mControlBytes.end(), mControlScratch.data(), mControlScratch.data() + received);
			result = decodeCaptureRequest(request);
		}
	}
	if (result == OmniPvdCaptureRequestResult::eERROR)
	{
		mDead = true;
		mSocket.close();
		mControlBytes.clear();
		mControlSelectors.clear();
	}
	return result;
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
		return !mDead; // already open, so nothing to do

	// Fresh open: clear the dead-peer flag so a closeStream() followed by openStream() on
	// the same stream object reconnects live (the constructor only clears mDead on the very
	// first open; without this reset a reconnect would stay dead and silently drop writes).
	mDead = false;
	mControlClosed = false;
	mControlScratch.resize(65536);

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
	mCaptureSessionId = ++gCaptureConnectionId;
	return true;
}

bool OMNI_PVD_CALL OmniPvdSocketWriteStreamImpl::closeStream()
{
	mSocket.close();
	mControlBytes.clear();
	mControlSelectors.clear();
	mCaptureSessionId = 0;
	mDead = false;
	mControlClosed = false;
	return true;
}
