// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxPvdImpl.h"
#include "PxPvdMemClient.h"

namespace physx
{
namespace pvdsdk
{

PvdMemClient::PvdMemClient(PvdImpl& pvd)
: mSDKPvd(pvd)
, mPvdDataStream(NULL)
, mIsConnected(false)
, mMemEventBuffer(profile::PxProfileMemoryEventBuffer::createMemoryEventBuffer(*gPvdAllocatorCallback))
{
}

PvdMemClient::~PvdMemClient()
{
	mSDKPvd.removeClient(this);
	if(mMemEventBuffer.hasClients())
		mPvdDataStream->destroyInstance(&mMemEventBuffer);
	mMemEventBuffer.release();
}

PvdDataStream* PvdMemClient::getDataStream()
{
	return mPvdDataStream;
}

bool PvdMemClient::isConnected() const
{
	return mIsConnected;
}

void PvdMemClient::onPvdConnected()
{
	if(mIsConnected)
		return;
	mIsConnected = true;

	mPvdDataStream = PvdDataStream::create(&mSDKPvd);
	mPvdDataStream->createInstance(&mMemEventBuffer);
	mMemEventBuffer.addClient(*this);
}

void PvdMemClient::onPvdDisconnected()
{
	if(!mIsConnected)
		return;
	mIsConnected = false;

	flush();

	mMemEventBuffer.removeClient(*this);
	mPvdDataStream->release();
	mPvdDataStream = NULL;
}

void PvdMemClient::onAllocation(size_t inSize, const char* inType, const char* inFile, int inLine, void* inAddr)
{
	mMutex.lock();
	mMemEventBuffer.onAllocation(inSize, inType, inFile, inLine, inAddr);
	mMutex.unlock();
}

void PvdMemClient::onDeallocation(void* inAddr)
{
	mMutex.lock();
	mMemEventBuffer.onDeallocation(inAddr);
	mMutex.unlock();
}

void PvdMemClient::flush()
{
	mMutex.lock();
	mMemEventBuffer.flushProfileEvents();
	mMutex.unlock();
}

void PvdMemClient::handleBufferFlush(const uint8_t* inData, uint32_t inLength)
{
	if(mPvdDataStream)
	    mPvdDataStream->setPropertyValue(&mMemEventBuffer, "events", inData, inLength);
}

void PvdMemClient::handleClientRemoved()
{
}

} // pvd
} // physx
