// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxPvdDefaultSocketTransport.h"

namespace physx
{
namespace pvdsdk
{
PvdDefaultSocketTransport::PvdDefaultSocketTransport(const char* host, int port, unsigned int timeoutInMilliseconds)
: mHost(host), mPort(uint16_t(port)), mTimeout(timeoutInMilliseconds), mConnected(false), mWrittenData(0)
{
}

PvdDefaultSocketTransport::~PvdDefaultSocketTransport()
{
}

bool PvdDefaultSocketTransport::connect()
{
	if(mConnected)
		return true;

	if(mSocket.connect(mHost, mPort, mTimeout))
	{
		mSocket.setBlocking(true);
		mConnected = true;
	}
	return mConnected;
}

void PvdDefaultSocketTransport::disconnect()
{
	mSocket.flush();
	mSocket.disconnect();
	mConnected = false;
}

bool PvdDefaultSocketTransport::isConnected()
{
	return mSocket.isConnected();
}

bool PvdDefaultSocketTransport::write(const uint8_t* inBytes, uint32_t inLength)
{
	if(mConnected)
	{
		if(inLength == 0)
			return true;

		uint32_t amountWritten = 0;
		uint32_t totalWritten = 0;
		do
		{
			// Sockets don't have to write as much as requested, so we need
			// to wrap this call in a do/while loop.
			// If they don't write any bytes then we consider them disconnected.
			amountWritten = mSocket.write(inBytes, inLength);
			inLength -= amountWritten;
			inBytes += amountWritten;
			totalWritten += amountWritten;
		} while(inLength && amountWritten);

		if(amountWritten == 0)
			return false;

		mWrittenData += totalWritten;

		return true;
	}
	else
		return false;
}

PxPvdTransport& PvdDefaultSocketTransport::lock()
{
	mMutex.lock();
	return *this;
}

void PvdDefaultSocketTransport::unlock()
{
	mMutex.unlock();
}

void PvdDefaultSocketTransport::flush()
{
	mSocket.flush();
}

uint64_t PvdDefaultSocketTransport::getWrittenDataSize()
{
	return mWrittenData;
}

void PvdDefaultSocketTransport::release()
{
	PX_DELETE_THIS;
}

} // namespace pvdsdk

PxPvdTransport* PxDefaultPvdSocketTransportCreate(const char* host, int port, unsigned int timeoutInMilliseconds)
{
	return PX_NEW(pvdsdk::PvdDefaultSocketTransport)(host, port, timeoutInMilliseconds);
}

} // namespace physx
