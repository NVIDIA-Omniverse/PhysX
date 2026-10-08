// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxPvdDefaultFileTransport.h"

namespace physx
{
namespace pvdsdk
{

PvdDefaultFileTransport::PvdDefaultFileTransport(const char* name) : mConnected(false), mWrittenData(0), mLocked(false)
{
	mFileBuffer = PX_NEW(PsFileBuffer)(name, PxFileBuf::OPEN_WRITE_ONLY);
}

PvdDefaultFileTransport::~PvdDefaultFileTransport()
{
}

bool PvdDefaultFileTransport::connect()
{
	PX_ASSERT(mFileBuffer);
	mConnected = mFileBuffer->isOpen();
	return mConnected;
}

void PvdDefaultFileTransport::disconnect()
{
	mConnected = false;
}

bool PvdDefaultFileTransport::isConnected()
{
	return mConnected;
}

bool PvdDefaultFileTransport::write(const uint8_t* inBytes, uint32_t inLength)
{
	PX_ASSERT(mLocked);
	PX_ASSERT(mFileBuffer);
	if (mConnected)
	{
		uint32_t len = mFileBuffer->write(inBytes, inLength);
		mWrittenData += len;
		return len == inLength;
	}
	else
		return false;
}

PxPvdTransport& PvdDefaultFileTransport::lock()
{
	mMutex.lock();
	PX_ASSERT(!mLocked);
	mLocked = true;
	return *this;
}

void PvdDefaultFileTransport::unlock()
{
	PX_ASSERT(mLocked);
	mLocked = false;
	mMutex.unlock();
}

void PvdDefaultFileTransport::flush()
{
}

uint64_t PvdDefaultFileTransport::getWrittenDataSize()
{
	return mWrittenData;
}

void PvdDefaultFileTransport::release()
{
	if (mFileBuffer)
	{
		mFileBuffer->close();
		delete mFileBuffer;
	}
	mFileBuffer = NULL;
	PX_DELETE_THIS;
}

class NullFileTransport : public physx::PxPvdTransport, public physx::PxUserAllocated
{
	PX_NOCOPY(NullFileTransport)
  public:
	NullFileTransport();
	virtual ~NullFileTransport();

	virtual bool connect() PX_OVERRIDE;
	virtual void disconnect() PX_OVERRIDE;
	virtual bool isConnected() PX_OVERRIDE;

	virtual bool write(const uint8_t* inBytes, uint32_t inLength) PX_OVERRIDE;

	virtual PxPvdTransport& lock() PX_OVERRIDE;
	virtual void unlock() PX_OVERRIDE;

	virtual void flush() PX_OVERRIDE;

	virtual uint64_t getWrittenDataSize() PX_OVERRIDE;

	virtual void release() PX_OVERRIDE;

  private:
	bool mConnected;
	uint64_t mWrittenData;
	physx::PxMutex mMutex;
	bool mLocked; // for debug, remove it when finished
};

NullFileTransport::NullFileTransport() : mConnected(false), mWrittenData(0), mLocked(false)
{
}

NullFileTransport::~NullFileTransport()
{
}

bool NullFileTransport::connect()
{
	mConnected = true;
	return true;
}

void NullFileTransport::disconnect()
{
	mConnected = false;
}

bool NullFileTransport::isConnected()
{
	return mConnected;
}

bool NullFileTransport::write(const uint8_t* /*inBytes*/, uint32_t inLength)
{
	PX_ASSERT(mLocked);
	if(mConnected)
	{
		uint32_t len = inLength;
		mWrittenData += len;
		return len == inLength;
	}
	else
		return false;
}

PxPvdTransport& NullFileTransport::lock()
{
	mMutex.lock();
	PX_ASSERT(!mLocked);
	mLocked = true;
	return *this;
}

void NullFileTransport::unlock()
{
	PX_ASSERT(mLocked);
	mLocked = false;
	mMutex.unlock();
}

void NullFileTransport::flush()
{
}

uint64_t NullFileTransport::getWrittenDataSize()
{
	return mWrittenData;
}

void NullFileTransport::release()
{
	PX_DELETE_THIS;
}

} // namespace pvdsdk

PxPvdTransport* PxDefaultPvdFileTransportCreate(const char* name)
{
	if(name)
		return PX_NEW(pvdsdk::PvdDefaultFileTransport)(name);
	else
		return PX_NEW(pvdsdk::NullFileTransport)();
}

} // namespace physx

