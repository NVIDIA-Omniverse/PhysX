// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PVD_DEFAULT_SOCKET_TRANSPORT_H
#define PX_PVD_DEFAULT_SOCKET_TRANSPORT_H

#include "pvd/PxPvdTransport.h"

#include "foundation/PxUserAllocated.h"
#include "foundation/PxSocket.h"
#include "foundation/PxMutex.h"

namespace physx
{
namespace pvdsdk
{
class PvdDefaultSocketTransport : public PxPvdTransport, public PxUserAllocated
{
	PX_NOCOPY(PvdDefaultSocketTransport)
  public:
	PvdDefaultSocketTransport(const char* host, int port, unsigned int timeoutInMilliseconds);
	virtual ~PvdDefaultSocketTransport();

	virtual bool connect() PX_OVERRIDE;
	virtual void disconnect() PX_OVERRIDE;
	virtual bool isConnected() PX_OVERRIDE;

	virtual bool write(const uint8_t* inBytes, uint32_t inLength) PX_OVERRIDE;

	virtual void flush() PX_OVERRIDE;

	virtual PxPvdTransport& lock() PX_OVERRIDE;
	virtual void unlock() PX_OVERRIDE;

	virtual uint64_t getWrittenDataSize() PX_OVERRIDE;

	virtual void release() PX_OVERRIDE;

  private:
	PxSocket mSocket;
	const char* mHost;
	uint16_t mPort;
	unsigned int mTimeout;
	bool mConnected;
	uint64_t mWrittenData;
	PxMutex mMutex;
	bool mlocked;
};

} // pvdsdk
} // physx

#endif

