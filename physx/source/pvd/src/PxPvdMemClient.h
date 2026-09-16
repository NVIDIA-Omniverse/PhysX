// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PVD_MEM_CLIENT_H
#define PX_PVD_MEM_CLIENT_H

#include "PxPvdClient.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxMutex.h"
#include "foundation/PxBroadcast.h"
#include "PxProfileEventBufferClient.h"
#include "PxProfileMemory.h"

namespace physx
{
class PvdDataStream;

namespace pvdsdk
{
class PvdImpl;
class PvdMemClient : public PvdClient,
                     public profile::PxProfileEventBufferClient,
                     public PxUserAllocated
{
	PX_NOCOPY(PvdMemClient)
  public:
	PvdMemClient(PvdImpl& pvd);
	virtual ~PvdMemClient();

	virtual bool isConnected() const	PX_OVERRIDE;
	virtual void onPvdConnected()	PX_OVERRIDE;
	virtual void onPvdDisconnected()	PX_OVERRIDE;
	virtual void flush()	PX_OVERRIDE;

	virtual	PvdDataStream* getDataStream()	PX_OVERRIDE;
	void sendMemEvents();

	// memory event
	void onAllocation(size_t size, const char* typeName, const char* filename, int line, void* allocatedMemory);
	void onDeallocation(void* addr);

  private:
	PvdImpl& mSDKPvd;
	PvdDataStream* mPvdDataStream;
	bool mIsConnected;

	// mem profile
	PxMutex mMutex; // mem onallocation can called from different threads
	profile::PxProfileMemoryEventBuffer& mMemEventBuffer;
	virtual	void handleBufferFlush(const uint8_t* inData, uint32_t inLength)	PX_OVERRIDE;
	virtual	void handleClientRemoved()	PX_OVERRIDE;
};

} // namespace pvdsdk
} // namespace physx

#endif
