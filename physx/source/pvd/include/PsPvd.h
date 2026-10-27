// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PS_PVD_H
#define PS_PVD_H

#include "pvd/PxPvd.h"
#include "foundation/PxBroadcast.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxPvdTransport;

#if !PX_DOXYGEN
namespace pvdsdk
{
#endif

class PvdDataStream;
class PvdClient;
class PvdOMMetaDataProvider;

// PsPvd is used for advanced user, it support custom pvd client API
class PsPvd : public physx::PxPvd, public PxAllocationListener
{
  public:
	virtual void addClient(PvdClient* client) = 0;
	virtual void removeClient(PvdClient* client) = 0;
	
	virtual bool registerObject(const void* inItem) = 0;
	virtual bool unRegisterObject(const void* inItem) = 0;

	//AllocationListener
	virtual void onAllocation(size_t size, const char* typeName, const char* filename, int line, void* allocatedMemory) = 0;
	virtual void onDeallocation(void* addr) = 0;

	virtual PvdOMMetaDataProvider& getMetaDataProvider() = 0;
	
	virtual uint64_t getNextStreamId() = 0;
	// Call to flush events to PVD
	virtual void flush() = 0;

};

#if !PX_DOXYGEN
} // namespace pvdsdk
} // namespace physx
#endif

#endif

