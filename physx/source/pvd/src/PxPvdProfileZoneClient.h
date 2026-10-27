// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PVD_PROFILE_ZONE_CLIENT_H
#define PX_PVD_PROFILE_ZONE_CLIENT_H

#include "PxPvdClient.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxMutex.h"
#include "PxProfileZoneManager.h"

namespace physx
{
namespace pvdsdk
{
class PvdImpl;
class PvdDataStream;

struct ProfileZoneClient;

class PvdProfileZoneClient : public PvdClient, public profile::PxProfileZoneHandler, public PxUserAllocated
{
	PX_NOCOPY(PvdProfileZoneClient)
  public:
	PvdProfileZoneClient(PvdImpl& pvd);
	virtual ~PvdProfileZoneClient();

	bool isConnected() const;
	void onPvdConnected();
	void onPvdDisconnected();
	void flush();

	PvdDataStream* getDataStream();

	// PxProfileZoneHandler
	void onZoneAdded(profile::PxProfileZone& inSDK);
	void onZoneRemoved(profile::PxProfileZone& inSDK);

  private:
	PxMutex mMutex; // zoneAdded can called from different threads
	PvdImpl& mSDKPvd;
	PvdDataStream* mPvdDataStream;	
	physx::PxArray<ProfileZoneClient*> mProfileZoneClients;
	bool mIsConnected;
};

} // namespace pvdsdk
} // namespace physx

#endif

