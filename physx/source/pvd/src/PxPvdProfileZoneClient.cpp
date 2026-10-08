// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxPvdImpl.h"
#include "PxPvdProfileZoneClient.h"
#include "PxPvdProfileZone.h"

namespace physx
{
namespace pvdsdk
{
struct ProfileZoneClient : public profile::PxProfileZoneClient, public PxUserAllocated
{
	profile::PxProfileZone& mZone;
	PvdDataStream& mStream;

	ProfileZoneClient(profile::PxProfileZone& zone, PvdDataStream& stream) : mZone(zone), mStream(stream)
	{
	}

	~ProfileZoneClient()
	{
		mZone.removeClient(*this);
	}

	virtual void createInstance()
	{
		mStream.addProfileZone(&mZone, mZone.getName());
		mStream.createInstance(&mZone);
		mZone.addClient(*this);
		profile::PxProfileNames names(mZone.getProfileNames());
		PVD_FOREACH(idx, names.eventCount)
		{
			handleEventAdded(names.events[idx]);
		}
	}

	virtual void handleEventAdded(const profile::PxProfileEventName& inName) PX_OVERRIDE
	{
		mStream.addProfileZoneEvent(&mZone, inName.name, inName.eventId.eventId, inName.eventId.compileTimeEnabled);
	}

	virtual void handleBufferFlush(const uint8_t* inData, uint32_t inLength) PX_OVERRIDE
	{
		mStream.setPropertyValue(&mZone, "events", inData, inLength);
	}

	virtual void handleClientRemoved() PX_OVERRIDE
	{
		mStream.destroyInstance(&mZone);
	}

  private:
	ProfileZoneClient& operator=(const ProfileZoneClient&);
};
}
}

using namespace physx;
using namespace pvdsdk;

PvdProfileZoneClient::PvdProfileZoneClient(PvdImpl& pvd) : mSDKPvd(pvd), mPvdDataStream(NULL), mIsConnected(false)
{
}

PvdProfileZoneClient::~PvdProfileZoneClient()
{
	mSDKPvd.removeClient(this);
	// all zones should removed
	PX_ASSERT(mProfileZoneClients.size() == 0);
}

PvdDataStream* PvdProfileZoneClient::getDataStream()
{
	return mPvdDataStream;
}

bool PvdProfileZoneClient::isConnected() const
{
	return mIsConnected;
}

void PvdProfileZoneClient::onPvdConnected()
{
	if(mIsConnected)
		return;
	mIsConnected = true;

	mPvdDataStream = PvdDataStream::create(&mSDKPvd);

}

void PvdProfileZoneClient::onPvdDisconnected()
{
	if(!mIsConnected)
		return;

	mIsConnected = false;
	flush();

	mPvdDataStream->release();
	mPvdDataStream = NULL;
}

void PvdProfileZoneClient::flush()
{
	PVD_FOREACH(idx, mProfileZoneClients.size())
	mProfileZoneClients[idx]->mZone.flushProfileEvents();
}

void PvdProfileZoneClient::onZoneAdded(profile::PxProfileZone& zone)
{
	PX_ASSERT(mIsConnected);
	ProfileZoneClient* client = PVD_NEW(ProfileZoneClient)(zone, *mPvdDataStream);
	mMutex.lock();
	client->createInstance();
	mProfileZoneClients.pushBack(client);
	mMutex.unlock();
}

void PvdProfileZoneClient::onZoneRemoved(profile::PxProfileZone& zone)
{
	for(uint32_t i = 0; i < mProfileZoneClients.size(); i++)
	{
		if(&zone == &mProfileZoneClients[i]->mZone)
		{
			mMutex.lock();
			ProfileZoneClient* client = mProfileZoneClients[i];
			mProfileZoneClients.replaceWithLast(i);
			PVD_DELETE(client);
			mMutex.unlock();
			return;
		}
	}
}
