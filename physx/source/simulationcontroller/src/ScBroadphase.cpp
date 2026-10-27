// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ScBroadphase.h"
#include "BpAABBManagerBase.h"
#include "ScShapeSim.h"
#include "common/PxProfileZone.h"

using namespace physx;
using namespace Sc;
using namespace Bp;

///////////////////////////////////////////////////////////////////////////////

BroadphaseManager::BroadphaseManager() :
	mBroadPhaseCallback	(NULL),
	mOutOfBoundsIDs		("sceneOutOfBoundsIds")
{
}

BroadphaseManager::~BroadphaseManager()
{
}

void BroadphaseManager::prepareOutOfBoundsCallbacks(AABBManagerBase* aabbManager)
{
	AABBManagerBase::OutOfBoundsData data;
	if(!aabbManager->getOutOfBoundsObjects(data))
		return;

	PxU32 nbOut = data.mNbOutOfBoundsObjects;
	void** outObjects = data.mOutOfBoundsObjects;

	mOutOfBoundsIDs.clear();

	while(nbOut--)
	{
		const ElementSim* volume = reinterpret_cast<const ElementSim*>(*outObjects++);

		const Sc::ShapeSim* sim = static_cast<const Sc::ShapeSim*>(volume);

		mOutOfBoundsIDs.pushBack(sim->getElementID());
	}
}

bool BroadphaseManager::fireOutOfBoundsCallbacks(Bp::AABBManagerBase* aabbManager, const ObjectIDTracker& tracker, PxU64 contextID)
{
	PX_UNUSED(contextID);

	AABBManagerBase::OutOfBoundsData data;
	if(!aabbManager->getOutOfBoundsObjects(data))
		return false;

	bool outputWarning = false;

	PxBroadPhaseCallback* cb = mBroadPhaseCallback;

	// Actors
	{
		PxU32 nbOut = data.mNbOutOfBoundsObjects;
		void** outObjects = data.mOutOfBoundsObjects;

		for(PxU32 i=0;i<nbOut;i++)
		{
			ElementSim* volume = reinterpret_cast<ElementSim*>(outObjects[i]);

			Sc::ShapeSim* sim = static_cast<Sc::ShapeSim*>(volume);

			// PT: TODO: I'm not sure the split between prepareOutOfBoundsCallbacks / fireOutOfBoundsCallbacks
			// and the test for deletion is still needed after the removal of SCB
			if(tracker.isDeletedID(mOutOfBoundsIDs[i]))
				continue;

			if(cb)
			{
				PX_PROFILE_ZONE("USERCODE - PxBroadPhaseCallback::onObjectOutOfBounds", contextID);
				ActorSim& actor = volume->getActor();
				RigidSim& rigidSim = static_cast<RigidSim&>(actor);
				PxActor* pxActor = rigidSim.getPxActor();
				PxShape* px = sim->getPxShape();
				cb->onObjectOutOfBounds(*px, *pxActor);
			}
			else
				outputWarning = true;
		}
	}

	// Aggregates
	{
		PxU32 nbOut = data.mNbOutOfBoundsAggregates;
		void** outAgg = data.mOutOfBoundsAggregates;

		for(PxU32 i=0;i<nbOut;i++)
		{
			PxAggregate* px = reinterpret_cast<PxAggregate*>(outAgg[i]);
			if(cb)
			{
				PX_PROFILE_ZONE("USERCODE - PxBroadPhaseCallback::onObjectOutOfBounds", contextID);
				cb->onObjectOutOfBounds(*px);
			}
			else
				outputWarning = true;
		}
	}

	aabbManager->clearOutOfBoundsObjects();

	return outputWarning;
}

void BroadphaseManager::flush(Bp::AABBManagerBase* /*aabbManager*/)
{
	mOutOfBoundsIDs.reset();
}
