// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

           
#include "ScIterators.h"
#include "ScBodySim.h"
#include "ScShapeSim.h"
#include "ScShapeInteraction.h"
#if PX_SUPPORT_OMNI_PVD
#include "PxsSimulationController.h"
#endif

using namespace physx;

///////////////////////////////////////////////////////////////////////////////

Sc::ContactIterator::Pair::Pair(const void*& contactPatches, const void*& contactPoints, const void*& frictionPatches, PxU32 /*contactDataSize*/, const PxReal*& forces, PxU32 numContacts, PxU32 numPatches,
	ShapeSimBase& shape0, ShapeSimBase& shape1, ActorSim* actor0, ActorSim* actor1)
: mIndex(0)
, mNumContacts(numContacts)
, mIter(reinterpret_cast<const PxU8*>(contactPatches), reinterpret_cast<const PxU8*>(contactPoints), forces ? reinterpret_cast<const PxU32*>(forces + numContacts) : NULL, numPatches, numContacts)
, mAnchorIter(reinterpret_cast<const PxU8*>(contactPatches), reinterpret_cast<const PxU8*>(frictionPatches), numPatches)
, mForces(forces)
, mActor0(actor0->getPxActor())
, mActor1(actor1->getPxActor())
{
	mCurrentContact.shape0 = shape0.getPxShape();
	mCurrentContact.shape1 = shape1.getPxShape();
	mCurrentContact.normalForceAvailable = (forces != NULL);
}

Sc::ContactIterator::Pair* Sc::ContactIterator::getNextPair()
{ 
	PX_ASSERT(mCurrent || (mCurrent == mLast));

	while(mCurrent < mLast)
	{
		// Sleeping pairs stay in the scene's overlap array, see Scene::initContactsIterator().
		if (!(*mCurrent)->readInteractionFlag(InteractionFlag::eIS_ACTIVE))
		{
			++mCurrent;
			mOffset = 0;
			continue;
		}
		ShapeInteraction* si = static_cast<ShapeInteraction*>(*mCurrent);
#if PX_SUPPORT_OMNI_PVD
		if (mOvdController && !mOvdController->ovdIsContactPairCaptured(*si->getActorSim0().getPxActor(), *si->getActorSim1().getPxActor()))
		{
			++mCurrent;
			mOffset = 0;
			continue;
		}
#endif

		const void* contactPatches = NULL;
		const void* contactPoints = NULL;
		PxU32 contactDataSize = 0;
		const PxReal* forces = NULL;
		PxU32 numContacts = 0;
		PxU32 numPatches = 0;
		const void* frictionPatches = NULL;

		PxU32 nextOffset = si->getContactPointData(contactPatches, contactPoints, contactDataSize, numContacts, numPatches, forces, mOffset, *mOutputs, frictionPatches);

		if (nextOffset == mOffset)
		{
			++mCurrent;
			mOffset = 0;
		}
		else
			mOffset = nextOffset;

#if PX_SUPPORT_OMNI_PVD
		// Pair construction reads patch flags, so fetch selected ranges before creating its iterators.
		if (mOvdController && !mOvdController->ovdReadbackContactPair(contactPatches, contactPoints, frictionPatches, forces, numContacts, numPatches))
		{
			mOvdReadbackError = true;
			mCurrent = mLast;
			return NULL;
		}
#endif

		mCurrentPair = Pair(contactPatches, contactPoints, frictionPatches, contactDataSize, forces, numContacts, numPatches, si->getShape0(), si->getShape1(), &si->getActorSim0(), &si->getActorSim1());
		return &mCurrentPair;
	}
	return NULL;
}

#if PX_SUPPORT_OMNI_PVD
void Sc::ContactIterator::enableOmniPvdReadback(PxsSimulationController& controller)
{
	mOvdController = &controller;
	if (!controller.ovdBeginContactReadback()) return;
	// Gather metadata without constructing an iterator, which would read device-only patch flags.
	for (ElementSimInteraction** current = mCurrent; current < mLast; ++current)
	{
		if (!(*current)->readInteractionFlag(InteractionFlag::eIS_ACTIVE)) continue;
		ShapeInteraction* interaction = static_cast<ShapeInteraction*>(*current);
		if (!controller.ovdIsContactPairCaptured(*interaction->getActorSim0().getPxActor(), *interaction->getActorSim1().getPxActor())) continue;
		PxU32 offset = 0;
		do
		{
			const void* patches = NULL; const void* points = NULL; const void* friction = NULL;
			const PxReal* forces = NULL;
			PxU32 bytes = 0, nbContacts = 0, nbPatches = 0;
			const PxU32 next = interaction->getContactPointData(patches, points, bytes, nbContacts, nbPatches, forces, offset, *mOutputs, friction);
			controller.ovdQueueContactReadback(patches, points, friction, forces, nbContacts, nbPatches);
			if (next == offset) break;
			offset = next;
		} while (true);
	}
	if (!controller.ovdFinishContactReadback())
	{
		mOvdReadbackError = true;
		mCurrent = mLast;
	}
}

bool Sc::SleepingTouchIterator::getNextPair(PxActor*& actor0, PxActor*& actor1)
{
	while (mCurrent < mLast)
	{
		const ShapeInteraction* si = static_cast<const ShapeInteraction*>(*mCurrent++);
		if (si->readInteractionFlag(InteractionFlag::eIS_ACTIVE) || !si->hasTouch()) continue;
		actor0 = si->getActorSim0().getPxActor();
		actor1 = si->getActorSim1().getPxActor();
		return true;
	}
	return false;
}
#endif

Sc::Contact* Sc::ContactIterator::Pair::getNextContact()
{
	if(mIndex < mNumContacts)
	{
		while (!mIter.hasNextContact())
		{
			if(!mIter.hasNextPatch())
				return NULL;
			mIter.nextPatch();
		}
		mIter.nextContact();

		mCurrentContact.normal = mIter.getContactNormal();
		mCurrentContact.point = mIter.getContactPoint();
		mCurrentContact.separation = mIter.getSeparation();
		mCurrentContact.normalForce = mForces ? mForces[mIndex] : 0;
		mCurrentContact.faceIndex0 = mIter.getFaceIndex0();
		mCurrentContact.faceIndex1 = mIter.getFaceIndex1();

		mIndex++;
		return &mCurrentContact;
	}
	return NULL;
}

Sc::FrictionAnchor* Sc::ContactIterator::Pair::getNextFrictionAnchor()
{
	if(mAnchorIter.hasNextPatch())
	{
		while (!mAnchorIter.hasNextFrictionAnchor())
		{
			if(!mAnchorIter.hasNextPatch())
				return NULL;
			mAnchorIter.nextPatch();
		}
		mAnchorIter.nextFrictionAnchor();

		mCurrentAnchor.normal = mAnchorIter.getNormal();
		mCurrentAnchor.point = mAnchorIter.getPosition();
		mCurrentAnchor.impulse = mAnchorIter.getImpulse();

		return &mCurrentAnchor;
	}
	return NULL;
}

///////////////////////////////////////////////////////////////////////////////
