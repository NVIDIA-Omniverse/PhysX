// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  
           
#include "ScShapeInteraction.h"
#if PX_SUPPORT_GPU_PHYSX
#include "ScParticleSystemSim.h"
#endif

using namespace physx;

PX_IMPLEMENT_OUTPUT_ERROR

Sc::ShapeInteraction::ShapeInteraction(ShapeSimBase& s1, ShapeSimBase& s2, PxPairFlags pairFlags, PxsContactManager* contactManager) :
	ElementSimInteraction	(s1, s2, InteractionType::eOVERLAP, InteractionFlag::eRB_ELEMENT|InteractionFlag::eFILTERABLE),
	mActorPair				(NULL),
	mManager				(NULL),
	mContactReportStamp		(PX_INVALID_U32),
	mReportPairIndex		(INVALID_REPORT_PAIR_ID),
	mEdgeIndex				(IG_INVALID_EDGE),
	mReportStreamIndex		(0)
{
	mFlags = 0;

	// The PxPairFlags get stored in the SipFlag, make sure any changes get noticed
	PX_COMPILE_TIME_ASSERT(PxPairFlag::eSOLVE_CONTACT == (1<<0));
	PX_COMPILE_TIME_ASSERT(PxPairFlag::eMODIFY_CONTACTS == (1<<1));
	PX_COMPILE_TIME_ASSERT(PxPairFlag::eNOTIFY_TOUCH_FOUND == (1<<2));
	PX_COMPILE_TIME_ASSERT(PxPairFlag::eNOTIFY_TOUCH_PERSISTS == (1<<3));
	PX_COMPILE_TIME_ASSERT(PxPairFlag::eNOTIFY_TOUCH_LOST == (1<<4));
	PX_COMPILE_TIME_ASSERT(PxPairFlag::eNOTIFY_TOUCH_CCD == (1<<5));
	PX_COMPILE_TIME_ASSERT(PxPairFlag::eNOTIFY_THRESHOLD_FORCE_FOUND == (1<<6));
	PX_COMPILE_TIME_ASSERT(PxPairFlag::eNOTIFY_THRESHOLD_FORCE_PERSISTS == (1<<7));
	PX_COMPILE_TIME_ASSERT(PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST == (1<<8));
	PX_COMPILE_TIME_ASSERT(PxPairFlag::eNOTIFY_CONTACT_POINTS == (1<<9));
	PX_COMPILE_TIME_ASSERT(PxPairFlag::eDETECT_DISCRETE_CONTACT == (1<<10));
	PX_COMPILE_TIME_ASSERT(PxPairFlag::eDETECT_CCD_CONTACT == (1<<11));
	PX_COMPILE_TIME_ASSERT(PxPairFlag::ePRE_SOLVER_VELOCITY == (1<<12));
	PX_COMPILE_TIME_ASSERT(PxPairFlag::ePOST_SOLVER_VELOCITY == (1<<13));
	PX_COMPILE_TIME_ASSERT(PxPairFlag::eCONTACT_EVENT_POSE == (1<<14));
	PX_COMPILE_TIME_ASSERT((PAIR_FLAGS_MASK & PxPairFlag::eSOLVE_CONTACT) == PxPairFlag::eSOLVE_CONTACT);
	PX_COMPILE_TIME_ASSERT((PxPairFlag::eSOLVE_CONTACT | PAIR_FLAGS_MASK) == PAIR_FLAGS_MASK);
	PX_COMPILE_TIME_ASSERT((PAIR_FLAGS_MASK & PxPairFlag::eCONTACT_EVENT_POSE) == PxPairFlag::eCONTACT_EVENT_POSE);
	PX_COMPILE_TIME_ASSERT((PxPairFlag::eCONTACT_EVENT_POSE | PAIR_FLAGS_MASK) == PAIR_FLAGS_MASK);

	setPairFlags(pairFlags);

	//Add a fresh edge to the island manager.
	Scene& scene = getScene();
	//Sc::BodySim* bs0 = getShape0().getBodySim();
	//Sc::BodySim* bs1 = getShape1().getBodySim();

	Sc::ActorSim& bs0 = getShape0().getActor();
	Sc::ActorSim& bs1 = getShape1().getActor();

	updateFlags(scene, bs0, bs1, pairFlags);

	if(contactManager == NULL)
	{
		PxNodeIndex indexA, indexB;
		//if(bs0)  // the first shape always belongs to a dynamic body (we assert for this above)
		{
			indexA = bs0.getNodeIndex();
			bs0.registerCountedInteraction();
		}
		if(!bs1.isStaticRigid())
		{
			indexB = bs1.getNodeIndex();
			bs1.registerCountedInteraction();
		}

		IG::SimpleIslandManager* simpleIslandManager = scene.getSimpleIslandManager();

		const PxActorType::Enum actorTypeLargest = PxMax(bs0.getActorType(), bs1.getActorType());

		IG::Edge::EdgeType type = IG::Edge::eCONTACT_MANAGER;
#if PX_SUPPORT_GPU_PHYSX
		if (actorTypeLargest == PxActorType::eSOFTBODY)
			type = IG::Edge::eSOFT_BODY_CONTACT;
		else if (actorTypeLargest == PxActorType::eFEMCLOTH)
			type = IG::Edge::eFEM_CLOTH_CONTACT;
		else if (actorTypeLargest == PxActorType::ePBD_PARTICLESYSTEM)
			type = IG::Edge::ePARTICLE_SYSTEM_CONTACT;
		else if (actorTypeLargest == PxActorType::eHAIRSYSTEM)
			type = IG::Edge::eHAIR_SYSTEM_CONTACT;
#endif
		mEdgeIndex = simpleIslandManager->addContactManager(NULL, indexA, indexB, this, type);

		{
			const bool active = onActivate(contactManager);
			registerInActors();
			scene.registerInteraction(this, active);
		}

		//If it is a soft body or particle overlap, treat it as a contact for now (we can hook up touch found/lost events later maybe)
		if (actorTypeLargest > PxActorType::eARTICULATION_LINK)
			simpleIslandManager->setEdgeConnected(mEdgeIndex, type);
	}
	else
	{
		onActivate(contactManager);
	}
}

Sc::ShapeInteraction::~ShapeInteraction()
{
	Sc::ActorSim* body0 = &getShape0().getActor();
	Sc::ActorSim* body1 = &getShape1().getActor();

	body0->unregisterCountedInteraction();
	if (body1)
		body1->unregisterCountedInteraction();

	if(mManager)
		destroyManager();

	if(mEdgeIndex != IG_INVALID_EDGE)
	{
		Scene& scene = getScene();

		scene.getSimpleIslandManager()->removeConnection(mEdgeIndex);
		mEdgeIndex = IG_INVALID_EDGE;

		scene.unregisterInteraction(this);
	}

	// This will remove the interaction from the actors list, which will prevent
	// update calls to this actor because of Body::wakeUp below.
	unregisterFromActors();

	if(mReportPairIndex != INVALID_REPORT_PAIR_ID)
		removeFromReportPairList();
}

void Sc::ShapeInteraction::clearIslandGenData()
{
	if(mEdgeIndex != IG_INVALID_EDGE)
	{
		Scene& scene = getScene();
		scene.getSimpleIslandManager()->removeConnection(mEdgeIndex);
		mEdgeIndex = IG_INVALID_EDGE;
	}
}

PX_FORCE_INLINE	void Sc::ShapeInteraction::processReportPairOnActivate()
{
	PX_ASSERT(isReportPair());
	PX_ASSERT(mReportPairIndex == INVALID_REPORT_PAIR_ID);

	if(readFlag(WAS_IN_PERSISTENT_EVENT_LIST))
	{
		getScene().getNPhaseCore()->addToPersistentContactEventPairs(this);
		mFlags &= ~WAS_IN_PERSISTENT_EVENT_LIST;
	}
}

PX_FORCE_INLINE	void Sc::ShapeInteraction::processReportPairOnDeactivate()
{
	PX_ASSERT(isReportPair());
	PX_ASSERT(mReportPairIndex != INVALID_REPORT_PAIR_ID);
	PX_COMPILE_TIME_ASSERT(IS_IN_PERSISTENT_EVENT_LIST == (WAS_IN_PERSISTENT_EVENT_LIST >> 1));
	PX_ASSERT(!(readFlag(WAS_IN_PERSISTENT_EVENT_LIST)));

	const PxU32 wasInPersList = (mFlags & IS_IN_PERSISTENT_EVENT_LIST) << 1;
	mFlags |= wasInPersList;

	removeFromReportPairList();
}

void Sc::ShapeInteraction::setContactReportPostSolverVelocity(ContactStreamManager& cs)
{
	Scene& scene = getScene();
	NPhaseCore* npcore = scene.getNPhaseCore();
	PxU8* stream = npcore->getContactReportPairData(cs.bufferIndex);
	
	ActorPairReport& apr = getActorPairReport();
	cs.setContactReportPostSolverVelocity(stream, apr.getActorA(), apr.getActorB());
}

void Sc::ShapeInteraction::resetManagerCachedState() const
{
	if(mManager)
	{
		Sc::Scene& scene = getScene();
		PxvNphaseImplementationContext* nphaseImplementationContext = scene.getLowLevelContext()->getNphaseImplementationContext();
		PX_ASSERT(nphaseImplementationContext);

		mManager->resetCachedState();	
		nphaseImplementationContext->refreshContactManager(mManager);
	}
}

/*
	This method can be called from various stages in the pipeline, some of which operate before the actor has advanced its pose and some after it has advanced its pose.
	Discrete touch found events operate before the pose has been updated. This is because we are using the found event to active the bodies before solve so that we can just
	solve the activated bodies.
	Lost touch events occur after the pose has been updated.
*/
void Sc::ShapeInteraction::processUserNotificationSync()
{
	PX_ASSERT(hasTouch());

	if(mManager)
		PxPrefetchLine(mManager);

	// make sure shape A and shape B are the same way round as the actors (in compounds they may be swapped)
	// TODO: make "unswapped" a SIP flag and set it in updateState()
	if(!mActorPair)
		return;

	NPhaseCore* npcore = getScene().getNPhaseCore();

	ActorPairReport& aPairReport = getActorPairReport();

	if(!aPairReport.isInContactReportActorPairSet())
	{
		aPairReport.setInContactReportActorPairSet();
		npcore->addToContactReportActorPairSet(&aPairReport);
		aPairReport.incRefCount();
	}

	aPairReport.createContactStreamManager(*npcore);
}

void Sc::ShapeInteraction::processUserNotificationAsync(PxU32 contactEvent, PxU16 infoFlags, bool touchLost, 
	PxU32 ccdPass, bool useCurrentTransform, PxsContactManagerOutputIterator& outputs, ContactReportAllocationManager* alloc)
{
	contactEvent = (!ccdPass) ? contactEvent : (contactEvent | PxPairFlag::eNOTIFY_TOUCH_CCD);

	if(!mActorPair)
		return;

	ActorPairReport& aPairReport = getActorPairReport();
	Scene& scene = getScene();
	NPhaseCore* npcore = scene.getNPhaseCore();
	ContactStreamManager& cs = aPairReport.createContactStreamManager(*npcore);
	// Prepare user notification
	const PxU32 timeStamp = scene.getTimeStamp();
	const PxU32 shapePairTimeStamp = scene.getReportShapePairTimeStamp();

	const PxU32 pairFlags = getPairFlags();
	PX_ASSERT(pairFlags & contactEvent);

	const PxU32 extraDataFlags = pairFlags & CONTACT_REPORT_EXTRA_DATA;

	PxU8* stream = NULL;
	ContactShapePair* pairStream = NULL;

	const bool unswapped = &aPairReport.getActorA() == &getShape0().getActor();
	const Sc::ShapeSimBase& shapeA = unswapped ? getShape0() : getShape1();
	const Sc::ShapeSimBase& shapeB = unswapped ? getShape1() : getShape0();

	if(aPairReport.streamResetStamp(timeStamp))
	{
		PX_ASSERT(mContactReportStamp != shapePairTimeStamp);  // actor pair and shape pair timestamps must both be out of sync in this case

		PxU32 maxCount;
		if(cs.maxPairCount != 0)
			maxCount = cs.maxPairCount;  // use value from previous report
		else
		{
			// TODO: Use some kind of heuristic
			maxCount = 2;
			cs.maxPairCount = maxCount;
		}

		PxU32 maxExtraDataSize;
		if(!extraDataFlags || touchLost)
		{
			maxExtraDataSize = 0;
			cs.setMaxExtraDataSize(maxExtraDataSize);
		}
		else
		{
			PxU32 currentMaxExtraDataSize = cs.getMaxExtraDataSize();
			maxExtraDataSize = ContactStreamManager::computeContactReportExtraDataSize(extraDataFlags, true);
			PX_ASSERT(maxExtraDataSize > 0);
			if(maxExtraDataSize <= currentMaxExtraDataSize)
				maxExtraDataSize = currentMaxExtraDataSize;  // use value from previous report
			else
				cs.setMaxExtraDataSize(maxExtraDataSize);
		}

		stream = npcore->reserveContactReportPairData(maxCount, maxExtraDataSize, cs.bufferIndex, alloc);

		if(!maxExtraDataSize)  // this is the usual case, so set it first for branch prediction
			cs.reset();
		else if(stream)
		{
			cs.reset();
			PX_ASSERT(extraDataFlags);
			PX_ASSERT(!touchLost);

			cs.fillInContactReportExtraData(stream, extraDataFlags, aPairReport.getActorA(), aPairReport.getActorB(), ccdPass, useCurrentTransform, 0, sizeof(ContactStreamHeader));
			if((extraDataFlags & PxPairFlag::ePOST_SOLVER_VELOCITY) && (pairFlags & PxPairFlag::eDETECT_CCD_CONTACT))
				scene.setPostSolverVelocityNeeded();
		}
	}
	else
	{
		const PxU32 currentPairCount = cs.currentPairCount;
		if(currentPairCount != 0)
		{
			PxU8* tmpStreamPtr = npcore->getContactReportPairData(cs.bufferIndex);
			if(!extraDataFlags)
				stream = tmpStreamPtr;  // this is the usual case, so set it first for branch prediction
			else
			{
				if(!touchLost)
				{
					// - the first few shape pair events might not request extra data
					// - the events so far were due to touch lost
					// - multiple reports due to CCD multiple passes
					// Hence, the extra data has to be created/extended now.
					//
					const PxU16 oldExtraDataSize = cs.extraDataSize;
					PxI32 lastContactPass;
					if(oldExtraDataSize)
					{
						ContactStreamHeader* strHeader = reinterpret_cast<ContactStreamHeader*>(tmpStreamPtr);
						lastContactPass = strHeader->contactPass;
					}
					else
						lastContactPass = -1;

					if(PxI32(ccdPass) > lastContactPass)  // do not send extra data mulitple times for the same contact pass
					{
						const PxU16 extraDataSize = PxU16(oldExtraDataSize + ContactStreamManager::computeContactReportExtraDataSize(extraDataFlags, (oldExtraDataSize == 0)));
						PxU8* strPtr;
						if (extraDataSize <= cs.getMaxExtraDataSize())
							strPtr = tmpStreamPtr;
						else
							strPtr = npcore->resizeContactReportPairData(currentPairCount < cs.maxPairCount ? cs.maxPairCount : PxU32(cs.maxPairCount+1), extraDataSize, cs);
							// the check for max pair count is there to avoid another potential allocation further below

						if(strPtr)
						{
							stream = strPtr;
							PxU32 sizeOffset;
							if(oldExtraDataSize)
								sizeOffset = oldExtraDataSize;
							else
								sizeOffset = sizeof(ContactStreamHeader);
							cs.fillInContactReportExtraData(strPtr, extraDataFlags, aPairReport.getActorA(), aPairReport.getActorB(), ccdPass, useCurrentTransform, currentPairCount, sizeOffset);
							if((extraDataFlags & PxPairFlag::ePOST_SOLVER_VELOCITY) && (pairFlags & PxPairFlag::eDETECT_CCD_CONTACT))
								scene.setPostSolverVelocityNeeded();
						}
						else
						{
							stream = tmpStreamPtr;
							cs.raiseFlags(ContactStreamManagerFlag::eINCOMPLETE_STREAM);
						}
					}
					else
						stream = tmpStreamPtr;
				}
				else
					stream = tmpStreamPtr;
			}
		}
	}

	if(stream)
		pairStream = cs.getShapePairs(stream);
	else
	{
		cs.raiseFlags(ContactStreamManagerFlag::eINVALID_STREAM);
		return;
	}

	ContactShapePair* cp;
	if(mContactReportStamp != shapePairTimeStamp)
	{
		// this shape pair is not in the contact notification stream yet

		if(cs.currentPairCount < cs.maxPairCount)
			cp = pairStream + cs.currentPairCount;
		else
		{
			const PxU32 newSize = PxU32(cs.currentPairCount + (cs.currentPairCount >> 1) + 1);
			stream = npcore->resizeContactReportPairData(newSize, cs.getMaxExtraDataSize(), cs);
			if(stream)
			{
				pairStream = cs.getShapePairs(stream);
				cp = pairStream + cs.currentPairCount;
			}
			else
			{
				cs.raiseFlags(ContactStreamManagerFlag::eINCOMPLETE_STREAM);
				return;
			}
		}

		//!!! why is alignment important here?
		PX_ASSERT(0==(reinterpret_cast<uintptr_t>(stream) & 0x0f));  // check 16Byte alignment
		
		mReportStreamIndex = cs.currentPairCount;
		cp->shapes[0] = shapeA.getPxShape();
		cp->shapes[1] = shapeB.getPxShape();
		cp->contactPatches = NULL;
		cp->contactPoints = NULL;
		cp->contactForces = NULL;
		cp->frictionPatches = NULL;
		cp->contactCount = 0;
		cp->patchCount = 0;
		cp->constraintStreamSize = 0;
		cp->requiredBufferSize = 0;
		cp->flags = infoFlags;
		PX_ASSERT(contactEvent <= 0xffff);
		cp->events = PxU16(contactEvent);
		cp->shapeID[0] = shapeA.getElementID();
		cp->shapeID[1] = shapeB.getElementID();

		cs.currentPairCount++;

		mContactReportStamp = shapePairTimeStamp;
	}
	else
	{
		// this shape pair is in the contact notification stream already but there is a second event (can happen with force threshold reports, for example).
		
		PX_ASSERT(mReportStreamIndex < cs.currentPairCount);
		cp = &pairStream[mReportStreamIndex];
		cp->events |= contactEvent;
		if(touchLost && cp->events & PxPairFlag::eNOTIFY_TOUCH_PERSISTS)
			cp->events &= PxU16(~PxPairFlag::eNOTIFY_TOUCH_PERSISTS);
		cp->flags |= infoFlags;
	}

	if((getPairFlags() & PxPairFlag::eNOTIFY_CONTACT_POINTS) && mManager && (!cp->contactPatches) && !(contactEvent & PxU32(PxPairFlag::eNOTIFY_TOUCH_LOST | PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST)))
	{
		const PxcNpWorkUnit& workUnit = mManager->getWorkUnit();
		PxsContactManagerOutput* output = NULL;
		if(workUnit.mNpIndex & PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK)
			output = &getScene().getLowLevelContext()->getNphaseImplementationContext()->getNewContactManagerOutput(workUnit.mNpIndex);
		else
			output = &outputs.getContactManager(workUnit.mNpIndex);

		const PxsCCDContactHeader* ccdContactData = reinterpret_cast<const PxsCCDContactHeader*>(workUnit.mCCDContacts);

		const bool isCCDPass = (ccdPass != 0);
		if((output->nbPatches && !isCCDPass) || (ccdContactData && (!ccdContactData->isFromPreviousPass) && isCCDPass))
		{
			const PxU8* contactPatchData;
			const PxU8* contactPointData;
			PxU32 cDataSize;
			PxU32 alignedContactDataSize;
			const PxReal* impulses;
			const PxU8* frictionPatches = output->frictionPatches;

			PxU32 nbPoints = output->nbContacts;
			PxU32 contactPatchCount = output->nbPatches;

			if(!isCCDPass)
			{
				PX_ASSERT(0==(reinterpret_cast<const uintptr_t>(output->contactPatches) & 0x0f));  // check 16Byte alignment
				contactPatchData =  output->contactPatches;
				contactPointData = output->contactPoints;
				cDataSize = sizeof(PxContactPatch)*output->nbPatches + sizeof(PxContact)*output->nbContacts;
				alignedContactDataSize = (cDataSize + 0xf) & 0xfffffff0;
				impulses = output->contactForces;
			}
			else
			{
				PX_ASSERT(0==(reinterpret_cast<const uintptr_t>(ccdContactData) & 0x0f));  // check 16Byte alignment
				contactPatchData = reinterpret_cast<const PxU8*>(ccdContactData) + sizeof(PxsCCDContactHeader);
				contactPointData = contactPatchData + sizeof(PxContactPatch);
				cDataSize = ccdContactData->contactStreamSize - sizeof(PxsCCDContactHeader);
				PxU32 tmpAlignedSize = (ccdContactData->contactStreamSize + 0xf) & 0xfffffff0;
				alignedContactDataSize = tmpAlignedSize - sizeof(PxsCCDContactHeader);
				impulses = reinterpret_cast<const PxReal*>(contactPatchData + alignedContactDataSize);
				nbPoints = 1;
				contactPatchCount = 1;
				frictionPatches = NULL;
			}

			infoFlags = cp->flags;
			infoFlags |= unswapped ? 0 : PxContactPairFlag::eINTERNAL_CONTACTS_ARE_FLIPPED;

			//PX_ASSERT(0==(reinterpret_cast<const uintptr_t>(impulses) & 0x0f));
			
			const PxU32 impulseSize = impulses ? (nbPoints * sizeof(PxReal)) : 0;
			if(impulseSize)
				infoFlags |= PxContactPairFlag::eINTERNAL_HAS_IMPULSES;
			cp->contactPatches = contactPatchData;
			cp->contactPoints = contactPointData;
			cp->frictionPatches = frictionPatches;
			cp->contactCount = PxTo8(nbPoints);
			cp->patchCount = PxTo8(contactPatchCount);
			cp->constraintStreamSize = PxTo16(cDataSize);
			cp->requiredBufferSize = alignedContactDataSize + impulseSize;
			cp->contactForces = impulses;

			cp->flags = infoFlags;
		}
	}
}

void Sc::ShapeInteraction::processUserNotification(PxU32 contactEvent, PxU16 infoFlags, bool touchLost, PxU32 ccdPass, bool useCurrentTransform, PxsContactManagerOutputIterator& outputs)
{
	processUserNotificationSync();
	processUserNotificationAsync(contactEvent, infoFlags, touchLost, ccdPass, useCurrentTransform, outputs);
}

void Sc::ShapeInteraction::sendLostTouchReport(bool shapeVolumeRemoved, PxU32 ccdPass, PxsContactManagerOutputIterator& outputs)
{
	PX_ASSERT(hasTouch());
	PX_ASSERT(isReportPair());

	// PT: adding this test here because:
	// - we dereference the pointer below
	// - the code in processUserNotification has a similar test and does nothing if mActorPair is null
	// - getActorPairReport below would also dereference a null pointer
	// => overall this function does nothing if that pointer is null.
	// ### DEFENSIVE
	if(!mActorPair)
	{
		outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Encountered lost touch report for invalid actor pair.");
		return;
	}

	const PxU32 pairFlags = getPairFlags();
	const PxU32 notifyTouchLost = pairFlags & PxU32(PxPairFlag::eNOTIFY_TOUCH_LOST);
	const PxIntBool thresholdExceeded = readFlag(ShapeInteraction::FORCE_THRESHOLD_EXCEEDED_NOW);
	const PxU32 notifyThresholdLost = thresholdExceeded ? (pairFlags & PxU32(PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST)) : 0;
	if(!notifyTouchLost && !notifyThresholdLost)
		return;

	PxU16 infoFlag = 0;
	if(mActorPair->getTouchCount() == 1)  // this code assumes that the actor pair touch count does get decremented afterwards
		infoFlag |= PxContactPairFlag::eACTOR_PAIR_LOST_TOUCH;

	//Lost touch is processed after solver, so we should use the previous transform to update the pose for objects if user request eCONTACT_EVENT_POSE
	const bool useCurrentTransform = false;

	const PxU32 triggeredFlags = notifyTouchLost | notifyThresholdLost;
	PX_ASSERT(triggeredFlags); 
	processUserNotification(triggeredFlags, infoFlag, true, ccdPass, useCurrentTransform, outputs);

	if(shapeVolumeRemoved)
	{
		ActorPairReport& apr = getActorPairReport();
		ContactStreamManager& cs = apr.getContactStreamManager();
		cs.raiseFlags(ContactStreamManagerFlag::eTEST_FOR_REMOVED_SHAPES);
	}
}

PxU32 Sc::ShapeInteraction::getContactPointData(const void*& contactPatches, const void*& contactPoints, PxU32& contactDataSize, PxU32& contactPointCount, PxU32& numPatches, const PxReal*& impulses, PxU32 startOffset,
	PxsContactManagerOutputIterator& outputs)
{
	const void* frictionPatches;
	return getContactPointData(contactPatches, contactPoints, contactDataSize,
							   contactPointCount, numPatches, impulses, startOffset,
							   outputs, frictionPatches);
}

PxU32 Sc::ShapeInteraction::getContactPointData(const void*& contactPatches, const void*& contactPoints, PxU32& contactDataSize, PxU32& contactPointCount, PxU32& numPatches, const PxReal*& impulses, PxU32 startOffset,
	PxsContactManagerOutputIterator& outputs, const void*& frictionPatches)
{
	// Process LL generated contacts
	if(mManager != NULL)
	{
		const PxcNpWorkUnit& workUnit = mManager->getWorkUnit();

		PxsContactManagerOutput* output = NULL;
		
		if(workUnit.mNpIndex & PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK)
			output = &getScene().getLowLevelContext()->getNphaseImplementationContext()->getNewContactManagerOutput(workUnit.mNpIndex);
		else
			output = &outputs.getContactManager(workUnit.mNpIndex);

		/*const void* dcdContactPatches;
		const void* dcdContactPoints;
		PxU32 dcdContactPatchCount;
		const PxReal* dcdImpulses;
		const PxsCCDContactHeader* ccdContactStream;
		PxU32 dcdContactCount = mManager->getContactPointData(dcdContactPatches, dcdContactPoints, dcdContactPatchCount, dcdImpulses, ccdContactStream);

		PX_ASSERT(((dcdContactCount == 0) && (!ccdContactStream)) || ((dcdContactCount > 0) && hasTouch()) || (ccdContactStream && hasCCDTouch()));*/

		const PxsCCDContactHeader* ccdContactStream = reinterpret_cast<const PxsCCDContactHeader*>(workUnit.mCCDContacts);

		PxU32 idx = 0;
		if(output) // preventive measure for omnicrash OM-109664 / PX-4510.	### DEFENSIVE
		{
			if(output->nbContacts)
			{
				if(startOffset == 0)
				{
					contactPatches = output->contactPatches;
					contactPoints = output->contactPoints;
					contactDataSize = sizeof(PxContactPatch) * output->nbPatches + sizeof(PxContact) * output->nbContacts;
					contactPointCount = output->nbContacts;
					numPatches = output->nbPatches;
					impulses = output->contactForces;
					frictionPatches = output->frictionPatches;

					if(!ccdContactStream)
						return startOffset;
					else
						return (startOffset + 1);
				}

				idx++;
			}

			while(ccdContactStream)
			{
				if(startOffset == idx)
				{
					const PxU8* stream = reinterpret_cast<const PxU8*>(ccdContactStream);
					PxU16 streamSize = ccdContactStream->contactStreamSize;
					contactPatches = stream + sizeof(PxsCCDContactHeader);
					contactPoints = stream + sizeof(PxsCCDContactHeader) + sizeof(PxContactPatch);
					contactDataSize = streamSize - sizeof(PxsCCDContactHeader);
					contactPointCount = 1;
					numPatches = 1;
					impulses = reinterpret_cast<const PxReal*>(stream + ((streamSize + 0xf) & 0xfffffff0));
					frictionPatches = NULL;

					if(!ccdContactStream->nextStream)
						return startOffset;
					else
						return (startOffset + 1);
				}
				
				idx++;
				ccdContactStream = ccdContactStream->nextStream;
			}
		}
		else
		{
			outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "PxsContactManagerOutput output is null!");
		}
	}

	contactPatches = NULL;
	contactPoints = NULL;
	contactDataSize = 0;
	contactPointCount = 0;
	numPatches = 0;
	impulses = NULL;
	frictionPatches = NULL;
	return startOffset;
}

// Note that LL will not send end touch events for managers that are destroyed while having contact
void Sc::ShapeInteraction::managerNewTouch(PxU32 ccdPass, bool adjustCounters, PxsContactManagerOutputIterator& outputs)
{
	if(readFlag(HAS_TOUCH))
		return; // Do not count the touch twice (for instance when recreating a manager with touch)
	// We have contact this frame
    setHasTouch();

	if(adjustCounters)
		adjustCountersOnNewTouch();
	
	if(!isReportPair())
		return;
	else
	{
		PX_ASSERT(hasTouch());
		PX_ASSERT(!readFlag(IS_IN_PERSISTENT_EVENT_LIST));
		PX_ASSERT(!readFlag(IS_IN_FORCE_THRESHOLD_EVENT_LIST));
		
		const PxU32 pairFlags = getPairFlags();
		if(pairFlags & PxPairFlag::eNOTIFY_TOUCH_FOUND)
		{
			PxU16 infoFlag = 0;
			if(mActorPair->getTouchCount() == 1)  // this code assumes that the actor pair touch count does get incremented beforehand
				infoFlag = PxContactPairFlag::eACTOR_PAIR_HAS_FIRST_TOUCH;

			processUserNotification(PxPairFlag::eNOTIFY_TOUCH_FOUND, infoFlag, false, ccdPass, true, outputs);
		}

		if(pairFlags & PxPairFlag::eNOTIFY_TOUCH_PERSISTS)
		{
			getScene().getNPhaseCore()->addToPersistentContactEventPairsDelayed(this);  // to make sure that from now on, the pairs are tested for persistent contact events
		}
		else if(pairFlags & ShapeInteraction::CONTACT_FORCE_THRESHOLD_PAIRS)
		{
			// new touch -> need to start checking for force threshold events
			// Note: this code assumes that it runs before the pairs get tested for force threshold exceeded
			getScene().getNPhaseCore()->addToForceThresholdContactEventPairs(this);
		}
	}
}

bool Sc::ShapeInteraction::managerLostTouch(PxU32 ccdPass, bool adjustCounters, PxsContactManagerOutputIterator& outputs)
{
	if(!readFlag(HAS_TOUCH))
		return false;

	// We do not have LL contacts this frame and also we lost LL contact this frame

	if(!isReportPair())
	{
		setHasNoTouch();
	}
	else
	{
		PX_ASSERT(hasTouch());
		
		sendLostTouchReport(false, ccdPass, outputs);

		if(readFlag(IS_IN_CONTACT_EVENT_LIST))
		{
			// don't need to worry about persistent/force-threshold contact events until next new touch

			if(readFlag(IS_IN_FORCE_THRESHOLD_EVENT_LIST))
			{
				getScene().getNPhaseCore()->removeFromForceThresholdContactEventPairs(this);
			}
			else
			{
				PX_ASSERT(readFlag(IS_IN_PERSISTENT_EVENT_LIST));
				getScene().getNPhaseCore()->removeFromPersistentContactEventPairs(this);
			}

			clearFlag(FORCE_THRESHOLD_EXCEEDED_FLAGS);
		}

		setHasNoTouch();
	}

	ActorSim& body0 = getShape0().getActor();
	ActorSim& body1 = getShape1().getActor();

	if(adjustCounters)
		adjustCountersOnLostTouch();

	if(body1.isStaticRigid())
	{
		body0.internalWakeUp();
		return false;
	}
	return true;
}

PX_FORCE_INLINE void Sc::ShapeInteraction::updateFlags(const Sc::Scene& scene, const Sc::ActorSim& bs0, const Sc::ActorSim& bs1, const PxU32 pairFlags)
{
	// the first shape always belongs to a dynamic body/ a soft body

	bool enabled = true;
	if (bs0.isDynamicRigid())
	{
		const Sc::BodySim& body0 = static_cast<const Sc::BodySim&>(bs0);
		enabled = !body0.isKinematic();
	}

	if (bs1.isDynamicRigid())
	{
		const Sc::BodySim& body1 = static_cast<const Sc::BodySim&>(bs1);
		enabled |= !body1.isKinematic();
	}

	// Check if collision response is disabled
	enabled = enabled && (pairFlags & PxPairFlag::eSOLVE_CONTACT);
	setFlag(CONTACTS_RESPONSE_DISABLED, !enabled);

	// Check if contact points needed
	setFlag(CONTACTS_COLLECT_POINTS, (	(pairFlags & PxPairFlag::eNOTIFY_CONTACT_POINTS) ||
										(pairFlags & PxPairFlag::eMODIFY_CONTACTS) || 
										scene.getVisualizationParameter(PxVisualizationParameter::eCONTACT_POINT) ||
										scene.getVisualizationParameter(PxVisualizationParameter::eCONTACT_NORMAL) ||
										scene.getVisualizationParameter(PxVisualizationParameter::eCONTACT_ERROR) ||
										scene.getVisualizationParameter(PxVisualizationParameter::eCONTACT_IMPULSE)) ||
										scene.getVisualizationParameter(PxVisualizationParameter::eFRICTION_POINT) ||
										scene.getVisualizationParameter(PxVisualizationParameter::eFRICTION_NORMAL) ||
										scene.getVisualizationParameter(PxVisualizationParameter::eFRICTION_IMPULSE) );
}

PX_INLINE PxReal ScGetRestOffset(const Sc::ShapeSimBase& shapeSim)
{
#if PX_SUPPORT_GPU_PHYSX
	if (shapeSim.getActor().isParticleSystem())
		return static_cast<Sc::ParticleSystemSim&>(shapeSim.getActor()).getCore().getRestOffset();
#endif
	return shapeSim.getRestOffset();
}

void Sc::ShapeInteraction::updateState(const PxU8 externalDirtyFlags)
{
	const PxU32 oldContactState = getManagerContactState();

	const PxU8 dirtyFlags = PxU8(getDirtyFlags() | externalDirtyFlags);
	const PxU32 pairFlags = getPairFlags();
	Scene& scene = getScene();
	IG::SimpleIslandManager* islandManager = scene.getSimpleIslandManager();

	if(dirtyFlags & (InteractionDirtyFlag::eFILTER_STATE | InteractionDirtyFlag::eVISUALIZATION))
	{
		Sc::ActorSim& bs0 = getShape0().getActor();
		Sc::ActorSim& bs1 = getShape1().getActor();

		PxIntBool wasDisabled = readFlag(CONTACTS_RESPONSE_DISABLED);

		updateFlags(scene, bs0, bs1, pairFlags);

		PxIntBool isDisabled = readFlag(CONTACTS_RESPONSE_DISABLED);

		if(!wasDisabled && isDisabled)
		{
			islandManager->setEdgeDisconnected(mEdgeIndex);
		}
		else if(wasDisabled && !isDisabled)
		{
			if(readFlag(ShapeInteraction::HAS_TOUCH))
				islandManager->setEdgeConnected(mEdgeIndex, IG::Edge::eCONTACT_MANAGER);
		}
	}

	const PxU32 newContactState = getManagerContactState();
	const bool recreateManager = (oldContactState != newContactState);

	// No use in updating manager properties if the manager is going to be re-created or does not exist yet
	if((!recreateManager) && (mManager != 0))
	{
		ShapeSimBase& shapeSim0 = getShape0();
		ShapeSimBase& shapeSim1 = getShape1();

		// Update dominance
		if(dirtyFlags & InteractionDirtyFlag::eDOMINANCE)
		{
			Sc::ActorSim& bs0 = shapeSim0.getActor();
			Sc::ActorSim& bs1 = shapeSim1.getActor();

			// Static actors are in dominance group zero and must remain there
			const PxDominanceGroup dom0 = bs0.getActorCore().getDominanceGroup();
			const PxDominanceGroup dom1 = !bs1.isStaticRigid() ? bs1.getActorCore().getDominanceGroup() : PxDominanceGroup(0);

			const PxDominanceGroupPair cdom = getScene().getDominanceGroupPair(dom0, dom1);
			mManager->setDominance0(cdom.dominance0);
			mManager->setDominance1(cdom.dominance1);
		}

		if (dirtyFlags & InteractionDirtyFlag::eBODY_KINEMATIC)
		{
			//Kinematic flags changed - clear flag for kinematic on the pair
			Sc::ActorSim& bs1 = shapeSim1.getActor();
			if (bs1.isDynamicRigid())
			{
				if (static_cast<BodySim&>(bs1).isKinematic())
					mManager->getWorkUnit().mFlags |= PxcNpWorkUnitFlag::eHAS_KINEMATIC_ACTOR;
				else
					mManager->getWorkUnit().mFlags &= (~PxcNpWorkUnitFlag::eHAS_KINEMATIC_ACTOR);
			}
		}

		// Update skin width
		if(dirtyFlags & InteractionDirtyFlag::eREST_OFFSET)
			mManager->setRestDistance(ScGetRestOffset(shapeSim0) + ScGetRestOffset(shapeSim1));

		//we may want to only write these if they have changed, the set code is a bit painful for the integration flags because of bit unpacking + packing.
		mManager->setCCD((getPairFlags() & PxPairFlag::eDETECT_CCD_CONTACT) != 0);

		if(dirtyFlags)
			resetManagerCachedState(); // this flushes changes through to the GPU
	}
	else if (readInteractionFlag(InteractionFlag::eIS_ACTIVE))  // only re-create the manager if the pair is active
	{
		PX_ASSERT(mManager);  // if the pair is active, there has to be a manager

		if (dirtyFlags & InteractionDirtyFlag::eBODY_KINEMATIC)
		{
			//Kinematic->dynamic transition
			const IG::IslandSim& islandSim = getScene().getSimpleIslandManager()->getSpeculativeIslandSim();

			//
			//check whether active in the speculative sim!
			const ActorSim& bodySim0 = getShape0().getActor();
			const ActorSim& bodySim1 = getShape1().getActor();

			if (!islandSim.getNode(bodySim0.getNodeIndex()).isActiveOrActivating() &&
				(bodySim1.isStaticRigid() || !islandSim.getNode(bodySim1.getNodeIndex()).isActiveOrActivating()))
			{
				onDeactivate();
				scene.notifyInteractionDeactivated(this);
			}
			else
			{
				//Else we are allowed to be active, so recreate
				if (mEdgeIndex != IG_INVALID_EDGE)
					islandManager->clearEdgeRigidCM(mEdgeIndex);
				destroyManager();
				createManager(NULL);
			}
		}
		else
		{
			PX_ASSERT(activeManagerAllowed());

			// A) This is a newly created pair
			//
			// B) The contact notification or processing state has changed.
			//    All existing managers need to be deleted and recreated with the correct flag set
			//    These flags can only be set at creation in LL
			//KS - added this code here because it is no longer done in destroyManager() - a side-effect of the parallelization of the interaction management code
			if (mEdgeIndex != IG_INVALID_EDGE)
				islandManager->clearEdgeRigidCM(mEdgeIndex);
			destroyManager();
			createManager(NULL);
		}
	}
}

bool Sc::ShapeInteraction::onActivate(void* contactManager)
{
	if(isReportPair())
	{
		// for pairs that go through a second island pass, there is the possibility that they get put to sleep again after the second pass.
		// So we do not want to check for re-insertion into the persistent report pair list yet.
		processReportPairOnActivate();
	}

	if(updateManager(contactManager))
	{
		raiseInteractionFlag(InteractionFlag::eIS_ACTIVE);
		return true;
	}
	else
		return false;
}

bool Sc::ShapeInteraction::onDeactivate()
{
	PX_ASSERT(!getShape0().getActor().isStaticRigid() || !getShape1().getActor().isStaticRigid());
	
	const ActorSim& bodySim0 = getShape0().getActor();
	const ActorSim& bodySim1 = getShape1().getActor();

	PX_ASSERT(	(bodySim0.isStaticRigid() && !bodySim1.isStaticRigid() && !bodySim1.isActive()) || 
				(bodySim1.isStaticRigid() && !bodySim0.isStaticRigid() && !bodySim0.isActive()) ||
				((!bodySim0.isStaticRigid() && !bodySim1.isStaticRigid() && (!bodySim0.isActive() || !bodySim1.isActive()))) );

	if((!bodySim0.isActive()) && (bodySim1.isStaticRigid() || !bodySim1.isActive()))
	{
		if(mReportPairIndex != INVALID_REPORT_PAIR_ID)
			processReportPairOnDeactivate();

		PX_ASSERT((mManager->getTouchStatus() > 0) == (hasTouch() > 0));

		Scene& scene = getScene();
		IG::SimpleIslandManager* islandManager = scene.getSimpleIslandManager();
		if(mManager)
		{
			if((!readFlag(TOUCH_KNOWN)) && mManager->touchStatusKnown() && (!mManager->getTouchStatus()))
			{
				// for pairs that are inserted asleep, we do not know the touch state. If they run through narrowphase and a touch is found,
				// then a managerNewTouch() call will inform this object about the found touch. However, if narrowphase detects that there 
				// is no touch, this object will not be informed about it. The low level manager will always know though. Now, before destroying
				// the pair manager, we need to record "does not have touch" state if available.
				raiseFlag(HAS_NO_TOUCH);
			}

			destroyManager();	
			if(mEdgeIndex != IG_INVALID_EDGE)
				islandManager->clearEdgeRigidCM(mEdgeIndex);
		}
		islandManager->deactivateEdge(mEdgeIndex);

		//
		// We distinguish two scenarios here:
		//
		// A) island generation deactivates objects:
		//    -> the deactivated body was active
		//    -> narrowphase ran on this pair
		//    -> the touch status is known
		//      -> touch:    the objects of the pair are in the same island
		//      -> no touch: the objects of the pair are in different islands
		//
		//    As a consequence, the edge state is not changed. The assumption is that anything that could break the touch status
		//    from here on will have to mark the edges connected (for example if the object gets moved).
		//
		// B) user deactivates objects:
		//    -> the touch status might not be known (for example, the pose gets integrated after the solver which might cause a change
		//       in touch status. If the object gets put to sleep after that, we have to be conservative and mark the edge connected.
		//       other example: an active object gets moved by the user and then deactivated).
		//

		clearInteractionFlag(InteractionFlag::eIS_ACTIVE);
		return true;
	}
	else
	{
		return false;
	}
}

void Sc::ShapeInteraction::createManager(void* contactManager)
{
	//PX_PROFILE_ZONE("ShapeInteraction.createManager", 0);

	Sc::Scene& scene = getScene();

	const PxU32 pairFlags = getPairFlags();

	const int disableCCDContact = !(pairFlags & PxPairFlag::eDETECT_CCD_CONTACT);

	PxsContactManager* manager = scene.getLowLevelContext()->createContactManager(reinterpret_cast<PxsContactManager*>(contactManager), !disableCCDContact);
	PxcNpWorkUnit& mNpUnit = manager->getWorkUnit();

	// Check if contact generation callback has been ordered on the pair
	int contactChangeable = 0;
	if(pairFlags & PxPairFlag::eMODIFY_CONTACTS)
		contactChangeable = 1;

	ShapeSimBase& shapeSim0 = getShape0();
	ShapeSimBase& shapeSim1 = getShape1();

	const PxActorType::Enum type0 = shapeSim0.getActor().getActorType();
	const PxActorType::Enum type1 = shapeSim1.getActor().getActorType();

	const int disableResponse = readFlag(CONTACTS_RESPONSE_DISABLED) ? 1 : 0;
	const int disableDiscreteContact = !(pairFlags & PxPairFlag::eDETECT_DISCRETE_CONTACT);
	const int reportContactInfo = readFlag(CONTACTS_COLLECT_POINTS);
	const int hasForceThreshold = !disableResponse && (pairFlags & CONTACT_FORCE_THRESHOLD_PAIRS);
	int touching;
	if(readFlag(TOUCH_KNOWN))
		touching = readFlag(HAS_TOUCH) ? 1 : -1;
	else
		touching = 0;

	// Static actors are in dominance group zero and must remain there

	Sc::ActorSim& bs0 = shapeSim0.getActor();
	Sc::ActorSim& bs1 = shapeSim1.getActor();

	const PxDominanceGroup dom0 = bs0.getActorCore().getDominanceGroup();
	const PxDominanceGroup dom1 = bs1.isStaticRigid() ? PxDominanceGroup(0) : bs1.getActorCore().getDominanceGroup();

	const bool kinematicActor = bs1.isDynamicRigid() ? static_cast<BodySim&>(bs1).isKinematic() : false;

	const PxDominanceGroupPair cdom = scene.getDominanceGroupPair(dom0, dom1);

	/*const PxI32 hasArticulations= (type0 == PxActorType::eARTICULATION_LINK) | (type1 == PxActorType::eARTICULATION_LINK)<<1;
	const PxI32 hasDynamics		= (type0 != PxActorType::eRIGID_STATIC)      | (type1 != PxActorType::eRIGID_STATIC)<<1;*/ 

	const PxsShapeCore* shapeCore0 = &shapeSim0.getCore().getCore();
	const PxsShapeCore* shapeCore1 = &shapeSim1.getCore().getCore();

	//Initialize the manager....

	manager->mRigidBody0		= bs0.isDynamicRigid() ? &static_cast<BodySim&>(bs0).getLowLevelBody() : NULL;
	manager->mRigidBody1		= bs1.isDynamicRigid() ? &static_cast<BodySim&>(bs1).getLowLevelBody() : NULL;
	manager->mShapeInteraction	= this;
	mNpUnit.mShapeCore0			= shapeCore0;
	mNpUnit.mShapeCore1			= shapeCore1;

	PX_ASSERT(shapeCore0->getTransform().isValid() && shapeCore1->getTransform().isValid());

	mNpUnit.mRigidCore0			= !bs0.isNonRigid() ? &static_cast<ShapeSim&>(shapeSim0).getPxsRigidCore() : NULL;
	mNpUnit.mRigidCore1			= !bs1.isNonRigid() ? &static_cast<ShapeSim&>(shapeSim1).getPxsRigidCore() : NULL;

	mNpUnit.mRestDistance		= ScGetRestOffset(shapeSim0) + ScGetRestOffset(shapeSim1);
	mNpUnit.mDominance0			= cdom.dominance0;
	mNpUnit.mDominance1			= cdom.dominance1;
	mNpUnit.mGeomType0			= PxU8(shapeCore0->mGeometry.getType());
	mNpUnit.mGeomType1			= PxU8(shapeCore1->mGeometry.getType());
	mNpUnit.mTransformCache0	= shapeSim0.getTransformCacheID();
	mNpUnit.mTransformCache1	= shapeSim1.getTransformCacheID();

	mNpUnit.mTorsionalPatchRadius = PxMax(shapeSim0.getTorsionalPatchRadius(),shapeSim1.getTorsionalPatchRadius());
	mNpUnit.mMinTorsionalPatchRadius = PxMax(shapeSim0.getMinTorsionalPatchRadius(), shapeSim1.getMinTorsionalPatchRadius());

	const PxReal slop0 = manager->mRigidBody0 ? manager->mRigidBody0->getCore().offsetSlop : 0.0f;
	const PxReal slop1 = manager->mRigidBody1 ? manager->mRigidBody1->getCore().offsetSlop : 0.0f;
	mNpUnit.mOffsetSlop = PxMax(slop0, slop1);

	PxU16 wuflags = 0;

	if(type0 == PxActorType::eARTICULATION_LINK)
		wuflags |= PxcNpWorkUnitFlag::eARTICULATION_BODY0;

	if(type1 == PxActorType::eARTICULATION_LINK)
		wuflags |= PxcNpWorkUnitFlag::eARTICULATION_BODY1;

	if(type0 == PxActorType::eRIGID_DYNAMIC)
		wuflags |= PxcNpWorkUnitFlag::eDYNAMIC_BODY0;

	if(type1 == PxActorType::eRIGID_DYNAMIC)
		wuflags |= PxcNpWorkUnitFlag::eDYNAMIC_BODY1;

#if PX_SUPPORT_GPU_PHYSX
	if (type0 == PxActorType::eSOFTBODY)
		wuflags |= PxcNpWorkUnitFlag::eSOFT_BODY;

	if (type1 == PxActorType::eSOFTBODY)
		wuflags |= PxcNpWorkUnitFlag::eSOFT_BODY;
#endif
	if(!disableResponse && !contactChangeable)
		wuflags |= PxcNpWorkUnitFlag::eOUTPUT_CONSTRAINTS;

	if(!disableDiscreteContact)
		wuflags |= PxcNpWorkUnitFlag::eDETECT_DISCRETE_CONTACT;
	if(kinematicActor)
		wuflags |= PxcNpWorkUnitFlag::eHAS_KINEMATIC_ACTOR;

	if(disableResponse)
		wuflags |= PxcNpWorkUnitFlag::eDISABLE_RESPONSE;
	if(!disableCCDContact)
		wuflags |= PxcNpWorkUnitFlag::eDETECT_CCD_CONTACTS;

	// this is just the user req: contact reports can also be generated by body thresholding
	if(reportContactInfo || contactChangeable)
		wuflags |= PxcNpWorkUnitFlag::eOUTPUT_CONTACTS;

	if(hasForceThreshold)
		wuflags |= PxcNpWorkUnitFlag::eFORCE_THRESHOLD;

	if(contactChangeable)
		wuflags |= PxcNpWorkUnitFlag::eMODIFIABLE_CONTACT;

	mNpUnit.mFlags = wuflags;

	manager->mFlags = PxU32(contactChangeable ? PxsContactManager::PXS_CM_CHANGEABLE : 0) | PxU32(disableCCDContact ? 0 : PxsContactManager::PXS_CM_CCD_LINEAR);

	//manager->mUserData				= this;
	
	mNpUnit.mNpIndex = 0xFFffFFff;

	mManager = manager;

	PxU8 statusFlags = 0;

	if(touching > 0)
		statusFlags |= PxcNpWorkUnitStatusFlag::eHAS_TOUCH;
	else if (touching < 0)
		statusFlags |= PxcNpWorkUnitStatusFlag::eHAS_NO_TOUCH;

	mNpUnit.mStatusFlags = statusFlags;

	//KS - do not register the CMs here if contactManager isn't null. This implies this is a newly-found pair so we'll do that addition outside in parallel

	if(contactManager == NULL)
	{
		scene.getSimpleIslandManager()->setEdgeRigidCM(mEdgeIndex, mManager);
		PxvNphaseImplementationContext* nphaseImplementationContext = scene.getLowLevelContext()->getNphaseImplementationContext();
		PX_ASSERT(nphaseImplementationContext);
		nphaseImplementationContext->registerContactManager(mManager, this, touching, 0);
	}
}

void Sc::ShapeInteraction::onShapeChangeWhileSleeping(bool shapeOfDynamicChanged)
{
	// if an operation that can break touch status occurs, all deactivated pairs need to set the sleep island edge
	// to connected to make sure that potentially joined islands get detected once parts of the island wake up.
	// Active interactions can be ignored because the edges of those will be marked connected on deactivation.
	if(!mManager)
	{
		Scene& scene = getScene();

		//soft body/dynamic before static
		ActorSim& body0 = getShape0().getActor();
	
		if(shapeOfDynamicChanged && !readFlag(TOUCH_KNOWN))
		{
			// conservative approach: if a pair was added asleep, and a body/shape gets moved, we want to check next frame
			// whether the other body should get woken up. The motivation behind this is to get a similar behavior as in
			// the case where the objects fell asleep rather than have been added asleep (in that case the object will be 
			// woken up with one frame delay).

			ActorSim& body1 = getShape1().getActor();
			
			if(body1.isDynamicRigid() && !readFlag(ShapeInteraction::CONTACTS_RESPONSE_DISABLED))  // the first shape always belongs to a dynamic body, hence no need to test body0
				scene.addToLostTouchList(body0, body1);  // note: this will cause duplicate entries if the pair loses AABB overlap the next frame
		}
	}
}

