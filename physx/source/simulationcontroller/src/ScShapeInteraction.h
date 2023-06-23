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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef SC_SHAPE_INTERACTION_H
#define SC_SHAPE_INTERACTION_H

#include "ScElementSimInteraction.h"
#include "ScShapeSim.h"
#include "ScActorPair.h"
#include "ScScene.h"
#include "ScBodySim.h"
#include "PxsContactManager.h"
#include "PxsContext.h"
#include "PxsSimpleIslandManager.h"

#define INVALID_REPORT_PAIR_ID	0xffffffff

namespace physx
{
static PX_FORCE_INLINE bool isParticleSystem(const PxActorType::Enum actorType)
{
	return actorType == PxActorType::ePBD_PARTICLESYSTEM || actorType == PxActorType::eFLIP_PARTICLESYSTEM
		|| actorType == PxActorType::eMPM_PARTICLESYSTEM;
}

class PxsContactManagerOutputIterator;
namespace Sc
{
	class ContactReportAllocationManager;
	/*
	Description: A ShapeInteraction represents a pair of objects which _may_ have contacts. Created by the broadphase
	and processed by the NPhaseCore.
	*/
	class ShapeInteraction : public ElementSimInteraction
	{
		friend class NPhaseCore;
		ShapeInteraction& operator=(const ShapeInteraction&);
	public:
		enum SiFlag
		{
			PAIR_FLAGS_MASK					= (PxPairFlag::eNEXT_FREE - 1),	// Bits where the PxPairFlags get stored
			NEXT_FREE						= ((PAIR_FLAGS_MASK << 1) & ~PAIR_FLAGS_MASK),

			HAS_TOUCH						= (NEXT_FREE << 0),		// Tracks the last know touch state
			HAS_NO_TOUCH					= (NEXT_FREE << 1),		// Tracks the last know touch state
			TOUCH_KNOWN						= (HAS_TOUCH | HAS_NO_TOUCH),  // If none of these flags is set, the touch state is not known (for example, this is true for pairs that never ran narrowphase

			CONTACTS_COLLECT_POINTS			= (NEXT_FREE << 2),		// The user wants to get the contact points (includes debug rendering)
			CONTACTS_RESPONSE_DISABLED		= (NEXT_FREE << 3),		// Collision response disabled (either by the user through PxPairFlag::eSOLVE_CONTACT or because the pair has two kinematics)

			CONTACT_FORCE_THRESHOLD_PAIRS	= PxU32(PxPairFlag::eNOTIFY_THRESHOLD_FORCE_FOUND) | PxU32(PxPairFlag::eNOTIFY_THRESHOLD_FORCE_PERSISTS) | PxU32(PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST),
			CONTACT_REPORT_EVENTS			= PxU32(PxPairFlag::eNOTIFY_TOUCH_FOUND) | PxU32(PxPairFlag::eNOTIFY_TOUCH_PERSISTS) | PxU32(PxPairFlag::eNOTIFY_TOUCH_LOST) |
												PxU32(PxPairFlag::eNOTIFY_THRESHOLD_FORCE_FOUND) | PxU32(PxPairFlag::eNOTIFY_THRESHOLD_FORCE_PERSISTS) | PxU32(PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST),
			CONTACT_REPORT_EXTRA_DATA		= PxU32(PxPairFlag::ePRE_SOLVER_VELOCITY) | PxU32(PxPairFlag::ePOST_SOLVER_VELOCITY) | PxU32(PxPairFlag::eCONTACT_EVENT_POSE),

			FORCE_THRESHOLD_EXCEEDED_NOW	= (NEXT_FREE << 4),
			FORCE_THRESHOLD_EXCEEDED_BEFORE	= (NEXT_FREE << 5),
			FORCE_THRESHOLD_EXCEEDED_FLAGS	= FORCE_THRESHOLD_EXCEEDED_NOW | FORCE_THRESHOLD_EXCEEDED_BEFORE,

			IS_IN_PERSISTENT_EVENT_LIST		= (NEXT_FREE << 6), // The pair is in the list of persistent contact events
			WAS_IN_PERSISTENT_EVENT_LIST	= (NEXT_FREE << 7), // The pair is inactive but used to be in the list of persistent contact events
			IN_PERSISTENT_EVENT_LIST		= IS_IN_PERSISTENT_EVENT_LIST | WAS_IN_PERSISTENT_EVENT_LIST,
			IS_IN_FORCE_THRESHOLD_EVENT_LIST= (NEXT_FREE << 8), // The pair is in the list of force threshold contact events
			IS_IN_CONTACT_EVENT_LIST		= IS_IN_PERSISTENT_EVENT_LIST | IS_IN_FORCE_THRESHOLD_EVENT_LIST,

			LL_MANAGER_RECREATE_EVENT		= CONTACT_REPORT_EVENTS | CONTACTS_COLLECT_POINTS |
											  CONTACTS_RESPONSE_DISABLED | PxU32(PxPairFlag::eMODIFY_CONTACTS)
		};
												ShapeInteraction(ShapeSimBase& s1, ShapeSimBase& s2, PxPairFlags pairFlags, PxsContactManager* contactManager);
												~ShapeInteraction();

		// Submits to contact stream
						void					processUserNotification(PxU32 contactEvent, PxU16 infoFlags, bool touchLost, PxU32 ccdPass, bool useCurrentTransform, 
												PxsContactManagerOutputIterator& outputs);  // ccdPass is 0 for discrete collision and then 1,2,... for the CCD passes
						void					processUserNotificationSync();
						void					processUserNotificationAsync(PxU32 contactEvent, PxU16 infoFlags, bool touchLost, PxU32 ccdPass, bool useCurrentTransform,
																			PxsContactManagerOutputIterator& outputs, ContactReportAllocationManager* alloc = NULL);  // ccdPass is 0 for discrete collision and then 1,2,... for the CCD passes

						void					visualize(	PxRenderOutput&, PxsContactManagerOutputIterator&,
															float scale, float param_contactForce, float param_contactNormal, float param_contactError, float param_contactPoint
															);

						PxU32					getContactPointData(const void*& contactPatches, const void*& contactPoints, PxU32& contactDataSize, PxU32& contactPointCount, PxU32& patchCount, const PxReal*& impulses, PxU32 startOffset, PxsContactManagerOutputIterator& outputs);

						bool					managerLostTouch(PxU32 ccdPass, bool adjustCounters, PxsContactManagerOutputIterator& outputs);
						void					managerNewTouch(PxU32 ccdPass, bool adjustCounters, PxsContactManagerOutputIterator& outputs);

		PX_FORCE_INLINE	void					adjustCountersOnLostTouch();
		PX_FORCE_INLINE	void					adjustCountersOnNewTouch();

		PX_FORCE_INLINE	void					sendCCDRetouch(PxU32 ccdPass, PxsContactManagerOutputIterator& outputs);
						void					setContactReportPostSolverVelocity(ContactStreamManager& cs);
		PX_FORCE_INLINE	void					sendLostTouchReport(bool shapeVolumeRemoved, PxU32 ccdPass, PxsContactManagerOutputIterator& ouptuts);
						void					resetManagerCachedState()	const;
	
		PX_FORCE_INLINE	ActorPair*				getActorPair()				const	{ return mActorPair;							}
		PX_FORCE_INLINE	void					setActorPair(ActorPair& aPair)		{ mActorPair = &aPair;							}
		PX_FORCE_INLINE	void					clearActorPair()					{ mActorPair = NULL;							}
		PX_FORCE_INLINE	ActorPairReport&		getActorPairReport()		const	{ return ActorPairReport::cast(*mActorPair);	}
		PX_INLINE		PxIntBool				isReportPair()				const	{ /*PX_ASSERT(!(PxIntBool(getPairFlags() & CONTACT_REPORT_EVENTS)) || mActorPair->isReportPair());*/ return PxIntBool(getPairFlags() & CONTACT_REPORT_EVENTS);	}
		PX_INLINE		PxIntBool				hasTouch()					const	{ return readFlag(HAS_TOUCH);					}
		PX_INLINE		PxIntBool				hasCCDTouch()				const	{ PX_ASSERT(mManager); return mManager->getHadCCDContact(); }
		PX_INLINE		void					swapAndClearForceThresholdExceeded();

		PX_FORCE_INLINE void					raiseFlag(SiFlag flag)				{ mFlags |= flag;					}
		PX_FORCE_INLINE	PxIntBool				readFlag(SiFlag flag)		const	{ return PxIntBool(mFlags & flag);	}
		PX_FORCE_INLINE	PxU32					getPairFlags()				const;

		PX_FORCE_INLINE	void					removeFromReportPairList();

						void					onShapeChangeWhileSleeping(bool shapeOfDynamicChanged);

		PX_FORCE_INLINE	PxIntBool				hasKnownTouchState() const;

						bool					onActivate(void* data);
						bool					onDeactivate();

						void					updateState(const PxU8 externalDirtyFlags);

					const PxsContactManager*	getContactManager() const { return mManager; }

						void					clearIslandGenData();

		PX_FORCE_INLINE PxU32					getEdgeIndex() const { return mEdgeIndex;  }

		PX_FORCE_INLINE	Sc::ShapeSimBase&		getShape0()	const { return static_cast<ShapeSimBase&>(getElement0()); }
		PX_FORCE_INLINE	Sc::ShapeSimBase&		getShape1()	const { return static_cast<ShapeSimBase&>(getElement1()); }

	private:
						ActorPair*				mActorPair;
						PxsContactManager*		mManager;
						PxU32					mContactReportStamp;
						PxU32					mReportPairIndex;	// Owned by NPhaseCore for its report pair list
						PxU32					mEdgeIndex;
						PxU16					mReportStreamIndex;  // position of this pair in the contact report stream

						void					createManager(void* contactManager);
		PX_INLINE		bool					updateManager(void* contactManager);
		PX_INLINE		void					destroyManager();
		PX_FORCE_INLINE	bool					activeManagerAllowed() const;
		PX_FORCE_INLINE	PxU32					getManagerContactState()		const	{ return mFlags & LL_MANAGER_RECREATE_EVENT; }

		PX_FORCE_INLINE void					clearFlag(SiFlag flag)					{ mFlags &= ~flag;				}
		PX_INLINE		void					setFlag(SiFlag flag, bool value)
												{
													if (value)
														raiseFlag(flag);
													else
														clearFlag(flag);
												}
		PX_FORCE_INLINE void					setHasTouch() { clearFlag(HAS_NO_TOUCH); raiseFlag(HAS_TOUCH); }
		PX_FORCE_INLINE void					setHasNoTouch() { clearFlag(HAS_TOUCH); raiseFlag(HAS_NO_TOUCH); }

		PX_FORCE_INLINE	void					setPairFlags(PxPairFlags flags);

		PX_FORCE_INLINE	void					processReportPairOnActivate();
		PX_FORCE_INLINE	void					processReportPairOnDeactivate();

		// Certain SiFlag cache properties of the pair. If these properties change then the flags have to be updated.
		// For example: is collision enabled for this pair? are contact points requested for this pair?
		PX_FORCE_INLINE	void					updateFlags(const Sc::Scene&, const Sc::ActorSim&, const Sc::ActorSim&, const PxU32 pairFlags);

		friend class Sc::Scene;
	};

} // namespace Sc

// PT: TODO: is there a reason for force-inlining all that stuff?

PX_FORCE_INLINE void Sc::ShapeInteraction::sendLostTouchReport(bool shapeVolumeRemoved, PxU32 ccdPass, PxsContactManagerOutputIterator& outputs)
{
	PX_ASSERT(hasTouch());
	PX_ASSERT(isReportPair());

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

PX_FORCE_INLINE void Sc::ShapeInteraction::setPairFlags(PxPairFlags flags)
{
	PX_ASSERT(PxU32(flags) < PxPairFlag::eNEXT_FREE);  // to find out if a new PxPairFlag has been added after eLAST instead of in front

	PxU32 newFlags = mFlags;
	PxU32 fl = PxU32(flags) & PAIR_FLAGS_MASK;
	newFlags &= (~PAIR_FLAGS_MASK);  // clear old flags
	newFlags |= fl;

	mFlags = newFlags;
}

// PT: returning PxU32 instead of PxPairFlags to remove LHS. Please do not undo this.
PX_FORCE_INLINE PxU32 Sc::ShapeInteraction::getPairFlags() const
{
	return (mFlags & PAIR_FLAGS_MASK);
}

PX_INLINE void Sc::ShapeInteraction::swapAndClearForceThresholdExceeded()
{
	PxU32 flags = mFlags;

	PX_COMPILE_TIME_ASSERT(FORCE_THRESHOLD_EXCEEDED_NOW == (FORCE_THRESHOLD_EXCEEDED_BEFORE >> 1));

	PxU32 nowToBefore = (flags & FORCE_THRESHOLD_EXCEEDED_NOW) << 1;
	flags &= ~(FORCE_THRESHOLD_EXCEEDED_NOW | FORCE_THRESHOLD_EXCEEDED_BEFORE);
	flags |= nowToBefore;

	mFlags = flags;
}

PX_FORCE_INLINE	void Sc::ShapeInteraction::removeFromReportPairList()
{
	// this method should only get called if the pair is in the list for
	// persistent or force based contact reports
	PX_ASSERT(mReportPairIndex != INVALID_REPORT_PAIR_ID);
	PX_ASSERT(readFlag(IS_IN_CONTACT_EVENT_LIST));

	Scene& scene = getScene();

	if (readFlag(IS_IN_FORCE_THRESHOLD_EVENT_LIST))
		scene.getNPhaseCore()->removeFromForceThresholdContactEventPairs(this);
	else 
	{
		PX_ASSERT(readFlag(IS_IN_PERSISTENT_EVENT_LIST));
		scene.getNPhaseCore()->removeFromPersistentContactEventPairs(this);
	}
}

PX_INLINE bool Sc::ShapeInteraction::updateManager(void* contactManager)
{
	if (activeManagerAllowed())
	{
		if (mManager == 0)
			createManager(contactManager);

		return (mManager != NULL);  // creation might fail (pool reached limit, mem allocation failed etc.)
	}
	else
		return false;
}

PX_INLINE void Sc::ShapeInteraction::destroyManager()
{
	PX_ASSERT(mManager);

	Scene& scene = getScene();
	
	PxvNphaseImplementationContext* nphaseImplementationContext = scene.getLowLevelContext()->getNphaseImplementationContext();
	PX_ASSERT(nphaseImplementationContext);
	nphaseImplementationContext->unregisterContactManager(mManager);

	/*if (mEdgeIndex != IG_INVALID_EDGE)
		scene.getSimpleIslandManager()->clearEdgeRigidCM(mEdgeIndex);*/
	scene.getLowLevelContext()->destroyContactManager(mManager);
	mManager = 0;
}

PX_FORCE_INLINE bool Sc::ShapeInteraction::activeManagerAllowed() const
{
	ShapeSimBase& shape0 = getShape0();
	ShapeSimBase& shape1 = getShape1();

	ActorSim& bodySim0 = shape0.getActor();
	ActorSim& bodySim1 = shape1.getActor();

	// the first shape always belongs to a dynamic body or soft body
#if PX_SUPPORT_GPU_PHYSX
	PX_ASSERT(bodySim0.isDynamicRigid() || bodySim0.isSoftBody() || bodySim0.isFEMCloth() || bodySim0.isParticleSystem() || bodySim0.isHairSystem());
#else
	PX_ASSERT(bodySim0.isDynamicRigid());
#endif
	
	const IG::IslandSim& islandSim = getScene().getSimpleIslandManager()->getSpeculativeIslandSim();

	//check whether active in the speculative sim!

	return (islandSim.getNode(bodySim0.getNodeIndex()).isActive() ||
		(!bodySim1.isStaticRigid() && islandSim.getNode(bodySim1.getNodeIndex()).isActive()));
}

PX_FORCE_INLINE void Sc::ShapeInteraction::sendCCDRetouch(PxU32 ccdPass, PxsContactManagerOutputIterator& outputs)
{
	const PxU32 pairFlags = getPairFlags();
	if (pairFlags & PxPairFlag::eNOTIFY_TOUCH_CCD)
		processUserNotification(PxPairFlag::eNOTIFY_TOUCH_CCD, 0, false, ccdPass, false, outputs);
}

PX_FORCE_INLINE void Sc::ShapeInteraction::adjustCountersOnLostTouch()
{
	PX_ASSERT(mActorPair->getTouchCount());

	mActorPair->decTouchCount();
}

PX_FORCE_INLINE void Sc::ShapeInteraction::adjustCountersOnNewTouch()
{
	mActorPair->incTouchCount();
}

PX_FORCE_INLINE PxIntBool Sc::ShapeInteraction::hasKnownTouchState() const
{
	// For a pair where the bodies were added asleep, the touch state is not known until narrowphase runs on the pair for the first time.
	// If such a pair looses AABB overlap before, the conservative approach is to wake the bodies up. This method provides an indicator that
	// this is such a pair. Note: this might also wake up objects that do not touch but that's the price to pay (unless we want to run
	// overlap tests on such pairs).
	if (mManager)
		return mManager->touchStatusKnown();
	else
		return readFlag(TOUCH_KNOWN);
}

}

#endif
