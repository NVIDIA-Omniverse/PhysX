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

#ifndef SC_ACTOR_PAIR_H
#define SC_ACTOR_PAIR_H

#include "ScRigidSim.h"
#include "ScContactStream.h"
#include "ScNPhaseCore.h"
#include "foundation/PxErrors.h"
#include "foundation/PxFoundation.h"
#include "foundation/PxPreprocessor.h"
#include "foundation/PxSimpleTypes.h"
#if PX_SUPPORT_GPU_PHYSX
#include "ScDeformableSurfaceSim.h"
#include "ScDeformableVolumeSim.h"
#endif


namespace physx
{
namespace Sc
{
	class ActorPairContactReportData
	{
	public:
		ActorPairContactReportData() : 
			mStrmResetStamp			(0xffffffff),
			mActorAID				(0xffffffff),
			mActorBID				(0xffffffff),
			mPxActorA				(NULL),
			mPxActorB				(NULL)
			{}

		ContactStreamManager	mContactStreamManager;
		PxU32					mStrmResetStamp;
		PxU32					mActorAID;
		PxU32					mActorBID;
		PxActor*				mPxActorA;
		PxActor*				mPxActorB;
	};

	/**
	\brief Class shared by all shape interactions for a pair of actors.

	This base class is used if no shape pair of an actor pair has contact reports requested.
	*/
	class ActorPair
	{
	public:

		enum ActorPairFlags
		{
			eIS_REPORT_PAIR	= (1<<0),
			eIS_IN_CONTACT_REPORT_ACTOR_PAIR_SET = (1<<1), // PT: whether the pair is already stored in the 'ContactReportActorPairSet' or not
			eMAX		= (1<<2)
		};

		PX_FORCE_INLINE					ActorPair() : mRefCount(0), mTouchCount_internalFlags(0) {}
		PX_FORCE_INLINE					~ActorPair() {}

		PX_FORCE_INLINE	PxIntBool		isReportPair() const { return (getFlags() & eIS_REPORT_PAIR); }

		PX_FORCE_INLINE	void			incTouchCount() { setTouchCount(getTouchCount() + 1); PX_ASSERT(getTouchCount()); }
		PX_FORCE_INLINE	void			decTouchCount() { PX_ASSERT(getTouchCount()); setTouchCount(getTouchCount() -1); }
		PX_FORCE_INLINE	PxU32			getTouchCount() const { return mTouchCount_internalFlags >> sTouchCountShift; }

		PX_FORCE_INLINE	void			incRefCount() { ++mRefCount; PX_ASSERT(mRefCount>0); }
		PX_FORCE_INLINE	PxU32			decRefCount() { PX_ASSERT(mRefCount>0); return --mRefCount; }
		PX_FORCE_INLINE	PxU32			getRefCount() const { return mRefCount; }

	private:
		ActorPair& operator=(const ActorPair&);

	protected:
						PxU32			mRefCount;
						PxU32			mTouchCount_internalFlags;

		PX_FORCE_INLINE PxU32			getFlags() const { return mTouchCount_internalFlags & sFlagMask; }
		PX_FORCE_INLINE void 			raiseFlags(PxU32 flags) { PX_ASSERT(flags < ActorPairFlags::eMAX); mTouchCount_internalFlags |= flags; }
		PX_FORCE_INLINE void 			clearFlags(PxU32 flags);
		PX_FORCE_INLINE void			setTouchCount(PxU32 count);

	public:
		static const PxU32	sFlagMask = (ActorPairFlags::eMAX - 1);
		static const PxU32	sTouchCountShift = 2;  // shift necessary to extract the touch count

		PX_COMPILE_TIME_ASSERT(ActorPairFlags::eMAX == (1 << sTouchCountShift)); // if this breaks please adjust the sTouchCountShift above.
	};

	/**
	\brief Class shared by all shape interactions for a pair of actors if contact reports are requested.

	This class is used if at least one shape pair of an actor pair has contact reports requested.

	\note If a pair of actors had contact reports requested for some of the shape interactions but all of them switch to not wanting contact reports
	any longer, then the ActorPairReport instance is kept being used and won't get replaced by a simpler ActorPair instance.
	*/
	class ActorPairReport : public ActorPair
	{
		PX_NOCOPY(ActorPairReport)
	public:

		PX_FORCE_INLINE					ActorPairReport(ActorSim&, ActorSim&);
		PX_FORCE_INLINE					~ActorPairReport();

		PX_INLINE ContactStreamManager&	createContactStreamManager(NPhaseCore&);
		PX_FORCE_INLINE ContactStreamManager& getContactStreamManager() const { PX_ASSERT(mReportData); return mReportData->mContactStreamManager; }
		PX_FORCE_INLINE	ActorSim&		getActorA() const { return mActorA; }
		PX_FORCE_INLINE ActorSim&		getActorB() const { return mActorB; }
		PX_INLINE		PxU32			getActorAID() const { PX_ASSERT(mReportData); return mReportData->mActorAID; }
		PX_INLINE		PxU32			getActorBID() const { PX_ASSERT(mReportData); return mReportData->mActorBID; }
		PX_INLINE		PxActor*		getPxActorA() const { PX_ASSERT(mReportData); return mReportData->mPxActorA; }
		PX_INLINE		PxActor*		getPxActorB() const { PX_ASSERT(mReportData); return mReportData->mPxActorB; }
		PX_INLINE		bool			streamResetStamp(PxU32 cmpStamp);

		PX_FORCE_INLINE	PxIntBool		isInContactReportActorPairSet() const { return (getFlags() & eIS_IN_CONTACT_REPORT_ACTOR_PAIR_SET); }
		PX_FORCE_INLINE	void			setInContactReportActorPairSet()	{ raiseFlags(eIS_IN_CONTACT_REPORT_ACTOR_PAIR_SET); }
		PX_FORCE_INLINE	void			clearInContactReportActorPairSet()	{ clearFlags(eIS_IN_CONTACT_REPORT_ACTOR_PAIR_SET); }

		PX_FORCE_INLINE void			createContactReportData(NPhaseCore&);

		PX_FORCE_INLINE	void			convert(ActorPair& aPair) { PX_ASSERT(!aPair.isReportPair()); setTouchCount(aPair.getTouchCount()); mRefCount = aPair.getRefCount(); }

		PX_FORCE_INLINE static ActorPairReport& cast(ActorPair& aPair) { PX_ASSERT(aPair.isReportPair()); return static_cast<ActorPairReport&>(aPair); }

		private:
						ActorSim&		mActorA;
						ActorSim&		mActorB;
		public:
			ActorPairContactReportData* mReportData;
	};
} // namespace Sc

//// Sc::ActorPair implementations

PX_FORCE_INLINE void Sc::ActorPair::setTouchCount(PxU32 count)
{
	PX_ASSERT(count <= (PX_MAX_U32 >> sTouchCountShift));
	if (count == (PX_MAX_U32 >> sTouchCountShift)) // we rely on inc/dec behaviour here which means we'll always pass the max value before overflowing.
	{
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Sc::ActorPair: maximum count of touching shapes per pair reached, any additional touch will introduce undefined behavior.");
	}

	mTouchCount_internalFlags = (mTouchCount_internalFlags & sFlagMask) | (count << sTouchCountShift);
}

PX_FORCE_INLINE void Sc::ActorPair::clearFlags(PxU32 flags)
{
	PX_ASSERT(flags < ActorPairFlags::eMAX);

	PxU32 tmpFlags = getFlags();
	tmpFlags &= ~flags;
	mTouchCount_internalFlags &= ~sFlagMask;
	raiseFlags(tmpFlags);
}

//// Sc::ActorPairReport implementations

PX_FORCE_INLINE Sc::ActorPairReport::ActorPairReport(ActorSim& actor0, ActorSim& actor1) : ActorPair(),
mActorA			(actor0),
mActorB			(actor1),
mReportData		(NULL)
{
	PX_ASSERT(getFlags() == 0);
	raiseFlags(ActorPair::eIS_REPORT_PAIR);
}

PX_FORCE_INLINE Sc::ActorPairReport::~ActorPairReport()
{
	PX_ASSERT(mReportData == NULL);
}

PX_INLINE bool Sc::ActorPairReport::streamResetStamp(PxU32 cmpStamp) 
{
	PX_ASSERT(mReportData);
	const bool ret = (cmpStamp != mReportData->mStrmResetStamp);
	mReportData->mStrmResetStamp = cmpStamp; 
	return ret; 
}

PX_INLINE Sc::ContactStreamManager&	Sc::ActorPairReport::createContactStreamManager(NPhaseCore& npCore)
{
	// Lazy create report data
	if(!mReportData)
		createContactReportData(npCore);

	return mReportData->mContactStreamManager;
}

PX_FORCE_INLINE void Sc::ActorPairReport::createContactReportData(NPhaseCore& npCore)
{
	PX_ASSERT(!mReportData);
	Sc::ActorPairContactReportData* reportData = npCore.createActorPairContactReportData(); 
	mReportData = reportData;

	if(reportData)
	{
		const ActorCore& actorCoreA = mActorA.getActorCore();
		const ActorCore& actorCoreB = mActorB.getActorCore();

		reportData->mActorAID = mActorA.getActorID();
		reportData->mActorBID = mActorB.getActorID();

#if PX_SUPPORT_GPU_PHYSX
		if (mActorA.getActorType() == PxActorType::eDEFORMABLE_VOLUME)
			reportData->mPxActorA = static_cast<const DeformableVolumeCore&>(actorCoreA).getPxActor();
		else if (mActorA.getActorType() == PxActorType::eDEFORMABLE_SURFACE)
			reportData->mPxActorA = static_cast<const DeformableSurfaceCore&>(actorCoreA).getPxActor();
		else
#endif
			reportData->mPxActorA = static_cast<const RigidCore&>(actorCoreA).getPxActor();

#if PX_SUPPORT_GPU_PHYSX
		if (mActorB.getActorType() == PxActorType::eDEFORMABLE_VOLUME)
			reportData->mPxActorB = static_cast<const DeformableVolumeCore&>(actorCoreB).getPxActor();
		else if (mActorB.getActorType() == PxActorType::eDEFORMABLE_SURFACE)
			reportData->mPxActorB = static_cast<const DeformableSurfaceCore&>(actorCoreB).getPxActor();
		else
#endif
			reportData->mPxActorB = static_cast<const RigidCore&>(actorCoreB).getPxActor();
	}
}

}

#endif
