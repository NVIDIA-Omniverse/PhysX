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

#ifndef SC_NPHASE_CORE_H
#define SC_NPHASE_CORE_H

#include "foundation/PxHash.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxHashSet.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxMutex.h"
#include "foundation/PxAtomic.h"
#include "PxPhysXConfig.h"

#include "foundation/PxPool.h"

#include "PxSimulationEventCallback.h"
#include "ScTriggerPairs.h"
#include "ScScene.h"
#include "ScContactReportBuffer.h"

namespace physx
{
namespace Bp
{
	struct AABBOverlap;
}

namespace Sc
{
	class ElementSim;
	class ShapeSimBase;

	class Interaction;
	class ElementSimInteraction;
	class ElementInteractionMarker;
	class TriggerInteraction;

	class ShapeInteraction;
	class ActorPair;
	class ActorPairReport;

	class ActorPairContactReportData;

	class ContactStreamManager;

	class TriggerContactTask;

	struct PairReleaseFlag
	{
		enum Enum
		{
			eRUN_LOST_TOUCH_LOGIC		=	(1 << 0),	// run the lost-touch-logic for a pair that gets removed.
			eWAKE_ON_LOST_TOUCH			=	(1 << 1)	// a pair that lost touch should check whether the actors should get woken up
		};
	};

	/*
	NPhaseCore encapsulates the near phase processing to allow multiple implementations(eg threading and non threaded).

	The broadphase inserts shape pairs into the NPhaseCore, which are then processed into contact point streams.
	Pairs can then be processed into AxisConstraints by the GroupSolveCore.
	*/

	struct BodyPairKey
	{
		PX_FORCE_INLINE	BodyPairKey(PxU32 sim0, PxU32 sim1) : mSim0(sim0), mSim1(sim1)	{}

		const PxU32 mSim0;
		const PxU32 mSim1;

		PX_FORCE_INLINE	bool operator == (const BodyPairKey& pair) const { return mSim0 == pair.mSim0 && mSim1 == pair.mSim1; }
	};

	PX_INLINE PxU32 PxComputeHash(const BodyPairKey& key)
	{
		const PxU32 add0 = key.mSim0;
		const PxU32 add1 = key.mSim1;

		const PxU32 base = PxU32((add0 & 0xFFFF) | (add1 << 16));

		return physx::PxComputeHash(base);
	}

	struct ElementSimKey
	{
		PxU32 mID0;
		PxU32 mID1;

		ElementSimKey() : mID0(0xffffffff), mID1(0xffffffff)
		{}

		ElementSimKey(PxU32 id0, PxU32 id1)
		{
			if(id0 > id1)
				PxSwap(id0, id1);
			 mID0 = id0;
			 mID1 = id1;
		}

		PX_FORCE_INLINE	bool operator == (const ElementSimKey& pair) const { return mID0 == pair.mID0 && mID1 == pair.mID1; }
	};

	PX_INLINE PxU32 PxComputeHash(const ElementSimKey& key)
	{
		const PxU64 base = PxU64(key.mID0) | (PxU64(key.mID1) << 32);
		return physx::PxComputeHash(base);
	}

	class ContactReportAllocationManager
	{
		PxU8* mBuffer;
		PxU32 mBufferSize;
		PxU32 mCurrentBufferIndex;
		PxU32 mCurrentOffset;
		ContactReportBuffer& mReportBuffer;
		PxMutex& mMutex;
		const PxU32 mBuferBlockSize;
		PX_NOCOPY(ContactReportAllocationManager)
	public:

		ContactReportAllocationManager(ContactReportBuffer& buffer, PxMutex& mutex, const PxU32 bufferBlockSize = 16384) : mBuffer(NULL), mBufferSize(0), mCurrentBufferIndex(0), 
			mCurrentOffset(0), mReportBuffer(buffer), mMutex(mutex), mBuferBlockSize(bufferBlockSize)
		{
		}

		PxU8* allocate(PxU32 size, PxU32& index, PxU32 alignment = 16u)
		{
			//(1) fix up offsets...
			const PxU32 pad = ((mCurrentBufferIndex + alignment - 1)&~(alignment - 1)) - mCurrentBufferIndex;
			PxU32 currOffset = mCurrentOffset + pad;

			if ((currOffset + size) > mBufferSize)
			{
				const PxU32 allocSize = PxMax(size, mBuferBlockSize);

				mMutex.lock();
				mBuffer = mReportBuffer.allocateNotThreadSafe(allocSize, mCurrentBufferIndex, alignment);
				mCurrentOffset = currOffset = 0;
				mBufferSize = allocSize;
				mMutex.unlock();
			}

			PxU8* ret = mBuffer + currOffset;
			index = mCurrentBufferIndex + currOffset;
			mCurrentOffset = currOffset + size;
			return ret;
		}
	};

	class TriggerProcessingContext
	{
	public:
		TriggerProcessingContext()
			: mTmpTriggerProcessingBlock(NULL)
			, mTmpTriggerPairCount(0)
		{
		}

		bool initialize(TriggerInteraction**, PxU32 pairCount, PxcScratchAllocator&);

		void deinitialize(PxcScratchAllocator&);

		PX_FORCE_INLINE TriggerInteraction* const* getTriggerInteractions() const
		{
			return reinterpret_cast<TriggerInteraction**>(mTmpTriggerProcessingBlock);
		}

		PX_FORCE_INLINE PxU32 getTriggerInteractionCount() const
		{
			return mTmpTriggerPairCount;
		}

		PX_FORCE_INLINE TriggerContactTask* getTriggerContactTasks()
		{
			const PxU32 offset = mTmpTriggerPairCount * sizeof(TriggerInteraction*);
			return reinterpret_cast<TriggerContactTask*>(mTmpTriggerProcessingBlock + offset);
		}

		PX_FORCE_INLINE PxMutex& getTriggerWriteBackLock()
		{
			return mTriggerWriteBackLock;
		}

	private:
		PxU8* mTmpTriggerProcessingBlock;  // temporary memory block to process trigger pairs in parallel
		                                   // (see comment in Sc::Scene::postIslandGen too)
		PxU32 mTmpTriggerPairCount;
		PxMutex mTriggerWriteBackLock;
	};

	class NPhaseCore : public PxUserAllocated
	{
		PX_NOCOPY(NPhaseCore)

	public:
		NPhaseCore(Scene& scene, const PxSceneDesc& desc);
		~NPhaseCore();

		ElementSimInteraction* findInteraction(const ElementSim* element0, const ElementSim* element1);

		void	onTriggerOverlapCreated(const Bp::AABBOverlap* PX_RESTRICT pairs, PxU32 pairCount);

		void	runOverlapFilters(	PxU32 nbToProcess, const Bp::AABBOverlap* PX_RESTRICT pairs, FilterInfo* PX_RESTRICT filterInfo,
									PxU32& nbToKeep, PxU32& nbToSuppress, PxU32* PX_RESTRICT keepMap);

		void onOverlapRemoved(ElementSim* volume0, ElementSim* volume1, PxU32 ccdPass, void* elemSim, PxsContactManagerOutputIterator& outputs);
		void onVolumeRemoved(ElementSim* volume, PxU32 flags, PxsContactManagerOutputIterator& outputs);

		void managerNewTouch(ShapeInteraction& interaction);

		PxU32 getDefaultContactReportStreamBufferSize() const;

		void fireCustomFilteringCallbacks(PxsContactManagerOutputIterator& outputs);

		void addToDirtyInteractionList(Interaction* interaction);
		void removeFromDirtyInteractionList(Interaction* interaction);
		void updateDirtyInteractions(PxsContactManagerOutputIterator& outputs);

		/**
		\brief Allocate buffers for trigger overlap test.

		See comment in Sc::Scene::postIslandGen for why this is split up into multiple parts.

		\param[in] continuation The task to run after trigger processing.
		\return The concluding trigger processing task if there is work to do, else NULL.
		*/
		PxBaseTask* prepareForTriggerInteractionProcessing(PxBaseTask* continuation);

		// Perform trigger overlap tests.
		void processTriggerInteractions(PxBaseTask& continuation);
		
		// Deactivate trigger interactions if possible, free buffers from overlap tests and clean up.
		// See comment in Sc::Scene::postIslandGen for why this is split up into multiple parts.
		void concludeTriggerInteractionProcessing(PxBaseTask* continuation);

		// Check candidates for persistent touch contact events and create those events if necessary.
		void processPersistentContactEvents(PxsContactManagerOutputIterator& outputs);

		PX_FORCE_INLINE void addToContactReportActorPairSet(ActorPairReport* pair) { mContactReportActorPairSet.pushBack(pair); }
		void clearContactReportActorPairs(bool shrinkToZero);
		PX_FORCE_INLINE PxU32 getNbContactReportActorPairs() const { return mContactReportActorPairSet.size(); }
		PX_FORCE_INLINE ActorPairReport* const* getContactReportActorPairs() const { return mContactReportActorPairSet.begin(); }

		void addToPersistentContactEventPairs(ShapeInteraction*);
		void addToPersistentContactEventPairsDelayed(ShapeInteraction*);
		void removeFromPersistentContactEventPairs(ShapeInteraction*);

		PX_FORCE_INLINE	PxU32						getCurrentPersistentContactEventPairCount()	const { return mNextFramePersistentContactEventPairIndex;	}
		PX_FORCE_INLINE	ShapeInteraction* const*	getCurrentPersistentContactEventPairs()		const { return mPersistentContactEventPairList.begin();		}
		PX_FORCE_INLINE	PxU32						getAllPersistentContactEventPairCount()		const { return mPersistentContactEventPairList.size();		}
		PX_FORCE_INLINE	ShapeInteraction* const*	getAllPersistentContactEventPairs()			const { return mPersistentContactEventPairList.begin();		}
		PX_FORCE_INLINE	void						preparePersistentContactEventListForNextFrame()
													{
														// reports have been processed -> "activate" next frame candidates for persistent contact events
														mNextFramePersistentContactEventPairIndex = mPersistentContactEventPairList.size();
													}

		void addToForceThresholdContactEventPairs(ShapeInteraction*);
		void removeFromForceThresholdContactEventPairs(ShapeInteraction*);

		PX_FORCE_INLINE PxU32 getForceThresholdContactEventPairCount() const { return mForceThresholdContactEventPairList.size(); }
		PX_FORCE_INLINE ShapeInteraction* const* getForceThresholdContactEventPairs() const { return mForceThresholdContactEventPairList.begin(); }

		PX_FORCE_INLINE PxU8* getContactReportPairData(const PxU32& bufferIndex) const { return mContactReportBuffer.getData(bufferIndex); }
		PxU8* reserveContactReportPairData(PxU32 pairCount, PxU32 extraDataSize, PxU32& bufferIndex, ContactReportAllocationManager* alloc = NULL);
		PxU8* resizeContactReportPairData(PxU32 pairCount, PxU32 extraDataSize, ContactStreamManager& csm);
		PX_FORCE_INLINE void clearContactReportStream() { mContactReportBuffer.reset(); }  // Do not free memory at all
		PX_FORCE_INLINE void freeContactReportStreamMemory() { mContactReportBuffer.flush(); }

		ActorPairContactReportData* createActorPairContactReportData();

		void registerInteraction(ElementSimInteraction* interaction);
		void unregisterInteraction(ElementSimInteraction* interaction);
		
		ElementSimInteraction* createRbElementInteraction(const FilterInfo& fInfo, ShapeSimBase& s0, ShapeSimBase& s1, PxsContactManager* contactManager, ShapeInteraction* shapeInteraction, 
			ElementInteractionMarker* interactionMarker, bool isTriggerPair);

		PX_FORCE_INLINE	void lockReports()		{ mReportAllocLock.lock();		}
		PX_FORCE_INLINE	void unlockReports()	{ mReportAllocLock.unlock();	}

	private:
		void callPairLost(const ShapeSimBase& s0, const ShapeSimBase& s1, bool objVolumeRemoved);

		ElementSimInteraction* createTriggerElementInteraction(ShapeSimBase& s0, ShapeSimBase& s1);

		// removedElement: points to the removed element (that is, the BP volume wrapper), if a pair gets removed or loses touch due to a removed element.
		//                 NULL if not triggered by a removed element.
		//
		void releaseElementPair(ElementSimInteraction* pair, PxU32 flags, ElementSim* removedElement, PxU32 ccdPass, bool removeFromDirtyList, PxsContactManagerOutputIterator& outputs);
		void lostTouchReports(ShapeInteraction* pair, PxU32 flags, ElementSim* removedElement, PxU32 ccdPass, PxsContactManagerOutputIterator& outputs);

		ShapeInteraction* createShapeInteraction(ShapeSimBase& s0, ShapeSimBase& s1, PxPairFlags pairFlags, PxsContactManager* contactManager, ShapeInteraction* shapeInteraction);
		TriggerInteraction* createTriggerInteraction(ShapeSimBase& s0, ShapeSimBase& s1, PxPairFlags triggerFlags);
		ElementInteractionMarker* createElementInteractionMarker(ElementSim& e0, ElementSim& e1, ElementInteractionMarker* marker);

		//------------- Filtering -------------

		ElementSimInteraction* refilterInteraction(ElementSimInteraction* pair, const FilterInfo* filterInfo, bool removeFromDirtyList, PxsContactManagerOutputIterator& outputs);
		//-------------------------------------

		ElementSimInteraction* convert(ElementSimInteraction* pair, InteractionType::Enum type, FilterInfo& filterInfo, bool removeFromDirtyList, PxsContactManagerOutputIterator& outputs);

		ActorPair* findActorPair(ShapeSimBase* s0, ShapeSimBase* s1, PxIntBool isReportPair);

		// Pooling
		Scene&										mOwnerScene;

		PxArray<ActorPairReport*>					mContactReportActorPairSet;
		PxArray<ShapeInteraction*>					mPersistentContactEventPairList;	// Pairs which request events which do not get triggered by the sdk and thus need to be tested actively every frame.
																						// May also contain force threshold event pairs (see mForceThresholdContactEventPairList)
																						// This list is split in two, the elements in front are for the current frame, the elements at the
																						// back will get added next frame.

		PxU32										mNextFramePersistentContactEventPairIndex;  // start index of the pairs which need to get added to the persistent list for next frame

		PxArray<ShapeInteraction*>					mForceThresholdContactEventPairList;	// Pairs which request force threshold contact events. A pair is only in this list if it does have contact.
																							// Note: If a pair additionally requests PxPairFlag::eNOTIFY_TOUCH_PERSISTS events, then it
																							// goes into mPersistentContactEventPairList instead. This allows to share the list index.

		//  data layout:
		//  ContactActorPair0_ExtraData, ContactShapePair0_0, ContactShapePair0_1, ... ContactShapePair0_N, 
		//  ContactActorPair1_ExtraData, ContactShapePair1_0, ...
		//
		ContactReportBuffer							mContactReportBuffer;				// Shape pair information for contact reports

		PxCoalescedHashSet<Interaction*>			mDirtyInteractions;
		// Pools
		PxPool<ActorPair>							mActorPairPool;
		PxPool<ActorPairReport>						mActorPairReportPool;
		PxPool<ShapeInteraction>					mShapeInteractionPool;
		PxPool<TriggerInteraction>					mTriggerInteractionPool;
		PxPool<ActorPairContactReportData>			mActorPairContactReportDataPool;
		PxPool<ElementInteractionMarker>			mInteractionMarkerPool;

		Cm::DelegateTask<NPhaseCore, &NPhaseCore::concludeTriggerInteractionProcessing> mConcludeTriggerInteractionProcessingTask;
		TriggerProcessingContext					mTriggerProcessingContext;
		PxHashMap<BodyPairKey, ActorPair*>			mActorPairMap; 

		PxHashMap<ElementSimKey, ElementSimInteraction*> mElementSimMap;

		PxMutex										mBufferAllocLock;
		PxMutex										mReportAllocLock;

		friend class Sc::Scene;
		friend class Sc::ShapeInteraction;
	};

	struct FilteringContext
	{
		PX_NOCOPY(FilteringContext)
	public:
		FilteringContext(const Scene& scene) :
			mFilterShader			(scene.getFilterShaderFast()),
			mFilterShaderData		(scene.getFilterShaderDataFast()),
			mFilterShaderDataSize	(scene.getFilterShaderDataSizeFast()),
			mFilterCallback			(scene.getFilterCallbackFast()),
			mKineKineFilteringMode	(scene.getKineKineFilteringMode()),
			mStaticKineFilteringMode(scene.getStaticKineFilteringMode())
		{
		}

		PxSimulationFilterShader			mFilterShader;
		const void*							mFilterShaderData;
		PxU32								mFilterShaderDataSize;
		PxSimulationFilterCallback*			mFilterCallback;
		const PxPairFilteringMode::Enum		mKineKineFilteringMode;
		const PxPairFilteringMode::Enum		mStaticKineFilteringMode;
	};

} // namespace Sc

}

#endif
