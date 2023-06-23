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

#ifndef BP_AABBMANAGER_H
#define BP_AABBMANAGER_H

#include "foundation/PxHashSet.h"
#include "foundation/PxHashMap.h"
#include "BpAABBManagerTasks.h"
#include "BpAABBManagerBase.h"

namespace physx
{
namespace Cm
{
	class FlushPool;
}

namespace Bp
{
	struct BroadPhasePair;

	class Aggregate;
	class PersistentPairs;
	class PersistentActorAggregatePair;
	class PersistentAggregateAggregatePair;
	class PersistentSelfCollisionPairs;

	struct AggPair
	{
		PX_FORCE_INLINE AggPair() {}
		PX_FORCE_INLINE	AggPair(ShapeHandle index0, ShapeHandle index1) : mIndex0(index0), mIndex1(index1)	{}
		ShapeHandle	mIndex0;
		ShapeHandle	mIndex1;

		PX_FORCE_INLINE bool operator==(const AggPair& p) const
		{
			return (p.mIndex0 == mIndex0) && (p.mIndex1 == mIndex1);
		}
	};
	typedef PxCoalescedHashMap<AggPair, PersistentPairs*> AggPairMap;

	// PT: TODO: isn't there a generic pair structure somewhere? refactor with AggPair anyway
	struct Pair
	{
		PX_FORCE_INLINE	Pair(PxU32 id0, PxU32 id1) : mID0(id0), mID1(id1)	{}
		PX_FORCE_INLINE Pair(){}

			PX_FORCE_INLINE bool operator<(const Pair& p) const
			{
				const PxU64 value0 = *reinterpret_cast<const PxU64*>(this);
				const PxU64 value1 = *reinterpret_cast<const PxU64*>(&p);
				return value0 < value1;
			}

			PX_FORCE_INLINE bool operator==(const Pair& p) const
			{
				return (p.mID0 == mID0) && (p.mID1 == mID1);
			}

			PX_FORCE_INLINE bool operator!=(const Pair& p) const
			{
				return (p.mID0 != mID0) || (p.mID1 != mID1);
			}

		PxU32	mID0;
		PxU32	mID1;
	};

	class AABBManager;

	class PostBroadPhaseStage2Task : public Cm::Task
	{
		Cm::FlushPool* mFlushPool;
		AABBManager& mManager;

		PX_NOCOPY(PostBroadPhaseStage2Task)
	public:

		PostBroadPhaseStage2Task(PxU64 contextID, AABBManager& manager) : Cm::Task(contextID), mFlushPool(NULL), mManager(manager)
		{
		}

		virtual const char* getName() const { return "PostBroadPhaseStage2Task"; }

		void setFlushPool(Cm::FlushPool* pool) { mFlushPool = pool; }

		virtual void runInternal();
	};

	class ProcessAggPairsBase;

	/**
	\brief A structure responsible for:
	* storing an aabb representation for each active shape in the related scene
	* managing the creation/removal of aabb representations when their related shapes are created/removed
	* updating all aabbs that require an update due to modification of shape geometry or transform
	* updating the aabb of all aggregates from the union of the aabbs of all shapes that make up each aggregate
	* computing and reporting the incremental changes to the set of overlapping aabb pairs 
	*/
	class AABBManager : public AABBManagerBase
	{
													PX_NOCOPY(AABBManager)
	public:
														AABBManager(BroadPhase& bp, BoundsArray& boundsArray, PxFloatArrayPinned& contactDistance,
																	PxU32 maxNbAggregates, PxU32 maxNbShapes, PxVirtualAllocator& allocator, PxU64 contextID,
																	PxPairFilteringMode::Enum kineKineFilteringMode, PxPairFilteringMode::Enum staticKineFilteringMode);

		virtual											~AABBManager() {}

		// AABBManagerBase
		virtual			void							destroy()	PX_OVERRIDE;
		virtual			AggregateHandle					createAggregate(BoundsIndex index, Bp::FilterGroup::Enum group, void* userData, PxU32 maxNumShapes, PxAggregateFilterHint filterHint)	PX_OVERRIDE;
		virtual			bool							destroyAggregate(BoundsIndex& index, Bp::FilterGroup::Enum& group, AggregateHandle aggregateHandle)	PX_OVERRIDE;
		virtual			bool							addBounds(BoundsIndex index, PxReal contactDistance, Bp::FilterGroup::Enum group, void* userdata, AggregateHandle aggregateHandle, ElementType::Enum volumeType)	PX_OVERRIDE;
		virtual			bool							removeBounds(BoundsIndex index)	PX_OVERRIDE;
		virtual			void							updateBPFirstPass(PxU32 numCpuTasks, Cm::FlushPool& flushPool, bool hasContactDistanceUpdated, PxBaseTask* continuation)	PX_OVERRIDE;
		virtual			void							updateBPSecondPass(PxcScratchAllocator* scratchAllocator, PxBaseTask* continuation)	PX_OVERRIDE;
		virtual			void							postBroadPhase(PxBaseTask*, Cm::FlushPool& flushPool)	PX_OVERRIDE;
		virtual			void							reallocateChangedAABBMgActorHandleMap(const PxU32 size)	PX_OVERRIDE;
		virtual			bool							getOutOfBoundsObjects(OutOfBoundsData& data)			PX_OVERRIDE;
		virtual			void							clearOutOfBoundsObjects()								PX_OVERRIDE;
		virtual			void							visualize(PxRenderOutput& out)	PX_OVERRIDE;
		virtual			void							releaseDeferredAggregateIds()	PX_OVERRIDE{}
		//~AABBManagerBase

						void							preBpUpdate_CPU(PxU32 numCpuTasks);

						// PT: TODO: what is that BpCacheData for?
						BpCacheData*					getBpCacheData();
						void							putBpCacheData(BpCacheData*);
						void							resetBpCacheData();

						PxMutex							mMapLock;
	private:
						//void reserveShapeSpace(PxU32 nbShapes);

						void							postBpStage2(PxBaseTask*, Cm::FlushPool&);
						void							postBpStage3(PxBaseTask*);

						PostBroadPhaseStage2Task									mPostBroadPhase2;
						Cm::DelegateTask<AABBManager, &AABBManager::postBpStage3>	mPostBroadPhase3;

						PreBpUpdateTask					mPreBpUpdateTask;

						PxU32							mTimestamp;
						PxU32							mFirstFreeAggregate;
						PxArray<Aggregate*>				mAggregates;		// PT: indexed by AggregateHandle
						PxArray<Aggregate*>				mDirtyAggregates;

						AggPairMap						mActorAggregatePairs;
						AggPairMap						mAggregateAggregatePairs;

						PxArray<ProcessAggPairsBase*>	mAggPairTasks;

						PxHashSet<Pair>					mCreatedPairsTmp;	// PT: temp hashset for dubious post filtering, persistent to minimize allocs

						PxSList							mBpThreadContextPool;

						PxArray<void*>					mOutOfBoundsObjects;
						PxArray<void*>					mOutOfBoundsAggregates;

		PX_FORCE_INLINE	Aggregate*						getAggregateFromHandle(AggregateHandle handle)
														{
															PX_ASSERT(handle<mAggregates.size());
															return mAggregates[handle];
														}

						void							startAggregateBoundsComputationTasks(PxU32 nbToGo, PxU32 numCpuTasks, Cm::FlushPool& flushPool);
						PersistentActorAggregatePair*	createPersistentActorAggregatePair(ShapeHandle volA, ShapeHandle volB);
					PersistentAggregateAggregatePair*	createPersistentAggregateAggregatePair(ShapeHandle volA, ShapeHandle volB);
						void							updatePairs(PersistentPairs& p, BpCacheData* data = NULL);
						void							handleOriginShift();

	public:
						void							processBPCreatedPair(const BroadPhasePair& pair);
						void							processBPDeletedPair(const BroadPhasePair& pair);

		friend class PersistentActorAggregatePair;
		friend class PersistentAggregateAggregatePair;
		friend class ProcessSelfCollisionPairsParallel;
		friend class PostBroadPhaseStage2Task;
	};

} //namespace Bp
} //namespace physx

#endif //BP_AABBMANAGER_H
