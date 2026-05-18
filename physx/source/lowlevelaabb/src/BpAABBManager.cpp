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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "BpAABBManager.h"

#define NB_SENTINELS			6

// PT: BOUNDS_DOUBLE_BUFFERING avoids some memcopies during sorting, at the cost of
// increased memory usage. The jury is still out. It feels better to save memory
// in this case, as the extended buffers can also produce more cache misses anyway.
#define BOUNDS_DOUBLE_BUFFERING	0

#include "foundation/PxHashSet.h"
#include "CmUtils.h"
#include "CmFlushPool.h"
#include "CmVisualization.h"
#include "CmRadixSort.h"
#include "BpBroadPhaseMBPCommon.h"
#include "BpBroadPhase.h"
#include "BpBroadPhaseShared.h"
#include "foundation/PxSort.h"
#include "foundation/PxVecMath.h"
#include "GuInternal.h"
#include "common/PxProfileZone.h"

using namespace physx;
using namespace Bp;
using namespace Cm;
using namespace aos;

static const bool gSingleThreaded = false;
#if PX_INTEL_FAMILY && !defined(PX_SIMD_DISABLED)
	#define ABP_SIMD_OVERLAP
#endif
#ifdef ABP_SIMD_OVERLAP
	typedef AABB_YZn	AABB_YZ;
#else
	typedef AABB_YZr	AABB_YZ;
#endif

#ifdef ABP_SIMD_OVERLAP
	static const bool gUseRegularBPKernel = false;	// false to use "version 13" in box pruning series
	static const bool gUnrollLoop = true;			// true to use "version 14" in box pruning series
#else
	// PT: tested on Switch, for some reason the regular version is fastest there
	static const bool gUseRegularBPKernel = true;	// false to use "version 13" in box pruning series
	static const bool gUnrollLoop = false;			// true to use "version 14" in box pruning series
#endif

namespace physx
{
namespace Bp
{
static PX_FORCE_INLINE uint32_t PxComputeHash(const Pair& p)
{
	return hash(p.mID0, p.mID1);
}

static PX_FORCE_INLINE uint32_t PxComputeHash(const AggPair& p)
{
	return hash(p.mIndex0, p.mIndex1);
}

static PX_FORCE_INLINE bool shouldPairBeDeleted(const PinnableArray<Bp::FilterGroup::Enum>& groups, ShapeHandle h0, ShapeHandle h1)
{
	PX_ASSERT(h0<groups.size());
	PX_ASSERT(h1<groups.size());
	return (groups[h0]==Bp::FilterGroup::eINVALID) || (groups[h1]==Bp::FilterGroup::eINVALID);
}

///

	typedef PxU32		InflatedType;

	// PT: TODO: revisit/optimize all that stuff once it works
	class Aggregate : public PxUserAllocated
	{
														PX_NOCOPY(Aggregate)
		public:
														Aggregate(BoundsIndex index, PxAggregateFilterHint filterHint);
														~Aggregate();

		PX_FORCE_INLINE	PxU32							getNbAggregated()		const	{ return mAggregated.size();					}
		PX_FORCE_INLINE	BoundsIndex						getAggregated(PxU32 i)	const	{ return mAggregated[i];						}
		PX_FORCE_INLINE	const BoundsIndex*				getIndices()			const	{ return mAggregated.begin();					}
		PX_FORCE_INLINE	void							addAggregated(BoundsIndex i)	{ mAggregated.pushBack(i);						}
		PX_FORCE_INLINE	bool							removeAggregated(BoundsIndex i)	{ return mAggregated.findAndReplaceWithLast(i);	}	// PT: TODO: optimize?
		PX_FORCE_INLINE	const PxBounds3&				getMergedBounds()		const	{ return mBounds;								}

		PX_FORCE_INLINE	void							resetDirtyState()				{ mDirtyIndex = PX_INVALID_U32;				}
		PX_FORCE_INLINE	bool							isDirty()				const	{ return mDirtyIndex != PX_INVALID_U32;		}
		PX_FORCE_INLINE void							markAsDirty(PxArray<Aggregate*>& dirtyAggregates)
														{
															if(!isDirty())
															{
																mDirtyIndex = dirtyAggregates.size();
																dirtyAggregates.pushBack(this);
															}
														}

						void							allocateBounds();
						void							computeBounds(const PxBounds3* PX_RESTRICT bounds, const float* PX_RESTRICT contactDistances) /*PX_RESTRICT*/;

#if BOUNDS_DOUBLE_BUFFERING
		PX_FORCE_INLINE	const AABB_Xi*					getBoundsX()		const	{ return mInflatedBoundsX + (mFlip ? (mAllocatedSize + NB_SENTINELS) : 0);	}
		PX_FORCE_INLINE	const AABB_YZ*					getBoundsYZ()		const	{ return mInflatedBoundsYZ + (mFlip ? mAllocatedSize: 0);					}
		PX_FORCE_INLINE	AABB_Xi*						getBoundsXCopy()			{ return mInflatedBoundsX + (mFlip ? 0 : (mAllocatedSize + NB_SENTINELS));	}
		PX_FORCE_INLINE	AABB_YZ*						getBoundsYZCopy()			{ return mInflatedBoundsYZ + (mFlip ? 0 : mAllocatedSize);					}
#else
		PX_FORCE_INLINE	const AABB_Xi*					getBoundsX()	const	{ return mInflatedBoundsX;	}
		PX_FORCE_INLINE	const AABB_YZ*					getBoundsYZ()	const	{ return mInflatedBoundsYZ;	}
#endif
		PX_FORCE_INLINE	void							getSortedMinBounds()
														{
															if(mDirtySort)
																sortBounds();
														}
		PX_FORCE_INLINE	PxAggregateFilterHint			getFilterHint()	const	{ return PxAggregateFilterHint(mFilterHint);		}

						PxArray<BoundsIndex>			mAggregated;	// PT: TODO: replace with linked list?
						AABB_Xi*						mInflatedBoundsX;
						AABB_YZ*						mInflatedBoundsYZ;
						PersistentSelfCollisionPairs*	mSelfCollisionPairs;
						PxBounds3						mBounds;
						const BoundsIndex				mIndex;
						PxU32							mDirtyIndex;	// PT: index in mDirtyAggregates
						PxU32							mAllocatedSize;
						PxU8							mFilterHint;
						bool							mDirtySort;
#if BOUNDS_DOUBLE_BUFFERING
						bool							mFlip;
#endif
		private:
						void							sortBounds();
	};

///

namespace
{
#define MBP_ALLOC(x)		PX_ALLOC(x, "MBP")
#define MBP_ALLOC_TMP(x)	PX_ALLOC(x, "MBP_TMP")
#define MBP_FREE(x)			PX_FREE(x)

	struct MBPEntry;
	struct RegionHandle;
	struct MBP_Object;

	class MBP_PairManager : public PairManagerData
	{
		public:
										MBP_PairManager()	{}
										~MBP_PairManager()	{}

		PX_FORCE_INLINE	InternalPair*	addPair(PxU32 id0, PxU32 id1);
	};

///////////////////////////////////////////////////////////////////////////////

PX_FORCE_INLINE InternalPair* MBP_PairManager::addPair(PxU32 id0, PxU32 id1)
{
	PX_ASSERT(id0!=INVALID_ID);
	PX_ASSERT(id1!=INVALID_ID);
	return addPairInternal(id0, id1);
}

///////////////////////////////////////////////////////////////////////////////

}

///

class PersistentPairs : public PxUserAllocated
{
	public:
									PersistentPairs() : mTimestamp(PX_INVALID_U32), mShouldBeDeleted(false)	{}
	virtual							~PersistentPairs()														{}

	virtual			bool			update(AABBManager& /*manager*/, BpCacheData* /*data*/ = NULL) { return false; }

	PX_FORCE_INLINE	void			updatePairs(PxU32 timestamp, const PxBounds3* bounds, const float* contactDistances, const Bp::FilterGroup::Enum* groups,
												const bool* lut, VolumeData* volumeData, PxArray<AABBOverlap>* createdOverlaps, PxArray<AABBOverlap>* destroyedOverlaps);
					void			outputDeletedOverlaps(PxArray<AABBOverlap>* overlaps, const VolumeData* volumeData);
	private:
	virtual			void			findOverlaps(MBP_PairManager& pairs, const PxBounds3* PX_RESTRICT bounds, const float* PX_RESTRICT contactDistances, const Bp::FilterGroup::Enum* PX_RESTRICT groups, const bool* PX_RESTRICT lut)	= 0;
	protected:
					PxU32			mTimestamp;
					MBP_PairManager	mPM;
	public:
					bool			mShouldBeDeleted;
};

/////

	struct outputPair_Shared
	{
		outputPair_Shared	(MBP_PairManager* pairManager, const Bp::FilterGroup::Enum* groups, const bool* lut) :
			mPairManager	(pairManager),
			mGroups			(groups),
			mLUT			(lut),
			mInToOut0		(NULL),
			mInToOut1		(NULL)
		{
		}

		PX_FORCE_INLINE	void outputPair(PxU32 index0, PxU32 index1)
		{
			const PxU32 aggIndex0 = mInToOut0[index0];
			const PxU32 aggIndex1 = mInToOut1[index1];
			if(groupFiltering(mGroups[aggIndex0], mGroups[aggIndex1], mLUT))
				mPairManager->addPair(aggIndex0, aggIndex1);
		}

		MBP_PairManager*				mPairManager;
		const Bp::FilterGroup::Enum*	mGroups;
		const bool*						mLUT;
		const BoundsIndex*				mInToOut0;
		const BoundsIndex*				mInToOut1;
	};

/////

	#define PosXType2	PxU32

	typedef	AABB_YZ	SIMD_AABB_YZ4;
	typedef	AABB_Xi	SIMD_AABB_X4;

static PX_FORCE_INLINE void outputPair(outputPair_Shared& pairManager, PxU32 index0, PxU32 index1)
{
	pairManager.outputPair(index0, index1);
}

#include "BpBoxPruningKernels.h"

static PX_FORCE_INLINE void doBipartiteBoxPruning_Leaf(
		MBP_PairManager* PX_RESTRICT pairManager, const bool* PX_RESTRICT lut,
		Aggregate* PX_RESTRICT aggregate0, Aggregate* PX_RESTRICT aggregate1, const Bp::FilterGroup::Enum* PX_RESTRICT groups)
{
	const PxU32 nb0 =  aggregate0->getNbAggregated();
	const PxU32 nb1 =  aggregate1->getNbAggregated();
	const BoundsIndex* PX_RESTRICT remap0 = aggregate0->getIndices();
	const BoundsIndex* PX_RESTRICT remap1 = aggregate1->getIndices();
	const AABB_Xi* PX_RESTRICT boxes0_X = aggregate0->getBoundsX();
	const AABB_YZ* PX_RESTRICT boxes0_YZ = aggregate0->getBoundsYZ();
	const AABB_Xi* PX_RESTRICT boxes1_X = aggregate1->getBoundsX();
	const AABB_YZ* PX_RESTRICT boxes1_YZ = aggregate1->getBoundsYZ();

	outputPair_Shared pm(pairManager, groups, lut);
	boxPruningKernel<0>(nb0, nb1, boxes0_X, boxes1_X, boxes0_YZ, boxes1_YZ, remap0, remap1, &pm);
	boxPruningKernel<1>(nb1, nb0, boxes1_X, boxes0_X, boxes1_YZ, boxes0_YZ, remap1, remap0, &pm);
}

/////

class PersistentActorAggregatePair : public PersistentPairs
{
	public:
								PersistentActorAggregatePair(Aggregate* aggregate, ShapeHandle actorHandle);
	virtual						~PersistentActorAggregatePair()	{}

	virtual			void		findOverlaps(MBP_PairManager& pairs, const PxBounds3* PX_RESTRICT bounds, const float* PX_RESTRICT contactDistances, const Bp::FilterGroup::Enum* PX_RESTRICT groups, const bool* PX_RESTRICT lut);
	virtual			bool		update(AABBManager& manager, BpCacheData* data);

					ShapeHandle	mAggregateHandle;
					ShapeHandle	mActorHandle;
					Aggregate*	mAggregate;
};

PersistentActorAggregatePair::PersistentActorAggregatePair(Aggregate* aggregate, ShapeHandle actorHandle) :
	mAggregateHandle	(aggregate->mIndex),
	mActorHandle		(actorHandle),
	mAggregate			(aggregate)
{
}

void PersistentActorAggregatePair::findOverlaps(MBP_PairManager& pairs, const PxBounds3* PX_RESTRICT bounds, const float* PX_RESTRICT contactDistances, const Bp::FilterGroup::Enum* PX_RESTRICT groups, const bool* PX_RESTRICT lut)
{
	if(0)
	{
		Aggregate singleActor(INVALID_ID, false);
		singleActor.addAggregated(mActorHandle);
		singleActor.allocateBounds();
		singleActor.computeBounds(bounds, contactDistances);
		singleActor.getSortedMinBounds();
		mAggregate->getSortedMinBounds();
		doBipartiteBoxPruning_Leaf(&pairs, lut, &singleActor, mAggregate, groups);
	}
	else
	{

	mAggregate->getSortedMinBounds();
	const PxU32 nb0 = mAggregate->getNbAggregated();
	const BoundsIndex* PX_RESTRICT remap0 = mAggregate->getIndices();
	const AABB_Xi* PX_RESTRICT boxes0_X = mAggregate->getBoundsX();
	const AABB_YZ* PX_RESTRICT boxes0_YZ = mAggregate->getBoundsYZ();

	PX_ALIGN(16, AABB_Xi inflatedBoundsX[1+NB_SENTINELS]);
	PX_ALIGN(16, AABB_YZ inflatedBoundsYZ);

	// Compute bounds
	{
		PX_ALIGN(16, PxVec4) boxMin;
		PX_ALIGN(16, PxVec4) boxMax;
		const BoundsIndex actorHandle = mActorHandle;
		const PxBounds3& b = bounds[actorHandle];
		const Vec4V offsetV = V4Load(contactDistances[actorHandle]);
		const Vec4V minimumV = V4Sub(V4LoadU(&b.minimum.x), offsetV);
		const Vec4V maximumV = V4Add(V4LoadU(&b.maximum.x), offsetV);
		V4StoreA(minimumV, &boxMin.x);
		V4StoreA(maximumV, &boxMax.x);
		inflatedBoundsX[0].initFromPxVec4(boxMin, boxMax);
		inflatedBoundsYZ.initFromPxVec4(boxMin, boxMax);

		for(PxU32 i=0;i<NB_SENTINELS;i++)
			inflatedBoundsX[1+i].initSentinel();
	}

	const PxU32 nb1 = 1;
	const BoundsIndex* PX_RESTRICT remap1 = &mActorHandle;
	const AABB_Xi* PX_RESTRICT boxes1_X = inflatedBoundsX;
	const AABB_YZ* PX_RESTRICT boxes1_YZ = &inflatedBoundsYZ;

	outputPair_Shared pm(&pairs, groups, lut);
	boxPruningKernel<0>(nb0, nb1, boxes0_X, boxes1_X, boxes0_YZ, boxes1_YZ, remap0, remap1, &pm);
	boxPruningKernel<1>(nb1, nb0, boxes1_X, boxes0_X, boxes1_YZ, boxes0_YZ, remap1, remap0, &pm);

	}
}

bool PersistentActorAggregatePair::update(AABBManager& manager, BpCacheData* data)
{
	if(mShouldBeDeleted || shouldPairBeDeleted(manager.mGroups, mAggregateHandle, mActorHandle))
		return true;

	if(!mAggregate->getNbAggregated())	// PT: needed with lazy empty actors
		return true;

	if(mAggregate->isDirty() || manager.mChangedHandleMap.boundedTest(mActorHandle))
		manager.updatePairs(*this, data);

	return false;
}

/////

class PersistentAggregateAggregatePair : public PersistentPairs
{
	public:
								PersistentAggregateAggregatePair(Aggregate* aggregate0, Aggregate* aggregate1);
	virtual						~PersistentAggregateAggregatePair()	{}

	virtual			void		findOverlaps(MBP_PairManager& pairs, const PxBounds3* PX_RESTRICT bounds, const float* PX_RESTRICT contactDistances, const Bp::FilterGroup::Enum* PX_RESTRICT groups, const bool* PX_RESTRICT lut);
	virtual			bool		update(AABBManager& manager, BpCacheData*);

					ShapeHandle	mAggregateHandle0;
					ShapeHandle	mAggregateHandle1;
					Aggregate*	mAggregate0;
					Aggregate*	mAggregate1;
};

PersistentAggregateAggregatePair::PersistentAggregateAggregatePair(Aggregate* aggregate0, Aggregate* aggregate1) :
	mAggregateHandle0	(aggregate0->mIndex),
	mAggregateHandle1	(aggregate1->mIndex),
	mAggregate0			(aggregate0),
	mAggregate1			(aggregate1)
{
}

void PersistentAggregateAggregatePair::findOverlaps(MBP_PairManager& pairs, const PxBounds3* PX_RESTRICT /*bounds*/, const float* PX_RESTRICT /*contactDistances*/, const Bp::FilterGroup::Enum* PX_RESTRICT groups, const bool* PX_RESTRICT lut)
{
	mAggregate0->getSortedMinBounds();
	mAggregate1->getSortedMinBounds();
	doBipartiteBoxPruning_Leaf(&pairs, lut, mAggregate0, mAggregate1, groups);
}

bool PersistentAggregateAggregatePair::update(AABBManager& manager, BpCacheData* data)
{
	if(mShouldBeDeleted || shouldPairBeDeleted(manager.mGroups, mAggregateHandle0, mAggregateHandle1))
		return true;

	if(!mAggregate0->getNbAggregated() || !mAggregate1->getNbAggregated())	// PT: needed with lazy empty actors
		return true;

	if(mAggregate0->isDirty() || mAggregate1->isDirty())
		manager.updatePairs(*this, data);

	return false;
}

/////

class PersistentSelfCollisionPairs : public PersistentPairs
{
	public:
						PersistentSelfCollisionPairs(Aggregate* aggregate);
	virtual				~PersistentSelfCollisionPairs()	{}

	virtual	void		findOverlaps(MBP_PairManager& pairs, const PxBounds3* PX_RESTRICT bounds, const float* PX_RESTRICT contactDistances, const Bp::FilterGroup::Enum* PX_RESTRICT groups, const bool* PX_RESTRICT lut);

			Aggregate*	mAggregate;
};

PersistentSelfCollisionPairs::PersistentSelfCollisionPairs(Aggregate* aggregate) :
	mAggregate	(aggregate)
{
}

void PersistentSelfCollisionPairs::findOverlaps(MBP_PairManager& pairs, const PxBounds3* PX_RESTRICT/*bounds*/, const float* PX_RESTRICT/*contactDistances*/, const Bp::FilterGroup::Enum* PX_RESTRICT groups, const bool* PX_RESTRICT lut)
{
	mAggregate->getSortedMinBounds();
	const PxU32 nb = mAggregate->getNbAggregated();
	const AABB_Xi* PX_RESTRICT boxes_X = mAggregate->getBoundsX();
	const AABB_YZ* PX_RESTRICT boxes_YZ = mAggregate->getBoundsYZ();

	outputPair_Shared pm(&pairs, groups, lut);
	doCompleteBoxPruning_Leaf(&pm, nb, boxes_X, boxes_YZ, mAggregate->getIndices());
}

/////

Aggregate::Aggregate(BoundsIndex index, PxAggregateFilterHint filterHint) :
	mInflatedBoundsX	(NULL),
	mInflatedBoundsYZ	(NULL),
	mIndex				(index),
	mAllocatedSize		(0),
	mFilterHint			(PxU8(filterHint)),
	mDirtySort			(false)
#if BOUNDS_DOUBLE_BUFFERING
	, mFlip				(false)
#endif
{
	PX_ASSERT(!(filterHint & 0xffffff00));	// PT: we are storing this in a byte
	resetDirtyState();
	const PxU32 selfCollisions = PxGetAggregateSelfCollisionBit(filterHint);
	mSelfCollisionPairs = selfCollisions ? PX_NEW(PersistentSelfCollisionPairs)(this) : NULL;

	//printf("%d\n", sizeof(Aggregate));	// 96 => 80
}

Aggregate::~Aggregate()
{
	PX_FREE(mInflatedBoundsYZ);
	PX_FREE(mInflatedBoundsX);

	PX_DELETE(mSelfCollisionPairs);
}

void Aggregate::sortBounds()
{
	mDirtySort = false;
	const PxU32 nbObjects = getNbAggregated();
	if(nbObjects<2)
		return;

	AABB_Xi* PX_RESTRICT boundsX = const_cast<AABB_Xi*>(getBoundsX());
	AABB_YZ* PX_RESTRICT boundsYZ = const_cast<AABB_YZ*>(getBoundsYZ());

	PX_ALLOCA(minPosBounds, InflatedType, nbObjects+1);
	bool alreadySorted =  true;
	InflatedType previousB = boundsX[0].mMinX;
	minPosBounds[0] = previousB;
	for(PxU32 i=1;i<nbObjects;i++)
	{
		const InflatedType minB = boundsX[i].mMinX;
		if(minB<previousB)
			alreadySorted = false;
		previousB = minB;
		minPosBounds[i] = minB;
	}
	if(alreadySorted)
		return;

	{
	Cm::RadixSortBuffered mRS;

	minPosBounds[nbObjects] = 0xffffffff;
	mRS.Sort(minPosBounds, nbObjects+1, RADIX_UNSIGNED);

	if(0)
	{
		PxArray<PxU32> copy = mAggregated;
		AABB_Xi* boundsXCopy = PX_ALLOCATE(AABB_Xi, nbObjects, "mInflatedBounds");
		AABB_YZ* boundsYZCopy = PX_ALLOCATE(AABB_YZ, nbObjects, "mInflatedBounds");
		PxMemCopy(boundsXCopy, boundsX, nbObjects*sizeof(AABB_Xi));
		PxMemCopy(boundsYZCopy, boundsYZ, nbObjects*sizeof(AABB_YZ));
		const PxU32* Sorted = mRS.GetRanks();
		for(PxU32 i=0;i<nbObjects;i++)
		{
			const PxU32 sortedIndex = Sorted[i];
			mAggregated[i] = copy[sortedIndex];
			boundsX[i] = boundsXCopy[sortedIndex];
			boundsYZ[i] = boundsYZCopy[sortedIndex];
		}
		PX_FREE(boundsYZCopy);
		PX_FREE(boundsXCopy);
	}
	else
	{
		PxArray<PxU32> copy = mAggregated;	// PT: TODO: revisit this, avoid the copy like we do for the other buffers
		AABB_Xi* sortedBoundsX = PX_ALLOCATE(AABB_Xi, (nbObjects+NB_SENTINELS), "mInflatedBounds");
		AABB_YZ* sortedBoundsYZ = PX_ALLOCATE(AABB_YZ, (nbObjects), "mInflatedBounds");

		const PxU32* Sorted = mRS.GetRanks();
		for(PxU32 i=0;i<nbObjects;i++)
		{
			const PxU32 sortedIndex = Sorted[i];
			mAggregated[i] = copy[sortedIndex];
			sortedBoundsX[i] = boundsX[sortedIndex];
			sortedBoundsYZ[i] = boundsYZ[sortedIndex];
		}
		for(PxU32 i=0;i<NB_SENTINELS;i++)
			sortedBoundsX[nbObjects+i].initSentinel();
		mAllocatedSize = nbObjects;
		PX_FREE(boundsYZ);
		PX_FREE(boundsX);
		mInflatedBoundsX = sortedBoundsX;
		mInflatedBoundsYZ = sortedBoundsYZ;
	}
	}
}

void Aggregate::allocateBounds()
{
	const PxU32 size = getNbAggregated();
	if(size != mAllocatedSize)
	{
		mAllocatedSize = size;
		PX_FREE(mInflatedBoundsYZ);
		PX_FREE(mInflatedBoundsX);
		const PxU32 sizeNeededX = size + NB_SENTINELS;
#if BOUNDS_DOUBLE_BUFFERING
		mInflatedBoundsX = PX_ALLOCATE(AABB_Xi, sizeNeededX * 2, "mInflatedBoundsX");
		mInflatedBoundsYZ = PX_ALLOCATE(AABB_YZ, size * 2, "mInflatedBoundsYZ");

		for(PxU32 i=0;i<NB_SENTINELS;i++)
		{
			mInflatedBoundsX[size+i].initSentinel();
			mInflatedBoundsX[sizeNeededX+size+i].initSentinel();
		}
#else
		mInflatedBoundsX = PX_ALLOCATE(AABB_Xi, sizeNeededX, "mInflatedBoundsX");
		mInflatedBoundsYZ = PX_ALLOCATE(AABB_YZ, size, "mInflatedBoundsYZ");

		for(PxU32 i=0;i<NB_SENTINELS;i++)
			mInflatedBoundsX[size+i].initSentinel();
#endif
	}
}

void Aggregate::computeBounds(const PxBounds3* PX_RESTRICT bounds, const float* PX_RESTRICT contactDistances) /*PX_RESTRICT*/
{
//	PX_PROFILE_ZONE("Aggregate::computeBounds",0);

	const PxU32 size = getNbAggregated();
	PX_ASSERT(size);

	AABB_Xi* PX_RESTRICT boundsX = const_cast<AABB_Xi*>(getBoundsX());
	AABB_YZ* PX_RESTRICT boundsYZ = const_cast<AABB_YZ*>(getBoundsYZ());

	// PT: TODO: delay the conversion to integers until we sort (i.e. really need) the aggregated bounds?
	PX_ALIGN(16, PxVec4) boxMin;
	PX_ALIGN(16, PxVec4) boxMax;

	const PxU32 lookAhead = 4;
	Vec4V minimumV;
	Vec4V maximumV;
	{
		const BoundsIndex index0 = getAggregated(0);
		const PxU32 last = PxMin(lookAhead, size-1);
		for(PxU32 i=1;i<=last;i++)
		{
			const BoundsIndex index = getAggregated(i);
			PxPrefetchLine(bounds + index, 0);
			PxPrefetchLine(contactDistances + index, 0);
		}
		const PxBounds3& b = bounds[index0];
		const Vec4V offsetV = V4Load(contactDistances[index0]);
		minimumV = V4Sub(V4LoadU(&b.minimum.x), offsetV);
		maximumV = V4Add(V4LoadU(&b.maximum.x), offsetV);
		V4StoreA(minimumV, &boxMin.x);
		V4StoreA(maximumV, &boxMax.x);
		boundsX[0].initFromPxVec4(boxMin, boxMax);
		boundsYZ[0].initFromPxVec4(boxMin, boxMax);
	}

	for(PxU32 i=1;i<size;i++)
	{
		const BoundsIndex index = getAggregated(i);
		if(i+lookAhead<size)
		{
			const BoundsIndex nextIndex = getAggregated(i+lookAhead);
			PxPrefetchLine(bounds + nextIndex, 0);
			PxPrefetchLine(contactDistances + nextIndex, 0);
		}
		const PxBounds3& b = bounds[index];
		const Vec4V offsetV = V4Load(contactDistances[index]);
		const Vec4V aggregatedBoundsMinV = V4Sub(V4LoadU(&b.minimum.x), offsetV);
		const Vec4V aggregatedBoundsMaxV = V4Add(V4LoadU(&b.maximum.x), offsetV);
		minimumV = V4Min(minimumV, aggregatedBoundsMinV);
		maximumV = V4Max(maximumV, aggregatedBoundsMaxV);
		V4StoreA(aggregatedBoundsMinV, &boxMin.x);
		V4StoreA(aggregatedBoundsMaxV, &boxMax.x);
		boundsX[i].initFromPxVec4(boxMin, boxMax);
		boundsYZ[i].initFromPxVec4(boxMin, boxMax);
	}

	StoreBounds(mBounds, minimumV, maximumV);
//	StoreBounds(boundsArray.begin()[mIndex], minimumV, maximumV);
//	boundsArray.setChangedState();

/*	if(0)
	{
		const PxBounds3& previousBounds = boundsArray.getBounds(mIndex);
		if(previousBounds.minimum==aggregateBounds.minimum && previousBounds.maximum==aggregateBounds.maximum)
		{
			// PT: same bounds as before
			printf("SAME BOUNDS\n");
		}
	}*/
	mDirtySort = true;
}

/////

static void buildFreeBitmap(PxBitMap& bitmap, PxU32 currentFree, const PxArray<Aggregate*>& aggregates)
{
	const PxU32 N = aggregates.size();

	bitmap.resizeAndClear(N);

	while(currentFree!=PX_INVALID_U32)
	{
		bitmap.set(currentFree);
		currentFree = PxU32(size_t(aggregates[currentFree]));
	}
}

#if PX_VC
#pragma warning(disable: 4355 )	// "this" used in base member initializer list
#endif

AABBManager::AABBManager(	BroadPhase& bp, BoundsArray& boundsArray, PinnableArray<PxReal>& contactDistance,
							PxU32 maxNbAggregates, PxU32 maxNbShapes, VirtualAllocatorCallback& allocator, PxU64 contextID,
							PxPairFilteringMode::Enum kineKineFilteringMode, PxPairFilteringMode::Enum staticKineFilteringMode) :
	AABBManagerBase			(bp, boundsArray, contactDistance, maxNbAggregates, maxNbShapes, allocator, contextID, kineKineFilteringMode, staticKineFilteringMode),
	mPostBroadPhase2		(contextID, *this),
	mPostBroadPhase3		(contextID, this, "AABBManager::postBroadPhaseStage3"),
	mPreBpUpdateTask		(contextID),
	mTimestamp				(0),
	mFirstFreeAggregate		(PX_INVALID_U32),
	mOutOfBoundsObjects		("AABBManager::mOutOfBoundsObjects"),
	mOutOfBoundsAggregates	("AABBManager::mOutOfBoundsAggregates")
{
}

static void releasePairs(AggPairMap& map)
{
	for(AggPairMap::Iterator iter = map.getIterator(); !iter.done(); ++iter)
		PX_DELETE(iter->second);
}

void AABBManager::destroy()
{
	releasePairs(mActorAggregatePairs);
	releasePairs(mAggregateAggregatePairs);

	{
		PxBitMap bitmap;
		buildFreeBitmap(bitmap, mFirstFreeAggregate, mAggregates);

		const PxU32 nb = mAggregates.size();
		for(PxU32 i=0;i<nb;i++)
		{
			if(bitmap.test(i))
				continue;

			Aggregate* a = mAggregates[i];
			PX_DELETE(a);
		}
	}

	BpCacheData* entry = static_cast<BpCacheData*>(mBpThreadContextPool.pop());
	while (entry)
	{
		entry->~BpCacheData();
		PX_FREE(entry);
		entry = static_cast<BpCacheData*>(mBpThreadContextPool.pop());
	}

	PX_DELETE_THIS;
}

static void removeAggregateFromDirtyArray(Aggregate* aggregate, PxArray<Aggregate*>& dirtyAggregates)
{
	// PT: TODO: do this lazily like for interactions?
	if(aggregate->isDirty())
	{
		const PxU32 dirtyIndex = aggregate->mDirtyIndex;
		PX_ASSERT(dirtyAggregates[dirtyIndex]==aggregate);
		dirtyAggregates.replaceWithLast(dirtyIndex);
		if(dirtyIndex<dirtyAggregates.size())
			dirtyAggregates[dirtyIndex]->mDirtyIndex = dirtyIndex;
		aggregate->resetDirtyState();
	}
	else
	{
		PX_ASSERT(!dirtyAggregates.findAndReplaceWithLast(aggregate));
	}
}

// PT: userData = Sc::ElementSim
bool AABBManager::addBounds(BoundsIndex index, PxReal contactDistance, Bp::FilterGroup::Enum group, void* userData, AggregateHandle aggregateHandle, ElementType::Enum volumeType, PxU32 envID)
{
	if(envID!=PX_INVALID_U32)
	{
		envID = PX_INVALID_U32;
		PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "AABBManager::addBounds - environment ID is not supported in CPU broadphases\n");
	}

//	PX_ASSERT(checkID(index));

	initEntry(index, contactDistance, group, userData, volumeType);

	if(aggregateHandle==PX_INVALID_U32)
	{
		mVolumeData[index].setSingleActor();

		addBPEntry(index);
	}
	else
	{
#if PX_CHECKED
		if(aggregateHandle>=mAggregates.size())
			return PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "AABBManager::addBounds - aggregateId out of bounds\n");

/*		{
			PxU32 firstFreeAggregate = mFirstFreeAggregate;
			while(firstFreeAggregate!=PX_INVALID_U32)
			{
				if(firstFreeAggregate==aggregateHandle)
				{
					PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "AABBManager::destroyAggregate - aggregate has already been removed\n");
					return BP_INVALID_BP_HANDLE;
				}
				firstFreeAggregate = PxU32(size_t(mAggregates[firstFreeAggregate]));
			}
		}*/
#endif
		mVolumeData[index].setAggregated(aggregateHandle);

		Aggregate* aggregate = getAggregateFromHandle(aggregateHandle);

		{
			// PT: schedule the aggregate for BP insertion here, if we just added its first shape
			if(!aggregate->getNbAggregated())
				addBPEntry(aggregate->mIndex);

			aggregate->addAggregated(index);

			// PT: new actor added to aggregate => mark dirty to recompute bounds later
			aggregate->markAsDirty(mDirtyAggregates);
		}
	}

	return true;
}

bool AABBManager::removeBounds(BoundsIndex index)
{
	// PT: TODO: shouldn't it be compared to mUsedSize?
	PX_ASSERT(index < mVolumeData.size());

	bool res = false;
	if(mVolumeData[index].isSingleActor())
	{
		res = removeBPEntry(index);
	}
	else
	{
		PX_ASSERT(mVolumeData[index].isAggregated());

		const AggregateHandle aggregateHandle = mVolumeData[index].getAggregateOwner();
		Aggregate* aggregate = getAggregateFromHandle(aggregateHandle);
		bool status = aggregate->removeAggregated(index);
		(void)status;
//		PX_ASSERT(status);	// PT: can be false when >128 shapes

		// PT: remove empty aggregates, otherwise the BP will crash with empty bounds
		if(!aggregate->getNbAggregated())
		{
			removeBPEntry(aggregate->mIndex);
			removeAggregateFromDirtyArray(aggregate, mDirtyAggregates);
		}
		else
			aggregate->markAsDirty(mDirtyAggregates);	// PT: actor removed from aggregate => mark dirty to recompute bounds later
	}

	resetEntry(index);
	return res;
}

// PT: TODO: the userData is actually a PxAggregate pointer. Maybe we could expose/use that.
AggregateHandle AABBManager::createAggregate(BoundsIndex index, Bp::FilterGroup::Enum group, void* userData, PxU32 /*maxNumShapes*/, PxAggregateFilterHint filterHint, PxU32 envID)
{
//	PX_ASSERT(checkID(index));

	if(envID!=PX_INVALID_U32)
	{
		envID = PX_INVALID_U32;
		PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "AABBManager::createAggregate - environment ID is not supported in CPU broadphases\n");
	}

	Aggregate* aggregate = PX_NEW(Aggregate)(index, filterHint);

	AggregateHandle handle;
	if(mFirstFreeAggregate==PX_INVALID_U32)
	{
		handle = mAggregates.size();
		mAggregates.pushBack(aggregate);
	}
	else
	{
		handle = mFirstFreeAggregate;
		mFirstFreeAggregate = PxU32(size_t(mAggregates[mFirstFreeAggregate]));
		mAggregates[handle] = aggregate;
	}

#if BP_USE_AGGREGATE_GROUP_TAIL
/*		PxU32 id = index;
		id<<=2;
		id|=FilterType::AGGREGATE;
		initEntry(index, 0.0f, Bp::FilterGroup::Enum(id), userData);
*/
	initEntry(index, 0.0f, getAggregateGroup(), userData);
	PX_UNUSED(group);
#else
	initEntry(index, 0.0f, group, userData);
#endif

	mVolumeData[index].setAggregate(handle);

	//creates an extra empty bound explicitly. Corresponding entry in the transform cache is guaranteed by resizing and advancing the index for next transforms, although explicitly setTransform is not called
	//bounds and transforms are in sync and this change is reflected in changes array for GPU transfer
	mBoundsArray.setBounds(PxBounds3::empty(), index);

	mNbAggregates++;

	// PT: we don't add empty aggregates to mAddedHandleMap yet, since they make the BP crash.
	return handle;
}

bool AABBManager::destroyAggregate(BoundsIndex& index_, Bp::FilterGroup::Enum& group_, AggregateHandle aggregateHandle)
{
#if PX_CHECKED
	if(aggregateHandle>=mAggregates.size())
		return PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "AABBManager::destroyAggregate - aggregateId out of bounds\n");

	{
		PxU32 firstFreeAggregate = mFirstFreeAggregate;
		while(firstFreeAggregate!=PX_INVALID_U32)
		{
			if(firstFreeAggregate==aggregateHandle)
				return PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "AABBManager::destroyAggregate - aggregate has already been removed\n");

			firstFreeAggregate = PxU32(size_t(mAggregates[firstFreeAggregate]));
		}
	}
#endif

	Aggregate* aggregate = getAggregateFromHandle(aggregateHandle);

#if PX_CHECKED
	if(aggregate->getNbAggregated())
		return PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "AABBManager::destroyAggregate - aggregate still has bounds that needs removed\n");
#endif

	const BoundsIndex index = aggregate->mIndex;
	removeAggregateFromDirtyArray(aggregate, mDirtyAggregates);

	if(mAddedHandleMap.test(index))			// PT: if object had been added this frame...
		mAddedHandleMap.reset(index);		// PT: ...then simply revert the previous operation locally (it hasn't been passed to the BP yet).
	else if(aggregate->getNbAggregated())	// PT: else we need to remove it from the BP if it has been added there. If there's no aggregated
		mRemovedHandleMap.set(index);		// PT: shapes then the aggregate has never been added, or already removed.

	PX_DELETE(aggregate);
	mAggregates[aggregateHandle] = reinterpret_cast<Aggregate*>(size_t(mFirstFreeAggregate));
	mFirstFreeAggregate = aggregateHandle;

	// PT: TODO: shouldn't it be compared to mUsedSize?
	PX_ASSERT(index < mVolumeData.size());

	index_ = index;
	group_ = mGroups[index];

#if BP_USE_AGGREGATE_GROUP_TAIL
	releaseAggregateGroup(mGroups[index]);
#endif
	resetEntry(index);

	PX_ASSERT(mNbAggregates);
	mNbAggregates--;

	return true;
}

void AABBManager::handleOriginShift()
{
	mOriginShifted = false;

	// PT: TODO: isn't the following loop potentially updating removed objects?
	// PT: TODO: check that aggregates code is correct here
	for(PxU32 i=0; i<mUsedSize; i++)
	{
		if(mGroups[i] == Bp::FilterGroup::eINVALID)
			continue;

		{
			if(mVolumeData[i].isSingleActor())
			{
				if(!mAddedHandleMap.test(i))
					mUpdatedHandles.pushBack(i);	// PT: TODO: BoundsIndex-to-ShapeHandle confusion here
			}
			else if(mVolumeData[i].isAggregate())
			{
				const AggregateHandle aggregateHandle = mVolumeData[i].getAggregate();
				Aggregate* aggregate = getAggregateFromHandle(aggregateHandle);
				if(aggregate->getNbAggregated())
				{
					aggregate->markAsDirty(mDirtyAggregates);
					aggregate->allocateBounds();
					aggregate->computeBounds(mBoundsArray.begin(), mContactDistance.begin());
					mBoundsArray.begin()[aggregate->mIndex] = aggregate->getMergedBounds();
					if(!mAddedHandleMap.test(i))
						mUpdatedHandles.pushBack(i);	// PT: TODO: BoundsIndex-to-ShapeHandle confusion here
				}
			}
		}
	}
}

void AggregateBoundsComputationTask::runInternal()
{
	const BoundsArray& boundArray = mManager->getBoundsArray();
	const float* contactDistances = mManager->getContactDistances();
	const PxBounds3* bounds = boundArray.begin();

	PxU32 size = mNbToGo;
	Aggregate** currentAggregate = mAggregates + mStart;
	while(size--)
	{
		if(size)
		{
			Aggregate* nextAggregate = *(currentAggregate+1);
			PxPrefetchLine(nextAggregate, 0);
			PxPrefetchLine(nextAggregate, 64);
		}

		(*currentAggregate)->computeBounds(bounds, contactDistances);
		currentAggregate++;
	}
}

void PreBpUpdateTask::runInternal()
{
	mManager->preBpUpdate_CPU(mNumCpuTasks);
}

void AABBManager::startAggregateBoundsComputationTasks(PxU32 nbToGo, PxU32 numCpuTasks, Cm::FlushPool& flushPool)
{
	const PxU32 nbAggregatesPerTask = nbToGo > numCpuTasks ? nbToGo / numCpuTasks : nbToGo;

	// PT: TODO: better load balancing

	PxU32 start = 0;
	while(nbToGo)
	{
		AggregateBoundsComputationTask* T = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(AggregateBoundsComputationTask)), AggregateBoundsComputationTask(mContextID));

		const PxU32 nb = nbToGo < nbAggregatesPerTask ? nbToGo : nbAggregatesPerTask;
		T->Init(this, start, nb, mDirtyAggregates.begin());
		start += nb;
		nbToGo -= nb;

		T->setContinuation(&mPreBpUpdateTask);
		T->removeReference();
	}
}

void AABBManager::updateBPFirstPass(PxU32 numCpuTasks, Cm::FlushPool& flushPool, bool /*hasContactDistanceUpdated*/, PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("AABBManager::updateBPFirstPass", mContextID);

	const bool singleThreaded = gSingleThreaded || numCpuTasks<2;
	if(!singleThreaded)
	{
		PX_ASSERT(numCpuTasks);
		mPreBpUpdateTask.Init(this, numCpuTasks);
		mPreBpUpdateTask.setContinuation(continuation);
	}

	// Add
	{
		PX_PROFILE_ZONE("AABBManager::updateBPFirstPass - add", mContextID);

		mAddedHandles.resetOrClear();

		const PxU32* bits = mAddedHandleMap.getWords();
		if(bits)
		{
			// PT: ### bitmap iterator pattern
			const PxU32 lastSetBit = mAddedHandleMap.findLast();
			for(PxU32 w = 0; w <= lastSetBit >> 5; ++w)
			{
				for(PxU32 b = bits[w]; b; b &= b-1)
				{
					const BoundsIndex handle = PxU32(w<<5|PxLowestSetBit(b));
					PX_ASSERT(!mVolumeData[handle].isAggregated());
					mAddedHandles.pushBack(handle);		// PT: TODO: BoundsIndex-to-ShapeHandle confusion here
				}
			}
		}
	}

	// Update
	{
		PX_PROFILE_ZONE("AABBManager::updateBPFirstPass - update", mContextID);

		mUpdatedHandles.resetOrClear();
		if(!mOriginShifted)
		{
			// PT: TODO:
			// - intercept calls marking aggregateD shapes dirty, in order to mark their aggregates dirty at the same time. That way we don't discover
			// them while parsing the map, i.e. the map is already fully complete when the parsing begins (no need to parse twice etc).
			// - once this is done, aggregateD shapes can be ignored during parsing (since we only needed them to go to their aggregates)
			// - we can then handle aggregates while parsing the map, i.e. there's no need for sorting anymore.
			// - there will be some thoughts to do about the dirty aggregates coming from the added map parsing: we still need to compute their bounds,
			// but we don't want to add these to mUpdatedHandles (since they're already in  mAddedHandles)
			// - we still need the set of dirty aggregates post broadphase, but we don't want to re-parse the full map for self-collisions. So we may still
			// need to put dirty aggregates in an array, but that might be simplified now
			// - the 'isDirty' checks to updatePairs can use the update map though - but the boundedTest is probably more expensive than the current test

			// PT: TODO: another idea: just output all aggregate handles by default then have a pass on mUpdatedHandles to remove them if that wasn't actually necessary
			// ...or just drop the artificial requirement for aggregates...

			{
				PX_PROFILE_ZONE("AABBManager::updateBPFirstPass - update - bitmap iteration", mContextID);

				const PxU32* bits = mChangedHandleMap.getWords();
				if(bits)
				{
					// PT: ### bitmap iterator pattern
					const PxU32 lastSetBit = mChangedHandleMap.findLast();
					for(PxU32 w = 0; w <= lastSetBit >> 5; ++w)
					{
						for(PxU32 b = bits[w]; b; b &= b-1)
						{
							const BoundsIndex handle = PxU32(w<<5|PxLowestSetBit(b));
							PX_ASSERT(!mRemovedHandleMap.test(handle));		// a handle may only be updated and deleted if it was just added.
							PX_ASSERT(!mVolumeData[handle].isAggregate());	// PT: make sure changedShapes doesn't contain aggregates

							if(mAddedHandleMap.test(handle))					// just-inserted handles may also be marked updated, so skip them
								continue;

							if(mVolumeData[handle].isSingleActor())
							{
								PX_ASSERT(mGroups[handle] != Bp::FilterGroup::eINVALID);
								mUpdatedHandles.pushBack(handle);	// PT: TODO: BoundsIndex-to-ShapeHandle confusion here
							}
							else
							{
								PX_ASSERT(mVolumeData[handle].isAggregated());
								const AggregateHandle aggregateHandle = mVolumeData[handle].getAggregateOwner();
								Aggregate* aggregate = getAggregateFromHandle(aggregateHandle);
								// PT: an actor from the aggregate has been updated => mark dirty to recompute bounds later
								// PT: we don't recompute the bounds right away since multiple actors from the same aggregate may have changed.
								aggregate->markAsDirty(mDirtyAggregates);
							}
						}
					}
				}
			}

			const PxU32 size = mDirtyAggregates.size();
			if(size)
			{
				PX_PROFILE_ZONE("AABBManager::updateBPFirstPass - update - dirty iteration", mContextID);
				for(PxU32 i=0;i<size;i++)
				{
					Aggregate* aggregate = mDirtyAggregates[i];
					if(i!=size-1)
					{
						Aggregate* nextAggregate = mDirtyAggregates[i];
						PxPrefetchLine(nextAggregate, 0);
					}

					aggregate->allocateBounds();
					if(singleThreaded)
					{
						aggregate->computeBounds(mBoundsArray.begin(), mContactDistance.begin());
						mBoundsArray.begin()[aggregate->mIndex] = aggregate->getMergedBounds();
					}

					// PT: Can happen when an aggregate has been created and then its actors have been changed (with e.g. setLocalPose)
					// before a BP call.
					if(!mAddedHandleMap.test(aggregate->mIndex))
						mUpdatedHandles.pushBack(aggregate->mIndex);	// PT: TODO: BoundsIndex-to-ShapeHandle confusion here
				}

				if(!singleThreaded)
					startAggregateBoundsComputationTasks(size, numCpuTasks, flushPool);

				// PT: we're already sorted if no dirty-aggregates are involved
				{
					PX_PROFILE_ZONE("AABBManager::updateAABBsAndBP - update - sort", mContextID);

					// PT: TODO: remove this
					PxSort(mUpdatedHandles.begin(), mUpdatedHandles.size());
				}
			}
		}
		else
		{
			handleOriginShift();
		}
	}

	// Remove
	{
		PX_PROFILE_ZONE("AABBManager::updateBPFirstPass - remove", mContextID);

		mRemovedHandles.resetOrClear();

		const PxU32* bits = mRemovedHandleMap.getWords();
		if(bits)
		{
			// PT: ### bitmap iterator pattern
			const PxU32 lastSetBit = mRemovedHandleMap.findLast();
			for(PxU32 w = 0; w <= lastSetBit >> 5; ++w)
			{
				for(PxU32 b = bits[w]; b; b &= b-1)
				{
					const BoundsIndex handle = PxU32(w<<5|PxLowestSetBit(b));
					PX_ASSERT(!mVolumeData[handle].isAggregated());
					mRemovedHandles.pushBack(handle);	// PT: TODO: BoundsIndex-to-ShapeHandle confusion here
				}
			}
		}
	}

	/////

	// PT: TODO: do we need to run these threads when we origin-shifted everything before?
	if(singleThreaded)
		preBpUpdate_CPU(numCpuTasks);
	else
		mPreBpUpdateTask.removeReference();
}

// PT: previously known as AABBManager::updateAABBsAndBP
void AABBManager::updateBPSecondPass(PxcScratchAllocator* scratchAllocator, PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("AABBManager::updateBPSecondPass", mContextID);

	// PT: TODO: do we need to run these threads when we origin-shifted everything before?
	//finalizeUpdate(numCpuTasks, scratchAllocator, continuation);
	// PT: code below used to be "finalizeUpdate"

	// PT: TODO: move to base?
	const BroadPhaseUpdateData updateData(mAddedHandles.begin(), mAddedHandles.size(),
		mUpdatedHandles.begin(), mUpdatedHandles.size(),
		mRemovedHandles.begin(), mRemovedHandles.size(),
		mBoundsArray.begin(), mGroups.begin(), mContactDistance.begin(), mBoundsArray.size(),
		mFilters,
		// PT: TODO: this could also be removed now. The key to understanding the refactorings is that none of the two bools below are actualy used by the CPU versions.
		mBoundsArray.hasChanged(),
		false);

	PX_ASSERT(updateData.isValid(mContextID) == BroadPhaseUpdateError::eNO_ERROR);

	const bool b = updateData.getNumCreatedHandles() || updateData.getNumRemovedHandles();

	//KS - skip broad phase if there are no updated shapes.
	// PT: BP UPDATE CALL
	if(b || updateData.getNumUpdatedHandles())
		mBroadPhase.update(scratchAllocator, updateData, continuation);
}

void AABBManager::preBpUpdate_CPU(PxU32 numCpuTasks)
{
	PX_PROFILE_ZONE("AABBManager::preBpUpdate", mContextID);

	const bool singleThreaded = gSingleThreaded || numCpuTasks<2;
	if (!singleThreaded)
	{
		const PxU32 size = mDirtyAggregates.size();
		for (PxU32 i = 0; i<size; i++)
		{
			Aggregate* aggregate = mDirtyAggregates[i];
			mBoundsArray.begin()[aggregate->mIndex] = aggregate->getMergedBounds();
		}
	}
}

static PX_FORCE_INLINE void outputOverlap(PxArray<AABBOverlap>* overlaps, const VolumeData* volumeData, PxU32 id0, PxU32 id1)
{
//	overlaps.pushBack(AABBOverlap(volumeData[id0].userData, volumeData[id1].userData, handle));
	const ElementType::Enum volumeType = PxMax(volumeData[id0].getVolumeType(), volumeData[id1].getVolumeType());
	//overlaps[volumeType].pushBack(AABBOverlap(reinterpret_cast<void*>(size_t(id0)), reinterpret_cast<void*>(size_t(id1))));

	AABBOverlap* overlap = Cm::reserveContainerMemory(overlaps[volumeType], 1);
	// PT: we don't convert to pointers right away because we need the IDs in postBpStage3
	overlap->mUserData0 = reinterpret_cast<void*>(size_t(id0));
	overlap->mUserData1 = reinterpret_cast<void*>(size_t(id1));
	// PT: note that overlap->mPairUserData remains uninitialized here
}

static PX_FORCE_INLINE void createOverlap(PxArray<AABBOverlap>* overlaps, const VolumeData* volumeData, PxU32 id0, PxU32 id1)
{
	outputOverlap(overlaps, volumeData, id0, id1);
}

static PX_FORCE_INLINE void deleteOverlap(PxArray<AABBOverlap>* overlaps, const VolumeData* volumeData, PxU32 id0, PxU32 id1)
{
//	PX_ASSERT(volumeData[id0].getUserData());
//	PX_ASSERT(volumeData[id1].getUserData());
	if (volumeData[id0].getUserData() && volumeData[id1].getUserData())	// PT: TODO: no idea if this is the right thing to do or if it's normal to get null ptrs here
		outputOverlap(overlaps, volumeData, id0, id1);
}

void PersistentPairs::outputDeletedOverlaps(PxArray<AABBOverlap>* overlaps, const VolumeData* volumeData)
{
	const PxU32 nbActivePairs = mPM.mNbActivePairs;
	for(PxU32 i=0;i<nbActivePairs;i++)
	{
		const InternalPair& p = mPM.mActivePairs[i];
		deleteOverlap(overlaps, volumeData, p.getId0(), p.getId1());
	}
}

PX_FORCE_INLINE void PersistentPairs::updatePairs(	PxU32 timestamp, const PxBounds3* bounds, const float* contactDistances, const Bp::FilterGroup::Enum* groups,
													const bool* lut, VolumeData* volumeData, PxArray<AABBOverlap>* createdOverlaps, PxArray<AABBOverlap>* destroyedOverlaps)
{
	if(mTimestamp==timestamp)
		return;

	mTimestamp = timestamp;

	findOverlaps(mPM, bounds, contactDistances, groups, lut);

	PxU32 i=0;
	PxU32 nbActivePairs = mPM.mNbActivePairs;
	while(i<nbActivePairs)
	{
		InternalPair& p = mPM.mActivePairs[i];
		const PxU32 id0 = p.getId0();
		const PxU32 id1 = p.getId1();

		if(p.isNew())
		{
			createOverlap(createdOverlaps, volumeData, id0, id1);

			p.clearNew();
			p.clearUpdated();

			i++;
		}
		else if(p.isUpdated())
		{
			p.clearUpdated();
			i++;
		}
		else
		{
			deleteOverlap(destroyedOverlaps, volumeData, id0, id1);

			const PxU32 hashValue = hash(id0, id1) & mPM.mMask;
			mPM.removePair(id0, id1, hashValue, i);
			nbActivePairs--;
		}
	}
	mPM.shrinkMemory();
}

void AABBManager::updatePairs(PersistentPairs& p, BpCacheData* data)
{
	if (data)
		p.updatePairs(mTimestamp, mBoundsArray.begin(), mContactDistance.begin(), mGroups.begin(), mFilters.getLUT(), mVolumeData.begin(), data->mCreatedPairs, data->mDeletedPairs);
	else
		p.updatePairs(mTimestamp, mBoundsArray.begin(), mContactDistance.begin(), mGroups.begin(), mFilters.getLUT(), mVolumeData.begin(), mCreatedOverlaps, mDestroyedOverlaps);
}

static PX_FORCE_INLINE Bp::FilterType::Enum convertFilterType(PxAggregateType::Enum	agType)
{
	if(agType==PxAggregateType::eGENERIC)
		return Bp::FilterType::DYNAMIC;
	else if(agType==PxAggregateType::eSTATIC)
		return Bp::FilterType::STATIC;
	PX_ASSERT(agType==PxAggregateType::eKINEMATIC);
	return Bp::FilterType::KINEMATIC;
}

PersistentActorAggregatePair* AABBManager::createPersistentActorAggregatePair(ShapeHandle volA, ShapeHandle volB)
{
	ShapeHandle	actorHandle;
	ShapeHandle	aggregateHandle;
	if(mVolumeData[volA].isAggregate())
	{
		aggregateHandle = volA;
		actorHandle = volB;
	}
	else
	{
		PX_ASSERT(mVolumeData[volB].isAggregate());
		aggregateHandle = volB;
		actorHandle = volA;
	}
	const AggregateHandle h = mVolumeData[aggregateHandle].getAggregate();
	Aggregate* aggregate = getAggregateFromHandle(h);
	PX_ASSERT(aggregate->mIndex==aggregateHandle);

	// Single-aggregate filtering
	{
		const PxAggregateType::Enum	agType = PxGetAggregateType(aggregate->getFilterHint());
		const int t0 = convertFilterType(agType);

		const int t1 = mGroups[actorHandle] & BP_FILTERING_TYPE_MASK;	// PT: from "groupFiltering" function

		if(!mFilters.mLUT[t0][t1])
			return NULL;
	}

	return PX_NEW(PersistentActorAggregatePair)(aggregate, actorHandle);	// PT: TODO: use a pool or something
}

PersistentAggregateAggregatePair* AABBManager::createPersistentAggregateAggregatePair(ShapeHandle volA, ShapeHandle volB)
{
	PX_ASSERT(mVolumeData[volA].isAggregate() && mVolumeData[volB].isAggregate());
	const AggregateHandle h0 = mVolumeData[volA].getAggregate();
	const AggregateHandle h1 = mVolumeData[volB].getAggregate();
	Aggregate* aggregate0 = getAggregateFromHandle(h0);
	Aggregate* aggregate1 = getAggregateFromHandle(h1);
	PX_ASSERT(aggregate0->mIndex==volA);
	PX_ASSERT(aggregate1->mIndex==volB);

	// Aggregate-aggregate filtering
	{
		const PxAggregateType::Enum	agType0 = PxGetAggregateType(aggregate0->getFilterHint());
		const PxAggregateType::Enum	agType1 = PxGetAggregateType(aggregate1->getFilterHint());
		const Bp::FilterType::Enum t0 = convertFilterType(agType0);
		const Bp::FilterType::Enum t1 = convertFilterType(agType1);
		if(!mFilters.mLUT[t0][t1])
			return NULL;
	}

	return PX_NEW(PersistentAggregateAggregatePair)(aggregate0, aggregate1);	// PT: TODO: use a pool or something
}

void AABBManager::processBPCreatedPair(const BroadPhasePair& pair)
{
	PX_ASSERT(!mVolumeData[pair.mVolA].isAggregated());
	PX_ASSERT(!mVolumeData[pair.mVolB].isAggregated());
	const bool isSingleActorA = mVolumeData[pair.mVolA].isSingleActor();
	const bool isSingleActorB = mVolumeData[pair.mVolB].isSingleActor();

	if(isSingleActorA && isSingleActorB)
	{
		createOverlap(mCreatedOverlaps, mVolumeData.begin(), pair.mVolA, pair.mVolB);	// PT: regular actor-actor pair
		return;
	}

	// PT: TODO: check if this is needed
	ShapeHandle volA = pair.mVolA;
	ShapeHandle volB = pair.mVolB;
	if(volB<volA)
		PxSwap(volA, volB);

	PersistentPairs* newPair;
	AggPairMap* pairMap;
	if(!isSingleActorA && !isSingleActorB)
	{
		pairMap = &mAggregateAggregatePairs;	// PT: aggregate-aggregate pair
		newPair = createPersistentAggregateAggregatePair(volA, volB);
	}
	else
	{
		pairMap = &mActorAggregatePairs;	// PT: actor-aggregate pair
		newPair = createPersistentActorAggregatePair(volA, volB);
	}

	if(newPair)
	{
		bool status = pairMap->insert(AggPair(volA, volB), newPair);
		PX_UNUSED(status);
		PX_ASSERT(status);
		updatePairs(*newPair);
	}
}

void AABBManager::processBPDeletedPair(const BroadPhasePair& pair)
{
	PX_ASSERT(!mVolumeData[pair.mVolA].isAggregated());
	PX_ASSERT(!mVolumeData[pair.mVolB].isAggregated());
	const bool isSingleActorA = mVolumeData[pair.mVolA].isSingleActor();
	const bool isSingleActorB = mVolumeData[pair.mVolB].isSingleActor();

	if(isSingleActorA && isSingleActorB)
	{
		deleteOverlap(mDestroyedOverlaps, mVolumeData.begin(), pair.mVolA, pair.mVolB);	// PT: regular actor-actor pair
		return;
	}

	// PT: TODO: check if this is needed
	ShapeHandle volA = pair.mVolA;
	ShapeHandle volB = pair.mVolB;
	if(volB<volA)
		PxSwap(volA, volB);

	AggPairMap* pairMap;
	if(!isSingleActorA && !isSingleActorB)
		pairMap = &mAggregateAggregatePairs;	// PT: aggregate-aggregate pair
	else
		pairMap = &mActorAggregatePairs;		// PT: actor-aggregate pair

	const AggPairMap::Entry* e = pairMap->find(AggPair(volA, volB));
	if(e && e->second)
	{
		PersistentPairs* p = e->second;
		p->outputDeletedOverlaps(mDestroyedOverlaps, mVolumeData.begin());
		p->mShouldBeDeleted = true;
	}
}

struct CreatedPairHandler
{
	static PX_FORCE_INLINE void processPair(AABBManager& manager, const BroadPhasePair& pair) { manager.processBPCreatedPair(pair); }
};

struct DeletedPairHandler
{
	static PX_FORCE_INLINE void processPair(AABBManager& manager, const BroadPhasePair& pair) { manager.processBPDeletedPair(pair); }
};

template<class FunctionT>
static void processBPPairs(PxU32 nbPairs, const BroadPhasePair* pairs, AABBManager& manager)
{
	// PT: TODO: figure out this ShapeHandle/BpHandle thing. Is it ok to use "BP_INVALID_BP_HANDLE" for a "ShapeHandle"?
#if PX_DEBUG
	ShapeHandle previousA = BP_INVALID_BP_HANDLE;
	ShapeHandle previousB = BP_INVALID_BP_HANDLE;
#endif
	while(nbPairs--)
	{
		PX_ASSERT(pairs->mVolA!=BP_INVALID_BP_HANDLE);
		PX_ASSERT(pairs->mVolB!=BP_INVALID_BP_HANDLE);
#if PX_DEBUG
		// PT: TODO: why is that test needed now? GPU broadphase?
		PX_ASSERT(pairs->mVolA != previousA || pairs->mVolB != previousB);
#endif
		//if(pairs->mVolA != previousA || pairs->mVolB != previousB)
		{
#if PX_DEBUG
			previousA = pairs->mVolA;
			previousB = pairs->mVolB;
#endif
			FunctionT::processPair(manager, *pairs);
		}
		pairs++;
	}
}

static void processAggregatePairs(AggPairMap& map, AABBManager& manager)
{
	// PT: TODO: hmmm we have a list of dirty aggregates but we don't have a list of dirty pairs.
	// PT: not sure how the 3.4 trunk solves this but let's just iterate all pairs for now
	// PT: atm we reuse this loop to delete removed interactions
	// PT: TODO: in fact we could handle all the "lost pairs" stuff right there with extra aabb-abb tests

	// PT: TODO: replace with decent hash map - or remove the hashmap entirely and use a linear array
	PxArray<AggPair> removedEntries;
	for(AggPairMap::Iterator iter = map.getIterator(); !iter.done(); ++iter)
	{
		PersistentPairs* p = iter->second;
		if(p->update(manager))
		{
			removedEntries.pushBack(iter->first);
			PX_DELETE(p);
		}
	}
	for(PxU32 i=0;i<removedEntries.size();i++)
	{
		bool status = map.erase(removedEntries[i]);
		PX_ASSERT(status);
		PX_UNUSED(status);
	}
}

struct PairData
{
	PxArray<AABBOverlap>* mArray;
	PxU32 mStartIdx;
	PxU32 mCount;

	PairData() : mArray(NULL), mStartIdx(0), mCount(0)
	{
	}
};

class ProcessAggPairsBase : public Cm::Task
{
public:
	static const PxU32 MaxPairs = 16;

	PairData mCreatedPairs[2];
	PairData mDestroyedPairs[2];

	ProcessAggPairsBase(PxU64 contextID) : Cm::Task(contextID)
	{
	}

	void setCache(BpCacheData& data)
	{
		for (PxU32 i = 0; i < 2; ++i)
		{
			mCreatedPairs[i].mArray = &data.mCreatedPairs[i];
			mCreatedPairs[i].mStartIdx = data.mCreatedPairs[i].size();
			mDestroyedPairs[i].mArray = &data.mDeletedPairs[i];
			mDestroyedPairs[i].mStartIdx = data.mDeletedPairs[i].size();
		}
	}

	void updateCounters()
	{
		for (PxU32 i = 0; i < 2; ++i)
		{
			mCreatedPairs[i].mCount = mCreatedPairs[i].mArray->size() - mCreatedPairs[i].mStartIdx;
			mDestroyedPairs[i].mCount = mDestroyedPairs[i].mArray->size() - mDestroyedPairs[i].mStartIdx;
		}
	}
};

class ProcessAggPairsParallelTask : public ProcessAggPairsBase
{
public:
	PersistentPairs* mPersistentPairs[MaxPairs];
	Bp::AggPair mAggPairs[MaxPairs];
	PxU32 mNbPairs;
	AABBManager* mManager;
	AggPairMap* mMap;
	PxMutex* mMutex;
	const char* mName;

	ProcessAggPairsParallelTask(PxU64 contextID, PxMutex* mutex, AABBManager* manager, AggPairMap* map, const char* name) : ProcessAggPairsBase(contextID),
		mNbPairs(0), mManager(manager), mMap(map), mMutex(mutex), mName(name)
	{
	}

	void runInternal()
	{
		BpCacheData* data = mManager->getBpCacheData();

		setCache(*data);

		PxInlineArray<AggPair, MaxPairs> removedEntries;
		for (PxU32 i = 0; i < mNbPairs; ++i)
		{
			if(mPersistentPairs[i]->update(*mManager, data))
			{
				removedEntries.pushBack(mAggPairs[i]);
				PX_DELETE(mPersistentPairs[i]);
			}
		}

		updateCounters();

		mManager->putBpCacheData(data);

		if (removedEntries.size())
		{
			PxMutex::ScopedLock lock(*mMutex);
			for (PxU32 i = 0; i < removedEntries.size(); i++)
			{
				bool status = mMap->erase(removedEntries[i]);
				PX_ASSERT(status);
				PX_UNUSED(status);
			}
		}
	}

	virtual const char* getName() const { return mName; }
};

class SortAggregateBoundsParallel : public Cm::Task
{
public:
	static const PxU32 MaxPairs = 16;
	Aggregate** mAggregates;
	const PxU32 mNbAggs;

	SortAggregateBoundsParallel(PxU64 contextID, Aggregate** aggs, PxU32 nbAggs) : Cm::Task(contextID), mAggregates(aggs), mNbAggs(nbAggs)
	{
	}

	void runInternal()
	{
		PX_PROFILE_ZONE("SortBounds", mContextID);

		// PT: it appears that we now sort the bounds all the time during the broadphase, which defeats the purpose of the
		// initial lazy-evaluation approach. We still leave that code in place (Aggregate::sortBounds) to catch potential
		// edge cases but we take advantage of the current batching here (multiple sorts done at the same time in the same
		// task/thread) to reduce the number of allocations.
		// Initial code:
		//	const PxU32 nb = mNbAggs;
		//	for(PxU32 i=0; i<nb; i++)
		//		mAggregates[i]->getSortedMinBounds();

		Cm::RadixSortBuffered rs;

		// PT: we will use the stack instead of actual allocations for "small" aggregates.
		PxU32 minPosBoundsCapa = 1024;
		InflatedType stackMinPosBounds[1024];
		InflatedType* minPosBounds = stackMinPosBounds;
#if !BOUNDS_DOUBLE_BUFFERING
		PxU32 objectsCapa = 1024;
		AABB_Xi stackBoundsXCopy[1024];
		AABB_YZ stackBoundsYZCopy[1024];
		AABB_Xi* boundsXCopy = stackBoundsXCopy;
		AABB_YZ* boundsYZCopy = stackBoundsYZCopy;
#endif
		const PxU32 nb = mNbAggs;
		for(PxU32 j=0; j<nb; j++)
		{
			Aggregate* agg = mAggregates[j];
			if(!agg->mDirtySort)
				continue;
			agg->mDirtySort = false;

			const PxU32 nbObjects = agg->getNbAggregated();
			if(nbObjects<2)
				continue;

			const PxU32 nbNeeded = nbObjects;
			if(nbNeeded > minPosBoundsCapa)	// PT: reuse previous buffers if we can
			{
				minPosBoundsCapa = nbNeeded;
				minPosBounds = PX_ALLOCATE(InflatedType, nbNeeded, "keys");
			}

			{
				const AABB_Xi* inflatedBoundsX = agg->getBoundsX();

				// PT: we need to copy the minX values to a separate buffer for the radix sort's API.
				bool alreadySorted =  true;
				InflatedType previousB = inflatedBoundsX[0].mMinX;
				minPosBounds[0] = previousB;
				for(PxU32 i=1; i<nbObjects; i++)
				{
					const InflatedType minB = inflatedBoundsX[i].mMinX;
					if(minB<previousB)
						alreadySorted = false;
					previousB = minB;
					minPosBounds[i] = minB;
				}
				// PT: we could have used the already-sorted test inside the radix sort itself but that doesn't
				// work when sharing the same radix for all aggregates (we'd need to reset the ranks).
				if(alreadySorted)
					continue;
			}

			const PxU32* sorted = rs.Sort(minPosBounds, nbNeeded, RADIX_UNSIGNED).GetRanks();

			// PT: we have a choice between memcopies or allocations in general here. In this version
			// we do copies while avoiding some allocations compared to the original code (Aggregate::sortBounds).
			PxU32* copy = rs.GetRecyclable();
			PxMemCopy(copy, agg->mAggregated.begin(), nbObjects * sizeof(PxU32));

#if BOUNDS_DOUBLE_BUFFERING
			const AABB_Xi* inflatedBoundsX = agg->getBoundsX();
			const AABB_YZ* inflatedBoundsYZ = agg->getBoundsYZ();

			AABB_Xi* boundsXCopy = agg->getBoundsXCopy();
			AABB_YZ* boundsYZCopy = agg->getBoundsYZCopy();

			for(PxU32 i=0;i<nbObjects;i++)
			{
				const PxU32 sortedIndex = sorted[i];
				agg->mAggregated[i] = copy[sortedIndex];
				boundsXCopy[i] = inflatedBoundsX[sortedIndex];
				boundsYZCopy[i] = inflatedBoundsYZ[sortedIndex];
			}
			agg->mFlip = !agg->mFlip;
#else
			if(nbObjects > objectsCapa)	// PT: reuse previous buffers if we can
			{
				objectsCapa = nbObjects;
				boundsXCopy = PX_ALLOCATE(AABB_Xi, nbObjects, "mInflatedBounds");
				boundsYZCopy = PX_ALLOCATE(AABB_YZ, nbObjects, "mInflatedBounds");
			}

			AABB_Xi* inflatedBoundsX = agg->mInflatedBoundsX;
			AABB_YZ* inflatedBoundsYZ = agg->mInflatedBoundsYZ;
			PxMemCopy(boundsXCopy, inflatedBoundsX, nbObjects * sizeof(AABB_Xi));
			PxMemCopy(boundsYZCopy, inflatedBoundsYZ, nbObjects * sizeof(AABB_YZ));

			// PT: now we copy back the data in sorted order, to the aggregate buffers.
			for(PxU32 i=0;i<nbObjects;i++)
			{
				const PxU32 sortedIndex = sorted[i];
				agg->mAggregated[i] = copy[sortedIndex];
				inflatedBoundsX[i] = boundsXCopy[sortedIndex];
				inflatedBoundsYZ[i] = boundsYZCopy[sortedIndex];
			}
#endif
		}

#if !BOUNDS_DOUBLE_BUFFERING
		if(boundsXCopy != stackBoundsXCopy)
			PX_FREE(boundsXCopy);

		if(boundsYZCopy != stackBoundsYZCopy)
			PX_FREE(boundsYZCopy);
#endif
		if(minPosBounds != stackMinPosBounds)
			PX_FREE(minPosBounds);
	}

	virtual const char* getName() const { return "SortAggregateBoundsParallel"; }
};

class ProcessSelfCollisionPairsParallel : public ProcessAggPairsBase
{
public:
	Aggregate** mAggregates;
	PxU32 mNbAggs;
	AABBManager* mManager;

	ProcessSelfCollisionPairsParallel(PxU64 contextID, Aggregate** aggs, PxU32 nbAggs, AABBManager* manager) : ProcessAggPairsBase(contextID),
		mAggregates(aggs), mNbAggs(nbAggs), mManager(manager)
	{
	}

	void runInternal()
	{
		BpCacheData* data = mManager->getBpCacheData();
		setCache(*data);
		PX_PROFILE_ZONE("ProcessSelfCollisionPairs", mContextID);
		for (PxU32 i = 0; i < mNbAggs; i++)
		{
			Aggregate* aggregate = mAggregates[i];

			// PT: TODO: don't add filtered ones to this class at all!
			if(aggregate->mSelfCollisionPairs && PxGetAggregateType(aggregate->getFilterHint())!=PxAggregateType::eSTATIC)
				mManager->updatePairs(*aggregate->mSelfCollisionPairs, data);
		}
		updateCounters();
		mManager->putBpCacheData(data);
	}

	virtual const char* getName() const { return "ProcessSelfCollisionPairsParallel"; }
};

static void processAggregatePairsParallel(AggPairMap& map, AABBManager& manager, Cm::FlushPool& flushPool,
	PxBaseTask* continuation, const char* taskName, PxArray<ProcessAggPairsBase*>& pairTasks)
{
	// PT: TODO: hmmm we have a list of dirty aggregates but we don't have a list of dirty pairs.
	// PT: not sure how the 3.4 trunk solves this but let's just iterate all pairs for now
	// PT: atm we reuse this loop to delete removed interactions
	// PT: TODO: in fact we could handle all the "lost pairs" stuff right there with extra aabb-abb tests

	// PT: TODO: replace with decent hash map - or remove the hashmap entirely and use a linear array

	manager.mMapLock.lock();

	ProcessAggPairsParallelTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ProcessAggPairsParallelTask)), ProcessAggPairsParallelTask)(0, &manager.mMapLock, &manager, &map, taskName);

	PxU32 startIdx = pairTasks.size();

	for (AggPairMap::Iterator iter = map.getIterator(); !iter.done(); ++iter)
	{
		task->mAggPairs[task->mNbPairs] = iter->first;
		task->mPersistentPairs[task->mNbPairs++] = iter->second;
		if (task->mNbPairs == ProcessAggPairsParallelTask::MaxPairs)
		{
			pairTasks.pushBack(task);
			task->setContinuation(continuation);
			task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ProcessAggPairsParallelTask)), ProcessAggPairsParallelTask)(0, &manager.mMapLock, &manager, &map, taskName);
		}
	}

	manager.mMapLock.unlock();

	for (PxU32 i = startIdx; i < pairTasks.size(); ++i)
	{
		pairTasks[i]->removeReference();
	}

	if (task->mNbPairs)
	{
		pairTasks.pushBack(task);
		task->setContinuation(continuation);
		task->removeReference();
		//task->runInternal();
	}
}

void AABBManager::postBroadPhase(PxBaseTask* continuation, Cm::FlushPool& flushPool)
{
	PX_PROFILE_ZONE("AABBManager::postBroadPhase", mContextID);

	//KS - There is a continuation task for discrete broad phase, but not for CCD broad phase. PostBroadPhase for CCD broad phase runs in-line.
	//This probably should be revisited but it means that we can't use the parallel codepath within CCD.
	if (continuation)
	{
		mPostBroadPhase3.setContinuation(continuation);
		mPostBroadPhase2.setContinuation(&mPostBroadPhase3);
	}

	mTimestamp++;

	// PT: TODO: consider merging mCreatedOverlaps & mDestroyedOverlaps
	// PT: TODO: revisit memory management of mCreatedOverlaps & mDestroyedOverlaps

	// PT: this is now only used for CPU BPs so I think the fetchBroadPhaseResults call is useless here
#ifdef REMOVED
	//KS - if we ran broad phase, fetch the results now
	if (mAddedHandles.size() != 0 || mUpdatedHandles.size() != 0 || mRemovedHandles.size() != 0)
	{
		PX_PROFILE_ZONE("AABBManager::postBroadPhase - fetchResults", mContextID);
		mBroadPhase.fetchBroadPhaseResults();
	}
#endif

	for(PxU32 i=0; i<ElementType::eCOUNT; i++)
	{
		mCreatedOverlaps[i].resetOrClear();
		mDestroyedOverlaps[i].resetOrClear();
	}

	{
		PX_PROFILE_ZONE("AABBManager::postBroadPhase - process deleted pairs", mContextID);

		PxU32 nbDeletedPairs;
		const BroadPhasePair* deletedPairs = mBroadPhase.getDeletedPairs(nbDeletedPairs);
		processBPPairs<DeletedPairHandler>(nbDeletedPairs, deletedPairs, *this);
	}

	{
		//If there is a continuation task, then this is not part of CCD, so we can trigger bounds to be recomputed in parallel before pair generation runs during
		//stage 2.
		if (continuation)
		{
			const PxU32 size = mDirtyAggregates.size();
			for (PxU32 i = 0; i < size; i += SortAggregateBoundsParallel::MaxPairs)
			{
				const PxU32 nbToProcess = PxMin(size - i, SortAggregateBoundsParallel::MaxPairs);

				SortAggregateBoundsParallel* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(SortAggregateBoundsParallel)), SortAggregateBoundsParallel)
					(mContextID, &mDirtyAggregates[i], nbToProcess);

				task->setContinuation(&mPostBroadPhase2);
				task->removeReference();
			}
		}
	}

	if (continuation)
	{
		mPostBroadPhase2.setFlushPool(&flushPool);
		mPostBroadPhase3.removeReference();
		mPostBroadPhase2.removeReference();
	}
	else
	{
		postBpStage2(NULL, flushPool);
		postBpStage3(NULL);
	}
}

void AABBManager::reallocateChangedAABBMgActorHandleMap(const PxU32 size)
{
	mChangedHandleMap.resizeAndClear(size);
}

void PostBroadPhaseStage2Task::runInternal()
{
	mManager.postBpStage2(mCont, *mFlushPool);
}

void AABBManager::postBpStage2(PxBaseTask* continuation, Cm::FlushPool& flushPool)
{
	{
		const PxU32 size = mDirtyAggregates.size();
		for (PxU32 i = 0; i < size; i += ProcessSelfCollisionPairsParallel::MaxPairs)
		{
			const PxU32 nbToProcess = PxMin(size - i, ProcessSelfCollisionPairsParallel::MaxPairs);

			ProcessSelfCollisionPairsParallel* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ProcessSelfCollisionPairsParallel)), ProcessSelfCollisionPairsParallel)
				(mContextID, &mDirtyAggregates[i], nbToProcess, this);

			startTask(task, continuation);

			mAggPairTasks.pushBack(task);
		}
	}

	{
		if (continuation)
			processAggregatePairsParallel(mAggregateAggregatePairs, *this, flushPool, continuation, "AggAggPairs", mAggPairTasks);
		else
			processAggregatePairs(mAggregateAggregatePairs, *this);
	}

	{
		if (continuation)
			processAggregatePairsParallel(mActorAggregatePairs, *this, flushPool, continuation, "AggActorPairs", mAggPairTasks);
		else
			processAggregatePairs(mActorAggregatePairs, *this);
	}
}

void AABBManager::postBpStage3(PxBaseTask*)
{
	{
		PX_PROFILE_ZONE("SimpleAABBManager::postBroadPhase - aggregate self-collisions", mContextID);
		const PxU32 size = mDirtyAggregates.size();
		for (PxU32 i = 0; i < size; i++)
		{
			Aggregate* aggregate = mDirtyAggregates[i];
			aggregate->resetDirtyState();
		}
		mDirtyAggregates.resetOrClear();
	}

	{
		PX_PROFILE_ZONE("SimpleAABBManager::postBroadPhase - append pairs", mContextID);

		for (PxU32 a = 0; a < mAggPairTasks.size(); ++a)
		{
			ProcessAggPairsBase* task = mAggPairTasks[a];
			for (PxU32 t = 0; t < 2; t++)
			{
				for (PxU32 i = 0, startIdx = task->mCreatedPairs[t].mStartIdx; i < task->mCreatedPairs[t].mCount; ++i)
				{
					mCreatedOverlaps[t].pushBack((*task->mCreatedPairs[t].mArray)[i + startIdx]);
				}
				for (PxU32 i = 0, startIdx = task->mDestroyedPairs[t].mStartIdx; i < task->mDestroyedPairs[t].mCount; ++i)
				{
					mDestroyedOverlaps[t].pushBack((*task->mDestroyedPairs[t].mArray)[i + startIdx]);
				}
			}
		}

		mAggPairTasks.forceSize_Unsafe(0);

		resetBpCacheData();
	}

	{
		PX_PROFILE_ZONE("AABBManager::postBroadPhase - process created pairs", mContextID);

		PxU32 nbCreatedPairs;
		const BroadPhasePair* createdPairs = mBroadPhase.getCreatedPairs(nbCreatedPairs);
		processBPPairs<CreatedPairHandler>(nbCreatedPairs, createdPairs, *this);
	}

	// PT: TODO: revisit this
	// Filter out pairs in mDestroyedOverlaps that already exist in mCreatedOverlaps. This should be done better using bitmaps
	// and some imposed ordering on previous operations. Later.
	// We could also have a dedicated function "reinsertBroadPhase()", which would preserve the existing interactions at Sc-level.
	if(1)
	{
		PX_PROFILE_ZONE("AABBManager::postBroadPhase - post-process", mContextID);

		PxU32 totalCreatedOverlaps = 0;
		for (PxU32 idx=0; idx<ElementType::eCOUNT; idx++)
			totalCreatedOverlaps += mCreatedOverlaps[idx].size();

		mCreatedPairsTmp.clear();
		mCreatedPairsTmp.reserve(totalCreatedOverlaps);
		
		// PT: so this is where we convert the userData IDs to pointers
		// I don't remember why we need the mCreatedPairs hashset / this filtering pass
		// PT: TODO: why do we need to convert to ptrs at all anyway?
		for(PxU32 idx=0; idx<ElementType::eCOUNT; idx++)
		{
			{
				const PxU32 nbDestroyedOverlaps = mDestroyedOverlaps[idx].size();
				PxU32 size = mCreatedOverlaps[idx].size();
				AABBOverlap* overlaps = mCreatedOverlaps[idx].begin();
				while(size--)
				{
					const PxU32 id0 = PxU32(size_t(overlaps->mUserData0));
					const PxU32 id1 = PxU32(size_t(overlaps->mUserData1));
					overlaps->mUserData0 = mVolumeData[id0].getUserData();
					overlaps->mUserData1 = mVolumeData[id1].getUserData();
					overlaps++;
					if(nbDestroyedOverlaps)
						mCreatedPairsTmp.insert(Pair(id0, id1));
				}
			}
			{
				AABBOverlap* overlapsSrc = mDestroyedOverlaps[idx].begin();
				AABBOverlap* overlapsDst = overlapsSrc;

				PxU32 size = mDestroyedOverlaps[idx].size();
				PxU32 newSize = 0;
				while(size--)
				{
					const PxU32 id0 = PxU32(size_t(overlapsSrc->mUserData0));
					const PxU32 id1 = PxU32(size_t(overlapsSrc->mUserData1));
					overlapsSrc++;

					if(!mCreatedPairsTmp.contains(Pair(id0, id1)))
					{
						overlapsDst->mUserData0 = mVolumeData[id0].getUserData();
						overlapsDst->mUserData1 = mVolumeData[id1].getUserData();
						overlapsDst++;
						newSize++;
					}
				}
				mDestroyedOverlaps[idx].forceSize_Unsafe(newSize);
			}
		}
	}

	// Handle out-of-bounds objects
	{
		PX_PROFILE_ZONE("AABBManager::postBroadPhase - out-of-bounds", mContextID);
		PxU32 nbObjects = mBroadPhase.getNbOutOfBoundsObjects();
		const PxU32* objects = mBroadPhase.getOutOfBoundsObjects();
		while(nbObjects--)
		{
			const PxU32 index = *objects++;
			if(!mRemovedHandleMap.test(index))
			{
				if(mVolumeData[index].isSingleActor())
					mOutOfBoundsObjects.pushBack(mVolumeData[index].getUserData());
				else
				{
					PX_ASSERT(mVolumeData[index].isAggregate());
					mOutOfBoundsAggregates.pushBack(mVolumeData[index].getUserData());
				}
			}
		}
	}

	{
		PX_PROFILE_ZONE("AABBManager::postBroadPhase - clear", mContextID);
		mAddedHandleMap.clear();
		mRemovedHandleMap.clear();
	}
}

BpCacheData* AABBManager::getBpCacheData()
{
	BpCacheData* rv = static_cast<BpCacheData*>(mBpThreadContextPool.pop());
	if (rv == NULL)
	{
		rv = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(BpCacheData), "BpCacheData"), BpCacheData)();
	}
	return rv;
}

void AABBManager::putBpCacheData(BpCacheData* data)
{
	mBpThreadContextPool.push(*data);
}

void AABBManager::resetBpCacheData()
{
	PxInlineArray<BpCacheData*, 16> bpCache;
	BpCacheData* entry = static_cast<BpCacheData*>(mBpThreadContextPool.pop());
	while (entry)
	{
		entry->reset();
		bpCache.pushBack(entry);
		entry = static_cast<BpCacheData*>(mBpThreadContextPool.pop());
	}

	//Now reinsert back into queue...
	for (PxU32 i = 0; i < bpCache.size(); ++i)
	{
		mBpThreadContextPool.push(*bpCache[i]);
	}
}

bool AABBManager::getOutOfBoundsObjects(OutOfBoundsData& data)
{
	data.mNbOutOfBoundsObjects = mOutOfBoundsObjects.size();
	data.mOutOfBoundsObjects = mOutOfBoundsObjects.begin();
	data.mNbOutOfBoundsAggregates = mOutOfBoundsAggregates.size();
	data.mOutOfBoundsAggregates = mOutOfBoundsAggregates.begin();
	return data.mNbOutOfBoundsObjects || data.mNbOutOfBoundsAggregates;
}

void AABBManager::clearOutOfBoundsObjects()
{
	mOutOfBoundsObjects.clear();
	mOutOfBoundsAggregates.clear();
}

// PT: disabled this by default, since it bypasses all per-shape/per-actor visualization flags
//static const bool gVisualizeAggregateElems = false;
void AABBManager::visualize(PxRenderOutput& out)
{
	const PxTransform idt = PxTransform(PxIdentity);
	out << idt;

	PxBitMap bitmap;
	buildFreeBitmap(bitmap, mFirstFreeAggregate, mAggregates);

	const PxU32 N = mAggregates.size();
	for(PxU32 i=0;i<N;i++)
	{
		if(bitmap.test(i))
			continue;

		Aggregate* aggregate = mAggregates[i];
		if(aggregate->getNbAggregated())
		{
			out << PxU32(PxDebugColor::eARGB_GREEN);
			const PxBounds3& b = mBoundsArray.getBounds(aggregate->mIndex);
			renderOutputDebugBox(out, b);
		}
	}

/*	const PxU32 N = mAggregateManager.getAggregatesCapacity();
	for(PxU32 i=0;i<N;i++)
	{
		const Aggregate* aggregate = mAggregateManager.getAggregate(i);
		if(aggregate->nbElems)
		{
			if(!mAggregatesUpdated.isInList(BpHandle(i)))
				out << PxU32(PxDebugColor::eARGB_GREEN);
			else
				out << PxU32(PxDebugColor::eARGB_RED);

			PxBounds3 decoded;
			const IntegerAABB& iaabb = mBPElems.getAABB(aggregate->bpElemId);
			iaabb.decode(decoded);

			out << DebugBox(decoded, true);

			if(gVisualizeAggregateElems)
			{
				PxU32 elem = aggregate->elemHeadID;
				while(BP_INVALID_BP_HANDLE!=elem)
				{
					out << PxU32(PxDebugColor::eARGB_CYAN);
					const IntegerAABB elemBounds = mAggregateElems.getAABB(elem);
					elemBounds.decode(decoded);
					out << DebugBox(decoded, true);
					elem = mAggregateElems.getNextId(elem);
				}
			}
		}
	}*/
}

} //namespace Bp

} //namespace physx

