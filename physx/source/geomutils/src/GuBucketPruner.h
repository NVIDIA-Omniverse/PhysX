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

#ifndef GU_BUCKET_PRUNER_H
#define GU_BUCKET_PRUNER_H

#include "common/PxPhysXCommonConfig.h"
#include "GuPruner.h"
#include "GuSqInternal.h"
#include "GuPruningPool.h"
#include "foundation/PxHash.h"

#define FREE_PRUNER_SIZE	16
//#define USE_REGULAR_HASH_MAP
#ifdef USE_REGULAR_HASH_MAP
	#include "foundation/PxHashMap.h"
#endif

namespace physx
{
	class PxRenderOutput;

namespace Gu
{
	typedef PxU32	BucketWord;
	
#if PX_VC 
    #pragma warning(push)
	#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif

	PX_ALIGN_PREFIX(16)	struct BucketBox
	{
		PxVec3	mCenter;
		PxU32	mData0;		// Integer-encoded min value along sorting axis
		PxVec3	mExtents;
		PxU32	mData1;		// Integer-encoded max value along sorting axis

	#if PX_DEBUG
		// PT: we need the original min value for debug checks. Using the center/extents version
		// fails because recomputing the min from them introduces FPU accuracy errors in the values.
		float	mDebugMin;
	#endif

		PX_FORCE_INLINE	PxVec3	getMin()	const
		{
			return mCenter - mExtents;
		}

		PX_FORCE_INLINE	PxVec3	getMax()	const
		{
			return mCenter + mExtents;
		}

		PX_FORCE_INLINE void	setEmpty()
		{
			mCenter = PxVec3(0.0f);
			mExtents = PxVec3(-PX_MAX_BOUNDS_EXTENTS);

	#if PX_DEBUG
			mDebugMin = PX_MAX_BOUNDS_EXTENTS;
	#endif
		}
	}PX_ALIGN_SUFFIX(16);

	PX_ALIGN_PREFIX(16) struct BucketPrunerNode
	{
					BucketPrunerNode();

		void		classifyBoxes(	float limitX, float limitZ,
									PxU32 nb,
									BucketBox* PX_RESTRICT boxes,
									const PrunerPayload* PX_RESTRICT objects,
									const PxTransform* PX_RESTRICT transforms,
									BucketBox* PX_RESTRICT sortedBoxes,
									PrunerPayload* PX_RESTRICT sortedObjects,
									PxTransform* PX_RESTRICT sortedTransforms,
									bool isCrossBucket, PxU32 sortAxis);

		PX_FORCE_INLINE	void	initCounters()
		{
			for(PxU32 i=0;i<5;i++)
				mCounters[i] = 0;
			for(PxU32 i=0;i<5;i++)
				mOffsets[i] = 0;
		}

		BucketWord	mCounters[5];	// Number of objects in each of the 5 children
		BucketWord	mOffsets[5];	// Start index of objects for each of the 5 children
		BucketBox	mBucketBox[5];	// AABBs around objects for each of the 5 children
		PxU16		mOrder[8];		// PNS: 5 children => 3 bits/index => 3*5=15 bits total, for each of the 8 canonical directions
	}PX_ALIGN_SUFFIX(16);

	PX_FORCE_INLINE PxU32 PxComputeHash(const PrunerPayload& payload)
	{
#if PX_P64_FAMILY
//		const PxU32 h0 = PxHash((const void*)payload.data[0]);
//		const PxU32 h1 = PxHash((const void*)payload.data[1]);
		const PxU32 h0 = PxU32(PX_MAX_U32 & payload.data[0]);
		const PxU32 h1 = PxU32(PX_MAX_U32 & payload.data[1]);
		return physx::PxComputeHash(PxU64(h0)|(PxU64(h1)<<32));
#else
		return physx::PxComputeHash(PxU64(payload.data[0])|(PxU64(payload.data[1])<<32));
#endif
	}

#ifdef USE_REGULAR_HASH_MAP
	struct BucketPrunerPair : public PxUserAllocated
	{
		PX_FORCE_INLINE	BucketPrunerPair()	{}
		PX_FORCE_INLINE	BucketPrunerPair(PxU32 index, PxU32 stamp) : mCoreIndex(index), mTimeStamp(stamp)	{}
		PxU32			mCoreIndex;	// index in mCoreObjects
		PxU32			mTimeStamp;
	};
	typedef PxHashMap<PrunerPayload, BucketPrunerPair> BucketPrunerMap;
#else
	struct BucketPrunerPair : public PxUserAllocated
	{
		PrunerPayload	mData;
		PxU32			mCoreIndex;	// index in mCoreObjects
		PxU32			mTimeStamp;
	};

	// Custom hash-map - currently faster than the regular hash-map (PxHashMap), in particular for 'find-and-erase' operations.
	class BucketPrunerMap : public PxUserAllocated
	{
		public:
												BucketPrunerMap();
												~BucketPrunerMap();

						void					purge();
						void					shrinkMemory();

						BucketPrunerPair*		addPair		(const PrunerPayload& payload, PxU32 coreIndex, PxU32 timeStamp);
						bool					removePair	(const PrunerPayload& payload, PxU32& coreIndex, PxU32& timeStamp);
						const BucketPrunerPair*	findPair	(const PrunerPayload& payload)	const;
		PX_FORCE_INLINE	PxU32					getPairIndex(const BucketPrunerPair* pair)	const
												{
													return (PxU32((size_t(pair) - size_t(mActivePairs)))/sizeof(BucketPrunerPair));
												}

						PxU32					mHashSize;
						PxU32					mMask;
						PxU32					mNbActivePairs;
						PxU32*					mHashTable;
						PxU32*					mNext;
						BucketPrunerPair*		mActivePairs;
						PxU32					mReservedMemory;

		PX_FORCE_INLINE	BucketPrunerPair*		findPair(const PrunerPayload& payload, PxU32 hashValue) const;
						void					removePairInternal(const PrunerPayload& payload, PxU32 hashValue, PxU32 pairIndex);
						void					reallocPairs();
						void					reserveMemory(PxU32 memSize);
	};
#endif

	class BucketPrunerCore : public PxUserAllocated
	{
		public:
		PX_PHYSX_COMMON_API						BucketPrunerCore(bool externalMemory=true);
		PX_PHYSX_COMMON_API						~BucketPrunerCore();

							void				release();

							void				setExternalMemory(PxU32 nbObjects, PxBounds3* boxes, PrunerPayload* objects, PxTransform* transforms);

		PX_PHYSX_COMMON_API	bool				addObject(const PrunerPayload& object, const PxBounds3& worldAABB, const PxTransform& transform, PxU32 timeStamp=0);
							bool				removeObject(const PrunerPayload& object, PxU32& timeStamp);
							bool				updateObject(const PxBounds3& worldAABB, const PrunerPayload& object, const PxTransform& transform);

		// PT: look for objects marked with input timestamp everywhere in the structure, and remove them. This is the same
		// as calling 'removeObject' individually for all these objects, but much more efficient. Returns number of removed objects.
							PxU32				removeMarkedObjects(PxU32 timeStamp);

		PX_PHYSX_COMMON_API	bool				raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback&) const;
		PX_PHYSX_COMMON_API	bool				overlap(const ShapeData& queryVolume, PrunerOverlapCallback&) const;
		PX_PHYSX_COMMON_API	bool				sweep(const ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback&) const;

							void				getGlobalBounds(PxBounds3& bounds)	const;

							void				shiftOrigin(const PxVec3& shift);

							void				visualize(PxRenderOutput& out, PxU32 color) const;

		PX_FORCE_INLINE		void				build()					{ classifyBoxes();	}

#ifdef FREE_PRUNER_SIZE
		PX_FORCE_INLINE		PxU32				getNbObjects()	const	{ return mNbFree + mCoreNbObjects;	}
#else
		PX_FORCE_INLINE		PxU32				getNbObjects()	const	{ return mCoreNbObjects;	}
#endif

//		private:
							PxU32				mCoreNbObjects;		// Current number of objects in core arrays
							PxU32				mCoreCapacity;		// Capacity of core arrays
							PxBounds3*			mCoreBoxes;			// Core array
							PrunerPayload*		mCoreObjects;		// Core array
							PxTransform*		mCoreTransforms;
							PxU32*				mCoreRemap;			// Remaps core index to sorted index, i.e. sortedIndex = mCoreRemap[coreIndex]

							BucketBox*			mSortedWorldBoxes;	// Sorted array
							PrunerPayload*		mSortedObjects;		// Sorted array
							PxTransform*		mSortedTransforms;
#ifdef FREE_PRUNER_SIZE
							PxU32				mNbFree;						// Current number of objects in the "free array" (mFreeObjects/mFreeBounds)
							PrunerPayload		mFreeObjects[FREE_PRUNER_SIZE];	// mNbFree objects are stored here
							PxBounds3			mFreeBounds[FREE_PRUNER_SIZE];	// mNbFree object bounds are stored here
							PxTransform			mFreeTransforms[FREE_PRUNER_SIZE];	// mNbFree transforms are stored here
							PxU32				mFreeStamps[FREE_PRUNER_SIZE];
#endif
							BucketPrunerMap		mMap;			// Maps (PrunerPayload) object to corresponding index in core array.
																// Objects in the free array do not appear in this map.
							PxU32				mSortedNb;
							PxU32				mSortedCapacity;
							PxU32				mSortAxis;

							BucketBox			mGlobalBox;		// Global bounds around all objects in the structure (except the ones in the "free" array)
							BucketPrunerNode	mLevel1;
							BucketPrunerNode	mLevel2[5];
							BucketPrunerNode	mLevel3[5][5];

							bool				mDirty;
							bool				mOwnMemory;
		private:
		PX_PHYSX_COMMON_API	void				classifyBoxes();
							void				allocateSortedMemory(PxU32 nb);
							void				resizeCore();
		PX_FORCE_INLINE		void				addObjectInternal(const PrunerPayload& object, const PxBounds3& worldAABB, const PxTransform& transform, PxU32 timeStamp);
	};

#if PX_VC 
     #pragma warning(pop) 
#endif

	class BucketPruner : public Pruner
	{
		public:
		PX_PHYSX_COMMON_API			BucketPruner(PxU64 contextID);
		virtual						~BucketPruner();

		// BasePruner
									DECLARE_BASE_PRUNER_API
		//~BasePruner

		// Pruner
									DECLARE_PRUNER_API_COMMON
		//~Pruner

		private:
				BucketPrunerCore	mCore;
				PruningPool			mPool;
	};

}

}

#endif
