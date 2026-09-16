// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef BP_BROADPHASE_SHARED_H
#define BP_BROADPHASE_SHARED_H

#include "BpBroadPhaseIntegerAABB.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxHash.h"
#include "foundation/PxVecMath.h"

namespace physx
{
namespace Bp
{
	#define	INVALID_ID		0xffffffff
	#define INVALID_USER_ID	0xffffffff

	struct InternalPair : public PxUserAllocated
	{
		PX_FORCE_INLINE	PxU32	getId0()	const	{ return id0_isNew & ~PX_SIGN_BITMASK;		}
		PX_FORCE_INLINE	PxU32	getId1()	const	{ return id1_isUpdated & ~PX_SIGN_BITMASK;	}

		PX_FORCE_INLINE	PxU32	isNew()		const	{ return id0_isNew & PX_SIGN_BITMASK;		}
		PX_FORCE_INLINE	PxU32	isUpdated()	const	{ return id1_isUpdated & PX_SIGN_BITMASK;	}

		PX_FORCE_INLINE	void	setNewPair(PxU32 id0, PxU32 id1)
		{
			PX_ASSERT(!(id0 & PX_SIGN_BITMASK));
			PX_ASSERT(!(id1 & PX_SIGN_BITMASK));
			id0_isNew = id0 | PX_SIGN_BITMASK;
			id1_isUpdated = id1;
		}

		PX_FORCE_INLINE	void	setNewPair2(PxU32 id0, PxU32 id1)
		{
			PX_ASSERT(!(id0 & PX_SIGN_BITMASK));
			PX_ASSERT(!(id1 & PX_SIGN_BITMASK));
			id0_isNew = id0;
			id1_isUpdated = id1;
		}

		PX_FORCE_INLINE	void	setUpdated()		{ id1_isUpdated |= PX_SIGN_BITMASK;		}
		PX_FORCE_INLINE	void	clearUpdated()		{ id1_isUpdated &= ~PX_SIGN_BITMASK;	}
		PX_FORCE_INLINE	void	clearNew()			{ id0_isNew &= ~PX_SIGN_BITMASK;		}

		protected:
		PxU32		id0_isNew;
		PxU32		id1_isUpdated;
	};

	PX_FORCE_INLINE bool	differentPair(const InternalPair& p, PxU32 id0, PxU32 id1)	{ return (id0!=p.getId0()) || (id1!=p.getId1());	}
	PX_FORCE_INLINE void	sort(PxU32& id0, PxU32& id1)								{ if(id0>id1)	PxSwap(id0, id1);					}

	PX_FORCE_INLINE PxU32	hash(PxU32 id0, PxU32 id1)
	{
		if(id0<=0xffff && id1<=0xffff)
			return PxComputeHash((id0)|(id1<<16));
		else
			return PxComputeHash(PxU64(id0)|(PxU64(id1)<<32));
	}

	class PairManagerData
	{
		public:
										PairManagerData();
										~PairManagerData();

		PX_FORCE_INLINE	PxU32			getPairIndex(const InternalPair* pair)	const
										{
											return (PxU32((size_t(pair) - size_t(mActivePairs)))/sizeof(InternalPair));
										}

		// Internal version saving hash computation
		PX_FORCE_INLINE InternalPair*	findPair(PxU32 id0, PxU32 id1, PxU32 hashValue) const
										{
											if(!mHashTable)
												return NULL;	// Nothing has been allocated yet

											InternalPair* PX_RESTRICT activePairs = mActivePairs;
											const PxU32* PX_RESTRICT next = mNext;

											// Look for it in the table
											PxU32 offset = mHashTable[hashValue];
											while(offset!=INVALID_ID && differentPair(activePairs[offset], id0, id1))
											{
												PX_ASSERT(activePairs[offset].getId0()!=INVALID_USER_ID);
												offset = next[offset];		// Better to have a separate array for this
											}
											if(offset==INVALID_ID)
												return NULL;
											PX_ASSERT(offset<mNbActivePairs);
											// Match mActivePairs[offset] => the pair is persistent

											return &activePairs[offset];
										}

		PX_FORCE_INLINE	InternalPair*	addPairInternal(PxU32 id0, PxU32 id1)
										{
											// Order the ids
											sort(id0, id1);

											const PxU32 fullHashValue = hash(id0, id1);
											PxU32 hashValue = fullHashValue & mMask;

											{
												InternalPair* PX_RESTRICT p = findPair(id0, id1, hashValue);
												if(p)
												{
													p->setUpdated();
													return p;	// Persistent pair
												}
											}

											// This is a new pair
											if(mNbActivePairs >= mHashSize)
												hashValue = growPairs(fullHashValue);

											const PxU32 pairIndex = mNbActivePairs++;

											InternalPair* PX_RESTRICT p = &mActivePairs[pairIndex];
											p->setNewPair(id0, id1);
											mNext[pairIndex] = mHashTable[hashValue];
											mHashTable[hashValue] = pairIndex;
											return p;
										}

						PxU32			mHashSize;
						PxU32			mMask;
						PxU32			mNbActivePairs;
						PxU32*			mHashTable;
						PxU32*			mNext;
						InternalPair*	mActivePairs;
						PxU32			mReservedMemory;

						void			purge();
						void			reallocPairs();
						void			shrinkMemory();
						void			reserveMemory(PxU32 memSize);
		PX_NOINLINE		PxU32			growPairs(PxU32 fullHashValue);
						void			removePair(PxU32 id0, PxU32 id1, PxU32 hashValue, PxU32 pairIndex);
	};

	// PT: the box-pruning kernels terminate their X sweeps on the sentinel value below, which only works if
	// the sentinel is strictly greater than any encoded coordinate of a real box. encodeFloat() maps the NaN
	// whose bit pattern is 0x7fffffff onto exactly gBoxPruningSentinel, so a NaN bound would defeat that and
	// the sweeps would run past the end of the box list. Clamping to gMaxEncodedCoord restores the invariant;
	// it is a no-op for every other input, including all finite floats and both infinities.
	static const PxU32	gBoxPruningSentinel	= 0xffffffff;
	static const PxU32	gMaxEncodedCoord	= 0xfffffffe;

	struct AABB_Xi	// PT: i for integer
	{
		PX_FORCE_INLINE	AABB_Xi()	{}
		PX_FORCE_INLINE	~AABB_Xi()	{}

		PX_FORCE_INLINE	void	initFromFloats(const void* PX_RESTRICT minX, const void* PX_RESTRICT maxX)
		{
			mMinX = PxMin(encodeFloat(*reinterpret_cast<const PxU32*>(minX)), gMaxEncodedCoord);
			mMaxX = PxMin(encodeFloat(*reinterpret_cast<const PxU32*>(maxX)), gMaxEncodedCoord);
		}

		PX_FORCE_INLINE	void	initFromPxVec4(const PxVec4& min, const PxVec4& max)
		{
			initFromFloats(&min.x, &max.x);
		}

		PX_FORCE_INLINE	void	operator = (const AABB_Xi& box)
		{
			mMinX = box.mMinX;
			mMaxX = box.mMaxX;
		}

		PX_FORCE_INLINE	void	initSentinel()
		{
			mMinX = gBoxPruningSentinel;
		}

		PX_FORCE_INLINE bool	isSentinel()	const
		{
			return mMinX == gBoxPruningSentinel;
		}

		PxU32 mMinX;
		PxU32 mMaxX;
	};

	struct AABB_YZn	// PT: n for negated min layout
	{
		PX_FORCE_INLINE	AABB_YZn()	{}
		PX_FORCE_INLINE	~AABB_YZn()	{}

		PX_FORCE_INLINE	void	initFromPxVec4(const PxVec4& min, const PxVec4& max)
		{
			mMinY	= -min.y;
			mMinZ	= -min.z;
			mMaxY	= max.y;
			mMaxZ	= max.z;
		}

		PX_FORCE_INLINE	void	operator = (const AABB_YZn& box)
		{
			using namespace physx::aos;
			V4StoreA(V4LoadA(&box.mMinY), &mMinY);
		}

		float mMinY;
		float mMinZ;
		float mMaxY;
		float mMaxZ;
	};

	struct AABB_YZr	// PT: r for regular layout
	{
		PX_FORCE_INLINE	AABB_YZr()	{}
		PX_FORCE_INLINE	~AABB_YZr()	{}

		PX_FORCE_INLINE	void	initFromPxVec4(const PxVec4& min, const PxVec4& max)
		{
			mMinY	= min.y;
			mMinZ	= min.z;
			mMaxY	= max.y;
			mMaxZ	= max.z;
		}

		PX_FORCE_INLINE	void	operator = (const AABB_YZr& box)
		{
			using namespace physx::aos;
			V4StoreA(V4LoadA(&box.mMinY), &mMinY);
		}

		float mMinY;
		float mMinZ;
		float mMaxY;
		float mMaxZ;
	};

} //namespace Bp
} //namespace physx

#endif // BP_BROADPHASE_SHARED_H
