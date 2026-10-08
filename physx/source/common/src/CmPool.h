// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef CM_POOL_H
#define CM_POOL_H

#include "foundation/PxSort.h"
#include "foundation/PxMutex.h"
#include "foundation/PxBasicTemplates.h"
#include "foundation/PxBitMap.h"

namespace physx
{
namespace Cm
{

/*!
Allocator for pools of data structures
Also decodes indices (which can be computed from handles) into objects. To make this
faster, the EltsPerSlab must be a power of two
*/
template <class T> 
class PoolList : public PxAllocatorTraits<T>::Type
{
	typedef typename PxAllocatorTraits<T>::Type Alloc;
	PX_NOCOPY(PoolList)
public:
	PX_INLINE PoolList(const Alloc& alloc, PxU32 eltsPerSlab)
		: Alloc(alloc),
		mEltsPerSlab(eltsPerSlab), 
		mSlabCount(0),
		mFreeList(0), 
		mFreeCount(0), 
		mSlabs(NULL)
	{
		PX_ASSERT(mEltsPerSlab>0);
		PX_ASSERT((mEltsPerSlab & (mEltsPerSlab-1)) == 0);
		mLog2EltsPerSlab = 0;

		for(mLog2EltsPerSlab=0; mEltsPerSlab!=PxU32(1<<mLog2EltsPerSlab); mLog2EltsPerSlab++)
				;
	}

	PX_INLINE ~PoolList()
	{
		destroy();
	}

	PX_INLINE void destroy()
	{
		// Run all destructors
		for(PxU32 i=0;i<mSlabCount;i++)
		{
			PX_ASSERT(mSlabs);
			T* slab = mSlabs[i];
			for(PxU32 j=0;j<mEltsPerSlab;j++)
			{
				slab[j].~T();
			}
		}

		//Deallocate
		for(PxU32 i=0;i<mSlabCount;i++)
		{
			Alloc::deallocate(mSlabs[i]);
			mSlabs[i] = NULL;
		}
		mSlabCount = 0;

		if(mFreeList)
			Alloc::deallocate(mFreeList);
		mFreeList = NULL;
		if(mSlabs)
		{
			Alloc::deallocate(mSlabs);
			mSlabs = NULL;
		}
	}

	PxU32 preallocate(const PxU32 nbRequired, T** elements)
	{
		//(1) Allocate and pull out an array of X elements

		PxU32 nbToAllocate = nbRequired > mFreeCount ? nbRequired - mFreeCount : 0;

		PxU32 nbElements = nbRequired - nbToAllocate;

		PxMemCopy(elements, mFreeList + (mFreeCount - nbElements), sizeof(T*) * nbElements);
		//PxU32 originalFreeCount = mFreeCount;
		mFreeCount -= nbElements;

		if (nbToAllocate)
		{
			PX_ASSERT(mFreeCount == 0);

			PxU32 nbSlabs = (nbToAllocate + mEltsPerSlab - 1) / mEltsPerSlab; //The number of slabs we need to allocate...
			//allocate our slabs...

			PxU32 freeCount = mFreeCount;

			for (PxU32 i = 0; i < nbSlabs; ++i)
			{

				//KS - would be great to allocate this using a single allocation but it will make releasing slabs fail later :(
				T * mAddr = reinterpret_cast<T*>(Alloc::allocate(mEltsPerSlab * sizeof(T), PX_FL));
				if (!mAddr)
					return nbElements; //Allocation failed so only return the set of elements we could allocate from the free list

				PxU32 newSlabCount = mSlabCount+1;

				// Make sure the usage bitmap is up-to-size
				if (mUseBitmap.size() < newSlabCount*mEltsPerSlab)
				{
					mUseBitmap.resize(2 * newSlabCount*mEltsPerSlab); //set last element as not used
					if (mFreeList)
						Alloc::deallocate(mFreeList);
					mFreeList = reinterpret_cast<T**>(Alloc::allocate(2 * newSlabCount * mEltsPerSlab * sizeof(T*), PX_FL));

					T** slabs = reinterpret_cast<T**>(Alloc::allocate(2* newSlabCount *sizeof(T*), PX_FL));
					if (mSlabs)
					{
						PxMemCopy(slabs, mSlabs, sizeof(T*)*mSlabCount);

						Alloc::deallocate(mSlabs);
					}

					mSlabs = slabs;
				}

				mSlabs[mSlabCount++] = mAddr;

				PxU32 baseIndex = (mSlabCount-1) * mEltsPerSlab;

				//Now add all these to the mFreeList and elements...
				PxI32 idx = PxI32(mEltsPerSlab - 1);

				for (; idx >= PxI32(nbToAllocate); --idx)
				{
					mFreeList[freeCount++] = PX_PLACEMENT_NEW(mAddr + idx, T(baseIndex + idx));
				}

				PxU32 origElements = nbElements;
				T** writeIdx = elements + nbElements;
				for (; idx >= 0; --idx)
				{
					writeIdx[idx] = PX_PLACEMENT_NEW(mAddr + idx, T(baseIndex + idx));
					nbElements++;
				}

				nbToAllocate -= (nbElements - origElements);
			}

			mFreeCount = freeCount;
		}
		
		PX_ASSERT(nbElements == nbRequired);

		for (PxU32 a = 0; a < nbElements; ++a)
		{
			mUseBitmap.set(elements[a]->getIndex());
		}

		return nbRequired;
	}

	// TODO: would be nice to add templated construct/destroy methods like ObjectPool

	PX_INLINE T* get()
	{
		if(mFreeCount == 0 && !extend())
			return 0;
		T* element = mFreeList[--mFreeCount];
		mUseBitmap.set(element->getIndex());
		return element;
	}

	PX_INLINE void put(T* element)
	{
		PxU32 i = element->getIndex();
		mUseBitmap.reset(i);
		mFreeList[mFreeCount++] = element;
	}

	/*
		WARNING: Unlike findByIndexFast below, this method is NOT safe to use if another thread 
		is concurrently updating the pool (e.g. through put/get/extend/getIterator), since the
		safety boundedTest uses mSlabCount and mUseBitmap.
	*/
	PX_FORCE_INLINE T* findByIndex(PxU32 index) const
	{
		if(index>=mSlabCount*mEltsPerSlab || !(mUseBitmap.boundedTest(index)))
			return 0;
		return mSlabs[index>>mLog2EltsPerSlab] + (index&(mEltsPerSlab-1));
	}

	/*
		This call is safe to do while other threads update the pool.
	*/
	PX_FORCE_INLINE T* findByIndexFast(PxU32 index) const
	{
		return mSlabs[index>>mLog2EltsPerSlab] + (index&(mEltsPerSlab-1));
	}

	bool extend()
	{
		T * mAddr = reinterpret_cast<T*>(Alloc::allocate(mEltsPerSlab * sizeof(T), PX_FL));
		if(!mAddr)
			return false;

		PxU32 newSlabCount = mSlabCount+1;

		// Make sure the usage bitmap is up-to-size
		if(mUseBitmap.size() < newSlabCount*mEltsPerSlab)
		{
			mUseBitmap.resize(2* newSlabCount*mEltsPerSlab); //set last element as not used
			if(mFreeList)
				Alloc::deallocate(mFreeList);
			mFreeList = reinterpret_cast<T**>(Alloc::allocate(2* newSlabCount * mEltsPerSlab * sizeof(T*), PX_FL));

			T** slabs = reinterpret_cast<T**>(Alloc::allocate(2 * newSlabCount * sizeof(T*), PX_FL));
			if (mSlabs)
			{
				PxMemCopy(slabs, mSlabs, sizeof(T*)*mSlabCount);

				Alloc::deallocate(mSlabs);
			}

			mSlabs = slabs;
		}

		mSlabs[mSlabCount++] = mAddr;
	
		// Add to free list in descending order so that lowest indices get allocated first - 
		// the FW context code currently *relies* on this behavior to grab the zero-index volume
		// which can't be allocated to the user. TODO: fix this

		PxU32 baseIndex = (mSlabCount-1) * mEltsPerSlab;
		PxU32 freeCount = mFreeCount;
		for(PxI32 i=PxI32(mEltsPerSlab-1);i>=0;i--)
			mFreeList[freeCount++] = PX_PLACEMENT_NEW(mAddr+i, T(baseIndex+ i));

		mFreeCount = freeCount;

		return true;
	}

	PX_INLINE PxU32 getMaxUsedIndex()	const
	{
		return mUseBitmap.findLast();
	}

	PX_INLINE PxBitMap::Iterator getIterator() const
	{
		return PxBitMap::Iterator(mUseBitmap);
	}

private:
	const PxU32				mEltsPerSlab;
	PxU32					mSlabCount;
	PxU32					mLog2EltsPerSlab;
	T**						mFreeList;
	PxU32					mFreeCount;
	T**						mSlabs;
	PxBitMap				mUseBitmap;
};


}
}

#endif
