// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_SHARED_QUEUE_ENTRY_POOL_H
#define EXT_SHARED_QUEUE_ENTRY_POOL_H

#include "foundation/PxAllocator.h"
#include "foundation/PxArray.h"
#include "foundation/PxSList.h"

namespace physx
{
namespace Ext
{
	class SharedQueueEntry : public PxSListEntry
	{
	public:
		SharedQueueEntry(void* objectRef) : mObjectRef(objectRef), mPooledEntry(false) {}
		SharedQueueEntry() : mObjectRef(NULL), mPooledEntry(true) {}

	public:
		void* mObjectRef;
		bool mPooledEntry; // True if the entry was preallocated in a pool
	};

#if PX_VC
#pragma warning(push)
#pragma warning(disable:4324)	// Padding was added at the end of a structure because of a __declspec(align) value.
#endif							// Because of the SList member I assume*/

	template<class Alloc = typename PxAllocatorTraits<SharedQueueEntry>::Type >
	class SharedQueueEntryPool : private Alloc
	{
	public:
		SharedQueueEntryPool(PxU32 poolSize, const Alloc& alloc = Alloc("SharedQueueEntryPool"));
		~SharedQueueEntryPool();

		SharedQueueEntry* getEntry(void* objectRef);
		void putEntry(SharedQueueEntry& entry);

	private:
		SharedQueueEntry*	mTaskEntryPool;
		PxSList				mTaskEntryPtrPool;
	};

#if PX_VC
#pragma warning(pop)
#endif

template <class Alloc>
SharedQueueEntryPool<Alloc>::SharedQueueEntryPool(PxU32 poolSize, const Alloc& alloc)
	: Alloc(alloc)
{
	PxAlignedAllocator<PX_SLIST_ALIGNMENT, Alloc> alignedAlloc("SharedQueueEntryPool");

	mTaskEntryPool = poolSize ? reinterpret_cast<SharedQueueEntry*>(alignedAlloc.allocate(sizeof(SharedQueueEntry) * poolSize, PX_FL)) : NULL;

	if (mTaskEntryPool)
	{
		for(PxU32 i=0; i < poolSize; i++)
		{
			PX_ASSERT((size_t(&mTaskEntryPool[i]) & (PX_SLIST_ALIGNMENT-1)) == 0);  // The SList entry must be aligned according to PX_SLIST_ALIGNMENT

			PX_PLACEMENT_NEW(&mTaskEntryPool[i], SharedQueueEntry)();
			PX_ASSERT(mTaskEntryPool[i].mPooledEntry == true);
			mTaskEntryPtrPool.push(mTaskEntryPool[i]);
		}
	}
}


template <class Alloc>
SharedQueueEntryPool<Alloc>::~SharedQueueEntryPool()
{
	if (mTaskEntryPool)
	{
		PxAlignedAllocator<PX_SLIST_ALIGNMENT, Alloc> alignedAlloc("SharedQueueEntryPool");
		alignedAlloc.deallocate(mTaskEntryPool);
	}
}


template <class Alloc>
SharedQueueEntry* SharedQueueEntryPool<Alloc>::getEntry(void* objectRef)
{
	SharedQueueEntry* e = static_cast<SharedQueueEntry*>(mTaskEntryPtrPool.pop());
	if (e)
	{
		PX_ASSERT(e->mPooledEntry == true);
		e->mObjectRef = objectRef;
		return e;
	}
	else
	{
		PxAlignedAllocator<PX_SLIST_ALIGNMENT, Alloc> alignedAlloc;
		e = reinterpret_cast<SharedQueueEntry*>(alignedAlloc.allocate(sizeof(SharedQueueEntry), PX_FL));
		if (e)
		{
			PX_PLACEMENT_NEW(e, SharedQueueEntry)(objectRef);
			PX_ASSERT(e->mPooledEntry == false);
		}

		return e;
	}
}


template <class Alloc>
void SharedQueueEntryPool<Alloc>::putEntry(SharedQueueEntry& entry)
{
	if (entry.mPooledEntry)
	{
		entry.mObjectRef = NULL;
		mTaskEntryPtrPool.push(entry);
	}
	else
	{
		PxAlignedAllocator<PX_SLIST_ALIGNMENT, Alloc> alignedAlloc;
		alignedAlloc.deallocate(&entry);
	}
}

} // namespace Ext
} // namespace physx

#endif
