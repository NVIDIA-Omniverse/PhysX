// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef NP_PTR_TABLE_STORAGE_MANAGER_H
#define NP_PTR_TABLE_STORAGE_MANAGER_H

#include "foundation/PxMutex.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxBitUtils.h"
#include "CmPtrTable.h"

namespace physx
{
class NpPtrTableStorageManager : public Cm::PtrTableStorageManager, public PxUserAllocated
{
	PX_NOCOPY(NpPtrTableStorageManager)

public:

	NpPtrTableStorageManager() {}
	~NpPtrTableStorageManager() {}

	// PtrTableStorageManager
	virtual	void**	allocate(PxU32 capacity) PX_OVERRIDE
	{
		PX_ASSERT(PxIsPowerOfTwo(capacity));

		PxMutex::ScopedLock lock(mMutex);

		return capacity<=4  ?	reinterpret_cast<void**>(mPool4.construct())
			 : capacity<=16 ?	reinterpret_cast<void**>(mPool16.construct())
			 : capacity<=64 ?	reinterpret_cast<void**>(mPool64.construct())
			 :					reinterpret_cast<void**>(PX_ALLOC(capacity*sizeof(void*), "CmPtrTable pointer array"));
	}

	virtual	void	deallocate(void** addr, PxU32 capacity) PX_OVERRIDE
	{
		PX_ASSERT(PxIsPowerOfTwo(capacity));

		PxMutex::ScopedLock lock(mMutex);

		if(capacity<=4)			mPool4.destroy(reinterpret_cast< PtrBlock<4>*>(addr));
		else if(capacity<=16)	mPool16.destroy(reinterpret_cast< PtrBlock<16>*>(addr));
		else if(capacity<=64)	mPool64.destroy(reinterpret_cast< PtrBlock<64>*>(addr));
		else					PX_FREE(addr);
	}

	// originalCapacity is the only way we know which pool the alloc request belongs to,
	// so if those are no longer going to match, we need to realloc.

	virtual	bool canReuse(PxU32 originalCapacity, PxU32 newCapacity) PX_OVERRIDE
	{
		PX_ASSERT(PxIsPowerOfTwo(originalCapacity));
		PX_ASSERT(PxIsPowerOfTwo(newCapacity));

		return poolId(originalCapacity) == poolId(newCapacity) && newCapacity<=64;
	}
	//~PtrTableStorageManager

private:
	PxMutex mMutex;

	int poolId(PxU32 size)
	{
		return size<=4	? 0
			 : size<=16 ? 1
			 : size<=64 ? 2
			 : 3;
	}

	template<int N> class PtrBlock { void* ptr[N]; };

	PxPool2<PtrBlock<4>, 4096 >		mPool4; 
	PxPool2<PtrBlock<16>, 4096 >	mPool16;
	PxPool2<PtrBlock<64>, 4096 >	mPool64;
};

}
#endif
