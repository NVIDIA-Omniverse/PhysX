// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef CM_PTR_TABLE_H
#define CM_PTR_TABLE_H

#include "foundation/PxConstructor.h"
#include "foundation/PxIO.h"
#include "common/PxPhysXCommonConfig.h"

namespace physx
{
class PxSerializationContext;
class PxDeserializationContext;
class PxOutputStream;

namespace Cm
{
class PtrTableStorageManager
{
	// This will typically be backed by a MultiPool implementation with fallback to the user
	// allocator. For MultiPool, when deallocating we want to know what the previously requested size was
	// so we can release into the right pool

public:

	virtual void**	allocate(PxU32 capacity)							= 0;
	virtual void	deallocate(void** addr, PxU32 originalCapacity)		= 0;

	// whether memory allocated at one capacity can (and should) be safely reused at a different capacity
	// allows realloc-style reuse by clients.

	virtual bool	canReuse(PxU32 originalCapacity, PxU32 newCapacity)		= 0;
protected:
	virtual ~PtrTableStorageManager() {}
};

// specialized class to hold an array of pointers with extrinsic storage management, 
// serialization-compatible with 3.3.1 PtrTable
//
// note that extrinsic storage implies you *must* clear the table before the destructor runs
//
// capacity is implicit: 
// if the memory is not owned (i.e. came from deserialization) then the capacity is exactly mCount
// else if mCount==0, capacity is 0
// else the capacity is the power of 2 >= mCount
// 
// one implication of this is that if we want to add or remove a pointer from unowned memory, we always realloc
struct PX_PHYSX_COMMON_API PtrTable
{
	PtrTable();
	~PtrTable();

	void	add(void* ptr, PtrTableStorageManager& sm);
	void	replaceWithLast(PxU32 index, PtrTableStorageManager& sm);
	void	clear(PtrTableStorageManager& sm);

	PxU32	find(const void* ptr) const;

	PX_FORCE_INLINE PxU32		getCount()	const	{ return mCount; }
	PX_FORCE_INLINE	void*const*	getPtrs()	const	{ return mCount == 1 ? &mSingle : mList;	}
	PX_FORCE_INLINE	void**		getPtrs()			{ return mCount == 1 ? &mSingle : mList;	}

	// SERIALIZATION

	// 3.3.1 compatibility fixup: this implementation ALWAYS sets 'ownsMemory' if the size is 0 or 1
	PtrTable(const PxEMPTY)
	{
		mOwnsMemory = mCount<2;
		if(mCount == 0)
			mList = NULL;
	}

	void	exportExtraData(PxSerializationContext& stream);
	void	importExtraData(PxDeserializationContext& context);

private:
	void realloc(PxU32 oldCapacity, PxU32 newCapacity, PtrTableStorageManager& sm);

	union
	{
		void*	mSingle;
		void**	mList;
	};

	PxU16	mCount;
	bool	mOwnsMemory;
	bool	mBufferUsed;	// dark magic in serialization requires this, otherwise redundant because it's logically equivalent to mCount == 1.
public:
	PxU32	mFreeSlot;		// PT: padding bytes on x64
};

} // namespace Cm

}

#endif
