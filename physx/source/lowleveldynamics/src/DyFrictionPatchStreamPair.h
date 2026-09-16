// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_FRICTION_PATCH_STREAM_PAIR_H
#define DY_FRICTION_PATCH_STREAM_PAIR_H

#include "foundation/PxSimpleTypes.h"
#include "PxPhysXConfig.h"
#include "foundation/PxMutex.h"
#include "foundation/PxArray.h"

// Each narrow phase thread has an input stream of friction patches from the
// previous frame and an output stream of friction patches which will be
// saved for next frame. The patches persist for exactly one frame at which
// point they get thrown away.


// There is a stream pair per thread. A contact callback reserves space
// for its friction patches and gets a cookie in return that can stash
// for next frame. Cookies are valid for one frame only.
//
// note that all friction patches reserved are guaranteed to be contiguous;
// this might turn out to be a bit inefficient if we often have a large
// number of friction patches

#include "PxcNpMemBlockPool.h"

namespace physx
{

class FrictionPatchStreamPair
{
public:
	FrictionPatchStreamPair(PxcNpMemBlockPool& blockPool);

	// reserve can fail and return null. Read should never fail
	template<class FrictionPatch>
	FrictionPatch*		reserve(const PxU32 size);

	template<class FrictionPatch>
	const FrictionPatch* findInputPatches(const PxU8* ptr) const;
	void					reset();

	PxcNpMemBlockPool& getBlockPool() { return mBlockPool;}
private:
	PxcNpMemBlockPool&	mBlockPool;
	PxcNpMemBlock*		mBlock;
	PxU32				mUsed;

	FrictionPatchStreamPair& operator=(const FrictionPatchStreamPair&);
};

PX_FORCE_INLINE FrictionPatchStreamPair::FrictionPatchStreamPair(PxcNpMemBlockPool& blockPool):
  mBlockPool(blockPool), mBlock(NULL), mUsed(0)
{
}

PX_FORCE_INLINE void FrictionPatchStreamPair::reset()
{
	mBlock = NULL;
	mUsed = 0;
}

// reserve can fail and return null. Read should never fail
template <class FrictionPatch>
FrictionPatch* FrictionPatchStreamPair::reserve(const PxU32 size)
{
	if(size>PxcNpMemBlock::SIZE)
	{
		return reinterpret_cast<FrictionPatch*>(-1);
	}

	PX_ASSERT(size <= PxcNpMemBlock::SIZE);

	FrictionPatch* ptr = NULL;

	if(mBlock == NULL || mUsed + size > PxcNpMemBlock::SIZE)
	{
		mBlock = mBlockPool.acquireFrictionBlock();
		mUsed = 0;
	}

	if(mBlock)
	{
		ptr = reinterpret_cast<FrictionPatch*>(mBlock->data+mUsed);
		mUsed += size;
	}

	return ptr;
}

template <class FrictionPatch>
const FrictionPatch* FrictionPatchStreamPair::findInputPatches(const PxU8* ptr) const
{
	return reinterpret_cast<const FrictionPatch*>(ptr);
}

}

#endif
