// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXC_NP_MEM_BLOCK_POOL_H
#define PXC_NP_MEM_BLOCK_POOL_H

#include "PxPhysXConfig.h"
#include "foundation/PxArray.h"
#include "foundation/PxMutex.h"

namespace physx
{
class PxcScratchAllocator;

struct PxcNpMemBlock
{
	enum
	{
		SIZE = 16384
	};
	PxU8 data[SIZE];
};

typedef PxArray<PxcNpMemBlock*> PxcNpMemBlockArray;

class PxcNpMemBlockPool
{
	PX_NOCOPY(PxcNpMemBlockPool)
public:
	PxcNpMemBlockPool(PxcScratchAllocator& allocator);
	~PxcNpMemBlockPool();

	void			init(PxU32 initial16KDataBlocks, PxU32 maxBlocks);
	void			flush();
	void			setBlockCount(PxU32 count);
	PxU32			getUsedBlockCount() const;
	PxU32			getMaxUsedBlockCount() const;
	PxU32			getPeakConstraintBlockCount() const;
	void			releaseUnusedBlocks();

	PxcNpMemBlock*	acquireConstraintBlock();
	PxcNpMemBlock*	acquireConstraintBlock(PxcNpMemBlockArray& memBlocks);
	PxcNpMemBlock*	acquireContactBlock();
	PxcNpMemBlock*	acquireFrictionBlock();
	PxcNpMemBlock*	acquireNpCacheBlock();

	PxU8*			acquireExceptionalConstraintMemory(PxU32 size);

	void			acquireConstraintMemory();
	void			releaseConstraintMemory();
	void			releaseConstraintBlocks(PxcNpMemBlockArray& memBlocks);
	void			releaseContacts();
	void			swapFrictionStreams();
	void			swapNpCacheStreams();

	void			flushUnused();
	
private:

	PxMutex					mLock;
	PxcNpMemBlockArray		mConstraints;
	PxcNpMemBlockArray		mContacts[2];
	PxcNpMemBlockArray		mFriction[2];
	PxcNpMemBlockArray		mNpCache[2];
	PxcNpMemBlockArray		mScratchBlocks;
	PxArray<PxU8*>			mExceptionalConstraints;

	PxcNpMemBlockArray		mUnused;

	PxU32					mNpCacheActiveStream;
	PxU32					mFrictionActiveStream;
	PxU32					mCCDCacheActiveStream;
	PxU32					mContactIndex;
	PxU32					mAllocatedBlocks;
	PxU32					mMaxBlocks;
	PxU32					mInitialBlocks;
	PxU32					mUsedBlocks;
	PxU32					mMaxUsedBlocks;
	PxcNpMemBlock*			mScratchBlockAddr;
	PxU32					mNbScratchBlocks;
	PxcScratchAllocator&	mScratchAllocator;

	PxU32					mPeakConstraintAllocations;
	PxU32					mConstraintAllocations;

	PxcNpMemBlock*	acquire(PxcNpMemBlockArray& trackingArray, PxU32* allocationCount = NULL, PxU32* peakAllocationCount = NULL, bool isScratchAllocation = false);
	void			release(PxcNpMemBlockArray& deadArray, PxU32* allocationCount = NULL);
};

}

#endif
