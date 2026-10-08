// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxcNpCacheStreamPair.h"
#include "foundation/PxUserAllocated.h"
#include "PxcNpMemBlockPool.h"

using namespace physx;

PxcNpCacheStreamPair::PxcNpCacheStreamPair(PxcNpMemBlockPool& blockPool) :
	mBlockPool	(blockPool),
	mBlock		(NULL),
	mUsed		(0)
{
}

// reserve can fail and return null. Read should never fail
PxU8* PxcNpCacheStreamPair::reserve(PxU32 size, bool& sizeTooLarge)
{
	size = (size+15)&~15;

	if(size>PxcNpMemBlock::SIZE)
	{
		sizeTooLarge = true;
		return NULL;
	}

	sizeTooLarge = false;

	if(mBlock == NULL || mUsed + size > PxcNpMemBlock::SIZE)
	{
		mBlock = mBlockPool.acquireNpCacheBlock();
		mUsed = 0;
	}

	PxU8* ptr;
	if(mBlock == NULL)
		ptr = NULL;
	else
	{
		ptr = mBlock->data + mUsed;
		mUsed += size;
	}

	return ptr;
}

