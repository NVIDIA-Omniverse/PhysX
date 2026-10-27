// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXC_NP_CACHE_STREAM_PAIR_H
#define PXC_NP_CACHE_STREAM_PAIR_H

#include "foundation/PxSimpleTypes.h"
#include "PxPhysXConfig.h"
#include "PxcNpMemBlockPool.h"

namespace physx
{
	struct PxcNpCacheStreamPair
	{
										PX_NOCOPY(PxcNpCacheStreamPair)
	public:
										PxcNpCacheStreamPair(PxcNpMemBlockPool& blockPool);

					// reserve can fail and return null.
					PxU8*				reserve(PxU32 byteCount, bool& sizeTooLarge);
	PX_FORCE_INLINE	void				reset()
										{
											mBlock = NULL;
											mUsed = 0;
										}
	private:
					PxcNpMemBlockPool&	mBlockPool;
					PxcNpMemBlock*		mBlock;
					PxU32				mUsed;
	};
}

#endif
