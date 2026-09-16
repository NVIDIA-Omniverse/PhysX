// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_EDGECACHE_H
#define GU_EDGECACHE_H

#include "foundation/PxMemory.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxHash.h"

namespace physx
{
namespace Gu
{
	class EdgeCache
	{
#define NUM_EDGES_IN_CACHE 64		//must be power of 2.	32 lines result in 10% extra work (due to cache misses), 64 lines in 6% extra work, 128 lines in 4%.
	public:
		EdgeCache()
		{
			PxMemZero(cacheLines, NUM_EDGES_IN_CACHE*sizeof(CacheLine));
		}

		PxU32 hash(PxU32 key)	const
		{
			return (NUM_EDGES_IN_CACHE - 1) & PxComputeHash(key);		//Only a 16 bit hash would be needed here.
		}

		bool isInCache(PxU8 vertex0, PxU8 vertex1)
		{
			PX_ASSERT(vertex1 >= vertex0);
			PxU16 key = PxU16((vertex0 << 8) | vertex1);
			PxU32 h = hash(key);
			CacheLine& cl = cacheLines[h];
			if (cl.fullKey == key)
			{
				return true;
			}
			else	//cache the line now as it's about to be processed
			{
				cl.fullKey = key;
				return false;
			}
		}

	private:
		struct CacheLine
		{
			PxU16 fullKey;
		};
		CacheLine cacheLines[NUM_EDGES_IN_CACHE];
#undef NUM_EDGES_IN_CACHE
	};
}

}

#endif

