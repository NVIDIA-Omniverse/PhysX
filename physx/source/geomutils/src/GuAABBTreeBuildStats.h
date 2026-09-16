// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_AABBTREE_BUILD_STATS_H
#define GU_AABBTREE_BUILD_STATS_H

#include "common/PxPhysXCommonConfig.h"

namespace physx
{
namespace Gu
{
	//! Contains AABB-tree build statistics
	struct PX_PHYSX_COMMON_API BuildStats
	{
								BuildStats() : mCount(0), mTotalPrims(0) {}

						PxU32	mCount;			//!< Number of nodes created
						PxU32	mTotalPrims;	//!< Total accumulated number of primitives. Should be much higher than the source
												//!< number of prims, since it accumulates all prims covered by each node (i.e. internal
												//!< nodes too, not just leaf ones)

		// PT: everything's public so consider dropping these
		PX_FORCE_INLINE	void	reset()					{ mCount = mTotalPrims = 0;	}
		PX_FORCE_INLINE	void	setCount(PxU32 nb)		{ mCount = nb;				}
		PX_FORCE_INLINE	void	increaseCount(PxU32 nb)	{ mCount += nb;				}
		PX_FORCE_INLINE	PxU32	getCount()		const	{ return mCount;			}
	};

} // namespace Gu
}

#endif // GU_AABBTREE_BUILD_STATS_H
