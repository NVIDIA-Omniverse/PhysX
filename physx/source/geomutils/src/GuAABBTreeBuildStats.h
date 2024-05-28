// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

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
