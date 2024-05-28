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

#ifndef GU_AABBTREE_NODE_H
#define GU_AABBTREE_NODE_H

#include "common/PxPhysXCommonConfig.h"
#include "foundation/PxBounds3.h"
#include "foundation/PxVecMath.h"
#include "foundation/PxUserAllocated.h"

namespace physx
{
using namespace aos;

namespace Gu
{
	struct BVHNode : public PxUserAllocated
	{
		public:
		PX_FORCE_INLINE					BVHNode()										{}
		PX_FORCE_INLINE					~BVHNode()										{}

		PX_FORCE_INLINE	PxU32			isLeaf()								const	{ return mData&1;			}
		PX_FORCE_INLINE	const PxU32*	getPrimitives(const PxU32* base)		const	{ return base + (mData>>5);	}
		PX_FORCE_INLINE	PxU32*			getPrimitives(PxU32* base)						{ return base + (mData>>5);	}
		PX_FORCE_INLINE	PxU32			getPrimitiveIndex()						const	{ return mData>>5;			}
		PX_FORCE_INLINE	PxU32			getNbPrimitives()						const	{ return (mData>>1)&15;		}
		PX_FORCE_INLINE	PxU32			getPosIndex()							const	{ return mData>>1;			}
		PX_FORCE_INLINE	PxU32			getNegIndex()							const	{ return (mData>>1) + 1;	}
		PX_FORCE_INLINE	const BVHNode*	getPos(const BVHNode* base)				const	{ return base + (mData>>1);									}
		PX_FORCE_INLINE	const BVHNode*	getNeg(const BVHNode* base)				const	{ const BVHNode* P = getPos(base); return P ? P+1 : NULL;	}
		PX_FORCE_INLINE	BVHNode*		getPos(BVHNode* base)							{ return base + (mData >> 1);								}
		PX_FORCE_INLINE	BVHNode*		getNeg(BVHNode* base)							{ BVHNode* P = getPos(base); return P ? P + 1 : NULL;		}

		PX_FORCE_INLINE	PxU32			getNbRuntimePrimitives()				const	{ return (mData>>1)&15;		}
		PX_FORCE_INLINE void			setNbRunTimePrimitives(PxU32 val)
										{
											PX_ASSERT(val<16);
											PxU32 data = mData & ~(15<<1);
											data |= val<<1;
											mData = data;
										}

		PX_FORCE_INLINE	void			getAABBCenterExtentsV(Vec3V* center, Vec3V* extents) const
										{
											const Vec4V minV = V4LoadU(&mBV.minimum.x);
											const Vec4V maxV = V4LoadU(&mBV.maximum.x);

											const float half = 0.5f;
											const FloatV halfV = FLoad(half);

											*extents = Vec3V_From_Vec4V(V4Scale(V4Sub(maxV, minV), halfV));
											*center = Vec3V_From_Vec4V(V4Scale(V4Add(maxV, minV), halfV));
										}

		PX_FORCE_INLINE	void			getAABBCenterExtentsV2(Vec3V* center, Vec3V* extents) const
										{
											const Vec4V minV = V4LoadU(&mBV.minimum.x);
											const Vec4V maxV = V4LoadU(&mBV.maximum.x);

											*extents = Vec3V_From_Vec4V(V4Sub(maxV, minV));
											*center = Vec3V_From_Vec4V(V4Add(maxV, minV));
										}

						PxBounds3		mBV;	// Global bounding-volume enclosing all the node-related primitives
						PxU32			mData;	// 27 bits node or prim index|4 bits #prims|1 bit leaf
	};

} // namespace Gu
}

#endif // GU_AABBTREE_NODE_H
