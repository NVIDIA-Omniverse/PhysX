// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
