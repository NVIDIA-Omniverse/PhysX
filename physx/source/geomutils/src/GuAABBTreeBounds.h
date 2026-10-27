// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_AABBTREE_BOUNDS_H
#define GU_AABBTREE_BOUNDS_H

#include "common/PxPhysXCommonConfig.h"

namespace physx
{
class PxBounds3;
namespace Gu
{
	class PX_PHYSX_COMMON_API AABBTreeBounds
	{
		public:
											AABBTreeBounds() : mBounds(NULL), mUserAllocated(false)	{}
											~AABBTreeBounds()										{ release();	}

						void				init(PxU32 nbBounds, const PxBounds3* bounds=NULL);
						void				resize(PxU32 newSize, PxU32 previousSize);
						void				release();

		PX_FORCE_INLINE	PxBounds3*			getBounds()			{ return mBounds;	}
		PX_FORCE_INLINE	const PxBounds3*	getBounds()	const	{ return mBounds;	}

		PX_FORCE_INLINE	void				moveFrom(AABBTreeBounds& source)
											{
												mBounds = source.mBounds;
												source.mBounds = NULL;
											}

		PX_FORCE_INLINE	void				takeOwnership()					{ mUserAllocated = true;					}
		PX_FORCE_INLINE	bool				ownsMemory()	const			{ return mUserAllocated==false;				}
		PX_FORCE_INLINE	void				setBounds(PxBounds3* bounds)	{ mBounds = bounds; mUserAllocated=true;	}

		private:
						PxBounds3*			mBounds;
						PxU32				mUserAllocated;
	};

} // namespace Gu
}

#endif // GU_AABBTREE_BOUNDS_H
