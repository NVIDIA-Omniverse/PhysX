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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef GU_AABBTREE_BOUNDS_H
#define GU_AABBTREE_BOUNDS_H

#include "common/PxPhysXCommonConfig.h"

namespace physx
{
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
