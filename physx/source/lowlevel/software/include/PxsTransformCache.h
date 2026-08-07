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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXS_TRANSFORM_CACHE_H
#define PXS_TRANSFORM_CACHE_H

#include "CmIDPool.h"
#include "PxsHeapStats.h"
#include "PxsCachedTransform.h"
#include "foundation/PxBitMap.h"
#include "foundation/PxUserAllocated.h"
#include "CmPinnableArray.h"

#define PX_DEFAULT_CACHE_SIZE 512 // this is currently not in use

namespace physx
{

	class PxsTransformCache : public PxUserAllocated
	{
		typedef PxU32 RefCountType;

	public:
		PxsTransformCache(Cm::VirtualAllocatorCallback& alloc, Cm::PinnableAllocatorFallback::Enum fallback) :
			mTransformCache(alloc, PxsHeapStats::eNARROWPHASE, fallback), mHasAnythingChanged(true), mAllocFailed(false)
		{
			/*mTransformCache.reserve(PX_DEFAULT_CACHE_SIZE);
			mTransformCache.forceSize_Unsafe(PX_DEFAULT_CACHE_SIZE);*/
			mUsedSize = 0;
		}

		bool initEntry(PxU32 index)
		{
			PxU32 oldCapacity = mTransformCache.capacity();
			if (index >= oldCapacity)
			{
				PxU32 newCapacity = PxNextPowerOfTwo(index);
				if(!mTransformCache.reserve(newCapacity))
				{
					mAllocFailed = true;
					return false;
				}
				mTransformCache.forceSize_Unsafe(newCapacity);
			}
			mUsedSize = PxMax(mUsedSize, index + 1u);
			return true;
		}

		PX_FORCE_INLINE void setTransformCache(const PxTransform& transform, PxU32 flags, PxU32 index)
		{
			mTransformCache[index].transform = transform;
			mTransformCache[index].flags = flags;
			mHasAnythingChanged = true;
		}

		PX_FORCE_INLINE const PxsCachedTransform& getTransformCache(PxU32 index) const
		{
			return mTransformCache[index];
		}

		PX_FORCE_INLINE PxsCachedTransform& getTransformCache(PxU32 index)
		{
			return mTransformCache[index];
		}

		PX_FORCE_INLINE void shiftTransforms(const PxVec3& shift)
		{
			for (PxU32 i = 0; i < mTransformCache.capacity(); i++)
			{
				mTransformCache[i].transform.p += shift;
			}
			mHasAnythingChanged = true;
		}

		PX_FORCE_INLINE PxU32 getTotalSize() const
		{
			return mUsedSize;
		}

		PX_FORCE_INLINE const PxsCachedTransform* getTransforms() const
		{
			return mTransformCache.begin();
		}

		PX_FORCE_INLINE PxsCachedTransform* getTransforms()
		{
			return mTransformCache.begin();
		}

		PX_FORCE_INLINE	void resetChangedState()	{ mHasAnythingChanged = false;	}
		PX_FORCE_INLINE	void setChangedState()		{ mHasAnythingChanged = true;	}
		PX_FORCE_INLINE	bool hasChanged()	const	{ return mHasAnythingChanged;	}

		PX_FORCE_INLINE bool hadAllocationFailure()	const	{ return mAllocFailed;	}

	protected:
		Cm::PinnableArray<PxsCachedTransform>	mTransformCache;

	private:
		PxU32									mUsedSize;
		bool									mHasAnythingChanged;
		bool									mAllocFailed;
	};
}

#endif
