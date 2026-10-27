// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_CONTACT_REPORT_BUFFER_H
#define SC_CONTACT_REPORT_BUFFER_H

#include "common/PxProfileZone.h"

namespace physx
{
	namespace Sc
	{
		class ContactReportBuffer
		{
		public:
			PX_FORCE_INLINE ContactReportBuffer(PxU32 initialSize, bool noResizeAllowed)
				: mBuffer(NULL)
				 ,mCurrentBufferIndex(0)
				 ,mCurrentBufferSize(initialSize)
				 ,mDefaultBufferSize(initialSize)
				 ,mLastBufferIndex(0)
				 ,mAllocationLocked(noResizeAllowed)
			{
				mBuffer = allocateBuffer(initialSize);
				PX_ASSERT(mBuffer);
			}

			~ContactReportBuffer()
			{
				PX_FREE(mBuffer);
			}

			PX_FORCE_INLINE void					reset();
			PX_FORCE_INLINE void					flush();

			PX_FORCE_INLINE PxU8*					allocateNotThreadSafe(PxU32 size, PxU32& index, PxU32 alignment = 16);
			PX_FORCE_INLINE PxU8*					reallocateNotThreadSafe(PxU32 size, PxU32& index, PxU32 alignment = 16, PxU32 lastIndex = 0xFFFFFFFF);
			PX_FORCE_INLINE	PxU8*					getData(const PxU32& index) const { return mBuffer+index; }

			PX_FORCE_INLINE PxU32					getDefaultBufferSize() const {return mDefaultBufferSize;}

		private:
			PX_FORCE_INLINE PxU8* allocateBuffer(PxU32 size);

		private:
			PxU8*			mBuffer;
			PxU32			mCurrentBufferIndex;
			PxU32			mCurrentBufferSize;
			PxU32			mDefaultBufferSize;
			PxU32			mLastBufferIndex;
			bool			mAllocationLocked;
		};

	} // namespace Sc

	//////////////////////////////////////////////////////////////////////////

	PX_FORCE_INLINE void Sc::ContactReportBuffer::reset()
	{
		mCurrentBufferIndex = 0;	
		mLastBufferIndex = 0xFFFFFFFF;
	}

	//////////////////////////////////////////////////////////////////////////

	void Sc::ContactReportBuffer::flush()
	{
		mCurrentBufferIndex = 0;
		mLastBufferIndex = 0xFFFFFFFF;

		if(mCurrentBufferSize != mDefaultBufferSize)
		{
			PX_FREE(mBuffer);

			mBuffer = allocateBuffer(mDefaultBufferSize);
			PX_ASSERT(mBuffer);

			mCurrentBufferSize = mDefaultBufferSize;			
		}
	}

	//////////////////////////////////////////////////////////////////////////

	PxU8* Sc::ContactReportBuffer::allocateNotThreadSafe(PxU32 size, PxU32& index ,PxU32 alignment/* =16 */)
	{
		PX_ASSERT(PxIsPowerOfTwo(alignment));
		
		// padding for alignment
		PxU32 pad = ((mCurrentBufferIndex+alignment-1)&~(alignment-1)) - mCurrentBufferIndex;

		index = mCurrentBufferIndex + pad;

		if (index + size > mCurrentBufferSize)
		{
			PX_PROFILE_ZONE("ContactReportBuffer::Resize", 0);
			if(mAllocationLocked)
				return NULL;

			PxU32 oldBufferSize = mCurrentBufferSize;
			while(index + size > mCurrentBufferSize)
			{
				mCurrentBufferSize *= 2;
			}
			
			PxU8* tempBuffer = allocateBuffer(mCurrentBufferSize);

			PxMemCopy(tempBuffer,mBuffer,oldBufferSize);

			PX_FREE(mBuffer);

			mBuffer = tempBuffer;
		}

		
		PxU8* ptr = mBuffer + index;
		mLastBufferIndex = index;
		PX_ASSERT((size_t(ptr)&(alignment-1)) == 0);	
		mCurrentBufferIndex += size + pad;
		return ptr;
	}

	//////////////////////////////////////////////////////////////////////////

	PxU8* Sc::ContactReportBuffer::reallocateNotThreadSafe(PxU32 size, PxU32& index ,PxU32 alignment/* =16 */, PxU32 lastIndex)
	{
		if(lastIndex != mLastBufferIndex)
		{
			return allocateNotThreadSafe(size,index,alignment);
		}
		else
		{
			mCurrentBufferIndex = mLastBufferIndex;
			return allocateNotThreadSafe(size,index,alignment);
		}
	}

	//////////////////////////////////////////////////////////////////////////

	PX_FORCE_INLINE PxU8* Sc::ContactReportBuffer::allocateBuffer(PxU32 size)
	{
		return (static_cast<PxU8*>(PX_ALLOC(size, "ContactReportBuffer")));
	}

} // namespace physx

#endif

