// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef CM_RADIX_SORT_H
#define CM_RADIX_SORT_H

#include "common/PxPhysXCommonConfig.h"

namespace physx
{
namespace Cm
{
	enum RadixHint
	{
		RADIX_SIGNED,		//!< Input values are signed
		RADIX_UNSIGNED,		//!< Input values are unsigned

		RADIX_FORCE_DWORD = 0x7fffffff
	};

#define INVALIDATE_RANKS	mCurrentSize|=0x80000000
#define VALIDATE_RANKS		mCurrentSize&=0x7fffffff
#define CURRENT_SIZE		(mCurrentSize&0x7fffffff)
#define INVALID_RANKS		(mCurrentSize&0x80000000)

	class PX_PHYSX_COMMON_API RadixSort
	{
										PX_NOCOPY(RadixSort)
		public:
										RadixSort();
		virtual							~RadixSort();
		// Sorting methods
						RadixSort&		Sort(const PxU32* input, PxU32 nb, RadixHint hint=RADIX_SIGNED);
						RadixSort&		Sort(const float* input, PxU32 nb);

		//! Access to results. mRanks is a list of indices in sorted order, i.e. in the order you may further process your data
		PX_FORCE_INLINE	const PxU32*	GetRanks()			const	{ return mRanks;		}

		//! mIndices2 gets trashed on calling the sort routine, but otherwise you can recycle it the way you want.
		PX_FORCE_INLINE	PxU32*			GetRecyclable()		const	{ return mRanks2;		}

		//! Returns the total number of calls to the radix sorter.
		PX_FORCE_INLINE	PxU32			GetNbTotalCalls()	const	{ return mTotalCalls;	}
		//! Returns the number of eraly exits due to temporal coherence.
		PX_FORCE_INLINE	PxU32			GetNbHits()			const	{ return mNbHits;		}

		PX_FORCE_INLINE	void			invalidateRanks()			{ INVALIDATE_RANKS;		}

						bool			SetBuffers(PxU32* ranks0, PxU32* ranks1, PxU32* histogram1024, PxU32** links256);
		protected:
						PxU32			mCurrentSize;		//!< Current size of the indices list
						PxU32*			mRanks;				//!< Two lists, swapped each pass
						PxU32*			mRanks2;
						PxU32*			mHistogram1024;
						PxU32**			mLinks256;
		// Stats
						PxU32			mTotalCalls;		//!< Total number of calls to the sort routine
						PxU32			mNbHits;			//!< Number of early exits due to coherence

						// Stack-radix
						bool			mDeleteRanks;		//!<
	};

	#define StackRadixSort(name, ranks0, ranks1)	\
		RadixSort name;								\
		PxU32 histogramBuffer[1024];				\
		PxU32* linksBuffer[256];					\
		name.SetBuffers(ranks0, ranks1, histogramBuffer, linksBuffer);

	class PX_PHYSX_COMMON_API RadixSortBuffered : public RadixSort
	{
	public:
							RadixSortBuffered();
							~RadixSortBuffered();

		void				reset();

		RadixSortBuffered&	Sort(const PxU32* input, PxU32 nb, RadixHint hint=RADIX_SIGNED);
		RadixSortBuffered&	Sort(const float* input, PxU32 nb);

	private:
							RadixSortBuffered(const RadixSortBuffered& object);
							RadixSortBuffered& operator=(const RadixSortBuffered& object);

		// Internal methods
		void				CheckResize(PxU32 nb);
		bool				Resize(PxU32 nb);
	};
}
}

#endif // CM_RADIX_SORT_H
