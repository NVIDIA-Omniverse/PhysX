// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_FILTERING_H
#define SC_FILTERING_H

#include "PxFiltering.h"
#include "foundation/PxUtilities.h"

namespace physx
{
namespace Sc
{
	struct FilterInfo
	{
		PX_FORCE_INLINE	FilterInfo()							: mPairFlags(0), mFilterFlags8(0), mHasPairID(false)			{}
		PX_FORCE_INLINE	FilterInfo(PxFilterFlags filterFlags)	: mPairFlags(0), mFilterFlags8(filterFlags), mHasPairID(false)	{}

		PX_FORCE_INLINE	PxFilterFlags	getFilterFlags()					const	{ return PxFilterFlags(mFilterFlags8);	}
		PX_FORCE_INLINE	void			setFilterFlags(PxFilterFlags flags)			{ mFilterFlags8 = PxTo8(PxU16(flags));	}
		PX_FORCE_INLINE	void			clearFilterFlags(PxFilterFlag::Enum flag)
										{
											PxFilterFlags flags = getFilterFlags();
											flags.clear(flag);
											setFilterFlags(flags);
										}

		PxPairFlags	mPairFlags;
		PxU8		mFilterFlags8;	// PT: PxFilterFlags but only using 8 bits
		PxU8		mHasPairID;
	};
}
}

#endif
