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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

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
