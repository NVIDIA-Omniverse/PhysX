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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef BP_VOLUME_DATA_H
#define BP_VOLUME_DATA_H

#include "PxvConfig.h"
#include "foundation/PxAssert.h"

namespace physx
{
	namespace Bp
	{
		typedef PxU32 AggregateHandle;	// PT: currently an index in mAggregates array

		struct ElementType
		{
			enum Enum
			{
				eSHAPE = 0,
				eTRIGGER,

				eCOUNT
			};
		};
		PX_COMPILE_TIME_ASSERT(ElementType::eCOUNT <= 4);			// 2 bits reserved for type

		#define PX_CUDA_INLINE	PX_CUDA_CALLABLE PX_FORCE_INLINE

		struct VolumeData
		{
			PX_CUDA_INLINE	void				reset()
												{
													mAggregate = PX_INVALID_U32;
													mUserData = NULL;
												}

			PX_CUDA_INLINE	void				setSingleActor()		{ mAggregate = PX_INVALID_U32;			}
			PX_CUDA_INLINE	bool				isSingleActor()	const	{ return mAggregate == PX_INVALID_U32;	}

			PX_CUDA_INLINE	void				setUserData(void* userData)
												{
//													PX_ASSERT(!(size_t(userData) & 3));
													mUserData = userData;
												}

			PX_CUDA_INLINE void*				getUserData() const
												{
													return reinterpret_cast<void*>(size_t(mUserData)& (~size_t(3)));
												}

			PX_CUDA_INLINE void					setVolumeType(ElementType::Enum volumeType)
												{
													PX_ASSERT(volumeType < 2);
													mUserData = reinterpret_cast<void*>(size_t(getUserData()) | size_t(volumeType));
												}

			PX_CUDA_INLINE ElementType::Enum	getVolumeType() const
												{
													return ElementType::Enum(size_t(mUserData) & 3);
												}

			PX_CUDA_INLINE	void				setAggregate(AggregateHandle handle)
												{
													PX_ASSERT(handle != PX_INVALID_U32);
													mAggregate = (handle << 1) | 1;
												}

			PX_CUDA_INLINE	bool				isAggregate()	const { return !isSingleActor() && ((mAggregate & 1) != 0); }

			PX_CUDA_INLINE	void				setAggregated(AggregateHandle handle)
												{
													PX_ASSERT(handle != PX_INVALID_U32);
													mAggregate = (handle << 1) | 0;
												}

			PX_CUDA_INLINE	bool				isAggregated()	const
												{
													return !isSingleActor() && ((mAggregate & 1) == 0);
												}

			PX_CUDA_INLINE	AggregateHandle		getAggregateOwner()	const { return mAggregate >> 1; }
			PX_CUDA_INLINE	AggregateHandle		getAggregate()		const { return mAggregate >> 1; }

		private:
							void*				mUserData;	// PT: in PhysX this is an Sc::ElementSim ptr
			// PT: TODO: consider moving this to a separate array, which wouldn't be allocated at all for people not using aggregates.
			// PT: current encoding:
			// aggregate == PX_INVALID_U32 => single actor
			// aggregate != PX_INVALID_U32 => aggregate index<<1|LSB. LSB==1 for aggregates, LSB==0 for aggregated actors.
							AggregateHandle		mAggregate;
		};

	}
}

#endif

