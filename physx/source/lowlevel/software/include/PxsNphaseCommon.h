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

#ifndef PXS_NPHASE_COMMON_H
#define PXS_NPHASE_COMMON_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxAssert.h"

namespace physx
{

	struct PxsContactManagerBase
	{
		static const PxU32 NEW_CONTACT_MANAGER_MASK = 0x80000000;
		static const PxU32 MaxBucketBits = 7;

		const PxU32									mBucketId;

		PxsContactManagerBase(const PxU32 bucketId) : mBucketId(bucketId)
		{
			PX_ASSERT(bucketId < (1 << MaxBucketBits));
		}


		PX_FORCE_INLINE PxU32 computeId(const PxU32 index) const { PX_ASSERT(index < PxU32(1 << (32 - (MaxBucketBits - 1)))); return (index << MaxBucketBits) | (mBucketId); }
		static PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 computeIndexFromId(const PxU32 id) { return id >> MaxBucketBits; }
		static PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 computeBucketIndexFromId(const PxU32 id) { return id & ((1 << MaxBucketBits) - 1); }

	private:
		PX_NOCOPY(PxsContactManagerBase)
	};
}

#endif
