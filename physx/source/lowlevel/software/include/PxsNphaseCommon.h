// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
