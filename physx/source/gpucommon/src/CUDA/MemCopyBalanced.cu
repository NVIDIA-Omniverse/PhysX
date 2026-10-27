// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_MEM_COPY_BALANCED_CU
#define PXG_MEM_COPY_BALANCED_CU

#include "foundation/PxMath.h"
#include <assert.h>
#include <stdio.h>
#include "PxgCopyDesc.h"
#include "PxgCommonDefines.h"

using namespace physx;

extern "C" __host__ void initCommonKernels0() {}

template<PxU32 warpsPerBlock>
__device__ void copyBalanced(
	PxgCopyDesc* PX_RESTRICT	desc,						/* Input */
	PxU32						count						/* Input */
)
{
	__shared__ PxgCopyDesc copyDesc[warpsPerBlock];

	if (blockIdx.x < count)
	{
		const PxU32 idxInWarp = threadIdx.x;
		const PxU32 warpIdxInBlock = threadIdx.y;

		if (idxInWarp == 0)
		{
			copyDesc[warpIdxInBlock] = desc[blockIdx.x];
		}

		__syncwarp();

		PxU32* srcPtr = reinterpret_cast<PxU32*>(copyDesc[warpIdxInBlock].source);
		PxU32* dstPtr = reinterpret_cast<PxU32*>(copyDesc[warpIdxInBlock].dest);
		PxU32 size = copyDesc[warpIdxInBlock].bytes / 4; //Size is in bytes, we're reading words...

		PxU32 groupThreadIdx = threadIdx.x + threadIdx.y * WARP_SIZE;

		for (PxU32 a = groupThreadIdx; a < size; a += WARP_SIZE * warpsPerBlock)
		{
			PxU32 sourceVal = srcPtr[a];
			dstPtr[a] = sourceVal;
		}
	}
}

extern "C"
__global__
void MemCopyBalanced(
	PxgCopyDesc* PX_RESTRICT	desc,
	PxU32						count
)
{
	copyBalanced<COPY_KERNEL_WARPS_PER_BLOCK>(
		desc,
		count
		);
}

extern "C" __global__ void clampMaxValue(PxU32* value, const PxU32 maxValue)
{
	if(*value > maxValue)
		*value = maxValue;
}

// temporary clamping function for contact counts: will be generalized in the future.
extern "C" __global__ void clampMaxValues(PxU32* value0, PxU32* value1, PxU32* value2, const PxU32 maxValue)
{
	if (*value0 > maxValue)
		*value0 = maxValue;
	if (*value1 > maxValue)
		*value1 = maxValue;
	if (*value2 > maxValue)
		*value2 = maxValue;
}


#endif 
