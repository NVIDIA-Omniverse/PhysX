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

#ifndef PXG_MEM_COPY_BALANCED_CU
#define PXG_MEM_COPY_BALANCED_CU

#include "foundation/PxMath.h"
#include <assert.h>
#include <stdio.h>
#include "PxgCopyManager.h"
#include "PxgCommonDefines.h"

using namespace physx;

extern "C" __host__ void initCommonKernels0() {}

template<PxU32 warpsPerBlock>
__device__ void copyBalanced(
	PxgCopyManager::CopyDesc* PX_RESTRICT	desc,						/* Input */
	PxU32									count						/* Input */
)
{
	__shared__ PxgCopyManager::CopyDesc copyDesc[warpsPerBlock];

	if (blockIdx.x < count)
	{
		const PxU32 idxInWarp = threadIdx.x;
		const PxU32 warpIdxInBlock = threadIdx.y;

		if (idxInWarp == 0)
		{
			PxgCopyManager::CopyDesc d = desc[blockIdx.x];
			copyDesc[warpIdxInBlock] = d;
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
	PxgCopyManager::CopyDesc* PX_RESTRICT	desc,
	PxU32								count
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
