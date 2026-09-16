// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef __CU_COPY_CUH__
#define __CU_COPY_CUH__

#include "foundation/PxSimpleTypes.h"
#include "PxgCommonDefines.h"
#include "cutil_math.h"
#include <assert.h>

template<typename T>
__device__ void warpCopy(T* dest, const T* source, const uint totalSize)
{
	assert((size_t(dest) & (alignof(T)-1)) == 0);
	assert((size_t(source) & (alignof(T)-1)) == 0);
	assert(totalSize % sizeof(T) == 0);

	const uint idxInWarp = threadIdx.x & (WARP_SIZE - 1);
	const uint requiredThreads = totalSize / sizeof(T);

	for (uint i = idxInWarp; i < requiredThreads; i += WARP_SIZE)
	{
		dest[i] = source[i];
	}
}

template<typename T>
__device__ void warpCopy(T* dest, const T& value, const uint totalSize)
{
	assert(((size_t(dest) & (alignof(T)-1)) == 0));
	assert(totalSize % sizeof(T) == 0);

	const uint idxInWarp = threadIdx.x & (WARP_SIZE - 1);
	const uint requiredThreads = totalSize / sizeof(T);

	for (uint i = idxInWarp; i < requiredThreads; i += WARP_SIZE)
	{
		dest[i] = value;
	}
}

template<typename T>
__device__ void blockCopy(T* dest, const T* source, const uint totalSize)
{
	assert((size_t(dest) & (alignof(T)-1)) == 0);
	assert((size_t(source) & (alignof(T)-1)) == 0);
	assert(totalSize % sizeof(T) == 0);

	const uint requiredThreads = totalSize / sizeof(T);

	for (uint i = threadIdx.x; i < requiredThreads; i += blockDim.x)
	{
		dest[i] = source[i];
	}
}

#endif
