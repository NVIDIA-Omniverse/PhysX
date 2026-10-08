// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef __CU_SPARSE_REMOVE_CUH__
#define __CU_SPARSE_REMOVE_CUH__

#include "reduction.cuh"

namespace physx
{

/**
This function initializes a keep-drop buffer. Assuming an array of size N is having K elements removed, it initializes the first (N-K) elements to 0 and the next (K) elements to 1.
*/

static __device__ void initializeKeepDropBuffer(PxU32* PX_RESTRICT globalRunSumBuffer, PxU32 totalCount, PxU32 nbToRemove)
{
	const PxU32 newArraySize = totalCount - nbToRemove;
	const PxU32 globalThreadIdx = threadIdx.x + WARP_SIZE * threadIdx.y + blockIdx.x * blockDim.x * blockDim.y;
	
	for(PxU32 i = globalThreadIdx; i < totalCount; i += blockDim.x * blockDim.y * gridDim.x)
	{
		globalRunSumBuffer[i] = i < newArraySize ? 0 : 1;
	}
}

/**
This function marks a keep-drop buffer based on an array of indices to remove. Assuming an array of length N with K elements being removed, this marks a 1 in any element in the first (N-K)
elements that is being removed with a 1. It marks any element in the last K elements being removed with a 0. This assumes that "initializeKeepDropBuffer" was performed on the array first
*/
static __device__ void markKeepDropBuff(const PxU32* PX_RESTRICT removeIndex, const PxU32 nbToRemove, PxU32* globalRunSumBuffer, const PxU32 totalCount)
{
	const PxU32 newArraySize = totalCount - nbToRemove;
	const PxU32 globalThreadIdx = threadIdx.x + WARP_SIZE * threadIdx.y + blockIdx.x * blockDim.x * blockDim.y;
	for(PxU32 i = globalThreadIdx; i < nbToRemove; i += blockDim.x * blockDim.y * gridDim.x)
	{
		PxU32 index = removeIndex[i];
		PxU32 mask = index < newArraySize ? 1 : 0;
		globalRunSumBuffer[index] = mask;
	}
}

template<PxU32 blockSize, PxU32 gridSize>
static __device__ void processKeepDropBuff(PxU32* PX_RESTRICT globalRunSumBuffer, const PxU32 totalCount, PxU32* crossBlockTotalAccumulator)
{
	ReadArrayFunctor<PxU32> readF(globalRunSumBuffer);
	WriteArrayFunctor<PxU32> writeF(globalRunSumBuffer);
	scanKernel1of2<blockSize, gridSize, AddOpPxU32, PxU32, ReadArrayFunctor<PxU32>,	WriteArrayFunctor<PxU32> >(
		readF,
		writeF,
		totalCount,
		crossBlockTotalAccumulator);
}

template<PxU32 gridSize>
static __device__ void accumulateKeepDrop(PxU32* PX_RESTRICT globalRunSumBuffer, const PxU32 totalCount, PxU32* crossBlockTotalAccumulator)
{
	ReadArrayFunctor<PxU32> readF(globalRunSumBuffer);
	WriteArrayFunctor<PxU32> writeArrayF(globalRunSumBuffer);
	WriteValueNOPFunctor<PxU32> writeValueF;
	scanKernel2of2<gridSize, AddOpPxU32, PxU32, ReadArrayFunctor<PxU32>, WriteArrayFunctor<PxU32>, WriteValueNOPFunctor<PxU32> >(
		readF,
		writeArrayF,
		writeValueF,
		totalCount,
		crossBlockTotalAccumulator);
}

static __device__ PxU32 getNbSwapsRequired(const PxU32* PX_RESTRICT globalRunSumBuffer, const PxU32 originalCount, const PxU32 nbToRemove)
{
	const PxU32 newTotalSize = originalCount - nbToRemove;
	const PxU32 nbToReplaceInBuffer = globalRunSumBuffer[newTotalSize];
	return nbToReplaceInBuffer;
}

static __device__ void getSwapIndices(const PxU32* PX_RESTRICT globalRunSumBuffer, const PxU32 totalSize, const PxU32 indexToFind, const PxU32 totalSwapsRequired, 
	PxU32& destIndex, PxU32& srcIndex)
{
	destIndex = binarySearch(globalRunSumBuffer, totalSize, indexToFind);
	srcIndex = binarySearch(globalRunSumBuffer, totalSize, indexToFind+totalSwapsRequired);
}

} // namespace physx

#endif
