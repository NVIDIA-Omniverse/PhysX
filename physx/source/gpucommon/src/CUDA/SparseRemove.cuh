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

#ifndef __CU_SPARSE_REMOVE_CUH__
#define __CU_SPARSE_REMOVE_CUH__

#include "reduction.cuh"

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

#endif
