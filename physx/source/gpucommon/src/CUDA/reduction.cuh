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

#ifndef __CU_REDUCTION_CUH__
#define __CU_REDUCTION_CUH__

#include <float.h>
#include "foundation/PxPreprocessor.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxMath.h"
#include "PxgCommonDefines.h"
#include "assert.h"
#include "utils.cuh"

using namespace physx;

struct MinOpFloat
{
	PX_CUDA_CALLABLE
	static float defaultValue()
	{
		return FLT_MAX;
	}

	PX_CUDA_CALLABLE
	static float op(float a, float b)
	{
		return fminf(a, b);
	}

	PX_CUDA_CALLABLE
	static float op(unsigned int& retIdx, float a, unsigned int idxA, float b, unsigned int idxB)
	{
		if(b < a)
		{
			retIdx = idxB;
			return b;
		}
		else
		{
			retIdx = idxA;
			return a;
		}
	}
};

struct MaxOpFloat
{
	PX_CUDA_CALLABLE
	static inline float defaultValue()
	{
		return -FLT_MAX;
	}

	PX_CUDA_CALLABLE
	static inline float op(float a, float b)
	{
		return fmaxf(a, b);
	}

	PX_CUDA_CALLABLE
	static float op(unsigned int& retIdx, float a, unsigned int idxA, float b, unsigned int idxB)
	{
		if(b > a)
		{
			retIdx = idxB;
			return b;
		}
		else
		{
			retIdx = idxA;
			return a;
		}
	}
};

struct MaxOpPxU32
{
	PX_CUDA_CALLABLE
	static inline PxU32 op(const PxU32 a, const PxU32 b)
	{
		return max(a, b);
	}
};

struct AndOpPxU32
{
	PX_CUDA_CALLABLE
	static PxU32 defaultValue()
	{
		return 0xFFffFFff;
	}

	PX_CUDA_CALLABLE
	static PxU32 op(PxU32 a, PxU32 b)
	{
		return a & b;
	}
};

struct AddOpPxU32
{
	PX_CUDA_CALLABLE
	static PxU32 defaultValue()
	{
		return 0ul;
	}

	PX_CUDA_CALLABLE
	static PxU32 op(PxU32 a, PxU32 b)
	{
		return a + b;
	}
};

struct AddOpPxReal
{
	PX_CUDA_CALLABLE
	static PxReal defaultValue()
	{
		return 0ul;
	}

	PX_CUDA_CALLABLE
	static PxReal op(PxReal a, PxReal b)
	{
		return a + b;
	}
}; 

struct AddOpPxI32
{
	PX_CUDA_CALLABLE
	static PxI32 defaultValue()
	{
		return 0l;
	}

	PX_CUDA_CALLABLE
	static PxI32 op(PxI32 a, PxI32 b)
	{
		return a + b;
	}
}; 

struct OrOpPxU32
{
	PX_CUDA_CALLABLE
	static PxU32 defaultValue()
	{
		return 0;
	}

	PX_CUDA_CALLABLE
	static PxU32 op(PxU32 a, PxU32 b)
	{
		return a | b;
	}
};

//This isn't a runsum. It will produce the sum and spat the result to all the active threads
template<typename OP, typename T, PxU32 log2threadGroupSize>
__device__ static inline T warpReduction(const PxU32 syncMask, T input)
{
	const PxU32 threadGroupSize = (1U << log2threadGroupSize);

	#pragma unroll
	for(PxU32 reductionRadius = threadGroupSize >> 1; reductionRadius > 0; reductionRadius >>= 1)
	{
		T val = __shfl_xor_sync(syncMask, input, reductionRadius, threadGroupSize);
		input = OP::op(input, val);
	}

	return input;
}

//This isn't a runsum. It will produce the sum and spat the result to all the active threads
template<typename OP, typename T>
__device__ static inline T warpReduction(const PxU32 syncMask, T input)
{
	return warpReduction<OP, T, LOG2_WARP_SIZE>(syncMask, input);
}

//makes sense only for comparison operations that don't alter the value. Expect -1 if the op is altering inputs
template<typename OP, typename T, PxU32 log2threadGroupSize>
__device__ static inline T warpReduction(const PxU32 syncMask, const T& input, PxU32& winnerLaneIndex)
{
	T best = warpReduction<OP, T, log2threadGroupSize>(syncMask, input);
	winnerLaneIndex = lowestSetIndex(__ballot_sync(syncMask, best == input));

	return best;
}

template<typename OP, typename T>
__device__ static inline T warpReduction(const PxU32 syncMask, const T& input, PxU32& winnerLaneIndex)
{
	return warpReduction<OP, T, LOG2_WARP_SIZE>(syncMask, input, winnerLaneIndex);
}

template<typename OP, typename T>
__device__ static inline T blockReduction(const PxU32 syncMask, const T& input, const T& initialValue, const PxU32 blockSize, volatile T* sharedMemoryOneEntryPerWarp)
{
	const PxU32 numWarpsPerBlock = blockSize / WARP_SIZE;

	const PxU32 warpIndex = threadIdx.x / WARP_SIZE;
	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);

	T warpResult = warpReduction<OP, T>(syncMask, input);

	if (threadIndexInWarp == 0)
	{
		sharedMemoryOneEntryPerWarp[warpIndex] = warpResult;
	}
	__syncthreads();

	T val = (threadIdx.x < numWarpsPerBlock) ? sharedMemoryOneEntryPerWarp[threadIndexInWarp] : initialValue;

	if (warpIndex == 0)
		return warpReduction<OP, T>(syncMask, val);
	else
		return initialValue;
}

template<typename OP, typename T, const PxU32 blockSize>
__device__ static inline T blockReduction(const PxU32 syncMask, const T& input, const T& initialValue)
{
	const PxU32 numWarpsPerBlock = blockSize / WARP_SIZE;

	volatile __shared__ T sData[numWarpsPerBlock];

	return blockReduction<OP, T>(syncMask, input, initialValue, blockSize, sData);
}


//inclusive scan
template<typename OP, typename T, PxU32 log2threadGroupSize>
__device__ static inline T warpScan(const PxU32 syncMask, T input)
{
	const PxU32 threadGroupSize = (1U << log2threadGroupSize);
	const PxU32 idxInGroup = threadIdx.x & (threadGroupSize-1);
	
	#pragma unroll
	for(PxU32 reductionRadius = 1; reductionRadius < threadGroupSize; reductionRadius <<= 1)
	{
		T val = __shfl_up_sync(syncMask, input, reductionRadius, threadGroupSize);

		if (idxInGroup >= reductionRadius)
			input = OP::op(input, val);
	}

	return input;
}

//inclusive scan
template<typename OP, typename T>
__device__ static inline T warpScan(const PxU32 syncMask, T input)
{
	return warpScan<OP, T, LOG2_WARP_SIZE>(syncMask, input);
}


//exclusive scan
template<typename OP, typename T>
__device__ static inline T warpScanExclusive(T input)
{
	T output = OP::defaultValue();

	const PxU32 idxInGroup = threadIdx.x & (WARP_SIZE - 1);

#pragma unroll
	for (PxU32 reductionRadius = 1; reductionRadius < WARP_SIZE; reductionRadius <<= 1)
	{
		T val = __shfl_up_sync(FULL_MASK, input, reductionRadius);

		if (idxInGroup >= reductionRadius)
		{
			input = OP::op(input, val);
			output = OP::op(output, val);
		}
	}

	return output;
}

template<typename T>
class ReadArrayFunctor
{
public:
	PX_CUDA_CALLABLE
	ReadArrayFunctor(const T* PX_RESTRICT arr): mArr(arr) {}

	PX_CUDA_CALLABLE
	T operator()(PxU32 idx) const {return mArr[idx];}
protected:
	const T* mArr;
};

template<typename T>
class WriteArrayFunctor
{
public:
	PX_CUDA_CALLABLE
	WriteArrayFunctor(T* PX_RESTRICT arr): mArr(arr) {}

	PX_CUDA_CALLABLE
	void operator()(PxU32 idx, const T& val) {mArr[idx] = val;}
protected:
	T* mArr;
};

template<typename T>
class WriteValueFunctor
{
public:
	PX_CUDA_CALLABLE
	WriteValueFunctor(T* PX_RESTRICT addr): mAddr(addr) {}

	PX_CUDA_CALLABLE
	void operator()(const T& val) {*mAddr = val;}

protected:
	T* mAddr;
};

template<typename T>
class WriteValueNOPFunctor
{
public:
	PX_CUDA_CALLABLE
	WriteValueNOPFunctor() {}

	PX_CUDA_CALLABLE
	void operator()(const T&) {}
};

template<PxU32 blockSize, PxU32 gridSize, typename OP, typename T, typename GetInputFunctor, typename SetTempFunctor>
static __device__ void scanKernel1of2(
									  const GetInputFunctor& getInputF,
									  SetTempFunctor& setTempF,
									  const PxU32 totalCount, 
									  T* crossBlockTotalAccumulator)
{
	__shared__ T crossWarpAccumulator[blockSize >> LOG2_WARP_SIZE];
	__shared__ T accum;

	if(threadIdx.x == 0)
		accum = OP::defaultValue();

	__syncthreads();

	const PxU32 nbThreads = blockSize;

	const PxU32 nbBlocksRequired = (totalCount + (nbThreads-1))/nbThreads;
	const PxU32 nbBlocksPerBlock = (nbBlocksRequired + gridDim.x-1)/gridDim.x;

	const PxU32 blockStartIndex = blockIdx.x * nbBlocksPerBlock;
	const PxU32 threadIndexInWarp = threadIdx.x;
	const PxU32 warpIndexInBlock = threadIdx.y;
	
	for (PxU32 i = 0; i < nbBlocksPerBlock; ++i)
	{
		const PxU32 threadIndex = threadIdx.x + WARP_SIZE * threadIdx.y + (blockStartIndex + i) * blockDim.x * blockDim.y;

		T val = OP::defaultValue();

		if (threadIndex < totalCount)
			val = getInputF(threadIndex);

		T res = warpScanExclusive<OP, T>(val);

		if (threadIndexInWarp == (WARP_SIZE - 1))
			crossWarpAccumulator[warpIndexInBlock] = OP::op(res, val);
				
		T prevAccum = accum;

		__syncthreads();

		if (warpIndexInBlock == 0)
		{
			T val2 = OP::defaultValue();

			if (threadIndexInWarp < (blockSize >> LOG2_WARP_SIZE))
				val2 = crossWarpAccumulator[threadIndexInWarp];


			T res2 = warpScanExclusive<OP, T>(val2);

			if (threadIndexInWarp < (blockSize >> LOG2_WARP_SIZE))
				crossWarpAccumulator[threadIndexInWarp] = res2;
		
			if (threadIndexInWarp == ((blockSize >> LOG2_WARP_SIZE) - 1))
			{
				accum = OP::op(accum, OP::op(res2, val2));
			}
		}

		__syncthreads();

		if (threadIndex < totalCount)
			setTempF(threadIndex, OP::op(res, OP::op(crossWarpAccumulator[warpIndexInBlock], prevAccum)));
	}

	if ((threadIdx.y * WARP_SIZE + threadIdx.x) == ((blockSize >> LOG2_WARP_SIZE)-1))
	{
		crossBlockTotalAccumulator[blockIdx.x] = accum;
	}
}

template<PxU32 gridSize, typename OP, typename T, typename GetTempFunctor, typename SetOutputFunctor, 
																				typename WriteGrandTotalFunctor>
static __device__ void scanKernel2of2(
									  const GetTempFunctor& getTempF,
									  SetOutputFunctor& setOutF,
									  WriteGrandTotalFunctor& writeTotalF,
									  const PxU32 totalCount,
									  const T* crossBlockTotalAccumulator)
{
	const PxU32 nbThreads = blockDim.x * blockDim.y;

	const PxU32 nbBlocksRequired = (totalCount + (nbThreads-1))/nbThreads;
	const PxU32 nbBlocksPerBlock = (nbBlocksRequired + gridDim.x-1)/gridDim.x;

	const PxU32 blockStartIndex = blockIdx.x * nbBlocksPerBlock;

	__shared__ T blockAccum[gridSize];

	const PxU32 threadIndexInWarp = threadIdx.x;
	

	if (threadIdx.y == 0)
	{
		T val = OP::defaultValue();

		if (threadIdx.x < gridSize)
			val = crossBlockTotalAccumulator[threadIndexInWarp];

		T res = warpScanExclusive<OP, T>(val);

	
		if (threadIdx.x < gridSize)
			blockAccum[threadIndexInWarp] = res;

		if (threadIdx.x == gridSize - 1 && blockIdx.x == 0)
		{
			writeTotalF(OP::op(val, res));
		}
	}

	__syncthreads();

	T accumulation = blockAccum[blockIdx.x];

	for(PxU32 i = 0; i < nbBlocksPerBlock; ++i)
	{
		const PxU32 threadIndex = threadIdx.x + WARP_SIZE * threadIdx.y + (blockStartIndex + i) * blockDim.x * blockDim.y;

		if(threadIndex < totalCount)
		{
			T val = OP::op(getTempF(threadIndex), accumulation);
			setOutF(threadIndex, val);
		}
	}
}

//keeping this for the broadphase: 

//This is the parallel version of sum. 
template<PxU32 nbElems, typename T>
static __inline__ __device__ T warpScanAdd(const PxU32 syncMask, const PxU32 index, const PxU32 threadIndexInWarp, T* sData, const T originalValue, const T value)
{

	unsigned mask_local = __ballot_sync(syncMask, threadIndexInWarp < nbElems);

	if(threadIndexInWarp < nbElems)
	{
		T val = originalValue;

		#pragma unroll
		for(PxU32 i = 1; i < nbElems; i<<=1)
		{
			const T temp = __shfl_sync(mask_local, val, threadIndexInWarp-i);

			if(threadIndexInWarp >= i)
				val += temp;
		}
		return val - value;
	}

	return 0;
}

template<PxU32 nbElems>
static __inline__ __device__ PxU32 warpScanMax(const PxU32 syncMask, const PxU32 index, const PxU32 threadIndexInWarp, PxU32* sData, const PxU32 originalValue)

{
	unsigned mask_local = __ballot_sync(syncMask, threadIndexInWarp < nbElems);

	if(threadIndexInWarp < nbElems)
	{
		PxU32 val = originalValue;

		PxU32 mask = nbElems < 32 ? (1 << nbElems) - 1 : 0xffffffff;

		#pragma unroll
		for(PxU32 i = 1; i < nbElems; i<<=1)
		{
			const PxU32 temp = __shfl_sync(mask_local, (int)val, threadIndexInWarp-i);
			
			if(threadIndexInWarp >= i)
				val = PxMax(temp, val);
		}
		return val;
	}

	return 0;	
}

//This is the parallel version of exclusive sum. We have 32 thread in a warp, so that we need to
//have 2 exp 5 step(16) of add
template<PxU32 nbElems>
static __inline__ __device__ PxU32 warpScanAddWriteToSharedMem(const PxU32 syncMask, const PxU32 index, const PxU32 threadIndexInWarp, PxU32* sData, const PxU32 originalValue, const PxU32 value)
{

	unsigned mask_local = __ballot_sync(syncMask, threadIndexInWarp < nbElems);

	if(threadIndexInWarp < nbElems)
	{
		PxU32 temp = 0;
		PxU32 val = originalValue;

		#pragma unroll
		for(PxU32 i = 1; i < nbElems; i<<=1)
		{
			temp = __shfl_sync(mask_local, (int)val, threadIndexInWarp-i);
			
			if(threadIndexInWarp >= i)
				val += temp;
		}

		val -= value;
		sData[index] = val;
		__syncwarp(mask_local); //Mem fence for shared data write
		return val;
	}

	return 0;	
}


PX_FORCE_INLINE __device__ PxU32 warpCountAndBroadcast(PxU32 mask, bool threadContributesElement)
{
	return __popc(__ballot_sync(mask, threadContributesElement));
}

PX_FORCE_INLINE __device__ PxU32 threadBlockCountAndBroadcast(bool threadContributesElement, PxU32* sharedMem)
{
	const PxU32 threadIndexInWarp = threadIdx.x & 31;
	const PxU32 warpIndex = threadIdx.x >> 5; // threadIdx.x / 32;

	PxU32 perWarpCount = warpCountAndBroadcast(FULL_MASK, threadContributesElement);

	if (threadIndexInWarp == 0)
		sharedMem[warpIndex] = perWarpCount;

	__syncthreads();

	return warpReduction<AddOpPxU32, PxU32>(FULL_MASK, sharedMem[threadIndexInWarp]);
}



// Performs an exclusive scan over a warp. Every thread can only contribute 0 or 1 to the scan through the mask
PX_FORCE_INLINE __device__ PxU32 warpScanExclusive(PxU32 mask, PxU32 threadIndexInWarp)
{
	return __popc(mask & ((1 << threadIndexInWarp) - 1));
}

//Performs an exclusive scan over a warp. Every thread can only contribute 0 or 1 to the sum. The return value will be the exclusive cumulative sum for every thread. totalSum will provide the total number of contributed elements.
//perWarpCountS must be shared memory with NbWarps entries available
template<PxU32 NbWarps>
PX_FORCE_INLINE __device__ PxU32 threadBlockScanExclusive(bool threadContributesElement, PxU32& totalSum, PxU32* perWarpCountS)
{
	const PxU32 threadIndexInWarp = threadIdx.x & 31;
	const PxU32 warpIndex = threadIdx.x >> 5; // threadIdx.x / 32;

	PxU32 ballotMask = __ballot_sync(FULL_MASK, threadContributesElement);

	PxU32 nbInteresting = __popc(ballotMask); //The number of elements emitted per warp

	if (threadIndexInWarp == 0)
		perWarpCountS[warpIndex] = nbInteresting;

	__syncthreads();

	PxU32 warpCount = threadIndexInWarp < NbWarps ? perWarpCountS[threadIndexInWarp] : 0;

	PxU32 total = warpScan<AddOpPxU32, PxU32>(FULL_MASK, warpCount); //The inclusive cumulative sum per warp. The last warp has the total number of elements created by the full thread block

	PxU32 carryExclusiveScan = __shfl_sync(FULL_MASK, total, warpIndex) - perWarpCountS[warpIndex]; //Broadcast the exclusive cumulative base sum (inclusiveSum - perWarpCount = exclusiveSum)

	totalSum = __shfl_sync(FULL_MASK, total, 31); //Broadcast the total sum of the last warp (which is the overall total sum) to all warps

	return carryExclusiveScan + warpScanExclusive(ballotMask, threadIndexInWarp); //Combine base sum and the sum per warp
}

template<PxU32 NbWarps>
PX_FORCE_INLINE __device__ PxU32 threadBlockScanExclusive(bool threadContributesElement, PxU32& totalSum)
{
	__shared__ PxU32 perWarpCountS[NbWarps];
	return threadBlockScanExclusive<NbWarps>(threadContributesElement, totalSum, perWarpCountS);
}


//Performs an exclusive scan over a warp. Every thread can contribute an arbitrary value to the sum. The return value will be the exclusive cumulative sum for every thread. totalSum will provide the total number of contributed elements.
//perWarpCountS must be shared memory with NbWarps entries available
template<PxU32 NbWarps>
PX_FORCE_INLINE __device__ PxU32 threadBlockScanExclusive(PxU32 numElementsFromCallingThread, PxU32& totalSum, PxU32* perWarpCountS)
{
	const PxU32 threadIndexInWarp = threadIdx.x & 31;
	const PxU32 warpIndex = threadIdx.x >> 5; // threadIdx.x / 32;

	PxU32 perWarpInclusiveScan = warpScan<AddOpPxU32, PxU32>(FULL_MASK, numElementsFromCallingThread);

	if (threadIndexInWarp == 31)
		perWarpCountS[warpIndex] = perWarpInclusiveScan;

	__syncthreads();

	PxU32 warpCount = threadIndexInWarp < NbWarps ? perWarpCountS[threadIndexInWarp] : 0;

	PxU32 total = warpScan<AddOpPxU32, PxU32>(FULL_MASK, warpCount); //The inclusive cumulative sum per warp. The last warp has the total number of elements created by the full thread block

	PxU32 carryExclusiveScan = __shfl_sync(FULL_MASK, total, warpIndex) - perWarpCountS[warpIndex]; //Broadcast the exclusive cumulative base sum (inclusiveSum - perWarpCount = exclusiveSum)

	totalSum = __shfl_sync(FULL_MASK, total, 31); //Broadcast the total sum of the last warp (which is the overall total sum) to all warps

	return carryExclusiveScan + perWarpInclusiveScan - numElementsFromCallingThread; //Combine base sum and the sum per warp
}

template<PxU32 NbWarps>
PX_FORCE_INLINE __device__ PxU32 threadBlockScanExclusive(PxU32 numElementsFromCallingThread, PxU32& totalSum)
{
	__shared__ PxU32 perWarpCountS[NbWarps];
	return threadBlockScanExclusive<NbWarps>(numElementsFromCallingThread, totalSum, perWarpCountS);
}

//Allows to get indices in an output array for every thread (even accross thread blocks) where every thread either emits an element or not.
//Only threads that contribute an element will get a valid index returned which is expected since all other threads don't output anything.
//The order of the ouput indices is not deterministic since atomic add is used
//The method must be called by one or preferrably multiple full warps, for a single warp see optimized version below
template<PxU32 NbWarps>
PX_FORCE_INLINE __device__ PxU32 globalScanExclusive(bool threadContributesElement, PxU32* atomicCounter)
{
	__syncthreads();

	__shared__ PxU32 perWarpCountS[NbWarps];
	PxU32 threadBlockTotalSum;
	PxU32 indexInThreadBlock = threadBlockScanExclusive<NbWarps>(threadContributesElement, threadBlockTotalSum, perWarpCountS);

	__shared__ PxU32 globalOffset;
	if (threadIdx.x == 0)
	{
		//Only one thread per thread block needs to perform an atomic add
		globalOffset = atomicAdd(atomicCounter, threadBlockTotalSum);
	}
	__syncthreads();
	return indexInThreadBlock + globalOffset;
}

//Optimized version where a thread block only consist out of one warp. Does not need shared memory.
//Allows to get indices in an output array for every thread (even accross thread blocks) where every thread either emits an element or not.
//Only threads that contribute an element will get a valid index returned which is expected since all other threads don't output anything.
//The order of the ouput indices is not deterministic since atomic add is used
//The method must be called by one full warp
PX_FORCE_INLINE __device__ PxU32 globalScanExclusiveSingleWarp(bool threadContributesElement, PxU32* atomicCounter)
{
	PxU32 idxInWarp = threadIdx.x & 31;

	const PxU32 resultWarp = __ballot_sync(FULL_MASK, threadContributesElement);
	const PxU32 offset = warpScanExclusive(resultWarp, idxInWarp); // __popc(resultWarp & ((1 << idxInWarp) - 1));
	const PxU32 validCount = __popc(resultWarp);

	PxU32 startIndex = 0xFFffFFff - 32; // -32 to prevent wrap-around
	if (idxInWarp == 0 && validCount > 0)
	{
		startIndex = atomicAdd(atomicCounter, validCount);
	}
	return __shfl_sync(FULL_MASK, startIndex, 0) + offset;
}

// returns the largest index into the (sorted!) data array s.t. data[index] <= value
// if there is no such index (i.e., data[0] > value) returns 0
template<class T>
static __device__ PxU32 binarySearch(const T* PX_RESTRICT data, const PxU32 numElements, const T& value)
{
	PxU32 left = 0;
	PxU32 right = numElements;
	
	//while((right - left) > 1)
	while(left < right)
	{
		PxU32 pos = (left + right) / 2;
		const T& element = data[pos];

		if (element <= value)
		{
			//Found exact value
			left = pos+1;
		}
		else
		{
			right = pos;
		}
	}

	return left ? left - 1 : 0;
}

#endif //CU_REDUCTION_CU
