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

#ifndef __CU_RADIX_SORT_CUH__
#define __CU_RADIX_SORT_CUH__

#include "foundation/PxPreprocessor.h"
#include "vector_types.h"
#include "PxgRadixSortDesc.h"
#include "PxgRadixSortKernelIndices.h"
#include "PxgCommonDefines.h"
#include "stdio.h"

#include "reduction.cuh"

#include "foundation/PxMath.h"
#include <assert.h>

using namespace physx;

#define		RADIX_SIZE			16
#define		RADIX_ACCUM_SIZE	8


static __device__ uint4 getRadix(uint4 input, const PxU32 startBit)
{
	uint4 radix;
	radix.x = (input.x >> startBit) & 0xF;
	radix.y = (input.y >> startBit) & 0xF;
	radix.z = (input.z >> startBit) & 0xF;
	radix.w = (input.w >> startBit) & 0xF;
	return radix;
	//return (input >> startBit) & 0xF;
}

static __device__ PxU32 getRadix(PxU32 input, const PxU32 startBit)
{
	return (input >> startBit) & 0xF;
}


//accumulated each individual warps to a responding radix
//radix0[0, 7] ==> radixSum[7], radix1[8, 15] ==> radixSum[15], radix2[16, 23] ==>radixSum[23], radix3[24, 31] ==> radixSum[31], 
//radix4[32, 39] ==> radixSum[39], radix5[40, 47] ==>radixSum[47], radix6[48, 55] ==>radixSum[55], radix7[56, 63] ==>radixSum[63]
//
template <PxU32 WARP_PERBLOCK_SIZE>
static __device__ PxU32 scanRadixWarps(const PxU32 threadIndexInWarp, PxU32* radixSum, const PxU32 originalVal, const PxU32 value)
{
	const PxU32 idx = threadIdx.x;

	const PxU32 radixIndex = threadIndexInWarp & (WARP_PERBLOCK_SIZE-1);

	int val = originalVal; 

	for(PxU32 a = 1; a < WARP_PERBLOCK_SIZE; a*=2)
	{
		int temp = __shfl_sync(FULL_MASK, val, idx-a);

		if(radixIndex >= a)
		{
			val += temp;
		}
	}
	
	return val - value;
}

template <PxU32 WARP_PERBLOCK_SIZE>
static __device__ void scanRadixes(const PxU32 warpIndexInBlock, const PxU32 threadIndexInWarp, PxU32* PX_RESTRICT gData, PxU32* PX_RESTRICT sData, PxU32* PX_RESTRICT accumulateBuffer)
{
	for(PxU32 i=warpIndexInBlock; i<RADIX_SIZE; i+=WARP_PERBLOCK_SIZE)
	{

		const PxU32 radixSumIndex = i*gridDim.x + threadIndexInWarp;

		const PxU32 value = gData[radixSumIndex];

		PxU32 output = warpScanAddWriteToSharedMem<WARP_SIZE>(FULL_MASK, radixSumIndex, threadIndexInWarp, sData, value, value);

		if(threadIndexInWarp == (WARP_SIZE-1))
			accumulateBuffer[i] = output + value;
	}
}

//there are 256 threads in a block and therefore 8 warp in a block

__device__ inline PxU32 sanitizeKeys(uint4& keyValue, PxU32 id, const PxU32 count)
{
	PxU32 goodVals = count - id;
	if (goodVals < 4)
	{
		PxU32 badVals = 4 - goodVals;

		switch (badVals)
		{
		case 3:
			keyValue.y = 0xFFffFFff;
		case 2:
			keyValue.z = 0xFFffFFff;
		case 1:
			keyValue.w = 0xFFffFFff;
		}
	}

	return goodVals;
}

struct RadixAccum
{
	PxU32 radixAccum[RADIX_ACCUM_SIZE];
};

__device__ inline void radixSortWarp(const uint4* PX_RESTRICT gInputKeys, const PxU32 idx, const PxU32 count, const PxU32 stride, 
	const PxU32 startBit, const PxU32 startIdx, const PxU32 totalCount, RadixAccum& radixAccum)
{
	//HISTOGRAM-KEYS && SCAN-BUCKET
	uint4 keyValue;
	uint4 radix;

	for(PxU32 i = idx; i < count; i += stride)
	{
		const PxU32 gInputIdx = i + startIdx;
		keyValue = gInputKeys[gInputIdx];

		const PxU32 nbVals = sanitizeKeys(keyValue, gInputIdx * 4, totalCount);

		radix = getRadix(keyValue, startBit);

		//each thread read 4 elements. We store the each element's radix[0, 15] into a local array. The code should be
		//radixAccum[radix.x]++; radixAccum[radix.y]++; radixAccum[radix.z]++; radixAccum[radix.w]++; 
		//However, in order to save register, each radixAccum can store 2 radix's accumulation result. Therefore, radixAccum is
		//half of the size of RADIX_SIZE and each radix has 16 bits in the radixAccum.
		//The for loop is used to trick the compiler to keep radixAccum array in registers
#pragma unroll
		for (PxU32 bit = 0; bit < 16; bit += 2)
		{
			PxU32 accum = (1u << ((radix.x - bit) << 4));
			accum += (1u << ((radix.y - bit) << 4));
			accum += (1u << ((radix.z - bit) << 4));
			accum += (1u << ((radix.w - bit) << 4));

			radixAccum.radixAccum[bit / 2] += accum;
		}
	}
}



template <PxU32 WARP_PERBLOCK_SIZE>
__device__ inline void radixSortSingleBlock(const uint4* PX_RESTRICT gInputKeys, const uint4* PX_RESTRICT gInputRanks, const PxU32 gNumKeys, const PxU32 startBit, PxU32* gRadixCount)
{
	const PxU32 nbBlocks = PxgRadixSortKernelGridDim::RADIX_SORT;
	PX_COMPILE_TIME_ASSERT(nbBlocks == 32);

	__shared__ PxU32 sRadixSum[RADIX_SIZE*WARP_PERBLOCK_SIZE];

	//the number of inputKeys is random. However, this algorithm sort inputKeys at a time, so that we need to initialize the number of keys properly
	const PxU32 numKeys = (gNumKeys+3)/4;

	const PxU32 totalBlockRequired = (numKeys + (blockDim.x-1))/ blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (nbBlocks-1))/ nbBlocks;

	//This identifies which warp a specific thread is in, we treat all warps in all blocks as a flatten warp array
	//and we are going to index the work based on that
	//const PxU32 warpIndex = blockIdx.x * blockStride + threadIdx.x/WARP_SIZE;
	const PxU32 warpIndexInBlock = threadIdx.x/WARP_SIZE;

	//This identifies which thread within a warp a specific thread is
	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE-1);

	const PxU32 idx = threadIdx.x;

	RadixAccum radixAccum;
	#pragma unroll
	for(PxU32 i=0; i< RADIX_ACCUM_SIZE; ++i)
	{
		radixAccum.radixAccum[i] = 0;
	}

	const PxU32 inputKeyIndex = PxMin(numIterationPerBlock * blockIdx.x * blockDim.x, numKeys);
	const PxU32 endIndex = PxMin(inputKeyIndex + numIterationPerBlock*blockDim.x, numKeys);
	const PxU32 count = endIndex - inputKeyIndex;
	radixSortWarp(gInputKeys, idx, count, WARP_SIZE*WARP_PERBLOCK_SIZE, startBit, inputKeyIndex, gNumKeys, radixAccum);

	PxU32 accumValue = 0;
	#pragma unroll
	for(PxU32 i=0; i<RADIX_SIZE; i+=2)
	{
		const PxU32 accum = radixAccum.radixAccum[i/2];

		const PxU32 val = warpScanAdd<WARP_SIZE>(FULL_MASK, idx, threadIndexInWarp, (PxU32*)NULL, accum, accumValue);

		const PxU32 val2 = __shfl_sync(FULL_MASK, (int)val, (WARP_SIZE - 1));// getLastElementValueInAWarp(val, idx, sData, WARP_SIZE);

		if(threadIndexInWarp < 2)
		{
			sRadixSum[(i+threadIndexInWarp)*WARP_PERBLOCK_SIZE + idx/WARP_SIZE ] = (val2 >> (threadIndexInWarp*16)) & 0xFFFF;
		}
	}
	
	__syncthreads();

	//unsigned mask_warpIndexInBlock = __ballot_sync(syncMask, warpIndexInBlock < (WARP_PERBLOCK_SIZE / 2));
	if (warpIndexInBlock < (WARP_PERBLOCK_SIZE/2))
	{
		const PxU32 originalValue = sRadixSum[idx];
		const PxU32 output = scanRadixWarps<WARP_PERBLOCK_SIZE>(threadIndexInWarp, sRadixSum, originalValue, 0);//output is the value should be in sRadixSum[idx]

		if((idx & (WARP_PERBLOCK_SIZE-1)) == (WARP_PERBLOCK_SIZE-1))
		{
			//copy to global memory
			//const PxU32 gRadixIndex = blockIdx.x * RADIX_SIZE + idx;
			const PxU32 gRadixIndex = blockIdx.x + idx/WARP_PERBLOCK_SIZE * gridDim.x;
		
			//gRadixCount have 16 radix and each radix has 32 blocks. Each block in gRadixCount store the numbers of elements in each radix
			gRadixCount[ gRadixIndex ] = output;
		}
	}
}

template <PxU32 WARP_PERBLOCK_SIZE>
__device__ inline void radixSortCalculateRanks(const uint4* PX_RESTRICT gInputKeys, const uint4* PX_RESTRICT gInputRanks, const PxU32 gNumOfKeys, const PxU32 startBit, PxU32* gRadixCount, PxU32* gOutputKeys, PxU32* gOutputRanks)
{
	const PxU32 nbBlocks = PxgRadixSortKernelGridDim::RADIX_SORT;
	PX_COMPILE_TIME_ASSERT(nbBlocks == 32);

	__shared__ PxU32 sRadixSumBetweenBlocks[RADIX_SIZE];//how many 0 before 1

	__shared__ PxU32 sRadixCountBetweenBlocks[RADIX_SIZE * (WARP_PERBLOCK_SIZE +2) + WARP_PERBLOCK_SIZE * WARP_SIZE];

	PxU32* sBuckets = &sRadixCountBetweenBlocks[0];
	PxU32* sRadixSum = sBuckets + WARP_PERBLOCK_SIZE * WARP_SIZE;
	PxU32* sRadixSumSum = sRadixSum + RADIX_SIZE*WARP_PERBLOCK_SIZE;
	PxU32* sRadixCount = sRadixSumSum + RADIX_SIZE;

	__shared__ PxU32 sKeys[WARP_SIZE  *WARP_PERBLOCK_SIZE * 4];
	__shared__ PxU32 sRanks[WARP_SIZE * WARP_PERBLOCK_SIZE * 4];

	//in the CPU we pass an array of PxU32 as the source inputKeys, therefore, we need to get the correct number of keys in GPU
	const PxU32 numKeys = (gNumOfKeys+3)/4;

	const PxU32 idx = threadIdx.x;

	//const PxU32 gridThreadIdx = idx + blockIdx.x * blockDim.x;

	const PxU32 warpIndexInBlock = threadIdx.x/WARP_SIZE;

	//This identifies which thread within a warp a specific thread is
	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE-1);

	scanRadixes<WARP_PERBLOCK_SIZE>(warpIndexInBlock, threadIndexInWarp, gRadixCount, sRadixCountBetweenBlocks, sRadixSumBetweenBlocks);

	__syncthreads();

	//accumulate total numbers of each radix in each warp inside the same block
	unsigned mask_idx = __ballot_sync(FULL_MASK, idx < RADIX_SIZE);
	if(idx < RADIX_SIZE)
	{
		const PxU32 value = sRadixSumBetweenBlocks[idx];

		const PxU32 output = warpScanAdd<RADIX_SIZE>(mask_idx, idx, threadIndexInWarp, sRadixSumBetweenBlocks, value, value);
		sRadixSumBetweenBlocks[idx] = output + sRadixCountBetweenBlocks[idx * nbBlocks + blockIdx.x];
	}

	__syncthreads();
	
	const PxU32 totalBlockRequired = (numKeys + (blockDim.x-1))/ blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (nbBlocks-1))/ nbBlocks;

	for(PxU32 i=0; i<numIterationPerBlock; ++i)
	{
		const PxU32 inputKeyIndex = i*WARP_SIZE*WARP_PERBLOCK_SIZE + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		uint4 keyValue;
		uint4 radix;
		uint4 keyIndex;
		uint4 radixOffset;

		radixOffset.x = radixOffset.y = radixOffset.z = radixOffset.w = 0;

		//read 4 elements at a time
		if(inputKeyIndex < numKeys)
		{
			keyIndex = gInputRanks[inputKeyIndex];
			
			keyValue = gInputKeys[inputKeyIndex];

			sanitizeKeys(keyValue, inputKeyIndex * 4, gNumOfKeys);

			radix = getRadix(keyValue, startBit);		
		}
		else
		{
			//pad the extra radix with sufficent large enough number(we have 8 passes and each pass just sort 4 bits so 0xff is sufficent large enough)
			radix.x = radix.y = radix.z = radix.w = 0xff;
		}
		
		//#pragma unroll
		for(PxU32 i=0; i<RADIX_SIZE; i+=4)
		{
			PxU32 accum = (1u << ((radix.x - i) << 3));
			accum += (1u << ((radix.y - i) << 3));
			accum += (1u << ((radix.z - i) << 3));
			accum += (1u << ((radix.w - i) << 3));

			PxU32 val = warpScanAdd<WARP_SIZE, PxU32>(FULL_MASK, idx, threadIndexInWarp, sBuckets, accum, 0);

			const PxU32 val2 = __shfl_sync(FULL_MASK, (int)val, (WARP_SIZE - 1)); //getLastElementValueInAWarp(val, idx, sBuckets, WARP_SIZE);

			if(threadIndexInWarp < 4)
			{
				sRadixSum[(i+threadIndexInWarp)*WARP_PERBLOCK_SIZE + idx/WARP_SIZE ] = (val2 >> (8*threadIndexInWarp)) & 0xFF;
			}

			val -= accum;

			//radix offset inside a warp
			PxU32 shiftBits = (radix.x - i) << 3;
			PxU32 offset = ((val >> shiftBits) & 0xFF);
			radixOffset.x |= offset;
			val += (1<<shiftBits);
			shiftBits = (radix.y - i) << 3;
			offset = ((val >> shiftBits) & 0xFF);
			radixOffset.y |= offset;
			val += (1<<shiftBits);
			shiftBits = (radix.z - i) << 3;
			offset = ((val >> shiftBits) & 0xFF);
			radixOffset.z |= offset;
			val += (1<<shiftBits);
			shiftBits = (radix.w - i) << 3;
			offset = ((val >> shiftBits) & 0xFF);
			radixOffset.w |= offset;			
		}

		__syncthreads();

		PxU32 lastRadixSum = 0;
		if(idx < RADIX_SIZE)
		{
			lastRadixSum = sRadixSum[idx*WARP_PERBLOCK_SIZE+(WARP_PERBLOCK_SIZE-1)];
		}
		__syncthreads();
		//scan sRadixSum for a block
		
		if(warpIndexInBlock < (WARP_PERBLOCK_SIZE/2))
		{
			const PxU32 tempVal = sRadixSum[idx];
			sRadixSum[idx] = scanRadixWarps<WARP_PERBLOCK_SIZE>(threadIndexInWarp, sRadixSum, tempVal, tempVal);
		}

		__syncthreads();

		unsigned mask_idx = __ballot_sync(FULL_MASK, idx < RADIX_SIZE);
		if(idx < RADIX_SIZE)
		{
			const PxU32 value = sRadixSum[idx*WARP_PERBLOCK_SIZE+(WARP_PERBLOCK_SIZE-1)] + lastRadixSum;
			sRadixCount[idx] = value;
			sRadixSumSum[idx] = value;
			__syncwarp(mask_idx);

			warpScanAddWriteToSharedMem<RADIX_SIZE>(mask_idx, idx, threadIndexInWarp, sRadixSumSum, value, value);
		}
		__syncthreads();

		
		if(idx < (WARP_PERBLOCK_SIZE * RADIX_SIZE))
		{
			sRadixSum[idx] += sRadixSumSum[idx/WARP_PERBLOCK_SIZE];
		}
		
		__syncthreads();

		if(idx < RADIX_SIZE)
			sRadixSumSum[idx] = sRadixSumBetweenBlocks[idx] - sRadixSumSum[idx];

		if(inputKeyIndex < numKeys)
		{
			//radix offset between warps inside a block
			radixOffset.x += sRadixSum[(WARP_PERBLOCK_SIZE * radix.x) + warpIndexInBlock]; 
			radixOffset.y += sRadixSum[(WARP_PERBLOCK_SIZE * radix.y) + warpIndexInBlock]; 
			radixOffset.z += sRadixSum[(WARP_PERBLOCK_SIZE * radix.z) + warpIndexInBlock]; 
			radixOffset.w += sRadixSum[(WARP_PERBLOCK_SIZE * radix.w) + warpIndexInBlock];

			sKeys[radixOffset.x] = keyValue.x;
			sKeys[radixOffset.y] = keyValue.y;
			sKeys[radixOffset.z] = keyValue.z;
			sKeys[radixOffset.w] = keyValue.w;

			sRanks[radixOffset.x] = keyIndex.x;
			sRanks[radixOffset.y] = keyIndex.y;
			sRanks[radixOffset.z] = keyIndex.z;
			sRanks[radixOffset.w] = keyIndex.w;
		}

		__syncthreads();
		const PxU32 baseInputKeyIndex = inputKeyIndex-idx;

		if(baseInputKeyIndex < numKeys)
		{
			//If there were keys to process... The if statement defends against the PxU32 becoming huge and us overflowing the arrays
			//const PxU32 keysToProcess = min(WARP_SIZE*WARP_PERBLOCK_SIZE*4, (numKeys - baseInputKeyIndex)*4);
			const PxU32 keysToProcess = min(WARP_SIZE*WARP_PERBLOCK_SIZE * 4, (gNumOfKeys - baseInputKeyIndex*4));

			for(PxU32 a = idx; a < keysToProcess; a += blockDim.x)
			{
				const PxU32 key = sKeys[a];
				const PxU32 radix = getRadix(key, startBit);
				const PxU32 writeIndex = a + sRadixSumSum[radix];
		
				gOutputKeys[writeIndex] = key;
				gOutputRanks[writeIndex] = sRanks[a];
			}
		}

		__syncthreads();

		if(idx < RADIX_SIZE)
		{
			sRadixSumBetweenBlocks[idx]+=sRadixCount[idx];
		}
	}
}


static __device__ void radixSortPassSingleWarp(const uint4* PX_RESTRICT gInputKeys, const uint4* PX_RESTRICT gInputRanks, const PxU32 gNumOfKeys, const PxU32 numUint4,
	PxU32* PX_RESTRICT gOutputKeys, PxU32* PX_RESTRICT gOutputRanks, const PxU32 startBit, PxU32* radixExclusiveRunsum)
{
	RadixAccum radixAccum;
#pragma unroll
	for (PxU32 i = 0; i< RADIX_ACCUM_SIZE; ++i)
	{
		radixAccum.radixAccum[i] = 0;
	}

	radixSortWarp(gInputKeys, threadIdx.x, numUint4, WARP_SIZE, startBit, 0, gNumOfKeys, radixAccum);

	PxU32 accumValue = 0;

#pragma unroll
	for (PxU32 i = 0; i<RADIX_SIZE; i += 2)
	{
		const PxU32 accum = radixAccum.radixAccum[i / 2];
		const PxU32 val = warpReduction<AddOpPxU32, PxU32>(FULL_MASK, accum);
		const PxU32 v0 = val & 0xFFFF;
		const PxU32 v1 = val >> 16;

		radixExclusiveRunsum[i] = accumValue;
		accumValue += v0;
		radixExclusiveRunsum[i+1] = accumValue;
		accumValue += v1;
	}

	//Now we loop and output the elements in order from the input buffer to the output buffer...
	__syncwarp();

	for (PxU32 i = 0; i < numUint4; i += WARP_SIZE)
	{
		const PxU32 inputKeyIndex = i + threadIdx.x;
		//All threads enter this stage because we need to do some warp synchronous stuff...
		uint4 keyValue;
		uint4 radix;
		uint4 keyIndex;

		//read 4 elements at a time
		if (inputKeyIndex < numUint4)
		{
			keyIndex = gInputRanks[inputKeyIndex];

			keyValue = gInputKeys[inputKeyIndex];

			sanitizeKeys(keyValue, inputKeyIndex * 4, gNumOfKeys);

			radix = getRadix(keyValue, startBit);
		}
		else
		{
			//pad the extra radix with sufficent large enough number(we have 8 passes and each pass just sort 4 bits so 0xff is sufficent large enough)
			radix.x = radix.y = radix.z = radix.w = 0xff;
		}

		//#pragma unroll
		for (PxU32 i = 0; i<RADIX_SIZE; i += 4)
		{
			PxU32 radixRankX = (radix.x - i);
			PxU32 radixRankY = (radix.y - i);
			PxU32 radixRankZ = (radix.z - i);
			PxU32 radixRankW = (radix.w - i);
			PxU32 accum0 = (1u << (radixRankX << 3));
			PxU32 accum1 = (1u << (radixRankY << 3));
			PxU32 accum2 = (1u << (radixRankZ << 3));
			PxU32 accum3 = (1u << (radixRankW << 3));

			PxU32 accum = accum0 + accum1 + accum2 + accum3;

			PxU32 val = warpScan<AddOpPxU32, PxU32>(FULL_MASK, accum);

			const PxU32 val2 = __shfl_sync(FULL_MASK, val, (WARP_SIZE - 1));

			//Take off how many I have so I have my local offset...
			val -= accum;

			if (accum)
			{
				//We have something in this radix range...output it!
				if (radixRankX < 4)
				{
					PxU32 outputIndex = radixExclusiveRunsum[radix.x] + ((val >> (radixRankX<<3u))&0xFF);
					val += (1 << (radixRankX << 3u));
					gOutputKeys[outputIndex] = keyValue.x;
					gOutputRanks[outputIndex] = keyIndex.x;
				}

				if (radixRankY < 4)
				{
					PxU32 outputIndex = radixExclusiveRunsum[radix.y] + ((val >> (radixRankY << 3u)) & 0xFF);
					val += (1 << (radixRankY << 3u));
					gOutputKeys[outputIndex] = keyValue.y;
					gOutputRanks[outputIndex] = keyIndex.y;
				}

				if (radixRankZ < 4)
				{
					PxU32 outputIndex = radixExclusiveRunsum[radix.z] + ((val >> (radixRankZ << 3u)) & 0xFF);
					val += (1 << (radixRankZ << 3u));
					gOutputKeys[outputIndex] = keyValue.z;
					gOutputRanks[outputIndex] = keyIndex.z;
				}

				if (radixRankW < 4)
				{
					PxU32 outputIndex = radixExclusiveRunsum[radix.w] + ((val >> (radixRankW << 3u)) & 0xFF);
					val += (1 << (radixRankW << 3u));
					gOutputKeys[outputIndex] = keyValue.w;
					gOutputRanks[outputIndex] = keyIndex.w;
				}

			}

			__syncwarp();

			if (threadIdx.x == 0) 
			{
				radixExclusiveRunsum[i] += (val2 & 0xFF);
				radixExclusiveRunsum[i + 1] += ((val2 >> 8) & 0xFF);
				radixExclusiveRunsum[i + 2] += ((val2 >> 16) & 0xFF);
				radixExclusiveRunsum[i + 3] += ((val2 >> 24) & 0xFF);
			}

			__syncwarp();
		}
	}
}

static __device__ void radixSortPassSingleWarpKeysOnly(const uint4* PX_RESTRICT gInputKeys, const PxU32 gNumOfKeys, const PxU32 numUint4,
	PxU32* PX_RESTRICT gOutputKeys, const PxU32 startBit, PxU32* radixExclusiveRunsum)
{
	RadixAccum radixAccum;
#pragma unroll
	for (PxU32 i = 0; i < RADIX_ACCUM_SIZE; ++i)
	{
		radixAccum.radixAccum[i] = 0;
	}

	radixSortWarp(gInputKeys, threadIdx.x, numUint4, WARP_SIZE, startBit, 0, gNumOfKeys, radixAccum);

	PxU32 accumValue = 0;

#pragma unroll
	for (PxU32 i = 0; i < RADIX_SIZE; i += 2)
	{
		const PxU32 accum = radixAccum.radixAccum[i / 2];
		const PxU32 val = warpReduction<AddOpPxU32, PxU32>(FULL_MASK, accum);
		const PxU32 v0 = val & 0xFFFF;
		const PxU32 v1 = val >> 16;

		radixExclusiveRunsum[i] = accumValue;
		accumValue += v0;
		radixExclusiveRunsum[i + 1] = accumValue;
		accumValue += v1;
	}

	//Now we loop and output the elements in order from the input buffer to the output buffer...
	__syncwarp();

	for (PxU32 i = 0; i < numUint4; i += WARP_SIZE)
	{
		const PxU32 inputKeyIndex = i + threadIdx.x;
		//All threads enter this stage because we need to do some warp synchronous stuff...
		uint4 keyValue;
		uint4 radix;

		//read 4 elements at a time
		if (inputKeyIndex < numUint4)
		{
			keyValue = gInputKeys[inputKeyIndex];

			sanitizeKeys(keyValue, inputKeyIndex * 4, gNumOfKeys);

			radix = getRadix(keyValue, startBit);
		}
		else
		{
			//pad the extra radix with sufficent large enough number(we have 8 passes and each pass just sort 4 bits so 0xff is sufficent large enough)
			radix.x = radix.y = radix.z = radix.w = 0xff;
		}

		//#pragma unroll
		for (PxU32 i = 0; i < RADIX_SIZE; i += 4)
		{
			PxU32 radixRankX = (radix.x - i);
			PxU32 radixRankY = (radix.y - i);
			PxU32 radixRankZ = (radix.z - i);
			PxU32 radixRankW = (radix.w - i);
			PxU32 accum0 = (1u << (radixRankX << 3));
			PxU32 accum1 = (1u << (radixRankY << 3));
			PxU32 accum2 = (1u << (radixRankZ << 3));
			PxU32 accum3 = (1u << (radixRankW << 3));

			PxU32 accum = accum0 + accum1 + accum2 + accum3;

			PxU32 val = warpScan<AddOpPxU32, PxU32>(FULL_MASK, accum);

			const PxU32 val2 = __shfl_sync(FULL_MASK, val, (WARP_SIZE - 1));

			//Take off how many I have so I have my local offset...
			val -= accum;

			if (accum)
			{
				//We have something in this radix range...output it!
				if (radixRankX < 4)
				{
					PxU32 outputIndex = radixExclusiveRunsum[radix.x] + ((val >> (radixRankX << 3u)) & 0xFF);
					val += (1 << (radixRankX << 3u));
					gOutputKeys[outputIndex] = keyValue.x;
				}

				if (radixRankY < 4)
				{
					PxU32 outputIndex = radixExclusiveRunsum[radix.y] + ((val >> (radixRankY << 3u)) & 0xFF);
					val += (1 << (radixRankY << 3u));
					gOutputKeys[outputIndex] = keyValue.y;
				}

				if (radixRankZ < 4)
				{
					PxU32 outputIndex = radixExclusiveRunsum[radix.z] + ((val >> (radixRankZ << 3u)) & 0xFF);
					val += (1 << (radixRankZ << 3u));
					gOutputKeys[outputIndex] = keyValue.z;
				}

				if (radixRankW < 4)
				{
					PxU32 outputIndex = radixExclusiveRunsum[radix.w] + ((val >> (radixRankW << 3u)) & 0xFF);
					val += (1 << (radixRankW << 3u));
					gOutputKeys[outputIndex] = keyValue.w;
				}

			}

			__syncwarp();

			if (threadIdx.x == 0)
			{
				radixExclusiveRunsum[i] += (val2 & 0xFF);
				radixExclusiveRunsum[i + 1] += ((val2 >> 8) & 0xFF);
				radixExclusiveRunsum[i + 2] += ((val2 >> 16) & 0xFF);
				radixExclusiveRunsum[i + 3] += ((val2 >> 24) & 0xFF);
			}

			__syncwarp();
		}
	}
}


template <PxU32 nbWarps>
static __device__ void radixSortSingleWarp(uint4* PX_RESTRICT gInputKeys, uint4* PX_RESTRICT gInputRanks, const PxU32 gNumOfKeys, const PxU32 numUint4,
	uint4* PX_RESTRICT gTempKeys, uint4* PX_RESTRICT gTempRanks, const PxU32 nbPasses)
{
	__shared__ PxU32 radixExclusiveRunsum[nbWarps][RADIX_SIZE];
	uint4* PX_RESTRICT k0 = gInputKeys;
	uint4* PX_RESTRICT k1 = gTempKeys;

	uint4* PX_RESTRICT r0 = gInputRanks;
	uint4* PX_RESTRICT r1 = gTempRanks;

	for (PxU32 i = 0, startBit = 0; i < nbPasses; ++i, startBit += 4)
	{
		radixSortPassSingleWarp(k0, r0, gNumOfKeys, numUint4, reinterpret_cast<PxU32*>(k1), reinterpret_cast<PxU32*>(r1), startBit,
			radixExclusiveRunsum[threadIdx.y]);
		//Swap buffers
		uint4* PX_RESTRICT t = k0; k0 = k1; k1 = t;
		t = r0; r0 = r1; r1 = t;
	}
}

template <PxU32 nbWarps>
static __device__ void radixSortSingleWarpKeysOnly(uint4* PX_RESTRICT gInputKeys, const PxU32 gNumOfKeys, const PxU32 numUint4,
	uint4* PX_RESTRICT gTempKeys, const PxU32 nbPasses)
{
	__shared__ PxU32 radixExclusiveRunsum[nbWarps][RADIX_SIZE];
	uint4* PX_RESTRICT k0 = gInputKeys;
	uint4* PX_RESTRICT k1 = gTempKeys;

	for (PxU32 i = 0, startBit = 0; i < nbPasses; ++i, startBit += 4)
	{
		radixSortPassSingleWarpKeysOnly(k0, gNumOfKeys, numUint4, reinterpret_cast<PxU32*>(k1), startBit,
			radixExclusiveRunsum[threadIdx.y]);
		//Swap buffers
		uint4* PX_RESTRICT t = k0; k0 = k1; k1 = t;
	}
}

/* bitonic sorting network for 32 inputs */
/* sorts in-place without extra storage  */
PX_FORCE_INLINE __device__ void bitonicSortWarp(const PxU32 mask, PxU32& key, PxU32& val)
{
	const PxU32 laneId = threadIdx.x & 0x1f;
	/* only if the complete warp participates */
	if (mask == UINT_MAX)
	{
		PxU32 sKey; PxReal sVal; bool swap;
		for (int k = 2; k <= 32; k <<=1)
		{
			for (PxU32 stride = k / 2; stride > 0; stride >>= 1)
			{
				sKey = __shfl_xor_sync(mask, key, stride);
				sVal = __shfl_xor_sync(mask, val, stride);	
				swap = (((laneId & stride) != 0 ? val > sVal : val < sVal))^((laneId&k) == 0);
				key = swap ? sKey : key, val = swap ? sVal : val;
			}
		}
	}
}

#endif // !PXG_RADIX_SORT_CUH
