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

#include "RadixSort.cuh"
#include "PxgRadixSortDesc.h"
#include "PxgRadixSortKernelIndices.h"
#include "stdio.h"
#include "PxNodeIndex.h"

extern "C" __host__ void initCommonKernels1() {}


//gNumOfKeys is device ptr
extern "C" __global__ 
__launch_bounds__(PxgRadixSortKernelBlockDim::RADIX_SORT, 1)
void radixSortMultiBlockLaunch(PxgRadixSortBlockDesc* desc, const PxU32 gStartBit)
{
	const uint4* PX_RESTRICT gInputKeys = reinterpret_cast<uint4*>(desc[blockIdx.y].inputKeys);
	const uint4* PX_RESTRICT gInputRanks = reinterpret_cast<uint4*>(desc[blockIdx.y].inputRanks);
	PxU32* gRadixCount = desc[blockIdx.y].radixBlockCounts;

	const PxU32 numKeys = *desc[blockIdx.y].numKeys;

	radixSortSingleBlock<PxgRadixSortKernelBlockDim::RADIX_SORT / WARP_SIZE>(gInputKeys, gInputRanks, numKeys, gStartBit, gRadixCount);
}

//gNumOfKeys is device ptr
extern "C" __global__ 
__launch_bounds__(PxgRadixSortKernelBlockDim::RADIX_SORT, 1)
void radixSortMultiCalculateRanksLaunch(PxgRadixSortBlockDesc* desc, const PxU32 gStartBit)
{
	const uint4* PX_RESTRICT gInputKeys = reinterpret_cast<uint4*>(desc[blockIdx.y].inputKeys);
	const uint4* PX_RESTRICT gInputRanks = reinterpret_cast<uint4*>(desc[blockIdx.y].inputRanks);
	PxU32* gOutputKeys = desc[blockIdx.y].outputKeys;
	PxU32* gOutputRanks = desc[blockIdx.y].outputRanks;

	PxU32* gRadixCount = desc[blockIdx.y].radixBlockCounts;

	const PxU32 numKeys = *desc[blockIdx.y].numKeys;

	radixSortCalculateRanks<PxgRadixSortKernelBlockDim::RADIX_SORT / WARP_SIZE>(gInputKeys, gInputRanks, numKeys, gStartBit, gRadixCount, gOutputKeys, gOutputRanks);
}

extern "C" __global__
__launch_bounds__(PxgRadixSortKernelBlockDim::RADIX_SORT, 1)
void radixSortMultiBlockLaunchWithoutCount(PxgRadixSortDesc* desc, const PxU32 gStartBit)
{
	const uint4* PX_RESTRICT gInputKeys = reinterpret_cast<uint4*>(desc[blockIdx.y].inputKeys);
	const uint4* PX_RESTRICT gInputRanks = reinterpret_cast<uint4*>(desc[blockIdx.y].inputRanks);
	PxU32* gRadixCount = desc[blockIdx.y].radixBlockCounts;

	radixSortSingleBlock<PxgRadixSortKernelBlockDim::RADIX_SORT / WARP_SIZE>(gInputKeys, gInputRanks, desc[blockIdx.y].count, gStartBit, gRadixCount);
}

//gNumOfKeys is device ptr
extern "C" __global__
__launch_bounds__(PxgRadixSortKernelBlockDim::RADIX_SORT, 1)
void radixSortMultiCalculateRanksLaunchWithoutCount(PxgRadixSortDesc* desc, const PxU32 gStartBit)
{
	const uint4* PX_RESTRICT gInputKeys = reinterpret_cast<uint4*>(desc[blockIdx.y].inputKeys);
	const uint4* PX_RESTRICT gInputRanks = reinterpret_cast<uint4*>(desc[blockIdx.y].inputRanks);
	PxU32* gOutputKeys = desc[blockIdx.y].outputKeys;
	PxU32* gOutputRanks = desc[blockIdx.y].outputRanks;
	PxU32* gRadixCount = desc[blockIdx.y].radixBlockCounts;

	radixSortCalculateRanks<PxgRadixSortKernelBlockDim::RADIX_SORT / WARP_SIZE>(gInputKeys, gInputRanks, desc[blockIdx.y].count, gStartBit, gRadixCount, gOutputKeys, gOutputRanks);
}


//gNumOfKeys is device ptr
extern "C" __global__ 
__launch_bounds__(PxgRadixSortKernelBlockDim::RADIX_SORT, 1)
void radixSortMultiBlockLaunchWithCount(PxgRadixSortDesc* desc, const PxU32 numKeys, const PxU32 gStartBit)
{
	const uint4* PX_RESTRICT gInputKeys = reinterpret_cast<uint4*>(desc[blockIdx.y].inputKeys);
	const uint4* PX_RESTRICT gInputRanks = reinterpret_cast<uint4*>(desc[blockIdx.y].inputRanks);
	PxU32* gRadixCount = desc[blockIdx.y].radixBlockCounts;

	radixSortSingleBlock<PxgRadixSortKernelBlockDim::RADIX_SORT / WARP_SIZE>(gInputKeys, gInputRanks, numKeys, gStartBit, gRadixCount);
}

//gNumOfKeys is device ptr
extern "C" __global__ 
__launch_bounds__(PxgRadixSortKernelBlockDim::RADIX_SORT, 1)
void radixSortMultiCalculateRanksLaunchWithCount(PxgRadixSortDesc* desc, const PxU32 numKeys, const PxU32 gStartBit)
{
	const uint4* PX_RESTRICT gInputKeys = reinterpret_cast<uint4*>(desc[blockIdx.y].inputKeys);
	const uint4* PX_RESTRICT gInputRanks = reinterpret_cast<uint4*>(desc[blockIdx.y].inputRanks);
	PxU32* gOutputKeys = desc[blockIdx.y].outputKeys;
	PxU32* gOutputRanks = desc[blockIdx.y].outputRanks;
	PxU32* gRadixCount = desc[blockIdx.y].radixBlockCounts;

	radixSortCalculateRanks<PxgRadixSortKernelBlockDim::RADIX_SORT / WARP_SIZE>(gInputKeys, gInputRanks, numKeys, gStartBit, gRadixCount, gOutputKeys, gOutputRanks);
}

extern "C" __global__ void radixSortCopyHigh32Bits(const PxU64* inValue, PxU32* outValue, PxU32* rank, const PxU32* numKeys)
{
	const PxU32 numElems = *numKeys;
	const PxU32 numIternations = (numElems + blockDim.x * gridDim.x - 1) / blockDim.x * gridDim.x;

	for (PxU32 i = 0; i < numIternations; ++i)
	{
		const PxU32 workIndex = threadIdx.x + blockIdx.x * blockDim.x + i * blockDim.x * gridDim.x;

		if (workIndex >= numElems)
			return;

		const PxU32 index = rank[workIndex];
		outValue[workIndex] = PxU32(inValue[index] >> 32);

		//printf("Copy 32 workIndex %i index %i blockDim.x %i gridDim.x %i\n", workIndex, index, blockDim.x, gridDim.x);

	}
}

extern "C" __global__ void radixSortDoubleCopyHigh32Bits(const PxU64 * inValue0, PxU32 * outValue0, PxU32 * rank0, const PxU64 * inValue1, PxU32 * outValue1, PxU32 * rank1, const PxU32 * numKeys)
{
	const PxU32 numElems = *numKeys;
	const PxU32 numIternations = (numElems + blockDim.x * gridDim.x - 1) / blockDim.x * gridDim.x;

	for (PxU32 i = 0; i < numIternations; ++i)
	{
		const PxU32 workIndex = threadIdx.x + blockIdx.x * blockDim.x + i * blockDim.x * gridDim.x;

		if (workIndex >= numElems)
			return;

		const PxU32 index0 = rank0[workIndex];
		outValue0[workIndex] = PxU32(inValue0[index0] >> 32);

		const PxU32 index1 = rank1[workIndex];
		outValue1[workIndex] = PxU32(inValue1[index1] >> 32);

		//printf("Copy 32 workIndex %i index %i blockDim.x %i gridDim.x %i\n", workIndex, index, blockDim.x, gridDim.x);

	}
}


extern "C" __global__ void radixSortCopy(const PxU64* inValue, PxU64* outValue, PxU32* rank, const PxU32* numKeys)
{

	const PxU32 numElems = *numKeys;
	const PxU32 numIternations = (numElems + blockDim.x * gridDim.x - 1) / blockDim.x * gridDim.x;


	for (PxU32 i = 0; i < numIternations; ++i)
	{
		const PxU32 workIndex = threadIdx.x + blockIdx.x * blockDim.x + i * blockDim.x * gridDim.x;

		if (workIndex >= numElems)
			return;

		const PxU32 index = rank[workIndex];
		outValue[workIndex] = inValue[index];

		//const PxNodeIndex nodeIndex = reinterpret_cast<const PxNodeIndex&>(inValue[index]);

		//printf("Copy 64 workIndex %i index %i value %i blockDim.x %i gridDim.x %i\n", workIndex, index, nodeIndex.index(), blockDim.x, gridDim.x);
	}

}

extern "C" __global__ void radixSortDoubleCopy(
	const PxU64 * inValue0, PxU64 * outValue0, PxU32 * rank0, 
	const PxU64 * inValue1, PxU64 * outValue1, PxU32 * rank1, 
	const PxU32 * numKeys)
{

	const PxU32 numElems = *numKeys;
	const PxU32 numIternations = (numElems + blockDim.x * gridDim.x - 1) / blockDim.x * gridDim.x;


	for (PxU32 i = 0; i < numIternations; ++i)
	{
		const PxU32 workIndex = threadIdx.x + blockIdx.x * blockDim.x + i * blockDim.x * gridDim.x;

		if (workIndex >= numElems)
			return;

		const PxU32 index0 = rank0[workIndex];
		outValue0[workIndex] = inValue0[index0];

		const PxU32 index1 = rank1[workIndex];
		outValue1[workIndex] = inValue1[index1];

		//const PxNodeIndex nodeIndex = reinterpret_cast<const PxNodeIndex&>(inValue[index]);

		//printf("Copy 64 workIndex %i index %i value %i blockDim.x %i gridDim.x %i\n", workIndex, index, nodeIndex.index(), blockDim.x, gridDim.x);
	}

}


extern "C" __global__ void radixSortCopyBits2(const PxU64* inValue, PxU32* outValue, PxU32* rank, const PxU32 numKeys,
	const bool lowBit)
{
	const PxU64 lowerMask = 0x00000000ffffffffull;
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	if (globalThreadIndex < numKeys)
	{
		const PxU32 index = rank[globalThreadIndex];

		//this is aggregate, nodeIndex should be 0xffffffff
		if (index == 0xffffffff)
			outValue[globalThreadIndex] = 0xffffffff;
		else
			outValue[globalThreadIndex] = lowBit ? PxU32(inValue[index] & lowerMask) : PxU32(inValue[index] >> 32ll);
	}
}


extern "C" __global__ void radixSortCopy2(const PxU64* inValue, PxU64* outValue, PxU32* rank, const PxU32 numKeys)
{

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	if(globalThreadIndex < numKeys)
	{
		const PxU32 index = rank[globalThreadIndex];
		const bool aggregate = (index == 0xffffffff);
		outValue[globalThreadIndex] = aggregate ? 0xffffffffffffffff : inValue[index];
	}

}



