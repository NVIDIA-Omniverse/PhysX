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

#include "foundation/PxPreprocessor.h"

#include "foundation/PxSimpleTypes.h"
#include "PxgSolverCoreDesc.h"
#include "PxgRadixSortDesc.h"
#include "DyThresholdTable.h"
#include "RadixSort.cuh"
#include "PxgRadixSortDesc.h"
#include "PxgCommonDefines.h"

#include "reduction.cuh"
#include <stdio.h>
#include "PxgSolverKernelIndices.h"

using namespace physx;

extern "C" __host__ void initSolverKernels0() {}

extern "C" __global__ void bodyInputAndRanksSingleBlockLaunch(const PxgSolverCoreDesc* solverDesc, const PxgRadixSortDesc* desc, const PxU32 gStartBit)
{
	uint4* gInputKeys = reinterpret_cast<uint4*>(desc->inputKeys);
	uint4* gInputRanks = reinterpret_cast<uint4*>(desc->inputRanks);
	PxU32* gRadixCount = desc->radixBlockCounts;
	const PxU32 gNumOfKeys = solverDesc->sharedThresholdStreamIndex;

	radixSortSingleBlock<PxgKernelBlockDim::RADIXSORT/WARP_SIZE>(gInputKeys, gInputRanks, gNumOfKeys, gStartBit, gRadixCount);
}

extern "C" __global__ void bodyInputAndRanksBlocksLaunch(const PxgSolverCoreDesc* solverDesc, PxgRadixSortDesc* desc, const PxU32 gStartBit)
{
	uint4* gInputKeys = reinterpret_cast<uint4*>(desc->inputKeys);
	uint4* gInputRanks = reinterpret_cast<uint4*>(desc->inputRanks);
	PxU32* gOutputKeys = desc->outputKeys;
	PxU32* gOutputRanks = desc->outputRanks;

	PxU32* gRadixCount = desc->radixBlockCounts;
	const PxU32 gNumOfKeys = solverDesc->sharedThresholdStreamIndex;

	radixSortCalculateRanks<PxgKernelBlockDim::RADIXSORT / WARP_SIZE>( gInputKeys, gInputRanks, gNumOfKeys, gStartBit, gRadixCount, gOutputKeys, gOutputRanks);
}

extern "C" __global__ void initialRanksAndBodyIndexB(const PxgSolverCoreDesc* solverDesc, const PxgRadixSortDesc* rsDesc)
{
	Dy::ThresholdStreamElement* thresholdStream = solverDesc->thresholdStream;
	const PxU32 nbThresholdElements = solverDesc->sharedThresholdStreamIndex;

	PxU32* gInputKeys = rsDesc->inputKeys;
	PxU32* gInputRanks = rsDesc->inputRanks;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x*blockIdx.x;

	for(PxU32 i=globalThreadIndex; i<nbThresholdElements; i+=blockDim.x*gridDim.x)
	{
		Dy::ThresholdStreamElement& elements =  thresholdStream[i];
		gInputKeys[i] = elements.nodeIndexB.index();
		gInputRanks[i] = i;
	}

	//we need to pad the handles to the multiply of 4 for the radix sort
	const PxU32 remainingThresholdElements = (4 - (nbThresholdElements & 3)) & 3;

	for(PxU32 i=globalThreadIndex; i < remainingThresholdElements; i+=blockDim.x*gridDim.x)
	{
		const PxU32 index = i + nbThresholdElements;
		gInputKeys[index]  = 0xffffffff;
		gInputRanks[index] = index;
	}

}

extern "C" __global__ void initialRanksAndBodyIndexA(const PxgSolverCoreDesc* solverDesc, const PxgRadixSortDesc* rsDesc)
{
	Dy::ThresholdStreamElement* thresholdStream = solverDesc->thresholdStream;
	const PxU32 nbThresholdElements = solverDesc->sharedThresholdStreamIndex;

	//we need to use the inputRanks from the bodyAIndex to reorganize the threshold stream
	PxU32* gInputKeys = rsDesc->inputKeys;
	PxU32* gInputRanks = rsDesc->inputRanks;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x*blockIdx.x;

	for(PxU32 i=globalThreadIndex; i<nbThresholdElements; i+=blockDim.x*gridDim.x)
	{
		Dy::ThresholdStreamElement& elements =  thresholdStream[gInputRanks[i]];
		gInputKeys[i] = elements.nodeIndexA.index();
	}

	//we need to pad the handles to the multiply of 4 for the radix sort
	const PxU32 remainingThresholdElements = (4 - (nbThresholdElements & 3)) & 3;

	for(PxU32 i=globalThreadIndex; i < remainingThresholdElements; i+=blockDim.x*gridDim.x)
	{
		const PxU32 index = i + nbThresholdElements;
		gInputKeys[index]  = 0xffffffff;
	}

}

extern "C" __global__ void reorganizeThresholdElements(const PxgSolverCoreDesc* solverDesc, const PxgRadixSortDesc* rsDesc)
{
	Dy::ThresholdStreamElement* thresholdStream = solverDesc->thresholdStream;
	Dy::ThresholdStreamElement* tmpThresholdStream = solverDesc->tmpThresholdStream;

	const PxU32 nbThresholdElements = solverDesc->sharedThresholdStreamIndex;

	const PxU32* gInputRanks = rsDesc->inputRanks;
	PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	for(PxU32 i = globalThreadIdx/8; i < nbThresholdElements; i+=((blockDim.x*gridDim.x)/8))
	{
		PxU32* dest = reinterpret_cast<PxU32*>(thresholdStream + i);
		PxU32* src = reinterpret_cast<PxU32*>(tmpThresholdStream + gInputRanks[i]);
		
		dest[threadIdx.x&7] = src[threadIdx.x&7];
	}

}

extern "C" __global__ void computeAccumulateThresholdStream(PxgSolverCoreDesc* solverDesc)
{
	const PxU32 nbBlocks = PxgKernelGridDim::COMPUTE_ACCUMULATED_THRESHOLDSTREAM;
	PX_COMPILE_TIME_ASSERT(nbBlocks == 32);

	const PxU32 WARP_PERBLOCK_SIZE = PxgKernelBlockDim::COMPUTE_ACCUMULATED_THRESHOLDSTREAM / WARP_SIZE;

	const PxU32 LOG2_WARP_PERBLOCK_SIZE = 3;

	assert(WARP_PERBLOCK_SIZE == (1 << LOG2_WARP_PERBLOCK_SIZE));

	__shared__ PxReal sWarpAccumulator[WARP_PERBLOCK_SIZE];
	__shared__ PxReal sBlockAccumulator;

	__shared__ PxU32 sWarpPairsAccumulator[WARP_PERBLOCK_SIZE];
	__shared__ PxU32 sBlockPairsAccumulator;

	//Each body can be made of multiple shapes, therefore, we need to accumulated difference forces from the shapes to the body pairs. In this case, we will have thresholdStreams have
	//same bodyAIndex and bodyBIndex

	//The threshold stream has been sorted based on the bodyAIndex and bodyBIndex, therefore, if pairs have the same bodyAIndex and bodyBIndex, they will laied in continuously memory
	Dy::ThresholdStreamElement* gThresholdStream = solverDesc->thresholdStream;

	PxReal* gThresholdStreamAccumulatedForce = solverDesc->thresholdStreamAccumulatedForce;
	PxReal* gThresholdStreamAccumulatedForceBetweenBlocks = solverDesc->thresholdStreamAccumulatedForceBetweenBlocks;

	PxU32* gThresholdStreamWriteIndex = solverDesc->thresholdStreamWriteIndex;
	PxU32* gThresholdStreamWriteIndexBetweenBlocks = solverDesc->thresholdStreamWriteIndexBetweenBlocks;

	bool* gThresholdStreamWriteable = solverDesc->thresholdStreamWriteable;

	const PxU32 nbThresholdElements = solverDesc->sharedThresholdStreamIndex;

	const PxU32 totalBlockRequired = (nbThresholdElements + (blockDim.x-1))/ blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (nbBlocks-1))/ nbBlocks;

	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE-1);
	const PxU32 warpIndex = threadIdx.x/(WARP_SIZE);

	const PxU32 idx = threadIdx.x;

	if(threadIdx.x == 0)
	{
		sBlockAccumulator = 0;
		sBlockPairsAccumulator = 0;
	}

	__syncthreads();

	for(PxU32 i=0; i<numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = idx + i*blockDim.x + blockDim.x * blockIdx.x *numIterationPerBlock; 
	
		PxReal val = 0.f;
		bool isNewPair = false;
		PxNodeIndex nodeIndexA(PX_INVALID_NODE);
		PxNodeIndex nodeIndexB(PX_INVALID_NODE);
	
		if(workIndex < nbThresholdElements)
		{
			val = gThresholdStream[workIndex].normalForce;
			nodeIndexA = gThresholdStream[workIndex].nodeIndexA;
			nodeIndexB = gThresholdStream[workIndex].nodeIndexB;

			if(workIndex+1 < nbThresholdElements)
			{
				Dy::ThresholdStreamElement& nElement = gThresholdStream[workIndex+1];
				if(!(nodeIndexA == nElement.nodeIndexA && nodeIndexB == nElement.nodeIndexB))
				{
					isNewPair = true;
				}
			}
			else
			{
				isNewPair = true;
			}
		}

		//warpScan is inclusive add but the accumVal is exclusive add result
		const PxReal accumVal = warpScan<AddOpPxReal, PxReal>(FULL_MASK, val) - val;

		const PxU32 threadMask = (1<<threadIndexInWarp)-1;

		const PxU32 accumPairs = __popc(__ballot_sync(FULL_MASK, isNewPair)&threadMask);

		if(threadIndexInWarp == (WARP_SIZE-1))
		{
			sWarpAccumulator[warpIndex] = accumVal + val;
			sWarpPairsAccumulator[warpIndex] = accumPairs + isNewPair;
		}

		const PxReal prevBlockAccumulator = sBlockAccumulator;
		const PxU32 prevsBlockPairsAccumulator = sBlockPairsAccumulator;

		__syncthreads();

		unsigned mask_idx = __ballot_sync(FULL_MASK, idx < WARP_PERBLOCK_SIZE);
		if(idx < WARP_PERBLOCK_SIZE)
		{
			PxReal forceVal = sWarpAccumulator[idx];

			const PxReal accumulatedForce = warpScan<AddOpPxReal, PxReal, LOG2_WARP_PERBLOCK_SIZE>(mask_idx, forceVal) - forceVal;
			sWarpAccumulator[idx] = accumulatedForce;
		
			PxU32 pairVal = sWarpPairsAccumulator[idx];
			const PxU32 accumulatedPairs = warpScan<AddOpPxU32, PxU32, LOG2_WARP_PERBLOCK_SIZE>(mask_idx, pairVal) - pairVal;
			sWarpPairsAccumulator[idx] = accumulatedPairs;

			if(threadIndexInWarp == (WARP_PERBLOCK_SIZE-1))
			{
				sBlockAccumulator += (accumulatedForce + forceVal);
				sBlockPairsAccumulator +=(accumulatedPairs + pairVal);
			}
			
		}
		
		__syncthreads();

		if(workIndex < nbThresholdElements)
		{
			//accumVal is exclusive result within a warp and sWarpAccumulator is the exclusive result within a block
			gThresholdStreamAccumulatedForce[workIndex] = val + accumVal + prevBlockAccumulator + sWarpAccumulator[warpIndex]; //this is inclusive
			gThresholdStreamWriteIndex[workIndex] = accumPairs + prevsBlockPairsAccumulator + sWarpPairsAccumulator[warpIndex];
			gThresholdStreamWriteable[workIndex] = isNewPair;
		}

	}
	
	if(threadIdx.x == 0)
	{
		gThresholdStreamAccumulatedForceBetweenBlocks[blockIdx.x] = sBlockAccumulator;
		gThresholdStreamWriteIndexBetweenBlocks[blockIdx.x] = sBlockPairsAccumulator;
	}

}

extern "C" __global__ void outputAccumulateThresholdStream(PxgSolverCoreDesc* solverDesc)
{
	const PxU32 nbBlocks = PxgKernelGridDim::OUTPUT_ACCUMULATED_THRESHOLDSTREAM;
	PX_COMPILE_TIME_ASSERT(nbBlocks == 32);

	PxReal* gThresholdStreamAccumulatedForce = solverDesc->thresholdStreamAccumulatedForce;
	PxReal* gThresholdStreamAccumulatedForceBetweenBlocks = solverDesc->thresholdStreamAccumulatedForceBetweenBlocks;

	PxU32* gThresholdStreamWriteIndex = solverDesc->thresholdStreamWriteIndex;
	PxU32* gThresholdStreamWriteIndexBetweenBlocks = solverDesc->thresholdStreamWriteIndexBetweenBlocks;

	
	const PxU32 nbThresholdElements = solverDesc->sharedThresholdStreamIndex;

	__shared__ PxReal sBlockForceAccum[nbBlocks];
	__shared__ PxU32 sBlockWriteIndexAccum[nbBlocks];

	const PxU32 idx = threadIdx.x;
//	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE-1);

	PxReal val = 0;
	PxReal res = 0;
	PxU32 pairIndex = 0;
	PxU32 pairRes = 0;

	unsigned mask_idx = __ballot_sync(FULL_MASK, idx < nbBlocks);
	if(idx < nbBlocks)
	{
		val = gThresholdStreamAccumulatedForceBetweenBlocks[idx];
		pairIndex = gThresholdStreamWriteIndexBetweenBlocks[idx];

		res = warpScan<AddOpPxReal, PxReal>(mask_idx, val) - val;
		pairRes = warpScan<AddOpPxU32, PxU32>(mask_idx, pairIndex) - pairIndex;

		sBlockForceAccum[idx] = res;
		sBlockWriteIndexAccum[idx] = pairRes;
	}

	__syncthreads();

	const PxU32 totalBlockRequired = (nbThresholdElements + (blockDim.x-1))/ blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (nbBlocks-1))/ nbBlocks;
	
	const PxReal blockForceAccum = sBlockForceAccum[blockIdx.x];

	const PxU32 blockWriteIndexAccum = sBlockWriteIndexAccum[blockIdx.x];

	//accumulate normal force between blocks
	for(PxU32 i=0; i<numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		if(workIndex < nbThresholdElements)
		{
			gThresholdStreamWriteIndex[workIndex] = gThresholdStreamWriteIndex[workIndex] + blockWriteIndexAccum;

			gThresholdStreamAccumulatedForce[workIndex] = gThresholdStreamAccumulatedForce[workIndex] + blockForceAccum;
			
		}
	}

}

extern "C" __global__ void writeoutAccumulatedForcePerObject(PxgSolverCoreDesc* solverDesc)
{
	
	PxReal* gAccumulatedForces = solverDesc->thresholdStreamAccumulatedForce;
	PxU32* gThresholdStreamWriteIndex = solverDesc->thresholdStreamWriteIndex;
	bool* gThresholdStreamWriteable = solverDesc->thresholdStreamWriteable;

	PxReal* gAccumulatedForceObjectPairs = solverDesc->accumulatedForceObjectPairs;

	const PxU32 nbThresholdElements = solverDesc->sharedThresholdStreamIndex;

	const PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	//accumulate normal force between blocks
	for(PxU32 workIndex = globalThreadIdx; workIndex < nbThresholdElements; workIndex+=(blockDim.x*gridDim.x))
	{
		const PxU32 writeIndex = gThresholdStreamWriteIndex[workIndex];

		bool isNewPairs = gThresholdStreamWriteable[workIndex];

		if(isNewPairs)
		{
			gAccumulatedForceObjectPairs[writeIndex] = gAccumulatedForces[workIndex];
		}
	}
}


extern "C" __global__ void computeExceededForceThresholdElementIndice(PxgSolverCoreDesc* solverDesc,
	PxgSolverSharedDesc<IterativeSolveData>* sharedDesc)
{
	const PxU32 nbBlocks = PxgKernelGridDim::COMPUTE_EXCEEDEDFORCE_THRESHOLDELEMENT_INDICE;
	PX_COMPILE_TIME_ASSERT(nbBlocks == 32);

	const PxU32 WARP_PERBLOCK_SIZE = PxgKernelBlockDim::COMPUTE_EXCEEDEDFORCE_THRESHOLDELEMENT_INDICE / WARP_SIZE;

	const PxU32 LOG2_WARP_PERBLOCK_SIZE = 3;

	assert((1 << LOG2_WARP_PERBLOCK_SIZE) == WARP_PERBLOCK_SIZE);

	__shared__ PxU32 sWarpPairsAccumulator[WARP_PERBLOCK_SIZE];
	__shared__ PxU32 sBlockPairsAccumulator;

	const PxReal dt = sharedDesc->dt;

	Dy::ThresholdStreamElement* gThresholdStream = solverDesc->thresholdStream;
	PxReal* gAccumulatedForceObjectPairs = solverDesc->accumulatedForceObjectPairs;

	PxU32* gThresholdStreamWriteIndex = solverDesc->thresholdStreamWriteIndex;
	PxU32* gThresholdStreamWriteIndexBetweenBlocks = solverDesc->thresholdStreamWriteIndexBetweenBlocks;
	bool* gThresholdStreamWriteable = solverDesc->thresholdStreamWriteable;

	const PxU32 nbThresholdElements = solverDesc->sharedThresholdStreamIndex;

	const PxU32 totalBlockRequired = (nbThresholdElements + (blockDim.x-1))/ blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (nbBlocks-1))/ nbBlocks;

	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE-1);
	const PxU32 warpIndex = threadIdx.x/(WARP_SIZE);

	const PxU32 idx = threadIdx.x;

	if(threadIdx.x == 0)
	{
		sBlockPairsAccumulator = 0;
	}

	__syncthreads();

	for(PxU32 i=0; i<numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = idx + i*blockDim.x + blockDim.x * blockIdx.x *numIterationPerBlock; 

		bool isExceededForce = false;
	
		if(workIndex < nbThresholdElements)
		{
			Dy::ThresholdStreamElement& element = gThresholdStream[workIndex];

			//we are reusing the write index buffer. However, because the work index is the same, so as long as we read before we write, it should be safe
			const PxU32 writeIndex = gThresholdStreamWriteIndex[workIndex];
			PxReal accumulatedForce = gAccumulatedForceObjectPairs[writeIndex];

			if(writeIndex > 0)
			{
				accumulatedForce -=  gAccumulatedForceObjectPairs[writeIndex-1];
			}

			//write back the accumulated force
			element.accumulatedForce = accumulatedForce;

			isExceededForce = accumulatedForce > (element.threshold * dt);
		}

		const PxU32 threadMask = (1<<threadIndexInWarp)-1;

		const PxU32 accumPairs = __popc(__ballot_sync(FULL_MASK, isExceededForce)&threadMask);

		if(threadIndexInWarp == (WARP_SIZE-1))
		{
			sWarpPairsAccumulator[warpIndex] = accumPairs + isExceededForce;
		}

		const PxU32 prevsBlockPairsAccumulator = sBlockPairsAccumulator;

		__syncthreads();

		unsigned mask_idx = __ballot_sync(FULL_MASK, idx < WARP_PERBLOCK_SIZE);
		if(idx < WARP_PERBLOCK_SIZE)
		{
			
			PxU32 pairVal = sWarpPairsAccumulator[idx];
			const PxU32 accumulatedPairs = warpScan<AddOpPxU32, PxU32, LOG2_WARP_PERBLOCK_SIZE>(mask_idx, pairVal) - pairVal;
			sWarpPairsAccumulator[idx] = accumulatedPairs;

			if(threadIndexInWarp == (WARP_PERBLOCK_SIZE-1))
			{
				sBlockPairsAccumulator +=(accumulatedPairs + pairVal);
			}
			
		}
		
		__syncthreads();

		if(workIndex < nbThresholdElements)
		{
			gThresholdStreamWriteIndex[workIndex] = accumPairs + prevsBlockPairsAccumulator + sWarpPairsAccumulator[warpIndex];
			gThresholdStreamWriteable[workIndex] = isExceededForce;
		}

	}
	
	if(threadIdx.x == 0)
	{
		gThresholdStreamWriteIndexBetweenBlocks[blockIdx.x] = sBlockPairsAccumulator;
	}

}

extern "C" __global__ void outputExceededForceThresholdElementIndice(PxgSolverCoreDesc* solverDesc)
{
	const PxU32 nbBlocks = PxgKernelGridDim::OUTPUT_EXCEEDEDFORCE_THRESHOLDELEMENT_INDICE;
	PX_COMPILE_TIME_ASSERT(nbBlocks == 32);

	const PxU32 WARP_PERBLOCK_SIZE = PxgKernelBlockDim::OUTPUT_EXCEEDEDFORCE_THRESHOLDELEMENT_INDICE/WARP_SIZE;
	
	PxU32* gThresholdStreamWriteIndex = solverDesc->thresholdStreamWriteIndex;
	PxU32* gThresholdStreamWriteIndexBetweenBlocks = solverDesc->thresholdStreamWriteIndexBetweenBlocks;

	const PxU32 nbThresholdElements = solverDesc->sharedThresholdStreamIndex;
	bool* gThresholdStreamWriteable = solverDesc->thresholdStreamWriteable;
	Dy::ThresholdStreamElement* gThresholdElements = solverDesc->thresholdStream;
	Dy::ThresholdStreamElement* gExceededForceElements = solverDesc->exceededForceElements;

	__shared__ PxU32 sBlockWriteIndexAccum[nbBlocks];

	const PxU32 idx = threadIdx.x;
//	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE-1);

	PxU32 pairIndex = 0;
	PxU32 pairRes = 0;

	unsigned mask_idx = __ballot_sync(FULL_MASK, idx < nbBlocks);
	if(idx < nbBlocks)
	{
		pairIndex = gThresholdStreamWriteIndexBetweenBlocks[idx];
		pairRes = warpScan<AddOpPxU32, PxU32>(mask_idx, pairIndex) - pairIndex;

		sBlockWriteIndexAccum[idx] = pairRes;
	}

	__syncthreads();

	const PxU32 totalBlockRequired = (nbThresholdElements + (blockDim.x-1))/ blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (nbBlocks-1))/ nbBlocks;
	
	const PxU32 blockWriteIndexAccum = sBlockWriteIndexAccum[blockIdx.x];

	//accumulate normal force between blocks
	for(PxU32 i=0; i<numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i*WARP_SIZE*WARP_PERBLOCK_SIZE + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		if(workIndex < nbThresholdElements)
		{
			///gThresholdStreamWriteIndex[workIndex] = gThresholdStreamWriteIndex[workIndex] + blockWriteIndexAccum;

			const PxU32 writeIndex = gThresholdStreamWriteIndex[workIndex] + blockWriteIndexAccum;

			bool isExceededForce = gThresholdStreamWriteable[workIndex];

			if(isExceededForce)
			{
				Dy::ThresholdStreamElement& element = gThresholdElements[workIndex];
				Dy::ThresholdStreamElement tempElement;
				tempElement.shapeInteraction = element.shapeInteraction;
				tempElement.nodeIndexA = element.nodeIndexA;
				tempElement.nodeIndexB = element.nodeIndexB;
				tempElement.normalForce = element.normalForce;
				tempElement.accumulatedForce = element.accumulatedForce;
				tempElement.threshold = element.threshold;
				gExceededForceElements[writeIndex] = tempElement;
			}

			//last element
			if(workIndex == nbThresholdElements -1)
			{
				solverDesc->nbExceededThresholdElements = isExceededForce ? (writeIndex + 1) : writeIndex;
			}
		}
	}
}


//we will insure nbPrevExceededThresholdPairs > 0 in the CPU and all pair masks have been set to 1
//The data laied out in gExceededForceElementMask is previous exceeded threshold element mask first, then current exceeded threshold element mask second.
//persistent exceeded threshold element mask last. Persistent exceeded elements have to be in the previous exceeded threshold elements array and current 
//exceeded threshold elements array so the number of persistent exceeded elements will be less than or equal to the previous exceeded threshold elements. 
//Therefore, Persistent exceeded thresold element maks has the same size as the previous exceeded threshold and corresponding to the same element as in
//previous exceeded force element. 
extern "C" __global__ void setThresholdElementsMask(PxgSolverCoreDesc* solverDesc)
{
	Dy::ThresholdStreamElement* gExceededForceElements = solverDesc->exceededForceElements;
	Dy::ThresholdStreamElement* gPrevExceededForceElements = solverDesc->prevExceededForceElements;
	PxU32* gExceededForceElementMask = solverDesc->thresholdStreamWriteIndex;

	const PxU32 nbExceededThresholdElements = solverDesc->nbExceededThresholdElements;
	const PxU32 nbPrevExceededThresholdElements = solverDesc->nbPrevExceededThresholdElements;

	const PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	for(PxU32 workIndex = globalThreadIdx; workIndex < nbExceededThresholdElements; workIndex+=(blockDim.x*gridDim.x))
	{
		Dy::ThresholdStreamElement& element = gExceededForceElements[workIndex];

		//this will find the last element match the element if value exist in the array 
		PxU32 pos = binarySearch<Dy::ThresholdStreamElement>(gPrevExceededForceElements, nbPrevExceededThresholdElements, element);
		Dy::ThresholdStreamElement* prePair = &gPrevExceededForceElements[pos];

		bool done = false;

		while (!done)
		{
			done = true;
			if (prePair->nodeIndexA == element.nodeIndexA && prePair->nodeIndexB == element.nodeIndexB)
			{
				if (prePair->shapeInteraction == element.shapeInteraction)
				{
					//found a pair, raise 0 in the masks so that we won't generate any force change event. Because the mask array store previous and current exceeded force pairs, we need to
					// raise 0 in two position: one for the previous mask and one for the current mask
					gExceededForceElementMask[pos] = 0;
					gExceededForceElementMask[nbPrevExceededThresholdElements + workIndex] = 0;
				}
				else if (pos > 1)
				{
					pos = pos - 1;
					prePair = &gPrevExceededForceElements[pos];
					done = false;
				}
			}
		}

	}
}

__device__ void setPersistentForceElementMask(PxgSolverCoreDesc* solverDesc)
{
	PxU32* gExceededForceElementMask = solverDesc->thresholdStreamWriteIndex;

	const PxU32 nbExceededThresholdElements = solverDesc->nbExceededThresholdElements;
	const PxU32 nbPrevExceededThresholdElements = solverDesc->nbPrevExceededThresholdElements;

	const PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	const PxU32 persistentExceededStart = nbPrevExceededThresholdElements + nbExceededThresholdElements;

	for (PxU32 workIndex = globalThreadIdx; workIndex < nbPrevExceededThresholdElements; workIndex += (blockDim.x*gridDim.x))
	{
		//based on the previous exceeded force elements
		gExceededForceElementMask[persistentExceededStart + workIndex] = !gExceededForceElementMask[workIndex];
	}
}

extern "C" __global__ void computeThresholdElementMaskIndices(PxgSolverCoreDesc* solverDesc)
{
	//this function should be called in setThresholdElementsMask. However, if there are no preExceededThresholdElements(which we will know in the CPU code), we don't
	//kick of setThresholdElementMask kernel at all so the persistentExceededThresholdElementMask is still set to be one. Therefore, we need to call the setPersistentForceElement
	//method in here

	const PxU32 nbBlocks = PxgKernelGridDim::COMPUTE_THRESHOLDELEMENT_MASK_INDICES;
	PX_COMPILE_TIME_ASSERT(nbBlocks == 32);

	const PxU32 WARP_PERBLOCK_SIZE = PxgKernelBlockDim::COMPUTE_THRESHOLDELEMENT_MASK_INDICES / WARP_SIZE;

	const PxU32 LOG2_WARP_PERBLOCK_SIZE = 3;

	assert((1 << LOG2_WARP_PERBLOCK_SIZE) == WARP_PERBLOCK_SIZE);

	setPersistentForceElementMask(solverDesc);

	__shared__ PxU32 sWarpPairsAccumulator[WARP_PERBLOCK_SIZE];
	__shared__ PxU32 sBlockPairsAccumulator;

	PxU32* gExceededForceElementMask = solverDesc->thresholdStreamWriteIndex;
	PxU32* gExceededForceElementMaskBetweenBlocks = solverDesc->thresholdStreamWriteIndexBetweenBlocks;
	bool* gThresholdStreamWriteable = solverDesc->thresholdStreamWriteable;

	const PxU32 nbExceededThresholdElements = solverDesc->nbExceededThresholdElements;
	const PxU32 nbPrevExceededThresholdElements = solverDesc->nbPrevExceededThresholdElements;

	//prev, current and persistent
	const PxU32 totalNbExceededThresholdElements = nbExceededThresholdElements + nbPrevExceededThresholdElements*2;

	const PxU32 totalBlockRequired = (totalNbExceededThresholdElements + (blockDim.x-1))/ blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (nbBlocks-1))/ nbBlocks;

	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE-1);
	const PxU32 warpIndex = threadIdx.x/(WARP_SIZE);

	const PxU32 idx = threadIdx.x;

	if(threadIdx.x == 0)
	{
		sBlockPairsAccumulator = 0;
	}

	__syncthreads();


	for(PxU32 i=0; i<numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = idx + i*blockDim.x + blockDim.x * blockIdx.x *numIterationPerBlock; 
	
		PxU32 forceChangeMask = 0;

		if(workIndex < totalNbExceededThresholdElements)
		{
			forceChangeMask = gExceededForceElementMask[workIndex];
		}

		const PxU32 threadMask = (1<<threadIndexInWarp)-1;

		const PxU32 accumPairs = __popc(__ballot_sync(FULL_MASK, forceChangeMask)&threadMask);

		if(threadIndexInWarp == (WARP_SIZE-1))
		{
			sWarpPairsAccumulator[warpIndex] = accumPairs + forceChangeMask;
		}

		const PxU32 prevsBlockPairsAccumulator = sBlockPairsAccumulator;

		__syncthreads();

		unsigned mask_idx = __ballot_sync(FULL_MASK, idx < WARP_PERBLOCK_SIZE);
		if(idx < WARP_PERBLOCK_SIZE)
		{
			PxU32 pairVal = sWarpPairsAccumulator[idx];
			const PxU32 accumulatedPairs = warpScan<AddOpPxU32, PxU32, LOG2_WARP_PERBLOCK_SIZE>(mask_idx, pairVal) - pairVal;
			sWarpPairsAccumulator[idx] = accumulatedPairs;

			if(threadIndexInWarp == (WARP_PERBLOCK_SIZE-1))
			{
				sBlockPairsAccumulator +=(accumulatedPairs + pairVal);
			}
			
		}
		
		__syncthreads();

		if(workIndex < totalNbExceededThresholdElements)
		{
			gExceededForceElementMask[workIndex] = accumPairs + prevsBlockPairsAccumulator + sWarpPairsAccumulator[warpIndex];
			gThresholdStreamWriteable[workIndex] = !!(forceChangeMask);
		}

	}
	
	if(threadIdx.x == 0)
	{
		gExceededForceElementMaskBetweenBlocks[blockIdx.x] = sBlockPairsAccumulator;
	}
}

extern "C" __global__ void outputThresholdPairsMaskIndices(PxgSolverCoreDesc* solverDesc)
{
	const PxU32 nbBlocks = PxgKernelGridDim::OUTPUT_THRESHOLDELEMENT_MASK_INDICES;
	PX_COMPILE_TIME_ASSERT(nbBlocks == 32);

	__shared__ PxU32 sBlockWriteIndexAccum[nbBlocks];

	PxU32* gExceededForceElementMask = solverDesc->thresholdStreamWriteIndex;
	PxU32* gExceededForceElementMaskBetweenBlocks = solverDesc->thresholdStreamWriteIndexBetweenBlocks;


	const PxU32 nbExceededThresholdElements = solverDesc->nbExceededThresholdElements;
	const PxU32 nbPrevExceededThresholdElements = solverDesc->nbPrevExceededThresholdElements;

	//previous, current and persistent
	const PxU32 totalNbExceededThresholdElements = nbExceededThresholdElements + nbPrevExceededThresholdElements*2;

	const PxU32 idx = threadIdx.x;


	PxU32 pairIndex = 0;
	PxU32 pairRes = 0;

	unsigned mask_idx = __ballot_sync(FULL_MASK, idx < nbBlocks);
	if(idx < nbBlocks)
	{
		pairIndex = gExceededForceElementMaskBetweenBlocks[idx];
		pairRes = warpScan<AddOpPxU32, PxU32>(mask_idx, pairIndex) - pairIndex;

		sBlockWriteIndexAccum[idx] = pairRes;
	}

	__syncthreads();

	const PxU32 totalBlockRequired = (totalNbExceededThresholdElements + (blockDim.x-1))/ blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (nbBlocks-1))/ nbBlocks;
	
	const PxU32 blockWriteIndexAccum = sBlockWriteIndexAccum[blockIdx.x];


	//accumulate normal force between blocks
	for(PxU32 i=0; i<numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i*PxgKernelBlockDim::OUTPUT_THRESHOLDELEMENT_MASK_INDICES + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		if(workIndex < totalNbExceededThresholdElements)
		{
			gExceededForceElementMask[workIndex] = gExceededForceElementMask[workIndex] + blockWriteIndexAccum;
		}
	}
}

extern "C" __global__ void createForceChangeThresholdElements(PxgSolverCoreDesc* solverDesc)
{
	PxU32* gExceededForceElementMask = solverDesc->thresholdStreamWriteIndex;
	Dy::ThresholdStreamElement* gExceededForceElements = solverDesc->exceededForceElements;
	Dy::ThresholdStreamElement* gPrevExceededForceElements = solverDesc->prevExceededForceElements;
	Dy::ThresholdStreamElement* gForceChangeElements = solverDesc->forceChangeThresholdElements;

	//we copy the original mask value to thresholdStreamWriteable in computeThresholdElementMaskIndices so it corresponding with mask
	bool* gThresholdStreamWriteable = solverDesc->thresholdStreamWriteable;
	const PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;
	const PxU32 nbExceededThresholdElements = solverDesc->nbExceededThresholdElements;
	const PxU32 nbPrevExceededThresholdElements = solverDesc->nbPrevExceededThresholdElements;

	//previous, current and persistent
	const PxU32 totalNbExceededThresholdElements = nbExceededThresholdElements + nbPrevExceededThresholdElements*2;
	const PxU32 persistentExceededStart = nbPrevExceededThresholdElements + nbExceededThresholdElements;

	for(PxU32 workIndex = globalThreadIdx; workIndex < totalNbExceededThresholdElements; workIndex+=(blockDim.x*gridDim.x))
	{
		const bool hasForceChangeOrPersistent = gThresholdStreamWriteable[workIndex];

		const PxU32 writeIndex = gExceededForceElementMask[workIndex];

		if (hasForceChangeOrPersistent)
		{
			bool lostPair = workIndex < nbPrevExceededThresholdElements;
			bool foundPair = (workIndex < persistentExceededStart) && !lostPair;

			Dy::ThresholdStreamElement* pair = NULL;
			if (lostPair)
			{
				pair = &gPrevExceededForceElements[workIndex];
			}
			else if (foundPair)
			{
				pair = &gExceededForceElements[workIndex - nbPrevExceededThresholdElements];
			}
			else
			{
				//persistent pair
				pair = &gPrevExceededForceElements[workIndex - persistentExceededStart];
			}

			//Dy::ThresholdStreamElement& pair = lostPair ? gPrevExceededForceElements[workIndex] : gExceededForceElements[workIndex - nbPrevExceededThresholdElements];
		
			Dy::ThresholdStreamElement tempPair;
			tempPair.shapeInteraction = pair->shapeInteraction;
			tempPair.nodeIndexA = pair->nodeIndexA;
			tempPair.nodeIndexB = pair->nodeIndexB;
			tempPair.normalForce = pair->normalForce;
			tempPair.accumulatedForce = lostPair ? 0.f : pair->accumulatedForce;
			tempPair.threshold = pair->threshold;
			
			gForceChangeElements[writeIndex] = tempPair;
		}

		if(workIndex == totalNbExceededThresholdElements-1)
		{
			solverDesc->nbForceChangeElements = hasForceChangeOrPersistent ? (writeIndex + 1) : writeIndex;
		}
	}
}
