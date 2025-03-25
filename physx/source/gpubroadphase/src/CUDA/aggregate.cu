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

#include "PxgAggregate.h"
#include "PxgBroadPhaseDesc.h"
#include "PxgCommonDefines.h"
#include "copy.cuh"
#include "foundation/PxBounds3.h"
#include <stdio.h>
#include "PxgBroadPhaseDesc.h"
#include "PxgAggregate.h"
#include "RadixSort.cuh"
#include "PxgIntegerAABB.h"
#include "PxgBroadPhaseKernelIndices.h"
#include "foundation/PxPreprocessor.h"
#include "reduction.cuh"
#include "PxAggregate.h"
#include "PxgSapBox1D.h"
#include "PxgBroadPhasePairReport.h"
#include "BpVolumeData.h"
#include "PxgAggregateDesc.h"

using namespace physx;

extern "C" __host__ void initBroadphaseKernels1() {}


//each warp copy one aggregate
extern "C" __global__ void updateDirtyAggregate(
	PxgAggregate* aggregates,
	const PxU32 numAggregates,
	PxU32*	dirtyAggregateIndices,
	PxgAggregate* dirtyAggregates,
	const PxU32 numDirtyAggregates,
	const PxU32* dirtyBoundIndices,
	const PxU32* dirtyBoundStartIndices
)
{
	const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + threadIdx.y;

	if (globalWarpIndex < numDirtyAggregates)
	{
		const PxU32 index = dirtyAggregateIndices[globalWarpIndex];
		PxgAggregate& agg = aggregates[index];
		PxgAggregate& dirtyAgg = dirtyAggregates[globalWarpIndex];

		PxU32* dBoundIndices = dirtyAgg.updateBoundIndices;

		const PxU32 startIndex = dirtyBoundStartIndices[globalWarpIndex];
		const PxU32 size = dirtyAgg.size;

		/*if (threadIdx.x == 0)
		{
			printf("index %i startIndex %i size %i\n", index, startIndex, size);
		}*/
		
		for (uint i = threadIdx.x; i < size; i += WARP_SIZE)
		{
			PxU32 ind = startIndex + i;
			dBoundIndices[i] = dirtyBoundIndices[ind];
		}

		if (agg.isNew)
		{
			uint* sAgg = reinterpret_cast<uint*>(&dirtyAgg);
			uint* dAgg = reinterpret_cast<uint*>(&agg);

			warpCopy<uint>(dAgg, sAgg, sizeof(PxgAggregate));
		}
		else
		{
			// we only update selectively if this is not new to avoid breaking the double-buffering and writing stale data.
			if (threadIdx.x == 0)
			{
				// This one is tricky - we always need to know the prevSize according to the double-buffering - so in theory
				// we should update prevSize at every step. But we don't have any kernels running over all aggregates,
				// so we cheat by: 1) initializing both prevSize and size when first adding, 2) if we update, we fix prevSize
				// here before the update by writing the old count to it, and 3) updating prevSize at the end of the same step
				// during clearDirty to make sure it's correct for the future. We can do this because they're still part of the
				// dirty lists in that case.
				agg.prevSize = agg.size;

				agg.updateBoundIndices = dirtyAgg.updateBoundIndices; //a list of shape bound index belong to this aggregate
				agg.mIndex = dirtyAgg.mIndex; // need to copy index to make sure we catch invalidations!
				agg.size = dirtyAgg.size;
				agg.filterHint = dirtyAgg.filterHint; 
			}
		}
	}
}

extern "C" __global__ void clearDirtyAggregates(
	PxgAggregate* aggregates,
	const PxU32 numAggregates,
	PxU32*	dirtyAggregateIndices,
	const PxU32 numDirtyAggregates
)
{
	const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + threadIdx.y;

	if (globalWarpIndex < numDirtyAggregates)
	{
		const PxU32 index = dirtyAggregateIndices[globalWarpIndex];
		PxgAggregate& agg = aggregates[index];

		if (threadIdx.x == 0)
		{
			agg.isNew = false;
			agg.prevSize = agg.size;
		}
	}
}

// the kernel below assumes wordsize of the bitmap is 32.
#define BITMAP_WORD_SIZE 32u
PX_COMPILE_TIME_ASSERT((sizeof(PxU32) * 8) == BITMAP_WORD_SIZE);

extern "C" __global__ void markAggregateBoundsUpdatedBitmap(
	PxgAggregateDesc* aggDesc,
	PxU32* changedAABBMgrHandles
)
{
	PxgAggregate* PX_RESTRICT aggregates = aggDesc->aggregates;
	const PxU32 nbAggregates = aggDesc->numAgregates;

	const PxU32 warpIdx = threadIdx.y + blockIdx.x * blockDim.y;

	if (warpIdx < nbAggregates)
	{
		PxgAggregate agg = aggregates[warpIdx];

		if (agg.isValid())
		{
			if(threadIdx.x == 0)
			{
				PxU32 bit = 1 << (agg.mIndex % BITMAP_WORD_SIZE);
				atomicOr(&changedAABBMgrHandles[agg.mIndex / BITMAP_WORD_SIZE], bit);
			}

			PxU32* boundInds = agg.updateBoundIndices;
			const PxU32 nbBounds = agg.size;
			for (PxU32 id = threadIdx.x; id < nbBounds; id += WARP_SIZE)
			{
				PxU32 idx = boundInds[id];
				PxU32 bit = 1 << (idx % BITMAP_WORD_SIZE);
				atomicAnd(&changedAABBMgrHandles[idx / BITMAP_WORD_SIZE], ~bit);
			}
		}
	}
}

extern "C" __global__ void updateAggregateBounds(
	PxgAggregateDesc* aggDesc,
	PxBounds3* bounds,
	PxReal* contactDistance
)
{
	PxgAggregate* PX_RESTRICT aggregates = aggDesc->aggregates;
	const PxU32 nbAggregates = aggDesc->numAgregates;

	const PxU32 warpIdx = threadIdx.y + blockIdx.x * blockDim.y;

	if (warpIdx < nbAggregates)
	{
		PxgAggregate agg = aggregates[warpIdx];
		if (agg.isValid())
		{
			PxU32* boundInds = agg.updateBoundIndices;
			const PxU32 nbBounds = agg.size;

			PxBounds3 aggBounds;
			aggBounds.setEmpty();
			for (PxU32 id = threadIdx.x; id < nbBounds; id += WARP_SIZE)
			{
				PxU32 idx = boundInds[id];
				//Not ideal loading pattern. Might need to use multiple threads to optimize, but hopefully won't be an issue
				PxBounds3 b3 = bounds[idx];
				const PxReal inflate = contactDistance[idx];
				aggBounds.minimum = aggBounds.minimum.minimum(b3.minimum - PxVec3(inflate));
				aggBounds.maximum = aggBounds.maximum.maximum(b3.maximum + PxVec3(inflate));
			}

			//Now do a warp-scan to compute the total bounds...
			for (PxU32 reductionRadius = 1; reductionRadius < WARP_SIZE; reductionRadius *= 2)
			{
				aggBounds.minimum.x = PxMin(__shfl_xor_sync(FULL_MASK, aggBounds.minimum.x, reductionRadius), aggBounds.minimum.x);
				aggBounds.minimum.y = PxMin(__shfl_xor_sync(FULL_MASK, aggBounds.minimum.y, reductionRadius), aggBounds.minimum.y);
				aggBounds.minimum.z = PxMin(__shfl_xor_sync(FULL_MASK, aggBounds.minimum.z, reductionRadius), aggBounds.minimum.z);
				aggBounds.maximum.x = PxMax(__shfl_xor_sync(FULL_MASK, aggBounds.maximum.x, reductionRadius), aggBounds.maximum.x);
				aggBounds.maximum.y = PxMax(__shfl_xor_sync(FULL_MASK, aggBounds.maximum.y, reductionRadius), aggBounds.maximum.y);
				aggBounds.maximum.z = PxMax(__shfl_xor_sync(FULL_MASK, aggBounds.maximum.z, reductionRadius), aggBounds.maximum.z);
			}

			if (threadIdx.x == 0)
			{
				/*printf("Agg bounds %i = (%f, %f, %f, %f, %f, %f)\n", agg.mIndex, aggBounds.minimum.x, aggBounds.minimum.y, 
					aggBounds.minimum.z, aggBounds.maximum.x, aggBounds.maximum.y, aggBounds.maximum.z);*/
				//Not ideal write memory access pattern. Might need to use 6 threads to write. Unfortunately, that either
				//requires shared memory or to reinterpret aggBounds as an array, which will dump it to local mem. Both will be slow
				bounds[agg.mIndex] = aggBounds;
			}

		}
	}
}
extern "C" __global__ void sortAndUpdateAggregateProjections(
	PxgBroadPhaseDesc* desc, 
	PxgAggregateDesc* aggDesc
)
{
	const PxU32 globalWarpIndex = threadIdx.y + blockDim.y * blockIdx.x;
	const PxU32 nbAggregates = aggDesc->numAgregates;
	PxgAggregate* PX_RESTRICT aggregates = aggDesc->aggregates;

	const PxgIntegerAABB* PX_RESTRICT iAABB = desc->newIntegerBounds;

	if (globalWarpIndex < nbAggregates)
	{
		PxgAggregate& agg = aggregates[globalWarpIndex];

		//Check to see if we need to update it!

		if (agg.isValid())
		{
			//We need to sort it because we have either self-collisions or pairs to process
			/*if(threadIdx.x == 0)
				printf("%i agg.mIndex = %i\n", globalWarpIndex, agg.mIndex);*/

			const PxU32 numBounds = agg.size;

			/*if(threadIdx.x == 0)
				printf("numBounds %i\n", numBounds);*/

			PxU32* boundIndices = agg.boundIndices[1];
			PxU32* projections = agg.sortedProjections[1];
			PxU32* handles = agg.sortedHandles[1];
			PxgSapBox1D* sapBox1D = agg.sapBox1D[1];
			PxU32* startMasks = agg.startMasks[1];

			if (threadIdx.x == 0)
			{
				//Flip buffer - projections [1] always stores previous projections if pair is active!

				//printf("%i: Projections[0] = %p, projections[1] = %p\n", globalWarpIndex, agg.sortedProjections[0], agg.sortedProjections[1]);
				agg.sortedProjections[1] = agg.sortedProjections[0];
				agg.sortedProjections[0] = projections;
				agg.sortedHandles[1] = agg.sortedHandles[0];
				agg.sortedHandles[0] = handles;
				agg.sapBox1D[1] = agg.sapBox1D[0];
				agg.sapBox1D[0] = sapBox1D;

				agg.startMasks[1] = agg.startMasks[0];
				agg.startMasks[0] = startMasks;

				PxU32* comp = agg.comparisons[1];
				agg.comparisons[1] = agg.comparisons[0];
				agg.comparisons[0] = comp;

				agg.boundIndices[1] = agg.boundIndices[0];
				agg.boundIndices[0] = boundIndices;
			}

			__syncwarp();

			const PxU32 nbProjections = numBounds * 2;

			const PxU32 oddMask = (threadIdx.x & 1);
			const PxU32 offset = oddMask *3;
			for (PxU32 i = threadIdx.x; i < nbProjections; i += WARP_SIZE)
			{
				const PxU32 id = i / 2;

				// AD: this makes sure we always have the most up-to-date boundIndices info in boundIndices[0]
				// would be great to write directly into the real boundIndices array when updating the bounds but
				// we need to maintain the double-buffering: in case we change the bounds of an aggregate, we need
				// the old boundIndices array for 1 step and then need to overwrite it. So we just always update
				// the one that points to the new bounds from what we submitted on CPU to keep the double-buffering alive.
				const PxU32 handle = agg.updateBoundIndices[id];
			
				PxU32 minMax = iAABB[handle].mMinMax[offset];

				projections[i] = minMax;
				//KS - we do not store the handle in here because we want to be able to index locally inside this aggregate!
				
				// AD: seems like we only use the start/end info in that handle. we cannot rely on the new info being correct - but we anyway have the bitmap as well!
				const PxU32 startEndHandle = createHandle(id, oddMask == 0, false);

				handles[i] = startEndHandle;

				if ((i % 2) == 0)
					boundIndices[id] = agg.updateBoundIndices[id];
			}

			__syncwarp(); //Make sure all writes have completed/are visible...

			const PxU32 nbUint4s = (nbProjections + 3)/4;

			//Now sort...
			const PxU32 nbWarps = PxgBPKernelBlockDim::BP_AGGREGATE_SORT/WARP_SIZE;
			radixSortSingleWarp<nbWarps>(reinterpret_cast<uint4*>(projections), reinterpret_cast<uint4*>(handles), 
				nbProjections, nbUint4s, reinterpret_cast<uint4*>(projections) + nbUint4s, 
				reinterpret_cast<uint4*>(handles) + nbUint4s, 8);

			__syncwarp();

			//OK. We need to compute a histogram runsum for the number of start projections.
			//This is used by both agg self-collision and agg-agg/agg-actor collision
			for (PxU32 i = 0; i < nbProjections; i += WARP_SIZE)
			{
				PxU32 idx = i + threadIdx.x;
				bool isStart = false;
				if (idx < nbProjections)
				{
					PxU32 sortedHandle = handles[idx];

					////Not actually a handle, but an index into the local list of bounds. Good enough 
					PxU32 handle = getHandle(sortedHandle);

					isStart = isStartProjection(sortedHandle);

					//printf("Handles[%i] = %i, handle = %i, isStartIndex = %i\n", idx, handles[idx], handle, (int)isStart);

					sapBox1D[handle].mMinMax[!isStart] = idx;

					/*if ((idx + 1) < nbProjections)
					{
						if (projections[idx] > projections[idx + 1])
							printf("Sort failure!!!\n");
						else
							printf("projection[%i] = %u, projection[%i] = %u\n", idx, projections[idx], idx + 1, projections[idx + 1]);
					}*/
				}

				PxU32 startMask = __ballot_sync(FULL_MASK, isStart);

				if (threadIdx.x == 0)
				{
					startMasks[i / 32] = startMask;
					//printf("StartMasks[%i] = %x\n", i / 32, startMask);
				}
			}
		}
	}
}

__device__ PxU32 getIthBit(PxU32 mask, PxU32 i)
{
	//The ith set bit can't be before bit i in this word so we can somewhat warm-start the binary search
	PxU32 lower = i; 
	PxU32 upper = 32;

	PxU32 nbBits = __popc(mask & ((2 << lower)-1));
	const PxU32 bitTarget = i+1;

	while ((upper - lower) > 1)
	{
		PxU32 midPoint = (lower + upper)/2;
		nbBits = __popc(mask&((2<< midPoint)-1));

		PxU32 active = mask & (1 << midPoint);

		if(nbBits > bitTarget || (!active && nbBits == bitTarget))
			upper = midPoint;
		else
			lower = midPoint;
	}

	return lower;
}

__device__ PxU32 getIthBitBruteForce(PxU32 mask, PxU32 target)
{
	PxU32 bitTarget = target + 1;
	PxU32 count = 0;
	for (PxU32 i = 0; i < 32; ++i)
	{
		if (mask & (1 << i))
		{
			count++;
		}
		if (count == bitTarget)
			return i;
	}
	return 0xFFFFFFFF;
}


__device__ PxU32 getIndex(const PxU32* PX_RESTRICT startMasks, const PxU32 startIdx, const PxU32 targetIndex, const PxU32 baseIndex, PxU32 boundsIdx = 0)
{
	const PxU32 startWord = startIdx / WARP_SIZE;

	PxU32 mask = ~((1 << (startIdx & 31)) - 1);

	PxU32 count = 0;

	PxU32 target = targetIndex - baseIndex; //How many after this one we want to process...

	PxU32 w = startWord;

	PxU32 maskedResult = startMasks[w] & mask;

	/*if (targetIndex == 18)
		printf("MaskedResult = %x, nbBits = %i, target = %i, startIdx = %i\n", maskedResult, __popc(maskedResult), target, startIdx);
*/
	for (; ; ++w)
	{
		PxU32 newCount = count + __popc(maskedResult);

		/*if (targetIndex == 18)
			printf("newCount = %i\n", newCount);*/

		if (newCount > target)
			break;

		count = newCount;
		maskedResult = startMasks[w + 1];
	}

	//OK. If we got here, we found the word and we have the masked result to find the offset...
	PxU32 offset = target - count;
	assert(offset < 32);

	/*if (targetIndex == 18)
		printf("offset = %i, maskedResult = %x\n", offset, maskedResult);*/

	PxU32 bit = getIthBit(maskedResult, offset);
	//PxU32 bit = getIthBitBruteForce(maskedResult, offset);

	/*if (targetIndex == 18)
		printf("bit = %i\n", bit);*/

	/*if (targetIndex == 7)
	{
		printf("%i: offset = %i, bit = %i bit2 = %i maskedResult = %x, bitSet = %i, offset = %i, bitSet2 = %i, offset2 = %i\n", threadIdx.x, offset, bit, bit2, maskedResult,
			maskedResult & (1<<bit), __popc(maskedResult&((1<<bit)-1)), maskedResult & (1<<bit2), __popc(maskedResult&((1<<bit2)-1)));
	}*/

	//Now we know which bit contains the comparison we care about...let's compare
	return w*WARP_SIZE + bit;
}

bool __device__ generatePairs(PxU32 i, PxU32 nbBounds, const PxU32* PX_RESTRICT comparisons,
	const PxgSapBox1D* PX_RESTRICT sapBox1D, const PxU32* PX_RESTRICT startMasks, 
	const PxU32* PX_RESTRICT sortedHandles, const PxgIntegerAABB* PX_RESTRICT currBounds, 
	const PxgIntegerAABB* PX_RESTRICT prevBounds, const PxU32* boundsId, const PxU32* groupId, PxgBroadPhasePair& pair)
{
	//First stage, let's do the prev comparisons to find out if we have lost self-collisions!
	{
		//Find out which pair we should be processing...
		PxU32 idx = binarySearch(comparisons, nbBounds, i);

		/*if (idx == 0)
			printf("nbBounds %i\n", nbBounds);*/

		//idx = the bounds that we're processing, offset = the number of comparisons after that we need to process...
		//we need to loop to find the comparison!
		const PxU32 startIdx = sapBox1D[idx].mMinMax[0];
		PxU32 boundsIdx = boundsId[idx];

		PxU32 index = getIndex(startMasks, startIdx+1, i, comparisons[idx], boundsIdx);

		//printf("Target = %i, index = %i startIdx %i comparision[%i] %i\n", i, index, startIdx, idx, comparisons[idx]);

		PxU32 compareHandle = sortedHandles[index];
		PxU32 handle = getHandle(compareHandle);
		PxU32 otherBoundsIdx = boundsId[handle];

		//Skip based on BP groups
		if (groupId[otherBoundsIdx] != groupId[boundsIdx])
		{
			//Test whether bounds overlap on all axes
			if (currBounds[otherBoundsIdx].intersects(currBounds[boundsIdx]))
			{
				//We have an intersection, now see if it was not present in the old bounds. 
				//If not, report this pair

				if (!prevBounds[otherBoundsIdx].intersects(prevBounds[boundsIdx]))
				{
					pair.mVolA = PxMin(boundsIdx, otherBoundsIdx);
					pair.mVolB = PxMax(boundsIdx, otherBoundsIdx);

					//printf("pair %i %i\n", pair.mVolA, pair.mVolB);
					return true;
				}
			}
		}
		return false;
	}
}

void  __device__ generateAllPairs(PxgAggregate& agg, const PxU32 totalComparisons, const PxU32 nbBounds, PxgBroadPhasePair* pairs, PxU32* sharedIndex,
	const PxgIntegerAABB* PX_RESTRICT bounds, const PxgIntegerAABB* PX_RESTRICT prevBounds, const PxU32* PX_RESTRICT groupIds, const PxU32 isLostPair,
	const PxU32 maxPairs)
{
	PxU32* comparisons = agg.comparisons[isLostPair];

	for (PxU32 i = 0; i < totalComparisons; i += WARP_SIZE)
	{
		PxU32 idx = threadIdx.x + i;

		bool generatedPair = false;
		PxgBroadPhasePair pair;
		if (idx < totalComparisons)
		{
			generatedPair = generatePairs(idx, nbBounds, comparisons, agg.sapBox1D[isLostPair], agg.startMasks[isLostPair],
				agg.sortedHandles[isLostPair], bounds, prevBounds, agg.boundIndices[isLostPair], groupIds, pair);
		}

		PxU32 mask = __ballot_sync(FULL_MASK, generatedPair);

		const PxU32 nbPairs = __popc(mask);

		PxU32 outIndex = 0;
		if (threadIdx.x == 0 && nbPairs)
		{
			outIndex = atomicAdd(sharedIndex, nbPairs);
		}

		outIndex = __shfl_sync(FULL_MASK, outIndex, 0);

		if (generatedPair)
		{
			PxU32 offset = warpScanExclusive(mask, threadIdx.x) + outIndex;
			//printf("isLostPair %i pair (%i, %i) to offset %i outIndex %i\n", isLostPair, pair.mVolA, pair.mVolB, offset, outIndex);
			if(offset < maxPairs)
				pairs[offset] = pair;
		}
	}
}

//sharedPairIndex[0] = sharedFoundPairIndex, sharedPairIndex[1] = sharedLostPairIndex
extern "C" __global__ void doSelfCollision(
	PxgBroadPhaseDesc* desc, 
	PxgAggregateDesc* aggDesc
)
{
	const PxU32 nbAggregates = aggDesc->numAgregates;
	PxgAggregate* aggregates = aggDesc->aggregates;

	const PxU32 globalWarpIndex = threadIdx.y + blockDim.y * blockIdx.x;

	/*if (globalWarpIndex == 0 && threadIdx.x == 0)
		printf("========================================================\n");*/

	if (globalWarpIndex < nbAggregates)
	{
		const PxgIntegerAABB* PX_RESTRICT iAABB = desc->newIntegerBounds;
		const PxgIntegerAABB* PX_RESTRICT iOldAABB = desc->oldIntegerBounds;
		PxgAggregate& agg = aggregates[globalWarpIndex];

		const PxU32 maxPairs = aggDesc->max_found_lost_pairs;

		//Check to see if we need to update it!

		PxU32 totalCount = 0;

		if (agg.isValid() && PxGetAggregateSelfCollisionBit(agg.filterHint))
		{
			const PxU32 numBounds = agg.size;

			PxgSapBox1D* sapBox1D = agg.sapBox1D[0];
			PxU32* startMasks = agg.startMasks[0];
			PxU32* comparisons = agg.comparisons[0];

			for (PxU32 i = threadIdx.x; i < numBounds; i += WARP_SIZE)
			{
				PxU32 startIdx = sapBox1D[i].mMinMax[0];
				PxU32 endIdx = sapBox1D[i].mMinMax[1];

				const PxU32 startWord = startIdx/WARP_SIZE;
				const PxU32 endWord = endIdx/WARP_SIZE;

				PxU32 mask = ~((2<<(startIdx &31))-1);
				
				PxU32 count = 0;

				for (PxU32 w = startWord; w < endWord; ++w)
				{
					count += __popc(startMasks[w] & mask);
					mask = 0xFFFFFFFF;
				}

				mask &= ((2 << (endIdx & 31))-1);

				count += __popc(startMasks[endWord]&mask);

				comparisons[i] = count;
			}

			__syncwarp();

			//Now do the pair runsum...

			for (PxU32 i = 0; i < numBounds; i += WARP_SIZE)
			{
				const PxU32 idx = i + threadIdx.x;
				PxU32 count = 0;
				if (idx < numBounds)
				{
					count = comparisons[idx];
				}

				PxU32 inclusiveSum = warpScan<AddOpPxU32, PxU32>(FULL_MASK, count);

				PxU32 offset = inclusiveSum - count + totalCount;
				if (idx < numBounds)
					comparisons[idx] = offset;
				
				totalCount += __shfl_sync(FULL_MASK, inclusiveSum, 31);
			}

			__syncwarp();

			//Find lost pairs by finding all previous pairs and comparing them to see if they
			//are missing now
			
			if (!agg.isNew)
			{
				generateAllPairs(agg, agg.prevComparisons, agg.prevSize, aggDesc->lostPairReport, &aggDesc->sharedLostPairIndex,
					iOldAABB, iAABB, desc->updateData_groups, 1, maxPairs);
			}

			//Find found pairs by finding all current pairs and comparing them to see if they
			//were not present last frame
			generateAllPairs(agg, totalCount, agg.size, aggDesc->foundPairReport, &aggDesc->sharedFoundPairIndex,
				iAABB, iOldAABB, desc->updateData_groups, 0, maxPairs);

		}

		// AD: This is a bit early, prevComparisons should probably be renamed to make it explicit that this is only about self collisions!
		if (threadIdx.x == 0)
		{
			agg.prevComparisons = totalCount;
		}	
	}
}

static __device__ PxU32 bSearchFirstProjection(const PxU32* PX_RESTRICT projections, const PxU32 numElements, const PxU32 proj)
{
	PxU32 left = 0;
	PxU32 right = numElements;

	while (left < right)
	{
		PxU32 pos = (left + right) / 2;
		const PxU32 element = projections[pos];

		if (element < proj)
		{
			left = pos+1;
		}
		else
		{
			right = pos;
		}
	}

	return left;
}

template<class T>
static __device__ PxU32 binarySearch1(const T* PX_RESTRICT data, const PxU32 numElements, const T& value)
{
	PxU32 left = 0;
	PxU32 right = numElements;

	//while((right - left) > 1)
	while (left < right)
	{
		PxU32 pos = (left + right) / 2;
		const T& element = data[pos];

		if (element <= value)
		{
			//Found exact value
			left = pos + 1;
		}
		else
		{
			right = pos;
		}
	}

	if (left == 0)
	{
		left = 0;
		right = numElements;

		//while((right - left) > 1)
		while (left < right)
		{
			PxU32 pos = (left + right) / 2;
			const T& element = data[pos];

			if (element <= value)
			{
				//Found exact value
				left = pos + 1;
			}
			else
			{
				right = pos;
			}
		}

		return left;
	}

	return left - 1;
}

__device__ PxU32 countComparisons(const PxgIntegerAABB& objBound, const PxU32* PX_RESTRICT sortedProjections,
	const PxU32 nbProjections, const PxU32* PX_RESTRICT startMasks, PxU32& outStartIdx, PxU32& outEndIdx, const PxU32 isFirst)
{
	//Find the index in which the startPojectionVal and endProjectionVal should exist...

	PxU32 startIdx = bSearchFirstProjection(sortedProjections, nbProjections, objBound.mMinMax[0]);
	PxU32 endIdx = binarySearch(sortedProjections, nbProjections, objBound.mMinMax[3]);
	
	//printf("StartIdx = %i, endIdx = %i\n", startIdx, endIdx);

	//Now we can calculate the number of bits between them...
	PxU32 startWordId = startIdx / 32; //Which word do I start in...
	PxU32 endWordId = (endIdx) / 32;

	PxU32 w = startMasks[startWordId] & (~((1 << (startIdx & 31)) - 1));
	PxU32 totalComparisons = 0;
	for (PxU32 i = startWordId; i < endWordId; w = startMasks[++i])
	{
		totalComparisons += __popc(w);
	}

	w &= ((2 << (endIdx & 31)) - 1);

	totalComparisons += __popc(w);
	outStartIdx = startIdx;
	outEndIdx = endIdx;
	return totalComparisons;
}


static __device__ void boxPruningProjection(const PxgIntegerAABB* PX_RESTRICT bounds, const PxgIntegerAABB* PX_RESTRICT oldBounds, 
	const PxU32* objBoundIndices, const PxU32 nbObj, const PxU32* PX_RESTRICT sortedProjections, const PxU32 nbProjections, 
	const PxU32* PX_RESTRICT sortedHandles, const PxU32* PX_RESTRICT startMasks, const PxU32* boundIndices, const PxU32* groupIds,
	PxgBroadPhasePair* PX_RESTRICT pairs, PxU32* PX_RESTRICT sharedIndex, bool findLostPairs, const PxU32 isFirst, bool isNew,
	const PxU32 maxPairs)
{
	//Find the index in which the startPojectionVal and endProjectionVal should exist...

	__shared__ PxU32 shComparisons[PxgBPKernelBlockDim::BP_AGGREGATE_SORT/WARP_SIZE][WARP_SIZE];
	__shared__ PxU32 shStartIdx[PxgBPKernelBlockDim::BP_AGGREGATE_SORT / WARP_SIZE][WARP_SIZE];

	for (PxU32 i = 0; i < nbObj; i += WARP_SIZE)
	{
		PxU32 objId = i + threadIdx.x;

		PxU32 startIdx = 0; PxU32 endIdx = 0; PxU32 count = 0;

		if (objId < nbObj)
		{
			PxU32 handle = objBoundIndices[objId];
			count = countComparisons(bounds[handle], sortedProjections, nbProjections, startMasks, startIdx, endIdx, isFirst);
		}

		//Now we need to do a runsum...

		PxU32 inclusiveSum = warpScan<AddOpPxU32, PxU32>(FULL_MASK, count);

		PxU32 totalCount = __shfl_sync(FULL_MASK, inclusiveSum, 31);
		inclusiveSum -= count;

		shComparisons[threadIdx.y][threadIdx.x] = inclusiveSum;
		shStartIdx[threadIdx.y][threadIdx.x] = startIdx;
		__syncwarp();

		//Now we go round the loop x times...

		for (PxU32 b = 0; b < totalCount; b += WARP_SIZE)
		{
			PxU32 cmpId = b + threadIdx.x;

			bool hasContact = false;
			PxgBroadPhasePair pair;

			if (cmpId < totalCount)
			{
				const PxU32 idx = binarySearch(shComparisons[threadIdx.y], 32, cmpId);

				//Now we know which entry we're in...pull data from that link...

				const PxU32 objId = i + idx;

				const PxU32 handle = objBoundIndices[objId];

				const PxU32 startIdx = shStartIdx[threadIdx.y][idx];

				const PxU32 index = getIndex(startMasks, startIdx, cmpId, shComparisons[threadIdx.y][idx]);

				const PxU32 otherId = getHandle(sortedHandles[index]);

				const PxU32 otherHandle = boundIndices[otherId];
				assert(handle != otherHandle);

				//Now we know which index we need to read, and we know which original bounds we are comparing, let's do collision...
				if (groupIds[handle] != groupIds[otherHandle])
				{
					// we check if the bounds intersect.
					// if foundPairs: these are the current bounds. 
					// if lostPairs: these are the bounds from the last step. If one of the bounds has been invalidated, this test
					//               returns no intersection.

					// AD: with the invalidation of both the old and new bounds of a bounds entry that was part of an aggregate, we are left
					// with the following logic:
					//
					// no topology changes in the aggregate:
					//   - findNewPairs: if new bounds intersect, test old bounds, if they don't, generate a pair.
					//   - findLostPairs: if old bounds intersect, test new bounds, if they don't, generate a pair.
					//
					// a bounds entry was added to the aggregate: - old bounds are invalid, new bounds are valid.
					//   - findNewPairs: if new bounds intersect, generate a new pair. the old bounds are invalid so we never
					//                   get an intersection there, leading us to generate a new pair.
					//   - findLostPairs: old bounds never intersect. So there is no pair to lose.
					//
					// a bounds entry was removed from the aggregate:
					//   - findNewPairs: new bounds will not intersect because they have been invalidated.
					//   - findLostPairs: old bounds will not intersect because they have been invalidated.
					//
					// if the whole aggregate or a single actor that is part of this pair has been removed, we're catching that outside of this function
					// by checking the removed handle bitmap of the regular broadphase.

					if (bounds[handle].intersects(bounds[otherHandle])) 
					{
						if (!(oldBounds[handle].intersects(oldBounds[otherHandle])))
						{
							if(isFirst || bounds[handle].mMinMax[0] != bounds[otherHandle].mMinMax[0])
							{
								hasContact = true;
								pair.mVolA = PxMin(handle, otherHandle);
								pair.mVolB = PxMax(handle, otherHandle);
								/*if (findLostPairs)
									printf("%i: LOST Pair %i, %i\n", threadIdx.x, pair.mVolA, pair.mVolB);
								else
									printf("%i: FOUND Pair %i, %i startIdx %i, idx %i, index %i\n", threadIdx.x, pair.mVolA, pair.mVolB, startIdx, idx, index);*/
							}
						}
					}
				}
			}


			PxU32 contactMask = __ballot_sync(FULL_MASK, hasContact);
			const PxU32 nbPairs = __popc(contactMask);

			if (nbPairs)
			{
				PxU32 outIndex = 0;
				if (threadIdx.x == 0)
				{
					outIndex = atomicAdd(sharedIndex, nbPairs);
				}

				outIndex = __shfl_sync(FULL_MASK, outIndex, 0);

				if (hasContact)
				{
					PxU32 offset = __popc(contactMask & ((1 << threadIdx.x) - 1)) + outIndex;

					if(offset < maxPairs)
						pairs[offset] = pair;

					//printf("threadIdx.x %i pair[%i] %i %i, offset %i, outIndex %i, nbPairs %i, contactMask %x\n", threadIdx.x, offset + outIndex, pair.mVolA, pair.mVolB, offset, outIndex, nbPairs, contactMask);
				}
			}

		}

		__syncwarp(); //Read and write operations on shared memory (shComparisons) in loops must be guarded before and after reads/writes
	}
}

static __device__ void bipartiteBoxPruning(const PxU32* PX_RESTRICT sortedProjections0, const PxU32* PX_RESTRICT sortedProjections1,
	const PxU32* PX_RESTRICT sortedHandles0, const PxU32* PX_RESTRICT sortedHandles1, const PxU32* PX_RESTRICT startMask0, 
	const PxU32* PX_RESTRICT startMask1, const PxU32* PX_RESTRICT boundIndices0, const PxU32* PX_RESTRICT boundIndices1,
	const PxgIntegerAABB* PX_RESTRICT bounds, const PxgIntegerAABB* PX_RESTRICT oldBounds, const PxU32* PX_RESTRICT groupIds,
	const PxU32 nbObj0, const PxU32 nbObj1, PxgBroadPhasePair* PX_RESTRICT pairs, PxU32* PX_RESTRICT sharedIndex, bool isLost, bool isNew,
	const PxU32 maxPairs)
{
	//A bipartite projection box pruning algorithm projects list 0 onto list 1 and then list 1 onto list 0
	boxPruningProjection(bounds, oldBounds, boundIndices0, nbObj0, sortedProjections1, nbObj1 * 2, sortedHandles1, startMask1, 
		boundIndices1, groupIds, pairs, sharedIndex, isLost, 1, isNew, maxPairs);

	boxPruningProjection(bounds, oldBounds, boundIndices1, nbObj1, sortedProjections0, nbObj0 * 2, sortedHandles0, startMask0,
		boundIndices0, groupIds, pairs, sharedIndex, isLost, 0, isNew, maxPairs);
}

static __device__ PxU32 PX_FORCE_INLINE testBitmap(PxU32 actorHandle, const PxU32* bitmap)
{
	const PxU32 word = actorHandle / 32;
	const PxU32 mask = bitmap[word];
	return (mask & (1 << (actorHandle & 31)))!=0;
}

extern "C" __global__ void doAggPairCollisions(
	PxgBroadPhaseDesc* desc, 
	PxgAggregateDesc* aggDesc)
{
	const PxU32 globalWarpIndex = threadIdx.y + blockDim.y * blockIdx.x;

	const PxU32 nbWarps = PxgBPKernelBlockDim::BP_AGGREGATE_SORT / WARP_SIZE;

	PxgAggregatePair* pairs = aggDesc->aggPairs;
	const PxU32 nbAggregatePairs = PxMin(*aggDesc->aggPairCount, aggDesc->max_agg_pairs);

	const PxU32 maxFoundLostPairs = aggDesc->max_found_lost_pairs;

	const PxU32* gRemovedHandles = desc->aabbMngr_removedHandleMap;

	PxU32* aggBitmap = aggDesc->removeBitmap;

	for(PxU32 i = globalWarpIndex; i < nbAggregatePairs; i += blockDim.y*gridDim.x)
	{
		PxgAggregatePair pair = pairs[i];
		bool shouldRemove = false;
		if (!pair.isDead)
		{
			const PxgIntegerAABB* bounds = desc->newIntegerBounds;
			Bp::VolumeData* volumeData = desc->aabbMngr_volumeData;
			PxgAggregate* aggregates = aggDesc->aggregates;

			// This whole kernel also processes lost pairs, because if we lose an aggregate pair, we still
			// need to process all of the aggregate contents and generate lost pairs.

			// AD: we need to make sure we don't process anything that refers to bounds that have been deleted.
			//
			// At this level, we're looking at pairs of agg-agg/actor-agg/agg-actor. We're looking at the bounds as seen by the regular
			// broadphase here.
			// 
			// let's think about what happens if we removed bounds.
			// - the bounds have been invalidated. So oldbounds are set to empty.
			// - the volumeData has been reset. so reset() has been called, which means userData is null and mAggregate is PX_INVALID_U32.
			//      But this happens only for actual bounds. If we are looking at an aggregate, we need to check the deleted map whether it
			//      has been deleted.
			//
			// So as a first step, we test the removedHandles map. This will cover singleActor bounds and aggregate bounds.
			// for bounds that are part of an aggregate, we separately invalidate the old and new bounds to make sure we fail the intersection tests. 

			bool isValid0 = (!testBitmap(pair.actorHandle0, gRemovedHandles));
			bool isValid1 = (!testBitmap(pair.actorHandle1, gRemovedHandles));

			// one of the bounds in this pair has been removed, mark for deletion and skip.
			if (!isValid0 || !isValid1)
			{
				if (threadIdx.x == 0)
					aggBitmap[i] = true;

				__syncwarp();
				continue;
			}

			// Then we test whether it still intersects. if not, this will become a lost pair.
			shouldRemove = !bounds[pair.actorHandle0].intersects(bounds[pair.actorHandle1]);

			//We allow to continue because we still need the lost shape pair reports below. The pair is then
			//destroyed in the pair deletion stage.

			//This is used to emulate an aggregate structure for simple shapes...
			__shared__ PxU32 sortedProjections[nbWarps][2][2];
			__shared__ PxU32 sortedHandles[nbWarps][2];
			__shared__ PxU32 startMask[nbWarps];
			__shared__ PxU32 boundIndices[nbWarps][2];

			if (threadIdx.x < 2)
			{
				sortedHandles[threadIdx.y][threadIdx.x] = createHandle(0, threadIdx.x == 1, false);
				if (threadIdx.x == 0)
				{
					// PT: this is also always hardcoded and perhaps has the same issue but I don't know what this mask means
					startMask[threadIdx.y] = 1;
				}
			}

			const PxU32* sortedProjections0[2];
			const PxU32* sortedProjections1[2];
			const PxU32* sortedHandles0[2];
			const PxU32* sortedHandles1[2];
			const PxU32* startMask0[2];
			const PxU32* startMask1[2];

			PxU32* boundIndices0[2];
			PxU32* boundIndices1[2];

			PxU32 nbObj0[2];
			PxU32 nbObj1[2];

			// AD: not sure if this is a good thing to do.
			// we actually set the userdata to the high-level aggregate pointer, double-check if this is exposed to users?
			assert(volumeData[pair.actorHandle0].getUserData());
			assert(volumeData[pair.actorHandle1].getUserData());

			const bool isAggregate0 = volumeData[pair.actorHandle0].isAggregate();
			const bool isAggregate1 = volumeData[pair.actorHandle1].isAggregate();

			if (isAggregate0)
			{
				const Bp::AggregateHandle aggHandle0 = volumeData[pair.actorHandle0].getAggregate();
				PxgAggregate& agg = aggregates[aggHandle0];
				sortedProjections0[0] = agg.sortedProjections[0];
				sortedProjections0[1] = agg.sortedProjections[1];
				sortedHandles0[0] = agg.sortedHandles[0];
				sortedHandles0[1] = agg.sortedHandles[1];
				startMask0[0] = agg.startMasks[0];
				startMask0[1] = agg.startMasks[1];
				boundIndices0[0] = agg.boundIndices[0];
				boundIndices0[1] = agg.boundIndices[1];
				nbObj0[0] = agg.size;
				nbObj0[1] = agg.prevSize;
			}
			else
			{
				sortedProjections0[0] = sortedProjections[threadIdx.y][0];
				sortedProjections0[1] = sortedProjections[threadIdx.y][1];
				sortedHandles0[0] = &sortedHandles[threadIdx.y][0];
				sortedHandles0[1] = &sortedHandles[threadIdx.y][1];
				startMask0[0] = &startMask[threadIdx.y];
				startMask0[1] = &startMask[threadIdx.y];
				boundIndices0[0] = &boundIndices[threadIdx.y][0];
				boundIndices0[1] = &boundIndices[threadIdx.y][1];
				nbObj0[0] = 1;
				nbObj0[1] = 1;

				if (threadIdx.x < 2)
				{
					boundIndices[threadIdx.y][threadIdx.x] = pair.actorHandle0;
					sortedProjections[threadIdx.y][0][threadIdx.x] = desc->newIntegerBounds[pair.actorHandle0].mMinMax[threadIdx.x * 3];
					sortedProjections[threadIdx.y][1][threadIdx.x] = desc->oldIntegerBounds[pair.actorHandle0].mMinMax[threadIdx.x * 3];
				}
			}

			if (isAggregate1)
			{
				const Bp::AggregateHandle aggHandle1 = volumeData[pair.actorHandle1].getAggregate();
				PxgAggregate& agg = aggregates[aggHandle1];
				sortedProjections1[0] = agg.sortedProjections[0];
				sortedProjections1[1] = agg.sortedProjections[1];
				sortedHandles1[0] = agg.sortedHandles[0];
				sortedHandles1[1] = agg.sortedHandles[1];
				startMask1[0] = agg.startMasks[0];
				startMask1[1] = agg.startMasks[1];
				boundIndices1[0] = agg.boundIndices[0];
				boundIndices1[1] = agg.boundIndices[1];
				nbObj1[0] = agg.size;
				nbObj1[1] = agg.prevSize;
			}
			else
			{
				sortedProjections1[0] = sortedProjections[threadIdx.y][0];
				sortedProjections1[1] = sortedProjections[threadIdx.y][1];
				sortedHandles1[0] = &sortedHandles[threadIdx.y][0];
				sortedHandles1[1] = &sortedHandles[threadIdx.y][1];
				startMask1[0] = &startMask[threadIdx.y];
				startMask1[1] = &startMask[threadIdx.y];
				boundIndices1[0] = &boundIndices[threadIdx.y][0];
				boundIndices1[1] = &boundIndices[threadIdx.y][1];
				nbObj1[0] = 1;
				nbObj1[1] = 1;

				if (threadIdx.x < 2)
				{
					boundIndices[threadIdx.y][threadIdx.x] = pair.actorHandle1;
					sortedProjections[threadIdx.y][0][threadIdx.x] = desc->newIntegerBounds[pair.actorHandle1].mMinMax[threadIdx.x * 3];
					sortedProjections[threadIdx.y][1][threadIdx.x] = desc->oldIntegerBounds[pair.actorHandle1].mMinMax[threadIdx.x * 3];
				}
			}

			//Make sure write is visible, then we can perform collision detection
			__syncwarp();

			const PxgIntegerAABB* oldBounds = desc->oldIntegerBounds;
			const PxU32* groupIds = desc->updateData_groups;

			
			bipartiteBoxPruning(sortedProjections0[0], sortedProjections1[0], sortedHandles0[0], sortedHandles1[0], startMask0[0],
				startMask1[0], boundIndices0[0], boundIndices1[0], bounds, oldBounds, groupIds, nbObj0[0], nbObj1[0], aggDesc->foundPairReport,
				&aggDesc->sharedFoundPairIndex, false, pair.isNew, maxFoundLostPairs);

			if (!pair.isNew)
			{
				bipartiteBoxPruning(sortedProjections0[1], sortedProjections1[1], sortedHandles0[1], sortedHandles1[1], startMask0[1],
					startMask1[1], boundIndices0[1], boundIndices1[1], oldBounds, bounds, groupIds, nbObj0[1], nbObj1[1], aggDesc->lostPairReport,
					&aggDesc->sharedLostPairIndex, true, false, maxFoundLostPairs);
			}

			__syncwarp();

			if (threadIdx.x == 0)
			{
				pairs[i].isNew = false;
			}
		}

		if (threadIdx.x == 0)
		{
			aggBitmap[i] = shouldRemove;
		}
	}
}

extern "C" __global__ void aggCopyReports(PxgAggregateDesc* aggDesc)
{
	const PxU32 nbCreatedPairs = aggDesc->sharedFoundPairIndex;
	const PxU32 nbLostPairs = aggDesc->sharedLostPairIndex;

	PxU32* foundReports = reinterpret_cast<PxU32*>(aggDesc->foundPairReport);
	PxU32* lostReports = reinterpret_cast<PxU32*>(aggDesc->lostPairReport);

	PxU32* foundReportMap = reinterpret_cast<PxU32*>(aggDesc->foundPairReportMap);
	PxU32* lostReportMap = reinterpret_cast<PxU32*>(aggDesc->lostPairReportMap);

	const PxU32 max_found_lost_pairs = aggDesc->max_found_lost_pairs;

	const PxU32 idx = threadIdx.x + blockIdx.x * blockDim.x;

	const bool foundOverflow = (nbCreatedPairs > max_found_lost_pairs);
	const bool lostOverflow = (nbLostPairs > max_found_lost_pairs);
	if (threadIdx.x == 0 && blockIdx.x == 0)
	{
		// AD: we could have a "fake" overflow in the addAggPairs stage, if we have too many candidate pairs
		// from the broadphase.
		aggDesc->found_pairs_overflow_flags |= foundOverflow;
		aggDesc->lost_pairs_overflow_flags = lostOverflow;
	}

	const PxU32 tNbCreatedPairs = foundOverflow ? max_found_lost_pairs : nbCreatedPairs;
	const PxU32 tNbLostPairs = lostOverflow ? max_found_lost_pairs : nbLostPairs;

	const PxU32 nbCreateElements = (tNbCreatedPairs * sizeof(PxgBroadPhasePair)) / sizeof(PxU32);
	const PxU32 nbLostElements = (tNbLostPairs * sizeof(PxgBroadPhasePair)) / sizeof(PxU32);

	/*if(idx == 0)
		printf("nbCreateElements %i nbCreatedPairs %i tNbCreatedPairs %i\n", nbCreateElements, nbCreatedPairs, tNbCreatedPairs);*/

	for (PxU32 i = idx; i < nbCreateElements; i += blockDim.x * gridDim.x)
	{
		foundReportMap[i] = foundReports[i];
		//printf("foundReport[%i] = %i\n", i, foundReports[i]);
	}

	for (PxU32 i = idx; i < nbLostElements; i += blockDim.x * gridDim.x)
	{
		lostReportMap[i] = lostReports[i];
	}
}

extern "C" __global__ void addAggPairsStage1(PxgAggregateDesc* aggDesc, PxgBroadPhaseDesc* bpDesc)
{
	PxgBroadPhasePair* foundAddPairs = bpDesc->foundAggPairReport;
	const PxU32 nbFoundAggPairs = bpDesc->sharedFoundAggPairIndex;
	const PxU32 nbFoundPairsSafe = PxMin(nbFoundAggPairs, aggDesc->max_found_lost_pairs);

	const PxU32 globalThreadIdx = threadIdx.x + blockIdx.x*blockDim.x;

	const PxU32 numFreeIndices = aggDesc->freeBufferList->numFreeIndices;
	const PxU32 maxIndex = aggDesc->freeBufferList->maxIndex;
	PxU32* freeIndices = aggDesc->freeIndices;

	PxgAggregatePair* aggPairs = aggDesc->aggPairs;

	const PxU32 maxAggPairs = aggDesc->max_agg_pairs;

	for (PxU32 i = globalThreadIdx; i < nbFoundPairsSafe; i += blockDim.x*gridDim.x)
	{
		//Where do we get the entry ID from...
		PxU32 index = 0;
		if (i < numFreeIndices)
		{
			PxU32 freeIndex = (numFreeIndices - i-1);
			index = freeIndices[freeIndex];
		}
		else
		{
			const PxU32 offset = i - numFreeIndices;
			index = maxIndex + offset;
		}

		//Now write the pair there...
		//printf("Adding aggregate pair (%i, %i) to index %i maxPairs %u nbFound %u\n", foundAddPairs[i].mVolA, foundAddPairs[i].mVolB, index, maxAggPairs, nbFoundAggPairs);
		if (index < maxAggPairs)
		{
			aggPairs[index].actorHandle0 = foundAddPairs[i].mVolA;
			aggPairs[index].actorHandle1 = foundAddPairs[i].mVolB;
			aggPairs[index].isNew = true;
			aggPairs[index].isDead = false;
			aggPairs[index].pad = 0; //Assign all values to make uninitialized memory sanitizer reports better readable
		}
	}
}

extern "C" __global__ void addAggPairsStage2(PxgAggregateDesc* aggDesc, PxgBroadPhaseDesc* bpDesc)
{
	const PxU32 globalThreadIdx = threadIdx.x + blockIdx.x*blockDim.x;

	if (globalThreadIdx == 0)
	{
		const PxU32 nbFoundAggPairs = bpDesc->sharedFoundAggPairIndex;
		const PxU32 nbFoundAggPairsSafe = PxMin(nbFoundAggPairs, aggDesc->max_found_lost_pairs);
		PxU32 nbFreeInds = aggDesc->freeBufferList->numFreeIndices;
		PxU32 newFreeInds = 0;
		PxU32 numToIncrement = 0;

		/*if (nbFoundAggPairs)
		{
			printf("NbFoundAggPairs = %i, nbFreeInds = %i\n", nbFoundAggPairs, nbFreeInds);
		}*/

		if(nbFreeInds >= nbFoundAggPairsSafe)
			newFreeInds = nbFreeInds - nbFoundAggPairsSafe;
		else
			numToIncrement = nbFoundAggPairsSafe - nbFreeInds;

		aggDesc->freeBufferList->numFreeIndices = newFreeInds;

		PxU32 maxIndex = aggDesc->freeBufferList->maxIndex + numToIncrement;

		// AD: we need to write this every time for simStats.
		// note that this is pinned memory, while aggPairCount we write below is device memory.
		aggDesc->aggPairOverflowCount = maxIndex;

		if (maxIndex > aggDesc->max_agg_pairs)
		{
			aggDesc->agg_pairs_overflow_flags = true;
			maxIndex = aggDesc->max_agg_pairs;
		}

		aggDesc->freeBufferList->maxIndex = maxIndex;
		*aggDesc->aggPairCount = maxIndex;

		// AD: we can already overflow the aggregate found pairs count in the regular broadphase, because
		// any found pair with an aggregate already takes up a slot. On top of that, we have self collisions.
		if (nbFoundAggPairs > aggDesc->max_found_lost_pairs)
		{
			aggDesc->found_pairs_overflow_flags = true;
			aggDesc->foundCandidatePairOverflowCount = nbFoundAggPairs;
		}

		/*if (nbFoundAggPairs)
		{
			printf("Agg count now %i\n", aggDesc->freeBufferList->maxIndex);
		}*/
	}
}

extern "C" __global__ __launch_bounds__(PxgBPKernelBlockDim::BP_AGGREGATE_REMOVE, 1) void removeAggPairsStage1(PxgAggregateDesc* aggDesc)
{
	//We need to compute a runsum of all the removed pairs and the total number of pairs we removed...
	const PxU32 nbAggregatePairs = *aggDesc->aggPairCount;
	const PxU32 nbAggPairMasks = nbAggregatePairs;
	PxU32* aggBitmap = aggDesc->removeBitmap;
	PxU32* aggHisto = aggDesc->removeHistogram;

	const PxU32 nbWarpsPerWave = (PxgBPKernelBlockDim::BP_AGGREGATE_REMOVE/WARP_SIZE)*gridDim.x;

	const PxU32 nbWarpsRequired = ((nbAggPairMasks+31)/32);

	const PxU32 globalWarpIndex = threadIdx.y + blockIdx.x*blockDim.y;

	const PxU32 nbIterations = (nbWarpsRequired + nbWarpsPerWave -1)/nbWarpsPerWave;

	__shared__ PxU32 shWarpSum[PxgBPKernelBlockDim::BP_AGGREGATE_REMOVE/WARP_SIZE];

	for (PxU32 i = 0; i < nbIterations; ++i)
	{
		PxU32 idx = i*nbWarpsPerWave + globalWarpIndex;

		const PxU32 thIdx = idx*WARP_SIZE + threadIdx.x;

		PxU32 remove = 0;
		if (thIdx < nbAggPairMasks)
		{
			remove = aggBitmap[thIdx];
			/*if (remove)
				printf("AggBitMap[%i] = true\n", thIdx);
			else
				printf("AggBitMap[%i] is false\n", thIdx);*/
		}

		/*PxU32 count = __popc(mask);

		const PxU32 warpCount = warpReduction<AddOpPxU32, PxU32>(FULL_MASK, count);*/

		const PxU32 mask = __ballot_sync(FULL_MASK, remove);

		if(threadIdx.x == 0)
			shWarpSum[threadIdx.y] = __popc(mask);

		__syncthreads();

		//Now do a block accumulation, then output the block accumulation to the global summation...

		if (threadIdx.y == 0)
		{
			//Only first thread does this...
			PxU32 ws = shWarpSum[threadIdx.x];
			const PxU32 blockCount = warpReduction<AddOpPxU32, PxU32>(FULL_MASK, ws);
			if(threadIdx.x == 0)
				aggHisto[blockIdx.x + i*gridDim.x] = blockCount;
		}
	}
}

extern "C" __global__ __launch_bounds__(PxgBPKernelBlockDim::BP_AGGREGATE_REMOVE, 1) void removeAggPairsStage2(PxgAggregateDesc* aggDesc)
{
	//We need to compute a runsum of all the removed pairs and the total number of pairs we removed...
	const PxU32 nbAggregatePairs = *aggDesc->aggPairCount;
	PxgAggregatePair* aggPairs = aggDesc->aggPairs;
	const PxU32 nbAggPairMasks = nbAggregatePairs;
	PxU32* aggBitmap = aggDesc->removeBitmap;
	PxU32* aggHisto = aggDesc->removeHistogram;
	const PxU32 numFreeInds = aggDesc->freeBufferList->numFreeIndices;
	PxU32* freeIndices = aggDesc->freeIndices;

	const PxU32 nbWarpsPerWave = (PxgBPKernelBlockDim::BP_AGGREGATE_REMOVE / WARP_SIZE)*gridDim.x;

	const PxU32 nbWarpsRequired = ((nbAggPairMasks + 31) / 32);

	const PxU32 globalWarpIndex = threadIdx.y + blockIdx.x*blockDim.y;

	const PxU32 nbIterations = (nbWarpsRequired + nbWarpsPerWave -1)/ nbWarpsPerWave;

	__shared__ PxU32 shWarpSum[PxgBPKernelBlockDim::BP_AGGREGATE_REMOVE / WARP_SIZE];	

	__syncthreads();

	PxU32 accum = 0;

	for (PxU32 i = 0; i < nbIterations; ++i)
	{
		PxU32 idx = i*nbWarpsPerWave + globalWarpIndex;

		const PxU32 thIdx = idx*WARP_SIZE + threadIdx.x;

		//Do runsum for the next 32 blocks of work. Accumulate summation

		PxU32 val = aggHisto[threadIdx.x + i * blockDim.x];

		PxU32 blockSum = warpScan<AddOpPxU32, PxU32>(FULL_MASK, val);

		PxU32 totalBlockSum = __shfl_sync(FULL_MASK, blockSum, 31);

		//Offset is stored in the thread correspondin to our blck Idx
		PxU32 blockOffset = __shfl_sync(FULL_MASK, blockSum - val, blockIdx.x) + accum;

		accum += totalBlockSum;

		PxU32 remove = 0;
		if (thIdx < nbAggPairMasks)
		{
			remove = aggBitmap[thIdx];
		}

		PxU32 removeMask = __ballot_sync(FULL_MASK, remove);
		const PxU32 offsetInWarp = __popc(removeMask & ((1 << threadIdx.x)-1));

		if (threadIdx.x == 31)
			shWarpSum[threadIdx.y] = __popc(removeMask);

		__syncthreads();

		//Now do a block accumulation
		PxU32 ws = shWarpSum[threadIdx.x];
		const PxU32 blockCount = warpScan<AddOpPxU32, PxU32>(FULL_MASK, ws);
		const PxU32 warpOffset = __shfl_sync(FULL_MASK, blockCount - ws, threadIdx.y);

		PxU32 outputIndex = blockOffset + warpOffset + offsetInWarp;

		if(remove)
		{
			//Get the index we need to remove...
			//printf("Removing pair %i putting index into freeIndices[%i] - indices %u %u\n", thIdx, numFreeInds + outputIndex, aggPairs[thIdx].actorHandle0, aggPairs[thIdx].actorHandle1);
			aggPairs[thIdx].isDead = true;
			freeIndices[numFreeInds + outputIndex] = thIdx;
			//printf("Removing aggregate pair %i, %i\n", aggPairs[thIdx].actorHandle0, aggPairs[thIdx].actorHandle1);
		}
	}

	if (threadIdx.x == 0 && threadIdx.y == 0 && blockIdx.x == 0)
	{
		aggDesc->nbRemoved = accum;
	}
}

extern "C" __global__ void removeAggPairsStage3(PxgAggregateDesc* aggDesc)
{
	const PxU32 globalThreadIdx = threadIdx.x + blockIdx.x*blockDim.x;

	if (globalThreadIdx == 0)
	{
		const PxU32 nbRemoved = aggDesc->nbRemoved;
		aggDesc->freeBufferList->numFreeIndices += nbRemoved;
	}
}

extern "C" __global__ void markAddedAndDeletedAggregatedBounds(
	PxgBroadPhaseDesc* bpDesc,
	PxgAggregateDesc* aggDesc,
	PxU32* deletedAggregatedBounds,
	PxU32 numDeletedAggregatedBounds,
	PxU32* addedAggregatedBounds,
	PxU32 numAddedAggregatedBounds
)
{
	// AD: threadIdx.y == 0 -> addedAggregatedBounds
	//     threadIdx.y == 1 -> deletedAggregatedBounds
	assert(blockDim.y == 2);

	const PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;
	PxU32 maxCount = threadIdx.y ? numDeletedAggregatedBounds : numAddedAggregatedBounds;

	PxgIntegerAABB* bounds = bpDesc->newIntegerBounds;
	PxgIntegerAABB* oldBounds = bpDesc->oldIntegerBounds;
	PxU32* src = threadIdx.y ? deletedAggregatedBounds : addedAggregatedBounds;

	if (globalThreadIdx < maxCount)
	{
		PxU32 handle = src[globalThreadIdx];

		if (threadIdx.y == 1) // invalidate both for deleted bounds.
		{
			bounds[handle].setEmpty();
		}

		oldBounds[handle].setEmpty();
	}
}