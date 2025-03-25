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

#include "vector_types.h"
#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"
#include "stdio.h"
#include "assert.h"
#include "cuda.h"

#include "sparseGridStandalone.cuh"

#define ENABLE_KERNEL_LAUNCH_ERROR_CHECK 0

#define NEW_SUBGRID 0xfffffffe
#define REUSED_SUBGRID 0xfffffffd

extern "C" __host__ void initSparseGridStandaloneKernels0() {}

extern "C" __global__ void sg_SparseGridCalcSubgridHashes(
	PxSparseGridParams							sparseGridParams,
	PxU32* PX_RESTRICT							indices,
	PxU32* PX_RESTRICT							hashkeyPerParticle,
	const PxVec4* const PX_RESTRICT			positions,
	const PxU32									numParticles,
	const PxU32* PX_RESTRICT					phases,
	const PxU32									validPhaseMask,
	const PxU32* PX_RESTRICT					activeIndices)
{
	PxU32 p = threadIdx.x + blockIdx.x * blockDim.x;
	if (p >= numParticles)
		return;

	if (activeIndices)
		p = activeIndices[p];

	const PxVec3 subgridDomainSize = getSubgridDomainSize(sparseGridParams, 0/*sparseGridParams.haloSize*/);

	const PxVec3 pos = positions[p].getXYZ();
	const int3 subgridId = calcSubgridId(pos, subgridDomainSize);

	bool isValidPhase = phases == NULL || (phases[p] & validPhaseMask);

	indices[p] = p;
	hashkeyPerParticle[p] = isValidPhase ? calcSubgridHash(subgridId) : EMPTY_SUBGRID;
}

__device__ void applyMask(PxU32* mask, const PxU32* PX_RESTRICT uniqueSortedHashkey, PxU32 hashkey, PxU32 maxNumSubgrids)
{
	if (hashkey == EMPTY_SUBGRID)
		return;

	PxU32 sortedIdx = 0;
	const bool hashFound = tryFindHashkey(uniqueSortedHashkey, 27 * maxNumSubgrids, hashkey, sortedIdx);
	if (hashFound)
	{
		if (mask[sortedIdx] == 1)
			return; //Was already marked by another thread

		mask[sortedIdx] = 1;

		int i = sortedIdx - 1;
		while (i >= 0 && uniqueSortedHashkey[i] == hashkey)
			mask[i--] = 1;

		i = sortedIdx + 1;
		while (i < 27 * maxNumSubgrids && uniqueSortedHashkey[i] == hashkey)
			mask[i++] = 1;
	}
}

extern "C" __global__ void sg_SparseGridMarkRequiredNeighbors(
	PxU32*									requiredNeighborMask,
	const PxU32* PX_RESTRICT				uniqueSortedHashkey,
	const PxSparseGridParams				sparseGridParams,
	PxU32									neighborhoodSize,
	const PxVec4*							particlePositions,
	const PxU32								numParticles,
	const PxU32* PX_RESTRICT				phases,
	const PxU32								validPhaseMask,
	const PxU32* PX_RESTRICT				activeIndices)
{
	PxU32 i = threadIdx.x + blockIdx.x * blockDim.x;
	if (i >= numParticles)
		return;

	if (activeIndices)
		i = activeIndices[i];

	if (phases && !(phases[i] & validPhaseMask))
		return; //Avoid to allocate sparse grids in regions of non-fluid particles

	const PxVec3 xp = particlePositions[i].getXYZ();

	const PxU32 haloSize = 0; // sparseGridParams.haloSize;
	const PxVec3 subgridDomainSize = getSubgridDomainSize(sparseGridParams, haloSize);
	const int3 subgridId = calcSubgridId(xp, subgridDomainSize); //subgridIdsPerParticle[i]; // flipSubgridHashToId(hashkey);
	const PxReal dx = sparseGridParams.gridSpacing;
	const PxReal invDx = 1.0f / dx;

	const PxVec3 subgridOrigin = PxVec3(
		subgridId.x * dx * (sparseGridParams.subgridSizeX - 2 * haloSize),
		subgridId.y * dx * (sparseGridParams.subgridSizeY - 2 * haloSize),
		subgridId.z * dx * (sparseGridParams.subgridSizeZ - 2 * haloSize));
	const PxVec3 localXp = xp - subgridOrigin;

	int3 gridBaseCoord;
	gridBaseCoord.x = PxClamp(int(floor(localXp.x * invDx)), 0, int(int(sparseGridParams.subgridSizeX) - 2 * haloSize - 1));
	gridBaseCoord.y = PxClamp(int(floor(localXp.y * invDx)), 0, int(int(sparseGridParams.subgridSizeY) - 2 * haloSize - 1));
	gridBaseCoord.z = PxClamp(int(floor(localXp.z * invDx)), 0, int(int(sparseGridParams.subgridSizeZ) - 2 * haloSize - 1));

	//Find the neighboring subgrids (step has values -1/0/1 for x/y/z) that need to exist
	int3 step;
	step.x = gridBaseCoord.x < neighborhoodSize ? -1 : (gridBaseCoord.x >= sparseGridParams.subgridSizeX - 2 * haloSize - neighborhoodSize ? 1 : 0);
	step.y = gridBaseCoord.y < neighborhoodSize ? -1 : (gridBaseCoord.y >= sparseGridParams.subgridSizeY - 2 * haloSize - neighborhoodSize ? 1 : 0);
	step.z = gridBaseCoord.z < neighborhoodSize ? -1 : (gridBaseCoord.z >= sparseGridParams.subgridSizeZ - 2 * haloSize - neighborhoodSize ? 1 : 0);

	//Mark the neighbor subgrids that need to exist such that particles with a radius >0 near the subgrid boundary can transfer their density to the grid	
	PxU32 buffer[8];
	int indexer = 0;

	buffer[indexer++] = calcSubgridHash(subgridId);

	if (step.x != 0 && step.y != 0 && step.z != 0) buffer[indexer++] = subgridHashOffset(subgridId, step.x, step.y, step.z);

	if (step.x != 0 && step.y != 0) buffer[indexer++] = subgridHashOffset(subgridId, step.x, step.y, 0);
	if (step.x != 0 && step.z != 0) buffer[indexer++] = subgridHashOffset(subgridId, step.x, 0, step.z);
	if (step.y != 0 && step.z != 0) buffer[indexer++] = subgridHashOffset(subgridId, 0, step.y, step.z);

	if (step.x != 0) buffer[indexer++] = subgridHashOffset(subgridId, step.x, 0, 0);
	if (step.y != 0) buffer[indexer++] = subgridHashOffset(subgridId, 0, step.y, 0);
	if (step.z != 0) buffer[indexer++] = subgridHashOffset(subgridId, 0, 0, step.z);


	for (int j = 0; j < indexer; ++j)
		applyMask(requiredNeighborMask, uniqueSortedHashkey, buffer[j], sparseGridParams.maxNumSubgrids);
}

extern "C" __global__ void sg_SparseGridSortedArrayToDelta(
	const PxU32*	in,
	const PxU32*    mask,
	PxU32*			out,
	PxU32			n)
{
	const PxU32 i = threadIdx.x + blockIdx.x * blockDim.x;
	if (i < n)
	{
		if (i < n - 1 && in[i] != in[i + 1])
			out[i] = mask ? mask[i] : 1;
		else
			out[i] = 0;
		if (i == n - 1)
			out[i] = mask ? mask[i] : 1;
	}
}

extern "C" __global__ void sg_SparseGridGetUniqueValues(
	const PxU32*		sortedData,
	const PxU32*		indices,
	PxU32*				uniqueValues,
	const PxU32			n,
	PxU32*				subgridNeighborCollector,
	const PxU32			uniqueValuesSize)
{
	const PxU32 i = threadIdx.x + blockIdx.x * blockDim.x;
	if (i < n)
	{
		if (i == n - 1 || indices[i] != indices[i + 1])
		{
			if (indices[i] < uniqueValuesSize)
			{
				uniqueValues[indices[i]] = sortedData[i];

				if (subgridNeighborCollector)
				{
					int4 id = subgridHashToId(sortedData[i]);
					int indexer = 27 * indices[i];
					for (int i = -1; i <= 1; ++i) for (int j = -1; j <= 1; ++j) for (int k = -1; k <= 1; ++k)
						subgridNeighborCollector[indexer++] = calcSubgridHash(make_int3(id.x + i, id.y + j, id.z + k));
				}
			}
		}
	}
}

extern "C" __global__ void sg_SparseGridClearDensity(
	PxReal* PX_RESTRICT		density,
	const PxReal				clearValue,
	const PxU32*				numActiveSubgrids,
	const PxU32					subgridSize
)
{
	const PxU32 idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= (numActiveSubgrids[0]) * subgridSize)
		return;

	density[idx] = clearValue;
}

extern "C" __global__ void sg_SparseGridBuildSubgridNeighbors(
	const PxU32* PX_RESTRICT			uniqueSortedHashkey,
	const PxU32* PX_RESTRICT			numActiveSubgrids,
	const PxU32							maxNumSubgrids,
	PxU32* PX_RESTRICT					subgridNeighbors
)
{
	const PxU32 si = blockIdx.x * blockDim.x + threadIdx.x;

	if (si >= maxNumSubgrids)
		return;

	const PxU32 hash = uniqueSortedHashkey[si];

	int4 sID = subgridHashToId(hash);

	subgridNeighbors[27 * si + SUBGRID_CENTER_IDX] = si;

	for (int z = -1; z <= 1; ++z) for (int y = -1; y <= 1; ++y) for (int x = -1; x <= 1; ++x)
	{
		const int3 nID = make_int3(sID.x + x, sID.y + y, sID.z + z);
		const PxU32 nHash = calcSubgridHash(nID);

		PxU32 n = EMPTY_SUBGRID;
		if (isSubgridInsideRange(nID))
		{
			PxU32 nSortedIdx = 0;
			if (tryFindHashkey(uniqueSortedHashkey, numActiveSubgrids[0]/* + 1*/, nHash, nSortedIdx))
				n = nSortedIdx;
		}
		subgridNeighbors[27 * si + subgridNeighborIndex(x, y, z)] = n;
	}
}

extern "C" __global__ void sg_MarkSubgridEndIndices(const PxU32* sortedParticleToSubgrid, PxU32 numParticles, PxU32* subgridEndIndices)
{
	PxI32 threadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadIndex >= numParticles)
		return;

	if (threadIndex < numParticles - 1)
	{
		if (sortedParticleToSubgrid[threadIndex] != sortedParticleToSubgrid[threadIndex + 1])
			subgridEndIndices[sortedParticleToSubgrid[threadIndex]] = threadIndex + 1;
	}
	else
		subgridEndIndices[sortedParticleToSubgrid[threadIndex]] = numParticles;
}

extern "C" __global__ void sg_ReuseSubgrids(
	const PxSparseGridParams	sparseGridParams, 
	const PxU32*				uniqueHashkeysPerSubgridPreviousUpdate,
	const PxU32*				numActiveSubgridsPreviousUpdate,
	PxU32*						subgridOrderMapPreviousUpdate,
	
	const PxU32*				uniqueHashkeysPerSubgrid,
	const PxU32*				numActiveSubgrids,
	PxU32*						subgridOrderMap)
{
	PxI32 threadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadIndex >= sparseGridParams.maxNumSubgrids)
		return;


	if (threadIndex >= numActiveSubgrids[0]) 
	{
		subgridOrderMap[threadIndex] = EMPTY_SUBGRID;
		return;
	}

	const PxU32 hashkey = uniqueHashkeysPerSubgrid[threadIndex];
	PxU32 sortedIdx = 0;
	const bool hashFound = tryFindHashkey(uniqueHashkeysPerSubgridPreviousUpdate, numActiveSubgridsPreviousUpdate[0], hashkey, sortedIdx);
	if (!hashFound) 
	{
		subgridOrderMap[threadIndex] = NEW_SUBGRID;
		return;
	}

	subgridOrderMap[threadIndex] = subgridOrderMapPreviousUpdate[sortedIdx];
	subgridOrderMapPreviousUpdate[sortedIdx] = REUSED_SUBGRID;
}

PX_FORCE_INLINE __device__ void addIdToUnusedSubgridStack(PxU32 idToAddToStack, PxU32* unusedSubgridStackSize, PxU32* unusedSubgridStack)
{
	const PxU32 id = atomicAdd(unusedSubgridStackSize, 1);
	unusedSubgridStack[id] = idToAddToStack;
}

PX_FORCE_INLINE __device__ PxU32 getSubgridIdFromUnusedStack(PxU32* unusedSubgridStackSize, PxU32* unusedSubgridStack)
{
	const PxU32 id = PxU32(atomicAdd(reinterpret_cast<PxI32*>(unusedSubgridStackSize), -1));
	return unusedSubgridStack[id - 1];
}

//TODO: This method uses atomics. For better debuging, it might be worth to offer a slower variant that generates 100% reproducible results
extern "C" __global__ void sg_AddReleasedSubgridsToUnusedStack(
	const PxU32*	numActiveSubgridsPreviousUpdate,
	const PxU32*	subgridOrderMapPreviousUpdate,
	
	PxU32*			unusedSubgridStackSize,
	PxU32*			unusedSubgridStack)
{
	PxI32 threadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadIndex >= numActiveSubgridsPreviousUpdate[0])
		return;

	if (subgridOrderMapPreviousUpdate[threadIndex] != REUSED_SUBGRID)
		addIdToUnusedSubgridStack(subgridOrderMapPreviousUpdate[threadIndex], unusedSubgridStackSize, unusedSubgridStack);
}

//TODO: This method uses atomics. For better debuging, it might be worth to offer a slower variant that generates 100% reproducible results
extern "C" __global__ void sg_AllocateNewSubgrids(
	const PxU32*				numActiveSubgrids,
	PxU32*						subgridOrderMap,
	
	PxU32*			unusedSubgridStackSize,
	PxU32*			unusedSubgridStack,
	
	const PxU32*	numActiveSubgridsPreviousUpdate,
	const PxU32		maxNumSubgrids)
{
	PxI32 threadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadIndex >= numActiveSubgrids[0])
		return;


	if (numActiveSubgridsPreviousUpdate[0] == 0)
	{
		PxU32 numActiveSubgridsClamped = PxMin(maxNumSubgrids, numActiveSubgrids[0]);

		//Special case to simplify debugging: If no subgrids were active in the previous frame, then all subgrids present now must be new
		//Make sure that the subgrid indices in the first frame are always identical. But the order might change in subsequent frames due to the use of atomics
		//subgridOrderMap[threadIndex] = unusedSubgridStack[maxNumSubgrids - threadIndex - 1];
		subgridOrderMap[threadIndex] = unusedSubgridStack[maxNumSubgrids - numActiveSubgridsClamped + threadIndex]; //Use this line to test with non-default subgrid order to ensure that the code does not only work with the default order
		if (threadIndex == 0)
			unusedSubgridStackSize[0] -= numActiveSubgridsClamped;
		//If launched with 1024 threads per block, one could do per block scan and support 100% reproducible subgrid allocations using a block scan if maxNumSubgrids<=1024
	}
	else
	{
		if (subgridOrderMap[threadIndex] == NEW_SUBGRID)
		{
			subgridOrderMap[threadIndex] = getSubgridIdFromUnusedStack(unusedSubgridStackSize, unusedSubgridStack);
		}
	}
}


