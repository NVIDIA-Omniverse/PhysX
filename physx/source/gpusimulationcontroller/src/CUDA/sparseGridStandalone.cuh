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
#include "foundation/PxSimpleTypes.h"
#include "PxSparseGridParams.h"
#include "PxgSparseGridDataStandalone.h"
#include <stdio.h>

#define MAX_SPARSEGRID_DIM 1024
#define MIN_SPARSEGRID_ID -512
#define MAX_SPARSEGRID_ID 511
#define EMPTY_SUBGRID 0xffffffff
#define NEW_SUBGRID 0xfffffffe
#define REUSED_SUBGRID 0xfffffffd
#define OUT_OF_BOUNDS -1
#define SUBGRID_CENTER_IDX 13

using namespace physx;

PX_FORCE_INLINE __device__ __host__ int clampValue(int f, int a, int b)
{
	return max(a, min(f, b));
}

__device__ inline PxVec3 getSubgridDomainSize(const PxSparseGridParams& params, const PxU32 haloSize)
{
	const PxReal dx = params.gridSpacing;
	return PxVec3(dx * (params.subgridSizeX - 2 * haloSize), dx * (params.subgridSizeY - 2 * haloSize), dx * (params.subgridSizeZ - 2 * haloSize));
}

PX_FORCE_INLINE __host__ __device__ int3 calcSubgridId(const PxVec3 pos, const PxVec3 domainSize)
{
	return make_int3((int)PxFloor(pos.x / domainSize.x), (int)PxFloor(pos.y / domainSize.y), (int)PxFloor(pos.z / domainSize.z));
}

PX_FORCE_INLINE __host__ __device__ PxU32 calcSubgridHash(const int3 subgridId)
{
	const int3 shifted = make_int3(subgridId.x - int(MIN_SPARSEGRID_ID), subgridId.y - int(MIN_SPARSEGRID_ID), subgridId.z - int(MIN_SPARSEGRID_ID));
	return MAX_SPARSEGRID_DIM * MAX_SPARSEGRID_DIM * shifted.z + MAX_SPARSEGRID_DIM * shifted.y + shifted.x;
}

PX_FORCE_INLINE __device__ PxU32 subgridHashOffset(int3 subgridId, int offsetX, int offsetY, int offsetZ)
{
	subgridId.x += offsetX;
	subgridId.y += offsetY;
	subgridId.z += offsetZ;
	return calcSubgridHash(subgridId);
}

PX_FORCE_INLINE __host__ __device__ PxI32 subgridNeighborIndex(const PxI32 x, const PxI32 y, const PxI32 z)
{
	return ((x + 1) + 3 * (y + 1) + 9 * (z + 1));
}

template<class T>
static __device__ PxU32 searchSorted(const T* PX_RESTRICT data, const PxU32 numElements, const T& value)
{
	PxU32 left = 0;
	PxU32 right = numElements;

	while ((right - left) > 1)
	{
		PxU32 pos = (left + right) >> 1;
		const T& element = data[pos];
		if (element <= value)
			left = pos;
		else
			right = pos;
	}

	return left;
}

PX_FORCE_INLINE __device__ bool tryFindHashkey(const PxU32* const PX_RESTRICT sortedHashkey, const PxU32 numSubgrids, const PxU32 hashToFind, PxU32& result)
{
	result = searchSorted(sortedHashkey, numSubgrids, hashToFind);
	return sortedHashkey[result] == hashToFind;
}

PX_FORCE_INLINE __host__ __device__ bool isSubgridInsideRange(const int3 val)
{
	return
		val.x >= MIN_SPARSEGRID_ID && val.x <= MAX_SPARSEGRID_ID &&
		val.y >= MIN_SPARSEGRID_ID && val.y <= MAX_SPARSEGRID_ID &&
		val.z >= MIN_SPARSEGRID_ID && val.z <= MAX_SPARSEGRID_ID;
}

PX_FORCE_INLINE __host__ __device__ int4 subgridHashToId(const PxU32 hashKey)
{
	const int ihashKey = static_cast<int>(hashKey);
	return make_int4(
		ihashKey % MAX_SPARSEGRID_DIM + MIN_SPARSEGRID_ID,
		(ihashKey / MAX_SPARSEGRID_DIM) % MAX_SPARSEGRID_DIM + MIN_SPARSEGRID_ID,
		ihashKey / MAX_SPARSEGRID_DIM / MAX_SPARSEGRID_DIM + MIN_SPARSEGRID_ID, 0);
}

PX_FORCE_INLINE __host__ __device__ PxI32 subgridNeighborOffset(const PxU32* const PX_RESTRICT subgridNeighbors, PxI32 si, PxI32 offsetX, PxI32 offsetY, PxI32 offsetZ)
{
	return subgridNeighbors[27 * (si)+subgridNeighborIndex((offsetX), (offsetY), (offsetZ))];
}

PX_FORCE_INLINE __device__ PxI32 mod(PxI32 a, PxI32 b)
{
	return (a + b) % b;
	//return (a + b) & (b - 1); //Assumes b is a power of two
}

PX_FORCE_INLINE __host__ __device__ PxU32 sparseGridAccess(const PxSparseGridParams& sparseGridParams, PxI32 i, PxI32 j, PxI32 k, PxI32 si, const PxU32* subgridOrdering)
{
	if (subgridOrdering)
		si = subgridOrdering[si];
	return i + sparseGridParams.subgridSizeX * (j + sparseGridParams.subgridSizeY * (k + sparseGridParams.subgridSizeZ * si));
}

PX_FORCE_INLINE __device__ PxU32 getIndex(const PxU32* const PX_RESTRICT subgridNeighbors, const PxSparseGridParams& sparseGridParams, PxI32 coordX, PxI32 coordY, PxI32 coordZ, PxU32 si, const PxU32* subgridOrdering)
{
	int haloSize = 0; // sparseGridParams.haloSize;
	coordX += haloSize;
	coordY += haloSize;
	coordZ += haloSize;
	const PxI32 stepX = coordX < 0 ? -1 : (coordX >= sparseGridParams.subgridSizeX ? 1 : 0);
	const PxI32 stepY = coordY < 0 ? -1 : (coordY >= sparseGridParams.subgridSizeY ? 1 : 0);
	const PxI32 stepZ = coordZ < 0 ? -1 : (coordZ >= sparseGridParams.subgridSizeZ ? 1 : 0);

	//if (stepX != 0 || stepY != 0 || stepZ != 0)
	//	printf("neighbor access\n");

	PxU32 n = subgridNeighborOffset(subgridNeighbors, si, stepX, stepY, stepZ);
	if (n == EMPTY_SUBGRID)
		return EMPTY_SUBGRID;

	return sparseGridAccess(sparseGridParams, mod(coordX, sparseGridParams.subgridSizeX), mod(coordY, sparseGridParams.subgridSizeY), mod(coordZ, sparseGridParams.subgridSizeZ), n, subgridOrdering);
}

//Assumes that 0.0 is a valid value for access outside of the grid
PX_FORCE_INLINE __device__ PxReal getGridValue(const PxU32* const PX_RESTRICT subgridNeighbors, const PxReal* data, const PxSparseGridParams& sparseGridParams, PxI32 coordX, PxI32 coordY, PxI32 coordZ, PxU32 si, const PxU32* subgridOrdering)
{
	const PxU32 id = getIndex(subgridNeighbors, sparseGridParams, coordX, coordY, coordZ, si, subgridOrdering);
	if (id == EMPTY_SUBGRID)
		return 0.0f;
	return data[id];
}

//This will transform p to cell local coordinates
PX_FORCE_INLINE __device__ int4 getCellIndexFromPosition(PxVec3& p, const PxSparseGridParams& sparseGridParams, const PxU32* uniqueHashkeyPerSubgrid, const PxU32* numSubgridsInUse)
{
	int haloSize = 0; // sparseGridParams.haloSize;
	const PxVec3 subgridDomainSize = getSubgridDomainSize(sparseGridParams, haloSize);
	int3 subgridId = calcSubgridId(p, subgridDomainSize);
	PxU32 subgridHash = calcSubgridHash(subgridId);

	PxU32 sortedIdx = 0;
	const bool hashFound = tryFindHashkey(uniqueHashkeyPerSubgrid, numSubgridsInUse[0], subgridHash, sortedIdx);
	if (!hashFound)
	{
		//printf("Hash not found %i\n", subgridHash);
		return make_int4(-1, -1, -1, OUT_OF_BOUNDS);
	}

	const PxReal dx = sparseGridParams.gridSpacing;
	const PxReal invDx = 1.0f / dx;

	const PxVec3 subgridOrigin = PxVec3(
		subgridId.x * dx * (sparseGridParams.subgridSizeX - 2 * haloSize),
		subgridId.y * dx * (sparseGridParams.subgridSizeY - 2 * haloSize),
		subgridId.z * dx * (sparseGridParams.subgridSizeZ - 2 * haloSize));
	p = p - subgridOrigin;

	int4 result = make_int4(PxI32(PxFloor(p.x * invDx)), PxI32(PxFloor(p.y * invDx)), PxI32(PxFloor(p.z * invDx)), sortedIdx);
	result.x = PxClamp(result.x, 0, sparseGridParams.subgridSizeX - 2 * haloSize-1);
	result.y = PxClamp(result.y, 0, sparseGridParams.subgridSizeY - 2 * haloSize-1);
	result.z = PxClamp(result.z, 0, sparseGridParams.subgridSizeZ - 2 * haloSize-1);
	return result;
}

PX_FORCE_INLINE __device__ PxVec3 getLocationFromHashkey(const PxU32 hash, const PxSparseGridParams& sparseGridParams, const int4& index)
{
	int haloSize = 0; // sparseGridParams.haloSize;
	const int4 subgridId = subgridHashToId(hash);
	const PxVec3 subgridOrigin = PxVec3(
		subgridId.x * sparseGridParams.gridSpacing * (sparseGridParams.subgridSizeX - 2 * haloSize),
		subgridId.y * sparseGridParams.gridSpacing * (sparseGridParams.subgridSizeY - 2 * haloSize),
		subgridId.z * sparseGridParams.gridSpacing * (sparseGridParams.subgridSizeZ - 2 * haloSize));
	return subgridOrigin + PxVec3(index.x, index.y, index.z) * sparseGridParams.gridSpacing;
}

PX_FORCE_INLINE __device__ int4 getGridCoordinates(const PxSparseGridParams& sparseGridParams, int threadIndex)
{
	const PxU32 numSubgridCells = sparseGridParams.subgridSizeX * sparseGridParams.subgridSizeY * sparseGridParams.subgridSizeZ;
	const PxU32 si = threadIndex / numSubgridCells;

	PxI32 localThreadIndex = threadIndex - si * numSubgridCells;
	const PxI32 xi = localThreadIndex % sparseGridParams.subgridSizeX;
	const PxI32 yi = (localThreadIndex / sparseGridParams.subgridSizeX) % sparseGridParams.subgridSizeY;
	const PxI32 zi = localThreadIndex / (sparseGridParams.subgridSizeX * sparseGridParams.subgridSizeY);

	//Following code assumes that subgridSizeX and subgridSizeY are a power of two
	/*const PxI32 xi = localThreadIndex & (sparseGridParams.subgridSizeX - 1);
	const PxI32 yi = (localThreadIndex / sparseGridParams.subgridSizeX) & (sparseGridParams.subgridSizeY - 1);
	const PxI32 zi = localThreadIndex / (sparseGridParams.subgridSizeX * sparseGridParams.subgridSizeY);*/

	/*if(sparseGridParams.haloSize>0 &&
		(xi < sparseGridParams.haloSize || yi < sparseGridParams.haloSize || zi < sparseGridParams.haloSize ||
			xi >= sparseGridParams.subgridSizeX - sparseGridParams.haloSize || 
			yi >= sparseGridParams.subgridSizeY - sparseGridParams.haloSize || 
			zi >= sparseGridParams.subgridSizeZ - sparseGridParams.haloSize))
		return make_int4(-1, -1, -1, OUT_OF_BOUNDS);*/

	const PxU32 haloSize = 0; // sparseGridParams.haloSize;
	return make_int4(xi - haloSize, yi - haloSize, zi - haloSize, si);
}

//Functions for the PxSparseGridData class - make sure they have the same name and aruments as their counterparts of the dense grid to simplify templating
PX_FORCE_INLINE __device__ int4 getGridCoordinates(const PxSparseGridData& data, int threadIndex)
{
	return getGridCoordinates(data.mGridParams, threadIndex);
}

PX_FORCE_INLINE __device__ PxU32 getCellIndex(PxSparseGridData& data, int4 index, bool applySubgridOrder = true)
{
	int haloSize = 0; // data.mGridParams.haloSize;
	index.x += haloSize;
	index.y += haloSize;
	index.z += haloSize;
	return sparseGridAccess(data.mGridParams, index.x, index.y, index.z, index.w, applySubgridOrder ? data.mSubgridOrderMap : NULL);	
}

PX_FORCE_INLINE __device__ PxU32 getCellIndex(PxSparseGridData& data, const int4& index, PxI32 offsetX, PxI32 offsetY, PxI32 offsetZ, bool applySubgridOrder = true)
{
	/*if (applySubgridOrder && data.subgridOrderMap)
	{
		if (index.w < 0 || data.subgridOrderMap[index.w] < 0 || data.subgridOrderMap[index.w] >= data.mNumSubgridsInUse[0])
			printf("problem\n");
	}*/

	return getIndex(data.mSubgridNeighbors, data.mGridParams, index.x + offsetX, index.y + offsetY, index.z + offsetZ, index.w, applySubgridOrder ? data.mSubgridOrderMap : NULL);
}

PX_FORCE_INLINE __device__ PxU32 getCellIndexSafe(PxSparseGridData& data, const int4& index, PxI32 offsetX, PxI32 offsetY, PxI32 offsetZ, bool applySubgridOrder = true)
{
	return getCellIndex(data, index, offsetX, offsetY, offsetZ, applySubgridOrder);
}

//Assumes that 0.0 is a valid value for access outside of the grid
PX_FORCE_INLINE __device__ PxReal getGridValue(PxSparseGridData& data, const PxReal* dataSource, const int4& index, PxI32 offsetX, PxI32 offsetY, PxI32 offsetZ)
{
	return getGridValue(data.mSubgridNeighbors, dataSource, data.mGridParams, index.x + offsetX, index.y + offsetY, index.z + offsetZ, index.w, data.mSubgridOrderMap);
}

//Assumes that 0.0 is a valid value for access outside of the grid
PX_FORCE_INLINE __device__ PxReal getGridValueSafe(PxSparseGridData& data, const PxReal* dataSource, const int4& index, PxI32 offsetX, PxI32 offsetY, PxI32 offsetZ)
{
	return getGridValue(data, dataSource, index, offsetX, offsetY, offsetZ);
}

PX_FORCE_INLINE __device__ bool outOfRange(PxSparseGridData& data, const int threadIndex)
{
	const PxSparseGridParams& sparseGridParams = data.mGridParams;
	return threadIndex >= sparseGridParams.maxNumSubgrids * sparseGridParams.subgridSizeX * sparseGridParams.subgridSizeY * sparseGridParams.subgridSizeZ;
}

PX_FORCE_INLINE __device__ bool outOfActiveCells(PxSparseGridData& data, const int threadIndex)
{
	const PxSparseGridParams& sparseGridParams = data.mGridParams;
	return threadIndex >= data.mNumSubgridsInUse[0] * sparseGridParams.subgridSizeX * sparseGridParams.subgridSizeY * sparseGridParams.subgridSizeZ;
}

PX_FORCE_INLINE __device__ bool outOfBounds(PxSparseGridData& data, const int4& index)
{
	return index.w >= data.mNumSubgridsInUse[0] || index.w >= data.mGridParams.maxNumSubgrids || index.w == OUT_OF_BOUNDS;
	//return data.subgridMask[index.w] == SUBGRID_INACTIVE;
}

PX_FORCE_INLINE __device__ bool isLastCell(PxSparseGridData& data, const int threadIndex)
{
	const PxSparseGridParams& sparseGridParams = data.mGridParams;
	return threadIndex == sparseGridParams.maxNumSubgrids * sparseGridParams.subgridSizeX * sparseGridParams.subgridSizeY * sparseGridParams.subgridSizeZ - 1;
}

PX_FORCE_INLINE __device__ PxVec3 getLocation(PxSparseGridData& data, const int4& index)
{
	const PxSparseGridParams& sparseGridParams = data.mGridParams;
	const PxU32 hash = data.mUniqueHashkeyPerSubgrid[index.w];
	return getLocationFromHashkey(hash, sparseGridParams, index);
}

//This will transform p to cell local coordinates
PX_FORCE_INLINE __device__ int4 getCellIndexFromParticleAndTransformToLocalCoordinates(PxSparseGridData& data, PxVec3& p)
{
	return getCellIndexFromPosition(p, data.mGridParams, data.mUniqueHashkeyPerSubgrid, data.mNumSubgridsInUse);
}

