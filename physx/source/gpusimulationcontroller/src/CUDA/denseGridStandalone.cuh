// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "vector_types.h"
#include "foundation/PxSimpleTypes.h"
#include "PxgDenseGridData.h"

//Should have the same value as the same define in sparseGrid.cuh
#define EMPTY_SUBGRID 0xffffffff 

namespace physx
{

PX_FORCE_INLINE __device__ __host__ int getCellNr(int numCellsX, int numCellsY, int xi, int yi, int zi)
{
	return (zi * numCellsY + yi) * numCellsX + xi;
}
PX_FORCE_INLINE __device__ __host__ int getCellNr(const int3& gridSize, int xi, int yi, int zi)
{
	return getCellNr(gridSize.x, gridSize.y, xi, yi, zi);
}

PX_FORCE_INLINE __device__ __host__ int4 getCellCoords(int numCellsX, int numCellsY, int cellNr)
{
	int4 result;
	result.x = cellNr % numCellsX;
	cellNr /= numCellsX;
	result.y = cellNr % numCellsY;
	result.z = cellNr / numCellsY;
	result.w = -1;
	return result;
}
PX_FORCE_INLINE __device__ __host__ int4 getCellCoords(const int3& gridSize, int cellNr)
{
	return getCellCoords(gridSize.x, gridSize.y, cellNr);
}

//Functions for the PxDenseGridData class - make sure they have the same name and aruments as their counterparts of the sparse grid to simplify templating
PX_FORCE_INLINE __device__ int4 getGridCoordinates(const PxDenseGridData& data, int threadIndex)
{
	return getCellCoords(data.mGridParams.numCellsX, data.mGridParams.numCellsY, threadIndex);
}

PX_FORCE_INLINE __device__ PxU32 getCellIndex(PxDenseGridData& data, const int4& index, bool applySubgridOrder = true)
{
	return getCellNr(data.mGridParams.numCellsX, data.mGridParams.numCellsY, index.x, index.y, index.z);
}

PX_FORCE_INLINE __device__ PxU32 getCellIndex(PxDenseGridData& data, const int4& index, PxI32 offsetX, PxI32 offsetY, PxI32 offsetZ, bool applySubgridOrder = true)
{
	return getCellNr(data.mGridParams.numCellsX, data.mGridParams.numCellsY, index.x + offsetX, index.y + offsetY, index.z + offsetZ);
}

PX_FORCE_INLINE __device__ PxU32 getCellIndexSafe(PxDenseGridData& data, const int4& index, PxI32 offsetX, PxI32 offsetY, PxI32 offsetZ, bool applySubgridOrder = true)
{
	if (index.x + offsetX < 0 || index.y + offsetY < 0 || index.z + offsetZ < 0 || index.x + offsetX >= data.mGridParams.numCellsX || index.y + offsetY >= data.mGridParams.numCellsY || index.z + offsetZ >= data.mGridParams.numCellsZ)
		return EMPTY_SUBGRID;
	return getCellNr(data.mGridParams.numCellsX, data.mGridParams.numCellsY, index.x + offsetX, index.y + offsetY, index.z + offsetZ);
}

PX_FORCE_INLINE __device__ PxReal getGridValue(PxDenseGridData& data, const PxReal* dataSource, const int4& index, PxI32 offsetX, PxI32 offsetY, PxI32 offsetZ)
{
	return dataSource[getCellIndex(data, index, offsetX, offsetY, offsetZ)];
}

//Assumes that 0.0 is a valid value for access outside of the grid
PX_FORCE_INLINE __device__ PxReal getGridValueSafe(PxDenseGridData& data, const PxReal* dataSource, int4 index, PxI32 offsetX, PxI32 offsetY, PxI32 offsetZ)
{
	if (index.x + offsetX < 0 || index.y + offsetY < 0 || index.z + offsetZ < 0 || index.x + offsetX >= data.mGridParams.numCellsX || index.y + offsetY >= data.mGridParams.numCellsY || index.z + offsetZ >= data.mGridParams.numCellsZ)
		return 0.0f;
	return dataSource[getCellIndex(data, index, offsetX, offsetY, offsetZ)];
}

PX_FORCE_INLINE __device__ bool outOfRange(PxDenseGridData& data, const int threadIndex)
{
	return threadIndex >= data.maxNumCells();
}

PX_FORCE_INLINE __device__ bool outOfActiveCells(PxDenseGridData& data, const int threadIndex)
{
	return threadIndex >= data.maxNumCells(); //All cells are always active on a dense grid
}

PX_FORCE_INLINE __device__ bool outOfBounds(PxDenseGridData& data, const int4& index)
{
	return index.x >= data.mGridParams.numCellsX - 1 || index.y >= data.mGridParams.numCellsY - 1 || index.z >= data.mGridParams.numCellsZ - 1 || index.x < 0 || index.y < 0 || index.z < 0;
}

PX_FORCE_INLINE __device__ bool isLastCell(PxDenseGridData& data, const int threadIndex)
{
	return threadIndex == (data.mGridParams.numCellsX - 1)*(data.mGridParams.numCellsY - 1)*(data.mGridParams.numCellsZ - 1) - 1;
}

PX_FORCE_INLINE __device__ PxVec3 getLocation(PxDenseGridData& data, const int4& index)
{
	return data.mGridParams.origin + PxVec3(index.x, index.y, index.z) * data.mGridParams.gridSpacing;
}

PX_FORCE_INLINE __device__ int4 getCellIndexFromParticleAndTransformToLocalCoordinates(PxDenseGridData& data, PxVec3& p)
{
	p = p - data.mGridParams.origin;
	PxReal invDx = 1.0f / data.mGridParams.gridSpacing;
	PxI32 cxi = (int)PxFloor(p.x * invDx);
	PxI32 cyi = (int)PxFloor(p.y * invDx);
	PxI32 czi = (int)PxFloor(p.z * invDx);
	return make_int4(cxi, cyi, czi, -1);
}

} // namespace physx
