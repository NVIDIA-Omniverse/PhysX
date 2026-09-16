// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_DENSE_GRID_DATA_H
#define PX_DENSE_GRID_DATA_H


#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"
#include "PxSparseGridParams.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	/**
	\brief Descriptor for axis aligned dense grids
	*/
	struct PxDenseGridParams
	{
		PxVec3 origin;			//!< The origin of the dense grid which is the corner with smallest x/y/z coordinates
		PxU32 numCellsX;		//!< The number of cells in x direction
		PxU32 numCellsY;		//!< The number of cells in y direction
		PxU32 numCellsZ;		//!< The number of cells in z direction
		PxReal gridSpacing;		//!< The cell size

		PxDenseGridParams() : origin(PxVec3(0.0f)), numCellsX(0), numCellsY(0), numCellsZ(0), gridSpacing(0) {}
	
		PxDenseGridParams(const PxVec3& origin_, PxU32 numCellsX_, PxU32 numCellsY_, PxU32 numCellsZ_, PxReal gridSpacing_) 
			: origin(origin_), numCellsX(numCellsX_), numCellsY(numCellsY_), numCellsZ(numCellsZ_), gridSpacing(gridSpacing_) {}
	};

	/**
	\brief Minimal set of data to access cells in a dense grid
	*/
	struct PxDenseGridData
	{
		PxDenseGridParams mGridParams;	//!< The grid descriptor

		PxDenseGridData() : mGridParams() {}

		/**
		\brief The number of cells in the dense grid

		\return The number of cells
		*/
		PX_FORCE_INLINE PX_CUDA_CALLABLE PxU32 maxNumCells()
		{
			return mGridParams.numCellsX * mGridParams.numCellsY * mGridParams.numCellsZ;
		}

		/**
		\brief The dense grid's cell size

		\return The cell size
		*/
		PX_FORCE_INLINE PX_CUDA_CALLABLE PxReal getCellSize()
		{
			return mGridParams.gridSpacing;
		}
	};

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
