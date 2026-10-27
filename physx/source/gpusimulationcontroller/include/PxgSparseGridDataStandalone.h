// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_SPARSE_GRID_DATA_STANDALONE_H
#define PXG_SPARSE_GRID_DATA_STANDALONE_H


#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"
#include "PxSparseGridParams.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	/**
	\brief Minimal set of data to access cells in a sparse grid
	*/
	struct PxSparseGridData
	{
		PxSparseGridParams mGridParams;		//!< The grid descriptor
		PxU32* mUniqueHashkeyPerSubgrid;	//!< A unique id for every subgrid that is currently in use
		PxU32* mSubgridNeighbors;			//!< Contains 27 elements for every subgrid in use and provides indices to the neighbors in the 3x3x3 neighborhood
		PxU32* mNumSubgridsInUse;			//!< The number of subgrids that are currently in use
		PxU32* mSubgridOrderMap;			//!< Only used for subgrids that have subgrid reuse enabled for consistent order across frames

		PxSparseGridData() : mSubgridOrderMap(NULL) {}

		/**
		\brief The number of cells in the sparse grid, not all of them are always in use

		\return The number of cells
		*/
		PX_FORCE_INLINE PX_CUDA_CALLABLE PxU32 maxNumCells()
		{
			return mGridParams.maxNumSubgrids * mGridParams.subgridSizeX *mGridParams.subgridSizeY *mGridParams.subgridSizeZ;
		}

		/**
		\brief The sparse grid's cell size

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
