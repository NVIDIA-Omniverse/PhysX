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
