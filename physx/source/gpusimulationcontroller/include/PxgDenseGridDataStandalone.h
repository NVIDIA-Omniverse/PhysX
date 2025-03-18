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
