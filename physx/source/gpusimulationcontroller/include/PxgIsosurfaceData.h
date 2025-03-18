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

#ifndef PX_ISOSURFACE_DATA_H
#define PX_ISOSURFACE_DATA_H


#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"
#include "PxSparseGridParams.h"
#include "PxgSparseGridDataStandalone.h"
#include "PxgDenseGridDataStandalone.h"

#if !PX_DOXYGEN
namespace physx
{
#endif
	/**
	\brief Bundles all data used to extract an isosurface on a dense grid
	*/
	struct PxIsosurfaceExtractionData
	{
		PxIsosurfaceExtractionData() : mGrid(), kernelSize(0.0f), restDensity(0), threshold(0), firstCellVert(NULL), swapState(0),
			numVerticesNumIndices(NULL), maxVerts(0), maxTriIds(0), verts(NULL), normals(NULL), triIds(NULL)
		{
		}

		PxDenseGridData mGrid;
		PxReal kernelSize;
		PxReal restDensity;
		PxReal threshold;

		// grid
		PxReal* buffer[2];
		PxU32* firstCellVert;

		PxU32 swapState;

		// mesh
		PxU32* numVerticesNumIndices; //Pointer to a GPU buffer to allow for device to host copy
		PxU32 maxVerts, maxTriIds;

		PxVec4* verts;
		PxVec4* normals;
		PxU32* triIds;

		PxVec4* smoothingBuffer;

		/**
		\brief Access to the density device array

		\return The density devixe array
		*/
		PX_CUDA_CALLABLE PxReal* density()
		{
			return buffer[swapState];
		}

		/**
		\brief Access to the start triangle id per cell device array

		\return The start triangle id per cell device array
		*/
		PX_CUDA_CALLABLE PxU32* firstCellTriId()
		{
			return reinterpret_cast<PxU32*>(buffer[1 - swapState]);
		}

		/**
		\brief The grid's cell size

		\return The cell size
		*/
		PX_CUDA_CALLABLE PxReal getSpacing()
		{
			return mGrid.mGridParams.gridSpacing;
		}

		/**
		\brief The number of cells in the dense grid

		\return The number of cells
		*/
		PX_CUDA_CALLABLE PxU32 maxNumCells()
		{
			return mGrid.maxNumCells();
		}
	};

	/**
	\brief Bundles all data used to extract an isosurface on a sparse grid
	*/
	struct PxSparseIsosurfaceExtractionData
	{
		PxSparseIsosurfaceExtractionData() : mGrid(), kernelSize(0.0f), restDensity(0), threshold(0), firstCellVert(NULL), swapState(0),
			numVerticesNumIndices(NULL), maxVerts(0), maxTriIds(0), verts(NULL), normals(NULL), triIds(NULL)
		{
		}

		PxSparseGridData mGrid;
		PxReal* buffer[2];

		PxReal kernelSize;
		PxReal restDensity;
		PxReal threshold;
		PxU32* firstCellVert;

		PxU32 swapState;

		// mesh
		PxU32* numVerticesNumIndices; //Pointer to a GPU buffer to allow for device to host copy
		PxU32 maxVerts, maxTriIds;

		PxVec4* verts;
		PxVec4* normals;
		PxU32* triIds;

		PxVec4* smoothingBuffer;


		/**
		\brief Access to the density device array

		\return The density device array
		*/
		PX_CUDA_CALLABLE PxReal* density()
		{
			return buffer[swapState];
		}

		/**
		\brief Access to the start triangle id per cell device array

		\return The start triangle id per cell device array
		*/
		PX_CUDA_CALLABLE PxU32* firstCellTriId()
		{
			return reinterpret_cast<PxU32*>(buffer[1 - swapState]);
		}

		/**
		\brief The grid's cell size

		\return The cell size
		*/
		PX_CUDA_CALLABLE PxReal getSpacing()
		{
			return mGrid.mGridParams.gridSpacing;
		}

		/**
		\brief The maximal number of cells in the sparse grid, not all of them are always in use

		\return The number of cells
		*/
		PX_CUDA_CALLABLE PxU32 maxNumCells()
		{
			return mGrid.maxNumCells();
		}
	};

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
