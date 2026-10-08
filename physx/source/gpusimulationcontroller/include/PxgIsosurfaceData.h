// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
