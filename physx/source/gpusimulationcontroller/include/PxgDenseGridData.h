// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_DENSE_GRID_DATA_H
#define PXG_DENSE_GRID_DATA_H


#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"
#include "PxSparseGridParams.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	struct PxgDenseGridData
	{
		PxVec3 mOrigin;
		PxReal mGridSpacing;
		PxU32 mNumCellsX;
		PxU32 mNumCellsY;
		PxU32 mNumCellsZ;

		PX_FORCE_INLINE PX_CUDA_CALLABLE PxReal getGridSpacing() const { return mGridSpacing; }

		PX_FORCE_INLINE PX_CUDA_CALLABLE PxI32 getHaloSize() const { return 0; }

		PX_FORCE_INLINE PX_CUDA_CALLABLE PxU32 maxNumCells()
		{
			return mNumCellsX * mNumCellsY * mNumCellsZ;
		}
	};

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
