// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_ALGORITHMS_DATA_H
#define PXG_ALGORITHMS_DATA_H


#include "foundation/PxSimpleTypes.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	/**
	\brief An integer vector with 4 components
	*/
	PX_ALIGN_PREFIX(16) struct PxInt4
	{
		PxI32 x;
		PxI32 y;
		PxI32 z;
		PxI32 w;

		/**
		\brief Comparison operator to check if two instances are equal
		*/
		bool operator==(const PxInt4& rhs) const
		{
			return x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w;
		}
	}PX_ALIGN_SUFFIX(16);
	
	/**
	\brief An bundle of four integer vectors with 4 components each
	*/
	PX_ALIGN_PREFIX(16) struct PxInt4x4
	{
		PxInt4 data[4];
	}PX_ALIGN_SUFFIX(16);

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
