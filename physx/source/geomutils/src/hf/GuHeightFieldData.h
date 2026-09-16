// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_HEIGHTFIELD_DATA_H
#define GU_HEIGHTFIELD_DATA_H

#include "foundation/PxSimpleTypes.h"
#include "geometry/PxHeightFieldFlag.h"
#include "geometry/PxHeightFieldSample.h"
#include "GuCenterExtents.h"

namespace physx
{

namespace Gu
{

#if PX_VC 
    #pragma warning(push)
	#pragma warning( disable : 4251 ) // class needs to have dll-interface to be used by clients of class
#endif
struct PX_PHYSX_COMMON_API HeightFieldData
{
// PX_SERIALIZATION
	PX_FORCE_INLINE								HeightFieldData()									{}
	PX_FORCE_INLINE								HeightFieldData(const PxEMPTY) :	flags(PxEmpty)	{}
//~PX_SERIALIZATION

	//properties
		// PT: WARNING: bounds must be followed by at least 32bits of data for safe SIMD loading
					CenterExtents				mAABB;
					PxU32						rows;					// PT: WARNING: don't change this member's name (used in ConvX)
					PxU32						columns;				// PT: WARNING: don't change this member's name (used in ConvX)
					PxU32						rowLimit;
					PxU32						colLimit;
					PxU32						nbColumns;
					PxHeightFieldSample*		samples;				// PT: WARNING: don't change this member's name (used in ConvX)
					PxReal						convexEdgeThreshold;

					PxHeightFieldFlags			flags;

					PxHeightFieldFormat::Enum	format;

	PX_FORCE_INLINE	const CenterExtentsPadded&	getPaddedBounds()				const
												{
													// PT: see compile-time assert below
													return static_cast<const CenterExtentsPadded&>(mAABB);
												}
};
#if PX_VC 
     #pragma warning(pop) 
#endif

	// PT: 'getPaddedBounds()' is only safe if we make sure the bounds member is followed by at least 32bits of data
	PX_COMPILE_TIME_ASSERT(PX_OFFSET_OF(Gu::HeightFieldData, rows)>=PX_OFFSET_OF(Gu::HeightFieldData, mAABB)+4);

} // namespace Gu

}

#endif
