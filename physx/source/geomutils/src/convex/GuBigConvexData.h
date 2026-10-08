// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_BIG_CONVEX_DATA_H
#define GU_BIG_CONVEX_DATA_H

#include "foundation/PxSimpleTypes.h"

namespace physx
{

class BigConvexDataBuilder;
class PxcHillClimb;
class BigConvexData;

// Data

namespace Gu
{

struct Valency
{
	PxU16		mCount;
	PxU16		mOffset;
};
PX_COMPILE_TIME_ASSERT(sizeof(Gu::Valency) == 4);

struct BigConvexRawData
{
	// Support vertex map
	PxU16		mSubdiv;		// "Gaussmap" subdivision
	PxU16		mNbSamples;		// Total #samples in gaussmap PT: this is not even needed at runtime!

	PxU8*		mSamples;
	PX_FORCE_INLINE const PxU8*	getSamples2()	const
	{
		return mSamples + mNbSamples;
	}
	//~Support vertex map

	// Valencies data
	PxU32			mNbVerts;		//!< Number of vertices
	PxU32			mNbAdjVerts;	//!< Total number of adjacent vertices  ### PT: this is useless at runtime and should not be stored here
	Gu::Valency*	mValencies;		//!< A list of mNbVerts valencies (= number of neighbors)
	PxU8*			mAdjacentVerts;	//!< List of adjacent vertices
	//~Valencies data
};
#if PX_P64_FAMILY
PX_COMPILE_TIME_ASSERT(sizeof(Gu::BigConvexRawData) == 40);
#else
PX_COMPILE_TIME_ASSERT(sizeof(Gu::BigConvexRawData) == 24);
#endif

} // namespace Gu

}

#endif
