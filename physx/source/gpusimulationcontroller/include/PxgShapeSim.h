// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_SHAPESIM_H
#define PXG_SHAPESIM_H

#include "foundation/PxTransform.h"
#include "foundation/PxBounds3.h"
#include "PxNodeIndex.h"

namespace physx
{
	struct PxgShapeSim
	{
		PxTransform		mTransform;			//
		PxBounds3		mLocalBounds;		// local bounds
		PxNodeIndex		mBodySimIndex;		// body sim index
		PxU32			mHullDataIndex;		// this index is shared with the np hull data(hull need to be gpu compatible)
		PxU16			mShapeFlags;		// shape flags
		PxU16			mShapeType;			// this indicates what type of shape(sphere, capsule, box or convexhull)
	};

	struct PxgNewShapeSim : PxgShapeSim
	{
		PxU32			mElementIndex;		// transform cache and bound index
	};

}//physx

#endif