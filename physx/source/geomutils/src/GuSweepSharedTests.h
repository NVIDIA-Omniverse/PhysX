// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_SWEEP_SHARED_TESTS_H
#define GU_SWEEP_SHARED_TESTS_H

#include "GuBoxConversion.h"

namespace physx
{
PX_FORCE_INLINE void computeWorldToBoxMatrix(PxMat34& worldToBox, const physx::Gu::Box& box)
{
	PxMat34 boxToWorld;
	physx::buildMatrixFromBox(boxToWorld, box);
	worldToBox = boxToWorld.getInverseRT();
}

PX_FORCE_INLINE PxU32 getTriangleIndex(PxU32 i, PxU32 cachedIndex)
{
	PxU32 triangleIndex;
	if(i==0)				triangleIndex = cachedIndex;
	else if(i==cachedIndex)	triangleIndex = 0;
	else					triangleIndex = i;
	return triangleIndex;
}
}


#endif
