// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_INTERSECTION_TRIANGLE_TRIANGLE_H
#define GU_INTERSECTION_TRIANGLE_TRIANGLE_H

#include "GuSegment.h"
#include "common/PxPhysXCommonConfig.h"

namespace physx
{
namespace Gu
{
	/**
	Tests if a two triangles intersect

	\param a1				[in] First point of the first triangle
	\param b1				[in] Second point of the first triangle
	\param c1				[in] Third point of the first triangle
	\param a2				[in] First point of the second triangle
	\param b2				[in] Second point of the second triangle
	\param c2				[in] Third point of the second triangle
	\param ignoreCoplanar	[in] True to filter out coplanar triangles
	\return	true if triangles intersect
	*/
	PX_PHYSX_COMMON_API bool intersectTriangleTriangle(	const PxVec3& a1, const PxVec3& b1, const PxVec3& c1,
														const PxVec3& a2, const PxVec3& b2, const PxVec3& c2,
														bool ignoreCoplanar = false);
} // namespace Gu
}

#endif
