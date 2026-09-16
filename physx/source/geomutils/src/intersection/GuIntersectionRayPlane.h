// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_INTERSECTION_RAY_PLANE_H
#define GU_INTERSECTION_RAY_PLANE_H

#include "foundation/PxPlane.h"

namespace physx
{
namespace Gu
{
	// Returns true if line and plane are not parallel
	PX_INLINE bool intersectRayPlane(const PxVec3& orig, const PxVec3& dir, const PxPlane& plane, float& distanceAlongLine, PxVec3* pointOnPlane = NULL)
	{
		const float dn = dir.dot(plane.n);
		if(-1E-7f < dn && dn < 1E-7f)
			return false; // parallel

		distanceAlongLine = -plane.distance(orig)/dn;

		if(pointOnPlane)
			*pointOnPlane = orig + distanceAlongLine * dir;

		return true;
	}

} // namespace Gu

}

#endif
