// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_DISTANCE_SEGMENT_TRIANGLE_H
#define GU_DISTANCE_SEGMENT_TRIANGLE_H

#include "common/PxPhysXCommonConfig.h"
#include "GuSegment.h"
#include "foundation/PxVecMath.h"

namespace physx
{
namespace Gu
{
	PX_PHYSX_COMMON_API PxReal distanceSegmentTriangleSquared(
		const PxVec3& segmentOrigin, const PxVec3& segmentExtent,
		const PxVec3& triangleOrigin, const PxVec3& triangleEdge0, const PxVec3& triangleEdge1,
		PxReal* t=NULL, PxReal* u=NULL, PxReal* v=NULL);

	PX_INLINE PxReal distanceSegmentTriangleSquared(
		const Gu::Segment& segment, 
		const PxVec3& triangleOrigin, const PxVec3& triangleEdge0, const PxVec3& triangleEdge1,
		PxReal* t=NULL, PxReal* u=NULL, PxReal* v=NULL)
	{
		return distanceSegmentTriangleSquared(
			segment.p0, segment.computeDirection(), triangleOrigin, triangleEdge0, triangleEdge1, t, u, v);
	}

	//	closest0 is the closest point on segment pq
	//	closest1 is the closest point on triangle abc
	PX_PHYSX_COMMON_API aos::FloatV distanceSegmentTriangleSquared(
		const aos::Vec3VArg p, const aos::Vec3VArg q,
		const aos::Vec3VArg a, const aos::Vec3VArg b, const aos::Vec3VArg c,
		aos::Vec3V& closest0, aos::Vec3V& closest1);

} // namespace Gu
}

#endif
