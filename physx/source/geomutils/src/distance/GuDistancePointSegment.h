// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_DISTANCE_POINT_SEGMENT_H
#define GU_DISTANCE_POINT_SEGMENT_H

#include "common/PxPhysXCommonConfig.h"
#include "GuSegment.h"

namespace physx
{
namespace Gu
{
	// dir = p1 - p0
	PX_FORCE_INLINE PxReal distancePointSegmentSquaredInternal(const PxVec3& p0, const PxVec3& dir, const PxVec3& point, PxReal* param=NULL)
	{
		PxVec3 diff = point - p0;
		PxReal fT = diff.dot(dir);

		if(fT<=0.0f)
		{
			fT = 0.0f;
		}
		else
		{
			const PxReal sqrLen = dir.magnitudeSquared();
			if(fT>=sqrLen)
			{
				fT = 1.0f;
				diff -= dir;
			}
			else
			{
				fT /= sqrLen;
				diff -= fT*dir;
			}
		}

		if(param)
			*param = fT;

		return diff.magnitudeSquared();
	}

	/**
	A segment is defined by S(t) = mP0 * (1 - t) + mP1 * t, with 0 <= t <= 1
	Alternatively, a segment is S(t) = Origin + t * Direction for 0 <= t <= 1.
	Direction is not necessarily unit length. The end points are Origin = mP0 and Origin + Direction = mP1.
	*/
	PX_FORCE_INLINE PxReal distancePointSegmentSquared(const PxVec3& p0, const PxVec3& p1, const PxVec3& point, PxReal* param=NULL)
	{
		return distancePointSegmentSquaredInternal(p0, p1 - p0, point, param);
	}

	PX_INLINE PxReal distancePointSegmentSquared(const Gu::Segment& segment, const PxVec3& point, PxReal* param=NULL)
	{
		return distancePointSegmentSquared(segment.p0, segment.p1, point, param);
	}

} // namespace Gu

}

#endif
