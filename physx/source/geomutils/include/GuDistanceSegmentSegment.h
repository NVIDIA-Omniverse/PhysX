// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_DISTANCE_SEGMENT_SEGMENT_H
#define GU_DISTANCE_SEGMENT_SEGMENT_H

#include "common/PxPhysXCommonConfig.h"
#include "GuSegment.h"
#include "foundation/PxVecMath.h"

namespace physx
{
namespace Gu
{
	// This version fixes accuracy issues (e.g.  TTP 4617), but needs to do 2 square roots in order
	// to find the normalized direction and length of the segments, and then
	// a division in order to renormalize the output

	PX_PHYSX_COMMON_API	PxReal	distanceSegmentSegmentSquared(	const PxVec3& origin0, const PxVec3& dir0, PxReal extent0,
																const PxVec3& origin1, const PxVec3& dir1, PxReal extent1,
																PxReal* s=NULL, PxReal* t=NULL);

	PX_PHYSX_COMMON_API PxReal	distanceSegmentSegmentSquared(	const PxVec3& origin0, const PxVec3& extent0,
																const PxVec3& origin1, const PxVec3& extent1,
																PxReal* s=NULL, PxReal* t=NULL);

	PX_FORCE_INLINE	PxReal	distanceSegmentSegmentSquared(	const Gu::Segment& segment0,
															const Gu::Segment& segment1,
															PxReal* s=NULL, PxReal* t=NULL)
	{
		return distanceSegmentSegmentSquared(	segment0.p0, segment0.computeDirection(),
												segment1.p0, segment1.computeDirection(),
												s, t);
	}

	PX_PHYSX_COMMON_API aos::FloatV distanceSegmentSegmentSquared(	const aos::Vec3VArg p1, const aos::Vec3VArg d1, const aos::Vec3VArg p2, const aos::Vec3VArg d2,
					 								aos::FloatV& param0, 
					 								aos::FloatV& param1);

	// This function do four segment segment closest point test in one go
	aos::Vec4V distanceSegmentSegmentSquared4(  const aos::Vec3VArg p, const aos::Vec3VArg d, 
												const aos::Vec3VArg p02, const aos::Vec3VArg d02, 
												const aos::Vec3VArg p12, const aos::Vec3VArg d12, 
												const aos::Vec3VArg p22, const aos::Vec3VArg d22,
												const aos::Vec3VArg p32, const aos::Vec3VArg d32,
												aos::Vec4V& s, aos::Vec4V& t);

} // namespace Gu
}

#endif
