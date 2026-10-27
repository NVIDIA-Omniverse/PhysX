// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuIntersectionCapsuleTriangle.h"
#include "GuDistancePointSegment.h"

using namespace physx;
using namespace Gu;

bool Gu::intersectCapsuleTriangle(const PxVec3& N, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2, const Gu::Capsule& capsule, const CapsuleTriangleOverlapData& params)
{
	PX_ASSERT(capsule.p0!=capsule.p1);

	{
		const PxReal d2 = distancePointSegmentSquaredInternal(capsule.p0, params.mCapsuleDir, p0);
		if(d2<=capsule.radius*capsule.radius)
			return true;
	}

//	const PxVec3 N = (p0 - p1).cross(p0 - p2);

	if(!testAxis(p0, p1, p2, capsule, N))
		return false;

	if(!testAxis(p0, p1, p2, capsule, computeEdgeAxis(p0, p1 - p0, capsule.p0, params.mCapsuleDir, params.mBDotB, params.mOneOverBDotB)))
		return false;

	if(!testAxis(p0, p1, p2, capsule, computeEdgeAxis(p1, p2 - p1, capsule.p0, params.mCapsuleDir, params.mBDotB, params.mOneOverBDotB)))
		return false;

	if(!testAxis(p0, p1, p2, capsule, computeEdgeAxis(p2, p0 - p2, capsule.p0, params.mCapsuleDir, params.mBDotB, params.mOneOverBDotB)))
		return false;

	return true;
}
