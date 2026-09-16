// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuDistancePointTetrahedron.h"
#include "GuDistancePointTriangle.h"

using namespace physx;

PxVec3 Gu::closestPtPointTetrahedron(const PxVec3& p, const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d, const PxVec4& result)
{
	const PxVec3 ab = b - a;
	const PxVec3 ac = c - a;
	const PxVec3 ad = d - a;
	const PxVec3 bc = c - b;
	const PxVec3 bd = d - b;
	//point is outside of this face
	PxVec3 bestClosestPt(0.f, 0.f, 0.f);
	PxReal bestSqDist = PX_MAX_F32;
	if (result.x < 0.f)
	{
		// 0, 1, 2
		bestClosestPt = closestPtPointTriangle2(p, a, b, c, ab, ac);
		bestSqDist = bestClosestPt.dot(bestClosestPt);
	}

	if (result.y < 0.f)
	{
		// 0, 2, 3
		const PxVec3 closestPt = closestPtPointTriangle2(p, a, c, d, ac, ad);
		const PxReal sqDist = closestPt.dot(closestPt);
		if (sqDist < bestSqDist)
		{
			bestClosestPt = closestPt;
			bestSqDist = sqDist;
		}
	}

	if (result.z < 0.f)
	{
		// 0, 3, 1
		const PxVec3 closestPt = closestPtPointTriangle2(p, a, d, b, ad, ab);
		const PxReal sqDist = closestPt.dot(closestPt);
		if (sqDist < bestSqDist)
		{
			bestClosestPt = closestPt;
			bestSqDist = sqDist;
		}
	}

	if (result.w < 0.f)
	{
		// 1, 3, 2
		const PxVec3 closestPt = closestPtPointTriangle2(p, b, d, c, bd, bc);
		const PxReal sqDist = closestPt.dot(closestPt);
		if (sqDist < bestSqDist)
		{
			bestClosestPt = closestPt;
			bestSqDist = sqDist;
		}
	}

	return bestClosestPt;
}
