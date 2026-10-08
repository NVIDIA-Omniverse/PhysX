// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "common/PxPhysXCommonConfig.h"

#include "geometry/PxConvexCoreGeometry.h"
#include "geometry/PxBoxGeometry.h"

#include "GuConvexSupport.h"

using namespace physx;
using namespace Gu;

PX_CUDA_CALLABLE
// generates contacts between a plane and a convex
PxU32 physx::Gu::generateContacts(const PxPlane& plane0, const ConvexShape& convex1, const PxReal contactDist,
	PxVec3& normal, PxVec3 points[MAX_CONVEX_CONTACTS], PxReal dists[MAX_CONVEX_CONTACTS])
{
	normal = -plane0.n;

	const PxVec3 point1 = convex1.support(normal);
	const PxReal dist = plane0.distance(point1);

	PxU32 numContacts = 0;

	if (dist < contactDist)
	{
		PxVec3 faceNormal, facePoints[Gu::ConvexCore::MAX_FACE_POINTS];
		const PxU32 numPoints = convex1.contactFace(normal, point1, faceNormal, facePoints);
				
		if (numPoints == 0)
		{
			const PxVec3 point = point1 + normal * dist * 0.5f;
			points[numContacts] = point;
			dists[numContacts] = dist;
			++numContacts;
		}

		for (PxU32 i = 0; i < numPoints; ++i)
		{
			const PxVec3 p1 = facePoints[i];
			const PxReal d = plane0.distance(p1);
			points[numContacts] = p1 + normal * d * 0.5f;
			dists[numContacts] = d;
			++numContacts;
		}
	}

	return numContacts;
}
