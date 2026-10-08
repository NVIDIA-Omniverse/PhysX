// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "geomutils/PxContactBuffer.h"
#include "GuDistancePointSegment.h"
#include "GuContactMethodImpl.h"
#include "GuInternal.h"

using namespace physx;

bool Gu::contactSphereCapsule(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);

	const PxSphereGeometry& sphereGeom = checkedCast<PxSphereGeometry>(shape0);
	const PxCapsuleGeometry& capsuleGeom = checkedCast<PxCapsuleGeometry>(shape1);

	// PT: get capsule in local space
	const PxVec3 capsuleLocalSegment = getCapsuleHalfHeightVector(transform1, capsuleGeom);
	const Segment localSegment(capsuleLocalSegment, -capsuleLocalSegment);

	// PT: get sphere in capsule space
	const PxVec3 sphereCenterInCapsuleSpace = transform0.p - transform1.p;

	const PxReal radiusSum = sphereGeom.radius + capsuleGeom.radius;
	const PxReal inflatedSum = radiusSum + params.mContactDistance;

	// PT: compute distance between sphere center & capsule's segment
	PxReal u;
	const PxReal squareDist = distancePointSegmentSquared(localSegment, sphereCenterInCapsuleSpace, &u);
	if(squareDist >= inflatedSum*inflatedSum)
		return false;

	// PT: compute contact normal
	PxVec3 normal = sphereCenterInCapsuleSpace - localSegment.getPointAt(u);
		
	// We do a *manual* normalization to check for singularity condition
	const PxReal lenSq = normal.magnitudeSquared();
	if(lenSq==0.0f) 
		normal = PxVec3(1.0f, 0.0f, 0.0f);	// PT: zero normal => pick up random one
	else
		normal *= PxRecipSqrt(lenSq);

	// PT: compute contact point
	const PxVec3 point = sphereCenterInCapsuleSpace + transform1.p - normal * sphereGeom.radius;

	// PT: output unique contact
	contactBuffer.contact(point, normal, PxSqrt(squareDist) - radiusSum);
	return true;
}
