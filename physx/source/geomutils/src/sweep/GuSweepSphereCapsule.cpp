// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuSweepSphereCapsule.h"
#include "GuSphere.h"
#include "GuCapsule.h"
#include "GuDistancePointSegment.h"
#include "GuSweepSphereSphere.h"
#include "GuIntersectionRayCapsule.h"

using namespace physx;
using namespace Gu;

bool Gu::sweepSphereCapsule(const Sphere& sphere, const Capsule& lss, const PxVec3& dir, PxReal length, PxReal& d, PxVec3& ip, PxVec3& nrm, PxHitFlags hitFlags)
{
	const PxReal radiusSum = lss.radius + sphere.radius;

	if(!(hitFlags & PxHitFlag::eASSUME_NO_INITIAL_OVERLAP))
	{
		// PT: test if shapes initially overlap
		if(distancePointSegmentSquared(lss.p0, lss.p1, sphere.center)<radiusSum*radiusSum)
		{
			d	= 0.0f;
			nrm	= -dir;
			return true;
		}
	}

	if(lss.p0 == lss.p1)
	{
		// Sphere vs. sphere
		if(sweepSphereSphere(sphere.center, sphere.radius, lss.p0, lss.radius, -dir*length, d, nrm))
		{
			d*=length;
//				if(hitFlags & PxHitFlag::ePOSITION)	// PT: TODO
				ip = sphere.center + nrm * sphere.radius;
			return true;
		}
		return false;
	}

	// Create inflated capsule
	Capsule Inflated(lss.p0, lss.p1, radiusSum);

	// Raycast against it
	PxReal t = 0.0f;
	if(intersectRayCapsule(sphere.center, dir, Inflated, t))
	{
		if(t>=0.0f && t<=length)
		{
			d = t;

// PT: TODO:
//			const PxIntBool needsImpactPoint = hitFlags & PxHitFlag::ePOSITION;
//			if(needsImpactPoint || hitFlags & PxHitFlag::eNORMAL)
			{
				// Move capsule against sphere
				const PxVec3 tdir = t*dir;
				Inflated.p0 -= tdir;
				Inflated.p1 -= tdir;

				// Compute closest point between moved capsule & sphere
				distancePointSegmentSquared(Inflated, sphere.center, &t);
				Inflated.computePoint(ip, t);

				// Normal
				nrm = (ip - sphere.center);
				nrm.normalize();

//					if(needsImpactPoint)	// PT: TODO
					ip -= nrm * lss.radius;
			}
			return true;
		}
	}
	return false;
}
