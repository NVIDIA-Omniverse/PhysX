// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuIntersectionRayCapsule.h"
#include "foundation/PxBasicTemplates.h"

using namespace physx;

static bool intersectRaySphere(const PxVec3& rayOrigin, const PxVec3& rayDir, const PxVec3& sphereCenter, float radius2, float& tmin, float& tmax)
{
	const PxVec3 CO = rayOrigin - sphereCenter;

	const float a = rayDir.dot(rayDir);
	const float b = 2.0f * CO.dot(rayDir);
	const float c = CO.dot(CO) - radius2;

	const float discriminant = b * b - 4.0f * a * c;
	if(discriminant < 0.0f)
		return false;

	const float OneOver2A = 1.0f / (2.0f * a);
	const float sqrtDet = sqrtf(discriminant);
	tmin = (-b - sqrtDet) * OneOver2A;
	tmax = (-b + sqrtDet) * OneOver2A;
	if(tmin > tmax)
		PxSwap(tmin, tmax);

	return true;
}

PxU32 Gu::intersectRayCapsuleInternal(const PxVec3& rayOrigin, const PxVec3& rayDir, const PxVec3& capsuleP0, const PxVec3& capsuleP1, float radius, PxReal s[2])
{
	const float radius2 = radius * radius;

	const PxVec3 AB = capsuleP1 - capsuleP0;
	const PxVec3 AO = rayOrigin - capsuleP0;

	const float AB_dot_d = AB.dot(rayDir);
	const float AB_dot_AO = AB.dot(AO);
	const float AB_dot_AB = AB.dot(AB);
	
	const float OneOverABDotAB = AB_dot_AB!=0.0f ? 1.0f / AB_dot_AB : 0.0f;
	const float m = AB_dot_d * OneOverABDotAB;
	const float n = AB_dot_AO * OneOverABDotAB;

	const PxVec3 Q = rayDir - (AB * m);
	const PxVec3 R = AO - (AB * n);

	const float a = Q.dot(Q);
	const float b = 2.0f * Q.dot(R);
	const float c = R.dot(R) - radius2;

	if(a == 0.0f)
	{
		float atmin, atmax, btmin, btmax;
		if(		!intersectRaySphere(rayOrigin, rayDir, capsuleP0, radius2, atmin, atmax)
			||	!intersectRaySphere(rayOrigin, rayDir, capsuleP1, radius2, btmin, btmax))
			return 0;

		s[0] = atmin < btmin ? atmin : btmin;
		return 1;
	}

	const float discriminant = b * b - 4.0f * a * c;
	if(discriminant < 0.0f)
		return 0;

	const float OneOver2A = 1.0f / (2.0f * a);
	const float sqrtDet = sqrtf(discriminant);

	float tmin = (-b - sqrtDet) * OneOver2A;
	float tmax = (-b + sqrtDet) * OneOver2A;
	if(tmin > tmax)
		PxSwap(tmin, tmax);

	const float t_k1 = tmin * m + n;
	if(t_k1 < 0.0f)
	{
		float stmin, stmax;
		if(intersectRaySphere(rayOrigin, rayDir, capsuleP0, radius2, stmin, stmax))
			s[0] = stmin;
		else 
			return 0;
	}
	else if(t_k1 > 1.0f)
	{
		float stmin, stmax;
		if(intersectRaySphere(rayOrigin, rayDir, capsuleP1, radius2, stmin, stmax))
			s[0] = stmin;
		else 
			return 0;
	}
	else
		s[0] = tmin;
	return 1;
}
