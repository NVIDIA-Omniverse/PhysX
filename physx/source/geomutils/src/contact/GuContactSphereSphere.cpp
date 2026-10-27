// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "geomutils/PxContactBuffer.h"
#include "GuContactMethodImpl.h"

using namespace physx;

bool Gu::contactSphereSphere(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);

	const PxSphereGeometry& sphereGeom0 = checkedCast<PxSphereGeometry>(shape0);
	const PxSphereGeometry& sphereGeom1 = checkedCast<PxSphereGeometry>(shape1);

	PxVec3 delta = transform0.p - transform1.p;

	const PxReal distanceSq = delta.magnitudeSquared();
	const PxReal radiusSum = sphereGeom0.radius + sphereGeom1.radius;
	const PxReal inflatedSum = radiusSum + params.mContactDistance;
	if(distanceSq >= inflatedSum*inflatedSum)
		return false;

	// We do a *manual* normalization to check for singularity condition
	const PxReal magn = PxSqrt(distanceSq);
	if(magn<=0.00001f)
		delta = PxVec3(1.0f, 0.0f, 0.0f);	// PT: spheres are exactly overlapping => can't create normal => pick up random one
	else
		delta *= 1.0f/magn;

	// PT: TODO: why is this formula different from the original code?
	const PxVec3 contact = delta * ((sphereGeom0.radius + magn - sphereGeom1.radius)*-0.5f) + transform0.p;
		
	contactBuffer.contact(contact, delta, magn - radiusSum);
	return true;
}
