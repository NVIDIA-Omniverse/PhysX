// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "geomutils/PxContactBuffer.h"
#include "GuContactMethodImpl.h"

using namespace physx;

bool Gu::contactSpherePlane(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);
	PX_UNUSED(shape1);

	// Get actual shape data
	const PxSphereGeometry& shapeSphere = checkedCast<PxSphereGeometry>(shape0);
	//const PxPlaneGeometry& shapePlane = checkedCast<PxPlaneGeometry>(shape1);

	//Sphere in plane space
	const PxVec3 sphere = transform1.transformInv(transform0.p);
	
	//Make sure we have a normalized plane
	//The plane is implicitly n=<1,0,0> d=0 (in plane-space)
	//PX_ASSERT(PxAbs(shape1.mNormal.magnitudeSquared() - 1.0f) < 0.000001f);

	//Separation
	const PxReal separation = sphere.x - shapeSphere.radius;

	if(separation<=params.mContactDistance)
	{
		const PxVec3 normal = transform1.q.getBasisVector0();
		const PxVec3 point  = transform0.p - normal * shapeSphere.radius;
		contactBuffer.contact(point, normal, separation);
		return true;
	}
	return false;
}
