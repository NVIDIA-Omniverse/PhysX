// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "geomutils/PxContactBuffer.h"
#include "foundation/PxVecTransform.h"
#include "GuContactMethodImpl.h"
#include "GuPCMContactGenUtil.h"

using namespace physx;

bool Gu::pcmContactSpherePlane(GU_CONTACT_METHOD_ARGS)
{
	using namespace aos;
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);
	PX_UNUSED(shape1);

	// Get actual shape data
	const PxSphereGeometry& shapeSphere = checkedCast<PxSphereGeometry>(shape0);

	//sphere transform
	const Vec3V p0 = V3LoadU_SafeReadW(transform0.p);	// PT: safe because 'mRefCount' follows 'mTransform' in PxsTransform

	//plane transform
	const Vec3V p1 = V3LoadU_SafeReadW(transform1.p);	// PT: safe because 'mRefCount' follows 'mTransform' in PxsTransform
	const QuatV q1 = QuatVLoadU(&transform1.q.x);

	const FloatV radius = FLoad(shapeSphere.radius);
	const FloatV contactDist = FLoad(params.mContactDistance);

	const PxTransformV transf1(p1, q1);
	//Sphere in plane space
	const Vec3V sphereCenterInPlaneSpace = transf1.transformInv(p0);
	
	//Separation
	const FloatV separation = FSub(V3GetX(sphereCenterInPlaneSpace), radius);

	if(FAllGrtrOrEq(contactDist, separation))
	{
		//get the plane normal
		const Vec3V worldNormal = QuatGetBasisVector0(q1);
		const Vec3V worldPoint = V3NegScaleSub(worldNormal, radius, p0);

		return outputSimplePCMContact(contactBuffer, worldPoint, worldNormal, separation);
	}
	return false;
}

