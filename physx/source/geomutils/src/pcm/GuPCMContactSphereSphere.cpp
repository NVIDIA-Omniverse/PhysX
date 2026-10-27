// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "geomutils/PxContactBuffer.h"
#include "GuContactMethodImpl.h"
#include "foundation/PxVecTransform.h"
#include "GuPCMContactGenUtil.h"

using namespace physx;

bool Gu::pcmContactSphereSphere(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(cache);
	PX_UNUSED(renderOutput);

	using namespace aos;
	const PxSphereGeometry& shapeSphere0 = checkedCast<PxSphereGeometry>(shape0);
	const PxSphereGeometry& shapeSphere1 = checkedCast<PxSphereGeometry>(shape1);
	
	const FloatV cDist = FLoad(params.mContactDistance);
	const Vec3V p0 = V3LoadA(&transform0.p.x);
	const Vec3V p1 = V3LoadA(&transform1.p.x);

	const FloatV r0	= FLoad(shapeSphere0.radius);
	const FloatV r1	= FLoad(shapeSphere1.radius);
	
	const Vec3V _delta = V3Sub(p0, p1);
	const FloatV distanceSq = V3Dot(_delta, _delta);
	const FloatV radiusSum = FAdd(r0, r1);
	const FloatV inflatedSum = FAdd(radiusSum, cDist);
	
	if(FAllGrtr(FMul(inflatedSum, inflatedSum), distanceSq))
	{
		const FloatV eps = FLoad(0.00001f);
		const FloatV dist = FSqrt(distanceSq);
		const BoolV bCon = FIsGrtrOrEq(eps, dist);
		const Vec3V normal = V3Sel(bCon, V3UnitX(), V3ScaleInv(_delta, dist));
		const Vec3V point = V3ScaleAdd(normal, r1, p1);
		const FloatV pen = FSub(dist, radiusSum);
		
		return outputSimplePCMContact(contactBuffer, point, normal, pen);
	}
	return false;
}
