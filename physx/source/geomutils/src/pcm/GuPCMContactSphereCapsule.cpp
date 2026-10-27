// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "geomutils/PxContactBuffer.h"
#include "GuVecSphere.h"
#include "GuVecCapsule.h"
#include "GuContactMethodImpl.h"
#include "GuPCMContactGenUtil.h"

using namespace physx;
using namespace aos;

static PX_FORCE_INLINE FloatV distancePointSegmentSquared(const Vec3VArg a, const Vec3VArg b, const Vec3VArg p, FloatV& param)
{
	const FloatV zero = FZero();
	const FloatV one = FOne();

	const Vec3V ap = V3Sub(p, a);
	const Vec3V ab = V3Sub(b, a);
	const FloatV nom = V3Dot(ap, ab);
	
	const FloatV denom = V3Dot(ab, ab);
	const FloatV tValue = FClamp(FDiv(nom, denom), zero, one);

	const FloatV t = FSel(FIsEq(denom, zero), zero, tValue);
	const Vec3V v = V3NegScaleSub(ab, t, ap);
	param = t;
	return V3Dot(v, v);
}

bool Gu::pcmContactSphereCapsule(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(cache);
	PX_UNUSED(renderOutput);

	const PxSphereGeometry& shapeSphere = checkedCast<PxSphereGeometry>(shape0);
	const PxCapsuleGeometry& shapeCapsule = checkedCast<PxCapsuleGeometry>(shape1);

	//Sphere in world space
	const Vec3V sphereCenter = V3LoadA(&transform0.p.x);
	const QuatV q1 = QuatVLoadA(&transform1.q.x);
	const Vec3V p1 = V3LoadA(&transform1.p.x);

	const FloatV sphereRadius = FLoad(shapeSphere.radius);
	const FloatV capsuleRadius = FLoad(shapeCapsule.radius);
	const FloatV cDist = FLoad(params.mContactDistance);

	//const FloatV r0 = FloatV_From_F32(shapeCapsule.radius);
	const FloatV halfHeight0 = FLoad(shapeCapsule.halfHeight);
	const Vec3V basisVector0 = QuatGetBasisVector0(q1);
	const Vec3V tmp0 = V3Scale(basisVector0, halfHeight0);
	const Vec3V s = V3Add(p1, tmp0);
	const Vec3V e = V3Sub(p1, tmp0);

	const FloatV radiusSum = FAdd(sphereRadius, capsuleRadius);
	const FloatV inflatedSum = FAdd(radiusSum, cDist);

	// Collision detection
	FloatV t;
	const FloatV squareDist = distancePointSegmentSquared(s, e, sphereCenter, t);
	const FloatV sqInflatedSum = FMul(inflatedSum, inflatedSum);

	if(FAllGrtr(sqInflatedSum, squareDist))//BAllEq(con, bTrue))
	{
		const Vec3V p = V3ScaleAdd(V3Sub(e, s), t, s);
		const Vec3V dir = V3Sub(sphereCenter, p);
		const Vec3V normal = V3NormalizeSafe(dir, V3UnitX());
		const Vec3V point = V3NegScaleSub(normal, sphereRadius, sphereCenter);//transform back to the world space

		const FloatV dist = FSub(FSqrt(squareDist), radiusSum);
		//context.mContactBuffer.contact(point, normal, FSub(FSqrt(squareDist), radiusSum));

		return outputSimplePCMContact(contactBuffer, point, normal, dist);
	}
	return false;
}

