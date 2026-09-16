// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "geomutils/PxContactBuffer.h"
#include "GuVecBox.h"
#include "GuVecSphere.h"
#include "GuContactMethodImpl.h"
#include "GuPCMContactGenUtil.h"

using namespace physx;

bool Gu::pcmContactSphereBox(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);

	using namespace aos;
	// Get actual shape data
	const PxSphereGeometry& shapeSphere = checkedCast<PxSphereGeometry>(shape0);
	const PxBoxGeometry& shapeBox = checkedCast<PxBoxGeometry>(shape1);

	//const PsTransformV transf0(transform0);
	const Vec3V sphereOrigin = V3LoadA(&transform0.p.x);
	//const PsTransformV transf1(transform1);

	const QuatV q1 = QuatVLoadA(&transform1.q.x);
	const Vec3V p1 = V3LoadA(&transform1.p.x);

	const FloatV radius = FLoad(shapeSphere.radius);
	
	const PxTransformV transf1(p1, q1);
	
	const FloatV cDist = FLoad(params.mContactDistance);

	const Vec3V boxExtents = V3LoadU(shapeBox.halfExtents);

	//translate sphere center into the box space
	const Vec3V sphereCenter = transf1.transformInv(sphereOrigin);

	const Vec3V nBoxExtents = V3Neg(boxExtents);

	//const FloatV radSq = FMul(radius, radius);

	const FloatV inflatedSum = FAdd(radius, cDist);
	const FloatV sqInflatedSum = FMul(inflatedSum, inflatedSum);

	const Vec3V p = V3Clamp(sphereCenter, nBoxExtents, boxExtents);
	const Vec3V v = V3Sub(sphereCenter, p);
	const FloatV lengthSq = V3Dot(v, v);

	PX_ASSERT(contactBuffer.count < PxContactBuffer::MAX_CONTACTS);

	if(FAllGrtr(sqInflatedSum, lengthSq))//intersect
	{
		//check whether the spherCenter is inside the box
		const BoolV bInsideBox = V3IsGrtrOrEq(boxExtents, V3Abs(sphereCenter));
		// PT: TODO: ??? revisit this, why do we have both BAllEqTTTT and BAllTrue3?
		if(BAllEqTTTT(BAllTrue3(bInsideBox)))//sphere center inside the box
		{
			//Pick directions and sign
			const Vec3V absP = V3Abs(p);
			const Vec3V distToSurface = V3Sub(boxExtents, absP);//dist from embedded center to box surface along 3 dimensions.
			
			const FloatV x = V3GetX(distToSurface);
			const FloatV y = V3GetY(distToSurface);
			const FloatV z = V3GetZ(distToSurface);

			const Vec3V xV = V3Splat(x);
			const Vec3V zV = V3Splat(z);

			//find smallest element of distToSurface
			const BoolV con0 = BAllTrue3(V3IsGrtrOrEq(distToSurface, zV));
			const BoolV con1 = BAllTrue3(V3IsGrtrOrEq(distToSurface, xV));
			const Vec3V sign = V3Sign(p);
	
			const Vec3V tmpX = V3Mul(V3UnitX(), sign);
			const Vec3V tmpY = V3Mul(V3UnitY(), sign);
			const Vec3V tmpZ = V3Mul(V3UnitZ(), sign);
			
			const Vec3V locNorm = V3Sel(con0, tmpZ, V3Sel(con1, tmpX, tmpY));////local coords contact normal
			const FloatV dist = FNeg(FSel(con0, z, FSel(con1, x, y)));

			//separation so far is just the embedding of the center point; we still have to push out all of the radius.
			const Vec3V normal = transf1.rotate(locNorm);
			const FloatV penetration = FSub(dist, radius);
			const Vec3V point = V3Sub(sphereOrigin , V3Scale(normal, dist));

			outputSimplePCMContact(contactBuffer, point, normal, penetration);
		}
		else
		{
			//get the closest point from the center to the box surface
			const FloatV recipLength = FRsqrt(lengthSq);
			const FloatV length = FRecip(recipLength);
			const Vec3V locNorm = V3Scale(v, recipLength);
			const FloatV penetration = FSub(length, radius);
			const Vec3V normal = transf1.rotate(locNorm);
			const Vec3V point = transf1.transform(p);

			outputSimplePCMContact(contactBuffer, point, normal, penetration);
		}
		return true;
	}
	return false;
}

