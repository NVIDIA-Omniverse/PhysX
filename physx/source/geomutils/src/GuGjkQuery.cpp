// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "geometry/PxGjkQuery.h"

#include "GuInternal.h"
#include "GuOverlapTests.h"
#include "GuSweepTests.h"
#include "GuRaycastTests.h"
#include "GuBoxConversion.h"
#include "GuTriangleMesh.h"
#include "GuMTD.h"
#include "GuBounds.h"
#include "GuDistancePointSegment.h"
#include "GuConvexMesh.h"
#include "GuDistancePointBox.h"
#include "GuMidphaseInterface.h"
#include "foundation/PxFPU.h"

using namespace physx;
using namespace Gu;

#include "GuGJK.h"
#include "GuGJKPenetration.h"
#include "GuGJKRaycast.h"
#include "GuEPA.h"
#include "geomutils/PxContactBuffer.h"

using namespace aos;

static PX_SUPPORT_INLINE PxVec3 Vec3V_To_PxVec3(const Vec3V& a)
{
	PxVec3 v;
	V3StoreU(a, v);
	return v;
}

static PX_SUPPORT_INLINE PxReal FloatV_To_PxReal(const FloatV& a)
{
	PxF32 f;
	FStore(a, &f);
	return f;
}

struct CustomConvexV : ConvexV
{
	const PxGjkQuery::Support* s;
	PxReal supportScale;

	CustomConvexV(const PxGjkQuery::Support& _s) : ConvexV(Gu::ConvexType::eCUSTOM), s(&_s), supportScale(1.0f)
	{
		setMinMargin(FLoad(0.001f));
		setSweepMargin(FLoad(0.001f));
	}
	PX_SUPPORT_INLINE Vec3V supportPoint(const PxI32 /*index*/) const
	{
		return supportLocal(V3LoadU(PxVec3(1, 0, 0)));
	}
	PX_SUPPORT_INLINE Vec3V supportLocal(const Vec3V& dir) const
	{
		return V3Scale(V3LoadU(s->supportLocal(Vec3V_To_PxVec3(dir))), FLoad(supportScale));
	}
	PX_SUPPORT_INLINE Vec3V supportLocal(const Vec3V& dir, PxI32& index) const
	{
		index = 0;
		return supportLocal(dir);
	}
	PX_SUPPORT_INLINE Vec3V supportRelative(const Vec3V& dir, const PxMatTransformV& aTob, const PxMatTransformV& aTobT) const
	{
		const Vec3V _dir = aTobT.rotate(dir);
		const Vec3V p = supportLocal(_dir);
		return aTob.transform(p);
	}
	PX_SUPPORT_INLINE Vec3V supportRelative(const Vec3V& dir, const PxMatTransformV& aTob, const PxMatTransformV& aTobT, PxI32& index) const
	{
		index = 0;
		return supportRelative(dir, aTob, aTobT);
	}
};

bool PxGjkQuery::proximityInfo(const Support& a, const Support& b, const PxTransform& poseA, const PxTransform& poseB, PxReal contactDistance, PxReal toleranceLength, PxVec3& pointA, PxVec3& pointB, PxVec3& separatingAxis, PxReal& separation)
{
	const PxTransformV transf0 = loadTransformU(poseA);
	const PxTransformV transf1 = loadTransformU(poseB);
	const PxTransformV curRTrans(transf1.transformInv(transf0));
	const PxMatTransformV aToB(curRTrans);
	const PxReal degenerateScale = 0.001f;

	CustomConvexV supportA(a);
	CustomConvexV supportB(b);
	const RelativeConvex<CustomConvexV> convexA(supportA, aToB);
	const LocalConvex<CustomConvexV> convexB(supportB);

	Vec3V initialSearchDir = aToB.p;
	FloatV contactDist = FLoad((a.getMargin() + b.getMargin()) + contactDistance);

	Vec3V aPoints[4];
	Vec3V bPoints[4];
	PxU8 size = 0;
	GjkOutput output;

	GjkStatus status = gjkPenetration(convexA, convexB, initialSearchDir, contactDist, true, aPoints, bPoints, size, output);

	if (status == GJK_DEGENERATE)
	{
		supportA.supportScale = supportB.supportScale = 1.0f - degenerateScale;
		status = gjkPenetration(convexA, convexB, initialSearchDir, contactDist, true, aPoints, bPoints, size, output);
		supportA.supportScale = supportB.supportScale = 1.0f;
	}

	if (status == GJK_CONTACT || status == GJK_DEGENERATE)
	{
		separatingAxis = poseB.rotate(Vec3V_To_PxVec3(output.normal).getNormalized());
		pointA = poseB.transform(Vec3V_To_PxVec3(output.closestA)) - separatingAxis * a.getMargin();
		pointB = poseB.transform(Vec3V_To_PxVec3(output.closestB)) + separatingAxis * b.getMargin();
		separation = (pointA - pointB).dot(separatingAxis);
		return true;
	}

	if (status == EPA_CONTACT)
	{
		status = epaPenetration(convexA, convexB, aPoints, bPoints, size, true, FLoad(toleranceLength), output);

		if (status == EPA_CONTACT || status == EPA_DEGENERATE)
		{
			separatingAxis = poseB.rotate(Vec3V_To_PxVec3(output.normal).getNormalized());
			pointA = poseB.transform(Vec3V_To_PxVec3(output.closestA)) - separatingAxis * a.getMargin();
			pointB = poseB.transform(Vec3V_To_PxVec3(output.closestB)) + separatingAxis * b.getMargin();
			separation = (pointA - pointB).dot(separatingAxis);
			return true;
		}
	}

	return false;
}

struct PointConvexV : ConvexV
{
	Vec3V zero;
	PointConvexV() : ConvexV(Gu::ConvexType::eCUSTOM)
	{
		zero = V3Zero();
		setMinMargin(FLoad(0.001f));
		setSweepMargin(FLoad(0.001f));
	}
	PX_SUPPORT_INLINE Vec3V supportPoint(const PxI32 /*index*/) const
	{
		return zero;
	}
	PX_SUPPORT_INLINE Vec3V supportLocal(const Vec3V& /*dir*/) const
	{
		return zero;
	}
	PX_SUPPORT_INLINE Vec3V supportLocal(const Vec3V& dir, PxI32& index) const
	{
		index = 0;
		return supportLocal(dir);
	}
	PX_SUPPORT_INLINE Vec3V supportRelative(const Vec3V& dir, const PxMatTransformV& aTob, const PxMatTransformV& aTobT) const
	{
		const Vec3V _dir = aTobT.rotate(dir);
		const Vec3V p = supportLocal(_dir);
		return aTob.transform(p);
	}
	PX_SUPPORT_INLINE Vec3V supportRelative(const Vec3V& dir, const PxMatTransformV& aTob, const PxMatTransformV& aTobT, PxI32& index) const
	{
		index = 0;
		return supportRelative(dir, aTob, aTobT);
	}
};

bool PxGjkQuery::raycast(const Support& shape, const PxTransform& pose, const PxVec3& rayStart, const PxVec3& unitDir, PxReal maxDist, PxReal& t, PxVec3& n, PxVec3& p)
{
	const PxTransformV transf0 = loadTransformU(pose);
	const PxTransformV transf1 = PxTransformV(V3LoadU(rayStart));
	const PxTransformV curRTrans(transf1.transformInv(transf0));
	const PxMatTransformV aToB(curRTrans);

	CustomConvexV supportA(shape);
	PointConvexV supportB;
	const RelativeConvex<CustomConvexV> convexA(supportA, aToB);
	const LocalConvex<PointConvexV> convexB(supportB);

	Vec3V initialDir = aToB.p;
	FloatV initialLambda = FLoad(0);
	Vec3V s = V3Zero();
	Vec3V r = V3LoadU(unitDir * maxDist);
	FloatV lambda;
	Vec3V normal, closestA;

	if (gjkRaycast(convexA, convexB, initialDir, initialLambda, s, r, lambda, normal, closestA, shape.getMargin()))
	{
		t = FloatV_To_PxReal(lambda) * maxDist;
		n = -Vec3V_To_PxVec3(normal).getNormalized();
		p = Vec3V_To_PxVec3(closestA) + n * shape.getMargin() + rayStart;
		return true;
	}

	return false;
}

bool PxGjkQuery::overlap(const Support& a, const Support& b, const PxTransform& poseA, const PxTransform& poseB)
{
	const PxTransformV transf0 = loadTransformU(poseA);
	const PxTransformV transf1 = loadTransformU(poseB);
	const PxTransformV curRTrans(transf1.transformInv(transf0));
	const PxMatTransformV aToB(curRTrans);

	CustomConvexV supportA(a);
	CustomConvexV supportB(b);
	const RelativeConvex<CustomConvexV> convexA(supportA, aToB);
	const LocalConvex<CustomConvexV> convexB(supportB);

	Vec3V initialSearchDir = aToB.p;
	FloatV contactDist = FLoad(a.getMargin() + b.getMargin());
	Vec3V closestA, closestB, normal;
	FloatV distance;
	GjkStatus status = gjk(convexA, convexB, initialSearchDir, contactDist, closestA, closestB, normal, distance);

	return status == GJK_CLOSE || status == GJK_CONTACT;
}

bool PxGjkQuery::sweep(const Support& a, const Support& b, const PxTransform& poseA, const PxTransform& poseB, const PxVec3& unitDir, PxReal maxDist, PxReal& t, PxVec3& n, PxVec3& p)
{
	const PxTransformV transf0 = loadTransformU(poseA);
	const PxTransformV transf1 = loadTransformU(poseB);
	const PxTransformV curRTrans(transf1.transformInv(transf0));
	const PxMatTransformV aToB(curRTrans);

	CustomConvexV supportA(a);
	CustomConvexV supportB(b);
	const RelativeConvex<CustomConvexV> convexA(supportA, aToB);
	const LocalConvex<CustomConvexV> convexB(supportB);

	Vec3V initialDir = aToB.p;
	FloatV initialLambda = FLoad(0);
	Vec3V s = V3Zero();
	Vec3V r = V3LoadU(poseB.rotateInv(unitDir * maxDist));
	FloatV lambda;
	Vec3V normal, closestA;

	if (gjkRaycast(convexA, convexB, initialDir, initialLambda, s, r, lambda, normal, closestA, a.getMargin() + b.getMargin()))
	{
		t = FloatV_To_PxReal(lambda) * maxDist;
		n = poseB.rotate(-(Vec3V_To_PxVec3(normal)).getNormalized());
		p = poseB.transform(Vec3V_To_PxVec3(closestA)) + n * a.getMargin();
		return true;
	}

	return false;
}

