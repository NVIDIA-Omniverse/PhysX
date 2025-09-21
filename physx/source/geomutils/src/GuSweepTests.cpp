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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "geometry/PxSphereGeometry.h"
#include "geometry/PxCustomGeometry.h"
#include "geometry/PxConvexCoreGeometry.h"
#include "geometry/PxHeightFieldGeometry.h"
#include "GuSweepTests.h"
#include "GuVecCapsule.h"
#include "GuVecBox.h"
#include "GuVecTriangle.h"
#include "GuSweepTriangleUtils.h"
#include "GuInternal.h"
#include "GuGJKRaycast.h"
#include "GuConvexGeometry.h"
#include "GuConvexSupport.h"
#include "GuMidphaseInterface.h"
#include "geometry/PxGjkQuery.h"
#include "GuHeightField.h"
#include "GuEntityReport.h"
#include "GuHeightFieldUtil.h"

using namespace physx;
using namespace Gu;
using namespace Cm;
using namespace aos;

//#define USE_VIRTUAL_GJK
#ifdef USE_VIRTUAL_GJK
static bool virtualGjkRaycastPenetration(const GjkConvex& a, const GjkConvex& b, const aos::Vec3VArg initialDir, const aos::FloatVArg initialLambda, const aos::Vec3VArg s, const aos::Vec3VArg r, aos::FloatV& lambda, 
		aos::Vec3V& normal, aos::Vec3V& closestA, const PxReal _inflation, const bool initialOverlap)
{
	return gjkRaycastPenetration<GjkConvex, GjkConvex >(a, b, initialDir, initialLambda, s, r, lambda, normal, closestA, _inflation, initialOverlap);
}
#endif

bool sweepCapsule_BoxGeom(GU_CAPSULE_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(hitFlags);
	PX_UNUSED(threadContext);

	using namespace aos;
	PX_ASSERT(geom.getType() == PxGeometryType::eBOX);
	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);

	const FloatV zero = FZero();
	const Vec3V zeroV = V3Zero();
	const Vec3V boxExtents0 = V3LoadU(boxGeom.halfExtents);
	const FloatV dist = FLoad(distance);
	const Vec3V worldDir = V3LoadU(unitDir);

	const PxTransformV capPos = loadTransformU(capsulePose_);
	const PxTransformV boxPos = loadTransformU(pose);

	const PxMatTransformV aToB(boxPos.transformInv(capPos));

	const FloatV capsuleHalfHeight = FLoad(capsuleGeom_.halfHeight);
	const FloatV capsuleRadius = FLoad(lss.radius);

	BoxV box(zeroV, boxExtents0);
	CapsuleV capsule(aToB.p, aToB.rotate(V3Scale(V3UnitX(), capsuleHalfHeight)), capsuleRadius);

	const Vec3V dir = boxPos.rotateInv(V3Neg(V3Scale(worldDir, dist)));

	const bool isMtd = hitFlags & PxHitFlag::eMTD;
	FloatV toi = FMax();
	Vec3V closestA, normal;//closestA and normal is in the local space of box
	const LocalConvex<CapsuleV> convexA(capsule);
	const LocalConvex<BoxV> convexB(box);
	const Vec3V initialSearchDir = V3Sub(capsule.getCenter(), box.getCenter());
#ifdef USE_VIRTUAL_GJK
	if(!virtualGjkRaycastPenetration(convexA, convexB, initialSearchDir, zero, zeroV, dir, toi, normal, closestA, lss.radius + inflation, isMtd))
		return false;
#else
	if(!gjkRaycastPenetration<LocalConvex<CapsuleV>, LocalConvex<BoxV> >(convexA, convexB, initialSearchDir, zero, zeroV, dir, toi, normal, closestA, lss.radius + inflation, isMtd))
		return false;
#endif
	sweepHit.flags = PxHitFlag::eNORMAL;

	if(FAllGrtrOrEq(zero, toi))
	{
		//initial overlap
		if(isMtd)
		{
			sweepHit.flags |= PxHitFlag::ePOSITION;
			const Vec3V worldPointA = boxPos.transform(closestA);
			const Vec3V destNormal = boxPos.rotate(normal);
			const FloatV length = toi;
			const Vec3V destWorldPointA = V3NegScaleSub(destNormal, length, worldPointA);
			V3StoreU(destWorldPointA, sweepHit.position);
			V3StoreU(destNormal, sweepHit.normal);
			FStore(length, &sweepHit.distance);
		}
		else
		{
			sweepHit.distance	= 0.0f;
			sweepHit.normal		= -unitDir;
		}
	}
	else
	{
		sweepHit.flags |= PxHitFlag::ePOSITION;
		const Vec3V worldPointA = boxPos.transform(closestA);
		const Vec3V destNormal = boxPos.rotate(normal);
		const FloatV length = FMul(dist, toi);
		const Vec3V destWorldPointA = V3ScaleAdd(worldDir, length, worldPointA);
		V3StoreU(destNormal, sweepHit.normal);
		V3StoreU(destWorldPointA, sweepHit.position);
		FStore(length, &sweepHit.distance);
	}
	return true;
}

bool sweepBox_SphereGeom(GU_BOX_SWEEP_FUNC_PARAMS)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eSPHERE);
	PX_UNUSED(threadContext);
	PX_UNUSED(hitFlags);
	PX_UNUSED(boxGeom_);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom);

	const FloatV zero = FZero();
	const Vec3V zeroV = V3Zero();
	const Vec3V boxExtents = V3LoadU(box.extents);
	const FloatV worldDist = FLoad(distance);
	const Vec3V  unitDirV = V3LoadU(unitDir);

	const FloatV sphereRadius = FLoad(sphereGeom.radius);

	const PxTransformV spherePos = loadTransformU(pose);
	const PxTransformV boxPos = loadTransformU(boxPose_);

	const PxMatTransformV aToB(boxPos.transformInv(spherePos));

	const BoxV boxV(zeroV, boxExtents);
	const CapsuleV capsuleV(aToB.p, sphereRadius);

	//transform into b space
	const Vec3V dir = boxPos.rotateInv(V3Scale(unitDirV, worldDist));

	const bool isMtd = hitFlags & PxHitFlag::eMTD;
	FloatV toi;
	Vec3V closestA, normal;//closestA and normal is in the local space of box
	const Vec3V initialSearchDir = V3Sub(capsuleV.getCenter(), boxV.getCenter());
	const LocalConvex<CapsuleV> convexA(capsuleV);
	const LocalConvex<BoxV> convexB(boxV);
#ifdef USE_VIRTUAL_GJK
	if(!virtualGjkRaycastPenetration(convexA, convexB, initialSearchDir, zero, zeroV, dir, toi, normal, closestA, sphereGeom.radius+inflation, isMtd))
		return false;
#else
	if(!gjkRaycastPenetration<LocalConvex<CapsuleV>, LocalConvex<BoxV> >(convexA, convexB, initialSearchDir, zero, zeroV, dir, toi, normal, closestA, sphereGeom.radius+inflation, isMtd))
		return false;
#endif
	sweepHit.flags = PxHitFlag::eNORMAL;

	//initial overlap
	if(FAllGrtrOrEq(zero, toi))
	{
		if(isMtd)
		{
			sweepHit.flags |= PxHitFlag::ePOSITION;
			const Vec3V destWorldPointA = boxPos.transform(closestA);
			const Vec3V destNormal = V3Neg(boxPos.rotate(normal));
			const FloatV length = toi;
			V3StoreU(destNormal, sweepHit.normal);
			V3StoreU(destWorldPointA, sweepHit.position);
			FStore(length, &sweepHit.distance);
		}
		else
		{
			sweepHit.distance	= 0.0f;
			sweepHit.normal		= -unitDir;
		}
	}
	else
	{
		sweepHit.flags |= PxHitFlag::ePOSITION;
		const Vec3V destWorldPointA = boxPos.transform(closestA);
		const Vec3V destNormal = V3Neg(boxPos.rotate(normal));
		const FloatV length = FMul(worldDist, toi);
		V3StoreU(destNormal, sweepHit.normal);
		V3StoreU(destWorldPointA, sweepHit.position);
		FStore(length, &sweepHit.distance);
	}
	return true;
}

bool sweepBox_CapsuleGeom(GU_BOX_SWEEP_FUNC_PARAMS)
{
	using namespace aos;
	PX_ASSERT(geom.getType() == PxGeometryType::eCAPSULE);
	PX_UNUSED(threadContext);
	PX_UNUSED(hitFlags);
	PX_UNUSED(boxGeom_);

	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom);

	const FloatV capsuleHalfHeight = FLoad(capsuleGeom.halfHeight);
	const FloatV capsuleRadius = FLoad(capsuleGeom.radius);

	const FloatV zero = FZero();
	const Vec3V zeroV = V3Zero();
	const Vec3V boxExtents = V3LoadU(box.extents);
	const FloatV worldDist = FLoad(distance);
	const Vec3V  unitDirV = V3LoadU(unitDir);

	const PxTransformV capPos = loadTransformU(pose);
	const PxTransformV boxPos = loadTransformU(boxPose_);

	const PxMatTransformV aToB(boxPos.transformInv(capPos));

	const BoxV boxV(zeroV, boxExtents);
	const CapsuleV capsuleV(aToB.p, aToB.rotate(V3Scale(V3UnitX(), capsuleHalfHeight)), capsuleRadius);

	//transform into b space
	const Vec3V dir = boxPos.rotateInv(V3Scale(unitDirV, worldDist));

	const bool isMtd = hitFlags & PxHitFlag::eMTD;
	FloatV toi;
	Vec3V closestA, normal;//closestA and normal is in the local space of box
	const Vec3V initialSearchDir = V3Sub(capsuleV.getCenter(), boxV.getCenter());
	const LocalConvex<CapsuleV> convexA(capsuleV);
	const LocalConvex<BoxV> convexB(boxV);
#ifdef USE_VIRTUAL_GJK
	if(!virtualGjkRaycastPenetration(convexA, convexB, initialSearchDir, zero, zeroV, dir, toi, normal, closestA, capsuleGeom.radius+inflation, isMtd))
		return false;
#else
	if(!gjkRaycastPenetration<LocalConvex<CapsuleV>, LocalConvex<BoxV> >(convexA, convexB, initialSearchDir, zero, zeroV, dir, toi, normal, closestA, capsuleGeom.radius+inflation, isMtd))
		return false;
#endif
	sweepHit.flags = PxHitFlag::eNORMAL;

	//initial overlap
	if(FAllGrtrOrEq(zero, toi))
	{
		if(isMtd)
		{
			sweepHit.flags |= PxHitFlag::ePOSITION;
			//initial overlap is toi < 0 
			const FloatV length = toi;
			const Vec3V destWorldPointA = boxPos.transform(closestA);
			const Vec3V destNormal = boxPos.rotate(normal);
			V3StoreU(V3Neg(destNormal), sweepHit.normal);
			V3StoreU(destWorldPointA, sweepHit.position);
			FStore(length, &sweepHit.distance);
		}
		else
		{
			sweepHit.distance	= 0.0f;
			sweepHit.normal		= -unitDir;
		}
		return true;
	}
	else
	{
		sweepHit.flags |= PxHitFlag::ePOSITION;
		const Vec3V destWorldPointA = boxPos.transform(closestA);
		const Vec3V destNormal = boxPos.rotate(normal);
		const FloatV length = FMul(worldDist, toi);
		V3StoreU(V3Neg(destNormal), sweepHit.normal);
		V3StoreU(destWorldPointA, sweepHit.position);
		FStore(length, &sweepHit.distance);
	}
	return true;	
}

bool sweepBox_BoxGeom(GU_BOX_SWEEP_FUNC_PARAMS)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eBOX);
	PX_UNUSED(threadContext);
	PX_UNUSED(boxGeom_);

	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);

	const FloatV zero = FZero();
	const Vec3V zeroV = V3Zero();
	const Vec3V boxExtents0 = V3LoadU(boxGeom.halfExtents);
	const Vec3V boxExtents1 = V3LoadU(box.extents);
	const FloatV worldDist = FLoad(distance);
	const Vec3V  unitDirV = V3LoadU(unitDir);

	const PxTransformV boxTrans0 = loadTransformU(pose);
	const PxTransformV boxTrans1 = loadTransformU(boxPose_);

	const PxMatTransformV aToB(boxTrans1.transformInv(boxTrans0));

	const BoxV box0(zeroV, boxExtents0);
	const BoxV box1(zeroV, boxExtents1);

	//transform into b space
	const Vec3V dir = boxTrans1.rotateInv(V3Scale(unitDirV, worldDist));
	const bool isMtd = hitFlags & PxHitFlag::eMTD;
	FloatV toi;
	Vec3V closestA, normal;//closestA and normal is in the local space of box
	const RelativeConvex<BoxV> convexA(box0, aToB);
	const LocalConvex<BoxV> convexB(box1);
#ifdef USE_VIRTUAL_GJK
	if(!virtualGjkRaycastPenetration(convexA, convexB, aToB.p, zero, zeroV, dir, toi, normal, closestA, inflation, isMtd))
		return false;
#else
	if(!gjkRaycastPenetration<RelativeConvex<BoxV>, LocalConvex<BoxV> >(convexA, convexB, aToB.p, zero, zeroV, dir, toi, normal, closestA, inflation, isMtd))
		return false;
#endif	
	sweepHit.flags = PxHitFlag::eNORMAL;
	if(FAllGrtrOrEq(zero, toi))
	{
		if(isMtd)
		{
			sweepHit.flags |= PxHitFlag::ePOSITION;
			const FloatV length = toi;
			const Vec3V destWorldPointA = boxTrans1.transform(closestA);
			const Vec3V destNormal = V3Normalize(boxTrans1.rotate(normal));
			V3StoreU(V3Neg(destNormal), sweepHit.normal);
			V3StoreU(destWorldPointA, sweepHit.position);
			FStore(length, &sweepHit.distance);
		}
		else
		{
			sweepHit.distance	= 0.0f;
			sweepHit.normal		= -unitDir;
		}
	}
	else
	{
		sweepHit.flags |= PxHitFlag::ePOSITION;
		const Vec3V destWorldPointA = boxTrans1.transform(closestA);
		const Vec3V destNormal = V3Normalize(boxTrans1.rotate(normal));
		const FloatV length = FMul(worldDist, toi);
		V3StoreU(V3Neg(destNormal), sweepHit.normal);
		V3StoreU(destWorldPointA, sweepHit.position);
		FStore(length, &sweepHit.distance);
	}
	return true;
}

bool Gu::sweepBoxTriangles(GU_SWEEP_TRIANGLES_FUNC_PARAMS(PxBoxGeometry))
{
	PX_UNUSED(hitFlags);

	if(!nbTris)
		return false;

	const bool meshBothSides = hitFlags & PxHitFlag::eMESH_BOTH_SIDES;
	const bool doBackfaceCulling = !doubleSided && !meshBothSides;

	Box box;
	buildFrom(box, pose.p, geom.halfExtents, pose.q);

	PxGeomSweepHit sweepHit;
	// Move to AABB space
	PxMat34 worldToBox;
	computeWorldToBoxMatrix(worldToBox, box);

	const PxVec3 localDir = worldToBox.rotate(unitDir);
	const PxVec3 localMotion = localDir * distance;

	const Vec3V base0 = V3LoadU(worldToBox.m.column0);
	const Vec3V base1 = V3LoadU(worldToBox.m.column1);
	const Vec3V base2 = V3LoadU(worldToBox.m.column2);
	const Mat33V matV(base0, base1, base2);
	const Vec3V p	  = V3LoadU(worldToBox.p);
	const PxMatTransformV worldToBoxV(p, matV);

	const FloatV zero = FZero();
	const Vec3V zeroV = V3Zero();
	const Vec3V boxExtents = V3LoadU(box.extents);
	const Vec3V boxDir = V3LoadU(localDir);
	const FloatV inflationV = FLoad(inflation);
	const Vec3V absBoxDir = V3Abs(boxDir);
	const FloatV boxRadiusV = FAdd(V3Dot(absBoxDir, boxExtents), inflationV);
	BoxV boxV(zeroV, boxExtents);

#if PX_DEBUG
	PxU32 totalTestsExpected = nbTris;
	PxU32 totalTestsReal = 0;
	PX_UNUSED(totalTestsExpected);
	PX_UNUSED(totalTestsReal);
#endif

	Vec3V boxLocalMotion = V3LoadU(localMotion);
	Vec3V minClosestA = zeroV, minNormal = zeroV;
	PxU32 minTriangleIndex = 0;
	PxVec3 bestTriNormal(0.0f);
	FloatV dist = FLoad(distance);

	const PxTransformV boxPos = loadTransformU(pose);

	bool status = false;

	const PxU32 idx = cachedIndex ? *cachedIndex : 0;

	for(PxU32 ii=0;ii<nbTris;ii++)
	{
		const PxU32 triangleIndex = getTriangleIndex(ii, idx);

		const Vec3V localV0 = V3LoadU(triangles[triangleIndex].verts[0]);
		const Vec3V localV1 = V3LoadU(triangles[triangleIndex].verts[1]);
		const Vec3V localV2 = V3LoadU(triangles[triangleIndex].verts[2]);

		const Vec3V triV0 = worldToBoxV.transform(localV0);
		const Vec3V triV1 = worldToBoxV.transform(localV1);
		const Vec3V triV2 = worldToBoxV.transform(localV2);

		const Vec3V triNormal = V3Cross(V3Sub(triV2, triV1),V3Sub(triV0, triV1)); 

		if(doBackfaceCulling && FAllGrtrOrEq(V3Dot(triNormal, boxLocalMotion), zero)) // backface culling
			continue;

		const FloatV dp0 = V3Dot(triV0, boxDir);
		const FloatV dp1 = V3Dot(triV1, boxDir);
		const FloatV dp2 = V3Dot(triV2, boxDir);
		
		const FloatV dp = FMin(dp0, FMin(dp1, dp2));

		const Vec3V dpV = V3Merge(dp0, dp1, dp2);

		const FloatV temp1 = FAdd(boxRadiusV, dist);
		const BoolV con0 = FIsGrtr(dp, temp1);
		const BoolV con1 = V3IsGrtr(zeroV, dpV);

		if(BAllEqTTTT(BOr(con0, con1)))
			continue;

#if PX_DEBUG
		totalTestsReal++;
#endif

		TriangleV triangleV(triV0, triV1, triV2);
		
		FloatV lambda;   
		Vec3V closestA, normal;//closestA and normal is in the local space of convex hull
		const LocalConvex<TriangleV> convexA(triangleV);
		const LocalConvex<BoxV> convexB(boxV);
		const Vec3V initialSearchDir = V3Sub(triangleV.getCenter(), boxV.getCenter());
#ifdef USE_VIRTUAL_GJK
		if(virtualGjkRaycastPenetration(convexA, convexB, initialSearchDir, zero, zeroV, boxLocalMotion, lambda, normal, closestA, inflation, false))
#else
		if(gjkRaycastPenetration<LocalConvex<TriangleV>, LocalConvex<BoxV> >(convexA, convexB, initialSearchDir, zero, zeroV, boxLocalMotion, lambda, normal, closestA, inflation, false))
#endif
		{
			//hitCount++;
		
			if(FAllGrtrOrEq(zero, lambda))
			{
				hit.distance	= 0.0f;
				hit.faceIndex	= triangleIndex;
				hit.normal		= -unitDir;
				hit.flags		= PxHitFlag::eNORMAL;
				return true;
			}

			dist = FMul(dist, lambda);
			boxLocalMotion = V3Scale(boxDir, dist);  
			minClosestA = closestA;
			minNormal = normal;
			minTriangleIndex = triangleIndex;
			V3StoreU(triNormal, bestTriNormal);
			status = true;
			if(hitFlags & PxHitFlag::eANY_HIT)
				break;
		}
	}

	if(!status)
		return false;

	hit.faceIndex	= minTriangleIndex;
	const Vec3V destNormal = V3Neg(V3Normalize(boxPos.rotate(minNormal)));
	const Vec3V destWorldPointA = boxPos.transform(minClosestA);
	V3StoreU(destNormal, hit.normal);
	V3StoreU(destWorldPointA, hit.position);
	FStore(dist, &hit.distance);

	// PT: by design, returned normal is opposed to the sweep direction.
	if(shouldFlipNormal(hit.normal, meshBothSides, doubleSided, bestTriNormal, unitDir))
		hit.normal = -hit.normal;

	hit.flags = PxHitFlag::ePOSITION|PxHitFlag::eNORMAL;
	return true;
}

bool sweepCapsule_SphereGeom		(GU_CAPSULE_SWEEP_FUNC_PARAMS);
bool sweepCapsule_PlaneGeom			(GU_CAPSULE_SWEEP_FUNC_PARAMS);
bool sweepCapsule_CapsuleGeom		(GU_CAPSULE_SWEEP_FUNC_PARAMS);
bool sweepCapsule_BoxGeom			(GU_CAPSULE_SWEEP_FUNC_PARAMS);
bool sweepCapsule_BoxGeom_Precise	(GU_CAPSULE_SWEEP_FUNC_PARAMS);
bool sweepCapsule_ConvexGeom		(GU_CAPSULE_SWEEP_FUNC_PARAMS);
bool sweepCapsule_MeshGeom			(GU_CAPSULE_SWEEP_FUNC_PARAMS);
bool sweepCapsule_HeightFieldGeom	(GU_CAPSULE_SWEEP_FUNC_PARAMS);

bool sweepBox_SphereGeom			(GU_BOX_SWEEP_FUNC_PARAMS);
bool sweepBox_SphereGeom_Precise	(GU_BOX_SWEEP_FUNC_PARAMS);
bool sweepBox_PlaneGeom				(GU_BOX_SWEEP_FUNC_PARAMS);
bool sweepBox_CapsuleGeom			(GU_BOX_SWEEP_FUNC_PARAMS);
bool sweepBox_CapsuleGeom_Precise	(GU_BOX_SWEEP_FUNC_PARAMS);
bool sweepBox_BoxGeom				(GU_BOX_SWEEP_FUNC_PARAMS);
bool sweepBox_BoxGeom_Precise		(GU_BOX_SWEEP_FUNC_PARAMS);
bool sweepBox_ConvexGeom			(GU_BOX_SWEEP_FUNC_PARAMS);
bool sweepBox_MeshGeom				(GU_BOX_SWEEP_FUNC_PARAMS);
bool sweepBox_HeightFieldGeom		(GU_BOX_SWEEP_FUNC_PARAMS);
bool sweepBox_HeightFieldGeom_Precise(GU_BOX_SWEEP_FUNC_PARAMS);

bool sweepConvex_SphereGeom			(GU_CONVEX_SWEEP_FUNC_PARAMS);
bool sweepConvex_PlaneGeom			(GU_CONVEX_SWEEP_FUNC_PARAMS);
bool sweepConvex_CapsuleGeom		(GU_CONVEX_SWEEP_FUNC_PARAMS);
bool sweepConvex_BoxGeom			(GU_CONVEX_SWEEP_FUNC_PARAMS);
bool sweepConvex_ConvexCoreGeom		(GU_CONVEX_SWEEP_FUNC_PARAMS);
bool sweepConvex_ConvexGeom			(GU_CONVEX_SWEEP_FUNC_PARAMS);
bool sweepConvex_MeshGeom			(GU_CONVEX_SWEEP_FUNC_PARAMS);
bool sweepConvex_HeightFieldGeom	(GU_CONVEX_SWEEP_FUNC_PARAMS);

static bool sweepCapsule_InvalidGeom(GU_CAPSULE_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(threadContext);
	PX_UNUSED(capsuleGeom_);
	PX_UNUSED(capsulePose_);
	PX_UNUSED(geom);
	PX_UNUSED(pose);
	PX_UNUSED(lss);
	PX_UNUSED(unitDir);
	PX_UNUSED(distance);
	PX_UNUSED(sweepHit);
	PX_UNUSED(hitFlags);
	PX_UNUSED(inflation);
	return false;
}

static bool sweepBox_InvalidGeom(GU_BOX_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(threadContext);
	PX_UNUSED(boxPose_);
	PX_UNUSED(boxGeom_);
	PX_UNUSED(geom);
	PX_UNUSED(pose);
	PX_UNUSED(box);
	PX_UNUSED(unitDir);
	PX_UNUSED(distance);
	PX_UNUSED(sweepHit);
	PX_UNUSED(hitFlags);
	PX_UNUSED(inflation);
	return false;
}

static bool sweepConvexCore_InvalidGeom(GU_CONVEXCORE_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(threadContext);
	PX_UNUSED(geom);
	PX_UNUSED(pose);
	PX_UNUSED(convexGeom);
	PX_UNUSED(convexPose);
	PX_UNUSED(unitDir);
	PX_UNUSED(distance);
	PX_UNUSED(sweepHit);
	PX_UNUSED(hitFlags);
	PX_UNUSED(inflation);
	return false;
}

static bool sweepConvex_InvalidGeom(GU_CONVEX_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(threadContext);
	PX_UNUSED(geom);
	PX_UNUSED(pose);
	PX_UNUSED(convexGeom);
	PX_UNUSED(convexPose);
	PX_UNUSED(unitDir);
	PX_UNUSED(distance);
	PX_UNUSED(sweepHit);
	PX_UNUSED(hitFlags);
	PX_UNUSED(inflation);
	return false;
}

static bool sweepCapsule_CustomGeom(GU_CAPSULE_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(lss);
	if(geom.getType() == PxGeometryType::eCUSTOM)
		return static_cast<const PxCustomGeometry&>(geom).callbacks->sweep(unitDir, distance, geom, pose, capsuleGeom_, capsulePose_, sweepHit, hitFlags, inflation, threadContext);
	return false;
}

static bool sweepBox_CustomGeom(GU_BOX_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(box);
	if(geom.getType() == PxGeometryType::eCUSTOM)
		return static_cast<const PxCustomGeometry&>(geom).callbacks->sweep(unitDir, distance, geom, pose, boxGeom_, boxPose_, sweepHit, hitFlags, inflation, threadContext);
	return false;
}

static bool sweepConvexCore_CustomGeom(GU_CONVEXCORE_SWEEP_FUNC_PARAMS)
{
	if (geom.getType() == PxGeometryType::eCUSTOM)
		return static_cast<const PxCustomGeometry&>(geom).callbacks->sweep(unitDir, distance, geom, pose, convexGeom, convexPose, sweepHit, hitFlags, inflation, threadContext);
	return false;
}

static bool sweepConvex_CustomGeom(GU_CONVEX_SWEEP_FUNC_PARAMS)
{
	if(geom.getType() == PxGeometryType::eCUSTOM)
		return static_cast<const PxCustomGeometry&>(geom).callbacks->sweep(unitDir, distance, geom, pose, convexGeom, convexPose, sweepHit, hitFlags, inflation, threadContext);
	return false;
}

static bool sweepCapsule_ConvexCoreGeom(GU_CAPSULE_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(threadContext);
	PX_UNUSED(capsuleGeom_);
	PX_UNUSED(capsulePose_);
	PX_UNUSED(geom);
	PX_UNUSED(pose);
	PX_UNUSED(lss);
	PX_UNUSED(unitDir);
	PX_UNUSED(distance);
	PX_UNUSED(sweepHit);
	PX_UNUSED(hitFlags);
	PX_UNUSED(inflation);

	struct CapsuleSupport : PxGjkQuery::Support
	{
		PxCapsuleGeometry capsule;
		CapsuleSupport(const PxCapsuleGeometry& c)
			: capsule(c) {}
		virtual PxReal getMargin() const
			{ return capsule.radius; }
		virtual PxVec3 supportLocal(const PxVec3& dir) const
			{ return PxVec3(PxSign(dir.x) * capsule.halfHeight, 0, 0); }
	};

	struct ConvexCoreSupport : PxGjkQuery::Support
	{
		Gu::ConvexShape shape;
		ConvexCoreSupport(const PxConvexCoreGeometry& g)
			{ Gu::makeConvexShape(g, PxTransform(PxIdentity), shape); }
		virtual PxReal getMargin() const
			{ return shape.margin; }
		virtual PxVec3 supportLocal(const PxVec3& dir) const
			{ return shape.supportLocal(dir); }
	};

	const PxConvexCoreGeometry& convex = static_cast<const PxConvexCoreGeometry&>(geom);
	if (convex.isValid())
	{
		PxBounds3 bounds = Gu::computeBounds(convex, pose);
		bounds.include(Gu::computeBounds(capsuleGeom_, capsulePose_));
		PxReal wiseDist = PxMin(distance, bounds.getDimensions().magnitude());
		PxReal t;
		PxVec3 n, p;
		if (PxGjkQuery::sweep(ConvexCoreSupport(convex), CapsuleSupport(capsuleGeom_), pose, capsulePose_, unitDir, wiseDist, t, n, p))
		{
			PxGeomSweepHit& hit = sweepHit;
			hit.distance = t;
			hit.position = p;
			hit.normal = n;
			hit.flags |= PxHitFlag::ePOSITION | PxHitFlag::eNORMAL;
			return 1;
		}
	}

	return 0;
}

static bool sweepBox_ConvexCoreGeom(GU_BOX_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(threadContext);
	PX_UNUSED(boxPose_);
	PX_UNUSED(boxGeom_);
	PX_UNUSED(geom);
	PX_UNUSED(pose);
	PX_UNUSED(box);
	PX_UNUSED(unitDir);
	PX_UNUSED(distance);
	PX_UNUSED(sweepHit);
	PX_UNUSED(hitFlags);
	PX_UNUSED(inflation);

	struct BoxSupport : PxGjkQuery::Support
	{
		PxBoxGeometry box;
		BoxSupport(const PxBoxGeometry& b)
			: box(b) {}
		virtual PxReal getMargin() const
			{ return 0; }
		virtual PxVec3 supportLocal(const PxVec3& dir) const
			{ return PxVec3(PxSign(dir.x) * box.halfExtents.x,
				PxSign(dir.y) * box.halfExtents.y, PxSign(dir.z) * box.halfExtents.z); }
	};

	struct ConvexCoreSupport : PxGjkQuery::Support
	{
		Gu::ConvexShape shape;
		ConvexCoreSupport(const PxConvexCoreGeometry& g)
			{ Gu::makeConvexShape(g, PxTransform(PxIdentity), shape); }
		virtual PxReal getMargin() const
			{ return shape.margin; }
		virtual PxVec3 supportLocal(const PxVec3& dir) const
			{ return shape.supportLocal(dir); }
	};

	const PxConvexCoreGeometry& convex = static_cast<const PxConvexCoreGeometry&>(geom);
	if (convex.isValid())
	{
		PxBounds3 bounds = Gu::computeBounds(convex, pose);
		bounds.include(Gu::computeBounds(boxGeom_, boxPose_));
		PxReal wiseDist = PxMin(distance, bounds.getDimensions().magnitude());
		PxReal t;
		PxVec3 n, p;
		if (PxGjkQuery::sweep(ConvexCoreSupport(convex), BoxSupport(boxGeom_), pose, boxPose_, unitDir, wiseDist, t, n, p))
		{
			PxGeomSweepHit& hit = sweepHit;
			hit.distance = t;
			hit.position = p;
			hit.normal = n;
			hit.flags |= PxHitFlag::ePOSITION | PxHitFlag::eNORMAL;
			return 1;
		}
	}

	return 0;
}

static bool sweepConvexCore_Plane(GU_CONVEXCORE_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(threadContext);
	PX_UNUSED(geom);
	PX_UNUSED(pose);
	PX_UNUSED(convexGeom);
	PX_UNUSED(convexPose);
	PX_UNUSED(unitDir);
	PX_UNUSED(distance);
	PX_UNUSED(sweepHit);
	PX_UNUSED(hitFlags);
	PX_UNUSED(inflation);

	PxPlane plane = getPlane(pose);

	Gu::ConvexShape shape;
	Gu::makeConvexShape(convexGeom, convexPose, shape);
	shape.margin += inflation;

	PxVec3 closestPoint = shape.support(-plane.n);
	PxReal closestDist = plane.distance(closestPoint);

	sweepHit.faceIndex = 0xffffffff;

	if (closestDist <= 0)
	{
		if (hitFlags & PxHitFlag::eMTD)
		{
			sweepHit.flags = PxHitFlag::eNORMAL | PxHitFlag::ePOSITION;
			sweepHit.distance = closestDist;
			sweepHit.normal = plane.n;
			sweepHit.position = closestPoint + plane.n * closestDist;
			return true;
		}
		else if (!(hitFlags & PxHitFlag::eASSUME_NO_INITIAL_OVERLAP))
		{
			sweepHit.flags = PxHitFlag::eNORMAL;
			sweepHit.distance = 0.0f;
			sweepHit.normal = -unitDir;
			return true;
		}
	}

	PxReal dirDotNorm = unitDir.dot(plane.n);

	const PxReal dotEps = 1e-5f;
	if (dirDotNorm < -dotEps)
	{
		PxReal hitDistance = closestDist / -dirDotNorm;
		if (hitDistance < distance)
		{
			sweepHit.flags = PxHitFlag::eNORMAL | PxHitFlag::ePOSITION;
			sweepHit.distance = hitDistance;
			sweepHit.normal = plane.n;
			sweepHit.position = closestPoint + unitDir * hitDistance;
			return true;
		}
	}

	return false;
}

static bool sweepConvexCore_Convex(GU_CONVEXCORE_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(threadContext);
	PX_UNUSED(geom);
	PX_UNUSED(pose);
	PX_UNUSED(convexGeom);
	PX_UNUSED(convexPose);
	PX_UNUSED(unitDir);
	PX_UNUSED(distance);
	PX_UNUSED(sweepHit);
	PX_UNUSED(hitFlags);
	PX_UNUSED(inflation);

	bool res = false;
	switch (geom.getType())
	{
		case PxGeometryType::eSPHERE:
			res = sweepCapsule_ConvexCoreGeom(convexGeom, convexPose, PxCapsuleGeometry(static_cast<const PxSphereGeometry&>(geom).radius, 0),
				pose, Gu::Capsule(), -unitDir, distance, sweepHit, hitFlags, inflation, threadContext);
			break;
		case PxGeometryType::eCAPSULE:
			res = sweepCapsule_ConvexCoreGeom(convexGeom, convexPose, static_cast<const PxCapsuleGeometry&>(geom), pose,
				Gu::Capsule(), -unitDir, distance, sweepHit, hitFlags, inflation, threadContext);
			break;
		case PxGeometryType::eBOX:
			res = sweepBox_ConvexCoreGeom(convexGeom, convexPose, static_cast<const PxBoxGeometry&>(geom), pose,
				Gu::Box(), -unitDir, distance, sweepHit, hitFlags, inflation, threadContext);
			break;
		case PxGeometryType::eCONVEXMESH:
			res = sweepConvex_ConvexCoreGeom(convexGeom, convexPose, static_cast<const PxConvexMeshGeometry&>(geom), pose,
				-unitDir, distance, sweepHit, hitFlags, inflation, threadContext);
			break;
		default:
			PX_ASSERT(0);
	}

	if (res)
	{
		if (sweepHit.flags & PxHitFlag::eNORMAL)
			sweepHit.normal = -sweepHit.normal;

		if (sweepHit.flags & PxHitFlag::ePOSITION)
			sweepHit.position += unitDir * sweepHit.distance;
	}

	return res;
}

static bool sweepConvexCore_ConvexCore(GU_CONVEXCORE_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(threadContext);
	PX_UNUSED(geom);
	PX_UNUSED(pose);
	PX_UNUSED(convexGeom);
	PX_UNUSED(convexPose);
	PX_UNUSED(unitDir);
	PX_UNUSED(distance);
	PX_UNUSED(sweepHit);
	PX_UNUSED(hitFlags);
	PX_UNUSED(inflation);

	struct ConvexCoreSupport : PxGjkQuery::Support
	{
		Gu::ConvexShape shape;
		ConvexCoreSupport(const PxConvexCoreGeometry& g)
			{ Gu::makeConvexShape(g, PxTransform(PxIdentity), shape); }
		virtual PxReal getMargin() const
			{ return shape.margin; }
		virtual PxVec3 supportLocal(const PxVec3& dir) const
			{ return shape.supportLocal(dir); }
	};

	const PxConvexCoreGeometry& convex = static_cast<const PxConvexCoreGeometry&>(geom);
	if (convex.isValid())
	{
		PxBounds3 bounds = Gu::computeBounds(convex, pose);
		bounds.include(Gu::computeBounds(convexGeom, convexPose));
		PxReal wiseDist = PxMin(distance, bounds.getDimensions().magnitude());
		PxReal t;
		PxVec3 n, p;
		if (PxGjkQuery::sweep(ConvexCoreSupport(convex), ConvexCoreSupport(convexGeom), pose, convexPose, unitDir, wiseDist, t, n, p))
		{
			PxGeomSweepHit& hit = sweepHit;
			hit.distance = t;
			hit.position = p;
			hit.normal = n;
			hit.flags |= PxHitFlag::ePOSITION | PxHitFlag::eNORMAL;
			return true;
		}
	}

	return false;
}

static bool sweepConvexCore_MeshGeom(GU_CONVEXCORE_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(threadContext);
	PX_UNUSED(geom);
	PX_UNUSED(pose);
	PX_UNUSED(convexGeom);
	PX_UNUSED(convexPose);
	PX_UNUSED(unitDir);
	PX_UNUSED(distance);
	PX_UNUSED(sweepHit);
	PX_UNUSED(hitFlags);
	PX_UNUSED(inflation);

	struct TriSupport : PxGjkQuery::Support
	{
		PxVec3 v0, v1, v2;

		TriSupport(const PxVec3& _v0, const PxVec3& _v1, const PxVec3& _v2)
			: v0(_v0), v1(_v1), v2(_v2) {}

		virtual PxReal getMargin() const
			{ return 0; }

		virtual PxVec3 supportLocal(const PxVec3& dir) const
			{ PxReal d0 = dir.dot(v0), d1 = dir.dot(v1), d2 = dir.dot(v2);
			  return (d0 > d1 && d0 > d2) ? v0 : (d1 > d2) ? v1 : v2; }
	};

	struct ConvexCoreSupport : PxGjkQuery::Support
	{
		Gu::ConvexShape shape;
		PxReal inflation;

		ConvexCoreSupport(const PxConvexCoreGeometry& g, PxReal inf)
			: inflation(inf)
			{ Gu::makeConvexShape(g, PxTransform(PxIdentity), shape); }

		virtual PxReal getMargin() const
			{ return shape.margin + inflation; }

		virtual PxVec3 supportLocal(const PxVec3& dir) const
			{ return shape.supportLocal(dir); }
	};

	struct Callback : MeshHitCallback<PxGeomRaycastHit>
	{
		ConvexCoreSupport convexSupport;
		PxTransform convexPose;
		PxVec3 unitDir;
		PxReal maxDist;

		PxGeomSweepHit closestHit;

		Callback(const PxConvexCoreGeometry& geom, const PxTransform& pose, const PxVec3& dir, PxReal dist, PxReal inf)
			:
			MeshHitCallback<PxGeomRaycastHit>(CallbackMode::eMULTIPLE),
			convexSupport(geom, inf), convexPose(pose), unitDir(dir), maxDist(dist)
		{
			closestHit.distance = FLT_MAX;
		}

		virtual PxAgain processHit(const PxGeomRaycastHit& hit, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, PxReal&, const PxU32*)
		{
			PxReal t; PxVec3 n, p;
			TriSupport triSupport(v0, v1, v2);
			if (PxGjkQuery::sweep(triSupport, convexSupport, PxTransform(PxIdentity), convexPose, unitDir, maxDist, t, n, p))
			{
				if (t < closestHit.distance)
				{
					closestHit.distance = t;
					closestHit.position = p;
					closestHit.normal = n;
					closestHit.faceIndex = hit.faceIndex;
				}
			}
			return true;
		}
	};

	const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom);
	const TriangleMesh* meshData = _getMeshData(meshGeom);
	const bool doubleSided = (meshGeom.meshFlags & PxMeshGeometryFlag::eDOUBLE_SIDED) || (hitFlags & PxHitFlag::eMESH_BOTH_SIDES);

	PxTransform localPose = pose.transformInv(convexPose);
	PxVec3 localDir = pose.transformInv(unitDir);
	PxBounds3 bounds = Gu::computeBounds(convexGeom, localPose);
	bounds.include(Gu::computeBounds(convexGeom, PxTransform(localPose.p + localDir * distance, localPose.q)));

	Box queryBox;
	queryBox.extents = bounds.getExtents() + PxVec3(inflation);
	queryBox.center = bounds.getCenter();
	queryBox.rot = PxMat33(PxIdentity);

	Callback callback(convexGeom, localPose, localDir, distance, inflation);
	Midphase::intersectOBB(meshData, queryBox, callback, doubleSided);

	if (callback.closestHit.distance <= distance)
	{
		PxGeomSweepHit& hit = sweepHit;
		hit.distance = callback.closestHit.distance;
		hit.position = pose.transform(callback.closestHit.position);
		hit.normal = pose.rotate(callback.closestHit.normal);
		hit.faceIndex = callback.closestHit.faceIndex;
		hit.flags |= PxHitFlag::ePOSITION | PxHitFlag::eNORMAL | PxHitFlag::eFACE_INDEX;
		return true;
	}

	return false;
}

bool sweepConvexCore_HeightFieldGeom(GU_CONVEXCORE_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(threadContext);
	PX_UNUSED(geom);
	PX_UNUSED(pose);
	PX_UNUSED(convexGeom);
	PX_UNUSED(convexPose);
	PX_UNUSED(unitDir);
	PX_UNUSED(distance);
	PX_UNUSED(sweepHit);
	PX_UNUSED(hitFlags);
	PX_UNUSED(inflation);

	struct TriSupport : PxGjkQuery::Support
	{
		PxVec3 v0, v1, v2;

		TriSupport(const PxVec3& _v0, const PxVec3& _v1, const PxVec3& _v2)
			: v0(_v0), v1(_v1), v2(_v2) {}

		virtual PxReal getMargin() const
			{ return 0; }

		virtual PxVec3 supportLocal(const PxVec3& dir) const
			{ PxReal d0 = dir.dot(v0), d1 = dir.dot(v1), d2 = dir.dot(v2);
			  return (d0 > d1 && d0 > d2) ? v0 : (d1 > d2) ? v1 : v2; }
	};

	struct ConvexCoreSupport : PxGjkQuery::Support
	{
		Gu::ConvexShape shape;
		PxReal inflation;

		ConvexCoreSupport(const PxConvexCoreGeometry& g, PxReal inf)
			: inflation(inf)
			{ Gu::makeConvexShape(g, PxTransform(PxIdentity), shape); }

		virtual PxReal getMargin() const
			{ return shape.margin + inflation; }

		virtual PxVec3 supportLocal(const PxVec3& dir) const
			{ return shape.supportLocal(dir); }
	};

	struct Callback : Gu::OverlapReport
	{
		ConvexCoreSupport convexSupport;
		PxTransform convexPose;
		PxVec3 unitDir;
		PxReal maxDist;
		const Gu::HeightFieldUtil mHfUtil;
		const PxTransform& mHFPose;

		PxGeomSweepHit closestHit;

		Callback(const PxConvexCoreGeometry& geom, const PxTransform& pose, const PxVec3& dir, PxReal dist, PxReal inf, const PxHeightFieldGeometry& hfGeom, const PxTransform& hfPose)
			: convexSupport(geom, inf), convexPose(pose), unitDir(dir), maxDist(dist), mHfUtil(hfGeom), mHFPose(hfPose)
		{
			closestHit.distance = FLT_MAX;
		}

		virtual bool reportTouchedTris(PxU32 nb, const PxU32* indices)
		{
			while(nb--)
			{
				const PxU32 triangleIndex = *indices++;

				PxTrianglePadded currentTriangle;
				mHfUtil.getTriangle(mHFPose, currentTriangle, NULL, NULL, triangleIndex, false, false);

				const PxVec3& v0 = currentTriangle.verts[0];
				const PxVec3& v1 = currentTriangle.verts[1];
				const PxVec3& v2 = currentTriangle.verts[2];

				PxReal t; PxVec3 n, p;
				TriSupport triSupport(v0, v1, v2);
				if (PxGjkQuery::sweep(triSupport, convexSupport, PxTransform(PxIdentity), convexPose, unitDir, maxDist, t, n, p))
				{
					if (t < closestHit.distance)
					{
						closestHit.distance = t;
						closestHit.position = p;
						closestHit.normal = n;
						closestHit.faceIndex = triangleIndex;
					}
				}
			}
			return true;
		}
	};

	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom);
	const HeightField* hf = static_cast<const HeightField*>(hfGeom.heightField);
	if(!hf)
		return false;

	PxTransform localPose = pose.transformInv(convexPose);
	PxVec3 localDir = pose.rotateInv(unitDir);
	PxBounds3 bounds = Gu::computeBounds(convexGeom, localPose);
	bounds.include(Gu::computeBounds(convexGeom, PxTransform(localPose.p + localDir * distance, localPose.q)));

	Callback callback(convexGeom, localPose, localDir, distance, inflation, hfGeom, pose);
	callback.mHfUtil.overlapAABBTriangles(bounds, callback, 4);

	if (callback.closestHit.distance <= distance)
	{
		PxGeomSweepHit& hit = sweepHit;
		hit.distance = callback.closestHit.distance;
		hit.position = pose.transform(callback.closestHit.position);
		hit.normal = pose.rotate(callback.closestHit.normal);
		hit.faceIndex = callback.closestHit.faceIndex;
		hit.flags |= PxHitFlag::ePOSITION | PxHitFlag::eNORMAL | PxHitFlag::eFACE_INDEX;
		return true;
	}

	return false;
}

bool sweepConvex_ConvexCoreGeom(GU_CONVEX_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(threadContext);
	PX_UNUSED(geom);
	PX_UNUSED(pose);
	PX_UNUSED(convexGeom);
	PX_UNUSED(convexPose);
	PX_UNUSED(unitDir);
	PX_UNUSED(distance);
	PX_UNUSED(sweepHit);
	PX_UNUSED(hitFlags);
	PX_UNUSED(inflation);

	struct ConvexSupport : PxGjkQuery::Support
	{
		PxConvexMeshGeometry convex;
		PxReal inflation;

		ConvexSupport(const PxConvexMeshGeometry& c, PxReal inf)
			: convex(c), inflation(inf) {}

		virtual PxReal getMargin() const
			{ return inflation; }

		virtual PxVec3 supportLocal(const PxVec3& dir) const
		{
			PxVec3 d = convex.scale.rotation.rotateInv(convex.scale.rotation.rotate(dir)
				.multiply(convex.scale.scale));

			const PxVec3* verts = convex.convexMesh->getVertices();
			int count = int(convex.convexMesh->getNbVertices());
			float maxDot = -FLT_MAX;
			int index = -1;
			for (int i = 0; i < count; ++i)
			{
				float dot = verts[i].dot(d);
				if (dot > maxDot)
				{
					maxDot = dot;
					index = i;
				}
			}

			if (index == -1)
				return PxVec3(0);

			return convex.scale.rotation.rotateInv(convex.scale.rotation.rotate(verts[index])
				.multiply(convex.scale.scale));
		}
	};

	struct ConvexCoreSupport : PxGjkQuery::Support
	{
		Gu::ConvexShape shape;
		ConvexCoreSupport(const PxConvexCoreGeometry& g)
			{ Gu::makeConvexShape(g, PxTransform(PxIdentity), shape); }
		virtual PxReal getMargin() const
			{ return shape.margin; }
		virtual PxVec3 supportLocal(const PxVec3& dir) const
			{ return shape.supportLocal(dir); }
	};

	const PxConvexCoreGeometry& convex = static_cast<const PxConvexCoreGeometry&>(geom);
	if (convex.isValid())
	{
		PxBounds3 bounds = Gu::computeBounds(convex, pose);
		bounds.include(Gu::computeBounds(convexGeom, convexPose));
		PxReal wiseDist = PxMin(distance, bounds.getDimensions().magnitude());
		PxReal t;
		PxVec3 n, p;
		if (PxGjkQuery::sweep(ConvexCoreSupport(convex), ConvexSupport(convexGeom, inflation), pose,
			convexPose, unitDir, wiseDist, t, n, p))
		{
			PxGeomSweepHit& hit = sweepHit;
			hit.distance = t;
			hit.position = p;
			hit.normal = n;
			hit.flags |= PxHitFlag::ePOSITION | PxHitFlag::eNORMAL;
			return true;
		}
	}

	return false;
}

Gu::GeomSweepFuncs gGeomSweepFuncs =
{
	{
		sweepCapsule_SphereGeom,
		sweepCapsule_PlaneGeom,
		sweepCapsule_CapsuleGeom,
		sweepCapsule_BoxGeom,
		sweepCapsule_ConvexCoreGeom,
		sweepCapsule_ConvexGeom,
		sweepCapsule_InvalidGeom,
		sweepCapsule_InvalidGeom,
		sweepCapsule_MeshGeom,
		sweepCapsule_HeightFieldGeom,
		sweepCapsule_CustomGeom
	},
	{
		sweepCapsule_SphereGeom,
		sweepCapsule_PlaneGeom,
		sweepCapsule_CapsuleGeom,
		sweepCapsule_BoxGeom_Precise,
		sweepCapsule_ConvexCoreGeom,
		sweepCapsule_ConvexGeom,
		sweepCapsule_InvalidGeom,
		sweepCapsule_InvalidGeom,
		sweepCapsule_MeshGeom ,
		sweepCapsule_HeightFieldGeom,
		sweepCapsule_CustomGeom
	},
	{
		sweepBox_SphereGeom,
		sweepBox_PlaneGeom,
		sweepBox_CapsuleGeom,
		sweepBox_BoxGeom,
		sweepBox_ConvexCoreGeom,
		sweepBox_ConvexGeom,
		sweepBox_InvalidGeom,
		sweepBox_InvalidGeom,
		sweepBox_MeshGeom,		
		sweepBox_HeightFieldGeom,
		sweepBox_CustomGeom
	},
	{
		sweepBox_SphereGeom_Precise,
		sweepBox_PlaneGeom,
		sweepBox_CapsuleGeom_Precise,
		sweepBox_BoxGeom_Precise,
		sweepBox_ConvexCoreGeom,
		sweepBox_ConvexGeom,
		sweepBox_InvalidGeom,
		sweepBox_InvalidGeom,
		sweepBox_MeshGeom,		
		sweepBox_HeightFieldGeom_Precise,
		sweepBox_CustomGeom
	},
	{
		sweepConvexCore_Convex,			// 0
		sweepConvexCore_Plane,			// 1
		sweepConvexCore_Convex,			// 2
		sweepConvexCore_Convex,			// 3
		sweepConvexCore_ConvexCore,		// 4
		sweepConvexCore_Convex,			// 5
		sweepConvexCore_InvalidGeom,	// 6
		sweepConvexCore_InvalidGeom,	// 7
		sweepConvexCore_MeshGeom,		// 8
		sweepConvexCore_HeightFieldGeom,// 9
		sweepConvexCore_CustomGeom		// 10
	},
	{
		sweepConvex_SphereGeom,		// 0
		sweepConvex_PlaneGeom,		// 1
		sweepConvex_CapsuleGeom,	// 2
		sweepConvex_BoxGeom,		// 3
		sweepConvex_ConvexCoreGeom,	// 4
		sweepConvex_ConvexGeom,		// 5
		sweepConvex_InvalidGeom,	// 6
		sweepConvex_InvalidGeom,	// 7
		sweepConvex_MeshGeom,		// 8			
		sweepConvex_HeightFieldGeom,// 9
		sweepConvex_CustomGeom		// 10
	}
};

PX_PHYSX_COMMON_API const GeomSweepFuncs& Gu::getSweepFuncTable()
{
	return gGeomSweepFuncs;
}

