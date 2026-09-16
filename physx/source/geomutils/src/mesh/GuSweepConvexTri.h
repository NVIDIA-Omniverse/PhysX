// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_SWEEP_CONVEX_TRI
#define GU_SWEEP_CONVEX_TRI

#include "geometry/PxConvexMeshGeometry.h"
#include "GuVecTriangle.h"
#include "GuVecConvexHull.h"
#include "GuConvexMesh.h"
#include "GuGJKRaycast.h"

// return true if hit, false if no hit
static PX_FORCE_INLINE bool sweepConvexVsTriangle(
	const PxVec3& v0, const PxVec3& v1, const PxVec3& v2,
	ConvexHullV& convexHull, const aos::PxMatTransformV& meshToConvex, const aos::PxTransformV& convexTransfV,
	const aos::Vec3VArg convexSpaceDir, const PxVec3& unitDir, const PxVec3& meshSpaceUnitDir,
	const aos::FloatVArg fullDistance, PxReal shrunkDistance,
	PxGeomSweepHit& hit, bool isDoubleSided, PxReal inflation, bool& initialOverlap, PxU32 faceIndex)
{
	using namespace aos;
	if(!isDoubleSided)
	{
		// Create triangle normal
		const PxVec3 denormalizedNormal = (v1 - v0).cross(v2 - v1);

		// Backface culling
		// PT: WARNING, the test is reversed compared to usual because we pass -unitDir to this function
		const bool culled = denormalizedNormal.dot(meshSpaceUnitDir) <= 0.0f;
		if(culled)
			return false;
	}

	const Vec3V zeroV = V3Zero();
	const FloatV zero = FZero();

	const Vec3V p0 = V3LoadU(v0); // in mesh local space
	const Vec3V	p1 = V3LoadU(v1);
	const Vec3V p2 = V3LoadU(v2);

	// transform triangle verts from mesh local to convex local space
	TriangleV triangleV(meshToConvex.transform(p0), meshToConvex.transform(p1), meshToConvex.transform(p2));

	FloatV toi;
	Vec3V closestA,normal;

	const LocalConvex<TriangleV> convexA(triangleV);
	const LocalConvex<ConvexHullV> convexB(convexHull);
	const Vec3V initialSearchDir = V3Sub(triangleV.getCenter(), convexHull.getCenter());
	// run GJK raycast
	// sweep triangle in convex local space vs convex, closestA will be the impact point in convex local space
	const bool gjkHit = gjkRaycastPenetration<LocalConvex<TriangleV>, LocalConvex<ConvexHullV> >(
		convexA, convexB, initialSearchDir, zero, zeroV, convexSpaceDir, toi, normal, closestA, inflation, false);
	if(!gjkHit)
		return false;

	if(FAllGrtrOrEq(zero, toi))
	{
		initialOverlap	= true;	// PT: TODO: redundant with hit distance, consider removing
		return setInitialOverlapResults(hit, unitDir, faceIndex);
	}

	const FloatV minDist = FLoad(shrunkDistance);
	const FloatV dist = FMul(toi, fullDistance); // scale the toi to original full sweep distance
	if(FAllGrtr(minDist, dist)) // is current dist < minDist?
	{
		hit.faceIndex	= faceIndex;
		hit.flags		= PxHitFlag::ePOSITION | PxHitFlag::eNORMAL | PxHitFlag::eFACE_INDEX;
		const Vec3V destWorldPointA = convexTransfV.transform(closestA);
		const Vec3V destNormal = V3Normalize(convexTransfV.rotate(normal));
		V3StoreU(destWorldPointA, hit.position);
		V3StoreU(destNormal, hit.normal);
		FStore(dist, &hit.distance);
		return true; // report a hit
	}
	return false; // report no hit
}

#endif
