// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxPlane.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "geometry/PxConvexMesh.h"
#include "extensions/PxConvexMeshExt.h"

using namespace physx;

static const PxReal gEpsilon = .01f;

PxU32 physx::PxFindFaceIndex(const PxConvexMeshGeometry& convexGeom, const PxTransform& pose, 
	const PxVec3& impactPos, const PxVec3& unitDir)
{
	PX_ASSERT(unitDir.isFinite());
	PX_ASSERT(unitDir.isNormalized());
	PX_ASSERT(impactPos.isFinite());
	PX_ASSERT(pose.isFinite());

	const PxVec3 impact = impactPos - unitDir * gEpsilon;

	const PxVec3 localPoint = pose.transformInv(impact);
	const PxVec3 localDir = pose.rotateInv(unitDir);

	// Create shape to vertex scale transformation matrix
	const PxMeshScale& meshScale = convexGeom.scale;
	const PxMat33 rot(meshScale.rotation);
	PxMat33 shape2VertexSkew = rot.getTranspose();
	const PxMat33 diagonal = PxMat33::createDiagonal(PxVec3(1.0f / meshScale.scale.x, 1.0f / meshScale.scale.y, 1.0f / meshScale.scale.z));
	shape2VertexSkew = shape2VertexSkew * diagonal;
	shape2VertexSkew = shape2VertexSkew * rot;

	const PxU32 nbPolys = convexGeom.convexMesh->getNbPolygons();
	PxU32 minIndex = 0;
	PxReal minD = PX_MAX_REAL;
	for (PxU32 j = 0; j < nbPolys; j++)
	{
		PxHullPolygon hullPolygon;
		convexGeom.convexMesh->getPolygonData(j, hullPolygon);
		
		// transform hull plane into shape space
		PxPlane plane;
		const PxVec3 tmp = shape2VertexSkew.transformTranspose(PxVec3(hullPolygon.mPlane[0],hullPolygon.mPlane[1],hullPolygon.mPlane[2]));
		const PxReal denom = 1.0f / tmp.magnitude();
		plane.n = tmp * denom;
		plane.d = hullPolygon.mPlane[3] * denom;

		PxReal d = plane.distance(localPoint);
		if (d < 0.0f)
			continue;

		const PxReal tweak = plane.n.dot(localDir) * gEpsilon;
		d += tweak;

		if (d < minD)
		{
			minIndex = j;
			minD = d;
		}
	}
	return minIndex;
}

