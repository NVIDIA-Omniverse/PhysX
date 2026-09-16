// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuSweepBoxSphere.h"
#include "GuOverlapTests.h"
#include "GuSphere.h"
#include "GuBoxConversion.h"
#include "GuCapsule.h"
#include "GuIntersectionRayCapsule.h"
#include "GuIntersectionRayBox.h"
#include "GuIntersectionSphereBox.h"
#include "GuDistancePointSegment.h"
#include "GuInternal.h"

using namespace physx;
using namespace Gu;

// PT: TODO: get rid of this copy
static const PxVec3 gNearPlaneNormal[] = 
{
	PxVec3(1.0f, 0.0f, 0.0f),
	PxVec3(0.0f, 1.0f, 0.0f),
	PxVec3(0.0f, 0.0f, 1.0f),
	PxVec3(-1.0f, 0.0f, 0.0f),
	PxVec3(0.0f, -1.0f, 0.0f),
	PxVec3(0.0f, 0.0f, -1.0f)
};

bool Gu::sweepBoxSphere(const Box& box, PxReal sphereRadius, const PxVec3& spherePos, const PxVec3& dir, PxReal length, PxReal& min_dist, PxVec3& normal, PxHitFlags hitFlags)
{
	if(!(hitFlags & PxHitFlag::eASSUME_NO_INITIAL_OVERLAP))
	{
		// PT: test if shapes initially overlap
		if(intersectSphereBox(Sphere(spherePos, sphereRadius), box))
		{
			// Overlap
			min_dist	= 0.0f;
			normal		= -dir;
			return true;
		}
	}

	PxVec3 boxPts[8];
	box.computeBoxPoints(boxPts);
	const PxU8* PX_RESTRICT edges = getBoxEdges();
	PxReal MinDist = length;
	bool Status = false;
	for(PxU32 i=0; i<12; i++)
	{
		const PxU8 e0 = *edges++;
		const PxU8 e1 = *edges++;
		const Capsule capsule(boxPts[e0], boxPts[e1], sphereRadius);

		PxReal t;
		if(intersectRayCapsule(spherePos, dir, capsule, t))
		{
			if(t>=0.0f && t<=MinDist)
			{
				MinDist = t;

				const PxVec3 ip = spherePos + t*dir;
				distancePointSegmentSquared(capsule, ip, &t);

				PxVec3 ip2;
				capsule.computePoint(ip2, t);

				normal = (ip2 - ip);
				normal.normalize();
				Status = true;
			}
		}
	}

	PxVec3 localPt;
	{
		PxMat34 M2;
		buildMatrixFromBox(M2, box);

		localPt = M2.rotateTranspose(spherePos - M2.p);
	}

	const PxVec3* boxNormals = gNearPlaneNormal;

	const PxVec3 localDir = box.rotateInv(dir);

	// PT: when the box exactly touches the sphere, the test for initial overlap can fail on some platforms.
	// In this case we reach the sweep code below, which may return a slightly negative time of impact (it should be 0.0
	// but it ends up a bit negative because of limited FPU accuracy). The epsilon ensures that we correctly detect a hit
	// in this case.
	const PxReal epsilon = -1e-5f;

	PxReal tnear, tfar;

	PxVec3 extents = box.extents;
	extents.x += sphereRadius;
	int plane = intersectRayAABB(-extents, extents, localPt, localDir, tnear, tfar);
	if(plane!=-1 && tnear>=epsilon && tnear <= MinDist)
	{
		MinDist = PxMax(tnear, 0.0f);
		normal = box.rotate(boxNormals[plane]);
		Status = true;
	}

	extents = box.extents;
	extents.y += sphereRadius;
	plane = intersectRayAABB(-extents, extents, localPt, localDir, tnear, tfar);
	if(plane!=-1 && tnear>=epsilon && tnear <= MinDist)
	{
		MinDist = PxMax(tnear, 0.0f);
		normal = box.rotate(boxNormals[plane]);
		Status = true;
	}

	extents = box.extents;
	extents.z += sphereRadius;
	plane = intersectRayAABB(-extents, extents, localPt, localDir, tnear, tfar);
	if(plane!=-1 && tnear>=epsilon && tnear <= MinDist)
	{
		MinDist = PxMax(tnear, 0.0f);
		normal = box.rotate(boxNormals[plane]);
		Status = true;
	}

	min_dist = MinDist;

	return Status;
}
