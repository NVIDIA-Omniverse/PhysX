// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_SWEEP_MTD_H
#define GU_SWEEP_MTD_H

namespace physx
{
	class PxConvexMeshGeometry;
	class PxTriangleMeshGeometry;
	class PxGeometry;
	class PxHeightFieldGeometry;
	class PxPlane;

namespace Gu
{
	class Sphere;
	class Capsule;

	bool computeCapsule_TriangleMeshMTD(const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose, Gu::CapsuleV& capsuleV, PxReal inflatedRadius, bool isDoubleSided, PxGeomSweepHit& hit);

	bool computeCapsule_HeightFieldMTD(const PxHeightFieldGeometry& heightFieldGeom, const PxTransform& pose, Gu::CapsuleV& capsuleV, PxReal inflatedRadius, bool isDoubleSided, PxGeomSweepHit& hit);

	bool computeBox_TriangleMeshMTD(const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose, const Gu::Box& box, const PxTransform& boxTransform, PxReal inflation,
									bool isDoubleSided, PxGeomSweepHit& hit);    

	bool computeBox_HeightFieldMTD(	const PxHeightFieldGeometry& heightFieldGeom, const PxTransform& pose, const Gu::Box& box, const PxTransform& boxTransform, PxReal inflation, bool isDoubleSided, PxGeomSweepHit& hit);

	bool computeConvex_TriangleMeshMTD(	const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexTransform, PxReal inflation,
										bool isDoubleSided, PxGeomSweepHit& hit);

	bool computeConvex_HeightFieldMTD(	const PxHeightFieldGeometry& heightFieldGeom, const PxTransform& pose, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexTransform, PxReal inflation, bool isDoubleSided, PxGeomSweepHit& hit);

	bool computeSphere_SphereMTD(const Sphere& sphere0, const Sphere& sphere1, PxGeomSweepHit& hit);
	bool computeSphere_CapsuleMTD(const Sphere& sphere, const Capsule& capsule, PxGeomSweepHit& hit);

	bool computeCapsule_CapsuleMTD(const Capsule& capsule0, const Capsule& capsule1, PxGeomSweepHit& hit);

	bool computePlane_CapsuleMTD(const PxPlane& plane, const Capsule& capsule, PxGeomSweepHit& hit);
	bool computePlane_BoxMTD(const PxPlane& plane, const Box& box, PxGeomSweepHit& hit);
	bool computePlane_ConvexMTD(const PxPlane& plane, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose, PxGeomSweepHit& hit);

	// PT: wrapper just to avoid duplicating these lines.
	PX_FORCE_INLINE void setupSweepHitForMTD(PxGeomSweepHit& sweepHit, bool hasContacts, const PxVec3& unitDir)
	{
		sweepHit.flags = PxHitFlag::eNORMAL | PxHitFlag::eFACE_INDEX;
		if(!hasContacts)
		{
			sweepHit.distance	= 0.0f;
			sweepHit.normal		= -unitDir;
		}
		else
		{
			//ML: touching contact. We need to overwrite the normal to the negative of sweep direction
			if(sweepHit.distance == 0.0f && sweepHit.normal.isZero())
				sweepHit.normal = -unitDir;

			sweepHit.flags |= PxHitFlag::ePOSITION;
		}
	}
}

}


#endif
