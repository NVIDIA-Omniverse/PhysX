// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_MESH_DEPENETRATION_H
#define GU_MESH_DEPENETRATION_H

#include "common/PxPhysXCommonConfig.h"
#include "geometry/PxGeometryHit.h"
#include "geometry/PxTriangleMesh.h"

namespace physx
{
	class PxTriangleMeshGeometry;
	class PxMeshScale;

namespace Gu
{
	/**
	Depenetrates two triangle meshes in a given direction (reference code).

	This is a slow experimental iterative algorithm.

	\param meshGeom0	[in] geom for 1st mesh
	\param meshGeom1	[in] geom for 2nd mesh
	\param pose0		[in] pose for 1st mesh
	\param pose1		[in] pose for 2nd mesh
	\param dir			[in] depenetration direction
	\param maxIter		[in] max number of iterations allowed
	\param nbIter		[out] number of iterations used
	\return	Displacement vector
	*/
	PX_PHYSX_COMMON_API	PxVec3 depenetrateMeshesRef(const PxTriangleMeshGeometry& meshGeom0, const PxTriangleMeshGeometry& meshGeom1, const PxTransform& pose0, const PxTransform& pose1, const PxVec3& dir, PxU32 maxIter = 10000, PxU32* nbIter = NULL);

	/**
	Depenetrates two triangle meshes in a given direction.

	This is a faster experimental iterative algorithm.

	\param meshGeom0	[in] geom for 1st mesh
	\param meshGeom1	[in] geom for 2nd mesh
	\param pose0		[in] pose for 1st mesh
	\param pose1		[in] pose for 2nd mesh
	\param dir			[in] depenetration direction
	\param maxIter		[in] max number of iterations allowed
	\param nbIter		[out] number of iterations used
	\return	Displacement vector
	*/
	PX_PHYSX_COMMON_API	PxVec3 depenetrateMeshes(const PxTriangleMeshGeometry& meshGeom0, const PxTriangleMeshGeometry& meshGeom1, const PxTransform& pose0, const PxTransform& pose1, const PxVec3& dir, PxU32 maxIter = 10000, PxU32* nbIter = NULL);

	//////////////////////////////////////////////////////////////////////////////

	// Use this version to avoid virtual calls
	PX_FORCE_INLINE void getVertexReferences(PxU32& vref0, PxU32& vref1, PxU32& vref2, PxU32 triIndex, const void* triangles, PxU32 has16BitIndices)
	{
		if(has16BitIndices)
		{
			const PxU16* triangles16 = reinterpret_cast<const PxU16*>(triangles);
			vref0 = triangles16[triIndex*3+0];
			vref1 = triangles16[triIndex*3+1];
			vref2 = triangles16[triIndex*3+2];
		}
		else
		{
			const PxU32* triangles32 = reinterpret_cast<const PxU32*>(triangles);
			vref0 = triangles32[triIndex*3+0];
			vref1 = triangles32[triIndex*3+1];
			vref2 = triangles32[triIndex*3+2];
		}
	}

	// This version has 2 virtual calls to PxTriangleMesh
	PX_FORCE_INLINE void getVertexReferences(PxU32& vref0, PxU32& vref1, PxU32& vref2, const PxTriangleMesh* tm, PxU32 triIndex)
	{
		getVertexReferences(vref0, vref1, vref2, triIndex, tm->getTriangles(), tm->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES);
	}

	PX_PHYSX_COMMON_API	void getTriangle(PxVec3& p0, PxVec3& p1, PxVec3& p2, PxU32 triangleIndex, const PxTriangleMeshGeometry& meshGeom, const PxTransform* meshPose);


	//////////////////////////////////////////////////////////////////////////////

	class EdgeList;

	PX_PHYSX_COMMON_API	void computeDepenetrationVectors(PxU32 nbDirs, PxVec3* dirs);

	struct MaxLocalDepthContext
	{
		PX_FORCE_INLINE	MaxLocalDepthContext() : mUserDirs(NULL), mNbUserDirs(0), mMinDepthCutoff(-PX_MAX_F32), mMaxDepthCutoff(PX_MAX_F32), mIncludeSATAxes(true), mIterative(false)
		{
		}

		const PxVec3*	mUserDirs;
		PxU32			mNbUserDirs;
		float			mMinDepthCutoff;
		float			mMaxDepthCutoff;
		bool			mIncludeSATAxes;
		bool			mIterative;
	};

	PX_PHYSX_COMMON_API	float computeMaxLocalDepth(	const PxTriangleMeshGeometry& meshGeom0, const PxTransform& meshPose0, const EdgeList* edgeList0,
													const PxTriangleMeshGeometry& meshGeom1, const PxTransform& meshPose1, const EdgeList* edgeList1,
													PxU32 nbPairs, const PxGeomIndexPair* pairs, const MaxLocalDepthContext& context);

	PX_PHYSX_COMMON_API	float computeMaxLocalDepthBasic(const PxTriangleMeshGeometry& meshGeom0, const PxTransform& meshPose0,
														const PxTriangleMeshGeometry& meshGeom1, const PxTransform& meshPose1,
														PxU32 nbPairs, const PxGeomIndexPair* pairs);

} // namespace Gu
}

#endif
