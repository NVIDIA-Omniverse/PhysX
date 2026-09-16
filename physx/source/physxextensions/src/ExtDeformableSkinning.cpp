// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "extensions/PxDeformableSkinningExt.h"
#include "GuAABBTree.h"
#include "foundation/PxMathUtils.h"
#include "foundation/PxFPU.h"

using namespace physx;


void PxDeformableSkinningExt::initializeInterpolatedVertices(
	PxTriangleMeshEmbeddingInfo* embeddingInfo,
	const PxVec3* guideVertices, const PxVec3* guideNormals,
	const PxU32* guideTriangles, PxU32 numGuideTriangles, const PxVec3* embeddedVertices,
	PxU32 numEmbeddedVertices)
{
	PX_UNUSED(guideNormals); //The guideNormals could be used for higher quality embedding infos

	PX_SIMD_GUARD

	Gu::TinyBVH bvh;
	Gu::TinyBVH::constructFromTriangles(guideTriangles, numGuideTriangles, guideVertices, bvh);

	Gu::ClosestDistanceToTrimeshTraversalController traversalController(guideTriangles, guideVertices, bvh.mTree.begin());

	//Implements to most basic variant - lots of room for improvements
	for (PxU32 i = 0; i < numEmbeddedVertices; ++i)
	{
		traversalController.setQueryPoint(embeddedVertices[i]);
		bvh.Traverse(traversalController);

		const PxU32 closestTriangleid = traversalController.getClosestTriId();

		const PxU32* tri = &guideTriangles[3 * closestTriangleid];
		const PxVec3& v0 = guideVertices[tri[0]];
		const PxVec3& v1 = guideVertices[tri[1]];
		const PxVec3& v2 = guideVertices[tri[2]];

		const PxVec3 closestPoint = traversalController.getClosestPoint();

		PxVec3 triNormal = (v1 - v0).cross(v2 - v0);
		triNormal.normalize();

		PxVec3 distVec = (embeddedVertices[i] - closestPoint);

		PxReal normalOffset = distVec.dot(triNormal);
		PxVec3 projPoint = embeddedVertices[i] - normalOffset*triNormal;

		PxVec4 barycentric;
		PxComputeBarycentric(v0, v1, v2, projPoint, barycentric);

		embeddingInfo[i] = PxTriangleMeshEmbeddingInfo(PxVec2(barycentric.x, barycentric.y), normalOffset, closestTriangleid);
	}
}

void PxDeformableSkinningExt::initializeInterpolatedVertices(
	PxTetrahedronMeshEmbeddingInfo* embeddingInfo,
	const PxVec3* guideVertices,
	const PxU32* guideTetrahedra, PxU32 numGuideTetrahedra, const PxVec3* embeddedVertices,
	PxU32 numEmbeddedVertices	)
{
	PX_SIMD_GUARD

	Gu::TinyBVH bvh;
	Gu::TinyBVH::constructFromTetrahedra(guideTetrahedra, numGuideTetrahedra, guideVertices, bvh);

	Gu::ClosestDistanceToTetmeshTraversalController traversalController(guideTetrahedra, guideVertices, bvh.mTree.begin());

	//Implements to most basic variant - lots of room for improvements
	for (PxU32 i = 0; i < numEmbeddedVertices; ++i)
	{
		traversalController.setQueryPoint(embeddedVertices[i]);
		bvh.Traverse(traversalController);

		const PxU32 closestTetid = traversalController.getClosestTetId();

		const PxU32* tet = &guideTetrahedra[4 * closestTetid];

		const PxVec3 closestPoint = traversalController.getClosestPoint();
		PxVec4 barycentric;
		PxComputeBarycentric(guideVertices[tet[0]], guideVertices[tet[1]], guideVertices[tet[2]], guideVertices[tet[3]], closestPoint, barycentric);

		embeddingInfo[i] = PxTetrahedronMeshEmbeddingInfo(barycentric.getXYZ(), closestTetid);
	}
}

