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

		const PxVec3 closestPoint = traversalController.getClosestPoint();
		PxVec4 barycentric;
		PxComputeBarycentric(guideVertices[tri[0]], guideVertices[tri[1]], guideVertices[tri[2]], closestPoint, barycentric);

		PxReal closestDistance = (embeddedVertices[i] - closestPoint).normalize();

		embeddingInfo[i] = PxTriangleMeshEmbeddingInfo(PxVec2(barycentric.x, barycentric.y), closestDistance, closestTriangleid);
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

