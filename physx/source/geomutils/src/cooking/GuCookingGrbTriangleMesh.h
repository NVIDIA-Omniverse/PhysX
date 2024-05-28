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

#ifndef GU_COOKING_GRB_TRIANGLE_MESH_H
#define GU_COOKING_GRB_TRIANGLE_MESH_H

#include "foundation/PxPlane.h"
#include "foundation/PxSort.h"
#include "GuMeshData.h"
#include "GuTriangle.h"
#include "GuEdgeList.h"
#include "cooking/PxCooking.h"
#include "CmRadixSort.h"

//#define CHECK_OLD_CODE_VS_NEW_CODE

namespace physx
{
namespace Gu
{
PX_ALIGN_PREFIX(16)
struct uint4
{
	unsigned int x, y, z, w;
}
PX_ALIGN_SUFFIX(16);


// TODO avoroshilov: remove duplicate definitions
static const PxU32 BOUNDARY = 0xffffffff;
static const PxU32 NONCONVEX_FLAG = 0x80000000;

#ifdef CHECK_OLD_CODE_VS_NEW_CODE

struct EdgeTriLookup
{
	PxU32 edgeId0, edgeId1;
	PxU32 triId;

	bool operator < (const EdgeTriLookup& edge1) const
	{
		return edgeId0 < edge1.edgeId0 || (edgeId0 == edge1.edgeId0 && edgeId1 < edge1.edgeId1);
	}

	bool operator <=(const EdgeTriLookup& edge1) const
	{
		return edgeId0 < edge1.edgeId0 || (edgeId0 == edge1.edgeId0 && edgeId1 <= edge1.edgeId1);
	}
};

static PxU32 binarySearch(const EdgeTriLookup* PX_RESTRICT data, const PxU32 numElements, const EdgeTriLookup& value)
{
	PxU32 left = 0;
	PxU32 right = numElements;

	while ((right - left) > 1)
	{
		const PxU32 pos = (left + right) / 2;
		const EdgeTriLookup& element = data[pos];
		if (element <= value)
		{
			left = pos;
		}
		else
		{
			right = pos;
		}
	}

	return left;
}

// slightly different behavior from collide2: boundary edges are filtered out

static PxU32 findAdjacent(const PxVec3* triVertices, const PxVec3* triNormals, const IndexedTriangle32* triIndices, 
	PxU32 nbTris, PxU32 i0, PxU32 i1, const PxPlane& plane,
	EdgeTriLookup* triLookups, PxU32 triangleIndex)
{
	PxU32 result = BOUNDARY;
	PxReal bestCos = -FLT_MAX;

	EdgeTriLookup lookup;
	lookup.edgeId0 = PxMin(i0, i1);
	lookup.edgeId1 = PxMax(i0, i1);

	PxU32 startIndex = binarySearch(triLookups, nbTris * 3, lookup);

	for (PxU32 a = startIndex; a > 0; --a)
	{
		if (triLookups[a - 1].edgeId0 == lookup.edgeId0 && triLookups[a - 1].edgeId1 == lookup.edgeId1)
			startIndex = a - 1;
		else
			break;
	}

	for (PxU32 a = startIndex; a < nbTris * 3; ++a)
	{
		const EdgeTriLookup& edgeTri = triLookups[a];

		if (edgeTri.edgeId0 != lookup.edgeId0 || edgeTri.edgeId1 != lookup.edgeId1)
			break;

		if (edgeTri.triId == triangleIndex)
			continue;

		const IndexedTriangle32& triIdx = triIndices[edgeTri.triId];
		const PxU32 vIdx0 = triIdx.mRef[0];
		const PxU32 vIdx1 = triIdx.mRef[1];
		const PxU32 vIdx2 = triIdx.mRef[2];

		const PxU32 other = vIdx0 + vIdx1 + vIdx2 - (i0 + i1);

		const PxReal c = plane.n.dot(triNormals[edgeTri.triId]);

		if (plane.distance(triVertices[other]) >= 0 && c > 0.f)
			return NONCONVEX_FLAG | edgeTri.triId;

		if (c>bestCos)
		{
			bestCos = c;
			result = edgeTri.triId;
		}
	}

	return result;
}
#endif

static PxU32 findAdjacent(const PxVec3* triVertices, const PxVec3* triNormals, const IndexedTriangle32* triIndices, const PxU32* faceByEdge, PxU32 nbTris, PxU32 i0, PxU32 i1, const PxPlane& plane, PxU32 triangleIndex)
{
	PxU32 result = BOUNDARY;
	PxReal bestCos = -FLT_MAX;

	for(PxU32 i=0; i<nbTris; i++)
	{
		const PxU32 candidateTriIndex = faceByEdge[i];
		if(triangleIndex==candidateTriIndex)
			continue;

		const IndexedTriangle32& triIdx = triIndices[candidateTriIndex];
		const PxU32 vIdx0 = triIdx.mRef[0];
		const PxU32 vIdx1 = triIdx.mRef[1];
		const PxU32 vIdx2 = triIdx.mRef[2];

		const PxU32 other = vIdx0 + vIdx1 + vIdx2 - (i0 + i1);

		const PxReal c = plane.n.dot(triNormals[candidateTriIndex]);

		if(plane.distance(triVertices[other]) >= 0 && c > 0.f)
			return NONCONVEX_FLAG | candidateTriIndex;

		if(c>bestCos)
		{
			bestCos = c;
			result = candidateTriIndex;
		}
	}

	return result;
}

static void buildAdjacencies(uint4* triAdjacencies, PxVec3* tempNormalsPerTri_prealloc, const PxVec3* triVertices, const IndexedTriangle32* triIndices, PxU32 nbTris)
{
#ifdef CHECK_OLD_CODE_VS_NEW_CODE
	{
		EdgeTriLookup* edgeLookups = PX_ALLOCATE(EdgeTriLookup, (nbTris * 3), "edgeLookups");

		for (PxU32 i = 0; i < nbTris; i++)
		{
			const IndexedTriangle32& triIdx = triIndices[i];
			const PxU32 vIdx0 = triIdx.mRef[0];
			const PxU32 vIdx1 = triIdx.mRef[1];
			const PxU32 vIdx2 = triIdx.mRef[2];

			tempNormalsPerTri_prealloc[i] = (triVertices[vIdx1] - triVertices[vIdx0]).cross(triVertices[vIdx2] - triVertices[vIdx0]).getNormalized();

			edgeLookups[i * 3].edgeId0 = PxMin(vIdx0, vIdx1);
			edgeLookups[i * 3].edgeId1 = PxMax(vIdx0, vIdx1);
			edgeLookups[i * 3].triId = i;

			edgeLookups[i * 3 + 1].edgeId0 = PxMin(vIdx1, vIdx2);
			edgeLookups[i * 3 + 1].edgeId1 = PxMax(vIdx1, vIdx2);
			edgeLookups[i * 3 + 1].triId = i;

			edgeLookups[i * 3 + 2].edgeId0 = PxMin(vIdx0, vIdx2);
			edgeLookups[i * 3 + 2].edgeId1 = PxMax(vIdx0, vIdx2);
			edgeLookups[i * 3 + 2].triId = i;
		}

		PxSort<EdgeTriLookup>(edgeLookups, PxU32(nbTris * 3));

		for (PxU32 i = 0; i < nbTris; i++)
		{
			const IndexedTriangle32& triIdx = triIndices[i];
			const PxU32 vIdx0 = triIdx.mRef[0];
			const PxU32 vIdx1 = triIdx.mRef[1];
			const PxU32 vIdx2 = triIdx.mRef[2];

			const PxPlane triPlane(triVertices[vIdx0], tempNormalsPerTri_prealloc[i]);
			uint4 triAdjIdx;

			triAdjIdx.x = findAdjacent(triVertices, tempNormalsPerTri_prealloc, triIndices, nbTris, vIdx0, vIdx1, triPlane, edgeLookups, i);
			triAdjIdx.y = findAdjacent(triVertices, tempNormalsPerTri_prealloc, triIndices, nbTris, vIdx1, vIdx2, triPlane, edgeLookups, i);
			triAdjIdx.z = findAdjacent(triVertices, tempNormalsPerTri_prealloc, triIndices, nbTris, vIdx2, vIdx0, triPlane, edgeLookups, i);
			triAdjIdx.w = 0;

			triAdjacencies[i] = triAdjIdx;
		}
	
		PX_FREE(edgeLookups);
	}
#endif

	if(1)
	{
		EDGELISTCREATE create;
		create.NbFaces		= nbTris;
		create.DFaces		= triIndices->mRef;
		create.WFaces		= NULL;
		create.FacesToEdges	= true;
		create.EdgesToFaces	= true;
		// PT: important: do NOT set the vertices, it triggers computation of edge flags that we don't need
		//create.Verts		= triVertices;
		EdgeList edgeList;
		if(edgeList.init(create))
		{
			for(PxU32 i=0; i<nbTris; i++)
			{
				const IndexedTriangle32& triIdx = triIndices[i];
				const PxU32 vIdx0 = triIdx.mRef[0];
				const PxU32 vIdx1 = triIdx.mRef[1];
				const PxU32 vIdx2 = triIdx.mRef[2];

				tempNormalsPerTri_prealloc[i] = (triVertices[vIdx1] - triVertices[vIdx0]).cross(triVertices[vIdx2] - triVertices[vIdx0]).getNormalized();
			}

			const EdgeTriangleData* edgeTriangleData = edgeList.getEdgeTriangles();
			const EdgeDescData* edgeToTriangle = edgeList.getEdgeToTriangles();
			const PxU32* faceByEdge = edgeList.getFacesByEdges();
			PX_ASSERT(edgeList.getNbFaces()==nbTris);

			for(PxU32 i=0; i<nbTris; i++)
			{
				const IndexedTriangle32& triIdx = triIndices[i];
				const PxU32 vIdx0 = triIdx.mRef[0];
				const PxU32 vIdx1 = triIdx.mRef[1];
				const PxU32 vIdx2 = triIdx.mRef[2];

				const PxPlane triPlane(triVertices[vIdx0], tempNormalsPerTri_prealloc[i]);

				const EdgeTriangleData& edgeTri = edgeTriangleData[i];
				const EdgeDescData& edgeData0 = edgeToTriangle[edgeTri.mLink[0] & MSH_EDGE_LINK_MASK];
				const EdgeDescData& edgeData1 = edgeToTriangle[edgeTri.mLink[1] & MSH_EDGE_LINK_MASK];
				const EdgeDescData& edgeData2 = edgeToTriangle[edgeTri.mLink[2] & MSH_EDGE_LINK_MASK];

				uint4 triAdjIdx;
				triAdjIdx.x = findAdjacent(triVertices, tempNormalsPerTri_prealloc, triIndices, faceByEdge + edgeData0.Offset, edgeData0.Count, vIdx0, vIdx1, triPlane, i);
				triAdjIdx.y = findAdjacent(triVertices, tempNormalsPerTri_prealloc, triIndices, faceByEdge + edgeData1.Offset, edgeData1.Count, vIdx1, vIdx2, triPlane, i);
				triAdjIdx.z = findAdjacent(triVertices, tempNormalsPerTri_prealloc, triIndices, faceByEdge + edgeData2.Offset, edgeData2.Count, vIdx2, vIdx0, triPlane, i);
				triAdjIdx.w = 0;

#ifdef CHECK_OLD_CODE_VS_NEW_CODE
				PX_ASSERT(triAdjacencies[i].x == triAdjIdx.x);
				PX_ASSERT(triAdjacencies[i].y == triAdjIdx.y);
				PX_ASSERT(triAdjacencies[i].z == triAdjIdx.z);
#endif
				triAdjacencies[i] = triAdjIdx;
			}
		}
	}
}

}
}

#endif
