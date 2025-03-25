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

#ifndef __CONVEX_TRIANGLE_CUH__
#define __CONVEX_TRIANGLE_CUH__

#include "dataReadWriteHelper.cuh"
#include "convexNpCommon.h"
#include "cudaNpCommon.h"
#include "triangle.cuh"
#include "nputils.cuh"
#include "shuffle.cuh"
#include "contactReduction.cuh"
#include "utils.cuh"

/*

// TODO:

//-- tri vertex indices / tri adjacency - uint4 transposed/shuffled within thread group
//-- adjacent tri normal caluclation transposed/shuffled within thread group
//-- shared memory re-purpose
//-- check if LDGs are generated
//-- add shuffle versions of groupReduction/Scan

*/

namespace physx
{
	struct ConvexScratch
	{
		const PxU8* PX_RESTRICT convexPtrA;                     //8
		PxU32 nbEdgesNbHullVerticesNbPolygons;                  //12
	
		PxVec3 convexScale;                                     //24
		PxQuat convexScaleRot;                                  //40

		PxVec3 convexCenterOfMass; //shape space                //52

		PxReal contactDist;                                     //56
		PxReal restDist;                                        //60

		PxVec3 triangleLocNormal;                               //72
		PxVec3 triLocVerts[3];                                  //108

		PxU8 threadIds[32];                                     //140

		//we need to assign the adjacent triangles indices to the edge buffer so 
		//we can do post processing
		uint triAdjTrisIdx[3];                                  //152

		PxVec3 convexPlaneN[CONVEX_MAX_VERTICES_POLYGONS];      //920
		PxReal convexPlaneD[CONVEX_MAX_VERTICES_POLYGONS];      //1176

		PxVec3 convexScaledVerts[CONVEX_MAX_VERTICES_POLYGONS]; //1944

		__device__ const float4 * PX_RESTRICT getVertices() const
		{
			return (const float4*)(convexPtrA + sizeof(float4) + sizeof(uint4) + sizeof(float4));
		}

		__device__ const float4 * PX_RESTRICT getPlanes() const
		{
			PxU32 hullDesc = nbEdgesNbHullVerticesNbPolygons;
			return (const float4*)(convexPtrA + sizeof(float4) + sizeof(uint4) + sizeof(float4)
				+ sizeof(float4)* getNbVerts(hullDesc));
		}

		__device__ const PxU32 * PX_RESTRICT getPolyDescs() const //vRef8NbVertsMinIndex
		{
			PxU32 hullDesc = nbEdgesNbHullVerticesNbPolygons;
			return (const PxU32*)(convexPtrA + sizeof(float4) + sizeof(uint4) + sizeof(float4)
				+ sizeof(float4)* getNbVerts(hullDesc)
				+ sizeof(float4)* getNbPolygons(hullDesc));
		}

		__device__ const PxU16 * PX_RESTRICT getVerticesByEdges16() const
		{
			PxU32 hullDesc = nbEdgesNbHullVerticesNbPolygons;
			return (const PxU16*)(convexPtrA + sizeof(float4) + sizeof(uint4) + sizeof(float4)
				+ sizeof(float4)* getNbVerts(hullDesc)
				+ (sizeof(float4) + sizeof(PxU32)) * getNbPolygons(hullDesc));
		}

		__device__ const PxU8 * PX_RESTRICT getFacesByEdges8() const
		{
			PxU32 hullDesc = nbEdgesNbHullVerticesNbPolygons;
			return (const PxU8*)(convexPtrA + sizeof(float4) + sizeof(uint4) + sizeof(float4)
				+ sizeof(float4)* getNbVerts(hullDesc)
				+ (sizeof(float4) + sizeof(PxU32)) * getNbPolygons(hullDesc)
				+ sizeof(PxU16) * 2 * getNbEdges(hullDesc));
		}

		__device__ const PxU8 * PX_RESTRICT getFacesByVertices8() const
		{
			PxU32 hullDesc = nbEdgesNbHullVerticesNbPolygons;
			return (const PxU8*)(convexPtrA + sizeof(float4) + sizeof(uint4) + sizeof(float4)
				+ sizeof(float4)* getNbVerts(hullDesc)
				+ (sizeof(float4) + sizeof(PxU32)) * getNbPolygons(hullDesc)
				+ (sizeof(PxU16) + sizeof(PxU8)) * 2 * getNbEdges(hullDesc));
		}

		__device__ const PxU8 * PX_RESTRICT getVertexData8() const
		{
			PxU32 hullDesc = nbEdgesNbHullVerticesNbPolygons;
			return (const PxU8*)(convexPtrA + sizeof(float4) + sizeof(uint4) + sizeof(float4)
				+ sizeof(float4)* getNbVerts(hullDesc)
				+ (sizeof(float4) + sizeof(PxU32)) * getNbPolygons(hullDesc)
				+ (sizeof(PxU16) + sizeof(PxU8)) * 2 * getNbEdges(hullDesc)
				+ sizeof(PxU8) * 3 * getNbVerts(hullDesc));
		}
	};

	//PX_ALIGN_PREFIX(16)
	struct ConvexMeshScratch : public ConvexScratch
	{
		PxTransform trimeshToConvexTransform;

		const float4* PX_RESTRICT trimeshVerts;
		const uint4* PX_RESTRICT trimeshTriIndices;

		//this is just valid for mesh contact gen to map the cpu triangle index to the gpu triangle index
		//height field don't have remap table
		const PxU32* PX_RESTRICT trimeshFaceRemap;

	};// PX_ALIGN_SUFFIX(16);
	PX_COMPILE_TIME_ASSERT(sizeof(ConvexMeshScratch) <= WARP_SIZE * 16 * sizeof(PxU32));
}

//__device__ inline static PxVec3 getAdjacentTriangleNormal( PxU32 adjIdx, const PxVec3 triangleLocNormal, const PxTransform & meshToConvex, const PxVec3 & trimeshScale,
//	const PxQuat & trimeshRot, const float4 * trimeshVerts, const uint4 * trimeshTriIndices
//)
//{
//	PxVec3 adjLocNormal;
//	if (adjIdx == BOUNDARY)
//	{
//		adjLocNormal = -triangleLocNormal;
//	}
//	else if (adjIdx & NONCONVEX_FLAG)
//	{
//		adjLocNormal = triangleLocNormal;
//	}
//	else
//	{
//		//uint4 triIndices = trimeshTriIndices[adjIdx & (~NONCONVEX_FLAG)]
//		uint4 triIndices = trimeshTriIndices[adjIdx];
//		// TODO: convert to AOS
//		float4 adjTriV0_f4 = trimeshVerts[triIndices.x];
//		float4 adjTriV1_f4 = trimeshVerts[triIndices.y];
//		float4 adjTriV2_f4 = trimeshVerts[triIndices.z];
//
//		PxVec3 adjTriV0 = PxLoad3(adjTriV0_f4);
//		PxVec3 adjTriV1 = PxLoad3(adjTriV1_f4);
//		PxVec3 adjTriV2 = PxLoad3(adjTriV2_f4);
//
//#if 0
//		// TODO: Per-vertex transform debug
//#else
//		const PxVec3 adjTriangleNormal_meshSpace = (adjTriV1 - adjTriV0).cross(adjTriV2 - adjTriV0);
//		const PxVec3 adjTriangleNormal_scaledMeshSpace = vertex2ShapeNormalVector(adjTriangleNormal_meshSpace, trimeshScale, trimeshRot);
//		const PxVec3 adjTriangleNormal_scaledConvexSpace = meshToConvex.rotate(adjTriangleNormal_scaledMeshSpace);
//
//		adjLocNormal = adjTriangleNormal_scaledConvexSpace.getNormalized();
//#endif
//	}
//
//	return adjLocNormal;
//}

//__device__ static bool checkVertFaceFilter(PxU32 triAdjVertsStart, PxU32 triAdjVertsNum, const float4* trimeshVerts, const PxU32* trimeshAdjVerts, const PxTransform& trimeshToConv,
//const PxVec3& trimeshScale, const PxQuat& trimeshScaleRot, PxVec3 triVerts_j, PxVec3 n0)
//{
//	bool vertFaceFilter = true;
//	for (PxU32 k = triAdjVertsStart, kend = triAdjVertsStart + triAdjVertsNum; k < kend; ++k)
//	{
//		float4 vertNext_f4 = trimeshVerts[trimeshAdjVerts[k]];
//		PxVec3 vertNext = trimeshToConv.transform(vertex2Shape(PxLoad3(vertNext_f4), trimeshScale, trimeshScaleRot));
//		PxVec3 triMeshEdge = vertNext - triVerts_j;
//
//		const PxReal eps = 0.0f;
//
//		if (triMeshEdge.dot(n0) < eps)
//		{
//			vertFaceFilter = false;
//			break;
//		}
//	}
//
//	return vertFaceFilter;
//}

__device__ static bool satConvexFaceNormals(ConvexScratch* s_scratch, PxU32 & feature, PxVec3 & faceNormal, PxReal & separation)
{
	PxReal _separation = -PX_MAX_F32;
	PxU32  _feature = 0;
	PxVec3  _faceNormal = faceNormal;

	PxU32 nbPolygons0 = getNbPolygons(s_scratch->nbEdgesNbHullVerticesNbPolygons);
	const PxU32* PX_RESTRICT polyDescs = s_scratch->getPolyDescs();
	const float4* PX_RESTRICT planes = s_scratch->getPlanes();
	const PxVec3* PX_RESTRICT convexVerts = s_scratch->convexScaledVerts;

	bool disjoint = false;

	//in the local space of polyData0
	for (PxU32 i = threadIdx.x; i < nbPolygons0; i += WARP_SIZE)
	{
		PxU32 polygonData = polyDescs[i];
		const float4 polygonf4 = planes[i];
		const PxVec3 minVertConv = convexVerts[getMinIndex(polygonData)];

		const PxReal planeDist = polygonf4.w;
		const PxVec3 vertexSpacePlaneNormal(polygonf4.x, polygonf4.y, polygonf4.z);

		//transform plane n to shape space
		PxVec3 convexScale = s_scratch->convexScale;
		PxQuat convexScaleRot = s_scratch->convexScaleRot;

		PxVec3 shapeSpacePlaneNormal = vertex2ShapeNormalVector(vertexSpacePlaneNormal, convexScale, convexScaleRot);

		const PxReal invMagnitude = 1.0f / shapeSpacePlaneNormal.normalize();

		const PxReal min0 = shapeSpacePlaneNormal.dot(minVertConv);
		const PxReal max0 = -planeDist * invMagnitude;

		PxReal min1 = FLT_MAX;
		PxReal max1 = -FLT_MAX;

		// Triangle pre-scaled
		PxReal d0 = s_scratch->triLocVerts[0].dot(shapeSpacePlaneNormal);
		PxReal d1 = s_scratch->triLocVerts[1].dot(shapeSpacePlaneNormal);
		PxReal d2 = s_scratch->triLocVerts[2].dot(shapeSpacePlaneNormal);

		max1 = fmaxf(d0, fmaxf(d1, d2));
		min1 = fminf(d0, fminf(d1, d2));

		if ((min1 > max0 + s_scratch->contactDist) || (min0 > max1 + s_scratch->contactDist))
		{
			disjoint = true;
			break;
		}

		s_scratch->convexPlaneN[i] = shapeSpacePlaneNormal;
		s_scratch->convexPlaneD[i] = -max0;

		const PxReal tempSeparation = min1 - max0;

		if ((i < nbPolygons0) && (_separation < tempSeparation))// && curFaceFilter)
		{
			_separation = tempSeparation;
			_feature = i;
			_faceNormal = shapeSpacePlaneNormal;
		}
	}

	if (__any_sync(FULL_MASK, disjoint))
	{
		return false;
	}

	PxU32 winnerLane;
	_separation = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, _separation, winnerLane);

	_feature = __shfl_sync(FULL_MASK, _feature, winnerLane);

	if (separation < _separation)
	{
		_faceNormal = shuffle(FULL_MASK, _faceNormal, winnerLane);

		faceNormal = _faceNormal;
		separation = _separation;
		feature = _feature;
	}

	return true;
}

__device__ static bool satTriangleNormal(const ConvexScratch* s_scratch, PxU32 & feature, PxVec3 & faceNormal, PxReal & separation)
{
	PxReal _separation = separation;
	PxVec3  _faceNormal = faceNormal;

	const PxU32 nbConvexVertices = getNbVerts(s_scratch->nbEdgesNbHullVerticesNbPolygons);

	const PxVec3 n0 = s_scratch->triangleLocNormal;
	const PxReal triProj = n0.dot(s_scratch->triLocVerts[0]);

	PxReal min1 = FLT_MAX;
	PxReal max1 = -FLT_MAX;

	const PxVec3* PX_RESTRICT vertices = s_scratch->convexScaledVerts;

	// Calculate min/max projections onto tri normal
	for (PxU32 i = threadIdx.x; i < nbConvexVertices; i += WARP_SIZE)
	{
		const PxReal dist = vertices[i].dot(n0);
		max1 = fmaxf(dist, max1);
		min1 = fminf(dist, min1);
	}

	// Calculate projection across all warps

	min1 = warpReduction<MinOpFloat, PxReal>(FULL_MASK, min1);
	max1 = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, max1);

	bool isSeparated = (min1 > triProj + s_scratch->contactDist) || (triProj > max1 + s_scratch->contactDist);

	if (isSeparated)
	{
		return false;
	}

	_separation = min1 - triProj;

	if (separation < _separation)
	{
		faceNormal = -n0;
		separation = _separation;
		feature = TRI_FEATURE_IDX;
	}

	return true;
}

__device__ static bool isMinkowskiFace(const PxVec3& A, const PxVec3& B, const PxVec3& B_x_A, const PxVec3& C, const PxVec3& D, const PxVec3& D_x_C)
{
	// Two edges build a face on the Minkowski sum if the associated arcs AB and CD intersect on the Gauss map. 
	// The associated arcs are defined by the adjacent face normals of each edge.  
	const float CBA = C.dot(B_x_A);
	const float DBA = D.dot(B_x_A);
	const float ADC = A.dot(D_x_C);
	const float BDC = B.dot(D_x_C);

	//intersection test, CBA * DBA < 0.0f && ADC * BDC < 0.0f
	//hemisphere test CBA * BDC > 0.0f
	return CBA * DBA < 0.0f && ADC * BDC < 0.0f && CBA * BDC > 0.0f;
}

__device__ static bool isEdgePairPassed(const PxVec3 & convexNormal0, const PxVec3 & convexNormal1, const PxVec3 & convexEdge, const PxVec3 & convexVertex, const PxVec3 triNormal0,
	const PxVec3 & triEdge, const PxVec3 triVertex, /*PxU32 triAdjIndex,*/ const PxVec3 & convexCenterOfMass, const PxReal contactDist, PxVec3 & faceNormal,
	PxReal & separation)
{
	if (!isMinkowskiFace(convexNormal0, convexNormal1, convexEdge, -triNormal0, triNormal0, triEdge))
		return true;

	PxVec3 normal = convexEdge.cross(triEdge);

	//check for edge parallel case
	PxReal eps = 1e-6;
	if (abs(normal.x) < eps || abs(normal.y) < eps || abs(normal.z) < eps)
		return true;

	normal = normal.getNormalized();

	if (normal.dot(convexVertex - convexCenterOfMass) < 0)
		normal = -normal; //make sure the normal is point from convex hull to triangle

	float dist = normal.dot(triVertex - convexVertex);

	if (dist > contactDist)
	{
		return false;
	}

	if (separation < dist)
	{
		separation = dist; 
		//faceNormal = (triAdjIndex & NONCONVEX_FLAG) ? -triNormal0 : normal;
		faceNormal = normal;
	}

	return true;
}

__device__ static bool satEdgeCrossDirection(ConvexScratch* s_scratch, PxU32 & feature, PxVec3 & faceNormal, PxReal & separation)
{
	PxVec3 _faceNormal = faceNormal;
	PxReal _separation = separation;

	//PxTransform trimeshToConv = s_scratch->trimeshToConvexTransform;

	const PxU32 numConvexEdges = getNbEdges(s_scratch->nbEdgesNbHullVerticesNbPolygons);

	const PxU16* PX_RESTRICT verticesByEdges16 = s_scratch->getVerticesByEdges16();
	const PxU8* PX_RESTRICT facesByEdges8 = s_scratch->getFacesByEdges8();
	const float4* PX_RESTRICT planes = s_scratch->getPlanes();
	const float4* PX_RESTRICT vertices = s_scratch->getVertices();

	bool isPassed = true;
	for (PxU32 it = 0; it < numConvexEdges * 3; it += WARP_SIZE)
	{
		const PxU32 index = it + threadIdx.x;

		if (index < numConvexEdges * 3)
		{
			PxU32 convexEdgeIdx = index / 3;

			const PxU32 vIndices = *(reinterpret_cast<const PxU32*>(verticesByEdges16 + 2 * convexEdgeIdx));
			//!WARNING: exploiting little-endianness
			const PxU16 v0idx1 = u32Low(vIndices);//= polyData0_VerticesByEdges16[2*axis1];
			const PxU16 v1idx1 = u32High(vIndices); //= polyData0_VerticesByEdges16[2*axis1 + 1];

			const PxU16 fIndices = *(reinterpret_cast<const PxU16*>(facesByEdges8 + 2 * convexEdgeIdx));
			//!WARNING: exploiting little-endianness
			const PxU8 f10 = u16Low(fIndices); //= polyData0_FacesByEdges8[axis1*2];
			const PxU8 f11 = u16High(fIndices);//= polyData0_FacesByEdges8[axis1*2 + 1];

			const float4 vertex10 = vertices[v0idx1];
			const float4 vertex11 = vertices[v1idx1];
			//= map1Vertex2Shape.transform(polyData0.mVertices[v0idx]);
			PxVec3 convexScale = s_scratch->convexScale;
			PxQuat convexScaleRot = s_scratch->convexScaleRot;
			PxVec3	convexVertex = vertex2Shape(PxVec3(vertex10.x, vertex10.y, vertex10.z), convexScale, convexScaleRot);
			PxVec3	convexEdge = convexVertex - vertex2Shape(PxVec3(vertex11.x, vertex11.y, vertex11.z), convexScale, convexScaleRot);
			convexEdge.normalize();

			const float4 cvxPlane0 = planes[f10];
			const float4 cvxPlane1 = planes[f11];

			const PxVec3 convexNormal0 = vertex2ShapeNormalVector(-PxVec3(cvxPlane0.x, cvxPlane0.y, cvxPlane0.z), convexScale, convexScaleRot);
			const PxVec3 convexNormal1 = vertex2ShapeNormalVector(-PxVec3(cvxPlane1.x, cvxPlane1.y, cvxPlane1.z), convexScale, convexScaleRot);

			const PxU32 vStartIndex = index % 3;
			const PxU32 vEndIndex = (vStartIndex + 1) % 3;

			//const PxVec3 adjLocNormal = s_scratch->adjLocNormal[vStartIndex];
			const PxVec3 triEdge = (s_scratch->triLocVerts[vEndIndex] - s_scratch->triLocVerts[vStartIndex]).getNormalized();
			const PxVec3 triLocVert = s_scratch->triLocVerts[vStartIndex];
			//const PxU32 triAdjIndex = reinterpret_cast<PxU32*>(&s_scratch->triAdjTrisIdx)[vStartIndex];
		
			isPassed = isEdgePairPassed(
				convexNormal0, convexNormal1, convexEdge, convexVertex,
				s_scratch->triangleLocNormal, triEdge, triLocVert, /*triAdjIndex,*/
				s_scratch->convexCenterOfMass,
				s_scratch->contactDist,
				_faceNormal, _separation
				);
		}
		
		if (__any_sync(FULL_MASK, !isPassed))
		{
			break;
		}
	}

	if (__any_sync(FULL_MASK, !isPassed))
	{
		return false;
	}

	// Calculate projection across all warps

	PxU32 winnerLane;
	_separation = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, _separation, winnerLane);

	//unsigned mask_separation = __ballot_sync(syncMask, separation < _separation);
	if (separation < _separation)
	{
		_faceNormal = shuffle(FULL_MASK, _faceNormal, winnerLane);

		faceNormal = _faceNormal;
		separation = _separation;
		feature = EDGE_FEATURE_IDX;
	}

	return true;
}

inline __device__ void supportConvex(const PxVec3* PX_RESTRICT vertices, const PxU32 nbVerts, const PxVec3& dir, PxReal& minVal, PxReal& maxVal)
{
	minVal = PX_MAX_F32;
	maxVal = -PX_MAX_F32;

	for (PxU32 i = 0; i < nbVerts; ++i)
	{
		PxReal d = vertices[i].dot(dir);
		minVal = PxMin(d, minVal);
		maxVal = PxMax(d, maxVal);
	}
}

inline __device__ void supportTriangle(const PxVec3* vertices, const PxVec3& dir, PxReal& minVal, PxReal& maxVal)
{
	minVal = PX_MAX_F32;
	maxVal = -PX_MAX_F32;

	for (PxU32 i = 0; i < 3; ++i)
	{
		PxReal d = vertices[i].dot(dir);
		minVal = PxMin(d, minVal);
		maxVal = PxMax(d, maxVal);
	}
}

inline __device__ void supportTriangleWarp(PxVec3* vertices, const PxVec3& dir, PxReal& minVal, PxReal& maxVal, 
	const PxU32 threadIndex, const PxU32 mask, const PxU32 groupStartIndex)
{
	PxReal val;
	if (threadIndex < 3)
	{
		val = vertices[threadIndex].dot(dir);
	}

	PxReal val1 = __shfl_sync(mask, val, groupStartIndex+1);
	PxReal val2 = __shfl_sync(mask, val, groupStartIndex+2);
	PxReal min = PxMin(val, PxMin(val1, val2));
	PxReal max = PxMax(val, PxMax(val1, val2));

	minVal = __shfl_sync(mask, min, groupStartIndex);
	maxVal = __shfl_sync(mask, max, groupStartIndex);
}

template <PxU32 logThreadGroupSize, PxU32 NbThreads>
inline __device__ void supportConvexWarp(const PxVec3* PX_RESTRICT vertices, const PxU32 nbVerts, 
	const PxVec3& dir, PxReal& minVal, PxReal& maxVal, const PxU32 threadIndex, const PxU32 mask)
{
	PxReal min = PX_MAX_F32;
	PxReal max = -PX_MAX_F32;

	for (PxU32 i = threadIndex; i < nbVerts; i += NbThreads)
	{
		PxReal d = vertices[i].dot(dir);
		min = PxMin(d, min);
		max = PxMax(d, max);
	}

	minVal = warpReduction<MinOpFloat, PxReal, logThreadGroupSize>(mask, min);
	maxVal = warpReduction<MaxOpFloat, PxReal, logThreadGroupSize>(mask, max);
}

__device__ inline bool isActiveEdge(PxU32 edgeIndex)
{
	return edgeIndex == BOUNDARY || !(edgeIndex & (NONCONVEX_FLAG));
}

__device__ inline bool isBoundaryEdge(PxU32 edgeIndex)
{
	return edgeIndex == BOUNDARY;
}

struct TriangleSearchFlags
{
	enum Enum : PxU32
	{
		eFACE_CONTACT = 0x80000000,
		eEDGE_CONTACT = 0xc0000000,
	};
};

__device__ static bool satEdgeCrossDirection2(const ConvexScratch* s_scratch, PxU32 & feature, PxVec3 & faceNormal, PxReal & separation)
{
	PxVec3 _faceNormal = faceNormal;
	PxReal _separation = separation;

	//PxTransform trimeshToConv = s_scratch->trimeshToConvexTransform;

	const PxU32 numConvexEdges = getNbEdges(s_scratch->nbEdgesNbHullVerticesNbPolygons);
	const PxU32 numVertices = getNbVerts(s_scratch->nbEdgesNbHullVerticesNbPolygons);

	const PxU16* PX_RESTRICT verticesByEdges16 = s_scratch->getVerticesByEdges16();
	const PxU8* PX_RESTRICT facesByEdges8 = s_scratch->getFacesByEdges8();
	//const float4* PX_RESTRICT planes = s_scratch->getPlanes();
	const PxVec3* PX_RESTRICT planeNormals = s_scratch->convexPlaneN;
	const PxVec3* PX_RESTRICT vertices = s_scratch->convexScaledVerts;

	PxVec3 convexScale = s_scratch->convexScale;
	PxQuat convexScaleRot = s_scratch->convexScaleRot;

	//PxVec3 shapeNormal = shape2Vertex(s_scratch->triangleLocNormal, convexScale, convexScaleRot);
	PxVec3 shapeNormal = s_scratch->triangleLocNormal;

	const PxReal contactDist = s_scratch->contactDist;

	bool isPassed = true;
	for (PxU32 it = 0; it < numConvexEdges * 3; it += WARP_SIZE)
	{
		const PxU32 index = it + threadIdx.x;

		if (index < numConvexEdges * 3)
		{
			const PxU32 vStartIndex = index % 3;

			if (isActiveEdge(s_scratch->triAdjTrisIdx[vStartIndex]))
			{
				const PxU32 vEndIndex = (vStartIndex + 1) % 3;

				PxU32 convexEdgeIdx = index / 3;

				const PxU32 vIndices = *(reinterpret_cast<const PxU32*>(verticesByEdges16 + 2 * convexEdgeIdx));
				//!WARNING: exploiting little-endianness
				const PxU16 v0idx1 = u32Low(vIndices);//= polyData0_VerticesByEdges16[2*axis1];
				const PxU16 v1idx1 = u32High(vIndices); //= polyData0_VerticesByEdges16[2*axis1 + 1];

				const PxU16 fIndices = *(reinterpret_cast<const PxU16*>(facesByEdges8 + 2 * convexEdgeIdx));
				
				//!WARNING: exploiting little-endianness
				const PxU8 f10 = u16Low(fIndices);
				const PxU8 f11 = u16High(fIndices);

				if (shapeNormal.dot(planeNormals[f10]) < 0.f || shapeNormal.dot(planeNormals[f11]) < 0.f)
				{
					PxVec3	convexEdge = vertices[v0idx1] - vertices[v1idx1];

					const PxVec3 triEdge = (s_scratch->triLocVerts[vEndIndex] - s_scratch->triLocVerts[vStartIndex]);
					PxVec3 v = convexEdge.cross(triEdge);

					PxReal magSquared = v.magnitudeSquared();
					PxReal dp = v.dot(faceNormal);

					if (v.magnitudeSquared() > 1e-10f)
					{
						//convexEdge.normalize();

						const PxVec3 shapeSpaceV = v.getNormalized();

						PxReal minTri, maxTri;
						PxReal minConv, maxConv;

						supportTriangle(s_scratch->triLocVerts, shapeSpaceV, minTri, maxTri);

						supportConvex(vertices, numVertices, shapeSpaceV, minConv, maxConv);

						PxReal tempOverlap0 = minTri - maxConv;
						PxReal tempOverlap1 = minConv - maxTri;

						if (minConv > (maxTri + contactDist) || minTri > (maxConv + contactDist))
						{
							isPassed = false;
						}
						else
						{
							if (_separation < tempOverlap0)
							{
								_separation = tempOverlap0;
								_faceNormal = shapeSpaceV;
							}

							if (_separation < tempOverlap1)
							{
								_separation = tempOverlap1;
								_faceNormal = -shapeSpaceV;
							}
						}
					}
				}
			}
		}

		if (__any_sync(FULL_MASK, !isPassed))
		{
			return false;
		}
	}

	// Calculate projection across all warps

	PxU32 winnerLane;
	_separation = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, _separation, winnerLane);

	//unsigned mask_separation = __ballot_sync(syncMask, separation < _separation);
	if (separation < _separation)
	{
		_faceNormal = shuffle(FULL_MASK, _faceNormal, winnerLane);

		faceNormal = _faceNormal;
		separation = _separation;
		feature = EDGE_FEATURE_IDX;
	}

	return true;
}


//__device__ static bool satEdgeCrossDirection2(ConvexScratch* s_scratch, PxU32 & feature, PxVec3 & faceNormal, PxReal & separation)
//{
//	PxVec3 _faceNormal = faceNormal;
//	PxReal _separation = separation;
//
//	//PxTransform trimeshToConv = s_scratch->trimeshToConvexTransform;
//
//	const PxU32 numConvexEdges = getNbEdges(s_scratch->nbEdgesNbHullVerticesNbPolygons);
//	const PxU32 numVertices = getNbVerts(s_scratch->nbEdgesNbHullVerticesNbPolygons);
//
//	const PxU16* PX_RESTRICT verticesByEdges16 = s_scratch->getVerticesByEdges16();
//	const PxU8* PX_RESTRICT facesByEdges8 = s_scratch->getFacesByEdges8();
//	//const float4* PX_RESTRICT planes = s_scratch->getPlanes();
//	const PxVec3* PX_RESTRICT planeNormals = s_scratch->convexPlaneN;
//	const PxVec3* PX_RESTRICT vertices = s_scratch->convexScaledVerts;
//
//	//PxVec3 shapeNormal = shape2Vertex(s_scratch->triangleLocNormal, convexScale, convexScaleRot);
//	PxVec3 shapeNormal = s_scratch->triangleLocNormal;
//
//	const PxReal contactDist = s_scratch->contactDist;
//
//	
//
//	bool isPassed = true;
//	for (PxU32 it = 0; it < numConvexEdges * 3; it += WARP_SIZE)
//	{
//		const PxU32 index = it + threadIdx.x;
//
//		bool testFace = false;
//		PxVec3 shapeSpaceVLocal;
//
//		if (index < numConvexEdges * 3)
//		{
//
//			const PxU32 vStartIndex = index % 3;
//
//			if (isActiveEdge(s_scratch->triAdjTrisIdx[vStartIndex]))
//			{
//
//				const PxU32 vEndIndex = (vStartIndex + 1) % 3;
//
//				PxU32 convexEdgeIdx = index / 3;
//
//				
//
//				const PxU16 fIndices = *(reinterpret_cast<const PxU16*>(facesByEdges8 + 2 * convexEdgeIdx));
//
//				//!WARNING: exploiting little-endianness
//				const PxU8 f10 = u16Low(fIndices);
//				const PxU8 f11 = u16High(fIndices);
//
//
//				if (shapeNormal.dot(planeNormals[f10]) < 0.f || shapeNormal.dot(planeNormals[f11]) < 0.f)
//				{
//					const PxU32 vIndices = *(reinterpret_cast<const PxU32*>(verticesByEdges16 + 2 * convexEdgeIdx));
//					//!WARNING: exploiting little-endianness
//					const PxU16 v0idx1 = u32Low(vIndices);
//					const PxU16 v1idx1 = u32High(vIndices);
//
//					PxVec3	convexEdge = vertices[v0idx1] - vertices[v1idx1];
//
//					const PxVec3 triEdge = (s_scratch->triLocVerts[vEndIndex] - s_scratch->triLocVerts[vStartIndex]);
//					PxVec3 v = convexEdge.cross(triEdge);
//
//					PxReal magSquared = v.magnitudeSquared();
//					//PxReal dp = v.dot(faceNormal);
//
//					if (v.magnitudeSquared() > 1e-10f)
//					{
//						testFace = true;
//						shapeSpaceVLocal = v.getNormalized();
//					}
//				}
//			}
//		}
//
//		PxU32 testMask = __ballot_sync(FULL_MASK, testFace);
//
//		const PxU32 nbTests = __popc(testMask);
//
//		if (testFace)
//			s_scratch->threadIds[warpScanExclusive(testMask, threadIdx.x)] = threadIdx.x;
//
//		__syncwarp();
//
//		
//		const PxU32 LogGroupSize = 3;
//		const PxU32 NbThreadsPerTest = 1<<LogGroupSize;
//		
//		const PxU32 threadIndexInGroup = threadIdx.x & (NbThreadsPerTest - 1);
//
//		const PxU32 Mask = 0xf << ((threadIdx.x/NbThreadsPerTest)*NbThreadsPerTest);
//
//		const PxU32 groupStartIndex = threadIdx.x&(~(NbThreadsPerTest-1));
//
//		for (PxU32 m = 0; m < nbTests; m += NbThreadsPerTest)
//		{
//			PxU32 idx = m + (threadIdx.x / NbThreadsPerTest);
//			PxU32 threadId = 0;
//			if(idx < nbTests)
//				threadId = s_scratch->threadIds[idx];
//			
//
//			PxReal minTri, maxTri;
//			PxReal minConv, maxConv;
//
//			PxVec3 shapeSpaceV;
//			shapeSpaceV.x = __shfl_sync(FULL_MASK, shapeSpaceVLocal.x, threadId);
//			shapeSpaceV.y = __shfl_sync(FULL_MASK, shapeSpaceVLocal.y, threadId);
//			shapeSpaceV.z = __shfl_sync(FULL_MASK, shapeSpaceVLocal.z, threadId);
//
//			if (idx < nbTests)
//			{
//
//				supportTriangleWarp(s_scratch->triLocVerts, shapeSpaceV, minTri, maxTri, threadIndexInGroup, Mask,
//					groupStartIndex);
//
//				supportConvexWarp<LogGroupSize, NbThreadsPerTest>(vertices, numVertices, shapeSpaceV, minConv, maxConv, threadIndexInGroup, Mask);
//
//				PxReal tempOverlap0 = minTri - maxConv;
//				PxReal tempOverlap1 = minConv - maxTri;
//
//				if (minConv > (maxTri + contactDist) || minTri > (maxConv + contactDist))
//				{
//					isPassed = false;
//				}
//				else
//				{
//					if (_separation < tempOverlap0)
//					{
//						_separation = tempOverlap0;
//						_faceNormal = shapeSpaceV;
//					}
//
//					if (_separation < tempOverlap1)
//					{
//						_separation = tempOverlap1;
//						_faceNormal = -shapeSpaceV;
//					}
//				}
//			}
//
//			if (__any_sync(FULL_MASK, !isPassed))
//			{
//				return false;
//			}
//		}
//
//		
//	}
//
//	//printf("%i tests out of %i\n", tests, numConvexEdges * 3);
//
//	PxU32 winnerLane;
//	_separation = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, _separation, winnerLane);
//
//	//unsigned mask_separation = __ballot_sync(syncMask, separation < _separation);
//	if (separation < _separation)
//	{
//		_faceNormal = shuffle(FULL_MASK, _faceNormal, winnerLane);
//		faceNormal = _faceNormal;
//		separation = _separation;
//		feature = EDGE_FEATURE_IDX;
//	}
//
//	return true;
//}

template <bool doEdgeEdge>
__device__ inline static bool convexMeshSAT(ConvexScratch* s_scratch, PxU32 & featureIndex, PxReal & separation, PxVec3 & minNormal)
{
	bool passedSAT;

	passedSAT = satConvexFaceNormals(
		s_scratch,
		featureIndex, minNormal, separation
	);

	__syncwarp(); //satConvexFaceNormals writes into s_scratch. A sync is required to avoid data races

	if (!passedSAT)
	{
		return false;
	}

	passedSAT = satTriangleNormal(
		s_scratch,
		featureIndex, minNormal, separation
	);

	if (!passedSAT)
	{
		return false;
	}

	if (doEdgeEdge)
	{
		passedSAT = satEdgeCrossDirection2(
			s_scratch,
			featureIndex, minNormal, separation
		);

		if (!passedSAT)
		{
			return false;
		}
	}

	return true;
}

// points are generated where 
// * a vertex of one poly is inside the other, including on an edge  (nb coincident verts generate two contacts)   
// * two poly edges, not including their endpoints, cross.
//
//
// the early outs are not actually an optimization, but have a functional purpose in an edge case: with two polys 
// exactly on top of one another, we don't want to generate the points multiple times

__device__ __forceinline__ void addContacts(int flags, PxVec3 pos, PxReal sep, PxReal maxSep, volatile float * s_contactsTransposed, int& nbContacts)
{
	flags &= __ballot_sync(FULL_MASK, !(sep > maxSep)); //Inverting test covers us for nans

	int index = warpScanExclusive(flags, threadIdx.x) + nbContacts;

	bool add = flags & (1 << threadIdx.x) && index < NUM_TMP_CONTACTS_PER_PAIR;

	if (add)
	{
		s_contactsTransposed[4 * index + 0] = pos.x;
		s_contactsTransposed[4 * index + 1] = pos.y;
		s_contactsTransposed[4 * index + 2] = pos.z;
		s_contactsTransposed[4 * index + 3] = sep;

		assert(PxIsFinite(pos.x));
		assert(PxIsFinite(pos.y));
		assert(PxIsFinite(pos.z));
		assert(PxIsFinite(sep));
	}

	nbContacts += __popc(__ballot_sync(FULL_MASK, add));
}

//plane0 is convex, plane1 is triangle
__device__ static int polyClip(const PxPlane plane0, PxVec3 v0, PxU32 nbVerts0, const PxPlane plane1, PxVec3 v1, const PxVec3 axis, PxReal maxSep, PxU32 triEdgeMask,
	volatile float * s_contactsTransposed, PxU32 initContacts)
{
	int tI = threadIdx.x;

	//ML: cosTheta = plane1.n.dot(axis), projectionDist = -plane1.distance(0) / cosTheta;
	/*PxReal s0 = -plane1.distance(v0) / plane1.n.dot(axis);
	PxReal s1 = plane0.distance(v1) / plane0.n.dot(axis);*/

	const PxReal dnom0 = plane1.n.dot(axis);
	const PxReal dnom1 = plane0.n.dot(axis);
	const PxReal t0 = plane1.distance(v0);
	const PxReal t1 = plane0.distance(v1);

	//ML: if the distance is over the other poly plane and is large than contact dist, we need to get rid of that contact. Otherwise, we will have phantom contacts 
	const PxReal s0 = ((t0 > maxSep) || (dnom0 == 0.f)) ? PX_MAX_F32 : -t0 / dnom0;
	const PxReal s1 = ((t1 > maxSep) || (dnom1 == 0.f)) ? PX_MAX_F32 : t1 / dnom1;

	int in0 = 0, out0 = 0;	// each element represents a point of poly 0 - the nth bit is status regarding the nth plane of poly0
	int in1 = 0, out1 = 0;	// each element represents a plane of poly 1 - the nth bit is status regarding the nth point of poly1

	int nbContacts = initContacts;

	// points of poly1 against planes of poly 0

	PxVec3 e0 = shuffle(FULL_MASK, v0, tI + 1 == nbVerts0 ? 0 : tI + 1) - v0;
	PxVec3 e0planeN = e0.cross(axis);
	PxReal e0planeD = e0planeN.dot(v0);

	//ML: in each thread, each triangle vert test against the same plane of poly0, then set the bit with the corresponding lane
	for (int j = 0, bj = 1; j < 3; j++, bj <<= 1)
	{
		//shuffleDot take different triangle vert to calculate the distance
		PxReal t = shuffleDot(FULL_MASK, v1, j, e0planeN) - e0planeD;
		in1 |= (t < 0 ? bj : 0), out1 |= (t > 0 ? bj : 0);
	}

	if (tI >= nbVerts0)
		in1 = out1 = 0;

	int b = ~out1 & __shfl_xor_sync(FULL_MASK, ~out1, 16);
	b &= __shfl_xor_sync(FULL_MASK, b, 8);
	b &= __shfl_xor_sync(FULL_MASK, b, 4);
	b &= __shfl_xor_sync(FULL_MASK, b, 2);
	b &= __shfl_xor_sync(FULL_MASK, b, 1);

	//ML: if b is the bit map for the 3 verts from the triangles, We need to keep the last 3 bits in b for the 3 verts and clear all other bits
	addContacts(b&7, v1 - s1*axis, s1, maxSep, s_contactsTransposed, nbContacts);

	if (!__any_sync(FULL_MASK, out1))										// all poly1 points inside poly0, so done
		return nbContacts;

	in1 |= in1 << 3, out1 |= out1 << 3;			// save poly1's edge crosses
	int e1cross = ((in1 & (out1 >> 1)) | ((in1 >> 1) & out1)) & 7; 
	//e1cross &= triEdgeMask;

	// points of poly0 against planes of poly 1

	PxVec3 e1 = shuffle(FULL_MASK, v1, tI + 1 == 3 ? 0 : tI + 1) - v1;
	PxVec3 e1planeN = axis.cross(e1);
	PxReal e1planeD = e1planeN.dot(v1);

	for (int j = 0; j<nbVerts0; j++)
	{
		PxReal t = shuffleDot(FULL_MASK, v0, j, e1planeN) - e1planeD;
		int in = __ballot_sync(FULL_MASK, t < 0), out = __ballot_sync(FULL_MASK, t > 0);
		if (j == tI)
			in0 = in, out0 = out;
	}

	in0 &= (1 << 3) - 1, out0 &= (1 << 3) - 1;
	addContacts(__ballot_sync(FULL_MASK, !out0) & ((1 << nbVerts0) - 1), v0, s0, maxSep, s_contactsTransposed, nbContacts);

	if (!__any_sync(FULL_MASK, out0))										// all poly0 points inside poly1, so done
		return nbContacts;

	int in0p = __shfl_sync(FULL_MASK, in0, tI + 1 == nbVerts0 ? 0 : tI + 1);
	int out0p = __shfl_sync(FULL_MASK, out0, tI + 1 == nbVerts0 ? 0 : tI + 1);
	int e0cross = (in0 & out0p) | (in0p & out0);			// get poly0's edge crosses

	// edge-edge crossings
	int c = e0cross & e1cross;
	assert(c + 1 < 1 << 3);

	PxReal s0d = __shfl_sync(FULL_MASK, s0, tI + 1 == nbVerts0 ? 0 : tI + 1) - s0;

	int a = c | __shfl_xor_sync(FULL_MASK, c, 16);
	a |= __shfl_xor_sync(FULL_MASK, a, 8);
	a |= __shfl_xor_sync(FULL_MASK, a, 4);
	a |= __shfl_xor_sync(FULL_MASK, a, 2);
	a |= __shfl_xor_sync(FULL_MASK, a, 1);

	for (; a; a &= (a - 1))
	{
		int index = lowestSetIndex(a);
		PxVec3 m = shuffle(FULL_MASK, e1planeN, index);
	/*	PxReal lambda = -(m.dot(v0) - shfl(e1planeD, index)) / m.dot(e0);
		const PxReal sep = s0 + lambda*s0d ;*/

		//do ray trace from an edge of the polygon to a triangle plane
		const PxReal denom = m.dot(e0);
		PxVec3 triVert0 = shuffle(FULL_MASK, v1, index);
		PxReal lambda = (triVert0 - v0).dot(m) / denom;
		const PxVec3 p = v0 + lambda*e0;
		//calculate the projection point to the triangle
		const PxReal t = -(p - triVert0).dot(plane1.n);
	
		//ML: if the distance is over the other poly plane and is large than contact dist, we need to get rid of that contact. Otherwise, we will have phantom contacts 
		//dnom0 = plane1.n.dot(axis)
		const PxReal sep = ((denom == 0.f || dnom0 == 0.f)) ? PX_MAX_F32 : t / dnom0;
		
		addContacts(__ballot_sync(FULL_MASK, c & lowestSetBit(a)),p, sep, maxSep, s_contactsTransposed, nbContacts);
	}

	return nbContacts;
}

////plane0 is convex, plane1 is triangle
//__device__ static int polyClip(const PxPlane plane0, PxVec3 v0, PxU32 nbVerts0, const PxPlane plane1, PxVec3 v1, const PxVec3 axis, PxReal maxSep, PxU32 triEdgeMask,
//	volatile float * s_contactsTransposed, PxU32 initContacts, PxU32 triangleIndex)
//{
//	int tI = threadIdx.x;
//
//	//ML: cosTheta = plane1.n.dot(axis), projectionDist = -plane1.distance(0) / cosTheta;
//	/*PxReal s0 = -plane1.distance(v0) / plane1.n.dot(axis);
//	PxReal s1 = plane0.distance(v1) / plane0.n.dot(axis);*/
//
//	const PxReal dnom0 = plane1.n.dot(axis);
//	const PxReal dnom1 = plane0.n.dot(axis);
//	const PxReal t0 = plane1.distance(v0);
//	const PxReal t1 = plane0.distance(v1);
//
//	//ML: if the distance is over the other poly plane and is large than contact dist, we need to get rid of that contact. Otherwise, we will have phantom contacts 
//	const PxReal s0 = ((t0 > maxSep) || (dnom0 == 0.f)) ? PX_MAX_F32 : -t0 / dnom0;
//	const PxReal s1 = ((t1 > maxSep) || (dnom1 == 0.f)) ? PX_MAX_F32 : t1 / dnom1;
//
//	int in0 = 0, out0 = 0;	// each element represents a point of poly 0 - the nth bit is status regarding the nth plane of poly0
//	int in1 = 0, out1 = 0;	// each element represents a plane of poly 1 - the nth bit is status regarding the nth point of poly1
//
//	int nbContacts = initContacts;
//
//	// points of poly1 against planes of poly 0
//
//	PxVec3 e0 = shuffle(FULL_MASK, v0, tI + 1 == nbVerts0 ? 0 : tI + 1) - v0;
//	PxVec3 e0planeN = e0.cross(axis);
//	PxReal e0planeD = e0planeN.dot(v0);
//
//	//ML: in each thread, each triangle vert test against the same plane of poly0, then set the bit with the corresponding lane
//	for (int j = 0, bj = 1; j < 3; j++, bj <<= 1)
//	{
//		//shuffleDot take different triangle vert to calculate the distance
//		PxReal t = shuffleDot(FULL_MASK, v1, j, e0planeN) - e0planeD;
//		in1 |= (t <= 0 ? bj : 0), out1 |= (t > 0 ? bj : 0);
//	}
//
//	if (tI >= nbVerts0)
//		in1 = out1 = 0;
//
//	int b = ~out1 & __shfl_xor_sync(FULL_MASK, ~out1, 16);
//	b &= __shfl_xor_sync(FULL_MASK, b, 8);
//	b &= __shfl_xor_sync(FULL_MASK, b, 4);
//	b &= __shfl_xor_sync(FULL_MASK, b, 2);
//	b &= __shfl_xor_sync(FULL_MASK, b, 1);
//
//	//ML: if b is the bit map for the 3 verts from the triangles, We need to keep the last 3 bits in b for the 3 verts and clear all other bits
//	addContacts(b & 7, v1 - s1 * axis, s1, maxSep, s_contactsTransposed, nbContacts);
//
//	// all poly1 points inside poly0 or all points are outside the same plane
//	if (!__any_sync(FULL_MASK, out1) || __all_sync(FULL_MASK, out1 == 7))
//	{
//		if (triangleIndex == 5935 && threadIdx.x == 0)
//		{
//			printf("Exiting - out1 = %x\n", out1);
//		}
//		return nbContacts;
//	}
//
//	in1 |= in1 << 3, out1 |= out1 << 3;			// save poly1's edge crosses
//	int e1cross = ((in1 & (out1 >> 1)) | ((in1 >> 1) & out1)) & 7;
//	//e1cross &= triEdgeMask;
//	if (triangleIndex == 5935 && tI < nbVerts0)
//	{
//		printf("%i: in1 = %x, out1 %x, e1cross %x\n", threadIdx.x, in1, out1, e1cross);
//	}
//
//	// points of poly0 against planes of poly 1
//
//	PxVec3 e1 = shuffle(FULL_MASK, v1, tI + 1 == 3 ? 0 : tI + 1) - v1;
//	PxVec3 e1planeN = axis.cross(e1);
//	PxReal e1planeD = e1planeN.dot(v1);
//
//	for (int j = 0; j < nbVerts0; j++)
//	{
//		PxReal t = shuffleDot(FULL_MASK, v0, j, e1planeN) - e1planeD;
//		int in = __ballot_sync(FULL_MASK, t <= 0), out = __ballot_sync(FULL_MASK, t > 0);
//
//		if (triangleIndex == 5935 && tI < 3)
//		{
//			printf("%i: e1PlaneN = (%f, %f, %f), e1PlaneD = %f, t = %f\n", tI, e1planeN.x, e1planeN.y, e1planeN.z,
//				e1planeD, t);
//		}
//
//		if (j == tI)
//			in0 = in, out0 = out;
//	}
//
//	in0 &= (1 << 3) - 1, out0 &= (1 << 3) - 1;
//	addContacts(__ballot_sync(FULL_MASK, !out0) & ((1 << nbVerts0) - 1), v0, s0, maxSep, s_contactsTransposed, nbContacts);
//
//	if (!__any_sync(FULL_MASK, out0))										// all poly0 points inside poly1, so done
//	{
//		if (triangleIndex == 5935 && threadIdx.x == 0)
//		{
//			printf("Exiting pass 2 - out0 = %x\n", out0);
//		}
//		return nbContacts;
//	}
//
//	int in0p = __shfl_sync(FULL_MASK, in0, tI + 1 == nbVerts0 ? 0 : tI + 1);
//	int out0p = __shfl_sync(FULL_MASK, out0, tI + 1 == nbVerts0 ? 0 : tI + 1);
//	int e0cross = (in0 & out0p) | (in0p & out0);			// get poly0's edge crosses
//
//	if (triangleIndex == 5935 && tI < nbVerts0)
//	{
//		printf("%i: in0p = %x, out0p %x, e0cross %x\n", threadIdx.x, in0p, out0p, e0cross);
//	}
//
//	// edge-edge crossings
//	int c = e0cross & e1cross;
//	//assert(c + 1 < 1 << 3);
//
//	PxReal s0d = __shfl_sync(FULL_MASK, s0, tI + 1 == nbVerts0 ? 0 : tI + 1) - s0;
//
//	int a = c | __shfl_xor_sync(FULL_MASK, c, 16);
//	a |= __shfl_xor_sync(FULL_MASK, a, 8);
//	a |= __shfl_xor_sync(FULL_MASK, a, 4);
//	a |= __shfl_xor_sync(FULL_MASK, a, 2);
//	a |= __shfl_xor_sync(FULL_MASK, a, 1);
//
//	if (triangleIndex == 5935)
//	{
//		printf("%i: Entering e-e case: a = %x, e0cross %x, e1cross %x\n", threadIdx.x, a, e0cross, e1cross);
//	}
//
//	for (; a; a &= (a - 1))
//	{
//		int index = lowestSetIndex(a);
//		PxVec3 m = shuffle(FULL_MASK, e1planeN, index);
//		/*	PxReal lambda = -(m.dot(v0) - shfl(e1planeD, index)) / m.dot(e0);
//			const PxReal sep = s0 + lambda*s0d ;*/
//
//			//do ray trace from an edge of the polygon to a triangle plane
//		const PxReal denom = m.dot(e0);
//		PxVec3 triVert0 = shuffle(FULL_MASK, v1, index);
//		PxReal lambda = (triVert0 - v0).dot(m) / denom;
//		const PxVec3 p = v0 + lambda * e0;
//		//calculate the projection point to the triangle
//		const PxReal t = -(p - triVert0).dot(plane1.n);
//
//		//ML: if the distance is over the other poly plane and is large than contact dist, we need to get rid of that contact. Otherwise, we will have phantom contacts 
//		//dnom0 = plane1.n.dot(axis)
//		const PxReal sep = ((t < -maxSep) || (denom == 0.f || dnom0 == 0.f)) ? PX_MAX_F32 : t / dnom0;
//
//		addContacts(__ballot_sync(FULL_MASK, c & lowestSetBit(a)), p, sep, maxSep, s_contactsTransposed, nbContacts);
//	}
//
//	return nbContacts;
//}

////plane0 is convex, plane1 is triangle
//__device__ static int polyClip(const PxPlane plane0, PxVec3 v0, PxU32 nbVerts0, const PxPlane plane1, PxVec3 v1, const PxVec3 axis, PxReal maxSep, PxU32 triEdgeMask,
//	volatile float * s_contactsTransposed, PxU32 initContacts, PxU32 triangleIndex)
//{
//	int tI = threadIdx.x;
//
//	//ML: cosTheta = plane1.n.dot(axis), projectionDist = -plane1.distance(0) / cosTheta;
//	/*PxReal s0 = -plane1.distance(v0) / plane1.n.dot(axis);
//	PxReal s1 = plane0.distance(v1) / plane0.n.dot(axis);*/
//
//
//	const PxReal dnom0 = plane1.n.dot(axis);
//	const PxReal dnom1 = plane0.n.dot(axis);
//	const PxReal t0 = plane1.distance(v0);
//	const PxReal t1 = plane0.distance(v1);
//
//	//ML: if the distance is over the other poly plane and is large than contact dist, we need to get rid of that contact. Otherwise, we will have phantom contacts 
//	/*const PxReal s0 = ((t0 > maxSep) || (dnom0 == 0.f)) ? PX_MAX_F32 : -t0 / dnom0;
//	const PxReal s1 = ((t1 > maxSep) || (dnom1 == 0.f)) ? PX_MAX_F32 : t1 / dnom1;*/
//
//	const PxReal s0 = ((dnom0 == 0.f)) ? 1e+10f : -t0 / dnom0;
//	const PxReal s1 = ((dnom1 == 0.f)) ? 1e+10f : t1 / dnom1;
//
//	
//
//	int in0 = 0, out0 = 0;	// each element represents a point of poly 0 - the nth bit is status regarding the nth plane of poly0
//	int in1 = 0, out1 = 0;	// each element represents a plane of poly 1 - the nth bit is status regarding the nth point of poly1
//
//	int nbContacts = initContacts;
//
//	// points of poly1 against planes of poly 0
//
//	PxVec3 e0 = shuffle(FULL_MASK, v0, tI + 1 == nbVerts0 ? 0 : tI + 1) - v0;
//	PxVec3 e0planeN = e0.cross(axis);
//	PxReal e0planeD = e0planeN.dot(v0);
//
//	//ML: in each thread, each triangle vert test against the same plane of poly0, then set the bit with the corresponding lane
//	for (int j = 0, bj = 1; j < 3; j++, bj <<= 1)
//	{
//		//shuffleDot take different triangle vert to calculate the distance
//		PxReal t = shuffleDot(FULL_MASK, v1, j, e0planeN) - e0planeD;
//		in1 |= (t <= 0 ? bj : 0), out1 |= (t > 0 ? bj : 0);
//	}
//
//	if (tI >= nbVerts0)
//		in1 = out1 = 0;
//
//	int b = ~out1 & __shfl_xor_sync(FULL_MASK, ~out1, 16);
//	b &= __shfl_xor_sync(FULL_MASK, b, 8);
//	b &= __shfl_xor_sync(FULL_MASK, b, 4);
//	b &= __shfl_xor_sync(FULL_MASK, b, 2);
//	b &= __shfl_xor_sync(FULL_MASK, b, 1);
//
//	//ML: if b is the bit map for the 3 verts from the triangles, We need to keep the last 3 bits in b for the 3 verts and clear all other bits
//	addContacts(b & 7, v1 - s1 * axis, s1, maxSep, s_contactsTransposed, nbContacts);
//
//	// all poly1 points inside poly0 or all points are outside the same plane
//	if (!__any_sync(FULL_MASK, out1))
//	{
//		if (triangleIndex == 5935 && threadIdx.x == 0)
//		{
//			printf("Exiting - out1 = %x\n", out1);
//		}
//		return nbContacts;
//	}
//
//	in1 |= in1 << 3, out1 |= out1 << 3;			// save poly1's edge crosses
//	int e1cross = ((in1 & (out1 >> 1)) | ((in1 >> 1) & out1)) & 7;
//	//e1cross &= triEdgeMask;
//	if (triangleIndex == 5935 && tI < nbVerts0)
//	{
//		printf("%i: in1 = %x, out1 %x, e1cross %x\n", threadIdx.x, in1, out1, e1cross);
//	}
//
//	// points of poly0 against planes of poly 1
//
//	PxVec3 e1 = shuffle(FULL_MASK, v1, tI + 1 == 3 ? 0 : tI + 1) - v1;
//	PxVec3 e1planeN = axis.cross(e1);
//	PxReal e1planeD = e1planeN.dot(v1);
//
//	for (int j = 0; j < nbVerts0; j++)
//	{
//		PxReal t = shuffleDot(FULL_MASK, v0, j, e1planeN) - e1planeD;
//		int in = __ballot_sync(FULL_MASK, t <= 0), out = __ballot_sync(FULL_MASK, t > 0);
//
//		if (triangleIndex == 5935 && tI < 3)
//		{
//			printf("%i: e1PlaneN = (%f, %f, %f), e1PlaneD = %f, t = %f\n", tI, e1planeN.x, e1planeN.y, e1planeN.z,
//				e1planeD, t);
//		}
//
//		if (j == tI)
//			in0 = in, out0 = out;
//	}
//
//	in0 &= (1 << 3) - 1, out0 &= (1 << 3) - 1;
//	addContacts(__ballot_sync(FULL_MASK, !out0) & ((1 << nbVerts0) - 1), v0, s0, maxSep, s_contactsTransposed, nbContacts);
//
//	if (!__any_sync(FULL_MASK, out0))										// all poly0 points inside poly1, so done
//	{
//		if (triangleIndex == 5935 && threadIdx.x == 0)
//		{
//			printf("Exiting pass 2 - out0 = %x\n", out0);
//		}
//		return nbContacts;
//	}
//
//	int in0p = __shfl_sync(FULL_MASK, in0, tI + 1 == nbVerts0 ? 0 : tI + 1);
//	int out0p = __shfl_sync(FULL_MASK, out0, tI + 1 == nbVerts0 ? 0 : tI + 1);
//
//	PxReal s0p = __shfl_sync(FULL_MASK, s0, tI + 1 == nbVerts0 ? 0 : tI + 1);
//	PxVec3 v0p = shuffle(FULL_MASK, v0, tI + 1 == nbVerts0 ? 0 : tI + 1);
//
//	PxReal minP = -PX_MAX_F32;
//	PxReal maxP = PX_MAX_F32;
//	PxU32 minAxis = 0;
//	PxU32 maxAxis = 0;
//
//	PxVec3 dir = v0p - v0;
//
//	//for (; a; a &= (a - 1))
//	for (PxU32 index = 0; index < 3; ++index)
//	{
//		//int index = lowestSetIndex(a);
//		PxVec3 m = shuffle(FULL_MASK, e1planeN, index);
//		PxReal d = __shfl_sync(FULL_MASK, e1planeD, index);
//
//		PxReal p0 = v0.dot(m) - d;
//		PxReal p0p = v0p.dot(m) - d;
//
//		//if ((p0*p0p) < 0.f)
//		{
//			//Swap sides...
//			
//			PxReal dif = (p0 - p0p);
//			if (PxAbs(dif) > 1e-8f)
//			{
//				PxReal proj = m.dot(dir);
//				PxReal t = p0 / (p0 - p0p);
//				if (proj < 0.f)
//				{
//					minP = PxMax(t, minP);
//					minAxis = index;
//				}
//				else
//				{
//					maxP = PxMin(t, maxP);
//					maxAxis = index;
//				}
//			}
//		}
//	}
//
//	
//
//	//Now we have a hit (minP/maxP), we record those hits...
//	PxVec3 pos0 = v0 + (v0p - v0)*minP;
//	PxVec3 triV0 = shuffle(FULL_MASK, v1, minAxis);
//	PxVec3 triE0 = shuffle(FULL_MASK, e1, minAxis).getNormalized();
//	PxVec3 triPos0 = triV0 + triE0 * (pos0 - triV0).dot(triE0);
//	PxReal sep0 = (triV0 - pos0).dot(axis);
//	
//	//PxReal sep0 = s0 + (s0p - s0)*minP;
//	PxVec3 pos1 = v0 + (v0p - v0)*maxP;
//	PxVec3 triV1 = shuffle(FULL_MASK, v1, maxAxis);
//	PxVec3 triE1 = shuffle(FULL_MASK, e1, maxAxis).getNormalized();
//	PxVec3 triPos1 = triV1 + triE1 * (pos1 - triV1).dot(triE1);
//	PxReal sep1 = (triV1 - pos1).dot(axis);
//	//PxReal sep1 = s0 + (s0p - s0)*minP;
//
//	bool hasHit0 = minP > 0.f && minP < maxP && tI < nbVerts0;
//	bool hasHit1 = maxP > 0.f && maxP < 1.f && minP < maxP && tI < nbVerts0;
//
//	if (hasHit0)
//	{
//		printf("%i: Min hit Hit (%f, %f, %f)sep(%f), s0(%f), s0p(%f), minP = %f, maxP = %f\n", tI, pos0.x, pos0.y, pos0.z, sep0, s0, s0p, minP, maxP);
//	}
//
//	if (hasHit1)
//	{
//		printf("%i: Max Hit (%f, %f, %f)sep(%f)s0(%f), s0p(%f), minP = %f, maxP = %f\n", tI, pos1.x, pos1.y, pos1.z, sep1, s0, s0p, minP, maxP);
//	}
//
//	addContacts(__ballot_sync(FULL_MASK, hasHit0), pos0, sep0, maxSep, s_contactsTransposed, nbContacts);
//	addContacts(__ballot_sync(FULL_MASK, hasHit1), pos1, sep1, maxSep, s_contactsTransposed, nbContacts);
//
//
//	return nbContacts;
//}

static __device__ bool delayGenerateContact(const ConvexMeshScratch* s_scratch, const PxVec3& minNormal, const PxU32 featureIndex)
{
	//this means the minimum seperating axis is the face normal of one of polygons in the convex hull
	if (featureIndex != EDGE_FEATURE_IDX && featureIndex != TRI_FEATURE_IDX)
	{
		//check to see whether we need to do post process for this convex hull. This means we don't generate contacts
		//for this convex hull and triangle and push this pair to the second pass buffer
		PxVec3 triLocNormal = -s_scratch->triangleLocNormal;
		PxReal cosTheta = minNormal.dot(triLocNormal);
		const PxReal threshold = 0.707106781f;//about 45 degree
		//const PxReal threshold = 0.985f;//about 10 degree

		return cosTheta < threshold;
	}

	return false;
}

static __device__ PxU32 selectPolygonFeatureIndex(const ConvexScratch* s_scratch, const PxVec3& minNormal, const PxU32 featureIndex, PxU32& polyIndex2)
{
	//If featureIndex is one of the polygon face index, we are done. Otherwise, we need to chose
	//polygon face index
	PxU32 polyFaceIndex = featureIndex;

	//unsigned mask_featureIndex = __ballot_sync(syncMask, featureIndex == EDGE_FEATURE_IDX || featureIndex == TRI_FEATURE_IDX);
	if (featureIndex == TRI_FEATURE_IDX || featureIndex == EDGE_FEATURE_IDX)
	{
		const PxVec3 convexScale = s_scratch->convexScale;
		const PxQuat convexScaleRot = s_scratch->convexScaleRot;

		const PxU32 nbPolygons0 = getNbPolygons(s_scratch->nbEdgesNbHullVerticesNbPolygons);

		PxReal minProj = -PX_MAX_REAL;

		PxU32 bestConvexFaceIndex = 0;

		const PxVec3* planeNs = s_scratch->convexPlaneN;
		for (PxU32 i = threadIdx.x; i < nbPolygons0; i += WARP_SIZE)
		{
			const PxReal proj = minNormal.dot(planeNs[i]);
			if (minProj < proj)
			{
				minProj = proj;
				bestConvexFaceIndex = (PxI32)i;
			}
		}

		PxU32 winnerLane;
		minProj = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, minProj, winnerLane);
		polyFaceIndex = __shfl_sync(FULL_MASK, bestConvexFaceIndex, winnerLane);

		//Now we loop over edges to find out whether there is an edge normal that is better. If there is, we choose the faces that contribute to
		//that edge. In this case, we will generate contacts with *two faces rather than one*.
		// PT: additionally we also now look for the "best" neighbor from the face we previously retrieved (polyFaceIndex). The best neighbor is
		// the one whose normal is closest to the one from polyFaceIndex. The idea here is that we're going to also return "two faces rather than one"
		// from the face-contact codepath, in order to make it more robust. Without doing so we found cases were the contact generation flipped from
		// one face to its neighbor and vice versa, producing jitter. The problem disappears, or at least is greatly reduced, if we just output the
		// two faces directly.
		const PxU32 nbEdges = getNbEdges(s_scratch->nbEdgesNbHullVerticesNbPolygons);

		PxU32 closestEdge = 0xFFFFFFFF;
		PxReal maxDpSq = minProj*minProj;

		PxReal maxDpSq2 = -10.0f;

		//if(threadIdx.x == 0)
		//	printf("%i: BestConvexFaceIndex = %i, minProj = %f, maxDpSq = %f, n = (%f, %f, %f)\n", threadIdx.y, polyFaceIndex, minProj, maxDpSq,
		//		featureNormal_vertexSpace.x, featureNormal_vertexSpace.y, featureNormal_vertexSpace.z);

		const PxU8* facesByEdges8 = s_scratch->getFacesByEdges8();
		for (PxU32 i = threadIdx.x; i < nbEdges; i += WARP_SIZE)
		{
			const PxU32 index = i * 2;
			const PxU8 f0 = facesByEdges8[index];
			const PxU8 f1 = facesByEdges8[index+1];

			const PxVec3 edgeN = planeNs[f0] + planeNs[f1];
			const PxReal enMagSq = edgeN.dot(edgeN);
			const PxReal dp = edgeN.dot(minNormal);
			const PxReal sqDp = dp * dp;

			if((f0==polyFaceIndex || f1==polyFaceIndex) && (sqDp > maxDpSq2))
			{
				maxDpSq2 = sqDp;
				polyIndex2 = f0==polyFaceIndex ? PxU32(f1) : PxU32(f0);
			}

			if (dp > 0.f && sqDp > (maxDpSq*enMagSq))
			{
				maxDpSq = sqDp / enMagSq;
				closestEdge = i;

				//printf("ClosestEdge = %i, maxDpSq = %f\n", closestEdge, maxDpSq);
			}
		}

		//Reduce to get answer...
		maxDpSq = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, maxDpSq, winnerLane);
		closestEdge = __shfl_sync(FULL_MASK, closestEdge, winnerLane);

		if (closestEdge != 0xFFFFFFFF)
		{
			//We found a better edge than the faces, so we pick both faces from the edge...
			const PxU32 index = closestEdge * 2;
			const PxU8 f0 = facesByEdges8[index];
			const PxU8 f1 = facesByEdges8[index + 1];

			const PxReal dp0 = planeNs[f0].dot(minNormal);
			const PxReal dp1 = planeNs[f1].dot(minNormal);

			if (dp0 > dp1)
			{
				polyFaceIndex = f0;
				polyIndex2 = f1;
			}
			else
			{
				polyFaceIndex = f1;
				polyIndex2 = f0;
			}
		}
		else
		{
			// PT: if we found an edge contact we end up in the other codepath, and the two
			// faces we output are "both faces from the edge". In this case we don't need to
			// worry about the "best neighbor" from polyFaceIndex and there is nothing to do.
			// In this codepath however we don't have an edge-contact and we need to retrieve
			// the best "polyIndex2" result from all participating threads.
			maxDpSq2 = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, maxDpSq2, winnerLane);
			polyIndex2 = __shfl_sync(FULL_MASK, polyIndex2, winnerLane);
		}
	}

	return polyFaceIndex;
}

static __device__ void barycentricCoordinates(const PxVec3& p, const PxVec3& a, const PxVec3& b, const PxVec3& c, PxReal& v, PxReal& w)
{
	const PxVec3 ab = b - a;
	const PxVec3 ac = c - a;

	const PxVec3 n = ab.cross(ac);

	const PxVec3 ap = a - p;
	const PxVec3 bp = b - p;
	const PxVec3 cp = c - p;

	const PxVec3 bCrossC = bp.cross(cp);
	const PxVec3 cCrossA = cp.cross(ap);
	const PxVec3 aCrossB = ap.cross(bp);

	const PxReal va = n.dot(bCrossC);//edge region of BC, signed area rbc, u = S(rbc)/S(abc) for a
	const PxReal vb = n.dot(cCrossA);//edge region of AC, signed area rac, v = S(rca)/S(abc) for b
	const PxReal vc = n.dot(aCrossB);//edge region of AB, signed area rab, w = S(rab)/S(abc) for c
	const PxReal totalArea = va + vb + vc;
	const PxReal denom = totalArea == 0.f ? 0.f : 1.f/totalArea;
	v = vb * denom;
	w = vc * denom;
}

static __device__ PxU32 getTriangleActiveMask(const PxReal v, const PxReal w)
{
	PxU32 mask = 0;

	const PxReal upperBound = 0.97f;
	const PxReal u = 1.f - v - w;

	if (v > upperBound)
	{
		mask = PxU32(ConvexTriIntermediateData::eV1);
	}
	else if (w > upperBound)
	{
		mask = PxU32(ConvexTriIntermediateData::eV2);
	}
	else if (u > upperBound)
	{
		mask = PxU32(ConvexTriIntermediateData::eV0);
	}
	else if ((v + u) > upperBound)
	{
		//0-1 edge
		mask = PxU32(ConvexTriIntermediateData::eE01);
	}
	else if ((u + w) > upperBound)
	{
		//0-2 edge
		mask = PxU32(ConvexTriIntermediateData::eE02);
	}
	else if ((w + v) > upperBound)
	{
		//1-2 edge
		mask = PxU32(ConvexTriIntermediateData::eE12);
	}

	return mask;
}

static __device__ PxU32 getTriangleActiveMaskFromNormal(const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, const PxVec3& n)
{
	const PxVec3 e01 = (v1 - v0).getNormalized();
	const PxVec3 e02 = (v2 - v0).getNormalized();
	const PxVec3 e12 = (v2 - v1).getNormalized();
	const PxReal d01 = e01.dot(n);
	const PxReal d02 = e02.dot(n);
	const PxReal d12 = e12.dot(n);
	const PxVec3 tn = e01.cross(e02).getNormalized();
	const PxReal eps = 0.0174f; // around 89 degree

	if (d01 < -eps && d02 < -eps)
		return ConvexTriIntermediateData::eV0;

	if (d01 > eps && d12 < -eps)
		return ConvexTriIntermediateData::eV1;

	if (d02 > eps && d12 > eps)
		return ConvexTriIntermediateData::eV2;

	if (tn.cross(e01).getNormalized().dot(n) < -eps)
		return ConvexTriIntermediateData::eE01;

	if (tn.cross(e02).getNormalized().dot(n) > eps)
		return ConvexTriIntermediateData::eE02;

	if (tn.cross(e12).getNormalized().dot(n) < -eps)
		return ConvexTriIntermediateData::eE12;

	return 0;
}

static __device__ PxU32 getTriangleActiveMask(const PxVec3& pa, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2)
{
	PxReal v, w;
	barycentricCoordinates(pa, v0, v1, v2, v, w);
	return getTriangleActiveMask(v, w);	
}

//ML: this function reuse the s_scratch memory to store the contacts!!!! s_scrach memory will get trash
static __device__ void convexTriangleContactGen(
	volatile PxU32* s_WarpSharedMemory, ConvexMeshScratch* s_scratch, const PxU32 convexTriPairOffset, const PxU32 convexTriPairOffsetPadded, const PxU32 remapCpuTriangleIdx, const PxU32 triangleIdx,
	const PxU32 globalWarpIndex, ConvexTriNormalAndIndex** PX_RESTRICT cvxTriNIPtr, ConvexTriContacts** PX_RESTRICT cvxTriContactsPtr, PxReal** PX_RESTRICT cvxTriMaxDepthPtr,
	ConvexTriIntermediateData** PX_RESTRICT cvxTriIntermPtr, PxU32** PX_RESTRICT orderedCvxTriIntermPtr, PxU32** PX_RESTRICT convTriSecondPairPassPtr, PxU32* nbSecondPassPairs,
	ConvexTriContact* tempConvexTriContacts, const PxU32 tempContactSize, PxU32* pTempContactIndex)
{
	// this will collide the whole convex against 1 triangle.

	// SAT
	PxReal separation = -PX_MAX_REAL;
	PxVec3 minNormal(PX_MAX_REAL);
	PxU32 featureIndex = 0xffffffff;

	const PxU32 nbVerts = getNbVerts(s_scratch->nbEdgesNbHullVerticesNbPolygons);
	const PxU32 nbPolygons = getNbPolygons(s_scratch->nbEdgesNbHullVerticesNbPolygons);

	assert(nbVerts <= CONVEX_MAX_VERTICES_POLYGONS);
	assert(nbPolygons <= CONVEX_MAX_VERTICES_POLYGONS);

	assert(nbVerts != 0);
	assert(nbPolygons != 0);

	for (PxU32 i = threadIdx.x; i < nbVerts; i += WARP_SIZE)
	{
		float4 v = s_scratch->getVertices()[i];
		s_scratch->convexScaledVerts[i] = vertex2Shape(PxVec3(v.x, v.y, v.z), s_scratch->convexScale, s_scratch->convexScaleRot);
	}

	__syncwarp();

	bool isSATPassed = convexMeshSAT<true>(
		s_scratch,
		featureIndex,
		separation,
		minNormal
		);

	PxReal maxSeparation = PX_MAX_F32;
	PxU32 nbOutputContacts = 0;
	bool delayContacts = false;

	__syncwarp(); //s_scratch->convexPlaneN/D is written in convexMeshSAT and read below

	// WARNING! Contacts re-use same shared memory as intermediate buffers
	volatile float * s_contactsTransposed = reinterpret_cast<volatile float*>(s_WarpSharedMemory);
	PxVec3 verts0, verts1;
	//unsigned mask_isSATPassed = __ballot_sync(syncMask, isSATPassed);
	if (isSATPassed)
	{
		//if delayContacts is true, which means this pair might generate edge contacts and we need to do
		//post process for this pair(check whether the neighbour triangle has already generate contacts with
		//this convex hull. If so, we don't need to generate contacts. Otherwise, generate contacts). 
		//However, this require us to reload all data for the pairs again. This is very expensive to do. 
		//Therefore, we will still generate contacts for this pair and we might need to throw away the contacts
		//after some condition checks
		delayContacts = delayGenerateContact(s_scratch, minNormal, featureIndex);

		// Select best polygon face
		PxU32 faceIndex2 = 0xFFFFFFFF;
		const PxU32 faceIndex1 = selectPolygonFeatureIndex(s_scratch, minNormal, featureIndex, faceIndex2);

		PxU32 numContactsGenerated = 0;

		// Generate contacts
		ConvexTriContacts* PX_RESTRICT cvxTriContacts = *cvxTriContactsPtr;

		PxVec3 convexScale = s_scratch->convexScale;
		PxQuat convexScaleRot = s_scratch->convexScaleRot;

		PxReal orthoEps = 1e-6;

		PxPlane triPlane(s_scratch->triLocVerts[0], s_scratch->triangleLocNormal);

		if (threadIdx.x < 3)
			verts1 = s_scratch->triLocVerts[threadIdx.x];

		const PxVec3* PX_RESTRICT vertices = s_scratch->convexScaledVerts;
		const PxU8* PX_RESTRICT vertexData8 = s_scratch->getVertexData8();

		const PxU32 * PX_RESTRICT polyDescs = s_scratch->getPolyDescs();
		const PxVec3* PX_RESTRICT planeNs = s_scratch->convexPlaneN;
		const PxReal* PX_RESTRICT planeDs = s_scratch->convexPlaneD;

		const PxReal contactDist = s_scratch->contactDist;

		PxPlane plane0(planeNs[faceIndex1], planeDs[faceIndex1]);
		PxPlane plane1;
		if(faceIndex2 != 0xFFFFFFFF)
			plane1 = PxPlane(planeNs[faceIndex2], planeDs[faceIndex2]);

		__syncwarp(); //s_scratch and s_contactsTransposed points to the same shared memory. s_scratch is read above and s_contactsTransposed is written below.

		for (PxU32 faceIndex = faceIndex1; faceIndex != 0xFFFFFFFF;)
		{
			const PxU32 polyDesc = polyDescs[faceIndex];

			//if (fabs(minNormal.dot(plane0.n)) >= orthoEps && fabs(minNormal.dot(triPlane.n)) >= orthoEps)
			bool predicate = (fabs(minNormal.dot(plane0.n)) >= orthoEps && fabs(minNormal.dot(triPlane.n)) >= orthoEps);
			//unsigned mask_predicate = __ballot_sync(FULL_MASK, predicate);

			if (predicate)
			{
				if (threadIdx.x < getNbVerts(polyDesc))
					verts0 = vertices[vertexData8[getVRef8(polyDesc) + threadIdx.x]];

				//what happen to thread 4 to 31? is verts1 garbage????
				assert(getNbVerts(polyDesc) <= 32);

#if 0//EDGE_FILTERING
				//Attention: s_scratch and s_contactsTransposed points to the same shared memory!
				PxU32 triEdgeMask = (s_scratch->triAdjTrisIdx.x != BOUNDARY && (s_scratch->triAdjTrisIdx.x & NONCONVEX_FLAG) ? 0 : 1u) | (s_scratch->triAdjTrisIdx.y != BOUNDARY && (s_scratch->triAdjTrisIdx.y & NONCONVEX_FLAG) ? 0 : 2u) |
					(s_scratch->triAdjTrisIdx.z != BOUNDARY && (s_scratch->triAdjTrisIdx.z & NONCONVEX_FLAG) ? 0 : 4u);
#else
				PxU32 triEdgeMask = 0xFFffFFff;
#endif
				numContactsGenerated = polyClip(plane0, verts0, getNbVerts(polyDesc), triPlane, verts1,
					minNormal, contactDist, triEdgeMask, s_contactsTransposed, numContactsGenerated);

			}

			faceIndex = faceIndex2;
			faceIndex2 = 0xFFFFFFFF;
			plane0 = plane1;
		}

		//Make sure that shared memory writes are visible
		__syncwarp();

		//unsigned mask_numContactsGenerated = __ballot_sync(mask_predicate, numContactsGenerated);
		if (numContactsGenerated)
		{
			PxVec3 pa;
			PxReal sep;

			if (threadIdx.x < numContactsGenerated)
			{
				pa = PxVec3(s_contactsTransposed[4 * threadIdx.x + 0], s_contactsTransposed[4 * threadIdx.x + 1],
					s_contactsTransposed[4 * threadIdx.x + 2]),
					sep = s_contactsTransposed[4 * threadIdx.x + 3];
			}

			int mask = (1 << numContactsGenerated) - 1;

			//unsigned mask_numContactsGeneratedReduce = __ballot_sync(mask_numContactsGenerated, numContactsGenerated > 4);
			/*if (numContactsGenerated > CVX_TRI_MAX_CONTACTS)
				mask = reduce<false, true, CVX_TRI_MAX_CONTACTS>(pa, sep, minNormal, numContactsGenerated);
*/

			if (numContactsGenerated > CVX_TRI_MAX_CONTACTS)
				mask = contactReduce<false, true, CVX_TRI_MAX_CONTACTS>(pa, sep, minNormal, mask, 0.f);
			/*if (threadIdx.x == 0)
				printf("Contact mask = %i\n", mask);*/

			maxSeparation = mask & (1 << threadIdx.x) ? sep : FLT_MAX;
			maxSeparation = warpReduction<MinOpFloat, PxReal>(FULL_MASK, maxSeparation);

			nbOutputContacts = __popc(mask);

			PxU32 startIndex;
			
			if (threadIdx.x == 0)
			{
				startIndex = atomicAdd(pTempContactIndex, nbOutputContacts);
				cvxTriContacts[convexTriPairOffset].index = startIndex;
			}

			startIndex = __shfl_sync(FULL_MASK, startIndex, 0);

			if ((startIndex + nbOutputContacts) < tempContactSize)
			{
				if (mask & (1 << threadIdx.x))
				{
					int index = warpScanExclusive(mask, threadIdx.x);

					float4* contactBuf = reinterpret_cast<float4 *>(tempConvexTriContacts + startIndex);
					//printf("%i: %i: Adding Contact (%f, %f, %f, %f)\n", threadIdx.x, index, pa.x, pa.y, pa.z, sep);
					contactBuf[index] = make_float4(pa.x, pa.y, pa.z, sep);
				}
			}
			else
				nbOutputContacts = 0;
		}

		delayContacts = delayContacts && numContactsGenerated;
	}

	ConvexTriNormalAndIndex* PX_RESTRICT cvxTriNI = *cvxTriNIPtr;
	ConvexTriIntermediateData* PX_RESTRICT cvxTriInterm = *cvxTriIntermPtr;
	PxU32* PX_RESTRICT orderedCvxTriInterm = *orderedCvxTriIntermPtr;

	// assert for correct stack memory allocation
#if PX_DEBUG
	ConvexTriContacts* PX_RESTRICT cvxTriContacts = *cvxTriContactsPtr;
	assert((size_t)&cvxTriContacts[convexTriPairOffset] < (size_t)&orderedCvxTriInterm[0]);
#endif

	// Writing other intermediate outputs

	PxVec3 verts2 = shuffle(FULL_MASK, verts1, 1);
	PxVec3 verts3 = shuffle(FULL_MASK, verts1, 2);
	if (threadIdx.x == 0)
	{
		PxReal* PX_RESTRICT cvxTriMaxDepth = *cvxTriMaxDepthPtr;
		cvxTriMaxDepth[convexTriPairOffset] = maxSeparation;
		// assert for correct stack memory allocation
		assert((size_t)&cvxTriMaxDepth[convexTriPairOffset] < (size_t)&cvxTriInterm[0]);
		
		if (delayContacts)
		{
			PxU32* PX_RESTRICT convTriSecondPairPass = *convTriSecondPairPassPtr;
			const PxU32 startIndex = atomicAdd(nbSecondPassPairs, 1);

			//Need to flag based on barycentrics of the first contact!!!
			
			PxVec3 pa(s_contactsTransposed[0], s_contactsTransposed[1],
				s_contactsTransposed[2]);

			PxU32 mask = getTriangleActiveMask(pa, verts1, verts2, verts3);

			convTriSecondPairPass[startIndex] = globalWarpIndex | mask;

			// assert for correct stack memory allocation
			assert((size_t)&cvxTriInterm[convexTriPairOffset] < (size_t)&convTriSecondPairPass[0]);
		}

		cvxTriInterm[convexTriPairOffset].gpuTriIndex = triangleIdx;
		const PxU32 mask = (nbOutputContacts && !delayContacts) ? 1 << 31 : 0;
		orderedCvxTriInterm[convexTriPairOffsetPadded] = mask | triangleIdx;

		// assert for correct stack memory allocation
		assert((size_t)&orderedCvxTriInterm[convexTriPairOffsetPadded] < (size_t)&cvxTriMaxDepth[0]);
	}

	const PxU32 delayContactMask = delayContacts ? ConvexTriNormalAndIndex::DeferredContactMask : 0;

	ConvexTriNormalAndIndex normalIndex;
	normalIndex.normal = minNormal;
	normalIndex.index = delayContactMask + (nbOutputContacts << ConvexTriNormalAndIndex::NbContactsShift) + remapCpuTriangleIdx;
	ConvexTriNormalAndIndex_WriteWarp(cvxTriNI + convexTriPairOffset, normalIndex);

	// assert for correct stack memory allocation
	assert((size_t)&cvxTriNI[convexTriPairOffset] < (size_t)&cvxTriContacts[0]);
}

#endif
