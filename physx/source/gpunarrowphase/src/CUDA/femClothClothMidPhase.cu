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

#include "foundation/PxBounds3.h"
#include "foundation/PxMath.h"
#include "foundation/PxPlane.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "foundation/PxVec3.h"

#include "geometry/PxGeometry.h"
#include "geometry/PxMeshScale.h"

#include "GuDistancePointTriangle.h"
#include "GuBV32.h"
#include "cudaNpCommon.h"

#include "PxgContactManager.h"
#include "PxgConvexConvexShape.h"
#include "PxgFEMCloth.h"
#include "PxgFEMClothCore.h"
#include "PxgFEMCore.h"
#include "PxgNpKernelIndices.h"
#include "PxgSimulationCoreDesc.h"

#include "PxsTransformCache.h"

#include <vector_types.h>

#include "PxgCommonDefines.h"
#include "dataReadWriteHelper.cuh"
#include "femMidphaseScratch.cuh"
#include "utils.cuh"
#include "deformableElementFilter.cuh"
#include "deformableCollision.cuh"
#include "triangletriangle.cuh"

#include "bv32Traversal.cuh"
#include "triangleMesh.cuh"

using namespace physx;

extern "C" __host__ void initNarrowphaseKernels21() {}

__device__ void computeTriangleBound2(const uint4* const PX_RESTRICT triIndices, const PxU32 triangleIdx, const float4* const PX_RESTRICT triangleVerts,
	PxBounds3& triangleBound)
{
	const uint4 vertInds = triIndices[triangleIdx];

	const PxVec3 worldV0 = PxLoad3(triangleVerts[vertInds.x]);
	const PxVec3 worldV1 = PxLoad3(triangleVerts[vertInds.y]);
	const PxVec3 worldV2 = PxLoad3(triangleVerts[vertInds.z]);

	triangleBound = triBoundingBox(worldV0, worldV1, worldV2);
}


struct TriangleLeafBoundMinMaxTraverser
{
	femMidphaseScratch* PX_RESTRICT s_warpScratch;
	Triangle* mTriangle;
	const PxVec3 tMin;
	PxReal mContactDistance;
	const PxVec3 tMax;
	PxReal mBestDistance;
	const physx::PxU32 triangleIndex;
	bool mIsMesh;
	PxU32 mId0; //If this is mTriangle belong to a mesh, mMask will be oxffffffff. If mTriangle belong to cloth, mId0 will be cloth Id
	PxU32 mId1; //this will be the other cloth Id
	const PxgNonRigidFilterPair* mFilterPairs;
	PxU32 mNumFilterPairs;

	//output
	PxU32 mPrimIndex;

	PX_FORCE_INLINE __device__ TriangleLeafBoundMinMaxTraverser(femMidphaseScratch* PX_RESTRICT s_warpScratch, physx::PxU32 triangleIndex, const PxBounds3& triangleBound,
		Triangle* triangle, const PxReal contactDistance, bool isMesh, const PxU32 id0, const PxU32 id1) :
		s_warpScratch(s_warpScratch), triangleIndex(triangleIndex), tMin(triangleBound.minimum), tMax(triangleBound.maximum), mTriangle(triangle), mContactDistance(contactDistance), mBestDistance(PX_MAX_F32), mIsMesh(isMesh), mId0(id0), mId1(id1),
		mFilterPairs(NULL), mNumFilterPairs(0), mPrimIndex(0xFFFFFFFF)
	{ }

	__device__ void setFiltering(const PxgNonRigidFilterPair* PX_RESTRICT filterPairs, const PxU32 numFilterPairs)
	{
		mFilterPairs = filterPairs;
		mNumFilterPairs = numFilterPairs;
	}

	PX_FORCE_INLINE __device__ void intersectPrimitiveFullWarp(PxU32 primitiveIndex, PxU32 idxInWarp)
	{
		if (primitiveIndex != 0xFFFFFFFF) 
		{
			using namespace physx;

			const uint4* trimeshTriangleIndices = s_warpScratch->meshVertsIndices;
			const float4 * trimeshVerts = s_warpScratch->meshVerts;

			const uint4 triIdx = trimeshTriangleIndices[primitiveIndex];

			const PxVec3 v0InVertexSpace = PxLoad3(trimeshVerts[triIdx.x]);
			const PxVec3 v1InVertexSpace = PxLoad3(trimeshVerts[triIdx.y]);
			const PxVec3 v2InVertexSpace = PxLoad3(trimeshVerts[triIdx.z]);

			Triangle triangle1;
			triangle1.verts[0] = v0InVertexSpace;
			triangle1.verts[1] = v1InVertexSpace;
			triangle1.verts[2] = v2InVertexSpace;
			triangle1.triPlane = PxPlane(v0InVertexSpace, v1InVertexSpace, v2InVertexSpace);

			PxReal separation;

			if (mIsMesh)
			{
				//get rid of double-sided triangle
				if (triangle1.triPlane.distance(mTriangle->centroid) >= 0)
				{
					separation = satIntersect<true>(*mTriangle, triangle1, PxMin(mBestDistance, mContactDistance));

					if (separation < mBestDistance)
					{
						mBestDistance = separation;
						mPrimIndex = primitiveIndex;
					}
				}
			}
			else
			{
				const PxU32 clothMask0 = PxEncodeClothIndex(mId0, triangleIndex);
				const PxU32 clothMask1 = PxEncodeClothIndex(mId1, primitiveIndex);
				const PxU32 clothFullMask0 = PxEncodeClothIndex(mId0, PX_MAX_NB_DEFORMABLE_SURFACE_TRI);
				if (!find(mFilterPairs, mNumFilterPairs, PxMin(clothFullMask0, clothMask1), PxMax(clothFullMask0, clothMask1)) && 
					!find(mFilterPairs, mNumFilterPairs, PxMin(clothMask0, clothMask1), PxMax(clothMask0, clothMask1)))
				{
					separation = satIntersect<true>(*mTriangle, triangle1, PxMin(mBestDistance, mContactDistance));

					if (separation < mBestDistance)
					{
						mBestDistance = separation;
						mPrimIndex = primitiveIndex;
					}
				}
			}
		}

		//return mBestDistance < mContactDistance;
	}

	PX_FORCE_INLINE __device__ bool intersectBoxFullWarp(bool hasBox, const PxVec3& min, const PxVec3& max) const
	{
		if (hasBox)
			return !(min.x > tMax.x || tMin.x > max.x || min.y > tMax.y || tMin.y > max.y || min.z > tMax.z || tMin.z > max.z);
		else
			return false;
	}
};

struct TriangleSelfCollisionLeafBoundMinMaxTraverser
{
	femMidphaseScratch* PX_RESTRICT s_warpScratch;
	const float4* PX_RESTRICT mRestPosition;
	PxU32 mFirstTriangleIdx;
	PxReal mSqContactDist;

	const PxVec3 tMin;
	const PxVec3 tMax;

	const physx::PxU32 cmIdx;
	const PxU32 stackSize;
	uint4* PX_RESTRICT stackPtr;
	physx::PxU32* midphasePairs;

	PX_FORCE_INLINE __device__ TriangleSelfCollisionLeafBoundMinMaxTraverser(femMidphaseScratch* PX_RESTRICT s_warpScratch, const PxBounds3& triangleBound, 
		const physx::PxU32 cmIdx, const PxU32 stackSize, uint4* PX_RESTRICT stackPtr, physx::PxU32* midphasePairs,
		const float4* restPosition,	const PxU32 triangleIdx, const PxReal contactDist) :
		s_warpScratch(s_warpScratch), tMin(triangleBound.minimum), tMax(triangleBound.maximum), 
		cmIdx(cmIdx), stackSize(stackSize), stackPtr(stackPtr), midphasePairs(midphasePairs),
		mRestPosition(restPosition), mFirstTriangleIdx(triangleIdx), mSqContactDist(contactDist)
	{ }

	PX_FORCE_INLINE __device__ void intersectPrimitiveFullWarp(PxU32 primitiveIndex, PxU32 idxInWarp) const
	{
		bool intersect = false;
		if (primitiveIndex == 0xFFFFFFFF)
		{
			using namespace physx;

			const uint4* trimeshTriangleIndices = s_warpScratch->meshVertsIndices;
			const float4 * trimeshVerts = s_warpScratch->meshVerts;

			const uint4 triIdx0 = trimeshTriangleIndices[mFirstTriangleIdx];
			const uint4 triIdx1 = trimeshTriangleIndices[primitiveIndex];

			if (triIdx0.x == triIdx1.x || triIdx0.x == triIdx1.y || triIdx0.x == triIdx1.z
				|| triIdx0.y == triIdx1.x || triIdx0.y == triIdx1.y || triIdx0.y == triIdx1.z
				|| triIdx0.z == triIdx1.x || triIdx0.z == triIdx1.y || triIdx0.z == triIdx1.z)
			{
				//printf("(%i %i)not intersect!\n", mFirstTriangleIdx, secondTriangleIdx);
				intersect = false;
			}
			else 
			{
				const PxVec3 v0InVertexSpace = PxLoad3(trimeshVerts[triIdx1.x]);
				const PxVec3 v1InVertexSpace = PxLoad3(trimeshVerts[triIdx1.y]);
				const PxVec3 v2InVertexSpace = PxLoad3(trimeshVerts[triIdx1.z]);

				PxReal tX = PxMin(v0InVertexSpace.x, v1InVertexSpace.x);
				PxReal tY = PxMin(v0InVertexSpace.y, v1InVertexSpace.y);
				PxReal tZ = PxMin(v0InVertexSpace.z, v1InVertexSpace.z);

				const PxReal minX = PxMin(tX, v2InVertexSpace.x);
				const PxReal minY = PxMin(tY, v2InVertexSpace.y);
				const PxReal minZ = PxMin(tZ, v2InVertexSpace.z);

				//compute max
				tX = PxMax(v0InVertexSpace.x, v1InVertexSpace.x);
				tY = PxMax(v0InVertexSpace.y, v1InVertexSpace.y);
				tZ = PxMax(v0InVertexSpace.z, v1InVertexSpace.z);

				const PxReal maxX = PxMax(tX, v2InVertexSpace.x);
				const PxReal maxY = PxMax(tY, v2InVertexSpace.y);
				const PxReal maxZ = PxMax(tZ, v2InVertexSpace.z);

				intersect = !(minX > tMax.x || tMin.x > maxX || minY > tMax.y || tMin.y > maxY ||
					minZ > tMax.z || tMin.z > maxZ);

				if (intersect)
				{
					//look into the rest position
					const PxVec3 p0 = PxLoad3(mRestPosition[triIdx0.x]);
					const PxVec3 p1 = PxLoad3(mRestPosition[triIdx0.y]);
					const PxVec3 p2 = PxLoad3(mRestPosition[triIdx0.z]);

					const PxVec3 a = PxLoad3(mRestPosition[triIdx1.x]);
					const PxVec3 b = PxLoad3(mRestPosition[triIdx1.y]);
					const PxVec3 c = PxLoad3(mRestPosition[triIdx1.z]);

					const PxVec3 ab = b - a;
					const PxVec3 ac = c - a;

					PxVec3 closest = Gu::closestPtPointTriangle2(p0, a, b, c, ab, ac);

					PxVec3 dir = p0 - closest;
					PxReal sqDist0 = dir.dot(dir);

					closest = Gu::closestPtPointTriangle2(p1, a, b, c, ab, ac);

					dir = p1 - closest;
					PxReal sqDist1 = dir.dot(dir);

					closest = Gu::closestPtPointTriangle2(p2, a, b, c, ab, ac);

					dir = p2 - closest;
					PxReal sqDist2 = dir.dot(dir);

					if (sqDist0 <= mSqContactDist || sqDist1 <= mSqContactDist || sqDist2 <= mSqContactDist)
						intersect = false;

					//printf("(%i %i) (%f, %f, %f) mSqContactDist %f \n", mFirstTriangleIdx, secondTriangleIdx, sqDist0, sqDist1, sqDist2, mSqContactDist);

				}
			}
		}

		pushOntoStackFullWarp(intersect, midphasePairs, stackPtr, stackSize, make_uint4(cmIdx, mFirstTriangleIdx, primitiveIndex, 0));
	}

	PX_FORCE_INLINE __device__ bool intersectBoxFullWarp(bool hasBox, const PxVec3& min, const PxVec3& max) const
	{
		if (hasBox)
			return !(min.x > tMax.x || tMin.x > max.x || min.y > tMax.y || tMin.y > max.y || min.z > tMax.z || tMin.z > max.z);
		else
			return false;
	}
};

template<unsigned int WarpsPerBlock>
__device__ static inline void femClothTriMeshMidphaseCore(
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgFEMCloth* PX_RESTRICT femClothes,
	const PxU32 stackSizeBytes,
	PxU8* PX_RESTRICT stackPtr,				//output
	PxU32* PX_RESTRICT midphasePairsNum,	//output
	femMidphaseScratch*	s_warpScratch
)
{
	const PxU32 cmIdx = blockIdx.y;
	// each block deal with one pair
	{
		const PxU32 globalWarpIdx = threadIdx.y + blockIdx.x * blockDim.y;

		PxgShape shape0, shape1;
		PxgShape* trimeshShape = NULL;
		PxgShape* clothShape = NULL;
		PxU32 trimeshCacheRef, clothCacheRef;
		{
			PxgContactManagerInput npWorkItem;
			PxgContactManagerInput_ReadWarp(npWorkItem, cmInputs, cmIdx);
			PxgShape_ReadWarp(shape0, gpuShapes + npWorkItem.shapeRef0);
			PxgShape_ReadWarp(shape1, gpuShapes + npWorkItem.shapeRef1);
			assert(shape0.type == PxGeometryType::eTRIANGLEMESH && shape1.type == PxGeometryType::eTRIANGLEMESH);

			if(shape0.particleOrSoftbodyId != 0xffffffff)
			{
				trimeshShape = &shape1;
				trimeshCacheRef = npWorkItem.transformCacheRef1;
				clothShape = &shape0;
				clothCacheRef = npWorkItem.transformCacheRef0;
			}
			else
			{
				trimeshShape = &shape0;
				trimeshCacheRef = npWorkItem.transformCacheRef0;
				clothShape = &shape1;
				clothCacheRef = npWorkItem.transformCacheRef1;
			}
			assert(trimeshShape->particleOrSoftbodyId == 0xffffffff);
		}

		PxU32 clothId = clothShape->particleOrSoftbodyId;
		const PxgFEMCloth* cloth = &femClothes[clothId];

		PxsCachedTransform trimeshTransformCache;
		PxsCachedTransform_ReadWarp(trimeshTransformCache, transformCache + trimeshCacheRef);

		if (threadIdx.x == 0)
		{
			const PxU8* triGeomPtr = reinterpret_cast<const PxU8*>(trimeshShape->hullOrMeshPtr);			
			readTriangleMesh(triGeomPtr, s_warpScratch->bv32PackedNodes, s_warpScratch->meshVerts, s_warpScratch->meshVertsIndices);
		}
		__syncthreads();

		const PxReal contactDist = contactDistance[clothCacheRef] + contactDistance[trimeshCacheRef];

		const PxU32 nbTriangles = cloth->mNbTriangles; // s_warpScratch->nbPrimitives[0];

		const PxU32 NbWarps = blockDim.y * gridDim.x;

		const uint4* vertIndices = cloth->mTriangleVertexIndices;   // s_warpScratch->trimeshVertsIndices[0];
		const float4* triangleVerts = cloth->mPosition_InvMass; // s_warpScratch->trimeshVerts[0];

		PxBounds3 triangleBound;
		PxTransform& trimeshToWorld = trimeshTransformCache.transform;
		const PxMeshScale& trimeshScale = trimeshShape->scale;

		for(PxU32 triangleIdx = globalWarpIdx; triangleIdx < nbTriangles; triangleIdx += NbWarps)
		{
			const uint4 vertInds = vertIndices[triangleIdx];

			const PxVec3 worldV0 = PxLoad3(triangleVerts[vertInds.x]);
			const PxVec3 worldV1 = PxLoad3(triangleVerts[vertInds.y]);
			const PxVec3 worldV2 = PxLoad3(triangleVerts[vertInds.z]);

			/// transform those point to triangle vertex space
			const PxVec3 v0 =
			    shape2Vertex(trimeshToWorld.transformInv(worldV0), trimeshScale.scale, trimeshScale.rotation);
			const PxVec3 v1 =
			    shape2Vertex(trimeshToWorld.transformInv(worldV1), trimeshScale.scale, trimeshScale.rotation);
			const PxVec3 v2 =
			    shape2Vertex(trimeshToWorld.transformInv(worldV2), trimeshScale.scale, trimeshScale.rotation);

			PxReal tX0 = PxMin(v0.x, v1.x);
			PxReal tY0 = PxMin(v0.y, v1.y);
			PxReal tZ0 = PxMin(v0.z, v1.z);

			triangleBound.minimum.x = PxMin(tX0, v2.x);
			triangleBound.minimum.y = PxMin(tY0, v2.y);
			triangleBound.minimum.z = PxMin(tZ0, v2.z);

			// compute max
			tX0 = PxMax(v0.x, v1.x);
			tY0 = PxMax(v0.y, v1.y);
			tZ0 = PxMax(v0.z, v1.z);

			triangleBound.maximum.x = PxMax(tX0, v2.x);
			triangleBound.maximum.y = PxMax(tY0, v2.y);
			triangleBound.maximum.z = PxMax(tZ0, v2.z);

			// bound in triangel vertext space
			triangleBound.fattenFast(contactDist);

			uint4* tStackPtr = reinterpret_cast<uint4*>(stackPtr);
			const PxU32 stackSize = stackSizeBytes / sizeof(uint4);

			Triangle triangle;
			triangle.verts[0] = v0;
			triangle.verts[1] = v1;
			triangle.verts[2] = v2;
			triangle.triPlane = PxPlane(v0, v1, v2);
			triangle.centroid = (v0 + v1 + v2) / 3.f;

			TriangleLeafBoundMinMaxTraverser op(s_warpScratch, triangleIdx, triangleBound, &triangle, contactDist, true,
			                                    0xffffffff, clothId);
			bv32TreeTraversal<TriangleLeafBoundMinMaxTraverser, WarpsPerBlock>(s_warpScratch->bv32PackedNodes,
			                                                                   s_warpScratch->sBv32Nodes, op);

			const PxU32 MaxNumPairs = 4;

			PxU32 mask = __ballot_sync(FULL_MASK, op.mBestDistance < contactDist);

			PxU32 count = PxMin(PxU32(__popc(mask)), MaxNumPairs);

			if(count)
			{
				PxU32 index;
				if(threadIdx.x == 0)
				{
					index = atomicAdd(midphasePairsNum, count);
				}

				index = __shfl_sync(FULL_MASK, index, 0);

				PxReal bestSep = -1.f;
				for(PxU32 i = 0; i < count && bestSep < contactDist; i++)
				{
					PxU32 winnerLane = minIndex(op.mBestDistance, FULL_MASK, bestSep);

					if(threadIdx.x == winnerLane)
					{
						if(op.mBestDistance < contactDist)
						{
							if((index + i) < stackSize)
								tStackPtr[index + i] = make_uint4(cmIdx, triangleIdx, op.mPrimIndex, 0);
							op.mBestDistance = PX_MAX_F32;
						}
					}
				}
			}
		}
	}
}

extern "C" __global__
//__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 4)
void cloth_meshMidphaseGeneratePairsLaunch(
	const PxReal tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgFEMCloth* PX_RESTRICT femClothes,
	const PxU32 stackSizeBytes,
	PxU8* PX_RESTRICT stackPtr,							//output
	PxU32* PX_RESTRICT midphasePairsNum					//output
)
{
	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][FEM_MIDPHASE_SCRATCH_SIZE];

	femClothTriMeshMidphaseCore<MIDPHASE_WARPS_PER_BLOCK>(
		tolerenceLength,
		cmInputs,
		transformCache,
		bounds,
		contactDistance,
		gpuShapes,
		femClothes,
		stackSizeBytes,
		stackPtr,
		midphasePairsNum,
		(femMidphaseScratch*)scratchMem[threadIdx.y]
		);
}


template<unsigned int WarpsPerBlock>
__device__ static inline void femClothClothMidphaseCore(
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgFEMCloth* PX_RESTRICT femClothes,
	const PxgNonRigidFilterPair* PX_RESTRICT filterPairs,
	const PxU32	numFilterPairs,
	const PxU32 stackSizeBytes,
	PxU8* PX_RESTRICT stackPtr,									//output
	PxU32* PX_RESTRICT midphasePairsNum,						//output
	femMidphaseScratch*	s_warpScratch
)
{

	const PxU32 cmIdx = blockIdx.y;
	//each block deal with one pair
	{
		const PxU32 globalWarpIdx = threadIdx.y + blockIdx.x*blockDim.y;

		PxgShape clothShape0, clothShape1;
		PxU32 clothCacheRef0, clothCacheRef1;
		LoadShapePairWarp<PxGeometryType::eTRIANGLEMESH, PxGeometryType::eTRIANGLEMESH>(cmInputs, cmIdx, gpuShapes,
			clothShape0, clothCacheRef0, clothShape1, clothCacheRef1);

		const PxgFEMCloth* cloth0 = &femClothes[clothShape0.particleOrSoftbodyId];
		const PxgFEMCloth* cloth1 = &femClothes[clothShape1.particleOrSoftbodyId];

		if (threadIdx.x == 0)
		{
			s_warpScratch->meshVerts = cloth1->mPosition_InvMass;
			s_warpScratch->meshVertsIndices = cloth1->mTriangleVertexIndices;

			PxU8* trimeshGeomPtr = reinterpret_cast<PxU8 *>(cloth1->mTriMeshData);
			trimeshGeomPtr += sizeof(uint4);

			Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(trimeshGeomPtr);
			s_warpScratch->bv32PackedNodes = bv32PackedNodes;
		}

		__syncthreads();

		const PxReal contactDist = contactDistance[clothCacheRef0] + contactDistance[clothCacheRef1];

		const PxU32 nbTriangles = cloth0->mNbTriangles;

		const PxU32 NbWarps = blockDim.y*gridDim.x;

		const uint4* vertIndices = cloth0->mTriangleVertexIndices;
		const float4* triangleVerts = cloth0->mPosition_InvMass;

		PxBounds3 triangleBound;

		for (PxU32 triangleIdx = globalWarpIdx; triangleIdx < nbTriangles; triangleIdx += NbWarps)
		{
			const PxU32 clothMask0 = PxEncodeClothIndex(clothShape0.particleOrSoftbodyId, triangleIdx);
			const PxU32 clothFullMask1 = PxEncodeClothIndex(clothShape1.particleOrSoftbodyId, PX_MAX_NB_DEFORMABLE_SURFACE_TRI);

			if (find(filterPairs, numFilterPairs, PxMin(clothMask0, clothFullMask1), PxMax(clothMask0, clothFullMask1)))
				continue;

			const uint4 vertInds = vertIndices[triangleIdx];

			const PxVec3 worldV0 = PxLoad3(triangleVerts[vertInds.x]);
			const PxVec3 worldV1 = PxLoad3(triangleVerts[vertInds.y]);
			const PxVec3 worldV2 = PxLoad3(triangleVerts[vertInds.z]);

			triangleBound = triBoundingBox(worldV0, worldV1, worldV2);

			//computeTriangleBound2(vertIndices, triangleIdx, triangleVerts, triangleBound);

			triangleBound.fattenFast(contactDist);

			uint4* tStackPtr = reinterpret_cast<uint4*>(stackPtr);
			const PxU32 stackSize = stackSizeBytes / sizeof(uint4);

			Triangle triangle;
			triangle.verts[0] = worldV0;
			triangle.verts[1] = worldV1;
			triangle.verts[2] = worldV2;
			triangle.triPlane = PxPlane(worldV0, worldV1, worldV2);

			triangle.centroid = (worldV0 + worldV1 + worldV2) / 3.f;

			TriangleLeafBoundMinMaxTraverser op(s_warpScratch, triangleIdx, triangleBound, &triangle, contactDist, false, clothShape0.particleOrSoftbodyId, clothShape1.particleOrSoftbodyId);
			op.setFiltering(filterPairs, numFilterPairs);
			bv32TreeTraversal<TriangleLeafBoundMinMaxTraverser, WarpsPerBlock>(s_warpScratch->bv32PackedNodes, s_warpScratch->sBv32Nodes, op);

			const PxU32 MaxNumPairs = 4;

			PxU32 mask = __ballot_sync(FULL_MASK, op.mBestDistance < contactDist);

			const PxU32 actualCount = PxU32(__popc(mask));

			PxU32 count = PxMin(PxU32(__popc(mask)), MaxNumPairs);

			if (count)
			{
				PxU32 index;
				if (threadIdx.x == 0)
				{
					index = atomicAdd(midphasePairsNum, count);
				}

				index = __shfl_sync(FULL_MASK, index, 0);

				PxReal bestSep = -1.f;
				for (PxU32 i = 0; i < count && bestSep < contactDist; i++)
				{
					PxU32 winnerLane = minIndex(op.mBestDistance, FULL_MASK, bestSep);

					if (threadIdx.x == winnerLane)
					{
						if (op.mBestDistance < contactDist)
						{
							if ((index + i) < stackSize)
								tStackPtr[index + i] = make_uint4(cmIdx, triangleIdx, op.mPrimIndex, 0);
							op.mBestDistance = PX_MAX_F32;
						}
					}
				}
			}
		}
	}
}

extern "C" __global__
//__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 4)
void cloth_clothMidphaseGeneratePairsLaunch(
	const PxReal tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgFEMCloth* PX_RESTRICT femClothes,

	const PxgNonRigidFilterPair* PX_RESTRICT filterPairs,
	const PxU32	numFilterPairs,
	const PxU32 stackSizeBytes,
	PxU8* PX_RESTRICT stackPtr,							//output
	PxU32* PX_RESTRICT midphasePairsNum					//output
)
{

	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][FEM_MIDPHASE_SCRATCH_SIZE];

	femClothClothMidphaseCore<MIDPHASE_WARPS_PER_BLOCK>(
		tolerenceLength,
		cmInputs,
		transformCache,
		bounds,
		contactDistance,
		gpuShapes,
		femClothes,
		filterPairs,
		numFilterPairs,
		stackSizeBytes,
		stackPtr,
		midphasePairsNum,
		(femMidphaseScratch*)scratchMem[threadIdx.y]
		);
}


template<unsigned int WarpsPerBlock>
__device__ static inline void cloth_selfCollisionMidphaseCore(
	const PxU32* activeClothes,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgFEMCloth* PX_RESTRICT clothes,
	const PxU32 stackSizeBytes,
	PxU8* PX_RESTRICT stackPtr,									//output
	PxU32* PX_RESTRICT midphasePairsNum,						//output

	femMidphaseScratch*	s_warpScratch
)
{
	const PxU32 cmIdx = blockIdx.y;

	const PxU32 clothId = activeClothes[cmIdx];

	const PxgFEMCloth* cloth = &clothes[clothId];

	if (cloth->mBodyFlags & PxDeformableBodyFlag::eDISABLE_SELF_COLLISION)
		return;

	const PxU32 numTriangles = cloth->mNbTriangles;

	//each block deal with one pair
	//if (cmIdx < numTets)
	{

		const PxU32 globalWarpIdx = threadIdx.y + blockIdx.x*blockDim.y;

		const PxU32 idx = threadIdx.x;

		if (idx == 0)
		{
			s_warpScratch->meshVerts = cloth->mPosition_InvMass;
			s_warpScratch->meshVertsIndices = cloth->mTriangleVertexIndices;
			//s_warpScratch->tetmeshTetSurfaceHint[idx] = softbody->mTetMeshSurfaceHint;

			PxU8* trimeshGeomPtr = reinterpret_cast<PxU8 *>(cloth->mTriMeshData);

			//const uint4 nbVerts_nbTets_maxDepth_nbBv32TreeNodes = *reinterpret_cast<const uint4 *>(tetmeshGeomPtr);
			trimeshGeomPtr += sizeof(uint4);

			//s_warpScratch->nbPrimitives[idx] = nbVerts_nbTets_maxDepth_nbBv32TreeNodes.y;

			Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(trimeshGeomPtr);
			s_warpScratch->bv32PackedNodes = bv32PackedNodes;
			/*tetmeshGeomPtr += sizeof(const Gu::BV32DataPacked)* nbVerts_nbTets_maxDepth_nbBv32TreeNodes.w;

			tetmeshGeomPtr += sizeof(const Gu::BV32DataDepthInfo) * nbVerts_nbTets_maxDepth_nbBv32TreeNodes.z
				+ sizeof(PxU32) * nbVerts_nbTets_maxDepth_nbBv32TreeNodes.w;*/

		}

		__syncthreads();

		const PxU32 elementInd = cloth->mElementIndex;

		const float4* restPosition = cloth->mRestPosition;

		const PxReal contactDist = contactDistance[elementInd] * 2.f;

		/*if (globalWarpIdx == 0 && threadIdx.x == 0)
		{
			printf("elementInd %i numTriangles %i\n", elementInd, numTriangles);
		}*/

		//const PxU32 nbTets = s_warpScratch->nbPrimitives[1];

		const PxU32 NbWarps = blockDim.y*gridDim.x;

		const uint4* triIndices = s_warpScratch->meshVertsIndices;
		const float4* triVerts = s_warpScratch->meshVerts;

		PxBounds3 triangleBound;

		for (PxU32 triangleIdx = globalWarpIdx; triangleIdx < numTriangles; triangleIdx += NbWarps)
		{
			computeTriangleBound2(triIndices, triangleIdx, triVerts, triangleBound);

			triangleBound.fattenFast(contactDist);


			const PxReal sqContactDist = contactDist * contactDist;
			uint4* tStackPtr = reinterpret_cast<uint4*>(stackPtr);
			const PxU32 stackSize = stackSizeBytes / sizeof(uint4);

			//Need to implement this leaf function for self collision
			bv32TreeTraversal<const TriangleSelfCollisionLeafBoundMinMaxTraverser, WarpsPerBlock>(s_warpScratch->bv32PackedNodes, s_warpScratch->sBv32Nodes, TriangleSelfCollisionLeafBoundMinMaxTraverser(s_warpScratch, triangleBound,
				cmIdx, stackSize, tStackPtr, midphasePairsNum, restPosition, triangleIdx, sqContactDist));
		}
	}
}

extern "C" __global__
//__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 4)
void cloth_selfCollisionMidphaseGeneratePairsLaunch(
	const PxU32* activeClothes,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgFEMCloth* PX_RESTRICT clothes,
	const PxU32 stackSizeBytes,
	PxU8* PX_RESTRICT stackPtr,							//output
	PxU32* PX_RESTRICT midphasePairsNum					//output
)
{
	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][FEM_MIDPHASE_SCRATCH_SIZE];

	cloth_selfCollisionMidphaseCore<MIDPHASE_WARPS_PER_BLOCK>(
		activeClothes,
		contactDistance,
		clothes,
		stackSizeBytes,
		stackPtr,
		midphasePairsNum,
		(femMidphaseScratch*)scratchMem[threadIdx.y]
		);
}

__device__ PxReal distInRestPose(const float4* restVerts, const float4* restVertexBuffer, PxU32 restVertexIndex, const uint4 triIndex)
{
	if (restVerts)
	{
		const PxVec3 a = PxLoad3(restVerts[triIndex.x]);
		const PxVec3 b = PxLoad3(restVerts[triIndex.y]);
		const PxVec3 c = PxLoad3(restVerts[triIndex.z]);

		const PxVec3 ab = b - a;
		const PxVec3 ac = c - a;

		PxVec3 restVertex = PxLoad3(restVertexBuffer[restVertexIndex]);
		PxVec3 restClosest = Gu::closestPtPointTriangle2(restVertex, a, b, c, ab, ac);

		PxVec3 restDir = restVertex - restClosest;
		PxReal sqRestDist = restDir.dot(restDir);

		return sqRestDist;
	}
	return PX_MAX_REAL;
}

struct ClothTreeVertexTraverser
{
	const PxU32 contactSize;
	physx::PxU32 vertexIdx;
	const PxReal contactDist;
	const PxReal filterDistanceSq;
	const PxU32 clothId0;
	const PxU32 clothId1;
	const float4* vertices0;
	const float4* restVerts0;
	const uint4* triIndices;
	PxVec3 vertex;
	PxgNonRigidFilterPair* clothClothPairs;
	const PxU32 numClothClothPairs;
	float4* outPoint;                  // output
	float4* outNormalRestDistSq;       // output
	float4* outBarycentric0;           // output
	float4* outBarycentric1;           // output
	PxgFemContactInfo* outContactInfo; // output
	PxU32* totalContactCount;          // output
	const float4* vertices1;
	const float4* restVerts1;

	PX_FORCE_INLINE __device__ ClothTreeVertexTraverser(
	    const PxU32 contactSize, const PxReal contactDist,
	    const PxReal filterDistanceSq, const PxU32 clothId0, const PxU32 clothId1, const float4* vertices0,
	    const float4* restVerts0, const uint4* triIndices, PxgNonRigidFilterPair* clothClothPairs,
	    const PxU32 numClothClothPairs,
	    float4* outPoint,                  // output
	    float4* outNormalRestDistSq,       // output
	    float4* outBarycentric0,           // output
	    float4* outBarycentric1,           // output
	    PxgFemContactInfo* outContactInfo, // output
	    PxU32* totalContactCount,          // output
		const float4* vertices1,
		const float4* restVerts1)
	: contactSize(contactSize)
	, contactDist(contactDist)
	, filterDistanceSq(filterDistanceSq)
	, clothId0(clothId0)
	, clothId1(clothId1)
	, vertices0(vertices0)
	, restVerts0(restVerts0)
	, triIndices(triIndices)
	, clothClothPairs(clothClothPairs)
	, numClothClothPairs(numClothClothPairs)
	, outPoint(outPoint)
	, outNormalRestDistSq(outNormalRestDistSq)
	, outBarycentric0(outBarycentric0)
	, outBarycentric1(outBarycentric1)
	, outContactInfo(outContactInfo)
	, totalContactCount(totalContactCount)
	, vertices1(vertices1)
	, restVerts1(restVerts1)
	{
	}

	PX_FORCE_INLINE __device__ void intersectPrimitiveFullWarp(PxU32 primitiveIndex, PxU32 idxInWarp)
	{
		PxReal s, t;
		float4 normal_restDistSq;

		bool intersect = false;
		if(primitiveIndex != 0xFFFFFFFF)
		{
			const uint4 triIdx1 = triIndices[primitiveIndex];

			// Check if the vertex is part of the triangle
			if(clothId0 != clothId1 || !(vertexIdx == triIdx1.x || vertexIdx == triIdx1.y || vertexIdx == triIdx1.z))
			{
				// Currently using the original contact distance
				const PxReal adjustedContactDist = contactDist;

				const PxVec3 x1 = PxLoad3(vertices0[triIdx1.x]);
				const PxVec3 x2 = PxLoad3(vertices0[triIdx1.y]);
				const PxVec3 x3 = PxLoad3(vertices0[triIdx1.z]);

				PxBounds3 bounds;
				bounds.minimum = x1;
				bounds.maximum = x1;
				bounds.include(x2);
				bounds.include(x3);
				bounds.fattenFast(adjustedContactDist);
				if (bounds.contains(vertex))
				{
					bool generateContact = true;
					const PxU32 clothVertMask0 = PxEncodeClothIndex(clothId0, vertexIdx);
					const PxU32 clothVertFullMask0 = PxEncodeClothIndex(clothId0, PX_MAX_NB_DEFORMABLE_SURFACE_VTX);
					const PxU32 clothTriMask1 = PxEncodeClothIndex(clothId1, primitiveIndex);
					const PxU32 clothTriFullMask1 = PxEncodeClothIndex(clothId1, PX_MAX_NB_DEFORMABLE_SURFACE_TRI);

					// Check if the vertex index is in the attachment list.
					if(find(clothClothPairs, numClothClothPairs, clothVertMask0, clothTriFullMask1) ||
					   find(clothClothPairs, numClothClothPairs, clothVertFullMask0, clothTriMask1) ||
					   find(clothClothPairs, numClothClothPairs, clothVertMask0, clothTriMask1))
						generateContact = false;

					if(generateContact)
					{
						const PxVec3 closestPt = closestPtPointTriangle(vertex, x1, x2, x3, s, t);

						PxVec3 dir = closestPt - vertex;
						const PxReal sqDist = dir.magnitudeSquared();

						if (sqDist < adjustedContactDist * adjustedContactDist && sqDist > 1.e-14f)
						{
							const PxReal restDistSq = distInRestPose(restVerts0, restVerts1, vertexIdx, triIdx1);

							// Check if the objects are closer than the filter distance threshold.
							if (restDistSq > filterDistanceSq)
							{
								PxReal dist = PxSqrt(sqDist);
								dir = dir * (1.0f / dist);

								PxVec3 normal = (x2 - x1).cross(x3 - x2).getNormalized();

								const PxReal proj = normal.dot(dir);
								if (clothId0 != clothId1 || PxAbs(proj) > 0.01f)
								{
									normal_restDistSq = make_float4(dir.x, dir.y, dir.z, restDistSq);
									intersect = s > -1e-6f && t > -1e-6f && (s + t) <= 1.0f;
								}
							}
						}
					}
				}
			}
		}

		const PxU32 index = globalScanExclusiveSingleWarp(intersect, totalContactCount);

		if(intersect && index < contactSize)
		{
			PxU32 pairInd0 = PxEncodeClothIndex(clothId0, vertexIdx);
			PxU32 pairInd1 = PxEncodeClothIndex(clothId1, primitiveIndex); // vert id

			outPoint[index] = make_float4(vertex.x, vertex.y, vertex.z, 0.f);
			outNormalRestDistSq[index] = normal_restDistSq;
			outBarycentric0[index] = make_float4(0.f, 0.f, 0.f, 1.f);
			outBarycentric1[index] = make_float4(1.f - s - t, s, t, 0.f); // barycentric;
			outContactInfo[index].pairInd0 = pairInd0;
			outContactInfo[index].pairInd1 = pairInd1;
		}
	}

	PX_FORCE_INLINE __device__ bool intersectBoxFullWarp(bool hasBox, const PxVec3& min, const PxVec3& max) const
	{
		if(hasBox)
		{
			PxVec3 closest = min.maximum(vertex.minimum(max));
			return (closest - vertex).magnitudeSquared() <= contactDist * contactDist;
		}
		else
			return false;
	}
};

template<unsigned int WarpsPerBlock>
__device__ static inline void cloth_vertexSelfCollisionMidphaseCore(
	const PxU32								maxContacts,
	const PxU32*							activeClothes,
	const PxReal* PX_RESTRICT				contactDistance,
	const PxgFEMCloth* PX_RESTRICT			clothes,
	PxgNonRigidFilterPair*					clothClothPairs,
	const PxU32								numClothClothPairs,
	float4*									outPoint,				//output
	float4*									outNormalPen,			//output
	float4*									outBarycentric0,		//output
	float4*									outBarycentric1,		//output
	PxgFemContactInfo*						outContactInfo,			//output
	PxU32*									totalContactCount,		//output
	femMidphaseScratch*						s_warpScratch
)
{
	const PxU32 cmIdx = blockIdx.y;

	const PxU32 clothId = activeClothes[cmIdx];

	const PxgFEMCloth* cloth = &clothes[clothId];

	const PxU32 globalWarpIdx = threadIdx.y + blockIdx.x * blockDim.y;

	if(cloth->mBodyFlags & PxDeformableBodyFlag::eDISABLE_SELF_COLLISION)
		return;

	const PxU32 numVerts = cloth->mNbVerts;

	{
		const PxU32 idx = threadIdx.x;

		if(idx == 0)
		{
			s_warpScratch->meshVerts = cloth->mPosition_InvMass;
			s_warpScratch->meshVertsIndices = cloth->mTriangleVertexIndices;

			PxU8* trimeshGeomPtr = reinterpret_cast<PxU8*>(cloth->mTriMeshData);
			trimeshGeomPtr += sizeof(uint4);

			Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(trimeshGeomPtr);
			s_warpScratch->bv32PackedNodes = bv32PackedNodes;
		}

		__syncthreads();

		const PxU32 elementInd = cloth->mElementIndex;

		const float4* restPosition = cloth->mRestPosition;

		const PxReal contactDist = contactDistance[elementInd] * 2.f;

		const PxReal filterDistance = cloth->mSelfCollisionFilterDistance;

		const PxU32 NbWarps = blockDim.y * gridDim.x;

		const float4* triVerts = s_warpScratch->meshVerts;

		ClothTreeVertexTraverser traverser(maxContacts, contactDist, filterDistance * filterDistance, clothId,
			clothId, s_warpScratch->meshVerts, restPosition,
			s_warpScratch->meshVertsIndices, clothClothPairs,
			numClothClothPairs, outPoint, outNormalPen, outBarycentric0,
			outBarycentric1, outContactInfo, totalContactCount,
			triVerts, restPosition);
		for(PxU32 vertexIdx = globalWarpIdx; vertexIdx < numVerts; vertexIdx += NbWarps)
		{
			traverser.vertexIdx = vertexIdx;
			traverser.vertex = PxLoad3(triVerts[vertexIdx]);

			bv32TreeTraversal<ClothTreeVertexTraverser, WarpsPerBlock>(s_warpScratch->bv32PackedNodes, s_warpScratch->sBv32Nodes, traverser);
		}
	}
}

extern "C" __global__
__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 16) //At least 16 blocks per multiprocessor helps with performance
void cloth_selfCollisionVertexMidphaseGeneratePairsLaunch(
	const PxU32									maxContacts,
	const PxU32*								activeClothes,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxgFEMCloth*	PX_RESTRICT				clothes,
	PxgNonRigidFilterPair*						clothClothPairs,
	const PxU32									numClothClothPairs,
	const PxU8*									updateContactPairs,
	float4*										outPoint,									//output
	float4*										outNormalPen,								//output
	float4*										outBarycentric0,							//output
	float4*										outBarycentric1,							//output
	PxgFemContactInfo*							outContactInfo,								//output
	PxU32*										totalContactCount							//output
)
{
	// Early exit if contact pairs are not updated.
	if (*updateContactPairs == 0)
		return;

	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][FEM_MIDPHASE_SCRATCH_SIZE];

	cloth_vertexSelfCollisionMidphaseCore<MIDPHASE_WARPS_PER_BLOCK>(
		maxContacts,
		activeClothes,
		contactDistance,
		clothes,
		clothClothPairs,
		numClothClothPairs,
		outPoint,
		outNormalPen,
		outBarycentric0,
		outBarycentric1,
		outContactInfo,
		totalContactCount,
		(femMidphaseScratch*)scratchMem[threadIdx.y]
		);
}

template<unsigned int WarpsPerBlock>
__device__ static inline void femClothClothVertexCollisionCore(
	const PxU32										maxContacts,
	const PxReal									toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT		cmInputs,
	const PxsCachedTransform* PX_RESTRICT			transformCache,
	const PxReal* PX_RESTRICT						contactDistance,
	const PxgShape* PX_RESTRICT						gpuShapes,
	const PxgFEMCloth* PX_RESTRICT					femClothes,
	PxgNonRigidFilterPair*							clothClothPairs,
	const PxU32										numClothClothPairs,
	float4*											outPoint,			//output
	float4*											outNormalPen,		//output
	float4*											outBarycentric0,	//output
	float4*											outBarycentric1,	//output
	PxgFemContactInfo*								outContactInfo,		//output
	PxU32*											totalContactCount,	//output
	femMidphaseScratch*								s_warpScratch
)
{
	const PxU32 cmIdx = blockIdx.y;
	// each block deal with one pair
	{
		const PxU32 globalWarpIdx = threadIdx.y + blockIdx.x * blockDim.y;

		PxgContactManagerInput npWorkItem;
		PxgContactManagerInput_ReadWarp(npWorkItem, cmInputs, cmIdx);

		PxgShape shape0;
		PxgShape_ReadWarp(shape0, gpuShapes + npWorkItem.shapeRef0);

		PxgShape shape1;
		PxgShape_ReadWarp(shape1, gpuShapes + npWorkItem.shapeRef1);

		assert(shape0.type == PxGeometryType::eTRIANGLEMESH && shape1.type == PxGeometryType::eTRIANGLEMESH);

		const PxU32 clothId0 = shape0.particleOrSoftbodyId;
		const PxU32 clothId1 = shape1.particleOrSoftbodyId;

		const PxgFEMCloth* cloth0 = &femClothes[clothId0];
		const PxgFEMCloth* cloth1 = &femClothes[clothId1];

		if(threadIdx.x == 0)
		{
			s_warpScratch->meshVerts = cloth1->mPosition_InvMass;
			s_warpScratch->meshVertsIndices = cloth1->mTriangleVertexIndices;

			PxU8* trimeshGeomPtr = reinterpret_cast<PxU8*>(cloth1->mTriMeshData);

			trimeshGeomPtr += sizeof(uint4);

			Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(trimeshGeomPtr);
			s_warpScratch->bv32PackedNodes = bv32PackedNodes;
		}

		__syncthreads();

		const PxReal contactDist = contactDistance[npWorkItem.transformCacheRef0] + contactDistance[npWorkItem.transformCacheRef1];

		PxU32 numVerts = cloth0->mNbVerts;

		const PxU32 NbWarps = blockDim.y * gridDim.x;

		const float4* verts0 = cloth0->mPosition_InvMass;

		{
			ClothTreeVertexTraverser traverser(maxContacts, contactDist, 0.0f, clothId0, clothId1,
				s_warpScratch->meshVerts, NULL, s_warpScratch->meshVertsIndices,
				clothClothPairs, numClothClothPairs, outPoint, outNormalPen,
				outBarycentric0, outBarycentric1, outContactInfo, totalContactCount,
				verts0, NULL);

			for (PxU32 vertexIdx = globalWarpIdx; vertexIdx < numVerts; vertexIdx += NbWarps)
			{
				traverser.vertexIdx = vertexIdx;
				traverser.vertex = PxLoad3(verts0[vertexIdx]);

				bv32TreeTraversal<ClothTreeVertexTraverser, WarpsPerBlock>(s_warpScratch->bv32PackedNodes,
					s_warpScratch->sBv32Nodes, traverser);
			}
		}

		if(1)
		{
			__syncthreads(); // wait for completion, then we flip the pairs such that we traverse the second cloth with
			                 // verts from the first

			if(threadIdx.x == 0)
			{
				s_warpScratch->meshVerts = cloth0->mPosition_InvMass;
				s_warpScratch->meshVertsIndices = cloth0->mTriangleVertexIndices;

				PxU8* trimeshGeomPtr = reinterpret_cast<PxU8*>(cloth0->mTriMeshData);

				trimeshGeomPtr += sizeof(uint4);

				Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(trimeshGeomPtr);
				s_warpScratch->bv32PackedNodes = bv32PackedNodes;
			}

			__syncthreads();

			numVerts = cloth1->mNbVerts;

			verts0 = cloth1->mPosition_InvMass;

			ClothTreeVertexTraverser traverser(
				maxContacts, contactDist, 0.0f, clothId1, clothId0, s_warpScratch->meshVerts,
				NULL, s_warpScratch->meshVertsIndices, clothClothPairs, numClothClothPairs, outPoint,
				outNormalPen, outBarycentric0, outBarycentric1, outContactInfo, totalContactCount,
				verts0, NULL);

			for(PxU32 vertexIdx = globalWarpIdx; vertexIdx < numVerts; vertexIdx += NbWarps)
			{
				traverser.vertexIdx = vertexIdx;
				traverser.vertex = PxLoad3(verts0[vertexIdx]);
				
				bv32TreeTraversal<ClothTreeVertexTraverser, WarpsPerBlock>(s_warpScratch->bv32PackedNodes, s_warpScratch->sBv32Nodes,
																		   traverser);
			}
		}
	}
}

extern "C" __global__
__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 16) //At least 16 blocks per multiprocessor helps with performance
void cloth_clothVertexCollisionLaunch(
	const PxU32									maxContacts,
	const PxReal								tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgFEMCloth* PX_RESTRICT				femClothes,
	PxgNonRigidFilterPair*						clothClothPairs,
	const PxU32									numClothClothPairs,
	const PxU8*									updateContactPairs,
	float4*										outPoint,									//output
	float4*										outNormalPen,								//output
	float4*										outBarycentric0,							//output
	float4*										outBarycentric1,							//output
	PxgFemContactInfo*							outContactInfo,								//output
	PxU32*										totalContactCount							//output
)
{
	// Early exit if contact pairs are not updated.
	if (*updateContactPairs == 0)
		return;

	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][FEM_MIDPHASE_SCRATCH_SIZE];

	femClothClothVertexCollisionCore<MIDPHASE_WARPS_PER_BLOCK>(
		maxContacts,
		tolerenceLength,
		cmInputs,
		transformCache,
		contactDistance,
		gpuShapes,
		femClothes,
		clothClothPairs,
		numClothClothPairs,
		outPoint,
		outNormalPen,
		outBarycentric0,
		outBarycentric1,
		outContactInfo,
		totalContactCount,
		(femMidphaseScratch*)scratchMem[threadIdx.y]
		);
}