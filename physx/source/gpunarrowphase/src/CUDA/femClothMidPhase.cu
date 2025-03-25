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
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "foundation/PxVec3.h"

#include "geometry/PxGeometry.h"
#include "geometry/PxMeshScale.h"

#include "GuBV32.h"
#include "cudaNpCommon.h"

#include "PxgContactManager.h"
#include "PxgConvexConvexShape.h"
#include "PxgFEMCloth.h"
#include "PxgNpKernelIndices.h"
#include "PxgParticleSystem.h"

#include "PxsTransformCache.h"

#include <vector_types.h>

#include "copy.cuh"
#include "dataReadWriteHelper.cuh"
#include "femMidphaseScratch.cuh"
#include "utils.cuh"

#include "bv32Traversal.cuh"
#include "triangleMesh.cuh"
#include "deformableCollision.cuh"

using namespace physx;

extern "C" __host__ void initNarrowphaseKernels18() {}

struct ClothTreeTraverser
{
	femMidphaseScratch* s_warpScratch;
	uint4* PX_RESTRICT stackPtr;
	PxU32* PX_RESTRICT midphasePairsNum;
	unsigned int cmIdx;
	const PxU32 stackSize;
	const PxBounds3& mBox;

	PX_FORCE_INLINE __device__ ClothTreeTraverser(
		femMidphaseScratch* s_warpScratch, 
		PxBounds3& aabb,
		uint4* PX_RESTRICT stackPtr,
		PxU32* PX_RESTRICT midphasePairsNum,
		unsigned int cmIdx,
		const PxU32 stackSize)
		: s_warpScratch(s_warpScratch), mBox(aabb), stackPtr(stackPtr), midphasePairsNum(midphasePairsNum),
		  cmIdx(cmIdx), stackSize(stackSize)
	{ }

	PX_FORCE_INLINE __device__ void intersectPrimitiveFullWarp(PxU32 primitiveIndex, PxU32 idxInWarp) const
	{
		bool intersect = false;
		if (primitiveIndex != 0xFFFFFFFF) 
		{
			uint4 triIdx = s_warpScratch->meshVertsIndices[primitiveIndex];

			const PxVec3 worldV0 = PxLoad3(s_warpScratch->meshVerts[triIdx.x]);
			const PxVec3 worldV1 = PxLoad3(s_warpScratch->meshVerts[triIdx.y]);
			const PxVec3 worldV2 = PxLoad3(s_warpScratch->meshVerts[triIdx.z]);			

			intersect = triBoundingBox(worldV0, worldV1, worldV2).intersects(mBox);
		}

		pushOntoStackFullWarp(intersect, midphasePairsNum, stackPtr, stackSize, make_uint4(cmIdx, primitiveIndex, 0, 0));
	}	

	PX_FORCE_INLINE __device__ bool intersectBoxFullWarp(bool hasBox, const PxVec3& min, const PxVec3& max) const
	{
		if (hasBox)
			return mBox.intersects(PxBounds3(min, max));
		else
			return false;
	}
};

template<unsigned int WarpsPerBlock>
__device__ static inline void femClothMidphaseCore(
	PxU32 numNPWorkItems,
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgFEMCloth* PX_RESTRICT femClothes,

	PxU8* PX_RESTRICT stackPtr,									//output
	PxU32* PX_RESTRICT midphasePairsNum,						//output

	femMidphaseScratch*	s_warpScratch,
	const PxU32 stackSize
)
{

	//wrap index
	const unsigned int warpIdx = threadIdx.y;//(idx >> LOG2_WARP_SIZE);
											 //wrap index in block
											 //const unsigned int idx = idxInWarp + warpIdx * WARP_SIZE;

	unsigned int cmIdx = warpIdx + blockIdx.x * blockDim.y;

	//unsigned mask_cmIdx = __ballot_sync(FULL_MASK, cmIdx < numNPWorkItems);
	if (cmIdx < numNPWorkItems)
	{
		/*	if(threadIdx.x == 0)
				printf("cmIdx %i warpIdx %i\n", cmIdx, warpIdx);*/

		PxgShape clothShape, rigidShape;
		PxU32 clothCacheRef, rigidCacheRef;
		LoadShapePairWarp<PxGeometryType::eTRIANGLEMESH>(cmInputs, cmIdx, gpuShapes,
			clothShape, clothCacheRef, rigidShape, rigidCacheRef);

		const PxU32 femClothId = clothShape.particleOrSoftbodyId;

		const PxgFEMCloth& femCloth = femClothes[femClothId];

		PxU8 * trimeshGeomPtr = reinterpret_cast<PxU8 *>(femCloth.mTriMeshData);

		trimeshGeomPtr += sizeof(uint4); // skip nbVerts_nbTets_maxDepth_nbBv32TreeNodes

		if (threadIdx.x == 0)
		{
			s_warpScratch->meshVerts = femCloth.mPosition_InvMass;
			s_warpScratch->meshVertsIndices = femCloth.mTriangleVertexIndices;
			
			Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(trimeshGeomPtr);
			s_warpScratch->bv32PackedNodes = bv32PackedNodes;
		}

		__syncwarp();


		//ML: this is the world bound AABB
		PxBounds3 aabb;
		PxBounds3_ReadWarp(aabb, bounds + rigidCacheRef);
		const PxReal contactDist = contactDistance[clothCacheRef] + contactDistance[rigidCacheRef];
		aabb.fattenFast(contactDist);

		/*if (threadIdx.x == 0)
		{
			printf("contactDist %f transformCacheRef0 %i transformCacheRef1 %i\n", contactDist, transformCacheRef0, transformCacheRef1);
			printf("min(%f, %f, %f) max(%f, %f, %f)\n", aabb.minimum.x, aabb.minimum.y, aabb.minimum.z,
				aabb.maximum.x, aabb.maximum.y, aabb.maximum.z);
		}*/

		uint4* tStackPtr = reinterpret_cast<uint4*>(stackPtr);
		bv32TreeTraversal<const ClothTreeTraverser, WarpsPerBlock>(s_warpScratch->bv32PackedNodes, s_warpScratch->sBv32Nodes, ClothTreeTraverser(
			s_warpScratch, aabb, tStackPtr, midphasePairsNum, cmIdx, stackSize));

		/*if (threadIdx.x == 0 && nbPairsPerCM > 0)
		printf("nbPairsPerCM %i\n", nbPairsPerCM);*/
	}
}

extern "C" __global__
//__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 4)
void cloth_midphaseGeneratePairsLaunch(
	PxU32 numWorkItems,
	const PxReal tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgFEMCloth* PX_RESTRICT femClothes,

	PxU8* PX_RESTRICT stackPtr,							//output
	PxU32* PX_RESTRICT midphasePairsNum,				//output
	const PxU32 stackSizeBytes
)
{

	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][WARP_SIZE * 7];

	femClothMidphaseCore<MIDPHASE_WARPS_PER_BLOCK>(
		numWorkItems,
		tolerenceLength,
		cmInputs,
		transformCache,
		bounds,
		contactDistance,
		gpuShapes,
		femClothes,
		stackPtr,
		midphasePairsNum,
		(femMidphaseScratch*)scratchMem[threadIdx.y],
		stackSizeBytes/sizeof(uint4)
		);
}


extern "C" __global__
//__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 4)
void cloth_midphaseGenerateVertexPairsLaunch(
	PxU32 numWorkItems,
	const PxReal tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgFEMCloth* PX_RESTRICT femClothes,

	uint4* PX_RESTRICT stackPtr,							//output
	PxU32* PX_RESTRICT midphasePairsNum					//output
)
{

	const unsigned int cmIdx = blockIdx.y;

	//unsigned mask_cmIdx = __ballot_sync(FULL_MASK, cmIdx < numNPWorkItems);
	if (cmIdx < numWorkItems)
	{
		PxgShape clothShape, rigidShape;
		PxU32 clothCacheRef, rigidCacheRef;
		LoadShapePairWarp<PxGeometryType::eTRIANGLEMESH>(cmInputs, cmIdx, gpuShapes,
			clothShape, clothCacheRef, rigidShape, rigidCacheRef);

		const PxU32 femClothId = clothShape.particleOrSoftbodyId;

		const PxgFEMCloth& femCloth = femClothes[femClothId];

		PxU8 * trimeshGeomPtr = reinterpret_cast<PxU8 *>(femCloth.mTriMeshData);

		trimeshGeomPtr += sizeof(uint4);

		const float4* const PX_RESTRICT meshVerts = femCloth.mPosition_InvMass;
		const PxU32 nbVerts = femCloth.mNbVerts;

		//ML: this is the world bound AABB
		PxBounds3 aabb;
		PxBounds3_ReadWarp(aabb, bounds + rigidCacheRef);
		const PxReal contactDist = contactDistance[clothCacheRef] + contactDistance[rigidCacheRef];
		//aabb.fattenFast(contactDist);

		const PxReal tolerance = contactDist * contactDist;

		const PxU32 idx = threadIdx.x + threadIdx.y*WARP_SIZE + blockIdx.x*blockDim.x*blockDim.y;

		for (PxU32 i = 0; i < nbVerts; i += blockDim.x*blockDim.y*gridDim.x)
		{
			PxU32 id = idx + i;

			bool hasHit = false;
			if (id < nbVerts)
			{
				float4 vertex = meshVerts[id];
				PxVec3 v(vertex.x, vertex.y, vertex.z);

				PxVec3 closest = aabb.closestPoint(v);

				hasHit = (closest - v).magnitudeSquared() <= tolerance;
			}

			PxU32 hitMask = __ballot_sync(FULL_MASK, hasHit);

			if (hitMask)
			{
				PxU32 prevStackShift = 0xFFffFFff;
				PxU32 validCount = __popc(hitMask);
				if (threadIdx.x == 0 && validCount > 0)
				{
					prevStackShift = atomicAdd(midphasePairsNum, validCount);

				}

				prevStackShift = __shfl_sync(FULL_MASK, prevStackShift, 0);

				if (hasHit)
				{
					PxU32 offset = warpScanExclusive(hitMask, threadIdx.x);
					stackPtr[prevStackShift + offset] = make_uint4(cmIdx, id, 0, 0);
				}
			}
		}
	}
}

struct ClothParticleTreeTraverser
{
	femMidphaseScratch* PX_RESTRICT s_warpScratch;
	const PxVec3 sphereCenter;
	const PxReal radius;
	const PxBounds3 mBox;
	const PxU32 globalParticleIndex;
	const PxU32 numParticles;
	const PxU32 cmIdx;
	const PxU32 stackSize;
	uint4* PX_RESTRICT stackPtr;					//output
	PxU32* midphasePairs;							//output
	const bool isMesh;


	PX_FORCE_INLINE __device__ ClothParticleTreeTraverser(
		femMidphaseScratch* PX_RESTRICT s_warpScratch,
		const PxVec3 sphereCenter,
		const PxReal radius,
		const PxU32 globalParticleIndex,
		const PxU32 numParticles,
		const PxU32 cmIdx,
		const PxU32 stackSize,
		uint4* PX_RESTRICT stackPtr,					//output
		PxU32* midphasePairs,							//output
		const bool isMesh) 
		: s_warpScratch(s_warpScratch), sphereCenter(sphereCenter), radius(radius), globalParticleIndex(globalParticleIndex), numParticles(numParticles),
		  cmIdx(cmIdx), stackSize(stackSize), stackPtr(stackPtr), midphasePairs(midphasePairs), isMesh(isMesh), mBox(sphereCenter - PxVec3(radius), sphereCenter + PxVec3(radius))
	{ }

	PX_FORCE_INLINE __device__ void intersectPrimitiveFullWarp(PxU32 primitiveIndex, PxU32 idxInWarp) const
	{
		bool intersect = false;
		if (primitiveIndex != 0xFFFFFFFF) 
		{
			/*if (myFistSBIndex == 933 && tetrahedronIdxSecondSb == 578)
			{
			const PxU32 parentNodeIndex = currentNodeIndex >> 6;
			printf("workIndex % i parentNodeIndex %i blockIdx.x %i tetrahedronIdxSecondSb %i\n", workIndex, parentNodeIndex, blockIdx.x, tetrahedronIdxSecondSb);
			}*/

			uint4 triIdx = s_warpScratch->meshVertsIndices[primitiveIndex];

			const PxVec3 worldV0 = PxLoad3(s_warpScratch->meshVerts[triIdx.x]);
			const PxVec3 worldV1 = PxLoad3(s_warpScratch->meshVerts[triIdx.y]);
			const PxVec3 worldV2 = PxLoad3(s_warpScratch->meshVerts[triIdx.z]);

			/*if ((triangleIdx == 6 || triangleIdx == 7))
			{
				printf("triangleIdx %i worldV0(%f, %f, %f), worldV1(%f, %f, %f), worldV2(%f, %f, %f)\n", triangleIdx, worldV0.x, worldV0.y, worldV0.z,
					worldV1.x, worldV1.y, worldV1.z, worldV2.x, worldV2.y, worldV2.z);
			}*/

			if (isMesh)
			{
				//PxPlane triPlane(worldV0, worldV1, worldV2);
				//get rid of double-sided triangle
				//if (triPlane.distance(sphereCenter) >= 0)
				{
					intersect = triBoundingBox(worldV0, worldV1, worldV2).intersects(mBox);
				}
			}
			else
			{
				intersect = triBoundingBox(worldV0, worldV1, worldV2).intersects(mBox);
			}
		}
	
		pushOntoStackFullWarp(intersect, midphasePairs, stackPtr, stackSize, make_uint4(cmIdx, globalParticleIndex, primitiveIndex, 0));
	}

	PX_FORCE_INLINE __device__ bool intersectBoxFullWarp(bool hasBox, const PxVec3& min, const PxVec3& max) const
	{
		if (hasBox)
			return mBox.intersects(PxBounds3(min, max));
		else
			return false;
	}
};

extern "C" __global__
//__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 4)
void cloth_psMidphaseGeneratePairsLaunch(
	PxU32 numWorkItems,
	const PxReal tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgFEMCloth* PX_RESTRICT clothes,
	PxgParticleSystem* PX_RESTRICT particleSystems,

	const PxU32 stackSizeBytes,
	PxU8* PX_RESTRICT stackPtr,							//output
	PxU32* PX_RESTRICT midphasePairsNum					//output
)
{
	__shared__ __align__(16) PxU8 particleSystemMemory[sizeof(PxgParticleSystem)];
	PxgParticleSystem& sParticleSystem = *(reinterpret_cast<PxgParticleSystem*>(particleSystemMemory));
	__shared__ femMidphaseScratch scratchMem[MIDPHASE_WARPS_PER_BLOCK];

	femMidphaseScratch* s_warpScratch = &scratchMem[threadIdx.y];

	if (blockIdx.y < numWorkItems)
	{
		//const PxU32 globalThreadIdx = threadIdx.y * WARP_SIZE + threadIdx.x + blockIdx.x*blockDim.x*blockDim.y;
		const PxU32 globalWarpIdx = threadIdx.y + blockIdx.x*blockDim.y;

		PxgShape particleShape, clothShape;
		PxU32 particleCacheRef, clothCacheRef;
		LoadShapePairWarp<PxGeometryType::ePARTICLESYSTEM, PxGeometryType::eTRIANGLEMESH>(cmInputs, blockIdx.y, gpuShapes,
			particleShape, particleCacheRef, clothShape, clothCacheRef);

		const PxU32 particleSystemId = particleShape.particleOrSoftbodyId;

		//each thread read 16 byte
		uint4* srcParticleSystem = reinterpret_cast<uint4*>(&particleSystems[particleSystemId]);
		uint4* dstParticleSystem = reinterpret_cast<uint4*>(&sParticleSystem);
		warpCopy<uint4>(dstParticleSystem, srcParticleSystem, sizeof(PxgParticleSystem));


		const PxU32 clohtId = clothShape.particleOrSoftbodyId;
		const PxgFEMCloth& cloth = clothes[clohtId];

		PxU8 * tetmeshGeomPtr = reinterpret_cast<PxU8 *>(cloth.mTriMeshData);

		//const uint4 nbVerts_nbTets_maxDepth_nbBv32TreeNodes = *reinterpret_cast<const uint4 *>(tetmeshGeomPtr);
		tetmeshGeomPtr += sizeof(uint4);

		/*if (workIndex == 0 && threadIdx.x == 0)
		printf("particleSystemId %i softbodyId %i\n", particleSystemId, softbodyId);*/

		if (threadIdx.x == 0)
		{
			s_warpScratch->meshVerts = cloth.mPosition_InvMass;
			s_warpScratch->meshVertsIndices = cloth.mTriangleVertexIndices;


			//const PxU32 & numVerts = nbVerts_nbTets_maxDepth_nbBv32TreeNodes.x;
			//const PxU32 & numTets = nbVerts_nbTets_maxDepth_nbBv32TreeNodes.y;
			//const PxU32 & maxDepth = nbVerts_nbTets_maxDepth_nbBv32TreeNodes.z;
			//const PxU32 & nbBv32PackedNodes = nbVerts_nbTets_maxDepth_nbBv32TreeNodes.w;

			//printf("maxDepth %i numVerts %i numTets %i nbBv32TreeNodes %i\n", maxDepth, numVerts, numTets, nbBv32TreeNodes);

			Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(tetmeshGeomPtr);
			s_warpScratch->bv32PackedNodes = bv32PackedNodes;

			/*tetmeshGeomPtr += sizeof(const Gu::BV32DataPacked)* nbBv32PackedNodes
				+ sizeof(const Gu::BV32DataDepthInfo) * nbVerts_nbTets_maxDepth_nbBv32TreeNodes.z
				+ sizeof(PxU32) * nbVerts_nbTets_maxDepth_nbBv32TreeNodes.w;*/

			/*const PxU8* surfaceHint = reinterpret_cast<const PxU8*>(tetmeshGeomPtr);
			s_warpScratch->tetmeshSurfaceHint = surfaceHint;*/

		}

		__syncthreads();

		PxReal cDistance = contactDistance[particleCacheRef] + contactDistance[clothCacheRef];

		//const PxU32 threadBaseAddress = globalThreadIdx & (~31);

		const PxU32 nbParticles = sParticleSystem.mCommonData.mNumParticles;

		//const PxU32 NbThreads = blockDim.x*blockDim.y*gridDim.x;
		const PxU32 NbWarps = blockDim.y*gridDim.x;

		float4* sortedPose = reinterpret_cast<float4*>(sParticleSystem.mSortedPositions_InvMass);

		uint4* tStackPtr = reinterpret_cast<uint4*>(stackPtr);
		const PxU32 stackSize = stackSizeBytes / sizeof(uint4);

		//for (PxU32 i = threadBaseAddress, particleIndex = globalThreadIdx; i < nbParticles; i+= NbThreads, particleIndex += NbThreads)
		for (PxU32 i = globalWarpIdx; i < nbParticles; i += NbWarps)
		{
			float4 pos = make_float4(0.f);
			//PxU32 particleIndex = 0xFFFFFFFF;

			//if(threadIdx.x == 0)
			PxU32 particleIndex = i;
			pos = sortedPose[particleIndex];


			/*if (particleIndex < nbParticles)
			{
				pos = sortedPose[particleIndex];
			}*/

			const PxVec3 particlePos(pos.x, pos.y, pos.z);

			bv32TreeTraversal<const ClothParticleTreeTraverser, MIDPHASE_WARPS_PER_BLOCK>(s_warpScratch->bv32PackedNodes, s_warpScratch->sBv32Nodes, ClothParticleTreeTraverser(s_warpScratch, particlePos, cDistance, particleIndex,
				nbParticles, blockIdx.y, stackSize, tStackPtr, midphasePairsNum, false));
		}
	}
}



extern "C" __global__
//__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 4)
void cloth_midphaseVertexMeshLaunch(
	PxU32 numWorkItems,
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

	__shared__ femMidphaseScratch scratchMem[MIDPHASE_WARPS_PER_BLOCK];

	femMidphaseScratch* s_warpScratch = &scratchMem[threadIdx.y];

	const PxU32 cmIdx = blockIdx.y;

	//if (cmIdx < numWorkItems)
	{
		//const PxU32 globalThreadIdx = threadIdx.y * WARP_SIZE + threadIdx.x + blockIdx.x*blockDim.x*blockDim.y;
		const PxU32 globalWarpIdx = threadIdx.y + blockIdx.x*blockDim.y;

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

			if (shape0.particleOrSoftbodyId != 0xffffffff)
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
			const PxU8* triGeomPtr = reinterpret_cast<const PxU8 *>(trimeshShape->hullOrMeshPtr);			
			readTriangleMesh(triGeomPtr, s_warpScratch->bv32PackedNodes, s_warpScratch->meshVerts, s_warpScratch->meshVertsIndices);
		}

		__syncthreads();

		PxTransform& trimeshToWorld = trimeshTransformCache.transform;
		const PxMeshScale& trimeshScale = trimeshShape->scale;

		PxReal cDistance = contactDistance[clothCacheRef] + contactDistance[trimeshCacheRef];

		//const PxU32 threadBaseAddress = globalThreadIdx & (~31);

		const PxU32 numVerts = cloth->mNbVerts;

		const float4* positions = cloth->mPosition_InvMass;

		//const PxU32 NbThreads = blockDim.x*blockDim.y*gridDim.x;
		const PxU32 NbWarps = blockDim.y*gridDim.x;

		uint4* tStackPtr = reinterpret_cast<uint4*>(stackPtr);
		const PxU32 stackSize = stackSizeBytes / sizeof(uint4);

		for (PxU32 particleIndex = globalWarpIdx; particleIndex < numVerts; particleIndex += NbWarps)
		{

			const float4 pos = positions[particleIndex];

			//particle pos need to be in triangle mesh vertex space
			const PxVec3 particlePos(pos.x, pos.y, pos.z);

			/// transform those point to triangle vertex space
			const PxVec3 localParticlePos = shape2Vertex(trimeshToWorld.transformInv(particlePos), trimeshScale.scale, trimeshScale.rotation);

			bv32TreeTraversal<const ClothParticleTreeTraverser, MIDPHASE_WARPS_PER_BLOCK>(s_warpScratch->bv32PackedNodes, s_warpScratch->sBv32Nodes, ClothParticleTreeTraverser(s_warpScratch, localParticlePos, cDistance, particleIndex,
				numVerts, cmIdx, stackSize, tStackPtr, midphasePairsNum, true));
		}

	}
}



