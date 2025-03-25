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

#include "foundation/PxBasicTemplates.h"
#include "foundation/PxBounds3.h"
#include "foundation/PxMath.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"

#include "geometry/PxGeometry.h"

#include "GuBV32.h"
#include "cudaNpCommon.h"

#include "PxgContactManager.h"
#include "PxgConvexConvexShape.h"
#include "PxgFEMCloth.h"
#include "PxgNpKernelIndices.h"
#include "PxgParticleSystem.h"
#include "PxgSoftBody.h"

#include "PxsTransformCache.h"

#include <vector_types.h>

#include "copy.cuh"
#include "dataReadWriteHelper.cuh"
#include "femMidphaseScratch.cuh"
#include "utils.cuh"
#include "sbMidphaseScratch.cuh"
#include "deformableCollision.cuh"

#include "bv32Traversal.cuh"

using namespace physx;

extern "C" __host__ void initNarrowphaseKernels13() {}

struct SoftbodyTreeTraverser
{
	sbMidphaseScratch* s_warpScratch;
	const PxBounds3& mBox;
	const PxU32 stackSize;
	uint4* PX_RESTRICT stackPtr;
	PxU32* PX_RESTRICT midphasePairsNum;
	unsigned int cmIdx;

	PX_FORCE_INLINE __device__ SoftbodyTreeTraverser(
		sbMidphaseScratch* s_warpScratch,
		PxBounds3& aabb,
		const PxU32 stackSize,
		uint4* PX_RESTRICT stackPtr,
		PxU32* PX_RESTRICT midphasePairsNum,
		unsigned int cmIdx)
		: s_warpScratch(s_warpScratch), mBox(aabb), stackSize(stackSize), stackPtr(stackPtr), midphasePairsNum(midphasePairsNum), cmIdx(cmIdx)
	{ }

	PX_FORCE_INLINE __device__ void intersectPrimitiveFullWarp(PxU32 primitiveIndex, PxU32 idxInWarp) const
	{
		bool intersect = false;
		if (primitiveIndex != 0xFFFFFFFF) 
		{
			const PxU8 hint = s_warpScratch->tetmeshSurfaceHint[primitiveIndex];

			if (hint)
			{
				uint4 tetIdx = s_warpScratch->tetmeshTetIndices[primitiveIndex];

				const PxVec3 worldV0 = PxLoad3(s_warpScratch->tetmeshVerts[tetIdx.x]);
				const PxVec3 worldV1 = PxLoad3(s_warpScratch->tetmeshVerts[tetIdx.y]);
				const PxVec3 worldV2 = PxLoad3(s_warpScratch->tetmeshVerts[tetIdx.z]);
				const PxVec3 worldV3 = PxLoad3(s_warpScratch->tetmeshVerts[tetIdx.w]);

				if (isValidTet(worldV0, worldV1, worldV2, worldV3))
				{
					intersect = tetBoundingBox(worldV0, worldV1, worldV2, worldV3).intersects(mBox);
				}
			}			
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
__device__ static inline void sbMidphaseCore(
	PxU32 numNPWorkItems,
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgSoftBody* PX_RESTRICT softbodies,

	const PxU32 stackSizeBytes,
	PxU8* PX_RESTRICT stackPtr,									//output
	PxU32* PX_RESTRICT midphasePairsNum,						//output

	sbMidphaseScratch*	s_warpScratch
)
{
	//wrap index
	const unsigned int warpIdx = threadIdx.y;
	unsigned int cmIdx = warpIdx + blockIdx.x * blockDim.y;

	//unsigned mask_cmIdx = __ballot_sync(FULL_MASK, cmIdx < numNPWorkItems);
	if (cmIdx < numNPWorkItems)
	{
		PxgShape softbodyShape, rigidShape;
		PxU32 softbodyCacheRef, rigidCacheRef;
		LoadShapePairWarp<PxGeometryType::eTETRAHEDRONMESH>(cmInputs, cmIdx, gpuShapes,
			softbodyShape, softbodyCacheRef, rigidShape, rigidCacheRef);

		const PxU32 softbodyId = softbodyShape.particleOrSoftbodyId;
		const PxgSoftBody& softbody = softbodies[softbodyId];

		PxU8 * tetmeshGeomPtr = reinterpret_cast<PxU8 *>(softbody.mTetMeshData);

		const uint4 nbVerts_nbTets_maxDepth_nbBv32TreeNodes = *reinterpret_cast<const uint4 *>(tetmeshGeomPtr);
		tetmeshGeomPtr += sizeof(uint4);

		if (threadIdx.x == 0)
		{
			s_warpScratch->tetmeshVerts = softbody.mPosition_InvMass;
			s_warpScratch->tetmeshTetIndices = softbody.mTetIndices;
			s_warpScratch->tetmeshSurfaceHint = softbody.mTetMeshSurfaceHint;
			
			//const PxU32 & numVerts = nbVerts_nbTets_maxDepth_nbBv32TreeNodes.x;
			//const PxU32 & numTets = nbVerts_nbTets_maxDepth_nbBv32TreeNodes.y;
			//const PxU32 & maxDepth = nbVerts_nbTets_maxDepth_nbBv32TreeNodes.z;
			const PxU32 & nbBv32PackedNodes = nbVerts_nbTets_maxDepth_nbBv32TreeNodes.w;

			//printf("maxDepth %i numVerts %i numTets %i nbBv32TreeNodes %i\n", maxDepth, numVerts, numTets, nbBv32TreeNodes);

			Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(tetmeshGeomPtr);
			s_warpScratch->bv32PackedNodes = bv32PackedNodes;

			tetmeshGeomPtr += sizeof(const Gu::BV32DataPacked)* nbBv32PackedNodes
				+ sizeof(const Gu::BV32DataDepthInfo) * nbVerts_nbTets_maxDepth_nbBv32TreeNodes.z
				+ sizeof(PxU32) * nbVerts_nbTets_maxDepth_nbBv32TreeNodes.w;
		}

		__syncwarp();


		//ML: this is the world bound AABB
		PxBounds3 aabb;
		PxBounds3_ReadWarp(aabb, bounds + rigidCacheRef);
		//PxReal contactDist = convexShape.contactOffset + trimeshShape.contactOffset;
		const PxReal contactDist = contactDistance[softbodyCacheRef] + contactDistance[rigidCacheRef];
		aabb.fattenFast(contactDist);

		uint4* tStackPtr = reinterpret_cast<uint4*>(stackPtr);
		const PxU32 stackSize = stackSizeBytes / sizeof(uint4);
		bv32TreeTraversal<const SoftbodyTreeTraverser, WarpsPerBlock>(s_warpScratch->bv32PackedNodes, s_warpScratch->sBv32Nodes, SoftbodyTreeTraverser(s_warpScratch, aabb, stackSize, tStackPtr, midphasePairsNum, cmIdx));

		/*if (threadIdx.x == 0 && nbPairsPerCM > 0)
		printf("nbPairsPerCM %i\n", nbPairsPerCM);*/
	}
}

extern "C" __global__
//__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 4)
void sb_midphaseGeneratePairsLaunch(
	PxU32 numWorkItems,
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgSoftBody* PX_RESTRICT softbodies,
	
	const PxU32 stackSizeBytes,
	PxU8* PX_RESTRICT stackPtr,							//output
	PxU32* PX_RESTRICT midphasePairsNum					//output
)
{

	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][WARP_SIZE * 7];

	sbMidphaseCore<MIDPHASE_WARPS_PER_BLOCK>(
		numWorkItems,
		toleranceLength,
		cmInputs,
		transformCache,
		bounds,
		contactDistance,
		gpuShapes,
		softbodies,
		stackSizeBytes,
		stackPtr,
		midphasePairsNum,
		(sbMidphaseScratch*)scratchMem[threadIdx.y]
		);
}

//struct sbParticleMidphaseScratch
//{
//	const float4 * PX_RESTRICT tetmeshVerts;
//	const uint4 * PX_RESTRICT tetmeshTetIndices;
//	const PxU8* PX_RESTRICT tetmeshSurfaceHint;
//
//	const Gu::BV32DataDepthInfo* PX_RESTRICT bv32DepthInfo;
//	const PxU32* PX_RESTRICT bv32RemapPackedNodeIndex;
//	//bv32 tree
//	Gu::BV32DataPacked* bv32PackedNodes;
//	int nbPackedNodes;
//	//statck for traversal
//	int sBv32Nodes[192]; //6 depth of the bv32 tree
//	PxU32 sBv32ActiveParticles[192];
//};
//PX_COMPILE_TIME_ASSERT(sizeof(sbParticleMidphaseScratch) <= WARP_SIZE * 15 * sizeof(PxU32));




struct SoftbodyBoxTraverser
{
	femMidphaseScratch* PX_RESTRICT s_warpScratch;
	const PxBounds3 mBox;
	const PxU32 queryIndex;
	const PxU32 cmIdx;
	const PxU32 stackSize;
	uint4* PX_RESTRICT stackPtr;					//output
	PxU32* midphasePairs;						   //output

	PX_FORCE_INLINE __device__ SoftbodyBoxTraverser(
		femMidphaseScratch* PX_RESTRICT s_warpScratch,
		const PxVec3 boxMin,
		const PxVec3 boxMax,
		const PxU32 queryIndex,
		const PxU32 cmIdx,
		const PxU32 stackSize,
		uint4* PX_RESTRICT stackPtr,					//output
		PxU32* midphasePairs)							//output
		: s_warpScratch(s_warpScratch), mBox(boxMin, boxMax), queryIndex(queryIndex), cmIdx(cmIdx),
		stackSize(stackSize), stackPtr(stackPtr), midphasePairs(midphasePairs)
	{ }

	PX_FORCE_INLINE __device__ void intersectPrimitiveFullWarp(PxU32 primitiveIndex, PxU32 idxInWarp) const
	{
		bool intersect = false;
		if (primitiveIndex != 0xFFFFFFFF) 
		{
			uint4 tetIdx = s_warpScratch->meshVertsIndices[primitiveIndex];

			const PxVec3 worldV0 = PxLoad3(s_warpScratch->meshVerts[tetIdx.x]);
			const PxVec3 worldV1 = PxLoad3(s_warpScratch->meshVerts[tetIdx.y]);
			const PxVec3 worldV2 = PxLoad3(s_warpScratch->meshVerts[tetIdx.z]);
			const PxVec3 worldV3 = PxLoad3(s_warpScratch->meshVerts[tetIdx.w]);

			if (isValidTet(worldV0, worldV1, worldV2, worldV3))
			{
				intersect = tetBoundingBox(worldV0, worldV1, worldV2, worldV3).intersects(mBox);
			}
		}

		pushOntoStackFullWarp(intersect, midphasePairs, stackPtr, stackSize, make_uint4(cmIdx, queryIndex, primitiveIndex, 0));
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
void sb_psMidphaseGeneratePairsLaunch(
	PxU32 numWorkItems,
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgSoftBody* PX_RESTRICT softbodies,
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
		
		PxgShape particleShape, softbodyShape;
		PxU32 particleCacheRef, softbodyCacheRef;
		LoadShapePairWarp<PxGeometryType::ePARTICLESYSTEM, PxGeometryType::eTETRAHEDRONMESH>(cmInputs, blockIdx.y, gpuShapes,
			particleShape, particleCacheRef, softbodyShape, softbodyCacheRef);

		const PxU32 particleSystemId = particleShape.particleOrSoftbodyId;

		//each thread read 16 byte
		uint4* srcParticleSystem = reinterpret_cast<uint4*>(&particleSystems[particleSystemId]);
		uint4* dstParticleSystem = reinterpret_cast<uint4*>(&sParticleSystem);
		warpCopy<uint4>(dstParticleSystem, srcParticleSystem, sizeof(PxgParticleSystem));


		const PxU32 softbodyId = softbodyShape.particleOrSoftbodyId;
		const PxgSoftBody& softbody = softbodies[softbodyId];

		PxU8 * tetmeshGeomPtr = reinterpret_cast<PxU8 *>(softbody.mTetMeshData);

		//const uint4 nbVerts_nbTets_maxDepth_nbBv32TreeNodes = *reinterpret_cast<const uint4 *>(tetmeshGeomPtr);
		tetmeshGeomPtr += sizeof(uint4);

		/*if (workIndex == 0 && threadIdx.x == 0)
		printf("particleSystemId %i softbodyId %i\n", particleSystemId, softbodyId);*/

		if (threadIdx.x == 0)
		{
			s_warpScratch->meshVerts = softbody.mPosition_InvMass;
			s_warpScratch->meshVertsIndices = softbody.mTetIndices;
	
			Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(tetmeshGeomPtr);
			s_warpScratch->bv32PackedNodes = bv32PackedNodes;

		}

		__syncthreads();

		PxReal cDistance = contactDistance[particleCacheRef] + contactDistance[softbodyCacheRef];

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

			bv32TreeTraversal<const SoftbodyBoxTraverser, MIDPHASE_WARPS_PER_BLOCK>(s_warpScratch->bv32PackedNodes, s_warpScratch->sBv32Nodes, SoftbodyBoxTraverser(s_warpScratch, particlePos - PxVec3(cDistance), particlePos + PxVec3(cDistance), particleIndex,
				blockIdx.y, stackSize, tStackPtr, midphasePairsNum));
		}
	}
}


template<unsigned int WarpsPerBlock>
__device__ static inline void sb_clothVertMidphaseCore(
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgSoftBody* PX_RESTRICT softbodies,
	const PxgFEMCloth* PX_RESTRICT clothes,
	const PxU32 stackSizeBytes,
	const PxReal nbCollisionPairUpdatesPerTimestep,
	PxU8* PX_RESTRICT stackPtr,									//output
	PxU32* PX_RESTRICT midphasePairsNum,						//output
	femMidphaseScratch*	s_warpScratch
)
{
	const PxU32 cmIdx = blockIdx.y;

	// each block deals with one pair
	{
		const PxU32 globalWarpIdx = threadIdx.y + blockIdx.x*blockDim.y;

		PxgShape softbodyShape, clothShape;
		PxU32 softbodyCacheRef, clothCacheRef;
		LoadShapePairWarp<PxGeometryType::eTETRAHEDRONMESH, PxGeometryType::eTRIANGLEMESH>(cmInputs, cmIdx, gpuShapes,
			softbodyShape, softbodyCacheRef, clothShape, clothCacheRef);

		const PxgSoftBody& softbody = softbodies[softbodyShape.particleOrSoftbodyId];
		const PxgFEMCloth& cloth = clothes[clothShape.particleOrSoftbodyId];

		PxU8 * tetmeshGeomPtr = reinterpret_cast<PxU8 *>(softbody.mTetMeshData);
		tetmeshGeomPtr += sizeof(uint4);

		if (threadIdx.x == 0)
		{
			s_warpScratch->meshVerts = softbody.mPosition_InvMass;
			s_warpScratch->meshVertsIndices = softbody.mTetIndices;

			Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(tetmeshGeomPtr);
			s_warpScratch->bv32PackedNodes = bv32PackedNodes;
		}
		
		__syncthreads();

		// if cloth collision pair is updated more than once per time step, early-out the influence of extended
		// cloth bounding box in broad phase.
		if (nbCollisionPairUpdatesPerTimestep > 1.f)
		{
			PxBounds3 clothBB, softbodyBB;
			PxBounds3_ReadWarp(clothBB, bounds + clothCacheRef);
			PxBounds3_ReadWarp(softbodyBB, bounds + softbodyCacheRef);

			clothBB.minimum += (nbCollisionPairUpdatesPerTimestep - 1.f) * PxVec3(contactDistance[clothCacheRef]);
			clothBB.maximum -= (nbCollisionPairUpdatesPerTimestep - 1.f) * PxVec3(contactDistance[clothCacheRef]);

			if (!clothBB.intersects(softbodyBB)) return;
		}

		const PxReal cDistance = contactDistance[softbodyCacheRef] + contactDistance[clothCacheRef];

		const PxU32 NbWarps = blockDim.y*gridDim.x;

		const PxU32 nbVerts = cloth.mNbVerts;

		const float4* positions = cloth.mPosition_InvMass;

		uint4* tStackPtr = reinterpret_cast<uint4*>(stackPtr);
		const PxU32 stackSize = stackSizeBytes / sizeof(uint4);

		for (PxU32 i = globalWarpIdx; i < nbVerts; i += NbWarps)
		{
			const float4 tPos = positions[i];

			const PxVec3 pos(tPos.x, tPos.y, tPos.z);

			bv32TreeTraversal<const SoftbodyBoxTraverser, WarpsPerBlock>(s_warpScratch->bv32PackedNodes, s_warpScratch->sBv32Nodes, SoftbodyBoxTraverser(s_warpScratch, pos - PxVec3(cDistance), pos + PxVec3(cDistance), i,
				blockIdx.y, stackSize, tStackPtr, midphasePairsNum));
		}
	}
}

extern "C" __global__
//__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 4)
void sb_clothVertMidphaseGeneratePairsLaunch(
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgSoftBody* PX_RESTRICT softbodies,
	const PxgFEMCloth* PX_RESTRICT clothes,
	const PxU32 stackSizeBytes,
	const PxReal nbCollisionPairUpdatesPerTimestep,
	PxU8* PX_RESTRICT stackPtr,							//output
	PxU32* PX_RESTRICT midphasePairsNum					//output
)
{
	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][WARP_SIZE * 18];

	sb_clothVertMidphaseCore<MIDPHASE_WARPS_PER_BLOCK>(
		toleranceLength,
		cmInputs,
		transformCache,
		bounds,
		contactDistance,
		gpuShapes,
		softbodies,
		clothes,
		stackSizeBytes,
		nbCollisionPairUpdatesPerTimestep,
		stackPtr,
		midphasePairsNum,
		(femMidphaseScratch*)scratchMem[threadIdx.y]
		);
}
