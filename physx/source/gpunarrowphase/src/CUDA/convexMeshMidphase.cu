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

#include <stdio.h>
#include <stdint.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include "GuIntersectionTriangleBoxRef.h"

#include "foundation/PxVec3.h"
#include "foundation/PxQuat.h"
#include "foundation/PxTransform.h"
#include "geometry/PxMeshScale.h"
#include "geometry/PxGeometry.h"

#include "cudaNpCommon.h"
#include "PxgPersistentContactManifold.h"
#include "PxgContactManager.h"
#include "PxgConvexConvexShape.h"
#include "PxsContactManagerState.h"
#include "PxsTransformCache.h"

#include "utils.cuh"

#include "PxgCommonDefines.h"

using namespace physx;

extern "C" __host__ void initNarrowphaseKernels4() {}

#include "manifold.cuh"
#include "vector_functions.h"

#include "midphaseAllocate.cuh"
#include "contactReduction.cuh"
#include "nputils.cuh"
#include "PxsMaterialCore.h"
#include "PxContact.h"
#include "materialCombiner.cuh"
#include "cudaNpCommon.h"
#include "PxgNpKernelIndices.h"

#include "bv32Traversal.cuh"
#include "sdfCollision.cuh"
#include "triangleMesh.cuh"

#pragma nv_diagnostic push
#pragma nv_diag_suppress 20054 // dynamic initialization is not supported for a function-scope static __shared__ variable within a __device__/__global__ function

struct MidphaseTreeTraverser
{
	MidphaseScratch* s_warpScratch;
	const PxVec3 pMin;
	const PxVec3 pMax;
	PxVec3& shapeCoMInTrimeshVertexSpace;
	uint4* PX_RESTRICT stackBasePtr;
	PxU32* PX_RESTRICT nbPairsFound;
	unsigned int cmIdx; 
	unsigned int shapeMeshIndex; 
	const PxU32 maxPairs;

	PxU32 nbPairsPerCM;

	PX_FORCE_INLINE __device__ MidphaseTreeTraverser(
		MidphaseScratch* s_warpScratch, PxBounds3& aabb_trimeshVertexSpace,
		PxVec3& shapeCoMInTrimeshVertexSpace, uint4* PX_RESTRICT stackBasePtr, PxU32* PX_RESTRICT nbPairsFound,
		unsigned int cmIdx, unsigned int shapeMeshIndex, const PxU32 maxPairs)
		: s_warpScratch(s_warpScratch), pMin(aabb_trimeshVertexSpace.minimum), pMax(aabb_trimeshVertexSpace.maximum), shapeCoMInTrimeshVertexSpace(shapeCoMInTrimeshVertexSpace), stackBasePtr(stackBasePtr),
		nbPairsFound(nbPairsFound), cmIdx(cmIdx), shapeMeshIndex(shapeMeshIndex), maxPairs(maxPairs), nbPairsPerCM(0)
	{ }

	PX_FORCE_INLINE __device__ void intersectPrimitiveFullWarp(PxU32 primitiveIndex, PxU32 idxInWarp)
	{
		bool primitiveIntersects = false;
		if (primitiveIndex != 0xFFFFFFFF) 
		{
			uint4 triIdx = s_warpScratch->trimeshTriIndices[primitiveIndex];

			PxVec3 triV0, triV1, triV2;
			triV0 = PxLoad3(s_warpScratch->trimeshVerts[triIdx.x]);
			triV1 = PxLoad3(s_warpScratch->trimeshVerts[triIdx.y]);
			triV2 = PxLoad3(s_warpScratch->trimeshVerts[triIdx.z]);

			PxPlane triPlane(triV0, triV1, triV2);

			if (triPlane.distance(shapeCoMInTrimeshVertexSpace) >= 0)
			{
				// convex center and extents are in the world
				PxTransform meshToWorld = s_warpScratch->meshToWorld;
				PxMeshScale trimeshScale = s_warpScratch->trimeshScale;

				PxVec3 worldSpaceP0 = meshToWorld.transform(vertex2Shape(triV0, trimeshScale.scale, trimeshScale.rotation));
				PxVec3 worldSpaceP1 = meshToWorld.transform(vertex2Shape(triV1, trimeshScale.scale, trimeshScale.rotation));
				PxVec3 worldSpaceP2 = meshToWorld.transform(vertex2Shape(triV2, trimeshScale.scale, trimeshScale.rotation));

				primitiveIntersects = Gu::intersectTriangleBox_RefImpl<true>(s_warpScratch->center, s_warpScratch->inflatedExtents,
					worldSpaceP0, worldSpaceP1, worldSpaceP2);
			}
		}

		//push the triangle into stack ptr
		PxU32 resultWarp = __ballot_sync(FULL_MASK, primitiveIntersects);
		PxU32 offset = warpScanExclusive(resultWarp, idxInWarp);
		PxU32 validCount = __popc(resultWarp);

		// Allocate only amount of memory needed for single warp-wide write
		PxU32 prevNbPairs = 0xFFffFFff;
		if (idxInWarp == 0 && validCount > 0)
		{
			prevNbPairs = atomicAdd(nbPairsFound, validCount); // even if we might overflow, we increase the stack offset. It's not written though.
		}

		prevNbPairs = __shfl_sync(FULL_MASK, prevNbPairs, 0);

		// only write the pairs that still fit within the max pairs limit
		if (primitiveIntersects && ((prevNbPairs + offset) < maxPairs))
		{
			stackBasePtr[prevNbPairs + offset] = make_uint4(cmIdx, primitiveIndex, nbPairsPerCM + offset, shapeMeshIndex);
		}

		// correct validCount: subtract the overflowing pairs.
		if ((validCount > 0) && ((validCount + prevNbPairs) >= maxPairs))
		{
			validCount = PxMax(maxPairs, prevNbPairs) - prevNbPairs;
		}

		assert(((validCount + prevNbPairs) <= maxPairs) || (validCount == 0));

		nbPairsPerCM += validCount;
	}

	PX_FORCE_INLINE __device__ bool intersectBoxFullWarp(bool hasBox, const PxVec3& min, const PxVec3& max) const
	{
		if (hasBox)
			return !(min.x > pMax.x || pMin.x > max.x || min.y > pMax.y || pMin.y > max.y || min.z > pMax.z || pMin.z > max.z);
		else
			return false;
	}
};

template<unsigned int WarpsPerBlock>
__device__ static inline void midphaseCore(
		PxU32 numNPWorkItems,
		const PxReal toleranceLength,
		const PxgContactManagerInput* PX_RESTRICT cmInputs,
		const PxsCachedTransform* PX_RESTRICT transformCache,
		const PxBounds3* PX_RESTRICT bounds,
		const PxReal* PX_RESTRICT contactDistance,
		const PxgShape* PX_RESTRICT gpuShapes,
		ConvexMeshPair* PX_RESTRICT cvxTrimeshPair,
		PxgPersistentContactMultiManifold* PX_RESTRICT multiManifolds,
		PxsContactManagerOutput* PX_RESTRICT cmOutputs,
		PxU8* PX_RESTRICT stackPtr,
		PxU32* PX_RESTRICT stackOffset,
		PxU32* PX_RESTRICT midphasePairsNum,
		PxU32* PX_RESTRICT midphasePairsNumPadded,

		MidphaseScratch*	s_warpScratch,
		const PxU32 maxPairs
		)
{
	//thread index in warp
	const unsigned int idxInWarp = threadIdx.x;
	//wrap index
	const unsigned int warpIdx = threadIdx.y;//(idx >> LOG2_WARP_SIZE);
	//wrap index in block
	//const unsigned int idx = idxInWarp + warpIdx * WARP_SIZE;

	PxU32 nbPairsPerCM = 0;

	unsigned int cmIdx = warpIdx + blockIdx.x * blockDim.y;

	//unsigned mask_cmIdx = __ballot_sync(FULL_MASK, cmIdx < numNPWorkItems);
	if (cmIdx < numNPWorkItems)
	{
		PxgContactManagerInput npWorkItem;
		PxgContactManagerInput_ReadWarp(npWorkItem, cmInputs, cmIdx);

		PxU32 transformCacheRef0 = npWorkItem.transformCacheRef0;
		PxU32 transformCacheRef1 = npWorkItem.transformCacheRef1;
		PxU32 shapeRef0 = npWorkItem.shapeRef0;
		PxU32 shapeRef1 = npWorkItem.shapeRef1;

		bool swap = gpuShapes[shapeRef0].type == PxGeometryType::eTRIANGLEMESH;

		if(swap)
		{
			PxSwap(transformCacheRef0, transformCacheRef1);
			PxSwap(shapeRef0, shapeRef1);
		}

		PxgShape shape;
		PxgShape_ReadWarp(shape, gpuShapes + shapeRef0);

		PxsCachedTransform transformCached, trimeshTransformCached;
		PxsCachedTransform_ReadWarp(transformCached, transformCache + transformCacheRef0);
		PxsCachedTransform_ReadWarp(trimeshTransformCached, transformCache + transformCacheRef1);

		const PxTransform shapeToMeshNoScale = trimeshTransformCached.transform.transformInv(transformCached.transform);
		PxReal ratio, minMargin, breakingThresholdRatio;
		if (shape.type == PxGeometryType::eSPHERE)
		{
			minMargin = shape.scale.scale.x; //sphere radius
			ratio = 0.02f;
			breakingThresholdRatio = 0.05f;
		}
		else if (shape.type == PxGeometryType::eCAPSULE)
		{
			minMargin = shape.scale.scale.y; //capsule radius;
			ratio = 0.02f;
			breakingThresholdRatio = 0.05f;
		}
		else
		{
			PxU8* hullPtr = reinterpret_cast<PxU8*>(shape.hullOrMeshPtr);

			const float4 extents4_f = *reinterpret_cast<float4*>(hullPtr + sizeof(float4) * 2);
			minMargin = calculatePCMConvexMargin(extents4_f, shape.scale.scale, toleranceLength);
			ratio = 0.2f;
			breakingThresholdRatio = 0.8f;
		}

		bool lostContacts = false;

		bool invalidate = invalidateManifold(shapeToMeshNoScale, multiManifolds[cmIdx], minMargin, ratio);

		if (!invalidate)
		{
			const PxReal projectBreakingThreshold = minMargin * breakingThresholdRatio;

			lostContacts = refreshManifolds(
				shapeToMeshNoScale,
				projectBreakingThreshold,
				multiManifolds + cmIdx
				);
		}

		bool fullContactGen = invalidate || lostContacts;

		PxgShape trimeshShape;
		PxgShape_ReadWarp(trimeshShape, gpuShapes + shapeRef1);

		if (threadIdx.x == 0)
		{
			s_warpScratch->meshToWorld = trimeshTransformCached.transform;
			s_warpScratch->shapeToMeshNoScale = shapeToMeshNoScale;
			s_warpScratch->trimeshScale = trimeshShape.scale;
			s_warpScratch->trimeshShape_materialIndex = trimeshShape.materialIndex;
			s_warpScratch->shape_materialIndex = shape.materialIndex;
		}

		__syncwarp();

		if (fullContactGen)
		{
			PxMeshScale trimeshScale = s_warpScratch->trimeshScale;

			PxVec3 shapeCoMInTrimeshVertexSpace;
			if (shape.type == PxGeometryType::eSPHERE || shape.type == PxGeometryType::eCAPSULE)
			{
				//world space
				PxVec3 sphereCoM = transformCached.transform.p;
				shapeCoMInTrimeshVertexSpace = shape2Vertex(trimeshTransformCached.transform.transformInv(sphereCoM), trimeshScale.scale,
					trimeshScale.rotation);
			}
			else
			{
				const PxU8 * convexGeomPtr = reinterpret_cast<const PxU8 *>(shape.hullOrMeshPtr);

				float4 cvxCoM = *reinterpret_cast<const float4 *>(convexGeomPtr);
				shapeCoMInTrimeshVertexSpace = shape2Vertex(shapeToMeshNoScale.transform(
					vertex2Shape(PxVec3(cvxCoM.x, cvxCoM.y, cvxCoM.z),
						shape.scale.scale, shape.scale.rotation)),
					trimeshScale.scale, trimeshScale.rotation);
			}

			//ML: this is the world bound AABB, we need to transform this bound to the local space of the convex hull
			//PxBounds3 aabb = bounds[npWorkItem.transformCacheRef0];
			PxBounds3 aabb;
			PxBounds3_ReadWarp(aabb, bounds + transformCacheRef0);
			//PxReal contactDist = convexShape.contactOffset + trimeshShape.contactOffset;
			const PxReal contactDist = contactDistance[transformCacheRef0] + contactDistance[transformCacheRef1];

			if (threadIdx.x == 0)
			{
				const PxVec3 inflatedExtents = aabb.getExtents() + PxVec3(contactDist);
				const PxVec3 center = aabb.getCenter();
				s_warpScratch->inflatedExtents = inflatedExtents;
				s_warpScratch->center = center;
			}

			//__threadfence_block(); //there is one fence lower before we use the data.

			PxTransform trimeshTransf = s_warpScratch->meshToWorld;

			aabb.minimum -= trimeshTransf.p;
			aabb.maximum -= trimeshTransf.p;

			PxMat33 worldToTrimeshVertex = trimeshScale.getInverse().toMat33() * PxMat33(trimeshTransf.q.getConjugate());
			PxBounds3 aabb_trimeshVertexSpace = aabb;
			aabb_trimeshVertexSpace.fattenFast(contactDist);
			aabb_trimeshVertexSpace = PxBounds3::transformFast(worldToTrimeshVertex, aabb_trimeshVertexSpace);

			const PxU8 * trimeshGeomPtr = reinterpret_cast<const PxU8 *>(trimeshShape.hullOrMeshPtr);

			if (threadIdx.x == 0)
			{
				uint4 count = readTriangleMesh(trimeshGeomPtr, s_warpScratch->bv32PackedNodes, s_warpScratch->trimeshVerts, s_warpScratch->trimeshTriIndices);
			}

			__syncwarp();

			MidphaseTreeTraverser t(s_warpScratch, aabb_trimeshVertexSpace, shapeCoMInTrimeshVertexSpace,
					reinterpret_cast<uint4*>(stackPtr), stackOffset, cmIdx, shapeRef1, maxPairs);
			bv32TreeTraversal<MidphaseTreeTraverser, WarpsPerBlock>(s_warpScratch->bv32PackedNodes, s_warpScratch->sBv32Nodes, t);
			nbPairsPerCM = t.nbPairsPerCM;

		}//close for full manifold

		// AD: the callback of the tree traverser makes sure we do not overflow the size of the collision stack.
		// it will readjust nbPairsPerCM to make sure we fit directly into the limit.
		// this means the the atomicAdd below will never overflow.

		// the stack write pointer will be higher than the actual number of pairs written though, it will have the total number of pairs we found.

		PxU32 prevIntermArraysOffset = 0xFFffFFff;
		PxU32 prevIntermArraysPaddedOffset = 0xFFffFFff;
		if (idxInWarp == 0 && nbPairsPerCM > 0)
		{
			prevIntermArraysOffset = atomicAdd(midphasePairsNum, nbPairsPerCM);
			prevIntermArraysPaddedOffset = atomicAdd(midphasePairsNumPadded, ((nbPairsPerCM + 3)&(~3)) * 2); // x2 because we need the temp memory for sorting!
		}

		prevIntermArraysOffset = __shfl_sync(FULL_MASK, prevIntermArraysOffset, 0);
		prevIntermArraysPaddedOffset = __shfl_sync(FULL_MASK, prevIntermArraysPaddedOffset, 0);

		ConvexMeshPair pairInfo;
		pairInfo.aToB = s_warpScratch->shapeToMeshNoScale;
		pairInfo.cmIndex = cmIdx;
		pairInfo.startIndex = prevIntermArraysOffset;
		pairInfo.count = fullContactGen ? nbPairsPerCM : CONVEX_TRIMESH_CACHED;
		pairInfo.roundedStartIndex = prevIntermArraysPaddedOffset;
		pairInfo.materialIndices = make_uint2(s_warpScratch->shape_materialIndex, s_warpScratch->trimeshShape_materialIndex);

		ConvexMeshPair_WriteWarp(cvxTrimeshPair + cmIdx, pairInfo);

		assert(*midphasePairsNum <= maxPairs);
	}
}

extern "C" __global__
//__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 4)
void midphaseGeneratePairs(
	PxU32 numContactManagers,
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	ConvexMeshPair* PX_RESTRICT cvxTrimeshPair,
	PxgPersistentContactMultiManifold* PX_RESTRICT multiManifolds,
	PxsContactManagerOutput* PX_RESTRICT cmOutputs,
	PxU8* PX_RESTRICT stackPtr,
	PxU32* PX_RESTRICT stackOffset,
	PxU32* PX_RESTRICT midphasePairsNum,
	PxU32* PX_RESTRICT midphasePairsNumPadded,
	const PxU32 stackSizeBytes // PxGpuDynamicsMemoryConfig::collisionStackSize
)
{
	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][WARP_SIZE * 16];

	const PxU32 maxPairs = calculateMaxPairs(stackSizeBytes, numContactManagers);
	
	midphaseCore<MIDPHASE_WARPS_PER_BLOCK>(
		numContactManagers,
		toleranceLength,
		cmInputs,
		transformCache,
		bounds,
		contactDistance,
		gpuShapes,
		cvxTrimeshPair,
		multiManifolds,
		cmOutputs,
		stackPtr,
		stackOffset,
		midphasePairsNum,
		midphasePairsNumPadded,
		(MidphaseScratch*) scratchMem[threadIdx.y],
		maxPairs
		);
}


#define MESH_MESH_CONTACT_LIMIT 6
#define PATCH_ACCEPTANCE_EPS2 0.85f
#define PATCH_ACCEPTANCE_EPS3 0.9995f

//#define PATCH_ACCEPTANCE_EPS2 0.9995f
//#define PATCH_ACCEPTANCE_EPS3 0.99995f

#define MAX_MESH_MESH_PATCHES2 MAX_MESH_MESH_PATCHES



struct Patch
{
	PxVec4 KeepPoints[MESH_MESH_CONTACT_LIMIT]; //The w component is the separation
	PxVec3 Normal;
	PxReal Deepest;
	PxU32 PointCount;
};

__device__ void reduceContactsSingleWarp(bool patchCreationMode, const PxU32 index, const PxU32 totalContacts, const PxVec3& nor, const PxReal clusterBias,
	const PxVec4* tempPointsS, Patch* patches)
{
	PxU32 currentContactMask = 0;

	PxVec3 pA;
	PxReal sep;
	bool hasContact = false;

	PxU32 nbContacts = 0;

	const PxU32 maxCounter = 100;
	PxU32 counter2 = 0;
	for (PxU32 c = 0; c < totalContacts && counter2 < maxCounter;)
	{
		++counter2;
		//calculate my read index (where I'm offset from...)
		PxU32 readMask = ~currentContactMask;
		bool needsContact = patchCreationMode ? readMask & (1 << threadIdx.x) : !hasContact;
		PxU32 readIndex = c + warpScanExclusive(readMask, threadIdx.x);

		if (needsContact && readIndex < totalContacts)
		{
			PxVec4 p = tempPointsS[readIndex];
			pA = p.getXYZ();
			sep = p.w;
			hasContact = true;
		}

		c += __popc(readMask);

		currentContactMask = __ballot_sync(FULL_MASK, hasContact); //Mask every thread that has a contact!
		currentContactMask = contactReduce2<true, true, MESH_MESH_CONTACT_LIMIT, false>(pA, sep, nor, currentContactMask, clusterBias);
		hasContact = currentContactMask & (1 << threadIdx.x);
	}

	//Find the deepestS penetration value...
	PxReal deepest;
	minIndex(sep, currentContactMask, deepest);
	patches[index].Deepest = deepest; //Don't use patches[index].Deepest directly for minIndex(...) since it is shared memory

	nbContacts = __popc(currentContactMask);

	assert(nbContacts <= PXG_MAX_NUM_POINTS_PER_CONTACT_PATCH);

	//Now output to the retained patch...
	if (threadIdx.x == 0)
	{
		patches[index].PointCount = nbContacts;
	}

	if (currentContactMask & (1 << threadIdx.x))
	{
		PxU32 offset = warpScanExclusive(currentContactMask, threadIdx.x);
		patches[index].KeepPoints[offset] = PxVec4(pA, sep);
	}
}

struct PxgPatchWriter
{
	PxContactPatch* patches;
	float4* contacts;
};

PX_FORCE_INLINE __device__ PxgPatchWriter createPatchWriterFullThreadBlock(
	PxU32 pairIndex,
	PxsContactManagerOutput* PX_RESTRICT cmOutputs,
	PxU8* PX_RESTRICT contactStream,
	PxU8* PX_RESTRICT patchStream,
	PxgPatchAndContactCounters* PX_RESTRICT patchAndContactCounters,
	PxU32* touchChangeFlags,
	PxU32* patchChangeFlags,
	PxU8* startContactPatches,
	PxU8* startContactPoints,
	PxU8* startContactForces,
	PxU32 patchBytesLimit,
	PxU32 contactBytesLimit,
	PxU32 forceBytesLimit,
	PxU32 numPatchesToKeep,
	PxU32 totalNumContacts)
{
	__shared__ PxU32 patchByteOffsetS;
	__shared__ PxU32 contactByteOffsetS;
	__shared__ PxU32 forceAndIndiceByteOffsetS;

	if (threadIdx.x == 0)
	{
		patchByteOffsetS = 0xFFFFFFFF;
		contactByteOffsetS = 0xFFFFFFFF;
		forceAndIndiceByteOffsetS = 0xFFFFFFFF;
	}
	
	__syncthreads();

	PxsContactManagerOutput* output = cmOutputs + pairIndex;

	PxU32 allflags = reinterpret_cast<PxU32*>(&output->allflagsStart)[0];
	PxU8 oldStatusFlags = u16Low(u32High(allflags));
	PxU8 statusFlags = oldStatusFlags;

	statusFlags &= (~PxsContactManagerStatusFlag::eTOUCH_KNOWN);

	if (numPatchesToKeep != 0)
		statusFlags |= PxsContactManagerStatusFlag::eHAS_TOUCH;
	else
		statusFlags |= PxsContactManagerStatusFlag::eHAS_NO_TOUCH;

	PxU8 prevPatches = u16High(u32Low(allflags));
	bool overflow = false;
	
	if (threadIdx.x == 0)
	{
		/*if(numPatchesToKeep != 0)
			printf("TotalKeepPatches = %i, totalNumContacts = %i, totalGenerated = %i\n", numPatchesToKeep, totalNumContacts, totalGenerated);*/
		if (totalNumContacts)
		{
			patchByteOffsetS = atomicAdd(&(patchAndContactCounters->patchesBytes), sizeof(PxContactPatch)*numPatchesToKeep);
			contactByteOffsetS = atomicAdd(&(patchAndContactCounters->contactsBytes), sizeof(PxContact) * totalNumContacts);
			forceAndIndiceByteOffsetS = atomicAdd(&(patchAndContactCounters->forceAndIndiceBytes), sizeof(PxU32) * totalNumContacts);

			if (patchByteOffsetS + sizeof(PxContactPatch) * numPatchesToKeep > patchBytesLimit)
			{
				patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::PATCH_BUFFER_OVERFLOW);
				patchByteOffsetS = 0xFFFFFFFF;
				overflow = true;
			}

			if (contactByteOffsetS + sizeof(PxContact) * totalNumContacts > contactBytesLimit)
			{
				patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::CONTACT_BUFFER_OVERFLOW);
				contactByteOffsetS = 0xFFFFFFFF;
				overflow = true;
			}
			if ((forceAndIndiceByteOffsetS + sizeof(PxU32) * totalNumContacts * 2) > forceBytesLimit)
			{
				patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::FORCE_BUFFER_OVERFLOW);
				forceAndIndiceByteOffsetS = 0xFFFFFFFF;
				overflow = true;
			}

			if (overflow)
			{
				numPatchesToKeep = totalNumContacts = 0;

				statusFlags &= (~PxsContactManagerStatusFlag::eTOUCH_KNOWN);
				statusFlags |= PxsContactManagerStatusFlag::eHAS_NO_TOUCH;
			}

			//printf("threadIdex0 nbManifolds %i\n", nbManifolds);
		}


		{
			bool previouslyHadTouch = oldStatusFlags & PxsContactManagerStatusFlag::eHAS_TOUCH;
			bool prevTouchKnown = oldStatusFlags & PxsContactManagerStatusFlag::eTOUCH_KNOWN;
			bool currentlyHasTouch = numPatchesToKeep != 0;

			const bool change = (previouslyHadTouch ^ currentlyHasTouch) || (!prevTouchKnown);
			touchChangeFlags[pairIndex] = change;
			patchChangeFlags[pairIndex] = (prevPatches != numPatchesToKeep);
		}

		{
			reinterpret_cast<PxU32*>(&output->allflagsStart)[0] = merge(merge(prevPatches, statusFlags),
				merge(numPatchesToKeep, PxU8(0)));
			output->nbContacts = totalNumContacts;

			if (!overflow)
			{
				output->contactForces = reinterpret_cast<PxReal*>(startContactForces + forceAndIndiceByteOffsetS);
				output->contactPatches = startContactPatches + patchByteOffsetS;
				output->contactPoints = startContactPoints + contactByteOffsetS;
			}
			else
			{
				output->contactForces = 0;
				output->contactPatches = 0;
				output->contactPoints = 0;
			}
		}
	}

	__syncthreads();

	PxgPatchWriter result;
	result.patches = patchByteOffsetS != 0xFFFFFFFF ? reinterpret_cast<PxContactPatch*>(patchStream + patchByteOffsetS) : NULL;
	result.contacts = contactByteOffsetS != 0xFFFFFFFF ? reinterpret_cast<float4*>(contactStream + contactByteOffsetS) : NULL;

	return result;
}

struct PxgContactMaterialProperties
{
	PxReal staticFriction;
	PxReal dynamicFriction;
	PxReal combinedRestitution;
	PxReal damping;
	PxU32 materialFlags;
	PxU32 materialIndex0;
	PxU32 materialIndex1;
};

PX_FORCE_INLINE __device__ void getContactMaterialProperties(const PxgShape& shape0, const PxgShape& shape1, const PxsMaterialData* PX_RESTRICT materials, PxgContactMaterialProperties& result)
{
	combineMaterials(materials, shape0.materialIndex, shape1.materialIndex,
		result.materialFlags, result.staticFriction, result.dynamicFriction,
		result.combinedRestitution, result.damping);
}

PX_FORCE_INLINE __device__ void writeContactPatch(const PxgPatchWriter& writer, PxU32 patchIndex, PxU32 startContactIndex,
	const PxVec3& patchNormal, PxU32 nbContactPoints, const PxgContactMaterialProperties& cmp)
{
	if (writer.patches) 
	{
		PxContactPatch& patch = writer.patches[patchIndex];
		patch.normal = patchNormal;
		patch.nbContacts = nbContactPoints;
		patch.startContactIndex = startContactIndex;
		patch.staticFriction = cmp.staticFriction;
		patch.dynamicFriction = cmp.dynamicFriction;
		patch.damping = cmp.damping;
		patch.restitution = cmp.combinedRestitution;
		patch.materialIndex0 = cmp.materialIndex0;
		patch.materialIndex1 = cmp.materialIndex1;
		patch.materialFlags = cmp.materialFlags;
		patch.mMassModification.linear0 = 1.f;
		patch.mMassModification.linear1 = 1.f;
		patch.mMassModification.angular0 = 1.f;
		patch.mMassModification.angular1 = 1.f;
		patch.internalFlags = 0;
	}
}

template<PxU32 NbWarps>
PX_FORCE_INLINE __device__ void writePatchesAndContacts(
	const PxsCachedTransform& transform0, const PxsCachedTransform& transform1,
	const PxgShape& shape0, const PxgShape& shape1,
	bool isFlipped,
	PxU32 pairIndex,
	PxsContactManagerOutput* PX_RESTRICT cmOutputs,
	const PxsMaterialData* PX_RESTRICT materials,
	PxU8* PX_RESTRICT contactStream,
	PxU8* PX_RESTRICT patchStream,
	PxgPatchAndContactCounters* PX_RESTRICT patchAndContactCounters,

	PxU32* PX_RESTRICT touchChangeFlags,
	PxU32* PX_RESTRICT patchChangeFlags,
	PxU8* PX_RESTRICT startContactPatches,
	PxU8* PX_RESTRICT startContactPoints,
	PxU8* PX_RESTRICT startContactForces,
	PxU32 patchBytesLimit,
	PxU32 contactBytesLimit,
	PxU32 forceBytesLimit,

	PxU32 numPatchesToKeep,
	const Patch* patchesS)
{
	__shared__ PxU32 patchStartIndexS[MAX_MESH_MESH_PATCHES2];

	PxU32 count = 0;
	if (threadIdx.x < numPatchesToKeep)
		count = patchesS[threadIdx.x].PointCount;
		
	assert(count <= PXG_MAX_NUM_POINTS_PER_CONTACT_PATCH);

	PxU32 totalSum;
	PxU32 id = threadBlockScanExclusive<NbWarps>(count, totalSum);	
	if (threadIdx.x < MAX_MESH_MESH_PATCHES2)
		patchStartIndexS[threadIdx.x] = id;	

	//No need for __syncthreads() here because there is a call to __syncthreads() inside createPatchWriterFullThreadBlock

	PxgPatchWriter patchWriter = createPatchWriterFullThreadBlock(pairIndex,
		 cmOutputs,	
		 contactStream,
		 patchStream,
		 patchAndContactCounters,
		 touchChangeFlags,
		 patchChangeFlags,
		 startContactPatches,
		 startContactPoints,
		 startContactForces,
		 patchBytesLimit,
		 contactBytesLimit,
		 forceBytesLimit, numPatchesToKeep, totalSum);

	if (threadIdx.x < numPatchesToKeep)
	{
		PxgContactMaterialProperties contactMaterialProperties;
		getContactMaterialProperties(isFlipped ? shape1 : shape0, isFlipped ? shape0 : shape1, materials, contactMaterialProperties);

		const Patch& p = patchesS[threadIdx.x];
		PxVec3 normal = transform1.transform.rotate(p.Normal);
		writeContactPatch(patchWriter, threadIdx.x, patchStartIndexS[threadIdx.x], isFlipped ? normal : -normal, p.PointCount, contactMaterialProperties);			
	}

	PxU32 patchIndex = (threadIdx.x / MESH_MESH_CONTACT_LIMIT);
	if (patchIndex < numPatchesToKeep)
	{
		const Patch& patch = patchesS[patchIndex];
		PxU32 idx = threadIdx.x % MESH_MESH_CONTACT_LIMIT;
		if (idx < patch.PointCount)
		{
			PxU32 writeIndex = patchStartIndexS[patchIndex] + idx;
			//printf("patchIndex %i: TId = %i, count = %i, writeIndex = %i\n", patchIndex, idx, patchCountsS[patchIndex], writeIndex);
			PxVec4 p = patch.KeepPoints[idx];
			PxVec3 pos = transform1.transform.transform(p.getXYZ());
			if (patchWriter.contacts)
				patchWriter.contacts[writeIndex] = make_float4(pos.x, pos.y, pos.z, p.w);
		}
	}
}

struct Contact
{
	PxVec3 pos = PxVec3(0.f);
	PxReal separation = PX_MAX_F32;
	PxVec3 normal = PxVec3(0.f);
	bool candidateContact = false;

	PX_FORCE_INLINE __device__ void markAsProcessed()
	{
		candidateContact = false;
		separation = PX_MAX_F32;
		normal = PxVec3(0.f);
	}
};

//Must be called by full warps
PX_FORCE_INLINE __device__ void tryToAssignContactToExistingPatch(Patch* patchesS, PxU32 p, Contact& contact, bool& anyKeepS, PxReal clusterBias,
	PxU32* countersS, PxReal* shMinSepS, PxI32& remainingContacts, PxVec4* tempPointsS)
{
	const PxU32 threadIndexInWarp = threadIdx.x & 31;
	const PxU32 warpIndex = threadIdx.x / 32;

	assert(p < MAX_MESH_MESH_PATCHES2);
	Patch& patch = patchesS[p];

	PxReal proj = contact.normal.dot(patch.Normal);

	//Normals are very close, this point is now a candidate for an existing patch...
	bool accept2 = proj > PATCH_ACCEPTANCE_EPS2;
	bool accept = proj > PATCH_ACCEPTANCE_EPS3;

	//We are going to eat this contact, but first scale its separation value by the projection onto the normal.
	//This is important to avoid instabilities due to approximating normals

	//Don't move the definition of keepAny below to work on accept2 since it will cause artificial friction!
	PxU32 reduceMask = __ballot_sync(FULL_MASK, accept);

	if (threadIndexInWarp == 0 && reduceMask)
	{
		anyKeepS = true;
	}

	// syncthreads() but with explicit barrier number to make sure we really have the whole block waiting here.
	asm volatile ("bar.sync 1;");

	//if (accept2)
	//{
	//	//If we are going to process this patch in an existing patch,
	//	//we ensure that it's either acceptable for this patch, or it is not deeper penetrated
	//	//than the deepest value in that patch.
	//	accept2 = accept || contact.separation > patch.Deepest;
	//}

	// AD: attention - we branch on a shared memory variable. Proper synchronization needs to be enforced, for all we know
	// some warp could be waiting right after that last barrier while another one is already around the loop!
	if (anyKeepS)
	{
		reduceMask = contactReduce2<true, true, MESH_MESH_CONTACT_LIMIT, false>(contact.pos, contact.separation, patch.Normal, reduceMask, clusterBias);

		PxU32 nbUsed = threadBlockCountAndBroadcast(accept2, countersS);
		__syncthreads(); // sync for countersS reuse.

		remainingContacts -= nbUsed;

		PxU32 totalContacts;
		bool threadEmitsElement = reduceMask & (1 << threadIndexInWarp);

		PxU32 offset = threadBlockScanExclusive<32>(threadEmitsElement, totalContacts, countersS);
		// no sync for countersS reuse because we sync below after setting the tempPoints.

		// AD: we should always have contacts because if the reduceMask of all warps is 0, we should never end up in here.
		assert(totalContacts != 0);
		assert(totalContacts <= 198);

		//We need to append to existing contacts and reduce...
		if (threadIdx.x < patch.PointCount)
			tempPointsS[threadIdx.x] = patch.KeepPoints[threadIdx.x];

		//Now append more stuff
		offset += patch.PointCount;

		if (threadEmitsElement)
			tempPointsS[offset] = PxVec4(contact.pos, contact.separation);

		totalContacts += patch.PointCount; //Don't access patch.PointCount after __syncthreads(): reduceContactsSingleWarp potentially changed patch.PointCount	
		__syncthreads();

		//Now we have up to 198 contacts that we need to reduce. For now, let's do this by reducing with a single warp,
		//consuming all the contacts...
		if (warpIndex == 0)
			reduceContactsSingleWarp(false, p, totalContacts, patch.Normal, clusterBias, tempPointsS, patchesS);	

		if (accept2)
			contact.markAsProcessed();
	}

	// AD: we need a barrier to make sure that all threads have executed the if above.
	// Otherwise we might change the flag and not all the threads will do the same thing.
	asm volatile ("bar.sync 2;");

	if (threadIdx.x == 0)
		anyKeepS = false;

	// we need to make sure all the warps are in the same iteration because of the scratchpads!
	// we could also have runaway warps messing with the shared memory anyKeepS
	__syncthreads();	
}

//Must be called by full warps
PX_FORCE_INLINE __device__ void createNewPatches(Patch* patchesS, Contact& contact, bool& anyKeepS, PxReal clusterBias,
	PxU32* countersS, PxI32& remainingContacts, PxVec4* tempPointsS, PxReal* shMinSepS, PxU32& numPatchesToKeep)
{
	const PxU32 threadIndexInWarp = threadIdx.x & 31;
	const PxU32 warpIndex = threadIdx.x / 32;


	//When we get here, we have the remaining set of contacts that were not consumed by
	//existing patches, so try to make new patches...

	bool hasAnyContacts = remainingContacts > 0;

	while (hasAnyContacts && (numPatchesToKeep < MAX_MESH_MESH_PATCHES2))
	{
		hasAnyContacts = false;

		//Adds/removes a tiny bias based on projection onto average normal. 
		PxReal minSep = contact.separation;

		PxU32 candidateMask = __ballot_sync(FULL_MASK, contact.candidateContact);

		const PxU32 nbWarpContacts = __popc(candidateMask);

		PxU32 minI = minIndex(minSep, candidateMask, minSep);

		if (threadIndexInWarp == 0)
		{
			shMinSepS[warpIndex] = minSep;
			countersS[warpIndex] = nbWarpContacts;
		}

		// syncthreads() but with explicit barrier number to make sure we really have the whole block waiting here.
		asm volatile ("bar.sync 3;");

		//Now, find the smallest separation across all threads...
		minSep = shMinSepS[threadIndexInWarp];

		PxU32 minIBlock = minIndex(minSep, FULL_MASK, minSep);

		// AD: attention, if on shared memory again!
		if (countersS[minIBlock] != 0)
		{
			Patch& patch = patchesS[numPatchesToKeep];

			hasAnyContacts = true;
			bool myNormal = false;
			if (warpIndex == minIBlock && threadIndexInWarp == minI)
			{
				myNormal = true;
				patch.Normal = contact.normal;
			}

			// __syncthreads(); but with explicit barrier number to make sure we really have the whole block entering the if and waiting here.
			asm volatile ("bar.sync 4;");

			//Now place any candidate contacts into the reduction buffer...

			bool accept = false;
			bool accept2 = false;
			if (contact.candidateContact)
			{
				PxReal proj = contact.normal.dot(patch.Normal);
				accept2 = proj > PATCH_ACCEPTANCE_EPS2;
				accept = proj > PATCH_ACCEPTANCE_EPS3 || myNormal;
			}

			//Now let's output to the temp accum buffer, then run reduction on it

			PxU32 reduceMask = __ballot_sync(FULL_MASK, accept);
			reduceMask = contactReduce2<true, true, MESH_MESH_CONTACT_LIMIT, false>(contact.pos, contact.separation, patch.Normal, reduceMask, clusterBias);

			//Now we have a mask of how many contacts match this patch...					
			PxU32 totalContacts;
			bool threadEmitsElement = reduceMask & (1 << threadIndexInWarp);
			PxU32 offset = threadBlockScanExclusive<32>(threadEmitsElement, totalContacts, countersS);
			// sync for countersS reuse is covered by the end-of-loop barrier.

			if (totalContacts != 0)
			{
				if (threadEmitsElement)
					tempPointsS[offset] = PxVec4(contact.pos, contact.separation);

				__syncthreads();

				assert(totalContacts <= 192);

				//Now we have up to 192 contacts that we need to reduce. For now, let's do this by reducing with a single warp,
				//consuming all the contacts...
				if (warpIndex == 0)
					reduceContactsSingleWarp(true, numPatchesToKeep, totalContacts, patch.Normal, clusterBias, tempPointsS, patchesS);

				__syncthreads();

				if (patchesS[numPatchesToKeep].PointCount)
					numPatchesToKeep++;
			}

			if (accept2)
				contact.markAsProcessed();
		}

		// we need to sync here as well because otherwise the if condition might be overwritten by
		// warps running ahead.
		// if you wonder why this is necessary even though we already have a blocking wait inside the if,
		// you need to remember that the if condition could be false, and if we don't block here
		// a warp could run ahead and make it true before everyone is past the branch.
		asm volatile ("bar.sync 5;");
	}
}

__device__ void doTriangleTriangleCollision(const PxsCachedTransform& transform0, const PxsCachedTransform& transform1,
	const PxgTriangleMesh& mesh0, const PxgTriangleMesh& mesh1, const PxgShape& shape0, const PxgShape& shape1, const PxReal contactDistance, bool isFlipped,
	PxU32 pairIndex,
	PxsContactManagerOutput* PX_RESTRICT cmOutputs,
	const PxsMaterialData* PX_RESTRICT materials,
	PxU8* PX_RESTRICT contactStream,
	PxU8* PX_RESTRICT patchStream,
	PxgPatchAndContactCounters* PX_RESTRICT patchAndContactCounters,
	PxU32* touchChangeFlags,
	PxU32* patchChangeFlags,
	PxU8* startContactPatches,
	PxU8* startContactPoints,
	PxU8* startContactForces,
	PxU32 patchBytesLimit,
	PxU32 contactBytesLimit,
	PxU32 forceBytesLimit,
	const PxReal clusterBias,
	bool generateContacts)
{
	//This buffer stores the set of contacts we want to carry over between iterations/eventually report to the application

	////Up to MAX_MESH_MESH_PATCHES2 contact patches
	__shared__ Patch patchesS[MAX_MESH_MESH_PATCHES2];

	//This is a temporary accumulation buffer used to a given patch. It is large enough to
	//include 6 contacts from each warp plus 6 contacts from the kept contacts. It does not need
	//multiple normals as all contacts being considered as a patch at this stage have very similar
	//normals. The w component is the separation.
	__shared__ PxVec4 tempPointsS[33 * MESH_MESH_CONTACT_LIMIT];

	//Per-warp counter. Used for various stages of reduction...
	__shared__ PxU32 countersS[32];
	//Per-warp minSep value, used to select the warp with the deepest point. This chooses the patch we attempt
	//to reduce to
	__shared__ PxReal shMinSepS[32];
	__shared__ uint2 triangleIndicesS[1024];

	//If a triangle is very big compared to the SDF object that collides against it, then schedule a second collision pass where the triangle gets subdivided
	__shared__ uint2 trianglesToSubdivideS[256]; //This buffer can only hold up to one quarter of the elements as triangleIndicesS because each triangle can produce up to 4 sub-triangles during on-the-fly refinement
	__shared__ PxU32 numTrianglesToSubdivideS;

	__shared__ __align__(16) char sSdfTexture[sizeof(SparseSDFTexture)];
	SparseSDFTexture& sdfTexture = reinterpret_cast<SparseSDFTexture&>(*sSdfTexture);

	if (threadIdx.x == 0)
		numTrianglesToSubdivideS = 0;

	__shared__ bool anyKeepS;
	PxU32 numPatchesToKeep = 0;

	const PxTransform aToB = transform1.transform.transformInv(transform0.transform);

	if (threadIdx.x == 0)
		anyKeepS = false;

	__syncthreads();
	

	PxU32 totalGenerated = 0;
	
	//mesh0 is first the triangle mesh while mesh1 is first the SDF mesh
	//In the second loop iteration it is the other way round
	for (PxU32 outer = 0; outer < 2 && generateContacts; outer++)
	{
		PxReal cullScale = contactDistance;
		PxU32 nbTriangles = 0;			

		if (outer == 0)
		{
			nbTriangles = mesh0.numTris;
			cullScale /= PxMin(shape1.scale.scale.x, PxMin(shape1.scale.scale.y, shape1.scale.scale.z));
			if (threadIdx.x == 0)
				sdfTexture.initialize(mesh1); //The texture is stored in shared memory - only one threads needs to initialize it
		}
		else
		{
			if (mesh0.mTexObject /*&& totalGenerated < PxMax(32u, PxMin((mesh0.numVerts + 7)/8, 256u))*/)
			{
				nbTriangles = mesh1.numTris;
				cullScale /= PxMin(shape0.scale.scale.x, PxMin(shape0.scale.scale.y, shape0.scale.scale.z));
				if (threadIdx.x == 0)
					sdfTexture.initialize(mesh0); //The texture is stored in shared memory - only one threads needs to initialize it
			}
			else break;
		}
		__syncthreads();

		PxU32 maxRefinementLevel = 8;
		PxReal refinementRatioSquared = 16 * 16;

		PxU32 numTrianglesToSubdividePerThread = 0;
		
		for (PxU32 i = 0; (i < nbTriangles || numTrianglesToSubdivideS > 0);)
		{
			Contact contact;

			PxU32 nbFoundTriangles = 0;
			
			if (outer == 0)
			{
				nbFoundTriangles = findInterestingTrianglesA<32, 1024>(mesh0.numTris, mesh0.indices, mesh0.trimeshVerts, shape0.scale, shape1.scale, 
					cullScale, sdfTexture, aToB, i, triangleIndicesS, numTrianglesToSubdividePerThread, trianglesToSubdivideS);
			}
			else
			{
				nbFoundTriangles = findInterestingTrianglesA<32, 1024>(mesh1.numTris, mesh1.indices, mesh1.trimeshVerts, shape1.scale, shape0.scale,
					cullScale, sdfTexture, aToB.getInverse(), i, triangleIndicesS, numTrianglesToSubdividePerThread, trianglesToSubdivideS);
			}
			numTrianglesToSubdividePerThread = 0;

			PxU32 ind = threadIdx.x < nbFoundTriangles ? triangleIndicesS[threadIdx.x].x : nbTriangles;
			PxU32 subInd = threadIdx.x < nbFoundTriangles ? triangleIndicesS[threadIdx.x].y : 0;
			
			bool needsRefinement = false;
			if (ind < nbTriangles)
			{
				if (outer == 0)
				{
					PxVec3 v0, v1, v2;
					getTriangleVertices(mesh0, shape0.scale, aToB, ind, subInd, v0, v1, v2);
					v0 = shape2Vertex(v0, shape1.scale.scale, shape1.scale.rotation);
					v1 = shape2Vertex(v1, shape1.scale.scale, shape1.scale.rotation);
					v2 = shape2Vertex(v2, shape1.scale.scale, shape1.scale.rotation);

					PxVec3 dir;
					PxReal sep = doTriangleSDFCollision(sdfTexture, v0, v1, v2,
						contact.pos, dir, cullScale);

					if (mesh0.mTexObject == NULL)
					{
						//Check if the triangle needs subdivision
						PxReal triRadius2 = triangleRadiusSquared(v0, v1, v2);
						PxReal sdfRadius2 = sdfRadiusSquared(sdfTexture);
						needsRefinement = triRadius2 * refinementRatioSquared > sdfRadius2;
					}

					contact.pos = vertex2Shape(contact.pos, shape1.scale.scale, shape1.scale.rotation);

					//if (bounds.contains(pos))
					{	
						if (sep < cullScale)
						{
							//dir.normalize(); //dir already normalized

							dir = vertex2ShapeNormalVector(dir, shape1.scale.scale, shape1.scale.rotation);

							PxReal m = dir.magnitudeSquared();

							if (mesh0.mTexObject == NULL)
							{
								// Only one of the objects has an SDF. This should only happen if we're colliding
								// against a flat, floor-like static triangle mesh because in the following, the
								// contact normal will be defined as the triangle normal.

								//Use the triangle's normal as direction instead of the direction from the SDF collision
								//But still use the magnitude from the original dir to compute the separation
								PxVec3 n = (v1 - v0).cross(v2 - v0);
								PxVec3 sdfBoxCenter = 0.5f * (sdfTexture.sdfBoxHigher + sdfTexture.sdfBoxLower);
								PxReal triangleNormalSign = -PxSign((sdfBoxCenter - v0).dot(n));
								PxVec3 dir2 = triangleNormalSign * vertex2ShapeNormalVector(n, shape1.scale.scale, shape1.scale.rotation);
								if (dir2.magnitudeSquared() > 1e-8f)
								{
									dir2.normalize();
									dir = dir2 * PxSqrt(m);
								}
							}

							if (m > 0.0f)
							{
								m = 1.0f / PxSqrt(m);
								sep = sep * m;
								dir = dir * m; 
							}

							contact.candidateContact = sep < contactDistance;
								
							if (contact.candidateContact)
							{
								contact.separation = sep;
								contact.normal = -dir; //PxVec3(0.0f, -1.0f, 0.0f); //
							}
						}
					}					
				}
				else 
				{
					uint4 inds = mesh1.indices[ind];
					PxVec3 v0 = vertex2Shape(PxLoad3(mesh1.trimeshVerts[inds.x]), shape1.scale.scale, shape1.scale.rotation);
					PxVec3 v1 = vertex2Shape(PxLoad3(mesh1.trimeshVerts[inds.y]), shape1.scale.scale, shape1.scale.rotation);
					PxVec3 v2 = vertex2Shape(PxLoad3(mesh1.trimeshVerts[inds.z]), shape1.scale.scale, shape1.scale.rotation);

					PxVec3 dir;
					PxReal sep = doTriangleSDFCollision(sdfTexture,
						shape2Vertex(aToB.transformInv(v0), shape0.scale.scale, shape0.scale.rotation),
						shape2Vertex(aToB.transformInv(v1), shape0.scale.scale, shape0.scale.rotation),
						shape2Vertex(aToB.transformInv(v2), shape0.scale.scale, shape0.scale.rotation),
						contact.pos, dir, cullScale);

					contact.pos = aToB.transform(vertex2Shape(contact.pos, shape0.scale.scale, shape0.scale.rotation));

					
					if (sep < cullScale)
					{
						//dir.normalize();

						dir = vertex2ShapeNormalVector(dir, shape0.scale.scale, shape0.scale.rotation);

						PxReal m = dir.magnitudeSquared();
						if (m > 0.0f)
						{
							m = 1.0f / PxSqrt(m);
							sep = sep * m;
							dir = dir * m;
						}

						contact.candidateContact = sep < contactDistance;

						if (contact.candidateContact)
						{
							contact.separation = sep;
							contact.normal = aToB.rotate(dir);
						}
					}										
				}
			}
			//needsRefinement = false;
			PxU32 newBufferCount = addToRefinementBuffer<32, 256>(needsRefinement, ind, subInd, 0, trianglesToSubdivideS, maxRefinementLevel);
			if (threadIdx.x == 0)
				numTrianglesToSubdivideS = newBufferCount;

			PxU32 generatedThisIter = threadBlockCountAndBroadcast(contact.candidateContact, countersS);
			//we need to sync because the function needs to complete for all warps before any warp attempts to reuse the counterS memory.
			__syncthreads();

			totalGenerated += generatedThisIter;		
			PxI32 remainingContacts = generatedThisIter;
			
			numTrianglesToSubdividePerThread = numTrianglesToSubdivideS; //Do this after threadBlockCountAndBroadcast because that method synchronizes the threads


			//First, iterate through existing patches to see if we have any interesting contacts for those patches...
			for (PxU32 p = 0; (p < numPatchesToKeep) && (remainingContacts > 0); ++p)
			{
				tryToAssignContactToExistingPatch(patchesS, p, contact, anyKeepS, clusterBias,
					countersS, shMinSepS, remainingContacts, tempPointsS);
			}

			createNewPatches(patchesS, contact, anyKeepS, clusterBias,
				countersS, remainingContacts, tempPointsS, shMinSepS, numPatchesToKeep);			
		}
	}

	assert(numPatchesToKeep <= MAX_MESH_MESH_PATCHES2);

	//Final step - output the contact patches and contact points...
	writePatchesAndContacts<32>(transform0, transform1, shape0, shape1, isFlipped, pairIndex,
		cmOutputs, materials, contactStream,
		patchStream, patchAndContactCounters, touchChangeFlags, patchChangeFlags, startContactPatches, startContactPoints, startContactForces,
		patchBytesLimit, contactBytesLimit, forceBytesLimit, numPatchesToKeep, patchesS);
}


extern "C" __global__
__launch_bounds__(1024, 1)
void evaluatePointDistancesSDFBatch(const PxgShape* PX_RESTRICT gpuShapes,
	const PxU32* shapeRefsSDF,	
	const PxVec4* localPositionsConcatenated,
	const PxU32* pointCountPerShape,
	const PxU32 maxPointCount,
	PxVec4* localGradientAndSDFValueConcatenated)
{
	const PxU32 batchId = blockIdx.y;

	__shared__ __align__(16) char sShape0[sizeof(PxgShape)];
	__shared__ __align__(16) char sMesh0[sizeof(PxgTriangleMesh)];
	__shared__ __align__(16) char texMem[sizeof(SparseSDFTexture)];
	PxgShape& shape0 = reinterpret_cast<PxgShape&>(*sShape0);
	PxgTriangleMesh& mesh0 = reinterpret_cast<PxgTriangleMesh&>(*sMesh0);
	SparseSDFTexture& sdfTexture = reinterpret_cast<SparseSDFTexture&>(*texMem);

	PxU32 shapeRef0 = shapeRefsSDF[batchId];
	const PxVec4* localPositions = &localPositionsConcatenated[batchId * maxPointCount];
	const PxU32 numSamplePositions = pointCountPerShape[batchId];
	PxVec4* localGradientAndSDFValue = &localGradientAndSDFValueConcatenated[batchId * maxPointCount];


	if (threadIdx.x < 32)
	{
		PxgShape_ReadWarp(shape0, gpuShapes + shapeRef0);
		__syncwarp();
		readTriangleMesh(shape0, mesh0);
	}

	if (threadIdx.x == 0)
	{
		/*printf("lower: %f, %f, %f \n", mesh0.meshLower.x, mesh0.meshLower.y, mesh0.meshLower.z);
		printf("dim: %i, %i, %i \n", mesh0.sdfDims.x, mesh0.sdfDims.y, mesh0.sdfDims.z);
		printf("spacing: %f \n", mesh0.spacing);*/

		sdfTexture.initialize(mesh0);
	}

	__syncthreads();
	

	PxU32 i = threadIdx.x + blockIdx.x * blockDim.x;
	
	if (i >= numSamplePositions)
		return;

	const PxVec3 localPos = localPositions[i].getXYZ();
	PxVec3 gradient = PxVolumeGrad(sdfTexture, localPos);
	gradient.normalizeSafe();
	PxReal signedDistance = PxSdfDistance(sdfTexture, localPos);
	localGradientAndSDFValue[i] = PxVec4(gradient, signedDistance);
}


extern "C" __global__
__launch_bounds__(1024, 1)
void triangleTriangleCollision(PxU32 numWorkItems,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistances,
	const PxgShape* PX_RESTRICT gpuShapes,
	PxsContactManagerOutput* PX_RESTRICT cmOutputs,
	PxU8* overlaps,
	const PxsMaterialData* PX_RESTRICT materials,
	PxU8* PX_RESTRICT contactStream,
	PxU8* PX_RESTRICT patchStream,
	PxgPatchAndContactCounters* PX_RESTRICT patchAndContactCounters,
	PxU32* touchChangeFlags,
	PxU32* patchChangeFlags,
	PxU8* startContactPatches,
	PxU8* startContactPoints,
	PxU8* startContactForces,
	PxU32 patchBytesLimit,
	PxU32 contactBytesLimit,
	PxU32 forceBytesLimit,
	const PxReal clusterBias
	)
{
	const PxU32 pairIdx = blockIdx.x;

	if (pairIdx >= numWorkItems)
		return;

	//Now read in the shapes and transforms etc.

	PxgContactManagerInput npWorkItem;
	PxgContactManagerInput_ReadWarp(npWorkItem, cmInputs, pairIdx);

	PxU32 transformCacheRef0 = npWorkItem.transformCacheRef0;
	PxU32 transformCacheRef1 = npWorkItem.transformCacheRef1;
	PxU32 shapeRef0 = npWorkItem.shapeRef0;
	PxU32 shapeRef1 = npWorkItem.shapeRef1;

	__shared__ __align__(16) char sShape0[sizeof(PxgShape)];
	__shared__ __align__(16) char sShape1[sizeof(PxgShape)];
	__shared__ __align__(16) char sTransform0[sizeof(PxsCachedTransform)];
	__shared__ __align__(16) char sTransform1[sizeof(PxsCachedTransform)];
	__shared__ __align__(16) char sMesh0[sizeof(PxgTriangleMesh)];
	__shared__ __align__(16) char sMesh1[sizeof(PxgTriangleMesh)];
	PxgShape& shape0 = reinterpret_cast<PxgShape&>(*sShape0);
	PxgShape& shape1 = reinterpret_cast<PxgShape&>(*sShape1);
	PxsCachedTransform& transform0 = reinterpret_cast<PxsCachedTransform&>(*sTransform0);
	PxsCachedTransform& transform1 = reinterpret_cast<PxsCachedTransform&>(*sTransform1);
	PxgTriangleMesh& mesh0 = reinterpret_cast<PxgTriangleMesh&>(*sMesh0);
	PxgTriangleMesh& mesh1 = reinterpret_cast<PxgTriangleMesh&>(*sMesh1);


	if (threadIdx.x < 32)
	{
		PxgShape_ReadWarp(shape0, gpuShapes + shapeRef0);
		PxgShape_ReadWarp(shape1, gpuShapes + shapeRef1);
		PxsCachedTransform_ReadWarp(transform0, transformCache + transformCacheRef0);
		PxsCachedTransform_ReadWarp(transform1, transformCache + transformCacheRef1);
		__syncwarp();
		readTriangleMesh(shape0, mesh0);
		readTriangleMesh(shape1, mesh1);
	}

	__syncthreads();

	const PxReal contactDistance = contactDistances[transformCacheRef0] + contactDistances[transformCacheRef1];

	bool doFirst = mesh0.mTexObject && (mesh1.mTexObject == NULL 
		|| ((!mesh0.sdfDims.w) && (mesh1.sdfDims.w)) 
		||((mesh1.sdfDims.w == mesh0.sdfDims.w) && mesh1.numVerts < mesh0.numVerts));


	if (doFirst)
	{
		doTriangleTriangleCollision(transform1, transform0, mesh1, mesh0, shape1, shape0, contactDistance, true, pairIdx,
			cmOutputs, materials, contactStream, patchStream, patchAndContactCounters, 
			touchChangeFlags, patchChangeFlags, startContactPatches, startContactPoints, startContactForces, 
			patchBytesLimit, contactBytesLimit, forceBytesLimit, clusterBias, overlaps[pairIdx]);
	}
	else
	{
		doTriangleTriangleCollision(transform0, transform1, mesh0, mesh1, shape0, shape1, contactDistance, false, pairIdx,
			cmOutputs, materials, contactStream, patchStream, patchAndContactCounters,
			touchChangeFlags, patchChangeFlags, startContactPatches, startContactPoints, startContactForces,
			patchBytesLimit, contactBytesLimit, forceBytesLimit, clusterBias, overlaps[pairIdx]);
	}
}


#include "schlockShared.h"

using namespace schlock;
#include "gjk.cuh"


extern "C" __global__
__launch_bounds__(1024, 1)
void triangleTriangleOverlaps(PxU32 numWorkItems,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistances,
	const PxgShape* PX_RESTRICT gpuShapes,
	PxsContactManagerOutput* PX_RESTRICT cmOutputs,
	PxU8* overlaps
)
{
	const PxU32 warpsPerBlock = NP_TRIMESH_WARPS_PER_BLOCK;

	const PxU32 threadIndexInWarp = threadIdx.x;
	const PxU32 warpIndex = threadIdx.y;

	__shared__ __align__(16) char sSharedGjkOutput[sizeof(schlock::GjkOutput) * warpsPerBlock];
	__shared__ __align__(16) char sSharedGjkCachedData[sizeof(schlock::GjkCachedData) * warpsPerBlock];
	__shared__ __align__(16) char sSharedPoints[sizeof(PxVec3) * warpsPerBlock * 16];
	schlock::GjkOutput* sharedGjkOutput = reinterpret_cast<schlock::GjkOutput*>(sSharedGjkOutput);
	schlock::GjkCachedData* sharedGjkCachedData = reinterpret_cast<schlock::GjkCachedData*>(sSharedGjkCachedData);
	//__shared__ __align__(16) PxVec3 sharedPoints[warpsPerBlock][16];
	PxVec3* sharedPoints = reinterpret_cast<PxVec3*>(sSharedPoints);

	schlock::GjkOutput& s_WarpGjkOutput = sharedGjkOutput[warpIndex];
	schlock::GjkCachedData& s_WarpGjkCachedData = sharedGjkCachedData[warpIndex];
	PxVec3* s_WarpPoints = &sharedPoints[warpIndex*16];

	const PxU32 globalWarpIndex = warpIndex + blockIdx.x * blockDim.y;
	
	if (globalWarpIndex >= numWorkItems)
		return;

	//Now read in the shapes and transforms etc.
	PxgContactManagerInput npWorkItem;
	PxgContactManagerInput_ReadWarp(npWorkItem, cmInputs, globalWarpIndex);

	PxU32 transformCacheRef0 = npWorkItem.transformCacheRef0;
	PxU32 transformCacheRef1 = npWorkItem.transformCacheRef1;
	PxU32 shapeRef0 = npWorkItem.shapeRef0;
	PxU32 shapeRef1 = npWorkItem.shapeRef1;


	PxgShape shape0;
	PxgShape shape1;
	PxsCachedTransform transform0;
	PxsCachedTransform transform1;
	PxgTriangleMesh mesh0;
	PxgTriangleMesh mesh1;

	PxgShape_ReadWarp(shape0, gpuShapes + shapeRef0);
	PxgShape_ReadWarp(shape1, gpuShapes + shapeRef1);
	PxsCachedTransform_ReadWarp(transform0, transformCache + transformCacheRef0);
	PxsCachedTransform_ReadWarp(transform1, transformCache + transformCacheRef1);
	__syncwarp();
	readTriangleMesh(shape0, mesh0);
	readTriangleMesh(shape1, mesh1);
	

	const PxReal contactDistance = contactDistances[transformCacheRef0] + contactDistances[transformCacheRef1];

	const PxTransform aToB = transform1.transform.transformInv(transform0.transform);

	PxVec3 min0 = mesh0.bv32PackedNodes[0].mMin[threadIndexInWarp].getXYZ();
	PxVec3 max0 = mesh0.bv32PackedNodes[0].mMax[threadIndexInWarp].getXYZ();
	PxU32 mask0 = (1 << mesh0.bv32PackedNodes[0].mNbNodes) - 1;

	PxVec3 min1 = mesh1.bv32PackedNodes[0].mMin[threadIndexInWarp].getXYZ();
	PxVec3 max1 = mesh1.bv32PackedNodes[0].mMax[threadIndexInWarp].getXYZ();
	PxU32 mask1 = (1 << mesh1.bv32PackedNodes[0].mNbNodes) - 1;

	minIndex(min0.x, mask0, min0.x);
	minIndex(min0.y, mask0, min0.y);
	minIndex(min0.z, mask0, min0.z);
	maxIndex(max0.x, mask0, max0.x);
	maxIndex(max0.y, mask0, max0.y);
	maxIndex(max0.z, mask0, max0.z);

	minIndex(min1.x, mask1, min1.x);
	minIndex(min1.y, mask1, min1.y);
	minIndex(min1.z, mask1, min1.z);
	maxIndex(max1.x, mask1, max1.x);
	maxIndex(max1.y, mask1, max1.y);
	maxIndex(max1.z, mask1, max1.z);

	if (threadIndexInWarp == 0)
		s_WarpGjkCachedData.size = 0;

	if (threadIndexInWarp < 8)
	{
		PxVec3 pos0;
		pos0.x = (threadIndexInWarp & 1) ? min0.x : max0.x;
		pos0.y = (threadIndexInWarp & 2) ? min0.y : max0.y;
		pos0.z = (threadIndexInWarp & 4) ? min0.z : max0.z;

		s_WarpPoints[threadIndexInWarp] = pos0;

		PxVec3 pos1;
		pos1.x = (threadIndexInWarp & 1) ? min1.x : max1.x;
		pos1.y = (threadIndexInWarp & 2) ? min1.y : max1.y;
		pos1.z = (threadIndexInWarp & 4) ? min1.z : max1.z;

		s_WarpPoints[threadIndexInWarp + 8] = pos1;
	}

	//Now scale them...

	__syncwarp();

	if (threadIndexInWarp < 16)
	{
		const PxgShape& shape = threadIndexInWarp < 8 ? shape0 : shape1;
	
		PxVec3 v = vertex2Shape(s_WarpPoints[threadIndexInWarp], shape.scale.scale, shape.scale.rotation);

		if (threadIndexInWarp < 8)
			v = aToB.transform(v);

		s_WarpPoints[threadIndexInWarp] = v;

	}

	__syncwarp();


	const PxReal convergenceRatio = 1 - 0.000225f;

	const PxReal sqDist = aToB.p.dot(aToB.p);
	PxVec3 initialDir = sqDist > 0.f ? aToB.p.getNormalized() : PxVec3(0.f, 1.f, 0.f);
	
	GjkResult::Enum result = squawk::gjk(
		s_WarpPoints, 8,
		&s_WarpPoints[8], 8,
		initialDir,
		contactDistance,
		convergenceRatio,
		s_WarpGjkOutput, s_WarpGjkCachedData);

	if (threadIndexInWarp == 0)
	{
		overlaps[globalWarpIndex] = (result != GjkResult::eDISTANT);
	}
}

#pragma nv_diagnostic pop
