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
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"

#include "geometry/PxGeometry.h"
#include "geometry/PxHeightFieldSample.h"

#include "PxgContactManager.h"
#include "PxgNpKernelIndices.h"
#include "PxgPersistentContactManifold.h"
#include "PxgSimulationCoreDesc.h"

#include "PxsContactManagerState.h"
#include "PxsTransformCache.h"

#include "convexNpCommon.h"
#include "cudaNpCommon.h"

#include "PxgCommonDefines.h"
#include "dataReadWriteHelper.cuh"
#include "heightfieldUtil.cuh"
#include "manifold.cuh"
#include "midphaseAllocate.cuh"

#include <vector_types.h>

using namespace physx;

extern "C" __host__ void initNarrowphaseKernels0() {}

PX_ALIGN_PREFIX(16)
struct HeigtFieldDataScratch
{
	PxTransform convexToHeightfieldNoScale;
	PxU32 convexShape_materialIndex;
	PxU32 heightfieldShape_materialIndex;	
}PX_ALIGN_SUFFIX(16);


template<unsigned int WarpsPerBlock>
__device__ static inline void heightfieldMidphaseCore(
	PxU32 numContactManagers,
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,

	ConvexMeshPair* PX_RESTRICT cvxTrimeshPair,
	PxgPersistentContactMultiManifold* PX_RESTRICT multiManifolds,
	PxsContactManagerOutput * PX_RESTRICT cmOutputs,
	uint4* PX_RESTRICT stackBasePtr,
	PxU32* PX_RESTRICT nbPairsFound,
	PxU32* PX_RESTRICT midphasePairsNum,
	PxU32* PX_RESTRICT midphasePairsNumPadded,

	HeigtFieldDataScratch*	s_warpScratch,
	const PxU32 maxPairs
	)
{
	//thread index in warp
	const unsigned int idxInWarp = threadIdx.x;
	//wrap index
	const unsigned int warpIdx = threadIdx.y;

	unsigned int cmIdx = warpIdx + blockIdx.x * blockDim.y;

	//this is number of contact managers
	PxU32 nbPairsPerCM = 0;

	if (cmIdx < numContactManagers)
	{
		PxgContactManagerInput npWorkItem;
		PxgContactManagerInput_ReadWarp(npWorkItem, cmInputs, cmIdx);

		PxsCachedTransform transformCached, heightfieldTransformCached;
		PxsCachedTransform_ReadWarp(transformCached, transformCache + npWorkItem.transformCacheRef0);
		PxsCachedTransform_ReadWarp(heightfieldTransformCached, transformCache + npWorkItem.transformCacheRef1);

		const PxTransform shapeToHeightfieldNoScale = heightfieldTransformCached.transform.transformInv(transformCached.transform);

		//read convex/sphere shape
		PxgShape shape;
		PxgShape_ReadWarp(shape, gpuShapes + npWorkItem.shapeRef0);


		PxReal ratio, minMargin, breakingThresholdRatio;
		if (shape.type == PxGeometryType::eSPHERE)
		{
			minMargin = shape.scale.scale.x; //sphere radius
			ratio = 0.02f;
			breakingThresholdRatio = 0.05f;
		}
		else if (shape.type == PxGeometryType::eCAPSULE)
		{
			minMargin = shape.scale.scale.y; //capsule radius
			ratio = 0.02;
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

		/*PxU8* hullPtr = reinterpret_cast<PxU8*>(convexShape.hullOrMeshPtr);

		const float4 extents4_f = *reinterpret_cast<float4*>(hullPtr + sizeof(float4) * 2);
		const PxReal minMargin = calculatePCMConvexMargin(extents4_f, convexShape.scale.scale, toleranceLength);*/

		bool lostContacts = false;

		const bool invalidate = invalidateManifold(shapeToHeightfieldNoScale, multiManifolds[cmIdx], minMargin, ratio);

		if (!invalidate)
		{
			const PxReal projectBreakingThreshold = minMargin * breakingThresholdRatio;

			lostContacts = refreshManifolds(
				shapeToHeightfieldNoScale,
				projectBreakingThreshold,
				multiManifolds + cmIdx
				);
		}

		bool fullContactGen = invalidate || lostContacts;

		//read heightfield shape
		PxgShape heightFieldShape;
		PxgShape_ReadWarp(heightFieldShape, gpuShapes + npWorkItem.shapeRef1);

		if (idxInWarp == 0)
		{
			s_warpScratch->convexToHeightfieldNoScale = shapeToHeightfieldNoScale;
			s_warpScratch->convexShape_materialIndex = shape.materialIndex;
			s_warpScratch->heightfieldShape_materialIndex = heightFieldShape.materialIndex;
		}

		__syncwarp();

		//if invalidate is true, generate full contacts 
		if (fullContactGen)
		{
			
			PxU32* heightfieldData = reinterpret_cast<PxU32*>(heightFieldShape.hullOrMeshPtr);
			const PxU32 nbRows = heightfieldData[0];
			const PxU32 nbCols = heightfieldData[1];
			PxHeightFieldSample* samples = reinterpret_cast<PxHeightFieldSample*>(&heightfieldData[2]);


			const PxReal oneOverHeightScale = 1.f / heightFieldShape.scale.scale.y;
			const PxReal oneOverRowScale = 1.f / PxAbs(heightFieldShape.scale.scale.x);
			const PxReal oneOverlColScale = 1.f / PxAbs(heightFieldShape.scale.scale.z);

			PxBounds3 worldBound;
			PxBounds3_ReadWarp(worldBound, bounds + npWorkItem.transformCacheRef0);
			
			//bound is in world space, we need to transform the bound to the local space of height field
			PxBounds3 localBound = PxBounds3::transformFast(heightfieldTransformCached.transform.getInverse(), worldBound);
			const PxReal contactDist = contactDistance[npWorkItem.transformCacheRef0] + contactDistance[npWorkItem.transformCacheRef1];
			localBound.fattenFast(contactDist);

			localBound.minimum.x *= oneOverRowScale;
			localBound.minimum.y *= oneOverHeightScale;
			localBound.minimum.z *= oneOverlColScale;

			localBound.maximum.x *= oneOverRowScale;
			localBound.maximum.y *= oneOverHeightScale;
			localBound.maximum.z *= oneOverlColScale;

			//row scale
			if (heightFieldShape.scale.scale.x < 0.f)
			{
				//swap min and max row scale
				const PxReal temp = localBound.minimum.x;
				localBound.minimum.x = localBound.maximum.x;
				localBound.maximum.x = temp;
			}

			//col scale
			if (heightFieldShape.scale.scale.z < 0.f)
			{
				PxReal swap = localBound.minimum.z;
				localBound.minimum.z = localBound.maximum.z;
				localBound.maximum.z = swap;
			}

			bool boundsDontOverlap = false;

			// this tests if the complete shape is outside of the bounds of the HF in the XZ plane.
			if ((localBound.minimum.x > nbRows - 1) || (localBound.minimum.z > nbCols - 1)
				|| (localBound.maximum.x < 0) || (localBound.maximum.z < 0)) 
			{
				boundsDontOverlap = true;
			}

			if (!boundsDontOverlap)
			{
				PxU32 minRow = getMinRow(localBound.minimum.x, nbRows);
				PxU32 maxRow = getMaxRow(localBound.maximum.x, nbRows); 
				PxU32 minColumn = getMinColumn(localBound.minimum.z, nbCols);
				PxU32 maxColumn = getMaxColumn(localBound.maximum.z, nbCols);

				bool noTriangles = false;

				// AD: This test whether we have any triangles at all.
				// Given the clamping above I can only see this happening if we have a 
				// flat shape that lies exactly on one of the grid lines for sampling.

				// Also, the 2x looks unnecessary here if my basic math isn't failing me.
				// This is the same code as CPU and has been there since 2009, probably before,
				// so I'm not going to change it now.
				if ((2 * (maxColumn - minColumn) * (maxRow - minRow)) == 0)
				{
					noTriangles = true;
				}

				if (!noTriangles)
				{

					const PxReal miny = localBound.minimum.y;
					const PxReal maxy = localBound.maximum.y;

					const PxU32 columnSpan = maxColumn - minColumn;

					//we have two materials corresponding to one vertexIndex, so each thread will deal with one of the materials
					const PxU32 totalNumProcessed = (maxRow - minRow) * columnSpan * 2;
					for (PxU32 i = 0; i < totalNumProcessed; i += WARP_SIZE)
					{
						bool result = false;
						PxU32 triangleIndex = 0xFFffFFff;

						const PxU32 workIndex = idxInWarp + i;

						if (workIndex < totalNumProcessed)
						{
							const PxU32 index = workIndex / 2;
							const PxU32 vertexIndex = (minRow + index / columnSpan) * nbCols + (minColumn + index % columnSpan);
							assert(isValidVertex(vertexIndex, nbRows, nbCols));
							PxReal h0 = getHeight(vertexIndex, samples);
							PxReal h1 = getHeight(vertexIndex + 1, samples);
							PxReal h2 = getHeight(vertexIndex + nbCols, samples);
							PxReal h3 = getHeight(vertexIndex + nbCols + 1, samples);
							const bool con0 = maxy < h0 && maxy < h1 && maxy < h2 && maxy < h3;
							const bool con1 = miny > h0 && miny > h1 && miny > h2 && miny > h3;

							if (!(con0 || con1))
							{
								const PxHeightFieldSample& sample = getSample(vertexIndex, samples);

								const bool isMaterial1 = (workIndex & 1) ? 1 : 0;
								PxU32 material = isMaterial1 ? sample.materialIndex1 : sample.materialIndex0;
								if (material != PxHeightFieldMaterial::eHOLE)
								{
									triangleIndex = isMaterial1 ? ((vertexIndex << 1) + 1) : (vertexIndex << 1);
									result = true;
								}
							}
						}

						PxU32 resultWarp = __ballot_sync(FULL_MASK, result);
						PxU32 offset = warpScanExclusive(resultWarp, idxInWarp);
						PxU32 validCount = __popc(resultWarp);

						// Allocate only amount of memory, needed for single warp-wide write
						PxU32 prevNbPairs = 0xFFffFFff;
						if (idxInWarp == 0 && validCount > 0)
						{
							prevNbPairs = atomicAdd(nbPairsFound, validCount);
						}

						prevNbPairs = __shfl_sync(FULL_MASK, prevNbPairs, 0);

						if (result && (prevNbPairs + offset) < maxPairs)
						{
							stackBasePtr[prevNbPairs + offset] = make_uint4(cmIdx, triangleIndex, nbPairsPerCM + offset, npWorkItem.shapeRef1);
						}

						if ((validCount > 0) && ((validCount + prevNbPairs) >= maxPairs))
						{
							validCount = PxMax(maxPairs, prevNbPairs) - prevNbPairs;
						}

						assert(((validCount + prevNbPairs) <= maxPairs) || (validCount == 0));

						nbPairsPerCM += validCount;
					}
				} // noTriangles
			} // boundsDontOverlap
		}

		PxU32 prevIntermArraysOffset = 0xFFffFFff;
		PxU32 prevIntermArraysPaddedOffset = 0xFFffFFff;
		if (idxInWarp == 0 && nbPairsPerCM > 0)
		{
			prevIntermArraysOffset = atomicAdd(midphasePairsNum, nbPairsPerCM);
			prevIntermArraysPaddedOffset = atomicAdd(midphasePairsNumPadded, ((nbPairsPerCM + 3)&(~3)) * 2); // AD: we need 2x space for the radix sort.
		}

		prevIntermArraysOffset = __shfl_sync(FULL_MASK, prevIntermArraysOffset, 0);
		prevIntermArraysPaddedOffset = __shfl_sync(FULL_MASK, prevIntermArraysPaddedOffset, 0);

		ConvexMeshPair pairInfo;
		pairInfo.aToB = s_warpScratch->convexToHeightfieldNoScale;
		pairInfo.cmIndex = cmIdx;
		pairInfo.startIndex = prevIntermArraysOffset;
		pairInfo.count = fullContactGen ? nbPairsPerCM : CONVEX_TRIMESH_CACHED;
		pairInfo.roundedStartIndex = prevIntermArraysPaddedOffset;
		pairInfo.materialIndices = make_uint2(s_warpScratch->convexShape_materialIndex, s_warpScratch->heightfieldShape_materialIndex);

		ConvexMeshPair_WriteWarp(cvxTrimeshPair + cmIdx, pairInfo);

		assert(*midphasePairsNum <= maxPairs);
	}
}

extern "C" __global__ void convexHeightFieldMidphase( 
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
	const PxU32 stackSizeBytes
)
{
	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][WARP_SIZE * 2];

	const PxU32 maxPairs = calculateMaxPairs(stackSizeBytes, numContactManagers);

	heightfieldMidphaseCore<MIDPHASE_WARPS_PER_BLOCK>(

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
		reinterpret_cast<uint4*>(stackPtr),
		stackOffset,
		midphasePairsNum,
		midphasePairsNumPadded,

		(HeigtFieldDataScratch*)scratchMem[threadIdx.y],
		maxPairs
	);
}
