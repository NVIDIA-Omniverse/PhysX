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

#include "PxsTransformCache.h"
#include "PxsContactManagerState.h"
#include "foundation/PxBounds3.h"
#include "PxgContactManager.h"
#include "PxgSoftBody.h"
#include "geometry/PxGeometry.h"
#include "PxsMaterialCore.h"
#include "cudaNpCommon.h"

#include "PxgCommonDefines.h"
#include "heightfieldUtil.cuh"
#include "utils.cuh"
#include "sphereTriangle.cuh"
#include "deformableCollision.cuh"
#include "assert.h"
#include "reduction.cuh"
#include <stdio.h>
#include "PxgSoftBodyCoreKernelIndices.h"
#include "PxgNpKernelIndices.h"

using namespace physx;

extern "C" __host__ void initNarrowphaseKernels16() {}

struct sbHeightfieldScratch
{

	const float4 * PX_RESTRICT tetmeshVerts;
	const uint4 * PX_RESTRICT tetmeshTetIndices;
	const PxU8*	PX_RESTRICT tetmeshTetSurfaceHint;
	PxU32 numTets;

	PxTransform heightfieldTransform;
	PxgShape heightfieldShape;

}; PX_COMPILE_TIME_ASSERT(sizeof(sbHeightfieldScratch) <= 1024);


//__device__ static inline PxU32 sbHeightfieldComputePairs(
//	const PxU32 minColumn,
//	const PxU32 maxColumn,
//	const PxU32 minRow,
//	const PxU32 maxRow,
//	const PxU32 nbRows,
//	const PxU32 nbCols,
//	PxHeightFieldSample* samples,
//	const PxU32 miny,
//	const PxU32 maxy
//)
//{
//
//	PxU32 nbPairs = 0;
//
//	const PxU32 columnSpan = maxColumn - minColumn;
//
//	//we have two materials corresponding to one vertexIndex, so each thread will deal with one of the materials
//	const PxU32 totalNumProcessed = (maxRow - minRow) * columnSpan * 2;
//
//	for (PxU32 i = 0; i < totalNumProcessed; ++i)
//	{
//
//		const PxU32 index = i / 2;
//		const PxU32 vertexIndex = (minRow + index / columnSpan) * nbCols + (minColumn + index % columnSpan);
//		assert(isValidVertex(vertexIndex, nbRows, nbCols));
//		PxReal h0 = getHeight(vertexIndex, samples);
//		PxReal h1 = getHeight(vertexIndex + 1, samples);
//		PxReal h2 = getHeight(vertexIndex + nbCols, samples);
//		PxReal h3 = getHeight(vertexIndex + nbCols + 1, samples);
//		const bool con0 = maxy < h0 && maxy < h1 && maxy < h2 && maxy < h3;
//		const bool con1 = miny > h0 && miny > h1 && miny > h2 && miny > h3;
//
//		if (!(con0 || con1))
//		{
//			const PxHeightFieldSample& sample = getSample(vertexIndex, samples);
//
//			const bool isMaterial1 = (i & 1) ? 1 : 0;
//			PxU32 material = isMaterial1 ? sample.materialIndex1 : sample.materialIndex0;
//			if (material != PxHeightFieldMaterial::eHOLE)
//				nbPairs++;
//		}
//	}//end of totalNumProcessed
//
//	return nbPairs;
//}

//__device__ static inline void sbHeightfieldOutputPairs(
//	const PxU32 minColumn,
//	const PxU32 maxColumn,
//	const PxU32 minRow,
//	const PxU32 maxRow,
//	const PxU32 nbRows,
//	const PxU32 nbCols,
//	PxHeightFieldSample* samples,
//	const PxU32 miny,
//	const PxU32 maxy,
//
//	const PxU32 cmInd,
//	const PxU32 tetrahedronInd,
//	const PxU32 startOffset,
//
//	uint4* stackPtr						//output
//
//)
//{
//	PxU32 pairCount = 0;
//	const PxU32 columnSpan = maxColumn - minColumn;
//
//	//we have two materials corresponding to one vertexIndex, so each thread will deal with one of the materials
//	const PxU32 totalNumProcessed = (maxRow - minRow) * columnSpan * 2;
//
//	for (PxU32 i = 0; i < totalNumProcessed; ++i)
//	{
//
//		PxU32 triangleIdx = 0xFFffFFff;
//
//		const PxU32 index = i / 2;
//		const PxU32 vertexIndex = (minRow + index / columnSpan) * nbCols + (minColumn + index % columnSpan);
//		assert(isValidVertex(vertexIndex, nbRows, nbCols));
//		PxReal h0 = getHeight(vertexIndex, samples);
//		PxReal h1 = getHeight(vertexIndex + 1, samples);
//		PxReal h2 = getHeight(vertexIndex + nbCols, samples);
//		PxReal h3 = getHeight(vertexIndex + nbCols + 1, samples);
//		const bool con0 = maxy < h0 && maxy < h1 && maxy < h2 && maxy < h3;
//		const bool con1 = miny > h0 && miny > h1 && miny > h2 && miny > h3;
//
//		if (!(con0 || con1))
//		{
//			const PxHeightFieldSample& sample = getSample(vertexIndex, samples);
//
//			const bool isMaterial1 = (i & 1) ? 1 : 0;
//			PxU32 material = isMaterial1 ? sample.materialIndex1 : sample.materialIndex0;
//			if (material != PxHeightFieldMaterial::eHOLE)
//			{
//				triangleIdx = isMaterial1 ? ((vertexIndex << 1) + 1) : (vertexIndex << 1);
//
//				stackPtr[startOffset + pairCount] = make_uint4(cmInd, tetrahedronInd, triangleIdx, 0);
//
//				pairCount++;
//			}
//		}
//	}//end of totalNumProcessed
//}

__device__ static inline void sbHeightfieldMidphaseCore(
	sbHeightfieldScratch* scratch,
	const PxU32 cmIdx,
	const PxBounds3 tetBound,
	const PxU32 tetrahedronIdx,
	const PxU32 workIndex,
	const PxU8 hint,
	const PxU32 numTets,
	const PxU32 stackSize,
	uint4* PX_RESTRICT stackPtr,							//output
	PxU32* PX_RESTRICT midphasePairsNum					//output
)
{
	PxU32 nbPairs = 0;
	PxBounds3 localBound;
	PxU32 nbRows = 0;
	PxU32 nbCols = 0;
	PxHeightFieldSample* samples = NULL;
	PxU32 minRow = 0;
	PxU32 maxRow = 0;
	PxU32 minColumn = 0;
	PxU32 maxColumn = 0;

	if (hint && tetrahedronIdx < numTets)
	{
		PxTransform heightfieldTransform = scratch->heightfieldTransform;

		PxgShape heightfieldShape = scratch->heightfieldShape;
		const PxReal oneOverHeightScale = 1.f / heightfieldShape.scale.scale.y;
		const PxReal oneOverRowScale = 1.f / PxAbs(heightfieldShape.scale.scale.x);
		const PxReal oneOverlColScale = 1.f / PxAbs(heightfieldShape.scale.scale.z);

		//bound is in world space, we need to transform the bound to the local space of height field
		localBound = PxBounds3::transformFast(heightfieldTransform.getInverse(), tetBound);

		localBound.minimum.x *= oneOverRowScale;
		localBound.minimum.y *= oneOverHeightScale;
		localBound.minimum.z *= oneOverlColScale;

		localBound.maximum.x *= oneOverRowScale;
		localBound.maximum.y *= oneOverHeightScale;
		localBound.maximum.z *= oneOverlColScale;

		//row scale
		if (heightfieldShape.scale.scale.x < 0.f)
		{
			//swap min and max row scale
			const PxReal temp = localBound.minimum.x;
			localBound.minimum.x = localBound.maximum.x;
			localBound.maximum.x = temp;
		}

		//col scale
		if (heightfieldShape.scale.scale.z < 0.f)
		{
			PxReal swap = localBound.minimum.z;
			localBound.minimum.z = localBound.maximum.z;
			localBound.maximum.z = swap;
		}

		PxU32* heightfieldData = reinterpret_cast<PxU32*>(heightfieldShape.hullOrMeshPtr);
		nbRows = heightfieldData[0];
		nbCols = heightfieldData[1];
		samples = reinterpret_cast<PxHeightFieldSample*>(&heightfieldData[2]);


		if (!(localBound.minimum.x > nbRows - 1) || (localBound.minimum.z > nbCols - 1)
			|| (localBound.maximum.x < 0) || (localBound.maximum.z < 0))
		{
			minRow = getMinRow(localBound.minimum.x, nbRows);
			maxRow = getMaxRow(localBound.maximum.x, nbRows);
			minColumn = getMinColumn(localBound.minimum.z, nbCols);
			maxColumn = getMaxColumn(localBound.maximum.z, nbCols);

			if ((2 * (maxColumn - minColumn) * (maxRow - minRow)) > 0)
			{
				nbPairs = heightfieldComputePairs(minColumn, maxColumn, minRow, maxRow, nbRows, nbCols,
					samples, localBound.minimum.y, localBound.maximum.y);
			}
		}
	}

	

	PxU32 pairOffset = warpScan<AddOpPxU32, PxU32>(FULL_MASK, nbPairs) - nbPairs;

	PxU32 validCount = 0;
	if (threadIdx.x == (WARP_SIZE - 1))
	{
		validCount = pairOffset + nbPairs;
	}
	validCount = __shfl_sync(FULL_MASK, validCount, WARP_SIZE - 1);


	PxU32 startIndex = 0xffffffff;
	if (threadIdx.x == 0 && validCount > 0)
	{
		startIndex = atomicAdd(midphasePairsNum, validCount);
	}

	startIndex = __shfl_sync(FULL_MASK, startIndex, 0);

	if (startIndex != 0xffffffff)
	{
		heightfieldOutputPairs(minColumn, maxColumn, minRow, maxRow, nbRows, nbCols,
			samples, localBound.minimum.y, localBound.maximum.y, cmIdx, tetrahedronIdx, startIndex + pairOffset, stackSize, stackPtr);
	}
}

extern "C" __global__ void sb_heightfieldMidphaseGeneratePairsLaunch(
	const PxU32									numNPWorkItems,
	const PxReal								toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,

	PxgSoftBody* PX_RESTRICT					softbodies,
	const PxU32									stackSizeBytes,

	PxU8*										stackPtr,							//output
	PxU32*										midphasePairsNum					//output
)
{

	__shared__ PxU32 scratchMem[1024];

	sbHeightfieldScratch* s_warpScratch = reinterpret_cast<sbHeightfieldScratch*>(scratchMem);

	const PxU32 nbWarpsPerCm = 64; // 64 warp(2048 threads) to deal with one pair
	const PxU32 nbWarpsRequired = nbWarpsPerCm * numNPWorkItems;

	const PxU32 nbThreadsPerCm = 64 * WARP_SIZE;

	const PxU32 numWarpPerBlock = MIDPHASE_WARPS_PER_BLOCK;
	const PxU32 totalNumWarps = PxgSoftBodyKernelGridDim::SB_SBMIDPHASE * numWarpPerBlock;

	const PxU32 warpIndex = threadIdx.y;

	const PxU32 nbIterationsPerWarps = (nbWarpsRequired + totalNumWarps - 1) / totalNumWarps;

	/*if (warpIndex == 0 && threadIdx.x == 0 && blockIdx.x == 0)
	{
		printf("nbIterationsPerWarps %i nbWarpsRequired %i\n", nbIterationsPerWarps, nbWarpsRequired);
	}*/
	
	for (PxU32 i = 0; i < nbIterationsPerWarps; ++i)
	{
		const PxU32 workIndex = i + (warpIndex + numWarpPerBlock * blockIdx.x) * nbIterationsPerWarps;

		if (workIndex < nbWarpsRequired)
		{
			unsigned int cmIdx = workIndex / nbWarpsPerCm;

			PxgShape softbodyShape, heightfieldShape;
			PxU32 softbodyCacheRef, heightfieldCacheRef;
			LoadShapePairWarp<PxGeometryType::eTETRAHEDRONMESH, PxGeometryType::eHEIGHTFIELD>(cmInputs, cmIdx, gpuShapes,
				softbodyShape, softbodyCacheRef, heightfieldShape, heightfieldCacheRef);


			PxsCachedTransform heightfieldTransformCache;
			PxsCachedTransform_ReadWarp(heightfieldTransformCache, transformCache + heightfieldCacheRef);

			const PxgSoftBody& softbody = softbodies[softbodyShape.particleOrSoftbodyId];

			PxU8* meshGeomPtr;
			meshGeomPtr = reinterpret_cast<PxU8 *>(softbody.mTetMeshData);
		
			const PxU32 idx = threadIdx.x;

			if (idx == 0)
			{
				s_warpScratch->tetmeshVerts = softbody.mPosition_InvMass;
				s_warpScratch->tetmeshTetIndices = softbody.mTetIndices;
				s_warpScratch->tetmeshTetSurfaceHint = softbody.mTetMeshSurfaceHint;

				const uint4 nbVerts_nbPrimitives_maxDepth_nbBv32TreeNodes = *reinterpret_cast<const uint4 *>(meshGeomPtr);
				s_warpScratch->numTets = nbVerts_nbPrimitives_maxDepth_nbBv32TreeNodes.y;

				meshGeomPtr += sizeof(uint4) + sizeof(const Gu::BV32DataPacked)* nbVerts_nbPrimitives_maxDepth_nbBv32TreeNodes.w 
					+sizeof(const Gu::BV32DataDepthInfo) * nbVerts_nbPrimitives_maxDepth_nbBv32TreeNodes.z
					+ sizeof(PxU32) * nbVerts_nbPrimitives_maxDepth_nbBv32TreeNodes.w;
				
				/*const PxU8* surfaceHint = reinterpret_cast<const PxU8*>(meshGeomPtr);
				s_warpScratch->tetmeshTetSurfaceHint = surfaceHint;*/

				s_warpScratch->heightfieldTransform = heightfieldTransformCache.transform;
				s_warpScratch->heightfieldShape = heightfieldShape;
			}

			__syncwarp();

			const PxReal contactDist = contactDistance[softbodyCacheRef] + contactDistance[heightfieldCacheRef];

			const PxU32 nbTets = s_warpScratch->numTets;

			const PxU32 nbIter = (nbTets + nbThreadsPerCm - 1) / nbThreadsPerCm;

			const uint4* tetIndices = s_warpScratch->tetmeshTetIndices;
			const PxU8* surfaceHint = s_warpScratch->tetmeshTetSurfaceHint;

			for (PxU32 j = 0; j < nbIter; ++j)
			{
				const PxU32 wrappedThreadIndex = (workIndex * WARP_SIZE + threadIdx.x) % nbThreadsPerCm;

				const PxU32 tetrahedronIdx = wrappedThreadIndex + nbThreadsPerCm * j;

				PxBounds3 tetBound = PxBounds3::empty();
				PxU8 hint = 1;
				if (tetrahedronIdx < nbTets)
				{
					//avoid generate contacts if the tetrahedron isn't a surface tetrahedron
					hint = surfaceHint[tetrahedronIdx];

					if (hint)
					{
						const uint4 tetIdx = tetIndices[tetrahedronIdx];
						const PxVec3 worldV0 = PxLoad3(s_warpScratch->tetmeshVerts[tetIdx.x]);
						const PxVec3 worldV1 = PxLoad3(s_warpScratch->tetmeshVerts[tetIdx.y]);
						const PxVec3 worldV2 = PxLoad3(s_warpScratch->tetmeshVerts[tetIdx.z]);
						const PxVec3 worldV3 = PxLoad3(s_warpScratch->tetmeshVerts[tetIdx.w]);

						tetBound = tetBoundingBox(worldV0, worldV1, worldV2, worldV3);

						tetBound.fattenFast(contactDist);
					}
				}

				uint4* tStackPtr = reinterpret_cast<uint4*>(stackPtr);
				const PxU32 stackSize = stackSizeBytes / sizeof(uint4);

				sbHeightfieldMidphaseCore(
					s_warpScratch,
					cmIdx,
					tetBound,
					tetrahedronIdx,
					workIndex,
					hint,
					nbTets,
					stackSize,
					tStackPtr,
					midphasePairsNum
				);
			}
		}
	}
}
