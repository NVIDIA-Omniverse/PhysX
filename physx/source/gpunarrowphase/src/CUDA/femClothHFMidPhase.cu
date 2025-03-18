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
#include "foundation/PxVec3.h"

#include "geometry/PxGeometry.h"
#include "geometry/PxHeightFieldSample.h"

#include "PxgContactManager.h"
#include "PxgConvexConvexShape.h"
#include "PxgFEMCloth.h"
#include "PxgNpKernelIndices.h"
#include "PxgSoftBodyCoreKernelIndices.h"

#include "PxsTransformCache.h"

#include <vector_types.h>

#include "PxgCommonDefines.h"
#include "dataReadWriteHelper.cuh"
#include "heightfieldUtil.cuh"
#include "utils.cuh"
#include "reduction.cuh"
#include "deformableCollision.cuh"

#include "assert.h"

using namespace physx;

extern "C" __host__ void initNarrowphaseKernels20() {}

struct clothHeightfieldScratch
{

	/*const float4 * PX_RESTRICT trimeshVerts;
	const uint4 * PX_RESTRICT trimeshTriVertIndices;
	
	PxU32 numTriangles;*/

	PxTransform heightfieldTransform;
	PxgShape heightfieldShape;

}; PX_COMPILE_TIME_ASSERT(sizeof(clothHeightfieldScratch) <= 1024);



__device__ static inline void clothHeightfieldMidphaseCore(
	clothHeightfieldScratch* scratch,
	const PxU32 cmIdx,
	const PxBounds3 triangleBound,
	const PxU32 triangleIdx,
	const PxU32 workIndex,
	const PxU32 numTriangles,
	PxU8* PX_RESTRICT stackPtr,							//output
	PxU32* PX_RESTRICT midphasePairsNum,				//output
	const PxU32 stackSize
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

	if (triangleIdx < numTriangles)
	{
		PxTransform heightfieldTransform = scratch->heightfieldTransform;

		PxgShape heightfieldShape = scratch->heightfieldShape;
		const PxReal oneOverHeightScale = 1.f / heightfieldShape.scale.scale.y;
		const PxReal oneOverRowScale = 1.f / PxAbs(heightfieldShape.scale.scale.x);
		const PxReal oneOverlColScale = 1.f / PxAbs(heightfieldShape.scale.scale.z);

		//bound is in world space, we need to transform the bound to the local space of height field
		localBound = PxBounds3::transformFast(heightfieldTransform.getInverse(), triangleBound);

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

	uint4* tStackPtr = reinterpret_cast<uint4*>(stackPtr);

	if (nbPairs != 0)
	{
		heightfieldOutputPairs(minColumn, maxColumn, minRow, maxRow, nbRows, nbCols,
			samples, localBound.minimum.y, localBound.maximum.y, cmIdx, triangleIdx, startIndex + pairOffset, stackSize, tStackPtr);
	}
}

extern "C" __global__ void cloth_heightfieldMidphaseGeneratePairsLaunch(
	const PxU32 numNPWorkItems,
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,

	PxgFEMCloth* PX_RESTRICT clothes,

	PxU8* stackPtr,							//output
	PxU32* midphasePairsNum,				//output
	const PxU32 stackSizeBytes
)
{

	__shared__ PxU32 scratchMem[1024];

	clothHeightfieldScratch* s_warpScratch = reinterpret_cast<clothHeightfieldScratch*>(scratchMem);

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

			PxgShape clothShape, heightfieldShape;
			PxU32 clothCacheRef, heightfieldCacheRef;
			LoadShapePair<PxGeometryType::eTRIANGLEMESH, PxGeometryType::eHEIGHTFIELD>(cmInputs, cmIdx, gpuShapes,
				clothShape, clothCacheRef, heightfieldShape, heightfieldCacheRef);

			PxsCachedTransform heightfieldTransformCache;
			PxsCachedTransform_ReadWarp(heightfieldTransformCache, transformCache + heightfieldCacheRef);

			const PxgFEMCloth& cloth = clothes[clothShape.particleOrSoftbodyId];

			if (threadIdx.x == 0)
			{
				s_warpScratch->heightfieldTransform = heightfieldTransformCache.transform;
				s_warpScratch->heightfieldShape = heightfieldShape;
			}

			__syncwarp();

			const PxReal contactDist = contactDistance[clothCacheRef] + contactDistance[heightfieldCacheRef];

			const PxU32 nbTriangles = cloth.mNbTriangles;// s_warpScratch->numTriangles;

			const PxU32 nbIter = (nbTriangles + nbThreadsPerCm - 1) / nbThreadsPerCm;

			const uint4* triVertIndices = cloth.mTriangleVertexIndices;// s_warpScratch->trimeshTriVertIndices;

			const float4* positions = cloth.mPosition_InvMass;
			
			for (PxU32 j = 0; j < nbIter; ++j)
			{
				const PxU32 wrappedThreadIndex = (workIndex * WARP_SIZE + threadIdx.x) % nbThreadsPerCm;

				const PxU32 triangleIdx = wrappedThreadIndex + nbThreadsPerCm * j;

				PxBounds3 triangleBound = PxBounds3::empty();
				
				if (triangleIdx < nbTriangles)
				{
					const uint4 vertIdx = triVertIndices[triangleIdx];
					const PxVec3 worldV0 = PxLoad3(positions[vertIdx.x]);
					const PxVec3 worldV1 = PxLoad3(positions[vertIdx.y]);
					const PxVec3 worldV2 = PxLoad3(positions[vertIdx.z]);

					triangleBound = triBoundingBox(worldV0, worldV1, worldV2);

					triangleBound.fattenFast(contactDist);
				}

				clothHeightfieldMidphaseCore(
					s_warpScratch,
					cmIdx,
					triangleBound,
					triangleIdx,
					workIndex,
					nbTriangles,
					stackPtr,
					midphasePairsNum,
					stackSizeBytes/sizeof(uint4)
				);
			}
		}
	}
}


extern "C" __global__ void cloth_midphaseVertexHeightfieldLaunch(
	const PxU32 numNPWorkItems,
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,

	PxgFEMCloth* PX_RESTRICT clothes,

	PxU8* stackPtr,							//output
	PxU32* midphasePairsNum,				//output
	const PxU32 stackSizeBytes
)
{

	__shared__ PxU32 scratchMem[1024];

	clothHeightfieldScratch* s_warpScratch = reinterpret_cast<clothHeightfieldScratch*>(scratchMem);

	const PxU32 nbWarpsPerCm = 64; // 64 warp(2048 threads) to deal with one pair
	const PxU32 nbWarpsRequired = nbWarpsPerCm * numNPWorkItems;

	const PxU32 nbThreadsPerCm = 64 * WARP_SIZE;

	const PxU32 numWarpPerBlock = MIDPHASE_WARPS_PER_BLOCK;
	const PxU32 totalNumWarps = PxgSoftBodyKernelGridDim::SB_SBMIDPHASE * numWarpPerBlock;

	const PxU32 warpIndex = threadIdx.y;

	const PxU32 nbIterationsPerWarps = (nbWarpsRequired + totalNumWarps - 1) / totalNumWarps;

	for (PxU32 i = 0; i < nbIterationsPerWarps; ++i)
	{
		const PxU32 workIndex = i + (warpIndex + numWarpPerBlock * blockIdx.x) * nbIterationsPerWarps;

		if (workIndex < nbWarpsRequired)
		{

			unsigned int cmIdx = workIndex / nbWarpsPerCm;

			PxgShape clothShape, heightfieldShape;
			PxU32 clothCacheRef, heightfieldCacheRef;
			LoadShapePairWarp<PxGeometryType::eTRIANGLEMESH, PxGeometryType::eHEIGHTFIELD>(cmInputs, cmIdx, gpuShapes,
				clothShape, clothCacheRef, heightfieldShape, heightfieldCacheRef);
			
			PxsCachedTransform heightfieldTransformCache;
			PxsCachedTransform_ReadWarp(heightfieldTransformCache, transformCache + heightfieldCacheRef);

			const PxgFEMCloth& cloth = clothes[clothShape.particleOrSoftbodyId];

			if (threadIdx.x == 0)
			{
				s_warpScratch->heightfieldTransform = heightfieldTransformCache.transform;
				s_warpScratch->heightfieldShape = heightfieldShape;
			}

			__syncwarp();

			const PxReal contactDist = contactDistance[clothCacheRef] + contactDistance[heightfieldCacheRef];

			const float4* positions = cloth.mPosition_InvMass;

			const PxU32 nbVerts = cloth.mNbVerts;

			const PxU32 nbIter = (nbVerts + nbThreadsPerCm - 1) / nbThreadsPerCm;

			//const uint4* triVertIndices = s_warpScratch->trimeshTriVertIndices;

			const PxVec3 radiusV(contactDist);

			for (PxU32 j = 0; j < nbIter; ++j)
			{
				const PxU32 wrappedThreadIndex = (workIndex * WARP_SIZE + threadIdx.x) % nbThreadsPerCm;

				const PxU32 vertIndex = wrappedThreadIndex + nbThreadsPerCm * j;

				const float4 pos = positions[vertIndex];

				//particle pos need to be in triangle mesh vertex space
				const PxVec3 particlePos(pos.x, pos.y, pos.z);

				PxBounds3 triangleBound = PxBounds3::empty();

				triangleBound.minimum = particlePos - radiusV;
				triangleBound.maximum = particlePos + radiusV;

				clothHeightfieldMidphaseCore(
					s_warpScratch,
					cmIdx,
					triangleBound,
					vertIndex,
					workIndex,
					nbVerts,
					stackPtr,
					midphasePairsNum,
					stackSizeBytes/sizeof(uint4)
				);
			}
		}
	}
}