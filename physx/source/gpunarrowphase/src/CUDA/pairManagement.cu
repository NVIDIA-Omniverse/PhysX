// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <assert.h>
#include "SparseRemove.cuh"

#include "cudaNpCommon.h"
#include "PxgPersistentContactManifold.h"
#include "PxsContactManagerState.h"
#include "PxgContactManager.h"
#include "PxgNpKernelIndices.h"

using namespace physx;

namespace physx
{
	class PxsContactManager;
}

extern "C" __host__ void initNarrowphaseKernels7() {}

extern "C" __global__ void removeContactManagers_Stage1(const PxgPairManagementData* PX_RESTRICT pairData)
{
	initializeKeepDropBuffer(pairData->mTempAccumulator, pairData->mNbPairs, pairData->mNbToRemove);
}

extern "C" __global__ void removeContactManagers_Stage2(const PxgPairManagementData* PX_RESTRICT pairData)
{
	markKeepDropBuff(pairData->mRemoveIndices, pairData->mNbToRemove, pairData->mTempAccumulator, pairData->mNbPairs);
}

extern "C" __global__ void removeContactManagers_Stage3(const PxgPairManagementData* PX_RESTRICT pairData)
{
	processKeepDropBuff<PxgNarrowPhaseBlockDims::REMOVE_CONTACT_MANAGERS, PxgNarrowPhaseGridDims::REMOVE_CONTACT_MANAGERS>
		(pairData->mTempAccumulator, pairData->mNbPairs, pairData->mBlockSharedAccumulator);
}

extern "C" __global__ void removeContactManagers_Stage4(const PxgPairManagementData* PX_RESTRICT pairData)
{
	accumulateKeepDrop<PxgNarrowPhaseGridDims::REMOVE_CONTACT_MANAGERS>(pairData->mTempAccumulator, pairData->mNbPairs, pairData->mBlockSharedAccumulator);
}

extern "C" __global__ void removeContactManagers_Stage5(const PxgPairManagementData* PX_RESTRICT pairData, const bool copyManifold)
{
	const PxU32* PX_RESTRICT globalRunSumBuffer = pairData->mTempAccumulator;
	const PxU32 totalSize = pairData->mNbPairs;
	PxU32 nbToSwap = getNbSwapsRequired(globalRunSumBuffer, totalSize, pairData->mNbToRemove);
	PxgContactManagerInput* inputData = pairData->mContactManagerInputData;
	PxsContactManagerOutput* outputData = pairData->mContactManagerOutputData;
	PxgPersistentContactManifold* manifolds = reinterpret_cast<PxgPersistentContactManifold*>(pairData->mPersistentContactManagers);
	PxsContactManager** cms = pairData->mCpuContactManagerMapping;
	Sc::ShapeInteraction** sis = pairData->mShapeInteractions;
	PxReal* rest = pairData->mRestDistances;
	PxsTorsionalFrictionData* torsional = pairData->mTorsionalData;

	for(PxU32 i = (threadIdx.x + WARP_SIZE * threadIdx.y + blockIdx.x * blockDim.x * blockDim.y)/16;
		i < nbToSwap; i+= ((blockDim.x * blockDim.y * gridDim.x)/16))
	{
		PxU32 srcIndex, dstIndex;
		getSwapIndices(globalRunSumBuffer, totalSize, i, nbToSwap, dstIndex, srcIndex);

		if((threadIdx.x&15) == 0)
		{
			inputData[dstIndex] = inputData[srcIndex];
			outputData[dstIndex] = outputData[srcIndex];
			cms[dstIndex] = cms[srcIndex];
			sis[dstIndex] = sis[srcIndex];
			rest[dstIndex] = rest[srcIndex];
			torsional[dstIndex] = torsional[srcIndex];
		}

		if (copyManifold)
		{
			float4* dst = (float4*)&manifolds[dstIndex];
			float4* src = (float4*)&manifolds[srcIndex];

			const PxU32 nbFloat4 = sizeof(PxgPersistentContactManifold)/sizeof(float4);

			for(PxU32 idx = threadIdx.x&15; idx < nbFloat4; idx += 16)			
				dst[idx] = src[idx];
		}
	}
}

extern "C" __global__ void removeContactManagers_Stage5_CvxTri(const PxgPairManagementData* PX_RESTRICT pairData, const bool copyManifold)
{
	const PxU32* PX_RESTRICT globalRunSumBuffer = pairData->mTempAccumulator;
	const PxU32 totalSize = pairData->mNbPairs;
	PxU32 nbToSwap = getNbSwapsRequired(globalRunSumBuffer, totalSize, pairData->mNbToRemove);
	PxgContactManagerInput* inputData = pairData->mContactManagerInputData;
	PxsContactManagerOutput* outputData = pairData->mContactManagerOutputData;
	PxgPersistentContactMultiManifold* manifolds = reinterpret_cast<PxgPersistentContactMultiManifold*>( pairData->mPersistentContactManagers);
	PxsContactManager** cms = pairData->mCpuContactManagerMapping;
	Sc::ShapeInteraction** sis = pairData->mShapeInteractions;
	PxReal* rest = pairData->mRestDistances;
	PxsTorsionalFrictionData* torsional = pairData->mTorsionalData;

	for (PxU32 i = (threadIdx.x + WARP_SIZE * threadIdx.y + blockIdx.x * blockDim.x * blockDim.y)/WARP_SIZE;
		i < nbToSwap; i += ((blockDim.x * blockDim.y * gridDim.x))/WARP_SIZE)
	{
		PxU32 srcIndex, dstIndex;
		getSwapIndices(globalRunSumBuffer, totalSize, i, nbToSwap, dstIndex, srcIndex);

		if(threadIdx.x == 0)
		{
			inputData[dstIndex] = inputData[srcIndex];
			outputData[dstIndex] = outputData[srcIndex];
			cms[dstIndex] = cms[srcIndex];
			sis[dstIndex] = sis[srcIndex];
			rest[dstIndex] = rest[srcIndex];
			torsional[dstIndex] = torsional[srcIndex];
		}

		if (copyManifold)
		{
			float4* dst = (float4*)&manifolds[dstIndex];
			float4* src = (float4*)&manifolds[srcIndex];

			for (PxU32 i = threadIdx.x; i < sizeof(PxgPersistentContactMultiManifold) / sizeof(float4); i += WARP_SIZE)
			{
				dst[i] = src[i];
			}
		}
	}
}

extern "C" __global__ void initializeManifolds(float4* destination, const float4* source, PxU32 dataSize, PxU32 nbTimesToReplicate)
{
	const PxU32 MaxStructureSize = 4096;

	__shared__ float sourceData[MaxStructureSize / sizeof(float)];

	//Number of threads required per element (rounded up to a warp size)
	const PxU32 nbThreadsPerElement = dataSize / sizeof(float4);
	//Number to process per block
	const PxU32 nbPerBlock = blockDim.x / nbThreadsPerElement;
	const PxU32 MaxThreadIdx = nbPerBlock*nbThreadsPerElement;

	const PxU32 nbBlocksRequired = (nbTimesToReplicate + nbPerBlock - 1) / nbPerBlock;

	if (threadIdx.x < nbThreadsPerElement)
	{
		const float4 value = source[threadIdx.x];
		sourceData[threadIdx.x] = value.x;
		sourceData[threadIdx.x + nbThreadsPerElement] = value.y;
		sourceData[threadIdx.x + 2*nbThreadsPerElement] = value.z;
		sourceData[threadIdx.x + 3*nbThreadsPerElement] = value.w;
	}

	const PxU32 threadReadIdx = threadIdx.x%nbThreadsPerElement;

	__syncthreads();

	for (PxU32 a = blockIdx.x; a < nbBlocksRequired; a += gridDim.x)
	{
		PxU32 startBlockIdx = nbPerBlock*a;
		float4* startPtr = destination + startBlockIdx*nbThreadsPerElement;

		PxU32 idx = startBlockIdx + threadIdx.x / nbThreadsPerElement;

		if (threadIdx.x < MaxThreadIdx && idx < nbTimesToReplicate)
		{
			startPtr[threadIdx.x] = make_float4(sourceData[threadReadIdx], sourceData[threadReadIdx + nbThreadsPerElement],
				sourceData[threadReadIdx + 2 * nbThreadsPerElement], sourceData[threadReadIdx + 3 * nbThreadsPerElement]);
		}
	}
}
