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

#include <cuda.h>
#include <cuda_runtime.h>
#include <assert.h>

//#include "foundation/PxVec3.h"
//#include "foundation/PxTransform.h"
#include "PxgContactManager.h"
#include "PxsContactManagerState.h"
#include "PxContact.h"
#include "PxgNpKernelIndices.h"
#include "PxgCommonDefines.h"
#include "reduction.cuh"
#include "stdio.h"
#include "PxNodeIndex.h"

using namespace physx;

extern "C" __host__ void initNarrowphaseKernels17() {}

extern "C" __global__ void compressContactStage1(
	PxsContactManagerOutput*	outputData,					//input
	PxU32						numTotalPairs,				//input
	PxU32*						gBlockNumPairs				//output

	)
{
	const PxU32 warpPerBlock = PxgNarrowPhaseBlockDims::COMPRESS_CONTACT / WARP_SIZE;

	__shared__ PxU32 sWarpAccum[warpPerBlock];
	__shared__ PxU32 sAccum;
	
	const PxU32 block_size = PxgNarrowPhaseGridDims::COMPRESS_CONTACT;

	const PxU32 totalBlockRequired = (numTotalPairs + (blockDim.x - 1)) / blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (block_size - 1)) / block_size;

	const PxU32 idx = threadIdx.x;

	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE - 1);
	const PxU32 warpIndex = threadIdx.x / WARP_SIZE;

	if (idx == (WARP_SIZE - 1))
	{
		sAccum = 0;
	}

	__syncthreads();

	for (PxU32 i = 0; i < numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		PxU32 nbContacts = 0;

		if (workIndex < numTotalPairs)
		{
			nbContacts = outputData[workIndex].nbContacts;
		}

		//we don't care about how many contacts in the header. There either have contacts or doesn't
		//has cotacts
		const PxU32 contactAccum = __popc(__ballot_sync(FULL_MASK, nbContacts));

		if (threadIndexInWarp == (WARP_SIZE - 1))
		{
			sWarpAccum[warpIndex] = contactAccum;
			//printf("blockIdx %i warpIndex %i contactAccum %i\n", blockIdx.x, warpIndex, contactAccum);
		}

		__syncthreads();

		unsigned mask_idx = __ballot_sync(FULL_MASK, threadIndexInWarp < warpPerBlock);
		if (idx < warpPerBlock)
		{
			const PxU32 value = sWarpAccum[idx];

			const PxU32 res = warpReduction<AddOpPxU32, PxU32>(mask_idx, value);

			if (idx == (warpPerBlock - 1))
			{
				sAccum += res;
			}
		}

		__syncthreads();

	}

	if (idx == (warpPerBlock - 1))
	{
		gBlockNumPairs[blockIdx.x] = sAccum;

		//printf("blockIdx.x %i numPairs %i \n", blockIdx.x, sAccum);
	}
}

//PxgNarrowPhaseGridDims::COMPRESS_CONTACT is 32, which means we can use one warp to do the run sum for
//block
extern "C" __global__ void compressContactStage2(
	const PxgContactManagerInput*	inputData,						//input
	const PxsContactManagerOutput*	outputData,						//input
	const PxU32						numTotalPairs,					//input
	PxNodeIndex*					shapeToRigidRemapTable,			//input
	PxActor**						transformCacheIdToActorTable,	//input
	PxU32*							gBlockNumPairs,					//input
	PxU8*							cpuCompressedPatchesBase,		//input
	PxU8*							cpuCompressedContactsBase,		//input
	PxReal*							cpuForceBufferBase,				//input
	PxU8*							gpuCompressedPatchesBase,		//input
	PxU8*							gpuCompressedContactsBase,		//input
	PxReal*							gpuForceBufferBase,				//input
	PxU8*							gpuFrictionPatchesBase,			//input
	PxU32*							numPairs,						//input
	PxGpuContactPair*				outputPatchs,					//output
	const PxU32						maxOutputPatches				//input
	
)
{
	const PxU32 warpPerBlock = PxgNarrowPhaseBlockDims::COMPRESS_CONTACT / WARP_SIZE;
	const PxU32 block_size = PxgNarrowPhaseGridDims::COMPRESS_CONTACT;

	
	__shared__ PxU32 sWarpAccum[warpPerBlock];
	__shared__ PxU32 sBlockHistogram[block_size];
	__shared__ PxU32 sAccum;

	const PxU32 idx = threadIdx.x;

	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE - 1);
	const PxU32 warpIndex = threadIdx.x / WARP_SIZE;

	if (idx == (WARP_SIZE - 1))
	{
		sAccum = 0;
	}

	//accumulate num pairs per block and compute exclusive run sum
	unsigned mask_idx = __ballot_sync(FULL_MASK, threadIndexInWarp < block_size);
	if (warpIndex == 0 && threadIndexInWarp < block_size)
	{
		const PxU32 blockNumPairs = gBlockNumPairs[threadIndexInWarp];

		const PxU32 result = warpScan<AddOpPxU32, PxU32>(mask_idx, blockNumPairs);
		//store exclusive run sum
		sBlockHistogram[threadIndexInWarp] = result - blockNumPairs;

		//printf("tInd %i blockNumPairs %i result %i\n", threadIndexInWarp, blockNumPairs, result);

		if (threadIndexInWarp == block_size - 1)
		{
			(*numPairs) = result;
			//printf("blockIdx %i blockNumPairs %i numPairs %i\n", blockIdx.x, blockNumPairs, result);
		}
		
	}


	__syncthreads();

	const PxU32 totalBlockRequired = (numTotalPairs + (blockDim.x - 1)) / blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (block_size - 1)) / block_size;

	const PxU32 blockStartIndex = sBlockHistogram[blockIdx.x];

	for (PxU32 i = 0; i < numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		PxU32 nbContacts = 0;
		

		if (workIndex < numTotalPairs)
		{
			nbContacts = outputData[workIndex].nbContacts;
		}

		const PxU32 threadMask = (1 << threadIndexInWarp) - 1;

		//we don't care about how many contacts in the header. There either have contacts or doesn't
		//has contacts

		const PxU32 contactMask = __ballot_sync(FULL_MASK, nbContacts);
		const PxU32 contactAccum = __popc(contactMask);
		const PxU32 offset = __popc(contactMask & threadMask);
		if (threadIndexInWarp == (WARP_SIZE - 1))
		{
			sWarpAccum[warpIndex] = contactAccum;
		}

		const PxU32 prevAccum = sAccum;

		__syncthreads();

		unsigned mask_idx = __ballot_sync(FULL_MASK, threadIndexInWarp < warpPerBlock);
		if (idx < warpPerBlock)
		{
			const PxU32 value = sWarpAccum[idx];

			const PxU32 res = warpScan<AddOpPxU32, PxU32>(mask_idx, value);

			sWarpAccum[idx] = res - value;

			if (idx == warpPerBlock - 1)
			{
				sAccum += res;
			}
		}

		__syncthreads();

		//write to output
		if (nbContacts > 0)
		{
			const PxgContactManagerInput& cmInput = inputData[workIndex];
			const PxsContactManagerOutput& cmOutput = outputData[workIndex];
			const PxU32 index = offset + prevAccum + sWarpAccum[warpIndex] + blockStartIndex;
			
			if (index < maxOutputPatches)
			{
				PxU32 patchStartIndex = cmOutput.contactPatches - cpuCompressedPatchesBase;
				PxU32 contactIndex = cmOutput.contactPoints - cpuCompressedContactsBase;
				PxU32 forceIndex = 0xFFFFFFFF;
				if (cmOutput.contactForces)
					forceIndex = cmOutput.contactForces - cpuForceBufferBase;

				PxGpuContactPair & contact = outputPatchs[index];
				
				const PxU32 transformCacheRef0 = cmInput.transformCacheRef0;
				const PxU32 transformCacheRef1 = cmInput.transformCacheRef1;
				contact.transformCacheRef0 = transformCacheRef0;
				contact.transformCacheRef1 = transformCacheRef1;
				contact.nodeIndex0 = shapeToRigidRemapTable[transformCacheRef0];
				contact.nodeIndex1 = shapeToRigidRemapTable[transformCacheRef1];
				contact.contactForces = gpuForceBufferBase + forceIndex;
				contact.frictionPatches = gpuFrictionPatchesBase + patchStartIndex / sizeof(PxContactPatch) * sizeof(PxFrictionPatch);
				contact.contactPatches = gpuCompressedPatchesBase + patchStartIndex;
				contact.contactPoints = gpuCompressedContactsBase + contactIndex;
				contact.nbPatches = cmOutput.nbPatches;
				contact.nbContacts = cmOutput.nbContacts;
				contact.actor0 = transformCacheIdToActorTable[transformCacheRef0];
				contact.actor1 = transformCacheIdToActorTable[transformCacheRef1];
			}
		}
	}
}

// update frictionPatches CPU pointers from contactPatches CPU pointers
extern "C" __global__ void updateFrictionPatches(
	const PxU32								pairCount,				//input
	const PxU8* PX_RESTRICT					startContactPatches,	//input
	PxU8* PX_RESTRICT						startFrictionPatches,	//input (but we need it non-const)
	PxsContactManagerOutput* PX_RESTRICT	outputs					//input/output
)
{
	const PxU32 threadIndex = blockIdx.x * blockDim.x + threadIdx.x;

	if (threadIndex < pairCount)
	{
		PxsContactManagerOutput& output = outputs[threadIndex];
		output.frictionPatches = startFrictionPatches + (output.contactPatches - startContactPatches) * sizeof(PxFrictionPatch) / sizeof(PxContactPatch);
	}
}
