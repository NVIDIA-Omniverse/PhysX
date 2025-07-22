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

#include "PxgBroadPhaseDesc.h"
#include "PxgIntegerAABB.h"
#include "PxgSapBox1D.h"
#include "PxgBroadPhasePairReport.h"
#include "BpVolumeData.h"
#include "cutil_math.h"
#include "PxgBroadPhaseKernelIndices.h"
#include "reduction.cuh"
#include "PxgCommonDefines.h"
#include <assert.h>
#include <stdio.h>

#define	BLOCK_SIZE				32
#define	AXIS_X					0
#define	AXIS_Y					1
#define	AXIS_Z					2
#define	REGION_SIZE_PER_AXIS	4
#define	TOTAL_REGION_SIZE		64
#define	WARP_PERBLOCK_SIZE_16	16

using namespace physx;

extern "C" __host__ void initBroadphaseKernels0() {}

#define USE_ENV_IDS	1

// PT: kernels marked with "###ONESHOT" are the ones that run for "one shot" queries (a single call to the broadphase).
// Others are needed for incremental updates.

// PT: we use(d) this macro to detect overflows
#define CHECK64(x)	if(x>0x00000000ffffffff)	{ printf("FOUND OVERFLOW! %s = %lld\n", #x, x);	}

#if USE_ENV_IDS
static __device__ PX_FORCE_INLINE bool filtering(PxU32 groupId, PxU32 otherGroupId, PxU32 envId, PxU32 otherEnvId)
{
	if(0)
		printf("%d %d %d %d\n", groupId, otherGroupId, envId, otherEnvId);
	// PT: filtering uses two distinct IDs: group IDs and environment IDs.
	//
	// Group IDs are for the standard group-based filtering implemented by all broadphases. This is typically used by
	// compound actors, to filter out collisions between shapes of the same actors (needed because the BPs are shape-centric).
	// In that context each actor has its own ID, and therefore two entries do not collide if they have the same group ID.
	//
	// Environment IDs are for RL-specific scenarios, and only used in the GPU BP. Each environment has its own ID, and
	// entries belong to a specific environment. Two entries collide if they are located in the same environment, i.e. if
	// they do have the same environment ID. By design, entries marked with an invalid environment ID will collide with
	// everything else. This is for e.g. ground plane shapes, which should support all other shapes regardless of their
	// environment. The alternative would be to duplicate the ground plane in each environment, which would be a waste.
	//
	return	(groupId != otherGroupId									// PT: true if shapes are not part of the same actor
		&&	((envId == otherEnvId)										// PT: true is shapes belong to the same environment
		||	(envId==PX_INVALID_U32) || (otherEnvId==PX_INVALID_U32)));	// PT: true for shapes shared by all environments
}
#endif

static __device__ PX_FORCE_INLINE PxU32 encodeFloat(PxReal f)
{
	const PxU32 i = __float_as_int(f);
	// PT: same as CPU-side "encodeFloat" but using the ternary operator. It is a mystery why we kept both versions...
	return i & PX_SIGN_BITMASK ? ~i : i | PX_SIGN_BITMASK;
}

// PT: shared objects (like ground planes) get assigned bounds that cover the entire space, so that they properly collide with
// all scattered envs. To avoid it, users should properly duplicate these objects in each env and give them a proper env ID.
#define gEncodedMinExtent	0x01800000	// encodeFloat(-PX_MAX_BOUNDS_EXTENTS) & (~1);
#define gEncodedMaxExtent	0xfe7fffff	// (encodeFloat(PX_MAX_BOUNDS_EXTENTS) & (~1)) | 1;

extern "C" __global__ void translateAABBsLaunch(const PxBounds3* PX_RESTRICT inArray,
												PxgIntegerAABB* PX_RESTRICT outArray,
												const PxReal* PX_RESTRICT contactDistances,
												const PxU32* PX_RESTRICT envIds,
												PxU32 boxesCapacity,
												PxU8 nbBitsShiftX, PxU8 nbBitsShiftY, PxU8 nbBitsShiftZ,
												PxU8 nbBitsEnvIDX, PxU8 nbBitsEnvIDY, PxU8 nbBitsEnvIDZ)	// BP_TRANSLATE_AABBS //###ONESHOT
{
	// translate the bounds in the array from float to PxgIntegerAABB, using 8 threads per AABB
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;
	// PT: max globalThreadIndex = 256 + 256 * nbBlocks = ~8x the capacity

	const PxU32 boxIndex = globalThreadIndex>>3;

	if(boxIndex < boxesCapacity)	// PT: TODO: why do we use capacity here? More than what's needed?
	{
		const PxU32 shifts[6] = {	nbBitsShiftX, nbBitsShiftY, nbBitsShiftZ,
									nbBitsShiftX, nbBitsShiftY, nbBitsShiftZ	};
		const PxU32 envIdShifts[6] = {	nbBitsEnvIDX, nbBitsEnvIDY, nbBitsEnvIDZ,
										nbBitsEnvIDX, nbBitsEnvIDY, nbBitsEnvIDZ	};

		const PxReal contactDistance = contactDistances[boxIndex];
		const PxU32 a = threadIdx.x & 7;	// PT: same as "globalThreadIndex & 7"
		if(a<6)	// PT: only 6 values per box to translate, 2 threads do nothing
		{
			// PT: read input coordinate that needs translating, will be a min (a<3) or a max (a>=3)
			const PxReal in = (reinterpret_cast<const PxReal*>(inArray + boxIndex))[a];

			// PT: mask = 0 for min, mask = 1 for max
			const PxU32 mask = a < 3 ? 0 : 1;
			// PT: runtime-inflated bounds
			const PxReal projection = a < 3 ? in - contactDistance : in + contactDistance;

			// PT: encode inflated coordinate & write it out
			const PxU32 shift = shifts[a];
			const PxU32 envIdShift = envIdShifts[a];

			const PxU32 b = (encodeFloat(projection) >> shift) & (~1);
			PxU32 out = b | mask;

			if(envIds && envIdShift)
			{
				const PxU32 envId = envIds[boxIndex];
				if(envId != PX_INVALID_U32)
					out |= envId << (32 - envIdShift);
				else
					out = mask ? gEncodedMaxExtent : gEncodedMinExtent;
			}

			outArray[boxIndex].mMinMax[a] = out;
		}
	}
}

//mark the handle as deleted
static __device__ PX_FORCE_INLINE void clearHandles(const PxU32 handle, const PxgSapBox1D* boxSapBox1D, PxU32* boxHandle)
{
	const PxgSapBox1D& sapBox = boxSapBox1D[handle];
	const PxU32 minIndex = sapBox.mMinMax[0];
	const PxU32 maxIndex = sapBox.mMinMax[1]; 

	boxHandle[minIndex] = markDeleted(boxHandle[minIndex]);
	boxHandle[maxIndex] = markDeleted(boxHandle[maxIndex]);
}

//CPU code should checked the removed handle size before we kick off this taks
extern "C" __global__ void markRemovedPairsLaunch(const PxgBroadPhaseDesc* bpDesc)	// BP_MARK_DELETEDPAIRS
{
	const PxU32 numRemovedHandleSize = bpDesc->numRemovedHandles;

	//We mark this in the previous frame's handles. Then, after sorting, we use the ranks to create the current
	//sorted handle list from the previous handle list. 

	PxU32* boxHandleX = bpDesc->boxHandles[1][0];//previous frame's handle
	PxU32* boxHandleY = bpDesc->boxHandles[1][1];
	PxU32* boxHandleZ = bpDesc->boxHandles[1][2];

	const PxgSapBox1D* boxSapBox1DX = bpDesc->boxSapBox1D[0];
	const PxgSapBox1D* boxSapBox1DY = bpDesc->boxSapBox1D[1];
	const PxgSapBox1D* boxSapBox1DZ = bpDesc->boxSapBox1D[2];

	PxgIntegerAABB* bounds = bpDesc->oldIntegerBounds;

	const PxU32* removedHandles = bpDesc->updateData_removedHandles;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x*blockIdx.x;

	for(PxU32 i=globalThreadIndex; i<numRemovedHandleSize; i+=blockDim.x*gridDim.x)
	{
		const PxU32 handle = removedHandles[i];
		clearHandles(handle, boxSapBox1DX, boxHandleX);
		clearHandles(handle, boxSapBox1DY, boxHandleY);
		clearHandles(handle, boxSapBox1DZ, boxHandleZ);

		bounds[handle].setEmpty();
	}
}  

//set projections and sap box as invalid
static __device__ PX_FORCE_INLINE void clearPairs(const PxU32 handle, PxgSapBox1D* boxSapBox1D, PxU32* boxProjection)
{
	PxgSapBox1D& sapBox = boxSapBox1D[handle];
	const PxU32 minIndex = sapBox.mMinMax[0];
	const PxU32 maxIndex = sapBox.mMinMax[1]; 

	boxProjection[minIndex] = PXG_INVALID_BP_PROJECTION;
	boxProjection[maxIndex] = PXG_INVALID_BP_PROJECTION;

	sapBox.mMinMax[0] = PXG_INVALID_BP_SAP_BOX;
	sapBox.mMinMax[1] = PXG_INVALID_BP_SAP_BOX;
}

//This method will be called after we perform incremental sap to make the projection as PXG_INVALID_BP_PROJECTION(0xffffffff) so that 
//when we perform the second sort for the projection with removed pairs and newly inserted pairs, the removed pairs will be shuffle to
//the end of the list
extern "C" __global__ void markRemovedPairsProjectionsLaunch(const PxgBroadPhaseDesc* bpDesc)	// BP_UPDATE_DELETEDPAIRS
{
	const PxU32 numRemovedHandleSize = bpDesc->numRemovedHandles;

	PxU32* boxProjectionX = bpDesc->boxProjections[0];
	PxU32* boxProjectionY = bpDesc->boxProjections[1];
	PxU32* boxProjectionZ = bpDesc->boxProjections[2];

	PxgSapBox1D* boxSapBox1DX = bpDesc->boxSapBox1D[0];
	PxgSapBox1D* boxSapBox1DY = bpDesc->boxSapBox1D[1];
	PxgSapBox1D* boxSapBox1DZ = bpDesc->boxSapBox1D[2];

	const PxU32* removedHandles = bpDesc->updateData_removedHandles;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x*blockIdx.x;

	for(PxU32 i=globalThreadIndex; i<numRemovedHandleSize; i+=blockDim.x*gridDim.x)
	{
		const PxU32 handle = removedHandles[i];
		clearPairs(handle, boxSapBox1DX, boxProjectionX);
		clearPairs(handle, boxSapBox1DY, boxProjectionY);
		clearPairs(handle, boxSapBox1DZ, boxProjectionZ);
	}
}   

//This kernel will be called when we have new pairs inserted into the scene
extern "C" __global__ void markCreatedPairsLaunch(const PxgBroadPhaseDesc* bpDesc)	// BP_UPDATE_CREATEDPAIRS //###ONESHOT
{
	const PxU32 numCreatedHandleSize = bpDesc->numCreatedHandles;

	//because we didn't actually get rid of the removed handles(we just mark the handles as deleted), we should append the newly created handle at the position of
	//the previous handles
	const PxU32 numInitialHandles = bpDesc->numPreviousHandles;

	const PxU32 startIndex = numInitialHandles;

	const PxU32* createdHandles = bpDesc->updateData_createdHandles;

	uint2* boxProjectionsX = reinterpret_cast<uint2*>(bpDesc->boxProjections[AXIS_X]);
	uint2* boxProjectionsY = reinterpret_cast<uint2*>(bpDesc->boxProjections[AXIS_Y]);
	uint2* boxProjectionsZ = reinterpret_cast<uint2*>(bpDesc->boxProjections[AXIS_Z]);

	uint2* boxHandlesX = reinterpret_cast<uint2*>(bpDesc->boxHandles[0][AXIS_X]); // current frame x axis box handles
	uint2* boxHandlesY = reinterpret_cast<uint2*>(bpDesc->boxHandles[0][AXIS_Y]);
	uint2* boxHandlesZ = reinterpret_cast<uint2*>(bpDesc->boxHandles[0][AXIS_Z]);

	const PxgIntegerAABB* boxBoundsMinMax = bpDesc->newIntegerBounds;
	PxgIntegerAABB* oldBounds = bpDesc->oldIntegerBounds;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x*blockIdx.x;

	for(PxU32 i=globalThreadIndex; i<numCreatedHandleSize; i+=blockDim.x*gridDim.x)
	{
		const PxU32 handle = createdHandles[i];

		const PxgIntegerAABB& iaabb = boxBoundsMinMax[handle];

		const PxU32 index = startIndex + i;

		const uint2 startEndHandle = make_uint2(createHandle(handle, true, true), createHandle(handle, false, true));

		const uint2 projectionX = make_uint2(iaabb.getMin(0), iaabb.getMax(0));
		const uint2 projectionY = make_uint2(iaabb.getMin(1), iaabb.getMax(1));
		const uint2 projectionZ = make_uint2(iaabb.getMin(2), iaabb.getMax(2));

		boxProjectionsX[index] = projectionX;
		boxProjectionsY[index] = projectionY;
		boxProjectionsZ[index] = projectionZ;

		boxHandlesX[index] = startEndHandle;
		boxHandlesY[index] = startEndHandle;
		boxHandlesZ[index] = startEndHandle;

		oldBounds[handle].setEmpty();
	}

	//we need to pad the handles to the multiply of 4 for the radix sort
	const PxU32 numExistingProjections = numCreatedHandleSize + startIndex;
	
	const PxU32 remainingProjections = numExistingProjections&1;
	
	if(globalThreadIndex < remainingProjections)
	{
		const PxU32 index = numExistingProjections + globalThreadIndex;
		
		boxProjectionsX[index] = make_uint2(PXG_INVALID_BP_PROJECTION, PXG_INVALID_BP_PROJECTION);
		boxHandlesX[index] = make_uint2(PXG_INVALID_BP_HANDLE, PXG_INVALID_BP_HANDLE);
		boxProjectionsY[index] = make_uint2(PXG_INVALID_BP_PROJECTION, PXG_INVALID_BP_PROJECTION);
		boxHandlesY[index] = make_uint2(PXG_INVALID_BP_HANDLE, PXG_INVALID_BP_HANDLE);
		boxProjectionsZ[index] = make_uint2(PXG_INVALID_BP_PROJECTION, PXG_INVALID_BP_PROJECTION);
		boxHandlesZ[index] = make_uint2(PXG_INVALID_BP_HANDLE, PXG_INVALID_BP_HANDLE);
	}
}

//handles has been sorted when we are calling the radix sort kernel, so in this kernel, we need to get out the sorted handle and initialize PxgSapBox1D
//we call this function twice: one is for incremental sap, the other is for generated new pairs
extern "C" __global__ void initializeSapBox1DLaunch(const PxgBroadPhaseDesc* bpDesc, const PxU32 numHandles, bool isNew)	// BP_INITIALIZE_SAPBOX //###ONESHOT
{
	//const PxU32 numHandles = bpDesc->numPreviousHandles + bpDesc->numCreatedHandles - bpDesc->numRemovedHandles;

	const PxU32 numProjections = numHandles*2;
	const PxU32* boxHandlesX = bpDesc->boxHandles[0][0]; // current frame's handle
	const PxU32* boxHandlesY = bpDesc->boxHandles[0][1]; //bpDesc->boxHandles[1];
	const PxU32* boxHandlesZ = bpDesc->boxHandles[0][2];//bpDesc->boxHandles[2];
	
	PxgSapBox1D* boxSapBox1DX = isNew ? bpDesc->boxNewSapBox1D[0] : bpDesc->boxSapBox1D[0];
	PxgSapBox1D* boxSapBox1DY = isNew ? bpDesc->boxNewSapBox1D[1] : bpDesc->boxSapBox1D[1];
	PxgSapBox1D* boxSapBox1DZ = isNew ? bpDesc->boxNewSapBox1D[2] : bpDesc->boxSapBox1D[2];

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x*blockIdx.x;

	//for(PxU32 i=globalThreadIndex; i<numProjections; i+=blockDim.x*gridDim.x)
	if(globalThreadIndex < numProjections)
	{
		PxU32 i = globalThreadIndex;
		const PxU32 sortedHandleX = boxHandlesX[i];
		const PxU32 sortedHandleY = boxHandlesY[i];
		const PxU32 sortedHandleZ = boxHandlesZ[i];

		const PxU32 handleX = getHandle(sortedHandleX);
		const PxU32 handleY = getHandle(sortedHandleY);
		const PxU32 handleZ = getHandle(sortedHandleZ);

		PxgSapBox1D& sapBoxX = boxSapBox1DX[handleX];
		PxgSapBox1D& sapBoxY = boxSapBox1DY[handleY];
		PxgSapBox1D& sapBoxZ = boxSapBox1DZ[handleZ];

		sapBoxX.mMinMax[!isStartProjection(sortedHandleX)] = i;
		sapBoxY.mMinMax[!isStartProjection(sortedHandleY)] = i;
		sapBoxZ.mMinMax[!isStartProjection(sortedHandleZ)] = i;
	}
}

extern "C" __global__ void markUpdatedPairsLaunch(const PxgBroadPhaseDesc* bpDesc)	// BP_UPDATE_UPDATEDPAIRS
{
	//PxU32 nbUpdatedHandles = bpDesc->numUpdatedHandles;
	//OK, we have some updated handles. First, we need to update all the values in the array.

	//PxU32* updatedHandles = bpDesc->updatedHandles;

	const PxU32 numElements = bpDesc->aabbMngr_changedHandleBitMapWordCounts * 32;

	const PxU32* changedAABBMgrHandles = bpDesc->aabbMngr_changedHandleMap;

	//PxU32 numBounds = bpDesc->numBounds;

	const PxU32* addedHandles = bpDesc->aabbMngr_addedHandleMap;
	const PxU32* removedHandles = bpDesc->aabbMngr_removedHandleMap;

	PxU32* boxProjectionX = bpDesc->boxProjections[AXIS_X];
	PxU32* boxProjectionY = bpDesc->boxProjections[AXIS_Y];
	PxU32* boxProjectionZ = bpDesc->boxProjections[AXIS_Z];

	const PxgSapBox1D* box1dX = bpDesc->boxSapBox1D[AXIS_X];
	const PxgSapBox1D* box1dY = bpDesc->boxSapBox1D[AXIS_Y];
	const PxgSapBox1D* box1dZ = bpDesc->boxSapBox1D[AXIS_Z];
	
	const PxgIntegerAABB* newBounds = bpDesc->newIntegerBounds;

	const PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	const PxU32 threadIndexInWarp = threadIdx.x &  (WARP_SIZE - 1);

	for(PxU32 handle = globalThreadIdx; handle < numElements; handle += blockDim.x*gridDim.x)
	{
		const PxU32 word = handle / 32;
		const PxU32 mask = changedAABBMgrHandles[word];

		//printf("mask %i\n", mask);

#if PX_DEBUG
		const PxU32* aggregatedBoundHandles = bpDesc->aabbMngr_aggregatedBoundHandles;
		const PxU32 aggregatedBoundMask = aggregatedBoundHandles[word];
		assert(!(mask & aggregatedBoundMask));
#endif

		const PxU32 addedBoundMask = addedHandles[word];
		const PxU32 removedBoundMask = removedHandles[word];

		const PxU32 addedOrRemoved = addedBoundMask | removedBoundMask;

		if ((mask & (~addedOrRemoved)) & (1 << threadIndexInWarp))
		{
			const PxgIntegerAABB& iaabb = newBounds[handle];

			const PxgSapBox1D& sapBoxX = box1dX[handle];
			const PxgSapBox1D& sapBoxY = box1dY[handle];
			const PxgSapBox1D& sapBoxZ = box1dZ[handle];

			const PxU32 minX = iaabb.getMin(AXIS_X);
			const PxU32 maxX = iaabb.getMax(AXIS_X);

			const PxU32 minY = iaabb.getMin(AXIS_Y);
			const PxU32 maxY = iaabb.getMax(AXIS_Y);

			const PxU32 minZ = iaabb.getMin(AXIS_Z);
			const PxU32 maxZ = iaabb.getMax(AXIS_Z);

			boxProjectionX[sapBoxX.mMinMax[0]] = minX;
			boxProjectionX[sapBoxX.mMinMax[1]] = maxX;

			boxProjectionY[sapBoxY.mMinMax[0]] = minY;
			boxProjectionY[sapBoxY.mMinMax[1]] = maxY;

			boxProjectionZ[sapBoxZ.mMinMax[0]] = minZ;
			boxProjectionZ[sapBoxZ.mMinMax[1]] = maxZ;
		}
	}
}

#ifdef SUPPORT_UPDATE_HANDLES_ARRAY_FOR_GPU
extern "C" __global__ void markUpdatedPairsLaunch2(const PxgBroadPhaseDesc* bpDesc)	// BP_UPDATE_UPDATEDPAIRS2
{
	//PxU32 nbUpdatedHandles = bpDesc->numUpdatedHandles;
	//OK, we have some updated handles. First, we need to update all the values in the array.

	//PxU32* updatedHandles = bpDesc->updatedHandles;

	// PT: I have no idea what I'm doing but this version makes the UT pass  ...   :)

	// PT: trying to resurrect the above code & buffer
	const PxU32 nbUpdatedHandles = bpDesc->numUpdatedHandles;
	const PxU32* updatedHandles = bpDesc->updateData_updatedHandles;

	PxU32* boxProjectionX = bpDesc->boxProjections[AXIS_X];
	PxU32* boxProjectionY = bpDesc->boxProjections[AXIS_Y];
	PxU32* boxProjectionZ = bpDesc->boxProjections[AXIS_Z];

	const PxgSapBox1D* box1dX = bpDesc->boxSapBox1D[AXIS_X];
	const PxgSapBox1D* box1dY = bpDesc->boxSapBox1D[AXIS_Y];
	const PxgSapBox1D* box1dZ = bpDesc->boxSapBox1D[AXIS_Z];
	
	const PxgIntegerAABB* newBounds = bpDesc->newIntegerBounds;

	// PT: TODO: understand this loop that I copied from other kernels

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x*blockIdx.x;

	for(PxU32 i=globalThreadIndex; i<nbUpdatedHandles; i+=blockDim.x*gridDim.x)
	{
		const PxU32 handle = updatedHandles[i];

		const PxgIntegerAABB& iaabb = newBounds[handle];

		const PxgSapBox1D& sapBoxX = box1dX[handle];
		const PxgSapBox1D& sapBoxY = box1dY[handle];
		const PxgSapBox1D& sapBoxZ = box1dZ[handle];

		const PxU32 minX = iaabb.getMin(AXIS_X);
		const PxU32 maxX = iaabb.getMax(AXIS_X);

		const PxU32 minY = iaabb.getMin(AXIS_Y);
		const PxU32 maxY = iaabb.getMax(AXIS_Y);

		const PxU32 minZ = iaabb.getMin(AXIS_Z);
		const PxU32 maxZ = iaabb.getMax(AXIS_Z);

		boxProjectionX[sapBoxX.mMinMax[0]] = minX;
		boxProjectionX[sapBoxX.mMinMax[1]] = maxX;

		boxProjectionY[sapBoxY.mMinMax[0]] = minY;
		boxProjectionY[sapBoxY.mMinMax[1]] = maxY;

		boxProjectionZ[sapBoxZ.mMinMax[0]] = minZ;
		boxProjectionZ[sapBoxZ.mMinMax[1]] = maxZ;
	}
}
#endif

static __device__ void firstPassEndPtsHistogram(
	const PxU32* const PX_RESTRICT boxHandles, const PxU32 numHandles, PxU32* PX_RESTRICT blocksEndPtsAccum, PxU32* PX_RESTRICT blocksStartPtsAccum,
	PxU32* PX_RESTRICT endPtsHistogram, PxU32* PX_RESTRICT startPtsHistogram, /*PxgSapBox1D* box1D,*/ PxU32* PX_RESTRICT endPtsHistogram2, PxU32* PX_RESTRICT blocksEndPtsAccum2)
{
	//if we are using shfl, the compiler should strip this out
	//__shared__ PxU32 sEndPtsAccum[WARP_PERBLOCK_SIZE*WARP_SIZE];

	const PxU32 WARP_PERBLOCK_SIZE = PxgBPKernelBlockDim::BP_COMPUTE_ENDPT_HISTOGRAM / WARP_SIZE;

	//For now, assume that all regions are the same dimension!
	//const PxU32 stride = (numProjections + PXG_BP_REGION_X-1)/PXG_BP_REGION_X; 

	__shared__ PxU32 sWarpEndPtsAccum[WARP_PERBLOCK_SIZE];
	__shared__ PxU32 sAccumStart;
	__shared__ PxU32 sAccumEnd;	
	__shared__ PxU32 sAccumEnd2;	

	const PxU32 numProjections = numHandles*2;	

	const PxU32 totalBlockRequired = (numProjections + (blockDim.x-1))/ blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (BLOCK_SIZE-1))/ BLOCK_SIZE;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE-1);
	const PxU32 warpIndex = threadIdx.x/(WARP_SIZE);

	if(idx == (WARP_PERBLOCK_SIZE-1))
	{
		sAccumEnd = 0;
		sAccumEnd2 = 0;
		sAccumStart = 0;
	}

	__syncthreads();

	for(PxU32 i=0; i<numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i*blockDim.x + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		PxU32 start = 0, end = 0, end2 = 0;
		if(workIndex < numProjections)
		{
			//KS - we exclude bounds that aren't in this region here.
			{
				const PxU32 sortedHandle = boxHandles[workIndex];
				const PxU32 handle = getHandle(sortedHandle);

				//printf("WorkIndex = %i handle %i in region 0\n", workIndex, handle);

				end2 = 1 - isStartProjection(sortedHandle);
				if (!isDeletedProjection(sortedHandle))
				{
					start = 1 - end2;
					end = end2;
				}
			}
		}

		const PxU32 threadMask = (1<<threadIndexInWarp)-1;

		const PxU32 startAccum = __popc(__ballot_sync(FULL_MASK, start)&threadMask);
		const PxU32 endAccum = __popc(__ballot_sync(FULL_MASK, end)&threadMask);
		const PxU32 end2Accum = __popc(__ballot_sync(FULL_MASK, end2)&threadMask);

		//KS - we can use 10 bits per counter because each warp can produce at most 32 of each number. There are 8 warps per block,
		//so the limit is 256, which is below the 1024 max value we can store in 10 bits. This avoids needing to do 3x 
		//reductions to sum 3x smaller numbers below
		PxU32 val = (start<<20)|(end<<10) | end2;
		PxU32 endPtAccum = ((startAccum <<20) | (endAccum << 10) | end2Accum);

		//we can use shfl but __ballot and __popc should be faster expecially for Fermi which doesn't support shfl
		//PxU32 val = (start<<20)|(end<<10) | end2;
		//PxU32 endPtAccum2 = warpScanAdd<WARP_SIZE>(idx, threadIndexInWarp, sEndPtsAccum, val, val);

		if(threadIndexInWarp == (WARP_SIZE-1))
			sWarpEndPtsAccum[warpIndex] = endPtAccum + val;

		//assert(endPtAccum2 == endPtAccum);
		const PxU32 prevAccumEnd = sAccumEnd;
		const PxU32 prevAccumEnd2 = sAccumEnd2;
		const PxU32 prevAccumStart = sAccumStart;

		__syncthreads();

		unsigned mask_idx = __ballot_sync(FULL_MASK, idx < WARP_PERBLOCK_SIZE);
		if(idx < WARP_PERBLOCK_SIZE)
		{
			const PxU32 value = sWarpEndPtsAccum[threadIndexInWarp];

			const PxU32 res = warpScanAddWriteToSharedMem<WARP_PERBLOCK_SIZE>(mask_idx, threadIndexInWarp, threadIndexInWarp, sWarpEndPtsAccum, value, value);

			if(threadIndexInWarp  == (WARP_PERBLOCK_SIZE - 1))
			{
				PxU32 accum = (res + value);
				sAccumEnd2 += (accum & ((1<<10)-1));
				sAccumEnd +=((accum>>10) & ((1<<10)-1));
				sAccumStart += accum>>20;
			}
		}

		__syncthreads();

		if(workIndex < numProjections)
		{
			const PxU32 accumulation = endPtAccum + sWarpEndPtsAccum[warpIndex];

			const PxU32 endCount = ((accumulation>>10) & ((1<<10)-1)) + prevAccumEnd;
			const PxU32 startCount = (accumulation>>20) + prevAccumStart;

			if(endPtsHistogram2)
			{
				const PxU32 end2Count = (accumulation & ((1<<10)-1)) + prevAccumEnd2;
				endPtsHistogram2[workIndex] = end2Count;
			}
			endPtsHistogram[workIndex] = endCount;
			startPtsHistogram[workIndex] = startCount;
		}
	}

	if(idx == (WARP_PERBLOCK_SIZE-1))
	{
		if(blocksEndPtsAccum2)
			blocksEndPtsAccum2[blockIdx.x] = sAccumEnd2;
		blocksEndPtsAccum[blockIdx.x] = sAccumEnd;
		blocksStartPtsAccum[blockIdx.x] = sAccumStart;
	}
}

static __device__ void secondPassEndPtsHistogram(
	const PxU32* blockEndPtHistogram, PxU32* endPtsHistogram, PxU32* orderedEndPtsHandles, const PxU32* orderedHandles,
	const PxU32* blockStartPtHistogram, PxU32* startPtsHistogram, PxU32* orderedStartPtsHandles, const PxU32 numHandles,// const PxgSapBox1D* /*box1D*/,
	PxU32* totalEndPtsHistogram, const PxU32* blockTotalEndPtHistogram)
{
	const PxU32 WARP_PERBLOCK_SIZE = PxgBPKernelBlockDim::BP_COMPUTE_ENDPT_HISTOGRAM / WARP_SIZE;

	__shared__ PxU32 sBlockEndPtHistogram[BLOCK_SIZE];
	__shared__ PxU32 sBlockTotalEndPtHistogram[BLOCK_SIZE];
	__shared__ PxU32 sBlockStartPtHistogram[BLOCK_SIZE];

	const PxU32 idx = threadIdx.x;

	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE-1);  
	const PxU32 warpIndex = threadIdx.x/WARP_SIZE;

	{
		unsigned mask_warpIndex = __ballot_sync(FULL_MASK, warpIndex == 0 && threadIndexInWarp < BLOCK_SIZE);
		if (warpIndex == 0 && threadIndexInWarp < BLOCK_SIZE)
		{
			const PxU32 valEnd = blockEndPtHistogram[threadIndexInWarp];
			warpScanAddWriteToSharedMem<BLOCK_SIZE>(mask_warpIndex, threadIndexInWarp, threadIndexInWarp, sBlockEndPtHistogram, valEnd, valEnd);
		}
	}
	{
		unsigned mask_warpIndex = __ballot_sync(FULL_MASK, warpIndex == 1 && threadIndexInWarp < BLOCK_SIZE);
		if (warpIndex == 1 && threadIndexInWarp < BLOCK_SIZE)
		{
			const PxU32 valStart = blockStartPtHistogram[threadIndexInWarp];
			warpScanAddWriteToSharedMem<BLOCK_SIZE>(mask_warpIndex, threadIndexInWarp, threadIndexInWarp, sBlockStartPtHistogram, valStart, valStart);
		}
	}
	{
		unsigned mask_warpIndex = __ballot_sync(FULL_MASK, warpIndex == 2 && threadIndexInWarp < BLOCK_SIZE && totalEndPtsHistogram != NULL);
		if (warpIndex == 2 && threadIndexInWarp < BLOCK_SIZE && totalEndPtsHistogram != NULL)
		{
			const PxU32 valStart = blockTotalEndPtHistogram[threadIndexInWarp];
			warpScanAddWriteToSharedMem<BLOCK_SIZE>(mask_warpIndex, threadIndexInWarp, threadIndexInWarp, sBlockTotalEndPtHistogram, valStart, valStart);
		}
	}

	__syncthreads();

	__shared__ PxU32* sPtsHistograms[2];
	__shared__ PxU32* sOrderedHandles[2];
	__shared__ PxU32 sBlockAccum[2];

	if(threadIdx.x == 0)
	{
		sPtsHistograms[0] = endPtsHistogram;
		sPtsHistograms[1] = startPtsHistogram;
		sOrderedHandles[0] = orderedEndPtsHandles;
		sOrderedHandles[1] = orderedStartPtsHandles;
		sBlockAccum[0] = sBlockEndPtHistogram[blockIdx.x];
		sBlockAccum[1] = sBlockStartPtHistogram[blockIdx.x];
	}

	const PxU32 numProjections = numHandles*2;

	const PxU32 totalBlockRequired = (numProjections + (blockDim.x-1))/ blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (BLOCK_SIZE-1))/ BLOCK_SIZE;

	__syncthreads();

	for(PxU32 i=0; i<numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i*WARP_SIZE*WARP_PERBLOCK_SIZE + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		if(workIndex < numProjections)
		{
			const PxU32 sortedHandle = orderedHandles[workIndex];
			const PxU32 startProjection = isStartProjection(sortedHandle);
			const bool isNewOrDeleted = isDeletedProjection(sortedHandle);

			const PxU32 outIndex = sPtsHistograms[startProjection][workIndex] + sBlockAccum[startProjection];
			const PxU32 outIndex2 = sPtsHistograms[1-startProjection][workIndex] + sBlockAccum[1-startProjection];
			
			sPtsHistograms[startProjection][workIndex] = outIndex;
			sPtsHistograms[1-startProjection][workIndex] = outIndex2;

			if(!isNewOrDeleted)
				sOrderedHandles[startProjection][outIndex] = getHandle(sortedHandle);
			
			//printf("Region %i: workIndex = %i, outIndex = %i, outIndex2 = %i\n", blockIdx.z, workIndex, outIndex, outIndex2);

			if(totalEndPtsHistogram)
			{
				const PxU32 outIndexTotalEnd = totalEndPtsHistogram[workIndex] + sBlockTotalEndPtHistogram[blockIdx.x];
				totalEndPtsHistogram[workIndex] = outIndexTotalEnd;			
			}
		}
	}
}

//This function is called before we create regions. We need to create three end points histogram for three axis. The idea for using this three
//end points histogram is to do spatial partitioning based on the exclusive end points.
extern "C" __global__ void computeEndPtsHistogram(const PxgBroadPhaseDesc* bpDesc, const bool isIncremental)	// BP_COMPUTE_ENDPT_HISTOGRAM //###ONESHOT
{
	//for(PxU32 axis = 0; axis < 3; ++axis)
	PxU32 axis = blockIdx.y;
	
	{
		//unsigned mask_isIncremental = __ballot_sync(FULL_MASK, isIncremental);
		//KS - we know that isIncremental is either true for all threads, or false
		if(isIncremental)
		{
			//incremental sap won't generate pairs for the newly inserted pairs so in the CPU code, we defer the insertion after the incremental sap.
			const PxU32 numHandles = bpDesc->numPreviousHandles;

			const PxU32* currentBoxHandles = bpDesc->boxHandles[0][axis];
			firstPassEndPtsHistogram(currentBoxHandles, numHandles, bpDesc->blockEndPtHistogram[0][axis], bpDesc->blockStartPtHistogram[0][axis], bpDesc->endPtHistogram[0][axis],
				bpDesc->startPtHistogram[0][axis], bpDesc->totalEndPtHistogram[axis], bpDesc->blockTotalEndPtHistogram[axis]);

			//we need to re-scan the end point for removal
			//unsigned mask_numRemovedHandles = __ballot_sync(mask_isIncremental, bpDesc->numRemovedHandles != 0);
			//We know that mask_isIncremental wil be FULL_MASK, and we know that bpDesc->numRemovedHandles be either 0 or != 0 for all threads
			if(bpDesc->numRemovedHandles != 0)
			{
				const PxU32* previousBoxHandles = bpDesc->boxHandles[1][axis];
				firstPassEndPtsHistogram(previousBoxHandles, bpDesc->numPreviousHandles , bpDesc->blockEndPtHistogram[1][axis], bpDesc->blockStartPtHistogram[1][axis], bpDesc->endPtHistogram[1][axis],
					bpDesc->startPtHistogram[1][axis], NULL, NULL);
			}
		}
		else
		{
			const PxU32 numHandles = bpDesc->numPreviousHandles + bpDesc->numCreatedHandles - bpDesc->numRemovedHandles;

			const PxU32* currentBoxHandles = bpDesc->boxHandles[0][axis];
			firstPassEndPtsHistogram(currentBoxHandles, numHandles, bpDesc->blockEndPtHistogram[0][axis], bpDesc->blockStartPtHistogram[0][axis], bpDesc->endPtHistogram[0][axis],
				bpDesc->startPtHistogram[0][axis], bpDesc->totalEndPtHistogram[axis], bpDesc->blockTotalEndPtHistogram[axis]);
		}
	}
}

//This function is called after computeEndPtsHistogramWithinABlockKernel before we create regions
// PT: the aforementioned kernel ("computeEndPtsHistogramWithinABlockKernel") doesn't exist anymore => probably computeEndPtsHistogram now
extern "C" __global__ void outputEndPtsHistogram(const PxgBroadPhaseDesc* bpDesc, const bool isIncremental)	// BP_OUTPUT_ENDPT_HISTOGRAM //###ONESHOT
{
	PxU32 axis = blockIdx.y;

	//for(PxU32 axis = 0; axis < 3; ++axis)
	{
		//unsigned mask_isIncremental = __ballot_sync(FULL_MASK, isIncremental);
		if(isIncremental)
		{
			const PxU32 numHandles = bpDesc->numPreviousHandles;

			PxU32* currentBoxHandles = bpDesc->boxHandles[0][axis];
			secondPassEndPtsHistogram(bpDesc->blockEndPtHistogram[0][axis], bpDesc->endPtHistogram[0][axis], bpDesc->endPointHandles[0][axis], currentBoxHandles,
				bpDesc->blockStartPtHistogram[0][axis], bpDesc->startPtHistogram[0][axis], bpDesc->startPointHandles[0][axis], numHandles,// bpDesc->boxSapBox1D[axis],
				bpDesc->totalEndPtHistogram[axis], bpDesc->blockTotalEndPtHistogram[axis]);

			//we need to re-scan the end point for removal
			//unsigned mask_numRemovedHandles = __ballot_sync(mask_isIncremental, bpDesc->numRemovedHandles != 0);
			if(bpDesc->numRemovedHandles != 0)
			{
				PxU32* previousBoxHandles = bpDesc->boxHandles[1][axis];
				secondPassEndPtsHistogram(bpDesc->blockEndPtHistogram[1][axis], bpDesc->endPtHistogram[1][axis], bpDesc->endPointHandles[1][axis], previousBoxHandles,
					bpDesc->blockStartPtHistogram[1][axis], bpDesc->startPtHistogram[1][axis], bpDesc->startPointHandles[1][axis], bpDesc->numPreviousHandles,// bpDesc->boxSapBox1D[axis],
					NULL, NULL);
			}
		}
		else
		{
			const PxU32 numHandles = bpDesc->numPreviousHandles + bpDesc->numCreatedHandles - bpDesc->numRemovedHandles;

			PxU32* currentBoxHandles = bpDesc->boxHandles[0][axis];
			secondPassEndPtsHistogram(bpDesc->blockEndPtHistogram[0][axis], bpDesc->endPtHistogram[0][axis], bpDesc->endPointHandles[0][axis], currentBoxHandles,
				bpDesc->blockStartPtHistogram[0][axis], bpDesc->startPtHistogram[0][axis], bpDesc->startPointHandles[0][axis], numHandles,// bpDesc->boxSapBox1D[axis],
				bpDesc->totalEndPtHistogram[axis], bpDesc->blockTotalEndPtHistogram[axis]);
		}
	}
}

extern "C" __global__ void createRegionsKernel(const PxgBroadPhaseDesc* bpDesc)	// BP_CREATE_REGIONS //###ONESHOT
{
	const PxU32* endPtsHistogramX = bpDesc->totalEndPtHistogram[AXIS_X];
	const PxU32* endPtsHistogramY = bpDesc->totalEndPtHistogram[AXIS_Y];
	const PxU32* endPtsHistogramZ = bpDesc->totalEndPtHistogram[AXIS_Z];

	const PxU32* handlesX = bpDesc->boxHandles[0][AXIS_X];//using current frame x axis
	const PxgSapBox1D* sapBoxX = bpDesc->boxSapBox1D[AXIS_X];
	const PxgSapBox1D* sapBoxY = bpDesc->boxSapBox1D[AXIS_Y];
	const PxgSapBox1D* sapBoxZ = bpDesc->boxSapBox1D[AXIS_Z];

	PxgIntegerRegion* regionRange = bpDesc->regionRange;

	PxU32* startRegionAccum = bpDesc->startRegionAccum;

	PxU32* regionAccum = bpDesc->regionAccum;

	const PxU32 numHandles = bpDesc->numPreviousHandles + bpDesc->numCreatedHandles - bpDesc->numRemovedHandles;
	const PxU32 numProjections = numHandles*2;

	const PxU32 totalBlockRequired = (numProjections + (blockDim.x-1))/ blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (BLOCK_SIZE-1))/ BLOCK_SIZE;

	const PxU32 elementsPerRegions = (numHandles + (REGION_SIZE_PER_AXIS-1))/ REGION_SIZE_PER_AXIS;

	const PxU32 idx = threadIdx.x;

	for(PxU32 i=0; i<numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i*PxgBPKernelBlockDim::BP_CREATE_REGIONS + idx + numIterationPerBlock * blockIdx.x * blockDim.x;
		if(workIndex < numProjections)
		{
			//get handle
			const PxU32 sortedHandle = handlesX[workIndex];
			PxU32 regionCount = 0;
			if(isStartProjection(sortedHandle))
			{
				const PxU32 handle = getHandle(sortedHandle);

				//calculate region index in x axis
				const PxgSapBox1D& sapX = sapBoxX[handle];
				const PxU32 startIndex = sapX.mMinMax[0];
				const PxU32 endIndex = sapX.mMinMax[1];
				
				const PxU32 startIndexX = endPtsHistogramX[startIndex];
				const PxU32 endIndexX = endPtsHistogramX[endIndex];
				const PxU32 sRegionX = startIndexX / elementsPerRegions;
				const PxU32 eRegionX = endIndexX / elementsPerRegions;
		
				//calculate region index in y axis
				const PxgSapBox1D& sapY = sapBoxY[handle];
				PxU32 startIndexY = sapY.mMinMax[0];
				PxU32 endIndexY = sapY.mMinMax[1];
				
				startIndexY = endPtsHistogramY[startIndexY];
				endIndexY = endPtsHistogramY[endIndexY];
				const PxU32 sRegionY = startIndexY / elementsPerRegions;
				const PxU32 eRegionY = endIndexY / elementsPerRegions;
	
				//calculate region index in z axis
				const PxgSapBox1D& sapZ = sapBoxZ[handle];
				PxU32 startIndexZ = sapZ.mMinMax[0];
				PxU32 endIndexZ = sapZ.mMinMax[1];
				startIndexZ = endPtsHistogramZ[startIndexZ];
				endIndexZ = endPtsHistogramZ[endIndexZ];
				const PxU32 sRegionZ = startIndexZ / elementsPerRegions;
				const PxU32 eRegionZ = endIndexZ / elementsPerRegions;

				regionRange[handle].minRange = make_uint4(sRegionX, sRegionY, sRegionZ, 0);
				regionRange[handle].maxRange = make_uint4(eRegionX, eRegionY, eRegionZ, 0);

				const PxU32 yzRegionAccum = (eRegionY-sRegionY+1) *(eRegionZ - sRegionZ + 1);
				startRegionAccum[startIndex]  = yzRegionAccum;
				startRegionAccum[endIndex] = 0;

				if (eRegionX >= sRegionX)
					regionCount = (eRegionX - sRegionX + 1) * yzRegionAccum;
			}
			regionAccum[workIndex] = regionCount;
		}
	}  
}

template <PxU32 WARP_PERBLOCK_SIZE>
static __device__ void computeHistogram(PxU32* histogram, PxU32* blockHistogram, const PxU32 maxWorkIndex)
{
	__shared__ PxU32 sHistogram[WARP_SIZE * WARP_PERBLOCK_SIZE];

	__shared__ PxU32 sWarpAccumulator[WARP_PERBLOCK_SIZE];

	__shared__ PxU32 sBlockAccumulator;

	const PxU32 nbBlocksRequired = (maxWorkIndex + blockDim.x-1)/blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x-1)/gridDim.x;

	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE-1);
	const PxU32 warpIndex = threadIdx.x/(WARP_SIZE);
	const PxU32 idx = threadIdx.x;

	if(threadIdx.x == 0)
		sBlockAccumulator = 0;

	__syncthreads();

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i*WARP_SIZE*WARP_PERBLOCK_SIZE + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		PxU32 histVal = 0; 
		if(workIndex < maxWorkIndex)
			histVal = histogram[workIndex];

		PxU32 accumVal = warpScanAdd<WARP_SIZE>(FULL_MASK, idx, threadIndexInWarp, sHistogram, histVal, histVal);

		if(threadIndexInWarp == (WARP_SIZE-1))
			sWarpAccumulator[warpIndex] = accumVal + histVal;

		const PxU32 prevBlockAccumulator = sBlockAccumulator;

		__syncthreads();
		
		unsigned mask_idx = __ballot_sync(FULL_MASK, idx < WARP_PERBLOCK_SIZE);
		if(idx < WARP_PERBLOCK_SIZE)
		{
			PxU32 value = sWarpAccumulator[threadIndexInWarp];

			const PxU32 output = warpScanAddWriteToSharedMem<WARP_PERBLOCK_SIZE>(mask_idx, idx, threadIndexInWarp, sWarpAccumulator, value, value);
			if(threadIndexInWarp == (WARP_PERBLOCK_SIZE-1))
				sBlockAccumulator += (output + value);
		}
		
		__syncthreads();

		if(workIndex < maxWorkIndex)
		{
			//Now output both histograms...
			histogram[workIndex] = accumVal + prevBlockAccumulator + sWarpAccumulator[warpIndex];
		}
	}

	if(threadIdx.x == 0)
		blockHistogram[blockIdx.x] = sBlockAccumulator;
}

template <PxU32 WARP_PERBLOCK_SIZE>
static __device__ void outputHistogram(PxU32* histogram, const PxU32* blockHistogram, PxU32& totalAccumulation, const PxU32 maxWorkIndex)
{
	__shared__ PxU32 sBlockAccum[BLOCK_SIZE];

	const PxU32 idx = threadIdx.x;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x*blockIdx.x;

	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE-1);  

	PxU32 val = 0;
	PxU32 res = 0;
	unsigned mask_idx = __ballot_sync(FULL_MASK, idx < BLOCK_SIZE);
	if(idx < BLOCK_SIZE)
	{
		val = blockHistogram[idx];
		res = warpScanAddWriteToSharedMem<BLOCK_SIZE>(mask_idx, idx, threadIndexInWarp, sBlockAccum, val, val);
	}

	if(globalThreadIndex == (BLOCK_SIZE-1))
		totalAccumulation = res + val;

	const PxU32 totalBlockRequired = (maxWorkIndex + (blockDim.x-1))/ blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (BLOCK_SIZE-1))/ BLOCK_SIZE;

	__syncthreads();
	
	PxU32 blockAccum = sBlockAccum[blockIdx.x];

	for(PxU32 i=0; i<numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i*WARP_SIZE*WARP_PERBLOCK_SIZE + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		if(workIndex < maxWorkIndex)
			histogram[workIndex] = histogram[workIndex] + blockAccum;
	}
}

extern "C" __global__ void computeStartRegionsHistogram(const PxgBroadPhaseDesc* bpDesc)	// BP_COMPUTE_START_REGION_HISTOGRAM //###ONESHOT
{
	PxU32* startRegionAccum = bpDesc->startRegionAccum;
	PxU32* blockStartRegionAccum = bpDesc->blockStartRegionAccum;

	const PxU32 numHandles = bpDesc->numPreviousHandles + bpDesc->numCreatedHandles - bpDesc->numRemovedHandles;
	//const PxU32 numHandles = bpDesc->numHandles;
	const PxU32 nbProjections = numHandles*2;

	computeHistogram<PxgBPKernelBlockDim::BP_COMPUTE_START_REGION_HISTOGRAM/WARP_SIZE>(startRegionAccum, blockStartRegionAccum, nbProjections);
}

extern "C" __global__ void outputStartRegionsHistogram(PxgBroadPhaseDesc* bpDesc)	// BP_OUTPUT_START_REGION_HISTOGRAM //###ONESHOT
{
	PxU32* startRegionAccum = bpDesc->startRegionAccum;
	const PxU32* blockStartRegionAccum = bpDesc->blockStartRegionAccum;
	//const PxU32 nbProjections = bpDesc->numHandles * 2;
	const PxU32 numHandles = bpDesc->numPreviousHandles + bpDesc->numCreatedHandles - bpDesc->numRemovedHandles;
	const PxU32 nbProjections = numHandles * 2;

	outputHistogram<PxgBPKernelBlockDim::BP_OUTPUT_START_REGION_HISTOGRAM/WARP_SIZE>(startRegionAccum, blockStartRegionAccum, bpDesc->startRegionAccumTotal, nbProjections);
}

extern "C" __global__ void computeRegionsHistogram(const PxgBroadPhaseDesc* bpDesc)	// BP_COMPUTE_REGION_HISTOGRAM //###ONESHOT
{
	PxU32* regionAccum = bpDesc->regionAccum;
	PxU32* blockRegionAccum = bpDesc->blockRegionAccum;

	const PxU32 numHandles = bpDesc->numPreviousHandles + bpDesc->numCreatedHandles - bpDesc->numRemovedHandles;
	const PxU32 nbProjections = numHandles * 2;
	
	computeHistogram<PxgBPKernelBlockDim::BP_COMPUTE_REGION_HISTOGRAM/WARP_SIZE>(regionAccum, blockRegionAccum, nbProjections);
}

extern "C" __global__ void outputRegionsHistogram(PxgBroadPhaseDesc* bpDesc)	// BP_OUTPUT_REGION_HISTOGRAM //###ONESHOT
{
	PxU32* regionAccum = bpDesc->regionAccum;
	const PxU32* blockRegionAccum = bpDesc->blockRegionAccum;

	const PxU32 numHandles = bpDesc->numPreviousHandles + bpDesc->numCreatedHandles - bpDesc->numRemovedHandles;
	const PxU32 nbProjections = numHandles * 2;

	outputHistogram<PxgBPKernelBlockDim::BP_OUTPUT_REGION_HISTOGRAM/WARP_SIZE>(regionAccum, blockRegionAccum, bpDesc->regionAccumTotal, nbProjections);
}

extern "C" __global__ void writeOutStartAndActiveRegionHistogram(const PxgBroadPhaseDesc* bpDesc)	// BP_WRITEOUT_ACTIVE_HISTOGRAM //###ONESHOT
{
	const PxU32* startRegionAccum = bpDesc->startRegionAccum;

	const PxgIntegerRegion* regionRange = bpDesc->regionRange;

	const PxU32 numHandles = bpDesc->numPreviousHandles + bpDesc->numCreatedHandles - bpDesc->numRemovedHandles;
	const PxU32 nbProjections = numHandles * 2;
	//const PxU32 totalNbProjections = (nbProjections+3)&(~3);

	const PxU32 totalComparision = bpDesc->startRegionAccumTotal;

	const PxU32 nbBlocksRequired = (totalComparision + blockDim.x-1)/blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x-1)/gridDim.x;

	const PxU32* sortedHandles = bpDesc->boxHandles[0][AXIS_X];//using current frame x axis handles

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i*PxgBPKernelBlockDim::BP_WRITEOUT_ACTIVE_HISTOGRAM + threadIdx.x + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if(workIndex < totalComparision)
		{
			const PxU32 pos = binarySearch(startRegionAccum, nbProjections, workIndex);

			const PxU32 sortedHandle = sortedHandles[pos];

			const PxU32 handle = getHandle(sortedHandle);

			const PxU32 offset = workIndex - startRegionAccum[pos];

			//work out the region index
			const PxgIntegerRegion& range = regionRange[handle];
			const uint4 minRange = range.minRange;
			const uint4 maxRange = range.maxRange;

			const PxU32 sRegionX = minRange.x;
			const PxU32 sRegionY = minRange.y;
			const PxU32 sRegionZ = minRange.z;
			const PxU32 eRegionZ = maxRange.z;
			
			const PxU32 nbZRegions = (eRegionZ - sRegionZ) + 1;
			const PxU32 zRegion = (offset % nbZRegions) + sRegionZ;
			const PxU32 yRegion = (offset / nbZRegions) + sRegionY;

			const PxU32 regionIndex = sRegionX + yRegion*REGION_SIZE_PER_AXIS+ zRegion*(REGION_SIZE_PER_AXIS*REGION_SIZE_PER_AXIS);

			const PxU32 regionStartOffset = regionIndex * nbProjections ;

			PxU32* activeRegionsHistogram = &bpDesc->activeRegionsHistogram[regionStartOffset];
			PxU32* startRegionsHistogram = &bpDesc->startRegionsHistogram[regionStartOffset];

			startRegionsHistogram[pos] = 1;

			activeRegionsHistogram[pos] = isNewProjection(sortedHandle);
		}
	}
}

extern "C" __global__ void computeStartAndActiveRegionHistogram(const PxgBroadPhaseDesc* bpDesc)	// BP_COMPUTE_ACTIVE_HISTOGRAM //###ONESHOT
{
	//the compiler should be able to strip this out
	__shared__ PxU32 sStartHistogram[WARP_SIZE * WARP_PERBLOCK_SIZE_16];

	__shared__ PxU32 sStartWarpAccumulator[WARP_PERBLOCK_SIZE_16];

	__shared__ PxU32 sActiveBlockAccumulator;
	__shared__ PxU32 sStartBlockAccumulator;

	const PxU32 numHandles = bpDesc->numPreviousHandles + bpDesc->numCreatedHandles - bpDesc->numRemovedHandles;
	const PxU32 nbProjections = numHandles * 2 ;
	//const PxU32 totalNbProjections = (nbProjections+3)&(~3);

	const PxU32 nbIterationsPerBlock = (nbProjections + blockDim.x-1)/blockDim.x;

	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE-1);
	const PxU32 warpIndex = threadIdx.x/(WARP_SIZE);
	const PxU32 idx = threadIdx.x;

	for(PxU32 a = blockIdx.x; a < TOTAL_REGION_SIZE; a+=gridDim.x)
	{
		if(threadIdx.x == 0)
		{
			sActiveBlockAccumulator = 0;
			sStartBlockAccumulator = 0;
		}

		__syncthreads();

		const PxU32 regionStartOffset = a * nbProjections;

		PxU32* activeRegionsHistogram = &bpDesc->activeRegionsHistogram[regionStartOffset];
		PxU32* startRegionsHistogram = &bpDesc->startRegionsHistogram[regionStartOffset];
		   
		for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
		{
			const PxU32 workIndex = i*WARP_SIZE*WARP_PERBLOCK_SIZE_16 + idx /*+ nbIterationsPerBlock * blockIdx.x * blockDim.x*/;
			
			PxU32 activeHistVal = 0;
			PxU32 startHistVal = 0;
			if(workIndex < nbProjections)
			{
				activeHistVal = activeRegionsHistogram[workIndex];
				startHistVal = startRegionsHistogram[workIndex];
			}
			const PxU32 combinedVal = (startHistVal<<16) | activeHistVal;

			const PxU32 startAccumVal = warpScanAdd<WARP_SIZE>(FULL_MASK, idx, threadIndexInWarp, sStartHistogram, combinedVal, combinedVal);

			if(threadIndexInWarp == (WARP_SIZE-1))
				sStartWarpAccumulator[warpIndex] = startAccumVal + combinedVal;

			const PxU32 prevActiveBlockAccumulator = sActiveBlockAccumulator;
			const PxU32 prevStartBlockAccumulator = sStartBlockAccumulator;

			__syncthreads();

			if(warpIndex == 0)
			{
				unsigned mask_threadIndexInWarp = __ballot_sync(FULL_MASK, threadIndexInWarp < WARP_PERBLOCK_SIZE_16);
				if(threadIndexInWarp < WARP_PERBLOCK_SIZE_16)
				{
					PxU32 value = sStartWarpAccumulator[threadIndexInWarp];

					const PxU32 output = warpScanAddWriteToSharedMem<WARP_PERBLOCK_SIZE_16>(mask_threadIndexInWarp, threadIndexInWarp, threadIndexInWarp, sStartWarpAccumulator, value, value);
					
					if(threadIndexInWarp == (WARP_PERBLOCK_SIZE_16-1))
					{
						const PxU32 res = output + value;
						sStartBlockAccumulator += (res >> 16);
						sActiveBlockAccumulator += (res & 0xFFFF);
					}
				}
			}

			__syncthreads();
			
			//out put OrderedActiveRegionHistogram
			if(workIndex < nbProjections)
			{
				//Now output both histograms...
				const PxU32 warpAccum =  startAccumVal +  sStartWarpAccumulator[warpIndex];
				const PxU32 activeAccum = warpAccum & 0xFFFF;
				const PxU32 startAccum = warpAccum >> 16;

				const PxU32 activeIndex = activeAccum + prevActiveBlockAccumulator;
				const PxU32 startIndex = startAccum + prevStartBlockAccumulator;

				activeRegionsHistogram[workIndex] = activeIndex;
				startRegionsHistogram[workIndex] = startIndex;  
			}
		}
	}
}

extern "C" __global__ void outputOrderedActiveRegionHistogram(const PxgBroadPhaseDesc* bpDesc)	// BP_OUTPUT_ACTIVE_HISTOGRAM //###ONESHOT
{
	const PxU32* startRegionAccum = bpDesc->startRegionAccum;

	const PxgIntegerRegion* regionRange = bpDesc->regionRange;

	const PxU32 numHandles = bpDesc->numPreviousHandles + bpDesc->numCreatedHandles - bpDesc->numRemovedHandles;
	const PxU32 nbProjections = numHandles * 2 ;
	//const PxU32 totalNbProjections = (nbProjections+3)&(~3);

	const PxU32 totalComparision = bpDesc->startRegionAccumTotal;

	const PxU32 nbBlocksRequired = (totalComparision + blockDim.x-1)/blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x-1)/gridDim.x;

	const PxU32* sortedHandles = bpDesc->boxHandles[0][AXIS_X]; //using current frame x axis handles
	const PxgSapBox1D* sapBoxes = bpDesc->boxSapBox1D[AXIS_X];

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i*PxgBPKernelBlockDim::BP_OUTPUT_ACTIVE_HISTOGRAM + threadIdx.x + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if(workIndex < totalComparision)
		{
			const PxU32 pos = binarySearch(startRegionAccum, nbProjections, workIndex);

			const PxU32 sortedHandle = sortedHandles[pos];

			const PxU32 handle = getHandle(sortedHandle);

			const PxU32 offset = workIndex - startRegionAccum[pos];

			//work out the region index
			const PxgIntegerRegion& range = regionRange[handle];
			const uint4 minRange = range.minRange;
			const uint4 maxRange = range.maxRange;

			const PxU32 sRegionX = minRange.x;
			const PxU32 sRegionY = minRange.y;
			const PxU32 sRegionZ = minRange.z;
			const PxU32 eRegionZ = maxRange.z;
			
			const PxU32 nbZRegions = (eRegionZ - sRegionZ) + 1;
			const PxU32 zRegion = (offset % nbZRegions) + sRegionZ;
			const PxU32 yRegion = (offset / nbZRegions) + sRegionY;

			const PxU32 regionIndex = sRegionX + yRegion*REGION_SIZE_PER_AXIS + zRegion*(REGION_SIZE_PER_AXIS*REGION_SIZE_PER_AXIS);

			const PxU32 regionStartOffset = regionIndex * nbProjections;

			const PxU32* startRegionsHistogram = &bpDesc->startRegionsHistogram[regionStartOffset];
			PxU32* orderedStartHandles = &bpDesc->orderedStartRegionHandles[regionStartOffset];

			const PxgSapBox1D& sapBox = sapBoxes[handle];
					
			const PxU32 startProjIndex = sapBox.mMinMax[0];
			const PxU32 startIndex = startRegionsHistogram[startProjIndex]; 
		
			if (eRegionZ >= sRegionZ)
			{
				//Output to orderedStartHandles
				orderedStartHandles[startIndex] = handle;
				if (isNewProjection(sortedHandle))
				{
					const PxU32* activeRegionsHistogram = &bpDesc->activeRegionsHistogram[regionStartOffset];
					PxU32* orderedActiveHandles = &bpDesc->orderedActiveRegionHandles[regionStartOffset];
					const PxU32 activeIndex = activeRegionsHistogram[startProjIndex];
					orderedActiveHandles[activeIndex] = handle;
				}
			}
		}
	}
}

extern "C" __global__ void writeOutOverlapChecksForInsertedBoundsRegionsHistogram(const PxgBroadPhaseDesc* bpDesc)	// BP_WRITEOUT_OVERLAPCHECKS_HISTOGRAM_NEWBOUNDS //###ONESHOT
{
	const PxU32 numHandles = bpDesc->numPreviousHandles + bpDesc->numCreatedHandles - bpDesc->numRemovedHandles;
	const PxU32 nbProjections = numHandles * 2 ;
	
	const PxgSapBox1D* boxSapBox1DX = bpDesc->boxSapBox1D[AXIS_X];

	regionOverlapType* overlapChecks = bpDesc->overlapChecksRegion;

	PxgHandleRegion* handleRegiones = bpDesc->overlapChecksHandleRegiones;

	const PxgIntegerRegion* regionRange = bpDesc->regionRange;

	const PxU32* regionAccum = bpDesc->regionAccum;

	const PxU32* handles = bpDesc->boxHandles[0][AXIS_X]; //using current frame x axis handles

	const PxU32 idx = threadIdx.x;

	const PxU32 totalComparision = bpDesc->regionAccumTotal;

	const PxU32 totalBlockRequired = (totalComparision + (blockDim.x-1))/ blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (BLOCK_SIZE-1))/ BLOCK_SIZE;

	for(PxU32 i=0; i<numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i*PxgBPKernelBlockDim::BP_WRITEOUT_OVERLAPCHECKS_HISTOGRAM_NEWBOUNDS + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		if(workIndex < totalComparision)
		{
			const PxU32 boxStartIndexX = binarySearch(regionAccum, nbProjections, workIndex);

			const PxU32 offset = workIndex - regionAccum[boxStartIndexX];
			const PxU32 sortedHandle = handles[boxStartIndexX];
			const PxU32 handle = getHandle(sortedHandle);

			const bool isNew = isNewProjection(sortedHandle);

			//TODO - can we remove this access somehow?
			const PxgSapBox1D& sapBoxX = boxSapBox1DX[handle];
			//PxU32 boxStartIndexX = sapBoxX.mMinMax[0];
			const PxU32 boxEndIndexX = sapBoxX.mMinMax[1];

			//work out the region index
			const PxgIntegerRegion& range = regionRange[handle];
			const uint4 minRange = range.minRange;
			const uint4 maxRange = range.maxRange;

			const PxU32 sRegionX = minRange.x;
			const PxU32 sRegionY = minRange.y;
			const PxU32 sRegionZ = minRange.z;
			const PxU32 eRegionY = maxRange.y;
			const PxU32 eRegionZ = maxRange.z;
			
			const PxU32 nbZRegions = (eRegionZ - sRegionZ) + 1;
			const PxU32 nbYRegions = (eRegionY - sRegionY) + 1;
			const PxU32 zRegion = (offset % nbZRegions) + sRegionZ;
			const PxU32 yRegion = ((offset / nbZRegions) % nbYRegions) + sRegionY;
			const PxU32 xRegion = (offset / (nbZRegions * nbYRegions)) + sRegionX;

			const PxU32 regionIndex =(xRegion + yRegion*REGION_SIZE_PER_AXIS+ zRegion*(REGION_SIZE_PER_AXIS*REGION_SIZE_PER_AXIS)) * nbProjections;

			const PxU32* startEndPtHistogram = isNew ? (&bpDesc->startRegionsHistogram[regionIndex]) : (&bpDesc->activeRegionsHistogram[regionIndex]);
			
			const PxU32 regionStartIndex = startEndPtHistogram[boxStartIndexX +1];
			const PxU32 regionEndIndex = startEndPtHistogram[boxEndIndexX];

			handleRegiones[workIndex].handleIndex = handle;
			handleRegiones[workIndex].regionIndex = regionIndex;
			overlapChecks[workIndex] = regionEndIndex - regionStartIndex;
			//CHECK64(overlapChecks[workIndex])	// PT: this one doesn't fire in this kernel
		}
	}
}

// PT: we use defines instead of typedefs to be able to use the same names but with different types
// in each kernel. At time of writing ltype is used where the UTs proved that 64bit was necessary,
// and ltype2 is used when the jury is still out - the UT does not fail with a 32bit type there but
// from looking at the code it seems that it should still be 64bit.
#define ltype	PxU64
#define ltype2	PxU64
extern "C" __global__ void computeOverlapChecksForRegionsHistogram(const PxgBroadPhaseDesc* bpDesc)	// BP_COMPUTE_OVERLAPCHECKS_HISTOGRAM //###ONESHOT
{
	//if the hardware support shfl, the compiler should be able to strip out this memory
	__shared__ ltype2 sPairCount[PxgBPKernelBlockDim::BP_COMPUTE_OVERLAPCHECKS_HISTOGRAM];

	const PxU32 WARP_PERBLOCK_SIZE = PxgBPKernelBlockDim::BP_COMPUTE_OVERLAPCHECKS_HISTOGRAM / WARP_SIZE;

	// PT: there is no evidence that sPairWarpCount must be 64bit but it is involved in 64bit accumulations
	__shared__ ltype2 sPairWarpCount[WARP_PERBLOCK_SIZE];

	regionOverlapType* blockOverlapChecks = bpDesc->blockOverlapChecksRegion;	// PT: must be 64bit because of overflow below (1)

	regionOverlapType* overlapChecks = bpDesc->overlapChecksRegion;	// PT: must be 64bit because of overflow below (2)

	const PxU32 idx = threadIdx.x;

	const PxU32 warpIndexInBlock = threadIdx.x/WARP_SIZE;

	//This identifies which thread within a warp a specific thread is
	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE-1);

	const PxU32 totalComparision = bpDesc->regionAccumTotal;

	const PxU32 totalBlockRequired = (totalComparision + (blockDim.x-1))/ blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (BLOCK_SIZE-1))/ BLOCK_SIZE;

	__shared__ ltype accum;	// PT: must be 64bit because of overflow below (1)

	if(idx == (WARP_PERBLOCK_SIZE - 1))
		accum = 0;

	for(PxU32 i=0; i<numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i*WARP_SIZE*WARP_PERBLOCK_SIZE + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		// PT: there is no evidence that nbComparision must be 64bit but it reads from a 64bit buffer so....
		ltype2 nbComparision = 0;
		if(workIndex < totalComparision)
			nbComparision = overlapChecks[workIndex];
		//CHECK64(nbComparision)	// PT: this one doesn't fire

		// PT: there is no evidence that pairCount must be 64bit but it is involved in 64bit accumulations
		const ltype2 pairCount = warpScanAdd<WARP_SIZE>(FULL_MASK, idx, threadIndexInWarp, sPairCount, nbComparision, nbComparision);
		//CHECK64(pairCount)	// PT: this one doesn't fire

		if(threadIndexInWarp == (WARP_SIZE-1))
		{
			sPairWarpCount[warpIndexInBlock] = pairCount + nbComparision;
			//CHECK64(sPairWarpCount[warpIndexInBlock])	// PT: this one doesn't fire
		}
	
		__syncthreads();

		// PT: there is no evidence that value and res must be 64bit but they are involved in 64bit accumulations
		ltype2 value = 0;
		ltype2 res = 0;
		unsigned mask_idx = __ballot_sync(FULL_MASK, idx < WARP_PERBLOCK_SIZE);
		if(idx  < WARP_PERBLOCK_SIZE)
		{
			value = sPairWarpCount[idx];
			//CHECK64(value)	// PT: this one doesn't fire

			res = warpScanAddWriteToSharedMem<WARP_PERBLOCK_SIZE>(mask_idx, idx, threadIndexInWarp, sPairWarpCount, value, value);
			//CHECK64(sPairWarpCount[idx])	// PT: this one doesn't fire
			//CHECK64(res)					// PT: this one doesn't fire
		}

		ltype prevAccum = accum;	// PT: must be 64bit because accum is 64bit (1)

		__syncthreads();

		if(workIndex < totalComparision)
		{
			overlapChecks[workIndex] = pairCount + sPairWarpCount[warpIndexInBlock] + prevAccum;
			//CHECK64(prevAccum)						// PT: this one fires
			//CHECK64(overlapChecks[workIndex])			// PT: this one fires so overlapChecks must be 64bit (2)
			//CHECK64(sPairWarpCount[warpIndexInBlock])	// PT: this one doesn't fire
		}

		__syncthreads();

		if(idx ==  (WARP_PERBLOCK_SIZE - 1))
			accum += (res + value);
	}

	if(idx ==  (WARP_PERBLOCK_SIZE - 1))
	{
		//write out the global start index address
		blockOverlapChecks[blockIdx.x] = accum;
		//CHECK64(accum)	// PT: this one fires so blockOverlapChecks and accum must be 64bit (1)
	}
}
#undef ltype
#undef ltype2

#define ltype	PxU64
extern "C" __global__ void outputOverlapChecksForRegionHistogram(PxgBroadPhaseDesc* bpDesc)	// BP_OUTPUT_OVERLAPCHECKS_HISTOGRAM //###ONESHOT
{
	// PT: must be 64bit because of overflow below (3)
	__shared__ ltype sAxisAccum[BLOCK_SIZE];

	const PxU32 idx = threadIdx.x;

	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE-1);  

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x*blockIdx.x;

	const regionOverlapType* blockOverlapChecks = bpDesc->blockOverlapChecksRegion;

	//exclusive accumulation 
	regionOverlapType* overlapChecks = bpDesc->overlapChecksRegion;

	ltype res = 0;	// PT: must be 64bit because of overflow below (1)
	ltype val = 0;	// PT: must be 64bit because of overflow below (2)
	unsigned mask_idx = __ballot_sync(FULL_MASK, idx < BLOCK_SIZE);
	if(idx < BLOCK_SIZE)
	{
		val = blockOverlapChecks[idx];
		//CHECK64(val)	// PT: this one fires so val must be 64bit (2)

		res = warpScanAddWriteToSharedMem<BLOCK_SIZE>(mask_idx, idx, threadIndexInWarp, sAxisAccum, val, val);
		//CHECK64(res)				// PT: this one fires so res must be 64bit (1)
		//CHECK64(sAxisAccum[idx])	// PT: this one fires so sAxisAccum must be 64bit (3)
	}

	if(globalThreadIndex == (BLOCK_SIZE-1))
	{
		bpDesc->overlapChecksTotalRegion = res + val;
		//CHECK64(bpDesc->overlapChecksTotalRegion)	// PT: this one fires so overlapChecksTotalRegion must be 64bit
	}

	const PxU32 totalComparision = bpDesc->regionAccumTotal;

	const PxU32 totalBlockRequired = (totalComparision + (blockDim.x-1))/ blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (BLOCK_SIZE-1))/ BLOCK_SIZE;

	__syncthreads();

	const ltype blockAccum = sAxisAccum[blockIdx.x];

	for(PxU32 i=0; i<numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i*PxgBPKernelBlockDim::BP_OUTPUT_OVERLAPCHECKS_HISTOGRAM + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		if(workIndex < totalComparision)
			overlapChecks[workIndex] = overlapChecks[workIndex] + blockAccum;
	}
}
#undef ltype

static __device__ bool isSeparatedBeforeAxis(const PxU32 axis, const PxgIntegerAABB& aabb0, const PxgIntegerAABB& aabb1)
{
	switch(axis)
	{
	case 2:
		if(!aabb0.intersects1D(aabb1, 1))
			return true;
		//fallthrough
	case 1:
		if(!aabb0.intersects1D(aabb1, 0))
			return true;
		//fallthrough
	default:
		break;
	}
	return false;
}

static __device__ PX_FORCE_INLINE void updatePair(PxU32 handle, PxU32 otherHandle, PxgBroadPhasePair* report, PxU32 index)
{
	//printf("*******BP Found/Lost %i, %i\n", handle, otherHandle);
	report[index] = PxgBroadPhasePair(handle, otherHandle);
}

extern "C" __global__ void performIncrementalSAP(PxgBroadPhaseDesc* bpDesc)	// BP_INCREMENTAL_SAP
{
	const PxU32 WARP_PERBLOCK_SIZE = PxgBPKernelBlockDim::BP_INCREMENTAL_SAP / WARP_SIZE;
	__shared__ PxU32 sTotalComparisons;
	__shared__ const PxU32* sComparisonHistograms;
	__shared__ const PxU32* sRanks;
	__shared__ PxgBroadPhasePair* sFoundOrLost[2];
	__shared__ const PxU32* sSortedHandles;
	__shared__ const PxgIntegerAABB* sAabbs[2];

	//0 = created pairs, 1 = destroyed pairs
	__shared__ PxU32 sFoundLostAccumulator[2][PxgBPKernelBlockDim::BP_INCREMENTAL_SAP];
	__shared__ PxU32 sFoundAccum[2][WARP_PERBLOCK_SIZE];

	__shared__ PxU32 sBaseWriteIndex[2];

	__shared__ PxU32* sSharedPointers[2];

	__shared__ const PxU32* sStartEndAccum[2][2];
	__shared__ const PxU32* sStartEndHandles[2][2];

	//const PxU32 numHandles = bpDesc->numPreviousHandles + bpDesc->numCreatedHandles - bpDesc->numRemovedHandles;
	const PxU32 numHandles = bpDesc->numPreviousHandles;
	const PxU32 nbProjections = numHandles * 2 ;

	const PxU32 max_found_lost_pair = bpDesc->max_found_lost_pairs;

	if(threadIdx.x == 0)
	{
		sTotalComparisons = bpDesc->totalIncrementalComparisons[blockIdx.y];
		sComparisonHistograms = bpDesc->incrementalComparisons[blockIdx.y];
		sRanks = bpDesc->boxProjectionRanks[blockIdx.y];
		//sRanks[threadIdx.x] = bpDesc->boxProjectionRanks[threadIdx.x];
		//sSortedHandles[threadIdx.x][0] = bpDesc->boxTempHandles[threadIdx.x];
		//sSortedHandles[threadIdx.x] = bpDesc->boxHandles[0][threadIdx.x]; 
		sSortedHandles = bpDesc->boxHandles[0][blockIdx.y];

		sStartEndAccum[0][0] = bpDesc->startPtHistogram[0][blockIdx.y];//this frame's start projections
		sStartEndAccum[0][1] = bpDesc->startPtHistogram[1][blockIdx.y];//last frame's start projections
		sStartEndAccum[1][0] = bpDesc->endPtHistogram[0][blockIdx.y]; //this frame's end projections
		sStartEndAccum[1][1] = bpDesc->endPtHistogram[1][blockIdx.y]; //Last frame's end projections

		sStartEndHandles[0][0] = bpDesc->startPointHandles[0][blockIdx.y];//this frame's start projections
		sStartEndHandles[0][1] = bpDesc->startPointHandles[1][blockIdx.y];//last frame's start projections
		sStartEndHandles[1][0] = bpDesc->endPointHandles[0][blockIdx.y]; //this frame's end projections
		sStartEndHandles[1][1] = bpDesc->endPointHandles[1][blockIdx.y]; //Last frame's end projections
	
		sFoundOrLost[0] = bpDesc->foundPairReport;
		sFoundOrLost[1] = bpDesc->lostPairReport;
		sAabbs[0] = bpDesc->newIntegerBounds;
		sAabbs[1] = bpDesc->oldIntegerBounds;
		sSharedPointers[0] = &bpDesc->sharedFoundPairIndex;
		sSharedPointers[1] = &bpDesc->sharedLostPairIndex;
	}

	const PxU32* groupIds = bpDesc->updateData_groups;
#if USE_ENV_IDS
	const PxU32* envIds = bpDesc->updateData_envIDs;
#endif
	__syncthreads();
	//Loop over all 3 axes, do incremental swaps, produce results...

	PxU32 nbBlocksProcessed = 0;
	PxU32 blockId = blockIdx.x;

	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE-1);
	const PxU32 warpIndex = threadIdx.x/WARP_SIZE;

	PxU32 axis = blockIdx.y;
	{
		//This is the number of iterations per-grid
		const PxU32 totalCmpsThisAxis = sTotalComparisons;
		const PxU32 nbBlocksRequired = (totalCmpsThisAxis + blockDim.x-1)/blockDim.x;
		const PxU32 blockStartIndex = nbBlocksProcessed;
		const PxU32 blockEndIndex = blockStartIndex + nbBlocksRequired;
		const PxU32* histogram = sComparisonHistograms;

		const PxU32* boxRanks = sRanks;

		for(; blockId < blockEndIndex; blockId += gridDim.x)
		{
			//OK. We need to process this block of work...
			//Work out the threadIdx to be able to process the pair...

			const PxU32 workId = (blockId-blockStartIndex) * (blockDim.x) + threadIdx.x;

			PxU32 handle = 0xFFFFFFFF;
			PxU32 otherHandle = 0xFFFFFFFF;
			PxU32 foundOrLostID = 0;
			PxU32 foundOrLostPair = 0;
			PxU32 downPass = 0;
			if(workId < totalCmpsThisAxis)
			{
				//Then we have something to do...
				//Stage 1 -> work out which projection we're processing...
				PxU32 index = binarySearch(histogram, nbProjections, workId);

				const PxU32 sortedHandle = sSortedHandles[index];

				const PxU32 isStartHandle = isStartProjection(sortedHandle);

				handle = getHandle(sortedHandle);

				const PxU32 groupId = groupIds[handle];
#if USE_ENV_IDS
				const PxU32 envId = envIds ? envIds[handle] : PX_INVALID_U32;
#endif
				//Now figure out how far offset we are...

				const PxU32 offset = workId - histogram[index];

				//Now figure out which swaps etc. we are processing...

				const PxU32 prevIndex = boxRanks[index];

				downPass = prevIndex > index;

				const PxU32 startIndex = downPass ? index : prevIndex+1;		

				const PxU32 willFindPair = (!isStartHandle) ^ downPass;

				//PxU32 processIndex = startIndex + offset;

				//Up sweeps sweep against the previous sorted handle list. Down-sweeps sweep against the new sorted handle list. This avoids missing any pairs
				//const PxU32 otherSortedHandle = sortedHandles[axis][1-downPass][processIndex];

				//otherHandle = getHandle(otherSortedHandle);		

				const PxU32* startEndAccum = sStartEndAccum[isStartHandle][downPass];
				const PxU32* startEndHandles = sStartEndHandles[isStartHandle][downPass];

				const PxU32 handleIndex = startEndAccum[startIndex] + offset;
				otherHandle = startEndHandles[handleIndex];

				//Perform the swap...

				const PxU32 otherGroupId = groupIds[otherHandle];
#if USE_ENV_IDS
				const PxU32 otherEnvId = envIds ? envIds[otherHandle] : PX_INVALID_U32;
				if(filtering(groupId, otherGroupId, envId, otherEnvId))
#else
				if((groupId != otherGroupId))// && (isStartHandle ^ isStartProjection(otherSortedHandle)))
#endif
				{
					//Then we need to do actual work...

					//As the logic for found/lost pairs is inversed based on whether we're doing an up or down-sweep, we must invert the new/old bounds to make sure that the
					//overlap logic functions correctly
					
					const PxgIntegerAABB* aabbs0 = sAabbs[1-willFindPair];
					const PxgIntegerAABB* aabbs1 = sAabbs[willFindPair];

					const PxgIntegerAABB& aabb = aabbs0[handle];
					const PxgIntegerAABB& oldAABB = aabbs1[handle];

					const PxgIntegerAABB& otherAABB = aabbs0[otherHandle];
					const PxgIntegerAABB& otherOldAABB = aabbs1[otherHandle];

					if(aabb.intersects(otherAABB) && !oldAABB.intersects1D(otherOldAABB, axis))
					{
						//If this is a down sweep, this is a found pair. If this is an up sweep, this is a lost pair
						if(!isSeparatedBeforeAxis(axis, oldAABB, otherOldAABB))
						{
							foundOrLostID = 1 - willFindPair;
							foundOrLostPair = 1;
						}
					}
				}
			}

			const PxU32 val0 = foundOrLostID == 0 ? foundOrLostPair : 0;
			const PxU32 val1 = foundOrLostID == 0 ? 0 : foundOrLostPair;

			const PxU32 res0 = warpScanAddWriteToSharedMem<WARP_SIZE>(FULL_MASK, threadIdx.x, threadIndexInWarp, sFoundLostAccumulator[0], val0, val0);
			const PxU32 res1 = warpScanAddWriteToSharedMem<WARP_SIZE>(FULL_MASK, threadIdx.x, threadIndexInWarp, sFoundLostAccumulator[1], val1, val1);

			if(threadIndexInWarp == (WARP_SIZE-1))
			{
				sFoundAccum[0][warpIndex] = res0 + val0;
				sFoundAccum[1][warpIndex] = res1 + val1;
			}
			 
			__syncthreads();

			const unsigned mask_warpIndex = __ballot_sync(FULL_MASK, warpIndex < 2 && threadIndexInWarp < WARP_PERBLOCK_SIZE);
			if(warpIndex < 2 && threadIndexInWarp < WARP_PERBLOCK_SIZE)
			{
				const PxU32 val = sFoundAccum[warpIndex][threadIndexInWarp];

				const PxU32 totalAccum = warpScanAddWriteToSharedMem<WARP_PERBLOCK_SIZE>(mask_warpIndex, threadIndexInWarp, threadIndexInWarp, sFoundAccum[warpIndex], val, val) + val;

				if(totalAccum > 0 && threadIndexInWarp == (WARP_PERBLOCK_SIZE-1))
				{
					//Atomic add and reserve space
					sBaseWriteIndex[warpIndex] = atomicAdd(sSharedPointers[warpIndex], totalAccum);
				}
			}

			__syncthreads();

			//Now write out found/lost pairs...

			if(foundOrLostPair)
			{
				const PxU32 writeIndex = sBaseWriteIndex[foundOrLostID] + sFoundLostAccumulator[foundOrLostID][threadIdx.x] + sFoundAccum[foundOrLostID][warpIndex];

				if (writeIndex < max_found_lost_pair)
					updatePair(handle, otherHandle, sFoundOrLost[foundOrLostID], writeIndex);
			}

			__syncthreads(); //Make sure all writes complete before we go round the loop again
		}

		nbBlocksProcessed += nbBlocksRequired;
	}
}

#define ltype	PxU64
#define ltype2	PxU32
extern "C" __global__ void generateFoundPairsForNewBoundsRegion(PxgBroadPhaseDesc* bpDesc)	// BP_GENERATE_FOUNDPAIR_NEWBOUNDS //###ONESHOT
{
	const PxU32 numHandles = bpDesc->numPreviousHandles + bpDesc->numCreatedHandles - bpDesc->numRemovedHandles;
	const PxU32 nbProjections = numHandles * 2 ;

	const PxgIntegerAABB* newBounds = bpDesc->newIntegerBounds;
	PxgBroadPhasePair* reports = bpDesc->foundPairReport;
	PxU32& shareReportIndex = bpDesc->sharedFoundPairIndex;

	const PxU32 WARP_PERBLOCK_SIZE = PxgBPKernelBlockDim::BP_GENERATE_FOUNDPAIR_NEWBOUNDS/WARP_SIZE;

	//if hardware support shfl, the compiler should be able to strip this memory out
	__shared__ ltype2 sFoundPairsCount[PxgBPKernelBlockDim::BP_GENERATE_FOUNDPAIR_NEWBOUNDS];

	__shared__ ltype2 sFoundPairsWarpCount[WARP_PERBLOCK_SIZE];
	__shared__ PxU32 sFoundStartIndex;

	__shared__ const PxU32* sStartEndPtHistograms[2];
	__shared__ const PxU32* sOrderedStartRegionHandles[2];

	if(threadIdx.x == 0)
	{
		sStartEndPtHistograms[0] = bpDesc->activeRegionsHistogram;
		sStartEndPtHistograms[1] = bpDesc->startRegionsHistogram;

		sOrderedStartRegionHandles[0] = bpDesc->orderedActiveRegionHandles;
		sOrderedStartRegionHandles[1] = bpDesc->orderedStartRegionHandles;
	}

	__syncthreads();

	const PxgIntegerRegion* regionRange = bpDesc->regionRange;
	
	const PxU32* boxGroups = bpDesc->updateData_groups;
#if USE_ENV_IDS
	const PxU32* boxEnvIDs = bpDesc->updateData_envIDs;
#endif
	const PxgSapBox1D* boxSapBox1DX = bpDesc->boxSapBox1D[AXIS_X];
	const PxU32* boxHandles = bpDesc->boxHandles[0][AXIS_X];

	const PxU32 warpIndexInBlock = threadIdx.x/WARP_SIZE;
	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE-1);

	const regionOverlapType* overlapChecks = bpDesc->overlapChecksRegion;
	const PxU32 overlCheckesSize = bpDesc->regionAccumTotal;

	const PxgHandleRegion* handleRegions = bpDesc->overlapChecksHandleRegiones;
	
	const ltype totalPair = bpDesc->overlapChecksTotalRegion;
	//CHECK64(totalPair)	// PT: this one fires so overlapChecksTotalRegion must be 64bit

	// PT: these ones don't fire in our repro but clearly they could depending on involved values, so we still use 64bit there.
	const ltype totalBlockRequired = (totalPair + (blockDim.x-1))/ blockDim.x;
	const ltype numIterationPerBlock = (totalBlockRequired + (gridDim.x-1))/ gridDim.x;
	//CHECK64(totalBlockRequired)
	//CHECK64(numIterationPerBlock)

	const PxU32 max_found_lost_pair = bpDesc->max_found_lost_pairs;	// PT: this is gpuDynamicsConfig.foundLostPairsCapacity
	//printf("max_found_lost_pair %d\n", max_found_lost_pair);

	for(ltype i=0; i<numIterationPerBlock; ++i)	// PT: using 64bit for the index because numIterationPerBlock is 64bit
	{	
		const ltype workIndex = i*blockDim.x + threadIdx.x + numIterationPerBlock * blockIdx.x * blockDim.x;
		//CHECK64(workIndex)	// PT: this one fires so workIndex must be 64bit

		//initialize found pair count buffer

		ltype2 foundCount = 0;

		PxU32 handle  =  0xFFFFFFFF;
		PxU32 otherHandle = 0xFFFFFFFF;

		if(workIndex < totalPair)
		{
			const PxU32 index = binarySearch(overlapChecks, overlCheckesSize, workIndex);

			handle = handleRegions[index].handleIndex;

			const PxgIntegerRegion& range = regionRange[handle];

			const PxU32 regionIndex = handleRegions[index].regionIndex;

			const PxgSapBox1D& sapBoxX = boxSapBox1DX[handle];
			const PxU32 boxStartIndexX = sapBoxX.mMinMax[0];
			const PxU32 boxEndIndexX = sapBoxX.mMinMax[1];

			const bool isNew = isNewProjection(boxHandles[boxStartIndexX]);

			const ltype offset = workIndex - overlapChecks[index];
			//CHECK64(offset)	// PT: this one doesn't fire

			const PxU32* startEndPtHistogramRegion = &sStartEndPtHistograms[isNew][regionIndex];
			const PxU32* orderedStartHandlesRegion = &sOrderedStartRegionHandles[isNew][regionIndex];
							
			const PxU32 regionStartIndex = startEndPtHistogramRegion[boxStartIndexX +1];
			const PxU32 regionEndIndex = startEndPtHistogramRegion[boxEndIndexX];
			
			if(regionEndIndex > regionStartIndex)
			{
				const ltype otherIndex = regionStartIndex + offset;
				//CHECK64(otherIndex)	// PT: this one doesn't fire

				if(otherIndex < regionEndIndex)
				{
					const PxU32 group = boxGroups[handle];
#if USE_ENV_IDS
					const PxU32 envID = boxEnvIDs ? boxEnvIDs[handle] : PX_INVALID_U32;
#endif
					otherHandle = orderedStartHandlesRegion[otherIndex];

					const uint4 minRange = range.minRange;
		
					const PxgIntegerRegion& otherRange = regionRange[otherHandle];
					const uint4 otherMinRange = otherRange.minRange;

					const PxU32 x = PxMax(minRange.x, otherMinRange.x);
					const PxU32 y = PxMax(minRange.y, otherMinRange.y);
					const PxU32 z = PxMax(minRange.z, otherMinRange.z);

					const PxU32 otherRegionIndex = (x + y*REGION_SIZE_PER_AXIS + z *(REGION_SIZE_PER_AXIS*REGION_SIZE_PER_AXIS)) * nbProjections;
					
					if(regionIndex == otherRegionIndex)
					{
						const PxU32 otherGroup = boxGroups[otherHandle];
#if USE_ENV_IDS
						const PxU32 otherEnvID = boxEnvIDs ? boxEnvIDs[otherHandle] : PX_INVALID_U32;
						if(filtering(group, otherGroup, envID, otherEnvID))
#else
						if(group != otherGroup)
#endif
						{
							const PxgIntegerAABB& iaabb = newBounds[handle];

							const PxgIntegerAABB& iotherAABB = newBounds[otherHandle]; 

							foundCount = iaabb.intersects(iotherAABB);
						}
					}
				}
			}  
		}
	
		ltype2 res = warpScanAdd<WARP_SIZE>(FULL_MASK, threadIdx.x, threadIndexInWarp, sFoundPairsCount, foundCount, foundCount);
		//CHECK64(res)	// PT: this one does not fire
		if(threadIndexInWarp == (WARP_SIZE-1))
		{
			sFoundPairsWarpCount[warpIndexInBlock] = res + foundCount;
			//CHECK64(sFoundPairsWarpCount[warpIndexInBlock])	// PT: this one does not fire
		}

		__syncthreads();

		ltype2 totalRes = 0;
		ltype2 value = 0;
		unsigned mask_threadIdx = __ballot_sync(FULL_MASK, threadIdx.x < WARP_PERBLOCK_SIZE);
		if(threadIdx.x < WARP_PERBLOCK_SIZE)
		{
			value = sFoundPairsWarpCount[threadIdx.x];

			totalRes = warpScanAddWriteToSharedMem<WARP_PERBLOCK_SIZE>(mask_threadIdx, threadIdx.x, threadIndexInWarp, sFoundPairsWarpCount, value, value);
			//CHECK64(totalRes)	// PT: this one does not fire
		}

		__syncthreads();

		const ltype2 totalOverlapPairs = totalRes + value;
		//CHECK64(totalOverlapPairs)	// PT: this one does not fire
		//write it out to global memory
		if(threadIdx.x == (WARP_PERBLOCK_SIZE-1) && totalOverlapPairs > 0)
			sFoundStartIndex = atomicAdd(&shareReportIndex, totalOverlapPairs);

		__syncthreads();

		if(foundCount)
		{
			const ltype2 foundOffset = res + sFoundPairsWarpCount[warpIndexInBlock];
			const ltype2 position = sFoundStartIndex + foundOffset;
			//CHECK64(foundOffset)	// PT: this one does not fire
			//CHECK64(position)	// PT: this one does not fire
			if (position < max_found_lost_pair)
				updatePair(handle, otherHandle, reports, position);
		}

		__syncthreads(); //Required because sFoundPairsWarpCount is read in next loop iteration
	}
}
#undef ltype
#undef ltype2

extern "C" __global__ void clearNewFlagLaunch(const PxgBroadPhaseDesc* bpDesc)	// BP_CLEAR_NEWFLAG //###ONESHOT
{
	const PxU32 numCreatedHandleSize = bpDesc->numCreatedHandles;
	
	const PxU32* createdHandles = bpDesc->updateData_createdHandles;
	//PxU32* updatedHandles = bpDesc->updatedHandles;
	
	const PxgSapBox1D* boxSapBox1DX = bpDesc->boxSapBox1D[AXIS_X];
	const PxgSapBox1D* boxSapBox1DY = bpDesc->boxSapBox1D[AXIS_Y];
	const PxgSapBox1D* boxSapBox1DZ = bpDesc->boxSapBox1D[AXIS_Z];

	PxU32* boxHandlesX = bpDesc->boxHandles[0][AXIS_X];
	PxU32* boxHandlesY = bpDesc->boxHandles[0][AXIS_Y];
	PxU32* boxHandlesZ = bpDesc->boxHandles[0][AXIS_Z];

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x*blockIdx.x;

	for(PxU32 i = globalThreadIndex; i<numCreatedHandleSize; i+=blockDim.x*gridDim.x)
	{
		const PxU32 objectHandle = createdHandles[i];

		const PxU32 startHandle = createHandle(objectHandle, true, false);
		const PxU32 endHandle = createHandle(objectHandle, false, false);

		boxHandlesX[boxSapBox1DX[objectHandle].mMinMax[0]] = startHandle;
		boxHandlesX[boxSapBox1DX[objectHandle].mMinMax[1]] = endHandle;

		boxHandlesY[boxSapBox1DY[objectHandle].mMinMax[0]] = startHandle;
		boxHandlesY[boxSapBox1DY[objectHandle].mMinMax[1]] = endHandle;

		boxHandlesZ[boxSapBox1DZ[objectHandle].mMinMax[0]] = startHandle;
		boxHandlesZ[boxSapBox1DZ[objectHandle].mMinMax[1]] = endHandle;
	}
}

extern "C" __global__ void computeIncrementalComparisonHistograms_Stage1(const PxgBroadPhaseDesc* bpDesc)	// BP_COMPUTE_INCREMENTAL_CMP_COUNTS1
{
	const PxU32 WARP_PERBLOCK_SIZE = PxgBPKernelBlockDim::BP_COMPUTE_INCREMENTAL_CMP_COUNTS1/WARP_SIZE;
	
	//const PxU32 numHandles = bpDesc->numPreviousHandles + bpDesc->numCreatedHandles - bpDesc->numRemovedHandles;
	const PxU32 numHandles = bpDesc->numPreviousHandles;
	PxU32 nbProjections = numHandles * 2;

	const PxU32* projectionRanks0 = bpDesc->boxProjectionRanks[AXIS_X];
	const PxU32* projectionRanks1 = bpDesc->boxProjectionRanks[AXIS_Y];
	const PxU32* projectionRanks2 = bpDesc->boxProjectionRanks[AXIS_Z];

	const PxU32* sortedHandles0 = bpDesc->boxHandles[0][AXIS_X];
	const PxU32* sortedHandles1 = bpDesc->boxHandles[0][AXIS_Y];
	const PxU32* sortedHandles2 = bpDesc->boxHandles[0][AXIS_Z];

	PxU32* incComparisons0 = bpDesc->incrementalComparisons[AXIS_X];
	PxU32* incComparisons1 = bpDesc->incrementalComparisons[AXIS_Y];
	PxU32* incComparisons2 = bpDesc->incrementalComparisons[AXIS_Z];

	//if hardware support shfl, the compiler should be able to strip these three array out
	__shared__ PxU32 sComparisons0[PxgBPKernelBlockDim::BP_COMPUTE_INCREMENTAL_CMP_COUNTS1];
	__shared__ PxU32 sComparisons1[PxgBPKernelBlockDim::BP_COMPUTE_INCREMENTAL_CMP_COUNTS1];
	__shared__ PxU32 sComparisons2[PxgBPKernelBlockDim::BP_COMPUTE_INCREMENTAL_CMP_COUNTS1];

	__shared__ PxU32 sBlockAccum0[WARP_PERBLOCK_SIZE];
	__shared__ PxU32 sBlockAccum1[WARP_PERBLOCK_SIZE];
	__shared__ PxU32 sBlockAccum2[WARP_PERBLOCK_SIZE];

	__shared__ PxU32 sAccum0;
	__shared__ PxU32 sAccum1;
	__shared__ PxU32 sAccum2;

	__shared__ PxU32* sStartEndAccum[2][2][3];

	if(threadIdx.x < 3)
	{
		sStartEndAccum[0][0][threadIdx.x] = bpDesc->startPtHistogram[0][threadIdx.x];//this frame's start projections
		sStartEndAccum[0][1][threadIdx.x] = bpDesc->startPtHistogram[1][threadIdx.x];//last frame's start projections
		sStartEndAccum[1][0][threadIdx.x] = bpDesc->endPtHistogram[0][threadIdx.x]; //this frame's end projections
		sStartEndAccum[1][1][threadIdx.x] = bpDesc->endPtHistogram[1][threadIdx.x]; //Last frame's end projections
	}

	if(threadIdx.x ==4)
	{
		sAccum0 = 0;
		sAccum1 = 0;
		sAccum2 = 0;
	}

	__syncthreads();

	const PxU32 nbBlocksRequired = (nbProjections + blockDim.x-1)/blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x-1)/gridDim.x;

	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE-1);
	const PxU32 warpIndex = threadIdx.x/WARP_SIZE;

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		PxU32 comparisons0 = 0;
		PxU32 comparisons1 = 0;
		PxU32 comparisons2 = 0;

		const PxI32 blockId = blockIdx.x*nbIterationsPerBlock + i;
		const PxI32 readIdx = blockId * blockDim.x + threadIdx.x;

		if(readIdx < nbProjections)
		{
			//Remap ranks into the local indices into the respective arrays (ranks view all 3 axes as a single flattened array)
			const PxU32 rank0 = projectionRanks0[readIdx];
			const PxU32 rank1 = projectionRanks1[readIdx];
			const PxU32 rank2 = projectionRanks2[readIdx];

			const PxU32 sortedHandle0 = sortedHandles0[readIdx];
			const PxU32 sortedHandle1 = sortedHandles1[readIdx];
			const PxU32 sortedHandle2 = sortedHandles2[readIdx];

			const PxU32 isDownPass0 = readIdx <= rank0;
			const PxU32 isDownPass1 = readIdx <= rank1;
			const PxU32 isDownPass2 = readIdx <= rank2;

			const PxU32 isStartHandle0 = isStartProjection(sortedHandle0);
			const PxU32 isStartHandle1 = isStartProjection(sortedHandle1);
			const PxU32 isStartHandle2 = isStartProjection(sortedHandle2);

			//KS - when processing the previous list, we need to clamp rank to within the size of the previous list to
			//avoid overrunning a buffer
			const PxU32 start0 = isDownPass0 ? readIdx : rank0+1;
			const PxU32 end0 = isDownPass0 ? PxMin(rank0, nbProjections-1) : readIdx;
			const PxU32 start1 = isDownPass1 ? readIdx : rank1+1;
			const PxU32 end1 = isDownPass1 ? PxMin(rank1, nbProjections-1) : readIdx;
			const PxU32 start2 = isDownPass2 ? readIdx : rank2+1;
			const PxU32 end2 = isDownPass2 ? PxMin(rank2, nbProjections-1) : readIdx;

			const PxU32* startEndAccum0 = sStartEndAccum[isStartHandle0][isDownPass0][0];
			const PxU32* startEndAccum1 = sStartEndAccum[isStartHandle1][isDownPass1][1];
			const PxU32* startEndAccum2 = sStartEndAccum[isStartHandle2][isDownPass2][2];

			//New projections are excluded from the incremental SAP but are used in the box pruning insertion pass. In that case,
			//0 comparisons are recorded for any "new" handle
			/*comparisons0 = isNewProjection(sortedHandle0) ? 0 : startEndAccum0[end0] - startEndAccum0[start0];
			comparisons1 = isNewProjection(sortedHandle1) ? 0 : startEndAccum1[end1] - startEndAccum1[start1];
			comparisons2 = isNewProjection(sortedHandle2) ? 0 : startEndAccum2[end2] - startEndAccum2[start2];*/

			if(!isDeletedProjection(sortedHandle0))
				comparisons0 = startEndAccum0[end0] - startEndAccum0[start0];
			if (!isDeletedProjection(sortedHandle1))
				comparisons1 = startEndAccum1[end1] - startEndAccum1[start1];
			if(!isDeletedProjection(sortedHandle2))
				comparisons2 = startEndAccum2[end2] - startEndAccum2[start2];
		}

		const PxU32 res0 = warpScanAdd<WARP_SIZE>(FULL_MASK, threadIdx.x, threadIndexInWarp, sComparisons0, comparisons0, comparisons0);
		const PxU32 res1 = warpScanAdd<WARP_SIZE>(FULL_MASK, threadIdx.x, threadIndexInWarp, sComparisons1, comparisons1, comparisons1);
		const PxU32 res2 = warpScanAdd<WARP_SIZE>(FULL_MASK, threadIdx.x, threadIndexInWarp, sComparisons2, comparisons2, comparisons2);

		if(threadIndexInWarp == (WARP_SIZE-1))
		{
			sBlockAccum0[warpIndex] = res0 + comparisons0;
			sBlockAccum1[warpIndex] = res1 + comparisons1;
			sBlockAccum2[warpIndex] = res2 + comparisons2;
		}

		PxU32 accum0 = sAccum0;
		PxU32 accum1 = sAccum1;
		PxU32 accum2 = sAccum2;

		__syncthreads();

		if(warpIndex == 0)
		{
			unsigned mask_threadIndexInWarp = __ballot_sync(FULL_MASK, threadIndexInWarp < WARP_PERBLOCK_SIZE);
			if(threadIndexInWarp < WARP_PERBLOCK_SIZE)
			{
				const PxU32 val = sBlockAccum0[threadIndexInWarp];
				const PxU32 res = warpScanAddWriteToSharedMem<WARP_PERBLOCK_SIZE>(mask_threadIndexInWarp, threadIndexInWarp, threadIndexInWarp, sBlockAccum0, val, val);

				if(threadIndexInWarp == (WARP_PERBLOCK_SIZE-1))
					sAccum0 += val+res;
			}
		}
		else if(warpIndex == 1)
		{
			unsigned mask_threadIndexInWarp = __ballot_sync(FULL_MASK, threadIndexInWarp < WARP_PERBLOCK_SIZE);
			if(threadIndexInWarp < WARP_PERBLOCK_SIZE)
			{
				const PxU32 val = sBlockAccum1[threadIndexInWarp];

				const PxU32 res = warpScanAddWriteToSharedMem<WARP_PERBLOCK_SIZE>(mask_threadIndexInWarp, threadIndexInWarp, threadIndexInWarp, sBlockAccum1, val, val);
				if(threadIndexInWarp == (WARP_PERBLOCK_SIZE-1))
					sAccum1 += val+res;
			}	
		}
		else if (warpIndex == 2)
		{
			unsigned mask_threadIndexInWarp = __ballot_sync(FULL_MASK, threadIndexInWarp < WARP_PERBLOCK_SIZE);
			if(threadIndexInWarp < WARP_PERBLOCK_SIZE)
			{
				const PxU32 val = sBlockAccum2[threadIndexInWarp];

				const PxU32 res = warpScanAddWriteToSharedMem<WARP_PERBLOCK_SIZE>(mask_threadIndexInWarp, threadIndexInWarp, threadIndexInWarp, sBlockAccum2, val, val);
				if(threadIndexInWarp == (WARP_PERBLOCK_SIZE-1))
					sAccum2 += val+res;
			}
		}

		__syncthreads();

		if(readIdx < nbProjections)
		{
			incComparisons0[readIdx] = accum0 + res0 + sBlockAccum0[warpIndex];
			incComparisons1[readIdx] = accum1 + res1 + sBlockAccum1[warpIndex];
			incComparisons2[readIdx] = accum2 + res2 + sBlockAccum2[warpIndex];
		}
		__syncthreads();
	}

	__syncthreads();

	if(threadIdx.x == 0)
	{
		//if(blockIdx.x == 0 && blockIdx.z == 0)
		//	printf("incrementalBlockComparisons[%i] = %i\n", region, sAccum1);
		bpDesc->incrementalBlockComparisons[0][blockIdx.x] = sAccum0;
		bpDesc->incrementalBlockComparisons[1][blockIdx.x] = sAccum1;
		bpDesc->incrementalBlockComparisons[2][blockIdx.x] = sAccum2;
	}
}

extern "C" __global__ void computeIncrementalComparisonHistograms_Stage2(PxgBroadPhaseDesc* bpDesc)	// BP_COMPUTE_INCREMENTAL_CMP_COUNTS2
{
	const PxU32 numHandles = bpDesc->numPreviousHandles;
	//const PxU32 numHandles = bpDesc->numPreviousHandles + bpDesc->numCreatedHandles - bpDesc->numRemovedHandles;
	PxU32 nbProjections = numHandles * 2;

	PxU32* incComparisons0 = bpDesc->incrementalComparisons[0];
	PxU32* incComparisons1 = bpDesc->incrementalComparisons[1];
	PxU32* incComparisons2 = bpDesc->incrementalComparisons[2];

	const PxU32* incBlockComparisons0 = bpDesc->incrementalBlockComparisons[0];
	const PxU32* incBlockComparisons1 = bpDesc->incrementalBlockComparisons[1];
	const PxU32* incBlockComparisons2 = bpDesc->incrementalBlockComparisons[2];

	__shared__ PxU32 sBlockAccum0[BLOCK_SIZE];
	__shared__ PxU32 sBlockAccum1[BLOCK_SIZE];
	__shared__ PxU32 sBlockAccum2[BLOCK_SIZE];

	const PxU32 warpIndex = threadIdx.x/WARP_SIZE;
	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE-1);

	if(warpIndex == 0)
	{
		const unsigned mask_threadIndexInWarp = __ballot_sync(FULL_MASK, threadIndexInWarp < BLOCK_SIZE);
		if(threadIndexInWarp < BLOCK_SIZE)
		{
			const PxU32 val = incBlockComparisons0[threadIndexInWarp];

			const PxU32 res = warpScanAddWriteToSharedMem<BLOCK_SIZE>(mask_threadIndexInWarp, threadIndexInWarp, threadIndexInWarp, sBlockAccum0, val, val);
			if(threadIndexInWarp == (BLOCK_SIZE-1) && blockIdx.x == 0)
				bpDesc->totalIncrementalComparisons[0] = res + val;
		}
	}
	else if (warpIndex == 1)
	{
		const unsigned mask_threadIndexInWarp = __ballot_sync(FULL_MASK, threadIndexInWarp < BLOCK_SIZE);
		if(threadIndexInWarp < BLOCK_SIZE)
		{
			const PxU32 val = incBlockComparisons1[threadIndexInWarp];

			const PxU32 res = warpScanAddWriteToSharedMem<BLOCK_SIZE>(mask_threadIndexInWarp, threadIndexInWarp, threadIndexInWarp, sBlockAccum1, val, val);
			if(threadIndexInWarp == (BLOCK_SIZE-1) && blockIdx.x == 0)
				bpDesc->totalIncrementalComparisons[1] = res + val;
		}
	}
	else if (warpIndex == 2)
	{
		const unsigned mask_threadIndexInWarp = __ballot_sync(FULL_MASK, threadIndexInWarp < BLOCK_SIZE);
		if(threadIndexInWarp < BLOCK_SIZE)
		{
			const PxU32 val = incBlockComparisons2[threadIndexInWarp];

			const PxU32 res = warpScanAddWriteToSharedMem<BLOCK_SIZE>(mask_threadIndexInWarp, threadIndexInWarp, threadIndexInWarp, sBlockAccum2, val, val);
			if(threadIndexInWarp == (BLOCK_SIZE-1) && blockIdx.x == 0)
				bpDesc->totalIncrementalComparisons[2] = res + val;
		}
	}

	__syncthreads();

	const PxU32 nbBlocksRequired = (nbProjections + blockDim.x-1)/blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x-1)/gridDim.x;

	const PxU32 blockAccumulation0 = sBlockAccum0[blockIdx.x];
	const PxU32 blockAccumulation1 = sBlockAccum1[blockIdx.x];
	const PxU32 blockAccumulation2 = sBlockAccum2[blockIdx.x];

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxI32 blockId = blockIdx.x*nbIterationsPerBlock + i;
		const PxI32 readIdx = blockId * blockDim.x + threadIdx.x;

		if(readIdx < nbProjections)
		{
			incComparisons0[readIdx] = incComparisons0[readIdx] + blockAccumulation0;
			incComparisons1[readIdx] = incComparisons1[readIdx] + blockAccumulation1;
			incComparisons2[readIdx] = incComparisons2[readIdx] + blockAccumulation2;
		}
	}
}

//This method is called before radix sort to initialize the ranks
extern "C" __global__ void initializeRadixRanks(const PxgBroadPhaseDesc* bpDesc)	// BP_INITIALIZE_RANKS
{
	//Pad for removed handles because they are present in the array right now

	//PxU32 nbProjections = (bpDesc->numPreviousHandles + bpDesc->numCreatedHandles) *2*3;
	//const PxU32 preProjections = bpDesc->numPreviousHandles * 2 *3;
	//const PxU32 nbProjections = (bpDesc->numPreviousHandles + bpDesc->numCreatedHandles) *2*3;
	const PxU32 nbProjections = bpDesc->numPreviousHandles  *2;

	const PxU32 nbProjectionsOver4 = (nbProjections + 3)/4;

	const PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	uint4* projectionRanksX = reinterpret_cast<uint4*>(bpDesc->boxProjectionRanks[0]);
	uint4* projectionRanksY = reinterpret_cast<uint4*>(bpDesc->boxProjectionRanks[1]);
	uint4* projectionRanksZ = reinterpret_cast<uint4*>(bpDesc->boxProjectionRanks[2]);

	for(PxU32 i = globalThreadIdx; i < nbProjectionsOver4; i+=(blockDim.x*gridDim.x))
	{
		const PxU32 fourI = 4*i;
		projectionRanksX[i] = make_uint4(fourI, fourI + 1, fourI + 2, fourI + 3);
		projectionRanksY[i] = make_uint4(fourI, fourI + 1, fourI + 2, fourI + 3);
		projectionRanksZ[i] = make_uint4(fourI, fourI + 1, fourI + 2, fourI + 3);
	}
}

extern "C" __global__ void updateHandles(const PxgBroadPhaseDesc* bpDesc)	// BP_UDPATE_HANDLES
{
	//PxU32 nbProjections = (bpDesc->numPreviousHandles + bpDesc->numCreatedHandles) * 2*3;

	//const PxU32 nbProjections = (bpDesc->numPreviousHandles + bpDesc->numCreatedHandles) *2*3;
	const PxU32 nbProjections = bpDesc->numPreviousHandles  *2;

	const PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	PxU32* handlesX = bpDesc->boxHandles[0][AXIS_X];
	PxU32* handlesY = bpDesc->boxHandles[0][AXIS_Y];
	PxU32* handlesZ = bpDesc->boxHandles[0][AXIS_Z];

	const PxU32* tmpHandlesX = bpDesc->boxHandles[1][AXIS_X];
	const PxU32* tmpHandlesY = bpDesc->boxHandles[1][AXIS_Y];
	const PxU32* tmpHandlesZ = bpDesc->boxHandles[1][AXIS_Z];

	const PxU32* projectionRanksX = bpDesc->boxProjectionRanks[AXIS_X];
	const PxU32* projectionRanksY = bpDesc->boxProjectionRanks[AXIS_Y];
	const PxU32* projectionRanksZ = bpDesc->boxProjectionRanks[AXIS_Z];
	//PxU32* projectionRanks = bpDesc->boxProjectionRanks[0];

	for(PxU32 i = globalThreadIdx; i < nbProjections; i+=(blockDim.x*gridDim.x))
	{
		const PxU32 projX = projectionRanksX[i];
		const PxU32 projY = projectionRanksY[i];
		const PxU32 projZ = projectionRanksZ[i];

		handlesX[i] = tmpHandlesX[projX];
		handlesY[i] = tmpHandlesY[projY];
		handlesZ[i] = tmpHandlesZ[projZ];
	}
}

//compute aggregate/actor pairs
//aggFoundPairs blockIdx.y == 0, aggLostPairs blockIdx.y == 2
//actorFoundPairs blockIdx.y == 1, actorFoundPairs blockIdx.y == 3 
extern "C" __global__ void accumulateReportsStage_1(const PxgBroadPhaseDesc* bpDesc)	// BP_ACCUMULATE_REPORT_STAGE_1 //###ONESHOT
{
	const PxU32 nbPairs = (blockIdx.y / 2 == 0) ? bpDesc->sharedFoundPairIndex : bpDesc->sharedLostPairIndex;

	const PxgBroadPhasePair* pairs = (blockIdx.y / 2 == 0) ? bpDesc->foundPairReport : bpDesc->lostPairReport;

	const bool outputAggReport = ((blockIdx.y % 2) == 0) ? true : false;
	
	const PxU32 maxPairs = bpDesc->max_found_lost_pairs;
	const PxU32 nbPairsSafe = PxMin(nbPairs, maxPairs);

	const Bp::VolumeData* volumeData = bpDesc->aabbMngr_volumeData;
	//if(!volumeData)
	//	printf("volumeData null\n");

	const PxU32 warpPerBlock = PxgBPKernelBlockDim::BP_COMPUTE_INCREMENTAL_CMP_COUNTS1 / WARP_SIZE;

	__shared__ PxU32 sWarpAccum[warpPerBlock];
	__shared__ PxU32 sAccum;

	const PxU32 block_size = PxgBPKernelGridDim::BP_COMPUTE_INCREMENTAL_CMP_COUNTS1;

	const PxU32 totalBlockRequired = (nbPairs + (blockDim.x - 1)) / blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (block_size - 1)) / block_size;

	const PxU32 idx = threadIdx.x;

	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE - 1);
	const PxU32 warpIndex = threadIdx.x / WARP_SIZE;

	if (idx == (WARP_SIZE - 1))
		sAccum = 0;

	__syncthreads();

	for (PxU32 i = 0; i < numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		bool isAgg = false;
		bool isActor = false;

		if (workIndex < nbPairsSafe)
		{
			const PxgBroadPhasePair& pair = pairs[workIndex];

			//const bool isSingleActorA = volumeData[pair.mVolA].isSingleActor();
			//const bool isSingleActorB = volumeData[pair.mVolB].isSingleActor();
			const bool isSingleActorA = volumeData ? volumeData[pair.mVolA].isSingleActor() : true;
			const bool isSingleActorB = volumeData ? volumeData[pair.mVolB].isSingleActor() : true;

			assert(pair.mVolA != pair.mVolB);

			if (!(isSingleActorA && isSingleActorB))
				isAgg = true;
			else
				isActor = true;
		}

		//There is an aggregate or an actor 
		const bool valToBallot = outputAggReport ? isAgg : isActor;
		const PxU32 accum = __popc(__ballot_sync(FULL_MASK, valToBallot));

		if (threadIndexInWarp == (WARP_SIZE - 1))
		{
			sWarpAccum[warpIndex] = accum;

			//printf("blockIdx.y %i outputAggReport %i blockIdx %i warpIndex %i aggAccum %i\n", blockIdx.y, outputAggReport, blockIdx.x, warpIndex, accum);
		}

		__syncthreads();

		unsigned mask_idx = __ballot_sync(FULL_MASK, threadIndexInWarp < warpPerBlock);
		if (idx < warpPerBlock)
		{
			const PxU32 value = sWarpAccum[idx];

			const PxU32 res = warpReduction<AddOpPxU32, PxU32>(mask_idx, value);

			if (idx == (warpPerBlock - 1))
				sAccum += res;
		}

		__syncthreads();
	}

	if (idx == (warpPerBlock - 1))
	{
		if (outputAggReport)
			bpDesc->aggReportBlock[(blockIdx.y / 2)][blockIdx.x] = sAccum;
		else
			bpDesc->actorReportBlock[(blockIdx.y / 2)][blockIdx.x] = sAccum;
	}
}

//aggFoundPairs blockIdx.y == 0, aggLostPairs blockIdx.y == 2
//actorFoundPairs blockIdx.y == 1, actorFoundPairs blockIdx.y == 3 
extern "C" __global__ void accumulateReportsStage_2(PxgBroadPhaseDesc* bpDesc)	// BP_ACCUMULATE_REPORT_STAGE_2 //###ONESHOT
{
	const PxU32 warpPerBlock = PxgBPKernelBlockDim::BP_COMPUTE_INCREMENTAL_CMP_COUNTS2 / WARP_SIZE;

	const PxU32 numBlocks = PxgBPKernelGridDim::BP_COMPUTE_INCREMENTAL_CMP_COUNTS2;

	__shared__ PxU32 sWarpAccum[warpPerBlock];
	__shared__ PxU32 sBlockHistogram[numBlocks];
	__shared__ PxU32 sAccum;

	const bool outputAggReport = ((blockIdx.y % 2)  == 0 )? true : false;

	const PxU32 idx = threadIdx.x;

	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE - 1);
	const PxU32 warpIndex = threadIdx.x / WARP_SIZE;

	if (idx == (WARP_SIZE - 1))
		sAccum = 0;

	//accumulate num pairs per block and compute exclusive run sum(blockIdx.y == 0 && blockIdx.y == 1 accumulate aggFoundPairs)
	//blockIdx.y == 2 && blockIdx.y == 3 accumulate aggLostPairs
	unsigned mask_idx = __ballot_sync(FULL_MASK, threadIndexInWarp < numBlocks);
	if (warpIndex == 0 && threadIndexInWarp < numBlocks)
	{
		PxU32 blockNumPairs;
		if(outputAggReport)
			blockNumPairs = bpDesc->aggReportBlock[(blockIdx.y / 2)][threadIndexInWarp];
		else
			blockNumPairs = bpDesc->actorReportBlock[(blockIdx.y / 2)][threadIndexInWarp];

		//const PxU32 blockNumPairs = bpDesc->aggReportBlock[(blockIdx.y / 2)][threadIndexInWarp];

		const PxU32 result = warpScan<AddOpPxU32, PxU32>(mask_idx, blockNumPairs);
		//store exclusive run sum
		sBlockHistogram[threadIndexInWarp] = result - blockNumPairs;

		if (threadIndexInWarp == numBlocks - 1)
		{
			if (blockIdx.y / 2 == 0)
			{
				if(blockIdx.y == 0)
					bpDesc->sharedFoundAggPairIndex = result;
			}
			else
			{
				if(blockIdx.y == 2)
					bpDesc->sharedLostAggPairIndex = result;
				//bpDesc->sharedLostActorPairIndex = bpDesc->sharedLostPairIndex - result;*/
			}
			//printf("blockIdx %i blockNumPairs %i numPairs %i\n", blockIdx.x, blockNumPairs, result);
		}
	}

	__syncthreads();

	PxU32 nbPairs;
	const PxgBroadPhasePair* pairs;
	PxgBroadPhasePair* aggOrActorPairs;
	PxU32 maxPairs;

	if (blockIdx.y == 0)
	{
		nbPairs = bpDesc->sharedFoundPairIndex;
		pairs = bpDesc->foundPairReport;
		maxPairs = bpDesc->max_found_lost_agg_pairs;
		aggOrActorPairs = bpDesc->foundAggPairReport;	//aggregate found pair
	}
	else if (blockIdx.y == 1)
	{
		nbPairs = bpDesc->sharedFoundPairIndex;
		pairs = bpDesc->foundPairReport;
		maxPairs = bpDesc->max_found_lost_pairs;
		aggOrActorPairs = bpDesc->foundActorPairReport; //actor found pair
	}
	else if (blockIdx.y == 2)
	{
		nbPairs = bpDesc->sharedLostPairIndex;
		pairs = bpDesc->lostPairReport;
		maxPairs = bpDesc->max_found_lost_agg_pairs;
		aggOrActorPairs = bpDesc->lostAggPairReport; //aggregate lost pair
	}
	else
	{
		nbPairs = bpDesc->sharedLostPairIndex;
		pairs = bpDesc->lostPairReport;
		maxPairs = bpDesc->max_found_lost_pairs;
		aggOrActorPairs = bpDesc->lostActorPairReport; //actor lost pair
	}

	const PxU32 nbPairsSafe = PxMin(nbPairs, bpDesc->max_found_lost_pairs); // AD: pairs buffer is sized according to this, aggregate count is not considered.
	
	const Bp::VolumeData* volumeData = bpDesc->aabbMngr_volumeData;

	const PxU32 totalBlockRequired = (nbPairs + (blockDim.x - 1)) / blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (numBlocks - 1)) / numBlocks;

	const PxU32 blockStartIndex = sBlockHistogram[blockIdx.x];

	for (PxU32 i = 0; i < numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		bool isAgg = false;
		bool isActor = false;

		PxgBroadPhasePair pair;

		if (workIndex < nbPairsSafe)
		{
			pair = pairs[workIndex];

			//const bool isSingleActorA = volumeData[pair.mVolA].isSingleActor();
			//const bool isSingleActorB = volumeData[pair.mVolB].isSingleActor();
			const bool isSingleActorA = volumeData ? volumeData[pair.mVolA].isSingleActor() : true;
			const bool isSingleActorB = volumeData ? volumeData[pair.mVolB].isSingleActor() : true;

			assert(pair.mVolA != pair.mVolB);

			if (!(isSingleActorA && isSingleActorB))
				isAgg = true;
			else
				isActor = true;
		}

		const PxU32 threadMask = (1 << threadIndexInWarp) - 1;

		const bool valToBallot = outputAggReport ? isAgg : isActor;

		PxU32 mask = __ballot_sync(FULL_MASK, valToBallot);

		const PxU32 accum = __popc(mask);

		const PxU32 offset =  __popc(mask&threadMask);

		if (threadIndexInWarp == (WARP_SIZE - 1))
			sWarpAccum[warpIndex] = accum;

		const PxU32 prevAccum = sAccum;

		__syncthreads();

		unsigned mask_idx = __ballot_sync(FULL_MASK, threadIndexInWarp < warpPerBlock);
		if (idx < warpPerBlock)
		{
			const PxU32 value = sWarpAccum[idx];

			const PxU32 res = warpScan<AddOpPxU32, PxU32>(mask_idx, value);

			sWarpAccum[idx] = res - value;

			if (idx == warpPerBlock - 1)
				sAccum += res;
		}

		__syncthreads();

		//write to output
		if (workIndex < nbPairs)
		{
			const PxU32 index = offset + prevAccum + sWarpAccum[warpIndex] + blockStartIndex;

			if (outputAggReport)
			{
				if (isAgg && index < maxPairs)
					aggOrActorPairs[index] = pair;
			}
			else
			{
				//actor
				if (isActor && index < maxPairs)
					aggOrActorPairs[index] = pair;
			}
		}

		__syncthreads(); // Needed because we write sWarpAccum before the first sync in the next iteration
	}
}

//copy found and lost reports to the mapped memory 
extern "C" __global__ void copyReports(PxgBroadPhaseDesc* bpDesc)	// BP_COPY_REPORTS //###ONESHOT
{
	const PxU32 max_found_lost_pairs = bpDesc->max_found_lost_pairs;

	// sharedFoundPairIndex will contain the number of pairs needed, this is coming directly from the part of the broadphase
	// that does not know about aggregates. The sharedFoundAggPairIndex is already corrected and will contain the actual number
	// of aggregate pairs in the pairs list.
	// this means that we need to correct the total size to account for the max size of the pairs buffer, and can then 
	// subtract the number of aggregate pairs written to that buffer to get the final actor count.
	const PxU32 nbCreatedPairs = PxMin(bpDesc->sharedFoundPairIndex, max_found_lost_pairs) - bpDesc->sharedFoundAggPairIndex;
	const PxU32 nbLostPairs = PxMin(bpDesc->sharedLostPairIndex, max_found_lost_pairs) - bpDesc->sharedLostAggPairIndex;

	const PxU32* foundReports = reinterpret_cast<const PxU32*>(bpDesc->foundActorPairReport);
	const PxU32* lostReports = reinterpret_cast<const PxU32*>(bpDesc->lostActorPairReport);

	PxU32* foundReportMap = reinterpret_cast<PxU32*>(bpDesc->foundPairReportMap);
	PxU32* lostReportMap = reinterpret_cast<PxU32*>(bpDesc->lostPairReportMap);

	const PxU32 idx = threadIdx.x + blockIdx.x * blockDim.x;

	const bool foundOverflow = (bpDesc->sharedFoundPairIndex > max_found_lost_pairs);
	const bool lostOverflow = (bpDesc->sharedLostPairIndex > max_found_lost_pairs);
	if (threadIdx.x == 0 && blockIdx.x == 0)
		bpDesc->found_lost_pairs_overflow_flags = foundOverflow || lostOverflow;

	const PxU32 nbCreateElements = (nbCreatedPairs * sizeof(PxgBroadPhasePair)) / sizeof(PxU32);
	const PxU32 nbLostElements = (nbLostPairs * sizeof(PxgBroadPhasePair)) / sizeof(PxU32);

	for (PxU32 i = idx; i<nbCreateElements; i += blockDim.x * gridDim.x)
		foundReportMap[i] = foundReports[i];

	for (PxU32 i = idx; i<nbLostElements; i += blockDim.x * gridDim.x)
		lostReportMap[i] = lostReports[i];
}

// Note: We're using this kernel also during test time to check that GPU asserts are working
extern "C" __global__ void bpSignalComplete(PxU32* signal)	// BP_SIGNAL_COMPLETE
{
	assert(signal);
	*signal = 1;
}
