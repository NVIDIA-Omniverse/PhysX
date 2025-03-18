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

#ifndef __MIDPHASE_ALLOCATION_CUH__
#define __MIDPHASE_ALLOCATION_CUH__

#include "foundation/PxSimpleTypes.h"
#include "convexNpCommon.h"
#include <vector_types.h>

using namespace physx;

/*
	// TODO: read/write infos

	AD 8.2023: some comments about all of this. The size of the collision stack is PxGpuDynamicsMemoryConfig::collisionStackSize

    The following parameters are involved:
    * numpairsAfterMP:        The number of convex - triangle pairs after the midphase. These are pairs between 1 triangle of a trimesh
                              and a complete convex. Will be padded to a multiple of 4 during stack allocation to satisfy 16B alignment.
    * numPaddedPairsAfterMP:  A pair here is again a single triangle of a trimesh and a complete convex. But in contrast to the one above,
                              the list holding the triangle indices is padded to multiples of 4 and duplicated for each contact manager.
                              This is done because we do a radix sort on these indices later, which happens on uint4s. So all the lists
                              need to be padded to a multiple of 4, and we need the temp buffer as well. So this count represents the length
                              of the flat list needed to place all the triangle indices, with per-CM padding and temp buffers.
                              The list will look something like this (x is padding, y is temp buffer):

                              cm 0            | cm 1            | cm 2 ..
                              0 1 2 p t t t t | 3 4 p p t t t t | 5 6 7 8 9 p p p t t t t t t t t | ...

	numPairsAfterMP4 = (numPairsAfterMP + 3) & (~3)

	This is the layout of the collision stack (growing to the bottom):

	|---------
	|* PairsGPU: uint4 x numPairsAfterMP4
	|    x: contact manager index
	|    y: triangle index (of the triangle mesh)
	|    z: offset from first triangle found for this contact manager
	|    w: npWorkItem.shapeRef1 (ref of convex mesh)
	|
	|---------
	|* TriNormalAndIndex: ConvexTriNormalAndIndex x numPairsAfterMP4
	|    x,y,z: minNormal from convexTriangleContactGen  (confirm: the normal is from a convex mesh triangle, right?)
	|    w:     remapCPUIndex of trimesh triangle index. (What? Why?)
	|---------
	|* Contacts: ConvexTriContacts x numPairsAfterMP4
	|
	|---------
	|* TriangleIndices: PxU32 * numPaddedPairsAfterMP
	|
	|---------
	|* maxDepth: PxReal * numPairsAfterMP4
	|
	|---------
	|* intermediate data: ConvexTriIntermediateData * numPairsAfterMP4
	|
	|---------
	|* secondPass data: PxU32 * numPairsAfterMP4

	We only fill the pairs array in the midphase kernels. Everything else will be written in the core narrowphase kernels.
	Not that pairs belonging to the same contact manager are not necessarily consecutive in the pairs list, because of the warp-wide
	write in the midphase. We do maintain a count + offset into a per-CM list for each pair when filling the pairs list with atomics.
	This count + offset is then used to calculate the index into per-CM lists to write all the other data in the core narrowphase kernels.

*/

__device__ __forceinline__ PxU32 calculateConvexMeshPairMemRequirement()
{
	return (sizeof(uint4)                     // sPairsGPU
          + sizeof(ConvexTriNormalAndIndex)   // normal and index, 4 floats,
          + sizeof(ConvexTriContacts)         // 4 contacts for convexTriContacts
          + sizeof(PxReal)                    // maxdepth
          + sizeof(ConvexTriIntermediateData) // intermediate data
          + sizeof(PxU32)                     // secondPass indices
		  + sizeof(PxU32) * 2);               // triangle indices + radix sort data. We do not account for the padding here.
}

// This is calculating the worst-case padding intentionally to avoid stack overflows!
__device__ PX_FORCE_INLINE PxU32 calculateAdditionalPadding(const PxU32 numContactManagers)
{
	// the number of pairs are padded to a multiple of 4, so we need additional space for at most 3.
	// we subtract the memory needed for the triangle index buffer here because that one is allocated with a different count.
	const PxU32 afterPairPadding = 3 * (calculateConvexMeshPairMemRequirement() - 2 * sizeof(PxU32));

	// for the triangle index buffer, we pad the triangle count for each contact manager to a multiple of 4.
	// We also need the temp buffer for the radix sort, so that means we need to account for another
	// 3 + 3 PxU32s per contact manager.
	const PxU32 afterTriangleIndexPadding = numContactManagers * 6 * sizeof(PxU32);

	return (afterPairPadding + afterTriangleIndexPadding);
}

__device__ PX_FORCE_INLINE PxU32 calculateMaxPairs(const PxU32 stackSizeBytes, const PxU32 numContactManagers)
{
	const PxU32 additionalPadding = calculateAdditionalPadding(numContactManagers);

	const PxU32 memAvailableAfterPadding = stackSizeBytes - additionalPadding;
	const PxU32 memRequiredPerPair = calculateConvexMeshPairMemRequirement();

	return (memAvailableAfterPadding / memRequiredPerPair);
}

__device__ __forceinline__ void midphaseAllocate( ConvexTriNormalAndIndex** cvxTriNIGPU, ConvexTriContacts** cvxTriContactsGPU, 
	PxReal** cvxTriMaxDepthGPU, ConvexTriIntermediateData** cvxTriIntermGPU, PxU32** orderedCvxTriIntermGPU,
	PxU32** cvxTriSecondPassedGPU, uint4** pairsGPU,
	PxU8 * stackPtr, const PxU32 numpairsAfterMPUnpadded, const PxU32 numPaddedPairsAfterMP)
{
	//Pad up to a multiple of 4 so we don't get misaligned addresses
	const PxU32 numPairsAfterMP = (numpairsAfterMPUnpadded + 3)&(~3);

	// I don't know why this line is needed.
	PxU8 * newGpuIntermStackPtr = reinterpret_cast<PxU8* >(stackPtr);

	// the pairs already exist, they have been written in the midphase kernel. So this allocation
	// only makes sure we write all the other data behind it.
	*pairsGPU = (uint4 *)newGpuIntermStackPtr;
	newGpuIntermStackPtr += numPairsAfterMP * sizeof(uint4);

	// Allocate intermediate arrays:
	*cvxTriNIGPU = reinterpret_cast<ConvexTriNormalAndIndex* >(newGpuIntermStackPtr);
	newGpuIntermStackPtr += numPairsAfterMP * sizeof(ConvexTriNormalAndIndex);

	*cvxTriContactsGPU = reinterpret_cast<ConvexTriContacts* >(newGpuIntermStackPtr);
	newGpuIntermStackPtr += numPairsAfterMP * sizeof(ConvexTriContacts); //ConvexTriContacts has space of 4 contacts

	*orderedCvxTriIntermGPU = reinterpret_cast<PxU32*>(newGpuIntermStackPtr);
	newGpuIntermStackPtr += numPaddedPairsAfterMP * sizeof(PxU32); // AD: see convexNpCommon.h - numPaddedPairsAfterMP is already padded up to multiple of 4 and duplicated for allow for a radix sort.

	*cvxTriMaxDepthGPU = reinterpret_cast<PxReal*>(newGpuIntermStackPtr);
	newGpuIntermStackPtr += numPairsAfterMP * sizeof(PxReal);

	*cvxTriIntermGPU = reinterpret_cast<ConvexTriIntermediateData*>(newGpuIntermStackPtr);
	newGpuIntermStackPtr += numPairsAfterMP * sizeof(ConvexTriIntermediateData);

	*cvxTriSecondPassedGPU = reinterpret_cast<PxU32*>(newGpuIntermStackPtr);
	newGpuIntermStackPtr += numPairsAfterMP * sizeof(PxU32);
}

#endif