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

#include "foundation/PxMath.h"
#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"

#include "AlignedTransform.h"

#include "PxgContactManager.h"
#include "PxgNpKernelIndices.h"
#include "PxgPersistentContactManifold.h"
#include "PxsContactManagerState.h"

#include "PxgCommonDefines.h"
#include "reduction.cuh"
#include "contactReduction.cuh"

using namespace physx;

extern "C" __host__ void initNarrowphaseKernels3() {}

#include "manifold.cuh"
#include "nputils.cuh"


#include "cudaNpCommon.h"
#include "convexNpCommon.h"


//This method compacts the depthBuffer + nicBuffer to only include the set of triangles that actually have contacts.
//These then must be sorted based on depth values to construct patches, then appended and then output
__device__ PX_FORCE_INLINE
int purgeEmptyTriangles(const PxU32 syncMask, PxReal* depthBuffer, ConvexTriNormalAndIndex* nicBuffer, int count, PxU32* tempCounts)
{
	const int tI = threadIdx.x;

	PxU32 nbTriangles = 0;

	int loaded = 0;
	for (; loaded < count; loaded += 32)
	{
		int nbToLoad = PxMin(count - loaded, 32);

		PxReal d = tI < nbToLoad ? depthBuffer[loaded + tI] : FLT_MAX;
		PxU32 nbContacts = nbToLoad ? ConvexTriNormalAndIndex::getNbContacts(nicBuffer[loaded + tI].index) : 0;
		int valid = d != FLT_MAX && nbContacts > 0;
		int validBits = __ballot_sync(syncMask, valid);
		int dest = __popc(validBits & ((1 << tI) - 1)) + nbTriangles;

		if (valid)
		{
			depthBuffer[dest] = d;
			tempCounts[dest] = ((loaded + tI) << 4) | nbContacts;
			nicBuffer[dest] = nicBuffer[loaded + tI];
		}

		nbTriangles += __popc(validBits);
	}

	return nbTriangles;
}

struct Scratch
{
	volatile ConvexMeshPair pair;
	int pairIndex;
};

__device__
void correlate(
	const ConvexMeshPair* PX_RESTRICT				meshPairBuffer,
	ConvexTriNormalAndIndex ** PX_RESTRICT			nicBufferPtr,
	const ConvexTriContacts ** PX_RESTRICT			contactBufferPtr,
	PxReal ** PX_RESTRICT							depthBufferPtr,
	PxU32* PX_RESTRICT								tempCounts,
	Scratch* PX_RESTRICT							s,
	PxgPersistentContactMultiManifold* PX_RESTRICT	outBuffer,
	PxsContactManagerOutput* PX_RESTRICT			cmOutputs,
	const PxU32										numPairs,
	const PxReal									clusterBias,
	ConvexTriContact* PX_RESTRICT					tempConvexTriContacts,
	PxU32* PX_RESTRICT								pTempContactIndex
)
{
	__shared__ PxU32 triangleRunSums[CORRELATE_WARPS_PER_BLOCK][WARP_SIZE];
	__shared__ PxU32 startIndices[CORRELATE_WARPS_PER_BLOCK][WARP_SIZE];
	__shared__ PxU32 triIndices[CORRELATE_WARPS_PER_BLOCK][WARP_SIZE];

	PxU32* triRunSums = triangleRunSums[threadIdx.y];
	PxU32* triStartIndex = startIndices[threadIdx.y];
	PxU32* triInds = triIndices[threadIdx.y];

	const int pairIndex = threadIdx.y + CORRELATE_WARPS_PER_BLOCK * blockIdx.x;

	if (pairIndex >= numPairs)
		return;

	s->pairIndex = pairIndex;
	const int tI = threadIdx.x;
	if (tI < sizeof(ConvexMeshPair) / 4)
		((int*)(&s->pair))[tI] = ((int*)(meshPairBuffer + pairIndex))[tI];

	__syncwarp();

	int start = s->pair.startIndex, count = s->pair.count;

	if (count == CONVEX_TRIMESH_CACHED)
	{
		// We've hit cache, nothing to do here
		return;
	}

	ConvexTriNormalAndIndex* PX_RESTRICT nicBuffer = *nicBufferPtr;
	const ConvexTriContacts* PX_RESTRICT contactBuffer = *contactBufferPtr;
	PxReal* PX_RESTRICT depthBuffer = *depthBufferPtr;


	PxU32 nbTriangles = purgeEmptyTriangles(FULL_MASK, depthBuffer + start, nicBuffer + start, count, tempCounts + start);

	__syncwarp();


	if (nbTriangles == 0)
	{
		outBuffer[pairIndex].mNbManifolds = 0;
		return;
	}

	//Now, we loop creating patches and contacts from the triangles that actually have contacts!

	PxU32 nbPatches = 0;
	PxU32 numContactsTotal = 0;

	PxU32 remainingTriangles = nbTriangles;



	for (; nbPatches < MULTIMANIFOLD_MAX_MANIFOLDS && remainingTriangles != 0; nbPatches++)
	{
		//We loop through all triangles, load contacts, compress etc...
		//(1) Find deepest triangle...


		int bestIndex = 0;
		PxReal bestDepth = PX_MAX_F32;
		for (PxU32 i = threadIdx.x; i < nbTriangles; i += 32)
		{
			if (i < nbTriangles)
			{
				if (tempCounts[start + i]) //Still has contacts!
				{
					PxReal d = depthBuffer[start + i];
					if (d < bestDepth)
					{
						bestDepth = d;
						bestIndex = start + i;
					}
				}
			}
		}

		int i0 = minIndex(bestDepth, FULL_MASK, bestDepth);

		bestIndex = __shfl_sync(FULL_MASK, bestIndex, i0);

		//Now we know which triangle defines this plane...

		PxVec3 bestNormal = nicBuffer[bestIndex].normal;

		PxU32 currentContactMask = 0;


		PxVec3 pA(0.f);
		PxReal separation = 0.f;
		PxU32 contactTriangleIndex = 0;
		bool hasContact = false;

		for (PxU32 i = 0; i < nbTriangles; i += 32)
		{
			PxVec3 normal(0.f);
			PxU32 triIndex = 0;
			PxU32 nbContacts = 0;
			PxU32 startIndex = 0;

			if ((i + threadIdx.x) < nbTriangles)
			{
				PxU32 index = tempCounts[start + i + threadIdx.x];
				//We read contacts from the temp contacts buffer because this will get cleared when the contacts are considered
				//for a patch...
				nbContacts = index & 7;
				startIndex = index >> 4;
				if (nbContacts)
				{
					ConvexTriNormalAndIndex nic = nicBuffer[start + i + threadIdx.x];
					normal = nic.normal;
					triIndex = ConvexTriNormalAndIndex::getTriangleIndex(nic.index);
				}
			}

			bool accept = normal.dot(bestNormal) > PATCH_ACCEPTANCE_EPS;

			if(!accept)
				nbContacts = 0;


			PxU32 acceptMask = __ballot_sync(FULL_MASK, accept);

			remainingTriangles -= __popc(acceptMask);

			PxU32 summedContacts = warpScan<AddOpPxU32, PxU32>(FULL_MASK, nbContacts);

			triRunSums[threadIdx.x] = summedContacts - nbContacts;
			triStartIndex[threadIdx.x] = startIndex;
			triInds[threadIdx.x] = triIndex;

			PxU32 totalContacts = __shfl_sync(FULL_MASK, summedContacts, 31);
			__syncwarp();

			for (PxU32 c = 0; c < totalContacts;)
			{
				//calculate my read index (where I'm offset from...)
				PxU32 readMask = ~currentContactMask;
				bool needsContact = readMask & (1 << threadIdx.x);
				PxU32 readIndex = c + warpScanExclusive(readMask, threadIdx.x);

				if (needsContact && readIndex < totalContacts)
				{
					PxU32 idx = binarySearch<PxU32>(triRunSums, 32u, readIndex);

					PxU32 offset = readIndex - triRunSums[idx];

					PxU32 startIdx = triStartIndex[idx] + start;

					const PxU32 contactStartIdx = contactBuffer[startIdx].index;


					float4 v = tempConvexTriContacts[contactStartIdx + offset].contact_sepW;

					pA = PxVec3(v.x, v.y, v.z);
					separation = v.w;
					contactTriangleIndex = triInds[idx];
					hasContact = true;
				}

				currentContactMask = __ballot_sync(FULL_MASK, hasContact); //Mask every thread that has a contact!

				c += __popc(readMask);
				PxU32 nbContacts = __popc(currentContactMask);

				if (nbContacts > SUBMANIFOLD_MAX_CONTACTS)
				{
					currentContactMask = contactReduce<true, false, SUBMANIFOLD_MAX_CONTACTS, false>(pA, separation, bestNormal, currentContactMask, clusterBias);
				}
				hasContact = currentContactMask & (1 << threadIdx.x);
			}

			__syncwarp(); //triRunSums is read and written in the same loop - separate read and write with syncs

			if (accept)
			{
				tempCounts[start + i + threadIdx.x] = 0; //Clear this triangle - it is done!
			}

		}

		__syncwarp();
		PxU32 nbContacts = __popc(currentContactMask);

		//When we reach here, we have handled all the triangles, so we should output this patch...
		PxgContact * cp = outBuffer[s->pair.cmIndex].mContacts[nbPatches] + warpScanExclusive(currentContactMask, tI);

		if (threadIdx.x == 0)
			outBuffer[s->pair.cmIndex].mNbContacts[nbPatches] = nbContacts;

		numContactsTotal += nbContacts;
		if (currentContactMask & (1 << tI))
		{
			PxVec3 nor = ldS(s->pair.aToB).rotate(bestNormal);

			cp->normal = -nor;
			cp->pointA = pA;
			// TODO: PxTransform::transform is incredibly profligate with registers (i.e. the write-out phase uses 
			// more regs than the contact culling phase). Better would be to stash the transform in matrix form 
			// when not under register pressure (i.e. when initially loading it) then use a custom shmem
			//  matrix multiply, which requires essentially no tmps

			PxVec3 pointB = ldS(s->pair.aToB).transform(pA) + nor * separation;

			cp->pointB = pointB;
			cp->penetration = separation;
			cp->triIndex = contactTriangleIndex;

			/*printf("cp normal(%f, %f, %f)\n", cp->normal.x, cp->normal.y, cp->normal.z);
			printf("cp pointA(%f, %f, %f)\n", pA.x, pA.y, pA.z);
			printf("cp pointB(%f, %f, %f)\n", pointB.x, pointB.y, pointB.z);*/

		}

	}

	if (nbPatches > 0)
	{
		// TODO avoroshilov: decide what to do with aligned transforms
		PxAlignedTransform aToB_aligned;// = s->pair.aToB;
		aToB_aligned.p.x = s->pair.aToB.p.x;
		aToB_aligned.p.y = s->pair.aToB.p.y;
		aToB_aligned.p.z = s->pair.aToB.p.z;

		aToB_aligned.q.q.x = s->pair.aToB.q.x;
		aToB_aligned.q.q.y = s->pair.aToB.q.y;
		aToB_aligned.q.q.z = s->pair.aToB.q.z;
		aToB_aligned.q.q.w = s->pair.aToB.q.w;

		PxAlignedTransform_WriteWarp(&outBuffer[s->pair.cmIndex].mRelativeTransform, aToB_aligned);
	}

	outBuffer[s->pair.cmIndex].mNbManifolds = nbPatches;
}


extern "C" __global__
void convexTrimeshCorrelate(
	const ConvexMeshPair * PX_RESTRICT				pairs,
	PxReal ** PX_RESTRICT							depthsPtr,
	ConvexTriNormalAndIndex ** PX_RESTRICT			normalAndIndexPtr,
	const ConvexTriContacts ** PX_RESTRICT			contactsPtr,

	PxgPersistentContactMultiManifold * PX_RESTRICT	output,
	PxsContactManagerOutput * PX_RESTRICT			cmOutputs,
	PxU32* PX_RESTRICT								tempCountAndOffsetBuffer,
	const PxU32										numPairs,
	const PxReal									clusterBias,
	ConvexTriContact* PX_RESTRICT					tempConvexTriContacts,
	PxU32*											pTempContactIndex
)
{
	const int warpIndex = threadIdx.y;
	__shared__ char scratch[sizeof(Scratch) *  CORRELATE_WARPS_PER_BLOCK];
	correlate(
		pairs,
		normalAndIndexPtr,
		contactsPtr,
		depthsPtr,
		tempCountAndOffsetBuffer,
		((Scratch*)&scratch) + warpIndex,
		output,
		cmOutputs,
		numPairs,
		clusterBias,
		tempConvexTriContacts,
		pTempContactIndex
	);
}
