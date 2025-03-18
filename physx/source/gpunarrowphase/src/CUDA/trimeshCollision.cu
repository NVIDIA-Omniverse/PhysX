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
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"

#include "geometry/PxGeometry.h"

#include "PxgContactManager.h"
#include "PxsContactManagerState.h"
#include "PxgConvexConvexShape.h"
#include "PxsMaterialCore.h"
#include "PxgPersistentContactManifold.h"
#include "PxsTransformCache.h"

#include "PxContact.h"

#include "GuBV32.h"

#include "cudaNpCommon.h"

#include "contactPatchUtils.cuh"
#include "contactReduction.cuh"
#include "utils.cuh"
#include "manifold.cuh"
#include "triangleMesh.cuh"

#define PLANE_TRI_MAX_CONTACTS 6

using namespace physx;

extern "C" __host__ void initNarrowphaseKernels22() {}

//each block deal with one test
extern "C" __global__ 
__launch_bounds__(1024, 1) 
void trimeshPlaneNarrowphase(
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	PxsContactManagerOutput* PX_RESTRICT cmOutputs,
	PxgShape* PX_RESTRICT shapes,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* contactDistance,
	const PxsMaterialData* PX_RESTRICT materials,
	PxgPersistentContactMultiManifold* PX_RESTRICT multiManifold,
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
	const PxReal clusterTolerance)
{
	__shared__ PxU32 sContacts[(sizeof(PxVec3)/sizeof(PxU32)) * (WARP_SIZE + 1) * PLANE_TRI_MAX_CONTACTS];
	PxVec3* contacts = reinterpret_cast<PxVec3*>(sContacts);
	__shared__ PxReal separations[(WARP_SIZE+1)*PLANE_TRI_MAX_CONTACTS];
	__shared__ PxU32 counters[WARP_SIZE];

	__shared__ PxU32 retainedCount;

	__shared__ bool doFullContactGen;

	if (threadIdx.x == 0)
	{
		retainedCount = 0;
	}

	const PxU32 workIndex = blockIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	PxgShape trimeshShape, planeShape;
	PxU32 trimeshCacheRef, planeCacheRef;
	LoadShapePairWarp<PxGeometryType::eTRIANGLEMESH, PxGeometryType::ePLANE>(cmInputs, workIndex, shapes,
		trimeshShape, trimeshCacheRef, planeShape, planeCacheRef);

	PxsCachedTransform trimeshTransformCache = transformCache[trimeshCacheRef];
	PxsCachedTransform planeTransformCache = transformCache[planeCacheRef];
	const PxReal cDistance = contactDistance[trimeshCacheRef] + contactDistance[planeCacheRef];

	PxTransform planeTransform = planeTransformCache.transform;
	PxTransform trimeshTransform = trimeshTransformCache.transform;

	// Geometries : Triangle Mesh
	const PxU8 * trimeshGeomPtr = reinterpret_cast<const PxU8 *>(trimeshShape.hullOrMeshPtr);

	const Gu::BV32DataPacked* nodes;
	const float4 * trimeshVerts;
	uint4 counts = readTriangleMesh(trimeshGeomPtr, nodes, trimeshVerts);
	const PxU32 numVerts = counts.x;
	
	const PxU32 numIterationsRequired = (numVerts + blockDim.x - 1) / blockDim.x;


	PxVec3 min(PX_MAX_F32), max(-PX_MAX_F32);

	if (threadIndexInWarp < nodes->mNbNodes)
	{
		const PxVec4 min4 = nodes->mMin[threadIndexInWarp];
		const PxVec4 max4 = nodes->mMax[threadIndexInWarp];

		min = PxVec3(min4.x, min4.y, min4.z);
		max = PxVec3(max4.x, max4.y, max4.z);
	}

	minIndex(min.x, FULL_MASK, min.x); minIndex(min.y, FULL_MASK, min.y); minIndex(min.z, FULL_MASK, min.z);
	maxIndex(max.x, FULL_MASK, max.x); maxIndex(max.y, FULL_MASK, max.y); maxIndex(max.z, FULL_MASK, max.z);

	const float4 extents4_f = make_float4(max.x - min.x, max.y - min.y, max.z - min.z, 0.f)*0.5f;

	PxReal minMargin = calculatePCMConvexMargin(extents4_f, trimeshShape.scale.scale, toleranceLength) * 0.5f;
	const PxReal ratio = 0.1f;
	const PxReal breakingThresholdRatio = 0.5f;

	bool lostContacts = false;

	PxTransform trimeshToPlane = planeTransform.transformInv(trimeshTransform);

	const PxU32 warpIndex = threadIdx.x / 32;

	if (warpIndex == 0)
	{
		const bool invalidate = invalidateManifold(trimeshToPlane, multiManifold[workIndex], minMargin, ratio);

		if (!invalidate)
		{
			const PxReal projectBreakingThreshold = minMargin * breakingThresholdRatio;

			lostContacts = refreshManifolds(
				trimeshToPlane,
				projectBreakingThreshold,
				multiManifold + workIndex
			);
		}

		bool fullContactGen = invalidate || lostContacts;

		if (threadIdx.x == 0)
			doFullContactGen = fullContactGen;
	}

	
	//const PxU32 numThreadsRequired = numIterationsRequired * blockDim.x;



	PxVec3 worldNormal = planeTransform.q.getBasisVector0();

	__syncthreads();

	if (doFullContactGen)
	{
		for (PxU32 vertInd = 0; vertInd < numIterationsRequired; vertInd++)
		{

			PxU32 i = vertInd * blockDim.x + threadIdx.x;
			bool hasContact = false;
			PxVec3 worldPoint;
			PxReal separation = PX_MAX_F32;

			if (i < numVerts)
			{
				float4 point = trimeshVerts[i];

				PxVec3 tp = PxLoad3(point);
				PxVec3 p = vertex2Shape(tp, trimeshShape.scale.scale, trimeshShape.scale.rotation);

				//v in plane space
				const PxVec3 pInPlaneSpace = trimeshToPlane.transform(p);

				//separation / penetration
				//separation = pInPlaneSpace.x;// FSub(V3GetX(sphereCenterInPlaneSpace), radius);


				if (cDistance >= pInPlaneSpace.x)
				{
					//get the plane normal
					separation = pInPlaneSpace.x;
					worldPoint = trimeshTransform.transform(p);
					hasContact = true;
				}
			}

			int mask = __ballot_sync(FULL_MASK, hasContact);

			mask = contactReduce<true, true, PLANE_TRI_MAX_CONTACTS, false>(worldPoint, separation, worldNormal, mask, clusterTolerance);

			counters[warpIndex] = __popc(mask);

			hasContact = mask & (1 << threadIndexInWarp);

			__syncthreads();

			PxU32 counter = counters[threadIndexInWarp];

			PxU32 contactCount = warpScan<AddOpPxU32, PxU32>(FULL_MASK, counter);

			//exclusive runsum
			PxU32 contactWarpOffset = contactCount - counter;


			contactWarpOffset = __shfl_sync(FULL_MASK, contactWarpOffset, warpIndex);


			const PxU32 offset = warpScanExclusive(mask, threadIndexInWarp);

			if (hasContact)
			{
				const PxU32 index = offset + contactWarpOffset + retainedCount;
				contacts[index] = worldPoint;
				separations[index] = separation;
			}


			const PxU32 totalContacts = __shfl_sync(FULL_MASK, contactCount, 31) + retainedCount;
			__syncthreads();

			//Now do another loop with warp 0 doing reduction

			if (warpIndex == 0)
			{
				hasContact = false;
				for (PxU32 ind = 0; ind < totalContacts;)
				{
					PxU32 readMask = ~(__ballot_sync(FULL_MASK, hasContact));

					if (!hasContact)
					{
						PxU32 readIndex = warpScanExclusive(readMask, threadIndexInWarp) + ind;

						if (readIndex < totalContacts)
						{
							worldPoint = contacts[readIndex];
							separation = separations[readIndex];
							hasContact = true;
						}
					}

					mask = __ballot_sync(FULL_MASK, hasContact);
					ind += __popc(readMask);

					mask = contactReduce<true, true, PLANE_TRI_MAX_CONTACTS, false>(worldPoint, separation, worldNormal, mask, clusterTolerance);

					hasContact = mask & (1 << threadIndexInWarp);
				}

				__syncwarp();
				retainedCount = __popc(__ballot_sync(FULL_MASK, hasContact));				

				if (hasContact)
				{
					PxU32 offset = warpScanExclusive(mask, threadIndexInWarp);
					contacts[offset] = worldPoint;
					separations[offset] = separation;
				}
			}

			__syncthreads();

		}

		//Store to PCM...
		if (warpIndex == 0)
		{
			PxgPersistentContactMultiManifold& manifold = multiManifold[workIndex];
			if (retainedCount)
			{
				if (threadIdx.x == 0)
				{
					manifold.mNbManifolds = 1;

					manifold.mNbContacts[0] = retainedCount;

					manifold.mRelativeTransform = trimeshToPlane;
				}

				if (threadIdx.x < retainedCount)
				{
					
					PxgContact& contact = manifold.mContacts[0][threadIdx.x];
					contact.pointA = trimeshTransform.transformInv(contacts[threadIdx.x]);
					PxVec3 planePoint = planeTransform.transformInv(contacts[threadIdx.x]);
					planePoint.x = 0.f;// separations[threadIdx.x];
					contact.pointB = planePoint;
					contact.normal = PxVec3(1.f, 0.f, 0.f);
					contact.penetration = separations[threadIdx.x];
					contact.triIndex = 0xFFFFFFFF;
				}
			}
			else
				manifold.mNbManifolds = 0;
		}

	}
	else
	{
		//Load contacts out of PCM...
		if (warpIndex == 0)
		{
			PxgPersistentContactMultiManifold& manifold = multiManifold[workIndex];

			if (manifold.mNbManifolds != 0)
			{
				if (threadIdx.x < manifold.mNbContacts[0])
				{

					PxgContact& contact = manifold.mContacts[0][threadIdx.x];
					contacts[threadIdx.x] = trimeshTransform.transform(contact.pointA);
					separations[threadIdx.x] = contact.penetration;
				}

				if (threadIdx.x == 0)
					retainedCount = manifold.mNbContacts[0];
			}
		}

		__syncthreads();
	}
	
	

	//Retained counts stores the set of contacts we kept, which are stored in the first N 
	//elements of contacts and separations buffer

	if (warpIndex == 0)
	{

		PxU32 nbContacts = retainedCount;

		PxU32 contactByteOffset = 0xFFFFFFFF;
		PxU32 forceAndIndiceByteOffset = 0xFFFFFFFF;

		if (threadIndexInWarp == 0)
		{
			if (nbContacts)
			{
				contactByteOffset = atomicAdd(&(patchAndContactCounters->contactsBytes), sizeof(PxContact) * nbContacts);
				forceAndIndiceByteOffset = atomicAdd(&(patchAndContactCounters->forceAndIndiceBytes), sizeof(PxU32) * nbContacts);

				if ((contactByteOffset + sizeof(PxContact)* nbContacts) > contactBytesLimit)
				{
					patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::CONTACT_BUFFER_OVERFLOW);
					contactByteOffset = 0xFFFFFFFF; //overflow
				}
				
				if ((forceAndIndiceByteOffset + sizeof(PxU32)* nbContacts) > forceBytesLimit)
				{
					patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::FORCE_BUFFER_OVERFLOW);
					forceAndIndiceByteOffset = 0xFFFFFFFF; //overflow
				}
			}
		}

		contactByteOffset = __shfl_sync(FULL_MASK, contactByteOffset, 0);


		PxsContactManagerOutput* output = cmOutputs + workIndex;
		//write point and penetration to the contact stream
		if (contactByteOffset != 0xFFFFFFFF)
		{
			//*((float4 *)(contactStream + contactByteOffset)) = pointPen;

			//output->contactForces = reinterpret_cast<PxReal*>(startContactForces + forceAndIndiceByteOffset);
			//output->contactPoints = startContactPoints + contactByteOffset;
			
			float4* baseContactStream = (float4*)(contactStream + contactByteOffset);
			if (threadIndexInWarp < nbContacts)
			{
				float4& outPointPen = baseContactStream[threadIndexInWarp];
				outPointPen = make_float4(contacts[threadIndexInWarp].x, contacts[threadIndexInWarp].y,
					contacts[threadIndexInWarp].z, separations[threadIndexInWarp]);
			}

			if (threadIndexInWarp == 0)
			{
				output->contactForces = reinterpret_cast<PxReal*>(startContactForces + forceAndIndiceByteOffset);
				output->contactPoints = startContactPoints + contactByteOffset;
			}
		}
		else 
		{
			if (threadIndexInWarp == 0)
			{
				output->contactForces = NULL;
				output->contactPoints = NULL;
			}
			nbContacts = 0;
		}

		if (threadIndexInWarp == 0)
		{
			PxU32 patchIndex = registerContactPatch(cmOutputs, patchAndContactCounters, touchChangeFlags, patchChangeFlags, startContactPatches, patchBytesLimit, workIndex, nbContacts);

			if (nbContacts == 0)
				(cmOutputs + workIndex)->contactPatches = 0;

			insertIntoPatchStream(materials, patchStream, planeShape, trimeshShape, patchIndex, worldNormal, nbContacts);
		}
	}
}



//each block deal with one test
extern "C" __global__
__launch_bounds__(1024, 1)
void trimeshHeightfieldNarrowphase(
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	PxsContactManagerOutput* PX_RESTRICT cmOutputs,
	PxgShape* PX_RESTRICT shapes,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* contactDistance,
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
	const PxReal clusterBias)
{
	__shared__ char sContacts[sizeof(PxVec3) * (WARP_SIZE + 1) * PLANE_TRI_MAX_CONTACTS];
	PxVec3* contacts = reinterpret_cast<PxVec3*>(sContacts);

	__shared__ PxReal separations[(WARP_SIZE + 1)*PLANE_TRI_MAX_CONTACTS];
	__shared__ PxU32 counters[WARP_SIZE];

	__shared__ PxU32 retainedCount;

	if (threadIdx.x == 0)
	{
		retainedCount = 0;
	}

	__syncthreads();

	const PxU32 workIndex = blockIdx.x;

	PxgShape trimeshShape, heightfieldShape;
	PxU32 trimeshCacheRef, heightfieldCacheRef;
	LoadShapePairWarp<PxGeometryType::eTRIANGLEMESH, PxGeometryType::eHEIGHTFIELD>(cmInputs, workIndex, shapes,
		trimeshShape, trimeshCacheRef, heightfieldShape, heightfieldCacheRef);

	PxsCachedTransform trimeshTransformCache = transformCache[trimeshCacheRef];
	PxsCachedTransform heightfieldTransformCache = transformCache[heightfieldCacheRef];
	const PxReal cDistance = contactDistance[trimeshCacheRef] + contactDistance[heightfieldCacheRef];

	PxTransform trimeshTransform = trimeshTransformCache.transform;
	PxTransform heightfieldTransform = heightfieldTransformCache.transform;

	// Geometries : Triangle Mesh
	const PxU8 * trimeshGeomPtr = reinterpret_cast<const PxU8 *>(trimeshShape.hullOrMeshPtr);

	const Gu::BV32DataPacked* nodes;
	const float4 * trimeshVerts;
	uint4 counts = readTriangleMesh(trimeshGeomPtr, nodes, trimeshVerts);
	const PxU32 numVerts = counts.x;

	const PxU32 numIterationsRequired = (numVerts + blockDim.x - 1) / blockDim.x;

	//const PxU32 numThreadsRequired = numIterationsRequired * blockDim.x;

	const PxU32 threadIndexInWarp = threadIdx.x & 31;
	const PxU32 warpIndex = threadIdx.x / 32;

	PxTransform trimeshToHeightfield = heightfieldTransform.transformInv(trimeshTransform);

	PxVec3 worldNormal = heightfieldTransform.q.getBasisVector0();

	for (PxU32 ind = 0; ind < numIterationsRequired; ind++)
	{

		PxU32 i = ind * blockDim.x + threadIdx.x;
		bool hasContact = false;
		PxVec3 worldPoint;
		PxReal separation;

		if (i < numVerts)
		{
			float4 point = trimeshVerts[i];

			PxVec3 tp = PxLoad3(point);
			PxVec3 p = vertex2Shape(tp, trimeshShape.scale.scale, trimeshShape.scale.rotation);

			//v in plane space
			const PxVec3 pInHeightfieldSpace = trimeshToHeightfield.transform(p);

			//separation / penetration
			separation = pInHeightfieldSpace.x;


			if (cDistance >= pInHeightfieldSpace.x)
			{
				//get the plane normal
				worldPoint = trimeshTransform.transform(p);
				hasContact = true;
			}
		}

		int mask = __ballot_sync(FULL_MASK, hasContact);

		mask = contactReduce<true, true, PLANE_TRI_MAX_CONTACTS>(worldPoint, separation, worldNormal, mask, clusterBias);

		counters[warpIndex] = __popc(mask);


		__syncthreads();

		PxU32 contactCount = warpScan<AddOpPxU32, PxU32>(FULL_MASK, counters[threadIndexInWarp]);

		//exclusive runsum
		PxU32 contactWarpOffset = contactCount - counters[threadIndexInWarp];


		contactWarpOffset = __shfl_sync(FULL_MASK, contactWarpOffset, warpIndex);


		const PxU32 offset = warpScanExclusive(mask, threadIndexInWarp);

		if (hasContact)
		{
			const PxU32 index = offset + contactWarpOffset + retainedCount;
			contacts[index] = worldPoint;
			separations[index] = separation;
		}

		__syncthreads();

		const PxU32 totalContacts = __shfl_sync(FULL_MASK, contactCount, 31) + retainedCount;


		//Now do another loop with warp 0 doing reduction

		if (warpIndex == 0)
		{
			hasContact = false;
			for (PxU32 ind = 0; ind < totalContacts;)
			{
				PxU32 readMask = ~(__ballot_sync(FULL_MASK, hasContact));

				if (!hasContact)
				{
					PxU32 readIndex = warpScanExclusive(readMask, threadIndexInWarp);

					if ((ind + readIndex) < totalContacts)
					{
						worldPoint = contacts[readIndex];
						separation = separations[readIndex];
						hasContact = true;
					}
				}

				mask = __ballot_sync(FULL_MASK, hasContact);
				ind += __popc(readMask);
				mask = contactReduce<true, true, PLANE_TRI_MAX_CONTACTS>(worldPoint, separation, worldNormal, mask, clusterBias);

				hasContact = mask & (1 << threadIndexInWarp);
			}

			retainedCount = __popc(__ballot_sync(FULL_MASK, hasContact));

			if (hasContact)
			{
				PxU32 offset = warpScanExclusive(mask, threadIndexInWarp);
				contacts[offset] = worldPoint;
				separations[offset] = separation;
			}
		}

		__syncthreads();

	}



	//Retained counts stores the set of contacts we kept, which are stored in the first N 
	//elements of contacts and separations buffer

	if (warpIndex == 0)
	{

		const PxU32 nbContacts = retainedCount;

		PxU32 contactByteOffset = 0xFFFFFFFF;
		PxU32 forceAndIndiceByteOffset = 0xFFFFFFFF;

		if (threadIndexInWarp == 0)
		{
			if (nbContacts)
			{
				contactByteOffset = atomicAdd(&(patchAndContactCounters->contactsBytes), sizeof(PxContact) * nbContacts);
				forceAndIndiceByteOffset = atomicAdd(&(patchAndContactCounters->forceAndIndiceBytes), sizeof(PxU32) * nbContacts);

				if ((contactByteOffset + sizeof(PxContact)) > contactBytesLimit)
				{
					patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::CONTACT_BUFFER_OVERFLOW);
					contactByteOffset = 0xFFFFFFFF; //overflow
				}
				else if ((forceAndIndiceByteOffset + sizeof(PxU32)) > forceBytesLimit)
				{
					patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::FORCE_BUFFER_OVERFLOW);
					forceAndIndiceByteOffset = 0xFFFFFFFF; //overflow
				}
			}
		}

		contactByteOffset = __shfl_sync(FULL_MASK, contactByteOffset, 0);


		PxsContactManagerOutput* output = cmOutputs + workIndex;
		//write point and penetration to the contact stream
		if (contactByteOffset != 0xFFFFFFFF)
		{
			//*((float4 *)(contactStream + contactByteOffset)) = pointPen;

			//output->contactForces = reinterpret_cast<PxReal*>(startContactForces + forceAndIndiceByteOffset);
			//output->contactPoints = startContactPoints + contactByteOffset;

			float4* baseContactStream = (float4*)(contactStream + contactByteOffset);
			if (threadIndexInWarp < nbContacts)
			{
				float4& outPointPen = baseContactStream[threadIndexInWarp];
				outPointPen = make_float4(contacts[threadIndexInWarp].x, contacts[threadIndexInWarp].y,
					contacts[threadIndexInWarp].z, separations[threadIndexInWarp]);
			}

			if (threadIndexInWarp == 0)
			{
				output->contactForces = reinterpret_cast<PxReal*>(startContactForces + forceAndIndiceByteOffset);
				output->contactPoints = startContactPoints + contactByteOffset;
			}
		}
		else if (threadIndexInWarp == 0)
		{
			output->contactForces = NULL;
			output->contactPoints = NULL;
		}

		if (threadIndexInWarp == 0)
		{
			PxU32 patchIndex = registerContactPatch(cmOutputs, patchAndContactCounters, touchChangeFlags, patchChangeFlags, startContactPatches, patchBytesLimit, workIndex, nbContacts);
			
			insertIntoPatchStream(materials, patchStream, heightfieldShape, trimeshShape, patchIndex, worldNormal, nbContacts);
		}
	}
}

