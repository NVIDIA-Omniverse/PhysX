// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxAssert.h"
#include "foundation/PxVec3.h"
#include "PxContact.h"

#include "PxsMaterialCore.h"
#include "PxsContactManagerState.h"
#include "PxgPersistentContactManifold.h"
#include "PxgConvexConvexShape.h"
#include "PxgContactManager.h"
#include "cudaNpCommon.h"
#include "PxsCachedTransform.h"
#include "dataReadWriteHelper.cuh"
#include "materialCombiner.cuh"
#include "geometry/PxGeometry.h"

#include <assert.h>

using namespace physx;

extern "C" __host__ void initNarrowphaseKernels5() {}

__device__ void writeCompressedContact(
	PxContactPatch* PX_RESTRICT patches, PxU32 patchWriteIndex, PxU32 contactWriteIndex,
	const PxU32 nbContacts,
	const PxVec3 normal,
	const PxU32 materialIndex0, const PxU32 materialIndex1,
	const PxsMaterialData* PX_RESTRICT materials
)
{
	PxReal staticFriction, dynamicFriction, combinedRestitution, combinedDamping;
	PxU32 materialFlags;

	combineMaterials(materials, static_cast<PxU16>(materialIndex0), static_cast<PxU16>(materialIndex1),
		materialFlags, staticFriction, dynamicFriction,
		combinedRestitution, combinedDamping);

	PxContactPatch& patch = patches[patchWriteIndex];
	patch.normal = normal;
	patch.nbContacts = nbContacts;
	patch.startContactIndex = contactWriteIndex;
	//KS - we could probably compress this further into the header but the complexity might not be worth it
	patch.staticFriction = staticFriction;
	patch.dynamicFriction = dynamicFriction;
	patch.restitution = combinedRestitution;
	patch.damping = combinedDamping;
	patch.materialIndex0 = materialIndex0;
	patch.materialIndex1 = materialIndex1;
	patch.materialFlags = static_cast<PxU8>(materialFlags);
	patch.mMassModification.linear0 = 1.f;
	patch.mMassModification.linear1 = 1.f;
	patch.mMassModification.angular0 = 1.f;
	patch.mMassModification.angular1 = 1.f;
	patch.internalFlags = PxContactPatch::eHAS_FACE_INDICES;
}

__device__ inline float getAvgContactValue(float val, PxU32 nbContacts)
{
	PxReal avgValue = val;

	#pragma unroll 3
	for (PxU32 reductionRadius = 4; reductionRadius > 0; reductionRadius >>= 1)
	{
		PxReal tmp = __shfl_down_sync(FULL_MASK, avgValue, reductionRadius);
  
		if (reductionRadius < nbContacts)
			avgValue += tmp;
	}

	return avgValue / PxReal(nbContacts);
}

//calculate patch normal by average each normal and transform the normal into world space
__device__ static inline PxVec3 getWorldNormal(
	const PxgContact* contacts, const PxTransform& meshTransform,
	const PxU32 numContacts, const PxU32 manifoldIndex, const PxU32 threadIndexInWarp, const PxU32 threadIndexInManifold
)
{
	PxVec3 normal = (threadIndexInManifold >= numContacts) ? PxVec3(0, 0, 0) : contacts[0].normal;

	//transform into world space
	PxVec3 unNormalizedNormal = meshTransform.rotate(normal);

	return unNormalizedNormal.getNormalized();
}

//Each blocks has 8 warps and each warps deal with a pair
//max number of contacts in each PxgPersistentContactMultiManifold is 24(maxinum 4 single manifold and maxinum 6 contacts in each manifold )
extern "C" __global__
void convexTrimeshFinishContacts(
								const ConvexMeshPair *	pairs,
								const PxsCachedTransform * transformCache,
								const PxgShape* PX_RESTRICT gpuShapes,
								const PxgContactManagerInput * cmInputs,
								PxsContactManagerOutput * cmOutputs,
								const PxgPersistentContactMultiManifold* cmMultiManifold,
								const PxU32 numPairs,
								const PxsMaterialData* PX_RESTRICT materials,
								PxU8* PX_RESTRICT contactStream,//contact and pen
								PxU8* PX_RESTRICT patchStream, // normal and friction
								PxU8* PX_RESTRICT forceAndIndiceStream, //triangle(face) index is appended after the froce stream
								bool insertAveragePoint,
								PxgPatchAndContactCounters* PX_RESTRICT patchAndContactCounters,
								PxU32* touchChangeFlags,
								PxU32* patchChangeFlags,
								PxU8* startContactPatches,
								PxU8* startContactPoints,
								PxU8* startContactForces,
								PxU32 patchBytesLimit,
								PxU32 contactBytesLimit,
								PxU32 forceBytesLimit
	)
{
	const PxU32 globalThreadIndex = threadIdx.x + blockIdx.x * blockDim.x;

	const PxU32 globalWarpIndex = globalThreadIndex / WARP_SIZE;
	if(globalWarpIndex >= numPairs)
		return;

	
	const PxgPersistentContactMultiManifold& multiManifold = cmMultiManifold[globalWarpIndex];
	PxU32 nbManifolds = multiManifold.mNbManifolds;

	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);
	const PxU32 singleManifoldIndex = threadIndexInWarp / PXG_SUBMANIFOLD_MAX_CONTACTS;
	const PxU32 threadIndexInManifold = threadIndexInWarp % PXG_SUBMANIFOLD_MAX_CONTACTS;

	const PxU32 numContacts = (singleManifoldIndex >= nbManifolds) ? 0 : multiManifold.mNbContacts[singleManifoldIndex];

	const bool hasContacts = threadIndexInManifold < numContacts;

	PxU32 contactMask = __ballot_sync(FULL_MASK, (PxU32) hasContacts);
	PxU32 totalNumContacts = __popc(contactMask);
	
	const PxU32 writeIndex = warpScanExclusive(contactMask, threadIndexInWarp); //tells me how many threads preceding me had contacts (i.e. what my write index will be)

	
	//Each single manifold should be one patch
	PxU32 patchByteOffset = 0xFFFFFFFF;
	PxU32 contactByteOffset = 0xFFFFFFFF;
	PxU32 forceAndIndiceByteOffset = 0xFFFFFFFF;

	PxsContactManagerOutput* output = cmOutputs + globalWarpIndex;

	PxU32 allflags = reinterpret_cast<PxU32*>(&output->allflagsStart)[0];
	PxU8 oldStatusFlags = u16Low(u32High(allflags));
	PxU8 statusFlags = oldStatusFlags;

	statusFlags &= (~PxsContactManagerStatusFlag::eTOUCH_KNOWN);
	
	if (totalNumContacts != 0)
		statusFlags |= PxsContactManagerStatusFlag::eHAS_TOUCH;
	else
		statusFlags |= PxsContactManagerStatusFlag::eHAS_NO_TOUCH;

	PxU8 prevPatches = u16High(u32Low(allflags)); //Get out the current number of patches to store as the previous frame's number of patches
	
	bool overflow = false;


	// PT: this kernel does not support average points: it never writes one. It used to reserve & count one contact per
	// manifold anyway, which made totalNumContacts (and thus output->nbContacts) over-report by nbManifolds, leaked
	// uninitialized contacts/impulses into the contact reports, and - since totalNumContacts was only incremented on
	// lane 0 - made the faceIndex base pointer below differ between lane 0 and the rest of the warp. Average points are
	// only supported by the CPU codepath and by GPU convex-convex (see cudaGJKEPA.cu). The parameter is kept so the
	// launch signature stays in sync with the host code, and to leave the door open for a proper implementation.
	PX_UNUSED(insertAveragePoint);

	if (threadIndexInWarp == 0 )
	{
		if (totalNumContacts)
		{
			patchByteOffset = atomicAdd(&(patchAndContactCounters->patchesBytes), sizeof(PxContactPatch)*nbManifolds);
			contactByteOffset = atomicAdd(&(patchAndContactCounters->contactsBytes), sizeof(PxContact) * totalNumContacts);
			forceAndIndiceByteOffset = atomicAdd(&(patchAndContactCounters->forceAndIndiceBytes), sizeof(PxU32) * totalNumContacts * 2);

			if ((patchByteOffset + sizeof(PxContactPatch) * nbManifolds) > patchBytesLimit)
			{
				patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::PATCH_BUFFER_OVERFLOW);
				patchByteOffset = 0xFFFFFFFF;
				overflow = true;
			}
			else if ((contactByteOffset + sizeof(PxContact) * totalNumContacts) > contactBytesLimit)
			{
				patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::CONTACT_BUFFER_OVERFLOW);
				contactByteOffset = 0xFFFFFFFF;
				overflow = true;
			}
			else if ((forceAndIndiceByteOffset + sizeof(PxU32) * totalNumContacts * 2) > forceBytesLimit)
			{
				patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::FORCE_BUFFER_OVERFLOW);
				forceAndIndiceByteOffset = 0xFFFFFFFF;
				overflow = true;
			}

			if (overflow)
			{
				nbManifolds = totalNumContacts =  0;

				statusFlags &= (~PxsContactManagerStatusFlag::eTOUCH_KNOWN);
				statusFlags |= PxsContactManagerStatusFlag::eHAS_NO_TOUCH;
			}

			//printf("threadIdex0 nbManifolds %i\n", nbManifolds);
		}
	

		bool previouslyHadTouch = oldStatusFlags & PxsContactManagerStatusFlag::eHAS_TOUCH;
		bool prevTouchKnown = oldStatusFlags & PxsContactManagerStatusFlag::eTOUCH_KNOWN;
		bool currentlyHasTouch = nbManifolds != 0;

		const bool change = (previouslyHadTouch ^ currentlyHasTouch) || (!prevTouchKnown);
		touchChangeFlags[globalWarpIndex] = change;
		patchChangeFlags[globalWarpIndex] = (prevPatches != nbManifolds);
	
		assert(totalNumContacts < 100);
		reinterpret_cast<PxU32*>(&output->allflagsStart)[0] = merge(merge(prevPatches, statusFlags),
													merge(nbManifolds, PxU8(0)));  
		output->nbContacts = totalNumContacts;

		if (!overflow)
		{
			output->contactForces = reinterpret_cast<PxReal*>(startContactForces + forceAndIndiceByteOffset);
			output->contactPatches = startContactPatches + patchByteOffset;
			output->contactPoints = startContactPoints + contactByteOffset;
		}
		else
		{
			output->contactForces = 0;
			output->contactPatches = 0;
			output->contactPoints = 0;
		}
	}

	if(__any_sync(FULL_MASK, overflow) || nbManifolds == 0)
		return;


	//const uint4 npWorkItem = *(reinterpret_cast<const uint4*>(cmInputs) + globalWarpIndex);

	PxgContactManagerInput npWorkItem;
	PxgContactManagerInput_ReadWarp(npWorkItem, cmInputs, globalWarpIndex);

	PxU32 shapeRef0 = npWorkItem.shapeRef0;
	PxU32 shapeRef1 = npWorkItem.shapeRef1;
	PxU32 transformCacheRef0 = npWorkItem.transformCacheRef0;
	PxU32 transformCacheRef1 = npWorkItem.transformCacheRef1;

	bool flip = gpuShapes[shapeRef0].type == PxGeometryType::eTRIANGLEMESH;

	if (flip)
	{
		PxSwap(shapeRef0, shapeRef1);
		PxSwap(transformCacheRef0, transformCacheRef1);
	}

	PxgShape shape0;
	PxgShape_ReadWarp(shape0, gpuShapes + shapeRef0);

	const uint2 materialIndices = pairs[globalWarpIndex].materialIndices;

	patchByteOffset = __shfl_sync(FULL_MASK, (PxI32) patchByteOffset, 0);
	contactByteOffset = __shfl_sync(FULL_MASK, (PxI32) contactByteOffset, 0);
	forceAndIndiceByteOffset = __shfl_sync(FULL_MASK, (PxI32)forceAndIndiceByteOffset, 0);

	PxContactPatch* patches = reinterpret_cast<PxContactPatch*>(patchStream + patchByteOffset);
	float4* contacts = reinterpret_cast<float4*>(contactStream + contactByteOffset);
	PxU32* faceIndex = reinterpret_cast<PxU32*>(forceAndIndiceStream + forceAndIndiceByteOffset + totalNumContacts*sizeof(PxU32));

	PxsCachedTransform trimeshTransformCached; 
	PxsCachedTransform_ReadWarp(trimeshTransformCached, transformCache + transformCacheRef1); //w is transformCacheRef1

	PxsCachedTransform sphereTransformCached;
	PxsCachedTransform_ReadWarp(sphereTransformCached, transformCache + transformCacheRef0); //w is transformCacheRef1

	//calculate patch normal by average each normal and transform the normal into world space, need to optimized
	const PxVec3 worldNormal = getWorldNormal(&multiManifold.mContacts[singleManifoldIndex][0], trimeshTransformCached.transform, numContacts, singleManifoldIndex, threadIndexInWarp, threadIndexInManifold);

	if(hasContacts)
	{
		const PxgContact& point = multiManifold.mContacts[singleManifoldIndex][threadIndexInManifold];

		if(threadIndexInManifold == 0)
			writeCompressedContact(patches, singleManifoldIndex, writeIndex, numContacts, flip ? -worldNormal : worldNormal, materialIndices.x, materialIndices.y, materials);

		assert(point.pointB.isFinite());

		PxReal pen = point.penetration;
		PxVec3 worldPt;
		if (shape0.type == PxGeometryType::eSPHERE)
		{
			const PxReal radius = shape0.scale.scale.y;
			pen = pen - radius;
			worldPt = sphereTransformCached.transform.p - worldNormal * radius;
		}
		else if (shape0.type == PxGeometryType::eCAPSULE)
		{
			const PxReal radius = shape0.scale.scale.y;
			pen = pen - radius;
			worldPt = sphereTransformCached.transform.transform(point.pointA) - worldNormal * radius;
		}
		else
		{
			worldPt = trimeshTransformCached.transform.transform(point.pointB);
		}

		//contacts[writeIndex] = make_float4(point.pointB.x, point.pointB.y, point.pointB.z, -point.penetration);
		if (contactByteOffset != 0xFFFFFFFF)
			contacts[writeIndex] = make_float4(worldPt.x, worldPt.y, worldPt.z, pen);

		/*if (threadIndexInWarp == 0)
		{
			printf("triIndex %i worldPt(%f, %f, %f)\n", point.triIndex, worldPt.x, worldPt.y, worldPt.z);
			printf("triIndex %i worldNormal(%f, %f, %f)\n", point.triIndex, worldNormal.x, worldNormal.y, worldNormal.z);
			printf("triIndex %i pen %f radius %f\n", point.triIndex, pen, shape0.scale.scale.x);
			PxVec3 p = sphereTransformCached.transform.p;
			PxQuat q = sphereTransformCached.transform.q;
			printf("sphereTransform p(%f, %f, %f), q(%f, %f, %f, %f)\n", p.x, p.y, p.z, q.x, q.y, q.z, q.w);
		}*/

		if(forceAndIndiceByteOffset != 0xFFFFFFFF)
			faceIndex[writeIndex] = point.triIndex;
 	}
}

