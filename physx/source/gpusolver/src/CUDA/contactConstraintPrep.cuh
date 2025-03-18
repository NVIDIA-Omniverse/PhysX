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


#ifndef	__CONTACT_CONSTRAINT_PREP_CUH__
#define	__CONTACT_CONSTRAINT_PREP_CUH__


#include "PxgSolverBody.h"
#include "PxgConstraintBlock.h"
#include "PxgFrictionPatch.h"
#include "PxgConstraintPrep.h"
#include "PxgSolverConstraintDesc.h"
#include "cutil_math.h"
#include "PxgCudaMemoryAllocator.h"
#include "DySolverConstraintTypes.h"
#include "DyCpuGpuArticulation.h"
#include "PxMaterial.h"
#include "PxgSolverKernelIndices.h"
#include "PxgSolverFlags.h"
#include "MemoryAllocator.cuh"
#include "vector.cuh"
#include "PxgCommonDefines.h"
#include "constraintPrepShared.cuh"
#include "copy.cuh"

using namespace physx;



static __device__ bool getFrictionPatches(PxgFrictionPatch&  frictionPatch,
	PxgFrictionAnchorPatch& anchorPatch,
	const PxgBlockFrictionIndex* PX_RESTRICT prevFrictionIndices,
	const PxU32 prevFrictionStartIndex,
	const PxgFrictionPatch* PX_RESTRICT previousPatches,
	const PxgFrictionAnchorPatch* PX_RESTRICT previousAnchors,
	PxU32 frictionPatchCount,
	const PxAlignedTransform& bodyFrame0,
	const PxAlignedTransform& bodyFrame1,
	PxReal correlationDistance,
	const PxU32 totalNbEdges,
	PxReal& patchExtents,
	const PxU32 threadIndexInWarp)
{
	if (prevFrictionStartIndex == 0xFFFFFFFF || frictionPatchCount == 0)
		return true;

	PxgFrictionPatch& newPatch = frictionPatch;
	PxgFrictionAnchorPatch& newAnchor = anchorPatch;

	for (PxU32 a = 0; a < frictionPatchCount; a++)
	{
		const PxU64 index = prevFrictionIndices[prevFrictionStartIndex + a*totalNbEdges].getPatchIndex();
		//indices += totalNbEdges;
		const PxgFrictionPatch& oldPatch = previousPatches[index];
		const PxgFrictionAnchorPatch& oldAnchor = previousAnchors[index];

		assert(oldPatch.broken == 0 || oldPatch.broken == 1);
		if (!oldPatch.broken)
		{
			const float4 oldBody0Normal = oldPatch.body0Normal;
			if (dot3(oldBody0Normal, newPatch.body0Normal) > PXC_SAME_NORMAL) //TODO - check that they're the same material!
			{
				const PxU8 anchorCount = oldPatch.anchorCount;
				if (anchorCount != 0)
				{
					assert(anchorCount <= 2);

					const PxAlignedTransform body1ToBody0 = bodyFrame0.transformInv(bodyFrame1);
					const float4 oldBody1Normal = oldPatch.body1Normal;
					const float result = dot3(oldBody0Normal, body1ToBody0.rotate(oldBody1Normal));
					if (dot3(oldBody0Normal, body1ToBody0.rotate(oldBody1Normal)) > PXC_SAME_NORMAL)
					{

						const float4 body0Anchor0 = oldAnchor.body0Anchors[0];
						const float4 body1Anchor0 = oldAnchor.body1Anchors[0];
						if (pointsAreClose(body1ToBody0, body0Anchor0, body1Anchor0, oldBody0Normal, correlationDistance))
						{
							const float4 body0Anchor1 = oldAnchor.body0Anchors[1];
							const float4 body1Anchor1 = oldAnchor.body1Anchors[1];
							if (anchorCount < 2 || pointsAreClose(body1ToBody0, body0Anchor1, body1Anchor1, oldBody0Normal, correlationDistance))
							{
								newPatch.contactID[0] = 0xff;
								newPatch.contactID[1] = 0xff;
								newPatch.anchorCount = anchorCount;
								newPatch.body0Normal = oldBody0Normal;
								newPatch.body1Normal = oldBody1Normal;
								newAnchor.body0Anchors[0] = body0Anchor0;
								newAnchor.body0Anchors[1] = body0Anchor1;
								newAnchor.body1Anchors[0] = body1Anchor0;
								newAnchor.body1Anchors[1] = body1Anchor1;

								const float4 ext = (body0Anchor0 - body0Anchor1);
								patchExtents = ext.x*ext.x + ext.y*ext.y + ext.z*ext.z;
								return true; //Found a match = terminate!
							}
						}
					}
				}
			}
		}
	}



	return true;
}

static __device__ void growPatches(PxgFrictionPatch& fp, PxgFrictionAnchorPatch& fAnchor,
	const physx::PxgContactPoint* msContacts, const PxU32 numContacts,
	const physx::PxTransform& msBodyFrame0,
	const physx::PxTransform& msBodyFrame1,
	float frictionOffsetThreshold,
	const PxReal anchorSqDistance,
	const float minimum,			//PxBounds3
	const float maximum,			//PxBounds3
	ScratchMemoryAllocator& sAlloc,
	const PxU32 threadIndexInWarp)
{
	using namespace physx;
	PxU32 oldAnchorCount = fp.anchorCount;

	ScratchMemoryMarker marker(sAlloc);


	if (oldAnchorCount == 2)
	{
		float dif = 0.f;
		if (threadIndexInWarp < 3)
		{
			dif = maximum - minimum;
			dif = dif * dif;
		}

		const PxReal frictionPatchDiagonalSq = __shfl_sync(FULL_MASK, dif, 0)
			+ __shfl_sync(FULL_MASK, dif, 1)
			+ __shfl_sync(FULL_MASK, dif, 2);

		//If the squared distance between the anchors is more than a quarter of the patch diagonal, we can keep, 
		//otherwise the anchors are potentially clustered around a corner so force a rebuild of the patch
		if ((anchorSqDistance * 4.f) >= frictionPatchDiagonalSq)
			return;

		oldAnchorCount = 0;

	}

	//__shared__ PxVec3 worldAnchors[2];
	//__shared__ PxU32 contactID[2];

	PxVec3* msWorldAnchors = sAlloc.allocAligned<PxVec3>(sizeof(PxVec3) * 2);
	PxU32* msContactID = sAlloc.allocAligned<PxU32>(sizeof(PxU32) * 2);

	PxU16 anchorCount = 0;
	PxReal pointDistSq = 0.0f, dist0, dist1;

	// if we have an anchor already, keep it
	if (oldAnchorCount == 1)
	{

		float v = 0.f;
		if (threadIndexInWarp < 3)
		{
			float* anchors = reinterpret_cast<float*>(&fAnchor.body0Anchors[0].x);
			v = anchors[threadIndexInWarp];
		}

		transform(v, msBodyFrame0, msWorldAnchors[0], threadIndexInWarp);


		if (threadIndexInWarp == 0)
			msContactID[0] = 0xFF;

		/*const PxVec3 v(fAnchor.body0Anchors[0].x, fAnchor.body0Anchors[0].y, fAnchor.body0Anchors[0].z);
		worldAnchors[0] = bodyFrame0.transform(v);
		contactID[0] = 0xFF;*/
		anchorCount++;
	}

	__syncwarp();

	//PxVec3& msWorldPoint = *sAlloc.alloc<PxVec3>(sizeof(PxVec3));

	for (PxU32 j = 0; j<numContacts; j++)
	{
		const PxReal separation = msContacts[j].point_separationW.w;

		if (separation < frictionOffsetThreshold)
		{
			//const float* contacts = reinterpret_cast<const float*>(&msContacts[j].point_separationW.x);
			const PxVec3& worldPoint = reinterpret_cast<const PxVec3&>(msContacts[j].point_separationW.x);
			switch (anchorCount)
			{
			case 0:
				if (threadIndexInWarp < 3)
				{
					msWorldAnchors[0][threadIndexInWarp] = worldPoint[threadIndexInWarp];
					if (threadIndexInWarp == 0)
						msContactID[0] = PxU16(j);
				}
				anchorCount++;
				__syncwarp();
				/*contactID[0] = PxU16(j);
				worldAnchors[0] = worldPoint;
				anchorCount++;*/
				break;
			case 1:
				//pointDistSq = (worldPoint - worldAnchors[0]).magnitudeSquared();
				pointDistSq = negateMagnitudeSquared(worldPoint, msWorldAnchors[0], threadIndexInWarp);
				if (pointDistSq > 1e-8f)
				{
					if (threadIndexInWarp < 3)
					{
						msWorldAnchors[1][threadIndexInWarp] = worldPoint[threadIndexInWarp];
						if (threadIndexInWarp == 0)
							msContactID[1] = PxU16(j);
					}
					anchorCount++;

					__syncwarp();

				}

				break;
			default: //case 2
				dist0 = negateMagnitudeSquared(worldPoint, msWorldAnchors[0], threadIndexInWarp);
				dist1 = negateMagnitudeSquared(worldPoint, msWorldAnchors[1], threadIndexInWarp);

				//dist0 = (worldPoint - worldAnchors[0]).magnitudeSquared();
				//dist1 = (worldPoint - worldAnchors[1]).magnitudeSquared();
				if (dist0 > dist1)
				{
					if (dist0 > pointDistSq)
					{
						if (threadIndexInWarp < 3)
						{
							msWorldAnchors[1][threadIndexInWarp] = worldPoint[threadIndexInWarp];
							if (threadIndexInWarp == 0)
								msContactID[1] = PxU16(j);
						}
						//contactID[1] = PxU16(j);
						//worldAnchors[1] = worldPoint;
						pointDistSq = dist0;

						__syncwarp();
					}
				}
				else if (dist1 > pointDistSq)
				{
					if (threadIndexInWarp < 3)
					{
						msWorldAnchors[0][threadIndexInWarp] = worldPoint[threadIndexInWarp];
						if (threadIndexInWarp == 0)
							msContactID[0] = PxU16(j);
					}
					/*contactID[0] = PxU16(j);
					worldAnchors[0] = worldPoint;*/
					pointDistSq = dist1;

					__syncwarp();
				}
			}
		}
	}



	switch (anchorCount)
	{
	case 2:
	{
		//KS - if there is a 2nd anchor, we always write it. If we already had 2 anchors, we would have exited earlier!

		transformInv(msWorldAnchors[1], msBodyFrame0, reinterpret_cast<PxVec3&>(fAnchor.body0Anchors[1]), threadIndexInWarp);
		transformInv(msWorldAnchors[1], msBodyFrame1, reinterpret_cast<PxVec3&>(fAnchor.body1Anchors[1]), threadIndexInWarp);

	}
	case 1:
		if (oldAnchorCount == 0)
		{
			//KS - if there is a 2nd anchor, we always write it. If we already had 2 anchors, we would have exited earlier!
			transformInv(msWorldAnchors[0], msBodyFrame0, reinterpret_cast<PxVec3&>(fAnchor.body0Anchors[0]), threadIndexInWarp);
			transformInv(msWorldAnchors[0], msBodyFrame1, reinterpret_cast<PxVec3&>(fAnchor.body1Anchors[0]), threadIndexInWarp);
		}
	default:
		break;
	};

	if (threadIndexInWarp == 0)
		fp.anchorCount = anchorCount;

	__syncwarp();
}

static __device__ void initFrictionPatch(physx::PxgFrictionPatch& p,
	const float msBody0Normal, const float msBody1Normal,
	const PxU32 threadIndexInWarp)
{
	if (threadIndexInWarp < 3)
	{
		float* dBody0Normal = &p.body0Normal.x;
		dBody0Normal[threadIndexInWarp] = msBody0Normal;

		float* dBody1Normal = &p.body1Normal.x;
		dBody1Normal[threadIndexInWarp] = msBody1Normal;

		if (threadIndexInWarp == 0)
		{
			dBody0Normal[3] = 0.f;
			dBody1Normal[3] = 0.f;
			p.anchorCount = 0;
			p.broken = 0;
		}
	}
	/*p.body0Normal = make_float4(body0Normal.x, body0Normal.y, body0Normal.z, 0.f);
	p.body1Normal = make_float4(body1Normal.x, body1Normal.y, body1Normal.z, 0.f);
	p.anchorCount = 0;
	p.broken = 0;*/
}


static __device__ void correlatePatches(PxgFrictionPatch& frictionPatch, const physx::PxgContactPoint* contacts, const PxU32 nbContacts,
	const float msNormal, const physx::PxAlignedTransform& msBodyFrame0, const physx::PxAlignedTransform& msBodyFrame1,
	float normalTolerance, ScratchMemoryAllocator& sAlloc, const PxU32 threadIndexInWarp)
{
	using namespace physx;

	if (nbContacts > 0)
	{

		const PxQuat& quat0 = reinterpret_cast<const PxQuat&>(msBodyFrame0.q);
		const PxQuat& quat1 = reinterpret_cast<const PxQuat&>(msBodyFrame1.q);

		float msBody0Normal = rotateInvR(msNormal, quat0, threadIndexInWarp);
		float msBody1Normal = rotateInvR(msNormal, quat1, threadIndexInWarp);

		initFrictionPatch(frictionPatch, msBody0Normal, msBody1Normal, threadIndexInWarp);

		__syncwarp();
	}
}

#endif