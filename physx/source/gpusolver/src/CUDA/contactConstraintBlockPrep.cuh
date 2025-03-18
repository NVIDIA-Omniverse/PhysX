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


#ifndef	__CONTACT_CONSTRAINT_BLOCK_PREP_CUH__
#define	__CONTACT_CONSTRAINT_BLOCK_PREP_CUH__


#include "PxgSolverBody.h"
#include "PxgConstraintBlock.h"
#include "PxgFrictionPatch.h"
#include "PxgConstraintPrep.h"
#include "PxgSolverConstraintDesc.h"
#include "cutil_math.h"
#include "PxgSolverFlags.h"
#include "constraintPrepShared.cuh"
#include "PxRigidBody.h"
#include "assert.h"

using namespace physx;

static __device__ void initFrictionPatch(physx::PxgBlockFrictionPatch& p, const PxU32 threadIndex, const physx::PxVec3& body0Normal, const physx::PxVec3& body1Normal)
{
	p.body0Normal[threadIndex] = make_float4(body0Normal.x, body0Normal.y, body0Normal.z, 0.f);
	p.body1Normal[threadIndex] = make_float4(body1Normal.x, body1Normal.y, body1Normal.z, 0.f);
	p.anchorCount[threadIndex] = 0;
	p.broken[threadIndex] = 0;
}

static __device__ void correlatePatches(PxgBlockFrictionPatch& frictionPatch, const physx::PxgBlockContactPoint* contacts, const PxU32 nbContacts,
							const PxVec3& normal, const physx::PxAlignedTransform& bodyFrame0, const physx::PxAlignedTransform& bodyFrame1,
							float normalTolerance, const PxU32 threadIndex)
{
	using namespace physx;

	if(nbContacts > 0)
	{
		//printf("NbContacts = %i\n", nbContacts);
		//printf("FrictionPatch = %p\n", &frictionPatch);
		//PxgCorrelationBuffer::ContactPatchData &c = fb->contactPatches[0];
		PxVec3 body0Normal = bodyFrame0.rotateInv(normal);
		PxVec3 body1Normal = bodyFrame1.rotateInv(normal);

		frictionPatch.contactID[0][threadIndex] = 0xff;
		frictionPatch.contactID[1][threadIndex]= 0xff;
		initFrictionPatch(frictionPatch, threadIndex,
			body0Normal,body1Normal);
	}
}

static __device__ void growPatches(PxgBlockFrictionPatch& fp, PxgBlockFrictionAnchorPatch& fAnchor,
	const physx::PxgBlockContactPoint* contacts, const PxU32 numContacts,
				const physx::PxAlignedTransform& bodyFrame0,
				 const physx::PxAlignedTransform& bodyFrame1,
				 float frictionOffsetThreshold,
				 const PxU32 threadIndex,
				 const PxReal anchorSqDistance)
{
	using namespace physx;

	PxBounds3 pointBounds = PxBounds3::empty();
	for (PxU32 i = 0; i < numContacts; ++i)
	{
		float4 cp = contacts[i].point_separationW[threadIndex];
		pointBounds.minimum.x = PxMin(cp.x, pointBounds.minimum.x);
		pointBounds.minimum.y = PxMin(cp.y, pointBounds.minimum.y);
		pointBounds.minimum.z = PxMin(cp.z, pointBounds.minimum.z);

		pointBounds.maximum.x = PxMax(cp.x, pointBounds.maximum.x);
		pointBounds.maximum.y = PxMax(cp.y, pointBounds.maximum.y);
		pointBounds.maximum.z = PxMax(cp.z, pointBounds.maximum.z);
	}

	PxU32 oldAnchorCount = fp.anchorCount[threadIndex];

	if (oldAnchorCount == 2)
	{
		const PxReal frictionPatchDiagonalSq = pointBounds.getDimensions().magnitudeSquared();

		//If the squared distance between the anchors is more than a quarter of the patch diagonal, we can keep, 
		//otherwise the anchors are potentially clustered around a corner so force a rebuild of the patch
		if ((anchorSqDistance * 4.f) >= frictionPatchDiagonalSq)
			return;

		oldAnchorCount = 0;
	}

	PxVec3 worldAnchors[2];
	PxU32 contactID[2];

	PxU16 anchorCount = 0;
	PxReal pointDistSq = 0.0f, dist0, dist1;

	// if we have an anchor already, keep it
	if(oldAnchorCount == 1)
	{
		const PxVec3 v(fAnchor.body0Anchors[0][threadIndex].x, fAnchor.body0Anchors[0][threadIndex].y, fAnchor.body0Anchors[0][threadIndex].z);
		worldAnchors[0] = bodyFrame0.transform(v);
		contactID[0] = 0xFF;
		anchorCount++;
	}

	const PxReal eps = 1e-8f;

	for(PxU32 j=0;j<numContacts;j++)
	{
		const float4 worldPoint4_separationW = contacts[j].point_separationW[threadIndex];
			

		if(worldPoint4_separationW.w < frictionOffsetThreshold)
		{
			const PxVec3 worldPoint(worldPoint4_separationW.x, worldPoint4_separationW.y, worldPoint4_separationW.z);
			switch(anchorCount)
			{
			case 0:
				contactID[0] = PxU16(j);
				worldAnchors[0] = worldPoint;
				anchorCount++;
				break;
			case 1:
				pointDistSq = (worldPoint - worldAnchors[0]).magnitudeSquared();
				if (pointDistSq > eps)
				{
					contactID[1] = j;
					worldAnchors[1] = worldPoint;
					anchorCount++;
				}
				break;
			default: //case 2
				dist0 = (worldPoint - worldAnchors[0]).magnitudeSquared();
				dist1 = (worldPoint - worldAnchors[1]).magnitudeSquared();
				if (dist0 > dist1)
				{
					if(dist0 > pointDistSq)
					{
						contactID[1] = PxU16(j);
						worldAnchors[1] = worldPoint;
						pointDistSq = dist0;
					}
				}
				else if (dist1 > pointDistSq)
				{
					contactID[0] = PxU16(j);
					worldAnchors[0] = worldPoint;
					pointDistSq = dist1;
				}
			}
		}
	}

	// add the new anchor(s) to the patch

	switch(anchorCount)
	{
	case 2:
		{
			//KS - if there is a 2nd anchor, we always write it. If we already had 2 anchors, we would have exited earlier!
			const PxVec3 localPoint0 = bodyFrame0.transformInv(worldAnchors[1]);
			const PxVec3 localPoint1 = bodyFrame1.transformInv(worldAnchors[1]);
			fAnchor.body0Anchors[1][threadIndex] = make_float4(localPoint0.x, localPoint0.y, localPoint0.z, 0.f);
			fAnchor.body1Anchors[1][threadIndex] = make_float4(localPoint1.x, localPoint1.y, localPoint1.z, 0.f);
			fp.contactID[1][threadIndex] = contactID[1];
		}
	case 1:
		if(oldAnchorCount == 0)
		{
			//KS - if there is a 2nd anchor, we always write it. If we already had 2 anchors, we would have exited earlier!
			const PxVec3 localPoint0 = bodyFrame0.transformInv(worldAnchors[0]);
			const PxVec3 localPoint1 = bodyFrame1.transformInv(worldAnchors[0]);
			fAnchor.body0Anchors[0][threadIndex] = make_float4(localPoint0.x, localPoint0.y, localPoint0.z, 0.f);
			fAnchor.body1Anchors[0][threadIndex] = make_float4(localPoint1.x, localPoint1.y, localPoint1.z, 0.f);
			fp.contactID[0][threadIndex] = contactID[0];
		}
	default:
		break;
	};

	fp.anchorCount[threadIndex] = anchorCount;
	
}


__device__ PX_FORCE_INLINE bool isSeparated(const PxgBlockFrictionPatch& patch, const PxgBlockFrictionAnchorPatch& fAnchors, const PxU32 threadIndex, const PxAlignedTransform& body1ToBody0, const PxReal correlationDistance)
{
	assert(patch.anchorCount[threadIndex] <= 2);
	const float4 body0Normal = patch.body0Normal[threadIndex];
	for(PxU32 a = 0; a < patch.anchorCount[threadIndex]; ++a)
	{
		const float4 body0Anchor = fAnchors.body0Anchors[a][threadIndex];
		const float4 body1Anchor = fAnchors.body1Anchors[a][threadIndex];
		if(!pointsAreClose(body1ToBody0, body0Anchor, body1Anchor, body0Normal, correlationDistance))
			return true;
	}
	return false;
}


static __device__ bool getFrictionPatches(PxgBlockFrictionPatch&  frictionPatch, 
						PxgBlockFrictionAnchorPatch& anchorPatch,
						const PxgBlockFrictionIndex* PX_RESTRICT prevFrictionIndices,
						const PxU32 prevFrictionStartIndex,
						const PxgBlockFrictionPatch* PX_RESTRICT previousPatches,
						const PxgBlockFrictionAnchorPatch* PX_RESTRICT previousAnchors,
						PxU32 frictionPatchCount,
						const PxAlignedTransform& bodyFrame0,
						const PxAlignedTransform& bodyFrame1,
						PxReal correlationDistance,
						const PxU32 threadIndex,
						const PxU32 totalNbEdges,
						PxReal& patchExtents,
						const PxU32 nbContacts)
{
	//printf("prevFrictionStartIndex = %i, frictionPatchCount = %i\n", prevFrictionStartIndex, frictionPatchCount);
	if(prevFrictionStartIndex == 0xFFFFFFFF || frictionPatchCount == 0 || !prevFrictionIndices)
		return true;

	//const PxgBlockFrictionIndex* indices = &prevFrictionIndices[prevFrictionStartIndex];

	PxgBlockFrictionPatch& newPatch = frictionPatch; 
	PxgBlockFrictionAnchorPatch& newAnchor = anchorPatch;

	//while(frictionPatchCount--)
	for(PxU32 a = 0; a < frictionPatchCount; ++a)
	{
		const PxgBlockFrictionIndex index = prevFrictionIndices[prevFrictionStartIndex + a*totalNbEdges];
		const PxU32 oldThreadId = index.getThreadIdx();
		const PxU64 patchIndex = index.getPatchIndex();
		//indices += totalNbEdges;
		const PxgBlockFrictionPatch& oldPatch = previousPatches[patchIndex];
		const PxgBlockFrictionAnchorPatch& oldAnchor = previousAnchors[patchIndex];
		

		assert (oldPatch.broken[oldThreadId] == 0 || oldPatch.broken[oldThreadId] == 1);
		if(!oldPatch.broken[oldThreadId])
		{
			const float4 oldBody0Normal = oldPatch.body0Normal[oldThreadId];
			if(dot3(oldBody0Normal, newPatch.body0Normal[threadIndex]) > PXC_SAME_NORMAL) //TODO - check that they're the same material!
			{
				const PxU8 anchorCount = oldPatch.anchorCount[oldThreadId];
				if(anchorCount != 0)
				{
					assert(anchorCount <= 2);

					if (anchorCount <= nbContacts) //KS - if we have more anchors than contacts, we need to throw away the patch and rebuild
					{

						const PxAlignedTransform body1ToBody0 = bodyFrame0.transformInv(bodyFrame1);
						const float4 oldBody1Normal = oldPatch.body1Normal[oldThreadId];
						const float result = dot3(oldBody0Normal, body1ToBody0.rotate(oldBody1Normal));
						if (dot3(oldBody0Normal, body1ToBody0.rotate(oldBody1Normal)) > PXC_SAME_NORMAL)
						{

							const float4 body0Anchor0 = oldAnchor.body0Anchors[0][oldThreadId];
							const float4 body1Anchor0 = oldAnchor.body1Anchors[0][oldThreadId];
							if (anchorCount == 0 || pointsAreClose(body1ToBody0, body0Anchor0, body1Anchor0, oldBody0Normal, correlationDistance))
							{
								const float4 body0Anchor1 = oldAnchor.body0Anchors[1][oldThreadId];
								const float4 body1Anchor1 = oldAnchor.body1Anchors[1][oldThreadId];
								if (anchorCount < 2 || pointsAreClose(body1ToBody0, body0Anchor1, body1Anchor1, oldBody0Normal, correlationDistance))
								{
									/*	c.contactID[0] = 0xff;
										c.contactID[1] = 0xff;*/
									newPatch.contactID[0][threadIndex] = 0xff;
									newPatch.contactID[1][threadIndex] = 0xff;
									newPatch.anchorCount[threadIndex] = anchorCount;
									newPatch.body0Normal[threadIndex] = oldBody0Normal;
									newPatch.body1Normal[threadIndex] = oldBody1Normal;
									newAnchor.body0Anchors[0][threadIndex] = body0Anchor0;
									newAnchor.body0Anchors[1][threadIndex] = body0Anchor1;
									newAnchor.body1Anchors[0][threadIndex] = body1Anchor0;
									newAnchor.body1Anchors[1][threadIndex] = body1Anchor1;
									const float4 ext = (body0Anchor0 - body0Anchor1);
									patchExtents = ext.x*ext.x + ext.y*ext.y + ext.z*ext.z;
									//printf("Keeping patch with %i anchors\n", anchorCount);
									return true; //Found a match = terminate!
								}
							}
						}
					}
				}
			}
		}
	}
	return true;
}




static __device__ void setupFinalizeSolverConstraintsBlock(PxgBlockContactData& contactData,
									const PxgBlockContactPoint* PX_RESTRICT contacts,
									const PxU32 contactCount,
									const PxgBlockFrictionPatch& frictionPatch,
									const PxgBlockFrictionAnchorPatch& fAnchor,
									const PxAlignedTransform& bodyFrame0,
									const PxAlignedTransform& bodyFrame1,
									const PxgSolverBodyData& data0,
									const PxgSolverBodyData& data1,
									const float4& initialLinVel0_invMassW,
									const float4& initialAngVel0_penBiasClamp,
									const float4& initialLinVel1_invMassW,
									const float4& initialAngVel1_penBiasClamp,
									const PxMat33& sqrtInvInertia0,
									const PxMat33& sqrtInvInertia1,
									const PxReal invDtF32,
									const PxReal dtF32,
									PxReal bounceThresholdF32,
									const PxU32 threadIndex,
									PxU32 forceWritebackBufferOffset,
									const bool perPointFriction,
									PxgBlockSolverContactHeader* PX_RESTRICT contactHeader,
									PxgBlockSolverFrictionHeader* PX_RESTRICT frictionHeader,
									PxgBlockSolverContactPoint* PX_RESTRICT contactConstraints,
									PxgBlockSolverContactFriction* PX_RESTRICT frictionConstraints,
									const PxgMaterialContactData& data,
									const PxReal ccdMaxSeparationThreshold,
									const PxReal solverOffsetSlop)
{
	using namespace physx;
	// NOTE II: the friction patches are sparse (some of them have no contact patches, and
	// therefore did not get written back to the cache) but the patch addresses are dense,
	// corresponding to valid patches

	const PxU8 flags = data.mSolverFlags; // encoding according to PxgSolverContactFlags

	const float restDistance = data.restDistance;//FLoad(n.restDistance); 

	const PxAlignedQuat bodyFrame0q(bodyFrame0.q);//QuatVLoadU(&bodyFrame0.q.x);
	const PxVec3 bodyFrame0p(bodyFrame0.p.x, bodyFrame0.p.y, bodyFrame0.p.z);
	
	const PxAlignedQuat bodyFrame1q(bodyFrame1.q);//QuatVLoadU(&bodyFrame1.q.x);
	const PxVec3 bodyFrame1p(bodyFrame1.p.x, bodyFrame1.p.y, bodyFrame1.p.z);

	uint32_t frictionPatchWritebackAddrIndex = 0;
	uint32_t contactWritebackCount = 0;

	//TODO - fix up!!!!
	bool isCCD = (data0.flags | data1.flags) & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD;
	const PxReal ccdMaxSeparation = isCCD ? ccdMaxSeparationThreshold : PX_MAX_F32;


	const float maxPenBias = fmaxf(initialAngVel0_penBiasClamp.w, initialAngVel1_penBiasClamp.w);//FMax(FLoad(data0.penBiasClamp), FLoad(data1.penBiasClamp));

	const PxVec3 linVel0(initialLinVel0_invMassW.x, initialLinVel0_invMassW.y, initialLinVel0_invMassW.z);//V3LoadU(data0.initialLinVel);
	const PxVec3 linVel1(initialLinVel1_invMassW.x, initialLinVel1_invMassW.y, initialLinVel1_invMassW.z);//V3LoadU(data1.initialLinVel);
	const PxVec3 angVel0(initialAngVel0_penBiasClamp.x, initialAngVel0_penBiasClamp.y, initialAngVel0_penBiasClamp.z);//V3LoadU(data0.initialAngVel);
	const PxVec3 angVel1(initialAngVel1_penBiasClamp.x, initialAngVel1_penBiasClamp.y, initialAngVel1_penBiasClamp.z);//V3LoadU(data1.initialAngVel);

	//KS - TODO - Get this efficiently in shared memory/registers somehow!
	const PxMat33 invSqrtInertia0 = sqrtInvInertia0;
	const PxMat33 invSqrtInertia1 = sqrtInvInertia1;

	const float invDt = invDtF32;
	const float p8 = 0.8f;
	const float bounceThreshold = bounceThresholdF32;

	const float invDtp8 = invDt * p8;//FMul(invDt, p8);

	const float4 lin0X_ang0Y_lin1Z_ang1W = contactData.mInvMassScale[threadIndex].lin0X_ang0Y_lin1Z_ang1W;

	const float invMass0D0 = initialLinVel0_invMassW.w * lin0X_ang0Y_lin1Z_ang1W.x;
	const float invMass1D1 = initialLinVel1_invMassW.w * lin0X_ang0Y_lin1Z_ang1W.z;

	{
		PxU32 anchorCount = frictionPatch.anchorCount[threadIndex] * 2;
	
		//shared memory counter for max contacts in the friction patch...

		if(contactCount == 0)
		{
			frictionHeader->numFrictionConstr[threadIndex] = 0;
			contactHeader->numNormalConstr[threadIndex] = 0;
			contactHeader->forceWritebackOffset[threadIndex] = 0xffffffff;
			return;
		}

		PxReal maxImpulse = PxMin(data0.maxImpulse, data1.maxImpulse);

		contactHeader->forceWritebackOffset[threadIndex] = forceWritebackBufferOffset;

		contactHeader->flags[threadIndex] = flags;
	
		contactHeader->invMass0_1_angDom0_1[threadIndex] = make_float4(invMass0D0, invMass1D1, lin0X_ang0Y_lin1Z_ang1W.y, lin0X_ang0Y_lin1Z_ang1W.w);
		
		const float4 normal4_rW = contactData.normal_restitutionW[threadIndex];
		const PxVec3 normal(normal4_rW.x, normal4_rW.y, normal4_rW.z);
		const float restitution = normal4_rW.w;
		const float damping = contactData.damping[threadIndex];
		const float normalLenSq = normal.magnitudeSquared();//V3LengthSq(normal);
		const float norVel0 = normal.dot(linVel0);//V3Dot(normal, linVel0);
		const float norVel1 = normal.dot(linVel1);//V3Dot(normal, linVel1);
		const float linNorVel = norVel0 - norVel1;

		const float invMassNorLenSq0 = invMass0D0 * normalLenSq;//FMul(invMass0_dom0fV, normalLenSq);
		const float invMassNorLenSq1 = invMass1D1 * normalLenSq;//FMul(invMass1_dom1fV, normalLenSq);

		contactHeader->restitution[threadIndex] = restitution;

		for(PxU32 j=0;j<contactCount;j++)
		{
			const PxgBlockContactPoint& contact = contacts[j];

			PxgBlockSolverContactPoint* solverContact = &contactConstraints[j];

			//const Vec3V targetVel = Vec3V_From_PxVec3_Aligned(contact.targetVel);
			const float4 point4_separationW = contact.point_separationW[threadIndex];
			const PxVec3 point(point4_separationW.x, point4_separationW.y, point4_separationW.z);
			const float separation = point4_separationW.w;

			const float4 targetVel4_maxImpulseW = contact.targetVel_maxImpulseW[threadIndex];
			const PxVec3 targetVel(targetVel4_maxImpulseW.x, targetVel4_maxImpulseW.y, targetVel4_maxImpulseW.z);

			const float cTargetVel = normal.dot(targetVel);//V3Dot(normal, V3LoadA(contact.targetVel));

			const PxVec3 ra = point - bodyFrame0p;//V3Sub(point, bodyFrame0p);
			const PxVec3 rb = point - bodyFrame1p;//V3Sub(point, bodyFrame1p);

			PxVec3 raXn = ra.cross(normal);
			PxVec3 rbXn = rb.cross(normal);

			float angNorVel = raXn.dot(angVel0) - rbXn.dot(angVel1);

			const PxReal slop = solverOffsetSlop * PxMax(linNorVel == 0.f ? 1.f : angNorVel / linNorVel, 1.f);

			raXn.x = PxAbs(raXn.x) < slop ? 0.f : raXn.x;
			raXn.y = PxAbs(raXn.y) < slop ? 0.f : raXn.y;
			raXn.z = PxAbs(raXn.z) < slop ? 0.f : raXn.z;

			rbXn.x = PxAbs(rbXn.x) < slop ? 0.f : rbXn.x;
			rbXn.y = PxAbs(rbXn.y) < slop ? 0.f : rbXn.y;
			rbXn.z = PxAbs(rbXn.z) < slop ? 0.f : rbXn.z;

			const PxVec3 raXnSqrtInertia = invSqrtInertia0 * raXn;//M33MulV3(invSqrtInertia0, raXn);
			const PxVec3 rbXnSqrtInertia = invSqrtInertia1 * rbXn;//M33MulV3(invSqrtInertia1, rbXn);				

			const float resp0 = lin0X_ang0Y_lin1Z_ang1W.y *(raXnSqrtInertia.dot(raXnSqrtInertia)) + invMassNorLenSq0;//FAdd(invMassNorLenSq0, FMul(V3Dot(raXnSqrtInertia, raXnSqrtInertia), angD0));
			const float resp1 = lin0X_ang0Y_lin1Z_ang1W.w *(rbXnSqrtInertia.dot(rbXnSqrtInertia)) + invMassNorLenSq1;//FSub(FMul(V3Dot(rbXnSqrtInertia, rbXnSqrtInertia), angD1), invMassNorLenSq1);

			float angNorVel0 = raXn.dot(angVel0);
			float angNorVel1 = rbXn.dot(angVel1);

			const float vrel = linNorVel + (angNorVel0 - angNorVel1);//FSub(vrel1, vrel2);

			const float penetration = separation - restDistance;
			const bool isSeparated = (penetration >= 0.0f);
			const float penetrationInvDt = penetration * invDt;
			const float penetrationInvDtPt8 = fmaxf(maxPenBias, penetration*invDtp8);

			const bool collidingWithVrel = ((-vrel) > penetrationInvDt);
			const bool isGreater2 = ((restitution > 0.f) && (bounceThreshold > vrel)) && collidingWithVrel;

			const bool ccdSeparationCondition = ccdMaxSeparation >= penetration;

			const float sumVRel(vrel);

			float targetVelocity = cTargetVel + (isGreater2 ? (-sumVRel)*restitution : 0.f);//FAdd(cTargetVel, FSel(isGreater2, FMul(FNeg(sumVRel), restitution), zero));

			targetVelocity = targetVelocity - vrel;//FSub(targetVelocity, vrel);

			// To compute velMultiplier, impulseMultiplier, unbiasedErr, and biasedErr every sub-timestep or iteration,
			// additional data is stored: coeff0, coeff1. See also "computeContactCoefficients".

			PxReal coeff0, coeff1;
			if (restitution < 0.f) // compliant contact case
			{
				queryReducedCompliantContactCoefficients(dtF32, flags, restitution, damping, penetration,
				                                         targetVelocity, isSeparated, collidingWithVrel, coeff0, coeff1);
			}
			else
			{
				// To recover full scaledBias, unbiasedErr, and biasedErr, velMultiplier needs to be multiplied by velMultiplier.
				// See also "computeContactCoefficients".
				float scaledBias = isSeparated ? penetrationInvDt : penetrationInvDtPt8;
				if (ccdSeparationCondition && isGreater2)
					scaledBias = 0.f;

				float unbiasedErr = targetVelocity;
				float biasedErr = unbiasedErr - scaledBias;

				if (!isGreater2)
					unbiasedErr -= PxMax(0.f, scaledBias);

				coeff0 = biasedErr;
				coeff1 = unbiasedErr;
			}

			solverContact->raXn_targetVelocity[threadIndex] = make_float4(raXnSqrtInertia.x, raXnSqrtInertia.y, raXnSqrtInertia.z, targetVelocity);
			solverContact->rbXn_maxImpulse[threadIndex] = make_float4(rbXnSqrtInertia.x, rbXnSqrtInertia.y, rbXnSqrtInertia.z, PxMin(maxImpulse, targetVel4_maxImpulseW.w));

			solverContact->resp0[threadIndex] = resp0;
			solverContact->resp1[threadIndex] = resp1;

			solverContact->coeff0[threadIndex] = coeff0;
			solverContact->coeff1[threadIndex] = coeff1;

			solverContact->appliedForce[threadIndex] = 0.f;
		}			

		contactWritebackCount += contactCount;

		const PxU32 aCount = frictionPatch.anchorCount[threadIndex];

		PxReal frictionScale = (aCount == 2) ? 0.5f : 1.f;

		const PxReal staticFriction = data.staticFriction*frictionScale;
		const PxReal dynamicFriction = data.dynamicFriction*frictionScale;

		const bool haveFriction = (anchorCount != 0) ;//PX_IR(n.staticFriction) > 0 || PX_IR(n.dynamicFriction) > 0;
		contactHeader->numNormalConstr[threadIndex]	= contactCount;
		frictionHeader->numFrictionConstr[threadIndex] = anchorCount;
	
		//header->type				= type;

		contactHeader->normal_staticFriction[threadIndex] = make_float4(normal.x, normal.y, normal.z, staticFriction);
		frictionHeader->dynamicFriction[threadIndex] = dynamicFriction;

		if(haveFriction)
		{
			const PxVec3 vrel = linVel0 - linVel1;//V3Sub(linVel0, linVel1);
			//const Vec3V normal = Vec3V_From_PxVec3_Aligned(buffer.contacts[c.contactPatches[c.correlationListHeads[i]].start].normal);

			const float orthoThreshold = 0.70710678f;
			const float p1 = 0.0001f;
			// fallback: normal.cross((1,0,0)) or normal.cross((0,0,1))
			const float normalX = normal.x;//V3GetX(normal);
			const float normalY = normal.y;//V3GetY(normal);
			const float normalZ = normal.z;//V3GetZ(normal);
			
			PxVec3 t0Fallback1(0.f, -normalZ, normalY);//V3Merge(zero, FNeg(normalZ), normalY);
			PxVec3 t0Fallback2(-normalY, normalX, 0.f);//V3Merge(FNeg(normalY), normalX, zero) ;
			PxVec3 t0Fallback = orthoThreshold > PxAbs(normalX) ? t0Fallback1 : t0Fallback2;//V3Sel(FIsGrtr(orthoThreshold, FAbs(normalX)), t0Fallback1, t0Fallback2);

			PxVec3 t0 = vrel - normal *(normal.dot(vrel));//V3Sub(vrel, V3Scale(normal, V3Dot(normal, vrel)));
			t0 = t0.magnitudeSquared() > p1 ? t0 : t0Fallback;//V3Sel(FIsGrtr(V3LengthSq(t0), p1), t0, t0Fallback);
			t0.normalize();

			const PxVec3 t1 = normal.cross(t0);

			frictionHeader->broken[threadIndex] = 0;

			

			frictionHeader->frictionNormals[0][threadIndex] = make_float4(t0.x, t0.y, t0.z, 0.f);
			frictionHeader->frictionNormals[1][threadIndex] = make_float4(t1.x, t1.y, t1.z, 0.f);
			
			for(PxU32 j = 0; j < aCount; j++)
			{
				PxgBlockSolverContactFriction* PX_RESTRICT f0 = &frictionConstraints[2*j];
				PxgBlockSolverContactFriction* PX_RESTRICT f1 = &frictionConstraints[2*j+1];

				const float4 body0Anchor4 = fAnchor.body0Anchors[j][threadIndex];
				const float4 body1Anchor4 = fAnchor.body1Anchors[j][threadIndex];

				const PxVec3 body0Anchor(body0Anchor4.x, body0Anchor4.y, body0Anchor4.z);
				const PxVec3 body1Anchor(body1Anchor4.x, body1Anchor4.y, body1Anchor4.z);

				const PxVec3 ra = bodyFrame0q.rotate(body0Anchor);//QuatRotate(bodyFrame0q, body0Anchor);
				const PxVec3 rb = bodyFrame1q.rotate(body1Anchor);//QuatRotate(bodyFrame1q, body1Anchor);

				const PxVec3 error = (ra + bodyFrame0p) - (rb + bodyFrame1p);//V3Sub(V3Add(ra, bodyFrame0p), V3Add(rb, bodyFrame1p));

				PxU32 index = perPointFriction ? frictionPatch.contactID[j][threadIndex] : 0;

				const float4 targetVel4 = contacts[index].targetVel_maxImpulseW[threadIndex];
				const PxVec3 tvel(targetVel4.x, targetVel4.y, targetVel4.z);

				{
					PxVec3 raXn = ra.cross(t0);//V3Cross(ra, t0Cross);
					PxVec3 rbXn = rb.cross(t0);//V3Cross(rb, t0Cross);

					raXn.x = PxAbs(raXn.x) < solverOffsetSlop ? 0.f : raXn.x;
					raXn.y = PxAbs(raXn.y) < solverOffsetSlop ? 0.f : raXn.y;
					raXn.z = PxAbs(raXn.z) < solverOffsetSlop ? 0.f : raXn.z;

					rbXn.x = PxAbs(rbXn.x) < solverOffsetSlop ? 0.f : rbXn.x;
					rbXn.y = PxAbs(rbXn.y) < solverOffsetSlop ? 0.f : rbXn.y;
					rbXn.z = PxAbs(rbXn.z) < solverOffsetSlop ? 0.f : rbXn.z;

					const PxVec3 raXnSqrtInertia = invSqrtInertia0 * raXn;//M33MulV3(invSqrtInertia0, raXn);
					const PxVec3 rbXnSqrtInertia = invSqrtInertia1 * rbXn;//M33MulV3(invSqrtInertia1, rbXn);	

					//const float resp0 = contactData.mInvMassScale.angular0 *(raXnSqrtInertia.dot(raXnSqrtInertia)) + invMass0D0;//FAdd(invMass0_dom0fV, FMul(angD0, V3Dot(raXnSqrtInertia, raXnSqrtInertia)));
					//const float resp1 = contactData.mInvMassScale.angular1 *(rbXnSqrtInertia.dot(rbXnSqrtInertia)) + invMass1D1;//FSub(FMul(angD1, V3Dot(rbXnSqrtInertia, rbXnSqrtInertia)), invMass1_dom1fV);
					const float resp0 = lin0X_ang0Y_lin1Z_ang1W.y *(raXnSqrtInertia.dot(raXnSqrtInertia)) + invMass0D0;//FAdd(invMass0_dom0fV, FMul(angD0, V3Dot(raXnSqrtInertia, raXnSqrtInertia)));
					const float resp1 = lin0X_ang0Y_lin1Z_ang1W.w *(rbXnSqrtInertia.dot(rbXnSqrtInertia)) + invMass1D1;//FSub(FMul(angD1, V3Dot(rbXnSqrtInertia, rbXnSqrtInertia)), invMass1_dom1fV);

					float targetVel = tvel.dot(t0);//V3Dot(V3LoadU(buffer.contacts[index].targetVel), t0);

					const float vrel1 = t0.dot(linVel0) + raXn.dot(angVel0);//FAdd(V3Dot(t0, linVel0), V3Dot(raXn, angVel0));
					const float vrel2 = t0.dot(linVel1) + rbXn.dot(angVel1);//FAdd(V3Dot(t0, linVel1), V3Dot(rbXn, angVel1));
					const float vrel = vrel1 - vrel2;//FSub(vrel1, vrel2);

					targetVel = targetVel - vrel;//FSub(targetVel, vrel);

					const float bias = t0.dot(error) * invDt;
					f0->resp0[threadIndex] = resp0;
					f0->resp1[threadIndex] = resp1;

					f0->raXn_bias[threadIndex] = make_float4(raXnSqrtInertia.x, raXnSqrtInertia.y, raXnSqrtInertia.z, bias);
					f0->rbXn_targetVelW[threadIndex] = make_float4(rbXnSqrtInertia.x, rbXnSqrtInertia.y, rbXnSqrtInertia.z, targetVel);
					f0->appliedForce[threadIndex] = 0.f;
				}

				{

					PxVec3 raXn = ra.cross(t1);//V3Cross(ra, t1Cross);
					PxVec3 rbXn = rb.cross(t1);//V3Cross(rb, t1Cross);

					raXn.x = PxAbs(raXn.x) < solverOffsetSlop ? 0.f : raXn.x;
					raXn.y = PxAbs(raXn.y) < solverOffsetSlop ? 0.f : raXn.y;
					raXn.z = PxAbs(raXn.z) < solverOffsetSlop ? 0.f : raXn.z;

					rbXn.x = PxAbs(rbXn.x) < solverOffsetSlop ? 0.f : rbXn.x;
					rbXn.y = PxAbs(rbXn.y) < solverOffsetSlop ? 0.f : rbXn.y;
					rbXn.z = PxAbs(rbXn.z) < solverOffsetSlop ? 0.f : rbXn.z;

					const PxVec3 raXnSqrtInertia = invSqrtInertia0 * raXn;//M33MulV3(invSqrtInertia0, raXn);
					const PxVec3 rbXnSqrtInertia = invSqrtInertia1 * rbXn;//M33MulV3(invSqrtInertia1, rbXn);	
					
					const float resp0 = lin0X_ang0Y_lin1Z_ang1W.y *(raXnSqrtInertia.dot(raXnSqrtInertia)) + invMass0D0;//FAdd(invMass0_dom0fV, FMul(angD0, V3Dot(raXnSqrtInertia, raXnSqrtInertia)));
					const float resp1 = lin0X_ang0Y_lin1Z_ang1W.w *(rbXnSqrtInertia.dot(rbXnSqrtInertia)) + invMass1D1;//FSub(FMul(angD1, V3Dot(rbXnSqrtInertia, rbXnSqrtInertia)), invMass1_dom1fV);

					float targetVel = tvel.dot(t1);//V3Dot(V3LoadU(buffer.contacts[index].targetVel), t1);

					const float vrel1 = t1.dot(linVel0) + raXn.dot(angVel0);//FAdd(V3Dot(t1, linVel0), V3Dot(raXn, angVel0));
					const float vrel2 = t1.dot(linVel1) + rbXn.dot(angVel1);//FAdd(V3Dot(t1, linVel1), V3Dot(rbXn, angVel1));
					const float vrel = vrel1 - vrel2;//FSub(vrel1, vrel2);

					targetVel = targetVel - vrel;//FSub(targetVel, vrel);

					const float bias = t1.dot(error) * invDt;

					f1->resp0[threadIndex] = resp0;
					f1->resp1[threadIndex] = resp1;

					f1->raXn_bias[threadIndex] = make_float4(raXnSqrtInertia.x, raXnSqrtInertia.y, raXnSqrtInertia.z, bias);
					f1->rbXn_targetVelW[threadIndex] = make_float4(rbXnSqrtInertia.x, rbXnSqrtInertia.y, rbXnSqrtInertia.z, targetVel);
					f1->appliedForce[threadIndex] = 0.f;
				}
			}
		}

		frictionPatchWritebackAddrIndex++;
	}
}


static __device__ void  createFinalizeSolverContactsBlockGPU(	PxgBlockContactData* contactData,
												const PxgBlockContactPoint* PX_RESTRICT contactPoints,
												PxgBlockFrictionPatch& frictionPatch,
												const PxgBlockFrictionPatch* PX_RESTRICT prevFrictionPatches,
												PxgBlockFrictionAnchorPatch& fAnchor,
												const PxgBlockFrictionAnchorPatch* PX_RESTRICT prevFrictionAnchors,
												const PxgBlockFrictionIndex* PX_RESTRICT prevFrictionIndices,
												const PxgSolverBodyData& solverBodyData0,
												const PxgSolverBodyData& solverBodyData1,
												const PxMat33& sqrtInvInertia0,
												const PxMat33& sqrtInvInertia1,
												const PxAlignedTransform& bodyFrame0,
												const PxAlignedTransform& bodyFrame1,
												const float4& initLinVel0,
												const float4& initAngVel0,
												const float4& initLinVel1,
												const float4& initAngVel1,
												const PxReal invDtF32,
												const PxReal dtF32,
												PxReal bounceThresholdF32,
												PxReal frictionOffsetThreshold,
												PxReal correlationDistance,
												const PxU32 threadIndex,
												PxU32 forceWritebackBufferOffset,
												PxgBlockSolverContactHeader* PX_RESTRICT contactHeader,
												PxgBlockSolverFrictionHeader* PX_RESTRICT frictionHeader,
												PxgBlockSolverContactPoint* PX_RESTRICT contactConstraints,
												PxgBlockSolverContactFriction* PX_RESTRICT frictionConstraints,
												PxU32 totalEdges,
												PxU32 prevFrictionStartIndex,
												PxReal ccdMaxSeparation,
												PxReal solverOffsetSlop)
{
	//Load the body datas in using warp-wide programming...Each solver body data is 128 bytes long, so we can load both bodies in with 2 lots of 128 byte coalesced reads. 

	const float4 data_ = reinterpret_cast<float4&>(contactData->contactData[threadIndex]);
	const PxgMaterialContactData data = reinterpret_cast<const PxgMaterialContactData&>(data_);

	const PxU32 nbContacts = data.mNumContacts;	

	const float4 normal4 = contactData->normal_restitutionW[threadIndex];
	const PxVec3 normal(normal4.x, normal4.y, normal4.z);

	correlatePatches(frictionPatch, contactPoints, nbContacts, normal,
		bodyFrame0, bodyFrame1, PXC_SAME_NORMAL, threadIndex);

	PxU8 flags = data.mSolverFlags;
	bool perPointFriction =  flags & (PxgSolverContactFlags::ePER_POINT_FRICTION);
	bool disableFriction = flags & PxgSolverContactFlags::eDISABLE_FRICTION;

	//KS - ensure that the friction patch broken bit is set to 0
	frictionPatch.broken[threadIndex] = 0;
	PxReal patchExtents;

	//Mark the friction patch as not broken!
	frictionPatch.broken[threadIndex] = 0;

	__syncwarp(); //Ensure writes from correlation are visible

	if (!(perPointFriction || disableFriction))// || (solverBodyData0.islandNodeIndex & 2) || (solverBodyData1.islandNodeIndex & 2)))
	{
		getFrictionPatches(frictionPatch, fAnchor, prevFrictionIndices, prevFrictionStartIndex, prevFrictionPatches, prevFrictionAnchors, data.prevFrictionPatchCount, bodyFrame0, bodyFrame1, correlationDistance, threadIndex, totalEdges, patchExtents,
			nbContacts);
	}

	if(!disableFriction)
		growPatches(frictionPatch, fAnchor, contactPoints, nbContacts, bodyFrame0, bodyFrame1, frictionOffsetThreshold + data.restDistance, threadIndex, patchExtents);

	setupFinalizeSolverConstraintsBlock(*contactData, contactPoints, nbContacts, frictionPatch, fAnchor, bodyFrame0, bodyFrame1, solverBodyData0, solverBodyData1, 
		initLinVel0, initAngVel0, initLinVel1, initAngVel1, sqrtInvInertia0, sqrtInvInertia1, invDtF32, dtF32,
		bounceThresholdF32, threadIndex, forceWritebackBufferOffset, perPointFriction, contactHeader, frictionHeader, contactConstraints, frictionConstraints, data, ccdMaxSeparation, solverOffsetSlop);


}



static __device__ void setupFinalizeSolverConstraintsBlockTGS(PxgBlockContactData& contactData,
	const PxgBlockContactPoint* PX_RESTRICT contacts,
	const PxU32 contactCount,
	const PxgBlockFrictionPatch& frictionPatch,
	const PxgBlockFrictionAnchorPatch& fAnchor,
	const PxAlignedTransform& bodyFrame0,
	const PxAlignedTransform& bodyFrame1,
	const PxgSolverBodyData& data0,
	const PxgSolverBodyData& data1,
	const float4& initialLinVel0_invMassW,
	const float4& initialAngVel0_penBiasClamp,
	const float4& initialLinVel1_invMassW,
	const float4& initialAngVel1_penBiasClamp,
	const PxMat33& sqrtInvInertia0,
	const PxMat33& sqrtInvInertia1,
	const PxReal invStepDtF32,
	const PxReal stepDt,
	const PxReal totalDt,
	const PxReal invTotalDtF32,
	const PxReal bounceThresholdF32,
	const PxReal biasCoefficient,
	const PxU32 threadIndex,
	PxU32 forceWritebackBufferOffset,
	const bool perPointFriction,
	PxgTGSBlockSolverContactHeader* PX_RESTRICT contactHeader,
	PxgTGSBlockSolverFrictionHeader* PX_RESTRICT frictionHeader,
	PxgTGSBlockSolverContactPoint* PX_RESTRICT contactConstraints,
	PxgTGSBlockSolverContactFriction* PX_RESTRICT frictionConstraints,
	const PxgMaterialContactData& data,
	const PxReal solverOffsetSlop,
	const float2 torsionalFrictionData)
{
	using namespace physx;
	// NOTE II: the friction patches are sparse (some of them have no contact patches, and
	// therefore did not get written back to the cache) but the patch addresses are dense,
	// corresponding to valid patches

	/*const float4 data_ = reinterpret_cast<float4&>(contactData.contactData[threadIndex]);
	const PxgMaterialContactData data = reinterpret_cast<const PxgMaterialContactData&>(data_);*/

	PxU8 flags = data.mSolverFlags;

	const float restDistance = data.restDistance;//FLoad(n.restDistance); 

	const PxAlignedQuat bodyFrame0q(bodyFrame0.q);//QuatVLoadU(&bodyFrame0.q.x);
	const PxVec3 bodyFrame0p(bodyFrame0.p.x, bodyFrame0.p.y, bodyFrame0.p.z);

	const PxAlignedQuat bodyFrame1q(bodyFrame1.q);//QuatVLoadU(&bodyFrame1.q.x);
	const PxVec3 bodyFrame1p(bodyFrame1.p.x, bodyFrame1.p.y, bodyFrame1.p.z);

	uint32_t frictionPatchWritebackAddrIndex = 0;
	uint32_t contactWritebackCount = 0;

	const float maxPenBias = fmaxf(initialAngVel0_penBiasClamp.w, initialAngVel1_penBiasClamp.w);//FMax(FLoad(data0.penBiasClamp), FLoad(data1.penBiasClamp));

	const PxVec3 linVel0(initialLinVel0_invMassW.x, initialLinVel0_invMassW.y, initialLinVel0_invMassW.z);//V3LoadU(data0.initialLinVel);
	const PxVec3 linVel1(initialLinVel1_invMassW.x, initialLinVel1_invMassW.y, initialLinVel1_invMassW.z);//V3LoadU(data1.initialLinVel);
	const PxVec3 angVel0(initialAngVel0_penBiasClamp.x, initialAngVel0_penBiasClamp.y, initialAngVel0_penBiasClamp.z);//V3LoadU(data0.initialAngVel);
	const PxVec3 angVel1(initialAngVel1_penBiasClamp.x, initialAngVel1_penBiasClamp.y, initialAngVel1_penBiasClamp.z);//V3LoadU(data1.initialAngVel);

	const PxMat33 invSqrtInertia0 = sqrtInvInertia0;
	const PxMat33 invSqrtInertia1 = sqrtInvInertia1;

	const float invStepDt = invStepDtF32;
	const float p8 = PxMin(0.8f, biasCoefficient);
	const float bounceThreshold = bounceThresholdF32;

	const float invDtp8 = invStepDt * p8;//FMul(invDt, p8);

	const float4 lin0X_ang0Y_lin1Z_ang1W = contactData.mInvMassScale[threadIndex].lin0X_ang0Y_lin1Z_ang1W;

	const float invMass0D0 = initialLinVel0_invMassW.w * lin0X_ang0Y_lin1Z_ang1W.x;
	const float invMass1D1 = initialLinVel1_invMassW.w * lin0X_ang0Y_lin1Z_ang1W.z;

	const float angD0 = lin0X_ang0Y_lin1Z_ang1W.y;
	const float angD1 = lin0X_ang0Y_lin1Z_ang1W.w;

	bool isKinematic0 = !!(data0.flags & PxRigidBodyFlag::eKINEMATIC);
	bool isKinematic1 = !!(data1.flags & PxRigidBodyFlag::eKINEMATIC);

	{

		const PxU32 aCount = frictionPatch.anchorCount[threadIndex];

		PxU32 anchorCount = aCount * 2;

		//shared memory counter for max contacts in the friction patch...

		if (contactCount == 0)
		{
			frictionHeader->numFrictionConstr[threadIndex] = 0;
			contactHeader->numNormalConstr[threadIndex] = 0;
			contactHeader->forceWritebackOffset[threadIndex] = 0xffffffff;
			return;
		}

		PxReal maxImpulse = PxMin(data0.maxImpulse, data1.maxImpulse);

		contactHeader->forceWritebackOffset[threadIndex] = forceWritebackBufferOffset;

		contactHeader->flags[threadIndex] = flags;
		contactHeader->maxPenBias[threadIndex] = maxPenBias;

		contactHeader->invMass0_1_angDom0_1[threadIndex] = make_float4(invMass0D0, invMass1D1, lin0X_ang0Y_lin1Z_ang1W.y, lin0X_ang0Y_lin1Z_ang1W.w);

		const float4 normal4_rW = contactData.normal_restitutionW[threadIndex];
		const PxVec3 normal(normal4_rW.x, normal4_rW.y, normal4_rW.z);
		const float damping = contactData.damping[threadIndex];
		const float restitution = normal4_rW.w;
		const float normalLenSq = normal.magnitudeSquared();//V3LengthSq(normal);
		const float norVel0 = normal.dot(linVel0);//V3Dot(normal, linVel0);
		const float norVel1 = normal.dot(linVel1);//V3Dot(normal, linVel1);

		const float linNorVel = norVel0 - norVel1;

		const float invMassNorLenSq0 = invMass0D0 * normalLenSq;//FMul(invMass0_dom0fV, normalLenSq);
		const float invMassNorLenSq1 = invMass1D1 * normalLenSq;//FMul(invMass1_dom1fV, normalLenSq);

		PxReal maxPenetration = PX_MAX_F32;

		contactHeader->restitutionXdt[threadIndex] = restitution * stepDt;
		contactHeader->p8[threadIndex] = p8;

		for (PxU32 j = 0; j<contactCount; j++)
		{
			const PxgBlockContactPoint& contact = contacts[j];

			PxgTGSBlockSolverContactPoint* solverContact = &contactConstraints[j];

			//const Vec3V targetVel = Vec3V_From_PxVec3_Aligned(contact.targetVel);

			const float4 point4_separationW = contact.point_separationW[threadIndex];
			const PxVec3 point(point4_separationW.x, point4_separationW.y, point4_separationW.z);
			const float separation = point4_separationW.w;

			const float4 targetVel4_maxImpulseW = contact.targetVel_maxImpulseW[threadIndex];
			const PxVec3 targetVel(targetVel4_maxImpulseW.x, targetVel4_maxImpulseW.y, targetVel4_maxImpulseW.z);

			const float cTargetVel = normal.dot(targetVel);//V3Dot(normal, V3LoadA(contact.targetVel));

			const PxVec3 ra = point - bodyFrame0p;//V3Sub(point, bodyFrame0p);
			const PxVec3 rb = point - bodyFrame1p;//V3Sub(point, bodyFrame1p);

			PxVec3 raXn = ra.cross(normal);
			PxVec3 rbXn = rb.cross(normal);

			float angNorVel = raXn.dot(angVel0) - rbXn.dot(angVel1);

			const PxReal slop = solverOffsetSlop * PxMax(linNorVel == 0.f ? 1.f : angNorVel/linNorVel, 1.f);
			

			raXn.x = PxAbs(raXn.x) < slop ? 0.f : raXn.x;
			raXn.y = PxAbs(raXn.y) < slop ? 0.f : raXn.y;
			raXn.z = PxAbs(raXn.z) < slop ? 0.f : raXn.z;

			rbXn.x = PxAbs(rbXn.x) < slop ? 0.f : rbXn.x;
			rbXn.y = PxAbs(rbXn.y) < slop ? 0.f : rbXn.y;
			rbXn.z = PxAbs(rbXn.z) < slop ? 0.f : rbXn.z;

			const PxVec3 raXnSqrtInertia = invSqrtInertia0 * raXn;//M33MulV3(invSqrtInertia0, raXn);
			const PxVec3 rbXnSqrtInertia = invSqrtInertia1 * rbXn;//M33MulV3(invSqrtInertia1, rbXn);				

			const float resp0 = (raXnSqrtInertia.dot(raXnSqrtInertia))*angD0 + invMassNorLenSq0;//FAdd(invMassNorLenSq0, FMul(V3Dot(raXnSqrtInertia, raXnSqrtInertia), angD0));
			const float resp1 = (rbXnSqrtInertia.dot(rbXnSqrtInertia))*angD1 + invMassNorLenSq1;//FSub(FMul(V3Dot(rbXnSqrtInertia, rbXnSqrtInertia), angD1), invMassNorLenSq1);

			const float unitResponse = resp0 + resp1;//FAdd(resp0, resp1);

			float angNorVel0 = raXn.dot(angVel0);
			float angNorVel1 = rbXn.dot(angVel1);

			const float vrel = linNorVel + (angNorVel0 - angNorVel1);//FSub(vrel1, vrel2);

			float scaledBias = 0.f, velMultiplier = 0.f;

			float penetration = separation - restDistance;
			const bool isSeparated = (penetration > 0.0f);
			const float penetrationInvDt = penetration * invTotalDtF32;

			const float recipResponse = (unitResponse > 0.f) ? (1.f / unitResponse) : 0.f;

			PxReal contactCoeff = 0.f;

			if (restitution < 0.f)
			{
				const bool collidingWithVrel = ((-vrel * totalDt) > penetration); // Note: using totalDt here instead of penetrationInvDt because the latter has a fudge factor if there are velocity iterations
				contactCoeff = computeCompliantContactCoefficientsTGS(stepDt, flags, restitution, damping, unitResponse, recipResponse,
					isSeparated, collidingWithVrel, velMultiplier, scaledBias);
			}
			else
			{
				velMultiplier = recipResponse;//FSel(FIsGrtr(unitResponse, zero), FRecip(unitResponse), zero);
				scaledBias = isSeparated ? -invStepDt : -invDtp8;
			}

			solverContact->resp0[threadIndex] = resp0;
			solverContact->resp1[threadIndex] = resp1;

			maxPenetration = PxMin(penetration, maxPenetration);

			const bool isGreater2 = ((restitution > 0.f) && (bounceThreshold > vrel)) && ((-vrel) > penetrationInvDt);

			float targetVelocity = cTargetVel + (isGreater2 ? (-vrel)*restitution : 0.f);//FAdd(cTargetVel, FSel(isGreater2, FMul(FNeg(sumVRel), restitution), zero));

		
			//Now add on bias for any kinematic velocities...
			if(isKinematic0)
				targetVelocity -= (norVel0 + angNorVel0);
			if(isKinematic1)
				targetVelocity += (norVel1 + angNorVel1);

			if (isGreater2)
			{
				PxReal ratio = totalDt +  penetration/vrel;
				penetration += targetVelocity * ratio;
			}

			solverContact->raXn_extraCoeff[threadIndex] = make_float4(raXnSqrtInertia.x, raXnSqrtInertia.y, raXnSqrtInertia.z, contactCoeff);
			solverContact->rbXn_targetVelW[threadIndex] = make_float4(rbXnSqrtInertia.x, rbXnSqrtInertia.y, rbXnSqrtInertia.z, targetVelocity);
			solverContact->separation[threadIndex] = penetration;
			solverContact->maxImpulse[threadIndex] = PxMin(maxImpulse, targetVel4_maxImpulseW.w);
			solverContact->appliedForce[threadIndex] = 0.f;
			solverContact->biasCoefficient[threadIndex] = scaledBias;
		}

		contactWritebackCount += contactCount;

		

		PxReal frictionScale = (aCount == 2) ? 0.5f : 1.f;

		bool hasTorsionalFriction = (torsionalFrictionData.x > 0.f || torsionalFrictionData.y > 0.f) &&	aCount == 1;

		const PxReal staticFriction = data.staticFriction*frictionScale;
		const PxReal dynamicFriction = data.dynamicFriction*frictionScale;

		const bool haveFriction = (anchorCount != 0);//PX_IR(n.staticFriction) > 0 || PX_IR(n.dynamicFriction) > 0;
		contactHeader->numNormalConstr[threadIndex] = contactCount;
		frictionHeader->numFrictionConstr[threadIndex] = anchorCount + (hasTorsionalFriction ? 1 : 0);

		//header->type				= type;

		contactHeader->normal_staticFriction[threadIndex] = make_float4(normal.x, normal.y, normal.z, staticFriction);
		frictionHeader->dynamicFriction[threadIndex] = dynamicFriction;

		if (haveFriction)
		{
			const PxVec3 vrel = linVel0 - linVel1;//V3Sub(linVel0, linVel1);
												  //const Vec3V normal = Vec3V_From_PxVec3_Aligned(buffer.contacts[c.contactPatches[c.correlationListHeads[i]].start].normal);

			const float orthoThreshold = 0.70710678f;
			const float p1 = 0.0001f;
			// fallback: normal.cross((1,0,0)) or normal.cross((0,0,1))
			const float normalX = normal.x;//V3GetX(normal);
			const float normalY = normal.y;//V3GetY(normal);
			const float normalZ = normal.z;//V3GetZ(normal);

			PxVec3 t0Fallback1(0.f, -normalZ, normalY);//V3Merge(zero, FNeg(normalZ), normalY);
			PxVec3 t0Fallback2(-normalY, normalX, 0.f);//V3Merge(FNeg(normalY), normalX, zero) ;
			PxVec3 t0Fallback = orthoThreshold > fabsf(normalX) ? t0Fallback1 : t0Fallback2;//V3Sel(FIsGrtr(orthoThreshold, FAbs(normalX)), t0Fallback1, t0Fallback2);

			PxVec3 t0 = vrel - normal *(normal.dot(vrel));//V3Sub(vrel, V3Scale(normal, V3Dot(normal, vrel)));
			t0 = t0.magnitudeSquared() > p1 ? t0 : t0Fallback;//V3Sel(FIsGrtr(V3LengthSq(t0), p1), t0, t0Fallback);
			t0.normalize();

			const PxVec3 t1 = normal.cross(t0);

			frictionHeader->broken[threadIndex] = 0;

			

			frictionHeader->frictionNormals[0][threadIndex] = make_float4(t0.x, t0.y, t0.z, 0.f);
			frictionHeader->frictionNormals[1][threadIndex] = make_float4(t1.x, t1.y, t1.z, 0.f);

			frictionHeader->biasCoefficient[threadIndex] = invStepDt;

			for (PxU32 j = 0; j < aCount; j++)
			{
				PxgTGSBlockSolverContactFriction* PX_RESTRICT f0 = &frictionConstraints[2 * j];
				PxgTGSBlockSolverContactFriction* PX_RESTRICT f1 = &frictionConstraints[2 * j + 1];

				const float4 body0Anchor4 = fAnchor.body0Anchors[j][threadIndex];
				const float4 body1Anchor4 = fAnchor.body1Anchors[j][threadIndex];

				const PxVec3 body0Anchor(body0Anchor4.x, body0Anchor4.y, body0Anchor4.z);
				const PxVec3 body1Anchor(body1Anchor4.x, body1Anchor4.y, body1Anchor4.z);

				const PxVec3 ra = bodyFrame0q.rotate(body0Anchor);//QuatRotate(bodyFrame0q, body0Anchor);
				const PxVec3 rb = bodyFrame1q.rotate(body1Anchor);//QuatRotate(bodyFrame1q, body1Anchor);

				const PxVec3 error = (ra + bodyFrame0p) - (rb + bodyFrame1p);//V3Sub(V3Add(ra, bodyFrame0p), V3Add(rb, bodyFrame1p));

				PxU32 index = perPointFriction ? frictionPatch.contactID[j][threadIndex] : 0;

				const float4 targetVel4 = contacts[index].targetVel_maxImpulseW[threadIndex];
				const PxVec3 tvel(targetVel4.x, targetVel4.y, targetVel4.z);

				const PxVec3 vel1 = linVel0 + angVel0.cross(ra);//FAdd(norVel0, V3Dot(raXn, angVel0));
				const PxVec3 vel2 = linVel1 + angVel1.cross(rb);//FAdd(norVel1, V3Dot(rbXn, angVel1));

				PxVec3 tv = tvel;
				if (isKinematic0)
					tv -= vel1;
				if (isKinematic1)
					tv += vel2;

				{
					PxVec3 raXn = ra.cross(t0);//V3Cross(ra, t0Cross);
					PxVec3 rbXn = rb.cross(t0);//V3Cross(rb, t0Cross);

					raXn.x = PxAbs(raXn.x) < solverOffsetSlop ? 0.f : raXn.x;
					raXn.y = PxAbs(raXn.y) < solverOffsetSlop ? 0.f : raXn.y;
					raXn.z = PxAbs(raXn.z) < solverOffsetSlop ? 0.f : raXn.z;

					rbXn.x = PxAbs(rbXn.x) < solverOffsetSlop ? 0.f : rbXn.x;
					rbXn.y = PxAbs(rbXn.y) < solverOffsetSlop ? 0.f : rbXn.y;
					rbXn.z = PxAbs(rbXn.z) < solverOffsetSlop ? 0.f : rbXn.z;

					const PxVec3 raXnSqrtInertia = invSqrtInertia0 * raXn;//M33MulV3(invSqrtInertia0, raXn);
					const PxVec3 rbXnSqrtInertia = invSqrtInertia1 * rbXn;//M33MulV3(invSqrtInertia1, rbXn);	

																		  //const float resp0 = contactData.mInvMassScale.angular0 *(raXnSqrtInertia.dot(raXnSqrtInertia)) + invMass0D0;//FAdd(invMass0_dom0fV, FMul(angD0, V3Dot(raXnSqrtInertia, raXnSqrtInertia)));
																		  //const float resp1 = contactData.mInvMassScale.angular1 *(rbXnSqrtInertia.dot(rbXnSqrtInertia)) + invMass1D1;//FSub(FMul(angD1, V3Dot(rbXnSqrtInertia, rbXnSqrtInertia)), invMass1_dom1fV);
					const float resp0 = angD0 *(raXnSqrtInertia.dot(raXnSqrtInertia)) + invMass0D0;//FAdd(invMass0_dom0fV, FMul(angD0, V3Dot(raXnSqrtInertia, raXnSqrtInertia)));
					const float resp1 = angD1 *(rbXnSqrtInertia.dot(rbXnSqrtInertia)) + invMass1D1;//FSub(FMul(angD1, V3Dot(rbXnSqrtInertia, rbXnSqrtInertia)), invMass1_dom1fV);

					float targetVel = tv.dot(t0);//V3Dot(V3LoadU(buffer.contacts[index].targetVel), t0);

					const float frictionError = t0.dot(error);

					f0->resp0[threadIndex] = resp0;
					f0->resp1[threadIndex] = resp1;

					f0->raXn_error[threadIndex] = make_float4(raXnSqrtInertia.x, raXnSqrtInertia.y, raXnSqrtInertia.z, frictionError);
					f0->rbXn_targetVelW[threadIndex] = make_float4(rbXnSqrtInertia.x, rbXnSqrtInertia.y, rbXnSqrtInertia.z, targetVel);
					f0->appliedForce[threadIndex] = 0.f;
				}

				{

					PxVec3 raXn = ra.cross(t1);//V3Cross(ra, t1Cross);
					PxVec3 rbXn = rb.cross(t1);//V3Cross(rb, t1Cross);

					raXn.x = PxAbs(raXn.x) < solverOffsetSlop ? 0.f : raXn.x;
					raXn.y = PxAbs(raXn.y) < solverOffsetSlop ? 0.f : raXn.y;
					raXn.z = PxAbs(raXn.z) < solverOffsetSlop ? 0.f : raXn.z;

					rbXn.x = PxAbs(rbXn.x) < solverOffsetSlop ? 0.f : rbXn.x;
					rbXn.y = PxAbs(rbXn.y) < solverOffsetSlop ? 0.f : rbXn.y;
					rbXn.z = PxAbs(rbXn.z) < solverOffsetSlop ? 0.f : rbXn.z;

					const PxVec3 raXnSqrtInertia = invSqrtInertia0 * raXn;//M33MulV3(invSqrtInertia0, raXn);
					const PxVec3 rbXnSqrtInertia = invSqrtInertia1 * rbXn;//M33MulV3(invSqrtInertia1, rbXn);	

					const float resp0 = angD0 *(raXnSqrtInertia.dot(raXnSqrtInertia)) + invMass0D0;//FAdd(invMass0_dom0fV, FMul(angD0, V3Dot(raXnSqrtInertia, raXnSqrtInertia)));
					const float resp1 = angD1 *(rbXnSqrtInertia.dot(rbXnSqrtInertia)) + invMass1D1;//FSub(FMul(angD1, V3Dot(rbXnSqrtInertia, rbXnSqrtInertia)), invMass1_dom1fV);

					float targetVel = tv.dot(t1);//V3Dot(V3LoadU(buffer.contacts[index].targetVel), t1);

					const float frictionError = t1.dot(error);
					f1->resp0[threadIndex] = resp0;
					f1->resp1[threadIndex] = resp1;

					f1->raXn_error[threadIndex] = make_float4(raXnSqrtInertia.x, raXnSqrtInertia.y, raXnSqrtInertia.z, frictionError);
					f1->rbXn_targetVelW[threadIndex] = make_float4(rbXnSqrtInertia.x, rbXnSqrtInertia.y, rbXnSqrtInertia.z, targetVel);
					f1->appliedForce[threadIndex] = 0.f;
				}
			}


			if (hasTorsionalFriction)
			{
				//Add in torsional friction term...

				PxgTGSBlockSolverContactFriction* PX_RESTRICT f0 = &frictionConstraints[2];

				const PxReal torsionalPatchRadius = torsionalFrictionData.x;
				const PxReal minTorsionalPatchRadius = torsionalFrictionData.y;

				PxReal torsionalFriction = PxMax(minTorsionalPatchRadius, PxSqrt(PxMax(0.f, -maxPenetration) * torsionalPatchRadius));

				frictionHeader->torsionalFrictionScale[threadIndex] = torsionalFriction;

				PxVec3 raXnI = invSqrtInertia0 * normal;
				PxVec3 rbXnI = invSqrtInertia1 * normal;

				const float resp0 = angD0 * (raXnI.dot(raXnI));
				const float resp1 = angD1 * (rbXnI.dot(rbXnI));

				PxReal targetVel = 0.f;
				if (isKinematic0)
					targetVel += normal.dot(angVel0);
				if (isKinematic1)
					targetVel -= normal.dot(angVel1);

				const float frictionError = 0.f;

				f0->resp0[threadIndex] = resp0;
				f0->resp1[threadIndex] = resp1;

				f0->raXn_error[threadIndex] = make_float4(raXnI.x, raXnI.y, raXnI.z, frictionError);
				f0->rbXn_targetVelW[threadIndex] = make_float4(rbXnI.x, rbXnI.y, rbXnI.z, targetVel);
				f0->appliedForce[threadIndex] = 0.f;
			}

		}

		frictionPatchWritebackAddrIndex++;
	}
}


static __device__ void  createFinalizeSolverContactsBlockGPUTGS(PxgBlockContactData* contactData,
	const PxgBlockContactPoint* PX_RESTRICT contactPoints,
	PxgBlockFrictionPatch& frictionPatch,
	const PxgBlockFrictionPatch* PX_RESTRICT prevFrictionPatches,
	PxgBlockFrictionAnchorPatch& fAnchor,
	const PxgBlockFrictionAnchorPatch* PX_RESTRICT prevFrictionAnchors,
	const PxgBlockFrictionIndex* PX_RESTRICT prevFrictionIndices,
	const PxgSolverBodyData& solverBodyData0,
	const PxgSolverBodyData& solverBodyData1,
	const PxMat33& sqrtInvInertia0,
	const PxMat33& sqrtInvInertia1,
	const PxAlignedTransform& bodyFrame0,
	const PxAlignedTransform& bodyFrame1,
	const float4& initLinVel0,
	const float4& initAngVel0,
	const float4& initLinVel1,
	const float4& initAngVel1,
	const PxReal invDtF32,
	const PxReal stepDt,
	const PxReal totalDt,
	const PxReal invTotalDt,
	const PxReal bounceThresholdF32,
	const PxReal frictionOffsetThreshold,
	const PxReal correlationDistance,
	const PxReal biasCoefficient,
	const PxU32 threadIndex,
	PxU32 forceWritebackBufferOffset,
	PxgTGSBlockSolverContactHeader* PX_RESTRICT contactHeader,
	PxgTGSBlockSolverFrictionHeader* PX_RESTRICT frictionHeader,
	PxgTGSBlockSolverContactPoint* PX_RESTRICT contactConstraints,
	PxgTGSBlockSolverContactFriction* PX_RESTRICT frictionConstraints,
	PxU32 totalEdges,
	PxU32 prevFrictionStartIndex,
	PxReal ccdMaxSeparation,
	PxReal solverOffsetSlop,
	const float2 torsionalFrictionData
	)
{
	//Load the body datas in using warp-wide programming...Each solver body data is 128 bytes long, so we can load both bodies in with 2 lots of 128 byte coalesced reads. 

	const float4 data_ = reinterpret_cast<float4&>(contactData->contactData[threadIndex]);
	const PxgMaterialContactData data = reinterpret_cast<const PxgMaterialContactData&>(data_);

	const PxU32 nbContacts = data.mNumContacts;

	const float4 normal4 = contactData->normal_restitutionW[threadIndex];
	const PxVec3 normal(normal4.x, normal4.y, normal4.z);

	correlatePatches(frictionPatch, contactPoints, nbContacts, normal,
		bodyFrame0, bodyFrame1, PXC_SAME_NORMAL, threadIndex);

	PxU8 flags = data.mSolverFlags;
	bool perPointFriction = flags & (PxgSolverContactFlags::ePER_POINT_FRICTION);
	bool disableFriction = flags & PxgSolverContactFlags::eDISABLE_FRICTION;

	//KS - for now, we disable friction correlation
	PxReal patchExtents;

	//Mark the friction patch as not broken!
	frictionPatch.broken[threadIndex] = 0;

	__syncwarp(); //Ensure writes from correlation are visible

	if (!(perPointFriction || disableFriction))// || (solverBodyData0.islandNodeIndex & 2) || (solverBodyData1.islandNodeIndex & 2)))
	{
		getFrictionPatches(frictionPatch, fAnchor, prevFrictionIndices, prevFrictionStartIndex, prevFrictionPatches, prevFrictionAnchors, data.prevFrictionPatchCount, bodyFrame0, bodyFrame1, correlationDistance, threadIndex, totalEdges, patchExtents,
			nbContacts);
	}

	if (!disableFriction)
		growPatches(frictionPatch, fAnchor, contactPoints, nbContacts, bodyFrame0, bodyFrame1, frictionOffsetThreshold + data.restDistance, threadIndex, patchExtents);

	setupFinalizeSolverConstraintsBlockTGS(*contactData, contactPoints, nbContacts, frictionPatch, fAnchor, bodyFrame0, bodyFrame1, solverBodyData0, solverBodyData1,
		initLinVel0, initAngVel0, initLinVel1, initAngVel1, sqrtInvInertia0, sqrtInvInertia1, invDtF32, stepDt, totalDt, invTotalDt,
		bounceThresholdF32, biasCoefficient, threadIndex, forceWritebackBufferOffset, perPointFriction, contactHeader, frictionHeader, 
		contactConstraints, frictionConstraints, data, solverOffsetSlop, torsionalFrictionData);


}



#endif
