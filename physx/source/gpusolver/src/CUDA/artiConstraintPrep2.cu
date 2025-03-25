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

#include "contactConstraintBlockPrep.cuh"
#include "PxgArticulation.h"
#include "PxgSolverBody.h"
#include "PxgConstraint.h"
#include "PxgFrictionPatch.h"
#include "PxgConstraintPrep.h"
#include "PxgSolverConstraintDesc.h"
#include "PxgArticulationCoreDesc.h"

#include "PxgSolverCoreDesc.h"
#include "PxgSolverFlags.h"
#include "PxgSolverConstraintBlock1D.h"

#include "DyCpuGpu1dConstraint.h"
#include "DyCpuGpuArticulation.h"
#include "DyConstraintPrep.h"

#include "jointConstraintBlockPrep.cuh"

#include <assert.h>


using namespace physx;

extern "C" __host__ void initSolverKernels13() {}

static __device__ void createPxgSolverExtBody(const PxNodeIndex& nodeIndex, const PxU32 solverBodyIndex, const PxgArticulation* const PX_RESTRICT articulations,
	const physx::PxgSolverBodyData* const PX_RESTRICT solverBodyData,
	const PxgSolverTxIData* const PX_RESTRICT txIDatas, physx::PxgSolverExtBody2& b,
	const PxAlignedTransform* const PX_RESTRICT bodyFrames)
{
	if (nodeIndex.isArticulation())
	{
		PxU32 linkID = nodeIndex.articulationLinkId();
		const PxgArticulation& artic = articulations[solverBodyIndex];

		b.bodyIndex = solverBodyIndex;
		b.linkIndex = linkID;
		b.body2World = artic.linkBody2Worlds[linkID];
		b.mSpatialResponse = artic.spatialResponseMatrixW[linkID];
		b.isKinematic = false;
		b.islandNodeIndex = nodeIndex;
		b.penBiasClamp = artic.links[linkID].initialAngVelXYZ_penBiasClamp.w;
		b.maxImpulse = PX_MAX_F32; //KS - TODO - hook up!
		b.velocity = artic.motionVelocities[linkID]; //KS - TODO - verify this is set up
		b.cfm = artic.cfms[linkID];
		b.offsetSlop = artic.links[linkID].offsetSlop;
		/*printf("%i response = (%f, %f, %f, %f, %f, %f), (%f, %f, %f, %f, %f, %f), (%f, %f, %f, %f, %f, %f), (%f, %f, %f, %f, %f, %f)\n", 
			linkID, b.mSpatialResponse.column[0][0], b.mSpatialResponse.column[0][1], b.mSpatialResponse.column[0][2], b.mSpatialResponse.column[0][3], b.mSpatialResponse.column[0][4], b.mSpatialResponse.column[0][5],
			b.mSpatialResponse.column[1][0], b.mSpatialResponse.column[1][1], b.mSpatialResponse.column[1][2], b.mSpatialResponse.column[1][3], b.mSpatialResponse.column[1][4], b.mSpatialResponse.column[1][5],
			b.mSpatialResponse.column[2][0], b.mSpatialResponse.column[2][1], b.mSpatialResponse.column[2][2], b.mSpatialResponse.column[2][3], b.mSpatialResponse.column[2][4], b.mSpatialResponse.column[2][5],
			b.mSpatialResponse.column[3][0], b.mSpatialResponse.column[3][1], b.mSpatialResponse.column[3][2], b.mSpatialResponse.column[3][3], b.mSpatialResponse.column[3][4], b.mSpatialResponse.column[3][5]);
		printf("%i response part 2 = (%f, %f, %f, %f, %f, %f), (%f, %f, %f, %f, %f, %f)\n",
			linkID, b.mSpatialResponse.column[4][0], b.mSpatialResponse.column[4][1], b.mSpatialResponse.column[4][2], b.mSpatialResponse.column[4][3], b.mSpatialResponse.column[4][4], b.mSpatialResponse.column[4][5],
			b.mSpatialResponse.column[5][0], b.mSpatialResponse.column[5][1], b.mSpatialResponse.column[5][2], b.mSpatialResponse.column[5][3], b.mSpatialResponse.column[5][4], b.mSpatialResponse.column[5][5]);*/
	}
	else
	{
		bool isStatic = nodeIndex.isStaticBody();
		const PxgSolverBodyData& solverBody = solverBodyData[solverBodyIndex];
		const bool isKinematic = solverBody.flags & PxRigidBodyFlag::eKINEMATIC;

		Cm::UnAlignedSpatialVector velocity(PxVec3(0.f), PxVec3(0.f));
		PxReal penBiasClamp = -PX_MAX_F32;
		PxReal invMass = 0.f;
		PxAlignedTransform body2World = bodyFrames[solverBodyIndex];
		PxReal offsetSlop = 0.f;
		if (!isStatic)
		{
			float4	initialAngVelXYZ_penBiasClamp = solverBody.initialAngVelXYZ_penBiasClamp;
			float4	initialLinVelXYZ_invMassW = solverBody.initialLinVelXYZ_invMassW;
			penBiasClamp = initialAngVelXYZ_penBiasClamp.w;
			invMass = initialLinVelXYZ_invMassW.w;

			velocity.top = PxVec3(initialAngVelXYZ_penBiasClamp.x, initialAngVelXYZ_penBiasClamp.y, initialAngVelXYZ_penBiasClamp.z);
			velocity.bottom = PxVec3(initialLinVelXYZ_invMassW.x, initialLinVelXYZ_invMassW.y, initialLinVelXYZ_invMassW.z);
			offsetSlop = solverBody.offsetSlop;
		}

		if (isStatic || isKinematic)
		{
			memset(b.mSpatialResponse.column, 0, sizeof(b.mSpatialResponse));
		}
		else
		{
			/*printf("SqrtInvInertia = (%f, %f, %f), (%f, %f, %f), (%f, %f, %f)\n", txIDatas[solverBodyIndex].sqrtInvInertia.column0.x,
				txIDatas[solverBodyIndex].sqrtInvInertia.column0.y, txIDatas[solverBodyIndex].sqrtInvInertia.column0.z,
				txIDatas[solverBodyIndex].sqrtInvInertia.column1.x, txIDatas[solverBodyIndex].sqrtInvInertia.column1.y, 
				txIDatas[solverBodyIndex].sqrtInvInertia.column1.z, txIDatas[solverBodyIndex].sqrtInvInertia.column2.x,
				txIDatas[solverBodyIndex].sqrtInvInertia.column2.y, txIDatas[solverBodyIndex].sqrtInvInertia.column2.z);*/
			b.mSpatialResponse.initialize(txIDatas[solverBodyIndex].sqrtInvInertia, invMass);
		}

		b.isKinematic = isKinematic;
		b.linkIndex = PxSolverConstraintDesc::RIGID_BODY;
		b.bodyIndex = solverBodyIndex;
		b.islandNodeIndex = nodeIndex;
		b.penBiasClamp = penBiasClamp;
		b.body2World.p = PxVec3(body2World.p.x, body2World.p.y, body2World.p.z);
		b.body2World.q = body2World.q;
		b.maxImpulse = solverBody.maxImpulse;
		b.velocity = velocity;
		b.cfm = 0.f;
		b.offsetSlop = offsetSlop;
	}
}

static __device__ Cm::UnAlignedSpatialVector createImpulseResponseVector(const physx::PxgSolverExtBody2& b, const Cm::UnAlignedSpatialVector& impulse)
{
	
	if (b.islandNodeIndex.isArticulation() || b.isKinematic)
	{
		//We do not alter the impulse. vector - no need because the velocity of the actor is in world space (unscaled)!
		return impulse;
	}
	else
	{
		//For rigid bodies, the angular velocity is scaled by sqrtInertia. This allows us to use the same vector to
		//project from sqrtInertia space to velocity space and from momentum (inertia) space to sqrtInertia space.
		return Cm::UnAlignedSpatialVector(b.mSpatialResponse.multiplyInertia(impulse.top), impulse.bottom);
	}
}

static __device__ PxReal projectVelocity(const PxgSolverExtBody2& b, const Cm::UnAlignedSpatialVector& responseVector)
{
	return b.velocity.dot(responseVector);
}

static __device__ PxReal projectAngular(const PxgSolverExtBody2& b, const PxVec3& responseVector)
{
	return b.velocity.top.dot(responseVector);
}

static __device__ Cm::UnAlignedSpatialVector getImpulseResponse(const PxgSolverExtBody2& b, const Cm::UnAlignedSpatialVector& vector)
{
	return b.mSpatialResponse * vector;
}


//for PGS solver(invDtF32 and invTotalDt is the same)
static __device__ void setupFinalizeExtSolverConstraintsBlock(PxgBlockContactData& contactData,
	const PxgBlockContactPoint* PX_RESTRICT contacts,
	const PxU32 contactCount,
	const PxgBlockFrictionPatch& frictionPatch,
	const PxgBlockFrictionAnchorPatch& fAnchor,
	const PxgSolverExtBody2& b0,
	const PxgSolverExtBody2& b1,
	const PxReal invDtF32,
	const PxReal dt,
	const PxReal invTotalDt,
	const PxReal bounceThresholdF32,
	const PxReal biasCoefficient,
	const PxU32 threadIndex,
	PxU32 forceWritebackBufferOffset,
	const bool perPointFriction,
	PxgContactBlockParams& params,
	PxgArticulationBlockResponse* PX_RESTRICT articulationResponses,
	const PxgMaterialContactData& data,
	const PxReal ccdMaxSeparationThreshold,
	const PxReal solverOffsetSlop,
	const float2 torsionalFrictionData,
	const PxReal totalDt)
{
	using namespace physx;
	// NOTE II: the friction patches are sparse (some of them have no contact patches, and
	// therefore did not get written back to the cache) but the patch addresses are dense,
	// corresponding to valid patches

	/*const float4 data_ = reinterpret_cast<float4&>(contactData.contactData[threadIndex]);
	const PxgMaterialContactData data = reinterpret_cast<const PxgMaterialContactData&>(data_);*/

	PxU8 flags = data.mSolverFlags; // encoding according to PxgSolverContactFlags

	const float restDistance = data.restDistance;//FLoad(n.restDistance); 

	const PxVec3& bodyFrame0p = b0.body2World.p;

	const PxAlignedQuat& bodyFrame1q = b1.body2World.q;//QuatVLoadU(&bodyFrame1.q.x);
	const PxVec3& bodyFrame1p = b1.body2World.p;

	uint32_t frictionPatchWritebackAddrIndex = 0;
	uint32_t contactWritebackCount = 0;

	//TODO - fix up!!!!
	bool isCCD = false;// (data0.islandNodeIndex & 1) || data1.islandNodeIndex & 1;
	const PxReal ccdMaxSeparation = isCCD ? ccdMaxSeparationThreshold : PX_MAX_F32;

	const float maxPenBias = fmaxf(b0.penBiasClamp, b1.penBiasClamp);
	//printf("%i: MaxPenBias = %f\n", threadIndex, maxPenBias);

	const float invDt = invDtF32;
	const float p8 = 0.8f;
	const float bounceThreshold = bounceThresholdF32;

	const float invDtp8 = invDt * p8;//FMul(invDt, p8);

	const float4 lin0X_ang0Y_lin1Z_ang1W = contactData.mInvMassScale[threadIndex].lin0X_ang0Y_lin1Z_ang1W;

	const PxVec3& linVel0 = b0.velocity.bottom;
	const PxVec3& linVel1 = b1.velocity.bottom;
	const PxVec3& angVel0 = b0.velocity.top;
	const PxVec3& angVel1 = b1.velocity.top;

	const PxReal cfm = PxMax(b0.cfm, b1.cfm);

	{
		PxU32 anchorCount = frictionPatch.anchorCount[threadIndex] * 2;

		//shared memory counter for max contacts in the friction patch...

		if (contactCount == 0)
		{
			//printf("%i: Not Prepping %i contacts\n", threadIndex, contactCount);
			params.blockFrictionHeader->numFrictionConstr[threadIndex] = 0;
			params.blockContactHeader->numNormalConstr[threadIndex] = 0;
			params.blockContactHeader->forceWritebackOffset[threadIndex] = 0xffffffff;
			return;
		}

		//printf("Prepping %i contacts!\n", contactCount);
		PxReal maxImpulse = PxMin(b0.maxImpulse, b1.maxImpulse);

		params.blockContactHeader->forceWritebackOffset[threadIndex] = forceWritebackBufferOffset;

		params.blockContactHeader->flags[threadIndex] = flags;

		//KS - despite the variable name, this only stores the invMassScale terms for extended contacts
		params.blockContactHeader->invMass0_1_angDom0_1[threadIndex] = make_float4(lin0X_ang0Y_lin1Z_ang1W.x, lin0X_ang0Y_lin1Z_ang1W.z, lin0X_ang0Y_lin1Z_ang1W.y, lin0X_ang0Y_lin1Z_ang1W.w);

		const float4 normal4_rW = contactData.normal_restitutionW[threadIndex];
		const PxVec3 normal(normal4_rW.x, normal4_rW.y, normal4_rW.z);
		const PxReal restitution = normal4_rW.w;
		const PxReal damping = contactData.damping[threadIndex];
		const PxReal normalLenSq = normal.magnitudeSquared();//V3LengthSq(normal);
		const PxReal norVel0 = normal.dot(linVel0);//V3Dot(normal, linVel0);
		const PxReal norVel1 = normal.dot(linVel1);//V3Dot(normal, linVel1);
		const PxReal linNorVel = norVel0 - norVel1;

		params.blockContactHeader->restitution[threadIndex] = restitution;
		params.blockContactHeader->cfm[threadIndex] = cfm;

		PxgArticulationBlockResponse* PX_RESTRICT artiResponse = articulationResponses;

		//printf("%i: Prepping %i contacts\n", threadIndex, contactCount);

		PxReal normalForce = 0.f;

		for (PxU32 j = 0; j<contactCount; j++)
		{
			const PxgBlockContactPoint& contact = contacts[j];

			PxgBlockSolverContactPoint* solverContact = &params.blockContactPoints[j];

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

			//printf("%i: normal = (%f, %f, %f), raXn = (%f, %f, %f), rbXn = (%f, %f, %f)\n", threadIndex, normal.x, normal.y, normal.z, raXn.x, raXn.y, raXn.z,
			//	rbXn.x, rbXn.y, rbXn.z);

			const Cm::UnAlignedSpatialVector response0 = createImpulseResponseVector(b0, Cm::UnAlignedSpatialVector(raXn, normal));
			const Cm::UnAlignedSpatialVector response1 = createImpulseResponseVector(b1, Cm::UnAlignedSpatialVector(-rbXn, -normal));

			//printf("%i: response0 = (%f, %f, %f, %f, %f, %f), response1 = (%f, %f, %f, %f, %f, %f)\n", 
			//	threadIndex, response0.top.x, response0.top.y, response0.top.z, response0.bottom.x, response0.bottom.y, response0.bottom.z,
			//	response1.top.x, response1.top.y, response1.top.z, response1.bottom.x, response1.bottom.y, response1.bottom.z);

			const Cm::UnAlignedSpatialVector deltaV0 = getImpulseResponse(b0, Cm::UnAlignedSpatialVector(normal, raXn).scale(lin0X_ang0Y_lin1Z_ang1W.x, lin0X_ang0Y_lin1Z_ang1W.y));
			const Cm::UnAlignedSpatialVector deltaV1 = getImpulseResponse(b1, Cm::UnAlignedSpatialVector(-normal, -rbXn).scale(lin0X_ang0Y_lin1Z_ang1W.z, lin0X_ang0Y_lin1Z_ang1W.w));

			//printf("%i: DeltaV0 = (%f, %f, %f, %f, %f, %f), deltaV1 = (%f, %f, %f, %f, %f, %f)\n", threadIndex, deltaV0.top.x, deltaV0.top.y, deltaV0.top.z,
			//	deltaV0.bottom.x, deltaV0.bottom.y, deltaV0.bottom.z, deltaV1.top.x, deltaV1.top.y, deltaV1.top.z,
			//	deltaV1.bottom.x, deltaV1.bottom.y, deltaV1.bottom.z);

			const float resp0 = deltaV0.dot(response0);
			const float resp1 = deltaV1.dot(response1);

			const float unitResponse = resp0 + resp1;//FAdd(resp0, resp1);

			const float vrel1 = norVel0 + projectAngular(b0, raXn);//FAdd(norVel0, V3Dot(raXn, angVel0));
			const float vrel2 = norVel1 + projectAngular(b1, rbXn);//FAdd(norVel1, V3Dot(rbXn, angVel1));
			const float vrel = vrel1 - vrel2;//FSub(vrel1, vrel2);

			//printf("%i: vrel = %f, vrel1 = %f, vrel2 = %f\n", threadIndex, vrel, vrel1, vrel2);


			const float penetration = separation - restDistance;
			const bool isSeparated = (penetration >= 0.0f);
			const float penetrationInvDt = penetration * invDt;
			const float penetrationInvDtPt8 = fmaxf(maxPenBias, penetration*invDtp8);

			const bool collidingWithVrel = ((-vrel) > penetrationInvDt);
			const bool isGreater2 = (restitution > 0.f) && (bounceThreshold > vrel) && collidingWithVrel;

			const bool ccdSeparationCondition = ccdMaxSeparation >= penetration;

			const float sumVRel(vrel);

			float targetVelocity = cTargetVel + (isGreater2 ? (-sumVRel)*restitution : 0.f);//FAdd(cTargetVel, FSel(isGreater2, FMul(FNeg(sumVRel), restitution), zero));

			const PxReal tvel = targetVelocity;

			if (!b0.islandNodeIndex.isArticulation())
				targetVelocity -= vrel1;
			if (!b1.islandNodeIndex.isArticulation())
				targetVelocity += vrel2;

			const float recipResponse = (unitResponse > 0.0f) ? (1.f / (unitResponse + cfm)) : 0.f; //FSel(FIsGrtr(unitResponse, zero), FRecip(unitResponse), zero);
			PxReal biasedErr, unbiasedErr, impulseMultiplier;
			PxReal velMultiplier = recipResponse;
			
			// To compute velMultiplier, impulseMultiplier, unbiasedErr, and biasedErr every sub-timestep or iteration,
			// additional data is stored: coeff0, coeff1.
			// Using coeff0 and coeff1 produces the same results as the previous implementation when mass-splitting does
			// not occur. See also "computeContactCoefficients".

			PxReal coeff0, coeff1;
			if (restitution < 0.f) // compliant contact case
			{
				coeff0 = computeCompliantContactCoefficients(
				    dt, flags, restitution, damping, unitResponse, recipResponse, penetration, targetVelocity,
				    isSeparated, collidingWithVrel, velMultiplier, impulseMultiplier, unbiasedErr, biasedErr);

				const PxReal nrdt = dt * restitution;
				coeff1 = -nrdt * penetration; 
			}
			else
			{
				float scaledBias = isSeparated ? penetrationInvDt : penetrationInvDtPt8;
				if(ccdSeparationCondition && isGreater2)
					scaledBias = 0.f;

				unbiasedErr = targetVelocity;
				biasedErr = unbiasedErr - scaledBias;

				if(!isGreater2)
					unbiasedErr -= PxMax(0.f, scaledBias);

				coeff0 = biasedErr; // When recovering full biasedErr, multiply coeff0 by velMultiplier. final biasedErr = coeff0 * velMultiplier
				coeff1 = unbiasedErr; // When recovering full unbiasedErr, multiply coeff0 by velMultiplier. final unbiasedErr = coeff1 * velMultiplier
			}

			solverContact->raXn_targetVelocity[threadIndex] = make_float4(response0.top.x, response0.top.y, response0.top.z, targetVelocity);
			solverContact->rbXn_maxImpulse[threadIndex] = make_float4(-response1.top.x, -response1.top.y, -response1.top.z, PxMin(maxImpulse, targetVel4_maxImpulseW.w));

			solverContact->resp0[threadIndex] = resp0;
			solverContact->resp1[threadIndex] = resp1;

			solverContact->coeff0[threadIndex] = coeff0;
			solverContact->coeff1[threadIndex] = coeff1;

			solverContact->appliedForce[threadIndex] = 0.f;

			// Note: 'deltaF' is used here only for an approximate query related to minNormalForce.
			// Mass-splitting is not applied in this context.
			// To apply mass-splitting when computing 'minNormalForce', perform the calculations in solveExtContactsBlock.
			const PxReal deltaF = fmaxf((tvel - (vrel + fmaxf(penetrationInvDt, 0.f))) * velMultiplier, 0.f);
			normalForce += deltaF;

			artiResponse->deltaRALin_x[threadIndex] = deltaV0.bottom.x;
			artiResponse->deltaRALin_y[threadIndex] = deltaV0.bottom.y;
			artiResponse->deltaRALin_z[threadIndex] = deltaV0.bottom.z;
			artiResponse->deltaRAAng_x[threadIndex] = deltaV0.top.x;
			artiResponse->deltaRAAng_y[threadIndex] = deltaV0.top.y;
			artiResponse->deltaRAAng_z[threadIndex] = deltaV0.top.z;
			artiResponse->deltaRBLin_x[threadIndex] = deltaV1.bottom.x;
			artiResponse->deltaRBLin_y[threadIndex] = deltaV1.bottom.y;
			artiResponse->deltaRBLin_z[threadIndex] = deltaV1.bottom.z;
			artiResponse->deltaRBAng_x[threadIndex] = deltaV1.top.x;
			artiResponse->deltaRBAng_y[threadIndex] = deltaV1.top.y;
			artiResponse->deltaRBAng_z[threadIndex] = deltaV1.top.z;
			artiResponse++;
		}

		params.blockContactHeader->minNormalForce[threadIndex] = (normalForce / contactCount)*0.25f;

		contactWritebackCount += contactCount;

		const PxU32 aCount = frictionPatch.anchorCount[threadIndex];

		PxReal frictionScale = (aCount == 2) ? 0.5f : 1.f;

		const PxReal staticFriction = data.staticFriction*frictionScale;
		const PxReal dynamicFriction = data.dynamicFriction*frictionScale;

		const bool haveFriction = (anchorCount != 0);//PX_IR(n.staticFriction) > 0 || PX_IR(n.dynamicFriction) > 0;
		params.blockContactHeader->numNormalConstr[threadIndex] = contactCount;
		params.blockFrictionHeader->numFrictionConstr[threadIndex] = anchorCount;

		//printf("threadIdx %i numNormalConst %i numFrictionConstr %i\n", threadIndex, contactCount, anchorCount);

		//header->type				= type;

		params.blockContactHeader->normal_staticFriction[threadIndex] = make_float4(normal.x, normal.y, normal.z, staticFriction);
		params.blockFrictionHeader->dynamicFriction[threadIndex] = dynamicFriction;

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
			PxVec3 t0Fallback = orthoThreshold > PxAbs(normalX) ? t0Fallback1 : t0Fallback2;//V3Sel(FIsGrtr(orthoThreshold, FAbs(normalX)), t0Fallback1, t0Fallback2);

			PxVec3 t0 = vrel - normal *(normal.dot(vrel));//V3Sub(vrel, V3Scale(normal, V3Dot(normal, vrel)));
			t0 = t0.magnitudeSquared() > p1 ? t0 : t0Fallback;//V3Sel(FIsGrtr(V3LengthSq(t0), p1), t0, t0Fallback);
			t0.normalize();

			const PxVec3 t1 = normal.cross(t0);

			params.blockFrictionHeader->broken[threadIndex] = 0;

			params.blockFrictionHeader->frictionNormals[0][threadIndex] = make_float4(t0.x, t0.y, t0.z, 0.f);
			params.blockFrictionHeader->frictionNormals[1][threadIndex] = make_float4(t1.x, t1.y, t1.z, 0.f);

			for (PxU32 j = 0; j < aCount; j++)
			{
				PxgBlockSolverContactFriction* PX_RESTRICT f0 = &params.blockFrictions[2 * j];
				PxgBlockSolverContactFriction* PX_RESTRICT f1 = &params.blockFrictions[2 * j + 1];

				const float4 body0Anchor4 = fAnchor.body0Anchors[j][threadIndex];
				const float4 body1Anchor4 = fAnchor.body1Anchors[j][threadIndex];

				const PxVec3 body0Anchor(body0Anchor4.x, body0Anchor4.y, body0Anchor4.z);
				const PxVec3 body1Anchor(body1Anchor4.x, body1Anchor4.y, body1Anchor4.z);

				const PxVec3 ra = b0.body2World.q.rotate(body0Anchor);//QuatRotate(bodyFrame0q, body0Anchor);
				const PxVec3 rb = b1.body2World.q.rotate(body1Anchor);//QuatRotate(bodyFrame1q, body1Anchor);

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
					
					const Cm::UnAlignedSpatialVector response0 = createImpulseResponseVector(b0, Cm::UnAlignedSpatialVector(raXn, t0));
					const Cm::UnAlignedSpatialVector response1 = createImpulseResponseVector(b1, Cm::UnAlignedSpatialVector(-rbXn, -t0));

					const Cm::UnAlignedSpatialVector deltaV0 = getImpulseResponse(b0, Cm::UnAlignedSpatialVector(t0, raXn).scale(lin0X_ang0Y_lin1Z_ang1W.x, lin0X_ang0Y_lin1Z_ang1W.y));
					const Cm::UnAlignedSpatialVector deltaV1 = getImpulseResponse(b1, Cm::UnAlignedSpatialVector(-t0, -rbXn).scale(lin0X_ang0Y_lin1Z_ang1W.z, lin0X_ang0Y_lin1Z_ang1W.w));

					//printf("%i: deltaV0 = (%f, %f, %f, %f, %f, %f)\n", b0.linkIndex, deltaV0.top.x, deltaV0.top.y, deltaV0.top.z,
					//	deltaV0.bottom.x, deltaV0.bottom.y, deltaV0.bottom.z);

					const float resp0 = deltaV0.dot(response0);
					const float resp1 = deltaV1.dot(response1);

					float targetVel = tvel.dot(t0);//V3Dot(V3LoadU(buffer.contacts[index].targetVel), t0);

					if (!b0.islandNodeIndex.isArticulation())
						targetVel -= projectVelocity(b0, Cm::UnAlignedSpatialVector(raXn, t0));
					if (!b1.islandNodeIndex.isArticulation())
						targetVel += projectVelocity(b1, Cm::UnAlignedSpatialVector(rbXn, t0));

					// Storing resp0 and resp1, separately.
					const float bias = t0.dot(error) * invDt;
					f0->raXn_bias[threadIndex] = make_float4(response0.top.x, response0.top.y, response0.top.z, bias);
					f0->rbXn_targetVelW[threadIndex] = make_float4(-response1.top.x, -response1.top.y, -response1.top.z, targetVel);
					f0->appliedForce[threadIndex] = 0.f;
					f0->resp0[threadIndex] = resp0;
					f0->resp1[threadIndex] = resp1;

					//printf("%i: Normal = (%f, %f, %f), raXn = (%f, %f, %f), Friction velMultiplier = %f, biasedErr = %f, targetVel = %f\n", 
					//	b0.linkIndex, t0.x, t0.y, t0.z, raXn.x, raXn.y, raXn.z, velMultiplier, t0.dot(error)*invDt, targetVel);

					artiResponse->deltaRALin_x[threadIndex] = deltaV0.bottom.x;
					artiResponse->deltaRALin_y[threadIndex] = deltaV0.bottom.y;
					artiResponse->deltaRALin_z[threadIndex] = deltaV0.bottom.z;
					artiResponse->deltaRAAng_x[threadIndex] = deltaV0.top.x;
					artiResponse->deltaRAAng_y[threadIndex] = deltaV0.top.y;
					artiResponse->deltaRAAng_z[threadIndex] = deltaV0.top.z;

					artiResponse->deltaRBLin_x[threadIndex] = deltaV1.bottom.x;
					artiResponse->deltaRBLin_y[threadIndex] = deltaV1.bottom.y;
					artiResponse->deltaRBLin_z[threadIndex] = deltaV1.bottom.z;
					artiResponse->deltaRBAng_x[threadIndex] = deltaV1.top.x;
					artiResponse->deltaRBAng_y[threadIndex] = deltaV1.top.y;
					artiResponse->deltaRBAng_z[threadIndex] = deltaV1.top.z;
					artiResponse++;
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

					const Cm::UnAlignedSpatialVector response0 = createImpulseResponseVector(b0, Cm::UnAlignedSpatialVector(raXn, t1));
					const Cm::UnAlignedSpatialVector response1 = createImpulseResponseVector(b1, Cm::UnAlignedSpatialVector(-rbXn, -t1));

					const Cm::UnAlignedSpatialVector deltaV0 = getImpulseResponse(b0, Cm::UnAlignedSpatialVector(t1, raXn).scale(lin0X_ang0Y_lin1Z_ang1W.x, lin0X_ang0Y_lin1Z_ang1W.y));
					const Cm::UnAlignedSpatialVector deltaV1 = getImpulseResponse(b1, Cm::UnAlignedSpatialVector(-t1, -rbXn).scale(lin0X_ang0Y_lin1Z_ang1W.z, lin0X_ang0Y_lin1Z_ang1W.w));

					const float resp0 = deltaV0.dot(response0);
					const float resp1 = deltaV1.dot(response1);

					float targetVel = tvel.dot(t1);//V3Dot(V3LoadU(buffer.contacts[index].targetVel), t0);

					if (!b0.islandNodeIndex.isArticulation())
						targetVel -= projectVelocity(b0, Cm::UnAlignedSpatialVector(raXn, t1));
					if (!b1.islandNodeIndex.isArticulation())
						targetVel += projectVelocity(b1, Cm::UnAlignedSpatialVector(rbXn, t1));

					// Storing resp0 and resp1, separately.
					const float bias = t1.dot(error) * invDt;
					f1->raXn_bias[threadIndex] = make_float4(response0.top.x, response0.top.y, response0.top.z, bias);
					f1->rbXn_targetVelW[threadIndex] = make_float4(-response1.top.x, -response1.top.y, -response1.top.z, targetVel);
					f1->appliedForce[threadIndex] = 0.f;
					f1->resp0[threadIndex] = resp0;
					f1->resp1[threadIndex] = resp1;

					artiResponse->deltaRALin_x[threadIndex] = deltaV0.bottom.x;
					artiResponse->deltaRALin_y[threadIndex] = deltaV0.bottom.y;
					artiResponse->deltaRALin_z[threadIndex] = deltaV0.bottom.z;
					artiResponse->deltaRAAng_x[threadIndex] = deltaV0.top.x;
					artiResponse->deltaRAAng_y[threadIndex] = deltaV0.top.y;
					artiResponse->deltaRAAng_z[threadIndex] = deltaV0.top.z;

					artiResponse->deltaRBLin_x[threadIndex] = deltaV1.bottom.x;
					artiResponse->deltaRBLin_y[threadIndex] = deltaV1.bottom.y;
					artiResponse->deltaRBLin_z[threadIndex] = deltaV1.bottom.z;
					artiResponse->deltaRBAng_x[threadIndex] = deltaV1.top.x;
					artiResponse->deltaRBAng_y[threadIndex] = deltaV1.top.y;
					artiResponse->deltaRBAng_z[threadIndex] = deltaV1.top.z;
					artiResponse++;
				}
			}
		}

		frictionPatchWritebackAddrIndex++;
	}
}


//for TGS solver(invDt is the step invDt and invTotalDt is the total invDt)
static __device__ void setupFinalizeExtSolverConstraintsBlock(PxgBlockContactData& contactData,
	const PxgBlockContactPoint* PX_RESTRICT contacts,
	const PxU32 contactCount,
	const PxgBlockFrictionPatch& frictionPatch,
	const PxgBlockFrictionAnchorPatch& fAnchor,
	const PxgSolverExtBody2& b0,
	const PxgSolverExtBody2& b1,
	const PxReal invDt,
	const PxReal stepDt,
	const PxReal invTotalDt,
	const PxReal bounceThresholdF32,
	const PxReal biasCoefficient,
	const PxU32 threadIndex,
	PxU32 forceWritebackBufferOffset,
	const bool perPointFriction,
	PxgTGSContactBlockParams& params,
	PxgArticulationBlockResponse* PX_RESTRICT articulationResponses,
	const PxgMaterialContactData& data,
	const PxReal ccdMaxSeparationThreshold,
	const PxReal solverOffsetSlop,
	const float2 torsionalFrictionData,
	const PxReal totalDt)
{
	using namespace physx;
	// NOTE II: the friction patches are sparse (some of them have no contact patches, and
	// therefore did not get written back to the cache) but the patch addresses are dense,
	// corresponding to valid patches

	/*const float4 data_ = reinterpret_cast<float4&>(contactData.contactData[threadIndex]);
	const PxgMaterialContactData data = reinterpret_cast<const PxgMaterialContactData&>(data_);*/

	PxU8 flags = data.mSolverFlags;

	const float restDistance = data.restDistance;//FLoad(n.restDistance); 

	const PxVec3& bodyFrame0p = b0.body2World.p;

	const PxAlignedQuat& bodyFrame1q = b1.body2World.q;//QuatVLoadU(&bodyFrame1.q.x);
	const PxVec3& bodyFrame1p = b1.body2World.p;

	uint32_t frictionPatchWritebackAddrIndex = 0;
	uint32_t contactWritebackCount = 0;

	const PxReal cfm = PxMax(b0.cfm, b1.cfm);

	const float maxPenBias = fmaxf(b0.penBiasClamp, b1.penBiasClamp);
	//printf("%i: MaxPenBias = %f\n", threadIndex, maxPenBias);

	const float p8 = PxMin(0.8f, biasCoefficient);
	const float bounceThreshold = bounceThresholdF32;

	const float invDtp8 = invDt * p8;//FMul(invDt, p8);

	const float4 lin0X_ang0Y_lin1Z_ang1W = contactData.mInvMassScale[threadIndex].lin0X_ang0Y_lin1Z_ang1W;

	const PxVec3& linVel0 = b0.velocity.bottom;
	const PxVec3& linVel1 = b1.velocity.bottom;
	const PxVec3& angVel0 = b0.velocity.top;
	const PxVec3& angVel1 = b1.velocity.top;

	{
		const PxU32 aCount = frictionPatch.anchorCount[threadIndex];
		PxU32 anchorCount = aCount * 2;

		//If we have just 1 anchor and torsional settings are enabled, we have torsional friction.
		const bool hasTorsional = (torsionalFrictionData.x != 0.f || torsionalFrictionData.y != 0.f) && aCount == 1;

		if (hasTorsional)
			anchorCount++;

		//shared memory counter for max contacts in the friction patch...

		if (contactCount == 0)
		{
			//printf("%i: Not Prepping %i contacts\n", threadIndex, contactCount);
			params.blockFrictionHeader->numFrictionConstr[threadIndex] = 0;
			params.blockContactHeader->numNormalConstr[threadIndex] = 0;
			params.blockContactHeader->forceWritebackOffset[threadIndex] = 0xffffffff;
			params.blockContactHeader->invMass0_1_angDom0_1[threadIndex] = make_float4(0.0f);
			return;
		}

		//printf("Prepping %i contacts!\n", contactCount);
		PxReal maxImpulse = PxMin(b0.maxImpulse, b1.maxImpulse);

		params.blockContactHeader->forceWritebackOffset[threadIndex] = forceWritebackBufferOffset;

		params.blockContactHeader->flags[threadIndex] = flags;
		
		params.blockContactHeader->maxPenBias[threadIndex] = maxPenBias;

		//KS - despite the variable name, this only stores the invMassScale terms for extended contacts
		params.blockContactHeader->invMass0_1_angDom0_1[threadIndex] = make_float4(lin0X_ang0Y_lin1Z_ang1W.x, lin0X_ang0Y_lin1Z_ang1W.z, lin0X_ang0Y_lin1Z_ang1W.y, lin0X_ang0Y_lin1Z_ang1W.w);

		const float4 normal4_rW = contactData.normal_restitutionW[threadIndex];
		const PxVec3 normal(normal4_rW.x, normal4_rW.y, normal4_rW.z);
		const PxReal restitution = normal4_rW.w;
		const PxReal normalLenSq = normal.magnitudeSquared();//V3LengthSq(normal);
		const PxReal norVel0 = normal.dot(linVel0);//V3Dot(normal, linVel0);
		const PxReal norVel1 = normal.dot(linVel1);//V3Dot(normal, linVel1);
		const PxReal linNorVel = norVel0 - norVel1;
		const PxReal damping = contactData.damping[threadIndex];

		params.blockFrictionHeader->biasCoefficient[threadIndex] = invDt;

		params.blockContactHeader->restitutionXdt[threadIndex] = restitution * stepDt;
		params.blockContactHeader->cfm[threadIndex] = cfm;
		params.blockContactHeader->p8[threadIndex] = p8;

		PxgArticulationBlockResponse* PX_RESTRICT artiResponse = articulationResponses;

		//printf("%i: Prepping %i contacts\n", threadIndex, contactCount);

		PxReal normalForce = 0.f;

		PxReal minPen = 0.f;
		
		for (PxU32 j = 0; j < contactCount; j++)
		{
			//printf("Contact %i\n", j);
			const PxgBlockContactPoint& contact = contacts[j];

			PxgTGSBlockSolverContactPoint* solverContact = &params.blockContactPoints[j];

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

			//printf("%i: normal = (%f, %f, %f), raXn = (%f, %f, %f), rbXn = (%f, %f, %f)\n", threadIndex, normal.x, normal.y, normal.z, raXn.x, raXn.y, raXn.z,
			//	rbXn.x, rbXn.y, rbXn.z);

			const Cm::UnAlignedSpatialVector response0 = createImpulseResponseVector(b0, Cm::UnAlignedSpatialVector(raXn, normal));
			const Cm::UnAlignedSpatialVector response1 = createImpulseResponseVector(b1, Cm::UnAlignedSpatialVector(-rbXn, -normal));

			//printf("%i: response0 = (%f, %f, %f, %f, %f, %f), response1 = (%f, %f, %f, %f, %f, %f)\n", 
			//	threadIndex, response0.top.x, response0.top.y, response0.top.z, response0.bottom.x, response0.bottom.y, response0.bottom.z,
			//	response1.top.x, response1.top.y, response1.top.z, response1.bottom.x, response1.bottom.y, response1.bottom.z);

			const Cm::UnAlignedSpatialVector deltaV0 = getImpulseResponse(b0, Cm::UnAlignedSpatialVector(normal, raXn).scale(lin0X_ang0Y_lin1Z_ang1W.x, lin0X_ang0Y_lin1Z_ang1W.y));
			const Cm::UnAlignedSpatialVector deltaV1 = getImpulseResponse(b1, Cm::UnAlignedSpatialVector(-normal, -rbXn).scale(lin0X_ang0Y_lin1Z_ang1W.z, lin0X_ang0Y_lin1Z_ang1W.w));

			//printf("%i: DeltaV0 = (%f, %f, %f, %f, %f, %f), deltaV1 = (%f, %f, %f, %f, %f, %f)\n", threadIndex, deltaV0.top.x, deltaV0.top.y, deltaV0.top.z,
			//	deltaV0.bottom.x, deltaV0.bottom.y, deltaV0.bottom.z, deltaV1.top.x, deltaV1.top.y, deltaV1.top.z,
			//	deltaV1.bottom.x, deltaV1.bottom.y, deltaV1.bottom.z);

			const float resp0 = deltaV0.dot(response0);
			const float resp1 = deltaV1.dot(response1);

			const float unitResponse = resp0 + resp1;//FAdd(resp0, resp1);

			const float vrel1 = norVel0 + projectAngular(b0, raXn);//FAdd(norVel0, V3Dot(raXn, angVel0));
			const float vrel2 = norVel1 + projectAngular(b1, rbXn);//FAdd(norVel1, V3Dot(rbXn, angVel1));
			const float vrel = vrel1 - vrel2;//FSub(vrel1, vrel2);

			//printf("%i: vrel = %f, vrel1 = %f, vrel2 = %f\n", threadIndex, vrel, vrel1, vrel2);

			const float penetration = separation - restDistance;
			const bool isSeparated = (penetration > 0.0f);
			const float penetrationInvDt = penetration * invTotalDt;

			float scaledBias, velMultiplier;

			const float recipResponse = (unitResponse > 0.f) ? (1.f / (unitResponse + cfm)) : 0.f;

			// To compute velMultiplier and scaledBias every sub-timestep or iteration, additional data is stored:
			// coeff0, coeff1. Using coeff0 and coeff1 produces the same results as the previous implementation when
			// mass-splitting does not occur.

			PxReal compliantContactCoeff = 0.f;

			if (restitution < 0.f)
			{
				const bool collidingWithVrel = ((-vrel * totalDt) > penetration); // Note: using totalDt here instead of penetrationInvDt because the latter has a fudge factor if there are velocity iterations
				compliantContactCoeff = computeCompliantContactCoefficientsTGS(stepDt, flags, restitution, damping, unitResponse, recipResponse,
					isSeparated, collidingWithVrel, velMultiplier, scaledBias);
			}
			else
			{
				velMultiplier = recipResponse;
				scaledBias = isSeparated ? -invDt : -invDtp8;
			}

			solverContact->resp0[threadIndex] = resp0;
			solverContact->resp1[threadIndex] = resp1;

			minPen = PxMin(penetration, minPen);

			//const float penetrationInvDtPt8 = fmaxf(maxPenBias, penetration*invDtp8);//FMax(maxPenBias, FMul(penetration, invDtp8));

			//printf("%i: Velmultipler =  %f, penetrationInvDtPt8 = %f, invDt %f, vrel %f\n", threadIndex, velMultiplier, penetrationInvDt, invTotalDt, vrel);

			const bool isGreater2 = ((restitution > 1e-5f) && (bounceThreshold > vrel)) && ((-vrel) > penetrationInvDt);

			float totalError = penetration;

			float targetVelocity = cTargetVel + isGreater2 ? (-vrel)*restitution : 0.f;//FAdd(cTargetVel, FSel(isGreater2, FMul(FNeg(sumVRel), restitution), zero));

			const PxReal deltaF = fmaxf((targetVelocity + (-penetrationInvDt - vrel)) * velMultiplier, 0.f);
			normalForce += deltaF;

			if (b0.isKinematic)
				targetVelocity -= vrel1;
			if (b1.isKinematic)
				targetVelocity += vrel2;

			if (isGreater2)
			{
				const PxReal ratio = totalDt + penetration/vrel;
				totalError += ratio * targetVelocity;
			}

			solverContact->raXn_extraCoeff[threadIndex] = make_float4(response0.top.x, response0.top.y, response0.top.z, compliantContactCoeff);
			solverContact->rbXn_targetVelW[threadIndex] = make_float4(-response1.top.x, -response1.top.y, -response1.top.z, targetVelocity);
			solverContact->appliedForce[threadIndex] = 0.f;
			solverContact->maxImpulse[threadIndex] = PxMin(maxImpulse, targetVel4_maxImpulseW.w);
			solverContact->separation[threadIndex] = totalError;
			solverContact->biasCoefficient[threadIndex] = scaledBias;

			artiResponse->deltaRALin_x[threadIndex] = deltaV0.bottom.x;
			artiResponse->deltaRALin_y[threadIndex] = deltaV0.bottom.y;
			artiResponse->deltaRALin_z[threadIndex] = deltaV0.bottom.z;
			artiResponse->deltaRAAng_x[threadIndex] = deltaV0.top.x;
			artiResponse->deltaRAAng_y[threadIndex] = deltaV0.top.y;
			artiResponse->deltaRAAng_z[threadIndex] = deltaV0.top.z;
			artiResponse->deltaRBLin_x[threadIndex] = deltaV1.bottom.x;
			artiResponse->deltaRBLin_y[threadIndex] = deltaV1.bottom.y;
			artiResponse->deltaRBLin_z[threadIndex] = deltaV1.bottom.z;
			artiResponse->deltaRBAng_x[threadIndex] = deltaV1.top.x;
			artiResponse->deltaRBAng_y[threadIndex] = deltaV1.top.y;
			artiResponse->deltaRBAng_z[threadIndex] = deltaV1.top.z;
			artiResponse++;
		}

		params.blockContactHeader->minNormalForce[threadIndex] = (normalForce / contactCount)*0.25f;

	
		contactWritebackCount += contactCount;

		

		PxReal frictionScale = (aCount == 2) ? 0.5f : 1.f;

		const PxReal staticFriction = data.staticFriction*frictionScale;
		const PxReal dynamicFriction = data.dynamicFriction*frictionScale;

		const bool haveFriction = (anchorCount != 0);//PX_IR(n.staticFriction) > 0 || PX_IR(n.dynamicFriction) > 0;
		params.blockContactHeader->numNormalConstr[threadIndex] = contactCount;
		params.blockFrictionHeader->numFrictionConstr[threadIndex] = anchorCount;

		//header->type				= type;

		params.blockContactHeader->normal_staticFriction[threadIndex] = make_float4(normal.x, normal.y, normal.z, staticFriction);
		params.blockFrictionHeader->dynamicFriction[threadIndex] = dynamicFriction;

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
			PxVec3 t0Fallback = orthoThreshold > PxAbs(normalX) ? t0Fallback1 : t0Fallback2;//V3Sel(FIsGrtr(orthoThreshold, FAbs(normalX)), t0Fallback1, t0Fallback2);

			PxVec3 t0 = vrel - normal * (normal.dot(vrel));//V3Sub(vrel, V3Scale(normal, V3Dot(normal, vrel)));
			t0 = t0.magnitudeSquared() > p1 ? t0 : t0Fallback;//V3Sel(FIsGrtr(V3LengthSq(t0), p1), t0, t0Fallback);
			t0.normalize();

			const PxVec3 t1 = normal.cross(t0);

			params.blockFrictionHeader->broken[threadIndex] = 0;

			params.blockFrictionHeader->frictionNormals[0][threadIndex] = make_float4(t0.x, t0.y, t0.z, 0.f);
			params.blockFrictionHeader->frictionNormals[1][threadIndex] = make_float4(t1.x, t1.y, t1.z, 0.f);

			const PxVec3 relTr = bodyFrame0p - bodyFrame1p;

			for (PxU32 j = 0; j < aCount; j++)
			{
				PxgTGSBlockSolverContactFriction* PX_RESTRICT f0 = &params.blockFrictions[2 * j];
				PxgTGSBlockSolverContactFriction* PX_RESTRICT f1 = &params.blockFrictions[2 * j + 1];

				const float4 body0Anchor4 = fAnchor.body0Anchors[j][threadIndex];
				const float4 body1Anchor4 = fAnchor.body1Anchors[j][threadIndex];

				const PxVec3 body0Anchor(body0Anchor4.x, body0Anchor4.y, body0Anchor4.z);
				const PxVec3 body1Anchor(body1Anchor4.x, body1Anchor4.y, body1Anchor4.z);

				const PxVec3 ra = b0.body2World.q.rotate(body0Anchor);//QuatRotate(bodyFrame0q, body0Anchor);
				const PxVec3 rb = b1.body2World.q.rotate(body1Anchor);//QuatRotate(bodyFrame1q, body1Anchor);

				//const PxVec3 error = (ra + bodyFrame0p) - (rb + bodyFrame1p);//V3Sub(V3Add(ra, bodyFrame0p), V3Add(rb, bodyFrame1p));

				PxU32 index = perPointFriction ? frictionPatch.contactID[j][threadIndex] : 0;

				const float4 targetVel4 = contacts[index].targetVel_maxImpulseW[threadIndex];
				const PxVec3 tvel(targetVel4.x, targetVel4.y, targetVel4.z);

				const PxVec3 error = (ra - rb) + relTr;

				{
					PxVec3 raXn = ra.cross(t0);//V3Cross(ra, t0Cross);
					PxVec3 rbXn = rb.cross(t0);//V3Cross(rb, t0Cross);

					raXn.x = PxAbs(raXn.x) < solverOffsetSlop ? 0.f : raXn.x;
					raXn.y = PxAbs(raXn.y) < solverOffsetSlop ? 0.f : raXn.y;
					raXn.z = PxAbs(raXn.z) < solverOffsetSlop ? 0.f : raXn.z;

					rbXn.x = PxAbs(rbXn.x) < solverOffsetSlop ? 0.f : rbXn.x;
					rbXn.y = PxAbs(rbXn.y) < solverOffsetSlop ? 0.f : rbXn.y;
					rbXn.z = PxAbs(rbXn.z) < solverOffsetSlop ? 0.f : rbXn.z;

					const Cm::UnAlignedSpatialVector response0 = createImpulseResponseVector(b0, Cm::UnAlignedSpatialVector(raXn, t0));
					const Cm::UnAlignedSpatialVector response1 = createImpulseResponseVector(b1, Cm::UnAlignedSpatialVector(-rbXn, -t0));

					const Cm::UnAlignedSpatialVector deltaV0 = getImpulseResponse(b0, Cm::UnAlignedSpatialVector(t0, raXn).scale(lin0X_ang0Y_lin1Z_ang1W.x, lin0X_ang0Y_lin1Z_ang1W.y));
					const Cm::UnAlignedSpatialVector deltaV1 = getImpulseResponse(b1, Cm::UnAlignedSpatialVector(-t0, -rbXn).scale(lin0X_ang0Y_lin1Z_ang1W.z, lin0X_ang0Y_lin1Z_ang1W.w));

					//printf("%i: deltaV0 = (%f, %f, %f, %f, %f, %f)\n", b0.linkIndex, deltaV0.top.x, deltaV0.top.y, deltaV0.top.z,
					//	deltaV0.bottom.x, deltaV0.bottom.y, deltaV0.bottom.z);

					const float resp0 = deltaV0.dot(response0);
					const float resp1 = deltaV1.dot(response1);

					float targetVel = tvel.dot(t0);//V3Dot(V3LoadU(buffer.contacts[index].targetVel), t0);

					if (b0.isKinematic)
						targetVel -= projectVelocity(b0, Cm::UnAlignedSpatialVector(raXn, t0));
					if (b1.isKinematic)
						targetVel += projectVelocity(b1, Cm::UnAlignedSpatialVector(rbXn, t0));

					//printf("%i: Normal = (%f, %f, %f), raXn = (%f, %f, %f), Friction velMultiplier = %f, biasedErr = %f, targetVel = %f\n", 
					//	b0.linkIndex, t0.x, t0.y, t0.z, raXn.x, raXn.y, raXn.z, velMultiplier, t0.dot(error)*invDt, targetVel);
					
					const float frictionError = error.dot(t0);

					f0->resp0[threadIndex] = resp0;
					f0->resp1[threadIndex] = resp1;
					f0->raXn_error[threadIndex] = make_float4(response0.top.x, response0.top.y, response0.top.z, frictionError);
					f0->rbXn_targetVelW[threadIndex] = make_float4(-response1.top.x, -response1.top.y, -response1.top.z, targetVel);
					f0->appliedForce[threadIndex] = 0.f;

					artiResponse->deltaRALin_x[threadIndex] = deltaV0.bottom.x;
					artiResponse->deltaRALin_y[threadIndex] = deltaV0.bottom.y;
					artiResponse->deltaRALin_z[threadIndex] = deltaV0.bottom.z;
					artiResponse->deltaRAAng_x[threadIndex] = deltaV0.top.x;
					artiResponse->deltaRAAng_y[threadIndex] = deltaV0.top.y;
					artiResponse->deltaRAAng_z[threadIndex] = deltaV0.top.z;

					artiResponse->deltaRBLin_x[threadIndex] = deltaV1.bottom.x;
					artiResponse->deltaRBLin_y[threadIndex] = deltaV1.bottom.y;
					artiResponse->deltaRBLin_z[threadIndex] = deltaV1.bottom.z;
					artiResponse->deltaRBAng_x[threadIndex] = deltaV1.top.x;
					artiResponse->deltaRBAng_y[threadIndex] = deltaV1.top.y;
					artiResponse->deltaRBAng_z[threadIndex] = deltaV1.top.z;
					artiResponse++;
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

					const Cm::UnAlignedSpatialVector response0 = createImpulseResponseVector(b0, Cm::UnAlignedSpatialVector(raXn, t1));
					const Cm::UnAlignedSpatialVector response1 = createImpulseResponseVector(b1, Cm::UnAlignedSpatialVector(-rbXn, -t1));

					const Cm::UnAlignedSpatialVector deltaV0 = getImpulseResponse(b0, Cm::UnAlignedSpatialVector(t1, raXn).scale(lin0X_ang0Y_lin1Z_ang1W.x, lin0X_ang0Y_lin1Z_ang1W.y));
					const Cm::UnAlignedSpatialVector deltaV1 = getImpulseResponse(b1, Cm::UnAlignedSpatialVector(-t1, -rbXn).scale(lin0X_ang0Y_lin1Z_ang1W.z, lin0X_ang0Y_lin1Z_ang1W.w));

					const float resp0 = deltaV0.dot(response0);
					const float resp1 = deltaV1.dot(response1);

					float targetVel = tvel.dot(t1);//V3Dot(V3LoadU(buffer.contacts[index].targetVel), t0);

					if (b0.isKinematic)
						targetVel -= projectVelocity(b0, Cm::UnAlignedSpatialVector(raXn, t1));
					if (b1.isKinematic)
						targetVel += projectVelocity(b1, Cm::UnAlignedSpatialVector(rbXn, t1));

					//printf("%i: deltaV0 = (%f, %f, %f, %f, %f, %f)\n", b0.linkIndex, deltaV0.top.x, deltaV0.top.y, deltaV0.top.z,
					//	deltaV0.bottom.x, deltaV0.bottom.y, deltaV0.bottom.z);

					//printf("%i: Normal = (%f, %f, %f), raXn = (%f, %f, %f), Friction velMultiplier = %f, biasedErr = %f, targetVel = %f\n",
					//	b0.linkIndex, t0.x, t0.y, t0.z, raXn.x, raXn.y, raXn.z, velMultiplier, t1.dot(error)*invDt, targetVel);

					const float frictionError = error.dot(t1);
					f1->resp0[threadIndex] = resp0;
					f1->resp1[threadIndex] = resp1;
					f1->raXn_error[threadIndex] = make_float4(response0.top.x, response0.top.y, response0.top.z, frictionError);
					f1->rbXn_targetVelW[threadIndex] = make_float4(-response1.top.x, -response1.top.y, -response1.top.z, targetVel);
					f1->appliedForce[threadIndex] = 0.f;

					artiResponse->deltaRALin_x[threadIndex] = deltaV0.bottom.x;
					artiResponse->deltaRALin_y[threadIndex] = deltaV0.bottom.y;
					artiResponse->deltaRALin_z[threadIndex] = deltaV0.bottom.z;
					artiResponse->deltaRAAng_x[threadIndex] = deltaV0.top.x;
					artiResponse->deltaRAAng_y[threadIndex] = deltaV0.top.y;
					artiResponse->deltaRAAng_z[threadIndex] = deltaV0.top.z;
					artiResponse->deltaRBLin_x[threadIndex] = deltaV1.bottom.x;
					artiResponse->deltaRBLin_y[threadIndex] = deltaV1.bottom.y;
					artiResponse->deltaRBLin_z[threadIndex] = deltaV1.bottom.z;
					artiResponse->deltaRBAng_x[threadIndex] = deltaV1.top.x;
					artiResponse->deltaRBAng_y[threadIndex] = deltaV1.top.y;
					artiResponse->deltaRBAng_z[threadIndex] = deltaV1.top.z;
					artiResponse++;
				}
			}

			if (hasTorsional)
			{
				//Setup torsional...
				PxgTGSBlockSolverContactFriction* PX_RESTRICT f0 = &params.blockFrictions[2];

				const PxReal frictionScale = PxMax(torsionalFrictionData.y, PxSqrt(PxMax(0.f, -minPen)*torsionalFrictionData.x));

				params.blockFrictionHeader->torsionalFrictionScale[threadIndex] = frictionScale;


				const Cm::UnAlignedSpatialVector response0 = createImpulseResponseVector(b0, Cm::UnAlignedSpatialVector(normal, PxVec3(0.f)));
				const Cm::UnAlignedSpatialVector response1 = createImpulseResponseVector(b1, Cm::UnAlignedSpatialVector(-normal, PxVec3(0.f)));

				const Cm::UnAlignedSpatialVector deltaV0 = getImpulseResponse(b0, Cm::UnAlignedSpatialVector(PxVec3(0.f), normal).scale(lin0X_ang0Y_lin1Z_ang1W.x, lin0X_ang0Y_lin1Z_ang1W.y));
				const Cm::UnAlignedSpatialVector deltaV1 = getImpulseResponse(b1, Cm::UnAlignedSpatialVector(PxVec3(0.f), -normal).scale(lin0X_ang0Y_lin1Z_ang1W.z, lin0X_ang0Y_lin1Z_ang1W.w));

				//printf("%i: deltaV0 = (%f, %f, %f, %f, %f, %f)\n", b0.linkIndex, deltaV0.top.x, deltaV0.top.y, deltaV0.top.z,
				//	deltaV0.bottom.x, deltaV0.bottom.y, deltaV0.bottom.z);

				const float resp0 = deltaV0.dot(response0);
				const float resp1 = deltaV1.dot(response1);

				const float frictionError = 0.f;

				f0->resp0[threadIndex] = resp0;
				f0->resp1[threadIndex] = resp1;
				f0->raXn_error[threadIndex] = make_float4(response0.top.x, response0.top.y, response0.top.z, frictionError);
				f0->rbXn_targetVelW[threadIndex] = make_float4(-response1.top.x, -response1.top.y, -response1.top.z, 0.f);
				f0->appliedForce[threadIndex] = 0.f;

				artiResponse->deltaRALin_x[threadIndex] = deltaV0.bottom.x;
				artiResponse->deltaRALin_y[threadIndex] = deltaV0.bottom.y;
				artiResponse->deltaRALin_z[threadIndex] = deltaV0.bottom.z;
				artiResponse->deltaRAAng_x[threadIndex] = deltaV0.top.x;
				artiResponse->deltaRAAng_y[threadIndex] = deltaV0.top.y;
				artiResponse->deltaRAAng_z[threadIndex] = deltaV0.top.z;
				artiResponse->deltaRBLin_x[threadIndex] = deltaV1.bottom.x;
				artiResponse->deltaRBLin_y[threadIndex] = deltaV1.bottom.y;
				artiResponse->deltaRBLin_z[threadIndex] = deltaV1.bottom.z;
				artiResponse->deltaRBAng_x[threadIndex] = deltaV1.top.x;
				artiResponse->deltaRBAng_y[threadIndex] = deltaV1.top.y;
				artiResponse->deltaRBAng_z[threadIndex] = deltaV1.top.z;
				artiResponse++;
			}
		}

		frictionPatchWritebackAddrIndex++;
	}
}

template <typename ContactParams>
static __device__ void artiCreateFinalizeSolverContactsBlockGPU(PxgBlockContactData* contactData,
	const PxgBlockContactPoint* PX_RESTRICT contactPoints,
	PxgBlockFrictionPatch& frictionPatch,
	const PxgBlockFrictionPatch* PX_RESTRICT prevFrictionPatches,
	PxgBlockFrictionAnchorPatch& fAnchor,
	const PxgBlockFrictionAnchorPatch* PX_RESTRICT prevFrictionAnchors,
	const PxgBlockFrictionIndex* PX_RESTRICT prevFrictionIndices,
	const PxgSolverExtBody2& body0,
	const PxgSolverExtBody2& body1,
	const PxReal invDt,
	const PxReal dt,
	const PxReal invTotalDt,
	const PxReal bounceThresholdF32,
	const PxReal frictionOffsetThreshold,
	const PxReal correlationDistance,
	const PxReal biasCoefficient,
	const PxU32 threadIndex,
	PxU32 forceWritebackBufferOffset,
	ContactParams& params,
	PxgArticulationBlockResponse* PX_RESTRICT response,
	PxU32 totalEdges,
	PxU32 prevFrictionStartIndex,
	PxReal ccdMaxSeparation,
	PxReal solverOffsetSlop,
	const float2 torsionalFrictionData,
	const PxReal totalDt)
{
	//Load the body datas in using warp-wide programming...Each solver body data is 128 bytes long, so we can load both bodies in with 2 lots of 128 byte coalesced reads. 

	//printf("PxgMaterialContactData\n");
	const float4 data_ = reinterpret_cast<float4&>(contactData->contactData[threadIndex]);
	const PxgMaterialContactData data = reinterpret_cast<const PxgMaterialContactData&>(data_);

	const PxU32 nbContacts = data.mNumContacts;

	const float4 normal4 = contactData->normal_restitutionW[threadIndex];
	const PxVec3 normal(normal4.x, normal4.y, normal4.z);

	
	//printf("warpIndex %i idx %i Prepping %i contacts\n", warpIndex, threadIndex, nbContacts);

	//printf("correlatePatches\n");

	correlatePatches(frictionPatch, contactPoints, nbContacts, normal,
		body0.body2World, body1.body2World, PXC_SAME_NORMAL, threadIndex);

	PxU8 flags = data.mSolverFlags;
	bool perPointFriction = flags & (PxgSolverContactFlags::ePER_POINT_FRICTION);
	bool disableFriction = flags & PxgSolverContactFlags::eDISABLE_FRICTION;

	//KS - ensure that the friction patch broken bit is set to 0
	frictionPatch.broken[threadIndex] = 0;
	PxReal patchExtents;

	__syncwarp(); //Ensure writes from correlation are visible


	//printf("getFrictionPatches\n");
	if (!(perPointFriction || disableFriction))// || (solverBodyData0.islandNodeIndex & 2) || (solverBodyData1.islandNodeIndex & 2)))
	{
		getFrictionPatches(frictionPatch, fAnchor, prevFrictionIndices, prevFrictionStartIndex, prevFrictionPatches, 
			prevFrictionAnchors, data.prevFrictionPatchCount, body0.body2World, body1.body2World, correlationDistance, threadIndex, totalEdges, patchExtents,
			nbContacts);
	}

	if (!disableFriction)
		growPatches(frictionPatch, fAnchor, contactPoints, nbContacts, body0.body2World, body1.body2World, frictionOffsetThreshold + data.restDistance, threadIndex, patchExtents);

	//printf("setupFinalizeSolverConstraintsBlock\n");
	setupFinalizeExtSolverConstraintsBlock(*contactData, contactPoints, nbContacts,
		frictionPatch, fAnchor, body0, body1, invDt, dt, invTotalDt,
		bounceThresholdF32, biasCoefficient, threadIndex, forceWritebackBufferOffset, perPointFriction,
		params, response, data, ccdMaxSeparation, solverOffsetSlop, torsionalFrictionData, totalDt);

}

template<typename ContactParams, typename IterativeData>
__device__ void fillBlockParams(IterativeData& iterativeData, ContactParams& params,
	const PxU32 descIndexBatch, const PxU32 startConstraintIndex, const PxU32 startFrictionIndex)
{
	params.blockContactHeader = &iterativeData.blockContactHeaders[descIndexBatch];
	params.blockFrictionHeader = &iterativeData.blockFrictionHeaders[descIndexBatch];
	params.blockContactPoints = &iterativeData.blockContactPoints[startConstraintIndex];
	params.blockFrictions = &iterativeData.blockFrictions[startFrictionIndex];
}

template<typename ContactParams, typename IterativeData>
__device__ void artiContactConstraintBlockPrepare(
	PxgConstraintPrepareDesc* constraintPrepDesc,
	PxgSolverSharedDesc<IterativeData>* sharedDesc,
	const PxReal invDt, 
	const PxReal dt,
	const PxReal invTotalDt,
	const PxReal totalDt)
{
	PxgBlockWorkUnit* workUnits = constraintPrepDesc->blockWorkUnit;

	const PxU32 warpSize = WARP_SIZE;

	const PxU32 blockStride = blockDim.y;

	//This identifies which warp a specific thread is in, we treat all warps in all blocks as a flatten warp array
	//and we are going to index the work based on that
	const PxU32 warpIndex = blockIdx.x * blockStride + threadIdx.y;

	//This identifies which thread within a warp a specific thread is
	const PxU32 threadIndexInWarp = threadIdx.x&(warpSize - 1);

	assert(blockDim.x == WARP_SIZE);
	const PxU32 threadIndexInBlock = threadIdx.x + WARP_SIZE * threadIdx.y;

	//total numbers of warps in all blocks
	//const PxU32 totalNumWarps = blockStride * gridDim.x;

	//PxF32* baseForceStream = constraintPrepDesc->forceBuffer;

	const PxU32 totalPreviousEdges = constraintPrepDesc->totalPreviousEdges;
	const PxU32 totalCurrentEdges = constraintPrepDesc->totalCurrentEdges;
	//We need to check against both. Static contact batches are handled separately in the internal solver...
	const PxU32 nbContactBatches = constraintPrepDesc->numArtiContactBatches + constraintPrepDesc->numArtiStaticContactBatches;// + constraintPrepDesc->numArtiSelfContactBatches;
	const PxU32 nbContactBatchesWithSelf = nbContactBatches + constraintPrepDesc->numArtiSelfContactBatches;
	const PxU32 offset = constraintPrepDesc->numBatches; //Offset = number of non-articulation batches!

	__shared__ PxgSolverBodyData* solverBodyDatas;
	__shared__ PxgSolverTxIData* solverTxIDatas;

	__shared__ PxgArticulationBlockResponse* responses;
	__shared__ PxU32* batchIndices;
	__shared__ PxgBlockFrictionIndex* frictionIndices;
	__shared__ PxgBlockFrictionIndex* prevFrictionIndices;
	__shared__ PxgBlockContactPoint* contactBase;
	__shared__ PxgBlockConstraintBatch* constraintBatch;
	__shared__ PxgBlockContactData* contactCurrentPrepPool;

	__shared__ PxgBlockFrictionPatch* prevFrictionPatches;
	__shared__ PxgBlockFrictionPatch* currFrictionPatches;
	__shared__ PxgBlockFrictionAnchorPatch* prevFrictionAnchors;
	__shared__ PxgBlockFrictionAnchorPatch* currFrictionAnchors;

	__shared__ PxAlignedTransform* bodyFrames;

	
	if (threadIndexInBlock == 0)
	{
		solverBodyDatas = constraintPrepDesc->solverBodyDataPool;
		solverTxIDatas = constraintPrepDesc->solverBodyTxIDataPool;

		//contactHeaders = sharedDesc->iterativeData.blockContactHeaders;
		//frictionHeaders = sharedDesc->iterativeData.blockFrictionHeaders;
		//contactPoints = sharedDesc->iterativeData.blockContactPoints;
		responses = sharedDesc->iterativeData.artiResponse;
		//frictions = sharedDesc->iterativeData.blockFrictions;
		batchIndices = constraintPrepDesc->artiContactConstraintBatchIndices;
		frictionIndices = constraintPrepDesc->blockCurrentFrictionIndices;
		prevFrictionIndices = constraintPrepDesc->blockPreviousFrictionIndices;

		contactBase = constraintPrepDesc->blockContactPoints;
		constraintBatch = sharedDesc->iterativeData.blockConstraintBatch;
		contactCurrentPrepPool = constraintPrepDesc->blockContactCurrentPrepPool;
		currFrictionPatches = sharedDesc->blockCurrentFrictionPatches;
		prevFrictionPatches = sharedDesc->blockPreviousFrictionPatches;
		prevFrictionAnchors = constraintPrepDesc->blockPreviousAnchorPatches;
		currFrictionAnchors = constraintPrepDesc->blockCurrentAnchorPatches;
		bodyFrames = constraintPrepDesc->body2WorldPool;

		//printf("offset %i\n", offset);
	}

	__syncthreads();

	PxU32 i = warpIndex;
	//unsigned mask_nbContactBatches = __ballot_sync(FULL_MASK, i < nbContactBatches);
	if (i < nbContactBatchesWithSelf)
	{
		const PxU32 batchIndex = batchIndices[i] + offset;

		
		PxgBlockConstraintBatch& batch = constraintBatch[batchIndex];

		const PxU32 descIndexBatch = batch.mConstraintBatchIndex;
		const PxU32 responseIndex = batch.mArticulationResponseIndex;

		const PxU32 descStride = batch.mDescStride;

		//printf("Prep: Index = %i, descStride = %i, descIndexInBatch = %i\n", threadIndexInWarp, descStride, descIndexBatch);

		
		if (threadIndexInWarp < descStride)
		{
			const PxNodeIndex nodeIndexA = batch.bodyANodeIndex[threadIndexInWarp];
			const PxNodeIndex nodeIndexB = batch.bodyBNodeIndex[threadIndexInWarp];

			const PxU32 bodyAIndex = nodeIndexA.isArticulation() ? batch.remappedBodyAIndex[threadIndexInWarp] : batch.bodyAIndex[threadIndexInWarp];
			const PxU32 bodyBIndex = nodeIndexB.isArticulation() ? batch.remappedBodyBIndex[threadIndexInWarp] : batch.bodyBIndex[threadIndexInWarp];

			ContactParams params;
			fillBlockParams<ContactParams, IterativeData>(sharedDesc->iterativeData, params, descIndexBatch, batch.startConstraintIndex, batch.startFrictionIndex);

			//printf("Index = %i, descStride = %i, descIndexInBatch = %i\n", threadIndexInWarp, descStride, descIndexBatch);
			
			//printf("currFrictionPatches = %p, currFrictionAnchors = %p\n", currFrictionPatches, currFrictionAnchors);
			//port contact code
			PxgBlockContactData& contactData = contactCurrentPrepPool[descIndexBatch];
			PxgBlockContactPoint* baseContact = contactBase + batch.blockContactIndex;
			//printf("ContactCurrentPrepPool = %p, batch.blockContactIndex = %i\n", contactCurrentPrepPool, batch.blockContactIndex);
			PxgBlockFrictionPatch& frictionPatch = currFrictionPatches[descIndexBatch];
			PxgBlockFrictionAnchorPatch& fAnchor = currFrictionAnchors[descIndexBatch];
			//if(i >= nbContactBatches)
				/*printf("%i: currFrictionPatches = %p, currFrictionAnchors = %p, workUnits = %p, descIndexInBatch %i\n", i, currFrictionPatches, currFrictionAnchors,
				&workUnits[descIndexBatch], descIndexBatch);*/

			//Fill in correlation information for next frame...

			PxgBlockWorkUnit& unit = workUnits[descIndexBatch];

			PxgBlockFrictionIndex index;
			index.createPatchIndex(descIndexBatch, threadIndexInWarp);

			//printf("PxgBlockFrictionIndex filled in unit.mEdgeIndex = %i, unit.mPatchIndex = %i\n", unit.mEdgeIndex[threadIndexInWarp],
			//	unit.mPatchIndex[threadIndexInWarp]);

			//PxU32 frictionIndex = unit.mFrictionIndex[threadIndexInWarp];
			PxU32 edgeIndex = unit.mEdgeIndex[threadIndexInWarp];
			PxU32 frictionIndex = edgeIndex + totalCurrentEdges * unit.mPatchIndex[threadIndexInWarp];

			//printf("EdgeIndex = %i, frictionIndex = i, frictionIndices = %p\n", edgeIndex, frictionIndex, frictionIndices);

			PxgBlockFrictionIndex* targetIndex = &frictionIndices[frictionIndex];

			//printf("edgeIndex = %i, frictionIndex = %i\n", edgeIndex, frictionIndex);

			*reinterpret_cast<uint2*>(targetIndex) = reinterpret_cast<uint2&>(index);

			PxgSolverExtBody2 b0, b1;

			createPxgSolverExtBody(nodeIndexA, bodyAIndex, sharedDesc->articulations, solverBodyDatas, solverTxIDatas, b0, bodyFrames);
			createPxgSolverExtBody(nodeIndexB, bodyBIndex, sharedDesc->articulations, solverBodyDatas, solverTxIDatas, b1, bodyFrames);

			PxReal offsetSlop = PxMax(b0.offsetSlop, b1.offsetSlop);

			//printf("Created solverExtBodies\n");

			PxU32 offset = unit.mWriteback[threadIndexInWarp];
			/*artiCreateFinalizeSolverContactsBlockGPU(&contactData, baseContact, frictionPatch, prevFrictionPatches, fAnchor, prevFrictionAnchors, prevFrictionIndices, 
				b0, b1, sharedDesc->invDtF32, constraintPrepDesc->bounceThresholdF32, constraintPrepDesc->frictionOffsetThreshold, constraintPrepDesc->correlationDistance,
				threadIndexInWarp, offset, &contactHeaders[descIndexBatch], &frictionHeaders[descIndexBatch], &contactPoints[batch.startConstraintIndex],
				&frictions[batch.startFrictionIndex], &responses[responseIndex], totalPreviousEdges, edgeIndex, constraintPrepDesc->ccdMaxSeparation, constraintPrepDesc->solverOffsetSlop);*/
			//if (i<nbContactBatches)
			artiCreateFinalizeSolverContactsBlockGPU(&contactData, baseContact, frictionPatch, prevFrictionPatches, fAnchor, prevFrictionAnchors, prevFrictionIndices,
				b0, b1, invDt, dt, invTotalDt, constraintPrepDesc->bounceThresholdF32, constraintPrepDesc->frictionOffsetThreshold, constraintPrepDesc->correlationDistance,
				constraintPrepDesc->biasCoefficient, threadIndexInWarp, offset, params, &responses[responseIndex], totalPreviousEdges, edgeIndex, constraintPrepDesc->ccdMaxSeparation, offsetSlop,
				unit.mTorsionalFrictionData[threadIndexInWarp], totalDt);

			frictionPatch.patchIndex[threadIndexInWarp] = unit.mFrictionPatchIndex[threadIndexInWarp];

			PxgBlockFrictionPatch& fpatch = frictionPatch;
			if (fpatch.anchorCount[threadIndexInWarp] >= 1)
				fpatch.anchorPoints[0][threadIndexInWarp] = PxSave3(b0.body2World.transform(PxLoad3(fAnchor.body0Anchors[0][threadIndexInWarp])));
			if (fpatch.anchorCount[threadIndexInWarp] == 2)
				fpatch.anchorPoints[1][threadIndexInWarp] = PxSave3(b0.body2World.transform(PxLoad3(fAnchor.body0Anchors[1][threadIndexInWarp])));
		}
	}

}

extern "C" __global__ void artiContactConstraintBlockPrepareLaunch(
	PxgConstraintPrepareDesc* constraintPrepDesc,
	PxgSolverSharedDesc<IterativeSolveData>* sharedDesc)
{
	artiContactConstraintBlockPrepare<PxgContactBlockParams, IterativeSolveData>(constraintPrepDesc, sharedDesc, sharedDesc->invDtF32, sharedDesc->dt,
		sharedDesc->invDtF32, sharedDesc->dt);
}

extern "C" __global__ void artiTGSContactConstraintBlockPrepareLaunch(
	PxgConstraintPrepareDesc* constraintPrepDesc,
	PxgSolverSharedDesc<IterativeSolveDataTGS>* sharedDesc)
{
	artiContactConstraintBlockPrepare<PxgTGSContactBlockParams, IterativeSolveDataTGS>(constraintPrepDesc, sharedDesc, sharedDesc->stepInvDtF32, sharedDesc->stepDt,
		sharedDesc->invDtF32, sharedDesc->dt);
}

static __device__ void preprocessRowsBlock(
	PxU32* sortedRowIndices, 
	PxgBlockConstraint1DData* constraintData,
	PxgBlockConstraint1DVelocities* rowVelocities, 
	PxgBlockConstraint1DParameters* rowParameters,
	const PxU32 threadIndex)
{
	using namespace physx;

	const PxU32 numRows = constraintData->mNumRows[threadIndex];

	for (PxU32 i = 0; i < numRows; i++)
	{
		PxgBlockConstraint1DParameters& r = rowParameters[i];

		PxU32 j = i;
		if (j > 0)
			assert(threadIndex < 32); //PxgBlockConstraint1DParameters::solveHint has only 32 elements
		for (; j > 0 && r.solveHint[threadIndex] < rowParameters[sortedRowIndices[j - 1]].solveHint[threadIndex]; j--)
		{
			sortedRowIndices[j] = sortedRowIndices[j - 1];
		}

		sortedRowIndices[j] = i;
	}
}

//PGS 1D constraints
static __device__ PxU32 setUpArti1DConstraintBlock(
	const PxU32* PX_RESTRICT sortedRowIndices,
	PxgBlockConstraint1DData* PX_RESTRICT constraintData,
	PxgBlockConstraint1DVelocities* PX_RESTRICT rowVelocities,
	PxgBlockConstraint1DParameters* PX_RESTRICT rowParameters,
	PxgBlockSolverConstraint1DCon* PX_RESTRICT constraintsCon,
	PxgBlockSolverConstraint1DMod* PX_RESTRICT constraintsMod,
	PxgArticulationBlockResponse* PX_RESTRICT articulationResponses,
	float dt, float recipdt, 
	PxgSolverExtBody2& b0, PxgSolverExtBody2& b1,
	const physx::PxgSolverBodyPrepData* bodyData0, 
	const physx::PxgSolverBodyPrepData* bodyData1,
	const PxU32 threadIndex)
{
	using namespace physx;

	const PxReal erp = 1.0f;
	PxU32 outCount = 0;
	
	PxgArticulationBlockResponse* PX_RESTRICT artiResponse = articulationResponses;

	const PxU32 numRows = constraintData->mNumRows[threadIndex];

	const float4 lin0X_ang0Y_lin1Z_ang1W = constraintData->mInvMassScale[threadIndex].lin0X_ang0Y_lin1Z_ang1W;

	const PxReal cfm = PxMax(b0.cfm, b1.cfm);

	for (PxU32 i = 0; i < numRows; i++)
	{
		PxgBlockSolverConstraint1DCon& ccon = constraintsCon[outCount];
		PxgBlockSolverConstraint1DMod& cmod = constraintsMod[outCount];
		const PxU32 index = sortedRowIndices[i];
		PxgBlockConstraint1DParameters& rp = rowParameters[index];
		PxgBlockConstraint1DVelocities& rv = rowVelocities[index];

		const PxU32 rpFlags = rp.flags[threadIndex];

		PxReal driveScale = rpFlags & Px1DConstraintFlag::eHAS_DRIVE_LIMIT && constraintData->mFlags[threadIndex] & PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES ? fminf(dt, 1.0f) : 1.0f;

		const float4 c_linear0XYZ_geometricErrorW = rv.linear0XYZ_geometricErrorW[threadIndex];
		const float4 c_linear1XYZ_minImpulseW = rv.linear1XYZ_minImpulseW[threadIndex];
		const float4 c_angular0XYZ_velocityTargetW = rv.angular0XYZ_velocityTargetW[threadIndex];
		const float4 c_angular1XYZ_maxImpulseW = rv.angular1XYZ_maxImpulseW[threadIndex];

		const PxVec3 clin0(c_linear0XYZ_geometricErrorW.x, c_linear0XYZ_geometricErrorW.y, c_linear0XYZ_geometricErrorW.z);
		const PxVec3 clin1(c_linear1XYZ_minImpulseW.x, c_linear1XYZ_minImpulseW.y, c_linear1XYZ_minImpulseW.z);
		const PxVec3 cang0(c_angular0XYZ_velocityTargetW.x, c_angular0XYZ_velocityTargetW.y, c_angular0XYZ_velocityTargetW.z);
		const PxVec3 cang1(c_angular1XYZ_maxImpulseW.x, c_angular1XYZ_maxImpulseW.y, c_angular1XYZ_maxImpulseW.z);

		const PxReal minImpulse = c_linear1XYZ_minImpulseW.w * driveScale;
		const PxReal maxImpulse = c_angular1XYZ_maxImpulseW.w * driveScale;

		const Cm::UnAlignedSpatialVector response0 = createImpulseResponseVector(b0, Cm::UnAlignedSpatialVector(cang0, clin0));
		const Cm::UnAlignedSpatialVector response1 = createImpulseResponseVector(b1, Cm::UnAlignedSpatialVector(-cang1, -clin1));

		const Cm::UnAlignedSpatialVector deltaV0 = getImpulseResponse(b0, Cm::UnAlignedSpatialVector(clin0, cang0).scale(lin0X_ang0Y_lin1Z_ang1W.x, lin0X_ang0Y_lin1Z_ang1W.y));
		const Cm::UnAlignedSpatialVector deltaV1 = getImpulseResponse(b1, Cm::UnAlignedSpatialVector(-clin1, -cang1).scale(lin0X_ang0Y_lin1Z_ang1W.z, lin0X_ang0Y_lin1Z_ang1W.w));
		const float resp0 = deltaV0.dot(response0);
		const float resp1 = deltaV1.dot(response1);

		float unitResponse = resp0 + resp1;//FAdd(resp0, resp1);

		cmod.ang0Writeback[threadIndex] = cang0;

		if (unitResponse <= DY_ARTICULATION_MIN_RESPONSE)
			continue; //Degenerate constraint, can't be solved so skip to avoid computation later in the solver!

		artiResponse->deltaRALin_x[threadIndex] = deltaV0.bottom.x;
		artiResponse->deltaRALin_y[threadIndex] = deltaV0.bottom.y;
		artiResponse->deltaRALin_z[threadIndex] = deltaV0.bottom.z;
		artiResponse->deltaRAAng_x[threadIndex] = deltaV0.top.x;
		artiResponse->deltaRAAng_y[threadIndex] = deltaV0.top.y;
		artiResponse->deltaRAAng_z[threadIndex] = deltaV0.top.z;

		artiResponse->deltaRBLin_x[threadIndex] = deltaV1.bottom.x;
		artiResponse->deltaRBLin_y[threadIndex] = deltaV1.bottom.y;
		artiResponse->deltaRBLin_z[threadIndex] = deltaV1.bottom.z;
		artiResponse->deltaRBAng_x[threadIndex] = deltaV1.top.x;
		artiResponse->deltaRBAng_y[threadIndex] = deltaV1.top.y;
		artiResponse->deltaRBAng_z[threadIndex] = deltaV1.top.z;
		artiResponse++;

		const bool needNormalVel = (rpFlags & Px1DConstraintFlag::eRESTITUTION)
			|| ((rpFlags & Px1DConstraintFlag::eSPRING) &&  (rpFlags & Px1DConstraintFlag::eACCELERATION_SPRING));

		PxReal jointSpeedForRestitutionBounce = 0.0f;
		PxReal initJointSpeed = 0.0f;
		const bool b0IsRigidDynamic = (b0.linkIndex == PxSolverConstraintDesc::RIGID_BODY);
		const bool b1IsRigidDynamic = (b1.linkIndex == PxSolverConstraintDesc::RIGID_BODY);
		if (needNormalVel || b0IsRigidDynamic || b1IsRigidDynamic)
		{
			const PxReal vel0 = projectVelocity(b0, Cm::UnAlignedSpatialVector(cang0, clin0));
			const PxReal vel1 = projectVelocity(b1, Cm::UnAlignedSpatialVector(cang1, clin1));

			Dy::computeJointSpeedPGS(vel0, b0IsRigidDynamic, vel1, b1IsRigidDynamic, jointSpeedForRestitutionBounce, initJointSpeed);
		}

		// Raising flags for spring and acceleration spring.
		// This is needed for computing contact coefficients every sub-timestep or iteration.
		// See "queryReduced1dConstraintSolverConstantsPGS" and "compute1dConstraintSolverConstantsPGS".
		if (rpFlags & Px1DConstraintFlag::eSPRING)
			cmod.flags[threadIndex] |= DY_SC_FLAG_SPRING;

		if (rpFlags & Px1DConstraintFlag::eACCELERATION_SPRING)
			cmod.flags[threadIndex] |= DY_SC_FLAG_ACCELERATION_SPRING;

		intializeBlock1D(rv, rp, jointSpeedForRestitutionBounce, initJointSpeed, resp0, resp1, erp, dt, recipdt, ccon, cmod,
			response0.bottom, -response1.bottom, response0.top, -response1.top, minImpulse, maxImpulse, cfm, threadIndex);

		if (rpFlags & Px1DConstraintFlag::eOUTPUT_FORCE)
			cmod.flags[threadIndex] |= DY_SC_FLAG_OUTPUT_FORCE;

		outCount++;
	}

	return outCount;
}




static __device__ void intializeTGSBlock1D(
	const physx::PxgBlockConstraint1DVelocities& rv,
	const physx::PxgBlockConstraint1DParameters& rp,
	const float jointSpeedForRestitutionBounce, const PxReal initJointSpeed,
	const float resp0, const float resp1, const float erp,
	const float stepDt, const float simDt, const float recipStepDt, const float recipSimDt,
	const float lengthScale, const PxReal minRowResponse,
	const PxVec3& _linear0, const PxVec3& _linear1,
	const PxVec3& _angular0, const PxVec3& _angular1,
	const PxU32 threadIndex,
	PxgTGSBlockSolverConstraint1DCon& scon,
	PxReal cfm)
{
	using namespace physx;

	PxReal maxBiasVelocity;
	{	
		const PxU16 flags = PxU16(rp.flags[threadIndex]);
		const PxReal stiffness = rp.mods.spring.stiffness[threadIndex];
		const PxReal damping = rp.mods.spring.damping[threadIndex];
		const PxReal restitution = rp.mods.bounce.restitution[threadIndex];
		const PxReal bounceVelocityThreshold = rp.mods.bounce.velocityThreshold[threadIndex];
		const PxReal geometricError = rv.linear0XYZ_geometricErrorW[threadIndex].w;
		const PxReal velocityTarget = rv.angular0XYZ_velocityTargetW[threadIndex].w;

		maxBiasVelocity = Dy::computeMaxBiasVelocityTGS(flags, jointSpeedForRestitutionBounce, bounceVelocityThreshold, 
			restitution, geometricError, true, lengthScale, recipSimDt);

		// To use different mass for mass-splitting every sub-timestep (or iteration),
		// recipResponse, velMultipler, biasCoefficient, etc. are computed every sub-timestep (or iteration).
		// To compute them every sub-timestep (or iteration), additional 4 coefficients are stored; see queryReduced1dConstraintSolverConstantsTGS.
		// This does not change the previous impulse formulation, but a different mass is used due to mass-splitting.

		PxReal coeff0, coeff1, coeff2, coeff3;
		Dy::queryReduced1dConstraintSolverConstantsTGS(flags, stiffness, damping, restitution, bounceVelocityThreshold,
		                                               geometricError, velocityTarget, jointSpeedForRestitutionBounce,
		                                               initJointSpeed, erp, stepDt, recipStepDt, coeff0, coeff1, coeff2, coeff3);

		scon.lin0XYZ_initBiasOrCoeff0[threadIndex] = make_float4(_linear0.x, _linear0.y, _linear0.z, coeff0);
		scon.lin1XYZ_biasScaleOrCoeff1[threadIndex] = make_float4(_linear1.x, _linear1.y, _linear1.z, coeff1);
		scon.ang0XYZ_velMultiplierOrCoeff2[threadIndex] = make_float4(_angular0.x, _angular0.y, _angular0.z, coeff2);
		scon.ang1XYZ_velTargetOrCoeff3[threadIndex] = make_float4(_angular1.x, _angular1.y, _angular1.z, coeff3);
		scon.geometricError[threadIndex] = geometricError;
		scon.maxBias[threadIndex] = maxBiasVelocity;
		scon.appliedForce[threadIndex] = 0.f;
	}
}

//TGS 1D constraint
static __device__ PxU32 setUpArti1DConstraintBlock(
	const PxU32* PX_RESTRICT sortedRowIndices,
	PxgBlockConstraint1DData* PX_RESTRICT constraintData,
	PxgBlockConstraint1DVelocities* PX_RESTRICT rowVelocities,
	PxgBlockConstraint1DParameters* PX_RESTRICT rowParameters,
	PxgTGSBlockSolverConstraint1DCon* PX_RESTRICT constraintsCon,
	PxgArticulationBlockResponse* PX_RESTRICT articulationResponses,
	float stepDt, float simDt, 
	float recipStepDt, float recipSimDt,
	float lengthScale,
	const PxReal biasCoefficient,
	PxgSolverExtBody2& b0, PxgSolverExtBody2& b1,
	const physx::PxgSolverBodyPrepData* bodyData0,
	const physx::PxgSolverBodyPrepData* bodyData1,
	const PxU32 threadIndex)
{
	using namespace physx;

	const PxReal erp = 0.5f * biasCoefficient;
	
	PxU32 outCount = 0;

	PxgArticulationBlockResponse* PX_RESTRICT artiResponse = articulationResponses;

	const PxU32 numRows = constraintData->mNumRows[threadIndex];
	const float4 lin0X_ang0Y_lin1Z_ang1W = constraintData->mInvMassScale[threadIndex].lin0X_ang0Y_lin1Z_ang1W;

	const PxReal cfm = PxMax(b0.cfm, b1.cfm);

	for (PxU32 i = 0; i < numRows; i++)
	{
		PxgTGSBlockSolverConstraint1DCon& ccon = constraintsCon[outCount];
	
		const PxU32 index = sortedRowIndices[i];
		PxgBlockConstraint1DParameters& rp = rowParameters[index];
		PxgBlockConstraint1DVelocities& rv = rowVelocities[index];

		const PxU32 rpFlags = rp.flags[threadIndex];
		const PxU32 rpSolveHint = rp.solveHint[threadIndex];

		const float4 c_linear0XYZ_geometricErrorW = rv.linear0XYZ_geometricErrorW[threadIndex];
		const float4 c_linear1XYZ_minImpulseW = rv.linear1XYZ_minImpulseW[threadIndex];
		const float4 c_angular0XYZ_velocityTargetW = rv.angular0XYZ_velocityTargetW[threadIndex];
		const float4 c_angular1XYZ_maxImpulseW = rv.angular1XYZ_maxImpulseW[threadIndex];

		const PxVec3 clin0(c_linear0XYZ_geometricErrorW.x, c_linear0XYZ_geometricErrorW.y, c_linear0XYZ_geometricErrorW.z);
		const PxVec3 clin1(c_linear1XYZ_minImpulseW.x, c_linear1XYZ_minImpulseW.y, c_linear1XYZ_minImpulseW.z);
		const PxVec3 cang0(c_angular0XYZ_velocityTargetW.x, c_angular0XYZ_velocityTargetW.y, c_angular0XYZ_velocityTargetW.z);
		const PxVec3 cang1(c_angular1XYZ_maxImpulseW.x, c_angular1XYZ_maxImpulseW.y, c_angular1XYZ_maxImpulseW.z);

		const Cm::UnAlignedSpatialVector response0 = createImpulseResponseVector(b0, Cm::UnAlignedSpatialVector(cang0, clin0));
		const Cm::UnAlignedSpatialVector response1 = createImpulseResponseVector(b1, Cm::UnAlignedSpatialVector(-cang1, -clin1));

		const Cm::UnAlignedSpatialVector deltaV0 = getImpulseResponse(b0, Cm::UnAlignedSpatialVector(clin0, cang0).scale(lin0X_ang0Y_lin1Z_ang1W.x, lin0X_ang0Y_lin1Z_ang1W.y));
		const Cm::UnAlignedSpatialVector deltaV1 = getImpulseResponse(b1, Cm::UnAlignedSpatialVector(-clin1, -cang1).scale(lin0X_ang0Y_lin1Z_ang1W.z, lin0X_ang0Y_lin1Z_ang1W.w));

		const float resp0 = deltaV0.dot(response0);
		const float resp1 = deltaV1.dot(response1);

		float unitResponse = resp0 + resp1;//FAdd(resp0, resp1);

		if (unitResponse <= DY_ARTICULATION_MIN_RESPONSE)
			continue; //Degenerate constraint, can't be solved so skip to avoid computation later in the solver!

		ccon.resp0[threadIndex] = resp0;
		ccon.resp1[threadIndex] = resp1;

		artiResponse->deltaRALin_x[threadIndex] = deltaV0.bottom.x;
		artiResponse->deltaRALin_y[threadIndex] = deltaV0.bottom.y;
		artiResponse->deltaRALin_z[threadIndex] = deltaV0.bottom.z;
		artiResponse->deltaRAAng_x[threadIndex] = deltaV0.top.x;
		artiResponse->deltaRAAng_y[threadIndex] = deltaV0.top.y;
		artiResponse->deltaRAAng_z[threadIndex] = deltaV0.top.z;
		artiResponse->deltaRBLin_x[threadIndex] = deltaV1.bottom.x;
		artiResponse->deltaRBLin_y[threadIndex] = deltaV1.bottom.y;
		artiResponse->deltaRBLin_z[threadIndex] = deltaV1.bottom.z;
		artiResponse->deltaRBAng_x[threadIndex] = deltaV1.top.x;
		artiResponse->deltaRBAng_y[threadIndex] = deltaV1.top.y;
		artiResponse->deltaRBAng_z[threadIndex] = deltaV1.top.z;
		artiResponse++;

		PxReal jointSpeedForRestitutionBounce;
		PxReal initJointSpeed;
		{
			const PxReal vel0 = projectVelocity(b0, Cm::UnAlignedSpatialVector(cang0, clin0));
			const PxReal vel1 = projectVelocity(b1, Cm::UnAlignedSpatialVector(cang1, clin1));
			Dy::computeJointSpeedTGS(vel0, b0.isKinematic, vel1, b1.isKinematic, jointSpeedForRestitutionBounce, initJointSpeed);
		}

		//https://omniverse-jirasw.nvidia.com/browse/PX-4383
		const PxReal minRowResponse = DY_ARTICULATION_MIN_RESPONSE;
	
		intializeTGSBlock1D(
			rv, rp, 
			jointSpeedForRestitutionBounce, initJointSpeed, 
			resp0, resp1, erp, 
			stepDt, simDt, recipStepDt, recipSimDt, 
			lengthScale, minRowResponse, 
			response0.bottom, -response1.bottom, response0.top, -response1.top, 
			threadIndex, 
			ccon, cfm);

		ccon.angularErrorScale[threadIndex] = (rp.flags[threadIndex] & Px1DConstraintFlag::eANGULAR_CONSTRAINT) ? 1.f : 0.f;

		const bool hasDriveLimit = rpFlags & Px1DConstraintFlag::eHAS_DRIVE_LIMIT;
		const bool driveLimitsAreForces = constraintData->mFlags[threadIndex] & PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES;
		Dy::computeMinMaxImpulseOrForceAsImpulse(
			c_linear1XYZ_minImpulseW.w, c_angular1XYZ_maxImpulseW.w, 
			hasDriveLimit, driveLimitsAreForces, simDt, 
			ccon.minImpulse[threadIndex], ccon.maxImpulse[threadIndex]);

		PxU32 flags = 0;
		Dy::raiseInternalFlagsTGS(rpFlags, rpSolveHint, flags);

		ccon.flags[threadIndex] = flags;

		outCount++;
	}

	return outCount;
}

//PGS
template<int NbThreads>
static __device__ void setupArtiSolverConstraintBlockGPU(
	PxgBlockConstraint1DData* PX_RESTRICT constraintData,
	PxgBlockConstraint1DVelocities* PX_RESTRICT rowVelocities,
	PxgBlockConstraint1DParameters* PX_RESTRICT rowParameters,
	const PxgSolverBodyPrepData* bodyData0,
	const PxgSolverBodyPrepData* bodyData1,
	PxgSolverTxIData* txIData0,
	PxgSolverTxIData* txIData1,
	PxgBlockConstraintBatch& batch,
	const PxU32 threadIndexInWarp,
	PxgJointBlockParams& params,
	PxgArticulationBlockResponse* PX_RESTRICT artiResponses,
	const PxgSolverConstraintManagerConstants& managerConstants,
	PxgSolverExtBody2& b0,
	PxgSolverExtBody2& b1)
{
	using namespace physx;

	//distance constraint might have zero number of rows	
	params.jointHeader->rowCounts[threadIndexInWarp] = PxU8(constraintData->mNumRows[threadIndexInWarp]);

	params.jointHeader->writeBackOffset[threadIndexInWarp] = managerConstants.mConstraintWriteBackIndex;

	const float4 lin0_ang0_lin1_ang1 = constraintData->mInvMassScale[threadIndexInWarp].lin0X_ang0Y_lin1Z_ang1W;

	const float4 raWorld_linBreakForce = constraintData->mRAWorld_linBreakForce[threadIndexInWarp];
	const float4 rbWorld_angBreakForce = constraintData->mRBWorld_AngBreakForce[threadIndexInWarp];
	const float linBreakImpulse = raWorld_linBreakForce.w * params.dt;
	const float angBreakForce = rbWorld_angBreakForce.w;
	const float angBreakImpulse = angBreakForce * params.dt;
	params.jointHeader->body0WorldOffset_linBreakImpulse[threadIndexInWarp] = make_float4(raWorld_linBreakForce.x, raWorld_linBreakForce.y, raWorld_linBreakForce.z, linBreakImpulse);
	params.jointHeader->angBreakImpulse[threadIndexInWarp] = angBreakImpulse;

	params.jointHeader->invMass0D0[threadIndexInWarp] = -lin0_ang0_lin1_ang1.x;
	params.jointHeader->invMass1D1[threadIndexInWarp] = lin0_ang0_lin1_ang1.z;
	params.jointHeader->invInertiaScale0[threadIndexInWarp] = -lin0_ang0_lin1_ang1.y;
	params.jointHeader->invInertiaScale1[threadIndexInWarp] = lin0_ang0_lin1_ang1.w;

	params.jointHeader->breakable[threadIndexInWarp] = PxU8((raWorld_linBreakForce.w != PX_MAX_F32) || (angBreakForce != PX_MAX_F32));

	const PxReal cfm = PxMax(b0.cfm, b1.cfm);
	params.jointHeader->cfm[threadIndexInWarp] = cfm;

	__shared__ PxU32 sortedRowIndices[NbThreads][Dy::MAX_CONSTRAINT_ROWS];

	preprocessRowsBlock(sortedRowIndices[threadIdx.x + threadIdx.y * blockDim.x], constraintData, rowVelocities, rowParameters, threadIndexInWarp);

	params.jointHeader->rowCounts[threadIndexInWarp] = setUpArti1DConstraintBlock(sortedRowIndices[threadIdx.x + threadIdx.y * blockDim.x], constraintData,
		rowVelocities, rowParameters, params.jointCon, params.jointMod, artiResponses, params.dt, params.invDt, b0, b1, bodyData0, bodyData1, threadIndexInWarp);
}

//TGS
template<int NbThreads>
static __device__ void setupArtiSolverConstraintBlockGPU(
	PxgBlockConstraint1DData* PX_RESTRICT constraintData,
	PxgBlockConstraint1DVelocities* PX_RESTRICT rowVelocities,
	PxgBlockConstraint1DParameters* PX_RESTRICT rowParameters,
	const PxgSolverBodyPrepData* bodyData0,
	const PxgSolverBodyPrepData* bodyData1,
	PxgSolverTxIData* txIData0,
	PxgSolverTxIData* txIData1,
	PxgBlockConstraintBatch& batch,
	const PxU32 threadIndexInWarp,
	PxgTGSJointBlockParams& params,
	PxgArticulationBlockResponse* PX_RESTRICT artiResponses,
	const PxgSolverConstraintManagerConstants& managerConstants,
	PxgSolverExtBody2& b0,
	PxgSolverExtBody2& b1)
{
	using namespace physx;

	//distance constraint might have zero number of rows	
	//params.jointHeader->rowCounts_breakable_orthoAxisCount[threadIndexInWarp].x = PxU8(constraintData->mNumRows[threadIndexInWarp]);

	params.jointHeader->writeBackOffset[threadIndexInWarp] = managerConstants.mConstraintWriteBackIndex;

	const float4 lin0_ang0_lin1_ang1 = constraintData->mInvMassScale[threadIndexInWarp].lin0X_ang0Y_lin1Z_ang1W;
	 
	const float4 raWorld_linBreakForce = constraintData->mRAWorld_linBreakForce[threadIndexInWarp];
	const float4 rbWorld_angBreakForce = constraintData->mRBWorld_AngBreakForce[threadIndexInWarp];
	const float linBreakImpulse = raWorld_linBreakForce.w * params.dt;
	const float angBreakForce = rbWorld_angBreakForce.w;
	const float angBreakImpulse = angBreakForce * params.dt;
	params.jointHeader->rAWorld_invMass0D0[threadIndexInWarp] = make_float4(raWorld_linBreakForce.x, raWorld_linBreakForce.y, raWorld_linBreakForce.z, -lin0_ang0_lin1_ang1.x);
	params.jointHeader->rBWorld_invMass1D1[threadIndexInWarp] = make_float4(rbWorld_angBreakForce.x, rbWorld_angBreakForce.y, rbWorld_angBreakForce.z, lin0_ang0_lin1_ang1.z);

	//params.jointHeader->body0WorldOffset_linBreakImpulse[threadIndexInWarp] = make_float4(raWorld_linBreakForce.x, raWorld_linBreakForce.y, raWorld_linBreakForce.z, linBreakImpulse);
	params.jointHeader->angBreakImpulse[threadIndexInWarp] = angBreakImpulse;

	//params.jointHeader->invMass0D0[threadIndexInWarp] = -lin0_ang0_lin1_ang1.x;
	//params.jointHeader->invMass1D1[threadIndexInWarp] = lin0_ang0_lin1_ang1.z;
	params.jointHeader->invInertiaScale0[threadIndexInWarp] = -lin0_ang0_lin1_ang1.y;
	params.jointHeader->invInertiaScale1[threadIndexInWarp] = lin0_ang0_lin1_ang1.w;

	params.jointHeader->linBreakImpulse[threadIndexInWarp] = linBreakImpulse;

	PxU8 rowCounts = PxU8(constraintData->mNumRows[threadIndexInWarp]);
	const PxU8 breakable = PxU8((raWorld_linBreakForce.w != PX_MAX_F32) || (angBreakForce != PX_MAX_F32));
	
	__shared__ PxU32 sortedRowIndices[NbThreads][Dy::MAX_CONSTRAINT_ROWS];

	preprocessRowsBlock(sortedRowIndices[threadIdx.x + threadIdx.y * blockDim.x], constraintData, rowVelocities, rowParameters, threadIndexInWarp);

	rowCounts = setUpArti1DConstraintBlock(sortedRowIndices[threadIdx.x + threadIdx.y * blockDim.x], constraintData,
		rowVelocities, rowParameters, 
		params.jointCon, artiResponses, params.dt, params.totalDt, params.invDt, params.invTotalDt, 
		params.lengthScale, params.biasCoefficient, b0, b1, bodyData0, bodyData1, threadIndexInWarp);

	params.jointHeader->rowCounts_breakable_orthoAxisCount[threadIndexInWarp] = make_uchar4(rowCounts, breakable, 0, 0);

	const PxReal cfm = PxMax(b0.cfm, b1.cfm);
	params.jointHeader->cfm[threadIndexInWarp] = cfm;
}


//PGS
__device__ void fillJointBlockParams(PxgSolverSharedDesc<IterativeSolveData>* sharedDesc, 
	PxgConstraintPrepareDesc* constraintPrepDesc, PxgJointBlockParams& params,
	const PxU32 descIndexBatch, const PxU32 startConstraintIndex)
{
	IterativeSolveData& solveData = sharedDesc->iterativeData;
	params.jointHeader = &solveData.blockJointConstraintHeaders[descIndexBatch];
	params.jointCon = &solveData.blockJointConstraintRowsCon[startConstraintIndex];
	params.jointMod = &solveData.blockJointConstraintRowsMod[startConstraintIndex];
	params.dt = sharedDesc->dt;
	params.invDt = sharedDesc->invDtF32;
}

//TGS
__device__ void fillJointBlockParams(PxgSolverSharedDesc<IterativeSolveDataTGS>* sharedDesc,
	PxgConstraintPrepareDesc* constraintPrepDesc, PxgTGSJointBlockParams& params, const PxU32 descIndexBatch, const PxU32 startConstraintIndex)
{
	IterativeSolveDataTGS& solveData = sharedDesc->iterativeData;
	params.jointHeader = &solveData.blockJointConstraintHeaders[descIndexBatch];
	params.jointCon = &solveData.blockJointConstraintRowsCon[startConstraintIndex];
	params.dt = sharedDesc->stepDt;
	params.totalDt = sharedDesc->dt;
	params.invDt = sharedDesc->stepInvDtF32;
	params.invTotalDt = sharedDesc->invDtF32;
	params.lengthScale = sharedDesc->lengthScale;
	params.biasCoefficient = constraintPrepDesc->biasCoefficient;
}

template<typename JointParams, typename IterativeData>
__device__ void artiJointConstraintBlockPrepare(
	PxgConstraintPrepareDesc* constraintPrepDesc,
	PxgSolverSharedDesc<IterativeData>* sharedDesc)
{

	//This identifies which warp a specific thread is in, we treat all warps in all blocks as a flatten warp array
	//and we are going to index the work based on that
	const PxU32 warpIndex = blockIdx.x * blockDim.y + threadIdx.y;

	//This identifies which thread within a warp a specific thread is
	const PxU32 threadIndexInWarp = threadIdx.x;

	PxgSolverBodyData* solverBodyData = constraintPrepDesc->solverBodyDataPool;
	PxgSolverTxIData* solverTxIData = constraintPrepDesc->solverBodyTxIDataPool;

	PxAlignedTransform* bodyFrames = constraintPrepDesc->body2WorldPool;

	PxU32* batchIndices = constraintPrepDesc->artiJointConstraintBatchIndices;

	const PxU32 totalArti1dConstraintBatches = constraintPrepDesc->numArti1dConstraintBatches + constraintPrepDesc->numArtiStatic1dConstraintBatches
		+ constraintPrepDesc->numArtiSelf1dConstraintBatches;

	const PxU32 offset = constraintPrepDesc->numBatches;

	//for(PxU32 i=warpIndex; i< constraintPrepDesc->num1dConstraintBatches; i+=totalNumWarps)
	PxU32 i = warpIndex;
	if (i < totalArti1dConstraintBatches)
	{
		const PxU32 batchIndex = batchIndices[i] + offset;
		
		PxgBlockConstraintBatch& batch = sharedDesc->iterativeData.blockConstraintBatch[batchIndex];
		//const PxU32 bodyAIndex = batch.bodyAIndex[threadIndexInWarp];
		//const PxU32 bodyBIndex = batch.bodyBIndex[threadIndexInWarp];

		const PxU32 descIndexBatch = batch.mConstraintBatchIndex;

		const PxU32 descStride = batch.mDescStride;

#if LOAD_BODY_DATA
		loadBodyData(solverBodyDatas, descStride, bodyAIndex, threadIndexInWarp, warpIndexInBlock, bodyData0.initialLinVelXYZ_invMassW, bodyData0.initialAngVelXYZ_penBiasClamp,
			bodyData0.sqrtInvInertia, bodyData0.body2World);
		loadBodyData(solverBodyDatas, descStride, bodyBIndex, threadIndexInWarp, warpIndexInBlock, bodyData1.initialLinVelXYZ_invMassW, bodyData1.initialAngVelXYZ_penBiasClamp,
			bodyData1.sqrtInvInertia, bodyData1.body2World);
#endif

		//mDescStride might less than 32, we need to guard against it
		if (threadIndexInWarp < descStride)
		{
			//desc.descIndex for joint in fact is the batch index
			PxgBlockConstraint1DData& constraintData = constraintPrepDesc->blockJointPrepPool[descIndexBatch];
			PxgBlockConstraint1DVelocities* rowVelocities = &constraintPrepDesc->blockJointPrepPool0[descIndexBatch * Dy::MAX_CONSTRAINT_ROWS];
			PxgBlockConstraint1DParameters* rowParameters = &constraintPrepDesc->blockJointPrepPool1[descIndexBatch * Dy::MAX_CONSTRAINT_ROWS];

			const PxNodeIndex nodeIndexA = batch.bodyANodeIndex[threadIndexInWarp];
			const PxNodeIndex nodeIndexB = batch.bodyBNodeIndex[threadIndexInWarp];

			const PxU32 bodyAIndex = nodeIndexA.isArticulation() ? batch.remappedBodyAIndex[threadIndexInWarp] : batch.bodyAIndex[threadIndexInWarp];
			const PxU32 bodyBIndex = nodeIndexB.isArticulation() ? batch.remappedBodyBIndex[threadIndexInWarp] : batch.bodyBIndex[threadIndexInWarp];

			PxgSolverBodyData* bodyData0 = &solverBodyData[bodyAIndex];
			PxgSolverBodyData* bodyData1 = &solverBodyData[bodyBIndex];
			PxgSolverTxIData* txIData0 = &solverTxIData[bodyAIndex];
			PxgSolverTxIData* txIData1 = &solverTxIData[bodyBIndex];
			
			PxgSolverExtBody2 b0, b1;
			createPxgSolverExtBody(nodeIndexA, bodyAIndex, sharedDesc->articulations, solverBodyData, solverTxIData, b0, bodyFrames);
			createPxgSolverExtBody(nodeIndexB, bodyBIndex, sharedDesc->articulations, solverBodyData, solverTxIData, b1, bodyFrames);

			/*PxU32 isArticulationA = b0.islandNodeIndex.isArticulation();
			PxU32 isArticulationB = b1.islandNodeIndex.isArticulation();*/

		/*	printf("isArticulationA %i isArticulationB %i, linkID0 %i, linkID1 %i\n", isArticulationA, isArticulationB,
				nodeIndexA.articulationLinkId(), nodeIndexB.articulationLinkId());*/

			JointParams params;
			fillJointBlockParams(sharedDesc, constraintPrepDesc, params, descIndexBatch, batch.startConstraintIndex);

			PxU32 uniqueIndex = constraintPrepDesc->artiConstraintUniqueIndices[batch.mStartPartitionIndex + threadIndexInWarp];
			
			PxgArticulationBlockResponse* responses = sharedDesc->iterativeData.artiResponse;
			const PxU32 responseIndex = batch.mArticulationResponseIndex;

			////printf("constraintPre2 startConstraintIndex %i\n", batch.startConstraintIndex);
			//setupArtiSolverConstraintBlockGPU<PxgKernelBlockDim::CONSTRAINT_PREPARE_BLOCK_PARALLEL>(&constraintData, rowVelocities, rowParameters, bodyData0, bodyData1, txIData0, txIData1, sharedDesc->dt, sharedDesc->invDtF32, batch, threadIndexInWarp,
			//	&jointConstraintHeaders[descIndexBatch], &jointConstraintRowsCon[batch.startConstraintIndex], &jointConstraintRowsMod[batch.startConstraintIndex], &responses[responseIndex],
			//	constraintPrepDesc->solverConstantData[uniqueIndex], b0, b1);

			//Attention: PxgKernelBlockDim::ARTI_CONSTRAINT_PREPARE is not launched as a linear block but as a 2D block. thradIdx.x and threadIdx.y are used for indexing inside the block.
			setupArtiSolverConstraintBlockGPU<PxgKernelBlockDim::ARTI_CONSTRAINT_PREPARE>(&constraintData, rowVelocities, rowParameters, bodyData0, bodyData1, txIData0, txIData1,
				batch, threadIndexInWarp, params, &responses[responseIndex],
				constraintPrepDesc->solverConstantData[uniqueIndex], b0, b1);
		}
	}
}


extern "C" __global__ void artiJointConstraintBlockPrepareParallelLaunch(PxgConstraintPrepareDesc* prepDesc, 
	PxgSolverSharedDesc<IterativeSolveData>* sharedDesc)
{
	artiJointConstraintBlockPrepare<PxgJointBlockParams, IterativeSolveData>(prepDesc, sharedDesc);
}

extern "C" __global__ void artiTGSJointConstraintBlockPrepareParallelLaunch(PxgConstraintPrepareDesc* prepDesc, 
	PxgSolverSharedDesc<IterativeSolveDataTGS>* sharedDesc)
{
	artiJointConstraintBlockPrepare<PxgTGSJointBlockParams, IterativeSolveDataTGS>(prepDesc, sharedDesc);
}
