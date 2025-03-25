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

#ifndef __SOLVER_CUH__
#define __SOLVER_CUH__

#include "PxgSolverBody.h"
#include "PxgSolverConstraint1D.h"
#include "PxgSolverConstraintBlock1D.h"
#include "PxgSolverConstraintDesc.h"
#include "PxgConstraint.h"
#include "PxgConstraintBlock.h"
#include "PxgSolverContext.h"
#include "PxgSolverCoreDesc.h"
#include "PxgCommonDefines.h"
#include "PxgIntrinsics.h"
#include "PxgArticulation.h"
#include "solverResidual.cuh"
#include "constraintPrepShared.cuh"

#include <stdio.h>
#include <assert.h>

using namespace physx;

// This function is for contacts involving articulations.
// To apply mass-splitting, different data is stored and used when computing impulses.
// Apart from mass-splitting, the formulation is the same as the previous implementation, see "setupFinalizeExtSolverConstraintsBlock"
static __device__ void solveExtContactsBlock(const PxgBlockConstraintBatch& batch, Cm::UnAlignedSpatialVector& vel0,
	Cm::UnAlignedSpatialVector& vel1, const bool doFriction, PxgBlockSolverContactHeader* contactHeaders,
	PxgBlockSolverFrictionHeader* frictionHeaders, PxgBlockSolverContactPoint* contactPoints,
	PxgBlockSolverContactFriction* frictionPoints, const PxgArticulationBlockResponse* const PX_RESTRICT responses, Cm::UnAlignedSpatialVector& impulse0,
	Cm::UnAlignedSpatialVector& impulse1, const PxU32 threadIndexInWarp, PxgErrorAccumulator* error, 
	PxReal ref0 = 1.f, PxReal ref1 = 1.f)
{
	Cm::UnAlignedSpatialVector imp0(PxVec3(0.f), PxVec3(0.f));
	Cm::UnAlignedSpatialVector imp1(PxVec3(0.f), PxVec3(0.f));

	PxgBlockSolverContactHeader& contactHeader = contactHeaders[batch.mConstraintBatchIndex];
	PxgBlockSolverFrictionHeader& frictionHeader = frictionHeaders[batch.mConstraintBatchIndex];

	const float4 invMass0_1_angDom0_1 = Pxldcg(contactHeader.invMass0_1_angDom0_1[threadIndexInWarp]);

	const PxgArticulationBlockResponse* resp = &responses[batch.mArticulationResponseIndex];

	const uint numNormalConstr = Pxldcg(contactHeader.numNormalConstr[threadIndexInWarp]);
	const uint	numFrictionConstr = Pxldcg(frictionHeader.numFrictionConstr[threadIndexInWarp]);

	const float restitution = contactHeader.restitution[threadIndexInWarp];
	const float p8 = 0.8f;
	const float cfm = contactHeader.cfm[threadIndexInWarp];
	const PxU8 flags = contactHeader.flags[threadIndexInWarp];

	PxgBlockSolverContactPoint* contacts = &contactPoints[batch.startConstraintIndex];

	PxReal accumulatedNormalImpulse = 0.f;

	Cm::UnAlignedSpatialVector v0 = vel0;
	Cm::UnAlignedSpatialVector v1 = vel1;

	const float4 normal_staticFriction = Pxldcg(contactHeader.normal_staticFriction[threadIndexInWarp]);

	const PxVec3 normal = PxVec3(normal_staticFriction.x, normal_staticFriction.y, normal_staticFriction.z);
	const PxReal staticCof = normal_staticFriction.w;

	float4 nextRaxn_extraCoeff = Pxldcg(contacts[0].raXn_targetVelocity[threadIndexInWarp]);
	float4 nextRbxn_maxImpulseW = Pxldcg(contacts[0].rbXn_maxImpulse[threadIndexInWarp]);
	float nextAppliedForce = Pxldcg(contacts[0].appliedForce[threadIndexInWarp]);

	float nextResp0 = Pxldcg(contacts[0].resp0[threadIndexInWarp]);
	float nextResp1 = Pxldcg(contacts[0].resp1[threadIndexInWarp]);

	float nextCoeff0 = Pxldcg(contacts[0].coeff0[threadIndexInWarp]);
	float nextCoeff1 = Pxldcg(contacts[0].coeff1[threadIndexInWarp]);

	float3 nextDeltaRALin = make_float3(Pxldcg(resp->deltaRALin_x[threadIndexInWarp]), Pxldcg(resp->deltaRALin_y[threadIndexInWarp]), Pxldcg(resp->deltaRALin_z[threadIndexInWarp]));
	float3 nextDeltaRAAng = make_float3(Pxldcg(resp->deltaRAAng_x[threadIndexInWarp]), Pxldcg(resp->deltaRAAng_y[threadIndexInWarp]), Pxldcg(resp->deltaRAAng_z[threadIndexInWarp]));
	float3 nextDeltaRBLin = make_float3(Pxldcg(resp->deltaRBLin_x[threadIndexInWarp]), Pxldcg(resp->deltaRBLin_y[threadIndexInWarp]), Pxldcg(resp->deltaRBLin_z[threadIndexInWarp]));
	float3 nextDeltaRBAng = make_float3(Pxldcg(resp->deltaRBAng_x[threadIndexInWarp]), Pxldcg(resp->deltaRBAng_y[threadIndexInWarp]), Pxldcg(resp->deltaRBAng_z[threadIndexInWarp]));

	PxgBlockSolverContactFriction* frictions = &frictionPoints[batch.startFrictionIndex];

	for (uint i = 0; i < numNormalConstr; i++)
	{
		PxgBlockSolverContactPoint& c = contacts[i];
		resp++;

		const float4 raXn_targetVelocity = nextRaxn_extraCoeff;
		const float4 rbXn_maxImpulse = nextRbxn_maxImpulseW;
		const float appliedForce = nextAppliedForce;
		const float resp0 = nextResp0;
		const float resp1 = nextResp1;
		const float coeff0 = nextCoeff0;
		const float coeff1 = nextCoeff1;

		const float3 deltaRALin = nextDeltaRALin;
		const float3 deltaRAAng = nextDeltaRAAng;
		const float3 deltaRBLin = nextDeltaRBLin;
		const float3 deltaRBAng = nextDeltaRBAng;

		if ((i + 1) < numNormalConstr)
		{
			const PxgBlockSolverContactPoint& nextC = contacts[i + 1];

			nextRaxn_extraCoeff = Pxldcg(nextC.raXn_targetVelocity[threadIndexInWarp]);
			nextRbxn_maxImpulseW = Pxldcg(nextC.rbXn_maxImpulse[threadIndexInWarp]);
			nextAppliedForce = Pxldcg(nextC.appliedForce[threadIndexInWarp]);

			nextResp0 = Pxldcg(nextC.resp0[threadIndexInWarp]);
			nextResp1 = Pxldcg(nextC.resp1[threadIndexInWarp]);

			nextCoeff0 = Pxldcg(nextC.coeff0[threadIndexInWarp]);
			nextCoeff1 = Pxldcg(nextC.coeff1[threadIndexInWarp]);

			nextDeltaRALin = make_float3(Pxldcg(resp->deltaRALin_x[threadIndexInWarp]), Pxldcg(resp->deltaRALin_y[threadIndexInWarp]), Pxldcg(resp->deltaRALin_z[threadIndexInWarp]));
			nextDeltaRAAng = make_float3(Pxldcg(resp->deltaRAAng_x[threadIndexInWarp]), Pxldcg(resp->deltaRAAng_y[threadIndexInWarp]), Pxldcg(resp->deltaRAAng_z[threadIndexInWarp]));
			nextDeltaRBLin = make_float3(Pxldcg(resp->deltaRBLin_x[threadIndexInWarp]), Pxldcg(resp->deltaRBLin_y[threadIndexInWarp]), Pxldcg(resp->deltaRBLin_z[threadIndexInWarp]));
			nextDeltaRBAng = make_float3(Pxldcg(resp->deltaRBAng_x[threadIndexInWarp]), Pxldcg(resp->deltaRBAng_y[threadIndexInWarp]), Pxldcg(resp->deltaRBAng_z[threadIndexInWarp]));
		}
		else if (numFrictionConstr && doFriction)
		{
			nextRaxn_extraCoeff = Pxldcg(frictions[0].raXn_bias[threadIndexInWarp]);
			nextRbxn_maxImpulseW = Pxldcg(frictions[0].rbXn_targetVelW[threadIndexInWarp]);
			nextAppliedForce = Pxldcg(frictions[0].appliedForce[threadIndexInWarp]);
			nextResp0 = Pxldcg(frictions[0].resp0[threadIndexInWarp]);
			nextResp1 = Pxldcg(frictions[0].resp1[threadIndexInWarp]);

			nextDeltaRALin = make_float3(Pxldcg(resp->deltaRALin_x[threadIndexInWarp]), Pxldcg(resp->deltaRALin_y[threadIndexInWarp]), Pxldcg(resp->deltaRALin_z[threadIndexInWarp]));
			nextDeltaRAAng = make_float3(Pxldcg(resp->deltaRAAng_x[threadIndexInWarp]), Pxldcg(resp->deltaRAAng_y[threadIndexInWarp]), Pxldcg(resp->deltaRAAng_z[threadIndexInWarp]));
			nextDeltaRBLin = make_float3(Pxldcg(resp->deltaRBLin_x[threadIndexInWarp]), Pxldcg(resp->deltaRBLin_y[threadIndexInWarp]), Pxldcg(resp->deltaRBLin_z[threadIndexInWarp]));
			nextDeltaRBAng = make_float3(Pxldcg(resp->deltaRBAng_x[threadIndexInWarp]), Pxldcg(resp->deltaRBAng_y[threadIndexInWarp]), Pxldcg(resp->deltaRBAng_z[threadIndexInWarp]));
		}

		const PxVec3 raXn = PxVec3(raXn_targetVelocity.x, raXn_targetVelocity.y, raXn_targetVelocity.z);
		const PxVec3 rbXn = PxVec3(rbXn_maxImpulse.x, rbXn_maxImpulse.y, rbXn_maxImpulse.z);
		const float targetVelocity = raXn_targetVelocity.w;
		const float maxImpulse = rbXn_maxImpulse.w;

		float unitResponse = ref0 * resp0 + ref1 * resp1;
		float recipResponse = (unitResponse > 0.0f) ? 1.0f / (unitResponse + cfm) : 0.0f;
		float velMultiplier = recipResponse;
		float impulseMul = 1.0f;
		float unbiasedError;
		float biasedErr;

		computeContactCoefficients(flags, restitution, unitResponse, recipResponse, targetVelocity, coeff0, coeff1,
			velMultiplier, impulseMul, unbiasedError, biasedErr);

		const Cm::UnAlignedSpatialVector deltaVA(ref0 * PxVec3(deltaRAAng.x, deltaRAAng.y, deltaRAAng.z),
												 ref0 * PxVec3(deltaRALin.x, deltaRALin.y, deltaRALin.z));

		const Cm::UnAlignedSpatialVector deltaVB(ref1 * PxVec3(deltaRBAng.x, deltaRBAng.y, deltaRBAng.z),
												 ref1 * PxVec3(deltaRBLin.x, deltaRBLin.y, deltaRBLin.z));

		const float v0_ = v0.bottom.dot(normal) + v0.top.dot(raXn);//V3MulAdd(linVel0, normal, V3Mul(angVel0, raXn));
		const float v1_ = v1.bottom.dot(normal) + v1.top.dot(rbXn);//V3MulAdd(linVel1, normal, V3Mul(angVel1, rbXn));
		const float normalVel = v0_ - v1_;

		//KS - clamp the maximum force
		const float tempDeltaF = biasedErr - normalVel * velMultiplier;
		const float _deltaF = fmaxf(tempDeltaF, -appliedForce);//FMax(FNegScaleSub(normalVel, velMultiplier, biasedErr), FNeg(appliedForce));
		const float _newForce = appliedForce * impulseMul + _deltaF;
		const float newForce = fminf(_newForce, maxImpulse);//FMin(_newForce, maxImpulse);
		const float deltaF = newForce - appliedForce;

		if(error)
			error->accumulateErrorLocal(deltaF, velMultiplier);
		Pxstcg(&c.appliedForce[threadIndexInWarp], newForce);

		imp0.bottom -= raXn * deltaF;
		imp0.top -= normal * deltaF;
		imp1.bottom += rbXn * deltaF;
		imp1.top += normal * deltaF;

		v0 += deltaVA * deltaF;
		v1 += deltaVB * deltaF;

		accumulatedNormalImpulse = accumulatedNormalImpulse + newForce;
	}

	//Force a minimum normal force for friction. This is required for articulations with multi-link collisions
	//because often normal force can be solved with just 1 link's collisions. However, this means that other links can slide on
	//a surface friction-free because there was no normal force applied.
	accumulatedNormalImpulse = PxMax(accumulatedNormalImpulse, contactHeader.minNormalForce[threadIndexInWarp]);

	if (numFrictionConstr && doFriction)
	{

		//printf("FrictionHeader = %i, count = %i\n", batch.startFrictionIndex, numFrictionConstr);
		const float dynamicFrictionCof = frictionHeader.dynamicFriction[threadIndexInWarp];
		const float maxFrictionImpulse = staticCof * accumulatedNormalImpulse;
		const float maxDynFrictionImpulse = dynamicFrictionCof * accumulatedNormalImpulse;
		//const float negMaxDynFrictionImpulse = -maxDynFrictionImpulse;

		PxU32 broken = 0;


		for (uint i = 0; i < numFrictionConstr; i++)
		{
			PxgBlockSolverContactFriction& f = frictions[i];
			resp++;

			const float4 frictionNormal = frictionHeader.frictionNormals[i & 1][threadIndexInWarp];

			const float4 raXn_extraCoeff = nextRaxn_extraCoeff;
			const float4 rbXn_targetVelW = nextRbxn_maxImpulseW;
			const float resp0 = nextResp0;
			const float resp1 = nextResp1;

			const float appliedForce = nextAppliedForce;


			const float3 deltaRALin = nextDeltaRALin;
			const float3 deltaRAAng = nextDeltaRAAng;
			const float3 deltaRBLin = nextDeltaRBLin;
			const float3 deltaRBAng = nextDeltaRBAng;

			if ((i + 1) < numFrictionConstr)
			{
				const PxgBlockSolverContactFriction& f2 = frictions[i + 1];

				nextRaxn_extraCoeff = Pxldcg(f2.raXn_bias[threadIndexInWarp]);
				nextRbxn_maxImpulseW = Pxldcg(f2.rbXn_targetVelW[threadIndexInWarp]);
				nextResp0 = Pxldcg(f2.resp0[threadIndexInWarp]);
				nextResp1 = Pxldcg(f2.resp1[threadIndexInWarp]);

				nextAppliedForce = Pxldcg(f2.appliedForce[threadIndexInWarp]);
				nextDeltaRALin = make_float3(Pxldcg(resp->deltaRALin_x[threadIndexInWarp]), Pxldcg(resp->deltaRALin_y[threadIndexInWarp]), Pxldcg(resp->deltaRALin_z[threadIndexInWarp]));
				nextDeltaRAAng = make_float3(Pxldcg(resp->deltaRAAng_x[threadIndexInWarp]), Pxldcg(resp->deltaRAAng_y[threadIndexInWarp]), Pxldcg(resp->deltaRAAng_z[threadIndexInWarp]));
				nextDeltaRBLin = make_float3(Pxldcg(resp->deltaRBLin_x[threadIndexInWarp]), Pxldcg(resp->deltaRBLin_y[threadIndexInWarp]), Pxldcg(resp->deltaRBLin_z[threadIndexInWarp]));
				nextDeltaRBAng = make_float3(Pxldcg(resp->deltaRBAng_x[threadIndexInWarp]), Pxldcg(resp->deltaRBAng_y[threadIndexInWarp]), Pxldcg(resp->deltaRBAng_z[threadIndexInWarp]));
			}

			const PxVec3 raXn = PxVec3(raXn_extraCoeff.x, raXn_extraCoeff.y, raXn_extraCoeff.z);
			const PxVec3 rbXn = PxVec3(rbXn_targetVelW.x, rbXn_targetVelW.y, rbXn_targetVelW.z);

			const float resp = ref0 * resp0 + ref1 * resp1;
			const float velMultiplier = (resp > PX_EPS_REAL) ? (p8 / resp) : 0.f;
			const float bias = raXn_extraCoeff.w;
			const float targetVel = rbXn_targetVelW.w;

			const Cm::UnAlignedSpatialVector deltaVA(ref0 * PxVec3(deltaRAAng.x, deltaRAAng.y, deltaRAAng.z),
													 ref0 * PxVec3(deltaRALin.x, deltaRALin.y, deltaRALin.z));

			const Cm::UnAlignedSpatialVector deltaVB(ref1 * PxVec3(deltaRBAng.x, deltaRBAng.y, deltaRBAng.z),
													 ref1 * PxVec3(deltaRBLin.x, deltaRBLin.y, deltaRBLin.z));

			const PxVec3 normal = PxVec3(frictionNormal.x, frictionNormal.y, frictionNormal.z);

			const float v0_ = v0.top.dot(raXn) + v0.bottom.dot(normal);//V3MulAdd(linVel0, normal, V3Mul(angVel0, raXn));
			const float v1_ = v1.top.dot(rbXn) + v1.bottom.dot(normal);//V3MulAdd(linVel1, normal, V3Mul(angVel1, rbXn));
			const float normalVel = v0_ - v1_;

			const float tmp1 = appliedForce - (bias - targetVel) * velMultiplier;

			const float totalImpulse = tmp1 - normalVel * velMultiplier;

			const bool clamp = fabsf(totalImpulse) > maxFrictionImpulse;

			const float totalClamped = fminf(maxDynFrictionImpulse, fmaxf(-maxDynFrictionImpulse, totalImpulse));

			const float newAppliedForce = clamp ? totalClamped : totalImpulse;

			float deltaF = newAppliedForce - appliedForce;//FSub(newAppliedForce, appliedForce);

			if (error)
				error->accumulateErrorLocal(deltaF, velMultiplier);

			//printf("v0 = (%f, %f, %f, %f, %f, %f), v1 = (%f, %f, %f, %f, %f, %f)\n", v0.top.x, v0.top.y, v0.top.z, v0.bottom.x, v0.bottom.y, v0.bottom.z,
			//	v1.top.x, v1.top.y, v1.top.z, v1.bottom.x, v1.bottom.y, v1.bottom.z);
			//printf("normal = (%f, %f, %f), raXn = (%f, %f, %f)\n", normal.x, normal.y, normal.z, raXn.x, raXn.y, raXn.z);

			//printf("Friction velMultiplier = %f, normalVel = %f, deltaF = %f\n", velMultiplier, normalVel, deltaF);

			v0 += deltaVA * deltaF;
			v1 += deltaVB * deltaF;

			imp0.bottom -= raXn * deltaF;
			imp0.top -= normal * deltaF;
			imp1.bottom += rbXn * deltaF;
			imp1.top += normal * deltaF;

			//f.appliedForce[threadIndex] = newAppliedForce;
			Pxstcg(&f.appliedForce[threadIndexInWarp], newAppliedForce);
			broken = broken | clamp;
		}
		Pxstcg(&frictionHeader.broken[threadIndexInWarp], broken);
	}

	impulse0 = imp0.scale(ref0 * invMass0_1_angDom0_1.z, ref0 * invMass0_1_angDom0_1.x);
	impulse1 = imp1.scale(ref1 * invMass0_1_angDom0_1.y, ref1 * invMass0_1_angDom0_1.w);

	vel0 = v0;
	vel1 = v1;
}

// A light version of the function "solveExtContactsBlock" to quickly check if there is any active contact.
static __device__ bool checkExtActiveContactBlock(const PxgBlockConstraintBatch& batch, const Cm::UnAlignedSpatialVector& vel0,
	const Cm::UnAlignedSpatialVector& vel1, PxgBlockSolverContactHeader* contactHeaders,
	PxgBlockSolverContactPoint* contactPoints, const PxgArticulationBlockResponse* const PX_RESTRICT responses, 
	const PxU32 threadIndexInWarp)
{
	PxgBlockSolverContactHeader& contactHeader = contactHeaders[batch.mConstraintBatchIndex];
	const float4 invMass0_1_angDom0_1 = Pxldcg(contactHeader.invMass0_1_angDom0_1[threadIndexInWarp]);
	const PxgArticulationBlockResponse* resp = &responses[batch.mArticulationResponseIndex];
	const uint numNormalConstr = Pxldcg(contactHeader.numNormalConstr[threadIndexInWarp]);

	const float restitution = contactHeader.restitution[threadIndexInWarp];
	const float cfm = contactHeader.cfm[threadIndexInWarp];
	const PxU8 flags = contactHeader.flags[threadIndexInWarp];

	PxgBlockSolverContactPoint* contacts = &contactPoints[batch.startConstraintIndex];

	Cm::UnAlignedSpatialVector v0 = vel0;
	Cm::UnAlignedSpatialVector v1 = vel1;

	const float4 normal_staticFriction = Pxldcg(contactHeader.normal_staticFriction[threadIndexInWarp]);
	const PxVec3 normal = PxVec3(normal_staticFriction.x, normal_staticFriction.y, normal_staticFriction.z);

	float4 nextRaxn_extraCoeff = Pxldcg(contacts[0].raXn_targetVelocity[threadIndexInWarp]);
	float4 nextRbxn_maxImpulseW = Pxldcg(contacts[0].rbXn_maxImpulse[threadIndexInWarp]);
	float nextAppliedForce = Pxldcg(contacts[0].appliedForce[threadIndexInWarp]);

	float nextResp0 = Pxldcg(contacts[0].resp0[threadIndexInWarp]);
	float nextResp1 = Pxldcg(contacts[0].resp1[threadIndexInWarp]);

	float nextCoeff0 = Pxldcg(contacts[0].coeff0[threadIndexInWarp]);
	float nextCoeff1 = Pxldcg(contacts[0].coeff1[threadIndexInWarp]);

	float3 nextDeltaRALin = make_float3(Pxldcg(resp->deltaRALin_x[threadIndexInWarp]), Pxldcg(resp->deltaRALin_y[threadIndexInWarp]), Pxldcg(resp->deltaRALin_z[threadIndexInWarp]));
	float3 nextDeltaRAAng = make_float3(Pxldcg(resp->deltaRAAng_x[threadIndexInWarp]), Pxldcg(resp->deltaRAAng_y[threadIndexInWarp]), Pxldcg(resp->deltaRAAng_z[threadIndexInWarp]));
	float3 nextDeltaRBLin = make_float3(Pxldcg(resp->deltaRBLin_x[threadIndexInWarp]), Pxldcg(resp->deltaRBLin_y[threadIndexInWarp]), Pxldcg(resp->deltaRBLin_z[threadIndexInWarp]));
	float3 nextDeltaRBAng = make_float3(Pxldcg(resp->deltaRBAng_x[threadIndexInWarp]), Pxldcg(resp->deltaRBAng_y[threadIndexInWarp]), Pxldcg(resp->deltaRBAng_z[threadIndexInWarp]));

	for (uint i = 0; i < numNormalConstr; i++)
	{
		resp++;

		const float4 raXn_targetVelocity = nextRaxn_extraCoeff;
		const float4 rbXn_maxImpulse = nextRbxn_maxImpulseW;
		const float appliedForce = nextAppliedForce;
		const float resp0 = nextResp0;
		const float resp1 = nextResp1;
		const float coeff0 = nextCoeff0;
		const float coeff1 = nextCoeff1;

		if ((i + 1) < numNormalConstr)
		{
			const PxgBlockSolverContactPoint& nextC = contacts[i + 1];

			nextRaxn_extraCoeff = Pxldcg(nextC.raXn_targetVelocity[threadIndexInWarp]);
			nextRbxn_maxImpulseW = Pxldcg(nextC.rbXn_maxImpulse[threadIndexInWarp]);
			nextAppliedForce = Pxldcg(nextC.appliedForce[threadIndexInWarp]);

			nextResp0 = Pxldcg(nextC.resp0[threadIndexInWarp]);
			nextResp1 = Pxldcg(nextC.resp1[threadIndexInWarp]);

			nextCoeff0 = Pxldcg(nextC.coeff0[threadIndexInWarp]);
			nextCoeff1 = Pxldcg(nextC.coeff1[threadIndexInWarp]);

			nextDeltaRALin = make_float3(Pxldcg(resp->deltaRALin_x[threadIndexInWarp]), Pxldcg(resp->deltaRALin_y[threadIndexInWarp]), Pxldcg(resp->deltaRALin_z[threadIndexInWarp]));
			nextDeltaRAAng = make_float3(Pxldcg(resp->deltaRAAng_x[threadIndexInWarp]), Pxldcg(resp->deltaRAAng_y[threadIndexInWarp]), Pxldcg(resp->deltaRAAng_z[threadIndexInWarp]));
			nextDeltaRBLin = make_float3(Pxldcg(resp->deltaRBLin_x[threadIndexInWarp]), Pxldcg(resp->deltaRBLin_y[threadIndexInWarp]), Pxldcg(resp->deltaRBLin_z[threadIndexInWarp]));
			nextDeltaRBAng = make_float3(Pxldcg(resp->deltaRBAng_x[threadIndexInWarp]), Pxldcg(resp->deltaRBAng_y[threadIndexInWarp]), Pxldcg(resp->deltaRBAng_z[threadIndexInWarp]));
		}

		const PxVec3 raXn = PxVec3(raXn_targetVelocity.x, raXn_targetVelocity.y, raXn_targetVelocity.z);
		const PxVec3 rbXn = PxVec3(rbXn_maxImpulse.x, rbXn_maxImpulse.y, rbXn_maxImpulse.z);
		const float targetVelocity = raXn_targetVelocity.w;
		const float maxImpulse = rbXn_maxImpulse.w;

		float unitResponse = resp0 + resp1;
		float recipResponse = (unitResponse > 0.f) ? 1.f / (unitResponse + cfm) : 0.f;
		float velMultiplier = recipResponse;
		float impulseMul = 1.f;
		float unbiasedError;
		float biasedErr;

		computeContactCoefficients(flags, restitution, unitResponse, recipResponse, targetVelocity, coeff0, coeff1,
			velMultiplier, impulseMul, unbiasedError, biasedErr);

		const float v0_ = v0.bottom.dot(normal) + v0.top.dot(raXn);//V3MulAdd(linVel0, normal, V3Mul(angVel0, raXn));
		const float v1_ = v1.bottom.dot(normal) + v1.top.dot(rbXn);//V3MulAdd(linVel1, normal, V3Mul(angVel1, rbXn));
		const float normalVel = v0_ - v1_;

		//KS - clamp the maximum force
		const float tempDeltaF = biasedErr - normalVel * velMultiplier;
		const float _deltaF = fmaxf(tempDeltaF, -appliedForce);//FMax(FNegScaleSub(normalVel, velMultiplier, biasedErr), FNeg(appliedForce));
		const float _newForce = appliedForce * impulseMul + _deltaF;
		const float newForce = fminf(_newForce, maxImpulse);//FMin(_newForce, maxImpulse);
		const float deltaF = newForce - appliedForce;

		// Check for active contact.
		if (PxAbs(deltaF) > 1.0e-8f)
		{
			return true;
		}
	}

	return false;
}

// To apply mass-splitting, different data is stored and used when computing impulses.
// Apart from mass-splitting, the formulation is the same as the previous implementation, see "setupFinalizeExtSolverConstraintsBlock"
static __device__ PX_FORCE_INLINE void solveExtContactBlockTGS(const PxgBlockConstraintBatch& batch, Cm::UnAlignedSpatialVector& vel0, Cm::UnAlignedSpatialVector& vel1, const Cm::UnAlignedSpatialVector& delta0, const Cm::UnAlignedSpatialVector& delta1,
	const PxU32 threadIndex, PxgTGSBlockSolverContactHeader* PX_RESTRICT contactHeaders, PxgTGSBlockSolverFrictionHeader* PX_RESTRICT frictionHeaders, PxgTGSBlockSolverContactPoint* PX_RESTRICT contactPoints, PxgTGSBlockSolverContactFriction* PX_RESTRICT frictionPoints,
	PxgArticulationBlockResponse* PX_RESTRICT responses, const PxReal elapsedTime, const PxReal minPen,
	Cm::UnAlignedSpatialVector& impulse0, Cm::UnAlignedSpatialVector& impulse1, PxgErrorAccumulator* error,
	PxReal ref0 = 1.f, PxReal ref1 = 1.f)
{
	PxVec3 linVel0 = vel0.bottom;
	PxVec3 linVel1 = vel1.bottom;
	PxVec3 angVel0 = vel0.top;
	PxVec3 angVel1 = vel1.top;

	float accumulatedNormalImpulse = 0.f;

	Cm::UnAlignedSpatialVector imp0(PxVec3(0.f), PxVec3(0.f)), imp1(PxVec3(0.f), PxVec3(0.f));
	{
		PxgTGSBlockSolverContactHeader* PX_RESTRICT contactHeader = &contactHeaders[batch.mConstraintBatchIndex];
		PxgTGSBlockSolverFrictionHeader* PX_RESTRICT frictionHeader = &frictionHeaders[batch.mConstraintBatchIndex];

		const uint numNormalConstr = contactHeader->numNormalConstr[threadIndex];
		const uint	totalFrictionConstr = frictionHeader->numFrictionConstr[threadIndex];
		const uint	numFrictionConstr = totalFrictionConstr & (~0x1);

		const PxReal maxPenBias = contactHeader->maxPenBias[threadIndex];

		PxgTGSBlockSolverContactPoint* PX_RESTRICT contacts = &contactPoints[batch.startConstraintIndex];

		PxgArticulationBlockResponse* PX_RESTRICT resp = &responses[batch.mArticulationResponseIndex];

		const float4 invMass0_1_angDom0_1 = contactHeader->invMass0_1_angDom0_1[threadIndex];

		const float4 normal_staticFriction = contactHeader->normal_staticFriction[threadIndex];

		const float restitutionXdt = contactHeader->restitutionXdt[threadIndex];
		const float p8 = contactHeader->p8[threadIndex];
		const float cfm = contactHeader->cfm[threadIndex];
		const PxU8 flags = (PxU8)contactHeader->flags[threadIndex];

		const PxVec3 normal = PxVec3(normal_staticFriction.x, normal_staticFriction.y, normal_staticFriction.z);

		//Bring forward a read event
		const float staticFrictionCof = normal_staticFriction.w;

		const PxVec3 relMotion = delta0.bottom - delta1.bottom;

		const float deltaV = normal.dot(relMotion);

		{
			for (uint i = 0; i < numNormalConstr; i++)
			{
				PxgTGSBlockSolverContactPoint& c = contacts[i];
				PxgArticulationBlockResponse& r = *resp;
				resp++;

				const float4 raXn_extraCoeff = c.raXn_extraCoeff[threadIndex];
				const PxVec3 raXn = PxVec3(raXn_extraCoeff.x, raXn_extraCoeff.y, raXn_extraCoeff.z);
				const float compliantContactCoef = raXn_extraCoeff.w;

				const float4 rbXn_targetVelW = c.rbXn_targetVelW[threadIndex];
				const float appliedForce = c.appliedForce[threadIndex];
				const float separation = c.separation[threadIndex];

				const float maxImpulse = c.maxImpulse[threadIndex];

				const PxVec3 rbXn = PxVec3(rbXn_targetVelW.x, rbXn_targetVelW.y, rbXn_targetVelW.z);

				const float targetVel = rbXn_targetVelW.w;

				float biasCoefficient = c.biasCoefficient[threadIndex];

				const float resp0 = ref0 * Pxldcs(c.resp0[threadIndex]);
				const float resp1 = ref1 * Pxldcs(c.resp1[threadIndex]);

				const float unitResponse = resp0 + resp1;
				const float recipResponse = (unitResponse > 0.f) ? (1.f / (unitResponse + cfm)) : 0.f;

				float velMultiplier = recipResponse;

				if (restitutionXdt < 0.f)
				{
					computeCompliantContactCoefficientsTGS(flags, restitutionXdt, unitResponse, recipResponse,
						compliantContactCoef, velMultiplier, biasCoefficient);
				}

				//Compute the normal velocity of the constraint.
				const PxReal v0 = angVel0.dot(raXn) + linVel0.dot(normal);
				const PxReal v1 = angVel1.dot(rbXn) + linVel1.dot(normal);
				const float normalVel = (v0 - v1);

				const PxReal deltaBias = deltaV + delta0.top.dot(raXn) - delta1.top.dot(rbXn) - targetVel * elapsedTime;

				const float sep = PxMax(minPen, separation + deltaBias);

				const PxReal biased = PxMin(-maxPenBias, biasCoefficient * sep);
				const PxReal tVelBias = recipResponse * biased;

				//KS - clamp the maximum force
				const float tempDeltaF = tVelBias - (normalVel - targetVel) * velMultiplier;
				const float _deltaF = fmaxf(tempDeltaF, -appliedForce);//FMax(FNegScaleSub(normalVel, velMultiplier, biasedErr), FNeg(appliedForce));
				const float _newForce = appliedForce + _deltaF;
				const float newForce = fminf(_newForce, maxImpulse);//FMin(_newForce, maxImpulse);
				const float deltaF = newForce - appliedForce;


				PxVec3 deltaRALin = ref0 * PxVec3(r.deltaRALin_x[threadIndex], r.deltaRALin_y[threadIndex], r.deltaRALin_z[threadIndex]);
				PxVec3 deltaRAAng = ref0 * PxVec3(r.deltaRAAng_x[threadIndex], r.deltaRAAng_y[threadIndex], r.deltaRAAng_z[threadIndex]);
				PxVec3 deltaRBLin = ref1 * PxVec3(r.deltaRBLin_x[threadIndex], r.deltaRBLin_y[threadIndex], r.deltaRBLin_z[threadIndex]);
				PxVec3 deltaRBAng = ref1 * PxVec3(r.deltaRBAng_x[threadIndex], r.deltaRBAng_y[threadIndex], r.deltaRBAng_z[threadIndex]);
				linVel0 += deltaRALin * deltaF;
				linVel1 += deltaRBLin * deltaF;
				angVel0 += deltaRAAng * deltaF;
				angVel1 += deltaRBAng * deltaF;

				imp0.top -= normal * deltaF;
				imp0.bottom -= raXn * deltaF;
				imp1.top += normal * deltaF;
				imp1.bottom += rbXn * deltaF;

				if(error)
					error->accumulateErrorLocal(deltaF, velMultiplier);

				c.appliedForce[threadIndex] = newForce;

				accumulatedNormalImpulse = accumulatedNormalImpulse + newForce;
			}

			accumulatedNormalImpulse = PxMax(accumulatedNormalImpulse, contactHeader->minNormalForce[threadIndex]);
		}

		if (numFrictionConstr)
		{
			PxgTGSBlockSolverContactFriction* PX_RESTRICT frictions = &frictionPoints[batch.startFrictionIndex];

			const float biasCoefficient = frictionHeader->biasCoefficient[threadIndex];

			const float dynamicFrictionCof = frictionHeader->dynamicFriction[threadIndex];
			const float maxFrictionImpulse = staticFrictionCof * accumulatedNormalImpulse;
			const float maxDynFrictionImpulse = dynamicFrictionCof * accumulatedNormalImpulse;

			PxU32 broken = 0;

			const float4 frictionNormal0 = frictionHeader->frictionNormals[0][threadIndex];
			const float4 frictionNormal1 = frictionHeader->frictionNormals[1][threadIndex];
			const PxVec3 normal0 = PxVec3(frictionNormal0.x, frictionNormal0.y, frictionNormal0.z);
			const PxVec3 normal1 = PxVec3(frictionNormal1.x, frictionNormal1.y, frictionNormal1.z);

			const PxReal deltaMotion0 = normal0.dot(relMotion);
			const PxReal deltaMotion1 = normal1.dot(relMotion);
			for (uint i = 0; i < numFrictionConstr; i += 2)
			{
				PxgTGSBlockSolverContactFriction& f0 = frictions[i];
				PxgArticulationBlockResponse& r0 = *resp;
				resp++;
				PxgTGSBlockSolverContactFriction& f1 = frictions[i + 1];
				PxgArticulationBlockResponse& r1 = *resp;
				resp++;

				const float4 raXn_error0 = f0.raXn_error[threadIndex];
				const float4 rbXn_targetVelW0 = f0.rbXn_targetVelW[threadIndex];
				const float initialError0 = raXn_error0.w;
				const float appliedForce0 = f0.appliedForce[threadIndex];
				const float targetVel0 = rbXn_targetVelW0.w;

				const float4 raXn_error1 = f1.raXn_error[threadIndex];
				const float4 rbXn_targetVelW1 = f1.rbXn_targetVelW[threadIndex];
				const float initialError1 = raXn_error1.w;
				const float appliedForce1 = f1.appliedForce[threadIndex];
				const float targetVel1 = rbXn_targetVelW1.w;

				const PxVec3 raXn0 = PxVec3(raXn_error0.x, raXn_error0.y, raXn_error0.z);
				const PxVec3 rbXn0 = PxVec3(rbXn_targetVelW0.x, rbXn_targetVelW0.y, rbXn_targetVelW0.z);

				const PxVec3 raXn1 = PxVec3(raXn_error1.x, raXn_error1.y, raXn_error1.z);
				const PxVec3 rbXn1 = PxVec3(rbXn_targetVelW1.x, rbXn_targetVelW1.y, rbXn_targetVelW1.z);

				const float resp0_0 = ref0 * f0.resp0[threadIndex];
				const float resp0_1 = ref1 * f0.resp1[threadIndex];
				const float resp0 = resp0_0 + resp0_1;
				const float velMultiplier0 = (resp0 > PX_EPS_REAL) ? (p8 / (resp0 + cfm)) : 0.f;

				const float resp1_0 = ref0 * f1.resp0[threadIndex];
				const float resp1_1 = ref1 * f1.resp1[threadIndex];
				const float resp1 = resp1_0 + resp1_1;
				const float velMultiplier1 = (resp1 > PX_EPS_REAL) ? (p8 / (resp1 + cfm)) : 0.f;

				const PxReal v00 = angVel0.dot(raXn0) + linVel0.dot(normal0);//V3MulAdd(linVel0, normal, V3Mul(angVel0, raXn));
				const PxReal v10 = angVel1.dot(rbXn0) + linVel1.dot(normal0);//V3MulAdd(linVel1, normal, V3Mul(angVel1, rbXn));
				const float normalVel0 = v00 - v10;

				const PxReal v01 = angVel0.dot(raXn1) + linVel0.dot(normal1);//V3MulAdd(linVel0, normal, V3Mul(angVel0, raXn));
				const PxReal v11 = angVel1.dot(rbXn1) + linVel1.dot(normal1);//V3MulAdd(linVel1, normal, V3Mul(angVel1, rbXn));
				const float normalVel1 = v01 - v11;

				const float error0 = initialError0 - targetVel0 * elapsedTime + (raXn0.dot(delta0.top) - rbXn0.dot(delta1.top) + deltaMotion0);
				const float bias0 = error0 * biasCoefficient;
				const float tmp10 = appliedForce0 - (bias0 - targetVel0) * velMultiplier0;
				const float totalImpulse0 = tmp10 - normalVel0 * velMultiplier0;

				const float error1 = initialError1 - targetVel1 * elapsedTime + (raXn1.dot(delta0.top) - rbXn1.dot(delta1.top) + deltaMotion1);
				const float bias1 = error1 * biasCoefficient;
				const float tmp11 = appliedForce1 - (bias1 - targetVel1) * velMultiplier1;
				const float totalImpulse1 = tmp11 - normalVel1 * velMultiplier1;

				const float totalImpulse = PxSqrt(totalImpulse0 * totalImpulse0 + totalImpulse1 * totalImpulse1);

				const bool clamp = totalImpulse > maxFrictionImpulse;

				const float ratio = clamp ? fminf(maxDynFrictionImpulse, totalImpulse) / totalImpulse : 1.f;

				const PxReal newAppliedForce0 = totalImpulse0 * ratio;
				const PxReal newAppliedForce1 = totalImpulse1 * ratio;

				float deltaF0 = newAppliedForce0 - appliedForce0;
				float deltaF1 = newAppliedForce1 - appliedForce1;

				if (error)
					error->accumulateErrorLocal(deltaF0, deltaF1, velMultiplier0, velMultiplier1);

				linVel0 += ref0 * PxVec3(r0.deltaRALin_x[threadIndex], r0.deltaRALin_y[threadIndex], r0.deltaRALin_z[threadIndex]) * deltaF0;
				linVel1 += ref1 * PxVec3(r0.deltaRBLin_x[threadIndex], r0.deltaRBLin_y[threadIndex], r0.deltaRBLin_z[threadIndex]) * deltaF0;
				angVel0 += ref0 * PxVec3(r0.deltaRAAng_x[threadIndex], r0.deltaRAAng_y[threadIndex], r0.deltaRAAng_z[threadIndex]) * deltaF0;
				angVel1 += ref1 * PxVec3(r0.deltaRBAng_x[threadIndex], r0.deltaRBAng_y[threadIndex], r0.deltaRBAng_z[threadIndex]) * deltaF0;

				linVel0 += ref0 * PxVec3(r1.deltaRALin_x[threadIndex], r1.deltaRALin_y[threadIndex], r1.deltaRALin_z[threadIndex]) * deltaF1;
				linVel1 += ref1 * PxVec3(r1.deltaRBLin_x[threadIndex], r1.deltaRBLin_y[threadIndex], r1.deltaRBLin_z[threadIndex]) * deltaF1;
				angVel0 += ref0 * PxVec3(r1.deltaRAAng_x[threadIndex], r1.deltaRAAng_y[threadIndex], r1.deltaRAAng_z[threadIndex]) * deltaF1;
				angVel1 += ref1 * PxVec3(r1.deltaRBAng_x[threadIndex], r1.deltaRBAng_y[threadIndex], r1.deltaRBAng_z[threadIndex]) * deltaF1;

				f0.appliedForce[threadIndex] = newAppliedForce0;
				f1.appliedForce[threadIndex] = newAppliedForce1;
				broken = broken | clamp;

				imp0.top -= normal0 * deltaF0;
				imp0.bottom -= raXn0 * deltaF0;
				imp1.top += normal0 * deltaF0;
				imp1.bottom += rbXn0 * deltaF0;

				imp0.top -= normal1 * deltaF1;
				imp0.bottom -= raXn1 * deltaF1;
				imp1.top += normal1 * deltaF1;
				imp1.bottom += rbXn1 * deltaF1;
			}

			if (numFrictionConstr < totalFrictionConstr)
			{
				//We have a torsional friction constraint

				const PxReal frictionScale = frictionHeader->torsionalFrictionScale[threadIndex];

				PxgTGSBlockSolverContactFriction& f0 = frictions[numFrictionConstr];
				PxgArticulationBlockResponse& r0 = *resp;
				resp++;

				const float4 raXn_error0 = f0.raXn_error[threadIndex];
				const float4 rbXn_targetVelW0 = f0.rbXn_targetVelW[threadIndex];
				const float appliedForce0 = f0.appliedForce[threadIndex];
				const float targetVel0 = rbXn_targetVelW0.w;

				const PxVec3 raXn0 = PxVec3(raXn_error0.x, raXn_error0.y, raXn_error0.z);
				const PxVec3 rbXn0 = PxVec3(rbXn_targetVelW0.x, rbXn_targetVelW0.y, rbXn_targetVelW0.z);

				const float resp0_0 = ref0 * f0.resp0[threadIndex];
				const float resp0_1 = ref1 * f0.resp1[threadIndex];
				const float resp0 = resp0_0 + resp0_1;
				const float velMultiplier0 = (resp0 > 0.f) ? (p8 / (resp0 + cfm)) : 0.f;

				const PxReal v00 = angVel0.dot(raXn0);
				const PxReal v10 = angVel1.dot(rbXn0);
				const float normalVel0 = v00 - v10;

				const float tmp10 = appliedForce0 - (-targetVel0) * velMultiplier0;
				const float totalImpulse = tmp10 - normalVel0 * velMultiplier0;

				const bool clamp = PxAbs(totalImpulse) > (maxFrictionImpulse * frictionScale);

				const PxReal totalClamped = PxClamp(totalImpulse, -maxDynFrictionImpulse * frictionScale, maxDynFrictionImpulse * frictionScale);

				const PxReal newAppliedForce = clamp ? totalClamped : totalImpulse;

				const PxReal deltaF0 = newAppliedForce - appliedForce0;
				if (error)
					error->accumulateErrorLocal(deltaF0, velMultiplier0);

				linVel0 += ref0 * PxVec3(r0.deltaRALin_x[threadIndex], r0.deltaRALin_y[threadIndex], r0.deltaRALin_z[threadIndex]) * deltaF0;
				linVel1 += ref1 * PxVec3(r0.deltaRBLin_x[threadIndex], r0.deltaRBLin_y[threadIndex], r0.deltaRBLin_z[threadIndex]) * deltaF0;
				angVel0 += ref0 * PxVec3(r0.deltaRAAng_x[threadIndex], r0.deltaRAAng_y[threadIndex], r0.deltaRAAng_z[threadIndex]) * deltaF0;
				angVel1 += ref1 * PxVec3(r0.deltaRBAng_x[threadIndex], r0.deltaRBAng_y[threadIndex], r0.deltaRBAng_z[threadIndex]) * deltaF0;

				f0.appliedForce[threadIndex] = newAppliedForce;
				broken = broken | clamp;

				imp0.bottom -= raXn0 * deltaF0;
				imp1.bottom += rbXn0 * deltaF0;
			}

			frictionHeader->broken[threadIndex] = broken;
		}

		vel0.bottom = linVel0;
		vel0.top = angVel0;
		vel1.bottom = linVel1;
		vel1.top = angVel1;

		impulse0 = imp0.scale(invMass0_1_angDom0_1.x * ref0, invMass0_1_angDom0_1.z * ref0);
		impulse1 = imp1.scale(invMass0_1_angDom0_1.y * ref1, invMass0_1_angDom0_1.w * ref1);
	}
}

// A light version of the function "solveExtContactBlockTGS" to quickly check if there is any active contact.
static __device__ PX_FORCE_INLINE bool checkExtActiveContactBlockTGS(const PxgBlockConstraintBatch& batch,
	const Cm::UnAlignedSpatialVector& vel0, const Cm::UnAlignedSpatialVector& vel1, const Cm::UnAlignedSpatialVector& delta0, const Cm::UnAlignedSpatialVector& delta1,
	const PxU32 threadIndex, PxgTGSBlockSolverContactHeader* PX_RESTRICT contactHeaders, PxgTGSBlockSolverContactPoint* PX_RESTRICT contactPoints, 
	PxgArticulationBlockResponse* PX_RESTRICT responses, const PxReal elapsedTime, const PxReal minPen)
{
	PxVec3 linVel0 = vel0.bottom;
	PxVec3 linVel1 = vel1.bottom;
	PxVec3 angVel0 = vel0.top;
	PxVec3 angVel1 = vel1.top;

	{
		PxgTGSBlockSolverContactHeader* PX_RESTRICT contactHeader = &contactHeaders[batch.mConstraintBatchIndex];
		const uint numNormalConstr = contactHeader->numNormalConstr[threadIndex];
		const PxReal maxPenBias = contactHeader->maxPenBias[threadIndex];
		PxgTGSBlockSolverContactPoint* PX_RESTRICT contacts = &contactPoints[batch.startConstraintIndex];
		PxgArticulationBlockResponse* PX_RESTRICT resp = &responses[batch.mArticulationResponseIndex];

		const float4 normal_staticFriction = contactHeader->normal_staticFriction[threadIndex];

		const float restitutionXdt = contactHeader->restitutionXdt[threadIndex];
		const float cfm = contactHeader->cfm[threadIndex];
		const PxU8 flags = (PxU8)contactHeader->flags[threadIndex];

		const PxVec3 normal = PxVec3(normal_staticFriction.x, normal_staticFriction.y, normal_staticFriction.z);

		// Bring forward a read event
		const PxVec3 relMotion = delta0.bottom - delta1.bottom;
		const float deltaV = normal.dot(relMotion);

		for(uint i = 0; i < numNormalConstr; i++)
		{
			PxgTGSBlockSolverContactPoint& c = contacts[i];
			resp++;

			const float4 raXn_extraCoeff = c.raXn_extraCoeff[threadIndex];
			const PxVec3 raXn = PxVec3(raXn_extraCoeff.x, raXn_extraCoeff.y, raXn_extraCoeff.z);
			const float compliantContactCoef = raXn_extraCoeff.w;

			const float4 rbXn_targetVelW = c.rbXn_targetVelW[threadIndex];
			const float appliedForce = c.appliedForce[threadIndex];
			const float separation = c.separation[threadIndex];

			const float maxImpulse = c.maxImpulse[threadIndex];

			const PxVec3 rbXn = PxVec3(rbXn_targetVelW.x, rbXn_targetVelW.y, rbXn_targetVelW.z);

			const float targetVel = rbXn_targetVelW.w;

			float biasCoefficient = c.biasCoefficient[threadIndex];

			const float resp0 = Pxldcs(c.resp0[threadIndex]);
			const float resp1 = Pxldcs(c.resp1[threadIndex]);

			const float unitResponse = resp0 + resp1;
			const float recipResponse = (unitResponse > 0.f) ? (1.f / (unitResponse + cfm)) : 0.f;

			float velMultiplier = recipResponse;

			if(restitutionXdt < 0.f)
			{
				computeCompliantContactCoefficientsTGS(flags, restitutionXdt, unitResponse, recipResponse,
				                                       compliantContactCoef, velMultiplier, biasCoefficient);
			}

			// Compute the normal velocity of the constraint.
			const PxReal v0 = angVel0.dot(raXn) + linVel0.dot(normal);
			const PxReal v1 = angVel1.dot(rbXn) + linVel1.dot(normal);
			const float normalVel = (v0 - v1);

			const PxReal deltaBias = deltaV + delta0.top.dot(raXn) - delta1.top.dot(rbXn) - targetVel * elapsedTime;

			const float sep = PxMax(minPen, separation + deltaBias);

			const PxReal biased = PxMin(-maxPenBias, biasCoefficient * sep);
			const PxReal tVelBias = recipResponse * biased;

			// KS - clamp the maximum force
			const float tempDeltaF = tVelBias - (normalVel - targetVel) * velMultiplier;
			const float _deltaF = fmaxf(tempDeltaF, -appliedForce); // FMax(FNegScaleSub(normalVel, velMultiplier,
			                                                        // biasedErr), FNeg(appliedForce));
			const float _newForce = appliedForce + _deltaF;
			const float newForce = fminf(_newForce, maxImpulse); // FMin(_newForce, maxImpulse);
			const float deltaF = newForce - appliedForce;

			// Check for active contact.
			if(PxAbs(deltaF) > 1.0e-8f)
			{
				return true;
			}
		}
	}

	return false;
}


#endif