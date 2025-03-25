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

#ifndef __SOLVER_BLOCK_TGS_CUH__
#define __SOLVER_BLOCK_TGS_CUH__

#include "common/PxPhysXCommonConfig.h"
#include <cuda.h>
#include <sm_35_intrinsics.h>
#include "PxgSolverBody.h"
//#include "PxgSolverConstraint1D.h"
#include "PxgSolverConstraintBlock1D.h"
#include "PxgSolverConstraintDesc.h"
#include "PxgConstraint.h"
#include "PxgConstraintBlock.h"
#include "PxgIslandContext.h"
#include "PxgSolverContext.h"
#include "cutil_math.h"
#include "PxgSolverCoreDesc.h"
#include "DyThresholdTable.h"
#include "PxgFrictionPatch.h"
#include "foundation/PxUtilities.h"
#include "PxgConstraintWriteBack.h"
#include "PxgSolverFlags.h"
#include "stdio.h"
#include "assert.h"
#include "PxgIntrinsics.h"
#include "solverResidual.cuh"

#include "DyCpuGpu1dConstraint.h"

#include "solverBlockCommon.cuh"
#include "constraintPrepShared.cuh"

using namespace physx;

static __forceinline__ __device__ float4 shfl(const PxU32 syncMask, const float4 f, const PxU32 index)
{
	float4 ret;
	ret.x = __shfl_sync(syncMask, f.x, index);
	ret.y = __shfl_sync(syncMask, f.y, index);
	ret.z = __shfl_sync(syncMask, f.z, index);
	ret.w = __shfl_sync(syncMask, f.w, index);

	return ret;
}

//Loads a set of TxIData inertia tensors quickly using shuffles
static __device__ void loadTxInertia(const PxU32 syncMask, const PxgSolverTxIData* datas, const PxU32 bodyIndex, const PxU32 nbToLoad, const PxU32 threadIndexInWarp, PxgSolverTxIData& out)
{
	if (1)
	{
		{
			float4 data0, data1, data2, data3;

			//There are 4x float4 in a PxgSolverTxIData
			const PxU32 elementIndex = threadIndexInWarp / 4;

			const PxU32 index0 = __shfl_sync(syncMask, bodyIndex, elementIndex);
			const PxU32 index1 = __shfl_sync(syncMask, bodyIndex, elementIndex + 8);
			const PxU32 index2 = __shfl_sync(syncMask, bodyIndex, elementIndex + 16);
			const PxU32 index3 = __shfl_sync(syncMask, bodyIndex, elementIndex + 24);
			if (elementIndex < nbToLoad)
			{
				data0 = reinterpret_cast<const float4*>(&datas[index0])[threadIndexInWarp & 3];
				if ((elementIndex + 8) < nbToLoad)
				{
					data1 = reinterpret_cast<const float4*>(&datas[index1])[threadIndexInWarp & 3];
					if ((elementIndex + 16) < nbToLoad)
					{
						data2 = reinterpret_cast<const float4*>(&datas[index2])[threadIndexInWarp & 3];
						if ((elementIndex + 24) < nbToLoad)
						{
							data3 = reinterpret_cast<const float4*>(&datas[index3])[threadIndexInWarp & 3];
						}
					}
				}
			}

			//OK. We have our 4 vectors...now we have to shuffle them
			//Thread 0 has the first float4 for T0, T8, T16, T24
			//Thread 1 has the 2nd float4 for T0, T8, T16, T24 ...
			//Thread 4 has the first float4 T1, T9, T17, T25...

			const PxU32 idxAnd3 = threadIndexInWarp & 3;

			float4 swapVal0 = idxAnd3 == 0 ? data0 : idxAnd3 == 1 ? data1 : idxAnd3 == 2 ? data2 : data3;
			float4 swapVal1 = idxAnd3 == 0 ? data3 : idxAnd3 == 1 ? data0 : idxAnd3 == 2 ? data1 : data2;
			float4 swapVal2 = idxAnd3 == 0 ? data2 : idxAnd3 == 1 ? data3 : idxAnd3 == 2 ? data0 : data1;
			float4 swapVal3 = idxAnd3 == 0 ? data1 : idxAnd3 == 1 ? data2 : idxAnd3 == 2 ? data3 : data0;

			const PxU32 threadReadBase = ((threadIndexInWarp * 4) & 31);

			const PxU32 offset = threadIndexInWarp / 8;

			float4 val0 = shfl(syncMask, swapVal0, threadReadBase + offset);
			float4 val1 = shfl(syncMask, swapVal1, threadReadBase + ((offset + 1) & 3));
			float4 val2 = shfl(syncMask, swapVal2, threadReadBase + ((offset + 2) & 3));
			float4 val3 = shfl(syncMask, swapVal3, threadReadBase + ((offset + 3) & 3));

			float4 shuffled0 = offset == 0 ? val0 : offset == 1 ? val3 : offset == 2 ? val2 : val1;
			float4 shuffled1 = offset == 0 ? val1 : offset == 1 ? val0 : offset == 2 ? val3 : val2;
			float4 shuffled2 = offset == 0 ? val2 : offset == 1 ? val1 : offset == 2 ? val0 : val3;
			float4 shuffled3 = offset == 0 ? val3 : offset == 1 ? val2 : offset == 2 ? val1 : val0;

			//Now copy into the structure...
			out.deltaBody2World.q.x = shuffled0.x; out.deltaBody2World.q.y = shuffled0.y; out.deltaBody2World.q.z = shuffled0.z; out.deltaBody2World.q.w = shuffled0.w;
			out.deltaBody2World.p.x = shuffled1.x; out.deltaBody2World.p.y = shuffled1.y; out.deltaBody2World.p.z = shuffled1.z;
			out.sqrtInvInertia.column0.x = shuffled1.w; out.sqrtInvInertia.column0.y = shuffled2.x; out.sqrtInvInertia.column0.z = shuffled2.y;
			out.sqrtInvInertia.column1.x = shuffled2.z; out.sqrtInvInertia.column1.y = shuffled2.w; out.sqrtInvInertia.column1.z = shuffled3.x;
			out.sqrtInvInertia.column2.x = shuffled3.y; out.sqrtInvInertia.column2.y = shuffled3.z; out.sqrtInvInertia.column2.z = shuffled3.w;
			
		}
	}
	else
	{
		if (threadIndexInWarp < nbToLoad)
			out = datas[bodyIndex];
	}
}

// Using the same logic as the previous implementation, but mass-splitting is additionally performed per sub-timestep.
// Required data is packed and stored differently from the previous implementation to split mass at each sub-timestep; 
// i.e., mass-related terms are computed at every sub-timestep.
// Refer to "Mass Splitting for Jitter-Free Parallel Rigid Body Simulation" for the general mass-splitting idea.

static __device__ void solveContactBlockTGS(const PxgBlockConstraintBatch& batch, PxVec3& b0LinVel, PxVec3& b0AngVel, PxVec3& b1LinVel, PxVec3& b1AngVel,
	const PxVec3& b0LinDelta, const PxVec3& b0AngDelta, const PxVec3& b1LinDelta, const PxVec3& b1AngDelta,
	const PxU32 threadIndex, const PxgTGSBlockSolverContactHeader* PX_RESTRICT contactHeaders,
	PxgTGSBlockSolverFrictionHeader* PX_RESTRICT frictionHeaders, PxgTGSBlockSolverContactPoint* PX_RESTRICT contactPoints,
	PxgTGSBlockSolverContactFriction* PX_RESTRICT frictionPoints,
	const PxReal elapsedTime, const PxReal minPen, PxgErrorAccumulator* error, 
	PxReal ref0 = 1.f, PxReal ref1 = 1.f)
{  
	using namespace physx;

	PxVec3 linVel0(b0LinVel.x, b0LinVel.y, b0LinVel.z);
	PxVec3 linVel1(b1LinVel.x, b1LinVel.y, b1LinVel.z);
	PxVec3 angVel0(b0AngVel.x, b0AngVel.y, b0AngVel.z);
	PxVec3 angVel1(b1AngVel.x, b1AngVel.y, b1AngVel.z);

	float accumulatedNormalImpulse = 0.f;

	{
		const PxgTGSBlockSolverContactHeader* PX_RESTRICT contactHeader = &contactHeaders[batch.mConstraintBatchIndex];
		PxgTGSBlockSolverFrictionHeader* PX_RESTRICT frictionHeader = &frictionHeaders[batch.mConstraintBatchIndex];
		PxgTGSBlockSolverContactFriction* PX_RESTRICT frictions = &frictionPoints[batch.startFrictionIndex];

		const uint numNormalConstr = Pxldcs(contactHeader->numNormalConstr[threadIndex]);
		const uint numFrictionConstrTotal = Pxldcs(frictionHeader->numFrictionConstr[threadIndex]);
		const uint numFrictionConstr = numFrictionConstrTotal & (~1);

		float accumDeltaF = 0.f;

		if (numNormalConstr)
		{
			const PxReal maxPenBias = Pxldcs(contactHeader->maxPenBias[threadIndex]);

			PxgTGSBlockSolverContactPoint* PX_RESTRICT contacts = &contactPoints[batch.startConstraintIndex];
			const float4 invMass0_1_angDom0_1 = Pxldcs(contactHeader->invMass0_1_angDom0_1[threadIndex]);

			const float invMassA = ref0 * invMass0_1_angDom0_1.x;
			const float invMassB = ref1 * invMass0_1_angDom0_1.y;

			const float angDom0 = ref0 * invMass0_1_angDom0_1.z;
			const float angDom1 = ref1 * invMass0_1_angDom0_1.w;

			const float sumInvMass = invMassA + invMassB;

			const float4 normal_staticFriction = Pxldcg(contactHeader->normal_staticFriction[threadIndex]);

			const PxVec3 normal = PxVec3(normal_staticFriction.x, normal_staticFriction.y, normal_staticFriction.z);

			const float restitutionXdt = contactHeader->restitutionXdt[threadIndex];
			const float p8 = contactHeader->p8[threadIndex]; // using previous p8 value in the prep step.
			const PxU8 flags = contactHeader->flags[threadIndex];

			const PxVec3 delLinVel0 = normal * invMassA;
			const PxVec3 delLinVel1 = normal * invMassB;

			//Bring forward a read event
			const float staticFrictionCof = normal_staticFriction.w;

			const PxVec3 relMotion = b0LinDelta - b1LinDelta;

			const float deltaV = normal.dot(relMotion);

			float relVel1 = (linVel0 - linVel1).dot(normal);

			float4 next_raXn_extraCoeff = Pxldcs(contacts->raXn_extraCoeff[threadIndex]);
			float4 next_rbXn_targetVelW = Pxldcs(contacts->rbXn_targetVelW[threadIndex]);
			float next_appliedForce = Pxldcs(contacts->appliedForce[threadIndex]);
			float next_error = Pxldcs(contacts->separation[threadIndex]);
			float next_maxImpulse = Pxldcs(contacts->maxImpulse[threadIndex]);
			float next_biasCoefficient = Pxldcs(contacts->biasCoefficient[threadIndex]);

			float next_resp0 = ref0 * Pxldcs(contacts->resp0[threadIndex]);
			float next_resp1 = ref1 * Pxldcs(contacts->resp1[threadIndex]);

			float next_unitResponse = next_resp0 + next_resp1;
			float next_recipResponse = (next_unitResponse > 0.f) ? (1.f / next_unitResponse) : 0.f;
			float next_velMultiplier = next_recipResponse;

			if (restitutionXdt < 0.f)
			{
				computeCompliantContactCoefficientsTGS(flags, restitutionXdt, next_unitResponse,
					next_recipResponse, next_raXn_extraCoeff.w, next_velMultiplier,
					next_biasCoefficient);
			}

			{
				for (uint i = 0; i < numNormalConstr; i++)
				{
					PxgTGSBlockSolverContactPoint& c = contacts[i];

					const float4 raXn_contactCoeff = next_raXn_extraCoeff;
					const float velMultiplier = next_velMultiplier;
					const float4 rbXn_targetVelW = next_rbXn_targetVelW;
					const float appliedForce = next_appliedForce;
					const float separation = next_error;
					const float maxImpulse = next_maxImpulse;
					const float biasCoefficient = next_biasCoefficient;
					const float recipResponse = next_recipResponse;

					if ((i + 1) < numNormalConstr)
					{
						const PxgTGSBlockSolverContactPoint& nextC = contacts[i + 1];

						next_raXn_extraCoeff = Pxldcs(nextC.raXn_extraCoeff[threadIndex]);
						next_rbXn_targetVelW = Pxldcs(nextC.rbXn_targetVelW[threadIndex]);
						next_appliedForce = Pxldcs(nextC.appliedForce[threadIndex]);
						next_error = Pxldcs(nextC.separation[threadIndex]);
						next_maxImpulse = Pxldcs(nextC.maxImpulse[threadIndex]);
						next_biasCoefficient = Pxldcs(nextC.biasCoefficient[threadIndex]);

						next_resp0 = ref0 * Pxldcs(nextC.resp0[threadIndex]);
						next_resp1 = ref1 * Pxldcs(nextC.resp1[threadIndex]);

						next_unitResponse = next_resp0 + next_resp1;
						next_recipResponse = (next_unitResponse > 0.f) ? (1.f / next_unitResponse) : 0.f;

						next_velMultiplier = next_recipResponse;

						if (restitutionXdt < 0.f)
						{
							computeCompliantContactCoefficientsTGS(flags, restitutionXdt, next_unitResponse,
								next_recipResponse, next_raXn_extraCoeff.w, next_velMultiplier,
								next_biasCoefficient);
						}
					}
					else if (numFrictionConstr)
					{
						next_raXn_extraCoeff = Pxldcs(frictions[0].raXn_error[threadIndex]);
						next_rbXn_targetVelW = Pxldcs(frictions[0].rbXn_targetVelW[threadIndex]);
						next_error = next_raXn_extraCoeff.w;
						next_appliedForce = Pxldcs(frictions[0].appliedForce[threadIndex]);

						next_resp0 = ref0 * Pxldcs(frictions[0].resp0[threadIndex]);
						next_resp1 = ref1 * Pxldcs(frictions[0].resp1[threadIndex]);

						const float next_resp = next_resp0 + next_resp1;
						next_velMultiplier = (next_resp > 0.f) ? (p8 / next_resp) : 0.f;
					}

					const PxVec3 raXn = PxVec3(raXn_contactCoeff.x, raXn_contactCoeff.y, raXn_contactCoeff.z);
					const PxVec3 rbXn = PxVec3(rbXn_targetVelW.x, rbXn_targetVelW.y, rbXn_targetVelW.z);
					const float targetVel = rbXn_targetVelW.w;

					//Compute the normal velocity of the constraint.
					const PxReal v0 = angVel0.dot(raXn);
					const PxReal v1 = angVel1.dot(rbXn);
					const float normalVel = relVel1 + (v0 - v1);

					const float sep = PxMax(minPen, separation + deltaV + b0AngDelta.dot(raXn) - b1AngDelta.dot(rbXn));

					//How much the target vel should have changed the position of this constraint...
					const PxReal tVelErr = targetVel * elapsedTime;

					const PxReal biasedErr = recipResponse * fminf(-maxPenBias, biasCoefficient * (sep - tVelErr));

					//KS - clamp the maximum force
					const float tempDeltaF = biasedErr - (normalVel - targetVel) * velMultiplier;
					const float _deltaF = fmaxf(tempDeltaF, -appliedForce);//FMax(FNegScaleSub(normalVel, velMultiplier, biasedErr), FNeg(appliedForce));
					const float _newForce = appliedForce + _deltaF;
					const float newForce = fminf(_newForce, maxImpulse);//FMin(_newForce, maxImpulse);
					const float deltaF = newForce - appliedForce;

					accumDeltaF += deltaF;

					relVel1 += sumInvMass * deltaF;

					angVel0 += raXn * (deltaF * angDom0);
					angVel1 -= rbXn * (deltaF * angDom1);

					if(error)
						error->accumulateErrorLocal(deltaF, velMultiplier);

					Pxstcs(&c.appliedForce[threadIndex], newForce);

					accumulatedNormalImpulse = accumulatedNormalImpulse + newForce;
				}
			}

			linVel0 += delLinVel0 * accumDeltaF;
			linVel1 -= delLinVel1 * accumDeltaF;

			if (numFrictionConstr)
			{
				const float biasCoefficient = Pxldcg(frictionHeader->biasCoefficient[threadIndex]);

				const float dynamicFrictionCof = Pxldcg(frictionHeader->dynamicFriction[threadIndex]);
				const float maxFrictionImpulse = staticFrictionCof * accumulatedNormalImpulse;
				const float maxDynFrictionImpulse = dynamicFrictionCof * accumulatedNormalImpulse;

				PxU32 broken = 0;

				const float4 frictionNormal0 = Pxldcg(frictionHeader->frictionNormals[0][threadIndex]);
				const float4 frictionNormal1 = Pxldcg(frictionHeader->frictionNormals[1][threadIndex]);
				const PxVec3 normal0 = PxVec3(frictionNormal0.x, frictionNormal0.y, frictionNormal0.z);
				const PxVec3 normal1 = PxVec3(frictionNormal1.x, frictionNormal1.y, frictionNormal1.z);

				const PxVec3 delLinVel00 = normal0 * invMassA;
				const PxVec3 delLinVel10 = normal0 * invMassB;
				const PxVec3 delLinVel01 = normal1 * invMassA;
				const PxVec3 delLinVel11 = normal1 * invMassB;

				float relDelta0 = relMotion.dot(normal0);
				float relDelta1 = relMotion.dot(normal1);

				for (uint i = 0; i < numFrictionConstr; i += 2)
				{
					PxgTGSBlockSolverContactFriction& f0 = frictions[i];
					PxgTGSBlockSolverContactFriction& f1 = frictions[i + 1];

					const float4 raXn_extraCoeff1 = f1.raXn_error[threadIndex];
					const float4 rbXn_targetVelW1 = f1.rbXn_targetVelW[threadIndex];
					const float initialError1 = raXn_extraCoeff1.w;
					const float appliedForce1 = f1.appliedForce[threadIndex];
					const float targetVel1 = rbXn_targetVelW1.w;

					const float4 raXn_extraCoeff0 = next_raXn_extraCoeff;
					const float4 rbXn_targetVelW0 = next_rbXn_targetVelW;
					const float initialError0 = next_error;
					const float appliedForce0 = next_appliedForce;
					const float targetVel0 = rbXn_targetVelW0.w;

					const float f1_resp0 = ref0 * f1.resp0[threadIndex];
					const float f1_resp1 = ref1 * f1.resp1[threadIndex];
					const float f1_resp = f1_resp0 + f1_resp1;
					const float velMultiplier1 = (f1_resp > 0.f) ? (p8 / f1_resp) : 0.f;
					const float velMultiplier0 = next_velMultiplier;

					if ((i + 2) < numFrictionConstrTotal)
					{
						next_raXn_extraCoeff = Pxldcs(frictions[i + 2].raXn_error[threadIndex]);
						next_rbXn_targetVelW = Pxldcs(frictions[i + 2].rbXn_targetVelW[threadIndex]);
						next_error = next_raXn_extraCoeff.w;
						next_appliedForce = Pxldcs(frictions[i + 2].appliedForce[threadIndex]);

						next_resp0 = ref0 * Pxldcs(frictions[i + 2].resp0[threadIndex]);
						next_resp1 = ref1 * Pxldcs(frictions[i + 2].resp1[threadIndex]);

						const float next_resp = next_resp0 + next_resp1;
						next_velMultiplier = (next_resp > 0.f) ? (p8 / next_resp) : 0.f;
					}

					const PxVec3 raXn0 = PxVec3(raXn_extraCoeff0.x, raXn_extraCoeff0.y, raXn_extraCoeff0.z);
					const PxVec3 rbXn0 = PxVec3(rbXn_targetVelW0.x, rbXn_targetVelW0.y, rbXn_targetVelW0.z);
					const PxReal v00 = angVel0.dot(raXn0) + linVel0.dot(normal0);
					const PxReal v10 = angVel1.dot(rbXn0) + linVel1.dot(normal0);
					const float normalVel0 = v00 - v10;

					const float error0 = initialError0 - targetVel0 * elapsedTime +
						raXn0.dot(b0AngDelta) - rbXn0.dot(b1AngDelta) + relDelta0;

					const float bias0 = error0 * biasCoefficient;
					const float tmp10 = appliedForce0 - (bias0 - targetVel0) * velMultiplier0;
					const float totalImpulse0 = tmp10 - normalVel0 * velMultiplier0;

					const PxVec3 raXn1 = PxVec3(raXn_extraCoeff1.x, raXn_extraCoeff1.y, raXn_extraCoeff1.z);
					const PxVec3 rbXn1 = PxVec3(rbXn_targetVelW1.x, rbXn_targetVelW1.y, rbXn_targetVelW1.z);

					const PxReal v01 = angVel0.dot(raXn1) + linVel0.dot(normal1);
					const PxReal v11 = angVel1.dot(rbXn1) + linVel1.dot(normal1);
					const float normalVel1 = v01 - v11;

					const float error1 = initialError1 - targetVel1 * elapsedTime +
						raXn1.dot(b0AngDelta) - rbXn1.dot(b1AngDelta) + relDelta1;

					const float bias1 = error1 * biasCoefficient;
					const float tmp11 = appliedForce1 - (bias1 - targetVel1) * velMultiplier1;
					const float totalImpulse1 = tmp11 - normalVel1 * velMultiplier1;

					const float totalImpulse = PxSqrt(totalImpulse0 * totalImpulse0 + totalImpulse1 * totalImpulse1);

					const bool clamp = totalImpulse > maxFrictionImpulse;

					const float ratio = clamp ? fminf(maxDynFrictionImpulse, totalImpulse) / totalImpulse : 1.f;

					const float newAppliedForce0 = totalImpulse0 * ratio;
					const float newAppliedForce1 = totalImpulse1 * ratio;

					float deltaF0 = newAppliedForce0 - appliedForce0;
					float deltaF1 = newAppliedForce1 - appliedForce1;

					if (error)
						error->accumulateErrorLocal(deltaF0, deltaF1, velMultiplier0, velMultiplier1);

					linVel0 += delLinVel00 * deltaF0;
					linVel1 -= delLinVel10 * deltaF0;
					angVel0 += raXn0 * (deltaF0 * angDom0);
					angVel1 -= rbXn0 * (deltaF0 * angDom1);

					linVel0 += delLinVel01 * deltaF1;
					linVel1 -= delLinVel11 * deltaF1;
					angVel0 += raXn1 * (deltaF1 * angDom0);
					angVel1 -= rbXn1 * (deltaF1 * angDom1);

					Pxstcs(&f0.appliedForce[threadIndex], newAppliedForce0);
					Pxstcs(&f1.appliedForce[threadIndex], newAppliedForce1);
					broken = broken | clamp;
				}

				if (numFrictionConstr < numFrictionConstrTotal)
				{
					const PxReal frictionScale = frictionHeader->torsionalFrictionScale[threadIndex];
			
					//We have a torsional friction anchor, solve this...
					PxgTGSBlockSolverContactFriction& f0 = frictions[numFrictionConstr];
					const float4 raXn_extraCoeff0 = next_raXn_extraCoeff;
					const PxVec3 raXn0 = PxVec3(raXn_extraCoeff0.x, raXn_extraCoeff0.y, raXn_extraCoeff0.z);
					const float velMultiplier0 = next_velMultiplier;
					const float4 rbXn_targetVelW0 = next_rbXn_targetVelW;
					const float appliedForce0 = next_appliedForce;
					const float targetVel0 = rbXn_targetVelW0.w;

					const PxVec3 rbXn0 = PxVec3(rbXn_targetVelW0.x, rbXn_targetVelW0.y, rbXn_targetVelW0.z);

					const PxReal v00 = angVel0.dot(raXn0);
					const PxReal v10 = angVel1.dot(rbXn0);
					const float normalVel0 = v00 - v10;

					const float tmp10 = appliedForce0 - (-targetVel0) * velMultiplier0;
					const float totalImpulse = tmp10 - normalVel0 * velMultiplier0;

					const bool clamp = PxAbs(totalImpulse) > (maxFrictionImpulse * frictionScale);

					const PxReal totalClamped = PxClamp(totalImpulse, -maxDynFrictionImpulse * frictionScale, maxDynFrictionImpulse * frictionScale);

					const PxReal newAppliedForce = clamp ? totalClamped : totalImpulse;

					float deltaF = newAppliedForce - appliedForce0;
					if (error)
						error->accumulateErrorLocal(deltaF, velMultiplier0);

					angVel0 += raXn0 * (deltaF * angDom0);
					angVel1 -= rbXn0 * (deltaF * angDom1);

					Pxstcs(&f0.appliedForce[threadIndex], newAppliedForce);
					broken = broken | clamp;
				}

				Pxstcs(&frictionHeader->broken[threadIndex], broken);
			}
		}
	}

	// Write back
	b0LinVel = PxVec3(linVel0.x, linVel0.y, linVel0.z);
	b0AngVel = PxVec3(angVel0.x, angVel0.y, angVel0.z);
	b1LinVel = PxVec3(linVel1.x, linVel1.y, linVel1.z);
	b1AngVel = PxVec3(angVel1.x, angVel1.y, angVel1.z);
}

// A light version of the function "solveContactBlockTGS" to quickly check if there is any active contact.
// TODO: Make this even lighter.

static __device__ bool checkActiveContactBlockTGS(const PxgBlockConstraintBatch& batch, const PxVec3& linVel0, const PxVec3& angVel0,
	const PxVec3& linVel1, const PxVec3& angVel1,	const PxVec3& b0LinDelta, const PxVec3& b0AngDelta, const PxVec3& b1LinDelta,
	const PxVec3& b1AngDelta, const PxU32 threadIndex, const PxgTGSBlockSolverContactHeader* PX_RESTRICT contactHeaders,
	PxgTGSBlockSolverContactPoint* PX_RESTRICT contactPoints, const PxReal elapsedTime, const PxReal minPen)
{
	using namespace physx;

	{
		const PxgTGSBlockSolverContactHeader* PX_RESTRICT contactHeader = &contactHeaders[batch.mConstraintBatchIndex];
		const uint numNormalConstr = Pxldcs(contactHeader->numNormalConstr[threadIndex]);

		if (numNormalConstr)
		{
			const PxReal maxPenBias = Pxldcs(contactHeader->maxPenBias[threadIndex]);

			PxgTGSBlockSolverContactPoint* PX_RESTRICT contacts = &contactPoints[batch.startConstraintIndex];
			const float4 invMass0_1_angDom0_1 = Pxldcs(contactHeader->invMass0_1_angDom0_1[threadIndex]);

			const float invMassA = invMass0_1_angDom0_1.x;
			const float invMassB = invMass0_1_angDom0_1.y;

			const float4 normal_staticFriction = Pxldcg(contactHeader->normal_staticFriction[threadIndex]);
			const PxVec3 normal = PxVec3(normal_staticFriction.x, normal_staticFriction.y, normal_staticFriction.z);

			const float restitutionXdt = contactHeader->restitutionXdt[threadIndex];
			const PxU8 flags = contactHeader->flags[threadIndex];
			const PxVec3 delLinVel0 = normal * invMassA;
			const PxVec3 delLinVel1 = normal * invMassB;

			//Bring forward a read event
			const PxVec3 relMotion = b0LinDelta - b1LinDelta;
			const float deltaV = normal.dot(relMotion);
			float relVel1 = (linVel0 - linVel1).dot(normal);

			float4 next_raXn_extraCoeff = Pxldcs(contacts->raXn_extraCoeff[threadIndex]);
			float4 next_rbXn_targetVelW = Pxldcs(contacts->rbXn_targetVelW[threadIndex]);
			float next_appliedForce = Pxldcs(contacts->appliedForce[threadIndex]);
			float next_error = Pxldcs(contacts->separation[threadIndex]);
			float next_maxImpulse = Pxldcs(contacts->maxImpulse[threadIndex]);
			float next_biasCoefficient = Pxldcs(contacts->biasCoefficient[threadIndex]);

			float next_resp0 = Pxldcs(contacts->resp0[threadIndex]);
			float next_resp1 = Pxldcs(contacts->resp1[threadIndex]);

			float next_unitResponse = next_resp0 + next_resp1;
			float next_recipResponse = (next_unitResponse > 0.f) ? (1.f / next_unitResponse) : 0.f;
			float next_velMultiplier = next_recipResponse;

			if (restitutionXdt < 0.f)
			{
				computeCompliantContactCoefficientsTGS(flags, restitutionXdt, next_unitResponse,
					next_recipResponse, next_raXn_extraCoeff.w, next_velMultiplier,
					next_biasCoefficient);
			}

			{
				for (uint i = 0; i < numNormalConstr; i++)
				{
					const float4 raXn_contactCoeff = next_raXn_extraCoeff;
					const float velMultiplier = next_velMultiplier;
					const float4 rbXn_targetVelW = next_rbXn_targetVelW;
					const float appliedForce = next_appliedForce;
					const float separation = next_error;
					const float maxImpulse = next_maxImpulse;
					const float biasCoefficient = next_biasCoefficient;
					const float recipResponse = next_recipResponse;

					if ((i + 1) < numNormalConstr)
					{
						const PxgTGSBlockSolverContactPoint& nextC = contacts[i + 1];
						next_raXn_extraCoeff = Pxldcs(nextC.raXn_extraCoeff[threadIndex]);
						next_rbXn_targetVelW = Pxldcs(nextC.rbXn_targetVelW[threadIndex]);
						next_appliedForce = Pxldcs(nextC.appliedForce[threadIndex]);
						next_error = Pxldcs(nextC.separation[threadIndex]);
						next_maxImpulse = Pxldcs(nextC.maxImpulse[threadIndex]);
						next_biasCoefficient = Pxldcs(nextC.biasCoefficient[threadIndex]);

						next_resp0 = Pxldcs(nextC.resp0[threadIndex]);
						next_resp1 = Pxldcs(nextC.resp1[threadIndex]);

						next_unitResponse = next_resp0 + next_resp1;
						next_recipResponse = (next_unitResponse > 0.f) ? (1.f / next_unitResponse) : 0.f;

						next_velMultiplier = next_recipResponse;

						if (restitutionXdt < 0.f)
						{
							computeCompliantContactCoefficientsTGS(flags, restitutionXdt, next_unitResponse,
								next_recipResponse, next_raXn_extraCoeff.w, next_velMultiplier,
								next_biasCoefficient);
						}
					}

					const PxVec3 raXn = PxVec3(raXn_contactCoeff.x, raXn_contactCoeff.y, raXn_contactCoeff.z);
					const PxVec3 rbXn = PxVec3(rbXn_targetVelW.x, rbXn_targetVelW.y, rbXn_targetVelW.z);
					const float targetVel = rbXn_targetVelW.w;

					//Compute the normal velocity of the constraint.
					const PxReal v0 = angVel0.dot(raXn);
					const PxReal v1 = angVel1.dot(rbXn);
					const float normalVel = relVel1 + (v0 - v1);

					const float sep = PxMax(minPen, separation + deltaV + b0AngDelta.dot(raXn) - b1AngDelta.dot(rbXn));

					//How much the target vel should have changed the position of this constraint...
					const PxReal tVelErr = targetVel * elapsedTime;
					const PxReal biasedErr = recipResponse * fminf(-maxPenBias, biasCoefficient * (sep - tVelErr));

					//KS - clamp the maximum force
					const float tempDeltaF = biasedErr - (normalVel - targetVel) * velMultiplier;
					const float _deltaF = fmaxf(tempDeltaF, -appliedForce);//FMax(FNegScaleSub(normalVel, velMultiplier, biasedErr), FNeg(appliedForce));
					const float _newForce = appliedForce + _deltaF;
					const float newForce = fminf(_newForce, maxImpulse);//FMin(_newForce, maxImpulse);
					const float deltaF = newForce - appliedForce;
					
					// active contact
					if (PxAbs(deltaF) > 1.0e-8f)
					{
						return true;
					}
				}
			}
		}
	}

	return false;
}

static __device__ void concludeContactBlockTGS(const PxgBlockConstraintBatch& batch, const PxU32 threadIndex, PxgTGSBlockSolverContactHeader* contactHeaders, PxgTGSBlockSolverFrictionHeader* frictionHeaders,
	PxgTGSBlockSolverContactPoint* contactPoints, PxgTGSBlockSolverContactFriction* frictions)
{

#if 0
	using namespace physx;

	{
		const PxgTGSBlockSolverContactHeader* contactHeader = &contactHeaders[batch.mConstraintBatchIndex];
		PxgTGSBlockSolverFrictionHeader* frictionHeader = &frictionHeaders[batch.mConstraintBatchIndex];

		const uint32_t numNormalConstr = contactHeader->numNormalConstr[threadIndex];
		//const uint32_t numFrictionConstr = frictionHeader->numFrictionConstr[threadIndex];

		PxgTGSBlockSolverContactPoint* contacts = &contactPoints[batch.startConstraintIndex];
		for(uint32_t i=0;i<numNormalConstr;i++)
		{
			//contactPoints[i].setScaledBias(fmaxf(contactPoints[i].getScaledBias(), 0.f));
			contacts[i].biasCoefficeint[threadIndex] = 0.f;
		}

		frictionHeader->biasCoefficient[threadIndex] = 0.f;

		//PxgTGSBlockSolverContactFriction* frictionConstr = &frictions[batch.startFrictionIndex];
		//for(uint32_t i=0;i<numFrictionConstr;i++)
		//{
		//	//frictionConstr[i].setBias(0.f);
		//	frictionConstr[i].bias[threadIndex] = 0.f;
		//}
	}
#endif
}



static __device__ void writeBackContactBlockTGS(const PxgBlockConstraintBatch& batch, const PxU32 threadIndex,
											 const PxgSolverBodyData* bodies, Dy::ThresholdStreamElement* thresholdStream,
											 PxI32* sharedThresholdStreamIndex, PxgTGSBlockSolverContactHeader* contactHeaders, PxgTGSBlockSolverFrictionHeader* frictionHeaders,
											PxgTGSBlockSolverContactPoint* contactPoints, PxgTGSBlockSolverContactFriction* frictions,
											PxF32* forcewritebackBuffer, PxgBlockFrictionPatch& frictionPatchBlock,
											PxgFrictionPatchGPU* frictionPatches)
{
	const PxU32 bodyAIndex = batch.bodyAIndex[threadIndex];
	const PxU32 bodyBIndex = batch.bodyBIndex[threadIndex];

	const PxgSolverBodyData& bd0 = bodies[bodyAIndex];
	const PxgSolverBodyData& bd1 = bodies[bodyBIndex];
	bool forceThreshold = false;

	float normalForce = 0.f;

	{
		const PxgTGSBlockSolverContactHeader* PX_RESTRICT contactHeader = &contactHeaders[batch.mConstraintBatchIndex];
		const PxgTGSBlockSolverFrictionHeader* PX_RESTRICT frictionHeader = &frictionHeaders[batch.mConstraintBatchIndex];
	
		PxU32 forceWritebackOffset = contactHeader->forceWritebackOffset[threadIndex];

		forceThreshold = contactHeader->flags[threadIndex] & PxgSolverContactFlags::eHAS_FORCE_THRESHOLDS;

		const PxU32	numFrictionConstr = frictionHeader->numFrictionConstr[threadIndex];

		const PxU32 numNormalConstr = contactHeader->numNormalConstr[threadIndex];
		if(forceWritebackOffset!=0xFFFFFFFF)
		{
			PxReal* vForceWriteback = &forcewritebackBuffer[forceWritebackOffset];
			PxgTGSBlockSolverContactPoint* c = &contactPoints[batch.startConstraintIndex];
			for(PxU32 i=0; i<numNormalConstr; i++)
			{
				const PxReal appliedForce = c[i].appliedForce[threadIndex];//FStore(c->getAppliedForce());
				*vForceWriteback++ = appliedForce;
				normalForce += appliedForce;
			}
		}

		writeBackContactBlockFriction(threadIndex, numFrictionConstr, frictionHeader,
			frictionPatchBlock, frictions + batch.startFrictionIndex, frictionPatches);

		if(numFrictionConstr && frictionHeader->broken[threadIndex])
		{
			frictionPatchBlock.broken[threadIndex] = 1;
		}
	}

	float reportThreshold0 = bd0.reportThreshold;
	float reportThreshold1 = bd1.reportThreshold;

	if((forceThreshold && normalForce !=0 && (reportThreshold0 < PX_MAX_REAL  || reportThreshold1 < PX_MAX_REAL)))
	{
		//ToDo : support PxgThresholdStreamElement
		Dy::ThresholdStreamElement elt;
		elt.normalForce = normalForce;
		elt.threshold = PxMin<float>(reportThreshold0, reportThreshold1);
		
		elt.nodeIndexA = bd0.islandNodeIndex;
		elt.nodeIndexB = bd1.islandNodeIndex;
		elt.shapeInteraction = batch.shapeInteraction[threadIndex];
		PxOrder(elt.nodeIndexA, elt.nodeIndexB);
		assert(elt.nodeIndexA < elt.nodeIndexB);

		PxI32 index = atomicAdd(sharedThresholdStreamIndex, 1);

		//KS - force a 16-byte coalesced write
		//((float4*)thresholdStream)[index] = *((float4*)&elt);
		thresholdStream[index] = elt;
	}
}

// Using the same logic as the previous implementation, but mass-splitting is additionally performed per sub-timestep.
// Required data is packed and stored differently from the previous implementation to split mass at each sub-timestep; 
// i.e., mass-related terms are computed at every sub-timestep.
// Refer to "Mass Splitting for Jitter-Free Parallel Rigid Body Simulation" for the general mass-splitting idea.


static __device__ void solve1DBlockTGS(const PxgBlockConstraintBatch& batch, PxVec3& b0LinVel, PxVec3& b0AngVel, PxVec3& b1LinVel, PxVec3& b1AngVel,
	const PxVec3& linDelta0, const PxVec3& angDelta0, const PxVec3& linDelta1, const PxVec3& angDelta1, const PxU32 threadIndex,
	const PxgTGSBlockSolverConstraint1DHeader* PX_RESTRICT headers, PxgTGSBlockSolverConstraint1DCon* PX_RESTRICT rowsCon,
	const PxgSolverTxIData& iData0, const PxgSolverTxIData& iData1, const PxReal elapsedTime, bool residualReportingEnabled,
	PxReal ref0 = 1.f, PxReal ref1 = 1.f)
{
	using namespace physx;

	//
	// please refer to solve1DStep() in DyTGSContactPrep.cpp for a description of the parameters and some of the logic
	//

	const PxgTGSBlockSolverConstraint1DHeader* PX_RESTRICT  header = &headers[batch.mConstraintBatchIndex];
	PxgTGSBlockSolverConstraint1DCon* PX_RESTRICT baseCon = &rowsCon[batch.startConstraintIndex];

	PxVec3 linVel0(b0LinVel.x, b0LinVel.y, b0LinVel.z);
	PxVec3 linVel1(b1LinVel.x, b1LinVel.y, b1LinVel.z);
	PxVec3 angVel0(b0AngVel.x, b0AngVel.y, b0AngVel.z);
	PxVec3 angVel1(b1AngVel.x, b1AngVel.y, b1AngVel.z);

	const float4 raInvMass0 = header->rAWorld_invMass0D0[threadIndex];
	const float4 rbInvMass1 = header->rBWorld_invMass1D1[threadIndex];

	float invMass0 = ref0 * raInvMass0.w; //FLoad(header->invMass0D0);
	float invMass1 = ref1 * rbInvMass1.w; //FLoad(header->invMass1D1);

	const PxVec3 raPrev(raInvMass0.x, raInvMass0.y, raInvMass0.z);
	const PxVec3 rbPrev(rbInvMass1.x, rbInvMass1.y, rbInvMass1.z);

	const PxVec3 ra = iData0.deltaBody2World.q.rotate(raPrev);
	const PxVec3 rb = iData1.deltaBody2World.q.rotate(rbPrev);

	const PxVec3 raMotion = (ra + linDelta0) - raPrev;
	const PxVec3 rbMotion = (rb + linDelta1) - rbPrev;

	float invInertiaScale0 = ref0 * header->invInertiaScale0[threadIndex];
	float invInertiaScale1 = ref1 * header->invInertiaScale1[threadIndex];


	const uchar4 rowCounts_breakable_orthoAxisCount = header->rowCounts_breakable_orthoAxisCount[threadIndex];

	const PxU32 rowCount = rowCounts_breakable_orthoAxisCount.x;

	const float4 ang0Ortho0_recipResponseW = header->angOrthoAxis0_recipResponseW[0][threadIndex];
	const float4 ang0Ortho1_recipResponseW = header->angOrthoAxis0_recipResponseW[1][threadIndex];
	const float4 ang0Ortho2_recipResponseW = header->angOrthoAxis0_recipResponseW[2][threadIndex];

	const float4 ang1OrthoAxis0_ErrorW = header->angOrthoAxis1_ErrorW[0][threadIndex];
	const float4 ang1OrthoAxis1_ErrorW = header->angOrthoAxis1_ErrorW[1][threadIndex];
	const float4 ang1OrthoAxis2_ErrorW = header->angOrthoAxis1_ErrorW[2][threadIndex];

	const float recipResponse0 = ang0Ortho0_recipResponseW.w;
	const float recipResponse1 = ang0Ortho1_recipResponseW.w;
	const float recipResponse2 = ang0Ortho2_recipResponseW.w;

	const PxVec3 ang0Ortho0(ang0Ortho0_recipResponseW.x, ang0Ortho0_recipResponseW.y, ang0Ortho0_recipResponseW.z);
	const PxVec3 ang0Ortho1(ang0Ortho1_recipResponseW.x, ang0Ortho1_recipResponseW.y, ang0Ortho1_recipResponseW.z);
	const PxVec3 ang0Ortho2(ang0Ortho2_recipResponseW.x, ang0Ortho2_recipResponseW.y, ang0Ortho2_recipResponseW.z);

	const PxVec3 ang1Ortho0(ang1OrthoAxis0_ErrorW.x, ang1OrthoAxis0_ErrorW.y, ang1OrthoAxis0_ErrorW.z);
	const PxVec3 ang1Ortho1(ang1OrthoAxis1_ErrorW.x, ang1OrthoAxis1_ErrorW.y, ang1OrthoAxis1_ErrorW.z);
	const PxVec3 ang1Ortho2(ang1OrthoAxis2_ErrorW.x, ang1OrthoAxis2_ErrorW.y, ang1OrthoAxis2_ErrorW.z);

	PxReal error0 = ang1OrthoAxis0_ErrorW.w + (ang0Ortho0.dot(angDelta0) - ang1Ortho0.dot(angDelta1));
	PxReal error1 = ang1OrthoAxis1_ErrorW.w + (ang0Ortho1.dot(angDelta0) - ang1Ortho1.dot(angDelta1));
	PxReal error2 = ang1OrthoAxis2_ErrorW.w + (ang0Ortho2.dot(angDelta0) - ang1Ortho2.dot(angDelta1));

	for (PxU32 i = 0; i < rowCount; ++i)
	{
		PxgTGSBlockSolverConstraint1DCon& ccon = baseCon[i];

		const float4 _clinVel0_coeff0W = ccon.lin0XYZ_initBiasOrCoeff0[threadIndex];
		const float4 _clinVel1_coeff1W = ccon.lin1XYZ_biasScaleOrCoeff1[threadIndex];
		const float4 _cangVel0_coeff2W = ccon.ang0XYZ_velMultiplierOrCoeff2[threadIndex];
		const float4 _cangVel1_coeff3W = ccon.ang1XYZ_velTargetOrCoeff3[threadIndex];

		const PxVec3 clinVel0(_clinVel0_coeff0W.x, _clinVel0_coeff0W.y, _clinVel0_coeff0W.z);
		const PxVec3 clinVel1(_clinVel1_coeff1W.x, _clinVel1_coeff1W.y, _clinVel1_coeff1W.z);
		const PxVec3 cangVel0_(_cangVel0_coeff2W.x, _cangVel0_coeff2W.y, _cangVel0_coeff2W.z);
		const PxVec3 cangVel1_(_cangVel1_coeff3W.x, _cangVel1_coeff3W.y, _cangVel1_coeff3W.z);

		PxReal initBias = _clinVel0_coeff0W.w;
		const PxReal biasScale = _clinVel1_coeff1W.w;
		const PxReal velMultiplier = _cangVel0_coeff2W.w;
		const PxReal targetVel = _cangVel1_coeff3W.w;

		const PxVec3 cangVel0 = cangVel0_ + ra.cross(clinVel0);
		const PxVec3 cangVel1 = cangVel1_ + rb.cross(clinVel1);

		const PxU32 flags = ccon.flags[threadIndex];

		const PxReal maxBias = ccon.maxBias[threadIndex];

		const PxReal minBias = Dy::computeMinBiasTGS(flags, maxBias);

		PxVec3 raXnI = iData0.sqrtInvInertia * cangVel0;
		PxVec3 rbXnI = iData1.sqrtInvInertia * cangVel1;

		//KS - TODO - orthogonalization here...
		if (flags & DY_SC_FLAG_ORTHO_TARGET)
		{
			const PxReal proj0 = (raXnI.dot(ang0Ortho0) +
				rbXnI.dot(ang1Ortho0)) * recipResponse0;

			const PxReal proj1 = (raXnI.dot(ang0Ortho1) +
				rbXnI.dot(ang1Ortho1)) * recipResponse1;

			const PxReal proj2 = (raXnI.dot(ang0Ortho2) +
				rbXnI.dot(ang1Ortho2)) * recipResponse2;

			const PxVec3 delta0 = ang0Ortho0 * proj0 + ang0Ortho1 * proj1 + ang0Ortho2 * proj2;
			const PxVec3 delta1 = ang1Ortho0 * proj0 + ang1Ortho1 * proj1 + ang1Ortho2 * proj2;

			raXnI = raXnI - delta0;
			rbXnI = rbXnI - delta1;

			const PxReal orthoBasisError = biasScale * (error0 * proj0 + error1 * proj1 + error2 * proj2);
			initBias = initBias - orthoBasisError;
		}

		// If biasScale is zero, initBias must also be zero.
		// biasScale is enforced to zero before the velocity itertaion (e.g., in conclude1DBlockTGS), but not initBias.
		// Thus, explicitly resetting it to zero here for velocity iteration.
		initBias = (biasScale == 0.f) ? 0.f : initBias;

		const bool isSpringConstraint = (flags & DY_SC_FLAG_SPRING);

		const PxReal errorDelta = Dy::computeResolvedGeometricErrorTGS(raMotion, rbMotion, clinVel0, clinVel1,
			angDelta0, angDelta1, raXnI, rbXnI,
			ccon.angularErrorScale[threadIndex],
			isSpringConstraint, targetVel, elapsedTime);

		const PxReal resp0 = invMass0 * clinVel0.dot(clinVel0) + raXnI.dot(raXnI) * invInertiaScale0;
		const PxReal resp1 = invMass1 * clinVel1.dot(clinVel1) + rbXnI.dot(rbXnI) * invInertiaScale1;

		//KS - may be a - required here...
		const PxReal response = resp0 + resp1;

		const PxReal recipResponse = response > 0.f ? 1.f / response : 0.f;

		const PxReal vMul = isSpringConstraint ? velMultiplier : recipResponse * velMultiplier;

		const PxReal unclampedBias = initBias + errorDelta * biasScale;

		const PxReal bias = PxClamp(unclampedBias, minBias, maxBias);

		const PxReal constant = isSpringConstraint ? bias + targetVel : recipResponse * (bias + targetVel);

		const float appliedForce = ccon.appliedForce[threadIndex];//FLoad(c.appliedForce);
																  //const FloatV targetVel = FLoad(c.targetVelocity);

		const float maxImpulse = ccon.maxImpulse[threadIndex];//FLoad(c.maxImpulse);
		const float minImpulse = ccon.minImpulse[threadIndex];//FLoad(c.minImpulse);

														//const PxVec3 v0 = linVel0.multiply(clinVel0) + angVel0.multiply(cangVel0);//V3MulAdd(linVel0, clinVel0, V3Mul(angVel0, cangVel0));
														//const PxVec3 v1 = linVel1.multiply(clinVel1) + angVel1.multiply(cangVel1);//V3MulAdd(linVel1, clinVel1, V3Mul(angVel1, cangVel1));

		const float v0 = linVel0.dot(clinVel0) + angVel0.dot(raXnI);//V3MulAdd(linVel0, clinVel0, V3Mul(angVel0, cangVel0));
		const float v1 = linVel1.dot(clinVel1) + angVel1.dot(rbXnI);//V3MulAdd(linVel1, clinVel1, V3Mul(angVel1, cangVel1));

		const float normalVel = v0 - v1;

		const float unclampedForce = appliedForce + (vMul * normalVel + constant);//FMulAdd(iMul, appliedForce, FMulAdd(vMul, normalVel, constant));
		const float clampedForce = fminf(maxImpulse, fmaxf(minImpulse, unclampedForce));//FMin(maxImpulse, (FMax(minImpulse, unclampedForce)));
		const float deltaF = clampedForce - appliedForce;//FSub(clampedForce, appliedForce);

		ccon.appliedForce[threadIndex] = clampedForce; // FStore(clampedForce);
		if(residualReportingEnabled)
			ccon.residual[threadIndex] = PxgErrorAccumulator::calculateResidual(deltaF, vMul);

		linVel0 = linVel0 + clinVel0 * (deltaF * invMass0);//V3ScaleAdd(clinVel0, FMul(deltaF, invMass0), linVel0);			
		linVel1 = linVel1 - clinVel1 * (deltaF * invMass1);//V3NegScaleSub(clinVel1, FMul(deltaF, invMass1), linVel1);
		angVel0 = angVel0 + raXnI * deltaF * invInertiaScale0;//V3ScaleAdd(cangVel0, deltaF, angVel0);
		angVel1 = angVel1 - rbXnI * deltaF * invInertiaScale1;//V3NegScaleSub(cangVel1, deltaF, angVel1);

	}


	b0LinVel = PxVec3(linVel0.x, linVel0.y, linVel0.z);
	b0AngVel = PxVec3(angVel0.x, angVel0.y, angVel0.z);
	b1LinVel = PxVec3(linVel1.x, linVel1.y, linVel1.z);
	b1AngVel = PxVec3(angVel1.x, angVel1.y, angVel1.z);
}

// Using the same logic as the previous implementation, but mass-splitting is additionally performed per sub-timestep.
// Required data is packed and stored differently from the previous implementation to split mass at each sub-timestep; 
// i.e., mass-related terms are computed at every sub-timestep.

static __device__ PX_FORCE_INLINE void solveExt1DBlockTGS(const PxgBlockConstraintBatch& batch,
	Cm::UnAlignedSpatialVector& vel0,
	Cm::UnAlignedSpatialVector& vel1,
	const Cm::UnAlignedSpatialVector& motion0,
	const Cm::UnAlignedSpatialVector& motion1,
	const PxU32 threadIndex,
	const PxgTGSBlockSolverConstraint1DHeader* PX_RESTRICT headers,
	PxgTGSBlockSolverConstraint1DCon* PX_RESTRICT rowsCon,
	PxgArticulationBlockResponse* PX_RESTRICT artiResponse,
	const PxQuat& deltaQ0, const PxQuat& deltaQ1,
	const PxReal elapsedTime,
	Cm::UnAlignedSpatialVector& impluse0,
	Cm::UnAlignedSpatialVector& impluse1,
	bool residualReportingEnabled, 
	PxReal ref0 = 1.f, PxReal ref1 = 1.f)
{
	using namespace physx;

	const PxgTGSBlockSolverConstraint1DHeader* PX_RESTRICT  header = &headers[batch.mConstraintBatchIndex];
	PxgTGSBlockSolverConstraint1DCon* PX_RESTRICT baseCon = &rowsCon[batch.startConstraintIndex];
	const PxgArticulationBlockResponse* PX_RESTRICT responses = &artiResponse[batch.mArticulationResponseIndex];

	PxVec3 linVel0 = vel0.bottom;
	PxVec3 linVel1 = vel1.bottom;
	PxVec3 angVel0 = vel0.top;
	PxVec3 angVel1 = vel1.top;

	const float4 raInvMass0 = header->rAWorld_invMass0D0[threadIndex];
	const float4 rbInvMass1 = header->rBWorld_invMass1D1[threadIndex];

	float invMass0 = ref0 * raInvMass0.w; //FLoad(header->invMass0D0);
	float invMass1 = ref1 * rbInvMass1.w; //FLoad(header->invMass1D1);

	float invInertiaScale0 = ref0 * header->invInertiaScale0[threadIndex];
	float invInertiaScale1 = ref1 * header->invInertiaScale1[threadIndex];

	const PxVec3 raPrev(raInvMass0.x, raInvMass0.y, raInvMass0.z);
	const PxVec3 rbPrev(rbInvMass1.x, rbInvMass1.y, rbInvMass1.z);

	const PxVec3 ra = deltaQ0.rotate(raPrev);
	const PxVec3 rb = deltaQ1.rotate(rbPrev);

	const PxVec3 raMotion = (ra + motion0.bottom) - raPrev;
	const PxVec3 rbMotion = (rb + motion1.bottom) - rbPrev;

	const uchar4 rowCounts_breakable_orthoAxisCount = header->rowCounts_breakable_orthoAxisCount[threadIndex];

	const PxU32 rowCount = rowCounts_breakable_orthoAxisCount.x;

	const PxReal cfm = header->cfm[threadIndex];

	PxVec3 li0(0.f, 0.f, 0.f);
	PxVec3 li1(0.f, 0.f, 0.f);
	PxVec3 ai0(0.f, 0.f, 0.f);
	PxVec3 ai1(0.f, 0.f, 0.f);

	for (PxU32 i = 0; i < rowCount; ++i)
	{
		PxgTGSBlockSolverConstraint1DCon& ccon = baseCon[i];
		const PxgArticulationBlockResponse& response = responses[i];

		const float4 _clinVel0_coeff0W = ccon.lin0XYZ_initBiasOrCoeff0[threadIndex];//V3LoadA(c.lin0);
		const float4 _clinVel1_coeff1W = ccon.lin1XYZ_biasScaleOrCoeff1[threadIndex];//V3LoadA(c.lin1);
		const float4 _cangVel0_coeff2W = ccon.ang0XYZ_velMultiplierOrCoeff2[threadIndex];//V3LoadA(c.ang0);
		const float4 _cangVel1_coeff3W = ccon.ang1XYZ_velTargetOrCoeff3[threadIndex];//V3LoadA(c.ang1);

		const PxVec3 clinVel0(_clinVel0_coeff0W.x, _clinVel0_coeff0W.y, _clinVel0_coeff0W.z);
		const PxVec3 clinVel1(_clinVel1_coeff1W.x, _clinVel1_coeff1W.y, _clinVel1_coeff1W.z);
		const PxVec3 cangVel0(_cangVel0_coeff2W.x, _cangVel0_coeff2W.y, _cangVel0_coeff2W.z);
		const PxVec3 cangVel1(_cangVel1_coeff3W.x, _cangVel1_coeff3W.y, _cangVel1_coeff3W.z);

		const PxU32 flags = ccon.flags[threadIndex];
		const bool isSpringConstraint = (flags & DY_SC_FLAG_SPRING);

		const PxReal resp0 = ref0 * ccon.resp0[threadIndex];
		const PxReal resp1 = ref1 * ccon.resp1[threadIndex];
		
		float unitResponse = resp0 + resp1;
		unitResponse += cfm;

		//https://omniverse-jirasw.nvidia.com/browse/PX-4383
		const PxReal minRowResponse = DY_ARTICULATION_MIN_RESPONSE;
		const PxReal recipResponse = Dy::computeRecipUnitResponse(unitResponse, minRowResponse);

		const bool isAccelerationSpring = (flags & DY_SC_FLAG_ACCELERATION_SPRING);

		const PxReal coeff0 = _clinVel0_coeff0W.w;
		const PxReal coeff1 = _clinVel1_coeff1W.w;
		const PxReal coeff2 = _cangVel0_coeff2W.w;
		const PxReal coeff3 = _cangVel1_coeff3W.w;
		const PxReal geometricError = ccon.geometricError[threadIndex];

		PxReal biasScale, velMultiplier, initBias, targetVel;
		Dy::compute1dConstraintSolverConstantsTGS(isSpringConstraint, isAccelerationSpring, geometricError,
			unitResponse, recipResponse, coeff0, coeff1, coeff2,
			coeff3, biasScale, velMultiplier, initBias, targetVel);

		// If biasScale is zero, initBias must also be zero.
		// biasScale is enforced to zero before the velocity itertaion (e.g., in conclude1DBlockTGS), but not initBias.
		// Thus, explicitly resetting it to zero here for velocity iteration.
		initBias = (biasScale == 0.f) ? 0.f : initBias;

		const PxReal errorDelta = Dy::computeResolvedGeometricErrorTGS(raMotion, rbMotion, clinVel0, clinVel1,
			motion0.top, motion1.top, cangVel0, cangVel1,
			ccon.angularErrorScale[threadIndex],
			isSpringConstraint, targetVel, elapsedTime);


		const PxReal maxBias = ccon.maxBias[threadIndex];

		const PxReal vMul = isSpringConstraint ? velMultiplier : recipResponse * velMultiplier;
		const float appliedForce = ccon.appliedForce[threadIndex];

		const PxReal unclampedBias = initBias + errorDelta * biasScale;
		const PxReal minBias = Dy::computeMinBiasTGS(flags, maxBias);
		const PxReal bias = PxClamp(unclampedBias, minBias, maxBias);

		const PxReal constant = isSpringConstraint ? (bias + targetVel) : recipResponse * (bias + targetVel);

		const float maxImpulse = ccon.maxImpulse[threadIndex];//FLoad(c.maxImpulse);
		const float minImpulse = ccon.minImpulse[threadIndex];//FLoad(c.minImpulse);

		const float v0 = linVel0.dot(clinVel0) + angVel0.dot(cangVel0);//V3MulAdd(linVel0, clinVel0, V3Mul(angVel0, cangVel0));
		const float v1 = linVel1.dot(clinVel1) + angVel1.dot(cangVel1);//V3MulAdd(linVel1, clinVel1, V3Mul(angVel1, cangVel1));

		const float normalVel = v0 - v1;

		const float unclampedForce = appliedForce + (vMul * normalVel + constant);//FMulAdd(iMul, appliedForce, FMulAdd(vMul, normalVel, constant));
		const float clampedForce = fminf(maxImpulse, fmaxf(minImpulse, unclampedForce));//FMin(maxImpulse, (FMax(minImpulse, unclampedForce)));
		const float deltaF = clampedForce - appliedForce;//FSub(clampedForce, appliedForce);

		ccon.appliedForce[threadIndex] = clampedForce; // FStore(clampedForce);
		if(residualReportingEnabled)
			ccon.residual[threadIndex] = PxgErrorAccumulator::calculateResidual(deltaF, vMul);

		li0 = clinVel0 * deltaF + li0;
		ai0 = cangVel0 * deltaF + ai0;
		li1 = clinVel1 * deltaF + li1;
		ai1 = cangVel1 * deltaF + ai1;

		PxVec3 linVa = ref0 * PxVec3(response.deltaRALin_x[threadIndex], response.deltaRALin_y[threadIndex], response.deltaRALin_z[threadIndex]);
		PxVec3 angVa = ref0 * PxVec3(response.deltaRAAng_x[threadIndex], response.deltaRAAng_y[threadIndex], response.deltaRAAng_z[threadIndex]);
		PxVec3 linVb = ref1 * PxVec3(response.deltaRBLin_x[threadIndex], response.deltaRBLin_y[threadIndex], response.deltaRBLin_z[threadIndex]);
		PxVec3 angVb = ref1 * PxVec3(response.deltaRBAng_x[threadIndex], response.deltaRBAng_y[threadIndex], response.deltaRBAng_z[threadIndex]);

		linVel0 = linVa * deltaF + linVel0;
		angVel0 = angVa * deltaF + angVel0;

		linVel1 = linVb * deltaF + linVel1;
		angVel1 = angVb * deltaF + angVel1;
	}

	vel0.top = angVel0; vel0.bottom = linVel0;
	vel1.top = angVel1; vel1.bottom = linVel1;

	impluse0.top = li0 * invMass0; impluse0.bottom = ai0 * invInertiaScale0;
	impluse1.top = li1 * invMass1; impluse1.bottom = ai1 * invInertiaScale1;
}

static __device__ void conclude1DBlockTGS(const PxgBlockConstraintBatch& batch, const PxU32 threadIndex, const PxgTGSBlockSolverConstraint1DHeader* PX_RESTRICT headers, PxgTGSBlockSolverConstraint1DCon* PX_RESTRICT rows)
{
	using namespace physx;

	const PxgTGSBlockSolverConstraint1DHeader* PX_RESTRICT  header = &headers[batch.mConstraintBatchIndex];
	PxgTGSBlockSolverConstraint1DCon* PX_RESTRICT base = &rows[batch.startConstraintIndex];

	const uchar4 rowCount = header->rowCounts_breakable_orthoAxisCount[threadIndex];

	for (PxU32 i = 0; i<rowCount.x; i++)
	{
		PxgTGSBlockSolverConstraint1DCon& c = base[i];

		// Previous data, biasScale, initBias, velTarget, and velMultiplier are repacked using other coefficients.
		// See "PxgTGSBlockSolverConstraint1DCon" for details.

		if(!(c.flags[threadIndex] & DY_SC_FLAG_KEEP_BIAS))
		{
			c.lin1XYZ_biasScaleOrCoeff1[threadIndex].w = 0.f; // setting biasScale to zero. Make sure to enforce
															  // initialBias to be zero, when biasScale is zero.
		}
		if(c.flags[threadIndex] & DY_SC_FLAG_SPRING)
		{
			// see CPU version for an explanation
			c.lin0XYZ_initBiasOrCoeff0[threadIndex].w = 0.f;
			c.lin1XYZ_biasScaleOrCoeff1[threadIndex].w = 0.f;

			c.ang0XYZ_velMultiplierOrCoeff2[threadIndex].w = 0.0f;
			c.ang1XYZ_velTargetOrCoeff3[threadIndex].w = 0.0f;
		}
	}
}


static __device__ void writeBack1DBlockTGS(const PxgBlockConstraintBatch& batch, const PxU32 threadIndex, const PxgTGSBlockSolverConstraint1DHeader* PX_RESTRICT headers,
	PxgTGSBlockSolverConstraint1DCon* PX_RESTRICT rowsCon, PxgConstraintWriteback* constraintWriteBacks)
{
	const PxgTGSBlockSolverConstraint1DHeader* PX_RESTRICT  header = &headers[batch.mConstraintBatchIndex];
	PxgTGSBlockSolverConstraint1DCon* conBase = &rowsCon[batch.startConstraintIndex];

	PxU32 forceWritebackOffset = header->writeBackOffset[threadIndex];

	const uchar4 rowCounts_breakable_orthoAxisCount = header->rowCounts_breakable_orthoAxisCount[threadIndex];

	const PxU8 breakable = rowCounts_breakable_orthoAxisCount.y;

	const PxU32 numRows = rowCounts_breakable_orthoAxisCount.x;

	if (forceWritebackOffset != 0xFFFFFFFF)
	{
		PxgConstraintWriteback& writeback = constraintWriteBacks[forceWritebackOffset];

		PxVec3 linVel(0), angVel(0);
		PxReal constraintErrorSq = 0.0f;
		for (PxU32 i = 0; i < numRows; ++i)
		{
			PxgTGSBlockSolverConstraint1DCon& con = conBase[i];
		
			if (con.flags[threadIndex] & DY_SC_FLAG_OUTPUT_FORCE)
			{
				const float4 lin0XYZ_ErrorW = con.lin0XYZ_initBiasOrCoeff0[threadIndex];
				const float4 ang0WriteBack_VMW = con.ang0XYZ_velMultiplierOrCoeff2[threadIndex];

				const PxVec3 lin0(lin0XYZ_ErrorW.x, lin0XYZ_ErrorW.y, lin0XYZ_ErrorW.z);
				const PxVec3 ang0WriteBack(ang0WriteBack_VMW.x, ang0WriteBack_VMW.y, ang0WriteBack_VMW.z);
				const PxReal appliedForce = con.appliedForce[threadIndex];
				linVel += lin0 * appliedForce;
				angVel += ang0WriteBack *appliedForce;
			}

			PxReal err = con.residual[threadIndex];
			constraintErrorSq += err * err;
		}

		const float4 body0WorldOffset_InvMass0D0 = header->rAWorld_invMass0D0[threadIndex];
		const PxVec3 body0WorldOffset(body0WorldOffset_InvMass0D0.x, body0WorldOffset_InvMass0D0.y, body0WorldOffset_InvMass0D0.z);
		angVel -= body0WorldOffset.cross(linVel);


		const PxU32 broken = breakable ? PxU32((linVel.magnitude() > header->linBreakImpulse[threadIndex]) || (angVel.magnitude() > header->angBreakImpulse[threadIndex])) : 0;
		writeback.angularImpulse_residual = make_float4(angVel.x, angVel.y, angVel.z, constraintErrorSq);
		writeback.linearImpulse_broken = make_float4(linVel.x, linVel.y, linVel.z, broken ? -0.0f : 0.0f);
	}

}




#endif
