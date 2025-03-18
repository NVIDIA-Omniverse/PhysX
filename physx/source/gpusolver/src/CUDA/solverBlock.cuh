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

#ifndef __SOLVER_BLOCK_CUH__
#define __SOLVER_BLOCK_CUH__

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
#include "PxgIntrinsics.h"
#include "stdio.h"
#include "assert.h"
#include "solverResidual.cuh"
#include "constraintPrepShared.cuh"

#include "solverBlockCommon.cuh"
#include "PxgDynamicsConfiguration.h"
#include "DyCpuGpu1dConstraint.h"

using namespace physx;

PX_FORCE_INLINE static __device__ uint32_t nextPowerOfTwo(uint32_t x)
{
	x |= (x >> 1);
	x |= (x >> 2);
	x |= (x >> 4);
	x |= (x >> 8);
	x |= (x >> 16);
	return x + 1;
}

PX_FORCE_INLINE static __device__ bool isPowerOfTwo(uint32_t x)
{
	return (x & (x - 1)) == 0;
}

// TGS uses three float4s to store linVel, angVel, linDelta, angDelta, while PGS uses 2 float4s for linVel
// and angVel. Default: PGS
PX_FORCE_INLINE static __device__ PxU32 ComputeAverageBodyBatchStartIndex(const PxU32 bodyIndex, const PxU32 float4sPerBody = 2)
{
	return float4sPerBody * (bodyIndex & (~31)) + (bodyIndex & 31);
}

PX_FORCE_INLINE static __device__ PxU32 countActiveSlabs(PxU32 index, PxU32 numSlabs, PxU32 numSolverBodies,
														 const PxU32* const encodedReferenceCounts)
{
	PxU32 referenceCount = 0;

	const PxU32 num32Slabs = (numSlabs + 31) / 32; // In case more than 32 slabs are used.
	for (PxU32 i = 0; i < num32Slabs; ++i)
	{
		const PxU32 id = encodedReferenceCounts[index + i * numSolverBodies];
		referenceCount += static_cast<PxU32>(__popc(id));
	}

	return PxMax(1u, referenceCount);
}

PX_FORCE_INLINE static __device__ void resetSlabCount(PxU32 index, PxU32 numSlabs, PxU32 numSolverBodies,
													  PxU32* PX_RESTRICT encodedReferenceCounts)
{
	const PxU32 num32Slabs = (numSlabs + 31) / 32; // In case more than 32 slabs are used.
	for(PxU32 i = 0; i < num32Slabs; ++i)
	{
		encodedReferenceCounts[index + i * numSolverBodies] = 0u;
	}
}

// Mass-splitting version of 1D constraints; mass-related terms are computed at every sub-timestep. See "setupSolverConstraintBlockGPU".
// Refer to "Mass Splitting for Jitter-Free Parallel Rigid Body Simulation" for the general mass-splitting concept.
static __device__ void solve1DBlock(const PxgBlockConstraintBatch& batch, PxVec3& b0LinVel, PxVec3& b0AngVel, PxVec3& b1LinVel, PxVec3& b1AngVel, const PxU32 threadIndex,
	const PxgBlockSolverConstraint1DHeader* PX_RESTRICT headers, PxgBlockSolverConstraint1DCon* PX_RESTRICT rowsCon,
	PxgBlockSolverConstraint1DMod* PX_RESTRICT rowsMod, bool residualReportingEnabled,
	PxReal ref0 = 1.f, PxReal ref1 = 1.f)
{
	using namespace physx;

	const PxgBlockSolverConstraint1DHeader* PX_RESTRICT  header = &headers[batch.mConstraintBatchIndex];
	PxgBlockSolverConstraint1DCon* PX_RESTRICT baseCon = &rowsCon[batch.startConstraintIndex];
	PxgBlockSolverConstraint1DMod* PX_RESTRICT baseMod = &rowsMod[batch.startConstraintIndex];

	PxVec3 linVel0 = b0LinVel;
	PxVec3 linVel1 = b1LinVel;
	PxVec3 angVel0 = b0AngVel;
	PxVec3 angVel1 = b1AngVel;

	float invMass0 = ref0 * header->invMass0D0[threadIndex];
	float invMass1 = ref1 * header->invMass1D1[threadIndex];

	float invInertiaScale0 = ref0 * header->invInertiaScale0[threadIndex];
	float invInertiaScale1 = ref1 * header->invInertiaScale1[threadIndex];

	for (PxU32 i = 0; i < header->rowCounts[threadIndex]; ++i)
	{
		PxgBlockSolverConstraint1DCon& ccon = baseCon[i];
		PxgBlockSolverConstraint1DMod& cmod = baseMod[i];

		const float4 _clinVel0_minImpulse = ccon.lin0XYZ_minImpulse[threadIndex];
		const float4 _clinVel1_maxImpulse = ccon.lin1XYZ_maxImpulse[threadIndex];
		const float4 _cangVel0_resp0 = ccon.ang0XYZ_resp0[threadIndex];
		const float4 _cangVel1_resp1 = ccon.ang1XYZ_resp1[threadIndex];

		const PxVec3 clinVel0(_clinVel0_minImpulse.x, _clinVel0_minImpulse.y, _clinVel0_minImpulse.z);
		const PxVec3 clinVel1(_clinVel1_maxImpulse.x, _clinVel1_maxImpulse.y, _clinVel1_maxImpulse.z);
		const PxVec3 cangVel0(_cangVel0_resp0.x, _cangVel0_resp0.y, _cangVel0_resp0.z);
		const PxVec3 cangVel1(_cangVel1_resp1.x, _cangVel1_resp1.y, _cangVel1_resp1.z);

		const PxReal resp0 = ref0 * _cangVel0_resp0.w;
		const PxReal resp1 = ref1 * _cangVel1_resp1.w;
		const PxReal initJointSpeed = ccon.initJointSpeed[threadIndex];

		const PxReal coeff0 = cmod.coeff0[threadIndex];
		const PxReal coeff1 = cmod.coeff1[threadIndex];
		const PxU32 flags = cmod.flags[threadIndex];

		const PxReal unitResponse = resp0 + resp1;

		//https://omniverse-jirasw.nvidia.com/browse/PX-4383
		const PxReal minRowResponse = DY_MIN_RESPONSE;
		const PxReal recipResponse = Dy::computeRecipUnitResponse(unitResponse, minRowResponse);

		PxReal constant, unbiasedConstant, vMul, iMul;

		bool isSpring = flags & DY_SC_FLAG_SPRING;
		bool isAccelerationSpring = flags & DY_SC_FLAG_ACCELERATION_SPRING;

		Dy::compute1dConstraintSolverConstantsPGS(isSpring, isAccelerationSpring, coeff0, coeff1, initJointSpeed, unitResponse,
												  recipResponse, constant, unbiasedConstant, vMul, iMul);

		// For velocity iterations, "constant" is overwritten by "unbiasedConstant".
		// This is currently done by assigning coeff1 to coeff0 in "conclude1DBlock".

		const float appliedForce = cmod.appliedForce[threadIndex];//FLoad(c.appliedForce);

		const float maxImpulse = _clinVel1_maxImpulse.w;//FLoad(c.maxImpulse);
		const float minImpulse = _clinVel0_minImpulse.w;//FLoad(c.minImpulse);

		const float v0 = linVel0.dot(clinVel0) + angVel0.dot(cangVel0);//V3MulAdd(linVel0, clinVel0, V3Mul(angVel0, cangVel0));
		const float v1 = linVel1.dot(clinVel1) + angVel1.dot(cangVel1);//V3MulAdd(linVel1, clinVel1, V3Mul(angVel1, cangVel1));

		const float normalVel = v0 - v1;

		const float unclampedForce = iMul*appliedForce + (vMul*normalVel + constant);//FMulAdd(iMul, appliedForce, FMulAdd(vMul, normalVel, constant));
		const float clampedForce = fminf(maxImpulse, fmaxf(minImpulse, unclampedForce));//FMin(maxImpulse, (FMax(minImpulse, unclampedForce)));
		const float deltaF = clampedForce - appliedForce;//FSub(clampedForce, appliedForce);

		cmod.appliedForce[threadIndex] = clampedForce;
		if(residualReportingEnabled)
			cmod.residual[threadIndex] = PxgErrorAccumulator::calculateResidual(deltaF, vMul);

		linVel0 = linVel0 + clinVel0*(deltaF*invMass0);//V3ScaleAdd(clinVel0, FMul(deltaF, invMass0), linVel0);			
		linVel1 = linVel1 - clinVel1*(deltaF*invMass1);//V3NegScaleSub(clinVel1, FMul(deltaF, invMass1), linVel1);
		angVel0 = angVel0 + cangVel0*deltaF*invInertiaScale0;//V3ScaleAdd(cangVel0, deltaF, angVel0);
		angVel1 = angVel1 - cangVel1*deltaF*invInertiaScale1;//V3NegScaleSub(cangVel1, deltaF, angVel1);

	}


	b0LinVel = linVel0;
	b0AngVel = angVel0;
	b1LinVel = linVel1;
	b1AngVel = angVel1;
	
}

// Mass-splitting version of 1D constraints; mass-related terms are computed at every sub-timestep. See "setupArtiSolverConstraintBlockGPU".
// Refer to "Mass Splitting for Jitter-Free Parallel Rigid Body Simulation" for the general mass-splitting concept.
static __device__ void solveExt1DBlock(const PxgBlockConstraintBatch& batch,
	Cm::UnAlignedSpatialVector& vel0,
	Cm::UnAlignedSpatialVector& vel1,
	const PxU32 threadIndex,
	const PxgBlockSolverConstraint1DHeader* PX_RESTRICT headers,
	PxgBlockSolverConstraint1DCon* PX_RESTRICT rowsCon,
	PxgBlockSolverConstraint1DMod* PX_RESTRICT rowsMod,
	PxgArticulationBlockResponse* PX_RESTRICT artiResponse,
	Cm::UnAlignedSpatialVector& impluse0,
	Cm::UnAlignedSpatialVector& impluse1,
	bool residualReportingEnabled,
	PxReal ref0 = 1.f, PxReal ref1 = 1.f)
{
	using namespace physx;

	const PxgBlockSolverConstraint1DHeader* PX_RESTRICT  header = &headers[batch.mConstraintBatchIndex];
	PxgBlockSolverConstraint1DCon* PX_RESTRICT baseCon = &rowsCon[batch.startConstraintIndex];
	PxgBlockSolverConstraint1DMod* PX_RESTRICT baseMod = &rowsMod[batch.startConstraintIndex];

	PxVec3 linVel0 = vel0.bottom;
	PxVec3 linVel1 = vel1.bottom;
	PxVec3 angVel0 = vel0.top;
	PxVec3 angVel1 = vel1.top;

	PxVec3 li0 = impluse0.bottom;
	PxVec3 li1 = impluse1.bottom;
	PxVec3 ai0 = impluse0.top;
	PxVec3 ai1 = impluse1.top;

	float invMass0 = ref0 * header->invMass0D0[threadIndex];
	float invMass1 = ref1 * header->invMass1D1[threadIndex];

	float invInertiaScale0 = ref0 * header->invInertiaScale0[threadIndex];
	float invInertiaScale1 = ref1 * header->invInertiaScale1[threadIndex];

	const float cfm = header->cfm[threadIndex];

	const PxU32 numRows = header->rowCounts[threadIndex];
	for (PxU32 i = 0; i < numRows; ++i)
	{
		PxgBlockSolverConstraint1DCon& ccon = baseCon[i];
		PxgBlockSolverConstraint1DMod& cmod = baseMod[i];
		PxgArticulationBlockResponse& response = artiResponse[i];

		const float4 _clinVel0_minImpulse = ccon.lin0XYZ_minImpulse[threadIndex];
		const float4 _clinVel1_maxImpulse = ccon.lin1XYZ_maxImpulse[threadIndex];
		const float4 _cangVel0_resp0 = ccon.ang0XYZ_resp0[threadIndex];
		const float4 _cangVel1_resp1 = ccon.ang1XYZ_resp1[threadIndex];

		const PxReal resp0 = ref0 * _cangVel0_resp0.w;
		const PxReal resp1 = ref1 * _cangVel1_resp1.w;
		const PxReal initJointSpeed = ccon.initJointSpeed[threadIndex];

		const PxVec3 clinVel0(_clinVel0_minImpulse.x, _clinVel0_minImpulse.y, _clinVel0_minImpulse.z);
		const PxVec3 clinVel1(_clinVel1_maxImpulse.x, _clinVel1_maxImpulse.y, _clinVel1_maxImpulse.z);
		const PxVec3 cangVel0(_cangVel0_resp0.x, _cangVel0_resp0.y, _cangVel0_resp0.z);
		const PxVec3 cangVel1(_cangVel1_resp1.x, _cangVel1_resp1.y, _cangVel1_resp1.z);

		const PxReal coeff0 = cmod.coeff0[threadIndex];
		const PxReal coeff1 = cmod.coeff1[threadIndex];
		const PxU32 flags = cmod.flags[threadIndex];

		const PxReal unitResponse = resp0 + resp1 + cfm;

		//https://omniverse-jirasw.nvidia.com/browse/PX-4383
		const PxReal minRowResponse = DY_MIN_RESPONSE;
		const PxReal recipResponse = Dy::computeRecipUnitResponse(unitResponse, minRowResponse);

		PxReal constant, unbiasedConstant, vMul, iMul;

		bool isSpring = flags & DY_SC_FLAG_SPRING;
		bool isAccelerationSpring = flags & DY_SC_FLAG_ACCELERATION_SPRING;

		compute1dConstraintSolverConstantsPGS(isSpring, isAccelerationSpring, coeff0, coeff1, initJointSpeed, unitResponse, recipResponse,
											  constant, unbiasedConstant, vMul, iMul);

		// For velocity iterations, "constant" is overwritten by "unbiasedConstant".
		// This is currently done by assigning coeff1 to coeff0 in "conclude1DBlock".

		const float appliedForce = cmod.appliedForce[threadIndex];//FLoad(c.appliedForce);
		const float maxImpulse = _clinVel1_maxImpulse.w;//FLoad(c.maxImpulse);
		const float minImpulse = _clinVel0_minImpulse.w;//FLoad(c.minImpulse);

		const float v0 = linVel0.dot(clinVel0) + angVel0.dot(cangVel0);//V3MulAdd(linVel0, clinVel0, V3Mul(angVel0, cangVel0));
		const float v1 = linVel1.dot(clinVel1) + angVel1.dot(cangVel1);//V3MulAdd(linVel1, clinVel1, V3Mul(angVel1, cangVel1));

		const float normalVel = v0 - v1;

		const float unclampedForce = iMul * appliedForce + (vMul * normalVel + constant);//FMulAdd(iMul, appliedForce, FMulAdd(vMul, normalVel, constant));
		const float clampedForce = fminf(maxImpulse, fmaxf(minImpulse, unclampedForce));//FMin(maxImpulse, (FMax(minImpulse, unclampedForce)));
		const float deltaF = clampedForce - appliedForce;//FSub(clampedForce, appliedForce);

		cmod.appliedForce[threadIndex] = clampedForce;
		if(residualReportingEnabled)
			cmod.residual[threadIndex] = PxgErrorAccumulator::calculateResidual(deltaF, vMul);

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

static __device__ void conclude1DBlock(const PxgBlockConstraintBatch& batch, const PxU32 threadIndex, const PxgBlockSolverConstraint1DHeader* PX_RESTRICT headers, PxgBlockSolverConstraint1DMod* PX_RESTRICT rowsMod)
{
	using namespace physx;
	const PxgBlockSolverConstraint1DHeader* PX_RESTRICT  header = &headers[batch.mConstraintBatchIndex];
	PxgBlockSolverConstraint1DMod* PX_RESTRICT base = &rowsMod[batch.startConstraintIndex];

	for (PxU32 i = 0; i < header->rowCounts[threadIndex]; i++)
	{
		PxgBlockSolverConstraint1DMod& c = base[i];
		if(!(c.flags[threadIndex] & DY_SC_FLAG_SPRING)) // For spring constraints, it is automatically satisfied.
		{
			c.coeff0[threadIndex] = c.coeff1[threadIndex]; // This makes sure "unbiased constant" is used as "constant".
														   // See also "queryReduced1dConstraintSolverConstantsPGS".
		}
	}
}

// Mass-splitting version of contact constraints; mass-related terms are computed at every sub-timestep. See "setupFinalizeSolverConstraintsBlock".
// Refer to "Mass Splitting for Jitter-Free Parallel Rigid Body Simulation" for the general mass-splitting concept.
static __device__ void solveContactBlock(const PxgBlockConstraintBatch& batch, PxVec3& b0LinVel, PxVec3& b0AngVel, PxVec3& b1LinVel, PxVec3& b1AngVel, bool doFriction, const PxU32 threadIndex,
	PxgBlockSolverContactHeader* contactHeaders, PxgBlockSolverFrictionHeader* frictionHeaders, PxgBlockSolverContactPoint* contactPoints, PxgBlockSolverContactFriction* frictionPoints,
	PxgErrorAccumulator* error, PxReal ref0 = 1.f, PxReal ref1 = 1.f)
{
	using namespace physx;

	PxVec3 linVel0 = b0LinVel;
	PxVec3 linVel1 = b1LinVel;
	PxVec3 angVel0 = b0AngVel;
	PxVec3 angVel1 = b1AngVel;

	{
		//printf("Normal batchIndex = %i, startConstraint = %i, startFriction = %i\n", batch.mConstraintBatchIndex, batch.startConstraintIndex, batch.startFrictionIndex);
		PxgBlockSolverContactHeader* PX_RESTRICT contactHeader = &contactHeaders[batch.mConstraintBatchIndex];
		PxgBlockSolverFrictionHeader* PX_RESTRICT frictionHeader = &frictionHeaders[batch.mConstraintBatchIndex];

		const uint numNormalConstr = Pxldcg(contactHeader->numNormalConstr[threadIndex]);
		const uint	numFrictionConstr = Pxldcg(frictionHeader->numFrictionConstr[threadIndex]);

		PxgBlockSolverContactPoint* PX_RESTRICT contacts = &contactPoints[batch.startConstraintIndex];
		PxgBlockSolverContactFriction* PX_RESTRICT frictions = &frictionPoints[batch.startFrictionIndex];

		float accumulatedNormalImpulse = 0.f;

		const float4 invMass0_1_angDom0_1 = Pxldcg(contactHeader->invMass0_1_angDom0_1[threadIndex]);

		const float invMassA = ref0 * invMass0_1_angDom0_1.x;
		const float invMassB = ref1 * invMass0_1_angDom0_1.y;

		const float angDom0 = ref0 * invMass0_1_angDom0_1.z;
		const float angDom1 = ref1 * invMass0_1_angDom0_1.w;

		const float4 normal_staticFriction = Pxldcg(contactHeader->normal_staticFriction[threadIndex]);

		const PxVec3 normal = PxVec3(normal_staticFriction.x, normal_staticFriction.y, normal_staticFriction.z);

		const float restitution = contactHeader->restitution[threadIndex];
		const float p8 = 0.8f;
		const PxU8 flags = contactHeader->flags[threadIndex];

		const PxVec3 delLinVel0 = normal * invMassA;
		const PxVec3 delLinVel1 = normal * invMassB;

		//Bring forward a read event
		const float staticFrictionCof = normal_staticFriction.w;

		float4 nextRaxn_extraCoeff;
		float4 nextRbxn_maxImpulseW;
		float nextAppliedForce;

		float nextResp0;
		float nextResp1;

		{
			nextRaxn_extraCoeff = Pxldcg(contacts[0].raXn_targetVelocity[threadIndex]);
			nextRbxn_maxImpulseW = Pxldcg(contacts[0].rbXn_maxImpulse[threadIndex]);
			nextAppliedForce = Pxldcg(contacts[0].appliedForce[threadIndex]);

			nextResp0 = Pxldcg(contacts[0].resp0[threadIndex]);
			nextResp1 = Pxldcg(contacts[0].resp1[threadIndex]);

			float nextCoeff0 = Pxldcg(contacts[0].coeff0[threadIndex]);
			float nextCoeff1 = Pxldcg(contacts[0].coeff1[threadIndex]);

			for (uint i = 0; i < numNormalConstr; i++)
			{
				PxgBlockSolverContactPoint& c = contacts[i];

				const float4 raXn_extraCoeff = nextRaxn_extraCoeff;
				const float4 rbXn_maxImpulse = nextRbxn_maxImpulseW;
				const float appliedForce = nextAppliedForce;

				const float resp0 = nextResp0;
				const float resp1 = nextResp1;

				const float coeff0 = nextCoeff0;
				const float coeff1 = nextCoeff1;

				if ((i + 1) < numNormalConstr)
				{
					const PxgBlockSolverContactPoint& nextC = contacts[i + 1];
					nextRaxn_extraCoeff = Pxldcg(nextC.raXn_targetVelocity[threadIndex]);
					nextRbxn_maxImpulseW = Pxldcg(nextC.rbXn_maxImpulse[threadIndex]);
					nextAppliedForce = Pxldcg(nextC.appliedForce[threadIndex]);

					nextResp0 = Pxldcg(nextC.resp0[threadIndex]);
					nextResp1 = Pxldcg(nextC.resp1[threadIndex]);

					nextCoeff0 = Pxldcg(nextC.coeff0[threadIndex]);
					nextCoeff1 = Pxldcg(nextC.coeff1[threadIndex]);
				}
				else if (numFrictionConstr && doFriction)
				{
					nextRaxn_extraCoeff = Pxldcg(frictions[0].raXn_bias[threadIndex]);
					nextRbxn_maxImpulseW = Pxldcg(frictions[0].rbXn_targetVelW[threadIndex]);
					nextAppliedForce = Pxldcg(frictions[0].appliedForce[threadIndex]);

					nextResp0 = Pxldcg(frictions[0].resp0[threadIndex]);
					nextResp1 = Pxldcg(frictions[0].resp1[threadIndex]);
				}

				const PxVec3 raXn = PxVec3(raXn_extraCoeff.x, raXn_extraCoeff.y, raXn_extraCoeff.z);
				const PxVec3 rbXn = PxVec3(rbXn_maxImpulse.x, rbXn_maxImpulse.y, rbXn_maxImpulse.z);
				const float targetVelocity = raXn_extraCoeff.w;
				const float maxImpulse = rbXn_maxImpulse.w;

				const float unitResponse = ref0 * resp0 + ref1 * resp1;
				const float recipResponse = (unitResponse > 0.f) ? (1.f / unitResponse) : 0.f;

				float velMultiplier = recipResponse;
				float impulseMul = 1.0f;
				float unbiasedError = 0.0f;
				float biasedErr = 0.0f;

				computeContactCoefficients(flags, restitution, unitResponse, recipResponse, targetVelocity, coeff0,
				                           coeff1, velMultiplier, impulseMul, unbiasedError, biasedErr);

				//Compute the normal velocity of the constraint.
				const float v0 = linVel0.dot(normal) + angVel0.dot(raXn);//V3MulAdd(linVel0, normal, V3Mul(angVel0, raXn));
				const float v1 = linVel1.dot(normal) + angVel1.dot(rbXn);//V3MulAdd(linVel1, normal, V3Mul(angVel1, rbXn));
				const float normalVel = v0 - v1;

				//KS - clamp the maximum force
				const float tempDeltaF = biasedErr - normalVel * velMultiplier;
				const float _deltaF = fmaxf(tempDeltaF, -appliedForce);//FMax(FNegScaleSub(normalVel, velMultiplier, biasedErr), FNeg(appliedForce));
				const float _newForce = appliedForce * impulseMul + _deltaF;
				const float newForce = fminf(_newForce, maxImpulse);//FMin(_newForce, maxImpulse);
				const float deltaF = newForce - appliedForce;

				linVel0 += delLinVel0 * deltaF;
				linVel1 -= delLinVel1 * deltaF;
				angVel0 += raXn * (deltaF * angDom0);
				angVel1 -= rbXn * (deltaF * angDom1);

				if(error)
					error->accumulateErrorLocal(deltaF, velMultiplier);

				Pxstcg(&c.appliedForce[threadIndex], newForce);

				accumulatedNormalImpulse = accumulatedNormalImpulse + newForce;
			}
		}

		if (numFrictionConstr && doFriction)
		{
			const float dynamicFrictionCof = Pxldcg(frictionHeader->dynamicFriction[threadIndex]);
			const float maxFrictionImpulse = staticFrictionCof * accumulatedNormalImpulse;
			const float maxDynFrictionImpulse = dynamicFrictionCof * accumulatedNormalImpulse;

			PxU32 broken = 0;

			for (uint i = 0; i < numFrictionConstr; i++)
			{
				PxgBlockSolverContactFriction& f = frictions[i];

				const float4 frictionNormal = Pxldg(frictionHeader->frictionNormals[i & 1][threadIndex]);

				const float4 raXn_extraCoeff = nextRaxn_extraCoeff;
				const float4 rbXn_targetVelW = nextRbxn_maxImpulseW;

				const float resp0 = nextResp0;
				const float resp1 = nextResp1;

				const float appliedForce = nextAppliedForce;

				if ((i + 1) < numFrictionConstr)
				{
					const PxgBlockSolverContactFriction& f2 = frictions[i + 1];
					nextRaxn_extraCoeff = Pxldcg(f2.raXn_bias[threadIndex]);
					nextRbxn_maxImpulseW = Pxldcg(f2.rbXn_targetVelW[threadIndex]);
					nextAppliedForce = Pxldcg(f2.appliedForce[threadIndex]);

					nextResp0 = Pxldcg(f2.resp0[threadIndex]);
					nextResp1 = Pxldcg(f2.resp1[threadIndex]);
				}

				const PxVec3 normal = PxVec3(frictionNormal.x, frictionNormal.y, frictionNormal.z);
				const PxVec3 raXn = PxVec3(raXn_extraCoeff.x, raXn_extraCoeff.y, raXn_extraCoeff.z);
				const PxVec3 rbXn = PxVec3(rbXn_targetVelW.x, rbXn_targetVelW.y, rbXn_targetVelW.z);

				const float resp = ref0 * resp0 + ref1 * resp1;
				const float velMultiplier = (resp > 0.f) ? (p8 / resp) : 0.f;

				const float bias = raXn_extraCoeff.w;

				const PxVec3 delLinVel0 = normal * invMassA;
				const PxVec3 delLinVel1 = normal * invMassB;

				const float targetVel = rbXn_targetVelW.w;

				const float v0 = angVel0.dot(raXn) + linVel0.dot(normal);//V3MulAdd(linVel0, normal, V3Mul(angVel0, raXn));
				const float v1 = angVel1.dot(rbXn) + linVel1.dot(normal);//V3MulAdd(linVel1, normal, V3Mul(angVel1, rbXn));
				const float normalVel = v0 - v1;

				const float tmp1 = appliedForce - (bias - targetVel) * velMultiplier;

				const float totalImpulse = tmp1 - normalVel * velMultiplier;

				const bool clamp = fabsf(totalImpulse) > maxFrictionImpulse;

				const float totalClamped = fminf(maxDynFrictionImpulse, fmaxf(-maxDynFrictionImpulse, totalImpulse));

				const float newAppliedForce = clamp ? totalClamped : totalImpulse;

				float deltaF = newAppliedForce - appliedForce;//FSub(newAppliedForce, appliedForce);

				if (error)
					error->accumulateErrorLocal(deltaF, velMultiplier);

				linVel0 += delLinVel0 * deltaF;
				linVel1 -= delLinVel1 * deltaF;
				angVel0 += raXn * (deltaF * angDom0);
				angVel1 -= rbXn * (deltaF * angDom1);

				Pxstcg(&f.appliedForce[threadIndex], newAppliedForce);
				broken = broken | clamp;
			}
			Pxstcg(&frictionHeader->broken[threadIndex], broken);
		}

	}

	// Write back
	b0LinVel = linVel0;
	b0AngVel = angVel0;
	b1LinVel = linVel1;
	b1AngVel = angVel1;
}

// A light version of the function "solveContactBlock" to quickly check if there is any active contact.
// TODO: Make this even lighter.

static __device__ bool checkActiveContactBlock(const PxgBlockConstraintBatch& batch, const PxVec3& linVel0,
	const PxVec3& angVel0, const PxVec3& linVel1, const PxVec3& angVel1, const PxU32 threadIndex,
	PxgBlockSolverContactHeader* contactHeaders, PxgBlockSolverContactPoint* contactPoints)
{
	using namespace physx;

	{
		PxgBlockSolverContactHeader* PX_RESTRICT contactHeader = &contactHeaders[batch.mConstraintBatchIndex];
		const uint numNormalConstr = Pxldcg(contactHeader->numNormalConstr[threadIndex]);
		PxgBlockSolverContactPoint* PX_RESTRICT contacts = &contactPoints[batch.startConstraintIndex];

		const float4 invMass0_1_angDom0_1 = Pxldcg(contactHeader->invMass0_1_angDom0_1[threadIndex]);

		const float4 normal_staticFriction = Pxldcg(contactHeader->normal_staticFriction[threadIndex]);
		const PxVec3 normal = PxVec3(normal_staticFriction.x, normal_staticFriction.y, normal_staticFriction.z);

		const float restitution = contactHeader->restitution[threadIndex];
		const PxU8 flags = contactHeader->flags[threadIndex];

		float4 nextRaxn_extraCoeff;
		float4 nextRbxn_maxImpulseW;
		float nextAppliedForce;

		float nextResp0;
		float nextResp1;

		{
			nextRaxn_extraCoeff = Pxldcg(contacts[0].raXn_targetVelocity[threadIndex]);
			nextRbxn_maxImpulseW = Pxldcg(contacts[0].rbXn_maxImpulse[threadIndex]);
			nextAppliedForce = Pxldcg(contacts[0].appliedForce[threadIndex]);

			nextResp0 = Pxldcg(contacts[0].resp0[threadIndex]);
			nextResp1 = Pxldcg(contacts[0].resp1[threadIndex]);

			float nextCoeff0 = Pxldcg(contacts[0].coeff0[threadIndex]);
			float nextCoeff1 = Pxldcg(contacts[0].coeff1[threadIndex]);

			for (uint i = 0; i < numNormalConstr; i++)
			{
				const float4 raXn_extraCoeff = nextRaxn_extraCoeff;
				const float4 rbXn_maxImpulse = nextRbxn_maxImpulseW;
				const float appliedForce = nextAppliedForce;

				const float resp0 = nextResp0;
				const float resp1 = nextResp1;

				const float coeff0 = nextCoeff0;
				const float coeff1 = nextCoeff1;

				if ((i + 1) < numNormalConstr)
				{
					const PxgBlockSolverContactPoint& nextC = contacts[i + 1];
					nextRaxn_extraCoeff = Pxldcg(nextC.raXn_targetVelocity[threadIndex]);
					nextRbxn_maxImpulseW = Pxldcg(nextC.rbXn_maxImpulse[threadIndex]);
					nextAppliedForce = Pxldcg(nextC.appliedForce[threadIndex]);

					nextResp0 = Pxldcg(nextC.resp0[threadIndex]);
					nextResp1 = Pxldcg(nextC.resp1[threadIndex]);

					nextCoeff0 = Pxldcg(nextC.coeff0[threadIndex]);
					nextCoeff1 = Pxldcg(nextC.coeff1[threadIndex]);
				}

				const PxVec3 raXn = PxVec3(raXn_extraCoeff.x, raXn_extraCoeff.y, raXn_extraCoeff.z);
				const PxVec3 rbXn = PxVec3(rbXn_maxImpulse.x, rbXn_maxImpulse.y, rbXn_maxImpulse.z);
				const float targetVelocity = raXn_extraCoeff.w;
				const float maxImpulse = rbXn_maxImpulse.w;

				const float unitResponse = resp0 + resp1;
				const float recipResponse = (unitResponse > 0.f) ? (1.f / unitResponse) : 0.f;

				float velMultiplier = recipResponse;
				float impulseMul = 1.0f;
				float unbiasedError = 0.0f;
				float biasedErr = 0.0f;

				computeContactCoefficients(flags, restitution, unitResponse, recipResponse, targetVelocity, coeff0,
					coeff1, velMultiplier, impulseMul, unbiasedError, biasedErr);

				//Compute the normal velocity of the constraint.
				const float v0 = linVel0.dot(normal) + angVel0.dot(raXn);//V3MulAdd(linVel0, normal, V3Mul(angVel0, raXn));
				const float v1 = linVel1.dot(normal) + angVel1.dot(rbXn);//V3MulAdd(linVel1, normal, V3Mul(angVel1, rbXn));
				const float normalVel = v0 - v1;

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
		}
	}

	return false;
}

static __device__ void concludeContactBlock(const PxgBlockConstraintBatch& batch, const PxU32 threadIndex, PxgBlockSolverContactHeader* contactHeaders, PxgBlockSolverFrictionHeader* frictionHeaders, 
	PxgBlockSolverContactPoint* contactPoints, PxgBlockSolverContactFriction* frictions)
{

	using namespace physx;

	{
		const PxgBlockSolverContactHeader* contactHeader = &contactHeaders[batch.mConstraintBatchIndex];
		const PxgBlockSolverFrictionHeader* frictionHeader = &frictionHeaders[batch.mConstraintBatchIndex];

		const uint32_t numNormalConstr = contactHeader->numNormalConstr[threadIndex];
		const uint32_t numFrictionConstr = frictionHeader->numFrictionConstr[threadIndex];

		PxgBlockSolverContactPoint* contacts = &contactPoints[batch.startConstraintIndex];
		if (numNormalConstr)
		{
			PxReal restitution = contactHeader->restitution[threadIndex];

			// Assigning unbiased error to biased error.
			// When restitution is negative (compliant contact), no additional care is required as it is automatically
			// enforced.
			if (restitution >= 0.f)
			{
				for (uint32_t i = 0; i < numNormalConstr; i++)
				{
					contacts[i].coeff0[threadIndex] = contacts[i].coeff1[threadIndex];
				}
			}
		}

		PxgBlockSolverContactFriction* frictionConstr = &frictions[batch.startFrictionIndex];
		for (uint32_t i = 0; i < numFrictionConstr; i++)
		{
			float4 raXn_bias = frictionConstr[i].raXn_bias[threadIndex];
			raXn_bias.w = 0.f;

			frictionConstr[i].raXn_bias[threadIndex] = raXn_bias;
		}
	}
}



static __device__ void writeBackContactBlock(const PxgBlockConstraintBatch& batch, const PxU32 threadIndex,
											 const PxgSolverBodyData* bodies, Dy::ThresholdStreamElement* thresholdStream,
											 PxI32* sharedThresholdStreamIndex, PxgBlockSolverContactHeader* contactHeaders, PxgBlockSolverFrictionHeader* frictionHeaders, 
											PxgBlockSolverContactPoint* contactPoints, PxgBlockSolverContactFriction* frictions,
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
		const PxgBlockSolverContactHeader* PX_RESTRICT contactHeader = &contactHeaders[batch.mConstraintBatchIndex];
		const PxgBlockSolverFrictionHeader* PX_RESTRICT frictionHeader = &frictionHeaders[batch.mConstraintBatchIndex];
	
		PxU32 forceWritebackOffset = contactHeader->forceWritebackOffset[threadIndex];

		forceThreshold = contactHeader->flags[threadIndex] & PxgSolverContactFlags::eHAS_FORCE_THRESHOLDS;

		const PxU32	numFrictionConstr = frictionHeader->numFrictionConstr[threadIndex];

		const PxU32 numNormalConstr = contactHeader->numNormalConstr[threadIndex];
		if(forceWritebackOffset!=0xFFFFFFFF)
		{
			PxReal* vForceWriteback = &forcewritebackBuffer[forceWritebackOffset];
			PxgBlockSolverContactPoint* c = &contactPoints[batch.startConstraintIndex];
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

static __device__ void writeBack1DBlock(const PxgBlockConstraintBatch& batch, const PxU32 threadIndex, const PxgBlockSolverConstraint1DHeader* PX_RESTRICT headers, 
	PxgBlockSolverConstraint1DCon* PX_RESTRICT rowsCon, PxgBlockSolverConstraint1DMod* PX_RESTRICT rowsMod,PxgConstraintWriteback* constraintWriteBacks)
{
	const PxgBlockSolverConstraint1DHeader* PX_RESTRICT  header = &headers[batch.mConstraintBatchIndex];
	PxgBlockSolverConstraint1DCon* conBase = &rowsCon[batch.startConstraintIndex];
	PxgBlockSolverConstraint1DMod* PX_RESTRICT modBase = &rowsMod[batch.startConstraintIndex];

	PxU32 forceWritebackOffset = header->writeBackOffset[threadIndex];

	const PxU8 breakable = header->breakable[threadIndex];

	const PxU32 numRows = header->rowCounts[threadIndex];

	if (forceWritebackOffset != 0xFFFFFFFF)
	{
		PxgConstraintWriteback& writeback = constraintWriteBacks[forceWritebackOffset];
	
		PxVec3 linVel(0), angVel(0);
		PxReal constraintErrorSq = 0.0f;
		for (PxU32 i = 0; i < numRows; ++i)
		{
			PxgBlockSolverConstraint1DCon& con = conBase[i];
			PxgBlockSolverConstraint1DMod& mod = modBase[i];

			if (mod.flags[threadIndex] & DY_SC_FLAG_OUTPUT_FORCE)
			{
				const float4 lin0XYZ_minImpulse = con.lin0XYZ_minImpulse[threadIndex];
				const PxVec3 lin0(lin0XYZ_minImpulse.x, lin0XYZ_minImpulse.y, lin0XYZ_minImpulse.z);
				const PxVec3 ang0WriteBack = mod.ang0Writeback[threadIndex];
				const PxReal appliedForce = mod.appliedForce[threadIndex];
				linVel += lin0 * appliedForce;
				angVel += ang0WriteBack *appliedForce;
			}

			PxReal err = mod.residual[threadIndex];
			constraintErrorSq += err * err;
		}

		const float4 body0WorldOffset_linBreakImpulse = header->body0WorldOffset_linBreakImpulse[threadIndex];
		const PxVec3 body0WorldOffset(body0WorldOffset_linBreakImpulse.x, body0WorldOffset_linBreakImpulse.y, body0WorldOffset_linBreakImpulse.z);
		angVel -= body0WorldOffset.cross(linVel);


		const PxU32 broken = breakable ? PxU32((linVel.magnitude() > body0WorldOffset_linBreakImpulse.w) || (angVel.magnitude() > header->angBreakImpulse[threadIndex])) : 0;
		writeback.angularImpulse_residual = make_float4(angVel.x, angVel.y, angVel.z, constraintErrorSq);
		writeback.linearImpulse_broken = make_float4(linVel.x, linVel.y, linVel.z, broken ? -0.0f : 0.0f);

	}

}


#endif
