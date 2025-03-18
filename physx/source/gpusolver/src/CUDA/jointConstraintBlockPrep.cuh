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

#ifndef	__JOINT_CONSTRAINT_BLOCK_PREP_CUH__
#define	__JOINT_CONSTRAINT_BLOCK_PREP_CUH__

#include "PxConstraintDesc.h"
#include "PxConstraint.h"
#include "DySolverConstraintTypes.h"
#include "DyCpuGpu1dConstraint.h"
#include "PxgSolverBody.h"
#include "PxgConstraint.h"
#include "PxgSolverConstraintBlock1D.h"
#include "PxgCudaMemoryAllocator.h"
#include "PxgSolverConstraintDesc.h"
#include "PxgConstraintPrep.h"
#include "foundation/PxVec4.h"
#include "MemoryAllocator.cuh"
#include "PxgSolverKernelIndices.h"

using namespace physx;

namespace physx
{
	struct PxgMassProps
	{
		float invMass0;
		float invMass1;
		float invInertiaScale0;
		float invInertiaScale1;

		__device__ PxgMassProps(const PxReal iMass0, const PxReal iMass1, 
			const float4 lin0_ang0_lin1_ang1)
		{
			invMass0 = iMass0 * lin0_ang0_lin1_ang1.x;
			invMass1 = iMass1 * lin0_ang0_lin1_ang1.z;
			invInertiaScale0 = lin0_ang0_lin1_ang1.y;
			invInertiaScale1 = lin0_ang0_lin1_ang1.w;
		}

	};
}


//
// See orthogonalize() in DyConstraintSetup.cpp for a general explanation
//
static __device__ void orthogonalize( PxU32* sortedRowIndices, PxgBlockConstraint1DVelocities* rvs, PxgBlockConstraint1DParameters* rps,
										PxVec3* angSqrtInvInertia0,
										PxVec3* angSqrtInvInertia1,
										PxU32 rowCount, 
										PxU32 eqRowCount,
										const physx::PxgMassProps* m,
										const PxU32 threadIndex)
{
	using namespace physx;

	assert(eqRowCount<=6);

	PxVec3 lin1m[6], ang1m[6], lin1[6], ang1[6];	
	PxVec3 lin0m[6], ang0m[6], lin0[6], ang0[6];

	PxReal geomErr[6];
	PxReal velTarget[6];

	for(PxU32 i=0;i<rowCount;i++)
	{
		const PxU32 index = sortedRowIndices[i];
		const float4 linear0XYZ_geometricErrorW = rvs[index].linear0XYZ_geometricErrorW[threadIndex];
		const float4 linear1XYZ_minImpulseW = rvs[index].linear1XYZ_minImpulseW[threadIndex];
		const float4 angular0XYZ_velocityTargetW = rvs[index].angular0XYZ_velocityTargetW[threadIndex];
		const float4 angular1XYZ_maxImpulseW = rvs[index].angular1XYZ_maxImpulseW[threadIndex];

		PxVec3 l0(linear0XYZ_geometricErrorW.x, linear0XYZ_geometricErrorW.y, linear0XYZ_geometricErrorW.z);
		PxVec3 a0(angular0XYZ_velocityTargetW.x, angular0XYZ_velocityTargetW.y, angular0XYZ_velocityTargetW.z);

		PxVec3 l1(linear1XYZ_minImpulseW.x, linear1XYZ_minImpulseW.y, linear1XYZ_minImpulseW.z);
		PxVec3 a1(angular1XYZ_maxImpulseW.x, angular1XYZ_maxImpulseW.y, angular1XYZ_maxImpulseW.z);

		PxVec3 angSqrtL0 = angSqrtInvInertia0[i];
		PxVec3 angSqrtL1 = angSqrtInvInertia1[i];

		PxReal g = linear0XYZ_geometricErrorW.w;
		PxReal T = angular0XYZ_velocityTargetW.w;

		PxU32 eliminationRows = PxMin<PxU32>(i, eqRowCount);
		for(PxU32 j=0;j<eliminationRows;j++)
		{
			const PxVec3 s0 = l1.multiply(lin1m[j]) + l0.multiply(lin0m[j]);
			const PxVec3 s1 = angSqrtL1.multiply(ang1m[j]) + angSqrtL0.multiply(ang0m[j]);
			const PxVec3 s0s1 = s0+s1;
			float t = s0s1.x + s0s1.y + s0s1.z;

			l0 = l0 - (lin0[j] * t);
			a0 = a0 - (ang0[j] * t);
			l1 = l1 - (lin1[j] * t);
			a1 = a1 - (ang1[j] * t);
			g = g - (geomErr[j] * t);
			T = T - (velTarget[j] * t);
			angSqrtL0 = angSqrtL0 - (angSqrtInvInertia0[j] * t);
			angSqrtL1 = angSqrtL1 - (angSqrtInvInertia1[j] * t);
		}

		rvs[index].linear0XYZ_geometricErrorW[threadIndex]		= make_float4(l0.x, l0.y, l0.z, g);
		rvs[index].angular0XYZ_velocityTargetW[threadIndex]		= make_float4(a0.x, a0.y, a0.z, T);
		rvs[index].linear1XYZ_minImpulseW[threadIndex]			= make_float4(l1.x,		l1.y,		l1.z,	linear1XYZ_minImpulseW.w);	
		rvs[index].angular1XYZ_maxImpulseW[threadIndex]			= make_float4(a1.x,		a1.y,		a1.z,	angular1XYZ_maxImpulseW.w);
		angSqrtInvInertia0[i] = angSqrtL0;
		angSqrtInvInertia1[i] = angSqrtL1;


		if(i<eqRowCount)
		{
			lin0[i] = l0;	
			ang0[i] = a0;
			geomErr[i] = g;
			velTarget[i] = T;
			lin1[i] = l1;	
			ang1[i] = a1;	
			angSqrtInvInertia0[i] = angSqrtL0;
			angSqrtInvInertia1[i] = angSqrtL1;
			
			const PxVec3 l0m = l0 * m->invMass0;
			const PxVec3 l1m = l1 * m->invMass1;
			const PxVec3 a0m = angSqrtL0 * m->invInertiaScale0;
			const PxVec3 a1m = angSqrtL1 * m->invInertiaScale1;

			const PxVec3 s0 = l0.multiply(l0m) + l1.multiply(l1m);
			const PxVec3 s1 = a0m.multiply(angSqrtL0) + a1m.multiply(angSqrtL1);
			const PxVec3 s0s1 = s0 + s1;
			const float s = s0s1.x + s0s1.y + s0s1.z;
			const float a = s > 0 ? 1.f/s : 0.f;  // with mass scaling, it's possible for the inner product of a row to be zero

			lin0m[i] = l0m * a;
			ang0m[i] = a0m * a;
			lin1m[i] = l1m * a;
			ang1m[i] = a1m * a;
		}
	}
}




static __device__ void preprocessRows(PxU32* sortedRowIndices, PxgBlockConstraint1DData* constraintData, 
									  PxgBlockConstraint1DVelocities* rowVelocities, PxgBlockConstraint1DParameters* rowParameters,
									  PxVec3* angSqrtInvInertia0, PxVec3* angSqrtInvInertia1,
									  const physx::PxgSolverBodyPrepData* bd0, const physx::PxgSolverBodyPrepData* bd1,
									  PxgSolverTxIData* txIData0, PxgSolverTxIData* txIData1,
									  const PxU32 threadIndex, bool disablePreprocessing)
{
	using namespace physx;

	//Px1DConstraint* sorted[MAX_CONSTRAINTS];
	// j is maxed at 12, typically around 7, so insertion sort is fine

	for(PxU32 i=0; i<constraintData->mNumRows[threadIndex]; i++)
	{
		PxgBlockConstraint1DParameters& r = rowParameters[i];
		
		PxU32 j = i;
		for(;j>0 && r.solveHint[threadIndex] < rowParameters[sortedRowIndices[j-1]].solveHint[threadIndex]; j--)
			sortedRowIndices[j] = sortedRowIndices[j-1];

		sortedRowIndices[j] = i;
	}

	/*for(PxU32 i=1;i<constraintData->mNumRows[threadIndex];i++)
		assert(sorted[i-1]->solveHint[threadIndex] <= sorted[i]->solveHint[threadIndex]);*/

	PxgMassProps m(bd0->initialLinVelXYZ_invMassW.w, bd1->initialLinVelXYZ_invMassW.w, reinterpret_cast<float4*>(constraintData->mInvMassScale)[threadIndex]);

	const PxMat33 i0 = txIData0->sqrtInvInertia;
	const PxMat33 i1 = txIData1->sqrtInvInertia;

	for(PxU32 i = 0; i < constraintData->mNumRows[threadIndex]; ++i)
	{
		/*const PxVec3 angDelta0 = bd0->sqrtInvInertia * sorted[i]->angular0[threadIndex];
		const PxVec3 angDelta1 = bd1->sqrtInvInertia * sorted[i]->angular1[threadIndex];*/
		PxgBlockConstraint1DVelocities& rv =  rowVelocities[sortedRowIndices[i]];

		const float4 angular0XYZ_velocityTargetW = rv.angular0XYZ_velocityTargetW[threadIndex];
		const float4 angular1XYZ_maxImpulseW = rv.angular1XYZ_maxImpulseW[threadIndex];
		const PxVec3 angular0(angular0XYZ_velocityTargetW.x,	angular0XYZ_velocityTargetW.y,	angular0XYZ_velocityTargetW.z);
		const PxVec3 angular1(angular1XYZ_maxImpulseW.x,		angular1XYZ_maxImpulseW.y,		angular1XYZ_maxImpulseW.z);
		/*const PxVec3 angDelta0 = bd0->sqrtInvInertia * angular0;
		const PxVec3 angDelta1 = bd1->sqrtInvInertia * angular1;
		angSqrtInvInertia0[i] = angDelta0;
		angSqrtInvInertia1[i] = angDelta1;*/
		angSqrtInvInertia0[i] = i0 * angular0;
		angSqrtInvInertia1[i] = i1 * angular1;
	}


	if (!disablePreprocessing)
	{
		//MassProps m(bd0, bd1, ims);
		for (PxU32 i = 0; i < constraintData->mNumRows[threadIndex];)
		{
			PxgBlockConstraint1DParameters& rp = rowParameters[sortedRowIndices[i]];

			const PxU32 groupMajorId = PxU32(rp.solveHint[threadIndex] >> 8), start = i++;
			while (i < constraintData->mNumRows[threadIndex] && PxU32(rowParameters[sortedRowIndices[i]].solveHint[threadIndex] >> 8) == groupMajorId)
				i++;

			if (groupMajorId == 4 || (groupMajorId == 8))
			{
				PxU32 bCount = start;		// count of bilateral constraints 
				for (; bCount < i && (rowParameters[sortedRowIndices[bCount]].solveHint[threadIndex] & 255) == 0; bCount++)
					;

				orthogonalize(sortedRowIndices + start, rowVelocities, rowParameters, angSqrtInvInertia0 + start, angSqrtInvInertia1 + start, i - start, bCount - start, &m, threadIndex);

			}
		}
	}
}
  
static __device__ void intializeBlock1D(const physx::PxgBlockConstraint1DVelocities& rv,
										const physx::PxgBlockConstraint1DParameters& rp,
											float jointSpeedForRestitutionBounce,
											float initJointSpeed,
											float resp0,
											float resp1,
											float erp,
											float dt,
											float recipdt,
											PxgBlockSolverConstraint1DCon& scon,
											PxgBlockSolverConstraint1DMod& smod,
											const PxVec3& _linear0, const PxVec3& _linear1, 
											const PxVec3& _angular0, const PxVec3& _angular1,
											const PxReal _minImpulse, const PxReal _maxImpulse, 
											const PxReal cfm,
											const PxU32 threadIndex
											)
{
	using namespace physx;
	
	{
		const PxU16 flags = rp.flags[threadIndex];
		const PxReal springStiffness = rp.mods.spring.stiffness[threadIndex];
		const PxReal springDamping = rp.mods.spring.damping[threadIndex];
		const PxReal restitution = rp.mods.bounce.restitution[threadIndex];
		const PxReal bounceThreshold = rp.mods.bounce.velocityThreshold[threadIndex];
		const PxReal geomError = rv.linear0XYZ_geometricErrorW[threadIndex].w;
		const PxReal velocityTarget = rv.angular0XYZ_velocityTargetW[threadIndex].w;

		PxReal coeff0, coeff1;
		queryReduced1dConstraintSolverConstantsPGS(flags, springStiffness, springDamping, restitution, bounceThreshold, geomError,
												   velocityTarget, jointSpeedForRestitutionBounce, erp, dt, recipdt, coeff0, coeff1);

		scon.lin0XYZ_minImpulse[threadIndex] = make_float4(_linear0.x, _linear0.y, _linear0.z, _minImpulse);
		scon.lin1XYZ_maxImpulse[threadIndex] = make_float4(_linear1.x, _linear1.y, _linear1.z, _maxImpulse);
		scon.ang0XYZ_resp0[threadIndex] = make_float4(_angular0.x, _angular0.y, _angular0.z, resp0);
		scon.ang1XYZ_resp1[threadIndex] = make_float4(_angular1.x, _angular1.y, _angular1.z, resp1);
		scon.initJointSpeed[threadIndex] = initJointSpeed;

		smod.coeff0[threadIndex] = coeff0;
		smod.coeff1[threadIndex] = coeff1;

		smod.appliedForce[threadIndex] = 0;
		smod.residual[threadIndex] = 0;

		// Instead of setting the flag to zero as in the previous implementation, the flag is used to mark spring and
		// acceleration spring.
		smod.flags[threadIndex] = 0;
		if (flags & Px1DConstraintFlag::eSPRING)
			smod.flags[threadIndex] |= DY_SC_FLAG_SPRING;

		if (flags & Px1DConstraintFlag::eACCELERATION_SPRING)
			smod.flags[threadIndex] |= DY_SC_FLAG_ACCELERATION_SPRING;
	}
}

static __device__ void setUp1DConstraintBlock(PxU32* sortedRowIndices, PxgBlockConstraint1DData* constraintData, PxgBlockConstraint1DVelocities* rowVelocities, PxgBlockConstraint1DParameters* rowParameters, 
								  PxVec3* angSqrtInvInertia0, PxVec3* angSqrtInvInertia1, PxgBlockSolverConstraint1DCon* constraintsCon, PxgBlockSolverConstraint1DMod* constraintsMod,
									float dt, float recipdt, const physx::PxgSolverBodyPrepData* sBodyData0, const physx::PxgSolverBodyPrepData* sBodyData1,
									const PxU32 threadIndex)
{
	using namespace physx;

	//PxU32 stride = sizeof(PxgSolverConstraint1D);

	const PxReal erp = 1.0f;
	const float4 sBodyData0_initialLinVelXYZ_invMassW0 = sBodyData0->initialLinVelXYZ_invMassW;
	const float4 sBodyData1_initialLinVelXYZ_invMassW1 = sBodyData1->initialLinVelXYZ_invMassW;

	const PxU32 numRows = constraintData->mNumRows[threadIndex];
	
	for(PxU32 i=0;i<numRows;i++)
	{
		PxgBlockSolverConstraint1DCon& ccon = constraintsCon[i];
		PxgBlockSolverConstraint1DMod& cmod = constraintsMod[i];
		//Pxg1DConstraintBlock& c = *sorted[i];
		const PxU32 index = sortedRowIndices[i];
		PxgBlockConstraint1DParameters& rp = rowParameters[index];
		PxgBlockConstraint1DVelocities& rv = rowVelocities[index];

		const float4 c_linear0XYZ_geometricErrorW = rv.linear0XYZ_geometricErrorW[threadIndex];
		const float4 c_linear1XYZ_minImpulseW = rv.linear1XYZ_minImpulseW[threadIndex];
		const float4 c_angular0XYZ_velocityTargetW = rv.angular0XYZ_velocityTargetW[threadIndex];
		const float4 c_angular1XYZ_maxImpulseW = rv.angular1XYZ_maxImpulseW[threadIndex];

		const PxVec3 clin0(c_linear0XYZ_geometricErrorW.x,	c_linear0XYZ_geometricErrorW.y,		c_linear0XYZ_geometricErrorW.z);
		const PxVec3 clin1(c_linear1XYZ_minImpulseW.x,		c_linear1XYZ_minImpulseW.y,			c_linear1XYZ_minImpulseW.z);
		const PxVec3 cang0(c_angular0XYZ_velocityTargetW.x, c_angular0XYZ_velocityTargetW.y,	c_angular0XYZ_velocityTargetW.z);
		const PxVec3 cang1(c_angular1XYZ_maxImpulseW.x,		c_angular1XYZ_maxImpulseW.y,		c_angular1XYZ_maxImpulseW.z);
		const PxVec3 ang0 = angSqrtInvInertia0[i];
		const PxVec3 ang1 = angSqrtInvInertia1[i];


		PxReal minImpulse;
		PxReal maxImpulse;
		{
			const bool hasDriveLimit = rp.flags[threadIndex] & Px1DConstraintFlag::eHAS_DRIVE_LIMIT;
			const bool driveLimitsAreForces = constraintData->mFlags[threadIndex] & PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES;
			Dy::computeMinMaxImpulseOrForceAsImpulse(
				c_linear1XYZ_minImpulseW.w, c_angular1XYZ_maxImpulseW.w,
				hasDriveLimit, driveLimitsAreForces, dt,
				minImpulse, maxImpulse);
		}	
	
		cmod.ang0Writeback[threadIndex] = cang0;

		const float4 lin0_ang0_lin1_ang1 = constraintData->mInvMassScale[threadIndex].lin0X_ang0Y_lin1Z_ang1W;
	
		PxReal resp0 = clin0.magnitudeSquared() * sBodyData0_initialLinVelXYZ_invMassW0.w * lin0_ang0_lin1_ang1.x + ang0.magnitudeSquared() * lin0_ang0_lin1_ang1.y;
		PxReal resp1 = clin1.magnitudeSquared() * sBodyData1_initialLinVelXYZ_invMassW1.w * lin0_ang0_lin1_ang1.z + ang1.magnitudeSquared() * lin0_ang0_lin1_ang1.w;

		const PxReal initJointSpeed = sBodyData0->projectVelocity(clin0, cang0) - sBodyData1->projectVelocity(clin1, cang1);

		// Following the previous implementation, cfm is not used in unitResponse, thus it is set to 0.
		intializeBlock1D(rv, rp, initJointSpeed, initJointSpeed, resp0, resp1, erp, dt, recipdt, ccon, cmod, clin0, clin1, ang0, ang1, minImpulse, maxImpulse, 0.0f, threadIndex);

		if(rp.flags[threadIndex] & Px1DConstraintFlag::eOUTPUT_FORCE)
			cmod.flags[threadIndex] |= DY_SC_FLAG_OUTPUT_FORCE;
	}
}


template<int NbThreads>
static __device__ void setupSolverConstraintBlockGPU(PxgBlockConstraint1DData* constraintData, PxgBlockConstraint1DVelocities* rowVelocities, PxgBlockConstraint1DParameters* rowParameters, 
													 const physx::PxgSolverBodyPrepData* sBodyData0, const physx::PxgSolverBodyPrepData* sBodyData1, PxgSolverTxIData* txIData0, PxgSolverTxIData* txIData1,
													float dt, float recipdt, PxgBlockConstraintBatch& batch, 
													 const PxU32 threadIndex, PxgBlockSolverConstraint1DHeader* header, PxgBlockSolverConstraint1DCon* rowsCon, PxgBlockSolverConstraint1DMod* rowsMod,
													 const PxgSolverConstraintManagerConstants& managerConstants)
{
	using namespace physx;

	//distance constraint might have zero number of rows	
	header->rowCounts[threadIndex] = PxU8(constraintData->mNumRows[threadIndex]);

	header->writeBackOffset[threadIndex] = managerConstants.mConstraintWriteBackIndex;

	const float4 lin0_ang0_lin1_ang1 = constraintData->mInvMassScale[threadIndex].lin0X_ang0Y_lin1Z_ang1W;

	const float4 initialLinVelXYZ_invMassW0 = sBodyData0->initialLinVelXYZ_invMassW;
	const float4 initialLinVelXYZ_invMassW1 = sBodyData1->initialLinVelXYZ_invMassW;
	
	const float4 raWorld_linBreakForce = constraintData->mRAWorld_linBreakForce[threadIndex];
	const float4 rbWorld_angBreakForce = constraintData->mRBWorld_AngBreakForce[threadIndex];
	const float linBreakImpulse = raWorld_linBreakForce.w * dt;
	const float angBreakForce = rbWorld_angBreakForce.w;
	const float angBreakImpulse = angBreakForce * dt;
	header->body0WorldOffset_linBreakImpulse[threadIndex] = make_float4(raWorld_linBreakForce.x, raWorld_linBreakForce.y, raWorld_linBreakForce.z, linBreakImpulse);
	header->angBreakImpulse[threadIndex] = angBreakImpulse;

	header->invMass0D0[threadIndex] = initialLinVelXYZ_invMassW0.w * lin0_ang0_lin1_ang1.x;
	header->invMass1D1[threadIndex] = initialLinVelXYZ_invMassW1.w * lin0_ang0_lin1_ang1.z;
	header->invInertiaScale0[threadIndex] = lin0_ang0_lin1_ang1.y;
	header->invInertiaScale1[threadIndex] = lin0_ang0_lin1_ang1.w;
	
	header->breakable[threadIndex] = PxU8((raWorld_linBreakForce.w != PX_MAX_F32) || (angBreakForce != PX_MAX_F32));

	__shared__ PxU32 sortedRowIndices[NbThreads][Dy::MAX_CONSTRAINT_ROWS];
	__shared__ PxVec3 angSqrtInvInertia0[NbThreads][Dy::MAX_CONSTRAINT_ROWS];
	__shared__ PxVec3 angSqrtInvInertia1[NbThreads][Dy::MAX_CONSTRAINT_ROWS];
	
	preprocessRows(sortedRowIndices[threadIdx.x], constraintData, rowVelocities, rowParameters, angSqrtInvInertia0[threadIdx.x], angSqrtInvInertia1[threadIdx.x], sBodyData0, sBodyData1, txIData0, txIData1, threadIndex, !!(constraintData->mFlags[threadIndex] & PxConstraintFlag::eDISABLE_PREPROCESSING));
	setUp1DConstraintBlock(sortedRowIndices[threadIdx.x], constraintData, rowVelocities, rowParameters, angSqrtInvInertia0[threadIdx.x], angSqrtInvInertia1[threadIdx.x], rowsCon, rowsMod, dt, recipdt, sBodyData0, sBodyData1, threadIndex);
}


#endif