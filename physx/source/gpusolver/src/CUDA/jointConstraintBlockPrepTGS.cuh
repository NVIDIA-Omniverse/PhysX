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

#ifndef	__JOINT_CONSTRAINT_BLOCK_PREP_TGS_CUH__
#define	__JOINT_CONSTRAINT_BLOCK_PREP_TGS_CUH__

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
#include "jointConstraintBlockPrep.cuh"


using namespace physx;
  
static __device__ PxU32 intializeBlock1DTGS
(const physx::PxgBlockConstraint1DVelocities& rv, const physx::PxgBlockConstraint1DParameters& rp, const PxgBlockConstraint1DData& constraintData,
 const PxReal jointSpeedForRestitutionBounce, const PxReal initJointSpeed,
 const PxReal unitResponse, const PxReal minRowResponse,
 const PxReal erp, const PxReal lengthScale,
 const PxReal stepDt, const PxReal simDt, const PxReal recipStepDt, const PxReal recipSimDt,
 const PxVec3& angSqrtInvInertia0, const PxVec3& angSqrtInvInertia1,
 const PxReal invInertiaScale0, const PxReal invInertiaScale1,
 const PxU32 eqCount, const PxU32 threadIndex, const bool disablePreprocessing,
 PxgTGSBlockSolverConstraint1DHeader& hdr,
 PxgTGSBlockSolverConstraint1DCon& scon)
{
	using namespace physx;

	//Copy min and max impulse because the convention of
	//function inputs and outputs is very confusing.

	PxReal maxBiasVelocity = 0.0f;
	PxReal recipUnitResponse = 0.0f;
	const PxReal geometricError = rv.linear0XYZ_geometricErrorW[threadIndex].w;
	Dy::Constraint1dSolverConstantsTGS desc = {0.0f, 0.0f, 0.0f, 0.0f};
	{
		const PxU16 flags = PxU16(rp.flags[threadIndex]);
		const PxReal stiffness = rp.mods.spring.stiffness[threadIndex];
		const PxReal damping = rp.mods.spring.damping[threadIndex];
		const PxReal restitution = rp.mods.bounce.restitution[threadIndex];
		const PxReal bounceVelocityThreshold = rp.mods.bounce.velocityThreshold[threadIndex];
		const PxReal velocityTarget = rv.angular0XYZ_velocityTargetW[threadIndex].w;

		maxBiasVelocity = Dy::computeMaxBiasVelocityTGS(flags, jointSpeedForRestitutionBounce, bounceVelocityThreshold, 
			restitution, geometricError, false, lengthScale, recipSimDt);

		recipUnitResponse = Dy::computeRecipUnitResponse(unitResponse, minRowResponse);

		desc = Dy::compute1dConstraintSolverConstantsTGS(
			flags, 
			stiffness, damping,
			restitution, bounceVelocityThreshold,
			geometricError, velocityTarget, 
			jointSpeedForRestitutionBounce, initJointSpeed, 
			unitResponse, recipUnitResponse, 
			erp, 
			stepDt, recipStepDt);
	}

	//Write to the w-components of each float4.
	//set the biasScale
	float4 lin1XYZ_biasScale = rv.linear1XYZ_minImpulseW[threadIndex];
	lin1XYZ_biasScale.w = desc.biasScale;
	//set the initBias
	float4 lin0XYZ_initBiasW = rv.linear0XYZ_geometricErrorW[threadIndex];
	lin0XYZ_initBiasW.w = desc.error;
	//set the velMultiplier
	float4 ang0XYZ_velMultiplierW =  rv.angular0XYZ_velocityTargetW[threadIndex];
	ang0XYZ_velMultiplierW.w = desc.velMultiplier;
	//set the velTarget
	float4 ang1XYZ_velTargetW = rv.angular1XYZ_maxImpulseW[threadIndex];
	ang1XYZ_velTargetW.w = desc.targetVel;
	
	PxReal angularErrorScale = 1.f;
	if (!(rp.flags[threadIndex] & Px1DConstraintFlag::eANGULAR_CONSTRAINT))
	{
		ang0XYZ_velMultiplierW.x = ang0XYZ_velMultiplierW.y = ang0XYZ_velMultiplierW.z = 0.f;
		ang1XYZ_velTargetW.x = ang1XYZ_velTargetW.y = ang1XYZ_velTargetW.z = 0.f;
		angularErrorScale = 0.f;
	}

	scon.lin0XYZ_initBiasOrCoeff0[threadIndex] = lin0XYZ_initBiasW;
	scon.lin1XYZ_biasScaleOrCoeff1[threadIndex] = lin1XYZ_biasScale;
	scon.ang0XYZ_velMultiplierOrCoeff2[threadIndex] = ang0XYZ_velMultiplierW;
	scon.ang1XYZ_velTargetOrCoeff3[threadIndex] = ang1XYZ_velTargetW;

	scon.maxBias[threadIndex] = maxBiasVelocity;
	scon.angularErrorScale[threadIndex] = angularErrorScale;
	scon.appliedForce[threadIndex] = 0.f;
	scon.residual[threadIndex] = 0.0f;

	const bool hasDriveLimit = rp.flags[threadIndex] & Px1DConstraintFlag::eHAS_DRIVE_LIMIT;
	const bool driveLimitsAreForces = constraintData.mFlags[threadIndex] & PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES;
	Dy::computeMinMaxImpulseOrForceAsImpulse(
			rv.linear1XYZ_minImpulseW[threadIndex].w, rv.angular1XYZ_maxImpulseW[threadIndex].w,
			hasDriveLimit, driveLimitsAreForces, simDt,
			scon.minImpulse[threadIndex], scon.maxImpulse[threadIndex]);

	PxU32 outFlags = 0;
	const PxU32 solveHint = rp.solveHint[threadIndex];
	Dy::raiseInternalFlagsTGS(rp.flags[threadIndex], solveHint, outFlags);

	PxU32 ret = 0;
	if (!disablePreprocessing)
	{
		if (solveHint == PxConstraintSolveHint::eROTATIONAL_EQUALITY)
		{
			ret = 1;
			outFlags |= DY_SC_FLAG_ROT_EQ;

			hdr.angOrthoAxis0_recipResponseW[eqCount][threadIndex] = make_float4(angSqrtInvInertia0.x * invInertiaScale0, angSqrtInvInertia0.y * invInertiaScale0,
				angSqrtInvInertia0.z * invInertiaScale0, recipUnitResponse);
			hdr.angOrthoAxis1_ErrorW[eqCount][threadIndex] = make_float4(angSqrtInvInertia1.x * invInertiaScale1, angSqrtInvInertia1.y * invInertiaScale1,
				angSqrtInvInertia1.z * invInertiaScale1, geometricError);
		}
		else if(solveHint & PxConstraintSolveHint::eEQUALITY)
			outFlags |= DY_SC_FLAG_ORTHO_TARGET;
	}

	scon.flags[threadIndex] = outFlags;
	
	return ret;
}


static __device__ PxU32 setUp1DConstraintBlockTGS
(PxU32* sortedRowIndices, PxgBlockConstraint1DData* constraintData, PxgBlockConstraint1DVelocities* rowVelocities, PxgBlockConstraint1DParameters* rowParameters, 
 PxVec3* angSqrtInvInertias0, PxVec3* angSqrtInvInertias1, PxgTGSBlockSolverConstraint1DHeader& header, PxgTGSBlockSolverConstraint1DCon* constraintsCon,
 float stepDt, float recipStepDt, float simDt, float recipSimDt, float biasCoefficient, const physx::PxgSolverBodyData* sBodyData0, const physx::PxgSolverBodyData* sBodyData1,
 const PxU32 threadIndex, const PxReal lengthScale, bool disablePreprocessing)
{
	using namespace physx;

	//PxU32 stride = sizeof(PxgSolverConstraint1D);

	const PxReal erp = 0.5f * biasCoefficient;
	const float4 sBodyData0_initialLinVelXYZ_invMassW0 = sBodyData0->initialLinVelXYZ_invMassW;
	const float4 sBodyData1_initialLinVelXYZ_invMassW1 = sBodyData1->initialLinVelXYZ_invMassW;

	const PxU32 numRows = constraintData->mNumRows[threadIndex];

	const bool isKinematic0 = !!(sBodyData0->flags & PxRigidBodyFlag::eKINEMATIC);
	const bool isKinematic1 = !!(sBodyData1->flags & PxRigidBodyFlag::eKINEMATIC);
	
	PxU32 eqCount = 0;
	for(PxU32 i=0;i<numRows;i++)
	{
		PxgTGSBlockSolverConstraint1DCon& ccon = constraintsCon[i];
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
		const PxVec3 angSqrtInvInertia0 = angSqrtInvInertias0[i];
		const PxVec3 angSqrtInvInertia1 = angSqrtInvInertias1[i];

		const float4 lin0_ang0_lin1_ang1 = constraintData->mInvMassScale[threadIndex].lin0X_ang0Y_lin1Z_ang1W;
	
		PxReal unitResponse;
		{
			const PxReal resp0 = clin0.magnitudeSquared() * sBodyData0_initialLinVelXYZ_invMassW0.w * lin0_ang0_lin1_ang1.x + angSqrtInvInertia0.magnitudeSquared() * lin0_ang0_lin1_ang1.y;
			const PxReal resp1 = clin1.magnitudeSquared() * sBodyData1_initialLinVelXYZ_invMassW1.w * lin0_ang0_lin1_ang1.z + angSqrtInvInertia1.magnitudeSquared() * lin0_ang0_lin1_ang1.w;
			unitResponse  = resp0 + resp1;
		}

		PxReal jointSpeedForRestitutionBounce;
		PxReal initJointSpeed;
		{
			const float vel0 = sBodyData0->projectVelocity(clin0, cang0);
			const float vel1 = sBodyData1->projectVelocity(clin1, cang1);
			Dy::computeJointSpeedTGS(
				vel0, isKinematic0, vel1, isKinematic1, 
				jointSpeedForRestitutionBounce, initJointSpeed);
		}


		//https://omniverse-jirasw.nvidia.com/browse/PX-4383
		const PxReal minRowResponse = DY_MIN_RESPONSE;

		eqCount += intializeBlock1DTGS(
			rv, rp, *constraintData,
			jointSpeedForRestitutionBounce, initJointSpeed,
			unitResponse, minRowResponse, erp, lengthScale,
			stepDt, simDt, recipStepDt, recipSimDt, 
			angSqrtInvInertia0, angSqrtInvInertia1, 
			lin0_ang0_lin1_ang1.y, lin0_ang0_lin1_ang1.w,
			eqCount, threadIndex, disablePreprocessing,			
			header, ccon);
	}

	for (PxU32 i = eqCount; i < 3; ++i)
	{
		header.angOrthoAxis0_recipResponseW[i][threadIndex] = make_float4(0.f);
		header.angOrthoAxis1_ErrorW[i][threadIndex] = make_float4(0.f);
	}

	return eqCount;
}


template<int NbThreads>
static __device__ void setupSolverConstraintBlockGPUTGS(PxgBlockConstraint1DData* constraintData, PxgBlockConstraint1DVelocities* rowVelocities, PxgBlockConstraint1DParameters* rowParameters, 
													const physx::PxgSolverBodyData* sBodyData0, const physx::PxgSolverBodyData* sBodyData1, PxgSolverTxIData* txIData0, PxgSolverTxIData* txIData1,
													float dt, float recipdt, float totalDt, float recipTotalDt, float lengthScale, float biasCoefficient, PxgBlockConstraintBatch& batch, 
													const PxU32 threadIndex, PxgTGSBlockSolverConstraint1DHeader* header, PxgTGSBlockSolverConstraint1DCon* rowsCon, const PxgSolverConstraintManagerConstants& managerConstants)
{
	using namespace physx;

	//distance constraint might have zero number of rows	
	const PxU32 numRows = constraintData->mNumRows[threadIndex];

	uchar4 rowCounts_breakable_orthoAxisCount;



	rowCounts_breakable_orthoAxisCount.x = PxU8(numRows);
	

	header->writeBackOffset[threadIndex] = managerConstants.mConstraintWriteBackIndex;

	const float4 lin0_ang0_lin1_ang1 = constraintData->mInvMassScale[threadIndex].lin0X_ang0Y_lin1Z_ang1W;

	const float4 initialLinVelXYZ_invMassW0 = sBodyData0->initialLinVelXYZ_invMassW;
	const float4 initialLinVelXYZ_invMassW1 = sBodyData1->initialLinVelXYZ_invMassW;

	const float4 rAWorld_linBreakForce = constraintData->mRAWorld_linBreakForce[threadIndex];
	const float4 rBWorld_AngBreakForce = constraintData->mRBWorld_AngBreakForce[threadIndex];
	
	const float linBreakImpulse = rAWorld_linBreakForce.w * totalDt;
	const float angBreakImpulse = rBWorld_AngBreakForce.w * totalDt;
	header->linBreakImpulse[threadIndex] = linBreakImpulse;
	header->angBreakImpulse[threadIndex] = angBreakImpulse;

	const float invMass0 = initialLinVelXYZ_invMassW0.w * lin0_ang0_lin1_ang1.x;
	const float invMass1 = initialLinVelXYZ_invMassW1.w * lin0_ang0_lin1_ang1.z;

	

	header->rAWorld_invMass0D0[threadIndex] = make_float4(rAWorld_linBreakForce.x, rAWorld_linBreakForce.y, rAWorld_linBreakForce.z, invMass0);
	header->rBWorld_invMass1D1[threadIndex] = make_float4(rBWorld_AngBreakForce.x, rBWorld_AngBreakForce.y, rBWorld_AngBreakForce.z, invMass1);
	header->invInertiaScale0[threadIndex] = lin0_ang0_lin1_ang1.y;
	header->invInertiaScale1[threadIndex] = lin0_ang0_lin1_ang1.w;

	rowCounts_breakable_orthoAxisCount.y = PxU8((rAWorld_linBreakForce.w != PX_MAX_F32) || (rBWorld_AngBreakForce.w != PX_MAX_F32));


	for (PxU32 i = 0; i < numRows; ++i)
	{
		if (rowParameters[i].flags[threadIndex] & Px1DConstraintFlag::eANGULAR_CONSTRAINT)
		{
			PxU32 hint = rowParameters[i].solveHint[threadIndex];
			if (hint == PxConstraintSolveHint::eEQUALITY)
				hint = PxConstraintSolveHint::eROTATIONAL_EQUALITY;
			else if (hint == PxConstraintSolveHint::eINEQUALITY)
				hint = PxConstraintSolveHint::eROTATIONAL_INEQUALITY;

			rowParameters[i].solveHint[threadIndex] = hint;
		}
	}


	__shared__ PxU32 sortedRowIndices[NbThreads][Dy::MAX_CONSTRAINT_ROWS];
	__shared__ PxVec3 angSqrtInvInertia0[NbThreads][Dy::MAX_CONSTRAINT_ROWS];
	__shared__ PxVec3 angSqrtInvInertia1[NbThreads][Dy::MAX_CONSTRAINT_ROWS];

	bool disablePreprocessing = !!(constraintData->mFlags[threadIndex] & PxConstraintFlag::eDISABLE_PREPROCESSING);

	preprocessRows(sortedRowIndices[threadIdx.x], constraintData, rowVelocities, rowParameters, angSqrtInvInertia0[threadIdx.x], angSqrtInvInertia1[threadIdx.x], 
		sBodyData0, sBodyData1, txIData0, txIData1, threadIndex, disablePreprocessing);
	rowCounts_breakable_orthoAxisCount.z = setUp1DConstraintBlockTGS(sortedRowIndices[threadIdx.x], constraintData, rowVelocities, rowParameters, angSqrtInvInertia0[threadIdx.x], angSqrtInvInertia1[threadIdx.x],
		*header, rowsCon, dt, recipdt, totalDt, recipTotalDt, biasCoefficient, sBodyData0, sBodyData1, threadIndex, lengthScale, disablePreprocessing);

	header->rowCounts_breakable_orthoAxisCount[threadIndex] = rowCounts_breakable_orthoAxisCount;
}


#endif