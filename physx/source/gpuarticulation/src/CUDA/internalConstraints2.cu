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

#include "DyCpuGpuArticulation.h"
#include "DyFeatherstoneArticulation.h"
#include "DyFeatherstoneArticulationUtils.h"
#include "PxSpatialMatrix.h"
#include "PxgArticulationCoreDesc.h"
#include "PxgArticulationCoreKernelIndices.h"
#include "PxgSolverConstraintDesc.h"
#include "PxgSolverCoreDesc.h"
#include "articulationDynamic.cuh"
#include "articulationImpulseResponse.cuh"
#include "reduction.cuh"
#include "solver.cuh"
#include "solverBlock.cuh"
#include "solverBlockTGS.cuh"
#include "utils.cuh"

using namespace physx;
using namespace Dy;

extern "C" __host__ void initArticulationKernels2() {}

static __device__ Cm::UnAlignedSpatialVector propagateImpulseDofAligned(const Cm::UnAlignedSpatialVector& isInvD, const PxVec3& childToParent,
	const Cm::UnAlignedSpatialVector& sa, const Cm::UnAlignedSpatialVector& Z, PxReal& qstZ)
{
	const PxReal stZ = sa.innerProduct(Z);
	const Cm::UnAlignedSpatialVector temp = isInvD * stZ;
	qstZ = -stZ;

	//parent space's spatial zero acceleration impulse
	return FeatherstoneArticulation::translateSpatialVector(childToParent, (Z - temp));
}

static __device__ void getImpulseSelfResponseDofAligned(
	const Cm::UnAlignedSpatialVector& impulse0,
	const Cm::UnAlignedSpatialVector& impulse1,
	Cm::UnAlignedSpatialVector& deltaV0,
	Cm::UnAlignedSpatialVector& deltaV1,
	const PxgArticulationBlockDofData& thisDof,
	const PxgArticulationBlockDofData* dofData,
	const PxVec3& childToParent,
	const PxSpatialMatrix& parentSpatialResponse,
	const Cm::UnAlignedSpatialVector& thisMotionVector,
	const PxU32 dofCount,
	const PxU32 dofIndex,
	const PxU32 threadIndexInWarp)
{

	Cm::UnAlignedSpatialVector Z1W(-impulse1.top, -impulse1.bottom);

	const Cm::UnAlignedSpatialVector isInvD = loadSpatialVector(thisDof.mIsInvDW, threadIndexInWarp);

	PxReal qstZ[3] = { 0.f, 0.f, 0.f };
	Cm::UnAlignedSpatialVector Z0W = propagateImpulseDofAligned(isInvD, childToParent, thisMotionVector, Z1W, qstZ[dofIndex]);

	const Cm::UnAlignedSpatialVector impulseDifW = impulse0 - Z0W;

	const Cm::UnAlignedSpatialVector deltaV0W = (parentSpatialResponse * impulseDifW);

	deltaV1 = propagateAccelerationW(childToParent, dofData, deltaV0W, dofCount, threadIndexInWarp, qstZ);

	deltaV0 = deltaV0W;
}

static __device__ void getImpulseSelfResponse(
	const Cm::UnAlignedSpatialVector& impulse0,
	const Cm::UnAlignedSpatialVector& impulse1,
	Cm::UnAlignedSpatialVector& deltaV0,
	Cm::UnAlignedSpatialVector& deltaV1,
	const PxgArticulationBlockDofData* dofData,
	const PxVec3& childToParent,
	const PxSpatialMatrix& parentSpatialResponse,
	const PxU32 dofCount,
	const PxU32 threadIndexInWarp)
{

	Cm::UnAlignedSpatialVector Z1W(-impulse1.top, -impulse1.bottom);

	PxReal qstZ[3] = { 0.f, 0.f, 0.f };
	Cm::UnAlignedSpatialVector Z0W = propagateImpulseW_1(childToParent, dofData, Z1W, NULL, dofCount, threadIndexInWarp, qstZ);

	const Cm::UnAlignedSpatialVector impulseDifW = impulse0 - Z0W;

	const Cm::UnAlignedSpatialVector deltaV0W = (parentSpatialResponse * impulseDifW);

	deltaV1 = propagateAccelerationW(childToParent, dofData, deltaV0W, dofCount, threadIndexInWarp, qstZ);

	deltaV0 = deltaV0W;
}

static __device__ void setupInternalConstraints(PxgArticulationBlockData& artiBlock,
	PxgArticulationBlockLinkData* PX_RESTRICT artiLinks,
	PxgArticulationBlockDofData* PX_RESTRICT artiDofs,
	const PxReal stepDt, const PxReal dt, const PxReal invDt, bool isTGSSolver, const PxU32 threadIndexInWarp)
{
	const PxU32 numLinks = artiBlock.mNumLinks[threadIndexInWarp];

	const PxReal maxForceScale = artiBlock.mFlags[threadIndexInWarp] & PxArticulationFlag::eDRIVE_LIMITS_ARE_FORCES ? dt : 1.f;

	//KS - we skip link 0 because it does not have any joints

	PxgArticulationBlockDofData* PX_RESTRICT dofs = artiDofs;
	
	__shared__ char sDriveError[sizeof(PxVec3) * WARP_SIZE];
	PxVec3* driveError = reinterpret_cast<PxVec3*>(sDriveError);


	for (PxU32 linkID = 1; linkID < numLinks; ++linkID)
	{
		PxgArticulationBlockLinkData& link = artiLinks[linkID];

		const PxU32 nbDofs = link.mDofs[threadIndexInWarp];
		const PxU32 parent = link.mParents[threadIndexInWarp];

		PxSpatialMatrix parentResponse;

		PxTransform cA2w;
		PxTransform cB2w;
		const float c2px = link.mRw_x[threadIndexInWarp];
		const float c2py = link.mRw_y[threadIndexInWarp];
		const float c2pz = link.mRw_z[threadIndexInWarp];
		PxVec3 child2Parent(c2px, c2py, c2pz);
		PxReal transmissionForce;
		PxReal cfm;

		bool loaded = false;

		//KS - maxFrictionForce stores the friction coefficient...
		if (nbDofs)
		{	
			PxReal frictionCoefficient = dofs->mConstraintData.mFrictionCoefficient[threadIndexInWarp] * stepDt;

			PxU32 jointType = link.mJointType[threadIndexInWarp];
			const bool isAngularConstraint = jointType == PxArticulationJointType::eREVOLUTE || jointType == PxArticulationJointType::eREVOLUTE_UNWRAPPED || jointType == PxArticulationJointType::eSPHERICAL;

			if (isAngularConstraint)
			{
				if (nbDofs > 1)
				{
					const float4 cRot = link.mAccumulatedPose.q[threadIndexInWarp];
					const float4 pRot = artiLinks[parent].mAccumulatedPose.q[threadIndexInWarp];
					const float4 parentQ = link.mParentPose.q[threadIndexInWarp];
					const float4 childQ = link.mChildPose.q[threadIndexInWarp];

					const PxQuat cA2w = PxQuat(pRot.x, pRot.y, pRot.z, pRot.w) * PxQuat(parentQ.x, parentQ.y, parentQ.z, parentQ.w);
					const PxQuat cB2w = PxQuat(cRot.x, cRot.y, cRot.z, cRot.w) * PxQuat(childQ.x, childQ.y, childQ.z, childQ.w);

					PxQuat qB2qA = cA2w.getConjugate() * cB2w;

					PxVec3 driveAxis(0.f);

					bool hasAngularDrives = false;

					for (PxU32 i = 0; i < nbDofs; ++i)
					{
						bool hasDrive = dofs[i].mMotion[threadIndexInWarp] != PxArticulationMotion::eLOCKED &&
							dofs[i].mConstraintData.mDriveType[threadIndexInWarp] != PxArticulationDriveType::eNONE;

						if (hasDrive)
						{
							float4 top = dofs[i].mLocalMotionMatrix.mTopxyz_bx[threadIndexInWarp];
							PxReal targetPos = dofs[i].mConstraintData.mDriveTargetPos[threadIndexInWarp];
							driveAxis += PxVec3(top.x, top.y, top.z)*targetPos;
							hasAngularDrives = true;
						}
					}

					if (hasAngularDrives)
					{
						PxReal angle = driveAxis.normalize();

						if (angle < 1e-12f)
						{
							driveAxis = PxVec3(1.f, 0.f, 0.f);
							angle = 0.f;
						}

						PxQuat targetQ = PxQuat(angle, driveAxis);

						if (targetQ.dot(qB2qA) < 0.f)
							targetQ = -targetQ;

						driveError[threadIndexInWarp] = -2.f * (targetQ.getConjugate() * qB2qA).getImaginaryPart();
					}
				}
				else
				{
					if (dofs[0].mMotion[threadIndexInWarp] != PxArticulationMotion::eLOCKED)
					{
						PxU32 i = dofs[0].mDofIds[threadIndexInWarp];
						const PxReal jointPos = dofs[0].mJointPositions[threadIndexInWarp];
						driveError[threadIndexInWarp][i] = dofs[0].mConstraintData.mDriveTargetPos[threadIndexInWarp] - jointPos;
					}
				}
			}

			for (PxU32 i = 0; i < nbDofs; ++i)
			{
				PxU32 motion = dofs[i].mMotion[threadIndexInWarp];

				PxU32 dofId = dofs[i].mDofIds[threadIndexInWarp];
				{
					const PxReal maxForce = dofs[i].mConstraintData.mMaxForce[threadIndexInWarp];

					PxReal stiffness = dofs[i].mConstraintData.mDriveStiffness[threadIndexInWarp];
					PxReal damping = dofs[i].mConstraintData.mDamping[threadIndexInWarp];

					if (!loaded)
					{
						loadSpatialMatrix(artiLinks[parent].mSpatialResponseMatrix, threadIndexInWarp, parentResponse);
						Cm::UnAlignedSpatialVector bias = loadSpatialVector(link.mBiasForce, threadIndexInWarp);
						transmissionForce = bias.magnitude() * frictionCoefficient;
						cfm = PxMax(artiLinks[parent].mCfm[threadIndexInWarp], link.mCfm[threadIndexInWarp]);
						loaded = true;
					}

					//Generate the response vectors...

					Cm::UnAlignedSpatialVector worldMotionVector;

					Cm::UnAlignedSpatialVector axis0, axis1;
					bool isLinear = dofId >= PxArticulationAxis::eX;
					PxReal position = dofs[i].mJointPositions[threadIndexInWarp];
					PxReal error;
					if (!isLinear)
					{
						worldMotionVector = loadSpatialVector(dofs[i].mWorldMotionMatrix, threadIndexInWarp);
						//Angular constraint...
						axis0.top = PxVec3(0.f);
						axis0.bottom = worldMotionVector.top;
						axis1.top = PxVec3(0.f);
						axis1.bottom = -worldMotionVector.top;
						error = driveError[threadIndexInWarp][dofId];
					}
					else
					{
						worldMotionVector = loadSpatialVector(dofs[i].mWorldMotionMatrix, threadIndexInWarp);
						//KS - the following are only required for linear constraints or lockes axes...
						const float4 cRot = link.mAccumulatedPose.q[threadIndexInWarp];
						const float4 pRot = artiLinks[parent].mAccumulatedPose.q[threadIndexInWarp];

						const float4 parentP = link.mParentPose.p[threadIndexInWarp];
						const float4 childP = link.mChildPose.p[threadIndexInWarp];
						const PxVec3 cA2w = PxQuat(pRot.x, pRot.y, pRot.z, pRot.w).rotate(PxVec3(parentP.x, parentP.y, parentP.z));
						const PxVec3 cB2w = PxQuat(cRot.x, cRot.y, cRot.z, cRot.w).rotate(PxVec3(childP.x, childP.y, childP.z));

						const PxVec3 axis = worldMotionVector.bottom;
						const PxVec3 ang0 = cA2w.cross(axis);
						const PxVec3 ang1 = cB2w.cross(axis);

						axis0.top = axis;
						axis0.bottom = ang0;
						axis1.top = -axis;
						axis1.bottom = -ang1;

						error = dofs[0].mConstraintData.mDriveTargetPos[threadIndexInWarp] - position;
					}

					Cm::UnAlignedSpatialVector deltaVA, deltaVB;

					getImpulseSelfResponseDofAligned(axis0, axis1,
						deltaVA, deltaVB, dofs[i], dofs, child2Parent, parentResponse,
						worldMotionVector, nbDofs, i, threadIndexInWarp);

					/*getImpulseSelfResponse(artiLinks[parent], link, axis0, axis1, deltaVA, deltaVB, dofs, child2Parent, parentResponse,
						childInertia, nbDofs, threadIndexInWarp);*/

					const PxReal r0 = deltaVA.innerProduct(axis0);
					const PxReal r1 = deltaVB.innerProduct(axis1);

					const PxReal unitResponse = r1 + r0;

					const PxReal recipResponse = 1.0f / (unitResponse + cfm);

					dofs[i].mConstraintData.mMaxFrictionForce[threadIndexInWarp] = transmissionForce;

					//Set up drives...
					if (motion == PxArticulationMotion::eLIMITED)
					{
						float2 limit = dofs[i].mConstraintData.mLimits_LowLimitX_highLimitY[threadIndexInWarp];

						dofs[i].mConstraintData.mLimitError_LowX_highY[threadIndexInWarp] = make_float2(position - limit.x, limit.y - position);
					}

					//Set up drive...
					if (maxForce > 0.f && (stiffness > 0.f || damping > 0.f))
					{
						const PxReal targetVelocity = dofs[i].mConstraintData.mDriveTargetVel[threadIndexInWarp];
						const PxArticulationDriveType::Enum type = PxArticulationDriveType::Enum(dofs[i].mConstraintData.mDriveType[threadIndexInWarp]);
						dofs[i].mConstraintData.setImplicitDriveDesc(threadIndexInWarp,
							computeImplicitDriveParams(
									type, stiffness, damping,
									isTGSSolver ? stepDt : dt, dt,
									unitResponse, recipResponse,
									error, targetVelocity,
									isTGSSolver));
						dofs[i].mConstraintData.mConstraintMaxForce[threadIndexInWarp] = maxForce * maxForceScale;
					}
					else
					{
						//Zero drives...
						dofs[i].mConstraintData.setImplicitDriveDesc(threadIndexInWarp, ArticulationImplicitDriveDesc(PxZero));
						dofs[i].mConstraintData.mConstraintMaxForce[threadIndexInWarp] = 0.f;
					}
					dofs[i].mConstraintData.mDriveForce[threadIndexInWarp] = 0.f;
					storeSpatialVector(dofs[i].mConstraintData.mDeltaVA, deltaVA, threadIndexInWarp);
					storeSpatialVector(dofs[i].mConstraintData.mDeltaVB, deltaVB, threadIndexInWarp);
					storeSpatialVector(dofs[i].mConstraintData.mRow0, axis0, threadIndexInWarp);
					storeSpatialVector(dofs[i].mConstraintData.mRow1, -axis1, threadIndexInWarp);
					dofs[i].mConstraintData.mRecipResponse[threadIndexInWarp] = recipResponse;
					dofs[i].mConstraintData.mResponse[threadIndexInWarp] = unitResponse;
					dofs[i].mConstraintData.mAccumulatedFrictionImpulse[threadIndexInWarp] = 0.0f;
					dofs[i].mConstraintData.mLowImpulse[threadIndexInWarp] = 0.f;
					dofs[i].mConstraintData.mHighImpulse[threadIndexInWarp] = 0.f;
				}
			}
		}
		dofs += nbDofs;
	}
}

static __device__ void loadSpatialVectorsForPropagationInwardsAndOutwards
(const PxgArticulationBlockDofData* PX_RESTRICT artiDofs, const PxU32 jointOffset, const PxU32 nbDofs, const PxU32 threadIndexInWarp,
 Cm::UnAlignedSpatialVector* PX_RESTRICT motionMatrixW, Cm::SpatialVectorF* PX_RESTRICT IsInvSTISW, Cm::SpatialVectorF* PX_RESTRICT ISW, InvStIs& invSTISW)
{
	//Gather some important terms in the form we need.
	//Note propagateAccelerationW cpu/gpu computes jAccel[i] = invSTISW[0][i] * j[0] + invSTISW[1][i] * j[1] + invSTISW[2][i] * j[2]. 
	//Note 	propagateAccelerationWNoJVelUpdate computes jAccel[i] = invStIsT[i].x * j[0] + invStIsT[i].y * j[1] + invStIsT[i].z * j[2]
	//We can conclude:  invStIsT[i].x has index [0][i], invStIsT[i].y has index [1][i] and invStIsT[i].z has index [2][1]
	for(PxU32 i = 0; i < nbDofs; i++)
	{
		motionMatrixW[i] = loadSpatialVector(artiDofs[jointOffset + i].mWorldMotionMatrix, threadIndexInWarp);
		IsInvSTISW[i] = loadSpatialVectorF(artiDofs[jointOffset + i].mIsInvDW, threadIndexInWarp);
		invSTISW.invStIs[0][i] = artiDofs[jointOffset + i].mInvStIsT_x[threadIndexInWarp];
		invSTISW.invStIs[1][i] = artiDofs[jointOffset + i].mInvStIsT_y[threadIndexInWarp];
		invSTISW.invStIs[2][i] = artiDofs[jointOffset + i].mInvStIsT_z[threadIndexInWarp];
		ISW[i] = loadSpatialVectorF(artiDofs[jointOffset + i].mIsW, threadIndexInWarp);
	}
} 

static __device__ void loadSpatialVectorsForPropagationInwards
(const PxgArticulationBlockDofData* PX_RESTRICT artiDofs, const PxU32 jointOffset, const PxU32 nbDofs, const PxU32 threadIndexInWarp,
 Cm::UnAlignedSpatialVector* PX_RESTRICT motionMatrixW, Cm::SpatialVectorF* IsInvSTISW)
{
	for(PxU32 i = 0; i < nbDofs; i++)
	{
		motionMatrixW[i] = loadSpatialVector(artiDofs[jointOffset + i].mWorldMotionMatrix, threadIndexInWarp);
		IsInvSTISW[i] = loadSpatialVectorF(artiDofs[jointOffset + i].mIsInvDW, threadIndexInWarp);
	}
}

static __device__ void loadSpatialVectorsForPropagationOutwards
(const PxgArticulationBlockDofData* PX_RESTRICT artiDofs, const PxU32 jointOffset, const PxU32 nbDofs, const PxU32 threadIndexInWarp,
 Cm::UnAlignedSpatialVector* PX_RESTRICT motionMatrixW, Cm::SpatialVectorF* ISW, InvStIs& invSTISW)
{
	//Note propagateAccelerationW cpu/gpu computes jAccel[i] = invSTISW[0][i] * j[0] + invSTISW[1][i] * j[1] + invSTISW[2][i] * j[2]. 
	//Note 	propagateAccelerationWNoJVelUpdate computes jAccel[i] = invStIsT[i].x * j[0] + invStIsT[i].y * j[1] + invStIsT[i].z * j[2]
	//We can conclude:  invStIsT[i].x has index [0][i], invStIsT[i].y has index [1][i] and invStIsT[i].z has index [2][1]
	for(PxU32 i = 0; i < nbDofs; i++)
	{
		motionMatrixW[i] = loadSpatialVector(artiDofs[jointOffset + i].mWorldMotionMatrix, threadIndexInWarp);
		invSTISW.invStIs[0][i] = artiDofs[jointOffset + i].mInvStIsT_x[threadIndexInWarp];
		invSTISW.invStIs[1][i] = artiDofs[jointOffset + i].mInvStIsT_y[threadIndexInWarp];
		invSTISW.invStIs[2][i] = artiDofs[jointOffset + i].mInvStIsT_z[threadIndexInWarp];
		ISW[i] = loadSpatialVectorF(artiDofs[jointOffset + i].mIsW, threadIndexInWarp);
	}
}

/**
\brief Compute the deltaQDot response of a joint dof to a unit joint impulse applied to that joint dof.
\param[in] linkIndex specifies the index of the child link of the joint under consideration.
\param[in] dof is the joint dof that will receive the test impulse.
\param[in] artiLinks has pre-computed values that will ber used in the computation.
\param[in] artiDofs has pre-computed values that will be used in the computation.
\param[in] threadIndexInWarp is an index in range (0,32) describing the index inside a warp of 32 threads.
\return The deltaQDot response of the specified joint and dof of a test joint impulse applied to the specified joint and dof.
\note dof is in range (0,3) because articulation joints only support 3 degrees of freedom.
*/
static __device__ PxReal computeMimicJointSelfResponse
(const PxU32 linkIndex, const PxU32 dof, const PxgArticulationBlockLinkData* PX_RESTRICT artiLinks, const PxgArticulationBlockDofData* PX_RESTRICT artiDofs, const PxU32 threadIndexInWarp)
{			
	const PxU32 parentLinkIndex = artiLinks[linkIndex].mParents[threadIndexInWarp];

	//childLinkPos - parentLinkPos
	const float parentLinkToChildLinkx = artiLinks[linkIndex].mRw_x[threadIndexInWarp];
	const float parentLinkToChildLinky = artiLinks[linkIndex].mRw_y[threadIndexInWarp];
	const float parentLinkToChildLinkz = artiLinks[linkIndex].mRw_z[threadIndexInWarp];

	const PxU32 jointOffset = artiLinks[linkIndex].mJointOffset[threadIndexInWarp];
	const PxU8 dofCount = artiLinks[linkIndex].mDofs[threadIndexInWarp];

	const PxReal testJointImpulses[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	const PxReal* testJointImpulse = testJointImpulses[dof];

	//Gather terms from the gpu data layout.
	Cm::UnAlignedSpatialVector motionMatrixW[3];
	Cm::SpatialVectorF IsInvSTISW[3];
	Cm::SpatialVectorF ISW[3];
	InvStIs invSTISW;
	loadSpatialVectorsForPropagationInwardsAndOutwards(artiDofs, jointOffset, dofCount, threadIndexInWarp, motionMatrixW, IsInvSTISW, ISW, invSTISW);

	//(1) Propagate joint impulse (and zero link impulse) to parent
	PxReal QMinusStZ[3] = { 0.f, 0.f, 0.f };
	const Cm::SpatialVectorF Zp = propagateImpulseW(
		PxVec3(parentLinkToChildLinkx, parentLinkToChildLinky, parentLinkToChildLinkz),
		Cm::SpatialVectorF(PxVec3(0, 0, 0), PxVec3(0, 0, 0)), 
		testJointImpulse, IsInvSTISW, motionMatrixW, dofCount,
		QMinusStZ);

	//(2) Get deltaV response for parent
	 Cm::SpatialVectorF deltaVParent;
	{
		PxSpatialMatrix mat;
		loadSpatialMatrix(artiLinks[parentLinkIndex].mSpatialResponseMatrix, threadIndexInWarp, mat);
		const Cm::UnAlignedSpatialVector ZpUnaligned(Zp.top, Zp.bottom);
		const Cm::UnAlignedSpatialVector deltaVParentUnagligned = mat * (-ZpUnaligned);
		deltaVParent = Cm::SpatialVectorF(deltaVParentUnagligned.top, deltaVParentUnagligned.bottom);
	}
		
	//(3) Propagate parent deltaV and apply test impulse (encoded in QMinusStZ).
	PxReal jointDeltaQDot[3]= {0, 0, 0};
	propagateAccelerationW(
		PxVec3(parentLinkToChildLinkx, parentLinkToChildLinky, parentLinkToChildLinkz), deltaVParent, 
		invSTISW, motionMatrixW, ISW, QMinusStZ, dofCount, 
		jointDeltaQDot);

	const PxReal jointSelfResponse = jointDeltaQDot[dof];
	return jointSelfResponse;
}

/**
\brief Compute the deltaQDot response of a joint dof given a unit impulse applied to a different joint and dof.
\param[in] linkA is the link whose inbound joint receives the test impulse.
\param[in] dofA is the relevant dof of the inbound joint of linkA.
\param[in] linkB is the link whose inbound joint receives the deltaQDot arising from the unit impulse applied to the inbound joint of linkA.
\param[in] dofB is the relevant dof of the the inbound joint of linkB.
\param[in] artiLinks has pre-computed values that will ber used in the computation.
\param[in] artiDofs has pre-computed values that will be used in the computation.
\param[in] artiPathToRootBitFields stores the bitfields describing the path to root for each link of an articulation.
\param[in] artiPathToRootBitFieldWordCount is the number of bitfields (with bitfield==PxU64) required to store the path to root for a link.
\param[in] threadIndexInWarp is an index in range (0,32) describing the index inside a warp of 32 threads.
\return The deltaQDot response of the specified joint and dof corresponding to linkB and dofB.
\note dofA and dofB are in range (0,3) because articulation joints only support 3 degrees of freedom.
\note artiDofs is not const because we cache temporary data in QMinusSTZ owned by artiDofs. 
The data will be cleared at the end of the computation because the buffer is used later in the solver
to accumulate link impulses from contact and constraint.
*/
static __device__ PxReal computeMimicJointCrossResponse
(const PxU32 linkA, const PxU32 dofA, const PxU32 linkB, const PxU32 dofB, 
 const PxgArticulationBlockData& artiBlock, 
 const PxgArticulationBlockLinkData* PX_RESTRICT artiLinks, 
 PxgArticulationBlockDofData* PX_RESTRICT artiDofs, 
 const PxgArticulationBitFieldData* PX_RESTRICT artiPathToRootBitFields, const PxU32 artiPathToRootBitFieldWordCount,
 const PxU32 threadIndexInWarp)
{
	//Compute the test impulse to apply the inbound joint of linkA.
	const PxReal testJointImpulses[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

	//Zero QMinusSTZ before using it.
	for(PxU32 i = 0; i < artiBlock.mTotalDofs[threadIndexInWarp]; i++)
		artiDofs[i].mDeferredQstZ[threadIndexInWarp] = 0.0f;

	//Iterate from linkA to root.
	Cm::SpatialVectorF Zp;
	for (PxI32 j = artiPathToRootBitFieldWordCount-1, bitOffset = (artiPathToRootBitFieldWordCount-1)*64; j >= 0; j--, bitOffset -= 64)
	{
		ArticulationBitField word = artiPathToRootBitFields[linkA*artiPathToRootBitFieldWordCount + j].bitField[threadIndexInWarp];
		while (word)
		{
			const PxU32 bitIndex = articulationHighestSetBit(word);
			const PxU32 linkIndex = bitIndex + bitOffset;
			word &= (~(1ull << bitIndex)); //Clear this bit

			//The root is included in pathToRoot but we cannot propagate an impulse upwards from the root.			
			if(0 != linkIndex)
			{
				const PxReal* jointImpulse = NULL;
				Cm::SpatialVectorF linkImpulse(PxVec3(0,0,0), PxVec3(0,0,0));
				if(linkA == linkIndex)
				{
					//Propagate joint impulse to parent
					jointImpulse = testJointImpulses[dofA];
					linkImpulse = Cm::SpatialVectorF(PxVec3(0,0,0), PxVec3(0,0,0));
				}
				else
				{
					//Propagate link impulse to parent.
					jointImpulse = NULL;
					linkImpulse = Zp;
				}

				//childLinkPos - parentLinkPos
				const float parentLinkToChildLinkx = artiLinks[linkIndex].mRw_x[threadIndexInWarp];
				const float parentLinkToChildLinky = artiLinks[linkIndex].mRw_y[threadIndexInWarp];
				const float parentLinkToChildLinkz = artiLinks[linkIndex].mRw_z[threadIndexInWarp];

				const PxU32 jointOffset = artiLinks[linkIndex].mJointOffset[threadIndexInWarp];
				const PxU8 dofCount = artiLinks[linkIndex].mDofs[threadIndexInWarp];

				Cm::UnAlignedSpatialVector motionMatrixW[3];
				Cm::SpatialVectorF IsInvSTISW[3];
				loadSpatialVectorsForPropagationInwards(artiDofs, jointOffset, dofCount, threadIndexInWarp, motionMatrixW, IsInvSTISW);

				PxReal QMinusSTZ[3] = {0, 0, 0};
				Zp = propagateImpulseW(
						PxVec3(parentLinkToChildLinkx, parentLinkToChildLinky, parentLinkToChildLinkz),
						linkImpulse,
						jointImpulse, IsInvSTISW, motionMatrixW, dofCount, 
						QMinusSTZ);

				//Copy QMinusSTZ to persistent array so we can use it again when we propagate deltaV downwards.
				for(PxU32 i = 0; i < dofCount; i++)
					artiDofs[jointOffset + i].mDeferredQstZ[threadIndexInWarp] = QMinusSTZ[i];		
			}	
		}			
	}

	//(2) Get deltaV response for root
	 Cm::SpatialVectorF deltaVRoot;
	{
		PxSpatialMatrix mat;
		loadSpatialMatrix(artiLinks[0].mSpatialResponseMatrix, threadIndexInWarp, mat);
		const Cm::UnAlignedSpatialVector ZpUnaligned(Zp.top, Zp.bottom);
		const Cm::UnAlignedSpatialVector deltaVRootUnagligned = mat * (-ZpUnaligned);
		deltaVRoot = Cm::SpatialVectorF(deltaVRootUnagligned.top, deltaVRootUnagligned.bottom);
	}

	//Propagate deltaVRoot from root to linkB.
	PxReal jointVelocity[3] = {0, 0, 0};
	Cm::SpatialVectorF deltaVParent = deltaVRoot;
	for (PxI32 j = 0, bitOffset = 0; j < artiPathToRootBitFieldWordCount; j++, bitOffset += 64)
	{
		ArticulationBitField word = artiPathToRootBitFields[linkB*artiPathToRootBitFieldWordCount + j].bitField[threadIndexInWarp];
		while (word)
		{
			const PxU32 bitIndex = articulationLowestSetBit(word);
			const PxU32 linkIndex = bitIndex + bitOffset;
			word &= (~(1ull << bitIndex)); //Clear this bit

			//The root is included in pathToRoot but we cannot propagate an impulse upwards from the root.			
			if(linkIndex != 0)
			{
				//childLinkPos - parentLinkPos
				const float parentLinkToChildLinkx = artiLinks[linkIndex].mRw_x[threadIndexInWarp];
				const float parentLinkToChildLinky = artiLinks[linkIndex].mRw_y[threadIndexInWarp];
				const float parentLinkToChildLinkz = artiLinks[linkIndex].mRw_z[threadIndexInWarp];

				const PxU32 jointOffset = artiLinks[linkIndex].mJointOffset[threadIndexInWarp];
				const PxU8 dofCount = artiLinks[linkIndex].mDofs[threadIndexInWarp];

				Cm::UnAlignedSpatialVector motionMatrixW[3];
				Cm::SpatialVectorF ISW[3];
				InvStIs invSTISW;
				loadSpatialVectorsForPropagationOutwards(artiDofs, jointOffset, dofCount, threadIndexInWarp, motionMatrixW, ISW, invSTISW);

				//Load the persistent QMinusSTZ that we cached when propagating upwards.
				PxReal QMinusSTZ[3] = {0, 0, 0};
				for(PxU32 i = 0; i < dofCount; i++)
				{
					QMinusSTZ[i] = artiDofs[jointOffset + i].mDeferredQstZ[threadIndexInWarp];
				}

				PxReal childJointSpeed[3] = {0, 0, 0};
				deltaVParent = propagateAccelerationW(
					PxVec3(parentLinkToChildLinkx, parentLinkToChildLinky, parentLinkToChildLinkz), deltaVParent, 
					invSTISW, motionMatrixW, ISW, QMinusSTZ, dofCount, 
					childJointSpeed); 

				jointVelocity[0] = childJointSpeed[0];
				jointVelocity[1] = childJointSpeed[1];
				jointVelocity[2] = childJointSpeed[2];
			}
		}
	}

	//Zero QMinusSTZ before exiting.
	for(PxU32 i = 0; i < artiBlock.mTotalDofs[threadIndexInWarp]; i++)
		artiDofs[i].mDeferredQstZ[threadIndexInWarp] = 0.0f;

	//Now pick out the dof associated with joint B.
	const PxReal r = jointVelocity[dofB];
	return r;
}

static  __device__ void setupInternalMimicJointConstraints
(const PxgArticulationBlockData& artiBlock, 
 const PxgArticulationBlockLinkData* PX_RESTRICT artiLinks, 
 const PxgArticulationBitFieldData* PX_RESTRICT artiPathToRootBitFields, const PxU32 artiPathToRootBitFieldWordCount,
 PxgArticulationBlockDofData* PX_RESTRICT artiDofs,
 PxgArticulationBlockMimicJointData* PX_RESTRICT mimicJoints, 
 const PxU32 threadIndexInWarp)
{
	const PxU32 nbMimicJoints = artiBlock.mNumMimicJoints[threadIndexInWarp];
	for(PxU32 i = 0; i < nbMimicJoints; i++)
	{
		//The coupled joints are the inbound joints of link0 and link1.
		const PxU32 linkA = mimicJoints[i].mLinkA[threadIndexInWarp];
		const PxU32 linkB = mimicJoints[i].mLinkB[threadIndexInWarp];

		//Store dofA and dofB
		const PxU32 axisA = mimicJoints[i].mAxisA[threadIndexInWarp];
		const PxU32 axisB = mimicJoints[i].mAxisB[threadIndexInWarp];
		const PxU32 dofA = artiLinks[linkA].mInvDofIds[axisA][threadIndexInWarp];
		const PxU32 dofB = artiLinks[linkB].mInvDofIds[axisB][threadIndexInWarp];
		mimicJoints[i].mInternalData.mDofA[threadIndexInWarp] = dofA;
		mimicJoints[i].mInternalData.mDofB[threadIndexInWarp] = dofB;

		//Compute all 4 response terms.	
		const PxReal rAA = computeMimicJointSelfResponse(linkA, dofA, artiLinks, artiDofs, threadIndexInWarp);
		const PxReal rBB = computeMimicJointSelfResponse(linkB, dofB, artiLinks, artiDofs, threadIndexInWarp);
		const PxReal rBA = computeMimicJointCrossResponse(
			linkA, dofA, linkB, dofB,
			artiBlock, artiLinks, artiDofs, artiPathToRootBitFields, artiPathToRootBitFieldWordCount,
			threadIndexInWarp);
		const PxReal rAB = computeMimicJointCrossResponse(
			linkB, dofB, linkA, dofA,
			artiBlock, artiLinks, artiDofs, artiPathToRootBitFields, artiPathToRootBitFieldWordCount,
			threadIndexInWarp);

		const PxReal gearRatio = mimicJoints[i].mGearRatio[threadIndexInWarp];
		mimicJoints[i].mInternalData.recipEffectiveInertia[threadIndexInWarp] = computeRecipMimicJointEffectiveInertia(rAA, rAB, rBB, rBA, gearRatio);
	}
}


static __device__ void getImpulseSelfResponseSlow(
	const PxU32 linkID0_,
	const PxU32 linkID1_,
	const Cm::UnAlignedSpatialVector& impulse0,
	const Cm::UnAlignedSpatialVector& impulse1,
	Cm::UnAlignedSpatialVector& deltaV0,
	Cm::UnAlignedSpatialVector& deltaV1,
	const PxgArticulationBlockData& artiBlock,
	const PxgArticulationBlockLinkData* linkData,
	PxgArticulationBlockDofData* dofData,
	const PxU32 threadIndexInWarp)
{

	PxU32 stack[DY_ARTICULATION_TENDON_MAX_SIZE];

	PxU32 linkID0 = linkID0_;
	PxU32 linkID1 = linkID1_;

	PxU32 i0, i1;

	for (i0 = linkID0, i1 = linkID1; i0 != i1;)	// find common path
	{
		if (i0 < i1)
			i1 = linkData[i1].mParents[threadIndexInWarp];
		else
			i0 = linkData[i0].mParents[threadIndexInWarp];
	}

	PxU32 common = i0;

	Cm::UnAlignedSpatialVector Z0(-impulse0.top, -impulse0.bottom);
	Cm::UnAlignedSpatialVector Z1(-impulse1.top, -impulse1.bottom);

	//initialize tmp qstz to be zero
	const PxU32 numLinks = artiBlock.mNumLinks[threadIndexInWarp];

	for (PxU32 i = 0; i < numLinks; ++i)
	{
		const PxU32 jointOffset = linkData[i].mJointOffset[threadIndexInWarp];
		const PxU32 dofCount = linkData[i].mDofs[threadIndexInWarp];
		PxgArticulationBlockDofData* curDofData = &dofData[jointOffset]; 
		for (PxU32 j = 0; j < dofCount; ++j)
		{
			curDofData[j].mTmpQstZ[threadIndexInWarp] = 0.f;
		}
	}

	for (i0 = 0; linkID0 != common; linkID0 = linkData[linkID0].mParents[threadIndexInWarp])
	{
		
		const PxU32 jointOffset = linkData[linkID0].mJointOffset[threadIndexInWarp];
		const PxU32 dofCount = linkData[linkID0].mDofs[threadIndexInWarp];

		const float rwx = linkData[linkID0].mRw_x[threadIndexInWarp];
		const float rwy = linkData[linkID0].mRw_y[threadIndexInWarp];
		const float rwz = linkData[linkID0].mRw_z[threadIndexInWarp];

		const PxVec3 childToParent(rwx, rwy, rwz);
	
		Z0 = propagateImpulseWTemp(childToParent, dofData + jointOffset, Z0, dofCount, threadIndexInWarp);

		stack[i0++] = linkID0;
	}

	for (i1 = i0; linkID1 != common; linkID1 = linkData[linkID1].mParents[threadIndexInWarp])
	{
		const PxU32 jointOffset = linkData[linkID1].mJointOffset[threadIndexInWarp];
		const PxU32 dofCount = linkData[linkID1].mDofs[threadIndexInWarp];

		const float rwx = linkData[linkID1].mRw_x[threadIndexInWarp];
		const float rwy = linkData[linkID1].mRw_y[threadIndexInWarp];
		const float rwz = linkData[linkID1].mRw_z[threadIndexInWarp];

		const PxVec3 childToParent(rwx, rwy, rwz);

		Z1 = propagateImpulseWTemp(childToParent, dofData + jointOffset, Z1, dofCount, threadIndexInWarp);
		
		stack[i1++] = linkID1;
	}

	Cm::UnAlignedSpatialVector ZZ = Z0 + Z1;

	PxSpatialMatrix spatialResponse;

	loadSpatialMatrix(linkData[common].mSpatialResponseMatrix, threadIndexInWarp, spatialResponse);

	const Cm::UnAlignedSpatialVector v = (spatialResponse * (-ZZ));

	Cm::UnAlignedSpatialVector dv1 = v;
	for (PxU32 index = i1; (index--) > i0;)
	{
		//Dy::ArticulationLinkData& tLinkDatum = data.getLinkData(stack[index]);
		const PxU32 id = stack[index];
		const PxU32 jointOffset = linkData[id].mJointOffset[threadIndexInWarp];
		const PxU32 dofCount = linkData[id].mDofs[threadIndexInWarp];

		const float rwx = linkData[id].mRw_x[threadIndexInWarp];
		const float rwy = linkData[id].mRw_y[threadIndexInWarp];
		const float rwz = linkData[id].mRw_z[threadIndexInWarp];

		const PxVec3 childToParent(rwx, rwy, rwz);

		dv1 = propagateAccelerationWTemp(childToParent, dofData + jointOffset, dv1, dofCount, threadIndexInWarp);
	}

	Cm::UnAlignedSpatialVector dv0 = v;
	for (PxU32 index = i0; (index--) > 0;)
	{
		const PxU32 id = stack[index];
		const PxU32 jointOffset = linkData[id].mJointOffset[threadIndexInWarp];
		const PxU32 dofCount = linkData[id].mDofs[threadIndexInWarp];

		const float rwx = linkData[id].mRw_x[threadIndexInWarp];
		const float rwy = linkData[id].mRw_y[threadIndexInWarp];
		const float rwz = linkData[id].mRw_z[threadIndexInWarp];

		const PxVec3 childToParent(rwx, rwy, rwz);

		dv0 = propagateAccelerationWTemp(childToParent, dofData + jointOffset, dv0, dofCount, threadIndexInWarp);
	}

	deltaV0.bottom = dv0.bottom;
	deltaV0.top = dv0.top;

	deltaV1.bottom = dv1.bottom;
	deltaV1.top = dv1.top;

}

static __device__ void getImpulseSelfResponse(
	const PxU32 linkId0, 
	const PxU32 linkId1,
	PxgArticulationBlockLinkData& link0,
	PxgArticulationBlockLinkData& link1,
	const Cm::UnAlignedSpatialVector& impulse0,
	const Cm::UnAlignedSpatialVector& impulse1,
	Cm::UnAlignedSpatialVector& deltaV0,
	Cm::UnAlignedSpatialVector& deltaV1,
	const PxgArticulationBlockData& artiBlock,
	const PxgArticulationBlockLinkData* linkData,
	PxgArticulationBlockDofData* dofData,
	const PxU32 threadIndexInWarp)
{
	if (link1.mParents[threadIndexInWarp] == linkId0)
	{
		PxSpatialMatrix parentSpatialResponse;

		loadSpatialMatrix(link0.mSpatialResponseMatrix, threadIndexInWarp, parentSpatialResponse);

		const PxU32 jointOffset = link1.mJointOffset[threadIndexInWarp];
		const PxU32 dofCount = link1.mDofs[threadIndexInWarp];

		const float rwx = link1.mRw_x[threadIndexInWarp];
		const float rwy = link1.mRw_y[threadIndexInWarp];
		const float rwz = link1.mRw_z[threadIndexInWarp];

		const PxVec3 childToParent(rwx, rwy, rwz);

		getImpulseSelfResponse(impulse0, impulse1, deltaV0, deltaV1, dofData + jointOffset, childToParent, parentSpatialResponse, dofCount, threadIndexInWarp);
	}
	else
	{
		getImpulseSelfResponseSlow(linkId0, linkId1, impulse0, impulse1, deltaV0, deltaV1, artiBlock, linkData, dofData, threadIndexInWarp);
	}
}

static __device__ void setupInternalSpatialTendonConstraints(
	PxgArticulationBlockData& artiBlock,
	PxgArticulationBlockLinkData* PX_RESTRICT artiLinks,
	PxgArticulationBlockDofData* PX_RESTRICT artiDofs,
	PxgArticulationBlockSpatialTendonData* PX_RESTRICT artiTendons,
	PxgArticulationInternalTendonConstraintData* PX_RESTRICT artiTendonConstraints,
	PxgArticulationBlockAttachmentData* PX_RESTRICT artiAttachments,
	const PxU32 maxAttachments, const PxReal stepDt, const PxReal dt, 
	const PxReal invDt, bool isTGSSolver, const PxU32 threadIndexInWarp)
{
	const PxU32 numTendons = artiBlock.mNumSpatialTendons[threadIndexInWarp];


	PxReal accumLength[DY_ARTICULATION_TENDON_MAX_SIZE];

	for (PxU32 i = 0; i < numTendons; ++i)
	{
		PxgArticulationBlockSpatialTendonData& tendonData = artiTendons[i];
		const PxReal stiffness = tendonData.mStiffness[threadIndexInWarp];
		const PxReal damping = tendonData.mDamping[threadIndexInWarp];
		const PxReal limitStiffness = tendonData.mLimitStiffness[threadIndexInWarp];
		const PxReal offset = tendonData.mOffset[threadIndexInWarp];

		PxgArticulationBlockAttachmentData* attachmentBlock = &artiAttachments[i * maxAttachments];
		PxgArticulationInternalTendonConstraintData* constraintBlock = &artiTendonConstraints[i * maxAttachments];

		const PxReal coefficient = attachmentBlock[0].mCoefficient[threadIndexInWarp];
		PxU64 bitStack = attachmentBlock[0].mChildrens[threadIndexInWarp];
		PxU32 stackCount = __popcll(bitStack);

		PxU32 parent = 0;

		PxU32 numConstraints = 0;

		accumLength[parent] = offset * coefficient;

		PxgArticulationBlockAttachmentData& rAttachmentData = attachmentBlock[parent];
		const PxU32 rAttachmentLinkIndex = rAttachmentData.mLinkIndex[threadIndexInWarp];
		PxgArticulationBlockLinkData& rAttachmentLink = artiLinks[rAttachmentLinkIndex];
		const PxTransform rAttachmentLinkBody2World = loadSpatialTransform(rAttachmentLink.mAccumulatedPose, threadIndexInWarp);
		const PxVec3 rRa = rAttachmentLinkBody2World.q.rotate(rAttachmentData.mRelativeOffset[threadIndexInWarp]);
		const PxVec3 rAttachPoint = rAttachmentLinkBody2World.p + rRa;
		PxVec3 rAxis, rRaXn;

		PxU32 child = 63 - __clzll(bitStack);

		while (stackCount != 0)
		{
			stackCount--;

			PxgArticulationBlockAttachmentData& attachmentData = attachmentBlock[child];
			
			const PxU32 linkInd = attachmentData.mLinkIndex[threadIndexInWarp];
			PxgArticulationBlockLinkData& cLink = artiLinks[linkInd];

			const PxReal cfm = cLink.mCfm[threadIndexInWarp];

			const PxTransform cBody2World = loadSpatialTransform(cLink.mAccumulatedPose, threadIndexInWarp);
			const PxVec3 rb = cBody2World.q.rotate(attachmentData.mRelativeOffset[threadIndexInWarp]);
			const PxVec3 cAttachPoint = cBody2World.p + rb;


			PxReal distance = 0.f;
			PxVec3 dif(0.f);
			//if the current attachment's parent is the root, we need to compute root axis and root raXn
			if (parent == 0)
			{
				dif = rAttachPoint - cAttachPoint;
				const PxReal distanceSq = dif.magnitudeSquared();
				distance = PxSqrt(distanceSq);

				rAxis = distance > 0.001f ? (dif / distance) : PxVec3(0.f);
				rRaXn = rRa.cross(rAxis);

			}
			else
			{
				PxgArticulationBlockAttachmentData& pAttachmentData = attachmentBlock[parent];
				const PxU32 pLinkInd = pAttachmentData.mLinkIndex[threadIndexInWarp];
				PxgArticulationBlockLinkData& pLink = artiLinks[pLinkInd];
				
				const PxTransform pBody2World = loadSpatialTransform(pLink.mAccumulatedPose, threadIndexInWarp);

				const PxVec3 ra = pBody2World.q.rotate(pAttachmentData.mRelativeOffset[threadIndexInWarp]);

				const PxVec3 pAttachPoint = pBody2World.p + ra;

				dif = pAttachPoint - cAttachPoint;
				const PxReal distanceSq = dif.magnitudeSquared();
				distance = PxSqrt(distanceSq);
			}

			const PxReal u = distance * attachmentData.mCoefficient[threadIndexInWarp] + accumLength[parent];

			PxU64 children = attachmentData.mChildrens[threadIndexInWarp];

			if (children)
			{
				const PxU32 numChildrens = __popcll(children);
				stackCount += numChildrens;

				accumLength[child] = u;
			}
			else
			{
		

				const PxVec3 axis = distance > 0.001f ? (dif / distance) : PxVec3(0.f);

				const PxVec3 rbXn = rb.cross(axis);

				Cm::UnAlignedSpatialVector axis0(rAxis, rRaXn);
				Cm::UnAlignedSpatialVector axis1(-axis, -rbXn);

				Cm::UnAlignedSpatialVector deltaV0, deltaV1;

				getImpulseSelfResponse(rAttachmentLinkIndex, linkInd, rAttachmentLink, cLink, axis0, axis1,
					deltaV0, deltaV1, artiBlock, artiLinks, artiDofs, threadIndexInWarp);

				
				
				const PxReal r0 = deltaV0.bottom.dot(rAxis) + deltaV0.top.dot(rRaXn);
				const PxReal r1 = deltaV1.bottom.dot(axis) + deltaV1.top.dot(rbXn);

				const PxReal unitResponse = r0 - r1;

				const PxReal recipResponse = unitResponse > DY_ARTICULATION_MIN_RESPONSE ? (1.0f / (unitResponse + cfm))  : 0.0f;

				PxgArticulationInternalTendonConstraintData& constraint = constraintBlock[numConstraints++];

				//constraint.mDeltaVA[threadIndexInWarp] = r0;
				//storeSpatialVector(constraint.mDeltaVA, deltaV0, threadIndexInWarp);
				//storeSpatialVector(constraint.mDeltaVB, deltaV1, threadIndexInWarp);
				storeSpatialVector(constraint.mRow0, axis0, threadIndexInWarp);
				storeSpatialVector(constraint.mRow1, -axis1, threadIndexInWarp);

				//storeSpatialVector(constraint.mDeltaVB, deltaV1, threadIndexInWarp);
				constraint.mDeltaVA[threadIndexInWarp] = r0;

				//constraint.mResponse[threadIndexInWarp] = unitResponse;
				constraint.mRecipResponse[threadIndexInWarp] = recipResponse;

				const PxReal a = stepDt * (stepDt*stiffness + damping);

				const PxReal a2 = stepDt * (stepDt* limitStiffness + damping);

				const PxReal x = unitResponse > 0.f ? 1.0f / (1.0f + a * unitResponse) : 0.f;

				const PxReal x2 = unitResponse > 0.f ? 1.0f / (1.0f + a2 * unitResponse) : 0.f;

				constraint.mVelMultiplier[threadIndexInWarp] = -x * a;
				constraint.mImpulseMultiplier[threadIndexInWarp] = isTGSSolver ? 1.f : 1.f - x;
				constraint.mBiasCoefficient[threadIndexInWarp] = (-stiffness * x * stepDt);
				constraint.mAppliedForce[threadIndexInWarp] = 0.f;

				constraint.mAccumulatedLength[threadIndexInWarp] = u;
				constraint.mLink0[threadIndexInWarp] = rAttachmentLinkIndex;
				constraint.mLink1[threadIndexInWarp] = linkInd;

				constraint.mLimitImpulseMultiplier[threadIndexInWarp] = isTGSSolver ? 1.f : 1.f - x2;
				constraint.mLimitBiasCoefficient[threadIndexInWarp] = (-limitStiffness * x2 * stepDt);
				constraint.mLimitAppliedForce[threadIndexInWarp] = 0.f;

				constraint.mRestDistance[threadIndexInWarp] = attachmentData.mRestDistance[threadIndexInWarp];
				constraint.mLowLimit[threadIndexInWarp] = attachmentData.mLowLimit[threadIndexInWarp];
				constraint.mHighLimit[threadIndexInWarp] = attachmentData.mHighLimit[threadIndexInWarp];
			}

			if (stackCount > 0)
			{
				//clear child
				bitStack &= (~(1ull << child));

				//add on children to the stack
				bitStack |= children;

				//pop up the next child from stack
				child = 63 - __clzll(bitStack);

				//assign the parent with next child's parent
				parent = attachmentBlock[child].mParents[threadIndexInWarp];
			}

		}

		tendonData.mNumConstraints[threadIndexInWarp] = numConstraints;

	}
}


static __device__ void setupInternalFixedTendonConstraints(
	PxgArticulationBlockData& artiBlock,
	PxgArticulationBlockLinkData* PX_RESTRICT artiLinks,
	PxgArticulationBlockDofData* PX_RESTRICT artiDofs,
	PxgArticulationBlockFixedTendonData* PX_RESTRICT artiTendons,
	PxgArticulationInternalTendonConstraintData* PX_RESTRICT artiTendonConstraints,
	PxgArticulationBlockTendonJointData* PX_RESTRICT artiTendonJoints,
	const PxU32 maxTendonJoints, const PxReal stepDt, const PxReal dt,
	const PxReal invDt, bool isTGSSolver, const PxU32 threadIndexInWarp)
{
	const PxU32 numTendons = artiBlock.mNumFixedTendons[threadIndexInWarp];

	for (PxU32 i = 0; i < numTendons; ++i)
	{
		PxgArticulationBlockFixedTendonData& tendonData = artiTendons[i];

		const PxReal stiffness = tendonData.mStiffness[threadIndexInWarp];
		const PxReal damping = tendonData.mDamping[threadIndexInWarp];
		const PxReal limitStiffness = tendonData.mLimitStiffness[threadIndexInWarp];

		PxgArticulationBlockTendonJointData* tendonJointBlock = &artiTendonJoints[i * maxTendonJoints];
		PxgArticulationInternalTendonConstraintData* constraintBlock = &artiTendonConstraints[i * maxTendonJoints];

		PxU64 bitStack = tendonJointBlock[0].mChildrens[threadIndexInWarp];

		PxU32 stackCount = __popcll(bitStack);

		PxU32 parent = 0;

		PxU32 numConstraints = 0;
		
		PxgArticulationBlockTendonJointData& sTendonJointData = tendonJointBlock[parent];
		const PxU32 sLinkIndex = sTendonJointData.mLinkIndex[threadIndexInWarp];
		PxgArticulationBlockLinkData& sLink = artiLinks[sLinkIndex];
		const PxTransform sBody2World = loadSpatialTransform(sLink.mAccumulatedPose, threadIndexInWarp);

		PxVec3 sAxis;
		PxVec3 sRaXn;


		PxU32 child = 63 - __clzll(bitStack);

		while (stackCount != 0)
		{
			stackCount--;

			PxgArticulationBlockTendonJointData& tjData = tendonJointBlock[child];

			const PxU32 tjAxis = tjData.mAxis[threadIndexInWarp];
			
			const PxU32 cLinkInd = tjData.mLinkIndex[threadIndexInWarp];
			PxgArticulationBlockLinkData& cLink = artiLinks[cLinkInd];

			const PxU32 parentLink = cLink.mParents[threadIndexInWarp];
			const PxReal cfm = PxMax(artiLinks[parentLink].mCfm[threadIndexInWarp], cLink.mCfm[threadIndexInWarp]);

			const PxU32 jointOffset = cLink.mJointOffset[threadIndexInWarp];
			const PxU8 dofIndex = cLink.mInvDofIds[tjAxis][threadIndexInWarp];
			PxgArticulationBlockDofData& dofData = artiDofs[jointOffset + dofIndex];
			
			const Cm::UnAlignedSpatialVector worldMotionVector = loadSpatialVector(dofData.mWorldMotionMatrix, threadIndexInWarp);

			//if the current tendon joint's parent is the root, we need to compute root axis and root raXn
			if (parent == 0)
			{
				
				if (tjAxis < PxArticulationAxis::eX)
				{

					sAxis = PxVec3(0.f);
					sRaXn = worldMotionVector.top;
				}
				else
				{
					const float4 p = cLink.mParentPose.p[threadIndexInWarp];
					const PxQuat q = reinterpret_cast<PxQuat&>(cLink.mParentPose.q[threadIndexInWarp]);
					const PxTransform parentPose(PxVec3(p.x, p.y, p.z), q);
					const PxTransform cA2w = sBody2World.transform(parentPose);
					const PxVec3 ang0 = (cA2w.p - sBody2World.p).cross(worldMotionVector.bottom);
					sAxis = worldMotionVector.bottom;
					sRaXn = ang0;
				}
			}
			
			PxU64 children = tjData.mChildrens[threadIndexInWarp];
			
			PxVec3 axis, rbXn;
			if (tjAxis < PxArticulationAxis::eX)
			{
				axis = PxVec3(0.f);
				rbXn = worldMotionVector.top;
			}
			else
			{
				const PxTransform cBody2World = loadSpatialTransform(cLink.mAccumulatedPose, threadIndexInWarp);

				const float4 p = cLink.mChildPose.p[threadIndexInWarp];
				const PxQuat q = reinterpret_cast<PxQuat&>(cLink.mChildPose.q[threadIndexInWarp]);
				const PxTransform childPose(PxVec3(p.x, p.y, p.z), q);
				const PxTransform cB2w = cBody2World.transform(childPose);

				const PxVec3 tAxis = worldMotionVector.bottom;
				axis = tAxis;
				rbXn = (cB2w.p - cBody2World.p).cross(axis);
			}
			

			//create constraint
			Cm::UnAlignedSpatialVector axis0(sAxis, sRaXn);
			Cm::UnAlignedSpatialVector axis1(-axis, -rbXn);

			Cm::UnAlignedSpatialVector deltaV0, deltaV1;

			getImpulseSelfResponse(sLinkIndex, cLinkInd, sLink, cLink, axis0, axis1,
				deltaV0, deltaV1, artiBlock, artiLinks, artiDofs, threadIndexInWarp);

			const PxReal r0 = deltaV0.bottom.dot(sAxis) + deltaV0.top.dot(sRaXn);
			const PxReal r1 = deltaV1.bottom.dot(axis) + deltaV1.top.dot(rbXn);

			const PxReal unitResponse = r0 - r1;

			const PxReal recipResponse = 1.0f / (unitResponse + cfm);

			PxgArticulationInternalTendonConstraintData& constraint = constraintBlock[numConstraints];

			constraint.mDeltaVA[threadIndexInWarp] = r0;
			storeSpatialVector(constraint.mDeltaVB, deltaV1, threadIndexInWarp);
			storeSpatialVector(constraint.mRow0, axis0, threadIndexInWarp);
			storeSpatialVector(constraint.mRow1, -axis1, threadIndexInWarp);

			constraint.mRecipResponse[threadIndexInWarp] = recipResponse;

			const PxReal a = stepDt * (stepDt*stiffness + damping);

			const PxReal a2 = stepDt * (stepDt*limitStiffness + damping);

			const PxReal x = unitResponse > 0.f ? 1.0f / (1.0f + a * unitResponse) : 0.f;

			const PxReal x2 = unitResponse > 0.f ? 1.0f / (1.0f + a2 * unitResponse) : 0.f;

			constraint.mVelMultiplier[threadIndexInWarp] = -x * a;
			constraint.mImpulseMultiplier[threadIndexInWarp] = isTGSSolver ? 1.f : 1.f - x;
			constraint.mBiasCoefficient[threadIndexInWarp] = (-stiffness * x * stepDt);
			constraint.mAppliedForce[threadIndexInWarp] = 0.f;
			
			constraint.mLink0[threadIndexInWarp] = sLinkIndex;
			constraint.mLink1[threadIndexInWarp] = cLinkInd;

			constraint.mLimitImpulseMultiplier[threadIndexInWarp] = isTGSSolver ? 1.f : 1.f - x2;
			constraint.mLimitBiasCoefficient[threadIndexInWarp] = (-limitStiffness * x2 * stepDt);
			constraint.mLimitAppliedForce[threadIndexInWarp] = 0.f;
		

			//assign constraint index to tendon joint data
			tjData.mConstraintId[threadIndexInWarp] = numConstraints;
			numConstraints++;

			if (children)
			{
				const PxU32 numChildrens = __popcll(children);
				stackCount += numChildrens;
			}

			if (stackCount > 0)
			{
				//clear child
				bitStack &= (~(1ull << child));

				//add on children to the stack
				bitStack |= children;

				//pop up the next child from stack
				child = 63 - __clzll(bitStack);

				//assign the parent with next child's parent
				parent = tendonJointBlock[child].mParents[threadIndexInWarp];
			}

		}

		tendonData.mNumConstraints[threadIndexInWarp] = numConstraints;

	}
}

extern "C" __global__ void setupInternalConstraintLaunch1T(
	PxgArticulationCoreDesc* scDesc, const PxReal stepDt, const PxReal dt,
	const PxReal invDt, bool isTGSSolver)
{
	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);
	const PxU32 warpIndex = threadIdx.y;

	const PxU32 nbArticulations = scDesc->nbArticulations;

	const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + warpIndex;
	const PxU32 globalThreadIndex = globalWarpIndex * WARP_SIZE + threadIndexInWarp;


	if (globalThreadIndex < nbArticulations)
	{
		const PxU32 maxLinks = scDesc->mMaxLinksPerArticulation;
		const PxU32 maxDofs = scDesc->mMaxDofsPerArticulation;
		

		PxgArticulationBlockData& artiBlock = scDesc->mArticulationBlocks[globalWarpIndex];
		PxgArticulationBlockLinkData* artiLinks = &scDesc->mArticulationLinkBlocks[globalWarpIndex * maxLinks];
		PxgArticulationBlockDofData* artiDofs = &scDesc->mArticulationDofBlocks[globalWarpIndex * maxDofs];

		{
			setupInternalConstraints(artiBlock, artiLinks, artiDofs, stepDt, dt, invDt, isTGSSolver, threadIndexInWarp);
		}

		{
			const PxU32 maxSpatialTendons = scDesc->mMaxSpatialTendonsPerArticulation;
			const PxU32 maxAttachments = scDesc->mMaxAttachmentPerArticulation;

			PxgArticulationBlockSpatialTendonData* artiSpatialTendons = &scDesc->mArticulationSpatialTendonBlocks[globalWarpIndex * maxSpatialTendons];
			PxgArticulationInternalTendonConstraintData* artiSpatialTendonConstraints = &scDesc->mArticulationSpatialTendonConstraintBlocks[globalWarpIndex * maxSpatialTendons * maxAttachments];
			PxgArticulationBlockAttachmentData* artiAttachments = &scDesc->mArticulationAttachmentBlocks[globalWarpIndex * maxSpatialTendons * maxAttachments];
			setupInternalSpatialTendonConstraints(artiBlock, artiLinks, artiDofs, artiSpatialTendons, artiSpatialTendonConstraints, artiAttachments, maxAttachments, stepDt, dt, invDt, isTGSSolver, threadIndexInWarp);
		}

		{
			const PxU32 maxFixedTendons = scDesc->mMaxFixedTendonsPerArticulation;
			const PxU32 maxTendonJoints = scDesc->mMaxTendonJointPerArticulation;

			PxgArticulationBlockFixedTendonData* artiFixedTendons = &scDesc->mArticulationFixedTendonBlocks[globalWarpIndex * maxFixedTendons];
			PxgArticulationInternalTendonConstraintData* artiFixedTendonConstraints = &scDesc->mArticulationFixedTendonConstraintBlocks[globalWarpIndex * maxFixedTendons * maxTendonJoints];
			PxgArticulationBlockTendonJointData* artiTendonJoints = &scDesc->mArticulationTendonJointBlocks[globalWarpIndex * maxFixedTendons * maxTendonJoints];
			setupInternalFixedTendonConstraints(artiBlock, artiLinks, artiDofs, artiFixedTendons, artiFixedTendonConstraints, artiTendonJoints, maxTendonJoints, stepDt, dt, invDt, isTGSSolver, threadIndexInWarp);
		}

		{
			//See comment accompanying mPathToRootBitFieldBlocks declaration for a quick reminder of the indexing of mPathToRootBitFieldBlocks. 
			const PxU32 artiPathToRootBitFieldWordCount = (maxLinks + 63) / 64;
			PxgArticulationBitFieldData* artiPathToRootBitFields = &scDesc->mPathToRootBitFieldBlocks[globalWarpIndex * maxLinks * artiPathToRootBitFieldWordCount];

			//Get the mimic joints for this articulation.
			const PxU32 maxMimicJoints = scDesc->mMaxMimicJointsPerArticulation;
			PxgArticulationBlockMimicJointData* artiMimicJoints = &scDesc->mArticulationMimicJointBlocks[globalWarpIndex * maxMimicJoints];

			setupInternalMimicJointConstraints(
				artiBlock,
				artiLinks, 
				artiPathToRootBitFields, artiPathToRootBitFieldWordCount, 
				artiDofs,
				artiMimicJoints,
				threadIndexInWarp);
		}
	}
}

//This is for PGS solver
static __device__ PX_FORCE_INLINE void solveStaticConstraints(PxgArticulationCoreDesc* PX_RESTRICT scDesc, PxgArticulationBlockLinkData& PX_RESTRICT data,
	const PxgSolverSharedDesc<IterativeSolveData>* const PX_RESTRICT sharedDesc,
	Cm::UnAlignedSpatialVector& PX_RESTRICT vel, Cm::UnAlignedSpatialVector& PX_RESTRICT impulse, Cm::UnAlignedSpatialVector& PX_RESTRICT deltaV, PxU32 threadIndexInWarp, bool doFriction,
	PxReal /*minPen*/, PxReal /*elapsedTime*/, PxU32 linkID, PxU32 constraintCounts, PxgErrorAccumulator* PX_RESTRICT error)
{

	
	const IterativeSolveData& iterativeData = sharedDesc->iterativeData;

	const PxU32 constraintBatchOffset = data.mStaticJointStartIndex[threadIndexInWarp];
	const PxU32 contactBatchOffset = data.mStaticContactStartIndex[threadIndexInWarp];
	const PxU32 contactCounts = data.mNbStaticContacts[threadIndexInWarp];

	Cm::UnAlignedSpatialVector oldVel = vel;
	Cm::UnAlignedSpatialVector vel0, vel1;
		
	for (PxU32 i = 0; i < constraintCounts; ++i)
	{
		const PxgBlockConstraintBatch& batch = iterativeData.blockConstraintBatch[constraintBatchOffset + i];

		

		PxU32 mask = batch.mask;

		PxU32 offset = warpScanExclusive(mask, threadIndexInWarp);

		const PxNodeIndex igNodeIndexA = batch.bodyANodeIndex[offset];

		if (igNodeIndexA.isArticulation())
		{
			vel0 = vel;
			vel1 = Cm::UnAlignedSpatialVector::Zero();
		}
		else
		{
			assert(batch.bodyBNodeIndex[offset].isArticulation());
			vel0 = Cm::UnAlignedSpatialVector::Zero();
			vel1 = vel;
		}

		PxgArticulationBlockResponse* responses = iterativeData.artiResponse;
		const PxU32 responseIndex = batch.mArticulationResponseIndex;

		
		Cm::UnAlignedSpatialVector impulse0 = Cm::UnAlignedSpatialVector::Zero();
		Cm::UnAlignedSpatialVector impulse1 = Cm::UnAlignedSpatialVector::Zero();

		{
			assert(batch.constraintType == PxgSolverConstraintDesc::eARTICULATION_CONSTRAINT_1D);

			// For interaction with static objects, mass-splitting is not used; thus, reference counts are 1 (default).
			solveExt1DBlock(batch, vel0, vel1, offset, iterativeData.blockJointConstraintHeaders,
				iterativeData.blockJointConstraintRowsCon, iterativeData.blockJointConstraintRowsMod,
				&responses[responseIndex], impulse0, impulse1, scDesc->mContactErrorAccumulator.mCounter >= 0);
		}

		if (igNodeIndexA.isArticulation())
		{
			impulse += impulse0;
			vel = vel0;
		}
		else
		{
			assert(batch.bodyBNodeIndex[offset].isArticulation());
			impulse += impulse1;
			vel = vel1;
		}

	}

	for (PxU32 i = 0; i < contactCounts; ++i)
	{

		const PxgBlockConstraintBatch& batch = iterativeData.blockConstraintBatch[contactBatchOffset + i];

		PxU32 mask = batch.mask;

		PxU32 offset = warpScanExclusive(mask, threadIndexInWarp);

		const PxNodeIndex igNodeIndexA = batch.bodyANodeIndex[offset];

		if (igNodeIndexA.isArticulation())
		{
			vel0 = vel;
			vel1 = Cm::UnAlignedSpatialVector::Zero();
		}
		else
		{
			assert(batch.bodyBNodeIndex[offset].isArticulation());
			vel0 = Cm::UnAlignedSpatialVector::Zero();
			vel1 = vel;
		}


		Cm::UnAlignedSpatialVector impulse0 = Cm::UnAlignedSpatialVector::Zero();
		Cm::UnAlignedSpatialVector impulse1 = Cm::UnAlignedSpatialVector::Zero();

		{
			assert(batch.constraintType == PxgSolverConstraintDesc::eARTICULATION_CONTACT);

			// For interaction with static objects, mass-splitting is not used; thus, reference counts are 1 (default).
			solveExtContactsBlock(batch, vel0, vel1, doFriction, iterativeData.blockContactHeaders,
				iterativeData.blockFrictionHeaders, iterativeData.blockContactPoints,
				iterativeData.blockFrictions, iterativeData.artiResponse, impulse0,
				impulse1, offset, error);
		}

		if (igNodeIndexA.isArticulation())
		{
			impulse += impulse0;
			vel = vel0;
		}
		else
		{
			assert(batch.bodyBNodeIndex[offset].isArticulation());
			impulse += impulse1;
			vel = vel1;
		}
	}

	if ((constraintCounts + contactCounts) > 0)
	{
		deltaV += vel - oldVel;
	}
}

//This is for TGS solver
static __device__ PX_FORCE_INLINE void solveStaticConstraints(PxgArticulationCoreDesc* PX_RESTRICT scDesc, const PxgArticulationBlockLinkData& PX_RESTRICT data, 
	const PxgSolverSharedDesc<IterativeSolveDataTGS>* const PX_RESTRICT sharedDesc,
	Cm::UnAlignedSpatialVector& PX_RESTRICT vel, Cm::UnAlignedSpatialVector& PX_RESTRICT impulse, Cm::UnAlignedSpatialVector& PX_RESTRICT deltaV, PxU32 threadIndexInWarp, bool doFriction, 
	PxReal minPen, PxReal elapsedTime, PxU32 linkID, PxU32 constraintCounts, PxgErrorAccumulator* PX_RESTRICT error)
{

	const IterativeSolveDataTGS& iterativeData = sharedDesc->iterativeData;

	const PxU32 constraintBatchOffset = data.mStaticJointStartIndex[threadIndexInWarp];
	const PxU32 contactBatchOffset = data.mStaticContactStartIndex[threadIndexInWarp];
	const PxU32 contactCounts = data.mNbStaticContacts[threadIndexInWarp];

	Cm::UnAlignedSpatialVector oldVel = vel;

	Cm::UnAlignedSpatialVector delta = loadSpatialVector(data.mDeltaMotion, threadIndexInWarp);

	PxQuat deltaQ;
	if(constraintCounts > 0)
		deltaQ = loadQuat(data.mDeltaQ, threadIndexInWarp);

	Cm::UnAlignedSpatialVector vel0, vel1;
	Cm::UnAlignedSpatialVector delta0, delta1;
	PxQuat deltaQ0, deltaQ1;

	for (PxU32 i = 0; i < constraintCounts; ++i)
	{
		const PxgBlockConstraintBatch& batch = iterativeData.blockConstraintBatch[constraintBatchOffset + i];

		PxU32 mask = batch.mask;

		PxU32 offset = warpScanExclusive(mask, threadIndexInWarp);

		const PxNodeIndex igNodeIndexA = batch.bodyANodeIndex[offset];
		
		if (igNodeIndexA.isArticulation())
		{
			vel0 = vel;
			delta0 = delta;
			deltaQ0 = deltaQ;

			vel1 = Cm::UnAlignedSpatialVector::Zero();
			delta1 = Cm::UnAlignedSpatialVector::Zero();
			deltaQ1 = PxQuat(PxIdentity);

		}
		else
		{
			assert(batch.bodyBNodeIndex[offset].isArticulation());
			vel0 = Cm::UnAlignedSpatialVector::Zero();
			delta0 = Cm::UnAlignedSpatialVector::Zero();
			deltaQ0 = PxQuat(PxIdentity);

			vel1 = vel;
			delta1 = delta;
			deltaQ1 = deltaQ;
		}


		Cm::UnAlignedSpatialVector impulse0 = Cm::UnAlignedSpatialVector::Zero();
		Cm::UnAlignedSpatialVector impulse1 = Cm::UnAlignedSpatialVector::Zero();

		{
			assert(batch.constraintType == PxgSolverConstraintDesc::eARTICULATION_CONSTRAINT_1D);

			// For interaction with static objects, mass-splitting is not used; thus, reference counts are 1 (default).
			solveExt1DBlockTGS(batch, vel0, vel1, delta0, delta1, offset, iterativeData.blockJointConstraintHeaders,
				iterativeData.blockJointConstraintRowsCon, iterativeData.artiResponse, deltaQ0, deltaQ1, elapsedTime, impulse0, impulse1, 
				scDesc->mContactErrorAccumulator.mCounter >= 0);

		}

		if (igNodeIndexA.isArticulation())
		{
			impulse += impulse0;
			vel = vel0;
			delta = delta0;
			deltaQ = deltaQ0;
			
		}
		else
		{
			assert(batch.bodyBNodeIndex[offset].isArticulation());
			impulse += impulse1;
			vel = vel1;
			delta = delta1;
			deltaQ = deltaQ1;
		}

	}

	for (PxU32 i = 0; i < contactCounts; ++i)
	{

		const PxgBlockConstraintBatch& batch = iterativeData.blockConstraintBatch[contactBatchOffset + i];

		PxU32 mask = batch.mask;

		PxU32 offset = warpScanExclusive(mask, threadIndexInWarp);

		const PxNodeIndex igNodeIndexA = batch.bodyANodeIndex[offset];

		if (igNodeIndexA.isArticulation())
		{
			vel0 = vel;
			delta0 = delta;
			deltaQ0 = deltaQ;

			vel1 = Cm::UnAlignedSpatialVector::Zero();
			delta1 = Cm::UnAlignedSpatialVector::Zero();
			deltaQ1 = PxQuat(PxIdentity);
		}
		else
		{
			assert(batch.bodyBNodeIndex[offset].isArticulation());
			vel0 = Cm::UnAlignedSpatialVector::Zero();
			delta0 = Cm::UnAlignedSpatialVector::Zero();
			deltaQ0 = PxQuat(PxIdentity);

			vel1 = vel;
			delta1 = delta;
			deltaQ1 = deltaQ;
		}

		Cm::UnAlignedSpatialVector impulse0 = Cm::UnAlignedSpatialVector::Zero();
		Cm::UnAlignedSpatialVector impulse1 = Cm::UnAlignedSpatialVector::Zero();

		{
			assert(batch.constraintType == PxgSolverConstraintDesc::eARTICULATION_CONTACT);
	
			// For interaction with static objects, mass-splitting is not used; thus, reference counts are 1 (default).
			solveExtContactBlockTGS(batch, vel0, vel1, delta0, delta1, offset,
				iterativeData.blockContactHeaders, iterativeData.blockFrictionHeaders, iterativeData.blockContactPoints,
				iterativeData.blockFrictions, iterativeData.artiResponse, elapsedTime, minPen, impulse0, impulse1, error);
		}

		if (igNodeIndexA.isArticulation())
		{
			impulse += impulse0;
			vel = vel0;
			delta = delta0;
			deltaQ = deltaQ0;
		}
		else
		{
			assert(batch.bodyBNodeIndex[offset].isArticulation());
			impulse += impulse1;
			vel = vel1;
			delta = delta1;
			deltaQ = deltaQ1;
		}

	}

	if ((constraintCounts + contactCounts) > 0)
	{
		deltaV += vel - oldVel;
	}
}

template <typename IterativeData, const bool isTGS, const bool residualReportingEnabled>
static __device__ void artiSolveInternalConstraints1T(PxgArticulationCoreDesc* PX_RESTRICT scDesc, const PxReal dt,
	const PxReal invDt, const PxReal elapsedTime, const bool isVelIter, const PxU32* const PX_RESTRICT staticContactUniqueIds,
	const PxU32* const PX_RESTRICT staticJointUniqueIds,
	const PxgSolverSharedDesc<IterativeData>* const PX_RESTRICT sharedDesc,
	const PxReal erp,
	bool doFriction, bool isExternalForceEveryStep )
{
	const PxU32 nbSlabs = scDesc->nbSlabs; // # articulation slabs
	const PxU32 nbArticulations = scDesc->nbArticulations;

	const PxU32 blockStride = blockDim.x;// / WARP_SIZE;

	//This identifies which warp a specific thread is in, we treat all warps in all blocks as a flatten warp array
	//and we are going to index the work based on that
	//const PxU32 warpIndex = threadIdx.y;
	const PxU32 globalThreadIndex = blockIdx.x * blockStride + threadIdx.x;

	const PxReal minPen = isVelIter ? 0.f : -PX_MAX_F32;

	PxgErrorAccumulator error;
	PxgErrorAccumulator contactError;

	if (globalThreadIndex < nbArticulations)
	{
		PxgArticulation& articulation = scDesc->articulations[globalThreadIndex];
		if(residualReportingEnabled)
		{
			articulation.internalResidualAccumulator.reset();
			articulation.contactResidualAccumulator.reset();
		}

		//KS - strong possiblity that nodeIndex and bodySim can be dropped because articId == globalWarpIndex!

		//Identify which block we are solving...

		PxgArticulationBlockLinkData* PX_RESTRICT data = scDesc->mArticulationLinkBlocks + scDesc->mMaxLinksPerArticulation * blockIdx.x;
		PxgArticulationBlockDofData* PX_RESTRICT dofData = scDesc->mArticulationDofBlocks + scDesc->mMaxDofsPerArticulation * blockIdx.x;
		PxgArticulationBlockData& blockData = scDesc->mArticulationBlocks[blockIdx.x];

		const PxU32 numLinks = blockData.mNumLinks[threadIdx.x];

		if (blockData.mStateDirty[threadIdx.x] & PxgArtiStateDirtyFlag::eHAS_IMPULSES)
		{
			averageLinkImpulsesAndPropagate(scDesc->slabHasChanges, scDesc->impulses, blockData, data, dofData, globalThreadIndex, scDesc->mMaxLinksPerArticulation,
				nbArticulations, nbSlabs, numLinks, threadIdx.x);
		}
		blockData.mStateDirty[threadIdx.x] = PxgArtiStateDirtyFlag::eVEL_DIRTY;

		const bool fixBase = blockData.mFlags[threadIdx.x] & PxArticulationFlag::eFIX_BASE;

		const Cm::UnAlignedSpatialVector rootDeferredZ = loadSpatialVector(blockData.mRootDeferredZ, threadIdx.x);

		Cm::UnAlignedSpatialVector parentDeltaV(PxVec3(0.f), PxVec3(0.f)), storeParentDeltaV(PxVec3(0.f), PxVec3(0.f));
		Cm::UnAlignedSpatialVector parentImp(PxVec3(0.f), PxVec3(0.f));
		

		if (!fixBase)
		{
			const PxU32 constraintCounts0 = data[0].mNbStaticJoints[threadIdx.x];

			Dy::SpatialMatrix spatialMatrix;
			loadSpatialMatrix(blockData.mInvSpatialArticulatedInertia, threadIdx.x, spatialMatrix);

			const Cm::UnAlignedSpatialVector motionVelocity0 = loadSpatialVector(data[0].mMotionVelocity, threadIdx.x);

			parentDeltaV = spatialMatrix * -rootDeferredZ;

			Cm::UnAlignedSpatialVector rootVel = motionVelocity0 + parentDeltaV;
			storeParentDeltaV = parentDeltaV;
			
			//Solve constraints...
			solveStaticConstraints(scDesc, data[0], sharedDesc, rootVel, parentImp, parentDeltaV, threadIdx.x,
				doFriction, minPen, elapsedTime, 0, constraintCounts0, residualReportingEnabled ? &contactError : NULL);
		}
		
		PxgArticulationTraversalStackData* PX_RESTRICT stack = scDesc->mArticulationTraversalStackBlocks + scDesc->mMaxLinksPerArticulation * blockIdx.x;

		PxU32 parent = 0;

		storeSpatialVector(stack[parent].deltaVStack, storeParentDeltaV, threadIdx.x);
		storeSpatialVector(stack[parent].impulseStack, parentImp, threadIdx.x);

		PxU32 offset = data[parent].mChildrenOffset[threadIdx.x];
		PxU32 numChildren = data[parent].mNumChildren[threadIdx.x];

		for (PxU32 i = 0; i < numChildren; ++i)
		{
			stack[i].indices[threadIdx.x] = offset + i;
		}

		PxU32 stackCount = numChildren;

		PxU32 linkID = ((stackCount != 0) ? stack[stackCount - 1].indices[threadIdx.x] : 0xffffffff);
		ArticulationImplicitDriveDesc implicitDriveDesc(PxZero);	// PT: moved outside of loop because we don't have an empty ctor

		while (stackCount != 0)
		{
			const bool isBackProp = (parent == linkID);

			PxgArticulationBlockLinkData& linkData = data[linkID];

			const float c2px = linkData.mRw_x[threadIdx.x];
			const float c2py = linkData.mRw_y[threadIdx.x];
			const float c2pz = linkData.mRw_z[threadIdx.x];
			
			const PxU32 jointOffset = linkData.mJointOffset[threadIdx.x];

			const PxU32 dofCount = linkData.mDofs[threadIdx.x];

			if (isBackProp)
			{
				parent = linkData.mParents[threadIdx.x];

				const Cm::UnAlignedSpatialVector solverSpatialImpulse = loadSpatialVector(linkData.mSolverSpatialImpulse, threadIdx.x);

				const Cm::UnAlignedSpatialVector ZInternalConstraint = loadSpatialVector(linkData.mSolverSpatialInternalConstraintImpulse, threadIdx.x);

				PxSpatialMatrix spatialResponse;
				loadSpatialMatrix(data[parent].mSpatialResponseMatrix, threadIdx.x, spatialResponse);

				stackCount--;
	
				Cm::UnAlignedSpatialVector impulse = parentImp;

				parentDeltaV = loadSpatialVector(stack[parent].deltaVStack, threadIdx.x); 
				parentImp = loadSpatialVector(stack[parent].impulseStack, threadIdx.x);

				//Accumulate the solver impulse applied to this link.
				storeSpatialVector(linkData.mSolverSpatialImpulse, solverSpatialImpulse + impulse - ZInternalConstraint, threadIdx.x);

				//We're finished with this link so we can move to the next one.
				linkID = stackCount > 0 ? stack[stackCount - 1].indices[threadIdx.x] : 0xffffffff;

				const Cm::UnAlignedSpatialVector propagateImp = propagateImpulseW_0(PxVec3(c2px, c2py, c2pz), dofData + jointOffset, impulse, dofCount, threadIdx.x);

				parentImp += propagateImp;


				Cm::UnAlignedSpatialVector deltaV = spatialResponse * -parentImp;

				parentDeltaV += deltaV;

				storeSpatialVector(stack[parent].impulseStack, parentImp, threadIdx.x);
			}
			else
			{
				Cm::UnAlignedSpatialVector parentV = parentDeltaV + loadSpatialVector(data[parent].mMotionVelocity, threadIdx.x);
				const Cm::UnAlignedSpatialVector childDelta = loadSpatialVector(linkData.mDeltaMotion, threadIdx.x);
				const Cm::UnAlignedSpatialVector parentDelta = loadSpatialVector(data[parent].mDeltaMotion, threadIdx.x);
				

				Cm::UnAlignedSpatialVector deltaV = propagateAccelerationW(PxVec3(c2px, c2py, c2pz), dofData + jointOffset,
					parentDeltaV, dofCount, NULL, threadIdx.x);

				Cm::UnAlignedSpatialVector childV = deltaV + loadSpatialVector(linkData.mMotionVelocity, threadIdx.x);

				storeSpatialVector(stack[linkID].deltaVStack, deltaV, threadIdx.x);

				Cm::UnAlignedSpatialVector impulse = Cm::UnAlignedSpatialVector(PxVec3(0.f), PxVec3(0.f));

				for (PxU32 dof = 0; dof < dofCount; ++dof)
				{
					PxgArticulationBlockDofData& PX_RESTRICT thisDof = dofData[jointOffset + dof];
					const PxU32 motion = thisDof.mMotion[threadIdx.x];
					if (motion != PxArticulationMotion::eLOCKED)
					{
						const PxReal maxJointVel = thisDof.mConstraintData.mMaxJointVelocity[threadIdx.x];
						// PT: preload as much data as we can
						const Cm::UnAlignedSpatialVector row0 = loadSpatialVector(thisDof.mConstraintData.mRow0, threadIdx.x);
						const Cm::UnAlignedSpatialVector row1 = loadSpatialVector(thisDof.mConstraintData.mRow1, threadIdx.x);
						const PxReal maxDriveForce = thisDof.mConstraintData.mConstraintMaxForce[threadIdx.x];
						const PxReal driveForce = thisDof.mConstraintData.mDriveForce[threadIdx.x];
						const PxReal recipResponse = thisDof.mConstraintData.mRecipResponse[threadIdx.x];
						const PxReal response = thisDof.mConstraintData.mResponse[threadIdx.x];
						const PxReal maxFrictionForce = thisDof.mConstraintData.mMaxFrictionForce[threadIdx.x];
						const Cm::UnAlignedSpatialVector deltaVA = loadSpatialVector(thisDof.mConstraintData.mDeltaVA, threadIdx.x);
						const Cm::UnAlignedSpatialVector deltaVB = loadSpatialVector(thisDof.mConstraintData.mDeltaVB, threadIdx.x);

						const PxReal effectiveDt = isTGS && isExternalForceEveryStep && !isVelIter ? dt : scDesc->dt;

						const PxReal staticFrictionImpulse = thisDof.mConstraintData.mStaticFrictionEffort[threadIdx.x] * effectiveDt;
						const PxReal dynamicFrictionImpulse = thisDof.mConstraintData.mDynamicFrictionEffort[threadIdx.x] * effectiveDt;
						const PxReal viscousFrictionCoefficient = thisDof.mConstraintData.mViscousFrictionCoefficient[threadIdx.x] * effectiveDt;

						if (!(isTGS && isVelIter))
							implicitDriveDesc = thisDof.mConstraintData.getImplicitDriveDesc(threadIdx.x);

						PxReal jointV = row1.innerProduct(childV) - row0.innerProduct(parentV);

						const PxReal jointDeltaP = row1.innerProduct(childDelta) - row0.innerProduct(parentDelta);

						PxReal frictionDeltaF = 0.0f;
						bool newFrictionModel = staticFrictionImpulse != 0.0f || viscousFrictionCoefficient !=  0.0f;

						// deprecated friction model
						if (!newFrictionModel)	
						{
							// Friction force is accumulated through all position iterations only for PGS
							const PxReal appliedFriction = isTGS ? 0.0f : thisDof.mConstraintData.mAccumulatedFrictionImpulse[threadIdx.x];

							const PxReal frictionForce = PxClamp(-jointV * recipResponse + appliedFriction,
																-maxFrictionForce, maxFrictionForce);
							thisDof.mConstraintData.mAccumulatedFrictionImpulse[threadIdx.x] = frictionForce; // This is not used for TGS

							frictionDeltaF = frictionForce - appliedFriction;

							jointV += frictionDeltaF * response;
						}

						PxReal driveDeltaF = 0.0f;
						{
							const PxReal unclampedForce = (isTGS && isVelIter) ? driveForce :
									computeDriveImpulse(driveForce, jointV, jointDeltaP, elapsedTime,
										implicitDriveDesc);

							const PxReal clampedForce = PxClamp(unclampedForce, -maxDriveForce, maxDriveForce);
							driveDeltaF = (clampedForce - driveForce);

							thisDof.mConstraintData.mDriveForce[threadIdx.x] = clampedForce;
						}
						jointV += driveDeltaF * response;

						if (newFrictionModel)
						{
							const PxReal appliedFriction = isTGS && isExternalForceEveryStep && !isVelIter ? 0.0f : thisDof.mConstraintData.mAccumulatedFrictionImpulse[threadIdx.x];
							PxReal totalImpulse = appliedFriction - jointV * recipResponse;
							totalImpulse = computeFrictionImpulse(totalImpulse, staticFrictionImpulse, dynamicFrictionImpulse, viscousFrictionCoefficient, jointV);
							frictionDeltaF = totalImpulse - appliedFriction;
							thisDof.mConstraintData.mAccumulatedFrictionImpulse[threadIdx.x] += frictionDeltaF; // to keep track of accumulated impulse for velIter in TGS with isExternalForceEveryStep
							jointV += frictionDeltaF * response;
						}

						PxReal posLimitDeltaF = 0.0f;
						if (motion == PxArticulationMotion::eLIMITED)
						{
							const PxReal errorLow = thisDof.mConstraintData.mLimitError_LowX_highY[threadIdx.x].x;
							const PxReal errorHigh = thisDof.mConstraintData.mLimitError_LowX_highY[threadIdx.x].y;
							PxReal& lowImp = thisDof.mConstraintData.mLowImpulse[threadIdx.x];
							PxReal& highImp = thisDof.mConstraintData.mHighImpulse[threadIdx.x];
							posLimitDeltaF = computeLimitImpulse(
								dt, invDt, isVelIter,
								response, recipResponse, erp,
								errorLow, errorHigh, jointDeltaP,
								lowImp, highImp, jointV);
						}

						PxReal velLimitDeltaF = 0.0f;
						if (PxAbs(jointV) > maxJointVel)
						{
							const PxReal newJointV = PxClamp(jointV, -maxJointVel, maxJointVel);
							velLimitDeltaF = (newJointV - jointV) * recipResponse;
							jointV = newJointV;
						}

						const PxReal deltaF = frictionDeltaF + driveDeltaF + posLimitDeltaF + velLimitDeltaF;

						//Accumulate error even if it is zero because the increment of the counter affects the RMS value
						if (residualReportingEnabled)
							error.accumulateErrorLocal(deltaF, recipResponse);

						//if (deltaF != 0.f)
						{
							// the signs look suspicious here
							const Cm::UnAlignedSpatialVector pDelta = deltaVA * -deltaF;
							const Cm::UnAlignedSpatialVector cDelta = deltaVB * -deltaF;

							parentDeltaV += pDelta;
							deltaV += cDelta;

							parentV += pDelta;
							childV += cDelta;

							//KS - TODO - remove msImpulses and msDeltaV from here!
							parentImp += row0 * deltaF;
							impulse -= row1 * deltaF;
						}
					}
				}

				const PxU32 constraintCounts = linkData.mNbStaticJoints[threadIdx.x];

				numChildren = linkData.mNumChildren[threadIdx.x];	// PT: preload to avoid stall

				//Store the internal constraint impulse applied to this link on this solver iteration.
				storeSpatialVector(linkData.mSolverSpatialInternalConstraintImpulse, impulse, threadIdx.x);

				solveStaticConstraints(scDesc, linkData, sharedDesc, childV, impulse, deltaV, threadIdx.x,
					doFriction, minPen, elapsedTime, linkID, constraintCounts, residualReportingEnabled ? &contactError : NULL);

				storeSpatialVector(stack[parent].impulseStack, parentImp, threadIdx.x);
				storeSpatialVector(stack[linkID].impulseStack, impulse, threadIdx.x);
				
				{
					parent = linkID;
					//if there are no children under the current link, we don't change the linkID so parent index
					//will be the same as linkID 
					if (numChildren > 0)
					{
						offset = linkData.mChildrenOffset[threadIdx.x];
						for (PxU32 i = 0; i < numChildren; ++i)
						{
							stack[stackCount++].indices[threadIdx.x] = offset + i;
						}

						linkID = stack[stackCount - 1].indices[threadIdx.x];
					}
					
					parentDeltaV = deltaV;
					parentImp = impulse;
						
				}
			}
		}

		storeSpatialVector(blockData.mRootDeferredZ, rootDeferredZ + parentImp, threadIdx.x);
		
		if (residualReportingEnabled) 
		{
			error.accumulateErrorGlobalNoAtomics(articulation.internalResidualAccumulator);
			contactError.accumulateErrorGlobalNoAtomics(articulation.contactResidualAccumulator);
		}
	}
}


static __device__ void updateSolveInternalTendonConstraintsTGS(
	PxgArticulationBlockData& artiBlock,
	PxgArticulationBlockLinkData* PX_RESTRICT artiLinks,
	PxgArticulationBlockSpatialTendonData* PX_RESTRICT artiTendons,
	PxgArticulationInternalTendonConstraintData* PX_RESTRICT artiTendonConstraints,
	PxgArticulationBlockAttachmentData* PX_RESTRICT artiAttachments,
	const PxU32 maxAttachments,
	const PxU32 threadIndexInWarp
	)
{
	const PxU32 numTendons = artiBlock.mNumSpatialTendons[threadIndexInWarp];

	PxReal accumLength[DY_ARTICULATION_TENDON_MAX_SIZE];

	for (PxU32 i = 0; i < numTendons; ++i)
	{
		const PxgArticulationBlockSpatialTendonData& tendonData = artiTendons[i];

		const PxReal offset = tendonData.mOffset[threadIndexInWarp];

		PxgArticulationBlockAttachmentData* attachmentBlock = &artiAttachments[i * maxAttachments];
		PxgArticulationInternalTendonConstraintData* constraintBlock = &artiTendonConstraints[i * maxAttachments];

		PxU64 bitStack = attachmentBlock[0].mChildrens[threadIndexInWarp];

		const PxReal coefficient = attachmentBlock[0].mCoefficient[threadIndexInWarp];

		PxU32 stackCount = __popcll(bitStack);

		PxU32 parent = 0;

		PxU32 numConstraints = 0;

		accumLength[parent] = offset * coefficient;


		PxU32 child = 63 - __clzll(bitStack);

		while (stackCount != 0)
		{
			stackCount--;

			PxgArticulationBlockAttachmentData& attachmentData = attachmentBlock[child];
			PxgArticulationBlockAttachmentData& pAttachmentData = attachmentBlock[parent];

			const PxU32 linkInd = attachmentData.mLinkIndex[threadIndexInWarp];
			PxgArticulationBlockLinkData& cLink = artiLinks[linkInd];
			const PxU32 pLinkInd = pAttachmentData.mLinkIndex[threadIndexInWarp];
			PxgArticulationBlockLinkData& pLink = artiLinks[pLinkInd];

			const PxTransform cBody2World = loadSpatialTransform(cLink.mAccumulatedPose, threadIndexInWarp);
			const PxTransform pBody2World = loadSpatialTransform(pLink.mAccumulatedPose, threadIndexInWarp);

			const PxVec3 rb = cBody2World.q.rotate(attachmentData.mRelativeOffset[threadIndexInWarp]);
			const PxVec3 ra = pBody2World.q.rotate(pAttachmentData.mRelativeOffset[threadIndexInWarp]);

			const PxVec3 cAttachPoint = cBody2World.p + rb;
			const PxVec3 pAttachPoint = pBody2World.p + ra;

			const PxVec3 dif = pAttachPoint - cAttachPoint;
			const PxReal distanceSq = dif.magnitudeSquared();
			const PxReal distance = PxSqrt(distanceSq);

			const PxReal u = distance * attachmentData.mCoefficient[threadIndexInWarp] + accumLength[parent];

			PxU64 children = attachmentData.mChildrens[threadIndexInWarp];

			if (children)
			{
				const PxU32 numChildrens = __popcll(children);
				stackCount += numChildrens;
				accumLength[child] = u;
			}
			else
			{
				PxgArticulationInternalTendonConstraintData& constraint = constraintBlock[numConstraints++];
				constraint.mAccumulatedLength[threadIndexInWarp] = u;
				
			}


			if (stackCount > 0)
			{
				//clear child
				bitStack &= (~(1ull << child));

				//add on children to the stack
				bitStack |= children;

				//pop up the next child from stack
				child = 63 - __clzll(bitStack);

				//assign the parent with next child's parent
				parent = attachmentBlock[child].mParents[threadIndexInWarp];
			}

		}

	}
}

static __device__ Cm::UnAlignedSpatialVector pxcFsGetVelocity(
	PxgArticulationBlockData& artiBlock,
	PxgArticulationBlockLinkData* linkData, 
	PxgArticulationBlockDofData* dofData,
	PxgArticulationBitFieldStackData* linkBitFieldData,
	const PxU32 wordSize, const PxU32 linkID, const bool fixBase, 
	PxReal* jointDofSpeeds,
	const PxU32 threadIndexInWarp)
{
	Cm::UnAlignedSpatialVector deltaV(PxVec3(0.f), PxVec3(0.f));

	if (!fixBase)
	{
		const Cm::UnAlignedSpatialVector rootDeferredZ = loadSpatialVector(artiBlock.mRootDeferredZ, threadIndexInWarp);

		Dy::SpatialMatrix spatialMatrix;

		loadSpatialMatrix(artiBlock.mInvSpatialArticulatedInertia, threadIndexInWarp, spatialMatrix);

		deltaV = spatialMatrix * -rootDeferredZ;
	}

	PxReal deltaJointDofSpeeds[3] = {0, 0, 0};

	PxgArticulationBlockLinkData& link = linkData[linkID];

	for (PxU32 j = 0, wordOffset = 0; j < wordSize; ++j, wordOffset += 64)
	{
		PxU64 pathToRoot = linkBitFieldData[linkID * wordSize + j].bitField[threadIndexInWarp];

		while (pathToRoot != 0)
		{
			const PxU32 index = articulationLowestSetBit(pathToRoot) + wordOffset;

			if (index != 0)
			{
				PxgArticulationBlockLinkData& cLink = linkData[index];

				const PxU32 jointOffset = cLink.mJointOffset[threadIndexInWarp];
				const PxU32 dofCount = cLink.mDofs[threadIndexInWarp];

				const float rwx = cLink.mRw_x[threadIdx.x];
				const float rwy = cLink.mRw_y[threadIdx.x];
				const float rwz = cLink.mRw_z[threadIdx.x];

				const PxVec3 childToParent(rwx, rwy, rwz);

				//Compute the deltaqDot on the inbound joint of linkID.
				PxReal* optionalDeltaJointSpeeds = ((linkID == index) && jointDofSpeeds) ? deltaJointDofSpeeds : NULL;

				deltaV = propagateAccelerationW(childToParent, dofData + jointOffset, deltaV, dofCount, optionalDeltaJointSpeeds, threadIndexInWarp);
			}

			//clear the lowest bit
			pathToRoot &= (pathToRoot - 1);
		}
	}

	//Optionally report the updated joint speed after accounting for the delta joint dof speed arising from the deferred impulses.
	if(jointDofSpeeds)
	{
		const PxU32 dofCount = link.mDofs[threadIndexInWarp];
		const PxU32 jointOffset = link.mJointOffset[threadIndexInWarp];
		for(PxU32 i = 0; i < dofCount; i++)
		{
			jointDofSpeeds[i] = dofData[jointOffset + i].mJointVelocities[threadIndexInWarp] + deltaJointDofSpeeds[i];
		}
	}

	Cm::UnAlignedSpatialVector motionVelocity = loadSpatialVector(link.mMotionVelocity, threadIndexInWarp);

	return motionVelocity + deltaV;
}

static __device__ void pxcFsApplyImpulses(PxgArticulationBlockData& blockData, 
	PxgArticulationBlockLinkData* PX_RESTRICT linkData,
	PxgArticulationBlockDofData* PX_RESTRICT dofData,
	PxgArticulationBitFieldStackData* PX_RESTRICT linkBitFields, const PxU32 linkBitFieldWordCount,
	const PxU32 linkID0, const PxVec3& linear0, const PxVec3& angular0, const PxReal* PX_RESTRICT jointImpulse0,
	const PxU32 linkID1, const PxVec3& linear1, const PxVec3& angular1, const PxReal* PX_RESTRICT jointImpulse1,
	const PxU32 threadIndexInWarp)
{
	PxU64 commonId = 0;
	PxU32 commonLink = 0;
	for (PxI32 i = linkBitFieldWordCount -1 ; i >= 0; --i)
	{
		const PxU64 wordA = linkBitFields[linkID0 * linkBitFieldWordCount + i].bitField[threadIndexInWarp];
		const PxU64 wordB = linkBitFields[linkID1 * linkBitFieldWordCount + i].bitField[threadIndexInWarp];
		commonId = wordA & wordB;
		if (commonId != 0)
		{
			commonLink = articulationHighestSetBit(commonId) + i * 64;
			break;
		}
	}
	

	Cm::UnAlignedSpatialVector Z0(-linear0, -angular0);
	Cm::UnAlignedSpatialVector Z1(-linear1, -angular1);
	
	//The common link will either be linkID1, or its ancestors.
	//The common link cannot be an index before either linkID1 or linkID0
	for (PxU32 i = linkID1; i != commonLink; i = linkData[i].mParents[threadIndexInWarp])
	{
		PxgArticulationBlockLinkData& tlink = linkData[i];
		const PxU32 jointOffset = tlink.mJointOffset[threadIndexInWarp];
		const PxU32 dofCount = tlink.mDofs[threadIndexInWarp];

		const float rwx = tlink.mRw_x[threadIndexInWarp];
		const float rwy = tlink.mRw_y[threadIndexInWarp];
		const float rwz = tlink.mRw_z[threadIndexInWarp];

		const PxVec3 child2Parent(rwx, rwy, rwz);

		//Only apply the joint impulse to the inbound joint of linkID1.
		//Note: linkID1 might be the common link. If this is the case, we will only apply 
		//jointImpulse1 when we propagate from the common link to the root.
		//Watch out for that when we propagate from the common link.
		const PxReal* jointImpulseToApply = (linkID1 == i) ? jointImpulse1 : NULL;

		addSpatialVector(linkData[i].mSolverSpatialImpulse, Z1, threadIndexInWarp);

		Z1 = propagateImpulseW_0(child2Parent, dofData + jointOffset, Z1, dofCount, threadIndexInWarp, jointImpulseToApply, 1.0f);
	}

	for (PxU32 i = linkID0; i != commonLink; i = linkData[i].mParents[threadIndexInWarp])
	{
		PxgArticulationBlockLinkData& tlink = linkData[i];
		const PxU32 jointOffset = tlink.mJointOffset[threadIndexInWarp];
		const PxU32 dofCount = tlink.mDofs[threadIndexInWarp];

		const float rwx = tlink.mRw_x[threadIndexInWarp];
		const float rwy = tlink.mRw_y[threadIndexInWarp];
		const float rwz = tlink.mRw_z[threadIndexInWarp];

		const PxVec3 child2Parent(rwx, rwy, rwz);
		
		//Only apply the joint impulse to the inbound joint of linkID0.
		//Note: linkID0 might be the common link. If this is the case, we will only apply 
		//jointImpulse0 when we propagate from the common link to the root.
		//Watch out for that when we propagate from the common link.
		const PxReal* jointImpulseToApply = (linkID0 == i) ? jointImpulse0 : NULL;

		addSpatialVector(linkData[i].mSolverSpatialImpulse, Z0, threadIndexInWarp);

		Z0 = propagateImpulseW_0(child2Parent, dofData + jointOffset, Z0, dofCount, threadIndexInWarp, jointImpulseToApply, 1.0f);
	}

	Cm::UnAlignedSpatialVector ZCommon = Z0 + Z1;

	//If either linkID0 (or linkID1) are the common link then we will not yet have applied 
	//jointImpulse0 (or jointImpulse1) to the inbound joint of the link.  
	//Work out how much joint impulse to apply to the inbound joint of the common link.
	PxReal jointImpulseToApplyAtCommonLink[3] = {0, 0, 0};
	if(((linkID0 == commonLink) && jointImpulse0) || ((linkID1 == commonLink) && jointImpulse1))
	{
		const PxU32 linkIndices[2] = {linkID0, linkID1};
		const PxReal* jointImpulses[2]= {jointImpulse0, jointImpulse1};
		const PxU32 dofCountAtCommonLink = linkData[commonLink].mDofs[threadIndexInWarp];
		for(PxU32 k = 0; k < 2; k++)
		{
			const PxU32 linkId = linkIndices[k];
			const PxReal* jointImpulse = jointImpulses[k];
			if((linkId == commonLink) && jointImpulse)
			{		
				for(PxU32 i = 0; i < dofCountAtCommonLink; i++)
				{
					jointImpulseToApplyAtCommonLink[i] += jointImpulse[i];
				}
			}
		}
	}

	for (PxU32 i = commonLink; i; i = linkData[i].mParents[threadIndexInWarp])
	{
		PxgArticulationBlockLinkData& tlink = linkData[i];

		const PxU32 jointOffset = tlink.mJointOffset[threadIndexInWarp];
		const PxU32 dofCount = tlink.mDofs[threadIndexInWarp];

		const float rwx = tlink.mRw_x[threadIndexInWarp];
		const float rwy = tlink.mRw_y[threadIndexInWarp];
		const float rwz = tlink.mRw_z[threadIndexInWarp];

		const PxVec3 child2Parent(rwx, rwy, rwz);
		

		//Only apply a joint impulse to the inbound joint of commonLink.
		//The joint impulse to apply to the inbound joint will only be non-zero if the common link 
		//is either linkID0 or linkID1.
		const PxReal* jointImpulseToApply = (commonLink == i) ? jointImpulseToApplyAtCommonLink : NULL;

		addSpatialVector(linkData[i].mSolverSpatialImpulse, ZCommon, threadIndexInWarp);

		ZCommon = propagateImpulseW_0(child2Parent, dofData + jointOffset, ZCommon, dofCount, threadIndexInWarp, jointImpulseToApply, 1.0f);
	}

	addSpatialVector(blockData.mRootDeferredZ, ZCommon, threadIndexInWarp);
}


static __device__ void solveInternalSpatialConstraints(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	const bool isTGS, const PxU32 threadIndexInWarp)
{
	const PxU32 maxLinks = scDesc->mMaxLinksPerArticulation;

	PxgArticulationBlockLinkData* data = scDesc->mArticulationLinkBlocks + maxLinks * blockIdx.x;
	PxgArticulationBlockDofData* dofData = scDesc->mArticulationDofBlocks + scDesc->mMaxDofsPerArticulation * blockIdx.x;

	const PxU32 wordSize = (maxLinks + 63 )/ 64;
	PxgArticulationBitFieldData* linkBitFieldsData = scDesc->mPathToRootBitFieldBlocks + maxLinks * wordSize * blockIdx.x;

	const PxU32 maxTendons = scDesc->mMaxSpatialTendonsPerArticulation;
	const PxU32 maxAttachments = scDesc->mMaxAttachmentPerArticulation;

	PxgArticulationBlockSpatialTendonData* tendonData = scDesc->mArticulationSpatialTendonBlocks + maxTendons * blockIdx.x;
	PxgArticulationInternalTendonConstraintData* tendonConstraintData = scDesc->mArticulationSpatialTendonConstraintBlocks + maxTendons * maxAttachments * blockIdx.x;
	PxgArticulationBlockAttachmentData* attachmentData = scDesc->mArticulationAttachmentBlocks + maxTendons * maxAttachments * blockIdx.x;
	PxgArticulationBlockData& blockData = scDesc->mArticulationBlocks[blockIdx.x];

	const bool fixBase = blockData.mFlags[threadIndexInWarp] & PxArticulationFlag::eFIX_BASE;


	if (isTGS)
	{
		//compute the accumulated errors
		updateSolveInternalTendonConstraintsTGS(blockData, data, tendonData, tendonConstraintData, attachmentData, maxAttachments, threadIndexInWarp);
	}


	const PxU32 numTendons = blockData.mNumSpatialTendons[threadIndexInWarp];

	for (PxU32 i = 0; i < numTendons; ++i)
	{
		const PxgArticulationBlockSpatialTendonData& tendonBlock = tendonData[i];

		PxgArticulationInternalTendonConstraintData* tendonConstraintBlock = &tendonConstraintData[i * maxAttachments];

		const PxU32 numConstraints = tendonBlock.mNumConstraints[threadIndexInWarp];

		//for the internal tendon constraint, the parent link will be extractly the same for all the constraints. Therefore, we can
		//precompute the velocity of parent

		if (numConstraints > 0)
		{

			PxgArticulationInternalTendonConstraintData& constraintData = tendonConstraintBlock[numConstraints - 1];
			const PxU32 parentID = constraintData.mLink0[threadIndexInWarp];
			Cm::UnAlignedSpatialVector parentVel = pxcFsGetVelocity(blockData, data, dofData, linkBitFieldsData, wordSize, parentID, fixBase, NULL, threadIndexInWarp);

			PxReal parentV = loadSpatialVector(tendonConstraintBlock[0].mRow0, threadIndexInWarp).innerProduct(parentVel);

			for (PxI32 j = numConstraints - 1; j >= 0; --j)
			{

				PxgArticulationInternalTendonConstraintData& constraintData = tendonConstraintBlock[j];

				assert(parentID == constraintData.mLink0[threadIndexInWarp]);
				const PxU32 childID = constraintData.mLink1[threadIndexInWarp];

				Cm::UnAlignedSpatialVector childVel = pxcFsGetVelocity(blockData, data, dofData, linkBitFieldsData, wordSize, childID, fixBase, NULL, threadIndexInWarp);
			
				const PxReal accumLength = constraintData.mAccumulatedLength[threadIndexInWarp];
				const PxReal error = constraintData.mRestDistance[threadIndexInWarp] - accumLength;
				
				PxReal error2 = 0.f;
				const PxReal lowLimit = constraintData.mLowLimit[threadIndexInWarp];
				const PxReal highLimit = constraintData.mHighLimit[threadIndexInWarp];

				if (accumLength > highLimit)
					error2 = highLimit - accumLength;
				else if (accumLength < lowLimit)
					error2 = lowLimit - accumLength;

				const Cm::UnAlignedSpatialVector row1 = loadSpatialVector(constraintData.mRow1, threadIndexInWarp);
				const Cm::UnAlignedSpatialVector row0 = loadSpatialVector(constraintData.mRow0, threadIndexInWarp);

				const PxReal jointV = row1.innerProduct(childVel) - parentV;

				const PxReal velMultiplier = constraintData.mVelMultiplier[threadIndexInWarp];
				const PxReal biasCoefficient = constraintData.mBiasCoefficient[threadIndexInWarp];
				const PxReal appliedForce = constraintData.mAppliedForce[threadIndexInWarp];
				const PxReal impulseMultiplier = constraintData.mImpulseMultiplier[threadIndexInWarp];

				const PxReal limitBiasCoefficient = constraintData.mLimitBiasCoefficient[threadIndexInWarp];
				const PxReal limitAppiledForce = constraintData.mLimitAppliedForce[threadIndexInWarp];
				const PxReal limitImpulseMultiplier = constraintData.mLimitImpulseMultiplier[threadIndexInWarp];

				const PxReal unclampedForce = jointV * velMultiplier + error * biasCoefficient + appliedForce * impulseMultiplier;

				PxReal unclampedForce2 = (error2 * limitBiasCoefficient) + limitAppiledForce * limitImpulseMultiplier;

				const PxReal deltaF = (unclampedForce - appliedForce) + (unclampedForce2 - limitAppiledForce);

				constraintData.mAppliedForce[threadIndexInWarp] = unclampedForce;

				constraintData.mLimitAppliedForce[threadIndexInWarp] = unclampedForce2;

				parentV += constraintData.mDeltaVA[threadIndexInWarp] * -deltaF;

				if (deltaF != 0.f)
				{
					Cm::UnAlignedSpatialVector i0 = row0 * (-deltaF);
					Cm::UnAlignedSpatialVector i1 = row1 * deltaF;

					pxcFsApplyImpulses(
						blockData, data, dofData, 
						linkBitFieldsData, wordSize, 
						parentID, i0.top, i0.bottom, NULL,
						childID, i1.top, i1.bottom, NULL,
						threadIndexInWarp);
				}
			}
		}
	}
}


static __device__ void solveInternalFixedConstraints(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	const bool isTGS, const PxU32 threadIndexInWarp)
{

	const PxU32 maxLinks = scDesc->mMaxLinksPerArticulation;
	const PxU32 wordSize = (maxLinks + 63) / 64;

	PxgArticulationBlockLinkData* artiLinks = scDesc->mArticulationLinkBlocks + maxLinks * blockIdx.x;
	PxgArticulationBlockDofData* artiDofs = scDesc->mArticulationDofBlocks + scDesc->mMaxDofsPerArticulation * blockIdx.x;

	PxgArticulationBitFieldData* linkBitFields = scDesc->mPathToRootBitFieldBlocks + maxLinks * wordSize * blockIdx.x;
	const PxU32 maxFixedTendons = scDesc->mMaxFixedTendonsPerArticulation;
	const PxU32 maxTendonJoints = scDesc->mMaxTendonJointPerArticulation;

	PxgArticulationBlockFixedTendonData* artiTendon = scDesc->mArticulationFixedTendonBlocks + maxFixedTendons * blockIdx.x;
	PxgArticulationInternalTendonConstraintData* artiTendonConstraints = scDesc->mArticulationFixedTendonConstraintBlocks + maxFixedTendons * maxTendonJoints * blockIdx.x;
	PxgArticulationBlockTendonJointData* artiTendonJoints = scDesc->mArticulationTendonJointBlocks + maxFixedTendons * maxTendonJoints * blockIdx.x;
	PxgArticulationBlockData& blockData = scDesc->mArticulationBlocks[blockIdx.x];


	const bool fixBase = blockData.mFlags[threadIndexInWarp] & PxArticulationFlag::eFIX_BASE;
	const PxU32 numTendons = blockData.mNumFixedTendons[threadIndexInWarp];
	
	for (PxU32 i = 0; i < numTendons; ++i)
	{
		PxgArticulationBlockFixedTendonData& tendonData = artiTendon[i];

		const PxReal lowLimit = tendonData.mLowLimit[threadIndexInWarp];
		const PxReal highLimit = tendonData.mHighLimit[threadIndexInWarp];
		
		PxgArticulationBlockTendonJointData* tendonJointBlock = &artiTendonJoints[i * maxTendonJoints];
		PxgArticulationInternalTendonConstraintData* constraintBlock = &artiTendonConstraints[i * maxTendonJoints];

		PxU64 bitStack = tendonJointBlock[0].mChildrens[threadIndexInWarp];

		PxU32 stackCount = __popcll(bitStack);

		PxU32 parent = 0;

		PxgArticulationBlockTendonJointData& sTendonJointData = tendonJointBlock[parent];
		const PxU32 sLinkIndex = sTendonJointData.mLinkIndex[threadIndexInWarp];
		PxgArticulationBlockLinkData& sLink = artiLinks[sLinkIndex];
		const PxTransform sBody2World = loadSpatialTransform(sLink.mAccumulatedPose, threadIndexInWarp);

		Cm::UnAlignedSpatialVector parentVel = pxcFsGetVelocity(blockData, artiLinks, artiDofs, linkBitFields, wordSize, sLinkIndex, fixBase, NULL, threadIndexInWarp);
		Cm::UnAlignedSpatialVector velA = loadSpatialVector(sLink.mMotionVelocity, threadIndexInWarp);

		Cm::UnAlignedSpatialVector delta = parentVel - velA;
		storeSpatialVector(sLink.mScratchImpulse, Cm::UnAlignedSpatialVector::Zero(), threadIndexInWarp);
		storeSpatialVector(sLink.mScratchDeltaV, delta, threadIndexInWarp);


		PxReal rootImp = 0.f;
		PxU64 totalStack = 0;
		

		PxReal error = 0.f;
		PxReal velocity = 0.f;

		const PxU32 firstChild = 63 - __clzll(bitStack);

		totalStack |= bitStack;


		while (stackCount != 0)
		{

			PxU32 child = 63 - __clzll(bitStack);
			bitStack &= (~(1ull << (child)));
			stackCount--;

			PxgArticulationBlockTendonJointData& tjData = tendonJointBlock[child];

			const PxU32 cLinkInd = tjData.mLinkIndex[threadIndexInWarp];
			PxgArticulationBlockLinkData& cLink = artiLinks[cLinkInd];
			const PxU32 pLinkInd = cLink.mParents[threadIndexInWarp];
			PxgArticulationBlockLinkData& pLink = artiLinks[pLinkInd];
			const PxU32 jointOffset = cLink.mJointOffset[threadIndexInWarp];
			const PxU32 dofCount = cLink.mDofs[threadIndexInWarp];

			const float rwx = cLink.mRw_x[threadIndexInWarp];
			const float rwy = cLink.mRw_y[threadIndexInWarp];
			const float rwz = cLink.mRw_z[threadIndexInWarp];

			const PxVec3 c2p(rwx, rwy, rwz);

			const PxU32 constraintId = tjData.mConstraintId[threadIndexInWarp];


			PxgArticulationInternalTendonConstraintData& constraint = constraintBlock[constraintId];

			const PxU32 tjAxis = tjData.mAxis[threadIndexInWarp];
			const PxReal coefficient = tjData.mCoefficient[threadIndexInWarp];

			const PxU32 dofIndex = cLink.mInvDofIds[tjAxis][threadIndexInWarp];
			PxgArticulationBlockDofData& dofData = artiDofs[jointOffset + dofIndex];

			const PxReal jointPose = dofData.mJointPositions[threadIndexInWarp];

			Cm::UnAlignedSpatialVector parentDeltaV = loadSpatialVector(pLink.mScratchDeltaV, threadIndexInWarp);
			Cm::UnAlignedSpatialVector parentV = loadSpatialVector(pLink.mMotionVelocity, threadIndexInWarp) + parentDeltaV;

			Cm::UnAlignedSpatialVector cDeltaV = propagateAccelerationW(c2p, artiDofs + jointOffset, parentDeltaV, dofCount, NULL, threadIndexInWarp);

			storeSpatialVector(cLink.mScratchDeltaV, cDeltaV, threadIndexInWarp);


			Cm::UnAlignedSpatialVector velB = loadSpatialVector(cLink.mMotionVelocity, threadIndexInWarp);
			Cm::UnAlignedSpatialVector childVel = velB + cDeltaV;

			storeSpatialVector(cLink.mScratchImpulse, Cm::UnAlignedSpatialVector(PxVec3(0.f), PxVec3(0.f)), threadIndexInWarp);



			PxU64 children = tjData.mChildrens[threadIndexInWarp];

			//KS - constraint.row0.innerProduct(rootVel) is a known so we can replace it with a dynamically updated scalar value...
			Cm::UnAlignedSpatialVector row0 = loadSpatialVector(constraint.mRow0, threadIndexInWarp);
			Cm::UnAlignedSpatialVector row1 = loadSpatialVector(constraint.mRow1, threadIndexInWarp);
			PxReal jointV = row1.innerProduct(childVel) - row0.innerProduct(parentV);


			error += jointPose * coefficient;
			velocity += jointV * coefficient;

			//Add myself in the list to propagate up changes with my children back up the system
			//assign child to parent
			parent = child;

			if (children)
			{
				stackCount += __popcll(children);
				//add on children to the stack
				bitStack |= children;
				totalStack |= children;
			}
		}

		const PxU32 count = __popcll(totalStack);
		const PxReal scale = count ? 1.f / PxReal(count) : 0.f;

		const PxReal length = error + tendonData.mOffset[threadIndexInWarp];
		

		PxReal limitError = 0.f;
		if (length < lowLimit)
			limitError = length - lowLimit;
		else if (length > highLimit)
			limitError = length - highLimit;

		error = (length - tendonData.mRestLength[threadIndexInWarp])*scale;
		limitError *= scale;
		velocity *= scale;

		//Once we get here, we've got the full stack...

		while (totalStack != 0)
		{
			PxU32 child = 63 - __clzll(totalStack);
			totalStack &= (~(1ull << (child)));

			PxgArticulationBlockTendonJointData& tjData = tendonJointBlock[child];

			const PxU32 cLinkInd = tjData.mLinkIndex[threadIndexInWarp];
			PxgArticulationBlockLinkData& cLink = artiLinks[cLinkInd];
			const PxU32 pLinkInd = cLink.mParents[threadIndexInWarp];
			PxgArticulationBlockLinkData& pLink = artiLinks[pLinkInd];
			const PxU32 jointOffset = cLink.mJointOffset[threadIndexInWarp];
			const PxU32 dofCount = cLink.mDofs[threadIndexInWarp];

			const float rwx = cLink.mRw_x[threadIndexInWarp];
			const float rwy = cLink.mRw_y[threadIndexInWarp];
			const float rwz = cLink.mRw_z[threadIndexInWarp];

			const PxVec3 c2p(rwx, rwy, rwz);

			const PxU32 constraintId = tjData.mConstraintId[threadIndexInWarp];

			PxgArticulationInternalTendonConstraintData& constraint = constraintBlock[constraintId];

			const PxReal recipCoefficient = tjData.mRecipCoefficient[threadIndexInWarp];
			const PxReal velMultiplier = constraint.mVelMultiplier[threadIndexInWarp];
			const PxReal biasCoefficient = constraint.mBiasCoefficient[threadIndexInWarp];
			const PxReal limitBiasCoefficient = constraint.mLimitBiasCoefficient[threadIndexInWarp];
			const PxReal impulseMultiplier = constraint.mImpulseMultiplier[threadIndexInWarp];
			const PxReal appliedForce = constraint.mAppliedForce[threadIndexInWarp];
			const PxReal limitImpulseMultiplier = constraint.mLimitImpulseMultiplier[threadIndexInWarp];
			const PxReal limitAppliedForce = constraint.mLimitAppliedForce[threadIndexInWarp];

			const PxReal unclampedForce = ((velocity * velMultiplier + error * biasCoefficient)*recipCoefficient)
				+ appliedForce * impulseMultiplier;


			const PxReal unclampedForce2 = (limitError * limitBiasCoefficient * recipCoefficient)
				+ limitAppliedForce * limitImpulseMultiplier;

			const PxReal deltaF = ((unclampedForce - appliedForce) + (unclampedForce2 - limitAppliedForce));

			constraint.mAppliedForce[threadIndexInWarp] = unclampedForce;
			constraint.mLimitAppliedForce[threadIndexInWarp] = unclampedForce2;


			rootImp += deltaF;

			const Cm::UnAlignedSpatialVector cImpulse = loadSpatialVector(cLink.mScratchImpulse, threadIndexInWarp);

			const Cm::UnAlignedSpatialVector impulse = loadSpatialVector(constraint.mRow1, threadIndexInWarp) * -deltaF + cImpulse;

			//Store (impulse - YInt)
			//but YInt = constraint.mRow1 * -deltaF
			//so (impulse - YInt) = cImpulse.			
			addSpatialVector(cLink.mSolverSpatialImpulse, cImpulse, threadIndexInWarp);

			Cm::UnAlignedSpatialVector propagatedImpulse = propagateImpulseW_0(c2p, artiDofs + jointOffset, impulse, dofCount, threadIndexInWarp);

			addSpatialVector(pLink.mScratchImpulse, propagatedImpulse, threadIndexInWarp);


		}


		const PxU32 firstConstraint = tendonJointBlock[firstChild].mConstraintId[threadIndexInWarp];
		const PxgArticulationInternalTendonConstraintData& constraint = constraintBlock[firstConstraint];

		const Cm::UnAlignedSpatialVector propagatedImpulse = loadSpatialVector(sLink.mScratchImpulse, threadIndexInWarp);

		Cm::UnAlignedSpatialVector Z = propagatedImpulse + loadSpatialVector(constraint.mRow0, threadIndexInWarp) * rootImp;


		for (PxU32 linkID = sLinkIndex; linkID; linkID = artiLinks[linkID].mParents[threadIndexInWarp])
		{
			PxgArticulationBlockLinkData& link = artiLinks[linkID];
			const PxU32 jointOffset = link.mJointOffset[threadIndexInWarp];
			const PxU32 dofCount = link.mDofs[threadIndexInWarp];

			const float rwx = link.mRw_x[threadIndexInWarp];
			const float rwy = link.mRw_y[threadIndexInWarp];
			const float rwz = link.mRw_z[threadIndexInWarp];

			const PxVec3 c2p(rwx, rwy, rwz);

			Z = propagateImpulseW_0(c2p, artiDofs + jointOffset, Z, dofCount, threadIndexInWarp);
		}

		addSpatialVector(blockData.mRootDeferredZ, Z, threadIndexInWarp);
	}

	if (numTendons > 0)
	{
		const PxU32 nbLinks = blockData.mNumLinks[threadIndexInWarp];
		for (PxU32 i = 0; i < nbLinks; ++i)
		{
			storeSpatialVector(artiLinks[i].mScratchImpulse, Cm::UnAlignedSpatialVector::Zero(), threadIndexInWarp);
		}
	}
}

static __device__ void solveInternalMimicJointConstraints(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc, const PxReal biasCoefficient, const PxReal dt, const PxReal recipDt, 
	const bool isVelocityIteration, const bool isTGS, const PxU32 threadIndexInWarp)
{
	const PxU32 maxLinks = scDesc->mMaxLinksPerArticulation;
	const PxU32 wordSize = (maxLinks + 63) / 64;

	PxgArticulationBlockLinkData* artiLinks = scDesc->mArticulationLinkBlocks + maxLinks * blockIdx.x;
	PxgArticulationBlockDofData* artiDofs = scDesc->mArticulationDofBlocks + scDesc->mMaxDofsPerArticulation * blockIdx.x;

	PxgArticulationBitFieldData* artiLinkBitFields = scDesc->mPathToRootBitFieldBlocks + maxLinks * wordSize * blockIdx.x;

	const PxU32 maxMimicJoints = scDesc->mMaxMimicJointsPerArticulation;
	PxgArticulationBlockMimicJointData* artiMmicJoints = scDesc->mArticulationMimicJointBlocks + maxMimicJoints * blockIdx.x;

	PxgArticulationBlockData& artiBlock = scDesc->mArticulationBlocks[blockIdx.x];

	const bool fixBase = artiBlock.mFlags[threadIndexInWarp] & PxArticulationFlag::eFIX_BASE;

	const PxU32 numMimicJoints = artiBlock.mNumMimicJoints[threadIndexInWarp];
	
	for (PxU32 i = 0; i < numMimicJoints; ++i)
	{
		const PxgArticulationBlockMimicJointData& mimicJointData = artiMmicJoints[i];
		const PxU32 linkA = mimicJointData.mLinkA[threadIndexInWarp];
		const PxU32 linkB = mimicJointData.mLinkB[threadIndexInWarp];
		const PxU32 dofA = mimicJointData.mInternalData.mDofA[threadIndexInWarp];
		const PxU32 dofB = mimicJointData.mInternalData.mDofB[threadIndexInWarp];
		const PxReal mimicJointRecipEffectiveInertia = mimicJointData.mInternalData.recipEffectiveInertia[threadIndexInWarp];
		const PxReal gearRatio = mimicJointData.mGearRatio[threadIndexInWarp];
		const PxReal offset = mimicJointData.mOffset[threadIndexInWarp];	
		const PxReal naturalFrequency = mimicJointData.mNaturalFrequency[threadIndexInWarp];
		const PxReal dampingRatio = mimicJointData.mDampingRatio[threadIndexInWarp];

		//Get the joint offsets.  We'll use these to gather the joint dof positions and speeds.
		const PxU32 jointOffsetA = artiLinks[linkA].mJointOffset[threadIndexInWarp];
		const PxU32 jointOffsetB = artiLinks[linkB].mJointOffset[threadIndexInWarp];

		//Get the joint positions.		
		//We don't care if are using PGS or TGS because we can directly query the latest joint position in either case.
		const PxReal qA = artiDofs[jointOffsetA + dofA].mJointPositions[threadIndexInWarp];
		const PxReal qB = artiDofs[jointOffsetB + dofB].mJointPositions[threadIndexInWarp];

		//Get the joint speeds.
		PxReal qADot = 0.0f;
		PxReal qBDot = 0.0f;
		{
			PxReal jointDofSpeedsA[3] = {0, 0, 0};
			PxReal jointDofSpeedsB[3] = {0, 0, 0};
			pxcFsGetVelocity(artiBlock, artiLinks, artiDofs, artiLinkBitFields, wordSize, linkA, fixBase, jointDofSpeedsA, threadIndexInWarp);
			pxcFsGetVelocity(artiBlock, artiLinks, artiDofs, artiLinkBitFields, wordSize, linkB, fixBase, jointDofSpeedsB, threadIndexInWarp);
			qADot = jointDofSpeedsA[dofA];
			qBDot = jointDofSpeedsB[dofB];
		}

		//We've got everything we need to compute the joint impulses.
		PxReal jointImpulseA[3] = {0, 0, 0};
		PxReal jointImpulseB[3] = {0, 0, 0};
		{
			PxReal jointImpDofA = 0.0f;
			PxReal jointImpDofB = 0.0f;
			computeMimicJointImpulses(
				biasCoefficient, dt, recipDt, 
				qA, qB, qADot, qBDot, 
				gearRatio, offset, naturalFrequency, dampingRatio, mimicJointRecipEffectiveInertia, 
				isVelocityIteration,
				jointImpDofA, jointImpDofB);
			jointImpulseA[dofA] = jointImpDofA;
			jointImpulseB[dofB] = jointImpDofB;
		}

		PxVec3 zero(0,0,0);
		pxcFsApplyImpulses(
			artiBlock, artiLinks, artiDofs, 
			artiLinkBitFields, wordSize, 
			linkA, zero, zero, jointImpulseA,
			linkB, zero, zero, jointImpulseB,
			threadIndexInWarp);
	}
}


extern "C" __global__
__launch_bounds__(WARP_SIZE, 12)
void artiSolveInternalTendonAndMimicJointConstraints1T(
const PxgArticulationCoreDesc* PX_RESTRICT scDesc, const PxReal biasCoefficient, const PxReal dt, const PxReal recipDt, const bool velocityIteration, const bool isTGS)
{
	const PxU32 nbArticulations = scDesc->nbArticulations;

	const PxU32 blockStride = blockDim.x;// / WARP_SIZE;

	//This identifies which warp a specific thread is in, we treat all warps in all blocks as a flatten warp array
	//and we are going to index the work based on that
	const PxU32 globalThreadIndex = blockIdx.x * blockStride + threadIdx.x;

	const PxU32 threadIndexInWarp = threadIdx.x;


	if (globalThreadIndex < nbArticulations)
	{
		PxgArticulationBlockData& blockData = scDesc->mArticulationBlocks[blockIdx.x];
		if (blockData.mNumSpatialTendons[threadIdx.x] || blockData.mNumFixedTendons[threadIdx.x] || blockData.mNumMimicJoints[threadIdx.x])
		{
			PxU32 dirtyFlag = blockData.mStateDirty[threadIdx.x];
			if (dirtyFlag & PxgArtiStateDirtyFlag::eHAS_IMPULSES)
			{
				const PxU32 numLinks = blockData.mNumLinks[threadIdx.x];
				PxgArticulationBlockLinkData* data = scDesc->mArticulationLinkBlocks + scDesc->mMaxLinksPerArticulation * blockIdx.x;
				PxgArticulationBlockDofData* dofData = scDesc->mArticulationDofBlocks + scDesc->mMaxDofsPerArticulation * blockIdx.x;
				averageLinkImpulsesAndPropagate(scDesc->slabHasChanges, scDesc->impulses, blockData, data, dofData, globalThreadIndex, scDesc->mMaxLinksPerArticulation,
					nbArticulations, scDesc->nbSlabs, numLinks, threadIdx.x);
			}

			blockData.mStateDirty[threadIdx.x] = PxgArtiStateDirtyFlag::eVEL_DIRTY;

			solveInternalSpatialConstraints(scDesc, isTGS, threadIndexInWarp);

			solveInternalFixedConstraints(scDesc, isTGS, threadIndexInWarp);

			solveInternalMimicJointConstraints(scDesc, biasCoefficient, dt, recipDt, velocityIteration, isTGS, threadIndexInWarp);
		}
	}
}


//two warp each block
extern "C" __global__
void artiSolveInternalConstraints1T(PxgArticulationCoreDesc* PX_RESTRICT scDesc, const PxReal dt,
	const PxReal invDt, const bool velocityIteration, const PxReal elapsedTime, const PxReal biasCoefficient, 
	const PxU32* const PX_RESTRICT staticContactUniqueIds,
	const PxU32* const PX_RESTRICT staticJointUniqueIds,
	const PxgSolverSharedDesc<IterativeSolveData>* const PX_RESTRICT sharedDesc,
	bool doFriction,
	bool residualReportingEnabled,
	bool isExternalForcesEveryTgsIterationEnabled)
{
	// This kernel also resets articulation reference counts to zero after all usage.
	if(residualReportingEnabled)
		artiSolveInternalConstraints1T<IterativeSolveData, false, true>(scDesc, dt, invDt, elapsedTime, velocityIteration, staticContactUniqueIds, staticJointUniqueIds,
			sharedDesc, biasCoefficient, doFriction, isExternalForcesEveryTgsIterationEnabled);
	else
		artiSolveInternalConstraints1T<IterativeSolveData, false, false>(scDesc, dt, invDt, elapsedTime, velocityIteration, staticContactUniqueIds, staticJointUniqueIds,
			sharedDesc, biasCoefficient, doFriction, isExternalForcesEveryTgsIterationEnabled);
}

extern "C" __global__
void artiSolveInternalConstraintsTGS1T(PxgArticulationCoreDesc* PX_RESTRICT scDesc, const PxReal dt,
	const PxReal invDt, const bool velocityIteration, const PxReal elapsedTime, const PxReal biasCoefficient,
	const PxU32* const PX_RESTRICT staticContactUniqueIds,
	const PxU32* const PX_RESTRICT staticJointUniqueIds,
	const PxgSolverSharedDesc<IterativeSolveDataTGS>* const PX_RESTRICT sharedDesc,
	bool doFriction, bool residualReportingEnabled,
	bool isExternalForceEveryStep)
{
	// This kernel also resets articulation reference counts to zero after all usage.
	const PxReal erp = PxMin(0.7f, biasCoefficient);
	if(residualReportingEnabled)
		artiSolveInternalConstraints1T<IterativeSolveDataTGS, true, true>(scDesc, dt, invDt, elapsedTime, velocityIteration, staticContactUniqueIds, staticJointUniqueIds,
			sharedDesc, erp, doFriction, isExternalForceEveryStep);
	else
		artiSolveInternalConstraints1T<IterativeSolveDataTGS, true, false>(scDesc, dt, invDt, elapsedTime, velocityIteration, staticContactUniqueIds, staticJointUniqueIds,
			sharedDesc, erp, doFriction, isExternalForceEveryStep);
}



//each block has 16 warps, each warp has 32 threads, 32 blocks
static __device__ void artiSumInternalContactAndJointBatches1(const PxU32* const PX_RESTRICT staticContactCount, const PxU32* const PX_RESTRICT staticJointCount,
	const PxU32* const PX_RESTRICT selfContactCounts, const PxU32* const PX_RESTRICT selfJointCounts,
	const PxU32 nbArticulations, PxU32* tempStaticContactUniqueIndicesBlockSum, PxU32* tempStaticJointUniqueIndicesBlockSum,
	PxU32* tempStaticContactHeaderBlockSum, PxU32* tempStaticJointHeaderBlockSum,
	PxU32* tempSelfContactUniqueIndicesBlockSum, PxU32* tempSelfJointUniqueIndicesBlockSum,
	PxU32* tempSelfContactHeaderBlockSum, PxU32* tempSelfJointHeaderBlockSum)
{

	const PxU32 numThreadsPerBlock = PxgArticulationCoreKernelBlockDim::COMPUTE_STATIC_CONTACT_CONSTRAINT_COUNT;

	const PxU32 warpPerBlock = numThreadsPerBlock / WARP_SIZE;

	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);
	const PxU32 warpIndex = threadIdx.x / WARP_SIZE;

	const PxU32 block_size = 32;

	const PxU32 totalBlockRequired = (nbArticulations + (numThreadsPerBlock - 1)) / numThreadsPerBlock;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (block_size - 1)) / block_size;



	__shared__ PxU32 shContactUniqueIndicesWarpSum[warpPerBlock];
	__shared__ PxU32 shJointUniqueIndicesWarpSum[warpPerBlock];
	__shared__ PxU32 shContactHeaderWarpSum[warpPerBlock];
	__shared__ PxU32 shJointHeaderWarpSum[warpPerBlock];

	__shared__ PxU32 sContactUniqueIndicesAccum;
	__shared__ PxU32 sJointUniqueIndicesAccum;
	__shared__ PxU32 sContactHeaderAccum;
	__shared__ PxU32 sJointHeaderAccum;


	__shared__ PxU32 shSelfContactUniqueIndicesWarpSum[warpPerBlock];
	__shared__ PxU32 shSelfJointUniqueIndicesWarpSum[warpPerBlock];
	__shared__ PxU32 shSelfContactHeaderWarpSum[warpPerBlock];
	__shared__ PxU32 shSelfJointHeaderWarpSum[warpPerBlock];

	__shared__ PxU32 sSelfContactUniqueIndicesAccum;
	__shared__ PxU32 sSelfJointUniqueIndicesAccum;
	__shared__ PxU32 sSelfContactHeaderAccum;
	__shared__ PxU32 sSelfJointHeaderAccum;


	if (threadIdx.x == (WARP_SIZE - 1))
	{
		sContactUniqueIndicesAccum = 0;
		sJointUniqueIndicesAccum = 0;
		sContactHeaderAccum = 0;
		sJointHeaderAccum = 0;
		sSelfContactUniqueIndicesAccum = 0;
		sSelfJointUniqueIndicesAccum = 0;
		sSelfContactHeaderAccum = 0;
		sSelfJointHeaderAccum = 0;
	}

	__syncthreads();

	for (PxU32 i = 0; i < numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + threadIdx.x + numIterationPerBlock * blockIdx.x * blockDim.x;

		PxU32 contactCount = 0;
		PxU32 jointCount = 0;
		PxU32 selfContactCount = 0;
		PxU32 selfJointCount = 0;

		if (workIndex < nbArticulations)
		{
			contactCount = staticContactCount[workIndex];
			jointCount = staticJointCount[workIndex];

			selfContactCount = selfContactCounts[workIndex];
			selfJointCount = selfJointCounts[workIndex];
		}


		PxU32 maxContact = contactCount;
		PxU32 maxJoint = jointCount;
		PxU32 maxSelfContact = selfContactCount;
		PxU32 maxSelfJoint = selfJointCount;

		contactCount = warpReduction<AddOpPxU32, PxU32>(FULL_MASK, contactCount);
		jointCount = warpReduction<AddOpPxU32, PxU32>(FULL_MASK, jointCount);
		maxContact = warpReduction<MaxOpPxU32, PxU32>(FULL_MASK, maxContact);
		maxJoint = warpReduction<MaxOpPxU32, PxU32>(FULL_MASK, maxJoint);

		selfContactCount = warpReduction<AddOpPxU32, PxU32>(FULL_MASK, selfContactCount);
		selfJointCount = warpReduction<AddOpPxU32, PxU32>(FULL_MASK, selfJointCount);
		maxSelfContact = warpReduction<MaxOpPxU32, PxU32>(FULL_MASK, maxSelfContact);
		maxSelfJoint = warpReduction<MaxOpPxU32, PxU32>(FULL_MASK, maxSelfJoint);

		if (threadIndexInWarp == (WARP_SIZE - 1))
		{
			shContactUniqueIndicesWarpSum[warpIndex] = contactCount;
			shJointUniqueIndicesWarpSum[warpIndex] = jointCount;
			shContactHeaderWarpSum[warpIndex] = maxContact;
			shJointHeaderWarpSum[warpIndex] = maxJoint;

			shSelfContactUniqueIndicesWarpSum[warpIndex] = selfContactCount;
			shSelfJointUniqueIndicesWarpSum[warpIndex] = selfJointCount;
			shSelfContactHeaderWarpSum[warpIndex] = maxSelfContact;
			shSelfJointHeaderWarpSum[warpIndex] = maxSelfJoint;
		}


		__syncthreads();

		unsigned mask_idx = __ballot_sync(FULL_MASK, threadIndexInWarp < warpPerBlock);

		if (threadIdx.x < warpPerBlock)
		{
			PxU32 contactUniqueIndicesWarpSum = shContactUniqueIndicesWarpSum[threadIndexInWarp];
			PxU32 jointUniqueIndicesWarpSum = shJointUniqueIndicesWarpSum[threadIndexInWarp];
			PxU32 contactHeaderWarpSum = shContactHeaderWarpSum[threadIndexInWarp];
			PxU32 jointHeaderWarpSum = shJointHeaderWarpSum[threadIndexInWarp];

			PxU32 selfContactUniqueIndicesWarpSum = shSelfContactUniqueIndicesWarpSum[threadIndexInWarp];
			PxU32 selfJointUniqueIndicesWarpSum = shSelfJointUniqueIndicesWarpSum[threadIndexInWarp];
			PxU32 selfContactHeaderWarpSum = shSelfContactHeaderWarpSum[threadIndexInWarp];
			PxU32 selfJointHeaderWarpSum = shSelfJointHeaderWarpSum[threadIndexInWarp];

			contactUniqueIndicesWarpSum = warpReduction<AddOpPxU32, PxU32>(mask_idx, contactUniqueIndicesWarpSum);
			jointUniqueIndicesWarpSum = warpReduction<AddOpPxU32, PxU32>(mask_idx, jointUniqueIndicesWarpSum);
			contactHeaderWarpSum = warpReduction<AddOpPxU32, PxU32>(mask_idx, contactHeaderWarpSum);
			jointHeaderWarpSum = warpReduction<AddOpPxU32, PxU32>(mask_idx, jointHeaderWarpSum);

			selfContactUniqueIndicesWarpSum = warpReduction<AddOpPxU32, PxU32>(mask_idx, selfContactUniqueIndicesWarpSum);
			selfJointUniqueIndicesWarpSum = warpReduction<AddOpPxU32, PxU32>(mask_idx, selfJointUniqueIndicesWarpSum);
			selfContactHeaderWarpSum = warpReduction<AddOpPxU32, PxU32>(mask_idx, selfContactHeaderWarpSum);
			selfJointHeaderWarpSum = warpReduction<AddOpPxU32, PxU32>(mask_idx, selfJointHeaderWarpSum);


			if (threadIdx.x == (warpPerBlock - 1))
			{
				sContactUniqueIndicesAccum += contactUniqueIndicesWarpSum;
				sJointUniqueIndicesAccum += jointUniqueIndicesWarpSum;
				sContactHeaderAccum += contactHeaderWarpSum;
				sJointHeaderAccum += jointHeaderWarpSum;

				sSelfContactUniqueIndicesAccum += selfContactUniqueIndicesWarpSum;
				sSelfJointUniqueIndicesAccum += selfJointUniqueIndicesWarpSum;
				sSelfContactHeaderAccum += selfContactHeaderWarpSum;
				sSelfJointHeaderAccum += selfJointHeaderWarpSum;
			}
		}

		__syncthreads();

	}

	if (threadIdx.x == (warpPerBlock - 1))
	{
		tempStaticContactUniqueIndicesBlockSum[blockIdx.x] = sContactUniqueIndicesAccum;
		tempStaticJointUniqueIndicesBlockSum[blockIdx.x] = sJointUniqueIndicesAccum;
		tempStaticContactHeaderBlockSum[blockIdx.x] = sContactHeaderAccum;
		tempStaticJointHeaderBlockSum[blockIdx.x] = sJointHeaderAccum;

		tempSelfContactUniqueIndicesBlockSum[blockIdx.x] = sSelfContactUniqueIndicesAccum;
		tempSelfJointUniqueIndicesBlockSum[blockIdx.x] = sSelfJointUniqueIndicesAccum;
		tempSelfContactHeaderBlockSum[blockIdx.x] = sSelfContactHeaderAccum;
		tempSelfJointHeaderBlockSum[blockIdx.x] = sSelfJointHeaderAccum;
	}
}


static __device__ void artiSumInternalContactAndJointBatches2(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	PxgSolverCoreDesc* PX_RESTRICT solverCoreDesc,
	const PxU32* const PX_RESTRICT staticContactCount,
	const PxU32* const PX_RESTRICT staticJointCount,
	PxgConstraintBatchHeader* PX_RESTRICT batchHeaders, 
	const PxU32* const PX_RESTRICT contactStaticUniqueIds,
	const PxU32* const PX_RESTRICT jointStaticUniqueIndices,
	PartitionNodeData* const PX_RESTRICT pNodeData,
	const PxU32 nbArticulations, 
	PxU32* tempStaticContactUniqueIndicesBlock, 
	PxU32* tempStaticJointUniqueIndicesBlock,
	PxU32* tempStaticContactHeaderBlock, 
	PxU32* tempStaticJointHeaderBlock, 
	PxU32 articulationStaticContactBatchOffset,
	PxU32 articulationStaticJointBatchOffset,
	PxU32 articulationBatchOffset,
	PxU32 contactUniqueIndexOffset, 
	PxU32 jointUniqueIndexOffset,
	PxU32* outContactUniqueIds,
	PxU32* outJointUniqueIndices,
	PxU32& outNumArtiStaticBatches,
	PxgConstraintPrepareDesc* constraintPrepDesc,
	const PxU32 numRigidBatches,
	const PxU32 numArtiContactBatches,
	const PxU32 numArtiJointBatches,
	const PxU32 numRigidContacts,
	const PxU32 numRigidJoints,
	const PxU32 numRigidStaticContacts,
	const PxU32 numRigidStaticJoints)
{

	const PxU32 numThreadsPerBlock = PxgArticulationCoreKernelBlockDim::COMPUTE_STATIC_CONTACT_CONSTRAINT_COUNT;

	const PxU32 warpPerBlock = numThreadsPerBlock / WARP_SIZE;

	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);

	const PxU32 warpIndex = threadIdx.x / WARP_SIZE;

	const PxU32 block_size = 32; // 32 blocks

	__shared__ PxU32 shContactUniqueIndicesWarpSum[warpPerBlock];
	__shared__ PxU32 shJointUniqueIndicesWarpSum[warpPerBlock];
	__shared__ PxU32 shContactHeaderWarpSum[warpPerBlock];
	__shared__ PxU32 shJointHeaderWarpSum[warpPerBlock];

	__shared__ PxU32 sContactUniqueIndicesBlockHistogram[block_size];
	__shared__ PxU32 sJointUniqueIndicesBlockHistogram[block_size];
	__shared__ PxU32 sContactHeaderBlockHistogram[block_size];
	__shared__ PxU32 sJointHeaderBlockHistogram[block_size];

	__shared__ PxU32 sContactUniqueIndicesAccum;
	__shared__ PxU32 sJointUniqueIndicesAccum;
	__shared__ PxU32 sContactHeaderAccum;
	__shared__ PxU32 sJointHeaderAccum;

	PxU32* artiJointConstraintBatchIndices = constraintPrepDesc->artiJointConstraintBatchIndices;
	PxU32* artiContactConstraintBatchIndices = constraintPrepDesc->artiContactConstraintBatchIndices;


	if (threadIdx.x == (WARP_SIZE - 1))
	{
		sContactUniqueIndicesAccum = 0;
		sJointUniqueIndicesAccum = 0;
		sContactHeaderAccum = 0;
		sJointHeaderAccum = 0;
	}


	//accumulate num pairs per block and compute exclusive run sum
	//unsigned mask_idx = __ballot_sync(FULL_MASK, threadIndexInWarp < block_size);
	if (warpIndex == 0/* && threadIndexInWarp < block_size*/)
	{
		const PxU32 oriContactUniqueIndiceOffset = tempStaticContactUniqueIndicesBlock[threadIndexInWarp];
		const PxU32 oriJointUniqueIndiceOffset = tempStaticJointUniqueIndicesBlock[threadIndexInWarp];
		const PxU32 oriContactHeaderOffset = tempStaticContactHeaderBlock[threadIndexInWarp];
		const PxU32 oriJointHeaderOffset = tempStaticJointHeaderBlock[threadIndexInWarp];

		const PxU32 contactUniqueIndiceOffset = warpScan<AddOpPxU32, PxU32>(FULL_MASK, oriContactUniqueIndiceOffset);
		const PxU32 jointUniqueIndiceOffset = warpScan<AddOpPxU32, PxU32>(FULL_MASK, oriJointUniqueIndiceOffset) ;
		const PxU32 contactHeaderOffset = warpScan<AddOpPxU32, PxU32>(FULL_MASK, oriContactHeaderOffset);
		const PxU32 jointHeaderOffset = warpScan<AddOpPxU32, PxU32>(FULL_MASK, oriJointHeaderOffset);
		//store exclusive run sum
		sContactUniqueIndicesBlockHistogram[threadIndexInWarp] = contactUniqueIndiceOffset - oriContactUniqueIndiceOffset;
		sJointUniqueIndicesBlockHistogram[threadIndexInWarp] = jointUniqueIndiceOffset - oriJointUniqueIndiceOffset;
		sContactHeaderBlockHistogram[threadIndexInWarp] = contactHeaderOffset - oriContactHeaderOffset;
		sJointHeaderBlockHistogram[threadIndexInWarp] = jointHeaderOffset - oriJointHeaderOffset;

		if (blockIdx.x == 0 && threadIdx.x == (WARP_SIZE - 1))
		{
			//Output total number of articulation static blocks
			const PxU32 totalNumArtiStaticBatches = contactHeaderOffset + jointHeaderOffset;
			outNumArtiStaticBatches = totalNumArtiStaticBatches;

			constraintPrepDesc->numArtiStaticContactBatches = contactHeaderOffset;
			constraintPrepDesc->numArtiStatic1dConstraintBatches = jointHeaderOffset;

			PxgIslandContext& island = solverCoreDesc->islandContextPool[0];
			island.mStaticArtiBatchCount = totalNumArtiStaticBatches;
		}
		
	}

	__syncthreads();

	//We now have the exclusive runsum for this block. Next step is to recompute the local
	//offsets within the block and output data...

	const PxU32 totalBlockRequired = (nbArticulations + (blockDim.x - 1)) / blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (block_size - 1)) / block_size;

	for (PxU32 a = 0; a < numIterationPerBlock; ++a)
	{
		const PxU32 workIndex = a * blockDim.x + threadIdx.x + numIterationPerBlock * blockIdx.x * blockDim.x;

		PxU32 contactCount = 0;
		PxU32 jointCount = 0;

		PxgArticulationBlockLinkData* data = scDesc->mArticulationLinkBlocks + scDesc->mMaxLinksPerArticulation * (workIndex / WARP_SIZE);

		if (workIndex < nbArticulations)
		{
			contactCount = staticContactCount[workIndex];
			jointCount = staticJointCount[workIndex];
		}

		//we need to use contactCount and jointCount later
		PxU32 sumContact = contactCount;
		PxU32 sumJoint = jointCount;
		PxU32 maxContact = contactCount;
		PxU32 maxJoint = jointCount;

		sumContact = warpReduction<AddOpPxU32, PxU32>(FULL_MASK, sumContact);
		sumJoint = warpReduction<AddOpPxU32, PxU32>(FULL_MASK, sumJoint);
		maxContact = warpReduction<MaxOpPxU32, PxU32>(FULL_MASK, maxContact);
		maxJoint = warpReduction<MaxOpPxU32, PxU32>(FULL_MASK, maxJoint);


		if (threadIndexInWarp == 31)
		{
			shContactUniqueIndicesWarpSum[warpIndex] = sumContact;
			shJointUniqueIndicesWarpSum[warpIndex] = sumJoint;
			shContactHeaderWarpSum[warpIndex] = maxContact;
			shJointHeaderWarpSum[warpIndex] = maxJoint;
		}

		__syncthreads();

		PxU32 contactWarpOffset = 0;
		PxU32 jointWarpOffset = 0;
		PxU32 contactBlockWarpOffset = 0;
		PxU32 jointBlockWarpOffset = 0;

		PxU32 contactUniqueIndicesWarpSum = 0;
		PxU32 jointUniqueIndicesWarpSum = 0;
		PxU32 contactHeaderWarpSum = 0;
		PxU32 jointHeaderWarpSum = 0;

		unsigned mask_idx = __ballot_sync(FULL_MASK, threadIndexInWarp < warpPerBlock);

		//warpPerBlock should be less than 32, each warp will do the runsum
		if (threadIndexInWarp < warpPerBlock)
		{

			const PxU32 oriContactUniqueIndicesWarpSum = shContactUniqueIndicesWarpSum[threadIndexInWarp];
			const PxU32 oriJointUniqueIndicesWarpSum = shJointUniqueIndicesWarpSum[threadIndexInWarp];
			const PxU32 oriContactHeaderWarpSum = shContactHeaderWarpSum[threadIndexInWarp];
			const PxU32 oriJointHeaderWarpSum = shJointHeaderWarpSum[threadIndexInWarp];

			contactUniqueIndicesWarpSum = warpScan<AddOpPxU32, PxU32>(mask_idx, oriContactUniqueIndicesWarpSum);
			jointUniqueIndicesWarpSum = warpScan<AddOpPxU32, PxU32>(mask_idx, oriJointUniqueIndicesWarpSum);
			contactHeaderWarpSum = warpScan<AddOpPxU32, PxU32>(mask_idx, oriContactHeaderWarpSum);
			jointHeaderWarpSum = warpScan<AddOpPxU32, PxU32>(mask_idx, oriJointHeaderWarpSum);


			//exclusive runsum
			contactWarpOffset = contactUniqueIndicesWarpSum - oriContactUniqueIndicesWarpSum;
			jointWarpOffset = jointUniqueIndicesWarpSum - oriJointUniqueIndicesWarpSum;
			contactBlockWarpOffset = contactHeaderWarpSum - oriContactHeaderWarpSum;
			jointBlockWarpOffset = jointHeaderWarpSum - oriJointHeaderWarpSum;


		}

		//make sure each thread in a warp has the correct warp offset
		contactWarpOffset = __shfl_sync(FULL_MASK, contactWarpOffset, warpIndex);
		jointWarpOffset = __shfl_sync(FULL_MASK, jointWarpOffset, warpIndex);
		contactBlockWarpOffset = __shfl_sync(FULL_MASK, contactBlockWarpOffset, warpIndex);
		jointBlockWarpOffset = __shfl_sync(FULL_MASK, jointBlockWarpOffset, warpIndex);

		//OK. We finally have enough information to figure out where to write the blocks related to this
		//articulation!

		//Where the contact uniqueIds should go. This is the start of where this warp should write.
		//The uids will be interleaved depending on the number of constraints in a contact block
		//contactUniqueIndexOffset : articulation static contact start offset
		//sContactUniqueIndicesAccum :: accumulation from the previous iterations
		PxU32 contactOffset = contactUniqueIndexOffset + contactWarpOffset + sContactUniqueIndicesBlockHistogram[blockIdx.x] + sContactUniqueIndicesAccum;
		//Where the joint unique Ids should go. See above for explanation
		PxU32 jointOffset = jointUniqueIndexOffset + jointWarpOffset + sJointUniqueIndicesBlockHistogram[blockIdx.x] + sJointUniqueIndicesAccum;
		//Where the blocks should go. Shared between all threads in a block
		PxU32 contactBlockOffset = contactBlockWarpOffset + sContactHeaderBlockHistogram[blockIdx.x] + sContactHeaderAccum ;
		PxU32 jointBlockOffset = jointBlockWarpOffset + sJointHeaderBlockHistogram[blockIdx.x] + sJointHeaderAccum;

		PxU32 blockOffset = contactBlockOffset + jointBlockOffset + articulationBatchOffset;
		

		//we have to sync in here so all the threads in a warp has finished reading sContactUniqueIndicesAccum, sJointUniqueIndicesAccum,
		//sContactHeaderAccum, sJointHeaderAccum before we overwrite those values for another iterations
		__syncthreads();

		if (threadIdx.x == (warpPerBlock - 1))
		{
			sContactUniqueIndicesAccum += contactUniqueIndicesWarpSum;
			sJointUniqueIndicesAccum += jointUniqueIndicesWarpSum;
			sContactHeaderAccum += contactHeaderWarpSum;
			sJointHeaderAccum += jointHeaderWarpSum;
		}

		__syncthreads();


		for (PxU32 i = 0; i < maxContact; ++i)
		{
			PxU32 mask = __ballot_sync(FULL_MASK, contactCount > i);

			const PxU32 stride = __popc(mask);
			const PxU32 offset = warpScanExclusive(mask, threadIndexInWarp);

			if (contactCount > i)
			{
				PxU32 contactUniqueId = contactStaticUniqueIds[workIndex + nbArticulations * i];
				outContactUniqueIds[contactOffset + offset] = contactUniqueId;

				const PartitionNodeData& nodeData = pNodeData[contactUniqueId];
				PxNodeIndex igNodeIndexA = nodeData.mNodeIndex0;
				PxNodeIndex igNodeIndexB = nodeData.mNodeIndex1;

				if (igNodeIndexA.isArticulation())
				{
					const PxU32 artiLinkID = igNodeIndexA.articulationLinkId();
				
					if (data[artiLinkID].mNbStaticContacts[threadIndexInWarp] == 0)
					{
						data[artiLinkID].mStaticContactStartIndex[threadIndexInWarp] = blockOffset;
					}
					data[artiLinkID].mNbStaticContacts[threadIndexInWarp]++;
				}

				if (igNodeIndexB.isArticulation())
				{
					const PxU32 artiLinkID = igNodeIndexB.articulationLinkId();

					
					if (data[artiLinkID].mNbStaticContacts[threadIndexInWarp] == 0)
					{
						data[artiLinkID].mStaticContactStartIndex[threadIndexInWarp] = blockOffset;
					}
					data[artiLinkID].mNbStaticContacts[threadIndexInWarp]++;
				}
			}

			if (threadIndexInWarp == 0)
			{
				const PxU32 batchIndex = contactBlockOffset + i + articulationStaticContactBatchOffset;
				PxgConstraintBatchHeader header;
				header.mDescStride = stride;
				header.constraintType = PxgSolverConstraintDesc::eARTICULATION_CONTACT;
				header.mConstraintBatchIndex = batchIndex;
				header.mStartPartitionIndex = contactOffset - numRigidContacts - numRigidStaticContacts;
				header.mask = mask;
				batchHeaders[blockOffset] = header;
				artiContactConstraintBatchIndices[contactBlockOffset + numArtiContactBatches + i] = blockOffset - numRigidBatches;
			}

			contactOffset += stride;
			blockOffset++;
		}

		for (PxU32 i = 0; i < maxJoint; ++i)
		{
			PxU32 mask = __ballot_sync(FULL_MASK, jointCount > i);

			const PxU32 stride = __popc(mask);
			const PxU32 offset = warpScanExclusive(mask, threadIndexInWarp);

			if (jointCount > i)
			{
				PxU32 jointUniqueId = jointStaticUniqueIndices[workIndex + nbArticulations * i];
				outJointUniqueIndices[jointOffset + offset] = jointUniqueId;

				const PartitionNodeData& nodeData = pNodeData[jointUniqueId];
				PxNodeIndex igNodeIndexA = nodeData.mNodeIndex0;
				PxNodeIndex igNodeIndexB = nodeData.mNodeIndex1;

				if (igNodeIndexA.isArticulation())
				{
					const PxU32 artiLinkID = igNodeIndexA.articulationLinkId();
					if (data[artiLinkID].mNbStaticJoints[threadIndexInWarp] == 0)
					{
						data[artiLinkID].mStaticJointStartIndex[threadIndexInWarp] = blockOffset;
					}
					data[artiLinkID].mNbStaticJoints[threadIndexInWarp]++;
				}

				if (igNodeIndexB.isArticulation())
				{
					const PxU32 artiLinkID = igNodeIndexB.articulationLinkId();
					if (data[artiLinkID].mNbStaticJoints[threadIndexInWarp] == 0)
					{
						data[artiLinkID].mStaticJointStartIndex[threadIndexInWarp] = blockOffset;
					}
					data[artiLinkID].mNbStaticJoints[threadIndexInWarp]++;
				}
			}

			if (threadIndexInWarp == 0)
			{
				PxgConstraintBatchHeader header;
				header.mDescStride = stride;
				const PxU32 batchIndex = jointBlockOffset + i + articulationStaticJointBatchOffset;
				header.constraintType = PxgSolverConstraintDesc::eARTICULATION_CONSTRAINT_1D;
				header.mConstraintBatchIndex = batchIndex;
				header.mStartPartitionIndex = jointOffset - numRigidJoints - numRigidStaticJoints;
				header.mask = mask;
				batchHeaders[blockOffset] = header;
				artiJointConstraintBatchIndices[jointBlockOffset + numArtiJointBatches + i] = blockOffset - numRigidBatches;
			}

			jointOffset += stride;
			blockOffset++;
		}
	}
}


extern "C" __global__ 
__launch_bounds__(PxgArticulationCoreKernelBlockDim::COMPUTE_STATIC_CONTACT_CONSTRAINT_COUNT, 1) 
void artiSumInternalContactAndJointBatches1Launch(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	PxgPrePrepDesc* prePrepDesc, 
	const PxU32 nbArticulations)
{	
	artiSumInternalContactAndJointBatches1(
		prePrepDesc->mArtiStaticContactCounts,
		prePrepDesc->mArtiStaticConstraintCounts,
		prePrepDesc->mArtiSelfContactCounts,
		prePrepDesc->mArtiSelfConstraintCounts,
		nbArticulations, scDesc->mTempContactUniqueIndicesBlock, scDesc->mTempConstraintUniqueIndicesBlock,
		scDesc->mTempContactHeaderBlock, scDesc->mTempConstraintHeaderBlock,
		scDesc->mTempSelfContactUniqueIndicesBlock, scDesc->mTempSelfConstraintUniqueIndicesBlock,
		scDesc->mTempSelfContactHeaderBlock, scDesc->mTempSelfConstraintHeaderBlock);
}


extern "C" __global__ 
__launch_bounds__(PxgArticulationCoreKernelBlockDim::COMPUTE_STATIC_CONTACT_CONSTRAINT_COUNT, 1)
void artiSumInternalContactAndJointBatches2Launch(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	PxgSolverCoreDesc* PX_RESTRICT solverCoreDesc,
	PxgPrePrepDesc* PX_RESTRICT prePrepDesc,
	PxgConstraintPrepareDesc* PX_RESTRICT constraintPrepDesc,
	const PxU32 nbArticulations
	)
{
	
	const PxU32 numRigidContacts = prePrepDesc->numTotalContacts;
	const PxU32 numRigidJoints = prePrepDesc->numTotalConstraints;
	const PxU32 articulationBatchOffset = prePrepDesc->numBatches + prePrepDesc->numArtiBatches;
	const PxU32 contactUniqueIndexOffset = numRigidContacts + prePrepDesc->numTotalStaticContacts + prePrepDesc->numTotalArtiContacts;
	const PxU32 jointUniqueIndexOffset = numRigidJoints + prePrepDesc->numTotalStaticConstraints + prePrepDesc->numTotalArtiConstraints;

	const PxU32 artiStaticContactBatchOffset = prePrepDesc->artiStaticContactBatchOffset;
	const PxU32 artiStaticConstraintBatchOffset = prePrepDesc->artiStaticConstraintBatchOffset;

	artiSumInternalContactAndJointBatches2(
		scDesc,
		solverCoreDesc,
		prePrepDesc->mArtiStaticContactCounts,
		prePrepDesc->mArtiStaticConstraintCounts,
		prePrepDesc->mBatchHeaders,
		prePrepDesc->mArtiStaticContactIndices,     //stride static contacts(nbArticulations)
		prePrepDesc->mArtiStaticConstraintIndices,  //stride external constraints(nbArticulations)
		prePrepDesc->mPartitionNodeData,
		nbArticulations,
		scDesc->mTempContactUniqueIndicesBlock,
		scDesc->mTempConstraintUniqueIndicesBlock,
		scDesc->mTempContactHeaderBlock,
		scDesc->mTempConstraintHeaderBlock,
		artiStaticContactBatchOffset,
		artiStaticConstraintBatchOffset,
		articulationBatchOffset,
		contactUniqueIndexOffset,
		jointUniqueIndexOffset,
		prePrepDesc->mContactUniqueIndices,
		prePrepDesc->mConstraintUniqueIndices,
		prePrepDesc->numArtiStaticBatches,
		constraintPrepDesc,
		constraintPrepDesc->numBatches,
		constraintPrepDesc->numArtiContactBatches,
		constraintPrepDesc->numArti1dConstraintBatches,
		numRigidContacts,
		numRigidJoints,
		prePrepDesc->numTotalStaticContacts,
		prePrepDesc->numTotalStaticConstraints
	);
}

extern "C" __global__ 
__launch_bounds__(PxgArticulationCoreKernelBlockDim::COMPUTE_STATIC_CONTACT_CONSTRAINT_COUNT, 1)
void artiSumSelfContactAndJointBatches(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	PxgSolverCoreDesc* PX_RESTRICT solverCoreDesc,
	PxgPrePrepDesc* PX_RESTRICT prePrepDesc,
	PxgConstraintPrepareDesc* PX_RESTRICT constraintPrepDesc,
	const PxU32 nbArticulations
)
{
	//The first stage of this ran with the static contact code.
	//We output self-contacts *after* static contacts in the batches.
	//We don't need to sum up the number of batches per link. We can instead
	//process just in a single iteration, making this code simpler.

	PxU32* tempSelfContactUniqueIndicesBlock = scDesc->mTempSelfContactUniqueIndicesBlock;
	PxU32* tempSelfJointUniqueIndicesBlock = scDesc->mTempSelfConstraintUniqueIndicesBlock;
	PxU32* tempSelfContactHeaderBlock = scDesc->mTempSelfContactHeaderBlock;
	PxU32* tempSelfJointHeaderBlock = scDesc->mTempSelfConstraintHeaderBlock;

	const PxU32 numRigidContacts = prePrepDesc->numTotalContacts;
	const PxU32 numRigidJoints = prePrepDesc->numTotalConstraints;

	const PxU32 numRigidStaticContacts = prePrepDesc->numTotalStaticContacts;
	const PxU32 numRigidStaticJoints = prePrepDesc->numTotalStaticConstraints;

	const PxU32 articulationBatchOffset = prePrepDesc->numBatches + prePrepDesc->numArtiBatches + prePrepDesc->numArtiStaticBatches;
	const PxU32 contactUniqueIndexOffset = numRigidContacts + prePrepDesc->numTotalStaticContacts + prePrepDesc->numTotalArtiContacts + prePrepDesc->numTotalStaticArtiContacts;
	const PxU32 jointUniqueIndexOffset = numRigidJoints + prePrepDesc->numTotalStaticConstraints + prePrepDesc->numTotalArtiConstraints + prePrepDesc->numTotalStaticArtiConstraints;


	const PxU32 numThreadsPerBlock = PxgArticulationCoreKernelBlockDim::COMPUTE_STATIC_CONTACT_CONSTRAINT_COUNT;

	const PxU32 warpPerBlock = numThreadsPerBlock / WARP_SIZE;

	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);

	const PxU32 warpIndex = threadIdx.x / WARP_SIZE;

	const PxU32 block_size = 32; // 32 blocks

	__shared__ PxU32 shContactUniqueIndicesWarpSum[warpPerBlock];
	__shared__ PxU32 shJointUniqueIndicesWarpSum[warpPerBlock];
	__shared__ PxU32 shContactHeaderWarpSum[warpPerBlock];
	__shared__ PxU32 shJointHeaderWarpSum[warpPerBlock];

	__shared__ PxU32 sContactUniqueIndicesBlockHistogram[block_size];
	__shared__ PxU32 sJointUniqueIndicesBlockHistogram[block_size];
	__shared__ PxU32 sContactHeaderBlockHistogram[block_size];
	__shared__ PxU32 sJointHeaderBlockHistogram[block_size];

	__shared__ PxU32 sContactUniqueIndicesAccum;
	__shared__ PxU32 sJointUniqueIndicesAccum;
	__shared__ PxU32 sContactHeaderAccum;
	__shared__ PxU32 sJointHeaderAccum;

	PxU32* artiJointConstraintBatchIndices = constraintPrepDesc->artiJointConstraintBatchIndices;
	PxU32* artiContactConstraintBatchIndices = constraintPrepDesc->artiContactConstraintBatchIndices;


	if (threadIdx.x == (WARP_SIZE - 1))
	{
		sContactUniqueIndicesAccum = 0;
		sJointUniqueIndicesAccum = 0;
		sContactHeaderAccum = 0;
		sJointHeaderAccum = 0;
	}


	//accumulate num pairs per block and compute exclusive run sum
	//unsigned mask_idx = __ballot_sync(FULL_MASK, threadIndexInWarp < block_size);
	if (warpIndex == 0/* && threadIndexInWarp < block_size*/)
	{
		const PxU32 oriContactUniqueIndiceOffset = tempSelfContactUniqueIndicesBlock[threadIndexInWarp];
		const PxU32 oriJointUniqueIndiceOffset = tempSelfJointUniqueIndicesBlock[threadIndexInWarp];
		const PxU32 oriContactHeaderOffset = tempSelfContactHeaderBlock[threadIndexInWarp];
		const PxU32 oriJointHeaderOffset = tempSelfJointHeaderBlock[threadIndexInWarp];

		const PxU32 contactUniqueIndiceOffset = warpScan<AddOpPxU32, PxU32>(FULL_MASK, oriContactUniqueIndiceOffset);
		const PxU32 jointUniqueIndiceOffset = warpScan<AddOpPxU32, PxU32>(FULL_MASK, oriJointUniqueIndiceOffset);
		const PxU32 contactHeaderOffset = warpScan<AddOpPxU32, PxU32>(FULL_MASK, oriContactHeaderOffset);
		const PxU32 jointHeaderOffset = warpScan<AddOpPxU32, PxU32>(FULL_MASK, oriJointHeaderOffset);
		//store exclusive run sum
		sContactUniqueIndicesBlockHistogram[threadIndexInWarp] = contactUniqueIndiceOffset - oriContactUniqueIndiceOffset;
		sJointUniqueIndicesBlockHistogram[threadIndexInWarp] = jointUniqueIndiceOffset - oriJointUniqueIndiceOffset;
		sContactHeaderBlockHistogram[threadIndexInWarp] = contactHeaderOffset - oriContactHeaderOffset;
		sJointHeaderBlockHistogram[threadIndexInWarp] = jointHeaderOffset - oriJointHeaderOffset;

		if (blockIdx.x == 0 && threadIdx.x == (WARP_SIZE - 1))
		{
			//Output total number of articulation static blocks
			const PxU32 totalNumArtiSelfBatches = contactHeaderOffset + jointHeaderOffset;
			prePrepDesc->numArtiSelfBatches = totalNumArtiSelfBatches;

			constraintPrepDesc->numArtiSelfContactBatches = contactHeaderOffset;
			constraintPrepDesc->numArtiSelf1dConstraintBatches = jointHeaderOffset;

			PxgIslandContext& island = solverCoreDesc->islandContextPool[0];
			island.mSelfArtiBatchCount = totalNumArtiSelfBatches;
		}

	}

	__syncthreads();

	//We now have the exclusive runsum for this block. Next step is to recompute the local
	//offsets within the block and output data...

	const PxU32 totalBlockRequired = (nbArticulations + (blockDim.x - 1)) / blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (block_size - 1)) / block_size;

	const PxU32 numArtiStaticContacts = constraintPrepDesc->numArtiStaticContactBatches;
	const PxU32 numArtiStaticJoints = constraintPrepDesc->numArtiStatic1dConstraintBatches;
	const PxU32 artiSelfContactBatchOffset = prePrepDesc->artiStaticContactBatchOffset + numArtiStaticContacts;
	const PxU32 artiSelfJointBatchOffset = prePrepDesc->artiStaticConstraintBatchOffset + numArtiStaticJoints;

	PxU32* selfContactCount = prePrepDesc->mArtiSelfContactCounts;
	PxU32* selfJointCount = prePrepDesc->mArtiSelfConstraintCounts;

	for (PxU32 a = 0; a < numIterationPerBlock; ++a)
	{
		const PxU32 workIndex = a * blockDim.x + threadIdx.x + numIterationPerBlock * blockIdx.x * blockDim.x;

		PxU32 contactCount = 0;
		PxU32 jointCount = 0;
		if (workIndex < nbArticulations)
		{
			contactCount = selfContactCount[workIndex];
			jointCount = selfJointCount[workIndex];
		}

		//we need to use contactCount and jointCount later
		PxU32 sumContact = contactCount;
		PxU32 sumJoint = jointCount;
		PxU32 maxContact = contactCount;
		PxU32 maxJoint = jointCount;

		sumContact = warpReduction<AddOpPxU32, PxU32>(FULL_MASK, sumContact);
		sumJoint = warpReduction<AddOpPxU32, PxU32>(FULL_MASK, sumJoint);
		maxContact = warpReduction<MaxOpPxU32, PxU32>(FULL_MASK, maxContact);
		maxJoint = warpReduction<MaxOpPxU32, PxU32>(FULL_MASK, maxJoint);


		if (threadIndexInWarp == 31)
		{
			shContactUniqueIndicesWarpSum[warpIndex] = sumContact;
			shJointUniqueIndicesWarpSum[warpIndex] = sumJoint;
			shContactHeaderWarpSum[warpIndex] = maxContact;
			shJointHeaderWarpSum[warpIndex] = maxJoint;
		}

		__syncthreads();

		PxU32 contactWarpOffset = 0;
		PxU32 jointWarpOffset = 0;
		PxU32 contactBlockWarpOffset = 0;
		PxU32 jointBlockWarpOffset = 0;

		PxU32 contactUniqueIndicesWarpSum = 0;
		PxU32 jointUniqueIndicesWarpSum = 0;
		PxU32 contactHeaderWarpSum = 0;
		PxU32 jointHeaderWarpSum = 0;

		//warpPerBlock should be less than 32, each warp will do the runsum

		PxU32 mask_idx = __ballot_sync(FULL_MASK, threadIndexInWarp < warpPerBlock);
		if (threadIndexInWarp < warpPerBlock)
		{

			const PxU32 oriContactUniqueIndicesWarpSum = shContactUniqueIndicesWarpSum[threadIndexInWarp];
			const PxU32 oriJointUniqueIndicesWarpSum = shJointUniqueIndicesWarpSum[threadIndexInWarp];
			const PxU32 oriContactHeaderWarpSum = shContactHeaderWarpSum[threadIndexInWarp];
			const PxU32 oriJointHeaderWarpSum = shJointHeaderWarpSum[threadIndexInWarp];

			contactUniqueIndicesWarpSum = warpScan<AddOpPxU32, PxU32>(mask_idx, oriContactUniqueIndicesWarpSum);
			jointUniqueIndicesWarpSum = warpScan<AddOpPxU32, PxU32>(mask_idx, oriJointUniqueIndicesWarpSum);
			contactHeaderWarpSum = warpScan<AddOpPxU32, PxU32>(mask_idx, oriContactHeaderWarpSum);
			jointHeaderWarpSum = warpScan<AddOpPxU32, PxU32>(mask_idx, oriJointHeaderWarpSum);


			//exclusive runsum
			contactWarpOffset = contactUniqueIndicesWarpSum - oriContactUniqueIndicesWarpSum;
			jointWarpOffset = jointUniqueIndicesWarpSum - oriJointUniqueIndicesWarpSum;
			contactBlockWarpOffset = contactHeaderWarpSum - oriContactHeaderWarpSum;
			jointBlockWarpOffset = jointHeaderWarpSum - oriJointHeaderWarpSum;


		}

		//make sure each thread in a warp has the correct warp offset
		contactWarpOffset = __shfl_sync(FULL_MASK, contactWarpOffset, warpIndex);
		jointWarpOffset = __shfl_sync(FULL_MASK, jointWarpOffset, warpIndex);
		contactBlockWarpOffset = __shfl_sync(FULL_MASK, contactBlockWarpOffset, warpIndex);
		jointBlockWarpOffset = __shfl_sync(FULL_MASK, jointBlockWarpOffset, warpIndex);


		//Where the contact uniqueIds should go. This is the start of where this warp should write.
		//The uids will be interleaved depending on the number of constraints in a contact block
		//contactUniqueIndexOffset : articulation static contact start offset
		//sContactUniqueIndicesAccum :: accumulation from the previous iterations
		PxU32 contactOffset = contactUniqueIndexOffset + contactWarpOffset + sContactUniqueIndicesBlockHistogram[blockIdx.x] + sContactUniqueIndicesAccum;
		//Where the joint unique Ids should go. See above for explanation
		PxU32 jointOffset = jointUniqueIndexOffset + jointWarpOffset + sJointUniqueIndicesBlockHistogram[blockIdx.x] + sJointUniqueIndicesAccum;
		//Where the blocks should go. Shared between all threads in a block
		PxU32 contactBlockOffset = contactBlockWarpOffset + sContactHeaderBlockHistogram[blockIdx.x] + sContactHeaderAccum;
		PxU32 jointBlockOffset = jointBlockWarpOffset + sJointHeaderBlockHistogram[blockIdx.x] + sJointHeaderAccum;

		PxU32 blockOffset = contactBlockOffset + jointBlockOffset + articulationBatchOffset;

		PxgConstraintBatchHeader* batchHeaders = prePrepDesc->mBatchHeaders;

		const PxU32 numRigidBatches = constraintPrepDesc->numBatches;
		const PxU32 numArtiContactBatches = constraintPrepDesc->numArtiContactBatches;
		const PxU32 numArtiJointBatches = constraintPrepDesc->numArti1dConstraintBatches;


		//we have to sync in here so all the threads in a warp has finished reading sContactUniqueIndicesAccum, sJointUniqueIndicesAccum,
		//sContactHeaderAccum, sJointHeaderAccum before we overwrite those values for another iterations
		__syncthreads();

		if (threadIdx.x == (warpPerBlock - 1))
		{
			sContactUniqueIndicesAccum += contactUniqueIndicesWarpSum;
			sJointUniqueIndicesAccum += jointUniqueIndicesWarpSum;
			sContactHeaderAccum += contactHeaderWarpSum;
			sJointHeaderAccum += jointHeaderWarpSum;
		}

		__syncthreads();

		PxU32* contactSelfUniqueIds = prePrepDesc->mArtiSelfContactIndices;
		PxU32* jointSelfUniqueIds = prePrepDesc->mArtiSelfConstraintIndices;

		PxU32* outContactUniqueIds = prePrepDesc->mContactUniqueIndices;
		PxU32* outJointUniqueIds = prePrepDesc->mConstraintUniqueIndices;

		PxgArticulationBlockData& data = scDesc->mArticulationBlocks[workIndex / WARP_SIZE];

		if ((workIndex < nbArticulations) && (threadIndexInWarp == 0))
		{
			data.mTotalSelfConstraintCount = maxContact + maxJoint;
			data.mSelfConstraintOffset = blockOffset;
		}


		//Now we loop, outputting the self contacts/joints to the header buffer...
		for (PxU32 i = 0; i < maxContact; ++i)
		{
			PxU32 mask = __ballot_sync(FULL_MASK, contactCount > i);

			const PxU32 stride = __popc(mask);
			const PxU32 offset = warpScanExclusive(mask, threadIndexInWarp);

			if (contactCount > i)
			{
				PxU32 contactUniqueId = contactSelfUniqueIds[workIndex + nbArticulations * i];
				outContactUniqueIds[contactOffset + offset] = contactUniqueId;
			}

			if (threadIndexInWarp == 0)
			{
				const PxU32 batchIndex = contactBlockOffset + i + artiSelfContactBatchOffset;
				PxgConstraintBatchHeader header;
				header.mDescStride = stride;
				header.constraintType = PxgSolverConstraintDesc::eARTICULATION_CONTACT;
				header.mConstraintBatchIndex = batchIndex;
				header.mStartPartitionIndex = contactOffset - numRigidContacts - numRigidStaticContacts;
				header.mask = mask;
				batchHeaders[blockOffset] = header;
				artiContactConstraintBatchIndices[contactBlockOffset + numArtiContactBatches + i + numArtiStaticContacts] = blockOffset - numRigidBatches;
			}

			contactOffset += stride;
			blockOffset++;
		}

		for (PxU32 i = 0; i < maxJoint; ++i)
		{
			PxU32 mask = __ballot_sync(FULL_MASK, jointCount > i);

			const PxU32 stride = __popc(mask);
			const PxU32 offset = warpScanExclusive(mask, threadIndexInWarp);

			if (jointCount > i)
			{
				PxU32 jointUniqueId = jointSelfUniqueIds[workIndex + nbArticulations * i];
				outJointUniqueIds[jointOffset + offset] = jointUniqueId;
			}

			if (threadIndexInWarp == 0)
			{
				PxgConstraintBatchHeader header;
				header.mDescStride = stride;
				const PxU32 batchIndex = jointBlockOffset + i + artiSelfJointBatchOffset;
				header.constraintType = PxgSolverConstraintDesc::eARTICULATION_CONSTRAINT_1D;
				header.mConstraintBatchIndex = batchIndex;
				header.mStartPartitionIndex = jointOffset - numRigidJoints - numRigidStaticJoints;
				header.mask = mask;
				batchHeaders[blockOffset] = header;
				artiJointConstraintBatchIndices[jointBlockOffset + numArtiJointBatches + i + numArtiStaticJoints] = blockOffset - numRigidBatches;
			}

			jointOffset += stride;
			blockOffset++;
		}
	}

}

static __device__ void solveConstraints(PxgArticulationCoreDesc* PX_RESTRICT scDesc, const IterativeSolveData& msIterativeData,
	const PxgBlockConstraintBatch& batch, Cm::UnAlignedSpatialVector& vel0, Cm::UnAlignedSpatialVector& vel1,
	const PxgArticulationBlockLinkData& link0, const PxgArticulationBlockLinkData& link1, Cm::UnAlignedSpatialVector& impulse0,
	Cm::UnAlignedSpatialVector& impulse1, const bool doFriction, const PxReal elapsedTime, const PxReal minPen, const PxU32 offset, PxgErrorAccumulator* errorAccumulator)
{
	// For internal constraints, mass-splitting is not used; thus, reference counts are 1 (default).
	if (batch.constraintType == PxgSolverConstraintDesc::eARTICULATION_CONTACT)
	{
		solveExtContactsBlock(batch, vel0, vel1, doFriction, msIterativeData.blockContactHeaders,
			msIterativeData.blockFrictionHeaders, msIterativeData.blockContactPoints,
			msIterativeData.blockFrictions, msIterativeData.artiResponse, impulse0,
			impulse1, offset, errorAccumulator);
	}
	else
	{
		assert(batch.constraintType == PxgSolverConstraintDesc::eARTICULATION_CONSTRAINT_1D);
		solveExt1DBlock(batch, vel0, vel1, offset, msIterativeData.blockJointConstraintHeaders,
			msIterativeData.blockJointConstraintRowsCon, msIterativeData.blockJointConstraintRowsMod,
			&msIterativeData.artiResponse[batch.mArticulationResponseIndex], impulse0, impulse1, 
			scDesc->mContactErrorAccumulator.mCounter >= 0);
	}
}

static __device__ void solveConstraints(PxgArticulationCoreDesc* PX_RESTRICT scDesc, const IterativeSolveDataTGS& msIterativeData,
	const PxgBlockConstraintBatch& batch, Cm::UnAlignedSpatialVector& vel0, Cm::UnAlignedSpatialVector& vel1,
	const PxgArticulationBlockLinkData& link0, const PxgArticulationBlockLinkData& link1, Cm::UnAlignedSpatialVector& impulse0,
	Cm::UnAlignedSpatialVector& impulse1, const bool doFriction, const PxReal elapsedTime, const PxReal minPen, const PxU32 offset, PxgErrorAccumulator* errorAccumulator)
{
	const Cm::UnAlignedSpatialVector delta0 = loadSpatialVector(link0.mDeltaMotion, threadIdx.x);
	const Cm::UnAlignedSpatialVector delta1 = loadSpatialVector(link1.mDeltaMotion, threadIdx.x);

	// For internal constraints, mass-splitting is not used; thus, reference counts are 1 (default).
	if (batch.constraintType == PxgSolverConstraintDesc::eARTICULATION_CONTACT)
	{
		solveExtContactBlockTGS(batch, vel0, vel1, delta0, delta1, offset,
			msIterativeData.blockContactHeaders, msIterativeData.blockFrictionHeaders, msIterativeData.blockContactPoints,
			msIterativeData.blockFrictions, msIterativeData.artiResponse, elapsedTime, minPen, impulse0, impulse1, errorAccumulator);
	}
	else
	{
		const PxQuat deltaQ0 = loadQuat(link0.mDeltaQ, threadIdx.x);
		const PxQuat deltaQ1 = loadQuat(link1.mDeltaQ, threadIdx.x);
		
		solveExt1DBlockTGS(batch, vel0, vel1, delta0, delta1, offset, msIterativeData.blockJointConstraintHeaders,
			msIterativeData.blockJointConstraintRowsCon, msIterativeData.artiResponse, deltaQ0, deltaQ1, elapsedTime, impulse0, impulse1,
			scDesc->mContactErrorAccumulator.mCounter >= 0);
	}
}


template <typename IterativeData>
static __device__ void artiPropagateRigidImpulsesAndSolveSelfConstraints1T(PxgArticulationCoreDesc* PX_RESTRICT scDesc, 
	const PxgSolverCoreDesc* const PX_RESTRICT solverDesc, const bool velocityIteration,
	const PxReal elapsedTime, const PxgSolverSharedDesc<IterativeData>* const PX_RESTRICT sharedDesc, bool doFriction)
{
	const PxU32 nbSlabs = scDesc->nbSlabs;
	const PxU32 nbArticulations = scDesc->nbArticulations;

	const PxU32 blockStride = blockDim.x;// / WARP_SIZE;
	const PxU32 globalThreadIndex = blockIdx.x * blockStride + threadIdx.x;

	if (globalThreadIndex < nbArticulations)
	{
		const PxReal minPen = velocityIteration ? 0.f : -PX_MAX_F32;
		
		//Identify which block we are solving...
		PxgArticulationBlockData& articulation = scDesc->mArticulationBlocks[blockIdx.x];

		const PxU32 maxLinks = scDesc->mMaxLinksPerArticulation;
		const PxU32 numLinks = articulation.mNumLinks[threadIdx.x];

		PxgArticulationBlockLinkData* artiLinks = scDesc->mArticulationLinkBlocks + maxLinks * blockIdx.x;
		PxgArticulationBlockDofData* artiDofs =
			scDesc->mArticulationDofBlocks + scDesc->mMaxDofsPerArticulation * blockIdx.x;

		const PxU32 articulationReferenceCountOffset =
			solverDesc->islandContextPool->mBodyCount + solverDesc->islandContextPool->mBodyStartIndex;
		const PxU32 numTotalBodies = articulationReferenceCountOffset + nbArticulations;

		// When there are impulses to propagate (must be impulses from rigid body contacts and joints), make sure to
		// propagate them here. This is irrelevant to whether there are self-constraints or not.
		PxU32 dirtyState = articulation.mStateDirty[threadIdx.x];
		if(dirtyState)
		{
			if(dirtyState & PxgArtiStateDirtyFlag::eHAS_IMPULSES)
			{
				// Counting the number of active slabs used in contacts and joints.
				// The split mass used in contacts and joints is tied back here.
				const PxU32 referenceCount = countActiveSlabs(articulationReferenceCountOffset + globalThreadIndex, nbSlabs, numTotalBodies,
				                                              sharedDesc->iterativeData.solverEncodedReferenceCount);

				const PxReal scale = 1.0f / static_cast<PxReal>(referenceCount);

				averageLinkImpulsesAndPropagate(scDesc->slabHasChanges, scDesc->impulses, articulation, artiLinks, artiDofs,
												globalThreadIndex, scDesc->mMaxLinksPerArticulation, nbArticulations, nbSlabs, numLinks,
												threadIdx.x, scale);
			}

			// Leave velocities at the root for now. We will figure out how to do this more lazily next...
			/*PxcFsFlushVelocity(articulation, artiLinks, artiDofs, numLinks,
			    articulation.mFlags[threadIdx.x] & PxArticulationFlag::eFIX_BASE, threadIdx.x);*/
		}

		// Resetting articulation reference counts after all usage.
		resetSlabCount(articulationReferenceCountOffset + globalThreadIndex, nbSlabs, numTotalBodies,
		               sharedDesc->iterativeData.solverEncodedReferenceCount);

		articulation.mStateDirty[threadIdx.x] = PxgArtiStateDirtyFlag::eVEL_DIRTY;

		// Solve self-constraints.
		const PxU32 nbSelfConstraints = articulation.mTotalSelfConstraintCount;
		const PxU32 startIndex = articulation.mSelfConstraintOffset;
		const PxU32 wordSize = (maxLinks + 63) / 64;
		PxgArticulationBitFieldData* linkBitFields = scDesc->mPathToRootBitFieldBlocks + maxLinks * wordSize * blockIdx.x;

		PxgArticulationBitFieldStackData* pathToRootBitFieldA = scDesc->mTempSharedBitFieldBlocks + wordSize * blockIdx.x;
		PxgArticulationBitFieldStackData* pathToRootBitFieldB = scDesc->mTempRootBitFieldBlocks + wordSize * blockIdx.x;
		PxgArticulationBitFieldStackData* commonBitField = scDesc->mTempPathToRootBitFieldBlocks + wordSize * blockIdx.x;

		Cm::UnAlignedSpatialVector rootDeferredZ = loadSpatialVector(articulation.mRootDeferredZ, threadIdx.x);
		Dy::SpatialMatrix rootInvArticulatedInertia;
		loadSpatialMatrix(articulation.mInvSpatialArticulatedInertia, threadIdx.x, rootInvArticulatedInertia);

		if(nbSelfConstraints)
		{
			const IterativeData& iterativeData = sharedDesc->iterativeData;

			PxgErrorAccumulator error;
			const bool accumulateError = scDesc->mContactErrorAccumulator.mCounter >= 0;

			for (PxU32 i = 0; i < nbSelfConstraints; ++i)
			{
				const PxgBlockConstraintBatch& batch = iterativeData.blockConstraintBatch[startIndex + i];

				PxU32 mask = batch.mask;

				if (mask & (1 << threadIdx.x))
				{

					const Cm::UnAlignedSpatialVector commonDelta = rootInvArticulatedInertia * -rootDeferredZ;

					const PxU32 offset = warpScanExclusive(mask, threadIdx.x);

					const PxNodeIndex igNodeIndexA = batch.bodyANodeIndex[offset];
					const PxNodeIndex igNodeIndexB = batch.bodyBNodeIndex[offset];

					//Get velocities for these links...

					assert(igNodeIndexA.index() == igNodeIndexB.index());
					const PxU32 linkIDA = igNodeIndexA.articulationLinkId();
					const PxU32 linkIDB = igNodeIndexB.articulationLinkId();

					PxgArticulationBlockLinkData& linkA = artiLinks[linkIDA];
					PxgArticulationBlockLinkData& linkB = artiLinks[linkIDB];

					for (PxU32 i = 0; i < wordSize; ++i)
					{
						const PxU64 wordA = linkBitFields[linkIDA * wordSize + i].bitField[threadIdx.x];
						const PxU64 wordB = linkBitFields[linkIDB * wordSize + i].bitField[threadIdx.x];
						pathToRootBitFieldA[i].bitField[threadIdx.x] = wordA;
						pathToRootBitFieldB[i].bitField[threadIdx.x] = wordB;
						commonBitField[i].bitField[threadIdx.x] = wordA & wordB;
					}

					Cm::UnAlignedSpatialVector velA = loadSpatialVector(linkA.mMotionVelocity, threadIdx.x);
					Cm::UnAlignedSpatialVector velB = loadSpatialVector(linkB.mMotionVelocity, threadIdx.x);

					Cm::UnAlignedSpatialVector deltaVA = commonDelta;

					for (PxU32 i = 0, wordOffset = 0; i < wordSize; ++i, wordOffset += 64)
					{
						PxU64 word = commonBitField[i].bitField[threadIdx.x];
						while (word != 0)
						{
							const PxU32 index = articulationLowestSetBit(word) + wordOffset;

							if (index != 0) // need to skip root because it has no parent and dofs.
							{
								const PxgArticulationBlockLinkData& link = artiLinks[index];

								deltaVA = propagateAccelerationWNoJVelUpdate(link, &artiDofs[link.mJointOffset[threadIdx.x]], link.mDofs[threadIdx.x],
									deltaVA, threadIdx.x);
							}
							//clear the lowest bit
							word &= (word - 1);
						}
					}


					//Now prop to linkA and linkB...
					Cm::UnAlignedSpatialVector deltaVB = deltaVA;
			
					for (PxU32 i = 0; i < wordSize; ++i)
					{
						pathToRootBitFieldA[i].bitField[threadIdx.x] &= (~commonBitField[i].bitField[threadIdx.x]);
						pathToRootBitFieldB[i].bitField[threadIdx.x] &= (~commonBitField[i].bitField[threadIdx.x]);
					}

					for (PxU32 i = 0, wordOffset = 0; i < wordSize; ++i, wordOffset += 64)
					{
						PxU64 word = pathToRootBitFieldA[i].bitField[threadIdx.x];
						while (word != 0)
						{
							const PxU32 index = articulationLowestSetBit(word) + wordOffset;

							assert(index != 0); //root should be lowered when we remove the common path above.

							const PxgArticulationBlockLinkData& link = artiLinks[index];

							deltaVA = propagateAccelerationWNoJVelUpdate(link, &artiDofs[link.mJointOffset[threadIdx.x]], link.mDofs[threadIdx.x],
								deltaVA, threadIdx.x);

							//clear the lowest bit
							word &= (word - 1);
						}
					}

					for (PxU32 i = 0, wordOffset = 0; i < wordSize; ++i, wordOffset += 64)
					{
						PxU64 word = pathToRootBitFieldB[i].bitField[threadIdx.x];
						while (word != 0)
						{
							const PxU32 index = articulationLowestSetBit(word) + wordOffset;

							assert(index != 0); //root should be lowered when we remove the common path above.

							const PxgArticulationBlockLinkData& link = artiLinks[index];

							deltaVB = propagateAccelerationWNoJVelUpdate(link, &artiDofs[link.mJointOffset[threadIdx.x]], link.mDofs[threadIdx.x],
								deltaVB, threadIdx.x);

							//clear the lowest bit
							word &= (word - 1);
						}
					}
		
					//Now we have updated velocities...
					velA += deltaVA;
					velB += deltaVB;


					//Now do the solve

					Cm::UnAlignedSpatialVector impulse0(PxVec3(0.f), PxVec3(0.f));
					Cm::UnAlignedSpatialVector impulse1(PxVec3(0.f), PxVec3(0.f));

					__syncwarp(mask);

					solveConstraints(scDesc, iterativeData, batch, velA, velB, linkA, linkB, impulse0, impulse1,
						doFriction, elapsedTime, minPen, offset, accumulateError ? &error : NULL);

					//Prop up 

					for (PxI32 i = wordSize-1, wordOffset = (wordSize-1)*64; i >= 0; i--, wordOffset -= 64)
					{
						PxU64 word = pathToRootBitFieldB[i].bitField[threadIdx.x];
						while (word != 0)
						{
							const PxU32 index = articulationHighestSetBit(word);
							//clear the highest bit
							word &= (~(1ull << index));

							assert((index + wordOffset) != 0); //root should be lowered when we remove the common path above.
						
							const PxgArticulationBlockLinkData& link = artiLinks[index + wordOffset];

							PxgArticulationBlockDofData* PX_RESTRICT dofData = &artiDofs[link.mJointOffset[threadIdx.x]];

							const float child2Parent_x = link.mRw_x[threadIdx.x];
							const float child2Parent_y = link.mRw_y[threadIdx.x];
							const float child2Parent_z = link.mRw_z[threadIdx.x];

							const PxU32 dofCount = link.mDofs[threadIdx.x];
							
							impulse1 = propagateImpulseW_0(PxVec3(child2Parent_x, child2Parent_y, child2Parent_z),
								dofData, impulse1, dofCount, threadIdx.x);
						}
					}

					for (PxI32 i = wordSize-1, wordOffset = (wordSize-1)*64; i >= 0; i--, wordOffset -= 64)
					{
						PxU64 word = pathToRootBitFieldA[i].bitField[threadIdx.x];
						while (word != 0)
						{
							const PxU32 index = articulationHighestSetBit(word);
							//clear the highest bit
							word &= (~(1ull << index));

							assert((index + wordOffset) != 0); //root should be lowered when we remove the common path above.

							const PxgArticulationBlockLinkData& link = artiLinks[index + wordOffset];

							PxgArticulationBlockDofData* PX_RESTRICT dofData = &artiDofs[link.mJointOffset[threadIdx.x]];

							const float rwx = link.mRw_x[threadIdx.x];
							const float rwy = link.mRw_y[threadIdx.x];
							const float rwz = link.mRw_z[threadIdx.x];

							const PxU32 dofCount = link.mDofs[threadIdx.x];

							impulse0 = propagateImpulseW_0(PxVec3(rwx, rwy, rwz),
								dofData, impulse0, dofCount, threadIdx.x);
						}
					}

					impulse0 += impulse1;


					for (PxI32 i = wordSize-1, wordOffset = (wordSize-1)*64; i >= 0; i--, wordOffset -= 64)
					{
						PxU64 word = commonBitField[i].bitField[threadIdx.x];
						while (word != 0)
						{
							const PxU32 index = articulationHighestSetBit(word);

							// need to skip root again because it is part of the common path but it has no dofs.
							// we can break because we go from leaf to root.
							if ((index + wordOffset) == 0)
								break; 
	
							//clear the highest bit
							word &= (~(1ull << index));

							const PxgArticulationBlockLinkData& link = artiLinks[index + wordOffset];

							PxgArticulationBlockDofData* PX_RESTRICT dofData = &artiDofs[link.mJointOffset[threadIdx.x]];

							const float rwx = link.mRw_x[threadIdx.x];
							const float rwy = link.mRw_y[threadIdx.x];
							const float rwz = link.mRw_z[threadIdx.x];
							const PxU32 dofCount = link.mDofs[threadIdx.x];

							impulse0 = propagateImpulseW_0(PxVec3(rwx, rwy, rwz),
								dofData, impulse0, dofCount, threadIdx.x);
						}
					}

					rootDeferredZ += impulse0;
				}
			}

			storeSpatialVector(articulation.mRootDeferredZ, rootDeferredZ, threadIdx.x);
		}
	}
}


//two warp each block
extern "C" __global__
void artiPropagateRigidImpulsesAndSolveSelfConstraints1T(PxgArticulationCoreDesc* PX_RESTRICT scDesc, const PxgSolverCoreDesc* const PX_RESTRICT solverDesc, 
	const bool velocityIteration, const PxReal elapsedTime,	const PxgSolverSharedDesc<IterativeSolveData>* const PX_RESTRICT sharedDesc, bool doFriction)
{
	artiPropagateRigidImpulsesAndSolveSelfConstraints1T<IterativeSolveData>(scDesc, solverDesc, velocityIteration, 0.0f, sharedDesc, doFriction);
}


extern "C" __global__
void artiPropagateRigidImpulsesAndSolveSelfConstraintsTGS1T(PxgArticulationCoreDesc* PX_RESTRICT scDesc, const PxgSolverCoreDesc* const PX_RESTRICT solverDesc, 
	const bool velocityIteration, const PxReal elapsedTime, const PxgSolverSharedDesc<IterativeSolveDataTGS>* const PX_RESTRICT sharedDesc, bool doFriction)
{
	artiPropagateRigidImpulsesAndSolveSelfConstraints1T<IterativeSolveDataTGS>(scDesc, solverDesc, velocityIteration, elapsedTime, sharedDesc, doFriction);
}

