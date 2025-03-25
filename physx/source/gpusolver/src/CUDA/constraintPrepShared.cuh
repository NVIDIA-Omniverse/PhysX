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

#ifndef	__CONSTRAINT_PREP_SHARED_CUH__
#define	__CONSTRAINT_PREP_SHARED_CUH__

#include "PxgBodySim.h"
#include "PxgSolverBody.h"
#include "PxgConstraint.h"
#include "PxgConstraintPrep.h"
#include "PxgSolverConstraintDesc.h"
#include "PxgSolverCoreDesc.h"
#include "DySolverConstraintTypes.h"
#include "PxNodeIndex.h"
#include "PxgArticulation.h"
#include "PxgEdgeType.h"
#include "PxgDynamicsConfiguration.h"
#include "stdio.h"
#include "utils.cuh"
#include "PxgSolverFlags.h"
#include "DyCpuGpu1dConstraint.h"
#include "PxgPartitionNode.h"

#define PXC_SAME_NORMAL 0.999f

static __device__ physx::PxU32 computeRemapIndexRigidBody(bool isSecondBody, 
	const physx::PxU32* const PX_RESTRICT partitionStartIndices, 
	const physx::PxU32* const PX_RESTRICT partitionArticStartIndices, 
	const physx::PxU32* const PX_RESTRICT partitionJointCounts,
	const physx::PxU32* const PX_RESTRICT partitionArticulationJointCounts, 
	const physx::PartitionIndexData& indexData, 
	physx::PxgSolverReferences* solverBodyReferences, 
	physx::PxU32 currPartition, 
	physx::PxU32 maxNbPartitions, 
	const physx::PxU32 totalActiveBodyCount,
	const physx::PxU32 bodyId,
	const physx::PxU32 activeBodyOffset, 
	const physx::PxU32 totalRigidBatches, 
	const physx::PxU32 totalArticBatches,
	const physx::PxU32 nbElemsPerBody,
	const physx::PxU32 nbSlabs,
	const physx::PxU32 solverBodyOutputVelocityOffset //Only used for assert
)
{
	using namespace physx;
	//Computes the remapped index for the rigid body being referenced by this constraint.
	//This is quite complicated. For rigid body constraints, there are up to 32 pairs of rigid bodies referenced
	//by each constraint batch.
	//Joints attached to articulations and articulation contacts are indexed similar to rigid bodies. The rigid 
	//bodies for articulation constraints appear after the rigid body constraints in the list, prior to the accumulation buffer.

	const PxU32 partitionIndex = indexData.mPartitionIndex;
	//printf("indexData partitionIndex %i\n", partitionIndex);
	const PxU32 partitionEntryIndex = indexData.mPartitionEntryIndex;
	//printf("indexData partitionEntryIndex %i\n", partitionEntryIndex);
	PxU32 index;

	//printf("indexData type %i\n", indexData.mCType);
	switch (indexData.mCType)
	{
	case PxgEdgeType::eCONTACT_MANAGER:
		index = partitionStartIndices[partitionIndex] + partitionJointCounts[partitionIndex] + partitionEntryIndex / PXG_BATCH_SIZE;
		break;
	case PxgEdgeType::eCONSTRAINT:
		index = partitionStartIndices[partitionIndex] + partitionEntryIndex / PXG_BATCH_SIZE;
		break;
	case PxgEdgeType::eARTICULATION_CONTACT:
		index = totalRigidBatches + partitionArticStartIndices[partitionIndex] + partitionArticulationJointCounts[partitionIndex] + partitionEntryIndex / PXG_BATCH_SIZE;
		break;
	case PxgEdgeType::eARTICULATION_CONSTRAINT:
		index = totalRigidBatches + partitionArticStartIndices[partitionIndex] + partitionEntryIndex / PXG_BATCH_SIZE;
		break;
	default:
		break;
	}

	/*printf("Index is now = %i\n", index);

	printf("partitionJointCounts[%i] = %i\n", partitionIndex, partitionJointCounts[partitionIndex]);
	printf("partitionArticStartIndices[%i] = %i\n", partitionIndex, partitionArticStartIndices[partitionIndex]);
	printf("partitionArticulationJointCounts[%i] = %i\n", partitionIndex, partitionArticulationJointCounts[partitionIndex]);
	printf("partitionEntryIndex %i\n", partitionEntryIndex);*/

	//if (!isArticulation)
	{
		//printf("Next is not an articulation constraint!!!\n");
		const PxU32 batchMask = PXG_BATCH_SIZE - 1;

		index = (index*PXG_BATCH_SIZE * 2)*nbElemsPerBody + (partitionEntryIndex & batchMask);
		if (isSecondBody)
			index += PXG_BATCH_SIZE*nbElemsPerBody;
	}
	//else
	//{
	//	//printf("Next is an articulation constraint index = %i!!!\n", index);
	//	//Make sure that there's the slot for index, index+32
	//	index = ((index & (~31))*nbElemsPerBody) + (index&31);
	//	index += totalRigidBatches * PXG_BATCH_SIZE * 2* nbElemsPerBody;
	//}

	//printf("index %i Type = %i\n", index, indexData.mCType);

	if (bodyId >= activeBodyOffset)
	{
		//printf("bodyId>= activeBodyOffset\n");
		if ((partitionIndex & (~(maxNbPartitions - 1))) != (currPartition & (~(maxNbPartitions - 1))) || partitionIndex <= currPartition)
		{
			//printf("recalculate slabId\n");
			//We changed slabs, so we need to introduce a new solver reference
			PxU32 slabId = partitionIndex / maxNbPartitions;

			//printf("slabId %i bodyId %i activeBodyOffset%i \n", slabId, bodyId, activeBodyOffset);
			//PxU32 referenceId = slabId * totalActiveBodyCount + bodyId - activeBodyOffset;
			//PxU32 referenceId = slabId * totalActiveBodyCount + bodyId - activeBodyOffset;
			PxU32 referenceId = nbSlabs * (bodyId - activeBodyOffset) + slabId;

			/*printf("%i: slabId = %i, nbSlabs = %i, bodyId = %i, activeBodyOffset = %i, referenceId = %i\n", 
				threadIdx.x, slabId, nbSlabs, bodyId, activeBodyOffset, referenceId);*/

			solverBodyReferences[referenceId].mRemappedBodyIndex = index;

			referenceId = (referenceId&(~31))*nbElemsPerBody + (referenceId & 31);

			//printf("solverBodyReferences[%i].mRemappedBodyIndex %i\n", referenceId, solverBodyReferences[referenceId].mRemappedBodyIndex);
			//Change remap table so that outputs that need to be averaged are placed in a format that is suitable for coalesced loading!
			//There are totalBatches * 32 * 2 * 2 float4 velocity vectors for the solver, then nbSlabs * bodyCount * 2 float4 velocity vectors for averaging!
			index = (totalArticBatches + totalRigidBatches) * PXG_BATCH_SIZE * 2 * nbElemsPerBody + referenceId;

			//printf("TotalRigidBatches = %i, totalArticBatches = %i\n", totalRigidBatches, totalArticBatches);
		}
	}

	assert(index < solverBodyOutputVelocityOffset);

	return index;
}


static __device__ PX_FORCE_INLINE bool pointsAreClose(const physx::PxAlignedTransform& body1ToBody0,
	const float4& localAnchor0, const float4& localAnchor1,
	const float4& axis, float correlDist)
{
	using namespace physx;

	const float4 body0PatchPoint1 = body1ToBody0.transform(localAnchor1);

	return PxAbs(dot3(localAnchor0 - body0PatchPoint1, axis))<correlDist;
}

// Decides whether or not the damper should be turned on. We don't want damping if the contact
// is not expected to be closed this step because the damper can produce repulsive forces
// even before the contact is closed.
static __device__ PX_FORCE_INLINE PxReal computeCompliantDamping(bool isSeparated, bool collidingWithVrel, PxReal damping)
{
	const float dampingIfEnabled = (isSeparated && !collidingWithVrel) ? 0.0f : damping;
	return dampingIfEnabled;
}

// Storing minimal coefficients to compute constant, unbiasedConstant, velMultiplier, and impulseMultiplier in
// "compute1dConstraintSolverConstantsPGS"
static __device__ PX_FORCE_INLINE void queryReduced1dConstraintSolverConstantsPGS(
	const PxU16 constraintFlags, const PxReal springStiffness, const PxReal springDamping, const PxReal restitution,
	const PxReal bounceThreshold, const PxReal geometricError, const PxReal velocityTarget,
	const PxReal jointSpeedForRestitutionBounce, const PxReal erp, const PxReal simDt, const PxReal recipSimDt,
	PxReal& coeff0, PxReal& coeff1)
{
	if (constraintFlags & Px1DConstraintFlag::eSPRING)
	{
		// coeff0: a, coeff1: b in "compute1dConstraintSolverConstantsPGS"

		const PxReal a = simDt * simDt * springStiffness + simDt * springDamping;
		const PxReal b = simDt * (springDamping * velocityTarget - springStiffness * geometricError);

		coeff0 = a;
		coeff1 = b;
	}
	else
	{
		// coeff0: constant (to be scaled by recipUnitResponse)
		// coeff1: unbiasedConstant (to be scaled by recipUnitResponse)

		const PxReal bounceVel = Dy::computeBounceVelocity(constraintFlags, jointSpeedForRestitutionBounce, bounceThreshold, restitution, geometricError);
		if (bounceVel != 0.0f)
		{
			coeff0 = bounceVel;
			coeff1 = bounceVel;
		}
		else
		{
			const PxReal geomError = geometricError * erp;
			coeff0 = velocityTarget - geomError * recipSimDt;
			coeff1 = (!(constraintFlags & Px1DConstraintFlag::eKEEPBIAS)) ? velocityTarget : coeff0;
		}
	}
}

// Computing constant, unbiasedConstant, velMultiplier, and impulseMultiplier using precomputed coefficients. 
// See also "queryReduced1dConstraintSolverConstantsPGS" and "compute1dConstraintSolverConstantsPGS."
static __device__ PX_FORCE_INLINE void compute1dConstraintSolverConstantsPGS
(bool isSpring, bool isAccelerationSpring, PxReal coeff0, PxReal coeff1, PxReal coeff2,
	const PxReal unitResponse, const PxReal recipUnitResponse,
	PxReal& constant, PxReal& unbiasedConstant, PxReal& velMultiplier, PxReal& impulseMultiplier)
{
	if (isSpring)
	{
		// coeff0: a
		// coeff1: b

		const PxReal a = coeff0;
		const PxReal b = coeff1;

		if (isAccelerationSpring)
		{
			const PxReal x = 1.0f / (1.0f + a);
			constant = x * recipUnitResponse * b;
			unbiasedConstant = constant;
			velMultiplier = -x * recipUnitResponse * a;
			impulseMultiplier = 1.0f - x;
		}
		else
		{
			const PxReal x = 1.0f / (1.0f + a * unitResponse);
			constant = x * b;
			unbiasedConstant = constant;
			velMultiplier = -x * a;
			impulseMultiplier = 1.0f - x;
		}
	}
	else
	{
		// coeff0: constant (to be scaled by recipUnitResponse)
		// coeff1: unbiasedConstant (to be scaled by recipUnitResponse)

		velMultiplier = -recipUnitResponse;
		impulseMultiplier = 1.0f;

		constant = coeff0 * recipUnitResponse;
		unbiasedConstant = coeff1 * recipUnitResponse;
	}

	// coeff2: initJointSpeed
	const PxReal velBias = coeff2 * velMultiplier;
	constant += velBias;
	unbiasedConstant += velBias;
}


static __device__ PX_FORCE_INLINE PxReal
computeCompliantContactCoefficients(PxReal dt, PxU8 flags, PxReal restitution, PxReal damping, PxReal unitResponse,
                                    PxReal recipResponse, PxReal penetration, PxReal targetVelocity, bool isSeparated,
                                    bool collidingWithVrel, PxReal& velMultiplier, PxReal& impulseMultiplier,
                                    PxReal& unbiasedErr, PxReal& biasedErr)
{
	const bool accelSpring = !!(flags & PxgSolverContactFlags::eCOMPLIANT_ACCELERATION_SPRING);
	const PxReal massIfAccelElseOne = accelSpring ? recipResponse : 1.0f;
	const PxReal oneIfAccelElseR = accelSpring ? 1.0f : unitResponse;

	const PxReal nrdt = dt * restitution;
	const PxReal dampingIfEnabled = computeCompliantDamping(isSeparated, collidingWithVrel, damping);
	const PxReal a = dt * (dampingIfEnabled - nrdt);
	const PxReal b = -(nrdt * penetration * massIfAccelElseOne);
	const PxReal x = 1.f / (a * oneIfAccelElseR + 1.f);
	// scaledBias = FSel(isSeparated, FNeg(invStepDt), FDiv(FMul(nrdt, FMul(x, unitResponse)), velMultiplier));
	const PxReal scaledBias = x * b;

	velMultiplier = x * a * massIfAccelElseOne;
	impulseMultiplier = 1.f - x;
	unbiasedErr = biasedErr = targetVelocity * velMultiplier - scaledBias;

	return a;
}

// Query two coefficients, coeff0 and coeff1, to compute contact coefficients efficiently at every sub-timestep or
// iteration. See "computeCompliantContactCoefficients".
static __device__ PX_FORCE_INLINE void
queryReducedCompliantContactCoefficients(PxReal dt, PxU8 flags, PxReal restitution, PxReal damping, PxReal penetration, 
										 PxReal targetVelocity, bool isSeparated, bool collidingWithVrel, 
										 PxReal& coeff0, PxReal& coeff1)
{
	const PxReal nrdt = dt * restitution;
	const PxReal dampingIfEnabled = computeCompliantDamping(isSeparated, collidingWithVrel, damping);
	coeff0 = dt * (dampingIfEnabled - nrdt); // a = coeff0
	coeff1 = -nrdt * penetration; // b = -(nrdt * penetration * massIfAccelElseOne) = coeff1 * massIfAccelElseOne
}


// Compute contact-related coefficients, velMultiplier, impulseMultiplier, unbiasedErr, and biasedErr with precomputed coefficients, coeff0 and coeff1.
// See "computeCompliantContactCoefficients".
static __device__ PX_FORCE_INLINE void 
computeContactCoefficients(PxU8 flags, PxReal restitution, PxReal unitResponse, PxReal recipResponse, PxReal targetVelocity, 
						   PxReal coeff0, PxReal coeff1, PxReal& velMultiplier, PxReal& impulseMultiplier, 
						   PxReal& unbiasedErr, PxReal& biasedErr)
{
	if (restitution < 0.f)
	{
		const bool accelSpring = !!(flags & PxgSolverContactFlags::eCOMPLIANT_ACCELERATION_SPRING);
		const PxReal massIfAccelElseOne = accelSpring ? recipResponse : 1.0f;
		const PxReal oneIfAccelElseR = accelSpring ? 1.0f : unitResponse;

		const PxReal a = coeff0;
		const PxReal b = coeff1 * massIfAccelElseOne;

		const PxReal x = 1.f / (a * oneIfAccelElseR + 1.f);
		const PxReal scaledBias = x * b;

		velMultiplier = x * a * massIfAccelElseOne;
		impulseMultiplier = 1.f - x;
		unbiasedErr = biasedErr = targetVelocity * velMultiplier - scaledBias;
	}
	else
	{
		velMultiplier = recipResponse;
		biasedErr = coeff0 * velMultiplier;
		unbiasedErr = coeff1 * velMultiplier;
		impulseMultiplier = 1.f;
	}
}

static __device__ PX_FORCE_INLINE PxReal 
computeCompliantContactCoefficientsTGS(PxReal stepDt, PxU8 flags,PxReal restitution, PxReal damping,
                                       PxReal unitResponse, PxReal recipResponse, bool isSeparated, bool collidingWithVrel,
                                       PxReal& velMultiplier, PxReal& scaledBias)
{
	const bool accelSpring = !!(flags & PxgSolverContactFlags::eCOMPLIANT_ACCELERATION_SPRING);
	const PxReal massIfAccelElseOne = accelSpring ? recipResponse : 1.0f;
	const PxReal oneIfAccelElseR = accelSpring ? 1.0f : unitResponse;

	const PxReal dampingIfEnabled = computeCompliantDamping(isSeparated, collidingWithVrel, damping);
	const PxReal nrdt = stepDt * restitution;
	const PxReal a = stepDt * (dampingIfEnabled - nrdt);
	const PxReal x = 1.f / (a * oneIfAccelElseR + 1.f);

	velMultiplier = x * a * massIfAccelElseOne;
	scaledBias = nrdt * x * oneIfAccelElseR;

	return a; // compliant contact coefficient a.
}

static __device__ PX_FORCE_INLINE void 
computeCompliantContactCoefficientsTGS(PxU8 flags, PxReal nrdt, PxReal unitResponse, PxReal recipResponse, PxReal a, 
									  PxReal& velMultiplier, PxReal& scaledBias)
{
	const bool accelSpring = !!(flags & PxgSolverContactFlags::eCOMPLIANT_ACCELERATION_SPRING);
	const PxReal massIfAccelElseOne = accelSpring ? recipResponse : 1.0f;
	const PxReal oneIfAccelElseR = accelSpring ? 1.0f : unitResponse;
	//const PxReal nrdt = stepDt * restitution;
	const PxReal x = 1.f / (a * oneIfAccelElseR + 1.f);

	velMultiplier = x * a * massIfAccelElseOne;
	scaledBias = nrdt * x * oneIfAccelElseR;
}

#endif