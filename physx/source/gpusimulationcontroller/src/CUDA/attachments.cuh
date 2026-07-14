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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef __ATTACHMENTS_CUH__
#define __ATTACHMENTS_CUH__

#include "foundation/PxVecMath.h"

namespace physx
{

PX_FORCE_INLINE __device__ PxVec3 calculateAttachmentDeltaImpulsePGS(const float4& raXn0_biasW, const float4& raXn1_biasW,
																	 const float4& raXn2_biasW, const float4& velMultiplierXYZ_invMassW,
																	 const PxgVelocityPackPGS& vel0, const PxVec3& linVel1,
																	 PxReal attachPointInvMass, PxReal attachPointInvMassSplit,
																	 PxU32 rigidBodyReferenceCount,
																	 PxReal invDt, PxReal biasFactor,
																	 PxVec3& deltaLinVel, PxVec3& deltaAngVel)
{
	const PxVec3 raXn0 = PxVec3(raXn0_biasW.x, raXn0_biasW.y, raXn0_biasW.z);
	const PxVec3 raXn1 = PxVec3(raXn1_biasW.x, raXn1_biasW.y, raXn1_biasW.z);
	const PxVec3 raXn2 = PxVec3(raXn2_biasW.x, raXn2_biasW.y, raXn2_biasW.z);

	// Compute the normal velocity of the constraint.

	const PxReal velOfRigidAtAttachmentPointX = vel0.linVel.x + vel0.angVel.dot(raXn0);
	const PxReal velOfRigidAtAttachmentPointY = vel0.linVel.y + vel0.angVel.dot(raXn1);
	const PxReal velOfRigidAtAttachmentPointZ = vel0.linVel.z + vel0.angVel.dot(raXn2);

	// Definition of errors is as follows
	// raXn0_biasW.w = ((rigidBodyCoM + comToPoint) - attachedPointLocation).x;
	// raXn1_biasW.w = ((rigidBodyCoM + comToPoint) - attachedPointLocation).y;
	// raXn2_biasW.w = ((rigidBodyCoM + comToPoint) - attachedPointLocation).z;
	const PxReal& positionErrorX = raXn0_biasW.w;
	const PxReal& positionErrorY = raXn1_biasW.w;
	const PxReal& positionErrorZ = raXn2_biasW.w;

	// Mass-splitting adjustment of the constraint denominator.
	//
	// Prep baked velMult = 1/(R + D) where R is the rigid-side response
	// (raXn^2 * invI + invMass0) and D = attachPointInvMass is the deformable-
	// side sum-of-squared-bary invMass at prep time (D = sum b_i^2 * invM_i).
	//
	// At solve time we want the per-vertex-refCount-split deformable
	// denominator D' = attachPointInvMassSplit (= sum b_i^2 * invM_i *
	// refCount_i, refCount read from mSimDelta[v].w / cloth.mDeltaPos[v].w
	// at solve, matching FEMCollision). refCount is dynamic across iters
	// so prep can't bake it in.
	//
	// For Jacobi-parallel stability across N constraints sharing this rigid,
	// we want velMult' = 1/(N*R + D'). Algebra:
	//   N*R + D' = N*(1/velMult - D) + D'
	//            = N/velMult - N*D + D'.
	// Reduces to the legacy formula when D == D'.
	const PxReal refN = static_cast<PxReal>(rigidBodyReferenceCount);
	const PxReal denomBias = refN * attachPointInvMass - attachPointInvMassSplit;
	const PxReal adjVelMult0 = 1.0f / (refN / velMultiplierXYZ_invMassW.x - denomBias);
	const PxReal adjVelMult1 = 1.0f / (refN / velMultiplierXYZ_invMassW.y - denomBias);
	const PxReal adjVelMult2 = 1.0f / (refN / velMultiplierXYZ_invMassW.z - denomBias);

	// For bias see here https://box2d.org/files/ErinCatto_SequentialImpulses_GDC2006.pdf
	// Slide 22, Bias Impulse
	PxVec3 velError(linVel1.x - positionErrorX * biasFactor * invDt, linVel1.y - positionErrorY * biasFactor * invDt,
					linVel1.z - positionErrorZ * biasFactor * invDt);

	// deltaF for PGS: impulse
	const PxReal deltaF0 = (velError.x - velOfRigidAtAttachmentPointX) * adjVelMult0;
	const PxReal deltaF1 = (velError.y - velOfRigidAtAttachmentPointY) * adjVelMult1;
	const PxReal deltaF2 = (velError.z - velOfRigidAtAttachmentPointZ) * adjVelMult2;

	const PxVec3 deltaImpulse = PxVec3(deltaF0, deltaF1, deltaF2);

	const PxReal invMass0 = velMultiplierXYZ_invMassW.w;

	deltaLinVel = deltaImpulse * invMass0;
	deltaAngVel = raXn0 * deltaF0 + raXn1 * deltaF1 + raXn2 * deltaF2;

	return deltaImpulse;
}

template <typename ConstraintType>
PX_FORCE_INLINE __device__ PxVec3 calculateAttachmentDeltaImpulsePGS(PxU32 offset, const ConstraintType& constraint,
																	 const PxgVelocityPackPGS& vel0, const PxVec3& linVel1,
																	 PxReal attachPointInvMass, PxReal attachPointInvMassSplit,
																	 PxReal invDt, PxReal biasFactor,
																	 PxVec3& deltaLinVel, PxVec3& deltaAngVel)
{
	return calculateAttachmentDeltaImpulsePGS(constraint.raXn0_biasW[offset], constraint.raXn1_biasW[offset], constraint.raXn2_biasW[offset],
											  constraint.velMultiplierXYZ_invMassW[offset], vel0, linVel1,
											  attachPointInvMass, attachPointInvMassSplit,
											  constraint.rigidBodyReferenceCount[offset],
											  invDt, biasFactor, deltaLinVel, deltaAngVel);
}

PX_FORCE_INLINE __device__ PxVec3 calculateAttachmentDeltaImpulseTGS(const float4& raXn0_biasW, const float4& raXn1_biasW,
																	 const float4& raXn2_biasW, const float4& velMultiplierXYZ_invMassW,
																	 const PxgVelocityPackTGS& vel0, const PxVec3& linDelta1,
																	 PxReal attachPointInvMass, PxReal attachPointInvMassSplit,
																	 PxU32 rigidBodyReferenceCount,
																	 PxReal dt, PxReal biasCoefficient, bool isVelocityIteration,
																	 PxVec3& deltaLinVel, PxVec3& deltaAngVel)
{
	const PxVec3 raXn0 = PxVec3(raXn0_biasW.x, raXn0_biasW.y, raXn0_biasW.z);
	const PxVec3 raXn1 = PxVec3(raXn1_biasW.x, raXn1_biasW.y, raXn1_biasW.z);
	const PxVec3 raXn2 = PxVec3(raXn2_biasW.x, raXn2_biasW.y, raXn2_biasW.z);

	const PxReal velOfRigidAtAttachmentPoint0 = vel0.linVel.x + vel0.angVel.dot(raXn0);
	const PxReal velOfRigidAtAttachmentPoint1 = vel0.linVel.y + vel0.angVel.dot(raXn1);
	const PxReal velOfRigidAtAttachmentPoint2 = vel0.linVel.z + vel0.angVel.dot(raXn2);

	const PxVec3 linDelta = linDelta1 - vel0.linDelta;

	// Definition of errors is as follows
	// raXn0_biasW.w = ((rigidBodyCoM + comToPoint) - attachedPointLocation).x;
	// raXn1_biasW.w = ((rigidBodyCoM + comToPoint) - attachedPointLocation).y;
	// raXn2_biasW.w = ((rigidBodyCoM + comToPoint) - attachedPointLocation).z;
	const PxReal& positionErrorX = raXn0_biasW.w;
	const PxReal& positionErrorY = raXn1_biasW.w;
	const PxReal& positionErrorZ = raXn2_biasW.w;

	// Velocity-space TGS code path matching FEMCollision: lift position-space terms (linDelta,
	// angDelta.raXn, baumgarte biasedErr*biasCoefficient) to velocity via *invDt; velAtPt is
	// already velocity, no scale. Output deltaF is in impulse units, matching PGS.
	// biasCoefficient is the bare per-iteration coefficient (no *invStepDt premultiply at the
	// callsite); it scales only the Baumgarte position-error term, not linDelta or angDelta.
	const PxReal invDt = 1.0f / dt;
	const PxVec3 tgsErrVel(
		(linDelta.x - vel0.angDelta.dot(raXn0) - positionErrorX * biasCoefficient) * invDt,
		(linDelta.y - vel0.angDelta.dot(raXn1) - positionErrorY * biasCoefficient) * invDt,
		(linDelta.z - vel0.angDelta.dot(raXn2) - positionErrorZ * biasCoefficient) * invDt);

	// Velocity iteration zeros the rigid-body velocity contribution (vel-iter constraint
	// enforcement ignores rigid-body motion at the substep).
	const PxReal velIterScale = isVelocityIteration ? 0.0f : 1.0f;

	// Mass-splitting adjustment of the constraint denominator (as in PGS path; see there).
	const PxReal refN = static_cast<PxReal>(rigidBodyReferenceCount);
	const PxReal denomBias = refN * attachPointInvMass - attachPointInvMassSplit;
	const PxReal adjVelMult0 = 1.0f / (refN / velMultiplierXYZ_invMassW.x - denomBias);
	const PxReal adjVelMult1 = 1.0f / (refN / velMultiplierXYZ_invMassW.y - denomBias);
	const PxReal adjVelMult2 = 1.0f / (refN / velMultiplierXYZ_invMassW.z - denomBias);

	const PxReal deltaF0 = (tgsErrVel.x - velOfRigidAtAttachmentPoint0 * velIterScale) * adjVelMult0;
	const PxReal deltaF1 = (tgsErrVel.y - velOfRigidAtAttachmentPoint1 * velIterScale) * adjVelMult1;
	const PxReal deltaF2 = (tgsErrVel.z - velOfRigidAtAttachmentPoint2 * velIterScale) * adjVelMult2;

	const PxVec3 deltaImpulse = PxVec3(deltaF0, deltaF1, deltaF2);

	deltaLinVel = deltaImpulse * velMultiplierXYZ_invMassW.w;
	deltaAngVel = raXn0 * deltaF0 + raXn1 * deltaF1 + raXn2 * deltaF2;

	return deltaImpulse;
}

template <typename ConstraintType>
PX_FORCE_INLINE __device__ PxVec3 calculateAttachmentDeltaImpulseTGS(PxU32 offset, const ConstraintType& constraint,
																	 const PxgVelocityPackTGS& vel0, const PxVec3& linDelta1,
																	 PxReal attachPointInvMass, PxReal attachPointInvMassSplit,
																	 PxReal dt, PxReal biasCoefficient, bool isVelocityIteration,
																	 PxVec3& deltaLinVel, PxVec3& deltaAngVel)
{
	return calculateAttachmentDeltaImpulseTGS(constraint.raXn0_biasW[offset], constraint.raXn1_biasW[offset],
											  constraint.raXn2_biasW[offset], constraint.velMultiplierXYZ_invMassW[offset], vel0, linDelta1,
											  attachPointInvMass, attachPointInvMassSplit,
											  constraint.rigidBodyReferenceCount[offset],
											  dt, biasCoefficient, isVelocityIteration, deltaLinVel, deltaAngVel);
}

} // namespace physx

#endif // __ATTACHMENTS_CUH__
