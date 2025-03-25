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

#ifndef __ATTACHMENTS_CUH__
#define __ATTACHMENTS_CUH__

#include "foundation/PxVecMath.h"

using namespace physx;

__device__ inline bool isConeLimitedEnabled(float maxAngle, float minDist, float maxDist)
{
	return maxAngle >= 0.f || minDist >= 0.f || maxDist >= 0.f;
}

/**
 * \param maxAngle The cone opening angle measured from the axis.
 * \param minDist Minimum distance measure from the cone tip.
 * \param maxDist Maximum distance measured from the cone tip.
 * \param relPos Position relative to the cone tip for which the error is computed (the minimum translation
 *				 vector to get from relPos to the allowed cone volume)
 */
__device__ inline PxVec3 computeConeLimitedError(float maxAngle, float minDist, float maxDist, const PxVec3& coneAxis, const PxVec3& relPos)
{
	PxReal len = relPos.magnitude();

	// angle constraint
	PxVec3 dir;
	if(maxAngle == 0.f)
	{
		dir = coneAxis;
	}
	else if(maxAngle > 0.f)
	{
		dir = (len > 1.0e-6f) ? (relPos / len) : coneAxis;
		const PxReal cosAngle = dir.dot(coneAxis);
		PxReal cosMaxAngle;
		PxReal sinMaxAngle;
		PxSinCos(maxAngle, sinMaxAngle, cosMaxAngle); // could be precomputed
		if(cosAngle < cosMaxAngle)					  // if theta > maxAngle
		{
			PxVec3 t1 = dir.cross(coneAxis);
			PxVec3 b1 = coneAxis.cross(t1).getNormalized();
			dir = cosMaxAngle * coneAxis + sinMaxAngle * b1; // new direction that is "maxAngle" deviated from the world axis.
		}
	}
	else
	{
		dir = (len > 1.0e-6f) ? (relPos / len) : coneAxis;
	}

	// length constraint
	len = PxClamp(len, minDist, maxDist >= 0.f ? maxDist : FLT_MAX);

	return relPos - len * dir; // ideal relPos = len * dir
}

PX_FORCE_INLINE __device__ PxVec3 calculateAttachmentDeltaImpulsePGS(const float4& raXn0_biasW, const float4& raXn1_biasW,
																	 const float4& raXn2_biasW, const float4& velMultiplierXYZ_invMassW,
																	 const float4& low_high_limits, const float4& worldAxis_angle,
																	 const PxgVelocityPackPGS& vel0, const PxVec3& linVel1, PxReal invDt,
																	 PxReal biasFactor, PxVec3& deltaLinVel, PxVec3& deltaAngVel)
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

	// For bias see here https://box2d.org/files/ErinCatto_SequentialImpulses_GDC2006.pdf
	// Slide 22, Bias Impulse
	PxVec3 velError(linVel1.x - positionErrorX * biasFactor * invDt, linVel1.y - positionErrorY * biasFactor * invDt,
					linVel1.z - positionErrorZ * biasFactor * invDt);

	if(isConeLimitedEnabled(worldAxis_angle.w, low_high_limits.x, low_high_limits.y))
	{
		// we don't understand why we need to scale by biasFactor in order to get the right error offset for the cone.
		PxReal weirdScale = invDt * biasFactor;
		PxVec3 posError = velError * (1.0f / weirdScale);
		const PxVec3 worldAxis(worldAxis_angle.x, worldAxis_angle.y, worldAxis_angle.z);
		posError = computeConeLimitedError(worldAxis_angle.w, low_high_limits.x, low_high_limits.y, worldAxis, posError);
		velError = posError * weirdScale;
	}

	// deltaF for PGS: impulse
	const PxReal deltaF0 = (velError.x - velOfRigidAtAttachmentPointX) * velMultiplierXYZ_invMassW.x;
	const PxReal deltaF1 = (velError.y - velOfRigidAtAttachmentPointY) * velMultiplierXYZ_invMassW.y;
	const PxReal deltaF2 = (velError.z - velOfRigidAtAttachmentPointZ) * velMultiplierXYZ_invMassW.z;

	const PxVec3 deltaImpulse = PxVec3(deltaF0, deltaF1, deltaF2);

	const PxReal invMass0 = velMultiplierXYZ_invMassW.w;

	deltaLinVel = deltaImpulse * invMass0;
	deltaAngVel = raXn0 * deltaF0 + raXn1 * deltaF1 + raXn2 * deltaF2;

	return deltaImpulse;
}

template <typename ConstraintType>
PX_FORCE_INLINE __device__ PxVec3 calculateAttachmentDeltaImpulsePGS(PxU32 offset, const ConstraintType& constraint,
																	 const PxgVelocityPackPGS& vel0, const PxVec3& linVel1, PxReal invDt,
																	 PxReal biasFactor, PxVec3& deltaLinVel, PxVec3& deltaAngVel)
{
	return calculateAttachmentDeltaImpulsePGS(constraint.raXn0_biasW[offset], constraint.raXn1_biasW[offset], constraint.raXn2_biasW[offset],
											  constraint.velMultiplierXYZ_invMassW[offset], constraint.low_high_limits[offset],
											  constraint.axis_angle[offset], vel0, linVel1, invDt, biasFactor, deltaLinVel, deltaAngVel);
}

PX_FORCE_INLINE __device__ PxVec3 calculateAttachmentDeltaImpulseTGS(const float4& raXn0_biasW, const float4& raXn1_biasW,
																	 const float4& raXn2_biasW, const float4& velMultiplierXYZ_invMassW,
																	 const float4& low_high_limits, const float4& worldAxis_angle,
																	 const PxgVelocityPackTGS& vel0, const PxVec3& linDelta1, PxReal dt,
																	 PxReal biasCoefficient, bool isVelocityIteration, PxVec3& deltaLinVel,
																	 PxVec3& deltaAngVel)
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

	// This is a position error as well (as opposed to a velocity error for PGS)
	PxVec3 tgsError(linDelta.x - positionErrorX - vel0.angDelta.dot(raXn0), linDelta.y - positionErrorY - vel0.angDelta.dot(raXn1),
					linDelta.z - positionErrorZ - vel0.angDelta.dot(raXn2));

	if(isConeLimitedEnabled(worldAxis_angle.w, low_high_limits.x, low_high_limits.y))
	{
		const PxVec3 worldAxis(worldAxis_angle.x, worldAxis_angle.y, worldAxis_angle.z);
		tgsError = computeConeLimitedError(worldAxis_angle.w, low_high_limits.x, low_high_limits.y, worldAxis, tgsError);
	}

	const PxReal velDt = isVelocityIteration ? 0.f : dt;

	// deltaF for TGS: position delta multiplied by effective inertia

	// Bias coefficient is already multiplied by dt
	// Bias seems to kind of act like damping
	const PxReal deltaF0 = (tgsError.x - velOfRigidAtAttachmentPoint0 * velDt) * velMultiplierXYZ_invMassW.x * biasCoefficient;
	const PxReal deltaF1 = (tgsError.y - velOfRigidAtAttachmentPoint1 * velDt) * velMultiplierXYZ_invMassW.y * biasCoefficient;
	const PxReal deltaF2 = (tgsError.z - velOfRigidAtAttachmentPoint2 * velDt) * velMultiplierXYZ_invMassW.z * biasCoefficient;

	// const PxVec3 deltaImpulse = (normal0 * deltaF0 + normal1 * deltaF1 + normal2 * deltaF2);
	const PxVec3 deltaImpulse = PxVec3(deltaF0, deltaF1, deltaF2);

	deltaLinVel = deltaImpulse * velMultiplierXYZ_invMassW.w;
	deltaAngVel = raXn0 * deltaF0 + raXn1 * deltaF1 + raXn2 * deltaF2;

	return deltaImpulse;
}

template <typename ConstraintType>
PX_FORCE_INLINE __device__ PxVec3 calculateAttachmentDeltaImpulseTGS(PxU32 offset, const ConstraintType& constraint,
																	 const PxgVelocityPackTGS& vel0, const PxVec3& linDelta1, PxReal dt,
																	 PxReal biasCoefficient, bool isVelocityIteration, PxVec3& deltaLinVel,
																	 PxVec3& deltaAngVel)
{
	return calculateAttachmentDeltaImpulseTGS(constraint.raXn0_biasW[offset], constraint.raXn1_biasW[offset],
											  constraint.raXn2_biasW[offset], constraint.velMultiplierXYZ_invMassW[offset],
											  constraint.low_high_limits[offset], constraint.axis_angle[offset], vel0, linDelta1, dt,
											  biasCoefficient, isVelocityIteration, deltaLinVel, deltaAngVel);
}

#endif // __ATTACHMENTS_CUH__
