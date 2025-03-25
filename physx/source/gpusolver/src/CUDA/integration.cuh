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

#include "foundation/PxSimpleTypes.h"
#include "PxgBodySim.h"
#include "PxgSolverBody.h"
#include "PxvDynamics.h"
#include "PxsRigidBody.h"
#include "PxgSolverKernelIndices.h"
#include "DySleepingConfigulation.h"
#include "stdio.h"

using namespace physx;

static __device__ void updateWakeCounter(bool& freeze, float4& solverBodyLinVel, float4& solverBodyAngVel, const PxAlignedTransform& body2World, 
	PxgBodySim& bodySim, PxgSolverBodySleepData& sleepData,
	const float4& inverseInertia, const PxVec3& linearMotionVel, const PxVec3& angularMotionVel, const float invertedMass, const float dt,
	const float invDt, const bool enableStabilization, const bool hasStaticTouch, PxU32 numCountedInteractions,
	PxU32 nodeIndex)
{
	// update the body's sleep state and 
	PxReal wakeCounterResetTime = 20.0f*0.02f;

	float4 freezeThresholdX_wakeCounterY_sleepThresholdZ_bodySimIndex = bodySim.freezeThresholdX_wakeCounterY_sleepThresholdZ_bodySimIndex;
	float4 sleepLinVelAccXYZ_freezeCountW = bodySim.sleepLinVelAccXYZ_freezeCountW;
	float4 sleepAngVelAccXYZ_accelScaleW = bodySim.sleepAngVelAccXYZ_accelScaleW;

	PxReal wc = freezeThresholdX_wakeCounterY_sleepThresholdZ_bodySimIndex.y; //wakeCounter;

	PxU32 flags = bodySim.internalFlags & (PxsRigidBody::eDISABLE_GRAVITY_GPU | PxsRigidBody::eFROZEN | PxsRigidBody::eENABLE_GYROSCOPIC | PxsRigidBody::eRETAIN_ACCELERATION);
	PxReal freezeCount = sleepLinVelAccXYZ_freezeCountW.w;
	PxReal accelScale = sleepAngVelAccXYZ_accelScaleW.w;

	bool alreadyUpdateWC = false;
	PxVec3 sleepLinVelAcc(0.f), sleepAngVelAcc(0.f);

	{
		if (enableStabilization)
		{
			const PxU32 maxCountedInteractions = 10u; //KS - arbitrary limit to make sure that 
													  //bool freeze = false;
													  //const PxAlignedTransform& body2World = solverBodyData.body2World;

													  // calculate normalized energy: kinetic energy divided by mass
			const PxVec3 inertia(inverseInertia.x > 0.f ? 1.0f / inverseInertia.x : 1.f, inverseInertia.y > 0.f ? 1.0f / inverseInertia.y : 1.f, inverseInertia.z > 0.f ? 1.0f / inverseInertia.z : 1.f);
			sleepLinVelAcc = linearMotionVel;
			sleepAngVelAcc = angularMotionVel;

			// scale threshold by cluster factor (more contacts => higher sleep threshold)
			const PxU32 clusterFactor = PxMin(numCountedInteractions, maxCountedInteractions);

			PxReal invMass = invertedMass;// intialVel_invMass.w;
			if (invMass == 0.f)
				invMass = 1.f;

			const PxReal angular = sleepAngVelAcc.multiply(sleepAngVelAcc).dot(inertia) * invMass;
			const PxReal linear = sleepLinVelAcc.magnitudeSquared();
			PxReal frameNormalizedEnergy = 0.5f * (angular + linear);

			const PxReal cf = hasStaticTouch ? clusterFactor : 0.f;
			const PxReal freezeThresh = cf * freezeThresholdX_wakeCounterY_sleepThresholdZ_bodySimIndex.x;// solverBodySleepData.freezeThreshold;
			freezeCount = PxMax(freezeCount - dt, 0.0f);

			bool settled = true;

			accelScale = PxMin(1.f, accelScale + dt);

			if (frameNormalizedEnergy >= freezeThresh)
			{
				settled = false;

				freezeCount = PXD_FREEZE_INTERVAL;
			}

			if (!hasStaticTouch)
			{
				accelScale = 1.f;
				settled = false;
			}

			if (settled)
			{
				//Dampen bodies that are just about to go to sleep
				if (cf > 1)
				{
					const PxReal d = 1.0f - (PXD_SLEEP_DAMPING * dt);

					solverBodyLinVel = solverBodyLinVel * d;
					solverBodyAngVel = solverBodyAngVel * d;
					accelScale = accelScale * 0.75f + 0.25f*PXD_FREEZE_SCALE;
				}
				
				freeze = freezeCount == 0.f && frameNormalizedEnergy < (freezeThresholdX_wakeCounterY_sleepThresholdZ_bodySimIndex.x * PXD_FREEZE_TOLERANCE);
			}

			if (freeze)
			{
				//current flag isn't frozen but freeze flag raise so we need to raise the frozen flag in this frame

				bool wasNotFrozen = (flags & PxsRigidBody::eFROZEN) == 0;
				flags |= PxsRigidBody::eFROZEN;
				if (wasNotFrozen)
				{
					flags |= PxsRigidBody::eFREEZE_THIS_FRAME;
				}
			}
			else
			{
				bool wasFrozen = (flags & PxsRigidBody::eFROZEN) != 0;
				flags &= (PxsRigidBody::eDISABLE_GRAVITY_GPU | PxsRigidBody::eENABLE_GYROSCOPIC | PxsRigidBody::eRETAIN_ACCELERATION);
				if (wasFrozen)
				{
					flags |= PxsRigidBody::eUNFREEZE_THIS_FRAME;
				}
			}

			/*KS: New algorithm for sleeping when using stabilization:
			* Energy *this frame* must be higher than sleep threshold and accumulated energy over previous frames
			* must be higher than clusterFactor*energyThreshold.
			*/
			if (wc < wakeCounterResetTime * 0.5f || wc < dt)
			{
				//Accumulate energy
				sleepLinVelAcc.x += sleepLinVelAccXYZ_freezeCountW.x;
				sleepLinVelAcc.y += sleepLinVelAccXYZ_freezeCountW.y;
				sleepLinVelAcc.z += sleepLinVelAccXYZ_freezeCountW.z;

				sleepAngVelAcc.x += sleepAngVelAccXYZ_accelScaleW.x;
				sleepAngVelAcc.y += sleepAngVelAccXYZ_accelScaleW.y;
				sleepAngVelAcc.z += sleepAngVelAccXYZ_accelScaleW.z;

				//If energy this frame is high
				const PxReal sleepThreshold = freezeThresholdX_wakeCounterY_sleepThresholdZ_bodySimIndex.z;

				if (frameNormalizedEnergy >= sleepThreshold)
				{
					//Compute energy over sleep preparation time
		
					const PxReal sleepAngular = sleepAngVelAcc.multiply(sleepAngVelAcc).dot(inertia) * invMass;
					const PxReal sleepLinear = sleepLinVelAcc.magnitudeSquared();

					PxReal normalizedEnergy = 0.5f * (sleepAngular + sleepLinear);
					PxReal sleepClusterFactor = clusterFactor + 1.f;

					// scale threshold by cluster factor (more contacts => higher sleep threshold)
					const PxReal threshold = sleepClusterFactor * sleepThreshold;

					//If energy over sleep preparation time is high
					if (normalizedEnergy >= threshold)
					{
						//Wake up
						//assert(isActive());
						
						sleepAngVelAcc = PxVec3(0);
						sleepLinVelAcc = PxVec3(0);

						const float factor = sleepThreshold == 0.f ? 2.0f : PxMin(normalizedEnergy / threshold, 2.0f);
						PxReal oldWc = wc;
						wc = factor * 0.5f * wakeCounterResetTime + dt * (sleepClusterFactor - 1.0f);

						//if (oldWc == 0.0f)  // for the case where a sleeping body got activated by the system (not the user) AND got processed by the solver as well
						//	notifyNotReadyForSleeping(bodyCore.nodeIndex);

						if (oldWc == 0.0f)
							flags |= PxsRigidBody::eACTIVATE_THIS_FRAME;

						alreadyUpdateWC = true;
					}
				}
			}
		}
		else
		{
			if (wc < wakeCounterResetTime * 0.5f || wc < dt)
			{
				//const PxAlignedTransform& body2World = solverBodyData.body2World;

				// calculate normalized energy: kinetic energy divided by mass
				const PxVec3 inertia(inverseInertia.x > 0.f ? 1.0f / inverseInertia.x : 1.f, inverseInertia.y > 0.f ? 1.0f / inverseInertia.y : 1.f, inverseInertia.z > 0.f ? 1.0f / inverseInertia.z : 1.f);

				sleepLinVelAcc = linearMotionVel;// originalBody->mAcceleration.linear;
				sleepAngVelAcc = body2World.q.rotateInv(angularMotionVel);// originalBody->mAcceleration.angular;

				sleepLinVelAcc.x += sleepLinVelAccXYZ_freezeCountW.x;
				sleepLinVelAcc.y += sleepLinVelAccXYZ_freezeCountW.y;
				sleepLinVelAcc.z += sleepLinVelAccXYZ_freezeCountW.z;

				sleepAngVelAcc.x += sleepAngVelAccXYZ_accelScaleW.x;
				sleepAngVelAcc.y += sleepAngVelAccXYZ_accelScaleW.y;
				sleepAngVelAcc.z += sleepAngVelAccXYZ_accelScaleW.z;
				

				PxReal invMass = invertedMass;
				if (invMass == 0.f)
					invMass = 1.f;

				const PxReal angular = sleepAngVelAcc.multiply(sleepAngVelAcc).dot(inertia) * invMass;
				const PxReal linear = sleepLinVelAcc.magnitudeSquared();
				PxReal normalizedEnergy = 0.5f * (angular + linear);

				// scale threshold by cluster factor (more contacts => higher sleep threshold)
				const PxReal clusterFactor = PxReal(1 + numCountedInteractions);

				const PxReal sleepThreshold = freezeThresholdX_wakeCounterY_sleepThresholdZ_bodySimIndex.z;

				const PxReal threshold = clusterFactor * sleepThreshold;

				if (normalizedEnergy >= threshold)
				{
					//assert(isActive());
					sleepLinVelAcc = PxVec3(0);
					sleepAngVelAcc = PxVec3(0);
					const float factor = threshold == 0.f ? 2.0f : PxMin(normalizedEnergy / threshold, 2.0f);
					PxReal oldWc = wc;
					wc = factor * 0.5f * wakeCounterResetTime + dt * (clusterFactor - 1.0f);
					
					if (oldWc == 0.0f)  // for the case where a sleeping body got activated by the system (not the user) AND got processed by the solver as well
					{
						flags |= PxsRigidBody::eACTIVATE_THIS_FRAME;
					}

					alreadyUpdateWC = true;
				}
			}
		}
	}

	if(!alreadyUpdateWC)
		wc = PxMax(wc - dt, 0.0f);

	bool wakeCounterZero = (wc == 0.0f);

	if (wakeCounterZero)
	{
		flags |= PxsRigidBody::eDEACTIVATE_THIS_FRAME;
		sleepLinVelAcc = PxVec3(0);
		sleepAngVelAcc = PxVec3(0);
	}

	bodySim.internalFlags = flags;
	bodySim.freezeThresholdX_wakeCounterY_sleepThresholdZ_bodySimIndex.y = wc;
	bodySim.sleepLinVelAccXYZ_freezeCountW = make_float4(sleepLinVelAcc.x, sleepLinVelAcc.y, sleepLinVelAcc.z, freezeCount);
	bodySim.sleepAngVelAccXYZ_accelScaleW = make_float4(sleepAngVelAcc.x, sleepAngVelAcc.y, sleepAngVelAcc.z, accelScale);

	if (!(flags & PxsRigidBody::eRETAIN_ACCELERATION))
	{
		bodySim.externalLinearAcceleration = make_float4(0.f, 0.f, 0.f, 0.f);
		bodySim.externalAngularAcceleration = make_float4(0.f, 0.f, 0.f, 0.f);
	}

	sleepData.internalFlags = flags;
	sleepData.wakeCounter = wc;
}

static __device__ bool sleepCheck(float4& solverBodyLinVel, float4& solverBodyAngVel, const PxAlignedTransform& body2World, PxgBodySim& bodySim, PxgSolverBodySleepData& sleepData,
	const float4& inverseInertia, const PxVec3& linearMotionVel, const PxVec3& angularMotionVel, const float invertedMass, const float dt, const float invDt, const bool enableStabilization,
	const bool hasStaticTouch, const PxU32 numCountedInteractions, PxU32 nodeIndex)
{
	bool freeze = false;
	updateWakeCounter(freeze, solverBodyLinVel, solverBodyAngVel, body2World, bodySim, sleepData, inverseInertia, linearMotionVel, angularMotionVel, invertedMass, dt,
		invDt, enableStabilization, hasStaticTouch, numCountedInteractions, nodeIndex);

	return freeze;
}

static __device__ void integrateCore(const float4 motionLinVelXYZW, const float4 motionAngVelXYZW, const float4& inverseInertia, float4& solverBodyLinVel,
	float4& solverBodyAngVel, PxAlignedTransform& body2World, const PxgSolverBodyData& solverBodyData, PxgBodySim& bodySim, PxgSolverBodySleepData& sleepData,
	const PxMat33& sqrtInvInertia, const float dt, const float invDt, const bool enableStabilization, const bool hasStaticTouch, const PxU32 numCountedInteractions,
	const PxU32 nodeIndex)
{
	// Integrate linear part
	const float4 initialLinVelXYZ_invMassW = solverBodyData.initialLinVelXYZ_invMassW;
	const float4 initialAngVelXYZ_penBiasClamp = solverBodyData.initialAngVelXYZ_penBiasClamp;
	//ML: solverBodyData.initialLinVelocity store the PxsBodyCore (original )linearVelocity and angularVelocity
	const PxVec3 initialLinVel(initialLinVelXYZ_invMassW.x, initialLinVelXYZ_invMassW.y, initialLinVelXYZ_invMassW.z);
	const PxVec3 initialAngVel(initialAngVelXYZ_penBiasClamp.x, initialAngVelXYZ_penBiasClamp.y, initialAngVelXYZ_penBiasClamp.z);

	PxU32 lockFlags = bodySim.lockFlags;

	//update body lin and ang velocity
	PxVec3 bodyLinearVelocity(solverBodyLinVel.x, solverBodyLinVel.y, solverBodyLinVel.z);
	PxVec3 bodyAngVelocity(solverBodyAngVel.x, solverBodyAngVel.y, solverBodyAngVel.z);

#ifndef IS_TGS_SOLVER
	bodyLinearVelocity = initialLinVel + bodyLinearVelocity;
	bodyAngVelocity = initialAngVel + sqrtInvInertia * bodyAngVelocity;
#else
	bodyAngVelocity = sqrtInvInertia * bodyAngVelocity;
#endif

	solverBodyLinVel = make_float4(lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_X ? 0.f : bodyLinearVelocity.x,
		lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Y ? 0.f : bodyLinearVelocity.y,
		lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Z ? 0.f : bodyLinearVelocity.z, 0.f);
	solverBodyAngVel = make_float4(lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_X ? 0.f : bodyAngVelocity.x,
		lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y ? 0.f : bodyAngVelocity.y,
		lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z ? 0.f : bodyAngVelocity.z, 0.f);

	//we need to perform sleep check here to decide whether we want to update body2World transform for the body
	const PxVec3 motionLinVel(lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_X ? 0.f : motionLinVelXYZW.x,
		lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Y ? 0.f : motionLinVelXYZW.y,
		lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Z ? 0.f : motionLinVelXYZW.z);
	const PxVec3 motionAngVel(lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_X ? 0.f : motionAngVelXYZW.x,
		lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y ? 0.f : motionAngVelXYZW.y,
		lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z ? 0.f : motionAngVelXYZW.z);

#ifndef IS_TGS_SOLVER
	PxVec3 linearMotionVel = initialLinVel + motionLinVel;
	PxVec3 angularMotionVel = initialAngVel + sqrtInvInertia * motionAngVel;

	//printf("%i: DeltaLinVel = (%f, %f, %f)\n", nodeIndex, motionLinVel.x, motionLinVel.y, motionLinVel.z);
#else
	PxVec3 linearMotionVel = motionLinVel;
	PxVec3 angularMotionVel = sqrtInvInertia * motionAngVel;
#endif

	// Integrate the rotation using closed form quaternion integrator
	PxReal w = angularMotionVel.magnitudeSquared();
	w = PxSqrt(w);
	const PxReal maxW = 1e+7f;		//Should be about sqrt(PX_MAX_REAL/2) or smaller
	if (w > maxW)
	{
		angularMotionVel = angularMotionVel.getNormalized() * maxW;
		w = maxW;
	}

	const bool freeze = sleepCheck(solverBodyLinVel, solverBodyAngVel, body2World, bodySim, sleepData, inverseInertia, linearMotionVel, angularMotionVel, initialLinVelXYZ_invMassW.w,
		dt, invDt, enableStabilization, hasStaticTouch, numCountedInteractions, nodeIndex);

	if (!freeze)
	{
		PxVec3 delta = linearMotionVel * dt;
		body2World.p.x += delta.x; body2World.p.y += delta.y; body2World.p.z += delta.z;

		if (w != 0.0f)
		{
			const PxReal v = dt * w * 0.5f;
			PxReal s, q;
			//s = sin(v);
			//q = cos(v);
			__sincosf(v, &s, &q);
			s /= w;

			const PxVec3 pqr = angularMotionVel * s;
			const PxAlignedQuat quatVel(pqr.x, pqr.y, pqr.z, 0);
			PxAlignedQuat result = quatVel * body2World.q;

			result += body2World.q * q;

			//ML: solverBodyData store the current transform for PxsBodyCore
			body2World.q = result.getNormalized();
		}
	}
}

static __device__ void integrateCoreTGS(const float4 motionLinVelXYZW, const float4 motionAngVelXYZW, const float4& inverseInertia, float4& solverBodyLinVel,
	float4& solverBodyAngVel, PxAlignedTransform& body2World, const PxTransform& deltaBody2World, const PxgSolverBodyData& solverBodyData, PxgBodySim& bodySim, PxgSolverBodySleepData& sleepData,
	const PxMat33& sqrtInvInertia, const float dt, const float invDt, const bool enableStabilization, const bool hasStaticTouch, const PxU32 numCountedInteractions,
	const PxU32 nodeIndex)
{
	const PxU32 lockFlags = bodySim.lockFlags;

	//KS - TODO - optimize this away
	const float4 initialLinVelXYZ_invMassW = solverBodyData.initialLinVelXYZ_invMassW;

	//update body lin and ang velocity
	PxVec3 bodyLinearVelocity(solverBodyLinVel.x, solverBodyLinVel.y, solverBodyLinVel.z);
	PxVec3 bodyAngVelocity(solverBodyAngVel.x, solverBodyAngVel.y, solverBodyAngVel.z);
	bodyAngVelocity = sqrtInvInertia * bodyAngVelocity;

	solverBodyLinVel = make_float4(lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_X ? 0.f : bodyLinearVelocity.x,
		lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Y ? 0.f : bodyLinearVelocity.y,
		lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Z ? 0.f : bodyLinearVelocity.z, 0.f);
	solverBodyAngVel = make_float4(lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_X ? 0.f : bodyAngVelocity.x,
		lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y ? 0.f : bodyAngVelocity.y,
		lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z ? 0.f : bodyAngVelocity.z, 0.f);

	//we need to perform sleep check here to decide whether we want to update body2World transform for the body
	const PxVec3 motionLinVel(lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_X ? 0.f : motionLinVelXYZW.x,
		lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Y ? 0.f : motionLinVelXYZW.y,
		lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Z ? 0.f : motionLinVelXYZW.z);
	const PxVec3 motionAngVel(lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_X ? 0.f : motionAngVelXYZW.x,
		lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y ? 0.f : motionAngVelXYZW.y,
		lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z ? 0.f : motionAngVelXYZW.z);

	PxVec3 linearMotionVel = motionLinVel;
	PxVec3 angularMotionVel = sqrtInvInertia * motionAngVel;

	// Integrate the rotation using closed form quaternion integrator
	PxReal w = angularMotionVel.magnitudeSquared();
	w = PxSqrt(w);
	const PxReal maxW = 1e+7f;		//Should be about sqrt(PX_MAX_REAL/2) or smaller
	if (w > maxW)
	{
		angularMotionVel = angularMotionVel.getNormalized() * maxW;
		w = maxW;
	}

	const bool freeze = sleepCheck(solverBodyLinVel, solverBodyAngVel, body2World, bodySim, sleepData, inverseInertia, linearMotionVel, angularMotionVel, initialLinVelXYZ_invMassW.w,
		dt, invDt, enableStabilization, hasStaticTouch, numCountedInteractions, nodeIndex);

	if (!freeze)
	{
		//printf("DeltaP = (%f, %f, %f)\n", deltaBody2World.p.x, deltaBody2World.p.y, deltaBody2World.p.z);
		body2World.p.x += deltaBody2World.p.x; body2World.p.y += deltaBody2World.p.y; body2World.p.z += deltaBody2World.p.z;
		PxQuat q(body2World.q.q.x, body2World.q.q.y, body2World.q.q.z, body2World.q.q.w);
		q = (deltaBody2World.q * q).getNormalized();

		body2World.q.q.x = q.x; body2World.q.q.y = q.y; body2World.q.q.z = q.z; body2World.q.q.w = q.w;
	}
}
