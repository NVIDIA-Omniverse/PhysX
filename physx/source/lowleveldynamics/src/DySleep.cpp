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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "DySleep.h"

using namespace physx;

// PT: TODO: refactor this, parts of the two codepaths are very similar

static PX_FORCE_INLINE PxReal updateWakeCounter(PxsRigidBody* originalBody, PxReal dt, bool enableStabilization, const Cm::SpatialVector& motionVelocity, bool hasStaticTouch)
{
	PxsBodyCore& bodyCore = originalBody->getCore();

	// update the body's sleep state and 
	const PxReal wakeCounterResetTime = 20.0f*0.02f;

	PxReal wc = bodyCore.wakeCounter;

	if (enableStabilization)
	{
		const PxTransform& body2World = bodyCore.body2World;

		// calculate normalized energy: kinetic energy divided by mass

		const PxVec3& t = bodyCore.inverseInertia;
		const PxVec3 inertia(	t.x > 0.0f ? 1.0f / t.x : 1.0f,
								t.y > 0.0f ? 1.0f / t.y : 1.0f,
								t.z > 0.0f ? 1.0f / t.z : 1.0f);

		const PxVec3& sleepLinVelAcc = motionVelocity.linear;
		const PxVec3 sleepAngVelAcc = body2World.q.rotateInv(motionVelocity.angular);

		// scale threshold by cluster factor (more contacts => higher sleep threshold)
		//const PxReal clusterFactor = PxReal(1u + getNumUniqueInteractions());

		PxReal invMass = bodyCore.inverseMass;
		if (invMass == 0.0f)
			invMass = 1.0f;

		const PxReal angular = sleepAngVelAcc.multiply(sleepAngVelAcc).dot(inertia) * invMass;
		const PxReal linear = sleepLinVelAcc.magnitudeSquared();
		const PxReal frameNormalizedEnergy = 0.5f * (angular + linear);

		const PxReal cf = hasStaticTouch ? PxReal(PxMin(10u, bodyCore.numCountedInteractions)) : 0.0f;
		const PxReal freezeThresh = cf*bodyCore.freezeThreshold;

		originalBody->freezeCount = PxMax(originalBody->freezeCount - dt, 0.0f);
		bool settled = true;

		PxReal accelScale = PxMin(1.0f, originalBody->accelScale + dt);

		if (frameNormalizedEnergy >= freezeThresh)
		{
			settled = false;
			originalBody->freezeCount = PXD_FREEZE_INTERVAL;
		}

		if (!hasStaticTouch)
		{
			accelScale = 1.0f;
			settled = false;
		}

		bool freeze = false;
		if (settled)
		{
			//Dampen bodies that are just about to go to sleep
			if (cf > 1.0f)
			{
				const PxReal sleepDamping = PXD_SLEEP_DAMPING;
				const PxReal sleepDampingTimesDT = sleepDamping*dt;
				const PxReal d = 1.0f - sleepDampingTimesDT;
				bodyCore.linearVelocity = bodyCore.linearVelocity * d;
				bodyCore.angularVelocity = bodyCore.angularVelocity * d;
				accelScale = accelScale * 0.75f + 0.25f*PXD_FREEZE_SCALE;
			}
			freeze = originalBody->freezeCount == 0.0f && frameNormalizedEnergy < (bodyCore.freezeThreshold * PXD_FREEZE_TOLERANCE);
		}

		originalBody->accelScale = accelScale;

		const PxU32 wasFrozen = originalBody->mInternalFlags & PxsRigidBody::eFROZEN;
		PxU16 flags;
		if(freeze)
		{
			//current flag isn't frozen but freeze flag raise so we need to raise the frozen flag in this frame
			flags = PxU16(PxsRigidBody::eFROZEN);
			if(!wasFrozen)
				flags |= PxsRigidBody::eFREEZE_THIS_FRAME;
			bodyCore.body2World = originalBody->getLastCCDTransform();
		}
		else
		{
			flags = 0;
			if(wasFrozen)
				flags |= PxsRigidBody::eUNFREEZE_THIS_FRAME;
		}
		originalBody->mInternalFlags = flags;

		/*KS: New algorithm for sleeping when using stabilization:
		* Energy *this frame* must be higher than sleep threshold and accumulated energy over previous frames
		* must be higher than clusterFactor*energyThreshold.
		*/
		if (wc < wakeCounterResetTime * 0.5f || wc < dt)
		{
			//Accumulate energy
			originalBody->sleepLinVelAcc += sleepLinVelAcc;
			originalBody->sleepAngVelAcc += sleepAngVelAcc;

			//If energy this frame is high
			if (frameNormalizedEnergy >= bodyCore.sleepThreshold)
			{
				//Compute energy over sleep preparation time
				const PxReal sleepAngular = originalBody->sleepAngVelAcc.multiply(originalBody->sleepAngVelAcc).dot(inertia) * invMass;
				const PxReal sleepLinear = originalBody->sleepLinVelAcc.magnitudeSquared();
				const PxReal normalizedEnergy = 0.5f * (sleepAngular + sleepLinear);
				const PxReal sleepClusterFactor = PxReal(1u + bodyCore.numCountedInteractions);
				// scale threshold by cluster factor (more contacts => higher sleep threshold)
				const PxReal threshold = sleepClusterFactor*bodyCore.sleepThreshold;

				//If energy over sleep preparation time is high
				if (normalizedEnergy >= threshold)
				{
					//Wake up
					//PX_ASSERT(isActive());
					originalBody->resetSleepFilter();

					const float factor = bodyCore.sleepThreshold == 0.0f ? 2.0f : PxMin(normalizedEnergy / threshold, 2.0f);
					PxReal oldWc = wc;
					wc = factor * 0.5f * wakeCounterResetTime + dt * (sleepClusterFactor - 1.0f);
					bodyCore.solverWakeCounter = wc;
					//if (oldWc == 0.0f)  // for the case where a sleeping body got activated by the system (not the user) AND got processed by the solver as well
					//	notifyNotReadyForSleeping(bodyCore.nodeIndex);

					if (oldWc == 0.0f)
						originalBody->mInternalFlags |= PxsRigidBody::eACTIVATE_THIS_FRAME;

					return wc;
				}
			}
		}
	}
	else 
	{
		if (wc < wakeCounterResetTime * 0.5f || wc < dt)
		{
			const PxTransform& body2World = bodyCore.body2World;

			// calculate normalized energy: kinetic energy divided by mass
			const PxVec3& t = bodyCore.inverseInertia;
			const PxVec3 inertia(	t.x > 0.0f ? 1.0f / t.x : 1.0f,
									t.y > 0.0f ? 1.0f / t.y : 1.0f,
									t.z > 0.0f ? 1.0f / t.z : 1.0f);

			const PxVec3& sleepLinVelAcc = motionVelocity.linear;
			const PxVec3 sleepAngVelAcc = body2World.q.rotateInv(motionVelocity.angular);

			originalBody->sleepLinVelAcc += sleepLinVelAcc;
			originalBody->sleepAngVelAcc += sleepAngVelAcc;

			PxReal invMass = bodyCore.inverseMass;
			if (invMass == 0.0f)
				invMass = 1.0f;

			const PxReal angular = originalBody->sleepAngVelAcc.multiply(originalBody->sleepAngVelAcc).dot(inertia) * invMass;
			const PxReal linear = originalBody->sleepLinVelAcc.magnitudeSquared();
			const PxReal normalizedEnergy = 0.5f * (angular + linear);

			// scale threshold by cluster factor (more contacts => higher sleep threshold)
			const PxReal clusterFactor = PxReal(1 + bodyCore.numCountedInteractions);
			const PxReal threshold = clusterFactor*bodyCore.sleepThreshold;

			if (normalizedEnergy >= threshold)
			{
				//PX_ASSERT(isActive());
				originalBody->resetSleepFilter();

				const float factor = threshold == 0.0f ? 2.0f : PxMin(normalizedEnergy / threshold, 2.0f);
				PxReal oldWc = wc;
				wc = factor * 0.5f * wakeCounterResetTime + dt * (clusterFactor - 1.0f);
				bodyCore.solverWakeCounter = wc;
				PxU16 flags = 0;
				if (oldWc == 0.0f)  // for the case where a sleeping body got activated by the system (not the user) AND got processed by the solver as well
				{
					flags |= PxsRigidBody::eACTIVATE_THIS_FRAME;
					//notifyNotReadyForSleeping(bodyCore.nodeIndex);
				}

				originalBody->mInternalFlags = flags;

				return wc;
			}
		}
	}

	wc = PxMax(wc - dt, 0.0f);
	bodyCore.solverWakeCounter = wc;
	return wc;
}

void Dy::sleepCheck(PxsRigidBody* originalBody, PxReal dt, bool enableStabilization, const Cm::SpatialVector& motionVelocity, bool hasStaticTouch)
{
	const PxReal wc = updateWakeCounter(originalBody, dt, enableStabilization, motionVelocity, hasStaticTouch);
	if(wc == 0.0f)
	{
		//PxsBodyCore& bodyCore = originalBody->getCore();
		originalBody->mInternalFlags |= PxsRigidBody::eDEACTIVATE_THIS_FRAME;
		//	notifyReadyForSleeping(bodyCore.nodeIndex);
		originalBody->resetSleepFilter();
	}
}
