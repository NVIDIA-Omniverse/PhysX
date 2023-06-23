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

#ifndef DY_BODYCORE_INTEGRATOR_H
#define DY_BODYCORE_INTEGRATOR_H

#include "PxvDynamics.h"
#include "DySolverBody.h"

namespace physx
{
namespace Dy
{
PX_FORCE_INLINE void bodyCoreComputeUnconstrainedVelocity
	(const PxVec3& gravity, PxReal dt, PxReal linearDamping, PxReal angularDamping, PxReal accelScale, 
	PxReal maxLinearVelocitySq, PxReal maxAngularVelocitySq, PxVec3& inOutLinearVelocity, PxVec3& inOutAngularVelocity,
	bool disableGravity)
{
	//Multiply everything that needs multiplied by dt to improve code generation.

	PxVec3 linearVelocity = inOutLinearVelocity;
	PxVec3 angularVelocity = inOutAngularVelocity;
	
	const PxReal linearDampingTimesDT=linearDamping*dt;
	const PxReal angularDampingTimesDT=angularDamping*dt;
	const PxReal oneMinusLinearDampingTimesDT=1.0f-linearDampingTimesDT;
	const PxReal oneMinusAngularDampingTimesDT=1.0f-angularDampingTimesDT;

	//TODO context-global gravity
	if(!disableGravity)
	{
		const PxVec3 linearAccelTimesDT = gravity*dt *accelScale;
		linearVelocity += linearAccelTimesDT;
	}

	//Apply damping.
	const PxReal linVelMultiplier = physx::intrinsics::fsel(oneMinusLinearDampingTimesDT, oneMinusLinearDampingTimesDT, 0.0f);
	const PxReal angVelMultiplier = physx::intrinsics::fsel(oneMinusAngularDampingTimesDT, oneMinusAngularDampingTimesDT, 0.0f);
	linearVelocity*=linVelMultiplier;
	angularVelocity*=angVelMultiplier;

	// Clamp velocity
	const PxReal linVelSq = linearVelocity.magnitudeSquared();
	if(linVelSq > maxLinearVelocitySq)
	{
		linearVelocity *= PxSqrt(maxLinearVelocitySq / linVelSq);
	}
	const PxReal angVelSq = angularVelocity.magnitudeSquared();
	if(angVelSq > maxAngularVelocitySq)
	{
		angularVelocity *= PxSqrt(maxAngularVelocitySq / angVelSq);
	}

	inOutLinearVelocity = linearVelocity;
	inOutAngularVelocity = angularVelocity;
}

PX_FORCE_INLINE void integrateCore(PxVec3& motionLinearVelocity, PxVec3& motionAngularVelocity, 
	PxSolverBody& solverBody, PxSolverBodyData& solverBodyData, PxF32 dt, PxU32 lockFlags)
{
	if (lockFlags)
	{
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_X)
		{
			motionLinearVelocity.x = 0.f;
			solverBody.linearVelocity.x = 0.f;
			solverBodyData.linearVelocity.x = 0.f;
		}
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Y)
		{
			motionLinearVelocity.y = 0.f;
			solverBody.linearVelocity.y = 0.f;
			solverBodyData.linearVelocity.y = 0.f;
		}
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Z)
		{
			motionLinearVelocity.z = 0.f;
			solverBody.linearVelocity.z = 0.f;
			solverBodyData.linearVelocity.z = 0.f;
		}
		
		//The angular velocity should be 0 because it is now impossible to make it rotate around that axis!
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_X)
		{
			motionAngularVelocity.x = 0.f;
			solverBody.angularState.x = 0.f;
		}
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y)
		{
			motionAngularVelocity.y = 0.f;
			solverBody.angularState.y = 0.f;
		}
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z)
		{
			motionAngularVelocity.z = 0.f;
			solverBody.angularState.z = 0.f;
		}
	}

	// Integrate linear part
	const PxVec3 linearMotionVel = solverBodyData.linearVelocity + motionLinearVelocity;
	const PxVec3 delta = linearMotionVel * dt;
	PxVec3 angularMotionVel = solverBodyData.angularVelocity + solverBodyData.sqrtInvInertia * motionAngularVelocity;
	PxReal w = angularMotionVel.magnitudeSquared();
	solverBodyData.body2World.p += delta;
	PX_ASSERT(solverBodyData.body2World.p.isFinite());

	//Store back the linear and angular velocities
	//core.linearVelocity += solverBody.linearVelocity * solverBodyData.sqrtInvMass;
	solverBodyData.linearVelocity += solverBody.linearVelocity;
	solverBodyData.angularVelocity += solverBodyData.sqrtInvInertia * solverBody.angularState;
	
	// Integrate the rotation using closed form quaternion integrator
	if (w != 0.0f)
	{
		w = PxSqrt(w);
		// Perform a post-solver clamping
		// TODO(dsequeira): ignore this for the moment
		//just clamp motionVel to half float-range
		const PxReal maxW = 1e+7f;		//Should be about sqrt(PX_MAX_REAL/2) or smaller
		if (w > maxW)
		{
			angularMotionVel = angularMotionVel.getNormalized() * maxW;
			w = maxW;
		}
		const PxReal v = dt * w * 0.5f;
		PxReal s, q;
		PxSinCos(v, s, q);
		s /= w;

		const PxVec3 pqr = angularMotionVel * s;
		const PxQuat quatVel(pqr.x, pqr.y, pqr.z, 0);
		PxQuat result = quatVel * solverBodyData.body2World.q;

		result += solverBodyData.body2World.q * q;

		solverBodyData.body2World.q = result.getNormalized();
		PX_ASSERT(solverBodyData.body2World.q.isSane());
		PX_ASSERT(solverBodyData.body2World.q.isFinite());
	}

	motionLinearVelocity = linearMotionVel;
	motionAngularVelocity = angularMotionVel;
}
}
}

#endif
