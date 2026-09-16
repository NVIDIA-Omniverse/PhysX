// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "CmUtils.h"
#include "DySolverBody.h"
#include "PxsRigidBody.h"
#include "PxvDynamics.h"

using namespace physx;

// PT: TODO: SIMDify all this...
void Dy::copyToSolverBodyData(const PxVec3& linearVelocity, const PxVec3& angularVelocity, PxReal invMass, const PxVec3& invInertia, const PxTransform& globalPose,
	PxReal maxDepenetrationVelocity, PxReal maxContactImpulse, PxU32 nodeIndex, PxReal reportThreshold, PxSolverBodyData& data, PxU32 lockFlags,
	PxReal dt, bool gyroscopicForces)
{
	data.nodeIndex = nodeIndex;

	const PxVec3p safeSqrtInvInertia = computeSafeSqrtInertia(invInertia);

	Cm::transformInertiaTensor(safeSqrtInvInertia, globalPose.q, data.sqrtInvInertia);

	PxVec3 ang = angularVelocity;
	PxVec3 lin = linearVelocity;

	if (gyroscopicForces)
	{
		const PxVec3 localInertia = Cm::safeRecip<PxVec3>(invInertia);

		const PxVec3 localAngVel = globalPose.q.rotateInv(ang);
		const PxVec3 origMom = localInertia.multiply(localAngVel);
		const PxVec3 torque = -localAngVel.cross(origMom);
		PxVec3 newMom = origMom + torque * dt;
		const PxReal denom = newMom.magnitude();
		const PxReal ratio = denom > 0.f ? origMom.magnitude() / denom : 0.f;
		newMom *= ratio;
		const PxVec3 newDeltaAngVel = globalPose.q.rotate(invInertia.multiply(newMom) - localAngVel);

		ang += newDeltaAngVel;
	}
	
	if (lockFlags)
	{
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_X)
			data.linearVelocity.x = 0.f;
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Y)
			data.linearVelocity.y = 0.f;
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Z)
			data.linearVelocity.z = 0.f;

		//KS - technically, we can zero the inertia columns and produce stiffer constraints. However, this can cause numerical issues with the 
		//joint solver, which is fixed by disabling joint preprocessing and setting minResponseThreshold to some reasonable value > 0. However, until
		//this is handled automatically, it's probably better not to zero these inertia rows
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_X)
		{
			ang.x = 0.f;
			//data.sqrtInvInertia.column0 = PxVec3(0.f);
		}
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y)
		{
			ang.y = 0.f;
			//data.sqrtInvInertia.column1 = PxVec3(0.f);
		}
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z)
		{
			ang.z = 0.f;
			//data.sqrtInvInertia.column2 = PxVec3(0.f);
		}
	}

	PX_ASSERT(lin.isFinite());
	PX_ASSERT(ang.isFinite());

	data.angularVelocity = ang;
	data.linearVelocity = lin;

	data.invMass = invMass;
	data.penBiasClamp = maxDepenetrationVelocity;
	data.maxContactImpulse = maxContactImpulse;
	data.body2World = globalPose;

	data.reportThreshold = reportThreshold;
}
