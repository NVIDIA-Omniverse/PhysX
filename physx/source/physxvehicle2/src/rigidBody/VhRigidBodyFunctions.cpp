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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "foundation/PxMat33.h"
#include "foundation/PxTransform.h"

#include "vehicle2/PxVehicleParams.h"

#include "vehicle2/rigidBody/PxVehicleRigidBodyFunctions.h"
#include "vehicle2/rigidBody/PxVehicleRigidBodyStates.h"
#include "vehicle2/rigidBody/PxVehicleRigidBodyParams.h"

#include "vehicle2/suspension/PxVehicleSuspensionStates.h"

#include "vehicle2/tire/PxVehicleTireStates.h"

namespace physx
{
namespace vehicle2
{

PX_FORCE_INLINE void transformInertiaTensor(const PxVec3& invD, const PxMat33& M, PxMat33& mIInv)
{
	const float	axx = invD.x*M(0, 0), axy = invD.x*M(1, 0), axz = invD.x*M(2, 0);
	const float	byx = invD.y*M(0, 1), byy = invD.y*M(1, 1), byz = invD.y*M(2, 1);
	const float	czx = invD.z*M(0, 2), czy = invD.z*M(1, 2), czz = invD.z*M(2, 2);

	mIInv(0, 0) = axx * M(0, 0) + byx * M(0, 1) + czx * M(0, 2);
	mIInv(1, 1) = axy * M(1, 0) + byy * M(1, 1) + czy * M(1, 2);
	mIInv(2, 2) = axz * M(2, 0) + byz * M(2, 1) + czz * M(2, 2);

	mIInv(0, 1) = mIInv(1, 0) = axx * M(1, 0) + byx * M(1, 1) + czx * M(1, 2);
	mIInv(0, 2) = mIInv(2, 0) = axx * M(2, 0) + byx * M(2, 1) + czx * M(2, 2);
	mIInv(1, 2) = mIInv(2, 1) = axy * M(2, 0) + byy * M(2, 1) + czy * M(2, 2);
}

PX_FORCE_INLINE void integrateBody
(const PxF32 mass, const PxVec3& moi, const PxVec3& force, const PxVec3& torque, const PxF32 dt,
 PxVec3& linvel, PxVec3& angvel, PxTransform& t)
{
	const PxF32 inverseMass = 1.0f/mass;
	const PxVec3 inverseMOI(1.0f/moi.x, 1.0f/moi.y, 1.0f/moi.z);

	//Integrate linear velocity.
	linvel += force * (inverseMass*dt);

	//Integrate angular velocity.
	PxMat33 inverseInertia;
	transformInertiaTensor(inverseMOI, PxMat33(t.q), inverseInertia);
	angvel += inverseInertia * (torque*dt);

	//Integrate position.
	t.p += linvel * dt;

	//Integrate quaternion.
	PxQuat wq(angvel.x, angvel.y, angvel.z, 0.0f);
	PxQuat q = t.q;
	PxQuat qdot = wq * q*(dt*0.5f);
	q += qdot;
	q.normalize();
	t.q = q;
}

void PxVehicleRigidBodyUpdate
(const PxVehicleAxleDescription& axleDescription, const PxVehicleRigidBodyParams& rigidBodyParams, 
 const PxVehicleArrayData<const PxVehicleSuspensionForce>& suspensionForces,
 const PxVehicleArrayData<const PxVehicleTireForce>& tireForces,
 const PxVehicleAntiRollTorque* antiRollTorque,
 const PxReal dt, const PxVec3& gravity,
 PxVehicleRigidBodyState& rigidBodyState)
{
	//Sum all the forces and torques.
	const PxU32 nbAxles = axleDescription.getNbAxles();
	PxVec3 force(PxZero);
	PxVec3 torque(PxZero);
	for (PxU32 i = 0; i < nbAxles; i++)
	{
		PxVec3 axleSuspForce(PxZero);
		PxVec3 axleTireLongForce(PxZero);
		PxVec3 axleTireLatForce(PxZero);
		PxVec3 axleSuspTorque(PxZero);
		PxVec3 axleTireLongTorque(PxZero);
		PxVec3 axleTireLatTorque(PxZero);
		for (PxU32 j = 0; j < axleDescription.getNbWheelsOnAxle(i); j++)
		{
			const PxU32 wheelId = axleDescription.getWheelOnAxle(j, i);
			const PxVehicleSuspensionForce& suspForce = suspensionForces[wheelId];
			const PxVehicleTireForce& tireForce = tireForces[wheelId];
			axleSuspForce += suspForce.force;
			axleTireLongForce += tireForce.forces[PxVehicleTireDirectionModes::eLONGITUDINAL];
			axleTireLatForce += tireForce.forces[PxVehicleTireDirectionModes::eLATERAL];
			axleSuspTorque += suspForce.torque;
			axleTireLongTorque += tireForce.torques[PxVehicleTireDirectionModes::eLONGITUDINAL];
			axleTireLatTorque += tireForce.torques[PxVehicleTireDirectionModes::eLATERAL];
		}
		const PxVec3 axleForce = axleSuspForce + axleTireLongForce + axleTireLatForce;
		const PxVec3 axleTorque = axleSuspTorque + axleTireLongTorque + axleTireLatTorque;
		force += axleForce;
		torque += axleTorque;
	}
	force += gravity * rigidBodyParams.mass;
	force += rigidBodyState.externalForce;
	torque += rigidBodyState.externalTorque;
	torque += (antiRollTorque ? antiRollTorque->antiRollTorque : PxVec3(PxZero));

	//Rigid body params.
	const PxF32 mass = rigidBodyParams.mass;
	const PxVec3& moi = rigidBodyParams.moi;

	//Perform  the integration.
	PxTransform& t = rigidBodyState.pose;
	PxVec3& linvel = rigidBodyState.linearVelocity;
	PxVec3& angvel = rigidBodyState.angularVelocity;
	integrateBody(
		mass, moi,
		force, torque, dt,
		linvel, angvel, t);

	//Reset the accumulated external forces after using them.
	rigidBodyState.externalForce = PxVec3(PxZero);
	rigidBodyState.externalTorque = PxVec3(PxZero);
}

} //namespace vehicle2
} //namespace physx
