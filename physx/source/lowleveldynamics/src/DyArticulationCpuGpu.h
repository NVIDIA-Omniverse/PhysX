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

#ifndef DY_ARTICULATION_CPUGPU_H
#define DY_ARTICULATION_CPUGPU_H

#include "foundation/PxSimpleTypes.h"
#include "PxArticulationJointReducedCoordinate.h"
#include "DyFeatherstoneArticulation.h"

#define DY_ARTICULATION_MIN_RESPONSE 1e-5f
#define DY_ARTICULATION_CFM	2e-4f
#define DY_ARTICULATION_PGS_BIAS_COEFFICIENT 0.8f

namespace physx
{
namespace Dy
{

PX_CUDA_CALLABLE PX_FORCE_INLINE ArticulationImplicitDriveDesc computeImplicitDriveParamsForceDrive
(const PxReal stiffness, const PxReal damping, const PxReal dt,
 const PxReal unitResponse, const PxReal geomError, const PxReal targetVelocity, const bool isTGSSolver)
{
	ArticulationImplicitDriveDesc driveDesc(PxZero);
	const PxReal a = dt * (dt*stiffness + damping);
	const PxReal b = dt * (damping * targetVelocity);
	const PxReal x = unitResponse > 0.f ? 1.0f / (1.0f + a*unitResponse) : 0.f;
	const PxReal driveBiasCoefficient = stiffness * x * dt;
	driveDesc.driveTargetVelPlusInitialBias = (x * b) + (geomError * driveBiasCoefficient);
	driveDesc.driveVelMultiplier = -x * a;
	driveDesc.driveBiasCoefficient = driveBiasCoefficient;
	driveDesc.driveImpulseMultiplier = isTGSSolver ? 1.f : 1.0f - x;
	return driveDesc;
}

PX_CUDA_CALLABLE PX_FORCE_INLINE ArticulationImplicitDriveDesc computeImplicitDriveParamsAccelerationDrive
(const PxReal stiffness, const PxReal damping, const PxReal dt,
 const PxReal recipUnitResponse, const PxReal geomError, const PxReal targetVelocity, const bool isTGSSolver)
{
	ArticulationImplicitDriveDesc driveDesc(PxZero);
	const PxReal a = dt * (dt*stiffness + damping);
	const PxReal b = dt * (damping * targetVelocity);
	const PxReal x = 1.0f / (1.0f + a);				
	const PxReal driveBiasCoefficient = stiffness * x * recipUnitResponse * dt;
	driveDesc.driveTargetVelPlusInitialBias = (x * b * recipUnitResponse)  + (geomError * driveBiasCoefficient);
	driveDesc.driveVelMultiplier = -x * a * recipUnitResponse;
	driveDesc.driveBiasCoefficient = driveBiasCoefficient;
	driveDesc.driveImpulseMultiplier = isTGSSolver ? 1.f : 1.0f - x;
	return driveDesc;	
}

/**
\brief Compute the parameters for an implicitly integrated spring.
\param[in] driveType is the type of drive. 
\param[in] stiffness is the drive stiffness (force per unit position bias)
\param[in] damping is the drive damping (force per unit velocity bias)
\param[in] dt is the timestep that will be used to forward integrate the spring position bias.
\param[in] unitResponse is the multiplier that converts impulse to velocity change.
\param[in] recipUnitResponse is the reciprocal of unitResponse
\param[in] geomError is the position bias with value (targetPos - currentPos)
\param[in] targetVelocity is the target velocity of the drive.
\param[in] isTGSSolver should be set true when computing implicit spring params for TGS and false for PGS.
\return The implicit spring parameters.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE ArticulationImplicitDriveDesc computeImplicitDriveParams
(const PxArticulationDriveType::Enum driveType, const PxReal stiffness, const PxReal damping, const PxReal dt,
 const PxReal unitResponse, const PxReal recipUnitResponse, const PxReal geomError, const PxReal targetVelocity, const bool isTGSSolver)
{
	ArticulationImplicitDriveDesc driveDesc(PxZero);
	switch (driveType)
	{
	case PxArticulationDriveType::eFORCE:
	{
		driveDesc = computeImplicitDriveParamsForceDrive(stiffness, damping, dt, unitResponse, geomError, targetVelocity, isTGSSolver);
	}
	break;
	case PxArticulationDriveType::eACCELERATION:
	{
		driveDesc = computeImplicitDriveParamsAccelerationDrive(stiffness, damping, dt, recipUnitResponse, geomError, targetVelocity, isTGSSolver);
	}
	break;
	case PxArticulationDriveType::eTARGET:
	{
		driveDesc = computeImplicitDriveParamsForceDrive(1e+25f, 0.0f, dt, unitResponse, geomError, targetVelocity, isTGSSolver);
	}
	break;
	case PxArticulationDriveType::eVELOCITY:
	{
		driveDesc = computeImplicitDriveParamsForceDrive(0.0f, 1e+25f, dt, unitResponse, geomError, targetVelocity, isTGSSolver);
	}
	break;
	case PxArticulationDriveType::eNONE:
	{
		PX_ASSERT(false);
	}
	break;
	}
	return driveDesc;
}

/**
\brief Compute the drive impulse for an implicitly integrated spring. 
\param[in] accumulatedDriveImpulse is the drive impulse that has accumulated since the the solver started on the current simulation step.
\param[in] jointVel is the current velocity of the joint.
\param[in] jointDeltaPos is the change in joint position that has accumulated since the the solver started on the current simulation step. 
\param[in] driveDesc is the implicit spring params.
\return The impulse for the implicitly integrated spring.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal computeDriveImpulse
(const PxReal accumulatedDriveImpulse, const PxReal jointVel, const PxReal jointDeltaPos, const ArticulationImplicitDriveDesc& driveDesc)
{
	const PxReal unclampedForce = 
		accumulatedDriveImpulse * driveDesc.driveImpulseMultiplier 
		+ jointVel * driveDesc.driveVelMultiplier
		+ driveDesc.driveTargetVelPlusInitialBias
		- jointDeltaPos * driveDesc.driveBiasCoefficient;
	return unclampedForce;
}

} //namespace Dy
} //namespace physx
#endif //DY_ARTICULATION_CPUGPU_H

