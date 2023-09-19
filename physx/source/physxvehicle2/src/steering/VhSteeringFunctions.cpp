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

#include "vehicle2/PxVehicleParams.h"

#include "vehicle2/commands/PxVehicleCommandHelpers.h"

#include "vehicle2/rigidBody/PxVehicleRigidBodyStates.h"

#include "vehicle2/steering/PxVehicleSteeringParams.h"

namespace physx
{
namespace vehicle2
{

void PxVehicleSteerCommandResponseUpdate
(const PxReal steer, const PxReal longitudinalSpeed,
 const PxU32 wheelId, const PxVehicleSteerCommandResponseParams& responseParams,
 PxReal& steerResponse)
{
	PxReal sign = PxSign(steer);
	steerResponse = sign * PxVehicleNonLinearResponseCompute(PxAbs(steer), longitudinalSpeed, wheelId, responseParams);
}

void PxVehicleAckermannSteerUpdate
(const PxReal steer,
 const PxVehicleSteerCommandResponseParams& steerResponseParams, const PxVehicleSizedArrayData<const PxVehicleAckermannParams>& ackermannParams,
 PxVehicleArrayData<PxReal>& steerResponseStates)
{
	for (PxU32 i = 0; i < ackermannParams.size; i++)
	{
		const PxVehicleAckermannParams& ackParams = ackermannParams[i];
		if (ackParams.strength > 0.0f)
		{
			//Axle yaw is the average of the two wheels.
			const PxF32 axleYaw =
				(PxVehicleLinearResponseCompute(steer, ackParams.wheelIds[0], steerResponseParams) +
					PxVehicleLinearResponseCompute(steer, ackParams.wheelIds[1], steerResponseParams))*0.5f;
			if (axleYaw != 0.0f)
			{
				//Work out the ackermann steer for +ve steer then swap and negate the steer angles if the steer is -ve.

				//Uncorrected yaw angle.  
				//One of the wheels will adopt this angle. 
				//The other will be corrected.
				const PxF32 posWheelYaw = PxAbs(axleYaw);

				//Work out the yaw of the other wheel.
				PxF32 negWheelCorrectedYaw;
				{
					const PxF32 dz = ackParams.wheelBase;
					const PxF32 dx = ackParams.trackWidth + ackParams.wheelBase / PxTan(posWheelYaw);
					const PxF32 negWheelPerfectYaw = PxAtan(dz / dx);
					negWheelCorrectedYaw = posWheelYaw + ackParams.strength*(negWheelPerfectYaw - posWheelYaw);
				}

				//Now assign  axleYaw and  negWheelCorrectedYaw to the correct wheels with the correct signs. 
				const PxF32 negWheelFinalYaw = intrinsics::fsel(axleYaw, negWheelCorrectedYaw, -posWheelYaw);
				const PxF32 posWheelFinalYaw = intrinsics::fsel(axleYaw, posWheelYaw, -negWheelCorrectedYaw);

				//Apply the per axle distributions to each wheel on the axle that is affected by this Ackermann correction.
				steerResponseStates[ackParams.wheelIds[0]] = negWheelFinalYaw;
				steerResponseStates[ackParams.wheelIds[1]] = posWheelFinalYaw;
			}
		}
	}
}

} //namespace vehicle2
} //namespace physx
