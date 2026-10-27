// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "vehicle/PxVehicleParams.h"

#include "vehicle/commands/PxVehicleCommandHelpers.h"

#include "vehicle/rigidBody/PxVehicleRigidBodyStates.h"

#include "vehicle/steering/PxVehicleSteeringParams.h"

namespace physx
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

} //namespace physx
