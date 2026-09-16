// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "vehicle/tire/PxVehicleTireHelpers.h"

namespace physx
{

bool PxVehicleAccelerationIntentCompute
(const PxVehicleAxleDescription& driveVehicleAxleDesc, const PxVehicleArrayData<const PxVehicleWheelActuationState>& driveVehicleActuationStates)
{
	bool isIntentionToAccelerate = false;
	for (PxU32 i = 0; i < driveVehicleAxleDesc.nbWheels; i++)
	{
		const PxU32 wheelId = driveVehicleAxleDesc.wheelIdsInAxleOrder[i];
		isIntentionToAccelerate = !isIntentionToAccelerate ? driveVehicleActuationStates[wheelId].isDriveApplied : isIntentionToAccelerate;
	}
	return isIntentionToAccelerate;
}

void PxVehicleTireStickyStateReset
(const bool intentionToAccelerate, 
 const PxVehicleAxleDescription& towedVehicleAxleDesc,
 PxVehicleArrayData<PxVehicleTireStickyState>& towedVehicleStickyState)
{
	if (!intentionToAccelerate)
		return;

	for (PxU32 i = 0; i < towedVehicleAxleDesc.nbWheels; i++)
	{
		const PxU32 wheelId = towedVehicleAxleDesc.wheelIdsInAxleOrder[i];
		towedVehicleStickyState[wheelId].setToDefault();
	}
}

} //namespace physx

