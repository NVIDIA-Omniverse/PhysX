// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "vehicle/PxVehicleLimits.h"
#if PX_SUPPORT_OMNI_PVD
#include "OmniPvdWriter.h"
#endif
#include "foundation/PxMemory.h"


#if !PX_DOXYGEN
namespace physx
{
#endif

struct PxVehiclePvdObjectHandles
{
#if PX_SUPPORT_OMNI_PVD

	OmniPvdObjectHandle vehicleOH;

	OmniPvdObjectHandle rigidBodyParamsOH;
	OmniPvdObjectHandle rigidBodyStateOH;

	OmniPvdObjectHandle suspStateCalcParamsOH;

	OmniPvdObjectHandle brakeResponseParamOHs[2];
	OmniPvdObjectHandle steerResponseParamsOH;
	OmniPvdObjectHandle brakeResponseStateOH;
	OmniPvdObjectHandle steerResponseStateOH;
	OmniPvdObjectHandle ackermannParamsOH;

	OmniPvdObjectHandle directDriveCommandStateOH;
	OmniPvdObjectHandle directDriveTransmissionCommandStateOH;
	OmniPvdObjectHandle directDriveThrottleResponseParamsOH;
	OmniPvdObjectHandle directDriveThrottleResponseStateOH;
	OmniPvdObjectHandle directDrivetrainOH;

	OmniPvdObjectHandle engineDriveCommandStateOH;
	OmniPvdObjectHandle engineDriveTransmissionCommandStateOH;
	OmniPvdObjectHandle clutchResponseParamsOH;
	OmniPvdObjectHandle clutchParamsOH;
	OmniPvdObjectHandle engineParamsOH;
	OmniPvdObjectHandle gearboxParamsOH;
	OmniPvdObjectHandle autoboxParamsOH;
	OmniPvdObjectHandle differentialParamsOH;
	OmniPvdObjectHandle clutchResponseStateOH;
	OmniPvdObjectHandle engineDriveThrottleResponseStateOH;
	OmniPvdObjectHandle engineStateOH;
	OmniPvdObjectHandle gearboxStateOH;
	OmniPvdObjectHandle autoboxStateOH;
	OmniPvdObjectHandle diffStateOH;
	OmniPvdObjectHandle clutchSlipStateOH;
	OmniPvdObjectHandle engineDrivetrainOH;

	OmniPvdObjectHandle* wheelAttachmentOHs;
	OmniPvdObjectHandle* wheelParamsOHs;
	OmniPvdObjectHandle* wheelActuationStateOHs;
	OmniPvdObjectHandle* wheelRigidBody1dStateOHs;
	OmniPvdObjectHandle* wheelLocalPoseStateOHs;
	OmniPvdObjectHandle* roadGeomStateOHs;
	OmniPvdObjectHandle* suspParamsOHs;
	OmniPvdObjectHandle* suspCompParamsOHs;
	OmniPvdObjectHandle* suspForceParamsOHs;
	OmniPvdObjectHandle* suspStateOHs;
	OmniPvdObjectHandle* suspCompStateOHs;
	OmniPvdObjectHandle* suspForceOHs;
	OmniPvdObjectHandle* tireParamsOHs;
	OmniPvdObjectHandle* tireDirectionStateOHs;
	OmniPvdObjectHandle* tireSpeedStateOHs;
	OmniPvdObjectHandle* tireSlipStateOHs;
	OmniPvdObjectHandle* tireStickyStateOHs;
	OmniPvdObjectHandle* tireGripStateOHs;
	OmniPvdObjectHandle* tireCamberStateOHs;
	OmniPvdObjectHandle* tireForceOHs;

	OmniPvdObjectHandle* physxWheelAttachmentOHs;
	OmniPvdObjectHandle* physxWheelShapeOHs;
	OmniPvdObjectHandle* physxConstraintParamOHs;
	OmniPvdObjectHandle* physxConstraintStateOHs;
	OmniPvdObjectHandle* physxRoadGeomStateOHs;
	OmniPvdObjectHandle physxSteerStateOH;
	OmniPvdObjectHandle* physxMaterialFrictionSetOHs;
	OmniPvdObjectHandle* physxMaterialFrictionOHs;

	OmniPvdObjectHandle physxRoadGeomQueryParamOH;
	OmniPvdObjectHandle physxRoadGeomQueryDefaultFilterDataOH;
	OmniPvdObjectHandle* physxRoadGeomQueryFilterDataOHs;
	OmniPvdObjectHandle physxRigidActorOH;

	OmniPvdObjectHandle* antiRollParamOHs;
	OmniPvdObjectHandle antiRollTorqueOH;

	PxU32 nbWheels;
	PxU32 nbPhysXMaterialFrictions;	
	PxU32 nbAntirolls;

	OmniPvdContextHandle contextHandle;

#endif //PX_SUPPORT_OMNI_PVD
};

#if !PX_DOXYGEN
} // namespace physx
#endif

