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

#pragma once

#include "vehicle2/PxVehicleLimits.h"
#if PX_SUPPORT_OMNI_PVD
#include "OmniPvdWriter.h"
#endif
#include "foundation/PxMemory.h"


#if !PX_DOXYGEN
namespace physx
{
namespace vehicle2
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
} // namespace vehicle2
} // namespace physx
#endif

