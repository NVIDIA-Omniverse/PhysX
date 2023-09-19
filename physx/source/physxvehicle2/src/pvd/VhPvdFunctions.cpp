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

#include "VhPvdAttributeHandles.h"
#include "VhPvdObjectHandles.h"
#include "VhPvdWriter.h"

#include "vehicle2/pvd/PxVehiclePvdFunctions.h"

#include "foundation/PxAllocatorCallback.h"

#include <stdio.h>
#include <stdlib.h>

namespace physx
{
namespace vehicle2
{

#if PX_SUPPORT_OMNI_PVD

PX_FORCE_INLINE void createPvdObject
(OmniPvdWriter& omniWriter, OmniPvdContextHandle contextHandle,
OmniPvdClassHandle classHandle, OmniPvdObjectHandle objectHandle, const char* objectName)
{
	omniWriter.createObject(contextHandle, classHandle, objectHandle, objectName);
}

PX_FORCE_INLINE void writeObjectHandleAttribute
(OmniPvdWriter& omniWriter, OmniPvdContextHandle ch,  OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, 
 OmniPvdObjectHandle val)
{
	PX_ASSERT(oh);
	PX_ASSERT(ah);
	if(val)
		omniWriter.setAttribute(ch, oh, ah, reinterpret_cast<const uint8_t*>(&val), sizeof(OmniPvdObjectHandle));
}

PX_FORCE_INLINE void addObjectHandleToUniqueList
(OmniPvdWriter& ow, OmniPvdContextHandle ch, OmniPvdObjectHandle setOwnerOH,  OmniPvdAttributeHandle setAH, OmniPvdObjectHandle ohToAdd)
{	
	PX_ASSERT(setOwnerOH);
	PX_ASSERT(setAH);
	if(ohToAdd)
		ow.addToUniqueListAttribute(ch, setOwnerOH, setAH, reinterpret_cast<const uint8_t*>(&ohToAdd), sizeof(OmniPvdObjectHandle));
}

PX_FORCE_INLINE void appendWithInt(char* buffer, PxU32 number)
{	
	char num[8];
	sprintf(num, "%d", number);
	strcat(buffer, num);
}

PX_FORCE_INLINE void createVehicleObject
(const PxVehiclePvdAttributeHandles& attributeHandles,
 PxVehiclePvdObjectHandles& objectHandles, OmniPvdWriter& omniWriter)
{
	// Register the top-level vehicle object if this hasn't already been done.
	if(0 == objectHandles.vehicleOH)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle vehicleOH = reinterpret_cast<OmniPvdObjectHandle>(&objectHandles.vehicleOH);
		createPvdObject(omniWriter, objectHandles.contextHandle, attributeHandles.vehicle.CH, vehicleOH, "Vehicle");
		objectHandles.vehicleOH = vehicleOH;
	}
}


/////////////////////////////////
//RIGID BODY
/////////////////////////////////

void PxVehiclePvdRigidBodyRegister
(const PxVehicleRigidBodyParams* rbodyParams, const PxVehicleRigidBodyState* rbodyState,
 const PxVehiclePvdAttributeHandles& ah,
 PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	// Register the top-level vehicle object if this hasn't already been done.
	createVehicleObject(ah, objHands, ow);

	const OmniPvdContextHandle ch = objHands.contextHandle;

	if(rbodyParams)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.rigidBodyParamsOH);
		createPvdObject(ow, ch, ah.rigidBodyParams.CH, oh, "RigidBodyParams");
		objHands.rigidBodyParamsOH = oh;
		writeObjectHandleAttribute(ow, ch, objHands.vehicleOH, ah.vehicle.rigidBodyParamsAH, oh);
	}

	if(rbodyState)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.rigidBodyStateOH);
		createPvdObject(ow, ch, ah.rigidBodyState.CH, oh, "RigidBodyState");
		objHands.rigidBodyStateOH = oh;
		writeObjectHandleAttribute(ow, ch, objHands.vehicleOH, ah.vehicle.rigidBodyStateAH, oh);
	}
}

void PxVehiclePvdRigidBodyWrite
(const PxVehicleRigidBodyParams* rbodyParams, const PxVehicleRigidBodyState* rbodyState,
 const PxVehiclePvdAttributeHandles& ah,
 const PxVehiclePvdObjectHandles& oh, OmniPvdWriter& ow)
{
	const OmniPvdContextHandle ch = oh.contextHandle;

	if(oh.rigidBodyParamsOH && rbodyParams)
	{
		writeRigidBodyParams(*rbodyParams,  oh.rigidBodyParamsOH, ah.rigidBodyParams, ow, ch);
	}

	if(oh.rigidBodyStateOH && rbodyState)
	{
		writeRigidBodyState(*rbodyState, oh.rigidBodyStateOH, ah.rigidBodyState, ow, ch);
	}
}

//////////////////////////////
//SUSP STATE CALC PARAMS
//////////////////////////////

void PxVehiclePvdSuspensionStateCalculationParamsRegister
(const PxVehicleSuspensionStateCalculationParams* suspStateCalcParams, 
 const PxVehiclePvdAttributeHandles& ah,
 PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	// Register the top-level vehicle object if this hasn't already been done.
	createVehicleObject(ah, objHands, ow);

	const OmniPvdContextHandle ch = objHands.contextHandle;

	if(suspStateCalcParams)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.suspStateCalcParamsOH);
		createPvdObject(ow, ch, ah.suspStateCalcParams.CH, oh, "SuspStateCalcParams");
		objHands.suspStateCalcParamsOH = oh;
		writeObjectHandleAttribute(ow, ch, objHands.vehicleOH, ah.vehicle.suspStateCalcParamsAH, oh);
	}
}

void PxVehiclePvdSuspensionStateCalculationParamsWrite
(const PxVehicleSuspensionStateCalculationParams* suspStateCalcParams, 
 const PxVehiclePvdAttributeHandles& ah,
 const PxVehiclePvdObjectHandles& oh, OmniPvdWriter& omniWriter)
{
	if(oh.suspStateCalcParamsOH && suspStateCalcParams)
	{
		writeSuspStateCalcParams(*suspStateCalcParams, oh.suspStateCalcParamsOH, ah.suspStateCalcParams, omniWriter, oh.contextHandle);
	}
}

/////////////////////////////
//COMMAND RESPONSE PARAMS
/////////////////////////////

void PxVehiclePvdCommandResponseRegister
(const PxVehicleSizedArrayData<const PxVehicleBrakeCommandResponseParams>& brakeResponseParams,
 const PxVehicleSteerCommandResponseParams* steerResponseParams,
 const PxVehicleAckermannParams* ackermannParams,
 const PxVehicleArrayData<PxReal>& brakeResponseStates,
 const PxVehicleArrayData<PxReal>& steerResponseStates,
 const PxVehiclePvdAttributeHandles& ah,
 PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	PX_CHECK_AND_RETURN(
		brakeResponseParams.size <= 2, 
		"PxVehiclePvdCommandResponseRegister : brakeResponseParams.size must have less than or equal to 2");

	// Register the top-level vehicle object if this hasn't already been done.
	createVehicleObject(ah, objHands, ow);

	const OmniPvdContextHandle ch = objHands.contextHandle;

	for(PxU32 i = 0; i < brakeResponseParams.size; i++)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.brakeResponseParamOHs[i]);
		char objectName[32] = "BrakeComandResponseParams";
		appendWithInt(objectName, i);
		createPvdObject(ow, ch, ah.brakeCommandResponseParams.CH, oh, objectName);
		objHands.brakeResponseParamOHs[i] = oh;

		addObjectHandleToUniqueList(ow, ch, objHands.vehicleOH, ah.vehicle.brakeResponseParamsSetAH, oh);
	}

	if(steerResponseParams)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.steerResponseParamsOH);
		const char objectName[32] = "SteerCommandResponseParams";
		createPvdObject(ow, ch, ah.steerCommandResponseParams.CH, oh, objectName);
		objHands.steerResponseParamsOH = oh;
		writeObjectHandleAttribute(ow, ch, objHands.vehicleOH, ah.vehicle.steerResponseParamsAH, oh);
	}

	if (ackermannParams)
	{
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.ackermannParamsOH);
		createPvdObject(ow, ch, ah.ackermannParams.CH, oh, "AckermannParams");
		objHands.ackermannParamsOH = oh;
		writeObjectHandleAttribute(ow, ch, objHands.vehicleOH, ah.vehicle.ackermannParamsAH, oh);
	}

	if(!brakeResponseStates.isEmpty())
	{
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.brakeResponseStateOH);
		const char objectName[32] = "BrakeCommandResponseStates";
		createPvdObject(ow, ch, ah.brakeCommandResponseStates.CH, oh, objectName);
		objHands.brakeResponseStateOH = oh;
		writeObjectHandleAttribute(ow, ch, objHands.vehicleOH, ah.vehicle.brakeResponseStatesAH, oh);
	}

	if(!steerResponseStates.isEmpty())
	{
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.steerResponseStateOH);
		const char objectName[32] = "SteerCommandResponseStates";
		createPvdObject(ow, ch, ah.steerCommandResponseStates.CH, oh, objectName);
		objHands.steerResponseStateOH = oh;
		writeObjectHandleAttribute(ow, ch, objHands.vehicleOH, ah.vehicle.steerResponseStatesAH, oh);
	}
}

void PxVehiclePvdCommandResponseWrite
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleSizedArrayData<const PxVehicleBrakeCommandResponseParams>& brakeResponseParams,
 const PxVehicleSteerCommandResponseParams* steerResponseParams, 
 const PxVehicleAckermannParams* ackermannParams, 
 const PxVehicleArrayData<PxReal>& brakeResponseStates,
 const PxVehicleArrayData<PxReal>& steerResponseStates,
 const PxVehiclePvdAttributeHandles& ah,
 const PxVehiclePvdObjectHandles& oh, OmniPvdWriter& ow)
{
	PX_CHECK_AND_RETURN(
		brakeResponseParams.size <= 2, 
		"PxVehiclePvdCommandResponseWrite : brakeResponseParams.size must have less than or equal to 2");

	const OmniPvdContextHandle ch = oh.contextHandle;

	for(PxU32 i = 0; i < brakeResponseParams.size; i++)
	{
		if(oh.brakeResponseParamOHs[i])
		{
			writeBrakeResponseParams(
				axleDesc, brakeResponseParams[i], 
				oh.brakeResponseParamOHs[i], ah.brakeCommandResponseParams, ow, ch);
		}
	}
	if(oh.steerResponseParamsOH && steerResponseParams)
	{
		writeSteerResponseParams(
			axleDesc, *steerResponseParams,
			oh.steerResponseParamsOH, ah.steerCommandResponseParams, ow, ch);
	}

	if (oh.ackermannParamsOH && ackermannParams)
	{
		writeAckermannParams(*ackermannParams, oh.ackermannParamsOH, ah.ackermannParams, ow, ch);
	}

	if(oh.brakeResponseStateOH && !brakeResponseStates.isEmpty())
	{
		writeBrakeResponseStates(axleDesc, brakeResponseStates, oh.brakeResponseStateOH, ah.brakeCommandResponseStates, ow, ch);
	}

	if(oh.steerResponseStateOH && !steerResponseStates.isEmpty())
	{
		writeSteerResponseStates(axleDesc, steerResponseStates, oh.steerResponseStateOH, ah.steerCommandResponseStates, ow, ch);
	}
}

void PxVehiclePvdWheelAttachmentsRegister
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleArrayData<const PxVehicleWheelParams>& wheelParams,
 const PxVehicleArrayData<const PxVehicleWheelActuationState>& wheelActuationStates,
 const PxVehicleArrayData<const PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
 const PxVehicleArrayData<const PxVehicleWheelLocalPose>& wheelLocalPoses,
 const PxVehicleArrayData<const PxVehicleRoadGeometryState>& roadGeometryStates,
 const PxVehicleArrayData<const PxVehicleSuspensionParams>& suspParams,
 const PxVehicleArrayData<const PxVehicleSuspensionComplianceParams>& suspCompParams,
 const PxVehicleArrayData<const PxVehicleSuspensionForceParams>& suspForceParams,
 const PxVehicleArrayData<const PxVehicleSuspensionState>& suspStates,
 const PxVehicleArrayData<const PxVehicleSuspensionComplianceState>& suspCompStates,
 const PxVehicleArrayData<const PxVehicleSuspensionForce>& suspForces,
 const PxVehicleArrayData<const PxVehicleTireForceParams>& tireForceParams,
 const PxVehicleArrayData<const PxVehicleTireDirectionState>& tireDirectionStates,
 const PxVehicleArrayData<const PxVehicleTireSpeedState>& tireSpeedStates,
 const PxVehicleArrayData<const PxVehicleTireSlipState>& tireSlipStates,
 const PxVehicleArrayData<const PxVehicleTireStickyState>& tireStickyStates,
 const PxVehicleArrayData<const PxVehicleTireGripState>& tireGripStates,
 const PxVehicleArrayData<const PxVehicleTireCamberAngleState>& tireCamberStates,
 const PxVehicleArrayData<const PxVehicleTireForce>& tireForces, 
 const PxVehiclePvdAttributeHandles& ah, 
 PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	// Register the top-level vehicle object if this hasn't already been done.
	createVehicleObject(ah, objHands, ow);

	const OmniPvdContextHandle ch = objHands.contextHandle;
	
	// Register the wheel attachments
	for(PxU32 i = 0; i < axleDesc.nbWheels; i++)
	{
		const PxU32 wheelId = axleDesc.wheelIdsInAxleOrder[i];

		PX_CHECK_AND_RETURN(
			wheelId < objHands.nbWheels,
			"PxVehiclePvdWheelAttachmentsRegister - axleDesc.axleToWheelIds[i] must be less than the value of the nbWheels argument in the function PxVehiclePvdObjectCreate()");
		
		if(!wheelParams.isEmpty())
		{
			//Get a unique id from a memory adress in objectHandles.
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.wheelParamsOHs[wheelId]);
			char objectName[32] = "WheelParams";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.wheelParams.CH, oh, objectName);
			objHands.wheelParamsOHs[wheelId] = oh;
		}

		if(!wheelActuationStates.isEmpty())
		{
			//Get a unique id from a memory adress in objectHandles.
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.wheelActuationStateOHs[wheelId]);
			char objectName[32] = "WheelActuationState";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.wheelActuationState.CH, oh, objectName);
			objHands.wheelActuationStateOHs[wheelId] = oh;
		}

		if(!wheelRigidBody1dStates.isEmpty())
		{
			//Get a unique id from a memory adress in objectHandles.
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.wheelRigidBody1dStateOHs[wheelId]);
			char objectName[32] = "WheelRigidBody1dState";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.wheelRigidBody1dState.CH, oh, objectName);
			objHands.wheelRigidBody1dStateOHs[wheelId] = oh;
		}


		if(!wheelLocalPoses.isEmpty())
		{
			//Get a unique id from a memory adress in objectHandles.
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.wheelLocalPoseStateOHs[wheelId]);
			char objectName[32] = "WheelLocalPoseState";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.wheelLocalPoseState.CH, oh, objectName);
			objHands.wheelLocalPoseStateOHs[wheelId] = oh;
		}

		if(!roadGeometryStates.isEmpty())
		{
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.roadGeomStateOHs[wheelId]);
			char objectName[32] = "RoadGeometryState";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.roadGeomState.CH, oh, objectName);
			objHands.roadGeomStateOHs[wheelId] = oh;
		}

		if(!suspParams.isEmpty())
		{
			//Get a unique id from a memory adress in objectHandles.
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.suspParamsOHs[wheelId]);
			char objectName[32] = "SuspParams";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.suspParams.CH, oh, objectName);
			objHands.suspParamsOHs[wheelId] = oh;
		}

		if(!suspCompParams.isEmpty())
		{
			//Get a unique id from a memory adress in objectHandles.
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.suspCompParamsOHs[wheelId]);
			char objectName[32] = "SuspCompParams";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.suspCompParams.CH, oh, objectName);
			objHands.suspCompParamsOHs[wheelId] = oh;
		}

		if(!suspForceParams.isEmpty())
		{
			//Get a unique id from a memory adress in objectHandles.
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.suspForceParamsOHs[wheelId]);
			char objectName[32] = "SuspForceParams";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.suspForceParams.CH, oh, objectName);
			objHands.suspForceParamsOHs[wheelId] = oh;
		}

		if(!suspStates.isEmpty())
		{
			//Get a unique id from a memory adress in objectHandles.
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.suspStateOHs[wheelId]);
			char objectName[32] = "SuspState";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.suspState.CH, oh, objectName);
			objHands.suspStateOHs[wheelId] = oh;
		}

		if(!suspCompStates.isEmpty())
		{
			//Get a unique id from a memory adress in objectHandles.
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.suspCompStateOHs[wheelId]);
			char objectName[32] = "SuspComplianceState";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.suspCompState.CH, oh, objectName);
			objHands.suspCompStateOHs[wheelId] = oh;
		}

		if(!suspForces.isEmpty())
		{
			//Get a unique id from a memory adress in objectHandles.
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.suspForceOHs[wheelId]);
			char objectName[32] = "SuspForce";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.suspForce.CH, oh, objectName);
			objHands.suspForceOHs[wheelId] = oh;
		}

		if(!tireForceParams.isEmpty())
		{
			//Get a unique id from a memory adress in objectHandles.
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.tireParamsOHs[wheelId]);
			char objectName[32] = "TireParams";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.tireParams.CH, oh, objectName);
			objHands.tireParamsOHs[wheelId] = oh;
		}

		if(!tireDirectionStates.isEmpty())
		{
			//Get a unique id from a memory adress in objectHandles.
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.tireDirectionStateOHs[wheelId]);
			char objectName[32] = "TireDirectionState";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.tireDirectionState.CH, oh, objectName);
			objHands.tireDirectionStateOHs[wheelId] = oh;
		}

		if(!tireSpeedStates.isEmpty())
		{
			//Get a unique id from a memory adress in objectHandles.
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.tireSpeedStateOHs[wheelId]);
			char objectName[32] = "TireSpeedState";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.tireSpeedState.CH, oh, objectName);
			objHands.tireSpeedStateOHs[wheelId] = oh;
		}

		if(!tireSlipStates.isEmpty())
		{
			//Get a unique id from a memory adress in objectHandles.
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.tireSlipStateOHs[wheelId]);
			char objectName[32] = "TireSlipState";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.tireSlipState.CH, oh, objectName);
			objHands.tireSlipStateOHs[wheelId] = oh;
		}

		if(!tireStickyStates.isEmpty())
		{
			//Get a unique id from a memory adress in objectHandles.
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.tireStickyStateOHs[wheelId]);
			char objectName[32] = "TireStickyState";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.tireStickyState.CH, oh, objectName);
			objHands.tireStickyStateOHs[wheelId] = oh;
		}

		if(!tireGripStates.isEmpty())
		{
			//Get a unique id from a memory adress in objectHandles.
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.tireGripStateOHs[wheelId]);
			char objectName[32] = "TireGripState";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.tireGripState.CH, oh, objectName);
			objHands.tireGripStateOHs[wheelId] = oh;
		}

		if(!tireCamberStates.isEmpty())
		{
			//Get a unique id from a memory adress in objectHandles.
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.tireCamberStateOHs[wheelId]);
			char objectName[32] = "TireCamberState";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.tireCamberState.CH, oh, objectName);
			objHands.tireCamberStateOHs[wheelId] = oh;
		}

		if(!tireForces.isEmpty())
		{
			//Get a unique id from a memory adress in objectHandles.
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.tireForceOHs[wheelId]);
			char objectName[32] = "TireForce";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.tireForce.CH, oh, objectName);
			objHands.tireForceOHs[wheelId] = oh;
		}

		{
			//Get a unique id from a memory adress in objectHandles.
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.wheelAttachmentOHs[wheelId]);
			char objectName[32] = "WheelAttachment";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.wheelAttachment.CH, oh, objectName);
			objHands.wheelAttachmentOHs[wheelId] = oh;

			// Point the wheel attachment object at the wheel params, susp state, tire force etc objects.
			writeObjectHandleAttribute(
					ow, ch, oh, ah.wheelAttachment.wheelParamsAH, objHands.wheelParamsOHs[wheelId]);
			writeObjectHandleAttribute(
					ow, ch, oh, ah.wheelAttachment.wheelActuationStateAH, objHands.wheelActuationStateOHs[wheelId]);
			writeObjectHandleAttribute(
					ow, ch, oh, ah.wheelAttachment.wheelRigidBody1dStateAH, objHands.wheelRigidBody1dStateOHs[wheelId]);
			writeObjectHandleAttribute(
					ow, ch, oh, ah.wheelAttachment.wheelLocalPoseStateAH, objHands.wheelLocalPoseStateOHs[wheelId]);
			writeObjectHandleAttribute(
					ow, ch, oh, ah.wheelAttachment.roadGeomStateAH, objHands.roadGeomStateOHs[wheelId]);
			writeObjectHandleAttribute(
					ow, ch, oh, ah.wheelAttachment.suspParamsAH, objHands.suspParamsOHs[wheelId]);
			writeObjectHandleAttribute(
					ow, ch, oh, ah.wheelAttachment.suspCompParamsAH, objHands.suspCompParamsOHs[wheelId]);
			writeObjectHandleAttribute(
					ow, ch, oh, ah.wheelAttachment.suspForceParamsAH, objHands.suspForceParamsOHs[wheelId]);
			writeObjectHandleAttribute(
					ow, ch, oh, ah.wheelAttachment.suspStateAH, objHands.suspStateOHs[wheelId]);
			writeObjectHandleAttribute(
					ow, ch, oh, ah.wheelAttachment.suspCompStateAH, objHands.suspCompStateOHs[wheelId]);
			writeObjectHandleAttribute(
					ow, ch, oh, ah.wheelAttachment.suspForceAH, objHands.suspForceOHs[wheelId]);
			writeObjectHandleAttribute(
					ow, ch, oh, ah.wheelAttachment.tireParamsAH, objHands.tireParamsOHs[wheelId]);
			writeObjectHandleAttribute(
					ow, ch, oh, ah.wheelAttachment.tireDirectionStateAH, objHands.tireDirectionStateOHs[wheelId]);
			writeObjectHandleAttribute(
					ow, ch, oh, ah.wheelAttachment.tireSpeedStateAH, objHands.tireSpeedStateOHs[wheelId]);
			writeObjectHandleAttribute(
					ow, ch, oh, ah.wheelAttachment.tireSlipStateAH, objHands.tireSlipStateOHs[wheelId]);
			writeObjectHandleAttribute(
					ow, ch, oh, ah.wheelAttachment.tireStickyStateAH, objHands.tireStickyStateOHs[wheelId]);
			writeObjectHandleAttribute(
					ow, ch, oh, ah.wheelAttachment.tireGripStateAH, objHands.tireGripStateOHs[wheelId]);
			writeObjectHandleAttribute(
					ow, ch, oh, ah.wheelAttachment.tireCamberStateAH, objHands.tireCamberStateOHs[wheelId]);
			writeObjectHandleAttribute(
					ow, ch, oh, ah.wheelAttachment.tireForceAH, objHands.tireForceOHs[wheelId]);

			//Point the vehicle object at the wheel attachment object.
			addObjectHandleToUniqueList(ow, ch, objHands.vehicleOH, ah.vehicle.wheelAttachmentSetAH, oh);
		}
	}
}

void PxVehiclePvdWheelAttachmentsWrite
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleArrayData<const PxVehicleWheelParams>& wheelParams,
 const PxVehicleArrayData<const PxVehicleWheelActuationState>& wheelActuationStates,
 const PxVehicleArrayData<const PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
 const PxVehicleArrayData<const PxVehicleWheelLocalPose>& wheelLocalPoses,
 const PxVehicleArrayData<const PxVehicleRoadGeometryState>& roadGeometryStates,
 const PxVehicleArrayData<const PxVehicleSuspensionParams>& suspParams,
 const PxVehicleArrayData<const PxVehicleSuspensionComplianceParams>& suspComplianceParams,
 const PxVehicleArrayData<const PxVehicleSuspensionForceParams>& suspForceParams,
 const PxVehicleArrayData<const PxVehicleSuspensionState>& suspStates,
 const PxVehicleArrayData<const PxVehicleSuspensionComplianceState>& suspCompStates,
 const PxVehicleArrayData<const PxVehicleSuspensionForce>& suspForces,
 const PxVehicleArrayData<const PxVehicleTireForceParams>& tireForceParams,
 const PxVehicleArrayData<const PxVehicleTireDirectionState>& tireDirectionStates,
 const PxVehicleArrayData<const PxVehicleTireSpeedState>& tireSpeedStates,
 const PxVehicleArrayData<const PxVehicleTireSlipState>& tireSlipStates,
 const PxVehicleArrayData<const PxVehicleTireStickyState>& tireStickyStates,
 const PxVehicleArrayData<const PxVehicleTireGripState>& tireGripStates,
 const PxVehicleArrayData<const PxVehicleTireCamberAngleState>& tireCamberStates,
 const PxVehicleArrayData<const PxVehicleTireForce>& tireForces, 
 const PxVehiclePvdAttributeHandles& ah,
 const PxVehiclePvdObjectHandles& oh, OmniPvdWriter& ow)
{
	const OmniPvdContextHandle ch = oh.contextHandle;

	for(PxU32 i = 0; i < axleDesc.nbWheels; i++)
	{
		const PxU32 wheelId = axleDesc.wheelIdsInAxleOrder[i];

		PX_CHECK_AND_RETURN(
			wheelId < oh.nbWheels,
			"PxVehiclePvdWheelAttachmentsRegister - axleDesc.axleToWheelIds[i] must be less than the value of the nbWheels argument in the function PxVehiclePvdObjectCreate()");
		
		if(oh.wheelParamsOHs[wheelId] && !wheelParams.isEmpty())
		{
			writeWheelParams(wheelParams[wheelId], oh.wheelParamsOHs[wheelId], ah.wheelParams, ow, ch);
		}

		if(oh.wheelActuationStateOHs[wheelId] && !wheelActuationStates.isEmpty())
		{
			writeWheelActuationState(wheelActuationStates[wheelId], oh.wheelActuationStateOHs[wheelId], ah.wheelActuationState, ow, ch);
		}

		if(oh.wheelRigidBody1dStateOHs[wheelId] && !wheelRigidBody1dStates.isEmpty())
		{
			writeWheelRigidBody1dState(wheelRigidBody1dStates[wheelId], oh.wheelRigidBody1dStateOHs[wheelId], ah.wheelRigidBody1dState, ow, ch);
		}

		if(oh.wheelLocalPoseStateOHs[wheelId] && !wheelLocalPoses.isEmpty())
		{
			writeWheelLocalPoseState(wheelLocalPoses[wheelId], oh.wheelLocalPoseStateOHs[wheelId], ah.wheelLocalPoseState, ow, ch);
		}
			
		if(oh.roadGeomStateOHs[wheelId] && !roadGeometryStates.isEmpty())
		{
			writeRoadGeomState(roadGeometryStates[wheelId], oh.roadGeomStateOHs[wheelId], ah.roadGeomState, ow, ch);
		}

		if(oh.suspParamsOHs[wheelId] && !suspParams.isEmpty())
		{
			writeSuspParams(suspParams[wheelId], oh.suspParamsOHs[wheelId], ah.suspParams, ow, ch);
		}

		if(oh.suspCompParamsOHs[wheelId] && !suspComplianceParams.isEmpty())
		{
			writeSuspComplianceParams(suspComplianceParams[wheelId], oh.suspCompParamsOHs[wheelId], ah.suspCompParams, ow, ch);
		}

		if(oh.suspForceParamsOHs[wheelId] && !suspForceParams.isEmpty())
		{
			writeSuspForceParams(suspForceParams[wheelId], oh.suspForceParamsOHs[wheelId], ah.suspForceParams, ow, ch);
		}
		
		if(oh.suspStateOHs[wheelId] && !suspStates.isEmpty())
		{
			writeSuspState(suspStates[wheelId], oh.suspStateOHs[wheelId], ah.suspState, ow, ch);
		}

		if(oh.suspCompStateOHs[wheelId] && !suspCompStates.isEmpty())
		{
			writeSuspComplianceState(suspCompStates[wheelId], oh.suspCompStateOHs[wheelId], ah.suspCompState, ow, ch);
		}

		if(oh.suspForceOHs[wheelId] && !suspForces.isEmpty())
		{
			writeSuspForce(suspForces[wheelId], oh.suspForceOHs[wheelId], ah.suspForce, ow, ch);
		}

		if(oh.tireParamsOHs[wheelId] && !tireForceParams.isEmpty())
		{
			writeTireParams(tireForceParams[wheelId], oh.tireParamsOHs[wheelId], ah.tireParams, ow, ch);
		}

		if(oh.tireDirectionStateOHs[wheelId] && !tireDirectionStates.isEmpty())
		{
			writeTireDirectionState(tireDirectionStates[wheelId], oh.tireDirectionStateOHs[wheelId], ah.tireDirectionState, ow, ch);
		}

		if(oh.tireSpeedStateOHs[wheelId] && !tireSpeedStates.isEmpty())
		{
			writeTireSpeedState(tireSpeedStates[wheelId], oh.tireSpeedStateOHs[wheelId], ah.tireSpeedState, ow, ch);
		}

		if(oh.tireSlipStateOHs[wheelId] && !tireSlipStates.isEmpty())
		{
			writeTireSlipState(tireSlipStates[wheelId], oh.tireSlipStateOHs[wheelId], ah.tireSlipState, ow, ch);
		}

		if(oh.tireStickyStateOHs[wheelId] && !tireStickyStates.isEmpty())
		{
			writeTireStickyState(tireStickyStates[wheelId], oh.tireStickyStateOHs[wheelId], ah.tireStickyState, ow, ch);
		}

		if(oh.tireGripStateOHs[wheelId] && !tireGripStates.isEmpty())
		{
			writeTireGripState(tireGripStates[wheelId], oh.tireGripStateOHs[wheelId], ah.tireGripState, ow, ch);
		}

		if(oh.tireCamberStateOHs[wheelId] && !tireCamberStates.isEmpty())
		{
			writeTireCamberState(tireCamberStates[wheelId], oh.tireCamberStateOHs[wheelId], ah.tireCamberState, ow, ch);
		}

		if(oh.tireForceOHs[wheelId] && !tireForces.isEmpty())
		{
			writeTireForce(tireForces[wheelId], oh.tireForceOHs[wheelId], ah.tireForce, ow, ch);
		}
	}
}
	
void PxVehiclePvdDirectDrivetrainRegister
(const PxVehicleCommandState* commandState, const PxVehicleDirectDriveTransmissionCommandState* transmissionState,
 const PxVehicleDirectDriveThrottleCommandResponseParams* directDriveThrottleResponseParams,
 const PxVehicleArrayData<PxReal>& directDriveThrottleResponseState,
 const PxVehiclePvdAttributeHandles& ah,
 PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	// Register the top-level vehicle object if this hasn't already been done.
	createVehicleObject(ah, objHands, ow);

	const OmniPvdContextHandle ch = objHands.contextHandle;

	if(commandState)
	{
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.directDriveCommandStateOH);
		createPvdObject(ow, ch, ah.directDriveCommandState.CH, oh, "DirectDriveCommandState");
		objHands.directDriveCommandStateOH = oh;
	}

	if(transmissionState)
	{
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.directDriveTransmissionCommandStateOH);
		createPvdObject(ow, ch, ah.directDriveTransmissionCommandState.CH, oh, "DirectDriveTransmissionCommandState");
		objHands.directDriveTransmissionCommandStateOH = oh;
	}

	if(directDriveThrottleResponseParams)
	{
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.directDriveThrottleResponseParamsOH);
		createPvdObject(ow, ch, ah.directDriveThrottleCommandResponseParams.CH, oh, "DirectDriveThrottleResponseParams");
		objHands.directDriveThrottleResponseParamsOH = oh;
	}

	if(!directDriveThrottleResponseState.isEmpty())
	{
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.directDriveThrottleResponseStateOH);
		createPvdObject(ow, ch, ah.directDriveThrottleCommandResponseState.CH, oh, "DirectDriveThrottleResponseState");
		objHands.directDriveThrottleResponseStateOH = oh;
	}

	{
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.directDrivetrainOH);
		createPvdObject(ow, ch, ah.directDrivetrain.CH, oh, "DirectDrivetrain");
		objHands.directDrivetrainOH = oh;

		writeObjectHandleAttribute(
			ow, ch, oh, ah.directDrivetrain.commandStateAH, objHands.directDriveCommandStateOH);
		writeObjectHandleAttribute(
			ow, ch, oh, ah.directDrivetrain.transmissionCommandStateAH, objHands.directDriveTransmissionCommandStateOH);
		writeObjectHandleAttribute(
			ow, ch, oh, ah.directDrivetrain.throttleResponseParamsAH, objHands.directDriveThrottleResponseParamsOH);
		writeObjectHandleAttribute(
			ow, ch, oh, ah.directDrivetrain.throttleResponseStateAH, objHands.directDriveThrottleResponseStateOH);

		writeObjectHandleAttribute(ow, ch, objHands.vehicleOH, ah.vehicle.directDrivetrainAH, oh);
	}
}

void PxVehiclePvdDirectDrivetrainWrite
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleCommandState* commandState, const PxVehicleDirectDriveTransmissionCommandState* transmissionState,
 const PxVehicleDirectDriveThrottleCommandResponseParams* directDriveThrottleResponseParams,
 const  PxVehicleArrayData<PxReal>& directDriveThrottleResponseState,
 const PxVehiclePvdAttributeHandles& ah,
 const PxVehiclePvdObjectHandles& oh, OmniPvdWriter& ow)
{
	const OmniPvdContextHandle ch = oh.contextHandle;

	if(oh.directDriveCommandStateOH && commandState)
	{
		writeDirectDriveCommandState(*commandState, oh.directDriveCommandStateOH, ah.directDriveCommandState, ow, ch);
	}

	if(oh.directDriveTransmissionCommandStateOH && transmissionState)
	{
		writeDirectDriveTransmissionCommandState(*transmissionState, oh.directDriveTransmissionCommandStateOH, ah.directDriveTransmissionCommandState, ow, ch);
	}
		
	if(oh.directDriveThrottleResponseParamsOH)
	{
		writeDirectDriveThrottleResponseParams(axleDesc, *directDriveThrottleResponseParams, oh.directDriveThrottleResponseParamsOH, ah.directDriveThrottleCommandResponseParams, ow, ch);	
	}

	if(oh.directDriveThrottleResponseStateOH && !directDriveThrottleResponseState.isEmpty())
	{
		writeDirectDriveThrottleResponseState(axleDesc, directDriveThrottleResponseState, oh.directDriveThrottleResponseStateOH, ah.directDriveThrottleCommandResponseState, ow, ch);
	}
}

void PxVehiclePvdEngineDrivetrainRegister
(const PxVehicleCommandState* commandState,
 const PxVehicleEngineDriveTransmissionCommandState* engineDriveTransmissionCommandState,
 const PxVehicleTankDriveTransmissionCommandState* tankDriveTransmissionCommandState,
 const PxVehicleClutchCommandResponseParams* clutchResponseParams,
 const PxVehicleClutchParams* clutchParms,
 const PxVehicleEngineParams* engineParams,
 const PxVehicleGearboxParams* gearboxParams,
 const PxVehicleAutoboxParams* autoboxParams,
 const PxVehicleMultiWheelDriveDifferentialParams* multiWheelDiffParams,
 const PxVehicleFourWheelDriveDifferentialParams* fourWheelDiffParams,
 const PxVehicleTankDriveDifferentialParams* tankDiffParams,
 const PxVehicleClutchCommandResponseState* clutchResponseState,
 const PxVehicleEngineDriveThrottleCommandResponseState* throttleResponseState,
 const PxVehicleEngineState* engineState,
 const PxVehicleGearboxState* gearboxState,
 const PxVehicleAutoboxState* autoboxState,
 const PxVehicleDifferentialState* diffState,
 const PxVehicleClutchSlipState* clutchSlipState,
 const PxVehiclePvdAttributeHandles& ah,
 PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	// Register the top-level vehicle object if this hasn't already been done.
	createVehicleObject(ah, objHands, ow);

	const OmniPvdContextHandle ch = objHands.contextHandle;

	if(commandState)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.engineDriveCommandStateOH);
		createPvdObject(ow, ch, ah.engineDriveCommandState.CH, oh, "EngineDriveCommandState");
		objHands.engineDriveCommandStateOH = oh;
	}

	if(engineDriveTransmissionCommandState || tankDriveTransmissionCommandState)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.engineDriveTransmissionCommandStateOH);
		objHands.engineDriveTransmissionCommandStateOH = oh;

		if (engineDriveTransmissionCommandState)
		{
			createPvdObject(ow, ch, ah.engineDriveTransmissionCommandState.CH, oh, "EngineDriveTransmissionCommandState");
		}
		else
		{
			PX_ASSERT(tankDriveTransmissionCommandState);
			createPvdObject(ow, ch, ah.tankDriveTransmissionCommandState.CH, oh, "TankDriveTransmissionCommandState");
		}
	}

	if(clutchResponseParams)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.clutchResponseParamsOH);
		createPvdObject(ow, ch, ah.clutchCommandResponseParams.CH, oh, "ClutchResponseParams");	
		objHands.clutchResponseParamsOH = oh;
	}

	if(clutchParms)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.clutchParamsOH);
		createPvdObject(ow, ch, ah.clutchParams.CH, oh, "ClutchParams");
		objHands.clutchParamsOH = oh;
	}

	if(engineParams)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.engineParamsOH);
		createPvdObject(ow, ch, ah.engineParams.CH, oh, "EngineParams");
		objHands.engineParamsOH = oh;
	}

	if(gearboxParams)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.gearboxParamsOH);
		createPvdObject(ow, ch, ah.gearboxParams.CH, oh, "GearboxParams");
		objHands.gearboxParamsOH = oh;
	}

	if(autoboxParams)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.autoboxParamsOH);
		createPvdObject(ow, objHands.contextHandle, ah.autoboxParams.CH, oh, "AutoboxParams");
		objHands.autoboxParamsOH = oh;
	}

	if(multiWheelDiffParams || fourWheelDiffParams || tankDiffParams)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.differentialParamsOH);
		objHands.differentialParamsOH = oh;

		if(multiWheelDiffParams)
		{
			createPvdObject(ow, ch, ah.multiwheelDiffParams.CH, oh, "MultiWheelDiffParams");
		}
		else if(fourWheelDiffParams)
		{
			createPvdObject(ow, ch, ah.fourwheelDiffParams.CH, oh, "FourWheelDiffParams");
		}
		else
		{
			PX_ASSERT(tankDiffParams);
			createPvdObject(ow, ch, ah.tankDiffParams.CH, oh, "TankDiffParams");
		}
	}

	if(clutchResponseState)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.clutchResponseStateOH);
		createPvdObject(ow, ch, ah.clutchResponseState.CH, oh, "ClutchResponseState");
		objHands.clutchResponseStateOH = oh;
	}

	if(throttleResponseState)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.engineDriveThrottleResponseStateOH);
		createPvdObject(ow, ch, ah.throttleResponseState.CH, oh, "ThrottleResponseState");
		objHands.engineDriveThrottleResponseStateOH = oh;
	}

	if(engineState)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.engineStateOH);
		createPvdObject(ow, ch, ah.engineState.CH, oh, "EngineState");
		objHands.engineStateOH = oh;
	}

	if(gearboxState)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.gearboxStateOH);
		createPvdObject(ow, ch, ah.gearboxState.CH, oh, "GearboxState");
		objHands.gearboxStateOH = oh;
	}

	if(autoboxState)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.autoboxStateOH);
		createPvdObject(ow, ch, ah.autoboxState.CH, oh, "AutoboxState");
		objHands.autoboxStateOH = oh;
	}

	if(diffState)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.diffStateOH);
		createPvdObject(ow, ch, ah.diffState.CH, oh, "DiffState");
		objHands.diffStateOH = oh;
	}

	if(clutchSlipState)
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.clutchSlipStateOH);
		createPvdObject(ow, ch, ah.clutchSlipState.CH, oh, "ClutchSlipState");
		objHands.clutchSlipStateOH = oh;
	}

	//Engine drivetrain
	{
		//Get a unique id from a memory address in objectHandles.
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.engineDrivetrainOH);
		createPvdObject(ow, ch, ah.engineDrivetrain.CH, oh, "EngineDrivetrain");
		objHands.engineDrivetrainOH = oh;

		writeObjectHandleAttribute(ow, ch, oh, ah.engineDrivetrain.commandStateAH, objHands.engineDriveCommandStateOH);
		writeObjectHandleAttribute(ow, ch, oh, ah.engineDrivetrain.transmissionCommandStateAH, objHands.engineDriveTransmissionCommandStateOH);
		writeObjectHandleAttribute(ow, ch, oh, ah.engineDrivetrain.clutchResponseParamsAH, objHands.clutchResponseParamsOH);
		writeObjectHandleAttribute(ow, ch, oh, ah.engineDrivetrain.clutchParamsAH, objHands.clutchParamsOH);
		writeObjectHandleAttribute(ow, ch, oh, ah.engineDrivetrain.engineParamsAH, objHands.engineParamsOH);
		writeObjectHandleAttribute(ow, ch, oh, ah.engineDrivetrain.gearboxParamsAH, objHands.gearboxParamsOH);
		writeObjectHandleAttribute(ow, ch, oh, ah.engineDrivetrain.autoboxParamsAH, objHands.autoboxParamsOH);
		writeObjectHandleAttribute(ow, ch, oh, ah.engineDrivetrain.differentialParamsAH, objHands.differentialParamsOH);
		writeObjectHandleAttribute(ow, ch, oh, ah.engineDrivetrain.clutchResponseStateAH, objHands.clutchResponseStateOH);
		writeObjectHandleAttribute(ow, ch, oh, ah.engineDrivetrain.throttleResponseStateAH, objHands.engineDriveThrottleResponseStateOH);
		writeObjectHandleAttribute(ow, ch, oh, ah.engineDrivetrain.engineStateAH, objHands.engineStateOH);
		writeObjectHandleAttribute(ow, ch, oh, ah.engineDrivetrain.gearboxStateAH, objHands.gearboxStateOH);
		writeObjectHandleAttribute(ow, ch, oh, ah.engineDrivetrain.autoboxStateAH, objHands.autoboxStateOH);
		writeObjectHandleAttribute(ow, ch, oh, ah.engineDrivetrain.diffStateAH, objHands.diffStateOH);
		writeObjectHandleAttribute(ow, ch, oh, ah.engineDrivetrain.clutchSlipStateAH, objHands.clutchSlipStateOH);

		writeObjectHandleAttribute(ow, ch, objHands.vehicleOH, ah.vehicle.engineDriveTrainAH, oh);
	}
}

void PxVehiclePvdEngineDrivetrainWrite
(const PxVehicleCommandState* commandState,
 const PxVehicleEngineDriveTransmissionCommandState* engineDriveTransmissionCommandState,
 const PxVehicleTankDriveTransmissionCommandState* tankDriveTransmissionCommandState,
 const PxVehicleClutchCommandResponseParams* clutchResponseParams,
 const PxVehicleClutchParams* clutchParms,
 const PxVehicleEngineParams* engineParams,
 const PxVehicleGearboxParams* gearboxParams,
 const PxVehicleAutoboxParams* autoboxParams,
 const PxVehicleMultiWheelDriveDifferentialParams* multiWheelDiffParams,
 const PxVehicleFourWheelDriveDifferentialParams* fourWheelDiffParams,
 const PxVehicleTankDriveDifferentialParams* tankDiffParams,
 const PxVehicleClutchCommandResponseState* clutchResponseState,
 const PxVehicleEngineDriveThrottleCommandResponseState* throttleResponseState,
 const PxVehicleEngineState* engineState,
 const PxVehicleGearboxState* gearboxState,
 const PxVehicleAutoboxState* autoboxState,
 const PxVehicleDifferentialState* diffState,
 const PxVehicleClutchSlipState* clutchSlipState,
 const PxVehiclePvdAttributeHandles& ah,
 const PxVehiclePvdObjectHandles& oh, OmniPvdWriter& omniWriter)
{
	const OmniPvdContextHandle ch = oh.contextHandle;

	if(oh.engineDriveCommandStateOH && commandState)
	{
		writeEngineDriveCommandState(*commandState, oh.engineDriveCommandStateOH, ah.engineDriveCommandState, omniWriter, ch);
	}

	if(oh.engineDriveTransmissionCommandStateOH)
	{
		if (engineDriveTransmissionCommandState)
		{
			writeEngineDriveTransmissionCommandState(*engineDriveTransmissionCommandState, 
				oh.engineDriveTransmissionCommandStateOH, ah.engineDriveTransmissionCommandState, omniWriter, ch);
		}
		else if (tankDriveTransmissionCommandState)
		{
			writeTankDriveTransmissionCommandState(*tankDriveTransmissionCommandState, oh.engineDriveTransmissionCommandStateOH,
				ah.engineDriveTransmissionCommandState, ah.tankDriveTransmissionCommandState, omniWriter, ch);
		}
	}
	
	if(oh.clutchResponseParamsOH && clutchResponseParams)
	{
		writeClutchResponseParams(*clutchResponseParams, oh.clutchResponseParamsOH, ah.clutchCommandResponseParams, omniWriter, ch);
	}

	if(oh.clutchParamsOH && clutchParms)
	{
		writeClutchParams(*clutchParms, oh.clutchParamsOH, ah.clutchParams, omniWriter, ch);
	}

	if(oh.engineParamsOH && engineParams)
	{
		writeEngineParams(*engineParams, oh.engineParamsOH, ah.engineParams, omniWriter, ch);
	}

	if(oh.gearboxParamsOH && gearboxParams)
	{
		writeGearboxParams(*gearboxParams, oh.gearboxParamsOH, ah.gearboxParams, omniWriter, ch);
	}

	if(oh.autoboxParamsOH && autoboxParams)
	{
		writeAutoboxParams(*autoboxParams, oh.autoboxParamsOH, ah.autoboxParams, omniWriter, ch);
	}

	if(oh.differentialParamsOH)
	{
		if (multiWheelDiffParams)
		{
			writeMultiWheelDiffParams(*multiWheelDiffParams, oh.differentialParamsOH,
				ah.multiwheelDiffParams, omniWriter, ch);
		}
		else if (fourWheelDiffParams)
		{
			writeFourWheelDiffParams(*fourWheelDiffParams, oh.differentialParamsOH,
				ah.multiwheelDiffParams, ah.fourwheelDiffParams, omniWriter, ch);
		}
		else if (tankDiffParams)
		{
			writeTankDiffParams(*tankDiffParams, oh.differentialParamsOH, 
				ah.multiwheelDiffParams, ah.tankDiffParams, omniWriter, ch);
		}
	}

	if(oh.clutchResponseStateOH && clutchResponseState)
	{
		writeClutchResponseState(*clutchResponseState, oh.clutchResponseStateOH, ah.clutchResponseState, omniWriter, ch);
	}

	if(oh.engineDriveThrottleResponseStateOH && throttleResponseState)
	{
		writeThrottleResponseState(*throttleResponseState, oh.engineDriveThrottleResponseStateOH, ah.throttleResponseState, omniWriter, ch);
	}

	if(oh.engineStateOH && engineState)
	{
		writeEngineState(*engineState, oh.engineStateOH, ah.engineState, omniWriter, ch);
	}

	if(oh.gearboxStateOH && gearboxState)
	{
		writeGearboxState(*gearboxState, oh.gearboxStateOH, ah.gearboxState, omniWriter, ch);
	}

	if(oh.autoboxStateOH && autoboxState)
	{
		writeAutoboxState(*autoboxState, oh.autoboxStateOH, ah.autoboxState, omniWriter, ch);
	}

	if(oh.diffStateOH && diffState)
	{
		writeDiffState(*diffState, oh.diffStateOH, ah.diffState, omniWriter, ch);
	}

	if(oh.clutchSlipStateOH && clutchSlipState)
	{
		writeClutchSlipState(*clutchSlipState, oh.clutchSlipStateOH, ah.clutchSlipState, omniWriter, ch);
	}
}

void PxVehiclePvdAntiRollsRegister
(const PxVehicleSizedArrayData<const PxVehicleAntiRollForceParams>& antiRollForceParams,
 const PxVehicleAntiRollTorque* antiRollTorque,
const PxVehiclePvdAttributeHandles& ah, 
 PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	PX_CHECK_AND_RETURN(
		antiRollForceParams.size <= objHands.nbAntirolls,
		"PxVehiclePvdAntiRollsRegister - antiRollForceParams.size  must be less than or equal to vallue of nbAntirolls argument in the function PxVehiclePvdObjectCreate");

	// Register the top-level vehicle object if this hasn't already been done.
	createVehicleObject(ah, objHands, ow);

	const OmniPvdContextHandle ch = objHands.contextHandle;

	// Register the antiroll params.
	for(PxU32 i = 0; i < antiRollForceParams.size; i++)
	{
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.antiRollParamOHs[i]);
		char objectName[32] = "AntiRollParams";
		appendWithInt(objectName, i);
		createPvdObject(ow, ch, ah.antiRollParams.CH, oh, objectName);
		objHands.antiRollParamOHs[i] = oh;
		addObjectHandleToUniqueList(ow, ch, objHands.vehicleOH, ah.vehicle.antiRollSetAH, oh);
	}

	// Register the antiroll force.
	if(antiRollTorque)
	{
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.antiRollTorqueOH);
		const char objectName[32] = "AntiRollTorque";
		createPvdObject(ow, ch, ah.antiRollForce.CH, oh, objectName);
		objHands.antiRollTorqueOH = oh;
		writeObjectHandleAttribute(ow, ch, objHands.vehicleOH, ah.vehicle.antiRollForceAH, oh);
	}
}

void PxVehiclePvdAntiRollsWrite
(const PxVehicleSizedArrayData<const PxVehicleAntiRollForceParams>& antiRollForceParams,
 const PxVehicleAntiRollTorque* antiRollTorque,
 const PxVehiclePvdAttributeHandles& ah, 
 const PxVehiclePvdObjectHandles& oh, OmniPvdWriter& ow)
{
	PX_CHECK_AND_RETURN(
		antiRollForceParams.size <= oh.nbAntirolls,
		"PxVehiclePvdAntiRollsWrite - antiRollForceParams.size  must be less than or equal to vallue of nbAntirolls argument in the function PxVehiclePvdObjectCreate");

	const OmniPvdContextHandle ch = oh.contextHandle;

	for(PxU32 i = 0; i < antiRollForceParams.size; i++)
	{
		if(oh.antiRollParamOHs[i] && !antiRollForceParams.isEmpty())
		{
			writeAntiRollParams(antiRollForceParams[i], oh.antiRollParamOHs[i], ah.antiRollParams, ow, ch);
		}
	}

	if(oh.antiRollTorqueOH && antiRollTorque)
	{
		writeAntiRollForce(*antiRollTorque, oh.antiRollTorqueOH, ah.antiRollForce, ow, ch);
	}
}

void PxVehiclePvdPhysXWheelAttachmentRegister
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleArrayData<const PxVehiclePhysXSuspensionLimitConstraintParams>& physxSuspLimitConstraintParams,
 const PxVehicleArrayData<const PxVehiclePhysXMaterialFrictionParams>& physxMaterialFrictionParams,
 const PxVehiclePhysXActor* physxActor, const PxVehiclePhysXRoadGeometryQueryParams* physxRoadGeomQryParams,
 const PxVehicleArrayData<const PxVehiclePhysXRoadGeometryQueryState>&  physxRoadGeomState,
 const PxVehicleArrayData<const PxVehiclePhysXConstraintState>& physxConstraintStates,
 const PxVehiclePvdAttributeHandles& ah,
 PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	PX_UNUSED(physxMaterialFrictionParams);
	PX_UNUSED(physxRoadGeomState);
	PX_UNUSED(physxConstraintStates);

	// Register the top-level vehicle object if this hasn't already been done.
	createVehicleObject(ah, objHands, ow);

	const OmniPvdContextHandle ch = objHands.contextHandle;

	// Register the wheel attachments
	for(PxU32 i = 0; i < axleDesc.nbWheels; i++)
	{
		const PxU32 wheelId = axleDesc.wheelIdsInAxleOrder[i];

		PX_CHECK_AND_RETURN(
			wheelId < objHands.nbWheels,
			"PxVehiclePvdPhysXWheelAttachmentRegister - axleDesc.axleToWheelIds[i] must be less than the value of the nbWheels argument in the function PxVehiclePvdObjectCreate()");

		if(!physxSuspLimitConstraintParams.isEmpty())
		{
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.physxConstraintParamOHs[wheelId]);
			char objectName[32] = "PhysXSuspLimtConstraintParams";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.physxSuspLimitConstraintParams.CH, oh, objectName);
			objHands.physxConstraintParamOHs[wheelId] = oh;
		}

		if(physxActor && physxActor->wheelShapes[wheelId])
		{
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.physxWheelShapeOHs[wheelId]);
			char objectName[32] = "PhysXWheelShape";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.physxWheelShape.CH, oh, objectName);
			objHands.physxWheelShapeOHs[wheelId] = oh;
		}

		if(!physxConstraintStates.isEmpty())
		{
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.physxConstraintStateOHs[wheelId]);
			char objectName[32] = "PhysXConstraintState";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.physxConstraintState.CH, oh, objectName);
			objHands.physxConstraintStateOHs[wheelId] = oh;

		}

		if(!physxRoadGeomState.isEmpty())
		{
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.physxRoadGeomStateOHs[wheelId]);
			char objectName[32] = "PhysXRoadGeomState";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.physxRoadGeomState.CH, oh, objectName);
			objHands.physxRoadGeomStateOHs[wheelId] = oh;
		}

		{
			const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.physxWheelAttachmentOHs[wheelId]);
			char objectName[32] = "PhysxWheelAttachment";
			appendWithInt(objectName, wheelId);
			createPvdObject(ow, ch, ah.physxWheelAttachment.CH, oh, objectName);
			objHands.physxWheelAttachmentOHs[wheelId] = oh;

			writeObjectHandleAttribute(ow, ch, oh, ah.physxWheelAttachment.physxConstraintParamsAH, objHands.physxConstraintParamOHs[wheelId]);
			writeObjectHandleAttribute(ow, ch, oh, ah.physxWheelAttachment.physxWeelShapeAH, objHands.physxWheelShapeOHs[wheelId]);
			writeObjectHandleAttribute(ow, ch, oh, ah.physxWheelAttachment.physxRoadGeometryStateAH, objHands.physxRoadGeomStateOHs[wheelId]);
			writeObjectHandleAttribute(ow, ch, oh, ah.physxWheelAttachment.physxConstraintStateAH, objHands.physxConstraintStateOHs[wheelId]);

			addObjectHandleToUniqueList(ow, ch, objHands.vehicleOH, ah.vehicle.physxWheelAttachmentSetAH, oh);
		}

		if(!physxMaterialFrictionParams.isEmpty())
		{
			for(PxU32 j = 0; j < objHands.nbPhysXMaterialFrictions; j++)
			{
				const PxU32 id = wheelId*objHands.nbPhysXMaterialFrictions + j;
				const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.physxMaterialFrictionOHs[id]);
				char objectName[32] = "PhysxMaterialFriction";
				appendWithInt(objectName, wheelId);
				strcat(objectName, "_");
 				appendWithInt(objectName, j);
				createPvdObject(ow, ch, ah.physxMaterialFriction.CH, oh, objectName);
				objHands.physxMaterialFrictionOHs[id] = oh;
				addObjectHandleToUniqueList(ow, ch, objHands.physxWheelAttachmentOHs[wheelId], ah.physxWheelAttachment.physxMaterialFrictionSetAH, oh);
			}
		}
	}

	if(physxRoadGeomQryParams)
	{
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.physxRoadGeomQueryParamOH);
		char objectName[32] = "PhysxRoadGeomQryParams";
		createPvdObject(ow, ch, ah.physxRoadGeometryQueryParams.CH, oh, objectName);
		objHands.physxRoadGeomQueryParamOH = oh;
		writeObjectHandleAttribute(ow, ch, objHands.vehicleOH, ah.vehicle.physxRoadGeometryQueryParamsAH, oh);

		const OmniPvdObjectHandle defaultFilterDataOH = reinterpret_cast<OmniPvdObjectHandle>(&objHands.physxRoadGeomQueryDefaultFilterDataOH);
		createPvdObject(ow, ch, ah.physxRoadGeometryQueryParams.filterDataParams.CH, defaultFilterDataOH, "");
		objHands.physxRoadGeomQueryDefaultFilterDataOH = defaultFilterDataOH;
		writeObjectHandleAttribute(ow, ch, objHands.physxRoadGeomQueryParamOH, ah.physxRoadGeometryQueryParams.defaultFilterDataAH, defaultFilterDataOH);

		if (physxRoadGeomQryParams->filterDataEntries)
		{
			for (PxU32 j = 0; j < axleDesc.nbWheels; j++)
			{
				const PxU32 wheelId = axleDesc.wheelIdsInAxleOrder[j];

				const OmniPvdObjectHandle filterDataOH = reinterpret_cast<OmniPvdObjectHandle>(&objHands.physxRoadGeomQueryFilterDataOHs[wheelId]);
				char filterDataObjectName[32] = "FilterData";
				appendWithInt(filterDataObjectName, wheelId);
				createPvdObject(ow, ch, ah.physxRoadGeometryQueryParams.filterDataParams.CH, filterDataOH, filterDataObjectName);
				objHands.physxRoadGeomQueryFilterDataOHs[wheelId] = filterDataOH;

				addObjectHandleToUniqueList(ow, ch, objHands.physxRoadGeomQueryParamOH, ah.physxRoadGeometryQueryParams.filterDataSetAH, filterDataOH);
			}
		}
#if PX_DEBUG
		else
		{
			for (PxU32 j = 0; j < axleDesc.nbWheels; j++)
			{
				// note: objHands.physxRoadGeomQueryFilterDataOHs entries are zero initialized
				//       which matches the invalid handle for now.

				PX_ASSERT(objHands.physxRoadGeomQueryFilterDataOHs[j] == 0);
				// TODO: test against invalid hanndle once it gets introduced
			}
		}
#endif
	}
}

void PxVehiclePvdPhysXWheelAttachmentWrite
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleArrayData<const PxVehiclePhysXSuspensionLimitConstraintParams>& physxSuspLimitConstraintParams,
 const PxVehicleArrayData<const PxVehiclePhysXMaterialFrictionParams>& physxMaterialFrictionParams,
 const PxVehiclePhysXActor* physxActor, const PxVehiclePhysXRoadGeometryQueryParams* physxRoadGeomQryParams,
 const PxVehicleArrayData<const PxVehiclePhysXRoadGeometryQueryState>& physxRoadGeomStates,
 const PxVehicleArrayData<const PxVehiclePhysXConstraintState>& physxConstraintStates,
 const PxVehiclePvdAttributeHandles& ah,
 const PxVehiclePvdObjectHandles& oh, OmniPvdWriter& ow)
{
	PX_UNUSED(physxMaterialFrictionParams);
	PX_UNUSED(physxRoadGeomStates);
	PX_UNUSED(physxConstraintStates);

	const OmniPvdContextHandle ch = oh.contextHandle;

	for(PxU32 i = 0; i < axleDesc.nbWheels; i++)
	{
		const PxU32 wheelId = axleDesc.wheelIdsInAxleOrder[i];

		PX_CHECK_AND_RETURN(
			wheelId < oh.nbWheels,
			"PxVehiclePvdPhysXWheelAttachmentRegister - axleDesc.axleToWheelIds[i] must be less than the value of the nbWheels argument in the function PxVehiclePvdObjectCreate()");
		
		if(oh.physxConstraintParamOHs[wheelId] && !physxSuspLimitConstraintParams.isEmpty())
		{
			writePhysXSuspLimitConstraintParams(physxSuspLimitConstraintParams[wheelId], oh.physxConstraintParamOHs[wheelId], ah.physxSuspLimitConstraintParams, ow, ch);
		}

		if(oh.physxWheelShapeOHs[wheelId] && physxActor)
		{
			writePhysXWheelShape(physxActor->wheelShapes[wheelId], oh.physxWheelShapeOHs[wheelId], ah.physxWheelShape, ow, ch);
		}

		if(oh.physxRoadGeomStateOHs[wheelId] && !physxRoadGeomStates.isEmpty())
		{
			writePhysXRoadGeomState(physxRoadGeomStates[wheelId], oh.physxRoadGeomStateOHs[wheelId], ah.physxRoadGeomState, ow, ch);
		}

		if(oh.physxConstraintStateOHs[wheelId] && !physxConstraintStates.isEmpty())
		{
			writePhysXConstraintState(physxConstraintStates[wheelId], oh.physxConstraintStateOHs[wheelId], ah.physxConstraintState, ow, ch);
		}

		if(!physxMaterialFrictionParams.isEmpty())
		{
			for(PxU32 j = 0; j < physxMaterialFrictionParams[wheelId].nbMaterialFrictions; j++)
			{
				const PxU32 id = wheelId*oh.nbPhysXMaterialFrictions + j;
				if(oh.physxMaterialFrictionOHs[id])
				{
					const PxVehiclePhysXMaterialFriction& m = physxMaterialFrictionParams[wheelId].materialFrictions[j];
					writePhysXMaterialFriction(m, oh.physxMaterialFrictionOHs[id], ah.physxMaterialFriction, ow, ch);
				}
			}
			for(PxU32 j = physxMaterialFrictionParams[wheelId].nbMaterialFrictions; j < oh.nbPhysXMaterialFrictions; j++)
			{
				const PxU32 id = wheelId*oh.nbPhysXMaterialFrictions + j;
				if(oh.physxMaterialFrictionOHs[id])
				{
					PxVehiclePhysXMaterialFriction m;
					m.friction = -1.0f;
					m.material = NULL;
					writePhysXMaterialFriction(m, oh.physxMaterialFrictionOHs[id], ah.physxMaterialFriction, ow, ch);
				}
			}
		}
	}

	if(oh.physxRoadGeomQueryParamOH  && physxRoadGeomQryParams)
	{
		writePhysXRoadGeometryQueryParams(*physxRoadGeomQryParams, axleDesc,
			oh.physxRoadGeomQueryParamOH, oh.physxRoadGeomQueryDefaultFilterDataOH, oh.physxRoadGeomQueryFilterDataOHs,
			ah.physxRoadGeometryQueryParams, ow, ch);
	}
}

void PxVehiclePvdPhysXRigidActorRegister
(const PxVehiclePhysXActor* physxActor,
 const PxVehiclePvdAttributeHandles& ah,
 PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	// Register the top-level vehicle object if this hasn't already been done.
	createVehicleObject(ah, objHands, ow);

	const OmniPvdContextHandle ch = objHands.contextHandle;

	if(physxActor && physxActor->rigidBody)
	{
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.physxRigidActorOH);
		createPvdObject(ow, ch, ah.physxRigidActor.CH, oh, "PhysXRigidActor");
		objHands.physxRigidActorOH = oh;
		writeObjectHandleAttribute(ow, ch, objHands.vehicleOH, ah.vehicle.physxRigidActorAH, oh);		
	}
}

void PxVehiclePvdPhysXRigidActorWrite
(const PxVehiclePhysXActor* physxActor,
 const PxVehiclePvdAttributeHandles& ah,
 const PxVehiclePvdObjectHandles& oh, OmniPvdWriter& ow)
{
	if(oh.physxRigidActorOH && physxActor)
	{
		writePhysXRigidActor(physxActor->rigidBody, oh.physxRigidActorOH, ah.physxRigidActor, ow, oh.contextHandle);
	}
}

void PxVehiclePvdPhysXSteerStateRegister
(const PxVehiclePhysXSteerState* physxSteerState,
 const PxVehiclePvdAttributeHandles& ah,
 PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	// Register the top-level vehicle object if this hasn't already been done.
	createVehicleObject(ah, objHands, ow);

	const OmniPvdContextHandle ch = objHands.contextHandle;

	if(physxSteerState)
	{
		const OmniPvdObjectHandle oh = reinterpret_cast<OmniPvdObjectHandle>(&objHands.physxSteerStateOH);
		createPvdObject(ow, ch, ah.physxSteerState.CH, oh, "PhysXSteerState");
		objHands.physxSteerStateOH = oh;
		writeObjectHandleAttribute(ow, ch, objHands.vehicleOH, ah.vehicle.physxSteerStateAH, oh);		
	}
}

void PxVehiclePvdPhysXSteerStateWrite
(const PxVehiclePhysXSteerState* physxSteerState,
 const PxVehiclePvdAttributeHandles& ah,
 const PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	if (objHands.physxSteerStateOH && physxSteerState)  // TODO: test against invalid handle once that gets introduced
	{
		writePhysXSteerState(*physxSteerState, objHands.physxSteerStateOH, ah.physxSteerState, ow, objHands.contextHandle);
	}
}

#else //PX_SUPPORT_OMNI_PVD

void PxVehiclePvdRigidBodyRegister
(const PxVehicleRigidBodyParams* rbodyParams, const PxVehicleRigidBodyState* rbodyState,
 const PxVehiclePvdAttributeHandles& ah,
 PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	PX_UNUSED(rbodyParams);
	PX_UNUSED(rbodyState);
	PX_UNUSED(ah);
	PX_UNUSED(objHands);
	PX_UNUSED(ow);
}

void PxVehiclePvdRigidBodyWrite
(const PxVehicleRigidBodyParams* rbodyParams, const PxVehicleRigidBodyState* rbodyState,
 const PxVehiclePvdAttributeHandles& ah,
 const PxVehiclePvdObjectHandles& oh, OmniPvdWriter& ow)
{
	PX_UNUSED(rbodyParams);
	PX_UNUSED(rbodyState);
	PX_UNUSED(ah);
	PX_UNUSED(oh);
	PX_UNUSED(ow);
}

void PxVehiclePvdSuspensionStateCalculationParamsRegister
(const PxVehicleSuspensionStateCalculationParams* suspStateCalcParams, 
 const PxVehiclePvdAttributeHandles& ah,
 PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	PX_UNUSED(suspStateCalcParams);
	PX_UNUSED(ah);
	PX_UNUSED(objHands);
	PX_UNUSED(ow);
}

void PxVehiclePvdSuspensionStateCalculationParamsWrite
(const PxVehicleSuspensionStateCalculationParams* suspStateCalcParams, 
 const PxVehiclePvdAttributeHandles& ah,
 const PxVehiclePvdObjectHandles& oh, OmniPvdWriter& ow)
{
	PX_UNUSED(suspStateCalcParams);
	PX_UNUSED(ah);
	PX_UNUSED(oh);
	PX_UNUSED(ow);
}

void PxVehiclePvdCommandResponseRegister
(const PxVehicleSizedArrayData<const PxVehicleBrakeCommandResponseParams>& brakeResponseParams,
 const PxVehicleSteerCommandResponseParams* steerResponseParams,
 const PxVehicleAckermannParams* ackermannParams,
 const PxVehicleArrayData<PxReal>& brakeResponseStates,
 const PxVehicleArrayData<PxReal>& steerResponseStates,
 const PxVehiclePvdAttributeHandles& ah,
 PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	PX_UNUSED(steerResponseParams);
	PX_UNUSED(brakeResponseParams);
	PX_UNUSED(ackermannParams);
	PX_UNUSED(steerResponseStates);
	PX_UNUSED(brakeResponseStates);
	PX_UNUSED(ah);
	PX_UNUSED(objHands);
	PX_UNUSED(ow);
}

void PxVehiclePvdCommandResponseWrite
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleSizedArrayData<const PxVehicleBrakeCommandResponseParams>& brakeResponseParams,
 const PxVehicleSteerCommandResponseParams* steerResponseParams, 
 const PxVehicleAckermannParams* ackermannParams,
 const PxVehicleArrayData<PxReal>& brakeResponseStates,
 const PxVehicleArrayData<PxReal>& steerResponseStates,
 const PxVehiclePvdAttributeHandles& ah,
 const PxVehiclePvdObjectHandles& oh, OmniPvdWriter& ow)
{
	PX_UNUSED(axleDesc);
	PX_UNUSED(steerResponseParams);
	PX_UNUSED(brakeResponseParams);
	PX_UNUSED(ackermannParams);
	PX_UNUSED(steerResponseStates);
	PX_UNUSED(brakeResponseStates);
	PX_UNUSED(ah);
	PX_UNUSED(oh);
	PX_UNUSED(ow);
}

void PxVehiclePvdWheelAttachmentsRegister
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleArrayData<const PxVehicleWheelParams>& wheelParams,
 const PxVehicleArrayData<const PxVehicleWheelActuationState>& wheelActuationStates,
 const PxVehicleArrayData<const PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
 const PxVehicleArrayData<const PxVehicleWheelLocalPose>& wheelLocalPoses,
 const PxVehicleArrayData<const PxVehicleRoadGeometryState>& roadGeometryStates,
 const PxVehicleArrayData<const PxVehicleSuspensionParams>& suspParams,
 const PxVehicleArrayData<const PxVehicleSuspensionComplianceParams>& suspCompParams,
 const PxVehicleArrayData<const PxVehicleSuspensionForceParams>& suspForceParams,
 const PxVehicleArrayData<const PxVehicleSuspensionState>& suspStates,
 const PxVehicleArrayData<const PxVehicleSuspensionComplianceState>& suspCompStates,
 const PxVehicleArrayData<const PxVehicleSuspensionForce>& suspForces,
 const PxVehicleArrayData<const PxVehicleTireForceParams>& tireForceParams,
 const PxVehicleArrayData<const PxVehicleTireDirectionState>& tireDirectionStates,
 const PxVehicleArrayData<const PxVehicleTireSpeedState>& tireSpeedStates,
 const PxVehicleArrayData<const PxVehicleTireSlipState>& tireSlipStates,
 const PxVehicleArrayData<const PxVehicleTireStickyState>& tireStickyStates,
 const PxVehicleArrayData<const PxVehicleTireGripState>& tireGripStates,
 const PxVehicleArrayData<const PxVehicleTireCamberAngleState>& tireCamberStates,
 const PxVehicleArrayData<const PxVehicleTireForce>& tireForces, 
 const PxVehiclePvdAttributeHandles& ah, 
 PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	PX_UNUSED(axleDesc);
	PX_UNUSED(wheelParams);
	PX_UNUSED(wheelActuationStates);
	PX_UNUSED(wheelRigidBody1dStates);
	PX_UNUSED(wheelLocalPoses);
	PX_UNUSED(roadGeometryStates);
	PX_UNUSED(suspParams);
	PX_UNUSED(suspCompParams);
	PX_UNUSED(suspForceParams);
	PX_UNUSED(suspStates);
	PX_UNUSED(suspCompStates);
	PX_UNUSED(suspForces);
	PX_UNUSED(tireForceParams);
	PX_UNUSED(tireDirectionStates);
	PX_UNUSED(tireSpeedStates);
	PX_UNUSED(tireSlipStates);
	PX_UNUSED(tireStickyStates);
	PX_UNUSED(tireGripStates);
	PX_UNUSED(tireCamberStates);
	PX_UNUSED(tireForces);
	PX_UNUSED(ah);
	PX_UNUSED(objHands);
	PX_UNUSED(ow);
}

void PxVehiclePvdWheelAttachmentsWrite
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleArrayData<const PxVehicleWheelParams>& wheelParams,
 const PxVehicleArrayData<const PxVehicleWheelActuationState>& wheelActuationStates,
 const PxVehicleArrayData<const PxVehicleWheelRigidBody1dState>& wheelRigidBody1dStates,
 const PxVehicleArrayData<const PxVehicleWheelLocalPose>& wheelLocalPoses,
 const PxVehicleArrayData<const PxVehicleRoadGeometryState>& roadGeometryStates,
 const PxVehicleArrayData<const PxVehicleSuspensionParams>& suspParams,
 const PxVehicleArrayData<const PxVehicleSuspensionComplianceParams>& suspComplianceParams,
 const PxVehicleArrayData<const PxVehicleSuspensionForceParams>& suspForceParams,
 const PxVehicleArrayData<const PxVehicleSuspensionState>& suspStates,
 const PxVehicleArrayData<const PxVehicleSuspensionComplianceState>& suspCompStates,
 const PxVehicleArrayData<const PxVehicleSuspensionForce>& suspForces,
 const PxVehicleArrayData<const PxVehicleTireForceParams>& tireForceParams,
 const PxVehicleArrayData<const PxVehicleTireDirectionState>& tireDirectionStates,
 const PxVehicleArrayData<const PxVehicleTireSpeedState>& tireSpeedStates,
 const PxVehicleArrayData<const PxVehicleTireSlipState>& tireSlipStates,
 const PxVehicleArrayData<const PxVehicleTireStickyState>& tireStickyStates,
 const PxVehicleArrayData<const PxVehicleTireGripState>& tireGripStates,
 const PxVehicleArrayData<const PxVehicleTireCamberAngleState>& tireCamberStates,
 const PxVehicleArrayData<const PxVehicleTireForce>& tireForces, 
 const PxVehiclePvdAttributeHandles& ah,
 const PxVehiclePvdObjectHandles& oh, OmniPvdWriter& ow)
{
	PX_UNUSED(axleDesc);
	PX_UNUSED(wheelParams);
	PX_UNUSED(wheelActuationStates);
	PX_UNUSED(wheelRigidBody1dStates);
	PX_UNUSED(wheelLocalPoses);
	PX_UNUSED(roadGeometryStates);
	PX_UNUSED(suspParams);
	PX_UNUSED(suspComplianceParams);
	PX_UNUSED(suspForceParams);
	PX_UNUSED(suspStates);
	PX_UNUSED(suspCompStates);
	PX_UNUSED(suspForces);
	PX_UNUSED(tireForceParams);
	PX_UNUSED(tireDirectionStates);
	PX_UNUSED(tireSpeedStates);
	PX_UNUSED(tireSlipStates);
	PX_UNUSED(tireStickyStates);
	PX_UNUSED(tireGripStates);
	PX_UNUSED(tireCamberStates);
	PX_UNUSED(tireForces);
	PX_UNUSED(ah);
	PX_UNUSED(oh);
	PX_UNUSED(ow);
}
	
void PxVehiclePvdDirectDrivetrainRegister
(const PxVehicleCommandState* commandState, const PxVehicleDirectDriveTransmissionCommandState* transmissionState,
 const PxVehicleDirectDriveThrottleCommandResponseParams* directDriveThrottleResponseParams,
 const PxVehicleArrayData<PxReal>& directDriveThrottleResponseState,
 const PxVehiclePvdAttributeHandles& ah,
 PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	PX_UNUSED(commandState);
	PX_UNUSED(transmissionState);
	PX_UNUSED(directDriveThrottleResponseParams);
	PX_UNUSED(directDriveThrottleResponseState);
	PX_UNUSED(ah);
	PX_UNUSED(objHands);
	PX_UNUSED(ow);
}

void PxVehiclePvdDirectDrivetrainWrite
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleCommandState* commandState, const PxVehicleDirectDriveTransmissionCommandState* transmissionState,
 const PxVehicleDirectDriveThrottleCommandResponseParams* directDriveThrottleResponseParams,
 const  PxVehicleArrayData<PxReal>& directDriveThrottleResponseState,
 const PxVehiclePvdAttributeHandles& ah,
 const PxVehiclePvdObjectHandles& oh, OmniPvdWriter& ow)
{
	PX_UNUSED(axleDesc);
	PX_UNUSED(commandState);
	PX_UNUSED(transmissionState);
	PX_UNUSED(directDriveThrottleResponseParams);
	PX_UNUSED(directDriveThrottleResponseState);
	PX_UNUSED(ah);
	PX_UNUSED(oh);
	PX_UNUSED(ow);
}

void PxVehiclePvdEngineDrivetrainRegister
(const PxVehicleCommandState* commandState,
 const PxVehicleEngineDriveTransmissionCommandState* engineDriveTransmissionCommandState,
 const PxVehicleTankDriveTransmissionCommandState* tankDriveTransmissionCommandState,
 const PxVehicleClutchCommandResponseParams* clutchResponseParams,
 const PxVehicleClutchParams* clutchParms,
 const PxVehicleEngineParams* engineParams,
 const PxVehicleGearboxParams* gearboxParams,
 const PxVehicleAutoboxParams* autoboxParams,
 const PxVehicleMultiWheelDriveDifferentialParams* multiWheelDiffParams,
 const PxVehicleFourWheelDriveDifferentialParams* fourWheelDiffPrams,
 const PxVehicleTankDriveDifferentialParams* tankDiffParams,
 const PxVehicleClutchCommandResponseState* clutchResponseState,
 const PxVehicleEngineDriveThrottleCommandResponseState* throttleResponseState,
 const PxVehicleEngineState* engineState,
 const PxVehicleGearboxState* gearboxState,
 const PxVehicleAutoboxState* autoboxState,
 const PxVehicleDifferentialState* diffState,
 const PxVehicleClutchSlipState* clutchSlipState,
 const PxVehiclePvdAttributeHandles& ah,
 PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	PX_UNUSED(commandState);
	PX_UNUSED(engineDriveTransmissionCommandState);
	PX_UNUSED(tankDriveTransmissionCommandState);
	PX_UNUSED(clutchResponseParams);
	PX_UNUSED(clutchParms);
	PX_UNUSED(engineParams);
	PX_UNUSED(gearboxParams);
	PX_UNUSED(autoboxParams);
	PX_UNUSED(multiWheelDiffParams);
	PX_UNUSED(fourWheelDiffPrams);
	PX_UNUSED(tankDiffParams);
	PX_UNUSED(clutchResponseState);
	PX_UNUSED(throttleResponseState);
	PX_UNUSED(engineState);
	PX_UNUSED(gearboxState);
	PX_UNUSED(autoboxState);
	PX_UNUSED(diffState);
	PX_UNUSED(clutchSlipState);
	PX_UNUSED(ah);
	PX_UNUSED(objHands);
	PX_UNUSED(ow);
}

void PxVehiclePvdEngineDrivetrainWrite
(const PxVehicleCommandState* commandState,
 const PxVehicleEngineDriveTransmissionCommandState* engineDriveTransmissionCommandState,
 const PxVehicleTankDriveTransmissionCommandState* tankDriveTransmissionCommandState,
 const PxVehicleClutchCommandResponseParams* clutchResponseParams,
 const PxVehicleClutchParams* clutchParms,
 const PxVehicleEngineParams* engineParams,
 const PxVehicleGearboxParams* gearboxParams,
 const PxVehicleAutoboxParams* autoboxParams,
 const PxVehicleMultiWheelDriveDifferentialParams* multiWheelDiffParams,
 const PxVehicleFourWheelDriveDifferentialParams* fourWheelDiffParams,
 const PxVehicleTankDriveDifferentialParams* tankDiffParams,
 const PxVehicleClutchCommandResponseState* clutchResponseState,
 const PxVehicleEngineDriveThrottleCommandResponseState* throttleResponseState,
 const PxVehicleEngineState* engineState,
 const PxVehicleGearboxState* gearboxState,
 const PxVehicleAutoboxState* autoboxState,
 const PxVehicleDifferentialState* diffState,
 const PxVehicleClutchSlipState* clutchSlipState,
 const PxVehiclePvdAttributeHandles& ah,
 const PxVehiclePvdObjectHandles& oh, OmniPvdWriter& omniWriter)
{
	PX_UNUSED(commandState);
	PX_UNUSED(engineDriveTransmissionCommandState);
	PX_UNUSED(tankDriveTransmissionCommandState);
	PX_UNUSED(clutchResponseParams);
	PX_UNUSED(clutchParms);
	PX_UNUSED(engineParams);
	PX_UNUSED(gearboxParams);
	PX_UNUSED(autoboxParams);
	PX_UNUSED(multiWheelDiffParams);
	PX_UNUSED(fourWheelDiffParams);
	PX_UNUSED(tankDiffParams);
	PX_UNUSED(clutchResponseState);
	PX_UNUSED(throttleResponseState);
	PX_UNUSED(engineState);
	PX_UNUSED(gearboxState);
	PX_UNUSED(autoboxState);
	PX_UNUSED(diffState);
	PX_UNUSED(clutchSlipState);
	PX_UNUSED(ah);
	PX_UNUSED(oh);
	PX_UNUSED(omniWriter);
}

void PxVehiclePvdAntiRollsRegister
(const PxVehicleSizedArrayData<const PxVehicleAntiRollForceParams>& antiRollForceParams,
 const PxVehicleAntiRollTorque* antiRollTorque,
const PxVehiclePvdAttributeHandles& ah, 
 PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	PX_UNUSED(antiRollForceParams);
	PX_UNUSED(antiRollTorque);
	PX_UNUSED(ah);
	PX_UNUSED(objHands);
	PX_UNUSED(ow);
}

void PxVehiclePvdAntiRollsWrite
(const PxVehicleSizedArrayData<const PxVehicleAntiRollForceParams>& antiRollForceParams,
 const PxVehicleAntiRollTorque* antiRollTorque,
 const PxVehiclePvdAttributeHandles& ah, 
 const PxVehiclePvdObjectHandles& oh, OmniPvdWriter& ow)
{
	PX_UNUSED(antiRollForceParams);
	PX_UNUSED(antiRollTorque);
	PX_UNUSED(ah);
	PX_UNUSED(oh);
	PX_UNUSED(ow);
}

void PxVehiclePvdPhysXWheelAttachmentRegister
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleArrayData<const PxVehiclePhysXSuspensionLimitConstraintParams>& physxSuspLimitConstraintParams,
 const PxVehicleArrayData<const PxVehiclePhysXMaterialFrictionParams>& physxMaterialFrictionParams,
 const PxVehiclePhysXActor* physxActor, const PxVehiclePhysXRoadGeometryQueryParams* physxRoadGeomQryParams,
 const PxVehicleArrayData<const PxVehiclePhysXRoadGeometryQueryState>&  physxRoadGeomState,
 const PxVehicleArrayData<const PxVehiclePhysXConstraintState>& physxConstraintStates,
 const PxVehiclePvdAttributeHandles& ah,
 PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	PX_UNUSED(axleDesc);
	PX_UNUSED(physxSuspLimitConstraintParams);
	PX_UNUSED(physxMaterialFrictionParams);
	PX_UNUSED(physxActor);
	PX_UNUSED(physxRoadGeomQryParams);
	PX_UNUSED(physxRoadGeomState);
	PX_UNUSED(physxConstraintStates);
	PX_UNUSED(ah);
	PX_UNUSED(objHands);
	PX_UNUSED(ow);
}

void PxVehiclePvdPhysXWheelAttachmentWrite
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleArrayData<const PxVehiclePhysXSuspensionLimitConstraintParams>& physxSuspLimitConstraintParams,
 const PxVehicleArrayData<const PxVehiclePhysXMaterialFrictionParams>& physxMaterialFrictionParams,
 const PxVehiclePhysXActor* physxActor, const PxVehiclePhysXRoadGeometryQueryParams* physxRoadGeomQryParams,
 const PxVehicleArrayData<const PxVehiclePhysXRoadGeometryQueryState>& physxRoadGeomStates,
 const PxVehicleArrayData<const PxVehiclePhysXConstraintState>& physxConstraintStates,
 const PxVehiclePvdAttributeHandles& ah,
 const PxVehiclePvdObjectHandles& oh, OmniPvdWriter& ow)
{
	PX_UNUSED(axleDesc);
	PX_UNUSED(physxSuspLimitConstraintParams);
	PX_UNUSED(physxMaterialFrictionParams);
	PX_UNUSED(physxActor);
	PX_UNUSED(physxRoadGeomQryParams);
	PX_UNUSED(physxRoadGeomStates);
	PX_UNUSED(physxConstraintStates);
	PX_UNUSED(ah);
	PX_UNUSED(oh);
	PX_UNUSED(ow);
}

void PxVehiclePvdPhysXRigidActorRegister
(const PxVehiclePhysXActor* physxActor,
 const PxVehiclePvdAttributeHandles& ah,
 PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	PX_UNUSED(physxActor);
	PX_UNUSED(ah);
	PX_UNUSED(objHands);
	PX_UNUSED(ow);
}

void PxVehiclePvdPhysXRigidActorWrite
(const PxVehiclePhysXActor* physxActor,
 const PxVehiclePvdAttributeHandles& ah,
 const PxVehiclePvdObjectHandles& oh, OmniPvdWriter& ow)
{
	PX_UNUSED(physxActor);
	PX_UNUSED(ah);
	PX_UNUSED(oh);
	PX_UNUSED(ow);
}

void PxVehiclePvdPhysXSteerStateRegister
(const PxVehiclePhysXSteerState* physxSteerState,
 const PxVehiclePvdAttributeHandles& ah,
 PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	PX_UNUSED(physxSteerState);
	PX_UNUSED(ah);
	PX_UNUSED(objHands);
	PX_UNUSED(ow);
}

void PxVehiclePvdPhysXSteerStateWrite
(const PxVehiclePhysXSteerState* physxSteerState,
 const PxVehiclePvdAttributeHandles& ah,
 const PxVehiclePvdObjectHandles& objHands, OmniPvdWriter& ow)
{
	PX_UNUSED(physxSteerState);
	PX_UNUSED(ah);
	PX_UNUSED(objHands);
	PX_UNUSED(ow);
}

#endif

} // namespace vehicle2
} // namespace physx

/** @} */
