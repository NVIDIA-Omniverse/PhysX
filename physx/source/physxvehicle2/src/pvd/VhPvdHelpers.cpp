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

#include "vehicle2/pvd/PxVehiclePvdHelpers.h"
#include "foundation/PxAllocatorCallback.h"
#include "VhPvdAttributeHandles.h"
#include "VhPvdObjectHandles.h"

namespace physx
{
namespace vehicle2
{

#if PX_SUPPORT_OMNI_PVD

///////////////////////////////
//ATTRIBUTE REGISTRATION
///////////////////////////////

PxVehiclePvdAttributeHandles* PxVehiclePvdAttributesCreate(PxAllocatorCallback& allocator, OmniPvdWriter& omniWriter)
{
	PxVehiclePvdAttributeHandles* attributeHandles = 
		reinterpret_cast<PxVehiclePvdAttributeHandles*>(
	    allocator.allocate(sizeof(PxVehiclePvdAttributeHandles), "PxVehiclePvdAttributeHandles", __FILE__, __LINE__));
	PxMemZero(attributeHandles, sizeof(PxVehiclePvdAttributeHandles));

	//Rigid body
	attributeHandles->rigidBodyParams = registerRigidBodyParams(omniWriter);
	attributeHandles->rigidBodyState = registerRigidBodyState(omniWriter);

	//Susp state calc params.
	attributeHandles->suspStateCalcParams = registerSuspStateCalcParams(omniWriter);

	//Controls
	attributeHandles->steerCommandResponseParams = registerSteerResponseParams(omniWriter);
	attributeHandles->brakeCommandResponseParams = registerBrakeResponseParams(omniWriter);
	attributeHandles->steerCommandResponseStates = registerSteerResponseStates(omniWriter);
	attributeHandles->brakeCommandResponseStates = registerBrakeResponseStates(omniWriter);

	//Wheel attachment
	attributeHandles->wheelParams = registerWheelParams(omniWriter);
	attributeHandles->wheelActuationState = registerWheelActuationState(omniWriter);
	attributeHandles->wheelRigidBody1dState = registerWheelRigidBody1dState(omniWriter);
	attributeHandles->wheelLocalPoseState = registerWheelLocalPoseState(omniWriter);
	attributeHandles->roadGeomState = registerRoadGeomState(omniWriter);
	attributeHandles->suspParams = registerSuspParams(omniWriter);
	attributeHandles->suspCompParams = registerSuspComplianceParams(omniWriter);
	attributeHandles->suspForceParams = registerSuspForceParams(omniWriter);
	attributeHandles->suspState = registerSuspState(omniWriter);
	attributeHandles->suspCompState = registerSuspComplianceState(omniWriter);
	attributeHandles->suspForce = registerSuspForce(omniWriter);
	attributeHandles->tireParams = registerTireParams(omniWriter);
	attributeHandles->tireDirectionState = registerTireDirectionState(omniWriter);
	attributeHandles->tireSpeedState = registerTireSpeedState(omniWriter);
	attributeHandles->tireSlipState = registerTireSlipState(omniWriter);
	attributeHandles->tireStickyState = registerTireStickyState(omniWriter);
	attributeHandles->tireGripState = registerTireGripState(omniWriter);
	attributeHandles->tireCamberState = registerTireCamberState(omniWriter);
	attributeHandles->tireForce = registerTireForce(omniWriter);
	attributeHandles->wheelAttachment = registerWheelAttachment(omniWriter);
		
	//Antiroll
	attributeHandles->antiRollParams = registerAntiRollParams(omniWriter);
	attributeHandles->antiRollForce = registerAntiRollForce(omniWriter);

	//Direct drivetrain
	attributeHandles->directDriveThrottleCommandResponseParams = registerDirectDriveThrottleResponseParams(omniWriter);
	attributeHandles->directDriveCommandState = registerDirectDriveCommandState(omniWriter);
	attributeHandles->directDriveTransmissionCommandState = registerDirectDriveTransmissionCommandState(omniWriter);
	attributeHandles->directDriveThrottleCommandResponseState = registerDirectDriveThrottleResponseState(omniWriter);
	attributeHandles->directDrivetrain = registerDirectDrivetrain(omniWriter);

	//Engine drivetrain
	attributeHandles->engineDriveCommandState = registerEngineDriveCommandState(omniWriter);
	attributeHandles->engineDriveTransmissionCommandState = registerEngineDriveTransmissionCommandState(omniWriter);
	attributeHandles->clutchCommandResponseParams = registerClutchResponseParams(omniWriter);
	attributeHandles->clutchParams = registerClutchParams(omniWriter);
	attributeHandles->engineParams = registerEngineParams(omniWriter);
	attributeHandles->gearboxParams = registerGearboxParams(omniWriter);
	attributeHandles->autoboxParams = registerAutoboxParams(omniWriter);
	attributeHandles->multiwheelDiffParams = registerMultiWheelDiffParams(omniWriter);
	attributeHandles->fourwheelDiffParams = registerFourWheelDiffParams(omniWriter);
	attributeHandles->clutchResponseState = registerClutchResponseState(omniWriter);
	attributeHandles->throttleResponseState = registerThrottleResponseState(omniWriter);
	attributeHandles->engineState = registerEngineState(omniWriter);
	attributeHandles->gearboxState = registerGearboxState(omniWriter);
	attributeHandles->autoboxState = registerAutoboxState(omniWriter);
	attributeHandles->diffState = registerDiffState(omniWriter);
	attributeHandles->clutchSlipState = registerClutchSlipState(omniWriter);
	attributeHandles->engineDrivetrain = registerEngineDrivetrain(omniWriter);

	//Physx wheel attachment
	attributeHandles->physxSuspLimitConstraintParams = registerSuspLimitConstraintParams(omniWriter);
	attributeHandles->physxWheelShape = registerPhysXWheelShape(omniWriter);
	attributeHandles->physxRoadGeomState = registerPhysXRoadGeomState(omniWriter);
	attributeHandles->physxConstraintState = registerPhysXConstraintState(omniWriter);
	attributeHandles->physxWheelAttachment = registerPhysXWheelAttachment(omniWriter);
	attributeHandles->physxMaterialFriction = registerPhysXMaterialFriction(omniWriter);

	//Physx rigid actor
	attributeHandles->physxRoadGeometryQueryParams = registerPhysXRoadGeometryQueryParams(omniWriter);
	attributeHandles->physxRigidActor = registerPhysXRigidActor(omniWriter);
	
	//Vehicle
	attributeHandles->vehicle = registerVehicle(omniWriter);

	return attributeHandles;
}

///////////////////////////////
//ATTRIBUTE DESTRUCTION
///////////////////////////////

void PxVehiclePvdAttributesRelease(PxAllocatorCallback& allocator, PxVehiclePvdAttributeHandles& attributeHandles)
{
	allocator.deallocate(&attributeHandles);
}

////////////////////////////////////////
//OBJECT REGISTRATION
////////////////////////////////////////

PxVehiclePvdObjectHandles* PxVehiclePvdObjectCreate
(const PxU32 nbWheels, const PxU32 nbAntirolls, const PxU32 maxNbPhysXMaterialFrictions,
 const OmniPvdContextHandle contextHandle,
 PxAllocatorCallback& allocator)
{
	const PxU32 byteSize = 
		sizeof(PxVehiclePvdObjectHandles) + 
		sizeof(OmniPvdObjectHandle)*nbWheels*(
			1 +								//OmniPvdObjectHandle* wheelAttachmentOHs;
			1 +								//OmniPvdObjectHandle* wheelParamsOHs;
			1 +								//OmniPvdObjectHandle* wheelActuationStateOHs;
			1 +								//OmniPvdObjectHandle* wheelRigidBody1dStateOHs;
			1 +								//OmniPvdObjectHandle* wheelLocalPoseStateOHs;
			1 +								//OmniPvdObjectHandle* roadGeomStateOHs;
			1 +								//OmniPvdObjectHandle* suspParamsOHs;
			1 +								//OmniPvdObjectHandle* suspCompParamsOHs;
			1 +								//OmniPvdObjectHandle* suspForceParamsOHs;
			1 +								//OmniPvdObjectHandle* suspStateOHs;
			1 +								//OmniPvdObjectHandle* suspCompStateOHs;
			1 +								//OmniPvdObjectHandle* suspForceOHs;
			1 +								//OmniPvdObjectHandle* tireParamsOHs;
			1 +								//OmniPvdObjectHandle* tireDirectionStateOHs;
			1 +								//OmniPvdObjectHandle* tireSpeedStateOHs;
			1 +								//OmniPvdObjectHandle* tireSlipStateOHs;
			1 +								//OmniPvdObjectHandle* tireStickyStateOHs;
			1 +								//OmniPvdObjectHandle* tireGripStateOHs;
			1 +								//OmniPvdObjectHandle* tireCamberStateOHs;
			1 +								//OmniPvdObjectHandle* tireForceOHs;
			1 +								//OmniPvdObjectHandle* physxWheelAttachmentOHs;
			1 +								//OmniPvdObjectHandle* physxWheelShapeOHs;
			1 +								//OmniPvdObjectHandle* physxConstraintParamOHs;
			1 +								//OmniPvdObjectHandle* physxConstraintStateOHs;
			1 +								//OmniPvdObjectHandle* physxRoadGeomStateOHs;
			1 +								//OmniPvdObjectHandle* physxMaterialFrictionSetOHs;
			maxNbPhysXMaterialFrictions) +	//OmniPvdObjectHandle* physxMaterialFrictionOHs;
		sizeof(OmniPvdObjectHandle)*nbAntirolls*(
			1);								//OmniPvdObjectHandle* antiRollParamOHs
		 
	
	PxU8* buffer = reinterpret_cast<PxU8*>(allocator.allocate(byteSize, "PxVehiclePvdObjectHandles", __FILE__, __LINE__));
#if PX_ENABLE_ASSERTS
	PxU8* start = buffer;
#endif
	PxMemZero(buffer, byteSize);
	
	PxVehiclePvdObjectHandles* objectHandles = reinterpret_cast<PxVehiclePvdObjectHandles*>(buffer);
	buffer += sizeof(PxVehiclePvdObjectHandles);

	if(nbWheels != 0)
	{
		objectHandles->wheelAttachmentOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->wheelParamsOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->wheelActuationStateOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->wheelRigidBody1dStateOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->wheelLocalPoseStateOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->roadGeomStateOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->suspParamsOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->suspCompParamsOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->suspForceParamsOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->suspStateOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->suspCompStateOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->suspForceOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->tireParamsOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->tireDirectionStateOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->tireSpeedStateOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->tireSlipStateOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->tireStickyStateOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->tireGripStateOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->tireCamberStateOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->tireForceOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->physxWheelAttachmentOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->physxConstraintParamOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->physxWheelShapeOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->physxConstraintStateOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		objectHandles->physxRoadGeomStateOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
		if(maxNbPhysXMaterialFrictions != 0)
		{
			objectHandles->physxMaterialFrictionSetOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
			buffer += sizeof(OmniPvdObjectHandle)*nbWheels;
			objectHandles->physxMaterialFrictionOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
			buffer += sizeof(OmniPvdObjectHandle)*nbWheels*maxNbPhysXMaterialFrictions;
		}
	}

	if(nbAntirolls != 0)
	{
		objectHandles->antiRollParamOHs = reinterpret_cast<OmniPvdObjectHandle*>(buffer);
		buffer += sizeof(OmniPvdObjectHandle)*nbAntirolls;
	}

	objectHandles->nbWheels = nbWheels;
	objectHandles->nbPhysXMaterialFrictions = maxNbPhysXMaterialFrictions;
	objectHandles->nbAntirolls = nbAntirolls;

	objectHandles->contextHandle = contextHandle;

	PX_ASSERT((start + byteSize) == buffer);

	return objectHandles;
}

////////////////////////////////////
//OBJECT DESTRUCTION
////////////////////////////////////

PX_FORCE_INLINE void destroyObject
(OmniPvdWriter& omniWriter, OmniPvdContextHandle ch,
 OmniPvdObjectHandle oh)
{
	// note: "0" needs to be replaced with a marker for invalid object handle as soon as PVD
	//       provides it (and PxVehiclePvdObjectCreate needs to initialize accordingly or
	//       compile time assert that the value is 0 for now)
	if(oh != 0)
		omniWriter.destroyObject(ch, oh);
}

void PxVehiclePvdObjectRelease
(OmniPvdWriter& ow, PxAllocatorCallback& allocator, PxVehiclePvdObjectHandles& oh)
{
	const OmniPvdContextHandle ch = oh.contextHandle;

	//rigid body
	destroyObject(ow, ch, oh.rigidBodyParamsOH);
	destroyObject(ow, ch, oh.rigidBodyStateOH);
	
	//susp state calc params
	destroyObject(ow, ch, oh.suspStateCalcParamsOH);

	//controls
	for(PxU32 i = 0; i < 2; i++)
	{
		destroyObject(ow, ch, oh.brakeResponseParamOHs[i]);
	}
	destroyObject(ow, ch, oh.steerResponseParamsOH);
	destroyObject(ow, ch, oh.brakeResponseStateOH);
	destroyObject(ow, ch, oh.steerResponseStateOH);

	//Wheel attachments
	for(PxU32 i = 0; i < oh.nbWheels; i++)
	{
		destroyObject(ow, ch, oh.wheelParamsOHs[i]);
		destroyObject(ow, ch, oh.wheelActuationStateOHs[i]);
		destroyObject(ow, ch, oh.wheelRigidBody1dStateOHs[i]);
		destroyObject(ow, ch, oh.wheelLocalPoseStateOHs[i]);
		destroyObject(ow, ch, oh.suspParamsOHs[i]);
		destroyObject(ow, ch, oh.suspCompParamsOHs[i]);
		destroyObject(ow, ch, oh.suspForceParamsOHs[i]);
		destroyObject(ow, ch, oh.suspStateOHs[i]);
		destroyObject(ow, ch, oh.suspCompStateOHs[i]);
		destroyObject(ow, ch, oh.suspForceOHs[i]);
		destroyObject(ow, ch, oh.tireParamsOHs[i]);
		destroyObject(ow, ch, oh.tireDirectionStateOHs[i]);
		destroyObject(ow, ch, oh.tireSpeedStateOHs[i]);
		destroyObject(ow, ch, oh.tireSlipStateOHs[i]);
		destroyObject(ow, ch, oh.tireStickyStateOHs[i]);
		destroyObject(ow, ch, oh.tireGripStateOHs[i]);
		destroyObject(ow, ch, oh.tireCamberStateOHs[i]);
		destroyObject(ow, ch, oh.tireForceOHs[i]);
		destroyObject(ow, ch, oh.wheelAttachmentOHs[i]);
	}	

	//Antiroll
	for(PxU32 i = 0; i < oh.nbAntirolls; i++)
	{
		destroyObject(ow, ch, oh.antiRollParamOHs[i]);
	}
	destroyObject(ow, ch, oh.antiRollTorqueOH);

	//direct drive
	destroyObject(ow, ch, oh.directDriveCommandStateOH);
	destroyObject(ow, ch, oh.directDriveTransmissionCommandStateOH);
	destroyObject(ow, ch, oh.directDriveThrottleResponseParamsOH);
	destroyObject(ow, ch, oh.directDriveThrottleResponseStateOH);
	destroyObject(ow, ch, oh.directDrivetrainOH);

	//engine drive
	destroyObject(ow, ch, oh.engineDriveCommandStateOH);
	destroyObject(ow, ch, oh.engineDriveTransmissionCommandStateOH);
	destroyObject(ow, ch, oh.clutchResponseParamsOH);
	destroyObject(ow, ch, oh.clutchParamsOH);
	destroyObject(ow, ch, oh.engineParamsOH);
	destroyObject(ow, ch, oh.gearboxParamsOH);
	destroyObject(ow, ch, oh.autoboxParamsOH);
	destroyObject(ow, ch, oh.multiWheelDiffParamsOH);
	destroyObject(ow, ch, oh.fourWheelDiffParamsOH);
	destroyObject(ow, ch, oh.clutchResponseStateOH);
	destroyObject(ow, ch, oh.engineDriveThrottleResponseStateOH);
	destroyObject(ow, ch, oh.engineStateOH);
	destroyObject(ow, ch, oh.gearboxStateOH);
	destroyObject(ow, ch, oh.autoboxStateOH);
	destroyObject(ow, ch, oh.diffStateOH);
	destroyObject(ow, ch, oh.clutchSlipStateOH);
	destroyObject(ow, ch, oh.engineDrivetrainOH);

	//PhysX Wheel attachments
	for(PxU32 i = 0; i < oh.nbWheels; i++)
	{
		destroyObject(ow, ch, oh.physxConstraintParamOHs[i]);
		destroyObject(ow, ch, oh.physxWheelShapeOHs[i]);
		destroyObject(ow, ch, oh.physxRoadGeomStateOHs[i]);
		destroyObject(ow, ch, oh.physxConstraintStateOHs[i]);
		for(PxU32 j = 0; j < oh.nbPhysXMaterialFrictions; j++)
		{
			const PxU32 id = i*oh.nbPhysXMaterialFrictions + j;
			destroyObject(ow, ch, oh.physxMaterialFrictionOHs[id]);
		}
		destroyObject(ow, ch, oh.physxWheelAttachmentOHs[i]);
	}	

	//Physx rigid actor
	destroyObject(ow, ch, oh.physxRoadGeomQueryParamOH);
	destroyObject(ow, ch, oh.physxRigidActorOH);

	//Free the memory.
	allocator.deallocate(&oh);
}

#else //#if PX_SUPPORT_OMNI_PVD

PxVehiclePvdAttributeHandles* PxVehiclePvdAttributesCreate(PxAllocatorCallback& allocator, OmniPvdWriter& omniWriter)
{
	PX_UNUSED(allocator);
	PX_UNUSED(omniWriter);
	return NULL;
}

void PxVehiclePvdAttributesRelease(PxAllocatorCallback& allocator, PxVehiclePvdAttributeHandles& attributeHandles)
{
	PX_UNUSED(allocator);
	PX_UNUSED(attributeHandles);
}

PxVehiclePvdObjectHandles* PxVehiclePvdObjectCreate
(const PxU32 nbWheels, const PxU32 nbAntirolls, const PxU32 maxNbPhysXMaterialFrictions,
 const PxU64 contextHandle,
 PxAllocatorCallback& allocator)
{
	PX_UNUSED(nbWheels);
	PX_UNUSED(nbAntirolls);
	PX_UNUSED(maxNbPhysXMaterialFrictions);
	PX_UNUSED(contextHandle);
	PX_UNUSED(allocator);
	return NULL;
}

void PxVehiclePvdObjectRelease
(OmniPvdWriter& ow, PxAllocatorCallback& allocator, PxVehiclePvdObjectHandles& oh)
{
	PX_UNUSED(ow);
	PX_UNUSED(allocator);
	PX_UNUSED(oh);
}

#endif //#if PX_SUPPORT_OMNI_PVD


} // namespace vehicle2
} // namespace physx

/** @} */
