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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#include "VhPvdWriter.h"

namespace physx
{
namespace vehicle2
{

#if PX_SUPPORT_OMNI_PVD

PX_FORCE_INLINE void writeFloatAttribute
(OmniPvdWriter* omniWriter, OmniPvdContextHandle ch, OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, float val)
{
	omniWriter->setAttributeShallow(ch, oh, ah, reinterpret_cast<uint8_t*>(&val), sizeof(float));
}

PX_FORCE_INLINE void writeFloatArrayAttribute
(OmniPvdWriter* omniWriter, OmniPvdContextHandle ch, OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, const float* val, const PxU32 nbVals)
{
	omniWriter->setAttributeShallow(ch, oh, ah, reinterpret_cast<const uint8_t*>(val), sizeof(float) * nbVals);
}

PX_FORCE_INLINE void writeUInt32ArrayAttribute
(OmniPvdWriter* omniWriter, OmniPvdContextHandle ch, OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, const uint32_t* val, const PxU32 nbVals)
{
	omniWriter->setAttributeShallow(ch, oh, ah, reinterpret_cast<const uint8_t*>(val), sizeof(uint32_t) * nbVals);
}


PX_FORCE_INLINE void writeVec3Attribute
(OmniPvdWriter* omniWriter, OmniPvdContextHandle ch, 
 OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, const PxVec3& val)
{
	omniWriter->setAttributeShallow(ch, oh, ah, reinterpret_cast<const uint8_t*>(&val), sizeof(PxVec3));
}

PX_FORCE_INLINE void writePlaneAttribute
(OmniPvdWriter* omniWriter, OmniPvdContextHandle ch, OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, const PxPlane& val)
{
	omniWriter->setAttributeShallow(ch, oh, ah, reinterpret_cast<const uint8_t*>(&val), sizeof(PxPlane));
}


PX_FORCE_INLINE void writeQuatAttribute
(OmniPvdWriter* omniWriter, OmniPvdContextHandle ch, OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, const PxQuat& val)
{
	omniWriter->setAttributeShallow(ch, oh, ah, reinterpret_cast<const uint8_t*>(&val), sizeof(PxQuat));
}

PX_FORCE_INLINE void writeUInt8Attribute
(OmniPvdWriter* omniWriter, OmniPvdContextHandle ch, OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, uint8_t val)
{
	omniWriter->setAttributeShallow(ch, oh, ah, reinterpret_cast<const uint8_t*>(&val), sizeof(uint8_t));
}

PX_FORCE_INLINE void writeUInt32Attribute
(OmniPvdWriter* omniWriter, OmniPvdContextHandle ch, OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, uint32_t val)
{
	omniWriter->setAttributeShallow(ch, oh, ah, reinterpret_cast<const uint8_t*>(&val), sizeof(uint32_t));
}

PX_FORCE_INLINE void writeFlagAttribute
(OmniPvdWriter* omniWriter, OmniPvdContextHandle ch, OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, uint32_t val)
{
	writeUInt32Attribute(omniWriter, ch, oh, ah, val);
}

PX_FORCE_INLINE void writeLookupTableAttribute
(OmniPvdWriter* omniWriter, OmniPvdContextHandle ch, 
 OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, PxVehicleFixedSizeLookupTable<float, 3> val)
{
	float buffer[6] = { -1.0f, 0.0f, -1.0f, 0.0f, -1.0f, 0.0f };
	for(PxU32 i = 0; i < val.nbDataPairs; i++)
	{
		buffer[2 * i + 0] = val.xVals[i];
		buffer[2 * i + 1] = val.yVals[i];
	}
	omniWriter->setAttributeShallow(ch, oh, ah, reinterpret_cast<const uint8_t*>(buffer), sizeof(buffer));
}

PX_FORCE_INLINE void writeLookupTableAttribute
(OmniPvdWriter* omniWriter, OmniPvdContextHandle ch, 
 OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, PxVehicleFixedSizeLookupTable<PxVec3, 3> val)
{
	float buffer[12] = { -1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f };
	for(PxU32 i = 0; i < val.nbDataPairs; i++)
	{
		buffer[4 * i + 0] = val.xVals[i];
		buffer[4 * i + 1] = val.yVals[i].x;
		buffer[4 * i + 2] = val.yVals[i].y;
		buffer[4 * i + 3] = val.yVals[i].z;
	}
	omniWriter->setAttributeShallow(ch, oh, ah, reinterpret_cast<const uint8_t*>(buffer), sizeof(buffer));
}

void writePtrAttribute
(OmniPvdWriter* omniWriter, OmniPvdContextHandle ch, 
 OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, const void* val)
{
	omniWriter->setAttributeShallow(ch, oh, ah, reinterpret_cast<const uint8_t*>(&val), sizeof(val));
}

////////////////////////////////
//RIGID BODY
////////////////////////////////

RigidBodyParams registerRigidBodyParams(OmniPvdWriter* omniWriter)
{
	RigidBodyParams r;
	r.CH = omniWriter->registerClass("RigidBodyParams");
	r.massAH = omniWriter->registerAttribute(r.CH, "mass", OmniPvdDataTypeEnum::eFLOAT32, 1);
	r.moiAH = omniWriter->registerAttribute(r.CH, "moi", OmniPvdDataTypeEnum::eFLOAT32, 3);
	return r;
}

void writeRigidBodyParams
(const PxVehicleRigidBodyParams& rbodyParams,
 const OmniPvdObjectHandle oh, const RigidBodyParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.massAH, rbodyParams.mass);
	writeVec3Attribute(omniWriter, ch, oh, ah.moiAH, rbodyParams.moi);
}

RigidBodyState registerRigidBodyState(OmniPvdWriter* omniWriter)
{
	RigidBodyState r;
	r.CH = omniWriter->registerClass("RigidBodyState");
	r.posAH = omniWriter->registerAttribute(r.CH, "pos", OmniPvdDataTypeEnum::eFLOAT32, 3);
	r.quatAH = omniWriter->registerAttribute(r.CH, "quat", OmniPvdDataTypeEnum::eFLOAT32, 4);
	r.linearVelocityAH = omniWriter->registerAttribute(r.CH, "linvel", OmniPvdDataTypeEnum::eFLOAT32, 3);
	r.angularVelocityAH = omniWriter->registerAttribute(r.CH, "angvel", OmniPvdDataTypeEnum::eFLOAT32, 3);
	r.previousLinearVelocityAH = omniWriter->registerAttribute(r.CH, "prevLinvel", OmniPvdDataTypeEnum::eFLOAT32, 3);
	r.previousAngularVelocityAH = omniWriter->registerAttribute(r.CH, "prevAngvel", OmniPvdDataTypeEnum::eFLOAT32, 3);
	r.externalForceAH = omniWriter->registerAttribute(r.CH, "extForce", OmniPvdDataTypeEnum::eFLOAT32, 3);
	r.externalTorqueAH = omniWriter->registerAttribute(r.CH, "extTorque", OmniPvdDataTypeEnum::eFLOAT32, 3);
	return r;
}

void writeRigidBodyState
(const PxVehicleRigidBodyState& rbodyState,
 const OmniPvdObjectHandle oh, const RigidBodyState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeVec3Attribute(omniWriter, ch, oh, ah.posAH, rbodyState.pose.p);
	writeQuatAttribute(omniWriter, ch, oh, ah.quatAH, rbodyState.pose.q);
	writeVec3Attribute(omniWriter, ch, oh, ah.linearVelocityAH, rbodyState.linearVelocity);
	writeVec3Attribute(omniWriter, ch, oh, ah.angularVelocityAH, rbodyState.angularVelocity);
	writeVec3Attribute(omniWriter, ch, oh, ah.previousLinearVelocityAH, rbodyState.previousLinearVelocity);
	writeVec3Attribute(omniWriter, ch, oh, ah.previousAngularVelocityAH, rbodyState.previousAngularVelocity);
	writeVec3Attribute(omniWriter, ch, oh, ah.externalForceAH, rbodyState.externalForce);
	writeVec3Attribute(omniWriter, ch, oh, ah.externalTorqueAH, rbodyState.externalTorque);
}




/////////////////////////////////
//CONTROLS
/////////////////////////////////

WheelResponseParams registerWheelResponseParams(const char* name, OmniPvdWriter* omniWriter)
{
	WheelResponseParams w;
	w.CH = omniWriter->registerClass(name);
	w.maxResponseAH = omniWriter->registerAttribute(w.CH, "maxResponse", OmniPvdDataTypeEnum::eFLOAT32, 1);
	w.responseMultipliers0To3AH = omniWriter->registerAttribute(w.CH, "wheelMultipliers0To3", OmniPvdDataTypeEnum::eFLOAT32, 4);
	w.responseMultipliers4To7AH = omniWriter->registerAttribute(w.CH, "wheelMultipliers4To7", OmniPvdDataTypeEnum::eFLOAT32, 4);
	w.responseMultipliers8To11AH = omniWriter->registerAttribute(w.CH, "wheelMultipliers8To11", OmniPvdDataTypeEnum::eFLOAT32, 4);
	w.responseMultipliers12To15AH = omniWriter->registerAttribute(w.CH, "wheelMultipliers12To15", OmniPvdDataTypeEnum::eFLOAT32, 4);
	w.responseMultipliers16To19AH = omniWriter->registerAttribute(w.CH, "wheelMultipliers16To19", OmniPvdDataTypeEnum::eFLOAT32, 4);
	return w;
}

WheelResponseParams registerSteerResponseParams(OmniPvdWriter* omniWriter)
{
	return registerWheelResponseParams("SteerResponseParams", omniWriter);
}

WheelResponseParams registerBrakeResponseParams(OmniPvdWriter* omniWriter)
{
	return registerWheelResponseParams("BrakeResponseParams", omniWriter);
}

WheelResponseStates registerWheelResponseStates(const char* name, OmniPvdWriter* omniWriter)
{
	WheelResponseStates w;
	w.CH = omniWriter->registerClass(name);
	w.responseStates0To3AH = omniWriter->registerAttribute(w.CH, "wheelStates0To3", OmniPvdDataTypeEnum::eFLOAT32, 4);
	w.responseStates4To7AH = omniWriter->registerAttribute(w.CH, "wheelStatess4To7", OmniPvdDataTypeEnum::eFLOAT32, 4);
	w.responseStates8To11AH = omniWriter->registerAttribute(w.CH, "wheelStates8To11", OmniPvdDataTypeEnum::eFLOAT32, 4);
	w.responseStates12To15AH = omniWriter->registerAttribute(w.CH, "wheelStates12To15", OmniPvdDataTypeEnum::eFLOAT32, 4);
	w.responseStates16To19AH = omniWriter->registerAttribute(w.CH, "wheelStates16To19", OmniPvdDataTypeEnum::eFLOAT32, 4);
	return w;
}

WheelResponseStates registerSteerResponseStates(OmniPvdWriter* omniWriter)
{
	return registerWheelResponseStates("SteerResponseState", omniWriter);
}

WheelResponseStates registerBrakeResponseStates(OmniPvdWriter* omniWriter)
{
	return registerWheelResponseStates("BrakeResponseState", omniWriter);
}

void writeWheelResponseParams
(const PxVehicleAxleDescription& axleDesc, const PxVehicleCommandResponseParams& responseParams,
 const OmniPvdObjectHandle oh, const WheelResponseParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.maxResponseAH, responseParams.maxResponse);

	float responseMultipliers[PxVehicleLimits::eMAX_NB_WHEELS];
	for(PxU32 i = 0; i < PxVehicleLimits::eMAX_NB_WHEELS; i++)
	{
		responseMultipliers[i] = PX_MAX_F32;
	}
	for(PxU32 i = 0; i < axleDesc.nbWheels; i++)
	{
		const PxU32 wheelId = axleDesc.wheelIdsInAxleOrder[i];
		responseMultipliers[wheelId] = responseParams.wheelResponseMultipliers[wheelId];
	}
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.responseMultipliers0To3AH, responseMultipliers + 0, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.responseMultipliers4To7AH, responseMultipliers + 4, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.responseMultipliers8To11AH, responseMultipliers + 8, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.responseMultipliers12To15AH, responseMultipliers + 12, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.responseMultipliers16To19AH, responseMultipliers + 16, 4);
}

void writeSteerResponseParams
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleSteerCommandResponseParams& steerResponseParams,
 const OmniPvdObjectHandle oh,  const WheelResponseParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeWheelResponseParams(axleDesc, steerResponseParams, oh, ah, omniWriter, ch);
}

void writeBrakeResponseParams
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleBrakeCommandResponseParams& brakeResponseParams,
 const OmniPvdObjectHandle oh,  const WheelResponseParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeWheelResponseParams(axleDesc, brakeResponseParams, oh, ah, omniWriter, ch);
}

void writeWheelResponseStates
(const PxVehicleAxleDescription& axleDesc, const PxVehicleArrayData<PxReal>& responseState,
 const OmniPvdObjectHandle oh, const WheelResponseStates& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	float responseStates[PxVehicleLimits::eMAX_NB_WHEELS];
	PxMemZero(responseStates, sizeof(responseStates));
	for(PxU32 i = 0; i < axleDesc.nbWheels; i++)
	{
		const PxU32 wheelId = axleDesc.wheelIdsInAxleOrder[i];
		responseStates[wheelId] = responseState[wheelId];
	}
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.responseStates0To3AH, responseStates + 0, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.responseStates4To7AH, responseStates + 4, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.responseStates8To11AH, responseStates + 8, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.responseStates12To15AH, responseStates + 12, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.responseStates16To19AH, responseStates + 16, 4);
}

void writeSteerResponseStates
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleArrayData<PxReal>& steerResponseStates, 
 const OmniPvdObjectHandle oh, const WheelResponseStates& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeWheelResponseStates(axleDesc, steerResponseStates, oh, ah, omniWriter, ch);
}

void writeBrakeResponseStates
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleArrayData<PxReal>& brakeResponseStates,
 const OmniPvdObjectHandle oh, const WheelResponseStates& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeWheelResponseStates(axleDesc, brakeResponseStates, oh, ah, omniWriter, ch);
}


void writeClutchResponseParams
(const PxVehicleClutchCommandResponseParams& clutchResponseParams, 
 const OmniPvdObjectHandle oh, const ClutchResponseParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.maxResponseAH, clutchResponseParams.maxResponse);
}

//////////////////////////////
//WHEEL ATTACHMENTS
//////////////////////////////

WheelParams registerWheelParams(OmniPvdWriter* omniWriter)
{
	WheelParams w;
	w.CH = omniWriter->registerClass("WheelParams");
	w.wheelRadiusAH = omniWriter->registerAttribute(w.CH, "radius", OmniPvdDataTypeEnum::eFLOAT32, 1);
	w.halfWidthAH = omniWriter->registerAttribute(w.CH, "halfWidth", OmniPvdDataTypeEnum::eFLOAT32, 1);
	w.massAH = omniWriter->registerAttribute(w.CH, "mass", OmniPvdDataTypeEnum::eFLOAT32, 1);
	w.moiAH = omniWriter->registerAttribute(w.CH, "moi", OmniPvdDataTypeEnum::eFLOAT32, 1);
	w.dampingRateAH = omniWriter->registerAttribute(w.CH, "dampingRate", OmniPvdDataTypeEnum::eFLOAT32, 1);
	return w;
}

void writeWheelParams
(const PxVehicleWheelParams& params, 
 const OmniPvdObjectHandle oh, const WheelParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.dampingRateAH, params.dampingRate);
	writeFloatAttribute(omniWriter, ch, oh, ah.halfWidthAH, params.halfWidth);
	writeFloatAttribute(omniWriter, ch, oh, ah.massAH, params.mass);
	writeFloatAttribute(omniWriter, ch, oh, ah.moiAH, params.moi);
	writeFloatAttribute(omniWriter, ch, oh, ah.wheelRadiusAH, params.radius);
}

WheelActuationState registerWheelActuationState(OmniPvdWriter* omniWriter)
{
	struct BoolAsEnum
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle falseAH;
		OmniPvdAttributeHandle trueAH;
	};
	BoolAsEnum boolAsEnum;
	boolAsEnum.CH = omniWriter->registerClass("WheelStateBool");
	boolAsEnum.falseAH = omniWriter->registerEnumValue(boolAsEnum.CH, "False", 0);
	boolAsEnum.trueAH = omniWriter->registerEnumValue(boolAsEnum.CH, "True", 1);


	WheelActuationState w;
	w.CH = omniWriter->registerClass("WheelActuationState");
	w.isBrakeAppliedAH = omniWriter->registerFlagsAttribute(w.CH, boolAsEnum.CH, "isBrakeApplied");
	w.isDriveAppliedAH = omniWriter->registerFlagsAttribute(w.CH, boolAsEnum.CH, "isDriveApplied");

	return w;
}

void writeWheelActuationState
(const PxVehicleWheelActuationState& actState,
 const OmniPvdObjectHandle oh, const WheelActuationState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFlagAttribute(omniWriter, ch, oh, ah.isBrakeAppliedAH, actState.isBrakeApplied ? 1 : 0);
	writeFlagAttribute(omniWriter, ch, oh, ah.isDriveAppliedAH, actState.isDriveApplied ? 1 : 0);
}

WheelRigidBody1dState registerWheelRigidBody1dState(OmniPvdWriter* omniWriter)
{
	WheelRigidBody1dState w;
	w.CH = omniWriter->registerClass("WheelRigidBodyState");
	w.rotationSpeedAH = omniWriter->registerAttribute(w.CH, "rotationSpeed", OmniPvdDataTypeEnum::eFLOAT32, 1);
	w.correctedRotationSpeedAH = omniWriter->registerAttribute(w.CH, "correctedRotationSpeed", OmniPvdDataTypeEnum::eFLOAT32, 1);
	w.rotationAngleAH = omniWriter->registerAttribute(w.CH, "rotationAngle", OmniPvdDataTypeEnum::eFLOAT32, 1);
	return w;
}

void writeWheelRigidBody1dState
(const PxVehicleWheelRigidBody1dState& rigidBodyState,
 const OmniPvdObjectHandle oh, const WheelRigidBody1dState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.rotationSpeedAH, rigidBodyState.rotationSpeed);
	writeFloatAttribute(omniWriter, ch, oh, ah.correctedRotationSpeedAH, rigidBodyState.correctedRotationSpeed);
	writeFloatAttribute(omniWriter, ch, oh, ah.rotationAngleAH, rigidBodyState.rotationAngle);
}

WheelLocalPoseState registerWheelLocalPoseState(OmniPvdWriter* omniWriter)
{
	WheelLocalPoseState w;
	w.CH = omniWriter->registerClass("WheelLocalPoseState");
	w.posAH = omniWriter->registerAttribute(w.CH, "posInRbodyFrame", OmniPvdDataTypeEnum::eFLOAT32, 3);
	w.quatAH = omniWriter->registerAttribute(w.CH, "quatInRbodyFrame", OmniPvdDataTypeEnum::eFLOAT32, 4);
	return w;
}

void writeWheelLocalPoseState
(const PxVehicleWheelLocalPose& pose,
 const OmniPvdObjectHandle oh, const WheelLocalPoseState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeVec3Attribute(omniWriter, ch, oh, ah.posAH, pose.localPose.p);
	writeQuatAttribute(omniWriter, ch, oh, ah.quatAH, pose.localPose.q);
}

RoadGeometryState registerRoadGeomState(OmniPvdWriter* omniWriter)
{
	struct BoolAsEnum
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle falseAH;
		OmniPvdAttributeHandle trueAH;
	};
	BoolAsEnum boolAsEnum;
	boolAsEnum.CH = omniWriter->registerClass("RoadGeomStateBool");
	boolAsEnum.falseAH = omniWriter->registerEnumValue(boolAsEnum.CH, "False", 0);
	boolAsEnum.trueAH = omniWriter->registerEnumValue(boolAsEnum.CH, "True", 1);

	RoadGeometryState r;
	r.CH = omniWriter->registerClass("RoadGeometryState");
	r.planeAH = omniWriter->registerAttribute(r.CH, "plane", OmniPvdDataTypeEnum::eFLOAT32, 4);
	r.frictionAH = omniWriter->registerAttribute(r.CH, "friction", OmniPvdDataTypeEnum::eFLOAT32, 1);
	r.hitStateAH = omniWriter->registerFlagsAttribute(r.CH, boolAsEnum.CH, "hitState");
	r.velocityAH = omniWriter->registerAttribute(r.CH,  "hitVelocity", OmniPvdDataTypeEnum::eFLOAT32, 3);
	return r;
}

void writeRoadGeomState
(const PxVehicleRoadGeometryState& roadGeometryState,
 const OmniPvdObjectHandle oh, const RoadGeometryState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writePlaneAttribute(omniWriter, ch, oh, ah.planeAH, roadGeometryState.plane);
	writeFloatAttribute(omniWriter, ch, oh, ah.frictionAH, roadGeometryState.friction);
	writeFlagAttribute(omniWriter, ch, oh, ah.hitStateAH, roadGeometryState.hitState);
	writeVec3Attribute(omniWriter, ch, oh, ah.velocityAH, roadGeometryState.velocity);
}

SuspParams registerSuspParams(OmniPvdWriter* omniWriter)
{
	SuspParams s;
	s.CH = omniWriter->registerClass("SuspensionParams");
	s.suspAttachmentPosAH = omniWriter->registerAttribute(s.CH, "suspAttachmentPos", OmniPvdDataTypeEnum::eFLOAT32, 3);
	s.suspAttachmentQuatAH = omniWriter->registerAttribute(s.CH, "suspAttachmentQuat", OmniPvdDataTypeEnum::eFLOAT32, 4);
	s.suspDirAH = omniWriter->registerAttribute(s.CH, "suspDir", OmniPvdDataTypeEnum::eFLOAT32, 3);
	s.suspTravleDistAH = omniWriter->registerAttribute(s.CH, "suspTravelDist", OmniPvdDataTypeEnum::eFLOAT32, 1);
	s.wheelAttachmentPosAH = omniWriter->registerAttribute(s.CH, "wheelAttachmentPos", OmniPvdDataTypeEnum::eFLOAT32, 3);
	s.wheelAttachmentQuatAH = omniWriter->registerAttribute(s.CH, "wheelAttachmentQuat", OmniPvdDataTypeEnum::eFLOAT32, 3);
	return s;
}

void writeSuspParams
(const PxVehicleSuspensionParams& suspParams, 
 const OmniPvdObjectHandle oh, const SuspParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeVec3Attribute(omniWriter, ch, oh, ah.suspAttachmentPosAH, suspParams.suspensionAttachment.p);
	writeQuatAttribute(omniWriter, ch, oh, ah.suspAttachmentQuatAH, suspParams.suspensionAttachment.q);
	writeVec3Attribute(omniWriter, ch, oh, ah.suspDirAH, suspParams.suspensionTravelDir);
	writeFloatAttribute(omniWriter, ch, oh, ah.suspTravleDistAH, suspParams.suspensionTravelDist);
	writeVec3Attribute(omniWriter, ch, oh, ah.wheelAttachmentPosAH, suspParams.wheelAttachment.p);
	writeQuatAttribute(omniWriter, ch, oh, ah.wheelAttachmentQuatAH, suspParams.wheelAttachment.q);
}


SuspCompParams registerSuspComplianceParams(OmniPvdWriter* omniWriter)
{
	SuspCompParams s;
	s.CH = omniWriter->registerClass("SuspensionComplianceParams");
	s.toeAngleAH = omniWriter->registerAttribute(s.CH, "toeAngle", OmniPvdDataTypeEnum::eFLOAT32, 6);
	s.camberAngleAH = omniWriter->registerAttribute(s.CH, "camberAngle", OmniPvdDataTypeEnum::eFLOAT32, 6);
	s.suspForceAppPointAH = omniWriter->registerAttribute(s.CH, "suspForceAppPoint", OmniPvdDataTypeEnum::eFLOAT32, 12);
	s.tireForceAppPointAH = omniWriter->registerAttribute(s.CH, "tireForceAppPoint", OmniPvdDataTypeEnum::eFLOAT32, 12);
	return s;
}

void writeSuspComplianceParams
(const PxVehicleSuspensionComplianceParams& compParams, 
 const OmniPvdObjectHandle oh, const SuspCompParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeLookupTableAttribute(omniWriter, ch, oh, ah.camberAngleAH, compParams.wheelCamberAngle);
	writeLookupTableAttribute(omniWriter, ch, oh, ah.toeAngleAH, compParams.wheelToeAngle);
	writeLookupTableAttribute(omniWriter, ch, oh, ah.suspForceAppPointAH, compParams.suspForceAppPoint);
	writeLookupTableAttribute(omniWriter, ch, oh, ah.tireForceAppPointAH, compParams.tireForceAppPoint);
}

SuspForceParams registerSuspForceParams(OmniPvdWriter* omniWriter)
{
	SuspForceParams s;
	s.CH = omniWriter->registerClass("SuspensionForceParams");
	s.stiffnessAH = omniWriter->registerAttribute(s.CH, "stiffness", OmniPvdDataTypeEnum::eFLOAT32, 1);
	s.dampingAH = omniWriter->registerAttribute(s.CH, "damping", OmniPvdDataTypeEnum::eFLOAT32, 1);
	s.sprungMassAH = omniWriter->registerAttribute(s.CH, "sprungMass", OmniPvdDataTypeEnum::eFLOAT32, 1);
	return s;
}

void writeSuspForceParams
(const PxVehicleSuspensionForceParams& forceParams, 
 const OmniPvdObjectHandle oh, const SuspForceParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.dampingAH, forceParams.damping);
	writeFloatAttribute(omniWriter, ch, oh, ah.sprungMassAH, forceParams.sprungMass);
	writeFloatAttribute(omniWriter, ch, oh, ah.stiffnessAH, forceParams.stiffness);
}

SuspState registerSuspState(OmniPvdWriter* omniWriter)
{
	SuspState s;
	s.CH = omniWriter->registerClass("SuspensionState");
	s.jounceAH = omniWriter->registerAttribute(s.CH, "jounce", OmniPvdDataTypeEnum::eFLOAT32, 1);
	s.jounceSpeedAH = omniWriter->registerAttribute(s.CH, "jounceSpeed", OmniPvdDataTypeEnum::eFLOAT32, 1);
	s.separationAH = omniWriter->registerAttribute(s.CH, "separation", OmniPvdDataTypeEnum::eFLOAT32, 1);
	return s;
}

void writeSuspState
(const PxVehicleSuspensionState& suspState, 
 const OmniPvdObjectHandle oh, const SuspState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.jounceAH, suspState.jounce);
	writeFloatAttribute(omniWriter, ch, oh, ah.jounceSpeedAH, suspState.jounceSpeed);
	writeFloatAttribute(omniWriter, ch, oh, ah.separationAH, suspState.separation);									
}

SuspCompState registerSuspComplianceState(OmniPvdWriter* omniWriter)
{
	SuspCompState s;
	s.CH = omniWriter->registerClass("SuspensionComplianceState");
	s.toeAH = omniWriter->registerAttribute(s.CH, "toe", OmniPvdDataTypeEnum::eFLOAT32, 1);
	s.camberAH = omniWriter->registerAttribute(s.CH, "camber", OmniPvdDataTypeEnum::eFLOAT32, 1);
	s.tireForceAppPointAH = omniWriter->registerAttribute(s.CH, "tireForceAppPoint", OmniPvdDataTypeEnum::eFLOAT32, 3);
	s.suspForceAppPointAH = omniWriter->registerAttribute(s.CH, "suspForceAppPoint", OmniPvdDataTypeEnum::eFLOAT32, 3);
	return s;
}

void writeSuspComplianceState
(const PxVehicleSuspensionComplianceState& suspCompState, 
 const OmniPvdObjectHandle oh, const SuspCompState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.camberAH, suspCompState.camber);
	writeFloatAttribute(omniWriter, ch, oh, ah.toeAH, suspCompState.toe);
	writeVec3Attribute(omniWriter, ch, oh, ah.suspForceAppPointAH, suspCompState.suspForceAppPoint);
	writeVec3Attribute(omniWriter, ch, oh, ah.tireForceAppPointAH, suspCompState.tireForceAppPoint);
}

SuspForce registerSuspForce(OmniPvdWriter* omniWriter)
{
	SuspForce s;
	s.CH = omniWriter->registerClass("SuspensionForce");
	s.forceAH = omniWriter->registerAttribute(s.CH, "force", OmniPvdDataTypeEnum::eFLOAT32, 3);
	s.torqueAH = omniWriter->registerAttribute(s.CH, "torque", OmniPvdDataTypeEnum::eFLOAT32, 3);
	s.normalForceAH = omniWriter->registerAttribute(s.CH, "normalForce", OmniPvdDataTypeEnum::eFLOAT32, 3);
	return s;
}

void writeSuspForce
(const PxVehicleSuspensionForce& suspForce, 
 const OmniPvdObjectHandle oh, const SuspForce& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeVec3Attribute(omniWriter, ch, oh, ah.forceAH, suspForce.force);
	writeVec3Attribute(omniWriter, ch, oh, ah.torqueAH, suspForce.torque);
	writeFloatAttribute(omniWriter, ch, oh, ah.normalForceAH, suspForce.normalForce);
}

TireParams registerTireParams(OmniPvdWriter* omniWriter)
{
	TireParams t;
	t.CH = omniWriter->registerClass("TireParams");
	t.latStiffXAH = omniWriter->registerAttribute(t.CH, "latStiffX", OmniPvdDataTypeEnum::eFLOAT32, 1);
	t.latStiffYAH = omniWriter->registerAttribute(t.CH, "latStiffY", OmniPvdDataTypeEnum::eFLOAT32, 1);
	t.longStiffAH = omniWriter->registerAttribute(t.CH, "longStiff", OmniPvdDataTypeEnum::eFLOAT32, 1);
	t.camberStiffAH = omniWriter->registerAttribute(t.CH, "camberStiff", OmniPvdDataTypeEnum::eFLOAT32, 1);
	t.frictionVsSlipAH = omniWriter->registerAttribute(t.CH, "frictionVsSlip", OmniPvdDataTypeEnum::eFLOAT32, 6);
	t.restLoadAH = omniWriter->registerAttribute(t.CH, "restLoad", OmniPvdDataTypeEnum::eFLOAT32, 1);
	t.loadFilterAH = omniWriter->registerAttribute(t.CH, "loadFilter", OmniPvdDataTypeEnum::eFLOAT32, 4);
	return t;
}

void writeTireParams
(const PxVehicleTireForceParams& tireParams, 
 const OmniPvdObjectHandle oh, const TireParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.latStiffXAH, tireParams.latStiffX);
	writeFloatAttribute(omniWriter, ch, oh, ah.latStiffYAH, tireParams.latStiffY);
	writeFloatAttribute(omniWriter, ch, oh, ah.longStiffAH, tireParams.longStiff);
	writeFloatAttribute(omniWriter, ch, oh, ah.camberStiffAH, tireParams.camberStiff);
	writeFloatAttribute(omniWriter, ch, oh, ah.restLoadAH, tireParams.restLoad);
	const float fricVsSlip[6] = 
	{
		tireParams.frictionVsSlip[0][0], tireParams.frictionVsSlip[0][1], tireParams.frictionVsSlip[1][0],
		tireParams.frictionVsSlip[1][1], tireParams.frictionVsSlip[2][0], tireParams.frictionVsSlip[2][1],
	};
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.frictionVsSlipAH, fricVsSlip, 6);
	const float loadFilter[4] = 
	{ 
		tireParams.loadFilter[0][0], tireParams.loadFilter[0][1], 
		tireParams.loadFilter[1][0], tireParams.loadFilter[1][1] 
	};
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.loadFilterAH, loadFilter, 4);
}

TireDirectionState registerTireDirectionState(OmniPvdWriter* omniWriter)
{
	TireDirectionState t;
	t.CH = omniWriter->registerClass("TireDirectionState");
	t.lngDirectionAH = omniWriter->registerAttribute(t.CH, "lngDir", OmniPvdDataTypeEnum::eFLOAT32, 3);
	t.latDirectionAH = omniWriter->registerAttribute(t.CH, "latDir", OmniPvdDataTypeEnum::eFLOAT32, 3);
	return t;
}

void writeTireDirectionState
(const PxVehicleTireDirectionState& tireDirState,
 const OmniPvdObjectHandle oh, const TireDirectionState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeVec3Attribute(omniWriter, ch, oh, ah.lngDirectionAH, tireDirState.directions[PxVehicleTireDirectionModes::eLONGITUDINAL]);
	writeVec3Attribute(omniWriter, ch, oh, ah.latDirectionAH, tireDirState.directions[PxVehicleTireDirectionModes::eLATERAL]);
}

TireSpeedState registerTireSpeedState(OmniPvdWriter* omniWriter)
{
	TireSpeedState t;
	t.CH = omniWriter->registerClass("TireSpeedState");
	t.lngSpeedAH = omniWriter->registerAttribute(t.CH, "lngSpeed", OmniPvdDataTypeEnum::eFLOAT32, 1);
	t.latSpeedAH = omniWriter->registerAttribute(t.CH, "latSpeed", OmniPvdDataTypeEnum::eFLOAT32, 1);
	return t;
}

void writeTireSpeedState
(const PxVehicleTireSpeedState& tireSpeedState,
 const OmniPvdObjectHandle oh, const TireSpeedState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.lngSpeedAH, tireSpeedState.speedStates[PxVehicleTireDirectionModes::eLONGITUDINAL]);
	writeFloatAttribute(omniWriter, ch, oh, ah.latSpeedAH, tireSpeedState.speedStates[PxVehicleTireDirectionModes::eLATERAL]);
}

TireSlipState registerTireSlipState(OmniPvdWriter* omniWriter)
{
	TireSlipState t;
	t.CH = omniWriter->registerClass("TireSlipState");
	t.lngSlipAH = omniWriter->registerAttribute(t.CH, "lngSlip", OmniPvdDataTypeEnum::eFLOAT32, 1);
	t.latSlipAH = omniWriter->registerAttribute(t.CH, "latSlip", OmniPvdDataTypeEnum::eFLOAT32, 1);
	return t;
}

void writeTireSlipState
(const PxVehicleTireSlipState& tireSlipState,
 const OmniPvdObjectHandle oh, const TireSlipState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.lngSlipAH, tireSlipState.slips[PxVehicleTireDirectionModes::eLONGITUDINAL]);
	writeFloatAttribute(omniWriter, ch, oh, ah.latSlipAH, tireSlipState.slips[PxVehicleTireDirectionModes::eLATERAL]);
}

TireStickyState registerTireStickyState(OmniPvdWriter* omniWriter)
{
	struct BoolAsEnum
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle falseAH;
		OmniPvdAttributeHandle trueAH;
	};
	BoolAsEnum boolAsEnum;
	boolAsEnum.CH = omniWriter->registerClass("StickyTireBool");
	boolAsEnum.falseAH = omniWriter->registerEnumValue(boolAsEnum.CH, "False", 0);
	boolAsEnum.trueAH = omniWriter->registerEnumValue(boolAsEnum.CH, "True", 1);

	TireStickyState t;
	t.CH = omniWriter->registerClass("TireStickyState");
	t.lngStickyStateTimer = omniWriter->registerAttribute(t.CH, "lngStickyTimer", OmniPvdDataTypeEnum::eFLOAT32, 1);
	t.lngStickyStateStatus = omniWriter->registerFlagsAttribute(t.CH, boolAsEnum.CH, "lngStickyStatus");
	t.latStickyStateTimer = omniWriter->registerAttribute(t.CH, "latStickyTimer", OmniPvdDataTypeEnum::eFLOAT32, 1);
	t.latStickyStateStatus = omniWriter->registerFlagsAttribute(t.CH, boolAsEnum.CH, "latStickyStatus");
	return t;
}

void writeTireStickyState
(const PxVehicleTireStickyState& tireStickyState,
 const OmniPvdObjectHandle oh, const TireStickyState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFlagAttribute(omniWriter, ch, oh, ah.latStickyStateStatus, tireStickyState.activeStatus[PxVehicleTireDirectionModes::eLATERAL] ? 1 : 0);
	writeFlagAttribute(omniWriter, ch, oh, ah.lngStickyStateStatus, tireStickyState.activeStatus[PxVehicleTireDirectionModes::eLONGITUDINAL] ? 1 : 0);
	writeFloatAttribute(omniWriter, ch, oh, ah.latStickyStateTimer, tireStickyState.lowSpeedTime[PxVehicleTireDirectionModes::eLATERAL]);
	writeFloatAttribute(omniWriter, ch, oh, ah.lngStickyStateTimer, tireStickyState.lowSpeedTime[PxVehicleTireDirectionModes::eLONGITUDINAL]);
}

TireGripState registerTireGripState(OmniPvdWriter* omniWriter)
{
	TireGripState t;
	t.CH = omniWriter->registerClass("TireGripState");
	t.loadAH = omniWriter->registerAttribute(t.CH, "load", OmniPvdDataTypeEnum::eFLOAT32, 1);
	t.frictionAH = omniWriter->registerAttribute(t.CH, "friction", OmniPvdDataTypeEnum::eFLOAT32, 1);
	return t;
}

void writeTireGripState
(const PxVehicleTireGripState& tireGripState, 
 const OmniPvdObjectHandle oh, const TireGripState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.frictionAH, tireGripState.friction);
	writeFloatAttribute(omniWriter, ch, oh, ah.loadAH, tireGripState.load);
}

TireCamberState registerTireCamberState(OmniPvdWriter* omniWriter)
{
	TireCamberState t;
	t.CH = omniWriter->registerClass("TireCamberState");
	t.camberAngleAH = omniWriter->registerAttribute(t.CH, "camberAngle", OmniPvdDataTypeEnum::eFLOAT32, 1);
	return t;
}

void writeTireCamberState
(const PxVehicleTireCamberAngleState& tireCamberState,
 const OmniPvdObjectHandle oh, const TireCamberState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.camberAngleAH, tireCamberState.camberAngle);
}

TireForce registerTireForce(OmniPvdWriter* omniWriter)
{
	TireForce t;
	t.CH = omniWriter->registerClass("TireForce");

	t.lngForceAH = omniWriter->registerAttribute(t.CH, "lngForce", OmniPvdDataTypeEnum::eFLOAT32, 3);
	t.lngTorqueAH = omniWriter->registerAttribute(t.CH, "lngTorque", OmniPvdDataTypeEnum::eFLOAT32, 3);

	t.latForceAH = omniWriter->registerAttribute(t.CH, "latForce", OmniPvdDataTypeEnum::eFLOAT32, 3);
	t.latTorqueAH = omniWriter->registerAttribute(t.CH, "latTorque", OmniPvdDataTypeEnum::eFLOAT32, 3);

	t.aligningMomentAH = omniWriter->registerAttribute(t.CH, "aligningMoment", OmniPvdDataTypeEnum::eFLOAT32, 3);
	t.wheelTorqueAH = omniWriter->registerAttribute(t.CH, "wheelTorque", OmniPvdDataTypeEnum::eFLOAT32, 3);

	return t;
}

void writeTireForce
(const PxVehicleTireForce& tireForce,
 const OmniPvdObjectHandle oh, const TireForce& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeVec3Attribute(omniWriter, ch, oh, ah.lngForceAH, tireForce.forces[PxVehicleTireDirectionModes::eLONGITUDINAL]);
	writeVec3Attribute(omniWriter, ch, oh, ah.lngTorqueAH, tireForce.torques[PxVehicleTireDirectionModes::eLONGITUDINAL]);
	writeVec3Attribute(omniWriter, ch, oh, ah.latForceAH, tireForce.forces[PxVehicleTireDirectionModes::eLATERAL]);
	writeVec3Attribute(omniWriter, ch, oh, ah.latTorqueAH, tireForce.torques[PxVehicleTireDirectionModes::eLATERAL]);
	writeFloatAttribute(omniWriter, ch, oh, ah.aligningMomentAH, tireForce.aligningMoment);
	writeFloatAttribute(omniWriter, ch, oh, ah.wheelTorqueAH, tireForce.wheelTorque);
}

WheelAttachment registerWheelAttachment(OmniPvdWriter* omniWriter)
{
	WheelAttachment w;
	w.CH = omniWriter->registerClass("WheelAttachment");
	w.wheelParamsAH = omniWriter->registerAttribute(w.CH, "wheelParams", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.wheelActuationStateAH = omniWriter->registerAttribute(w.CH, "wheelActuationState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.wheelRigidBody1dStateAH = omniWriter->registerAttribute(w.CH, "wheelRigidBody1dState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.wheelLocalPoseStateAH = omniWriter->registerAttribute(w.CH, "wheelLocalPosetate", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.roadGeomStateAH = omniWriter->registerAttribute(w.CH, "roadGeomState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.suspParamsAH = omniWriter->registerAttribute(w.CH, "suspParams", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.suspCompParamsAH = omniWriter->registerAttribute(w.CH, "suspComplianceParams", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.suspForceParamsAH = omniWriter->registerAttribute(w.CH, "suspForceParams", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.suspStateAH = omniWriter->registerAttribute(w.CH, "suspState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.suspCompStateAH = omniWriter->registerAttribute(w.CH, "suspComplianceState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.suspForceAH = omniWriter->registerAttribute(w.CH, "suspForce", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.tireParamsAH = omniWriter->registerAttribute(w.CH, "tireParams", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.tireDirectionStateAH = omniWriter->registerAttribute(w.CH, "tireDirectionState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.tireSpeedStateAH = omniWriter->registerAttribute(w.CH, "tireSpeedState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.tireSlipStateAH = omniWriter->registerAttribute(w.CH, "tireSlipState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.tireStickyStateAH = omniWriter->registerAttribute(w.CH, "tireStickyState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.tireGripStateAH = omniWriter->registerAttribute(w.CH, "tireGripState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.tireCamberStateAH = omniWriter->registerAttribute(w.CH, "tireCamberState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.tireForceAH = omniWriter->registerAttribute(w.CH, "tireForce", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	return w;
}

//////////////////////////
//ANTIROLL
//////////////////////////

AntiRollParams registerAntiRollParams(OmniPvdWriter* omniWriter)
{ 
	AntiRollParams a; 
	a.CH = omniWriter->registerClass("AntiRollParams");
	a.wheel0AH = omniWriter->registerAttribute(a.CH, "wheel0", OmniPvdDataTypeEnum::eUINT32, 1);
	a.wheel1AH = omniWriter->registerAttribute(a.CH, "wheel1", OmniPvdDataTypeEnum::eUINT32, 1);
	a.stiffnessAH = omniWriter->registerAttribute(a.CH, "stiffness", OmniPvdDataTypeEnum::eFLOAT32, 1);
	return a;
}

void writeAntiRollParams
(const PxVehicleAntiRollForceParams& antiRollParams,
 const OmniPvdObjectHandle oh, const AntiRollParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeUInt32Attribute(omniWriter, ch, oh, ah.wheel0AH, antiRollParams.wheel0);
	writeUInt32Attribute(omniWriter, ch, oh, ah.wheel1AH, antiRollParams.wheel1);
	writeFloatAttribute(omniWriter, ch, oh, ah.stiffnessAH, antiRollParams.stiffness);
}

AntiRollForce registerAntiRollForce(OmniPvdWriter* omniWriter)
{
	AntiRollForce a;
	a.CH = omniWriter->registerClass("AntiRollForce");
	a.torqueAH = omniWriter->registerAttribute(a.CH, "torque", OmniPvdDataTypeEnum::eFLOAT32, 3);
	return a;
}

void writeAntiRollForce
(const PxVehicleAntiRollTorque& antiRollForce, 
 const OmniPvdObjectHandle oh, const AntiRollForce& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeVec3Attribute(omniWriter, ch, oh, ah.torqueAH, antiRollForce.antiRollTorque);
}


//////////////////////////////////
//SUSPENSION STATE CALCULATION
//////////////////////////////////

SuspStateCalcParams registerSuspStateCalcParams(OmniPvdWriter* omniWriter)
{
	struct SuspJounceCalcType
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle raycastAH;
		OmniPvdAttributeHandle sweepAH;
		OmniPvdAttributeHandle noneAH;
	};
	SuspJounceCalcType jounceCalcType;
	jounceCalcType.CH = omniWriter->registerClass("SuspJounceCalculationType");
	jounceCalcType.raycastAH = omniWriter->registerEnumValue(jounceCalcType.CH, "raycast", PxVehicleSuspensionJounceCalculationType::eRAYCAST);
	jounceCalcType.sweepAH = omniWriter->registerEnumValue(jounceCalcType.CH, "sweep", PxVehicleSuspensionJounceCalculationType::eSWEEP);
	jounceCalcType.noneAH = omniWriter->registerEnumValue(jounceCalcType.CH, "none", PxVehicleSuspensionJounceCalculationType::eMAX_NB);

	struct BoolAsEnum
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle falseAH;
		OmniPvdAttributeHandle trueAH;
	};
	BoolAsEnum boolAsEnum;
	boolAsEnum.CH = omniWriter->registerClass("SuspStateCalcParamsBool");
	boolAsEnum.falseAH = omniWriter->registerEnumValue(boolAsEnum.CH, "False", 0);
	boolAsEnum.trueAH = omniWriter->registerEnumValue(boolAsEnum.CH, "True", 1);

	SuspStateCalcParams s;
	s.CH = omniWriter->registerClass("SuspStateCalculationParams");
	s.calcTypeAH = omniWriter->registerFlagsAttribute(s.CH, jounceCalcType.CH, "suspJounceCalculationType");
	s.limitExpansionValAH = omniWriter->registerFlagsAttribute(s.CH, boolAsEnum.CH,  "limitSuspensionExpansionVelocity");
	return s;
}

void writeSuspStateCalcParams
(const PxVehicleSuspensionStateCalculationParams& suspStateCalcParams,
 const OmniPvdObjectHandle oh, const SuspStateCalcParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFlagAttribute(omniWriter, ch, oh, ah.limitExpansionValAH, suspStateCalcParams.limitSuspensionExpansionVelocity ? 1 : 0);
	writeFlagAttribute(omniWriter, ch, oh, ah.calcTypeAH, suspStateCalcParams.suspensionJounceCalculationType);
}

//////////////////////////////////////
//DIRECT DRIVETRAIN
//////////////////////////////////////

DirectDriveCommandState registerDirectDriveCommandState(OmniPvdWriter* omniWriter)
{
	DirectDriveCommandState c;
	c.CH = omniWriter->registerClass("DirectDriveCommandState");
	c.brakesAH= omniWriter->registerAttribute(c.CH, "brakes", OmniPvdDataTypeEnum::eFLOAT32, 2);
	c.throttleAH= omniWriter->registerAttribute(c.CH, "throttle", OmniPvdDataTypeEnum::eFLOAT32, 1);
	c.steerAH= omniWriter->registerAttribute(c.CH, "steer", OmniPvdDataTypeEnum::eFLOAT32, 1);
	return c;	
}

void writeDirectDriveCommandState
(const PxVehicleCommandState& commands,
 const OmniPvdObjectHandle oh, const DirectDriveCommandState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	float brakes[2];
	for(PxU32 i = 0; i < commands.nbBrakes; i++)
	{
		brakes[i] = commands.brakes[i];
	}
	for(PxU32 i = commands.nbBrakes; i < 2; i++) 
	{
		brakes[i] = PX_MAX_F32;
	}
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.brakesAH, brakes, 2);
	writeFloatAttribute(omniWriter, ch, oh, ah.throttleAH, commands.throttle);
	writeFloatAttribute(omniWriter, ch, oh, ah.steerAH, commands.steer);
}

DirectDriveTransmissionCommandState registerDirectDriveTransmissionCommandState(OmniPvdWriter* omniWriter)
{
	struct DirectDriveGear
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle reverse;
		OmniPvdAttributeHandle neutral;
		OmniPvdAttributeHandle forward;
	};

	DirectDriveGear g;
	g.CH = omniWriter->registerClass("DirectDriveGear");
	g.reverse = omniWriter->registerEnumValue(g.CH, "reverse", PxVehicleDirectDriveTransmissionCommandState::eREVERSE);
	g.neutral = omniWriter->registerEnumValue(g.CH, "neutral", PxVehicleDirectDriveTransmissionCommandState::eNEUTRAL);
	g.forward = omniWriter->registerEnumValue(g.CH, "forward", PxVehicleDirectDriveTransmissionCommandState::eFORWARD);

	DirectDriveTransmissionCommandState c;
	c.CH = omniWriter->registerClass("DirectDriveTransmissionCommandState");
	c.gearAH = omniWriter->registerFlagsAttribute(c.CH, g.CH, "gear");
	return c;	
}

void writeDirectDriveTransmissionCommandState
(const PxVehicleDirectDriveTransmissionCommandState& transmission,
 const OmniPvdObjectHandle oh, const DirectDriveTransmissionCommandState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFlagAttribute(omniWriter, ch, oh, ah.gearAH, transmission.gear);
}

WheelResponseParams registerDirectDriveThrottleResponseParams(OmniPvdWriter* omniWriter)
{
	return registerWheelResponseParams("DirectDriveThrottleResponseParams", omniWriter);
}

void writeDirectDriveThrottleResponseParams
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleDirectDriveThrottleCommandResponseParams& directDriveThrottleResponseParams,
 const OmniPvdObjectHandle oh, const WheelResponseParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeWheelResponseParams(axleDesc, directDriveThrottleResponseParams, oh, ah, omniWriter, ch);
}

DirectDriveThrottleResponseState registerDirectDriveThrottleResponseState(OmniPvdWriter* omniWriter)
{
	DirectDriveThrottleResponseState d;
	d.CH = omniWriter->registerClass("DirectDriveThrottleResponseState");
	d.states0To3AH = omniWriter->registerAttribute(d.CH, "responseState0To3", OmniPvdDataTypeEnum::eFLOAT32, 4);
	d.states4To7AH = omniWriter->registerAttribute(d.CH, "responseState4To7", OmniPvdDataTypeEnum::eFLOAT32, 4);
	d.states8To11AH = omniWriter->registerAttribute(d.CH, "responseState8To11", OmniPvdDataTypeEnum::eFLOAT32, 4);
	d.states12To15AH = omniWriter->registerAttribute(d.CH, "responseState12To15", OmniPvdDataTypeEnum::eFLOAT32, 4);
	d.states16To19AH = omniWriter->registerAttribute(d.CH, "responseState16To19", OmniPvdDataTypeEnum::eFLOAT32, 4);
	return d;
}

void writeDirectDriveThrottleResponseState
(const PxVehicleAxleDescription& axleDesc, const PxVehicleArrayData<PxReal>& throttleResponseState,
 const OmniPvdObjectHandle oh, const DirectDriveThrottleResponseState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	//States are always in setToDefault() state as default.
	PxF32 states[PxVehicleLimits::eMAX_NB_WHEELS];
	PxMemZero(states, sizeof(states));
	
	if(!throttleResponseState.isEmpty())
	{
		for(PxU32 i = 0; i < axleDesc.nbWheels; i++)
		{
			const PxU32 wheelId = axleDesc.wheelIdsInAxleOrder[i];
			states[wheelId] = throttleResponseState[wheelId];
		}
	}

	writeFloatArrayAttribute(omniWriter, ch, oh, ah.states0To3AH, states + 0, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.states4To7AH, states + 4, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.states8To11AH, states + 8, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.states12To15AH, states + 12, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.states16To19AH, states + 16, 4);
}

DirectDrivetrain registerDirectDrivetrain(OmniPvdWriter* omniWriter)
{
	DirectDrivetrain d;
	d.CH = omniWriter->registerClass("DirectDrivetrain");
	d.throttleResponseParamsAH = omniWriter->registerAttribute(d.CH, "throttleResponseParams", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	d.commandStateAH = omniWriter->registerAttribute(d.CH, "directDriveCommandState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	d.throttleResponseStateAH = omniWriter->registerAttribute(d.CH, "throttleResponseState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	return d;
}


//////////////////////////////
//ENGINE DRIVETRAIN
//////////////////////////////

EngineDriveCommandState registerEngineDriveCommandState(OmniPvdWriter* omniWriter)
{
	EngineDriveCommandState c;
	c.CH = omniWriter->registerClass("EngineDriveCommandState");
	c.brakes= omniWriter->registerAttribute(c.CH, "brakes", OmniPvdDataTypeEnum::eFLOAT32, 2);
	c.throttle= omniWriter->registerAttribute(c.CH, "throttle", OmniPvdDataTypeEnum::eFLOAT32, 1);
	c.steer= omniWriter->registerAttribute(c.CH, "steer", OmniPvdDataTypeEnum::eFLOAT32, 1);
	return c;
}


void writeEngineDriveCommandState
(const PxVehicleCommandState& commands,
 const OmniPvdObjectHandle oh, const EngineDriveCommandState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	float brakes[2];
	for(PxU32 i = 0; i < commands.nbBrakes; i++)
	{
		brakes[i] = commands.brakes[i];
	}
	for(PxU32 i = commands.nbBrakes; i < 2; i++) 
	{
		brakes[i] = PX_MAX_F32;
	}
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.brakes, brakes, 2);
	writeFloatAttribute(omniWriter, ch, oh, ah.throttle, commands.throttle);
	writeFloatAttribute(omniWriter, ch, oh, ah.steer, commands.steer);
}

EngineDriveTransmissionCommandState registerEngineDriveTransmissionCommandState(OmniPvdWriter* omniWriter)
{
	EngineDriveTransmissionCommandState c;
	c.CH = omniWriter->registerClass("EngineDriveTranmissionCommandState");
	c.gearAH = omniWriter->registerAttribute(c.CH, "targetGear", OmniPvdDataTypeEnum::eUINT32, 1);	
	c.clutchAH = omniWriter->registerAttribute(c.CH, "clutch", OmniPvdDataTypeEnum::eFLOAT32, 1);	
	return c;
}

void writeEngineDriveTransmissionCommandState
(const PxVehicleEngineDriveTransmissionCommandState& transmission,
 const OmniPvdObjectHandle oh, const EngineDriveTransmissionCommandState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeUInt32Attribute(omniWriter, ch, oh, ah.gearAH, transmission.targetGear);
	writeFloatAttribute(omniWriter, ch, oh, ah.clutchAH, transmission.clutch);
}

ClutchResponseParams registerClutchResponseParams(OmniPvdWriter* omniWriter)
{
	ClutchResponseParams c;
	c.CH = omniWriter->registerClass("ClutchResponseParams");
	c.maxResponseAH = omniWriter->registerAttribute(c.CH, "MaxResponse", OmniPvdDataTypeEnum::eFLOAT32, 1);
	return c;
}

ClutchParams registerClutchParams(OmniPvdWriter* omniWriter)
{
	struct VehicleClutchAccuracyMode
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle estimateAH;
		OmniPvdAttributeHandle bestPossibleAH;
	};
	VehicleClutchAccuracyMode mode;
	mode.CH = omniWriter->registerClass("ClutchAccuracyMode");
	mode.estimateAH = omniWriter->registerEnumValue(mode.CH, "estimate", PxVehicleClutchAccuracyMode::eESTIMATE);
	mode.bestPossibleAH = omniWriter->registerEnumValue(mode.CH, "bestPossible", PxVehicleClutchAccuracyMode::eBEST_POSSIBLE);

	ClutchParams v;
	v.CH = omniWriter->registerClass("ClutchParams");
	v.accuracyAH = omniWriter->registerFlagsAttribute(v.CH, mode.CH, "accuracyMode");
	v.iterationsAH = omniWriter->registerAttribute(v.CH, "iterations", OmniPvdDataTypeEnum::eUINT32, 1);
	return v;	
}

void writeClutchParams
(const PxVehicleClutchParams& clutchParams, 
 const OmniPvdObjectHandle oh, const ClutchParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFlagAttribute(omniWriter, ch, oh, ah.accuracyAH, clutchParams.accuracyMode);
	writeUInt32Attribute(omniWriter, ch, oh, ah.iterationsAH, clutchParams.estimateIterations);
}

EngineParams registerEngineParams(OmniPvdWriter* omniWriter)
{
	EngineParams e;
	e.CH = omniWriter->registerClass("EngineParams");
	e.torqueCurveAH = omniWriter->registerAttribute(e.CH, "torqueCurve", OmniPvdDataTypeEnum::eFLOAT32, PxVehicleEngineParams::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES*2);
	e.peakTorqueAH = omniWriter->registerAttribute(e.CH, "peakTorque", OmniPvdDataTypeEnum::eFLOAT32, 1); 
	e.moiAH = omniWriter->registerAttribute(e.CH, "moi", OmniPvdDataTypeEnum::eFLOAT32, 1); 
	e.idleOmegaAH = omniWriter->registerAttribute(e.CH, "idleOmega", OmniPvdDataTypeEnum::eFLOAT32, 1); 
	e.maxOmegaAH = omniWriter->registerAttribute(e.CH, "maxOmega", OmniPvdDataTypeEnum::eFLOAT32, 1); 
	e.dampingRateFullThrottleAH = omniWriter->registerAttribute(e.CH, "dampingRateFullThrottleAH", OmniPvdDataTypeEnum::eFLOAT32, 1); 
	e.dampingRateZeroThrottleClutchDisengagedAH = omniWriter->registerAttribute(e.CH, "dampingRateZeroThrottleClutchDisengaged", OmniPvdDataTypeEnum::eFLOAT32, 1); 
	e.dampingRateZeroThrottleClutchEngagedAH = omniWriter->registerAttribute(e.CH, "dampingRateZeroThrottleClutchEngaged", OmniPvdDataTypeEnum::eFLOAT32, 1); 
	return e;
}

void writeEngineParams
(const PxVehicleEngineParams& engineParams, 
 const OmniPvdObjectHandle oh, const EngineParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	float torqueCurve[PxVehicleEngineParams::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES*2];
	for(PxU32 i = 0; i < engineParams.torqueCurve.nbDataPairs; i++)
	{
		torqueCurve[2*i + 0] = engineParams.torqueCurve.xVals[i];
		torqueCurve[2*i + 1] = engineParams.torqueCurve.yVals[i];
	}
	for(PxU32 i = engineParams.torqueCurve.nbDataPairs; i < PxVehicleEngineParams::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES; i++)
	{
		torqueCurve[2*i + 0] = PX_MAX_F32;
		torqueCurve[2*i + 1] = PX_MAX_F32;
	}

	writeFloatArrayAttribute(omniWriter, ch, oh, ah.torqueCurveAH, torqueCurve,  PxVehicleEngineParams::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES*2);
	writeFloatAttribute(omniWriter, ch, oh, ah.peakTorqueAH, engineParams.peakTorque);
	writeFloatAttribute(omniWriter, ch, oh, ah.moiAH, engineParams.moi);
	writeFloatAttribute(omniWriter, ch, oh, ah.idleOmegaAH, engineParams.idleOmega);
	writeFloatAttribute(omniWriter, ch, oh, ah.maxOmegaAH, engineParams.maxOmega);
	writeFloatAttribute(omniWriter, ch, oh, ah.dampingRateFullThrottleAH, engineParams.dampingRateFullThrottle);
	writeFloatAttribute(omniWriter, ch, oh, ah.dampingRateZeroThrottleClutchDisengagedAH, engineParams.dampingRateZeroThrottleClutchDisengaged);
	writeFloatAttribute(omniWriter, ch, oh, ah.dampingRateZeroThrottleClutchEngagedAH, engineParams.dampingRateZeroThrottleClutchEngaged);
}

GearboxParams registerGearboxParams(OmniPvdWriter* omniWriter)
{
	GearboxParams g;
	g.CH = omniWriter->registerClass("GearboxParams");
	g.ratiosAH = omniWriter->registerAttribute(g.CH, "ratios", OmniPvdDataTypeEnum::eFLOAT32, PxVehicleGearboxParams::eMAX_NB_GEARS); 
	g.nbRatiosAH =  omniWriter->registerAttribute(g.CH, "nbRatios", OmniPvdDataTypeEnum::eUINT32, 1); 
	g.neutralGearAH =  omniWriter->registerAttribute(g.CH, "neutralGear", OmniPvdDataTypeEnum::eUINT32, 1); 
	g.finalRatioAH =  omniWriter->registerAttribute(g.CH, "finalRatio", OmniPvdDataTypeEnum::eFLOAT32, 1); 
	g.switchTimeAH =  omniWriter->registerAttribute(g.CH, "switchTime", OmniPvdDataTypeEnum::eFLOAT32, 1); 
	return g;
}

void writeGearboxParams
(const PxVehicleGearboxParams& gearboxParams, 
 const OmniPvdObjectHandle oh, const GearboxParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	float ratios[PxVehicleGearboxParams::eMAX_NB_GEARS];
	PxMemCopy(ratios, gearboxParams.ratios, sizeof(float)*gearboxParams.nbRatios);
	for(PxU32 i = gearboxParams.nbRatios; i < PxVehicleGearboxParams::eMAX_NB_GEARS; i++)
	{
		ratios[i] = PX_MAX_F32;
	}
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.ratiosAH, ratios, PxVehicleGearboxParams::eMAX_NB_GEARS);
	writeUInt32Attribute(omniWriter, ch, oh, ah.nbRatiosAH,  gearboxParams.nbRatios);
	writeUInt32Attribute(omniWriter, ch, oh, ah.neutralGearAH, gearboxParams.neutralGear);
	writeFloatAttribute(omniWriter, ch, oh, ah.finalRatioAH, gearboxParams.finalRatio);
	writeFloatAttribute(omniWriter, ch, oh, ah.switchTimeAH, gearboxParams.switchTime);
}

AutoboxParams registerAutoboxParams(OmniPvdWriter* omniWriter)
{
	AutoboxParams a;
	a.CH = omniWriter->registerClass("AutoboxParams");
	a.upRatiosAH = omniWriter->registerAttribute(a.CH, "upRatios", OmniPvdDataTypeEnum::eFLOAT32, PxVehicleGearboxParams::eMAX_NB_GEARS); 
	a.downRatiosAH = omniWriter->registerAttribute(a.CH, "downRatios", OmniPvdDataTypeEnum::eFLOAT32, PxVehicleGearboxParams::eMAX_NB_GEARS); 
	a.latencyAH = omniWriter->registerAttribute(a.CH, "latency", OmniPvdDataTypeEnum::eFLOAT32, 1); 
	return a;
}

void writeAutoboxParams
(const PxVehicleAutoboxParams& autoboxParams, 
 const OmniPvdObjectHandle oh, const AutoboxParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.upRatiosAH, autoboxParams.upRatios, PxVehicleGearboxParams::eMAX_NB_GEARS);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.downRatiosAH, autoboxParams.downRatios, PxVehicleGearboxParams::eMAX_NB_GEARS);
	writeFloatAttribute(omniWriter, ch, oh, ah.latencyAH, autoboxParams.latency);
}

MultiWheelDiffParams registerMultiWheelDiffParams(const char* name, OmniPvdWriter* omniWriter)
{
	MultiWheelDiffParams m;
	m.CH = omniWriter->registerClass(name);
	m.torqueRatios0To3AH = omniWriter->registerAttribute(m.CH, "torqueRatios0To3", OmniPvdDataTypeEnum::eFLOAT32, 4); 
	m.torqueRatios4To7AH = omniWriter->registerAttribute(m.CH, "torqueRatios4To7", OmniPvdDataTypeEnum::eFLOAT32, 4); 
	m.torqueRatios8To11AH = omniWriter->registerAttribute(m.CH, "torqueRatios8To11", OmniPvdDataTypeEnum::eFLOAT32, 4); 
	m.torqueRatios12To15AH = omniWriter->registerAttribute(m.CH, "torqueRatios12To15", OmniPvdDataTypeEnum::eFLOAT32, 4); 
	m.torqueRatios16To19AH = omniWriter->registerAttribute(m.CH, "torqueRatios16To19", OmniPvdDataTypeEnum::eFLOAT32, 4); 
	m.aveWheelSpeedRatios0To3AH = omniWriter->registerAttribute(m.CH, "aveWheelSpeedRatios0To3", OmniPvdDataTypeEnum::eFLOAT32, 4); 
	m.aveWheelSpeedRatios4To7AH = omniWriter->registerAttribute(m.CH, "aveWheelSpeedRatios4To7", OmniPvdDataTypeEnum::eFLOAT32, 4); 
	m.aveWheelSpeedRatios8To11AH = omniWriter->registerAttribute(m.CH, "aveWheelSpeedRatios8To11", OmniPvdDataTypeEnum::eFLOAT32, 4); 
	m.aveWheelSpeedRatios12To15AH = omniWriter->registerAttribute(m.CH, "aveWheelSpeedRatios12To15", OmniPvdDataTypeEnum::eFLOAT32, 4); 
	m.aveWheelSpeedRatios16To19AH = omniWriter->registerAttribute(m.CH, "aveWheelSpeedRatios16To19", OmniPvdDataTypeEnum::eFLOAT32, 4); 
	return m;
}

MultiWheelDiffParams registerMultiWheelDiffParams(OmniPvdWriter* omniWriter)
{
	return registerMultiWheelDiffParams("MultiWheelDiffParams", omniWriter);
}

FourWheelDiffParams registerFourWheelDiffParams(OmniPvdWriter* omniWriter)
{
	FourWheelDiffParams m;
	static_cast<MultiWheelDiffParams&>(m) = registerMultiWheelDiffParams("FourWheelDiffParams", omniWriter);
	m.frontBiasAH = omniWriter->registerAttribute(m.CH, "frontBias", OmniPvdDataTypeEnum::eFLOAT32, 1);
	m.frontTargetAH = omniWriter->registerAttribute(m.CH, "frontTarget", OmniPvdDataTypeEnum::eFLOAT32, 1);
	m.rearBiasAH = omniWriter->registerAttribute(m.CH, "rearBias", OmniPvdDataTypeEnum::eFLOAT32, 1);
	m.rearTargetAH = omniWriter->registerAttribute(m.CH, "rearTarget", OmniPvdDataTypeEnum::eFLOAT32, 1);
	m.centreBiasAH = omniWriter->registerAttribute(m.CH, "centerBias", OmniPvdDataTypeEnum::eFLOAT32, 1);
	m.centreTargetAH = omniWriter->registerAttribute(m.CH, "centerTarget", OmniPvdDataTypeEnum::eFLOAT32, 1);
	m.frontWheelsAH = omniWriter->registerAttribute(m.CH, "frontWheels", OmniPvdDataTypeEnum::eUINT32, 2);
	m.rearWheelsAH = omniWriter->registerAttribute(m.CH, "rearWheels", OmniPvdDataTypeEnum::eUINT32, 2);
	return m;
}

void writeMultiWheelDiffParams
(const PxVehicleMultiWheelDriveDifferentialParams& diffParams, 
 const OmniPvdObjectHandle oh, const MultiWheelDiffParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.aveWheelSpeedRatios0To3AH, diffParams.aveWheelSpeedRatios + 0, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.aveWheelSpeedRatios4To7AH, diffParams.aveWheelSpeedRatios + 4, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.aveWheelSpeedRatios8To11AH, diffParams.aveWheelSpeedRatios + 8, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.aveWheelSpeedRatios12To15AH, diffParams.aveWheelSpeedRatios + 12, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.aveWheelSpeedRatios16To19AH, diffParams.aveWheelSpeedRatios + 16, 4);
										
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.torqueRatios0To3AH, diffParams.torqueRatios + 0, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.torqueRatios4To7AH, diffParams.torqueRatios + 4, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.torqueRatios8To11AH, diffParams.torqueRatios + 8, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.torqueRatios12To15AH, diffParams.torqueRatios + 12, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.torqueRatios16To19AH, diffParams.torqueRatios + 16, 4);
}

void writeFourWheelDiffParams
(const PxVehicleFourWheelDriveDifferentialParams& diffParams, 
 const OmniPvdObjectHandle oh, const FourWheelDiffParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeMultiWheelDiffParams(diffParams, oh, ah, omniWriter, ch);
	writeFloatAttribute(omniWriter, ch, oh, ah.frontBiasAH, diffParams.frontBias);
	writeFloatAttribute(omniWriter, ch, oh, ah.frontTargetAH, diffParams.frontTarget);
	writeFloatAttribute(omniWriter, ch, oh, ah.rearBiasAH, diffParams.rearBias);
	writeFloatAttribute(omniWriter, ch, oh, ah.rearTargetAH, diffParams.rearTarget);
	writeFloatAttribute(omniWriter, ch, oh, ah.centreBiasAH, diffParams.centerBias);
	writeFloatAttribute(omniWriter, ch, oh, ah.centreTargetAH, diffParams.centerTarget);
	writeUInt32ArrayAttribute(omniWriter, ch, oh, ah.frontWheelsAH, diffParams.frontWheelIds, 2);
	writeUInt32ArrayAttribute(omniWriter, ch, oh, ah.rearWheelsAH, diffParams.rearWheelIds, 2);
}

ClutchResponseState registerClutchResponseState(OmniPvdWriter* omniWriter)
{
	ClutchResponseState c;
	c.CH = omniWriter->registerClass("ClutchResponseState");
	c.normalisedResponseAH = omniWriter->registerAttribute(c.CH, "normalisedResponse", OmniPvdDataTypeEnum::eFLOAT32, 1);
	c.responseAH = omniWriter->registerAttribute(c.CH, "response", OmniPvdDataTypeEnum::eFLOAT32, 1);
	return c;
}

void writeClutchResponseState
(const PxVehicleClutchCommandResponseState& clutchResponseState, 
 const OmniPvdObjectHandle oh, const ClutchResponseState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.normalisedResponseAH, clutchResponseState.normalisedCommandResponse);
	writeFloatAttribute(omniWriter, ch, oh, ah.responseAH, clutchResponseState.commandResponse);
}

ThrottleResponseState registerThrottleResponseState(OmniPvdWriter* omniWriter)
{
	ThrottleResponseState t;
	t.CH  = omniWriter->registerClass("ThrottleResponseState");
	t.responseAH = omniWriter->registerAttribute(t.CH, "response", OmniPvdDataTypeEnum::eFLOAT32, 1);
	return t;
}

void writeThrottleResponseState
(const PxVehicleEngineDriveThrottleCommandResponseState& throttleResponseState, 
 const OmniPvdObjectHandle oh, const ThrottleResponseState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.responseAH, throttleResponseState.commandResponse);
}

EngineState registerEngineState(OmniPvdWriter* omniWriter)
{
	EngineState e;
	e.CH = omniWriter->registerClass("EngineState");
	e.rotationSpeedAH = omniWriter->registerAttribute(e.CH, "rotationSpeed", OmniPvdDataTypeEnum::eFLOAT32, 1);
	return e;
}

void writeEngineState
(const PxVehicleEngineState& engineState, 
 const OmniPvdObjectHandle oh, const EngineState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.rotationSpeedAH, engineState.rotationSpeed);
}

GearboxState registerGearboxState(OmniPvdWriter* omniWriter)
{
	GearboxState g;
	g.CH = omniWriter->registerClass("GearboxState");
	g.currentGearAH = omniWriter->registerAttribute(g.CH, "currentGear", OmniPvdDataTypeEnum::eUINT32, 1);
	g.targetGearAH = omniWriter->registerAttribute(g.CH, "targetGear", OmniPvdDataTypeEnum::eUINT32, 1);
	g.switchTimeAH = omniWriter->registerAttribute(g.CH, "switchTime", OmniPvdDataTypeEnum::eFLOAT32, 1);
	return g;
}

void writeGearboxState
(const PxVehicleGearboxState& gearboxState, 
 const OmniPvdObjectHandle oh, const GearboxState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeUInt32Attribute(omniWriter, ch, oh, ah.currentGearAH, gearboxState.currentGear);
	writeUInt32Attribute(omniWriter, ch, oh, ah.targetGearAH, gearboxState.targetGear);
	writeFloatAttribute(omniWriter, ch, oh, ah.switchTimeAH, gearboxState.gearSwitchTime);
}

AutoboxState registerAutoboxState(OmniPvdWriter* omniWriter)
{
	struct BoolAsEnum
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle falseAH;
		OmniPvdAttributeHandle trueAH;
	};
	BoolAsEnum boolAsEnum;
	boolAsEnum.CH = omniWriter->registerClass("AutoboxStateBool");
	boolAsEnum.falseAH = omniWriter->registerEnumValue(boolAsEnum.CH, "False", 0);
	boolAsEnum.trueAH = omniWriter->registerEnumValue(boolAsEnum.CH, "True", 1);

	AutoboxState a;
	a.CH = omniWriter->registerClass("GearboxState");
	a.timeSinceLastShiftAH = omniWriter->registerAttribute(a.CH, "timeSinceLastShift", OmniPvdDataTypeEnum::eFLOAT32, 1);
	a.activeAutoboxGearShiftAH = omniWriter->registerFlagsAttribute(a.CH, boolAsEnum.CH, "activeAutoboxShift");
	return a;
}

void writeAutoboxState
(const PxVehicleAutoboxState& autoboxState, 
 const OmniPvdObjectHandle oh, const AutoboxState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.timeSinceLastShiftAH, autoboxState.timeSinceLastShift);
	writeFlagAttribute(omniWriter, ch, oh, ah.activeAutoboxGearShiftAH, autoboxState.activeAutoboxGearShift ? 1: 0);
}

DiffState registerDiffState(OmniPvdWriter* omniWriter)
{
	DiffState d;
	d.CH = omniWriter->registerClass("DifferentialState");
	d.torqueRatios0To3AH = omniWriter->registerAttribute(d.CH, "torqueRatios0To3", OmniPvdDataTypeEnum::eFLOAT32, 4); 
	d.torqueRatios4To7AH = omniWriter->registerAttribute(d.CH, "torqueRatios4To7", OmniPvdDataTypeEnum::eFLOAT32, 4); 
	d.torqueRatios8To11AH = omniWriter->registerAttribute(d.CH, "torqueRatios8To11", OmniPvdDataTypeEnum::eFLOAT32, 4); 
	d.torqueRatios12To15AH = omniWriter->registerAttribute(d.CH, "torqueRatios12To15", OmniPvdDataTypeEnum::eFLOAT32, 4); 
	d.torqueRatios16To19AH = omniWriter->registerAttribute(d.CH, "torqueRatios16To19", OmniPvdDataTypeEnum::eFLOAT32, 4); 
	d.aveWheelSpeedRatios0To3AH = omniWriter->registerAttribute(d.CH, "aveWheelSpeedRatios0To3", OmniPvdDataTypeEnum::eFLOAT32, 4); 
	d.aveWheelSpeedRatios4To7AH = omniWriter->registerAttribute(d.CH, "aveWheelSpeedRatios4To7", OmniPvdDataTypeEnum::eFLOAT32, 4); 
	d.aveWheelSpeedRatios8To11AH = omniWriter->registerAttribute(d.CH, "aveWheelSpeedRatios8To11", OmniPvdDataTypeEnum::eFLOAT32, 4); 
	d.aveWheelSpeedRatios12To15AH = omniWriter->registerAttribute(d.CH, "aveWheelSpeedRatios12To15", OmniPvdDataTypeEnum::eFLOAT32, 4); 
	d.aveWheelSpeedRatios16To19AH = omniWriter->registerAttribute(d.CH, "aveWheelSpeedRatios16To19", OmniPvdDataTypeEnum::eFLOAT32, 4); 
	return d;
}

void writeDiffState
(const PxVehicleDifferentialState& diffState, 
 const OmniPvdObjectHandle oh, const DiffState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.aveWheelSpeedRatios0To3AH, diffState.aveWheelSpeedContributionAllWheels + 0, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.aveWheelSpeedRatios4To7AH, diffState.aveWheelSpeedContributionAllWheels + 4, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.aveWheelSpeedRatios8To11AH, diffState.aveWheelSpeedContributionAllWheels + 8, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.aveWheelSpeedRatios12To15AH, diffState.aveWheelSpeedContributionAllWheels + 12, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.aveWheelSpeedRatios16To19AH, diffState.aveWheelSpeedContributionAllWheels + 16, 4);

	writeFloatArrayAttribute(omniWriter, ch, oh, ah.torqueRatios0To3AH, diffState.torqueRatiosAllWheels + 0, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.torqueRatios4To7AH, diffState.torqueRatiosAllWheels + 4, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.torqueRatios8To11AH, diffState.torqueRatiosAllWheels + 8, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.torqueRatios12To15AH, diffState.torqueRatiosAllWheels + 12, 4);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.torqueRatios16To19AH, diffState.torqueRatiosAllWheels + 16, 4);
}

ClutchSlipState registerClutchSlipState(OmniPvdWriter* omniWriter)
{
	ClutchSlipState c;
	c.CH = omniWriter->registerClass("ClutchSlipState");
	c.slipAH = omniWriter->registerAttribute(c.CH, "clutchSlip", OmniPvdDataTypeEnum::eFLOAT32, 1);
	return c;
}

void writeClutchSlipState
(const PxVehicleClutchSlipState& clutchSlipState, 
 const OmniPvdObjectHandle oh, const ClutchSlipState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.slipAH, clutchSlipState.clutchSlip);
}

EngineDrivetrain registerEngineDrivetrain(OmniPvdWriter* omniWriter)
{
	EngineDrivetrain e;
	e.CH = omniWriter->registerClass("EngineDrivetrain");
	e.commandStateAH = omniWriter->registerAttribute(e.CH, "engineDriveCommandState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	e.transmissionCommandStateAH = omniWriter->registerAttribute(e.CH, "engineDriveTransmissionCommandState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	e.clutchResponseParamsAH = omniWriter->registerAttribute(e.CH, "clutchResponseParams", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	e.clutchParamsAH = omniWriter->registerAttribute(e.CH, "elutchParams", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	e.engineParamsAH = omniWriter->registerAttribute(e.CH, "engineParams", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	e.gearboxParamsAH = omniWriter->registerAttribute(e.CH, "gearboxParams", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	e.autoboxParamsAH = omniWriter->registerAttribute(e.CH, "autoboxParams", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	e.multiWheelDiffParamsAH = omniWriter->registerAttribute(e.CH, "multiWheelDiffParams", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	e.fourWheelDiffParamsAH = omniWriter->registerAttribute(e.CH, "fourWheelDiffParams", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	e.clutchResponseStateAH= omniWriter->registerAttribute(e.CH, "clutchResponseState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	e.throttleResponseStateAH= omniWriter->registerAttribute(e.CH, "throttleResponseState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	e.engineStateAH = omniWriter->registerAttribute(e.CH, "engineState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	e.gearboxStateAH = omniWriter->registerAttribute(e.CH, "gearboxState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	e.autoboxStateAH = omniWriter->registerAttribute(e.CH, "autoboxState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	e.diffStateAH = omniWriter->registerAttribute(e.CH, "diffState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	e.clutchSlipStateAH = omniWriter->registerAttribute(e.CH, "clutchSlipState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	return e;
}


////////////////////////////
//PHYSX WHEEL ATTACHMENT
////////////////////////////

PhysXSuspensionLimitConstraintParams registerSuspLimitConstraintParams(OmniPvdWriter* omniWriter)
{
	struct DirSpecifier
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle suspensionAH;
		OmniPvdAttributeHandle geomNormalAH;
		OmniPvdAttributeHandle noneAH;
	};
	DirSpecifier s;
	s.CH = omniWriter->registerClass("DirectionSpecifier");
	s.suspensionAH = omniWriter->registerEnumValue(s.CH, "suspensionDir", 
		PxVehiclePhysXSuspensionLimitConstraintParams::DirectionSpecifier::eSUSPENSION);
	s.geomNormalAH = omniWriter->registerEnumValue(s.CH, "geomNormalDir", 
		PxVehiclePhysXSuspensionLimitConstraintParams::DirectionSpecifier::eROAD_GEOMETRY_NORMAL);
	s.noneAH = omniWriter->registerEnumValue(s.CH, "geomNormalDir", 
		PxVehiclePhysXSuspensionLimitConstraintParams::DirectionSpecifier::eNONE);

	PhysXSuspensionLimitConstraintParams c;
	c.CH = omniWriter->registerClass("PhysXSuspLimitConstraintParams");
	c.restitutionAH = omniWriter->registerAttribute(c.CH, "restitution", OmniPvdDataTypeEnum::eFLOAT32, 1);
	c.directionForSuspensionLimitConstraintAH = omniWriter->registerFlagsAttribute(c.CH, s.CH, "directionMode");
	return c;
}

void writePhysXSuspLimitConstraintParams
(const PxVehiclePhysXSuspensionLimitConstraintParams& params,
 const OmniPvdObjectHandle oh, const PhysXSuspensionLimitConstraintParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFlagAttribute(omniWriter, ch, oh, ah.directionForSuspensionLimitConstraintAH, params.directionForSuspensionLimitConstraint);
	writeFloatAttribute(omniWriter, ch, oh, ah.restitutionAH, params.restitution);
}


PhysXWheelShape registerPhysXWheelShape(OmniPvdWriter* omniWriter)
{
	PhysXWheelShape w;
	w.CH = omniWriter->registerClass("PhysXWheelShape");
	w.shapePtrAH = omniWriter->registerAttribute(w.CH, "pxShapePtr", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	return w;
}

void writePhysXWheelShape
(const PxShape* wheelShape, 
 const OmniPvdObjectHandle oh, const PhysXWheelShape& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writePtrAttribute(omniWriter, ch, oh, ah.shapePtrAH, wheelShape);
}

PhysXRoadGeomState registerPhysXRoadGeomState(OmniPvdWriter* omniWriter)
{
	PhysXRoadGeomState g;
	g.CH = omniWriter->registerClass("PhysXRoadGeomState");
	g.hitPositionAH = omniWriter->registerAttribute(g.CH, "hitPosition", OmniPvdDataTypeEnum::eFLOAT32, 3);
	g.hitActorPtrAH = omniWriter->registerAttribute(g.CH, "PxActor", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	g.hitShapePtrAH = omniWriter->registerAttribute(g.CH, "PxShape", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	g.hitMaterialPtrAH = omniWriter->registerAttribute(g.CH, "PxMaterial", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	return g;
}

void writePhysXRoadGeomState
(const PxVehiclePhysXRoadGeometryQueryState& roadGeomState,
const OmniPvdObjectHandle oh, const PhysXRoadGeomState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeVec3Attribute(omniWriter, ch, oh, ah.hitPositionAH, roadGeomState.hitPosition);
	writePtrAttribute(omniWriter, ch, oh, ah.hitActorPtrAH, roadGeomState.actor);
	writePtrAttribute(omniWriter, ch, oh, ah.hitMaterialPtrAH, roadGeomState.material);
	writePtrAttribute(omniWriter, ch, oh, ah.hitShapePtrAH, roadGeomState.shape);
}

PhysXConstraintState registerPhysXConstraintState(OmniPvdWriter* omniWriter)
{
	struct BoolAsEnum
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle falseAH;
		OmniPvdAttributeHandle trueAH;
	};
	BoolAsEnum boolAsEnum;
	boolAsEnum.CH = omniWriter->registerClass("PhysXConstraintStateBool");
	boolAsEnum.falseAH = omniWriter->registerEnumValue(boolAsEnum.CH, "False", 0);
	boolAsEnum.trueAH = omniWriter->registerEnumValue(boolAsEnum.CH, "True", 1);

	PhysXConstraintState c;
	c.CH = omniWriter->registerClass("PhysXConstraintState");
	c.tireLongActiveStatusAH = omniWriter->registerFlagsAttribute(c.CH, boolAsEnum.CH, "tireLongitudinalActiveStatus");
	c.tireLongLinearAH = omniWriter->registerAttribute(c.CH, "tireLongitudinalLinear", OmniPvdDataTypeEnum::eFLOAT32, 3);
	c.tireLongAngularAH = omniWriter->registerAttribute(c.CH, "tireLongitudinalAngular", OmniPvdDataTypeEnum::eFLOAT32, 3);
	c.tireLongDampingAH = omniWriter->registerAttribute(c.CH, "tireLongitudinalDamping", OmniPvdDataTypeEnum::eFLOAT32, 1);
	c.tireLatActiveStatusAH = omniWriter->registerFlagsAttribute(c.CH, boolAsEnum.CH, "tireLateralActiveStatus");
	c.tireLatLinearAH = omniWriter->registerAttribute(c.CH, "tireLateralLinear", OmniPvdDataTypeEnum::eFLOAT32, 3);
	c.tireLatAngularAH = omniWriter->registerAttribute(c.CH, "tireLateralAngular", OmniPvdDataTypeEnum::eFLOAT32, 3);
	c.tireLatDampingAH = omniWriter->registerAttribute(c.CH, "tireLateralDamping", OmniPvdDataTypeEnum::eFLOAT32, 1);
	c.suspActiveStatusAH = omniWriter->registerFlagsAttribute(c.CH, boolAsEnum.CH, "suspActiveStatus");
	c.suspLinearAH = omniWriter->registerAttribute(c.CH, "suspLinear", OmniPvdDataTypeEnum::eFLOAT32, 3);
	c.suspAngularAH = omniWriter->registerAttribute(c.CH, "suspAngular", OmniPvdDataTypeEnum::eFLOAT32, 3);
	c.suspRestitutionAH = omniWriter->registerAttribute(c.CH, "suspRestitution", OmniPvdDataTypeEnum::eFLOAT32, 1);
	c.suspGeometricErrorAH = omniWriter->registerAttribute(c.CH, "suspGeometricError", OmniPvdDataTypeEnum::eFLOAT32, 1);
	return c;
}

void writePhysXConstraintState
(const PxVehiclePhysXConstraintState& roadGeomState,
 const OmniPvdObjectHandle oh, const PhysXConstraintState& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFlagAttribute(omniWriter, ch, oh, ah.suspActiveStatusAH, roadGeomState.suspActiveStatus ? 1 : 0);
	writeVec3Attribute(omniWriter, ch, oh, ah.suspLinearAH, roadGeomState.suspLinear);
	writeVec3Attribute(omniWriter, ch, oh, ah.suspAngularAH, roadGeomState.suspAngular);
	writeFloatAttribute(omniWriter, ch, oh, ah.suspGeometricErrorAH, roadGeomState.suspGeometricError);
	writeFloatAttribute(omniWriter, ch, oh, ah.suspRestitutionAH, roadGeomState.restitution);

	writeFlagAttribute(omniWriter, ch, oh, ah.tireLongActiveStatusAH, roadGeomState.tireActiveStatus[PxVehicleTireDirectionModes::eLONGITUDINAL] ? 1 : 0);
	writeVec3Attribute(omniWriter, ch, oh, ah.tireLongLinearAH, roadGeomState.tireLinears[PxVehicleTireDirectionModes::eLONGITUDINAL]);
	writeVec3Attribute(omniWriter, ch, oh, ah.tireLongAngularAH, roadGeomState.tireAngulars[PxVehicleTireDirectionModes::eLONGITUDINAL]);
	writeFloatAttribute(omniWriter, ch, oh, ah.tireLongDampingAH, roadGeomState.tireDamping[PxVehicleTireDirectionModes::eLONGITUDINAL]);

	writeFlagAttribute(omniWriter, ch, oh, ah.tireLatActiveStatusAH, roadGeomState.tireActiveStatus[PxVehicleTireDirectionModes::eLATERAL] ? 1 : 0);
	writeVec3Attribute(omniWriter, ch, oh, ah.tireLatLinearAH, roadGeomState.tireLinears[PxVehicleTireDirectionModes::eLATERAL]);
	writeVec3Attribute(omniWriter, ch, oh, ah.tireLatAngularAH, roadGeomState.tireAngulars[PxVehicleTireDirectionModes::eLATERAL]);
	writeFloatAttribute(omniWriter, ch, oh, ah.tireLatDampingAH, roadGeomState.tireDamping[PxVehicleTireDirectionModes::eLATERAL]);
}

PhysXMaterialFriction registerPhysXMaterialFriction(OmniPvdWriter* omniWriter)
{
	PhysXMaterialFriction f;
	f.CH = omniWriter->registerClass("PhysXMaterialFriction");
	f.frictionAH = omniWriter->registerAttribute(f.CH, "friction", OmniPvdDataTypeEnum::eFLOAT32, 1);
	f.materialPtrAH = omniWriter->registerAttribute(f.CH, "material", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	return f;
}

void writePhysXMaterialFriction
(const PxVehiclePhysXMaterialFriction& materialFriction,
 const OmniPvdObjectHandle oh, const PhysXMaterialFriction& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.frictionAH, materialFriction.friction);
	writePtrAttribute(omniWriter, ch, oh, ah.materialPtrAH, materialFriction.material);
}

PhysXWheelAttachment registerPhysXWheelAttachment(OmniPvdWriter* omniWriter)
{
	PhysXWheelAttachment w;
	w.CH = omniWriter->registerClass("PhysXWheelAttachment");
	w.physxConstraintParamsAH = omniWriter->registerAttribute(w.CH, "physxConstraintParams", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.physxConstraintStateAH = omniWriter->registerAttribute(w.CH, "physxConstraintState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.physxWeelShapeAH = omniWriter->registerAttribute(w.CH, "physxWheelShape", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.physxRoadGeometryStateAH = omniWriter->registerAttribute(w.CH, "physxRoadGeomState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	w.physxMaterialFrictionSetAH = omniWriter->registerSetAttribute(w.CH, "physXMaterialFrictions", OmniPvdDataTypeEnum::eOBJECT_HANDLE);
	return w;
}

//////////////////////////
//PHYSX RIGID ACTOR
//////////////////////////

PhysXRigidActor registerPhysXRigidActor(OmniPvdWriter* omniWriter)
{
	PhysXRigidActor a;
	a.CH = omniWriter->registerClass("PhysXRigidActor");
	a.rigidActorAH = omniWriter->registerAttribute(a.CH, "rigidActor", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	return a;
}

void writePhysXRigidActor
(const PxRigidActor* actor, 
 const OmniPvdObjectHandle oh, const PhysXRigidActor& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writePtrAttribute(omniWriter, ch, oh, ah.rigidActorAH, actor);
}

PhysXRoadGeometryQueryParams registerPhysXRoadGeometryQueryParams(OmniPvdWriter* omniWriter)
{
	struct Type
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle raycastAH;
		OmniPvdAttributeHandle sweepAH;
		OmniPvdAttributeHandle noneAH;
	};
	Type type;
	type.CH = omniWriter->registerClass("PhysXRoadGeometryQueryTpe");
	type.raycastAH = omniWriter->registerEnumValue(type.CH, "raycast", PxVehicleSuspensionJounceCalculationType::eRAYCAST);
	type.sweepAH = omniWriter->registerEnumValue(type.CH, "sweep", PxVehicleSuspensionJounceCalculationType::eSWEEP);
	type.noneAH = omniWriter->registerEnumValue(type.CH, "none", PxVehicleSuspensionJounceCalculationType::eMAX_NB);

	PhysXRoadGeometryQueryParams a;
	a.CH = omniWriter->registerClass("PhysXRoadGeomQueryParams");
	a.queryTypeAH = omniWriter->registerFlagsAttribute(a.CH, type.CH, "physxQueryType");
	return a;
}

void writePhysXRoadGeometryQueryParams
(const PxVehiclePhysXRoadGeometryQueryParams& actor, 
 const OmniPvdObjectHandle oh, const PhysXRoadGeometryQueryParams& ah, OmniPvdWriter* omniWriter, OmniPvdContextHandle ch)
{
	writeFlagAttribute(omniWriter, ch, oh, ah.queryTypeAH, actor.roadGeometryQueryType);
}


//////////////////////
//VEHICLE
//////////////////////

Vehicle registerVehicle(OmniPvdWriter* omniWriter)
{
	Vehicle v;
	v.CH = omniWriter->registerClass("Vehicle");

	v.rigidBodyParamsAH = omniWriter->registerAttribute(v.CH, "rigidBodyParams", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	v.rigidBodyStateAH = omniWriter->registerAttribute(v.CH, "rigidBodyState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);

	v.suspStateCalcParamsAH = omniWriter->registerAttribute(v.CH, "suspStateCalcParams", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);

	v.wheelAttachmentSetAH = omniWriter->registerSetAttribute(v.CH, "wheelAttachmentSet", OmniPvdDataTypeEnum::eOBJECT_HANDLE);

	v.antiRollSetAH = omniWriter->registerSetAttribute(v.CH, "antiRollSet", OmniPvdDataTypeEnum::eOBJECT_HANDLE);
	v.antiRollForceAH = omniWriter->registerAttribute(v.CH, "antiRollForce", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);

	v.brakeResponseParamsSetAH = omniWriter->registerSetAttribute(v.CH, "brakeResponseParamsSet", OmniPvdDataTypeEnum::eOBJECT_HANDLE);
	v.steerResponseParamsAH = omniWriter->registerAttribute(v.CH, "steerResponseParams", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	v.brakeResponseStatesAH = omniWriter->registerAttribute(v.CH, "brakeResponseState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	v.steerResponseParamsAH = omniWriter->registerAttribute(v.CH, "steerResponseState", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);

	v.directDrivetrainAH = omniWriter->registerAttribute(v.CH, "directDrivetrain", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	v.engineDriveTrainAH = omniWriter->registerAttribute(v.CH, "engineDrivetrain", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);

	v.physxWheelAttachmentSetAH = omniWriter->registerSetAttribute(v.CH, "physXheelAttachmentSet", OmniPvdDataTypeEnum::eOBJECT_HANDLE);

	v.physxRoadGeometryQueryParamsAH =  omniWriter->registerAttribute(v.CH, "physxRoadGeomQryParams", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);
	v.physxRigidActorAH = omniWriter->registerAttribute(v.CH, "physxRigidActor", OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1);

	return v;
}

#endif //PX_SUPPORT_OMNI_PVD

} // namespace vehicle2
} // namespace physx

/** @} */
