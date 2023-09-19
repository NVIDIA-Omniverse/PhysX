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

#include "VhPvdWriter.h"

namespace physx
{
namespace vehicle2
{

#if PX_SUPPORT_OMNI_PVD

PX_FORCE_INLINE void writeFloatAttribute
(OmniPvdWriter& omniWriter, OmniPvdContextHandle ch, OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, float val)
{
	omniWriter.setAttribute(ch, oh, ah, reinterpret_cast<uint8_t*>(&val), sizeof(float));
}

PX_FORCE_INLINE void writeFloatArrayAttribute
(OmniPvdWriter& omniWriter, OmniPvdContextHandle ch, OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, const float* val, const PxU32 nbVals)
{
	omniWriter.setAttribute(ch, oh, ah, reinterpret_cast<const uint8_t*>(val), sizeof(float) * nbVals);
}

PX_FORCE_INLINE void writeUInt32ArrayAttribute
(OmniPvdWriter& omniWriter, OmniPvdContextHandle ch, OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, const uint32_t* val, const PxU32 nbVals)
{
	omniWriter.setAttribute(ch, oh, ah, reinterpret_cast<const uint8_t*>(val), sizeof(uint32_t) * nbVals);
}


PX_FORCE_INLINE void writeVec3Attribute
(OmniPvdWriter& omniWriter, OmniPvdContextHandle ch, 
 OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, const PxVec3& val)
{
	omniWriter.setAttribute(ch, oh, ah, reinterpret_cast<const uint8_t*>(&val), sizeof(PxVec3));
}

PX_FORCE_INLINE void writePlaneAttribute
(OmniPvdWriter& omniWriter, OmniPvdContextHandle ch, OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, const PxPlane& val)
{
	omniWriter.setAttribute(ch, oh, ah, reinterpret_cast<const uint8_t*>(&val), sizeof(PxPlane));
}


PX_FORCE_INLINE void writeQuatAttribute
(OmniPvdWriter& omniWriter, OmniPvdContextHandle ch, OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, const PxQuat& val)
{
	omniWriter.setAttribute(ch, oh, ah, reinterpret_cast<const uint8_t*>(&val), sizeof(PxQuat));
}

PX_FORCE_INLINE void writeUInt8Attribute
(OmniPvdWriter& omniWriter, OmniPvdContextHandle ch, OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, uint8_t val)
{
	omniWriter.setAttribute(ch, oh, ah, reinterpret_cast<const uint8_t*>(&val), sizeof(uint8_t));
}

PX_FORCE_INLINE void writeUInt32Attribute
(OmniPvdWriter& omniWriter, OmniPvdContextHandle ch, OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, uint32_t val)
{
	omniWriter.setAttribute(ch, oh, ah, reinterpret_cast<const uint8_t*>(&val), sizeof(uint32_t));
}

PX_FORCE_INLINE void writeFlagAttribute
(OmniPvdWriter& omniWriter, OmniPvdContextHandle ch, OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, uint32_t val)
{
	writeUInt32Attribute(omniWriter, ch, oh, ah, val);
}

PX_FORCE_INLINE void writeLookupTableAttribute
(OmniPvdWriter& omniWriter, OmniPvdContextHandle ch, 
 OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, PxVehicleFixedSizeLookupTable<float, 3> val)
{
	float buffer[6] = { -1.0f, 0.0f, -1.0f, 0.0f, -1.0f, 0.0f };
	for(PxU32 i = 0; i < val.nbDataPairs; i++)
	{
		buffer[2 * i + 0] = val.xVals[i];
		buffer[2 * i + 1] = val.yVals[i];
	}
	omniWriter.setAttribute(ch, oh, ah, reinterpret_cast<const uint8_t*>(buffer), sizeof(buffer));
}

PX_FORCE_INLINE void writeLookupTableAttribute
(OmniPvdWriter& omniWriter, OmniPvdContextHandle ch, 
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
	omniWriter.setAttribute(ch, oh, ah, reinterpret_cast<const uint8_t*>(buffer), sizeof(buffer));
}

void writePtrAttribute
(OmniPvdWriter& omniWriter, OmniPvdContextHandle ch, 
 OmniPvdObjectHandle oh, OmniPvdAttributeHandle ah, const void* val)
{
	omniWriter.setAttribute(ch, oh, ah, reinterpret_cast<const uint8_t*>(&val), sizeof(val));
}

////////////////////////////////
//RIGID BODY
////////////////////////////////

RigidBodyParams registerRigidBodyParams(OmniPvdWriter& omniWriter)
{
	RigidBodyParams r;
	r.CH = omniWriter.registerClass("RigidBodyParams");
	r.massAH = omniWriter.registerAttribute(r.CH, "mass", OmniPvdDataType::eFLOAT32, 1);
	r.moiAH = omniWriter.registerAttribute(r.CH, "moi", OmniPvdDataType::eFLOAT32, 3);
	return r;
}

void writeRigidBodyParams
(const PxVehicleRigidBodyParams& rbodyParams,
 const OmniPvdObjectHandle oh, const RigidBodyParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.massAH, rbodyParams.mass);
	writeVec3Attribute(omniWriter, ch, oh, ah.moiAH, rbodyParams.moi);
}

RigidBodyState registerRigidBodyState(OmniPvdWriter& omniWriter)
{
	RigidBodyState r;
	r.CH = omniWriter.registerClass("RigidBodyState");
	r.posAH = omniWriter.registerAttribute(r.CH, "pos", OmniPvdDataType::eFLOAT32, 3);
	r.quatAH = omniWriter.registerAttribute(r.CH, "quat", OmniPvdDataType::eFLOAT32, 4);
	r.linearVelocityAH = omniWriter.registerAttribute(r.CH, "linvel", OmniPvdDataType::eFLOAT32, 3);
	r.angularVelocityAH = omniWriter.registerAttribute(r.CH, "angvel", OmniPvdDataType::eFLOAT32, 3);
	r.previousLinearVelocityAH = omniWriter.registerAttribute(r.CH, "prevLinvel", OmniPvdDataType::eFLOAT32, 3);
	r.previousAngularVelocityAH = omniWriter.registerAttribute(r.CH, "prevAngvel", OmniPvdDataType::eFLOAT32, 3);
	r.externalForceAH = omniWriter.registerAttribute(r.CH, "extForce", OmniPvdDataType::eFLOAT32, 3);
	r.externalTorqueAH = omniWriter.registerAttribute(r.CH, "extTorque", OmniPvdDataType::eFLOAT32, 3);
	return r;
}

void writeRigidBodyState
(const PxVehicleRigidBodyState& rbodyState,
 const OmniPvdObjectHandle oh, const RigidBodyState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
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

WheelResponseParams registerWheelResponseParams(const char* name, OmniPvdWriter& omniWriter)
{
	WheelResponseParams w;
	w.CH = omniWriter.registerClass(name);
	w.maxResponseAH = omniWriter.registerAttribute(w.CH, "maxResponse", OmniPvdDataType::eFLOAT32, 1);
	w.responseMultipliers0To3AH = omniWriter.registerAttribute(w.CH, "wheelMultipliers0To3", OmniPvdDataType::eFLOAT32, 4);
	w.responseMultipliers4To7AH = omniWriter.registerAttribute(w.CH, "wheelMultipliers4To7", OmniPvdDataType::eFLOAT32, 4);
	w.responseMultipliers8To11AH = omniWriter.registerAttribute(w.CH, "wheelMultipliers8To11", OmniPvdDataType::eFLOAT32, 4);
	w.responseMultipliers12To15AH = omniWriter.registerAttribute(w.CH, "wheelMultipliers12To15", OmniPvdDataType::eFLOAT32, 4);
	w.responseMultipliers16To19AH = omniWriter.registerAttribute(w.CH, "wheelMultipliers16To19", OmniPvdDataType::eFLOAT32, 4);
	return w;
}

WheelResponseParams registerSteerResponseParams(OmniPvdWriter& omniWriter)
{
	return registerWheelResponseParams("SteerResponseParams", omniWriter);
}

WheelResponseParams registerBrakeResponseParams(OmniPvdWriter& omniWriter)
{
	return registerWheelResponseParams("BrakeResponseParams", omniWriter);
}

WheelResponseStates registerWheelResponseStates(const char* name, OmniPvdWriter& omniWriter)
{
	WheelResponseStates w;
	w.CH = omniWriter.registerClass(name);
	w.responseStates0To3AH = omniWriter.registerAttribute(w.CH, "wheelStates0To3", OmniPvdDataType::eFLOAT32, 4);
	w.responseStates4To7AH = omniWriter.registerAttribute(w.CH, "wheelStatess4To7", OmniPvdDataType::eFLOAT32, 4);
	w.responseStates8To11AH = omniWriter.registerAttribute(w.CH, "wheelStates8To11", OmniPvdDataType::eFLOAT32, 4);
	w.responseStates12To15AH = omniWriter.registerAttribute(w.CH, "wheelStates12To15", OmniPvdDataType::eFLOAT32, 4);
	w.responseStates16To19AH = omniWriter.registerAttribute(w.CH, "wheelStates16To19", OmniPvdDataType::eFLOAT32, 4);
	return w;
}

WheelResponseStates registerSteerResponseStates(OmniPvdWriter& omniWriter)
{
	return registerWheelResponseStates("SteerResponseState", omniWriter);
}

WheelResponseStates registerBrakeResponseStates(OmniPvdWriter& omniWriter)
{
	return registerWheelResponseStates("BrakeResponseState", omniWriter);
}

AckermannParams registerAckermannParams(OmniPvdWriter& omniWriter)
{
	AckermannParams a;
	a.CH = omniWriter.registerClass("AckermannParams");
	a.wheelIdsAH = omniWriter.registerAttribute(a.CH, "wheelIds", OmniPvdDataType::eUINT32, 2);
	a.wheelBaseAH = omniWriter.registerAttribute(a.CH, "wheelBase", OmniPvdDataType::eFLOAT32, 1);
	a.trackWidthAH = omniWriter.registerAttribute(a.CH, "trackWidth", OmniPvdDataType::eFLOAT32, 1);
	a.strengthAH = omniWriter.registerAttribute(a.CH, "strength", OmniPvdDataType::eFLOAT32, 1);
	return a;
}

void writeWheelResponseParams
(const PxVehicleAxleDescription& axleDesc, const PxVehicleCommandResponseParams& responseParams,
 const OmniPvdObjectHandle oh, const WheelResponseParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
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
 const OmniPvdObjectHandle oh,  const WheelResponseParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeWheelResponseParams(axleDesc, steerResponseParams, oh, ah, omniWriter, ch);
}

void writeBrakeResponseParams
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleBrakeCommandResponseParams& brakeResponseParams,
 const OmniPvdObjectHandle oh,  const WheelResponseParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeWheelResponseParams(axleDesc, brakeResponseParams, oh, ah, omniWriter, ch);
}

void writeWheelResponseStates
(const PxVehicleAxleDescription& axleDesc, const PxVehicleArrayData<PxReal>& responseState,
 const OmniPvdObjectHandle oh, const WheelResponseStates& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
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
 const OmniPvdObjectHandle oh, const WheelResponseStates& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeWheelResponseStates(axleDesc, steerResponseStates, oh, ah, omniWriter, ch);
}

void writeBrakeResponseStates
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleArrayData<PxReal>& brakeResponseStates,
 const OmniPvdObjectHandle oh, const WheelResponseStates& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeWheelResponseStates(axleDesc, brakeResponseStates, oh, ah, omniWriter, ch);
}

void writeAckermannParams
(const PxVehicleAckermannParams& ackermannParams,
 const OmniPvdObjectHandle oh, const AckermannParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeUInt32ArrayAttribute(omniWriter, ch, oh, ah.wheelIdsAH, ackermannParams.wheelIds, 
		sizeof(PxVehicleAckermannParams::wheelIds) / sizeof(PxVehicleAckermannParams::wheelIds[0]));
	writeFloatAttribute(omniWriter, ch, oh, ah.wheelBaseAH, ackermannParams.wheelBase);
	writeFloatAttribute(omniWriter, ch, oh, ah.trackWidthAH, ackermannParams.trackWidth);
	writeFloatAttribute(omniWriter, ch, oh, ah.strengthAH, ackermannParams.strength);
}


void writeClutchResponseParams
(const PxVehicleClutchCommandResponseParams& clutchResponseParams, 
 const OmniPvdObjectHandle oh, const ClutchResponseParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.maxResponseAH, clutchResponseParams.maxResponse);
}

//////////////////////////////
//WHEEL ATTACHMENTS
//////////////////////////////

WheelParams registerWheelParams(OmniPvdWriter& omniWriter)
{
	WheelParams w;
	w.CH = omniWriter.registerClass("WheelParams");
	w.wheelRadiusAH = omniWriter.registerAttribute(w.CH, "radius", OmniPvdDataType::eFLOAT32, 1);
	w.halfWidthAH = omniWriter.registerAttribute(w.CH, "halfWidth", OmniPvdDataType::eFLOAT32, 1);
	w.massAH = omniWriter.registerAttribute(w.CH, "mass", OmniPvdDataType::eFLOAT32, 1);
	w.moiAH = omniWriter.registerAttribute(w.CH, "moi", OmniPvdDataType::eFLOAT32, 1);
	w.dampingRateAH = omniWriter.registerAttribute(w.CH, "dampingRate", OmniPvdDataType::eFLOAT32, 1);
	return w;
}

void writeWheelParams
(const PxVehicleWheelParams& params, 
 const OmniPvdObjectHandle oh, const WheelParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.dampingRateAH, params.dampingRate);
	writeFloatAttribute(omniWriter, ch, oh, ah.halfWidthAH, params.halfWidth);
	writeFloatAttribute(omniWriter, ch, oh, ah.massAH, params.mass);
	writeFloatAttribute(omniWriter, ch, oh, ah.moiAH, params.moi);
	writeFloatAttribute(omniWriter, ch, oh, ah.wheelRadiusAH, params.radius);
}

WheelActuationState registerWheelActuationState(OmniPvdWriter& omniWriter)
{
	struct BoolAsEnum
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle falseAH;
		OmniPvdAttributeHandle trueAH;
	};
	BoolAsEnum boolAsEnum;
	boolAsEnum.CH = omniWriter.registerClass("WheelStateBool");
	boolAsEnum.falseAH = omniWriter.registerEnumValue(boolAsEnum.CH, "False", 0);
	boolAsEnum.trueAH = omniWriter.registerEnumValue(boolAsEnum.CH, "True", 1);


	WheelActuationState w;
	w.CH = omniWriter.registerClass("WheelActuationState");
	w.isBrakeAppliedAH = omniWriter.registerFlagsAttribute(w.CH, "isBrakeApplied", boolAsEnum.CH);
	w.isDriveAppliedAH = omniWriter.registerFlagsAttribute(w.CH, "isDriveApplied", boolAsEnum.CH);

	return w;
}

void writeWheelActuationState
(const PxVehicleWheelActuationState& actState,
 const OmniPvdObjectHandle oh, const WheelActuationState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFlagAttribute(omniWriter, ch, oh, ah.isBrakeAppliedAH, actState.isBrakeApplied ? 1 : 0);
	writeFlagAttribute(omniWriter, ch, oh, ah.isDriveAppliedAH, actState.isDriveApplied ? 1 : 0);
}

WheelRigidBody1dState registerWheelRigidBody1dState(OmniPvdWriter& omniWriter)
{
	WheelRigidBody1dState w;
	w.CH = omniWriter.registerClass("WheelRigidBodyState");
	w.rotationSpeedAH = omniWriter.registerAttribute(w.CH, "rotationSpeed", OmniPvdDataType::eFLOAT32, 1);
	w.correctedRotationSpeedAH = omniWriter.registerAttribute(w.CH, "correctedRotationSpeed", OmniPvdDataType::eFLOAT32, 1);
	w.rotationAngleAH = omniWriter.registerAttribute(w.CH, "rotationAngle", OmniPvdDataType::eFLOAT32, 1);
	return w;
}

void writeWheelRigidBody1dState
(const PxVehicleWheelRigidBody1dState& rigidBodyState,
 const OmniPvdObjectHandle oh, const WheelRigidBody1dState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.rotationSpeedAH, rigidBodyState.rotationSpeed);
	writeFloatAttribute(omniWriter, ch, oh, ah.correctedRotationSpeedAH, rigidBodyState.correctedRotationSpeed);
	writeFloatAttribute(omniWriter, ch, oh, ah.rotationAngleAH, rigidBodyState.rotationAngle);
}

WheelLocalPoseState registerWheelLocalPoseState(OmniPvdWriter& omniWriter)
{
	WheelLocalPoseState w;
	w.CH = omniWriter.registerClass("WheelLocalPoseState");
	w.posAH = omniWriter.registerAttribute(w.CH, "posInRbodyFrame", OmniPvdDataType::eFLOAT32, 3);
	w.quatAH = omniWriter.registerAttribute(w.CH, "quatInRbodyFrame", OmniPvdDataType::eFLOAT32, 4);
	return w;
}

void writeWheelLocalPoseState
(const PxVehicleWheelLocalPose& pose,
 const OmniPvdObjectHandle oh, const WheelLocalPoseState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeVec3Attribute(omniWriter, ch, oh, ah.posAH, pose.localPose.p);
	writeQuatAttribute(omniWriter, ch, oh, ah.quatAH, pose.localPose.q);
}

RoadGeometryState registerRoadGeomState(OmniPvdWriter& omniWriter)
{
	struct BoolAsEnum
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle falseAH;
		OmniPvdAttributeHandle trueAH;
	};
	BoolAsEnum boolAsEnum;
	boolAsEnum.CH = omniWriter.registerClass("RoadGeomStateBool");
	boolAsEnum.falseAH = omniWriter.registerEnumValue(boolAsEnum.CH, "False", 0);
	boolAsEnum.trueAH = omniWriter.registerEnumValue(boolAsEnum.CH, "True", 1);

	RoadGeometryState r;
	r.CH = omniWriter.registerClass("RoadGeometryState");
	r.planeAH = omniWriter.registerAttribute(r.CH, "plane", OmniPvdDataType::eFLOAT32, 4);
	r.frictionAH = omniWriter.registerAttribute(r.CH, "friction", OmniPvdDataType::eFLOAT32, 1);
	r.hitStateAH = omniWriter.registerFlagsAttribute(r.CH, "hitState", boolAsEnum.CH);
	r.velocityAH = omniWriter.registerAttribute(r.CH,  "hitVelocity", OmniPvdDataType::eFLOAT32, 3);
	return r;
}

void writeRoadGeomState
(const PxVehicleRoadGeometryState& roadGeometryState,
 const OmniPvdObjectHandle oh, const RoadGeometryState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writePlaneAttribute(omniWriter, ch, oh, ah.planeAH, roadGeometryState.plane);
	writeFloatAttribute(omniWriter, ch, oh, ah.frictionAH, roadGeometryState.friction);
	writeFlagAttribute(omniWriter, ch, oh, ah.hitStateAH, roadGeometryState.hitState);
	writeVec3Attribute(omniWriter, ch, oh, ah.velocityAH, roadGeometryState.velocity);
}

SuspParams registerSuspParams(OmniPvdWriter& omniWriter)
{
	SuspParams s;
	s.CH = omniWriter.registerClass("SuspensionParams");
	s.suspAttachmentPosAH = omniWriter.registerAttribute(s.CH, "suspAttachmentPos", OmniPvdDataType::eFLOAT32, 3);
	s.suspAttachmentQuatAH = omniWriter.registerAttribute(s.CH, "suspAttachmentQuat", OmniPvdDataType::eFLOAT32, 4);
	s.suspDirAH = omniWriter.registerAttribute(s.CH, "suspDir", OmniPvdDataType::eFLOAT32, 3);
	s.suspTravleDistAH = omniWriter.registerAttribute(s.CH, "suspTravelDist", OmniPvdDataType::eFLOAT32, 1);
	s.wheelAttachmentPosAH = omniWriter.registerAttribute(s.CH, "wheelAttachmentPos", OmniPvdDataType::eFLOAT32, 3);
	s.wheelAttachmentQuatAH = omniWriter.registerAttribute(s.CH, "wheelAttachmentQuat", OmniPvdDataType::eFLOAT32, 3);
	return s;
}

void writeSuspParams
(const PxVehicleSuspensionParams& suspParams, 
 const OmniPvdObjectHandle oh, const SuspParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeVec3Attribute(omniWriter, ch, oh, ah.suspAttachmentPosAH, suspParams.suspensionAttachment.p);
	writeQuatAttribute(omniWriter, ch, oh, ah.suspAttachmentQuatAH, suspParams.suspensionAttachment.q);
	writeVec3Attribute(omniWriter, ch, oh, ah.suspDirAH, suspParams.suspensionTravelDir);
	writeFloatAttribute(omniWriter, ch, oh, ah.suspTravleDistAH, suspParams.suspensionTravelDist);
	writeVec3Attribute(omniWriter, ch, oh, ah.wheelAttachmentPosAH, suspParams.wheelAttachment.p);
	writeQuatAttribute(omniWriter, ch, oh, ah.wheelAttachmentQuatAH, suspParams.wheelAttachment.q);
}


SuspCompParams registerSuspComplianceParams(OmniPvdWriter& omniWriter)
{
	SuspCompParams s;
	s.CH = omniWriter.registerClass("SuspensionComplianceParams");
	s.toeAngleAH = omniWriter.registerAttribute(s.CH, "toeAngle", OmniPvdDataType::eFLOAT32, 6);
	s.camberAngleAH = omniWriter.registerAttribute(s.CH, "camberAngle", OmniPvdDataType::eFLOAT32, 6);
	s.suspForceAppPointAH = omniWriter.registerAttribute(s.CH, "suspForceAppPoint", OmniPvdDataType::eFLOAT32, 12);
	s.tireForceAppPointAH = omniWriter.registerAttribute(s.CH, "tireForceAppPoint", OmniPvdDataType::eFLOAT32, 12);
	return s;
}

void writeSuspComplianceParams
(const PxVehicleSuspensionComplianceParams& compParams, 
 const OmniPvdObjectHandle oh, const SuspCompParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeLookupTableAttribute(omniWriter, ch, oh, ah.camberAngleAH, compParams.wheelCamberAngle);
	writeLookupTableAttribute(omniWriter, ch, oh, ah.toeAngleAH, compParams.wheelToeAngle);
	writeLookupTableAttribute(omniWriter, ch, oh, ah.suspForceAppPointAH, compParams.suspForceAppPoint);
	writeLookupTableAttribute(omniWriter, ch, oh, ah.tireForceAppPointAH, compParams.tireForceAppPoint);
}

SuspForceParams registerSuspForceParams(OmniPvdWriter& omniWriter)
{
	SuspForceParams s;
	s.CH = omniWriter.registerClass("SuspensionForceParams");
	s.stiffnessAH = omniWriter.registerAttribute(s.CH, "stiffness", OmniPvdDataType::eFLOAT32, 1);
	s.dampingAH = omniWriter.registerAttribute(s.CH, "damping", OmniPvdDataType::eFLOAT32, 1);
	s.sprungMassAH = omniWriter.registerAttribute(s.CH, "sprungMass", OmniPvdDataType::eFLOAT32, 1);
	return s;
}

void writeSuspForceParams
(const PxVehicleSuspensionForceParams& forceParams, 
 const OmniPvdObjectHandle oh, const SuspForceParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.dampingAH, forceParams.damping);
	writeFloatAttribute(omniWriter, ch, oh, ah.sprungMassAH, forceParams.sprungMass);
	writeFloatAttribute(omniWriter, ch, oh, ah.stiffnessAH, forceParams.stiffness);
}

SuspState registerSuspState(OmniPvdWriter& omniWriter)
{
	SuspState s;
	s.CH = omniWriter.registerClass("SuspensionState");
	s.jounceAH = omniWriter.registerAttribute(s.CH, "jounce", OmniPvdDataType::eFLOAT32, 1);
	s.jounceSpeedAH = omniWriter.registerAttribute(s.CH, "jounceSpeed", OmniPvdDataType::eFLOAT32, 1);
	s.separationAH = omniWriter.registerAttribute(s.CH, "separation", OmniPvdDataType::eFLOAT32, 1);
	return s;
}

void writeSuspState
(const PxVehicleSuspensionState& suspState, 
 const OmniPvdObjectHandle oh, const SuspState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.jounceAH, suspState.jounce);
	writeFloatAttribute(omniWriter, ch, oh, ah.jounceSpeedAH, suspState.jounceSpeed);
	writeFloatAttribute(omniWriter, ch, oh, ah.separationAH, suspState.separation);									
}

SuspCompState registerSuspComplianceState(OmniPvdWriter& omniWriter)
{
	SuspCompState s;
	s.CH = omniWriter.registerClass("SuspensionComplianceState");
	s.toeAH = omniWriter.registerAttribute(s.CH, "toe", OmniPvdDataType::eFLOAT32, 1);
	s.camberAH = omniWriter.registerAttribute(s.CH, "camber", OmniPvdDataType::eFLOAT32, 1);
	s.tireForceAppPointAH = omniWriter.registerAttribute(s.CH, "tireForceAppPoint", OmniPvdDataType::eFLOAT32, 3);
	s.suspForceAppPointAH = omniWriter.registerAttribute(s.CH, "suspForceAppPoint", OmniPvdDataType::eFLOAT32, 3);
	return s;
}

void writeSuspComplianceState
(const PxVehicleSuspensionComplianceState& suspCompState, 
 const OmniPvdObjectHandle oh, const SuspCompState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.camberAH, suspCompState.camber);
	writeFloatAttribute(omniWriter, ch, oh, ah.toeAH, suspCompState.toe);
	writeVec3Attribute(omniWriter, ch, oh, ah.suspForceAppPointAH, suspCompState.suspForceAppPoint);
	writeVec3Attribute(omniWriter, ch, oh, ah.tireForceAppPointAH, suspCompState.tireForceAppPoint);
}

SuspForce registerSuspForce(OmniPvdWriter& omniWriter)
{
	SuspForce s;
	s.CH = omniWriter.registerClass("SuspensionForce");
	s.forceAH = omniWriter.registerAttribute(s.CH, "force", OmniPvdDataType::eFLOAT32, 3);
	s.torqueAH = omniWriter.registerAttribute(s.CH, "torque", OmniPvdDataType::eFLOAT32, 3);
	s.normalForceAH = omniWriter.registerAttribute(s.CH, "normalForce", OmniPvdDataType::eFLOAT32, 3);
	return s;
}

void writeSuspForce
(const PxVehicleSuspensionForce& suspForce, 
 const OmniPvdObjectHandle oh, const SuspForce& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeVec3Attribute(omniWriter, ch, oh, ah.forceAH, suspForce.force);
	writeVec3Attribute(omniWriter, ch, oh, ah.torqueAH, suspForce.torque);
	writeFloatAttribute(omniWriter, ch, oh, ah.normalForceAH, suspForce.normalForce);
}

TireParams registerTireParams(OmniPvdWriter& omniWriter)
{
	TireParams t;
	t.CH = omniWriter.registerClass("TireParams");
	t.latStiffXAH = omniWriter.registerAttribute(t.CH, "latStiffX", OmniPvdDataType::eFLOAT32, 1);
	t.latStiffYAH = omniWriter.registerAttribute(t.CH, "latStiffY", OmniPvdDataType::eFLOAT32, 1);
	t.longStiffAH = omniWriter.registerAttribute(t.CH, "longStiff", OmniPvdDataType::eFLOAT32, 1);
	t.camberStiffAH = omniWriter.registerAttribute(t.CH, "camberStiff", OmniPvdDataType::eFLOAT32, 1);
	t.frictionVsSlipAH = omniWriter.registerAttribute(t.CH, "frictionVsSlip", OmniPvdDataType::eFLOAT32, 6);
	t.restLoadAH = omniWriter.registerAttribute(t.CH, "restLoad", OmniPvdDataType::eFLOAT32, 1);
	t.loadFilterAH = omniWriter.registerAttribute(t.CH, "loadFilter", OmniPvdDataType::eFLOAT32, 4);
	return t;
}

void writeTireParams
(const PxVehicleTireForceParams& tireParams, 
 const OmniPvdObjectHandle oh, const TireParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
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

TireDirectionState registerTireDirectionState(OmniPvdWriter& omniWriter)
{
	TireDirectionState t;
	t.CH = omniWriter.registerClass("TireDirectionState");
	t.lngDirectionAH = omniWriter.registerAttribute(t.CH, "lngDir", OmniPvdDataType::eFLOAT32, 3);
	t.latDirectionAH = omniWriter.registerAttribute(t.CH, "latDir", OmniPvdDataType::eFLOAT32, 3);
	return t;
}

void writeTireDirectionState
(const PxVehicleTireDirectionState& tireDirState,
 const OmniPvdObjectHandle oh, const TireDirectionState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeVec3Attribute(omniWriter, ch, oh, ah.lngDirectionAH, tireDirState.directions[PxVehicleTireDirectionModes::eLONGITUDINAL]);
	writeVec3Attribute(omniWriter, ch, oh, ah.latDirectionAH, tireDirState.directions[PxVehicleTireDirectionModes::eLATERAL]);
}

TireSpeedState registerTireSpeedState(OmniPvdWriter& omniWriter)
{
	TireSpeedState t;
	t.CH = omniWriter.registerClass("TireSpeedState");
	t.lngSpeedAH = omniWriter.registerAttribute(t.CH, "lngSpeed", OmniPvdDataType::eFLOAT32, 1);
	t.latSpeedAH = omniWriter.registerAttribute(t.CH, "latSpeed", OmniPvdDataType::eFLOAT32, 1);
	return t;
}

void writeTireSpeedState
(const PxVehicleTireSpeedState& tireSpeedState,
 const OmniPvdObjectHandle oh, const TireSpeedState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.lngSpeedAH, tireSpeedState.speedStates[PxVehicleTireDirectionModes::eLONGITUDINAL]);
	writeFloatAttribute(omniWriter, ch, oh, ah.latSpeedAH, tireSpeedState.speedStates[PxVehicleTireDirectionModes::eLATERAL]);
}

TireSlipState registerTireSlipState(OmniPvdWriter& omniWriter)
{
	TireSlipState t;
	t.CH = omniWriter.registerClass("TireSlipState");
	t.lngSlipAH = omniWriter.registerAttribute(t.CH, "lngSlip", OmniPvdDataType::eFLOAT32, 1);
	t.latSlipAH = omniWriter.registerAttribute(t.CH, "latSlip", OmniPvdDataType::eFLOAT32, 1);
	return t;
}

void writeTireSlipState
(const PxVehicleTireSlipState& tireSlipState,
 const OmniPvdObjectHandle oh, const TireSlipState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.lngSlipAH, tireSlipState.slips[PxVehicleTireDirectionModes::eLONGITUDINAL]);
	writeFloatAttribute(omniWriter, ch, oh, ah.latSlipAH, tireSlipState.slips[PxVehicleTireDirectionModes::eLATERAL]);
}

TireStickyState registerTireStickyState(OmniPvdWriter& omniWriter)
{
	struct BoolAsEnum
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle falseAH;
		OmniPvdAttributeHandle trueAH;
	};
	BoolAsEnum boolAsEnum;
	boolAsEnum.CH = omniWriter.registerClass("StickyTireBool");
	boolAsEnum.falseAH = omniWriter.registerEnumValue(boolAsEnum.CH, "False", 0);
	boolAsEnum.trueAH = omniWriter.registerEnumValue(boolAsEnum.CH, "True", 1);

	TireStickyState t;
	t.CH = omniWriter.registerClass("TireStickyState");
	t.lngStickyStateTimer = omniWriter.registerAttribute(t.CH, "lngStickyTimer", OmniPvdDataType::eFLOAT32, 1);
	t.lngStickyStateStatus = omniWriter.registerFlagsAttribute(t.CH, "lngStickyStatus", boolAsEnum.CH);
	t.latStickyStateTimer = omniWriter.registerAttribute(t.CH, "latStickyTimer", OmniPvdDataType::eFLOAT32, 1);
	t.latStickyStateStatus = omniWriter.registerFlagsAttribute(t.CH, "latStickyStatus", boolAsEnum.CH);
	return t;
}

void writeTireStickyState
(const PxVehicleTireStickyState& tireStickyState,
 const OmniPvdObjectHandle oh, const TireStickyState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFlagAttribute(omniWriter, ch, oh, ah.latStickyStateStatus, tireStickyState.activeStatus[PxVehicleTireDirectionModes::eLATERAL] ? 1 : 0);
	writeFlagAttribute(omniWriter, ch, oh, ah.lngStickyStateStatus, tireStickyState.activeStatus[PxVehicleTireDirectionModes::eLONGITUDINAL] ? 1 : 0);
	writeFloatAttribute(omniWriter, ch, oh, ah.latStickyStateTimer, tireStickyState.lowSpeedTime[PxVehicleTireDirectionModes::eLATERAL]);
	writeFloatAttribute(omniWriter, ch, oh, ah.lngStickyStateTimer, tireStickyState.lowSpeedTime[PxVehicleTireDirectionModes::eLONGITUDINAL]);
}

TireGripState registerTireGripState(OmniPvdWriter& omniWriter)
{
	TireGripState t;
	t.CH = omniWriter.registerClass("TireGripState");
	t.loadAH = omniWriter.registerAttribute(t.CH, "load", OmniPvdDataType::eFLOAT32, 1);
	t.frictionAH = omniWriter.registerAttribute(t.CH, "friction", OmniPvdDataType::eFLOAT32, 1);
	return t;
}

void writeTireGripState
(const PxVehicleTireGripState& tireGripState, 
 const OmniPvdObjectHandle oh, const TireGripState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.frictionAH, tireGripState.friction);
	writeFloatAttribute(omniWriter, ch, oh, ah.loadAH, tireGripState.load);
}

TireCamberState registerTireCamberState(OmniPvdWriter& omniWriter)
{
	TireCamberState t;
	t.CH = omniWriter.registerClass("TireCamberState");
	t.camberAngleAH = omniWriter.registerAttribute(t.CH, "camberAngle", OmniPvdDataType::eFLOAT32, 1);
	return t;
}

void writeTireCamberState
(const PxVehicleTireCamberAngleState& tireCamberState,
 const OmniPvdObjectHandle oh, const TireCamberState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.camberAngleAH, tireCamberState.camberAngle);
}

TireForce registerTireForce(OmniPvdWriter& omniWriter)
{
	TireForce t;
	t.CH = omniWriter.registerClass("TireForce");

	t.lngForceAH = omniWriter.registerAttribute(t.CH, "lngForce", OmniPvdDataType::eFLOAT32, 3);
	t.lngTorqueAH = omniWriter.registerAttribute(t.CH, "lngTorque", OmniPvdDataType::eFLOAT32, 3);

	t.latForceAH = omniWriter.registerAttribute(t.CH, "latForce", OmniPvdDataType::eFLOAT32, 3);
	t.latTorqueAH = omniWriter.registerAttribute(t.CH, "latTorque", OmniPvdDataType::eFLOAT32, 3);

	t.aligningMomentAH = omniWriter.registerAttribute(t.CH, "aligningMoment", OmniPvdDataType::eFLOAT32, 3);
	t.wheelTorqueAH = omniWriter.registerAttribute(t.CH, "wheelTorque", OmniPvdDataType::eFLOAT32, 3);

	return t;
}

void writeTireForce
(const PxVehicleTireForce& tireForce,
 const OmniPvdObjectHandle oh, const TireForce& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeVec3Attribute(omniWriter, ch, oh, ah.lngForceAH, tireForce.forces[PxVehicleTireDirectionModes::eLONGITUDINAL]);
	writeVec3Attribute(omniWriter, ch, oh, ah.lngTorqueAH, tireForce.torques[PxVehicleTireDirectionModes::eLONGITUDINAL]);
	writeVec3Attribute(omniWriter, ch, oh, ah.latForceAH, tireForce.forces[PxVehicleTireDirectionModes::eLATERAL]);
	writeVec3Attribute(omniWriter, ch, oh, ah.latTorqueAH, tireForce.torques[PxVehicleTireDirectionModes::eLATERAL]);
	writeFloatAttribute(omniWriter, ch, oh, ah.aligningMomentAH, tireForce.aligningMoment);
	writeFloatAttribute(omniWriter, ch, oh, ah.wheelTorqueAH, tireForce.wheelTorque);
}

WheelAttachment registerWheelAttachment(OmniPvdWriter& omniWriter)
{
	WheelAttachment w;
	w.CH = omniWriter.registerClass("WheelAttachment");
	w.wheelParamsAH = omniWriter.registerAttribute(w.CH, "wheelParams", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.wheelActuationStateAH = omniWriter.registerAttribute(w.CH, "wheelActuationState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.wheelRigidBody1dStateAH = omniWriter.registerAttribute(w.CH, "wheelRigidBody1dState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.wheelLocalPoseStateAH = omniWriter.registerAttribute(w.CH, "wheelLocalPosetate", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.roadGeomStateAH = omniWriter.registerAttribute(w.CH, "roadGeomState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.suspParamsAH = omniWriter.registerAttribute(w.CH, "suspParams", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.suspCompParamsAH = omniWriter.registerAttribute(w.CH, "suspComplianceParams", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.suspForceParamsAH = omniWriter.registerAttribute(w.CH, "suspForceParams", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.suspStateAH = omniWriter.registerAttribute(w.CH, "suspState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.suspCompStateAH = omniWriter.registerAttribute(w.CH, "suspComplianceState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.suspForceAH = omniWriter.registerAttribute(w.CH, "suspForce", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.tireParamsAH = omniWriter.registerAttribute(w.CH, "tireParams", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.tireDirectionStateAH = omniWriter.registerAttribute(w.CH, "tireDirectionState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.tireSpeedStateAH = omniWriter.registerAttribute(w.CH, "tireSpeedState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.tireSlipStateAH = omniWriter.registerAttribute(w.CH, "tireSlipState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.tireStickyStateAH = omniWriter.registerAttribute(w.CH, "tireStickyState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.tireGripStateAH = omniWriter.registerAttribute(w.CH, "tireGripState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.tireCamberStateAH = omniWriter.registerAttribute(w.CH, "tireCamberState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.tireForceAH = omniWriter.registerAttribute(w.CH, "tireForce", OmniPvdDataType::eOBJECT_HANDLE, 1);
	return w;
}

//////////////////////////
//ANTIROLL
//////////////////////////

AntiRollParams registerAntiRollParams(OmniPvdWriter& omniWriter)
{ 
	AntiRollParams a; 
	a.CH = omniWriter.registerClass("AntiRollParams");
	a.wheel0AH = omniWriter.registerAttribute(a.CH, "wheel0", OmniPvdDataType::eUINT32, 1);
	a.wheel1AH = omniWriter.registerAttribute(a.CH, "wheel1", OmniPvdDataType::eUINT32, 1);
	a.stiffnessAH = omniWriter.registerAttribute(a.CH, "stiffness", OmniPvdDataType::eFLOAT32, 1);
	return a;
}

void writeAntiRollParams
(const PxVehicleAntiRollForceParams& antiRollParams,
 const OmniPvdObjectHandle oh, const AntiRollParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeUInt32Attribute(omniWriter, ch, oh, ah.wheel0AH, antiRollParams.wheel0);
	writeUInt32Attribute(omniWriter, ch, oh, ah.wheel1AH, antiRollParams.wheel1);
	writeFloatAttribute(omniWriter, ch, oh, ah.stiffnessAH, antiRollParams.stiffness);
}

AntiRollForce registerAntiRollForce(OmniPvdWriter& omniWriter)
{
	AntiRollForce a;
	a.CH = omniWriter.registerClass("AntiRollForce");
	a.torqueAH = omniWriter.registerAttribute(a.CH, "torque", OmniPvdDataType::eFLOAT32, 3);
	return a;
}

void writeAntiRollForce
(const PxVehicleAntiRollTorque& antiRollForce, 
 const OmniPvdObjectHandle oh, const AntiRollForce& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeVec3Attribute(omniWriter, ch, oh, ah.torqueAH, antiRollForce.antiRollTorque);
}


//////////////////////////////////
//SUSPENSION STATE CALCULATION
//////////////////////////////////

SuspStateCalcParams registerSuspStateCalcParams(OmniPvdWriter& omniWriter)
{
	struct SuspJounceCalcType
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle raycastAH;
		OmniPvdAttributeHandle sweepAH;
		OmniPvdAttributeHandle noneAH;
	};
	SuspJounceCalcType jounceCalcType;
	jounceCalcType.CH = omniWriter.registerClass("SuspJounceCalculationType");
	jounceCalcType.raycastAH = omniWriter.registerEnumValue(jounceCalcType.CH, "raycast", PxVehicleSuspensionJounceCalculationType::eRAYCAST);
	jounceCalcType.sweepAH = omniWriter.registerEnumValue(jounceCalcType.CH, "sweep", PxVehicleSuspensionJounceCalculationType::eSWEEP);
	jounceCalcType.noneAH = omniWriter.registerEnumValue(jounceCalcType.CH, "none", PxVehicleSuspensionJounceCalculationType::eMAX_NB);

	struct BoolAsEnum
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle falseAH;
		OmniPvdAttributeHandle trueAH;
	};
	BoolAsEnum boolAsEnum;
	boolAsEnum.CH = omniWriter.registerClass("SuspStateCalcParamsBool");
	boolAsEnum.falseAH = omniWriter.registerEnumValue(boolAsEnum.CH, "False", 0);
	boolAsEnum.trueAH = omniWriter.registerEnumValue(boolAsEnum.CH, "True", 1);

	SuspStateCalcParams s;
	s.CH = omniWriter.registerClass("SuspStateCalculationParams");
	s.calcTypeAH = omniWriter.registerFlagsAttribute(s.CH, "suspJounceCalculationType", jounceCalcType.CH);
	s.limitExpansionValAH = omniWriter.registerFlagsAttribute(s.CH, "limitSuspensionExpansionVelocity", boolAsEnum.CH);
	return s;
}

void writeSuspStateCalcParams
(const PxVehicleSuspensionStateCalculationParams& suspStateCalcParams,
 const OmniPvdObjectHandle oh, const SuspStateCalcParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFlagAttribute(omniWriter, ch, oh, ah.limitExpansionValAH, suspStateCalcParams.limitSuspensionExpansionVelocity ? 1 : 0);
	writeFlagAttribute(omniWriter, ch, oh, ah.calcTypeAH, suspStateCalcParams.suspensionJounceCalculationType);
}

//////////////////////////////////////
//DIRECT DRIVETRAIN
//////////////////////////////////////

DirectDriveCommandState registerDirectDriveCommandState(OmniPvdWriter& omniWriter)
{
	DirectDriveCommandState c;
	c.CH = omniWriter.registerClass("DirectDriveCommandState");
	c.brakesAH= omniWriter.registerAttribute(c.CH, "brakes", OmniPvdDataType::eFLOAT32, 2);
	c.throttleAH= omniWriter.registerAttribute(c.CH, "throttle", OmniPvdDataType::eFLOAT32, 1);
	c.steerAH= omniWriter.registerAttribute(c.CH, "steer", OmniPvdDataType::eFLOAT32, 1);
	return c;	
}

void writeDirectDriveCommandState
(const PxVehicleCommandState& commands,
 const OmniPvdObjectHandle oh, const DirectDriveCommandState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
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

DirectDriveTransmissionCommandState registerDirectDriveTransmissionCommandState(OmniPvdWriter& omniWriter)
{
	struct DirectDriveGear
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle reverse;
		OmniPvdAttributeHandle neutral;
		OmniPvdAttributeHandle forward;
	};

	DirectDriveGear g;
	g.CH = omniWriter.registerClass("DirectDriveGear");
	g.reverse = omniWriter.registerEnumValue(g.CH, "reverse", PxVehicleDirectDriveTransmissionCommandState::eREVERSE);
	g.neutral = omniWriter.registerEnumValue(g.CH, "neutral", PxVehicleDirectDriveTransmissionCommandState::eNEUTRAL);
	g.forward = omniWriter.registerEnumValue(g.CH, "forward", PxVehicleDirectDriveTransmissionCommandState::eFORWARD);

	DirectDriveTransmissionCommandState c;
	c.CH = omniWriter.registerClass("DirectDriveTransmissionCommandState");
	c.gearAH = omniWriter.registerFlagsAttribute(c.CH, "gear", g.CH);
	return c;	
}

void writeDirectDriveTransmissionCommandState
(const PxVehicleDirectDriveTransmissionCommandState& transmission,
 const OmniPvdObjectHandle oh, const DirectDriveTransmissionCommandState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFlagAttribute(omniWriter, ch, oh, ah.gearAH, transmission.gear);
}

WheelResponseParams registerDirectDriveThrottleResponseParams(OmniPvdWriter& omniWriter)
{
	return registerWheelResponseParams("DirectDriveThrottleResponseParams", omniWriter);
}

void writeDirectDriveThrottleResponseParams
(const PxVehicleAxleDescription& axleDesc,
 const PxVehicleDirectDriveThrottleCommandResponseParams& directDriveThrottleResponseParams,
 const OmniPvdObjectHandle oh, const WheelResponseParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeWheelResponseParams(axleDesc, directDriveThrottleResponseParams, oh, ah, omniWriter, ch);
}

DirectDriveThrottleResponseState registerDirectDriveThrottleResponseState(OmniPvdWriter& omniWriter)
{
	DirectDriveThrottleResponseState d;
	d.CH = omniWriter.registerClass("DirectDriveThrottleResponseState");
	d.states0To3AH = omniWriter.registerAttribute(d.CH, "responseState0To3", OmniPvdDataType::eFLOAT32, 4);
	d.states4To7AH = omniWriter.registerAttribute(d.CH, "responseState4To7", OmniPvdDataType::eFLOAT32, 4);
	d.states8To11AH = omniWriter.registerAttribute(d.CH, "responseState8To11", OmniPvdDataType::eFLOAT32, 4);
	d.states12To15AH = omniWriter.registerAttribute(d.CH, "responseState12To15", OmniPvdDataType::eFLOAT32, 4);
	d.states16To19AH = omniWriter.registerAttribute(d.CH, "responseState16To19", OmniPvdDataType::eFLOAT32, 4);
	return d;
}

void writeDirectDriveThrottleResponseState
(const PxVehicleAxleDescription& axleDesc, const PxVehicleArrayData<PxReal>& throttleResponseState,
 const OmniPvdObjectHandle oh, const DirectDriveThrottleResponseState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
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

DirectDrivetrain registerDirectDrivetrain(OmniPvdWriter& omniWriter)
{
	DirectDrivetrain d;
	d.CH = omniWriter.registerClass("DirectDrivetrain");
	d.throttleResponseParamsAH = omniWriter.registerAttribute(d.CH, "throttleResponseParams", OmniPvdDataType::eOBJECT_HANDLE, 1);
	d.commandStateAH = omniWriter.registerAttribute(d.CH, "commandState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	d.transmissionCommandStateAH = omniWriter.registerAttribute(d.CH, "transmissionCommandState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	d.throttleResponseStateAH = omniWriter.registerAttribute(d.CH, "throttleResponseState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	return d;
}


//////////////////////////////
//ENGINE DRIVETRAIN
//////////////////////////////

EngineDriveCommandState registerEngineDriveCommandState(OmniPvdWriter& omniWriter)
{
	EngineDriveCommandState c;
	c.CH = omniWriter.registerClass("EngineDriveCommandState");
	c.brakesAH = omniWriter.registerAttribute(c.CH, "brakes", OmniPvdDataType::eFLOAT32, 2);
	c.throttleAH = omniWriter.registerAttribute(c.CH, "throttle", OmniPvdDataType::eFLOAT32, 1);
	c.steerAH = omniWriter.registerAttribute(c.CH, "steer", OmniPvdDataType::eFLOAT32, 1);
	return c;
}


void writeEngineDriveCommandState
(const PxVehicleCommandState& commands,
 const OmniPvdObjectHandle oh, const EngineDriveCommandState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
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

EngineDriveTransmissionCommandState registerEngineDriveTransmissionCommandState(OmniPvdWriter& omniWriter)
{
	EngineDriveTransmissionCommandState c;
	c.CH = omniWriter.registerClass("EngineDriveTransmissionCommandState");
	c.gearAH = omniWriter.registerAttribute(c.CH, "targetGear", OmniPvdDataType::eUINT32, 1);	
	c.clutchAH = omniWriter.registerAttribute(c.CH, "clutch", OmniPvdDataType::eFLOAT32, 1);	
	return c;
}

void writeEngineDriveTransmissionCommandState
(const PxVehicleEngineDriveTransmissionCommandState& transmission,
 const OmniPvdObjectHandle oh, const EngineDriveTransmissionCommandState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeUInt32Attribute(omniWriter, ch, oh, ah.gearAH, transmission.targetGear);
	writeFloatAttribute(omniWriter, ch, oh, ah.clutchAH, transmission.clutch);
}

static const PxU32 tankThrustsCommandEntryCount = sizeof(PxVehicleTankDriveTransmissionCommandState::thrusts) / sizeof(PxVehicleTankDriveTransmissionCommandState::thrusts[0]);

TankDriveTransmissionCommandState registerTankDriveTransmissionCommandState(OmniPvdWriter& omniWriter, OmniPvdClassHandle baseClass)
{
	TankDriveTransmissionCommandState t;
	t.CH = omniWriter.registerClass("TankDriveTransmissionCommandState", baseClass);
	t.thrustsAH = omniWriter.registerAttribute(t.CH, "thrusts", OmniPvdDataType::eFLOAT32, tankThrustsCommandEntryCount);	
	return t;
}

void writeTankDriveTransmissionCommandState
(const PxVehicleTankDriveTransmissionCommandState& transmission,
 const OmniPvdObjectHandle oh, const EngineDriveTransmissionCommandState& engineDriveAH, 
 const TankDriveTransmissionCommandState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeEngineDriveTransmissionCommandState(transmission, oh, engineDriveAH, omniWriter, ch);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.thrustsAH, transmission.thrusts, tankThrustsCommandEntryCount);
}

ClutchResponseParams registerClutchResponseParams(OmniPvdWriter& omniWriter)
{
	ClutchResponseParams c;
	c.CH = omniWriter.registerClass("ClutchResponseParams");
	c.maxResponseAH = omniWriter.registerAttribute(c.CH, "MaxResponse", OmniPvdDataType::eFLOAT32, 1);
	return c;
}

ClutchParams registerClutchParams(OmniPvdWriter& omniWriter)
{
	struct VehicleClutchAccuracyMode
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle estimateAH;
		OmniPvdAttributeHandle bestPossibleAH;
	};
	VehicleClutchAccuracyMode mode;
	mode.CH = omniWriter.registerClass("ClutchAccuracyMode");
	mode.estimateAH = omniWriter.registerEnumValue(mode.CH, "estimate", PxVehicleClutchAccuracyMode::eESTIMATE);
	mode.bestPossibleAH = omniWriter.registerEnumValue(mode.CH, "bestPossible", PxVehicleClutchAccuracyMode::eBEST_POSSIBLE);

	ClutchParams v;
	v.CH = omniWriter.registerClass("ClutchParams");
	v.accuracyAH = omniWriter.registerFlagsAttribute(v.CH, "accuracyMode", mode.CH);
	v.iterationsAH = omniWriter.registerAttribute(v.CH, "iterations", OmniPvdDataType::eUINT32, 1);
	return v;	
}

void writeClutchParams
(const PxVehicleClutchParams& clutchParams, 
 const OmniPvdObjectHandle oh, const ClutchParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFlagAttribute(omniWriter, ch, oh, ah.accuracyAH, clutchParams.accuracyMode);
	writeUInt32Attribute(omniWriter, ch, oh, ah.iterationsAH, clutchParams.estimateIterations);
}

EngineParams registerEngineParams(OmniPvdWriter& omniWriter)
{
	EngineParams e;
	e.CH = omniWriter.registerClass("EngineParams");
	e.torqueCurveAH = omniWriter.registerAttribute(e.CH, "torqueCurve", OmniPvdDataType::eFLOAT32, PxVehicleEngineParams::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES*2);
	e.peakTorqueAH = omniWriter.registerAttribute(e.CH, "peakTorque", OmniPvdDataType::eFLOAT32, 1); 
	e.moiAH = omniWriter.registerAttribute(e.CH, "moi", OmniPvdDataType::eFLOAT32, 1); 
	e.idleOmegaAH = omniWriter.registerAttribute(e.CH, "idleOmega", OmniPvdDataType::eFLOAT32, 1); 
	e.maxOmegaAH = omniWriter.registerAttribute(e.CH, "maxOmega", OmniPvdDataType::eFLOAT32, 1); 
	e.dampingRateFullThrottleAH = omniWriter.registerAttribute(e.CH, "dampingRateFullThrottleAH", OmniPvdDataType::eFLOAT32, 1); 
	e.dampingRateZeroThrottleClutchDisengagedAH = omniWriter.registerAttribute(e.CH, "dampingRateZeroThrottleClutchDisengaged", OmniPvdDataType::eFLOAT32, 1); 
	e.dampingRateZeroThrottleClutchEngagedAH = omniWriter.registerAttribute(e.CH, "dampingRateZeroThrottleClutchEngaged", OmniPvdDataType::eFLOAT32, 1); 
	return e;
}

void writeEngineParams
(const PxVehicleEngineParams& engineParams, 
 const OmniPvdObjectHandle oh, const EngineParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
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

GearboxParams registerGearboxParams(OmniPvdWriter& omniWriter)
{
	GearboxParams g;
	g.CH = omniWriter.registerClass("GearboxParams");
	g.ratiosAH = omniWriter.registerAttribute(g.CH, "ratios", OmniPvdDataType::eFLOAT32, PxVehicleGearboxParams::eMAX_NB_GEARS); 
	g.nbRatiosAH =  omniWriter.registerAttribute(g.CH, "nbRatios", OmniPvdDataType::eUINT32, 1); 
	g.neutralGearAH =  omniWriter.registerAttribute(g.CH, "neutralGear", OmniPvdDataType::eUINT32, 1); 
	g.finalRatioAH =  omniWriter.registerAttribute(g.CH, "finalRatio", OmniPvdDataType::eFLOAT32, 1); 
	g.switchTimeAH =  omniWriter.registerAttribute(g.CH, "switchTime", OmniPvdDataType::eFLOAT32, 1); 
	return g;
}

void writeGearboxParams
(const PxVehicleGearboxParams& gearboxParams, 
 const OmniPvdObjectHandle oh, const GearboxParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
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

AutoboxParams registerAutoboxParams(OmniPvdWriter& omniWriter)
{
	AutoboxParams a;
	a.CH = omniWriter.registerClass("AutoboxParams");
	a.upRatiosAH = omniWriter.registerAttribute(a.CH, "upRatios", OmniPvdDataType::eFLOAT32, PxVehicleGearboxParams::eMAX_NB_GEARS); 
	a.downRatiosAH = omniWriter.registerAttribute(a.CH, "downRatios", OmniPvdDataType::eFLOAT32, PxVehicleGearboxParams::eMAX_NB_GEARS); 
	a.latencyAH = omniWriter.registerAttribute(a.CH, "latency", OmniPvdDataType::eFLOAT32, 1); 
	return a;
}

void writeAutoboxParams
(const PxVehicleAutoboxParams& autoboxParams, 
 const OmniPvdObjectHandle oh, const AutoboxParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.upRatiosAH, autoboxParams.upRatios, PxVehicleGearboxParams::eMAX_NB_GEARS);
	writeFloatArrayAttribute(omniWriter, ch, oh, ah.downRatiosAH, autoboxParams.downRatios, PxVehicleGearboxParams::eMAX_NB_GEARS);
	writeFloatAttribute(omniWriter, ch, oh, ah.latencyAH, autoboxParams.latency);
}

MultiWheelDiffParams registerMultiWheelDiffParams(OmniPvdWriter& omniWriter)
{
	MultiWheelDiffParams m;
	m.CH = omniWriter.registerClass("MultiWheelDiffParams");
	m.torqueRatios0To3AH = omniWriter.registerAttribute(m.CH, "torqueRatios0To3", OmniPvdDataType::eFLOAT32, 4); 
	m.torqueRatios4To7AH = omniWriter.registerAttribute(m.CH, "torqueRatios4To7", OmniPvdDataType::eFLOAT32, 4); 
	m.torqueRatios8To11AH = omniWriter.registerAttribute(m.CH, "torqueRatios8To11", OmniPvdDataType::eFLOAT32, 4); 
	m.torqueRatios12To15AH = omniWriter.registerAttribute(m.CH, "torqueRatios12To15", OmniPvdDataType::eFLOAT32, 4); 
	m.torqueRatios16To19AH = omniWriter.registerAttribute(m.CH, "torqueRatios16To19", OmniPvdDataType::eFLOAT32, 4); 
	m.aveWheelSpeedRatios0To3AH = omniWriter.registerAttribute(m.CH, "aveWheelSpeedRatios0To3", OmniPvdDataType::eFLOAT32, 4); 
	m.aveWheelSpeedRatios4To7AH = omniWriter.registerAttribute(m.CH, "aveWheelSpeedRatios4To7", OmniPvdDataType::eFLOAT32, 4); 
	m.aveWheelSpeedRatios8To11AH = omniWriter.registerAttribute(m.CH, "aveWheelSpeedRatios8To11", OmniPvdDataType::eFLOAT32, 4); 
	m.aveWheelSpeedRatios12To15AH = omniWriter.registerAttribute(m.CH, "aveWheelSpeedRatios12To15", OmniPvdDataType::eFLOAT32, 4); 
	m.aveWheelSpeedRatios16To19AH = omniWriter.registerAttribute(m.CH, "aveWheelSpeedRatios16To19", OmniPvdDataType::eFLOAT32, 4); 
	return m;
}

FourWheelDiffParams registerFourWheelDiffParams(OmniPvdWriter& omniWriter, OmniPvdClassHandle baseClass)
{
	FourWheelDiffParams m;
	m.CH = omniWriter.registerClass("FourWheelDiffParams", baseClass);
	m.frontBiasAH = omniWriter.registerAttribute(m.CH, "frontBias", OmniPvdDataType::eFLOAT32, 1);
	m.frontTargetAH = omniWriter.registerAttribute(m.CH, "frontTarget", OmniPvdDataType::eFLOAT32, 1);
	m.rearBiasAH = omniWriter.registerAttribute(m.CH, "rearBias", OmniPvdDataType::eFLOAT32, 1);
	m.rearTargetAH = omniWriter.registerAttribute(m.CH, "rearTarget", OmniPvdDataType::eFLOAT32, 1);
	m.centreBiasAH = omniWriter.registerAttribute(m.CH, "centerBias", OmniPvdDataType::eFLOAT32, 1);
	m.centreTargetAH = omniWriter.registerAttribute(m.CH, "centerTarget", OmniPvdDataType::eFLOAT32, 1);
	m.frontWheelsAH = omniWriter.registerAttribute(m.CH, "frontWheels", OmniPvdDataType::eUINT32, 2);
	m.rearWheelsAH = omniWriter.registerAttribute(m.CH, "rearWheels", OmniPvdDataType::eUINT32, 2);
	return m;
}

TankDiffParams registerTankDiffParams(OmniPvdWriter& omniWriter, OmniPvdClassHandle baseClass)
{
	TankDiffParams t;
	t.CH = omniWriter.registerClass("TankDiffParams", baseClass);
	t.nbTracksAH = omniWriter.registerAttribute(t.CH, "nbTracks", OmniPvdDataType::eUINT32, 1);
	t.thrustIdPerTrackAH = omniWriter.registerAttribute(t.CH, "thrustIdPerTrack", OmniPvdDataType::eUINT32, 0);
	t.nbWheelsPerTrackAH = omniWriter.registerAttribute(t.CH, "nbWheelsPerTrack", OmniPvdDataType::eUINT32, 0);
	t.trackToWheelIdsAH = omniWriter.registerAttribute(t.CH, "trackToWheelIds", OmniPvdDataType::eUINT32, 0);
	t.wheelIdsInTrackOrderAH = omniWriter.registerAttribute(t.CH, "wheelIdsInTrackOrder", OmniPvdDataType::eUINT32, 0);
	return t;
}

void writeMultiWheelDiffParams
(const PxVehicleMultiWheelDriveDifferentialParams& diffParams, 
 const OmniPvdObjectHandle oh, const MultiWheelDiffParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
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
 const OmniPvdObjectHandle oh, const MultiWheelDiffParams& multiWheelDiffAH, const FourWheelDiffParams& ah, 
 OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeMultiWheelDiffParams(diffParams, oh, multiWheelDiffAH, omniWriter, ch);
	writeFloatAttribute(omniWriter, ch, oh, ah.frontBiasAH, diffParams.frontBias);
	writeFloatAttribute(omniWriter, ch, oh, ah.frontTargetAH, diffParams.frontTarget);
	writeFloatAttribute(omniWriter, ch, oh, ah.rearBiasAH, diffParams.rearBias);
	writeFloatAttribute(omniWriter, ch, oh, ah.rearTargetAH, diffParams.rearTarget);
	writeFloatAttribute(omniWriter, ch, oh, ah.centreBiasAH, diffParams.centerBias);
	writeFloatAttribute(omniWriter, ch, oh, ah.centreTargetAH, diffParams.centerTarget);
	writeUInt32ArrayAttribute(omniWriter, ch, oh, ah.frontWheelsAH, diffParams.frontWheelIds, 2);
	writeUInt32ArrayAttribute(omniWriter, ch, oh, ah.rearWheelsAH, diffParams.rearWheelIds, 2);
}

void writeTankDiffParams
(const PxVehicleTankDriveDifferentialParams& diffParams, 
 const OmniPvdObjectHandle oh, const MultiWheelDiffParams& multiWheelDiffAH, const TankDiffParams& ah,
 OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	PxU32 entryCount = 0;
	for (PxU32 i = 0; i < diffParams.nbTracks; i++)
	{
		entryCount = PxMax(entryCount, diffParams.trackToWheelIds[i] + diffParams.nbWheelsPerTrack[i]);
		// users can remove tracks such that there are holes in the wheelIdsInTrackOrder buffer
	}

	writeMultiWheelDiffParams(diffParams, oh, multiWheelDiffAH, omniWriter, ch);
	writeUInt32Attribute(omniWriter, ch, oh, ah.nbTracksAH, diffParams.nbTracks);
	writeUInt32ArrayAttribute(omniWriter, ch, oh, ah.thrustIdPerTrackAH, diffParams.thrustIdPerTrack, diffParams.nbTracks);
	writeUInt32ArrayAttribute(omniWriter, ch, oh, ah.nbWheelsPerTrackAH, diffParams.nbWheelsPerTrack, diffParams.nbTracks);
	writeUInt32ArrayAttribute(omniWriter, ch, oh, ah.trackToWheelIdsAH, diffParams.trackToWheelIds, diffParams.nbTracks);
	writeUInt32ArrayAttribute(omniWriter, ch, oh, ah.wheelIdsInTrackOrderAH, diffParams.wheelIdsInTrackOrder, entryCount);
}

ClutchResponseState registerClutchResponseState(OmniPvdWriter& omniWriter)
{
	ClutchResponseState c;
	c.CH = omniWriter.registerClass("ClutchResponseState");
	c.normalisedResponseAH = omniWriter.registerAttribute(c.CH, "normalisedResponse", OmniPvdDataType::eFLOAT32, 1);
	c.responseAH = omniWriter.registerAttribute(c.CH, "response", OmniPvdDataType::eFLOAT32, 1);
	return c;
}

void writeClutchResponseState
(const PxVehicleClutchCommandResponseState& clutchResponseState, 
 const OmniPvdObjectHandle oh, const ClutchResponseState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.normalisedResponseAH, clutchResponseState.normalisedCommandResponse);
	writeFloatAttribute(omniWriter, ch, oh, ah.responseAH, clutchResponseState.commandResponse);
}

ThrottleResponseState registerThrottleResponseState(OmniPvdWriter& omniWriter)
{
	ThrottleResponseState t;
	t.CH  = omniWriter.registerClass("ThrottleResponseState");
	t.responseAH = omniWriter.registerAttribute(t.CH, "response", OmniPvdDataType::eFLOAT32, 1);
	return t;
}

void writeThrottleResponseState
(const PxVehicleEngineDriveThrottleCommandResponseState& throttleResponseState, 
 const OmniPvdObjectHandle oh, const ThrottleResponseState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.responseAH, throttleResponseState.commandResponse);
}

EngineState registerEngineState(OmniPvdWriter& omniWriter)
{
	EngineState e;
	e.CH = omniWriter.registerClass("EngineState");
	e.rotationSpeedAH = omniWriter.registerAttribute(e.CH, "rotationSpeed", OmniPvdDataType::eFLOAT32, 1);
	return e;
}

void writeEngineState
(const PxVehicleEngineState& engineState, 
 const OmniPvdObjectHandle oh, const EngineState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.rotationSpeedAH, engineState.rotationSpeed);
}

GearboxState registerGearboxState(OmniPvdWriter& omniWriter)
{
	GearboxState g;
	g.CH = omniWriter.registerClass("GearboxState");
	g.currentGearAH = omniWriter.registerAttribute(g.CH, "currentGear", OmniPvdDataType::eUINT32, 1);
	g.targetGearAH = omniWriter.registerAttribute(g.CH, "targetGear", OmniPvdDataType::eUINT32, 1);
	g.switchTimeAH = omniWriter.registerAttribute(g.CH, "switchTime", OmniPvdDataType::eFLOAT32, 1);
	return g;
}

void writeGearboxState
(const PxVehicleGearboxState& gearboxState, 
 const OmniPvdObjectHandle oh, const GearboxState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeUInt32Attribute(omniWriter, ch, oh, ah.currentGearAH, gearboxState.currentGear);
	writeUInt32Attribute(omniWriter, ch, oh, ah.targetGearAH, gearboxState.targetGear);
	writeFloatAttribute(omniWriter, ch, oh, ah.switchTimeAH, gearboxState.gearSwitchTime);
}

AutoboxState registerAutoboxState(OmniPvdWriter& omniWriter)
{
	struct BoolAsEnum
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle falseAH;
		OmniPvdAttributeHandle trueAH;
	};
	BoolAsEnum boolAsEnum;
	boolAsEnum.CH = omniWriter.registerClass("AutoboxStateBool");
	boolAsEnum.falseAH = omniWriter.registerEnumValue(boolAsEnum.CH, "False", 0);
	boolAsEnum.trueAH = omniWriter.registerEnumValue(boolAsEnum.CH, "True", 1);

	AutoboxState a;
	a.CH = omniWriter.registerClass("AutoboxState");
	a.timeSinceLastShiftAH = omniWriter.registerAttribute(a.CH, "timeSinceLastShift", OmniPvdDataType::eFLOAT32, 1);
	a.activeAutoboxGearShiftAH = omniWriter.registerFlagsAttribute(a.CH, "activeAutoboxShift", boolAsEnum.CH);
	return a;
}

void writeAutoboxState
(const PxVehicleAutoboxState& autoboxState, 
 const OmniPvdObjectHandle oh, const AutoboxState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.timeSinceLastShiftAH, autoboxState.timeSinceLastShift);
	writeFlagAttribute(omniWriter, ch, oh, ah.activeAutoboxGearShiftAH, autoboxState.activeAutoboxGearShift ? 1: 0);
}

DiffState registerDiffState(OmniPvdWriter& omniWriter)
{
	DiffState d;
	d.CH = omniWriter.registerClass("DifferentialState");
	d.torqueRatios0To3AH = omniWriter.registerAttribute(d.CH, "torqueRatios0To3", OmniPvdDataType::eFLOAT32, 4); 
	d.torqueRatios4To7AH = omniWriter.registerAttribute(d.CH, "torqueRatios4To7", OmniPvdDataType::eFLOAT32, 4); 
	d.torqueRatios8To11AH = omniWriter.registerAttribute(d.CH, "torqueRatios8To11", OmniPvdDataType::eFLOAT32, 4); 
	d.torqueRatios12To15AH = omniWriter.registerAttribute(d.CH, "torqueRatios12To15", OmniPvdDataType::eFLOAT32, 4); 
	d.torqueRatios16To19AH = omniWriter.registerAttribute(d.CH, "torqueRatios16To19", OmniPvdDataType::eFLOAT32, 4); 
	d.aveWheelSpeedRatios0To3AH = omniWriter.registerAttribute(d.CH, "aveWheelSpeedRatios0To3", OmniPvdDataType::eFLOAT32, 4); 
	d.aveWheelSpeedRatios4To7AH = omniWriter.registerAttribute(d.CH, "aveWheelSpeedRatios4To7", OmniPvdDataType::eFLOAT32, 4); 
	d.aveWheelSpeedRatios8To11AH = omniWriter.registerAttribute(d.CH, "aveWheelSpeedRatios8To11", OmniPvdDataType::eFLOAT32, 4); 
	d.aveWheelSpeedRatios12To15AH = omniWriter.registerAttribute(d.CH, "aveWheelSpeedRatios12To15", OmniPvdDataType::eFLOAT32, 4); 
	d.aveWheelSpeedRatios16To19AH = omniWriter.registerAttribute(d.CH, "aveWheelSpeedRatios16To19", OmniPvdDataType::eFLOAT32, 4); 
	return d;
}

void writeDiffState
(const PxVehicleDifferentialState& diffState, 
 const OmniPvdObjectHandle oh, const DiffState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
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

ClutchSlipState registerClutchSlipState(OmniPvdWriter& omniWriter)
{
	ClutchSlipState c;
	c.CH = omniWriter.registerClass("ClutchSlipState");
	c.slipAH = omniWriter.registerAttribute(c.CH, "clutchSlip", OmniPvdDataType::eFLOAT32, 1);
	return c;
}

void writeClutchSlipState
(const PxVehicleClutchSlipState& clutchSlipState, 
 const OmniPvdObjectHandle oh, const ClutchSlipState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.slipAH, clutchSlipState.clutchSlip);
}

EngineDrivetrain registerEngineDrivetrain(OmniPvdWriter& omniWriter)
{
	EngineDrivetrain e;
	e.CH = omniWriter.registerClass("EngineDrivetrain");
	e.commandStateAH = omniWriter.registerAttribute(e.CH, "commandState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	e.transmissionCommandStateAH = omniWriter.registerAttribute(e.CH, "transmissionCommandState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	e.clutchResponseParamsAH = omniWriter.registerAttribute(e.CH, "clutchResponseParams", OmniPvdDataType::eOBJECT_HANDLE, 1);
	e.clutchParamsAH = omniWriter.registerAttribute(e.CH, "clutchParams", OmniPvdDataType::eOBJECT_HANDLE, 1);
	e.engineParamsAH = omniWriter.registerAttribute(e.CH, "engineParams", OmniPvdDataType::eOBJECT_HANDLE, 1);
	e.gearboxParamsAH = omniWriter.registerAttribute(e.CH, "gearboxParams", OmniPvdDataType::eOBJECT_HANDLE, 1);
	e.autoboxParamsAH = omniWriter.registerAttribute(e.CH, "autoboxParams", OmniPvdDataType::eOBJECT_HANDLE, 1);
	e.differentialParamsAH = omniWriter.registerAttribute(e.CH, "differentialParams", OmniPvdDataType::eOBJECT_HANDLE, 1);
	e.clutchResponseStateAH= omniWriter.registerAttribute(e.CH, "clutchResponseState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	e.throttleResponseStateAH= omniWriter.registerAttribute(e.CH, "throttleResponseState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	e.engineStateAH = omniWriter.registerAttribute(e.CH, "engineState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	e.gearboxStateAH = omniWriter.registerAttribute(e.CH, "gearboxState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	e.autoboxStateAH = omniWriter.registerAttribute(e.CH, "autoboxState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	e.diffStateAH = omniWriter.registerAttribute(e.CH, "diffState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	e.clutchSlipStateAH = omniWriter.registerAttribute(e.CH, "clutchSlipState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	return e;
}


////////////////////////////
//PHYSX WHEEL ATTACHMENT
////////////////////////////

PhysXSuspensionLimitConstraintParams registerSuspLimitConstraintParams(OmniPvdWriter& omniWriter)
{
	struct DirSpecifier
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle suspensionAH;
		OmniPvdAttributeHandle geomNormalAH;
		OmniPvdAttributeHandle noneAH;
	};
	DirSpecifier s;
	s.CH = omniWriter.registerClass("DirectionSpecifier");
	s.suspensionAH = omniWriter.registerEnumValue(s.CH, "suspensionDir", 
		PxVehiclePhysXSuspensionLimitConstraintParams::DirectionSpecifier::eSUSPENSION);
	s.geomNormalAH = omniWriter.registerEnumValue(s.CH, "geomNormalDir", 
		PxVehiclePhysXSuspensionLimitConstraintParams::DirectionSpecifier::eROAD_GEOMETRY_NORMAL);
	s.noneAH = omniWriter.registerEnumValue(s.CH, "geomNormalDir", 
		PxVehiclePhysXSuspensionLimitConstraintParams::DirectionSpecifier::eNONE);

	PhysXSuspensionLimitConstraintParams c;
	c.CH = omniWriter.registerClass("PhysXSuspLimitConstraintParams");
	c.restitutionAH = omniWriter.registerAttribute(c.CH, "restitution", OmniPvdDataType::eFLOAT32, 1);
	c.directionForSuspensionLimitConstraintAH = omniWriter.registerFlagsAttribute(c.CH, "directionMode", s.CH);
	return c;
}

void writePhysXSuspLimitConstraintParams
(const PxVehiclePhysXSuspensionLimitConstraintParams& params,
 const OmniPvdObjectHandle oh, const PhysXSuspensionLimitConstraintParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFlagAttribute(omniWriter, ch, oh, ah.directionForSuspensionLimitConstraintAH, params.directionForSuspensionLimitConstraint);
	writeFloatAttribute(omniWriter, ch, oh, ah.restitutionAH, params.restitution);
}


PhysXWheelShape registerPhysXWheelShape(OmniPvdWriter& omniWriter)
{
	PhysXWheelShape w;
	w.CH = omniWriter.registerClass("PhysXWheelShape");
	w.shapePtrAH = omniWriter.registerAttribute(w.CH, "pxShapePtr", OmniPvdDataType::eOBJECT_HANDLE, 1);
	return w;
}

void writePhysXWheelShape
(const PxShape* wheelShape, 
 const OmniPvdObjectHandle oh, const PhysXWheelShape& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writePtrAttribute(omniWriter, ch, oh, ah.shapePtrAH, wheelShape);
}

PhysXRoadGeomState registerPhysXRoadGeomState(OmniPvdWriter& omniWriter)
{
	PhysXRoadGeomState g;
	g.CH = omniWriter.registerClass("PhysXRoadGeomState");
	g.hitPositionAH = omniWriter.registerAttribute(g.CH, "hitPosition", OmniPvdDataType::eFLOAT32, 3);
	g.hitActorPtrAH = omniWriter.registerAttribute(g.CH, "PxActor", OmniPvdDataType::eOBJECT_HANDLE, 1);
	g.hitShapePtrAH = omniWriter.registerAttribute(g.CH, "PxShape", OmniPvdDataType::eOBJECT_HANDLE, 1);
	g.hitMaterialPtrAH = omniWriter.registerAttribute(g.CH, "PxMaterial", OmniPvdDataType::eOBJECT_HANDLE, 1);
	return g;
}

void writePhysXRoadGeomState
(const PxVehiclePhysXRoadGeometryQueryState& roadGeomState,
const OmniPvdObjectHandle oh, const PhysXRoadGeomState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeVec3Attribute(omniWriter, ch, oh, ah.hitPositionAH, roadGeomState.hitPosition);
	writePtrAttribute(omniWriter, ch, oh, ah.hitActorPtrAH, roadGeomState.actor);
	writePtrAttribute(omniWriter, ch, oh, ah.hitMaterialPtrAH, roadGeomState.material);
	writePtrAttribute(omniWriter, ch, oh, ah.hitShapePtrAH, roadGeomState.shape);
}

PhysXConstraintState registerPhysXConstraintState(OmniPvdWriter& omniWriter)
{
	struct BoolAsEnum
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle falseAH;
		OmniPvdAttributeHandle trueAH;
	};
	BoolAsEnum boolAsEnum;
	boolAsEnum.CH = omniWriter.registerClass("PhysXConstraintStateBool");
	boolAsEnum.falseAH = omniWriter.registerEnumValue(boolAsEnum.CH, "False", 0);
	boolAsEnum.trueAH = omniWriter.registerEnumValue(boolAsEnum.CH, "True", 1);

	PhysXConstraintState c;
	c.CH = omniWriter.registerClass("PhysXConstraintState");
	c.tireLongActiveStatusAH = omniWriter.registerFlagsAttribute(c.CH, "tireLongitudinalActiveStatus", boolAsEnum.CH);
	c.tireLongLinearAH = omniWriter.registerAttribute(c.CH, "tireLongitudinalLinear", OmniPvdDataType::eFLOAT32, 3);
	c.tireLongAngularAH = omniWriter.registerAttribute(c.CH, "tireLongitudinalAngular", OmniPvdDataType::eFLOAT32, 3);
	c.tireLongDampingAH = omniWriter.registerAttribute(c.CH, "tireLongitudinalDamping", OmniPvdDataType::eFLOAT32, 1);
	c.tireLatActiveStatusAH = omniWriter.registerFlagsAttribute(c.CH, "tireLateralActiveStatus", boolAsEnum.CH);
	c.tireLatLinearAH = omniWriter.registerAttribute(c.CH, "tireLateralLinear", OmniPvdDataType::eFLOAT32, 3);
	c.tireLatAngularAH = omniWriter.registerAttribute(c.CH, "tireLateralAngular", OmniPvdDataType::eFLOAT32, 3);
	c.tireLatDampingAH = omniWriter.registerAttribute(c.CH, "tireLateralDamping", OmniPvdDataType::eFLOAT32, 1);
	c.suspActiveStatusAH = omniWriter.registerFlagsAttribute(c.CH, "suspActiveStatus", boolAsEnum.CH);
	c.suspLinearAH = omniWriter.registerAttribute(c.CH, "suspLinear", OmniPvdDataType::eFLOAT32, 3);
	c.suspAngularAH = omniWriter.registerAttribute(c.CH, "suspAngular", OmniPvdDataType::eFLOAT32, 3);
	c.suspRestitutionAH = omniWriter.registerAttribute(c.CH, "suspRestitution", OmniPvdDataType::eFLOAT32, 1);
	c.suspGeometricErrorAH = omniWriter.registerAttribute(c.CH, "suspGeometricError", OmniPvdDataType::eFLOAT32, 1);
	return c;
}

void writePhysXConstraintState
(const PxVehiclePhysXConstraintState& roadGeomState,
 const OmniPvdObjectHandle oh, const PhysXConstraintState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
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

PhysXMaterialFriction registerPhysXMaterialFriction(OmniPvdWriter& omniWriter)
{
	PhysXMaterialFriction f;
	f.CH = omniWriter.registerClass("PhysXMaterialFriction");
	f.frictionAH = omniWriter.registerAttribute(f.CH, "friction", OmniPvdDataType::eFLOAT32, 1);
	f.materialPtrAH = omniWriter.registerAttribute(f.CH, "material", OmniPvdDataType::eOBJECT_HANDLE, 1);
	return f;
}

void writePhysXMaterialFriction
(const PxVehiclePhysXMaterialFriction& materialFriction,
 const OmniPvdObjectHandle oh, const PhysXMaterialFriction& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.frictionAH, materialFriction.friction);
	writePtrAttribute(omniWriter, ch, oh, ah.materialPtrAH, materialFriction.material);
}

PhysXWheelAttachment registerPhysXWheelAttachment(OmniPvdWriter& omniWriter)
{
	PhysXWheelAttachment w;
	w.CH = omniWriter.registerClass("PhysXWheelAttachment");
	w.physxConstraintParamsAH = omniWriter.registerAttribute(w.CH, "physxConstraintParams", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.physxConstraintStateAH = omniWriter.registerAttribute(w.CH, "physxConstraintState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.physxWeelShapeAH = omniWriter.registerAttribute(w.CH, "physxWheelShape", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.physxRoadGeometryStateAH = omniWriter.registerAttribute(w.CH, "physxRoadGeomState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	w.physxMaterialFrictionSetAH = omniWriter.registerUniqueListAttribute(w.CH, "physXMaterialFrictions", OmniPvdDataType::eOBJECT_HANDLE);
	return w;
}

//////////////////////////
//PHYSX RIGID ACTOR
//////////////////////////

PhysXRigidActor registerPhysXRigidActor(OmniPvdWriter& omniWriter)
{
	PhysXRigidActor a;
	a.CH = omniWriter.registerClass("PhysXRigidActor");
	a.rigidActorAH = omniWriter.registerAttribute(a.CH, "rigidActor", OmniPvdDataType::eOBJECT_HANDLE, 1);
	return a;
}

void writePhysXRigidActor
(const PxRigidActor* actor, 
 const OmniPvdObjectHandle oh, const PhysXRigidActor& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writePtrAttribute(omniWriter, ch, oh, ah.rigidActorAH, actor);
}

PhysXRoadGeometryQueryParams registerPhysXRoadGeometryQueryParams(OmniPvdWriter& omniWriter)
{
	struct QueryType
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle raycastAH;
		OmniPvdAttributeHandle sweepAH;
		OmniPvdAttributeHandle noneAH;
	};
	QueryType queryType;
	queryType.CH = omniWriter.registerClass("PhysXRoadGeometryQueryTpe");
	queryType.raycastAH = omniWriter.registerEnumValue(queryType.CH, "raycast", PxVehiclePhysXRoadGeometryQueryType::eRAYCAST);
	queryType.sweepAH = omniWriter.registerEnumValue(queryType.CH, "sweep", PxVehiclePhysXRoadGeometryQueryType::eSWEEP);
	queryType.noneAH = omniWriter.registerEnumValue(queryType.CH, "none", PxVehiclePhysXRoadGeometryQueryType::eNONE);

	PhysXRoadGeometryQueryParams a;
	a.CH = omniWriter.registerClass("PhysXRoadGeomQueryParams");
	a.queryTypeAH = omniWriter.registerFlagsAttribute(a.CH, "physxQueryType", queryType.CH);

	struct QueryFlag
	{
		OmniPvdClassHandle CH;
		OmniPvdAttributeHandle staticAH;
		OmniPvdAttributeHandle dynamicAH;
		OmniPvdAttributeHandle preFilterAH;
		OmniPvdAttributeHandle postFilterAH;
		OmniPvdAttributeHandle anyHitAH;
		OmniPvdAttributeHandle noBlockAH;
		OmniPvdAttributeHandle batchQueryLegacyBehaviourAH;
		OmniPvdAttributeHandle disableHardcodedFilterAH;
	};
	QueryFlag queryFlag;
	queryFlag.CH = omniWriter.registerClass("PhysXRoadGeometryQueryFlag");
	queryFlag.staticAH = omniWriter.registerEnumValue(queryFlag.CH, "eSTATIC", PxQueryFlag::eSTATIC);
	queryFlag.dynamicAH = omniWriter.registerEnumValue(queryFlag.CH, "eDYNAMIC", PxQueryFlag::eDYNAMIC);
	queryFlag.preFilterAH = omniWriter.registerEnumValue(queryFlag.CH, "ePREFILTER", PxQueryFlag::ePREFILTER);
	queryFlag.postFilterAH = omniWriter.registerEnumValue(queryFlag.CH, "ePOSTFILTER", PxQueryFlag::ePOSTFILTER);
	queryFlag.anyHitAH = omniWriter.registerEnumValue(queryFlag.CH, "eANY_HIT", PxQueryFlag::eANY_HIT);
	queryFlag.noBlockAH = omniWriter.registerEnumValue(queryFlag.CH, "eNO_BLOCK", PxQueryFlag::eNO_BLOCK);
	queryFlag.batchQueryLegacyBehaviourAH = omniWriter.registerEnumValue(queryFlag.CH, "eBATCH_QUERY_LEGACY_BEHAVIOUR", PxQueryFlag::eBATCH_QUERY_LEGACY_BEHAVIOUR);
	queryFlag.disableHardcodedFilterAH = omniWriter.registerEnumValue(queryFlag.CH, "eDISABLE_HARDCODED_FILTER", PxQueryFlag::eDISABLE_HARDCODED_FILTER);

	a.filterDataParams.CH = omniWriter.registerClass("PhysXRoadGeometryQueryFilterData");
	a.filterDataParams.word0AH = omniWriter.registerAttribute(a.filterDataParams.CH, "word0", OmniPvdDataType::eUINT32, 1);
	a.filterDataParams.word1AH = omniWriter.registerAttribute(a.filterDataParams.CH, "word1", OmniPvdDataType::eUINT32, 1);
	a.filterDataParams.word2AH = omniWriter.registerAttribute(a.filterDataParams.CH, "word2", OmniPvdDataType::eUINT32, 1);
	a.filterDataParams.word3AH = omniWriter.registerAttribute(a.filterDataParams.CH, "word3", OmniPvdDataType::eUINT32, 1);
	a.filterDataParams.flagsAH = omniWriter.registerFlagsAttribute(a.filterDataParams.CH, "flags", queryFlag.CH);

	a.defaultFilterDataAH = omniWriter.registerAttribute(a.CH, "defaultFilterData", OmniPvdDataType::eOBJECT_HANDLE, 1);
	a.filterDataSetAH = omniWriter.registerUniqueListAttribute(a.CH, "filterDataSet", OmniPvdDataType::eOBJECT_HANDLE);

	return a;
}

void writePhysXRoadGeometryQueryFilterData
(const PxQueryFilterData& queryFilterData,
 const OmniPvdObjectHandle oh, const PhysXRoadGeometryQueryFilterData& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeUInt32Attribute(omniWriter, ch, oh, ah.word0AH, queryFilterData.data.word0);
	writeUInt32Attribute(omniWriter, ch, oh, ah.word1AH, queryFilterData.data.word1);
	writeUInt32Attribute(omniWriter, ch, oh, ah.word2AH, queryFilterData.data.word2);
	writeUInt32Attribute(omniWriter, ch, oh, ah.word3AH, queryFilterData.data.word3);
	writeFlagAttribute(omniWriter, ch, oh, ah.flagsAH, queryFilterData.flags);
}

void writePhysXRoadGeometryQueryParams
(const PxVehiclePhysXRoadGeometryQueryParams& queryParams, const PxVehicleAxleDescription& axleDesc,
 const OmniPvdObjectHandle queryParamsOH, const OmniPvdObjectHandle defaultFilterDataOH, const OmniPvdObjectHandle* filterDataOHs,
 const PhysXRoadGeometryQueryParams& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFlagAttribute(omniWriter, ch, queryParamsOH, ah.queryTypeAH, queryParams.roadGeometryQueryType);

	if (defaultFilterDataOH)  // TODO: test against invalid hanndle once it gets introduced
	{
		writePhysXRoadGeometryQueryFilterData(queryParams.defaultFilterData, defaultFilterDataOH, ah.filterDataParams,
			omniWriter, ch);
	}

	if (queryParams.filterDataEntries)
	{
		for (PxU32 i = 0; i < axleDesc.nbWheels; i++)
		{
			const PxU32 wheelId = axleDesc.wheelIdsInAxleOrder[i];

			const OmniPvdObjectHandle fdOH = filterDataOHs[wheelId];
			if (fdOH)  // TODO: test against invalid hanndle once it gets introduced
			{
				writePhysXRoadGeometryQueryFilterData(queryParams.filterDataEntries[wheelId], fdOH, ah.filterDataParams,
					omniWriter, ch);
			}
		}
	}
}

PhysXSteerState registerPhysXSteerState(OmniPvdWriter& omniWriter)
{
	PhysXSteerState s;
	s.CH = omniWriter.registerClass("PhysXSteerState");
	s.previousSteerCommandAH = omniWriter.registerAttribute(s.CH, "previousSteerCommand", OmniPvdDataType::eFLOAT32, 1);
	return s;
}

void writePhysXSteerState
(const PxVehiclePhysXSteerState& steerState, 
 const OmniPvdObjectHandle oh, const PhysXSteerState& ah, OmniPvdWriter& omniWriter, OmniPvdContextHandle ch)
{
	writeFloatAttribute(omniWriter, ch, oh, ah.previousSteerCommandAH, steerState.previousSteerCommand);
}


//////////////////////
//VEHICLE
//////////////////////

Vehicle registerVehicle(OmniPvdWriter& omniWriter)
{
	Vehicle v;
	v.CH = omniWriter.registerClass("Vehicle");

	v.rigidBodyParamsAH = omniWriter.registerAttribute(v.CH, "rigidBodyParams", OmniPvdDataType::eOBJECT_HANDLE, 1);
	v.rigidBodyStateAH = omniWriter.registerAttribute(v.CH, "rigidBodyState", OmniPvdDataType::eOBJECT_HANDLE, 1);

	v.suspStateCalcParamsAH = omniWriter.registerAttribute(v.CH, "suspStateCalcParams", OmniPvdDataType::eOBJECT_HANDLE, 1);

	v.wheelAttachmentSetAH = omniWriter.registerUniqueListAttribute(v.CH, "wheelAttachmentSet", OmniPvdDataType::eOBJECT_HANDLE);

	v.antiRollSetAH = omniWriter.registerUniqueListAttribute(v.CH, "antiRollSet", OmniPvdDataType::eOBJECT_HANDLE);
	v.antiRollForceAH = omniWriter.registerAttribute(v.CH, "antiRollForce", OmniPvdDataType::eOBJECT_HANDLE, 1);

	v.brakeResponseParamsSetAH = omniWriter.registerUniqueListAttribute(v.CH, "brakeResponseParamsSet", OmniPvdDataType::eOBJECT_HANDLE);
	v.steerResponseParamsAH = omniWriter.registerAttribute(v.CH, "steerResponseParams", OmniPvdDataType::eOBJECT_HANDLE, 1);
	v.brakeResponseStatesAH = omniWriter.registerAttribute(v.CH, "brakeResponseState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	v.steerResponseStatesAH = omniWriter.registerAttribute(v.CH, "steerResponseState", OmniPvdDataType::eOBJECT_HANDLE, 1);
	v.ackermannParamsAH = omniWriter.registerAttribute(v.CH, "ackermannParams", OmniPvdDataType::eOBJECT_HANDLE, 1);

	v.directDrivetrainAH = omniWriter.registerAttribute(v.CH, "directDrivetrain", OmniPvdDataType::eOBJECT_HANDLE, 1);
	v.engineDriveTrainAH = omniWriter.registerAttribute(v.CH, "engineDrivetrain", OmniPvdDataType::eOBJECT_HANDLE, 1);

	v.physxWheelAttachmentSetAH = omniWriter.registerUniqueListAttribute(v.CH, "physxWheelAttachmentSet", OmniPvdDataType::eOBJECT_HANDLE);

	v.physxRoadGeometryQueryParamsAH =  omniWriter.registerAttribute(v.CH, "physxRoadGeomQryParams", OmniPvdDataType::eOBJECT_HANDLE, 1);
	v.physxRigidActorAH = omniWriter.registerAttribute(v.CH, "physxRigidActor", OmniPvdDataType::eOBJECT_HANDLE, 1);
	v.physxSteerStateAH = omniWriter.registerAttribute(v.CH, "physxSteerState", OmniPvdDataType::eOBJECT_HANDLE, 1);

	return v;
}

#endif //PX_SUPPORT_OMNI_PVD

} // namespace vehicle2
} // namespace physx

/** @} */
