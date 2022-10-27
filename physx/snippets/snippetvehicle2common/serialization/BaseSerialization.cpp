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

#include "BaseSerialization.h"
#include "SerializationCommon.h"

#include <fstream>
#include <sstream>

namespace snippetvehicle2
{

bool readAxleDescription(const rapidjson::Document& config, PxVehicleAxleDescription& axleDesc)
{
	if (!config.HasMember("AxleDescription"))
		return false;

	const PxU32 nbAxles = config["AxleDescription"].Size();
	if (nbAxles > PxVehicleLimits::eMAX_NB_AXLES)
		return false;

	axleDesc.setToDefault();

	//Read each axle in turn.
	for (PxU32 i = 0; i < nbAxles; i++)
	{
		const rapidjson::Value& axle = config["AxleDescription"][i];

		if (!axle.HasMember("WheelIds"))
			return false;
		const rapidjson::Value& wheelIds = axle["WheelIds"];

		const PxU32 nbWheelIds = wheelIds.Size();
		if(nbWheelIds > PxVehicleLimits::eMAX_NB_WHEELS)
			return false;
		PxU32 axleWheelIds[PxVehicleLimits::eMAX_NB_WHEELS];
		for (PxU32 j = 0; j < nbWheelIds; j++)
		{
			axleWheelIds[j]= wheelIds[j].GetInt();
		}
		axleDesc.addAxle(nbWheelIds, axleWheelIds);
	}

	return true;
}

bool writeAxleDescription(const PxVehicleAxleDescription& axleDesc, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("AxleDescription");

	writer.StartArray();
	for(PxU32 i = 0; i < axleDesc.getNbAxles();  i++)
	{
		writer.StartObject();
		writer.Key("WheelIds");
		writer.StartArray();
		for(PxU32  j = 0; j < axleDesc.getNbWheelsOnAxle(i); j++)
		{
			writer.Int(axleDesc.getWheelOnAxle(j, i));
		}
		writer.EndArray();
		writer.EndObject();
	}
	writer.EndArray();

	return true;
}

bool readFrame(const rapidjson::Document& config, PxVehicleFrame& frame)
{
	if (!config.HasMember("Frame"))
		return false;

	if (!config["Frame"].HasMember("LngAxis"))
		return false;
	if (!config["Frame"].HasMember("LatAxis"))
		return false;
	if (!config["Frame"].HasMember("VrtAxis"))
		return false;


	const PxU32 lngAxis = config["Frame"]["LngAxis"].GetInt();
	const PxU32 latAxis = config["Frame"]["LatAxis"].GetInt();
	const PxU32 vrtAxis = config["Frame"]["VrtAxis"].GetInt();
	if ((lngAxis == latAxis) || (lngAxis == vrtAxis) || (latAxis == vrtAxis))
	{
		return false;
	}

	frame.lngAxis = static_cast<PxVehicleAxes::Enum>(lngAxis);
	frame.latAxis = static_cast<PxVehicleAxes::Enum>(latAxis);
	frame.vrtAxis = static_cast<PxVehicleAxes::Enum>(vrtAxis);

	return true;
}

bool writeFrame(const PxVehicleFrame& frame, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("Frame");
	writer.StartObject();
	writer.Key("LngAxis");
	writer.Int(static_cast<PxU32>(frame.lngAxis));
	writer.Key("LatAxis");
	writer.Int(static_cast<PxU32>(frame.latAxis));
	writer.Key("VrtAxis");
	writer.Int(static_cast<PxU32>(frame.vrtAxis));
	writer.EndObject();
	return true;
}

bool readScale(const rapidjson::Document& config, PxVehicleScale& scale)
{
	if (!config.HasMember("Scale"))
		return false;

	if (!config["Scale"].HasMember("Scale"))
		return false;

	scale.scale = static_cast<PxReal>(config["Scale"]["Scale"].GetDouble());
	return true;
}

bool writeScale(const PxVehicleScale& scale, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("Scale");
	writer.StartObject();
	writer.Key("Scale");
	writer.Double(static_cast<double>(scale.scale));
	writer.EndObject();
	return true;
}

bool readBrakeResponseParams
(const rapidjson::Document& config, const PxVehicleAxleDescription& axleDesc,
	PxVehicleBrakeCommandResponseParams& brakeResponseParams)
{
	if (!config.HasMember("BrakeCommandResponseParams"))
		return false;

	if (!readCommandResponseParams(config["BrakeCommandResponseParams"], axleDesc, brakeResponseParams))
		return false;

	return true;
}

bool writeBrakeResponseParams
(const PxVehicleBrakeCommandResponseParams& brakeResponseParams, const PxVehicleAxleDescription& axleDesc,
 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("BrakeCommandResponseParams");
	writer.StartObject();
	writeCommandResponseParams(brakeResponseParams, axleDesc, writer);
	writer.EndObject();
	return true;
}

bool readHandbrakeResponseParams
(const rapidjson::Document& config, const PxVehicleAxleDescription& axleDesc,
	PxVehicleBrakeCommandResponseParams& handbrakeResponseParams)
{
	if (!config.HasMember("HandbrakeCommandResponseParams"))
		return false;

	if (!readCommandResponseParams(config["HandbrakeCommandResponseParams"], axleDesc, handbrakeResponseParams))
		return false;

	return true;
}

bool writeHandbrakeResponseParams
(const PxVehicleBrakeCommandResponseParams& handrakeResponseParams, const PxVehicleAxleDescription& axleDesc,
 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("HandbrakeCommandResponseParams");
	writer.StartObject();
	writeCommandResponseParams(handrakeResponseParams, axleDesc, writer);
	writer.EndObject();
	return true;
}

bool readSteerResponseParams
(const rapidjson::Document& config, const PxVehicleAxleDescription& axleDesc,
 PxVehicleSteerCommandResponseParams& steerResponseParams)
{
	if (!config.HasMember("SteerCommandResponseParams"))
		return false;

	if (!readCommandResponseParams(config["SteerCommandResponseParams"], axleDesc, steerResponseParams))
		return false;

	return true;
}

bool writeSteerResponseParams
(const PxVehicleSteerCommandResponseParams& steerResponseParams, const PxVehicleAxleDescription& axleDesc,
 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("SteerCommandResponseParams");
	writer.StartObject();
	writeCommandResponseParams(steerResponseParams, axleDesc, writer);
	writer.EndObject();
	return true;
}

bool readAckermannParams(const rapidjson::Document& config, PxVehicleAckermannParams& ackermannParams)
{
	if (!config.HasMember("AckermannParams"))
		return false;

	const rapidjson::Value& corrections = config["AckermannParams"];

	if (!corrections.HasMember("WheelBase"))
		return false;
	if (!corrections.HasMember("TrackWidth"))
		return false;
	if (!corrections.HasMember("Strength"))
		return false;

	ackermannParams.wheelBase = static_cast<PxReal>(corrections["WheelBase"].GetDouble());
	ackermannParams.trackWidth = static_cast<PxReal>(corrections["TrackWidth"].GetDouble());
	ackermannParams.strength = static_cast<PxReal>(corrections["Strength"].GetDouble());

	if (!corrections.HasMember("WheelIds"))
		return false;
	const rapidjson::Value& wheelIds = corrections["WheelIds"];
	const PxU32 nbWheelIds = wheelIds.Size();
	if (nbWheelIds != 2)
		return false;
	for (PxU32 j = 0; j < 2; j++)
	{
		ackermannParams.wheelIds[j] = wheelIds[j].GetInt();
	}

	return true;
}

bool writeAckermannParams(const PxVehicleAckermannParams& ackermannParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("AckermannParams");
	writer.StartObject();
	writer.Key("WheelIds");
	writer.StartArray();
	writer.Int(ackermannParams.wheelIds[0]);
	writer.Int(ackermannParams.wheelIds[1]);
	writer.EndArray();
	writer.Key("WheelBase");
	writer.Double(static_cast<double>(ackermannParams.wheelBase));
	writer.Key("TrackWidth");
	writer.Double(static_cast<double>(ackermannParams.trackWidth));
	writer.Key("Strength");
	writer.Double(static_cast<double>(ackermannParams.strength));
	writer.EndObject();
	return true;
}

bool readRigidBodyParams
(const rapidjson::Document& config, PxVehicleRigidBodyParams& rigidBodyParams)
{
	if (!config.HasMember("RigidBodyParams"))
		return false;

	if (!config["RigidBodyParams"].HasMember("Mass"))
		return false;
	if (!config["RigidBodyParams"].HasMember("MOI"))
		return false;

	rigidBodyParams.mass = static_cast<PxReal>(config["RigidBodyParams"]["Mass"].GetDouble());
	if (!readVec3(config["RigidBodyParams"]["MOI"], rigidBodyParams.moi))
		return false;

	return true;
}

bool writeRigidBodyParams
(const PxVehicleRigidBodyParams& rigidBodyParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("RigidBodyParams");
	writer.StartObject();
	writer.Key("Mass");
	writer.Double(static_cast<double>(rigidBodyParams.mass));
	writer.Key("MOI");
	writeVec3(rigidBodyParams.moi, writer);
	writer.EndObject();
	return true;
}

bool readSuspensionParams(const rapidjson::Document& config, const PxVehicleAxleDescription& axleDesc, PxVehicleSuspensionParams* suspParams)
{
	if (!config.HasMember("SuspensionParams"))
		return false;

	const rapidjson::Value& suspensions = config["SuspensionParams"];
	const PxU32 nbSuspensions = suspensions.Size();
	if (nbSuspensions != axleDesc.getNbWheels())
		return false;

	for (PxU32 i = 0; i < nbSuspensions; i++)
	{
		if(!suspensions[i].HasMember("WheelId"))
			return false;
		if (!suspensions[i].HasMember("SuspensionAttachment"))
			return false;
		if (!suspensions[i].HasMember("SuspensionTravelDir"))
			return false;
		if (!suspensions[i].HasMember("SuspensionTravelDist"))
			return false;
		if (!suspensions[i].HasMember("WheelAttachment"))
			return false;

		const PxU32 wheelId = suspensions[i]["WheelId"].GetInt();
		PxVehicleSuspensionParams& sp = suspParams[wheelId];

		if (!readTransform(suspensions[i]["SuspensionAttachment"], sp.suspensionAttachment))
			return false;
		if (!readVec3(suspensions[i]["SuspensionTravelDir"], sp.suspensionTravelDir))
			return false;
		sp.suspensionTravelDist = static_cast<PxReal>(suspensions[i]["SuspensionTravelDist"].GetDouble());
		if (!readTransform(suspensions[i]["WheelAttachment"], sp.wheelAttachment))
			return false;
	}

	return true;
}

bool writeSuspensionParams(const PxVehicleSuspensionParams* suspParams, const PxVehicleAxleDescription& axleDesc,
 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("SuspensionParams");
	writer.StartArray();

	for(PxU32 i = 0; i < axleDesc.nbWheels; i++)
	{
		writer.StartObject();

		writer.Key("WheelId");
		const PxU32 wheelId = axleDesc.wheelIdsInAxleOrder[i];
		writer.Int(wheelId);

		writer.Key("SuspensionAttachment");
		writeTransform(suspParams[wheelId].suspensionAttachment, writer);

		writer.Key("SuspensionTravelDir");
		writeVec3(suspParams[wheelId].suspensionTravelDir, writer);

		writer.Key("SuspensionTravelDist");
		writer.Double(static_cast<double>(suspParams[wheelId].suspensionTravelDist));

		writer.Key("WheelAttachment");
		writeTransform(suspParams[wheelId].wheelAttachment, writer);

		writer.EndObject();
	}

	writer.EndArray();
	return true;
}

static const char* kLimitSuspensionExpansionVelocity = "LimitSuspensionExpansionVelocity";

bool readSuspensionStateCalculationParams(const rapidjson::Document& config, PxVehicleSuspensionStateCalculationParams& suspParams)
{
	if (!config.HasMember("SuspensionStateCalculationParams"))
		return false;
	const rapidjson::Value& suspCalcParams = config["SuspensionStateCalculationParams"];

	if (!suspCalcParams.HasMember("JounceCalculationType"))
		return false;

	if (!suspCalcParams.HasMember(kLimitSuspensionExpansionVelocity))
		return false;

	suspParams.suspensionJounceCalculationType = static_cast<PxVehicleSuspensionJounceCalculationType::Enum>(suspCalcParams["JounceCalculationType"].GetInt());
	suspParams.limitSuspensionExpansionVelocity = suspCalcParams[kLimitSuspensionExpansionVelocity].GetBool();
	return true;
}

bool writeSuspensionStateCalculationParams
(const PxVehicleSuspensionStateCalculationParams& suspParams, 
 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("SuspensionStateCalculationParams");
	writer.StartObject();
	writer.Key("JounceCalculationType");
	writer.Int(static_cast<PxU32>(suspParams.suspensionJounceCalculationType));
	writer.Key(kLimitSuspensionExpansionVelocity);
	writer.Bool(suspParams.limitSuspensionExpansionVelocity);
	writer.EndObject();
	return true;
}

bool readSuspensionComplianceParams(const rapidjson::Document& config, const PxVehicleAxleDescription& axleDesc, PxVehicleSuspensionComplianceParams* suspCompParams)
{
	if (!config.HasMember("SuspensionComplianceParams"))
		return false;

	const rapidjson::Value& suspensions = config["SuspensionComplianceParams"];
	const PxU32 nbSuspensions = suspensions.Size();
	if (nbSuspensions != axleDesc.getNbWheels())
		return false;

	for (PxU32 i = 0; i < nbSuspensions; i++)
	{
		if (!suspensions[i].HasMember("WheelId"))
			return false;
		if (!suspensions[i].HasMember("WheelToeAngle"))
			return false;
		if (!suspensions[i].HasMember("WheelCamberAngle"))
			return false;
		if (!suspensions[i].HasMember("SuspForceAppPoint"))
			return false;
		if (!suspensions[i].HasMember("SuspForceAppPoint"))
			return false;

		const PxU32 wheelId = suspensions[i]["WheelId"].GetInt();

		if (!readFloatLookupTable(suspensions[i]["WheelToeAngle"], suspCompParams[wheelId].wheelToeAngle))
			return false;
		if (!readFloatLookupTable(suspensions[i]["WheelCamberAngle"], suspCompParams[wheelId].wheelCamberAngle))
			return false;
		if (!readVec3LookupTable(suspensions[i]["SuspForceAppPoint"], suspCompParams[wheelId].suspForceAppPoint))
			return false;
		if (!readVec3LookupTable(suspensions[i]["TireForceAppPoint"], suspCompParams[wheelId].tireForceAppPoint))
			return false;
	}

	return true;
}

bool writeSuspensionComplianceParams
(const PxVehicleSuspensionComplianceParams* suspParams, const PxVehicleAxleDescription& axleDesc,
 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("SuspensionComplianceParams");
	writer.StartArray();

	for (PxU32 i = 0; i < axleDesc.nbWheels; i++)
	{
		writer.StartObject();

		writer.Key("WheelId");
		const PxU32 wheelId = axleDesc.wheelIdsInAxleOrder[i];
		writer.Int(wheelId);

		writer.Key("WheelToeAngle");
		writeFloatLookupTable(suspParams[wheelId].wheelToeAngle, writer);

		writer.Key("WheelCamberAngle");
		writeFloatLookupTable(suspParams[wheelId].wheelCamberAngle, writer);

		writer.Key("SuspForceAppPoint");
		writeVec3LookupTable(suspParams[wheelId].suspForceAppPoint, writer);

		writer.Key("TireForceAppPoint");
		writeVec3LookupTable(suspParams[wheelId].tireForceAppPoint, writer);

		writer.EndObject();
	}

	writer.EndArray();
	return true;
}

bool readSuspensionForceParams(const rapidjson::Document& config, const PxVehicleAxleDescription& axleDesc,
	PxVehicleSuspensionForceParams* suspParams)
{
	if (!config.HasMember("SuspensionForceParams"))
		return false;

	const rapidjson::Value& suspensions = config["SuspensionForceParams"];
	const PxU32 nbSuspensions = suspensions.Size();
	if (nbSuspensions != axleDesc.getNbWheels())
		return false;

	for (PxU32 i = 0; i < nbSuspensions; i++)
	{
		if (!suspensions[i].HasMember("WheelId"))
			return false;
		if (!suspensions[i].HasMember("Damping"))
			return false;
		if (!suspensions[i].HasMember("Stiffness"))
			return false;
		if (!suspensions[i].HasMember("SprungMass"))
			return false;

		const PxU32 wheelId = suspensions[i]["WheelId"].GetInt();
		suspParams[wheelId].damping = static_cast<PxReal>(suspensions[i]["Damping"].GetDouble());
		suspParams[wheelId].stiffness = static_cast<PxReal>(suspensions[i]["Stiffness"].GetDouble());
		suspParams[wheelId].sprungMass = static_cast<PxReal>(suspensions[i]["SprungMass"].GetDouble());
	}

	return true;
}

bool writeSuspensionForceParams
(const PxVehicleSuspensionForceParams* suspParams, const PxVehicleAxleDescription& axleDesc,
 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("SuspensionForceParams");
	writer.StartArray();

	for (PxU32 i = 0; i < axleDesc.nbWheels; i++)
	{
		writer.StartObject();

		writer.Key("WheelId");
		const PxU32 wheelId = axleDesc.wheelIdsInAxleOrder[i];
		writer.Int(wheelId);

		writer.Key("Damping");
		writer.Double(double(suspParams[wheelId].damping));

		writer.Key("Stiffness");
		writer.Double(double(suspParams[wheelId].stiffness));

		writer.Key("SprungMass");
		writer.Double(double(suspParams[wheelId].sprungMass));

		writer.EndObject();
	}

	writer.EndArray();
	return true;
}

bool readTireForceParams
(const rapidjson::Document& config, const PxVehicleAxleDescription& axleDesc,
	PxVehicleTireForceParams* tireParams)
{
	if (!config.HasMember("TireForceParams"))
		return false;

	const rapidjson::Value& tires = config["TireForceParams"];
	const PxU32 nbTires = tires.Size();
	if (nbTires != axleDesc.getNbWheels())
		return false;

	for (PxU32 i = 0; i < nbTires; i++)
	{
		if (!tires[i].HasMember("WheelId"))
			return false;
		if (!tires[i].HasMember("LongitudinalStiffness"))
			return false;
		if (!tires[i].HasMember("LateralStiffnessX"))
			return false;
		if (!tires[i].HasMember("LateralStiffnessY"))
			return false;
		if (!tires[i].HasMember("CamberStiffness"))
			return false;
		if (!tires[i].HasMember("RestLoad"))
			return false;
		if (!tires[i].HasMember("FrictionVsSlip"))
			return false;
		if (!tires[i].HasMember("TireLoadFilter"))
			return false;

		const PxU32 wheelId = tires[i]["WheelId"].GetInt();
		tireParams[wheelId].longStiff = static_cast<PxReal>(tires[i]["LongitudinalStiffness"].GetDouble());
		tireParams[wheelId].latStiffX = static_cast<PxReal>(tires[i]["LateralStiffnessX"].GetDouble());
		tireParams[wheelId].latStiffY = static_cast<PxReal>(tires[i]["LateralStiffnessY"].GetDouble());
		tireParams[wheelId].camberStiff = static_cast<PxReal>(tires[i]["CamberStiffness"].GetDouble());
		tireParams[wheelId].restLoad = static_cast<PxReal>(tires[i]["RestLoad"].GetDouble());

		const rapidjson::Value& frictionVsSlip = tires[i]["FrictionVsSlip"];
		if (frictionVsSlip.Size() != 3)
			return false;
		for (PxU32 j = 0; j < 3; j++)
		{
			const rapidjson::Value& element = tires[i]["FrictionVsSlip"][j];
			if (element.Size() != 2)
				return false;

			tireParams[wheelId].frictionVsSlip[j][0] = static_cast<PxReal>(element[0].GetDouble());
			tireParams[wheelId].frictionVsSlip[j][1] = static_cast<PxReal>(element[1].GetDouble());
		}

		const rapidjson::Value& tireLoadFilter = tires[i]["TireLoadFilter"];
		if (tireLoadFilter.Size() != 2)
			return false;
		for (PxU32 j = 0; j < 2; j++)
		{
			const rapidjson::Value& element = tires[i]["TireLoadFilter"][j];
			if (element.Size() != 2)
				return false;

			tireParams[wheelId].loadFilter[j][0] = static_cast<PxReal>(element[0].GetDouble());
			tireParams[wheelId].loadFilter[j][1] = static_cast<PxReal>(element[1].GetDouble());
		}
	}

	return true;
}

bool writeTireForceParams
(const PxVehicleTireForceParams* tireParams, const PxVehicleAxleDescription& axleDesc,
 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("TireForceParams");
	writer.StartArray();

	for (PxU32 i = 0; i < axleDesc.nbWheels; i++)
	{
		writer.StartObject();

		writer.Key("WheelId");
		const PxU32 wheelId = axleDesc.wheelIdsInAxleOrder[i];
		writer.Int(wheelId);

		writer.Key("LongitudinalStiffness");
		writer.Double(static_cast<double>(tireParams[wheelId].longStiff));

		writer.Key("LateralStiffnessX");
		writer.Double(static_cast<double>(tireParams[wheelId].latStiffX));

		writer.Key("LateralStiffnessY");
		writer.Double(static_cast<double>(tireParams[wheelId].latStiffY));

		writer.Key("CamberStiffness");
		writer.Double(static_cast<double>(tireParams[wheelId].camberStiff));

		writer.Key("RestLoad");
		writer.Double(static_cast<double>(tireParams[wheelId].restLoad));

		writer.Key("FrictionVsSlip");
		writer.StartArray();
		for(PxU32 k = 0; k < 3; k++)
		{
			writer.StartArray();
			writer.Double(static_cast<double>(tireParams[wheelId].frictionVsSlip[k][0]));
			writer.Double(static_cast<double>(tireParams[wheelId].frictionVsSlip[k][1]));
			writer.EndArray();
		}
		writer.EndArray();

		writer.Key("TireLoadFilter");
		writer.StartArray();
		for (PxU32 k = 0; k < 2; k++)
		{
			writer.StartArray();
			writer.Double(static_cast<double>(tireParams[wheelId].loadFilter[k][0]));
			writer.Double(static_cast<double>(tireParams[wheelId].loadFilter[k][1]));
			writer.EndArray();
		}
		writer.EndArray();

		writer.EndObject();
	}

	writer.EndArray();
	return true;
}

bool readWheelParams
(const rapidjson::Document& config, const PxVehicleAxleDescription& axleDesc,
 PxVehicleWheelParams* wheelParams)
{
	if (!config.HasMember("WheelParams"))
		return false;

	const rapidjson::Value& wheels = config["WheelParams"];
	const PxU32 nbWheels = wheels.Size();
	if (nbWheels != axleDesc.getNbWheels())
		return false;

	for (PxU32 i = 0; i < nbWheels; i++)
	{
		if (!wheels[i].HasMember("WheelId"))
			return false;
		if (!wheels[i].HasMember("HalfWidth"))
			return false;
		if (!wheels[i].HasMember("Radius"))
			return false;
		if (!wheels[i].HasMember("Mass"))
			return false;
		if (!wheels[i].HasMember("MOI"))
			return false;
		if (!wheels[i].HasMember("DampingRate"))
			return false;

		const PxU32 wheelId = wheels[i]["WheelId"].GetInt();
		wheelParams[wheelId].halfWidth = static_cast<PxReal>(wheels[i]["HalfWidth"].GetDouble());
		wheelParams[wheelId].radius = static_cast<PxReal>(wheels[i]["Radius"].GetDouble());
		wheelParams[wheelId].mass = static_cast<PxReal>(wheels[i]["Mass"].GetDouble());
		wheelParams[wheelId].moi = static_cast<PxReal>(wheels[i]["MOI"].GetDouble());
		wheelParams[wheelId].dampingRate = static_cast<PxReal>(wheels[i]["DampingRate"].GetDouble());
	}

	return true;
}

bool writeWheelParams
(const PxVehicleWheelParams* wheelParams, const PxVehicleAxleDescription& axleDesc,
 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("WheelParams");
	writer.StartArray();

	for (PxU32 i = 0; i < axleDesc.nbWheels; i++)
	{
		writer.StartObject();

		writer.Key("WheelId");
		const PxU32 wheelId = axleDesc.wheelIdsInAxleOrder[i];
		writer.Int(wheelId);

		writer.Key("HalfWidth");
		writer.Double(static_cast<double>(wheelParams[wheelId].halfWidth));

		writer.Key("Radius");
		writer.Double(static_cast<double>(wheelParams[wheelId].radius));

		writer.Key("Mass");
		writer.Double(static_cast<double>(wheelParams[wheelId].mass));

		writer.Key("MOI");
		writer.Double(static_cast<double>(wheelParams[wheelId].moi));
			
		writer.Key("DampingRate");
		writer.Double(static_cast<double>(wheelParams[wheelId].dampingRate));

		writer.EndObject();
	}

	writer.EndArray();
	return true;
}


bool readBaseParamsFromJsonFile(const char* directory, const char* filename, 
	BaseVehicleParams& baseParams)
{
	rapidjson::Document config;
	if (!openDocument(directory, filename, config))
		return false;

	//////////////////////////////
	//Read the high level params
	//////////////////////////////

	PxMemSet(&baseParams.axleDescription, 0xff, sizeof(baseParams.axleDescription));
	if (!readAxleDescription(config, baseParams.axleDescription))
		return false;

	PxMemSet(&baseParams.frame, 0xff, sizeof(baseParams.frame));
	if (!readFrame(config, baseParams.frame))
		return false;

	PxMemSet(&baseParams.scale, 0xff, sizeof(baseParams.scale));
	if (!readScale(config, baseParams.scale))
		return false;

	//////////////////////////////
	//Read the rigid body params
	/////////////////////////////

	PxMemSet(&baseParams.rigidBodyParams, 0xff, sizeof(baseParams.rigidBodyParams));
	if (!readRigidBodyParams(config, baseParams.rigidBodyParams))
		return false;

	//////////////////////////////
	//Read the suspension state calculation params.
	//////////////////////////////

	PxMemSet(&baseParams.suspensionStateCalculationParams, 0xff, sizeof(baseParams.suspensionStateCalculationParams));
	if (!readSuspensionStateCalculationParams(config, baseParams.suspensionStateCalculationParams))
		return false;

	///////////////////////////////
	//Read the command responses
	///////////////////////////////

	PxMemSet(&baseParams.brakeResponseParams[0], 0xff, sizeof(baseParams.brakeResponseParams[0]));
	if (!readBrakeResponseParams(config, baseParams.axleDescription, baseParams.brakeResponseParams[0]))
		return false;

	PxMemSet(&baseParams.brakeResponseParams[1], 0xff, sizeof(baseParams.brakeResponseParams[1]));
	if (!readHandbrakeResponseParams(config, baseParams.axleDescription, baseParams.brakeResponseParams[1]))
		return false;

	PxMemSet(&baseParams.steerResponseParams, 0xff, sizeof(baseParams.steerResponseParams));
	if (!readSteerResponseParams(config, baseParams.axleDescription, baseParams.steerResponseParams))
		return false;

	PxMemSet(&baseParams.ackermannParams, 0xff, sizeof(baseParams.ackermannParams));
	if (!readAckermannParams(config, baseParams.ackermannParams[0]))
		return false;

	///////////////////////////////////
	//Read the suspension params
	///////////////////////////////////

	PxMemSet(baseParams.suspensionParams, 0xff, sizeof(baseParams.suspensionParams));
	if (!readSuspensionParams(config, baseParams.axleDescription, baseParams.suspensionParams))
		return false;

	PxMemSet(baseParams.suspensionComplianceParams, 0x00, sizeof(baseParams.suspensionComplianceParams));
	if (!readSuspensionComplianceParams(config, baseParams.axleDescription, baseParams.suspensionComplianceParams))
		return false;

	PxMemSet(baseParams.suspensionForceParams, 0xff, sizeof(baseParams.suspensionForceParams));
	if (!readSuspensionForceParams(config, baseParams.axleDescription, baseParams.suspensionForceParams))
		return false;

	///////////////////////////////////
	//Read the tire params
	///////////////////////////////////

	PxMemSet(baseParams.tireForceParams, 0xff, sizeof(baseParams.tireForceParams));
	if (!readTireForceParams(config, baseParams.axleDescription, baseParams.tireForceParams))
		return false;

	//////////////////////////
	//Read the wheel params
	//////////////////////////

	PxMemSet(baseParams.wheelParams, 0xff, sizeof(baseParams.wheelParams));
	if (!readWheelParams(config, baseParams.axleDescription, baseParams.wheelParams))
		return false;

	return true;
}

bool writeBaseParamsToJsonFile(const char* directory, const char* filename, const BaseVehicleParams& baseParams)
{
	rapidjson::StringBuffer strbuf;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(strbuf);
	writer.StartObject();

	//////////////////////////////
	//Write the high level params
	//////////////////////////////

	writeAxleDescription(baseParams.axleDescription, writer);
	writeFrame(baseParams.frame, writer);
	writeScale(baseParams.scale, writer);

	//////////////////////////////
	//Write the rigid body params
	/////////////////////////////

	writeRigidBodyParams(baseParams.rigidBodyParams, writer);


	//////////////////////////////
	//Write the suspension state calculation params
	//////////////////////////////

	writeSuspensionStateCalculationParams(baseParams.suspensionStateCalculationParams, writer);

	///////////////////////////////
	//Write the command responses
	///////////////////////////////

	writeBrakeResponseParams(baseParams.brakeResponseParams[0], baseParams.axleDescription, writer);
	writeHandbrakeResponseParams(baseParams.brakeResponseParams[1], baseParams.axleDescription, writer);
	writeSteerResponseParams(baseParams.steerResponseParams, baseParams.axleDescription, writer);
	writeAckermannParams(baseParams.ackermannParams[0], writer);

	///////////////////////////////////
	//Write the suspension params
	///////////////////////////////////

	writeSuspensionParams(baseParams.suspensionParams, baseParams.axleDescription, writer);
	writeSuspensionComplianceParams(baseParams.suspensionComplianceParams, baseParams.axleDescription, writer);
	writeSuspensionForceParams(baseParams.suspensionForceParams, baseParams.axleDescription, writer);

	///////////////////////////////////
	//Write the tire params
	///////////////////////////////////

	writeTireForceParams(baseParams.tireForceParams, baseParams.axleDescription, writer);

	//////////////////////////
	//Write the wheel params
	//////////////////////////

	writeWheelParams(baseParams.wheelParams, baseParams.axleDescription, writer);

	writer.EndObject();

	std::ofstream myfile;
	myfile.open(std::string(directory) + "/" + filename);
	myfile << strbuf.GetString() << std::endl;
	myfile.close();

	return true;
}

}//namespace snippetvehicle2
