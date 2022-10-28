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

#include "SerializationCommon.h"

#include <fstream>
#include <sstream>

namespace snippetvehicle2
{

bool openDocument(const char* directory, const char* filename,
	rapidjson::Document& document)
{
	// Check the json file exists.
	std::string fileWithPath(directory);
	fileWithPath.push_back('/');
	fileWithPath.append(filename);
	std::ifstream inputFile(fileWithPath);
	if (!inputFile.is_open())
	{
		printf("Opening file \"%s\" failed.\n", fileWithPath.c_str());
		return false;
	}

	// Check the json file can be loaded by rapidjson.
	// Failures might be missing commas or braces.
	std::string inputData{ std::istreambuf_iterator<char>(inputFile), std::istreambuf_iterator<char>() };
	document.Parse(inputData.c_str());
	inputFile.close();
	if (!document.IsObject())
	{
		printf("Parsing file \"%s\" failed.\n", fileWithPath.c_str());
		return false;
	}

	return true;
}

bool readCommandResponseParams
(const rapidjson::Value& value, const PxVehicleAxleDescription& axleDesc,
	PxVehicleCommandResponseParams& responseParams)
{
	if (!value.HasMember("MaxResponse"))
		return false;
	responseParams.maxResponse = static_cast<PxReal>(value["MaxResponse"].GetDouble());

	if (!value.HasMember("WheelResponseMultipliers"))
		return false;
	const rapidjson::Value& responseMultipliers = value["WheelResponseMultipliers"];

	if (responseMultipliers.Size() < axleDesc.nbWheels)
		return false;
	if (responseMultipliers.Size() > PxVehicleLimits::eMAX_NB_WHEELS)
		return false;

	for (PxU32 i = 0; i < responseMultipliers.Size(); i++)
	{
		responseParams.wheelResponseMultipliers[i] = static_cast<PxReal>(responseMultipliers[i].GetDouble());
	}

	//Nonlinear response is not mandatory.
	responseParams.nonlinearResponse.clear();
	if(!value.HasMember("NonLinearResponse"))
		return true;
	const rapidjson::Value& nonlinearResponse = value["NonLinearResponse"];
	const int nbCommandValues = nonlinearResponse.Size();
	for (int i = 0; i < nbCommandValues; i++)
	{
		PxVehicleCommandValueResponseTable commandValueResponse;

		//Read the throttle value and set it.
		const rapidjson::Value& commandAndResponse = nonlinearResponse[i];
		if (!commandAndResponse.HasMember("Throttle"))
			return false;
		const PxReal commandValue = static_cast<PxReal>(commandAndResponse["Throttle"].GetDouble());
		commandValueResponse.commandValue = commandValue;

		//Read the array of (speed, torque) entries.
		if (!commandAndResponse.HasMember("ResponseCurve"))
			return false;
		const rapidjson::Value& torqueCurve = commandAndResponse["ResponseCurve"];
		const PxU32 nbItems = torqueCurve.Size();
		for (PxU32 j = 0; j < nbItems; j++)
		{
			const rapidjson::Value& torqueCurveItem = torqueCurve[j];
			if (torqueCurveItem.Size() != 2)
				return false;
			const PxReal speed = static_cast<PxReal>(torqueCurveItem[0].GetDouble());
			const PxReal normalisedResponse = static_cast<PxReal>(torqueCurveItem[1].GetDouble());
			commandValueResponse.speedResponses.addPair(speed, normalisedResponse);
		}

		responseParams.nonlinearResponse.addResponse(commandValueResponse);
	}

	return true;
}

bool writeCommandResponseParams
(const PxVehicleCommandResponseParams& responseParams, const PxVehicleAxleDescription& axleDesc,
 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("MaxResponse");
	writer.Double(static_cast<double>(responseParams.maxResponse));

	writer.Key("WheelResponseMultipliers");
	writer.StartArray();
	for(PxU32 i = 0; i < axleDesc.nbWheels; i++)
	{
		const PxU32 wheelId = axleDesc.wheelIdsInAxleOrder[i];
		writer.Double(static_cast<double>(responseParams.wheelResponseMultipliers[wheelId]));
	}
	writer.EndArray();

	writer.Key("ResponseCurve");
	writer.StartArray();
	for (PxU32 i = 0; i < responseParams.nonlinearResponse.nbCommandValues; i++)
	{
		writer.Key("Throttle");
		writer.Double(static_cast<double>(responseParams.nonlinearResponse.commandValues[i]));

		writer.Key("ResponseCurve");
		writer.StartArray();
		const PxReal* speeds = responseParams.nonlinearResponse.speedResponses + responseParams.nonlinearResponse.speedResponsesPerCommandValue[i];
		const PxReal* responses = speeds + responseParams.nonlinearResponse.nbSpeedRenponsesPerCommandValue[i];
		for (PxU32 j = 0; j < responseParams.nonlinearResponse.nbSpeedRenponsesPerCommandValue[i]; j++)
		{
			writer.StartArray();
			writer.Double(static_cast<double>(speeds[j]));
			writer.Double(static_cast<double>(responses[j]));
			writer.EndArray();
		}
		writer.EndArray();
	}
	writer.EndArray();

	return true;
}

void writeCommandResponseParams
(const PxVehicleAxleDescription& axleDesc, const PxVehicleCommandResponseParams& throttleResponseParams, 
 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();

	writer.Key("MaxResponse");
	writer.Double(static_cast<double>(throttleResponseParams.maxResponse));

	writer.Key("WheelResponseMultipliers");
	writer.StartArray();
	for(PxU32 i = 0; i < axleDesc.nbWheels; i++)
	{
		const PxU32 wheelId = axleDesc.wheelIdsInAxleOrder[i];
		writer.Double(static_cast<double>(throttleResponseParams.wheelResponseMultipliers[wheelId]));
	}
	writer.EndArray();

	writer.EndObject();
}


bool readVec3(const rapidjson::Value& values, PxVec3& r)
{
	if (values.Size() != 3)
		return false;

	r.x = static_cast<PxReal>(values[0].GetDouble());
	r.y = static_cast<PxReal>(values[1].GetDouble());
	r.z = static_cast<PxReal>(values[2].GetDouble());
	return true;
}

bool writeVec3(const PxVec3& r, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartArray();
	writer.Double(static_cast<double>(r.x));
	writer.Double(static_cast<double>(r.y));
	writer.Double(static_cast<double>(r.z));
	writer.EndArray();
	return true;
}


bool readQuat(const rapidjson::Value& values, PxQuat& r)
{
	if (values.Size() != 4)
		return false;

	r.x = static_cast<PxReal>(values[0].GetDouble());
	r.y = static_cast<PxReal>(values[1].GetDouble());
	r.z = static_cast<PxReal>(values[2].GetDouble());
	r.w = static_cast<PxReal>(values[3].GetDouble());

	return true;
}

bool writeQuat(const PxQuat& r, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartArray();
	writer.Double(static_cast<double>(r.x));
	writer.Double(static_cast<double>(r.y));
	writer.Double(static_cast<double>(r.z));
	writer.Double(static_cast<double>(r.w));
	writer.EndArray();
	return true;
}

bool readTransform(const rapidjson::Value& values, PxTransform& r)
{
	if (!values.HasMember("Pos"))
		return false;
	if (!values.HasMember("Quat"))
		return false;

	if (!readVec3(values["Pos"], r.p))
		return false;
	if (!readQuat(values["Quat"], r.q))
		return false;

	return true;
}

bool writeTransform(const PxTransform& r, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key("Pos");
	writeVec3(r.p, writer);
	writer.Key("Quat");
	writeQuat(r.q, writer);
	writer.EndObject();
	return true;
}

template<const PxU32 N> 
bool readFloatLookupTableN(const rapidjson::Value& values, PxVehicleFixedSizeLookupTable<PxReal, N>& lookupTable)
{
	lookupTable.clear();

	if (values.Size() > N)
		return false;

	for (PxU32 i = 0; i < values.Size(); i++)
	{
		const rapidjson::Value& element = values[i];
		if (element.Size() != 2)
			return false;

		const PxReal x = static_cast<PxReal>(values[i][0].GetDouble());
		const PxReal y = static_cast<PxReal>(values[i][1].GetDouble());
		lookupTable.addPair(x, y);
	}

	return true;
}

bool readFloatLookupTable
(const rapidjson::Value& values, PxVehicleFixedSizeLookupTable<PxReal, 3>& lookupTable)
{
	return readFloatLookupTableN<3>(values, lookupTable);
}

bool readFloatLookupTable
(const rapidjson::Value& values, PxVehicleFixedSizeLookupTable<PxReal, PxVehicleEngineParams::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES>& lookupTable)
{
	return readFloatLookupTableN<PxVehicleEngineParams::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES>(values, lookupTable);
}

template<const PxU32 N>
bool writeFloatLookupTable(const PxVehicleFixedSizeLookupTable<PxReal, N>& lookupTable, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartArray();
	for (PxU32 i = 0; i < lookupTable.nbDataPairs; i++)
	{
		writer.StartArray();
		writer.Double(static_cast<double>(lookupTable.xVals[i]));
		writer.Double(static_cast<double>(lookupTable.yVals[i]));
		writer.EndArray();
	}
	writer.EndArray();
	return true;
}

bool writeFloatLookupTable(const PxVehicleFixedSizeLookupTable<PxReal, 3>& lookupTable, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	return writeFloatLookupTable<3>(lookupTable, writer);
}

bool writeFloatLookupTable(const PxVehicleFixedSizeLookupTable<PxReal, PxVehicleEngineParams::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES>& lookupTable, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	return writeFloatLookupTable<PxVehicleEngineParams::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES>(lookupTable, writer);
}

bool readVec3LookupTable(const rapidjson::Value& values, PxVehicleFixedSizeLookupTable<PxVec3, 3>& lookupTable)
{
	lookupTable.clear();

	if (values.Size() > 3)
		return false;

	for (PxU32 i = 0; i < values.Size(); i++)
	{
		const rapidjson::Value& element = values[i];
		if (element.Size() != 4)
			return false;

		const PxReal x = static_cast<PxReal>(values[i][0].GetDouble());
		const PxVec3 y(static_cast<PxReal>(
			values[i][1].GetDouble()), static_cast<PxReal>(values[i][2].GetDouble()), static_cast<PxReal>(values[i][3].GetDouble()));
		lookupTable.addPair(x, y);
	}

	return true;
}

bool writeVec3LookupTable(const PxVehicleFixedSizeLookupTable<PxVec3, 3>& lookupTable, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartArray();
	for (PxU32 i = 0; i < lookupTable.nbDataPairs; i++)
	{
		writer.StartArray();
		writer.Double(static_cast<double>(lookupTable.xVals[i]));
		writer.Double(static_cast<double>(lookupTable.yVals[i].x));
		writer.Double(static_cast<double>(lookupTable.yVals[i].y));
		writer.Double(static_cast<double>(lookupTable.yVals[i].z));
		writer.EndArray();
	}
	writer.EndArray();
	return true;

}

}//namespace snippetvehicle2




