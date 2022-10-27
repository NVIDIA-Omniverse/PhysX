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

#include "EngineDrivetrainSerialization.h"
#include "SerializationCommon.h"

#include <fstream>
#include <sstream>

namespace snippetvehicle2
{

bool readAutoboxParams(const rapidjson::Document& config, PxVehicleAutoboxParams& autoboxParams)
{
	if(!config.HasMember("AutoboxParams"))
		return false;

	if(!config["AutoboxParams"].HasMember("UpRatios"))
		return false;
	if (!config["AutoboxParams"].HasMember("DownRatios"))
		return false;
	if (!config["AutoboxParams"].HasMember("Latency"))
		return false;

	const rapidjson::Value& upRatios = config["AutoboxParams"]["UpRatios"];
	for(PxU32 i = 0; i < upRatios.Size(); i++)
	{
		autoboxParams.upRatios[i] = static_cast<PxReal>(upRatios[i].GetDouble());
	}

	const rapidjson::Value& downRatios = config["AutoboxParams"]["DownRatios"];
	for (PxU32 i = 0; i < downRatios.Size(); i++)
	{
		autoboxParams.downRatios[i] = static_cast<PxReal>(downRatios[i].GetDouble());
	}

	autoboxParams.latency = static_cast<PxReal>(config["AutoboxParams"]["Latency"].GetDouble());

	return true;
}

bool writeAutoboxParams(const PxVehicleAutoboxParams& autoboxParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("AutoboxParams");
	writer.StartObject();

	writer.Key("UpRatios");
	writer.StartArray();
	for(PxU32 i = 0; i < PxVehicleGearboxParams::eMAX_NB_GEARS; i++)
	{
		writer.Double(static_cast<double>(autoboxParams.upRatios[i]));
	}
	writer.EndArray();

	writer.Key("DownRatios");
	writer.StartArray();
	for (PxU32 i = 0; i < PxVehicleGearboxParams::eMAX_NB_GEARS; i++)
	{
		writer.Double(static_cast<double>(autoboxParams.downRatios[i]));
	}
	writer.EndArray();

	writer.Key("Latency");
	writer.Double(static_cast<double>(autoboxParams.latency));

	writer.EndObject();
	return true;
}


bool readClutchCommandResponseParams(const rapidjson::Document& config, PxVehicleClutchCommandResponseParams& clutchCommandResponseParams)
{
	if(!config.HasMember("ClutchCommandResponseParams"))
		return false;

	if(!config["ClutchCommandResponseParams"].HasMember("MaxResponse"))
		return false;

	clutchCommandResponseParams.maxResponse = static_cast<PxReal>(config["ClutchCommandResponseParams"]["MaxResponse"].GetDouble());

	return true;
}

bool writeClutchCommandResponseParams(const PxVehicleClutchCommandResponseParams& clutchCommandResponseParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("ClutchCommandResponseParams");
	writer.StartObject();

	writer.Key("MaxResponse");
	writer.Double(static_cast<double>(clutchCommandResponseParams.maxResponse));

	writer.EndObject();
	return true;
}

bool readEngineParams(const rapidjson::Document& config, PxVehicleEngineParams& engineParams)
{
	if(!config.HasMember("EngineParams"))
		return false;

	if(!config["EngineParams"].HasMember("TorqueCurve"))
		return false;
	if (!config["EngineParams"].HasMember("MOI"))
		return false;
	if (!config["EngineParams"].HasMember("PeakTorque"))
		return false;
	if (!config["EngineParams"].HasMember("IdleOmega"))
		return false;
	if (!config["EngineParams"].HasMember("MaxOmega"))
		return false;
	if (!config["EngineParams"].HasMember("DampingRateFullThrottle"))
		return false;
	if (!config["EngineParams"].HasMember("DampingRateZeroThrottleClutchEngaged"))
		return false;
	if (!config["EngineParams"].HasMember("DampingRateZeroThrottleClutchDisengaged"))
		return false;

	if(!readFloatLookupTable(config["EngineParams"]["TorqueCurve"], engineParams.torqueCurve))
		return false;

	engineParams.moi = static_cast<PxReal>(config["EngineParams"]["MOI"].GetDouble());
	engineParams.peakTorque = static_cast<PxReal>(config["EngineParams"]["PeakTorque"].GetDouble());
	engineParams.idleOmega = static_cast<PxReal>(config["EngineParams"]["IdleOmega"].GetDouble());
	engineParams.maxOmega = static_cast<PxReal>(config["EngineParams"]["MaxOmega"].GetDouble());
	engineParams.dampingRateFullThrottle = static_cast<PxReal>(config["EngineParams"]["DampingRateFullThrottle"].GetDouble());
	engineParams.dampingRateZeroThrottleClutchEngaged = static_cast<PxReal>(config["EngineParams"]["DampingRateZeroThrottleClutchEngaged"].GetDouble());
	engineParams.dampingRateZeroThrottleClutchDisengaged = static_cast<PxReal>(config["EngineParams"]["DampingRateZeroThrottleClutchDisengaged"].GetDouble());
	
	return true;
}

bool writeEngineParams(const PxVehicleEngineParams& engineParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("EngineParams");
	writer.StartObject();

	writer.Key("TorqueCurve");
	writeFloatLookupTable(engineParams.torqueCurve, writer);

	writer.Key("MOI");
	writer.Double(static_cast<double>(engineParams.moi));

	writer.Key("PeakTorque");
	writer.Double(static_cast<double>(engineParams.peakTorque));

	writer.Key("IdleOmega");
	writer.Double(static_cast<double>(engineParams.idleOmega));

	writer.Key("MaxOmega");
	writer.Double(static_cast<double>(engineParams.maxOmega));

	writer.Key("DampingRateFullThrottle");
	writer.Double(static_cast<double>(engineParams.dampingRateFullThrottle));

	writer.Key("DampingRateZeroThrottleClutchEngaged");
	writer.Double(double(engineParams.dampingRateZeroThrottleClutchEngaged));

	writer.Key("DampingRateZeroThrottleClutchDisengaged");
	writer.Double(double(engineParams.dampingRateZeroThrottleClutchDisengaged));

	writer.EndObject();
	return true;
}


bool readGearboxParams(const rapidjson::Document& config, PxVehicleGearboxParams& gearboxParams)
{
	if (!config.HasMember("GearboxParams"))
		return false;

	if (!config["GearboxParams"].HasMember("NeutralGear"))
		return false;
	if (!config["GearboxParams"].HasMember("Ratios"))
		return false;
	if (!config["GearboxParams"].HasMember("FinalRatio"))
		return false;
	if (!config["GearboxParams"].HasMember("SwitchTime"))
		return false;

	gearboxParams.neutralGear = config["GearboxParams"]["NeutralGear"].GetInt();
	gearboxParams.finalRatio = static_cast<PxReal>(config["GearboxParams"]["FinalRatio"].GetDouble());
	gearboxParams.switchTime = static_cast<PxReal>(config["GearboxParams"]["SwitchTime"].GetDouble());

	const rapidjson::Value& ratios = config["GearboxParams"]["Ratios"];
	for(PxU32 i = 0; i < ratios.Size(); i++)
	{
		gearboxParams.ratios[i] = static_cast<PxReal>(ratios[i].GetDouble());
	}
	gearboxParams.nbRatios = ratios.Size();

	return true;
}

bool writeGearboxParams(const PxVehicleGearboxParams& gearboxParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("GearboxParams");
	writer.StartObject();

	writer.Key("NeutralGear");
	writer.Int(gearboxParams.neutralGear);

	writer.Key("FinalRatio");
	writer.Double(static_cast<double>(gearboxParams.finalRatio));

	writer.Key("SwitchTime");
	writer.Double(static_cast<double>(gearboxParams.switchTime));

	writer.Key("Ratios");
	writer.StartArray();
	for(PxU32 i = 0; i < gearboxParams.nbRatios; i++)
	{
		writer.Double(static_cast<double>(gearboxParams.ratios[i]));
	}
	writer.EndArray();

	writer.EndObject();
	return true;
}

bool readMultiWheelDiffParams(const rapidjson::Value& config, PxVehicleMultiWheelDriveDifferentialParams& multiWheelDiffParams)
{
	if (!config.HasMember("TorqueRatios"))
		return false;
	if (!config.HasMember("AveWheelSpeedRatios"))
		return false;

	const rapidjson::Value& aveWheelSpeedRatios = config["AveWheelSpeedRatios"];
	const rapidjson::Value& torqueRatios = config["TorqueRatios"];
	if (aveWheelSpeedRatios.Size() != torqueRatios.Size())
		return false;

	for (PxU32 i = 0; i < aveWheelSpeedRatios.Size(); i++)
	{
		multiWheelDiffParams.aveWheelSpeedRatios[i] = static_cast<PxReal>(aveWheelSpeedRatios[i].GetDouble());
		multiWheelDiffParams.torqueRatios[i] = static_cast<PxReal>(torqueRatios[i].GetDouble());
	}

	return true;
}

void writeMultiWheelDiffParams
(const PxVehicleMultiWheelDriveDifferentialParams& multiWheelDiffParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("TorqueRatios");
	writer.StartArray();
	for (PxU32 i = 0; i < PxVehicleLimits::eMAX_NB_WHEELS; i++)
	{
		writer.Double(static_cast<double>(multiWheelDiffParams.torqueRatios[i]));
	}
	writer.EndArray();

	writer.Key("AveWheelSpeedRatios");
	writer.StartArray();
	for (PxU32 i = 0; i < PxVehicleLimits::eMAX_NB_WHEELS; i++)
	{
		writer.Double(static_cast<double>(multiWheelDiffParams.aveWheelSpeedRatios[i]));
	}
	writer.EndArray();
}

bool readMultiWheelDifferentialParams(const rapidjson::Document& config, PxVehicleMultiWheelDriveDifferentialParams& multiWheelDifferentialParams)
{
	multiWheelDifferentialParams.setToDefault();

	if (!config.HasMember("MultiWheelDifferentialParams"))
		return false;

	if (!readMultiWheelDiffParams(config["MultiWheelDifferentialParams"], multiWheelDifferentialParams))
		return false;

	return true;
}

bool writeMultiWheelDifferentialParams
(const PxVehicleMultiWheelDriveDifferentialParams& multiWheelDifferentialParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("MultiWheelDifferentialParams");
	writer.StartObject();

	writeMultiWheelDiffParams(multiWheelDifferentialParams, writer);

	writer.EndObject();
	return true;
}

bool readFourWheelDifferentialParams(const rapidjson::Document& config, PxVehicleFourWheelDriveDifferentialParams& fourWheelDifferentialParams)
{
	fourWheelDifferentialParams.setToDefault();

	if (!config.HasMember("FourWheelDifferentialParams"))
		return false;

	if (!readMultiWheelDiffParams(config["FourWheelDifferentialParams"], static_cast<PxVehicleMultiWheelDriveDifferentialParams&>(fourWheelDifferentialParams)))
		return false;

	if (!config["FourWheelDifferentialParams"].HasMember("FrontWheelIds"))
		return false;
	if (!config["FourWheelDifferentialParams"].HasMember("RearWheelIds"))
		return false;
	if (!config["FourWheelDifferentialParams"].HasMember("CenterBias"))
		return false;
	if (!config["FourWheelDifferentialParams"].HasMember("CenterTarget"))
		return false;
	if (!config["FourWheelDifferentialParams"].HasMember("FrontBias"))
		return false;
	if (!config["FourWheelDifferentialParams"].HasMember("FrontTarget"))
		return false;
	if (!config["FourWheelDifferentialParams"].HasMember("RearBias"))
		return false;
	if (!config["FourWheelDifferentialParams"].HasMember("RearTarget"))
		return false;
	if (!config["FourWheelDifferentialParams"].HasMember("Rate"))
		return false;

	const rapidjson::Value& frontWheelIds = config["FourWheelDifferentialParams"]["FrontWheelIds"];
	if(frontWheelIds.Size() != 2)
		return false;
	fourWheelDifferentialParams.frontWheelIds[0] = frontWheelIds[0].GetInt();
	fourWheelDifferentialParams.frontWheelIds[1] = frontWheelIds[1].GetInt();

	const rapidjson::Value& rearWheelIds = config["FourWheelDifferentialParams"]["RearWheelIds"];
	if (rearWheelIds.Size() != 2)
		return false;
	fourWheelDifferentialParams.rearWheelIds[0] = rearWheelIds[0].GetInt();
	fourWheelDifferentialParams.rearWheelIds[1] = rearWheelIds[1].GetInt();

	fourWheelDifferentialParams.centerBias = static_cast<PxReal>(config["FourWheelDifferentialParams"]["CenterBias"].GetDouble());
	fourWheelDifferentialParams.centerTarget = static_cast<PxReal>(config["FourWheelDifferentialParams"]["CenterTarget"].GetDouble());

	fourWheelDifferentialParams.frontBias = static_cast<PxReal>(config["FourWheelDifferentialParams"]["FrontBias"].GetDouble());
	fourWheelDifferentialParams.frontTarget = static_cast<PxReal>(config["FourWheelDifferentialParams"]["FrontTarget"].GetDouble());

	fourWheelDifferentialParams.rearBias = static_cast<PxReal>(config["FourWheelDifferentialParams"]["RearBias"].GetDouble());
	fourWheelDifferentialParams.rearTarget = static_cast<PxReal>(config["FourWheelDifferentialParams"]["RearTarget"].GetDouble());

	fourWheelDifferentialParams.rate = static_cast<PxReal>(config["FourWheelDifferentialParams"]["Rate"].GetDouble());

	return true;
}

bool writeFourWheelDifferentialParams(const PxVehicleFourWheelDriveDifferentialParams& fourWheelDifferentialParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("FourWheelDifferentialParams");
	writer.StartObject();

	writeMultiWheelDiffParams(static_cast<const PxVehicleMultiWheelDriveDifferentialParams&>(fourWheelDifferentialParams), writer);

	writer.Key("FrontWheelIds");
	writer.StartArray();
	writer.Int(fourWheelDifferentialParams.frontWheelIds[0]);
	writer.Int(fourWheelDifferentialParams.frontWheelIds[1]);
	writer.EndArray();

	writer.Key("RearWheelIds");
	writer.StartArray();
	writer.Int(fourWheelDifferentialParams.rearWheelIds[0]);
	writer.Int(fourWheelDifferentialParams.rearWheelIds[1]);
	writer.EndArray();

	writer.Key("CenterBias");
	writer.Double(static_cast<double>(fourWheelDifferentialParams.centerBias));

	writer.Key("CenterTarget");
	writer.Double(static_cast<double>(fourWheelDifferentialParams.centerTarget));

	writer.Key("FrontBias");
	writer.Double(static_cast<double>(fourWheelDifferentialParams.frontBias));

	writer.Key("FrontTarget");
	writer.Double(static_cast<double>(fourWheelDifferentialParams.frontTarget));

	writer.Key("RearBias");
	writer.Double(static_cast<double>(fourWheelDifferentialParams.rearBias));

	writer.Key("RearTarget");
	writer.Double(static_cast<double>(fourWheelDifferentialParams.rearTarget));

	writer.Key("Rate");
	writer.Double(static_cast<double>(fourWheelDifferentialParams.rate));

	writer.EndObject();
	return true;
}

bool readTankDifferentialParams(const rapidjson::Document& config, PxVehicleTankDriveDifferentialParams& tankDifferentialParams)
{
	tankDifferentialParams.setToDefault();

	if (!config.HasMember("TankDifferentialParams"))
		return false;

	if(!readMultiWheelDiffParams(config["TankDifferentialParams"], static_cast<PxVehicleMultiWheelDriveDifferentialParams&>(tankDifferentialParams)))
		return false;

	if (!config["TankDifferentialParams"].HasMember("TankTracks"))
		return false;

	const rapidjson::Value& tankTracks = config["TankDifferentialParams"]["TankTracks"];
	for (PxU32 i = 0; i < tankTracks.Size(); i++)
	{
		const rapidjson::Value& tankTrack = tankTracks[i];

		PxU32 wheelIds[PxVehicleLimits::eMAX_NB_WHEELS];
		PxU32 nbWheels = 0;
		for (PxU32 j = 0; j < tankTrack.Size(); j++)
		{
			wheelIds[nbWheels] = static_cast<PxU32>(tankTrack[j].GetInt());
			nbWheels++;
		}

		tankDifferentialParams.addTankTrack(nbWheels, wheelIds, i);
	}

	return true;
}

bool writeTankDifferentialParams(const PxVehicleTankDriveDifferentialParams& tankDifferentialParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("TankDifferentialParams");
	writer.StartObject();

	writeMultiWheelDiffParams(static_cast<const PxVehicleMultiWheelDriveDifferentialParams&>(tankDifferentialParams), writer);

	writer.Key("TankTracks");
	writer.StartArray();
	for (PxU32 i = 0; i < tankDifferentialParams.nbTracks; i++)
	{
		writer.StartArray();
		for (PxU32 j = 0; j < tankDifferentialParams.getNbWheelsInTrack(i); j++)
		{
			writer.Int(static_cast<int>(tankDifferentialParams.getWheelInTrack(j, i)));
		}
		writer.EndArray();
	}
	writer.EndArray();

	writer.EndObject();
	return true;
}


bool reaClutchParams(const rapidjson::Document& config, PxVehicleClutchParams& clutchParams)
{
	if (!config.HasMember("ClutchParams"))
		return false;

	if (!config["ClutchParams"].HasMember("AccuracyMode"))
		return false;
	if (!config["ClutchParams"].HasMember("EstimateIterations"))
		return false;

	clutchParams.accuracyMode = static_cast<PxVehicleClutchAccuracyMode::Enum>(config["ClutchParams"]["AccuracyMode"].GetInt());
	clutchParams.estimateIterations = config["ClutchParams"]["EstimateIterations"].GetInt();

	return true;
}

bool writeClutchParams(const PxVehicleClutchParams& clutchParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.Key("ClutchParams");
	writer.StartObject();

	writer.Key("AccuracyMode");
	writer.Int(static_cast<PxU32>(clutchParams.accuracyMode));

	writer.Key("EstimateIterations");
	writer.Int(clutchParams.estimateIterations);

	writer.EndObject();
	return true;
}


bool readEngineDrivetrainParamsFromJsonFile(const char* directory, const char* filename,
	EngineDrivetrainParams& engineDrivetrainParams)
{
	rapidjson::Document config;
	if (!openDocument(directory, filename, config))
		return false;

	if(!readAutoboxParams(config, engineDrivetrainParams.autoboxParams))
		return false;
	if(!readClutchCommandResponseParams(config, engineDrivetrainParams.clutchCommandResponseParams))
		return false;
	if (!readEngineParams(config, engineDrivetrainParams.engineParams))
		return false;
	if (!readGearboxParams(config, engineDrivetrainParams.gearBoxParams))
		return false;
	if (!readMultiWheelDifferentialParams(config, engineDrivetrainParams.multiWheelDifferentialParams))
		return false;
	if (!readFourWheelDifferentialParams(config, engineDrivetrainParams.fourWheelDifferentialParams))
		return false;
	if (!readTankDifferentialParams(config, engineDrivetrainParams.tankDifferentialParams))
		return false;
	if (!reaClutchParams(config, engineDrivetrainParams.clutchParams))
		return false;

	return true;
}

bool writeEngineDrivetrainParamsToJsonFile(const char* directory, const char* filename,
	const EngineDrivetrainParams& engineDrivetrainParams)
{
	rapidjson::StringBuffer strbuf;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(strbuf);
	writer.StartObject();

	writeAutoboxParams(engineDrivetrainParams.autoboxParams, writer);
	writeClutchCommandResponseParams(engineDrivetrainParams.clutchCommandResponseParams, writer);
	writeEngineParams(engineDrivetrainParams.engineParams, writer);
	writeGearboxParams(engineDrivetrainParams.gearBoxParams, writer);
	writeMultiWheelDifferentialParams(engineDrivetrainParams.multiWheelDifferentialParams, writer);
	writeFourWheelDifferentialParams(engineDrivetrainParams.fourWheelDifferentialParams, writer);
	writeTankDifferentialParams(engineDrivetrainParams.tankDifferentialParams, writer);
	writeClutchParams(engineDrivetrainParams.clutchParams, writer);

	writer.EndObject();

	std::ofstream myfile;
	myfile.open(std::string(directory) + "/" + filename);
	myfile << strbuf.GetString() << std::endl;
	myfile.close();

	return true;
}

}//namespace snippetvehicle2
