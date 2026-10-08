// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "vehicle/PxVehicleAPI.h"

#include "../enginedrivetrain/EngineDrivetrain.h"

#if PX_SWITCH
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wexpansion-to-defined"
#elif PX_OSX
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wexpansion-to-defined"
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wimplicit-fallthrough"
#elif PX_LINUX && PX_CLANG
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#endif
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#if (PX_LINUX && PX_CLANG) || PX_SWITCH
#pragma clang diagnostic pop
#endif

namespace snippetvehicle
{

using namespace physx;

bool readAutoboxParams(const rapidjson::Document& config, PxVehicleAutoboxParams& autoboxParams);
bool writeAutoboxParams(const PxVehicleAutoboxParams& autoboxParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);

bool readClutchCommandResponseParams(const rapidjson::Document& config, PxVehicleClutchCommandResponseParams& clutchCommandResponseParams);
bool writeClutchCommandResponseParams(const PxVehicleClutchCommandResponseParams& clutchCommandResponseParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);

bool readEngineParams(const rapidjson::Document& config, PxVehicleEngineParams& engineParams);
bool writeEngineParams(const PxVehicleEngineParams& engineParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);

bool readGearboxParams(const rapidjson::Document& config, PxVehicleGearboxParams& gearboxParams);
bool writeGearboxParams(const PxVehicleGearboxParams& gearboxParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);

bool readFourWheelDifferentialParams(const rapidjson::Document& config, PxVehicleFourWheelDriveDifferentialParams& fourWheelDifferentialParams);
bool writeFourWheelDifferentialParams(const PxVehicleFourWheelDriveDifferentialParams& fourWheelDifferentialParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);

bool readMultiWheelDifferentialParams(const rapidjson::Document& config, PxVehicleMultiWheelDriveDifferentialParams& multiWheelDifferentialParams);
bool writeMultiWheelDifferentialParams(const PxVehicleMultiWheelDriveDifferentialParams& multiWheelDifferentialParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);

bool readTankDifferentialParams(const rapidjson::Document& config, PxVehicleTankDriveDifferentialParams& tankDifferentialParams);
bool writeTankDifferentialParams(const PxVehicleTankDriveDifferentialParams& tankDifferentialParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);

bool reaClutchParams(const rapidjson::Document& config, PxVehicleClutchParams& clutchParams);
bool writeClutchParams(const PxVehicleClutchParams& clutchParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);

bool readEngineDrivetrainParamsFromJsonFile(const char* directory, const char* filename,
	EngineDrivetrainParams&);
bool writeEngineDrivetrainParamsToJsonFile(const char* directory, const char* filename,
	const EngineDrivetrainParams&);

}//namespace snippetvehicle
