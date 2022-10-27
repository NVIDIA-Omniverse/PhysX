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

#pragma once

#include "vehicle2/PxVehicleAPI.h"

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

namespace snippetvehicle2
{

using namespace physx;
using namespace physx::vehicle2;

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

}//namespace snippetvehicle2
