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

#include "../base/Base.h"

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

bool readAxleDescription(const rapidjson::Document& config, PxVehicleAxleDescription& axleDesc);
bool writeAxleDescription(const PxVehicleAxleDescription& axleDesc, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);


bool readFrame(const rapidjson::Document& config, PxVehicleFrame& frame);
bool writeFrame(const PxVehicleFrame& frame, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);


bool readScale(const rapidjson::Document& config, PxVehicleScale& scale);
bool writeScale(const PxVehicleScale& scale, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);


bool readBrakeResponseParams
	(const rapidjson::Document& config, const PxVehicleAxleDescription& axleDesc,
	 PxVehicleBrakeCommandResponseParams& brakeResponseParams);
bool writeBrakeResponseParams
	(const PxVehicleBrakeCommandResponseParams& brakeResponseParams, const PxVehicleAxleDescription& axleDesc,
	 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);


bool readHandbrakeResponseParams
	(const rapidjson::Document& config, const PxVehicleAxleDescription& axleDesc,
	 PxVehicleBrakeCommandResponseParams& handbrakeResponseParams);
bool writeHandbrakeResponseParams
	(const PxVehicleBrakeCommandResponseParams& handrakeResponseParams, const PxVehicleAxleDescription& axleDesc,
	 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);


bool readSteerResponseParams
	(const rapidjson::Document& config, const PxVehicleAxleDescription& axleDesc,
	 PxVehicleSteerCommandResponseParams& steerResponseParams);
bool writeSteerResponseParams
	(const PxVehicleSteerCommandResponseParams& steerResponseParams, const PxVehicleAxleDescription& axleDesc,
	 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);

bool readAckermannParams
	(const rapidjson::Document& config, PxVehicleAckermannParams& ackermannParams);
bool writeAckermannParams
	(const PxVehicleAckermannParams& ackermannParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);

bool readRigidBodyParams
	(const rapidjson::Document& config, PxVehicleRigidBodyParams& rigidBodyParams);
bool writeRigidBodyParams
	(const PxVehicleRigidBodyParams& rigidBodyParams, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);

bool readSuspensionParams
	(const rapidjson::Document& config, const PxVehicleAxleDescription& axleDesc, 
	 PxVehicleSuspensionParams* suspParams);
bool writeSuspensionParams
	(const PxVehicleSuspensionParams* suspParams, const PxVehicleAxleDescription& axleDesc,
	 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);

bool readSuspensionStateCalculationParams
	(const rapidjson::Document& config, 
	 PxVehicleSuspensionStateCalculationParams& suspParams);
bool writeSuspensionStateCalculationParams
	(const PxVehicleSuspensionStateCalculationParams& suspParams,
	 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);


bool readSuspensionComplianceParams
	(const rapidjson::Document& config, const PxVehicleAxleDescription& axleDesc, 
	 PxVehicleSuspensionComplianceParams* suspParams);
bool writeSuspensionComplianceParams
	(const PxVehicleSuspensionComplianceParams* suspParams, const PxVehicleAxleDescription& axleDesc,
	 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);


bool readSuspensionForceParams
	(const rapidjson::Document& config, const PxVehicleAxleDescription& axleDesc,
	 PxVehicleSuspensionForceParams* suspParams);
bool writeSuspensionForceParams
	(const PxVehicleSuspensionForceParams* suspParams, const PxVehicleAxleDescription& axleDesc,
	 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);


bool readSuspensionForceLegacyParams
	(const rapidjson::Document& config, const PxVehicleAxleDescription& axleDesc,
	 PxVehicleSuspensionForceLegacyParams* suspParams);
bool writeSuspensionForceLegacyParams
	(const PxVehicleSuspensionForceLegacyParams* suspParams, const PxVehicleAxleDescription& axleDesc,
	 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);


bool readTireSlipParams
	(const rapidjson::Document& config, const PxVehicleAxleDescription& axleDesc,
	 PxVehicleTireSlipParams* tireParams);
bool writeTireSlipParams
	(const PxVehicleTireSlipParams* tireParams, const PxVehicleAxleDescription& axleDesc,
	 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);


bool readTireStickyParams
	(const rapidjson::Document& config, const PxVehicleAxleDescription& axleDesc,
	 PxVehicleTireStickyParams* tireParams);
bool writeTireStickyParams
	(const PxVehicleTireStickyParams* tireParams, const PxVehicleAxleDescription& axleDesc,
	 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);


bool readTireForceParams
	(const rapidjson::Document& config, const PxVehicleAxleDescription& axleDesc,
	 PxVehicleTireForceParams* tireParams);
bool writeTireForceParams
	(const PxVehicleTireForceParams* tireParams, const PxVehicleAxleDescription& axleDesc,
	 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);


bool readWheelParams
	(const rapidjson::Document& config, const PxVehicleAxleDescription& axleDesc,
	 PxVehicleWheelParams* wheelParams);
bool writeWheelParams
	(const PxVehicleWheelParams* wheelParams, const PxVehicleAxleDescription& axleDesc,
	 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);


bool readBaseParamsFromJsonFile(const char* directory, const char* filename, BaseVehicleParams&);
bool writeBaseParamsToJsonFile(const char* directory, const char* filename, const BaseVehicleParams&);

}//namespace snippetvehicle2
