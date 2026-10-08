// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "vehicle/PxVehicleAPI.h"

#include "../directdrivetrain/DirectDrivetrain.h"

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

bool readThrottleResponseParams
(const rapidjson::Document& config, const PxVehicleAxleDescription& axleDesc,
	PxVehicleDirectDriveThrottleCommandResponseParams& throttleResponseParams);

bool writeThrottleResponseParams
(const PxVehicleDirectDriveThrottleCommandResponseParams& throttleResponseParams, const PxVehicleAxleDescription& axleDesc,
	rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);


bool readDirectDrivetrainParamsFromJsonFile(const char* directory, const char* filename,
	const PxVehicleAxleDescription& axleDescription, DirectDrivetrainParams&);

bool writeDirectDrivetrainParamsToJsonFile(const char* directory, const char* filename,
	const PxVehicleAxleDescription& axleDescription, const DirectDrivetrainParams&);

}//namespace snippetvehicle
