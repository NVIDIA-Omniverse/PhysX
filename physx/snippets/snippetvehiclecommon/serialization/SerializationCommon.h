// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "vehicle/PxVehicleAPI.h"

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
#if (PX_LINUX && PX_CLANG) || PX_SWITCH ||  PX_OSX
#pragma clang diagnostic pop
#endif

namespace snippetvehicle
{

using namespace physx;

bool openDocument(const char* directory, const char* filename, rapidjson::Document&);

bool readCommandResponseParams
(const rapidjson::Value& value, const PxVehicleAxleDescription& axleDesc,
 PxVehicleCommandResponseParams& responseParams);
bool writeCommandResponseParams
(const PxVehicleCommandResponseParams& responseParams, const PxVehicleAxleDescription& axleDesc,
 rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);

bool readVec3(const rapidjson::Value& values, PxVec3& r);
bool writeVec3(const PxVec3& r, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);

bool readQuat(const rapidjson::Value& values, PxQuat& r);
bool writeQuat(const PxQuat& r, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);

bool readTransform(const rapidjson::Value& values, PxTransform& r);
bool writeTransform(const PxTransform& r, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);

bool readFloatLookupTable(const rapidjson::Value& values, PxVehicleFixedSizeLookupTable<PxReal, 3>& lookupTable);
bool readFloatLookupTable(const rapidjson::Value& values, PxVehicleFixedSizeLookupTable<PxReal, PxVehicleEngineParams::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES>& lookupTable);
bool readVec3LookupTable(const rapidjson::Value& values, PxVehicleFixedSizeLookupTable<PxVec3, 3>& lookupTable);
bool writeVec3LookupTable(const PxVehicleFixedSizeLookupTable<PxVec3, 3>& lookupTable, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
bool writeFloatLookupTable(const PxVehicleFixedSizeLookupTable<PxReal, 3>& lookupTable, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
bool writeFloatLookupTable(const PxVehicleFixedSizeLookupTable<PxReal, PxVehicleEngineParams::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES>& lookupTable, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
}//namespace snippetvehicle
