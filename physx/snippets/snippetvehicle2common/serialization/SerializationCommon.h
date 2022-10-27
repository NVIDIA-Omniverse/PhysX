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

namespace snippetvehicle2
{

using namespace physx;
using namespace physx::vehicle2;

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
}//namespace snippetvehicle2
