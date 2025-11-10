// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include <private/omni/physx/IPhysxTests.h>

namespace omni
{
namespace physx
{
PhysicsStats getPhysicsStats();
float getMassInformation(const char* path, carb::Float3& inertia, carb::Float3& com);
void getMaterialsPaths(const pxr::SdfPath& path, std::vector<pxr::SdfPath>& materials);
void startLoggerCheck(const char* message, bool expectedResult, bool partialStringMatch);
void startLoggerCheckForMultiple(std::vector<std::string>& messages,
                                 bool expectedResult,
                                 bool expectAll,
                                 bool partialStringMatch);
bool endLoggerCheck();
bool isCudaLibPresent();

} // namespace physx
} // namespace omni
