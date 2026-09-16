// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

// getMaterialsPaths/getSceneInternalActorCount take TestPathArg (a plain std::string, see
// IPhysxTests.h) and are ObjectKey-native via IPhysicsSource::findByPath/textFor.
#include <private/omni/physx/IPhysxTests.h>

namespace omni
{
namespace physx
{
PhysicsStats getPhysicsStats();
BatchedContactBufferStats getBatchedContactBufferStats();
float getMassInformation(const char* path, carb::Float3& inertia, carb::Float3& com);
void getMaterialsPaths(const TestPathArg& path, std::vector<TestPathArg>& materials);
void startLoggerCheck(const char* message, bool expectedResult, bool partialStringMatch);
void startLoggerCheckForMultiple(std::vector<std::string>& messages,
                                 bool expectedResult,
                                 bool expectAll,
                                 bool partialStringMatch);
bool endLoggerCheck();
bool isCudaLibPresent();
size_t getSceneInternalActorCount(const TestPathArg& scenePath);

} // namespace physx
} // namespace omni
