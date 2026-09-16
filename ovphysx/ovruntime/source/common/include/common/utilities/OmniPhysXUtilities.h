// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-SIM-DEFAULT-001
 * @covers AC-1
 */

#pragma once

#include <omni/physx/IPhysxSettings.h>

#include <carb/settings/ISettings.h>

#include <cstring>

namespace omni
{
namespace physx
{

// pxr-free. The scene-ownership gate lives in usdLoad/LoadStage.cpp (canSceneBeProcessedByPhysXSource).
inline bool isPhysXDefaultSimulator()
{
    carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
    const char* defaultSimulator = settings ? settings->getStringBuffer(kSettingDefaultSimulator) : nullptr;
    return defaultSimulator && strcmp(defaultSimulator, "PhysX") == 0;
}

} // namespace physx
} // namespace omni
