// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

namespace omni
{
namespace physics
{
namespace tensors
{
class ISimulationBackend;
class ISimulationView;

struct BackendRegistry
{
    CARB_PLUGIN_INTERFACE("omni::physics::tensors::BackendRegistry", 0, 1);

    bool(CARB_ABI* registerBackend)(const char* name, ISimulationBackend* backend);
    void(CARB_ABI* unregisterBackend)(const char* name);
};

struct TensorApi
{
    CARB_PLUGIN_INTERFACE("omni::physics::tensors::TensorApi", 0, 1);

    ISimulationView*(CARB_ABI* createSimulationView)(long stageId);
    void(CARB_ABI* reset)();
};

} // namespace tensors
} // namespace physics
} // namespace omni
