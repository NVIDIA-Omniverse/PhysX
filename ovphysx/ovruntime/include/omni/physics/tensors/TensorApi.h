// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

namespace omni
{
namespace physics
{
namespace tensors
{
class ISimulationView;

struct TensorApi
{
    // PhysX owns the tensor backend directly; there is no backend selector since PhysX is the only backend.

    ISimulationView*(CARB_ABI* createSimulationView)(long stageId);
    void(CARB_ABI* reset)();
    void(CARB_ABI* resetStage)(long stageId);
};

} // namespace tensors
} // namespace physics
} // namespace omni
