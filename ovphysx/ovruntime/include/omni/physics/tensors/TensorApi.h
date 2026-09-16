// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include <omni/physics/AttachHandle.h>

namespace omni
{
namespace physics
{
namespace tensors
{
class ISimulationView;

// The attach identity moved to omni::physics (ADR-0016 Decision 2) once it
// stopped being a tensors concept. Aliased back here so every existing
// omni::physics::tensors:: spelling keeps compiling; see
// omni/physics/AttachHandle.h for the contract.
using omni::physics::AttachHandle;
using omni::physics::kActiveAttach;
using omni::physics::kNoAttach;

struct TensorApi
{
    // PhysX owns the tensor backend directly; there is no backend selector since PhysX is the only backend.

    /// Creates a view over the objects of `attachHandle`. Returns null if the
    /// handle does not resolve -- which includes a handle whose attach has ended.
    ISimulationView*(CARB_ABI* createSimulationView)(AttachHandle attachHandle);
    void(CARB_ABI* reset)();
    /// Invalidates every view of `attachHandle` and drops its simulation data.
    /// Must be called before the attach is torn down: views borrow it.
    void(CARB_ABI* resetStage)(AttachHandle attachHandle);
};

} // namespace tensors
} // namespace physics
} // namespace omni
