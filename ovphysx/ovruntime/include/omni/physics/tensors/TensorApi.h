// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-VIEWS-001
// @covers AC-1 AC-2
// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-3 AC-4

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

/// Public views retain their owner identity. Independent owners may run on separate
/// client threads; calls on one owner require external serialization. Immutable
/// selection metadata remains readable while simulation is pending. Scene operations
/// reject conflicting same-owner work: bool methods return false and void methods
/// throw std::runtime_error before side effects. Callbacks must not suspend.
struct TensorApi
{
    // PhysX owns the tensor backend directly; there is no backend selector since PhysX is the only backend.

    /// Creates a view over the objects of `attachHandle`. Returns null if the
    /// handle does not resolve, is zero/active, or runtime work is busy. The
    /// caller must pass the exact live attachment returned by attach.
    ISimulationView*(CARB_ABI* createSimulationView)(AttachHandle attachHandle);
    /// Whole-scene articulation kinematic (FK) refresh. A scene-level DirectGPU op that needs no
    /// per-selection view, so callers do not construct one just to invoke it. Returns false on failure.
    bool(CARB_ABI* updateArticulationsKinematic)(AttachHandle attachHandle);
    /// Invalidates every view of `attachHandle` and drops its simulation data.
    /// Returns false for an invalid owner, pending work or callback reentry.
    /// Does not reset peer views. Attachment retirement also invalidates its views.
    bool(CARB_ABI* resetStage)(AttachHandle attachHandle);
};

} // namespace tensors
} // namespace physics
} // namespace omni
