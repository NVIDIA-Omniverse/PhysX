// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-002
 * @covers AC-1
 */
#pragma once

#include <cstdint>

namespace omni
{
namespace physics
{

/// Identifies an *attach*, not a USD stage (ADR-0013).
///
/// This used to be a `long stageId` read as a UsdUtilsStageCache key, which made
/// the tensor API unusable for a source with no backing USD stage: such an attach
/// reports stage id 0, and 0 also means "nothing attached", so the value could
/// neither be resolved nor checked for staleness. A handle is nonzero for every
/// live attach and minted fresh per attach, so `handle == kNoAttach` means
/// exactly one thing, and comparing a stored handle against the current one
/// actually detects a detach or a reattach.
///
/// ADR-0016 generalises this from the tensor surface to the whole public runtime
/// API, which is why the type lives here rather than in `tensors`: every entry
/// point that names an *attach* takes an `AttachHandle`, and only entry points
/// that name the USD stage object itself keep a stage id.
///
/// Obtain one from `IPhysxSimulation::getAttachHandle()`.
using AttachHandle = uint64_t;

/// No attach. Never identifies a live attach.
constexpr AttachHandle kNoAttach = 0;

/// "Whichever attach is currently active" -- resolved at call time. Convenience
/// for callers that do not track a handle of their own.
///
/// \note This resolves only while exactly one attach is live:
/// `IPhysxSimulation::getAttachHandle()` returns `kNoAttach` under multi-attach,
/// so a consumer that attaches more than once must track its own handles.
constexpr AttachHandle kActiveAttach = ~AttachHandle(0);

} // namespace physics

namespace physx
{

// The attach identity is repo-wide, not a tensors or an omni.physx concept
// (ADR-0016 Decision 2). Aliased here so the omni.physx interfaces can spell it
// unqualified; it is the same type and the same sentinels as omni::physics'.
using omni::physics::AttachHandle;
using omni::physics::kActiveAttach;
using omni::physics::kNoAttach;

} // namespace physx
} // namespace omni
