// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

namespace omni
{
namespace physx
{
struct IPhysx;
struct IPhysxSimulation;
struct IPhysxPrivate;
struct IPhysxJoint;

namespace tensors
{
class SimulationBackend;

SimulationBackend* GetSimulationBackend();  // Returns nullptr if backend not initialized or already shutdown

// Tensor-plugin lifecycle, driven from the owner of the static omni.physx runtime.
// tensorsPluginStartup() creates the primary CUDA context (GPU mode) then runs tensorsInit()
void tensorsPluginStartup();
void tensorsShutdown();

extern omni::physx::IPhysx* g_physx;
extern omni::physx::IPhysxSimulation* g_physxSimulation;
extern omni::physx::IPhysxPrivate* g_physxPrivate;
extern omni::physx::IPhysxJoint* g_physxJoint;

// When true (default), the pre-leaf tokens of a pattern are matched strictly
// (level by level) and only the final named or glob leaf token is searched at
// any depth beneath the matched parent, with same-name suppression. For example
// `/envs/*/Robot/link_0` finds a `link_0` nested anywhere under `Robot` (Isaac
// Sim URDF-converter output), while `/envs/*/Robot/base_link` matches only the
// `base_link` under the strictly-matched `Robot` -- it does NOT fan out to a
// `Robot` buried elsewhere (e.g. `Group/Robot/base_link`); use an explicit `**`
// for that. A same name nested inside its own match (base_link under base_link)
// is suppressed. A bare `*` leaf is always strict (direct children only)
// regardless of this setting -- callers who want to recurse below a prim must
// use `**` explicitly. kDefaultRecursiveLeafPatternMatch is the single source of
// truth: tensorsInit() passes it to setDefaultBool, and the runtime
// reader in BaseSimulationView falls back to it when the key is not
// present so the documented default does not depend on init order.
constexpr const char* kSettingRecursiveLeafPatternMatch = "/physics/tensors/recursiveLeafPatternMatch";
constexpr bool kDefaultRecursiveLeafPatternMatch = true;

} // namespace tensors
} // namespace physx
} // namespace omni
