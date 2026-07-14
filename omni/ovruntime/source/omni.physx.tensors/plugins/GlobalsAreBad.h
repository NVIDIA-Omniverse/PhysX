// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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

extern omni::physx::IPhysx* g_physx;
extern omni::physx::IPhysxSimulation* g_physxSimulation;
extern omni::physx::IPhysxPrivate* g_physxPrivate;
extern omni::physx::IPhysxJoint* g_physxJoint;

// When true (default), a *named* last token in a pattern passed to
// create_*_view is matched against every descendant name of the
// penultimate-level roots rather than only direct children. A bare `*`
// leaf is always strict (direct children only) regardless of this
// setting — callers who want to recurse below a prim must use `**`
// explicitly. kDefaultRecursiveLeafPatternMatch is the single source of
// truth: tensorsInit() passes it to setDefaultBool, and the runtime
// reader in BaseSimulationView falls back to it when the key is not
// present so the documented default does not depend on init order.
constexpr const char* kSettingRecursiveLeafPatternMatch = "/physics/tensors/recursiveLeafPatternMatch";
constexpr bool kDefaultRecursiveLeafPatternMatch = true;

} // namespace tensors
} // namespace physx
} // namespace omni
