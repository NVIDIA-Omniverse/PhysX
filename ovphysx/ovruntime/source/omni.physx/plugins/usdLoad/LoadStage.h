// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <private/omni/physx/PhysxUsd.h>

#include <PhysXReplicator.h>

#include "LoadTools.h"
#include "Mass.h"

#include <string>
#include <vector>

namespace omni
{
namespace physx
{
namespace usdparser
{

// `scanRoots`/`updateRoots` are source-path strings (the vocabulary
// omni::physics::parse::scanStage takes) and `excludePaths` is the ObjectKey-keyed
// PathSet (LoadTools.h).
bool loadFromStage(AttachedStage& attachedStage, const PathSet* excludePaths = nullptr);
void loadPhysicsFromPrimitive(AttachedStage& attachedStage, const std::vector<std::string>& updateRoots);

} // namespace usdparser
} // namespace physx
} // namespace omni
