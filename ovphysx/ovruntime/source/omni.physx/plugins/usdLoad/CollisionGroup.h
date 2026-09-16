// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "LoadTools.h"
#include <private/omni/physx/PhysxUsd.h>

namespace omni
{
namespace physx
{
namespace usdparser
{

// Append one collision group's resolved member list into `collisionGroupsMap`.
// The scan is rooted by source path and routed through the active scan backend,
// matching the initial-load collision-group resolution path.
//
// See ADR-0006 and REQ-PARSE-COLGROUP-002.
void appendCollisionGroupFromPath(AttachedStage& attachedStage,
                                  omni::physics::parse::ObjectKey collisionGroupKey,
                                  CollisionGroupsMap& collisionGroupsMap);

ObjectId getCollisionGroup(AttachedStage& attachedStage, omni::physics::parse::ObjectKey colliderKey);

} // namespace usdparser
} // namespace physx
} // namespace omni
