// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-FEED-006
 * @covers AC-7
 */

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
// Create the physics scene at `scenePath` as a whole-stage load does, from the same
// source reads, without any of its descendants. A compiled-attachment hit restores
// everything else from its bundle.
bool loadSceneFromStage(AttachedStage& attachedStage, const std::string& scenePath);
// loadSceneFromStage in two steps: read the descriptor of the scene `key` names (false
// when the prim is no PhysX scene), then create the scene. A whole-stage compiled-cache
// attach reads before it captures the stage's inputs and creates while they are captured.
bool readSceneFromStage(AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, PhysxSceneDesc& desc);
ObjectId createSceneFromDesc(AttachedStage& attachedStage, PhysxSceneDesc& desc);
bool loadPhysicsFromPrimitive(AttachedStage& attachedStage, const std::vector<std::string>& updateRoots,
                              bool applyAttachmentExclusions = true);

} // namespace usdparser
} // namespace physx
} // namespace omni
