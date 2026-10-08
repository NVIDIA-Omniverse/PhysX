// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <private/omni/physx/PhysxUsd.h>
#include "LoadTools.h"

namespace omni
{
namespace physx
{
namespace usdparser
{

// Traverse the articulation links and articulation joints to construct the articulation hierarchy and create it
void createArticulationLinks(AttachedStage& attachedStage,
                             BodyMap& bodyMap,
                             JointVector& jointVector,
                             const ArticulationMap& articulationMap,
                             const JointPathIndexMap& jointPathIndexMap);

} // namespace usdparser
} // namespace physx
} // namespace omni
