// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/PhysxUsd.h>
#include "LoadTools.h"

namespace omni
{
namespace physx
{
namespace usdparser
{

void parseArticulation(const pxr::UsdStageWeakPtr stage,
                       const omni::physics::schema::ArticulationDesc& inDesc,
                       CollisionPairVector& filteredPairs,
                       ArticulationMap& articulationMap);

// Traverse the articulation links and articulation joints to construct the articulation hierarchy and create it
void createArticulationLinks(AttachedStage& attachedStage,
                             pxr::UsdGeomXformCache& xformCache,
                             BodyMap& bodyMap,
                             JointVector& jointVector,
                             const ArticulationMap& articulationMap,
                             const JointPathIndexMap& jointPathIndexMap);

bool checkArticulatonBodySimulationOwners(AttachedStage& attachedStage,
                                          pxr::UsdStageWeakPtr stage,
                                          const omni::physics::schema::ArticulationDesc& inDesc);

} // namespace usdparser
} // namespace physx
} // namespace omni
