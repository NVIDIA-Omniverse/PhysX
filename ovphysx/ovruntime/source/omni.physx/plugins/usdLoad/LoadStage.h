// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <private/omni/physx/PhysxUsd.h>

#include <PhysXReplicator.h>

#include "LoadTools.h"
#include "Mass.h"

#include <set>

namespace omni
{
namespace physx
{
namespace usdparser
{

void loadFromStage(AttachedStage& attachedStage, const PathSet* excludePaths = nullptr);
void loadPhysicsFromPrimitive(AttachedStage& attachedStage, const std::set<PXR_NS::SdfPath>& updateRoots);

} // namespace usdparser
} // namespace physx
} // namespace omni
