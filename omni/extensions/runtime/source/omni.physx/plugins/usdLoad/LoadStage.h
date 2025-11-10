// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/PhysxUsd.h>

#include <private/omni/physics/schema/IUsdPhysics.h>
#include <PhysXReplicator.h>

#include "LoadTools.h"
#include "Mass.h"

namespace omni
{
namespace physx
{
namespace usdparser
{

void loadFromStage(AttachedStage& attachedStage, const PathSet* excludePaths = nullptr);
void loadPhysicsFromPrimitive(AttachedStage& attachedStage, omni::physics::schema::PrimIteratorMapRange& primIteratorMap);

} // namespace usdparser
} // namespace physx
} // namespace omni
