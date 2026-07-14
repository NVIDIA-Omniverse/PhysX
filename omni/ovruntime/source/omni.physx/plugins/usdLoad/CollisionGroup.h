// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "LoadTools.h"
#include <private/omni/physx/PhysxUsd.h>

namespace omni
{
namespace physx
{
namespace usdparser
{

void updateCollisionCollection(const PXR_NS::UsdPrim& collisionGroupPrim, CollisionGroupsMap& collisionGroupsMap);
ObjectId getCollisionGroup(AttachedStage& attachedStage, const PXR_NS::SdfPath& colliderPath);
void updateFabricCollisionGroups(AttachedStage& attachedStage);

} // namespace usdparser
} // namespace physx
} // namespace omni
