// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

void updateCollisionCollection(const pxr::UsdPrim& collisionGroupPrim, CollisionGroupsMap& collisionGroupsMap);
ObjectId getCollisionGroup(AttachedStage& attachedStage, const pxr::SdfPath& colliderPath);
void updateFabricCollisionGroups(AttachedStage& attachedStage);

} // namespace usdparser
} // namespace physx
} // namespace omni
