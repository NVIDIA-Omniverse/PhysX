// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "UsdLoad.h"

namespace omni
{
namespace physics
{
namespace schema
{

CollisionGroupDesc* parseCollisionGroup(const pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& usdPrim);
pxr::SdfPath getCollisionGroup(const pxr::SdfPath& colliderPath, const CollisionGroupMap& collisionGroupsMap);

} // namespace schema
} // namespace physics
} // namespace omni
