// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

CollisionGroupDesc* parseCollisionGroup(const PXR_NS::UsdStageWeakPtr stage, const PXR_NS::UsdPrim& usdPrim);
PXR_NS::SdfPath getCollisionGroup(const PXR_NS::SdfPath& colliderPath, const CollisionGroupMap& collisionGroupsMap);

} // namespace schema
} // namespace physics
} // namespace omni
