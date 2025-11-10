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
bool hasEnabledBodyParent(const pxr::UsdStageWeakPtr stage,
                          const pxr::UsdPrim& usdPrim,
                          const BodyMap& bodyMap,
                          pxr::UsdPrim& bodyPrim,
                          ObjectType::Enum type);
pxr::SdfPath getRigidBody(const pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& usdPrim, const BodyMap& bodyMap);
RigidBodyDesc* parseRigidBody(const pxr::UsdStageWeakPtr stage,
                              pxr::UsdGeomXformCache& xfCache,
                              const pxr::UsdPrim& bodyPrim,
                              const BodyMap& bodyMap,
                              uint64_t primTypes);

} // namespace schema
} // namespace physics
} // namespace omni
