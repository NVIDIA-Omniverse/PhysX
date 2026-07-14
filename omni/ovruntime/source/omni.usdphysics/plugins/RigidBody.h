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
bool hasEnabledBodyParent(const PXR_NS::UsdStageWeakPtr stage,
                          const PXR_NS::UsdPrim& usdPrim,
                          const BodyMap& bodyMap,
                          PXR_NS::UsdPrim& bodyPrim,
                          ObjectType::Enum type);
PXR_NS::SdfPath getRigidBody(const PXR_NS::UsdStageWeakPtr stage, const PXR_NS::UsdPrim& usdPrim, const BodyMap& bodyMap);
RigidBodyDesc* parseRigidBody(const PXR_NS::UsdStageWeakPtr stage,
                              PXR_NS::UsdGeomXformCache& xfCache,
                              const PXR_NS::UsdPrim& bodyPrim,
                              const BodyMap& bodyMap,
                              uint64_t primTypes);

} // namespace schema
} // namespace physics
} // namespace omni
