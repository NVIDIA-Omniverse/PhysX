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

// parse the given PhysicsJoint prim and return filled joint desc
JointDesc* parseJoint(const PXR_NS::UsdStageWeakPtr stage,
                      const PXR_NS::UsdPrim& usdPrim,
                      const PXR_NS::TfTokenVector& customTokens,
                      uint64_t primTypes);

void finalizeJoint(const PXR_NS::UsdStageWeakPtr stage, JointDesc* jointDesc, PXR_NS::UsdGeomXformCache& xfCache);

} // namespace schema
} // namespace physics
} // namespace omni
