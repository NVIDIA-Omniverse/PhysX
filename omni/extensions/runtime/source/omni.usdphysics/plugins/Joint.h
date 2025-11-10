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

// parse the given PhysicsJoint prim and return filled joint desc
JointDesc* parseJoint(const pxr::UsdStageWeakPtr stage,
                      const pxr::UsdPrim& usdPrim,
                      const pxr::TfTokenVector& customTokens,
                      uint64_t primTypes);

void finalizeJoint(const pxr::UsdStageWeakPtr stage, JointDesc* jointDesc, pxr::UsdGeomXformCache& xfCache);

} // namespace schema
} // namespace physics
} // namespace omni
