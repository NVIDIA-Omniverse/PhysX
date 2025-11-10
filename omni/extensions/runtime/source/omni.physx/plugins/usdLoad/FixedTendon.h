// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/PhysxUsd.h>

using namespace pxr;

namespace omni
{
namespace physx
{
namespace usdparser
{
void parseTendonAxes(AttachedStage& attachedStage,
                     UsdPrim prim,
                     const omni::physics::schema::JointDesc* jointDesc,
                     TendonAxisMap& tendonAxes,
                     FixedTendonVector& fixedTendons);

void createFixedTendons(AttachedStage& attachedStage, TendonAxisMap& tendonAxes, FixedTendonVector& fixedTendons);

std::vector<PhysxTendonAxisHierarchyDesc*> getAxesUiInfo(TendonAxisMap& tendonAxes, FixedTendonVector& fixedTendons);
} // namespace usdparser
} // namespace physx
} // namespace omni
