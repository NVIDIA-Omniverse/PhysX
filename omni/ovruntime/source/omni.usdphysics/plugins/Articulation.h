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

ArticulationDesc* parseArticulation(const PXR_NS::UsdStageWeakPtr stage,
                                    const PXR_NS::UsdPrim& usdPrim,
                                    const ArticulationMap& articulationMap);

void finalizeArticulations(const PXR_NS::UsdStageWeakPtr stage,
                           ArticulationMap& articulationMap,
                           const BodyMap& bodyMap,
                           const JointMap& jointMap);

} // namespace schema
} // namespace physics
} // namespace omni
