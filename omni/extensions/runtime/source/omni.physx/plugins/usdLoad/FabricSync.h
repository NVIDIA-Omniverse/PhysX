// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <omni/fabric/FabricUSD.h>
#include <pxr/usd/usd/stage.h>

namespace omni
{
namespace physx
{

// Applies pending changes from USD to Fabric and updates transform cache in Fabric
void flushUsdToFabric(const omni::fabric::UsdStageId& stageId, bool changeTime, double timeCode = 0.0);
void flushUsdToFabric(const PXR_NS::UsdStageWeakPtr& stage, bool changeTime, double timeCode = 0.0);

} // namespace physx
} // namespace omni
