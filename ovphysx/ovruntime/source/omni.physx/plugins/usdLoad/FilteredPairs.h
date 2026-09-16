// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <private/omni/physx/PhysxUsd.h>

#include "AttachedStage.h"
#include "LoadTools.h"

namespace omni
{
namespace physx
{
namespace usdparser
{

void collectFilteredPairs(AttachedStage& attachedStage,
                          omni::physics::parse::ObjectKey primKey,
                          const std::vector<omni::physics::parse::ObjectKey>& filterPairKeys,
                          ObjectIdPairVector& pairVector);

} // namespace usdparser
} // namespace physx
} // namespace omni
