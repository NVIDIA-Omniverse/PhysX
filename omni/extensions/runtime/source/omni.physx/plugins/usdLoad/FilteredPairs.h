// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/PhysxUsd.h>
#include <private/omni/physics/schema/IUsdPhysics.h>

#include "AttachedStage.h"
#include "LoadTools.h"

namespace omni
{
namespace physx
{
namespace usdparser
{

void collectFilteredPairs(AttachedStage& attachedStage,
                          const pxr::SdfPath& primPath,
                          const pxr::SdfPathVector& filterPairPaths,
                          ObjectIdPairVector& pairVector);

} // namespace usdparser
} // namespace physx
} // namespace omni
