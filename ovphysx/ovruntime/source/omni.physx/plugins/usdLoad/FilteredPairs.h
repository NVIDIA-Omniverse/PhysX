// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <private/omni/physx/PhysxUsd.h>
#include <omni/physics/usd/PrimIterator.h>

#include "AttachedStage.h"
#include "LoadTools.h"

namespace omni
{
namespace physx
{
namespace usdparser
{

void collectFilteredPairs(AttachedStage& attachedStage,
                          const PXR_NS::SdfPath& primKey,
                          const PXR_NS::SdfPathVector& filterPairPaths,
                          ObjectIdPairVector& pairVector);

} // namespace usdparser
} // namespace physx
} // namespace omni
