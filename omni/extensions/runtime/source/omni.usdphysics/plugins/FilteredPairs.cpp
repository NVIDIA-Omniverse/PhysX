// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <private/omni/physics/schema/IUsdPhysicsListener.h>

#include "FilteredPairs.h"

using namespace pxr;

namespace omni
{
namespace physics
{
namespace schema
{

void parseFilteredPairs(const pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& usdPrim, pxr::SdfPathVector& filteredPairs)
{    
    UsdPhysicsFilteredPairsAPI filteredPairsAPI = UsdPhysicsFilteredPairsAPI::Get(stage, usdPrim.GetPrimPath());
    if (filteredPairsAPI && filteredPairsAPI.GetFilteredPairsRel())
    {
        filteredPairsAPI.GetFilteredPairsRel().GetTargets(&filteredPairs);
    }
}

} // namespace schema
} // namespace physics
} // namespace omni
