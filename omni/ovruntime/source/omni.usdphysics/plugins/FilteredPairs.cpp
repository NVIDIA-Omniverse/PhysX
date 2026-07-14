// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

using namespace PXR_NS;

namespace omni
{
namespace physics
{
namespace schema
{

void parseFilteredPairs(const PXR_NS::UsdStageWeakPtr stage, const PXR_NS::UsdPrim& usdPrim, PXR_NS::SdfPathVector& filteredPairs)
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
