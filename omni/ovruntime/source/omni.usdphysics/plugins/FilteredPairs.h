// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

namespace omni
{
namespace physics
{
namespace schema
{

void parseFilteredPairs(const PXR_NS::UsdStageWeakPtr stage, const PXR_NS::UsdPrim& usdPrim, PXR_NS::SdfPathVector& filteredPairs);

} // namespace schema
} // namespace physics
} // namespace omni
