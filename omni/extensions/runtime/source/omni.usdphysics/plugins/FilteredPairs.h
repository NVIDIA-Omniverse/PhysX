// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

namespace omni
{
namespace physics
{
namespace schema
{

void parseFilteredPairs(const pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& usdPrim, pxr::SdfPathVector& filteredPairs);

} // namespace schema
} // namespace physics
} // namespace omni
