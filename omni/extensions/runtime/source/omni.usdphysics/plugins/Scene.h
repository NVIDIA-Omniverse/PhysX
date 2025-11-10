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

SceneDesc* parseSceneDesc(const pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& usdPrim);

} // namespace schema
} // namespace physics
} // namespace omni
