// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/PhysxUsd.h>

namespace omni
{
namespace physx
{
namespace usdparser
{


PhysxSceneDesc* parseSceneDesc(const pxr::UsdStageWeakPtr stage, const omni::physics::schema::SceneDesc& sceneDesc);
void setToDefault(pxr::UsdStageWeakPtr stage, PhysxSceneDesc& desc);

} // namespace usdparser
} // namespace physx
} // namespace omni
