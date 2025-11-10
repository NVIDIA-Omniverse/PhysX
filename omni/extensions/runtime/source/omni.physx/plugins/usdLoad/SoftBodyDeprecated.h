// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/PhysxUsd.h>
#include "LoadTools.h"


namespace omni
{
namespace physx
{
namespace usdparser
{
class AttachedStage;

FemSoftBodyMaterialDesc* ParseDeformableBodyMaterialDeprecated(const pxr::UsdPrim& usdPrim);
SoftBodyDesc* ParseDeformableBodyDeprecated(AttachedStage& attachedStage, const pxr::UsdPrim& usdPrim);

FemClothMaterialDesc* ParseDeformableSurfaceMaterialDeprecated(const pxr::UsdPrim& usdPrim);
FEMClothDesc* ParseDeformableSurfaceDeprecated(AttachedStage& attachedStage, const pxr::UsdPrim& usdPrim);
} // namespace usdparser
} // namespace physx
} // namespace omni
