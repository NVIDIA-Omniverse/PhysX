// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdLoad.h"

namespace omni
{
namespace physics
{
namespace schema
{
DeformableBodyDesc* parseDeformableBody(const pxr::UsdStageWeakPtr stage,
                                        pxr::UsdGeomXformCache& xfCache,
                                        const pxr::UsdPrim& bodyPrim,
                                        const BodyMap& bodyMap,
                                        uint64_t primTypes);

} // namespace schema
} // namespace physics
} // namespace omni
