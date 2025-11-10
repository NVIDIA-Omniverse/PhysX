// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

// parse the collision, this is the main function parsing the geom prim and getting
// back the shape descriptor based on the geom prim and schema definition
void parseCollision(const pxr::UsdStageWeakPtr stage,
                    pxr::UsdGeomXformCache& xfCache,
                    const pxr::UsdPrim& usdPrim,
                    const TokenVector& customTokens,
                    const pxr::TfTokenVector& apis,
                    std::vector<ShapeDesc*>& shapes);


// finalize the collision information, compute local transforms based on the body
void finalizeCollision(const pxr::UsdStageWeakPtr stage,
                       const RigidBodyDesc* bodyDesc,
                       pxr::UsdGeomXformCache& xfCache,
                       ShapeDesc* shapeDesc);

} // namespace schema
} // namespace physics
} // namespace omni
