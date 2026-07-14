// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
void parseCollision(const PXR_NS::UsdStageWeakPtr stage,
                    PXR_NS::UsdGeomXformCache& xfCache,
                    const PXR_NS::UsdPrim& usdPrim,
                    const TokenVector& customTokens,
                    const PXR_NS::TfTokenVector& apis,
                    std::vector<ShapeDesc*>& shapes);


// finalize the collision information, compute local transforms based on the body
void finalizeCollision(const PXR_NS::UsdStageWeakPtr stage,
                       const RigidBodyDesc* bodyDesc,
                       PXR_NS::UsdGeomXformCache& xfCache,
                       ShapeDesc* shapeDesc);

} // namespace schema
} // namespace physics
} // namespace omni
