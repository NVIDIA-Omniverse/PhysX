// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
using ObjectIdVector = std::vector<ObjectId>;
using ShapeDescVector = std::vector<std::pair<PXR_NS::SdfPath, PhysxShapeDesc*>>;
struct TargetDesc
{
    TargetDesc() : desc(nullptr), outsideInstancer(false)
    {
    }

    PhysxObjectDesc* desc;
    PXR_NS::SdfPath descPath;
    ShapeDescVector shapeDescVector;
    PXR_NS::UsdPrim targetPrim;
    bool outsideInstancer;
};

using TargetDescVector = std::vector<TargetDesc>;


void parseRigidBodyInstancer(AttachedStage& attachedStage,
                             PXR_NS::UsdGeomXformCache& xfCache,
                             const PXR_NS::UsdPrim& usdPrim,
                             CollisionPairVector& filteredPairs);

bool isOutsideInstancer(const PXR_NS::UsdPrim& prim, const PXR_NS::UsdPrim& instancerPrim);
} // namespace usdparser
} // namespace physx
} // namespace omni
