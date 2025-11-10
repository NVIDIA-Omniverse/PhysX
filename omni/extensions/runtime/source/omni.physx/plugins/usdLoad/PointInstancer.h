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
using ObjectIdVector = std::vector<ObjectId>;
using ShapeDescVector = std::vector<std::pair<pxr::SdfPath, PhysxShapeDesc*>>;
struct TargetDesc
{
    TargetDesc() : desc(nullptr), outsideInstancer(false)
    {
    }

    PhysxObjectDesc* desc;
    pxr::SdfPath descPath;
    ShapeDescVector shapeDescVector;
    pxr::UsdPrim targetPrim;
    bool outsideInstancer;
};

using TargetDescVector = std::vector<TargetDesc>;


void parseRigidBodyInstancer(AttachedStage& attachedStage,
                             pxr::UsdGeomXformCache& xfCache,
                             const pxr::UsdPrim& usdPrim,
                             CollisionPairVector& filteredPairs);

bool isOutsideInstancer(const pxr::UsdPrim& prim, const pxr::UsdPrim& instancerPrim);
} // namespace usdparser
} // namespace physx
} // namespace omni
