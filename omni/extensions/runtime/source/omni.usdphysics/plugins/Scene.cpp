// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <private/omni/physics/schema/IUsdPhysicsListener.h>

#include "Scene.h"

using namespace pxr;

namespace omni
{
namespace physics
{
namespace schema
{


SceneDesc* parseSceneDesc(const UsdStageWeakPtr stage, const UsdPrim& usdPrim)
{
    const UsdPhysicsScene scene = (UsdPhysicsScene)usdPrim;
    SceneDesc* sceneDesc = new SceneDesc();

    GfVec3f gravityDirection;
    scene.GetGravityDirectionAttr().Get(&gravityDirection);
    if (gravityDirection == GfVec3f(0.0f))
    {
        TfToken upAxis = pxr::UsdGeomGetStageUpAxis(stage);
        if (upAxis == pxr::UsdGeomTokens.Get()->x)
            gravityDirection = GfVec3f(-1.0f, 0.0f, 0.0f);
        else if (upAxis == pxr::UsdGeomTokens.Get()->y)
            gravityDirection = GfVec3f(0.0f, -1.0f, 0.0f);
        else
            gravityDirection = GfVec3f(0.0f, 0.0f, -1.0f);
    }
    else
    {
        gravityDirection.Normalize();
    }

    float gravityMagnitude;
    scene.GetGravityMagnitudeAttr().Get(&gravityMagnitude);
    if (gravityMagnitude < -0.5e38f)
    {
        float metersPerUnit = (float)pxr::UsdGeomGetStageMetersPerUnit(stage);
        gravityMagnitude = 9.81f / metersPerUnit;
    }

    sceneDesc->gravityMagnitude = gravityMagnitude;
    sceneDesc->gravityDirection = gravityDirection;
    return sceneDesc;
}

} // namespace schema
} // namespace physics
} // namespace omni
