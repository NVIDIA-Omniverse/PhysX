// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

namespace omni
{
namespace physx
{
namespace internal
{
bool setupTransformOpsAsScaleOrientTranslate(const pxr::UsdPrim& prim,
                                             pxr::GfMatrix4d* preMatrix = nullptr,
                                             bool* preMatrixValid = nullptr,
                                             pxr::GfMatrix4d* postMatrix = nullptr,
                                             bool* postMatrixValid = nullptr);
// extracts and returns scale vec in precision set in template
// use GfVec3d, GfVec3f, GfVec3h accordingly
// returns success of read && cast operation
template <class T>
bool getScaleFromXformOp(T& scaleVec, const pxr::UsdPrim& prim)
{
    pxr::UsdGeomXformable primXform(prim);
    if (!primXform)
    {
        return false;
    }

    bool resetXformStack = false;
    const std::vector<pxr::UsdGeomXformOp> xformOps = primXform.GetOrderedXformOps(&resetXformStack);
    // this will be used mostly on sanitized xform op stacks, so search for
    // scale op in reverse order as scale will usually be on top:
    for (auto crit = xformOps.crbegin(); crit != xformOps.crend(); ++crit)
    {
        if (crit->GetOpType() == pxr::UsdGeomXformOp::TypeScale)
        {
            return crit->GetAs<T>(&scaleVec, pxr::UsdTimeCode::Default());
        }
    }
    // not found
    return false;
}
} // namespace internal
} // namespace physx
} // namespace omni
