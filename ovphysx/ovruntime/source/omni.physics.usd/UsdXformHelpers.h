// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-WRITE-TRANSFORM-001
 * @covers AC-4
 */
#pragma once

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/xformable.h>
#include <pxr/base/gf/matrix4d.h>

namespace omni::physics::usd
{

// Normalize a prim's xform-op stack so its transform is defined ONLY by the
// three default ops scale / orient(quat) / translate, in that order. All other
// ops are removed (their authored attributes retained); existing op precisions
// and a reset-xform-stack flag are preserved. Any ops that could not be folded
// (metrics-assembler unit-resolve ops) are returned as a residual pre/post
// matrix the caller folds into subsequent writes. Returns false on failure
// (e.g. no editable prim spec / unsupported op precision).
//
// USD/Gf-only; lives in the USD backend module so the USD-backed
// IPhysicsDataWrite sink can prepare a prim without depending on omni.physx.
bool setupTransformOpsAsScaleOrientTranslate(const PXR_NS::UsdPrim& prim,
                                             PXR_NS::GfMatrix4d* preMatrix = nullptr,
                                             bool* preMatrixValid = nullptr,
                                             PXR_NS::GfMatrix4d* postMatrix = nullptr,
                                             bool* postMatrixValid = nullptr);

// Nearest xformable ancestor of `prim` (skips prims that reset the xform
// stack). Returns false (and leaves `parentXformPrim` untouched) when `prim`
// resets its stack or no xformable ancestor exists.
inline bool getParentXform(const PXR_NS::UsdPrim& prim, PXR_NS::UsdPrim& parentXformPrim)
{
    if (PXR_NS::UsdGeomXformable(prim).GetResetXformStack())
    {
        return false;
    }

    PXR_NS::UsdPrim parent = prim.GetParent();
    while (parent && !parent.IsPseudoRoot())
    {
        if (parent.IsA<PXR_NS::UsdGeomXformable>())
        {
            parentXformPrim = parent;
            return true;
        }
        parent = parent.GetParent();
    }

    return false;
}

} // namespace omni::physics::usd
