// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <pxr/usd/usd/prim.h>
#include <pxr/base/vt/value.h>
#include <pxr/usd/sdf/path.h>
#include <string>

namespace primutils
{
    // Returns custom metadata (if found) associated with a prim into an 'output' buffer. Returns 'true' if the key
    // was found and 'output' is to be considered the metadata value.
    bool getMetaData(const PXR_NS::UsdPrim& prim, const PXR_NS::TfToken& key, std::string& output);
    bool setMetaData(const PXR_NS::UsdPrim& prim, const PXR_NS::TfToken& key, std::string value);
    bool removeMetaData(const PXR_NS::UsdPrim& prim, const PXR_NS::TfToken& key);
    // Checks whether a given prim is visible (also checks metadata) and no parent is marked as 'invisible'
    bool isHidden(const PXR_NS::UsdPrim& prim);
    // Returns true if this prim is a parent of a collision mesh debug visualization. This prim might be marked as
    // 'invisible' but should still have physics parsing done on it
    bool isCollisionMeshParentPrim(const PXR_NS::UsdPrim& prim);

    // Body world transforms are supplied by the caller (e.g. read through the
    // physics source) rather than computed here from a UsdGeomXformCache, so
    // this common utility stays free of scene-access concerns. `bodyNValid`
    // indicates whether `bodyNWorld` is valid: when false, `bodyNWorld` is
    // ignored and only the local pose is used.
    bool isBodyTransformEqual(  const PXR_NS::GfMatrix4d& body0World,
                                bool body0Valid,
                                const PXR_NS::GfMatrix4d& body1World,
                                bool body1Valid,
                                PXR_NS::GfVec3f localPose0Position,
                                PXR_NS::GfQuatf localPose0Orientation,
                                PXR_NS::GfVec3f localPose1Position,
                                PXR_NS::GfQuatf localPose1Orientation,
                                double jointBodyTransformCheckTolerance,
                                bool checkPosition, bool checkRotation,
                                unsigned char axis = 0xff);


    template <typename T>
    inline void setMetadata(const PXR_NS::UsdPrim& prim, const PXR_NS::TfToken& token, T value)
    {
        PXR_NS::UsdEditContext context(prim.GetStage(), prim.GetStage()->GetSessionLayer());
        bool ret = prim.SetMetadata(token, value);
    }

    inline void setNoDelete(const PXR_NS::UsdPrim& prim, bool noDelete)
    {
        static const PXR_NS::TfToken kNoDelete("no_delete");
        setMetadata(prim, kNoDelete, noDelete);
    }

    inline void setHideInStageWindow(const PXR_NS::UsdPrim& prim, bool hide)
    {
        static const PXR_NS::TfToken kHideInStageWindow("hide_in_stage_window");
        setMetadata(prim, kHideInStageWindow, hide);
    }

    bool IsTransformTimeVarying(const PXR_NS::UsdPrim& prim);

} // namespace primutils
