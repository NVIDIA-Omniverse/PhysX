// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <pxr/usd/usd/prim.h>
#include <pxr/base/vt/value.h>
#include <pxr/usd/sdf/path.h>
#include <string>

namespace primutils
{
    // Common prim utility functions

    // Returns custom metadata (if found) associated with a prim into an 'output' buffer. Returns 'true' if the key
    // was found and 'output' is to be considered the metadata value.
    bool getMetaData(const pxr::UsdPrim& prim, const pxr::TfToken& key, std::string& output);
    bool setMetaData(const pxr::UsdPrim& prim, const pxr::TfToken& key, std::string value);
    bool removeMetaData(const pxr::UsdPrim& prim, const pxr::TfToken& key);
    // Checks whether a given prim is visible (also checks metadata) and no parent is marked as 'invisible'
    bool isHidden(const pxr::UsdPrim& prim);
    // Returns true if this prim is a parent of a collision mesh debug visualization. This prim might be marked as
    // 'invisible' but should still have physics parsing done on it
    bool isCollisionMeshParentPrim(const pxr::UsdPrim& prim);

    bool isBodyTransformEqual(  pxr::UsdStageWeakPtr stage, 
                                pxr::SdfPath body0,
                                pxr::SdfPath body1,
                                pxr::GfVec3f localPose0Position,
                                pxr::GfQuatf localPose0Orientation,
                                pxr::GfVec3f localPose1Position,
                                pxr::GfQuatf localPose1Orientation,
                                pxr::UsdGeomXformCache& xfCache,
                                double jointBodyTransformCheckTolerance,
                                bool checkPosition, bool checkRotation,
                                unsigned char axis = 0xff);


    template <typename T>
    inline void setMetadata(const pxr::UsdPrim& prim, const pxr::TfToken& token, T value)
    {
        pxr::UsdEditContext context(prim.GetStage(), prim.GetStage()->GetSessionLayer());
        bool ret = prim.SetMetadata(token, value);
    }

    inline void setNoDelete(const pxr::UsdPrim& prim, bool noDelete)
    {
        static const pxr::TfToken kNoDelete("no_delete");
        setMetadata(prim, kNoDelete, noDelete);
    }

    inline void setHideInStageWindow(const pxr::UsdPrim& prim, bool hide)
    {
        static const pxr::TfToken kHideInStageWindow("hide_in_stage_window");
        setMetadata(prim, kHideInStageWindow, hide);
    }

    bool IsTransformTimeVarying(const pxr::UsdPrim& prim);

} // namespace primutils
