// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-WRITE-LOCALXFORM-001
 * @covers AC-4
 */

#pragma once

#include <pxr/base/gf/quatf.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/vt/value.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/editContext.h>
#include <pxr/usd/usd/prim.h>

#include <foundation/PxMat44.h>

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
    //
    // The world transforms are PxMat44d because that is what the callers hold
    // (the source hands out PhysX matrices); the Gf re-typing they need for the
    // comparison below happens on this side of the boundary, so the caller need
    // not touch Gf. Element copy, no transpose -- the two types hold the same
    // sixteen doubles (see common/foundation/MatrixTools.h).
    bool isBodyTransformEqual(  const ::physx::PxMat44d& body0World,
                                bool body0Valid,
                                const ::physx::PxMat44d& body1World,
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
