// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include "PrimUtilities.h"

// This additional GfIsClose is defined in <omni/usd/UsdUtils.h> but we don't really want to depend from kit in this lib
PXR_NAMESPACE_OPEN_SCOPE
inline bool GfIsClose(const pxr::GfQuatd& val1, const pxr::GfQuatd& val2, double tolerance)
{
    bool result1 = pxr::GfIsClose(val1.GetReal(), val2.GetReal(), tolerance) &&
                   GfIsClose(val1.GetImaginary(), val2.GetImaginary(), tolerance);
    bool result2 = GfIsClose(val1.GetReal(), -val2.GetReal(), tolerance) &&
                   GfIsClose(val1.GetImaginary(), -val2.GetImaginary(), tolerance);
    return result1 || result2;
}
PXR_NAMESPACE_CLOSE_SCOPE

namespace primutils
{
    bool getMetaData(const pxr::UsdPrim& prim, const pxr::TfToken& key, std::string& output)
    {
        bool ret = false;

        pxr::VtValue v;
        if (prim.GetMetadataByDictKey(pxr::SdfFieldKeys->CustomData, key, &v))
        {
            output = v.Get<std::string>();
            ret = true;
        }

        return ret;
    }

    bool setMetaData(const pxr::UsdPrim& prim, const pxr::TfToken& key, std::string value)
    {
        return prim.SetMetadataByDictKey(pxr::SdfFieldKeys->CustomData, key, value);
    }

    bool removeMetaData(const pxr::UsdPrim& prim, const pxr::TfToken& key)
    {
        return prim.ClearMetadataByDictKey(pxr::SdfFieldKeys->CustomData, key);
    }

    bool isHidden(const pxr::UsdPrim& prim)
    {
        bool ret = false;

        if (prim)
        {
            if (prim.IsA<pxr::UsdGeomImageable>())
            {
                // We do allow invisible prims if they are associated as a parent of
                // a debug visualization collision prim
                static pxr::TfToken cToken("CollisionMeshParent");
                std::string metadata;
                if (!getMetaData(prim, cToken, metadata))
                {
                    pxr::UsdGeomImageable imageable(prim);
                    pxr::TfToken vis = imageable.ComputeVisibility();
                    if (vis == pxr::UsdGeomTokens->invisible)
                    {
                        ret = true;
                    }
                }
            }
        }
        return ret;
    }

    bool isCollisionMeshParentPrim(const pxr::UsdPrim& prim)
    {
        bool ret = false;

        static pxr::TfToken gCollisionMeshParentToken("CollisionMeshParent");
        std::string metadata;
        if (getMetaData(prim, gCollisionMeshParentToken, metadata))
        {
            ret = true;
        }

        return ret;
    }

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
                                unsigned char axis /*= 0xff*/)
    {
        auto getJointBodyTransform = [&xfCache](const pxr::UsdPrim& bodyPrim, const pxr::GfVec3f& gfLocPos,
                                                const pxr::GfQuatf& gfLocRot) -> pxr::GfTransform {
            pxr::GfTransform gizmoTr;
            gizmoTr.SetRotation(pxr::GfRotation(gfLocRot));

            if (bodyPrim.IsValid())
            {
                const pxr::GfMatrix4d mat = xfCache.GetLocalToWorldTransform(bodyPrim);
                const pxr::GfVec3f scale = pxr::GfVec3f(pxr::GfTransform(mat).GetScale());
                gizmoTr.SetTranslation(
                    pxr::GfVec3f(gfLocPos[0] / scale[0], gfLocPos[1] / scale[1], gfLocPos[2] / scale[2]));
                gizmoTr = gizmoTr * mat;
            }
            else
            {
                gizmoTr.SetTranslation(gfLocPos);
            }

            return gizmoTr;
        };

        const pxr::UsdPrim body0prim = stage->GetPrimAtPath(body0);
        const pxr::UsdPrim body1prim = stage->GetPrimAtPath(body1);

        const pxr::GfTransform body0tm = getJointBodyTransform(body0prim, localPose0Position, localPose0Orientation);
        const pxr::GfTransform body1tm = getJointBodyTransform(body1prim, localPose1Position, localPose1Orientation);

        const double eps = jointBodyTransformCheckTolerance;

        if (checkPosition)
        {
            if (axis < 3)
            {
                pxr::GfVec3d tran = body0tm.GetTranslation() - body1tm.GetTranslation();

                static std::array<std::array<unsigned char, 2>, 3> axes = { { { 1, 2 }, { 0, 2 }, { 0, 1 } } };
                double a1 = tran[(axes[axis][0])];
                double a2 = tran[(axes[axis][1])];
                if (!(pxr::GfIsClose(a1, 0.0, eps) && pxr::GfIsClose(a2, 0.0, eps)))
                {
                    return false;
                }
            }
            else
            {
                if (!pxr::GfIsClose(body0tm.GetTranslation(), body1tm.GetTranslation(), eps))
                {
                    return false;
                }
            }
        }

        if (checkRotation)
        {
            const pxr::GfQuatd rot0 = body0tm.GetRotation().GetQuat();
            const pxr::GfQuatd rot1 = body1tm.GetRotation().GetQuat();

            if (!pxr::GfIsClose(rot0, rot1, eps))
            {
                return false;
            }
        }

        return true;
    }

    bool IsTransformTimeVarying(const pxr::UsdPrim& prim)
    {
        // checking for animated transformations
        pxr::UsdPrim parent = prim;
        const pxr::UsdPrim root = prim.GetStage()->GetPseudoRoot();
        while (parent != root)
        {
            pxr::UsdGeomXformable xform(parent);
            bool resetsXformStack;
            for (auto xformOp : xform.GetOrderedXformOps(&resetsXformStack))
            {
                if (xformOp.GetNumTimeSamples() > 1)
                {
                    return true;
                }
            }
            parent = parent.GetParent();
        }
        return false;
    }

} // namespace primutils
