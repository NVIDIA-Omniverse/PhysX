// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//


// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include "PrimUtilities.h"

// This additional GfIsClose is defined in <omni/usd/UsdUtils.h> but we don't really want to depend from kit in this lib
PXR_NAMESPACE_OPEN_SCOPE
inline bool GfIsClose(const PXR_NS::GfQuatd& val1, const PXR_NS::GfQuatd& val2, double tolerance)
{
    bool result1 = PXR_NS::GfIsClose(val1.GetReal(), val2.GetReal(), tolerance) &&
                   GfIsClose(val1.GetImaginary(), val2.GetImaginary(), tolerance);
    bool result2 = GfIsClose(val1.GetReal(), -val2.GetReal(), tolerance) &&
                   GfIsClose(val1.GetImaginary(), -val2.GetImaginary(), tolerance);
    return result1 || result2;
}
PXR_NAMESPACE_CLOSE_SCOPE

namespace primutils
{
    bool getMetaData(const PXR_NS::UsdPrim& prim, const PXR_NS::TfToken& key, std::string& output)
    {
        bool ret = false;

        PXR_NS::VtValue v;
        if (prim.GetMetadataByDictKey(PXR_NS::SdfFieldKeys->CustomData, key, &v))
        {
            output = v.Get<std::string>();
            ret = true;
        }

        return ret;
    }

    bool setMetaData(const PXR_NS::UsdPrim& prim, const PXR_NS::TfToken& key, std::string value)
    {
        return prim.SetMetadataByDictKey(PXR_NS::SdfFieldKeys->CustomData, key, value);
    }

    bool removeMetaData(const PXR_NS::UsdPrim& prim, const PXR_NS::TfToken& key)
    {
        return prim.ClearMetadataByDictKey(PXR_NS::SdfFieldKeys->CustomData, key);
    }

    bool isHidden(const PXR_NS::UsdPrim& prim)
    {
        bool ret = false;

        if (prim)
        {
            if (prim.IsA<PXR_NS::UsdGeomImageable>())
            {
                // We do allow invisible prims if they are associated as a parent of
                // a debug visualization collision prim
                static PXR_NS::TfToken cToken("CollisionMeshParent");
                std::string metadata;
                if (!getMetaData(prim, cToken, metadata))
                {
                    PXR_NS::UsdGeomImageable imageable(prim);
                    PXR_NS::TfToken vis = imageable.ComputeVisibility();
                    if (vis == PXR_NS::UsdGeomTokens->invisible)
                    {
                        ret = true;
                    }
                }
            }
        }
        return ret;
    }

    bool isCollisionMeshParentPrim(const PXR_NS::UsdPrim& prim)
    {
        bool ret = false;

        static PXR_NS::TfToken gCollisionMeshParentToken("CollisionMeshParent");
        std::string metadata;
        if (getMetaData(prim, gCollisionMeshParentToken, metadata))
        {
            ret = true;
        }

        return ret;
    }

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
                                unsigned char axis /*= 0xff*/)
    {
        auto getJointBodyTransform = [](const PXR_NS::GfMatrix4d& bodyWorld, bool bodyValid,
                                        const PXR_NS::GfVec3f& gfLocPos,
                                        const PXR_NS::GfQuatf& gfLocRot) -> PXR_NS::GfTransform {
            PXR_NS::GfTransform gizmoTr;
            gizmoTr.SetRotation(PXR_NS::GfRotation(gfLocRot));

            if (bodyValid)
            {
                const PXR_NS::GfVec3f scale = PXR_NS::GfVec3f(PXR_NS::GfTransform(bodyWorld).GetScale());
                gizmoTr.SetTranslation(
                    PXR_NS::GfVec3f(gfLocPos[0] / scale[0], gfLocPos[1] / scale[1], gfLocPos[2] / scale[2]));
                gizmoTr = gizmoTr * bodyWorld;
            }
            else
            {
                gizmoTr.SetTranslation(gfLocPos);
            }

            return gizmoTr;
        };

        const PXR_NS::GfTransform body0tm = getJointBodyTransform(body0World, body0Valid, localPose0Position, localPose0Orientation);
        const PXR_NS::GfTransform body1tm = getJointBodyTransform(body1World, body1Valid, localPose1Position, localPose1Orientation);

        const double eps = jointBodyTransformCheckTolerance;

        if (checkPosition)
        {
            if (axis < 3)
            {
                PXR_NS::GfVec3d tran = body0tm.GetTranslation() - body1tm.GetTranslation();

                static std::array<std::array<unsigned char, 2>, 3> axes = { { { 1, 2 }, { 0, 2 }, { 0, 1 } } };
                double a1 = tran[(axes[axis][0])];
                double a2 = tran[(axes[axis][1])];
                if (!(PXR_NS::GfIsClose(a1, 0.0, eps) && PXR_NS::GfIsClose(a2, 0.0, eps)))
                {
                    return false;
                }
            }
            else
            {
                if (!PXR_NS::GfIsClose(body0tm.GetTranslation(), body1tm.GetTranslation(), eps))
                {
                    return false;
                }
            }
        }

        if (checkRotation)
        {
            const PXR_NS::GfQuatd rot0 = body0tm.GetRotation().GetQuat();
            const PXR_NS::GfQuatd rot1 = body1tm.GetRotation().GetQuat();

            if (!PXR_NS::GfIsClose(rot0, rot1, eps))
            {
                return false;
            }
        }

        return true;
    }

    bool IsTransformTimeVarying(const PXR_NS::UsdPrim& prim)
    {
        // checking for animated transformations
        PXR_NS::UsdPrim parent = prim;
        const PXR_NS::UsdPrim root = prim.GetStage()->GetPseudoRoot();
        while (parent != root)
        {
            PXR_NS::UsdGeomXformable xform(parent);
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
