// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "InternalTools.h"

#include <carb/logging/Log.h>

using namespace pxr;

namespace omni
{
namespace physx
{
namespace internal
{

static inline void setVecWithPrecision(UsdGeomXformOp& op, const GfVec3d& vec)
{
    if (!op)
        return;

    const UsdGeomXformOp::Precision precision = op.GetPrecision();
    if (precision == pxr::UsdGeomXformOp::PrecisionFloat)
        op.Set(pxr::GfVec3f(vec));
    else if (precision == pxr::UsdGeomXformOp::PrecisionDouble)
        op.Set(vec);
    else if (precision == pxr::UsdGeomXformOp::PrecisionHalf)
        op.Set(pxr::GfVec3h(vec));
}

static inline void setQuatWithPrecision(UsdGeomXformOp& op, const GfQuatd& orient)
{
    if (!op)
        return;

    const UsdGeomXformOp::Precision precision = op.GetPrecision();
    if (precision == pxr::UsdGeomXformOp::PrecisionFloat)
        op.Set(pxr::GfQuatf(orient));
    else if (precision == pxr::UsdGeomXformOp::PrecisionDouble)
        op.Set(orient);
    else if (precision == pxr::UsdGeomXformOp::PrecisionHalf)
        op.Set(pxr::GfQuath(orient));
}

// This setup code makes sure that the prim's transform is defined ONLY by the three default
// xFormOps scale, orient (i.e. quat), and translate, in that order (reverse in the op stack std::vector).
// All other xFormOps are removed from the stack, but their authored attributes are retained.
// If any of the three default ops are already present, their precision is retained.
// A reset xform stack flag will also be retained.
bool setupTransformOpsAsScaleOrientTranslate(const pxr::UsdPrim& prim, pxr::GfMatrix4d* preMatrix, bool* preMatrixValid, pxr::GfMatrix4d* postMatrix, bool* postMatrixValid)
{
    static const PXR_NS::TfToken scaleOpAttributeName = UsdGeomXformOp::GetOpName(UsdGeomXformOp::TypeScale);
    static const PXR_NS::TfToken metricsAssemblerSuffixName("unitsResolve");

    // default op list: Translate, Orient quat rotation, Scale
    constexpr UsdGeomXformOp::Type defaultOpList[3] = { UsdGeomXformOp::TypeTranslate, UsdGeomXformOp::TypeOrient,
                                                        UsdGeomXformOp::TypeScale };
    pxr::UsdGeomXformable primXform(prim);

    bool resetXfromStack = false;
    const std::vector<UsdGeomXformOp> xformOps = primXform.GetOrderedXformOps(&resetXfromStack);

    bool hasScale = false;
    UsdGeomXformOp::Precision scalePrecision = UsdGeomXformOp::Precision::PrecisionFloat;
    UsdGeomXformOp::Precision translatePrecision = UsdGeomXformOp::Precision::PrecisionFloat;
    UsdGeomXformOp::Precision orientPrecision = UsdGeomXformOp::Precision::PrecisionFloat;

    // check stack and get precisions
    bool isStackDefault = xformOps.size() == 3u; // must have exactly three ops defined
    bool preOp = false;
    GfMatrix4d extraTr(1.0);
    std::vector<UsdGeomXformOp> extraOps;
    for (size_t i = 0; i < xformOps.size(); ++i)
    {
        const UsdGeomXformOp& op = xformOps[i];
        isStackDefault &= op.GetOpType() == defaultOpList[i];

        if (op.HasSuffix(metricsAssemblerSuffixName))
        {
            if (i == 0)
                preOp = true;
            extraOps.push_back(op);
        }
        else
        {
            if (op.GetOpType() == UsdGeomXformOp::TypeScale)
            {
                scalePrecision = op.GetPrecision();
                hasScale = true;
            }
            else if (op.GetOpType() == UsdGeomXformOp::TypeTransform)
            {
                scalePrecision = op.GetPrecision();
                hasScale = true;
            }
            else if (op.GetOpType() == UsdGeomXformOp::TypeOrient)
            {
                orientPrecision = op.GetPrecision();
            }
            else if (op.GetOpType() == UsdGeomXformOp::TypeTranslate && !op.HasSuffix(PXR_NS::UsdGeomTokens->pivot))
            {
                translatePrecision = op.GetPrecision();
            }
        }

    }

    if (!extraOps.empty())
    {        
        for (size_t i = 0; i < extraOps.size(); ++i)
        {
            const UsdGeomXformOp& op = extraOps[i];
            extraTr *= op.GetOpTransform(UsdTimeCode::Default());
        }
        if (preOp)
        {
            if (preMatrixValid)
                *preMatrixValid = true;
            if (postMatrixValid)
                *postMatrixValid = false;
            if (preMatrix)
                *preMatrix = extraTr;
        }
        else
        {
            if (preMatrixValid)
                *preMatrixValid = false;
            if (postMatrixValid)
                *postMatrixValid = true;
            if (postMatrix)
                *postMatrix = extraTr;
        }
    }

    if (!isStackDefault)
    {
        bool resetStack;
        pxr::GfMatrix4d localToParent;
        primXform.GetLocalTransformation(&localToParent, &resetStack, xformOps, pxr::UsdTimeCode::Default());

        // Test if rotation matrix part is pure scaling and rotation - returns true also if rows are scaled
        if (!localToParent.HasOrthogonalRows3())
            CARB_LOG_WARN("Skew detected on prim %s that will be removed resulting in altered transform.",
                          prim.GetPath().GetString().c_str());

        // extra transform
        if (!extraOps.empty())
        {
            const GfMatrix4d extraTrInv = extraTr.GetInverse();
            if (preOp)
            {
                localToParent = localToParent * extraTrInv;
            }
            else
            {
                localToParent = extraTrInv * localToParent;
            }
        }

        pxr::GfTransform localToParentTransform(localToParent);

        // Sdf Code path is needed here
        const SdfPath primPath = prim.GetPrimPath();
        pxr::SdfPrimSpecHandle primSpec =
            pxr::SdfCreatePrimInLayer(prim.GetStage()->GetEditTarget().GetLayer(), primPath);

        if (!primSpec)
            return false;

        static TfToken gTranslate("xformOp:translate");
        pxr::SdfPath attributePath = primPath.AppendProperty(gTranslate);
        SdfAttributeSpecHandle posAttr = primSpec->GetAttributeAtPath(attributePath);
        if (!posAttr)
        {
            if (translatePrecision == UsdGeomXformOp::Precision::PrecisionFloat)
                posAttr = pxr::SdfAttributeSpec::New(primSpec, gTranslate.GetString(), pxr::SdfValueTypeNames->Float3);
            else if (translatePrecision == UsdGeomXformOp::Precision::PrecisionDouble)
                posAttr = pxr::SdfAttributeSpec::New(primSpec, gTranslate.GetString(), pxr::SdfValueTypeNames->Double3);
            else
            {
                CARB_LOG_WARN("Unsupported precision for translate xformOp on prim %s", prim.GetPath().GetString().c_str());
                return false;
            }
        }

        static TfToken gOrient("xformOp:orient");
        attributePath = primPath.AppendProperty(gOrient);
        SdfAttributeSpecHandle orAttr = primSpec->GetAttributeAtPath(attributePath);
        if (!orAttr)
        {
            if (orientPrecision == UsdGeomXformOp::Precision::PrecisionFloat)
                orAttr = pxr::SdfAttributeSpec::New(primSpec, gOrient.GetString(), pxr::SdfValueTypeNames->Quatf);
            else if (orientPrecision == UsdGeomXformOp::Precision::PrecisionDouble)
                orAttr = pxr::SdfAttributeSpec::New(primSpec, gOrient.GetString(), pxr::SdfValueTypeNames->Quatd);
            else
            {
                CARB_LOG_WARN("Unsupported precision for orient xformOp on prim %s", prim.GetPath().GetString().c_str());
                return false;
            }
        }

        static TfToken gScale("xformOp:scale");
        attributePath = primPath.AppendProperty(gScale);
        SdfAttributeSpecHandle scaleAttr = primSpec->GetAttributeAtPath(attributePath);
        if (!scaleAttr)
        {
            scaleAttr = pxr::SdfAttributeSpec::New(primSpec, gScale.GetString(), pxr::SdfValueTypeNames->Float3);
        }

        VtTokenArray xformOpOrder;

        if (resetXfromStack)
            xformOpOrder.push_back(UsdGeomXformOpTypes->resetXformStack);

        if (!extraOps.empty() && preOp)
        {
            for (size_t i = 0; i < extraOps.size(); i++)
            {
                xformOpOrder.push_back(extraOps[i].GetOpName());
            }
        }

        xformOpOrder.push_back(gTranslate);
        xformOpOrder.push_back(gOrient);
        xformOpOrder.push_back(gScale);

        if (translatePrecision == UsdGeomXformOp::Precision::PrecisionFloat)
            posAttr->SetDefaultValue(pxr::VtValue(GfVec3f(localToParentTransform.GetTranslation())));
        else
            posAttr->SetDefaultValue(pxr::VtValue(localToParentTransform.GetTranslation()));
        
        if (orientPrecision == UsdGeomXformOp::Precision::PrecisionFloat)
            orAttr->SetDefaultValue(pxr::VtValue(GfQuatf(localToParentTransform.GetRotation().GetQuat())));
        else
            orAttr->SetDefaultValue(pxr::VtValue(localToParentTransform.GetRotation().GetQuat()));

        if (hasScale)
        {
            if (scalePrecision == UsdGeomXformOp::Precision::PrecisionFloat)
                scaleAttr->SetDefaultValue(pxr::VtValue(GfVec3f(localToParentTransform.GetScale())));
            else if (scalePrecision == UsdGeomXformOp::Precision::PrecisionDouble)
                scaleAttr->SetDefaultValue(pxr::VtValue(localToParentTransform.GetScale()));
            else if (scalePrecision == UsdGeomXformOp::Precision::PrecisionHalf)
                scaleAttr->SetDefaultValue(pxr::VtValue(GfVec3h(localToParentTransform.GetScale())));
        }
        else
        {
            if (scalePrecision == UsdGeomXformOp::Precision::PrecisionFloat)
                scaleAttr->SetDefaultValue(pxr::VtValue(GfVec3f(1.0f)));
            else if (scalePrecision == UsdGeomXformOp::Precision::PrecisionDouble)
                scaleAttr->SetDefaultValue(pxr::VtValue(GfVec3d(1.0)));
            else if (scalePrecision == UsdGeomXformOp::Precision::PrecisionHalf)
                scaleAttr->SetDefaultValue(pxr::VtValue(GfVec3h(1.0)));
        }

        if (!extraOps.empty() && !preOp)
        {
            for (size_t i = 0; i < extraOps.size(); i++)
            {
                xformOpOrder.push_back(extraOps[i].GetOpName());
            }                
        }

        // write new xformOpOrder
        attributePath = primPath.AppendProperty(UsdGeomTokens->xformOpOrder);
        SdfAttributeSpecHandle xformOpAttr = primSpec->GetAttributeAtPath(attributePath);
        if (!xformOpAttr)
        {
            xformOpAttr = pxr::SdfAttributeSpec::New(
                primSpec, UsdGeomTokens->xformOpOrder.GetString(), pxr::SdfValueTypeNames->TokenArray);
        }

        xformOpAttr->SetDefaultValue(VtValue(xformOpOrder));

        ////////////////////////////////////////////////////////////////////////////////
        // Should remove all the attributes that are not :
        //   xformOp:orient, xformOp:scale or oxformOp : translate
        // They are no longer part of the XformOpOrder vector, but we also don't want
        //   them to linger as attributes of the Prim at all so purge them!
        // List the attributes
        //   if xformOp
        //     pass if (xformOp:orient, xformOp:scale or oxformOp : translate)
        //     remove if otherwise
        ////////////////////////////////////////////////////////////////////////////////

        std::vector<UsdAttribute> attribs = prim.GetAttributes();
        const size_t nbrAttribs = attribs.size();
        for (int i = 0; i < nbrAttribs; i++)
        {
            if (UsdGeomXformOp::IsXformOp(attribs[i]))
            {
                const UsdGeomXformOp testOp(attribs[i]);
                const UsdGeomXformOp::Type opType = testOp.GetOpType();
                bool isValidOp = false;
                switch (opType)
                {
                case UsdGeomXformOp::Type::TypeOrient:
                case UsdGeomXformOp::Type::TypeScale:
                case UsdGeomXformOp::Type::TypeTranslate:
                    isValidOp = true;
                    break;
                default:
                    break;
                }
                // Remove the attribute?
                if (!isValidOp && !testOp.HasSuffix(metricsAssemblerSuffixName))
                {
                    const TfToken name = attribs[i].GetName();
                    // TODO : this means that the function is not const for the Prim! Fix.
                    // This ugly hack casts the Prim to non-const. Ay ay ay. Tried to fix it
                    // but creates an avalanche of problems backwards that all rely on const.
                    ((pxr::UsdPrim&)prim).RemoveProperty(name);
                }
            }
        }
    }

    return true;
}

}
}
}
