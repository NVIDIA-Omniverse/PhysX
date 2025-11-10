// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include <carb/logging/Log.h>

#include "InternalXformOpResetStorage.h"

using namespace omni::physx::internal;
using namespace pxr;
using namespace carb;

void XformOpResetStorage::store(const pxr::UsdGeomXformable & xformable)
{
    const std::vector<UsdGeomXformOp> xformOps = xformable.GetOrderedXformOps(&mResetXfromStack);
    resetToSize(xformOps.size());

    const SdfPath primPath = xformable.GetPrim().GetPrimPath();

    mStartEditTarget = xformable.GetPrim().GetStage()->GetEditTarget();
    const SdfLayerHandle layer = mStartEditTarget.GetLayer();
    mXformStackWritten = layer->GetAttributeAtPath(xformable.GetXformOpOrderAttr().GetPath()) == nullptr ? false : true;

    mTranslateOpWritten = false;
    mOrientOpWritten = false;
    mScaleOpWritten = false;

    for (const UsdGeomXformOp& op : xformOps)
    {
        VtValue value;
        op.GetAttr().Get(&value);  // preist: Is it guaranteed that GetAttr() is valid always?
        mOpType.push_back(op.GetOpType());
        mPrecision.push_back(op.GetPrecision());
        mValue.push_back(value);
        mName.push_back(op.GetAttr().GetName());
        std::vector<std::string> substrings = op.SplitName();
        bool inverted = false;
        const bool opWritten = layer->GetAttributeAtPath(op.GetAttr().GetPath()) == nullptr ? false : true;
        mOpWritten.push_back(opWritten);
        std::string suffix = "";
        if (!substrings.empty())
        {
            if (substrings[0] == "!invert!")
            {
                inverted = true;
                if (substrings.size() > 3)
                {
                    suffix = substrings[3];
                }
            }
            else if (substrings.size() > 2)
            {
                suffix = substrings[2];
            }
        }
        if (!inverted)
        {
            inverted = op.IsInverseOp();
        }
        mInvertOp.push_back(inverted);
        mSuffix.push_back(suffix == "" ? TfToken() : TfToken(suffix));

        if (op.GetOpType() == UsdGeomXformOp::Type::TypeTranslate)
        {
            mTranslateOpWritten = opWritten;
        }
        else if (op.GetOpType() == UsdGeomXformOp::Type::TypeOrient)
        {
            mOrientOpWritten = opWritten;
        }
        else if (op.GetOpType() == UsdGeomXformOp::Type::TypeScale)
        {
            mScaleOpWritten = opWritten;
        }
    }
}

void XformOpResetStorage::restore(pxr::UsdGeomXformable & xformable, const bool purgeOrphanedTranslateOrientScaleVal) const
{
    if (mStartEditTarget != xformable.GetPrim().GetStage()->GetEditTarget())
    {
        CARB_LOG_WARN_ONCE("Edit target changed between simulation start and simulation end! The transformation reset will write into possibly incorrect layer.");
    }

    bool opOrderRemoved = true;
    if (mXformStackWritten)
    {
        xformable.ClearXformOpOrder();
        xformable.SetResetXformStack(mResetXfromStack);
    }
    else
    {
        opOrderRemoved = xformable.GetPrim().RemoveProperty(UsdGeomTokens->xformOpOrder);
    }

    // Very special case1, if the xformOp stack was right and is inside a reference we cant remove the opOrder
    // but then we cant write new ops, we can just update the ops
    // Case2
    // OrderWas removed, but is still valid in the reference, since ops were written, we need to set values
    const bool setValues = !opOrderRemoved || (!mXformStackWritten && opOrderRemoved);
    for (size_t x = 0; x < mOpType.size(); x++)
    {
        if (mOpWritten[x] || mXformStackWritten)
        {            
            if (setValues)
            {
                UsdAttribute attr = xformable.GetPrim().GetAttribute(mName[x]);
                if (attr)
                {
                    attr.Set(mValue[x]);
                }
            }
            else
            {
                UsdGeomXformOp op = xformable.AddXformOp(mOpType[x], mPrecision[x], mSuffix[x], mInvertOp[x]);
                if (op && !mInvertOp[x])
                {
                    op.GetAttr().Set(mValue[x]);
                }
            }
        }       
    }
    if (purgeOrphanedTranslateOrientScaleVal)
        purgeOrphanedTranslateOrientScale(xformable);
}

void XformOpResetStorage::resetToSize(const size_t size)
{
    mOpType.clear();
    mPrecision.clear();
    mValue.clear();
    mOpType.reserve(size);
    mPrecision.reserve(size);
    mValue.reserve(size);
}

void XformOpResetStorage::purgeOrphanedTranslateOrientScale(pxr::UsdGeomXformable & xformable) const
{
    static const pxr::TfToken xformOpTranslate = pxr::TfToken("xformOp:translate");
    static const pxr::TfToken xformOpOrient = pxr::TfToken("xformOp:orient");
    static const pxr::TfToken xformOpScale = pxr::TfToken("xformOp:scale");

    if (!mTranslateOpWritten)
    {
        xformable.GetPrim().RemoveProperty(xformOpTranslate);
    }
    if (!mOrientOpWritten)
    {
        xformable.GetPrim().RemoveProperty(xformOpOrient);
    }
    if (!mScaleOpWritten)
    {
        xformable.GetPrim().RemoveProperty(xformOpScale);
    }
}
