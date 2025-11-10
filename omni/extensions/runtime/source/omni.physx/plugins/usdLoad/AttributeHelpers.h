// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

namespace omni
{
namespace physx
{
namespace usdparser
{

template <typename T>
static bool getAttribute(T& val, const pxr::UsdAttribute& attribute, T minVal, T maxVal, usdparser::OnUpdateObjectFn onUpdate)
{
    bool retVal = false;
    if (attribute && attribute.HasAuthoredValue())
    {
        T attrVal;
        attribute.Get(&attrVal);
        if (isinf((float)attrVal))
        {
            if ((float)attrVal < 0)
                attrVal = minVal;
            else
                attrVal = maxVal;
        }
        if (attrVal >= minVal && attrVal <= maxVal)
        {
            val = attrVal;
            retVal = true;
        }
        else if (attrVal < minVal)
        {
            val = minVal;
        }
        else // attrVal > maxVal
        {
            val = maxVal;
        }

        // check for time samples
        if (onUpdate && attribute.ValueMightBeTimeVarying())
        {
            UsdLoad::getUsdLoad()->registerTimeSampledAttribute(attribute, onUpdate);
        }
    }
    return retVal;
}

static void getBoolAttribute(bool& val, const pxr::UsdAttribute& attribute, usdparser::OnUpdateObjectFn onUpdate)
{
    if (attribute && attribute.HasAuthoredValue())
    {
        attribute.Get(&val);

        // check for time samples
        if (onUpdate && attribute.ValueMightBeTimeVarying())
        {
            UsdLoad::getUsdLoad()->registerTimeSampledAttribute(attribute, onUpdate);
        }
    }
}

template <typename T>
static void getAttribute(T& val, const pxr::UsdAttribute& attribute, usdparser::OnUpdateObjectFn onUpdate)
{
    if (attribute)
    {
        attribute.Get(&val);

        // check for time samples
        if (onUpdate && attribute.ValueMightBeTimeVarying())
        {
            UsdLoad::getUsdLoad()->registerTimeSampledAttribute(attribute, onUpdate);
        }
    }
}

template <typename T>
static bool getAttribute(
    T& val, const pxr::UsdPrim prim, const pxr::TfToken attrName, T minVal, T maxVal, usdparser::OnUpdateObjectFn onUpdate)
{
    return getAttribute(val, prim.GetAttribute(attrName), minVal, maxVal, onUpdate);
}

static void getBoolAttribute(bool& val,
                             const pxr::UsdPrim prim,
                             const pxr::TfToken attrName,
                             usdparser::OnUpdateObjectFn onUpdate)
{
    getBoolAttribute(val, prim.GetAttribute(attrName), onUpdate);
}

template <typename T>
static void getAttribute(T& val, const pxr::UsdPrim prim, const pxr::TfToken attrName, usdparser::OnUpdateObjectFn onUpdate)
{
    getAttribute(val, prim.GetAttribute(attrName), onUpdate);
}

} // namespace usdparser
} // namespace physx
} // namespace omni
