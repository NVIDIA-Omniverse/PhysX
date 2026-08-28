// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

// PhysXTools.h provides the source-backed internal::getValue + the complete
// IPhysicsSource definition the ObjectKey/attribute-name overloads below need
// (some includers of this header don't pull PhysXTools.h in themselves).
#include <PhysXTools.h>

namespace omni
{
namespace physx
{
namespace usdparser
{

template <typename T>
static bool getAttribute(AttachedStage& attachedStage, T& val, const PXR_NS::UsdAttribute& attribute, T minVal, T maxVal, usdparser::OnUpdateObjectFn onUpdate)
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

        if (onUpdate && attribute.ValueMightBeTimeVarying())
        {
            attachedStage.registerTimeSampledAttribute(attribute.GetPath(), onUpdate);
        }
    }
    return retVal;
}

template <typename T>
static void getAttribute(AttachedStage& attachedStage, T& val, const PXR_NS::UsdAttribute& attribute, usdparser::OnUpdateObjectFn onUpdate)
{
    if (attribute)
    {
        attribute.Get(&val);

        if (onUpdate && attribute.ValueMightBeTimeVarying())
        {
            attachedStage.registerTimeSampledAttribute(attribute.GetPath(), onUpdate);
        }
    }
}

// ---------------------------------------------------------------------------
// ObjectKey + attribute-name overloads — source-routed counterparts of the
// UsdAttribute forms above (no UsdAttribute / UsdPrim). The value is read
// through IPhysicsSource (internal::getValue); the time-sampled-callback
// registration uses the attribute path rebuilt from pathFor(key) + the
// attribute name. Behaviour mirrors the UsdAttribute overloads: the clamped
// form gates on an authored value, the plain form resolves a fallback.
// ---------------------------------------------------------------------------

template <typename T>
static bool getAttribute(AttachedStage& attachedStage, T& val, omni::physics::parse::ObjectKey key,
                         const PXR_NS::TfToken& attributeName, T minVal, T maxVal, usdparser::OnUpdateObjectFn onUpdate)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const omni::physics::parse::TokenId tok = src->internToken(attributeName.GetString());
    bool retVal = false;
    if (src->hasAuthoredAttribute(key, tok))
    {
        T attrVal{};
        omni::physx::internal::getValue(attachedStage, key, attributeName, PXR_NS::UsdTimeCode::Default(), attrVal);
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

        if (onUpdate && src->mightBeTimeVarying(key, tok))
        {
            attachedStage.registerTimeSampledAttribute(attachedStage.pathFor(key).AppendProperty(attributeName), onUpdate);
        }
    }
    return retVal;
}

template <typename T>
static void getAttribute(AttachedStage& attachedStage, T& val, omni::physics::parse::ObjectKey key,
                         const PXR_NS::TfToken& attributeName, usdparser::OnUpdateObjectFn onUpdate)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return;
    omni::physx::internal::getValue(attachedStage, key, attributeName, PXR_NS::UsdTimeCode::Default(), val);

    const omni::physics::parse::TokenId tok = src->internToken(attributeName.GetString());
    if (onUpdate && src->mightBeTimeVarying(key, tok))
    {
        attachedStage.registerTimeSampledAttribute(attachedStage.pathFor(key).AppendProperty(attributeName), onUpdate);
    }
}

} // namespace usdparser
} // namespace physx
} // namespace omni
