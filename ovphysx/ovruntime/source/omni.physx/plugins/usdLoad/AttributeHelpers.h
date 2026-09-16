// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PROPS-CCT-001
 * @covers AC-1
 */

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

// ---------------------------------------------------------------------------
// ObjectKey + attribute-name attribute readers, routed through IPhysicsSource
// (no UsdAttribute / UsdPrim). The value is read via internal::getValue; the
// time-sampled-callback registration uses the attribute path rebuilt from
// pathFor(key) + the attribute name. The clamped form gates on an authored
// value, the plain form resolves a fallback.
// ---------------------------------------------------------------------------

template <typename T>
static bool getAttribute(AttachedStage& attachedStage, T& val, omni::physics::parse::ObjectKey key,
                         omni::physics::parse::TokenId attributeName, T minVal, T maxVal,
                         omni::physics::parse::ReadTime time, usdparser::OnUpdateObjectFn onUpdate)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    bool retVal = false;
    if (src->hasAuthoredAttribute(key, attributeName))
    {
        T attrVal{};
        // A failed read must leave `val` alone -- see the TfToken overload above.
        if (!omni::physx::internal::getValue(attachedStage, key, attributeName, time, attrVal))
            return false;
        if (isinf((float)attrVal))
        {
            if ((float)attrVal < 0)
                attrVal = minVal;
            else
                attrVal = maxVal;
        }
        // Clamped or not, an authored readable value is a successful read -- see
        // the TfToken overload above.
        if (attrVal >= minVal && attrVal <= maxVal)
        {
            val = attrVal;
        }
        else if (attrVal < minVal)
        {
            val = minVal;
        }
        else // attrVal > maxVal
        {
            val = maxVal;
        }
        retVal = true;

        if (onUpdate && src->mightBeTimeVarying(key, attributeName))
        {
            attachedStage.registerTimeSampledAttribute(key, attributeName, onUpdate);
        }
    }
    return retVal;
}

template <typename T>
static void getAttribute(AttachedStage& attachedStage, T& val, omni::physics::parse::ObjectKey key,
                         omni::physics::parse::TokenId attributeName, omni::physics::parse::ReadTime time,
                         usdparser::OnUpdateObjectFn onUpdate)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return;
    // As above: leave `val` at the caller's default when the source cannot answer.
    T readVal{};
    if (omni::physx::internal::getValue(attachedStage, key, attributeName, time, readVal))
        val = readVal;

    if (onUpdate && src->mightBeTimeVarying(key, attributeName))
    {
        attachedStage.registerTimeSampledAttribute(key, attributeName, onUpdate);
    }
}

} // namespace usdparser
} // namespace physx
} // namespace omni
