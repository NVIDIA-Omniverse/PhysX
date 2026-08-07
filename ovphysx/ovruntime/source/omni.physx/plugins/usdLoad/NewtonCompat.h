// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//


// Newton → PhysX schema compatibility helpers.
//
// Newton USD schemas define physics attributes with Newton naming/semantics.
// When a prim has both Newton and PhysX schemas applied, PhysX authored values
// take priority.  These helpers read Newton attributes as fallbacks when the
// corresponding PhysX attribute was not authored.
//
// Priority (highest → lowest):
//   1. PhysX Schema Value authored
//   2. Newton Schema Value authored
//   3. PhysX Default
//   4. Newton Default

#pragma once

#include "NewtonSchemaTokens.h"
#include <pxr/pxr.h>

PXR_NAMESPACE_OPEN_SCOPE
TF_DECLARE_PUBLIC_TOKENS(NewtonSchemaTokens, NEWTON_SCHEMA_TOKENS);
PXR_NAMESPACE_CLOSE_SCOPE

namespace omni
{
namespace physx
{
namespace newton
{

// Read a bounded Newton attribute as fallback for a PhysX attribute that was
// not authored.  Returns true if a Newton value was applied.
template <typename T>
static bool getFallback(T& val, const PXR_NS::UsdPrim& prim, const PXR_NS::TfToken& newtonToken, T minVal, T maxVal)
{
    PXR_NS::UsdAttribute attr = prim.GetAttribute(newtonToken);
    if (attr && attr.HasAuthoredValue())
    {
        T attrVal;
        attr.Get(&attrVal);
        if (attrVal >= minVal && attrVal <= maxVal)
        {
            val = attrVal;
            return true;
        }
        else if (attrVal < minVal)
        {
            val = minVal;
            return true;
        }
        else
        {
            val = maxVal;
            return true;
        }
    }
    return false;
}

// Read a Newton boolean attribute as fallback.  Returns true if a Newton value was applied.
static bool getBoolFallback(bool& val, const PXR_NS::UsdPrim& prim, const PXR_NS::TfToken& newtonToken)
{
    PXR_NS::UsdAttribute attr = prim.GetAttribute(newtonToken);
    if (attr && attr.HasAuthoredValue())
    {
        attr.Get(&val);
        return true;
    }
    return false;
}

// Read a Newton float attribute without bounds checking (for special-case handling).
// Returns true if a Newton value was authored.
static bool getFloatFallback(float& val, const PXR_NS::UsdPrim& prim, const PXR_NS::TfToken& newtonToken)
{
    PXR_NS::UsdAttribute attr = prim.GetAttribute(newtonToken);
    if (attr && attr.HasAuthoredValue())
    {
        attr.Get(&val);
        return true;
    }
    return false;
}

// Read a Newton int attribute as a uint32_t fallback (for hull vertex limits etc.).
// Ignores the Newton value if it is -1 (Newton's "use default" sentinel).
// Returns true if a Newton value was applied.
static bool getIntAsUint32Fallback(uint32_t& val, const PXR_NS::UsdPrim& prim, const PXR_NS::TfToken& newtonToken)
{
    PXR_NS::UsdAttribute attr = prim.GetAttribute(newtonToken);
    if (attr && attr.HasAuthoredValue())
    {
        int intVal;
        attr.Get(&intVal);
        if (intVal > 0)
        {
            val = static_cast<uint32_t>(intVal);
            return true;
        }
    }
    return false;
}

} // namespace newton
} // namespace physx
} // namespace omni
