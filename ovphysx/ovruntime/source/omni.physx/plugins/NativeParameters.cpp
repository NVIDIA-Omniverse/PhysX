// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CUSTOM-PARAMETERS-001
// @covers AC-1 AC-2 AC-3
#include <omni/physx/NativeParameters.h>
#include <omni/physx/PhysXRuntime.h>
#include "RuntimeExecution.h"
#include "usdLoad/AttachedStage.h"
#include <cmath>
#include <vector>

namespace omni::physx::native
{
namespace {
bool valid(const ParameterValue& value)
{
    if (value.reserved) return false;
    switch (value.kind)
    {
    case ParameterKind::Float: return std::isfinite(value.scalar);
    case ParameterKind::Double: return std::isfinite(value.real);
    case ParameterKind::Int: return true;
    case ParameterKind::Bool: return value.boolean <= 1;
    case ParameterKind::Float3:
        return std::isfinite(value.vector[0]) && std::isfinite(value.vector[1]) && std::isfinite(value.vector[2]);
    }
    return false;
}
}
bool copyParameters(uint64_t attachment, uint64_t objectKey, const ParameterSpec* specs,
    uint32_t parameterCount, ParameterValue* output, uint32_t& invalidParameter) noexcept
try
{
    invalidParameter = UINT32_MAX;
    const omni::physx::usdparser::AttachedStage * owner = customCreationOwner();
    if (!runtime::tryGetPhysxInterface() || !attachment || !objectKey ||
        (parameterCount && (!specs || !output)) || !owner || owner->getAttachHandle() != attachment) return false;
    if (!owner || !owner->getSource()) return false;
    const omni::physics::parse::IPhysicsSource * source = owner->getSource();
    const omni::physics::parse::ObjectKey key{objectKey};
    if (!source->exists(key)) return false;
    std::vector<ParameterValue> values;
    values.reserve(parameterCount);
    for (uint32_t i=0; i<parameterCount; ++i)
    {
        invalidParameter = i;
        const omni::physx::native::ParameterSpec & spec = specs[i];
        if (!spec.name || !spec.name[0] || spec.required > 1 ||
            spec.fallback.kind != spec.kind || !valid(spec.fallback)) return false;
        const omni::physics::parse::AttrValue attr = source->getAttribute(key, source->internToken(spec.name));
        ParameterValue value{};
        value.kind = spec.kind;
        using Kind = omni::physics::parse::AttrValue::Kind;
        if (!attr.valid())
        {
            if (spec.required) return false;
            value = spec.fallback;
        }
        else switch (spec.kind)
        {
        case ParameterKind::Float:
            if (attr.kind != Kind::eFloat) return false;
            value.scalar = attr.f; break;
        case ParameterKind::Double:
            if (attr.kind != Kind::eDouble) return false;
            value.real = attr.d; break;
        case ParameterKind::Int:
            if (attr.kind != Kind::eInt) return false;
            value.integer = attr.i; break;
        case ParameterKind::Bool:
            if (attr.kind != Kind::eBool) return false;
            value.boolean = attr.b ? 1u : 0u; break;
        case ParameterKind::Float3:
            if (attr.kind != Kind::eFloat3) return false;
            value.vector[0] = attr.f3.x; value.vector[1] = attr.f3.y; value.vector[2] = attr.f3.z; break;
        default: return false;
        }
        if (!valid(value)) return false;
        values.push_back(value);
    }
    for (uint32_t i=0; i<parameterCount; ++i) output[i]=values[i];
    invalidParameter = UINT32_MAX;
    return true;
}
catch (...)
{
    invalidParameter = UINT32_MAX;
    return false;
}
}
