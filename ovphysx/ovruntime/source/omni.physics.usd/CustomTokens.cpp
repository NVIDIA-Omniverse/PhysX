// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * `TfToken`-typed shim over the source-agnostic custom-token registry.
 *
 * The registry itself lives in the USD-free parse core
 * (`omni/physics/parse/CustomTokens.h`) so both walkers can consult it. This
 * translation unit exists only to keep the `omni::physics::usd` surface — the
 * one the runtime's extension points and `NativeWalker.cpp` are written
 * against — byte-identical in signature and behaviour.
 *
 * @implements REQ-PARSE-CORE-005
 * @covers AC-1 AC-2
 */

#include <omni/physics/usd/CustomTokens.h>

#include <omni/physics/parse/CustomTokens.h>

namespace omni::physics::usd
{

namespace
{
using omni::physics::parse::CustomTokenKind;
}

void registerCustomShapeToken(const PXR_NS::TfToken& token)
{
    parse::registerCustomToken(CustomTokenKind::eShape, token.GetString());
}

void unregisterCustomShapeToken(const PXR_NS::TfToken& token)
{
    parse::unregisterCustomToken(CustomTokenKind::eShape, token.GetString());
}

void registerCustomJointToken(const PXR_NS::TfToken& token)
{
    parse::registerCustomToken(CustomTokenKind::eJoint, token.GetString());
}

void unregisterCustomJointToken(const PXR_NS::TfToken& token)
{
    parse::unregisterCustomToken(CustomTokenKind::eJoint, token.GetString());
}

void registerCustomPhysicsInstancerToken(const PXR_NS::TfToken& token)
{
    parse::registerCustomToken(CustomTokenKind::ePhysicsInstancer, token.GetString());
}

void unregisterCustomPhysicsInstancerToken(const PXR_NS::TfToken& token)
{
    parse::unregisterCustomToken(CustomTokenKind::ePhysicsInstancer, token.GetString());
}

// Internal lookup helpers consumed by `NativeWalker.cpp`, which forward-declares
// them at this namespace's scope rather than including a header.

bool isCustomShapeToken(const PXR_NS::TfToken& token)
{
    return parse::isCustomToken(CustomTokenKind::eShape, token.GetString());
}

bool isCustomJointToken(const PXR_NS::TfToken& token)
{
    return parse::isCustomToken(CustomTokenKind::eJoint, token.GetString());
}

bool isCustomPhysicsInstancerToken(const PXR_NS::TfToken& token)
{
    return parse::isCustomToken(CustomTokenKind::ePhysicsInstancer, token.GetString());
}

} // namespace omni::physics::usd
