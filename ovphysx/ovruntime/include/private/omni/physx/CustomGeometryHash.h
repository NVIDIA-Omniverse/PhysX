// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-SHAPE-002
 * @covers AC-4
 */

#pragma once

#include <omni/physics/parse/CustomTokens.h>

#include <cstddef>
#include <string>

namespace omni
{
namespace physx
{

// Stable content hash for custom-geometry token names.  Used as the
// `PhysXCustomGeometryManager::mCustomGeometryTypeMap` key, and folded into
// the debug-viz mesh-cache MeshKey for `eCustomShape` shapes, so
// registration and consumer sides agree without depending on TfToken's
// interned-pointer hash (which is process-stable but harder to reproduce
// from a USD-free parse-lib path).
//
// The implementation lives in the USD-free parse core
// (`omni::physics::parse::customGeometryTokenHash`) because the ovstage walker
// mints the same hash and cannot name `TfToken`. This is a spelling
// convenience over it, not a second implementation. The TfToken overload
// this file used to carry is gone under ADR-0019: `IPhysxCustomGeometry`'s
// public surface no longer names `TfToken` (or `SdfPath`), so no caller here
// holds one anymore.
inline size_t computeCustomGeometryHash(const std::string& tokenStr)
{
    return omni::physics::parse::customGeometryTokenHash(tokenStr);
}

} // namespace physx
} // namespace omni
