// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-SHAPE-002
 * @covers AC-4
 */

#pragma once

#include <carb/extras/Hash.h>

#include <pxr/base/tf/token.h>

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
// Uses `carb::extras::fnv128hash` (the same hash family the cooking service
// uses internally via MeshKey::compute*Hash) and XOR-reduces to 64-bit,
// matching the `MeshKey::getHashIndex` reduction.
inline size_t computeCustomGeometryHash(const std::string& tokenStr)
{
    const auto h = carb::extras::fnv128hash(
        reinterpret_cast<const uint8_t*>(tokenStr.data()), tokenStr.size());
    return static_cast<size_t>(h.d[0] ^ h.d[1]);
}

inline size_t computeCustomGeometryHash(const PXR_NS::TfToken& token)
{
    return computeCustomGeometryHash(token.GetString());
}

} // namespace physx
} // namespace omni
