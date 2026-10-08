// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-HANDLE-001
 * @covers AC-3
 */

#pragma once

#include <pxr/usd/sdf/path.h>

#include <cstdint>
#include <cstring>

// Legacy SdfPath<->uint64 bit encoders (relocated from common/utilities/Utilities.h, ADR-0027).
// Only the USD library (IUsdReparse::legacyPathBitsToString/stringToLegacyPathBits) and
// USD-linked tests use them; OvruntimePhysX never does (REQ-PUBLICAPI-HANDLE-001).

// asInt() is the same as SdfPath::_AsInt()
// The path->ObjectId encoding relies on asInt(a)==asInt(b) <=> a is same path as b,
// which is how SdfPath::operator== is currently defined.
// If USD changes sizeof(PXR_NS::SdfPath), we will need to change

inline uint64_t asInt(const PXR_NS::SdfPath& path)
{
    static_assert(sizeof(PXR_NS::SdfPath) == sizeof(uint64_t), "Change to make the same size as PXR_NS::SdfPath");
    uint64_t ret;
    std::memcpy(&ret, &path, sizeof(PXR_NS::SdfPath));

    return ret;
}

inline const PXR_NS::SdfPath& intToPath(const uint64_t& path)
{
    static_assert(sizeof(PXR_NS::SdfPath) == sizeof(uint64_t), "Change to make the same size as PXR_NS::SdfPath");

    return reinterpret_cast<const PXR_NS::SdfPath&>(path);
}
