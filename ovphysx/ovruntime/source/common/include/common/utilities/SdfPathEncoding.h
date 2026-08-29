// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

// Header-only SdfPath <-> integer encoding helpers: reinterpret SdfPath's internal 8-byte
// representation (a pointer/refcount to an interned path node) as an integer key and back --
// a fast, allocation-free object key used by the PhysX runtime.

#include <pxr/usd/sdf/path.h>

#include <cstdint>
#include <cstring>

namespace omni
{
namespace physx
{

/// Encode an SdfPath into two 32-bit halves.
inline void encodeSdfPath(const PXR_NS::SdfPath& path, uint32_t& ePart0, uint32_t& ePart1)
{
    uint64_t ui64Path;
    std::memcpy(&ui64Path, &path, sizeof(PXR_NS::SdfPath));
    ePart0 = ui64Path & 0xFFffFFff;
    ePart1 = (ui64Path >> 32);
}

/// Decode an SdfPath previously encoded with encodeSdfPath().
inline PXR_NS::SdfPath decodeSdfPath(const uint32_t ePart0, const uint32_t ePart1)
{
    const uint64_t part0 = uint64_t(ePart0);
    const uint64_t part1 = uint64_t(ePart1);
    const uint64_t uintPath = part0 + (part1 << 32);
    return *(PXR_NS::SdfPath*)&uintPath;
}

/// Reinterpret an SdfPath as a 64-bit integer key.
/// The encoded bits are SdfPath's internal node pointer; the source SdfPath (or another
/// owning SdfPath for the same node) must stay alive while the key is in use.
inline uint64_t sdfPathToInt(const PXR_NS::SdfPath& path)
{
    uint64_t retInt;
    std::memcpy(&retInt, &path, sizeof(PXR_NS::SdfPath));
    return retInt;
}

/// Reinterpret a 64-bit integer key produced by sdfPathToInt() back into an SdfPath.
inline PXR_NS::SdfPath intToSdfPath(const uint64_t intPath)
{
    return *(PXR_NS::SdfPath*)&intPath;
}

} // namespace physx
} // namespace omni
