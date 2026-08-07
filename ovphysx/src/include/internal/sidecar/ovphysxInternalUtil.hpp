// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <pxr/usd/sdf/path.h>

#include <cstdint>
#include <cstring>

// Sidecar-internal C++ helpers shared across subsystem translation units.
// Not exported (no OVPHYSX_INTERNAL_API) — these are inline and only used by
// the sidecar's own .cpp files.

namespace ovphysx::internal {

// Bit-reinterpret SdfPath as uint64_t. SdfPath is a single 8-byte handle into
// the SdfPath token pool, so the raw bits round-trip safely. This is the
// encoding IPhysxSceneQuery hit results use, so callers can pass our
// uint64_t back into PhysX query APIs without translation.
inline uint64_t sdfPathToInt(const PXR_NS::SdfPath& path) {
    static_assert(sizeof(PXR_NS::SdfPath) == sizeof(uint64_t),
                  "SdfPath must be 8 bytes for path encoding to work");
    uint64_t retInt;
    std::memcpy(&retInt, &path, sizeof(PXR_NS::SdfPath));
    return retInt;
}

} // namespace ovphysx::internal
