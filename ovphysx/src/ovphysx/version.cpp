// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-PROFILER-001
 * @covers AC-5
 */

#include "ovphysx/ovphysx.h"
#include "internal/Profiling.h"
#include "ovphysx/version.h"

void ovphysx_get_version(uint32_t* out_major, uint32_t* out_minor, uint32_t* out_patch)
{
    OVPHYSX_API_PROFILE_ZONE(get_version);
    if (out_major) *out_major = OVPHYSX_VERSION_MAJOR;
    if (out_minor) *out_minor = OVPHYSX_VERSION_MINOR;
    if (out_patch) *out_patch = OVPHYSX_VERSION_PATCH;
}

const char* ovphysx_get_version_string(void)
{
    OVPHYSX_API_PROFILE_ZONE(get_version_string);
    return OVPHYSX_VERSION_STRING;
}
