// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include "ovphysx/ovphysx.h"
#include "ovphysx/version.h"

void ovphysx_get_version(uint32_t* out_major, uint32_t* out_minor, uint32_t* out_patch)
{
    if (out_major) *out_major = OVPHYSX_VERSION_MAJOR;
    if (out_minor) *out_minor = OVPHYSX_VERSION_MINOR;
    if (out_patch) *out_patch = OVPHYSX_VERSION_PATCH;
}

const char* ovphysx_get_version_string(void)
{
    return OVPHYSX_VERSION_STRING;
}
