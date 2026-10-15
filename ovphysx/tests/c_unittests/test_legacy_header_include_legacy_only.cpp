// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-LEGACY-001
 * @covers AC-1 AC-4
 */

#include "ovphysx/ovphysx_legacy.h"

extern "C" void* ovphysx_legacy_include_legacy_only_anchor(void)
{
    return reinterpret_cast<void*>(&ovphysx_raycast);
}
