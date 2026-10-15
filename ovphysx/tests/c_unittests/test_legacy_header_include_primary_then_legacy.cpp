// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-LEGACY-001
 * @covers AC-2 AC-4
 */

#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_legacy.h"

extern "C" void* ovphysx_legacy_include_primary_then_legacy_anchor(void)
{
    return reinterpret_cast<void*>(&ovphysx_get_object_type);
}
