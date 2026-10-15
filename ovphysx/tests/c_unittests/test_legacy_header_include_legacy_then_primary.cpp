// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-LEGACY-001
 * @covers AC-2 AC-4
 */

#include "ovphysx/ovphysx_legacy.h"
#include "ovphysx/ovphysx.h"

extern "C" void* ovphysx_legacy_include_legacy_then_primary_anchor(void)
{
    return reinterpret_cast<void*>(&ovphysx_create_contact_binding);
}
