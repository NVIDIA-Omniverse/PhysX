// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-LEGACY-001
 * @covers AC-3 AC-4
 */

#include "ovphysx/ovphysx.h"

extern "C" void* ovphysx_legacy_include_primary_only_anchor(void)
{
    return reinterpret_cast<void*>(&ovphysx_create_tensor_binding);
}
