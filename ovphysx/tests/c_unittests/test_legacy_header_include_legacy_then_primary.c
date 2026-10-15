// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/* SPDX-License-Identifier: Apache-2.0 */

/**
 * @implements REQ-CAPI-LEGACY-001
 * @covers AC-2 AC-4
 */

#include "ovphysx/ovphysx_legacy.h"
#include "ovphysx/ovphysx.h"

_Static_assert(
    _Generic(
        &ovphysx_create_contact_binding,
        ovphysx_result_t (*)(
            ovphysx_instance_handle_t,
            const ovphysx_string_t*,
            uint32_t,
            const ovphysx_string_t*,
            uint32_t,
            uint32_t,
            ovphysx_contact_binding_handle_t*): 1,
        default: 0),
    "legacy-then-primary include names ovphysx_create_contact_binding");
