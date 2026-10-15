// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/* SPDX-License-Identifier: Apache-2.0 */

/**
 * @implements REQ-CAPI-LEGACY-001
 * @covers AC-3 AC-4
 */

#include "ovphysx/ovphysx.h"

_Static_assert(
    _Generic(
        &ovphysx_create_tensor_binding,
        ovphysx_result_t (*)(
            ovphysx_instance_handle_t,
            const ovphysx_tensor_binding_desc_t*,
            ovphysx_tensor_binding_handle_t*): 1,
        default: 0),
    "umbrella include exposes ovphysx_create_tensor_binding");
