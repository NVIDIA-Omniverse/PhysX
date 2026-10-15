// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/* SPDX-License-Identifier: Apache-2.0 */

/**
 * @implements REQ-CAPI-LEGACY-001
 * @covers AC-2 AC-4
 */

#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_legacy.h"

_Static_assert(
    _Generic(
        &ovphysx_get_object_type,
        ovphysx_result_t (*)(
            ovphysx_instance_handle_t,
            ovphysx_string_t,
            ovphysx_object_type_t*): 1,
        default: 0),
    "primary-then-legacy include names ovphysx_get_object_type");
