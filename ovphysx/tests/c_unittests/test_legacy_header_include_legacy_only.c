// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/* SPDX-License-Identifier: Apache-2.0 */

/**
 * @implements REQ-CAPI-LEGACY-001
 * @covers AC-1 AC-4
 */

#include "ovphysx/ovphysx_legacy.h"

_Static_assert(
    _Generic(
        &ovphysx_raycast,
        ovphysx_result_t (*)(
            ovphysx_instance_handle_t,
            const float*,
            const float*,
            float,
            bool,
            ovphysx_scene_query_mode_t,
            const ovphysx_scene_query_hit_t**,
            uint32_t*): 1,
        default: 0),
    "legacy-only include names ovphysx_raycast");
