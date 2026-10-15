// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-SDF-001
 * @covers AC-1
 */

/* The geometry-query header compiles on its own: it brings the types its declarations use. */
#include "ovphysx/ovphysx_geometry_query.h"

_Static_assert(
    _Generic(
        &ovphysx_evaluate_sdf_shapes,
        ovphysx_enqueue_result_t (*)(
            ovphysx_instance_handle_t,
            ovphysx_query_handle_t,
            const DLTensor*,
            DLTensor*,
            ovstage_cuda_sync_t): 1,
        default: 0),
    "geometry-query-only include names ovphysx_evaluate_sdf_shapes");
_Static_assert(OVPHYSX_QUERY_TARGET_SDF_SHAPE == 24,
               "geometry-query-only include names OVPHYSX_QUERY_TARGET_SDF_SHAPE");
