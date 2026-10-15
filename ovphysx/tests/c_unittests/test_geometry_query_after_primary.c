// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-SDF-001
 * @covers AC-1
 */

/* The geometry-query header composes with the primary header, which does not include it. */
#include "ovphysx/ovphysx.h"
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
    "primary then geometry-query include names ovphysx_evaluate_sdf_shapes");
