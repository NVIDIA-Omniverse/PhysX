// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-JOINT-ARRAYS-001
 * @covers AC-1
 */

/* The utilities header composes with the primary header, which does not include it. */
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_utils.h"

_Static_assert(
    _Generic(
        &ovphysx_create_joint_arrays_from_read_group,
        ovphysx_result_t (*)(ovstage_instance_t*, const ovstage_read_group_t*, ovphysx_joint_arrays_t**): 1,
        default: 0),
    "primary then utils include names ovphysx_create_joint_arrays_from_read_group");
_Static_assert(
    _Generic(
        &ovphysx_create_joint_arrays_from_map_group,
        ovphysx_result_t (*)(ovstage_instance_t*, const ovstage_map_group_t*, ovphysx_joint_arrays_t**): 1,
        default: 0),
    "primary then utils include names ovphysx_create_joint_arrays_from_map_group");
_Static_assert(
    _Generic(
        &ovphysx_get_joint_arrays_prim_list, ovx_primpath_list_t (*)(const ovphysx_joint_arrays_t*): 1, default: 0),
    "primary then utils include names ovphysx_get_joint_arrays_prim_list");
_Static_assert(
    _Generic(
        &ovphysx_get_joint_arrays_write_data, ovstage_write_data_t (*)(const ovphysx_joint_arrays_t*): 1, default: 0),
    "primary then utils include names ovphysx_get_joint_arrays_write_data");
_Static_assert(_Generic(&ovphysx_destroy_joint_arrays, void (*)(ovphysx_joint_arrays_t*): 1, default: 0),
               "primary then utils include names ovphysx_destroy_joint_arrays");
