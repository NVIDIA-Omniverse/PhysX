// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @cond OVPHYSX_PLC_TRACE */
/**
 * @implements REQ-CAPI-JOINT-ARRAYS-001
 * @covers AC-1 AC-2 AC-3 AC-4
 */
/** @endcond */

#ifndef OVPHYSX_UTILS_H
#define OVPHYSX_UTILS_H

#include "ovphysx/ovphysx_export.h"
#include "ovphysx/ovphysx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * @defgroup ovphysx_utils Utilities
     * Conversions of ovphysx output into other layouts, for example the one ovstage stores.
     *
     * Include this header alongside `ovphysx.h`, which does not include it.
     */
    /** @addtogroup ovphysx_utils */
    /** @{ */

    /**
     * An articulation-joint group in ovstage's form: one array per joint.
     *
     * ovphysx reads and writes joint state and joint properties as a fixed group with one row per unlocked
     * DOF. A joint's rows are consecutive, and its prim path repeats once per DOF in the group's prim list.
     * ovstage stores a joint's values as one array per joint prim. A joint-arrays object holds that form
     * without copying anything: one tensor per joint, each a view of that joint's rows in the group's own
     * memory, and a path list naming each joint once.
     *
     * The tensors alias the group, so they are valid only while it is: until the read group is released or
     * the write group is committed. Writing through a tensor of a write group fills that group's column.
     * Create one per fetched group. An object is used from one thread at a time.
     *
     * @code{.c}
     * #include <ovphysx/ovphysx_utils.h>
     *
     * // Waits for an enqueued ovstage operation and releases it; true when it succeeded.
     * static bool wait_op(ovstage_instance_t* stage, ovstage_enqueue_result_t task)
     * {
     *     if (task.status != OVSTAGE_OK)
     *         return false;
     *     if (task.op_index.raw_id == 0)
     *         return true; // nothing left to run
     *     ovstage_op_wait_result_t wait = { 0 };
     *     const bool ok = ovstage_wait_op(stage, task.op_index, OVSTAGE_TIMEOUT_INFINITE, &wait) == OVSTAGE_OK &&
     *                     wait.error_op_id_count == 0;
     *     ovstage_release_op(stage, task.op_index);
     *     return ok;
     * }
     *
     * // Publishes a fetched joint read group as one array per joint prim. The views alias the group, so it stays
     * // unreleased until the write has completed.
     * static bool publish_joints(ovstage_instance_t* stage, const ovstage_read_group_t* group,
     *                            ovx_token_or_string_t attribute, ovstage_ordinal_t ordinal)
     * {
     *     ovphysx_joint_arrays_t* joints = NULL;
     *     if (ovphysx_create_joint_arrays_from_read_group(stage, group, &joints).status != OVPHYSX_API_SUCCESS)
     *         return false; // ovphysx_get_last_error() says why
     *     bool ok = false;
     *     ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
     *     if (ovstage_query_from_path_list(stage, ovphysx_get_joint_arrays_prim_list(joints), &query) == OVSTAGE_OK)
     *     {
     *         ok = wait_op(stage, ovstage_write_attribute(stage, query, attribute, ordinal,
     *                                                     ovphysx_get_joint_arrays_write_data(joints),
     *                                                     OVSTAGE_PRIM_MODE_UPSERT));
     *         ok = wait_op(stage, ovstage_release_query(stage, query)) && ok;
     *     }
     *     ovphysx_destroy_joint_arrays(joints); // after the query, which uses the joints' path list
     *     return ok;
     * }
     * @endcode
     */
    typedef struct ovphysx_joint_arrays_t ovphysx_joint_arrays_t;

    /**
     * @brief Regroup a fetched articulation-joint read group into one array per joint.
     *
     * The group's semantic and CUDA sync carry over to ovphysx_get_joint_arrays_write_data().
     *
     * @param stage The attached stage whose path dictionary holds the group's prim list.
     * @param group A joint read group from ovphysx_fetch_read_next(), not yet released.
     * @param[out] out_arrays Receives the new object, or NULL on failure. Destroy it with
     *   ovphysx_destroy_joint_arrays().
     * @return SUCCESS, or a failure whose message ovphysx_get_last_error() returns on the calling thread:
     *   - OVPHYSX_API_INVALID_ARGUMENT: a null argument, a stage without a path dictionary, a group with
     *     other than one column, a deleted column, or a group not shaped as a joint group (more than one
     *     tensor, an index map or mask, a tensor that is not one-dimensional and contiguous, or a row count
     *     different from its prim count).
     *   - OVPHYSX_API_ERROR: the path dictionary refused the group's prim list or the joint list, or an
     *     allocation failed.
     */
    OVPHYSX_API ovphysx_result_t ovphysx_create_joint_arrays_from_read_group(
        ovstage_instance_t* stage,
        const ovstage_read_group_t* group,
        ovphysx_joint_arrays_t** out_arrays);

    /**
     * @brief Regroup a fetched articulation-joint write group into one array per joint.
     *
     * Writing through the tensors fills the group's column for ovphysx_commit_group(). Errors are those of
     * ovphysx_create_joint_arrays_from_read_group(), without the column and deleted cases.
     *
     * @param stage The attached stage whose path dictionary holds the group's prim list.
     * @param group A joint write group from ovphysx_fetch_write_next(), not yet committed.
     * @param[out] out_arrays Receives the new object, or NULL on failure.
     * @return SUCCESS, or a failure as for ovphysx_create_joint_arrays_from_read_group().
     */
    OVPHYSX_API ovphysx_result_t ovphysx_create_joint_arrays_from_map_group(
        ovstage_instance_t* stage,
        const ovstage_map_group_t* group,
        ovphysx_joint_arrays_t** out_arrays);

    /**
     * @brief Each joint once, in the group's order, for ovstage_query_from_path_list().
     *
     * Owned by @p arrays and released with it. OVX_INVALID_PRIMPATH_LIST for an empty group or a NULL
     * @p arrays.
     */
    OVPHYSX_API ovx_primpath_list_t ovphysx_get_joint_arrays_prim_list(const ovphysx_joint_arrays_t* arrays);

    /**
     * @brief The per-joint views, as an ovstage write payload of an array attribute (`is_array` set).
     *
     * `tensors` holds one tensor per joint, in prim-list order, and `tensor_count` is the number of joints.
     * Each tensor has shape [its DOF count] and the group's element type, lanes and device. The payload
     * carries a read group's semantic and CUDA sync. The tensors are owned by @p arrays; the payload is
     * valid while @p arrays and the group are. All zero for a NULL @p arrays.
     */
    OVPHYSX_API ovstage_write_data_t ovphysx_get_joint_arrays_write_data(const ovphysx_joint_arrays_t* arrays);

    /** @brief Destroy @p arrays and release its path list. NULL is ignored. */
    OVPHYSX_API void ovphysx_destroy_joint_arrays(ovphysx_joint_arrays_t* arrays);

    /** @} */

#ifdef __cplusplus
}
#endif

#endif // OVPHYSX_UTILS_H
