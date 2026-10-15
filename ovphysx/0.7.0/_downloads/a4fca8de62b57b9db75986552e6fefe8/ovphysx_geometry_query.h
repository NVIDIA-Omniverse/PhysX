// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @cond OVPHYSX_PLC_TRACE */
/**
 * @implements REQ-CAPI-SDF-001
 * @covers AC-1 AC-2
 */
/** @endcond */

#ifndef OVPHYSX_GEOMETRY_QUERY_H
#define OVPHYSX_GEOMETRY_QUERY_H

#include "ovphysx/ovphysx_export.h"
#include "ovphysx/ovphysx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * @defgroup ovphysx_geometry_query Geometry queries
     * Questions asked of the scene's collision geometry at caller-supplied inputs, through a session query.
     *
     * Include this header alongside `ovphysx.h`, which does not include it.
     */

    /** @addtogroup ovphysx_geometry_query */
    /** @{ */

    /**
     * @brief Evaluate the signed distance fields of the SDF shapes a query selects.
     *
     * The successor of the deprecated SDF view (ovphysx_create_sdf_view / ovphysx_evaluate_sdf), on the
     * session query. Open the query with type OVPHYSX_QUERY_TARGET_SDF_SHAPE: its rows are the collision shapes
     * the SDF view accepts -- a prim carrying PhysicsCollisionAPI and PhysxSDFMeshCollisionAPI whose shape is
     * a triangle mesh with an SDF, runtime clones included. A query from @ref ovphysx_query_selection orders
     * its rows as its selection: explicit prim handles in the order given, a pattern in the same matcher
     * order the SDF view used; a whole-set query orders them as the runtime stores them. A selected prim
     * that is not an SDF shape is skipped, with a warning, as the SDF view skipped it.
     *
     * For each row i and point j, @p out[i][j] receives the gradient (x, y, z) and then the signed distance
     * at @p points[i][j], negative inside. Points and gradients are in the shape's mesh-local space -- the
     * unscaled frame of the mesh's vertices -- and no transform is applied. Q, the points per shape, is read
     * from the tensors on every call. A point outside the shape's SDF grid (the mesh bounds padded by the SDF
     * margin) is evaluated, not refused, as by the SDF view: its distance is the SDF value at the nearest grid
     * point plus the distance to the grid, a positive upper bound, and its gradient points away from the shape.
     *
     * Evaluation needs a DirectGPU scene, all the rows in one scene, at most 65535 rows -- PhysX launches its
     * SDF evaluation with one CUDA grid row per shape -- and at most 2^31 - 1 points in all (N x Q), which
     * PhysX indexes in 32-bit arithmetic. It also needs the scene to have simulated since its physics objects
     * were last created or released, because PhysX uploads shape data to the GPU only when it simulates. On
     * the first evaluation after loading USD, an automatic warmup simulation step may be performed, as for the
     * SDF view; after that the call never steps, so step once after such a change before evaluating. @p out
     * is complete once the task has completed.
     *
     * ## Synchronization
     *
     * @p points_ready is ovstage's `{stream, wait_event}` handoff of the producer of @p points, as
     * @ref ovphysx_commit_group takes it: a non-zero stream is drained and a non-zero event waited on before
     * the points are read. Only `{0, 0}` asserts that nothing is outstanding. Stream 1 names the legacy
     * default stream of the CUDA context current at this call. Keep a supplied stream and event alive until
     * the task completes. A handoff that cannot be honored fails the task with nothing evaluated.
     *
     * @p out must have no outstanding device reads or writes when the evaluation starts, or those accesses
     * must be covered by @p points_ready: work queued on its stream, or before its event, has finished before
     * anything is written. Keep @p out exclusively available to this call -- no other reader or writer --
     * until the task completes.
     *
     * @param handle Physics instance handle.
     * @param query  Live query owned by @p handle, with type OVPHYSX_QUERY_TARGET_SDF_SHAPE. Its rows are
     *               evaluated when the task executes.
     * @param points float32 [N, Q, 3], C-contiguous, on the scene's CUDA device; N is the query's row count.
     * @param out    float32 [N, Q, 4], C-contiguous, on the same device; written in place. Exclusively the
     *               call's until the task completes (see Synchronization).
     * @param points_ready Producer handoff for @p points; `{0, 0}` when nothing is outstanding.
     * @return Admission receipt.
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT, before a task is created: a null tensor or data pointer, a tensor
     *   that is not float32 [N, Q, 3] / [N, Q, 4] and C-contiguous on a CUDA device, tensors that disagree
     *   on N, Q or device, Q of 0, N above 65535, or N x Q above 2^31 - 1.
     * - OVPHYSX_API_NOT_FOUND, before a task is created: a query this instance does not own.
     * - OVPHYSX_API_ERROR reported by waiting for the task: a query of another type or with no SDF shape,
     *   N different from its row count, rows outside one DirectGPU scene, a scene that has not simulated
     *   since its physics objects last changed, tensors on a CUDA device other than the scene's, a query kept
     *   across a stage detach, a handoff that cannot be honored, or a failed evaluation.
     */
    OVPHYSX_API ovphysx_enqueue_result_t ovphysx_evaluate_sdf_shapes(
        ovphysx_instance_handle_t handle,
        ovphysx_query_handle_t query,
        const DLTensor* points,
        DLTensor* out,
        ovstage_cuda_sync_t points_ready);

    /** @} */

#ifdef __cplusplus
}
#endif

#endif // OVPHYSX_GEOMETRY_QUERY_H
