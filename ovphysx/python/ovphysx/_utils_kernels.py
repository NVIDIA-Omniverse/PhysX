# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-FRAME-001
# @covers AC-3

"""Warp kernels used by :mod:`ovphysx.utils`."""

import warp as wp


@wp.func
def _matrix_scale(matrices: wp.array2d(dtype=wp.float64), row: int):
    if not wp.isfinite(matrices[row, 12]) or not wp.isfinite(matrices[row, 13]) or not wp.isfinite(matrices[row, 14]):
        return wp.vec3d(1.0, 1.0, 1.0)

    m0 = wp.vec3d(matrices[row, 0], matrices[row, 1], matrices[row, 2])
    m1 = wp.vec3d(matrices[row, 4], matrices[row, 5], matrices[row, 6])
    m2 = wp.vec3d(matrices[row, 8], matrices[row, 9], matrices[row, 10])

    r0 = m0
    length = wp.length(r0)
    if length > 1.0e-12 and wp.isfinite(length):
        r0 = r0 / length
    else:
        r0 = wp.vec3d(1.0, 0.0, 0.0)

    r1 = m1 - wp.dot(r0, m1) * r0
    length = wp.length(r1)
    if length > 1.0e-12 and wp.isfinite(length):
        r1 = r1 / length
    else:
        r1 = wp.vec3d(0.0, 1.0, 0.0)

    r2 = m2 - wp.dot(r0, m2) * r0 - wp.dot(r1, m2) * r1
    length = wp.length(r2)
    if length > 1.0e-12 and wp.isfinite(length):
        r2 = r2 / length
    else:
        r2 = wp.vec3d(0.0, 0.0, 1.0)

    # Match omni::physx::decomposeMatrix: refine the orthogonal basis with
    # Newton polar iteration before projecting the original basis onto it.
    for _iteration in range(16):
        determinant = wp.dot(r0, wp.cross(r1, r2))
        if not wp.isfinite(determinant) or wp.abs(determinant) < 1.0e-20:
            break
        inverse_determinant = wp.float64(1.0) / determinant
        inverse0 = wp.cross(r1, r2) * inverse_determinant
        inverse1 = wp.cross(r2, r0) * inverse_determinant
        inverse2 = wp.cross(r0, r1) * inverse_determinant
        next0 = wp.float64(0.5) * (r0 + inverse0)
        next1 = wp.float64(0.5) * (r1 + inverse1)
        next2 = wp.float64(0.5) * (r2 + inverse2)
        delta0 = wp.abs(r0 - next0)
        delta1 = wp.abs(r1 - next1)
        delta2 = wp.abs(r2 - next2)
        max_delta = wp.max(
            wp.max(wp.max(delta0[0], delta0[1]), delta0[2]),
            wp.max(
                wp.max(wp.max(delta1[0], delta1[1]), delta1[2]),
                wp.max(wp.max(delta2[0], delta2[1]), delta2[2]),
            ),
        )
        r0 = next0
        r1 = next1
        r2 = next2
        if max_delta < 1.0e-12:
            break

    sx = wp.float32(wp.dot(r0, m0))
    sy = wp.float32(wp.dot(r1, m1))
    sz = wp.float32(wp.dot(r2, m2))
    if wp.dot(r0, wp.cross(r1, r2)) < 0.0:
        sx = -sx
        sy = -sy
        sz = -sz
    if not wp.isfinite(sx):
        sx = 1.0
    if not wp.isfinite(sy):
        sy = 1.0
    if not wp.isfinite(sz):
        sz = 1.0
    return wp.vec3d(wp.float64(sx), wp.float64(sy), wp.float64(sz))


@wp.func
def _compose_world_xform(position: wp.vec3f, orientation: wp.quatf, scale: wp.vec3d):
    position64 = wp.vec3d(wp.float64(position[0]), wp.float64(position[1]), wp.float64(position[2]))
    orientation64 = wp.normalize(
        wp.quatd(
            wp.float64(orientation[0]),
            wp.float64(orientation[1]),
            wp.float64(orientation[2]),
            wp.float64(orientation[3]),
        )
    )
    # Warp uses column vectors; OVStage uses USD's row-vector convention.
    return wp.transpose(wp.transform_compose(position64, orientation64, scale))


@wp.kernel
def extract_world_scales(
    matrices: wp.array2d(dtype=wp.float64),
    source_rows: wp.array(dtype=wp.int32),
    destination_rows: wp.array(dtype=wp.int32),
    scales: wp.array2d(dtype=wp.float64),
):
    row = wp.tid()
    source_row = source_rows[row]
    destination_row = destination_rows[row]
    scale = _matrix_scale(matrices, source_row)
    scales[destination_row, 0] = scale[0]
    scales[destination_row, 1] = scale[1]
    scales[destination_row, 2] = scale[2]


@wp.kernel
def extract_contiguous_world_scales(
    matrices: wp.array2d(dtype=wp.float64),
    scales: wp.array2d(dtype=wp.float64),
):
    row = wp.tid()
    scale = _matrix_scale(matrices, row)
    scales[row, 0] = scale[0]
    scales[row, 1] = scale[1]
    scales[row, 2] = scale[2]


@wp.kernel
def compose_world_xforms(
    positions: wp.array2d(dtype=wp.float32),
    orientations: wp.array2d(dtype=wp.float32),
    scales: wp.array2d(dtype=wp.float64),
    matrices: wp.array(dtype=wp.mat44d),
):
    row = wp.tid()
    position = wp.vec3f(positions[row, 0], positions[row, 1], positions[row, 2])
    orientation = wp.quatf(
        orientations[row, 0],
        orientations[row, 1],
        orientations[row, 2],
        orientations[row, 3],
    )
    scale = wp.vec3d(scales[row, 0], scales[row, 1], scales[row, 2])
    matrices[row] = _compose_world_xform(position, orientation, scale)


@wp.kernel
def compose_world_xforms_from_matrices(
    positions: wp.array2d(dtype=wp.float32),
    orientations: wp.array2d(dtype=wp.float32),
    source_matrices: wp.array2d(dtype=wp.float64),
    matrices: wp.array(dtype=wp.mat44d),
):
    row = wp.tid()
    position = wp.vec3f(positions[row, 0], positions[row, 1], positions[row, 2])
    orientation = wp.quatf(
        orientations[row, 0],
        orientations[row, 1],
        orientations[row, 2],
        orientations[row, 3],
    )
    matrices[row] = _compose_world_xform(position, orientation, _matrix_scale(source_matrices, row))


@wp.kernel
def merge_instancer_poses(
    positions: wp.array2d(dtype=wp.float32),
    orientations: wp.array2d(dtype=wp.float32),
    read_count: int,
    baseline_positions: wp.array2d(dtype=wp.float32),
    baseline_orientations: wp.array2d(dtype=wp.float16),
    baseline_position_count: int,
    baseline_orientation_count: int,
    output_positions: wp.array2d(dtype=wp.float32),
    output_orientations: wp.array2d(dtype=wp.float16),
):
    row = wp.tid()
    live = False
    if row < read_count:
        live = (
            orientations[row, 0] != 0.0
            or orientations[row, 1] != 0.0
            or orientations[row, 2] != 0.0
            or orientations[row, 3] != 0.0
        )

    if live:
        output_positions[row, 0] = positions[row, 0]
        output_positions[row, 1] = positions[row, 1]
        output_positions[row, 2] = positions[row, 2]
        output_orientations[row, 0] = wp.float16(orientations[row, 0])
        output_orientations[row, 1] = wp.float16(orientations[row, 1])
        output_orientations[row, 2] = wp.float16(orientations[row, 2])
        output_orientations[row, 3] = wp.float16(orientations[row, 3])
    else:
        if row < baseline_position_count:
            output_positions[row, 0] = baseline_positions[row, 0]
            output_positions[row, 1] = baseline_positions[row, 1]
            output_positions[row, 2] = baseline_positions[row, 2]
        else:
            output_positions[row, 0] = 0.0
            output_positions[row, 1] = 0.0
            output_positions[row, 2] = 0.0
        if row < baseline_orientation_count:
            output_orientations[row, 0] = baseline_orientations[row, 0]
            output_orientations[row, 1] = baseline_orientations[row, 1]
            output_orientations[row, 2] = baseline_orientations[row, 2]
            output_orientations[row, 3] = baseline_orientations[row, 3]
        else:
            output_orientations[row, 0] = wp.float16(0.0)
            output_orientations[row, 1] = wp.float16(0.0)
            output_orientations[row, 2] = wp.float16(0.0)
            output_orientations[row, 3] = wp.float16(1.0)
