# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import warp as wp


@wp.kernel
def _arange_k(a: wp.array(dtype=wp.int32)):
    tid = wp.tid()
    a[tid] = tid


def arange(n, device="cpu"):
    a = wp.empty(n, dtype=wp.int32, device=device)
    wp.launch(kernel=_arange_k, dim=n, inputs=[a], device=device)
    wp.synchronize()
    return a


@wp.kernel
def _linspace_k(a: wp.array(dtype=wp.float32), offset: wp.float32, step: wp.float32):
    tid = wp.tid()
    a[tid] = offset + float(tid) * step


def linspace(n, start, end, include_end=False, include_start=True, device="cpu"):
    d = n - 1
    if not include_start:
        d += 1
    if not include_end:
        d += 1

    step = (end - start) / d
    if not include_start:
        offset = start + step
    else:
        offset = start

    a = wp.empty(n, dtype=wp.float32, device=device)
    wp.launch(kernel=_linspace_k, dim=n, inputs=[a, offset, step], device=device)
    wp.synchronize()
    return a


@wp.kernel
def _fill_float32_k(a: wp.array(dtype=wp.float32), value: wp.float32):
    tid = wp.tid()
    a[tid] = value


def fill_float32(n, value=0.0, device="cpu"):
    a = wp.empty(n, dtype=wp.float32, device=device)
    wp.launch(kernel=_fill_float32_k, dim=n, inputs=[a, value], device=device)
    wp.synchronize()
    return a


@wp.kernel
def _fill_vec3_k(a: wp.array(dtype=wp.vec3), value: wp.vec3):
    tid = wp.tid()
    a[tid] = value


def fill_vec3(n, value=wp.vec3(0.0, 0.0, 0.0), device="cpu"):
    a = wp.empty(n, dtype=wp.vec3, device=device)
    wp.launch(kernel=_fill_vec3_k, dim=n, inputs=[a, value], device=device)
    wp.synchronize()
    return a


@wp.kernel
def _random_k(seed: int, a: wp.array(dtype=float), lower:float, upper:float):
    tid = wp.tid()
    state = wp.rand_init(seed, tid)
    a[tid] = wp.randf(state, lower, upper)


def random(n, lower=0.0, upper=1.0, device="cpu", seed=42):
    a = wp.zeros(n, dtype=float, device=device)
    wp.launch(kernel=_random_k, dim=n, inputs=[seed, a, lower, upper], device=device)
    wp.synchronize()
    return a


@wp.kernel
def _compute_dof_forces_k(
    pos: wp.array(dtype=float, ndim=2),
    vel: wp.array(dtype=float, ndim=2),
    force: wp.array(dtype=float, ndim=2),
    stiffness: float,
    damping: float,
):
    i, j = wp.tid()
    pos_target = 0.0
    force[i, j] = stiffness * (pos_target - pos[i, j]) - damping * vel[i, j]


def compute_dof_forces(pos, vel, force, stiffness, damping, device="cpu"):
    wp.launch(kernel=_compute_dof_forces_k, dim=force.shape, inputs=[pos, vel, force, stiffness, damping], device=device)
    wp.synchronize()
