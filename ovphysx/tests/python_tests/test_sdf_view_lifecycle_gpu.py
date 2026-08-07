# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
"""GPU SDF view lifecycle tests (NVBug 6473872).

Regression guards for undisposed or stale SdfView handles across reset_stage()
and detach_ovstage(). The stale-evaluate and live-view teardown tests are the
red/green evidence; destroy-before-detach is a smoke test only.
"""

import numpy as np
import pytest
from test_utils import CudaArray, data_path, load_usd_with_ovstage

_CUBE_PATTERN = "/World/Cube"
_MAX_QUERY_POINTS = 2


def _load_sdf_cube(sdk):
    load_usd_with_ovstage(sdk, data_path("sdf_cube.usda"))
    sdk.wait_all()
    sdk.warmup_gpu()


def _make_sdf_query_buffers(max_q=_MAX_QUERY_POINTS):
    query_pts = np.array(
        [[[0.45, 0.0, 0.0], [0.55, 0.0, 0.0]]],
        dtype=np.float32,
    )
    out = np.zeros((1, max_q, 4), dtype=np.float32)
    in_gpu = CudaArray(query_pts.shape, dtype=np.float32)
    out_gpu = CudaArray(out.shape, dtype=np.float32)
    in_gpu.upload(query_pts)
    out_gpu.upload(out)
    return in_gpu, out_gpu


def _evaluate_sdf_view(sdf_view, in_gpu, out_gpu):
    sdf_view.evaluate(in_gpu.dltensor, out_gpu.dltensor)
    return out_gpu.numpy()


def test_sdf_view_destroy_before_detach_succeeds(physx_sdk):
    """Explicit destroy before detach must return cleanly (no SIGSEGV)."""
    _load_sdf_cube(physx_sdk)

    sdf_view = physx_sdk.create_sdf_view(
        pattern=_CUBE_PATTERN, max_query_points=_MAX_QUERY_POINTS
    )
    assert sdf_view.count == 1

    in_gpu, out_gpu = _make_sdf_query_buffers()
    result = _evaluate_sdf_view(sdf_view, in_gpu, out_gpu)
    assert result[0, 0, 3] < 0
    assert result[0, 1, 3] > 0

    sdf_view.destroy()
    physx_sdk.detach_ovstage()


def test_sdf_view_stale_evaluate_after_reset_stage_raises(physx_sdk):
    """Reusing an SdfView across reset_stage must raise, not crash."""
    _load_sdf_cube(physx_sdk)

    sdf_view = physx_sdk.create_sdf_view(
        pattern=_CUBE_PATTERN, max_query_points=_MAX_QUERY_POINTS
    )
    in_gpu, out_gpu = _make_sdf_query_buffers()
    _evaluate_sdf_view(sdf_view, in_gpu, out_gpu)

    physx_sdk.reset_stage()
    physx_sdk.wait_all()

    load_usd_with_ovstage(physx_sdk, data_path("sdf_cube.usda"))
    physx_sdk.wait_all()
    physx_sdk.warmup_gpu()

    with pytest.raises(RuntimeError, match="SDF view|stage changed|not found"):
        _evaluate_sdf_view(sdf_view, in_gpu, out_gpu)


def test_sdf_view_reset_stage_with_live_view_does_not_crash(physx_sdk):
    """reset_stage with an undisposed SdfView must not SIGSEGV the process."""
    _load_sdf_cube(physx_sdk)

    sdf_view = physx_sdk.create_sdf_view(
        pattern=_CUBE_PATTERN, max_query_points=_MAX_QUERY_POINTS
    )
    in_gpu, out_gpu = _make_sdf_query_buffers()
    _evaluate_sdf_view(sdf_view, in_gpu, out_gpu)

    physx_sdk.reset_stage()
    physx_sdk.wait_all()

    with pytest.raises(RuntimeError, match="SDF view|stage changed|not found"):
        _evaluate_sdf_view(sdf_view, in_gpu, out_gpu)


def test_sdf_view_detach_with_live_view_does_not_crash(physx_sdk):
    """detach_ovstage with an undisposed SdfView must not SIGSEGV the process."""
    _load_sdf_cube(physx_sdk)

    sdf_view = physx_sdk.create_sdf_view(
        pattern=_CUBE_PATTERN, max_query_points=_MAX_QUERY_POINTS
    )
    in_gpu, out_gpu = _make_sdf_query_buffers()
    _evaluate_sdf_view(sdf_view, in_gpu, out_gpu)

    physx_sdk.detach_ovstage()


def test_sdf_view_destroy_after_reset_stage_is_idempotent(physx_sdk):
    """destroy() on a stale handle after reset_stage cleanup must not raise."""
    _load_sdf_cube(physx_sdk)

    sdf_view = physx_sdk.create_sdf_view(
        pattern=_CUBE_PATTERN, max_query_points=_MAX_QUERY_POINTS
    )
    in_gpu, out_gpu = _make_sdf_query_buffers()
    _evaluate_sdf_view(sdf_view, in_gpu, out_gpu)

    physx_sdk.reset_stage()
    physx_sdk.wait_all()

    sdf_view.destroy()
    sdf_view.destroy()


def test_sdf_view_new_view_after_reset_works(physx_sdk):
    """After reset and reload, a freshly created SdfView must evaluate normally."""
    _load_sdf_cube(physx_sdk)

    with physx_sdk.create_sdf_view(
        pattern=_CUBE_PATTERN, max_query_points=_MAX_QUERY_POINTS
    ) as sdf_view:
        in_gpu, out_gpu = _make_sdf_query_buffers()
        _evaluate_sdf_view(sdf_view, in_gpu, out_gpu)

    physx_sdk.reset_stage()
    physx_sdk.wait_all()

    _load_sdf_cube(physx_sdk)

    sdf_view = physx_sdk.create_sdf_view(
        pattern=_CUBE_PATTERN, max_query_points=_MAX_QUERY_POINTS
    )
    try:
        in_gpu, out_gpu = _make_sdf_query_buffers()
        result = _evaluate_sdf_view(sdf_view, in_gpu, out_gpu)
        assert result[0, 0, 3] < 0
        assert result[0, 1, 3] > 0
    finally:
        sdf_view.destroy()
