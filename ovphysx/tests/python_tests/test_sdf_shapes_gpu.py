# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""SDF shapes evaluated through the session query, on a DirectGPU simulation (ADR-0048).

``PhysX.evaluate_sdf_shapes`` is the successor of the deprecated SDF view: the same shapes, values and mesh-local
frame, with Q taken from the arrays on every call and the query opened and released by each call.
"""

# @implements REQ-CAPI-SDF-001
# @covers AC-2 AC-4
# @maps_to TEST-CAPI-SDF-001

import ctypes
import time

import numpy as np
import pytest
from test_utils import CudaArray, data_path, get_cuda_driver, load_usd_with_ovstage


# CUhostFn: a host function a CUDA stream runs in order with its other work.
_HOST_FN = ctypes.CFUNCTYPE(None, ctypes.c_void_p)
# CUDA_ERROR_NOT_READY: what a stream or event query returns while its work is outstanding.
_CUDA_ERROR_NOT_READY = 600


def _load_sdf_cube(sdk):
    load_usd_with_ovstage(sdk, data_path("sdf_cube.usda"))
    sdk.wait_all()


def _evaluate(sdk, points, **selection):
    """One evaluation of host ``points`` [N, Q, 3]; the [N, Q, 4] result read back."""
    in_gpu = CudaArray(points.shape, dtype=np.float32)
    out_gpu = CudaArray(points.shape[:2] + (4,), dtype=np.float32)
    in_gpu.upload(points)
    sdk.evaluate_sdf_shapes(in_gpu.dltensor, out_gpu.dltensor, **selection)
    return out_gpu.numpy()


def test_evaluate_sdf_shapes_cube_distance_signs_and_gradients(physx_sdk):
    """The SDF view's value test through the successor, without a warmup call: the evaluation warms up itself."""
    _load_sdf_cube(physx_sdk)
    # In the cube's local frame (half-extent 0.5): just inside and just outside the +x face.
    points = np.array([[[0.45, 0.0, 0.0], [0.55, 0.0, 0.0]]], dtype=np.float32)
    result = _evaluate(physx_sdk, points, pattern="/World/Cube")
    assert result.shape == (1, 2, 4)
    assert result[0, 0, 3] < 0, f"Expected negative distance inside the cube, got {result[0, 0]}"
    assert result[0, 1, 3] > 0, f"Expected positive distance outside the cube, got {result[0, 1]}"
    assert result[0, 0, 0] > 0.5, f"Expected an outward gradient ~(1, 0, 0) inside, got {result[0, 0]}"


def test_evaluate_sdf_shapes_takes_q_from_each_call(physx_sdk):
    """Two points, then five, through fresh calls: Q is the arrays', and shared points agree across calls."""
    _load_sdf_cube(physx_sdk)
    two = np.array([[[0.45, 0.0, 0.0], [0.55, 0.0, 0.0]]], dtype=np.float32)
    five = np.array(
        [[[0.45, 0.0, 0.0], [0.55, 0.0, 0.0], [0.25, 0.0, 0.0], [0.0, 0.25, 0.0], [0.0, 0.0, -0.25]]],
        dtype=np.float32,
    )
    first = _evaluate(physx_sdk, two, pattern="/World/Cube")
    second = _evaluate(physx_sdk, five, pattern="/World/Cube")
    assert second.shape == (1, 5, 4)
    np.testing.assert_allclose(second[0, :2], first[0], atol=1e-5)
    # Inside, each nearest face's normal: +x, +y, then -z.
    assert second[0, 2, 3] < 0 and second[0, 2, 0] > 0.5, second[0, 2]
    assert second[0, 3, 3] < 0 and second[0, 3, 1] > 0.5, second[0, 3]
    assert second[0, 4, 3] < 0 and second[0, 4, 2] < -0.5, second[0, 4]


def test_evaluate_sdf_shapes_whole_set_matches_the_pattern(physx_sdk):
    """With no selection, every SDF shape: sdf_cube.usda has one, so the whole set reads what the pattern reads."""
    _load_sdf_cube(physx_sdk)
    points = np.array([[[0.45, 0.0, 0.0], [0.55, 0.0, 0.0]]], dtype=np.float32)
    np.testing.assert_allclose(
        _evaluate(physx_sdk, points), _evaluate(physx_sdk, points, pattern="/World/Cube"), atol=1e-6
    )


def test_evaluate_sdf_shapes_refusals_name_the_cause(physx_sdk):
    """N other than the shape count names the count; a pattern matching no SDF shape says so."""
    _load_sdf_cube(physx_sdk)
    with pytest.raises(RuntimeError, match="the query has 1 SDF shapes"):
        _evaluate(physx_sdk, np.zeros((2, 1, 3), dtype=np.float32), pattern="/World/Cube")
    with pytest.raises(RuntimeError, match="did not match any SDF shapes"):
        _evaluate(physx_sdk, np.zeros((1, 1, 3), dtype=np.float32), pattern="/World/Nothing")
    with pytest.raises(RuntimeError, match="at most 65535 SDF shapes"):
        _evaluate(physx_sdk, np.zeros((65536, 1, 3), dtype=np.float32), pattern="/World/Cube")


@pytest.mark.parametrize("handoff", ["stream", "event", "both"])
def test_evaluate_sdf_shapes_waits_for_the_producer(physx_sdk, handoff):
    """A late producer is waited for, through its stream, its event, or both: the copy of new points is queued behind
    a host function that holds the caller's stream, from pinned memory so it truly stays queued, and is still pending
    when the evaluation is called. Only the handoff makes the evaluation read the new points, whose signs are the
    reverse of those already on the device."""
    _load_sdf_cube(physx_sdk)
    # Warmed up first: the first evaluation's own warmup step would otherwise run out the producer's delay.
    physx_sdk.warmup()
    before = np.array([[[0.45, 0.0, 0.0], [0.55, 0.0, 0.0]]], dtype=np.float32)  # inside, then outside
    after = np.array([[[0.55, 0.0, 0.0], [0.45, 0.0, 0.0]]], dtype=np.float32)  # outside, then inside

    drv = get_cuda_driver()
    stream = ctypes.c_void_p()
    event = ctypes.c_void_p()
    pinned = ctypes.c_void_p()
    assert drv.cuStreamCreate(ctypes.byref(stream), 1) == 0  # CU_STREAM_NON_BLOCKING
    assert drv.cuEventCreate(ctypes.byref(event), 2) == 0  # CU_EVENT_DISABLE_TIMING
    assert drv.cuMemAllocHost_v2(ctypes.byref(pinned), ctypes.c_size_t(after.nbytes)) == 0
    hold = _HOST_FN(lambda _: time.sleep(0.5))
    try:
        ctypes.memmove(pinned, after.ctypes.data, after.nbytes)
        in_gpu = CudaArray(before.shape, dtype=np.float32)
        out_gpu = CudaArray(before.shape[:2] + (4,), dtype=np.float32)
        in_gpu.upload(before)
        assert drv.cuLaunchHostFunc(stream, hold, None) == 0
        assert drv.cuMemcpyHtoDAsync_v2(
            ctypes.c_uint64(in_gpu.dltensor.data), pinned, ctypes.c_size_t(after.nbytes), stream) == 0
        assert drv.cuEventRecord(event, stream) == 0
        # Still outstanding as the evaluation is called; otherwise the test could not tell a wait from none.
        assert drv.cuStreamQuery(stream) == _CUDA_ERROR_NOT_READY
        assert drv.cuEventQuery(event) == _CUDA_ERROR_NOT_READY
        physx_sdk.evaluate_sdf_shapes(
            in_gpu.dltensor, out_gpu.dltensor, pattern="/World/Cube",
            cuda_stream=stream.value if handoff != "event" else 0,
            cuda_wait_event=event.value if handoff != "stream" else 0)
        result = out_gpu.numpy()
        assert result[0, 0, 3] > 0 and result[0, 1, 3] < 0, f"read the points before the producer's copy: {result}"
    finally:
        drv.cuStreamSynchronize(stream)
        drv.cuMemFreeHost(pinned)
        drv.cuEventDestroy_v2(event)
        drv.cuStreamDestroy_v2(stream)


def test_evaluate_sdf_shapes_after_detach_ovstage(physx_sdk):
    """Each call opens its own query, so nothing goes stale across a reload: the reloaded cube evaluates again."""
    _load_sdf_cube(physx_sdk)
    points = np.array([[[0.45, 0.0, 0.0]]], dtype=np.float32)
    before = _evaluate(physx_sdk, points, pattern="/World/Cube")
    physx_sdk.detach_ovstage()
    physx_sdk.wait_all()
    _load_sdf_cube(physx_sdk)
    np.testing.assert_allclose(_evaluate(physx_sdk, points, pattern="/World/Cube"), before, atol=1e-5)
