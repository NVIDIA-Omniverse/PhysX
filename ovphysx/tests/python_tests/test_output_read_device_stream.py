# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""CUDA stream handoff tests for direct Warp output-read arrays.

Direct pointer wrapping has no producer event protocol of its own. Before a CUDA
array is returned, a private checked wait orders Warp's current stream after
the borrowed completion event.
"""

# @implements REQ-PYTHON-READ-001
# @covers AC-4
# @maps_to TEST-PYTHON-READ-001


import ctypes

import numpy as np
import pytest
from ovphysx.types import ObjectScope, SimObjectType
from test_utils import data_path, load_usd_with_ovstage


def _warp_cuda():
    wp = pytest.importorskip("warp", reason="warp-lang is required for output-read tests")
    wp.init()
    if wp.get_cuda_device_count() == 0:
        pytest.skip("no CUDA device available")
    return wp


def _device_columns(result):
    """Every CUDA Warp column in a read, as (group, tensor) pairs."""
    import warp as wp

    return [
        (group, tensor)
        for group in result.groups
        for tensor in group.tensors
        if isinstance(tensor, wp.array) and tensor.device.is_cuda
    ]


def _load(physx_sdk, steps):
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    _step(physx_sdk, steps)


def _step(physx_sdk, count):
    for _ in range(count):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()


def _read_positions(physx_sdk):
    return physx_sdk.read_sync(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL)


def _read_on_stream(array, wp, stream):
    """Copy on the selected consumer stream before inspecting host values."""
    host = wp.empty(shape=array.shape, dtype=array.dtype, device="cpu", pinned=True)
    wp.copy(host, array, stream=stream)
    wp.synchronize_stream(stream)
    return host.numpy()


def test_read_orders_each_event_once_on_current_nondefault_warp_stream(physx_sdk, monkeypatch):
    """The read itself, not a later DLPack import, performs the Warp-stream handoff."""
    wp = _warp_cuda()
    _load(physx_sdk, 2)

    real_wait = physx_sdk._lib.ovphysx_internal_cuda_stream_wait_event
    calls = []

    def recording_wait(stream, event):
        calls.append((stream.value, event.value))
        return real_wait(stream, event)

    monkeypatch.setattr(physx_sdk._lib, "ovphysx_internal_cuda_stream_wait_event", recording_wait)
    stream = wp.Stream(device="cuda:0")
    with wp.ScopedStream(stream), _read_positions(physx_sdk) as result:
        columns = _device_columns(result)
        # NOT a skip: _warp_cuda() and the fixture have already established a CUDA device
        # and a DirectGPU scene, so an empty result means the device read path is gone.
        # That is the regression this file catches, and skipping would turn it green.
        assert columns, "DirectGPU read produced no CUDA columns -- the device read path is gone"
        awaited = {id(group): int(group.cuda_wait_event) for group, _ in columns if group.cuda_wait_event}
        events = set(awaited.values())
        values = _read_on_stream(columns[0][1], wp, stream)

    assert events
    # One wait per CUDA group, not per column: an array group emits one tensor per prim.
    assert len(calls) == len(awaited)
    assert {event for _, event in calls} == events
    assert {stream_handle for stream_handle, _ in calls} == {int(stream.cuda_stream)}
    assert values.size


def test_cuda_warp_array_is_fresh_on_nondefault_stream(physx_sdk):
    """A direct Warp array is readable on the stream selected when the groups materialize."""
    wp = _warp_cuda()
    _load(physx_sdk, 2)
    stream = wp.Stream(device="cuda:0")

    with wp.ScopedStream(stream), _read_positions(physx_sdk) as result:
        columns = _device_columns(result)
        # NOT a skip: _warp_cuda() and the fixture have already established a CUDA device
        # and a DirectGPU scene, so an empty result means the device read path is gone.
        # That is the regression this file catches, and skipping would turn it green.
        assert columns, "DirectGPU read produced no CUDA columns -- the device read path is gone"
        early = _read_on_stream(columns[0][1], wp, stream)
        again = _read_on_stream(columns[0][1], wp, stream)

    assert early.size and np.isfinite(early).all()
    np.testing.assert_array_equal(early, again)

    _step(physx_sdk, 8)
    with wp.ScopedStream(stream), _read_positions(physx_sdk) as result:
        columns = _device_columns(result)
        assert columns
        later = _read_on_stream(columns[0][1], wp, stream)

    assert later.shape == early.shape
    assert np.isfinite(later).all()
    assert not np.array_equal(early, later)
    assert (later[:, 2] < early[:, 2] + 1.0e-4).all()


# @implements REQ-PYTHON-TASK-001
# @covers AC-6 AC-9
# @maps_to TEST-PYTHON-TASK-001
def test_async_fetch_materializes_on_first_result_stream_not_collection_stream(physx_sdk, monkeypatch):
    """Native collection alone must not bind a future array to the collector's stream."""
    wp = _warp_cuda()
    _load(physx_sdk, 2)
    query = physx_sdk.query(SimObjectType.RIGID_BODY, metadata=False).wait()
    read = physx_sdk.read(query, ["position"]).wait()
    operation = read.fetch_next()
    real_wait = physx_sdk._lib.ovphysx_internal_cuda_stream_wait_event
    calls = []

    def recording_wait(stream, event):
        calls.append((stream.value, event.value))
        return real_wait(stream, event)

    monkeypatch.setattr(physx_sdk._lib, "ovphysx_internal_cuda_stream_wait_event", recording_wait)
    collector = wp.Stream(device="cuda:0")
    consumer = wp.Stream(device="cuda:0")
    try:
        with wp.ScopedStream(collector):
            physx_sdk.wait_all()
        assert calls == [] and operation.done
        with wp.ScopedStream(consumer):
            group = operation.result
            assert group is not None and group.cuda_wait_event
            assert group.tensors[0].device.is_cuda
            values = _read_on_stream(group.tensors[0], wp, consumer)
            assert operation.wait() is group
        assert calls == [(int(consumer.cuda_stream), int(group.cuda_wait_event))]
        assert values.size and np.isfinite(values).all()
    finally:
        read.close().wait()
        query.close().wait()


def test_private_wait_zero_event_is_noop():
    """No producer event must not require CUDA or dereference the stream."""
    from ovphysx import _bindings

    response = _bindings._lib.ovphysx_internal_cuda_stream_wait_event(
        ctypes.c_void_p(0), ctypes.c_void_p(0)
    )
    assert response.status == 0
