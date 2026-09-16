# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""CUDA stream handoff tests for direct Warp output-read arrays.

Direct pointer wrapping has no producer event protocol of its own. Before a CUDA
array is returned, ovphysx orders its completion event onto Warp's current stream
for that device. The bridge receives Warp's raw CUDA stream handle: 0 for the
default stream and the integer handle otherwise.
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
    return physx_sdk.read(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL)


def test_read_orders_each_event_once_on_current_nondefault_warp_stream(physx_sdk, monkeypatch):
    """The read itself, not a later DLPack import, performs the Warp-stream handoff."""
    wp = _warp_cuda()
    _load(physx_sdk, 2)

    real_wait = physx_sdk._lib.ovphysx_cuda_stream_wait_event
    calls = []

    def recording_wait(stream, event):
        calls.append((stream.value or 0, event.value or 0))
        return real_wait(stream, event)

    monkeypatch.setattr(physx_sdk._lib, "ovphysx_cuda_stream_wait_event", recording_wait)
    stream = wp.Stream(device="cuda:0")
    with wp.ScopedStream(stream), _read_positions(physx_sdk) as result:
        columns = _device_columns(result)
        # NOT a skip: _warp_cuda() and the fixture have already established a CUDA device
        # and a DirectGPU scene, so an empty result means the device read path is gone.
        # That is the regression this file catches, and skipping would turn it green.
        assert columns, "DirectGPU read produced no CUDA columns -- the device read path is gone"
        awaited = {id(group): int(group.cuda_wait_event) for group, _ in columns if group.cuda_wait_event}
        events = set(awaited.values())
        values = columns[0][1].numpy()

    assert events
    # One wait per CUDA group, not per column: an array group emits one tensor per prim.
    assert len(calls) == len(awaited)
    assert {event for _, event in calls} == events
    assert {stream_handle for stream_handle, _ in calls} == {int(stream.cuda_stream)}
    assert values.size


def test_cuda_warp_array_is_fresh_on_nondefault_stream(physx_sdk):
    """A direct Warp array is readable on the stream selected when the read opened."""
    wp = _warp_cuda()
    _load(physx_sdk, 2)
    stream = wp.Stream(device="cuda:0")

    with wp.ScopedStream(stream), _read_positions(physx_sdk) as result:
        columns = _device_columns(result)
        # NOT a skip: _warp_cuda() and the fixture have already established a CUDA device
        # and a DirectGPU scene, so an empty result means the device read path is gone.
        # That is the regression this file catches, and skipping would turn it green.
        assert columns, "DirectGPU read produced no CUDA columns -- the device read path is gone"
        early = columns[0][1].numpy()
        again = columns[0][1].numpy()

    assert early.size and np.isfinite(early).all()
    np.testing.assert_array_equal(early, again)

    _step(physx_sdk, 8)
    with wp.ScopedStream(stream), _read_positions(physx_sdk) as result:
        columns = _device_columns(result)
        assert columns
        later = columns[0][1].numpy()

    assert later.shape == early.shape
    assert np.isfinite(later).all()
    assert not np.array_equal(early, later)
    assert (later[:, 2] < early[:, 2] + 1.0e-4).all()


def test_wait_entry_point_accepts_a_live_producer_event(physx_sdk):
    """The native bridge accepts a real producer event on a non-default stream and reports success.

    What this does NOT claim, and why no stream-drain oracle is used:

    A test that queues large copies on the stream, calls the bridge, and asserts
    `not stream.is_complete` reads "the copies are still pending, so the bridge did not
    drain". Whether they are still pending depends on GPU speed and runner load, so that is a
    timing race presented as a synchronization oracle, and growing the buffer only buys
    margin. A test that needs 128 MiB of margin to stay green measures the runner, not the
    product.

    A deterministic version needs a HOST-CONTROLLED GATE: enqueue a dependency the host has not
    yet satisfied, show the call returns anyway, then satisfy it and watch a downstream sentinel
    observe the producer in order. CUDA can express that with cuStreamWaitValue32 or a kernel
    spinning on mapped memory. Warp exposes neither: Event offers only is_complete, and
    Stream only record_event / wait_event. Without a gate BOTH halves are unfalsifiable: a
    missing dependency is visible only while the gather is still in flight, so on any card fast
    enough to finish first, the test passes whether or not the wait was ever issued.

    So the ordering claim is left where it can be made deterministically. That the read issues
    one wait per CUDA group, on exactly the groups' own events, on exactly the caller's stream,
    is asserted by test_read_orders_each_event_once_on_current_nondefault_warp_stream, a
    direct witness of the calls, kept as a SEPARATE assertion from this one. What native CUDA
    does with a correctly-issued wait is not verified here, and is not claimed to be.
    """
    wp = _warp_cuda()
    _load(physx_sdk, 10)
    stream = wp.Stream(device="cuda:0")

    with _read_positions(physx_sdk) as result:
        columns = _device_columns(result)
        # NOT a skip: _warp_cuda() and the fixture have already established a CUDA device
        # and a DirectGPU scene, so an empty result means the device read path is gone.
        # That is the regression this file catches, and skipping would turn it green.
        assert columns, "DirectGPU read produced no CUDA columns -- the device read path is gone"
        event = int(columns[0][0].cuda_wait_event)
        assert event, "a device column must carry a completion event"

        with wp.ScopedStream(stream):
            response = physx_sdk._lib.ovphysx_cuda_stream_wait_event(
                ctypes.c_void_p(stream.cuda_stream), ctypes.c_void_p(event)
            )
            assert response.status == 0, "the bridge rejected a live producer event"
            # Ordering the stream after the producer must leave it usable, not wedged.
            wp.synchronize_stream(stream)


def test_wait_reports_an_error_when_cuda_is_unavailable():
    """Without a CUDA shim, a nonzero event reports an error rather than failing hard."""
    from ovphysx import _bindings

    _bindings._lib.ovphysx_get_optional_cuda_internal.restype = ctypes.c_void_p
    if _bindings._lib.ovphysx_get_optional_cuda_internal() is not None:
        pytest.skip("CUDA is available in this process; the no-CUDA branch is unreachable here")

    response = _bindings._lib.ovphysx_cuda_stream_wait_event(
        ctypes.c_void_p(1), ctypes.c_void_p(0xDEAD)
    )
    assert response.status != 0
    error = _bindings._lib.ovphysx_get_last_error()
    assert error and error.ptr
    detail = ctypes.string_at(error.ptr, error.length).decode("utf-8", errors="replace")
    assert "ovphysx_cuda_stream_wait_event" in detail


def test_zero_event_succeeds_without_cuda():
    """Nothing to await is a success and must not require CUDA."""
    from ovphysx import _bindings

    response = _bindings._lib.ovphysx_cuda_stream_wait_event(
        ctypes.c_void_p(0), ctypes.c_void_p(0)
    )
    assert response.status == 0
