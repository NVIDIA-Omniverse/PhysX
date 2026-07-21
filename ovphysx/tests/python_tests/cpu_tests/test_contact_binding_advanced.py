# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
"""Advanced ContactBinding tests: properties, unfiltered error paths, destroy
idempotency, use-after-destroy, and context manager.

Complements test_tensor_bindings_api.py (TestContactBinding) which covers
create/destroy specs, runtime clones, net forces, force matrix, flat buffers,
and capacity/filter preconditions. This file adds:
  - max_contact_data_count / sensor_paths / filter_paths property checks
  - read_force_matrix error on unfiltered binding
  - read_contact_data error on unfiltered binding
  - destroy() idempotency (documented contract on ContactBinding.destroy)
  - use-after-destroy for read_force_matrix and read_net_forces
  - context manager auto-destroy (ContactBinding implements __enter__/__exit__)
"""

import os

import numpy as np
import pytest
from test_utils import load_usd_with_ovstage

_TEST_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def data_path(filename):
    return os.path.join(_TEST_DIR, "data", filename)


_SENSOR_PATTERN = "/World/Cube*"
_FILTER_PATTERN = "/World/GroundPlane"


def _load_scene(sdk):
    """Load the rigid-body scene; contact bindings must be created BEFORE the first step."""
    load_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
    sdk.wait_all()
    return sdk


# ---------------------------------------------------------------------------
# Property: max_contact_data_count
# ---------------------------------------------------------------------------


def test_max_contact_data_count_property(physx_sdk):
    """max_contact_data_count must equal the value passed at creation."""
    _load_scene(physx_sdk)
    CAPACITY = 128
    cb = physx_sdk.create_contact_binding(
        sensor_patterns=[_SENSOR_PATTERN],
        filter_patterns=[_FILTER_PATTERN],
        filters_per_sensor=1,
        max_contact_data_count=CAPACITY,
    )
    try:
        assert cb.max_contact_data_count == CAPACITY
    finally:
        cb.destroy()


# ---------------------------------------------------------------------------
# Property: sensor_paths
# ---------------------------------------------------------------------------


def test_sensor_paths_returns_list_of_strings(physx_sdk):
    """sensor_paths is a list whose entries are strings and length == sensor_count."""
    _load_scene(physx_sdk)
    cb = physx_sdk.create_contact_binding(
        sensor_patterns=[_SENSOR_PATTERN],
        filter_patterns=[_FILTER_PATTERN],
        filters_per_sensor=1,
        max_contact_data_count=64,
    )
    try:
        paths = cb.sensor_paths
        assert isinstance(paths, list)
        assert len(paths) == cb.sensor_count
        for p in paths:
            assert isinstance(p, str)
    finally:
        cb.destroy()


# ---------------------------------------------------------------------------
# Property: filter_paths
# ---------------------------------------------------------------------------


def test_filter_paths_returns_list(physx_sdk):
    """filter_paths is a list[list[str]] with one inner list per sensor."""
    _load_scene(physx_sdk)
    cb = physx_sdk.create_contact_binding(
        sensor_patterns=[_SENSOR_PATTERN],
        filter_patterns=[_FILTER_PATTERN],
        filters_per_sensor=1,
        max_contact_data_count=64,
    )
    try:
        fpaths = cb.filter_paths
        assert isinstance(fpaths, list)
        assert len(fpaths) > 0
        # Pin the actual contract: list[list[str]] (one inner list per sensor)
        assert isinstance(fpaths[0], list), f"Expected nested list, got {type(fpaths[0])}"
        for inner in fpaths:
            assert isinstance(inner, list)
            for path in inner:
                assert isinstance(path, str)
    finally:
        cb.destroy()


# ---------------------------------------------------------------------------
# read_force_matrix error on unfiltered binding
# ---------------------------------------------------------------------------


def test_read_force_matrix_unfiltered_binding_raises(physx_sdk):
    """read_force_matrix is not meaningful on an unfiltered binding (filter_count=0).

    The C API enforces dst.shape == (sensor_count, filter_count, 3). For an
    unfiltered binding that means filter_count=0, and any non-degenerate dst
    is rejected with a shape-mismatch error. Pin that rejection so accidental
    shape mismatches don't silently succeed.
    """
    _load_scene(physx_sdk)
    cb = physx_sdk.create_contact_binding(
        sensor_patterns=[_SENSOR_PATTERN],
        max_contact_data_count=64,
    )
    # Step so there is some contact state
    physx_sdk.step_sync(1.0 / 60.0)
    try:
        buf = np.zeros((cb.sensor_count, 1, 3), dtype=np.float32)
        with pytest.raises(RuntimeError, match="expected dst shape"):
            cb.read_force_matrix(buf)
    finally:
        cb.destroy()


# ---------------------------------------------------------------------------
# read_contact_data error on unfiltered binding
# ---------------------------------------------------------------------------


def test_read_contact_data_unfiltered_binding_raises(physx_sdk):
    """read_contact_data on an unfiltered binding must raise RuntimeError."""
    _load_scene(physx_sdk)
    cb = physx_sdk.create_contact_binding(
        sensor_patterns=[_SENSOR_PATTERN],
        max_contact_data_count=64,
    )
    physx_sdk.step_sync(1.0 / 60.0)
    try:
        S = cb.sensor_count
        C = cb.max_contact_data_count
        forces = np.zeros((C, 1), dtype=np.float32)
        positions = np.zeros((C, 3), dtype=np.float32)
        normals = np.zeros((C, 3), dtype=np.float32)
        separations = np.zeros((C, 1), dtype=np.float32)
        counts = np.zeros((S, 1), dtype=np.int32)
        starts = np.zeros((S, 1), dtype=np.int32)
        with pytest.raises(RuntimeError, match="filters_per_sensor"):
            cb.read_contact_data(forces, positions, normals, separations, counts, starts)
    finally:
        cb.destroy()


# ---------------------------------------------------------------------------
# destroy() idempotency
# ---------------------------------------------------------------------------


def test_contact_binding_destroy_idempotent(physx_sdk):
    """Calling destroy() twice must not raise.

    ContactBinding.destroy() docstring (api.py:902) states "Safe to call
    multiple times." This pins that contract.
    """
    _load_scene(physx_sdk)
    cb = physx_sdk.create_contact_binding(
        sensor_patterns=[_SENSOR_PATTERN],
        max_contact_data_count=64,
    )
    cb.destroy()
    cb.destroy()  # second call must be a no-op


# ---------------------------------------------------------------------------
# Use after destroy
# ---------------------------------------------------------------------------


def _make_filtered_cb(sdk):
    return sdk.create_contact_binding(
        sensor_patterns=[_SENSOR_PATTERN],
        filter_patterns=[_FILTER_PATTERN],
        filters_per_sensor=1,
        max_contact_data_count=64,
    )


@pytest.mark.parametrize(
    "method_name",
    [
        "read_net_forces",
        "read_force_matrix",
    ],
)
def test_contact_binding_read_after_destroy_raises(physx_sdk, method_name):
    """Calling any read method on a destroyed ContactBinding must raise RuntimeError."""
    _load_scene(physx_sdk)
    cb = _make_filtered_cb(physx_sdk)
    physx_sdk.step_sync(1.0 / 60.0)
    # Cache sensor_count and filter_count BEFORE destroy — accessing them on a
    # destroyed binding raises RuntimeError, which would mask the read failure
    # under test.
    sc = cb.sensor_count
    fc = cb.filter_count
    cb.destroy()

    if method_name == "read_net_forces":
        buf = np.zeros((sc, 3), dtype=np.float32)
        with pytest.raises(RuntimeError):
            cb.read_net_forces(buf)
    elif method_name == "read_force_matrix":
        buf = np.zeros((sc, fc, 3), dtype=np.float32)
        with pytest.raises(RuntimeError):
            cb.read_force_matrix(buf)


# ---------------------------------------------------------------------------
# Context manager auto-destroy
# ---------------------------------------------------------------------------


def test_contact_binding_context_manager(physx_sdk):
    """ContactBinding used as a context manager is auto-destroyed on exit.

    ContactBinding implements __enter__/__exit__ (api.py:935-939); __exit__
    calls destroy(). After the `with` block, any read must raise.
    """
    _load_scene(physx_sdk)
    with physx_sdk.create_contact_binding(
        sensor_patterns=[_SENSOR_PATTERN],
        max_contact_data_count=64,
    ) as cb:
        physx_sdk.step_sync(1.0 / 60.0)
        sc = cb.sensor_count
        net = np.zeros((sc, 3), dtype=np.float32)
        cb.read_net_forces(net)
    # After context exit, reading must raise.
    with pytest.raises(RuntimeError):
        cb.read_net_forces(net)
