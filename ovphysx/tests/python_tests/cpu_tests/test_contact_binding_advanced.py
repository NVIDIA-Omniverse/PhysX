# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-CONTACT-003
# @covers AC-1 AC-2 AC-3 AC-4

"""Advanced ContactBinding tests: properties, unfiltered error paths, destroy
idempotency, use-after-destroy, and context manager.

Complements test_tensor_bindings_api.py (TestContactBinding) which covers
create/destroy specs, runtime clones, net forces, force matrix, flat buffers,
and capacity/filter preconditions. This file adds:
  - max_contact_data_count / sensor_paths / filter_paths property checks
  - read_normal_force_matrix error on unfiltered binding
  - read_normal_contact_data error on unfiltered binding
  - destroy() idempotency (documented contract on ContactBinding.destroy)
  - use-after-destroy for read_normal_force_matrix and read_net_normal_forces
  - context manager auto-destroy (ContactBinding implements __enter__/__exit__)
  - normal/friction force components, output validation, and deprecated aliases
"""

import ast
import os
import warnings
from pathlib import Path

import ovphysx.api as api

import numpy as np
import pytest
from test_utils import contact_test_case, load_usd_with_ovstage

_TEST_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def data_path(filename):
    return os.path.join(_TEST_DIR, "data", filename)


_SENSOR_PATTERN = "/World/Cube*"
_FILTER_PATTERN = "/World/GroundPlane"


def _load_scene(sdk):
    """Load the rigid-body scene. Contact bindings must be created BEFORE the first step."""
    load_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
    sdk.wait_all()
    return sdk


def _slide_sensor(sdk):
    """Set tangential motion so the next step reports a substantial friction force."""
    from ovphysx.types import TensorType

    with pytest.warns(DeprecationWarning):
        velocity = sdk.create_tensor_binding(
            pattern="/World/Cube1", tensor_type=TensorType.RIGID_BODY_VELOCITY,
        )
    try:
        values = np.zeros(velocity.shape, dtype=np.float32)
        values[0, 0] = 0.5
        velocity.write(values)
        sdk.wait_all()
        sdk.step_sync(1.0 / 60.0)
    finally:
        velocity.destroy()


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
# read_normal_force_matrix error on unfiltered binding
# ---------------------------------------------------------------------------


def test_read_normal_force_matrix_unfiltered_binding_raises(physx_sdk):
    """read_normal_force_matrix is not meaningful on an unfiltered binding (filter_count=0).

    The C API enforces dst.shape == (sensor_count, filter_count, 3). For an
    unfiltered binding that means filter_count=0, and any non-degenerate dst
    is rejected with a shape-mismatch error. Pin that rejection so accidental
    shape mismatches do not silently succeed.
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
            cb.read_normal_force_matrix(buf)
    finally:
        cb.destroy()


# ---------------------------------------------------------------------------
# read_normal_contact_data error on unfiltered binding
# ---------------------------------------------------------------------------


def test_read_normal_contact_data_unfiltered_binding_raises(physx_sdk):
    """read_normal_contact_data on an unfiltered binding must raise RuntimeError."""
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
            cb.read_normal_contact_data(forces, positions, normals, separations, counts, starts)
    finally:
        cb.destroy()


# ---------------------------------------------------------------------------
# destroy() idempotency
# ---------------------------------------------------------------------------


def test_contact_binding_destroy_idempotent(physx_sdk):
    """Calling destroy() twice must not raise.

    ContactBinding.destroy() is documented as "Safe to call multiple times."
    This pins that contract.
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
        "read_net_normal_forces",
        "read_normal_force_matrix",
    ],
)
def test_contact_binding_read_after_destroy_raises(physx_sdk, method_name):
    """Calling any read method on a destroyed ContactBinding must raise RuntimeError."""
    _load_scene(physx_sdk)
    cb = _make_filtered_cb(physx_sdk)
    physx_sdk.step_sync(1.0 / 60.0)
    # Cache sensor_count and filter_count BEFORE destroy. Accessing them on a
    # destroyed binding raises RuntimeError, which would mask the read failure
    # under test.
    sc = cb.sensor_count
    fc = cb.filter_count
    cb.destroy()

    if method_name == "read_net_normal_forces":
        buf = np.zeros((sc, 3), dtype=np.float32)
        with pytest.raises(RuntimeError):
            cb.read_net_normal_forces(buf)
    elif method_name == "read_normal_force_matrix":
        buf = np.zeros((sc, fc, 3), dtype=np.float32)
        with pytest.raises(RuntimeError):
            cb.read_normal_force_matrix(buf)


# ---------------------------------------------------------------------------
# Context manager auto-destroy
# ---------------------------------------------------------------------------


def test_contact_binding_context_manager(physx_sdk):
    """ContactBinding used as a context manager is auto-destroyed on exit.

    ContactBinding implements __enter__/__exit__, and __exit__ calls destroy().
    After the `with` block, any read must raise.
    """
    _load_scene(physx_sdk)
    with physx_sdk.create_contact_binding(
        sensor_patterns=[_SENSOR_PATTERN],
        max_contact_data_count=64,
    ) as cb:
        physx_sdk.step_sync(1.0 / 60.0)
        sc = cb.sensor_count
        net = np.zeros((sc, 3), dtype=np.float32)
        cb.read_net_normal_forces(net)
    # After context exit, reading must raise.
    with pytest.raises(RuntimeError):
        cb.read_net_normal_forces(net)


def test_force_components(physx_sdk):
    """Net friction includes the support excluded by the filter; components remain separate."""
    _load_scene(physx_sdk)
    with (
        physx_sdk.create_contact_binding(sensor_patterns=["/World/Cube1"]) as unfiltered,
        physx_sdk.create_contact_binding(
            sensor_patterns=["/World/Cube1"],
            filter_patterns=["/World/GroundPlane/CollisionPlane"],
            filters_per_sensor=1,
        ) as excluded,
        physx_sdk.create_contact_binding(
            sensor_patterns=["/World/Cube1"], filter_patterns=["/World/BigBase"],
            filters_per_sensor=1,
        ) as support,
    ):
        for _ in range(120):
            physx_sdk.step_sync(1.0 / 60.0)
        _slide_sensor(physx_sdk)

        normal = np.empty((1, 3), dtype=np.float32)
        friction = np.empty_like(normal)
        excluded_friction = np.empty_like(normal)
        unfiltered.read_net_normal_forces(normal)
        unfiltered.read_net_friction_forces(friction)
        assert normal[0, 2] > 0.0
        assert friction[0, 0] < -0.1
        excluded.read_net_friction_forces(excluded_friction)
        np.testing.assert_allclose(excluded_friction, friction)

        matrix = np.empty((1, 1, 3), dtype=np.float32)
        support.read_normal_force_matrix(matrix)
        np.testing.assert_allclose(matrix[:, 0], normal)
        excluded.read_normal_force_matrix(matrix)
        np.testing.assert_array_equal(matrix, 0.0)
        support.read_friction_force_matrix(matrix)
        np.testing.assert_allclose(matrix[:, 0], friction)
        excluded.read_friction_force_matrix(matrix)
        np.testing.assert_array_equal(matrix, 0.0)
        with pytest.raises(RuntimeError, match="expected dst shape"):
            support.read_friction_force_matrix(np.empty((1, 1, 2), dtype=np.float32))
        with pytest.raises(RuntimeError, match="expected dst shape"):
            unfiltered.read_friction_force_matrix(matrix)
        empty_matrix = np.empty((1, 0, 3), dtype=np.float32)
        unfiltered.read_normal_force_matrix(empty_matrix)
        unfiltered.read_friction_force_matrix(empty_matrix)
        with pytest.raises(RuntimeError, match="expected dst shape"):
            unfiltered.read_net_friction_forces(np.empty((1, 2), dtype=np.float32))
    with pytest.raises(RuntimeError, match="destroyed"):
        unfiltered.read_net_friction_forces(friction)
    with pytest.raises(RuntimeError, match="destroyed"):
        support.read_friction_force_matrix(matrix)


def test_contact_component_validation(physx_sdk):
    """All force components reject invalid output tensors and use after destruction."""
    _load_scene(physx_sdk)
    readers = (
        ("read_net_normal_forces", False), ("read_net_friction_forces", False),
        ("read_normal_force_matrix", True), ("read_friction_force_matrix", True),
    )
    outputs = []
    with physx_sdk.create_contact_binding(
        sensor_patterns=["/World/Cube1"], filter_patterns=["/World/BigBase"],
        filters_per_sensor=1,
    ) as binding:
        physx_sdk.step_sync(1.0 / 60.0)
        for reader, is_matrix in readers:
            with contact_test_case(reader=reader):
                shape = (1, 1, 3) if is_matrix else (1, 3)
                method = getattr(binding, reader)
                with pytest.raises(RuntimeError, match="float32"):
                    method(np.zeros(shape, dtype=np.float64))
                bad_shapes = ((1, 2), (1, 1, 2)) if is_matrix else ((1, 2),)
                for bad_shape in bad_shapes:
                    with pytest.raises(RuntimeError, match="expected dst shape"):
                        method(np.zeros(bad_shape, dtype=np.float32))
                output = np.zeros(shape, dtype=np.float32)
                method(output)
                outputs.append((reader, method, output))
    for reader, method, output in outputs:
        with contact_test_case(reader=reader), pytest.raises(RuntimeError, match="destroyed"):
            method(output)


def _contact_data_buffers(binding, component):
    """Allocate output buffers without selecting an API entry point."""
    capacity = binding.max_contact_data_count
    pair_shape = (binding.sensor_count, binding.filter_count)
    if component == "normal":
        shapes = [(capacity, 1), (capacity, 3), (capacity, 3), (capacity, 1)]
    else:
        shapes = [(capacity, 3), (capacity, 3)]
    return ([np.zeros(shape, dtype=np.float32) for shape in shapes]
            + [np.zeros(pair_shape, dtype=np.int32) for _ in range(2)])


@pytest.mark.parametrize("capacity", [1, 64])
@pytest.mark.parametrize("component", ["normal", "friction"])
def test_detailed_contact_data_force_components(physx_sdk, capacity, component):
    """Detailed forces reconstruct complete matrices and report truncation demand."""
    _load_scene(physx_sdk)
    with physx_sdk.create_contact_binding(
        sensor_patterns=["/World/Cube1"], filter_patterns=["/World/BigBase"],
        filters_per_sensor=1, max_contact_data_count=capacity,
    ) as binding:
        for _ in range(120):
            physx_sdk.step_sync(1.0 / 60.0)

        if component == "normal":
            read = binding.read_normal_contact_data
        else:
            _slide_sensor(physx_sdk)
            read = binding.read_friction_contact_data
        buffers = _contact_data_buffers(binding, component)
        required = read(*buffers)
        assert required > 1

        forces, counts, starts = buffers[0], buffers[-2], buffers[-1]
        start, count = int(starts[0, 0]), int(counts[0, 0])
        assert 0 < count <= capacity
        assert start + count <= capacity
        if component == "normal":
            assert np.all(forces[start:start + count] > 0.0)
        else:
            assert forces[start:start + count, 0].sum() < -0.1
            friction_matrix = np.empty((1, 1, 3), dtype=np.float32)
            net_friction = np.empty((1, 3), dtype=np.float32)
            binding.read_friction_force_matrix(friction_matrix)
            binding.read_net_friction_forces(net_friction)
            np.testing.assert_allclose(friction_matrix[:, 0], net_friction, rtol=1e-5, atol=1e-5)
        if required <= capacity:
            if component == "normal":
                normal_matrix = np.empty((1, 1, 3), dtype=np.float32)
                binding.read_normal_force_matrix(normal_matrix)
                np.testing.assert_allclose(
                    (forces[start:start + count] * buffers[2][start:start + count]).sum(axis=0),
                    normal_matrix[0, 0], rtol=1e-5, atol=1e-5,
                )
            else:
                np.testing.assert_allclose(
                    forces[start:start + count].sum(axis=0), friction_matrix[0, 0], rtol=1e-5, atol=1e-5,
                )
        else:
            assert count == capacity


# Compatibility only: these tests retire with the deprecated contact aliases.
@pytest.mark.parametrize("current_name, deprecated_name, is_matrix", [
    ("read_net_normal_forces", "read_net_forces", False),
    ("read_normal_force_matrix", "read_force_matrix", True),
])
def test_deprecated_aggregate_contact_aliases(physx_sdk, current_name, deprecated_name, is_matrix):
    _load_scene(physx_sdk)
    with physx_sdk.create_contact_binding(
        sensor_patterns=["/World/Cube1"], filter_patterns=["/World/BigBase"],
        filters_per_sensor=1,
    ) as binding:
        for _ in range(120):
            physx_sdk.step_sync(1.0 / 60.0)
        shape = (1, 1, 3) if is_matrix else (1, 3)
        current = np.zeros(shape, dtype=np.float32)
        deprecated = np.zeros_like(current)
        getattr(binding, current_name)(current)
        assert current[..., 2].item() > 0.0
        with pytest.warns(DeprecationWarning, match=current_name):
            getattr(binding, deprecated_name)(deprecated)
        np.testing.assert_array_equal(deprecated, current)


@pytest.mark.parametrize("capacity", [1, 64])
@pytest.mark.parametrize("component, current_name, deprecated_name", [
    ("normal", "read_normal_contact_data", "read_contact_data"),
    ("friction", "read_friction_contact_data", "read_friction_data"),
])
def test_deprecated_detailed_contact_aliases(physx_sdk, capacity, component, current_name, deprecated_name):
    _load_scene(physx_sdk)
    with physx_sdk.create_contact_binding(
        sensor_patterns=["/World/Cube1"], filter_patterns=["/World/BigBase"],
        filters_per_sensor=1, max_contact_data_count=capacity,
    ) as binding:
        for _ in range(120):
            physx_sdk.step_sync(1.0 / 60.0)
        if component == "friction":
            _slide_sensor(physx_sdk)
        current = _contact_data_buffers(binding, component)
        deprecated = _contact_data_buffers(binding, component)
        required = getattr(binding, current_name)(*current)
        assert required > 1
        assert (required > capacity) == (capacity == 1)
        with pytest.warns(DeprecationWarning, match=current_name):
            deprecated_required = getattr(binding, deprecated_name)(*deprecated)
        assert deprecated_required == required
        for actual, expected in zip(deprecated, current):
            np.testing.assert_array_equal(actual, expected)



def test_contact_binding_factory_deprecation(physx_sdk):
    """The deprecated factory warns at the caller and still returns a usable binding."""
    _load_scene(physx_sdk)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always", DeprecationWarning)
        binding = physx_sdk.create_contact_binding([_SENSOR_PATTERN])
    try:
        deprecations = [warning for warning in caught
                        if issubclass(warning.category, DeprecationWarning)
                        and "contact bindings are deprecated" in str(warning.message)]
        assert len(deprecations) == 1
        assert Path(deprecations[0].filename).resolve() == Path(__file__).resolve()
        assert binding.sensor_count > 0
        output = np.zeros((binding.sensor_count, 3), dtype=np.float32)
        binding.read_net_normal_forces(output)
        np.testing.assert_array_equal(output, 0.0)
    finally:
        binding.destroy()


def test_contact_component_deprecation_annotations():
    """Type checkers and generated docs identify the component methods as deprecated."""
    module = ast.parse(Path(api.__file__).with_suffix(".pyi").read_text(encoding="utf-8"))
    classes = {node.name: node for node in module.body if isinstance(node, ast.ClassDef)}
    methods = (
        "read_net_normal_forces", "read_net_friction_forces",
        "read_normal_force_matrix", "read_friction_force_matrix",
        "read_normal_contact_data", "read_friction_contact_data",
    )
    contact = classes["ContactBinding"]
    declarations = [contact]
    declarations += [node for node in contact.body
                     if isinstance(node, ast.FunctionDef) and node.name in methods]
    declarations += [node for node in classes["PhysX"].body
                     if isinstance(node, ast.FunctionDef) and node.name == "create_contact_binding"]
    assert len(declarations) == len(methods) + 2
    for node in declarations:
        markers = [decorator for decorator in node.decorator_list
                   if isinstance(decorator, ast.Call)
                   and isinstance(decorator.func, ast.Name)
                   and decorator.func.id == "deprecated"]
        assert len(markers) == 1, node.name
        assert "contact-binding API is deprecated" in markers[0].args[0].value, node.name
    for obj in (api.ContactBinding, api.PhysX.create_contact_binding,
                *(getattr(api.ContactBinding, method) for method in methods)):
        assert ".. deprecated:: 0.6.0" in obj.__doc__, obj.__name__
