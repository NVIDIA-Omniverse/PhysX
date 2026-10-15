# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-READ-ATTRS-001
# @covers AC-1 AC-10
# @maps_to TEST-READ-ATTRS-001
# @implements REQ-CAPI-WRITE-001
# @covers AC-9
# @maps_to TEST-CAPI-WRITE-001

"""Session-API shape-level property tests: material properties, contact offsets, rest offsets, read and write round-trips."""

import os

import numpy as np
from ovphysx.types import SimObjectType
from session_helpers import read_column, write_column
from test_utils import attach_usd_with_ovstage

_TEST_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def data_path(filename):
    return os.path.join(_TEST_DIR, "data", filename)


RB_PATTERN = "/World/Cube*"
ARTI_PATTERN = "/World/articulation*"
# Their links and joints: a bare last `*` matches direct children only.
ARTI_LINK_PATTERN = "/World/articulation*/*"

MATERIAL_ATTRIBUTES = ("staticFriction", "dynamicFriction", "restitution")


def _shape_rows(sdk, stage, object_type, attribute, pattern):
    """{prim path -> per-shape values of `attribute`}, each row cut to the prim's shapeCount."""
    counts = read_column(sdk, stage, object_type, "shapeCount", pattern=pattern)
    rows = read_column(sdk, stage, object_type, attribute, pattern=pattern)
    return {path: row[: int(counts[path][0])] for path, row in rows.items()}


def _write_shape_rows(sdk, stage, object_type, attribute, pattern, rows):
    """Write `rows` ({prim path -> per-shape values}) to `attribute`; each row's padding is left alone."""

    def fill(path, row):
        row[: rows[path].size] = rows[path]
        return row

    return write_column(sdk, stage, object_type, attribute, fill, pattern=pattern)


def _assert_shape_rows(actual, expected):
    assert actual.keys() == expected.keys()
    for path, row in expected.items():
        np.testing.assert_allclose(actual[path], row, rtol=1e-5, err_msg=path)


# ---------------------------------------------------------------------------
# Rigid body shape-level tensors
# ---------------------------------------------------------------------------


class TestRigidBodyShapeTensors:

    def _load(self, sdk):
        return attach_usd_with_ovstage(sdk, data_path("simple_physics_scene.usda"))

    def _rows(self, sdk, stage, attribute):
        return _shape_rows(sdk, stage, SimObjectType.RIGID_BODY, attribute, RB_PATTERN)

    def _write(self, sdk, stage, attribute, rows):
        return _write_shape_rows(sdk, stage, SimObjectType.RIGID_BODY, attribute, RB_PATTERN, rows)

    def test_material_properties_read(self, physx_sdk_cpu):
        """Material properties should be readable and contain finite values."""
        stage = self._load(physx_sdk_cpu)
        for attribute in MATERIAL_ATTRIBUTES:
            out = self._rows(physx_sdk_cpu, stage, attribute)
            assert out, attribute
            assert all(np.all(np.isfinite(row)) for row in out.values()), attribute

    def test_contact_offset_read(self, physx_sdk_cpu):
        """Contact offsets should be readable and contain finite values."""
        stage = self._load(physx_sdk_cpu)
        out = self._rows(physx_sdk_cpu, stage, "contactOffset")
        assert out
        assert all(np.all(np.isfinite(row)) for row in out.values())

    def test_rest_offset_read(self, physx_sdk_cpu):
        """Rest offsets should be readable and contain finite values."""
        stage = self._load(physx_sdk_cpu)
        out = self._rows(physx_sdk_cpu, stage, "restOffset")
        assert out
        assert all(np.all(np.isfinite(row)) for row in out.values())

    def test_material_properties_write_roundtrip(self, physx_sdk_cpu):
        stage = self._load(physx_sdk_cpu)
        for attribute, value in zip(MATERIAL_ATTRIBUTES, (0.7, 0.5, 0.3)):
            original = self._rows(physx_sdk_cpu, stage, attribute)
            modified = {path: np.full_like(row, value) for path, row in original.items()}
            self._write(physx_sdk_cpu, stage, attribute, modified)
            _assert_shape_rows(self._rows(physx_sdk_cpu, stage, attribute), modified)

    def test_contact_offset_write_roundtrip(self, physx_sdk_cpu):
        stage = self._load(physx_sdk_cpu)
        original = self._rows(physx_sdk_cpu, stage, "contactOffset")
        modified = {path: row + 0.01 for path, row in original.items()}
        self._write(physx_sdk_cpu, stage, "contactOffset", modified)
        _assert_shape_rows(self._rows(physx_sdk_cpu, stage, "contactOffset"), modified)

    def test_rest_offset_write_roundtrip(self, physx_sdk_cpu):
        stage = self._load(physx_sdk_cpu)
        original = self._rows(physx_sdk_cpu, stage, "restOffset")
        modified = {path: row + 0.005 for path, row in original.items()}
        self._write(physx_sdk_cpu, stage, "restOffset", modified)
        _assert_shape_rows(self._rows(physx_sdk_cpu, stage, "restOffset"), modified)


# ---------------------------------------------------------------------------
# Articulation shape-level tensors
# ---------------------------------------------------------------------------


class TestArticulationShapeTensors:

    def _load(self, sdk):
        stage = attach_usd_with_ovstage(sdk, data_path("two_articulations.usda"))
        # Articulations are served once the first simulate has inserted them; warmup is that 1 ns step.
        sdk.warmup()
        return stage

    # A whole-articulation row holds every shape of the articulation but is read-only; per-shape
    # writes go through the links.
    def _link_rows(self, sdk, stage, attribute):
        return _shape_rows(sdk, stage, SimObjectType.ARTICULATION_LINK, attribute, ARTI_LINK_PATTERN)

    def _write_links(self, sdk, stage, attribute, rows):
        return _write_shape_rows(sdk, stage, SimObjectType.ARTICULATION_LINK, attribute, ARTI_LINK_PATTERN, rows)

    def test_material_properties_read(self, physx_sdk_cpu):
        """Articulation material properties should be readable and finite."""
        stage = self._load(physx_sdk_cpu)
        for attribute in MATERIAL_ATTRIBUTES:
            out = _shape_rows(physx_sdk_cpu, stage, SimObjectType.ARTICULATION, attribute, ARTI_PATTERN)
            assert out, attribute
            assert all(np.all(np.isfinite(row)) for row in out.values()), attribute

    def test_material_properties_write_roundtrip(self, physx_sdk_cpu):
        stage = self._load(physx_sdk_cpu)
        for attribute, value in zip(MATERIAL_ATTRIBUTES, (0.6, 0.4, 0.2)):
            original = self._link_rows(physx_sdk_cpu, stage, attribute)
            modified = {path: np.full_like(row, value) for path, row in original.items()}
            self._write_links(physx_sdk_cpu, stage, attribute, modified)
            _assert_shape_rows(self._link_rows(physx_sdk_cpu, stage, attribute), modified)

    def test_contact_offset_write_roundtrip(self, physx_sdk_cpu):
        stage = self._load(physx_sdk_cpu)
        original = self._link_rows(physx_sdk_cpu, stage, "contactOffset")
        modified = {path: row + 0.01 for path, row in original.items()}
        self._write_links(physx_sdk_cpu, stage, "contactOffset", modified)
        _assert_shape_rows(self._link_rows(physx_sdk_cpu, stage, "contactOffset"), modified)

    def test_rest_offset_write_roundtrip(self, physx_sdk_cpu):
        stage = self._load(physx_sdk_cpu)
        original = self._link_rows(physx_sdk_cpu, stage, "restOffset")
        modified = {path: row + 0.005 for path, row in original.items()}
        self._write_links(physx_sdk_cpu, stage, "restOffset", modified)
        _assert_shape_rows(self._link_rows(physx_sdk_cpu, stage, "restOffset"), modified)
