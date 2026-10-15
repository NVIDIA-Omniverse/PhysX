# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-WRITE-001
# @covers AC-9
# @maps_to TEST-CAPI-WRITE-001

"""Session read/write GPU DirectGPU coverage for rigid-body and articulation-root attributes."""

import numpy as np
from ovphysx.types import SimObjectType
from session_helpers import read_column, write_column, write_is_refused, write_uniform
from test_utils import data_path
from test_utils import attach_usd_with_ovstage


_RB_PATTERN = "/World/Cube*"
_ARTI_PATTERN = "/World/articulation*"


def _load_rb(sdk, n_steps=3):
    stage = attach_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
    sdk.wait_all()
    sdk.warmup()
    for _ in range(n_steps):
        sdk.step_sync(1.0 / 60.0)
    return stage


def _load_artic(sdk, n_steps=3):
    stage = attach_usd_with_ovstage(sdk, data_path("two_articulations.usda"))
    sdk.wait_all()
    sdk.warmup()
    for _ in range(n_steps):
        sdk.step_sync(1.0 / 60.0)
    return stage


def _read_rb(sdk, stage, attribute):
    return read_column(sdk, stage, SimObjectType.RIGID_BODY, attribute, pattern=_RB_PATTERN)


# ---------------------------------------------------------------------------
# Rigid body properties: GPU roundtrip
# ---------------------------------------------------------------------------


def test_rigid_body_mass_gpu_roundtrip(physx_sdk):
    """mass: write new mass values on GPU, read back, values match."""
    stage = _load_rb(physx_sdk)
    baseline = _read_rb(physx_sdk, stage, "mass")
    assert baseline, "mass read returned no rows"

    assert all(row[0] > 0 for row in baseline.values()), "Mass should be positive"

    new_mass = {path: row * 2.0 for path, row in baseline.items()}
    write_column(physx_sdk, stage, SimObjectType.RIGID_BODY, "mass", lambda path, _row: new_mass[path],
                 pattern=_RB_PATTERN)

    result = _read_rb(physx_sdk, stage, "mass")
    assert set(result) == set(new_mass)
    for path, row in result.items():
        np.testing.assert_allclose(row, new_mass[path], rtol=1e-4)


def test_rigid_body_inertia_gpu_roundtrip(physx_sdk):
    """inertia: write the inertia matrix on GPU, read back."""
    stage = _load_rb(physx_sdk)
    baseline = _read_rb(physx_sdk, stage, "inertia")
    assert baseline, "inertia read returned no rows"

    new_inertia = {}
    for path, row in baseline.items():
        new_inertia[path] = row.copy()
        new_inertia[path][[0, 4, 8]] *= 2.0
    write_column(physx_sdk, stage, SimObjectType.RIGID_BODY, "inertia", lambda path, _row: new_inertia[path],
                 pattern=_RB_PATTERN)

    result = _read_rb(physx_sdk, stage, "inertia")
    assert set(result) == set(new_inertia)
    for path, row in result.items():
        np.testing.assert_allclose(row, new_inertia[path], rtol=1e-4)


def test_rigid_body_com_pose_gpu_roundtrip(physx_sdk):
    """centerOfMassPosition / centerOfMassOrientation: write the COM local pose on GPU, read back."""
    stage = _load_rb(physx_sdk)
    assert _read_rb(physx_sdk, stage, "centerOfMassOrientation"), "centerOfMassOrientation read returned no rows"

    write_uniform(physx_sdk, stage, SimObjectType.RIGID_BODY, "centerOfMassPosition", [0.0, 0.0, 0.0],
                  pattern=_RB_PATTERN)
    new_orientation = [0.0, 0.0, 0.0, 1.0]  # qw = 1
    write_uniform(physx_sdk, stage, SimObjectType.RIGID_BODY, "centerOfMassOrientation", new_orientation,
                  pattern=_RB_PATTERN)

    result = _read_rb(physx_sdk, stage, "centerOfMassOrientation")
    for row in result.values():
        np.testing.assert_allclose(row, new_orientation, atol=1e-4)


# Covers test_rigid_body_com_pose_gpu_roundtrip's orientation check with a value unlike the cubes' identity baseline.
def test_rigid_body_com_orientation_gpu_write_lands(physx_sdk):
    """A non-identity centerOfMassOrientation written on GPU reads back as written."""
    stage = _load_rb(physx_sdk)
    s = 2.0 ** -0.5
    rotated = [0.0, 0.0, s, s]  # 90 degrees about +z
    written = write_uniform(physx_sdk, stage, SimObjectType.RIGID_BODY, "centerOfMassOrientation", rotated,
                            pattern=_RB_PATTERN)
    assert written > 0

    result = _read_rb(physx_sdk, stage, "centerOfMassOrientation")
    assert len(result) == written
    for row in result.values():
        np.testing.assert_allclose(row, rotated, atol=1e-4)


# ---------------------------------------------------------------------------
# Articulation root: GPU roundtrip
# ---------------------------------------------------------------------------


def test_articulation_root_pose_gpu_roundtrip(physx_sdk):
    """rootPosition / rootOrientation: write root pose on GPU, step, read back."""
    stage = _load_artic(physx_sdk)
    written = write_uniform(physx_sdk, stage, SimObjectType.ARTICULATION, "rootPosition", [0.0, 0.0, 1.0],
                            pattern=_ARTI_PATTERN)  # pz = 1
    assert written, "rootPosition write affected no articulations"
    write_uniform(physx_sdk, stage, SimObjectType.ARTICULATION, "rootOrientation", [0.0, 0.0, 0.0, 1.0],
                  pattern=_ARTI_PATTERN)  # qw = 1

    physx_sdk.step_sync(1.0 / 60.0)

    positions = read_column(physx_sdk, stage, SimObjectType.ARTICULATION, "rootPosition", pattern=_ARTI_PATTERN)
    orientations = read_column(physx_sdk, stage, SimObjectType.ARTICULATION, "rootOrientation", pattern=_ARTI_PATTERN)

    assert len(positions) == written and set(orientations) == set(positions)
    assert all(row.size == 3 for row in positions.values())
    assert all(row.size == 4 for row in orientations.values())


def test_articulation_root_velocity_gpu_roundtrip(physx_sdk):
    """rootLinearVelocity / rootAngularVelocity: write root velocity on GPU, step, read back."""
    stage = _load_artic(physx_sdk)
    written = write_uniform(physx_sdk, stage, SimObjectType.ARTICULATION, "rootLinearVelocity", [0.0, 0.0, 0.0],
                            pattern=_ARTI_PATTERN)
    assert written, "rootLinearVelocity write affected no articulations"
    write_uniform(physx_sdk, stage, SimObjectType.ARTICULATION, "rootAngularVelocity", [0.0, 0.0, 0.0],
                  pattern=_ARTI_PATTERN)

    physx_sdk.step_sync(1.0 / 60.0)

    linear = read_column(physx_sdk, stage, SimObjectType.ARTICULATION, "rootLinearVelocity", pattern=_ARTI_PATTERN)
    angular = read_column(physx_sdk, stage, SimObjectType.ARTICULATION, "rootAngularVelocity", pattern=_ARTI_PATTERN)
    assert len(linear) == written and set(angular) == set(linear)
    assert all(row.size == 3 for row in linear.values())
    assert all(row.size == 3 for row in angular.values())


# ---------------------------------------------------------------------------
# Write-only attribute read enforcement on GPU
# ---------------------------------------------------------------------------


def test_write_only_rigid_body_force_read_raises_gpu(physx_sdk):
    """force (write-only): a read carries no force column on GPU; the read skips names a type does not emit."""
    stage = _load_rb(physx_sdk)
    assert _read_rb(physx_sdk, stage, "position"), "position read returned no rows"
    assert _read_rb(physx_sdk, stage, "force") == {}


def test_write_only_rigid_body_wrench_read_raises_gpu(physx_sdk):
    """wrench (write-only): a read carries no wrench column on GPU; the read skips names a type does not emit."""
    stage = _load_rb(physx_sdk)
    assert _read_rb(physx_sdk, stage, "position"), "position read returned no rows"
    assert _read_rb(physx_sdk, stage, "wrench") == {}


# ---------------------------------------------------------------------------
# Read-only attribute write enforcement on GPU
# ---------------------------------------------------------------------------


def test_read_only_link_pose_write_raises_gpu(physx_sdk):
    """Articulation-link position / orientation (read-only): a write is refused on GPU."""
    stage = _load_artic(physx_sdk)
    links = read_column(physx_sdk, stage, SimObjectType.ARTICULATION_LINK, "position", pattern=_ARTI_PATTERN)
    assert links, "articulation-link position read returned no rows"
    for attribute in ("position", "orientation"):
        assert write_is_refused(physx_sdk, SimObjectType.ARTICULATION_LINK, attribute, pattern=_ARTI_PATTERN), attribute


def test_read_only_link_velocity_write_raises_gpu(physx_sdk):
    """Articulation-link linearVelocity / angularVelocity (read-only): a write is refused on GPU."""
    stage = _load_artic(physx_sdk)
    links = read_column(physx_sdk, stage, SimObjectType.ARTICULATION_LINK, "linearVelocity", pattern=_ARTI_PATTERN)
    assert links, "articulation-link linearVelocity read returned no rows"
    for attribute in ("linearVelocity", "angularVelocity"):
        assert write_is_refused(physx_sdk, SimObjectType.ARTICULATION_LINK, attribute, pattern=_ARTI_PATTERN), attribute
