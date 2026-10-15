# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-WRITE-001
# @covers AC-9
# @maps_to TEST-CAPI-WRITE-001
# @implements REQ-CAPI-SELECT-001
# @covers AC-5 AC-6
# @maps_to TEST-CAPI-SELECT-001

"""Session read/write GPU DirectGPU coverage for deformable bodies and materials."""

import numpy as np
from ovphysx.types import SimObjectType
from session_helpers import handles_for, read_column, write_column, write_is_refused, write_uniform
from test_utils import attach_usd_with_ovstage, data_path


_BODY_PATTERN = "/World/DeformableBody"
_MAT_PATTERN = "/World/DeformableMaterial"
_MULTI_BODY_PATTERN = "/World/DeformableBody_*"
_MULTI_MAT_PATTERN = "/World/DeformableMaterial_*"

_VOLUME = SimObjectType.DEFORMABLE_VOLUME
_SURFACE = SimObjectType.DEFORMABLE_SURFACE
_MATERIAL = SimObjectType.DEFORMABLE_MATERIAL


def _load_deformable(sdk, name="volume_deformable_simple.usda"):
    stage = attach_usd_with_ovstage(sdk, data_path(name))
    sdk.wait_all()
    sdk.warmup()
    return stage


def _replace(values):
    """A write_column fill that replaces every selected row with `values`."""
    return lambda _path, _row: values


def _read_material(sdk, stage, attribute, pattern=_MAT_PATTERN):
    """The single material `pattern` selects: its `attribute` row."""
    values = read_column(sdk, stage, _MATERIAL, attribute, pattern=pattern)
    assert list(values) == [pattern], attribute
    return values[pattern]


def test_deformable_material_cpu_importable_without_gpu(physx_sdk):
    """Deformable material properties are CPU-resident and readable without DirectGPU."""
    stage = _load_deformable(physx_sdk)

    np.testing.assert_allclose(_read_material(physx_sdk, stage, "deformableDynamicFriction"), [0.5], atol=1.0e-4)


def test_deformable_body_gpu_read_write_and_metadata(physx_sdk):
    stage = _load_deformable(physx_sdk)

    def read(attribute):
        values = read_column(physx_sdk, stage, _VOLUME, attribute, pattern=_BODY_PATTERN)
        assert list(values) == [_BODY_PATTERN], attribute
        return values[_BODY_PATTERN]

    def write(attribute, values):
        assert write_column(physx_sdk, stage, _VOLUME, attribute, _replace(values), pattern=_BODY_PATTERN) == 1

    # One body: 5 sim nodes (vec3 each) and 2 tetrahedra.
    positions = read("points")
    assert positions.size == 5 * 3
    np.testing.assert_allclose(positions[:3], [0.0, 0.0, 0.0], atol=1.0e-4)

    velocities = read("velocities")
    assert velocities.size == positions.size
    np.testing.assert_allclose(velocities, 0.0, atol=1.0e-4)

    rest_positions = read("restPoints")
    np.testing.assert_allclose(rest_positions, positions, atol=1.0e-4)

    np.testing.assert_array_equal(read("simElementIndices"), [0, 1, 2, 3, 1, 2, 3, 4])

    new_positions = positions.reshape(5, 3).copy()
    new_positions[:, 1] += 0.05
    new_positions = new_positions.reshape(-1)
    write("points", new_positions)
    np.testing.assert_allclose(read("points"), new_positions, atol=1.0e-4)

    new_velocities = np.zeros((5, 3))
    new_velocities[:, 0] = 0.25
    new_velocities = new_velocities.reshape(-1)
    write("velocities", new_velocities)
    np.testing.assert_allclose(read("velocities"), new_velocities, atol=1.0e-4)

    # One (x, y, z, w) target per node, where w = 0 drives the node. The read serves the targets of a
    # driven body only, so every node is driven here.
    new_targets = np.zeros((5, 4))
    new_targets[:, :3] = new_positions.reshape(5, 3) + np.array([0.0, 0.03, 0.0])
    new_targets = new_targets.reshape(-1)

    def fill_targets(_path, row):
        assert row.size == new_targets.size
        return new_targets

    assert write_column(physx_sdk, stage, _VOLUME, "kinematicTarget", fill_targets, pattern=_BODY_PATTERN) == 1
    np.testing.assert_allclose(read("kinematicTarget"), new_targets, atol=1.0e-4)

    assert write_is_refused(physx_sdk, _VOLUME, "restPoints", pattern=_BODY_PATTERN), "restPoints is read-only"
    assert write_is_refused(physx_sdk, _VOLUME, "simElementIndices", pattern=_BODY_PATTERN), \
        "simElementIndices is read-only"

    selected_positions = new_positions.reshape(5, 3).copy()
    selected_positions[:, 2] += 0.025
    selected_positions = selected_positions.reshape(-1)
    write("points", selected_positions)
    np.testing.assert_allclose(read("points"), selected_positions, atol=1.0e-4)


# Session counterpart of the authored reads and write round trips in
# test_deformable_material_cpu_read_indexed_and_masked_write.
def test_deformable_material_session_read_and_write(physx_sdk):
    stage = _load_deformable(physx_sdk)

    def read(attribute):
        return _read_material(physx_sdk, stage, attribute)

    def write(attribute, value):
        assert write_uniform(physx_sdk, stage, _MATERIAL, attribute, [value], pattern=_MAT_PATTERN) == 1

    np.testing.assert_allclose(read("deformableDynamicFriction"), [0.5], atol=1.0e-4)
    np.testing.assert_allclose(read("deformableYoungsModulus"), [1000.0], atol=1.0e-3)
    np.testing.assert_allclose(read("deformablePoissonsRatio"), [0.3], atol=1.0e-4)

    write("deformableDynamicFriction", 0.6)
    np.testing.assert_allclose(read("deformableDynamicFriction"), [0.6], atol=1.0e-4)

    write("deformableYoungsModulus", 1200.0)
    np.testing.assert_allclose(read("deformableYoungsModulus"), [1200.0], atol=1.0e-3)

    write("deformablePoissonsRatio", 0.25)
    np.testing.assert_allclose(read("deformablePoissonsRatio"), [0.25], atol=1.0e-4)


# Session counterpart of test_deformable_body_indexed_write_selects_requested_row's check: a write selected
# by one body changes that body only.
def test_deformable_body_selected_write_changes_only_the_selection(physx_sdk):
    stage = _load_deformable(physx_sdk, "volume_deformable_multi.usda")
    body0, body1 = "/World/DeformableBody_0", "/World/DeformableBody_1"

    def read():
        return read_column(physx_sdk, stage, _VOLUME, "points", pattern=_MULTI_BODY_PATTERN)

    positions = read()
    assert sorted(positions) == [body0, body1]
    assert all(row.size == 5 * 3 for row in positions.values())
    np.testing.assert_allclose(positions[body0][:3], [0.0, 0.0, 0.0], atol=1.0e-4)
    np.testing.assert_allclose(positions[body1][:3], [2.0, 0.0, 0.0], atol=1.0e-4)

    updated = positions[body1].reshape(5, 3).copy()
    updated[:, 1] += 0.125
    updated = updated.reshape(-1)
    written = write_column(physx_sdk, stage, _VOLUME, "points", _replace(updated),
                           prim_paths=handles_for(stage, [body1]))
    assert written == 1

    readback = read()
    np.testing.assert_allclose(readback[body0], positions[body0], atol=1.0e-4)
    np.testing.assert_allclose(readback[body1], updated, atol=1.0e-4)


# Session counterpart of test_deformable_material_partial_mask_compacts_selection's check: a write selected
# by one material changes that material only.
def test_deformable_material_selected_write_changes_only_the_selection(physx_sdk):
    stage = _load_deformable(physx_sdk, "volume_deformable_multi.usda")
    mat0, mat1 = "/World/DeformableMaterial_0", "/World/DeformableMaterial_1"

    def read():
        values = read_column(physx_sdk, stage, _MATERIAL, "deformablePoissonsRatio", pattern=_MULTI_MAT_PATTERN)
        assert sorted(values) == [mat0, mat1]
        return [values[mat0][0], values[mat1][0]]

    np.testing.assert_allclose(read(), [0.3, 0.35], atol=1.0e-4)

    # The selection names only material 0, so material 1 must retain its authored value.
    written = write_uniform(physx_sdk, stage, _MATERIAL, "deformablePoissonsRatio", [0.22],
                            prim_paths=handles_for(stage, [mat0]))
    assert written == 1
    np.testing.assert_allclose(read(), [0.22, 0.35], atol=1.0e-4)


def test_volume_deformable_collision_element_indices_read(physx_sdk):
    stage = _load_deformable(physx_sdk)

    collision = read_column(physx_sdk, stage, _VOLUME, "collisionElementIndices", pattern=_BODY_PATTERN)
    assert list(collision) == [_BODY_PATTERN]
    indices = collision[_BODY_PATTERN]
    assert indices.size > 0  # getNumNodesPerElement() per element, 4 for a volume tetmesh
    assert np.all(indices >= 0), f"Negative index in collision mesh: {indices}"

    assert write_is_refused(physx_sdk, _VOLUME, "collisionElementIndices", pattern=_BODY_PATTERN), \
        "collisionElementIndices is read-only"


def test_surface_deformable_body_gpu_read_write_and_metadata(physx_sdk):
    stage = _load_deformable(physx_sdk, "surface_deformable_simple.usda")

    def read(attribute):
        values = read_column(physx_sdk, stage, _SURFACE, attribute, pattern=_BODY_PATTERN)
        assert list(values) == [_BODY_PATTERN], attribute
        return values[_BODY_PATTERN]

    def write(attribute, values):
        assert write_column(physx_sdk, stage, _SURFACE, attribute, _replace(values), pattern=_BODY_PATTERN) == 1

    # One body: 4 nodes (square cloth), 2 triangles
    positions = read("points")
    assert positions.size == 4 * 3
    np.testing.assert_allclose(positions[:3], [0.0, 0.0, 0.0], atol=1.0e-4)

    velocities = read("velocities")
    assert velocities.size == positions.size
    np.testing.assert_allclose(velocities, 0.0, atol=1.0e-4)

    rest_positions = read("restPoints")
    assert rest_positions.size == 4 * 3
    np.testing.assert_allclose(rest_positions, positions, atol=1.0e-4)

    element_indices = read("simElementIndices")
    assert element_indices.size == 2 * 3   # K=3 trimesh
    assert np.all(element_indices >= 0)

    new_positions = positions.reshape(4, 3).copy()
    new_positions[:, 1] += 0.05
    new_positions = new_positions.reshape(-1)
    write("points", new_positions)
    np.testing.assert_allclose(read("points"), new_positions, atol=1.0e-4)

    new_velocities = np.zeros((4, 3))
    new_velocities[:, 0] = 0.1
    new_velocities = new_velocities.reshape(-1)
    write("velocities", new_velocities)
    np.testing.assert_allclose(read("velocities"), new_velocities, atol=1.0e-4)

    assert write_is_refused(physx_sdk, _SURFACE, "restPoints", pattern=_BODY_PATTERN), "restPoints is read-only"
    assert write_is_refused(physx_sdk, _SURFACE, "simElementIndices", pattern=_BODY_PATTERN), \
        "simElementIndices is read-only"


def test_deformable_material_elasticity_damping_read_write(physx_sdk):
    stage = _load_deformable(physx_sdk)

    def write(value):
        assert write_uniform(physx_sdk, stage, _MATERIAL, "deformableElasticityDamping", [value],
                             pattern=_MAT_PATTERN) == 1

    np.testing.assert_allclose(_read_material(physx_sdk, stage, "deformableElasticityDamping"), [0.01], atol=1.0e-4)

    write(0.05)
    np.testing.assert_allclose(_read_material(physx_sdk, stage, "deformableElasticityDamping"), [0.05], atol=1.0e-4)

    write(0.03)
    np.testing.assert_allclose(_read_material(physx_sdk, stage, "deformableElasticityDamping"), [0.03], atol=1.0e-4)


def test_surface_deformable_material_bending_properties_read_write(physx_sdk):
    stage = attach_usd_with_ovstage(physx_sdk, data_path("surface_deformable_material.usda"))
    physx_sdk.wait_all()
    material = "/World/SurfaceDeformableMaterial"

    def read(attribute):
        return _read_material(physx_sdk, stage, attribute, pattern=material)

    def write(attribute, value):
        assert write_uniform(physx_sdk, stage, _MATERIAL, attribute, [value], pattern=material) == 1

    np.testing.assert_allclose(read("deformableBendingStiffness"), [100.0], atol=0.1)
    np.testing.assert_allclose(read("deformableThickness"), [0.01], atol=1.0e-4)
    np.testing.assert_allclose(read("deformableBendingDamping"), [0.05], atol=1.0e-4)

    write("deformableBendingStiffness", 200.0)
    np.testing.assert_allclose(read("deformableBendingStiffness"), [200.0], atol=0.1)

    write("deformableThickness", 0.02)
    np.testing.assert_allclose(read("deformableThickness"), [0.02], atol=1.0e-4)
