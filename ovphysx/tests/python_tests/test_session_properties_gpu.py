# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-WRITE-001
# @covers AC-9
# @maps_to TEST-CAPI-WRITE-001
# @implements REQ-CAPI-SELECT-001
# @covers AC-5 AC-6 AC-7
# @maps_to TEST-CAPI-SELECT-001
# @implements REQ-READ-ATTRS-001
# @covers AC-8 AC-16 AC-22
# @maps_to TEST-READ-ATTRS-001
# @implements REQ-READ-INVDYN-001
# @covers AC-1 AC-6
# @maps_to TEST-READ-INVDYN-001
# @implements REQ-TENSOR-FK-001
# @covers AC-2 AC-3
# @maps_to TEST-TENSOR-FK-001

"""GPU-mode session read/write tests for DOF, body, body-inverse, inverse-dynamics, link-wrench, tendon, shape, metadata and kinematic properties (DirectGPU)."""

import ctypes
import math
from pathlib import Path

import numpy as np
import pytest
from ovphysx.types import LogLevel, SimObjectType
from session_helpers import handles_for, read_column, write_column, write_is_refused, write_uniform
from test_utils import attach_usd_with_ovstage, data_path, load_usd_with_ovstage


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

ARTI_PATTERN = "/World/articulation*"
# two_articulations.usda holds only the two articulations ARTI_PATTERN names, so a session read or write
# with no selection covers exactly them. FIRST_ARTI + "/*" selects the first one's links and joints.
FIRST_ARTI = "/World/articulation"
DT = 1.0 / 60.0


def _load_and_step(sdk, scene="two_articulations.usda", n_steps=5):
    stage = attach_usd_with_ovstage(sdk, data_path(scene))
    sdk.wait_all()
    sdk.warmup()
    for _ in range(n_steps):
        sdk.step(DT)
    sdk.wait_all()
    return stage


def _values(columns):
    """Every row of a read_column result, concatenated in prim-path order."""
    assert columns, "the read returned no rows"
    return np.concatenate([columns[path] for path in sorted(columns)])


def _in_first_arti(path):
    return path.startswith(FIRST_ARTI + "/")


def _check_first_arti_write(original, readback, value):
    """After a write of `value` selecting FIRST_ARTI: its rows hold `value`, the other articulation's are unchanged."""
    assert any(map(_in_first_arti, original)) and not all(map(_in_first_arti, original))
    assert readback.keys() == original.keys()
    for path, row in readback.items():
        if _in_first_arti(path):
            np.testing.assert_allclose(row, value, atol=1e-5, err_msg=f"selected {path} should be updated")
        else:
            np.testing.assert_allclose(row, original[path], atol=1e-5, err_msg=f"unselected {path} should be unchanged")


def _make_usd_with_authored_dof_max_velocity(tmp_path, max_velocity_rad=5.0):
    src = Path(data_path("two_articulations.usda"))
    text = src.read_text()
    text = text.replace(
        'apiSchemas = ["PhysicsDriveAPI:angular"]', 'apiSchemas = ["PhysicsDriveAPI:angular", "PhysxJointAxisAPI:angular"]'
    )
    max_velocity_deg = max_velocity_rad * 180.0 / math.pi
    text = text.replace(
        'uniform token physics:axis = "Y"',
        f"float physxJointAxis:angular:maxJointVelocity = {max_velocity_deg:.9f}\n" '            uniform token physics:axis = "Y"',
    )
    dst = tmp_path / "two_articulations_authored_max_velocity.usda"
    dst.write_text(text)
    return dst


# ---------------------------------------------------------------------------
# Articulation kinematic update
# ---------------------------------------------------------------------------


class TestArticulationKinematicUpdateGpu:

    def test_first_fk_update_auto_warmups_gpu(self, physx_sdk, native_log_callback_factory):
        """First GPU FK refresh should initialize DirectGPU buffers itself."""
        from ovphysx._bindings import _lib

        load_usd_with_ovstage(physx_sdk, data_path("two_articulations.usda"))
        physx_sdk.wait_all()

        records = []

        @native_log_callback_factory
        def collector(level, message, channel, timestamp, user_data):
            text = ctypes.string_at(message.ptr, message.length).decode("utf-8", errors="replace")
            records.append((level, text))

        result = _lib.ovphysx_set_log_callback(
            LogLevel.VERBOSE, None, ctypes.cast(collector, ctypes.c_void_p), None
        )
        assert result.status == 0, "Failed to register native log callback"

        try:
            physx_sdk.update_articulations_kinematic()
        finally:
            _lib.ovphysx_set_log_callback(LogLevel.DEFAULT, None, None, None)

        direct_gpu_errors = [
            text
            for _, text in records
            if "computeArticulationData" in text or "DirectGPU API has not been initialized" in text
        ]
        assert not direct_gpu_errors, "\n".join(direct_gpu_errors)

    def test_dof_position_write_updates_link_pose_after_fk(self, physx_sdk):
        """DOF writes followed by explicit FK refresh link poses without stepping."""
        stage = _load_and_step(physx_sdk, n_steps=1)

        before = read_column(physx_sdk, stage, SimObjectType.ARTICULATION_LINK, "position")
        # 0.5 rad on every DOF; the session takes angular joint positions in degrees.
        write_uniform(physx_sdk, stage, SimObjectType.ARTICULATION_JOINT, "jointPosition", [math.degrees(0.5)])

        physx_sdk.update_articulations_kinematic()
        after = read_column(physx_sdk, stage, SimObjectType.ARTICULATION_LINK, "position")

        assert not np.allclose(
            _values(after), _values(before), atol=1e-4
        ), "Link positions must reflect the written DOF positions after FK update"


# ---------------------------------------------------------------------------
# Standalone rigid-body GPU state tensors
# ---------------------------------------------------------------------------


class TestRigidBodyStateGpu:

    # Covers test_reported_cpu_only_types_are_native_cpu: these authored properties read back host-resident.
    @pytest.mark.parametrize(
        ("scene", "object_type", "attribute"),
        [
            ("boxes_falling_on_groundplane.usda", SimObjectType.RIGID_BODY, "contactOffset"),
            ("boxes_falling_on_groundplane.usda", SimObjectType.RIGID_BODY, "restOffset"),
            ("boxes_falling_on_groundplane.usda", SimObjectType.RIGID_BODY, "disableGravity"),
            ("two_articulations.usda", SimObjectType.ARTICULATION_LINK, "contactOffset"),
            ("two_articulations.usda", SimObjectType.ARTICULATION_LINK, "restOffset"),
            ("two_articulations.usda", SimObjectType.ARTICULATION_JOINT, "jointSpeedEffortGradient"),
            ("two_articulations.usda", SimObjectType.ARTICULATION_JOINT, "jointMaxActuatorVelocity"),
            ("two_articulations.usda", SimObjectType.ARTICULATION_JOINT, "jointVelocityDependentResistance"),
            ("two_articulations.usda", SimObjectType.ARTICULATION_JOINT, "jointDriveType"),
            ("two_articulations.usda", SimObjectType.ARTICULATION_LINK, "disableGravity"),
        ],
    )
    def test_property_columns_are_host_resident(self, physx_sdk, scene, object_type, attribute):
        # A DirectGPU read needs a first step.
        _load_and_step(physx_sdk, scene=scene, n_steps=0)

        with physx_sdk.read_sync(object_type, [attribute]) as result:
            tensors = [tensor for group in result.groups for tensor in group.tensors]
            assert tensors, f"{attribute} produced no column"
            assert all(tensor.device.is_cpu for tensor in tensors), f"{attribute} must be host-resident"

    def test_acceleration_readable_on_gpu(self, physx_sdk):
        stage = _load_and_step(physx_sdk, scene="boxes_falling_on_groundplane.usda")
        for attribute in ("linearAcceleration", "angularAcceleration"):
            rows = read_column(physx_sdk, stage, SimObjectType.RIGID_BODY, attribute, pattern="/World/Cube*")
            assert all(row.size == 3 for row in rows.values())
            assert np.all(np.isfinite(_values(rows)))


# ---------------------------------------------------------------------------
# Shape properties: host-resident per-shape columns
# ---------------------------------------------------------------------------


class TestShapePropertiesGpu:

    # (session attributes, name): the material spans three columns.
    _SHAPE_PROPERTIES = [
        (("staticFriction", "dynamicFriction", "restitution"), "material"),
        (("contactOffset",), "contact_offset"),
        (("restOffset",), "rest_offset"),
    ]

    _MATERIAL_VALUES = {"staticFriction": 0.7, "dynamicFriction": 0.5, "restitution": 0.3}

    @classmethod
    def _modify_shape_property(cls, original: np.ndarray, attribute: str) -> np.ndarray:
        if attribute in cls._MATERIAL_VALUES:
            return np.full_like(original, cls._MATERIAL_VALUES[attribute])
        if attribute == "contactOffset":
            return original + 0.01
        return original + 0.005

    @classmethod
    def _roundtrip(cls, sdk, stage, object_type, attributes, label, pattern=None):
        for attribute in attributes:
            original = read_column(sdk, stage, object_type, attribute, pattern=pattern)
            assert np.all(np.isfinite(_values(original))), f"{attribute} values must be finite"
            modified = {path: cls._modify_shape_property(row, attribute) for path, row in original.items()}
            write_column(sdk, stage, object_type, attribute, lambda path, _row: modified[path], pattern=pattern)
            readback = read_column(sdk, stage, object_type, attribute, pattern=pattern)
            np.testing.assert_allclose(
                _values(readback), _values(modified), atol=1e-5, err_msg=f"{label} {attribute} roundtrip failed"
            )

    @pytest.mark.parametrize("attributes,name", _SHAPE_PROPERTIES, ids=[n for _, n in _SHAPE_PROPERTIES])
    def test_rigid_shape_property_cpu_roundtrip_in_gpu_mode(self, physx_sdk, attributes, name):
        stage = _load_and_step(physx_sdk, scene="simple_physics_scene.usda")
        self._roundtrip(physx_sdk, stage, SimObjectType.RIGID_BODY, attributes, "Rigid-body", pattern="/World/Cube*")

    @pytest.mark.parametrize("attributes,name", _SHAPE_PROPERTIES, ids=[n for _, n in _SHAPE_PROPERTIES])
    def test_articulation_shape_property_cpu_roundtrip_in_gpu_mode(self, physx_sdk, attributes, name):
        stage = _load_and_step(physx_sdk)
        self._roundtrip(physx_sdk, stage, SimObjectType.ARTICULATION_LINK, attributes, "Articulation")


# ---------------------------------------------------------------------------
# DOF properties: host-resident joint property columns
# ---------------------------------------------------------------------------


class TestDofPropertiesGpu:

    _DOF_PROPERTIES = [
        ("jointStiffness", "stiffness"),
        ("jointDamping", "damping"),
        ("jointMaxVelocity", "max_velocity"),
        ("jointMaxForce", "max_force"),
        ("jointArmature", "armature"),
    ]

    @pytest.mark.parametrize("attribute,name", _DOF_PROPERTIES, ids=[n for _, n in _DOF_PROPERTIES])
    def test_read_finite(self, physx_sdk, attribute, name):
        stage = _load_and_step(physx_sdk)
        values = read_column(physx_sdk, stage, SimObjectType.ARTICULATION_JOINT, attribute)
        assert np.all(np.isfinite(_values(values))), f"{name} must be finite after stepping"

    @pytest.mark.parametrize("attribute,name", _DOF_PROPERTIES, ids=[n for _, n in _DOF_PROPERTIES])
    def test_write_roundtrip(self, physx_sdk, attribute, name):
        stage = _load_and_step(physx_sdk)
        joint = SimObjectType.ARTICULATION_JOINT
        original = read_column(physx_sdk, stage, joint, attribute)
        write_column(physx_sdk, stage, joint, attribute, lambda path, _row: original[path] + 1.0)
        readback = read_column(physx_sdk, stage, joint, attribute)
        np.testing.assert_allclose(
            _values(readback), _values(original) + 1.0, atol=1e-5, err_msg=f"{name} roundtrip failed"
        )

    def test_dof_limit_roundtrip(self, physx_sdk):
        stage = _load_and_step(physx_sdk)
        joint = SimObjectType.ARTICULATION_JOINT
        original = read_column(physx_sdk, stage, joint, "jointLimit")
        # One (lower, upper) interval per single-axis joint.
        assert all(row.size == 2 for row in original.values())
        write_uniform(physx_sdk, stage, joint, "jointLimit", [-99.0, 99.0])
        readback = read_column(physx_sdk, stage, joint, "jointLimit")
        np.testing.assert_allclose(_values(readback), np.tile([-99.0, 99.0], len(original)), atol=1e-5)

    def test_max_velocity_reads_physx_joint_usd_authoring(self, physx_sdk, tmp_path):
        usd_path = _make_usd_with_authored_dof_max_velocity(tmp_path, max_velocity_rad=5.0)
        stage = attach_usd_with_ovstage(physx_sdk, str(usd_path))
        physx_sdk.wait_all()
        physx_sdk.warmup()
        physx_sdk.wait_all()

        values = read_column(physx_sdk, stage, SimObjectType.ARTICULATION_JOINT, "jointMaxVelocity")
        # The session reports angular rates in degrees, as authored.
        np.testing.assert_allclose(_values(values), math.degrees(5.0), atol=1e-5)

    # Session counterpart of test_max_velocity_indexed_write_roundtrip: the same subset written through a selection.
    def test_max_velocity_selected_write_roundtrip(self, physx_sdk):
        stage = _load_and_step(physx_sdk)
        joint = SimObjectType.ARTICULATION_JOINT
        original = read_column(physx_sdk, stage, joint, "jointMaxVelocity")
        write_uniform(physx_sdk, stage, joint, "jointMaxVelocity", [123.0], pattern=FIRST_ARTI + "/*")
        readback = read_column(physx_sdk, stage, joint, "jointMaxVelocity")
        _check_first_arti_write(original, readback, 123.0)

    def test_friction_properties_column_isolation(self, physx_sdk):
        stage = _load_and_step(physx_sdk)
        joint = SimObjectType.ARTICULATION_JOINT
        dynamic = read_column(physx_sdk, stage, joint, "jointDynamicFriction")
        viscous = read_column(physx_sdk, stage, joint, "jointViscousFriction")
        write_uniform(physx_sdk, stage, joint, "jointStaticFriction", [42.0])
        np.testing.assert_allclose(
            _values(read_column(physx_sdk, stage, joint, "jointStaticFriction")), 42.0, atol=1e-5
        )
        np.testing.assert_allclose(
            _values(read_column(physx_sdk, stage, joint, "jointDynamicFriction")), _values(dynamic), atol=1e-5
        )
        np.testing.assert_allclose(
            _values(read_column(physx_sdk, stage, joint, "jointViscousFriction")), _values(viscous), atol=1e-5
        )

    # Session counterpart of test_masked_write: the same subset written through a selection.
    def test_selected_write(self, physx_sdk):
        stage = _load_and_step(physx_sdk)
        joint = SimObjectType.ARTICULATION_JOINT
        original = read_column(physx_sdk, stage, joint, "jointStiffness")
        write_uniform(physx_sdk, stage, joint, "jointStiffness", [999.0], pattern=FIRST_ARTI + "/*")
        readback = read_column(physx_sdk, stage, joint, "jointStiffness")
        _check_first_arti_write(original, readback, 999.0)


# ---------------------------------------------------------------------------
# Body properties: host-resident link property columns
# ---------------------------------------------------------------------------


class TestBodyPropertiesGpu:

    def test_body_mass_roundtrip(self, physx_sdk):
        stage = _load_and_step(physx_sdk)
        link = SimObjectType.ARTICULATION_LINK
        original = read_column(physx_sdk, stage, link, "mass")
        assert np.all(_values(original) > 0), "masses should be positive"
        write_column(physx_sdk, stage, link, "mass", lambda path, _row: original[path] * 2.0)
        readback = read_column(physx_sdk, stage, link, "mass")
        np.testing.assert_allclose(_values(readback), _values(original) * 2.0, atol=1e-5)

    def test_body_com_pose_roundtrip(self, physx_sdk):
        stage = _load_and_step(physx_sdk)
        link = SimObjectType.ARTICULATION_LINK
        position = read_column(physx_sdk, stage, link, "centerOfMassPosition")
        orientation = read_column(physx_sdk, stage, link, "centerOfMassOrientation")
        assert all(row.size == 3 for row in position.values()) and all(row.size == 4 for row in orientation.values())
        modified = {path: row + [0.01, 0.0, 0.0] for path, row in position.items()}
        write_column(physx_sdk, stage, link, "centerOfMassPosition", lambda path, _row: modified[path])
        np.testing.assert_allclose(
            _values(read_column(physx_sdk, stage, link, "centerOfMassPosition")), _values(modified), atol=1e-5
        )
        np.testing.assert_allclose(
            _values(read_column(physx_sdk, stage, link, "centerOfMassOrientation")), _values(orientation), atol=1e-5
        )

    def test_body_inertia_roundtrip(self, physx_sdk):
        stage = _load_and_step(physx_sdk)
        link = SimObjectType.ARTICULATION_LINK
        original = read_column(physx_sdk, stage, link, "inertia")
        assert all(row.size == 9 for row in original.values())
        modified = {path: row * [1.5, 1, 1, 1, 1, 1, 1, 1, 1] for path, row in original.items()}
        write_column(physx_sdk, stage, link, "inertia", lambda path, _row: modified[path])
        readback = read_column(physx_sdk, stage, link, "inertia")
        np.testing.assert_allclose(_values(readback), _values(modified), atol=1e-5)

    # Session counterpart of test_body_mass_indexed_write: the same subset written through a selection.
    def test_body_mass_selected_write(self, physx_sdk):
        stage = _load_and_step(physx_sdk)
        link = SimObjectType.ARTICULATION_LINK
        original = read_column(physx_sdk, stage, link, "mass")
        write_uniform(physx_sdk, stage, link, "mass", [123.0], pattern=FIRST_ARTI + "/*")
        readback = read_column(physx_sdk, stage, link, "mass")
        _check_first_arti_write(original, readback, 123.0)


# ---------------------------------------------------------------------------
# Body inverse mass/inertia: host-resident, read-only
# ---------------------------------------------------------------------------


class TestBodyInverseGpu:

    def test_inv_mass_shape_and_values(self, physx_sdk):
        stage = _load_and_step(physx_sdk)
        values = read_column(physx_sdk, stage, SimObjectType.ARTICULATION_LINK, "inverseMass")
        assert np.all(np.isfinite(_values(values)))

    def test_inv_inertia_shape(self, physx_sdk):
        stage = _load_and_step(physx_sdk)
        values = read_column(physx_sdk, stage, SimObjectType.ARTICULATION_LINK, "inverseInertia")
        assert all(row.size == 9 for row in values.values())
        assert np.all(np.isfinite(_values(values)))

    def test_inv_mass_consistency_with_mass(self, physx_sdk):
        """Verify inv_mass * mass ~= 1.0 (catches garbage return values)."""
        stage = _load_and_step(physx_sdk)
        mass = read_column(physx_sdk, stage, SimObjectType.ARTICULATION_LINK, "mass")
        inv_mass = read_column(physx_sdk, stage, SimObjectType.ARTICULATION_LINK, "inverseMass")
        assert mass.keys() == inv_mass.keys()
        product = _values(mass) * _values(inv_mass)
        np.testing.assert_allclose(product, 1.0, atol=1e-4, err_msg="inv_mass * mass should be ~1.0")

    def test_inv_mass_write_rejected(self, physx_sdk):
        _load_and_step(physx_sdk)
        assert write_is_refused(physx_sdk, SimObjectType.ARTICULATION_LINK, "inverseMass")


# ---------------------------------------------------------------------------
# Inverse dynamics: GPU state, read-only
# ---------------------------------------------------------------------------


class TestInverseDynamicsTensorsGpu:

    def test_jacobian_readable(self, physx_sdk):
        stage = _load_and_step(physx_sdk)
        jacobians = read_column(physx_sdk, stage, SimObjectType.ARTICULATION, "jacobian")
        shapes = read_column(physx_sdk, stage, SimObjectType.ARTICULATION, "jacobianShape")
        # Each articulation's Jacobian is a rows x cols matrix; jacobianShape holds (rows, cols).
        for path, jacobian in jacobians.items():
            rows, cols = shapes[path]
            assert jacobian.size == rows * cols
        assert np.all(np.isfinite(_values(jacobians)))

    def test_mass_matrix_symmetric(self, physx_sdk):
        stage = _load_and_step(physx_sdk)
        matrices = read_column(physx_sdk, stage, SimObjectType.ARTICULATION, "massMatrix")
        assert np.all(np.isfinite(_values(matrices)))
        for path, flat in matrices.items():
            size = math.isqrt(flat.size)
            assert size * size == flat.size, "mass matrix must be square"
            matrix = flat.reshape(size, size)
            np.testing.assert_allclose(
                matrix, matrix.T, atol=1e-4, err_msg=f"Mass matrix for articulation {path} not symmetric"
            )

    def test_gravity_force_nonzero(self, physx_sdk):
        stage = _load_and_step(physx_sdk)
        values = _values(read_column(physx_sdk, stage, SimObjectType.ARTICULATION, "gravityForce"))
        assert np.all(np.isfinite(values))
        assert not np.allclose(values, 0.0), "gravity forces should be non-zero with gravity enabled"

    def test_coriolis_readable(self, physx_sdk):
        stage = _load_and_step(physx_sdk)
        values = read_column(physx_sdk, stage, SimObjectType.ARTICULATION, "coriolisForce")
        assert np.all(np.isfinite(_values(values)))

    def test_projected_joint_force_readable(self, physx_sdk):
        stage = _load_and_step(physx_sdk)
        values = read_column(physx_sdk, stage, SimObjectType.ARTICULATION_JOINT, "jointProjectedForce")
        assert np.all(np.isfinite(_values(values)))

    def test_link_incoming_joint_force_readable(self, physx_sdk):
        stage = _load_and_step(physx_sdk)
        values = read_column(physx_sdk, stage, SimObjectType.ARTICULATION_LINK, "linkIncomingJointForce")
        assert all(row.size == 6 for row in values.values())
        assert np.all(np.isfinite(_values(values)))

    _INVERSE_DYNAMICS_COLUMNS = [
        (SimObjectType.ARTICULATION, "jacobian"),
        (SimObjectType.ARTICULATION, "massMatrix"),
        (SimObjectType.ARTICULATION, "coriolisForce"),
        (SimObjectType.ARTICULATION, "gravityForce"),
        (SimObjectType.ARTICULATION_LINK, "linkIncomingJointForce"),
        (SimObjectType.ARTICULATION_JOINT, "jointProjectedForce"),
    ]

    @pytest.mark.parametrize(
        "object_type,attribute",
        _INVERSE_DYNAMICS_COLUMNS,
        ids=["jacobian", "mass_matrix", "coriolis", "gravity", "link_incoming_joint_force", "projected_joint_force"],
    )
    def test_inverse_dynamics_write_rejected(self, physx_sdk, object_type, attribute):
        _load_and_step(physx_sdk)
        assert write_is_refused(physx_sdk, object_type, attribute)


# ---------------------------------------------------------------------------
# Link wrench: write-only + effect on simulation
# ---------------------------------------------------------------------------


class TestLinkWrenchGpu:

    def test_write_and_effect(self, physx_sdk):
        """A wrench on a link moves its articulation's joints and leaves the other articulation at rest.

        Gravity is off so the wrench is the only load: the fixture's stiff drive damping lets the pushed
        joints move only by about 1e-5 degrees, less than gravity alone moves them.
        """
        stage = _load_and_step(physx_sdk, n_steps=0)
        write_uniform(physx_sdk, stage, SimObjectType.ARTICULATION_LINK, "disableGravity", [1])
        pos_before = read_column(physx_sdk, stage, SimObjectType.ARTICULATION_JOINT, "jointPosition")

        # 1000 N along +x on the first articulation's middle link, applied at the world origin.
        middle_link = handles_for(stage, [FIRST_ARTI + "/articulationLink1"])
        write_uniform(physx_sdk, stage, SimObjectType.ARTICULATION_LINK, "wrench", [1000.0], prim_paths=middle_link)

        for _ in range(5):
            physx_sdk.step(DT)
        physx_sdk.wait_all()

        pos_after = read_column(physx_sdk, stage, SimObjectType.ARTICULATION_JOINT, "jointPosition")
        moved = {path: float(np.max(np.abs(pos_after[path] - pos_before[path]))) for path in pos_before}
        pushed = max(change for path, change in moved.items() if _in_first_arti(path))
        at_rest = max(change for path, change in moved.items() if not _in_first_arti(path))
        assert pushed > 1e-6, f"the wrench should move the pushed articulation's joints, moved {moved}"
        assert pushed > 100.0 * at_rest, f"only the pushed articulation should move, moved {moved}"

    def test_read_rejected(self, physx_sdk):
        stage = _load_and_step(physx_sdk)
        # wrench is write-only: a read emits no column for it.
        assert read_column(physx_sdk, stage, SimObjectType.ARTICULATION_LINK, "wrench") == {}

    # Session counterpart of test_indexed_write: the same subset written through a selection.
    def test_selected_write(self, physx_sdk):
        stage = _load_and_step(physx_sdk)
        written = write_uniform(
            physx_sdk, stage, SimObjectType.ARTICULATION_LINK, "wrench", [0.0], pattern=FIRST_ARTI + "/*"
        )
        assert written == 3, "the selection should cover the first articulation's three links"


# ---------------------------------------------------------------------------
# Fixed tendons: GPU roundtrip on a scene with tendons
# ---------------------------------------------------------------------------


class TestFixedTendonGpu:

    _PATTERN = "/FixedTendonTest"

    def _load(self, sdk):
        stage = attach_usd_with_ovstage(sdk, data_path("FixedTendonTest.usda"))
        sdk.wait_all()
        sdk.warmup()
        for _ in range(3):
            sdk.step(DT)
        sdk.wait_all()
        return stage

    def test_fixed_tendon_count_positive(self, physx_sdk):
        stage = self._load(physx_sdk)
        counts = read_column(physx_sdk, stage, SimObjectType.ARTICULATION, "fixedTendonCount", pattern=self._PATTERN)
        assert np.all(_values(counts) > 0), "FixedTendonTest.usda should have at least one fixed tendon"

    _FIXED_ATTRIBUTES = [
        ("tendonStiffness", "stiffness"),
        ("tendonDamping", "damping"),
        ("tendonLimitStiffness", "limit_stiffness"),
        ("tendonRestLength", "rest_length"),
        ("tendonOffset", "offset"),
    ]

    @pytest.mark.parametrize("attribute,name", _FIXED_ATTRIBUTES, ids=[n for _, n in _FIXED_ATTRIBUTES])
    def test_read_finite(self, physx_sdk, attribute, name):
        stage = self._load(physx_sdk)
        values = read_column(physx_sdk, stage, SimObjectType.FIXED_TENDON, attribute)
        assert values, f"Expected a fixed tendon for {name}"
        assert np.all(np.isfinite(_values(values))), f"Fixed tendon {name} must be finite"

    def test_stiffness_write_roundtrip(self, physx_sdk):
        stage = self._load(physx_sdk)
        tendon = SimObjectType.FIXED_TENDON
        original = read_column(physx_sdk, stage, tendon, "tendonStiffness")
        write_column(physx_sdk, stage, tendon, "tendonStiffness", lambda path, _row: original[path] + 5.0)
        readback = read_column(physx_sdk, stage, tendon, "tendonStiffness")
        np.testing.assert_allclose(_values(readback), _values(original) + 5.0, atol=1e-4)

    # Session counterpart of test_masked_write: the same subset written through a selection.
    def test_selected_write(self, physx_sdk):
        stage = self._load(physx_sdk)
        tendon = SimObjectType.FIXED_TENDON
        original = read_column(physx_sdk, stage, tendon, "tendonDamping")
        # A selection naming every tendon.
        every_tendon = handles_for(stage, list(original))
        write_column(
            physx_sdk, stage, tendon, "tendonDamping", lambda path, _row: original[path] + 77.0, prim_paths=every_tendon
        )
        readback = read_column(physx_sdk, stage, tendon, "tendonDamping")
        np.testing.assert_allclose(_values(readback), _values(original) + 77.0, atol=1e-4)

    def test_limit_shape_3d(self, physx_sdk):
        stage = self._load(physx_sdk)
        values = read_column(physx_sdk, stage, SimObjectType.FIXED_TENDON, "tendonLimit")
        assert all(row.size == 2 for row in values.values()), "Limit must be a (lower, upper) pair per tendon"
        assert np.all(np.isfinite(_values(values)))


# ---------------------------------------------------------------------------
# Spatial tendons: GPU roundtrip on a scene with tendons
# ---------------------------------------------------------------------------


class TestSpatialTendonGpu:

    _PATTERN = "/SpatialTendonTest"

    def _load(self, sdk, scene="SpatialTendonTest.usda"):
        stage = attach_usd_with_ovstage(sdk, data_path(scene))
        sdk.wait_all()
        sdk.warmup()
        for _ in range(3):
            sdk.step(DT)
        sdk.wait_all()
        return stage

    def test_spatial_tendon_count_positive(self, physx_sdk):
        stage = self._load(physx_sdk)
        counts = read_column(physx_sdk, stage, SimObjectType.ARTICULATION, "spatialTendonCount", pattern=self._PATTERN)
        assert np.all(_values(counts) > 0), "SpatialTendonTest.usda should have at least one spatial tendon"

    _SPATIAL_ATTRIBUTES = [
        ("tendonStiffness", "stiffness"),
        ("tendonDamping", "damping"),
        ("tendonLimitStiffness", "limit_stiffness"),
        ("tendonOffset", "offset"),
    ]

    @pytest.mark.parametrize("attribute,name", _SPATIAL_ATTRIBUTES, ids=[n for _, n in _SPATIAL_ATTRIBUTES])
    def test_read_finite(self, physx_sdk, attribute, name):
        stage = self._load(physx_sdk)
        values = read_column(physx_sdk, stage, SimObjectType.SPATIAL_TENDON, attribute)
        assert values, f"Expected a spatial tendon for {name}"
        assert np.all(np.isfinite(_values(values))), f"Spatial tendon {name} must be finite"

    def test_damping_write_roundtrip(self, physx_sdk):
        stage = self._load(physx_sdk)
        tendon = SimObjectType.SPATIAL_TENDON
        original = read_column(physx_sdk, stage, tendon, "tendonDamping")
        write_column(physx_sdk, stage, tendon, "tendonDamping", lambda path, _row: original[path] + 10.0)
        readback = read_column(physx_sdk, stage, tendon, "tendonDamping")
        np.testing.assert_allclose(_values(readback), _values(original) + 10.0, atol=1e-4)

    # Session counterpart of test_masked_write: the same subset written through a selection.
    def test_selected_write(self, physx_sdk):
        # A >= 2 tendon scene so the prim_paths filter is actually exercised: write one, leave the rest.
        stage = self._load(physx_sdk, scene="MultipleSpatialTendonsTest.usda")
        tendon = SimObjectType.SPATIAL_TENDON
        original = read_column(physx_sdk, stage, tendon, "tendonStiffness")
        paths = sorted(original)
        assert len(paths) >= 2, "MultipleSpatialTendonsTest.usda should expose >= 2 spatial tendons"
        selected_paths = paths[:1]
        write_column(
            physx_sdk,
            stage,
            tendon,
            "tendonStiffness",
            lambda path, _row: original[path] + 42.0,
            prim_paths=handles_for(stage, selected_paths),
        )
        readback = read_column(physx_sdk, stage, tendon, "tendonStiffness")
        for path in paths:
            expected = original[path] + (42.0 if path in selected_paths else 0.0)
            np.testing.assert_allclose(
                readback[path], expected, atol=1e-4, err_msg=f"spatial tendon {path} selection isolation"
            )

    def test_multiple_spatial_tendons_count(self, physx_sdk):
        stage = self._load(physx_sdk, scene="MultipleSpatialTendonsTest.usda")
        counts = _values(
            read_column(physx_sdk, stage, SimObjectType.ARTICULATION, "spatialTendonCount", pattern=self._PATTERN)
        )
        assert np.all(counts == 2), f"MultipleSpatialTendonsTest.usda has 2 spatial tendons, got {counts}"


# ---------------------------------------------------------------------------
# Articulation metadata and inverse dynamics: GPU
# ---------------------------------------------------------------------------


class TestArticulationMetadataAndInverseDynamicsGpu:

    def test_metadata(self, physx_sdk):
        """Verify all 6 topology-metadata columns of each articulation (GPU simulation mode)."""
        stage = _load_and_step(physx_sdk)
        expected = {
            "dofCount": 2,
            "linkCount": 3,
            "jointCount": 2,
            "isFixedBase": 1,
            "fixedTendonCount": 0,
            "spatialTendonCount": 0,
        }
        for attribute, value in expected.items():
            values = read_column(physx_sdk, stage, SimObjectType.ARTICULATION, attribute, pattern=ARTI_PATTERN)
            np.testing.assert_array_equal(_values(values), value, err_msg=attribute)
