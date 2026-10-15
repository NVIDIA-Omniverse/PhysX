# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-READ-ATTRS-001
# @covers AC-1 AC-22 AC-23
# @maps_to TEST-READ-ATTRS-001
# @implements REQ-CAPI-WRITE-001
# @covers AC-9
# @maps_to TEST-CAPI-WRITE-001
# @implements REQ-READ-INVDYN-001
# @covers AC-1 AC-6
# @maps_to TEST-READ-INVDYN-001

"""Session read/write tests for articulation DOF/body/metadata/inverse-dynamics and rigid-body properties -- the successor coverage for the physics facts the deprecated tensor-binding tests checked."""

import os

import numpy as np
import ovstage
from ovphysx.types import SimObjectType
from session_helpers import read_column, write_column, write_is_refused, write_uniform
from test_utils import attach_usd_with_ovstage


_TEST_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def data_path(filename):
    return os.path.join(_TEST_DIR, "data", filename)


ARTI_PATTERN = "/World/articulation*"
EXPECTED_N = 2  # articulations
EXPECTED_L = 3  # links per articulation
EXPECTED_D = 2  # DOFs per articulation


def _load_two_articulations(sdk):
    """Load two_articulations.usda and wait for completion; returns the ovstage Stage."""
    stage = attach_usd_with_ovstage(sdk, data_path("two_articulations.usda"))
    sdk.wait_all()
    return stage


def _load_and_warm(sdk):
    """Load two_articulations.usda and run the first step a session articulation read or write needs."""
    stage = _load_two_articulations(sdk)
    sdk.warmup()
    sdk.wait_all()
    return stage


def _load_and_step(sdk, n_steps=5, dt=1.0 / 60.0):
    """Load two_articulations.usda and advance simulation so dynamics data is populated."""
    stage = _load_two_articulations(sdk)
    for _ in range(n_steps):
        sdk.step(dt)
    sdk.wait_all()
    return stage


def _assert_rows_close(actual, expected, **tolerance):
    """`actual` reports exactly the prims of `expected`, each with its expected row."""
    assert actual.keys() == expected.keys()
    for path, row in expected.items():
        np.testing.assert_allclose(actual[path], row, err_msg=path, **tolerance)


def _row_paths(sdk, stage, object_type, attribute):
    """Prim path of every row a session read of `attribute` emits, in read order and not deduplicated."""
    with sdk.read_sync(object_type, [attribute]) as result:
        with ovstage.PathDictionary(stage) as dictionary:
            return [
                dictionary.path_to_string(handle)
                for group in result.groups
                for handle in dictionary.get_paths(group.prim_list)[
                    group.prim_offset : group.prim_offset + group.prim_count
                ]
            ]


def _by_articulation(paths):
    """Group member prim paths under the two articulation roots of two_articulations.usda."""
    roots = ("/World/articulation", "/World/articulation2")
    return {root: [path for path in paths if path.startswith(root + "/")] for root in roots}


def _dof_joint_paths(sdk, stage):
    """{articulation path -> joint prim path of each DOF}, from the session's per-DOF dofJointPath column.

    Read raw rather than through read_column: the handles are uint64, which a float64 row cannot carry exactly.
    """
    out = {}
    with sdk.read_sync(SimObjectType.ARTICULATION, ["dofJointPath"]) as result:
        with ovstage.PathDictionary(stage) as dictionary:
            for group in result.groups:
                assert group.index_map is None
                handles = np.asarray(group.tensors[0].numpy()).reshape(group.prim_count, -1)
                roots = dictionary.get_paths(group.prim_list)
                for row in range(group.prim_count):
                    root = dictionary.path_to_string(roots[group.prim_offset + row])
                    # Rows are padded to the widest articulation with null handles.
                    out[root] = [dictionary.path_to_string(int(handle)) for handle in handles[row] if handle]
    return out


RIGID_BODY_PATTERN = "/World/Cube*"
EXPECTED_RIGID_BODIES = 11  # Cube1 through Cube11 in boxes scene


def _load_boxes(sdk, n_steps=5, dt=1.0 / 60.0):
    """Load boxes_falling_on_groundplane.usda and step to populate data; returns the ovstage Stage."""
    stage = attach_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
    sdk.wait_all()
    sdk.warmup()
    for _ in range(n_steps):
        sdk.step(dt)
    sdk.wait_all()
    return stage


def _read_cubes(sdk, stage, attribute):
    """{cube path -> row} for `attribute` of every rigid body RIGID_BODY_PATTERN selects."""
    return read_column(sdk, stage, SimObjectType.RIGID_BODY, attribute, pattern=RIGID_BODY_PATTERN)


def _write_cubes(sdk, stage, attribute, rows):
    """Write {cube path -> row} to the rigid bodies RIGID_BODY_PATTERN selects."""
    write_column(
        sdk, stage, SimObjectType.RIGID_BODY, attribute, lambda path, _row: rows[path], pattern=RIGID_BODY_PATTERN
    )


class TestDofProperties:
    """Session read/write tests for the DOF properties: stiffness, damping, limits, max velocity,
    max force, armature and friction. They verify that:
      1. A full session read succeeds and produces finite values.
      2. A modified session write followed by a second read reflects the change (round-trip).
    The session reports one row per revolute joint, one value per axis, with angles in degrees.
    """

    def _read(self, sdk, stage, attribute):
        return read_column(sdk, stage, SimObjectType.ARTICULATION_JOINT, attribute)

    def _write(self, sdk, stage, attribute, rows):
        """Write {joint path -> row} and return the rows re-read after the write."""
        write_column(sdk, stage, SimObjectType.ARTICULATION_JOINT, attribute, lambda path, _row: rows[path])
        return self._read(sdk, stage, attribute)

    def test_dof_stiffness_read(self, physx_sdk_cpu):
        stage = _load_and_warm(physx_sdk_cpu)
        out = self._read(physx_sdk_cpu, stage, "jointStiffness")
        assert sum(row.size for row in out.values()) == EXPECTED_N * EXPECTED_D
        assert all(np.all(np.isfinite(row)) for row in out.values()), "stiffness must be finite"

    def test_dof_stiffness_write_roundtrip(self, physx_sdk_cpu):
        stage = _load_and_warm(physx_sdk_cpu)
        original = self._read(physx_sdk_cpu, stage, "jointStiffness")
        modified = {path: row + 100.0 for path, row in original.items()}
        result = self._write(physx_sdk_cpu, stage, "jointStiffness", modified)
        _assert_rows_close(result, modified, rtol=1e-5)

    def test_dof_damping_roundtrip(self, physx_sdk_cpu):
        stage = _load_and_warm(physx_sdk_cpu)
        out = self._read(physx_sdk_cpu, stage, "jointDamping")
        assert sum(row.size for row in out.values()) == EXPECTED_N * EXPECTED_D
        new_vals = {path: row + 5.0 for path, row in out.items()}
        result = self._write(physx_sdk_cpu, stage, "jointDamping", new_vals)
        _assert_rows_close(result, new_vals, rtol=1e-5)

    def test_dof_limit_read(self, physx_sdk_cpu):
        stage = _load_and_warm(physx_sdk_cpu)
        out = self._read(physx_sdk_cpu, stage, "jointLimit")
        assert len(out) == EXPECTED_N * EXPECTED_D
        for path, (lower, upper) in out.items():
            assert np.isfinite(lower) and np.isfinite(upper), f"{path}: DOF limits must be finite"
            assert lower <= upper, f"{path}: lower limit must be <= upper limit"

    def test_dof_limit_roundtrip(self, physx_sdk_cpu):
        stage = _load_and_warm(physx_sdk_cpu)
        original = self._read(physx_sdk_cpu, stage, "jointLimit")
        # (lower, upper) = (-1.5, 1.5) rad; the session carries joint angles in degrees.
        new_vals = {path: np.degrees([-1.5, 1.5]) for path in original}
        result = self._write(physx_sdk_cpu, stage, "jointLimit", new_vals)
        _assert_rows_close(result, new_vals, rtol=1e-5)

    def test_dof_max_velocity_roundtrip(self, physx_sdk_cpu):
        stage = _load_and_warm(physx_sdk_cpu)
        original = self._read(physx_sdk_cpu, stage, "jointMaxVelocity")
        assert sum(row.size for row in original.values()) == EXPECTED_N * EXPECTED_D
        modified = {path: row + 10.0 for path, row in original.items()}
        result = self._write(physx_sdk_cpu, stage, "jointMaxVelocity", modified)
        _assert_rows_close(result, modified, rtol=1e-5)

    def test_dof_max_force_roundtrip(self, physx_sdk_cpu):
        stage = _load_and_warm(physx_sdk_cpu)
        original = self._read(physx_sdk_cpu, stage, "jointMaxForce")
        assert sum(row.size for row in original.values()) == EXPECTED_N * EXPECTED_D
        modified = {path: row + 50.0 for path, row in original.items()}
        result = self._write(physx_sdk_cpu, stage, "jointMaxForce", modified)
        _assert_rows_close(result, modified, rtol=1e-5)

    def test_dof_damping_and_bounds_distinct_value_roundtrip(self, physx_sdk_cpu):
        """Damping, max velocity and max force read back a written value far from the authored one.

        The scene's damping (1e8) absorbs the +5 of test_dof_damping_roundtrip within its tolerance, and its
        bounds are the FLT_MAX "unlimited" sentinel the +10 / +50 round trips leave unchanged, so those
        three cannot tell a write from a no-op.
        """
        stage = _load_and_warm(physx_sdk_cpu)
        for attribute, value in (("jointDamping", 1000.0), ("jointMaxVelocity", 10.0), ("jointMaxForce", 50.0)):
            current = self._read(physx_sdk_cpu, stage, attribute)
            assert len(current) == EXPECTED_N * EXPECTED_D, attribute
            rows = {path: np.full(row.shape, value) for path, row in current.items()}
            _assert_rows_close(self._write(physx_sdk_cpu, stage, attribute, rows), rows, rtol=1e-5)

    def test_dof_armature_roundtrip(self, physx_sdk_cpu):
        stage = _load_and_warm(physx_sdk_cpu)
        original = self._read(physx_sdk_cpu, stage, "jointArmature")
        assert sum(row.size for row in original.values()) == EXPECTED_N * EXPECTED_D
        new_vals = {path: np.full(row.shape, 0.01) for path, row in original.items()}
        result = self._write(physx_sdk_cpu, stage, "jointArmature", new_vals)
        _assert_rows_close(result, new_vals, rtol=1e-5)

    # The session splits the friction triple into one attribute per quantity.
    _FRICTION_ATTRIBUTES = ("jointStaticFriction", "jointDynamicFriction", "jointViscousFriction")

    def test_dof_friction_properties_read(self, physx_sdk_cpu):
        stage = _load_and_warm(physx_sdk_cpu)
        for attribute in self._FRICTION_ATTRIBUTES:
            out = self._read(physx_sdk_cpu, stage, attribute)
            values = np.concatenate(list(out.values()))
            assert values.size == EXPECTED_N * EXPECTED_D, attribute
            assert np.all(np.isfinite(values)), f"{attribute} must be finite"
            assert np.all(values >= 0.0), f"{attribute} must be >= 0"

    def test_dof_friction_properties_roundtrip(self, physx_sdk_cpu):
        stage = _load_and_warm(physx_sdk_cpu)
        new_values = {
            "jointStaticFriction": 0.1,  # static friction effort
            "jointDynamicFriction": 0.05,  # dynamic friction effort
            "jointViscousFriction": 0.01,  # viscous friction coefficient
        }
        for attribute, value in new_values.items():
            write_uniform(physx_sdk_cpu, stage, SimObjectType.ARTICULATION_JOINT, attribute, [value])
        for attribute, value in new_values.items():
            result = self._read(physx_sdk_cpu, stage, attribute)
            assert len(result) == EXPECTED_N * EXPECTED_D, attribute
            _assert_rows_close(result, {path: [value] for path in result}, rtol=1e-5)


class TestBodyProperties:
    """Session read/write tests for the articulation link properties: mass, COM, inertia.
    Also tests the read-only link acceleration.
    """

    def _read(self, sdk, stage, attribute):
        return read_column(sdk, stage, SimObjectType.ARTICULATION_LINK, attribute)

    def _write(self, sdk, stage, attribute, rows):
        write_column(sdk, stage, SimObjectType.ARTICULATION_LINK, attribute, lambda path, _row: rows[path])

    def test_body_mass_read(self, physx_sdk_cpu):
        stage = _load_and_warm(physx_sdk_cpu)
        out = self._read(physx_sdk_cpu, stage, "mass")
        assert len(out) == EXPECTED_N * EXPECTED_L
        values = np.concatenate(list(out.values()))
        assert np.all(np.isfinite(values)), "mass must be finite"
        assert np.all(values > 0.0), "all link masses must be positive"

    def test_body_mass_roundtrip(self, physx_sdk_cpu):
        stage = _load_and_warm(physx_sdk_cpu)
        original = self._read(physx_sdk_cpu, stage, "mass")
        new_mass = {path: row * 2.0 for path, row in original.items()}
        self._write(physx_sdk_cpu, stage, "mass", new_mass)
        _assert_rows_close(self._read(physx_sdk_cpu, stage, "mass"), new_mass, rtol=1e-5)

    def test_body_com_pose_read(self, physx_sdk_cpu):
        stage = _load_and_warm(physx_sdk_cpu)
        position = self._read(physx_sdk_cpu, stage, "centerOfMassPosition")
        quat = self._read(physx_sdk_cpu, stage, "centerOfMassOrientation")
        assert len(position) == len(quat) == EXPECTED_N * EXPECTED_L
        for path in position:
            assert np.all(np.isfinite(position[path])), f"{path}: COM values must be finite"
            assert np.all(np.isfinite(quat[path])), f"{path}: COM values must be finite"
            np.testing.assert_allclose(
                np.linalg.norm(quat[path]), 1.0, atol=1e-4, err_msg=f"{path}: COM quaternions must be unit norm"
            )

    def test_body_com_pose_roundtrip(self, physx_sdk_cpu):
        # The COM pose is two session attributes; both are written, the orientation unchanged.
        stage = _load_and_warm(physx_sdk_cpu)
        position = self._read(physx_sdk_cpu, stage, "centerOfMassPosition")
        quat = self._read(physx_sdk_cpu, stage, "centerOfMassOrientation")
        modified = {path: row + [0.01, 0.0, 0.0] for path, row in position.items()}
        self._write(physx_sdk_cpu, stage, "centerOfMassPosition", modified)
        self._write(physx_sdk_cpu, stage, "centerOfMassOrientation", quat)
        _assert_rows_close(self._read(physx_sdk_cpu, stage, "centerOfMassPosition"), modified, rtol=1e-5)
        _assert_rows_close(self._read(physx_sdk_cpu, stage, "centerOfMassOrientation"), quat, rtol=1e-5)

    def test_body_com_orientation_write_roundtrip(self, physx_sdk_cpu):
        """A written link COM orientation reads back; the round trip above writes it unchanged."""
        stage = _load_and_warm(physx_sdk_cpu)
        half = np.sqrt(0.5)
        rotated = {path: [half, 0.0, 0.0, half] for path in self._read(physx_sdk_cpu, stage, "centerOfMassOrientation")}
        assert len(rotated) == EXPECTED_N * EXPECTED_L
        self._write(physx_sdk_cpu, stage, "centerOfMassOrientation", rotated)
        _assert_rows_close(self._read(physx_sdk_cpu, stage, "centerOfMassOrientation"), rotated, atol=1e-5)

    def test_body_inertia_read(self, physx_sdk_cpu):
        stage = _load_and_warm(physx_sdk_cpu)
        out = self._read(physx_sdk_cpu, stage, "inertia")
        assert len(out) == EXPECTED_N * EXPECTED_L
        values = np.stack(list(out.values()))
        assert np.all(np.isfinite(values)), "inertia values must be finite"
        diag_idx = [0, 4, 8]
        for i in diag_idx:
            assert np.all(values[:, i] > 0.0), f"inertia diagonal element {i} must be positive"

    def test_body_inertia_roundtrip(self, physx_sdk_cpu):
        stage = _load_and_warm(physx_sdk_cpu)
        original = self._read(physx_sdk_cpu, stage, "inertia")
        scaled = {path: row * 2.0 for path, row in original.items()}
        self._write(physx_sdk_cpu, stage, "inertia", scaled)
        _assert_rows_close(self._read(physx_sdk_cpu, stage, "inertia"), scaled, rtol=1e-5)

    def test_body_inverse_mass_and_inertia_read(self, physx_sdk_cpu):
        """A link's inverse mass and inverse inertia are the inverses of its mass and inertia."""
        stage = _load_and_warm(physx_sdk_cpu)
        mass = self._read(physx_sdk_cpu, stage, "mass")
        inertia = self._read(physx_sdk_cpu, stage, "inertia")
        inverse_mass = self._read(physx_sdk_cpu, stage, "inverseMass")
        inverse_inertia = self._read(physx_sdk_cpu, stage, "inverseInertia")
        assert len(mass) == EXPECTED_N * EXPECTED_L
        _assert_rows_close(inverse_mass, {path: 1.0 / row for path, row in mass.items()}, rtol=1e-4)
        _assert_rows_close(
            inverse_inertia,
            {path: np.linalg.inv(row.reshape(3, 3)).reshape(-1) for path, row in inertia.items()},
            rtol=1e-4,
            atol=1e-6,
        )

    def test_link_acceleration_read(self, physx_sdk_cpu):
        stage = _load_and_step(physx_sdk_cpu)
        # The session splits the 6-wide acceleration into its linear and angular halves.
        for attribute in ("linearAcceleration", "angularAcceleration"):
            out = self._read(physx_sdk_cpu, stage, attribute)
            assert len(out) == EXPECTED_N * EXPECTED_L, attribute
            assert all(np.all(np.isfinite(row)) for row in out.values()), "link accelerations must be finite"

    def test_link_acceleration_is_read_only(self, physx_sdk_cpu):
        _load_two_articulations(physx_sdk_cpu)
        for attribute in ("linearAcceleration", "angularAcceleration"):
            assert write_is_refused(physx_sdk_cpu, SimObjectType.ARTICULATION_LINK, attribute), attribute


class TestArticulationMetadata:
    """Session tests for the articulation topology: DOF, link and joint counts, the fixed-base flag,
    and the per-DOF / per-link / per-joint prims (the session's counterpart of dof_names, body_names
    and joint_names).
    """

    def _read_metadata(self, sdk, attribute):
        """{articulation path -> int} for one whole-articulation metadata column."""
        stage = _load_and_warm(sdk)
        values = read_column(sdk, stage, SimObjectType.ARTICULATION, attribute)
        assert len(values) == EXPECTED_N, attribute
        return {path: int(row[0]) for path, row in values.items()}

    def _member_paths(self, sdk, object_type, attribute):
        """{articulation path -> prim path of each member row} for links or joints."""
        stage = _load_and_warm(sdk)
        return _by_articulation(_row_paths(sdk, stage, object_type, attribute))

    def test_dof_count(self, physx_sdk_cpu):
        for path, dof_count in self._read_metadata(physx_sdk_cpu, "dofCount").items():
            assert dof_count == EXPECTED_D, f"{path}: Expected dofCount={EXPECTED_D}, got {dof_count}"

    def test_body_count(self, physx_sdk_cpu):
        for path, link_count in self._read_metadata(physx_sdk_cpu, "linkCount").items():
            assert link_count == EXPECTED_L, f"{path}: Expected linkCount={EXPECTED_L}, got {link_count}"

    def test_joint_count(self, physx_sdk_cpu):
        for path, joint_count in self._read_metadata(physx_sdk_cpu, "jointCount").items():
            assert joint_count == EXPECTED_D, f"{path}: Expected jointCount={EXPECTED_D}, got {joint_count}"

    def test_is_fixed_base(self, physx_sdk_cpu):
        for path, fixed_base in self._read_metadata(physx_sdk_cpu, "isFixedBase").items():
            assert fixed_base == 1, f"{path}: two_articulations.usda has fixed base (PhysicsFixedJoint rootJoint)"

    def test_dof_names_count(self, physx_sdk_cpu):
        stage = _load_and_warm(physx_sdk_cpu)
        joints_per_dof = _dof_joint_paths(physx_sdk_cpu, stage)
        assert len(joints_per_dof) == EXPECTED_N
        for path, joints in joints_per_dof.items():
            assert len(joints) == EXPECTED_D, f"{path}: Expected {EXPECTED_D} DOFs, got {len(joints)}"

    def test_dof_names_are_strings(self, physx_sdk_cpu):
        stage = _load_and_warm(physx_sdk_cpu)
        for joints in _dof_joint_paths(physx_sdk_cpu, stage).values():
            for joint in joints:
                assert isinstance(joint, str) and len(joint) > 0, f"DOF joint must be a non-empty path, got {joint!r}"

    def test_body_names_count(self, physx_sdk_cpu):
        for path, links in self._member_paths(physx_sdk_cpu, SimObjectType.ARTICULATION_LINK, "mass").items():
            assert len(links) == EXPECTED_L, f"{path}: Expected {EXPECTED_L} links, got {len(links)}"

    def test_body_names_are_strings(self, physx_sdk_cpu):
        for links in self._member_paths(physx_sdk_cpu, SimObjectType.ARTICULATION_LINK, "mass").values():
            for link in links:
                assert isinstance(link, str) and len(link) > 0, f"Link must be a non-empty path, got {link!r}"

    def test_joint_names_count(self, physx_sdk_cpu):
        joints = self._member_paths(physx_sdk_cpu, SimObjectType.ARTICULATION_JOINT, "jointPosition")
        for path, members in joints.items():
            assert len(members) == EXPECTED_D, f"{path}: Expected {EXPECTED_D} joints, got {len(members)}"

    def test_joint_names_are_strings(self, physx_sdk_cpu):
        joints = self._member_paths(physx_sdk_cpu, SimObjectType.ARTICULATION_JOINT, "jointPosition")
        for members in joints.values():
            for joint in members:
                assert isinstance(joint, str) and len(joint) > 0, f"Joint must be a non-empty path, got {joint!r}"

    def test_dof_names_unique(self, physx_sdk_cpu):
        # A DOF is named by its joint prim and its axis on that joint.
        stage = _load_and_warm(physx_sdk_cpu)
        joints_per_dof = _dof_joint_paths(physx_sdk_cpu, stage)
        axes = read_column(physx_sdk_cpu, stage, SimObjectType.ARTICULATION, "dofAxisIndex")
        for path, joints in joints_per_dof.items():
            dofs = list(zip(joints, axes[path][: len(joints)]))
            assert len(dofs) == len(set(dofs)), f"{path}: DOFs must be unique"

    def test_body_names_unique(self, physx_sdk_cpu):
        for path, links in self._member_paths(physx_sdk_cpu, SimObjectType.ARTICULATION_LINK, "mass").items():
            assert len(links) == len(set(links)), f"{path}: Links must be unique"


class TestInverseDynamicsTensors:
    """Tests for inverse dynamics query tensors: Jacobian, mass matrix, Coriolis+centrifugal,
    gravity compensation, and link incoming joint force.

    IMPORTANT: All tests step the simulation first because Jacobian/mass-matrix
    shapes are computed lazily after the first sim step. The session flattens each
    articulation's matrix row-major into one row.
    """

    def _read(self, sdk, stage, attribute):
        """{articulation path -> flat row} for one whole-articulation inverse-dynamics column."""
        out = read_column(sdk, stage, SimObjectType.ARTICULATION, attribute)
        assert len(out) == EXPECTED_N, attribute
        return out

    def _mass_matrices(self, sdk, stage):
        """{articulation path -> [M, M] mass matrix}."""
        out = {}
        for path, row in self._read(sdk, stage, "massMatrix").items():
            m = int(round(np.sqrt(row.size)))
            assert m * m == row.size, f"{path}: mass matrix must be square, got {row.size} values"
            out[path] = row.reshape(m, m)
        return out

    def test_jacobian_dimensions_fixed_base(self, physx_sdk_cpu):
        stage = _load_and_step(physx_sdk_cpu)
        shapes = self._read(physx_sdk_cpu, stage, "jacobianShape")
        jacobians = self._read(physx_sdk_cpu, stage, "jacobian")
        for path, (rows, cols) in shapes.items():
            assert rows == (EXPECTED_L - 1) * 6, f"{path}: fixed-base Jacobian rows should be (L-1)*6, got {rows}"
            assert cols == EXPECTED_D, f"{path}: fixed-base Jacobian cols should equal DOF count, got {cols}"
            assert jacobians[path].size == rows * cols, f"{path}: jacobian must carry rows*cols values"

    def test_jacobian_read(self, physx_sdk_cpu):
        stage = _load_and_step(physx_sdk_cpu)
        out = self._read(physx_sdk_cpu, stage, "jacobian")
        assert all(np.all(np.isfinite(row)) for row in out.values()), "Jacobian values must be finite"

    def test_jacobian_is_read_only(self, physx_sdk_cpu):
        _load_and_step(physx_sdk_cpu)
        assert write_is_refused(physx_sdk_cpu, SimObjectType.ARTICULATION, "jacobian")

    def test_mass_matrix_dimension_fixed_base(self, physx_sdk_cpu):
        stage = _load_and_step(physx_sdk_cpu)
        for path, matrix in self._mass_matrices(physx_sdk_cpu, stage).items():
            assert matrix.shape == (EXPECTED_D, EXPECTED_D), (
                f"{path}: fixed-base mass matrix dim should equal D={EXPECTED_D}, got {matrix.shape}"
            )

    def test_mass_matrix_symmetry(self, physx_sdk_cpu):
        stage = _load_and_step(physx_sdk_cpu)
        for path, matrix in self._mass_matrices(physx_sdk_cpu, stage).items():
            assert np.all(np.isfinite(matrix)), f"{path}: mass matrix must be finite"
            np.testing.assert_allclose(matrix, matrix.T, atol=1e-4, err_msg=f"{path}: mass matrix must be symmetric")

    def test_mass_matrix_positive_definite_diagonal(self, physx_sdk_cpu):
        stage = _load_and_step(physx_sdk_cpu)
        for path, matrix in self._mass_matrices(physx_sdk_cpu, stage).items():
            for i in range(matrix.shape[0]):
                assert matrix[i, i] > 0.0, f"{path}: Mass matrix diagonal element [{i},{i}] must be positive"

    def test_mass_matrix_is_read_only(self, physx_sdk_cpu):
        _load_and_step(physx_sdk_cpu)
        assert write_is_refused(physx_sdk_cpu, SimObjectType.ARTICULATION, "massMatrix")

    def test_coriolis_read(self, physx_sdk_cpu):
        stage = _load_and_step(physx_sdk_cpu)
        out = self._read(physx_sdk_cpu, stage, "coriolisForce")
        assert all(np.all(np.isfinite(row)) for row in out.values()), "coriolis values must be finite"

    def test_coriolis_is_read_only(self, physx_sdk_cpu):
        _load_and_step(physx_sdk_cpu)
        assert write_is_refused(physx_sdk_cpu, SimObjectType.ARTICULATION, "coriolisForce")

    def test_gravity_read(self, physx_sdk_cpu):
        stage = _load_and_step(physx_sdk_cpu)
        out = self._read(physx_sdk_cpu, stage, "gravityForce")
        assert all(np.all(np.isfinite(row)) for row in out.values()), "gravity compensation values must be finite"

    def test_gravity_is_read_only(self, physx_sdk_cpu):
        _load_and_step(physx_sdk_cpu)
        assert write_is_refused(physx_sdk_cpu, SimObjectType.ARTICULATION, "gravityForce")

    def test_link_incoming_joint_force_read(self, physx_sdk_cpu):
        stage = _load_and_step(physx_sdk_cpu)
        out = read_column(physx_sdk_cpu, stage, SimObjectType.ARTICULATION_LINK, "linkIncomingJointForce")
        assert len(out) == EXPECTED_N * EXPECTED_L
        assert all(np.all(np.isfinite(row)) for row in out.values()), "link incoming joint forces must be finite"

    def test_link_incoming_joint_force_is_read_only(self, physx_sdk_cpu):
        _load_and_step(physx_sdk_cpu)
        assert write_is_refused(physx_sdk_cpu, SimObjectType.ARTICULATION_LINK, "linkIncomingJointForce")

    def test_mass_matrix_and_coriolis_same_M_dimension(self, physx_sdk_cpu):
        """M dimension must be consistent across mass matrix, coriolis, and gravity."""
        stage = _load_and_step(physx_sdk_cpu)
        coriolis = self._read(physx_sdk_cpu, stage, "coriolisForce")
        gravity = self._read(physx_sdk_cpu, stage, "gravityForce")
        for path, matrix in self._mass_matrices(physx_sdk_cpu, stage).items():
            M = matrix.shape[0]
            assert coriolis[path].size == M, f"{path}: Coriolis M must match mass matrix M"
            assert gravity[path].size == M, f"{path}: Gravity M must match mass matrix M"


class TestRigidBodyProperties:
    """Tests for standalone rigid body properties.

    Uses boxes_falling_on_groundplane.usda, which has 11 independent rigid body
    cubes.
    """

    def test_rigid_body_acceleration_read(self, physx_sdk_cpu):
        stage = _load_boxes(physx_sdk_cpu)
        # The session splits the 6-wide acceleration into its linear and angular halves.
        for attribute in ("linearAcceleration", "angularAcceleration"):
            buf = _read_cubes(physx_sdk_cpu, stage, attribute)
            assert len(buf) == EXPECTED_RIGID_BODIES, attribute
            assert all(np.all(np.isfinite(row)) for row in buf.values()), attribute

    def test_rigid_body_acceleration_is_read_only(self, physx_sdk_cpu):
        _load_boxes(physx_sdk_cpu)
        for attribute in ("linearAcceleration", "angularAcceleration"):
            assert write_is_refused(physx_sdk_cpu, SimObjectType.RIGID_BODY, attribute, pattern=RIGID_BODY_PATTERN)

    def test_rigid_body_mass_read(self, physx_sdk_cpu):
        stage = _load_boxes(physx_sdk_cpu)
        buf = _read_cubes(physx_sdk_cpu, stage, "mass")
        assert len(buf) == EXPECTED_RIGID_BODIES
        values = np.concatenate(list(buf.values()))
        assert np.all(np.isfinite(values))
        assert np.all(values > 0), "rigid body masses should be positive"

    def test_rigid_body_mass_write_roundtrip(self, physx_sdk_cpu):
        stage = _load_boxes(physx_sdk_cpu)
        original = _read_cubes(physx_sdk_cpu, stage, "mass")
        modified = {path: row * 2.0 for path, row in original.items()}
        _write_cubes(physx_sdk_cpu, stage, "mass", modified)
        _assert_rows_close(_read_cubes(physx_sdk_cpu, stage, "mass"), modified, rtol=1e-5)
        _write_cubes(physx_sdk_cpu, stage, "mass", original)

    def test_rigid_body_inv_mass_shape_and_read(self, physx_sdk_cpu):
        stage = _load_boxes(physx_sdk_cpu)
        buf = _read_cubes(physx_sdk_cpu, stage, "inverseMass")
        assert len(buf) == EXPECTED_RIGID_BODIES
        assert all(row.size == 1 for row in buf.values())
        values = np.concatenate(list(buf.values()))
        assert np.all(np.isfinite(values))
        assert np.all(values >= 0), "inverse masses should be non-negative"

    def test_rigid_body_inv_mass_is_read_only(self, physx_sdk_cpu):
        _load_boxes(physx_sdk_cpu)
        assert write_is_refused(physx_sdk_cpu, SimObjectType.RIGID_BODY, "inverseMass", pattern=RIGID_BODY_PATTERN)

    def test_rigid_body_inertia_read(self, physx_sdk_cpu):
        stage = _load_boxes(physx_sdk_cpu)
        buf = _read_cubes(physx_sdk_cpu, stage, "inertia")
        assert len(buf) == EXPECTED_RIGID_BODIES
        assert all(np.all(np.isfinite(row)) for row in buf.values())

    def test_rigid_body_inv_inertia_shape_and_read(self, physx_sdk_cpu):
        stage = _load_boxes(physx_sdk_cpu)
        buf = _read_cubes(physx_sdk_cpu, stage, "inverseInertia")
        assert len(buf) == EXPECTED_RIGID_BODIES
        assert all(row.size == 9 for row in buf.values())
        assert all(np.all(np.isfinite(row)) for row in buf.values())

    def test_rigid_body_inv_inertia_is_read_only(self, physx_sdk_cpu):
        _load_boxes(physx_sdk_cpu)
        assert write_is_refused(physx_sdk_cpu, SimObjectType.RIGID_BODY, "inverseInertia", pattern=RIGID_BODY_PATTERN)

    def test_rigid_body_com_pose_read(self, physx_sdk_cpu):
        stage = _load_boxes(physx_sdk_cpu)
        position = _read_cubes(physx_sdk_cpu, stage, "centerOfMassPosition")
        quat = _read_cubes(physx_sdk_cpu, stage, "centerOfMassOrientation")
        assert len(position) == len(quat) == EXPECTED_RIGID_BODIES
        assert all(np.all(np.isfinite(row)) for row in [*position.values(), *quat.values()])
        quat_norms = np.linalg.norm(np.stack(list(quat.values())), axis=1)
        np.testing.assert_allclose(quat_norms, 1.0, atol=1e-5, err_msg="COM pose quaternions should be unit length")

    def test_rigid_body_com_pose_write_roundtrip(self, physx_sdk_cpu):
        # The COM pose is two session attributes; both are written, the orientation unchanged.
        stage = _load_boxes(physx_sdk_cpu)
        position = _read_cubes(physx_sdk_cpu, stage, "centerOfMassPosition")
        quat = _read_cubes(physx_sdk_cpu, stage, "centerOfMassOrientation")
        modified = {path: row + [0.01, 0.0, 0.0] for path, row in position.items()}
        _write_cubes(physx_sdk_cpu, stage, "centerOfMassPosition", modified)
        _write_cubes(physx_sdk_cpu, stage, "centerOfMassOrientation", quat)
        # COM position should reflect written values.
        _assert_rows_close(_read_cubes(physx_sdk_cpu, stage, "centerOfMassPosition"), modified, rtol=1e-5)
        quat_norms = np.linalg.norm(
            np.stack(list(_read_cubes(physx_sdk_cpu, stage, "centerOfMassOrientation").values())), axis=1
        )
        np.testing.assert_allclose(
            quat_norms, 1.0, atol=1e-4, err_msg="COM quaternions should stay unit norm after write"
        )
        _write_cubes(physx_sdk_cpu, stage, "centerOfMassPosition", position)
        _write_cubes(physx_sdk_cpu, stage, "centerOfMassOrientation", quat)


class TestFixedTendon:
    """Session tests for fixed tendons.

    two_articulations.usda has no fixed tendons, so the session reports a fixedTendonCount of 0.
    """

    def test_fixed_tendon_count_zero(self, physx_sdk_cpu):
        stage = _load_and_warm(physx_sdk_cpu)
        counts = read_column(physx_sdk_cpu, stage, SimObjectType.ARTICULATION, "fixedTendonCount")
        assert len(counts) == EXPECTED_N
        for path, count in counts.items():
            assert count[0] == 0, f"{path}: two_articulations.usda has no tendons, expected T=0"


class TestProjectedJointForce:
    """Session tests for the projected joint force (session jointProjectedForce), read-only."""

    def test_projected_joint_force_read(self, physx_sdk_cpu):
        stage = _load_and_step(physx_sdk_cpu)
        buf = read_column(physx_sdk_cpu, stage, SimObjectType.ARTICULATION_JOINT, "jointProjectedForce")
        assert sum(row.size for row in buf.values()) == EXPECTED_N * EXPECTED_D
        assert all(np.all(np.isfinite(row)) for row in buf.values()), "projected joint forces must be finite"

    def test_projected_joint_force_write_raises(self, physx_sdk_cpu):
        _load_and_step(physx_sdk_cpu)
        assert write_is_refused(physx_sdk_cpu, SimObjectType.ARTICULATION_JOINT, "jointProjectedForce")


class TestSpatialTendon:
    """Session tests for spatial tendons.

    two_articulations.usda has no spatial tendons, so the session reports a spatialTendonCount of 0.
    """

    def test_spatial_tendon_count_zero(self, physx_sdk_cpu):
        stage = _load_and_warm(physx_sdk_cpu)
        counts = read_column(physx_sdk_cpu, stage, SimObjectType.ARTICULATION, "spatialTendonCount")
        assert len(counts) == EXPECTED_N
        for path, count in counts.items():
            assert count[0] == 0, f"{path}: two_articulations.usda has no spatial tendons, expected T=0"


class TestLinkWrench:
    """Session tests for the link wrench (session wrench).

    Write-only, for applying external wrenches to articulation links; the session writes one row per link.
    """

    def test_write_succeeds(self, physx_sdk_cpu):
        stage = _load_and_warm(physx_sdk_cpu)

        def fill(path, row):
            row[:] = 0.0
            if path == "/World/articulation/articulationLink0":
                row[0] = 1.0  # apply force in x on first link
            return row

        written = write_column(physx_sdk_cpu, stage, SimObjectType.ARTICULATION_LINK, "wrench", fill)
        assert written == EXPECTED_N * EXPECTED_L

    def test_read_raises(self, physx_sdk_cpu):
        """Link wrench is write-only, so the session read emits no column for it."""
        stage = _load_and_warm(physx_sdk_cpu)
        assert read_column(physx_sdk_cpu, stage, SimObjectType.ARTICULATION_LINK, "mass"), "links must be readable"
        with physx_sdk_cpu.read_sync(SimObjectType.ARTICULATION_LINK, ["wrench"]) as result:
            assert result.groups == []
