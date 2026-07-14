# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

"""
Base test infrastructure for ovruntime physics tests.

Ported from omni.physx.tests physicsBase.py for standalone (non-Kit) use.
Uses unittest.TestCase and the _physx bindings module directly.
"""

import _physics_setup  # noqa: F401 - loads Carbonite + physics plugins + PhysX schemas

import unittest
from enum import Enum
from functools import partial
from typing import Iterator, Dict, Union
from pxr import Usd, UsdGeom, UsdPhysics, UsdUtils, Gf, Sdf
import _physx
import carb.logging


class TestCategory(Enum):
    __test__ = False  # prevent pytest collection
    Kit = 1
    Core = 2
    Local = 3


class PhysicsBaseTestCase(unittest.TestCase):
    """Simplified base test case for ovruntime physics tests (no Kit dependencies).

    Includes assertion helpers ported from omni.physx.tests assertTestUtils.py.
    """

    def setUp(self):
        self._stage = None

    def tearDown(self):
        pass

    def get_stage(self):
        return self._stage

    def set_stage(self, stage):
        self._stage = stage

    # --- Assertion helpers (ported from assertTestUtils.py) ---

    def _check_physx_object_counts(self, expected_stats: Dict[str, int]) -> None:
        """Checks expected number of objects against the number in the simulation."""
        simulation_stats = _physx.acquire_physxunittests_interface().get_physics_stats()
        for statistic_key, expected_value in expected_stats.items():
            with self.subTest(statistic_key):
                self.assertEqual(simulation_stats[statistic_key], expected_value)

    def assertFloatIterableAlmostEqual(self, iterable1: Iterator[float], iterable2: Iterator[float], delta: float = 0.0, rel_tol: float = None, msg: str = None) -> None:
        """Checks if each pair of values of two float iterables are within the tolerance of delta."""
        self.assertEqual(len(iterable1), len(iterable2), msg="Iterables are not the same length.")
        for item1, item2 in zip(iterable1, iterable2):
            if rel_tol:
                self.assertRelativeAlmostEqual(item1, item2, rel_tol=rel_tol, msg=msg)
            else:
                self.assertAlmostEqual(item1, item2, delta=delta, msg=msg)

    def assertRelativeAlmostEqual(self, expected_value: float, actual_value: float, rel_tol: float = 0.001, msg: str = None):
        """Checks if the actual value is relatively equal to the expected value."""
        delta = abs(rel_tol * expected_value)
        self.assertAlmostEqual(expected_value, actual_value, delta=delta, msg=msg)

    def assertQuaternionAlmostEqual(self, quat1: Union[Gf.Quatd, Gf.Quatf], quat2: Union[Gf.Quatd, Gf.Quatf], delta_deg: float = 0.0, msg: str = None) -> None:
        """Checks if two quaternions are within the tolerance of delta_deg."""
        quat_diff = Gf.Quatf(quat1.GetInverse()) * Gf.Quatf(quat2)
        angle_diff = Gf.Rotation(quat_diff).GetAngle()
        self.assertLessEqual(angle_diff, delta_deg, msg=msg)

    def assertTransformAlmostEqual(self, t0: Gf.Matrix4d, t1: Gf.Matrix4d, delta_lin=1e-6, delta_deg: float = 0.1, delta_det: float = 0.01, msg: str = None) -> None:
        """Checks if two transforms are within the linear and angular tolerances."""
        linDelta = (t0.ExtractTranslation() - t1.ExtractTranslation()).GetLength()
        self.assertLessEqual(linDelta, delta_lin, msg=msg)
        angDeltaDegrees = (t0.ExtractRotation().GetInverse() * t1.ExtractRotation()).GetAngle()
        self.assertLessEqual(angDeltaDegrees, delta_deg, msg=msg)
        self.assertLessEqual(abs(t0.GetDeterminant3() - 1.0), delta_det, msg=msg)
        self.assertLessEqual(abs(t1.GetDeterminant3() - 1.0), delta_det, msg=msg)


class PhysicsMemoryStageBaseTestCase(PhysicsBaseTestCase):
    """Test case using in-memory USD stages with physics simulation."""

    def setUp(self):
        super().setUp()
        self._stage_attached = False

    def tearDown(self):
        self.release_stage(self._stage_attached)
        super().tearDown()

    def new_stage(self, def_up_and_mpu=True, up=UsdGeom.Tokens.y, mpu=0.01, attach_stage=True, file_to_load=""):
        """Creates a new in-memory stage.

        Args:
            def_up_and_mpu: If true the new stage will use the specified up axis and meters per unit.
            up: Up axis token (default: y).
            mpu: Meters per unit (default: 0.01).
            attach_stage: If true the stage will be attached after being created.
            file_to_load: A .usd(a) file to load instead of creating an empty stage.

        Returns:
            The new stage.
        """
        self.release_stage(self._stage_attached)
        if file_to_load:
            self._stage = Usd.Stage.Open(file_to_load)
        else:
            self._stage = Usd.Stage.CreateInMemory()
        cache = UsdUtils.StageCache.Get()
        cache.Insert(self._stage)
        if attach_stage:
            stage_id = cache.GetId(self._stage).ToLongInt()
            _physx.acquire_physx_simulation_interface().attach_stage(stage_id)
            self._stage_attached = True
        else:
            self._stage_attached = False
        if def_up_and_mpu:
            UsdGeom.SetStageUpAxis(self._stage, up)
            UsdGeom.SetStageMetersPerUnit(self._stage, mpu)
        return self._stage

    def release_stage(self, detach_stage=True):
        """Erases the stage from the stage cache.

        Args:
            detach_stage: If true the stage will be detached before being released.
        """
        if self._stage is not None:
            if detach_stage:
                _physx.acquire_physx_simulation_interface().detach_stage()
            cache = UsdUtils.StageCache.Get()
            cache.Erase(cache.GetId(self._stage))
            self._stage = None
            self._stage_attached = not detach_stage

    def attach_stage(self):
        """Attaches _stage to physx simulation.

        Returns:
            The stage ID of the attached stage.
        """
        assert self._stage is not None, "attach_stage: Cannot attach because no memory stage in _stage, use new_stage first."
        cache = UsdUtils.StageCache.Get()
        stage_id = cache.GetId(self._stage).ToLongInt()
        _physx.acquire_physx_simulation_interface().attach_stage(stage_id)
        self._stage_attached = True
        return stage_id

    def detach_stage(self):
        """Detach USD stage from physx simulation."""
        _physx.acquire_physx_simulation_interface().detach_stage()
        self._stage_attached = False

    def step(self, num_steps=1, dt=1.0 / 60.0, reset_simulation_after=False):
        """Executes simulation steps.

        Args:
            num_steps: The number of steps to take.
            dt: The amount of time each step is.
            reset_simulation_after: If true the simulation will be reset after all steps have completed.
        """
        assert self._stage is not None, "step: Cannot step because no memory stage in _stage, use new_stage first."
        if not self._stage_attached:
            self.attach_stage()
        sim = _physx.acquire_physx_simulation_interface()
        for i in range(num_steps):
            sim.simulate(dt, i * dt)
            sim.fetch_results()
        if reset_simulation_after:
            _physx.acquire_physx_interface().reset_simulation()


def check_stats(test_case, expected_stats):
    """Assert that physics simulation stats match expected values.

    Args:
        test_case: unittest.TestCase instance.
        expected_stats: dict of stat name -> expected value.
    """
    sim_stats = _physx.acquire_physxunittests_interface().get_physics_stats()
    for key, value in expected_stats.items():
        test_case.assertEqual(sim_stats[key], value, f"Stats mismatch for '{key}': expected {value}, got {sim_stats[key]}")


class ExpectMessage:
    """Context manager to test for expected log messages during physics operations.

    Ported from omni.physx.scripts.utils.ExpectMessage for standalone use.
    """

    def __init__(self, test_case, expected, expected_result=True, expect_all=True, result_fn=None, partial_string_match=False):
        """
        Args:
            test_case: Test case instance to call assertTrue on.
            expected: A single string or a list of strings of expected messages.
            expected_result: True for presence test, False for absence test.
            expect_all: True to test for presence/absence of all messages in the list.
            result_fn: Callback with the result as parameter instead of assertTrue.
            partial_string_match: If True, match substrings instead of exact strings.
        """
        self._expected = expected
        self._expected_result = expected_result
        self._expect_all = expect_all
        self._result_fn = result_fn
        self._partial_string_match = partial_string_match
        self._logging = carb.logging.acquire_logging()
        self._test_case = test_case

    def __enter__(self):
        def fail(test_case_fail, message):
            if not message.startswith("Test failure because of "):
                test_case_fail(message)

        self._test_case.fail = partial(fail, self._test_case.fail)
        ut = _physx.acquire_physxunittests_interface()
        if type(self._expected) is list:
            ut.start_logger_check_for_multiple(self._expected, self._expected_result, self._expect_all, self._partial_string_match)
        else:
            ut.start_logger_check(self._expected, self._expected_result, self._partial_string_match)
        expt = ("Expecting" if self._expected_result else "Not expecting") + (" all" if self._expect_all else "") + ": "
        print(f"{expt} '{self._expected}'.")

    def __exit__(self, type, value, traceback):
        res = _physx.acquire_physxunittests_interface().end_logger_check()
        print(f"Expectation {'met' if res else 'not met'}.")
        if self._result_fn:
            self._result_fn(res)
        else:
            self._test_case.assertTrue(res)


def add_rigid_box(stage, path, size=Gf.Vec3f(1.0), position=Gf.Vec3f(0.0), orientation=Gf.Quatf(1.0)):
    """Add a rigid body box to the stage.

    Creates a UsdGeom.Cube with RigidBodyAPI, CollisionAPI, and MassAPI (density=1.0).

    Args:
        stage: The Usd.Stage.
        path: The prim path for the cube.
        size: Box size as Gf.Vec3f (scale).
        position: Position as Gf.Vec3f.
        orientation: Orientation as Gf.Quatf.

    Returns:
        The Usd.Prim of the created cube.
    """
    if not isinstance(size, Gf.Vec3f):
        size = Gf.Vec3f(size)

    path = Sdf.Path(path)
    cube_geom = UsdGeom.Cube.Define(stage, path)
    cube_size = 1.0
    half_extent = cube_size / 2
    cube_geom.CreateSizeAttr(cube_size)
    cube_geom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])

    cube_geom.AddTranslateOp().Set(position)
    cube_geom.AddOrientOp().Set(orientation)
    cube_geom.AddScaleOp().Set(size)

    prim = stage.GetPrimAtPath(path)
    UsdPhysics.CollisionAPI.Apply(prim)
    rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(prim)
    rigid_body_api.CreateVelocityAttr().Set(Gf.Vec3f(0.0))
    rigid_body_api.CreateAngularVelocityAttr().Set(Gf.Vec3f(0.0))
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateDensityAttr(1.0)

    return prim
