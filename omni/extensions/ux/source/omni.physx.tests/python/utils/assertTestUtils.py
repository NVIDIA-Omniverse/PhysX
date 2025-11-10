# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx import get_physxunittests_interface
from omni.kit.commands import execute
from typing import Iterator, Dict, Union
from pxr import Gf


class AssertTestUtils:
    """A class to contain all general assert test helper functions that will be inherited by all test fixtures through
    the base test class"""

    def _check_physx_object_counts(self, expected_stats: Dict[str, int]) -> None:
        """Checks expected number of objects against the number in the simulation. Asserts if they do not match.

        Statistics keys that can be used are:
            'numDynamicRigids': Number of dynamic rigids in simulation.
            'numStaticRigids': Number of static rigids in simulation.
            'numKinematicBodies': Number of kinematic rigids in simulation.
            'numArticulations': Number of articulations in simulation.
            'numSphereShapes': Number of sphere shapes in simulation.
            'numBoxShapes': Number of box shapes in simulation.
            'numCapsuleShapes': Number of capsule shapes in simulation.
            'numCylinderShapes': Number of cylinder shapes in simulation.
            'numConvexShapes': Number of convex shapes in simulation.
            'numTriMeshShapes': Number of triangle mesh shapes in simulation.
            'numPlaneShapes': Number of plane shapes in simulation.
            'numConeShapes': Number of cone shapes in simulation.

        Args:
            expected_stats: The stats that are expected as a dictionary in the format {statistic_key: expected_value}.

        Raises:
            AssertionError: If the object count is not equal to the expected value.
        """
        simulation_stats = get_physxunittests_interface().get_physics_stats()
        for statistic_key, expected_value in expected_stats.items():
            with self.subTest(statistic_key):
                self.assertEqual(simulation_stats[statistic_key], expected_value)

    def _execute_and_check(self, cmd_name: str, **kwargs):
        """Executes a command and checks it executed successfully, returning the result.

        Args:
            cmd_name: Name of the command to be executed.
            **kwargs: Arguments to execute the command with.

        Returns:
            Returns the result of the command.

        Raises:
            AssertionError: If the command fails to execute.
        """
        executed, result = execute(cmd_name, **kwargs)
        self.assertTrue(executed, msg=f"{cmd_name} failed to execute")
        return result

    def assertFloatIterableAlmostEqual(self, iterable1: Iterator[float], iterable2: Iterator[float], delta: float = 0.0, rel_tol: float = None, msg: str = None) -> None:
        """Checks if each pair of values of two float iterables are within the tolerance of delta.

        This can be used to check if two vectors are almost equal.
        iterable1 and 2 must be the same length.

        Args:
            iterable1: A float iterable to check against iterable1.
            iterable2: A float iterable to check against iterable2.
            delta: The tolerance between the two iterables.
            rel_tol: [optional] the relative tolerance between the two iterables. If provided, delta[item] = relTol * iterable1[item].
            msg: [optional] message to output if check fails.

        Raises:
            AssertionError: If the length of both iterables are not equal.
            AssertionError: If the difference between item1 and item2 is greater than the delta.
        """
        self.assertEqual(len(iterable1), len(iterable2), msg="Iterables are not the same length.")
        for item1, item2 in zip(iterable1, iterable2):
            if rel_tol:
                self.assertRelativeAlmostEqual(item1, item2, rel_tol=rel_tol, msg=msg)
            else:
                self.assertAlmostEqual(item1, item2, delta=delta, msg=msg)

    def assertRelativeAlmostEqual(self, expected_value: float, actual_value: float, rel_tol: float, msg: str = None):
        """Checks if the actual value is relatively equal to the expected value.

        Args:
            actual_value: the value to be checked.
            expected_value: the expected value.
            rel_tol: the relative tolerance between the two values.
            msg: [optional] message to output if check fails.

        Raises:
            AssertionError: if the relative difference between the two values is greater than rel_tol.
        """
        delta = abs(rel_tol * expected_value)
        self.assertAlmostEqual(expected_value, actual_value, delta=delta, msg=msg)

    def assertQuaternionAlmostEqual(self, quat1: Union[Gf.Quatd, Gf.Quatf], quat2: Union[Gf.Quatd, Gf.Quatf], delta_deg: float = 0.0, msg: str = None) -> None:
        """Checks if two quaternions are within the tolerance of delta_deg.

        Args:
            quat1: a quaternion to check against quat2.
            quat2: a quaternion to check against quat1.
            delta_deg: the angle of the tolerance in degrees.
            msg: [optional] message to output if check fails.

        Raises:
            AssertionError: if the angle between the two quaternions is greater than delta_deg.
        """
        quat_diff = Gf.Quatf(quat1.GetInverse()) * Gf.Quatf(quat2)
        angle_diff = Gf.Rotation(quat_diff).GetAngle()

        self.assertLessEqual(angle_diff, delta_deg, msg=msg)

    def assertTransformAlmostEqual(self, t0: Gf.Matrix4d, t1: Gf.Matrix4d, delta_lin=1e-6, delta_deg: float=0.1, delta_det: float = 0.01, msg: str = None) -> None:
        """Checks if two transforms are within the linear and angular tolerances and that they have unit determinant.

        Args:
            t0: a transform (as 4x4 matrix) to check against t1.
            t1: a transform (as 4x4 matrix) to check against t0.
            delta_lin: the tolerance for the translational part.
            delta_deg: the tolerance for the rotational part in degrees.
            delta_det: the tolerance for the determinant.
            msg: [optional] message to output if check fails.

        Raises:
            AssertionError: if the transforms are not sufficiently equal.
        """
        linDelta = (t0.ExtractTranslation() - t1.ExtractTranslation()).GetLength()
        self.assertLessEqual(linDelta, delta_lin, msg=msg)

        angDeltaDegrees = (t0.ExtractRotation().GetInverse() * t1.ExtractRotation()).GetAngle()
        self.assertLessEqual(angDeltaDegrees, delta_deg, msg=msg)

        self.assertLessEqual(abs(t0.GetDeterminant3() - 1.0), delta_det, msg=msg)
        self.assertLessEqual(abs(t1.GetDeterminant3() - 1.0), delta_det, msg=msg)
