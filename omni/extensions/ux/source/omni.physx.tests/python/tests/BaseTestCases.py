# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
import omni.kit.app
from omni.physxtests import utils
from omni.physxtests.testBases.massTestBase import MassTestBase
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory
from pxr import Gf, UsdGeom, UsdUtils
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physxtests.testBases.articulationTestBase import ArticulationTestBase


class MemoryStageBaseTest(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core

    async def test_new_stage(self):
        stage = await self.new_stage()
        self.assertTrue(UsdUtils.StageCache.Get().Contains(stage))
        self.assertEqual(stage, self._stage)

        new_stage = await self.new_stage()
        self.assertEqual(new_stage, self._stage)

        detached_stage = await self.new_stage(attach_stage=False)
        physicsUtils.add_rigid_box(detached_stage, "/box")
        self._check_physx_object_counts({"numBoxShapes": 0, "numDynamicRigids": 0, "numStaticRigids": 0})

    async def test_release_stage(self):
        stage = await self.new_stage()

        self.release_stage()

        self.assertFalse(UsdUtils.StageCache.Get().Contains(stage))
        self.assertIsNone(self._stage)

    async def test_attach_stage(self):
        with self.assertRaises(AssertionError):
            self.attach_stage()

        await self.new_stage(attach_stage=False)
        physicsUtils.add_rigid_box(self._stage, "/box")
        self.attach_stage()

        self._check_physx_object_counts({"numBoxShapes": 1, "numDynamicRigids": 1, "numStaticRigids": 0})

    async def test_detach_stage(self):
        await self.new_stage(attach_stage=True)
        physicsUtils.add_rigid_box(self._stage, "/box")

        self.detach_stage()
        self._check_physx_object_counts({"numBoxShapes": 0, "numDynamicRigids": 0, "numStaticRigids": 0})

    async def test_kit_stage_not_present_in_test_when_stepping_after_detaching_stage(self):
        await utils.new_stage_setup()
        kit_stage = omni.usd.get_context().get_stage()
        physicsUtils.add_rigid_box(kit_stage, "/box")
        await self.new_stage(attach_stage=False)

        self.step()
        self._check_physx_object_counts({"numBoxShapes": 0, "numDynamicRigids": 0, "numStaticRigids": 0})

        self.release_stage()
        await utils.new_stage_setup()

    def test_step_with_no_memory_stage(self):
        with self.assertRaises(AssertionError):
            self.step()


class ArticulationTestBaseTests(PhysicsMemoryStageBaseAsyncTestCase, ArticulationTestBase):
    category = TestCategory.Core

    def test_axis_angle_to_quat(self):
        with self.subTest(axis="Y"):
            self.assertQuaternionAlmostEqual(self._axis_angle_to_quat("Y", 10.0), Gf.Quatd(0.9961947, 0.0, 0.0871557, 0.0), delta_deg=0.0)
        with self.subTest(axis="Z"):
            self.assertQuaternionAlmostEqual(self._axis_angle_to_quat("Z", 10.0), Gf.Quatd(0.9961947, 0.0, 0.0, 0.0871557), delta_deg=0.0)

        for axis in ("X", Gf.Vec3d.XAxis(), Gf.Vec3f.XAxis()):
            with self.subTest(axis=axis):
                self.assertQuaternionAlmostEqual(self._axis_angle_to_quat(axis, 10.0), Gf.Quatd(0.9961947, 0.0871557, 0.0, 0.0), delta_deg=0.0)

    async def test_get_rotation_angle(self):
        await self.new_stage()
        orientation_angle = 10.0

        box = physicsUtils.add_box(self._stage, "/box", orientation=self._axis_angle_to_quat("X", orientation_angle))

        xform = UsdGeom.Xformable(box)
        self.assertAlmostEqual(self._get_rotation_angle(xform), orientation_angle, delta=0.0001)

    async def test_get_rotation_quaternion(self):
        await self.new_stage()
        orientation = Gf.Quatf(0.5, 0.5, 0.5, 0.5)

        box = physicsUtils.add_box(self._stage, "/box", orientation=orientation)

        xform = UsdGeom.Xformable(box)
        self.assertQuaternionAlmostEqual(self._get_rotation_quaternion(xform), orientation, delta_deg=0.0)


class MassTestBaseTests(PhysicsMemoryStageBaseAsyncTestCase, MassTestBase):
    category = TestCategory.Core

    async def test_calculate_shape_inertia(self):
        await self.mass_test_stage_setup()
        # shape: expected inertia
        expected_results = {
            "cube": Gf.Vec3f(5 / 3),  # uniform cube
            "cuboid": Gf.Vec3f(65 / 6, 25 / 3, 25 / 6),  # nonuniform cube
            "sphere": Gf.Vec3f(4),
            "capsule": Gf.Vec3f(7.28571, 4.42857, 7.28571)
        }

        for shape in expected_results:
            with self.subTest(shape):
                if shape == "cuboid":
                    calculated_inertia = self.calculate_shape_inertia("cube", Gf.Vec3f(1, 2, 3), mass=10, axes_offset=Gf.Vec3f(0))
                else:
                    calculated_inertia = self.calculate_shape_inertia(shape, Gf.Vec3f(1), mass=10, axes_offset=Gf.Vec3f(0))
                self.assertFloatIterableAlmostEqual(expected_results[shape], calculated_inertia, rel_tol=0.001)

    def test_calculate_inertia_parallel_shift(self):
        mass = 10
        initial_inertia = Gf.Vec3f(5 / 3)  # inertia of cube of size 1
        expected_inertia = Gf.Vec3f((5 / 3), (5 / 3) + 10 * 5 * 5, (5 / 3) + 10 * 5 * 5)

        shift = Gf.Vec3f(5, 0, 0)  # shift inertia origin by 5 in x axis
        calculated_inertia = self.calculate_inertia_shift(initial_inertia, shift, mass)
        self.assertFloatIterableAlmostEqual(expected_inertia, calculated_inertia, rel_tol=0.001)

        negative_shift = Gf.Vec3f(-5, 0, 0)  # shift inertia origin by -5 in x axis will result in the same as previous
        calculated_neg_inertia = self.calculate_inertia_shift(initial_inertia, negative_shift, mass)
        self.assertFloatIterableAlmostEqual(expected_inertia, calculated_neg_inertia, rel_tol=0.001)

    async def test_calculate_shape_mass(self):
        # density not provided and defaultDensity not setup
        with self.assertRaises(AssertionError):
            self.calculate_shape_mass("cube", Gf.Vec3f(1))

        # shape: expected mass
        expected_results = {
            "cube": 2,
            "sphere": 8 * math.pi / 3,
            "cylinder": 2 * math.pi,
            "capsule": 14 * math.pi / 3
        }
        # density provided
        density = 2
        for shape in expected_results:
            with self.subTest(shape):
                calculated_mass = self.calculate_shape_mass(shape, Gf.Vec3f(1), density)
                self.assertAlmostEqual(expected_results[shape], calculated_mass)

        # defaultDensity used
        self.defaultDensity = 2
        for shape in expected_results:
            with self.subTest(shape):
                calculated_mass = self.calculate_shape_mass(shape, Gf.Vec3f(1))
                self.assertAlmostEqual(expected_results[shape], calculated_mass)

    async def test_check_shape_mass_properties(self):
        await self.mass_test_stage_setup()
        physicsUtils.add_rigid_box(self._stage, "/cube", density=self.defaultDensity)
        # check when com is none and scale_com is true.
        await self.check_shape_mass_properties("cube", "/cube", Gf.Vec3f(1), Gf.Vec3f(1), scale_com=True)
