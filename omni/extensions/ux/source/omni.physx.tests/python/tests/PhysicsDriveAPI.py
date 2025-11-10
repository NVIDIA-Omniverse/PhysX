# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory
from omni.physx.bindings._physx import SETTING_LOG_ROBOTICS
from omni.physxtests import utils
from omni.physxtests.testBases.articulationTestBase import ArticulationTestBase
from pxr import Usd, Gf, UsdPhysics, UsdGeom
import itertools
from omni.physx.scripts import physicsUtils

class PhysicsDriveAPITestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase, ArticulationTestBase):
    category = TestCategory.Core

    async def setUp(self):
        await super().setUp()
        await self.articulation_stage_and_scene_setup()

        self._setup_two_link_articulation()

    async def test_articulation_spherical_joint_with_attached_d6_drive(self):
        cone_angle_x = 45
        cone_angle_z = 45
        # joint axis is Y, so 0 is Z, and 1 is X
        self._setup_spherical_joint(cone_angle0=cone_angle_z, cone_angle1=cone_angle_x)
        deflection_angle = 4.2  # deg
        # setup D6 so that under gravity the dynamic link deflects by deflection_angle
        self._setup_D6_driver(deflection_angle=deflection_angle)

        self.step(num_steps=40)

        # and check link
        link_angle = self._get_rotation_angle(self._dynamic_link)
        debug_message = f"Target angle = {deflection_angle}; link angle = {link_angle}"
        self.assertAlmostEqual(deflection_angle, link_angle, delta=1.0, msg=debug_message)

    async def test_articulation_revolute_joint_drive(self):
        self._setup_revolute_joint()
        deflection_angle = 5
        self._setup_revolute_drive(deflection_angle)

        self.step(num_steps=30)
        link_angle = self._get_rotation_angle(self._dynamic_link)
        self.assertAlmostEqual(deflection_angle, link_angle, delta=0.5)

    async def test_articulation_revolute_joint_drive_force_limit(self):
        self._setup_revolute_joint()
        deflection_angle = 5
        self._setup_revolute_drive(deflection_angle)

        # First test set the limit to exactly gravity torque so test must succeed
        drive_api = self._drives["angular"]
        gravity_torque = self._gravity_magnitude * self._link_length * self._link_mass * 0.5
        drive_api.CreateMaxForceAttr().Set(gravity_torque)
        self.step(num_steps=45)

        # Reduce and the link must fall
        drive_api.CreateMaxForceAttr().Set(gravity_torque * 0.5)
        self.step(num_steps=30)
        self.assertGreater(self._get_rotation_angle(self._dynamic_link), deflection_angle * 2.0)

    #
    # test that the drive force limit is respected (and that it is treated as a force
    # and not an impulse).
    #
    async def test_prismatic_joint_drive_linear_force_limit(self):
        for i in range(2):
            if i == 0:
                setupAsArticulation = True
            else:
                setupAsArticulation = False

            await self._setup_stage_and_bodies(setupAsArticulation = setupAsArticulation)
            self._setup_joint("prismatic", axis = "X")

            mass = 1.0
            dt = 1.0 / 60.0
            targetVelocity = 1.0

            # expectedVel = (force / mass) * dt
            # => force = (expectedVel / dt) * mass
            # force = targetVelocity * damping = (expectedVel / dt) * mass
            # for this test, choose damping such that expectedVel is targetVelocity if the force is not clamped
            # => damping = mass / dt
            damping = mass / dt
            force = targetVelocity * damping
            maxForce = force / 100.0  # target velocity should not be reached in one sim step due to clamping
            expectedClampedVel = (maxForce / mass) * dt

            massAPI = UsdPhysics.MassAPI(self._dynamic_box)
            massAPI.CreateMassAttr(mass)
            massAPI.CreateDensityAttr(0.0)

            driveAPI = self._setup_joint_drive(type = "linear", damping = damping, targetVel = targetVelocity)
            driveAPI.CreateMaxForceAttr(maxForce)
        
            self.step(1, dt)

            rigidBodyAPI = UsdPhysics.RigidBodyAPI(self._dynamic_box)
            linVel = rigidBodyAPI.GetVelocityAttr().Get()

            self.assertAlmostEqual(expectedClampedVel, linVel[0], delta = expectedClampedVel * 0.01)

    async def test_articulation_prismatic_joint_drive(self): # OM-85167
        stage = await self.new_stage()
        if not stage.GetDefaultPrim():
            root_prim = UsdGeom.Xform.Define(stage, "/World")
            stage.SetDefaultPrim(root_prim.GetPrim())

        physicsUtils.add_rigid_box(stage, "/rootLink")
        dynamic_link_prim = physicsUtils.add_rigid_box(stage, "/dynamicLink")
        fixed_joint_path = stage.GetDefaultPrim().GetPath().AppendChild("baseFixedJoint")
        fixed_base_joint = UsdPhysics.FixedJoint.Define(stage, fixed_joint_path)
        fixed_base_joint.CreateBody1Rel().SetTargets(["/World/rootLink"])
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetDefaultPrim())

        joint = UsdPhysics.PrismaticJoint.Define(stage, "/World/dynamicLink/PrismaticJoint")
        joint.CreateBody0Rel().SetTargets(["/World/rootLink"])
        joint.CreateBody1Rel().SetTargets(["/World/dynamicLink"])
        axis = "X"
        joint.CreateAxisAttr(axis)

        drive_api = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive_api.CreateStiffnessAttr().Set(9999999999999999)
        target = 10
        drive_api.GetTargetPositionAttr().Set(value=target)
        for _ in range(1):
            self.step()

        pos = dynamic_link_prim.GetAttribute("xformOp:translate").Get()
        epsilon = 0.0001
        self.assertTrue(pos[0] > target - epsilon and pos[0] < target + epsilon )

    async def test_articulation_revolute_joint_drive_limit_wrapped(self):
        self.deflection_angle = 360
        self.joint = self._setup_revolute_joint()
        self._setup_revolute_drive(self.deflection_angle)
        self._drives["angular"].GetTargetPositionAttr().Set(self.deflection_angle - 1)
        self.settings = carb.settings.get_settings()
        self.settings.set(SETTING_LOG_ROBOTICS, True)

        message = f"Physics USD: Drive position target set to {self.deflection_angle + 1} on {self.joint.GetPath()} will be wrapped in [-360, 360] range.Consider setting explicit limits to enable use of unwrapped joints"
        # As we have no setup limits and we are at 359 degrees, we don't expect the warning
        with utils.ExpectMessage(self, message, expected_result=False):
            self.step()

        # Now we change Drive Target to 361 during sim, and this will yield a warning
        with utils.ExpectMessage(self, message, expected_result=True):
            self._drives["angular"].GetTargetPositionAttr().Set(self.deflection_angle + 1)
            self.step(reset_simulation_after=True)

        # Drive target is not reset after simulation stop, so it's still 361 and we expect warning on sim start
        with utils.ExpectMessage(self, message, expected_result=True):
            self.step(reset_simulation_after=True)

        # Setting up limits will force use of unwrapped joint, drive target is still 361 but no warning is issued
        with utils.ExpectMessage(self, message, expected_result=False):
            self.joint.CreateLowerLimitAttr(-720)
            self.joint.CreateUpperLimitAttr(+720)
            self.step(reset_simulation_after=False)

        self.settings.set(SETTING_LOG_ROBOTICS, False)

    async def test_drive_time_sampled_values(self):
        joint_types = ["Revolute", "Prismatic", "D6"]
        drive_types = ["pos", "vel"]
        axes = ["X", "Y", "Z"]
        articulation = [True, False]

        angular_target = 20  # pos: 20deg, vel: 20deg/s
        linerar_target = 1  # pos: 1m, vel: 1m/s

        for joint_type, drive_type, axis, articulation in itertools.product(joint_types, drive_types, axes, articulation):
            with self.subTest(joint_type=joint_type, drive_type=drive_type, axis=axis, articulation=articulation):
                if joint_type == "Revolute":
                    target = angular_target
                    await self._revolute_test_setup(axis=axis, driveType=drive_type, setupAsArticulation=articulation, targetValue=target)
                elif joint_type == "Prismatic":
                    target = linerar_target
                    await self._prismatic_test_setup(axis=axis, driveType=drive_type, setupAsArticulation=articulation, targetValue=target)
                elif joint_type == "D6":
                    target = angular_target
                    await self._d6_revolute_test_setup(axis=axis, driveType=drive_type, setupAsArticulation=articulation, targetValue=target)

                if axis == "X":
                    target_vec3 = Gf.Vec3d(target, 0, 0)
                elif axis == "Y":
                    target_vec3 = Gf.Vec3d(0, target, 0)
                elif axis == "Z":
                    target_vec3 = Gf.Vec3d(0, 0, target)

                if drive_type == "pos":
                    self._driveAPI.CreateStiffnessAttr().Set(9999999999999999)
                    self._driveAPI.GetTargetPositionAttr().Set(value=0, time=Usd.TimeCode(0))
                    self._driveAPI.GetTargetPositionAttr().Set(value=target, time=Usd.TimeCode(1))

                    self.step(6, reset_simulation_after=False)

                    if joint_type == "Prismatic":
                        position = self._dynamic_box.GetPrim().GetAttribute('xformOp:translate').Get()
                        self.assertFloatIterableAlmostEqual(position, target_vec3, delta=1)

                    else:
                        orient = Gf.Quatd(self._dynamic_box.GetPrim().GetAttribute('xformOp:orient').Get())
                        self.assertQuaternionAlmostEqual(orient, self._axis_angle_to_quat(axis, target), 1)

                elif drive_type == "vel":
                    self._driveAPI.CreateDampingAttr().Set(9999999999999999)
                    self._driveAPI.GetTargetVelocityAttr().Set(value=0, time=Usd.TimeCode(0))
                    self._driveAPI.GetTargetVelocityAttr().Set(value=target, time=Usd.TimeCode(1))

                    self.step(6, reset_simulation_after=False)

                    bodyAPI = UsdPhysics.RigidBodyAPI(self._dynamic_box)
                    self.assertTrue(bodyAPI)
                    if joint_type == "Prismatic":
                        vel = bodyAPI.GetVelocityAttr().Get()
                    else:
                        vel = bodyAPI.GetAngularVelocityAttr().Get()
                    self.assertFloatIterableAlmostEqual(vel, target_vec3, delta=1)
