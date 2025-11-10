# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory
from omni.physxtests import utils
from pxr import UsdGeom, UsdPhysics, PhysxSchema
import os
from omni.physxtests.testBases.articulationTestBase import ArticulationTestBase


class PhysicsJointStateAPITestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase, ArticulationTestBase):
    category = TestCategory.Core

    async def setUp(self):
        await super().setUp()
        await self.articulation_stage_and_scene_setup()

    async def test_articulation_revolute_joint_state(self):
        self._setup_two_link_articulation()

        self._setup_revolute_joint()

        # Start from 45 degrees
        deflection_angle = 45
        self._setup_revolute_joint_state(deflection_angle)
        self.step(num_steps=1)

        # Gravity will pull the angle to about 41 degrees
        deflection_quat = self._axis_angle_to_quat("X", 41.5)
        self.assertQuaternionAlmostEqual(deflection_quat, self._get_rotation_quaternion(self._dynamic_link), delta_deg=0.5)

        joint_state_api = self._joint_states["angular"]

        # Let's change the initial angle to 50 degrees
        deflection_angle = 50
        joint_state_api.GetPositionAttr().Set(deflection_angle)
        self.step(num_steps=1)

        # Gravity will pull the angle to about 43 degrees
        deflection_quat = self._axis_angle_to_quat("X", 43.5)
        self.assertQuaternionAlmostEqual(deflection_quat, self._get_rotation_quaternion(self._dynamic_link), delta_deg=0.5)

    async def test_articulation_revolute_joint_state_save_restore(self):
        filename = "articulationsJointState.usda"

        # Setup the articulation
        self._setup_two_link_articulation()
        self._setup_revolute_joint()
        deflection_angle = 45

        # Setup the joint state API
        self._setup_revolute_joint_state(deflection_angle)

        # Run from 45 degrees
        self.step(num_steps=1)

        # Check that we're few degreees away
        deflection_quat = self._axis_angle_to_quat("X", 41.5)
        self.assertQuaternionAlmostEqual(deflection_quat, self._get_rotation_quaternion(self._dynamic_link), delta_deg=0.5)

        # Export to usda file and load it back
        self._stage.Export(filename)
        await self.new_stage(def_up_and_mpu=False, file_to_load=filename)
        self._dynamic_link = UsdGeom.Cube(self._stage.GetPrimAtPath("/World/dynamicLink"))

        self.step(num_steps=1)

        # Check that we're continuing from where we stopped
        deflection_quat = self._axis_angle_to_quat("X", 34.5)
        self.assertQuaternionAlmostEqual(deflection_quat, self._get_rotation_quaternion(self._dynamic_link), delta_deg=0.5)
        os.remove(filename)

    async def test_articulation_joint_state_D6_translation_error(self):
        self._setup_two_link_articulation()

        d6_joint = self._setup_D6_joint()

        # Lock all but TransX
        d6_prim = d6_joint.GetPrim()
        limit_api = UsdPhysics.LimitAPI.Apply(d6_prim, UsdPhysics.Tokens.transY)
        limit_api.CreateLowAttr(1.0)
        limit_api.CreateHighAttr(-1.0)

        limit_api = UsdPhysics.LimitAPI.Apply(d6_prim, UsdPhysics.Tokens.transZ)
        limit_api.CreateLowAttr(1.0)
        limit_api.CreateHighAttr(-1.0)

        limit_api = UsdPhysics.LimitAPI.Apply(d6_prim, UsdPhysics.Tokens.rotX)
        limit_api.CreateLowAttr(1.0)
        limit_api.CreateHighAttr(-1.0)

        limit_api = UsdPhysics.LimitAPI.Apply(d6_prim, UsdPhysics.Tokens.rotY)
        limit_api.CreateLowAttr(1.0)
        limit_api.CreateHighAttr(-1.0)

        limit_api = UsdPhysics.LimitAPI.Apply(d6_prim, UsdPhysics.Tokens.rotZ)
        limit_api.CreateLowAttr(1.0)
        limit_api.CreateHighAttr(-1.0)

        # Apply Joint State API
        joint_state_api = PhysxSchema.JointStateAPI.Apply(d6_prim, UsdPhysics.Tokens.transX)
        joint_state_api.CreatePositionAttr().Set(45.0)
        joint_state_api.CreateVelocityAttr().Set(0.0)

        message = f"Linear joint states on articulation D6 joints are not supported - state is ignored. Joint: {d6_joint.GetPath().pathString}."

        with utils.ExpectMessage(self, message):
            self.step(reset_simulation_after=True)
