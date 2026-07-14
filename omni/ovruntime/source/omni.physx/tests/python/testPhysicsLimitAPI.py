# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Ported from omni.physx.tests PhysicsLimitAPI.py for standalone ovruntime testing.

import _physics_setup  # noqa: F401

import unittest
import carb
from physicsBase import PhysicsMemoryStageBaseTestCase, TestCategory, ExpectMessage
import _physx
from pxr import Gf
from articulationTestBase import ArticulationTestBase

SETTING_LOG_ROBOTICS = _physx.SETTING_LOG_ROBOTICS


class PhysicsLimitAPITestMemoryStage(PhysicsMemoryStageBaseTestCase, ArticulationTestBase):
    category = TestCategory.Core

    def setUp(self):
        super().setUp()
        self.articulation_stage_and_scene_setup()

        self._setup_two_link_articulation()

    def test_articulation_spherical_joint_limit_cone_angle1(self):
        cone_angle_x = 45
        cone_angle_z = 10
        # joint axis is Y, so 0 is Z, and 1 is X
        self._setup_spherical_joint(cone_angle0=cone_angle_z, cone_angle1=cone_angle_x)

        self.step(num_steps=10)
        link_angle = self._get_rotation_angle(self._dynamic_link)
        self.assertAlmostEqual(cone_angle_x, link_angle, delta=0.1)

    def test_articulation_spherical_joint_limit_cone_angle0(self):
        cone_angle_x = 45
        cone_angle_z = 10
        # joint axis is Y, so 0 is Z, and 1 is X
        self._setup_spherical_joint(cone_angle0=cone_angle_z, cone_angle1=cone_angle_x)
        self._scene.GetGravityDirectionAttr().Set(Gf.Vec3f(-1.0, 0.0, 0.0))

        self.step(num_steps=10)
        link_angle = self._get_rotation_angle(self._dynamic_link)
        self.assertAlmostEqual(cone_angle_z, link_angle, delta=0.1)

    def test_articulation_revolute_joint_limits(self):
        upper = 20
        lower = -10
        self._setup_revolute_joint(limits=(lower, upper))

        self.step(num_steps=20)
        link_angle = self._get_rotation_angle(self._dynamic_link)
        self.assertAlmostEqual(-lower, link_angle, delta=0.1)

    def test_articulation_revolute_joint_drive_limit_warnings(self):
        self.deflection_angle = 360
        self.joint = self._setup_revolute_joint()
        self._setup_revolute_drive(self.deflection_angle)
        self._drives["angular"].GetTargetPositionAttr().Set(self.deflection_angle + 1)
        self.settings = carb.settings.get_settings()
        self.settings.set(SETTING_LOG_ROBOTICS, True)

        self.joint.CreateLowerLimitAttr(-720)
        self.joint.CreateUpperLimitAttr(+720)
        self.step(reset_simulation_after=False)

        # Try removing limits during runtime, that is currently unsupported as requires the joint to be removed and
        # readded to the scene
        message = f"Cannot change low limit to -inf on {self.joint.GetPath()} during simulation. This change requires a simulation restart."
        with ExpectMessage(self, message, expected_result=True):
            self.joint.CreateLowerLimitAttr(float('-inf'))
            self.step()

        message = f"Cannot change high limit to inf on {self.joint.GetPath()} during simulation. This change requires a simulation restart."
        with ExpectMessage(self, message, expected_result=True):
            self.joint.CreateUpperLimitAttr(float('inf'))
            self.step(reset_simulation_after=True)

        # Now create the joint without limits (will be created as regular WRAPPED joint)
        # Let's also set target drive to 0 so we don't get previously checked warnings anymore
        self._drives["angular"].GetTargetPositionAttr().Set(0)
        self.step()

        # now we expect the error to be risen anyway as we cannot switch to unwrapped joint (or even change setMotion state)
        message = f"Cannot update articulation joint low limit because the joint was not initially configured with limits: ({self.joint.GetPath()})"
        with ExpectMessage(self, message, expected_result=True):
            self.joint.CreateLowerLimitAttr(-720)
            self.step()

        message = f"Cannot update articulation joint high limit because the joint was not initially configured with limits: ({self.joint.GetPath()})"
        with ExpectMessage(self, message, expected_result=True):
            self.joint.CreateUpperLimitAttr(+720)
            self.step(reset_simulation_after=True)

        self.settings.set(SETTING_LOG_ROBOTICS, False)


if __name__ == "__main__":
    unittest.main()
