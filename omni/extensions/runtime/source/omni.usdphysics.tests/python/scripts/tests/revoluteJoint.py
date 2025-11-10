# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
from .base import UsdPhysicsBaseTest


class UsdPhysicsRevoluteJointTest(UsdPhysicsBaseTest):

    async def test_revolute_joint(self):
        self.fail_on_log_error = True
        revolute_joint_dict = {}
        self.expected_prims = {}

        # expected revolute joint
        revolute_joint_dict["enabled"] = True
        revolute_joint_dict["body0"] = "/World/box0"
        revolute_joint_dict["body1"] = "/World/box1"
        revolute_joint_dict["local_pose0_position"] = carb.Float3(0, 60, 0)
        revolute_joint_dict["local_pose0_rotation"] = carb.Float4(0, 0, 0, 1)        
        revolute_joint_dict["local_pose1_position"] = carb.Float3(0, -60, 0)
        revolute_joint_dict["local_pose1_rotation"] = carb.Float4(0, 0, 0, 1)
        revolute_joint_dict["axis"] = "X"
        revolute_joint_dict["limit_enabled"] = True
        revolute_joint_dict["lower"] = -90.0
        revolute_joint_dict["upper"] = 90.0
        revolute_joint_dict["drive_angular_damping"] = 10000000000.0
        revolute_joint_dict["drive_angular_acceleration"] = False
        revolute_joint_dict["drive_angular_stiffness"] = 0.0
        revolute_joint_dict["drive_angular_force_limit"] = 1e20
        revolute_joint_dict["drive_angular_target_position"] = 0.0
        revolute_joint_dict["drive_angular_target_velocity"] = -1.0
        self.expected_prims["/World/revoluteJoint" + "/revoluteJoint"] = revolute_joint_dict

        await self.parse("RevoluteJoint")
        
    async def test_revolute_joint_scale(self):
        self.fail_on_log_error = True
        revolute_joint_dict = {}
        self.expected_prims = {}

        # expected revolute joint
        revolute_joint_dict["enabled"] = True
        revolute_joint_dict["body0"] = ""
        revolute_joint_dict["body1"] = "/World/box1"
        revolute_joint_dict["local_pose0_position"] = carb.Float3(0, 30, 1000)
        revolute_joint_dict["local_pose0_rotation"] = carb.Float4(0, 0, 0.70710677, 0.70710677)        
        revolute_joint_dict["local_pose1_position"] = carb.Float3(0, -6, 0)
        revolute_joint_dict["local_pose1_rotation"] = carb.Float4(0, 0, 0, 1)
        revolute_joint_dict["axis"] = "X"
        revolute_joint_dict["limit_enabled"] = True
        revolute_joint_dict["lower"] = -90.0
        revolute_joint_dict["upper"] = 90.0
        revolute_joint_dict["drive_angular_damping"] = 10000000000.0
        revolute_joint_dict["drive_angular_acceleration"] = False
        revolute_joint_dict["drive_angular_stiffness"] = 0.0
        revolute_joint_dict["drive_angular_force_limit"] = 1e20
        revolute_joint_dict["drive_angular_target_position"] = 0.0
        revolute_joint_dict["drive_angular_target_velocity"] = -1.0
        self.expected_prims["/World/revoluteJoint" + "/revoluteJoint"] = revolute_joint_dict

        await self.parse("RevoluteJointScale")
