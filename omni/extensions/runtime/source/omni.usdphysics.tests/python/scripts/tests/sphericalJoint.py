# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
from .base import UsdPhysicsBaseTest


class UsdPhysicsSphericalJointTest(UsdPhysicsBaseTest):

    async def test_spherical_joint(self):
        self.fail_on_log_error = True
        spherical_joint_dict = {}
        self.expected_prims = {}

        # expected revolute joint
        spherical_joint_dict["enabled"] = True
        spherical_joint_dict["body0"] = "/World/box0"
        spherical_joint_dict["body1"] = "/World/box1"
        spherical_joint_dict["local_pose0_position"] = carb.Float3(0, 60, 0)
        spherical_joint_dict["local_pose0_rotation"] = carb.Float4(0, 0, 0, 1)        
        spherical_joint_dict["local_pose1_position"] = carb.Float3(0, -60, 0)
        spherical_joint_dict["local_pose1_rotation"] = carb.Float4(0, 0, 0, 1)
        spherical_joint_dict["axis"] = "Y"
        spherical_joint_dict["limit_enabled"] = True
        spherical_joint_dict["cone_angle0"] = 45.0
        spherical_joint_dict["cone_angle1"] = 45.0
        self.expected_prims["/World/sphericalJoint" + "/sphericalJoint"] = spherical_joint_dict

        await self.parse("SphericalJoint")

    async def test_spherical_joint_disable_cone(self):
        self.fail_on_log_error = True
        spherical_joint_dict = {}
        self.expected_prims = {}

        # expected revolute joint
        spherical_joint_dict["enabled"] = True
        spherical_joint_dict["body0"] = "/World/box0"
        spherical_joint_dict["body1"] = "/World/box1"
        spherical_joint_dict["local_pose0_position"] = carb.Float3(0, 60, 0)
        spherical_joint_dict["local_pose0_rotation"] = carb.Float4(0, 0, 0, 1)        
        spherical_joint_dict["local_pose1_position"] = carb.Float3(0, -60, 0)
        spherical_joint_dict["local_pose1_rotation"] = carb.Float4(0, 0, 0, 1)
        spherical_joint_dict["axis"] = "Y"
        spherical_joint_dict["limit_enabled"] = False
        self.expected_prims["/World/sphericalJoint" + "/sphericalJoint"] = spherical_joint_dict

        await self.parse("SphericalJointNoCone")        
