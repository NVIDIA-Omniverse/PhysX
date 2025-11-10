# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
from .base import UsdPhysicsBaseTest


class UsdPhysicsDistanceJointTest(UsdPhysicsBaseTest):

    async def test_distance_joint(self):
        self.fail_on_log_error = True
        distance_joint_dict = {}
        self.expected_prims = {}

        # expected distance joint
        distance_joint_dict["enabled"] = True
        distance_joint_dict["body0"] = "/World/StaticBox"
        distance_joint_dict["body1"] = "/World/DynamicBox"
        distance_joint_dict["local_pose0_position"] = carb.Float3(0, 60, 0)
        distance_joint_dict["local_pose0_rotation"] = carb.Float4(0, 0, 0, 1)        
        distance_joint_dict["local_pose1_position"] = carb.Float3(0, -60, 0)
        distance_joint_dict["local_pose1_rotation"] = carb.Float4(0, 0, 0, 1)
        distance_joint_dict["min_enabled"] = True
        distance_joint_dict["min_distance"] = 10.0
        distance_joint_dict["max_enabled"] = True
        distance_joint_dict["max_distance"] = 50.0
        self.expected_prims["/World/DistanceJoint" + "/distanceJoint"] = distance_joint_dict

        await self.parse("DistanceJoint")
