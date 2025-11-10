# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
from .base import UsdPhysicsBaseTest


class UsdPhysicsD6JointTest(UsdPhysicsBaseTest):

    async def test_d6_joint(self):
        self.fail_on_log_error = True
        d6_joint_dict = {}
        self.expected_prims = {}

        # expected d6 joint
        d6_joint_dict["enabled"] = True
        d6_joint_dict["body0"] = "/World/box0"
        d6_joint_dict["body1"] = "/World/box1"
        d6_joint_dict["local_pose0_position"] = carb.Float3(0, 60, 0)
        d6_joint_dict["local_pose0_rotation"] = carb.Float4(0, 0, 0, 1)        
        d6_joint_dict["local_pose1_position"] = carb.Float3(0, -60, 0)
        d6_joint_dict["local_pose1_rotation"] = carb.Float4(0, 0, 0, 1)        
        d6_joint_dict["limit_rot_x_enabled"] = True
        d6_joint_dict["limit_rot_x_lower"] = 1.0
        d6_joint_dict["limit_rot_x_upper"] = -1.0
        d6_joint_dict["limit_rot_y_enabled"] = True
        d6_joint_dict["limit_rot_y_lower"] = 1.0
        d6_joint_dict["limit_rot_y_upper"] = -1.0
        d6_joint_dict["limit_rot_z_enabled"] = True
        d6_joint_dict["limit_rot_z_lower"] = 1.0
        d6_joint_dict["limit_rot_z_upper"] = -1.0
        d6_joint_dict["limit_trans_x_enabled"] = True
        d6_joint_dict["limit_trans_x_lower"] = 1.0
        d6_joint_dict["limit_trans_x_upper"] = -1.0
        d6_joint_dict["limit_trans_y_enabled"] = True
        d6_joint_dict["limit_trans_y_lower"] = 1.0
        d6_joint_dict["limit_trans_y_upper"] = -1.0
        d6_joint_dict["limit_trans_z_enabled"] = True
        d6_joint_dict["limit_trans_z_lower"] = 1.0
        d6_joint_dict["limit_trans_z_upper"] = -1.0
        self.expected_prims["/World/d6FixedJoint" + "/d6Joint"] = d6_joint_dict

        await self.parse("D6Joint")
