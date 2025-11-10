# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
from .base import UsdPhysicsBaseTest


class UsdPhysicsFixedJointTest(UsdPhysicsBaseTest):

    async def test_fixed_joint(self):
        self.fail_on_log_error = True
        fixed_joint_dict = {}
        self.expected_prims = {}

        # expected fixed joint        
        fixed_joint_dict["enabled"] = True
        fixed_joint_dict["body0"] = "/World/StaticBox"
        fixed_joint_dict["body1"] = "/World/DynamicBox"
        fixed_joint_dict["local_pose0_position"] = carb.Float3(0, 60, 0)
        fixed_joint_dict["local_pose0_rotation"] = carb.Float4(0, 0, 0, 1)        
        fixed_joint_dict["local_pose1_position"] = carb.Float3(0, -60, 0)
        fixed_joint_dict["local_pose1_rotation"] = carb.Float4(0, 0, 0, 1)
        self.expected_prims["/World/FixedJoint" + "/fixedJoint"] = fixed_joint_dict

        await self.parse("FixedJoint")
