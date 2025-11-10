# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
from .base import UsdPhysicsBaseTest


class UsdPhysicsPrismaticJointTest(UsdPhysicsBaseTest):

    async def test_prismatic_joint(self):
        self.fail_on_log_error = True
        prismatic_joint_dict = {}
        self.expected_prims = {}

        # expected prismatic joint
        prismatic_joint_dict["enabled"] = True
        prismatic_joint_dict["body0"] = "/World/StaticBox"
        prismatic_joint_dict["body1"] = "/World/DynamicBox"
        prismatic_joint_dict["local_pose0_position"] = carb.Float3(0, 60, 0)
        prismatic_joint_dict["local_pose0_rotation"] = carb.Float4(0, 0, 0, 1)        
        prismatic_joint_dict["local_pose1_position"] = carb.Float3(0, -60, 0)
        prismatic_joint_dict["local_pose1_rotation"] = carb.Float4(0, 0, 0, 1)
        prismatic_joint_dict["axis"] = "Y"
        prismatic_joint_dict["limit_enabled"] = True
        prismatic_joint_dict["lower"] = -40.0
        prismatic_joint_dict["upper"] = 40.0
        self.expected_prims["/World/PrismaticJoint" + "/prismaticJoint"] = prismatic_joint_dict

        await self.parse("PrismaticJoint")
