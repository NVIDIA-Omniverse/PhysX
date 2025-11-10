# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
from .base import UsdPhysicsBaseTest


class UsdPhysicsRigidBodyTest(UsdPhysicsBaseTest):

    async def test_rigid_body(self):
        self.fail_on_log_error = True
        box_col_dict = {}
        self.expected_prims = {}
        box_mesh_col_dict = {}
        box_rb_dict = {}

        # expected box collision
        box_col_dict["rigid_body"] = "/World/BoxActor"
        box_col_dict["material"] = ""
        box_col_dict["enabled"] = True
        box_col_dict["half_extents"] = carb.Float3(12.5, 12.5, 12.5)
        box_col_dict["local_position"] = carb.Float3(0, 0, 0)
        box_col_dict["local_rotation"] = carb.Float4(0, 0, 0, 1)
        box_col_dict["local_scale"] = carb.Float3(1, 1, 1)
        self.expected_prims["/World/BoxActor" + "/cubeShape"] = box_col_dict

        # expected box rigidbody
        box_rb_dict["simulation_owner"] = ""
        box_rb_dict["enabled"] = True
        box_rb_dict["kinematic"] = False
        box_rb_dict["start_asleep"] = False
        box_rb_dict["position"] = carb.Float3(0, 0, 500)
        box_rb_dict["rotation"] = carb.Float4(0, 0, 0, 1)
        box_rb_dict["scale"] = carb.Float3(1, 1, 1)
        box_rb_dict["linear_velocity"] = carb.Float3(2, 1, 2)
        box_rb_dict["angular_velocity"] = carb.Float3(1, 0, 0)
        box_rb_dict["collisions"] = ["/World/BoxActor"]
        self.expected_prims["/World/BoxActor" + "/rigidBody"] = box_rb_dict

        # expected mesh static collision
        box_mesh_col_dict["rigid_body"] = ""
        box_mesh_col_dict["material"] = ""
        box_mesh_col_dict["enabled"] = True
        box_mesh_col_dict["half_extents"] = carb.Float3(750.0, 750.0, 10.0)
        box_mesh_col_dict["local_position"] = carb.Float3(0, 0, 0)
        box_mesh_col_dict["local_rotation"] = carb.Float4(0, 0, 0, 1)
        box_mesh_col_dict["local_scale"] = carb.Float3(750.0, 750.0, 10.0)
        self.expected_prims["/World/Ground" + "/cubeShape"] = box_mesh_col_dict
        await self.parse("RigidBody")
