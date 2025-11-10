# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
from .base import UsdPhysicsBaseTest


class UsdPhysicsCollisionTest(UsdPhysicsBaseTest):

    async def test_collision(self):
        self.fail_on_log_error = True
        box_col_dict = {}
        self.expected_prims = {}
        mesh_col_dict = {}

        # expected box static collision
        box_col_dict["rigid_body"] = ""
        box_col_dict["material"] = ""
        box_col_dict["enabled"] = True
        box_col_dict["simulation_owner"] = None
        box_col_dict["half_extents"] = carb.Float3(12.5, 12.5, 12.5)
        box_col_dict["local_position"] = carb.Float3(0, 0, 500)
        box_col_dict["local_rotation"] = carb.Float4(0, 0, 0, 1)
        box_col_dict["local_scale"] = carb.Float3(1, 1, 1)
        self.expected_prims["/World/BoxActor" + "/cubeShape"] = box_col_dict

        # expected mesh static collision
        mesh_col_dict["approximation"] = "none"
        mesh_col_dict["rigid_body"] = ""
        mesh_col_dict["material"] = ""
        mesh_col_dict["simulation_owner"] = None
        mesh_col_dict["enabled"] = True
        mesh_col_dict["local_position"] = carb.Float3(0, 0, 0)
        mesh_col_dict["local_rotation"] = carb.Float4(0, 0, 0, 1)
        mesh_col_dict["local_scale"] = carb.Float3(750, 750, 750)
        self.expected_prims["/World/Ground" + "/meshShape"] = mesh_col_dict
        await self.parse("Collision")
