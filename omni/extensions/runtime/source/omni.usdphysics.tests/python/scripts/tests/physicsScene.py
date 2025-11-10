# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
from .base import UsdPhysicsBaseTest


class UsdPhysicsSceneTest(UsdPhysicsBaseTest):

    async def test_scene(self):
        self.fail_on_log_error = True
        scene_dict = {}
        self.expected_prims = {}

        # expected scene 
        scene_dict["gravity"] = carb.Float3(0, 0, -981)
        self.expected_prims["/World/physicsScene" + "/scene"] = scene_dict

        await self.parse("PhysicsScene")

    async def test_physics_scene_default_gravity(self):
        self.fail_on_log_error = True
        scene_dict = {}
        self.expected_prims = {}

        # expected scene 
        scene_dict["gravity"] = carb.Float3(0, 0, -981)
        self.expected_prims["/World/physicsScene" + "/scene"] = scene_dict

        await self.parse("DefaultGravity")
