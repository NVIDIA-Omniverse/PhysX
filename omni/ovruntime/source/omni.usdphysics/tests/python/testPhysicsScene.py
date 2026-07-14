# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import carb
from usdPhysicsBase import UsdPhysicsBaseTest


class UsdPhysicsSceneTest(UsdPhysicsBaseTest):

    def test_scene(self):
        scene_dict = {}
        self.expected_prims = {}

        # expected scene
        scene_dict["gravity"] = carb.Float3(0, 0, -981)
        self.expected_prims["/World/physicsScene" + "/scene"] = scene_dict

        self.parse("PhysicsScene")

    def test_physics_scene_default_gravity(self):
        scene_dict = {}
        self.expected_prims = {}

        # expected scene
        scene_dict["gravity"] = carb.Float3(0, 0, -981)
        self.expected_prims["/World/physicsScene" + "/scene"] = scene_dict

        self.parse("DefaultGravity")
