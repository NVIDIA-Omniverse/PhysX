# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from usdPhysicsBase import UsdPhysicsBaseTest


class UsdPhysicsCollisionGroupTest(UsdPhysicsBaseTest):

    def test_collision_group(self):
        col_group_dict = {}
        self.expected_prims = {}

        # expected collision group
        self.expected_prims["/World/collisionGroupSpheres" + "/collisionGroup"] = col_group_dict
        self.parse("CollisionGroup")
