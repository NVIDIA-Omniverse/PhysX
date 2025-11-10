# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from .base import UsdPhysicsBaseTest


class UsdPhysicsArticulationTest(UsdPhysicsBaseTest):

    async def test_fixed_articulation(self):
        self.fail_on_log_error = True
        articulation_dict = {}
        self.expected_prims = {}

        # expected articulation
        articulation_dict["root_prims"] = [ "/World/articulation/rootJoint" ]
        self.expected_prims["/World/articulation" + "/articulation"] = articulation_dict

        await self.parse("FixedArticulation")

    async def test_floating_articulation(self):
        self.fail_on_log_error = True
        articulation_dict = {}
        self.expected_prims = {}

        # expected articulation
        articulation_dict["root_prims"] = [ "/World/articulation/rootLink" ]
        self.expected_prims["/World/articulation" + "/articulation"] = articulation_dict

        await self.parse("FloatingArticulation")

    async def test_articulation_maximal_joint(self):
        self.fail_on_log_error = True
        articulation_dict = {}
        self.expected_prims = {}

        # expected articulation
        articulation_dict["root_prims"] = [ "/World/box0" ]
        self.expected_prims["/World/box0" + "/articulation"] = articulation_dict

        # expected d6 joint
        d6_joint_dict = {}
        d6_joint_dict["enabled"] = True
        d6_joint_dict["body0"] = "/World/box0"
        d6_joint_dict["body1"] = "/World/box1"
        d6_joint_dict["exclude_from_articulation"] = True
        self.expected_prims["/World/D6DriverJoint" + "/d6Joint"] = d6_joint_dict

        await self.parse("ArticulationMaximalJoint")
