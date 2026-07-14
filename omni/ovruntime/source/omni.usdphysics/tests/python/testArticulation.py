# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from usdPhysicsBase import UsdPhysicsBaseTest


class UsdPhysicsArticulationTest(UsdPhysicsBaseTest):

    def test_fixed_articulation(self):
        articulation_dict = {}
        self.expected_prims = {}

        # expected articulation
        articulation_dict["root_prims"] = ["/World/articulation/rootJoint"]
        self.expected_prims["/World/articulation" + "/articulation"] = articulation_dict

        self.parse("FixedArticulation")

    def test_floating_articulation(self):
        articulation_dict = {}
        self.expected_prims = {}

        # expected articulation
        articulation_dict["root_prims"] = ["/World/articulation/rootLink"]
        self.expected_prims["/World/articulation" + "/articulation"] = articulation_dict

        self.parse("FloatingArticulation")

    def test_articulation_maximal_joint(self):
        articulation_dict = {}
        self.expected_prims = {}

        # expected articulation
        articulation_dict["root_prims"] = ["/World/box0"]
        self.expected_prims["/World/box0" + "/articulation"] = articulation_dict

        # expected d6 joint
        d6_joint_dict = {}
        d6_joint_dict["enabled"] = True
        d6_joint_dict["body0"] = "/World/box0"
        d6_joint_dict["body1"] = "/World/box1"
        d6_joint_dict["exclude_from_articulation"] = True
        self.expected_prims["/World/D6DriverJoint" + "/d6Joint"] = d6_joint_dict

        self.parse("ArticulationMaximalJoint")
