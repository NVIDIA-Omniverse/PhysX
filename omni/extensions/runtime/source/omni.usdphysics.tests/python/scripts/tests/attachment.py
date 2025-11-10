# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
from .base import UsdPhysicsBaseTest


class UsdPhysicsAttachmentTest(UsdPhysicsBaseTest):

    async def test_attachment(self):
        self.fail_on_log_error = True
        self.expected_prims = {}

        vtxvtx_dict = {}
        vtxvtx_dict["enabled"] = False
        vtxvtx_dict["src0"] = "/World/Actor0"
        vtxvtx_dict["src1"] = "/World/Actor1"
        vtxvtx_dict["stiffness"] = float(1.0)
        vtxvtx_dict["damping"] = float(10.0)
        self.expected_prims["/World/VtxVtxAttachment" + "/vtxVtxAttachment"] = vtxvtx_dict

        vtxtri_dict = {}
        vtxtri_dict["enabled"] = True
        vtxtri_dict["src0"] = "/World/Actor0"
        vtxtri_dict["src1"] = "/World/Actor1"
        vtxtri_dict["stiffness"] = float(1.0)
        vtxtri_dict["damping"] = float(10.0)
        self.expected_prims["/World/VtxTriAttachment" + "/vtxTriAttachment"] = vtxtri_dict

        vtxtet_dict = {}
        vtxtet_dict["enabled"] = True
        vtxtet_dict["src0"] = "/World/Actor0"
        vtxtet_dict["src1"] = "/World/Actor1"
        vtxtet_dict["stiffness"] = float(1.0)
        vtxtet_dict["damping"] = float(10.0)
        self.expected_prims["/World/VtxTetAttachment" + "/vtxTetAttachment"] = vtxtet_dict

        vtxcrv_dict = {}
        vtxcrv_dict["enabled"] = True
        vtxcrv_dict["src0"] = "/World/Actor0"
        vtxcrv_dict["src1"] = "/World/Actor1"
        vtxcrv_dict["stiffness"] = float(1.0)
        vtxcrv_dict["damping"] = float(10.0)
        self.expected_prims["/World/VtxCrvAttachment" + "/vtxCrvAttachment"] = vtxcrv_dict

        vtxxform_dict = {}
        vtxxform_dict["enabled"] = True
        vtxxform_dict["src0"] = "/World/Actor0"
        vtxxform_dict["src1"] = "/World/Actor1"
        vtxxform_dict["stiffness"] = float(1.0)
        vtxxform_dict["damping"] = float(10.0)
        self.expected_prims["/World/VtxXformAttachment" + "/vtxXformAttachment"] = vtxxform_dict

        tetxform_dict = {}
        tetxform_dict["enabled"] = True
        tetxform_dict["src0"] = "/World/Actor0"
        tetxform_dict["src1"] = "/World/Actor1"
        tetxform_dict["stiffness"] = float(1.0)
        tetxform_dict["damping"] = float(10.0)
        self.expected_prims["/World/TetXformAttachment" + "/tetXformAttachment"] = tetxform_dict

        tritri_dict = {}
        tritri_dict["enabled"] = True
        tritri_dict["src0"] = "/World/Actor0"
        tritri_dict["src1"] = "/World/Actor1"
        tritri_dict["stiffness"] = float(1.0)
        tritri_dict["damping"] = float(10.0)
        self.expected_prims["/World/TriTriAttachment" + "/triTriAttachment"] = tritri_dict

        await self.parse("Attachment")


    async def test_collision_filter(self):
        self.fail_on_log_error = True
        collision_filter_dict = {}
        self.expected_prims = {}

        collision_filter_dict["enabled"] = True
        collision_filter_dict["src0"] = "/World/Actor0"
        collision_filter_dict["src1"] = "/World/Actor1"
        self.expected_prims["/World/CollisionFilter" + "/collisionFilter"] = collision_filter_dict

        await self.parse("Attachment")
