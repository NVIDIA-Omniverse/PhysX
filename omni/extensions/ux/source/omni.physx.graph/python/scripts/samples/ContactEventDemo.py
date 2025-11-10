# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import os
import omni.physxdemos as demo
from .BasicSetup import get_usd_asset_path
from omni.physx.scripts import utils
from omni.physx.scripts import physicsUtils
from pxr import Gf, Sdf, UsdGeom, PhysxSchema, UsdPhysics, UsdLux
import carb.settings

class OmniGraphContactEventDemo(demo.Base):
    title = "Contact Event (OmniGraph)"
    category = demo.Categories.CONTACTS
    short_description = "Demo of using Contact Event OmniGraph nodes with ActionGraph to make bumpers respond to pinballs."
    description = "A demonstration of how to use PhysX Contact Event OmniGraph nodes with ActionGraph to make bumpers " \
        "respond to pinballs."
        
    params = {"pinballs": demo.IntParam(1, 1, 250, 1),
              "major_rows": demo.IntParam(2, 2, 25, 1),
              "major_columns": demo.IntParam(2, 2, 25, 1)}

    def create(self, stage, pinballs, major_rows, major_columns):
        default_prim_path, scene = demo.setup_physics_scene(self, stage, primPathAsString = False, upAxis = UsdGeom.Tokens.y)

        main_file_path = get_usd_asset_path("ogn_contact_main.usda")
        main_path = default_prim_path.AppendChild("main")
        assert (stage.DefinePrim(main_path).GetReferences().AddReference(main_file_path) ) 

        base = stage.GetPrimAtPath(main_path.AppendPath("Base"))

        xScale = (major_rows - 1) * 20.0 + 25.0
        zScale = (major_columns - 1) * 20.0 + 25.0
        base.GetAttribute("xformOp:scale").Set(Gf.Vec3d(xScale, 5.0, zScale))
        
        for side in range(4):
            side_path = main_path.AppendPath(f"Side_0{side+1}")
            if side % 2 == 0.0:
                width = (major_columns - 1) * 20.0
                height = (major_rows - 1) * 20.0
            else:
                width = (major_rows - 1) * 20.0
                height = (major_columns - 1) * 20.0

            side_edge = stage.GetPrimAtPath(side_path.AppendPath("Edge"))
            side_edge.GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, width / 2) + side_edge.GetAttribute("xformOp:translate").Get())
            side_edge.GetAttribute("xformOp:scale").Set(Gf.Vec3d(height, 0.0, 0.0) + side_edge.GetAttribute("xformOp:scale").Get())
            side_edge_inclined = stage.GetPrimAtPath(side_path.AppendPath("Edge_inclined"))
            side_edge_inclined.GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, width / 2) + side_edge_inclined.GetAttribute("xformOp:translate").Get())
            side_edge_inclined.GetAttribute("xformOp:scale").Set(Gf.Vec3d(height, 0.0, 0.0) + side_edge_inclined.GetAttribute("xformOp:scale").Get())

            side_corner = stage.GetPrimAtPath(side_path.AppendPath("Corner"))
            side_corner.GetAttribute("xformOp:translate").Set(Gf.Vec3d(height / 2, 0.0, width / 2) + side_corner.GetAttribute("xformOp:translate").Get())
            side_corner_inclined = stage.GetPrimAtPath(side_path.AppendPath("Corner_inclined"))
            side_corner_inclined.GetAttribute("xformOp:translate").Set(Gf.Vec3d(height / 2, 0.0, width / 2) + side_corner_inclined.GetAttribute("xformOp:translate").Get())

        bumper_file_path = get_usd_asset_path("ogn_contact_bumper.usda")
        bumper_position = Gf.Vec3f(-10.0 * (major_rows - 1), 0.0, -10.0 * (major_columns - 1))
        for row in range(major_rows * 2 - 1):
            for column in range(major_columns - row % 2):
                bumper_path = default_prim_path.AppendChild(f"bumper_{row}_{column}")
                assert (stage.DefinePrim(bumper_path).GetReferences().AddReference(bumper_file_path))
                bumper = stage.GetPrimAtPath(bumper_path)
                bumper.GetAttribute("xformOp:translate").Set(bumper_position)
                bumper_position[2] += 20.0
                bumper_top = stage.GetPrimAtPath(f"{bumper_path}" + "/Bumper/BumperRigidBody/Bumper_top")
                filter_api = UsdPhysics.FilteredPairsAPI(bumper_top)
                rel = filter_api.GetFilteredPairsRel()
                rel.AddTarget(Sdf.Path(base.GetPrimPath()))

            bumper_position[0] += 10.0
            bumper_position[2] = -10.0 * (major_columns - 1)
            if row % 2 == 0.0:
                bumper_position[2] += 10.0

        pinball_file_path = get_usd_asset_path("ogn_contact_pinball.usda")
        pinballs_per_side = int(pinballs / 4)
        pinballs_num = [pinballs_per_side] * 4
        if pinballs % 4 > 0.0:
            pinballs_num[0] += 1
        if pinballs % 4 > 1.0:
            pinballs_num[2] += 1
        if pinballs % 4 > 2.0:
            pinballs_num[1] += 1

        offset_adjustment = 2.5
        vertical_spacing = 3.0
        for side in range(4):
            if pinballs_num[side] < 1:
                continue
            
            if side % 2 == 0.0:
                spacing = (major_rows) * 20.0 / pinballs_num[side]
                origin = Gf.Vec3d(-(major_rows) * 10.0 + spacing * 0.5 + offset_adjustment, 20.0, (major_columns) * 10.0)
                if side == 2:
                    origin[0] = -origin[0]
                    origin[2] = -origin[2]
                    spacing = -spacing
                offset = Gf.Vec3d(spacing, vertical_spacing, 0.0)
            else:
                spacing = (major_columns) * 20.0 / pinballs_num[side]
                origin = Gf.Vec3d(-(major_rows) * 10.0, 20.0, -(major_columns) * 10.0 + spacing * 0.5 + offset_adjustment )
                if side == 3:
                    origin[0] = -origin[0]
                    origin[2] = -origin[2]
                    spacing = -spacing
                offset = Gf.Vec3d(0.0, vertical_spacing, spacing)

            for pinball in range(pinballs_num[side]):
                pinball_path = default_prim_path.AppendChild(f"pinball_{side}_{pinball}")
                assert (stage.DefinePrim(pinball_path).GetReferences().AddReference(pinball_file_path))
                xform = UsdGeom.Xform.Get(stage, pinball_path)
                physicsUtils.set_or_add_translate_op(xform, translate=(origin + offset * pinball))
                prim = stage.GetPrimAtPath(pinball_path.AppendChild("Pinball"))
                physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                physxCollisionAPI.CreateContactOffsetAttr(1.5)


        # in the future, the materials should be adjusted so that the default room or museum light is adequate
        lightPath = default_prim_path.AppendPath("demoLight")
        sphereLight = UsdLux.SphereLight.Define(stage, lightPath)
        sphereLight.CreateRadiusAttr(20.0)
        sphereLight.CreateIntensityAttr(50000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(0.0, 50.0, 0.0))
        sphereLight.CreateSpecularAttr().Set(0.1)

        basePath = str(default_prim_path) + "/main"
        room = demo.get_demo_room(self, stage, floorOffset = -5.0, camElevation = 10.0, camPitch = 0.25, zoom = 0.1, tableSurfaceDim = Gf.Vec2f(zScale + 50.0, xScale + 50.0), pathsToHide = [
            basePath + "/DiskLight"
        ])
