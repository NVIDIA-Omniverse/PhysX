# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import os
import omni.physxdemos as demo
from .BasicSetup import get_usd_asset_path
from pxr import Gf, Sdf, UsdGeom, PhysxSchema, UsdPhysics, UsdLux

class OmniGraphTriggerConveyorDemo(demo.Base):
    title = "Trigger Conveyor (OmniGraph)"
    category = demo.Categories.TRIGGERS
    short_description = "Show a conveyor logic done entirely in OmniGraph, using the trigger node."
    description = (
        "Demo showcasing how to use the Trigger Nodes. Each trigger will activate different settings for velocities of the conveyor in order to sort the parts in different bins."
    )
    
    def create(self, stage):
        default_prim_path, scene = demo.setup_physics_scene(self, stage, primPathAsString = False)

        main_file_path = get_usd_asset_path("ogn_trigger_conveyor.usda")
        main_path = default_prim_path.AppendChild("main")
        assert (stage.DefinePrim(main_path).GetReferences().AddReference(main_file_path) ) 

        # in the future, the materials should be adjusted so that the default room or museum light is adequate
        lightPath = default_prim_path.AppendPath("demoLight")
        sphereLight = UsdLux.SphereLight.Define(stage, lightPath)
        sphereLight.CreateRadiusAttr(20.0)
        sphereLight.CreateIntensityAttr(50000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 100.0))
        sphereLight.CreateSpecularAttr().Set(0.0)

        basePath = str(default_prim_path) + "/main"
        room = demo.get_demo_room(self, stage, zoom = 0.2, camPitch = 0.25, pathsToHide = [
            basePath + "/SphereLight",
            basePath + "/Camera"
        ])
