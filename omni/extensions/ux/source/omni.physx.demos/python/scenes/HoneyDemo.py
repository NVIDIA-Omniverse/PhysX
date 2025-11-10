# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
from pxr import Gf, Sdf, UsdGeom, PhysxSchema
import omni.physxdemos as demo
from omni.physx.scripts import utils
from omni.physx.scripts import physicsUtils
import numpy as np
from omni.physx.scripts.assets_paths import AssetFolders

class HoneyDemo(demo.AsyncDemoBase):
    title = "Honey"
    category = demo.Categories.COMPLEX_SHOWCASES
    short_description = "Honey flowing into a glass jar"
    description = (
        "Demo showcasing the PhysX signed-distance-field (SDF) collision feature in connection "
        "with a highly viscous fluid that produces the characteristic coiling motion."
    )

    kit_settings = {
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
    }

    def __init__(self):
        super().__init__(enable_fabric=False, fabric_compatible=False)
        self.demo_base_usd_url = demo.get_demo_asset_path(AssetFolders.HONEY, "Honey.usd")

    def on_startup(self):
        pass

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage, upAxis = UsdGeom.Tokens.y)
        scenePrim = stage.GetPrimAtPath(defaultPrimPath + "/physicsScene")
        utils.set_physics_scene_asyncsimrender(scenePrim)
        timeStepsPerSecond = 120.0
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scenePrim)
        physxSceneAPI.GetTimeStepsPerSecondAttr().Set(timeStepsPerSecond)
        physxSceneAPI.CreateGpuFoundLostPairsCapacityAttr().Set(10 * 1024)
        physxSceneAPI.CreateGpuCollisionStackSizeAttr().Set(64 * 1024 * 1024)
        physxSceneAPI.CreateGpuMaxRigidContactCountAttr().Set(1024 * 512 * 2)
        physxSceneAPI.CreateGpuMaxRigidPatchCountAttr().Set(1024 * 80 * 2)
        physxSceneAPI.CreateSolverTypeAttr().Set("TGS")

        room = demo.get_demo_room(self, stage, zoom = 0.05, floorOffset = -3.5, pathsToHide = [
            defaultPrimPath + "/DomeLight",
            defaultPrimPath + "/GroundPlane/CollisionMesh",
            defaultPrimPath + "/GroundPlane/CollisionPlane",
            "/Environment/sky_CloudySky/DomeLight"
        ])
        
    def on_shutdown(self):
        super().on_shutdown()
