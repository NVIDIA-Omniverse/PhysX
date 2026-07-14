# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
import omni.physxdemos as demo
import omni.physx.bindings._physx as physx_settings_bindings
from omni.physx.scripts.assets_paths import AssetFolders
import carb

class TeddyOnIceDemo(demo.Base):
    title = "Teddy On Ice"
    category = demo.Categories.DEFORMABLES
    short_description = "Demo showing a teddy bear with frozen ice cubes attached to its feet"
    description = "The teddy bear is simulated as a deformable body, and is attached to rigid-body ice-cubes at its feet."

    kit_settings = {
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
        physx_settings_bindings.SETTING_MIN_FRAME_RATE: 30,
        physx_settings_bindings.SETTING_UPDATE_VELOCITIES_TO_USD: False,
        physx_settings_bindings.SETTING_UPDATE_TO_USD: True,
        "rtx/post/aa/op": 3,
        "rtx/post/dlss/execMode": 1,
        "rtx/translucency/maxRefractionBounces": 6
    }

    def __init__(self):
        self.demo_base_usd_url = demo.get_demo_asset_path(AssetFolders.TEDDY_ON_ICE, "TeddyOnIce.usd")

    def create(self, stage):
        # Asset already has a PhysicsScene; skip demo.setup_physics_scene to avoid
        # ending up with both `/World/PhysicsScene` (asset) and `/World/physicsScene`
        # (helper) on the stage.
        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        room = demo.get_demo_room(self, stage, staticDynamicRestitution = Gf.Vec3f(0.1, 0.1, 1.0), hasTable = False, pathsToHide = [
            defaultPrimPath + "/Lights/DomeLight",
            defaultPrimPath + "/Lights/DiskLight",
            defaultPrimPath + "/GroundPlane/CollisionMesh",
            defaultPrimPath + "/GroundPlane/CollisionPlane"
        ])

        prim = stage.GetPrimAtPath(defaultPrimPath + "/Teddy")
        # xformable = UsdGeom.Xformable(prim)
        prim.GetAttribute("xformOp:scale").Set(Gf.Vec3f(2.5))

        prim = stage.GetPrimAtPath(defaultPrimPath + "/IceCubes")
        # xformable = UsdGeom.Xformable(prim)
        prim.GetAttribute("xformOp:scale").Set(Gf.Vec3f(0.5))

        ice_mat = stage.GetPrimAtPath(defaultPrimPath + "/IceCubes/PhysicsMaterial")
        UsdPhysics.MaterialAPI.Apply(ice_mat).CreateDensityAttr().Set(0.01)

        scene_prim = stage.GetPrimAtPath(defaultPrimPath + "/PhysicsScene")
        PhysxSchema.PhysxSceneAPI.Apply(scene_prim).CreateEnableExternalForcesEveryIterationAttr().Set(True)
