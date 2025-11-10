# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
import random
import carb
import omni
from pxr import Gf, Sdf, UsdGeom, PhysxSchema
import omni.physxdemos as demo
from omni.physx.scripts import utils
from omni.physx.scripts import physicsUtils
from omni.physx.scripts.assets_paths import AssetFolders

class NutsAndBoltsDemo(demo.AsyncDemoBase):
    title = "Nuts and Bolts"
    category = demo.Categories.COMPLEX_SHOWCASES
    short_description = "Nuts threading onto bolts showcasing PhysX SDF collision"
    description = (
        "The nuts and bolts leverage the new signed-distance-field (SDF) collision feature of PhysX, "
        "which enables a contact-rich interaction. The friction parameters are tuned low enough such that the nuts are "
        "threading onto the bolts solely under the influence of gravity. You may further tune the behavior by adjusting "
        "the friction properties of the materials in /World/PhysicsMaterials."
    )

    params = {"Scene_Size": demo.IntParam(200, 1, 1024, 1)}

    kit_settings = {
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
    }

    demo_camera = Sdf.Path("/World/Camera")

    def __init__(self):
        super().__init__(enable_fabric=True)
        self._reset_hydra_instancing_on_shutdown = False
        self.asset_paths = {
            "nut": demo.get_demo_asset_path(AssetFolders.NUTS_AND_BOLTS, "M20_Nut_Tight_R256_SI_Sparse.usd"),
            "bolt": demo.get_demo_asset_path(AssetFolders.NUTS_AND_BOLTS, "M20_Bolt_Tight_R512_SI_Sparse.usd"),
        }
        self.demo_base_usd_url = demo.get_demo_asset_path(AssetFolders.NUTS_AND_BOLTS, "base_stage_SI.usd")

    def on_startup(self):
        sceneGraphInstancingEnabled = carb.settings.get_settings().get("/persistent/omnihydra/useSceneGraphInstancing")
        if not sceneGraphInstancingEnabled:
            carb.settings.get_settings().set("/persistent/omnihydra/useSceneGraphInstancing", True)
            self._reset_hydra_instancing_on_shutdown = True

    def create(self, stage, Scene_Size):
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)  # set to SI
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        scenePrim = stage.GetPrimAtPath("/World/PhysicsScene")
        utils.set_physics_scene_asyncsimrender(scenePrim)

        # setup ground collision plane:
        utils.addPlaneCollider(stage, "/World/physicsGroundPlaneCollider", "Z")

        # add physics materials:
        density = 7.85e3  # kg / m3
        staticFriction = 0.021
        dynamicFriction = 0.021
        UsdGeom.Scope.Define(stage, "/World/PhysicsMaterials")
        boltPhysicsMaterialPath = "/World/PhysicsMaterials/BoltMaterial"
        nutPhysicsMaterialPath = "/World/PhysicsMaterials/NutMaterial"
        utils.addRigidBodyMaterial(
            stage,
            nutPhysicsMaterialPath,
            density=density,
            staticFriction=staticFriction,
            dynamicFriction=dynamicFriction,
        )
        utils.addRigidBodyMaterial(
            stage,
            boltPhysicsMaterialPath,
            density=density,
            staticFriction=staticFriction,
            dynamicFriction=dynamicFriction,
        )

        # other parameters
        positionIterations = 16
        velocityIterations = 1
        maxAngularVelocity = 1.0e6
        setInstanceable = True

        # create grid of nuts and bolts:
        nut_height = 0.092
        spacing = 0.10
        position_noise = 0.015
        alt_row_offset = spacing * 0.5
        num_nuts_per_side = max(1, round(math.sqrt(Scene_Size)))
        row_half_length = (num_nuts_per_side - 1) * spacing * 0.5

        # position cam
        cam_y_offset = -1.010
        camPos = Gf.Vec3d(0, cam_y_offset - row_half_length, 0.35)
        physicsUtils.set_or_add_translate_op(UsdGeom.Camera.Get(stage, self.demo_camera), translate=camPos)

        defaultPrimPath = stage.GetDefaultPrim().GetPath()
        for i in range(Scene_Size):
            col_number = i % num_nuts_per_side
            row_number = i // num_nuts_per_side
            row_offset = (row_number % 2) * alt_row_offset
            y_pos = -row_half_length + row_number * spacing
            y_pos += random.uniform(0, 2.0 * position_noise) - position_noise
            x_pos = -row_half_length + col_number * spacing - row_offset
            x_pos += random.uniform(0, 2.0 * position_noise) - position_noise
            # create xform
            xform = UsdGeom.Xform.Define(stage, defaultPrimPath.AppendChild(f"nut_bolt_{i}"))
            xform.AddTranslateOp().Set(Gf.Vec3f(x_pos, y_pos, 0.0))
            nutPath = xform.GetPath().AppendChild("nut")
            assert stage.DefinePrim(nutPath).GetReferences().AddReference(self.asset_paths["nut"])
            nut_prim = stage.GetPrimAtPath(nutPath)
            nut_prim.SetInstanceable(setInstanceable)
            physicsUtils.add_physics_material_to_prim(stage, nut_prim, nutPhysicsMaterialPath)
            rbAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(nut_prim)
            rbAPI.CreateSolverPositionIterationCountAttr(positionIterations)
            rbAPI.CreateSolverVelocityIterationCountAttr(velocityIterations)
            rbAPI.CreateMaxAngularVelocityAttr().Set(maxAngularVelocity)
            rbAPI.CreateSleepThresholdAttr().Set(0.0)  # disable sleep for nut so friction changes can stop/restart motion
            physicsUtils.set_or_add_translate_op(
                UsdGeom.Xformable.Get(stage, nutPath), translate=Gf.Vec3f(0.0, 0.0, nut_height)
            )

            boltPath = xform.GetPath().AppendChild("bolt")
            assert stage.DefinePrim(boltPath).GetReferences().AddReference(self.asset_paths["bolt"])
            bolt_prim = stage.GetPrimAtPath(boltPath)
            bolt_prim.SetInstanceable(setInstanceable)
            physicsUtils.add_physics_material_to_prim(stage, bolt_prim, boltPhysicsMaterialPath)
            rbAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(bolt_prim)
            rbAPI.CreateSolverPositionIterationCountAttr(positionIterations)
            rbAPI.CreateSolverVelocityIterationCountAttr(velocityIterations)
        omni.usd.get_context().get_selection().set_selected_prim_paths([], False)

    def on_shutdown(self):
        super().on_shutdown()
        if self._reset_hydra_instancing_on_shutdown:
            carb.settings.get_settings().set("/persistent/omnihydra/useSceneGraphInstancing", False)
