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

class ChairStackingDemo(demo.AsyncDemoBase):
    title = "Chair Stacking"
    category = demo.Categories.COMPLEX_SHOWCASES
    short_description = "Stacks of chairs with SDF collision"
    description = (
        "Demo showcasing the PhysX signed-distance-field (SDF) collision feature enabling "
        "dynamic objects with arbitrary collision shapes."
    )

    params = {"Num_Stacks": demo.IntParam(3, 1, 5, 2),
              "Num_Chairs": demo.IntParam(20, 5, 35, 1)}

    kit_settings = {
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
    }

    demo_camera = Sdf.Path("/World/Camera")

    def __init__(self):
        super().__init__(enable_fabric=True)
        self._reset_hydra_instancing_on_shutdown = False
        self.asset_paths = {
            "chair": demo.get_demo_asset_path(AssetFolders.CHAIR_STACKING, "SubUSDs/Chair/Chair_Wire_R512_Sparse.usd"),
        }
        self.demo_base_usd_url = demo.get_demo_asset_path(AssetFolders.CHAIR_STACKING, "StagingChairStacking.usd")

        # rng
        self._rng_seed = 42
        self._rng = np.random.default_rng(self._rng_seed)

    def on_startup(self):
        sceneGraphInstancingEnabled = carb.settings.get_settings().get("/persistent/omnihydra/useSceneGraphInstancing")
        if not sceneGraphInstancingEnabled:
            carb.settings.get_settings().set("/persistent/omnihydra/useSceneGraphInstancing", True)
            self._reset_hydra_instancing_on_shutdown = True

    def create(self, stage, Num_Stacks, Num_Chairs):
        defaultPrimPath = stage.GetDefaultPrim().GetPath()
        scenePrim = stage.GetPrimAtPath("/World/PhysicsScene")
        utils.set_physics_scene_asyncsimrender(scenePrim)
        timeStepsPerSecond = 120.0
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scenePrim)
        physxSceneAPI.CreateEnableExternalForcesEveryIterationAttr().Set(True)
        physxSceneAPI.GetTimeStepsPerSecondAttr().Set(timeStepsPerSecond)
        physxSceneAPI.CreateGpuFoundLostPairsCapacityAttr().Set(10 * 1024)
        physxSceneAPI.CreateGpuCollisionStackSizeAttr().Set(64 * 1024 * 1024)
        physxSceneAPI.CreateGpuMaxRigidContactCountAttr().Set(1024 * 512 * 2)
        physxSceneAPI.CreateGpuMaxRigidPatchCountAttr().Set(1024 * 80 * 2)
        physxSceneAPI.CreateSolverTypeAttr().Set("TGS")

        # add physics materials:
        density = 3e-3  # kg / cm3
        staticFriction = 0.6
        dynamicFriction = 0.6
        UsdGeom.Scope.Define(stage, "/World/PhysicsMaterials")
        chairPhysicsMaterialPath = "/World/PhysicsMaterials/ChairMaterial"
        utils.addRigidBodyMaterial(
            stage,
            chairPhysicsMaterialPath,
            density=density,
            staticFriction=staticFriction,
            dynamicFriction=dynamicFriction,
        )
        matPrim = stage.GetPrimAtPath(chairPhysicsMaterialPath)
        physxMaterialAPI = PhysxSchema.PhysxMaterialAPI.Apply(matPrim)

        # add material to ground plane:
        groundPlanePrim = stage.GetPrimAtPath("/World/GroundPlane")
        physicsUtils.add_physics_material_to_prim(stage, groundPlanePrim, chairPhysicsMaterialPath)

        # other parameters
        positionIterations = 50
        velocityIterations = 1
        maxAngularVelocity = 500
        setInstanceable = True
        sleepThreshold = 5e-3
        stabilizationThreshold = 0.005
        maxDepenetrationVel = 500.0 # cm/s

        # scene geometry:
        stack_delta_y = 150.0
        stack_start_y = stack_delta_y * (Num_Stacks // 2)
        stack_start_pos = Gf.Vec3f(0.0, -stack_start_y, 0.0)
        stack_delta = Gf.Vec3f(0.0, stack_delta_y, 0.0)
        chair_delta_pos = Gf.Vec3f(4, 0, 24)
        position_noise_half_range = 0.001

        # position cam
        cam = UsdGeom.Camera.Define(stage, self.demo_camera)
        cam.CreateFocalLengthAttr().Set(16.0)
        camPos = Gf.Vec3d(749.0, 398.0, 15.0)
        camRot = Gf.Vec3d(107.0, 0.0, 119.0)
        cam.AddTranslateOp().Set(camPos)
        cam.AddRotateXYZOp().Set(camRot)

        for stack_idx in range(Num_Stacks):
            stack_path = defaultPrimPath.AppendChild(f"stack_{stack_idx}")
            stack_xform = UsdGeom.Xform.Define(stage, stack_path)
            physicsUtils.set_or_add_translate_op(stack_xform, translate=stack_start_pos + stack_delta * stack_idx)

            for chair_idx in range(Num_Chairs):
                pos_noise = self._rng.uniform(-position_noise_half_range, position_noise_half_range, 3)
                pos_noise = Gf.Vec3f(pos_noise.tolist())
                chair_position = Gf.Vec3f(0.0) + chair_idx * chair_delta_pos + pos_noise

                # create xform
                chair_path = stack_path.AppendChild(f"chair_{chair_idx}")
                assert stage.DefinePrim(chair_path).GetReferences().AddReference(self.asset_paths["chair"])
                xform = UsdGeom.Xform.Get(stage, chair_path)
                physicsUtils.set_or_add_translate_op(xform, translate=chair_position)
                chair_prim = stage.GetPrimAtPath(chair_path)
                chair_prim.SetInstanceable(setInstanceable)

                physicsUtils.add_physics_material_to_prim(stage, chair_prim, chairPhysicsMaterialPath)
                rbAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(chair_prim)
                rbAPI.CreateSolverPositionIterationCountAttr(positionIterations)
                rbAPI.CreateSolverVelocityIterationCountAttr(velocityIterations)
                rbAPI.CreateMaxAngularVelocityAttr().Set(maxAngularVelocity)
                rbAPI.CreateSleepThresholdAttr(sleepThreshold)
                rbAPI.CreateStabilizationThresholdAttr(stabilizationThreshold)
                rbAPI.CreateEnableGyroscopicForcesAttr().Set(True)
                rbAPI.CreateMaxDepenetrationVelocityAttr().Set(maxDepenetrationVel)

    def on_shutdown(self):
        super().on_shutdown()
        if self._reset_hydra_instancing_on_shutdown:
            carb.settings.get_settings().set("/persistent/omnihydra/useSceneGraphInstancing", False)
