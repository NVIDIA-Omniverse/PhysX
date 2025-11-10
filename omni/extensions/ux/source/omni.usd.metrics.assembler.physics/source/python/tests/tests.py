# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import os

import omni.kit.test
from omni.metrics.assembler.core import get_metrics_assembler_interface
from pxr import PhysxSchema, Plug, Tf, Usd, UsdGeom, UsdPhysics, UsdUtils

TOLERANCE = 1e-3


class MetricsAssemblerPhysicsTests(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        pass

    async def tearDown(self):
        pass

    def setup_stage(self, stage):
        cache = UsdUtils.StageCache.Get()
        cache.Insert(stage)
        stage_id = cache.GetId(stage).ToLongInt()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)
        UsdPhysics.SetStageKilogramsPerUnit(stage, 0.1)
        data_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, "../../../../../../data/tests/")))
        data_folder = data_folder.replace("\\", "/") + "/"

        xform = UsdGeom.Xform.Define(stage, "/World/xformInst")
        xform.GetPrim().GetReferences().AddReference(data_folder + "cubePhysicsMeters.usda")
        return stage_id

    async def test_divergency_base_check(self):
        stage = Usd.Stage.CreateInMemory()
        stage_id = self.setup_stage(stage)

        divergent_stage = get_metrics_assembler_interface().resolve_stage(stage_id)
        self.assertTrue(divergent_stage)

    async def test_scene_resolve_units(self):
        stage = Usd.Stage.CreateInMemory()
        stage_id = self.setup_stage(stage)

        physics_scene = UsdPhysics.Scene.Get(stage, "/World/xformInst/PhysicsScene")
        self.assertTrue(physics_scene)

        gravity_magnitude = physics_scene.GetGravityMagnitudeAttr().Get()
        self.assertTrue(abs(gravity_magnitude - 9.8) < TOLERANCE)

        divergent_stage = get_metrics_assembler_interface().resolve_stage(stage_id)
        self.assertTrue(divergent_stage)

        gravity_magnitude = physics_scene.GetGravityMagnitudeAttr().Get()
        self.assertTrue(abs(gravity_magnitude - 980.0) < TOLERANCE)

    async def test_rigid_body_resolve_units(self):
        stage = Usd.Stage.CreateInMemory()
        stage_id = self.setup_stage(stage)

        rigid_body_api = UsdPhysics.RigidBodyAPI.Get(stage, "/World/xformInst/Cube")
        self.assertTrue(rigid_body_api)
        physx_rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Get(stage, "/World/xformInst/Cube")
        self.assertTrue(physx_rigid_body_api)

        mass_api = UsdPhysics.MassAPI.Get(stage, "/World/xformInst/Cube")
        self.assertTrue(mass_api)

        density = mass_api.GetDensityAttr().Get()
        self.assertTrue(abs(density - 10.0) < TOLERANCE)

        velocity = rigid_body_api.GetVelocityAttr().Get()
        for i in range(3):
            self.assertTrue(abs(velocity[i] - 2.0) < TOLERANCE)

        sleep_threshold = physx_rigid_body_api.GetSleepThresholdAttr().Get()
        self.assertTrue(abs(sleep_threshold - 1.0) < TOLERANCE)

        divergent_stage = get_metrics_assembler_interface().resolve_stage(stage_id)
        self.assertTrue(divergent_stage)

        density = mass_api.GetDensityAttr().Get()
        self.assertTrue(abs(density - 10.0 / ( 100.0 * 100.0 * 100.0 ) ) < TOLERANCE)

        velocity = rigid_body_api.GetVelocityAttr().Get()
        for i in range(3):
            self.assertTrue(abs(velocity[i] - 200.0) < TOLERANCE)

        sleep_threshold = physx_rigid_body_api.GetSleepThresholdAttr().Get()
        self.assertTrue(abs(sleep_threshold - 100.0 * 100.0) < TOLERANCE)

    async def test_collision_resolve_units(self):
        stage = Usd.Stage.CreateInMemory()
        stage_id = self.setup_stage(stage)

        physx_collision_api = PhysxSchema.PhysxCollisionAPI.Get(stage, "/World/xformInst/Cube")
        self.assertTrue(physx_collision_api)

        torsial_patch_radius = physx_collision_api.GetTorsionalPatchRadiusAttr().Get()
        self.assertTrue(abs(torsial_patch_radius - 0.01) < TOLERANCE)

        divergent_stage = get_metrics_assembler_interface().resolve_stage(stage_id)
        self.assertTrue(divergent_stage)

        torsial_patch_radius = physx_collision_api.GetTorsionalPatchRadiusAttr().Get()
        self.assertTrue(abs(torsial_patch_radius - 1.0) < TOLERANCE)

    async def test_material_resolve_units(self):
        stage = Usd.Stage.CreateInMemory()
        stage_id = self.setup_stage(stage)

        material_api = UsdPhysics.MaterialAPI.Get(stage, "/World/xformInst/PhysicsMaterial")
        self.assertTrue(material_api)

        density = material_api.GetDensityAttr().Get()
        self.assertTrue(abs(density - 500.0) < TOLERANCE)

        divergent_stage = get_metrics_assembler_interface().resolve_stage(stage_id)
        self.assertTrue(divergent_stage)

        density = material_api.GetDensityAttr().Get()
        self.assertTrue(abs(density - 500.0 * 10.0 / (100.0 * 100.0 * 100.0)) < TOLERANCE)

    async def test_joint_resolve_units(self):
        stage = Usd.Stage.CreateInMemory()
        stage_id = self.setup_stage(stage)

        prismatic_joint = UsdPhysics.PrismaticJoint.Get(stage, "/World/xformInst/PrismaticJoint")
        self.assertTrue(prismatic_joint)

        limit_api = PhysxSchema.PhysxLimitAPI(prismatic_joint.GetPrim(), "linear")

        stiffness = limit_api.GetStiffnessAttr().Get()
        self.assertTrue(abs(stiffness - 1.0) < TOLERANCE)
        lower_limit = prismatic_joint.GetLowerLimitAttr().Get()
        self.assertTrue(abs(lower_limit + 1.0) < TOLERANCE)
        upper_limit = prismatic_joint.GetUpperLimitAttr().Get()
        self.assertTrue(abs(upper_limit - 1.0) < TOLERANCE)

        divergent_stage = get_metrics_assembler_interface().resolve_stage(stage_id)
        self.assertTrue(divergent_stage)

        stiffness = limit_api.GetStiffnessAttr().Get()
        self.assertTrue(abs(stiffness - 10.0) < TOLERANCE)
        lower_limit = prismatic_joint.GetLowerLimitAttr().Get()
        self.assertTrue(abs(lower_limit + 100.0) < TOLERANCE)
        upper_limit = prismatic_joint.GetUpperLimitAttr().Get()
        self.assertTrue(abs(upper_limit - 100.0) < TOLERANCE)
