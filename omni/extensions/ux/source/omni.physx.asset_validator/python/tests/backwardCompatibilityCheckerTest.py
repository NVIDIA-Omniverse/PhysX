# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from ..scripts.backwardCompatibilityChecker import BackwardCompatibilityChecker
from omni.physx.scripts import physicsUtils
from omni.physxtests import utils as testUtils
from pxr import UsdPhysics, Sdf, PhysxSchema
from .utils import BaseValidatorTest, ExpectedResult


class BackwardCompatibilityCheckerTestCase(BaseValidatorTest):
    rules = [BackwardCompatibilityChecker]
    test_folder = "BackwardCompatibilityChecker"
    expected_result = ExpectedResult(suggestion_message="Do backward compatibility fixes")

    async def test_backward_compat_vehicles(self):
        await self.open_usd_and_validate_and_check_and_fix(
            f"{self.test_folder}/vehicles_compat",
            self.expected_result
        )

    async def test_backward_compat_base(self):
        await self.open_usd_and_validate_and_check_and_fix(
            f"{self.test_folder}/base_compat",
            self.expected_result
        )

    async def test_backward_compat_cct(self):
        await self.open_usd_and_validate_and_check_and_fix(
            f"{self.test_folder}/cct_compat",
            self.expected_result
        )

    async def test_backward_compat_physx_mesh_collision_api(self):
        await self.open_usd_and_validate_and_check_and_fix(
            f"{self.test_folder}/physxMeshCollisionAPI_compat",
            self.expected_result
        )

    async def test_backward_compat_trianglemesh_to_sdf(self):
        stage = await self.open_usd("BackwardCompatibilityChecker/trianglemesh_to_sdf_compat")
        self.set_stage(stage)

        meshPrim = stage.GetPrimAtPath(str(stage.GetDefaultPrim().GetPath()) + "/meshCube").GetPrim()

        params = ["physxTriangleMeshCollision:sdfResolution",
                  "physxTriangleMeshCollision:sdfBitsPerSubgridPixel",
                  "physxTriangleMeshCollision:sdfMargin",
                  "physxTriangleMeshCollision:sdfNarrowBandThickness",
                  "physxTriangleMeshCollision:sdfSubgridResolution"]

        oldvals = [meshPrim.GetAttribute(p).Get() for p in params]

        self.validate_and_check_and_fix(self.expected_result)

        physxSDFMeshCollisionAPI = PhysxSchema.PhysxSDFMeshCollisionAPI(meshPrim)
        self.assertIsNotNone(physxSDFMeshCollisionAPI)
        newvals = [physxSDFMeshCollisionAPI.GetSdfResolutionAttr().Get(),
                   physxSDFMeshCollisionAPI.GetSdfBitsPerSubgridPixelAttr().Get(),
                   physxSDFMeshCollisionAPI.GetSdfMarginAttr().Get(),
                   physxSDFMeshCollisionAPI.GetSdfNarrowBandThicknessAttr().Get(),
                   physxSDFMeshCollisionAPI.GetSdfSubgridResolutionAttr().Get()]

        for o, n in zip(oldvals, newvals):
            self.assertEqual(o, n)

        for o in params:
            self.assertFalse(meshPrim.GetAttribute(o))

    # Test that the physxScene:asyncSimRender attribute (now in schema, no longer an attribute)
    # is properly handled and converted to the new schema value
    async def test_backward_compat_async_scene_simulation(self):
        stage = await self.new_stage()
        self.set_stage(stage)
        self.default_prim_path = str(stage.GetDefaultPrim().GetPath())

        scene = UsdPhysics.Scene.Define(stage, self.default_prim_path + "/physicsScene")
        scenePrim = scene.GetPrim()
        # apply physx scene schema
        PhysxSchema.PhysxSceneAPI.Apply(scenePrim)
        # Add the old attribute manually
        scenePrim.CreateAttribute("asyncSimRender", Sdf.ValueTypeNames.Bool, True).Set(True)

        self.validate_and_check_and_fix(self.expected_result)

    # Test that the physxsdfcollision:resolution attribute is properly handled
    async def test_backward_compat_sdf_resolution(self):
        stage = await self.new_stage()
        self.set_stage(stage)

        meshCube = physicsUtils.create_mesh_cube(stage, "/meshCube", 20.0)
        UsdPhysics.RigidBodyAPI.Apply(meshCube.GetPrim())
        UsdPhysics.CollisionAPI.Apply(meshCube.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(meshCube.GetPrim())

        # Add the api with old custom attribute manually
        PhysxSchema.PhysxTriangleMeshCollisionAPI.Apply(meshCube.GetPrim())
        oldAttrvalue = 8
        meshCube.GetPrim().CreateAttribute("physxsdfcollision:resolution", Sdf.ValueTypeNames.Int, True).Set(oldAttrvalue)

        self.validate_and_check_and_fix(self.expected_result)

        # and that the old attr is gone and the new one has the correct value and the new api is applied
        physxSDFMeshCollisionAPI = PhysxSchema.PhysxSDFMeshCollisionAPI(meshCube.GetPrim())
        self.assertIsNotNone(physxSDFMeshCollisionAPI)
        newAttrValue = physxSDFMeshCollisionAPI.GetSdfResolutionAttr().Get()
        self.assertEqual(oldAttrvalue, newAttrValue)
        self.assertFalse(meshCube.GetPrim().GetAttribute("physxsdfcollision:resolution"))

    # Test that the PhysxArticulationForceSensorAPI triggers a deprecation warning
    # when running a backwards compatibility check
    async def test_backward_compat_articulation_force_sensor(self):
        stage = await self.open_usd("BackwardCompatibilityChecker/articulationForceSensor_compat")
        self.set_stage(stage)
        self.create_validation_engine()

        # check that we got a compatibility warning.
        message = "Physics backwardsCompatibility: PhysxArticulationForceSensorAPI has been removed. Prim (/World/Cube)."
        with testUtils.ExpectMessage(self, message):
            results = self.run_validation_and_assert_result(self.expected_result)

        # remove the PhysxArticulationForceSensorAPI from the Cube prim because it is no longer a valid schema.
        self.run_validation_fix(results)

        # check that we get no compatibility warning.
        self.run_validation_and_assert_result(None)

    # Test that the physxScene iteration counts rename works correctly
    async def test_backward_scene_iteration_count(self):
        stage = await self.new_stage()
        self.set_stage(stage)
        self.default_prim_path = str(stage.GetDefaultPrim().GetPath())

        # create the scene
        scene = UsdPhysics.Scene.Define(stage, self.default_prim_path + "/physicsScene")
        scenePrim = scene.GetPrim()
        sceneApi = PhysxSchema.PhysxSceneAPI.Apply(scenePrim)

        # Add the old attribute manually
        oldMaxValue = 130
        oldMinValue = 125
        scenePrim.CreateAttribute("physxScene:maxIterationCount", Sdf.ValueTypeNames.UInt, True).Set(oldMaxValue)
        scenePrim.CreateAttribute("physxScene:minIterationCount", Sdf.ValueTypeNames.UInt, True).Set(oldMinValue)

        self.validate_and_check_and_fix(self.expected_result)

        # and that the old attr is gone and the new one has the correct value
        self.assertFalse(scenePrim.GetAttribute("physxScene:maxIterationCount"))
        self.assertFalse(scenePrim.GetAttribute("physxScene:minIterationCount"))
        newMinValue = sceneApi.GetMinPositionIterationCountAttr().Get()
        newMaxValue = sceneApi.GetMaxPositionIterationCountAttr().Get()
        self.assertEqual(oldMinValue, newMinValue)
        self.assertEqual(oldMaxValue, newMaxValue)
