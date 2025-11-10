# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.physx.scripts.utils
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, TestCategory
from omni.physx import get_physx_cooking_interface
from pxr import UsdGeom, UsdPhysics, UsdUtils, PhysicsSchemaTools
from omni.physx.bindings._physx import PhysxCollisionRepresentationResult, PhysxConvexMeshData

# The functionality of request_collision_representation is tested in the C++ unit test TestCollisionRepresentation.cpp.
# This python test is only checking that the python binding is properly functioning.
# It also provies as a small minimal example on how to use the API


class PhysxCookingInterfaceTestKitStage(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Core

    def setup_stage(self, stage):
        cache = UsdUtils.StageCache.Get()
        self.stage_id = cache.GetId(stage).ToLongInt()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        self.defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        UsdPhysics.Scene.Define(stage, self.defaultPrimPath + "/physicsScene")
        get_physx_cooking_interface().release_local_mesh_cache()

    def _create_concave_mesh(self, stage, path: str, size: float, density: float) -> int:
        concaveGeom = physicsUtils.create_mesh_concave(stage, path, size)
        # concaveGeom.AddTranslateOp().Set(Gf.Vec3f(200.0))
        UsdPhysics.RigidBodyAPI.Apply(concaveGeom.GetPrim())
        UsdPhysics.CollisionAPI.Apply(concaveGeom.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(concaveGeom.GetPrim())
        mass_api.CreateDensityAttr(density)
        meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(concaveGeom.GetPrim())
        meshCollisionAPI.CreateApproximationAttr().Set("convexDecomposition")
        return PhysicsSchemaTools.sdfPathToInt(path)

    async def test_collision_representation(self):
        """ Tests the ability to create a convex collision representation task """

        stage = await self.new_stage()
        self.setup_stage(stage)

        path = self.defaultPrimPath + "/concave_mesh"
        self._create_concave_mesh(stage, path, size=10, density=1)
        prim_id = PhysicsSchemaTools.sdfPathToInt(path)

        returned_convexes: list[PhysxConvexMeshData] = None
        returned_result: PhysxCollisionRepresentationResult = None

        def on_convex_representation_ready(result: PhysxCollisionRepresentationResult,
                                           convex_result: list[PhysxConvexMeshData]):
            nonlocal returned_convexes
            nonlocal returned_result
            returned_convexes = convex_result
            returned_result = result

        physx_cooking = get_physx_cooking_interface()
        # Do an asynchronous request for a convex representation
        physx_cooking.request_convex_collision_representation(stage_id=self.stage_id,
                                                              collision_prim_id=prim_id,
                                                              run_asynchronously=True,
                                                              on_result=on_convex_representation_ready)
        idx = 0
        while idx < 100:
            await omni.kit.app.get_app().next_update_async()
            idx = idx + 1
        self.assertEqual(returned_result, PhysxCollisionRepresentationResult.RESULT_VALID)
        self.assertEqual(len(returned_convexes), 4)
        convex: PhysxConvexMeshData = returned_convexes[0]
        # Test that field names of the dictionary are stable
        self.assertGreater(len(convex.vertices), 0)
        self.assertGreater(len(convex.indices), 0)
        self.assertGreater(len(convex.polygons), 0)
        polygon = convex.polygons[0]
        self.assertEqual(len(polygon.plane), 4)
        self.assertGreater(polygon.num_vertices, 0)
        self.assertGreater(polygon.index_base, -1)

    async def test_collision_representation_cancel(self):
        """ Tests the ability to cancel a collision representation task """
        stage = await self.new_stage()
        self.setup_stage(stage)

        path = self.defaultPrimPath + "/concave_mesh"
        self._create_concave_mesh(stage, path, size=10, density=1)
        prim_id = PhysicsSchemaTools.sdfPathToInt(path)

        callback_called = False

        def on_convex_representation_ready(result: PhysxCollisionRepresentationResult, convexes: list[PhysxConvexMeshData]):
            nonlocal callback_called
            callback_called = True

        physx_cooking = get_physx_cooking_interface()
        # Do an asynchronous request for a convex representation
        task_handle = physx_cooking.request_convex_collision_representation(stage_id=self.stage_id,
                                                                            collision_prim_id=prim_id,
                                                                            run_asynchronously=True,
                                                                            on_result=on_convex_representation_ready)
        # Start the async task
        await omni.kit.app.get_app().next_update_async()

        # Immediately cancel it without calling the callback
        physx_cooking.cancel_collision_representation_task(task=task_handle, invoke_callback=False)

        # Give it some time for cooking to finish
        idx = 0
        while idx < 100:
            await omni.kit.app.get_app().next_update_async()
            idx = idx + 1

        # Make sure it was actually cancelled
        self.assertFalse(callback_called)
