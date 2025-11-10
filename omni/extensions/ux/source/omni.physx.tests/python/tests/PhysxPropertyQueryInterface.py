# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.physx.scripts.utils
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, TestCategory
from omni.physx import get_physx_property_query_interface, get_physx_cooking_interface, get_physx_cooking_private_interface
from omni.physx.bindings._physx import PhysxPropertyQueryArticulationResponse, PhysxPropertyQueryRigidBodyResponse, PhysxPropertyQueryColliderResponse, PhysxPropertyQueryResult, PhysxPropertyQueryMode
from pxr import Gf, UsdGeom, UsdPhysics, UsdUtils, PhysicsSchemaTools, PhysxSchema
import time
import unittest
import carb
import omni.physx.bindings._physx as physx_bindings

class PhysxPropertyQueryInterfaceTestKitStage(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Core

    def setup_stage(self, stage):
        cache = UsdUtils.StageCache.Get()
        self.stage_id = cache.GetId(stage).ToLongInt()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        self.defaultPrimPath = str(stage.GetDefaultPrim().GetPath())
        
        UsdPhysics.Scene.Define(stage, self.defaultPrimPath + "/physicsScene")
        get_physx_cooking_interface().release_local_mesh_cache()

    def _create_concave_mesh(self, stage, path:str, size:float, density:float) -> int :
        concaveGeom = physicsUtils.create_mesh_concave(stage, path, size)
        # concaveGeom.AddTranslateOp().Set(Gf.Vec3f(200.0))
        UsdPhysics.RigidBodyAPI.Apply(concaveGeom.GetPrim())
        UsdPhysics.CollisionAPI.Apply(concaveGeom.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(concaveGeom.GetPrim())
        mass_api.CreateDensityAttr(density)
        meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(concaveGeom.GetPrim())
        meshCollisionAPI.CreateApproximationAttr().Set("convexDecomposition")
        return PhysicsSchemaTools.sdfPathToInt(path)

    def _create_cube_mesh(self, stage, path:str, size:float, density:float) -> int :
        cubeGeom = physicsUtils.create_mesh_cube(stage, path, size / 2)
        # cubeGeom.AddTranslateOp().Set(Gf.Vec3f(200.0))
        UsdPhysics.RigidBodyAPI.Apply(cubeGeom.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cubeGeom.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(cubeGeom.GetPrim())
        mass_api.CreateDensityAttr(density)
        meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(cubeGeom.GetPrim())
        meshCollisionAPI.CreateApproximationAttr().Set("convexDecomposition")
        return PhysicsSchemaTools.sdfPathToInt(path)

    def _create_capsule(self, stage, path:str, radius:float, height:float, density:float) -> int :
        capsuleGeom = physicsUtils.add_capsule(stage, path, radius, height)
        UsdPhysics.RigidBodyAPI.Apply(capsuleGeom.GetPrim())
        UsdPhysics.CollisionAPI.Apply(capsuleGeom.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(capsuleGeom.GetPrim())
        mass_api.CreateDensityAttr(density)
        meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(capsuleGeom.GetPrim())
        meshCollisionAPI.CreateApproximationAttr().Set("convexDecomposition")
        return PhysicsSchemaTools.sdfPathToInt(path)

    def _create_xform_child_colliders(self, stage, path:str, sizes: list[float], density) -> int :
        path = physicsUtils.get_stage_next_free_path(stage, path, True)
        xform_rb = UsdGeom.Xform.Define(stage, path)
        xform_collider = UsdGeom.Xform.Define(stage, path + "/collider")
        cubes = []
        pos_offset = Gf.Vec3f(0.0)
        for n in range(len(sizes)):
            pos_offset = pos_offset + Gf.Vec3f(sizes[n], 0.0, sizes[n])
            cubes.append(physicsUtils.add_cube(stage, path + "/collider/cube" + str(n), sizes[n], position=pos_offset))
            pos_offset = pos_offset + Gf.Vec3f(sizes[n], 0.0, sizes[n])

        UsdPhysics.RigidBodyAPI.Apply(xform_rb.GetPrim())
        UsdPhysics.CollisionAPI.Apply(xform_collider.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(xform_collider.GetPrim())
        mass_api.CreateDensityAttr(density)
        meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(xform_collider.GetPrim())
        meshCollisionAPI.CreateApproximationAttr().Set(UsdPhysics.Tokens.convexDecomposition)
        return PhysicsSchemaTools.sdfPathToInt(path)

    def _create_mesh_merge_collision(self, stage, path:str, sizes: list[float], density) -> int :
        path = physicsUtils.get_stage_next_free_path(stage, path, True)
        xform_rb = UsdGeom.Xform.Define(stage, path)
        prim_rb = xform_rb.GetPrim()

        UsdPhysics.RigidBodyAPI.Apply(prim_rb)
        UsdPhysics.CollisionAPI.Apply(prim_rb)

        mass_api = UsdPhysics.MassAPI.Apply(prim_rb)
        mass_api.CreateDensityAttr(density)

        meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(prim_rb)
        meshCollisionAPI.CreateApproximationAttr().Set(UsdPhysics.Tokens.convexDecomposition)

        mesh_merge_api = PhysxSchema.PhysxMeshMergeCollisionAPI(prim_rb)
        collection_api = mesh_merge_api.GetCollisionMeshesCollectionAPI()

        cubes = []
        pos_offset = Gf.Vec3f(0.0)
        for n in range(len(sizes)):
            pos_offset = pos_offset + Gf.Vec3f(sizes[n], 0.0, sizes[n])
            cubes.append(physicsUtils.add_cube(stage, path + "/cube" + str(n), sizes[n], position=pos_offset))
            pos_offset = pos_offset + Gf.Vec3f(sizes[n], 0.0, sizes[n])
            collection_api.GetIncludesRel().AddTarget(path + "/cube" + str(n))

        return PhysicsSchemaTools.sdfPathToInt(path)

    async def _wait_cooking_finished(self):
        while True:
            await omni.kit.app.get_app().next_update_async()
            cooking_statistics = get_physx_cooking_private_interface().get_cooking_statistics()
            running_tasks = cooking_statistics.total_scheduled_tasks - cooking_statistics.total_finished_tasks
            self.assertGreaterEqual(running_tasks, 0)
            if running_tasks <= 0:
                break
        
    async def test_property_query_rb_shape(self):
        stage = await self.new_stage()

        self.setup_stage(stage)
        self.callback_called = False
        def query_report(rigid_info : PhysxPropertyQueryRigidBodyResponse):
            self.assertEqual(rigid_info.result, PhysxPropertyQueryResult.VALID)
            self.assertEqual(rigid_info.mass, 125.0)
            self.callback_called = True
        path = self.defaultPrimPath + "/boxActor"
        physicsUtils.add_rigid_box(stage, path=path, size=Gf.Vec3f(5), density=1)
        prim_id = PhysicsSchemaTools.sdfPathToInt(path)
        get_physx_property_query_interface().query_prim(stage_id = self.stage_id, prim_id = prim_id, rigid_body_fn = query_report)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(self.callback_called)

    async def test_property_query_rb_mesh_invalid_usd_stage(self):
        stage = await self.new_stage()
    
        self.setup_stage(stage)
        self.callback_called = False
        def query_report(rigid_info : PhysxPropertyQueryRigidBodyResponse):
            self.assertEqual(rigid_info.result, PhysxPropertyQueryResult.ERROR_INVALID_USD_STAGE)
            self.callback_called = True

        # We are creating a concave mesh that requires background cooking
        # This test will fail with ERROR_INVALID_USD_STAGE because we are replacing the stage before we
        # have a chance to finish the cooking and deliver the result
        # We need also to make sure that cache is disabled to avoid query_prim returning before next update
        # because it got the cooked data from cache
        settings = carb.settings.get_settings()
        use_local_cache = settings.get_as_bool(physx_bindings.SETTING_USE_LOCAL_MESH_CACHE)
        settings.set_bool(physx_bindings.SETTING_USE_LOCAL_MESH_CACHE, False)
        
        path = self.defaultPrimPath + "/concave"
        # we are using a unique size to enforce not using any cooking task that may be in flight from previous tests
        prim_id = self._create_concave_mesh(stage, path, size=123.456789, density=1)

        get_physx_property_query_interface().query_prim(stage_id = self.stage_id, prim_id = prim_id, rigid_body_fn = query_report)
        stage = await self.new_stage()
        await omni.kit.app.get_app().next_update_async()
        settings.set_bool(physx_bindings.SETTING_USE_LOCAL_MESH_CACHE, use_local_cache)
        self.assertTrue(self.callback_called)

    async def test_property_query_rb_mesh_invalid_usd_prim(self):
        stage = await self.new_stage()
    
        self.setup_stage(stage)
        self.callback_called = False
        def query_report(rigid_info : PhysxPropertyQueryRigidBodyResponse):
            self.assertEqual(rigid_info.result, PhysxPropertyQueryResult.ERROR_INVALID_USD_PRIM)
            self.callback_called = True

        # We are creating a concave mesh that requires background cooking
        # This test will fail with ERROR_INVALID_USD_PRIM because we are deleting the Usd Prim before we
        # have a chance to finish the cooking and deliver the result
        path = self.defaultPrimPath + "/concave"
        prim_id = self._create_concave_mesh(stage, path, size=10, density=1)
        get_physx_property_query_interface().query_prim(stage_id = self.stage_id, prim_id = prim_id, rigid_body_fn = query_report)
        self._stage.RemovePrim(path)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(self.callback_called)

    @unittest.skip("107 crash/timeout")
    async def test_property_query_rb_mesh_invalid_usd_path(self):
        stage = await self.new_stage()
    
        self.setup_stage(stage)
        self.callback_called = False
        def query_report(rigid_info : PhysxPropertyQueryRigidBodyResponse):
            self.assertEqual(rigid_info.result, PhysxPropertyQueryResult.ERROR_INVALID_USD_PATH)
            self.callback_called = True

        # We are creating a concave mesh that requires background cooking
        # This test will fail with ERROR_INVALID_USD_PATH because we are giving a wrongpath
        path = self.defaultPrimPath + "/concave"
        prim_id = self._create_concave_mesh(stage, path, size=10, density=1)
        prim_id = PhysicsSchemaTools.sdfPathToInt(path + "WrongPath")
        get_physx_property_query_interface().query_prim(stage_id = self.stage_id, prim_id = prim_id, rigid_body_fn = query_report)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(self.callback_called)

    async def test_property_query_rb_mesh_valid(self):
        stage = await self.new_stage(def_up_and_mpu=True)
    
        self.setup_stage(stage)
        self.callback_called = False
        def query_report(rigid_info : PhysxPropertyQueryRigidBodyResponse):
            self.assertEqual(rigid_info.result, PhysxPropertyQueryResult.VALID)
            self.assertEqual(rigid_info.mass, 125)
            self.callback_called = True

        path = self.defaultPrimPath + "/cube"
        prim_id = self._create_cube_mesh(stage, path, size=5, density=1)
        get_physx_property_query_interface().query_prim(stage_id = self.stage_id, prim_id = prim_id, rigid_body_fn = query_report)

        # We are creating a mesh that requires background cooking so we must wait for it to be done
        await self._wait_cooking_finished()
        self.assertTrue(self.callback_called)

    async def test_property_query_rb_mesh_timeout(self):
        stage = await self.new_stage(def_up_and_mpu=True)
    
        self.setup_stage(stage)
        self.callback_called = False
        def query_report(rigid_info : PhysxPropertyQueryRigidBodyResponse):
            self.assertEqual(rigid_info.result, PhysxPropertyQueryResult.ERROR_TIMEOUT)
            self.callback_called = True

        path = self.defaultPrimPath + "/cube"
        prim_id = self._create_cube_mesh(stage, path, size=5, density=1)
        get_physx_property_query_interface().query_prim(stage_id = self.stage_id, prim_id = prim_id, rigid_body_fn = query_report, timeout_ms = 0)

        # We are creating a mesh that requires background cooking but with a 0 timeout and we forcefully
        # sleep for 100 ms before giving it a chance to be processed, so we are sure to get the TIMEOUT error
        time.sleep(0.1)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(self.callback_called)

    async def test_property_query_rb_invalid_parsing(self):
        stage = await self.new_stage(def_up_and_mpu=True)
    
        self.setup_stage(stage)
        self.callback_called = False
        def query_report(rigid_info : PhysxPropertyQueryRigidBodyResponse):
            self.assertEqual(rigid_info.result, PhysxPropertyQueryResult.ERROR_PARSING)
            self.callback_called = True

        path = self.defaultPrimPath + "/Sphere"
        physicsUtils.add_sphere(stage, path=path)
        prim_id = PhysicsSchemaTools.sdfPathToInt(path)
        get_physx_property_query_interface().query_prim( stage_id = self.stage_id,  prim_id = prim_id, rigid_body_fn = query_report)
        # We are tring to parse rigid body on a regular sphere, so that will fail (synchronously)
        self.assertTrue(self.callback_called)

    async def execute_property_query_rb_mesh_colliders(self, stage, during_runtime : bool):
        from omni.physx import get_physx_simulation_interface
        self.rigid_callback_called = False
        self.collider_callback_called = False
        self.finished_callback_called = False
        toleranceEpsilon = 0.00001

        expected_mass = 10.55058765411377
        expected_volume = 7.033725261688232
        expected_aabb_local_max = (1.0303,1.0303,1.0303)
        expected_aabb_local_min = (-1,-1,-1)
        expected_path = self.defaultPrimPath + "/concave"
        expected_local_pos = (0, 0, 0)
        expected_local_rot = (0, 0, 0, 1)

        # Creating a concave mesh with a convex decomposition to check that we only get a single report for the compound
        query_prim_id = self._create_concave_mesh(stage, expected_path, size=1, density=1.5)

        def report_body(rigid_info : PhysxPropertyQueryRigidBodyResponse):
            self.assertEqual(rigid_info.result, PhysxPropertyQueryResult.VALID)
            self.assertTrue(abs(rigid_info.mass - expected_mass) < toleranceEpsilon)
            self.rigid_callback_called = True

        self.num_callback_called = 0
        def report_collider(collider_info : PhysxPropertyQueryColliderResponse):
            self.assertEqual(collider_info.result, PhysxPropertyQueryResult.VALID)
            self.assertEqual(str(PhysicsSchemaTools.intToSdfPath(collider_info.path_id)), expected_path)
            self.assertTrue(collider_info.aabb_local_max.x - expected_aabb_local_max[0] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_max.y - expected_aabb_local_max[1] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_max.z - expected_aabb_local_max[2] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_min.x - expected_aabb_local_min[0] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_min.y - expected_aabb_local_min[1] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_min.z - expected_aabb_local_min[2] < toleranceEpsilon)
            self.assertTrue(collider_info.volume - expected_volume < toleranceEpsilon)
            self.assertFloatIterableAlmostEqual(collider_info.local_pos, expected_local_pos, toleranceEpsilon)
            self.assertFloatIterableAlmostEqual(collider_info.local_rot, expected_local_rot, toleranceEpsilon)

            self.num_callback_called = self.num_callback_called + 1

        def report_finished():
            self.finished_callback_called = True
        if during_runtime:
            await self.stop()
            await self.step(1)
        get_physx_property_query_interface().query_prim(stage_id = self.stage_id,  prim_id = query_prim_id,
                                                        query_mode = PhysxPropertyQueryMode.QUERY_RIGID_BODY_WITH_COLLIDERS, 
                                                        finished_fn = report_finished,
                                                        rigid_body_fn = report_body,  
                                                        collider_fn = report_collider)

        # We are creating a mesh that requires background cooking so we must wait for it to be done
        await self._wait_cooking_finished()
        self.assertTrue(self.rigid_callback_called)
        self.assertTrue(self.num_callback_called == 1)
        self.assertTrue(self.finished_callback_called)
        # --------------------------------------------------------------------

        # Scale the shape, but expecting 
        # - Local AABB will not  change with scale as it's declared as a local quantity
        # - Volume will change as it's declared as a global quantity
        expected_volume = 14.067450523376465

        def report_collider_scaled(collider_info : PhysxPropertyQueryColliderResponse):
            self.assertEqual(collider_info.result, PhysxPropertyQueryResult.VALID)
            self.assertEqual(str(PhysicsSchemaTools.intToSdfPath(collider_info.path_id)), expected_path)
            self.assertTrue(collider_info.aabb_local_max.x - expected_aabb_local_max[0] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_max.y - expected_aabb_local_max[1] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_max.z - expected_aabb_local_max[2] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_min.x - expected_aabb_local_min[0] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_min.y - expected_aabb_local_min[1] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_min.z - expected_aabb_local_min[2] < toleranceEpsilon)
            self.assertTrue(collider_info.volume - expected_volume < toleranceEpsilon)
            self.num_callback_called = self.num_callback_called + 1
            self.assertFloatIterableAlmostEqual(collider_info.local_pos, expected_local_pos, toleranceEpsilon)
            self.assertFloatIterableAlmostEqual(collider_info.local_rot, expected_local_rot, toleranceEpsilon)

        self.num_callback_called = 0
        
        physicsUtils.set_or_add_scale_op(UsdGeom.Xformable.Get(stage, expected_path), Gf.Vec3f(1.0, 2.0, 1.0))
        if during_runtime:
            await self.stop()
            await self.step(1)
        get_physx_property_query_interface().query_prim(stage_id = self.stage_id,  prim_id = query_prim_id,
                                                        query_mode = PhysxPropertyQueryMode.QUERY_RIGID_BODY_WITH_COLLIDERS, 
                                                        collider_fn = report_collider_scaled)
        await self._wait_cooking_finished()
        self.assertTrue(self.num_callback_called == 1)

        # --------------------------------------------------------------------
        self.num_callback_called = 0
        expected_path = self.defaultPrimPath + "/capsule"
        expected_volume = 35.60471725463867
        expected_aabb_local_max = (1, 6, 1)
        expected_aabb_local_min = (-1, -6, -1)
        expected_local_rot = (0.707107, -0.707107, 0, 0)

        def report_collider_capsule(collider_info : PhysxPropertyQueryColliderResponse):
            self.assertEqual(collider_info.result, PhysxPropertyQueryResult.VALID)
            self.assertEqual(str(PhysicsSchemaTools.intToSdfPath(collider_info.path_id)), expected_path)
            self.assertTrue(collider_info.aabb_local_max.x - expected_aabb_local_max[0] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_max.y - expected_aabb_local_max[1] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_max.z - expected_aabb_local_max[2] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_min.x - expected_aabb_local_min[0] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_min.y - expected_aabb_local_min[1] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_min.z - expected_aabb_local_min[2] < toleranceEpsilon)
            self.assertTrue(collider_info.volume - expected_volume < toleranceEpsilon)
            self.assertFloatIterableAlmostEqual(collider_info.local_pos, expected_local_pos, toleranceEpsilon)
            self.assertFloatIterableAlmostEqual(collider_info.local_rot, expected_local_rot, toleranceEpsilon)

            self.num_callback_called = self.num_callback_called + 1

        query_prim_id = self._create_capsule(stage, expected_path, radius=1, height=10, density=1)
        if during_runtime:
            await self.stop()
            await self.step(1)
        get_physx_property_query_interface().query_prim(stage_id = self.stage_id,  prim_id = query_prim_id,
                                                        query_mode = PhysxPropertyQueryMode.QUERY_RIGID_BODY_WITH_COLLIDERS, 
                                                        collider_fn = report_collider_capsule)
        await self._wait_cooking_finished()
        self.assertTrue(self.num_callback_called == 1)

        # Xform mesh composition test
        self.num_callback_called = 0
        expected_path = self.defaultPrimPath + "/xform_rb/collider"
        expected_volume = [8.0, 27.0]
        expected_aabb_local_max = [(1, 1, 1), (1.5, 1.5, 1.5)]
        expected_aabb_local_min = [(-1, -1, -1), (-1.5, -1.5, -1.5)]
        expected_local_pos = [(2, 0, 2), (7, 0, 7)]
        expected_local_rot = [(0, 0, 0, 1), (0, 0, 0, 1)]

        def report_xform_child_collider(collider_info : PhysxPropertyQueryColliderResponse):
            self.assertEqual(collider_info.result, PhysxPropertyQueryResult.VALID)
            self.assertEqual(str(PhysicsSchemaTools.intToSdfPath(collider_info.path_id)), expected_path)

            self.assertTrue(collider_info.aabb_local_max.x - expected_aabb_local_max[self.num_callback_called][0] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_max.y - expected_aabb_local_max[self.num_callback_called][1] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_max.z - expected_aabb_local_max[self.num_callback_called][2] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_min.x - expected_aabb_local_min[self.num_callback_called][0] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_min.y - expected_aabb_local_min[self.num_callback_called][1] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_min.z - expected_aabb_local_min[self.num_callback_called][2] < toleranceEpsilon)
            self.assertTrue(collider_info.volume - expected_volume[self.num_callback_called] < toleranceEpsilon)

            self.assertFloatIterableAlmostEqual(collider_info.local_pos, expected_local_pos[self.num_callback_called], toleranceEpsilon)
            self.assertFloatIterableAlmostEqual(collider_info.local_rot, expected_local_rot[self.num_callback_called], toleranceEpsilon)

            self.num_callback_called += 1

        query_prim_id = self._create_xform_child_colliders(stage, self.defaultPrimPath + "/xform_rb", [2.0, 3.0], 10.0)
        if during_runtime:
            await self.stop()
            await self.step(1)
        get_physx_property_query_interface().query_prim(stage_id = self.stage_id,  prim_id = query_prim_id,
                                                        query_mode = PhysxPropertyQueryMode.QUERY_RIGID_BODY_WITH_COLLIDERS, 
                                                        collider_fn = report_xform_child_collider)
        await self._wait_cooking_finished()
        self.assertTrue(self.num_callback_called == 2)

        # Mesh merge test

        self.num_callback_called = 0
        expected_path = self.defaultPrimPath + "/mesh_merge"

        def report_mesh_merge_collider(collider_info : PhysxPropertyQueryColliderResponse):
            self.assertEqual(collider_info.result, PhysxPropertyQueryResult.VALID)
            self.assertEqual(str(PhysicsSchemaTools.intToSdfPath(collider_info.path_id)), expected_path)

            self.assertTrue(collider_info.aabb_local_max.x - expected_aabb_local_max[self.num_callback_called][0] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_max.y - expected_aabb_local_max[self.num_callback_called][1] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_max.z - expected_aabb_local_max[self.num_callback_called][2] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_min.x - expected_aabb_local_min[self.num_callback_called][0] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_min.y - expected_aabb_local_min[self.num_callback_called][1] < toleranceEpsilon)
            self.assertTrue(collider_info.aabb_local_min.z - expected_aabb_local_min[self.num_callback_called][2] < toleranceEpsilon)
            self.assertTrue(collider_info.volume - expected_volume[self.num_callback_called] < toleranceEpsilon)

            self.assertFloatIterableAlmostEqual(collider_info.local_pos, expected_local_pos[self.num_callback_called], toleranceEpsilon)
            self.assertFloatIterableAlmostEqual(collider_info.local_rot, expected_local_rot[self.num_callback_called], toleranceEpsilon)
            self.num_callback_called += 1

        query_prim_id = self._create_mesh_merge_collision(stage, expected_path, [2.0, 3.0], 10.0)
        if during_runtime:
            await self.stop()
            await self.step(1)
        get_physx_property_query_interface().query_prim(stage_id = self.stage_id,  prim_id = query_prim_id,
                                                        query_mode = PhysxPropertyQueryMode.QUERY_RIGID_BODY_WITH_COLLIDERS, 
                                                        collider_fn = report_mesh_merge_collider)

        await self._wait_cooking_finished()
        self.assertTrue(self.num_callback_called == 2)

    async def test_property_query_rb_mesh_colliders(self):
        stage = await self.new_stage(def_up_and_mpu=True)
        self.setup_stage(stage)
        await self.execute_property_query_rb_mesh_colliders(stage, during_runtime=False)

    async def test_property_query_rb_mesh_colliders_simulation(self):
        stage = await self.new_stage(def_up_and_mpu=True)    
        self.setup_stage(stage)
        await self.execute_property_query_rb_mesh_colliders(stage, during_runtime=True)

    async def test_property_query_articulation(self):
        stage = await self.new_stage()

        self.setup_stage(stage)
        self.callback_called = False

        def query_report(articulation_info : PhysxPropertyQueryArticulationResponse):
            self.assertEqual(articulation_info.result, PhysxPropertyQueryResult.VALID)
            links = articulation_info.links
            self.assertEqual(len(links), 2)
            self.assertEqual(links[0].rigid_body, PhysicsSchemaTools.sdfPathToInt(self.defaultPrimPath + "/xform/boxActor0"))
            self.assertEqual(links[0].rigid_body_name, self.defaultPrimPath + "/xform/boxActor0")
            self.assertEqual(links[0].joint, 0)
            self.assertEqual(links[0].joint_dof, 0)
            self.assertEqual(links[1].rigid_body, PhysicsSchemaTools.sdfPathToInt(self.defaultPrimPath + "/xform/boxActor1"))
            self.assertEqual(links[1].rigid_body_name, self.defaultPrimPath + "/xform/boxActor1")
            self.assertEqual(links[1].joint, PhysicsSchemaTools.sdfPathToInt(self.defaultPrimPath + "/xform/revoluteJoint"))
            self.assertEqual(links[1].joint_name, self.defaultPrimPath + "/xform/revoluteJoint")
            self.assertEqual(links[1].joint_dof, 1)
            self.callback_called = True
        
        top_xform = UsdGeom.Xform.Define(stage, self.defaultPrimPath + "/xform")
        UsdPhysics.ArticulationRootAPI.Apply(top_xform.GetPrim())

        path = self.defaultPrimPath + "/xform/boxActor0"
        physicsUtils.add_rigid_box(stage, path=path, size=Gf.Vec3f(5), density=1)
        path = self.defaultPrimPath + "/xform/boxActor1"
        physicsUtils.add_rigid_box(stage, path=path, size=Gf.Vec3f(5), density=1)
        prim_id = PhysicsSchemaTools.sdfPathToInt(top_xform.GetPrim().GetPrimPath())

        joint = UsdPhysics.RevoluteJoint.Define(stage, self.defaultPrimPath + "/xform/revoluteJoint")
        joint.CreateBody0Rel().SetTargets([self.defaultPrimPath + "/xform/boxActor0"])
        joint.CreateBody1Rel().SetTargets([self.defaultPrimPath + "/xform/boxActor1"])

        get_physx_property_query_interface().query_prim(stage_id = self.stage_id, query_mode = PhysxPropertyQueryMode.QUERY_ARTICULATION, prim_id = prim_id, articulation_fn = query_report)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(self.callback_called)
