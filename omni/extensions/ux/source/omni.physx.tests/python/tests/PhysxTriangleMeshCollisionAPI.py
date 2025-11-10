# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import unittest
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, TestCategory
from omni.physxcommands import SetRigidBodyCommand
from omni.physx.scripts import physicsUtils
from omni.physxtests import utils
from omni.physx import get_physx_cooking_interface, get_physx_cooking_private_interface
from pxr import Gf, UsdPhysics, UsdGeom, Sdf, PhysxSchema
import omni.timeline


class PhysxTriangleMeshCollisionAPITestKitStage(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Core

    def _reset_cooking_statistics(self):
        self._cooking_statistics = get_physx_cooking_private_interface().get_cooking_statistics()
        num_running_tasks = self._cooking_statistics.total_scheduled_tasks - self._cooking_statistics.total_finished_tasks
        self.assertEqual(num_running_tasks, 0)

    async def _wait_cooking_finished(self):
        while True:
            await omni.kit.app.get_app().next_update_async()
            cooking_statistics = get_physx_cooking_private_interface().get_cooking_statistics()
            running_tasks = cooking_statistics.total_scheduled_tasks - cooking_statistics.total_finished_tasks
            self.assertGreaterEqual(running_tasks, 0)
            if running_tasks <= 0:
                break
            
    async def _expect_cooking_hits_or_misses(self, additional_cache_misses, additional_cache_hits):
        # wait until all running tasks are finished
        await self._wait_cooking_finished()
        cooking_statistics = get_physx_cooking_private_interface().get_cooking_statistics()
        cache_misses = cooking_statistics.total_finished_cache_miss_tasks - self._cooking_statistics.total_finished_cache_miss_tasks
        cache_hits = cooking_statistics.total_finished_cache_hit_tasks - self._cooking_statistics.total_finished_cache_hit_tasks
        self.assertEqual(cache_misses, additional_cache_misses)
        self.assertEqual(cache_hits, additional_cache_hits)

    async def setup_stage(self):
        self._stage = await utils.new_stage_setup()
        self._reset_cooking_statistics()

        # Reset cache
        get_physx_cooking_interface().release_local_mesh_cache()

        utils.execute_and_check(self, "AddPhysicsScene", stage=self._stage, path="/physicsScene")
        physicsUtils.add_ground_plane(self._stage, "/GroundPlane", "Y", 25.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        # setup a dynamic rigid body cube with collision using SDF
        self._cube_path = Sdf.Path("/cube")
        self._cube_mesh = physicsUtils.create_mesh_cube(self._stage, self._cube_path, 0.25)
        physicsUtils.set_or_add_translate_op(self._cube_mesh, Gf.Vec3f(0.0, 5.0, 0.0))
        SetRigidBodyCommand.execute(self._cube_path, "none", False)

        # This is something else. The command accepts a prim_path argument, but then sets it as the default path
        # and searches for the next free path under it, which means it's not going to be constructed in the path
        # I want it to be.
        # then the command returns the path, except it returns a tuple with the path in the second part. WTF.
        path_tuple = omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Torus")
        self._torus_path = path_tuple[1]
        self._torus_mesh = UsdGeom.Mesh.Get(self._stage, self._torus_path)
        self.assertTrue(self._torus_mesh)

        # placement + rigid body.
        physicsUtils.set_or_add_scale_op(self._torus_mesh, Gf.Vec3f(0.02))
        physicsUtils.set_or_add_translate_op(self._torus_mesh, Gf.Vec3f(0.0, 1.0, 0.0))
        SetRigidBodyCommand.execute(self._torus_path, "none", False)        

    @unittest.skip("NVBug 5454903")
    async def test_sdf_cooking(self):
        await self.setup_stage()

        # change SDF params and check if recooking happens.
        meshCollision = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(self._stage.GetPrimAtPath(self._torus_path))
        meshCollision.CreateSdfResolutionAttr().Set(1000)

        # need to explicitly change to triangle mesh to start SDF cooking.
        meshcollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(self._stage.GetPrimAtPath(self._torus_path))
        meshcollisionAPI.CreateApproximationAttr().Set("sdf")

        # some cooking should be happening.
        await self._expect_cooking_hits_or_misses(additional_cache_misses=2, additional_cache_hits=0)
        self._reset_cooking_statistics()

        # change SDF resolution to 300 - should trigger cooking
        meshCollision.CreateSdfResolutionAttr().Set(1100)
        await self._expect_cooking_hits_or_misses(additional_cache_misses=1, additional_cache_hits=0)
        self._reset_cooking_statistics()

        # change back to 256, no cooking
        meshCollision.CreateSdfResolutionAttr().Set(1000)
        await self._expect_cooking_hits_or_misses(additional_cache_misses=0, additional_cache_hits=0)
        self._reset_cooking_statistics()

        # change bits per Subgrid pixel - should trigger recooking
        meshCollision.CreateSdfBitsPerSubgridPixelAttr().Set("BitsPerPixel8")
        await self._expect_cooking_hits_or_misses(additional_cache_misses=1, additional_cache_hits=0)
        self._reset_cooking_statistics()

        # change margin - should recook
        meshCollision.CreateSdfMarginAttr().Set(0.02)
        await self._expect_cooking_hits_or_misses(additional_cache_misses=1, additional_cache_hits=0)
        self._reset_cooking_statistics()

        # change narrow band thickness - should recook
        meshCollision.CreateSdfNarrowBandThicknessAttr().Set(0.02)
        await self._expect_cooking_hits_or_misses(additional_cache_misses=1, additional_cache_hits=0)
        self._reset_cooking_statistics()

        # change remeshing - should recook
        meshCollision.CreateSdfResolutionAttr().Set(256)
        meshCollision.CreateSdfEnableRemeshingAttr().Set(True)
        await self._expect_cooking_hits_or_misses(additional_cache_misses=1, additional_cache_hits=0)
        self._reset_cooking_statistics()

        # change remeshing - should recook
        meshCollision.CreateSdfTriangleCountReductionFactorAttr().Set(0.5)
        await self._expect_cooking_hits_or_misses(additional_cache_misses=1, additional_cache_hits=0)
        self._reset_cooking_statistics()

        # sanity check - cube should fall into the torus if SDF collision is on
        await utils.play_and_step_and_pause(self, 10)

        # it's already cooked so wt don't expect anything to be cooked or read from cache
        await self._expect_cooking_hits_or_misses(additional_cache_misses=0, additional_cache_hits=0)
        pos = omni.usd.get_world_transform_matrix(self._cube_mesh).ExtractTranslation()
        self.assertLess(pos[1], 1.0)

    @unittest.skip("NVBug 5454903")
    async def test_sdf_change_approximation(self):

        await self.setup_stage()

        # set SDF
        meshCollision = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(self._stage.GetPrimAtPath(self._torus_path))
        meshCollision.CreateSdfResolutionAttr().Set(256)

        # need to explicitly change to triangle mesh to start SDF cooking.
        meshcollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(self._stage.GetPrimAtPath(self._torus_path))
        meshcollisionAPI.CreateApproximationAttr().Set("sdf")

        # some cooking should be happening.
        await self._expect_cooking_hits_or_misses(additional_cache_hits=0, additional_cache_misses=2)

        # play - should fall into cube
        await utils.play_and_step_and_pause(self, 10)
        pos = omni.usd.get_world_transform_matrix(self._cube_mesh).ExtractTranslation()
        self.assertLess(pos[1], 1.0)

        # stop the sim - should force a reset
        await utils.play_and_step_and_stop(self, 1)

        # change back to convex hull
        meshcollisionAPI.CreateApproximationAttr().Set("convexHull")

        # should not fall into cube
        await utils.play_and_step_and_pause(self, 10)
        pos = omni.usd.get_world_transform_matrix(self._cube_mesh).ExtractTranslation()
        self.assertGreater(pos[1], 1.0)

        self._reset_cooking_statistics()

        # back to SDF, should load from cache and fall into torus again.
        meshcollisionAPI.CreateApproximationAttr().Set("sdf")

        # no cooking should happen
        await self._expect_cooking_hits_or_misses(additional_cache_hits=1, additional_cache_misses=0)

        # play - should fall into cube
        await utils.play_and_step_and_pause(self, 10)
        pos = omni.usd.get_world_transform_matrix(self._cube_mesh).ExtractTranslation()
        self.assertLess(pos[1], 1.0)


    async def test_sdf_load_from_file(self):
        filename = "sdf.usda"

        await self.setup_stage()

         # set SDF
        meshCollision = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(self._stage.GetPrimAtPath(self._torus_path))
        meshCollision.CreateSdfResolutionAttr().Set(256)

        # need to explicitly change to triangle mesh to start SDF cooking.
        meshcollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(self._stage.GetPrimAtPath(self._torus_path))
        meshcollisionAPI.CreateApproximationAttr().Set("sdf")

        # some cooking should be happening.
        await self._expect_cooking_hits_or_misses(additional_cache_hits=0, additional_cache_misses=2)

        # save the file
        self._stage.Export(filename)

        # close this stage by opening a new one.
        await omni.usd.get_context().new_stage_async()

        # wipe the cooking cache
        get_physx_cooking_interface().release_local_mesh_cache()

        self._reset_cooking_statistics()

        # load the file again
        await omni.usd.get_context().open_stage_async(filename)
        new_stage = omni.usd.get_context().get_stage()
        
        # no cooking should be happening
        await self._expect_cooking_hits_or_misses(additional_cache_hits=0, additional_cache_misses=0)

        # play as sanity check.
        await utils.play_and_step_and_pause(self, 10)

        # As local mesh cache was released, playing forces to cook again
        await self._expect_cooking_hits_or_misses(additional_cache_hits=0, additional_cache_misses=2)

        pos = omni.usd.get_world_transform_matrix(new_stage.GetPrimAtPath(self._cube_path)).ExtractTranslation()
        self.assertLess(pos[1], 1.0)

    async def test_SDF_wait_for_cooking_to_finish(self):
        stage = await self.new_stage()
        omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Cube", prim_path="/cube")
        cube = UsdGeom.Xform.Get(stage, "/World/cube")
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        meshcollision = UsdPhysics.MeshCollisionAPI.Apply(cube.GetPrim())
        PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(cube.GetPrim())
        meshcollision.CreateApproximationAttr("sdf")
        await self._wait_cooking_finished()

    async def test_convex_hull_wait_for_cooking_to_finish(self):
        stage = await self.new_stage()
        omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Cube", prim_path="/cube")
        cube = UsdGeom.Xform.Get(stage, "/World/cube")
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        meshcollision = UsdPhysics.MeshCollisionAPI.Apply(cube.GetPrim())
        PhysxSchema.PhysxConvexHullCollisionAPI.Apply(cube.GetPrim())
        meshcollision.CreateApproximationAttr("convexHull")
        await self._wait_cooking_finished()

    async def test_convex_decomposition_wait_for_cooking_to_finish(self):
        stage = await self.new_stage()
        omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Cube", prim_path="/cube")
        cube = UsdGeom.Xform.Get(stage, "/World/cube")
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        meshcollision = UsdPhysics.MeshCollisionAPI.Apply(cube.GetPrim())
        PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(cube.GetPrim())
        meshcollision.CreateApproximationAttr("convexDecomposition")
        await self._wait_cooking_finished()
