# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import carb
import omni.kit.test
import omni.kit.app
import omni.physx.bindings._physx as physx_bindings
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physx import get_physx_cooking_interface
from omni.physx.scripts.ifaces import get_physx_cooking_private_interface
from omni.physxcommands import SetRigidBodyCommand
from omni.physxtests import utils
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdUtils


class PhysXCookingTests(PhysicsKitStageAsyncTestCase):
    def _reset_cooking_statistics(self):
        self._cooking_statistics = get_physx_cooking_private_interface().get_cooking_statistics()
        num_running_tasks = self._cooking_statistics.total_scheduled_tasks - self._cooking_statistics.total_finished_tasks
        self.assertEqual(num_running_tasks, 0)

    async def _step_sim_and_reset_cooking_statistics(self):
        # start and stop the sim to clear the runtime mesh cache
        await utils.play_and_step_and_stop(self, 1)
        # reset the cooking stats AFTER starting/stopping the sim
        self._reset_cooking_statistics()

    def setup_stage(self, stage):
        cache = UsdUtils.StageCache.Get()
        self.stage_id = cache.GetId(stage).ToLongInt()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        self.defaultPrimPath = str(stage.GetDefaultPrim().GetPath())
        
        UsdPhysics.Scene.Define(stage, self.defaultPrimPath + "/physicsScene")

        self._reset_cooking_statistics()
        get_physx_cooking_interface().release_local_mesh_cache()

    def _create_cube_mesh(self, stage, path:str, size:float, density:float) -> Usd.Prim :
        cubeGeom = physicsUtils.create_mesh_cube(stage, path, size / 2)
        SetRigidBodyCommand.execute(path, "none", False)
        return cubeGeom.GetPrim()

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

    async def test_cooking(self):
        # Get the UJITSO settings dictionary
        settings = carb.settings.get_settings()
        ujitso_settings = settings.get_settings_dictionary("/UJITSO/datastore/GRPCDataStoreServer")

        # Get the child count of the UJITSO settings dictionary
        child_count = len(ujitso_settings.get_keys()) if ujitso_settings else 0

        # Check if remote cache is enabled
        use_remote_cache = settings.get_as_bool(physx_bindings.SETTING_UJITSO_REMOTE_CACHE_ENABLED)

        # Assert that remote cache is enabled only if child count is greater than 0
        assert use_remote_cache == (child_count > 0), "Remote cache enabled status doesn't match UJITSO settings"

        # This test will create a simple dynamic rigid body and ensure that it cooks and is cached correctly
        stage = await self.new_stage(def_up_and_mpu=True)
        self.setup_stage(stage)

        path = self.defaultPrimPath + "/cube"
        cube_prim = self._create_cube_mesh(stage, path, size=5, density=1)
        assert cube_prim.IsValid()

        # some cooking should be triggered by creating the cube
        await self._expect_cooking_hits_or_misses(additional_cache_misses=1, additional_cache_hits=0)
        await self._step_sim_and_reset_cooking_statistics()

        # modify the first point in the list - should trigger cooking
        mesh = UsdGeom.Mesh(cube_prim)
        points = mesh.CreatePointsAttr().Get()
        assert len(points) > 0
        points[0] = Gf.Vec3f(-points[0][0], points[0][1], points[0][2])
        mesh.CreatePointsAttr().Set(points)
        await self._expect_cooking_hits_or_misses(additional_cache_misses=1, additional_cache_hits=0)
        await self._step_sim_and_reset_cooking_statistics()

        # restore the firt point to the original value, no cooking
        points[0] = Gf.Vec3f(-points[0][0], points[0][1], points[0][2])
        mesh.CreatePointsAttr().Set(points)
        await self._expect_cooking_hits_or_misses(additional_cache_misses=0, additional_cache_hits=1)
        await self._step_sim_and_reset_cooking_statistics()

        # create a duplicate cube, should find it in the cache
        duplicate_path = self.defaultPrimPath + "/duplicate_cube"
        duplicate_cube_prim = self._create_cube_mesh(stage, duplicate_path, size=5, density=1)
        assert duplicate_cube_prim.IsValid()
        await self._expect_cooking_hits_or_misses(additional_cache_misses=0, additional_cache_hits=1)
        await self._step_sim_and_reset_cooking_statistics()
