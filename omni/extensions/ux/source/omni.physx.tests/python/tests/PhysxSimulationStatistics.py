# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import unittest
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory
from omni.physx import get_physx_statistics_interface, get_physx_simulation_interface
from pxr import Gf, Usd, UsdPhysics, UsdGeom, Sdf, UsdUtils, PhysicsSchemaTools
from omni.physx.bindings._physx import PhysicsSceneStats
from omni.physxtests import utils


class PhysxStatisticsTestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core
    
    async def test_physx_statistics_stage_error(self):
        stage = await self.new_stage(attach_stage=False)        

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        path = PhysicsSchemaTools.sdfPathToInt("/physicsScene")
        stage_id = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()

        stats = PhysicsSceneStats()

        # get stats from not loaded stage, should return false
        message = f"getPhysXSceneStatistics: stageId {stage_id} not attached."
        with utils.ExpectMessage(self, message):
            ret_val = get_physx_statistics_interface().get_physx_scene_statistics(stage_id, path, stats)
            self.assertTrue(ret_val == False)

    async def test_physx_statistics_base(self):
        stage = await self.new_stage(attach_stage=False)        

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        
        cube = UsdGeom.Cube.Define(stage, "/cube")
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

        path = PhysicsSchemaTools.sdfPathToInt("/physicsScene")
        stage_id = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()

        stats = PhysicsSceneStats()

        # load stage
        get_physx_simulation_interface().attach_stage(stage_id)        

        get_physx_simulation_interface().simulate(0.01, 0.01)
        get_physx_simulation_interface().fetch_results()

        ret_val = get_physx_statistics_interface().get_physx_scene_statistics(stage_id, path, stats)
        self.assertTrue(ret_val == True)

        self.assertTrue(stats.nb_dynamic_rigids == 1)
        self.assertTrue(stats.nb_active_dynamic_rigids == 1)
        self.assertTrue(stats.nb_box_shapes == 1)

        get_physx_simulation_interface().detach_stage()
