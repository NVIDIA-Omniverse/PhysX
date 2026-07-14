# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Ported from omni.physx.tests PhysxSimulationStatistics.py for standalone ovruntime testing.

import _physics_setup  # noqa: F401 - loads Carbonite + physics plugins + PhysX schemas

import unittest
from physicsBase import PhysicsMemoryStageBaseTestCase, TestCategory, ExpectMessage
from pxr import Gf, Usd, UsdPhysics, UsdGeom, Sdf, UsdUtils, PhysicsSchemaTools
import _physx


class PhysxStatisticsTestMemoryStage(PhysicsMemoryStageBaseTestCase):
    category = TestCategory.Core

    def test_physx_statistics_stage_error(self):
        stage = self.new_stage(attach_stage=False)

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        path = PhysicsSchemaTools.sdfPathToInt("/physicsScene")
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        stats = _physx.PhysicsSceneStats()

        # get stats from not loaded stage, should return false
        message = f"getPhysXSceneStatistics: stageId {stage_id} not attached."
        with ExpectMessage(self, message):
            ret_val = _physx.acquire_physx_statistics_interface().get_physx_scene_statistics(stage_id, path, stats)
            self.assertTrue(ret_val == False)

    def test_physx_statistics_base(self):
        stage = self.new_stage(attach_stage=False)

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        cube = UsdGeom.Cube.Define(stage, "/cube")
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

        path = PhysicsSchemaTools.sdfPathToInt("/physicsScene")
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        stats = _physx.PhysicsSceneStats()

        # load stage
        _physx.acquire_physx_simulation_interface().attach_stage(stage_id)

        _physx.acquire_physx_simulation_interface().simulate(0.01, 0.01)
        _physx.acquire_physx_simulation_interface().fetch_results()

        ret_val = _physx.acquire_physx_statistics_interface().get_physx_scene_statistics(stage_id, path, stats)
        self.assertTrue(ret_val == True)

        self.assertTrue(stats.nb_dynamic_rigids == 1)
        self.assertTrue(stats.nb_active_dynamic_rigids == 1)
        self.assertTrue(stats.nb_box_shapes == 1)

        _physx.acquire_physx_simulation_interface().detach_stage()


if __name__ == "__main__":
    unittest.main()
