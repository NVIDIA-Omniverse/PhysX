# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.test
import omni.usd
from pxr import Usd, UsdUtils, UsdPhysics, UsdGeom
import omni.timeline
from usdrt import Usd as UsdRt, Rt as UsdRtGeom
import math

class TestCubeFabricOutput(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        (result, err) = await omni.usd.get_context().new_stage_async()
        self.assertTrue(result)

        stage = omni.usd.get_context().get_stage()

        cube_prim = UsdGeom.Cube.Define(stage, "/World/cube").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(cube_prim)
        UsdPhysics.CollisionAPI.Apply(cube_prim)

    async def tearDown(self):
        (result, err) = await omni.usd.get_context().new_stage_async()
        self.assertTrue(result)

    async def test_timeline_play(self):
        stage_id = omni.usd.get_context().get_stage_id()

        usdrt_stage = UsdRt.Stage.Attach(stage_id)
        prim_usdrt = usdrt_stage.GetPrimAtPath("/World/cube")
        xformable = UsdRtGeom.Xformable(prim_usdrt)
        xformable_attr_fabric_hierarchy_world_matrix = xformable.GetFabricHierarchyWorldMatrixAttr()

        self.assertTrue(xformable_attr_fabric_hierarchy_world_matrix)
        xform_world = xformable_attr_fabric_hierarchy_world_matrix.Get()

        position = xform_world.ExtractTranslation()
        tolerance = 1e-4
        self.assertLess(math.fabs(position[1]), tolerance)

        omni.timeline.get_timeline_interface().play()

        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        xformable_attr_fabric_hierarchy_world_matrix = xformable.GetFabricHierarchyWorldMatrixAttr()

        self.assertTrue(xformable_attr_fabric_hierarchy_world_matrix)
        xform_world = xformable_attr_fabric_hierarchy_world_matrix.Get()

        position = xform_world.ExtractTranslation()
        self.assertGreater(math.fabs(position[1]), tolerance)

        omni.timeline.get_timeline_interface().stop()

    async def test_timeline_play_pause_stepping(self):
        stage_id = omni.usd.get_context().get_stage_id()

        usdrt_stage = UsdRt.Stage.Attach(stage_id)
        prim_usdrt = usdrt_stage.GetPrimAtPath("/World/cube")
        xformable = UsdRtGeom.Xformable(prim_usdrt)
        xformable_attr_fabric_hierarchy_world_matrix = xformable.GetFabricHierarchyWorldMatrixAttr()

        self.assertTrue(xformable_attr_fabric_hierarchy_world_matrix)
        xform_world = xformable_attr_fabric_hierarchy_world_matrix.Get()

        position = xform_world.ExtractTranslation()
        tolerance = 1e-4
        self.assertLess(math.fabs(position[1]), tolerance)

        omni.timeline.get_timeline_interface().play()

        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        xformable_attr_fabric_hierarchy_world_matrix = xformable.GetFabricHierarchyWorldMatrixAttr()

        self.assertTrue(xformable_attr_fabric_hierarchy_world_matrix)
        xform_world = xformable_attr_fabric_hierarchy_world_matrix.Get()

        position = xform_world.ExtractTranslation()
        new_position_y = position[1]
        self.assertGreater(math.fabs(new_position_y), tolerance)

        omni.timeline.get_timeline_interface().pause()
        omni.timeline.get_timeline_interface().commit()

        xformable_attr_fabric_hierarchy_world_matrix = xformable.GetFabricHierarchyWorldMatrixAttr()

        self.assertTrue(xformable_attr_fabric_hierarchy_world_matrix)
        xform_world = xformable_attr_fabric_hierarchy_world_matrix.Get()

        position = xform_world.ExtractTranslation()
        self.assertLess(math.fabs(position[1] - new_position_y), tolerance)

        omni.timeline.get_timeline_interface().play()
        omni.timeline.get_timeline_interface().commit()

        xformable_attr_fabric_hierarchy_world_matrix = xformable.GetFabricHierarchyWorldMatrixAttr()

        self.assertTrue(xformable_attr_fabric_hierarchy_world_matrix)
        xform_world = xformable_attr_fabric_hierarchy_world_matrix.Get()

        position = xform_world.ExtractTranslation()
        self.assertLess(math.fabs(position[1] - new_position_y), tolerance)

        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        xformable_attr_fabric_hierarchy_world_matrix = xformable.GetFabricHierarchyWorldMatrixAttr()

        self.assertTrue(xformable_attr_fabric_hierarchy_world_matrix)
        xform_world = xformable_attr_fabric_hierarchy_world_matrix.Get()

        position = xform_world.ExtractTranslation()
        self.assertGreater(math.fabs(position[1] - new_position_y), tolerance)

        omni.timeline.get_timeline_interface().stop()
