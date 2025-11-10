# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.kit.viewport.utility.camera_state import ViewportCameraState
from omni.physxtestsvisual.utils import TestCase
from omni.physxtests.utils.physicsBase import TestCategory, PhysicsBaseAsyncTestCase
from omni.physx.scripts import physicsUtils
import omni.usd
import omni.physx
from pxr import Gf, UsdPhysics, UsdGeom, UsdLux, Sdf, Usd
from omni.physx.bindings._physx import SETTING_VISUALIZATION_COLLISION_MESH
import omni.ui as ui
import unittest
from omni.physx import get_physx_cooking_interface, get_physx_cooking_private_interface


class PhysxSolidColliderVisualization(TestCase):
    category = TestCategory.Core

    def __init__(self, tests=()):
        super().__init__(tests)
        self._viewport_settings[SETTING_VISUALIZATION_COLLISION_MESH] = (True, False)
        self._viewport_settings["/app/window/hideUi"] = (True, False)
        self._viewport_settings["/app/transform/operation"] = ("select", "move")

    async def setUp(self):
        await super().setUp()
        get_physx_cooking_interface().release_local_mesh_cache()

    async def tearDown(self):
        await super().tearDown()

    async def _wait_cooking_finished(self):
        while True:
            await omni.kit.app.get_app().next_update_async()
            cooking_statistics = get_physx_cooking_private_interface().get_cooking_statistics()
            running_tasks = cooking_statistics.total_scheduled_tasks - cooking_statistics.total_finished_tasks
            self.assertGreaterEqual(running_tasks, 0)
            if running_tasks <= 0:
                break

    async def test_physics_ui_solid_collider(self):
        all_tests_passed = True

        async def do_test():
            stage : Usd.Stage = await self.new_stage()
            await self.wait(120) # Wait for rendering
            camera_state = ViewportCameraState()
            camera_state.set_position_world(Gf.Vec3d(500.0, 50.0, 0.0), True)
            camera_state.set_target_world(Gf.Vec3d(0.0, 50.0, 0.0), True)

            concave_path = "/World/Concave"
            concave_prim = physicsUtils.create_mesh_concave(stage, concave_path, 50)
            concave_xform = UsdGeom.Xform(concave_prim.GetPrim())
            concave_xform.AddRotateXYZOp().Set(Gf.Vec3d(-50,50,70))

            path_tuple = omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Torus")
            torus_path = path_tuple[1]
            torus_prim = stage.GetPrimAtPath(torus_path)
            torus_xform : UsdGeom.Xform = UsdGeom.Xform(torus_prim)
            xformops = torus_xform.GetOrderedXformOps()
            xformops[0].Set(Gf.Vec3d(0,0, 160))
            xformops[1].Set(Gf.Vec3d(0,0, 80))
            torus_collision : UsdPhysics.CollisionAPI = UsdPhysics.CollisionAPI.Apply(torus_prim)
            torus_collision : UsdPhysics.MeshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(torus_prim)
            torus_collision.CreateApproximationAttr("convexDecomposition")

            self._selection = omni.usd.get_context().get_selection()
            self._selection.set_selected_prim_paths([concave_path], True)

            window = ui.Workspace.get_window("Viewport")
            test_window_width = 1000
            test_window_height = 800

            async def apply_test_configuration():
                await self.setup_viewport_test(test_window_width, test_window_height)
                window.padding_x = 0
                window.padding_y = 0
                window.position_x = 0
                window.position_y = 0
                window.noTabBar = True
                window.flags = ui.WINDOW_FLAGS_NO_SCROLLBAR | ui.WINDOW_FLAGS_NO_TITLE_BAR | ui.WINDOW_FLAGS_NO_RESIZE | ui.WINDOW_FLAGS_NO_CLOSE | ui.WINDOW_FLAGS_NO_COLLAPSE
                window.auto_resize = False
                window.width = test_window_width
                window.height = test_window_height

            nonlocal all_tests_passed

            # Test 1: Initial setup
            await self.wait(30)
            await apply_test_configuration()
            await self.wait(120) # Wait for rendering
            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_solid_collider_no_collider_0",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            )

            # Test 2: Change Concave Prim approximation to Convex Decomposition
            await self.wait(30)
            await apply_test_configuration()
            concave_collision : UsdPhysics.CollisionAPI = UsdPhysics.CollisionAPI.Apply(concave_prim.GetPrim())
            concave_collision : UsdPhysics.MeshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(concave_prim.GetPrim())
            concave_collision.CreateApproximationAttr("convexDecomposition")
            await self.wait(30)
            await self._wait_cooking_finished()
            await self.wait(120) # Wait for rendering

            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_solid_collider_convex_decomposition",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            )
            
            # Test 3: Change Concave Prim approximation to Convex Hull
            await self.wait(30)
            await apply_test_configuration()
            concave_collision.GetApproximationAttr().Set("convexHull")
            await self.wait(30)
            await self._wait_cooking_finished()
            await self.wait(120) # Wait for rendering

            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_solid_collider_convex_hull",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            )

            # Test 4: Deselect Concave Prim
            await self.wait(30)
            await apply_test_configuration()
            self._selection.set_selected_prim_paths([], True)
            await self.wait(10) # Wait for rendering
            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_solid_collider_no_collider_1",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            )            
        
            # Test 5: Select Torus
            await self.wait(30)
            await apply_test_configuration()
            self._selection.set_selected_prim_paths([torus_path], True)
            await self.wait(30)
            await self._wait_cooking_finished()
            await self.wait(120) # Wait for rendering
            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_solid_collider_torus_decomposition",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            )
            
            # Test 6: Select Torus and Convex
            await self.wait(30)
            await apply_test_configuration()
            self._selection.set_selected_prim_paths([torus_path, concave_path], True)
            await self.wait(30)
            await self._wait_cooking_finished()
            await self.wait(120) # Wait for rendering
            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_solid_collider_torus_and_concave",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            )

            # Test 7: Deselect both
            await self.wait(30)
            await apply_test_configuration()
            self._selection.set_selected_prim_paths([], True)
            await self.wait(10) # Wait for rendering
            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_solid_collider_no_collider_2",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            )       
        
        await do_test()

        await omni.usd.get_context().close_stage_async()
        await self.new_stage()
        
        self.assertTrue(all_tests_passed)
