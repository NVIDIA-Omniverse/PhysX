# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx.scripts import physicsUtils
from omni.physxtestsvisual.utils import TestCase, GOLDEN_DIR
from omni.physxtests.utils.physicsBase import TestCategory
from pxr import Gf
import omni.kit.app
import omni.physx.bindings._physx as physx_bindings
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.physx.scripts.utils as core_utils
from omni.physx.scripts.utils import MESH_APPROXIMATIONS
import omni.ui as ui
from omni.physx import get_physx_cooking_interface, get_physx_cooking_private_interface
from omni.physxui import get_physxui_interface, get_physxui_private_interface


class PhysxDebugVisTest(TestCase):
    category = TestCategory.Core

    def __init__(self, tests=()):
        super().__init__(tests)
        self._test_window_width = 1000
        self._test_window_height = 800
        self._use_distant_light = True
        self._viewport_settings[physx_bindings.SETTING_DEBUG_VIS_QUERY_USDRT_FOR_TRAVERSAL] = (False, False)
        self._viewport_settings[physx_bindings.SETTING_DISPLAY_COLLIDERS] = (2, 0)
        self._viewport_settings[physx_bindings.SETTING_DEBUG_VIS_SIMPLIFY_AT_DISTANCE] = (1500.0, 2500.0)
        self._viewport_settings["/app/window/hideUi"] = (True, False)

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

    async def setup_stage(self, stage, use_usdrt: bool = False):
        print("Setting up stage...")
        self._viewport_settings[physx_bindings.SETTING_DEBUG_VIS_QUERY_USDRT_FOR_TRAVERSAL] = (use_usdrt, False)
        await self.setup_viewport_test(self._test_window_width, self._test_window_height)
        await self._setup_render_settings(self._use_distant_light)
        window = ui.Workspace.get_window("Viewport")
        window.padding_x = 0
        window.padding_y = 0
        window.position_x = 0
        window.position_y = 0
        window.noTabBar = True
        window.flags = (
            ui.WINDOW_FLAGS_NO_SCROLLBAR
            | ui.WINDOW_FLAGS_NO_TITLE_BAR
            | ui.WINDOW_FLAGS_NO_RESIZE
            | ui.WINDOW_FLAGS_NO_CLOSE
            | ui.WINDOW_FLAGS_NO_COLLAPSE
        )
        window.auto_resize = False
        window.width = self._test_window_width
        window.height = self._test_window_height
        get_physxui_interface().enable_redraw_optimizations(False)  # all debug vis drawing operations will be instant

    async def restore_stage_settings(self):
        get_physxui_interface().enable_redraw_optimizations(True)
        await self.restore()

    async def load_test_stage(self, filename):
        file_path = GOLDEN_DIR.joinpath("DebugVisTestStages").joinpath(filename)
        usd_context = omni.usd.get_context()
        path_str = str(file_path.absolute())
        print(f"Loading test stage '{path_str}'...")
        await usd_context.open_stage_async(path_str)
        return usd_context.get_stage()

    async def wait_for_render(self):
        await self.wait(10)

    async def run_visual_test(self, stage, img_suffix, use_usdrt, custom_fnc=None, setup_and_restore=False, threshold=0.001) -> bool:
        print(f"Capturing [{img_suffix}]{' [usdrt]' if use_usdrt else ''}{' [w/custom_fnc]' if custom_fnc else ''}{' [setup_and_restore]' if setup_and_restore else ''}")
        await self._wait_cooking_finished()
        if setup_and_restore:
            await self.setup_stage(stage, use_usdrt)
        await self.wait_for_render()
        r = await self.do_visual_test(
            img_name="",
            img_suffix=f"{img_suffix}{'_usdrt' if use_usdrt else ''}",
            use_distant_light=self._use_distant_light,
            skip_assert=True,
            threshold=threshold,
            use_renderer_capture=True,
            setup_and_restore=setup_and_restore
        )
        if custom_fnc is not None:
            r = custom_fnc() and r  # Notice the order of AND operands -> custom_fnc is intentionally called even if r is false
        return r

    # ====================================== TESTS ======================================

    async def test_physics_debug_vis_basic(self):

        def get_img_file_name(suffix):
            return f"test_physics_debug_vis_basic_{suffix}"

        async def run_tests(use_usdrt: bool = False) -> bool:
            all_tests_passed = True

            print("Creating test stage...")
            stage = await self.new_stage()
            await self.wait_for_render()
            camera_state = ViewportCameraState()
            camera_state.set_position_world(Gf.Vec3d(450.0, 50.0, 0.0), True)
            camera_state.set_target_world(Gf.Vec3d(-50.0, 50.0, 0.0), True)
            cube0 = physicsUtils.create_mesh_cube(stage, "/World/cube0", 40.0)
            cube0.AddTranslateOp().Set(Gf.Vec3d(0.0, 50.0, 100.0))
            cube0.AddRotateXYZOp().Set(Gf.Vec3d(5, 140, -5))
            cube1 = physicsUtils.create_mesh_cube(stage, "/World/cube1", 50.0)
            cube1.AddTranslateOp().Set(Gf.Vec3d(0.0, 50.0, -100.0))
            cube1.AddRotateXYZOp().Set(Gf.Vec3d(100, -30, 30))
            primitives = (cube0, cube1)

            await self.setup_stage(stage, use_usdrt)

            # Special case for approximations which do not support scale properly
            unscaled_approximations = ["sphereFill"]
            for approx in unscaled_approximations:
                for primitive in primitives:
                    prim = primitive.GetPrim()
                    core_utils.removeCollider(prim)
                    core_utils.setCollider(prim, approx)
                all_tests_passed = (
                    await self.run_visual_test(
                        stage,
                        get_img_file_name(str.lower(approx)),
                        use_usdrt,
                        threshold=0.01  # sphere-fill seems to be giving unstable results on Linux. Is it still experimental?
                    )
                    and all_tests_passed
                )

            # add a scale
            cube0.AddScaleOp().Set(Gf.Vec3d(2, 1.5, 0.5))
            cube1.AddScaleOp().Set(Gf.Vec3d(1.5, 1.0, 0.8))

            ignore_list = ["convexMeshSimplification"] + unscaled_approximations
            for approx in MESH_APPROXIMATIONS.keys():
                if approx in ignore_list:
                    continue
                for primitive in primitives:
                    prim = primitive.GetPrim()
                    core_utils.removeCollider(prim)
                    core_utils.setCollider(prim, approx)
                # Notice the order of AND operands -> visual test is intentionally performed even if
                # all_tests_passed is false (previous test failed) to get all results in one batch.
                all_tests_passed = (
                    await self.run_visual_test(
                        stage,
                        get_img_file_name(str.lower(approx)),
                        use_usdrt
                    )
                    and all_tests_passed
                )

            await self.restore_stage_settings()

            return all_tests_passed

        self.assertTrue(await run_tests(False))

        await omni.usd.get_context().close_stage_async()

    async def test_physics_debug_vis_multi(self):

        def get_img_file_name(suffix):
            return f"test_physics_debug_vis_multi_{suffix}"

        async def run_tests(use_usdrt: bool = False) -> bool:
            all_tests_passed = True

            stage = await self.load_test_stage("debugvis.usda")
            self.assertIsNotNone(stage)
            await self.wait_for_render()

            await self.setup_stage(stage, use_usdrt)

            camera_state = ViewportCameraState()
            init_pos = Gf.Vec3d(0, 800, 700)
            target_vec = Gf.Vec3d(0.0, 0.0, 50.0)

            camera_state.set_position_world(init_pos, True)
            camera_state.set_target_world(target_vec, True)
            # Notice the order of AND operands -> visual test is intentionally performed even if
            # all_tests_passed is false (previous test failed) to get all results in one batch.
            all_tests_passed = await self.run_visual_test(
                stage,
                get_img_file_name(0),
                use_usdrt,
                lambda: get_physxui_private_interface().test_debug_vis_internal_state(0)
            ) and all_tests_passed

            camera_state.set_position_world((0, 1700, 1500), True)
            camera_state.set_target_world(target_vec, True)
            all_tests_passed = await self.run_visual_test(
                stage,
                get_img_file_name(1),
                use_usdrt,
                lambda: get_physxui_private_interface().test_debug_vis_internal_state(1)
            ) and all_tests_passed

            camera_state.set_position_world((0, 1270, 1100), True)
            camera_state.set_target_world(target_vec, True)
            all_tests_passed = await self.run_visual_test(
                stage,
                get_img_file_name(2),
                use_usdrt,
                lambda: get_physxui_private_interface().test_debug_vis_internal_state(2)
            ) and all_tests_passed

            camera_state.set_position_world(init_pos, True)
            camera_state.set_target_world(target_vec, True)
            all_tests_passed = await self.run_visual_test(
                stage,
                get_img_file_name(3),
                use_usdrt,
                lambda: get_physxui_private_interface().test_debug_vis_internal_state(0)
            ) and all_tests_passed

            await self.restore_stage_settings()

            return all_tests_passed

        self.assertTrue(await run_tests(False))
        self.assertTrue(await run_tests(True))

        await omni.usd.get_context().close_stage_async()
