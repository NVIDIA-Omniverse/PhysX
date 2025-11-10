# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.kit.viewport.utility.camera_state import ViewportCameraState
from omni.physxtestsvisual.utils import TestCase
from omni.physxtests.utils.physicsBase import TestCategory
from omni.physx.scripts import physicsUtils
import omni.usd
from carb.settings import get_settings
from pxr import Gf, UsdGeom, UsdPhysics
from omni.physxui.scripts.physicsViewportMassEdit import SETTING_DISPLAY_MASS_PROPERTIES_NONE, SETTING_DISPLAY_MASS_PROPERTIES_SELECTED, SETTING_DISPLAY_MASS_PROPERTIES_ALL
from omni.physx.bindings._physx import SETTING_MASS_DISTRIBUTION_MANIPULATOR, SETTING_DISPLAY_MASS_PROPERTIES
import omni.ui as ui

class PhysxMassVisualizationTest(TestCase):
    category = TestCategory.Core

    def __init__(self, tests=()):
        super().__init__(tests)
        self._viewport_settings[SETTING_DISPLAY_MASS_PROPERTIES] = (SETTING_DISPLAY_MASS_PROPERTIES_NONE, SETTING_DISPLAY_MASS_PROPERTIES_NONE)
        self._viewport_settings[SETTING_MASS_DISTRIBUTION_MANIPULATOR] = (True, False)
        self._viewport_settings["/app/window/hideUi"] = (True, False)
        self._viewport_settings["/app/transform/operation"] = ("select", "move")

        self._render_settings["/ngx/enabled"] = (False, True)
        self._render_settings["/rtx/indirectDiffuse/enabled"] = (False, True)
        self._render_settings["/rtx/sceneDB/ambientLightIntensity"] = (0.0, 0.0)
        self._render_settings["/rtx/directLighting/sampledLighting/enabled"] = (False, False)
        self._render_settings["/rtx/directLighting/sampledLighting/autoEnable"] = (False, True)
        self._render_settings["/rtx/newDenoiser/enabled"] = (False, True)

    async def setUp(self):
        await super().setUp()

    async def tearDown(self):
        await super().tearDown()

    async def _timeline_play_steps(self, num_frames):
        omni.timeline.get_timeline_interface().play()
        await self.wait(num_frames)
        omni.timeline.get_timeline_interface().stop()

    async def test_physics_ui_mass_visualization(self):

        all_tests_passed = True

        settings = get_settings()
        setting_initial_view = settings.get_as_int(SETTING_DISPLAY_MASS_PROPERTIES)
        setting_initial_edit = settings.get_as_bool(SETTING_MASS_DISTRIBUTION_MANIPULATOR)

        async def do_test():
            stage = await self.new_stage()
            await self.wait(10)
            camera_state = ViewportCameraState()
            camera_state.set_position_world(Gf.Vec3d(500.0, 50.0, 0.0), True)
            camera_state.set_target_world(Gf.Vec3d(0.0, 50.0, 0.0), True)
            physicsUtils.add_rigid_box(stage, "/World/Cube", 100.0, Gf.Vec3f(0.0, 50.0, 100.0), Gf.Quatf(0.0, 0.3826834, 0.0, 0.9238796))
            sphere = physicsUtils.add_sphere(stage, "/World/Sphere", 50.0, Gf.Vec3f(0.0, 50.0, -100.0))
            selection = omni.usd.get_context().get_selection()
            selection.set_selected_prim_paths(["/World/Cube"], True)
            await self.wait(10)

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
                window.flags = (
                                ui.WINDOW_FLAGS_NO_TITLE_BAR |
                                ui.WINDOW_FLAGS_NO_CLOSE |
                                ui.WINDOW_FLAGS_NO_COLLAPSE |
                                ui.WINDOW_FLAGS_NO_MOVE |
                                ui.WINDOW_FLAGS_NO_RESIZE |
                                ui.WINDOW_FLAGS_NO_SCROLLBAR
                                )
                window.auto_resize = False
                window.width = test_window_width
                window.height = test_window_height

            await apply_test_configuration()

            await self.wait(10)

            nonlocal all_tests_passed
            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_mass_visualization_cube",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            )

            await apply_test_configuration()
            selection.set_selected_prim_paths(["/World/Sphere"], True)
            physicsUtils._add_rigid(sphere, 1.0, Gf.Vec3f(0.0), Gf.Vec3f(0.0))
            await self.wait(10)

            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_mass_visualization_sphere",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            )
            
            await apply_test_configuration()
            xform = UsdGeom.Xform.Define(stage, "/World/Xform")
            physicsUtils.set_or_add_scale_orient_translate(xform, scale=Gf.Vec3f(1.0), orient=Gf.Quatf(0, 0, -0.4871745, -0.8733046), translate=Gf.Vec3f(0.0, 50.0, 0.0))
            selection.set_selected_prim_paths(["/World/Xform"], True)
            physicsUtils._add_rigid(xform.GetPrim(), 1.0, Gf.Vec3f(0.0), Gf.Vec3f(0.0))
            mass_api = UsdPhysics.MassAPI(xform)
            mass_api.CreateMassAttr().Set(1.0)
            mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 10.0)) 
            mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(5000.0, 10000.0, 2500.0))
            mass_api.CreatePrincipalAxesAttr().Set(Gf.Quatf(-0.5678994, 0.3701006, -0.3168037, 0.663439))
            await self.wait(10)

            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_mass_visualization_xform",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            ) 

            await apply_test_configuration()
            mass_api.GetMassAttr().Set(3.0)
            mass_api.GetCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0)) 
            mass_api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(5000.0, 10000.0, 10000.0))
            mass_api.GetPrincipalAxesAttr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            await self.wait(10) 

            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_mass_visualization_xform_update",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            ) 

            await apply_test_configuration()
            selection.set_selected_prim_paths(["/World/Xform", "/World/Cube", "/World/Sphere"], True)
            await self.wait(10) 

            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_mass_visualization_mult",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            )

            await apply_test_configuration()
            selection.set_selected_prim_paths([], True)
            await self.wait(10)

            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_mass_visualization_none",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            )

            await apply_test_configuration()
            settings.set(SETTING_MASS_DISTRIBUTION_MANIPULATOR, False)
            settings.set(SETTING_DISPLAY_MASS_PROPERTIES, SETTING_DISPLAY_MASS_PROPERTIES_ALL)
            await self.wait(10)

            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_mass_visualization_view_all",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            )


            await apply_test_configuration()
            selection.set_selected_prim_paths(["/World/Cube"], True)
            settings.set(SETTING_MASS_DISTRIBUTION_MANIPULATOR, False)
            settings.set(SETTING_DISPLAY_MASS_PROPERTIES, SETTING_DISPLAY_MASS_PROPERTIES_SELECTED)
            await self.wait(10)

            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_mass_visualization_view_selection",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            )

            await apply_test_configuration()
            selection.set_selected_prim_paths(["/World/Xform", "/World/Sphere"], True)
            await self.wait(10)

            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_mass_visualization_view_selection_changed",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            )

            await apply_test_configuration()
            settings.set(SETTING_MASS_DISTRIBUTION_MANIPULATOR, False)
            settings.set(SETTING_DISPLAY_MASS_PROPERTIES, SETTING_DISPLAY_MASS_PROPERTIES_NONE)
            await self.wait(10)

            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_mass_visualization_view_disable",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            )

            await self.wait(10)

        await do_test()
        await omni.usd.get_context().close_stage_async()
        await self.new_stage()

        settings.set(SETTING_DISPLAY_MASS_PROPERTIES, setting_initial_view)
        settings.set(SETTING_MASS_DISTRIBUTION_MANIPULATOR, setting_initial_edit)

        self.assertTrue(all_tests_passed)
