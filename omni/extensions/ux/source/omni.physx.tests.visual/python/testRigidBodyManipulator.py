# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.kit.viewport.utility.camera_state import ViewportCameraState
from omni.physxtestsvisual.utils import TestCase, settings
from omni.physxtests.utils.physicsBase import TestCategory
from omni.physx.scripts import physicsUtils
import omni.usd
import omni.physx
from pxr import Gf
import omni.ui as ui
import omni.kit.ui_test as ui_test
from carb.input import MouseEventType
import omni.appwindow
import carb


SETTINGS_TRANSFORM_OP = "/app/transform/operation"
class PhysxRigidBodyManipulatorTest(TestCase):
    category = TestCategory.Core

    def __init__(self, tests=()):
        super().__init__(tests)
        self._viewport_settings["/app/window/hideUi"] = (True, False)
        del self._viewport_settings[SETTINGS_TRANSFORM_OP]

    async def setUp(self):
        await super().setUp()

    async def tearDown(self):
        await super().tearDown()

    async def test_physics_ui_rigid_body_manipulation(self):

        all_tests_passed = True
        app_window = omni.appwindow.get_default_app_window()
        app_window.set_input_blocking_state(carb.input.DeviceType.MOUSE, False)

        async def do_test():
            nonlocal all_tests_passed
            stage = await self.new_stage()
            await self.wait(10)
            camera_state = ViewportCameraState()
            camera_state.set_position_world(Gf.Vec3d(500.0, 50.0, 0.0), True)
            camera_state.set_target_world(Gf.Vec3d(0.0, 50.0, 0.0), True)
            
            physicsUtils.add_quad_plane(stage, "/World/groundPlane", "Y", 2000.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))
            physicsUtils.add_rigid_box(stage, "/World/CubeA", 100.0, Gf.Vec3f(0.0, 50.0, 120.0))
            physicsUtils.add_rigid_box(stage, "/World/CubeB", 100.0, Gf.Vec3f(0.0, 50.0, -120.0))
            self._selection = omni.usd.get_context().get_selection()
            self._selection.set_selected_prim_paths(["/World/CubeA"], True)
            await self.wait(10)
            window = ui.Workspace.get_window("Viewport")
            test_window_width = 1000
            test_window_height = 800

            transform_op_initial = settings.get_as_string(SETTINGS_TRANSFORM_OP)

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
            await self.wait(1)

            settings.set_string(SETTINGS_TRANSFORM_OP, "move")
            omni.timeline.get_timeline_interface().play()

            begin_pos = ui_test.Vec2(test_window_width * 0.5 - 20, test_window_height - 105) / 2
            end_pos = ui_test.Vec2(test_window_width * 1.5 - 20, test_window_height - 105) / 2
            await ui_test.input.emulate_mouse(MouseEventType.MOVE, begin_pos)
            await self.wait(10)
            await ui_test.input.emulate_mouse(MouseEventType.LEFT_BUTTON_DOWN)
            await ui_test.input.emulate_mouse_slow_move(begin_pos, end_pos, 40, 1)
            await self.wait(30)
            await ui_test.input.emulate_mouse(MouseEventType.LEFT_BUTTON_UP)
            await self.wait(10)
            omni.timeline.get_timeline_interface().pause()

            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_rigid_body_manipulation_move",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            )

            await self.wait(1)

            await apply_test_configuration()

            # Make sure that the boxes return to their original locations upon simulation end.
            omni.timeline.get_timeline_interface().stop()

            await self.wait(1)

            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_rigid_body_manipulation_reset",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            )

            await self.wait(1)

            await apply_test_configuration()

            settings.set_string(SETTINGS_TRANSFORM_OP, "rotate")

            self._selection.set_selected_prim_paths(["/World/CubeB", "/World/CubeA"], True)

            omni.timeline.get_timeline_interface().play()

            begin_pos = ui_test.Vec2(test_window_width * 0.5 - 25, test_window_height - 10 ) / 2
            end_pos = ui_test.Vec2(test_window_width * 0.5 + 200, test_window_height - 10) / 2
            await ui_test.input.emulate_mouse(MouseEventType.MOVE, begin_pos)
            await self.wait(10)
            await ui_test.input.emulate_mouse(MouseEventType.LEFT_BUTTON_DOWN)
            await ui_test.input.emulate_mouse_slow_move(begin_pos, end_pos, 40, 1)
            await self.wait(30)
            await ui_test.input.emulate_mouse(MouseEventType.LEFT_BUTTON_UP)
            await self.wait(10)

            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_rigid_body_manipulation_rotate",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            )

            omni.timeline.get_timeline_interface().stop()

            settings.set_string(SETTINGS_TRANSFORM_OP, transform_op_initial)

        await do_test()
        await omni.usd.get_context().close_stage_async()
        await self.new_stage()
        
        self.assertTrue(all_tests_passed)
