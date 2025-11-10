# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.kit.viewport.utility.camera_state import ViewportCameraState
from omni.physxtestsvisual.utils import TestCase
from omni.physxtests.utils.physicsBase import TestCategory, PhysicsBaseAsyncTestCase
from omni.physx.scripts import physicsUtils
import omni.usd
import omni.physx
from pxr import Gf, UsdGeom
import omni.ui as ui
import unittest
import omni.kit.ui_test as ui_test
from carb.input import KeyboardInput, KeyboardEventType, MouseEventType
from omni.physx import get_physx_interface

class PhysxMouseForceInteractionTest(TestCase):
    category = TestCategory.Core

    def __init__(self, tests=()):
        super().__init__(tests)
        self._viewport_settings["/app/window/hideUi"] = (True, False)

    async def setUp(self):
        await super().setUp()

    async def tearDown(self):
        await super().tearDown()

    async def test_physics_ui_mouse_force_interaction(self):

        all_tests_passed = True

        async def do_test():
            stage = await self.new_stage()
            await self.wait(10)
            camera_state = ViewportCameraState()
            camera_state.set_position_world(Gf.Vec3d(500.0, 50.0, 0.0), True)
            camera_state.set_target_world(Gf.Vec3d(0.0, 50.0, 0.0), True)
            physicsUtils.add_quad_plane(stage, "/World/groundPlane", "Y", 2000.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))
            cube = physicsUtils.add_rigid_box(stage, "/World/Cube", 100.0, Gf.Vec3f(0.0, 50.0, 0.0))
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

            # Test basic grabbing.
            await apply_test_configuration()

            await self.wait(1)

            omni.timeline.get_timeline_interface().play()

            await self.wait(1)

            await ui_test.input.emulate_keyboard(KeyboardEventType.KEY_PRESS, KeyboardInput.LEFT_SHIFT)

            await self.wait(1)

            center = ui_test.Vec2(test_window_width, test_window_height) / 2
            end_pos = ui_test.Vec2(test_window_width * 1.5, test_window_height) / 2

            await ui_test.input.emulate_mouse(MouseEventType.MOVE, center)
            await ui_test.input.emulate_mouse(MouseEventType.LEFT_BUTTON_DOWN)
            await ui_test.input.emulate_mouse_slow_move(center, end_pos, 40, 1)
            await ui_test.input.emulate_mouse(MouseEventType.LEFT_BUTTON_UP)
            # Move back to verify that we've properly released the grab.
            await ui_test.input.emulate_mouse_slow_move(end_pos, center, 40, 1)
            await ui_test.input.emulate_keyboard(KeyboardEventType.KEY_RELEASE, KeyboardInput.LEFT_SHIFT)
            await self.wait(30)

            omni.timeline.get_timeline_interface().pause()

            nonlocal all_tests_passed
            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_mouse_force_interaction_grab",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            )

            omni.timeline.get_timeline_interface().stop()

            await apply_test_configuration()

            await self.wait(1)

            omni.timeline.get_timeline_interface().play()

            await self.wait(1)

            await ui_test.input.emulate_keyboard(KeyboardEventType.KEY_PRESS, KeyboardInput.LEFT_SHIFT)

            await self.wait(1)

            center = ui_test.Vec2(test_window_width, test_window_height) / 2

            await ui_test.input.emulate_mouse(MouseEventType.MOVE, center)
            await ui_test.input.emulate_mouse(MouseEventType.LEFT_BUTTON_DOWN)
            await self.wait(1)
            await ui_test.input.emulate_mouse(MouseEventType.LEFT_BUTTON_UP)
            await self.wait(1)
            await ui_test.input.emulate_mouse(MouseEventType.LEFT_BUTTON_DOWN)
            await self.wait(30)
            await ui_test.input.emulate_mouse(MouseEventType.LEFT_BUTTON_UP)
            await ui_test.input.emulate_keyboard(KeyboardEventType.KEY_RELEASE, KeyboardInput.LEFT_SHIFT)
            await self.wait(30)

            omni.timeline.get_timeline_interface().pause()

            all_tests_passed = all_tests_passed and await self.do_visual_test(
                img_name="",
                img_suffix="test_physics_visual_mouse_force_interaction_push",
                use_distant_light=True,
                skip_assert=True,
                threshold=0.0025,
                use_renderer_capture=True
            )

            omni.timeline.get_timeline_interface().stop()

            await self.wait(10)

        await do_test()
        await omni.usd.get_context().close_stage_async()
        await self.new_stage()
        
        self.assertTrue(all_tests_passed)
