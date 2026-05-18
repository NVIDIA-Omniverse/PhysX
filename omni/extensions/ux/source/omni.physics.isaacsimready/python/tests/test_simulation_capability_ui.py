# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from pathlib import Path
import omni.kit.viewport.utility as viewport_utility
from omni.ui.tests.test_base import OmniUiTest
from omni.kit.ui_test.input import emulate_mouse_move_and_click
from omni.kit.ui_test.vec2 import Vec2
from omni.kit.test.async_unittest import AsyncTestCase
import carb.settings


TEST_DATA_PATH = Path(__file__).parents[4].joinpath("data").joinpath("tests").joinpath("golden_img")
TEST_WIDTH = 800
TEST_HEIGHT = 600


class SimulationCapabilityTests(OmniUiTest):
    async def setUp(self):
        await self.create_test_area(width=TEST_WIDTH, height=TEST_HEIGHT, block_devices=False)

        _, viewport_window = viewport_utility.get_active_viewport_and_window()
        await self.docked_test_window(viewport_window, width=TEST_WIDTH, height=TEST_HEIGHT, block_devices=False)
        pass

    async def tearDown(self):
        pass

    async def test_capability(self):
        pass
