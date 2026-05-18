# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from pathlib import Path
import omni.kit.viewport.utility as viewport_utility
from omni.ui.tests.test_base import OmniUiTest
from omni.kit.ui_test.input import emulate_mouse_move_and_click
from omni.kit.ui_test.vec2 import Vec2
from omni.physics.core import Simulation, get_physics_interface, k_invalid_simulation_id

TEST_DATA_PATH = Path(__file__).parents[4].joinpath("data").joinpath("tests").joinpath("golden_img")
TEST_WIDTH = 800
TEST_HEIGHT = 600

class SimulationConfigTests(OmniUiTest):
    async def setUp(self):
        await self.create_test_area(width=TEST_WIDTH, height=TEST_HEIGHT, block_devices=False)

        _, viewport_window = viewport_utility.get_active_viewport_and_window()
        await self.docked_test_window(viewport_window, width=TEST_WIDTH, height=TEST_HEIGHT, block_devices=False)

        self.simulation_id = k_invalid_simulation_id
        self.simulation_id2 = k_invalid_simulation_id

        pass

    async def tearDown(self):
        if self.simulation_id != k_invalid_simulation_id:
            get_physics_interface().unregister_simulation(self.simulation_id)
        if self.simulation_id2 != k_invalid_simulation_id:
            get_physics_interface().unregister_simulation(self.simulation_id2)

    async def test_simulator_toggle(self):
        physics_interface = get_physics_interface()

        simulation = Simulation()

        self.simulation_id = physics_interface.register_simulation(simulation, "TestSimulation")
        self.simulation_id2 = physics_interface.register_simulation(simulation, "TestSimulation2")

        self.assertNotEqual(self.simulation_id, k_invalid_simulation_id)
        self.assertNotEqual(self.simulation_id2, k_invalid_simulation_id)

        await self.wait_n_updates(3)

        async def run_sub_test(img_name: str):
            await self.finalize_test(
                golden_img_dir=TEST_DATA_PATH, hide_menu_bar=False, golden_img_name=f"simulator_toggle_{img_name}.png")

        await emulate_mouse_move_and_click(Vec2(10, 10))
        await run_sub_test("register")

        await emulate_mouse_move_and_click(Vec2(40, 60))
        await run_sub_test("deactivate")

        await emulate_mouse_move_and_click(Vec2(40, 60))
        await run_sub_test("activate")

        physics_interface.unregister_simulation(self.simulation_id)
        self.simulation_id = k_invalid_simulation_id

        await self.wait_n_updates(3)
        await run_sub_test("unregister")
