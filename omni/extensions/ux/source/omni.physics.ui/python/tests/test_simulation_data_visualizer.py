# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from pathlib import Path
import math
from typing import Any
import carb
import carb.settings
import omni.usd
import omni.timeline
import omni.kit.ui_test as ui_test
import omni.ui as ui
from omni.ui.tests.test_base import OmniUiTest
from omni.kit.ui_test.vec2 import Vec2
from carb.input import MouseEventType
from pxr import Gf, UsdGeom

from ..scripts.simulationDataVisualizer import (
    SimulationDataVisualizerWindowManager,
    SimulationDataVisualizerWindow,
    SETTINGS_UI_WINDOW_OPACITY,
    SETTINGS_UI_PHYSICS_SIMULATION_DATA_VISUALIZER_ENABLED
)
from omni.physics.core import (
    Simulation,
    get_physics_simulation_interface,
    get_physics_interaction_interface,
    get_physics_interface,
    DebugDataItemType,
    k_invalid_simulation_id,
    PhysicsStepContext,
    SimulationId,
)

TEST_DATA_PATH = Path(__file__).parents[4].joinpath("data").joinpath("tests").joinpath("golden_img")
TEST_WIDTH = SimulationDataVisualizerWindow.WINDOW_WIDTH_MIN
TEST_HEIGHT = 800

class MockInteraction:
    """Mock implementation of physics interaction functionality"""
    def __init__(self):
        self.simulation_id = 0
        self.time_steps_per_second = 60
        self.simulation_timestamp = 0
        self.simulation_step_count = 0
        self.step_event_callbacks = []

        self.debug_data_entries = {
            "test float": {"type": DebugDataItemType.FLOAT, "value": 0.0},
            "test string": {"type": DebugDataItemType.STRING, "value": "test"},
            "test vector3": {"type": DebugDataItemType.VECTOR, "value": (0.0, 0.0, 0.0)},
            "test point3": {"type": DebugDataItemType.POINT, "value": (0.0, 0.0, 0.0)},
            "test quaternion": {"type": DebugDataItemType.QUATERNION, "value": (0.0, 0.0, 0.0, 1.0)},
        }
        self.increment = 0.0

    def simulate(self, elapsed_time, current_time):
        self.simulation_timestamp += 1
        self.simulation_step_count += 1

        # Call step event callbacks
        context = PhysicsStepContext()
        context.scene_path = 0
        context.simulation_id = SimulationId(self.simulation_id)

        for callback in self.step_event_callbacks:
            callback(elapsed_time, context)

    def subscribe_physics_on_step_events(self, pre_step, order, on_update):
        self.step_event_callbacks.append(on_update)
        return len(self.step_event_callbacks)

    def unsubscribe_physics_on_step_events(self, subscription_id):
        if subscription_id > 0 and subscription_id <= len(self.step_event_callbacks):
            self.step_event_callbacks.pop(subscription_id - 1)

    def get_simulation_time_steps_per_second(self, stage_id, scene_path):
        return self.time_steps_per_second

    def get_simulation_timestamp(self):
        return self.simulation_timestamp

    def get_simulation_step_count(self):
        return self.simulation_step_count

    def get_prim_debug_data(self, prim_path : str) -> dict[str, dict[str, Any]]:
        self.debug_data_entries["test float"]["value"] = self.increment
        self.debug_data_entries["test vector3"]["value"] = (self.increment, self.increment, self.increment)
        self.debug_data_entries["test point3"]["value"] = (0.0, 0.0, self.increment)
        angle = math.fmod(self.increment * math.pi * 2.0, 2.0 * math.pi)
        self.debug_data_entries["test quaternion"]["value"] = (math.sin(angle / 2.0), 0.0, 0.0, math.cos(angle / 2.0))
        self.increment += 0.01
        return self.debug_data_entries

class SimulationDataVisualizerTests(OmniUiTest):
    async def setUp(self):
        await super().setUp()
        # Store initial settings
        settings = carb.settings.get_settings()
        self._initial_opacity = settings.get_as_float(SETTINGS_UI_WINDOW_OPACITY)
        self._initial_visualizer_enabled = settings.get_as_bool(SETTINGS_UI_PHYSICS_SIMULATION_DATA_VISUALIZER_ENABLED)

        # Configure settings for the test
        settings.set(SETTINGS_UI_WINDOW_OPACITY, 1.0)

        # Get the physics interface
        self.physics = get_physics_interface()
        self.assertIsNotNone(self.physics)

        # Get the physics interaction interface
        self.physics_interaction = get_physics_interaction_interface()
        self.assertIsNotNone(self.physics_interaction)

        # Create mock implementation
        self.mock_interaction = MockInteraction()

        # Create simulation with mock interaction functions
        self.simulation = Simulation()
        self.simulation.simulation_fns.simulate = self.mock_interaction.simulate
        self.simulation.simulation_fns.subscribe_physics_on_step_events = self.mock_interaction.subscribe_physics_on_step_events
        self.simulation.simulation_fns.unsubscribe_physics_on_step_events = self.mock_interaction.unsubscribe_physics_on_step_events
        self.simulation.simulation_fns.get_simulation_time_steps_per_second = self.mock_interaction.get_simulation_time_steps_per_second
        self.simulation.simulation_fns.get_simulation_timestamp = self.mock_interaction.get_simulation_timestamp
        self.simulation.simulation_fns.get_simulation_step_count = self.mock_interaction.get_simulation_step_count
        self.simulation.interaction_fns.get_prim_debug_data = self.mock_interaction.get_prim_debug_data

        # Register simulation with physics interface
        self.simulation_id = self.physics.register_simulation(self.simulation, "MockInteractionSim")
        self.assertNotEqual(self.simulation_id, k_invalid_simulation_id)

        # Create a new stage
        usd_context = omni.usd.get_context()
        await usd_context.new_stage_async()  # type: ignore
        self._stage = usd_context.get_stage()  # type: ignore

    async def tearDown(self):
        # Unregister simulation
        if hasattr(self, 'simulation_id') and self.simulation_id != k_invalid_simulation_id:
            self.physics.unregister_simulation(self.simulation_id)
            self.simulation_id = None
            self.simulation = None

        omni.usd.get_context().close_stage()

        # Restore all settings
        settings = carb.settings.get_settings()
        settings.set_bool(SETTINGS_UI_PHYSICS_SIMULATION_DATA_VISUALIZER_ENABLED, self._initial_visualizer_enabled)
        settings.set(SETTINGS_UI_WINDOW_OPACITY, self._initial_opacity)

        self._stage = None
        await super().tearDown()

    async def test_simulation_data_visualizer(
        self):
        
        settings = carb.settings.get_settings()
        settings.set_bool(SETTINGS_UI_PHYSICS_SIMULATION_DATA_VISUALIZER_ENABLED, False)

        # Create a basic xform prim
        xform_path = "/World/TestXform"
        xform = UsdGeom.Xform.Define(self._stage, xform_path)
        xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
        
        await self.wait_n_updates(10)


        SimulationDataVisualizerWindow.SharedStates.plots_enabled.add("test float")
        SimulationDataVisualizerWindow.SharedStates.plots_enabled.add("test vector3")
        SimulationDataVisualizerWindow.SharedStates.plots_enabled.add("test point3")
        SimulationDataVisualizerWindow.SharedStates.plots_enabled.add("test quaternion")

        settings.set_bool(SETTINGS_UI_PHYSICS_SIMULATION_DATA_VISUALIZER_ENABLED, True)

        await self.wait_n_updates(10)

        window = ui.Workspace.get_window(SimulationDataVisualizerWindow.WINDOW_TITLE)
        if window is None or not window.visible or not isinstance(window, SimulationDataVisualizerWindow):
            self.fail("Could not retrieve Simulation Data Visualizer window.")

        await self.docked_test_window(window, width=TEST_WIDTH, height=TEST_HEIGHT, block_devices=False)  # type: ignore

        # Setup the window to prevent mouse tooltip appearing
        await self.wait_n_updates(10)
        await ui_test.input.emulate_mouse(MouseEventType.MOVE, Vec2(0, 0))
        await self.wait_n_updates(10)
        # Select the xform prim
        omni.usd.get_context().get_selection().set_selected_prim_paths([xform_path], False)
        
        # Start timeline
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        
        # Wait for simulation steps
        for step in range(180):
            get_physics_simulation_interface().simulate(1.0 / 60.0, step * 1.0 / 60.0)
            get_physics_simulation_interface().fetch_results()
            await self.wait_n_updates(1)

        timeline.pause()
        # Capture the visual result
        await self.finalize_test(
            golden_img_dir=TEST_DATA_PATH,
            hide_menu_bar=False,
            golden_img_name="simulation_data_visualizer.png"
        )

        timeline.stop()
