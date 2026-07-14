# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#

import _carb_setup  # noqa: F401 - initializes Carbonite framework

import carb
from typing import Any
import unittest
from _physics import (
    Simulation,
    acquire_physics_interaction_interface,
    acquire_physics_interface,
    DebugDataItemType,
    k_invalid_simulation_id,
)


debug_data_entries = {
    "test float": {"type": DebugDataItemType.FLOAT, "value": 0.0},
    "test int": {"type": DebugDataItemType.INT, "value": 0},
    "test string": {"type": DebugDataItemType.STRING, "value": "test"},
    "test bool": {"type": DebugDataItemType.BOOL, "value": True},
    "test vector3": {"type": DebugDataItemType.VECTOR, "value": (1.0, 2.0, 3.0)},
    "test point3": {"type": DebugDataItemType.POINT, "value": (1.0, 2.0, 3.0)},
    "test quaternion": {"type": DebugDataItemType.QUATERNION, "value": (1.0, 2.0, 3.0, 4.0)},
}


class MockInteraction:
    def __init__(self):
        self.is_reset_on_stop_disabled = False
        self.raycast_count = 0
        self.last_raycast_input = False

    def disable_reset_on_stop(self, disable):
        self.is_reset_on_stop_disabled = disable

    def is_disabled_reset_on_stop(self):
        return self.is_reset_on_stop_disabled

    def handle_raycast(self, origin, direction, has_input):
        self.raycast_count += 1
        self.last_raycast_input = has_input
        return True

    def get_prim_debug_data(self, prim_path: str) -> dict[str, dict[str, Any]]:
        return debug_data_entries


class TestSimulateInteraction(unittest.TestCase):
    def setUp(self):
        self.physics = acquire_physics_interface()
        self.physics_interaction = acquire_physics_interaction_interface()
        self.mock = MockInteraction()

        self.simulation = Simulation()
        self.simulation.interaction_fns.disable_reset_on_stop = self.mock.disable_reset_on_stop
        self.simulation.interaction_fns.is_disabled_reset_on_stop = self.mock.is_disabled_reset_on_stop
        self.simulation.interaction_fns.handle_raycast = self.mock.handle_raycast
        self.simulation.interaction_fns.get_prim_debug_data = self.mock.get_prim_debug_data

        self.simulation_id = self.physics.register_simulation(self.simulation, "MockInteractionSim")
        self.assertNotEqual(self.simulation_id, k_invalid_simulation_id)

    def tearDown(self):
        if self.simulation_id != k_invalid_simulation_id:
            self.physics.unregister_simulation(self.simulation_id)

    def test_reset_on_stop_control(self):
        self.assertFalse(self.physics_interaction.is_disabled_reset_on_stop(self.simulation_id))

        self.physics_interaction.disable_reset_on_stop(True)
        self.assertTrue(self.mock.is_reset_on_stop_disabled)
        self.assertTrue(self.physics_interaction.is_disabled_reset_on_stop(self.simulation_id))

        self.physics_interaction.disable_reset_on_stop(False)
        self.assertFalse(self.mock.is_reset_on_stop_disabled)

    def test_raycast_handling(self):
        origin = carb.Float3(1.0, 2.0, 3.0)
        direction = carb.Float3(0.0, 1.0, 0.0)

        self.physics_interaction.handle_raycast(origin, direction, True)
        self.assertEqual(self.mock.raycast_count, 1)
        self.assertTrue(self.mock.last_raycast_input)

        self.physics_interaction.handle_raycast(origin, direction, False)
        self.assertEqual(self.mock.raycast_count, 2)
        self.assertFalse(self.mock.last_raycast_input)

    def test_get_prim_debug_data(self):
        debug_data = self.physics_interaction.get_prim_debug_data("/World/Cube")
        self.assertEqual(set(debug_data.keys()), set(debug_data_entries.keys()))
        for key, entry in debug_data.items():
            golden = debug_data_entries[key]
            for item_key, item_value in entry.items():
                self.assertEqual(item_value, golden[item_key],
                                 f"Mismatch for key {key}, item {item_key}")


if __name__ == "__main__":
    unittest.main()
