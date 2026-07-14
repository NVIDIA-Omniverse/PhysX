# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#

import _carb_setup  # noqa: F401 - initializes Carbonite framework

import unittest
from _physics import Simulation, acquire_physics_interface, k_invalid_simulation_id, SimulationRegistryEventType

from expectedError import ExpectedError


class TestSimulationRegistry(unittest.TestCase):
    def setUp(self):
        self.physics = acquire_physics_interface()
        self.assertIsNotNone(self.physics)

    def test_create_new_simulation(self):
        """Test creating a new simulation"""
        simulation = Simulation()
        simulation_name = "TestSimulation"

        simulation_id = self.physics.register_simulation(simulation, simulation_name)
        self.assertNotEqual(simulation_id, k_invalid_simulation_id)

        retrieved_name = self.physics.get_simulation_name(simulation_id)
        self.assertEqual(retrieved_name, simulation_name)

        self.physics.unregister_simulation(simulation_id)

    def test_invalid_simulation_operations(self):
        """Test operations with invalid simulation IDs"""
        with ExpectedError():
            invalid_sim = self.physics.get_simulation(k_invalid_simulation_id)
            self.assertIsNone(invalid_sim)

        with ExpectedError():
            invalid_name = self.physics.get_simulation_name(k_invalid_simulation_id)
            self.assertEqual(invalid_name, "")

        with ExpectedError():
            self.assertFalse(self.physics.is_simulation_active(k_invalid_simulation_id))

        with ExpectedError():
            self.physics.unregister_simulation(k_invalid_simulation_id)

        with ExpectedError():
            self.physics.activate_simulation(k_invalid_simulation_id)

        with ExpectedError():
            self.physics.deactivate_simulation(k_invalid_simulation_id)

    def test_register_and_unregister_simulation(self):
        """Test registering and unregistering a simulation"""
        simulation = Simulation()
        simulation_name = "TestSimulation"

        simulation_id = self.physics.register_simulation(simulation, simulation_name)
        self.assertNotEqual(simulation_id, k_invalid_simulation_id)

        retrieved_sim = self.physics.get_simulation(simulation_id)
        self.assertTrue(retrieved_sim is not None)

        retrieved_name = self.physics.get_simulation_name(simulation_id)
        self.assertEqual(retrieved_name, simulation_name)

        self.physics.unregister_simulation(simulation_id)
        with ExpectedError():
            retrieved_sim = self.physics.get_simulation(simulation_id)
            self.assertIsNone(retrieved_sim)

    def test_multiple_simulations_management(self):
        """Test managing multiple simulations"""
        sim1 = Simulation()
        sim2 = Simulation()
        sim3 = Simulation()

        id1 = self.physics.register_simulation(sim1, "Simulation1")
        id2 = self.physics.register_simulation(sim2, "Simulation2")
        id3 = self.physics.register_simulation(sim3, "Simulation3")

        self.assertNotEqual(id1, k_invalid_simulation_id)
        self.assertNotEqual(id2, k_invalid_simulation_id)
        self.assertNotEqual(id3, k_invalid_simulation_id)

        num_sims = self.physics.get_num_simulations()
        self.assertEqual(num_sims, 3)

        sim_ids = self.physics.get_simulation_ids()
        self.assertEqual(len(sim_ids), num_sims)

        found_id1 = found_id2 = found_id3 = False
        for sim_id in sim_ids:
            if sim_id == id1:
                found_id1 = True
                self.assertEqual(self.physics.get_simulation_name(sim_id), "Simulation1")
            if sim_id == id2:
                found_id2 = True
                self.assertEqual(self.physics.get_simulation_name(sim_id), "Simulation2")
            if sim_id == id3:
                found_id3 = True
                self.assertEqual(self.physics.get_simulation_name(sim_id), "Simulation3")

        self.assertTrue(found_id1)
        self.assertTrue(found_id2)
        self.assertTrue(found_id3)

        self.physics.unregister_simulation(id1)
        self.physics.unregister_simulation(id2)
        self.physics.unregister_simulation(id3)

    def test_simulation_activation(self):
        """Test simulation activation and deactivation"""
        simulation = Simulation()
        simulation_id = self.physics.register_simulation(simulation, "TestSimulation")
        self.assertNotEqual(simulation_id, k_invalid_simulation_id)

        self.assertTrue(self.physics.is_simulation_active(simulation_id))

        self.physics.deactivate_simulation(simulation_id)
        self.assertFalse(self.physics.is_simulation_active(simulation_id))

        self.physics.activate_simulation(simulation_id)
        self.assertTrue(self.physics.is_simulation_active(simulation_id))

        self.physics.unregister_simulation(simulation_id)

    def _on_simulation_registry_event(self, event_type, simulation_id, simulation_name):
        self.event_type = event_type
        self.simulation_id = simulation_id
        self.simulation_name = simulation_name

    def test_simulation_registry_events(self):
        """Test simulation registry events"""
        simulation = Simulation()
        simulation_name = "TestSimulation"

        subscription = self.physics.subscribe_simulation_registry_events(self._on_simulation_registry_event)

        simulation_id = self.physics.register_simulation(simulation, simulation_name)
        self.assertNotEqual(simulation_id, k_invalid_simulation_id)
        self.assertEqual(self.simulation_id, simulation_id)
        self.assertEqual(self.simulation_name, simulation_name)
        self.assertEqual(self.event_type, SimulationRegistryEventType.SIMULATION_REGISTERED)

        self.physics.deactivate_simulation(simulation_id)
        self.assertEqual(self.event_type, SimulationRegistryEventType.SIMULATION_DEACTIVATED)

        self.physics.activate_simulation(simulation_id)
        self.assertEqual(self.event_type, SimulationRegistryEventType.SIMULATION_ACTIVATED)

        self.physics.unregister_simulation(simulation_id)
        self.assertEqual(self.event_type, SimulationRegistryEventType.SIMULATION_UNREGISTERED)

        # Unsubscribe and verify no more events
        subscription = None
        self.simulation_id = k_invalid_simulation_id
        self.simulation_name = ""
        self.event_type = None

        simulation_id = self.physics.register_simulation(simulation, simulation_name)
        self.assertNotEqual(simulation_id, k_invalid_simulation_id)
        self.assertEqual(self.simulation_id, k_invalid_simulation_id)
        self.assertEqual(self.simulation_name, "")
        self.assertEqual(self.event_type, None)

        self.physics.unregister_simulation(simulation_id)


if __name__ == "__main__":
    unittest.main()
