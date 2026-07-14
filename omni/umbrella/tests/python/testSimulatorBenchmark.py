# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#

import _carb_setup  # noqa: F401 - initializes Carbonite framework

import unittest
from _physics import (
    PhysicsProfileStats,
    Simulation,
    acquire_physics_interface,
    acquire_physics_benchmarks_interface,
    k_invalid_simulation_id,
    k_invalid_subscription_id,
)


class MockBenchmark:
    def __init__(self):
        self.subscription_count = 0
        self.last_subscription_id = k_invalid_subscription_id
        self.profile_stats_callback = None

    def subscribe_profile_stats_events(self, on_event):
        self.profile_stats_callback = on_event
        self.subscription_count += 1
        self.last_subscription_id = self.subscription_count
        return self.last_subscription_id

    def unsubscribe_profile_stats_events(self, subscription_id):
        if subscription_id == self.last_subscription_id:
            self.profile_stats_callback = None
            self.last_subscription_id = k_invalid_subscription_id

    def simulate_profile_stats(self):
        if self.profile_stats_callback:
            stats = [PhysicsProfileStats(), PhysicsProfileStats(), PhysicsProfileStats()]
            stats[0].zone_name = "Simulation"
            stats[0].ms = 16.6
            stats[1].zone_name = "Collision Detection"
            stats[1].ms = 5.2
            stats[2].zone_name = "Integration"
            stats[2].ms = 2.1
            self.profile_stats_callback(stats)

    def has_active_subscription(self):
        return self.profile_stats_callback is not None


class TestSimulatorBenchmark(unittest.TestCase):
    def setUp(self):
        self.physics = acquire_physics_interface()
        self.physics_benchmarks = acquire_physics_benchmarks_interface()
        self.mock = MockBenchmark()

        self.simulation = Simulation()
        self.simulation.benchmark_fns.subscribe_profile_stats_events = self.mock.subscribe_profile_stats_events
        self.simulation.benchmark_fns.unsubscribe_profile_stats_events = self.mock.unsubscribe_profile_stats_events

        self.simulation_id = self.physics.register_simulation(self.simulation, "MockBenchmarkSim")
        self.assertNotEqual(self.simulation_id, k_invalid_simulation_id)

    def tearDown(self):
        if self.simulation_id != k_invalid_simulation_id:
            self.physics.unregister_simulation(self.simulation_id)

    def test_profile_stats_subscription(self):
        callback_called = False
        received_stats = []

        def test_callback(stats):
            nonlocal callback_called, received_stats
            callback_called = True
            received_stats = stats

        sub = self.physics_benchmarks.subscribe_profile_stats_events(test_callback)
        self.assertNotEqual(sub, k_invalid_subscription_id)
        self.assertTrue(self.mock.has_active_subscription())

        self.mock.simulate_profile_stats()

        self.assertTrue(callback_called)
        self.assertEqual(len(received_stats), 3)
        self.assertEqual(received_stats[0].zone_name, "Simulation")
        self.assertAlmostEqual(received_stats[0].ms, 16.6, delta=1e-5)

        sub = None
        self.assertFalse(self.mock.has_active_subscription())

    def test_no_callback_after_unsubscribe(self):
        callback_called = False

        def test_callback(stats):
            nonlocal callback_called
            callback_called = True

        sub = self.physics_benchmarks.subscribe_profile_stats_events(test_callback)
        sub = None
        self.assertFalse(self.mock.has_active_subscription())

        self.mock.simulate_profile_stats()
        self.assertFalse(callback_called)

    def test_callback_with_empty_stats(self):
        callback_called = False
        received_stats = []

        def test_callback(stats):
            nonlocal callback_called, received_stats
            callback_called = True
            received_stats = stats

        sub = self.physics_benchmarks.subscribe_profile_stats_events(test_callback)
        if self.mock.profile_stats_callback:
            self.mock.profile_stats_callback([])
        self.assertTrue(callback_called)
        self.assertEqual(len(received_stats), 0)
        sub = None


if __name__ == "__main__":
    unittest.main()
