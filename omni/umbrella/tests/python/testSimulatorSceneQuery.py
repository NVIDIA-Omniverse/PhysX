# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#

import _carb_setup  # noqa: F401 - initializes Carbonite framework

import carb
import unittest
from _physics import (
    OverlapHit,
    RaycastHit,
    Simulation,
    SweepHit,
    acquire_physics_interface,
    acquire_physics_scene_query_interface,
    k_invalid_simulation_id,
)


class MockSceneQuery:
    def __init__(self):
        self.raycast_count = 0
        self.sweep_count = 0
        self.overlap_count = 0
        self.report_fn_calls = 0

    def raycast_closest(self, origin, unit_dir, distance, hit, both_sides):
        self.raycast_count += 1
        return True

    def raycast_any(self, origin, unit_dir, distance, both_sides):
        self.raycast_count += 1
        return True

    def raycast_all(self, origin, unit_dir, distance, report_fn, both_sides):
        self.raycast_count += 1
        hit = RaycastHit()
        hit.position = carb.Float3(1.0, 2.0, 3.0)
        hit.normal = carb.Float3(0.0, 1.0, 0.0)
        hit.distance = 5.0
        if report_fn:
            self.report_fn_calls += 1
            report_fn(hit)
        return True

    def sweep_sphere_closest(self, radius, origin, unit_dir, distance, hit, both_sides):
        self.sweep_count += 1
        return True

    def sweep_sphere_any(self, radius, origin, unit_dir, distance, both_sides):
        self.sweep_count += 1
        return True

    def sweep_sphere_all(self, radius, origin, unit_dir, distance, report_fn, both_sides):
        self.sweep_count += 1
        hit = SweepHit()
        hit.position = carb.Float3(1.0, 2.0, 3.0)
        hit.normal = carb.Float3(0.0, 1.0, 0.0)
        hit.distance = 5.0
        if report_fn:
            self.report_fn_calls += 1
            report_fn(hit)
        return True

    def overlap_sphere(self, radius, pos, report_fn):
        self.overlap_count += 1
        hit = OverlapHit()
        if report_fn:
            self.report_fn_calls += 1
            report_fn(hit)
        return 1

    def overlap_sphere_any(self, radius, pos):
        self.overlap_count += 1
        return True


class TestSimulatorSceneQuery(unittest.TestCase):
    def setUp(self):
        self.physics = acquire_physics_interface()
        self.physics_scene_query = acquire_physics_scene_query_interface()
        self.mock = MockSceneQuery()

        self.simulation = Simulation()
        self.simulation.scene_query_fns.raycast_closest = self.mock.raycast_closest
        self.simulation.scene_query_fns.raycast_any = self.mock.raycast_any
        self.simulation.scene_query_fns.raycast_all = self.mock.raycast_all
        self.simulation.scene_query_fns.sweep_sphere_closest = self.mock.sweep_sphere_closest
        self.simulation.scene_query_fns.sweep_sphere_any = self.mock.sweep_sphere_any
        self.simulation.scene_query_fns.sweep_sphere_all = self.mock.sweep_sphere_all
        self.simulation.scene_query_fns.overlap_sphere = self.mock.overlap_sphere
        self.simulation.scene_query_fns.overlap_sphere_any = self.mock.overlap_sphere_any

        self.simulation_id = self.physics.register_simulation(self.simulation, "MockSceneQuerySim")
        self.assertNotEqual(self.simulation_id, k_invalid_simulation_id)

    def tearDown(self):
        if self.simulation_id != k_invalid_simulation_id:
            self.physics.unregister_simulation(self.simulation_id)

    def test_raycast_queries(self):
        origin = carb.Float3(1.0, 2.0, 3.0)
        direction = carb.Float3(0.0, 1.0, 0.0)

        result, hit = self.physics_scene_query.raycast_closest(origin, direction, 10.0, True)
        self.assertTrue(result)
        self.assertEqual(self.mock.raycast_count, 1)

        result = self.physics_scene_query.raycast_any(origin, direction, 10.0, True)
        self.assertTrue(result)
        self.assertEqual(self.mock.raycast_count, 2)

        hit_received = False

        def report_fn(hit):
            nonlocal hit_received
            hit_received = True
            self.assertIsInstance(hit, RaycastHit)
            self.assertEqual(hit.distance, 5.0)
            return True

        self.physics_scene_query.raycast_all(origin, direction, 10.0, report_fn, True)
        self.assertEqual(self.mock.raycast_count, 3)
        self.assertTrue(hit_received)

    def test_sweep_queries(self):
        origin = carb.Float3(1.0, 2.0, 3.0)
        direction = carb.Float3(0.0, 1.0, 0.0)

        result, hit = self.physics_scene_query.sweep_sphere_closest(1.0, origin, direction, 10.0, True)
        self.assertTrue(result)
        self.assertEqual(self.mock.sweep_count, 1)

        result = self.physics_scene_query.sweep_sphere_any(1.0, origin, direction, 10.0, True)
        self.assertTrue(result)
        self.assertEqual(self.mock.sweep_count, 2)

        hit_received = False

        def report_fn(hit):
            nonlocal hit_received
            hit_received = True
            self.assertIsInstance(hit, SweepHit)
            return True

        self.physics_scene_query.sweep_sphere_all(1.0, origin, direction, 10.0, report_fn, True)
        self.assertTrue(hit_received)

    def test_overlap_queries(self):
        position = carb.Float3(1.0, 2.0, 3.0)
        hit_received = False

        def report_fn(hit):
            nonlocal hit_received
            hit_received = True
            self.assertIsInstance(hit, OverlapHit)
            return True

        result = self.physics_scene_query.overlap_sphere(1.0, position, report_fn)
        self.assertEqual(result, 1)
        self.assertTrue(hit_received)

        result = self.physics_scene_query.overlap_sphere_any(1.0, position)
        self.assertTrue(result)


if __name__ == "__main__":
    unittest.main()
