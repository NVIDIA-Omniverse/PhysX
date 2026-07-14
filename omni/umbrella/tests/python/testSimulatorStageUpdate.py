# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#

import _carb_setup  # noqa: F401 - initializes Carbonite framework

import carb
import unittest
from _physics import (
    Simulation,
    acquire_physics_interface,
    acquire_physics_stage_update_interface,
    k_invalid_simulation_id,
    SimulationEvent,
)


class MockPhysicsStageUpdate:
    class TimelineState:
        STOPPED = 0
        PAUSED = 1
        PLAYING = 2

    def __init__(self):
        self.attached_stage_id = None
        self.is_physics_enabled = False
        self.is_paused = False
        self.update_count = 0
        self.raycast_hits = []
        self.force_load_called = False
        self.release_objects_called = False
        self.is_physics_loaded = False
        self.timeline_state = self.TimelineState.STOPPED
        self.reset_simulation_called = False
        self.start_simulation_called = False

    def on_attach(self, stage_id):
        self.attached_stage_id = stage_id
        return True

    def on_detach(self):
        self.attached_stage_id = None
        self.is_physics_loaded = False
        self.timeline_state = self.TimelineState.STOPPED
        return True

    def on_update(self, time, delta_time, physics_enabled):
        self.is_physics_enabled = physics_enabled
        self.update_count += 1
        return True

    def on_resume(self, time):
        if self.attached_stage_id is not None:
            self.timeline_state = self.TimelineState.PLAYING
        self.is_paused = False
        return True

    def on_pause(self):
        if self.attached_stage_id is not None:
            self.timeline_state = self.TimelineState.PAUSED
        self.is_paused = True
        return True

    def on_reset(self):
        if self.attached_stage_id is not None:
            self.timeline_state = self.TimelineState.STOPPED
        self.update_count = 0
        self.is_paused = False
        return True

    def force_load_physics_from_usd(self):
        if self.attached_stage_id is not None:
            self.is_physics_loaded = True
        self.force_load_called = True
        return True

    def release_physics_objects(self):
        if self.attached_stage_id is not None:
            self.is_physics_loaded = False
        self.release_objects_called = True
        return True

    def reset_simulation(self):
        if self.attached_stage_id is not None:
            self.is_physics_loaded = False
            self.timeline_state = self.TimelineState.STOPPED
            self.reset_simulation_called = True
        return True

    def start_simulation(self):
        if self.attached_stage_id is not None:
            self.start_simulation_called = True
        return True

    def handle_raycast(self, origin, direction, has_input):
        if has_input:
            self.raycast_hits.append((origin, direction))
        return True


class TestSimulateStageUpdate(unittest.TestCase):
    def setUp(self):
        self.physics = acquire_physics_interface()
        self.physics_stage_update = acquire_physics_stage_update_interface()
        self.mock = MockPhysicsStageUpdate()

        self.simulation = Simulation()
        self.simulation.stage_update_fns.on_attach = self.mock.on_attach
        self.simulation.stage_update_fns.on_detach = self.mock.on_detach
        self.simulation.stage_update_fns.on_update = self.mock.on_update
        self.simulation.stage_update_fns.on_resume = self.mock.on_resume
        self.simulation.stage_update_fns.on_pause = self.mock.on_pause
        self.simulation.stage_update_fns.on_reset = self.mock.on_reset
        self.simulation.stage_update_fns.force_load_physics_from_usd = self.mock.force_load_physics_from_usd
        self.simulation.stage_update_fns.release_physics_objects = self.mock.release_physics_objects
        self.simulation.stage_update_fns.handle_raycast = self.mock.handle_raycast
        self.simulation.stage_update_fns.reset_simulation = self.mock.reset_simulation
        self.simulation.stage_update_fns.start_simulation = self.mock.start_simulation

        self.simulation_id = self.physics.register_simulation(self.simulation, "MockStageUpdateSim")
        self.assertNotEqual(self.simulation_id, k_invalid_simulation_id)

    def tearDown(self):
        if self.simulation_id != k_invalid_simulation_id:
            self.physics.unregister_simulation(self.simulation_id)

    def test_stage_attachment(self):
        self.physics_stage_update.on_attach(123)
        self.assertEqual(self.mock.attached_stage_id, 123)
        self.physics_stage_update.on_detach()
        self.assertIsNone(self.mock.attached_stage_id)

    def test_stage_update(self):
        self.physics_stage_update.on_attach(123)
        self.physics_stage_update.on_update(1.0, 0.016, True)
        self.assertTrue(self.mock.is_physics_enabled)
        self.assertEqual(self.mock.update_count, 1)
        self.physics_stage_update.on_update(1.0, 0.016, False)
        self.assertFalse(self.mock.is_physics_enabled)
        self.physics_stage_update.on_detach()

    def test_timeline_control(self):
        self.physics_stage_update.on_attach(123)
        self.physics_stage_update.on_resume(1.0)
        self.assertFalse(self.mock.is_paused)
        self.physics_stage_update.on_pause()
        self.assertTrue(self.mock.is_paused)
        self.physics_stage_update.on_reset()
        self.assertEqual(self.mock.update_count, 0)
        self.assertFalse(self.mock.is_paused)
        self.physics_stage_update.on_detach()

    def test_physics_loading(self):
        self.physics_stage_update.on_attach(123)
        self.physics_stage_update.force_load_physics_from_usd()
        self.assertTrue(self.mock.is_physics_loaded)
        self.physics_stage_update.release_physics_objects()
        self.assertFalse(self.mock.is_physics_loaded)
        self.physics_stage_update.on_detach()

    def test_raycast_handling(self):
        self.physics_stage_update.on_attach(123)
        origin = carb.Float3(0.0, 0.0, 0.0)
        direction = carb.Float3(0.0, 0.0, 1.0)
        self.physics_stage_update.handle_raycast(origin, direction, True)
        self.assertEqual(len(self.mock.raycast_hits), 1)
        self.physics_stage_update.handle_raycast(origin, direction, False)
        self.assertEqual(len(self.mock.raycast_hits), 1)
        self.physics_stage_update.on_detach()

    def test_reset_simulation(self):
        self.physics_stage_update.on_attach(123)
        self.physics_stage_update.force_load_physics_from_usd()
        self.physics_stage_update.on_resume(1.0)
        self.physics_stage_update.reset_simulation()
        self.assertTrue(self.mock.reset_simulation_called)
        self.assertFalse(self.mock.is_physics_loaded)
        self.assertEqual(self.mock.timeline_state, MockPhysicsStageUpdate.TimelineState.STOPPED)
        self.physics_stage_update.on_detach()

    def test_start_simulation(self):
        self.physics_stage_update.on_attach(123)
        self.physics_stage_update.start_simulation()
        self.assertTrue(self.mock.start_simulation_called)
        self.physics_stage_update.on_detach()

    def test_simulation_event_stream_exists(self):
        event_stream = self.physics_stage_update.get_simulation_event_stream()
        self.assertIsNotNone(event_stream)

    def test_simulation_event_resumed(self):
        event_stream = self.physics_stage_update.get_simulation_event_stream()
        received_events = []

        def on_event(event):
            received_events.append(event.type)

        subscription = event_stream.create_subscription_to_pop(on_event)
        self.physics_stage_update.on_attach(123)
        self.physics_stage_update.on_resume(1.0)
        event_stream.pump()

        self.assertEqual(len(received_events), 1)
        self.assertEqual(received_events[0], SimulationEvent.RESUMED)
        self.physics_stage_update.on_detach()

    def test_simulation_event_paused(self):
        event_stream = self.physics_stage_update.get_simulation_event_stream()
        received_events = []

        def on_event(event):
            received_events.append(event.type)

        subscription = event_stream.create_subscription_to_pop(on_event)
        self.physics_stage_update.on_attach(123)
        self.physics_stage_update.on_resume(1.0)
        event_stream.pump()
        received_events.clear()

        self.physics_stage_update.on_pause()
        event_stream.pump()
        self.assertEqual(len(received_events), 1)
        self.assertEqual(received_events[0], SimulationEvent.PAUSED)
        self.physics_stage_update.on_detach()

    def test_simulation_event_stopped(self):
        event_stream = self.physics_stage_update.get_simulation_event_stream()
        received_events = []

        def on_event(event):
            received_events.append(event.type)

        subscription = event_stream.create_subscription_to_pop(on_event)
        self.physics_stage_update.on_attach(123)
        self.physics_stage_update.on_resume(1.0)
        event_stream.pump()
        received_events.clear()

        self.physics_stage_update.on_reset()
        event_stream.pump()
        self.assertEqual(len(received_events), 1)
        self.assertEqual(received_events[0], SimulationEvent.STOPPED)
        self.physics_stage_update.on_detach()

    def test_simulation_event_full_lifecycle(self):
        event_stream = self.physics_stage_update.get_simulation_event_stream()
        received_events = []

        def on_event(event):
            received_events.append(event.type)

        subscription = event_stream.create_subscription_to_pop(on_event)
        self.physics_stage_update.on_attach(123)

        self.physics_stage_update.on_resume(1.0)
        event_stream.pump()
        self.assertEqual(received_events[-1], SimulationEvent.RESUMED)

        self.physics_stage_update.on_pause()
        event_stream.pump()
        self.assertEqual(received_events[-1], SimulationEvent.PAUSED)

        self.physics_stage_update.on_resume(2.0)
        event_stream.pump()
        self.assertEqual(received_events[-1], SimulationEvent.RESUMED)

        self.physics_stage_update.on_reset()
        event_stream.pump()
        self.assertEqual(received_events[-1], SimulationEvent.STOPPED)

        self.assertEqual(len(received_events), 4)
        self.physics_stage_update.on_detach()


if __name__ == "__main__":
    unittest.main()
