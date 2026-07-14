# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#

import _carb_setup  # noqa: F401 - initializes Carbonite framework

import unittest
from _physics import (
    PhysicsStepContext,
    Simulation,
    acquire_physics_interface,
    acquire_physics_simulation_interface,
    k_invalid_simulation_id,
    ContactEventHeaderVector,
    ContactDataVector,
    FrictionAnchorsDataVector,
    ContactEventType,
    ContactEventHeader,
    ContactData,
    FrictionAnchor,
    ForceMode,
)


class MockSimulator:
    """Mock implementation of physics simulation functionality"""

    def __init__(self):
        self.attached_stage_id = 0
        self.change_tracking_paused = False
        self.time_steps_per_second = 60
        self.simulation_timestamp = 0
        self.simulation_step_count = 0
        self.step_event_callbacks = []
        self.contact_event_callbacks = []
        self.simulation_id = 0
        self.capability_check_enabled = True
        self.supported_capabilities = {}

    def initialize(self, stage_id):
        self.attached_stage_id = stage_id
        return True

    def close(self):
        self.attached_stage_id = 0

    def get_attached_stage(self):
        return self.attached_stage_id

    def simulate_async(self, elapsed_time, current_time):
        self.simulation_timestamp += 1
        self.simulation_step_count += 1
        context = PhysicsStepContext()
        context.scene_path = self.attached_stage_id
        context.simulation_id = self.simulation_id
        for callback in self.step_event_callbacks:
            callback(elapsed_time, context)

    def simulate(self, elapsed_time, current_time):
        self.simulate_async(elapsed_time, current_time)
        self.fetch_results()

    def fetch_results(self):
        contact_event_headers = ContactEventHeaderVector()
        contact_data = ContactDataVector()
        friction_anchors = FrictionAnchorsDataVector()
        for callback in self.contact_event_callbacks:
            callback(contact_event_headers, contact_data, friction_anchors)

    def check_results(self):
        return True

    def flush_changes(self):
        pass

    def pause_change_tracking(self, pause):
        self.change_tracking_paused = pause

    def is_change_tracking_paused(self):
        return self.change_tracking_paused

    def subscribe_physics_contact_report_events(self, on_event):
        self.contact_event_callbacks.append(on_event)
        return len(self.contact_event_callbacks)

    def unsubscribe_physics_contact_report_events(self, subscription_id):
        if subscription_id > 0 and subscription_id <= len(self.contact_event_callbacks):
            self.contact_event_callbacks.pop(subscription_id - 1)

    def get_simulation_time_steps_per_second(self, stage_id, scene_path):
        return self.time_steps_per_second

    def get_simulation_timestamp(self):
        return self.simulation_timestamp

    def get_simulation_step_count(self):
        return self.simulation_step_count

    def subscribe_physics_on_step_events(self, pre_step, order, on_update):
        self.step_event_callbacks.append(on_update)
        return len(self.step_event_callbacks)

    def unsubscribe_physics_on_step_events(self, subscription_id):
        if subscription_id > 0 and subscription_id <= len(self.step_event_callbacks):
            self.step_event_callbacks.pop(subscription_id - 1)

    def is_capable_of_simulating(self, schema_names):
        if not self.capability_check_enabled:
            return (False, [])
        capabilities = []
        for schema_name in schema_names:
            capabilities.append(self.supported_capabilities.get(schema_name, False))
        return (True, capabilities)

    def set_capability_check_enabled(self, enabled):
        self.capability_check_enabled = enabled

    def set_supported_capability(self, schema_name, is_supported):
        self.supported_capabilities[schema_name] = is_supported


def _setup_simulation_fns(simulation, mock):
    """Wire all simulation function pointers to the mock."""
    simulation.simulation_fns.initialize = mock.initialize
    simulation.simulation_fns.close = mock.close
    simulation.simulation_fns.get_attached_stage = mock.get_attached_stage
    simulation.simulation_fns.simulate_async = mock.simulate_async
    simulation.simulation_fns.simulate = mock.simulate
    simulation.simulation_fns.fetch_results = mock.fetch_results
    simulation.simulation_fns.check_results = mock.check_results
    simulation.simulation_fns.flush_changes = mock.flush_changes
    simulation.simulation_fns.pause_change_tracking = mock.pause_change_tracking
    simulation.simulation_fns.is_change_tracking_paused = mock.is_change_tracking_paused
    simulation.simulation_fns.subscribe_physics_contact_report_events = mock.subscribe_physics_contact_report_events
    simulation.simulation_fns.unsubscribe_physics_contact_report_events = mock.unsubscribe_physics_contact_report_events
    simulation.simulation_fns.get_simulation_time_steps_per_second = mock.get_simulation_time_steps_per_second
    simulation.simulation_fns.get_simulation_timestamp = mock.get_simulation_timestamp
    simulation.simulation_fns.get_simulation_step_count = mock.get_simulation_step_count
    simulation.simulation_fns.subscribe_physics_on_step_events = mock.subscribe_physics_on_step_events
    simulation.simulation_fns.unsubscribe_physics_on_step_events = mock.unsubscribe_physics_on_step_events
    simulation.simulation_fns.is_capable_of_simulating = mock.is_capable_of_simulating


class TestSimulatorSimulation(unittest.TestCase):
    def setUp(self):
        self.physics = acquire_physics_interface()
        self.physics_simulation = acquire_physics_simulation_interface()
        self.mock_simulator = MockSimulator()

        self.simulation = Simulation()
        _setup_simulation_fns(self.simulation, self.mock_simulator)

        self.simulation_id = self.physics.register_simulation(self.simulation, "MockSimulator")
        self.assertNotEqual(self.simulation_id, k_invalid_simulation_id)
        self.mock_simulator.simulation_id = self.simulation_id

    def tearDown(self):
        if self.simulation_id != k_invalid_simulation_id:
            self.physics.unregister_simulation(self.simulation_id)

    def test_stage_attachment(self):
        self.assertEqual(self.mock_simulator.attached_stage_id, 0)
        self.assertTrue(self.physics_simulation.initialize(123))
        self.assertEqual(self.mock_simulator.attached_stage_id, 123)
        self.assertEqual(self.physics_simulation.get_attached_stage(), 123)
        self.physics_simulation.close()
        self.assertEqual(self.mock_simulator.attached_stage_id, 0)

    def test_simulation_timing(self):
        self.assertEqual(self.physics_simulation.get_simulation_time_steps_per_second(self.simulation_id, 123, 0), 60)
        self.assertEqual(self.physics_simulation.get_simulation_timestamp(self.simulation_id), 0)
        self.assertEqual(self.physics_simulation.get_simulation_step_count(self.simulation_id), 0)

        self.physics_simulation.simulate_async(1.0 / 60.0, 0.0)
        self.physics_simulation.fetch_results()
        self.assertEqual(self.physics_simulation.get_simulation_timestamp(self.simulation_id), 1)

        for i in range(5):
            self.physics_simulation.simulate_async(1.0 / 60.0, (i + 1) * 1.0 / 60.0)
            self.physics_simulation.fetch_results()
        self.assertEqual(self.physics_simulation.get_simulation_timestamp(self.simulation_id), 6)

    def test_change_tracking(self):
        self.assertFalse(self.physics_simulation.is_change_tracking_paused(self.simulation_id))
        self.physics_simulation.pause_change_tracking(True)
        self.assertTrue(self.physics_simulation.is_change_tracking_paused(self.simulation_id))
        self.physics_simulation.pause_change_tracking(False)
        self.assertFalse(self.physics_simulation.is_change_tracking_paused(self.simulation_id))

    def test_contact_callbacks(self):
        contact_callback_called = False

        def on_contact_event(event_headers, contact_data, friction_anchors):
            nonlocal contact_callback_called
            contact_callback_called = True

        sub = self.physics_simulation.subscribe_physics_contact_report_events(on_contact_event)
        self.assertIsNotNone(sub)
        self.physics_simulation.simulate_async(1.0 / 60.0, 0.0)
        self.physics_simulation.fetch_results()
        self.assertTrue(contact_callback_called)

        contact_callback_called = False
        sub = None
        self.physics_simulation.simulate_async(1.0 / 60.0, 1.0 / 60.0)
        self.physics_simulation.fetch_results()
        self.assertFalse(contact_callback_called)

    def test_step_callbacks(self):
        step_callback_called = False
        step_elapsed_time = None

        def on_step_event(elapsed_time, context):
            nonlocal step_callback_called, step_elapsed_time
            step_callback_called = True
            step_elapsed_time = elapsed_time

        sub = self.physics_simulation.subscribe_physics_on_step_events(False, 0, on_step_event)
        self.assertIsNotNone(sub)

        elapsed_time = 1.0 / 60.0
        self.physics_simulation.simulate_async(elapsed_time, 0.0)
        self.physics_simulation.fetch_results()

        self.assertTrue(step_callback_called)
        self.assertAlmostEqual(step_elapsed_time, elapsed_time)

        step_callback_called = False
        sub = None
        self.physics_simulation.simulate_async(1.0 / 60.0, 1.0 / 60.0)
        self.assertFalse(step_callback_called)

    def test_contact_event_type_enum(self):
        self.assertNotEqual(ContactEventType.CONTACT_FOUND, ContactEventType.CONTACT_LOST)
        self.assertNotEqual(ContactEventType.CONTACT_FOUND, ContactEventType.CONTACT_PERSIST)

    def test_force_mode_enum(self):
        self.assertNotEqual(ForceMode.FORCE, ForceMode.IMPULSE)
        self.assertNotEqual(ForceMode.FORCE, ForceMode.VELOCITY_CHANGE)
        self.assertNotEqual(ForceMode.FORCE, ForceMode.ACCELERATION)

    def test_contact_event_header(self):
        header = ContactEventHeader()
        header.type = ContactEventType.CONTACT_FOUND
        header.stage_id = 123
        header.actor0 = 456
        self.assertEqual(header.type, ContactEventType.CONTACT_FOUND)
        self.assertEqual(header.stage_id, 123)
        self.assertEqual(header.actor0, 456)

    def test_contact_data(self):
        contact = ContactData()
        contact.position = [1.0, 2.0, 3.0]
        contact.normal = [0.0, 1.0, 0.0]
        contact.separation = -0.1
        contact.impulse = [5.0, 6.0, 7.0]
        self.assertEqual(contact.position, [1.0, 2.0, 3.0])
        self.assertAlmostEqual(contact.separation, -0.1, delta=1e-5)

    def test_contact_vectors(self):
        header_vector = ContactEventHeaderVector()
        self.assertEqual(len(header_vector), 0)
        header1 = ContactEventHeader()
        header1.type = ContactEventType.CONTACT_FOUND
        header_vector.append(header1)
        self.assertEqual(len(header_vector), 1)

    def test_enhanced_contact_callbacks(self):
        received_headers = None

        def on_contact_event(event_headers, contact_data, friction_anchors):
            nonlocal received_headers
            received_headers = event_headers

        sub = self.physics_simulation.subscribe_physics_contact_report_events(on_contact_event)
        self.physics_simulation.simulate_async(1.0 / 60.0, 0.0)
        self.physics_simulation.fetch_results()
        self.assertIsInstance(received_headers, ContactEventHeaderVector)
        sub = None


class TestSimulatorCapabilityCheck(unittest.TestCase):
    def setUp(self):
        self.physics = acquire_physics_interface()
        self.physics_simulation = acquire_physics_simulation_interface()
        self.mock_simulator = MockSimulator()
        self.mock_simulator.set_supported_capability("PhysicsRigidBodyAPI", True)
        self.mock_simulator.set_supported_capability("PhysicsCollisionAPI", True)
        self.mock_simulator.set_supported_capability("UnsupportedSchemaAPI", False)

        self.simulation = Simulation()
        _setup_simulation_fns(self.simulation, self.mock_simulator)

        self.simulation_id = self.physics.register_simulation(self.simulation, "MockCapabilitySim")
        self.assertNotEqual(self.simulation_id, k_invalid_simulation_id)

    def tearDown(self):
        if self.simulation_id != k_invalid_simulation_id:
            self.physics.unregister_simulation(self.simulation_id)

    def test_check_single_supported_capability(self):
        success, caps = self.physics_simulation.is_capable_of_simulating(self.simulation_id, ["PhysicsRigidBodyAPI"])
        self.assertTrue(success)
        self.assertTrue(caps[0])

    def test_check_single_unsupported_capability(self):
        success, caps = self.physics_simulation.is_capable_of_simulating(self.simulation_id, ["UnsupportedSchemaAPI"])
        self.assertTrue(success)
        self.assertFalse(caps[0])

    def test_check_capability_with_invalid_simulation_id(self):
        success, caps = self.physics_simulation.is_capable_of_simulating(k_invalid_simulation_id, ["PhysicsRigidBodyAPI"])
        self.assertFalse(success)
        self.assertEqual(len(caps), 0)

    def test_check_empty_schema_list(self):
        success, caps = self.physics_simulation.is_capable_of_simulating(self.simulation_id, [])
        self.assertTrue(success)
        self.assertEqual(len(caps), 0)

    def test_dynamic_capability_registration(self):
        success, caps = self.physics_simulation.is_capable_of_simulating(self.simulation_id, ["NewDynamicAPI"])
        self.assertTrue(success)
        self.assertFalse(caps[0])
        self.mock_simulator.set_supported_capability("NewDynamicAPI", True)
        success, caps = self.physics_simulation.is_capable_of_simulating(self.simulation_id, ["NewDynamicAPI"])
        self.assertTrue(success)
        self.assertTrue(caps[0])


if __name__ == "__main__":
    unittest.main()
