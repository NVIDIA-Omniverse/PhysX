# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Ported from omni.physx.tests PhysxInterfaceSimulationEvents.py for standalone ovruntime testing.
# Only the MemoryStage class is ported; the KitStage class requires Kit/timeline and is not portable.

import _physics_setup  # noqa: F401 - loads Carbonite + physics plugins + PhysX schemas

import unittest
import _physx
from _physx import SimulationEvent
import physicsUtils
import carb.settings
from physicsBase import PhysicsMemoryStageBaseTestCase, TestCategory
from pxr import UsdGeom, Gf, UsdPhysics


class PhysxInterfaceSimulationEventsTestMemoryStage(PhysicsMemoryStageBaseTestCase):
    category = TestCategory.Core

    def _on_simulation_event(self, event):
        if event.type == int(SimulationEvent.STOPPED):
            self._stoppedEventCount = self._stoppedEventCount + 1

    def test_physics_sim_event_stopped_on_close(self):
        stage = self.new_stage()

        physxInterface = _physx.acquire_physx_interface()
        for j in range(2):
            # 0: close stage after simulation was started -> expect event
            # 1: close stage after simulation ended       -> expect no extra event

            self._stoppedEventCount = 0
            simulationEventSubcription = physxInterface.get_simulation_event_stream_v2().create_subscription_to_pop(
                self._on_simulation_event
            )

            UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
            UsdGeom.SetStageMetersPerUnit(stage, 1)

            scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
            scene.CreateGravityDirectionAttr(Gf.Vec3f(0, -1.0, 0))
            scene.CreateGravityMagnitudeAttr(10.0)

            sphereActorPath = "/SphereActor"
            radius = 0.1
            color = Gf.Vec3f(255.0, 255.0, 255.0)
            physicsUtils.add_rigid_sphere(
                stage,
                sphereActorPath,
                radius,
                Gf.Vec3f(0.0, 1.0, 0.0),
                Gf.Quatf(1.0),
                color,
                1000.0,
                Gf.Vec3f(0.0),
                Gf.Vec3f(0.0),
            )

            timeStep = 1.0 / 60.0
            physxInterface.start_simulation()
            for i in range(10):
                physxInterface.update_simulation(timeStep, timeStep)

            if j == 1:
                physxInterface.reset_simulation()
                self.assertTrue(self._stoppedEventCount == 1)

            stage = self.new_stage()

            # A) if the simulation was not ended, an event should get sent on close
            # B) if the simulation was ended, there should not be another event
            self.assertTrue(self._stoppedEventCount == 1)

            simulationEventSubcription = None

    @unittest.skip("Requires omni.timeline and omni.kit.app (Kit-only)")
    def test_physics_subscription_default_scene(self):
        pass

    @unittest.skip("Requires omni.timeline and omni.kit.app (Kit-only)")
    def test_physics_pre_post_subscriptions_default_scene_check_events_and_ordering(self):
        pass

    def test_physics_step_subscription_simulation_iface(self):
        settings = carb.settings.acquire_settings_interface()
        saved_num_threads = settings.get_as_int(_physx.SETTING_NUM_THREADS)
        settings.set(_physx.SETTING_NUM_THREADS, 4)
        stage = self.new_stage()
        self.check_dt = 0.0  # set this to zero to start

        def on_update(dt):
            self.check_dt = dt

        scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")

        sub = _physx.acquire_physx_interface().subscribe_physics_step_events(on_update)

        _physx.acquire_physx_simulation_interface().simulate(1.0/60.0, 1.0/60.0)
        _physx.acquire_physx_simulation_interface().fetch_results()

        self.assertNotEqual(self.check_dt, 0.0)

        # check fails
        sub = None
        settings.set(_physx.SETTING_NUM_THREADS, saved_num_threads)

    def test_physics_step_pre_subscription_simulation_iface(self):
        settings = carb.settings.acquire_settings_interface()
        saved_num_threads = settings.get_as_int(_physx.SETTING_NUM_THREADS)
        settings.set(_physx.SETTING_NUM_THREADS, 4)
        stage = self.new_stage()
        self.check_dt = 0.0  # set this to zero to start

        def on_update(dt):
            self.check_dt = dt

        sub = _physx.acquire_physx_interface().subscribe_physics_on_step_events(on_update, True, 0)

        _physx.acquire_physx_simulation_interface().simulate(1.0/60.0, 1.0/60.0)
        _physx.acquire_physx_simulation_interface().fetch_results()

        self.assertNotEqual(self.check_dt, 0.0)

        # check fails
        sub = None
        settings.set(_physx.SETTING_NUM_THREADS, saved_num_threads)

    @unittest.skip("flaky")
    def test_physics_step_subscription_remove_from_step(self):
        stage = self.new_stage()
        self.sub = None
        self.check_sub = False

        def on_update(dt):
            self.check_sub = True
            self.sub = None

        UsdPhysics.Scene.Define(stage, "/PhysicsScene")
        self.sub = _physx.acquire_physx_interface().subscribe_physics_step_events(on_update)

        _physx.acquire_physx_simulation_interface().simulate(1.0/60.0, 1.0/60.0)
        _physx.acquire_physx_simulation_interface().fetch_results()

        self.assertEqual(self.check_sub, True)
        self.check_sub = False

        _physx.acquire_physx_simulation_interface().simulate(1.0/60.0, 1.0/60.0)
        _physx.acquire_physx_simulation_interface().fetch_results()
        self.assertEqual(self.check_sub, False)

        # check fails
        self.sub = None

    def test_physics_step_subscription_remove_from_step_b(self):
        stage = self.new_stage()
        self.sub_a = None
        self.check_sub_a = False
        self.sub_b = None
        self.check_sub_b = False

        def on_update_a(dt):
            self.check_sub_a = True
            self.sub_b = None

        def on_update_b(dt):
            self.check_sub_b = True

        UsdPhysics.Scene.Define(stage, "/PhysicsScene")
        self.sub_a = _physx.acquire_physx_interface().subscribe_physics_on_step_events(on_update_a, False, 1)
        self.sub_b = _physx.acquire_physx_interface().subscribe_physics_on_step_events(on_update_b, False, 2)

        _physx.acquire_physx_simulation_interface().simulate(1.0/60.0, 1.0/60.0)
        _physx.acquire_physx_simulation_interface().fetch_results()

        self.assertEqual(self.check_sub_a, True)
        self.assertEqual(self.check_sub_b, False)
        self.check_sub_a = False

        _physx.acquire_physx_simulation_interface().simulate(1.0/60.0, 1.0/60.0)
        _physx.acquire_physx_simulation_interface().fetch_results()
        self.assertEqual(self.check_sub_a, True)
        self.assertEqual(self.check_sub_b, False)

        self.sub_a = None

    def test_physics_step_subscription_add_from_step(self):
        stage = self.new_stage()
        self.sub_a = None
        self.check_sub_a = False
        self.sub_b = None
        self.check_sub_b = False
        self.sub_c = None
        self.check_sub_c = False

        def on_update_b(dt):
            self.check_sub_b = True

        def on_update_c(dt):
            self.check_sub_c = True

        def on_update_a(dt):
            self.check_sub_a = True

            self.sub_b = _physx.acquire_physx_interface().subscribe_physics_step_events(on_update_b)
            self.sub_c = _physx.acquire_physx_interface().subscribe_physics_step_events(on_update_c)

            self.sub_a = None

        self.sub_a = _physx.acquire_physx_interface().subscribe_physics_step_events(on_update_a)

        _physx.acquire_physx_simulation_interface().simulate(1.0/60.0, 1.0/60.0)
        _physx.acquire_physx_simulation_interface().fetch_results()

        self.assertEqual(self.check_sub_a, True)
        self.assertEqual(self.check_sub_b, False)
        self.assertEqual(self.check_sub_c, False)
        self.check_sub_a = False

        _physx.acquire_physx_simulation_interface().simulate(1.0/60.0, 1.0/60.0)
        _physx.acquire_physx_simulation_interface().fetch_results()
        self.assertEqual(self.check_sub_a, False)
        self.assertEqual(self.check_sub_b, True)
        self.assertEqual(self.check_sub_c, True)

        self.sub_b = None
        self.sub_c = None

        self.check_sub_a = False
        self.check_sub_b = False
        self.check_sub_c = False

        _physx.acquire_physx_simulation_interface().simulate(1.0/60.0, 1.0/60.0)
        _physx.acquire_physx_simulation_interface().fetch_results()

        self.assertEqual(self.check_sub_a, False)
        self.assertEqual(self.check_sub_b, False)
        self.assertEqual(self.check_sub_c, False)

        self.sub_a = None
        self.sub_b = None
        self.sub_c = None


if __name__ == "__main__":
    unittest.main()
