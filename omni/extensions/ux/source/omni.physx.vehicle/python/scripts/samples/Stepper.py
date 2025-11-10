# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
import threading

import omni.usd
import omni.timeline
import omni.physx
from omni.physx.bindings._physx import SimulationEvent
from carb.eventdispatcher import get_eventdispatcher

class Scenario:
    def __init__(self, secondsToRun, timeStep = 1.0 / 60.0):
        self._targetIterationCount = math.ceil(secondsToRun / timeStep)

    def get_iteration_count(self):
        return self._targetIterationCount

    # override in subclass as needed
    def on_start(self):
        return

    def on_end(self):
        return

    def on_step(self, deltaTime, totalTime):
        return


class SimStepTracker:
    def __init__(self, scenario, resetAtEnd, scenarioDoneSignal):
        self._scenario = scenario
        self._targetIterationCount = scenario.get_iteration_count()
        self._resetAtEnd = resetAtEnd
        self._scenarioDoneSignal = scenarioDoneSignal

        self._timeline = omni.timeline.get_timeline_interface()

        self._physx = omni.physx.get_physx_interface()
        self._physxSimEventSubscription = self._physx.get_simulation_event_stream_v2().create_subscription_to_pop(
            self._on_simulation_event
        )

        self._processStepEvents = False
        self._hasStarted = False
        self._resetOnNextResume = False

    def abort(self):
        self._processStepEvents = False

        # note: closing a stage sends the SimulationEvent.STOPPED event now. Thus, it would not be necessary
        #       anymore to check and trigger the stop logic. However, this method is kept general in case
        #       it gets called on other occasions in the future.
        if self._hasStarted:
            self._on_stop()

        self._physxSimEventSubscription = None

        self._physx = (
            None
        )  # should release automatically (note: explicit release call results in double release being reported)

        self._timeline = None

        self._scenarioDoneSignal.set()

    def _on_stop(self):
        self._hasStarted = False
        self._physxStepEventSubscription = None  # should unsubscribe automatically
        self._scenario.on_end()

    def _on_scenario_start(self):
        self._scenario.on_start()
        self._iterationCount = 0
        self._totalTime = 0

    def _on_simulation_event(self, event):
        if event.type == int(SimulationEvent.RESUMED):
            if not self._hasStarted:
                self._on_scenario_start()
                self._physxStepEventSubscription = self._physx.subscribe_physics_step_events(self._on_physics_step)
                self._hasStarted = True
            elif self._resetOnNextResume:
                self._resetOnNextResume = False

                # the simulation step callback is still registered and should remain so, thus no unsubscribe
                self._scenario.on_end()

                self._on_scenario_start()

            self._processStepEvents = True
        elif event.type == int(SimulationEvent.STOPPED):
            self._processStepEvents = False
            self._on_stop()

    def _on_physics_step(self, dt):
        if self._processStepEvents:
            #print(f"step: {self._iterationCount}\n")

            if self._iterationCount < self._targetIterationCount:
                self._scenario.on_step(dt, self._totalTime)
                self._iterationCount += 1
                self._totalTime += dt
            else:
                self._processStepEvents = False

                # scenario has finished -> stop or pause. Pressing play after that should restart the scenario
                if self._resetAtEnd:
                    self._timeline.stop()
                else:
                    self._timeline.pause()
                    self._resetOnNextResume = True


class StageEventListener:
    def __init__(self, simStepTracker):
        self._simStepTracker = simStepTracker
        self._stageEventSubscription = get_eventdispatcher().observe_event(
            observer_name="omni.physx.vehicle:SamplesStepper",
            event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.CLOSING),
            on_event=lambda _: self._simStepTracker.abort()
        )

    def cleanup(self):
        self._stageEventSubscription = None


def _run_scenario_internal(scenario, resetAtEnd):
    scenarioDoneSignal = threading.Event()
    simStepTracker = SimStepTracker(scenario, resetAtEnd, scenarioDoneSignal)
    stageEventListener = StageEventListener(simStepTracker)

    scenarioDoneSignal.wait()

    stageEventListener.cleanup()
    del stageEventListener
    del simStepTracker


def run_scenario(scenario, resetAtEnd=True):
    thread = threading.Thread(target=_run_scenario_internal, args=(scenario, resetAtEnd))
    thread.start()
