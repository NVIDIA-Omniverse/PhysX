# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math

from pxr import Gf, PhysxSchema

from .VehicleSampleBase import VehicleSampleBase
from . import Stepper
from . import BasicSetup

import omni.physxdemos as demo


class TankDemo(VehicleSampleBase):
    title = "Tank"
    category = demo.Categories.VEHICLES
    short_description = "Wheeled tank type vehicle"
    description = "Demo showing how to control a vehicle that is set up as a wheeled tank."

    def create(self, stage):
        super().create(stage)

        create(stage)

        self.autofocus = True # autofocus on the scene at first update
        self.autofocus_zoom = 0.28 # Get a bit closer


class TankScenario(Stepper.Scenario):
    def __init__(
        self, stage, vehiclePath, timeStepsPerSecond
    ):
        self._rotateEnd = 2.5
        self._forwardEnd = self._rotateEnd + 1.5
        self._brakeEnd = self._forwardEnd + 1.5
        self._reverseEnd = self._brakeEnd + 1.0
        self._turnEnd = self._reverseEnd + 3.0
        secondsToRun = self._turnEnd

        super().__init__(secondsToRun, 1.0 / timeStepsPerSecond)
        self._stage = stage
        vehiclePrim = stage.GetPrimAtPath(vehiclePath)
        self._vehicleController = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)
        self._vehicleTankController = PhysxSchema.PhysxVehicleTankControllerAPI(vehiclePrim)

    def on_start(self):
        self._vehicleController.GetAcceleratorAttr().Set(1.0)
        self._vehicleTankController.GetThrust0Attr().Set(1.0)
        self._vehicleTankController.GetThrust1Attr().Set(-1.0)

    def on_end(self):
        return

    def on_step(self, deltaTime, totalTime):
        if (totalTime > self._rotateEnd) and ((totalTime - self._rotateEnd) <= deltaTime):
            self._vehicleTankController.GetThrust1Attr().Set(1.0)

        if (totalTime > self._forwardEnd) and ((totalTime - self._forwardEnd) <= deltaTime):
            self._vehicleController.GetAcceleratorAttr().Set(0.0)
            self._vehicleController.GetBrake0Attr().Set(1.0)
            self._vehicleController.GetBrake1Attr().Set(1.0)

        if (totalTime > self._brakeEnd) and ((totalTime - self._brakeEnd) <= deltaTime):
            self._vehicleController.GetBrake0Attr().Set(0.0)
            self._vehicleController.GetBrake1Attr().Set(0.0)
            self._vehicleController.GetAcceleratorAttr().Set(1.0)
            self._vehicleTankController.GetThrust0Attr().Set(-1.0)
            self._vehicleTankController.GetThrust1Attr().Set(-1.0)

        if (totalTime > self._reverseEnd) and ((totalTime - self._reverseEnd) <= deltaTime):
            self._vehicleTankController.GetThrust1Attr().Set(0.7)


def create(stage):
    timeStepsPerSec = 60

    primPaths = BasicSetup.create(stage, True, configureAsTank = True)

    scenario = TankScenario(
        stage, primPaths.vehiclePath, timeStepsPerSec
    )
    Stepper.run_scenario(scenario)
