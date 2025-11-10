# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math

from pxr import Gf, PhysxSchema

from ..helpers import Factory
from .VehicleSampleBase import VehicleSampleBase
from . import Stepper

import omni.physxdemos as demo


class WheelControllerDemo(VehicleSampleBase):
    title = "Wheel controller"
    category = demo.Categories.VEHICLES
    short_description = "Usage of wheel controller"
    description = "Demo showing how to control wheels directly when not using a drive model."

    def create(self, stage):
        super().create(stage)

        create(stage)

        self.autofocus = True # autofocus on the scene at first update
        self.autofocus_zoom = 0.28 # Get a bit closer


class WheelControllerScenario(Stepper.Scenario):
    def __init__(
        self, stage, vehiclePaths, wheelAttachmentPaths, wheelDriveTorques, wheelBrakeTorques, wheelSteerAngles,
        timeStepsPerSecond
    ):
        secondsToRun = 6.0
        super().__init__(secondsToRun, 1.0 / timeStepsPerSecond)
        self._stage = stage
        self._vehicleCount = len(vehiclePaths)
        self._vehiclePaths = vehiclePaths
        self._wheelAttachmentPaths = wheelAttachmentPaths
        self._wheelControllers = []
        for i in range(self._vehicleCount):
            self._wheelControllers.append([])
            for j in range(len(self._wheelAttachmentPaths[i])):
                prim = self._stage.GetPrimAtPath(self._wheelAttachmentPaths[i][j])
                self._wheelControllers[i].append(PhysxSchema.PhysxVehicleWheelControllerAPI(prim))
        self._wheelDriveTorques = wheelDriveTorques
        self._wheelBrakeTorques = wheelBrakeTorques
        self._wheelSteerAngles = wheelSteerAngles

        self._steerStop = 0.3 * secondsToRun
        self._accelStop = 0.5 * secondsToRun
        self._brakeStart = self._accelStop

    def on_start(self):
        for i in range(self._vehicleCount):
            for j in range(len(self._wheelAttachmentPaths[i])):
                self._wheelControllers[i][j].GetBrakeTorqueAttr().Set(0)
                self._wheelControllers[i][j].GetSteerAngleAttr().Set(0)

            self._wheelControllers[i][Factory.WHEEL_FRONT_LEFT].GetDriveTorqueAttr().Set(self._wheelDriveTorques[i])
            self._wheelControllers[i][Factory.WHEEL_FRONT_RIGHT].GetDriveTorqueAttr().Set(self._wheelDriveTorques[i])

    def on_end(self):
        return

    def on_step(self, deltaTime, totalTime):
        if totalTime < self._steerStop:
            for i in range(self._vehicleCount):
                steerAngle = (totalTime / self._steerStop) * self._wheelSteerAngles[i]

                self._wheelControllers[i][Factory.WHEEL_FRONT_LEFT].GetSteerAngleAttr().Set(steerAngle)
                self._wheelControllers[i][Factory.WHEEL_FRONT_RIGHT].GetSteerAngleAttr().Set(steerAngle)
        elif (totalTime - self._steerStop) <= deltaTime:
            for i in range(self._vehicleCount):
                self._wheelControllers[i][Factory.WHEEL_FRONT_LEFT].GetSteerAngleAttr().Set(0)
                self._wheelControllers[i][Factory.WHEEL_FRONT_RIGHT].GetSteerAngleAttr().Set(0)

        if (totalTime > self._accelStop) and ((totalTime - self._accelStop) <= deltaTime):
            for i in range(self._vehicleCount):
                self._wheelControllers[i][Factory.WHEEL_FRONT_LEFT].GetDriveTorqueAttr().Set(0)
                self._wheelControllers[i][Factory.WHEEL_FRONT_RIGHT].GetDriveTorqueAttr().Set(0)

        if (totalTime >= self._brakeStart) and ((totalTime - self._brakeStart) <= deltaTime):
            for i in range(self._vehicleCount):
                self._wheelControllers[i][Factory.WHEEL_REAR_LEFT].GetBrakeTorqueAttr().Set(self._wheelBrakeTorques[i])
                self._wheelControllers[i][Factory.WHEEL_REAR_RIGHT].GetBrakeTorqueAttr().Set(self._wheelBrakeTorques[i])


def create(stage):
    vehicleCount = 3
    vehiclePaths = []
    wheelAttachmentPaths = []
    timeStepsPerSec = 60

    Factory.create4WheeledCarsScenario(
        stage,
        1.0,
        vehicleCount,
        createCollisionShapesForWheels=True,
        driveMode=Factory.DRIVE_NONE,
        vehiclePathsOut=vehiclePaths,
        wheelAttachmentPathsOut=wheelAttachmentPaths,
        vehicleDelta=[-3, 0, 0],
        timeStepsPerSecond = timeStepsPerSec
    )

    wheelDriveTorques = [300, 600, 900]

    wheelBrakeTorques = [500, 1000, 2000]

    wheelSteerAngles = [(45 * math.pi) / 180, (30 * math.pi) / 180, (10 * math.pi) / 180]

    scenario = WheelControllerScenario(
        stage, vehiclePaths, wheelAttachmentPaths, wheelDriveTorques, wheelBrakeTorques, wheelSteerAngles,
        timeStepsPerSec
    )
    Stepper.run_scenario(scenario)
