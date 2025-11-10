# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math

from pxr import Gf, UsdGeom, PhysxSchema

import omni.physxvehicle

from ..helpers import Factory
from .VehicleSampleBase import VehicleSampleBase
from . import Stepper

import omni.physxdemos as demo


class AckermannSteeringDemo(VehicleSampleBase):
    title = "Ackermann steering"
    category = demo.Categories.VEHICLES
    short_description = "Usage of Ackermann steering helper method"
    description = "Demo showing how to use the Ackermann steering helper method."

    def create(self, stage):
        super().create(stage)

        self._physxVehicleInterface = omni.physxvehicle.get_physx_vehicle_interface()

        rootPath = str(stage.GetDefaultPrim().GetPath())
        cameraPath = rootPath + "/Camera"
        camera = UsdGeom.Camera.Define(stage, cameraPath)
        camera.AddTranslateOp().Set(Gf.Vec3d(-5, 35, 0))
        camera.AddRotateYXZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(90, 180, 0))
        self.demo_camera = cameraPath

        create(stage, self._physxVehicleInterface)

    def on_shutdown(self):
        self._physxVehicleInterface = None


class AckermannSteeringHelperScenario(Stepper.Scenario):
    def __init__(self, stage, physxVehicleInterface, wheelAttachmentPaths, wheelSteerAngles, axleSeparation, axleWidth,
        timeStepsPerSecond):
        secondsToRun = 4.0
        super().__init__(secondsToRun, 1.0 / timeStepsPerSecond)
        self._stage = stage
        self._physxVehicleInterface = physxVehicleInterface
        self._wheelAttachmentPaths = wheelAttachmentPaths
        self._wheelSteerAngles = wheelSteerAngles
        self._axleSeparation = axleSeparation
        self._axleWidth = axleWidth

        self._steerStop = secondsToRun
        self._steerCycle = self._steerStop

    def on_start(self):
        for i in range(len(self._wheelAttachmentPaths)):
            for j in range(len(self._wheelAttachmentPaths[i])):
                prim = self._stage.GetPrimAtPath(self._wheelAttachmentPaths[i][j])
                wheelControllerAPI = PhysxSchema.PhysxVehicleWheelControllerAPI(prim)
                wheelControllerAPI.GetSteerAngleAttr().Set(0)

    def on_end(self):
        return

    def on_step(self, deltaTime, totalTime):
        if totalTime < self._steerStop:
            fullSteerCycleCount = math.trunc(totalTime / self._steerCycle)
            currentCycle = totalTime - (fullSteerCycleCount * self._steerCycle)
            ratio = currentCycle / self._steerCycle

            steerAngleInnerWheel = ratio * self._wheelSteerAngles[0]
            steerAngleOuterWheel = self._physxVehicleInterface.compute_ackermann_steering_angle(
                steerAngleInnerWheel, self._axleSeparation, self._axleWidth
            )

            prim = self._stage.GetPrimAtPath(self._wheelAttachmentPaths[0][Factory.WHEEL_FRONT_LEFT])
            wheelControllerAPI_FL = PhysxSchema.PhysxVehicleWheelControllerAPI(prim)

            prim = self._stage.GetPrimAtPath(self._wheelAttachmentPaths[0][Factory.WHEEL_FRONT_RIGHT])
            wheelControllerAPI_FR = PhysxSchema.PhysxVehicleWheelControllerAPI(prim)

            if steerAngleInnerWheel >= 0:
                wheelControllerAPI_FL.GetSteerAngleAttr().Set(steerAngleInnerWheel)
                wheelControllerAPI_FR.GetSteerAngleAttr().Set(steerAngleOuterWheel)
            else:
                wheelControllerAPI_FL.GetSteerAngleAttr().Set(steerAngleOuterWheel)
                wheelControllerAPI_FR.GetSteerAngleAttr().Set(steerAngleInnerWheel)

            # ---

            # the helper can of course also be used to set steering angles for rear wheels.
            # In that case the angles just have to be applied in the opposite order.

            steerAngleInnerWheel = ratio * self._wheelSteerAngles[1]
            steerAngleOuterWheel = self._physxVehicleInterface.compute_ackermann_steering_angle(
                steerAngleInnerWheel, self._axleSeparation, self._axleWidth
            )

            prim = self._stage.GetPrimAtPath(self._wheelAttachmentPaths[1][Factory.WHEEL_REAR_LEFT])
            wheelControllerAPI_RL = PhysxSchema.PhysxVehicleWheelControllerAPI(prim)

            prim = self._stage.GetPrimAtPath(self._wheelAttachmentPaths[1][Factory.WHEEL_REAR_RIGHT])
            wheelControllerAPI_RR = PhysxSchema.PhysxVehicleWheelControllerAPI(prim)

            if steerAngleInnerWheel >= 0:
                wheelControllerAPI_RL.GetSteerAngleAttr().Set(steerAngleOuterWheel)
                wheelControllerAPI_RR.GetSteerAngleAttr().Set(steerAngleInnerWheel)
            else:
                wheelControllerAPI_RL.GetSteerAngleAttr().Set(steerAngleInnerWheel)
                wheelControllerAPI_RR.GetSteerAngleAttr().Set(steerAngleOuterWheel)

            # ---

            # or all four wheels can be set such that they share the same center of the turning circle.

            steerAngleInnerWheel = ratio * self._wheelSteerAngles[2]
            steerAngleOuterWheel = self._physxVehicleInterface.compute_ackermann_steering_angle(
                steerAngleInnerWheel, self._axleSeparation / 2, self._axleWidth
            )

            prim = self._stage.GetPrimAtPath(self._wheelAttachmentPaths[2][Factory.WHEEL_FRONT_LEFT])
            wheelControllerAPI_FL = PhysxSchema.PhysxVehicleWheelControllerAPI(prim)

            prim = self._stage.GetPrimAtPath(self._wheelAttachmentPaths[2][Factory.WHEEL_FRONT_RIGHT])
            wheelControllerAPI_FR = PhysxSchema.PhysxVehicleWheelControllerAPI(prim)

            prim = self._stage.GetPrimAtPath(self._wheelAttachmentPaths[2][Factory.WHEEL_REAR_LEFT])
            wheelControllerAPI_RL = PhysxSchema.PhysxVehicleWheelControllerAPI(prim)

            prim = self._stage.GetPrimAtPath(self._wheelAttachmentPaths[2][Factory.WHEEL_REAR_RIGHT])
            wheelControllerAPI_RR = PhysxSchema.PhysxVehicleWheelControllerAPI(prim)

            if steerAngleInnerWheel >= 0:
                wheelControllerAPI_FL.GetSteerAngleAttr().Set(steerAngleInnerWheel)
                wheelControllerAPI_FR.GetSteerAngleAttr().Set(steerAngleOuterWheel)
                wheelControllerAPI_RL.GetSteerAngleAttr().Set(-steerAngleInnerWheel)
                wheelControllerAPI_RR.GetSteerAngleAttr().Set(-steerAngleOuterWheel)
            else:
                wheelControllerAPI_FL.GetSteerAngleAttr().Set(steerAngleOuterWheel)
                wheelControllerAPI_FR.GetSteerAngleAttr().Set(steerAngleInnerWheel)
                wheelControllerAPI_RL.GetSteerAngleAttr().Set(-steerAngleOuterWheel)
                wheelControllerAPI_RR.GetSteerAngleAttr().Set(-steerAngleInnerWheel)


def create(stage, physxVehicleInterface):
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
        vehicleDelta=[-5, 0, 0],
        addChassisRenderMesh=False,
        timeStepsPerSecond=timeStepsPerSec
    )

    wheelSteerAngles = [(45 * math.pi) / 180, (-45 * math.pi) / 180, (45 * math.pi) / 180]

    prim = stage.GetPrimAtPath(wheelAttachmentPaths[0][Factory.WHEEL_FRONT_LEFT])
    posFL = prim.GetAttribute("xformOp:translate").Get()

    prim = stage.GetPrimAtPath(wheelAttachmentPaths[0][Factory.WHEEL_FRONT_RIGHT])
    posFR = prim.GetAttribute("xformOp:translate").Get()

    prim = stage.GetPrimAtPath(wheelAttachmentPaths[0][Factory.WHEEL_REAR_LEFT])
    posRL = prim.GetAttribute("xformOp:translate").Get()

    axleSeparation = posFL[2] - posRL[2]
    axleWidth = posFL[0] - posFR[0]

    scenario = AckermannSteeringHelperScenario(
        stage, physxVehicleInterface, wheelAttachmentPaths, wheelSteerAngles, axleSeparation, axleWidth,
        timeStepsPerSec
    )
    Stepper.run_scenario(scenario, resetAtEnd=False)
