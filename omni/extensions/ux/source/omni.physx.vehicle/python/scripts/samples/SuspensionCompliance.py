# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math

from pxr import Gf, PhysxSchema, UsdPhysics, UsdUtils, PhysicsSchemaTools

from ..helpers import Factory
from . import Stepper
from .VehicleSampleBase import VehicleSampleBase

import omni.physx
import omni.physxdemos as demo


class SuspensionComplianceDemo(VehicleSampleBase):
    title = "Suspension Compliance"
    category = demo.Categories.VEHICLES
    short_description = "Usage of suspension compliance"
    description = ("Demo showing how to use suspension compliance related parameters to have "
        "properties like toe/camber angle or force application points dynamically adjust with "
        "changing suspension jounce.")

    def create(self, stage):
        super().create(stage)

        create(stage)

        self.autofocus = True # autofocus on the scene at first update
        self.autofocus_zoom = 0.1 # Get a bit closer


class SuspensionComplianceScenario(Stepper.Scenario):
    def __init__(
        self, stage, vehiclePath, timeStepsPerSecond
    ):
        secondsToRun = 6.0
        super().__init__(secondsToRun, 1.0 / timeStepsPerSecond)
        self._physxSimulationInterface = omni.physx.get_physx_simulation_interface()
        self._stage = stage
        self._stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self._vehiclePath = vehiclePath
        self._rbo_encoded = PhysicsSchemaTools.sdfPathToInt(self._vehiclePath)
        self._currentCycleTime = 0.0
        self._forceCycleTime = 2.0

        vehiclePrim = self._stage.GetPrimAtPath(vehiclePath)

        self._forcePos = vehiclePrim.GetAttribute("xformOp:translate").Get()

        massAPI = UsdPhysics.MassAPI(vehiclePrim)
        self._maxForceAbs = massAPI.GetMassAttr().Get() * 9.81

    def on_start(self):
        return

    def on_end(self):
        return

    def on_step(self, deltaTime, totalTime):
        self._currentCycleTime = self._currentCycleTime + deltaTime
        if (self._currentCycleTime > self._forceCycleTime):
            self._currentCycleTime = self._currentCycleTime - self._forceCycleTime

        multiplier = math.sin((self._currentCycleTime / self._forceCycleTime) * 2.0 * math.pi)
        forceMagn = self._maxForceAbs * multiplier
        
        self._physxSimulationInterface.apply_force_at_pos(self._stage_id, self._rbo_encoded, Gf.Vec3f(0.0, forceMagn, 0.0), 
            self._forcePos, "Force")


def create(stage):
    vehicleCount = 1
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
        timeStepsPerSecond = timeStepsPerSec
    )

    toeAngleAbs = (20 * math.pi) / 180
    camberAngleAbs = (30 * math.pi) / 180

    for i in range(len(wheelAttachmentPaths[0])):
        path = wheelAttachmentPaths[0][i]
        prim = stage.GetPrimAtPath(path)
        suspensionComplianceAPI = PhysxSchema.PhysxVehicleSuspensionComplianceAPI.Apply(prim)

        if (i == Factory.WHEEL_FRONT_LEFT):
            suspensionComplianceAPI.CreateWheelToeAngleAttr([Gf.Vec2f(0.0, toeAngleAbs), Gf.Vec2f(1.0, -toeAngleAbs)])
            suspensionComplianceAPI.CreateWheelCamberAngleAttr([Gf.Vec2f(0.0, -camberAngleAbs), Gf.Vec2f(0.5, 0.0), Gf.Vec2f(1.0, camberAngleAbs)])
        elif (i == Factory.WHEEL_FRONT_RIGHT):
            suspensionComplianceAPI.CreateWheelToeAngleAttr([Gf.Vec2f(0.0, -toeAngleAbs), Gf.Vec2f(1.0, toeAngleAbs)])
            suspensionComplianceAPI.CreateWheelCamberAngleAttr([Gf.Vec2f(0.0, camberAngleAbs), Gf.Vec2f(0.5, 0.0), Gf.Vec2f(1.0, -camberAngleAbs)])
        elif (i == Factory.WHEEL_REAR_LEFT):
            # empty array (or non-authored value) will be treated as 0 toe angle
            suspensionComplianceAPI.CreateWheelToeAngleAttr([])
            suspensionComplianceAPI.CreateWheelCamberAngleAttr([Gf.Vec2f(0.0, -camberAngleAbs), Gf.Vec2f(0.5, 0.0), Gf.Vec2f(1.0, camberAngleAbs)])
        else:
            suspensionComplianceAPI.CreateWheelToeAngleAttr([])
            suspensionComplianceAPI.CreateWheelCamberAngleAttr([Gf.Vec2f(0.0, camberAngleAbs), Gf.Vec2f(0.5, 0.0), Gf.Vec2f(1.0, -camberAngleAbs)])

        def _get_suspension(wheelAttPrim, stage):
            if (wheelAttPrim.HasAPI(PhysxSchema.PhysxVehicleSuspensionAPI)):
                return PhysxSchema.PhysxVehicleSuspensionAPI(wheelAttPrim)
            wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(wheelAttPrim)
            suspPath = wheelAttAPI.GetSuspensionRel().GetTargets()[0]
            suspPrim = stage.GetPrimAtPath(suspPath)
            susp = PhysxSchema.PhysxVehicleSuspensionAPI(suspPrim)
            return susp

        suspensionAPI = _get_suspension(prim, stage)
        travelDistance = suspensionAPI.GetTravelDistanceAttr().Get();

        # let the force application points move along the suspension direction as the
        # suspension compresses.
        forceAppPointList = [
            Gf.Vec4f(0.0, 0.0, -travelDistance, 0),
            Gf.Vec4f(1.0, 0.0, 0.0, 0.0)
        ]
        suspensionComplianceAPI.CreateSuspensionForceAppPointAttr(forceAppPointList)
        suspensionComplianceAPI.CreateTireForceAppPointAttr(forceAppPointList)

    scenario = SuspensionComplianceScenario(
        stage, vehiclePaths[0], timeStepsPerSec
    )
    Stepper.run_scenario(scenario)
