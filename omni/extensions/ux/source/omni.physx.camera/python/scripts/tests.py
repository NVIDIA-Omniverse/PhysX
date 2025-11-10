# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import asyncio
import math
import carb
import carb.input
import os
import random
import platform
from pxr import Gf, UsdGeom, UsdUtils, UsdPhysics, PhysxSchema
import omni.kit.test
import omni.kit.undo
import omni.kit.app
import omni.appwindow
import omni.usd
import omni.kit.stage_templates

from omni.physxcamera.scripts.commands import *
from omni.physxvehicle.scripts.helpers import Factory

import unittest
from omni.physxtests.utils.physicsBase import (
    PhysicsMemoryStageBaseAsyncTestCase,
    PhysicsKitStageAsyncTestCase,
    TestCategory
)


gPhysXInterface = None
gPhysXVehicleInterface = None
gPhysXCameraInterface = None


def setPhysxInterface(physxInterface):
    global gPhysXInterface
    gPhysXInterface = physxInterface

def clearPhysxInterface():
    global gPhysXInterface
    gPhysXInterface = None


def setPhysxSimInterface(physxSimInterface):
    global gPhysXSimInterface
    gPhysXSimInterface = physxSimInterface

def clearPhysxSimInterface():
    global gPhysXSimInterface
    gPhysXSimInterface = None
    

def setPhysxVehicleInterface(physxVehicleInterface):
    global gPhysXVehicleInterface
    gPhysXVehicleInterface = physxVehicleInterface

def clearPhysxVehicleInterface():
    global gPhysXVehicleInterface
    gPhysXVehicleInterface = None


def setPhysxCameraInterface(physxCameraInterface):
    global gPhysXCameraInterface
    gPhysXCameraInterface = physxCameraInterface

def clearPhysxCameraInterface():
    global gPhysXCameraInterface
    gPhysXCameraInterface = None



class PhysXCameraTestBase:

    def _get_time_step(self):
        return 1.0 / 60.0

    def _prepare_for_simulation(self):
        return

    def _simulate_one_frame(self, elapsedTime=None):
        return

    def _simulate_one_frame_with_prep(self):
        self._prepare_for_simulation()
        self._simulate_one_frame()

    def _simulate(self, secondsToRun):
        return

    def _simulate_with_prep(self, secondsToRun):
        self._prepare_for_simulation()
        self._simulate(secondsToRun)

    def _write_back_transforms(self, updateToFastCache, updateToUsd, updateVelocitiesToUsd):
        global gPhysXInterface
        gPhysXInterface.update_transformations(updateToFastCache, updateToUsd, updateVelocitiesToUsd)



class PhysXCameraTestKitStage(PhysicsKitStageAsyncTestCase, PhysXCameraTestBase):

    def _prepare_for_simulation(self):
        global gPhysXInterface
        gPhysXInterface.start_simulation()

        stage = self.get_stage()
        stageId = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        global gPhysXVehicleInterface
        gPhysXVehicleInterface.attach_stage(stageId, False)
        
        global gPhysXCameraInterface
        gPhysXCameraInterface.attach_stage(stageId, False)

    def _simulate_one_frame(self, elapsedTime=None):
        if (elapsedTime is None):
            timeStep = self._get_time_step()
        else:
            timeStep = elapsedTime
        currentTime = 0.0

        global gPhysXInterface
        gPhysXInterface.update_simulation(timeStep, currentTime)

        global gPhysXVehicleInterface
        gPhysXVehicleInterface.update_controllers(timeStep)

    def _simulate(self, secondsToRun):
        timeStep = self._get_time_step()
        targetIterationCount = math.ceil(secondsToRun / timeStep)
        currentTime = 0.0

        global gPhysXInterface
        global gPhysXVehicleInterface

        for i in range(targetIterationCount):
            gPhysXInterface.update_simulation(timeStep, currentTime)
            gPhysXVehicleInterface.update_controllers(timeStep)
            currentTime += timeStep

    async def _open_usd(self, filename):
        data_path = "../../../../data/tests"
        schema_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, data_path)))
        schema_folder = schema_folder.replace("\\", "/") + "/"
        
        await omni.usd.get_context().open_stage_async(schema_folder + filename + ".usda")

        usd_context = omni.usd.get_context()
        self.assertIn(filename, usd_context.get_stage_url())

    #
    # Commands to add vehicle camera.
    #
    async def test_command_add_vehicle_camera(self):
        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths
        )

        #
        # follow camera
        #

        followCameraPath = "/FollowLookCamera"
        self.assertFalse(stage.GetPrimAtPath(followCameraPath))

        PhysXAddFollowLookCameraCommand.execute(vehiclePaths[0], followCameraPath)

        self.assertTrue(stage.GetPrimAtPath(followCameraPath))
        omni.kit.undo.undo()
        self.assertFalse(stage.GetPrimAtPath(followCameraPath))
        omni.kit.undo.redo()
        self.assertTrue(stage.GetPrimAtPath(followCameraPath))

        cameraPrim = stage.GetPrimAtPath(followCameraPath)
        self.assertTrue(cameraPrim.HasAPI(PhysxSchema.PhysxCameraFollowLookAPI))

        #
        # drone camera
        #

        droneCameraPath = "/DroneCamera"
        self.assertFalse(stage.GetPrimAtPath(droneCameraPath))

        PhysXAddDroneCameraCommand.execute(vehiclePaths[0], droneCameraPath)

        self.assertTrue(stage.GetPrimAtPath(droneCameraPath))
        omni.kit.undo.undo()
        self.assertFalse(stage.GetPrimAtPath(droneCameraPath))
        omni.kit.undo.redo()
        self.assertTrue(stage.GetPrimAtPath(droneCameraPath))

        cameraPrim = stage.GetPrimAtPath(droneCameraPath)
        self.assertTrue(cameraPrim.HasAPI(PhysxSchema.PhysxCameraDroneAPI))

        await self.new_stage()

    #
    # Test adding and removing cameras while the sim is running.
    #
    async def test_dynamic_camera_creation(self):
        stage = await self.new_stage()
        
        # Create one vehicle to set up the simulation.
        vehiclePaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            1,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_STANDARD,
            vehiclePathsOut=vehiclePaths,
        )

        vehiclePath = vehiclePaths[0]
        vehiclePrim = stage.GetPrimAtPath(vehiclePath)

        # Run the simulation to make sure the vehicle moves.
        vehicleController = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)
        vehicleController.GetAcceleratorAttr().Set(1.0)
        vehicleController.GetSteerAttr().Set(1.0)

        self._prepare_for_simulation()

        numberOfCameras = 30
        cameraPath = ""
                
        random.seed(0)

        for i in range(numberOfCameras):
            p = random.uniform(0.0, 1.0)

            if p > 0.2:
                if random.randint(0, 1):
                    # Create a follow camera
                    cameraPath = vehiclePath + "/FollowLookCamera" + str(i + 1)
                    self.assertFalse(stage.GetPrimAtPath(cameraPath))

                    PhysXAddFollowLookCameraCommand.execute(vehiclePath, cameraPath)
                    self.assertTrue(stage.GetPrimAtPath(cameraPath))
                else:
                    # Create a drone camera
                    cameraPath = vehiclePath + "/DroneCamera" + str(i + 1)
                    self.assertFalse(stage.GetPrimAtPath(cameraPath))

                    PhysXAddDroneCameraCommand.execute(vehiclePath, cameraPath)
                    self.assertTrue(stage.GetPrimAtPath(cameraPath))

                self._simulate_one_frame()

            else:
                # Or remove one.
                stage.RemovePrim(cameraPath)

                self._simulate_one_frame()

        await self.new_stage()

    #
    # Camera tests ensure cameras end up in their expected positions.
    #
    async def test_vehicle_cameras(self):
        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_STANDARD,
            vehiclePathsOut=vehiclePaths,
        )

        prim = stage.GetPrimAtPath(vehiclePaths[0])
        vehicleController = PhysxSchema.PhysxVehicleControllerAPI(prim)
        vehicleController.GetSteerAttr().Set(1.0)
        vehicleController.GetAcceleratorAttr().Set(1.0)
 
        # create the cameras
        cameraPath = "/TestFollowLookCamera"
        UsdGeom.Camera.Define(stage, cameraPath)
        followCameraPrim = stage.GetPrimAtPath(cameraPath)

        cameraFollowLookApi = PhysxSchema.PhysxCameraFollowLookAPI.Apply(followCameraPrim)
        cameraApi = PhysxSchema.PhysxCameraAPI(cameraFollowLookApi)
        cameraApi.GetPhysxCameraSubjectRel().ClearTargets(True)
        cameraApi.GetPhysxCameraSubjectRel().AddTarget(vehiclePaths[0])

        positionOffset = Gf.Vec3f(0, 0, 0)
        cameraPositionTC = Gf.Vec3f(0.5, 0.1, 0.5)
        lookPositionTC = Gf.Vec3f(0.2, 0.5, 0.2)
 
        cameraFollowApi = PhysxSchema.PhysxCameraFollowAPI(cameraFollowLookApi)
        cameraFollowApi.CreateYawAngleAttr().Set(20.0)
        cameraFollowApi.CreatePitchAngleAttr().Set(15.0)
        cameraFollowApi.CreatePitchAngleTimeConstantAttr().Set(0.2)
        cameraFollowApi.CreateSlowSpeedPitchAngleScaleAttr().Set(0.5)
        cameraFollowApi.CreateSlowPitchAngleSpeedAttr().Set(1000.0)
        cameraFollowApi.CreateVelocityNormalMinSpeedAttr().Set(600.0)
        cameraFollowApi.CreateFollowMinSpeedAttr().Set(3.0)
        cameraFollowApi.CreateFollowMinDistanceAttr().Set(15.0)
        cameraFollowApi.CreateFollowMaxSpeedAttr().Set(30.0)
        cameraFollowApi.CreateFollowMaxDistanceAttr().Set(10.0)
        cameraFollowApi.CreateYawRateTimeConstantAttr().Set(0.2)
        cameraFollowApi.CreateFollowTurnRateGainAttr().Set(0.2)
        cameraFollowApi.CreateCameraPositionTimeConstantAttr().Set(cameraPositionTC)
        cameraFollowApi.CreatePositionOffsetAttr().Set(positionOffset)
        cameraFollowApi.CreateLookAheadMinSpeedAttr().Set(0.0)
        cameraFollowApi.CreateLookAheadMinDistanceAttr().Set(0.0)
        cameraFollowApi.CreateLookAheadMaxSpeedAttr().Set(20.0)
        cameraFollowApi.CreateLookAheadMaxDistanceAttr().Set(5.0)
        cameraFollowApi.CreateLookAheadTurnRateGainAttr().Set(0.2)
        cameraFollowApi.CreateLookPositionHeightAttr().Set(0.5)
        cameraFollowApi.CreateLookPositionTimeConstantAttr().Set(lookPositionTC)

        cameraFollowLookApi.CreateDownHillGroundAngleAttr().Set(-45.0)
        cameraFollowLookApi.CreateDownHillGroundPitchAttr().Set(10.0)
        cameraFollowLookApi.CreateUpHillGroundAngleAttr().Set(45.0)
        cameraFollowLookApi.CreateUpHillGroundPitchAttr().Set(-10.0)
        cameraFollowLookApi.CreateVelocityBlendTimeConstantAttr().Set(0.1)
        cameraFollowLookApi.CreateFollowReverseSpeedAttr().Set(15.0)
        cameraFollowLookApi.CreateFollowReverseDistanceAttr().Set(30.0)

        cameraPath = "/TestDroneCamera"
        UsdGeom.Camera.Define(stage, cameraPath)
        droneCameraPrim = stage.GetPrimAtPath(cameraPath)

        droneCameraApi = PhysxSchema.PhysxCameraDroneAPI.Apply(droneCameraPrim)
        cameraApiDrone = PhysxSchema.PhysxCameraAPI(droneCameraPrim)
        cameraApiDrone.GetPhysxCameraSubjectRel().ClearTargets(True)
        cameraApiDrone.GetPhysxCameraSubjectRel().AddTarget(vehiclePaths[0])

        positionOffset = Gf.Vec3f(0, 0, 0)

        droneCameraApi.CreateFollowHeightAttr().Set(15.0)
        droneCameraApi.CreateFollowDistanceAttr().Set(30.0)
        droneCameraApi.CreateMaxDistanceAttr().Set(100.0)
        droneCameraApi.CreateMaxSpeedAttr().Set(20.0)
        droneCameraApi.CreateHorizontalVelocityGainAttr().Set(1.0)
        droneCameraApi.CreateVerticalVelocityGainAttr().Set(1.0)
        droneCameraApi.CreateFeedForwardVelocityGainAttr().Set(0.1)
        droneCameraApi.CreateVelocityFilterTimeConstantAttr().Set(1.0)
        droneCameraApi.CreateRotationFilterTimeConstantAttr().Set(0.2)
        droneCameraApi.CreatePositionOffsetAttr().Set(positionOffset)

        secondsToRun = 5.0 + self._get_time_step()
        # moving to in-memory-stage test somehow resulted in a tiny difference after one frame
        # but that was enough to cause the vehicle to start driving one frame later than previously.
        self._simulate_with_prep(secondsToRun)

        # To see where the vehicle ends up, uncomment this line.
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        # Follow look camera
        xformable = UsdGeom.Xformable(followCameraPrim)

        endGlobalPose = xformable.ComputeLocalToWorldTransform(0)
        endPos = endGlobalPose.ExtractTranslation()
        endQuat = endGlobalPose.ExtractRotation()

        expectedPos = (20.33, 2.55, -0.37)
        expectedQuatAxis = (-0.086, 0.996, 0.029)
        expectedQuatAngle = 36.86

        errTol = 1.0
        self.assertLess(math.fabs(endPos[0] - expectedPos[0]), errTol)
        self.assertLess(math.fabs(endPos[1] - expectedPos[1]), errTol)
        self.assertLess(math.fabs(endPos[2] - expectedPos[2]), errTol)

        errTol = 1.0
        self.assertLess(math.fabs(endQuat.angle - expectedQuatAngle), errTol)

        errTol = 0.1
        self.assertLess(math.fabs(endQuat.axis[0] - expectedQuatAxis[0]), errTol)
        self.assertLess(math.fabs(endQuat.axis[1] - expectedQuatAxis[1]), errTol)
        self.assertLess(math.fabs(endQuat.axis[2] - expectedQuatAxis[2]), errTol)

        # Drone camera
        xformable = UsdGeom.Xformable(droneCameraPrim)

        endGlobalPose = xformable.ComputeLocalToWorldTransform(0)
        endPos = endGlobalPose.ExtractTranslation()
        endQuat = endGlobalPose.ExtractRotation()
        
        expectedPos = (-0.53, 16.000, -30.377)
        expectedQuatAxis = (0.093, 0.946, 0.311)
        expectedQuatAngle = 211.566

        errTol = 1.0
        self.assertLess(math.fabs(endPos[0] - expectedPos[0]), errTol)
        self.assertLess(math.fabs(endPos[1] - expectedPos[1]), errTol)
        self.assertLess(math.fabs(endPos[2] - expectedPos[2]), errTol)

        errTol = 1.0
        self.assertLess(math.fabs(endQuat.angle - expectedQuatAngle), errTol)

        errTol = 0.1
        self.assertLess(math.fabs(endQuat.axis[0] - expectedQuatAxis[0]), errTol)
        self.assertLess(math.fabs(endQuat.axis[1] - expectedQuatAxis[1]), errTol)
        self.assertLess(math.fabs(endQuat.axis[2] - expectedQuatAxis[2]), errTol)

        await self.new_stage()
