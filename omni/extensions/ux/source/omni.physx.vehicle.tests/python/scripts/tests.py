# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import asyncio
import math
import carb
import carb.input
from carb.eventdispatcher import get_eventdispatcher
from carb.input import KeyboardEventType, KeyboardInput, GamepadInput
import os
import random
import platform
from pxr import Gf, UsdGeom, UsdUtils, UsdPhysics, PhysxSchema, PhysicsSchemaTools
import omni.kit.test
import omni.kit.undo
import omni.kit.app
import omni.appwindow
import omni.usd
import omni.kit.stage_templates
import omni.physxgraph
from omni.physx import get_physx_simulation_interface
from omni.physx.bindings._physx import (
    VEHICLE_AUTOMATIC_TRANSMISSION_GEAR_VALUE,

    VEHICLE_WHEEL_STATE_LOCAL_POSE_POSITION,
    VEHICLE_WHEEL_STATE_LOCAL_POSE_QUATERNION,
    VEHICLE_WHEEL_STATE_ROTATION_SPEED,
    VEHICLE_WHEEL_STATE_ROTATION_ANGLE,
    VEHICLE_WHEEL_STATE_STEER_ANGLE,
    VEHICLE_WHEEL_STATE_GROUND_PLANE,
    VEHICLE_WHEEL_STATE_GROUND_ACTOR,
    VEHICLE_WHEEL_STATE_GROUND_SHAPE,
    VEHICLE_WHEEL_STATE_GROUND_MATERIAL,
    VEHICLE_WHEEL_STATE_GROUND_HIT_POSITION,
    VEHICLE_WHEEL_STATE_SUSPENSION_JOUNCE,
    VEHICLE_WHEEL_STATE_SUSPENSION_FORCE,
    VEHICLE_WHEEL_STATE_IS_ON_GROUND,
    VEHICLE_WHEEL_STATE_TIRE_FRICTION,
    VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_SLIP,
    VEHICLE_WHEEL_STATE_TIRE_LATERAL_SLIP,
    VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_DIRECTION,
    VEHICLE_WHEEL_STATE_TIRE_LATERAL_DIRECTION,
    VEHICLE_WHEEL_STATE_TIRE_FORCE,

    VEHICLE_DRIVE_STATE_ACCELERATOR,
    VEHICLE_DRIVE_STATE_BRAKE0,
    VEHICLE_DRIVE_STATE_BRAKE1,
    VEHICLE_DRIVE_STATE_STEER,
    VEHICLE_DRIVE_STATE_CLUTCH,
    VEHICLE_DRIVE_STATE_CURRENT_GEAR,
    VEHICLE_DRIVE_STATE_TARGET_GEAR,
    VEHICLE_DRIVE_STATE_GEAR_SWITCH_TIME,
    VEHICLE_DRIVE_STATE_AUTOBOX_TIME_SINCE_LAST_SHIFT,
    VEHICLE_DRIVE_STATE_ENGINE_ROTATION_SPEED,
    VEHICLE_DRIVE_STATE_AUTOMATIC_TRANSMISSION,

    SETTING_COLLISION_APPROXIMATE_CYLINDERS,
)
from omni.physxvehicle.scripts.wizards import physxVehicleWizard as VehicleWizard
from omni.physxvehicle.scripts.helpers import Factory
from omni.physxvehicle.scripts.helpers.UnitScale import UnitScale
from omni.physxvehicle.scripts.commands import *
import unittest
from omni.physxtests.utils.physicsBase import (
    PhysicsMemoryStageBaseAsyncTestCase,
    PhysicsKitStageAsyncTestCase,
    TestCategory
)
from omni.physx.scripts.physicsUtils import add_rigid_box, add_collision_to_collision_group
from omni.physx.scripts.utils import (
    get_custom_metadata,
    set_custom_metadata
)
from omni.physx.bindings._physx import (
    SETTING_UPDATE_TO_USD,
    SETTING_UPDATE_VELOCITIES_TO_USD,
)


AUTOMATIC_GEAR = VEHICLE_AUTOMATIC_TRANSMISSION_GEAR_VALUE
FACTORY_PLANE_SUBPATH = "/GroundPlane"
FACTORY_COLLISION_PLANE_SUBPATH = FACTORY_PLANE_SUBPATH + "/CollisionPlane"
FACTORY_SCENE_SUBPATH = "/PhysicsScene"

gPhysXInterface = None
gPhysXSceneQueryInterface = None
gPhysXUnitTestInterface = None
gPhysXVehicleInterface = None
gPhysXVehicleTestingInterface = None


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


def setPhysxSceneQueryInterface(physxSceneQueryInterface):
    global gPhysXSceneQueryInterface
    gPhysXSceneQueryInterface = physxSceneQueryInterface

def clearPhysxSceneQueryInterface():
    global gPhysXSceneQueryInterface
    gPhysXSceneQueryInterface = None


def setPhysxUnitTestInterface(physxUnitTestInterface):
    global gPhysXUnitTestInterface
    gPhysXUnitTestInterface = physxUnitTestInterface

def clearPhysxUnitTestInterface():
    global gPhysXUnitTestInterface
    gPhysXUnitTestInterface = None


def setPhysxVehicleInterface(physxVehicleInterface):
    global gPhysXVehicleInterface
    gPhysXVehicleInterface = physxVehicleInterface

def clearPhysxVehicleInterface():
    global gPhysXVehicleInterface
    gPhysXVehicleInterface = None


def setPhysxVehicleTestingInterface(physxVehicleTestingInterface):
    global gPhysXVehicleTestingInterface
    gPhysXVehicleTestingInterface = physxVehicleTestingInterface

def clearPhysxVehicleTestingInterface():
    global gPhysXVehicleTestingInterface
    gPhysXVehicleTestingInterface = None

def get_unit_scale(stage):
    metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)
    lengthScale = 1.0 / metersPerUnit
    kilogramsPerUnit = UsdPhysics.GetStageKilogramsPerUnit(stage)
    massScale = 1.0 / kilogramsPerUnit
    return UnitScale(lengthScale, massScale)

class PhysXVehicleTestBase:
    def setUpVehicleTestSettings(self):
        # note: these settings are per-stage, this needs to be called after a stage is opened
        settings = carb.settings.get_settings()
        settings.set_bool(SETTING_UPDATE_TO_USD, False)
        settings.set_bool(SETTING_UPDATE_VELOCITIES_TO_USD, False)

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

    def _get_delta_angle_radians(self, dirA, dirB):
        angle = math.acos(dirA.GetDot(dirB))
        return angle

    def _get_delta_angle_degree(self, dirA, dirB):
        angle = (self._get_delta_angle_radians(dirA, dirB) * 180) / math.pi
        return angle

    def _rotate_dir(self, dir, transform):
        # dir is expected to be a GfVec3
        # transform is expected to be a GfMatrix4
        quat = transform.ExtractRotation()
        rotatedDir = quat.TransformDir(dir)
        return rotatedDir

    def _local_dir_to_world(self, dir, xformable):
        # dir is expected to be a GfVec3
        return self._rotate_dir(dir, xformable.ComputeLocalToWorldTransform(0))

    def _get_tire(self, wheelAttPrim, stage):
        if (wheelAttPrim.HasAPI(PhysxSchema.PhysxVehicleTireAPI)):
            return PhysxSchema.PhysxVehicleTireAPI(wheelAttPrim)
        wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(wheelAttPrim)
        tirePath = wheelAttAPI.GetTireRel().GetTargets()[0]
        tirePrim = stage.GetPrimAtPath(tirePath)
        tire = PhysxSchema.PhysxVehicleTireAPI(tirePrim)
        return tire

    def _get_tire_friction_table(self, wheelAttPrim, stage):
        if (wheelAttPrim.HasAPI(PhysxSchema.PhysxVehicleTireAPI)):
            tireAPI = PhysxSchema.PhysxVehicleTireAPI(wheelAttPrim)
        else:
            wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(wheelAttPrim)
            tirePath = wheelAttAPI.GetTireRel().GetTargets()[0]
            tirePrim = stage.GetPrimAtPath(tirePath)
            tireAPI = PhysxSchema.PhysxVehicleTireAPI(tirePrim)

        frictionTablePath = tireAPI.GetFrictionTableRel().GetTargets()[0]
        frictionTablePrim = stage.GetPrimAtPath(frictionTablePath)
        return PhysxSchema.PhysxVehicleTireFrictionTable(frictionTablePrim)

    def _get_suspension(self, wheelAttPrim, stage):
        if (wheelAttPrim.HasAPI(PhysxSchema.PhysxVehicleSuspensionAPI)):
            return PhysxSchema.PhysxVehicleSuspensionAPI(wheelAttPrim)
        wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(wheelAttPrim)
        suspPath = wheelAttAPI.GetSuspensionRel().GetTargets()[0]
        suspPrim = stage.GetPrimAtPath(suspPath)
        susp = PhysxSchema.PhysxVehicleSuspensionAPI(suspPrim)
        return susp

    def _get_wheel(self, wheelAttPrim, stage):
        if (wheelAttPrim.HasAPI(PhysxSchema.PhysxVehicleWheelAPI)):
            return PhysxSchema.PhysxVehicleWheelAPI(wheelAttPrim)
        wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(wheelAttPrim)
        wheelPath = wheelAttAPI.GetWheelRel().GetTargets()[0]
        wheelPrim = stage.GetPrimAtPath(wheelPath)
        wheel = PhysxSchema.PhysxVehicleWheelAPI(wheelPrim)
        return wheel

    def _get_wheel_radius_and_width(self, wheelAttPrim, stage):
        wheel = self._get_wheel(wheelAttPrim, stage)
        wheelRadius = wheel.GetRadiusAttr().Get()
        wheelWidth = wheel.GetWidthAttr().Get()
        return (wheelRadius, wheelWidth)

    def _get_drive(self, vehiclePrim, stage):
        if (vehiclePrim.HasAPI(PhysxSchema.PhysxVehicleDriveBasicAPI)):
            return PhysxSchema.PhysxVehicleDriveBasicAPI(vehiclePrim)
        elif (vehiclePrim.HasAPI(PhysxSchema.PhysxVehicleDriveStandardAPI)):
            return PhysxSchema.PhysxVehicleDriveStandardAPI(vehiclePrim)

        vehicleAPI = PhysxSchema.PhysxVehicleAPI(vehiclePrim)
        drivePath = vehicleAPI.GetDriveRel().GetTargets()[0]
        drivePrim = stage.GetPrimAtPath(drivePath)
        if (drivePrim.HasAPI(PhysxSchema.PhysxVehicleDriveBasicAPI)):
            return PhysxSchema.PhysxVehicleDriveBasicAPI(drivePrim)
        else:
            return PhysxSchema.PhysxVehicleDriveStandardAPI(drivePrim)

    def _get_engine(self, drivePrim, stage):
        if (drivePrim.HasAPI(PhysxSchema.PhysxVehicleEngineAPI)):
            return PhysxSchema.PhysxVehicleEngineAPI(drivePrim)

        driveStandard = PhysxSchema.PhysxVehicleDriveStandardAPI(drivePrim)
        enginePath = driveStandard.GetEngineRel().GetTargets()[0]
        enginePrim = stage.GetPrimAtPath(enginePath)
        return PhysxSchema.PhysxVehicleEngineAPI(enginePrim)

    def _get_gears(self, drivePrim, stage):
        driveStandard = PhysxSchema.PhysxVehicleDriveStandardAPI(drivePrim)
        gearsPath = driveStandard.GetGearsRel().GetTargets()[0]
        gearsPrim = stage.GetPrimAtPath(gearsPath)
        return PhysxSchema.PhysxVehicleGearsAPI(gearsPrim)

    def _get_sim_stats(self):
        global gPhysXUnitTestInterface
        return gPhysXUnitTestInterface.get_physics_stats()

    def _check_sim_stats(
        self,
        simStats,
        numDynRig,
        numStatRig,
        numKinRig,
        numSphereShapes,
        numBoxShapes,
        numCapsShapes,
        numCylShapes,
        numConvexShapes,
        numTriShapes,
        numPlShapes,
        numConeShapes,
    ):
        self.assertEqual(simStats["numDynamicRigids"], numDynRig)
        self.assertEqual(simStats["numStaticRigids"], numStatRig)
        self.assertEqual(simStats["numKinematicBodies"], numKinRig)
        self.assertEqual(simStats["numSphereShapes"], numSphereShapes)
        self.assertEqual(simStats["numBoxShapes"], numBoxShapes)
        self.assertEqual(simStats["numCapsuleShapes"], numCapsShapes)
        self.assertEqual(simStats["numCylinderShapes"], numCylShapes)
        self.assertEqual(simStats["numConvexShapes"], numConvexShapes)
        self.assertEqual(simStats["numTriMeshShapes"], numTriShapes)
        self.assertEqual(simStats["numPlaneShapes"], numPlShapes)
        self.assertEqual(simStats["numConeShapes"], numConeShapes)

    def _remove_factory_ground_plane_and_set_gravity_zero(self):
        stage = self.get_stage()
        rootPath = str(stage.GetDefaultPrim().GetPath())
        scenePrim = stage.GetPrimAtPath(rootPath + FACTORY_SCENE_SUBPATH)
        scene = UsdPhysics.Scene(scenePrim)
        scene.GetGravityMagnitudeAttr().Set(0.0)

        # remove the ground plane
        groundPlanePath = rootPath + FACTORY_PLANE_SUBPATH
        stage.RemovePrim(groundPlanePath)

    def _set_vehicle_speed(self, stage, vehiclePrim, wheelAttachmentPaths, speed,
        speedDir, physxInterface):
        # speedDir is expected to be a Gf.Vec3f

        speedVec = speedDir * speed

        rigidBodyAPI = UsdPhysics.RigidBodyAPI(vehiclePrim)
        rigidBodyAPI.GetVelocityAttr().Set(speedVec)

        for i in range(len(wheelAttachmentPaths)):
            wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[i])
            (wheelRadius, wheelWidth) = self._get_wheel_radius_and_width(wheelAttPrim, stage)
            rotationSpeed = speed / wheelRadius  # assuming free rolling

            physxInterface.set_wheel_rotation_speed(wheelAttachmentPaths[i], rotationSpeed)

    def _set_xform_pos(self, prim, pos):
        # pos is expected to be a Gf.Vec3f
        prim.GetAttribute("xformOp:translate").Set(pos)

    def _get_xform_pos(self, prim):
        pos = prim.GetAttribute("xformOp:translate").Get()
        return pos

    def _set_xform_orient(self, prim, orient):
        # orient is expected to be a Gf.Quatf
        prim.GetAttribute("xformOp:orient").Set(orient)

    def _get_xform_orient(self, prim):
        orient = prim.GetAttribute("xformOp:orient").Get()
        return orient


class PhysXVehicleTestKitStage(PhysicsKitStageAsyncTestCase, PhysXVehicleTestBase):

    gamepadCameraControlSettingID = "/persistent/app/omniverse/gamepadCameraControl"

    async def setUp(self):
        await super().setUp()

        # to inject gamepad input events for some tests, the gamepad camera control has to
        # be disabled
        settings = carb.settings.get_settings()
        self._gamepad_camera_control = settings.get_as_bool(self.gamepadCameraControlSettingID)
        settings.set_bool(self.gamepadCameraControlSettingID, False)

    async def tearDown(self):
        settings = carb.settings.get_settings()
        settings.set_bool(self.gamepadCameraControlSettingID, self._gamepad_camera_control)

        await super().tearDown()

    async def new_stage(self, def_up_and_mpu=True):
        await super().new_stage(def_up_and_mpu)
        super().setUpVehicleTestSettings()
        return self._stage

    def _prepare_for_simulation(self):
        global gPhysXInterface
        gPhysXInterface.start_simulation()

        stage = self.get_stage()
        stageId = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        global gPhysXVehicleInterface
        gPhysXVehicleInterface.attach_stage(stageId, False)

    def _simulate_one_frame(self, elapsedTime=None):
        if (elapsedTime is None):
            timeStep = self._get_time_step()
        else:
            timeStep = elapsedTime
        currentTime = 0.0

        global gPhysXVehicleInterface
        gPhysXVehicleInterface.update_controllers(timeStep)

        global gPhysXInterface
        gPhysXInterface.update_simulation(timeStep, currentTime)

    def _simulate(self, secondsToRun):
        timeStep = self._get_time_step()
        targetIterationCount = math.ceil(secondsToRun / timeStep)
        currentTime = 0.0

        global gPhysXVehicleInterface
        global gPhysXInterface
        for i in range(targetIterationCount):
            gPhysXVehicleInterface.update_controllers(timeStep)
            gPhysXInterface.update_simulation(timeStep, currentTime)
            currentTime += timeStep

    async def _open_usd(self, filename):
        data_path = "../../../../data/tests"
        schema_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, data_path)))
        open_file_name = schema_folder + "/" + filename
        open_file_name = open_file_name.replace("\\", "/")

        await omni.usd.get_context().open_stage_async(open_file_name + ".usda")

        usd_context = omni.usd.get_context()
        self.assertIn(filename, usd_context.get_stage_url())

    async def _load_usd_file(
        self,
        fileName,
        numDynRig,
        numStatRig,
        numKinRig,
        numSphereShapes,
        numBoxShapes,
        numCapsShapes,
        numCylShapes,
        numConvexShapes,
        numTriShapes,
        numPlShapes,
        numConeShapes,
    ):
        await self._open_usd(fileName)

        self.set_stage(omni.usd.get_context().get_stage())  # else preparation fails as get_stage() returns None
        self._simulate_one_frame_with_prep()
        simStats = self._get_sim_stats()

        self._check_sim_stats(
            simStats,
            numDynRig,
            numStatRig,
            numKinRig,
            numSphereShapes,
            numBoxShapes,
            numCapsShapes,
            numCylShapes,
            numConvexShapes,
            numTriShapes,
            numPlShapes,
            numConeShapes,
        )

        global gPhysXVehicleTestingInterface
        stage = self.get_stage()
        for prim in stage.Traverse():
            if prim.HasAPI(PhysxSchema.PhysxVehicleAPI):
                self.assertTrue(gPhysXVehicleTestingInterface.does_vehicle_exist(prim.GetPath().pathString))

        await self.new_stage()

    #
    # make sure the example USD file can be loaded and creates the expected number of objects
    #
    async def test_USD(self):
        await self._load_usd_file("Vehicle_Schema_Tests/Vehicle", 1, 1, 0, 0, 0, 0, 4, 1, 0, 1, 0)

    #
    # The Kit stage tests expect a certain time step. Verify this matches with the default value for tests.
    #
    async def test_expected_timestep(self):
        stage = await self.new_stage()

        expectedTimeStepsPerSecond = 60

        #
        # check that factory sets to 60 steps per second
        #

        vehicleCount = 1
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount
        )

        rootPath = str(stage.GetDefaultPrim().GetPath())
        scenePrim = stage.GetPrimAtPath(rootPath + FACTORY_SCENE_SUBPATH)

        self.assertTrue(scenePrim)
        self.assertTrue(scenePrim.HasAPI(PhysxSchema.PhysxSceneAPI))
        physxSceneAPI = PhysxSchema.PhysxSceneAPI(scenePrim)

        timeStepsPerSecond = physxSceneAPI.GetTimeStepsPerSecondAttr().Get()

        self.assertEqual(timeStepsPerSecond, expectedTimeStepsPerSecond)

        stage = await self.new_stage()

        #
        # check that wizard sets to 60 steps per second
        #

        vehicleData = VehicleWizard.VehicleData(get_unit_scale(stage),
            VehicleWizard.VehicleData.AXIS_Y, VehicleWizard.VehicleData.AXIS_Z)

        # Place the vehicle under the default prim.
        vehicleData.rootVehiclePath = rootPath + VehicleWizard.VEHICLE_ROOT_BASE_PATH
        vehicleData.rootSharedPath = rootPath + VehicleWizard.SHARED_DATA_ROOT_BASE_PATH
        vehiclePath = vehicleData.rootVehiclePath + "/Vehicle"

        (success, (messageList, _)) = PhysXVehicleWizardCreateCommand.execute(vehicleData)
        self.assertTrue(success)
        self.assertFalse(messageList)  # python style guide to check for empty list

        scenePrim = stage.GetPrimAtPath(vehicleData.rootSharedPath + VehicleWizard.DEFAULT_SCENE_PATH)

        self.assertTrue(scenePrim)
        self.assertTrue(scenePrim.HasAPI(PhysxSchema.PhysxSceneAPI))
        physxSceneAPI = PhysxSchema.PhysxSceneAPI(scenePrim)

        timeStepsPerSecond = physxSceneAPI.GetTimeStepsPerSecondAttr().Get()

        self.assertEqual(timeStepsPerSecond, expectedTimeStepsPerSecond)

        stage = await self.new_stage()

    #
    # when the simulation is stopped (not paused), the transforms before the simulation started should
    # get restored
    #
    # note: this is a kit stage test as storing the initial transforms is a thing of omni.physx in a
    #       kit stage
    #
    async def test_restore_initial_wheel_transforms(self):
        stage = await self.new_stage()
        settings = carb.settings.get_settings()
        # restoring the value requires update to USD
        settings.set_bool(SETTING_UPDATE_TO_USD, True)

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
        )

        wheelPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][0])
        xformable = UsdGeom.Xformable(wheelPrim)
        startGlobalPose = xformable.ComputeLocalToWorldTransform(0)
        startPos = startGlobalPose.ExtractTranslation()
        startQuat = startGlobalPose.ExtractRotation()

        prim = stage.GetPrimAtPath(vehiclePaths[0])
        vehicleController = PhysxSchema.PhysxVehicleControllerAPI(prim)
        vehicleController.GetSteerAttr().Set(1.0)

        secondsToRun = 1.0
        self._simulate_with_prep(secondsToRun)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        endGlobalPose = xformable.ComputeLocalToWorldTransform(0)
        endPos = endGlobalPose.ExtractTranslation()
        endQuat = endGlobalPose.ExtractRotation()

        wheelDirStart = startQuat.TransformDir((0, 0, 1))
        wheelDirEnd = endQuat.TransformDir((0, 0, 1))
        deltaAngle = self._get_delta_angle_degree(wheelDirStart, wheelDirEnd)
        self.assertGreater(math.fabs(deltaAngle), 10)  # expect wheel to steer

        global gPhysXInterface
        gPhysXInterface.reset_simulation()

        resetGlobalPose = xformable.ComputeLocalToWorldTransform(0)
        resetPos = resetGlobalPose.ExtractTranslation()
        resetQuat = resetGlobalPose.ExtractRotation()

        errTol = 0.001
        self.assertLess(math.fabs(startPos[0] - resetPos[0]), errTol)
        self.assertLess(math.fabs(startPos[1] - resetPos[1]), errTol)
        self.assertLess(math.fabs(startPos[2] - resetPos[2]), errTol)

        self.assertLess(math.fabs(startQuat.axis[0] - resetQuat.axis[0]), errTol)
        self.assertLess(math.fabs(startQuat.axis[1] - resetQuat.axis[1]), errTol)
        self.assertLess(math.fabs(startQuat.axis[2] - resetQuat.axis[2]), errTol)
        self.assertLess(math.fabs(startQuat.angle - resetQuat.angle), errTol)

    #
    # when the simulation is stopped (not paused), the controller attributes before the simulation started
    # should get restored
    #
    # note: this is a kit stage test as storing the initial attributes is a thing of omni.physx in a
    #       kit stage
    #
    async def test_restore_initial_controller_attributes(self):
        settings = carb.settings.get_settings()

        for i in range(3):
            stage = await self.new_stage()
            # restoring the value requires update to USD
            settings.set_bool(SETTING_UPDATE_TO_USD, True)

            if (i < 2):
                driveModeIn = Factory.DRIVE_STANDARD
            else:
                driveModeIn = Factory.DRIVE_NONE

            if (i == 1):
                useDeprecatedAPIListIn = [True]
            else:
                useDeprecatedAPIListIn = None

            vehicleCount = 1
            vehiclePaths = []
            wheelAttachmentPaths = []
            Factory.create4WheeledCarsScenario(
                stage,
                1.0,
                vehicleCount,
                driveMode=driveModeIn,
                vehiclePathsOut=vehiclePaths,
                wheelAttachmentPathsOut=wheelAttachmentPaths,
                useDeprecatedAPIList=useDeprecatedAPIListIn
            )

            if (i < 2):
                prim = stage.GetPrimAtPath(vehiclePaths[0])
                vehicleController = PhysxSchema.PhysxVehicleControllerAPI(prim)
            else:
                wheelControllerList = []
                for path in wheelAttachmentPaths[0]:
                    prim = stage.GetPrimAtPath(path)
                    wheelControllerList.append(PhysxSchema.PhysxVehicleWheelControllerAPI(prim))

            self._prepare_for_simulation()

            if (i < 2):
                vehicleController.GetTargetGearAttr().Set(-1)
                vehicleController.GetAcceleratorAttr().Set(1.0)
                if (i == 1):
                    vehicleController.GetBrakeAttr().Set(0.1)
                    vehicleController.GetHandbrakeAttr().Set(0.1)
                    vehicleController.GetSteerLeftAttr().Set(0.7)
                    vehicleController.GetSteerRightAttr().Set(0.3)
                else:
                    vehicleController.GetBrake0Attr().Set(0.1)
                    vehicleController.GetBrake1Attr().Set(0.1)
                    vehicleController.GetSteerAttr().Set(0.7)
            else:
                for wheelController in wheelControllerList:
                    wheelController.GetDriveTorqueAttr().Set(500.0)
                    wheelController.GetBrakeTorqueAttr().Set(100.0)
                    wheelController.GetSteerAngleAttr().Set(math.pi / 90.0)

            self._simulate_one_frame()

            global gPhysXInterface
            gPhysXInterface.reset_simulation()

            if (i < 2):
                self.assertEqual(vehicleController.GetTargetGearAttr().Get(), AUTOMATIC_GEAR)
                self.assertEqual(vehicleController.GetAcceleratorAttr().Get(), 0)
                if (i == 1):
                    self.assertEqual(vehicleController.GetBrakeAttr().Get(), 0)
                    self.assertEqual(vehicleController.GetHandbrakeAttr().Get(), 0)
                    self.assertEqual(vehicleController.GetSteerLeftAttr().Get(), 0)
                    self.assertEqual(vehicleController.GetSteerRightAttr().Get(), 0)
                else:
                    self.assertEqual(vehicleController.GetBrake0Attr().Get(), 0)
                    self.assertEqual(vehicleController.GetBrake1Attr().Get(), 0)
                    self.assertEqual(vehicleController.GetSteerAttr().Get(), 0)
            else:
                for wheelController in wheelControllerList:
                    self.assertEqual(wheelController.GetDriveTorqueAttr().Get(), 0)
                    self.assertEqual(wheelController.GetBrakeTorqueAttr().Get(), 0)
                    self.assertEqual(wheelController.GetSteerAngleAttr().Get(), 0)

        await self.new_stage()

    #
    # Test that the vehicle can enter auto-reverse if enabled and that it does not
    # go into auot-reverse if disabled
    #
    async def _auto_reverse_device_input(self, driveModeIn, useGamepad):

        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=driveModeIn,
            vehiclePathsOut=vehiclePaths,
        )

        prim = stage.GetPrimAtPath(vehiclePaths[0])
        startPos = prim.GetAttribute("xformOp:translate").Get()

        global gPhysXVehicleInterface

        # Auto reverse defaults to True, so turn it off initially. Pressing brake key
        # should not result in the car going into reverse
        gPhysXVehicleInterface.set_auto_reverse_enabled(vehiclePaths[0], False)
        autoReverseEnabled = gPhysXVehicleInterface.get_auto_reverse_enabled(vehiclePaths[0])
        self.assertFalse(autoReverseEnabled)

        #
        # note: gamepad camera control has to be disabled for this to work. This is done in
        #       PhysXVehicleTestKitStage::setUp()
        #
        class GamepadInjector:
            def __init__(self):
                self._inputProvider = carb.input.acquire_input_provider()
                self._gamepad = self._inputProvider.create_gamepad("test_gamepad_logical_name", "test_gamepad_guid")
                self._inputProvider.set_gamepad_connected(self._gamepad, True)

            def tear_down(self):
                self._inputProvider.set_gamepad_connected(self._gamepad, False)
                self._inputProvider.destroy_gamepad(self._gamepad)
                self._inputProvider = None
                self._gamepad = None

            def send_gamepad_event(self, input, value):
                self._inputProvider.update_gamepad(self._gamepad)
                self._inputProvider.buffer_gamepad_event(self._gamepad, input, value)

        class KeyboardInjector:
            def __init__(self):
                self._inputProvider = carb.input.acquire_input_provider()
                self._appwindow = omni.appwindow.get_default_app_window()
                self._keyboard = self._appwindow.get_keyboard()
                self._updateLoopSubs = None
                self._keyInput = None

            def tear_down(self):
                self._keyInput = None
                self._updateLoopSubs = None
                self._inputProvider = None
                self._keyboard = None
                self._appwindow = None

            def set_key_input(self, keyInput):
                self._keyInput = keyInput

            def register_to_update_loop(self):
                self._updateLoopSubs = get_eventdispatcher().observe_event(
                    event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
                    on_event=self._update,
                    observer_name="omni.physx.vehicle.test: test_auto_reverse",
                )

            def unregister_from_update_loop(self):
                self._updateLoopSubs = None

            def _update(self, event):
                # key input is not stateful, thus need to "press" every update
                if (self._keyInput is not None):
                    self._inputProvider.update_keyboard(self._keyboard)
                    # to detect state change like key unpress etc. but probably not really needed for this test

                    self._inputProvider.buffer_keyboard_key_event(self._keyboard, KeyboardEventType.KEY_PRESS,
                        self._keyInput, 0)

        secondsToRun = 1.5
        timeStep = self._get_time_step()
        timeout_per_step = 10 * timeStep
        simStepCount = math.ceil(secondsToRun / timeStep)

        if (useGamepad):
            gamepadInjector = GamepadInjector()
        else:
            keyboardInjector = KeyboardInjector()

        # run through play.
        await self.step(1, 2 * timeout_per_step)
        # the sim is now paused, send the input event
        if (useGamepad):
            gamepadInjector.send_gamepad_event(GamepadInput.LEFT_TRIGGER, 1.0)  # input for braking
        else:
            keyboardInjector.set_key_input(KeyboardInput.DOWN)  # key for braking
            keyboardInjector.register_to_update_loop()

        await omni.kit.app.get_app().next_update_async()

        await self.step(simStepCount, timeout_per_step)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)
        endPos = prim.GetAttribute("xformOp:translate").Get()
        delta = endPos - startPos

        errTol = 0.01
        self.assertLess(math.fabs(delta[0]), errTol)
        self.assertLess(math.fabs(delta[1]), errTol)
        self.assertLess(math.fabs(delta[2]), errTol)

        # Turn auto reverse on. Pressing brake key should result in the car going into reverse.
        # Note that the gamepad signals on state change, thus, brake trigger should still be
        # seen as pressed.
        gPhysXVehicleInterface.set_auto_reverse_enabled(vehiclePaths[0], True)
        autoReverseEnabled = gPhysXVehicleInterface.get_auto_reverse_enabled(vehiclePaths[0])
        self.assertTrue(autoReverseEnabled)

        # run through play
        await self.step(simStepCount, timeout_per_step)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)
        endPos = prim.GetAttribute("xformOp:translate").Get()
        delta = endPos - startPos

        omni.timeline.get_timeline_interface().stop()
        await omni.kit.app.get_app().next_update_async()

        # tear down the input injector before validation to avoid dangling objects if the test fails.
        # Furthermore, making sure this is done after the vehicle stuff has been cleaned up as destroyed
        # gamepads still seem to send connection events when unsubscribeToGamepadConnectionEvents() is
        # called.

        if (useGamepad):
            gamepadInjector.tear_down()
            gamepadInjector = None
        else:
            keyboardInjector.tear_down()
            keyboardInjector = None

        await omni.kit.app.get_app().next_update_async()

        self.assertLess(math.fabs(delta[0]), errTol)
        self.assertLess(math.fabs(delta[1]), errTol)
        self.assertLess(delta[2], -0.3)  # vehicle should have driven backwards

        await self.new_stage()

    #
    # Test that the vehicle can enter auto-reverse if enabled and that it does not
    # go into auot-reverse if disabled
    #
    async def test_auto_reverse_standard_keyboard(self):
        await self._auto_reverse_device_input(Factory.DRIVE_STANDARD, False)

    #
    # Test that the vehicle can enter auto-reverse if enabled and that it does not
    # go into auot-reverse if disabled
    #
    async def test_auto_reverse_basic_keyboard(self):
        await self._auto_reverse_device_input(Factory.DRIVE_BASIC, False)

    #
    # Test that the vehicle can enter auto-reverse if enabled and that it does not
    # go into auot-reverse if disabled
    #
    async def test_auto_reverse_standard_gamepad(self):
        await self._auto_reverse_device_input(Factory.DRIVE_STANDARD, True)

    #
    # Test that the vehicle can enter auto-reverse if enabled and that it does not
    # go into auot-reverse if disabled
    #
    async def test_auto_reverse_basic_gamepad(self):
        await self._auto_reverse_device_input(Factory.DRIVE_BASIC, True)

    #
    # verify the link to the data/audio folder was created and links to the correct folder.
    # verify that the sound files needed in the VehicleAudio.py demo are present.
    #
    async def test_audio_file_location(self):
        stage = await self.new_stage()

        data_path = "../../../../data/audio"

        audio_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, data_path)))
        audio_folder = audio_folder.replace("\\", "/")

        # print("audio_folder = " + audio_folder)
        # print(dir(AudioSchema.Sound))

        # Engine sounds
        self.assertTrue(os.path.exists(audio_folder + "/loww1000.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/loww1500.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/loww2000.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/loww2500.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/loww3000.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/loww3500.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/loww4000.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/loww4500.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/loww5000.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/loww5500.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/loww6000.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/loww6500.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/high1000.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/high1500.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/high2000.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/high2500.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/high3000.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/high3500.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/high4000.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/high4500.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/high5000.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/high5500.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/high6000.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/high6500.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/tireslip.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/driveon10.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/driveon20.wav"))
        self.assertTrue(os.path.exists(audio_folder + "/driveon60.wav"))

        # For now, all of the sound lengths are returning 0.0 seconds. They might not be
        # loaded yet. Playing the simulation first, doesn't seem to help.
        # audioInterface = omni.usd.audio.get_stage_audio_interface()

        # loww1000 = AudioSchema.Sound.Define(stage, "/Vehicle/Audio/loww1000")
        # loww1000.CreateFilePathAttr().Set(audio_folder + "/loww1000.wav")

        # print(audioInterface.get_sound_length(loww1000))
        # self.assertGreater(audioInterface.get_sound_length(loww1000), 0.0)

        await self.new_stage()

    #
    # using attach_stage/detach_stage to step vehicle simulation explicitly and unregister from stage
    # update events.
    #
    async def test_attach_detach_stage(self):

        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths
        )

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])
        vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)
        vehicleControllerAPI.GetAcceleratorAttr().Set(1.0)

        initialPos = vehiclePrim.GetAttribute("xformOp:translate").Get()
        initialOrient = vehiclePrim.GetAttribute("xformOp:orient").Get()
        startPos = initialPos

        didMoveThreshold = 0.03

        stageId = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        global gPhysXInterface
        global gPhysXSimInterface
        global gPhysXVehicleInterface

        gPhysXSimInterface.attach_stage(stageId)
        gPhysXVehicleInterface.attach_stage(stageId, True)

        timeStep = self._get_time_step()
        timeout_per_step = 10 * timeStep
        stepCount = math.floor(0.3 / timeStep)

        for i in range(stepCount):
            gPhysXVehicleInterface.update_controllers(timeStep)
            gPhysXSimInterface.simulate(timeStep, i * timeStep)
            gPhysXSimInterface.fetch_results()

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        pos = vehiclePrim.GetAttribute("xformOp:translate").Get()
        deltaPos = pos - startPos
        self.assertGreater(deltaPos[2], didMoveThreshold)  # the car should have moved
        self.assertLess(math.fabs(deltaPos[1]), 0.001)  # extra check to make sure the thing did not explode

        startPos = pos

        class PhysxStepCounter:
            def __init__(self):
                self.stepCount = 0

            def on_tick(self, dt):
                self.stepCount = self.stepCount + 1

        stepCounter = PhysxStepCounter()
        physxSimEventSubscription = gPhysXInterface.subscribe_physics_step_events(stepCounter.on_tick)

        # run through play. The vehicle should not be affected
        omni.timeline.get_timeline_interface().play()
        for i in range(stepCount):
            await omni.kit.app.get_app().next_update_async()

        omni.timeline.get_timeline_interface().stop()
        await omni.kit.app.get_app().next_update_async()

        self.assertEqual(stepCounter.stepCount, 0)  # the simulation should not run

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)
        pos = vehiclePrim.GetAttribute("xformOp:translate").Get()
        deltaPos = pos - startPos
        self.assertLess(math.fabs(deltaPos[1]), 0.0001)
        self.assertLess(math.fabs(deltaPos[2]), 0.0001)

        gPhysXVehicleInterface.detach_stage(True)
        gPhysXSimInterface.detach_stage()

        startPos = initialPos

        rigidBodyAPI = UsdPhysics.RigidBodyAPI(vehiclePrim)
        rigidBodyAPI.GetVelocityAttr().Set(Gf.Vec3f(0))
        rigidBodyAPI.GetAngularVelocityAttr().Set(Gf.Vec3f(0))
        vehiclePrim.GetAttribute("xformOp:translate").Set(initialPos)
        vehiclePrim.GetAttribute("xformOp:orient").Set(initialOrient)

        # run through play. The vehicle should now be affected
        await self.step(stepCount, timeout_per_step)

        physxSimEventSubscription = None
        stepCounter = None

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)
        pos = vehiclePrim.GetAttribute("xformOp:translate").Get()

        omni.timeline.get_timeline_interface().stop()
        await omni.kit.app.get_app().next_update_async()

        deltaPos = pos - startPos
        self.assertGreater(deltaPos[2], didMoveThreshold)  # the car should have moved
        self.assertLess(math.fabs(deltaPos[1]), 0.001)  # extra check to make sure the thing did not explode

        await self.new_stage()

    #
    # Command to compute suspension frame transforms.
    #
    async def test_command_suspension_frame_transforms_helper(self):
        for i in range(2):
            # 0: adjust using the vehicle prim
            # 1: adjust using the wheel attachment prim

            stage = await self.new_stage()

            vehicleCount = 1
            vehiclePaths = []
            wheelAttachmentPaths = []
            Factory.create4WheeledCarsScenario(
                stage,
                1.0,
                vehicleCount,
                createCollisionShapesForWheels=True,
                driveMode=Factory.DRIVE_NONE,
                vehiclePathsOut=vehiclePaths,
                wheelAttachmentPathsOut=wheelAttachmentPaths
            )

            vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])

            oldSuspFramePosList = []
            oldSuspFrameOrientList = []
            newPosList = []

            rotAngle = math.pi / 6.0
            rotAngleHalf = 0.5 * rotAngle
            newOrient = Gf.Quatf(math.cos(rotAngleHalf), 0.0, math.sin(rotAngleHalf), 0.0)

            # get original attribute values and modify wheel positions/orientations
            for j in range(4):
                wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][j])
                wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(wheelAttPrim)
                oldSuspFramePosList.append(wheelAttAPI.GetSuspensionFramePositionAttr().Get())

                # the factory does not define orientation on the wheel attachment Xforms
                # -> do it now
                identityOrient = Gf.Quatf(1, 0, 0, 0)
                oldSuspFrameOrientList.append(identityOrient)
                xform = UsdGeom.Xform(wheelAttPrim)
                xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(identityOrient)

                wheelTransAttr = wheelAttPrim.GetAttribute("xformOp:translate")
                wheelPos = wheelTransAttr.Get()
                if wheelPos[0] < 0:
                    wheelPos[0] = wheelPos[0] - 1
                else:
                    wheelPos[0] = wheelPos[0] + 1
                newPosList.append(wheelPos)
                wheelTransAttr.Set(wheelPos)

                wheelOrientAttr = wheelAttPrim.GetAttribute("xformOp:orient")
                wheelOrientAttr.Set(newOrient)

            # execute command to run the helper method to adjust wheel simulation transforms
            if i == 0:
                PhysXVehicleSuspensionFrameTransformsAutocomputeCommand.execute(vehiclePaths[0])
            else:
                PhysXVehicleSuspensionFrameTransformsAutocomputeCommand.execute(wheelAttachmentPaths[0][0])

            def checkAttributes(test, wheelAttPath, refSuspFramePosition, refSuspFrameOrientation):
                errTol = 0.001

                wheelAttPrim = stage.GetPrimAtPath(wheelAttPath)
                wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(wheelAttPrim)
                suspFramePosition = wheelAttAPI.GetSuspensionFramePositionAttr().Get()
                suspFrameOrient = wheelAttAPI.GetSuspensionFrameOrientationAttr().Get()

                test.assertLess((suspFramePosition - refSuspFramePosition).GetLength(), errTol)
                test.assertLess((suspFrameOrient - refSuspFrameOrientation).GetLength(), errTol)

            for j in range(4):
                if ((i == 0) or (j == 0)):
                    checkAttributes(self, wheelAttachmentPaths[0][j],
                        newPosList[j], newOrient)
                else:
                    checkAttributes(self, wheelAttachmentPaths[0][j],
                        oldSuspFramePosList[j], oldSuspFrameOrientList[j])

            omni.kit.undo.undo()

            for j in range(4):
                checkAttributes(self, wheelAttachmentPaths[0][j],
                    oldSuspFramePosList[j], oldSuspFrameOrientList[j])

            omni.kit.undo.redo()

            for j in range(4):
                if ((i == 0) or (j == 0)):
                    checkAttributes(self, wheelAttachmentPaths[0][j],
                        newPosList[j], newOrient)
                else:
                    checkAttributes(self, wheelAttachmentPaths[0][j],
                        oldSuspFramePosList[j], oldSuspFrameOrientList[j])

        await self.new_stage()

    #
    # Command to compute suspension frame transforms (scales different than 1 involved).
    #
    async def test_command_suspension_frame_transforms_helper_with_scale(self):
        stage = await self.new_stage()

        identityOrient = Gf.Quatf(1, 0, 0, 0)

        # create a parent prim of the vehicle that uses non-identity scale
        vehicleParentScale = 1.414
        rootPath = str(stage.GetDefaultPrim().GetPath())
        vehicleParentPath = rootPath + "/VehicleParent"
        vehicleParentXform = UsdGeom.Xform.Define(stage, vehicleParentPath)
        vehicleParentXform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
            Gf.Vec3f(0, 0, 0)
        )
        vehicleParentXform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(identityOrient)
        vehicleParentXform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
            Gf.Vec3f(vehicleParentScale, vehicleParentScale, vehicleParentScale)
        )

        vehiclePrimSubPath = "/Vehicle"
        vehiclePath = vehicleParentPath + vehiclePrimSubPath

        vehicleCount = 1
        wheelAttachmentPaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_NONE,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
            vehiclePathsIn=[vehiclePath]
        )

        # add a non-identity scale on the vehicle prim too
        vehicleScale = vehicleParentScale
        vehiclePrim = stage.GetPrimAtPath(vehiclePath)
        vehicleXform = UsdGeom.Xform(vehiclePrim)
        vehicleXform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
            Gf.Vec3f(vehicleScale, vehicleScale, vehicleScale)
        )

        # set suspension frame position to dummy to test it actually gets computed
        wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][0])
        wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(wheelAttPrim)
        wheelAttAPI.GetSuspensionFramePositionAttr().Set(Gf.Vec3f(0, 0, 0))

        # make sure there is non-zero wheel offset
        wheelOffset = Gf.Vec3f(0.1, 0, 0)
        wheelAttAPI.CreateWheelFramePositionAttr().Set(wheelOffset)

        wheelAttPos = wheelAttPrim.GetAttribute("xformOp:translate").Get()

        # execute command to run the helper method to adjust wheel simulation transforms
        PhysXVehicleSuspensionFrameTransformsAutocomputeCommand.execute(wheelAttachmentPaths[0][0])

        # with the given prim hierarchy we have:
        #
        # totalVehicleScale = vehicleParentScale * vehicleScale
        #
        # (wheelAttPos * totalVehicleScale) = (suspFramePos * totalVehicleScale) + (wheelOffset * totalVehicleScale)
        # =>
        # suspFramePos = ((wheelAttPos - wheelOffset) * totalVehicleScale) * (1.0 / totalVehicleScale)
        #              = (wheelAttPos - wheelOffset)
        #
        # note: simplified in this specific case because all relevant orientations are identity
        #
        expectedSuspFramePos = (wheelAttPos - wheelOffset)

        errTol = 0.001
        newSuspFramePosition = wheelAttAPI.GetSuspensionFramePositionAttr().Get()
        err = (newSuspFramePosition - expectedSuspFramePos).GetLength()
        self.assertLess(err, errTol)

        self._remove_factory_ground_plane_and_set_gravity_zero()

        self._simulate_one_frame_with_prep()

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        suspAPI = self._get_suspension(wheelAttPrim, stage)
        suspTravelDist = suspAPI.GetTravelDistanceAttr().Get()

        vehicleGlobalPose = vehicleXform.ComputeLocalToWorldTransform(0)
        vehicleGlobalPos = Gf.Vec3f(vehicleGlobalPose.ExtractTranslation())

        totalVehicleScale = vehicleParentScale * vehicleScale
        expectedWheelGlobalPos = (vehicleGlobalPos + (newSuspFramePosition * totalVehicleScale) +
            (wheelOffset * totalVehicleScale) + Gf.Vec3f(0, -suspTravelDist, 0))
        # note: travel distance is not considered relative to a frame, thus scaling is ignored

        wheelXform = UsdGeom.Xform(wheelAttPrim)
        wheelGlobalPose = wheelXform.ComputeLocalToWorldTransform(0)
        wheelGlobalPos = Gf.Vec3f(wheelGlobalPose.ExtractTranslation())

        err = (wheelGlobalPos - expectedWheelGlobalPos).GetLength()
        self.assertLess(err, errTol)

        global gPhysXInterface
        wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[0][0])

        wheelLocalPosF3 = wheelState[VEHICLE_WHEEL_STATE_LOCAL_POSE_POSITION]
        wheelLocalPos = Gf.Vec3f(wheelLocalPosF3.x, wheelLocalPosF3.y, wheelLocalPosF3.z)
        expectedWheelLocalPos = ((newSuspFramePosition * totalVehicleScale) +
            (wheelOffset * totalVehicleScale) + Gf.Vec3f(0, -suspTravelDist, 0))
        # note: the returned local position has total scale applied, so the same needs to be
        #       done when computing the expected value

        err = (wheelLocalPos - expectedWheelLocalPos).GetLength()
        self.assertLess(err, errTol)

        await self.new_stage()

    #
    # Command to change single relationship.
    #
    async def test_command_set_relationship(self):
        stage = await self.new_stage()

        materialPaths = []
        tireFrictionTablePaths = []
        Factory.createMaterialsAndTireFrictionTables(stage, materialPaths, tireFrictionTablePaths)

        oldFrictionTablePath = tireFrictionTablePaths[Factory.TIRE_FRICTION_TABLE_WINTER_TIRE]
        newFrictionTablePath = tireFrictionTablePaths[Factory.TIRE_FRICTION_TABLE_SUMMER_TIRE]

        self.assertNotEqual(oldFrictionTablePath, newFrictionTablePath)

        wheelComponentsPaths = Factory.WheelComponentPaths()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
        unitScale = get_unit_scale(stage)
        Factory.createWheelComponents(stage, unitScale, oldFrictionTablePath, wheelComponentsPaths)

        wheelTirePrim = stage.GetPrimAtPath(wheelComponentsPaths.tirePaths[Factory.TIRE_FRONT])
        wheelTire = PhysxSchema.PhysxVehicleTireAPI(wheelTirePrim)
        frictionTableRel = wheelTire.GetFrictionTableRel()

        PhysXVehicleSetRelationshipCommand.execute(frictionTableRel.GetPath().pathString, newFrictionTablePath)

        self.assertEqual(frictionTableRel.GetTargets()[0].pathString, newFrictionTablePath)

        omni.kit.undo.undo()

        self.assertEqual(frictionTableRel.GetTargets()[0].pathString, oldFrictionTablePath)

        omni.kit.undo.redo()

        self.assertEqual(frictionTableRel.GetTargets()[0].pathString, newFrictionTablePath)

        await self.new_stage()

    #
    # Command to add tire friction table.
    #
    async def test_command_add_tire_friction_table(self):
        stage = await self.new_stage()

        def _count_friction_table(stage, excludePath):
            pathList = []
            count = 0
            for prim in stage.Traverse():
                if prim.IsA(PhysxSchema.PhysxVehicleTireFrictionTable):
                    count = count + 1
                    path = prim.GetPath()
                    if (path != excludePath):
                        pathList.append(path)

            return (count, pathList)

        setPrimAsSelected = True

        PhysXVehicleTireFrictionTableAddCommand.execute(setPrimAsSelected)

        (count, paths) = _count_friction_table(stage, Sdf.Path())

        self.assertEqual(count, 1)
        firstTablePath = paths[0]

        PhysXVehicleTireFrictionTableAddCommand.execute(setPrimAsSelected)

        (count, paths) = _count_friction_table(stage, firstTablePath)

        self.assertEqual(count, 2)
        self.assertEqual(len(paths), 1)

        omni.kit.undo.undo()

        (count, paths) = _count_friction_table(stage, firstTablePath)

        self.assertEqual(count, 1)
        self.assertEqual(len(paths), 0)

        omni.kit.undo.redo()

        (count, paths) = _count_friction_table(stage, firstTablePath)

        self.assertEqual(count, 2)
        self.assertEqual(len(paths), 1)

        await self.new_stage()

    #
    # Commands to adjust tire friction table.
    #
    async def test_command_modify_tire_friction_table(self):
        stage = await self.new_stage()

        materialPaths = []
        tireFrictionTablePaths = []
        Factory.createMaterialsAndTireFrictionTables(stage, materialPaths, tireFrictionTablePaths)

        tireFrictionTablePath = tireFrictionTablePaths[Factory.TIRE_FRICTION_TABLE_WINTER_TIRE]
        tireFrictionTablePrim = stage.GetPrimAtPath(tireFrictionTablePath)
        tireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable(tireFrictionTablePrim)
        initialEntryCount = len(tireFrictionTable.GetGroundMaterialsRel().GetTargets())

        # changing friction value

        oldMat0Friction = tireFrictionTable.GetFrictionValuesAttr().Get()[0]
        oldMat1Friction = tireFrictionTable.GetFrictionValuesAttr().Get()[1]
        newMat0Friction = 0.1

        PhysXVehicleTireFrictionTableChangeEntryCommand.execute(tireFrictionTablePath, 0, newMat0Friction)

        self.assertEqual(round(tireFrictionTable.GetFrictionValuesAttr().Get()[0], 2), newMat0Friction)
        self.assertEqual(tireFrictionTable.GetFrictionValuesAttr().Get()[1], oldMat1Friction)

        omni.kit.undo.undo()

        self.assertEqual(tireFrictionTable.GetFrictionValuesAttr().Get()[0], oldMat0Friction)
        self.assertEqual(tireFrictionTable.GetFrictionValuesAttr().Get()[1], oldMat1Friction)

        omni.kit.undo.redo()

        self.assertEqual(round(tireFrictionTable.GetFrictionValuesAttr().Get()[0], 2), newMat0Friction)
        self.assertEqual(tireFrictionTable.GetFrictionValuesAttr().Get()[1], oldMat1Friction)

        # removing entry

        PhysXVehicleTireFrictionTableRemoveEntryCommand.execute(tireFrictionTablePath, 0)

        newEntryCount = initialEntryCount - 1
        self.assertEqual(len(tireFrictionTable.GetGroundMaterialsRel().GetTargets()), newEntryCount)
        self.assertEqual(len(tireFrictionTable.GetFrictionValuesAttr().Get()), newEntryCount)
        self.assertEqual(tireFrictionTable.GetGroundMaterialsRel().GetTargets()[0], materialPaths[1])
        self.assertEqual(tireFrictionTable.GetFrictionValuesAttr().Get()[0], oldMat1Friction)

        omni.kit.undo.undo()

        self.assertEqual(len(tireFrictionTable.GetGroundMaterialsRel().GetTargets()), initialEntryCount)
        self.assertEqual(len(tireFrictionTable.GetFrictionValuesAttr().Get()), initialEntryCount)
        self.assertEqual(tireFrictionTable.GetGroundMaterialsRel().GetTargets()[0], materialPaths[0])
        self.assertEqual(round(tireFrictionTable.GetFrictionValuesAttr().Get()[0], 2), newMat0Friction)
        self.assertEqual(tireFrictionTable.GetGroundMaterialsRel().GetTargets()[1], materialPaths[1])
        self.assertEqual(tireFrictionTable.GetFrictionValuesAttr().Get()[1], oldMat1Friction)

        omni.kit.undo.redo()

        self.assertEqual(len(tireFrictionTable.GetGroundMaterialsRel().GetTargets()), newEntryCount)
        self.assertEqual(len(tireFrictionTable.GetFrictionValuesAttr().Get()), newEntryCount)
        self.assertEqual(tireFrictionTable.GetGroundMaterialsRel().GetTargets()[0], materialPaths[1])
        self.assertEqual(tireFrictionTable.GetFrictionValuesAttr().Get()[0], oldMat1Friction)

        # adding entry

        addedMatFriction = 0.9
        addedMatPath = materialPaths[0]
        PhysXVehicleTireFrictionTableAddEntryCommand.execute(tireFrictionTablePath, materialPaths[0], addedMatFriction)

        self.assertEqual(len(tireFrictionTable.GetGroundMaterialsRel().GetTargets()), initialEntryCount)
        self.assertEqual(len(tireFrictionTable.GetFrictionValuesAttr().Get()), initialEntryCount)
        self.assertEqual(tireFrictionTable.GetGroundMaterialsRel().GetTargets()[0], materialPaths[1])
        self.assertEqual(tireFrictionTable.GetFrictionValuesAttr().Get()[0], oldMat1Friction)
        self.assertEqual(tireFrictionTable.GetGroundMaterialsRel().GetTargets()[1], addedMatPath)
        self.assertEqual(round(tireFrictionTable.GetFrictionValuesAttr().Get()[1], 2), addedMatFriction)

        omni.kit.undo.undo()

        self.assertEqual(len(tireFrictionTable.GetGroundMaterialsRel().GetTargets()), newEntryCount)
        self.assertEqual(len(tireFrictionTable.GetFrictionValuesAttr().Get()), newEntryCount)
        self.assertEqual(tireFrictionTable.GetGroundMaterialsRel().GetTargets()[0], materialPaths[1])
        self.assertEqual(tireFrictionTable.GetFrictionValuesAttr().Get()[0], oldMat1Friction)

        omni.kit.undo.redo()

        self.assertEqual(len(tireFrictionTable.GetGroundMaterialsRel().GetTargets()), initialEntryCount)
        self.assertEqual(len(tireFrictionTable.GetFrictionValuesAttr().Get()), initialEntryCount)
        self.assertEqual(tireFrictionTable.GetGroundMaterialsRel().GetTargets()[0], materialPaths[1])
        self.assertEqual(tireFrictionTable.GetFrictionValuesAttr().Get()[0], oldMat1Friction)
        self.assertEqual(tireFrictionTable.GetGroundMaterialsRel().GetTargets()[1], addedMatPath)
        self.assertEqual(round(tireFrictionTable.GetFrictionValuesAttr().Get()[1], 2), addedMatFriction)

        await self.new_stage()

    #
    # Commands to control input behavior (input, mouse, auto-reverse) of vehicle controller.
    #
    async def test_command_vehicle_controller_input_behavior(self):
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

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])

        global gPhysXVehicleInterface

        #
        # input enabled
        #

        # assert expected initial state
        self.assertFalse(gPhysXVehicleInterface.get_input_enabled(vehiclePaths[0]))

        PhysXVehicleControllerEnableInputCommand.execute(vehiclePaths[0], True)

        self.assertTrue(gPhysXVehicleInterface.get_input_enabled(vehiclePaths[0]))
        omni.kit.undo.undo()
        self.assertFalse(gPhysXVehicleInterface.get_input_enabled(vehiclePaths[0]))
        omni.kit.undo.redo()
        self.assertTrue(gPhysXVehicleInterface.get_input_enabled(vehiclePaths[0]))

        #
        # mouse enabled
        #

        # assert expected initial state
        self.assertFalse(gPhysXVehicleInterface.get_mouse_enabled(vehiclePaths[0]))

        PhysXVehicleControllerEnableMouseCommand.execute(vehiclePaths[0], True)

        self.assertTrue(gPhysXVehicleInterface.get_mouse_enabled(vehiclePaths[0]))
        omni.kit.undo.undo()
        self.assertFalse(gPhysXVehicleInterface.get_mouse_enabled(vehiclePaths[0]))
        omni.kit.undo.redo()
        self.assertTrue(gPhysXVehicleInterface.get_mouse_enabled(vehiclePaths[0]))

        #
        # auto-reverse enabled
        #

        # assert expected initial state
        self.assertTrue(gPhysXVehicleInterface.get_auto_reverse_enabled(vehiclePaths[0]))

        PhysXVehicleControllerEnableAutoReverseCommand.execute(vehiclePaths[0], False)

        self.assertFalse(gPhysXVehicleInterface.get_auto_reverse_enabled(vehiclePaths[0]))
        omni.kit.undo.undo()
        self.assertTrue(gPhysXVehicleInterface.get_auto_reverse_enabled(vehiclePaths[0]))
        omni.kit.undo.redo()
        self.assertFalse(gPhysXVehicleInterface.get_auto_reverse_enabled(vehiclePaths[0]))

        await self.new_stage()

    #
    # Command to configure parts of the command response setup
    #
    async def test_command_vehicle_command_response_configuration(self):
        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths
        )

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])

        steeringAPI = PhysxSchema.PhysxVehicleSteeringAPI(vehiclePrim)
        stWheelAtt = steeringAPI.GetWheelsAttr()
        stWheelAttPath = stWheelAtt.GetPath().pathString
        stAngleMulAtt = steeringAPI.GetAngleMultipliersAttr()
        stAngleMulAttPath = stAngleMulAtt.GetPath().pathString

        brakes0API = PhysxSchema.PhysxVehicleBrakesAPI(vehiclePrim, PhysxSchema.Tokens.brakes0)
        br0WheelAtt = brakes0API.GetWheelsAttr()
        br0WheelAttPath = br0WheelAtt.GetPath().pathString
        br0TorqueMulAtt = brakes0API.GetTorqueMultipliersAttr()
        br0TorqueMulAttPath = br0TorqueMulAtt.GetPath().pathString

        wheelCount = 4
        w0Idx = 0
        w1Idx = 1
        w2Idx = 2
        w3Idx = 3
        testValue = 0.2
        defaultValue = 1.0
        errEps = 1e-7

        #
        # wheel indices defined, value array defined,
        # setting value for index that is in index array
        # => only the entry in the vallue array will change
        #
        startVal = 0.8
        stAngleMulAtt.Set([startVal, startVal])

        indices = stWheelAtt.Get()
        self.assertEqual(indices[0], 0)
        self.assertEqual(indices[1], 1)

        (success, newWheelListIndex) = PhysXVehicleCommandResponseChangeEntryCommand.execute(
            stAngleMulAttPath, stWheelAttPath, 0, testValue, startVal, w0Idx, defaultValue, wheelCount)

        self.assertEqual(newWheelListIndex, 0)

        values = stAngleMulAtt.Get()
        self.assertEqual(len(values), 2)
        self.assertAlmostEqual(values[0], testValue, delta=errEps)
        self.assertAlmostEqual(values[1], startVal, delta=errEps)

        (success, newWheelListIndex) = PhysXVehicleCommandResponseChangeEntryCommand.execute(
            stAngleMulAttPath, stWheelAttPath, 1, testValue, startVal, w1Idx, defaultValue, wheelCount)

        self.assertEqual(newWheelListIndex, 1)

        values = stAngleMulAtt.Get()
        self.assertEqual(len(values), 2)
        self.assertAlmostEqual(values[0], testValue, delta=errEps)
        self.assertAlmostEqual(values[1], testValue, delta=errEps)

        omni.kit.undo.undo()
        omni.kit.undo.undo()

        values = stAngleMulAtt.Get()
        self.assertEqual(len(values), 2)
        self.assertAlmostEqual(values[0], startVal, delta=errEps)
        self.assertAlmostEqual(values[1], startVal, delta=errEps)

        #
        # wheel indices defined, value array empty,
        # setting value for index that is in index array
        # => each entry in the index array will also get an entry in the value
        #    aray, using defaultValue for the ones where the value is not defined
        #
        stAngleMulAtt.Clear()
        self.assertFalse(stAngleMulAtt.HasAuthoredValue())

        (success, newWheelListIndex) = PhysXVehicleCommandResponseChangeEntryCommand.execute(
            stAngleMulAttPath, stWheelAttPath, 0, testValue, startVal, w0Idx, defaultValue, wheelCount)

        self.assertEqual(newWheelListIndex, 0)

        values = stAngleMulAtt.Get()
        self.assertEqual(len(values), 2)
        self.assertAlmostEqual(values[0], testValue, delta=errEps)
        self.assertAlmostEqual(values[1], defaultValue, delta=errEps)

        omni.kit.undo.undo()
        self.assertFalse(stAngleMulAtt.HasAuthoredValue())

        (success, newWheelListIndex) = PhysXVehicleCommandResponseChangeEntryCommand.execute(
            stAngleMulAttPath, stWheelAttPath, 1, testValue, startVal, w1Idx, defaultValue, wheelCount)

        self.assertEqual(newWheelListIndex, 1)

        values = stAngleMulAtt.Get()
        self.assertEqual(len(values), 2)
        self.assertAlmostEqual(values[0], defaultValue, delta=errEps)
        self.assertAlmostEqual(values[1], testValue, delta=errEps)

        #
        # wheel indices defined, value array defined,
        # setting value for index that is not in index array
        # => The new index and value will get added to the corresponding arrays
        #
        stAngleMulAtt.Set([startVal, startVal])

        (success, newWheelListIndex) = PhysXVehicleCommandResponseChangeEntryCommand.execute(
            stAngleMulAttPath, stWheelAttPath, -1, testValue, startVal, w3Idx, defaultValue, wheelCount)

        self.assertEqual(newWheelListIndex, 2)

        values = stAngleMulAtt.Get()
        self.assertEqual(len(values), 3)
        self.assertAlmostEqual(values[0], startVal, delta=errEps)
        self.assertAlmostEqual(values[1], startVal, delta=errEps)
        self.assertAlmostEqual(values[2], testValue, delta=errEps)

        indices = stWheelAtt.Get()
        self.assertEqual(len(indices), 3)
        self.assertEqual(indices[0], w0Idx)
        self.assertEqual(indices[1], w1Idx)
        self.assertEqual(indices[2], w3Idx)

        omni.kit.undo.undo()

        values = stAngleMulAtt.Get()
        self.assertEqual(len(values), 2)
        self.assertAlmostEqual(values[0], startVal, delta=errEps)
        self.assertAlmostEqual(values[1], startVal, delta=errEps)

        indices = stWheelAtt.Get()
        self.assertEqual(len(indices), 2)
        self.assertEqual(indices[0], w0Idx)
        self.assertEqual(indices[1], w1Idx)

        #
        # wheel indices defined, value array empty,
        # setting value for index that is not in index array
        # => the already existing indices in the array will each get an entry
        #    in the value array using defaultValue. The new index and value
        #    will get added to the corresponding arrays
        #
        stAngleMulAtt.Clear()
        self.assertFalse(stAngleMulAtt.HasAuthoredValue())

        (success, newWheelListIndex) = PhysXVehicleCommandResponseChangeEntryCommand.execute(
            stAngleMulAttPath, stWheelAttPath, -1, testValue, startVal, w3Idx, defaultValue, wheelCount)

        self.assertEqual(newWheelListIndex, 2)

        values = stAngleMulAtt.Get()
        self.assertEqual(len(values), 3)
        self.assertAlmostEqual(values[0], defaultValue, delta=errEps)
        self.assertAlmostEqual(values[1], defaultValue, delta=errEps)
        self.assertAlmostEqual(values[2], testValue, delta=errEps)

        indices = stWheelAtt.Get()
        self.assertEqual(len(indices), 3)
        self.assertEqual(indices[0], w0Idx)
        self.assertEqual(indices[1], w1Idx)
        self.assertEqual(indices[2], w3Idx)

        omni.kit.undo.undo()

        values = stAngleMulAtt.Get()
        self.assertFalse(stAngleMulAtt.HasAuthoredValue())

        indices = stWheelAtt.Get()
        self.assertEqual(len(indices), 2)
        self.assertEqual(indices[0], w0Idx)
        self.assertEqual(indices[1], w1Idx)

        #
        # wheel indices empty, value array empty,
        # setting value
        # => the index and value array will get filled in for all wheels
        #
        br0TorqueMulAtt.Clear()
        self.assertFalse(br0TorqueMulAtt.HasAuthoredValue())

        br0WheelAtt.Clear()
        self.assertFalse(br0WheelAtt.HasAuthoredValue())

        (success, newWheelListIndex) = PhysXVehicleCommandResponseChangeEntryCommand.execute(
            br0TorqueMulAttPath, br0WheelAttPath, -1, testValue, startVal, w2Idx, defaultValue, wheelCount)

        self.assertEqual(newWheelListIndex, 2)

        values = br0TorqueMulAtt.Get()
        self.assertEqual(len(values), wheelCount)
        self.assertAlmostEqual(values[0], defaultValue, delta=errEps)
        self.assertAlmostEqual(values[1], defaultValue, delta=errEps)
        self.assertAlmostEqual(values[2], testValue, delta=errEps)
        self.assertAlmostEqual(values[3], defaultValue, delta=errEps)

        indices = br0WheelAtt.Get()
        self.assertEqual(len(indices), wheelCount)
        self.assertEqual(indices[0], w0Idx)
        self.assertEqual(indices[1], w1Idx)
        self.assertEqual(indices[2], w2Idx)
        self.assertEqual(indices[3], w3Idx)

        omni.kit.undo.undo()

        self.assertFalse(br0TorqueMulAtt.HasAuthoredValue())
        self.assertFalse(br0WheelAtt.HasAuthoredValue())

        await self.new_stage()

    #
    # Command to configure parts of the multi wheel differential setup
    #
    async def test_command_vehicle_differential_configuration(self):
        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_STANDARD,
            vehiclePathsOut=vehiclePaths
        )

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])

        diffAPI = PhysxSchema.PhysxVehicleMultiWheelDifferentialAPI(vehiclePrim)
        wheelsAtt = diffAPI.GetWheelsAttr()
        wheelsAttPath = wheelsAtt.GetPath().pathString
        trRatiosAtt = diffAPI.GetTorqueRatiosAttr()
        trRatiosAttPath = trRatiosAtt.GetPath().pathString
        awsRatiosAtt = diffAPI.GetAverageWheelSpeedRatiosAttr()
        awsRatiosAttPath = awsRatiosAtt.GetPath().pathString
        valueArrayAttPaths = [trRatiosAttPath, awsRatiosAttPath]
        trRatiosIdx = 0
        awsRatiosIdx = 1

        wheelCount = 4
        w0Idx = 0
        w1Idx = 1
        w2Idx = 2
        w3Idx = 3
        testValue0 = -0.3
        testValue1 = 0.2
        errEps = 1e-7

        #
        # wheel indices defined, value arrays defined,
        # setting value for index that is in index array
        # => only the entry in the vallue array will change
        #
        startVal = 0.5
        wheelsAtt.Set([w0Idx, w1Idx])
        trRatiosAtt.Set([startVal, startVal])
        awsRatiosAtt.Set([startVal, startVal])

        (success, newWheelListIndex) = PhysXVehicleDifferentialChangeEntryCommand.execute(
            valueArrayAttPaths, trRatiosIdx, wheelsAttPath, 0, testValue0, startVal, w0Idx)

        self.assertEqual(newWheelListIndex, 0)

        (success, newWheelListIndex) = PhysXVehicleDifferentialChangeEntryCommand.execute(
            valueArrayAttPaths, awsRatiosIdx, wheelsAttPath, 1, testValue1, startVal, w1Idx)

        self.assertEqual(newWheelListIndex, 1)

        values = trRatiosAtt.Get()
        self.assertEqual(len(values), 2)
        self.assertAlmostEqual(values[0], testValue0, delta=errEps)
        self.assertAlmostEqual(values[1], startVal, delta=errEps)

        values = awsRatiosAtt.Get()
        self.assertEqual(len(values), 2)
        self.assertAlmostEqual(values[0], startVal, delta=errEps)
        self.assertAlmostEqual(values[1], testValue1, delta=errEps)

        omni.kit.undo.undo()
        omni.kit.undo.undo()

        values = trRatiosAtt.Get()
        self.assertEqual(len(values), 2)
        self.assertAlmostEqual(values[0], startVal, delta=errEps)
        self.assertAlmostEqual(values[1], startVal, delta=errEps)

        values = awsRatiosAtt.Get()
        self.assertEqual(len(values), 2)
        self.assertAlmostEqual(values[0], startVal, delta=errEps)
        self.assertAlmostEqual(values[1], startVal, delta=errEps)

        # setting all value entries to 0 should remove the entry from all arrays
        (success, newWheelListIndex) = PhysXVehicleDifferentialChangeEntryCommand.execute(
            valueArrayAttPaths, trRatiosIdx, wheelsAttPath, 0, 0.0, startVal, w0Idx)

        self.assertEqual(newWheelListIndex, 0)

        indices = wheelsAtt.Get()
        self.assertEqual(len(indices), 2)

        values = trRatiosAtt.Get()
        self.assertEqual(len(values), 2)
        self.assertAlmostEqual(values[0], 0.0, delta=errEps)
        self.assertAlmostEqual(values[1], startVal, delta=errEps)

        values = awsRatiosAtt.Get()
        self.assertEqual(len(values), 2)
        self.assertAlmostEqual(values[0], startVal, delta=errEps)
        self.assertAlmostEqual(values[1], startVal, delta=errEps)

        (success, newWheelListIndex) = PhysXVehicleDifferentialChangeEntryCommand.execute(
            valueArrayAttPaths, awsRatiosIdx, wheelsAttPath, 0, 0.0, startVal, w0Idx)

        self.assertEqual(newWheelListIndex, -1)

        indices = wheelsAtt.Get()
        self.assertEqual(len(indices), 1)
        self.assertEqual(indices[0], w1Idx)

        values = trRatiosAtt.Get()
        self.assertEqual(len(values), 1)
        self.assertAlmostEqual(values[0], startVal, delta=errEps)

        values = awsRatiosAtt.Get()
        self.assertEqual(len(values), 1)
        self.assertAlmostEqual(values[0], startVal, delta=errEps)

        omni.kit.undo.undo()

        indices = wheelsAtt.Get()
        self.assertEqual(len(indices), 2)

        values = trRatiosAtt.Get()
        self.assertEqual(len(values), 2)
        self.assertAlmostEqual(values[0], 0.0, delta=errEps)
        self.assertAlmostEqual(values[1], startVal, delta=errEps)

        values = awsRatiosAtt.Get()
        self.assertEqual(len(values), 2)
        self.assertAlmostEqual(values[0], startVal, delta=errEps)
        self.assertAlmostEqual(values[1], startVal, delta=errEps)

        omni.kit.undo.undo()

        indices = wheelsAtt.Get()
        self.assertEqual(len(indices), 2)
        self.assertEqual(indices[0], w0Idx)
        self.assertEqual(indices[1], w1Idx)

        values = trRatiosAtt.Get()
        self.assertEqual(len(values), 2)
        self.assertAlmostEqual(values[0], startVal, delta=errEps)
        self.assertAlmostEqual(values[1], startVal, delta=errEps)

        values = awsRatiosAtt.Get()
        self.assertEqual(len(values), 2)
        self.assertAlmostEqual(values[0], startVal, delta=errEps)
        self.assertAlmostEqual(values[1], startVal, delta=errEps)

        # setting all value entries to 0 should remove the entry from all arrays
        # (empty value arrays should allow deletion too)

        trRatiosAtt.Clear()
        self.assertFalse(trRatiosAtt.HasAuthoredValue())

        (success, newWheelListIndex) = PhysXVehicleDifferentialChangeEntryCommand.execute(
            valueArrayAttPaths, awsRatiosIdx, wheelsAttPath, 1, 0.0, startVal, w1Idx)

        self.assertEqual(newWheelListIndex, -1)

        indices = wheelsAtt.Get()
        self.assertEqual(len(indices), 1)
        self.assertEqual(indices[0], w0Idx)

        self.assertFalse(trRatiosAtt.HasAuthoredValue())

        values = awsRatiosAtt.Get()
        self.assertEqual(len(values), 1)
        self.assertAlmostEqual(values[0], startVal, delta=errEps)

        omni.kit.undo.undo()

        indices = wheelsAtt.Get()
        self.assertEqual(len(indices), 2)
        self.assertEqual(indices[0], w0Idx)
        self.assertEqual(indices[1], w1Idx)

        self.assertFalse(trRatiosAtt.HasAuthoredValue())

        values = awsRatiosAtt.Get()
        self.assertEqual(len(values), 2)
        self.assertAlmostEqual(values[0], startVal, delta=errEps)
        self.assertAlmostEqual(values[1], startVal, delta=errEps)

        #
        # wheel indices defined, value arrays empty,
        # setting value for index that is in index array
        # => each entry in the index array will also get an entry in the value
        #    array, evenly distributing the "remaining ratio" among the entries
        #    that were not set explicitly
        #
        wheelsAtt.Set([w0Idx, w1Idx, w3Idx])
        trRatiosAtt.Clear()
        self.assertFalse(trRatiosAtt.HasAuthoredValue())
        awsRatiosAtt.Clear()
        self.assertFalse(awsRatiosAtt.HasAuthoredValue())

        (success, newWheelListIndex) = PhysXVehicleDifferentialChangeEntryCommand.execute(
            valueArrayAttPaths, trRatiosIdx, wheelsAttPath, 1, testValue0, None, w1Idx)

        self.assertEqual(newWheelListIndex, 1)

        fillValue = (1.0 - math.fabs(testValue0)) / 2.0
        values = trRatiosAtt.Get()
        self.assertEqual(len(values), 3)
        self.assertAlmostEqual(values[0], fillValue, delta=errEps)
        self.assertAlmostEqual(values[1], testValue0, delta=errEps)
        self.assertAlmostEqual(values[2], fillValue, delta=errEps)

        # other value arrays should not be touched
        self.assertFalse(awsRatiosAtt.HasAuthoredValue())

        omni.kit.undo.undo()

        self.assertFalse(trRatiosAtt.HasAuthoredValue())
        self.assertFalse(awsRatiosAtt.HasAuthoredValue())

        #
        # wheel indices defined, value array defined,
        # setting value for index that is not in index array
        # => The new index and value will get added to the corresponding arrays
        #    (other value arrays will get extended too if not empty)
        #
        wheelsAtt.Set([w0Idx, w1Idx])
        trRatiosAtt.Set([startVal, startVal])
        awsRatiosAtt.Set([startVal, startVal])

        (success, newWheelListIndex) = PhysXVehicleDifferentialChangeEntryCommand.execute(
            valueArrayAttPaths, trRatiosIdx, wheelsAttPath, -1, testValue0, None, w3Idx)

        self.assertEqual(newWheelListIndex, 2)

        indices = wheelsAtt.Get()
        self.assertEqual(len(indices), 3)
        self.assertEqual(indices[0], w0Idx)
        self.assertEqual(indices[1], w1Idx)
        self.assertEqual(indices[2], w3Idx)

        values = trRatiosAtt.Get()
        self.assertEqual(len(values), 3)
        self.assertAlmostEqual(values[0], startVal, delta=errEps)
        self.assertAlmostEqual(values[1], startVal, delta=errEps)
        self.assertAlmostEqual(values[2], testValue0, delta=errEps)

        values = awsRatiosAtt.Get()
        self.assertEqual(len(values), 3)
        self.assertAlmostEqual(values[0], startVal, delta=errEps)
        self.assertAlmostEqual(values[1], startVal, delta=errEps)
        self.assertAlmostEqual(values[2], 0, delta=errEps)

        omni.kit.undo.undo()

        indices = wheelsAtt.Get()
        self.assertEqual(len(indices), 2)
        self.assertEqual(indices[0], w0Idx)
        self.assertEqual(indices[1], w1Idx)

        values = trRatiosAtt.Get()
        self.assertEqual(len(values), 2)
        self.assertAlmostEqual(values[0], startVal, delta=errEps)
        self.assertAlmostEqual(values[1], startVal, delta=errEps)

        values = awsRatiosAtt.Get()
        self.assertEqual(len(values), 2)
        self.assertAlmostEqual(values[0], startVal, delta=errEps)
        self.assertAlmostEqual(values[1], startVal, delta=errEps)

        #
        # wheel indices defined, value array empty,
        # setting value for index that is not in index array
        # => the target value array will get values for all entries,
        #    other empty value arrays remain untouched
        #
        wheelsAtt.Set([w0Idx, w1Idx])
        trRatiosAtt.Clear()
        self.assertFalse(trRatiosAtt.HasAuthoredValue())
        awsRatiosAtt.Clear()
        self.assertFalse(awsRatiosAtt.HasAuthoredValue())

        (success, newWheelListIndex) = PhysXVehicleDifferentialChangeEntryCommand.execute(
            valueArrayAttPaths, trRatiosIdx, wheelsAttPath, -1, testValue0, None, w3Idx)

        self.assertEqual(newWheelListIndex, 2)

        indices = wheelsAtt.Get()
        self.assertEqual(len(indices), 3)
        self.assertEqual(indices[0], w0Idx)
        self.assertEqual(indices[1], w1Idx)
        self.assertEqual(indices[2], w3Idx)

        fillValue = (1.0 - math.fabs(testValue0)) / 2.0
        values = trRatiosAtt.Get()
        self.assertEqual(len(values), 3)
        self.assertAlmostEqual(values[0], fillValue, delta=errEps)
        self.assertAlmostEqual(values[1], fillValue, delta=errEps)
        self.assertAlmostEqual(values[2], testValue0, delta=errEps)

        self.assertFalse(awsRatiosAtt.HasAuthoredValue())

        omni.kit.undo.undo()

        indices = wheelsAtt.Get()
        self.assertEqual(len(indices), 2)
        self.assertEqual(indices[0], w0Idx)
        self.assertEqual(indices[1], w1Idx)

        self.assertFalse(trRatiosAtt.HasAuthoredValue())
        self.assertFalse(awsRatiosAtt.HasAuthoredValue())

        #
        # wheel indices empty, value array empty,
        # setting value
        # => single entry in the index array and the specified value array
        #
        wheelsAtt.Clear()
        self.assertFalse(wheelsAtt.HasAuthoredValue())
        trRatiosAtt.Clear()
        self.assertFalse(trRatiosAtt.HasAuthoredValue())
        awsRatiosAtt.Clear()
        self.assertFalse(awsRatiosAtt.HasAuthoredValue())

        (success, newWheelListIndex) = PhysXVehicleDifferentialChangeEntryCommand.execute(
            valueArrayAttPaths, trRatiosIdx, wheelsAttPath, -1, testValue0, None, w2Idx)

        self.assertEqual(newWheelListIndex, 0)

        indices = wheelsAtt.Get()
        self.assertEqual(len(indices), 1)
        self.assertEqual(indices[0], w2Idx)

        values = trRatiosAtt.Get()
        self.assertEqual(len(values), 1)
        self.assertAlmostEqual(values[0], testValue0, delta=errEps)

        self.assertFalse(awsRatiosAtt.HasAuthoredValue())

        omni.kit.undo.undo()

        self.assertFalse(wheelsAtt.HasAuthoredValue())
        self.assertFalse(trRatiosAtt.HasAuthoredValue())
        self.assertFalse(awsRatiosAtt.HasAuthoredValue())

        await self.new_stage()

    #
    # Test the vehicle creation wizard command.
    #
    async def test_vehicle_wizard_create(self):
        for i in range(2):
            stage = await self.new_stage()

            vehicleData = VehicleWizard.VehicleData(get_unit_scale(stage),
                VehicleWizard.VehicleData.AXIS_Y, VehicleWizard.VehicleData.AXIS_Z)

            if (i == 0):
                vehicleData.createShareableComponents = False
            else:
                vehicleData.createShareableComponents = True

            # Place the vehicle under the default prim.
            defaultPath = str(stage.GetDefaultPrim().GetPath())
            vehicleData.rootVehiclePath = defaultPath + VehicleWizard.VEHICLE_ROOT_BASE_PATH
            vehicleData.rootSharedPath = defaultPath + VehicleWizard.SHARED_DATA_ROOT_BASE_PATH
            vehiclePath = vehicleData.rootVehiclePath + "/Vehicle"

            (success, (messageList, _)) = PhysXVehicleWizardCreateCommand.execute(vehicleData)
            self.assertTrue(success)
            self.assertFalse(messageList)  # python style guide to check for empty list

            # Create a ground plane to drive on.
            collisionGroupPath = vehicleData.rootSharedPath + VehicleWizard.VEHICLE_COLLISION_GROUP_GROUND_SURFACE_PATH
            groundMaterialPath = vehicleData.rootSharedPath + VehicleWizard.VEHICLE_GROUND_MATERIAL_PATH
            Factory.createGroundPlane(stage, vehicleData.unitScale, collisionGroupPath, groundMaterialPath)

            # Ensure the default vehicle was created.
            vehiclePrim = stage.GetPrimAtPath(vehiclePath)
            self.assertTrue(vehiclePrim.HasAPI(PhysxSchema.PhysxVehicleAPI))

            vehicleController = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)
            vehicleController.GetSteerAttr().Set(1.0)
            vehicleController.GetAcceleratorAttr().Set(1.0)

            # Run the simulation to make sure the vehicle moves.
            secondsToRun = 2.0
            self._prepare_for_simulation()
            self._simulate(secondsToRun)

            self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

            # Check vehicle position
            endPos = vehiclePrim.GetAttribute("xformOp:translate").Get()
            # print(str(endPos[0]) + "," + str(endPos[1]) + "," + str(endPos[2]))

            expectedPos = (293.02, 84.99, 336.62)

            horizontalErrTol = 10.0
            verticalErrTol = 1.0
            self.assertLess(math.fabs(endPos[0] - expectedPos[0]), horizontalErrTol)
            self.assertLess(math.fabs(endPos[1] - expectedPos[1]), verticalErrTol)
            self.assertLess(math.fabs(endPos[2] - expectedPos[2]), horizontalErrTol)

        await self.new_stage()

    #
    # Test the vehicle creation wizard command, providing the vehicle and wheel
    # attachment prims (apply APIs in-place) and using scale.
    #
    async def test_vehicle_wizard_create_with_user_prims(self):
        driveTypeList = [VehicleWizard.DRIVE_TYPE_STANDARD, VehicleWizard.DRIVE_TYPE_BASIC, VehicleWizard.DRIVE_TYPE_NONE]
        createShareableComponentsList = [False, True]

        for driveType in driveTypeList:
            for createShareableComponents in createShareableComponentsList:
                stage = await self.new_stage()

                metersPerUnit = 1.0
                kilogramsPerUnit = 1.0
                UsdGeom.SetStageMetersPerUnit(stage, metersPerUnit)
                UsdPhysics.SetStageKilogramsPerUnit(stage, kilogramsPerUnit)
                UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)

                vehicleData = VehicleWizard.VehicleData(get_unit_scale(stage),
                    VehicleWizard.VehicleData.AXIS_Y, VehicleWizard.VehicleData.AXIS_Z)

                # Place the vehicle under the default prim.
                defaultPath = str(stage.GetDefaultPrim().GetPath())
                vehiclePath = defaultPath + "/Vehicle"
                chassisPath = vehiclePath  + "/Chassis"
                wheelRootPath = vehiclePath  + "/Wheels"
                wheelPathList = [
                    wheelRootPath + "/FL",
                    wheelRootPath + "/FR",
                    wheelRootPath + "/RL",
                    wheelRootPath + "/RR",
                ]

                vehicleScale = Gf.Vec3f(2.0, 2.0, 2.0)
                vehicleScaleInverse = Gf.Vec3f(1.0 / vehicleScale[0], 1.0 / vehicleScale[1], 1.0 / vehicleScale[2])
                vehiclePos = Gf.Vec3f(0.0, 0.0, 0.0)
                wheelRadius = 0.3
                wheelWidth = 0.2
                chassisSize = Gf.Vec3f(1.8, 1.2, 3.6)
                chassisOffset = Gf.Vec3f(
                    chassisSize[0] * 0.5,
                    wheelRadius + (chassisSize[1] * 0.5),
                    chassisSize[2] * 0.5
                )
                chassisPos = vehiclePos + chassisOffset
                chassisOffsetScaled = Gf.CompMult(chassisOffset, vehicleScaleInverse)
                wheelOffsetBase = Gf.Vec3f((chassisSize[0] * 0.5) + wheelWidth, chassisSize[1] * -0.5, chassisSize[2] * 0.5)

                vehicleXform = UsdGeom.Xform.Define(stage, vehiclePath)
                vehicleXform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(vehiclePos)
                vehicleXform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(vehicleScale)
                vehiclePrim = vehicleXform.GetPrim()

                chassisCube = UsdGeom.Cube.Define(stage, chassisPath)
                chassisCube.CreateSizeAttr(1.0)
                chassisCube.CreatePurposeAttr(UsdGeom.Tokens.render)
                extent = UsdGeom.Cube.ComputeExtentFromPlugins(chassisCube, 0)
                chassisCube.CreateExtentAttr(extent)
                chassisCube.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(chassisOffsetScaled)
                chassisCube.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
                    Gf.Vec3f(
                        vehicleScaleInverse[0] * chassisSize[0],
                        vehicleScaleInverse[1] * chassisSize[1],
                        vehicleScaleInverse[2] * chassisSize[2]
                    )
                )

                UsdGeom.Xform.Define(stage, wheelRootPath)

                wheelCylList = []
                wheelPosWorldList = []

                for frontIdx in range(2):
                    for sideIdx in range(2):
                        wheelIndex = (2*frontIdx) + sideIdx

                        wheelPath = wheelPathList[wheelIndex]

                        if (frontIdx == 0):
                            frontSign = 1.0
                        else:
                            frontSign = -1.0

                        if (sideIdx == 0):
                            sideSign = 1.0
                        else:
                            sideSign = -1.0

                        wheelOffset = Gf.Vec3f(wheelOffsetBase[0] * sideSign, wheelOffsetBase[1], wheelOffsetBase[2] * frontSign)
                        wheelPosLocal = wheelOffset + chassisOffset
                        wheelPosLocalScaled = Gf.CompMult(wheelPosLocal, vehicleScaleInverse)

                        wheelCyl = UsdGeom.Cylinder.Define(stage, wheelPath)
                        wheelCyl.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(wheelPosLocalScaled)
                        wheelCyl.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(vehicleScaleInverse) # undo the vehicle scale to get expected width/radius
                        wheelCyl.CreatePurposeAttr(UsdGeom.Tokens.render)
                        wheelCyl.CreateHeightAttr(wheelWidth)
                        wheelCyl.CreateRadiusAttr(wheelRadius)
                        wheelCyl.CreateAxisAttr(UsdGeom.Tokens.x)
                        # if height or radius is authored, USD expects extent to be authored too
                        cylExtent = UsdGeom.Cylinder.ComputeExtentFromPlugins(wheelCyl, 0)
                        wheelCyl.CreateExtentAttr(cylExtent)

                        wheelCylList.append(wheelCyl)

                        vehicleData.tireList[wheelIndex].path = wheelPath
                        vehicleData.tireList[wheelIndex].position = Gf.Vec3f(1000.0, 1000.0, 1000.0)  # note: set nonsense to ensure it's ignored if a path is set
                        vehicleData.tireList[wheelIndex].radius = wheelRadius
                        vehicleData.tireList[wheelIndex].width = wheelWidth

                        wheelPosWorld = wheelOffset + chassisPos
                        wheelPosWorldList.append(wheelPosWorld)

                vehicleData.rootVehiclePath = vehiclePath
                vehicleData.rootSharedPath = defaultPath + VehicleWizard.SHARED_DATA_ROOT_BASE_PATH
                vehicleData.vehiclePath = vehiclePath
                vehicleData.position = chassisPos  # note: in world space
                vehicleData.chassisLength = chassisSize[2]
                vehicleData.chassisWidth = chassisSize[0]
                vehicleData.chassisHeight = chassisSize[1]
                vehicleData.numberOfAxles = 2
                vehicleData._reset_weightDistribution()
                vehicleData.tireRadius = wheelRadius
                vehicleData.tireWidth = wheelWidth
                vehicleData.createShareableComponents = createShareableComponents
                vehicleData.set_drive_type(driveType)

                (success, (messageList, _)) = PhysXVehicleWizardCreateCommand.execute(vehicleData)
                self.assertTrue(success)
                self.assertFalse(messageList)  # python style guide to check for empty list

                self.assertTrue(vehiclePrim.HasAPI(PhysxSchema.PhysxVehicleAPI))

                for wheelIndex in range(len(wheelCylList)):
                    wheelAttPrim = wheelCylList[wheelIndex].GetPrim()

                    self.assertTrue(wheelAttPrim.HasAPI(PhysxSchema.PhysxVehicleWheelAttachmentAPI))

                    (radius, width) = self._get_wheel_radius_and_width(wheelAttPrim, stage)

                    errTol = 0.0001
                    self.assertAlmostEqual(radius, wheelRadius, delta=errTol)
                    self.assertAlmostEqual(width, wheelWidth, delta=errTol)

                omni.kit.undo.undo()

                # all applied API schemas should have been removed again
                self.assertFalse(vehiclePrim.GetAppliedSchemas())

                for wheelIndex in range(len(wheelCylList)):
                    wheelAttPrim = wheelCylList[wheelIndex].GetPrim()

                    self.assertFalse(wheelAttPrim.GetAppliedSchemas())

                omni.kit.undo.redo()

                # Create a ground plane to drive on.
                collisionGroupPath = vehicleData.rootSharedPath + VehicleWizard.VEHICLE_COLLISION_GROUP_GROUND_SURFACE_PATH
                groundMaterialPath = vehicleData.rootSharedPath + VehicleWizard.VEHICLE_GROUND_MATERIAL_PATH
                Factory.createGroundPlane(stage, vehicleData.unitScale, collisionGroupPath, groundMaterialPath)

                # Run the simulation and make sure the vehicle/wheels end up at the expected positions.
                secondsToRun = 0.5
                self._simulate_with_prep(secondsToRun)

                self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

                errTol = 0.001

                # Check vehicle chassis position
                pos = chassisCube.ComputeLocalToWorldTransform(0).ExtractTranslation()
                self.assertAlmostEqual(chassisPos[0], pos[0], delta=errTol)
                self.assertAlmostEqual(chassisPos[1], pos[1], delta=errTol)
                self.assertAlmostEqual(chassisPos[2], pos[2], delta=errTol)

                for wheelIndex in range(len(wheelCylList)):
                    wheelCyl = wheelCylList[wheelIndex]
                    wheelPosWorld = wheelPosWorldList[wheelIndex]

                    pos = wheelCyl.ComputeLocalToWorldTransform(0).ExtractTranslation()
                    self.assertAlmostEqual(wheelPosWorld[0], pos[0], delta=errTol)
                    self.assertAlmostEqual(wheelPosWorld[1], pos[1], delta=errTol)
                    self.assertAlmostEqual(wheelPosWorld[2], pos[2], delta=errTol)

        await self.new_stage()

    #
    # Test adding and removing vehicles while the sim is running using the vehicle creation wizard command.
    #
    async def test_dynamic_vehicle_creation(self):
        stage = await self.new_stage()

        metersPerUnit = 0.01
        lengthScale = 1.0 / metersPerUnit
        kilogramsPerUnit = 1.0
        UsdGeom.SetStageMetersPerUnit(stage, metersPerUnit)
        UsdPhysics.SetStageKilogramsPerUnit(stage, kilogramsPerUnit)

        vehicleData = VehicleWizard.VehicleData(get_unit_scale(stage),
            VehicleWizard.VehicleData.AXIS_Y, VehicleWizard.VehicleData.AXIS_Z)

        # Place the vehicle under the default prim.
        defaultPath = str(stage.GetDefaultPrim().GetPath())
        vehicleData.rootVehiclePath = defaultPath + VehicleWizard.VEHICLE_ROOT_BASE_PATH
        vehicleData.rootSharedPath = defaultPath + VehicleWizard.SHARED_DATA_ROOT_BASE_PATH

        # Create one vehicle to set up the simulation.
        (success, (messageList, _)) = PhysXVehicleWizardCreateCommand.execute(vehicleData)
        self.assertTrue(success)
        self.assertFalse(messageList)  # python style guide to check for empty list

        # Ensure the default vehicle was created.
        vehiclePath = vehicleData.rootVehiclePath + "/Vehicle"
        vehiclePrim = stage.GetPrimAtPath(vehiclePath)
        self.assertTrue(vehiclePrim.HasAPI(PhysxSchema.PhysxVehicleAPI))

        vehicleController = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)
        vehicleController.GetAcceleratorAttr().Set(1.0)

        # Create a ground plane to drive on.
        collisionGroupPath = vehicleData.rootSharedPath + VehicleWizard.VEHICLE_COLLISION_GROUP_GROUND_SURFACE_PATH
        groundMaterialPath = vehicleData.rootSharedPath + VehicleWizard.VEHICLE_GROUND_MATERIAL_PATH
        Factory.createGroundPlane(stage, vehicleData.unitScale, collisionGroupPath, groundMaterialPath)

        # Run the simulation to make sure the vehicle moves.
        self._prepare_for_simulation()
        self._simulate_one_frame()

        numberOfVehicles = 50
        vehicleList = []
        startPosList = []

        # Pseudo random number generator, but deterministic.
        random.seed(0)

        for i in range(numberOfVehicles):
            p = random.uniform(0.0, 1.0)

            if p > 0.4:
                # Create various drive types.
                vehicleData.driveTypeIndex = random.randint(0, 2)

                # Create the next vehicle
                vehicleData.rootVehiclePath = defaultPath + VehicleWizard.VEHICLE_ROOT_BASE_PATH + str(i + 1)
                (success, (messageList, _)) = PhysXVehicleWizardCreateCommand.execute(vehicleData)
                self.assertTrue(success)
                self.assertFalse(messageList)  # python style guide to check for empty list
                vehicleList.append(vehicleData.rootVehiclePath)

                # Ensure the default vehicle was created.
                vehiclePath = vehicleData.rootVehiclePath + "/Vehicle"
                vehiclePrim = stage.GetPrimAtPath(vehiclePath)
                self.assertTrue(vehiclePrim.HasAPI(PhysxSchema.PhysxVehicleAPI))
                posAttr = vehiclePrim.GetAttribute("xformOp:translate")
                startPos = posAttr.Get()
                startPos[0] = startPos[0] + (5.0 * lengthScale * (i + 1))
                posAttr.Set(startPos)
                startPosList.append(startPos)

                vehicleController = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)

                if vehicleData.driveTypeIndex == VehicleWizard.DRIVE_TYPE_NONE:
                    driveTorque = 500.0 * lengthScale * lengthScale
                    wheelPath0 = vehiclePath + "/LeftWheel1References"
                    prim0 = stage.GetPrimAtPath(wheelPath0)
                    wheelController0 = PhysxSchema.PhysxVehicleWheelControllerAPI(prim0)
                    wheelController0.GetDriveTorqueAttr().Set(driveTorque)

                    wheelPath1 = vehiclePath + "/RightWheel1References"
                    prim1 = stage.GetPrimAtPath(wheelPath1)
                    wheelController1 = PhysxSchema.PhysxVehicleWheelControllerAPI(prim1)
                    wheelController1.GetDriveTorqueAttr().Set(driveTorque)
                else:
                    vehicleController.GetAcceleratorAttr().Set(1.0)
                    vehicleController.GetTargetGearAttr().Set(1)
            else:
                # Or remove one.
                index = random.randint(0, len(vehicleList) - 1)
                stage.RemovePrim(vehicleList[index])
                vehicleList.remove(vehicleList[index])
                del startPosList[index]

            self._simulate_one_frame()

        secondsToRun = 2.0
        self._simulate(secondsToRun)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        for i in range(len(vehicleList)):
            # Check vehicle position
            vehiclePath = vehicleList[i] + "/Vehicle"
            vehiclePrim = stage.GetPrimAtPath(vehiclePath)
            endPos = vehiclePrim.GetAttribute("xformOp:translate").Get()
            motionVector = endPos - startPosList[i]

            errTolVertical = 0.001 * lengthScale
            errTolLateral = 0.01 * lengthScale
            self.assertLess(math.fabs(motionVector[0]), errTolLateral)
            self.assertLess(math.fabs(motionVector[1]), errTolVertical)
            self.assertGreater(motionVector[2], 1.0 * lengthScale)

        await self.new_stage()

    #
    # Determine what the minimum number of vehicle prims are needed before the simulation is started
    # to get a vehicle created successfully. Only a scene should be required.
    #
    async def test_dynamic_vehicle_minimums(self):
        stage = await self.new_stage()

        vehicleData = VehicleWizard.VehicleData(get_unit_scale(stage),
            VehicleWizard.VehicleData.AXIS_Y, VehicleWizard.VehicleData.AXIS_Z)

        # Place the vehicle under the default prim.
        defaultPath = str(stage.GetDefaultPrim().GetPath())
        vehicleData.rootVehiclePath = defaultPath + VehicleWizard.VEHICLE_ROOT_BASE_PATH
        vehicleData.rootSharedPath = defaultPath + VehicleWizard.SHARED_DATA_ROOT_BASE_PATH

        # A scene is required before the simulation is started and it must have
        # PhysxVehicleContextAPI applied.
        gravityDirection = -vehicleData.get_vertical_axis_vector()
        gravityMagnitude = 10 * vehicleData.unitScale.lengthScale

        scenePath = vehicleData.rootSharedPath + VehicleWizard.DEFAULT_SCENE_PATH
        scene = UsdPhysics.Scene.Define(stage, scenePath)
        scene.CreateGravityDirectionAttr(gravityDirection)
        scene.CreateGravityMagnitudeAttr(gravityMagnitude)

        vehicleContext = PhysxSchema.PhysxVehicleContextAPI.Apply(scene.GetPrim())
        vehicleContext.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
        vehicleContext.CreateVerticalAxisAttr(PhysxSchema.Tokens.posY)
        vehicleContext.CreateLongitudinalAxisAttr(PhysxSchema.Tokens.posZ)

        # Run the simulation to make sure the vehicle moves.
        self._prepare_for_simulation()
        secondsToRun = 4.0

        # Create one vehicle to set up the simulation.
        (success, (messageList, _)) = PhysXVehicleWizardCreateCommand.execute(vehicleData)
        self.assertTrue(success)
        self.assertFalse(messageList)  # python style guide to check for empty list

        # Ensure the default vehicle was created.
        vehiclePath = vehicleData.rootVehiclePath + "/Vehicle"
        vehiclePrim = stage.GetPrimAtPath(vehiclePath)
        self.assertTrue(vehiclePrim.HasAPI(PhysxSchema.PhysxVehicleAPI))
        startPos = vehiclePrim.GetAttribute("xformOp:translate").Get()

        vehicleController = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)
        vehicleController.GetAcceleratorAttr().Set(1.0)

        # Create a ground plane to drive on.
        collisionGroupPath = vehicleData.rootSharedPath + VehicleWizard.VEHICLE_COLLISION_GROUP_GROUND_SURFACE_PATH
        groundMaterialPath = vehicleData.rootSharedPath + VehicleWizard.VEHICLE_GROUND_MATERIAL_PATH
        Factory.createGroundPlane(stage, vehicleData.unitScale, collisionGroupPath, groundMaterialPath)

        # Run the simulation to make sure the vehicle moves.
        self._simulate(secondsToRun)
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        # Check vehicle position
        endPos = vehiclePrim.GetAttribute("xformOp:translate").Get()
        motionVector = endPos - startPos

        # print(str(endPos[0]) + "," + str(endPos[1]) + "," + str(endPos[2]))

        errTolVertical = 0.001 * vehicleData.unitScale.lengthScale
        errTolLateral = 0.005 * vehicleData.unitScale.lengthScale
        self.assertLess(math.fabs(motionVector[0]), errTolLateral)
        self.assertLess(math.fabs(motionVector[1]), errTolVertical)
        self.assertGreater(motionVector[2], 3.0 * vehicleData.unitScale.lengthScale)

        await self.new_stage()

    #
    # Test that adding a vehicle while the sim is running does work even if the vehicle is a child of
    # the added prim.
    #
    async def test_dynamic_vehicle_add_from_tree(self):
        stage = await self.new_stage()

        rootPath = str(stage.GetDefaultPrim().GetPath())
        vehicleRootPath0 = rootPath + "/VehicleRoot0"
        UsdGeom.Scope.Define(stage, vehicleRootPath0)
        vehiclePrimSubPath = "/Vehicle"
        vehiclePaths = [vehicleRootPath0 + vehiclePrimSubPath]
        wheelAttachmentPaths = []

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            1,
            driveMode=Factory.DRIVE_BASIC,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
            addChassisCollisionBox=False,
            useShareableComponentsList=[False],
            vehiclePathsIn=vehiclePaths
        )

        # since a reference to the vehicle will be created, all relationships pointing outside that
        # referenced root prim will be invalid and trigger a warning message. Thus, the relationships
        # are cleared now.
        for wheelAttPath in wheelAttachmentPaths[0]:
            wheelAttPrim = stage.GetPrimAtPath(wheelAttPath)
            wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(wheelAttPrim)
            wheelAttAPI.GetCollisionGroupRel().ClearTargets(True)
            tireAPI = PhysxSchema.PhysxVehicleTireAPI(wheelAttPrim)
            tireAPI.GetFrictionTableRel().ClearTargets(True)

        self._prepare_for_simulation()

        # creating a reference to the existing vehicle to have only the notification
        # about the root prim being added
        vehicleRootPath1 = rootPath + "/VehicleRoot1"
        vehicleRootPrim = UsdGeom.Scope.Define(stage, vehicleRootPath1).GetPrim()
        vehicleRootPrim.GetReferences().AddInternalReference(vehicleRootPath0)
        vehiclePath1 = vehicleRootPath1 + vehiclePrimSubPath
        vehiclePrim = stage.GetPrimAtPath(vehiclePath1)

        vehicleController = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)
        vehicleController.GetAcceleratorAttr().Set(1.0)

        posAttr = vehiclePrim.GetAttribute("xformOp:translate")
        startPos = posAttr.Get()
        startPos[0] = startPos[0] - 3.0
        posAttr.Set(startPos)

        self._simulate_one_frame()

        # make sure that the objects of the vehicle extension have been created too
        global gPhysXVehicleInterface
        inputEnabled = gPhysXVehicleInterface.get_input_enabled(vehiclePath1)
        self.assertFalse(inputEnabled)
        gPhysXVehicleInterface.set_input_enabled(vehiclePath1, True)
        inputEnabled = gPhysXVehicleInterface.get_input_enabled(vehiclePath1)
        self.assertTrue(inputEnabled)

        secondsToRun = 1.0
        self._simulate(secondsToRun)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)
        endPos = posAttr.Get()
        delta = endPos - startPos
        errTol = 0.001
        self.assertLess(math.fabs(delta[0]), errTol)
        self.assertLess(math.fabs(delta[1]), errTol)
        self.assertGreater(delta[2], 0.3)  # the vehicle should have moved forward

        await self.new_stage()

    async def test_vehicle_wizard_units(self):
        chassisLength = 4.0

        expectedDistance = 0.0

        for j in range(2):
            if j == 0:
                kilogramsPerUnit = 0.01
            else:
                kilogramsPerUnit = 1.0
            for i in range(2):
                if i == 0:
                    metersPerUnit = 0.01
                else:
                    metersPerUnit = 1.0

                stage = await self.new_stage()
                UsdGeom.SetStageMetersPerUnit(stage, metersPerUnit)
                UsdPhysics.SetStageKilogramsPerUnit(stage, kilogramsPerUnit)
                unitScale = get_unit_scale(stage)

                vehicleDataManager = VehicleWizard.VehicleDataManager(unitScale,
                    VehicleWizard.VehicleData.AXIS_Y, VehicleWizard.VehicleData.AXIS_X)

                # Place the vehicle under the default prim.
                defaultPath = str(stage.GetDefaultPrim().GetPath())
                vehicleDataManager.vehicleData.rootVehiclePath = defaultPath + VehicleWizard.VEHICLE_ROOT_BASE_PATH
                vehicleDataManager.vehicleData.rootSharedPath = defaultPath + VehicleWizard.SHARED_DATA_ROOT_BASE_PATH
                vehiclePath = vehicleDataManager.vehicleData.rootVehiclePath + "/Vehicle"

                vehicleDataManager.vehicleData.chassisLength = chassisLength * vehicleDataManager.vehicleData.unitScale.lengthScale
                vehicleDataManager.vehicleData.chassisWidth = vehicleDataManager.vehicleData.chassisLength / 2
                vehicleDataManager.vehicleData.chassisHeight = vehicleDataManager.vehicleData.chassisWidth / 2
                vehicleDataManager.update()

                (success, (messageList, _)) = PhysXVehicleWizardCreateCommand.execute(vehicleDataManager.vehicleData)
                self.assertTrue(success)
                self.assertFalse(messageList)  # python style guide to check for empty list

                # Create a ground plane to drive on.
                collisionGroupPath = vehicleDataManager.vehicleData.rootSharedPath + VehicleWizard.VEHICLE_COLLISION_GROUP_GROUND_SURFACE_PATH
                groundMaterialPath = vehicleDataManager.vehicleData.rootSharedPath + VehicleWizard.VEHICLE_GROUND_MATERIAL_PATH
                Factory.createGroundPlane(stage, unitScale, collisionGroupPath, groundMaterialPath)

                # Drive forward.
                vehiclePrim = stage.GetPrimAtPath(vehiclePath)
                vehicleController = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)
                vehicleController.GetAcceleratorAttr().Set(1.0)

                # Run the simulation to make sure the vehicle is stable.
                secondsToRun = 5.0
                self._prepare_for_simulation()
                self._simulate(secondsToRun)

                self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

                # Check vehicle position
                endPos = vehiclePrim.GetAttribute("xformOp:translate").Get()

                if i == 0:
                    expectedDistance = endPos[2] / unitScale.lengthScale
                else:
                    self.assertLess(math.fabs(endPos[2] - expectedDistance), 0.01)

        await self.new_stage()

    async def _test_wizard_various_vehicle_sizes(self, driveModeIn):
        chassisLengthInMetersList = [4.0, 2.0, 1.0, 0.5, 0.2, 0.1, 0.05]

        for chassisLengthInMeters in chassisLengthInMetersList:
            stage = await self.new_stage()

            metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)
            lengthScale = 1.0 / metersPerUnit

            chassisLength = chassisLengthInMeters * lengthScale

            vehicleDataManager = VehicleWizard.VehicleDataManager(get_unit_scale(stage),
                VehicleWizard.VehicleData.AXIS_Y, VehicleWizard.VehicleData.AXIS_Z)
            vehicleDataManager.vehicleData.createShareableComponents = False

            vehicleDataManager.set_drive_type(driveModeIn)

            # Place the vehicle under the default prim.
            defaultPath = str(stage.GetDefaultPrim().GetPath())
            vehicleDataManager.vehicleData.rootVehiclePath = defaultPath + VehicleWizard.VEHICLE_ROOT_BASE_PATH
            vehicleDataManager.vehicleData.rootSharedPath = defaultPath + VehicleWizard.SHARED_DATA_ROOT_BASE_PATH
            vehiclePath = vehicleDataManager.vehicleData.rootVehiclePath + "/Vehicle"

            vehicleDataManager.set_chassis_length(chassisLength)
            vehicleDataManager.set_chassis_width(vehicleDataManager.vehicleData.chassisLength / 2)
            vehicleDataManager.set_chassis_height(vehicleDataManager.vehicleData.chassisWidth / 2)
            vehicleDataManager.update()

            rideHeight = 0.5 * vehicleDataManager.vehicleData.chassisHeight + vehicleDataManager.vehicleData.tireRadius

            # Set the simulation rate
            stepsPerSecond = 60
            scenePath = defaultPath + VehicleWizard.DEFAULT_SCENE_PATH
            scene = UsdPhysics.Scene.Define(stage, scenePath)
            scenePrim = scene.GetPrim()
            sceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scenePrim)
            sceneAPI.GetTimeStepsPerSecondAttr().Set(stepsPerSecond)

            (success, (messageList, _)) = PhysXVehicleWizardCreateCommand.execute(vehicleDataManager.vehicleData)
            self.assertTrue(success)
            self.assertFalse(messageList)  # python style guide to check for empty list

            # Create a ground plane to drive on.
            collisionGroupPath = vehicleDataManager.vehicleData.rootSharedPath + VehicleWizard.VEHICLE_COLLISION_GROUP_GROUND_SURFACE_PATH
            groundMaterialPath = vehicleDataManager.vehicleData.rootSharedPath + VehicleWizard.VEHICLE_GROUND_MATERIAL_PATH
            Factory.createGroundPlane(stage, vehicleDataManager.vehicleData.unitScale, collisionGroupPath, groundMaterialPath)

            # Ensure the default vehicle was created.
            vehiclePrim = stage.GetPrimAtPath(vehiclePath)
            self.assertTrue(vehiclePrim.HasAPI(PhysxSchema.PhysxRigidBodyAPI))
            self.assertTrue(vehiclePrim.HasAPI(PhysxSchema.PhysxVehicleAPI))

            # Turn off sleep modes
            # N.B. This changes the physics behavior when it should not. Investigate what is happening.
            rigidBodyAPI = PhysxSchema.PhysxRigidBodyAPI(vehiclePrim)
            rigidBodyAPI.GetSleepThresholdAttr().Set(0)
            rigidBodyAPI.GetStabilizationThresholdAttr().Set(0)

            # Run the simulation to make sure the vehicle is stable.
            secondsToRun = 1.0
            stepsToSimulate = int(secondsToRun * stepsPerSecond)
            timeStep = 1.0 / stepsPerSecond
            self._prepare_for_simulation()

            errorTolerancePercent = 0.01
            horizontalErrTol = errorTolerancePercent * chassisLength
            verticalErrTol = errorTolerancePercent * rideHeight

            expectedPos = (0, rideHeight, 0)

            for t in range(stepsToSimulate):
                self._simulate_one_frame(timeStep)
                self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

                # Check vehicle position
                endPos = vehiclePrim.GetAttribute("xformOp:translate").Get()

                self.assertLess(math.fabs(endPos[0] - expectedPos[0]), horizontalErrTol)
                self.assertLess(math.fabs(endPos[1] - expectedPos[1]), verticalErrTol)
                self.assertLess(math.fabs(endPos[2] - expectedPos[2]), horizontalErrTol)

            # Run the simulation to ensure the vehicle drives properly.
            vehicleController = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)
            vehicleController.GetAcceleratorAttr().Set(1.0)

            secondsToRun = 10.0
            stepsToSimulate = int(secondsToRun * stepsPerSecond)

            lastEndPos = endPos

            # the total lateral drift should remain small. Standard drive has significantly larger drift,
            # especially since the logic has changed to start in first gear and as a result the vehicle
            # gets further.
            if (driveModeIn == VehicleWizard.DRIVE_TYPE_STANDARD):
                lateralErrTol = 0.16 * chassisLength
                lateralErrTolPerStep = 2.0 * (lateralErrTol / stepsToSimulate)
            else:
                lateralErrTol = 0.02 * chassisLength
                lateralErrTolPerStep = 2.0 * (lateralErrTol / stepsToSimulate)

            for t in range(stepsToSimulate):
                self._simulate_one_frame(timeStep)
                self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

                # Check vehicle position
                endPos = vehiclePrim.GetAttribute("xformOp:translate").Get()

                # Ensure the vehicle remains stable and moves forward
                self.assertLess(math.fabs(lastEndPos[0] - endPos[0]), lateralErrTolPerStep)
                self.assertLess(math.fabs(endPos[1] - expectedPos[1]), verticalErrTol)
                self.assertGreater(endPos[2], lastEndPos[2] - horizontalErrTol)

                lastEndPos = endPos

            self.assertGreater(endPos[2], 10.0 * chassisLength)
            self.assertLess(math.fabs(endPos[0]), lateralErrTol)

        await self.new_stage()

    #
    # verify that the vehicles created by the wizard are stable at rest and when
    # accelerating. Test this for various vehicle sizes and standard drive.
    #
    async def test_wizard_various_vehicle_sizes_drive_standard(self):
        await self._test_wizard_various_vehicle_sizes(VehicleWizard.DRIVE_TYPE_STANDARD)

    #
    # verify that the vehicles created by the wizard are stable at rest and when
    # accelerating. Test this for various vehicle sizes and basic drive.
    #
    async def test_wizard_various_vehicle_sizes_drive_basic(self):
        await self._test_wizard_various_vehicle_sizes(VehicleWizard.DRIVE_TYPE_BASIC)

    #
    # verify that the wizard sets sleep thresholds such that small vehicles can accelerate
    # as expected without falling asleep every few frames.
    #
    async def test_wizard_sleep_threshold(self):
        chassisLengthList = [0.2, 0.1, 0.05]

        for chassisLength in chassisLengthList:
            stage = await self.new_stage()

            unitScale = get_unit_scale(stage)
            chassisLength = chassisLength * unitScale.lengthScale

            vehicleDataManager = VehicleWizard.VehicleDataManager(unitScale,
                VehicleWizard.VehicleData.AXIS_Y, VehicleWizard.VehicleData.AXIS_Z)
            vehicleDataManager.vehicleData.createShareableComponents = False

            vehicleDataManager.set_drive_type(VehicleWizard.DRIVE_TYPE_BASIC)

            # Place the vehicle under the default prim.
            vehiclePrimPrefix = "/Vehicle"
            defaultPath = str(stage.GetDefaultPrim().GetPath())
            vehicleDataManager.vehicleData.rootVehiclePath = defaultPath + VehicleWizard.VEHICLE_ROOT_BASE_PATH + "0"
            vehicleDataManager.vehicleData.rootSharedPath = defaultPath + VehicleWizard.SHARED_DATA_ROOT_BASE_PATH
            vehiclePath0 = vehicleDataManager.vehicleData.rootVehiclePath + vehiclePrimPrefix

            vehicleDataManager.set_chassis_length(chassisLength)
            vehicleDataManager.set_chassis_width(chassisLength / 2)
            vehicleDataManager.set_chassis_height(chassisLength / 4)
            vehicleDataManager.update()

            # Set the simulation rate
            scenePath = defaultPath + VehicleWizard.DEFAULT_SCENE_PATH
            scene = UsdPhysics.Scene.Define(stage, scenePath)
            scenePrim = scene.GetPrim()
            sceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scenePrim)
            stepsPerSecond = 60
            sceneAPI.GetTimeStepsPerSecondAttr().Set(stepsPerSecond)

            (success, (messageList, _)) = PhysXVehicleWizardCreateCommand.execute(vehicleDataManager.vehicleData)
            self.assertTrue(success)
            self.assertFalse(messageList)  # python style guide to check for empty list

            vehicleDataManager.vehicleData.rootVehiclePath = defaultPath + VehicleWizard.VEHICLE_ROOT_BASE_PATH + "1"
            vehiclePath1 = vehicleDataManager.vehicleData.rootVehiclePath + vehiclePrimPrefix
            (success, (messageList, _)) = PhysXVehicleWizardCreateCommand.execute(vehicleDataManager.vehicleData)
            self.assertTrue(success)
            self.assertFalse(messageList)  # python style guide to check for empty list

            vehiclePrim0 = stage.GetPrimAtPath(vehiclePath0)
            pos0Attr = vehiclePrim0.GetAttribute("xformOp:translate")
            vehiclePrim1 = stage.GetPrimAtPath(vehiclePath1)
            pos1Attr = vehiclePrim1.GetAttribute("xformOp:translate")

            pos1 = pos1Attr.Get()
            pos1[0] = pos1[0] + (2.0 * vehicleDataManager.vehicleData.chassisWidth)
            pos1Attr.Set(pos1)

            # Create a ground plane to drive on.
            collisionGroupPath = vehicleDataManager.vehicleData.rootSharedPath + VehicleWizard.VEHICLE_COLLISION_GROUP_GROUND_SURFACE_PATH
            groundMaterialPath = vehicleDataManager.vehicleData.rootSharedPath + VehicleWizard.VEHICLE_GROUND_MATERIAL_PATH
            Factory.createGroundPlane(stage, unitScale, collisionGroupPath, groundMaterialPath)

            # Set sleep threshold to 0 for vehicle 0
            rigidBodyAPI = PhysxSchema.PhysxRigidBodyAPI(vehiclePrim0)
            rigidBodyAPI.GetSleepThresholdAttr().Set(0)
            rigidBodyAPI.GetStabilizationThresholdAttr().Set(0)

            vehicleController0 = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim0)
            vehicleController0.GetAcceleratorAttr().Set(1.0)
            vehicleController1 = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim1)
            vehicleController1.GetAcceleratorAttr().Set(1.0)

            # Run the simulation and compare travelled distance of the two vehicles
            secondsToRun = 1.0
            stepsToSimulate = int(secondsToRun * stepsPerSecond)
            timeStep = 1.0 / stepsPerSecond
            self._prepare_for_simulation()

            startPos0 = pos0Attr.Get()
            startPos1 = pos1Attr.Get()

            for t in range(stepsToSimulate):
                self._simulate_one_frame(timeStep)

            self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

            endPos0 = pos0Attr.Get()
            endPos1 = pos1Attr.Get()

            delta0 = endPos0 - startPos0
            delta1 = endPos1 - startPos1

            # The vehicle with sleeping disabled should not move further than the one with sleeping enabled
            self.assertLess(math.fabs(delta0[2] - delta1[2]), chassisLength * 0.01)
            self.assertGreater(delta0[2], 0.5 * chassisLength)  # just to make sure the vehicle moved at all

        await self.new_stage()

    #
    # verify that the brake behavior is similar for various sized vehicles created via the
    # wizard.
    #
    async def test_wizard_brake_torque(self):
        stage = await self.new_stage()

        unitScale = get_unit_scale(stage)

        chassisLengthList = [4.0, 1.0, 0.2, 0.1]
        vehicleCount = len(chassisLengthList)
        accelerateToTargetVel = False  # accelerate to target velocity or set it directly
        speedAtBrake = 2.0 * unitScale.lengthScale
        minSecondsToRunWithBrake = 1.0
        maxSecondsToAccelerate = 10.0

        vehicleDataManager = VehicleWizard.VehicleDataManager(unitScale,
            VehicleWizard.VehicleData.AXIS_Y, VehicleWizard.VehicleData.AXIS_Z)
        vehicleDataManager.vehicleData.createShareableComponents = False

        #driveType = VehicleWizard.DRIVE_TYPE_STANDARD
        driveType = VehicleWizard.DRIVE_TYPE_BASIC
        vehicleDataManager.set_drive_type(driveType)

        # Place the vehicle under the default prim.
        vehiclePrimPrefix = "/Vehicle"
        defaultPath = str(stage.GetDefaultPrim().GetPath())
        vehicleDataManager.vehicleData.rootSharedPath = defaultPath + VehicleWizard.SHARED_DATA_ROOT_BASE_PATH

        # Set the simulation rate
        scenePath = defaultPath + VehicleWizard.DEFAULT_SCENE_PATH
        scene = UsdPhysics.Scene.Define(stage, scenePath)
        scenePrim = scene.GetPrim()
        sceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scenePrim)
        stepsPerSecond = 60
        sceneAPI.GetTimeStepsPerSecondAttr().Set(stepsPerSecond)

        maxChassisLength = -1.0
        for chassisLength in chassisLengthList:
            maxChassisLength = max(maxChassisLength, chassisLength * unitScale.lengthScale)

        vehiclePaths = []
        vehiclePrims = []
        vehicleWheelRadii = []
        vehiclePosAttrs = []
        startPosList = []
        sidePos = 0.0
        for i in range(vehicleCount):
            chassisLength = chassisLengthList[i] * unitScale.lengthScale
            vehicleRootPath = defaultPath + VehicleWizard.VEHICLE_ROOT_BASE_PATH + str(i)
            vehicleDataManager.vehicleData.rootVehiclePath = vehicleRootPath
            vehiclePath = vehicleRootPath + vehiclePrimPrefix
            vehiclePaths.append(vehiclePath)

            vehicleDataManager.set_chassis_length(chassisLength)
            vehicleDataManager.set_chassis_width(chassisLength / 2)
            vehicleDataManager.set_chassis_height(chassisLength / 4)
            vehicleDataManager.update()
            vehicleWheelRadii.append(vehicleDataManager.vehicleData.tireRadius)

            (success, (messageList, _)) = PhysXVehicleWizardCreateCommand.execute(vehicleDataManager.vehicleData)
            self.assertTrue(success)
            self.assertFalse(messageList)  # python style guide to check for empty list

            vehiclePrim = stage.GetPrimAtPath(vehiclePath)
            vehiclePrims.append(vehiclePrim)
            posAttr = vehiclePrim.GetAttribute("xformOp:translate")
            vehiclePosAttrs.append(posAttr)
            pos = posAttr.Get()
            pos[0] = sidePos
            startPosList.append(pos)
            posAttr.Set(pos)

            sidePos = sidePos + maxChassisLength

        # Create a ground plane to drive on.
        collisionGroupPath = vehicleDataManager.vehicleData.rootSharedPath + VehicleWizard.VEHICLE_COLLISION_GROUP_GROUND_SURFACE_PATH
        groundMaterialPath = vehicleDataManager.vehicleData.rootSharedPath + VehicleWizard.VEHICLE_GROUND_MATERIAL_PATH
        Factory.createGroundPlane(stage, unitScale, collisionGroupPath, groundMaterialPath)

        global gPhysXInterface

        self._prepare_for_simulation()

        wheelSubPaths = ["/LeftWheel1References", "/RightWheel1References"]
        for i in range(vehicleCount):
            physxRigidBodyAPI = PhysxSchema.PhysxRigidBodyAPI(vehiclePrims[i])
            physxRigidBodyAPI.GetSleepThresholdAttr().Set(0)
            physxRigidBodyAPI.GetStabilizationThresholdAttr().Set(0)

            if (not accelerateToTargetVel):
                rigidBodyAPI = UsdPhysics.RigidBodyAPI(vehiclePrims[i])
                rigidBodyAPI.GetVelocityAttr().Set(Gf.Vec3f(0, 0, speedAtBrake))

                rotationSpeed = speedAtBrake / vehicleWheelRadii[i]  # assuming free rolling

                for wheelSubPath in wheelSubPaths:

                    wheelAttPath = vehiclePaths[i] + wheelSubPath
                    wheelAttPrim = stage.GetPrimAtPath(wheelAttPath)

                    gPhysXInterface.set_wheel_rotation_speed(wheelAttPath, rotationSpeed)

        # Run the simulation and compare travelled distances of the vehicles
        minStepsToSimulateForBrake = int(minSecondsToRunWithBrake * stepsPerSecond)
        maxStepsToSimulateForAccel = int(maxSecondsToAccelerate * stepsPerSecond)
        timeStep = 1.0 / stepsPerSecond

        self._simulate_one_frame(timeStep)

        if (accelerateToTargetVel):
            for i in range(vehicleCount):
                vehicleController = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrims[i])
                vehicleController.GetAcceleratorAttr().Set(1.0)
                if (driveType == VehicleWizard.DRIVE_TYPE_STANDARD):
                    vehicleController.GetTargetGearAttr().Set(1)

            speedReachedCount = 0
            speedReached = [False] * vehicleCount
            step = 0
            while (speedReachedCount < vehicleCount) and (step < maxStepsToSimulateForAccel):
                self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=True)
                for i in range(vehicleCount):
                    if (not speedReached[i]):
                        rigidBodyAPI = UsdPhysics.RigidBodyAPI(vehiclePrims[i])
                        vel = rigidBodyAPI.GetVelocityAttr().Get()[2]
                        if (vel >= speedAtBrake):
                            speedReachedCount = speedReachedCount + 1
                            speedReached[i] = True
                            vehicleController = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrims[i])
                            vehicleController.GetAcceleratorAttr().Set(0.0)
                            vehicleController.GetBrake0Attr().Set(1.0)
                            startPosList[i] = vehiclePosAttrs[i].Get()

                self._simulate_one_frame(timeStep)
                step = step + 1
        else:
            for i in range(vehicleCount):
                vehicleController = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrims[i])
                vehicleController.GetBrake0Attr().Set(1.0)

        for i in range(vehicleCount):
            vehicleController = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrims[i])
            vehicleController.GetAcceleratorAttr().Set(0.0001)  # avoid sticky tire friction kicking in

        for t in range(minStepsToSimulateForBrake):
            self._simulate_one_frame(timeStep)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        deltas = []
        for i in range(vehicleCount):
            delta = vehiclePosAttrs[i].Get() - startPosList[i]
            deltas.append(delta)

            chassisLength = chassisLengthList[i] * unitScale.lengthScale
            self.assertLess(math.fabs(delta[0]), chassisLength * 0.01)  # make sure there is no significant side drift

        for i in range(vehicleCount - 1):
            # With the current wizard scaling approach, smaller vehicles accelerate faster to reach a
            # higher max rotation speed faster (and thus similar top speed). However, the values
            # are chosen not to have a 100% match, that is, smaller vehicles still have smaller top
            # speed and take longer to reach it. Similarly, brake torque is more aggressive for smaller
            # vehicles but it will still take longer to brake to 0 given the same speed. This test makes
            # sure that is the case.
            # Note: for vehicles below 10cm, density values change and thus this does not apply the
            #       same way anymore
            self.assertGreater(deltas[i+1][2], 1.1 * deltas[i][2])

        await self.new_stage()

    #
    # verify that the OgnVehicleGetDriveState node is providing the expected info
    #
    async def test_ogn_get_drive_and_wheel_state(self):
        import array as arr

        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_STANDARD,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths
        )

        prim = stage.GetPrimAtPath(vehiclePaths[0])
        vehicleController = PhysxSchema.PhysxVehicleControllerAPI(prim)
        vehicleController.GetAcceleratorAttr().Set(1.0)
        vehicleController.GetSteerAttr().Set(0.1)

        driveStateNodeName = "driveState"
        wheelStateNodeName = "wheelState"

        ogController = omni.graph.core.Controller()

        # "dirty_push" gives more control over when the nodes of a graph are evaluated
        (graph, (driveStateNode, wheelStateNode,), _, _) = ogController.edit(
            {
                "graph_path": "/TestGraph",
                "evaluator_name": "dirty_push"
            },
            {
                ogController.Keys.CREATE_NODES: [
                    (driveStateNodeName, "omni.physx.graph.VehicleGetDriveState"),
                    (wheelStateNodeName, "omni.physx.graph.VehicleGetWheelState")
                ],
                ogController.Keys.SET_VALUES: [
                    (driveStateNodeName + ".inputs:vehiclePath", vehiclePaths[0]),
                    (wheelStateNodeName + ".inputs:vehicleWheelAttachmentPaths", wheelAttachmentPaths[0])
                ]
            }
        )

        #print(dir(graph))

        self._prepare_for_simulation()

        global gPhysXInterface
        previousEngSpeed = 0
        for i in range(0, 3):
            self._simulate_one_frame()

            await ogController.evaluate(graph)
            driveStateNode.set_compute_incomplete()
            wheelStateNode.set_compute_incomplete()
            #driveStateNode.request_compute()  #!!! this is supposed to be the right thing to use but does not work
            #wheelStateNode.request_compute()  #!!! this is supposed to be the right thing to use but does not work

            #get the engine rotation speed from the node and from the interface so that we can compare them
            #to determine that the node is working properly.
            currentEngSpeed = driveStateNode.get_attribute("outputs:engineRotationSpeed").get()
            driveState = gPhysXInterface.get_vehicle_drive_state(vehiclePaths[0])
            refEngSpeed = driveState[VEHICLE_DRIVE_STATE_ENGINE_ROTATION_SPEED]
            self.assertGreater(currentEngSpeed, previousEngSpeed)
            self.assertAlmostEqual(currentEngSpeed, refEngSpeed, delta=0.0001)
            previousEngSpeed = currentEngSpeed

            currentAccelerator = driveStateNode.get_attribute("outputs:accelerator").get()
            refAccelerator = driveState[VEHICLE_DRIVE_STATE_ACCELERATOR]
            self.assertAlmostEqual(currentAccelerator, refAccelerator, delta=0.0001)

            currentBrake = driveStateNode.get_attribute("outputs:brake0").get()
            refBrake = driveState[VEHICLE_DRIVE_STATE_BRAKE0]
            self.assertAlmostEqual(currentBrake, refBrake, delta=0.0001)

            currentSteer = driveStateNode.get_attribute("outputs:steer").get()
            refSteer = driveState[VEHICLE_DRIVE_STATE_STEER]
            self.assertAlmostEqual(currentSteer, refSteer, delta=0.0001)

            currentGear = driveStateNode.get_attribute("outputs:currentGear").get()
            refGear = driveState[VEHICLE_DRIVE_STATE_CURRENT_GEAR]
            self.assertEqual(currentSteer, refSteer)

            targetGear = driveStateNode.get_attribute("outputs:targetGear").get()
            refTargetGear = driveState[VEHICLE_DRIVE_STATE_TARGET_GEAR]
            self.assertEqual(targetGear, refTargetGear)

            currentAutobox = driveStateNode.get_attribute("outputs:automaticTransmission").get()
            refAutobox = driveState[VEHICLE_DRIVE_STATE_AUTOMATIC_TRANSMISSION]
            self.assertEqual(currentAutobox, refAutobox)

            #get a wheel rotation speed from the node and from the interface so that we can compare them
            #to determine that the node is working properly.
            for wheelId in range(0, 4):

                wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[0][wheelId])

                currentWheelRotationSpeed = wheelStateNode.get_attribute("outputs:wheelRotationSpeeds").get()[wheelId]
                refWheelRotationSpeed = wheelState[VEHICLE_WHEEL_STATE_ROTATION_SPEED]
                self.assertAlmostEqual(currentWheelRotationSpeed, refWheelRotationSpeed, delta=0.0001)

                currentWheelRotationAngle = wheelStateNode.get_attribute("outputs:wheelRotationAngles").get()[wheelId]
                refWheelRotationAngle = wheelState[VEHICLE_WHEEL_STATE_ROTATION_ANGLE]
                self.assertAlmostEqual(currentWheelRotationAngle, refWheelRotationAngle, delta=0.0001)

                currentWheelSteerAngle = wheelStateNode.get_attribute("outputs:wheelSteerAngles").get()[wheelId]
                refWheelSteerAngle = wheelState[VEHICLE_WHEEL_STATE_STEER_ANGLE]
                self.assertAlmostEqual(currentWheelSteerAngle, refWheelSteerAngle, delta=0.0001)

                currentIsOnGround = wheelStateNode.get_attribute("outputs:groundContactStates").get()[wheelId]
                refIsOnGround = wheelState[VEHICLE_WHEEL_STATE_IS_ON_GROUND]
                self.assertEqual(currentIsOnGround, refIsOnGround)

                currentGroundPlane = wheelStateNode.get_attribute("outputs:groundPlanes").get()[wheelId]
                refGroundPlane = wheelState[VEHICLE_WHEEL_STATE_GROUND_PLANE]
                self.assertAlmostEqual(currentGroundPlane[0], refGroundPlane[0], 0.0001)
                self.assertAlmostEqual(currentGroundPlane[1], refGroundPlane[1], 0.0001)
                self.assertAlmostEqual(currentGroundPlane[2], refGroundPlane[2], 0.0001)
                self.assertAlmostEqual(currentGroundPlane[3], refGroundPlane[3], 0.0001)

                currentGroundPos = wheelStateNode.get_attribute("outputs:groundHitPositions").get()[wheelId]
                refGroundPos = wheelState[VEHICLE_WHEEL_STATE_GROUND_HIT_POSITION]
                self.assertAlmostEqual(currentGroundPos[0], refGroundPos[0], 0.0001)
                self.assertAlmostEqual(currentGroundPos[1], refGroundPos[1], 0.0001)
                self.assertAlmostEqual(currentGroundPos[2], refGroundPos[2], 0.0001)

                currentGroundPhysXActor = wheelStateNode.get_attribute("outputs:groundPhysXActors").get()[wheelId]
                refGroundPhysXActor = wheelState[VEHICLE_WHEEL_STATE_GROUND_ACTOR]
                self.assertEqual(currentGroundPhysXActor, refGroundPhysXActor)

                currentGroundPhysXShape = wheelStateNode.get_attribute("outputs:groundPhysXShapes").get()[wheelId]
                refGroundPhysXShape = wheelState[VEHICLE_WHEEL_STATE_GROUND_SHAPE]
                self.assertEqual(currentGroundPhysXShape, refGroundPhysXShape)

                currentGroundPhysXMaterial = wheelStateNode.get_attribute("outputs:groundPhysXMaterials").get()[wheelId]
                refGroundPhysXMaterial = wheelState[VEHICLE_WHEEL_STATE_GROUND_MATERIAL]
                self.assertEqual(currentGroundPhysXMaterial, refGroundPhysXMaterial)

                currentSuspensionJounce = wheelStateNode.get_attribute("outputs:suspensionJounces").get()[wheelId]
                refSuspensionJounce = wheelState[VEHICLE_WHEEL_STATE_SUSPENSION_JOUNCE]
                self.assertAlmostEqual(currentSuspensionJounce, refSuspensionJounce, 0.0001)

                currentSuspensionForce = wheelStateNode.get_attribute("outputs:suspensionForces").get()[wheelId]
                refSuspensionForce = wheelState[VEHICLE_WHEEL_STATE_SUSPENSION_FORCE]
                self.assertAlmostEqual(currentSuspensionForce[0], refSuspensionForce[0], 0.0001)
                self.assertAlmostEqual(currentSuspensionForce[1], refSuspensionForce[1], 0.0001)
                self.assertAlmostEqual(currentSuspensionForce[2], refSuspensionForce[2], 0.0001)

                currentTireFriction = wheelStateNode.get_attribute("outputs:tireFrictions").get()[wheelId]
                refTireFriction = wheelState[VEHICLE_WHEEL_STATE_TIRE_FRICTION]
                self.assertAlmostEqual(currentTireFriction, refTireFriction, 0.0001)

                currentTireLongSlip = wheelStateNode.get_attribute("outputs:tireLongitudinalSlips").get()[wheelId]
                refTireLongSlip = wheelState[VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_SLIP]
                self.assertAlmostEqual(currentTireLongSlip, refTireLongSlip, 0.0001)

                currentTireLatSlip = wheelStateNode.get_attribute("outputs:tireLateralSlips").get()[wheelId]
                refTireLatSlip = wheelState[VEHICLE_WHEEL_STATE_TIRE_LATERAL_SLIP]
                self.assertAlmostEqual(currentTireLatSlip, refTireLatSlip, 0.0001)

                currentTireLongDir = wheelStateNode.get_attribute("outputs:tireLongitudinalDirections").get()[wheelId]
                refTireLongDir = wheelState[VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_DIRECTION]
                self.assertAlmostEqual(currentTireLongDir[0], refTireLongDir[0], 0.0001)
                self.assertAlmostEqual(currentTireLongDir[1], refTireLongDir[1], 0.0001)
                self.assertAlmostEqual(currentTireLongDir[2], refTireLongDir[2], 0.0001)

                currentTireLatDir = wheelStateNode.get_attribute("outputs:tireLateralDirections").get()[wheelId]
                refTireLatDir = wheelState[VEHICLE_WHEEL_STATE_TIRE_LATERAL_DIRECTION]
                self.assertAlmostEqual(currentTireLatDir[0], refTireLatDir[0], 0.0001)
                self.assertAlmostEqual(currentTireLatDir[1], refTireLatDir[1], 0.0001)
                self.assertAlmostEqual(currentTireLatDir[2], refTireLatDir[2], 0.0001)

                currentTirForce = wheelStateNode.get_attribute("outputs:tireForces").get()[wheelId]
                refTireForce = wheelState[VEHICLE_WHEEL_STATE_TIRE_FORCE]
                self.assertAlmostEqual(currentTirForce[0], refTireForce[0], 0.0001)
                self.assertAlmostEqual(currentTirForce[1], refTireForce[1], 0.0001)
                self.assertAlmostEqual(currentTirForce[2], refTireForce[2], 0.0001)

TEST_ENGINE_ATTR_MOI = 0
TEST_ENGINE_ATTR_PEAK_TORQUE = 1
TEST_ENGINE_ATTR_MAX_ROT_SPEED = 2
TEST_ENGINE_ATTR_IDLE_ROT_SPEED = 3

TEST_SUSPENSION_ATTR_SPRING_STRENGTH = 0
TEST_SUSPENSION_ATTR_SPRING_DAMPER_RATE = 1
TEST_SUSPENSION_ATTR_MAX_COMPRESSION = 2  # deprecated
TEST_SUSPENSION_ATTR_MAX_DROOP = 3  # deprecated
TEST_SUSPENSION_ATTR_TRAVEL_DISTANCE = 4
TEST_SUSPENSION_ATTR_SPRUNG_MASS = 5
TEST_SUSPENSION_ATTR_CAMBER_AT_REST = 6  # deprecated
TEST_SUSPENSION_ATTR_CAMBER_AT_MAX_COMPRESSION = 7  # deprecated
TEST_SUSPENSION_ATTR_CAMBER_AT_MAX_DROOP = 8  # deprecated

TEST_SUSPENSION_COMPLIANCE_ATTR_TOE_ANGLE = 0
TEST_SUSPENSION_COMPLIANCE_ATTR_CAMBER_ANGLE = 1
TEST_SUSPENSION_COMPLIANCE_ATTR_SUSP_FORCE_APP_POINT = 2
TEST_SUSPENSION_COMPLIANCE_ATTR_TIRE_FORCE_APP_POINT = 3

TEST_TIRE_ATTR_LAT_STIFF_X = 0  # deprecated
TEST_TIRE_ATTR_LAT_STIFF_Y = 1  # deprecated
TEST_TIRE_ATTR_LAT_STIFF_GRAPH_X = 2
TEST_TIRE_ATTR_LAT_STIFF_GRAPH_Y = 3
TEST_TIRE_ATTR_LONG_STIFF_PER_GRAV = 4  # deprecated
TEST_TIRE_ATTR_LONG_STIFF = 5
TEST_TIRE_ATTR_CAMBER_STIFF_PER_GRAV = 6  # deprecated
TEST_TIRE_ATTR_CAMBER_STIFF = 7
TEST_TIRE_ATTR_TIRE_FRICTION_TABLE = 8
TEST_TIRE_ATTR_REST_LOAD = 9

TEST_TIRE_MIN_SLIP_DENOM_LONG_PASSIVE = 0
TEST_TIRE_MIN_SLIP_DENOM_LONG_ACTIVE = 1
TEST_TIRE_MIN_SLIP_DENOM_LAT = 2

TEST_WHEEL_ATTR_RADIUS = 0
TEST_WHEEL_ATTR_WIDTH = 1
TEST_WHEEL_ATTR_MASS = 2
TEST_WHEEL_ATTR_MOI = 3
TEST_WHEEL_ATTR_DAMPING_RATE = 4
TEST_WHEEL_ATTR_TOE_ANGLE = 5  # deprecated

TEST_WHEEL_ATTACHMENT_ATTR_SUSP_TRAVEL_DIR = 0
TEST_WHEEL_ATTACHMENT_ATTR_SUSP_FRAME_POSITION = 1
TEST_WHEEL_ATTACHMENT_ATTR_SUSP_FORCE_APP_POINT_OFFSET = 2  # deprecated
TEST_WHEEL_ATTACHMENT_ATTR_WHEEL_CENTER_OFFSET = 3  # deprecated
TEST_WHEEL_ATTACHMENT_ATTR_TIRE_FORCE_APP_POINT_OFFSET = 4  # deprecated
TEST_WHEEL_ATTACHMENT_ATTR_COLLISON_GROUP = 5


class PhysXVehicleTestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase, PhysXVehicleTestBase):

    async def setUp(self):
        await super().setUp()
        self._stage_attached_vehicle = False

    async def tearDown(self):
        self._clean_up()
        await super().tearDown()

    async def new_stage(self):
        self._clean_up()
        await super().new_stage(attach_stage=False)
        self.setUpVehicleTestSettings()
        return self._stage

    def _prepare_for_simulation(self):
        global gPhysXVehicleInterface

        stageId = self.attach_stage()

        gPhysXVehicleInterface.attach_stage(stageId, True)
        self._stage_attached_vehicle = True

    def _clean_up(self):
        global gPhysXVehicleInterface

        if (self._stage_attached_vehicle):
            gPhysXVehicleInterface.detach_stage(True)
            self._stage_attached_vehicle = False

        if (self._stage_attached):
            self.detach_stage()

    def _simulate_one_frame(self, elapsedTime=None):
        if (elapsedTime is None):
            timeStep = self._get_time_step()
        else:
            timeStep = elapsedTime

        global gPhysXVehicleInterface
        gPhysXVehicleInterface.update_controllers(timeStep)

        self.step(1, timeStep)

    def _simulate(self, secondsToRun):
        timeStep = self._get_time_step()
        targetIterationCount = math.ceil(secondsToRun / timeStep)
        currentTime = 0.0

        global gPhysXVehicleInterface
        global gPhysXSimInterface
        for i in range(targetIterationCount):
            gPhysXVehicleInterface.update_controllers(timeStep)
            gPhysXSimInterface.simulate(timeStep, currentTime)
            gPhysXSimInterface.fetch_results()
            currentTime += timeStep

    #
    # make sure collision shapes for wheels are optional
    #
    async def test_wheel_collision_shape_option(self):
        stage = await self.new_stage()

        global gPhysXVehicleTestingInterface

        vehiclePaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            1,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_STANDARD,
            vehiclePathsOut=vehiclePaths,
        )
        self._simulate_one_frame_with_prep()
        simStats = self._get_sim_stats()
        self._check_sim_stats(simStats, 1, 1, 0, 0, 1, 0, 4, 0, 0, 1, 0)
        self.assertTrue(gPhysXVehicleTestingInterface.does_vehicle_exist(vehiclePaths[0]))

        stage = await self.new_stage()

        vehiclePaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            1,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_STANDARD,
            vehiclePathsOut=vehiclePaths,
        )
        self._simulate_one_frame_with_prep()
        simStats = self._get_sim_stats()
        self._check_sim_stats(simStats, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0)
        self.assertTrue(gPhysXVehicleTestingInterface.does_vehicle_exist(vehiclePaths[0]))

    #
    # verify that all the different prim setups for managing wheel transformations work as expected
    # and that the wheel collision shape is transformed as well under certain conditions
    #
    async def test_wheel_transform_management(self):

        expectedVehicleEndPos = None

        for i in range(5):

            #
            # 0: transform should be managed; no collision shape
            # 1: transform should be managed; collision shape at root
            # 2: transform should be managed; collision shape at root
            #    using a mesh with convex hull
            # 3: transform should be managed; collision shape as child
            # 4: transform should not be managed
            #

            if i == 0:
                createCollisionShapesForWheels_ = False
                useMeshAsWheelCollisionShape_ = False
                placeWheelCollisionShapesAtRoot_ = False
                noWheelRootTransformManagement_ = False
            elif i == 1:
                createCollisionShapesForWheels_ = True
                useMeshAsWheelCollisionShape_ = False
                placeWheelCollisionShapesAtRoot_ = True
                noWheelRootTransformManagement_ = False
            elif i == 2:
                createCollisionShapesForWheels_ = True
                useMeshAsWheelCollisionShape_ = True
                placeWheelCollisionShapesAtRoot_ = True
                noWheelRootTransformManagement_ = False
            elif i == 3:
                createCollisionShapesForWheels_ = True
                useMeshAsWheelCollisionShape_ = False
                placeWheelCollisionShapesAtRoot_ = False
                noWheelRootTransformManagement_ = False
            elif i == 4:
                createCollisionShapesForWheels_ = False
                useMeshAsWheelCollisionShape_ = False
                placeWheelCollisionShapesAtRoot_ = False
                noWheelRootTransformManagement_ = True

            stage = await self.new_stage()

            vehicleCount = 1
            vehiclePaths = []
            wheelAttachmentPaths = []
            Factory.create4WheeledCarsScenario(
                stage,
                1.0,
                vehicleCount,
                createCollisionShapesForWheels=createCollisionShapesForWheels_,
                driveMode=Factory.DRIVE_BASIC,
                vehiclePathsOut=vehiclePaths,
                wheelAttachmentPathsOut=wheelAttachmentPaths,
                placeWheelCollisionShapesAtRoot=placeWheelCollisionShapesAtRoot_,
                useMeshAsWheelCollisionShape=useMeshAsWheelCollisionShape_,
                noWheelRootTransformManagement=noWheelRootTransformManagement_,
            )

            wheelPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][0])
            xformable = None
            startQuat = None
            if i != 4:
                self.assertTrue(wheelPrim.IsA(UsdGeom.Xformable))

                xformable = UsdGeom.Xformable(wheelPrim)
                startGlobalPose = xformable.ComputeLocalToWorldTransform(0)
                startQuat = startGlobalPose.ExtractRotation()
            else:
                self.assertFalse(wheelPrim.IsA(UsdGeom.Xformable))

            prim = stage.GetPrimAtPath(vehiclePaths[0])
            vehicleController = PhysxSchema.PhysxVehicleControllerAPI(prim)
            vehicleController.GetSteerAttr().Set(1.0)
            vehicleStartPos = prim.GetAttribute("xformOp:translate").Get()

            secondsToRun = 1.0
            self._simulate_with_prep(secondsToRun)

            self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

            deltaAngleMin = 10
            deltaAngle = 0
            wheelDirStart = None
            refDirLocal = Gf.Vec3f(0, 0, 1)
            if i != 4:
                wheelDirCurrent = self._local_dir_to_world(refDirLocal, xformable)
                wheelDirStart = startQuat.TransformDir(refDirLocal)
                deltaAngle = self._get_delta_angle_degree(wheelDirStart, wheelDirCurrent)
                self.assertGreater(math.fabs(deltaAngle), deltaAngleMin)  # expect wheel to steer

            if i == 3:
                shapePrim = wheelPrim.GetChildren()[0]
                self.assertTrue(shapePrim.IsA(UsdGeom.Xformable))
                shapeXformable = UsdGeom.Xformable(shapePrim)
                shapeDirCurrent = self._local_dir_to_world(refDirLocal, shapeXformable)
                deltaAngleShape = self._get_delta_angle_degree(wheelDirStart, shapeDirCurrent)
                self.assertLess(math.fabs(deltaAngle - deltaAngleShape), 0.01)  # should be the same as for the wheel

            # now set the wheels straight again and accelerate
            vehicleController.GetSteerAttr().Set(0.0)
            self._simulate(secondsToRun)

            secondsToRun = 2.0
            vehicleController.GetAcceleratorAttr().Set(1.0)
            self._simulate(secondsToRun)

            self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

            if i == 0:
                expectedVehicleEndPos = prim.GetAttribute("xformOp:translate").Get()

            vehicleEndPos = prim.GetAttribute("xformOp:translate").Get()

            # the vehicle should have moved
            self.assertGreater(math.fabs(vehicleEndPos[2] - vehicleStartPos[2]), 1.0)

            # the simulation should not be affected by the configurations and the vehicle should always reach the
            # same position
            errTol = 0.001
            self.assertLess(math.fabs(expectedVehicleEndPos[0] - vehicleEndPos[0]), errTol)
            self.assertLess(math.fabs(expectedVehicleEndPos[1] - vehicleEndPos[1]), errTol)
            self.assertLess(math.fabs(expectedVehicleEndPos[2] - vehicleEndPos[2]), errTol)

    #
    # make sure wheel attachment prims do not have to be direct children of the vehicle prim
    #
    async def test_wheel_deeper_hierarchy(self):
        stage = await self.new_stage()

        vehiclePaths = []

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            1,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_STANDARD,
            vehiclePathsOut=vehiclePaths,
            placeWheelsAsGrandchildren=True,
        )

        self._simulate_one_frame_with_prep()

        global gPhysXVehicleTestingInterface
        self.assertTrue(gPhysXVehicleTestingInterface.does_vehicle_exist(vehiclePaths[0]))

    #
    # vehicles without a VehicleControllerAPI or WheelControllerAPI are valid and should be created without issues
    #
    async def test_vehicle_without_controller(self):
        stage = await self.new_stage()

        vehiclePaths = []

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            1,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_STANDARD,
            vehiclePathsOut=vehiclePaths,
            omitControllers=True,
        )

        self._simulate_one_frame_with_prep()

        global gPhysXVehicleTestingInterface
        self.assertTrue(gPhysXVehicleTestingInterface.does_vehicle_exist(vehiclePaths[0]))

    #
    # applying drive torque on the wheels should move the vehicle forward
    #
    async def test_acceleration(self):
        stage = await self.new_stage()

        vehicleCount = 3
        vehiclePaths = []
        wheelAttachmentPaths = []
        basePosForwardDir = 0
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_NONE,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
            vehicleDelta=[-3, 0, 0],
            basePositionForwardDir=basePosForwardDir,
        )

        wheelDriveTorques = [0.0, 300, 600]

        for i in range(vehicleCount):
            if i > 0:  # to verify that by default no drive torque is applied
                prim = stage.GetPrimAtPath(wheelAttachmentPaths[i][Factory.WHEEL_FRONT_LEFT])
                wheelController = PhysxSchema.PhysxVehicleWheelControllerAPI(prim)
                wheelController.GetDriveTorqueAttr().Set(wheelDriveTorques[i])

                prim = stage.GetPrimAtPath(wheelAttachmentPaths[i][Factory.WHEEL_FRONT_RIGHT])
                wheelController = PhysxSchema.PhysxVehicleWheelControllerAPI(prim)
                wheelController.GetDriveTorqueAttr().Set(wheelDriveTorques[i])

        secondsToRun = 2.0
        self._simulate_with_prep(secondsToRun)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        prim = stage.GetPrimAtPath(vehiclePaths[0])
        forwardPos = prim.GetAttribute("xformOp:translate").Get()[2]
        diff = math.fabs(forwardPos - basePosForwardDir)
        self.assertLess(diff, 0.01)

        # note: the following values are chosen by experiment. If the vehicle factory code changes, adjustments
        #       might be necessary

        prim = stage.GetPrimAtPath(vehiclePaths[1])
        forwardPos = prim.GetAttribute("xformOp:translate").Get()[2]
        diff = forwardPos - basePosForwardDir
        self.assertGreater(diff, 1.0)

        prim = stage.GetPrimAtPath(vehiclePaths[2])
        forwardPos = prim.GetAttribute("xformOp:translate").Get()[2]
        diff = forwardPos - basePosForwardDir
        self.assertGreater(diff, 3.0)

    #
    # read/write of wheel rotation speed and expecting the vehicle to move a certain distance
    #
    async def test_wheel_rotation_speed(self):
        stage = await self.new_stage()

        startSpeed = 10

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []
        basePosForwardDir = 0
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_NONE,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
            vehicleDelta=[-3, 0, 0],
            basePositionForwardDir=basePosForwardDir,
        )

        global gPhysXInterface

        self._prepare_for_simulation()

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])
        rigidBodyAPI = UsdPhysics.RigidBodyAPI(vehiclePrim)
        rigidBodyAPI.GetVelocityAttr().Set(Gf.Vec3f(0, 0, startSpeed))

        for j in range(4):
            wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][j])
            (wheelRadius, wheelWidth) = self._get_wheel_radius_and_width(wheelAttPrim, stage)
            rotationSpeed = startSpeed / wheelRadius  # assuming free rolling

            wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[0][j])
            currentRotSpeed = wheelState[VEHICLE_WHEEL_STATE_ROTATION_SPEED]
            self.assertEqual(currentRotSpeed, 0.0)

            gPhysXInterface.set_wheel_rotation_speed(wheelAttachmentPaths[0][j], rotationSpeed)

            wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[0][j])
            currentRotSpeed = wheelState[VEHICLE_WHEEL_STATE_ROTATION_SPEED]
            self.assertLess(math.fabs(currentRotSpeed - rotationSpeed), 0.001)

        secondsToRun = 2.0
        self._simulate(secondsToRun)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        forwardPos = vehiclePrim.GetAttribute("xformOp:translate").Get()[2]
        diff = forwardPos - basePosForwardDir
        self.assertLess(diff, (secondsToRun * startSpeed))  # there is damping and friction
        self.assertGreater(
            diff, (secondsToRun * startSpeed * 0.9)
        )  # should still be close to scenario with no friction or damping

    #
    # read/write of wheel rotation angle. Change angle and expect wheel geometry to adjust after one sim step.
    #
    async def test_wheel_rotation_angle(self):
        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []
        basePosForwardDir = 0
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
            vehicleDelta=[-3, 0, 0],
            basePositionForwardDir=basePosForwardDir,
        )

        global gPhysXInterface

        self._prepare_for_simulation()

        refDirStart = []
        addRotAngle = (30 * math.pi) / 180
        refDirLocal = Gf.Vec3f(0, 1, 0)
        for j in range(4):
            wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][j])
            self.assertTrue(wheelAttPrim.IsA(UsdGeom.Xformable))
            xformable = UsdGeom.Xformable(wheelAttPrim)
            refDir = self._local_dir_to_world(refDirLocal, xformable)
            refDirStart.append(refDir)

            wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[0][j])
            rotAngle = wheelState[VEHICLE_WHEEL_STATE_ROTATION_ANGLE]

            newRotAngle = rotAngle + addRotAngle
            gPhysXInterface.set_wheel_rotation_angle(wheelAttachmentPaths[0][j], newRotAngle)

            wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[0][j])
            rotAngle = wheelState[VEHICLE_WHEEL_STATE_ROTATION_ANGLE]
            self.assertLess(math.fabs(rotAngle - newRotAngle), 0.001)

        self._simulate_one_frame()

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        for j in range(4):
            wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][j])
            self.assertTrue(wheelAttPrim.IsA(UsdGeom.Xformable))
            xformable = UsdGeom.Xformable(wheelAttPrim)
            refDir = self._local_dir_to_world(refDirLocal, xformable)

            deltaAngle = self._get_delta_angle_radians(refDirStart[j], refDir)
            self.assertLess((math.fabs(deltaAngle) - addRotAngle), 0.001)
            # the wheel should have rotated by the expected amount

    #
    # Ackermann steering helper method. Provided angle should ensure center of turning circle matches for wheels.
    #
    async def test_ackermann_steering_helper(self):
        global gPhysXVehicleInterface

        axleSeparation = 4.0
        axleWidth = 1.8

        steerAngle = (30 * math.pi) / 180

        outerWheelSteerAngle = gPhysXVehicleInterface.compute_ackermann_steering_angle(
            steerAngle, axleSeparation, axleWidth
        )

        distInner = axleSeparation / math.tan(steerAngle)
        distOuter = axleSeparation / math.tan(outerWheelSteerAngle)

        err = math.fabs((distOuter - distInner) - axleWidth)
        self.assertLess(err, 0.01)

        # --

        steerAngle = (-15 * math.pi) / 180

        outerWheelSteerAngle = gPhysXVehicleInterface.compute_ackermann_steering_angle(
            steerAngle, axleSeparation, axleWidth
        )

        distInner = axleSeparation / math.tan(steerAngle)
        distOuter = axleSeparation / math.tan(outerWheelSteerAngle)

        err = math.fabs((distOuter - distInner) + axleWidth)
        self.assertLess(err, 0.01)

    #
    # default values of length scale related attributes need to take length scale into account.
    # The test creates vehicles without specifying attributes with default values, accelerates for
    # some interval and compares the results for 2 different scales.
    #
    async def test_default_values_and_length_scale(self):
        for j in range(2):
            if j == 0:
                drvMode = Factory.DRIVE_BASIC
            else:
                drvMode = Factory.DRIVE_STANDARD

            pos = []

            for k in range(2):
                if k == 0:
                    massScale = 1.0
                else:
                    massScale = 100.0

                for i in range(2):
                    stage = await self.new_stage()

                    if i == 0:
                        lengthScale = 1.0
                    else:
                        lengthScale = 100.0

                    vehicleCount = 1
                    vehiclePaths = []
                    basePosForwardDir = 0
                    Factory.create4WheeledCarsScenario(
                        stage,
                        lengthScale,
                        vehicleCount,
                        createCollisionShapesForWheels=False,
                        driveMode=drvMode,
                        vehiclePathsOut=vehiclePaths,
                        basePositionForwardDir=basePosForwardDir,
                        skipAttrWithDefaultValues=True,
                        massScale=massScale
                    )

                    prim = stage.GetPrimAtPath(vehiclePaths[0])
                    vehicleController = PhysxSchema.PhysxVehicleControllerAPI(prim)
                    vehicleController.GetAcceleratorAttr().Set(1.0)
                    vehicleController.GetTargetGearAttr().Set(1)

                    secondsToRun = 2.0
                    self._simulate_with_prep(secondsToRun)

                    self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

                    pos.append(prim.GetAttribute("xformOp:translate").Get() / lengthScale)

            self.assertGreater((pos[0][2] - basePosForwardDir), 1.0)  # make sure vehicle moved at all

            errTol = 0.01
            if (drvMode == Factory.DRIVE_STANDARD):
                errTolLong = 0.13
            else:
                errTolLong = 0.01

            for n in range(1, len(pos)):
                delta = pos[0] - pos[n]

                self.assertLess(math.fabs(delta[0]), errTol)
                self.assertLess(math.fabs(delta[1]), errTol)
                self.assertLess(math.fabs(delta[2]), errTolLong)

    #
    # teleporting a moving vehicle and setting to rest state should keep the vehicle in place.
    #
    async def test_set_to_rest_state(self):
        global gPhysXInterface

        for i in range(2):
            stage = await self.new_stage()

            if i == 0:
                drvMode = Factory.DRIVE_BASIC
            else:
                drvMode = Factory.DRIVE_STANDARD

            vehicleCount = 1
            vehiclePaths = []
            wheelAttachmentPaths = []
            basePosForwardDir = 0
            Factory.create4WheeledCarsScenario(
                stage,
                1.0,
                vehicleCount,
                createCollisionShapesForWheels=True,
                driveMode=drvMode,
                vehiclePathsOut=vehiclePaths,
                wheelAttachmentPathsOut=wheelAttachmentPaths,
                vehicleDelta=[-3, 0, 0],
                basePositionForwardDir=basePosForwardDir,
            )

            prim = stage.GetPrimAtPath(vehiclePaths[0])
            vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI(prim)
            vehicleControllerAPI.GetAcceleratorAttr().Set(1.0)
            if i == 1:
                vehicleControllerAPI.GetTargetGearAttr().Set(1)

            posAttr = prim.GetAttribute("xformOp:translate")
            startPos = posAttr.Get()
            orientAttr = prim.GetAttribute("xformOp:orient")
            startOrient = orientAttr.Get()

            self._prepare_for_simulation()

            secondsToRun = 1.0
            self._simulate(secondsToRun)

            self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

            pos = posAttr.Get()
            self.assertGreater((pos[2] - startPos[2]), 0.3)  # make sure vehicle moved

            posAttr.Set(startPos)
            orientAttr.Set(startOrient)
            vehicleControllerAPI.GetAcceleratorAttr().Set(0.0)

            gPhysXInterface.set_vehicle_to_rest_state(vehiclePaths[0])

            self._simulate(secondsToRun)

            self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=True)

            endPos = posAttr.Get()
            self.assertLess(math.fabs(endPos[2] - startPos[2]), 0.005)

            rigidBodyAPI = UsdPhysics.RigidBodyAPI(prim)
            vel = rigidBodyAPI.GetVelocityAttr().Get()
            self.assertLess(vel.GetLength(), 0.007)

    #
    # Test that the drive data telemetry is functioning.
    #
    async def test_drive_data(self):

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

        secondsToRun = 2.0
        self._simulate_with_prep(secondsToRun)

        # To see where the vehicle ends up, uncomment this line.
        # self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        global gPhysXInterface
        driveData = gPhysXInterface.get_vehicle_drive_state(vehiclePaths[0])

        accelerator = 1.0
        brake0 = 0.0
        brake1 = 0.0
        steer = 1.0
        clutch = 0.0
        autoboxTimeSinceLastShift = 1.466
        currentGear = 2
        engineRotationSpeed = 600.0
        targetGear = 2
        automaticTransmission = True

        errTol = 0.1
        self.assertLess(math.fabs(accelerator - driveData[VEHICLE_DRIVE_STATE_ACCELERATOR]), errTol)
        self.assertLess(math.fabs(brake0 - driveData[VEHICLE_DRIVE_STATE_BRAKE0]), errTol)
        self.assertLess(math.fabs(brake1 - driveData[VEHICLE_DRIVE_STATE_BRAKE1]), errTol)
        self.assertLess(math.fabs(steer - driveData[VEHICLE_DRIVE_STATE_STEER]), errTol)
        self.assertLess(math.fabs(clutch - driveData[VEHICLE_DRIVE_STATE_CLUTCH]), errTol)
        self.assertLess(math.fabs(autoboxTimeSinceLastShift - driveData[VEHICLE_DRIVE_STATE_AUTOBOX_TIME_SINCE_LAST_SHIFT]), errTol)
        self.assertLess(math.fabs(engineRotationSpeed - driveData[VEHICLE_DRIVE_STATE_ENGINE_ROTATION_SPEED]), errTol)
        self.assertLess(driveData[VEHICLE_DRIVE_STATE_GEAR_SWITCH_TIME], 0.0)
        self.assertTrue(automaticTransmission == driveData[VEHICLE_DRIVE_STATE_AUTOMATIC_TRANSMISSION])
        self.assertTrue(currentGear == driveData[VEHICLE_DRIVE_STATE_CURRENT_GEAR])
        self.assertTrue(targetGear == driveData[VEHICLE_DRIVE_STATE_TARGET_GEAR])

        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
        )

        prim = stage.GetPrimAtPath(vehiclePaths[0])
        vehicleController = PhysxSchema.PhysxVehicleControllerAPI(prim)
        vehicleController.GetSteerAttr().Set(1.0)
        vehicleController.GetAcceleratorAttr().Set(1.0)

        secondsToRun = 1.0
        self._simulate_with_prep(secondsToRun)

        # To see where the vehicle ends up, uncomment this line.
        # self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        driveData = gPhysXInterface.get_vehicle_drive_state(vehiclePaths[0])

        accelerator = 1.0
        brake0 = 0.0
        brake1 = 0.0
        steer = 1.0
        clutch = 0.0
        autoboxTimeSinceLastShift = 0.0
        currentGear = 0
        engineRotationSpeed = 0.0
        gearSwitchTime = 0.0
        targetGear = 0
        automaticTransmission = False

        errTol = 0.1
        self.assertLess(math.fabs(accelerator - driveData[VEHICLE_DRIVE_STATE_ACCELERATOR]), errTol)
        self.assertLess(math.fabs(brake0 - driveData[VEHICLE_DRIVE_STATE_BRAKE0]), errTol)
        self.assertLess(math.fabs(brake1 - driveData[VEHICLE_DRIVE_STATE_BRAKE1]), errTol)
        self.assertLess(math.fabs(steer - driveData[VEHICLE_DRIVE_STATE_STEER]), errTol)
        self.assertLess(math.fabs(clutch - driveData[VEHICLE_DRIVE_STATE_CLUTCH]), errTol)
        self.assertLess(math.fabs(autoboxTimeSinceLastShift - driveData[VEHICLE_DRIVE_STATE_AUTOBOX_TIME_SINCE_LAST_SHIFT]), errTol)
        self.assertLess(math.fabs(engineRotationSpeed - driveData[VEHICLE_DRIVE_STATE_ENGINE_ROTATION_SPEED]), errTol)
        self.assertLess(math.fabs(gearSwitchTime - driveData[VEHICLE_DRIVE_STATE_GEAR_SWITCH_TIME]), errTol)
        self.assertTrue(automaticTransmission == driveData[VEHICLE_DRIVE_STATE_AUTOMATIC_TRANSMISSION])
        self.assertTrue(currentGear == driveData[VEHICLE_DRIVE_STATE_CURRENT_GEAR])
        self.assertTrue(targetGear == driveData[VEHICLE_DRIVE_STATE_TARGET_GEAR])

    #
    # Test that the vehicle can enter reverse and drive in reverse
    # without auto-reverse.
    #
    async def _reverse(self, driveModeIn):

        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=driveModeIn,
            vehiclePathsOut=vehiclePaths,
        )

        prim = stage.GetPrimAtPath(vehiclePaths[0])
        vehicleController = PhysxSchema.PhysxVehicleControllerAPI(prim)

        startPos = prim.GetAttribute("xformOp:translate").Get()

        self._prepare_for_simulation()

        global gPhysXVehicleInterface

        # Auto reverse defaults to True, so turn it off initially.
        gPhysXVehicleInterface.set_auto_reverse_enabled(vehiclePaths[0], False)
        autoReverseEnabled = gPhysXVehicleInterface.get_auto_reverse_enabled(vehiclePaths[0])
        self.assertFalse(autoReverseEnabled)

        # Put the vehicle into reverse manually.
        vehicleController.GetTargetGearAttr().Set(-1)
        vehicleController.GetAcceleratorAttr().Set(1.0)

        secondsToRun = 1.0
        self._simulate(secondsToRun)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        # Check vehicle position
        endPos = prim.GetAttribute("xformOp:translate").Get()
        delta = endPos - startPos

        errTol = 0.01
        self.assertLess(math.fabs(delta[0]), errTol)
        self.assertLess(math.fabs(delta[1]), errTol)
        self.assertLess(delta[2], -0.3)  # vehicle should have driven backwards

    #
    # Test that the vehicle can enter reverse and drive in reverse
    # without auto-reverse.
    #
    async def test_reverse_standard(self):
        await self._reverse(Factory.DRIVE_STANDARD)

    #
    # Test that the vehicle can enter reverse and drive in reverse
    # without auto-reverse.
    #
    async def test_reverse_basic(self):
        await self._reverse(Factory.DRIVE_BASIC)

    #
    # testing different combinations for the vehicle up- and forward-axis. The vehicle should move in
    # the right direction and the wheel collision shape should not be rotated in a weird way.
    #
    async def test_up_and_forward_axis_combinations(self):

        # Use convex hull approximation as vehicle's hacks don't
        # work with new convex core cylinders (OMPE-26698)
        settings = carb.settings.get_settings()
        settings.set_bool(SETTING_COLLISION_APPROXIMATE_CYLINDERS, True)

        for i in range(6):

            #
            # 0: y = up, z = forward
            # 1: y = up, x = forward
            # 2: z = up, x = forward
            # 3: z = up, y = forward
            # 4: x = up, y = forward
            # 5: x = up, z = forward
            #

            if i == 0:
                axes_ = Factory.AxesIndices(1, 2, 0)
            elif i == 1:
                axes_ = Factory.AxesIndices(1, 0, 2)
            elif i == 2:
                axes_ = Factory.AxesIndices(2, 0, 1)
            elif i == 3:
                axes_ = Factory.AxesIndices(2, 1, 0)
            elif i == 4:
                axes_ = Factory.AxesIndices(0, 1, 2)
            elif i == 5:
                axes_ = Factory.AxesIndices(0, 2, 1)

            stage = await self.new_stage()

            vehicleCount = 1
            vehiclePaths = []
            wheelAttachmentPaths = []
            Factory.create4WheeledCarsScenario(
                stage,
                1.0,
                vehicleCount,
                createCollisionShapesForWheels=True,
                driveMode=Factory.DRIVE_BASIC,
                vehiclePathsOut=vehiclePaths,
                wheelAttachmentPathsOut=wheelAttachmentPaths,
                addChassisCollisionBox=False,
                axes=axes_
            )

            prim = stage.GetPrimAtPath(vehiclePaths[0])
            vehicleController = PhysxSchema.PhysxVehicleControllerAPI(prim)
            vehicleController.GetAcceleratorAttr().Set(1.0)

            vehicleStartPos = prim.GetAttribute("xformOp:translate").Get()

            self._prepare_for_simulation()

            self._simulate(1 / 60)
            self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

            wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][0])
            xformable = UsdGeom.Xformable(wheelAttPrim)
            wheelPose = xformable.ComputeLocalToWorldTransform(0)
            wheelPos = wheelPose.ExtractTranslation()
            (wheelRadius, wheelWidth) = self._get_wheel_radius_and_width(wheelAttPrim, stage)
            rayUpOffset = 1.5 * wheelRadius
            rayStartPos = carb.Float3(wheelPos[0], wheelPos[1], wheelPos[2])
            rayStartPos[axes_.up] = rayStartPos[axes_.up] + rayUpOffset
            rayDir = carb.Float3(0, 0, 0)
            rayDir[axes_.up] = -1.0
            rayLength = 2 * wheelRadius

            global gPhysXSceneQueryInterface

            # this one is just to make sure the raycast does work as expected and the wheel is hit
            hitInfo = gPhysXSceneQueryInterface.raycast_closest(rayStartPos, rayDir, rayLength)
            self.assertTrue(hitInfo["hit"])
            self.assertTrue(hitInfo["collision"] == wheelAttPrim.GetChildren()[0].GetPath().pathString)
            self.assertLess(math.fabs(hitInfo["distance"] - (rayUpOffset - wheelRadius)), 0.01)

            # now making sure the PhysX collision shape is not rotated 90 degrees or similar (was the case at some point due
            # to the way cylinders were implemented)
            if (rayStartPos[axes_.side] > 0):
                rayStartPos[axes_.side] = rayStartPos[axes_.side] + wheelWidth
            else:
                rayStartPos[axes_.side] = rayStartPos[axes_.side] - wheelWidth
            hitInfo = gPhysXSceneQueryInterface.raycast_closest(rayStartPos, rayDir, rayLength)
            self.assertFalse(hitInfo["hit"])

            secondsToRun = 2.0
            self._simulate(secondsToRun)

            self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

            vehicleEndPos = prim.GetAttribute("xformOp:translate").Get()

            # the vehicle should have moved in the forward direction
            deltaForward = vehicleEndPos[axes_.forward] - vehicleStartPos[axes_.forward]
            self.assertGreater(deltaForward, 1.0)

            # the vehicle should only move minimally in other directions
            errTol = 0.001
            self.assertLess(math.fabs(vehicleEndPos[axes_.up] - vehicleStartPos[axes_.up]), errTol)
            self.assertLess(math.fabs(vehicleEndPos[axes_.side] - vehicleStartPos[axes_.side]), errTol)

    #
    # modifying a friction value in the tire friction table should effect the simulation accordingly.
    #
    async def test_tire_friction_table_friction_change(self):

        for i in range(2):

            #
            # 0: factory-defined friction value
            # 1: setting friction value to 0 and then back to high value
            #

            stage = await self.new_stage()

            vehicleCount = 1
            vehiclePaths = []
            wheelAttachmentPaths = []
            Factory.create4WheeledCarsScenario(
                stage,
                1.0,
                vehicleCount,
                createCollisionShapesForWheels=True,
                driveMode=Factory.DRIVE_BASIC,
                vehiclePathsOut=vehiclePaths,
                wheelAttachmentPathsOut=wheelAttachmentPaths
            )

            prim = stage.GetPrimAtPath(vehiclePaths[0])
            vehicleController = PhysxSchema.PhysxVehicleControllerAPI(prim)
            vehicleController.GetAcceleratorAttr().Set(1.0)

            vehicleStartPos = prim.GetAttribute("xformOp:translate").Get()

            frictionTable = None
            if (i == 1):
                # set friction to 0

                wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][0])
                frictionTable = self._get_tire_friction_table(wheelAttPrim, stage)

                frictionValues = frictionTable.GetFrictionValuesAttr().Get()
                frictionValues[Factory.MATERIAL_TARMAC] = 0
                frictionTable.GetFrictionValuesAttr().Set(frictionValues)

            secondsToRun = 2.0
            self._simulate_with_prep(secondsToRun)

            self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

            vehicleEndPos = prim.GetAttribute("xformOp:translate").Get()

            errTol = 0.001

            # the vehicle should not move in the forward direction if friction is 0
            deltaForward = vehicleEndPos[2] - vehicleStartPos[2]
            if (i == 0):
                self.assertGreater(deltaForward, 1.0)
            else:
                self.assertLess(math.fabs(deltaForward), errTol)

            if (i == 1):
                # increase friction again while playing
                frictionValues = frictionTable.GetFrictionValuesAttr().Get()
                frictionValues[Factory.MATERIAL_TARMAC] = 1
                frictionTable.GetFrictionValuesAttr().Set(frictionValues)

                self._simulate(secondsToRun)

                self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

                vehicleEndPos = prim.GetAttribute("xformOp:translate").Get()
                deltaForward = vehicleEndPos[2] - vehicleStartPos[2]
                self.assertGreater(deltaForward, 1.0)

            # the vehicle should only move minimally in other directions
            self.assertLess(math.fabs(vehicleEndPos[0] - vehicleStartPos[0]), errTol)
            self.assertLess(math.fabs(vehicleEndPos[1] - vehicleStartPos[1]), errTol)

    #
    # modifying the default friction value in the tire friction table should effect the simulation accordingly.
    #
    async def test_tire_friction_table_default_friction_change(self):

        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths
        )

        prim = stage.GetPrimAtPath(vehiclePaths[0])
        vehicleController = PhysxSchema.PhysxVehicleControllerAPI(prim)
        vehicleController.GetAcceleratorAttr().Set(1.0)

        vehicleStartPos = prim.GetAttribute("xformOp:translate").Get()

        wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][0])
        frictionTable = self._get_tire_friction_table(wheelAttPrim, stage)
        frictionTable.GetDefaultFrictionValueAttr().Set(0)

        rootPath = str(stage.GetDefaultPrim().GetPath())
        groundPlanePath = rootPath + FACTORY_COLLISION_PLANE_SUBPATH
        groundPlanePrim = stage.GetPrimAtPath(groundPlanePath)

        # remove physics material from plane to make sure the default friction value is used
        bindingAPI = UsdShade.MaterialBindingAPI(groundPlanePrim)
        bindingAPI.UnbindAllBindings()

        secondsToRun = 2.0
        self._simulate_with_prep(secondsToRun)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        vehicleEndPos = prim.GetAttribute("xformOp:translate").Get()

        errTol = 0.001

        # the vehicle should not move in the forward direction if friction is 0
        deltaForward = vehicleEndPos[2] - vehicleStartPos[2]
        self.assertLess(math.fabs(deltaForward), errTol)

        # increase friction again while playing
        frictionTable.GetDefaultFrictionValueAttr().Set(1)

        self._simulate(secondsToRun)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        vehicleEndPos = prim.GetAttribute("xformOp:translate").Get()
        deltaForward = vehicleEndPos[2] - vehicleStartPos[2]
        self.assertGreater(deltaForward, 1.0)

        # the vehicle should only move minimally in other directions
        self.assertLess(math.fabs(vehicleEndPos[0] - vehicleStartPos[0]), errTol)
        self.assertLess(math.fabs(vehicleEndPos[1] - vehicleStartPos[1]), errTol)

    #
    # adding a tire friction table after the simulation has started.
    #
    async def test_tire_friction_table_add(self):

        stage = await self.new_stage()

        vehicleCount = 2
        vehiclePaths = []
        wheelAttachmentPaths = []
        tireFrictionTablePaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
            tireFrictionTablePathsOut=tireFrictionTablePaths,
            useShareableComponentsList = [False, False]
        )

        tireFrictionTablePrim = stage.GetPrimAtPath(tireFrictionTablePaths[Factory.TIRE_FRICTION_TABLE_WINTER_TIRE])
        tireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable(tireFrictionTablePrim)
        tarmacMaterialPath = tireFrictionTable.GetGroundMaterialsRel().GetTargets()[Factory.MATERIAL_TARMAC]

        rootPath = str(stage.GetDefaultPrim().GetPath())
        customMaterialPath = rootPath + "/CustomMaterial"
        material = UsdShade.Material.Define(stage, customMaterialPath)
        materialAPI = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        materialAPI.CreateStaticFrictionAttr(0.9)
        materialAPI.CreateDynamicFrictionAttr(0.7)
        materialAPI.CreateRestitutionAttr(0.0)
        PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())

        prim0 = stage.GetPrimAtPath(vehiclePaths[0])
        vehicleStartPos0 = prim0.GetAttribute("xformOp:translate").Get()
        vehicleController0 = PhysxSchema.PhysxVehicleControllerAPI(prim0)
        vehicleController0.GetAcceleratorAttr().Set(1.0)

        prim1 = stage.GetPrimAtPath(vehiclePaths[1])
        vehicleStartPos1 = prim1.GetAttribute("xformOp:translate").Get()
        vehicleController1 = PhysxSchema.PhysxVehicleControllerAPI(prim1)
        vehicleController1.GetAcceleratorAttr().Set(1.0)

        self._prepare_for_simulation()

        # create another tire friction table after sim has started
        customTireFrictionTablePath = rootPath + "/CustomTireFrictionTable"
        customTireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable.Define(stage, customTireFrictionTablePath)
        customTireFrictionTableGroundMaterialsRel = customTireFrictionTable.CreateGroundMaterialsRel()
        customTireFrictionTableGroundMaterialsRel.AddTarget(customMaterialPath)
        customTireFrictionTableGroundMaterialsRel.AddTarget(tarmacMaterialPath)
        customTireFrictionTable.CreateFrictionValuesAttr([0.75, 0.0])

        # flush the USD changes. Without that, the relationship changes further below would get processed first
        # and fail because the table does not exist yet.
        global gPhysXInterface
        gPhysXInterface.force_load_physics_from_usd()

        # use newly created tire friction table for one of the vehicles
        for wheelAttPath in wheelAttachmentPaths[1]:
            wheelAttPrim = stage.GetPrimAtPath(wheelAttPath)
            tireAPI = PhysxSchema.PhysxVehicleTireAPI(wheelAttPrim)

            targets = tireAPI.GetFrictionTableRel().GetTargets()
            targets[0] = customTireFrictionTablePath
            tireAPI.GetFrictionTableRel().SetTargets(targets)

        secondsToRun = 1.0
        self._simulate(secondsToRun)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        vehicleEndPos0 = prim0.GetAttribute("xformOp:translate").Get()
        delta0 = vehicleEndPos0 - vehicleStartPos0

        vehicleEndPos1 = prim1.GetAttribute("xformOp:translate").Get()
        delta1 = vehicleEndPos1 - vehicleStartPos1

        errTol = 0.001

        self.assertGreater(delta0[2], 0.3)
        self.assertLess(delta1[2], errTol)  # the new table has friction value of 0

        # the vehicles should only move minimally in other directions
        self.assertLess(math.fabs(delta0[0]), errTol)
        self.assertLess(math.fabs(delta0[1]), errTol)
        self.assertLess(math.fabs(delta1[0]), errTol)
        self.assertLess(math.fabs(delta1[1]), errTol)

    #
    # vehicles with no tire friction table defined should use whatever table is
    # available or default values.
    #
    async def test_no_tire_friction_table(self):

        for i in range(2):
            stage = await self.new_stage()

            vehicleCount = 1
            vehiclePaths = []
            wheelAttachmentPaths = []
            tireFrictionTablePaths = []
            Factory.create4WheeledCarsScenario(
                stage,
                1.0,
                vehicleCount,
                createCollisionShapesForWheels=False,
                driveMode=Factory.DRIVE_BASIC,
                vehiclePathsOut=vehiclePaths,
                wheelAttachmentPathsOut=wheelAttachmentPaths,
                tireFrictionTablePathsOut=tireFrictionTablePaths,
                useShareableComponentsList = [False]
            )

            for path in tireFrictionTablePaths:
                stage.RemovePrim(path)

            for wheelAttPath in wheelAttachmentPaths[0]:
                wheelAttPrim = stage.GetPrimAtPath(wheelAttPath)
                tireAPI = PhysxSchema.PhysxVehicleTireAPI(wheelAttPrim)

                if (i == 0):
                    tireAPI.GetFrictionTableRel().ClearTargets(True)
                else:
                    # at some point an authored relationship with 0 entries was wrongly
                    # treated as a setup error
                    tireAPI.GetFrictionTableRel().SetTargets([])

            prim = stage.GetPrimAtPath(vehiclePaths[0])
            startPos = prim.GetAttribute("xformOp:translate").Get()
            vehicleController = PhysxSchema.PhysxVehicleControllerAPI(prim)
            vehicleController.GetAcceleratorAttr().Set(1.0)

            secondsToRun = 1.0
            self._simulate_with_prep(secondsToRun)

            self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

            endPos = prim.GetAttribute("xformOp:translate").Get()

            errTol = 0.001
            delta = endPos - startPos

            self.assertGreater(delta[2], 0.3)

            # the vehicles should only move minimally in other directions
            self.assertLess(math.fabs(delta[0]), errTol)
            self.assertLess(math.fabs(delta[1]), errTol)

    #
    # switching from disabled to enabled and vice versa should affect the simulation accordingly.
    #
    async def test_vehicle_enabled_basic(self):

        for i in range(4):
            stage = await self.new_stage()

            if i == 0:
                vehicleEnabled = [False]
                vehicleTestIndex = 0
            elif i == 1:
                vehicleEnabled = [True]
                vehicleTestIndex = 0
            if i == 2:
                vehicleEnabled = [False, True, False, True]  # testing a larger set to cover other internal list swapping codepaths
                vehicleTestIndex = 2
            elif i == 3:
                vehicleEnabled = [True, False, True, False]
                vehicleTestIndex = 0

            vehicleCount = len(vehicleEnabled)
            vehiclePaths = []
            Factory.create4WheeledCarsScenario(
                stage,
                1.0,
                vehicleCount,
                createCollisionShapesForWheels=True,
                driveMode=Factory.DRIVE_STANDARD,
                vehiclePathsOut=vehiclePaths,
                vehicleEnabledList=vehicleEnabled
            )

            prim = []
            for j in range(vehicleCount):
                p = stage.GetPrimAtPath(vehiclePaths[j])
                prim.append(p)
                vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI(p)
                vehicleControllerAPI.GetAcceleratorAttr().Set(1.0)
                vehicleControllerAPI.GetTargetGearAttr().Set(1)
            vehicleAPI = PhysxSchema.PhysxVehicleAPI(prim[vehicleTestIndex])

            startPos = []
            for j in range(vehicleCount):
                startPos.append(prim[j].GetAttribute("xformOp:translate").Get())

            secondsToRun = 1.0
            self._simulate_with_prep(secondsToRun)

            self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=True)

            pos = []
            deltaPos = []
            for j in range(vehicleCount):
                pos.append(prim[j].GetAttribute("xformOp:translate").Get())
                d = pos[j] - startPos[j]
                deltaPos.append(d)

            didMoveThreshold = 0.3
            didNotMoveThreshold = 0.005

            if (i == 0) or (i == 2):
                self.assertLess(math.fabs(deltaPos[vehicleTestIndex][2]), didNotMoveThreshold)  # make sure the vehicle did not move
                vehicleAPI.GetVehicleEnabledAttr().Set(True)

                # since gravity is disabled on vehicle rigid bodies, the position should not have changed
                # even though the body gets simulated. Thus, nothing needs to be reset here.
            else:
                self.assertGreater(deltaPos[vehicleTestIndex][2], didMoveThreshold)  # make sure vehicle moved
                vehicleAPI.GetVehicleEnabledAttr().Set(False)

                # disabling the vehicle does not stop the rigid body simulation
                rigidBodyAPI = UsdPhysics.RigidBodyAPI(prim[vehicleTestIndex])
                rigidBodyAPI.GetVelocityAttr().Set(Gf.Vec3f(0.0))
                rigidBodyAPI.GetAngularVelocityAttr().Set(Gf.Vec3f(0.0))

                # enable gravity again
                pxRigidBodyAPI = PhysxSchema.PhysxRigidBodyAPI(prim[vehicleTestIndex])
                pxRigidBodyAPI.GetDisableGravityAttr().Set(False)

            for j in range(vehicleCount):
                if (j != vehicleTestIndex):
                    if (vehicleEnabled[j]):
                        self.assertGreater(deltaPos[j][2], didMoveThreshold)
                    else:
                        self.assertLess(math.fabs(deltaPos[j][2]), didNotMoveThreshold)

            self._simulate(secondsToRun)

            self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=True)

            endPos = []
            deltaPos = []
            for j in range(vehicleCount):
                endPos.append(prim[j].GetAttribute("xformOp:translate").Get())
                d = endPos[j] - pos[j]
                deltaPos.append(d)

            if (i == 0) or (i == 2):
                self.assertGreater(deltaPos[vehicleTestIndex][2], didMoveThreshold)  # make sure vehicle moved
            else:
                self.assertLess(math.fabs(deltaPos[vehicleTestIndex][2]), didNotMoveThreshold)  # make sure the vehicle did not move
                self.assertLess(deltaPos[vehicleTestIndex][1], -0.2)  # make sure the vehicle body fell to the ground

            for j in range(vehicleCount):
                if (j != vehicleTestIndex):
                    if (vehicleEnabled[j]):
                        self.assertGreater(deltaPos[j][2], didMoveThreshold)
                    else:
                        self.assertLess(math.fabs(deltaPos[j][2]), didNotMoveThreshold)

    #
    # switching to disabled should clear certain properties in the wheel state.
    #
    async def test_vehicle_disabled_wheel_state(self):

        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths
        )

        prim = stage.GetPrimAtPath(vehiclePaths[0])
        vehicleAPI = PhysxSchema.PhysxVehicleAPI(prim)

        self._simulate_one_frame_with_prep()

        global gPhysXInterface
        wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[0][0])

        isWheelOnGround = wheelState[VEHICLE_WHEEL_STATE_IS_ON_GROUND]
        self.assertTrue(isWheelOnGround)

        vehicleAPI.GetVehicleEnabledAttr().Set(False)

        wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[0][0])

        isWheelOnGround = wheelState[VEHICLE_WHEEL_STATE_IS_ON_GROUND]
        self.assertFalse(isWheelOnGround)

    #
    # switching from enabled to disabled, using the vehicle body as kinematic and switching back to enabled.
    #
    async def _vehicle_enabled_kinematic(self, driveModeIn):
        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=driveModeIn,
            vehiclePathsOut=vehiclePaths
        )

        prim = stage.GetPrimAtPath(vehiclePaths[0])
        vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI(prim)
        vehicleControllerAPI.GetAcceleratorAttr().Set(1.0)
        if (driveModeIn == Factory.DRIVE_STANDARD):
            vehicleControllerAPI.GetTargetGearAttr().Set(1)
        vehicleAPI = PhysxSchema.PhysxVehicleAPI(prim)
        bodyAPI = UsdPhysics.RigidBodyAPI(prim)

        oldPos = prim.GetAttribute("xformOp:translate").Get()

        self._prepare_for_simulation()

        secondsToRun = 1.0
        self._simulate(secondsToRun)
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=True)

        pos = prim.GetAttribute("xformOp:translate").Get()
        deltaPos = pos - oldPos
        oldPos = pos

        didMoveThreshold = 0.3

        self.assertGreater(deltaPos[2], didMoveThreshold)  # make sure vehicle moved

        vehicleAPI.GetVehicleEnabledAttr().Set(False)
        #--
        bodyAPI.GetKinematicEnabledAttr().Set(True)

        targetDeltaX = 2.0
        targetPos = pos + Gf.Vec3f(targetDeltaX, 0.0, 0.0)
        prim.GetAttribute("xformOp:translate").Set(targetPos)

        self._simulate_one_frame()
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=True)

        pos = prim.GetAttribute("xformOp:translate").Get()
        deltaPos = pos - oldPos
        oldPos = pos
        self.assertGreater(deltaPos[0], (targetDeltaX * 0.9))
        self.assertLess(math.fabs(deltaPos[2]), 0.001)

        bodyAPI.GetKinematicEnabledAttr().Set(False)
        #--
        # clear high velocity from kinematic sim step
        bodyAPI.GetVelocityAttr().Set(Gf.Vec3f(0.0))
        bodyAPI.GetAngularVelocityAttr().Set(Gf.Vec3f(0.0))
        #--
        vehicleAPI.GetVehicleEnabledAttr().Set(True)

        self._simulate(secondsToRun)
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=True)

        pos = prim.GetAttribute("xformOp:translate").Get()
        deltaPos = pos - oldPos
        oldPos = pos
        self.assertGreater(deltaPos[2], didMoveThreshold)  # make sure vehicle moved
        self.assertLess(math.fabs(deltaPos[1]), 0.001)

    #
    # see _vehicle_enabled_kinematic
    #
    async def test_vehicle_enabled_kinematic_drive_standard(self):
        await self._vehicle_enabled_kinematic(Factory.DRIVE_STANDARD)

    #
    # see _vehicle_enabled_kinematic
    #
    async def test_vehicle_enabled_kinematic_drive_basic(self):
        await self._vehicle_enabled_kinematic(Factory.DRIVE_BASIC)

    #
    # Test that vehicles using sweeps behave the same as vehicles using raycasts (on a flat plane).
    #
    async def test_sweep_basics(self):

        stage = await self.new_stage()

        vehicleCount = 2
        vehiclePaths = []

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            useRaycastsList=[True, False]
        )

        prim = []
        startPos = []
        for path in vehiclePaths:
            p = stage.GetPrimAtPath(path)
            prim.append(p)
            controller = PhysxSchema.PhysxVehicleControllerAPI(p)
            controller.GetAcceleratorAttr().Set(1.0)
            startPos.append(p.GetAttribute("xformOp:translate").Get())

        secondsToRun = 1.0
        self._simulate_with_prep(secondsToRun)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        # Check vehicle position
        errTol = 0.01
        endPos = []
        for i in range(2):
            pos = prim[i].GetAttribute("xformOp:translate").Get()
            endPos.append(pos)
            delta = pos - startPos[i]

            self.assertLess(math.fabs(delta[0]), errTol)
            self.assertLess(math.fabs(delta[1]), errTol)
            self.assertGreater(delta[2], 0.3)  # vehicle should have moved

        self.assertLess(math.fabs(endPos[0][2] - endPos[1][2]), errTol)

    #
    # Test that vehicles using sweeps react to ground obstacles where raycasts would not.
    #
    async def test_sweep_ground_gap(self):

        stage = await self.new_stage()

        vehicleCount = 2
        vehiclePaths = []
        wheelAttachmentPaths = []
        collisionGroupPaths = []

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
            collisionGroupPathsOut=collisionGroupPaths,
            useRaycastsList=[False, True]
        )

        groundSurfaceGroupPrimPath = collisionGroupPaths[Factory.COLL_GROUP_GROUND_SURFACE]

        wheelPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][Factory.WHEEL_FRONT_LEFT])
        xformable = UsdGeom.Xformable(wheelPrim)
        pos0 = xformable.ComputeLocalToWorldTransform(0).ExtractTranslation()

        wheelPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][Factory.WHEEL_REAR_LEFT])
        xformable = UsdGeom.Xformable(wheelPrim)
        pos1 = xformable.ComputeLocalToWorldTransform(0).ExtractTranslation()

        boxDeltaPos = 0.3
        boxHalfHeight = 0.1
        boxHeight = 2 * boxHalfHeight
        boxExtent = Gf.Vec3f(10.0, boxHeight, (2*boxDeltaPos) - 0.1)

        startPos = []
        for i in range(vehicleCount):
            prim = stage.GetPrimAtPath(vehiclePaths[i])
            pos = prim.GetAttribute("xformOp:translate").Get()
            pos[1] = pos[1] + boxHeight
            prim.GetAttribute("xformOp:translate").Set(pos)
            startPos.append(pos)

        defaultPath = str(stage.GetDefaultPrim().GetPath())
        boxPathBase = "/box"
        boxPos = [
            Gf.Vec3f(pos0[0], boxHalfHeight, pos0[2] + boxDeltaPos),
            Gf.Vec3f(pos0[0], boxHalfHeight, pos0[2] - boxDeltaPos),
            Gf.Vec3f(pos1[0], boxHalfHeight, pos1[2] + boxDeltaPos),
            Gf.Vec3f(pos1[0], boxHalfHeight, pos1[2] - boxDeltaPos)
        ]
        for i in range(4):
            boxPath = boxPathBase + str(i)
            add_rigid_box(stage, boxPath, boxExtent, boxPos[i], Gf.Quatf(0.0, 0.0, 0.0, 1.0),
                Gf.Vec3f(1), 0.0, Gf.Vec3f(0), Gf.Vec3f(0))
            # add_rigid_box() seems to place it under the default root prim
            boxPath = defaultPath + boxPath
            add_collision_to_collision_group(stage, boxPath, groundSurfaceGroupPrimPath)

        secondsToRun = 1.0
        self._simulate_with_prep(secondsToRun)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        # Check vehicle position
        endPos = []
        for i in range(vehicleCount):
            prim = stage.GetPrimAtPath(vehiclePaths[i])
            endPos.append(prim.GetAttribute("xformOp:translate").Get())

        delta0 = endPos[0] - startPos[0]
        delta1 = endPos[1] - startPos[1]

        self.assertLess(math.fabs(delta0[1]), 0.01)
        self.assertLess(delta1[1], -boxHalfHeight)

    #
    # shifting: gear up.
    #
    async def test_target_gear(self):
        stage = await self.new_stage()

        vehiclePaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            1,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_STANDARD,
            vehiclePathsOut=vehiclePaths
        )

        prim = stage.GetPrimAtPath(vehiclePaths[0])
        vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI(prim)
        vehicleControllerAPI.GetAcceleratorAttr().Set(1.0)
        targetGear = 1
        vehicleControllerAPI.GetTargetGearAttr().Set(targetGear)

        drive = self._get_drive(prim, stage)
        gears = self._get_gears(drive.GetPrim(), stage)
        gearMax = 5
        gearSwitchTime = 0.05
        gearSwitchSteps = math.ceil(gearSwitchTime / self._get_time_step())
        gears.GetSwitchTimeAttr().Set(gearSwitchTime)

        startPos = prim.GetAttribute("xformOp:translate").Get()

        self._prepare_for_simulation()

        global gPhysXInterface
        shifting = False
        for i in range(80):
            driveData = gPhysXInterface.get_vehicle_drive_state(vehiclePaths[0])
            rpm = 60.0 * (driveData[VEHICLE_DRIVE_STATE_ENGINE_ROTATION_SPEED] / (2.0 * math.pi))
            if not shifting:
                if ((rpm > 3000) and (driveData[VEHICLE_DRIVE_STATE_TARGET_GEAR] == driveData[VEHICLE_DRIVE_STATE_CURRENT_GEAR]) and (targetGear < gearMax)):
                    targetGear = targetGear + 1
                    vehicleControllerAPI.GetTargetGearAttr().Set(targetGear)
                    vehicleControllerAPI.GetAcceleratorAttr().Set(0.0)
                    shifting = True
            else:
                if (driveData[VEHICLE_DRIVE_STATE_TARGET_GEAR] == driveData[VEHICLE_DRIVE_STATE_CURRENT_GEAR]):
                    vehicleControllerAPI.GetAcceleratorAttr().Set(1.0)
                    shifting = False

            self._simulate_one_frame()

        driveData = gPhysXInterface.get_vehicle_drive_state(vehiclePaths[0])
        self.assertTrue(driveData[VEHICLE_DRIVE_STATE_CURRENT_GEAR] == gearMax)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=True)

        pos = prim.GetAttribute("xformOp:translate").Get()
        deltaPos = pos - startPos

        # make sure the thing did not explode
        self.assertLess(math.fabs(deltaPos[0]), 0.001)
        self.assertLess(math.fabs(deltaPos[1]), 0.001)

    async def _engine_scenario(self, driveType, attr):
        stage = await self.new_stage()

        if (attr == TEST_ENGINE_ATTR_IDLE_ROT_SPEED):
            vehicleCount = 2
            useShrCompList = [False] * vehicleCount
        else:
            vehicleCount = 1
            useShrCompList = None

        vehiclePaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=driveType,
            vehiclePathsOut=vehiclePaths,
            useShareableComponentsList=useShrCompList
        )

        prim = []
        vehicleStartPos = []
        modElement = []
        for i in range(vehicleCount):
            p = stage.GetPrimAtPath(vehiclePaths[i])
            vehicleController = PhysxSchema.PhysxVehicleControllerAPI(p)
            if (attr == TEST_ENGINE_ATTR_IDLE_ROT_SPEED):
                vehicleController.GetAcceleratorAttr().Set(0.0001) # to avoid sticky tire mode kicking in
            else:
                vehicleController.GetAcceleratorAttr().Set(1.0)
            vehicleStartPos.append(p.GetAttribute("xformOp:translate").Get())
            prim.append(p)

            # get shared drive/engine
            if (driveType == Factory.DRIVE_STANDARD):
                drive = self._get_drive(p, stage)
                modElement.append(self._get_engine(drive.GetPrim(), stage))
            else:
                modElement.append(self._get_drive(p, stage))

        # IMPORTANT: do this before _prepare_for_simulation() as this way another code
        #            path gets tested too (modification before USD parsing)
        if (attr == TEST_ENGINE_ATTR_MOI):
            startMoi = 1e6  # set to a large value to get almost no movement
            modifiedMoi = 1
            modElement[0].GetMoiAttr().Set(startMoi)
        elif (attr == TEST_ENGINE_ATTR_PEAK_TORQUE):
            startPeakTorque = 1.0  # set to a small value to get almost no movement
            modifiedPeakTorque = 500
            modElement[0].GetPeakTorqueAttr().Set(startPeakTorque)
        elif (attr == TEST_ENGINE_ATTR_MAX_ROT_SPEED):
            startMaxRotSpeed = 0.1  # set to a small value to get almost no movement
            modifiedMaxRotSpeed = 600
            modElement[0].GetMaxRotationSpeedAttr().Set(startMaxRotSpeed)
        elif (attr == TEST_ENGINE_ATTR_IDLE_ROT_SPEED):
            modElement[0].GetIdleRotationSpeedAttr().Set(600)
            modElement[1].GetIdleRotationSpeedAttr().Set(0)
        else:
            self.assertTrue(False)

        self._prepare_for_simulation()

        secondsToRun = 2.0
        self._simulate(secondsToRun)
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)
        deltaPos = []
        for i in range(vehicleCount):
            vehicleEndPos = prim[i].GetAttribute("xformOp:translate").Get()
            deltaPos.append(vehicleEndPos - vehicleStartPos[i])

        if (attr == TEST_ENGINE_ATTR_IDLE_ROT_SPEED):
            self.assertGreater(deltaPos[0][2], 1.0)  # high idle speed -> should have moved
            self.assertLess(deltaPos[1][2], 0.09)
        else:
            # the vehicle should only move minimally
            self.assertLess(deltaPos[0][2], 0.09)

        # now change the attribute while the simulation is running
        if (attr == TEST_ENGINE_ATTR_MOI):
            modElement[0].GetMoiAttr().Set(modifiedMoi)
        elif (attr == TEST_ENGINE_ATTR_PEAK_TORQUE):
            modElement[0].GetPeakTorqueAttr().Set(modifiedPeakTorque)
        elif (attr == TEST_ENGINE_ATTR_MAX_ROT_SPEED):
            modElement[0].GetMaxRotationSpeedAttr().Set(modifiedMaxRotSpeed)
        elif (attr == TEST_ENGINE_ATTR_IDLE_ROT_SPEED):
            modElement[1].GetIdleRotationSpeedAttr().Set(600)
        else:
            self.assertTrue(False)

        self._simulate(secondsToRun)
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        deltaPos = []
        for i in range(vehicleCount):
            vehicleEndPos = prim[i].GetAttribute("xformOp:translate").Get()
            deltaPos.append(vehicleEndPos - vehicleStartPos[i])

        if (attr == TEST_ENGINE_ATTR_IDLE_ROT_SPEED):
            self.assertGreater(deltaPos[1][2], 1.0)  # high idle speed -> should have moved
        else:
            self.assertGreater(deltaPos[0][2], 1.0)

    #
    # modifying the engine moment of inertia should affect the simulation accordingly.
    #
    async def test_engine_moi(self):
        await self._engine_scenario(Factory.DRIVE_STANDARD, TEST_ENGINE_ATTR_MOI)

    #
    # modifying the engine peak torque should affect the simulation accordingly.
    #
    async def test_engine_peak_torque(self):
        for i in range(2):

            #
            # 0: drive basic
            # 1: drive standard
            #

            if (i == 0):
                driveType = Factory.DRIVE_BASIC
            else:
                driveType = Factory.DRIVE_STANDARD

            await self._engine_scenario(driveType, TEST_ENGINE_ATTR_PEAK_TORQUE)

    #
    # modifying the engine max rotation speed should affect the simulation accordingly.
    #
    async def test_engine_max_rotation_speed(self):
        await self._engine_scenario(Factory.DRIVE_STANDARD, TEST_ENGINE_ATTR_MAX_ROT_SPEED)

    #
    # modifying the engine idle rotation speed should affect the simulation accordingly.
    #
    async def test_engine_idle_rotation_speed(self):
        await self._engine_scenario(Factory.DRIVE_STANDARD, TEST_ENGINE_ATTR_IDLE_ROT_SPEED)

    #
    # modifying the engine damping rates should affect the simulation accordingly.
    #
    async def test_engine_damping(self):
        for i in range(3):
            stage = await self.new_stage()

            #
            # 0: full throttle
            # 1: zero throttle clutch engaged
            # 2: zero throttle clutch disengaged
            #

            vehicleCount = 1
            vehiclePaths = []
            Factory.create4WheeledCarsScenario(
                stage,
                1.0,
                vehicleCount,
                createCollisionShapesForWheels=True,
                driveMode=Factory.DRIVE_STANDARD,
                vehiclePathsOut=vehiclePaths
            )

            prim = stage.GetPrimAtPath(vehiclePaths[0])
            vehicleController = PhysxSchema.PhysxVehicleControllerAPI(prim)
            vehicleController.GetAcceleratorAttr().Set(1.0)
            vehicleController.GetTargetGearAttr().Set(1)

            vehicleStartPos = prim.GetAttribute("xformOp:translate").Get()

            drive = self._get_drive(prim, stage)
            engine = self._get_engine(drive.GetPrim(), stage)

            highDampingValue = 1e6

            if (i == 0):
                startDamping = highDampingValue  # set to a large value to get almost no movement
                modifiedDamping = 0.15
                engine.GetDampingRateFullThrottleAttr().Set(startDamping)
            else:
                modifiedDamping = highDampingValue

            self._prepare_for_simulation()

            secondsToRun = 2.0
            self._simulate(secondsToRun)
            self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)
            vehicleEndPos = prim.GetAttribute("xformOp:translate").Get()

            # the vehicle should only move minimally
            deltaForward = vehicleEndPos[2] - vehicleStartPos[2]
            if (i == 0):
                self.assertLess(deltaForward, 0.05)
            else:
                self.assertGreater(deltaForward, 1.0)

            # now change the damping while the simulation is running
            if (i == 0):
                engine.GetDampingRateFullThrottleAttr().Set(modifiedDamping)
            elif (i == 1):
                vehicleController.GetAcceleratorAttr().Set(0.0)

                # set to a large value to stop fast
                engine.GetDampingRateZeroThrottleClutchEngagedAttr().Set(modifiedDamping)
            else:
                vehicleController.GetAcceleratorAttr().Set(0.0)
                vehicleController.GetTargetGearAttr().Set(0)

                # set to a large value to stop fast
                engine.GetDampingRateZeroThrottleClutchDisengagedAttr().Set(modifiedDamping)

            self._simulate(secondsToRun)
            self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)
            vehicleEndPos = prim.GetAttribute("xformOp:translate").Get()

            if (i == 0):
                deltaForward = vehicleEndPos[2] - vehicleStartPos[2]
                self.assertGreater(deltaForward, 1.0)
            else:
                global gPhysXInterface
                driveData = gPhysXInterface.get_vehicle_drive_state(vehiclePaths[0])
                self.assertLess(driveData[VEHICLE_DRIVE_STATE_ENGINE_ROTATION_SPEED], 0.001)
                # without damping changes, the rotation speed would be above 40

    #
    # modifying the mass of the vehicle should trigger recomputation of sprung mass values.
    #
    async def test_mass_change(self):
        stage = await self.new_stage()

        vehicleCount = 2
        vehiclePaths = []
        wheelAttachmentPaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths
        )

        vehiclePrim = []
        for i in range(vehicleCount):
            vehiclePrim.append(stage.GetPrimAtPath(vehiclePaths[i]))

        massAPI = UsdPhysics.MassAPI(vehiclePrim[0])
        baseMass = massAPI.GetMassAttr().Get()
        newMass = baseMass * 2.0

        self._prepare_for_simulation()

        # change mass of one vehicle while playing
        massAPI = UsdPhysics.MassAPI(vehiclePrim[1])
        massAPI.GetMassAttr().Set(newMass)

        secondsToRun = 2.0
        self._simulate(secondsToRun)

        suspForce = []
        global gPhysXInterface
        for i in range(vehicleCount):
            wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[i][0])
            sf = wheelState[VEHICLE_WHEEL_STATE_SUSPENSION_FORCE]
            sfMagn = math.sqrt((sf[0] * sf[0]) + (sf[1] * sf[1]) + (sf[2] * sf[2]))
            suspForce.append(sfMagn)

        self.assertGreater(suspForce[1], (suspForce[0] * 1.9))  # not using 2 to give some margin for simulation dynamics

    async def _suspension_scenario(self, attr):
        stage = await self.new_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        if ((attr == TEST_SUSPENSION_ATTR_MAX_COMPRESSION) or
            (attr == TEST_SUSPENSION_ATTR_MAX_DROOP) or
            (attr == TEST_SUSPENSION_ATTR_CAMBER_AT_REST) or
            (attr == TEST_SUSPENSION_ATTR_CAMBER_AT_MAX_COMPRESSION) or
            (attr == TEST_SUSPENSION_ATTR_CAMBER_AT_MAX_DROOP)):
            useDepAPIList = [True, True]
        else:
            useDepAPIList = None

        vehicleCount = 2
        vehiclePaths = []
        wheelAttachmentPaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
            useDeprecatedAPIList=useDepAPIList
        )

        mass = 0
        vehicleStartPos = []
        for i in range(vehicleCount):
            p = stage.GetPrimAtPath(vehiclePaths[i])
            vehicleStartPos.append(p.GetAttribute("xformOp:translate").Get())
            massAPI = UsdPhysics.MassAPI(p)
            mass = massAPI.GetMassAttr().Get()

        wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][Factory.WHEEL_FRONT_LEFT])
        susp1 = self._get_suspension(wheelAttPrim, stage)

        wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][Factory.WHEEL_REAR_RIGHT])
        susp2 = self._get_suspension(wheelAttPrim, stage)

        # change factory result such that vehicle 1 uses suspension 1 and vehicle 2 uses
        # suspension 2
        for i in range(2):
            for j in range(len(wheelAttachmentPaths[i])):
                wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[i][j])
                wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(wheelAttPrim)
                wheelAttAPI.GetSuspensionRel().ClearTargets(True)
                if (i == 0):
                    wheelAttAPI.GetSuspensionRel().AddTarget(susp1.GetPath())
                else:
                    wheelAttAPI.GetSuspensionRel().AddTarget(susp2.GetPath())

        if (attr == TEST_SUSPENSION_ATTR_SPRING_STRENGTH):
            susp1SpringStrength = susp1.GetSpringStrengthAttr().Get()
            susp2SpringStrength = susp2.GetSpringStrengthAttr().Get()

            self.assertEqual(susp1SpringStrength, susp2SpringStrength)
        elif (attr == TEST_SUSPENSION_ATTR_SPRING_DAMPER_RATE):
            susp1SpringDamperRate = susp1.GetSpringDamperRateAttr().Get()
            susp2SpringDamperRate = susp2.GetSpringDamperRateAttr().Get()

            self.assertEqual(susp1SpringDamperRate, susp2SpringDamperRate)
        elif (attr == TEST_SUSPENSION_ATTR_MAX_COMPRESSION):
            susp1MaxCompression = susp1.GetMaxCompressionAttr().Get()
            susp2MaxCompression = susp2.GetMaxCompressionAttr().Get()

            self.assertEqual(susp1MaxCompression, susp2MaxCompression)
        elif (attr == TEST_SUSPENSION_ATTR_MAX_DROOP):
            susp1MaxDroop = susp1.GetMaxDroopAttr().Get()
            susp2MaxDroop = susp2.GetMaxDroopAttr().Get()

            self.assertEqual(susp1MaxDroop, susp2MaxDroop)
        elif (attr == TEST_SUSPENSION_ATTR_TRAVEL_DISTANCE):
            susp1TravelDist = susp1.GetTravelDistanceAttr().Get()
            susp2TravelDist = susp2.GetTravelDistanceAttr().Get()

            self.assertEqual(susp1TravelDist, susp2TravelDist)
        elif (attr == TEST_SUSPENSION_ATTR_SPRUNG_MASS):
            sprungMass = mass / 4
            susp1.GetSprungMassAttr().Set(sprungMass)
            susp2.GetSprungMassAttr().Set(sprungMass)
        elif ((attr == TEST_SUSPENSION_ATTR_CAMBER_AT_REST) or
              (attr == TEST_SUSPENSION_ATTR_CAMBER_AT_MAX_COMPRESSION) or
              (attr == TEST_SUSPENSION_ATTR_CAMBER_AT_MAX_DROOP)):
            camber = (math.pi / 180.0) * 30.0
        else:
            self.assertTrue(False)

        self._prepare_for_simulation()

        global gPhysXInterface

        stepsToRun = 1 * 60
        if (attr == TEST_SUSPENSION_ATTR_MAX_COMPRESSION):
            forceMass = mass * 1.5  # chosen to hit max compression
        elif (attr == TEST_SUSPENSION_ATTR_MAX_DROOP):
            forceMass = 0
        elif (attr == TEST_SUSPENSION_ATTR_TRAVEL_DISTANCE):
            forceMass = 0
        elif (attr == TEST_SUSPENSION_ATTR_SPRUNG_MASS):
            forceMass = 0
        elif (attr == TEST_SUSPENSION_ATTR_CAMBER_AT_REST):
            forceMass = 0
        elif (attr == TEST_SUSPENSION_ATTR_CAMBER_AT_MAX_COMPRESSION):
            forceMass = mass * 1.5  # chosen to hit max compression
        elif (attr == TEST_SUSPENSION_ATTR_CAMBER_AT_MAX_DROOP):
            forceMass = -mass * 1.5  # chosen to hit max droop (vehicle in air)
        else:
            forceMass = mass * 0.2  # chosen to not hit max compression

        # mass has been explicitly authored here and does not need to be scaled by kilogramsPerUnit
        # metersPerUnit is set to "1" for this test
        force = forceMass * 9.81

        for i in range(stepsToRun):
            for j in range(vehicleCount):
                rbo_encoded = PhysicsSchemaTools.sdfPathToInt(vehiclePaths[j])
                get_physx_simulation_interface().apply_force_at_pos(stage_id, rbo_encoded, Gf.Vec3f(0.0, -force, 0.0),
                    vehicleStartPos[j], "Force")

            self._simulate_one_frame()

        if (attr == TEST_SUSPENSION_ATTR_SPRING_STRENGTH):
            susp2SpringStrength = 0.8 * susp2SpringStrength
            susp2.GetSpringStrengthAttr().Set(susp2SpringStrength)
        elif (attr == TEST_SUSPENSION_ATTR_SPRING_DAMPER_RATE):
            susp2SpringDamperRate = 5.0 * susp2SpringDamperRate
            susp2.GetSpringDamperRateAttr().Set(susp2SpringDamperRate)
        elif (attr == TEST_SUSPENSION_ATTR_MAX_COMPRESSION):
            susp2MaxCompression = 2.0 * susp2MaxCompression
            susp2.GetMaxCompressionAttr().Set(susp2MaxCompression)
        elif (attr == TEST_SUSPENSION_ATTR_MAX_DROOP):
            susp2MaxDroop = 2.0 * susp2MaxDroop
            susp2.GetMaxDroopAttr().Set(susp2MaxDroop)

            stepsToRun = 1

            for i in range(vehicleCount):
                newPos = vehicleStartPos[i]
                newPos[1] += susp2MaxDroop - 0.01
                p = stage.GetPrimAtPath(vehiclePaths[i])
                p.GetAttribute("xformOp:translate").Set(newPos)
        elif (attr == TEST_SUSPENSION_ATTR_TRAVEL_DISTANCE):
            wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[1][0])
            restJounce = wheelState[VEHICLE_WHEEL_STATE_SUSPENSION_JOUNCE]
            extraSuspTravelDist = restJounce
            susp2TravelDist = susp2TravelDist + extraSuspTravelDist
            susp2.GetTravelDistanceAttr().Set(susp2TravelDist)

            stepsToRun = 1

            for i in range(vehicleCount):
                newPos = vehicleStartPos[i]
                newPos[1] += (restJounce + extraSuspTravelDist) - 0.01
                p = stage.GetPrimAtPath(vehiclePaths[i])
                p.GetAttribute("xformOp:translate").Set(newPos)
        elif (attr == TEST_SUSPENSION_ATTR_SPRUNG_MASS):
            susp2.GetSprungMassAttr().Set(sprungMass * 0.5)
        elif (attr == TEST_SUSPENSION_ATTR_CAMBER_AT_REST):
            susp2.GetCamberAtRestAttr().Set(camber)
        elif (attr == TEST_SUSPENSION_ATTR_CAMBER_AT_MAX_COMPRESSION):
            susp2.GetCamberAtMaxCompressionAttr().Set(camber)
        elif (attr == TEST_SUSPENSION_ATTR_CAMBER_AT_MAX_DROOP):
            susp2.GetCamberAtMaxDroopAttr().Set(camber)

        for i in range(stepsToRun):
            if (attr != TEST_SUSPENSION_ATTR_SPRING_DAMPER_RATE):
                for j in range(vehicleCount):
                    rbo_encoded = PhysicsSchemaTools.sdfPathToInt(vehiclePaths[j])
                    get_physx_simulation_interface().apply_force_at_pos(stage_id, rbo_encoded, Gf.Vec3f(0.0, -force, 0.0),
                        vehicleStartPos[j], "Force")

            self._simulate_one_frame()

        suspJounce = []
        orientation = []
        for i in range(vehicleCount):
            wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[i][0])
            suspJounce.append(wheelState[VEHICLE_WHEEL_STATE_SUSPENSION_JOUNCE])
            orientation.append(wheelState[VEHICLE_WHEEL_STATE_LOCAL_POSE_QUATERNION])

        if (attr == TEST_SUSPENSION_ATTR_SPRING_STRENGTH):
            testRatio = (susp1SpringStrength / susp2SpringStrength) * 0.9  # 0.9 to give some margin for simulation dynamics, damping etc.
            self.assertGreater(suspJounce[1], (suspJounce[0] * testRatio))
        elif (attr == TEST_SUSPENSION_ATTR_SPRING_DAMPER_RATE):
            self.assertGreater((suspJounce[1] - suspJounce[0]), 0.002)
        elif (attr == TEST_SUSPENSION_ATTR_MAX_COMPRESSION):
            self.assertGreater(suspJounce[1], (susp1MaxCompression * 1.2))
        elif (attr == TEST_SUSPENSION_ATTR_MAX_DROOP):
            self.assertEqual(suspJounce[0], 0.0)  # should not touch ground
            self.assertGreater(suspJounce[1], 0.0)  # should touch ground
        elif (attr == TEST_SUSPENSION_ATTR_TRAVEL_DISTANCE):
            self.assertEqual(suspJounce[0], 0.0)  # should not touch ground
            self.assertGreater(suspJounce[1], 0.0)  # should touch ground
        elif (attr == TEST_SUSPENSION_ATTR_SPRUNG_MASS):
            # to test the effect of the change, the suspension direction could, for example,
            # get adjusted to form a 45 degree angle relative to the ground and chassis.
            # For now, this just tests that there is no crash and that there is no impact.
            self.assertAlmostEqual(suspJounce[0], suspJounce[1], delta=0.001)
        elif ((attr == TEST_SUSPENSION_ATTR_CAMBER_AT_REST) or
              (attr == TEST_SUSPENSION_ATTR_CAMBER_AT_MAX_COMPRESSION) or
              (attr == TEST_SUSPENSION_ATTR_CAMBER_AT_MAX_DROOP)):
            refDir = Gf.Vec3f(0, 1, 0)
            oneDegree = (math.pi / 180.0)

            rotQuat = Gf.Quatf(orientation[0][1], orientation[0][2], orientation[0][3], orientation[0][0])
            rot = Gf.Rotation(rotQuat)
            refDir0 = rot.TransformDir(refDir)
            refDir0[2] = 0
            refDir0.Normalize()
            deltaAngle = self._get_delta_angle_radians(refDir0, refDir)
            self.assertLess(math.fabs(deltaAngle), oneDegree)

            rotQuat = Gf.Quatf(orientation[1][1], orientation[1][2], orientation[1][3], orientation[1][0])
            rot = Gf.Rotation(rotQuat)
            refDir1 = rot.TransformDir(refDir)
            refDir1[2] = 0
            refDir1.Normalize()
            deltaAngle = self._get_delta_angle_radians(refDir1, refDir)
            self.assertLess(math.fabs(deltaAngle - camber), oneDegree)

    #
    # modifying the suspension spring strength should affect the simulation accordingly.
    #
    async def test_suspension_spring_strength(self):
        await self._suspension_scenario(TEST_SUSPENSION_ATTR_SPRING_STRENGTH)

    #
    # modifying the suspension spring damping rate should affect the simulation accordingly.
    #
    async def test_suspension_spring_damper_rate(self):
        await self._suspension_scenario(TEST_SUSPENSION_ATTR_SPRING_DAMPER_RATE)

    #
    # modifying the suspension max compression should affect the simulation accordingly.
    #
    async def test_suspension_max_compression(self):
        await self._suspension_scenario(TEST_SUSPENSION_ATTR_MAX_COMPRESSION)

    #
    # modifying the suspension max droop should affect the simulation accordingly.
    #
    async def test_suspension_max_droop(self):
        await self._suspension_scenario(TEST_SUSPENSION_ATTR_MAX_DROOP)

    #
    # modifying the travel distance should affect the simulation accordingly.
    #
    async def test_suspension_travel_distance(self):
        await self._suspension_scenario(TEST_SUSPENSION_ATTR_TRAVEL_DISTANCE)

    #
    # modifying the suspension sprung mass should affect the simulation accordingly.
    #
    async def test_suspension_sprung_mass(self):
        await self._suspension_scenario(TEST_SUSPENSION_ATTR_SPRUNG_MASS)

    #
    # modifying the camber at rest should affect the simulation accordingly.
    #
    async def test_suspension_camber_at_rest(self):
        await self._suspension_scenario(TEST_SUSPENSION_ATTR_CAMBER_AT_REST)

    #
    # modifying the camber at max compression should affect the simulation accordingly.
    #
    async def test_suspension_camber_at_max_compression(self):
        await self._suspension_scenario(TEST_SUSPENSION_ATTR_CAMBER_AT_MAX_COMPRESSION)

    #
    # modifying the camber at max droop should affect the simulation accordingly.
    #
    async def test_suspension_camber_at_max_droop(self):
        await self._suspension_scenario(TEST_SUSPENSION_ATTR_CAMBER_AT_MAX_DROOP)

    #
    # verify behavior when max droop is auto-computed
    #
    async def test_suspension_max_droop_auto_compute_deprecated(self):
        stage = await self.new_stage()

        vehicleCount = 2
        vehiclePaths = []
        wheelAttachmentPaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
            useShareableComponentsList=[False, False],
            useDeprecatedAPIList=[True, True]
        )

        vehicleStartPos = []
        for i in range(vehicleCount):
            p = stage.GetPrimAtPath(vehiclePaths[i])
            vehicleStartPos.append(p.GetAttribute("xformOp:translate").Get())

        # the second vehicle should compute max droop automatically
        for path in wheelAttachmentPaths[1]:
            wheelAttPrim = stage.GetPrimAtPath(path)
            susp = self._get_suspension(wheelAttPrim, stage)
            susp.GetMaxDroopAttr().Set(-1.0)

        self._simulate_with_prep(1.0);
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        vehicleDeltaPos = []
        for i in range(vehicleCount):
            p = stage.GetPrimAtPath(vehiclePaths[i])
            endPos = p.GetAttribute("xformOp:translate").Get()
            vehicleDeltaPos.append(endPos - vehicleStartPos[i])

        # both vehicles should end up at the same up value
        self.assertAlmostEqual(vehicleDeltaPos[0][1], vehicleDeltaPos[1][1], delta=0.001)

    async def _tire_scenario(self, attr):
        stage = await self.new_stage()

        if ((attr == TEST_TIRE_ATTR_LAT_STIFF_X) or
            (attr == TEST_TIRE_ATTR_LAT_STIFF_Y) or
            (attr == TEST_TIRE_ATTR_LONG_STIFF_PER_GRAV) or
            (attr == TEST_TIRE_ATTR_CAMBER_STIFF_PER_GRAV)):
            useDepAPIList = [True, True]
        else:
            useDepAPIList = None

        vehicleCount = 2
        vehiclePaths = []
        wheelAttachmentPaths = []
        tireFrictionTablePaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
            vehicleDelta=[-10, 0, 0],
            tireFrictionTablePathsOut=tireFrictionTablePaths,
            useDeprecatedAPIList=useDepAPIList
        )

        startSpeed = 10
        vehicleStartPos = []
        for i in range(vehicleCount):
            p = stage.GetPrimAtPath(vehiclePaths[i])
            vehicleStartPos.append(p.GetAttribute("xformOp:translate").Get())

        latStiffY = 17
        latStiffGraph = Gf.Vec2f(2.0, 17 * 4500.0)

        wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][Factory.WHEEL_FRONT_LEFT])
        tire1 = self._get_tire(wheelAttPrim, stage)

        wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][Factory.WHEEL_REAR_RIGHT])
        tire2 = self._get_tire(wheelAttPrim, stage)

        if (useDepAPIList is not None):
            tire1.GetLatStiffYAttr().Set(latStiffY)
            tire2.GetLatStiffYAttr().Set(latStiffY)
        else:
            tire1.GetLateralStiffnessGraphAttr().Set(latStiffGraph)
            tire2.GetLateralStiffnessGraphAttr().Set(latStiffGraph)

        # change factory result such that vehicle 1 uses tire 1 and vehicle 2 uses
        # tire 2
        for i in range(2):
            for j in range(len(wheelAttachmentPaths[i])):
                wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[i][j])
                wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(wheelAttPrim)
                wheelAttAPI.GetTireRel().ClearTargets(True)
                if (i == 0):
                    wheelAttAPI.GetTireRel().AddTarget(tire1.GetPath())
                else:
                    wheelAttAPI.GetTireRel().AddTarget(tire2.GetPath())

        if (attr == TEST_TIRE_ATTR_LAT_STIFF_X):
            tire1LatStiffX = tire1.GetLatStiffXAttr().Get()
            tire2LatStiffX = tire2.GetLatStiffXAttr().Get()

            self.assertEqual(tire1LatStiffX, tire2LatStiffX)
        elif (attr == TEST_TIRE_ATTR_LAT_STIFF_Y):
            tire1LatStiffY = tire1.GetLatStiffYAttr().Get()
            tire2LatStiffY = tire2.GetLatStiffYAttr().Get()

            self.assertEqual(tire1LatStiffY, tire2LatStiffY)
        elif (attr == TEST_TIRE_ATTR_LAT_STIFF_GRAPH_X):
            tire1LatStiffX = tire1.GetLateralStiffnessGraphAttr().Get()[0]
            tire2LatStiffX = tire2.GetLateralStiffnessGraphAttr().Get()[0]

            self.assertEqual(tire1LatStiffX, tire2LatStiffX)
        elif (attr == TEST_TIRE_ATTR_LAT_STIFF_GRAPH_Y):
            tire1LatStiffY = tire1.GetLateralStiffnessGraphAttr().Get()[1]
            tire2LatStiffY = tire2.GetLateralStiffnessGraphAttr().Get()[1]

            self.assertEqual(tire1LatStiffY, tire2LatStiffY)
        elif (attr == TEST_TIRE_ATTR_LONG_STIFF_PER_GRAV):
            tire1LongStiffPerGrav = tire1.GetLongitudinalStiffnessPerUnitGravityAttr().Get()
            tire2LongStiffPerGrav = tire2.GetLongitudinalStiffnessPerUnitGravityAttr().Get()

            self.assertEqual(tire1LongStiffPerGrav, tire2LongStiffPerGrav)
        elif (attr == TEST_TIRE_ATTR_LONG_STIFF):
            tire1LongStiff = tire1.GetLongitudinalStiffnessAttr().Get()
            tire2LongStiff = tire2.GetLongitudinalStiffnessAttr().Get()

            self.assertEqual(tire1LongStiff, tire2LongStiff)
        elif (attr == TEST_TIRE_ATTR_CAMBER_STIFF_PER_GRAV):
            tire1CamberStiffPerGrav = 360
            tire2CamberStiffPerGrav = tire1CamberStiffPerGrav
            tire1.GetCamberStiffnessPerUnitGravityAttr().Set(tire1CamberStiffPerGrav)
            tire2.GetCamberStiffnessPerUnitGravityAttr().Set(tire2CamberStiffPerGrav)

            self.assertEqual(tire1CamberStiffPerGrav, tire2CamberStiffPerGrav)

            camber = (math.pi / 180) * 5.0  # with no steer, the vehicle is expected to drift to its right

            suspAPIList = []
            wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][Factory.WHEEL_FRONT_LEFT])
            suspAPIList.append(self._get_suspension(wheelAttPrim, stage))

            wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][Factory.WHEEL_REAR_RIGHT])
            suspAPIList.append(self._get_suspension(wheelAttPrim, stage))

            for suspAPI in suspAPIList:
                suspAPI.GetCamberAtRestAttr().Set(camber)
                suspAPI.GetCamberAtMaxCompressionAttr().Set(camber)
                suspAPI.GetCamberAtMaxDroopAttr().Set(camber)
        elif (attr == TEST_TIRE_ATTR_CAMBER_STIFF):
            tire1CamberStiff = 3600
            tire2CamberStiff = tire1CamberStiff
            tire1.GetCamberStiffnessAttr().Set(tire1CamberStiff)
            tire2.GetCamberStiffnessAttr().Set(tire2CamberStiff)

            self.assertEqual(tire1CamberStiff, tire2CamberStiff)

            camber = (math.pi / 180) * 5.0  # with no steer, the vehicle is expected to drift to its right
            for i in range(2):
                for path in wheelAttachmentPaths[i]:
                    wheelAttPrim = stage.GetPrimAtPath(path)
                    suspComplAPI = PhysxSchema.PhysxVehicleSuspensionComplianceAPI(wheelAttPrim)
                    suspComplAPI.GetWheelCamberAngleAttr().Set([Gf.Vec2f(0.0, camber)])
        elif (attr == TEST_TIRE_ATTR_TIRE_FRICTION_TABLE):
            tireFrictionTablePrim = stage.GetPrimAtPath(tireFrictionTablePaths[Factory.TIRE_FRICTION_TABLE_SUMMER_TIRE])
            tireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable(tireFrictionTablePrim)
            frictionValues = tireFrictionTable.GetFrictionValuesAttr().Get()
            for i in range(len(frictionValues)):
                frictionValues[i] = 0.0
            tireFrictionTable.GetFrictionValuesAttr().Set(frictionValues)

            self.assertEqual(tire2.GetFrictionTableRel().GetTargets()[0], tireFrictionTablePaths[Factory.TIRE_FRICTION_TABLE_WINTER_TIRE])
        elif (attr == TEST_TIRE_ATTR_REST_LOAD):
            tire2InitialRestLoad = (1800 / 4) * 9.81
            tire2.GetRestLoadAttr().Set(tire2InitialRestLoad)
        else:
            self.assertTrue(False)

        self._simulate_one_frame_with_prep()

        global gPhysXInterface

        for i in range(vehicleCount):
            p = stage.GetPrimAtPath(vehiclePaths[i])
            vehContrAPI = PhysxSchema.PhysxVehicleControllerAPI(p)

            if ((attr == TEST_TIRE_ATTR_LAT_STIFF_X) or
                (attr == TEST_TIRE_ATTR_LAT_STIFF_Y) or
                (attr == TEST_TIRE_ATTR_LAT_STIFF_GRAPH_X) or
                (attr == TEST_TIRE_ATTR_LAT_STIFF_GRAPH_Y) or
                (attr == TEST_TIRE_ATTR_CAMBER_STIFF_PER_GRAV) or
                (attr == TEST_TIRE_ATTR_CAMBER_STIFF)):

                # setup vehicle to drive with certain velocity
                self._set_vehicle_speed(stage, p, wheelAttachmentPaths[i], startSpeed,
                    Gf.Vec3f(0, 0, 1), gPhysXInterface)

                if ((attr != TEST_TIRE_ATTR_CAMBER_STIFF_PER_GRAV) and
                    (attr != TEST_TIRE_ATTR_CAMBER_STIFF)):
                    vehContrAPI.GetSteerAttr().Set(1.0)

            elif ((attr == TEST_TIRE_ATTR_LONG_STIFF_PER_GRAV) or
                  (attr == TEST_TIRE_ATTR_LONG_STIFF) or
                  (attr == TEST_TIRE_ATTR_TIRE_FRICTION_TABLE) or
                  (attr == TEST_TIRE_ATTR_REST_LOAD)):
                vehContrAPI.GetAcceleratorAttr().Set(1.0)

        if (attr == TEST_TIRE_ATTR_LAT_STIFF_X):
            tire2LatStiffX = 100 * tire2LatStiffX
            tire2.GetLatStiffXAttr().Set(tire2LatStiffX)
        elif (attr == TEST_TIRE_ATTR_LAT_STIFF_Y):
            tire2LatStiffY = 0.1
            tire2.GetLatStiffYAttr().Set(tire2LatStiffY)
        elif (attr == TEST_TIRE_ATTR_LAT_STIFF_GRAPH_X):
            latStiff = tire2.GetLateralStiffnessGraphAttr().Get()
            tire2LatStiffX = 100 * tire2LatStiffX
            latStiff[0] = tire2LatStiffX
            tire2.GetLateralStiffnessGraphAttr().Set(latStiff)
        elif (attr == TEST_TIRE_ATTR_LAT_STIFF_GRAPH_Y):
            latStiff = tire2.GetLateralStiffnessGraphAttr().Get()
            tire2LatStiffY = 100
            latStiff[1] = tire2LatStiffY
            tire2.GetLateralStiffnessGraphAttr().Set(latStiff)
        elif (attr == TEST_TIRE_ATTR_LONG_STIFF_PER_GRAV):
            tire2LongStiffPerGrav = 0.1
            tire2.GetLongitudinalStiffnessPerUnitGravityAttr().Set(tire2LongStiffPerGrav)
        elif (attr == TEST_TIRE_ATTR_LONG_STIFF):
            tire2LongStiff = 1.0
            tire2.GetLongitudinalStiffnessAttr().Set(tire2LongStiff)
        elif (attr == TEST_TIRE_ATTR_CAMBER_STIFF_PER_GRAV):
            tire2CamberStiffPerGrav = 100 * tire2CamberStiffPerGrav
            tire2.GetCamberStiffnessPerUnitGravityAttr().Set(tire2CamberStiffPerGrav)
        elif (attr == TEST_TIRE_ATTR_CAMBER_STIFF):
            tire2CamberStiff = 100 * tire2CamberStiff
            tire2.GetCamberStiffnessAttr().Set(tire2CamberStiff)
        elif (attr == TEST_TIRE_ATTR_TIRE_FRICTION_TABLE):
            targets = tire2.GetFrictionTableRel().GetTargets()
            self.assertEqual(len(targets), 1)
            targets[0] = tireFrictionTable.GetPath()
            tire2.GetFrictionTableRel().SetTargets(targets)
        elif (attr == TEST_TIRE_ATTR_REST_LOAD):
            tire2InitialRestLoad
            tire2.GetRestLoadAttr().Set(tire2InitialRestLoad * 0.1)

        self._simulate(1)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        vehicleEndPos = []
        for i in range(vehicleCount):
            p = stage.GetPrimAtPath(vehiclePaths[i])
            vehicleEndPos.append(p.GetAttribute("xformOp:translate").Get())

        delta0 = vehicleEndPos[0] - vehicleStartPos[0]
        delta1 = vehicleEndPos[1] - vehicleStartPos[1]

        if ((attr == TEST_TIRE_ATTR_LAT_STIFF_X) or (attr == TEST_TIRE_ATTR_LAT_STIFF_Y) or
            (attr == TEST_TIRE_ATTR_LAT_STIFF_GRAPH_X) or (attr == TEST_TIRE_ATTR_LAT_STIFF_GRAPH_Y)):
            # with low stiffness values, the tires can not apply much force and thus there will be not
            # much turning compared to the default values
            self.assertGreater(delta0[0], (2 * delta1[0]))
        elif ((attr == TEST_TIRE_ATTR_LONG_STIFF_PER_GRAV) or (attr == TEST_TIRE_ATTR_LONG_STIFF)):
            # with low stiffness values, the tires can not apply much force and thus there will be not
            # much forward movement compared to the default values
            self.assertGreater(delta0[2], (2 * delta1[2]))
        elif ((attr == TEST_TIRE_ATTR_CAMBER_STIFF_PER_GRAV) or (attr == TEST_TIRE_ATTR_CAMBER_STIFF)):
            # higher camber stiffness -> more force and the vehicle is expected to drift more to its right
            self.assertLess((delta1[0] - delta0[0]), -0.02)
        elif (attr == TEST_TIRE_ATTR_TIRE_FRICTION_TABLE):
            # switching to a tire friction table with low friction values should prevent the vehicle
            # from moving forward
            self.assertGreater(delta0[2], (2 * delta1[2]))
        elif (attr == TEST_TIRE_ATTR_REST_LOAD):
            # decreasing rest load should result in slightly more grip
            self.assertGreater(delta1[2], (delta0[2] * 1.005))

    #
    # modifying the tire lateral stiffness X parameter should affect the simulation accordingly.
    #
    async def test_tire_lat_stiff_x(self):
        await self._tire_scenario(TEST_TIRE_ATTR_LAT_STIFF_X)

    #
    # modifying the tire lateral stiffness Y parameter should affect the simulation accordingly.
    #
    async def test_tire_lat_stiff_y(self):
        await self._tire_scenario(TEST_TIRE_ATTR_LAT_STIFF_Y)

    #
    # modifying the tire lateral stiffness parameter should affect the simulation accordingly.
    #
    async def test_tire_lat_stiff_graph_x(self):
        await self._tire_scenario(TEST_TIRE_ATTR_LAT_STIFF_GRAPH_X)

    #
    # modifying the tire lateral stiffness parameter should affect the simulation accordingly.
    #
    async def test_tire_lat_stiff_graph_y(self):
        await self._tire_scenario(TEST_TIRE_ATTR_LAT_STIFF_GRAPH_Y)

    #
    # modifying the tire longitudinal stiffness per unit gravity parameter should affect the
    # simulation accordingly.
    #
    async def test_tire_long_stiff_per_grav(self):
        await self._tire_scenario(TEST_TIRE_ATTR_LONG_STIFF_PER_GRAV)

    #
    # modifying the tire longitudinal stiffness parameter should affect the
    # simulation accordingly.
    #
    async def test_tire_long_stiff(self):
        await self._tire_scenario(TEST_TIRE_ATTR_LONG_STIFF)

    #
    # modifying the tire camber stiffness per unit gravity parameter should affect the
    # simulation accordingly.
    #
    async def test_tire_camber_stiff_per_grav(self):
        await self._tire_scenario(TEST_TIRE_ATTR_CAMBER_STIFF_PER_GRAV)

    #
    # modifying the tire camber stiffness parameter should affect the
    # simulation accordingly.
    #
    async def test_tire_camber_stiff(self):
        await self._tire_scenario(TEST_TIRE_ATTR_CAMBER_STIFF)

    #
    # modifying the tire friction table relationship should affect the
    # simulation accordingly.
    #
    async def test_tire_change_tire_friction_table(self):
        await self._tire_scenario(TEST_TIRE_ATTR_TIRE_FRICTION_TABLE)

    #
    # modifying the tire rest load parameter should affect the simulation accordingly.
    #
    async def test_tire_rest_load(self):
        await self._tire_scenario(TEST_TIRE_ATTR_REST_LOAD)

    async def _tire_min_slip_denom_scenario(self, attr):
        stage = await self.new_stage()

        if (attr == TEST_TIRE_MIN_SLIP_DENOM_LAT):
            vehDelta=[0, 0, -10]
        else:
            vehDelta=[-10, 0, 0]

        vehicleCount = 2
        vehiclePaths = []
        wheelAttachmentPaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_STANDARD,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
            vehicleDelta=vehDelta
        )

        startSpeed = 10
        vehicleStartPos = []
        vehicleAPI = []
        for i in range(vehicleCount):
            p = stage.GetPrimAtPath(vehiclePaths[i])
            vehicleStartPos.append(p.GetAttribute("xformOp:translate").Get())
            vehAPI = PhysxSchema.PhysxVehicleAPI(p)
            vehicleAPI.append(vehAPI)

            # note: using standard drive to have vehicle decelerate much faster
            #       for the passive (no drive/brake torque) case
            vehContrAPI = PhysxSchema.PhysxVehicleControllerAPI(p)
            vehContrAPI.GetTargetGearAttr().Set(1)

            # make sure sticky tire mode will not kick in
            vehAPI.GetLongitudinalStickyTireThresholdSpeedAttr().Set(0)
            vehAPI.GetLateralStickyTireThresholdSpeedAttr().Set(0)

        self._simulate_one_frame_with_prep()

        global gPhysXInterface

        for i in range(vehicleCount):
            p = stage.GetPrimAtPath(vehiclePaths[i])
            vehContrAPI = PhysxSchema.PhysxVehicleControllerAPI(p)
            rigidBodyAPI = UsdPhysics.RigidBodyAPI(p)

            if (attr == TEST_TIRE_MIN_SLIP_DENOM_LONG_PASSIVE):
                # setup vehicle to drive with certain velocity
                self._set_vehicle_speed(stage, p, wheelAttachmentPaths[i], startSpeed,
                    Gf.Vec3f(0, 0, 1), gPhysXInterface)
            elif (attr == TEST_TIRE_MIN_SLIP_DENOM_LONG_ACTIVE):
                vehContrAPI.GetAcceleratorAttr().Set(1.0)
            elif (attr == TEST_TIRE_MIN_SLIP_DENOM_LAT):
                # setup vehicle to slide laterally with certain velocity
                rigidBodyAPI.GetVelocityAttr().Set(Gf.Vec3f(startSpeed, 0, 0))

        if (attr == TEST_TIRE_MIN_SLIP_DENOM_LONG_PASSIVE):
            vehicleAPI[1].GetMinPassiveLongitudinalSlipDenominatorAttr().Set(100000)
        elif (attr == TEST_TIRE_MIN_SLIP_DENOM_LONG_ACTIVE):
            vehicleAPI[1].GetMinActiveLongitudinalSlipDenominatorAttr().Set(100000)
        elif (attr == TEST_TIRE_MIN_SLIP_DENOM_LAT):
            vehicleAPI[1].GetMinLateralSlipDenominatorAttr().Set(100000)

        self._simulate(1)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        vehicleEndPos = []
        for i in range(vehicleCount):
            p = stage.GetPrimAtPath(vehiclePaths[i])
            vehicleEndPos.append(p.GetAttribute("xformOp:translate").Get())

        delta0 = vehicleEndPos[0] - vehicleStartPos[0]
        delta1 = vehicleEndPos[1] - vehicleStartPos[1]

        errEps = 0.001

        if (attr == TEST_TIRE_MIN_SLIP_DENOM_LONG_PASSIVE):
            # with an artificially large min slip denominator, the slip will be small and the
            # longitudinal vehicle velocity will decrease slower, thus the vehicle will move further
            self.assertLess(delta0[2], (0.9 * delta1[2]))
            self.assertLess(math.fabs(delta1[0]), errEps)  # extra check to distinguish from uncontrolled explosion
        elif (attr == TEST_TIRE_MIN_SLIP_DENOM_LONG_ACTIVE):
            # with an artificially large min slip denominator, the slip will be small and an accelerating
            # vehicle should move forward a lot less
            self.assertGreater(delta0[2], (2 * delta1[2]))
            self.assertLess(math.fabs(delta1[0]), errEps)  # extra check to distinguish from uncontrolled explosion
        elif (attr == TEST_TIRE_MIN_SLIP_DENOM_LAT):
            # with an artificially large min slip denominator, the slip will be small and the
            # lateral vehicle velocity will decrease slower, thus the vehicle will move further
            self.assertLess(delta0[0], (0.7 * delta1[0]))
            self.assertLess(math.fabs(delta1[2]), errEps)  # extra check to distinguish from uncontrolled explosion

    #
    # modifying the minPassiveLongitudinalSlipDenominator parameter should affect the
    # simulation accordingly.
    #
    async def test_tire_min_slip_denom_long_passive(self):
        await self._tire_min_slip_denom_scenario(TEST_TIRE_MIN_SLIP_DENOM_LONG_PASSIVE)

    #
    # modifying the minActiveLongitudinalSlipDenominator parameter should affect the
    # simulation accordingly.
    #
    async def test_tire_min_slip_denom_long_active(self):
        await self._tire_min_slip_denom_scenario(TEST_TIRE_MIN_SLIP_DENOM_LONG_ACTIVE)

    #
    # modifying the minLateralSlipDenominator parameter should affect the
    # simulation accordingly.
    #
    async def test_tire_min_slip_denom_lat(self):
        await self._tire_min_slip_denom_scenario(TEST_TIRE_MIN_SLIP_DENOM_LAT)

    async def _wheel_scenario(self, attr, driveModeIn = Factory.DRIVE_BASIC):
        stage = await self.new_stage()

        if (attr == TEST_WHEEL_ATTR_TOE_ANGLE):
            useDepAPIList = [True, True]
        else:
            useDepAPIList = None

        vehicleCount = 2
        vehiclePaths = []
        wheelAttachmentPaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=driveModeIn,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
            vehicleDelta=[-10, 0, 0],
            useDeprecatedAPIList=useDepAPIList
        )

        vehicleStartPos = []
        for i in range(vehicleCount):
            p = stage.GetPrimAtPath(vehiclePaths[i])
            vehicleStartPos.append(p.GetAttribute("xformOp:translate").Get())

        wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][Factory.WHEEL_FRONT_LEFT])
        wheel1 = self._get_wheel(wheelAttPrim, stage)

        wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][Factory.WHEEL_REAR_RIGHT])
        wheel2 = self._get_wheel(wheelAttPrim, stage)

        # change factory result such that vehicle 1 uses wheel 1 and vehicle 2 uses
        # wheel 2
        for i in range(2):
            for j in range(len(wheelAttachmentPaths[i])):
                wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[i][j])
                wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(wheelAttPrim)
                wheelAttAPI.GetWheelRel().ClearTargets(True)
                if (i == 0):
                    wheelAttAPI.GetWheelRel().AddTarget(wheel1.GetPath())
                else:
                    wheelAttAPI.GetWheelRel().AddTarget(wheel2.GetPath())

        if (attr == TEST_WHEEL_ATTR_RADIUS):
            wheel1Radius = wheel1.GetRadiusAttr().Get()
            wheel2Radius = wheel2.GetRadiusAttr().Get()

            self.assertEqual(wheel1Radius, wheel2Radius)
        elif (attr == TEST_WHEEL_ATTR_MOI):
            startSpeed = 10

            wheel1Moi = wheel1.GetMoiAttr().Get()
            wheel2Moi = wheel2.GetMoiAttr().Get()

            self.assertEqual(wheel1Moi, wheel2Moi)
        elif (attr == TEST_WHEEL_ATTR_DAMPING_RATE):
            startSpeed = 2

            wheel1DampingRate = wheel1.GetDampingRateAttr().Get()
            wheel2DampingRate = wheel2.GetDampingRateAttr().Get()

            self.assertEqual(wheel1DampingRate, wheel2DampingRate)
        elif (attr == TEST_WHEEL_ATTR_TOE_ANGLE):
            wheel1ToeAngle = wheel1.GetToeAngleAttr().Get()
            wheel2ToeAngle = wheel2.GetToeAngleAttr().Get()

            self.assertEqual(wheel1ToeAngle, wheel2ToeAngle)
        else:
            self.assertTrue(False)

        self._simulate_one_frame_with_prep()

        global gPhysXInterface

        for i in range(vehicleCount):
            p = stage.GetPrimAtPath(vehiclePaths[i])
            vehContrAPI = PhysxSchema.PhysxVehicleControllerAPI(p)

            if ((attr == TEST_WHEEL_ATTR_MOI) or
                (attr == TEST_WHEEL_ATTR_DAMPING_RATE)):

                # setup vehicle to drive with certain velocity
                self._set_vehicle_speed(stage, p, wheelAttachmentPaths[i], startSpeed,
                    Gf.Vec3f(0, 0, 1), gPhysXInterface)

                if (driveModeIn == Factory.DRIVE_STANDARD):
                    vehContrAPI = PhysxSchema.PhysxVehicleControllerAPI(p)
                    if (startSpeed <= 2):
                        vehContrAPI.GetTargetGearAttr().Set(1)
                    elif (startSpeed <= 10):
                        vehContrAPI.GetTargetGearAttr().Set(2)
                    else:
                        vehContrAPI.GetTargetGearAttr().Set(3)

                if (attr == TEST_WHEEL_ATTR_MOI):
                    vehContrAPI.GetBrake0Attr().Set(1.0)

        if (attr == TEST_WHEEL_ATTR_RADIUS):
            wheel2RadiusExtra = wheel2Radius * 0.05
            wheel2Radius = wheel2Radius + wheel2RadiusExtra
            wheel2.GetRadiusAttr().Set(wheel2Radius)
        elif (attr == TEST_WHEEL_ATTR_MOI):
            wheel2Moi = 100.0 * wheel2Moi
            wheel2.GetMoiAttr().Set(wheel2Moi)
        elif (attr == TEST_WHEEL_ATTR_DAMPING_RATE):
            wheel2DampingRate = 100.0 * wheel2DampingRate
            wheel2.GetDampingRateAttr().Set(wheel2DampingRate)
        elif (attr == TEST_WHEEL_ATTR_TOE_ANGLE):
            wheel2ToeAngle = (10 * math.pi) / 180
            wheel2.GetToeAngleAttr().Set(wheel2ToeAngle)

        self._simulate(1)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        vehicleEndPos = []
        for i in range(vehicleCount):
            p = stage.GetPrimAtPath(vehiclePaths[i])
            vehicleEndPos.append(p.GetAttribute("xformOp:translate").Get())

        delta0 = vehicleEndPos[0] - vehicleStartPos[0]
        delta1 = vehicleEndPos[1] - vehicleStartPos[1]

        oneDegree = math.pi / 180

        if (attr == TEST_WHEEL_ATTR_RADIUS):
            self.assertLess((delta0[1] + (0.9 * wheel2RadiusExtra)), delta1[1])
        elif (attr == TEST_WHEEL_ATTR_MOI):
            # larger moment of inertia results in smaller angular acceleration when braking, thus
            # it takes longer to slow down
            self.assertLess(delta0[2], (0.9 * delta1[2]))
        elif (attr == TEST_WHEEL_ATTR_DAMPING_RATE):
            # higher damping should result in the vehicle slowing down faster
            self.assertGreater(delta0[2], (1.1 * delta1[2]))
        elif (attr == TEST_WHEEL_ATTR_TOE_ANGLE):
            restDir = Gf.Vec3f(0, 0, 1)

            wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[1][Factory.WHEEL_FRONT_LEFT])
            xformable = UsdGeom.Xformable(wheelAttPrim)
            wheel2Dir = self._local_dir_to_world(restDir, xformable)
            deltaAngle2 = self._get_delta_angle_radians(restDir, wheel2Dir)

            self.assertLess(math.fabs(deltaAngle2 - wheel2ToeAngle), oneDegree)

    #
    # modifying the wheel radius parameter should affect the simulation accordingly.
    #
    async def test_wheel_radius(self):
        await self._wheel_scenario(TEST_WHEEL_ATTR_RADIUS)

    #
    # modifying the wheel width parameter should affect the simulation accordingly.
    #
    #async def test_wheel_width(self):
    #    await self._wheel_scenario(TEST_WHEEL_ATTR_WIDTH)
    #
    # has no simulation effect in PhysX, only a collision effect if sweeps are used

    #
    # modifying the wheel mass parameter should affect the simulation accordingly.
    #
    #async def test_wheel_mass(self):
    #    await self._wheel_scenario(TEST_WHEEL_ATTR_MASS)
    #
    # changing the wheel mass has no clear enough effect in PhysX that could be compared, as
    # the suspension model is a single mass model and the rest load is not really used in the
    # tire force computation (or rather, it largely cancels out).

    #
    # modifying the wheel moment of inertia parameter should affect the simulation accordingly.
    #
    async def test_wheel_moment_of_inertia(self):
        await self._wheel_scenario(TEST_WHEEL_ATTR_MOI)

    #
    # modifying the wheel damping rate parameter should affect the simulation accordingly.
    #
    async def test_wheel_damping_rate(self):
        await self._wheel_scenario(TEST_WHEEL_ATTR_DAMPING_RATE)

    #
    # modifying the wheel toe angle parameter should affect the simulation accordingly.
    #
    async def test_wheel_toe_angle(self):
        await self._wheel_scenario(TEST_WHEEL_ATTR_TOE_ANGLE, Factory.DRIVE_STANDARD)

    async def _wheel_attachment_scenario(self, attr, driveModeIn = Factory.DRIVE_BASIC):
        stage = await self.new_stage()

        if ((attr == TEST_WHEEL_ATTACHMENT_ATTR_SUSP_FORCE_APP_POINT_OFFSET) or
            (attr == TEST_WHEEL_ATTACHMENT_ATTR_WHEEL_CENTER_OFFSET) or
            (attr == TEST_WHEEL_ATTACHMENT_ATTR_TIRE_FORCE_APP_POINT_OFFSET)):
            useDepAPIList = [True, True]
        else:
            useDepAPIList = None

        vehicleCount = 2
        vehiclePaths = []
        wheelAttachmentPaths = []
        collisionGroupPaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=driveModeIn,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
            vehicleDelta=[-6, 0, 0],
            collisionGroupPathsOut=collisionGroupPaths,
            useDeprecatedAPIList=useDepAPIList
        )

        vehicleStartPos = []
        for i in range(vehicleCount):
            p = stage.GetPrimAtPath(vehiclePaths[i])
            vehicleStartPos.append(p.GetAttribute("xformOp:translate").Get())

            if (attr == TEST_WHEEL_ATTACHMENT_ATTR_TIRE_FORCE_APP_POINT_OFFSET):
                vehContrAPI = PhysxSchema.PhysxVehicleControllerAPI(p)
                vehContrAPI.GetAcceleratorAttr().Set(1)
                if (driveModeIn == Factory.DRIVE_STANDARD):
                    vehContrAPI.GetTargetGearAttr().Set(1)

        wheelAttAPI = []
        for i in range(len(wheelAttachmentPaths[1])):
            wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[1][i])
            wheelAttAPITmp = PhysxSchema.PhysxVehicleWheelAttachmentAPI(wheelAttPrim)
            wheelAttAPI.append(wheelAttAPITmp)

        if (attr == TEST_WHEEL_ATTACHMENT_ATTR_SUSP_TRAVEL_DIR):
            suspTravelDir = Gf.Vec3f(0.0, -1.0, 2.0)
            suspTravelDir.Normalize()
        elif (attr == TEST_WHEEL_ATTACHMENT_ATTR_SUSP_FRAME_POSITION):
            suspFramePosFL = wheelAttAPI[Factory.WHEEL_FRONT_LEFT].GetSuspensionFramePositionAttr().Get()
            suspFramePosRL = wheelAttAPI[Factory.WHEEL_REAR_LEFT].GetSuspensionFramePositionAttr().Get()
            wheelDist = suspFramePosFL[2] - suspFramePosRL[2]
            newWheelForwardAxis = (0.1 * wheelDist)
        elif (attr == TEST_WHEEL_ATTACHMENT_ATTR_SUSP_FORCE_APP_POINT_OFFSET):
            suspForceAppPointFL = wheelAttAPI[Factory.WHEEL_FRONT_LEFT].GetSuspensionForceAppPointOffsetAttr().Get()
            suspForceAppPointRL = wheelAttAPI[Factory.WHEEL_REAR_LEFT].GetSuspensionForceAppPointOffsetAttr().Get()
            dist = suspForceAppPointFL[2] - suspForceAppPointRL[2]
            newSuspForceAppPointForwardAxis = (0.4 * dist) + suspForceAppPointRL[2]
        elif (attr == TEST_WHEEL_ATTACHMENT_ATTR_WHEEL_CENTER_OFFSET):
            wheelCoMOffsetFL = wheelAttAPI[Factory.WHEEL_FRONT_LEFT].GetWheelCenterOfMassOffsetAttr().Get()
            wheelCoMOffsetRL = wheelAttAPI[Factory.WHEEL_REAR_LEFT].GetWheelCenterOfMassOffsetAttr().Get()
            wheelDist = wheelCoMOffsetFL[2] - wheelCoMOffsetRL[2]
            newWheelForwardAxis = (0.1 * wheelDist)
        elif (attr == TEST_WHEEL_ATTACHMENT_ATTR_TIRE_FORCE_APP_POINT_OFFSET):
            tireForceAppPtFL = wheelAttAPI[Factory.WHEEL_FRONT_LEFT].GetTireForceAppPointOffsetAttr().Get()
            tireForceAppPtFL[1] = tireForceAppPtFL[1] * 1000
            tireForceAppPtFR = wheelAttAPI[Factory.WHEEL_FRONT_RIGHT].GetTireForceAppPointOffsetAttr().Get()
            tireForceAppPtFR[1] = tireForceAppPtFR[1] * 1000
        elif (attr == TEST_WHEEL_ATTACHMENT_ATTR_COLLISON_GROUP):
            collisionGroupVehicleGroundQueryNewPath = "/CollisionGroupNew"
            collisionGroupVehicleGroundQueryNew = UsdPhysics.CollisionGroup.Define(stage, collisionGroupVehicleGroundQueryNewPath)
            collisionGroupVehicleGroundQueryRel = collisionGroupVehicleGroundQueryNew.CreateFilteredGroupsRel()
            collisionGroupVehicleGroundQueryRel.AddTarget(collisionGroupPaths[Factory.COLL_GROUP_VEHICLE_CHASSIS])
            collisionGroupVehicleGroundQueryRel.AddTarget(collisionGroupPaths[Factory.COLL_GROUP_VEHICLE_WHEEL])
            collisionGroupVehicleGroundQueryRel.AddTarget(collisionGroupPaths[Factory.COLL_GROUP_GROUND_SURFACE])

        self._simulate_one_frame_with_prep()

        if (attr == TEST_WHEEL_ATTACHMENT_ATTR_SUSP_TRAVEL_DIR):
            wheelAttAPI[Factory.WHEEL_FRONT_LEFT].GetSuspensionTravelDirectionAttr().Set(suspTravelDir)
            wheelAttAPI[Factory.WHEEL_FRONT_RIGHT].GetSuspensionTravelDirectionAttr().Set(suspTravelDir)
        elif (attr == TEST_WHEEL_ATTACHMENT_ATTR_SUSP_FRAME_POSITION):
            suspFramePosFL[2] = newWheelForwardAxis

            suspFramePosFR = wheelAttAPI[Factory.WHEEL_FRONT_RIGHT].GetSuspensionFramePositionAttr().Get()
            suspFramePosFR[2] = newWheelForwardAxis

            wheelAttAPI[Factory.WHEEL_FRONT_LEFT].GetSuspensionFramePositionAttr().Set(suspFramePosFL)
            wheelAttAPI[Factory.WHEEL_FRONT_RIGHT].GetSuspensionFramePositionAttr().Set(suspFramePosFR)
        elif (attr == TEST_WHEEL_ATTACHMENT_ATTR_SUSP_FORCE_APP_POINT_OFFSET):
            suspForceAppPtOffFL = wheelAttAPI[Factory.WHEEL_FRONT_LEFT].GetSuspensionForceAppPointOffsetAttr().Get()
            suspForceAppPtOffFL[2] = newSuspForceAppPointForwardAxis
            suspForceAppPtOffFR = wheelAttAPI[Factory.WHEEL_FRONT_RIGHT].GetSuspensionForceAppPointOffsetAttr().Get()
            suspForceAppPtOffFR[2] = newSuspForceAppPointForwardAxis

            wheelAttAPI[Factory.WHEEL_FRONT_LEFT].GetSuspensionForceAppPointOffsetAttr().Set(suspForceAppPtOffFL)
            wheelAttAPI[Factory.WHEEL_FRONT_RIGHT].GetSuspensionForceAppPointOffsetAttr().Set(suspForceAppPtOffFR)
        elif (attr == TEST_WHEEL_ATTACHMENT_ATTR_WHEEL_CENTER_OFFSET):
            wheelCoMOffsetFL[2] = newWheelForwardAxis

            wheelCoMOffsetFR = wheelAttAPI[Factory.WHEEL_FRONT_RIGHT].GetWheelCenterOfMassOffsetAttr().Get()
            wheelCoMOffsetFR[2] = newWheelForwardAxis

            wheelAttAPI[Factory.WHEEL_FRONT_LEFT].GetWheelCenterOfMassOffsetAttr().Set(wheelCoMOffsetFL)
            wheelAttAPI[Factory.WHEEL_FRONT_RIGHT].GetWheelCenterOfMassOffsetAttr().Set(wheelCoMOffsetFR)
        elif (attr == TEST_WHEEL_ATTACHMENT_ATTR_TIRE_FORCE_APP_POINT_OFFSET):
            wheelAttAPI[Factory.WHEEL_FRONT_LEFT].GetTireForceAppPointOffsetAttr().Set(tireForceAppPtFL)
            wheelAttAPI[Factory.WHEEL_FRONT_RIGHT].GetTireForceAppPointOffsetAttr().Set(tireForceAppPtFR)
        elif (attr == TEST_WHEEL_ATTACHMENT_ATTR_COLLISON_GROUP):
            for i in range(len(wheelAttAPI)):
                wheelAttAPI[i].GetCollisionGroupRel().ClearTargets(True)
                wheelAttAPI[i].GetCollisionGroupRel().AddTarget(collisionGroupVehicleGroundQueryNewPath)

        if (attr == TEST_WHEEL_ATTACHMENT_ATTR_TIRE_FORCE_APP_POINT_OFFSET):
            self._simulate(0.25)
        else:
            self._simulate(1)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        vehicleEndPos = []
        for i in range(vehicleCount):
            p = stage.GetPrimAtPath(vehiclePaths[i])
            vehicleEndPos.append(p.GetAttribute("xformOp:translate").Get())

        delta0 = vehicleEndPos[0] - vehicleStartPos[0]
        delta1 = vehicleEndPos[1] - vehicleStartPos[1]

        if (attr == TEST_WHEEL_ATTACHMENT_ATTR_SUSP_TRAVEL_DIR):
            # the chassis should tip to the ground at the front as the suspension travel direction
            # was rotated from vertical to near horizontal
            self.assertLess(math.fabs(delta0[1]), 0.001)
            self.assertLess(delta1[1], -0.05)
        elif (attr == TEST_WHEEL_ATTACHMENT_ATTR_SUSP_FRAME_POSITION):
            # the chassis should drop at the front as the front wheels are moved close to the center
            # of mass and thus carry more of the total mass
            self.assertLess(math.fabs(delta0[1]), 0.001)
            self.assertLess(delta1[1], -0.04)
            self.assertLess(math.fabs(delta1[0]), 0.001)  # extra check to distinguish from uncontrolled explosion
        elif (attr == TEST_WHEEL_ATTACHMENT_ATTR_SUSP_FORCE_APP_POINT_OFFSET):
            # the chassis should get lifted at the rear as force position offsets are moved
            # to the back
            self.assertLess(math.fabs(delta0[1]), 0.001)
            self.assertGreater(delta1[1], 0.2)
            self.assertLess(math.fabs(delta1[0]), 0.002)  # extra check to distinguish from uncontrolled explosion
        elif (attr == TEST_WHEEL_ATTACHMENT_ATTR_WHEEL_CENTER_OFFSET):
            # the chassis should drop at the front as the front wheels are moved close to the center
            # of mass and thus carry more of the total mass
            self.assertLess(math.fabs(delta0[1]), 0.001)
            self.assertLess(delta1[1], -0.04)
            self.assertLess(math.fabs(delta1[0]), 0.001)  # extra check to distinguish from uncontrolled explosion
        elif (attr == TEST_WHEEL_ATTACHMENT_ATTR_TIRE_FORCE_APP_POINT_OFFSET):
            # moving tire force app point far away below the vehicle should
            # cause a rotation around the side axis and lift the vehicle
            self.assertLess(math.fabs(delta0[1]), 0.005)
            self.assertGreater(delta1[1], 0.1)
        elif (attr == TEST_WHEEL_ATTACHMENT_ATTR_COLLISON_GROUP):
            # new collision group disabled query hits, thus the chassis should hit the ground
            self.assertLess(math.fabs(delta0[1]), 0.001)
            self.assertLess(delta1[1], -0.2)

    #
    # modifying the wheel attachment suspension travel direction parameter should affect
    # the simulation accordingly.
    #
    async def test_wheel_attachment_susp_travel_dir(self):
        await self._wheel_attachment_scenario(TEST_WHEEL_ATTACHMENT_ATTR_SUSP_TRAVEL_DIR, Factory.DRIVE_BASIC)

    #
    # modifying the wheel attachment suspension frame position should affect the simulation accordingly.
    #
    async def test_wheel_attachment_suspension_frame_position(self):
        await self._wheel_attachment_scenario(TEST_WHEEL_ATTACHMENT_ATTR_SUSP_FRAME_POSITION,
            Factory.DRIVE_STANDARD)

    #
    # modifying the wheel attachment suspension force application point offset should
    # affect the simulation accordingly.
    #
    async def test_wheel_attachment_susp_force_app_point_offset(self):
        await self._wheel_attachment_scenario(TEST_WHEEL_ATTACHMENT_ATTR_SUSP_FORCE_APP_POINT_OFFSET,
            Factory.DRIVE_STANDARD)

    #
    # modifying the wheel attachment wheel center of mass offset should affect the simulation accordingly.
    #
    async def test_wheel_attachment_wheel_com_offset(self):
        await self._wheel_attachment_scenario(TEST_WHEEL_ATTACHMENT_ATTR_WHEEL_CENTER_OFFSET,
            Factory.DRIVE_STANDARD)

    #
    # modifying the wheel attachment tire force application point offset should affect the simulation accordingly.
    #
    async def test_wheel_attachment_tire_force_app_point_offset(self):
        await self._wheel_attachment_scenario(TEST_WHEEL_ATTACHMENT_ATTR_TIRE_FORCE_APP_POINT_OFFSET,
            Factory.DRIVE_BASIC)

    #
    # modifying the wheel attachment collision group should affect the simulation accordingly.
    #
    async def test_wheel_attachment_collision_group(self):
        await self._wheel_attachment_scenario(TEST_WHEEL_ATTACHMENT_ATTR_COLLISON_GROUP,
            Factory.DRIVE_BASIC)

    #
    # modifying the wheel attachment wheel frame transform should affect the simulation accordingly.
    #
    async def test_wheel_attachment_wheel_frame(self):
        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths
        )

        wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][Factory.WHEEL_FRONT_LEFT])
        wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(wheelAttPrim)

        wheelXformable = UsdGeom.Xformable(wheelAttPrim)
        wheelGlobalPose = wheelXformable.ComputeLocalToWorldTransform(0)
        flPosStart = Gf.Vec3f(wheelGlobalPose.ExtractTranslation())

        deltaX = 0.2
        flPosDelta = Gf.Vec3f(deltaX, 0, 0)

        rotAngle = math.pi / 6.0
        rotAngleHalf = 0.5 * rotAngle
        flOrientDelta = Gf.Quatf(math.cos(rotAngleHalf), 0.0, math.sin(rotAngleHalf), 0.0)

        wheelAttAPI.GetWheelFramePositionAttr().Set(flPosDelta)
        wheelAttAPI.GetWheelFrameOrientationAttr().Set(flOrientDelta)

        self._simulate_one_frame_with_prep()

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        errEps = 0.0001

        wheelGlobalPose = wheelXformable.ComputeLocalToWorldTransform(0)
        flPos = Gf.Vec3f(wheelGlobalPose.ExtractTranslation())
        delta = flPos - flPosStart
        self.assertAlmostEqual(delta[0], deltaX, delta=errEps)
        self.assertLess(math.fabs(delta[1]), errEps)
        self.assertLess(math.fabs(delta[2]), errEps)

        forwardDirRef = Gf.Vec3f(0, 0, 1)
        forwardDir = self._local_dir_to_world(forwardDirRef, wheelXformable)
        deltaAngle = self._get_delta_angle_radians(forwardDirRef, forwardDir)
        self.assertAlmostEqual(deltaAngle, rotAngle, delta=rotAngle * 0.001)

        # change properties while sim is running

        deltaX = -0.1
        flPosDelta = Gf.Vec3f(deltaX, 0, 0)

        rotAngle = math.pi / 12.0
        rotAngleHalf = 0.5 * rotAngle
        flOrientDelta = Gf.Quatf(math.cos(rotAngleHalf), 0.0, math.sin(rotAngleHalf), 0.0)

        wheelAttAPI.GetWheelFramePositionAttr().Set(flPosDelta)
        wheelAttAPI.GetWheelFrameOrientationAttr().Set(flOrientDelta)

        self._simulate_one_frame()

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        wheelGlobalPose = wheelXformable.ComputeLocalToWorldTransform(0)
        flPos = Gf.Vec3f(wheelGlobalPose.ExtractTranslation())
        delta = flPos - flPosStart
        self.assertAlmostEqual(delta[0], deltaX, delta=errEps)
        self.assertLess(math.fabs(delta[1]), errEps)
        self.assertLess(math.fabs(delta[2]), errEps)

        forwardDirRef = Gf.Vec3f(0, 0, 1)
        forwardDir = self._local_dir_to_world(forwardDirRef, wheelXformable)
        deltaAngle = self._get_delta_angle_radians(forwardDirRef, forwardDir)
        self.assertAlmostEqual(deltaAngle, rotAngle, delta=rotAngle * 0.001)

    async def _suspension_compliance_angle_scenario(self, attr):
        stage = await self.new_stage()

        vehicleCount = 2
        vehiclePaths = []
        wheelAttachmentPaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths
        )

        for i in range(vehicleCount):
            p = stage.GetPrimAtPath(vehiclePaths[i])
            posAttr = p.GetAttribute("xformOp:translate")
            pos = posAttr.Get()
            if (i == 0):
                # make sure first vehicle is high up in air such that jounce will be 0
                pos[1] = pos[1] + 1000
                posAttr.Set(pos)

        angleAbs = (30 * math.pi) / 180

        for i in range(vehicleCount):
            for path in wheelAttachmentPaths[i]:
                wheelAttPrim = stage.GetPrimAtPath(path)
                suspensionComplianceAPI = PhysxSchema.PhysxVehicleSuspensionComplianceAPI(wheelAttPrim)

                if (attr == TEST_SUSPENSION_COMPLIANCE_ATTR_TOE_ANGLE):
                    suspensionComplianceAPI.CreateWheelToeAngleAttr([])
                elif (attr == TEST_SUSPENSION_COMPLIANCE_ATTR_CAMBER_ANGLE):
                    suspensionComplianceAPI.CreateWheelCamberAngleAttr([])

        self._prepare_for_simulation()
        self._simulate_one_frame()
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        if (attr == TEST_SUSPENSION_COMPLIANCE_ATTR_TOE_ANGLE):
            refDirLocal = Gf.Vec3f(0, 0, 1)  # forward
        elif (attr == TEST_SUSPENSION_COMPLIANCE_ATTR_CAMBER_ANGLE):
            refDirLocal = Gf.Vec3f(0, 1, 0)  # up

        wheelAttXforms = []
        for i in range(vehicleCount):
            wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[i][0])
            xForm = UsdGeom.Xformable(wheelAttPrim)
            dir = self._local_dir_to_world(refDirLocal, xForm)
            wheelAttXforms.append(xForm)

            # with empty array, toe/camber is expected to be 0
            deltaLength = (dir - refDirLocal).GetLength()
            self.assertLess(deltaLength, 0.001)

        for i in range(vehicleCount):
            for path in wheelAttachmentPaths[i]:
                wheelAttPrim = stage.GetPrimAtPath(path)
                suspensionComplianceAPI = PhysxSchema.PhysxVehicleSuspensionComplianceAPI(wheelAttPrim)

                if (attr == TEST_SUSPENSION_COMPLIANCE_ATTR_TOE_ANGLE):
                    suspensionComplianceAPI.GetWheelToeAngleAttr().Set([Gf.Vec2f(0.0, angleAbs), Gf.Vec2f(1.0, -angleAbs)])
                elif (attr == TEST_SUSPENSION_COMPLIANCE_ATTR_CAMBER_ANGLE):
                    suspensionComplianceAPI.GetWheelCamberAngleAttr().Set([Gf.Vec2f(0.0, -angleAbs), Gf.Vec2f(1.0, angleAbs)])

        # push second vehicle down to get max jounce
        prim = stage.GetPrimAtPath(vehiclePaths[1])
        posAttr = p.GetAttribute("xformOp:translate")
        pos = posAttr.Get()
        pos[1] = pos[1] - 0.11  # value chosen to make sure full compression is reached
        posAttr.Set(pos)

        self._simulate_one_frame()
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        # first vehicle is in air and should have 0 jounce -> expect corresponding angle
        dir = self._local_dir_to_world(refDirLocal, wheelAttXforms[0])
        deltaAngle = self._get_delta_angle_radians(refDirLocal, dir)
        oneDegreeInRadians = (1 * math.pi) / 180
        self.assertAlmostEqual(math.fabs(deltaAngle), angleAbs, delta=oneDegreeInRadians)
        self.assertGreater(dir[0], 0.01)  # make sure rotation happened in correct direction

        # second vehicle should have max jounce -> expect corresponding angle
        dir = self._local_dir_to_world(refDirLocal, wheelAttXforms[1])
        deltaAngle = self._get_delta_angle_radians(refDirLocal, dir)
        self.assertAlmostEqual(math.fabs(deltaAngle), angleAbs, delta=oneDegreeInRadians)
        self.assertLess(dir[0], -0.01)  # make sure rotation happened in correct direction

    #
    # modifying the suspension compliance toe angle parameter should affect the simulation accordingly.
    #
    async def test_suspension_compliance_toe_angle(self):
        await self._suspension_compliance_angle_scenario(TEST_SUSPENSION_COMPLIANCE_ATTR_TOE_ANGLE)

    #
    # modifying the suspension compliance camber angle parameter should affect the simulation accordingly.
    #
    async def test_suspension_compliance_camber_angle(self):
        await self._suspension_compliance_angle_scenario(TEST_SUSPENSION_COMPLIANCE_ATTR_CAMBER_ANGLE)

    async def _suspension_compliance_point_scenario(self, attr):
        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths
        )

        p = stage.GetPrimAtPath(vehiclePaths[0])
        vehicleStartPos = p.GetAttribute("xformOp:translate").Get()

        if (attr == TEST_SUSPENSION_COMPLIANCE_ATTR_TIRE_FORCE_APP_POINT):
            vehContrAPI = PhysxSchema.PhysxVehicleControllerAPI(p)
            vehContrAPI.GetAcceleratorAttr().Set(1)

        wheelAttAPI = []
        suspComplAPI = []
        for path in wheelAttachmentPaths[0]:
            wheelAttPrim = stage.GetPrimAtPath(path)
            wheelAttAPI.append(PhysxSchema.PhysxVehicleWheelAttachmentAPI(wheelAttPrim))
            suspComplAPI.append(PhysxSchema.PhysxVehicleSuspensionComplianceAPI(wheelAttPrim))

        if (attr == TEST_SUSPENSION_COMPLIANCE_ATTR_SUSP_FORCE_APP_POINT):
            suspFramePosFL = wheelAttAPI[Factory.WHEEL_FRONT_LEFT].GetSuspensionFramePositionAttr().Get()
            suspFramePosRL = wheelAttAPI[Factory.WHEEL_REAR_LEFT].GetSuspensionFramePositionAttr().Get()
            dist = suspFramePosFL[2] - suspFramePosRL[2]
            newSuspForceAppPointForwardAxis = (-0.6 * dist)
        elif (attr == TEST_SUSPENSION_COMPLIANCE_ATTR_TIRE_FORCE_APP_POINT):
            tireForceAppPtFL = suspComplAPI[Factory.WHEEL_FRONT_LEFT].GetTireForceAppPointAttr().Get()[0]
            # index 2 (up axis in this case) since the first entry is the normalized jounce
            tireForceAppPtFL[2] = tireForceAppPtFL[2] * 1000
            tireForceAppPtFR = suspComplAPI[Factory.WHEEL_FRONT_RIGHT].GetTireForceAppPointAttr().Get()[0]
            tireForceAppPtFR[2] = tireForceAppPtFR[2] * 1000

        self._simulate_one_frame_with_prep()

        if (attr == TEST_SUSPENSION_COMPLIANCE_ATTR_SUSP_FORCE_APP_POINT):
            suspForceAppPtFL = suspComplAPI[Factory.WHEEL_FRONT_LEFT].GetSuspensionForceAppPointAttr().Get()[0]
            # index 3 (forward axis in this case) since the first entry is the normalized jounce
            suspForceAppPtFL[3] = newSuspForceAppPointForwardAxis
            suspForceAppPtFR = suspComplAPI[Factory.WHEEL_FRONT_RIGHT].GetSuspensionForceAppPointAttr().Get()[0]
            suspForceAppPtFR[3] = newSuspForceAppPointForwardAxis

            suspComplAPI[Factory.WHEEL_FRONT_LEFT].GetSuspensionForceAppPointAttr().Set([suspForceAppPtFL])
            suspComplAPI[Factory.WHEEL_FRONT_RIGHT].GetSuspensionForceAppPointAttr().Set([suspForceAppPtFR])
        elif (attr == TEST_SUSPENSION_COMPLIANCE_ATTR_TIRE_FORCE_APP_POINT):
            suspComplAPI[Factory.WHEEL_FRONT_LEFT].GetTireForceAppPointAttr().Set([tireForceAppPtFL])
            suspComplAPI[Factory.WHEEL_FRONT_RIGHT].GetTireForceAppPointAttr().Set([tireForceAppPtFR])

        if (attr == TEST_SUSPENSION_COMPLIANCE_ATTR_TIRE_FORCE_APP_POINT):
            self._simulate(0.25)
        else:
            self._simulate(1)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        p = stage.GetPrimAtPath(vehiclePaths[0])
        vehicleEndPos = p.GetAttribute("xformOp:translate").Get()

        delta = vehicleEndPos - vehicleStartPos

        if (attr == TEST_SUSPENSION_COMPLIANCE_ATTR_SUSP_FORCE_APP_POINT):
            # the chassis should get lifted at the rear as force position offsets are moved
            # to the back
            self.assertGreater(delta[1], 0.2)
            self.assertLess(math.fabs(delta[0]), 0.002)  # extra check to distinguish from uncontrolled explosion
        elif (attr == TEST_SUSPENSION_COMPLIANCE_ATTR_TIRE_FORCE_APP_POINT):
            # moving tire force app point far away below the vehicle should
            # cause a rotation around the side axis and lift the vehicle
            self.assertGreater(delta[1], 0.1)

    #
    # modifying the suspension compliance suspension force application point should
    # affect the simulation accordingly.
    #
    async def test_suspension_compliance_susp_force_app_point_offset(self):
        await self._suspension_compliance_point_scenario(TEST_SUSPENSION_COMPLIANCE_ATTR_SUSP_FORCE_APP_POINT)

    #
    # modifying the suspension compliance tire force application point should
    # affect the simulation accordingly.
    #
    async def test_suspension_compliance_tire_force_app_point_offset(self):
        await self._suspension_compliance_point_scenario(TEST_SUSPENSION_COMPLIANCE_ATTR_TIRE_FORCE_APP_POINT)

    #
    # sprung mass computation method should return expected values.
    #
    async def test_compute_sprung_masses(self):
        global gPhysXVehicleInterface

        mass = 1600.0

        positions = []
        positions.append(carb.Float3(1.0, 0.0, 2.0))
        positions.append(carb.Float3(-1.0, 0.0, 2.0))
        positions.append(carb.Float3(1.0, 0.0, -2.0))
        positions.append(carb.Float3(-1.0, 0.0, -2.0))

        sprungMasses = gPhysXVehicleInterface.compute_sprung_masses(
            mass, 1, positions
        )

        for sm in sprungMasses:
            self.assertAlmostEqual(sm, 400.0, delta=0.01)

    #
    # test that vehicles simulate fine even if none of the vehicle extension attach or update methods are called.
    # Technically, this test should rather live in omni.physx.test but duplicating all the setup helpers seems
    # not great.
    #
    async def test_no_vehicle_extension_tick(self):
        for j in range(3):
            if (j == 0):
                drvMode = Factory.DRIVE_BASIC
            elif (j == 1):
                drvMode = Factory.DRIVE_STANDARD
            else:
                drvMode = Factory.DRIVE_NONE

            stage = await self.new_stage()

            vehicleCount = 1
            vehiclePaths = []
            wheelAttachmentPaths = []

            Factory.create4WheeledCarsScenario(
                stage,
                1.0,
                vehicleCount,
                createCollisionShapesForWheels=False,
                driveMode=drvMode,
                vehiclePathsOut=vehiclePaths,
                wheelAttachmentPathsOut=wheelAttachmentPaths,
            )

            vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])
            startPos = vehiclePrim.GetAttribute("xformOp:translate").Get()

            if (drvMode != Factory.DRIVE_NONE):
                vehicleController = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)
                vehicleController.GetAcceleratorAttr().Set(1.0)

                if (drvMode == Factory.DRIVE_STANDARD):
                    vehicleController.GetTargetGearAttr().Set(1)
            else:
                prim = stage.GetPrimAtPath(wheelAttachmentPaths[0][Factory.WHEEL_FRONT_LEFT])
                wheelController = PhysxSchema.PhysxVehicleWheelControllerAPI(prim)
                wheelController.GetDriveTorqueAttr().Set(300.0)

                prim = stage.GetPrimAtPath(wheelAttachmentPaths[0][Factory.WHEEL_FRONT_RIGHT])
                wheelController = PhysxSchema.PhysxVehicleWheelControllerAPI(prim)
                wheelController.GetDriveTorqueAttr().Set(300.0)

            secondsToRun = 1.0
            self.attach_stage()

            timeStep = self._get_time_step()
            targetIterationCount = math.ceil(secondsToRun / timeStep)
            self.step(targetIterationCount, timeStep)

            self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

            endPos = vehiclePrim.GetAttribute("xformOp:translate").Get()

            delta = endPos - startPos
            self.assertGreater(delta[2], 0.3)  # make sure vehicle moved at all
            self.assertAlmostEqual(delta[0], 0.0, delta=0.001)
            self.assertAlmostEqual(delta[1], 0.0, delta=0.001)

    #
    # test the compact vehicle USD representation where no components are shared and compare the simulation
    # result to the shared component scenario.
    #
    async def test_compact_vehicle_USD_representation(self):
        for j in range(3):
            if (j == 0):
                drvMode = Factory.DRIVE_BASIC
            elif (j == 1):
                drvMode = Factory.DRIVE_STANDARD
            else:
                drvMode = Factory.DRIVE_NONE

            stage = await self.new_stage()

            vehicleCount = 2
            vehiclePaths = []
            wheelAttachmentPaths = []
            useShareableComponentsListIn = [False, True]

            Factory.create4WheeledCarsScenario(
                stage,
                1.0,
                vehicleCount,
                createCollisionShapesForWheels=False,
                driveMode=drvMode,
                vehiclePathsOut=vehiclePaths,
                wheelAttachmentPathsOut=wheelAttachmentPaths,
                useShareableComponentsList=useShareableComponentsListIn,
            )

            vehiclePrimExclComp = stage.GetPrimAtPath(vehiclePaths[0])
            if (drvMode == Factory.DRIVE_BASIC):
                self.assertTrue(vehiclePrimExclComp.HasAPI(PhysxSchema.PhysxVehicleDriveBasicAPI))
            elif (drvMode == Factory.DRIVE_STANDARD):
                self.assertTrue(vehiclePrimExclComp.HasAPI(PhysxSchema.PhysxVehicleDriveStandardAPI))
                self.assertTrue(vehiclePrimExclComp.HasAPI(PhysxSchema.PhysxVehicleEngineAPI))
                self.assertTrue(vehiclePrimExclComp.HasAPI(PhysxSchema.PhysxVehicleGearsAPI))
                self.assertTrue(vehiclePrimExclComp.HasAPI(PhysxSchema.PhysxVehicleAutoGearBoxAPI))
                self.assertTrue(vehiclePrimExclComp.HasAPI(PhysxSchema.PhysxVehicleClutchAPI))

            wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][Factory.WHEEL_FRONT_LEFT])
            self.assertTrue(wheelAttPrim.HasAPI(PhysxSchema.PhysxVehicleWheelAPI))
            self.assertTrue(wheelAttPrim.HasAPI(PhysxSchema.PhysxVehicleTireAPI))
            self.assertTrue(wheelAttPrim.HasAPI(PhysxSchema.PhysxVehicleSuspensionAPI))

            startPos = []

            for i in range(vehicleCount):
                vehiclePrim = stage.GetPrimAtPath(vehiclePaths[i])
                startPos.append(vehiclePrim.GetAttribute("xformOp:translate").Get())

                if (drvMode != Factory.DRIVE_NONE):
                    vehicleController = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)
                    vehicleController.GetAcceleratorAttr().Set(1.0)

                    if (drvMode == Factory.DRIVE_STANDARD):
                        vehicleController.GetTargetGearAttr().Set(1)
                else:
                    prim = stage.GetPrimAtPath(wheelAttachmentPaths[i][Factory.WHEEL_FRONT_LEFT])
                    wheelController = PhysxSchema.PhysxVehicleWheelControllerAPI(prim)
                    wheelController.GetDriveTorqueAttr().Set(300.0)

                    prim = stage.GetPrimAtPath(wheelAttachmentPaths[i][Factory.WHEEL_FRONT_RIGHT])
                    wheelController = PhysxSchema.PhysxVehicleWheelControllerAPI(prim)
                    wheelController.GetDriveTorqueAttr().Set(300.0)

            secondsToRun = 1.0
            self._simulate_with_prep(secondsToRun)

            self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

            endPos = []

            for i in range(vehicleCount):
                vehiclePrim = stage.GetPrimAtPath(vehiclePaths[i])
                endPos.append(vehiclePrim.GetAttribute("xformOp:translate").Get())

            delta = endPos[0] - startPos[0]
            self.assertGreater(delta[2], 0.3)  # make sure vehicle moved at all
            self.assertAlmostEqual(delta[0], 0.0, delta=0.001)
            self.assertAlmostEqual(delta[1], 0.0, delta=0.001)

            delta = endPos[1] - endPos[0]
            self.assertAlmostEqual(delta[2], 0.0, delta=0.002)  # make sure both vehicles behaved the same

    #
    # Test that deleting a root prim of a vehicle while the simulation is running
    # does not crash.
    #
    async def test_dynamic_remove_root_prim(self):
        stage = await self.new_stage()

        rootPath = "/TestRoot"
        vehiclePath = rootPath + "/Car"
        vehiclePaths = [vehiclePath]

        vehicleCount = 1
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsIn=vehiclePaths
        )

        self._prepare_for_simulation()

        self._simulate_one_frame()

        stage.RemovePrim(rootPath)

        for i in range(3):
            self._simulate_one_frame()

    #
    # Test that user defined sprung mass values do not have to sum up to the
    # vehicle mass (and send no corresponding error message).
    #
    async def test_suspension_user_defined_sprung_mass(self):
        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_NONE,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths
        )

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])
        massAPI = UsdPhysics.MassAPI(vehiclePrim)
        mass = massAPI.GetMassAttr().Get()
        sprungMass = mass / 4.0

        wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][Factory.WHEEL_REAR_LEFT])
        susp = self._get_suspension(wheelAttPrim, stage)
        susp.GetSprungMassAttr().Set(sprungMass)
        wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(wheelAttPrim)
        wheelAttAPI.GetIndexAttr().Set(0)

        wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][Factory.WHEEL_REAR_RIGHT])
        susp = self._get_suspension(wheelAttPrim, stage)
        susp.GetSprungMassAttr().Set(sprungMass)
        wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(wheelAttPrim)
        wheelAttAPI.GetIndexAttr().Set(1)

        stage.RemovePrim(wheelAttachmentPaths[0][Factory.WHEEL_FRONT_LEFT])
        stage.RemovePrim(wheelAttachmentPaths[0][Factory.WHEEL_FRONT_RIGHT])

        self._simulate_with_prep(1.0)

        compressionThreshold = 0.11
        # the front wheels were removed, thus the vehicle front will fall to the ground
        # and there will be more force on the rear suspensions (equilibrium would have
        # been at jounce=0.1 ("deprecated" maxDroop value))

        global gPhysXInterface
        wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[0][Factory.WHEEL_REAR_LEFT])
        suspJounce = wheelState[VEHICLE_WHEEL_STATE_SUSPENSION_JOUNCE]
        self.assertGreater(suspJounce, compressionThreshold)

        wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[0][Factory.WHEEL_REAR_RIGHT])
        suspJounce = wheelState[VEHICLE_WHEEL_STATE_SUSPENSION_JOUNCE]
        self.assertGreater(suspJounce, compressionThreshold)

    #
    # verify that having UsdPhysics APIs (body, collison, material...) only is valid and works
    #
    async def test_usd_physics_api_only(self):

        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []
        tireFrictionTablePaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
            placeWheelCollisionShapesAtRoot=True,
            tireFrictionTablePathsOut=tireFrictionTablePaths
        )

        # remove all relevant Physx...APIs
        tireFrictionTablePrim = stage.GetPrimAtPath(tireFrictionTablePaths[Factory.TIRE_FRICTION_TABLE_WINTER_TIRE])
        tireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable(tireFrictionTablePrim)
        tarmacMaterialPath = tireFrictionTable.GetGroundMaterialsRel().GetTargets()[Factory.MATERIAL_TARMAC]
        gravelMaterialPath = tireFrictionTable.GetGroundMaterialsRel().GetTargets()[Factory.MATERIAL_GRAVEL]

        prim = stage.GetPrimAtPath(tarmacMaterialPath)
        self.assertTrue(prim.HasAPI(PhysxSchema.PhysxMaterialAPI))
        prim.RemoveAPI(PhysxSchema.PhysxMaterialAPI)

        prim = stage.GetPrimAtPath(gravelMaterialPath)
        self.assertTrue(prim.HasAPI(PhysxSchema.PhysxMaterialAPI))
        prim.RemoveAPI(PhysxSchema.PhysxMaterialAPI)

        for wheelPath in wheelAttachmentPaths[0]:
            prim = stage.GetPrimAtPath(wheelPath)
            self.assertTrue(prim.HasAPI(PhysxSchema.PhysxCollisionAPI))
            prim.RemoveAPI(PhysxSchema.PhysxCollisionAPI)

        prim = stage.GetPrimAtPath(vehiclePaths[0])
        self.assertTrue(prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI))
        prim.RemoveAPI(PhysxSchema.PhysxRigidBodyAPI)

        vehicleController = PhysxSchema.PhysxVehicleControllerAPI(prim)
        vehicleController.GetAcceleratorAttr().Set(1.0)

        vehicleStartPos = prim.GetAttribute("xformOp:translate").Get()

        secondsToRun = 1.0
        self._simulate_with_prep(secondsToRun)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        vehicleEndPos = prim.GetAttribute("xformOp:translate").Get()

        delta = vehicleEndPos - vehicleStartPos
        self.assertGreater(delta[2], 0.3)  # make sure vehicle moved at all
        self.assertAlmostEqual(delta[0], 0.0, delta=0.001)
        self.assertAlmostEqual(delta[1], 0.0, delta=0.001)

    #
    # verify that having no auto gear box but setting to automatic in the controller will not crash
    # (but send an error message)
    #
    async def test_automatic_without_auto_gear_box(self):

        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_STANDARD,
            vehiclePathsOut=vehiclePaths
        )

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])
        vehicleAPI = PhysxSchema.PhysxVehicleAPI(vehiclePrim)

        drivePath = vehicleAPI.GetDriveRel().GetTargets()
        drivePrim = stage.GetPrimAtPath(drivePath[0])
        driveAPI = PhysxSchema.PhysxVehicleDriveStandardAPI(drivePrim)
        autoGearBoxPath = driveAPI.GetAutoGearBoxRel().GetTargets()
        stage.RemovePrim(autoGearBoxPath[0])
        driveAPI.GetAutoGearBoxRel().ClearTargets(True)

        vehicleController = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)
        vehicleController.GetTargetGearAttr().Set(AUTOMATIC_GEAR)
        vehicleController.GetAcceleratorAttr().Set(1.0)

        vehicleStartPos = vehiclePrim.GetAttribute("xformOp:translate").Get()

        message = "PhysX Vehicle: transmission set to automatic but no auto gear box was defined.\n"
        with omni.physxtests.utils.ExpectMessage(self, message):
            self._prepare_for_simulation()

        # setting back to manual should not trigger an error message
        vehicleController.GetTargetGearAttr().Set(0)

        # run a bit to have the shift complete
        secondsToRun = 1.0
        self._simulate(secondsToRun)

        with omni.physxtests.utils.ExpectMessage(self, message):
            vehicleController.GetTargetGearAttr().Set(AUTOMATIC_GEAR)

        secondsToRun = 1.0
        self._simulate(secondsToRun)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        vehicleEndPos = vehiclePrim.GetAttribute("xformOp:translate").Get()

        delta = vehicleEndPos - vehicleStartPos
        self.assertAlmostEqual(delta[2], 0.0, delta=0.001)  # the vehicle should not have moved. When using automatic
                                                            # mode without an autobox, the current gear state should remain

        vehicleController.GetTargetGearAttr().Set(1)

        self._simulate(secondsToRun)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        vehicleEndPos = vehiclePrim.GetAttribute("xformOp:translate").Get()

        delta = vehicleEndPos - vehicleStartPos
        self.assertGreater(delta[2], 0.3)  # the vehicle should have moved
        self.assertAlmostEqual(delta[0], 0.0, delta=0.002)
        self.assertAlmostEqual(delta[1], 0.0, delta=0.002)

    #
    # verify that removing a wheel attachment prim while in play mode does not crash and disables
    # the wheels
    #
    async def test_dynamic_remove_wheel_attachment(self):

        for i in range(2):
            stage = await self.new_stage()

            vehicleCount = 1
            vehiclePaths = []
            wheelAttachmentPaths = []
            if (i == 0):
                driveModeIn = Factory.DRIVE_BASIC
            else:
                driveModeIn = Factory.DRIVE_STANDARD

            Factory.create4WheeledCarsScenario(
                stage,
                1.0,
                vehicleCount,
                createCollisionShapesForWheels=True,  # important to test with shapes as those add more complexity
                driveMode=driveModeIn,
                vehiclePathsOut=vehiclePaths,
                wheelAttachmentPathsOut=wheelAttachmentPaths,
            )

            vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])

            vehicleController = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)
            vehicleController.GetAcceleratorAttr().Set(1.0)

            vehicleStartPos = vehiclePrim.GetAttribute("xformOp:translate").Get()

            self._prepare_for_simulation()

            for wheelAttPath in wheelAttachmentPaths[0]:
                stage.RemovePrim(wheelAttPath)
                self._simulate_one_frame()

            secondsToRun = 0.25
            self._simulate(secondsToRun)

            self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

            vehicleEndPos = vehiclePrim.GetAttribute("xformOp:translate").Get()

            delta = vehicleEndPos - vehicleStartPos
            self.assertLess(delta[1], -0.2)  # the vehicle chassis should have fallen to the ground

    #
    # verify that removing the collision shape of a wheel attachment prim while in play mode does not crash
    # and the vehicle continues driving
    #
    async def test_dynamic_remove_wheel_attachment_shape(self):

        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
        )

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])

        vehicleController = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)
        vehicleController.GetAcceleratorAttr().Set(1.0)

        vehicleStartPos = vehiclePrim.GetAttribute("xformOp:translate").Get()

        self._prepare_for_simulation()

        for wheelAttPath in wheelAttachmentPaths[0]:
            shapePath = wheelAttPath + "/Collision"
            stage.RemovePrim(shapePath)

            self._simulate_one_frame()

        secondsToRun = 1.0
        self._simulate(secondsToRun)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        vehicleEndPos = vehiclePrim.GetAttribute("xformOp:translate").Get()

        delta = vehicleEndPos - vehicleStartPos
        self.assertGreater(delta[2], 0.3)  # make sure vehicle moved at all
        self.assertAlmostEqual(delta[0], 0.0, delta=0.001)
        self.assertAlmostEqual(delta[1], 0.0, delta=0.001)

    #
    # verify that removing a wheel attachment prim while in play mode does not crash, disables
    # the wheels and reconfigures the tank differential setup.
    #
    async def test_dynamic_remove_wheel_attachment_tank_diff(self):

        stage = await self.new_stage()

        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        vehicleData = VehicleWizard.VehicleData(get_unit_scale(stage),
            VehicleWizard.VehicleData.AXIS_Y, VehicleWizard.VehicleData.AXIS_Z)
        vehicleData.createShareableComponents = False
        vehicleData.set_number_of_axles(3)
        vehicleData.set_drive_type(VehicleWizard.DRIVE_TYPE_STANDARD)
        vehicleData.set_tank_mode(True)

        # Place the vehicle under the default prim.
        defaultPath = str(stage.GetDefaultPrim().GetPath())
        vehicleData.rootVehiclePath = defaultPath + VehicleWizard.VEHICLE_ROOT_BASE_PATH
        vehicleData.rootSharedPath = defaultPath + VehicleWizard.SHARED_DATA_ROOT_BASE_PATH
        vehiclePath = vehicleData.rootVehiclePath + "/Vehicle"

        (messageList, _) = VehicleWizard.create_vehicle(stage, vehicleData)
        self.assertFalse(messageList)  # python style guide to check for empty list

        # Create a ground plane to drive on.
        collisionGroupPath = vehicleData.rootSharedPath + VehicleWizard.VEHICLE_COLLISION_GROUP_GROUND_SURFACE_PATH
        groundMaterialPath = vehicleData.rootSharedPath + VehicleWizard.VEHICLE_GROUND_MATERIAL_PATH
        Factory.createGroundPlane(stage, vehicleData.unitScale, collisionGroupPath, groundMaterialPath)

        wheelAttPathSuffix = "References"
        wheelAttPaths = [
            vehiclePath + "/LeftWheel1" + wheelAttPathSuffix,
            vehiclePath + "/RightWheel2" + wheelAttPathSuffix,
            vehiclePath + "/LeftWheel3" + wheelAttPathSuffix,
            vehiclePath + "/RightWheel1" + wheelAttPathSuffix,
            vehiclePath + "/LeftWheel2" + wheelAttPathSuffix,
            vehiclePath + "/RightWheel3" + wheelAttPathSuffix,
        ]

        vehiclePrim = stage.GetPrimAtPath(vehiclePath)

        self.assertTrue(vehiclePrim.HasAPI(PhysxSchema.PhysxVehicleTankDifferentialAPI))

        vehicleStartPos = vehiclePrim.GetAttribute("xformOp:translate").Get()

        self._prepare_for_simulation()

        for wheelAttPath in wheelAttPaths:
            stage.RemovePrim(wheelAttPath)
            self._simulate_one_frame()

        secondsToRun = 0.25
        self._simulate(secondsToRun)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        vehicleEndPos = vehiclePrim.GetAttribute("xformOp:translate").Get()

        delta = vehicleEndPos - vehicleStartPos
        self.assertLess(delta[1], -0.2)  # the vehicle chassis should have fallen to the ground

    #
    # verify that removing the RigidBodyAPI schema of a vehicle prim does not crash and that
    # an error message gets sent
    #
    async def test_dynamic_remove_body_API(self):

        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=True,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
        )

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])

        vehicleController = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)
        vehicleController.GetAcceleratorAttr().Set(1.0)

        vehicleStartPos = vehiclePrim.GetAttribute("xformOp:translate").Get()

        self._prepare_for_simulation()

        message = "Usd Physics: vehicle \"" + vehiclePrim.GetName() + "\" needs to have RigidBodyAPI applied.\n"
        with omni.physxtests.utils.ExpectMessage(self, message):
            vehiclePrim.RemoveAPI(UsdPhysics.RigidBodyAPI)
            self._simulate_one_frame()

        secondsToRun = 0.5
        self._simulate(secondsToRun)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        vehicleEndPos = vehiclePrim.GetAttribute("xformOp:translate").Get()

        # with the removal of the body API schema, the vehicle should get destroyed and not
        # move any longer
        delta = vehicleEndPos - vehicleStartPos
        self.assertAlmostEqual(delta[0], 0.0, delta=0.001)
        self.assertAlmostEqual(delta[1], 0.0, delta=0.001)
        self.assertAlmostEqual(delta[2], 0.0, delta=0.001)

    #
    # verify that some parameters are scaled correctly by length scale by the wizard
    #
    async def test_wizard_param_unit_scaling(self):

        # make sure that for very small vehicles, the density interpolation works as expected
        # for different length scale settings

        chassis_length_meters = 0.01

        lengthScaleList = [1.0, 100.0]
        chassisMassList = []
        tireMassList = []

        for lengthScale in lengthScaleList:
            unitScale = UnitScale(lengthScale, 1.0)
            vehicleDataManager = VehicleWizard.VehicleDataManager(unitScale,
                VehicleWizard.VehicleData.AXIS_Y, VehicleWizard.VehicleData.AXIS_Z)

            vehicleDataManager.set_chassis_length(chassis_length_meters * lengthScale)
            vehicleDataManager.set_chassis_width(vehicleDataManager.vehicleData.chassisLength / 2)
            vehicleDataManager.set_chassis_height(vehicleDataManager.vehicleData.chassisWidth / 2)
            vehicleDataManager.update()

            chassisMassList.append(vehicleDataManager.vehicleData.chassisMass)
            tireMassList.append(vehicleDataManager.vehicleData.tireMass)

        epsilon = 1e-6
        self.assertAlmostEqual(chassisMassList[0], chassisMassList[1], delta=epsilon)
        self.assertAlmostEqual(tireMassList[0], tireMassList[1], delta=epsilon)

    #
    # verify that the compute_vehicle_velocity helper method works as expected.
    #
    async def test_compute_vehicle_velocity(self):

        stage = await self.new_stage()

        startSpeed = 10.0

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths
        )

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])

        rotAngleHalf = math.pi / 8.0
        vehicleOrient = Gf.Quatf(math.cos(rotAngleHalf), 0.0, math.sin(rotAngleHalf), 0.0)
        rot = Gf.Rotation(vehicleOrient)
        vehicleStartVelDir = rot.TransformDir(Gf.Vec3f(0, 0, 1))

        # rotate the vehicle to avoid a too simple setup hiding bugs
        vehiclePrim.GetAttribute("xformOp:orient").Set(vehicleOrient)

        global gPhysXInterface

        self._prepare_for_simulation()

        self._set_vehicle_speed(stage, vehiclePrim, wheelAttachmentPaths[0], startSpeed,
            vehicleStartVelDir, gPhysXInterface)

        self._simulate_one_frame()

        errThreshold = startSpeed * 0.001

        speed = gPhysXInterface.compute_vehicle_velocity(vehiclePaths[0], None)

        # passing in None should pick the local forward axis as direction
        self.assertAlmostEqual(speed, startSpeed, delta=errThreshold)

        sideDir = carb.Float3(1.0, 0.0, 0.0)
        speed = gPhysXInterface.compute_vehicle_velocity(vehiclePaths[0], sideDir)

        # the speed along the local side dir should be almost zero
        self.assertAlmostEqual(speed, 0.0, delta=errThreshold)

    #
    # verify that get_wheel_state() provides the expected hit information
    #
    async def test_wheel_state_hit_info(self):

        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []
        tireFrictionTablePaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
            tireFrictionTablePathsOut=tireFrictionTablePaths,
        )

        tireFrictionTablePrim = stage.GetPrimAtPath(tireFrictionTablePaths[Factory.TIRE_FRICTION_TABLE_WINTER_TIRE])
        tireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable(tireFrictionTablePrim)
        tarmacMaterialPath = tireFrictionTable.GetGroundMaterialsRel().GetTargets()[Factory.MATERIAL_TARMAC]

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])

        self._simulate_one_frame_with_prep()

        wheelPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][0])
        xformable = UsdGeom.Xformable(wheelPrim)
        wheelGlobalPose = xformable.ComputeLocalToWorldTransform(0)
        expectedHitPos = wheelGlobalPose.ExtractTranslation()
        expectedHitPos[1] = 0.0

        global gPhysXInterface
        wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[0][0])

        self.assertTrue(wheelState[VEHICLE_WHEEL_STATE_IS_ON_GROUND])

        rootPath = str(stage.GetDefaultPrim().GetPath())
        groundPlanePath = rootPath + FACTORY_COLLISION_PLANE_SUBPATH

        self.assertEqual(groundPlanePath, wheelState[VEHICLE_WHEEL_STATE_GROUND_ACTOR])
        self.assertEqual(groundPlanePath, wheelState[VEHICLE_WHEEL_STATE_GROUND_SHAPE])

        self.assertEqual(tarmacMaterialPath.pathString, wheelState[VEHICLE_WHEEL_STATE_GROUND_MATERIAL])

        epsilon = 0.001

        hitPlane = wheelState[VEHICLE_WHEEL_STATE_GROUND_PLANE]
        self.assertAlmostEqual(hitPlane[0], 0.0, delta=epsilon)
        self.assertAlmostEqual(hitPlane[1], 1.0, delta=epsilon)
        self.assertAlmostEqual(hitPlane[2], 0.0, delta=epsilon)
        self.assertAlmostEqual(hitPlane[3], 0.0, delta=epsilon)

        hitPos = wheelState[VEHICLE_WHEEL_STATE_GROUND_HIT_POSITION]
        self.assertAlmostEqual(hitPos[0], expectedHitPos[0], delta=epsilon)
        self.assertAlmostEqual(hitPos[1], expectedHitPos[1], delta=epsilon)
        self.assertAlmostEqual(hitPos[2], expectedHitPos[2], delta=epsilon)

    #
    # test that the sticky tire mode params affect the simulation as expected.
    #
    async def test_tire_sticky_params_longitudinal(self):
        stage = await self.new_stage()

        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        vehicleCount = 4
        vehiclePaths = []

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_NONE,
            vehiclePathsOut=vehiclePaths,
        )

        vehicleAPI = []
        startPos = []

        for i in range(vehicleCount):
            vehiclePrim = stage.GetPrimAtPath(vehiclePaths[i])
            startPos.append(vehiclePrim.GetAttribute("xformOp:translate").Get())
            vehicleAPI.append(PhysxSchema.PhysxVehicleAPI(vehiclePrim))

        stickyTireThresholdTime = 0.5 * vehicleAPI[0].GetLongitudinalStickyTireThresholdTimeAttr().Get()

        for i in range(vehicleCount):
            vehicleAPI[i].GetLongitudinalStickyTireThresholdTimeAttr().Set(stickyTireThresholdTime)
            vehicleAPI[i].CreateLongitudinalStickyTireThresholdSpeedAttr(0.2)

        self._simulate_one_frame_with_prep()

        vehicleAPI[1].GetLongitudinalStickyTireThresholdSpeedAttr().Set(0)
        vehicleAPI[2].GetLongitudinalStickyTireThresholdTimeAttr().Set(1000)
        vehicleAPI[3].GetLongitudinalStickyTireDampingAttr().Set(0)

        # run until sticky mode kicks in (for reference setup)
        secondsToRun = stickyTireThresholdTime * 1.2
        self._simulate(secondsToRun)

        acceleration = 60.0
        force = acceleration * 1800
        global gPhysXInterface
        for i in range(vehicleCount):
            rbo_encoded = PhysicsSchemaTools.sdfPathToInt(vehiclePaths[i])
            get_physx_simulation_interface().apply_force_at_pos(stage_id, rbo_encoded, Gf.Vec3f(0.0, 0.0, force),
                startPos[i], "Force")

        secondsToRun = 1.0
        self._simulate(secondsToRun)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        deltaPosList = []

        for i in range(vehicleCount):
            vehiclePrim = stage.GetPrimAtPath(vehiclePaths[i])
            endPos = vehiclePrim.GetAttribute("xformOp:translate").Get()
            deltaPos = endPos - startPos[i]
            deltaPosList.append(deltaPos)
            self.assertAlmostEqual(deltaPos[0], 0.0, delta=0.001)
            self.assertAlmostEqual(deltaPos[1], 0.0, delta=0.001)

        self.assertAlmostEqual(deltaPosList[0][2], 0.0, delta=0.01)

        # all vehicles without reference settings are configured such that the tires should
        # not enter the sticky mode (or sticky mode should not have an impact). As a consequence,
        # applying a force should move the vehicle
        self.assertGreater(deltaPosList[1][2], 0.5)
        self.assertGreater(deltaPosList[2][2], 0.5)
        self.assertGreater(deltaPosList[3][2], 0.5)

    #
    # test that the sticky tire mode params affect the simulation as expected.
    #
    async def test_tire_sticky_params_lateral(self):
        stage = await self.new_stage()

        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        vehicleCount = 4
        vehiclePaths = []

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_NONE,
            vehiclePathsOut=vehiclePaths,
            vehicleDelta=[-10, 0, 0],
        )

        vehicleAPI = []
        startPos = []

        for i in range(vehicleCount):
            vehiclePrim = stage.GetPrimAtPath(vehiclePaths[i])
            startPos.append(vehiclePrim.GetAttribute("xformOp:translate").Get())
            vehicleAPI.append(PhysxSchema.PhysxVehicleAPI(vehiclePrim))

        stickyTireThresholdTimeLong = 0.5 * vehicleAPI[0].GetLongitudinalStickyTireThresholdTimeAttr().Get()
        stickyTireThresholdTimeLat = 0.5 * vehicleAPI[0].GetLateralStickyTireThresholdTimeAttr().Get()

        for i in range(vehicleCount):
            vehicleAPI[i].GetLongitudinalStickyTireThresholdTimeAttr().Set(stickyTireThresholdTimeLong)
            vehicleAPI[i].GetLateralStickyTireThresholdTimeAttr().Set(stickyTireThresholdTimeLat)
            vehicleAPI[i].GetLateralStickyTireDampingAttr().Set(200)
            vehicleAPI[i].CreateLateralStickyTireThresholdSpeedAttr(0.2)

        self._simulate_one_frame_with_prep()

        vehicleAPI[1].GetLateralStickyTireThresholdSpeedAttr().Set(0)
        vehicleAPI[2].GetLateralStickyTireThresholdTimeAttr().Set(1000)
        vehicleAPI[3].GetLateralStickyTireDampingAttr().Set(0)

        # run until sticky mode kicks in (for reference setup)
        secondsToRun = (stickyTireThresholdTimeLat + stickyTireThresholdTimeLong) * 1.2
        self._simulate(secondsToRun)

        acceleration = 60.0
        force = acceleration * 1800
        global gPhysXInterface
        for i in range(vehicleCount):
            rbo_encoded = PhysicsSchemaTools.sdfPathToInt(vehiclePaths[i])
            get_physx_simulation_interface().apply_force_at_pos(stage_id, rbo_encoded, Gf.Vec3f(force, 0.0, 0.0),
                startPos[i], "Force")

        secondsToRun = 1.0
        self._simulate(secondsToRun)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        deltaPosList = []

        for i in range(vehicleCount):
            vehiclePrim = stage.GetPrimAtPath(vehiclePaths[i])
            endPos = vehiclePrim.GetAttribute("xformOp:translate").Get()
            deltaPos = endPos - startPos[i]
            deltaPosList.append(deltaPos)
            self.assertAlmostEqual(deltaPos[1], 0.0, delta=0.001)
            self.assertAlmostEqual(deltaPos[2], 0.0, delta=0.001)

        self.assertAlmostEqual(deltaPosList[0][0], 0.0, delta=0.005)

        # all vehicles without reference settings are configured such that the tires should
        # not enter the sticky mode (or sticky mode should not have an impact). As a consequence,
        # applying a force should move the vehicle
        self.assertGreater(deltaPosList[1][0], 0.02)
        self.assertGreater(deltaPosList[2][0], 0.02)
        self.assertGreater(deltaPosList[3][0], 0.02)

    async def _test_multi_wheel_differential(self, driveModeIn, useDeprecatedAPI):
        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []
        if (not useDeprecatedAPI):
            useDepAPIList = None
        else:
            useDepAPIList = [True]

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=driveModeIn,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
            useDeprecatedAPIList=useDepAPIList
        )

        self._remove_factory_ground_plane_and_set_gravity_zero()

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])
        vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)
        vehicleControllerAPI.GetAcceleratorAttr().Set(1.0)

        if (driveModeIn == Factory.DRIVE_STANDARD):
            vehicleControllerAPI.GetTargetGearAttr().Set(1)

        wheelAttIndexList = [3, 2, 1, 0]  # re-arrange wheel attachment indices to test if that works too
        drivenWheelIndexList = [3, 0]
        drivenWheelPathIndexList = [0, 3]
        ratios = [0.66, 0.33]

        for i in range(4):
            wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][i])
            wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(wheelAttPrim)

            if (not useDeprecatedAPI):
                wheelAttAPI.GetIndexAttr().Set(wheelAttIndexList[i])
            else:
                if (i in drivenWheelPathIndexList):
                    wheelAttAPI.GetDrivenAttr().Set(1)
                else:
                    wheelAttAPI.GetDrivenAttr().Set(0)

        if (not useDeprecatedAPI):
            multiWheelDiffAPI = PhysxSchema.PhysxVehicleMultiWheelDifferentialAPI(vehiclePrim)
            multiWheelDiffAPI.GetWheelsAttr().Set(drivenWheelIndexList)
            multiWheelDiffAPI.GetTorqueRatiosAttr().Set(ratios)

            if (driveModeIn == Factory.DRIVE_STANDARD):
                multiWheelDiffAPI.GetAverageWheelSpeedRatiosAttr().Set(ratios)

        secondsToRun = 0.1
        self._simulate_with_prep(secondsToRun)

        global gPhysXInterface

        errTol = 0.001
        minDrivenSpeed = 1.0

        wheelRotSpeeds = []
        for i in range(4):
            wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[0][i])
            rotSpeed = wheelState[VEHICLE_WHEEL_STATE_ROTATION_SPEED]
            wheelRotSpeeds.append(rotSpeed)

            if (i in drivenWheelPathIndexList):
                self.assertGreater(rotSpeed, minDrivenSpeed)
            else:
                self.assertAlmostEqual(rotSpeed, 0.0, delta=errTol)

        if (not useDeprecatedAPI):
            self.assertAlmostEqual(wheelRotSpeeds[drivenWheelPathIndexList[0]] / ratios[0],
                wheelRotSpeeds[drivenWheelPathIndexList[1]] / ratios[1], delta=0.01)

        # now unlink one of the driven wheels from the "engine" while the simulation is running

        gPhysXInterface.set_vehicle_to_rest_state(vehiclePaths[0])

        drivenWheelIndexList = [0]
        drivenWheelPathIndexList = [3]
        ratios = [0, 1.0]

        if (not useDeprecatedAPI):
            multiWheelDiffAPI.GetTorqueRatiosAttr().Set(ratios)
            if (driveModeIn == Factory.DRIVE_STANDARD):
                multiWheelDiffAPI.GetAverageWheelSpeedRatiosAttr().Set(ratios)
        else:
            wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][0])
            wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(wheelAttPrim)
            wheelAttAPI.GetDrivenAttr().Set(0)

        self._simulate(secondsToRun)

        wheelRotSpeeds = []
        for i in range(4):
            wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[0][i])
            rotSpeed = wheelState[VEHICLE_WHEEL_STATE_ROTATION_SPEED]
            wheelRotSpeeds.append(rotSpeed)

            if (i in drivenWheelPathIndexList):
                self.assertGreater(rotSpeed, minDrivenSpeed)
            else:
                self.assertAlmostEqual(rotSpeed, 0.0, delta=errTol)

    #
    # verify that multi wheel differential param changes effect simulation as expected
    #
    async def test_multi_wheel_differential_basic(self):
        await self._test_multi_wheel_differential(Factory.DRIVE_BASIC, False)

    #
    # verify that multi wheel differential param changes effect simulation as expected
    #
    async def test_multi_wheel_differential_basic_deprecated(self):
        await self._test_multi_wheel_differential(Factory.DRIVE_BASIC, True)

    #
    # verify that multi wheel differential param changes effect simulation as expected
    #
    async def test_multi_wheel_differential_standard(self):
        await self._test_multi_wheel_differential(Factory.DRIVE_STANDARD, False)

    #
    # verify that multi wheel differential param changes effect simulation as expected
    #
    async def test_multi_wheel_differential_standard_deprecated(self):
        await self._test_multi_wheel_differential(Factory.DRIVE_STANDARD, True)

    async def _test_brakes(self, driveModeIn, useDeprecatedAPI):
        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []
        if (not useDeprecatedAPI):
            useDepAPIList = None
        else:
            useDepAPIList = [True]

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=driveModeIn,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
            useShareableComponentsList=[False],
            useDeprecatedAPIList=useDepAPIList
        )

        self._remove_factory_ground_plane_and_set_gravity_zero()

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])
        vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)

        wheelList = []
        for path in wheelAttachmentPaths[0]:
            wheelAttPrim = stage.GetPrimAtPath(path)
            wheelAPI = self._get_wheel(wheelAttPrim, stage)
            wheelList.append(wheelAPI)

        brakes0Wheels = [Factory.WHEEL_FRONT_LEFT, Factory.WHEEL_REAR_RIGHT]
        brakes1Wheels = [Factory.WHEEL_FRONT_RIGHT, Factory.WHEEL_REAR_LEFT]
        defaultMaxBrakeTorque = 3600
        if (not useDeprecatedAPI):
            brakes0API = PhysxSchema.PhysxVehicleBrakesAPI(vehiclePrim, PhysxSchema.Tokens.brakes0)
            brakes0API.GetWheelsAttr().Set(brakes0Wheels)
            brakes0API.GetMaxBrakeTorqueAttr().Set(defaultMaxBrakeTorque)
            brakes0API.CreateTorqueMultipliersAttr([1] * len(brakes0Wheels))

            brakes1API = PhysxSchema.PhysxVehicleBrakesAPI(vehiclePrim, PhysxSchema.Tokens.brakes1)
            brakes1API.GetWheelsAttr().Set(brakes1Wheels)
            brakes1API.GetMaxBrakeTorqueAttr().Set(defaultMaxBrakeTorque)
            brakes1API.CreateTorqueMultipliersAttr([1] * len(brakes1Wheels))
        else:
            for index in brakes0Wheels:
                wheelList[index].GetMaxBrakeTorqueAttr().Set(defaultMaxBrakeTorque)
                wheelList[index].GetMaxHandBrakeTorqueAttr().Set(0)
            for index in brakes1Wheels:
                wheelList[index].GetMaxBrakeTorqueAttr().Set(0)
                wheelList[index].GetMaxHandBrakeTorqueAttr().Set(defaultMaxBrakeTorque)

        startSpeed = 30
        refWheelSpeeds = []
        for i in range(4):
            wheelRadius = wheelList[i].GetRadiusAttr().Get()
            rotationSpeed = startSpeed / wheelRadius  # assuming free rolling
            refWheelSpeeds.append(rotationSpeed)

        self._prepare_for_simulation()
        noBrakeSpeedMin = refWheelSpeeds[0] * 0.95
        brakeSpeedMax = refWheelSpeeds[0] * 0.5

        global gPhysXInterface

        def set_wheel_speeds(wheelAttachmentPaths, speedsIn):
            for i in range(4):
                gPhysXInterface.set_wheel_rotation_speed(wheelAttachmentPaths[i], speedsIn[i])

        def get_wheel_speeds(wheelAttachmentPaths, speedsOut):
            speedsOut.clear()

            for i in range(len(wheelAttachmentPaths)):
                wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[i])
                rotSpeed = wheelState[VEHICLE_WHEEL_STATE_ROTATION_SPEED]
                speedsOut.append(rotSpeed)

        wheelRotationSpeeds = []

        #
        # no brake -> speed should be almost the same
        #

        set_wheel_speeds(wheelAttachmentPaths[0], refWheelSpeeds)
        self._simulate_one_frame()
        get_wheel_speeds(wheelAttachmentPaths[0], wheelRotationSpeeds)

        for index in brakes0Wheels:
            self.assertGreater(wheelRotationSpeeds[index], noBrakeSpeedMin)
        for index in brakes1Wheels:
            self.assertGreater(wheelRotationSpeeds[index], noBrakeSpeedMin)

        #
        # brake0 -> corresponding wheels should slow down a lot
        #
        if (not useDeprecatedAPI):
            vehicleControllerAPI.GetBrake0Attr().Set(1)
            vehicleControllerAPI.GetBrake1Attr().Set(0)
        else:
            vehicleControllerAPI.GetBrakeAttr().Set(1)
            vehicleControllerAPI.GetHandbrakeAttr().Set(0)

        set_wheel_speeds(wheelAttachmentPaths[0], refWheelSpeeds)
        self._simulate_one_frame()
        get_wheel_speeds(wheelAttachmentPaths[0], wheelRotationSpeeds)

        for index in brakes0Wheels:
            self.assertLess(wheelRotationSpeeds[index], brakeSpeedMax)
        for index in brakes1Wheels:
            self.assertGreater(wheelRotationSpeeds[index], noBrakeSpeedMin)

        #
        # brake1 -> corresponding wheels should slow down a lot
        #
        if (not useDeprecatedAPI):
            vehicleControllerAPI.GetBrake0Attr().Set(0)
            vehicleControllerAPI.GetBrake1Attr().Set(1)
        else:
            vehicleControllerAPI.GetBrakeAttr().Set(0)
            vehicleControllerAPI.GetHandbrakeAttr().Set(1)

        set_wheel_speeds(wheelAttachmentPaths[0], refWheelSpeeds)
        self._simulate_one_frame()
        get_wheel_speeds(wheelAttachmentPaths[0], wheelRotationSpeeds)

        for index in brakes0Wheels:
            self.assertGreater(wheelRotationSpeeds[index], noBrakeSpeedMin)
        for index in brakes1Wheels:
            self.assertLess(wheelRotationSpeeds[index], brakeSpeedMax)

        #
        # brake0, brake1 but max torque of brake0=0
        # -> speed should be almost the same
        #
        if (not useDeprecatedAPI):
            vehicleControllerAPI.GetBrake0Attr().Set(1)
            vehicleControllerAPI.GetBrake1Attr().Set(1)
            brakes0API.GetMaxBrakeTorqueAttr().Set(0)
            brakes1API.GetMaxBrakeTorqueAttr().Set(defaultMaxBrakeTorque)
        else:
            vehicleControllerAPI.GetBrakeAttr().Set(1)
            vehicleControllerAPI.GetHandbrakeAttr().Set(1)
            for index in brakes0Wheels:
                wheelList[index].GetMaxBrakeTorqueAttr().Set(0)
            for index in brakes1Wheels:
                wheelList[index].GetMaxHandBrakeTorqueAttr().Set(defaultMaxBrakeTorque)

        set_wheel_speeds(wheelAttachmentPaths[0], refWheelSpeeds)
        self._simulate_one_frame()
        get_wheel_speeds(wheelAttachmentPaths[0], wheelRotationSpeeds)

        for index in brakes0Wheels:
            self.assertGreater(wheelRotationSpeeds[index], noBrakeSpeedMin)
        for index in brakes1Wheels:
            self.assertLess(wheelRotationSpeeds[index], brakeSpeedMax)

        #
        # brake0, brake1 but max torque of brake1=0
        # -> speed should be almost the same
        #
        if (not useDeprecatedAPI):
            vehicleControllerAPI.GetBrake0Attr().Set(1)
            vehicleControllerAPI.GetBrake1Attr().Set(1)
            brakes0API.GetMaxBrakeTorqueAttr().Set(defaultMaxBrakeTorque)
            brakes1API.GetMaxBrakeTorqueAttr().Set(0)
        else:
            vehicleControllerAPI.GetBrakeAttr().Set(1)
            vehicleControllerAPI.GetHandbrakeAttr().Set(1)
            for index in brakes0Wheels:
                wheelList[index].GetMaxBrakeTorqueAttr().Set(defaultMaxBrakeTorque)
            for index in brakes1Wheels:
                wheelList[index].GetMaxHandBrakeTorqueAttr().Set(0)

        set_wheel_speeds(wheelAttachmentPaths[0], refWheelSpeeds)
        self._simulate_one_frame()
        get_wheel_speeds(wheelAttachmentPaths[0], wheelRotationSpeeds)

        for index in brakes0Wheels:
            self.assertLess(wheelRotationSpeeds[index], brakeSpeedMax)
        for index in brakes1Wheels:
            self.assertGreater(wheelRotationSpeeds[index], noBrakeSpeedMin)

        if (not useDeprecatedAPI):
            #
            # brake0, brake1 but setting certain torque multipliers to 1
            # -> speed should be almost the same
            #
            vehicleControllerAPI.GetBrake0Attr().Set(1)
            vehicleControllerAPI.GetBrake1Attr().Set(1)
            brakes0API.GetMaxBrakeTorqueAttr().Set(defaultMaxBrakeTorque)
            brakes0API.GetTorqueMultipliersAttr().Set([1, 0])
            brakes1API.GetMaxBrakeTorqueAttr().Set(defaultMaxBrakeTorque)
            brakes1API.GetTorqueMultipliersAttr().Set([0, 1])

            set_wheel_speeds(wheelAttachmentPaths[0], refWheelSpeeds)
            self._simulate_one_frame()
            get_wheel_speeds(wheelAttachmentPaths[0], wheelRotationSpeeds)

            self.assertLess(wheelRotationSpeeds[brakes0Wheels[0]], brakeSpeedMax)
            self.assertGreater(wheelRotationSpeeds[brakes0Wheels[1]], noBrakeSpeedMin)
            self.assertGreater(wheelRotationSpeeds[brakes1Wheels[0]], noBrakeSpeedMin)
            self.assertLess(wheelRotationSpeeds[brakes1Wheels[1]], brakeSpeedMax)

    #
    # verify that brakes param changes effect simulation as expected
    #
    async def test_brakes(self):
        await self._test_brakes(Factory.DRIVE_BASIC, False)

    #
    # verify that brakes param changes effect simulation as expected
    #
    async def test_brakes_deprecated(self):
        await self._test_brakes(Factory.DRIVE_BASIC, True)

    #
    # verify that everything works too if only the "brakes1" instance of the
    # PhysxVehicleBrakesAPI API schema is used
    #
    async def test_brakes1_only(self):
        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths
        )

        self._remove_factory_ground_plane_and_set_gravity_zero()

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])
        vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)

        # remove the "brakes0" instance of PhysxVehicleBrakesAPI
        vehiclePrim.RemoveAPI(PhysxSchema.PhysxVehicleBrakesAPI, PhysxSchema.Tokens.brakes0)

        defaultMaxBrakeTorque = 3600
        brakes1API = PhysxSchema.PhysxVehicleBrakesAPI(vehiclePrim, PhysxSchema.Tokens.brakes1)
        brakes1API.GetMaxBrakeTorqueAttr().Set(defaultMaxBrakeTorque)
        brakes1API.GetWheelsAttr().Set([])

        wheelList = []
        for path in wheelAttachmentPaths[0]:
            wheelAttPrim = stage.GetPrimAtPath(path)
            wheelAPI = self._get_wheel(wheelAttPrim, stage)
            wheelList.append(wheelAPI)

        startSpeed = 30
        refWheelSpeeds = []
        for i in range(4):
            wheelRadius = wheelList[i].GetRadiusAttr().Get()
            rotationSpeed = startSpeed / wheelRadius  # assuming free rolling
            refWheelSpeeds.append(rotationSpeed)

        self._prepare_for_simulation()
        noBrakeSpeedMin = refWheelSpeeds[0] * 0.95
        brakeSpeedMax = refWheelSpeeds[0] * 0.5

        global gPhysXInterface

        def set_wheel_speeds(wheelAttachmentPaths, speedsIn):
            for i in range(4):
                gPhysXInterface.set_wheel_rotation_speed(wheelAttachmentPaths[i], speedsIn[i])

        def get_wheel_speeds(wheelAttachmentPaths, speedsOut):
            speedsOut.clear()

            for i in range(len(wheelAttachmentPaths)):
                wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[i])
                rotSpeed = wheelState[VEHICLE_WHEEL_STATE_ROTATION_SPEED]
                speedsOut.append(rotSpeed)

        wheelRotationSpeeds = []

        #
        # brake0 should have no effect
        #

        vehicleControllerAPI.GetBrake0Attr().Set(1)
        vehicleControllerAPI.GetBrake1Attr().Set(0)

        set_wheel_speeds(wheelAttachmentPaths[0], refWheelSpeeds)
        self._simulate_one_frame()
        get_wheel_speeds(wheelAttachmentPaths[0], wheelRotationSpeeds)

        for i in range(4):
            self.assertGreater(wheelRotationSpeeds[i], noBrakeSpeedMin)

        #
        # brake1 -> wheels should slow down a lot
        #

        vehicleControllerAPI.GetBrake0Attr().Set(0)
        vehicleControllerAPI.GetBrake1Attr().Set(1)

        set_wheel_speeds(wheelAttachmentPaths[0], refWheelSpeeds)
        self._simulate_one_frame()
        get_wheel_speeds(wheelAttachmentPaths[0], wheelRotationSpeeds)

        for i in range(4):
            self.assertLess(wheelRotationSpeeds[i], brakeSpeedMax)

    async def _test_steering(self, driveModeIn, useDeprecatedAPI):
        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []
        if (not useDeprecatedAPI):
            useDepAPIList = None
        else:
            useDepAPIList = [True]

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=driveModeIn,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
            useShareableComponentsList=[False],
            useDeprecatedAPIList=useDepAPIList
        )

        self._remove_factory_ground_plane_and_set_gravity_zero()

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])
        vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)

        wheelList = []
        for path in wheelAttachmentPaths[0]:
            wheelAttPrim = stage.GetPrimAtPath(path)
            wheelAPI = self._get_wheel(wheelAttPrim, stage)
            wheelList.append(wheelAPI)

        steeredWheels = [Factory.WHEEL_FRONT_LEFT, Factory.WHEEL_REAR_RIGHT]
        nonSteeredWheels = [Factory.WHEEL_FRONT_RIGHT, Factory.WHEEL_REAR_LEFT]
        defaultMaxSteerAngle = 0.554264
        if (not useDeprecatedAPI):
            steeringAPI = PhysxSchema.PhysxVehicleSteeringAPI(vehiclePrim)
            steeringAPI.GetWheelsAttr().Set(steeredWheels)
            steeringAPI.GetMaxSteerAngleAttr().Set(defaultMaxSteerAngle)
            steeringAPI.CreateAngleMultipliersAttr([1] * len(steeredWheels))
        else:
            for index in steeredWheels:
                wheelList[index].GetMaxSteerAngleAttr().Set(defaultMaxSteerAngle)
            for index in nonSteeredWheels:
                wheelList[index].GetMaxSteerAngleAttr().Set(0)

        self._prepare_for_simulation()
        errTol = 0.001

        def get_wheel_steer_angles(wheelAttachmentPaths, anglesOut):
            anglesOut.clear()
            refDir = Gf.Vec3f(0, 0, 1)

            for wheelAttPath in wheelAttachmentPaths:
                wheelAttPrim = stage.GetPrimAtPath(wheelAttPath)
                xformable = UsdGeom.Xformable(wheelAttPrim)
                wheelDir = self._local_dir_to_world(refDir, xformable)
                deltaAngle = abs(self._get_delta_angle_radians(refDir, wheelDir))
                if (wheelDir[0] < -0.01):
                    deltaAngle = -deltaAngle
                anglesOut.append(deltaAngle)

        wheelSteerAngles = []

        #
        # no steer -> angles should be 0
        #

        self._simulate_one_frame()
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)
        get_wheel_steer_angles(wheelAttachmentPaths[0], wheelSteerAngles)

        for index in steeredWheels:
            self.assertAlmostEqual(wheelSteerAngles[index], 0.0, delta=errTol)
        for index in nonSteeredWheels:
            self.assertAlmostEqual(wheelSteerAngles[index], 0.0, delta=errTol)

        #
        # full steer left -> corresponding wheels should have max steer angle
        #
        if (not useDeprecatedAPI):
            vehicleControllerAPI.GetSteerAttr().Set(1)
        else:
            vehicleControllerAPI.GetSteerLeftAttr().Set(1)
            vehicleControllerAPI.GetSteerRightAttr().Set(0)

        self._simulate_one_frame()
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)
        get_wheel_steer_angles(wheelAttachmentPaths[0], wheelSteerAngles)

        for index in steeredWheels:
            self.assertAlmostEqual(wheelSteerAngles[index], defaultMaxSteerAngle, delta=errTol)
        for index in nonSteeredWheels:
            self.assertAlmostEqual(wheelSteerAngles[index], 0.0, delta=errTol)

        #
        # full steer right -> corresponding wheels should have negative max steer angle
        #
        if (not useDeprecatedAPI):
            vehicleControllerAPI.GetSteerAttr().Set(-1)
        else:
            vehicleControllerAPI.GetSteerLeftAttr().Set(0)
            vehicleControllerAPI.GetSteerRightAttr().Set(1)

        self._simulate_one_frame()
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)
        get_wheel_steer_angles(wheelAttachmentPaths[0], wheelSteerAngles)

        for index in steeredWheels:
            self.assertAlmostEqual(wheelSteerAngles[index], -defaultMaxSteerAngle, delta=errTol)
        for index in nonSteeredWheels:
            self.assertAlmostEqual(wheelSteerAngles[index], 0.0, delta=errTol)

        #
        # full steer but max steer angle zero -> zero steer angle
        #
        if (not useDeprecatedAPI):
            vehicleControllerAPI.GetSteerAttr().Set(1)
            steeringAPI.GetMaxSteerAngleAttr().Set(0)
        else:
            vehicleControllerAPI.GetSteerLeftAttr().Set(1)
            vehicleControllerAPI.GetSteerRightAttr().Set(0)
            for index in steeredWheels:
                wheelList[index].GetMaxSteerAngleAttr().Set(0)

        self._simulate_one_frame()
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)
        get_wheel_steer_angles(wheelAttachmentPaths[0], wheelSteerAngles)

        for index in steeredWheels:
            self.assertAlmostEqual(wheelSteerAngles[index], 0.0, delta=errTol)
        for index in nonSteeredWheels:
            self.assertAlmostEqual(wheelSteerAngles[index], 0.0, delta=errTol)

        if (not useDeprecatedAPI):
            #
            # full steer and default max steer angle but custom angle multipliers
            #
            vehicleControllerAPI.GetSteerAttr().Set(1)
            steeringAPI.GetMaxSteerAngleAttr().Set(defaultMaxSteerAngle)
            steeringAPI.GetAngleMultipliersAttr().Set([0, -1])

            self._simulate_one_frame()
            self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)
            get_wheel_steer_angles(wheelAttachmentPaths[0], wheelSteerAngles)

            self.assertAlmostEqual(wheelSteerAngles[steeredWheels[0]], 0.0, delta=errTol)
            self.assertAlmostEqual(wheelSteerAngles[steeredWheels[1]], -defaultMaxSteerAngle, delta=errTol)

            for index in nonSteeredWheels:
                self.assertAlmostEqual(wheelSteerAngles[index], 0.0, delta=errTol)

    #
    # verify that steering param changes effect simulation as expected
    #
    async def test_steering(self):
        await self._test_steering(Factory.DRIVE_BASIC, False)

    #
    # verify that steering param changes effect simulation as expected
    #
    async def test_steering_deprecated(self):
        await self._test_steering(Factory.DRIVE_BASIC, True)

    async def _test_ackermann_steering(self, driveModeIn):
        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=driveModeIn,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
            useAckermannCorrection=True
        )

        self._remove_factory_ground_plane_and_set_gravity_zero()

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])
        vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)

        steeredWheels = [Factory.WHEEL_FRONT_RIGHT, Factory.WHEEL_FRONT_LEFT]
        nonSteeredWheels = [Factory.WHEEL_REAR_LEFT, Factory.WHEEL_REAR_RIGHT]
        defaultMaxSteerAngle = 0.554264
        maxAngleLow = defaultMaxSteerAngle * 0.5;
        maxAngleHigh = defaultMaxSteerAngle * 0.95;

        steeringAPI = PhysxSchema.PhysxVehicleAckermannSteeringAPI(vehiclePrim)
        steeringAPI.GetWheel0Attr().Set(steeredWheels[0])
        steeringAPI.GetWheel1Attr().Set(steeredWheels[1])
        steeringAPI.GetMaxSteerAngleAttr().Set(defaultMaxSteerAngle)

        wheelBase = steeringAPI.GetWheelBaseAttr().Get()
        trackWidth = steeringAPI.GetTrackWidthAttr().Get()

        self._prepare_for_simulation()
        errTol = 0.001

        def get_wheel_steer_angles(wheelAttachmentPaths, anglesOut):
            anglesOut.clear()
            refDir = Gf.Vec3f(0, 0, 1)

            for wheelAttPath in wheelAttachmentPaths:
                wheelAttPrim = stage.GetPrimAtPath(wheelAttPath)
                xformable = UsdGeom.Xformable(wheelAttPrim)
                wheelDir = self._local_dir_to_world(refDir, xformable)
                deltaAngle = abs(self._get_delta_angle_radians(refDir, wheelDir))
                if (wheelDir[0] < -0.01):
                    deltaAngle = -deltaAngle
                anglesOut.append(deltaAngle)

        wheelSteerAngles = []

        #
        # no steer -> angles should be 0
        #

        self._simulate_one_frame()
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)
        get_wheel_steer_angles(wheelAttachmentPaths[0], wheelSteerAngles)

        for index in steeredWheels:
            self.assertAlmostEqual(wheelSteerAngles[index], 0.0, delta=errTol)
        for index in nonSteeredWheels:
            self.assertAlmostEqual(wheelSteerAngles[index], 0.0, delta=errTol)

        #
        # full steer left -> inner wheel should have max steer angle, outer wheel a bit less
        #
        vehicleControllerAPI.GetSteerAttr().Set(1)

        self._simulate_one_frame()
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)
        get_wheel_steer_angles(wheelAttachmentPaths[0], wheelSteerAngles)

        self.assertAlmostEqual(wheelSteerAngles[steeredWheels[1]], defaultMaxSteerAngle, delta=errTol)
        self.assertGreater(wheelSteerAngles[steeredWheels[0]], maxAngleLow)
        self.assertLess(wheelSteerAngles[steeredWheels[0]], maxAngleHigh)
        for index in nonSteeredWheels:
            self.assertAlmostEqual(wheelSteerAngles[index], 0.0, delta=errTol)

        outerWheelAngle = wheelSteerAngles[steeredWheels[0]]

        #
        # full steer right -> inner wheel should have max steer angle, outer wheel a bit less
        #
        vehicleControllerAPI.GetSteerAttr().Set(-1)

        self._simulate_one_frame()
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)
        get_wheel_steer_angles(wheelAttachmentPaths[0], wheelSteerAngles)

        self.assertAlmostEqual(wheelSteerAngles[steeredWheels[0]], -defaultMaxSteerAngle, delta=errTol)
        self.assertLess(wheelSteerAngles[steeredWheels[1]], -maxAngleLow)
        self.assertGreater(wheelSteerAngles[steeredWheels[1]], -maxAngleHigh)
        for index in nonSteeredWheels:
            self.assertAlmostEqual(wheelSteerAngles[index], 0.0, delta=errTol)

        #
        # full steer but max steer angle zero -> zero steer angle
        #
        vehicleControllerAPI.GetSteerAttr().Set(1)
        steeringAPI.GetMaxSteerAngleAttr().Set(0)

        self._simulate_one_frame()
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)
        get_wheel_steer_angles(wheelAttachmentPaths[0], wheelSteerAngles)

        for index in steeredWheels:
            self.assertAlmostEqual(wheelSteerAngles[index], 0.0, delta=errTol)
        for index in nonSteeredWheels:
            self.assertAlmostEqual(wheelSteerAngles[index], 0.0, delta=errTol)

        steeringAPI.GetMaxSteerAngleAttr().Set(defaultMaxSteerAngle)

        #
        # full steer left and increase wheel base -> outer wheel should turn more than previously
        #
        vehicleControllerAPI.GetSteerAttr().Set(1)
        steeringAPI.GetWheelBaseAttr().Set(2.0 * wheelBase)

        self._simulate_one_frame()
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)
        get_wheel_steer_angles(wheelAttachmentPaths[0], wheelSteerAngles)

        self.assertAlmostEqual(wheelSteerAngles[steeredWheels[1]], defaultMaxSteerAngle, delta=errTol)
        self.assertGreater(wheelSteerAngles[steeredWheels[0]], outerWheelAngle * 1.05)
        self.assertLess(wheelSteerAngles[steeredWheels[0]], maxAngleHigh)
        for index in nonSteeredWheels:
            self.assertAlmostEqual(wheelSteerAngles[index], 0.0, delta=errTol)

        steeringAPI.GetWheelBaseAttr().Set(wheelBase)

        #
        # full steer left and increase track width -> outer wheel should turn less than previously
        #
        vehicleControllerAPI.GetSteerAttr().Set(1)
        steeringAPI.GetTrackWidthAttr().Set(2.0 * trackWidth)

        self._simulate_one_frame()
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)
        get_wheel_steer_angles(wheelAttachmentPaths[0], wheelSteerAngles)

        self.assertAlmostEqual(wheelSteerAngles[steeredWheels[1]], defaultMaxSteerAngle, delta=errTol)
        self.assertGreater(wheelSteerAngles[steeredWheels[0]], maxAngleLow)
        self.assertLess(wheelSteerAngles[steeredWheels[0]], outerWheelAngle * 0.95)
        for index in nonSteeredWheels:
            self.assertAlmostEqual(wheelSteerAngles[index], 0.0, delta=errTol)

        steeringAPI.GetTrackWidthAttr().Set(trackWidth)

        #
        # full steer and zero Ackermann correction -> corresponding wheels should have max steer angle
        #
        vehicleControllerAPI.GetSteerAttr().Set(1)
        steeringAPI.GetStrengthAttr().Set(0.0)

        self._simulate_one_frame()
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)
        get_wheel_steer_angles(wheelAttachmentPaths[0], wheelSteerAngles)

        for index in steeredWheels:
            self.assertAlmostEqual(wheelSteerAngles[index], defaultMaxSteerAngle, delta=errTol)
        for index in nonSteeredWheels:
            self.assertAlmostEqual(wheelSteerAngles[index], 0.0, delta=errTol)

        steeringAPI.GetStrengthAttr().Set(1.0)

    #
    # verify that Ackermann steering param changes effect simulation as expected
    #
    async def test_ackermann_steering_drive_basic(self):
        await self._test_ackermann_steering(Factory.DRIVE_BASIC)

    #
    # verify that Ackermann steering param changes effect simulation as expected
    #
    async def test_ackermann_steering_drive_standard(self):
        await self._test_ackermann_steering(Factory.DRIVE_STANDARD)

    #
    # test the vehicle tank configuration (tank differential and tank controller)
    #
    async def test_tank(self):
        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_STANDARD,
            vehiclePathsOut=vehiclePaths,
            configureAsTank=True
        )

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])
        vehicleTankControllerAPI = PhysxSchema.PhysxVehicleTankControllerAPI(vehiclePrim)
        vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI(vehicleTankControllerAPI)

        vehicleControllerAPI.GetAcceleratorAttr().Set(1.0)
        vehicleTankControllerAPI.GetThrust0Attr().Set(-1.0)
        vehicleTankControllerAPI.GetThrust1Attr().Set(1.0)

        startPos = vehiclePrim.GetAttribute("xformOp:translate").Get()

        refDir = Gf.Vec3f(0.0, 0.0, 1.0)
        xformable = UsdGeom.Xformable(vehiclePrim)

        startDir = self._local_dir_to_world(refDir, xformable)

        secondsToRun = 2.0
        self._simulate_with_prep(secondsToRun)
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        endPos = vehiclePrim.GetAttribute("xformOp:translate").Get()
        endDir = self._local_dir_to_world(refDir, xformable)

        deltaPos = endPos - startPos
        deltaDir = endDir - startDir
        deltaAngle = self._get_delta_angle_degree(startDir, endDir)

        # make sure the vehicle rotated in the right direction while staying somewhat
        # close to the initial position
        errTol = 0.001
        self.assertLess(math.fabs(deltaPos[0]), 0.8)
        self.assertAlmostEqual(deltaPos[1], 0.0, delta=errTol)
        self.assertLess(math.fabs(deltaPos[2]), 0.3)

        self.assertGreater(deltaAngle, 50.0)
        self.assertGreater(deltaDir[0], 0.5)
        self.assertAlmostEqual(deltaDir[1], 0.0, delta=errTol)

        startPos = endPos
        startDir = endDir

        vehicleControllerAPI.GetBrake0Attr().Set(1.0)
        vehicleControllerAPI.GetBrake1Attr().Set(1.0)
        vehicleTankControllerAPI.GetThrust0Attr().Set(0.0)
        vehicleTankControllerAPI.GetThrust1Attr().Set(0.0)

        secondsToRun = 0.3
        self._simulate(secondsToRun)
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        startPos = vehiclePrim.GetAttribute("xformOp:translate").Get()
        startDir = self._local_dir_to_world(refDir, xformable)

        vehicleControllerAPI.GetBrake0Attr().Set(0.0)
        vehicleControllerAPI.GetBrake1Attr().Set(0.0)
        vehicleTankControllerAPI.GetThrust0Attr().Set(1.0)
        vehicleTankControllerAPI.GetThrust1Attr().Set(1.0)

        secondsToRun = 1.0
        self._simulate(secondsToRun)
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        endPos = vehiclePrim.GetAttribute("xformOp:translate").Get()
        endDir = self._local_dir_to_world(refDir, xformable)

        deltaPos = endPos - startPos
        deltaPosMagn = deltaPos.GetLength()
        deltaPosProjMagn = endDir.GetDot(deltaPos)

        # make sure the vehicle moved forward along the direction achieved after braking
        self.assertGreater(deltaPosProjMagn, deltaPosMagn * 0.95)

    #
    # test misconfiguration of having no scene with PhysxVehicleContextAPI applied or
    # no scene prim at all. Should not crash and send error message.
    #
    async def test_no_vehiclecontext_or_no_scene(self):

        for i in range(2):
            stage = await self.new_stage()

            vehicleCount = 1
            Factory.create4WheeledCarsScenario(
                stage,
                1.0,
                vehicleCount,
                createCollisionShapesForWheels=False,
                driveMode=Factory.DRIVE_BASIC
            )

            stage = self.get_stage()
            rootPath = str(stage.GetDefaultPrim().GetPath())
            scenePath = rootPath + FACTORY_SCENE_SUBPATH

            if (i == 0):
                stage.RemovePrim(scenePath)
            else:
                scenePrim = stage.GetPrimAtPath(scenePath)
                scenePrim.RemoveAPI(PhysxSchema.PhysxVehicleContextAPI)

            message = ("PhysX Vehicle: vehicle is being created but the stage has no matching physics scene prim with "
                "PhysxVehicleContextAPI applied. Default values are being used but might not match the required setup for the vehicle.\n")

            with omni.physxtests.utils.ExpectMessage(self, message):
                self._prepare_for_simulation()

            self._simulate_one_frame()

    #
    # test misconfiguration of having rigidBodyEnabled=false or (kinematicEnabled=true & vehicleEnabled=true).
    # Should not crash and send error message.
    #
    async def test_invalid_rigidbody_setup(self):

        for j in range(2):
            testAtRuntime = (j == 0)

            for i in range(2):
                stage = await self.new_stage()

                vehicleCount = 1
                vehiclePaths = []
                Factory.create4WheeledCarsScenario(
                    stage,
                    1.0,
                    vehicleCount,
                    createCollisionShapesForWheels=False,
                    driveMode=Factory.DRIVE_BASIC,
                    vehiclePathsOut=vehiclePaths,
                )

                stage = self.get_stage()
                vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])
                rigidBodyAPI = UsdPhysics.RigidBodyAPI(vehiclePrim)

                if (testAtRuntime):
                    self._prepare_for_simulation()

                    if (i == 0):
                        message0 = (f"Setting rigidBodyEnabled to false is not supported if the rigid body is part of a vehicle. Body: {vehiclePaths[0]}")

                        with omni.physxtests.utils.ExpectMessage(self, message0):
                            rigidBodyAPI.GetRigidBodyEnabledAttr().Set(False)
                    else:
                        message0 = ("Setting kinematicEnabled to true on a rigid body that is part of an enabled vehicle is illegal. "
                            "Please disable the vehicle first (see vehicleEnabled on the PhysxVehicleAPI USD API schema). The vehicle will "
                            f"be disabled internally now. Body: {vehiclePaths[0]}")

                        with omni.physxtests.utils.ExpectMessage(self, message0):
                            rigidBodyAPI.GetKinematicEnabledAttr().Set(True)
                else:
                    if (i == 0):
                        rigidBodyAPI.GetRigidBodyEnabledAttr().Set(False)

                        message0 = (f"Usd Physics: vehicle \"{vehiclePrim.GetName()}\": the attribute \"rigidBodyEnabled\" of RigidBodyAPI is false "
                            "which is incompatible with vehicles.\n")
                    else:
                        rigidBodyAPI.GetKinematicEnabledAttr().Set(True)

                        message0 = (f"Usd Physics: vehicle \"{vehiclePrim.GetName()}\": the attribute \"kinematicEnabled\" of RigidBodyAPI is true. "
                            "This is only supported if the vehicle simulation is disabled (see attribute vehicleEnabled).\n")

                    message1 = f"PhysX Vehicle: internal vehicle controller object for vehicle \"{vehiclePrim.GetPath().pathString}\" could not be found.\n"

                    with omni.physxtests.utils.ExpectMessage(self, [message0, message1]):
                        self._prepare_for_simulation()

                self._simulate_one_frame()

                if (testAtRuntime):
                    # setting the attribute back to the expected value should not trigger an error message

                    if (i == 0):
                        rigidBodyAPI.GetRigidBodyEnabledAttr().Set(True)
                    else:
                        rigidBodyAPI.GetKinematicEnabledAttr().Set(False)

                    self._simulate_one_frame()

    #
    # test the custom metadata physxVehicle:referenceFrameIsCenterOfMass.
    #
    async def test_reference_frame_is_com(self):

        stage = await self.new_stage()

        vehicleCount = 2
        vehiclePaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_NONE,
            vehiclePathsOut=vehiclePaths,
            referenceFrameIsCoMList=[True, False]
        )

        stage = self.get_stage()

        vehiclePrimList = []
        startPosList = []

        for i in range(vehicleCount):
            vehiclePrim = stage.GetPrimAtPath(vehiclePaths[i])
            vehiclePrimList.append(vehiclePrim)
            startPosList.append(vehiclePrim.GetAttribute("xformOp:translate").Get())

        self.assertTrue(get_custom_metadata(vehiclePrimList[0], PhysxSchema.Tokens.referenceFrameIsCenterOfMass))
        self.assertFalse(get_custom_metadata(vehiclePrimList[1], PhysxSchema.Tokens.referenceFrameIsCenterOfMass))

        # the first vehicle was configured for center of mass being the reference frame. Switching
        # referenceFrameIsCenterOfMass to False should result in the vehicle moving down a bit
        set_custom_metadata(vehiclePrimList[0], PhysxSchema.Tokens.referenceFrameIsCenterOfMass, False)

        secondsToRun = 0.5
        self._simulate_with_prep(secondsToRun)
        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        errTol = 0.01
        deltaPosList = []
        for i in range(vehicleCount):
            deltaPos = vehiclePrimList[i].GetAttribute("xformOp:translate").Get() - startPosList[i]
            deltaPosList.append(deltaPos)

            # make sure things were stable on these axes
            self.assertAlmostEqual(deltaPos[0], 0.0, delta=errTol)
            self.assertAlmostEqual(deltaPos[2], 0.0, delta=errTol)

        self.assertAlmostEqual(deltaPosList[1][1], 0.0, delta=errTol)

        # the first vehicle should move down a bit
        self.assertLess(deltaPosList[0][1], -0.1)

    #
    # test that the wheel index order does not need to match the order of the wheel attachment prims
    #
    async def test_wheel_index_shuffle(self):
        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_NONE,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
        )

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])

        wheelAttIndexList = [2, 3, 0, 1]

        for i in range(4):
            wheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][i])
            wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(wheelAttPrim)

            wheelAttAPI.GetIndexAttr().Set(wheelAttIndexList[i])

            suspPosAttr = wheelAttAPI.GetSuspensionFramePositionAttr()
            p = suspPosAttr.Get()
            if (i == 0):
                wheelAtt0Prim = wheelAttPrim
                wheelAtt0PosX = p[0] + 1.0
                p[0] = wheelAtt0PosX
            elif (i == 1):
                wheelAtt1Prim = wheelAttPrim
                wheelAtt1PosX = p[0] - 1.0
                p[0] = wheelAtt1PosX
            suspPosAttr.Set(p)

        self._simulate_one_frame_with_prep()

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        # make sure the adjusted positions map to the correct prims even if the wheel indices changed

        pos = wheelAtt0Prim.GetAttribute("xformOp:translate").Get()
        self.assertGreater(pos[0], 0.95 * wheelAtt0PosX)

        pos = wheelAtt1Prim.GetAttribute("xformOp:translate").Get()
        self.assertLess(pos[0], 0.95 * wheelAtt1PosX)

    #
    # test that limiting the suspension expansion velocity has the desired effect
    #
    async def test_limit_suspension_expansion_velocity(self):
        stage = await self.new_stage()

        vehicleCount = 2
        vehiclePaths = []
        wheelAttachmentPaths = []

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_NONE,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
        )

        vehiclePrimList = []
        vehiclePrimList.append(stage.GetPrimAtPath(vehiclePaths[0]))
        vehiclePrimList.append(stage.GetPrimAtPath(vehiclePaths[1]))

        vehicleAPI = PhysxSchema.PhysxVehicleAPI(vehiclePrimList[1])
        vehicleAPI.GetLimitSuspensionExpansionVelocityAttr().Set(True)

        for vp in vehiclePrimList:
            rigidBodyAPI = PhysxSchema.PhysxRigidBodyAPI(vp)
            rigidBodyAPI.GetSleepThresholdAttr().Set(0)
            rigidBodyAPI.GetStabilizationThresholdAttr().Set(0)

        massAPI = UsdPhysics.MassAPI(vehiclePrimList[1])
        mass = massAPI.GetMassAttr().Get()
        sprungMass = mass / 4.0

        self._prepare_for_simulation()

        # run a few simulation steps to settle down. It's also crucial to run at least one step
        # or else the previous jounce value will be unknown and set to the current jounce. As
        # a consequence, the logic to limit suspension expansion velocity would be omitted since
        # velocity is zero (the assumption is that the initial jounce state also reflects the
        # previous state).
        self._simulate(0.5)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        suspensionFramePosList = []
        suspensionTravelDistList = []
        suspensionStartJounceList = []
        for i in range(len(wheelAttachmentPaths[0])):
            v0WheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][i])
            v1WheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[1][i])

            v0SuspensionAPI = self._get_suspension(v0WheelAttPrim, stage)
            v1SuspensionAPI = self._get_suspension(v1WheelAttPrim, stage)

            travelDist = v0SuspensionAPI.GetTravelDistanceAttr().Get()
            suspensionTravelDistList.append(travelDist)

            wheelAttAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI(v0WheelAttPrim)
            suspFramePos = wheelAttAPI.GetSuspensionFramePositionAttr().Get()
            suspensionFramePosList.append(suspFramePos)

            pos = self._get_xform_pos(v0WheelAttPrim)
            jounce = travelDist + (pos[1] - suspFramePos[1])
            suspensionStartJounceList.append(jounce)

            # rough estimate for stiffness and damping such that it takes a few sim steps
            # to fully expand
            wheelAPI = self._get_wheel(v0WheelAttPrim, stage)
            wheelMass = wheelAPI.GetMassAttr().Get()
            stepCountToFullExpansion = 10
            timeToFullExpansion = stepCountToFullExpansion * self._get_time_step()
            accToFullExpansion = 2.0 * jounce / (timeToFullExpansion * timeToFullExpansion)
            stiffness = (wheelMass * accToFullExpansion) / jounce
            dampingRatio = 0.25
            damping = dampingRatio * 2.0 * math.sqrt(stiffness * sprungMass)

            v0SuspensionAPI.GetSpringStrengthAttr().Set(stiffness)
            v1SuspensionAPI.GetSpringStrengthAttr().Set(stiffness)

            v0SuspensionAPI.GetSpringDamperRateAttr().Set(damping)
            v1SuspensionAPI.GetSpringDamperRateAttr().Set(damping)

        self._remove_factory_ground_plane_and_set_gravity_zero()

        self._simulate_one_frame()

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        for i in range(len(wheelAttachmentPaths[0])):
            v0WheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][i])
            v1WheelAttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[1][i])

            # without limiting the suspension expansion velocity, the suspension should go to
            # full expansion in one step. Else, stiffness and damping decide how long it takes.

            pos0 = self._get_xform_pos(v0WheelAttPrim)
            jounce0 = suspensionTravelDistList[i] + (pos0[1] - suspensionFramePosList[i][1])
            self.assertAlmostEqual(jounce0, 0.0, delta=0.001)

            pos1 = self._get_xform_pos(v1WheelAttPrim)
            jounce1 = suspensionTravelDistList[i] + (pos1[1] - suspensionFramePosList[i][1])
            # we run a single step while it would take at least stepCountToFullExpansion steps
            # to get to 0 jounce. Since damping etc. is ignored, that seems like an OK rough
            # estimate for a conservative bound for the expected jounce
            fraction = (stepCountToFullExpansion - 1) / float(stepCountToFullExpansion)
            lowerBound = suspensionStartJounceList[i] * fraction
            self.assertGreater(jounce1, lowerBound)

    #
    # test nonlinear command response for accelerator command with basic drive vehicle
    #
    async def test_nonlinear_command_response_drive(self):
        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
        )

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])

        driveBasicAPI = self._get_drive(vehiclePrim, stage)
        nonlinCmdRespAPI = PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI.Apply(driveBasicAPI.GetPrim(), PhysxSchema.Tokens.drive)

        cmdValues = []
        speedResponsesPerCommandValue = []
        speedResponses = []

        # apply no drive torque when accelerator equal cmdValue0 or below
        cmdValue0 = 0.2
        cmdValue0SpeedResponses = [Gf.Vec2f(0.0, 0.0)]

        cmdValues.append(cmdValue0)
        speedResponsesPerCommandValue.append(len(speedResponses))
        speedResponses.extend(cmdValue0SpeedResponses)

        # apply full drive torque when accelerator above cmdValue1 and speed below sp0
        # but apply no torque when speed above sp1
        sp0 = 0.5
        sp1 = 2.0
        cmdValue1 = 0.21
        cmdValue1SpeedResponses = [Gf.Vec2f(sp0, 1.0), Gf.Vec2f(sp1, 0.0)]

        cmdValues.append(cmdValue1)
        speedResponsesPerCommandValue.append(len(speedResponses))
        speedResponses.extend(cmdValue1SpeedResponses)

        nonlinCmdRespAPI.GetCommandValuesAttr().Set(cmdValues)
        nonlinCmdRespAPI.GetSpeedResponsesPerCommandValueAttr().Set(speedResponsesPerCommandValue)
        nonlinCmdRespAPI.GetSpeedResponsesAttr().Set(speedResponses)

        vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)
        accAttr = vehicleControllerAPI.GetAcceleratorAttr()

        rigidBodyAPI = UsdPhysics.RigidBodyAPI(vehiclePrim)
        velAttr = rigidBodyAPI.GetVelocityAttr()

        global gPhysXInterface

        self._prepare_for_simulation()

        forwardDir = Gf.Vec3f(0, 0, 1)

        startPos = self._get_xform_pos(vehiclePrim)
        startOrient = self._get_xform_orient(vehiclePrim)

        def resetVehicle():
            self._set_xform_pos(vehiclePrim, startPos)
            self._set_xform_orient(vehiclePrim, startOrient)

            gPhysXInterface.set_vehicle_to_rest_state(vehiclePaths[0])

        #
        # zero velocity and accelerator below cmdValue0 => no torque => expect no movement
        #

        accAttr.Set(0.9 * cmdValue0)

        self._simulate(0.5)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=False)

        endPos = self._get_xform_pos(vehiclePrim)
        delta = endPos - startPos
        self.assertAlmostEqual(delta[2], 0.0, delta=0.001)

        #
        # non-zero velocity but accelerator below cmdValue0 => no torque => speed should not increase
        #

        resetVehicle()

        accAttr.Set(0.9 * cmdValue0)

        startVel = sp0 * 0.9

        self._set_vehicle_speed(stage, vehiclePrim, wheelAttachmentPaths[0], startVel,
            forwardDir, gPhysXInterface)

        self._simulate(0.1)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=True)

        endVel = velAttr.Get()[2]
        self.assertLessEqual(endVel, startVel)

        #
        # accelerator above cmdValue1 and speed below sp0 => torque => speed should increase
        #

        resetVehicle()

        accAttr.Set(1.1 * cmdValue1)

        startVel = sp0 * 0.5

        self._set_vehicle_speed(stage, vehiclePrim, wheelAttachmentPaths[0], startVel,
            forwardDir, gPhysXInterface)

        self._simulate(0.1)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=True)

        endVel = velAttr.Get()[2]
        self.assertGreater(endVel, startVel)

        #
        # accelerator above cmdValue1 and speed above sp1 => no torque => speed should not increase
        #

        resetVehicle()

        accAttr.Set(1.1 * cmdValue1)

        startVel = sp1 * 1.5

        self._set_vehicle_speed(stage, vehiclePrim, wheelAttachmentPaths[0], startVel,
            forwardDir, gPhysXInterface)

        self._simulate(0.1)

        self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=True)

        endVel = velAttr.Get()[2]
        self.assertLessEqual(endVel, startVel)

    #
    # test nonlinear command response for brake command
    #
    async def test_nonlinear_command_response_brake(self):
        stage = await self.new_stage()

        vehicleCount = 2
        vehiclePaths = []
        wheelAttachmentPaths = []

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
        )

        vehiclePrimList = []
        brake0AttrList = []
        brake1AttrList = []
        velocityAttrList = []
        startPosList = []
        startOrientList = []
        for p in vehiclePaths:
            vehiclePrim = stage.GetPrimAtPath(p)
            vehiclePrimList.append(vehiclePrim)

            vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)

            brake0AttrList.append(vehicleControllerAPI.GetBrake0Attr())
            brake1AttrList.append(vehicleControllerAPI.GetBrake1Attr())

            rigidBodyAPI = UsdPhysics.RigidBodyAPI(vehiclePrim)
            velocityAttrList.append(rigidBodyAPI.GetVelocityAttr())

            startPosList.append(self._get_xform_pos(vehiclePrim))
            startOrientList.append(self._get_xform_orient(vehiclePrim))

        nonlinCmdRespAPI0 = PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI.Apply(vehiclePrimList[1], PhysxSchema.Tokens.brakes0)
        nonlinCmdRespAPI1 = PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI.Apply(vehiclePrimList[1], PhysxSchema.Tokens.brakes1)

        cmdValues = []
        speedResponsesPerCommandValue = []
        speedResponses = []

        # apply no brake torque when brake control equal cmdValue0 or below
        cmdValue0 = 0.5
        cmdValue0SpeedResponses = [Gf.Vec2f(0.0, 0.0)]

        cmdValues.append(cmdValue0)
        speedResponsesPerCommandValue.append(len(speedResponses))
        speedResponses.extend(cmdValue0SpeedResponses)

        # apply full brake torque when brake control above cmdValue1 and speed below sp0
        # but apply no torque when speed above sp1
        sp0 = 0.5
        sp1 = 2.0
        cmdValue1 = 0.51
        cmdValue1SpeedResponses = [Gf.Vec2f(sp0, 1.0), Gf.Vec2f(sp1, 0.0)]

        cmdValues.append(cmdValue1)
        speedResponsesPerCommandValue.append(len(speedResponses))
        speedResponses.extend(cmdValue1SpeedResponses)

        nonlinCmdRespAPI0.GetCommandValuesAttr().Set(cmdValues)
        nonlinCmdRespAPI0.GetSpeedResponsesPerCommandValueAttr().Set(speedResponsesPerCommandValue)
        nonlinCmdRespAPI0.GetSpeedResponsesAttr().Set(speedResponses)

        nonlinCmdRespAPI1.GetCommandValuesAttr().Set(cmdValues)
        nonlinCmdRespAPI1.GetSpeedResponsesPerCommandValueAttr().Set(speedResponsesPerCommandValue)
        nonlinCmdRespAPI1.GetSpeedResponsesAttr().Set(speedResponses)

        global gPhysXInterface

        self._prepare_for_simulation()

        forwardDir = Gf.Vec3f(0, 0, 1)

        def resetVehicle():
            for i in range(len(vehiclePrimList)):
                self._set_xform_pos(vehiclePrimList[i], startPosList[i])
                self._set_xform_orient(vehiclePrimList[i], startOrientList[i])

                gPhysXInterface.set_vehicle_to_rest_state(vehiclePaths[i])

        def runScenario(commandValue, startVel, useBrake0):

            resetVehicle()

            if (useBrake0):
                brake0AttrList[0].Set(val)
                brake1AttrList[0].Set(0)

                brake0AttrList[1].Set(val)
                brake1AttrList[1].Set(0)
            else:
                brake0AttrList[0].Set(0)
                brake1AttrList[0].Set(val)

                brake0AttrList[1].Set(0)
                brake1AttrList[1].Set(val)

            self._set_vehicle_speed(stage, vehiclePrimList[0], wheelAttachmentPaths[0], startVel,
                forwardDir, gPhysXInterface)

            self._set_vehicle_speed(stage, vehiclePrimList[1], wheelAttachmentPaths[1], startVel,
                forwardDir, gPhysXInterface)

            self._simulate(0.1)

            self._write_back_transforms(updateToFastCache=False, updateToUsd=True, updateVelocitiesToUsd=True)

            return [velocityAttrList[0].Get()[2], velocityAttrList[1].Get()[2]]

        #
        # non-zero velocity and brake control below cmdValue0 => no torque
        # => speed should be larger than for reference vehicle
        #

        val = 0.9 * cmdValue0
        startVel = sp0 * 0.9

        for b in range(2):
            endVelList = runScenario(val, startVel, (b == 0))

            self.assertGreater(endVelList[1], endVelList[0] * 1.1)

        #
        # brake control above cmdValue1 and speed below sp0 => torque
        # => speed should match reference vehicle
        #

        val = 1.1 * cmdValue1
        startVel = sp0 * 0.9

        for b in range(2):
            endVelList = runScenario(val, startVel, (b == 0))

            self.assertAlmostEqual(endVelList[0], endVelList[1], delta=endVelList[0] * 0.01)

        #
        # brake control above cmdValue1 and speed above sp1 => no torque
        # => speed should be larger than for reference vehicle
        #

        val = 1.1 * cmdValue1
        startVel = sp1 * 1.5

        for b in range(2):
            endVelList = runScenario(val, startVel, (b == 0))

            self.assertGreater(endVelList[1], endVelList[0] * 1.1)

    #
    # test nonlinear command response for steer command
    #
    async def test_nonlinear_command_response_steer(self):
        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        wheelAttachmentPaths = []

        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
            wheelAttachmentPathsOut=wheelAttachmentPaths,
        )

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])
        wheel0AttPrim = stage.GetPrimAtPath(wheelAttachmentPaths[0][0])

        nonlinCmdRespAPI = PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI.Apply(vehiclePrim, PhysxSchema.Tokens.steer)

        cmdValues = []
        speedResponsesPerCommandValue = []
        speedResponses = []

        # apply no steer when steer control (absolute value) equal cmdValue0 or below
        cmdValue0 = 0.2
        cmdValue0SpeedResponses = [Gf.Vec2f(0.0, 0.0)]

        cmdValues.append(cmdValue0)
        speedResponsesPerCommandValue.append(len(speedResponses))
        speedResponses.extend(cmdValue0SpeedResponses)

        # apply steer when steer control (absolute value) above cmdValue1 and speed below sp0
        # but apply no steer when speed above sp1
        sp0 = 0.5
        sp1 = 2.0
        cmdValue1 = 0.21
        cmdValue1SpeedResponses = [Gf.Vec2f(sp0, cmdValue1), Gf.Vec2f(sp1, 0.0)]

        cmdValues.append(cmdValue1)
        speedResponsesPerCommandValue.append(len(speedResponses))
        speedResponses.extend(cmdValue1SpeedResponses)

        nonlinCmdRespAPI.GetCommandValuesAttr().Set(cmdValues)
        nonlinCmdRespAPI.GetSpeedResponsesPerCommandValueAttr().Set(speedResponsesPerCommandValue)
        nonlinCmdRespAPI.GetSpeedResponsesAttr().Set(speedResponses)

        vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI(vehiclePrim)
        steerAttr = vehicleControllerAPI.GetSteerAttr()

        steeringAPI = PhysxSchema.PhysxVehicleSteeringAPI(vehiclePrim)
        maxSteerAngle = steeringAPI.GetMaxSteerAngleAttr().Get()

        global gPhysXInterface

        self._prepare_for_simulation()

        forwardDir = Gf.Vec3f(0, 0, 1)

        startPos = self._get_xform_pos(vehiclePrim)
        startOrient = self._get_xform_orient(vehiclePrim)

        errTol = 0.0001

        def resetVehicle():
            self._set_xform_pos(vehiclePrim, startPos)
            self._set_xform_orient(vehiclePrim, startOrient)

            gPhysXInterface.set_vehicle_to_rest_state(vehiclePaths[0])

        #
        # zero velocity and steer (absolute value) below cmdValue0 => no steer
        #

        resetVehicle()

        steerAttr.Set(-0.9 * cmdValue0)

        self._simulate_one_frame()

        wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[0][0])
        self.assertAlmostEqual(wheelState[VEHICLE_WHEEL_STATE_STEER_ANGLE], 0.0, delta=errTol)

        #
        # non-zero velocity but steer (absolute value) below cmdValue0 => no steer
        #

        resetVehicle()

        steerAttr.Set(-0.9 * cmdValue0)

        startVel = sp0 * 0.9

        self._set_vehicle_speed(stage, vehiclePrim, wheelAttachmentPaths[0], startVel,
            forwardDir, gPhysXInterface)

        self._simulate_one_frame()

        wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[0][0])
        self.assertAlmostEqual(wheelState[VEHICLE_WHEEL_STATE_STEER_ANGLE], 0.0, delta=errTol)

        #
        # steer (absolute value) above cmdValue1 and speed below sp0 => steer
        #

        resetVehicle()

        steerAttr.Set(-1.1 * cmdValue1)

        startVel = sp0 * 0.5

        self._set_vehicle_speed(stage, vehiclePrim, wheelAttachmentPaths[0], startVel,
            forwardDir, gPhysXInterface)

        self._simulate_one_frame()

        wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[0][0])
        self.assertAlmostEqual(wheelState[VEHICLE_WHEEL_STATE_STEER_ANGLE], -cmdValue1 * maxSteerAngle, delta=errTol)

        #
        # steer (absolute value) above cmdValue1 and speed above sp1 => no steer
        #

        resetVehicle()

        steerAttr.Set(-1.1 * cmdValue1)

        startVel = sp1 * 1.5

        self._set_vehicle_speed(stage, vehiclePrim, wheelAttachmentPaths[0], startVel,
            forwardDir, gPhysXInterface)

        self._simulate_one_frame()

        wheelState = gPhysXInterface.get_wheel_state(wheelAttachmentPaths[0][0])
        self.assertAlmostEqual(wheelState[VEHICLE_WHEEL_STATE_STEER_ANGLE], 0.0, delta=errTol)

    #
    # test that applying the PhysxVehicleAckermannSteeringAPI and changing related parameters
    # after the sim has started will not crash. Check that an error message is sent.
    #
    async def test_dynamic_add_ackermann_steering_api(self):

        stage = await self.new_stage()

        vehicleCount = 1
        vehiclePaths = []
        Factory.create4WheeledCarsScenario(
            stage,
            1.0,
            vehicleCount,
            createCollisionShapesForWheels=False,
            driveMode=Factory.DRIVE_BASIC,
            vehiclePathsOut=vehiclePaths,
        )

        vehiclePrim = stage.GetPrimAtPath(vehiclePaths[0])

        vehiclePrim.RemoveAPI(PhysxSchema.PhysxVehicleSteeringAPI)

        self._prepare_for_simulation()

        ackermannSteeringAPI = PhysxSchema.PhysxVehicleAckermannSteeringAPI.Apply(vehiclePrim)

        sharedMessage = (f" at prim \"{vehiclePaths[0]}\": the vehicle has not been configured for Ackermann steering. "
            "Note that adding the PhysxVehicleAckermannSteeringAPI after the simulation has started is not supported.\n")

        message = "Attribute \"physxVehicleAckermannSteering:maxSteerAngle\"" + sharedMessage

        with omni.physxtests.utils.ExpectMessage(self, message):
            ackermannSteeringAPI.GetMaxSteerAngleAttr().Set(math.pi / 6)

        message = "Attribute \"physxVehicleAckermannSteering:wheelBase\"" + sharedMessage

        with omni.physxtests.utils.ExpectMessage(self, message):
            ackermannSteeringAPI.GetWheelBaseAttr().Set(4.0)

        message = "Attribute \"physxVehicleAckermannSteering:trackWidth\"" + sharedMessage

        with omni.physxtests.utils.ExpectMessage(self, message):
            ackermannSteeringAPI.GetTrackWidthAttr().Set(1.0)

        message = "Attribute \"physxVehicleAckermannSteering:strength\"" + sharedMessage

        with omni.physxtests.utils.ExpectMessage(self, message):
            ackermannSteeringAPI.GetStrengthAttr().Set(0.5)

        self._simulate_one_frame()
