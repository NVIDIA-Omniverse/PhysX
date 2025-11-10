# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
from pxr import Usd, UsdLux, UsdGeom, UsdShade, Sdf, Gf, UsdPhysics
from pxr import PhysxSchema
import omni.physxdemos as demo
from omni.physx.scripts.physicsUtils import *
from omni.physx.scripts.utils import (
    set_custom_metadata
)
from omni.physx.bindings._physx import (
    VEHICLE_AUTOMATIC_TRANSMISSION_GEAR_VALUE
)
from .VehicleSampleBase import VehicleSampleBase

class BasicSetupDriveBasicDemo(VehicleSampleBase):
    title = "Setup (drive basic)"
    category = demo.Categories.VEHICLES
    short_description = "Vehicle setup using basic drive model"
    description = ("Demo showing a basic vehicle setup using the basic drive model (engine torque gets applied to the wheels directly). "
        "The arrow keys can be used to steer, accelerate and brake. "
        "To use a gamepad for controlling the vehicle, make sure to disable Gamepad Camera Control in the Viewport Settings.")

    def create(self, stage):
        super().create(stage)

        create(stage, False)

        self.autofocus = True # autofocus on the scene at first update
        self.autofocus_zoom = 0.28 # Get a bit closer


class BasicSetupDriveStandardDemo(VehicleSampleBase):
    title = "Setup (drive standard)"
    category = demo.Categories.VEHICLES
    short_description = "Vehicle setup using standard drive model"
    description = ("Demo showing a basic vehicle setup using the standard drive model (an engine is connected to the wheels through a clutch and gear system). "
        "The arrow keys can be used to steer, accelerate and brake. "
        "To use a gamepad for controlling the vehicle, make sure to disable Gamepad Camera Control in the Viewport Settings.")

    def create(self, stage):
        super().create(stage)

        create(stage, True)

        self.autofocus = True # autofocus on the scene at first update
        self.autofocus_zoom = 0.28 # Get a bit closer

class PrimPaths:
    def __init__(self):
        self.vehiclePath = None
        self.wheelAttachmentPaths = None


def create(stage, useDriveStandard, configureAsTank = False):
    #
    # configureAsTank: only considered if useDriveStandard is True
    #

    lengthScale = 1.0
    massScale = 1.0
    lengthScaleSqr = lengthScale * lengthScale
    kgmsScale = massScale * lengthScaleSqr
    forceScale = massScale * lengthScale

    createCollisionShapesForWheels = True

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0 / lengthScale)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0 / massScale)
    
    rootPath = str(stage.GetDefaultPrim().GetPath())

    # light
    sphereLight = UsdLux.SphereLight.Define(stage, rootPath + "/SphereLight")
    sphereLight.CreateRadiusAttr(1.5 * lengthScale)
    sphereLight.CreateIntensityAttr(30000)
    sphereLight.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(6.5, 11.5, 0) * lengthScale)

    # physics scene
    gravityMagnitude = 10 * lengthScale
    scene = UsdPhysics.Scene.Define(stage, rootPath + "/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, -1, 0))
    scene.CreateGravityMagnitudeAttr(gravityMagnitude)

    # vehicle context
    vehicleContextAPI = PhysxSchema.PhysxVehicleContextAPI.Apply(scene.GetPrim())
    vehicleContextAPI.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
    vehicleContextAPI.CreateVerticalAxisAttr(PhysxSchema.Tokens.posY)
    vehicleContextAPI.CreateLongitudinalAxisAttr(PhysxSchema.Tokens.posZ)

    # materials
    tarmacMaterialPath = rootPath + "/TarmacMaterial"
    UsdShade.Material.Define(stage, tarmacMaterialPath)
    tarmacMaterial = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(tarmacMaterialPath))
    tarmacMaterial.CreateStaticFrictionAttr(0.9)
    tarmacMaterial.CreateDynamicFrictionAttr(0.7)
    tarmacMaterial.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(tarmacMaterial.GetPrim())

    gravelMaterialPath = rootPath + "/GravelMaterial"
    UsdShade.Material.Define(stage, gravelMaterialPath)
    gravelMaterial = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(gravelMaterialPath))
    gravelMaterial.CreateStaticFrictionAttr(0.6)
    gravelMaterial.CreateDynamicFrictionAttr(0.6)
    gravelMaterial.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(gravelMaterial.GetPrim())

    # tire friction tables
    winterTireFrictionTablePath = rootPath + "/WinterTireFrictionTable"
    winterTireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable.Define(stage, winterTireFrictionTablePath)
    winterTireFrictionTableGroundMaterialsRel = winterTireFrictionTable.CreateGroundMaterialsRel()
    winterTireFrictionTableGroundMaterialsRel.AddTarget(tarmacMaterialPath)
    winterTireFrictionTableGroundMaterialsRel.AddTarget(gravelMaterialPath)
    winterTireFrictionTable.CreateFrictionValuesAttr([0.75, 0.6])

    summerTireFrictionTablePath = rootPath + "/SummerTireFrictionTable"
    summerTireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable.Define(stage, summerTireFrictionTablePath)
    summerTireFrictionTableGroundMaterialsRel = summerTireFrictionTable.CreateGroundMaterialsRel()
    summerTireFrictionTableGroundMaterialsRel.AddTarget(tarmacMaterialPath)
    summerTireFrictionTableGroundMaterialsRel.AddTarget(gravelMaterialPath)
    summerTireFrictionTable.CreateFrictionValuesAttr([0.7, 0.6])

    # collision groups
    collisionGroupVehicleChassisPath = rootPath + "/VehicleChassisCollisionGroup"
    collisionGroupVehicleWheelPath = rootPath + "/VehicleWheelCollisionGroup"
    collisionGroupVehicleGroundQueryPath = rootPath + "/VehicleGroundQueryGroup"
    collisionGroupGroundSurfacePath = rootPath + "/GroundSurfaceCollisionGroup"

    collisionGroupVehicleChassis = UsdPhysics.CollisionGroup.Define(stage, collisionGroupVehicleChassisPath)
    collisionGroupVehicleChassisRel = collisionGroupVehicleChassis.CreateFilteredGroupsRel()
    collisionGroupVehicleChassisRel.AddTarget(collisionGroupVehicleGroundQueryPath)

    collisionGroupVehicleWheel = UsdPhysics.CollisionGroup.Define(stage, collisionGroupVehicleWheelPath)
    collisionGroupVehicleWheelRel = collisionGroupVehicleWheel.CreateFilteredGroupsRel()
    collisionGroupVehicleWheelRel.AddTarget(collisionGroupVehicleGroundQueryPath)
    collisionGroupVehicleWheelRel.AddTarget(collisionGroupGroundSurfacePath)

    collisionGroupVehicleGroundQuery = UsdPhysics.CollisionGroup.Define(stage, collisionGroupVehicleGroundQueryPath)
    collisionGroupVehicleGroundQueryRel = collisionGroupVehicleGroundQuery.CreateFilteredGroupsRel()
    collisionGroupVehicleGroundQueryRel.AddTarget(collisionGroupVehicleChassisPath)
    collisionGroupVehicleGroundQueryRel.AddTarget(collisionGroupVehicleWheelPath)

    collisionGroupGroundSurface = UsdPhysics.CollisionGroup.Define(stage, collisionGroupGroundSurfacePath)
    collisionGroupGroundSurfaceRel = collisionGroupGroundSurface.CreateFilteredGroupsRel()
    collisionGroupGroundSurfaceRel.AddTarget(collisionGroupGroundSurfacePath)
    collisionGroupGroundSurfaceRel.AddTarget(collisionGroupVehicleWheelPath)

    # wheels
    wheelRadius = 0.35 * lengthScale
    wheelWidth = 0.15 * lengthScale

    # vehicle and corresponding rigid body
    vehiclePath = rootPath + "/Vehicle"
    vehicle = UsdGeom.Xform.Define(stage, vehiclePath)
    vehicle.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(0, 3, 0) * lengthScale)
    vehicle.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Quatf(1, 0, 0, 0))

    vehiclePrim = vehicle.GetPrim()

    UsdPhysics.RigidBodyAPI.Apply(vehiclePrim)

    vehicleMassBoxDim = Gf.Vec3f(1.8, 1.0, 4.8) * lengthScale
    centerOfMassToGround = 0.75 * lengthScale
    chassisHalfHeight = 0.7 * lengthScale
    chassisDistToGround = 0.3 * lengthScale
    chassisCenterToGround = chassisHalfHeight + chassisDistToGround
    mass = 1800 * massScale
    massAPI = UsdPhysics.MassAPI.Apply(vehiclePrim)
    massAPI.CreateMassAttr(mass)
    centerOfMassOffset = Gf.Vec3f(0, centerOfMassToGround - chassisCenterToGround, 0)
    massAPI.CreateCenterOfMassAttr(centerOfMassOffset)
    massAPI.CreateDiagonalInertiaAttr(
        Gf.Vec3f(
            (vehicleMassBoxDim[1] * vehicleMassBoxDim[1]) + (vehicleMassBoxDim[2] * vehicleMassBoxDim[2]),
            (vehicleMassBoxDim[0] * vehicleMassBoxDim[0]) + (vehicleMassBoxDim[2] * vehicleMassBoxDim[2]),
            (vehicleMassBoxDim[0] * vehicleMassBoxDim[0]) + (vehicleMassBoxDim[1] * vehicleMassBoxDim[1]),
        )
        * (1 / 12)
        * mass
    )
    massAPI.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))

    rigidBodyAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(vehiclePrim)
    rigidBodyAPI.CreateDisableGravityAttr(True)

    vehicleAPI = PhysxSchema.PhysxVehicleAPI.Apply(vehiclePrim)
    vehicleAPI.CreateVehicleEnabledAttr(True)
    vehicleAPI.CreateSubStepThresholdLongitudinalSpeedAttr(5.0 * lengthScale)
    vehicleAPI.CreateLowForwardSpeedSubStepCountAttr(3)
    vehicleAPI.CreateHighForwardSpeedSubStepCountAttr(1)
    vehicleAPI.CreateMinPassiveLongitudinalSlipDenominatorAttr(4.0 * lengthScale)
    vehicleAPI.CreateMinActiveLongitudinalSlipDenominatorAttr(0.1 * lengthScale)
    vehicleAPI.CreateMinLateralSlipDenominatorAttr(1.0 * lengthScale)
    set_custom_metadata(vehiclePrim, PhysxSchema.Tokens.referenceFrameIsCenterOfMass, False)

    defaultMaxBrakeTorque = 3600 * kgmsScale
    if (useDriveStandard and configureAsTank):
        # set up one brake configuration for the wheels of the left track
        brakes0API = PhysxSchema.PhysxVehicleBrakesAPI.Apply(vehiclePrim, PhysxSchema.Tokens.brakes0)
        brakes0API.CreateWheelsAttr([0, 2])
        brakes0API.CreateMaxBrakeTorqueAttr(defaultMaxBrakeTorque)

        # set up one brake configuration for the wheels of the right track
        brakes1API = PhysxSchema.PhysxVehicleBrakesAPI.Apply(vehiclePrim, PhysxSchema.Tokens.brakes1)
        brakes1API.CreateWheelsAttr([1, 3])
        brakes1API.CreateMaxBrakeTorqueAttr(defaultMaxBrakeTorque)

        tankDifferentialAPI = PhysxSchema.PhysxVehicleTankDifferentialAPI.Apply(vehiclePrim)
        differentialAPI = PhysxSchema.PhysxVehicleMultiWheelDifferentialAPI(tankDifferentialAPI)  # tank differential also applies multi wheel differential
        differentialAPI.CreateWheelsAttr([0, 1, 2, 3])
        differentialAPI.CreateTorqueRatiosAttr([0.25, 0.25, 0.25, 0.25])
    else:
        # set up one brake configuration that applies to all wheels
        brakes0API = PhysxSchema.PhysxVehicleBrakesAPI.Apply(vehiclePrim, PhysxSchema.Tokens.brakes0)
        brakes0API.CreateMaxBrakeTorqueAttr(defaultMaxBrakeTorque)

        # set up a handbrake configuration that applies to the rear wheels only
        brakes1API = PhysxSchema.PhysxVehicleBrakesAPI.Apply(vehiclePrim, PhysxSchema.Tokens.brakes1)
        brakes1API.CreateWheelsAttr([2, 3])
        brakes1API.CreateMaxBrakeTorqueAttr(3000 * kgmsScale)

        # set up a steering configuration that applies to the front wheels
        steeringAPI = PhysxSchema.PhysxVehicleSteeringAPI.Apply(vehiclePrim)
        steeringAPI.CreateWheelsAttr([0, 1])
        steeringAPI.CreateMaxSteerAngleAttr(0.554264)

        differentialAPI = PhysxSchema.PhysxVehicleMultiWheelDifferentialAPI.Apply(vehiclePrim)
        differentialAPI.CreateWheelsAttr([0, 1])
        differentialAPI.CreateTorqueRatiosAttr([0.5, 0.5])

    if useDriveStandard:
        if (configureAsTank):
            differentialAPI.CreateAverageWheelSpeedRatiosAttr([0.25, 0.25, 0.25, 0.25])

            # both tracks have two wheels
            tankDifferentialAPI.CreateNumberOfWheelsPerTrackAttr([2, 2])

            # track 0 uses thrust control 0 and track 1 uses thrust control 1
            # (see CreateThrust0Attr())
            tankDifferentialAPI.CreateThrustIndexPerTrackAttr([0, 1])

            # wheels of track 0 start at index 0, wheels of track 1 start at index 2
            # (with respect to WheelIndicesInTrackOrder list further below)
            tankDifferentialAPI.CreateTrackToWheelIndicesAttr([0, 2])

            # track 0 has front left and rear left wheel assigned,
            # track 1 has front right and rear right wheel assigned
            # (the indices refer to the wheel attachment indices, see CreateIndexAttr())
            tankDifferentialAPI.CreateWheelIndicesInTrackOrderAttr([0, 2, 1, 3])
        else:
            differentialAPI.CreateAverageWheelSpeedRatiosAttr([0.5, 0.5])

        PhysxSchema.PhysxVehicleDriveStandardAPI.Apply(vehiclePrim)

        # engine
        engineAPI = PhysxSchema.PhysxVehicleEngineAPI.Apply(vehiclePrim)
        engineAPI.CreateMoiAttr(1.0 * kgmsScale)
        engineAPI.CreatePeakTorqueAttr(1000.0 * kgmsScale)
        engineAPI.CreateMaxRotationSpeedAttr(600.0)
        engineAPI.CreateTorqueCurveAttr([Gf.Vec2f(0.0, 0.8), Gf.Vec2f(0.33, 1.0), Gf.Vec2f(1.0, 0.8)])
        engineAPI.CreateDampingRateFullThrottleAttr(0.15 * kgmsScale)
        engineAPI.CreateDampingRateZeroThrottleClutchEngagedAttr(2.0 * kgmsScale)
        engineAPI.CreateDampingRateZeroThrottleClutchDisengagedAttr(0.35 * kgmsScale)

        # gears
        gearsAPI = PhysxSchema.PhysxVehicleGearsAPI.Apply(vehiclePrim)
        gearsAPI.CreateRatiosAttr([-4.0, 4.0, 2.0, 1.5, 1.1, 1.0])
        gearsAPI.CreateRatioScaleAttr(4.0)
        gearsAPI.CreateSwitchTimeAttr(0.5)

        # auto gear box
        autoGearBoxAPI = PhysxSchema.PhysxVehicleAutoGearBoxAPI.Apply(vehiclePrim)
        autoGearBoxAPI.CreateUpRatiosAttr([0.65, 0.65, 0.65, 0.65])
        autoGearBoxAPI.CreateDownRatiosAttr([0.5, 0.5, 0.5, 0.5])
        autoGearBoxAPI.CreateLatencyAttr(2.0)

        # clutch
        clutchAPI = PhysxSchema.PhysxVehicleClutchAPI.Apply(vehiclePrim)
        clutchAPI.CreateStrengthAttr(10.0 * kgmsScale)
    else:
        # drive
        driveAPI = PhysxSchema.PhysxVehicleDriveBasicAPI.Apply(vehiclePrim)
        driveAPI.CreatePeakTorqueAttr(1000.0 * kgmsScale)

    if (useDriveStandard and configureAsTank):
        vehicleTankControllerAPI = PhysxSchema.PhysxVehicleTankControllerAPI.Apply(vehiclePrim)

        vehicleTankControllerAPI.CreateThrust0Attr(0)
        vehicleTankControllerAPI.CreateThrust1Attr(0)

        vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI(vehicleTankControllerAPI)  # tank controller also applies base controller
    else:
        vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI.Apply(vehiclePrim)
    vehicleControllerAPI.CreateAcceleratorAttr(0)
    vehicleControllerAPI.CreateBrake0Attr(0)
    vehicleControllerAPI.CreateBrake1Attr(0)
    vehicleControllerAPI.CreateSteerAttr(0)
    if useDriveStandard:
        vehicleControllerAPI.CreateTargetGearAttr(VEHICLE_AUTOMATIC_TRANSMISSION_GEAR_VALUE)
    else:
        vehicleControllerAPI.CreateTargetGearAttr(1)

    wheelPaths = []
    wheelRestPositionY = wheelRadius - chassisCenterToGround
    wheelRadiusRender = wheelRadius
    wheelRadiusCollision = wheelRadius

    # suspensions
    sprungMass = mass / 4
    springStrength = 45000 * massScale
    springDamping = 4500 * massScale
    maxDroop = (sprungMass * gravityMagnitude) / springStrength
    suspensionTravelDistance = 2.0 * maxDroop
    wheelRestSuspLength = suspensionTravelDistance - maxDroop
    tireRestLoad = sprungMass * gravityMagnitude
    tireLongStiffness = 5000 * forceScale
    tireCamberStiffness = 0 * forceScale

    suspensionFramePositionY = wheelRestPositionY + wheelRestSuspLength

    #
    # front left wheel
    #
    wheelPos = Gf.Vec3f(0.8 * lengthScale, wheelRestPositionY, 1.6 * lengthScale)
    suspFramePos = Gf.Vec3f(wheelPos[0], suspensionFramePositionY, wheelPos[2])
    vehicleWheelPath = vehiclePath + "/FrontLeftWheel"
    wheelPaths.append(vehicleWheelPath)
    vehicleWheel = UsdGeom.Xform.Define(stage, vehicleWheelPath)
    vehicleWheel.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(wheelPos)

    wheelAttachmentAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI.Apply(vehicleWheel.GetPrim())

    collisionGroupRel = wheelAttachmentAPI.CreateCollisionGroupRel()
    collisionGroupRel.AddTarget(collisionGroupVehicleGroundQueryPath)

    wheelAttachmentAPI.CreateSuspensionTravelDirectionAttr(Gf.Vec3f(0, -1, 0))
    wheelAttachmentAPI.CreateSuspensionFramePositionAttr(suspFramePos)
    wheelAttachmentAPI.CreateIndexAttr(0)

    wheelAPI = PhysxSchema.PhysxVehicleWheelAPI.Apply(wheelAttachmentAPI.GetPrim())
    wheelAPI.CreateRadiusAttr(wheelRadius)
    wheelAPI.CreateWidthAttr(wheelWidth)
    wheelAPI.CreateMassAttr(20 * massScale)
    wheelAPI.CreateMoiAttr(1.225 * kgmsScale)
    wheelAPI.CreateDampingRateAttr(0.25 * kgmsScale)

    tireAPI = PhysxSchema.PhysxVehicleTireAPI.Apply(wheelAttachmentAPI.GetPrim())
    tireAPI.CreateLateralStiffnessGraphAttr(Gf.Vec2f(2.0, 17.0 * tireRestLoad))
    tireAPI.CreateLongitudinalStiffnessAttr(tireLongStiffness)
    tireAPI.CreateCamberStiffnessAttr(tireCamberStiffness)
    tireAPI.CreateFrictionVsSlipGraphAttr([Gf.Vec2f(0.0, 1.0), Gf.Vec2f(0.1, 1.0), Gf.Vec2f(1.0, 1.0)])
    tireFrictionTableRel = tireAPI.CreateFrictionTableRel()
    tireFrictionTableRel.AddTarget(winterTireFrictionTablePath)

    suspensionAPI = PhysxSchema.PhysxVehicleSuspensionAPI.Apply(wheelAttachmentAPI.GetPrim())
    suspensionAPI.CreateSpringStrengthAttr(springStrength)
    suspensionAPI.CreateSpringDamperRateAttr(springDamping)
    suspensionAPI.CreateTravelDistanceAttr(suspensionTravelDistance)

    #
    # front right wheel
    #
    wheelPos = Gf.Vec3f(-0.8 * lengthScale, wheelRestPositionY, 1.6 * lengthScale)
    suspFramePos = Gf.Vec3f(wheelPos[0], suspensionFramePositionY, wheelPos[2])
    vehicleWheelPath = vehiclePath + "/FrontRightWheel"
    wheelPaths.append(vehicleWheelPath)
    vehicleWheel = UsdGeom.Xform.Define(stage, vehicleWheelPath)
    vehicleWheel.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(wheelPos)

    wheelAttachmentAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI.Apply(vehicleWheel.GetPrim())

    collisionGroupRel = wheelAttachmentAPI.CreateCollisionGroupRel()
    collisionGroupRel.AddTarget(collisionGroupVehicleGroundQueryPath)

    wheelAttachmentAPI.CreateSuspensionTravelDirectionAttr(Gf.Vec3f(0, -1, 0))
    wheelAttachmentAPI.CreateSuspensionFramePositionAttr(suspFramePos)
    wheelAttachmentAPI.CreateIndexAttr(1)

    wheelAPI = PhysxSchema.PhysxVehicleWheelAPI.Apply(wheelAttachmentAPI.GetPrim())
    wheelAPI.CreateRadiusAttr(wheelRadius)
    wheelAPI.CreateWidthAttr(wheelWidth)
    wheelAPI.CreateMassAttr(20 * massScale)
    wheelAPI.CreateMoiAttr(1.225 * kgmsScale)
    wheelAPI.CreateDampingRateAttr(0.25 * kgmsScale)

    tireAPI = PhysxSchema.PhysxVehicleTireAPI.Apply(wheelAttachmentAPI.GetPrim())
    tireAPI.CreateLateralStiffnessGraphAttr(Gf.Vec2f(2.0, 17.0 * tireRestLoad))
    tireAPI.CreateLongitudinalStiffnessAttr(tireLongStiffness)
    tireAPI.CreateCamberStiffnessAttr(tireCamberStiffness)
    tireAPI.CreateFrictionVsSlipGraphAttr([Gf.Vec2f(0.0, 1.0), Gf.Vec2f(0.1, 1.0), Gf.Vec2f(1.0, 1.0)])
    tireFrictionTableRel = tireAPI.CreateFrictionTableRel()
    tireFrictionTableRel.AddTarget(winterTireFrictionTablePath)

    suspensionAPI = PhysxSchema.PhysxVehicleSuspensionAPI.Apply(wheelAttachmentAPI.GetPrim())
    suspensionAPI.CreateSpringStrengthAttr(springStrength)
    suspensionAPI.CreateSpringDamperRateAttr(springDamping)
    suspensionAPI.CreateTravelDistanceAttr(suspensionTravelDistance)

    #
    # rear left wheel
    #
    wheelPos = Gf.Vec3f(0.8 * lengthScale, wheelRestPositionY, -1.6 * lengthScale)
    suspFramePos = Gf.Vec3f(wheelPos[0], suspensionFramePositionY, wheelPos[2])
    vehicleWheelPath = vehiclePath + "/RearLeftWheel"
    wheelPaths.append(vehicleWheelPath)
    vehicleWheel = UsdGeom.Xform.Define(stage, vehicleWheelPath)
    vehicleWheel.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(wheelPos)

    wheelAttachmentAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI.Apply(vehicleWheel.GetPrim())

    collisionGroupRel = wheelAttachmentAPI.CreateCollisionGroupRel()
    collisionGroupRel.AddTarget(collisionGroupVehicleGroundQueryPath)

    wheelAttachmentAPI.CreateSuspensionTravelDirectionAttr(Gf.Vec3f(0, -1, 0))
    wheelAttachmentAPI.CreateSuspensionFramePositionAttr(suspFramePos)
    wheelAttachmentAPI.CreateIndexAttr(2)

    wheelAPI = PhysxSchema.PhysxVehicleWheelAPI.Apply(wheelAttachmentAPI.GetPrim())
    wheelAPI.CreateRadiusAttr(wheelRadius)
    wheelAPI.CreateWidthAttr(wheelWidth)
    wheelAPI.CreateMassAttr(20 * massScale)
    wheelAPI.CreateMoiAttr(1.225 * kgmsScale)
    wheelAPI.CreateDampingRateAttr(0.25 * kgmsScale)

    tireAPI = PhysxSchema.PhysxVehicleTireAPI.Apply(wheelAttachmentAPI.GetPrim())
    tireAPI.CreateLateralStiffnessGraphAttr(Gf.Vec2f(2.0, 25.0 * tireRestLoad))
    tireAPI.CreateLongitudinalStiffnessAttr(tireLongStiffness)
    tireAPI.CreateCamberStiffnessAttr(tireCamberStiffness)
    tireAPI.CreateFrictionVsSlipGraphAttr([Gf.Vec2f(0.0, 1.0), Gf.Vec2f(0.1, 1.0), Gf.Vec2f(1.0, 1.0)])
    tireFrictionTableRel = tireAPI.CreateFrictionTableRel()
    tireFrictionTableRel.AddTarget(winterTireFrictionTablePath)

    suspensionAPI = PhysxSchema.PhysxVehicleSuspensionAPI.Apply(wheelAttachmentAPI.GetPrim())
    suspensionAPI.CreateSpringStrengthAttr(springStrength)
    suspensionAPI.CreateSpringDamperRateAttr(springDamping)
    suspensionAPI.CreateTravelDistanceAttr(suspensionTravelDistance)

    #
    # rear right wheel
    #
    wheelPos = Gf.Vec3f(-0.8 * lengthScale, wheelRestPositionY, -1.6 * lengthScale)
    suspFramePos = Gf.Vec3f(wheelPos[0], suspensionFramePositionY, wheelPos[2])
    vehicleWheelPath = vehiclePath + "/RearRightWheel"
    wheelPaths.append(vehicleWheelPath)
    vehicleWheel = UsdGeom.Xform.Define(stage, vehicleWheelPath)
    vehicleWheel.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(wheelPos)

    wheelAttachmentAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI.Apply(vehicleWheel.GetPrim())

    collisionGroupRel = wheelAttachmentAPI.CreateCollisionGroupRel()
    collisionGroupRel.AddTarget(collisionGroupVehicleGroundQueryPath)

    wheelAttachmentAPI.CreateSuspensionTravelDirectionAttr(Gf.Vec3f(0, -1, 0))
    wheelAttachmentAPI.CreateSuspensionFramePositionAttr(suspFramePos)
    wheelAttachmentAPI.CreateIndexAttr(3)

    wheelAPI = PhysxSchema.PhysxVehicleWheelAPI.Apply(wheelAttachmentAPI.GetPrim())
    wheelAPI.CreateRadiusAttr(wheelRadius)
    wheelAPI.CreateWidthAttr(wheelWidth)
    wheelAPI.CreateMassAttr(20 * massScale)
    wheelAPI.CreateMoiAttr(1.225 * kgmsScale)
    wheelAPI.CreateDampingRateAttr(0.25 * kgmsScale)

    tireAPI = PhysxSchema.PhysxVehicleTireAPI.Apply(wheelAttachmentAPI.GetPrim())
    tireAPI.CreateLateralStiffnessGraphAttr(Gf.Vec2f(2.0, 25.0 * tireRestLoad))
    tireAPI.CreateLongitudinalStiffnessAttr(tireLongStiffness)
    tireAPI.CreateCamberStiffnessAttr(tireCamberStiffness)
    tireAPI.CreateFrictionVsSlipGraphAttr([Gf.Vec2f(0.0, 1.0), Gf.Vec2f(0.1, 1.0), Gf.Vec2f(1.0, 1.0)])
    tireFrictionTableRel = tireAPI.CreateFrictionTableRel()
    tireFrictionTableRel.AddTarget(winterTireFrictionTablePath)

    suspensionAPI = PhysxSchema.PhysxVehicleSuspensionAPI.Apply(wheelAttachmentAPI.GetPrim())
    suspensionAPI.CreateSpringStrengthAttr(springStrength)
    suspensionAPI.CreateSpringDamperRateAttr(springDamping)
    suspensionAPI.CreateTravelDistanceAttr(suspensionTravelDistance)

    #
    # wheel collision shapes
    #
    if createCollisionShapesForWheels:
        for wheelPath in wheelPaths:
            vehicleWheelCollPath = wheelPath + "/Collision"
            vehicleWheelColl = UsdGeom.Cylinder.Define(stage, vehicleWheelCollPath)
            vehicleWheelColl.CreatePurposeAttr(UsdGeom.Tokens.guide)
            vehicleWheelColl.CreateHeightAttr(wheelWidth)
            vehicleWheelColl.CreateRadiusAttr(wheelRadiusCollision)
            vehicleWheelColl.CreateAxisAttr(UsdGeom.Tokens.x)
            # if height or radius is authored, USD expects extent to be authored too
            cylExtent = UsdGeom.Cylinder.ComputeExtentFromPlugins(vehicleWheelColl, 0)
            vehicleWheelColl.CreateExtentAttr(cylExtent)

            vehicleWheelCollPrim = vehicleWheelColl.GetPrim()

            collisionAPI = UsdPhysics.CollisionAPI.Apply(vehicleWheelCollPrim)
            add_collision_to_collision_group(stage, vehicleWheelCollPath, collisionGroupVehicleWheelPath)

            physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(vehicleWheelCollPrim)
            physxCollisionAPI.CreateRestOffsetAttr(0.0 * lengthScale)
            physxCollisionAPI.CreateContactOffsetAttr(0.02 * lengthScale)

    # wheels (render)
    for wheelPath in wheelPaths:
        vehicleWheelRenderPath = wheelPath + "/Render"
        vehicleWheelRender = UsdGeom.Cylinder.Define(stage, vehicleWheelRenderPath)
        vehicleWheelRender.CreateHeightAttr(wheelWidth)
        vehicleWheelRender.CreateRadiusAttr(wheelRadiusRender)
        vehicleWheelRender.CreateAxisAttr(UsdGeom.Tokens.x)
        # if height or radius is authored, USD expects extent to be authored too
        cylExtent = UsdGeom.Cylinder.ComputeExtentFromPlugins(vehicleWheelRender, 0)
        vehicleWheelRender.CreateExtentAttr(cylExtent)

    # chassis (collision)
    chassisHalfExtents = Gf.Vec3f(vehicleMassBoxDim[0] * 0.5, chassisHalfHeight, vehicleMassBoxDim[2] * 0.5)
    chassisOffset = Gf.Vec3f(0, 0, 0) * lengthScale
    vehicleChassisPath = vehiclePath + "/ChassisCollision"
    vehicleChassis = UsdGeom.Cube.Define(stage, vehicleChassisPath)
    vehicleChassis.CreatePurposeAttr(UsdGeom.Tokens.guide)
    vehicleChassis.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(chassisOffset)
    vehicleChassis.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(chassisHalfExtents)

    vehicleChassisPrim = vehicleChassis.GetPrim()

    collisionAPI = UsdPhysics.CollisionAPI.Apply(vehicleChassisPrim)
    add_collision_to_collision_group(stage, vehicleChassisPrim.GetPrimPath(), collisionGroupVehicleChassisPath)

    physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(vehicleChassisPrim)
    physxCollisionAPI.CreateRestOffsetAttr(0.0 * lengthScale)
    physxCollisionAPI.CreateContactOffsetAttr(0.02 * lengthScale)

    # chassis (render)
    vehicleChassisPath = vehiclePath + "/ChassisRender"
    vehicleChassis = UsdGeom.Mesh.Define(stage, vehicleChassisPath)
    vehicleChassis.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(chassisOffset)
    vehicleChassis.CreateDisplayColorAttr([Gf.Vec3f(0.2784314, 0.64705884, 1)])

    faceVertexCounts = [4, 4, 4, 4, 4, 4]

    faceVertexIndices = [0, 1, 3, 2, 4, 5, 7, 6, 10, 11, 13, 12, 14, 15, 9, 8, 17, 23, 21, 19, 22, 16, 18, 20]

    normals = [
        Gf.Vec3f(0, 1, 0),
        Gf.Vec3f(0, 1, 0),
        Gf.Vec3f(0, 1, 0),
        Gf.Vec3f(0, 1, 0),
        Gf.Vec3f(0, -1, 0),
        Gf.Vec3f(0, -1, 0),
        Gf.Vec3f(0, -1, 0),
        Gf.Vec3f(0, -1, 0),
        Gf.Vec3f(0, 0, -1),
        Gf.Vec3f(0, 0, -1),
        Gf.Vec3f(0, 0, 1),
        Gf.Vec3f(0, 0, 1),
        Gf.Vec3f(0, 0, 1),
        Gf.Vec3f(0, 0, 1),
        Gf.Vec3f(0, 0, -1),
        Gf.Vec3f(0, 0, -1),
        Gf.Vec3f(-1, 0, 0),
        Gf.Vec3f(1, 0, 0),
        Gf.Vec3f(-1, 0, 0),
        Gf.Vec3f(1, 0, 0),
        Gf.Vec3f(-1, 0, 0),
        Gf.Vec3f(1, 0, 0),
        Gf.Vec3f(-1, 0, 0),
        Gf.Vec3f(1, 0, 0),
    ]

    chassisHalfExtents[0] = 0.7 * lengthScale  # reduced width to make the wheels easily visible
    points = [
        Gf.Vec3f(-chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
    ]

    vehicleChassis.CreateFaceVertexCountsAttr(faceVertexCounts)
    vehicleChassis.CreateFaceVertexIndicesAttr(faceVertexIndices)
    vehicleChassis.CreateNormalsAttr(normals)
    vehicleChassis.CreatePointsAttr(points)

    # ground plane
    groundPlanePath = rootPath + "/GroundPlane"
    groundPlane = UsdGeom.Mesh.Define(stage, groundPlanePath)
    groundPlane.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(0, 0, 0) * lengthScale)
    groundPlane.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Quatf(1, 0, 0, 0))
    groundPlane.CreateDisplayColorAttr([Gf.Vec3f(0.5, 0.5, 0.5)])

    faceVertexCounts = [4]

    faceVertexIndices = [0, 1, 2, 3]

    normals = [Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 1, 0)]

    points = [
        Gf.Vec3f(-15, 0, -15) * lengthScale,
        Gf.Vec3f(15, 0, -15) * lengthScale,
        Gf.Vec3f(15, 0, 15) * lengthScale,
        Gf.Vec3f(-15, 0, 15) * lengthScale,
    ]

    groundPlane.CreateFaceVertexCountsAttr(faceVertexCounts)
    groundPlane.CreateFaceVertexIndicesAttr(faceVertexIndices)
    groundPlane.CreateNormalsAttr(normals)
    groundPlane.CreatePointsAttr(points)

    collisionPlanePath = groundPlanePath + "/CollisionPlane"
    collisionPlane = UsdGeom.Plane.Define(stage, collisionPlanePath)
    collisionPlane.CreatePurposeAttr(UsdGeom.Tokens.guide)
    collisionPlane.CreateAxisAttr(UsdGeom.Tokens.y)

    collisionPlanePrim = collisionPlane.GetPrim()

    collisionAPI = UsdPhysics.CollisionAPI.Apply(collisionPlanePrim)    
    add_collision_to_collision_group(stage, collisionPlanePrim.GetPrimPath(), collisionGroupGroundSurfacePath)    
    add_physics_material_to_prim(stage, collisionPlanePrim, Sdf.Path(tarmacMaterialPath))

    primPaths = PrimPaths()
    primPaths.vehiclePath = vehiclePath
    primPaths.wheelAttachmentPaths = wheelPaths
    return primPaths
