# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
import copy
from pxr import Usd, UsdLux, UsdGeom, UsdShade, Sdf, Gf, UsdPhysics
from pxr import PhysxSchema
import omni.usd
import omni.kit
import omni.physxdemos as demo
from omni.physx.scripts.physicsUtils import *
from omni.physx.scripts.utils import (
    set_custom_metadata
)
from omni.physx.bindings._physx import SimulationEvent
from ..helpers.UnitScale import UnitScale
from .VehicleSampleBase import VehicleSampleBase
from carb.eventdispatcher import get_eventdispatcher


TRAILER_FL_WHEEL = "/FrontLeftWheel"
TRAILER_FR_WHEEL = "/FrontRightWheel"
TRAILER_RL_WHEEL = "/RearLeftWheel"
TRAILER_RR_WHEEL = "/RearRightWheel"


class TruckDemo(VehicleSampleBase):
    title = "Truck"
    category = demo.Categories.VEHICLES
    short_description = "Vehicle setup for a simple truck"
    description = ("Demo providing two examples of how to set up a simple truck. One approach uses 3 bodies and 2 joints, the other uses 2 bodies and user defined sprung mass values on the trailer. "
        "The arrow keys can be used to steer, accelerate and brake. To use a gamepad for controlling the vehicle, make sure to disable Gamepad Camera Control in the Viewport Settings.")

    def create(self, stage):
        super().create(stage)

        _create(stage)

        self.autofocus = True # autofocus on the scene at first update
        self.autofocus_zoom = 0.28 # Get a bit closer


def _computeStiffnessAndDamping(naturalFrequency, dampingRatio, sprungMass, gravity):
    stiffTmp = naturalFrequency * 2.0  * math.pi
    stiffness = (stiffTmp * stiffTmp) * sprungMass
    damping = dampingRatio * 2.0 * math.sqrt(stiffness * sprungMass)
    maxDroop = (sprungMass * gravity) / stiffness
    return (stiffness, damping, maxDroop)


def _createSuspension(stage, suspensionPath,
    naturalFrequency, dampingRatio, sprungMass, gravity):

    (stiffness, damping, maxDroop) = _computeStiffnessAndDamping(naturalFrequency, dampingRatio, sprungMass, gravity)
    travelDistance = 2.0 * maxDroop

    suspensionPrim = UsdGeom.Scope.Define(stage, suspensionPath).GetPrim()
    suspension = PhysxSchema.PhysxVehicleSuspensionAPI.Apply(suspensionPrim)
    suspension.CreateSpringStrengthAttr().Set(stiffness)
    suspension.CreateSpringDamperRateAttr().Set(damping)
    suspension.CreateTravelDistanceAttr(travelDistance)
    suspension.CreateSprungMassAttr().Set(sprungMass)

    return (travelDistance - maxDroop)


def _createBody(stage, unitScale:UnitScale, bodyPath, position, orientation,
    mass, massBoxDim, centerOfMassOffset,
    collisionGeomPath, collisionGeomOffset, collisionGeomHalfExtents, collisionGroupPath,
    renderGeomPath):

    lengthScale = unitScale.lengthScale

    body = UsdGeom.Xform.Define(stage, bodyPath)
    body.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(position)
    body.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(orientation)

    bodyPrim = body.GetPrim()

    UsdPhysics.RigidBodyAPI.Apply(bodyPrim)

    massAPI = UsdPhysics.MassAPI.Apply(bodyPrim)
    massAPI.CreateMassAttr().Set(mass)
    massAPI.CreateCenterOfMassAttr().Set(centerOfMassOffset)
    massAPI.CreateDiagonalInertiaAttr().Set(
        Gf.Vec3f(
            (massBoxDim[1] * massBoxDim[1]) + (massBoxDim[2] * massBoxDim[2]),
            (massBoxDim[0] * massBoxDim[0]) + (massBoxDim[2] * massBoxDim[2]),
            (massBoxDim[0] * massBoxDim[0]) + (massBoxDim[1] * massBoxDim[1]),
        )
        * (1 / 12)
        * mass
    )
    massAPI.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))

    rigidBodyAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(bodyPrim)
    rigidBodyAPI.CreateSleepThresholdAttr().Set(0)
    rigidBodyAPI.CreateStabilizationThresholdAttr().Set(0)
    rigidBodyAPI.CreateDisableGravityAttr(True)
    rigidBodyAPI.CreateSolverPositionIterationCountAttr().Set(4)

    # collision
    collisionGeom = UsdGeom.Cube.Define(stage, collisionGeomPath)
    collisionGeom.CreatePurposeAttr().Set(UsdGeom.Tokens.guide)
    collisionGeom.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(collisionGeomOffset)
    collisionGeom.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(collisionGeomHalfExtents)

    collisionGeomPrim = collisionGeom.GetPrim()

    collisionAPI = UsdPhysics.CollisionAPI.Apply(collisionGeomPrim)
    add_collision_to_collision_group(stage, collisionGeomPrim.GetPrimPath(), collisionGroupPath)

    physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(collisionGeomPrim)
    physxCollisionAPI.CreateRestOffsetAttr().Set(0.0 * lengthScale)
    physxCollisionAPI.CreateContactOffsetAttr().Set(0.02 * lengthScale)

    # render
    renderGeom = UsdGeom.Mesh.Define(stage, renderGeomPath)
    renderGeom.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(collisionGeomOffset)
    renderGeom.CreateDisplayColorAttr().Set([Gf.Vec3f(0.2784314, 0.64705884, 1)])

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

    points = [
        Gf.Vec3f(-collisionGeomHalfExtents[0], collisionGeomHalfExtents[1], -collisionGeomHalfExtents[2]),
        Gf.Vec3f(collisionGeomHalfExtents[0], collisionGeomHalfExtents[1], -collisionGeomHalfExtents[2]),
        Gf.Vec3f(-collisionGeomHalfExtents[0], collisionGeomHalfExtents[1], collisionGeomHalfExtents[2]),
        Gf.Vec3f(collisionGeomHalfExtents[0], collisionGeomHalfExtents[1], collisionGeomHalfExtents[2]),
        Gf.Vec3f(-collisionGeomHalfExtents[0], -collisionGeomHalfExtents[1], collisionGeomHalfExtents[2]),
        Gf.Vec3f(collisionGeomHalfExtents[0], -collisionGeomHalfExtents[1], collisionGeomHalfExtents[2]),
        Gf.Vec3f(-collisionGeomHalfExtents[0], -collisionGeomHalfExtents[1], -collisionGeomHalfExtents[2]),
        Gf.Vec3f(collisionGeomHalfExtents[0], -collisionGeomHalfExtents[1], -collisionGeomHalfExtents[2]),
        Gf.Vec3f(-collisionGeomHalfExtents[0], collisionGeomHalfExtents[1], -collisionGeomHalfExtents[2]),
        Gf.Vec3f(collisionGeomHalfExtents[0], collisionGeomHalfExtents[1], -collisionGeomHalfExtents[2]),
        Gf.Vec3f(-collisionGeomHalfExtents[0], collisionGeomHalfExtents[1], collisionGeomHalfExtents[2]),
        Gf.Vec3f(collisionGeomHalfExtents[0], collisionGeomHalfExtents[1], collisionGeomHalfExtents[2]),
        Gf.Vec3f(-collisionGeomHalfExtents[0], -collisionGeomHalfExtents[1], collisionGeomHalfExtents[2]),
        Gf.Vec3f(collisionGeomHalfExtents[0], -collisionGeomHalfExtents[1], collisionGeomHalfExtents[2]),
        Gf.Vec3f(-collisionGeomHalfExtents[0], -collisionGeomHalfExtents[1], -collisionGeomHalfExtents[2]),
        Gf.Vec3f(collisionGeomHalfExtents[0], -collisionGeomHalfExtents[1], -collisionGeomHalfExtents[2]),
        Gf.Vec3f(-collisionGeomHalfExtents[0], collisionGeomHalfExtents[1], -collisionGeomHalfExtents[2]),
        Gf.Vec3f(collisionGeomHalfExtents[0], collisionGeomHalfExtents[1], -collisionGeomHalfExtents[2]),
        Gf.Vec3f(-collisionGeomHalfExtents[0], collisionGeomHalfExtents[1], collisionGeomHalfExtents[2]),
        Gf.Vec3f(collisionGeomHalfExtents[0], collisionGeomHalfExtents[1], collisionGeomHalfExtents[2]),
        Gf.Vec3f(-collisionGeomHalfExtents[0], -collisionGeomHalfExtents[1], collisionGeomHalfExtents[2]),
        Gf.Vec3f(collisionGeomHalfExtents[0], -collisionGeomHalfExtents[1], collisionGeomHalfExtents[2]),
        Gf.Vec3f(-collisionGeomHalfExtents[0], -collisionGeomHalfExtents[1], -collisionGeomHalfExtents[2]),
        Gf.Vec3f(collisionGeomHalfExtents[0], -collisionGeomHalfExtents[1], -collisionGeomHalfExtents[2]),
    ]

    renderGeom.CreateFaceVertexCountsAttr().Set(faceVertexCounts)
    renderGeom.CreateFaceVertexIndicesAttr().Set(faceVertexIndices)
    renderGeom.CreateNormalsAttr().Set(normals)
    renderGeom.CreatePointsAttr().Set(points)

    return bodyPrim


def _createVehicleBody(stage, unitScale:UnitScale, vehiclePath, position, orientation, vehicleMass, vehicleMassBoxDim,
    centerOfMassOffset, chassisHalfExtents, chassisOffset, collisionGroupVehicleChassisPath):

    lengthScale = unitScale.lengthScale
    vehicleChassisCollisionPath = vehiclePath + "/ChassisCollision"
    vehicleChassisRenderPath = vehiclePath + "/ChassisRender"

    vehiclePrim = _createBody(stage, unitScale, vehiclePath, position, orientation, vehicleMass, vehicleMassBoxDim, centerOfMassOffset,
        vehicleChassisCollisionPath, chassisOffset, chassisHalfExtents, collisionGroupVehicleChassisPath,
        vehicleChassisRenderPath)

    vehicleAPI = PhysxSchema.PhysxVehicleAPI.Apply(vehiclePrim)
    vehicleAPI.CreateVehicleEnabledAttr().Set(True)
    vehicleAPI.CreateSubStepThresholdLongitudinalSpeedAttr().Set(5.0 * lengthScale)
    vehicleAPI.CreateLowForwardSpeedSubStepCountAttr().Set(3)
    vehicleAPI.CreateHighForwardSpeedSubStepCountAttr().Set(1)
    vehicleAPI.CreateMinPassiveLongitudinalSlipDenominatorAttr().Set(4.0 * lengthScale)
    vehicleAPI.CreateMinActiveLongitudinalSlipDenominatorAttr().Set(0.1 * lengthScale)
    vehicleAPI.CreateMinLateralSlipDenominatorAttr().Set(1.0 * lengthScale)
    set_custom_metadata(vehiclePrim, PhysxSchema.Tokens.referenceFrameIsCenterOfMass, False)


def _createWheelAttachments(stage, wheelAttachmentPaths, wheelPositions, suspensionFramePositions,
    wheelPaths, tirePaths, suspensionPaths, wheelWidth, wheelRadius,
    groundQueryCollisionGroupPath):
    for i in range(len(wheelAttachmentPaths)):
        wheelPos = wheelPositions[i]
        suspFramePos = suspensionFramePositions[i]
        vehicleWheelAttPath = wheelAttachmentPaths[i]

        vehicleWheelAtt = UsdGeom.Xform.Define(stage, vehicleWheelAttPath)

        vehicleWheelAtt.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(wheelPos)

        vehicleWheelAttPrim = vehicleWheelAtt.GetPrim()
        wheelAttachmentAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI.Apply(vehicleWheelAttPrim)

        wheelRel = wheelAttachmentAPI.CreateWheelRel()
        wheelRel.AddTarget(wheelPaths[i])
        tireRel = wheelAttachmentAPI.CreateTireRel()
        tireRel.AddTarget(tirePaths[i])
        suspensionRel = wheelAttachmentAPI.CreateSuspensionRel()
        suspensionRel.AddTarget(suspensionPaths[i])
        collisionGroupRel = wheelAttachmentAPI.CreateCollisionGroupRel()
        collisionGroupRel.AddTarget(groundQueryCollisionGroupPath)

        suspTravelDir = Gf.Vec3f(0.0, -1.0, 0.0)
        wheelAttachmentAPI.CreateSuspensionTravelDirectionAttr().Set(suspTravelDir)
        wheelAttachmentAPI.CreateSuspensionFramePositionAttr().Set(suspFramePos)
        wheelAttachmentAPI.CreateIndexAttr().Set(i)

        vehicleWheelRenderPath = vehicleWheelAttPath + "/Render"
        vehicleWheelRender = UsdGeom.Cylinder.Define(stage, vehicleWheelRenderPath)
        vehicleWheelRender.CreateHeightAttr().Set(wheelWidth)
        vehicleWheelRender.CreateRadiusAttr().Set(wheelRadius)
        vehicleWheelRender.CreateAxisAttr().Set(UsdGeom.Tokens.x)
        # if height or radius is authored, USD expects extent to be authored too
        cylExtent = UsdGeom.Cylinder.ComputeExtentFromPlugins(vehicleWheelRender, 0)
        vehicleWheelRender.CreateExtentAttr().Set(cylExtent)


def _createTractor(stage, unitScale:UnitScale, vehiclePath, positionOffset, vehicleMass,
    vehicleLength, wheelRadius, wheelWidth, maxBrakeTorque, tractorLinkFromBack,
    naturalFrequency, dampingRatio, gravity,
    drivePath, wheelPath, tirePath,
    collisionGroupVehicleChassisPath, collisionGroupVehicleGroundQueryPath):

    lengthScale = unitScale.lengthScale
    vehicleMassBoxDim = Gf.Vec3f(2.5 * lengthScale, 1.0 * lengthScale, vehicleLength)

    wheelDiam = wheelRadius * 2.0
    centerOfMassToGround = wheelDiam
    chassisHalfHeight = 0.8 * lengthScale
    chassisDistToGround = wheelRadius
    chassisCenterToGround = chassisHalfHeight + chassisDistToGround
    centerOfMassOffset = Gf.Vec3f(0, centerOfMassToGround - chassisCenterToGround, 0)

    vehiclePos = Gf.Vec3f(positionOffset[0], chassisCenterToGround + positionOffset[1], positionOffset[2])
    vehicleOrient = Gf.Quatf(1, 0, 0, 0)

    chassisHalfExtents = Gf.Vec3f(vehicleMassBoxDim[0] * 0.5, chassisHalfHeight, vehicleMassBoxDim[2] * 0.5)
    chassisOffset = Gf.Vec3f(0, 0, 0) * lengthScale

    _createVehicleBody(stage, unitScale, vehiclePath, vehiclePos, vehicleOrient, vehicleMass, vehicleMassBoxDim,
        centerOfMassOffset, chassisHalfExtents, chassisOffset, collisionGroupVehicleChassisPath)

    if (drivePath is not None):
        vehiclePrim = stage.GetPrimAtPath(vehiclePath)
        vehicleAPI = PhysxSchema.PhysxVehicleAPI(vehiclePrim)
        driveRel = vehicleAPI.CreateDriveRel()
        driveRel.AddTarget(drivePath)
        vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI.Apply(vehiclePrim)
        vehicleControllerAPI.CreateTargetGearAttr(1)

        # set up one brake configuration that applies to all wheels
        brakes0API = PhysxSchema.PhysxVehicleBrakesAPI.Apply(vehiclePrim, PhysxSchema.Tokens.brakes0)
        brakes0API.CreateMaxBrakeTorqueAttr(maxBrakeTorque)

        # set up a steering configuration that applies to the front wheels
        steeringAPI = PhysxSchema.PhysxVehicleSteeringAPI.Apply(vehiclePrim)
        steeringAPI.CreateWheelsAttr([0, 1])
        steeringAPI.CreateMaxSteerAngleAttr(0.554264)

        multiWheelDiffAPI = PhysxSchema.PhysxVehicleMultiWheelDifferentialAPI.Apply(vehiclePrim)
        multiWheelDiffAPI.CreateWheelsAttr([0, 1])
        multiWheelDiffAPI.CreateTorqueRatiosAttr([0.5, 0.5])

    wheelRestPositionY = wheelRadius - chassisCenterToGround

    # front left wheel, front right wheel, rear left wheel, rear right wheel
    wheelAttachmentPathPrefix = vehiclePath
    wheelAttachmentPaths = [
        wheelAttachmentPathPrefix + "/FrontLeftWheel",
        wheelAttachmentPathPrefix + "/FrontRightWheel",
        wheelAttachmentPathPrefix + "/RearLeftWheel",
        wheelAttachmentPathPrefix + "/RearRightWheel",
    ]

    halfVehicleLength = vehicleLength * 0.5
    wheelZFront = halfVehicleLength - wheelDiam
    wheelZRear = -(halfVehicleLength - tractorLinkFromBack)
    wheelX = chassisHalfExtents[0] - (wheelWidth * 0.5 * 0.9)
    wheelPositions = [
        Gf.Vec3f(wheelX, wheelRestPositionY, wheelZFront),
        Gf.Vec3f(-wheelX, wheelRestPositionY, wheelZFront),
        Gf.Vec3f(wheelX * lengthScale, wheelRestPositionY, wheelZRear),
        Gf.Vec3f(-wheelX * lengthScale, wheelRestPositionY, wheelZRear),
    ]

    physxVehicleInterface = omni.physxvehicle.get_physx_vehicle_interface()
    sprungMasses = physxVehicleInterface.compute_sprung_masses(vehicleMass, 1, wheelPositions)
    # note: compute_sprung_masses expects the provided positions to be relative to the center
    #       of mass, however, since in this example the center of mass only differs in the vertical
    #       axis from the vehicle prim position, the computed sprung mass values will be the same.

    frontSuspensionPath = vehiclePath + "/FrontSuspension"
    wheelAtRestSuspLengthFront = _createSuspension(stage, frontSuspensionPath,
        naturalFrequency, dampingRatio, sprungMasses[0], gravity)

    rearSuspensionPath = vehiclePath + "/RearSuspension"
    wheelAtRestSuspLengthRear = _createSuspension(stage, rearSuspensionPath,
        naturalFrequency, dampingRatio, sprungMasses[2], gravity)

    wheelPaths = [
        wheelPath, wheelPath, wheelPath, wheelPath
    ]
    tirePaths = [
        tirePath, tirePath, tirePath, tirePath
    ]
    suspensionPaths = [
        frontSuspensionPath, frontSuspensionPath, rearSuspensionPath, rearSuspensionPath
    ]

    suspensionFramePositionYFront = wheelRestPositionY + wheelAtRestSuspLengthFront
    suspensionFramePositionYRear = wheelRestPositionY + wheelAtRestSuspLengthRear
    suspensionFramePositions = [
        Gf.Vec3f(wheelX, suspensionFramePositionYFront, wheelZFront),
        Gf.Vec3f(-wheelX, suspensionFramePositionYFront, wheelZFront),
        Gf.Vec3f(wheelX * lengthScale, suspensionFramePositionYRear, wheelZRear),
        Gf.Vec3f(-wheelX * lengthScale, suspensionFramePositionYRear, wheelZRear),
    ]

    _createWheelAttachments(stage, wheelAttachmentPaths, wheelPositions, suspensionFramePositions,
        wheelPaths, tirePaths, suspensionPaths, wheelWidth, wheelRadius,
        collisionGroupVehicleGroundQueryPath)


def _createTrailer(stage, unitScale:UnitScale, vehiclePath, bodyPartPath, bodyPartRatio, positionOffset, vehicleMass,
    vehicleLength, wheelRadius, wheelWidth, trailerLinkFromFront, linkY,
    naturalFrequency, dampingRatio, gravity,
    wheelPath, tirePath,
    collisionGroupVehicleChassisPath, collisionGroupVehicleGroundQueryPath):

    lengthScale = unitScale.lengthScale
    centerOfMassToGround = wheelRadius * 2.5
    chassisHalfHeight = 1.1 * lengthScale
    chassisDistToGround = wheelRadius
    chassisCenterToGround = chassisHalfHeight + chassisDistToGround
    centerOfMassOffset = Gf.Vec3f(0, centerOfMassToGround - chassisCenterToGround, 0)

    vehicleMassBoxDim = Gf.Vec3f(2.4 * lengthScale, 2.0 * lengthScale, vehicleLength)

    chassisHalfExtents = Gf.Vec3f(vehicleMassBoxDim[0] * 0.5, chassisHalfHeight, vehicleMassBoxDim[2] * 0.5)
    chassisOffset = Gf.Vec3f(0, 0, 0) * lengthScale

    orient = Gf.Quatf(1, 0, 0, 0)

    trailerHalfLength = chassisHalfExtents[2]
    trailerLength = 2.0 * trailerHalfLength
    bodyPartLength = trailerLength * bodyPartRatio
    vehiclePartLength = trailerLength - bodyPartLength
    vehicleWheelsCenterZ = -trailerHalfLength + (0.5 * vehiclePartLength)

    if bodyPartPath is not None:
        # the trailer is split into two bodies. One body is treated as a vehicle and covers more or less the section with the wheels,
        # the other body is no vehicle and will just be connected to the vehicle body with a joint
        bodyMassBoxDim = Gf.Vec3f(vehicleMassBoxDim)
        bodyMassBoxDim[2] = bodyMassBoxDim[2] * bodyPartRatio
        vehicleMassBoxDim[2] = vehicleMassBoxDim[2] * (1.0 - bodyPartRatio)

        bodyMass = vehicleMass * bodyPartRatio
        vehicleMass = vehicleMass * (1.0 - bodyPartRatio)

        bodyPos = Gf.Vec3f(positionOffset[0], chassisCenterToGround + positionOffset[1],
            positionOffset[2] + trailerHalfLength - (0.5 * bodyPartLength))

        collGeomHalfExtents = Gf.Vec3f(chassisHalfExtents)
        collGeomHalfExtents[2] = bodyPartLength * 0.5
        bodyPartPrim = _createBody(stage, unitScale, bodyPartPath, bodyPos, orient,
            bodyMass, bodyMassBoxDim, centerOfMassOffset,
            bodyPartPath + "/Collision", chassisOffset, collGeomHalfExtents, collisionGroupVehicleChassisPath,
            bodyPartPath + "/Render")

        chassisHalfExtents[2] = vehiclePartLength * 0.5
        vehiclePos = Gf.Vec3f(positionOffset[0], chassisCenterToGround + positionOffset[1],
            positionOffset[2] + vehicleWheelsCenterZ)
        wheelOffsetZ = 0
    else:
        vehiclePos = Gf.Vec3f(positionOffset[0], chassisCenterToGround + positionOffset[1], positionOffset[2])
        wheelOffsetZ = vehicleWheelsCenterZ

    _createVehicleBody(stage, unitScale, vehiclePath, vehiclePos, orient, vehicleMass, vehicleMassBoxDim,
        centerOfMassOffset, chassisHalfExtents, chassisOffset, collisionGroupVehicleChassisPath)

    if bodyPartPath is not None:
        fixedJointPath = bodyPartPath + "Joint"
        fixedJoint = UsdPhysics.FixedJoint.Define(stage, fixedJointPath)
        body0Rel = fixedJoint.CreateBody0Rel()
        body0Rel.AddTarget(bodyPartPath)
        body1Rel = fixedJoint.CreateBody1Rel()
        body1Rel.AddTarget(vehiclePath)
        fixedJoint.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, -(bodyPartLength * 0.5)))
        fixedJoint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, (vehiclePartLength * 0.5)))

    wheelRestPositionY = wheelRadius - chassisCenterToGround

    # front left wheel, front right wheel, rear left wheel, rear right wheel
    wheelAttachmentPathPrefix = vehiclePath
    wheelAttachmentPaths = [
        wheelAttachmentPathPrefix + TRAILER_FL_WHEEL,
        wheelAttachmentPathPrefix + TRAILER_FR_WHEEL,
        wheelAttachmentPathPrefix + TRAILER_RL_WHEEL,
        wheelAttachmentPathPrefix + TRAILER_RR_WHEEL,
    ]

    wheelDeltaZ = vehiclePartLength * 0.15
    wheelX = chassisHalfExtents[0] - (wheelWidth * 0.5 * 0.9)
    wheelPositions = [
        Gf.Vec3f(wheelX, wheelRestPositionY, wheelOffsetZ + wheelDeltaZ),
        Gf.Vec3f(-wheelX, wheelRestPositionY, wheelOffsetZ + wheelDeltaZ),
        Gf.Vec3f(wheelX, wheelRestPositionY, wheelOffsetZ - wheelDeltaZ),
        Gf.Vec3f(-wheelX, wheelRestPositionY, wheelOffsetZ - wheelDeltaZ),
    ]

    if bodyPartPath is None:
        trailerAttachmentPoint = Gf.Vec3f(
            0.0,
            linkY - centerOfMassToGround,
            trailerHalfLength - trailerLinkFromFront)
        wheelPositions.append(trailerAttachmentPoint)

    physxVehicleInterface = omni.physxvehicle.get_physx_vehicle_interface()
    sprungMasses = physxVehicleInterface.compute_sprung_masses(vehicleMass, 1, wheelPositions)
    # note: compute_sprung_masses expects the provided positions to be relative to the center
    #       of mass, however, since in this example the center of mass only differs in the vertical
    #       axis from the vehicle prim position, the computed sprung mass values will be the same.

    frontSuspensionPath = vehiclePath + "/FrontSuspension"
    wheelAtRestSuspLengthFront = _createSuspension(stage, frontSuspensionPath,
        naturalFrequency, dampingRatio, sprungMasses[0], gravity)

    rearSuspensionPath = vehiclePath + "/RearSuspension"
    wheelAtRestSuspLengthRear = _createSuspension(stage, rearSuspensionPath,
        naturalFrequency, dampingRatio, sprungMasses[2], gravity)

    wheelPaths = [
        wheelPath, wheelPath, wheelPath, wheelPath
    ]
    tirePaths = [
        tirePath, tirePath, tirePath, tirePath
    ]
    suspensionPaths = [
        frontSuspensionPath, frontSuspensionPath, rearSuspensionPath, rearSuspensionPath
    ]

    suspensionFramePositionYFront = wheelRestPositionY + wheelAtRestSuspLengthFront
    suspensionFramePositionYRear = wheelRestPositionY + wheelAtRestSuspLengthRear
    suspensionFramePositions = [
        Gf.Vec3f(wheelX, suspensionFramePositionYFront, wheelOffsetZ + wheelDeltaZ),
        Gf.Vec3f(-wheelX, suspensionFramePositionYFront, wheelOffsetZ + wheelDeltaZ),
        Gf.Vec3f(wheelX, suspensionFramePositionYRear, wheelOffsetZ - wheelDeltaZ),
        Gf.Vec3f(-wheelX, suspensionFramePositionYRear, wheelOffsetZ - wheelDeltaZ),
    ]

    _createWheelAttachments(stage, wheelAttachmentPaths, wheelPositions, suspensionFramePositions,
        wheelPaths, tirePaths, suspensionPaths, wheelWidth, wheelRadius,
        collisionGroupVehicleGroundQueryPath)

    for i in range(len(wheelAttachmentPaths)):
        wheelPrim = stage.GetPrimAtPath(wheelAttachmentPaths[i])
        PhysxSchema.PhysxVehicleWheelControllerAPI.Apply(wheelPrim)


def _createTractorAndTrailer(stage, unitScale:UnitScale, tractorPath, trailerPath, trailerBodyPartPath, trailerBodyPartRatio,
    positionOffset, tractorMass, trailerMass, tractorLength, trailerLength, wheelRadius, wheelWidth, maxBrakeTorque,
    tractorLinkFromBack, trailerLinkFromFront, tractorLinkLocalUp, useRevolutJoint,
    naturalFrequency, dampingRatio, gravity,
    drivePath, wheelPath, tirePath,
    collisionGroupVehicleChassisPath, collisionGroupVehicleGroundQueryPath):

    lengthScale = unitScale.lengthScale
    _createTractor(stage, unitScale, tractorPath, positionOffset, tractorMass,
        tractorLength, wheelRadius, wheelWidth, maxBrakeTorque, tractorLinkFromBack,
        naturalFrequency, dampingRatio, gravity,
        drivePath, wheelPath, tirePath,
        collisionGroupVehicleChassisPath, collisionGroupVehicleGroundQueryPath)

    tractorJointPrim = stage.GetPrimAtPath(tractorPath)
    tractorPos = tractorJointPrim.GetAttribute("xformOp:translate").Get()
    linkY = tractorPos[1] + tractorLinkLocalUp

    trailerPositionOffset = Gf.Vec3f(positionOffset)
    trailerPositionOffset[2] = trailerPositionOffset[2] - (tractorLength * 0.5) - (trailerLength * 0.5) + (tractorLinkFromBack + trailerLinkFromFront)
    _createTrailer(stage, unitScale, trailerPath, trailerBodyPartPath, trailerBodyPartRatio, trailerPositionOffset,
        trailerMass, trailerLength, wheelRadius, wheelWidth, trailerLinkFromFront, linkY,
        naturalFrequency, dampingRatio, gravity,
        wheelPath, tirePath,
        collisionGroupVehicleChassisPath, collisionGroupVehicleGroundQueryPath)

    jointPath = tractorPath + "Joint"
    if useRevolutJoint:
        joint = UsdPhysics.RevoluteJoint.Define(stage, jointPath)
    else:
        joint = UsdPhysics.Joint.Define(stage, jointPath)

    body0Rel = joint.CreateBody0Rel()
    body0Rel.AddTarget(tractorPath)
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, tractorLinkLocalUp, -(tractorLength * 0.5) + tractorLinkFromBack))
    body1Rel = joint.CreateBody1Rel()

    if (trailerBodyPartPath is None):
        body1Rel.AddTarget(trailerPath)
        trailerJointPrim = stage.GetPrimAtPath(trailerPath)
        trailerLinkZ = (trailerLength * 0.5) - trailerLinkFromFront

    else:
        body1Rel.AddTarget(trailerBodyPartPath)
        trailerJointPrim = stage.GetPrimAtPath(trailerBodyPartPath)
        trailerLinkZ = (trailerLength * trailerBodyPartRatio * 0.5) - trailerLinkFromFront

    trailerPos = trailerJointPrim.GetAttribute("xformOp:translate").Get()
    trailerLinkY = linkY - trailerPos[1]
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, trailerLinkY, trailerLinkZ))

    if useRevolutJoint:
        joint.CreateAxisAttr().Set(UsdPhysics.Tokens.y)
        joint.CreateLowerLimitAttr().Set(-90.0)
        joint.CreateUpperLimitAttr().Set(90.0)
    else:
        linearLimit = 0.01 * lengthScale
        jointPrim = joint.GetPrim()

        # allow a bit of movement along x-axis
        limitAPI = UsdPhysics.LimitAPI.Apply(jointPrim, "transX")
        limitAPI.CreateLowAttr().Set(-linearLimit)
        limitAPI.CreateHighAttr().Set(linearLimit)

        # use a soft limit along y-axis
        limitAPI = UsdPhysics.LimitAPI.Apply(jointPrim, "transY")
        limitAPI.CreateLowAttr().Set(-0.0001)
        limitAPI.CreateHighAttr().Set(0.0001)
        limitAPI = PhysxSchema.PhysxLimitAPI.Apply(jointPrim, "transY")
        limitAPI.CreateStiffnessAttr().Set(4000000)  # unitless
        limitAPI.CreateDampingAttr().Set(800000)  # unitless

        # allow a bit of movement along z-axis
        limitAPI = UsdPhysics.LimitAPI.Apply(jointPrim, "transZ")
        limitAPI.CreateLowAttr().Set(-linearLimit)
        limitAPI.CreateHighAttr().Set(linearLimit)

        # use a soft limit around x-axis
        limitAPI = UsdPhysics.LimitAPI.Apply(jointPrim, "rotX")
        limitAPI.CreateLowAttr().Set(-0.0001)
        limitAPI.CreateHighAttr().Set(0.0001)
        limitAPI = PhysxSchema.PhysxLimitAPI.Apply(jointPrim, "rotX")
        limitAPI.CreateStiffnessAttr().Set(4000000)  # unitless
        limitAPI.CreateDampingAttr().Set(800000)  # unitless

        # use a hard limit around y-axis
        limitAPI = UsdPhysics.LimitAPI.Apply(jointPrim, "rotY")
        limitAPI.CreateLowAttr().Set(-90.0)
        limitAPI.CreateHighAttr().Set(90.0)

        # lock rotation around z-axis
        limitAPI = UsdPhysics.LimitAPI.Apply(jointPrim, "rotZ")
        limitAPI.CreateLowAttr().Set(1.0)
        limitAPI.CreateHighAttr().Set(-1.0)


class TrailerControl:
    ACCELERATE = 0
    BRAKE = 1

    def __init__(self):
        self.tractorControllerAPI = None
        self.trailerWheelControllerAPIs = []
        self.controlType = None
        self.wasControlValueZero = True


class ControlsNotificationHandler:
    def __init__(self):
        self._physxSimEventSubscription = None
        self._appUpdateEventSubscription = None
        self._change_info_path_subscriptions = []
        self._controlsUsdPathDict = None
        self._maxBrakeTorque = None
        self._tinyDriveTorque = None
        self._usdChangeList = []

    def set_up(self, controlsUsdPathDict, maxBrakeTorque, tinyDriveTorque):
        self._controlsUsdPathDict = controlsUsdPathDict
        self._maxBrakeTorque = maxBrakeTorque
        self._tinyDriveTorque = tinyDriveTorque

        physxInterface = omni.physx.get_physx_interface()
        self._physxSimEventSubscription = physxInterface.get_simulation_event_stream_v2().create_subscription_to_pop(
            self._on_simulation_event
        )

        self._stageEventSubscription = get_eventdispatcher().observe_event(
            observer_name="omni.physx.vehicle:TruckDemo",
            event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.CLOSING),
            on_event=lambda _: self._on_stage_closing_event()
        )

    def _on_simulation_event(self, event):
        if event.type == int(SimulationEvent.RESUMED):
            self._appUpdateEventSubscription = get_eventdispatcher().observe_event(
                event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
                on_event=self._on_update,
                observer_name="vehicle sample truck update",
            )
            for usdPath in self._controlsUsdPathDict:
                self._change_info_path_subscriptions.append(omni.usd.get_watcher().subscribe_to_change_info_path(
                    usdPath, self._on_change_info_path))
        elif (event.type == int(SimulationEvent.STOPPED)) or (event.type == int(SimulationEvent.PAUSED)):
            self._appUpdateEventSubscription = None
            self._change_info_path_subscriptions = []

            self._usdChangeList.clear()
            for (usdPath, trailerControl) in self._controlsUsdPathDict.items():
                trailerControl.wasControlValueZero = True

    def _on_change_info_path(self, path):
        self._usdChangeList.append(path)

    def _on_update(self, e):
        for path in self._usdChangeList:
            trailerControl = self._controlsUsdPathDict[path]
            if (trailerControl.controlType == TrailerControl.ACCELERATE):
                acc = trailerControl.tractorControllerAPI.GetAcceleratorAttr().Get()
                if (acc > 0.0):
                    if (trailerControl.wasControlValueZero):
                        # sticky friction constraints fix the trailer to the ground when not moving.
                        # A very small drive torque is applied whenever the tractor accelerates to
                        # disable sticky friction
                        for wheelControllerAPI in trailerControl.trailerWheelControllerAPIs:
                            wheelControllerAPI.GetDriveTorqueAttr().Set(self._tinyDriveTorque)

                    trailerControl.wasControlValueZero = False
                else:
                    if (not trailerControl.wasControlValueZero):
                        for wheelControllerAPI in trailerControl.trailerWheelControllerAPIs:
                            wheelControllerAPI.GetDriveTorqueAttr().Set(0.0)

                    trailerControl.wasControlValueZero = True
            else:
                brake = trailerControl.tractorControllerAPI.GetBrake0Attr().Get()
                if (brake > 0.0):
                    # let the trailer brake too whenever the tractor brakes
                    brakeTorque = brake * self._maxBrakeTorque
                    for wheelControllerAPI in trailerControl.trailerWheelControllerAPIs:
                        wheelControllerAPI.GetBrakeTorqueAttr().Set(brakeTorque)

                    trailerControl.wasControlValueZero = False
                else:
                    if (not trailerControl.wasControlValueZero):
                        for wheelControllerAPI in trailerControl.trailerWheelControllerAPIs:
                            wheelControllerAPI.GetBrakeTorqueAttr().Set(0.0)

                    trailerControl.wasControlValueZero = True

        self._usdChangeList.clear()

    def _on_stage_closing_event(self):
        self._controlsUsdPathDict = None
        self._physxSimEventSubscription = None
        self._stageEventSubscription = None


def _create(stage):

    lengthScale = 1
    massScale = 1
    lengthScaleSqr = lengthScale * lengthScale
    kgmsScale = massScale * lengthScaleSqr
    forceScale = massScale * lengthScale
    unitScale = UnitScale(lengthScale, massScale)

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0 / lengthScale)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0 / massScale)

    rootPath = str(stage.GetDefaultPrim().GetPath())

    # light
    sphereLight = UsdLux.SphereLight.Define(stage, rootPath + "/SphereLight")
    sphereLight.CreateRadiusAttr().Set(1.5 * lengthScale)
    sphereLight.CreateIntensityAttr().Set(30000)
    sphereLight.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(6.5, 11.5, 0) * lengthScale)

    # physics scene
    scene = UsdPhysics.Scene.Define(stage, rootPath + "/PhysicsScene")
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, -1, 0))
    gravity = 9.81 * lengthScale
    scene.CreateGravityMagnitudeAttr().Set(gravity)

    # vehicle context
    vehicleContextAPI = PhysxSchema.PhysxVehicleContextAPI.Apply(scene.GetPrim())
    vehicleContextAPI.CreateUpdateModeAttr().Set(PhysxSchema.Tokens.velocityChange)
    vehicleContextAPI.CreateVerticalAxisAttr(PhysxSchema.Tokens.posY)
    vehicleContextAPI.CreateLongitudinalAxisAttr(PhysxSchema.Tokens.posZ)

    # materials
    tarmacMaterialPath = rootPath + "/TarmacMaterial"
    UsdShade.Material.Define(stage, tarmacMaterialPath)
    tarmacMaterial = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(tarmacMaterialPath))
    tarmacMaterial.CreateStaticFrictionAttr().Set(0.9)
    tarmacMaterial.CreateDynamicFrictionAttr().Set(0.7)
    tarmacMaterial.CreateRestitutionAttr().Set(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(tarmacMaterial.GetPrim())

    # tire friction tables
    tireFrictionTablePath = rootPath + "/TireFrictionTable"
    tireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable.Define(stage, tireFrictionTablePath)
    tireFrictionTableGroundMaterialsRel = tireFrictionTable.CreateGroundMaterialsRel()
    tireFrictionTableGroundMaterialsRel.AddTarget(tarmacMaterialPath)
    tireFrictionTable.CreateFrictionValuesAttr([0.7])

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

    # drive basic
    driveBasicPath = rootPath + "/DriveBasic"
    driveBasicPrim = UsdGeom.Scope.Define(stage, driveBasicPath).GetPrim()
    driveBasicAPI = PhysxSchema.PhysxVehicleDriveBasicAPI.Apply(driveBasicPrim)
    driveBasicAPI.CreatePeakTorqueAttr().Set(8000.0 * kgmsScale)

    # brakes
    maxBrakeTorque = 12000 * kgmsScale

    # wheels
    wheelRadius = 0.45 * lengthScale
    wheelWidth = 0.25 * lengthScale
    wheelMass = 100 * massScale
    moi = 0.5 * wheelMass * wheelRadius * wheelRadius
    wheelDampingRate = 2.0 * kgmsScale

    wheelPath = rootPath + "/Wheel"
    wheelPrim = UsdGeom.Scope.Define(stage, wheelPath).GetPrim()
    wheel = PhysxSchema.PhysxVehicleWheelAPI.Apply(wheelPrim)
    wheel.CreateRadiusAttr().Set(wheelRadius)
    wheel.CreateWidthAttr().Set(wheelWidth)
    wheel.CreateMassAttr().Set(wheelMass)
    wheel.CreateMoiAttr().Set(moi)
    wheel.CreateDampingRateAttr().Set(wheelDampingRate)

    # mass
    tractorMass = 7000 * massScale
    trailerMass = 15000 * massScale
    trailerBodyPartRatio = 0.6

    # tires
    tireRestLoadEstimate = (tractorMass / 4.0) * gravity;
    tirePath = rootPath + "/Tire"
    tirePrim = UsdGeom.Scope.Define(stage, tirePath).GetPrim()
    tire = PhysxSchema.PhysxVehicleTireAPI.Apply(tirePrim)
    tire.CreateLateralStiffnessGraphAttr().Set(Gf.Vec2f(2.0, 17 * tireRestLoadEstimate))
    tire.CreateLongitudinalStiffnessAttr().Set(5000 * forceScale)
    tire.CreateCamberStiffnessAttr().Set(0 * forceScale)
    tire.CreateFrictionVsSlipGraphAttr().Set([Gf.Vec2f(0.0, 1.0), Gf.Vec2f(0.1, 1.0), Gf.Vec2f(1.0, 1.0)])
    tireFrictionTableRel = tire.CreateFrictionTableRel()
    tireFrictionTableRel.AddTarget(tireFrictionTablePath)

    # suspensions
    naturalFrequency = 1.5
    dampingRatio = 0.25

    tractorLength = 4.8 * lengthScale
    trailerLength = 10 * lengthScale
    tractorLinkFromBack = 1.0 * lengthScale
    trailerLinkFromFront = 1.0 * lengthScale
    tractorLinkLocalUp = -0.5 * lengthScale

    useRevolutJoint = True
    # If True, a simple revolute joint is used to connect the tractor and the trailer.
    # If False, a more advanced joint setup with more degrees of freedom is used.

    tractorPathList = []
    trailerPathList = []
    posIterCountList = []

    #
    # tractor & semitrailer, single body trailer
    #
    tractorPath = rootPath + "/Tractor1"
    trailerPath = rootPath + "/Trailer1"
    tractorPathList.append(tractorPath)
    trailerPathList.append(trailerPath)
    positionOffset = Gf.Vec3f(0.0, 0.0, 0.0) * lengthScale
    _createTractorAndTrailer(stage, unitScale, tractorPath, trailerPath, None, trailerBodyPartRatio,
        positionOffset, tractorMass, trailerMass, tractorLength, trailerLength,
        wheelRadius, wheelWidth, maxBrakeTorque,
        tractorLinkFromBack, trailerLinkFromFront, tractorLinkLocalUp, useRevolutJoint,
        naturalFrequency, dampingRatio, gravity,
        driveBasicPath, wheelPath, tirePath,
        collisionGroupVehicleChassisPath, collisionGroupVehicleGroundQueryPath)
    posIterCountList.append(16)

    #
    # tractor & semitrailer, two-body trailer
    #
    tractorPath = rootPath + "/Tractor2"
    trailerPath = rootPath + "/Trailer2BackBody"
    tractorPathList.append(tractorPath)
    trailerPathList.append(trailerPath)
    trailerBodyPartPath = rootPath + "/Trailer2FrontBody"
    positionOffset = Gf.Vec3f(5.0, 0.0, 0.0) * lengthScale
    _createTractorAndTrailer(stage, unitScale, tractorPath, trailerPath, trailerBodyPartPath, trailerBodyPartRatio,
        positionOffset, tractorMass, trailerMass, tractorLength, trailerLength,
        wheelRadius, wheelWidth, maxBrakeTorque,
        tractorLinkFromBack, trailerLinkFromFront, tractorLinkLocalUp, useRevolutJoint,
        naturalFrequency, dampingRatio, gravity,
        driveBasicPath, wheelPath, tirePath,
        collisionGroupVehicleChassisPath, collisionGroupVehicleGroundQueryPath)
    posIterCountList.append(12)

    # ground plane
    groundPlanePath = rootPath + "/GroundPlane"
    groundPlane = UsdGeom.Mesh.Define(stage, groundPlanePath)
    groundPlane.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(0, 0, 0) * lengthScale)
    groundPlane.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Quatf(1, 0, 0, 0))
    groundPlane.CreateDisplayColorAttr().Set([Gf.Vec3f(0.5, 0.5, 0.5)])

    faceVertexCounts = [4]

    faceVertexIndices = [0, 1, 2, 3]

    normals = [Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 1, 0)]

    points = [
        Gf.Vec3f(-15, 0, -15) * lengthScale,
        Gf.Vec3f(15, 0, -15) * lengthScale,
        Gf.Vec3f(15, 0, 15) * lengthScale,
        Gf.Vec3f(-15, 0, 15) * lengthScale,
    ]

    groundPlane.CreateFaceVertexCountsAttr().Set(faceVertexCounts)
    groundPlane.CreateFaceVertexIndicesAttr().Set(faceVertexIndices)
    groundPlane.CreateNormalsAttr().Set(normals)
    groundPlane.CreatePointsAttr().Set(points)

    collisionPlanePath = groundPlanePath + "/CollisionPlane"
    collisionPlane = UsdGeom.Plane.Define(stage, collisionPlanePath)
    collisionPlane.CreatePurposeAttr().Set(UsdGeom.Tokens.guide)
    collisionPlane.CreateAxisAttr().Set(UsdGeom.Tokens.y)

    collisionPlanePrim = collisionPlane.GetPrim()

    collisionAPI = UsdPhysics.CollisionAPI.Apply(collisionPlanePrim)
    add_collision_to_collision_group(stage, collisionPlanePrim.GetPrimPath(), collisionGroupGroundSurfacePath)
    add_physics_material_to_prim(stage, collisionPlanePrim, Sdf.Path(tarmacMaterialPath))

    constrolsUsdPathDict = {}
    for i in range(len(tractorPathList)):
        tractorPath = tractorPathList[i]
        trailerPath = trailerPathList[i]

        tractorPrim = stage.GetPrimAtPath(tractorPath)
        tractorControllerAPI = PhysxSchema.PhysxVehicleControllerAPI(tractorPrim)

        # system of sticky friction and joint constraints needs more iterations for stability.
        tractorBody = PhysxSchema.PhysxRigidBodyAPI.Apply(tractorPrim)
        tractorBody.GetSolverPositionIterationCountAttr().Set(posIterCountList[i])

        wheelAttachmentPaths = [
            trailerPath + TRAILER_FL_WHEEL,
            trailerPath + TRAILER_FR_WHEEL,
            trailerPath + TRAILER_RL_WHEEL,
            trailerPath + TRAILER_RR_WHEEL,
        ]
        trailerWheelControllerAPIs = []
        for wheelAttPath in wheelAttachmentPaths:
            wheelAttPrim = stage.GetPrimAtPath(wheelAttPath)
            wheelControllerAPI = PhysxSchema.PhysxVehicleWheelControllerAPI(wheelAttPrim)
            trailerWheelControllerAPIs.append(wheelControllerAPI)

        trailerControlAcc = TrailerControl()
        trailerControlAcc.tractorControllerAPI = tractorControllerAPI
        trailerControlAcc.trailerWheelControllerAPIs = trailerWheelControllerAPIs
        trailerControlAcc.controlType = TrailerControl.ACCELERATE
        trailerControlAcc.wasControlValueZero = True

        trailerControlBrake = copy.copy(trailerControlAcc)
        trailerControlBrake.controlType = TrailerControl.BRAKE

        constrolsUsdPathDict[tractorControllerAPI.GetAcceleratorAttr().GetPath()] = trailerControlAcc
        constrolsUsdPathDict[tractorControllerAPI.GetBrake0Attr().GetPath()] = trailerControlBrake

    controlsNotificationHandler = ControlsNotificationHandler()
    tinyDriveTorque = 0.00001
    controlsNotificationHandler.set_up(constrolsUsdPathDict, maxBrakeTorque, tinyDriveTorque)
