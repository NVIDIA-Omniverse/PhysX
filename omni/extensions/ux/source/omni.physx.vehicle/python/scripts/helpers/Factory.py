# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import Usd, UsdLux, UsdGeom, UsdShade, Sdf, Gf, UsdPhysics
from pxr import PhysxSchema
from omni.physx.scripts.physicsUtils import *
from omni.physx.scripts.utils import (
    set_custom_metadata
)
from omni.physx.bindings._physx import (
    VEHICLE_AUTOMATIC_TRANSMISSION_GEAR_VALUE
)
from .UnitScale import UnitScale

COLL_GROUP_VEHICLE_CHASSIS = 0
COLL_GROUP_VEHICLE_WHEEL = 1
COLL_GROUP_VEHICLE_GROUND_QUERY = 2
COLL_GROUP_GROUND_SURFACE = 3

MATERIAL_TARMAC = 0
MATERIAL_GRAVEL = 1

TIRE_FRICTION_TABLE_WINTER_TIRE = 0
TIRE_FRICTION_TABLE_SUMMER_TIRE = 1

WHEEL_FRONT = 0
WHEEL_REAR = 1

TIRE_FRONT = 0
TIRE_REAR = 1

SUSPENSION_FRONT = 0
SUSPENSION_REAR = 1

WHEEL_FRONT_LEFT = 0
WHEEL_FRONT_RIGHT = 1
WHEEL_REAR_LEFT = 2
WHEEL_REAR_RIGHT = 3

DRIVE_NONE = 0
DRIVE_BASIC = 1
DRIVE_STANDARD = 2


# ring around x-axis
gRingMeshPoints = [Gf.Vec3f(1.000000, 0.000000, -1.000000), Gf.Vec3f(-1.000000, 0.000000, -1.000000), Gf.Vec3f(1.000000, 0.195090, -0.980785), Gf.Vec3f(-1.000000, 0.195090, -0.980785), Gf.Vec3f(1.000000, 0.382683, -0.923880), Gf.Vec3f(-1.000000, 0.382683, -0.923880), Gf.Vec3f(1.000000, 0.555570, -0.831470), Gf.Vec3f(-1.000000, 0.555570, -0.831470), Gf.Vec3f(1.000000, 0.707107, -0.707107), Gf.Vec3f(-1.000000, 0.707107, -0.707107), Gf.Vec3f(1.000000, 0.831470, -0.555570), Gf.Vec3f(-1.000000, 0.831470, -0.555570), Gf.Vec3f(1.000000, 0.923880, -0.382683), Gf.Vec3f(-1.000000, 0.923880, -0.382683), Gf.Vec3f(1.000000, 0.980785, -0.195090), Gf.Vec3f(-1.000000, 0.980785, -0.195090), Gf.Vec3f(1.000000, 1.000000, 0.000000), Gf.Vec3f(-1.000000, 1.000000, 0.000000), Gf.Vec3f(1.000000, 0.980785, 0.195090), Gf.Vec3f(-1.000000, 0.980785, 0.195090), Gf.Vec3f(1.000000, 0.923880, 0.382684), Gf.Vec3f(-1.000000, 0.923880, 0.382684), Gf.Vec3f(1.000000, 0.831470, 0.555570), Gf.Vec3f(-1.000000, 0.831470, 0.555570), Gf.Vec3f(1.000000, 0.707107, 0.707107), Gf.Vec3f(-1.000000, 0.707107, 0.707107), Gf.Vec3f(1.000000, 0.555570, 0.831470), Gf.Vec3f(-1.000000, 0.555570, 0.831470), Gf.Vec3f(1.000000, 0.382683, 0.923880), Gf.Vec3f(-1.000000, 0.382683, 0.923880), Gf.Vec3f(1.000000, 0.195090, 0.980785), Gf.Vec3f(-1.000000, 0.195090, 0.980785), Gf.Vec3f(1.000000, -0.000000, 1.000000), Gf.Vec3f(-1.000000, -0.000000, 1.000000), Gf.Vec3f(1.000000, -0.195090, 0.980785), Gf.Vec3f(-1.000000, -0.195090, 0.980785), Gf.Vec3f(1.000000, -0.382683, 0.923880), Gf.Vec3f(-1.000000, -0.382683, 0.923880), Gf.Vec3f(1.000000, -0.555570, 0.831470), Gf.Vec3f(-1.000000, -0.555570, 0.831470), Gf.Vec3f(1.000000, -0.707107, 0.707107), Gf.Vec3f(-1.000000, -0.707107, 0.707107), Gf.Vec3f(1.000000, -0.831470, 0.555570), Gf.Vec3f(-1.000000, -0.831470, 0.555570), Gf.Vec3f(1.000000, -0.923880, 0.382683), Gf.Vec3f(-1.000000, -0.923880, 0.382683), Gf.Vec3f(1.000000, -0.980785, 0.195090), Gf.Vec3f(-1.000000, -0.980785, 0.195090), Gf.Vec3f(1.000000, -1.000000, -0.000000), Gf.Vec3f(-1.000000, -1.000000, -0.000000), Gf.Vec3f(1.000000, -0.980785, -0.195090), Gf.Vec3f(-1.000000, -0.980785, -0.195090), Gf.Vec3f(1.000000, -0.923879, -0.382684), Gf.Vec3f(-1.000000, -0.923879, -0.382684), Gf.Vec3f(1.000000, -0.831469, -0.555570), Gf.Vec3f(-1.000000, -0.831469, -0.555570), Gf.Vec3f(1.000000, -0.707107, -0.707107), Gf.Vec3f(-1.000000, -0.707107, -0.707107), Gf.Vec3f(1.000000, -0.555570, -0.831470), Gf.Vec3f(-1.000000, -0.555570, -0.831470), Gf.Vec3f(1.000000, -0.382683, -0.923880), Gf.Vec3f(-1.000000, -0.382683, -0.923880), Gf.Vec3f(1.000000, -0.195090, -0.980785), Gf.Vec3f(-1.000000, -0.195090, -0.980785)]

gRingMeshFaceVertexIndices = [1, 0, 2, 1, 2, 3, 3, 2, 4, 3, 4, 5, 5, 4, 6, 5, 6, 7, 7, 6, 8, 7, 8, 9, 9, 8, 10, 9, 10, 11, 11, 10, 12, 11, 12, 13, 13, 12, 14, 13, 14, 15, 15, 14, 16, 15, 16, 17, 17, 16, 18, 17, 18, 19, 19, 18, 20, 19, 20, 21, 21, 20, 22, 21, 22, 23, 23, 22, 24, 23, 24, 25, 25, 24, 26, 25, 26, 27, 27, 26, 28, 27, 28, 29, 29, 28, 30, 29, 30, 31, 31, 30, 32, 31, 32, 33, 33, 32, 34, 33, 34, 35, 35, 34, 36, 35, 36, 37, 37, 36, 38, 37, 38, 39, 39, 38, 40, 39, 40, 41, 41, 40, 42, 41, 42, 43, 43, 42, 44, 43, 44, 45, 45, 44, 46, 45, 46, 47, 47, 46, 48, 47, 48, 49, 49, 48, 50, 49, 50, 51, 51, 50, 52, 51, 52, 53, 53, 52, 54, 53, 54, 55, 55, 54, 56, 55, 56, 57, 57, 56, 58, 57, 58, 59, 59, 58, 60, 59, 60, 61, 61, 60, 62, 61, 62, 63, 63, 62, 0, 63, 0, 1]

gRingMeshFaceVertexCounts = [3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3]


gMaxCompressionInMeters = 0.1
gMaxDroopInMeters = 0.1
gTravelDistanceInMeters = gMaxCompressionInMeters + gMaxDroopInMeters


class AxesIndices:
    def __init__(self, upAxisIdx, forwardAxisIdx, sideAxisIdx):
        # x-axis = 0
        # y-axis = 1
        # z-axis = 2
        
        self.up = upAxisIdx
        self.forward = forwardAxisIdx
        self.side = sideAxisIdx


gYZXAxes = AxesIndices(1, 2, 0)


def _get_axis_token(axisIndex):
    if (axisIndex == 0):
        return PhysxSchema.Tokens.posX
    elif (axisIndex == 1):
        return PhysxSchema.Tokens.posY
    else:
        return PhysxSchema.Tokens.posZ


def _convert_gf_vec3_list(pointList, axesSrc: AxesIndices,
    axesDst: AxesIndices):

    for i in range(len(pointList)):
        point = pointList[i]
        
        dst = Gf.Vec3f(0.0)
        dst[axesDst.up] = point[axesSrc.up]
        dst[axesDst.forward] = point[axesSrc.forward]
        dst[axesDst.side] = point[axesSrc.side]
        
        pointList[i] = dst


def createCollisionGroups(stage, collisionGroupPaths):
    rootPath = str(stage.GetDefaultPrim().GetPath())

    collisionGroupVehicleChassisPath = rootPath + "/VehicleChassisCollisionGroup"
    collisionGroupVehicleWheelPath = rootPath + "/VehicleWheelCollisionGroup"
    collisionGroupVehicleGroundQueryPath = rootPath + "/VehicleGroundQueryGroup"
    collisionGroupGroundSurfacePath = rootPath + "/GroundSurfaceCollisionGroup"

    collisionGroupPaths.extend(["", "", "", ""])
    collisionGroupPaths[COLL_GROUP_VEHICLE_CHASSIS] = collisionGroupVehicleChassisPath
    collisionGroupPaths[COLL_GROUP_VEHICLE_WHEEL] = collisionGroupVehicleWheelPath
    collisionGroupPaths[COLL_GROUP_VEHICLE_GROUND_QUERY] = collisionGroupVehicleGroundQueryPath
    collisionGroupPaths[COLL_GROUP_GROUND_SURFACE] = collisionGroupGroundSurfacePath

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


def createMaterialsAndTireFrictionTables(stage, materialPaths, tireFrictionTablePaths):
    rootPath = str(stage.GetDefaultPrim().GetPath())

    # materials
    materialPaths.extend(["", ""])

    tarmacMaterialPath = rootPath + "/TarmacMaterial"
    materialPaths[MATERIAL_TARMAC] = tarmacMaterialPath
    UsdShade.Material.Define(stage, tarmacMaterialPath)
    tarmacMaterial = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(tarmacMaterialPath))
    tarmacMaterial.CreateStaticFrictionAttr(0.9)
    tarmacMaterial.CreateDynamicFrictionAttr(0.7)
    tarmacMaterial.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(tarmacMaterial.GetPrim())

    gravelMaterialPath = rootPath + "/GravelMaterial"
    materialPaths[MATERIAL_GRAVEL] = gravelMaterialPath
    UsdShade.Material.Define(stage, gravelMaterialPath)
    gravelMaterial = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(gravelMaterialPath))
    gravelMaterial.CreateStaticFrictionAttr(0.6)
    gravelMaterial.CreateDynamicFrictionAttr(0.6)
    gravelMaterial.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(gravelMaterial.GetPrim())

    # tire friction tables
    tireFrictionTablePaths.extend(["", ""])

    winterTireFrictionTablePath = rootPath + "/WinterTireFrictionTable"
    tireFrictionTablePaths[TIRE_FRICTION_TABLE_WINTER_TIRE] = winterTireFrictionTablePath
    winterTireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable.Define(stage, winterTireFrictionTablePath)
    winterTireFrictionTableGroundMaterialsRel = winterTireFrictionTable.CreateGroundMaterialsRel()
    winterTireFrictionTableGroundMaterialsRel.AddTarget(tarmacMaterialPath)
    winterTireFrictionTableGroundMaterialsRel.AddTarget(gravelMaterialPath)
    winterTireFrictionTable.CreateFrictionValuesAttr([0.75, 0.6])

    summerTireFrictionTablePath = rootPath + "/SummerTireFrictionTable"
    tireFrictionTablePaths[TIRE_FRICTION_TABLE_SUMMER_TIRE] = summerTireFrictionTablePath
    summerTireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable.Define(stage, summerTireFrictionTablePath)
    summerTireFrictionTableGroundMaterialsRel = summerTireFrictionTable.CreateGroundMaterialsRel()
    summerTireFrictionTableGroundMaterialsRel.AddTarget(tarmacMaterialPath)
    summerTireFrictionTableGroundMaterialsRel.AddTarget(gravelMaterialPath)
    summerTireFrictionTable.CreateFrictionValuesAttr([0.7, 0.6])


def createSceneBasics(stage, unitScale:UnitScale, axes = AxesIndices(1, 2, 0), timeStepsPerSecond = 60,
    useDeprecatedAPI=False):

    rootPath = str(stage.GetDefaultPrim().GetPath())
    lengthScale = unitScale.lengthScale

    if (axes.up == 0):
        # x-axis as up is not supported
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    elif (axes.up == 1):
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    else:
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    lightPos = Gf.Vec3f(0)
    lightPos[axes.up] = 11.5
    lightPos[axes.forward] = 0
    lightPos[axes.side] = 6.5

    UsdGeom.SetStageMetersPerUnit(stage, 1.0 / lengthScale)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0 / unitScale.massScale)

    # light
    sphereLight = UsdLux.SphereLight.Define(stage, rootPath + "/SphereLight")
    sphereLight.CreateRadiusAttr(lengthScale * 1.5)
    sphereLight.CreateIntensityAttr(30000)
    sphereLight.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
        lightPos * lengthScale)

    # physics scene
    scene = UsdPhysics.Scene.Define(stage, rootPath + "/PhysicsScene")
    gravity = Gf.Vec3f(0)
    gravity[axes.up] = -1
    scene.CreateGravityDirectionAttr(gravity)
    scene.CreateGravityMagnitudeAttr(10 * lengthScale)

    physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physxSceneAPI.GetTimeStepsPerSecondAttr().Set(timeStepsPerSecond)

    # vehicle context
    vehicleContextAPI = PhysxSchema.PhysxVehicleContextAPI.Apply(scene.GetPrim())
    vehicleContextAPI.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
    if (useDeprecatedAPI):
        upAxis = Gf.Vec3f(0)
        upAxis[axes.up] = 1
    
        forwardAxis = Gf.Vec3f(0)
        forwardAxis[axes.forward] = 1

        vehicleContextAPI.CreateUpAxisAttr(upAxis)
        vehicleContextAPI.CreateForwardAxisAttr(forwardAxis)
    else:
        verticalAxis = _get_axis_token(axes.up)
        longitudinalAxis = _get_axis_token(axes.forward)

        vehicleContextAPI.CreateVerticalAxisAttr(verticalAxis)
        vehicleContextAPI.CreateLongitudinalAxisAttr(longitudinalAxis)


def createGroundPlane(stage, unitScale:UnitScale, planeCollisionGroupPath, planeMaterialPath, 
    axes = AxesIndices(1, 2, 0)
):
    rootPath = str(stage.GetDefaultPrim().GetPath())
    lengthScale = unitScale.lengthScale

    groundPlanePath = rootPath + "/GroundPlane"
    groundPlane = UsdGeom.Mesh.Define(stage, groundPlanePath)
    groundPlane.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(0) * lengthScale)
    groundPlane.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Quatf(1, 0, 0, 0))
    groundPlane.CreateDisplayColorAttr([Gf.Vec3f(0.5, 0.5, 0.5)])

    faceVertexCounts = [4]

    faceVertexIndices = [0, 1, 2, 3]

    normals = [Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 1, 0)]
    _convert_gf_vec3_list(normals, gYZXAxes, axes)

    points = [
        Gf.Vec3f(-15, 0, -15) * lengthScale,
        Gf.Vec3f(15, 0, -15) * lengthScale,
        Gf.Vec3f(15, 0, 15) * lengthScale,
        Gf.Vec3f(-15, 0, 15) * lengthScale,
    ]
    _convert_gf_vec3_list(points, gYZXAxes, axes)

    groundPlane.CreateFaceVertexCountsAttr(faceVertexCounts)
    groundPlane.CreateFaceVertexIndicesAttr(faceVertexIndices)
    groundPlane.CreateNormalsAttr(normals)
    groundPlane.CreatePointsAttr(points)

    collisionPlanePath = groundPlanePath + "/CollisionPlane"
    collisionPlane = UsdGeom.Plane.Define(stage, collisionPlanePath)
    collisionPlane.CreatePurposeAttr(UsdGeom.Tokens.guide)
    if (axes.up == 0):
        collisionPlane.CreateAxisAttr(UsdGeom.Tokens.x)
    elif (axes.up == 1):
        collisionPlane.CreateAxisAttr(UsdGeom.Tokens.y)
    else:
        collisionPlane.CreateAxisAttr(UsdGeom.Tokens.z)

    collisionPlanePrim = collisionPlane.GetPrim()

    UsdPhysics.CollisionAPI.Apply(collisionPlanePrim)
    add_collision_to_collision_group(stage, collisionPlanePath, planeCollisionGroupPath)
    add_physics_material_to_prim(stage, collisionPlanePrim, Sdf.Path(planeMaterialPath))


def _setUpFrontWheel(
    prim,
    unitScale:UnitScale,
    useDeprecatedAPI=False
):
    lengthScale = unitScale.lengthScale
    lengthScaleSqr = lengthScale * lengthScale
    kgmsScale = unitScale.massScale * lengthScaleSqr
    wheelAPI = PhysxSchema.PhysxVehicleWheelAPI.Apply(prim)
    wheelAPI.CreateRadiusAttr(0.35 * lengthScale)
    wheelAPI.CreateWidthAttr(0.15 * lengthScale)
    wheelAPI.CreateMassAttr(20 * unitScale.massScale)
    wheelAPI.CreateMoiAttr(1.225 * kgmsScale)
    wheelAPI.CreateDampingRateAttr(0.25 * kgmsScale)
    if (useDeprecatedAPI):
        wheelAPI.CreateMaxBrakeTorqueAttr(3600 * kgmsScale)
        wheelAPI.CreateMaxHandBrakeTorqueAttr(0 * kgmsScale)
        wheelAPI.CreateMaxSteerAngleAttr(0.554264)
        wheelAPI.CreateToeAngleAttr(0)


def _setUpRearWheel(
    prim,
    unitScale:UnitScale,
    useDeprecatedAPI=False
):
    lengthScale = unitScale.lengthScale
    lengthScaleSqr = lengthScale * lengthScale
    kgmsScale = unitScale.massScale * lengthScaleSqr
    wheelAPI = PhysxSchema.PhysxVehicleWheelAPI.Apply(prim)
    wheelAPI.CreateRadiusAttr(0.35 * lengthScale)
    wheelAPI.CreateWidthAttr(0.15 * lengthScale)
    wheelAPI.CreateMassAttr(20 * unitScale.massScale)
    wheelAPI.CreateMoiAttr(1.225 * kgmsScale)
    wheelAPI.CreateDampingRateAttr(0.25 * kgmsScale)
    if (useDeprecatedAPI):
        wheelAPI.CreateMaxBrakeTorqueAttr(3600 * kgmsScale)
        wheelAPI.CreateMaxHandBrakeTorqueAttr(3000 * kgmsScale)
        wheelAPI.CreateMaxSteerAngleAttr(0)
        wheelAPI.CreateToeAngleAttr(0)


def _setUpFrontTire(
    prim,
    skipAttrWithDefaultValues,
    setAttrToDefaultValues,
    tireFrictionTablePath,
    unitScale:UnitScale,
    useDeprecatedAPI=False
):
    forceScale = unitScale.massScale * unitScale.lengthScale
    tireRestLoad = 450 * 10 * forceScale

    tireAPI = PhysxSchema.PhysxVehicleTireAPI.Apply(prim)
    if not skipAttrWithDefaultValues:
        if not setAttrToDefaultValues:
            if (useDeprecatedAPI):
                tireAPI.CreateLatStiffXAttr(2)
                tireAPI.CreateLatStiffYAttr(17)
                tireAPI.CreateLongitudinalStiffnessPerUnitGravityAttr(500 * unitScale.massScale)
                tireAPI.CreateCamberStiffnessPerUnitGravityAttr(0 * unitScale.massScale)
            else:
                tireAPI.CreateLateralStiffnessGraphAttr(Gf.Vec2f(2, 17 * tireRestLoad))
                tireAPI.CreateLongitudinalStiffnessAttr(5000 * forceScale)
                tireAPI.CreateCamberStiffnessAttr(0 * forceScale)
            tireAPI.CreateFrictionVsSlipGraphAttr([Gf.Vec2f(0.0, 1.0), Gf.Vec2f(0.1, 1.0), Gf.Vec2f(1.0, 1.0)])
        else:
            if (useDeprecatedAPI):
                tireAPI.CreateLatStiffXAttr(2)
                tireAPI.CreateLatStiffYAttr(17.095)
                tireAPI.CreateLongitudinalStiffnessPerUnitGravityAttr(500 * unitScale.massScale)
                tireAPI.CreateCamberStiffnessPerUnitGravityAttr(0 * unitScale.massScale)
            else:
                tireAPI.CreateLateralStiffnessGraphAttr(Gf.Vec2f(2, 17.095 * tireRestLoad))
                tireAPI.CreateLongitudinalStiffnessAttr(5000 * forceScale)
                tireAPI.CreateCamberStiffnessAttr(0 * forceScale)
            tireAPI.CreateFrictionVsSlipGraphAttr([Gf.Vec2f(0.0, 1.0), Gf.Vec2f(0.1, 1.0), Gf.Vec2f(1.0, 1.0)])
    tireFrictionTableRel = tireAPI.CreateFrictionTableRel()
    tireFrictionTableRel.AddTarget(tireFrictionTablePath)


def _setUpRearTire(
    prim,
    skipAttrWithDefaultValues,
    setAttrToDefaultValues,
    tireFrictionTablePath,
    unitScale:UnitScale,
    useDeprecatedAPI=False
):
    forceScale = unitScale.massScale * unitScale.lengthScale
    tireRestLoad = 450 * 10 * forceScale

    tireAPI = PhysxSchema.PhysxVehicleTireAPI.Apply(prim)
    if not skipAttrWithDefaultValues:
        if not setAttrToDefaultValues:
            if (useDeprecatedAPI):
                tireAPI.CreateLatStiffXAttr(2)
                tireAPI.CreateLatStiffYAttr(25)
                tireAPI.CreateLongitudinalStiffnessPerUnitGravityAttr(500 * unitScale.massScale)
                tireAPI.CreateCamberStiffnessPerUnitGravityAttr(0 * unitScale.massScale)
            else:
                tireAPI.CreateLateralStiffnessGraphAttr(Gf.Vec2f(2, 25 * tireRestLoad))
                tireAPI.CreateLongitudinalStiffnessAttr(5000 * forceScale)
                tireAPI.CreateCamberStiffnessAttr(0 * forceScale)
            tireAPI.CreateFrictionVsSlipGraphAttr([Gf.Vec2f(0.0, 1.0), Gf.Vec2f(0.1, 1.0), Gf.Vec2f(1.0, 1.0)])
        else:
            if (useDeprecatedAPI):
                tireAPI.CreateLatStiffXAttr(2)
                tireAPI.CreateLatStiffYAttr(17.095)
                tireAPI.CreateLongitudinalStiffnessPerUnitGravityAttr(500 * unitScale.massScale)
                tireAPI.CreateCamberStiffnessPerUnitGravityAttr(0 * unitScale.massScale)
            else:
                tireAPI.CreateLateralStiffnessGraphAttr(Gf.Vec2f(2, 17.095 * tireRestLoad))
                tireAPI.CreateLongitudinalStiffnessAttr(5000 * forceScale)
                tireAPI.CreateCamberStiffnessAttr(0 * forceScale)
            tireAPI.CreateFrictionVsSlipGraphAttr([Gf.Vec2f(0.0, 1.0), Gf.Vec2f(0.1, 1.0), Gf.Vec2f(1.0, 1.0)])
    tireFrictionTableRel = tireAPI.CreateFrictionTableRel()
    tireFrictionTableRel.AddTarget(tireFrictionTablePath)


def _setUpFrontSuspension(
    prim,
    unitScale:UnitScale,
    useDeprecatedAPI=False
):
    lengthScale = unitScale.lengthScale
    suspensionAPI = PhysxSchema.PhysxVehicleSuspensionAPI.Apply(prim)
    suspensionAPI.CreateSpringStrengthAttr(45000 * unitScale.massScale)
    suspensionAPI.CreateSpringDamperRateAttr(4500 * unitScale.massScale)
    if (useDeprecatedAPI):
        suspensionAPI.CreateMaxCompressionAttr(gMaxCompressionInMeters * lengthScale)
        suspensionAPI.CreateMaxDroopAttr(gMaxDroopInMeters * lengthScale)
        suspensionAPI.CreateCamberAtRestAttr(0)
        suspensionAPI.CreateCamberAtMaxCompressionAttr(0)
        suspensionAPI.CreateCamberAtMaxDroopAttr(0)
    else:
        suspensionAPI.CreateTravelDistanceAttr(gTravelDistanceInMeters * lengthScale)


def _setUpRearSuspension(
    prim,
    unitScale:UnitScale,
    useDeprecatedAPI=False
):
    _setUpFrontSuspension(prim, unitScale, useDeprecatedAPI)


class WheelComponentPaths:
    def __init__(self):
        self.wheelPaths = []
        self.tirePaths = []
        self.suspensionPaths = []


def createWheelComponents(
    stage,
    unitScale:UnitScale,
    tireFrictionTablePath,
    componentPathsOut,
    skipAttrWithDefaultValues=False,
    setAttrToDefaultValues=False,
    useDeprecatedAPI=False
):
    rootPath = str(stage.GetDefaultPrim().GetPath())

    lengthScale = unitScale.lengthScale
    lengthScaleSqr = lengthScale * lengthScale
    kgmsScale = unitScale.massScale * lengthScaleSqr

    # wheels
    componentPathsOut.wheelPaths.extend(["", ""])

    frontWheelPath = rootPath + "/FrontWheel"
    componentPathsOut.wheelPaths[WHEEL_FRONT] = frontWheelPath
    frontWheelPrim = UsdGeom.Scope.Define(stage, frontWheelPath).GetPrim()
    _setUpFrontWheel(frontWheelPrim, unitScale, useDeprecatedAPI)

    rearWheelPath = rootPath + "/RearWheel"
    componentPathsOut.wheelPaths[WHEEL_REAR] = rearWheelPath
    rearWheelPrim = UsdGeom.Scope.Define(stage, rearWheelPath).GetPrim()
    _setUpRearWheel(rearWheelPrim, unitScale, useDeprecatedAPI)

    # tires
    componentPathsOut.tirePaths.extend(["", ""])

    frontTirePath = rootPath + "/FrontTire"
    componentPathsOut.tirePaths[TIRE_FRONT] = frontTirePath
    frontTirePrim = UsdGeom.Scope.Define(stage, frontTirePath).GetPrim()
    _setUpFrontTire(frontTirePrim, skipAttrWithDefaultValues, setAttrToDefaultValues,
        tireFrictionTablePath, unitScale, useDeprecatedAPI)

    rearTirePath = rootPath + "/RearTire"
    componentPathsOut.tirePaths[TIRE_REAR] = rearTirePath
    rearTirePrim = UsdGeom.Scope.Define(stage, rearTirePath).GetPrim()
    _setUpRearTire(rearTirePrim, skipAttrWithDefaultValues, setAttrToDefaultValues,
        tireFrictionTablePath, unitScale, useDeprecatedAPI)

    # suspensions
    componentPathsOut.suspensionPaths.extend(["", ""])

    frontSuspensionPath = rootPath + "/FrontSuspension"
    componentPathsOut.suspensionPaths[SUSPENSION_FRONT] = frontSuspensionPath
    frontSuspensionPrim = UsdGeom.Scope.Define(stage, frontSuspensionPath).GetPrim()
    _setUpFrontSuspension(frontSuspensionPrim, unitScale, useDeprecatedAPI)

    rearSuspensionPath = rootPath + "/RearSuspension"
    componentPathsOut.suspensionPaths[SUSPENSION_REAR] = rearSuspensionPath
    rearSuspensionPrim = UsdGeom.Scope.Define(stage, rearSuspensionPath).GetPrim()
    _setUpRearSuspension(rearSuspensionPrim, unitScale, useDeprecatedAPI)


def _setUpEngine(
    prim,
    skipAttrWithDefaultValues,
    setAttrToDefaultValues,
    unitScale:UnitScale
):
    lengthScale = unitScale.lengthScale
    lengthScaleSqr = lengthScale * lengthScale
    kgmsScale = unitScale.massScale * lengthScaleSqr
    engineAPI = PhysxSchema.PhysxVehicleEngineAPI.Apply(prim)
    if not skipAttrWithDefaultValues:
        if not setAttrToDefaultValues:
            engineAPI.CreateMoiAttr(1.0 * kgmsScale)
            engineAPI.CreatePeakTorqueAttr(1000.0 * kgmsScale)
            engineAPI.CreateMaxRotationSpeedAttr(600.0)
            engineAPI.CreateTorqueCurveAttr([Gf.Vec2f(0.0, 0.8), Gf.Vec2f(0.33, 1.0), Gf.Vec2f(1.0, 0.8)])
            engineAPI.CreateDampingRateFullThrottleAttr(0.15 * kgmsScale)
            engineAPI.CreateDampingRateZeroThrottleClutchEngagedAttr(2.0 * kgmsScale)
            engineAPI.CreateDampingRateZeroThrottleClutchDisengagedAttr(0.35 * kgmsScale)
        else:
            engineAPI.CreateMoiAttr(1.0 * kgmsScale)
            engineAPI.CreatePeakTorqueAttr(500.0 * kgmsScale)
            engineAPI.CreateMaxRotationSpeedAttr(600.0)
            engineAPI.CreateTorqueCurveAttr([Gf.Vec2f(0.0, 0.8), Gf.Vec2f(0.33, 1.0), Gf.Vec2f(1.0, 0.8)])
            engineAPI.CreateDampingRateFullThrottleAttr(0.15 * kgmsScale)
            engineAPI.CreateDampingRateZeroThrottleClutchEngagedAttr(2.0 * kgmsScale)
            engineAPI.CreateDampingRateZeroThrottleClutchDisengagedAttr(0.35 * kgmsScale)


def _setUpGears(
    prim,
    skipAttrWithDefaultValues,
    setAttrToDefaultValues
):
    gearsAPI = PhysxSchema.PhysxVehicleGearsAPI.Apply(prim)
    if not skipAttrWithDefaultValues:
        if not setAttrToDefaultValues:
            gearsAPI.CreateRatiosAttr([-4.0, 4.0, 2.0, 1.5, 1.1, 1.0])
            gearsAPI.CreateRatioScaleAttr(4.0)
            gearsAPI.CreateSwitchTimeAttr(0.5)
        else:
            gearsAPI.CreateRatiosAttr([-4.0, 4.0, 2.0, 1.5, 1.1, 1.0])
            gearsAPI.CreateRatioScaleAttr(4.0)
            gearsAPI.CreateSwitchTimeAttr(0.5)


def _setUpAutoGearBox(
    prim,
    skipAttrWithDefaultValues,
    setAttrToDefaultValues
):
    autoGearBoxAPI = PhysxSchema.PhysxVehicleAutoGearBoxAPI.Apply(prim)
    if not skipAttrWithDefaultValues:
        if not setAttrToDefaultValues:
            autoGearBoxAPI.CreateUpRatiosAttr([0.65, 0.65, 0.65, 0.65])
            autoGearBoxAPI.CreateDownRatiosAttr([0.5, 0.5, 0.5, 0.5])
            autoGearBoxAPI.CreateLatencyAttr(2.0)
        else:
            autoGearBoxAPI.CreateUpRatiosAttr([0.65, 0.65, 0.65, 0.65])
            autoGearBoxAPI.CreateDownRatiosAttr([0.5, 0.5, 0.5, 0.5])
            autoGearBoxAPI.CreateLatencyAttr(2.0)


def _setUpClutch(
    prim,
    skipAttrWithDefaultValues,
    setAttrToDefaultValues,
    unitScale:UnitScale
):
    lengthScale = unitScale.lengthScale
    lengthScaleSqr = lengthScale * lengthScale
    kgmsScale = unitScale.massScale * lengthScaleSqr
    clutchAPI = PhysxSchema.PhysxVehicleClutchAPI.Apply(prim)
    if not skipAttrWithDefaultValues:
        if not setAttrToDefaultValues:
            clutchAPI.CreateStrengthAttr(10.0 * kgmsScale)
        else:
            clutchAPI.CreateStrengthAttr(10.0 * kgmsScale)


def _setUpDriveBasic(
    prim,
    skipAttrWithDefaultValues,
    setAttrToDefaultValues,
    unitScale:UnitScale,
    useDeprecatedAPI=False
):
    lengthScale = unitScale.lengthScale
    lengthScaleSqr = lengthScale * lengthScale
    kgmsScale = unitScale.massScale * lengthScaleSqr
    driveAPI = PhysxSchema.PhysxVehicleDriveBasicAPI.Apply(prim)

    if not skipAttrWithDefaultValues:
        if not setAttrToDefaultValues:
            if (useDeprecatedAPI):
                driveAPI.CreatePeakTorqueAttr(250 * kgmsScale)
            else:
                driveAPI.CreatePeakTorqueAttr(500 * kgmsScale)
        else:
            if (useDeprecatedAPI):
                driveAPI.CreatePeakTorqueAttr(500 * kgmsScale)
            else:
                driveAPI.CreatePeakTorqueAttr(1000 * kgmsScale)


class DriveComponentPaths:
    def __init__(self):
        self.enginePath = None
        self.gearsPath = None
        self.autoGearBoxPath = None
        self.clutchPath = None
        self.drivePath = None


def createDriveComponents(
    stage,
    unitScale:UnitScale,
    componentPathsOut,
    driveMode=DRIVE_STANDARD,
    createAutoGearBox=True,
    skipAttrWithDefaultValues=False,
    setAttrToDefaultValues=False,
    useDeprecatedAPI=False
):
    rootPath = str(stage.GetDefaultPrim().GetPath())

    lengthScale = unitScale.lengthScale
    lengthScaleSqr = lengthScale * lengthScale
    kgmsScale = unitScale.massScale * lengthScaleSqr

    if driveMode == DRIVE_STANDARD:
        # engine
        enginePath = rootPath + "/Engine"
        componentPathsOut.enginePath = enginePath
        enginePrim = UsdGeom.Scope.Define(stage, enginePath).GetPrim()
        _setUpEngine(enginePrim, skipAttrWithDefaultValues, setAttrToDefaultValues, unitScale)

        # gears
        gearsPath = rootPath + "/Gears"
        componentPathsOut.gearsPath = gearsPath
        gearsPrim = UsdGeom.Scope.Define(stage, gearsPath).GetPrim()
        _setUpGears(gearsPrim, skipAttrWithDefaultValues, setAttrToDefaultValues)

        # auto gear box
        autoGearBoxPath = rootPath + "/AutoGearBox"
        componentPathsOut.autoGearBoxPath = autoGearBoxPath
        if createAutoGearBox:
            autoGearBoxPrim = UsdGeom.Scope.Define(stage, autoGearBoxPath).GetPrim()
            _setUpAutoGearBox(autoGearBoxPrim, skipAttrWithDefaultValues, setAttrToDefaultValues)

        # clutch
        clutchPath = rootPath + "/Clutch"
        componentPathsOut.clutchPath = clutchPath
        clutchPrim = UsdGeom.Scope.Define(stage, clutchPath).GetPrim()
        _setUpClutch(clutchPrim, skipAttrWithDefaultValues, setAttrToDefaultValues, unitScale)

        # drive
        drivePath = rootPath + "/DriveStandard"
        componentPathsOut.drivePath = drivePath
        drivePrim = UsdGeom.Scope.Define(stage, drivePath).GetPrim()
        driveAPI = PhysxSchema.PhysxVehicleDriveStandardAPI.Apply(drivePrim)
        engineRel = driveAPI.CreateEngineRel()
        engineRel.AddTarget(enginePath)
        gearsRel = driveAPI.CreateGearsRel()
        gearsRel.AddTarget(gearsPath)
        if createAutoGearBox:
            autoGearBoxRel = driveAPI.CreateAutoGearBoxRel()
            autoGearBoxRel.AddTarget(autoGearBoxPath)
        clutchRel = driveAPI.CreateClutchRel()
        clutchRel.AddTarget(clutchPath)
    else:
        # drive
        drivePath = rootPath + "/DriveBasic"
        componentPathsOut.drivePath = drivePath
        drivePrim = UsdGeom.Scope.Define(stage, drivePath).GetPrim()
        _setUpDriveBasic(drivePrim, skipAttrWithDefaultValues, setAttrToDefaultValues, unitScale,
            useDeprecatedAPI)


def _createCylinderMeshWithConvexHull(stage, path, axes = AxesIndices(1, 2, 0)):
    mesh = UsdGeom.Mesh.Define(stage, path)

    mesh.CreateDoubleSidedAttr(False)
    if ((axes.up == 1) and (axes.forward == 2) and (axes.side == 0)):
        ringMeshPoints = gRingMeshPoints
    else:
        ringMeshPoints = gRingMeshPoints.copy()
        _convert_gf_vec3_list(ringMeshPoints, gYZXAxes, axes)
    mesh.CreatePointsAttr(ringMeshPoints)
    mesh.CreateFaceVertexIndicesAttr(gRingMeshFaceVertexIndices)
    mesh.CreateFaceVertexCountsAttr(gRingMeshFaceVertexCounts)
    
    meshPrim = mesh.GetPrim()
    meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(meshPrim)
    meshCollisionAPI.CreateApproximationAttr("convexHull")
    
    return mesh


def create4WheeledCar(
    stage,
    unitScale:UnitScale,
    vehiclePath,
    vehiclePosition,
    createCollisionShapesForWheels,
    wheelIsDriven,
    wheelPaths,
    tirePaths,
    suspensionPaths,
    tireFrictionTablePath,
    drivePath,
    driveMode,
    createAutoGearBox,
    chassisCollisionGroupPath,
    wheelCollisionGroupPath,
    groundQueryCollisionGroupPath,
    wheelAttachmentPathsOut,
    addChassisRenderMesh=True,
    addChassisCollisionBox=True,
    placeWheelsAsGrandchildren=False,
    placeWheelCollisionShapesAtRoot=False,
    useMeshAsWheelCollisionShape=False,
    noWheelRootTransformManagement=False,
    omitControllers=False,
    skipAttrWithDefaultValues=False,
    setAttrToDefaultValues=False,
    axes=AxesIndices(1, 2, 0),
    enabled=True,
    useRaycasts=True,
    useDeprecatedAPI=False,
    configureAsTank=False,
    useAckermannCorrection=False,
    referenceFrameIsCoM=False,
):

    # note:
    # - tireFrictionTablePath can be set to None if tirePaths is not None
    # - driveMode and createAutoGearBox do not matter if drivePath is set with the exception
    #   that they will still impact whether target gear is set to first or automatic mode
    # - placeWheelCollisionShapesAtRoot has no effect if createCollisionShapesForWheels is False
    # - useMeshAsWheelCollisionShape has no effect if createCollisionShapesForWheels is False
    # - noWheelRootTransformManagement: if True, no collision shapes will be generated and the wheel
    #   root transform will not be controlled by the simulation (a vanilla prim will be used as root)
    # - configureAsTank will only have an effect if drive mode DRIVE_STANDARD is used
    # - useAckermannCorrection will only have an effect if drive mode DRIVE_BASIC/DRIVE_STANDARD is used

    lengthScale = unitScale.lengthScale
    lengthScaleSqr = lengthScale * lengthScale
    kgmsScale = unitScale.massScale * lengthScaleSqr

    vehicle = UsdGeom.Xform.Define(stage, vehiclePath)
    vehicle.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
        Gf.Vec3f(vehiclePosition[0], vehiclePosition[1], vehiclePosition[2])
    )
    vehicle.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Quatf(1, 0, 0, 0))

    vehiclePrim = vehicle.GetPrim()

    set_custom_metadata(vehiclePrim, PhysxSchema.Tokens.referenceFrameIsCenterOfMass, referenceFrameIsCoM)

    UsdPhysics.RigidBodyAPI.Apply(vehiclePrim)

    vehicleMassBoxDim = Gf.Vec3f(0)
    vehicleMassBoxDim[axes.up] = 1.0 * lengthScale
    vehicleMassBoxDim[axes.forward] = 4.8 * lengthScale
    vehicleMassBoxDim[axes.side] = 1.8 * lengthScale
    centerOfMassToGround = 0.75 * lengthScale
    chassisHalfHeight = 0.7 * lengthScale
    chassisDistToGround = 0.3 * lengthScale
    chassisCenterToGround = chassisHalfHeight + chassisDistToGround
    wheelRadius = 0.35 * lengthScale
    wheelWidth = 0.15 * lengthScale
    mass = 1800 * unitScale.massScale
    massAPI = UsdPhysics.MassAPI.Apply(vehiclePrim)
    massAPI.CreateMassAttr(mass)
    centerOfMassOffset = Gf.Vec3f(0)
    centerOfMassOffset[axes.up] = centerOfMassToGround - chassisCenterToGround
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
    rigidBodyAPI.CreateSleepThresholdAttr(0)
    rigidBodyAPI.CreateStabilizationThresholdAttr(0)
    rigidBodyAPI.CreateDisableGravityAttr(True)

    vehicleAPI = PhysxSchema.PhysxVehicleAPI.Apply(vehiclePrim)
    if (not skipAttrWithDefaultValues):
        vehicleAPI.CreateVehicleEnabledAttr(enabled)

    if drivePath is not None:
        driveRel = vehicleAPI.CreateDriveRel()
        driveRel.AddTarget(drivePath)
    elif (driveMode == DRIVE_BASIC):
        _setUpDriveBasic(vehiclePrim, skipAttrWithDefaultValues, setAttrToDefaultValues, unitScale, useDeprecatedAPI)
    elif (driveMode == DRIVE_STANDARD):
        PhysxSchema.PhysxVehicleDriveStandardAPI.Apply(vehiclePrim)
        _setUpEngine(vehiclePrim, skipAttrWithDefaultValues, setAttrToDefaultValues, unitScale)
        _setUpGears(vehiclePrim, skipAttrWithDefaultValues, setAttrToDefaultValues)
        if (createAutoGearBox):
            _setUpAutoGearBox(vehiclePrim, skipAttrWithDefaultValues, setAttrToDefaultValues)
        _setUpClutch(vehiclePrim, skipAttrWithDefaultValues, setAttrToDefaultValues, unitScale)

    if not skipAttrWithDefaultValues:
        if not setAttrToDefaultValues:
            vehicleAPI.CreateSubStepThresholdLongitudinalSpeedAttr(5.0 * lengthScale)
            vehicleAPI.CreateLowForwardSpeedSubStepCountAttr(3)
            vehicleAPI.CreateHighForwardSpeedSubStepCountAttr(1)
            if (useDeprecatedAPI):
                vehicleAPI.CreateMinLongitudinalSlipDenominatorAttr(4.0 * lengthScale)
            else:
                vehicleAPI.CreateMinPassiveLongitudinalSlipDenominatorAttr(4.0 * lengthScale)
                vehicleAPI.CreateMinActiveLongitudinalSlipDenominatorAttr(0.1 * lengthScale)
                vehicleAPI.CreateMinLateralSlipDenominatorAttr(1.0 * lengthScale)
            vehicleAPI.CreateLongitudinalStickyTireThresholdSpeedAttr().Set(0.2 * lengthScale)
            vehicleAPI.CreateLongitudinalStickyTireThresholdTimeAttr().Set(1)
            vehicleAPI.CreateLongitudinalStickyTireDampingAttr().Set(200)
            vehicleAPI.CreateLateralStickyTireThresholdSpeedAttr().Set(0.2 * lengthScale)
            vehicleAPI.CreateLateralStickyTireThresholdTimeAttr().Set(1)
            vehicleAPI.CreateLateralStickyTireDampingAttr().Set(20)
        else:
            vehicleAPI.CreateSubStepThresholdLongitudinalSpeedAttr(5.0 * lengthScale)
            vehicleAPI.CreateLowForwardSpeedSubStepCountAttr(3)
            vehicleAPI.CreateHighForwardSpeedSubStepCountAttr(1)
            if (useDeprecatedAPI):
                vehicleAPI.CreateMinLongitudinalSlipDenominatorAttr(4.0 * lengthScale)
            else:
                vehicleAPI.CreateMinPassiveLongitudinalSlipDenominatorAttr(4.0 * lengthScale)
                vehicleAPI.CreateMinActiveLongitudinalSlipDenominatorAttr(0.1 * lengthScale)
                vehicleAPI.CreateMinLateralSlipDenominatorAttr(1.0 * lengthScale)
            vehicleAPI.CreateLongitudinalStickyTireThresholdSpeedAttr().Set(0.2 * lengthScale)
            vehicleAPI.CreateLongitudinalStickyTireThresholdTimeAttr().Set(1)
            vehicleAPI.CreateLongitudinalStickyTireDampingAttr().Set(200)
            vehicleAPI.CreateLateralStickyTireThresholdSpeedAttr().Set(0.2 * lengthScale)
            vehicleAPI.CreateLateralStickyTireThresholdTimeAttr().Set(1)
            vehicleAPI.CreateLateralStickyTireDampingAttr().Set(20)

    if (useRaycasts):
        if not skipAttrWithDefaultValues:
            vehicleAPI.CreateSuspensionLineQueryTypeAttr(PhysxSchema.Tokens.raycast)
    else:
        vehicleAPI.CreateSuspensionLineQueryTypeAttr(PhysxSchema.Tokens.sweep)

    if ((drivePath is not None) or (driveMode != DRIVE_NONE)) and (not omitControllers):
        if ((driveMode == DRIVE_STANDARD) and (configureAsTank)):
            vehicleTankControllerAPI = PhysxSchema.PhysxVehicleTankControllerAPI.Apply(vehiclePrim)
            vehicleTankControllerAPI.CreateThrust0Attr(0)
            vehicleTankControllerAPI.CreateThrust1Attr(0)

            vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI(vehicleTankControllerAPI)  # tank controller also applies base controller
        else:
            vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI.Apply(vehiclePrim)

        vehicleControllerAPI.CreateAcceleratorAttr(0)
        if (useDeprecatedAPI):
            vehicleControllerAPI.CreateBrakeAttr(0)
            vehicleControllerAPI.CreateHandbrakeAttr(0)

            vehicleControllerAPI.CreateSteerLeftAttr(0)
            vehicleControllerAPI.CreateSteerRightAttr(0)
        else:
            vehicleControllerAPI.CreateBrake0Attr(0)
            vehicleControllerAPI.CreateBrake1Attr(0)

            vehicleControllerAPI.CreateSteerAttr(0)

        if ((driveMode == DRIVE_STANDARD) and createAutoGearBox):
            vehicleControllerAPI.CreateTargetGearAttr(VEHICLE_AUTOMATIC_TRANSMISSION_GEAR_VALUE)
        else:
            vehicleControllerAPI.CreateTargetGearAttr(1)

    if placeWheelsAsGrandchildren:
        wheelParentPath = vehiclePath + "/Wheels"
        UsdGeom.Scope.Define(stage, wheelParentPath)
        wheelAttachmentPathPrefix = wheelParentPath
    else:
        wheelAttachmentPathPrefix = vehiclePath

    # front left wheel, front right wheel, rear left wheel, rear right wheel
    wheelAttachmentPaths = [
        wheelAttachmentPathPrefix + "/FrontLeftWheel",
        wheelAttachmentPathPrefix + "/FrontRightWheel",
        wheelAttachmentPathPrefix + "/RearLeftWheel",
        wheelAttachmentPathPrefix + "/RearRightWheel",
    ]
    wheelAttachmentPathsOut.extend(["", "", "", ""])
    wheelAttachmentPathsOut[WHEEL_FRONT_LEFT] = wheelAttachmentPaths[0]
    wheelAttachmentPathsOut[WHEEL_FRONT_RIGHT] = wheelAttachmentPaths[1]
    wheelAttachmentPathsOut[WHEEL_REAR_LEFT] = wheelAttachmentPaths[2]
    wheelAttachmentPathsOut[WHEEL_REAR_RIGHT] = wheelAttachmentPaths[3]

    wheelAttachmentIndices = [
        WHEEL_FRONT_LEFT,
        WHEEL_FRONT_RIGHT,
        WHEEL_REAR_LEFT,
        WHEEL_REAR_RIGHT
    ]

    if referenceFrameIsCoM:
        wheelRestPositionY = wheelRadius - centerOfMassToGround
    else:
        wheelRestPositionY = wheelRadius - chassisCenterToGround
    longitudinalWheelOffset = 1.6 * lengthScale
    lateralWheelOffset = 0.8 * lengthScale
    wheelPositions = [
        Gf.Vec3f(lateralWheelOffset, wheelRestPositionY, longitudinalWheelOffset),
        Gf.Vec3f(-lateralWheelOffset, wheelRestPositionY, longitudinalWheelOffset),
        Gf.Vec3f(lateralWheelOffset, wheelRestPositionY, -longitudinalWheelOffset),
        Gf.Vec3f(-lateralWheelOffset, wheelRestPositionY, -longitudinalWheelOffset),
    ]
    _convert_gf_vec3_list(wheelPositions, gYZXAxes, axes)

    suspensionFramePositionY = wheelRestPositionY + (gMaxCompressionInMeters * lengthScale)
    suspensionFramePositions = [
        Gf.Vec3f(lateralWheelOffset, suspensionFramePositionY, longitudinalWheelOffset),
        Gf.Vec3f(-lateralWheelOffset, suspensionFramePositionY, longitudinalWheelOffset),
        Gf.Vec3f(lateralWheelOffset, suspensionFramePositionY, -longitudinalWheelOffset),
        Gf.Vec3f(-lateralWheelOffset, suspensionFramePositionY, -longitudinalWheelOffset),
    ]
    _convert_gf_vec3_list(suspensionFramePositions, gYZXAxes, axes)

    wheelIsFront = [
        True,
        True,
        False,
        False,
    ]

    drivenWheelIndexList = []

    for i in range(4):
        wheelPos = wheelPositions[i]
        suspFramePos = suspensionFramePositions[i]
        vehicleWheelPath = wheelAttachmentPaths[i]
        if noWheelRootTransformManagement:
            vehicleWheel = UsdGeom.Scope.Define(stage, vehicleWheelPath).GetPrim()
        else:
            if createCollisionShapesForWheels and placeWheelCollisionShapesAtRoot:
                if useMeshAsWheelCollisionShape:
                    vehicleWheel = _createCylinderMeshWithConvexHull(stage, vehicleWheelPath, axes)
                else:
                    vehicleWheel = UsdGeom.Cylinder.Define(stage, vehicleWheelPath)
            else:
                vehicleWheel = UsdGeom.Xform.Define(stage, vehicleWheelPath)

            if referenceFrameIsCoM:
                vehicleWheel.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(wheelPos + centerOfMassOffset)
            else:
                vehicleWheel.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(wheelPos)

        vehicleWheelPrim = vehicleWheel.GetPrim()
        wheelAttachmentAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI.Apply(vehicleWheelPrim)

        if (wheelPaths is not None):
            wheelRel = wheelAttachmentAPI.CreateWheelRel()
            wheelRel.AddTarget(wheelPaths[i])
        else:
            if (wheelIsFront[i]):
                _setUpFrontWheel(vehicleWheelPrim, unitScale, useDeprecatedAPI)
            else:
                _setUpRearWheel(vehicleWheelPrim, unitScale, useDeprecatedAPI)

        if (tirePaths is not None):
            tireRel = wheelAttachmentAPI.CreateTireRel()
            tireRel.AddTarget(tirePaths[i])
        else:
            if (wheelIsFront[i]):
                _setUpFrontTire(vehicleWheelPrim, skipAttrWithDefaultValues, setAttrToDefaultValues, tireFrictionTablePath,
                                unitScale, useDeprecatedAPI)
            else:
                _setUpRearTire(vehicleWheelPrim, skipAttrWithDefaultValues, setAttrToDefaultValues, tireFrictionTablePath,
                               unitScale, useDeprecatedAPI)

        if (suspensionPaths is not None):
            suspensionRel = wheelAttachmentAPI.CreateSuspensionRel()
            suspensionRel.AddTarget(suspensionPaths[i])
        else:
            if (wheelIsFront[i]):
                _setUpFrontSuspension(vehicleWheelPrim, unitScale, useDeprecatedAPI)
            else:
                _setUpRearSuspension(vehicleWheelPrim, unitScale, useDeprecatedAPI)

        collisionGroupRel = wheelAttachmentAPI.CreateCollisionGroupRel()
        collisionGroupRel.AddTarget(groundQueryCollisionGroupPath)

        suspTravelDir = Gf.Vec3f(0)
        suspTravelDir[axes.up] = -1
        wheelAttachmentAPI.CreateSuspensionTravelDirectionAttr(suspTravelDir)

        if (useDeprecatedAPI):
            wheelAttachmentAPI.CreateSuspensionForceAppPointOffsetAttr(wheelPos)
            wheelAttachmentAPI.CreateTireForceAppPointOffsetAttr(wheelPos)
            wheelAttachmentAPI.CreateWheelCenterOfMassOffsetAttr(wheelPos)
        else:
            wheelAttachmentAPI.CreateSuspensionFramePositionAttr(suspFramePos)
            wheelAttachmentAPI.CreateSuspensionFrameOrientationAttr(Gf.Quatf(1, 0, 0, 0))

        wheelIndex = wheelAttachmentIndices[i]
        if (useDeprecatedAPI):
            wheelAttachmentAPI.CreateDrivenAttr(wheelIsDriven[i])
        else:
            wheelAttachmentAPI.CreateIndexAttr(wheelIndex)

        if (wheelIsDriven[i]):
            drivenWheelIndexList.append(wheelIndex)

        if (not useDeprecatedAPI):
            suspensionComplianceAPI = PhysxSchema.PhysxVehicleSuspensionComplianceAPI.Apply(vehicleWheelPrim)
            # empty values for some attributes just to make sure they exist for tests that might want to make
            # use of them
            suspensionComplianceAPI.CreateWheelToeAngleAttr([])
            suspensionComplianceAPI.CreateWheelCamberAngleAttr([])
            forceAppPoint = wheelPos - suspFramePos
            forceAppPointEntry = Gf.Vec4f(0.0, forceAppPoint[0], forceAppPoint[1], forceAppPoint[2])
            suspensionComplianceAPI.CreateSuspensionForceAppPointAttr([forceAppPointEntry])
            suspensionComplianceAPI.CreateTireForceAppPointAttr([forceAppPointEntry])

        if ((drivePath is None) and (driveMode == DRIVE_NONE)) and (not omitControllers):
            wheelControllerAPI = PhysxSchema.PhysxVehicleWheelControllerAPI.Apply(vehicleWheelPrim)
            wheelControllerAPI.CreateDriveTorqueAttr(0)
            wheelControllerAPI.CreateBrakeTorqueAttr(0)
            wheelControllerAPI.CreateSteerAngleAttr(0)

        if (not noWheelRootTransformManagement) and createCollisionShapesForWheels:
            if placeWheelCollisionShapesAtRoot:
                collisionGeom = vehicleWheel
            else:
                vehicleWheelCollPath = vehicleWheelPath + "/Collision"
                if useMeshAsWheelCollisionShape:
                    collisionGeom = _createCylinderMeshWithConvexHull(stage, vehicleWheelCollPath, axes)
                else:
                    collisionGeom = UsdGeom.Cylinder.Define(stage, vehicleWheelCollPath)

            if (useMeshAsWheelCollisionShape):
                scale = Gf.Vec3f(1)
                scale[axes.up] = wheelRadius
                scale[axes.forward] = wheelRadius
                scale[axes.side] = wheelWidth * 0.5
                collisionGeom.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(scale)
            else:
                collisionGeom.CreateHeightAttr(wheelWidth)
                collisionGeom.CreateRadiusAttr(wheelRadius)
                if (axes.side == 0):
                    collisionGeom.CreateAxisAttr(UsdGeom.Tokens.x)
                elif (axes.side == 1):
                    collisionGeom.CreateAxisAttr(UsdGeom.Tokens.y)
                else:
                    collisionGeom.CreateAxisAttr(UsdGeom.Tokens.z)
                # if height or radius is authored, USD expects extent to be authored too
                cylExtent = UsdGeom.Cylinder.ComputeExtentFromPlugins(collisionGeom, 0)
                collisionGeom.CreateExtentAttr(cylExtent)

            # collisionGeom.CreatePurposeAttr(UsdGeom.Tokens.guide)

            collisionGeomPrim = collisionGeom.GetPrim()

            collisionAPI = UsdPhysics.CollisionAPI.Apply(collisionGeomPrim)
            add_collision_to_collision_group(stage, collisionGeomPrim.GetPrimPath(), wheelCollisionGroupPath)

            physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(collisionGeomPrim)
            physxCollisionAPI.CreateRestOffsetAttr(0.0 * lengthScale)
            physxCollisionAPI.CreateContactOffsetAttr(0.02 * lengthScale)

    if (not useDeprecatedAPI):
        defaultMaxBrakeTorque = 3600 * kgmsScale

        if ((driveMode == DRIVE_STANDARD) and (configureAsTank)):
            # set up one brake configuration for the wheels of the left track
            brakes0API = PhysxSchema.PhysxVehicleBrakesAPI.Apply(vehiclePrim, PhysxSchema.Tokens.brakes0)
            brakes0API.CreateWheelsAttr([WHEEL_FRONT_LEFT, WHEEL_REAR_LEFT])
            brakes0API.CreateMaxBrakeTorqueAttr(defaultMaxBrakeTorque)

            # set up one brake configuration for the wheels of the right track
            brakes1API = PhysxSchema.PhysxVehicleBrakesAPI.Apply(vehiclePrim, PhysxSchema.Tokens.brakes1)
            brakes1API.CreateWheelsAttr([WHEEL_FRONT_RIGHT, WHEEL_REAR_RIGHT])
            brakes1API.CreateMaxBrakeTorqueAttr(defaultMaxBrakeTorque)
        elif (driveMode != DRIVE_NONE):
            # set up one brake configuration that applies to all wheels
            brakes0API = PhysxSchema.PhysxVehicleBrakesAPI.Apply(vehiclePrim, PhysxSchema.Tokens.brakes0)
            brakes0API.CreateMaxBrakeTorqueAttr(defaultMaxBrakeTorque)

            # set up a handbrake configuration that applies to the rear wheels only
            brakes1API = PhysxSchema.PhysxVehicleBrakesAPI.Apply(vehiclePrim, PhysxSchema.Tokens.brakes1)
            brakes1API.CreateWheelsAttr([WHEEL_REAR_LEFT, WHEEL_REAR_RIGHT])
            brakes1API.CreateMaxBrakeTorqueAttr(3000 * kgmsScale)

            # set up a steering configuration that applies to the front wheels
            if (useAckermannCorrection):
                steeringAPI = PhysxSchema.PhysxVehicleAckermannSteeringAPI.Apply(vehiclePrim)
                steeringAPI.CreateWheel0Attr(WHEEL_FRONT_RIGHT)
                steeringAPI.CreateWheel1Attr(WHEEL_FRONT_LEFT)
                steeringAPI.CreateMaxSteerAngleAttr(0.554264)
                steeringAPI.CreateWheelBaseAttr(longitudinalWheelOffset * 2.0)
                steeringAPI.CreateTrackWidthAttr(lateralWheelOffset * 2.0)
                if not skipAttrWithDefaultValues:
                    if not setAttrToDefaultValues:
                        steeringAPI.CreateStrengthAttr(1.0)
                    else:
                        steeringAPI.CreateStrengthAttr(1.0)
            else:
                steeringAPI = PhysxSchema.PhysxVehicleSteeringAPI.Apply(vehiclePrim)
                steeringAPI.CreateWheelsAttr([WHEEL_FRONT_LEFT, WHEEL_FRONT_RIGHT])
                steeringAPI.CreateMaxSteerAngleAttr(0.554264)

        if ((driveMode == DRIVE_STANDARD) and (configureAsTank)):
            tankDifferentialAPI = PhysxSchema.PhysxVehicleTankDifferentialAPI.Apply(vehiclePrim)

            tankDifferentialAPI.CreateNumberOfWheelsPerTrackAttr([2, 2])
            tankDifferentialAPI.CreateThrustIndexPerTrackAttr([0, 1])
            tankDifferentialAPI.CreateTrackToWheelIndicesAttr([0, 2])
            tankDifferentialAPI.CreateWheelIndicesInTrackOrderAttr(
                [
                    WHEEL_FRONT_LEFT,
                    WHEEL_REAR_LEFT,
                    WHEEL_FRONT_RIGHT,
                    WHEEL_REAR_RIGHT
                ]
            )

            differentialAPI = PhysxSchema.PhysxVehicleMultiWheelDifferentialAPI(tankDifferentialAPI)  # tank differential also applies multi wheel differential
        elif (drivenWheelIndexList):  # python style guide to check for empty list
            differentialAPI = PhysxSchema.PhysxVehicleMultiWheelDifferentialAPI.Apply(vehiclePrim)

        if (drivenWheelIndexList):  # python style guide to check for empty list
            drivenWheelCount = len(drivenWheelIndexList)
            ratio = 1.0 / drivenWheelCount

            differentialAPI.CreateWheelsAttr(drivenWheelIndexList)
            differentialAPI.CreateTorqueRatiosAttr([ratio] * drivenWheelCount)

            if (driveMode == DRIVE_STANDARD):
                differentialAPI.CreateAverageWheelSpeedRatiosAttr([ratio] * drivenWheelCount)

    chassisHalfExtents = Gf.Vec3f(0)
    chassisHalfExtents[axes.up] = chassisHalfHeight
    chassisHalfExtents[axes.forward] = 2.4 * lengthScale
    chassisHalfExtents[axes.side] = 0.9 * lengthScale
    chassisOffset = Gf.Vec3f(0)
    chassisOffset[axes.up] = 0 * lengthScale
    chassisOffset[axes.forward] = 0 * lengthScale
    chassisOffset[axes.side] = 0 * lengthScale

    # chassis (collision)
    if (addChassisCollisionBox):
        vehicleChassisPath = vehiclePath + "/ChassisCollision"
        vehicleChassis = UsdGeom.Cube.Define(stage, vehicleChassisPath)
        vehicleChassis.CreatePurposeAttr(UsdGeom.Tokens.guide)
        vehicleChassis.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(chassisOffset)
        vehicleChassis.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(chassisHalfExtents)

        vehicleChassisPrim = vehicleChassis.GetPrim()

        collisionAPI = UsdPhysics.CollisionAPI.Apply(vehicleChassisPrim)
        add_collision_to_collision_group(stage, vehicleChassisPath, chassisCollisionGroupPath)

        physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(vehicleChassisPrim)
        physxCollisionAPI.CreateRestOffsetAttr(0.0 * lengthScale)
        physxCollisionAPI.CreateContactOffsetAttr(0.02 * lengthScale)

    # chassis (render)
    if addChassisRenderMesh:
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
        _convert_gf_vec3_list(normals, gYZXAxes, axes)

        chassisHalfExtents[axes.side] = 0.7 * lengthScale  # reduced width to make the wheels easily visible
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


def create4WheeledCarsScenario(
    stage,
    lengthScale,
    vehicleCount,
    createCollisionShapesForWheels=False,
    driveMode=DRIVE_NONE,
    createAutoGearBox=True,
    vehiclePathsOut=None,
    wheelAttachmentPathsOut=None,
    vehicleDelta=[-3, 0, 0],
    basePositionForwardDir=0,
    basePositionSideDir=0,
    addChassisRenderMesh=True,
    addChassisCollisionBox=True,
    placeWheelsAsGrandchildren=False,
    placeWheelCollisionShapesAtRoot=False,
    useMeshAsWheelCollisionShape=False,
    noWheelRootTransformManagement=False,
    omitControllers=False,
    skipAttrWithDefaultValues=False,
    setAttrToDefaultValues=False,  # for some parameters, the factory uses different values than the defaults. Set to True to stick to defaults.
    axes=AxesIndices(1, 2, 0),
    tireFrictionTablePathsOut=None,
    collisionGroupPathsOut=None,
    vehicleEnabledList=None,
    useRaycastsList=None,
    useShareableComponentsList=None,
    timeStepsPerSecond=60,
    vehiclePathsIn=None,
    massScale=1.0,
    useDeprecatedAPIList=None,  # option to specify for each vehicle if deprecated APIs should be used
    configureAsTank=False,
    useAckermannCorrection=False,
    referenceFrameIsCoMList=None,  # option to specify for each vehicle if suspension frame etc. are relative to the center of mass frame or
                                   # relative to the vehicle prim frame
):
    rootPath = str(stage.GetDefaultPrim().GetPath())

    unitScale = UnitScale(
        lengthScale,
        massScale
    )

    deprecatedAPIOnly = False
    if (useDeprecatedAPIList is not None):
        deprecatedAPIOnly = True
        for useDepAPI in useDeprecatedAPIList:
            if (not useDepAPI):
                # if any vehicle does not want to use deprecated APIs, then shared components
                # will be based on the latest API
                deprecatedAPIOnly = False
                break

    createSceneBasics(stage, unitScale, axes, timeStepsPerSecond, deprecatedAPIOnly)

    collisionGroupPaths = []
    createCollisionGroups(stage, collisionGroupPaths)
    if (collisionGroupPathsOut is not None):
        for i in range(len(collisionGroupPaths)):
            collisionGroupPathsOut.append(collisionGroupPaths[i])

    materialPaths = []
    tireFrictionTablePaths = []
    createMaterialsAndTireFrictionTables(stage, materialPaths, tireFrictionTablePaths)
    if (tireFrictionTablePathsOut is not None):
        for i in range(len(tireFrictionTablePaths)):
            tireFrictionTablePathsOut.append(tireFrictionTablePaths[i])

    createGroundPlane(
        stage, unitScale, collisionGroupPaths[COLL_GROUP_GROUND_SURFACE], materialPaths[MATERIAL_TARMAC], axes
    )

    if (useShareableComponentsList is not None):
        createShareableComponents = False
        for useShareableComponents in useShareableComponentsList:
            if useShareableComponents:
                createShareableComponents = True
                break
    else:
        createShareableComponents = True

    driveComponentsPaths = DriveComponentPaths()
    tireFrictionTablePath = tireFrictionTablePaths[TIRE_FRICTION_TABLE_WINTER_TIRE]

    if createShareableComponents:
        wheelComponentsPaths = WheelComponentPaths()
        createWheelComponents(
            stage,
            unitScale,
            tireFrictionTablePath,
            wheelComponentsPaths,
            skipAttrWithDefaultValues,
            setAttrToDefaultValues,
            deprecatedAPIOnly
        )

        if driveMode != DRIVE_NONE:
            createDriveComponents(
                stage,
                unitScale,
                driveComponentsPaths,
                driveMode,
                createAutoGearBox,
                skipAttrWithDefaultValues,
                setAttrToDefaultValues,
                deprecatedAPIOnly
            )

        carWheelPaths = [
            wheelComponentsPaths.wheelPaths[WHEEL_FRONT],
            wheelComponentsPaths.wheelPaths[WHEEL_FRONT],
            wheelComponentsPaths.wheelPaths[WHEEL_REAR],
            wheelComponentsPaths.wheelPaths[WHEEL_REAR],
        ]

        carTirePaths = [
            wheelComponentsPaths.tirePaths[TIRE_FRONT],
            wheelComponentsPaths.tirePaths[TIRE_FRONT],
            wheelComponentsPaths.tirePaths[TIRE_REAR],
            wheelComponentsPaths.tirePaths[TIRE_REAR],
        ]

        carSuspensionPaths = [
            wheelComponentsPaths.suspensionPaths[SUSPENSION_FRONT],
            wheelComponentsPaths.suspensionPaths[SUSPENSION_FRONT],
            wheelComponentsPaths.suspensionPaths[SUSPENSION_REAR],
            wheelComponentsPaths.suspensionPaths[SUSPENSION_REAR],
        ]

    if driveMode == DRIVE_NONE:
        carWheelIsDriven = [0, 0, 0, 0]
    else:
        carWheelIsDriven = [1, 1, 0, 0]

    vehiclePositionBase = [0, 0, 0]
    vehiclePositionBase[axes.up] = 1 * unitScale.lengthScale
    vehiclePositionBase[axes.forward] = basePositionForwardDir
    vehiclePositionBase[axes.side] = basePositionSideDir

    for i in range(vehicleCount):
        carWheelAttachmentPaths = []

        if (vehiclePathsIn is None):
            vehiclePath = rootPath + "/Car_" + str(i)
        else:
            vehiclePath = vehiclePathsIn[i]

        if vehiclePathsOut is not None:
            vehiclePathsOut.append(vehiclePath)

        vehiclePosition = [
            vehiclePositionBase[0] + (vehicleDelta[0] * i),
            vehiclePositionBase[1] + (vehicleDelta[1] * i),
            vehiclePositionBase[2] + (vehicleDelta[2] * i),
        ]
        
        vehicleEnabled = True
        if (vehicleEnabledList is not None):
            vehicleEnabled = vehicleEnabledList[i]

        useRaycasts = True
        if (useRaycastsList is not None):
            useRaycasts = useRaycastsList[i]

        useShareableComponents = True
        if (useShareableComponentsList is not None):
            useShareableComponents = useShareableComponentsList[i]

        if (useShareableComponents):
            carWheelPathsIn = carWheelPaths
            carTirePathsIn = carTirePaths
            carSuspensionPathsIn = carSuspensionPaths
            drivePathIn = driveComponentsPaths.drivePath
        else:
            carWheelPathsIn = None
            carTirePathsIn = None
            carSuspensionPathsIn = None
            drivePathIn = None

        useDeprecatedAPI = False
        if (useDeprecatedAPIList is not None):
            useDeprecatedAPI = useDeprecatedAPIList[i]

        referenceFrameIsCoM = False
        if (referenceFrameIsCoMList is not None):
             referenceFrameIsCoM = referenceFrameIsCoMList[i]

        create4WheeledCar(
            stage,
            unitScale,
            vehiclePath,
            vehiclePosition,
            createCollisionShapesForWheels,
            carWheelIsDriven,
            carWheelPathsIn,
            carTirePathsIn,
            carSuspensionPathsIn,
            tireFrictionTablePath,
            drivePathIn,
            driveMode,
            createAutoGearBox,
            collisionGroupPaths[COLL_GROUP_VEHICLE_CHASSIS],
            collisionGroupPaths[COLL_GROUP_VEHICLE_WHEEL],
            collisionGroupPaths[COLL_GROUP_VEHICLE_GROUND_QUERY],
            carWheelAttachmentPaths,
            addChassisRenderMesh,
            addChassisCollisionBox,
            placeWheelsAsGrandchildren,
            placeWheelCollisionShapesAtRoot,
            useMeshAsWheelCollisionShape,
            noWheelRootTransformManagement,
            omitControllers,
            skipAttrWithDefaultValues,
            setAttrToDefaultValues,
            axes,
            vehicleEnabled,
            useRaycasts,
            useDeprecatedAPI,
            configureAsTank,
            useAckermannCorrection,
            referenceFrameIsCoM,
        )

        if wheelAttachmentPathsOut is not None:
            wheelAttachmentPathsOut.append(carWheelAttachmentPaths)
