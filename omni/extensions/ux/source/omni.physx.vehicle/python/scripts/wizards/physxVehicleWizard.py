# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ui
import omni.usd
import omni.timeline
import math
import carb
import carb.profiler
from carb.eventdispatcher import get_eventdispatcher
import usdrt
from pxr import UsdGeom, UsdShade, Sdf, Gf, UsdPhysics, PhysxSchema
from omni.physx.scripts.physicsUtils import *
from omni.physx.scripts.utils import (
    set_custom_metadata
)
from omni.physx.bindings._physx import (
    VEHICLE_AUTOMATIC_TRANSMISSION_GEAR_VALUE
)
from omni.physxui import PhysicsMenu
from omni.kit.window.popup_dialog import MessageDialog

from ..helpers import UI
from ..helpers.UnitScale import UnitScale

from .. import commands

from . import vehicleWizardBasics
from . import vehicleWizardAxles
from . import vehicleWizardNextSteps


DRIVE_TYPE_STANDARD = 0
DRIVE_TYPE_BASIC = 1
DRIVE_TYPE_NONE = 2

TIRE_TYPE_SUMMER = 0
TIRE_TYPE_ALL_SEASON = 1
TIRE_TYPE_SLICK = 2

QUERY_TYPE_RAYCAST = 0
QUERY_TYPE_SWEEP = 1

DEFAULT_SCENE_PATH = "/PhysicsScene"
SHARED_DATA_ROOT_BASE_PATH = "/WizardSharedVehicleData"
VEHICLE_ROOT_BASE_PATH = "/WizardVehicle"
VEHICLE_GROUND_MATERIAL_PATH = "/VehicleGroundMaterial"
VEHICLE_COLLISION_GROUP_CHASSIS_PATH = "/VehicleChassisCollisionGroup"
VEHICLE_COLLISION_GROUP_WHEEL_PATH = "/VehicleWheelCollisionGroup"
VEHICLE_COLLISION_GROUP_GROUND_QUERY_PATH = "/VehicleGroundQueryGroup"
VEHICLE_COLLISION_GROUP_GROUND_SURFACE_PATH = "/GroundSurfaceCollisionGroup"

# Some parameter values will be computed by scaling values from a reference vehicle
#
# As a common ground to start from, vehicle parameters for differently sized vehicles will get
# scaled such that the wheel angular velocity dynamics will match with a reference vehicle.
#
# For the standard drive vehicle, the wheel angular velocities are described by a system of
# linear equations. For a wheel, the relevant (with respect to scaling) terms of the linear
# system can be roughly summarized as follows:
#
# left hand side       to solve for       right hand side
# ============================================================
# K/I0, d0/I0          w0                 Tb/I0, Tt/I0
# K/Ie, de/Ie          we                 Te/Ie
#
# K: clutch strength
# d0: damping rate of wheel
# de: damping rate of engine
# Tb: brake torque
# Tt: tire torque
# Te: engine torque
# I0: moment of inertia of wheel
# Ie: moment of inertia of engine
# w0: wheel angular velocity
# we: engine angular velocity
#
# I0 is a given as the user can set the wheel mass and size in the wizard and those params
# are the ones most obviously affected by scaling. Thus, to keep the dynamics of the wheel
# and engine angular velocities the same as for the reference vehicle, all parameters that
# interact with I0 have to be scaled to compensate for the change of I0 in the scaled
# vehicle. K, d0, Tb, Tt are divided by I0, thus they get computed from the reference
# parameters by multiplying with s = I0/I0Ref, where I0Ref is the moment of inertia of the
# wheel for the reference vehicle.
#
# e.g.
#
# K = KRef * s
# => K / I0  =  KRef * s / I0  =  KRef * (I0/I0Ref) / I0  =  KRef / I0Ref
#
# Since K also appears in K/Ie and has been defined as above, Ie has to be computed using
# Ie = IeRef * s to keep the term the same as for the reference vehicle.
#
# => K / Ie  =  KRef * s / Ie  =  KRef * s / IeRef * s  =  KRef / IeRef
#
# This in turn requires de and Te to get computed from the reference parameters by multiplying
# with s too
#
# Tb, Tt and Te are parameters computed by the sim but they do evolve from max brake torque,
# longitudinal stiffness and engine peak torque.
#
# A note about longitudinal stiffness (ls) and Tt. Simplified we have:
# Tt = Ft * r = ls * something * r
# (something = longitudinal slip etc. but that is not scale dependent as such)
#
# Ft: tire force
# r: wheel radius
#
# Since the wheel radius is involved, we need to adjust the multiplier:
# ls = lsRef * (s / (r/rRef))
#
# => Tt / I0  =  ls * r / I0  =  lsRef * (s / (r/rRef)) * r / I0
# =  lsRef * s * rRef / I0  =  lsRef * (I0/I0Ref) * rRef / I0  =  lsRef * rRef / I0Ref
#
# The approach can currently also be described like this: every parameter gets scaled according
# to the units that depend on scale, e.g., torque is defined in [kg m^2 / s^2], thus the
# reference value will be scaled by m/mRef * r/rRef * r/rRef (m being wheel mass and r wheel
# radius).
#
# As mentioned at the beginning, this will be the starting point for choosing parameter values.
# Since the wheel angular velocity dynamic will thus mimic the one of the reference vehicle,
# that also means that smaller vehicles will have a lower top speed as the wheel radius is
# smaller. If we wanted to preserve top speed, more changes are needed as the wheels have to
# spin faster.
#
# engine max rotation speed needs to be divided by r/rRef
#
# => top speed (roughly)  =  wMax * r  =  (wMaxRef / (r/rRef)) * r  =  wMaxRef * rRef
#
# The same speed as for the reference vehicle might be reached that way but not within the same
# time. The engine needs to first get to the much higher angular velocity. To get there faster,
# the engine peak torque has to be increased too such that it takes similar time to reach max
# rotation speed.
#
# engine peak torque is divided by r/rRef  (which will affect angular acceleration w' accordingly)
#
# => dt  =  wMax / w'Max  =  wMax / (w'MaxRef / (r/rRef))
# =  (wMaxRef / (r/rRef)) / (w'MaxRef / (r/rRef))  =  wMaxRef / w'MaxRef
#
# Similarly, brake torque etc. have to be adjusted too.
#
# The higher spin does not help, if the wheels can not translate it to higher linear acceleration.
# As a consequence, longitudinal stiffness has to be divided by r/rRef too
#
# => a  =  Ft / m  =  ls / m  =  lsRef * (s / (r/rRef)) / (r/rRef) / m
# =  lsRef * ((I0/I0Ref) / (r/rRef)) / (r/rRef) / m
# =  lsRef * ((m/mRef * r/rRef * r/rRef) / (r/rRef)) / (r/rRef) / m  =  lsRef / mRef
#
# Note that the wizard will not try to keep the same top speed as for the reference vehicle. Smaller
# vehicles will have lower top speed but the wizard will use ((r/rRef) * constant + (1 - constant))
# (constant < 1) such that there is some tuning control for how fast top speed will drop for smaller
# vehicles.
#
REFERENCE_CHASSIS_LENGTH = 4                                                               # [m]
REFERENCE_CHASSIS_WIDTH = 2                                                                # [m]
REFERENCE_CHASSIS_HEIGHT = 1                                                               # [m]
REFERENCE_CHASSIS_MASS = 1800                                                              # [kg]
REFERENCE_ENGINE_MOI = 1                                                                   # [kg m^2]
REFERENCE_ENGINE_PEAK_TORQUE_STANDARD = 500                                                # [kg m^2 / s^2]
REFERENCE_ENGINE_PEAK_TORQUE_BASIC = 1000                                                  # [kg m^2 / s^2]
REFERENCE_ENGINE_MAX_ROTATION_SPEED = 600                                                  # [rad / s]
REFERENCE_ENGINE_MAX_RPM = (REFERENCE_ENGINE_MAX_ROTATION_SPEED * 60.0) / (2.0 * math.pi)  # [revolutions / min]
REFERENCE_ENGINE_DAMPING_RATE_FULL_THROTTLE = 0.15                                         # [kg m^2 / s]
REFERENCE_ENGINE_DAMPING_RATE_ZERO_THROTTLE_CLUTCH_ENGAGED = 2.0                           # [kg m^2 / s]
REFERENCE_ENGINE_DAMPING_RATE_ZERO_THROTTLE_CLUTCH_DISENGAGED = 0.35                       # [kg m^2 / s]
REFERENCE_CLUTCH_STRENGTH = 10.0                                                           # [kg m^2 / s]
REFERENCE_WHEEL_RADIUS = 0.35                                                              # [m]
REFERENCE_WHEEL_WIDTH = 0.15                                                               # [m]
REFERENCE_WHEEL_MASS = 20                                                                  # [kg]
REFERENCE_WHEEL_DAMPING_RATE = 0.25                                                        # [kg m^2 / s]
REFERENCE_WHEEL_BRAKE_TORQUE = 3600                                                        # [kg m^2 / s^2]
REFERENCE_TIRE_LONGITUDINAL_STIFFNESS = 500 * 9.81                                         # [kg m / s^2] = [N]
REFERENCE_TIRE_CAMBER_STIFFNESS = 0                                                        # [kg m / s^2 rad] = [N / rad]
REFERENCE_SLEEP_THRESHOLD = 0.005                                                          # [m^2 / s^2]
REFERENCE_STABILIZATION_THRESHOLD = 0.001                                                  # [m^2 / s^2]
REFERENCE_CONTACT_OFFSET = 0.02                                                            # [m]

TOP_SPEED_PRESERVATION_COEFFICIENT = 0.8
# value between 0 and 1
# 0.0: do not try to preserve top speed of reference vehicle (mimic angular velocity dynamics)
# 1.0: try to fully preserve top speed of reference vehicle (change angular velocity dynamics)

DEFAULT_TIRE_RADIUS_SCALE = REFERENCE_WHEEL_RADIUS / REFERENCE_CHASSIS_LENGTH  # default radius is chosen as multiple of vehicle length
DEFAULT_TIRE_WIDTH_SCALE = REFERENCE_WHEEL_WIDTH / REFERENCE_WHEEL_RADIUS      # default width is chosen as multiple of wheel radius

DEFAULT_WHEEL_DENSITY = REFERENCE_WHEEL_MASS / (math.pi * REFERENCE_WHEEL_RADIUS * REFERENCE_WHEEL_RADIUS * REFERENCE_WHEEL_WIDTH)
DEFAULT_VEHICLE_DENSITY = REFERENCE_CHASSIS_MASS / (REFERENCE_CHASSIS_LENGTH * REFERENCE_CHASSIS_WIDTH * REFERENCE_CHASSIS_HEIGHT)

MIN_VEHICLE_LENGTH = 0.05
MIN_VEHICLE_WIDTH = 0.02
MIN_VEHICLE_HEIGHT = 0.01

DEFAULT_VEHICLE_DENSITY_LENGTH = 0.1  # the default density scale will be used for vehicles longer or equal 0.1 meters
DEFAULT_VEHICLE_DENSITY_SCALE = 1.0

SMALL_VEHICLE_DENSITY_LENGTH = 0.01  # the small vehicle density scale will be used for vehicles longer or equal 0.01 meters
SMALL_VEHICLE_DENSITY_SCALE = 20.0

HORSEPOWER_CONVERSION_CONSTANT = 7120.756

WINDOW_WIDTH_START = 630
WINDOW_HEIGHT_START = 590
CONTENT_HEIGHT_WITHOUT_FOOTER = 500  # the "footer" is the part with Next/Back/... buttons

FLT_MAX = 1.0e38

def Lerp(x0, y0, x1, y1, x):
    t = (x - x0) / (x1 - x0)
    t = min(max(t, 0.0), 1.0)

    return y0 + t * (y1 - y0)

def compute_mass(densitySI, volume, unitScale: UnitScale):
    # the reference density is in kg/m^3
    # => transform to expected length and mass unit
    density = unitScale.massScale * densitySI / (unitScale.lengthScale * unitScale.lengthScale * unitScale.lengthScale)

    mass = density * volume  # note: volume is expected to be in the unit of unitScale.lengthScale

    return mass

def compute_chassis_mass(chassisLength, chassisWidth, chassisHeight, unitScale: UnitScale):
    densityScale = 1.0

    # get length in meters
    length = chassisLength / unitScale.lengthScale

    densityScale = Lerp(SMALL_VEHICLE_DENSITY_LENGTH, SMALL_VEHICLE_DENSITY_SCALE,
        DEFAULT_VEHICLE_DENSITY_LENGTH, DEFAULT_VEHICLE_DENSITY_SCALE, length)

    densitySI = densityScale * DEFAULT_VEHICLE_DENSITY
    volume = chassisLength * chassisWidth * chassisHeight

    return compute_mass(densitySI, volume, unitScale)

def compute_engine_max_rpm(scaleFactor):

    return REFERENCE_ENGINE_MAX_RPM * scaleFactor

# horsepower gets unit scaling when it is converted into torque
def compute_horsepower(engineMaxRPM, referencePeakTorque, scaleFactor):
    horsepower = (referencePeakTorque * scaleFactor) * engineMaxRPM / HORSEPOWER_CONVERSION_CONSTANT

    return horsepower

def compute_tire_radius(chassisLength):
    tireRadius = DEFAULT_TIRE_RADIUS_SCALE * chassisLength

    return tireRadius

def compute_tire_width(tireRadius):
    tireWidth = DEFAULT_TIRE_WIDTH_SCALE * tireRadius

    return tireWidth

def compute_tire_mass(tireRadius, tireWidth, unitScale: UnitScale):
    defaulTireRadius = DEFAULT_VEHICLE_DENSITY_LENGTH * DEFAULT_TIRE_RADIUS_SCALE
    smallTireRadius = SMALL_VEHICLE_DENSITY_LENGTH * DEFAULT_TIRE_RADIUS_SCALE

    densityScale = 1.0

    # get radius in meters
    radius = tireRadius / unitScale.lengthScale

    densityScale = Lerp(smallTireRadius, SMALL_VEHICLE_DENSITY_SCALE,
        defaulTireRadius, DEFAULT_VEHICLE_DENSITY_SCALE, radius)

    densitySI = densityScale * DEFAULT_WHEEL_DENSITY
    volume = math.pi * tireRadius * tireRadius * tireWidth

    return compute_mass(densitySI, volume, unitScale)

def compute_param_scaling_factors(tireRadius: float, tireMass: float, unitScale: UnitScale):
    # see long description where the reference parameters are defined
    # cancel out the unit scaling to get the base ratios - scaling will be applied later

    massRatio = tireMass / (REFERENCE_WHEEL_MASS * unitScale.massScale)
    radiusRatio = tireRadius / (REFERENCE_WHEEL_RADIUS * unitScale.lengthScale)
    stiffnessScaling = massRatio * radiusRatio
    moiScaling = stiffnessScaling * radiusRatio
    topSpeedScaling = 1.0 / (radiusRatio * TOP_SPEED_PRESERVATION_COEFFICIENT + (1 - TOP_SPEED_PRESERVATION_COEFFICIENT))

    return (moiScaling, stiffnessScaling, topSpeedScaling, radiusRatio, massRatio)

def compute_param_scaling_factors_average(vehicleData):
    # see long description where the reference parameters are defined

    averageRadius = 0
    averageMass = 0

    wheelCount = vehicleData.numberOfAxles * 2
    for i in range(wheelCount):
        averageRadius = averageRadius + vehicleData.tireList[i].radius
        averageMass = averageMass + vehicleData.tireList[i].mass

    averageRadius = averageRadius / wheelCount
    averageMass = averageMass / wheelCount

    return compute_param_scaling_factors(averageRadius, averageMass, vehicleData.unitScale)


class TireData:
    def __init__(self):
        self.radius = 0.0
        self.width = 0.0
        self.mass = 0.0
        self.position = None  # position (as Gf.Vec3f) if a user defined position should be used
        self.path = ""  # prim path (as string) if a user defined prim should be used


class CollisionGroupIncludeData:
    def __init__(self, primPath, collisionGroupPath):
        self.path = primPath
        self.groupPath = collisionGroupPath


# logic to track modifications done by the wizard. Not really needed for
# the wizard logic itself but needed for undoing what the wizard did.
#
# note: there will be no apply API callbacks for prims that were created by the
#       wizard (prims recorded in createdPrimPaths)
#
class OperationTracker:
    def __init__(self):
        self.applyAPISingleCallback = None
        # func(prim: Usd.Prim, apiSchema) -> None

        self.applyAPIMultipleCallback = None
        # func(prim: Usd.Prim, apiSchema, api_prefix, multiple_api_token) -> None
        # api_prefix: example: PhysxSchema.Tokens.physxVehicleBrakes
        # multiple_api_token: example: PhysxSchema.Tokens.brakes0

        self.reset()

    def reset(self):
        self.createdPrimPaths = []  # list of the paths of the prims that have been created.
                                    # Note that this may contain parent and child prims, so
                                    # if this list is used to delete the created prims, there
                                    # needs to be a check if the prim exists or the deletion
                                    # needs to happen from back to front

        self.collisionGroupIncludeList = []  # list of CollisionGroupIncludeData objects
                                             # describing the prim paths that were added to
                                             # the include list of a collision group
        self.scenePath = None


class VehicleData:
    MAX_NUMBER_OF_AXLES = 10

    AXIS_X = 0
    AXIS_Y = 1
    AXIS_Z = 2

    AXIS_TO_VEC_MAP = [
        Gf.Vec3f(1, 0, 0),
        Gf.Vec3f(0, 1, 0),
        Gf.Vec3f(0, 0, 1)
    ]

    def __init__(self, unitScale: UnitScale, verticalAxis: int, longitudinalAxis: int):
        self.reset_all(unitScale, verticalAxis, longitudinalAxis)

    def reset_all(self, unitScale: UnitScale, verticalAxis: int, longitudinalAxis: int):
        self.unitScale = unitScale
        self.reset_basics(verticalAxis, longitudinalAxis)
        self.reset_axles()

        self.rootVehiclePath = ""  # note: only used if the user does not define the vehicle prim

    def reset_basics(self, verticalAxis: int, longitudinalAxis: int):
        # Basics
        self.vehiclePath = ""  # if set, then the prim at this path will be used as the vehicle prim
        self.position = None
        self.chassisLength = 4.0 * self.unitScale.lengthScale
        self.chassisWidth = 2.0 * self.unitScale.lengthScale
        self.chassisHeight = 1.0 * self.unitScale.lengthScale
        self.chassisMass = compute_chassis_mass(self.chassisLength, self.chassisWidth, self.chassisHeight, self.unitScale)
        self.numberOfAxles = 2
        self.set_drive_type(DRIVE_TYPE_STANDARD, skipHorsepowerUpdate = True)

        self.tireTypeIndex = TIRE_TYPE_SUMMER
        self.tireRadius = compute_tire_radius(self.chassisLength)
        self.tireWidth = compute_tire_width(self.tireRadius)
        self.tireMass = compute_tire_mass(self.tireRadius, self.tireWidth, self.unitScale)
        self.wheelCollisionGeometry = False
        self.queryTypeIndex = QUERY_TYPE_RAYCAST

        (moiScaling, stiffnessScaling, topSpeedScaling, radiusRatio, massRatio) = compute_param_scaling_factors(
            self.tireRadius, self.tireMass, self.unitScale)
        self.engineMaxRPM = compute_engine_max_rpm(topSpeedScaling)
        self.horsepower = compute_horsepower(self.engineMaxRPM, self.referencePeakTorque, moiScaling * topSpeedScaling)
        self.numberOfGears = 5
        # note: self.tankMode is set by set_drive_type()
        self.createShareableComponents = False
        self.useAckermannCorrection = False

        self.set_axes(verticalAxis, longitudinalAxis)
        self._reset_weightDistribution()

    def _get_default_tire_data(self):
        tire = TireData()
        tire.radius = self.tireRadius
        tire.width = self.tireWidth
        tire.mass = self.tireMass
        return tire

    def reset_tire(self, index):
        tire = self._get_default_tire_data()
        self.tireList[index] = tire

    def reset_axles(self):
        # Axles

        self.tireList = []

        for tireIndex in range(2 * self.MAX_NUMBER_OF_AXLES):
            tire = self._get_default_tire_data()
            self.tireList.append(tire)

        self.damping = [0.3] * self.MAX_NUMBER_OF_AXLES

        self._reset_weightDistribution()

    def _reset_weightDistribution(self):
        self.weightDistribution = [100.0 / self.numberOfAxles] * self.MAX_NUMBER_OF_AXLES

    def _set_max_steer_and_driven(self):
        self.maxSteerAngle = [0.0] * self.MAX_NUMBER_OF_AXLES

        if (self.tankMode):
            self.driven = [True] * self.MAX_NUMBER_OF_AXLES
        else:
            self.driven = [False] * self.MAX_NUMBER_OF_AXLES

            if self.driveTypeIndex != DRIVE_TYPE_NONE:
                self.maxSteerAngle[0] = 30.0
                self.driven[0] = True

    def set_number_of_axles(self, numberOfAxles: int, updateDependentParams: bool = True):

        self.numberOfAxles = numberOfAxles

        if updateDependentParams:
            self.reset_axles()

            if (numberOfAxles < 2):
                self.useAckermannCorrection = False

    def set_drive_type(self, driveTypeIndex: int, updateDependentParams: bool = True,
            skipHorsepowerUpdate: bool = False):

        self.driveTypeIndex = driveTypeIndex

        if updateDependentParams:
            if (driveTypeIndex == DRIVE_TYPE_BASIC):
                newReferencePeakTorque = REFERENCE_ENGINE_PEAK_TORQUE_BASIC
            else:
                newReferencePeakTorque = REFERENCE_ENGINE_PEAK_TORQUE_STANDARD

            if (not skipHorsepowerUpdate):
                oldReferencePeakTorque = self.referencePeakTorque
                ratio = newReferencePeakTorque / oldReferencePeakTorque
                self.horsepower = self.horsepower * ratio

            self.tankMode = False

            self.referencePeakTorque = newReferencePeakTorque

            if (driveTypeIndex == DRIVE_TYPE_NONE):
                self.useAckermannCorrection = False

            self._set_max_steer_and_driven()

    def set_tank_mode(self, tankMode: bool, updateDependentParams: bool = True):

        self.tankMode = tankMode

        if updateDependentParams:
            self._set_max_steer_and_driven()

            if (tankMode):
                self.useAckermannCorrection = False

    def set_use_ackermann_correction(self, ackermannCorrection: bool, updateDependentParams: bool = True):

        self.useAckermannCorrection = ackermannCorrection

        if updateDependentParams:
            for i in range(1, len(self.maxSteerAngle)):
                self.maxSteerAngle[i] = 0.0

    def set_axes(self, verticalAxis: int, longitudinalAxis: int):
        self.verticalAxis = verticalAxis
        self.longitudinalAxis = longitudinalAxis

    def _get_axis_vector(self, axis: int) -> Gf.Vec3f:
        return self.AXIS_TO_VEC_MAP[axis]

    def get_vertical_axis_vector(self) -> Gf.Vec3f:
        return self._get_axis_vector(self.verticalAxis)

    def get_longitudinal_axis_vector(self) -> Gf.Vec3f:
        return self._get_axis_vector(self.longitudinalAxis)

    def get_lateral_axis_vector(self) -> Gf.Vec3f:
        vert = self.get_vertical_axis_vector()
        long = self.get_longitudinal_axis_vector()
        return vert.GetCross(long)

    def has_wheel_attachment_prim_defined(self) -> bool:
        for axle in range(self.numberOfAxles):
            for side in range(2):
                wheelIndex = 2 * axle + side
                if (self.tireList[wheelIndex].path):
                    return True

        return False


class VehicleDataManager:
    CHASSIS_LENGTH_DIRTY          = 1 << 0
    CHASSIS_WIDTH_OR_HEIGHT_DIRTY = 1 << 1
    CHASSIS_DIMENSIONS_DIRTY      = CHASSIS_LENGTH_DIRTY | CHASSIS_WIDTH_OR_HEIGHT_DIRTY
    TIRE_DIMENSIONS_DIRTY         = 1 << 2
    TIRE_MASS_DIRTY               = 1 << 3
    ALL_DIRTY                     = 0xffffffff

    def __init__(self, unitScale: UnitScale, verticalAxis: int, longitudinalAxis: int):
        self.vehicleData = None
        self.reset_all(unitScale, verticalAxis, longitudinalAxis)

    def reset_all(self, unitScale: UnitScale, verticalAxis: int, longitudinalAxis: int):
        self.autoChassisMass = True
        self.autoHorsepower = True
        self.autoEngineMaxRPM = True
        self.autoTireDimensions = True
        self.autoTireMass = True

        self._dirtyFlags = 0

        if (self.vehicleData is None):
            self.vehicleData = VehicleData(unitScale, verticalAxis, longitudinalAxis)
        else:
            self.vehicleData.reset_all(unitScale, verticalAxis, longitudinalAxis)

    def set_longitudinal_axis(self, longitudinalAxis: int):
        self.vehicleData.longitudinalAxis = longitudinalAxis

    def get_vertical_axis_vector(self) -> Gf.Vec3f:
        return self.vehicleData.get_vertical_axis_vector()

    def get_longitudinal_axis_vector(self) -> Gf.Vec3f:
        return self.vehicleData.get_longitudinal_axis_vector()

    def get_lateral_axis_vector(self) -> Gf.Vec3f:
        return self.vehicleData.get_lateral_axis_vector()

    def set_dirty(self, dirtyFlag: int):
        self._dirtyFlags |= dirtyFlag

    def update(self):
        if (self._dirtyFlags):
            if (self.autoChassisMass and (self._dirtyFlags & VehicleDataManager.CHASSIS_DIMENSIONS_DIRTY)):
                self.vehicleData.chassisMass = compute_chassis_mass(self.vehicleData.chassisLength,
                    self.vehicleData.chassisWidth, self.vehicleData.chassisHeight, self.vehicleData.unitScale)

            if (self.autoTireDimensions and (self._dirtyFlags & VehicleDataManager.CHASSIS_LENGTH_DIRTY)):
                self.vehicleData.tireRadius = compute_tire_radius(self.vehicleData.chassisLength)
                self.vehicleData.tireWidth = compute_tire_width(self.vehicleData.tireRadius)
                self._dirtyFlags |= VehicleDataManager.TIRE_DIMENSIONS_DIRTY

            if (self.autoTireMass and (self._dirtyFlags & VehicleDataManager.TIRE_DIMENSIONS_DIRTY)):
                self.vehicleData.tireMass = compute_tire_mass(
                    self.vehicleData.tireRadius, self.vehicleData.tireWidth, self.vehicleData.unitScale)
                self._dirtyFlags |= VehicleDataManager.TIRE_MASS_DIRTY

            if (self._dirtyFlags & (VehicleDataManager.TIRE_DIMENSIONS_DIRTY | VehicleDataManager.TIRE_MASS_DIRTY)):
                for i in range(2 * self.vehicleData.MAX_NUMBER_OF_AXLES):
                    if (self.vehicleData.tireList[i].position is None):
                        self.vehicleData.tireList[i].radius = self.vehicleData.tireRadius
                        self.vehicleData.tireList[i].width = self.vehicleData.tireWidth
                        self.vehicleData.tireList[i].mass = self.vehicleData.tireMass

                if (self.autoEngineMaxRPM or self.autoHorsepower):
                    (moiScaling, stiffnessScaling, topSpeedScaling, radiusRatio, massRatio) = compute_param_scaling_factors_average(self.vehicleData)

                    if (self.autoEngineMaxRPM):
                        self.vehicleData.engineMaxRPM = compute_engine_max_rpm(topSpeedScaling)

                    if (self.autoHorsepower):
                        self.vehicleData.horsepower = compute_horsepower(self.vehicleData.engineMaxRPM, self.vehicleData.referencePeakTorque,
                            moiScaling * topSpeedScaling)

            self._dirtyFlags = 0

    def set_chassis_length(self, chassisLength: float, updateDependentParams: bool = True):
        self.vehicleData.chassisLength = chassisLength
        self.set_dirty(VehicleDataManager.CHASSIS_LENGTH_DIRTY)

        if updateDependentParams:
            self.update()

    def set_chassis_width(self, chassisWidth: float, updateDependentParams: bool = True):
        self.vehicleData.chassisWidth = chassisWidth
        self.set_dirty(VehicleDataManager.CHASSIS_WIDTH_OR_HEIGHT_DIRTY)

        if updateDependentParams:
            self.update()

    def set_chassis_height(self, chassisHeight: float, updateDependentParams: bool = True):
        self.vehicleData.chassisHeight = chassisHeight
        self.set_dirty(VehicleDataManager.CHASSIS_WIDTH_OR_HEIGHT_DIRTY)

        if updateDependentParams:
            self.update()

    def set_chassis_mass(self, chassisMass: float):
        self.vehicleData.chassisMass = chassisMass
        self.autoChassisMass = False

    def set_horsepower(self, horsepower: float):
        self.vehicleData.horsepower = horsepower
        self.autoHorsepower = False

    def set_engine_max_rpm(self, engineMaxRPM: float):
        self.vehicleData.engineMaxRPM = engineMaxRPM
        self.autoEngineMaxRPM = False

    def set_drive_type(self, driveTypeIndex: int, updateDependentParams: bool = True):
        self.vehicleData.set_drive_type(driveTypeIndex, updateDependentParams)

    def set_tire_radius(self, tireRadius: float, updateDependentParams: bool = True):
        self.vehicleData.tireRadius = tireRadius
        self.set_dirty(VehicleDataManager.TIRE_DIMENSIONS_DIRTY)
        self.autoTireDimensions = False

        if updateDependentParams:
            self.update()

    def set_tire_width(self, tireWidth: float, updateDependentParams: bool = True):
        self.vehicleData.tireWidth = tireWidth
        self.set_dirty(VehicleDataManager.TIRE_DIMENSIONS_DIRTY)
        self.autoTireDimensions = False

        if updateDependentParams:
            self.update()

    def set_tire_mass(self, tireMass: float, updateDependentParams: bool = True):
        self.vehicleData.tireMass = tireMass
        self.set_dirty(VehicleDataManager.TIRE_MASS_DIRTY)
        self.autoTireMass = False

        if updateDependentParams:
            self.update()


class PhysXVehicleWizard:
    def __init__(self):

        # Create the one UI window used by all of the pages.
        self._window = omni.ui.Window(
            "PhysX Vehicle Wizard",
            dockPreference = omni.ui.DockPreference.DISABLED,
            width = WINDOW_WIDTH_START,
            height = WINDOW_HEIGHT_START,
            padding_x = UI.DEFAULT_WINDOW_PADDING_X,
            padding_y = UI.DEFAULT_WINDOW_PADDING_Y
        )

        # Add the wizard to the menu
        self._physicsMenuItem = {"name": "Vehicle", "onclick_fn": lambda *_: self._on_menu_click()}
        PhysicsMenu.add_context_menu("Create", self._physicsMenuItem)

        self._usdContext = omni.usd.get_context()
        self._stage = self._usdContext.get_stage()

        self._currentPage = 1
        self._activePage = None

        self._sharedDataRootPath = None  # None if the wizard did not run on the stage yet

        self.vehicleDataManager = None
        self.reset_data()

        # Create all of the pages.

        self._basicsPage = vehicleWizardBasics.VehicleWizardBasics(self)
        self._axlesPage = vehicleWizardAxles.VehicleWizardAxles(self)
        self._nextStepsPage = vehicleWizardNextSteps.VehicleWizardNextSteps(self)

        self._switch_active_page()

        self._window.set_visibility_changed_fn(self._on_visibility_changed)

        self._stage_event_subs = [
            get_eventdispatcher().observe_event(
                observer_name="omni.physx.vehicle:PhysXVehicleWizard",
                event_name=self._usdContext.stage_event_name(event),
                on_event=func
            )
            for event, func in (
                (omni.usd.StageEventType.OPENED, lambda _: self._on_stage_opened_event()),
                (omni.usd.StageEventType.CLOSING, lambda _: self._on_stage_closed_event()),
            )
        ]

        self._resync_path_subscription = None

    def on_shutdown(self):
        PhysicsMenu.remove_context_menu("Create", self._physicsMenuItem)

        self._resync_path_subscription = None

        self._stage_event_subs = None

        self._clear_active_page()

        self.vehicleDataManager = None
        self._basicsPage = None
        self._axlesPage = None
        self._nextStepsPage = None

        if self._window is not None:
            self._window.frame.clear()
            self._window = None

        self._usdContext = None

    def _get_axis_from_vector(self, vector: Gf.Vec3f) -> int:
        absX = math.fabs(vector[0])
        absY = math.fabs(vector[1])
        if (absY >= absX):
            if (absY >= math.fabs(vector[2])):
                return VehicleData.AXIS_Y
            else:
                return VehicleData.AXIS_Z
        else:
            if (math.fabs(vector[2]) >= absX):
                return VehicleData.AXIS_Z
            else:
                return VehicleData.AXIS_X

    def _get_axis_from_token(self, token: str) -> int:
        if ((token == PhysxSchema.Tokens.posX) or (token == PhysxSchema.Tokens.negX)):
            return VehicleData.AXIS_X
        elif ((token == PhysxSchema.Tokens.posY) or (token == PhysxSchema.Tokens.negY)):
            return VehicleData.AXIS_Y
        else:
            return VehicleData.AXIS_Z

    def _get_length_scale_and_coordinates(self):
        lengthScale = 1.0
        massScale = 1.0
        verticalAxis = VehicleData.AXIS_Y
        longitudinalAxis = VehicleData.AXIS_Z
        vehicleAxesAreImmutable = False

        if (self._stage):
            metersPerUnit = UsdGeom.GetStageMetersPerUnit(self._stage)
            lengthScale = 1.0 / metersPerUnit
            kilogramsPerUnit = UsdPhysics.GetStageKilogramsPerUnit(self._stage)
            massScale = 1.0 / kilogramsPerUnit
            usdrtStage = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())

            # This code uses USDRT to quickly identify in O(1) prims with PhysxVehicleContextAPI applied.
            # It is much faster than normally traversing the USD stage. On the other hand it relies on
            # having a Fabric Scene Delegate available that populated fabric already.
            vehicleContextPrimPaths = usdrtStage.GetPrimsWithAppliedAPIName("PhysxVehicleContextAPI")
            for vehicleContextPrimPath in vehicleContextPrimPaths:
                # Note: why not usdrtStage.GetPrimAtPath here?
                # USDRT population does *not* fill in attributes, just types. Do not use USDRT (yet?) for prim/attribute access.
                prim = self._stage.GetPrimAtPath(str(vehicleContextPrimPath))
                vehicleContext = PhysxSchema.PhysxVehicleContextAPI(prim)

                axisToken = vehicleContext.GetVerticalAxisAttr().Get()
                if axisToken != PhysxSchema.Tokens.undefined:
                    verticalAxis = self._get_axis_from_token(axisToken)
                else:  # deprecated code path
                    attr = vehicleContext.GetUpAxisAttr()
                    if (attr.HasValue()):
                        verticalAxis = self._get_axis_from_vector(attr.Get())

                axisToken = vehicleContext.GetLongitudinalAxisAttr().Get()
                if axisToken != PhysxSchema.Tokens.undefined:
                    longitudinalAxis = self._get_axis_from_token(axisToken)
                else:  # deprecated code path
                    attr = vehicleContext.GetForwardAxisAttr()
                    if (attr.HasValue()):
                        longitudinalAxis = self._get_axis_from_vector(attr.Get())

                vehicleAxesAreImmutable = True

                self._register_usd_listeners(str(vehicleContextPrimPath))

                break

            if (not vehicleAxesAreImmutable):
                upAxisToken = UsdGeom.GetStageUpAxis(self._stage)

                # Kit only supports Y up and Z up. Y up scenario is already
                # set up at the beginning of this function.

                if upAxisToken is UsdGeom.Tokens.z:
                    verticalAxis = VehicleData.AXIS_Z
                    longitudinalAxis = VehicleData.AXIS_X

        return (lengthScale, massScale, verticalAxis, longitudinalAxis, vehicleAxesAreImmutable)

    def reset_data(self):
        (lengthScale, massScale, verticalAxis, longitudinalAxis, vehicleAxesAreImmutable) = self._get_length_scale_and_coordinates()
        self.vehicleAxesAreImmutable = vehicleAxesAreImmutable

        unitScale = UnitScale(lengthScale, massScale)

        if (self.vehicleDataManager is not None):
            self.vehicleDataManager.reset_all(unitScale, verticalAxis, longitudinalAxis)
        else:
            self.vehicleDataManager = VehicleDataManager(unitScale, verticalAxis, longitudinalAxis)

    def show(self):
        self._window.visible = True

    def hide(self):
        self._window.visible = False

    def _on_visibility_changed(self, visible):
        if not visible:
            self._currentPage = 1
            self._switch_active_page()

    def _on_menu_click(self):
        self.show()

    @carb.profiler.profile
    def _on_stage_opened_event(self):
        self._usdContext = omni.usd.get_context()
        self._stage = self._usdContext.get_stage()

        self.reset_data()
        self._currentPage = 1
        self._switch_active_page()

        self._sharedDataRootPath = None

    def _on_stage_closed_event(self):
        self._resync_path_subscription = None

    def _clear_active_page(self):
        if self._activePage is not None:
            self._activePage.tear_down()
            self._activePage = None

    def _switch_active_page(self):
        if self._stage is None:
            return

        self._clear_active_page()

        if self._currentPage == 1:
            self._activePage = self._basicsPage
        elif self._currentPage == 2:
            self._activePage = self._axlesPage
        elif self._currentPage == 3:
            self._activePage = self._nextStepsPage

        if self._activePage is not None:
            self._activePage.set_up()

    def switch_to_next_page(self):
        self._currentPage = self._currentPage + 1
        self._switch_active_page()

    def switch_to_previous_page(self):
        self._currentPage = self._currentPage - 1
        self._switch_active_page()

    def find_free_path(self, basePath):
        count = 0
        path = basePath + str(count + 1)

        while self._stage.GetPrimAtPath(path):
            count = count + 1
            path = basePath + str(count)

        return path

    def _on_resync_path(self, path):
        if (path.IsPrimPath()):
            prim = self._stage.GetPrimAtPath(path)
            if ((not prim.IsValid()) or (not prim.IsActive()) or (not prim.HasAPI(PhysxSchema.PhysxVehicleContextAPI))):
                # prim or vehicle context API has been removed
                self.vehicleAxesAreImmutable = False
                self._switch_active_page()

    def _register_usd_listeners(self, scenePath):
        self._resync_path_subscription = omni.usd.get_watcher().subscribe_to_resync_path(
            scenePath, self._on_resync_path)

    def create_vehicle_command(self):
        if (self._stage):
            # Place the vehicle under the default prim.
            defaultPath = str(self._stage.GetDefaultPrim().GetPath())
            rootVehiclePath = defaultPath + VEHICLE_ROOT_BASE_PATH

            if (self.vehicleDataManager.vehicleData.vehiclePath):
                self.vehicleDataManager.vehicleData.rootVehiclePath = ""
            else:
                self.vehicleDataManager.vehicleData.rootVehiclePath = self.find_free_path(rootVehiclePath)

            self.vehicleDataManager.vehicleData.rootSharedPath = defaultPath + SHARED_DATA_ROOT_BASE_PATH

            (success, (messageList, operationTracker)) = commands.PhysXVehicleWizardCreateCommand.execute(self.vehicleDataManager.vehicleData)

            for message in messageList:
                def on_okay_clicked(dialog: MessageDialog):
                    dialog.hide()

                errorDialog = MessageDialog(
                    width=400,
                    message=message,
                    ok_handler=lambda dialog: on_okay_clicked(dialog),
                    ok_label="Okay",
                    title="Vehicle Creation Wizard",
                    disable_cancel_button=True
                )
                errorDialog.show()

            if ((not self.vehicleAxesAreImmutable) and operationTracker.scenePath):
                scenePrim = self._stage.GetPrimAtPath(operationTracker.scenePath)
                if (scenePrim and scenePrim.HasAPI(PhysxSchema.PhysxVehicleContextAPI)):
                    self._register_usd_listeners(operationTracker.scenePath)

            self.vehicleAxesAreImmutable = True
            # either a vehicle context existed or it will have been created by the command, thus the
            # vertical- and longitudinal-axis are now fixed

            self.hide()

def _setUpTire(
    prim,
    longStiffness,
    camberStiffness,
    maxLateralStiffness,
    tireFrictionTablePath,
    operationTracker
):
    tireAPI = _apply_api_single(prim, PhysxSchema.PhysxVehicleTireAPI, operationTracker)
    tireAPI.CreateLateralStiffnessGraphAttr(Gf.Vec2f(2, maxLateralStiffness))
    tireAPI.CreateLongitudinalStiffnessAttr(longStiffness)
    tireAPI.CreateCamberStiffnessAttr(camberStiffness)
    tireAPI.CreateFrictionVsSlipGraphAttr([Gf.Vec2f(0.0, 1.0), Gf.Vec2f(0.1, 1.0), Gf.Vec2f(1.0, 1.0)])
    tireFrictionTableRel = tireAPI.CreateFrictionTableRel()
    tireFrictionTableRel.AddTarget(tireFrictionTablePath)

def _setUpSuspension(
    prim,
    springStiffness,
    dampingRate,
    travelDistance,
    operationTracker
):
    suspensionAPI = _apply_api_single(prim, PhysxSchema.PhysxVehicleSuspensionAPI, operationTracker)
    suspensionAPI.CreateSpringStrengthAttr(springStiffness)
    suspensionAPI.CreateSpringDamperRateAttr(dampingRate)
    suspensionAPI.CreateTravelDistanceAttr(travelDistance)

def _setUpWheel(
    prim,
    wheelRadius,
    wheelWidth,
    wheelMass,
    wheelMoi,
    dampingRate,
    operationTracker
):
    wheelAPI = _apply_api_single(prim, PhysxSchema.PhysxVehicleWheelAPI, operationTracker)
    wheelAPI.CreateRadiusAttr(wheelRadius)
    wheelAPI.CreateWidthAttr(wheelWidth)
    wheelAPI.CreateMassAttr(wheelMass)
    wheelAPI.CreateMoiAttr(wheelMoi)
    wheelAPI.CreateDampingRateAttr(dampingRate)

def _get_token_from_axis(axis: int) -> str:
    if (axis == VehicleData.AXIS_X):
        return PhysxSchema.Tokens.posX
    elif (axis == VehicleData.AXIS_Y):
        return PhysxSchema.Tokens.posY
    else:
        return PhysxSchema.Tokens.posZ

def _get_or_create_prim(stage, path, targetClass, operationTracker):
    prim = stage.GetPrimAtPath(path)
    if (prim):
        return (targetClass(prim), False)
    else:
        typedPrim = targetClass.Define(stage, path)

        if (operationTracker):
            operationTracker.createdPrimPaths.append(path)

        return (typedPrim, True)

def _apply_api_single(prim, apiSchema, operationTracker):
    if (operationTracker and operationTracker.applyAPISingleCallback):
        operationTracker.applyAPISingleCallback(prim, apiSchema)

    return apiSchema.Apply(prim)

def _apply_api_multiple(prim, apiSchema, api_prefix, multiple_api_token, operationTracker):
    if (operationTracker and operationTracker.applyAPIMultipleCallback):
        operationTracker.applyAPIMultipleCallback(prim, apiSchema, api_prefix, multiple_api_token)

    return apiSchema.Apply(prim, multiple_api_token)

def create_vehicle(stage, vehicleData, operationTracker: OperationTracker = None):
    messageList = []  # error/warning messages

    verticalAxisVector = vehicleData.get_vertical_axis_vector()
    longitudinalAxisVector = vehicleData.get_longitudinal_axis_vector()
    lateralAxisVector = vehicleData.get_lateral_axis_vector()
    lateralAxisUsdGeomToken = UsdGeom.Tokens.x
    if (vehicleData.verticalAxis == VehicleData.AXIS_X):
        if (vehicleData.longitudinalAxis == VehicleData.AXIS_Y):
            lateralAxisUsdGeomToken = UsdGeom.Tokens.z
        else:
            lateralAxisUsdGeomToken = UsdGeom.Tokens.y
    elif (vehicleData.longitudinalAxis == VehicleData.AXIS_X):
        if (vehicleData.verticalAxis == VehicleData.AXIS_Y):
            lateralAxisUsdGeomToken = UsdGeom.Tokens.z
        else:
            lateralAxisUsdGeomToken = UsdGeom.Tokens.y

    # Determine if the simulation is running.
    timeline = omni.timeline.get_timeline_interface()
    isStopped = timeline.is_stopped()

    lengthScaleSqr = vehicleData.unitScale.lengthScale * vehicleData.unitScale.lengthScale
    kgmsScale = vehicleData.unitScale.massScale * lengthScaleSqr
    forceScale = vehicleData.unitScale.massScale * vehicleData.unitScale.lengthScale

    (moiScaling, stiffnessScaling, topSpeedScaling, radiusRatio, massRatio) = compute_param_scaling_factors_average(vehicleData)
    contactOffset = vehicleData.chassisLength * (REFERENCE_CONTACT_OFFSET / REFERENCE_CHASSIS_LENGTH)

    # vehicle context, physics scene
    vehicleContext = None
    scenePrim = None
    scenePath = None

    gravityDirection = -verticalAxisVector
    gravityMagnitude = 9.81 * vehicleData.unitScale.lengthScale
    sceneStepsPerSecond = 60.0

    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            scenePath = prim.GetPath()
            if (operationTracker):
                operationTracker.scenePath = scenePath
            scenePrim = prim
            scene = UsdPhysics.Scene(scenePrim)
            gravityDirection = scene.GetGravityDirectionAttr().Get()
            gravityMagnitude = scene.GetGravityMagnitudeAttr().Get()

            # The default gravity value is -inf. Scene.cpp sets the gravity to 9.81 * lengthScale
            # when this default value is encountered, so mimic that behavior here.
            if gravityMagnitude < 0.0:
                gravityMagnitude = 9.81 * vehicleData.unitScale.lengthScale

            if scenePrim.HasAPI(PhysxSchema.PhysxVehicleContextAPI):
                vehicleContext = PhysxSchema.PhysxVehicleContextAPI(scenePrim)

            if scenePrim.HasAPI(PhysxSchema.PhysxSceneAPI):
                sceneAPI = PhysxSchema.PhysxSceneAPI(scenePrim)
                sceneStepsPerSecond = sceneAPI.GetTimeStepsPerSecondAttr().Get()

            break

    if (scenePrim is None) and not isStopped:

        sceneErrorMessage = ("A physics scene is needed but it cannot be created while the physics simulation is " +
                                "running. Please stop the simulation and create the vehicle again.")

        messageList.append(sceneErrorMessage)

        carb.log_error(sceneErrorMessage)

        return (messageList, operationTracker)

    (rootSharedScope, _) = _get_or_create_prim(stage, vehicleData.rootSharedPath,
        UsdGeom.Scope, operationTracker)

    if (scenePrim is None):
        scenePrimCreated = True
        scenePath = vehicleData.rootSharedPath + DEFAULT_SCENE_PATH

        scene = UsdPhysics.Scene.Define(stage, scenePath)
        if (operationTracker):
            operationTracker.createdPrimPaths.append(scenePath)
            operationTracker.scenePath = scenePath
        scene.CreateGravityDirectionAttr(gravityDirection)
        scene.CreateGravityMagnitudeAttr(gravityMagnitude)
        scenePrim = scene.GetPrim()

        # Apply the scene API so the time steps per second can be changed.
        PhysxSchema.PhysxSceneAPI.Apply(scenePrim)
    else:
        scenePrimCreated = False

    if (vehicleContext is None):
        vehicleContext = _apply_api_single(scenePrim, PhysxSchema.PhysxVehicleContextAPI, None if scenePrimCreated else operationTracker)
        vehicleContext.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
        vehicleContext.CreateVerticalAxisAttr(_get_token_from_axis(vehicleData.verticalAxis))
        vehicleContext.CreateLongitudinalAxisAttr(_get_token_from_axis(vehicleData.longitudinalAxis))

    # materials
    materialPath = vehicleData.rootSharedPath + VEHICLE_GROUND_MATERIAL_PATH

    (usdMaterial, primCreated) = _get_or_create_prim(stage, materialPath,
        UsdShade.Material, operationTracker)
    if (primCreated):
        materialPrim = usdMaterial.GetPrim()
        material = UsdPhysics.MaterialAPI.Apply(materialPrim)
        material.CreateStaticFrictionAttr(0.9)
        material.CreateDynamicFrictionAttr(0.7)
        material.CreateRestitutionAttr(0.0)
        PhysxSchema.PhysxMaterialAPI.Apply(materialPrim)

    # tire friction tables
    frictionTablePath = ""

    if vehicleData.tireTypeIndex is TIRE_TYPE_SUMMER:
        frictionTablePath = vehicleData.rootSharedPath + "/SummerTireFrictionTable"

        (summerTire, primCreated) = _get_or_create_prim(stage, frictionTablePath,
            PhysxSchema.PhysxVehicleTireFrictionTable, operationTracker)
        if (primCreated):
            summerTireGroundMaterialsRel = summerTire.CreateGroundMaterialsRel()
            summerTireGroundMaterialsRel.AddTarget(materialPath)
            summerTire.CreateFrictionValuesAttr([0.75])

    elif vehicleData.tireTypeIndex is TIRE_TYPE_ALL_SEASON:
        frictionTablePath = vehicleData.rootSharedPath + "/AllSeasonFrictionTable"

        (allSeason, primCreated) = _get_or_create_prim(stage, frictionTablePath,
            PhysxSchema.PhysxVehicleTireFrictionTable, operationTracker)
        if (primCreated):
            allSeasonGroundMaterialsRel = allSeason.CreateGroundMaterialsRel()
            allSeasonGroundMaterialsRel.AddTarget(materialPath)
            allSeason.CreateFrictionValuesAttr([0.5])

    else:
        frictionTablePath = vehicleData.rootSharedPath + "/SlickTireFrictionTable"

        (slickTire, primCreated) = _get_or_create_prim(stage, frictionTablePath,
            PhysxSchema.PhysxVehicleTireFrictionTable, operationTracker)
        if (primCreated):
            slickTireGroundMaterialsRel = slickTire.CreateGroundMaterialsRel()
            slickTireGroundMaterialsRel.AddTarget(materialPath)
            slickTire.CreateFrictionValuesAttr([1.0])

    # collision groups
    collisionGroupVehicleChassisPath = vehicleData.rootSharedPath + VEHICLE_COLLISION_GROUP_CHASSIS_PATH
    collisionGroupVehicleWheelPath = vehicleData.rootSharedPath + VEHICLE_COLLISION_GROUP_WHEEL_PATH
    collisionGroupVehicleGroundQueryPath = vehicleData.rootSharedPath + VEHICLE_COLLISION_GROUP_GROUND_QUERY_PATH
    collisionGroupGroundSurfacePath = vehicleData.rootSharedPath + VEHICLE_COLLISION_GROUP_GROUND_SURFACE_PATH

    (collisionGroupVehicleChassis, primCreated) = _get_or_create_prim(stage, collisionGroupVehicleChassisPath,
        UsdPhysics.CollisionGroup, operationTracker)
    if (primCreated):
        collisionGroupVehicleChassisRel = collisionGroupVehicleChassis.CreateFilteredGroupsRel()
        collisionGroupVehicleChassisRel.AddTarget(collisionGroupVehicleGroundQueryPath)

    (collisionGroupVehicleWheel, primCreated) = _get_or_create_prim(stage, collisionGroupVehicleWheelPath,
        UsdPhysics.CollisionGroup, operationTracker)
    if (primCreated):
        collisionGroupVehicleWheelRel = collisionGroupVehicleWheel.CreateFilteredGroupsRel()
        collisionGroupVehicleWheelRel.AddTarget(collisionGroupVehicleWheelPath)
        collisionGroupVehicleWheelRel.AddTarget(collisionGroupVehicleGroundQueryPath)

    (collisionGroupVehicleGroundQuery, primCreated) = _get_or_create_prim(stage, collisionGroupVehicleGroundQueryPath,
        UsdPhysics.CollisionGroup, operationTracker)
    if (primCreated):
        collisionGroupVehicleGroundQueryRel = collisionGroupVehicleGroundQuery.CreateFilteredGroupsRel()
        collisionGroupVehicleGroundQueryRel.AddTarget(collisionGroupVehicleChassisPath)
        collisionGroupVehicleGroundQueryRel.AddTarget(collisionGroupVehicleWheelPath)

    (collisionGroupGroundSurface, primCreated) = _get_or_create_prim(stage, collisionGroupGroundSurfacePath,
        UsdPhysics.CollisionGroup, operationTracker)
    if (primCreated):
        collisionGroupGroundSurfaceRel = collisionGroupGroundSurface.CreateFilteredGroupsRel()
        collisionGroupGroundSurfaceRel.AddTarget(collisionGroupGroundSurfacePath)
        collisionGroupGroundSurfaceRel.AddTarget(collisionGroupVehicleWheelPath)

    # vehicle prim
    chassisHalfHeight = 0.5 * vehicleData.chassisHeight
    chassisDistToGround = vehicleData.tireRadius
    centerToGround = chassisHalfHeight + chassisDistToGround

    if (vehicleData.vehiclePath):
        vehiclePrimCreated = False
        rootVehiclePath = vehicleData.vehiclePath
        vehiclePath = vehicleData.vehiclePath

        def validatePath(stage, path, primType, messageList):
            prim = stage.GetPrimAtPath(path)
            if (prim):
                if (prim.IsA(UsdGeom.Xformable)):
                    return prim
                else:
                    xformErrorMessage = f"The specified {primType} prim at path \"{path}\" is not a UsdGeomXformable."
                    messageList.append(xformErrorMessage)
                    carb.log_error(xformErrorMessage)

            else:
                primErrorMessage = f"The specified {primType} prim at path \"{path}\" does not exist."
                messageList.append(primErrorMessage)
                carb.log_error(primErrorMessage)

            return None

        # make sure user defined wheel attachment prims fulfill the requirements
        for axle in range(vehicleData.numberOfAxles):
            for side in range(2):
                wheelIndex = 2 * axle + side
                wheelAttPath = vehicleData.tireList[wheelIndex].path
                if (wheelAttPath):
                    prim = validatePath(stage, wheelAttPath, "wheel attachment", messageList)
                    if (prim is None):
                        return (messageList, operationTracker)

        vehiclePrim = validatePath(stage, vehiclePath, "vehicle", messageList)
        if (vehiclePrim):
            vehicleXform = UsdGeom.Xform(vehiclePrim)

            vehicleMatrix = vehicleXform.ComputeLocalToWorldTransform(0)
            vehicleMatrixInverse = vehicleMatrix.GetInverse()  # note: transpose is not enough if scale is involved
            vehicleTransform = Gf.Transform(vehicleMatrix)
            vehiclePosition = vehicleTransform.GetTranslation()
            vehicleOrientation = vehicleTransform.GetRotation()
            vehicleScale = Gf.Vec3f(vehicleTransform.GetScale())
            vehicleScaleInverse = Gf.Vec3f(1.0 / vehicleScale[0], 1.0 / vehicleScale[1], 1.0 / vehicleScale[2])
        else:
            return (messageList, operationTracker)

    else:
        # make sure the user has not defined a path for any wheel attachment prim
        for axle in range(vehicleData.numberOfAxles):
            for side in range(2):
                wheelIndex = 2 * axle + side
                wheelAttPath = vehicleData.tireList[wheelIndex].path
                if (wheelAttPath):
                    wheelAttPathErrorMessage = f"A wheel attachment path can not be defined unless a path for the vehicle prim has been defined too ({wheelAttPath})."
                    messageList.append(wheelAttPathErrorMessage)
                    carb.log_error(wheelAttPathErrorMessage)
                    return (messageList, operationTracker)

        rootVehiclePath = vehicleData.rootVehiclePath
        vehiclePath = rootVehiclePath + "/Vehicle"

        (rootVehicleScope, _) = _get_or_create_prim(stage, rootVehiclePath,
            UsdGeom.Scope, operationTracker)

        if (vehicleData.position is not None):
            vehiclePosition = vehicleData.position
        else:
            vehiclePosition = centerToGround * verticalAxisVector

        (vehicleXform, vehiclePrimCreated) = _get_or_create_prim(stage, vehiclePath,
            UsdGeom.Xform, operationTracker)

        vehiclePrim = vehicleXform.GetPrim()

        vehicleXform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(vehiclePosition)
        vehicleXform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Quatf(1, 0, 0, 0))
        vehicleXform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(1.0))

    # tires
    longStiffness = (REFERENCE_TIRE_LONGITUDINAL_STIFFNESS * forceScale) * stiffnessScaling * topSpeedScaling
    camberStiffness = (REFERENCE_TIRE_CAMBER_STIFFNESS * forceScale) * massRatio
    # note: using massRatio only instead of stiffnessScaling because for the lateral camber force, we are not
    #       interested in scaling according to torque but force only
    tireRestLoadEst = (vehicleData.chassisMass / (vehicleData.numberOfAxles * 2)) * gravityMagnitude
    maxLateralStiffness = 17 * tireRestLoadEst

    tirePath = None
    if vehicleData.createShareableComponents:
        if vehicleData.tireTypeIndex is TIRE_TYPE_SUMMER:
            tirePath = rootVehiclePath + "/SummerTire"
        elif vehicleData.tireTypeIndex is TIRE_TYPE_ALL_SEASON:
            tirePath = rootVehiclePath + "/AllSeasonTire"
        else:
            tirePath = rootVehiclePath + "/SlickTire"

        (tireScope, tirePrimCreated) = _get_or_create_prim(stage, tirePath,
            UsdGeom.Scope, operationTracker)
        tirePrim = tireScope.GetPrim()

        _setUpTire(tirePrim, longStiffness, camberStiffness,
            maxLateralStiffness, frictionTablePath, None if tirePrimCreated else operationTracker)

    # wheels
    wheelRadius = vehicleData.tireRadius
    wheelWidth = vehicleData.tireWidth
    wheelMass = vehicleData.tireMass

    if vehicleData.numberOfAxles == 1:
        wheelSpacing = 0.0
        wheelPosition = 0.0
    else:
        wheelSpacing = (vehicleData.chassisLength - 4.0 * wheelRadius) / (vehicleData.numberOfAxles - 1)
        wheelPosition = 0.5 * vehicleData.chassisLength - 2.0 * wheelRadius

    axlePosition = wheelPosition

    # center of mass
    totalWeightDistribution = 0.0

    for axle in range(vehicleData.numberOfAxles):
        totalWeightDistribution = totalWeightDistribution + vehicleData.weightDistribution[axle]

    averageWheelDistance = 0.0

    wheelPosList = []
    wheelPosScaledList = []

    hasChassisOffset = (vehicleData.vehiclePath) and (vehicleData.position is not None)
    # the user defined a vehicle prim and scanned prims for a chassis position and volume
    # estimate. The offset between the vehicle prim position and the chassis position has
    # to be taken into account for center of mass estimation and for positioning the wheels
    # (if not specified by the user).

    if (hasChassisOffset):
        chassisLocalPosition = Gf.Vec3f(vehicleMatrixInverse.TransformAffine(vehicleData.position))
        chassisLocalPositionWorldScale = Gf.CompMult(chassisLocalPosition, vehicleScale)
        chassisLocalPositionWorldScaleLongitudinalPart = chassisLocalPositionWorldScale.GetDot(longitudinalAxisVector)

    for axle in range(vehicleData.numberOfAxles):
        sprungMassPercent = 0.5 * vehicleData.weightDistribution[axle] / totalWeightDistribution

        for side in range(2):
            wheelIndex = 2 * axle + side

            wheelAttachPathUser = vehicleData.tireList[wheelIndex].path
            if (wheelAttachPathUser):
                # note: validation happens further above
                wheelAttachmentPrim = stage.GetPrimAtPath(wheelAttachPathUser)
                wheelAttachmentXform = UsdGeom.Xform(wheelAttachmentPrim)

                wheelAttMatrix = wheelAttachmentXform.ComputeLocalToWorldTransform(0)
                wheelAttTransform = Gf.Transform(wheelAttMatrix)

                wheelPosWorld = wheelAttTransform.GetTranslation()
                wheelPosScaled = Gf.Vec3f(vehicleMatrixInverse.TransformAffine(wheelPosWorld))
                wheelPos = Gf.CompMult(wheelPosScaled, vehicleScale)  # transform to world scale

                wheelDistance = wheelPos.GetDot(longitudinalAxisVector)
            else:
                if (vehicleData.tireList[wheelIndex].position is None):
                    wheelPos = None
                    wheelPosScaled = None

                    wheelDistance = axlePosition

                    if (hasChassisOffset):
                        wheelDistance = wheelDistance + chassisLocalPositionWorldScaleLongitudinalPart
                else:
                    wheelPosWorld = vehicleData.tireList[wheelIndex].position

                    if (vehicleData.vehiclePath):
                        wheelPosScaled = Gf.Vec3f(vehicleMatrixInverse.TransformAffine(wheelPosWorld))
                        wheelPos = Gf.CompMult(wheelPosScaled, vehicleScale)  # transform to world scale
                    else:
                        wheelPos = wheelPosWorld - vehiclePosition
                        wheelPosScaled = wheelPos

                    wheelDistance = wheelPos.GetDot(longitudinalAxisVector)

            wheelPosList.append(wheelPos)
            wheelPosScaledList.append(wheelPosScaled)

            weightedWheelDistance = sprungMassPercent * wheelDistance
            averageWheelDistance = averageWheelDistance + weightedWheelDistance

        axlePosition = axlePosition - wheelSpacing

    centerOfMass = 0.1 * vehicleData.chassisHeight + vehicleData.tireRadius
    centerOfMassOffset = (centerOfMass - centerToGround) * verticalAxisVector + averageWheelDistance * longitudinalAxisVector

    if (vehicleData.vehiclePath):
        # take vehicle prim scale into account
        centerOfMassOffset = Gf.CompMult(centerOfMassOffset, vehicleScaleInverse)

        if (hasChassisOffset):
            verticalPart = chassisLocalPosition.GetDot(verticalAxisVector)
            lateralPart = chassisLocalPosition.GetDot(lateralAxisVector)
            centerOfMassOffset = (centerOfMassOffset + (verticalPart * verticalAxisVector) +
                (lateralPart * lateralAxisVector))
            # the longitudinal part was computed using the vehicle local space positions already

    tireDampingRate = REFERENCE_WHEEL_DAMPING_RATE * kgmsScale * moiScaling

    drivenWheelIndexList = []
    leftWheelsIndexList = []
    rightWheelsIndexList = []
    steerableWheelsIndexList = []
    steerableWheelsMaxAngleList = []
    maxNaturalFrequency = -1.0
    maxAbsSteerAngleDegrees = 0.0
    setUpAckermannCorrection = (vehicleData.useAckermannCorrection and
        (vehicleData.numberOfAxles > 1) and
        (not vehicleData.tankMode) and
        (vehicleData.driveTypeIndex != DRIVE_TYPE_NONE))
    firstAxleWheelIndexList = []  # only filled in if setUpAckermannCorrection is true
    wheelPositionsFirstAxle = []  # only filled in if setUpAckermannCorrection is true
    wheelPositionsLastAxle = []  # only filled in if setUpAckermannCorrection is true
    for axle in range(vehicleData.numberOfAxles):

        maxAbsSteerAngleDegrees = max(math.fabs(vehicleData.maxSteerAngle[axle]), maxAbsSteerAngleDegrees)

        # suspensions
        maxWheelRadius = max(vehicleData.tireList[2 * axle + 0].radius, vehicleData.tireList[2 * axle + 1].radius)
        sprungMassPercent = vehicleData.weightDistribution[axle] / totalWeightDistribution / 2.0
        sprungMass = sprungMassPercent * vehicleData.chassisMass

        maxCompression = 0.5 * maxWheelRadius
        maxDroop = maxCompression
        suspensionTravelDistance = maxCompression + maxDroop

        # to maintain a linear spring, the spring force must decay to zero at max droop
        springStiffness = sprungMass * gravityMagnitude / maxDroop

        naturalFrequency = math.sqrt(springStiffness / sprungMass) / (2.0 * math.pi)
        maxNaturalFrequency = max(naturalFrequency, maxNaturalFrequency)

        # compute the damping rate from the spring stiffness
        dampingRate = 2.0 * vehicleData.damping[axle] * math.sqrt(springStiffness * sprungMass)

        suspensionPath = None

        if vehicleData.createShareableComponents:
            suspensionPath = rootVehiclePath + "/Suspension" + str(axle + 1)
            (suspensionScope, suspensionPrimCreated) = _get_or_create_prim(stage, suspensionPath,
                UsdGeom.Scope, operationTracker)
            suspensionPrim = suspensionScope.GetPrim()

            _setUpSuspension(suspensionPrim, springStiffness, dampingRate, suspensionTravelDistance, None if suspensionPrimCreated else operationTracker)

        # wheel attachments
        for side in range(2):
            wheelIndex = 2 * axle + side

            wheelPos = wheelPosList[wheelIndex]
            wheelPosScaled = wheelPosScaledList[wheelIndex]

            wheelAttachPathUser = vehicleData.tireList[wheelIndex].path

            wheelRadius = vehicleData.tireList[wheelIndex].radius
            wheelWidth = vehicleData.tireList[wheelIndex].width
            wheelMass = vehicleData.tireList[wheelIndex].mass

            wheelMoi = 0.5 * wheelMass * wheelRadius * wheelRadius

            if side == 0:
                wheelPath = rootVehiclePath + "/LeftWheel" + str(axle + 1)
                sign = 1.0
                leftWheelsIndexList.append(wheelIndex)
            else:
                wheelPath = rootVehiclePath + "/RightWheel" + str(axle + 1)
                sign = -1.0
                rightWheelsIndexList.append(wheelIndex)

            if ((vehicleData.tireList[wheelIndex].position is None) and
                (not wheelAttachPathUser)):

                wheelPos = (sign * 0.5 * vehicleData.chassisWidth * lateralAxisVector
                    - chassisHalfHeight * verticalAxisVector
                    + wheelPosition * longitudinalAxisVector)

                if (vehicleData.vehiclePath):
                    if (hasChassisOffset):
                        wheelPos = wheelPos + chassisLocalPositionWorldScale

                    # take vehicle prim scale into account
                    wheelPosScaled = Gf.CompMult(wheelPos, vehicleScaleInverse)
                else:
                    wheelPosScaled = wheelPos

            if (setUpAckermannCorrection):
                if (axle == 0):
                    wheelPositionsFirstAxle.append(wheelPos)
                    firstAxleWheelIndexList.append(wheelIndex)
                elif (axle == (vehicleData.numberOfAxles - 1)):
                    wheelPositionsLastAxle.append(wheelPos)

            if (wheelAttachPathUser):
                wheelAttachmentPrimCreated = False
                wheelAttachPath = wheelAttachPathUser

                # note: validation happens further above
                wheelAttachmentPrim = stage.GetPrimAtPath(wheelAttachPathUser)
            else:
                if side == 0:
                    wheelAttachPath = vehiclePath + "/LeftWheel" + str(axle + 1) + "References"
                else:
                    wheelAttachPath = vehiclePath + "/RightWheel" + str(axle + 1) + "References"

                (wheelAttachmentXform, wheelAttachmentPrimCreated) = _get_or_create_prim(stage, wheelAttachPath,
                    UsdGeom.Xform, operationTracker)
                wheelAttachmentPrim = wheelAttachmentXform.GetPrim()
                wheelAttachmentXform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(wheelPosScaled)

            wheelAttachmentAPI = _apply_api_single(wheelAttachmentPrim, PhysxSchema.PhysxVehicleWheelAttachmentAPI, None if wheelAttachmentPrimCreated else operationTracker)

            # wheels
            wheelPrim = None
            if vehicleData.createShareableComponents:
                (wheelScope, wheelPrimCreated) = _get_or_create_prim(stage, wheelPath,
                    UsdGeom.Scope, operationTracker)
                wheelPrim = wheelScope.GetPrim()

                wheelRel = wheelAttachmentAPI.CreateWheelRel()
                wheelRel.AddTarget(wheelPath)

                tireRel = wheelAttachmentAPI.CreateTireRel()
                tireRel.AddTarget(tirePath)

                suspensionRel = wheelAttachmentAPI.CreateSuspensionRel()
                suspensionRel.AddTarget(suspensionPath)
            else:
                wheelPrim = wheelAttachmentPrim
                wheelPrimCreated = wheelAttachmentPrimCreated

                _setUpSuspension(wheelAttachmentPrim, springStiffness, dampingRate,
                    suspensionTravelDistance, None if wheelAttachmentPrimCreated else operationTracker)

                _setUpTire(wheelAttachmentPrim, longStiffness, camberStiffness,
                    maxLateralStiffness, frictionTablePath, None if wheelAttachmentPrimCreated else operationTracker)

            _setUpWheel(wheelPrim, wheelRadius, wheelWidth, wheelMass, wheelMoi,
                tireDampingRate, None if wheelPrimCreated else operationTracker)

            if (vehicleData.maxSteerAngle[axle] != 0.0):
                steerableWheelsIndexList.append(wheelIndex)
                steerableWheelsMaxAngleList.append(vehicleData.maxSteerAngle[axle])

            collisionGroupRel = wheelAttachmentAPI.CreateCollisionGroupRel()
            collisionGroupRel.AddTarget(collisionGroupVehicleGroundQueryPath)

            wheelAttachmentAPI.CreateSuspensionTravelDirectionAttr(-verticalAxisVector)

            suspFramePosLocal = wheelPos + (verticalAxisVector * maxCompression)
            if (vehicleData.vehiclePath):
                suspFramePosLocalScaled = Gf.CompMult(suspFramePosLocal, vehicleScaleInverse)
            else:
                suspFramePosLocalScaled = suspFramePosLocal
            wheelAttachmentAPI.CreateSuspensionFramePositionAttr(suspFramePosLocalScaled)
            # note: the orientation of a user defined wheel attachment prim is ignored for now. The
            #       complexity to make use of it seems too high. Thinking of cases where the vehicle
            #       frame is not axis aligned, such that the suspension frame could compensate for
            #       it. But then the whole volume estimation based on AABBs would need to change as
            #       well and some estimation about the "real" longitudinal/lateral/vertical axes
            #       would be required etc. etc.

            wheelAttachmentAPI.CreateIndexAttr(wheelIndex)
            if (vehicleData.driven[axle]):
                drivenWheelIndexList.append(wheelIndex)

            if vehicleData.driveTypeIndex is DRIVE_TYPE_NONE:
                wheelControllerAPI = _apply_api_single(wheelAttachmentPrim, PhysxSchema.PhysxVehicleWheelControllerAPI, None if wheelAttachmentPrimCreated else operationTracker)
                wheelControllerAPI.CreateDriveTorqueAttr(0)
                wheelControllerAPI.CreateBrakeTorqueAttr(0)
                wheelControllerAPI.CreateSteerAngleAttr(0)

            # wheel collision
            if (vehicleData.wheelCollisionGeometry and (not wheelAttachPathUser)):
                # creating or setting collision shapes is not supported by the wizard if the user defines the
                # wheel attachment prims (lot of stuff that would have to be taken care of: path needs to be
                # unique, case where the collision prim was generated already, scale of attachment prim to
                # consider etc. etc.
                vehicleWheelCollPath = wheelAttachPath + "/Collision"
                (vehicleWheelColl, _) = _get_or_create_prim(stage, vehicleWheelCollPath,
                    UsdGeom.Cylinder, operationTracker)
                vehicleWheelColl.CreatePurposeAttr(UsdGeom.Tokens.guide)
                vehicleWheelColl.CreateAxisAttr(lateralAxisUsdGeomToken)
                vehicleWheelColl.CreateHeightAttr(wheelWidth)
                vehicleWheelColl.CreateRadiusAttr(wheelRadius)
                # if height or radius is authored, USD expects extent to be authored too
                cylExtent = UsdGeom.Cylinder.ComputeExtentFromPlugins(vehicleWheelColl, 0)
                vehicleWheelColl.CreateExtentAttr(cylExtent)

                if (vehicleData.vehiclePath):
                    # the user provided vehicle prim might be scaled, however, the radius and width are computed
                    # in world scale, thus adding a scale operation to undo that scale. Using the world
                    # space scale vector directly since the wizard in general assumes the vehicle is world axis
                    # aligned (also, non-uniform scale on the vehicle root prim will not work anyway because as
                    # soon as the wheel rotates, the geom will get scaled in undesired ways).
                    vehicleWheelColl.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(vehicleScaleInverse)

                vehicleWheelCollPrim = vehicleWheelColl.GetPrim()

                UsdPhysics.CollisionAPI.Apply(vehicleWheelCollPrim)

                wheelCollPrimPath = vehicleWheelCollPrim.GetPath()

                if (not is_in_collision_group(stage, wheelCollPrimPath, collisionGroupVehicleWheelPath)):
                    add_collision_to_collision_group(stage, wheelCollPrimPath, collisionGroupVehicleWheelPath)
                    if (operationTracker):
                        operationTracker.collisionGroupIncludeList.append(CollisionGroupIncludeData(wheelCollPrimPath, collisionGroupVehicleWheelPath))

                physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(vehicleWheelCollPrim)
                physxCollisionAPI.CreateRestOffsetAttr(0.0 * vehicleData.unitScale.lengthScale)
                physxCollisionAPI.CreateContactOffsetAttr(contactOffset)

            # wheel render
            if (not wheelAttachPathUser):
                vehicleWheelRenderPath = wheelAttachPath + "/Render"
                (vehicleWheelRender, _) = _get_or_create_prim(stage, vehicleWheelRenderPath,
                    UsdGeom.Cylinder, operationTracker)
                vehicleWheelRender.CreateAxisAttr(lateralAxisUsdGeomToken)
                vehicleWheelRender.CreateHeightAttr(wheelWidth)
                vehicleWheelRender.CreateRadiusAttr(wheelRadius)
                # if height or radius is authored, USD expects extent to be authored too
                cylExtent = UsdGeom.Cylinder.ComputeExtentFromPlugins(vehicleWheelRender, 0)
                vehicleWheelRender.CreateExtentAttr(cylExtent)

                if (vehicleData.vehiclePath):
                    # see comment for wheel collision prim above
                    vehicleWheelRender.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(vehicleScaleInverse)

        wheelPosition = wheelPosition - wheelSpacing

    drivePath = None

    if vehicleData.createShareableComponents:
        drivePath = rootVehiclePath + "/Drive"

    peakTorque = (vehicleData.horsepower * HORSEPOWER_CONVERSION_CONSTANT / vehicleData.engineMaxRPM) * kgmsScale
    # note: no moiScaling or topSpeedScaling as this is included in the horsepower computation

    if vehicleData.driveTypeIndex is DRIVE_TYPE_STANDARD:

        if vehicleData.createShareableComponents:
            enginePath = rootVehiclePath + "/Engine"
            (engineScope, enginePrimCreated) = _get_or_create_prim(stage, enginePath,
                UsdGeom.Scope, operationTracker)
            enginePrim = engineScope.GetPrim()

            gearsPath = rootVehiclePath + "/Gears"
            (gearsScope, gearsPrimCreated) = _get_or_create_prim(stage, gearsPath,
                UsdGeom.Scope, operationTracker)
            gearsPrim = gearsScope.GetPrim()

            autoGearBoxPath = rootVehiclePath + "/AutoGearBox"
            (autoGearBoxScope, autoGearBoxPrimCreated) = _get_or_create_prim(stage, autoGearBoxPath,
                UsdGeom.Scope, operationTracker)
            autoGearBoxPrim = autoGearBoxScope.GetPrim()

            clutchPath = rootVehiclePath + "/Clutch"
            (clutchScope, clutchPrimCreated) = _get_or_create_prim(stage, clutchPath,
                UsdGeom.Scope, operationTracker)
            clutchPrim = clutchScope.GetPrim()

            (driveScope, drivePrimCreated) = _get_or_create_prim(stage, drivePath,
                UsdGeom.Scope, operationTracker)
            drivePrim = driveScope.GetPrim()

            driveAPI = _apply_api_single(drivePrim, PhysxSchema.PhysxVehicleDriveStandardAPI, None if drivePrimCreated else operationTracker)
            engineRel = driveAPI.CreateEngineRel()
            engineRel.AddTarget(enginePath)
            gearsRel = driveAPI.CreateGearsRel()
            gearsRel.AddTarget(gearsPath)
            autoGearBoxRel = driveAPI.CreateAutoGearBoxRel()
            autoGearBoxRel.AddTarget(autoGearBoxPath)
            clutchRel = driveAPI.CreateClutchRel()
            clutchRel.AddTarget(clutchPath)
        else:
            enginePrim = vehiclePrim
            enginePrimCreated = vehiclePrimCreated
            gearsPrim = vehiclePrim
            gearsPrimCreated = vehiclePrimCreated
            autoGearBoxPrim = vehiclePrim
            autoGearBoxPrimCreated = vehiclePrimCreated
            clutchPrim = vehiclePrim
            clutchPrimCreated = vehiclePrimCreated
            drivePrim = vehiclePrim
            drivePrimCreated = vehiclePrimCreated
            _apply_api_single(drivePrim, PhysxSchema.PhysxVehicleDriveStandardAPI, None if drivePrimCreated else operationTracker)

        # engine
        engineAPI = _apply_api_single(enginePrim, PhysxSchema.PhysxVehicleEngineAPI, None if enginePrimCreated else operationTracker)
        engineAPI.CreateMoiAttr(REFERENCE_ENGINE_MOI * kgmsScale * moiScaling)
        engineAPI.CreatePeakTorqueAttr(peakTorque)
        maxRotSpeed = vehicleData.engineMaxRPM * 2.0 * math.pi / 60.0
        engineAPI.CreateMaxRotationSpeedAttr(maxRotSpeed)
        engineAPI.CreateTorqueCurveAttr([Gf.Vec2f(0.0, 0.8), Gf.Vec2f(0.33, 1.0), Gf.Vec2f(1.0, 0.8)])
        engineAPI.CreateDampingRateFullThrottleAttr(REFERENCE_ENGINE_DAMPING_RATE_FULL_THROTTLE * kgmsScale * moiScaling)
        engineAPI.CreateDampingRateZeroThrottleClutchEngagedAttr(REFERENCE_ENGINE_DAMPING_RATE_ZERO_THROTTLE_CLUTCH_ENGAGED * kgmsScale * moiScaling)
        engineAPI.CreateDampingRateZeroThrottleClutchDisengagedAttr(REFERENCE_ENGINE_DAMPING_RATE_ZERO_THROTTLE_CLUTCH_DISENGAGED * kgmsScale * moiScaling)

        # gears
        # use an exponential decay function to compute the gear ratios
        # compute tau so the first gear ratio is equal to the number of gears, e.g. 5:1 with 5 total gears,
        # and the gear ratio of the top gear is 1:1.
        gearRatio = []

        if vehicleData.numberOfGears > 1:
            tau = -(vehicleData.numberOfGears - 1) / math.log(1.0 / vehicleData.numberOfGears)

            for gear in range(1, vehicleData.numberOfGears + 1):
                gearRatio.append(vehicleData.numberOfGears * math.exp(-(gear - 1) / tau))

        else:
            gearRatio.append(1.0)

        # add the reverse gear
        gearRatio.insert(0, -gearRatio[0])

        gearsAPI = _apply_api_single(gearsPrim, PhysxSchema.PhysxVehicleGearsAPI, None if gearsPrimCreated else operationTracker)
        gearsAPI.CreateRatiosAttr(gearRatio)
        gearsAPI.CreateRatioScaleAttr(4.0)
        gearsAPI.CreateSwitchTimeAttr(0.5)

        # auto gear box
        upRatio = [0.7] * (vehicleData.numberOfGears - 1)
        downRatio = [0.5] * (vehicleData.numberOfGears - 1)

        autoGearBoxAPI = _apply_api_single(autoGearBoxPrim, PhysxSchema.PhysxVehicleAutoGearBoxAPI, None if autoGearBoxPrimCreated else operationTracker)
        autoGearBoxAPI.CreateUpRatiosAttr(upRatio)
        autoGearBoxAPI.CreateDownRatiosAttr(downRatio)
        autoGearBoxAPI.CreateLatencyAttr(2.0)

        # clutch
        clutchAPI = _apply_api_single(clutchPrim, PhysxSchema.PhysxVehicleClutchAPI, None if clutchPrimCreated else operationTracker)
        clutchStrength = REFERENCE_CLUTCH_STRENGTH * moiScaling * kgmsScale
        clutchAPI.CreateStrengthAttr(clutchStrength)

    elif vehicleData.driveTypeIndex is DRIVE_TYPE_BASIC:

        # drive
        if vehicleData.createShareableComponents:
            (driveScope, drivePrimCreated) = _get_or_create_prim(stage, drivePath,
                UsdGeom.Scope, operationTracker)
            drivePrim = driveScope.GetPrim()
        else:
            drivePrim = vehiclePrim
            drivePrimCreated = vehiclePrimCreated

        driveAPI = _apply_api_single(drivePrim, PhysxSchema.PhysxVehicleDriveBasicAPI, None if drivePrimCreated else operationTracker)
        driveAPI.CreatePeakTorqueAttr(peakTorque)

    if (vehicleData.driveTypeIndex != DRIVE_TYPE_NONE):

        maxBrakeTorque = REFERENCE_WHEEL_BRAKE_TORQUE * kgmsScale * moiScaling * topSpeedScaling
        if ((vehicleData.driveTypeIndex == DRIVE_TYPE_STANDARD) and (vehicleData.tankMode)):
            # set up one brake configuration for the wheels of the left track
            brakes0API = _apply_api_multiple(vehiclePrim, PhysxSchema.PhysxVehicleBrakesAPI, PhysxSchema.Tokens.physxVehicleBrakes,
                PhysxSchema.Tokens.brakes0, None if vehiclePrimCreated else operationTracker)
            brakes0API.CreateWheelsAttr(leftWheelsIndexList)
            brakes0API.CreateMaxBrakeTorqueAttr(maxBrakeTorque)

            # set up one brake configuration for the wheels of the right track
            brakes1API = _apply_api_multiple(vehiclePrim, PhysxSchema.PhysxVehicleBrakesAPI, PhysxSchema.Tokens.physxVehicleBrakes,
                PhysxSchema.Tokens.brakes1, None if vehiclePrimCreated else operationTracker)
            brakes1API.CreateWheelsAttr(rightWheelsIndexList)
            brakes1API.CreateMaxBrakeTorqueAttr(maxBrakeTorque)
        else:
            # set up one brake configuration that applies to all wheels
            brakes0API = _apply_api_multiple(vehiclePrim, PhysxSchema.PhysxVehicleBrakesAPI, PhysxSchema.Tokens.physxVehicleBrakes,
                PhysxSchema.Tokens.brakes0, None if vehiclePrimCreated else operationTracker)
            brakes0API.CreateMaxBrakeTorqueAttr(maxBrakeTorque)

            # set up a steering configuration for the wheels marked as steerable
            if (maxAbsSteerAngleDegrees != 0.0):
                maxAbsSteerAngle = maxAbsSteerAngleDegrees * math.pi / 180.0
                if (setUpAckermannCorrection):
                    steeringAPI = _apply_api_single(vehiclePrim, PhysxSchema.PhysxVehicleAckermannSteeringAPI, None if vehiclePrimCreated else operationTracker)
                    if (wheelPositionsFirstAxle[0].GetDot(lateralAxisVector) >= 0.0):
                        steeringAPI.CreateWheel0Attr(firstAxleWheelIndexList[1])
                        steeringAPI.CreateWheel1Attr(firstAxleWheelIndexList[0])
                    else:
                        steeringAPI.CreateWheel0Attr(firstAxleWheelIndexList[0])
                        steeringAPI.CreateWheel1Attr(firstAxleWheelIndexList[1])
                    steeringAPI.CreateMaxSteerAngleAttr(maxAbsSteerAngle)

                    deltaWheelPosFront = wheelPositionsFirstAxle[0] - wheelPositionsFirstAxle[1]
                    trackWidth = math.fabs(deltaWheelPosFront.GetDot(lateralAxisVector))
                    firstAxlePosAverage = (wheelPositionsFirstAxle[0] + wheelPositionsFirstAxle[1]) * 0.5
                    lastAxlePosAverage = (wheelPositionsLastAxle[0] + wheelPositionsLastAxle[1]) * 0.5
                    deltaAxlePos = firstAxlePosAverage - lastAxlePosAverage
                    wheelBase = math.fabs(deltaAxlePos.GetDot(longitudinalAxisVector))

                    steeringAPI.CreateWheelBaseAttr(wheelBase)
                    steeringAPI.CreateTrackWidthAttr(trackWidth)
                    steeringAPI.CreateStrengthAttr(1.0)
                else:
                    steeringAPI = _apply_api_single(vehiclePrim, PhysxSchema.PhysxVehicleSteeringAPI, None if vehiclePrimCreated else operationTracker)
                    steeringAPI.CreateWheelsAttr(steerableWheelsIndexList)
                    steeringAPI.CreateMaxSteerAngleAttr(maxAbsSteerAngle)

                    angleMultipliers = [(maxAngle / maxAbsSteerAngleDegrees) for maxAngle in steerableWheelsMaxAngleList]
                    steeringAPI.CreateAngleMultipliersAttr(angleMultipliers)

        if ((vehicleData.driveTypeIndex == DRIVE_TYPE_STANDARD) and (vehicleData.tankMode)):
            tankDifferentialAPI = _apply_api_single(vehiclePrim, PhysxSchema.PhysxVehicleTankDifferentialAPI, None if vehiclePrimCreated else operationTracker)

            tankDifferentialAPI.CreateNumberOfWheelsPerTrackAttr([len(leftWheelsIndexList), len(rightWheelsIndexList)])
            tankDifferentialAPI.CreateThrustIndexPerTrackAttr([0, 1])
            tankDifferentialAPI.CreateTrackToWheelIndicesAttr([0, len(leftWheelsIndexList)])
            tankDifferentialAPI.CreateWheelIndicesInTrackOrderAttr(leftWheelsIndexList + rightWheelsIndexList)

            differentialAPI = PhysxSchema.PhysxVehicleMultiWheelDifferentialAPI(tankDifferentialAPI)  # tank differential also applies multi wheel differential
        elif (drivenWheelIndexList):  # python style guide to check for empty list
            differentialAPI = _apply_api_single(vehiclePrim, PhysxSchema.PhysxVehicleMultiWheelDifferentialAPI, None if vehiclePrimCreated else operationTracker)

        if (drivenWheelIndexList):  # python style guide to check for empty list
            drivenWheelCount = len(drivenWheelIndexList)
            ratio = 1.0 / drivenWheelCount

            differentialAPI.CreateWheelsAttr(drivenWheelIndexList)
            differentialAPI.CreateTorqueRatiosAttr([ratio] * drivenWheelCount)

            if (vehicleData.driveTypeIndex == DRIVE_TYPE_STANDARD):
                differentialAPI.CreateAverageWheelSpeedRatiosAttr([ratio] * drivenWheelCount)

    # vehicle and corresponding rigid body
    rigidBodyAPI = _apply_api_single(vehiclePrim, UsdPhysics.RigidBodyAPI, None if vehiclePrimCreated else operationTracker)

    rigidBodyAPI.CreateVelocityAttr(Gf.Vec3f(0))
    rigidBodyAPI.CreateAngularVelocityAttr(Gf.Vec3f(0))

    vehicleMassBoxDim = (vehicleData.chassisWidth * lateralAxisVector
        + vehicleData.chassisHeight * verticalAxisVector
        + vehicleData.chassisLength * longitudinalAxisVector)
    vehicleMassBoxDim[0] = math.fabs(vehicleMassBoxDim[0])
    vehicleMassBoxDim[1] = math.fabs(vehicleMassBoxDim[1])
    vehicleMassBoxDim[2] = math.fabs(vehicleMassBoxDim[2])

    mass = vehicleData.chassisMass
    massAPI = _apply_api_single(vehiclePrim, UsdPhysics.MassAPI, None if vehiclePrimCreated else operationTracker)
    massAPI.CreateMassAttr(mass)
    massAPI.CreateCenterOfMassAttr(centerOfMassOffset)
    massAPI.CreateDiagonalInertiaAttr(
        Gf.Vec3f(
            (vehicleMassBoxDim[1] * vehicleMassBoxDim[1]) + (vehicleMassBoxDim[2] * vehicleMassBoxDim[2]),
            (vehicleMassBoxDim[0] * vehicleMassBoxDim[0]) + (vehicleMassBoxDim[2] * vehicleMassBoxDim[2]),
            (vehicleMassBoxDim[0] * vehicleMassBoxDim[0]) + (vehicleMassBoxDim[1] * vehicleMassBoxDim[1]),
        )
        * (1 / 12)
        * mass)
    massAPI.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))

    physxRigidBodyAPI = _apply_api_single(vehiclePrim, PhysxSchema.PhysxRigidBodyAPI, None if vehiclePrimCreated else operationTracker)
    physxRigidBodyAPI.CreateDisableGravityAttr(True)

    # the vehicle relies on the rigid body sleep mechanism which is based on an energy threshold
    # with respect to the body velocities. However, the rigid body knows nothing about the rotating
    # wheels, thus the body might fall asleep for one frame during acceleration. It will get woken
    # up again but at that point have lost all energy. Thus, the smaller the vehicle gets, the
    # lower the sleep threshold is set.
    # Note that the threshold would in theory scale with radiusRatio squared but linear scale
    # worked good enough
    sleepThreshold = REFERENCE_SLEEP_THRESHOLD * lengthScaleSqr * radiusRatio
    stabilizationThreshold = REFERENCE_STABILIZATION_THRESHOLD * lengthScaleSqr * radiusRatio
    physxRigidBodyAPI.CreateSleepThresholdAttr(sleepThreshold)
    physxRigidBodyAPI.CreateStabilizationThresholdAttr(stabilizationThreshold)

    vehicleAPI = _apply_api_single(vehiclePrim, PhysxSchema.PhysxVehicleAPI, None if vehiclePrimCreated else operationTracker)
    vehicleAPI.CreateVehicleEnabledAttr(True)
    set_custom_metadata(vehiclePrim, PhysxSchema.Tokens.referenceFrameIsCenterOfMass, False)

    if (vehicleData.queryTypeIndex == QUERY_TYPE_RAYCAST):
        vehicleAPI.CreateSuspensionLineQueryTypeAttr(PhysxSchema.Tokens.raycast)
    else:
        vehicleAPI.CreateSuspensionLineQueryTypeAttr(PhysxSchema.Tokens.sweep)

    # check whether the simulation time step and the vehicle substeps are enough to sample the
    # suspension spring period with enough detail.
    lowForwardSpeedSubstepCount = 3
    highForwardSpeedSubstepCount = 1
    minSimFrequency = maxNaturalFrequency * 10.0  # we want at least 10 samples per period
    substepCount = math.ceil(minSimFrequency / sceneStepsPerSecond)
    if (substepCount > lowForwardSpeedSubstepCount):
        lowForwardSpeedSubstepCount = substepCount
    if (substepCount > highForwardSpeedSubstepCount):
        highForwardSpeedSubstepCount = substepCount

    vehicleAPI.CreateSubStepThresholdLongitudinalSpeedAttr(5.0 * vehicleData.unitScale.lengthScale)
    vehicleAPI.CreateLowForwardSpeedSubStepCountAttr(lowForwardSpeedSubstepCount)
    vehicleAPI.CreateHighForwardSpeedSubStepCountAttr(highForwardSpeedSubstepCount)
    vehicleAPI.CreateMinPassiveLongitudinalSlipDenominatorAttr(4.0 * vehicleData.unitScale.lengthScale)
    vehicleAPI.CreateMinActiveLongitudinalSlipDenominatorAttr(0.1 * vehicleData.unitScale.lengthScale)
    vehicleAPI.CreateMinLateralSlipDenominatorAttr(1.0 * vehicleData.unitScale.lengthScale)

    if vehicleData.driveTypeIndex != DRIVE_TYPE_NONE:
        if vehicleData.createShareableComponents:
            driveRel = vehicleAPI.CreateDriveRel()
            driveRel.AddTarget(drivePath)

        if ((vehicleData.driveTypeIndex == DRIVE_TYPE_STANDARD) and (vehicleData.tankMode)):
            tankVehicleControllerAPI = _apply_api_single(vehiclePrim, PhysxSchema.PhysxVehicleTankControllerAPI, None if vehiclePrimCreated else operationTracker)
            tankVehicleControllerAPI.CreateThrust0Attr(0)
            tankVehicleControllerAPI.CreateThrust1Attr(0)

            vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI(tankVehicleControllerAPI)  # tank controller also applies base controller
        else:
            vehicleControllerAPI = _apply_api_single(vehiclePrim, PhysxSchema.PhysxVehicleControllerAPI, None if vehiclePrimCreated else operationTracker)

        vehicleControllerAPI.CreateAcceleratorAttr(0)
        vehicleControllerAPI.CreateBrake0Attr(0)
        vehicleControllerAPI.CreateBrake1Attr(0)
        vehicleControllerAPI.CreateSteerAttr(0)
        if vehicleData.driveTypeIndex == DRIVE_TYPE_STANDARD:
            vehicleControllerAPI.CreateTargetGearAttr(VEHICLE_AUTOMATIC_TRANSMISSION_GEAR_VALUE)
        else:
            vehicleControllerAPI.CreateTargetGearAttr(1)

    # chassis (collision)
    if (not vehicleData.vehiclePath):
        # creating or setting collision shapes is not supported by the wizard if the user defines the
        # vehicle prim (lot of stuff that would have to be taken care of: path needs to be unique,
        # case where the collision prim was generated already, scale of attachment prim to
        # consider etc. etc.
        chassisHalfExtents = 0.5 * Gf.Vec3f(vehicleMassBoxDim[0], vehicleMassBoxDim[1], vehicleMassBoxDim[2])
        chassisOffset = Gf.Vec3f(0, 0, 0) * vehicleData.unitScale.lengthScale
        vehicleChassisPath = vehiclePath + "/ChassisCollision"
        (vehicleChassis, _) = _get_or_create_prim(stage, vehicleChassisPath,
            UsdGeom.Cube, operationTracker)
        vehicleChassis.CreatePurposeAttr(UsdGeom.Tokens.guide)
        vehicleChassis.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(chassisOffset)
        vehicleChassis.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(chassisHalfExtents)

        vehicleChassisPrim = vehicleChassis.GetPrim()

        UsdPhysics.CollisionAPI.Apply(vehicleChassisPrim)

        vehicleChassisPrimPath = vehicleChassisPrim.GetPrimPath()

        if (not is_in_collision_group(stage, vehicleChassisPrimPath, collisionGroupVehicleChassisPath)):
            add_collision_to_collision_group(stage, vehicleChassisPrimPath, collisionGroupVehicleChassisPath)
            if (operationTracker):
                operationTracker.collisionGroupIncludeList.append(CollisionGroupIncludeData(vehicleChassisPrimPath, collisionGroupVehicleChassisPath))

        physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(vehicleChassisPrim)
        physxCollisionAPI.CreateRestOffsetAttr(0.0 * vehicleData.unitScale.lengthScale)
        physxCollisionAPI.CreateContactOffsetAttr(contactOffset)

        # chassis (render)
        vehicleRenderChassisPath = vehiclePath + "/ChassisRender"
        (vehicleRenderChassis, _) = _get_or_create_prim(stage, vehicleRenderChassisPath,
            UsdGeom.Mesh, operationTracker)
        vehicleRenderChassis.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(chassisOffset)
        vehicleRenderChassis.CreateDisplayColorAttr([Gf.Vec3f(0.2784314, 0.64705884, 1)])

        faceVertexCounts = [4, 4, 4, 4, 4, 4]

        faceVertexIndices = [0, 1, 3, 2, 4, 5, 7, 6, 10, 11, 13, 12, 14, 15, 9, 8, 17, 23, 21, 19, 22, 16, 18, 20]

        normals = [
            verticalAxisVector,
            verticalAxisVector,
            verticalAxisVector,
            verticalAxisVector,
            -verticalAxisVector,
            -verticalAxisVector,
            -verticalAxisVector,
            -verticalAxisVector,
            -longitudinalAxisVector,
            -longitudinalAxisVector,
            longitudinalAxisVector,
            longitudinalAxisVector,
            longitudinalAxisVector,
            longitudinalAxisVector,
            -longitudinalAxisVector,
            -longitudinalAxisVector,
            lateralAxisVector,
            -lateralAxisVector,
            lateralAxisVector,
            -lateralAxisVector,
            lateralAxisVector,
            -lateralAxisVector,
            lateralAxisVector,
            -lateralAxisVector,
        ]

        # chassisHalfExtents[0] = 0.7 * vehicleData.unitScale.lengthScale  # reduced width to make the wheels easily visible
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

        vehicleRenderChassis.CreateFaceVertexCountsAttr(faceVertexCounts)
        vehicleRenderChassis.CreateFaceVertexIndicesAttr(faceVertexIndices)
        vehicleRenderChassis.CreateNormalsAttr(normals)
        vehicleRenderChassis.CreatePointsAttr(points)

    return (messageList, operationTracker)
