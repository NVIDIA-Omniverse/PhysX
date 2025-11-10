# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import enum
import math
import os

import OmniAudioSchema as AudioSchema
from pxr import PhysxSchema

import omni.kit.app
import omni.physxdemos as demo
import omni.timeline
import omni.usd
from omni.physx.bindings._physx import (
    VEHICLE_DRIVE_STATE_ENGINE_ROTATION_SPEED,
    VEHICLE_WHEEL_STATE_ROTATION_SPEED, VEHICLE_WHEEL_STATE_TIRE_LATERAL_SLIP,
    VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_SLIP)

from . import BasicSetup
from .VehicleSampleBase import VehicleSampleBase

from carb.eventdispatcher import get_eventdispatcher

CMPS_TO_MPH = 0.0223694
RPS_TO_RPM = 30.0 / 3.1415

NOTE0 = 13.6
NOTE1 = 0.0066
NOTE2 = -2.95e-7

GAUSSIAN_A = 0.8
GAUSSIAN_B = 0.0
GAUSSIAN_C = 666.0


class VehicleSoundDemo(VehicleSampleBase):
    title = "Vehicle sounds"
    category = demo.Categories.VEHICLES
    short_description = "Usage of vehicle telemetry to emit sounds"
    description = ("Demo of engine, tire roll and skidding audio and how to connect it to the vehicle telemetry. The arrow keys can be used to steer, accelerate and brake. "
        "To use a gamepad for controlling the vehicle, make sure to disable Gamepad Camera Control in the Viewport Settings.")

    def create(self, stage):
        super().create(stage)

        self._physxInterface = omni.physx.get_physx_interface()
        BasicSetup.create(stage, True)
        create(stage, self._physxInterface)

        self.autofocus = True # autofocus on the scene at first update
        self.autofocus_zoom = 0.28 # Get a bit closer

    def on_shutdown(self):
        self._physxInterface = None


class SoundType(enum.Enum):
    engine = 1
    tire = 2
    skid = 3


class VehicleSound:
    def __init__(self, vehiclePrim, soundPrim, soundType, lowLoad, rpm, speed):
        self._vehicle = vehiclePrim
        self._sound = soundPrim
        self._soundType = soundType
        self._lowLoad = lowLoad
        self._rpm = rpm
        self._speed = speed


class LowPassFilter:
    def __init__(self):
        self._timeConstant = 1.0
        self._oneOverTimeConstant = 0.0
        self._value = 0.0

    def setTimeConstant(self, timeConstant):
        if (timeConstant > 0.0):
            self._timeConstant = timeConstant
            self._oneOverTimeConstant = 1.0 / timeConstant

    def getValue(self):
        return self._value

    def filter(self, value, timeStep):
        if (timeStep < self._timeConstant):
            k = timeStep * self._oneOverTimeConstant
            self._value = k * value + (1.0 - k) * self._value
        else:
            self._value = value

        return self._value


def Lerp(x0, y0, x1, y1, x):
    t = (x - x0) / (x1 - x0)
    t = min(max(t, 0.0), 1.0)

    return y0 + t * (y1 - y0)


def gaussian(a, b, c, x):
    numerator = -(x - b) * (x - b)
    denominator = 2.0 * c * c
    gauss = a * math.exp(numerator / denominator)
    return gauss


class VehicleAudioSample:

    def __init__(self, stage, physxInterface):
        self._physxInterface = physxInterface
        self._stage = stage

        # the update loop callback seems to get triggered before the vehicle update and thus the vehicles have not
        # been set up yet. Unfortunately, there seems no way to describe depenencies on other tasks to enforce
        # an order. Hence, just wait for one update to pass before starting.
        #
        self._firstUpdatePassed = False

        self._slipFilter = LowPassFilter()
        self._slipFilter.setTimeConstant(0.1)

        self._soundList = []

        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline_event_sub = self._timeline.get_timeline_event_stream().create_subscription_to_pop(self._on_timeline_event)

        self._usd_context = omni.usd.get_context()

        self._stageEventSubscription = get_eventdispatcher().observe_event(
            observer_name="omni.physx.vehicle:VehicleAudioSample",
            event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.CLOSING),
            on_event=lambda _: self.shutdown()
        )

        vehiclePrim = None
        self._maxRPM = 0
        self._minRPM = 99999

        for prim in self._stage.Traverse():
            # print(dir(prim))

            if (prim.HasAPI(PhysxSchema.PhysxVehicleAPI)):
                vehiclePrim = prim

            elif (prim.IsA(AudioSchema.Sound)):

                # Initialize all of the sounds.
                # Silent and looping.
                prim.GetAttribute("timeScale").Set(1.0)
                prim.GetAttribute("gain").Set(0.0)
                prim.GetAttribute("loopCount").Set(-1)

                primPath = str(prim.GetPath())

                rpm = 0
                speed = 0
                lowLoad = False

                lowLoadIndex = primPath.find("loww")
                highLoadIndex = primPath.find("high")
                tireIndex = primPath.find("driveon")
                skidIndex = primPath.find("tireslip")

                if (lowLoadIndex != -1):
                    soundType = SoundType.engine
                    lowLoad = True
                    rpm = int(primPath[lowLoadIndex + 4:])
                    self._maxRPM = max(self._maxRPM, rpm)
                    self._minRPM = min(self._minRPM, rpm)

                elif (highLoadIndex != -1):
                    soundType = SoundType.engine
                    rpm = int(primPath[highLoadIndex + 4:])

                elif (tireIndex != -1):
                    soundType = SoundType.tire
                    speed = int(primPath[tireIndex + 7:])

                elif (skidIndex != -1):
                    soundType = SoundType.skid

                newSound = VehicleSound(vehiclePrim, prim, soundType, lowLoad, rpm, speed)
                self._soundList.append(newSound)

    def _on_timeline_event(self, e):
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            self._firstUpdatePassed = False
        if e.type == int(omni.timeline.TimelineEventType.CURRENT_TIME_TICKED):
            self.update(e)

    def shutdown(self):
        self._slipFilter = None
        self._soundList = None
        self._timeline_event_sub = None
        self._stageEventSubscription = None

    def update(self, e):
        if (self._timeline.is_playing()):
            if (self._firstUpdatePassed):
                deltaTime = e.payload["dt"]

                vehiclePrim = None
                throttle = None
                wheelState = None

                tireRadius = 35.0

                for vehicleSound in self._soundList:
                    gain = 0.0
                    timeScale = 1.0

                    if (vehiclePrim != vehicleSound._vehicle):
                        vehiclePrim = vehicleSound._vehicle

                        throttle = vehiclePrim.GetAttribute("physxVehicleController:accelerator").Get()

                        rearLeftWheelPath = vehiclePrim.GetPath().pathString + "/RearLeftWheel"

                        wheelState = self._physxInterface.get_wheel_state(rearLeftWheelPath)
                        driveState = self._physxInterface.get_vehicle_drive_state(vehiclePrim.GetPath().pathString)

                    longitudinalSlip = math.fabs(wheelState[VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_SLIP])
                    lateralSlip = math.fabs(wheelState[VEHICLE_WHEEL_STATE_TIRE_LATERAL_SLIP])
                    slip = max(longitudinalSlip, lateralSlip)
                    slip = min(max(slip, 0.0), 1.0)

                    wheelRotationSpeed = wheelState[VEHICLE_WHEEL_STATE_ROTATION_SPEED]
                    wheelSpeedMPH = tireRadius * math.fabs(wheelRotationSpeed) * CMPS_TO_MPH

                    speedMPH = wheelSpeedMPH

                    rpm = driveState[VEHICLE_DRIVE_STATE_ENGINE_ROTATION_SPEED] * RPS_TO_RPM
                    rpm = min(max(rpm, self._minRPM), self._maxRPM)

                    currentNote = rpm * rpm * NOTE2 + rpm * NOTE1 + NOTE0

                    if (vehicleSound._soundType == SoundType.engine):
                        # Engine sounds
                        loadGain = throttle

                        if (vehicleSound._lowLoad):
                            loadGain = 1 - throttle

                        deltaRPM = rpm - vehicleSound._rpm
                        rpmGain = gaussian(GAUSSIAN_A, GAUSSIAN_B, GAUSSIAN_C, deltaRPM)

                        gain = loadGain * rpmGain

                        soundNote = vehicleSound._rpm * vehicleSound._rpm * NOTE2 + vehicleSound._rpm * NOTE1 + NOTE0
                        semitoneDelta = currentNote - soundNote
                        timeScale = math.pow(2.0, semitoneDelta / 12.0)

                    elif (vehicleSound._soundType == SoundType.tire):
                        # Tire rolling sounds
                        if (vehicleSound._speed == 10.0):
                            if (speedMPH < 10.0):
                                gain = Lerp(0.0, 0.0, 10.0, 0.7, speedMPH)
                                semitoneDelta = Lerp(0.0, -2.0, 10.0, 0.0, speedMPH)
                            else:
                                gain = Lerp(10.0, 0.7, 20.0, 0.3, speedMPH)
                                semitoneDelta = Lerp(10.0, 0.0, 20.0, 2.0, speedMPH)

                        elif (vehicleSound._speed == 20.0):
                            if (speedMPH < 20.0):
                                gain = Lerp(10.0, 0.0, 20.0, 0.7, speedMPH)
                                semitoneDelta = Lerp(10.0, -2.0, 20.0, 0.0, speedMPH)
                            else:
                                gain = Lerp(20.0, 0.7, 60.0, 0.35, speedMPH)
                                semitoneDelta = Lerp(20.0, 0.0, 60.0, 3.0, speedMPH)

                        elif (vehicleSound._speed == 60.0):
                            gain = Lerp(0.0, 0.0, 30.0, 1.0, speedMPH)

                            if (speedMPH < 60.0):
                                semitoneDelta = Lerp(10.0, -2.0, 60.0, 0.0, speedMPH)
                            else:
                                semitoneDelta = Lerp(60.0, 0.0, 150.0, 2.0, speedMPH)

                        timeScale = math.pow(2.0, semitoneDelta / 12.0)

                    elif (vehicleSound._soundType == SoundType.skid):
                        # Tire skidding sounds
                        gain = self._slipFilter.filter(slip, deltaTime)

                    vehicleSound._sound.GetAttribute("gain").Set(gain)
                    vehicleSound._sound.GetAttribute("timeScale").Set(timeScale)
            else:
                self._firstUpdatePassed = True
        else:
            self._firstUpdatePassed = False


def _create_audio_prim(stage, audioFolder, rootPath, filename):
    sound = AudioSchema.Sound.Define(stage, rootPath + "/Vehicle/Audio/" + filename)
    sound.CreateFilePathAttr().Set(audioFolder + "/" + filename + ".wav")


def create(stage, physxInterface):
    # print(dir(AudioSchema.Sound))

    data_path = "../../../../../data/audio"

    audio_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, data_path)))
    audio_folder = audio_folder.replace("\\", "/")
    # print(audio_folder)

    rootPath = str(stage.GetDefaultPrim().GetPath())

    # Engine sounds
    _create_audio_prim(stage, audio_folder, rootPath, "loww1000")
    _create_audio_prim(stage, audio_folder, rootPath, "loww1500")
    _create_audio_prim(stage, audio_folder, rootPath, "loww2000")
    _create_audio_prim(stage, audio_folder, rootPath, "loww2500")
    _create_audio_prim(stage, audio_folder, rootPath, "loww3000")
    _create_audio_prim(stage, audio_folder, rootPath, "loww3500")
    _create_audio_prim(stage, audio_folder, rootPath, "loww4000")
    _create_audio_prim(stage, audio_folder, rootPath, "loww4500")
    _create_audio_prim(stage, audio_folder, rootPath, "loww5000")
    _create_audio_prim(stage, audio_folder, rootPath, "loww5500")
    _create_audio_prim(stage, audio_folder, rootPath, "loww6000")
    _create_audio_prim(stage, audio_folder, rootPath, "loww6500")

    _create_audio_prim(stage, audio_folder, rootPath, "high1000")
    _create_audio_prim(stage, audio_folder, rootPath, "high1500")
    _create_audio_prim(stage, audio_folder, rootPath, "high2000")
    _create_audio_prim(stage, audio_folder, rootPath, "high2500")
    _create_audio_prim(stage, audio_folder, rootPath, "high3000")
    _create_audio_prim(stage, audio_folder, rootPath, "high3500")
    _create_audio_prim(stage, audio_folder, rootPath, "high4000")
    _create_audio_prim(stage, audio_folder, rootPath, "high4500")
    _create_audio_prim(stage, audio_folder, rootPath, "high5000")
    _create_audio_prim(stage, audio_folder, rootPath, "high5500")
    _create_audio_prim(stage, audio_folder, rootPath, "high6000")
    _create_audio_prim(stage, audio_folder, rootPath, "high6500")

    # Skid sounds
    _create_audio_prim(stage, audio_folder, rootPath, "tireslip")

    # Drive on sounds
    _create_audio_prim(stage, audio_folder, rootPath, "driveon10")
    _create_audio_prim(stage, audio_folder, rootPath, "driveon20")
    _create_audio_prim(stage, audio_folder, rootPath, "driveon60")

    vehicleAudio = VehicleAudioSample(stage, physxInterface)
