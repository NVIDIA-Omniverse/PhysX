# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import enum
import math

import carb
import OmniAudioSchema as AudioSchema
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from omni.physx.scripts.assets_paths import AssetFolders
import omni.kit.viewport.utility as vp_utils
import omni.physxdemos as demo
import omni.timeline

DRIVE_DAMPING = 100000.0
DRIVE_STIFFNESS = 0.0
MAX_DRIVE_VELOCITY = -2000.0

STICTION_DAMPING = 100000000.0

BRAKE_DAMPING = 1000.0
BRAKE_STIFFNESS = 0.0

REVERSE_SPEED = 10.0
REVERSE_TIMER = 0.1

STEER_GAIN = -10.0

AUDIO_RPM_SCALE = 20.0

CMPS_TO_MPH = 0.0223694
RPS_TO_RPM = 30.0 / 3.1415

NOTE0 = 13.6
NOTE1 = 0.0066
NOTE2 = -2.95e-7

MIN_GAIN_CHANGE = 0.1
MIN_TIME_SCALE_CHANGE = 0.05

GAUSSIAN_A = 0.8
GAUSSIAN_B = 0.0
GAUSSIAN_C = 666.0

PI = 3.14159265


class SoundType(enum.Enum):
    engine = 1
    tire = 2
    skid = 3


class VehicleSound:
    def __init__(self, soundPrim, soundType, lowLoad, rpm, speed):
        self._sound = soundPrim
        self._soundType = soundType
        self._lowLoad = lowLoad
        self._rpm = rpm
        self._speed = speed
        self._gain = 0.0
        self._timeScale = 1.0


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


class LegoTechnicBuggy(demo.Base):
    title = "Lego Technic Buggy"
    category = demo.Categories.COMPLEX_SHOWCASES
    short_description = "A Lego Technic Buggy composed of joints and rigid bodies, including audio."
    description = """A gamepad is needed to control the car.
                  \n\nPress 1 to hide/show the chassis.
                  \nPress 2 to switch between camera views.
                  \n\nOriginal model by user Batka on Mecabricks.com."""

    kit_settings = {
        "/persistent/app/omniverse/gamepadCameraControl": False,  # Turn off the gamepad camera control.
                                                  # audio   |   meshes  | gpu mem   | fps
        "persistent/app/viewport/displayOptions": (1 << 12) | (1 << 10) | (1 << 13) | 1
    }

    def __init__(self):
        self.demo_base_usd_url = demo.get_demo_asset_path(AssetFolders.LEGO_BUGGY, "LegoTechnicBuggy_Springs.usd")
        self.audioPath = demo.get_demo_asset_path(AssetFolders.LEGO_BUGGY, "audio")

    def setup_cameras(self):
        self._cameras_enabled = demo.utils.enable_extension_with_check("omni.physx.camera")

    def create(self, stage):
        self.setup_cameras()

        self._stage = stage
        self.defaultPrimPath = str(stage.GetDefaultPrim().GetPath())
        self.globalTime = 0.0
        self.currentCamera = -1

        self.steerLeft = 0.0
        self.steerRight = 0.0
        self.steerAngle = 0.0
        self.throttle = 0.0
        self.brake = 0.0

        self.reverse = False
        self.reverseTimer = 0.0

        # Hide/show prims
        self._chassis = stage.GetPrimAtPath(self.defaultPrimPath + "/Chassis_1")
        self._cockpit = stage.GetPrimAtPath(self.defaultPrimPath + "/Cockpit_36")
        self._engine = stage.GetPrimAtPath(self.defaultPrimPath + "/Engine_block_33")

        # Physics prims
        self._rear_left_drive = stage.GetPrimAtPath(self.defaultPrimPath + "/Free_rear_wheel_hinge_61")
        self._rear_right_drive = stage.GetPrimAtPath(self.defaultPrimPath + "/Connected_rear_wheel_hinge_62")
        self._front_left_drive = stage.GetPrimAtPath(self.defaultPrimPath + "/Front_wheel_hinge_8")
        self._front_right_drive = stage.GetPrimAtPath(self.defaultPrimPath + "/Front_wheel_hinge_9")

        self._steering_wheel = stage.GetPrimAtPath(self.defaultPrimPath + "/Steering_wheel_19")
        self._steering_wheel_hinge = stage.GetPrimAtPath(self.defaultPrimPath + "/Steering_wheel_hinge_40")

        self._steering_rack_d6 = stage.GetPrimAtPath(self.defaultPrimPath + "/Rack_prismatic_28")

        self._front_left_tire = stage.GetPrimAtPath(self.defaultPrimPath + "/Front_wheel_8")
        self._front_right_tire = stage.GetPrimAtPath(self.defaultPrimPath + "/Front_wheel_9")
        self._rear_left_tire = stage.GetPrimAtPath(self.defaultPrimPath + "/Free_rear_wheel_hinge_32")
        self._rear_right_tire = stage.GetPrimAtPath(self.defaultPrimPath + "/Connected_rear_wheel_31")

        self._front_left_spring_bottom = stage.GetPrimAtPath(self.defaultPrimPath + "/_76138c_json_mb_194_mesh_10")
        self._front_left_spring_top = stage.GetPrimAtPath(self.defaultPrimPath + "/Front_suspension_spring_12")
        self._front_left_spring = stage.GetPrimAtPath(self.defaultPrimPath + "/Front_suspension_spring_12/Shape0")

        self._front_right_spring_bottom = stage.GetPrimAtPath(self.defaultPrimPath + "/_76138c_json_mb_194_mesh_13")
        self._front_right_spring_top = stage.GetPrimAtPath(self.defaultPrimPath + "/Front_suspension_spring_11")
        self._front_right_spring = stage.GetPrimAtPath(self.defaultPrimPath + "/Front_suspension_spring_11/Shape0")

        self._rear_spring_bottom = stage.GetPrimAtPath(self.defaultPrimPath + "/Rear_suspension_spring_34")
        self._rear_spring_top = stage.GetPrimAtPath(self.defaultPrimPath + "/_76138c_json_mb_194_mesh_35")
        self._rear_spring = stage.GetPrimAtPath(self.defaultPrimPath + "/Rear_suspension_spring_34/Shape1")

        # Audio prims
        self._drive_shaft = stage.GetPrimAtPath(self.defaultPrimPath + "/Central_rod_18")

        self._visible = True
        self._rear_wheel = stage.GetPrimAtPath(self.defaultPrimPath + "/Engine_block_33")

        # Camera prims
        self._cameras = []
        self._cameras.append(self.defaultPrimPath + "/FollowCamera")
        self._cameras.append(self.defaultPrimPath + "/DroneCamera")
        self._cameras.append(self.defaultPrimPath + "/Chassis_1/CockpitCamera")

        # initialize the keyboard and gamepad
        appwindow = omni.appwindow.get_default_app_window()
        self._keyboard = appwindow.get_keyboard()
        self._gamepad = appwindow.get_gamepad(0)

        input = carb.input.acquire_input_interface()
        self._keyboardSubId = input.subscribe_to_keyboard_events(self._keyboard, self.on_input)
        self._gamepadSubId = input.subscribe_to_gamepad_events(self._gamepad, self.on_gamepad)

        # load all of the audio
        self.load_audio()

        # initialize the spring lenghts
        flXformBottom = UsdGeom.Xformable(self._front_left_spring_bottom)
        flBottom = flXformBottom.ComputeLocalToWorldTransform(0).ExtractTranslation()
        flXformTop = UsdGeom.Xformable(self._front_left_spring_top)
        flTop = flXformTop.ComputeLocalToWorldTransform(0).ExtractTranslation()
        self._flDistance = (flBottom - flTop).GetLength()

        frXformBottom = UsdGeom.Xformable(self._front_right_spring_bottom)
        frBottom = frXformBottom.ComputeLocalToWorldTransform(0).ExtractTranslation()
        frXformTop = UsdGeom.Xformable(self._front_right_spring_top)
        frTop = frXformTop.ComputeLocalToWorldTransform(0).ExtractTranslation()
        self._frDistance = (frBottom - frTop).GetLength()

        rXformBottom = UsdGeom.Xformable(self._rear_spring_bottom)
        rBottom = rXformBottom.ComputeLocalToWorldTransform(0).ExtractTranslation()
        rXformTop = UsdGeom.Xformable(self._rear_spring_top)
        rTop = rXformTop.ComputeLocalToWorldTransform(0).ExtractTranslation()
        self._rDistance = (rBottom - rTop).GetLength()

    def on_shutdown(self):
        input = carb.input.acquire_input_interface()
        input.unsubscribe_to_keyboard_events(self._keyboard, self._keyboardSubId)
        input.unsubscribe_to_gamepad_events(self._gamepad, self._gamepadSubId)

    def load_audio(self):
        # Engine sounds
        primPath = str(self._drive_shaft.GetPath())

        loww1000 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/loww1000")
        loww1000.CreateFilePathAttr().Set(self.audioPath + "/loww1000.wav")
        loww1500 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/loww1500")
        loww1500.CreateFilePathAttr().Set(self.audioPath + "/loww1500.wav")
        loww2000 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/loww2000")
        loww2000.CreateFilePathAttr().Set(self.audioPath + "/loww2000.wav")
        loww2500 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/loww2500")
        loww2500.CreateFilePathAttr().Set(self.audioPath + "/loww2500.wav")
        loww3000 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/loww3000")
        loww3000.CreateFilePathAttr().Set(self.audioPath + "/loww3000.wav")
        loww3500 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/loww3500")
        loww3500.CreateFilePathAttr().Set(self.audioPath + "/loww3500.wav")
        loww4000 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/loww4000")
        loww4000.CreateFilePathAttr().Set(self.audioPath + "/loww4000.wav")
        loww4500 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/loww4500")
        loww4500.CreateFilePathAttr().Set(self.audioPath + "/loww4500.wav")
        loww5000 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/loww5000")
        loww5000.CreateFilePathAttr().Set(self.audioPath + "/loww5000.wav")
        loww5500 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/loww5500")
        loww5500.CreateFilePathAttr().Set(self.audioPath + "/loww5500.wav")
        loww6000 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/loww6000")
        loww6000.CreateFilePathAttr().Set(self.audioPath + "/loww6000.wav")
        loww6500 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/loww6500")
        loww6500.CreateFilePathAttr().Set(self.audioPath + "/loww6500.wav")

        high1000 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/high1000")
        high1000.CreateFilePathAttr().Set(self.audioPath + "/high1000.wav")
        high1500 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/high1500")
        high1500.CreateFilePathAttr().Set(self.audioPath + "/high1500.wav")
        high2000 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/high2000")
        high2000.CreateFilePathAttr().Set(self.audioPath + "/high2000.wav")
        high2500 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/high2500")
        high2500.CreateFilePathAttr().Set(self.audioPath + "/high2500.wav")
        high3000 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/high3000")
        high3000.CreateFilePathAttr().Set(self.audioPath + "/high3000.wav")
        high3500 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/high3500")
        high3500.CreateFilePathAttr().Set(self.audioPath + "/high3500.wav")
        high4000 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/high4000")
        high4000.CreateFilePathAttr().Set(self.audioPath + "/high4000.wav")
        high4500 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/high4500")
        high4500.CreateFilePathAttr().Set(self.audioPath + "/high4500.wav")
        high5000 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/high5000")
        high5000.CreateFilePathAttr().Set(self.audioPath + "/high5000.wav")
        high5500 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/high5500")
        high5500.CreateFilePathAttr().Set(self.audioPath + "/high5500.wav")
        high6000 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/high6000")
        high6000.CreateFilePathAttr().Set(self.audioPath + "/high6000.wav")
        high6500 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/high6500")
        high6500.CreateFilePathAttr().Set(self.audioPath + "/high6500.wav")

        # Skid sounds
        tireslip = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/tireslip")
        tireslip.CreateFilePathAttr().Set(self.audioPath + "/tireslip.wav")

        # Drive on sounds
        driveon10 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/driveon10")
        driveon10.CreateFilePathAttr().Set(self.audioPath + "/driveon10.wav")
        driveon20 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/driveon20")
        driveon20.CreateFilePathAttr().Set(self.audioPath + "/driveon20.wav")
        driveon60 = AudioSchema.Sound.Define(self._stage, primPath + "/Audio/driveon60")
        driveon60.CreateFilePathAttr().Set(self.audioPath + "/driveon60.wav")

        self._soundList = []

        self._maxRPM = 0
        self._minRPM = 99999

        self._slipFilter = LowPassFilter()
        self._slipFilter.setTimeConstant(0.1)

        for prim in self._stage.Traverse():
            if (prim.IsA(AudioSchema.Sound)):

                # Initialize all of the sounds.
                # Silent and looping.
                prim.GetAttribute("timeScale").Set(1.0)
                prim.GetAttribute("gain").Set(0.0)
                prim.GetAttribute("loopCount").Set(-1)
                prim.GetAttribute("attenuationRange").Set(Gf.Vec2d(0.0, 10000.0))

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

                newSound = VehicleSound(prim, soundType, lowLoad, rpm, speed)
                self._soundList.append(newSound)

    def hide_objects(self):
        imagable = UsdGeom.Imageable(self._chassis)
        imagable.MakeInvisible()
        imagable2 = UsdGeom.Imageable(self._cockpit)
        imagable2.MakeInvisible()
        imagable3 = UsdGeom.Imageable(self._engine)
        imagable3.MakeInvisible()
        self._visible = False

    def show_objects(self):
        imagable = UsdGeom.Imageable(self._chassis)
        imagable.MakeVisible()
        imagable2 = UsdGeom.Imageable(self._cockpit)
        imagable2.MakeVisible()
        imagable3 = UsdGeom.Imageable(self._engine)
        imagable3.MakeVisible()
        self._visible = True

    def update(self, stage, dt, viewport, physxIFace):
        self._stage = stage

        if omni.timeline.get_timeline_interface().is_playing():
            self.globalTime += dt

            rigidBody = UsdPhysics.RigidBodyAPI(self._chassis)
            self.velocity = rigidBody.GetVelocityAttr().Get()
            self.speed = self.velocity.GetLength()

            xform = UsdGeom.Xformable(self._chassis)
            localRotation = xform.GetLocalTransformation().ExtractRotation().GetInverse()
            self.bodyVelocity = localRotation.TransformDir(self.velocity)

            self.update_steer_angle(dt)
            self.update_drive(dt)
            self.update_springs(True)
            self.update_audio(dt)

    def update_steer_angle(self, dt):
        if self.steerLeft >= self.steerRight:
            self.steerAngle = -self.steerLeft
        else:
            self.steerAngle = self.steerRight

        steeringRackDrive = UsdPhysics.DriveAPI.Get(self._steering_rack_d6, "transX")

        steeringRackDrive.GetTargetPositionAttr().Set(-self.steerAngle)

    def update_drive(self, dt):
        # Auto reverse
        if self.reverse:
            throttle = self.brake
            brake = self.throttle

            if self.velocity.GetLength() < REVERSE_SPEED and brake > 0.0:
                self.reverseTimer = self.reverseTimer + dt

                if self.reverseTimer > REVERSE_TIMER:
                    #print("DRIVE")
                    self.reverse = False
                    self.reverseTimer = 0.0

        else:
            throttle = self.throttle
            brake = self.brake

            if self.velocity.GetLength() < REVERSE_SPEED and brake > 0.0:
                self.reverseTimer = self.reverseTimer + dt

                if self.reverseTimer > REVERSE_TIMER:
                    #print("REVERSE")
                    self.reverse = True
                    self.reverseTimer = 0.0

        if throttle < 0.05 and brake < 0.05 and self.speed < 10.0:
            rear_right_drive = UsdPhysics.DriveAPI.Get(self._rear_right_drive, "angular")
            rear_right_drive.GetDampingAttr().Set(STICTION_DAMPING)
            rear_right_drive.GetTargetPositionAttr().Set(0.0)
            #rear_right_drive.GetStiffnessAttr().Set(STICTION_DAMPING)
            #rear_right_drive.GetTargetVelocityAttr().Set(0.0)

            rear_left_drive = UsdPhysics.DriveAPI.Get(self._rear_left_drive, "angular")
            rear_left_drive.GetDampingAttr().Set(STICTION_DAMPING)
            rear_left_drive.GetTargetVelocityAttr().Set(0.0)
            #rear_left_drive.GetStiffnessAttr().Set(STICTION_DAMPING)
            #rear_left_drive.GetTargetPositionAttr().Set(0.0)

            front_left_drive = UsdPhysics.DriveAPI.Get(self._front_left_drive, "angular")
            front_left_drive.GetDampingAttr().Set(STICTION_DAMPING)
            #front_left_drive.GetStiffnessAttr().Set(STICTION_DAMPING)

            front_right_drive = UsdPhysics.DriveAPI.Get(self._front_right_drive, "angular")
            front_right_drive.GetDampingAttr().Set(STICTION_DAMPING)
            #front_right_drive.GetStiffnessAttr().Set(STICTION_DAMPING)

        elif throttle > brake:
            driveVelocity = throttle * MAX_DRIVE_VELOCITY

            if self.reverse:
                driveVelocity = driveVelocity * -1.0

            rear_right_drive = UsdPhysics.DriveAPI.Get(self._rear_right_drive, "angular")
            rear_right_drive.GetDampingAttr().Set(DRIVE_DAMPING)
            rear_right_drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
            rear_right_drive.GetTargetVelocityAttr().Set(driveVelocity)

            rear_left_drive = UsdPhysics.DriveAPI.Get(self._rear_left_drive, "angular")
            rear_left_drive.GetDampingAttr().Set(DRIVE_DAMPING)
            rear_left_drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
            rear_left_drive.GetTargetVelocityAttr().Set(-driveVelocity)

            front_left_drive = UsdPhysics.DriveAPI.Get(self._front_left_drive, "angular")
            front_left_drive.GetDampingAttr().Set(0.0)
            front_left_drive.GetStiffnessAttr().Set(0.0)

            front_right_drive = UsdPhysics.DriveAPI.Get(self._front_right_drive, "angular")
            front_right_drive.GetDampingAttr().Set(0.0)
            front_right_drive.GetStiffnessAttr().Set(0.0)
        else:
            brakeDamping = brake * BRAKE_DAMPING

            rear_right_drive = UsdPhysics.DriveAPI.Get(self._rear_right_drive, "angular")
            rear_right_drive.GetDampingAttr().Set(brakeDamping)
            rear_right_drive.GetStiffnessAttr().Set(BRAKE_STIFFNESS)
            rear_right_drive.GetTargetVelocityAttr().Set(0.0)

            rear_left_drive = UsdPhysics.DriveAPI.Get(self._rear_left_drive, "angular")
            rear_left_drive.GetDampingAttr().Set(brakeDamping)
            rear_left_drive.GetStiffnessAttr().Set(BRAKE_STIFFNESS)
            rear_left_drive.GetTargetVelocityAttr().Set(0.0)

            front_left_drive = UsdPhysics.DriveAPI.Get(self._front_left_drive, "angular")
            front_left_drive.GetDampingAttr().Set(0.0)
            front_left_drive.GetStiffnessAttr().Set(0.0)

            front_right_drive = UsdPhysics.DriveAPI.Get(self._front_right_drive, "angular")
            front_right_drive.GetDampingAttr().Set(0.0)
            front_right_drive.GetStiffnessAttr().Set(0.0)

    def update_springs(self, isPlaying):
        flXformBottom = UsdGeom.Xformable(self._front_left_spring_bottom)
        flBottom = flXformBottom.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        flXformTop = UsdGeom.Xformable(self._front_left_spring_top)
        flTop = flXformTop.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        flDistance = (flBottom - flTop).GetLength()
        flScale = min(flDistance/self._flDistance, 1.0)
        self._front_left_spring.GetAttribute("xformOp:scale").Set(Gf.Vec3f(1.0, flScale, 1.0))

        frXformBottom = UsdGeom.Xformable(self._front_right_spring_bottom)
        frBottom = frXformBottom.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        frXformTop = UsdGeom.Xformable(self._front_right_spring_top)
        frTop = frXformTop.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        frDistance = (frBottom - frTop).GetLength()
        frScale = min(frDistance/self._frDistance, 1.0)
        self._front_right_spring.GetAttribute("xformOp:scale").Set(Gf.Vec3f(1.0, frScale, 1.0))

        rXformBottom = UsdGeom.Xformable(self._rear_spring_bottom)
        rBottom = rXformBottom.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        rXformTop = UsdGeom.Xformable(self._rear_spring_top)
        rTop = rXformTop.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        rDistance = (rBottom - rTop).GetLength()
        rScale = max(min(rDistance/self._rDistance, 2.0), 0.5)
        self._rear_spring.GetAttribute("xformOp:scale").Set(Gf.Vec3f(1.0, rScale, 1.0))

    def update_audio(self, deltaTime):
            # Inputs
            rpm = abs(self.bodyVelocity[1]) * AUDIO_RPM_SCALE
            slip = min(max(0.1 * self.bodyVelocity[0], -1.0), 1.0)

            # Equate 50 cm/s to 50 MPH for audio purposes
            speedMPH = self.bodyVelocity[1]

            currentNote = rpm * rpm * NOTE2 + rpm * NOTE1 + NOTE0

            for vehicleSound in self._soundList:
                gain = 0.0
                timeScale = 1.0

                if (vehicleSound._soundType == SoundType.engine):
                    # Engine sounds
                    loadGain = self.throttle

                    if (vehicleSound._lowLoad):
                        loadGain = 1 - self.throttle

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

                # USD writes are expensive so only write when the data changes are large enough
                if abs(gain - vehicleSound._gain) > MIN_GAIN_CHANGE:
                    vehicleSound._sound.GetAttribute("gain").Set(gain)
                    vehicleSound._gain = gain

                if abs(timeScale - vehicleSound._timeScale) > MIN_TIME_SCALE_CHANGE:
                    vehicleSound._sound.GetAttribute("timeScale").Set(timeScale)
                    vehicleSound._timeScale = timeScale

    def switch_camera(self):
        self.currentCamera += 1

        if self.currentCamera >= len(self._cameras):
            self.currentCamera = 0

        vp_utils.get_active_viewport().set_active_camera(self._cameras[self.currentCamera])

    def on_input(self, event):
        #print("{} ({})".format(event.input, event.type))

        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input == carb.input.KeyboardInput.KEY_1:
                if self._visible:
                    self.hide_objects()
                else:
                    self.show_objects()
            elif event.input == carb.input.KeyboardInput.KEY_2:
                self.switch_camera()
        return True

    def on_gamepad(self, event):
        #print("{} ({})".format(event.input, event.value))

        if event.input == carb.input.GamepadInput.RIGHT_TRIGGER:
            self.throttle = event.value

        if event.input == carb.input.GamepadInput.LEFT_TRIGGER:
            self.brake = event.value

        if event.input == carb.input.GamepadInput.LEFT_STICK_LEFT:
            self.steerLeft = event.value

        if event.input == carb.input.GamepadInput.LEFT_STICK_RIGHT:
            self.steerRight = event.value

        return True

