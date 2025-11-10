# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import Gf, PhysxSchema
import omni.physxdemos as demo
from .VehicleSampleBase import VehicleSampleBase
from . import BasicSetup

class NonlinearCommandResponseDemo(VehicleSampleBase):
    title = "Nonlinear command response"
    category = demo.Categories.VEHICLES
    short_description = "Nonlinear command response for basic drive"
    description = ("Demo showing how to set up a drive torque curve when using the basic drive model. "
        "The arrow keys can be used to steer, accelerate and brake. "
        "To use a gamepad for controlling the vehicle, make sure to disable Gamepad Camera Control in the Viewport Settings.")

    def create(self, stage):
        super().create(stage)

        primPaths = BasicSetup.create(stage, False)
        create(stage, primPaths)

        self.autofocus = True # autofocus on the scene at first update
        self.autofocus_zoom = 0.28 # Get a bit closer


def create(stage, primPaths: BasicSetup.PrimPaths):
    vehiclePrim = stage.GetPrimAtPath(primPaths.vehiclePath)

    # the PhysxVehicleNonlinearCommandResponseAPI has to be applied to the prim that has
    # PhysxVehicleDriveBasicAPI applied. In this sample this is just the vehicle prim.
    nonlinCmdRespAPI = PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI.Apply(vehiclePrim, PhysxSchema.Tokens.drive)

    #
    # two graphs are provided. One defines a 0 response, if the command input is 0 (independent of speed).
    # The other graph covers command input 1, has a flat maximum response up to a certain speed, then drops
    # rather steeply and approaches a certain value asymptotically. When driving backwards, a flat maximum
    # response is used up to a certain (absolute) speed and then drops linearly to 0.
    #
    # Note: nonlinear command responses can also be used for the steering and braking systems.
    #

    commandValues = []
    speedResponsesPerCommandValue = []
    speedResponses = []

    # first graph
    commandValues.append(0.0)
    speedResponsesPerCommandValue.append(len(speedResponses))
    speedResponses.extend([Gf.Vec2f(0.0, 0.0)])  # note: 0 is chosen for the speed entry (first entry) but could be any speed in this case

    # second graph
    commandValues.append(1.0)
    speedResponsesPerCommandValue.append(len(speedResponses))
    speedResponses.extend(
        [
            Gf.Vec2f(-6.7, 0.0),
            Gf.Vec2f(-4.5, 1.0),
            Gf.Vec2f(0.0, 1.0),
            Gf.Vec2f(18.0, 1.0),
            Gf.Vec2f(22.0, 0.83),
            Gf.Vec2f(27.0, 0.7),
            Gf.Vec2f(31.0, 0.58),
            Gf.Vec2f(36.0, 0.48),
            Gf.Vec2f(40.0, 0.38),
            Gf.Vec2f(44.0, 0.3),
            Gf.Vec2f(49.0, 0.25),
            Gf.Vec2f(54.0, 0.2),
            Gf.Vec2f(58.0, 0.18),
        ]
    )

    # set the attributes
    nonlinCmdRespAPI.GetCommandValuesAttr().Set(commandValues)
    nonlinCmdRespAPI.GetSpeedResponsesPerCommandValueAttr().Set(speedResponsesPerCommandValue)
    nonlinCmdRespAPI.GetSpeedResponsesAttr().Set(speedResponses)
