# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.commands
import math
import copy

from omni.usd.commands.usd_commands import DeletePrimsCommand

from pxr import Usd, UsdGeom, UsdShade, Gf, Sdf, UsdPhysics, PhysxSchema
from omni.physxcommands import PhysicsCommand
from omni.physx.scripts.pythonUtils import autoassign


#
# Some general info:
# - closing a stage is supposed to clear the undo stack
# - storing references to prims in commands will not work in most cases since
#   after the command, the referenced prim might get deleted. An undo will 
#   recreate the object but the prim will not be the same. Thus, only path
#   strings seem safe to store.
#


class PhysXAddFollowLookCameraCommand(PhysicsCommand):
    @autoassign
    def __init__(self, subjectPrimPath, cameraPrimPath):
        pass

    def do(self):
        physxCameraInterface = omni.physxcamera.get_physx_camera_interface()
        physxCameraInterface.add_follow_look_camera(self._subjectPrimPath, self._cameraPrimPath)

    def undo(self):
        DeletePrimsCommand([self._cameraPrimPath]).do()


class PhysXAddFollowVelocityCameraCommand(PhysicsCommand):
    @autoassign
    def __init__(self, subjectPrimPath, cameraPrimPath):
        pass

    def do(self):
        physxCameraInterface = omni.physxcamera.get_physx_camera_interface()
        physxCameraInterface.add_follow_velocity_camera(self._subjectPrimPath, self._cameraPrimPath)

    def undo(self):
        DeletePrimsCommand([self._cameraPrimPath]).do()


class PhysXAddDroneCameraCommand(PhysicsCommand):
    @autoassign
    def __init__(self, subjectPrimPath, cameraPrimPath):
        pass

    def do(self):
        physxCameraInterface = omni.physxcamera.get_physx_camera_interface()
        physxCameraInterface.add_drone_camera(self._subjectPrimPath, self._cameraPrimPath)

    def undo(self):
        DeletePrimsCommand([self._cameraPrimPath]).do()


omni.kit.commands.register_all_commands_in_module(__name__)
