# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ui
import omni.usd
import carb

from functools import partial

from pxr import UsdGeom
from pxr import PhysxSchema
from pxr import Usd


from .commands import (
    PhysXAddFollowLookCameraCommand,
    PhysXAddFollowVelocityCameraCommand,
    PhysXAddDroneCameraCommand
)

from .propertyWidgets import PropertyWidgetCameraBase, PROPERTY_WIDGET_STYLE
from omni.kit.property.physx.widgets import InvisibleMenuWidgetBase, MainFrameWidget
from omni.kit.property.physx.database import ExtraAddItem


# this widget has no visual, it's here just to fill the add menu with extra items
# refresh the property window on undo/redo of apply/unapply commands
class CameraInvisibleWidget(InvisibleMenuWidgetBase):
    name = "physx_camera_invisible"

    def __init__(self):
        extra_add_items = [
            ExtraAddItem(
                "Follow Look",
                lambda p: p.HasAPI(PhysxSchema.PhysxRigidBodyAPI) or p.IsA(UsdGeom.Xformable),
                lambda p: self._add_follow_look_camera(p),
                lambda p: p.HasAPI(PhysxSchema.PhysxRigidBodyAPI) or p.IsA(UsdGeom.Xformable),
            ),
            ExtraAddItem(
                "Follow Velocity",
                lambda p: p.HasAPI(PhysxSchema.PhysxRigidBodyAPI) or p.IsA(UsdGeom.Xformable),
                lambda p: self._add_follow_velocity_camera(p),
                lambda p: p.HasAPI(PhysxSchema.PhysxRigidBodyAPI) or p.IsA(UsdGeom.Xformable),
            ),
            ExtraAddItem(
                "Drone",
                lambda p: p.HasAPI(PhysxSchema.PhysxRigidBodyAPI) or p.IsA(UsdGeom.Xformable),
                lambda p: self._add_drone_camera(p),
                lambda p: p.HasAPI(PhysxSchema.PhysxRigidBodyAPI) or p.IsA(UsdGeom.Xformable),
            )
        ]

        super().__init__(
            "Physics Camera Invisible",
            "Cameras",
            extras=extra_add_items
        )

    def _create_camera_path(self, subjectPrimPath, postfix):
        stage = omni.usd.get_context().get_stage()
        if (stage):
            cameraPathBase = subjectPrimPath + postfix
            count = 0
            cameraPath = cameraPathBase + str(count)
            while stage.GetPrimAtPath(cameraPath):
                count = count + 1
                cameraPath = cameraPathBase + str(count)

            return cameraPath
        else:
            return None

    def _add_follow_look_camera(self, prim):
        primPath = str(prim.GetPath())
        cameraPath = self._create_camera_path(primPath, "LookFollowCamera")
        PhysXAddFollowLookCameraCommand.execute(primPath, cameraPath)

    def _add_follow_velocity_camera(self, prim):
        primPath = str(prim.GetPath())
        cameraPath = self._create_camera_path(primPath, "VelocityFollowCamera")
        PhysXAddFollowVelocityCameraCommand.execute(primPath, cameraPath)

    def _add_drone_camera(self, prim):
        primPath = str(prim.GetPath())
        cameraPath = self._create_camera_path(primPath, "DroneCamera")
        PhysXAddDroneCameraCommand.execute(primPath, cameraPath)

    def _undo_redo_on_change(self, cmds):
        if any(item in [
            "PhysXAddFollowLookCameraCommand",
            "PhysXAddFollowVelocityCameraCommand",
            "PhysXAddDroneCameraCommand"
        ] for item in cmds):
            MainFrameWidget.instance.request_rebuild()
