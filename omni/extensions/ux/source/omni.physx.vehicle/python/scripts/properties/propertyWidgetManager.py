# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from .propertyWidgetVehicle import (
    PropertyWidgetVehicleControllerSettings,
    PropertyWidgetVehicleAuthoring
)
from .propertyWidgetVehicleWheelAttachment import PropertyWidgetVehicleWheelAttachmentAuthoring
from .propertyWidgetVehicleTireFrictionTable import PropertyWidgetVehicleTireFrictionTable
from .propertyWidgetVehicleTire import PropertyWidgetVehicleTire
from .propertyWidgetVehicleNonlinearCmdResponse import (
    PropertyWidgetVehicleNonlinearCmdResponseDrive,
    PropertyWidgetVehicleNonlinearCmdResponseSteer,
    PropertyWidgetVehicleNonlinearCmdResponseBrakes0,
    PropertyWidgetVehicleNonlinearCmdResponseBrakes1
)


class PropertyWidgetManager:
    def __init__(self, physxVehicleInterface):
        self._physxVehicleInterface = physxVehicleInterface

    def set_up(self):
        self._widgets = [
            PropertyWidgetVehicleControllerSettings(self._physxVehicleInterface),
            PropertyWidgetVehicleAuthoring(),
            PropertyWidgetVehicleWheelAttachmentAuthoring(),
            PropertyWidgetVehicleTireFrictionTable(),
            PropertyWidgetVehicleTire(),
            PropertyWidgetVehicleNonlinearCmdResponseDrive(),
            PropertyWidgetVehicleNonlinearCmdResponseSteer(),
            PropertyWidgetVehicleNonlinearCmdResponseBrakes0(),
            PropertyWidgetVehicleNonlinearCmdResponseBrakes1(),
        ]

        self._register_widgets()

    def tear_down(self):
        self._unregister_widgets()

        self._widgets = None
        self._physxVehicleInterface = None

    def _register_widgets(self):
        from omni.kit.property.physx import register_widget

        for widget in self._widgets:
            register_widget(widget.name, widget)

    def _unregister_widgets(self):
        from omni.kit.property.physx import unregister_widget

        for widget in self._widgets:
            unregister_widget(widget.name)
