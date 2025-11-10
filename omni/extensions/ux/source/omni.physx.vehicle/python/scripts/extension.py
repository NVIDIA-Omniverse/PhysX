# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ext
import omni.physx
from omni.physxvehicle.bindings import _physxVehicle
from .properties.propertyWidgetManager import PropertyWidgetManager
from .wizards.physxVehicleWizard import PhysXVehicleWizard
import omni.physxdemos as demo
from omni.physxui import PhysicsMenu
from .commands import PhysXVehicleTireFrictionTableAddCommand


DEMO_SCENES = "omni.physxvehicle.scripts.samples"
CREATE_MENU_NAME = "Create"


class PhysxVehicleExtension(omni.ext.IExt):
    def __init__(self):
        super().__init__()
        self._vehiclePropertyWidgetManager = None
        self._vehicleWizard = None

    def on_startup(self):
        self._physxVehicleInterface = _physxVehicle.acquire_physx_vehicle_interface()

        self._vehiclePropertyWidgetManager = PropertyWidgetManager(self._physxVehicleInterface)
        self._vehiclePropertyWidgetManager.set_up()
        self._vehicleWizard = PhysXVehicleWizard()
        self._vehicleWizard.hide()

        # Add menu entry to create tire friction table
        self._createTireFrictionTableMenuItem = {"name": "Tire Friction Table", "onclick_fn": lambda *_: self._on_tire_friction_table_menu_click()}
        PhysicsMenu.add_context_menu(CREATE_MENU_NAME, self._createTireFrictionTableMenuItem)

        demo.register(DEMO_SCENES)

    def on_shutdown(self):
        PhysicsMenu.remove_context_menu(CREATE_MENU_NAME, self._createTireFrictionTableMenuItem)

        if self._vehiclePropertyWidgetManager is not None:
            self._vehiclePropertyWidgetManager.tear_down()
            self._vehiclePropertyWidgetManager = None

        if self._vehicleWizard is not None:
            self._vehicleWizard.on_shutdown()
            self._vehicleWizard = None

        if self._physxVehicleInterface is not None:
            _physxVehicle.release_physx_vehicle_interface(self._physxVehicleInterface)
            _physxVehicle.release_physx_vehicle_interface_scripting(self._physxVehicleInterface) # OM-60917
            self._physxVehicleInterface = None

        demo.unregister(DEMO_SCENES)

    def _on_tire_friction_table_menu_click(self):
        PhysXVehicleTireFrictionTableAddCommand.execute()
