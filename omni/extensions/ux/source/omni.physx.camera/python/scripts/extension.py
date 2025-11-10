# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ext
from omni.physx import get_physx_interface, get_physx_simulation_interface
from omni.physxvehicle import get_physx_vehicle_interface
import omni.physxdemos as demo
from .propertyWidgetManager import PropertyWidgetManager
from omni.physx.scripts.utils import safe_import_tests
from .. import get_physx_camera_interface
from ..bindings._physxCamera import release_physx_camera_interface, release_physx_camera_interface_scripting

RUNNING_WTESTS = safe_import_tests("omni.physxcamera.scripts", ["tests"])
DEMO_SCENES = "omni.physxcamera.scripts.samples"


class PhysxCameraExtension(omni.ext.IExt):
    def __init__(self):
        super().__init__()

    def on_startup(self):
        self._physxCameraInterface = get_physx_camera_interface()
        self._cameraPropertyWidgetManager = PropertyWidgetManager(self._physxCameraInterface)
        self._cameraPropertyWidgetManager.set_up()
        self._physxVehicleInterface = get_physx_vehicle_interface()
        self._physxInterface = get_physx_interface()
        self._physxSimInterface = get_physx_simulation_interface()

        if RUNNING_WTESTS:
            from .tests import setPhysxCameraInterface, setPhysxVehicleInterface, setPhysxInterface, setPhysxSimInterface
            setPhysxCameraInterface(self._physxCameraInterface)
            setPhysxVehicleInterface(self._physxVehicleInterface)
            setPhysxInterface(self._physxInterface)
            setPhysxSimInterface(self._physxSimInterface)

        demo.register(DEMO_SCENES)

    def on_shutdown(self):
        if self._cameraPropertyWidgetManager is not None:
            self._cameraPropertyWidgetManager.tear_down()
            self._cameraPropertyWidgetManager = None

        if self._physxCameraInterface is not None:
            if RUNNING_WTESTS:
                from .tests import clearPhysxSimInterface
                clearPhysxSimInterface()
            release_physx_camera_interface(self._physxCameraInterface)
            release_physx_camera_interface_scripting(self._physxCameraInterface)  # OM-60917
            self._physxCameraInterface = None

        if RUNNING_WTESTS:
            from .tests import clearPhysxCameraInterface, clearPhysxVehicleInterface, clearPhysxInterface
            clearPhysxInterface()
            clearPhysxVehicleInterface()
            clearPhysxCameraInterface()

        self._physxInterface = None
        self._physxVehicleInterface = None
        self._physxCameraInterface = None

        demo.unregister(DEMO_SCENES)
