# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ext
from omni.physxvehicletests.bindings import _physxVehicleTests
import omni.physxvehicletests.scripts.tests as tests
from omni.physxtests import import_tests_auto

import_tests_auto("omni.physxvehicletests.scripts", ["tests"])


class PhysxVehicleTestsExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        self._physxVehicleInterface = omni.physxvehicle.get_physx_vehicle_interface()
        tests.setPhysxVehicleInterface(self._physxVehicleInterface)

        self._physxVehicleTestingInterface = _physxVehicleTests.acquire_physx_vehicle_testing_interface()
        tests.setPhysxVehicleTestingInterface(self._physxVehicleTestingInterface)

        self._physxInterface = omni.physx.get_physx_interface()
        tests.setPhysxInterface(self._physxInterface)
        self._physxUnitTestInterface = omni.physx.get_physxunittests_interface()
        tests.setPhysxUnitTestInterface(self._physxUnitTestInterface)
        self._physxSceneQueryInterface = omni.physx.get_physx_scene_query_interface()
        tests.setPhysxSceneQueryInterface(self._physxSceneQueryInterface)
        self._physxSimInterface = omni.physx.get_physx_simulation_interface()
        tests.setPhysxSimInterface(self._physxSimInterface)

    def on_shutdown(self):
        tests.clearPhysxSimInterface()
        self._physxSimInterface = None

        tests.clearPhysxUnitTestInterface()
        self._physxUnitTestInterface = None

        tests.clearPhysxInterface()
        self._physxInterface = None

        tests.clearPhysxSceneQueryInterface()
        self._physxSceneQueryInterface = None

        tests.clearPhysxVehicleInterface()
        self._physxVehicleInterface = None

        tests.clearPhysxVehicleTestingInterface()
        if self._physxVehicleTestingInterface is not None:
            _physxVehicleTests.release_physx_vehicle_testing_interface(self._physxVehicleTestingInterface)
        self._physxVehicleTestingInterface = None
