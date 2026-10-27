# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from .bindings._physxVehicleTests import acquire_physx_vehicle_testing_interface, IPhysxVehicleTesting

from .ifacecache import cached_interface

def get_physx_vehicle_testing_interface() -> IPhysxVehicleTesting:
    return cached_interface(get_physx_vehicle_testing_interface, acquire_physx_vehicle_testing_interface)

from .scripts.extension import *
